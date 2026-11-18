// SPDX-License-Identifier: GPL-2.0-only

/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * Sahara image loader for flashless external modems (SDX55 on the OnePlus 8T).
 * Derived from the qaic Sahara driver: after the MHI core pushes SBL over BHI,
 * the modem asks for the rest of its images by Sahara image ID on the SAHARA
 * channel. Images come from the firmware loader (the stock modem partition,
 * via firmware_class.path) or straight from raw partitions (EFS, DDR training).
 */

#include <linux/devcoredump.h>
#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/limits.h>
#include <linux/mhi.h>
#include <linux/minmax.h>
#include <linux/overflow.h>
#include <linux/types.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>

#include <linux/blkdev.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>

/*
 * Off by default: writing the X55's EFS sync stream back to mdm1m9kefs*
 * changes persistent modem state. Enable with sahara_mdm.efs_writeback=1
 * once the received image set has been verified against the partitions.
 */
static bool efs_writeback;
module_param(efs_writeback, bool, 0644);
MODULE_PARM_DESC(efs_writeback,
		 "Write EFS sync images back to mdm1m9kefs* partitions (default: off)");

#define SAHARA_HELLO_CMD		0x1  /* Min protocol version 1.0 */
#define SAHARA_HELLO_RESP_CMD		0x2  /* Min protocol version 1.0 */
#define SAHARA_READ_DATA_CMD		0x3  /* Min protocol version 1.0 */
#define SAHARA_END_OF_IMAGE_CMD		0x4  /* Min protocol version 1.0 */
#define SAHARA_DONE_CMD			0x5  /* Min protocol version 1.0 */
#define SAHARA_DONE_RESP_CMD		0x6  /* Min protocol version 1.0 */
#define SAHARA_RESET_CMD		0x7  /* Min protocol version 1.0 */
#define SAHARA_RESET_RESP_CMD		0x8  /* Min protocol version 1.0 */
#define SAHARA_MEM_DEBUG_CMD		0x9  /* Min protocol version 2.0 */
#define SAHARA_MEM_READ_CMD		0xa  /* Min protocol version 2.0 */
#define SAHARA_CMD_READY_CMD		0xb  /* Min protocol version 2.1 */
#define SAHARA_SWITCH_MODE_CMD		0xc  /* Min protocol version 2.1 */
#define SAHARA_EXECUTE_CMD		0xd  /* Min protocol version 2.1 */
#define SAHARA_EXECUTE_RESP_CMD		0xe  /* Min protocol version 2.1 */
#define SAHARA_EXECUTE_DATA_CMD		0xf  /* Min protocol version 2.1 */
#define SAHARA_MEM_DEBUG64_CMD		0x10 /* Min protocol version 2.5 */
#define SAHARA_MEM_READ64_CMD		0x11 /* Min protocol version 2.5 */
#define SAHARA_READ_DATA64_CMD		0x12 /* Min protocol version 2.8 */
#define SAHARA_RESET_STATE_CMD		0x13 /* Min protocol version 2.9 */
#define SAHARA_WRITE_DATA_CMD		0x14 /* Min protocol version 3.0 */

/* the SDX55 host side (downstream mhi_uci, mhi,buffer-len) used 32 KiB transfers */
#define SAHARA_PACKET_MAX_SIZE		0x8000U
#define SAHARA_TRANSFER_MAX_SIZE	0x80000
#define SAHARA_READ_MAX_SIZE		0x8000U /* fits one rx buffer */
/*
 * READ_DATA replies are streamed as MHI_CHAIN runs; chained TREs raise no
 * completion until an EOT, so each run of up to this many buffers ends in
 * an EOT (see read_data_helper). The SAHARA UL ring has 128 elements; keep
 * a few for commands.
 */
#define SAHARA_NUM_TX_BUF		120
#define SAHARA_IMAGE_ID_NONE		U32_MAX

#define SAHARA_VERSION			2
#define SAHARA_SUCCESS			0
#define SAHARA_TABLE_ENTRY_STR_LEN	20

#define SAHARA_MODE_IMAGE_TX_PENDING	0x0
#define SAHARA_MODE_IMAGE_TX_COMPLETE	0x1
#define SAHARA_MODE_MEMORY_DEBUG	0x2
#define SAHARA_MODE_COMMAND		0x3

#define SAHARA_HELLO_LENGTH		0x30
#define SAHARA_READ_DATA_LENGTH		0x14
#define SAHARA_END_OF_IMAGE_LENGTH	0x10
#define SAHARA_DONE_LENGTH		0x8
#define SAHARA_RESET_LENGTH		0x8
#define SAHARA_MEM_DEBUG64_LENGTH	0x18
#define SAHARA_MEM_READ64_LENGTH	0x18
#define SAHARA_MEM_DEBUG_LENGTH		0x10
#define SAHARA_MEM_READ_LENGTH		0x10

struct sahara_packet {
	__le32 cmd;
	__le32 length;

	union {
		struct {
			__le32 version;
			__le32 version_compat;
			__le32 max_length;
			__le32 mode;
		} hello;
		struct {
			__le32 version;
			__le32 version_compat;
			__le32 status;
			__le32 mode;
		} hello_resp;
		struct {
			__le32 image;
			__le32 offset;
			__le32 length;
		} read_data;
		struct {
			__le32 image;
			__le32 status;
		} end_of_image;
		struct {
			__le32 table_address;
			__le32 table_length;
		} memory_debug;
		struct {
			__le32 memory_address;
			__le32 memory_length;
		} memory_read;
		struct {
			__le64 table_address;
			__le64 table_length;
		} memory_debug64;
		struct {
			__le64 memory_address;
			__le64 memory_length;
		} memory_read64;
	};
};

struct sahara_debug_table_entry64 {
	__le64	type;
	__le64	address;
	__le64	length;
	char	description[SAHARA_TABLE_ENTRY_STR_LEN];
	char	filename[SAHARA_TABLE_ENTRY_STR_LEN];
};

/* 32-bit memory debug table (SDX55 remote EFS sync uses this form) */
struct sahara_debug_table_entry32 {
	__le32	type;
	__le32	address;
	__le32	length;
	char	description[SAHARA_TABLE_ENTRY_STR_LEN];
	char	filename[SAHARA_TABLE_ENTRY_STR_LEN];
} __packed;

struct sahara_dump_table_entry {
	u64	type;
	u64	address;
	u64	length;
	char	description[SAHARA_TABLE_ENTRY_STR_LEN];
	char	filename[SAHARA_TABLE_ENTRY_STR_LEN];
};

#define SAHARA_DUMP_V1_MAGIC 0x1234567890abcdef
#define SAHARA_DUMP_V1_VER   1
struct sahara_memory_dump_meta_v1 {
	u64	magic;
	u64	version;
	u64	dump_size;
	u64	table_size;
};

/*
 * Layout of crashdump provided to user via devcoredump
 *              +------------------------------------------+
 *              |         Crashdump Meta structure         |
 *              | type: struct sahara_memory_dump_meta_v1  |
 *              +------------------------------------------+
 *              |             Crashdump Table              |
 *              | type: array of struct                    |
 *              |       sahara_dump_table_entry            |
 *              |                                          |
 *              |                                          |
 *              +------------------------------------------+
 *              |                Crashdump                 |
 *              |                                          |
 *              |                                          |
 *              |                                          |
 *              |                                          |
 *              |                                          |
 *              +------------------------------------------+
 *
 * First is the metadata header. Userspace can use the magic number to verify
 * the content type, and then check the version for the rest of the format.
 * New versions should keep the magic number location/value, and version
 * location, but increment the version value.
 *
 * For v1, the metadata lists the size of the entire dump (header + table +
 * dump) and the size of the table. Then the dump image table, which describes
 * the contents of the dump. Finally all the images are listed in order, with
 * no deadspace in between. Userspace can use the sizes listed in the image
 * table to reconstruct the individual images.
 */

struct sahara_context {
	struct sahara_packet		*cmd;
	struct sahara_packet		*tx[SAHARA_NUM_TX_BUF];
	unsigned int			tx_next;
	atomic_t			tx_inflight;
	struct mutex			tx_lock;
	struct sahara_packet		*rx;
	struct work_struct		fw_work;
	struct work_struct		dump_work;
	struct work_struct		read_data_work;
	struct mhi_device		*mhi_dev;
	const char * const		*image_table;
	u32				table_size;
	u32				active_image_id;
	const struct firmware		*firmware;
	void				*part_buf;
	const u8			*img_data;
	size_t				img_size;
	u64				dump_table_address;
	u64				dump_table_length;
	size_t				rx_size;
	size_t				rx_size_requested;
	void				*mem_dump;
	size_t				mem_dump_sz;
	struct sahara_dump_table_entry	*dump_image;
	u64				dump_image_offset;
	void				*mem_dump_freespace;
	u64				dump_images_left;
	u32				read_data_offset;
	u32				read_data_length;
	bool				is_mem_dump_mode;
	/*
	 * Bound to the mission-mode "EFS" channel: the X55 (RMTEFS build)
	 * pushes its EFS back to the host here as a Sahara memory-debug
	 * session (stock: mdm_helper runs ks -m on /dev/mhi_pipe_10, writing
	 * mdm1m9kefs*). For now the data is received and DISCARDED, never
	 * written to the partitions, so a bad transfer can't touch EFS.
	 */
	bool				is_efs;
	/* current memory debug session uses the 32-bit MEM_DEBUG/MEM_READ */
	bool				dump32;
	/* number of entries parsed from the current dump table */
	u32				dump_table_entries;
	/* write EFS sync images back to the mdm1m9kefs* partitions */
	bool				efs_writeback;
};

/*
 * Image ID map from the stock libmdmimgload (SDX55M). Names starting with '@'
 * are raw partitions under /dev/block/bootdevice/by-name/, the rest are looked
 * up by the firmware loader relative to firmware_class.path.
 */
static const char * const sdx55m_image_table[] = {
	[6]  = "sdx55m/apps.mbn",
	[8]  = "sdx55m/qdsp6sw.mbn",
	[16] = "@mdm1m9kefs1",
	[17] = "@mdm1m9kefs2",
	/*
	 * OnePlus swaps ID 20 (stock QC: mdm1m9kefs3) for the OEM RF NV image
	 * (MBN hdr id 0x14 -> 0x80400200: board rf_id + static/dynamic RF NV),
	 * see oemnvbk_add_map_config_v2() in liboemnvbk_img_helper.so. Without
	 * it the X55 comes up with no RF card: zero bands, stuck offline.
	 */
	[20] = "@mdm1oemnvbktmp",
	[21] = "sdx55m/sbl1.mbn",
	[23] = "sdx55m/aop.mbn",
	[25] = "sdx55m/tz.mbn",
	[29] = "sdx55m/acdb.mbn",
	[33] = "sdx55m/hyp.mbn",
	[34] = "@mdmddr",
	[36] = "sdx55m/multi_image_qti.mbn",
	[37] = "sdx55m/multi_image.mbn",
	[38] = "sdx55m/xbl_config.elf",
	[40] = "sdx55m/apdp.mbn",
	[41] = "sdx55m/devcfg.mbn",
	[42] = "sdx55m/sec.elf",
};

static int sahara_load_partition(struct sahara_context *context, const char *name)
{
	struct device *dev = &context->mhi_dev->dev;
	struct file *bdev_file;
	char path[64];
	loff_t pos = 0;
	ssize_t ret;
	u64 size;

	snprintf(path, sizeof(path), "/dev/block/bootdevice/by-name/%s", name);
	bdev_file = bdev_file_open_by_path(path, BLK_OPEN_READ, NULL, NULL);
	if (IS_ERR(bdev_file)) {
		dev_err(dev, "cannot open %s: %pe\n", path, bdev_file);
		return PTR_ERR(bdev_file);
	}

	size = bdev_nr_bytes(file_bdev(bdev_file));
	if (!size || size > SZ_64M) {
		fput(bdev_file);
		return -EINVAL;
	}

	context->part_buf = vmalloc(size);
	if (!context->part_buf) {
		fput(bdev_file);
		return -ENOMEM;
	}

	ret = kernel_read(bdev_file, context->part_buf, size, &pos);
	fput(bdev_file);
	if (ret != size) {
		vfree(context->part_buf);
		context->part_buf = NULL;
		dev_err(dev, "short read of %s: %zd\n", path, ret);
		return ret < 0 ? ret : -EIO;
	}

	context->img_data = context->part_buf;
	context->img_size = size;
	return 0;
}

static int sahara_find_image(struct sahara_context *context, u32 image_id)
{
	int ret;

	if (image_id == context->active_image_id)
		return 0;

	if (context->active_image_id != SAHARA_IMAGE_ID_NONE) {
		dev_err(&context->mhi_dev->dev, "image id %d is not valid as %d is active\n",
			image_id, context->active_image_id);
		return -EINVAL;
	}

	if (image_id >= context->table_size || !context->image_table[image_id]) {
		dev_err(&context->mhi_dev->dev, "request for unknown image: %d\n", image_id);
		return -EINVAL;
	}

	/*
	 * This image might be optional. The device may continue without it.
	 * Only the device knows. Suppress error messages that could suggest an
	 * a problem when we were actually able to continue.
	 */
	if (context->image_table[image_id][0] == '@') {
		ret = sahara_load_partition(context, context->image_table[image_id] + 1);
	} else {
		/* no sysfs fallback: the modem won't wait 60 s for an image */
		ret = request_firmware_direct(&context->firmware,
					      context->image_table[image_id],
					      &context->mhi_dev->dev);
		if (!ret) {
			context->img_data = context->firmware->data;
			context->img_size = context->firmware->size;
		}
	}
	if (ret) {
		dev_warn(&context->mhi_dev->dev, "request for image id %d / file %s failed %d\n",
			image_id, context->image_table[image_id], ret);
		return ret;
	}

	context->active_image_id = image_id;
	dev_info(&context->mhi_dev->dev, "sending image %u (%s, %zu bytes)\n",
		 image_id, context->image_table[image_id], context->img_size);

	return 0;
}

static void sahara_release_image(struct sahara_context *context)
{
	if (context->active_image_id != SAHARA_IMAGE_ID_NONE) {
		release_firmware(context->firmware);
		vfree(context->part_buf);
	}
	context->firmware = NULL;
	context->part_buf = NULL;
	context->img_data = NULL;
	context->img_size = 0;
	context->active_image_id = SAHARA_IMAGE_ID_NONE;
}

static void sahara_send_reset(struct sahara_context *context)
{
	int ret;

	context->is_mem_dump_mode = false;
	context->read_data_offset = 0;
	context->read_data_length = 0;

	context->cmd->cmd = cpu_to_le32(SAHARA_RESET_CMD);
	context->cmd->length = cpu_to_le32(SAHARA_RESET_LENGTH);

	ret = mhi_queue_buf(context->mhi_dev, DMA_TO_DEVICE, context->cmd,
			    SAHARA_RESET_LENGTH, MHI_EOT);
	if (ret)
		dev_err(&context->mhi_dev->dev, "Unable to send reset response %d\n", ret);
}

static void sahara_hello(struct sahara_context *context)
{
	int ret;

	dev_info(&context->mhi_dev->dev,
		"HELLO cmd received. length:%d version:%d version_compat:%d max_length:%d mode:%d\n",
		le32_to_cpu(context->rx->length),
		le32_to_cpu(context->rx->hello.version),
		le32_to_cpu(context->rx->hello.version_compat),
		le32_to_cpu(context->rx->hello.max_length),
		le32_to_cpu(context->rx->hello.mode));

	if (le32_to_cpu(context->rx->length) != SAHARA_HELLO_LENGTH) {
		dev_err(&context->mhi_dev->dev, "Malformed hello packet - length %d\n",
			le32_to_cpu(context->rx->length));
		return;
	}
	if (le32_to_cpu(context->rx->hello.version) < SAHARA_VERSION ||
	    le32_to_cpu(context->rx->hello.version_compat) > SAHARA_VERSION) {
		dev_err(&context->mhi_dev->dev, "Unsupported hello packet - version %d\n",
			le32_to_cpu(context->rx->hello.version));
		return;
	}

	if (le32_to_cpu(context->rx->hello.mode) != SAHARA_MODE_IMAGE_TX_PENDING &&
	    le32_to_cpu(context->rx->hello.mode) != SAHARA_MODE_IMAGE_TX_COMPLETE &&
	    le32_to_cpu(context->rx->hello.mode) != SAHARA_MODE_MEMORY_DEBUG) {
		dev_err(&context->mhi_dev->dev, "Unsupported hello packet - mode %d\n",
			le32_to_cpu(context->rx->hello.mode));
		return;
	}

	context->cmd->cmd = cpu_to_le32(SAHARA_HELLO_RESP_CMD);
	context->cmd->length = cpu_to_le32(SAHARA_HELLO_LENGTH);
	context->cmd->hello_resp.version = cpu_to_le32(SAHARA_VERSION);
	context->cmd->hello_resp.version_compat = cpu_to_le32(SAHARA_VERSION);
	context->cmd->hello_resp.status = cpu_to_le32(SAHARA_SUCCESS);
	context->cmd->hello_resp.mode = context->rx->hello_resp.mode;

	ret = mhi_queue_buf(context->mhi_dev, DMA_TO_DEVICE, context->cmd,
			    SAHARA_HELLO_LENGTH, MHI_EOT);
	if (ret)
		dev_err(&context->mhi_dev->dev, "Unable to send hello response %d\n", ret);
}

/* Queue as many chunks of the current READ_DATA reply as there are free buffers */
static int read_data_helper(struct sahara_context *context)
{
	enum mhi_flags mhi_flag;
	u32 pkt_data_len;
	int ret = 0;
	void *buf;

	mutex_lock(&context->tx_lock);
	while (context->read_data_length &&
	       atomic_read(&context->tx_inflight) < SAHARA_NUM_TX_BUF) {
		pkt_data_len = min(context->read_data_length, SAHARA_PACKET_MAX_SIZE);
		buf = context->tx[context->tx_next];

		memcpy(buf, &context->img_data[context->read_data_offset], pkt_data_len);

		context->read_data_offset += pkt_data_len;
		context->read_data_length -= pkt_data_len;
		/*
		 * As downstream mhi_uci: chain while more buffers can follow
		 * right away, but force EOT on the last one that fits so the
		 * SBL completes it and the refill from ul_xfer_cb continues
		 * the stream (qdsp6sw asks for ~84 MB in one READ_DATA).
		 */
		if (context->read_data_length &&
		    atomic_read(&context->tx_inflight) + 1 < SAHARA_NUM_TX_BUF)
			mhi_flag = MHI_CHAIN;
		else
			mhi_flag = MHI_EOT;

		atomic_inc(&context->tx_inflight);
		ret = mhi_queue_buf(context->mhi_dev, DMA_TO_DEVICE, buf,
				    pkt_data_len, mhi_flag);
		if (ret) {
			atomic_dec(&context->tx_inflight);
			context->read_data_length = 0;
			dev_err(&context->mhi_dev->dev,
				"Unable to send read_data response %d\n", ret);
			break;
		}
		context->tx_next = (context->tx_next + 1) % SAHARA_NUM_TX_BUF;
	}
	mutex_unlock(&context->tx_lock);

	return ret;
}

static void sahara_read_data(struct sahara_context *context)
{
	u32 image_id, data_offset, data_len;
	int ret;

	dev_dbg(&context->mhi_dev->dev,
		"READ_DATA cmd received. length:%d image:%d offset:%d data_length:%d\n",
		le32_to_cpu(context->rx->length),
		le32_to_cpu(context->rx->read_data.image),
		le32_to_cpu(context->rx->read_data.offset),
		le32_to_cpu(context->rx->read_data.length));

	if (le32_to_cpu(context->rx->length) != SAHARA_READ_DATA_LENGTH) {
		dev_err(&context->mhi_dev->dev, "Malformed read_data packet - length %d\n",
			le32_to_cpu(context->rx->length));
		return;
	}

	image_id = le32_to_cpu(context->rx->read_data.image);
	data_offset = le32_to_cpu(context->rx->read_data.offset);
	data_len = le32_to_cpu(context->rx->read_data.length);

	ret = sahara_find_image(context, image_id);
	if (ret) {
		sahara_send_reset(context);
		return;
	}

	/*
	 * Image is released when the device is done with it via
	 * SAHARA_END_OF_IMAGE_CMD. sahara_send_reset() will either cause the
	 * device to retry the operation with a modification, or decide to be
	 * done with the image and trigger SAHARA_END_OF_IMAGE_CMD.
	 * release_image() is called from SAHARA_END_OF_IMAGE_CMD. processing
	 * and is not needed here on error.
	 */

	if (data_offset >= context->img_size) {
		dev_err(&context->mhi_dev->dev, "Malformed read_data packet - data offset %d exceeds file size %zu\n",
			data_offset, context->img_size);
		sahara_send_reset(context);
		return;
	}

	if (size_add(data_offset, data_len) > context->img_size) {
		dev_err(&context->mhi_dev->dev, "Malformed read_data packet - data offset %d and length %d exceeds file size %zu\n",
			data_offset, data_len, context->img_size);
		sahara_send_reset(context);
		return;
	}

	if (data_len > 0x40000)
		dev_info(&context->mhi_dev->dev, "READ_DATA image %u offset %u len %u\n",
			 image_id, data_offset, data_len);

	context->read_data_offset = data_offset;
	context->read_data_length = data_len;

	read_data_helper(context);
}

static void sahara_end_of_image(struct sahara_context *context)
{
	int ret;

	dev_info(&context->mhi_dev->dev,
		"END_OF_IMAGE cmd received. length:%d image:%d status:%d\n",
		le32_to_cpu(context->rx->length),
		le32_to_cpu(context->rx->end_of_image.image),
		le32_to_cpu(context->rx->end_of_image.status));

	if (le32_to_cpu(context->rx->length) != SAHARA_END_OF_IMAGE_LENGTH) {
		dev_err(&context->mhi_dev->dev, "Malformed end_of_image packet - length %d\n",
			le32_to_cpu(context->rx->length));
		return;
	}

	if (context->active_image_id != SAHARA_IMAGE_ID_NONE &&
	    le32_to_cpu(context->rx->end_of_image.image) != context->active_image_id) {
		dev_err(&context->mhi_dev->dev, "Malformed end_of_image packet - image %d is not the active image\n",
			le32_to_cpu(context->rx->end_of_image.image));
		return;
	}

	sahara_release_image(context);

	if (le32_to_cpu(context->rx->end_of_image.status))
		return;

	context->cmd->cmd = cpu_to_le32(SAHARA_DONE_CMD);
	context->cmd->length = cpu_to_le32(SAHARA_DONE_LENGTH);

	ret = mhi_queue_buf(context->mhi_dev, DMA_TO_DEVICE, context->cmd,
			    SAHARA_DONE_LENGTH, MHI_EOT);
	if (ret)
		dev_dbg(&context->mhi_dev->dev, "Unable to send done response %d\n", ret);
}

static int sahara_send_mem_read(struct sahara_context *context, u64 addr, u64 len)
{
	if (context->dump32) {
		context->cmd->cmd = cpu_to_le32(SAHARA_MEM_READ_CMD);
		context->cmd->length = cpu_to_le32(SAHARA_MEM_READ_LENGTH);
		context->cmd->memory_read.memory_address = cpu_to_le32(addr);
		context->cmd->memory_read.memory_length = cpu_to_le32(len);
	} else {
		context->cmd->cmd = cpu_to_le32(SAHARA_MEM_READ64_CMD);
		context->cmd->length = cpu_to_le32(SAHARA_MEM_READ64_LENGTH);
		context->cmd->memory_read64.memory_address = cpu_to_le64(addr);
		context->cmd->memory_read64.memory_length = cpu_to_le64(len);
	}
	context->rx_size_requested = len;

	return mhi_queue_buf(context->mhi_dev, DMA_TO_DEVICE, context->cmd,
			     le32_to_cpu(context->cmd->length), MHI_EOT);
}

/* MEMORY_DEBUG (32-bit), as sent by the SDX55 remotefs_sahara EFS sync */
static void sahara_memory_debug(struct sahara_context *context)
{
	int ret;

	dev_info(&context->mhi_dev->dev,
		 "MEMORY DEBUG cmd received. length:%d table_address:%#x table_length:%#x\n",
		 le32_to_cpu(context->rx->length),
		 le32_to_cpu(context->rx->memory_debug.table_address),
		 le32_to_cpu(context->rx->memory_debug.table_length));

	if (le32_to_cpu(context->rx->length) != SAHARA_MEM_DEBUG_LENGTH) {
		dev_err(&context->mhi_dev->dev, "Malformed memory debug packet - length %d\n",
			le32_to_cpu(context->rx->length));
		return;
	}

	context->dump_table_address = le32_to_cpu(context->rx->memory_debug.table_address);
	context->dump_table_length = le32_to_cpu(context->rx->memory_debug.table_length);

	if (!context->dump_table_length ||
	    context->dump_table_length % sizeof(struct sahara_debug_table_entry32) ||
	    context->dump_table_length / sizeof(struct sahara_debug_table_entry32) *
	    sizeof(struct sahara_debug_table_entry64) > SAHARA_PACKET_MAX_SIZE) {
		dev_err(&context->mhi_dev->dev, "Unsupported memory debug table length %lld\n",
			context->dump_table_length);
		sahara_send_reset(context);
		return;
	}

	context->is_mem_dump_mode = true;
	context->dump32 = true;

	ret = sahara_send_mem_read(context, context->dump_table_address,
				   context->dump_table_length);
	if (ret)
		dev_err(&context->mhi_dev->dev, "Unable to send read for dump table %d\n", ret);
}

/* Rewrite a 32-bit debug table in context->rx into the 64-bit layout */
static void sahara_widen_dump_table(struct sahara_context *context)
{
	u64 n = context->dump_table_length / sizeof(struct sahara_debug_table_entry32);
	struct sahara_debug_table_entry32 *t32;
	struct sahara_debug_table_entry64 *t64;
	u64 i;

	t32 = kmemdup(context->rx, context->dump_table_length, GFP_KERNEL);
	if (!t32) {
		context->dump_table_length = 0;
		return;
	}
	t64 = (struct sahara_debug_table_entry64 *)context->rx;
	for (i = 0; i < n; i++) {
		t64[i].type = cpu_to_le64(le32_to_cpu(t32[i].type));
		t64[i].address = cpu_to_le64(le32_to_cpu(t32[i].address));
		t64[i].length = cpu_to_le64(le32_to_cpu(t32[i].length));
		memcpy(t64[i].description, t32[i].description, SAHARA_TABLE_ENTRY_STR_LEN);
		memcpy(t64[i].filename, t32[i].filename, SAHARA_TABLE_ENTRY_STR_LEN);
	}
	kfree(t32);
	context->dump_table_length = n * sizeof(*t64);
}

static void sahara_memory_debug64(struct sahara_context *context)
{
	int ret;

	dev_info(&context->mhi_dev->dev,
		"MEMORY DEBUG64 cmd received. length:%d table_address:%#llx table_length:%#llx\n",
		le32_to_cpu(context->rx->length),
		le64_to_cpu(context->rx->memory_debug64.table_address),
		le64_to_cpu(context->rx->memory_debug64.table_length));

	if (le32_to_cpu(context->rx->length) != SAHARA_MEM_DEBUG64_LENGTH) {
		dev_err(&context->mhi_dev->dev, "Malformed memory debug64 packet - length %d\n",
			le32_to_cpu(context->rx->length));
		return;
	}

	context->dump_table_address = le64_to_cpu(context->rx->memory_debug64.table_address);
	context->dump_table_length = le64_to_cpu(context->rx->memory_debug64.table_length);

	if (context->dump_table_length % sizeof(struct sahara_debug_table_entry64) != 0 ||
	    !context->dump_table_length) {
		dev_err(&context->mhi_dev->dev, "Malformed memory debug64 packet - table length %lld\n",
			context->dump_table_length);
		return;
	}

	/*
	 * From this point, the protocol flips. We make memory_read requests to
	 * the device, and the device responds with the raw data. If the device
	 * has an error, it will send an End of Image command. First we need to
	 * request the memory dump table so that we know where all the pieces
	 * of the dump are that we can consume.
	 */

	context->is_mem_dump_mode = true;
	context->dump32 = false;

	/*
	 * Assume that the table is smaller than our MTU so that we can read it
	 * in one shot. The spec does not put an upper limit on the table, but
	 * no known device will exceed this.
	 */
	if (context->dump_table_length > SAHARA_PACKET_MAX_SIZE) {
		dev_err(&context->mhi_dev->dev, "Memory dump table length %lld exceeds supported size. Discarding dump\n",
			context->dump_table_length);
		sahara_send_reset(context);
		return;
	}

	ret = sahara_send_mem_read(context, context->dump_table_address,
				   context->dump_table_length);
	if (ret)
		dev_err(&context->mhi_dev->dev, "Unable to send read for dump table %d\n", ret);
}

static void sahara_processing(struct work_struct *work)
{
	struct sahara_context *context = container_of(work, struct sahara_context, fw_work);
	int ret;

	switch (le32_to_cpu(context->rx->cmd)) {
	case SAHARA_HELLO_CMD:
		sahara_hello(context);
		break;
	case SAHARA_READ_DATA_CMD:
		sahara_read_data(context);
		break;
	case SAHARA_END_OF_IMAGE_CMD:
		sahara_end_of_image(context);
		break;
	case SAHARA_DONE_RESP_CMD:
		dev_info(&context->mhi_dev->dev, "DONE_RESP: image tx status %d\n",
			 le32_to_cpu(context->rx->hello.version));
		break;
	case SAHARA_RESET_RESP_CMD:
		/* Intentional do nothing as we don't need to exit an app */
		if (context->is_efs)
			dev_info(&context->mhi_dev->dev, "RESET_RESP: EFS sync session closed\n");
		break;
	case SAHARA_MEM_DEBUG64_CMD:
		sahara_memory_debug64(context);
		break;
	case SAHARA_MEM_DEBUG_CMD:
		sahara_memory_debug(context);
		break;
	default:
		dev_err(&context->mhi_dev->dev, "Unknown command %d\n",
			le32_to_cpu(context->rx->cmd));
		break;
	}

	ret = mhi_queue_buf(context->mhi_dev, DMA_FROM_DEVICE, context->rx,
			    SAHARA_PACKET_MAX_SIZE, MHI_EOT);
	if (ret)
		dev_err(&context->mhi_dev->dev, "Unable to requeue rx buf %d\n", ret);
}

static void sahara_parse_dump_table(struct sahara_context *context)
{
	struct sahara_dump_table_entry *image_out_table;
	struct sahara_debug_table_entry64 *dev_table;
	struct sahara_memory_dump_meta_v1 *dump_meta;
	u64 table_nents;
	u64 dump_length;
	u64 mul_bytes;
	int ret;
	u64 i;

	table_nents = context->dump_table_length / sizeof(*dev_table);
	context->dump_images_left = table_nents;
	context->dump_table_entries = table_nents;
	dump_length = 0;

	dev_table = (struct sahara_debug_table_entry64 *)(context->rx);
	for (i = 0; i < table_nents; ++i) {
		/* Do not trust the device, ensure the strings are terminated */
		dev_table[i].description[SAHARA_TABLE_ENTRY_STR_LEN - 1] = 0;
		dev_table[i].filename[SAHARA_TABLE_ENTRY_STR_LEN - 1] = 0;

		if (check_add_overflow(dump_length,
				       le64_to_cpu(dev_table[i].length),
				       &dump_length)) {
			/* Discard the dump */
			sahara_send_reset(context);
			return;
		}

		dev_info(&context->mhi_dev->dev,
			"Memory dump table entry %lld type: %lld address: %#llx length: %#llx description: \"%s\" filename \"%s\"\n",
			i,
			le64_to_cpu(dev_table[i].type),
			le64_to_cpu(dev_table[i].address),
			le64_to_cpu(dev_table[i].length),
			dev_table[i].description,
			dev_table[i].filename);
	}

	if (check_add_overflow(dump_length, (u64)sizeof(*dump_meta), &dump_length)) {
		/* Discard the dump */
		sahara_send_reset(context);
		return;
	}
	if (check_mul_overflow((u64)sizeof(*image_out_table), table_nents, &mul_bytes)) {
		/* Discard the dump */
		sahara_send_reset(context);
		return;
	}
	if (check_add_overflow(dump_length, mul_bytes, &dump_length)) {
		/* Discard the dump */
		sahara_send_reset(context);
		return;
	}

	context->mem_dump_sz = dump_length;
	context->mem_dump = vzalloc(dump_length);
	if (!context->mem_dump) {
		/* Discard the dump */
		sahara_send_reset(context);
		return;
	}

	/* Populate the dump metadata and table for userspace */
	dump_meta = context->mem_dump;
	dump_meta->magic = SAHARA_DUMP_V1_MAGIC;
	dump_meta->version = SAHARA_DUMP_V1_VER;
	dump_meta->dump_size = dump_length;
	dump_meta->table_size = context->dump_table_length;

	image_out_table = context->mem_dump + sizeof(*dump_meta);
	for (i = 0; i < table_nents; ++i) {
		image_out_table[i].type = le64_to_cpu(dev_table[i].type);
		image_out_table[i].address = le64_to_cpu(dev_table[i].address);
		image_out_table[i].length = le64_to_cpu(dev_table[i].length);
		strscpy(image_out_table[i].description, dev_table[i].description,
			SAHARA_TABLE_ENTRY_STR_LEN);
		strscpy(image_out_table[i].filename,
			dev_table[i].filename,
			SAHARA_TABLE_ENTRY_STR_LEN);
	}

	context->mem_dump_freespace = &image_out_table[i];

	/* Done parsing the table, switch to image dump mode */
	context->dump_table_length = 0;

	/* Request the first chunk of the first image */
	context->dump_image = &image_out_table[0];
	dump_length = min_t(u64, context->dump_image->length, SAHARA_READ_MAX_SIZE);
	/* Avoid requesting EOI sized data so that we can identify errors */
	if (dump_length == SAHARA_END_OF_IMAGE_LENGTH)
		dump_length = SAHARA_END_OF_IMAGE_LENGTH / 2;

	context->dump_image_offset = dump_length;

	ret = sahara_send_mem_read(context, context->dump_image->address, dump_length);
	if (ret)
		dev_err(&context->mhi_dev->dev, "Unable to send read for dump content %d\n", ret);
}

#define SAHARA_EFS_MAX_IMAGE	(64u * 1024 * 1024)

/*
 * Map the names the X55 reports in its EFS memory-debug table to the
 * partition labels on the host. The modem sends the short form
 * ("m9kefs2"); the GPT labels carry the "mdm1" prefix.
 */
static const char *sahara_efs_partition_name(const char *name)
{
	static const struct {
		const char *reported;
		const char *partition;
	} map[] = {
		{ "m9kefs1", "mdm1m9kefs1" },
		{ "mdm1m9kefs1", "mdm1m9kefs1" },
		{ "m9kefs2", "mdm1m9kefs2" },
		{ "mdm1m9kefs2", "mdm1m9kefs2" },
		{ "m9kefs3", "mdm1m9kefs3" },
		{ "mdm1m9kefs3", "mdm1m9kefs3" },
		{ "oemnvbk", "mdm1oemnvbktmp" },
		{ "oemnvbktmp", "mdm1oemnvbktmp" },
		{ "mdm1oemnvbktmp", "mdm1oemnvbktmp" },
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(map); i++)
		if (!strcmp(name, map[i].reported))
			return map[i].partition;
	return NULL;
}

static int sahara_efs_write_file(struct sahara_context *context, const char *name,
				 const void *data, u64 length)
{
	char path[PATH_MAX];
	struct file *filp;
	struct block_device *bdev;
	loff_t pos = 0, part_size;
	ssize_t n;
	int ret;

	/* Never touch a partition unless the whole image arrived and fits. */
	if (!length || length > SAHARA_EFS_MAX_IMAGE)
		return -EINVAL;

	snprintf(path, sizeof(path), "/dev/block/bootdevice/by-name/%s", name);
	filp = filp_open(path, O_RDWR | O_LARGEFILE, 0);
	if (IS_ERR(filp)) {
		dev_err(&context->mhi_dev->dev, "EFS %s: open failed %ld\n",
			name, PTR_ERR(filp));
		return PTR_ERR(filp);
	}

	bdev = file_bdev(filp);
	if (!bdev) {
		dev_err(&context->mhi_dev->dev, "EFS %s: not a block device\n",
			name);
		ret = -ENODEV;
		goto out;
	}
	part_size = bdev_nr_bytes(bdev);
	if (length != part_size) {
		dev_err(&context->mhi_dev->dev,
			"EFS %s: image %llu bytes != partition %lld bytes, not written\n",
			name, length, part_size);
		ret = -EINVAL;
		goto out;
	}

	n = kernel_write(filp, data, length, &pos);
	if (n != (ssize_t)length) {
		dev_err(&context->mhi_dev->dev, "EFS %s: short write %zd/%llu\n",
			name, n, length);
		ret = n < 0 ? n : -EIO;
		goto out;
	}

	ret = vfs_fsync(filp, 0);
	if (!ret)
		dev_info(&context->mhi_dev->dev, "EFS %s: wrote %llu bytes\n",
			 name, length);
out:
	filp_close(filp, NULL);
	return ret;
}

static void sahara_efs_persist(struct sahara_context *context)
{
	struct sahara_dump_table_entry *entries;
	const u8 *data;
	u32 i;

	if (!context->efs_writeback) {
		dev_info(&context->mhi_dev->dev,
			 "EFS sync received (%zu bytes), write-back disabled\n",
			 context->mem_dump_sz);
		return;
	}

	entries = (struct sahara_dump_table_entry *)
		((u8 *)context->mem_dump + sizeof(struct sahara_memory_dump_meta_v1));
	data = (const u8 *)&entries[context->dump_table_entries];

	for (i = 0; i < context->dump_table_entries; i++) {
		const char *part;

		entries[i].filename[SAHARA_TABLE_ENTRY_STR_LEN - 1] = 0;
		part = sahara_efs_partition_name(entries[i].filename);
		if (part) {
			dev_info(&context->mhi_dev->dev,
				 "EFS entry \"%s\" (%llu bytes) -> partition %s\n",
				 entries[i].filename, entries[i].length, part);
			sahara_efs_write_file(context, part, data, entries[i].length);
		} else
			dev_warn(&context->mhi_dev->dev,
				 "EFS entry \"%s\" (%llu bytes) not in partition whitelist\n",
				 entries[i].filename, entries[i].length);
		data += entries[i].length;
	}
}

static void sahara_parse_dump_image(struct sahara_context *context)
{
	u64 dump_length;
	int ret;

	memcpy(context->mem_dump_freespace, context->rx, context->rx_size);
	context->mem_dump_freespace += context->rx_size;

	if (context->dump_image_offset >= context->dump_image->length) {
		/* Need to move to next image */
		context->dump_image++;
		context->dump_images_left--;
		context->dump_image_offset = 0;

		if (!context->dump_images_left) {
			/* Dump done */
			if (context->is_efs) {
				sahara_efs_persist(context);
				vfree(context->mem_dump);
			} else {
				dev_coredumpv(context->mhi_dev->mhi_cntrl->cntrl_dev,
					      context->mem_dump,
					      context->mem_dump_sz,
					      GFP_KERNEL);
			}
			context->mem_dump = NULL;
			sahara_send_reset(context);
			return;
		}
	}

	/* Get next image chunk */
	dump_length = context->dump_image->length - context->dump_image_offset;
	dump_length = min_t(u64, dump_length, SAHARA_READ_MAX_SIZE);
	/* Avoid requesting EOI sized data so that we can identify errors */
	if (dump_length == SAHARA_END_OF_IMAGE_LENGTH)
		dump_length = SAHARA_END_OF_IMAGE_LENGTH / 2;

	ret = sahara_send_mem_read(context,
				   context->dump_image->address + context->dump_image_offset,
				   dump_length);
	context->dump_image_offset += dump_length;
	if (ret)
		dev_err(&context->mhi_dev->dev,
			"Unable to send read for dump content %d\n", ret);
}

static void sahara_dump_processing(struct work_struct *work)
{
	struct sahara_context *context = container_of(work, struct sahara_context, dump_work);
	int ret;

	/*
	 * We should get the expected raw data, but if the device has an error
	 * it is supposed to send EOI with an error code.
	 */
	if (context->rx_size != context->rx_size_requested &&
	    context->rx_size != SAHARA_END_OF_IMAGE_LENGTH) {
		dev_err(&context->mhi_dev->dev,
			"Unexpected response to read_data. Expected size: %#zx got: %#zx\n",
			context->rx_size_requested,
			context->rx_size);
		goto error;
	}

	if (context->rx_size == SAHARA_END_OF_IMAGE_LENGTH &&
	    le32_to_cpu(context->rx->cmd) == SAHARA_END_OF_IMAGE_CMD) {
		dev_err(&context->mhi_dev->dev,
			"Unexpected EOI response to read_data. Status: %d\n",
			le32_to_cpu(context->rx->end_of_image.status));
		goto error;
	}

	if (context->rx_size == SAHARA_END_OF_IMAGE_LENGTH &&
	    le32_to_cpu(context->rx->cmd) != SAHARA_END_OF_IMAGE_CMD) {
		dev_err(&context->mhi_dev->dev,
			"Invalid EOI response to read_data. CMD: %d\n",
			le32_to_cpu(context->rx->cmd));
		goto error;
	}

	/*
	 * Need to know if we received the dump table, or part of a dump image.
	 * Since we get raw data, we cannot tell from the data itself. Instead,
	 * we use the stored dump_table_length, which we zero after we read and
	 * process the entire table.
	 */
	if (context->dump_table_length && context->dump32) {
		sahara_widen_dump_table(context);
		if (!context->dump_table_length)
			goto error;
	}
	if (context->dump_table_length)
		sahara_parse_dump_table(context);
	else
		sahara_parse_dump_image(context);

	ret = mhi_queue_buf(context->mhi_dev, DMA_FROM_DEVICE, context->rx,
			    SAHARA_PACKET_MAX_SIZE, MHI_EOT);
	if (ret)
		dev_err(&context->mhi_dev->dev, "Unable to requeue rx buf %d\n", ret);

	return;

error:
	vfree(context->mem_dump);
	context->mem_dump = NULL;
	sahara_send_reset(context);
}

static void sahara_read_data_processing(struct work_struct *work)
{
	struct sahara_context *context = container_of(work, struct sahara_context, read_data_work);

	read_data_helper(context);
}

static int sahara_mhi_probe(struct mhi_device *mhi_dev, const struct mhi_device_id *id)
{
	struct sahara_context *context;
	int ret;
	int i;

	context = devm_kzalloc(&mhi_dev->dev, sizeof(*context), GFP_KERNEL);
	if (!context)
		return -ENOMEM;

	context->rx = devm_kzalloc(&mhi_dev->dev, SAHARA_PACKET_MAX_SIZE, GFP_KERNEL);
	if (!context->rx)
		return -ENOMEM;

	context->image_table = sdx55m_image_table;
	context->table_size = ARRAY_SIZE(sdx55m_image_table);

	context->cmd = devm_kzalloc(&mhi_dev->dev, SAHARA_PACKET_MAX_SIZE, GFP_KERNEL);
	if (!context->cmd)
		return -ENOMEM;

	/*
	 * There are two firmware implementations for READ_DATA handling.
	 * The older "SBL" implementation defines a Sahara transfer size, and
	 * expects that the response is a single transport transfer. If the
	 * FW wants to transfer a file that is larger than the transfer size,
	 * the FW will issue multiple READ_DATA commands. For this
	 * implementation, we need to allocate enough buffers to contain the
	 * entire Sahara transfer size.
	 *
	 * The newer "XBL" implementation does not define a maximum transfer
	 * size and instead expects the data to be streamed over using the
	 * transport level MTU. The FW will issue a single READ_DATA command
	 * of whatever size, and consume multiple transport level transfers
	 * until the expected amount of data is consumed. For this
	 * implementation we only need a single buffer of the transport MTU
	 * but we'll need to be able to use it multiple times for a single
	 * READ_DATA request.
	 *
	 * AIC100 is the SBL implementation and defines SAHARA_TRANSFER_MAX_SIZE
	 * and we need 9x SAHARA_PACKET_MAX_SIZE to cover that. We can use
	 * MHI_CHAIN to link multiple buffers into a single transfer but the
	 * remote side will not consume the buffers until it sees an EOT, thus
	 * we need to allocate enough buffers to put in the tx fifo to cover an
	 * entire READ_DATA request of the max size.
	 *
	 * AIC200 is the XBL implementation, and so a single buffer will work.
	 */
	for (i = 0; i < SAHARA_NUM_TX_BUF; ++i) {
		context->tx[i] = devm_kzalloc(&mhi_dev->dev,
					      SAHARA_PACKET_MAX_SIZE,
					      GFP_KERNEL);
		if (!context->tx[i])
			return -ENOMEM;
	}
	mutex_init(&context->tx_lock);
	atomic_set(&context->tx_inflight, 0);

	context->mhi_dev = mhi_dev;
	context->is_efs = !strcmp(id->chan, "EFS");
	context->efs_writeback = efs_writeback;
	INIT_WORK(&context->fw_work, sahara_processing);
	INIT_WORK(&context->dump_work, sahara_dump_processing);
	INIT_WORK(&context->read_data_work, sahara_read_data_processing);

	context->active_image_id = SAHARA_IMAGE_ID_NONE;
	dev_set_drvdata(&mhi_dev->dev, context);

	ret = mhi_prepare_for_transfer(mhi_dev);
	if (ret)
		return ret;

	ret = mhi_queue_buf(mhi_dev, DMA_FROM_DEVICE, context->rx, SAHARA_PACKET_MAX_SIZE, MHI_EOT);
	if (ret) {
		mhi_unprepare_from_transfer(mhi_dev);
		return ret;
	}

	return 0;
}

static void sahara_mhi_remove(struct mhi_device *mhi_dev)
{
	struct sahara_context *context = dev_get_drvdata(&mhi_dev->dev);

	cancel_work_sync(&context->fw_work);
	cancel_work_sync(&context->dump_work);
	vfree(context->mem_dump);
	sahara_release_image(context);
	mhi_unprepare_from_transfer(mhi_dev);
}

static void sahara_mhi_ul_xfer_cb(struct mhi_device *mhi_dev, struct mhi_result *mhi_result)
{
	struct sahara_context *context = dev_get_drvdata(&mhi_dev->dev);

	if (mhi_result->buf_addr == context->cmd)
		return;

	/* a run's TREs all complete on its EOT event: refill once it drains */
	if (atomic_dec_return(&context->tx_inflight) == 0 &&
	    !mhi_result->transaction_status && context->read_data_length)
		schedule_work(&context->read_data_work);
}

static void sahara_mhi_dl_xfer_cb(struct mhi_device *mhi_dev, struct mhi_result *mhi_result)
{
	struct sahara_context *context = dev_get_drvdata(&mhi_dev->dev);

	if (!mhi_result->transaction_status) {
		context->rx_size = mhi_result->bytes_xferd;
		if (context->is_mem_dump_mode)
			schedule_work(&context->dump_work);
		else
			schedule_work(&context->fw_work);
	}

}

static const struct mhi_device_id sahara_mhi_match_table[] = {
	{ .chan = "SAHARA", },
	{ .chan = "EFS", },
	{},
};

static struct mhi_driver sahara_mhi_driver = {
	.id_table = sahara_mhi_match_table,
	.remove = sahara_mhi_remove,
	.probe = sahara_mhi_probe,
	.ul_xfer_cb = sahara_mhi_ul_xfer_cb,
	.dl_xfer_cb = sahara_mhi_dl_xfer_cb,
	.driver = {
		.name = "sahara-mdm",
	},
};

MODULE_DEVICE_TABLE(mhi, sahara_mhi_match_table);
module_mhi_driver(sahara_mhi_driver);

MODULE_DESCRIPTION("Sahara image loader for flashless MHI modems");
MODULE_LICENSE("GPL");
