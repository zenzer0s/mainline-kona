// SPDX-License-Identifier: GPL-2.0-only
/*
 *  Driver for Synaptics TCM Oncell Touchscreens
 *
 *  Copyright (c) 2024 Frieder Hannenheim <frieder.hannenheim@proton.me>
 *  Copyright (c) 2024 Casey Connolly <kcxt@postmarketos.org>
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/unaligned.h>

/*
 * The TCM oncell interface uses a command byte, which may be followed by additional
 * data. The packet format is defined in the tcm_cmd struct.
 *
 * The following list only defines commands that are used in this driver (and their
 * counterparts for context). Vendor reference implementations can be found at
 * https://github.com/LineageOS/android_kernel_oneplus_sm8250/tree/ee0a7ee1939ffd53000e42051caf8f0800defb27/drivers/input/touchscreen/synaptics_tcm
 */

/*
 * Request information about the chip. We don't send this command explicitly as
 * the controller automatically sends this information when starting up.
 */
#define TCM_IDENTIFY				0x02

/* Enable/disable reporting touch inputs */
#define TCM_ENABLE_REPORT			0x05
#define TCM_DISABLE_REPORT			0x06

/*
 * After powering on, we send this to exit the bootloader mode and run the main
 * firmware.
 */
#define TCM_RUN_APPLICATION_FIRMWARE		0x14

/*
 * Reports information about the vendor provided application firmware. This is
 * also used to determine when the firmware has finished booting.
 */
#define TCM_GET_APPLICATION_INFO		0x20
#define TCM_SET_DYNAMIC_CONFIG			0x24
#define TCM_GET_TOUCH_REPORT_CONFIG		0x25

#define TCM_MESSAGE_MARKER			0xa5

#define MODE_APPLICATION			0x01

#define APP_STATUS_OK				0x00
#define APP_STATUS_BOOTING			0x01
#define APP_STATUS_UPDATING			0x02

/* Dynamic config IDs */
#define DC_DARK_ZONE_ENABLE			0xdd
#define DC_GRIP_ENABLED				0xde

/* Touch object classification */
#define TCM_TOUCH_LIFT				0x00

/* Touch report configuration codes */
#define TOUCH_END				0x00
#define TOUCH_FOREACH_ACTIVE_OBJECT		0x01
#define TOUCH_FOREACH_OBJECT			0x02
#define TOUCH_FOREACH_END			0x03
#define TOUCH_PAD_TO_NEXT_BYTE			0x04
#define TOUCH_OBJECT_N_INDEX			0x06
#define TOUCH_OBJECT_N_CLASSIFICATION		0x07
#define TOUCH_OBJECT_N_X_POSITION		0x08
#define TOUCH_OBJECT_N_Y_POSITION		0x09
#define TOUCH_OBJECT_N_Z			0x0a
#define TOUCH_OBJECT_N_X_WIDTH			0x0b
#define TOUCH_OBJECT_N_Y_WIDTH			0x0c
#define TOUCH_NUM_OF_ACTIVE_OBJECTS		0x18

#define TCM_MAX_OBJECTS				10
#define TCM_BUF_SIZE				512
#define TCM_MAX_REPORT_CONFIG_SIZE		256
#define TCM_MAX_TOUCH_WIDTH			200

/* status codes */
#define REPORT_IDLE				0x00
#define REPORT_OK				0x01
#define REPORT_BUSY				0x02
#define REPORT_CONTINUED_READ			0x03
#define REPORT_RECEIVE_BUFFER_OVERFLOW		0x0c
#define REPORT_PREVIOUS_COMMAND_PENDING		0x0d
#define REPORT_NOT_IMPLEMENTED			0x0e
#define REPORT_ERROR				0x0f

/* report types */
#define REPORT_IDENTIFY				0x10
#define REPORT_TOUCH				0x11
#define REPORT_DELTA				0x12
#define REPORT_RAW				0x13
#define REPORT_DEBUG				0x14
#define REPORT_LOG				0x1d
#define REPORT_TOUCH_HOLD			0x20
#define REPORT_INVALID				0xff

struct tcm_message_header {
	u8 marker;
	u8 code;
	__le16 length;
} __packed;

struct tcm_cmd {
	u8 cmd;
	__le16 length;
	u8 data[];
} __packed;

struct tcm_identification {
	struct tcm_message_header header;
	u8 version;
	u8 mode;
	char part_number[16];
	u8 build_id[4];
	u8 max_write_size[2];
} __packed;

struct tcm_app_info {
	struct tcm_message_header header;
	u8 version[2];
	__le16 status;
	u8 static_config_size[2];
	u8 dynamic_config_size[2];
	u8 app_config_start_write_block[2];
	u8 app_config_size[2];
	u8 max_touch_report_config_size[2];
	u8 max_touch_report_payload_size[2];
	char customer_config_id[16];
	__le16 max_x;
	__le16 max_y;
	u8 max_objects[2];
	u8 num_of_buttons[2];
	u8 num_of_image_rows[2];
	u8 num_of_image_cols[2];
	u8 has_hybrid_data[2];
} __packed;

struct tcm_data {
	struct i2c_client *client;
	struct regmap *regmap;
	struct input_dev *input;
	struct gpio_desc *reset_gpio;
	struct completion response;
	struct touchscreen_properties props;
	struct regulator_bulk_data supplies[2];

	u16 max_x;
	u16 max_y;
	u16 max_objects;

	u8 report_config[TCM_MAX_REPORT_CONFIG_SIZE];
	u16 report_config_len;
	u16 report_obj_bits;

	/* annoying state */
	u16 buf_size;
	u8 buf[TCM_BUF_SIZE];
	u8 irq_buf[TCM_BUF_SIZE];
};

static int tcm_send_cmd(struct tcm_data *tcm, struct tcm_cmd *cmd)
{
	struct i2c_client *client = tcm->client;
	struct i2c_msg msg;
	u16 payload_len = le16_to_cpu(cmd->length);
	int ret;

	dev_dbg(&client->dev, "sending command %#x len %u\n", cmd->cmd, payload_len);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = payload_len ? (sizeof(*cmd) + payload_len) : 1;
	msg.buf = (u8 *)cmd;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret == 1)
		return 0;
	else if (ret < 0)
		return ret;
	else
		return -EIO;
}

static int tcm_recv_report(struct tcm_data *tcm,
			   void *buf, size_t length)
{
	struct i2c_client *client = tcm->client;
	struct i2c_msg msg;
	int ret;

	msg.addr = client->addr;
	msg.flags = I2C_M_RD;
	msg.len = length;
	msg.buf = buf;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret == 1)
		return 0;
	else if (ret < 0)
		return ret;
	else
		return -EIO;
}

static int tcm_read_message_len(struct tcm_data *tcm, u8 cmd, const void *payload,
				u16 payload_len, void *buf, size_t length, u16 *out_len)
{
	u8 cmd_buf[sizeof(struct tcm_cmd) + 16];
	struct tcm_cmd *c = (struct tcm_cmd *)cmd_buf;
	const struct tcm_message_header *hdr;
	int ret;

	if (payload_len > sizeof(cmd_buf) - sizeof(*c))
		return -EINVAL;

	c->cmd = cmd;
	c->length = cpu_to_le16(payload_len);
	if (payload_len && payload)
		memcpy(c->data, payload, payload_len);

	reinit_completion(&tcm->response);
	ret = tcm_send_cmd(tcm, c);
	if (ret)
		return ret;

	ret = wait_for_completion_timeout(&tcm->response, msecs_to_jiffies(1000));
	if (ret == 0)
		return -ETIMEDOUT;

	hdr = (const struct tcm_message_header *)tcm->buf;
	if (hdr->code != REPORT_OK && hdr->code != REPORT_IDENTIFY)
		return -EIO;

	if (out_len)
		*out_len = tcm->buf_size;

	if (buf) {
		if (length > tcm->buf_size) {
			dev_dbg(&tcm->client->dev, "expected %zu bytes, got %u\n",
				length, tcm->buf_size);
		}
		length = min_t(size_t, tcm->buf_size, length);
		memcpy(buf, tcm->buf, length);
	}

	return 0;
}

static int tcm_read_message(struct tcm_data *tcm, u8 cmd, void *buf, size_t length)
{
	return tcm_read_message_len(tcm, cmd, NULL, 0, buf, length, NULL);
}

static int tcm_set_dynamic_config(struct tcm_data *tcm, u8 id, u16 val)
{
	u8 payload[3] = {
		id,
		(u8)(val & 0xff),
		(u8)(val >> 8),
	};

	return tcm_read_message_len(tcm, TCM_SET_DYNAMIC_CONFIG, payload,
				    sizeof(payload), NULL, 0, NULL);
}

static void tcm_power_off(void *data)
{
	struct tcm_data *tcm = data;

	disable_irq(tcm->client->irq);
	regulator_bulk_disable(ARRAY_SIZE(tcm->supplies), tcm->supplies);
}

static int tcm_input_open(struct input_dev *dev)
{
	struct tcm_data *tcm = input_get_drvdata(dev);

	return i2c_smbus_write_byte(tcm->client, TCM_ENABLE_REPORT);
}

static void tcm_input_close(struct input_dev *dev)
{
	struct tcm_data *tcm = input_get_drvdata(dev);
	int ret;

	ret = i2c_smbus_write_byte(tcm->client, TCM_DISABLE_REPORT);
	if (ret)
		dev_err(&tcm->client->dev, "failed to turn off sensing\n");
}

/*
 * The default report config looks like this:
 *
 * a5 01 80 00 11 08 1e 08 0f 01 04 01 06 04 07 04
 * 08 0c 09 0c 0a 08 0b 08 0c 08 0d 10 0e 10 03 00
 * 00 00
 *
 * a5 01 80 00 - HEADER + length
 *
 * 11 08 - TOUCH_FRAME_RATE (8 bits)
 * 30 08 - UNKNOWN (8 bits)
 * 0f 01 - TOUCH_0D_BUTTONS_STATE (1 bit)
 * 04 01 - TOUCH_PAD_TO_NEXT_BYTE (7 bits - padding)
 * 06 04 - TOUCH_OBJECT_N_INDEX (4 bits)
 * 07 04 - TOUCH_OBJECT_N_CLASSIFICATION (4 bits)
 * 08 0c - TOUCH_OBJECT_N_X_POSITION (12 bits)
 * 09 0c - TOUCH_OBJECT_N_Y_POSITION (12 bits)
 * 0a 08 - TOUCH_OBJECT_N_Z (8 bits)
 * 0b 08 - TOUCH_OBJECT_N_X_WIDTH (8 bits)
 * 0c 08 - TOUCH_OBJECT_N_Y_WIDTH (8 bits)
 * 0d 10 - TOUCH_OBJECT_N_TX_POSITION_TIXELS (16 bits) ??
 * 0e 10 - TOUCH_OBJECT_N_RX_POSITION_TIXELS (16 bits) ??
 * 03 00 - TOUCH_FOREACH_END (0 bits)
 * 00 00 - TOUCH_END (0 bits)
 *
 * Since we only support this report config, we just hardcode the format below.
 * To support additional report configs, we would need to parse the config and
 * use it to parse the reports dynamically.
 */

struct tcm_report_point {
	u8 unknown;
	u8 buttons;
	__le32 point; /* idx : 4, class : 4, x : 12, y : 12 */
	// u8 idx : 4;
	// u8 classification : 4;
	// u16 x : 12;
	// u16 y : 12;
	u8 z;
	u8 width_x;
	u8 width_y;
	u8 tx;
	u8 rx;
} __packed;

struct tcm_touch_object {
	u8 status;
	u16 x;
	u16 y;
	u16 width_x;
	u16 width_y;
	u8 z;
};

static u32 tcm_get_report_bits(const u8 *buf, size_t buf_len,
			       unsigned int offset, unsigned int bits)
{
	unsigned int bit_offset = offset % 8;
	unsigned int byte_offset = offset / 8;
	unsigned int remaining = bits;
	u32 output = 0;

	if (bits == 0 || bits > 32 || offset + bits > buf_len * 8)
		return 0;

	while (remaining) {
		unsigned int avail = 8 - bit_offset;
		unsigned int take = min(avail, remaining);
		u8 mask = (u8)(0xff >> (8 - take));
		u8 val = (buf[byte_offset] >> bit_offset) & mask;

		output |= (u32)val << (bits - remaining);
		bit_offset = 0;
		byte_offset++;
		remaining -= take;
	}

	return output;
}

static u16 tcm_calc_obj_bits(const u8 *cfg, u16 cfg_len)
{
	unsigned int idx = 0;
	unsigned int bits = 0;
	bool in_loop = false;

	while (idx < cfg_len) {
		u8 code = cfg[idx++];

		switch (code) {
		case TOUCH_END:
			return bits ? bits : 56;
		case TOUCH_FOREACH_ACTIVE_OBJECT:
		case TOUCH_FOREACH_OBJECT:
			in_loop = true;
			bits = 0;
			break;
		case TOUCH_FOREACH_END:
			return bits ? bits : 56;
		case TOUCH_PAD_TO_NEXT_BYTE:
			if (in_loop)
				bits = ALIGN(bits, 8);
			break;
		default:
			if (idx >= cfg_len)
				return bits ? bits : 56;
			if (in_loop)
				bits += cfg[idx];
			idx++;
			break;
		}
	}

	return bits ? bits : 56;
}

static void tcm_report_objects(struct tcm_data *tcm,
			       const struct tcm_touch_object *objs)
{
	u16 idx;

	/* We don't need to report releases because we have INPUT_MT_DROP_UNUSED */
	for (idx = 0; idx < tcm->max_objects; idx++) {
		u16 major_width, minor_width;

		if (objs[idx].status == TCM_TOUCH_LIFT)
			continue;

		if (objs[idx].x > tcm->max_x || objs[idx].y > tcm->max_y ||
		    objs[idx].width_x > TCM_MAX_TOUCH_WIDTH ||
		    objs[idx].width_y > TCM_MAX_TOUCH_WIDTH) {
			dev_dbg(&tcm->client->dev,
				"dropping anomalous touch idx=%u status=%u x=%u y=%u wx=%u wy=%u\n",
				idx, objs[idx].status, objs[idx].x, objs[idx].y,
				objs[idx].width_x, objs[idx].width_y);
			continue;
		}

		minor_width = objs[idx].width_x;
		major_width = objs[idx].width_y;
		if (minor_width > major_width)
			swap(major_width, minor_width);

		dev_dbg(&tcm->client->dev, "touch report: idx %u x %u y %u\n",
			idx, objs[idx].x, objs[idx].y);

		input_mt_slot(tcm->input, idx);
		input_mt_report_slot_state(tcm->input, MT_TOOL_FINGER, true);

		touchscreen_report_pos(tcm->input, &tcm->props,
				       objs[idx].x, objs[idx].y, true);

		input_report_abs(tcm->input, ABS_MT_TOUCH_MAJOR, major_width);
		input_report_abs(tcm->input, ABS_MT_TOUCH_MINOR, minor_width);
		input_report_abs(tcm->input, ABS_MT_PRESSURE,
				 objs[idx].z ? objs[idx].z : 1);
	}

	input_mt_sync_frame(tcm->input);
	input_sync(tcm->input);
}

static int tcm_handle_touch_report_dynamic(struct tcm_data *tcm,
					   u8 *payload, size_t payload_len)
{
	struct tcm_touch_object objs[TCM_MAX_OBJECTS] = { 0 };
	const u8 *cfg = tcm->report_config;
	u16 cfg_len = tcm->report_config_len;
	unsigned int idx = 0, next = 0, end_of_foreach = 0;
	unsigned int offset = 0, obj = 0, objects = 0, active_objects = 0;
	unsigned int min_obj_bits = min_t(unsigned int, tcm->report_obj_bits, 56);
	bool active_only = false, has_num_active = false;

	/*
	 * Aftermarket replacement OnePlus 8T panels may insert a 2-byte
	 * 0xa5 0x03 transient prefix at bytes [8..9] when (payload[7] & 0x0f)
	 * is 0, shifting the touch payload and dropping quick taps.
	 */
	if (payload_len >= 10 && (payload[7] & 0x0f) == 0x00 &&
	    payload[8] == TCM_MESSAGE_MARKER && payload[9] == REPORT_CONTINUED_READ) {
		memmove(&payload[8], &payload[10], payload_len - 10);
		memset(&payload[payload_len - 2], 0, 2);
	}

	while (idx < cfg_len) {
		u8 code = cfg[idx++];
		unsigned int bits;
		u32 data;

		switch (code) {
		case TOUCH_END:
			goto done;
		case TOUCH_FOREACH_ACTIVE_OBJECT:
			obj = 0;
			objects = 0;
			next = idx;
			active_only = true;
			if ((has_num_active && active_objects == 0) ||
			    offset + min_obj_bits > payload_len * 8) {
				while (idx < cfg_len && cfg[idx] != TOUCH_FOREACH_END) {
					u8 skip = cfg[idx++];

					if (skip != TOUCH_PAD_TO_NEXT_BYTE && idx < cfg_len)
						idx++;
				}
				if (idx < cfg_len)
					idx++;
			}
			break;
		case TOUCH_FOREACH_OBJECT:
			obj = 0;
			next = idx;
			active_only = false;
			break;
		case TOUCH_FOREACH_END:
			end_of_foreach = idx;
			if (active_only) {
				if (has_num_active) {
					objects++;
					if (objects < active_objects)
						idx = next;
				} else if (offset + tcm->report_obj_bits <= payload_len * 8) {
					idx = next;
				}
			} else {
				obj++;
				if (obj < tcm->max_objects)
					idx = next;
			}
			break;
		case TOUCH_PAD_TO_NEXT_BYTE:
			offset = ALIGN(offset, 8);
			break;
		case TOUCH_OBJECT_N_INDEX:
			if (idx >= cfg_len)
				goto done;
			bits = cfg[idx++];
			obj = tcm_get_report_bits(payload, payload_len, offset, bits);
			offset += bits;
			break;
		case TOUCH_OBJECT_N_CLASSIFICATION:
			if (idx >= cfg_len)
				goto done;
			bits = cfg[idx++];
			data = tcm_get_report_bits(payload, payload_len, offset, bits);
			if (obj < tcm->max_objects)
				objs[obj].status = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_X_POSITION:
			if (idx >= cfg_len)
				goto done;
			bits = cfg[idx++];
			data = tcm_get_report_bits(payload, payload_len, offset, bits);
			if (obj < tcm->max_objects)
				objs[obj].x = data & 0x0fff;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_Y_POSITION:
			if (idx >= cfg_len)
				goto done;
			bits = cfg[idx++];
			data = tcm_get_report_bits(payload, payload_len, offset, bits);
			if (obj < tcm->max_objects)
				objs[obj].y = data & 0x0fff;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_Z:
			if (idx >= cfg_len)
				goto done;
			bits = cfg[idx++];
			data = tcm_get_report_bits(payload, payload_len, offset, bits);
			if (obj < tcm->max_objects)
				objs[obj].z = data;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_X_WIDTH:
			if (idx >= cfg_len)
				goto done;
			bits = cfg[idx++];
			data = tcm_get_report_bits(payload, payload_len, offset, bits);
			if (obj < tcm->max_objects)
				objs[obj].width_x = data & 0x0fff;
			offset += bits;
			break;
		case TOUCH_OBJECT_N_Y_WIDTH:
			if (idx >= cfg_len)
				goto done;
			bits = cfg[idx++];
			data = tcm_get_report_bits(payload, payload_len, offset, bits);
			if (obj < tcm->max_objects)
				objs[obj].width_y = data & 0x0fff;
			offset += bits;
			break;
		case TOUCH_NUM_OF_ACTIVE_OBJECTS:
			if (idx >= cfg_len)
				goto done;
			bits = cfg[idx++];
			active_objects = tcm_get_report_bits(payload, payload_len, offset, bits);
			has_num_active = true;
			offset += bits;
			if (active_objects == 0 && end_of_foreach)
				idx = end_of_foreach;
			break;
		default:
			if (idx >= cfg_len)
				goto done;
			bits = cfg[idx++];
			offset += bits;
			break;
		}
	}

done:
	tcm_report_objects(tcm, objs);
	return 0;
}

static int tcm_handle_touch_report(struct tcm_data *tcm, u8 *buf, size_t len)
{
	struct tcm_touch_object objs[TCM_MAX_OBJECTS] = { 0 };
	const struct tcm_report_point *point;

	/* If the input device hasn't registered yet then we can't do anything */
	if (!tcm->input)
		return 0;

	if (len <= sizeof(struct tcm_message_header))
		return 0;

	buf += sizeof(struct tcm_message_header);
	len -= sizeof(struct tcm_message_header);

	dev_dbg(&tcm->client->dev, "touch report len %zu\n", len);

	if (tcm->report_config_len > 0)
		return tcm_handle_touch_report_dynamic(tcm, buf, len);

	if (len < 1 + sizeof(*point)) {
		tcm_report_objects(tcm, objs);
		return 0;
	}

	if ((len - 3) % sizeof(*point))
		dev_dbg(&tcm->client->dev, "unexpected legacy touch report length %zu\n", len);

	buf++; /* Skip the FPS report */

	for (size_t i = 0; i < (len - 1) / sizeof(*point); i++) {
		u32 _point;
		u16 idx;
		u8 status;

		point = (const struct tcm_report_point *)buf;
		_point = le32_to_cpu(point->point);

		idx = _point & 0xf;
		status = (_point >> 4) & 0xf;
		if (idx < tcm->max_objects) {
			objs[idx].status = status;
			objs[idx].x = (_point >> 8) & 0xfff;
			objs[idx].y = (_point >> 20) & 0xfff;
			objs[idx].z = point->z;
			objs[idx].width_x = point->width_x;
			objs[idx].width_y = point->width_y;
		}

		buf += sizeof(*point);
	}

	tcm_report_objects(tcm, objs);
	return 0;
}

static irqreturn_t tcm_report_irq(int irq, void *data)
{
	struct tcm_data *tcm = data;
	struct tcm_message_header *header;
	u8 *buf = tcm->irq_buf;
	u16 len;
	int ret;

	header = (struct tcm_message_header *)buf;
	ret = tcm_recv_report(tcm, buf, sizeof(tcm->irq_buf));
	if (ret) {
		dev_err(&tcm->client->dev, "failed to read report: %d\n", ret);
		return IRQ_HANDLED;
	}

	if (header->marker != TCM_MESSAGE_MARKER) {
		dev_dbg(&tcm->client->dev, "Ignoring invalid marker %#x\n", header->marker);
		return IRQ_HANDLED;
	}

	switch (header->code) {
	case REPORT_OK:
	case REPORT_IDENTIFY:
	case REPORT_TOUCH:
	case REPORT_DELTA:
	case REPORT_RAW:
	case REPORT_DEBUG:
	case REPORT_TOUCH_HOLD:
	case REPORT_NOT_IMPLEMENTED:
	case REPORT_ERROR:
		break;
	default:
		dev_dbg(&tcm->client->dev, "Ignoring report %#x\n", header->code);
		return IRQ_HANDLED;
	}

	len = le16_to_cpu(header->length);

	dev_dbg(&tcm->client->dev, "report %#x len %u\n", header->code, len);
	print_hex_dump_bytes("report: ", DUMP_PREFIX_OFFSET, buf,
			     min_t(size_t, sizeof(tcm->irq_buf), len + sizeof(*header)));

	if (len > sizeof(tcm->irq_buf) - sizeof(*header)) {
		dev_err(&tcm->client->dev, "report too long (%u)\n", len);
		return IRQ_HANDLED;
	}

	/* Touch indications are always handled directly and must not wake command waiters. */
	if (header->code == REPORT_TOUCH) {
		tcm_handle_touch_report(tcm, buf, len + sizeof(*header));
		return IRQ_HANDLED;
	}

	if (!completion_done(&tcm->response)) {
		tcm->buf_size = len + sizeof(*header);
		memcpy(tcm->buf, buf, len + sizeof(*header));
		complete(&tcm->response);
	}

	return IRQ_HANDLED;
}

static int tcm_hw_init(struct tcm_data *tcm, u16 *max_x, u16 *max_y)
{
	int ret;
	struct tcm_identification id = { 0 };
	struct tcm_app_info app_info = { 0 };
	u8 cfg_resp[sizeof(struct tcm_message_header) + TCM_MAX_REPORT_CONFIG_SIZE];
	u16 cfg_total = 0;
	u16 status;

	/*
	 * Tell the firmware to start up. After starting it sends an IDENTIFY report, which
	 * we treat like a response to this message even though it's technically a new report.
	 */
	ret = tcm_read_message(tcm, TCM_RUN_APPLICATION_FIRMWARE, &id, sizeof(id));
	if (ret) {
		dev_err(&tcm->client->dev, "failed to identify device: %d\n", ret);
		return ret;
	}

	dev_dbg(&tcm->client->dev, "Synaptics TCM %s v%d mode %d\n",
		id.part_number, id.version, id.mode);
	if (id.mode != MODE_APPLICATION) {
		/* We don't support firmware updates or anything else */
		dev_err(&tcm->client->dev, "Device is not in application mode\n");
		return -ENODEV;
	}

	do {
		msleep(20);
		ret = tcm_read_message(tcm, TCM_GET_APPLICATION_INFO, &app_info, sizeof(app_info));
		if (ret) {
			dev_err(&tcm->client->dev, "failed to get application info: %d\n", ret);
			return ret;
		}
		status = le16_to_cpu(app_info.status);
	} while (status == APP_STATUS_BOOTING || status == APP_STATUS_UPDATING);

	dev_dbg(&tcm->client->dev, "Application firmware v%d.%d (customer '%s') status %d\n",
		 app_info.version[0], app_info.version[1], app_info.customer_config_id,
		 status);

	*max_x = le16_to_cpu(app_info.max_x);
	*max_y = le16_to_cpu(app_info.max_y);
	tcm->max_x = *max_x;
	tcm->max_y = *max_y;
	tcm->max_objects = clamp_t(u16, get_unaligned_le16(app_info.max_objects),
				   1, TCM_MAX_OBJECTS);

	ret = tcm_read_message_len(tcm, TCM_GET_TOUCH_REPORT_CONFIG, NULL, 0,
				   cfg_resp, sizeof(cfg_resp), &cfg_total);
	if (!ret && cfg_total > sizeof(struct tcm_message_header)) {
		u16 payload_len = min_t(u16, cfg_total - sizeof(struct tcm_message_header),
					TCM_MAX_REPORT_CONFIG_SIZE);

		memcpy(tcm->report_config,
		       cfg_resp + sizeof(struct tcm_message_header), payload_len);
		tcm->report_config_len = payload_len;
		tcm->report_obj_bits = tcm_calc_obj_bits(tcm->report_config, payload_len);
		dev_info(&tcm->client->dev,
			 "loaded dynamic touch report config (%u bytes, %u bits/obj, %u max_obj)\n",
			 payload_len, tcm->report_obj_bits, tcm->max_objects);
	} else {
		dev_warn(&tcm->client->dev,
			 "failed to fetch touch report config (%d), using legacy layout\n", ret);
	}

	/*
	 * Best-effort disable firmware edge grip / dark zones that cause dead
	 * edges and corners on aftermarket OnePlus 8T replacement panels.
	 */
	if (tcm_set_dynamic_config(tcm, DC_GRIP_ENABLED, 0))
		dev_dbg(&tcm->client->dev, "DC_GRIP_ENABLED not supported\n");
	if (tcm_set_dynamic_config(tcm, DC_DARK_ZONE_ENABLE, 0))
		dev_dbg(&tcm->client->dev, "DC_DARK_ZONE_ENABLE not supported\n");

	return 0;
}

static int tcm_power_on(struct tcm_data *tcm)
{
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(tcm->supplies),
				    tcm->supplies);
	if (ret)
		return ret;

	gpiod_set_value_cansleep(tcm->reset_gpio, 1);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(tcm->reset_gpio, 0);
	usleep_range(80000, 81000);

	return 0;
}

static int tcm_probe(struct i2c_client *client)
{
	struct tcm_data *tcm;
	u16 max_x, max_y;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C |
						I2C_FUNC_SMBUS_BYTE_DATA |
						I2C_FUNC_SMBUS_I2C_BLOCK))
		return -ENODEV;

	tcm = devm_kzalloc(&client->dev, sizeof(struct tcm_data), GFP_KERNEL);
	if (!tcm)
		return -ENOMEM;

	i2c_set_clientdata(client, tcm);
	tcm->client = client;

	init_completion(&tcm->response);

	tcm->supplies[0].supply = "vdd";
	tcm->supplies[1].supply = "vcc";
	ret = devm_regulator_bulk_get(&client->dev, ARRAY_SIZE(tcm->supplies),
				      tcm->supplies);
	if (ret)
		return ret;

	tcm->reset_gpio = devm_gpiod_get(&client->dev, "reset", GPIOD_OUT_LOW);

	ret = devm_add_action_or_reset(&client->dev, tcm_power_off,
				       tcm);
	if (ret)
		return ret;

	ret = tcm_power_on(tcm);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
					tcm_report_irq,
					IRQF_ONESHOT,
					"synaptics_tcm_report", tcm);
	if (ret < 0)
		return ret;

	ret = tcm_hw_init(tcm, &max_x, &max_y);
	if (ret) {
		dev_err(&client->dev, "failed to initialize hardware\n");
		return ret;
	}

	tcm->input = devm_input_allocate_device(&client->dev);
	if (!tcm->input)
		return -ENOMEM;

	tcm->input->name = "Synaptics TCM Oncell Touchscreen";
	tcm->input->id.bustype = BUS_I2C;
	tcm->input->open = tcm_input_open;
	tcm->input->close = tcm_input_close;

	input_set_abs_params(tcm->input, ABS_MT_POSITION_X, 0, max_x, 0, 0);
	input_set_abs_params(tcm->input, ABS_MT_POSITION_Y, 0, max_y, 0, 0);
	input_set_abs_params(tcm->input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(tcm->input, ABS_MT_TOUCH_MINOR, 0, 255, 0, 0);
	input_set_abs_params(tcm->input, ABS_MT_PRESSURE, 0, 255, 0, 0);

	touchscreen_parse_properties(tcm->input, true, &tcm->props);

	ret = input_mt_init_slots(tcm->input, 10, INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (ret)
		return ret;

	input_set_drvdata(tcm->input, tcm);

	ret = input_register_device(tcm->input);
	if (ret)
		return ret;

	return 0;
}

static int tcm_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	disable_irq(client->irq);
	return 0;
}

static int tcm_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	enable_irq(client->irq);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(tcm_pm_ops, tcm_suspend, tcm_resume);

static const struct of_device_id syna_driver_ids[] = {
	{
		.compatible = "syna,s3908",
	},
	{}
};
MODULE_DEVICE_TABLE(of, syna_driver_ids);

static const struct i2c_device_id syna_i2c_ids[] = {
	{ "synaptics-tcm", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, syna_i2c_ids);

static struct i2c_driver syna_i2c_driver = {
	.probe		= tcm_probe,
	.id_table	= syna_i2c_ids,
	.driver		= {
		.name		= "synaptics-tcm",
		.of_match_table	= syna_driver_ids,
		.pm		= pm_sleep_ptr(&tcm_pm_ops),
	},
};

module_i2c_driver(syna_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Frieder Hannenheim <frieder.hannenheim@proton.me>");
MODULE_AUTHOR("Caleb Connolly <caleb@postmarketos.org>");
MODULE_DESCRIPTION("A driver for Synaptics TCM Oncell Touchpanels");
