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
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/proc_fs.h>
#include <linux/property.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/synaptics_tcm_notify.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

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
#define TCM_GET_DYNAMIC_CONFIG			0x23
#define TCM_SET_DYNAMIC_CONFIG			0x24
#define TCM_GET_TOUCH_REPORT_CONFIG		0x25
#define TCM_SET_TOUCH_REPORT_CONFIG		0x26
#define TCM_ENTER_DEEP_SLEEP			0x2c
#define TCM_EXIT_DEEP_SLEEP			0x2d

#define TCM_MESSAGE_MARKER			0xa5

#define MODE_APPLICATION			0x01

#define APP_STATUS_OK				0x00
#define APP_STATUS_BOOTING			0x01
#define APP_STATUS_UPDATING			0x02

/* Dynamic config IDs (matching OPlus 4.19 synaptics_tcm_oncell) */
#define DC_NO_DOZE				0x01
#define DC_IN_WAKEUP_GESTURE_MODE		0x09
#define DC_TOUCH_HOLD				0xd4
#define DC_ERROR_PRIORITY			0xd5
#define DC_NOISE_LENGTH				0xd6
#define DC_DARK_ZONE_ENABLE			0xdd
#define DC_GRIP_ENABLED				0xde
#define DC_SET_REPORT_FRE			0xe6
#define DC_GESTURE_MASK				0xfe

/* Report rates from 4.19 kebab kona-mtp.dtsi: 1=180Hz, 2=120Hz, 3=240Hz */
#define TCM_REPORT_RATE_DEFAULT			1
#define TCM_REPORT_RATE_GAME			3
#define TCM_GAME_NOISE_LENGTH			0x0a

/* OPlus touchscreen gesture IDs & keycodes (KEY_GESTURE_START + id) */
#define KEY_GESTURE_START			246
#define GESTURE_DOUBLE_TAP			1
#define GESTURE_UP_VEE				2
#define GESTURE_DOWN_VEE			3
#define GESTURE_LEFT_VEE			4
#define GESTURE_RIGHT_VEE			5
#define GESTURE_CIRCLE				6
#define GESTURE_DOUBLE_SWIPE			7
#define GESTURE_LEFT_TO_RIGHT			8
#define GESTURE_RIGHT_TO_LEFT			9
#define GESTURE_UP_TO_DOWN			10
#define GESTURE_DOWN_TO_UP			11
#define GESTURE_M				12
#define GESTURE_W				13
#define GESTURE_SINGLE_TAP			16
#define GESTURE_HEART				17
#define GESTURE_S				18
#define GESTURE_MAX_ID				18

/* Synaptics TCM LPWG gesture detect codes */
#define DTAP_DETECT				0x01
#define SWIPE_DETECT				0x02
#define CIRCLE_DETECT				0x04
#define VEE_DETECT				0x05
#define HEART_DETECT				0x07
#define STAP_DETECT				0x10
#define M_UNICODE				0x6d
#define S_UNICODE				0x73
#define W_UNICODE				0x77

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
#define TOUCH_GESTURE_DOUBLE_TAP		0x10
#define TOUCH_NUM_OF_ACTIVE_OBJECTS		0x18
#define TOUCH_REPORT_GESTURE_INFO		198

#define TCM_MAX_OBJECTS				10
#define TCM_RD_CHUNK_SIZE			256
#define TCM_BUF_SIZE				512
#define TCM_DEFAULT_REPORT_CONFIG_SIZE		128
#define TCM_MAX_REPORT_CONFIG_SIZE		256
#define TCM_MAX_TOUCH_WIDTH			200
#define TCM_FALLBACK_MAX_X			1080
#define TCM_FALLBACK_MAX_Y			2400

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
	struct mutex cmd_mutex;
	struct mutex rw_mutex;
	struct touchscreen_properties props;
	struct regulator_bulk_data supplies[2];

	u16 max_x;
	u16 max_y;
	u16 max_objects;

	u8 report_config[TCM_MAX_REPORT_CONFIG_SIZE];
	u16 report_config_len;
	u16 report_obj_bits;

	/* Screen-off gestures & game mode (HTPR) state */
	struct mutex mode_mutex;
	struct work_struct screen_work;
	struct proc_dir_entry *proc_dir;
	struct proc_dir_entry *proc_gesture;
	struct proc_dir_entry *proc_game;
	u32 gesture_mask;
	u16 default_noise_length;
	bool zero_len_cmd_3byte;
	bool report_ctrl_unsupported;
	bool game_mode;
	bool screen_on;
	bool in_gesture_mode;
	bool in_deep_sleep;
	bool irq_wake_enabled;
	bool irq_disabled;
	unsigned long last_gesture_jiffies;

	/* annoying state */
	u16 buf_size;
	u8 buf[TCM_BUF_SIZE];
	u8 irq_buf[TCM_BUF_SIZE];
	u8 chunk_buf[TCM_RD_CHUNK_SIZE];
	u8 cmd_buf[sizeof(struct tcm_cmd) + TCM_MAX_REPORT_CONFIG_SIZE];
};

static struct tcm_data *g_tcm_data;

static int tcm_send_cmd(struct tcm_data *tcm, struct tcm_cmd *cmd)
{
	struct i2c_client *client = tcm->client;
	struct i2c_msg msg;
	u16 payload_len = le16_to_cpu(cmd->length);
	int ret;

	dev_dbg(&client->dev, "sending command %#x len %u\n", cmd->cmd, payload_len);

	/*
	 * Zero-payload TCM commands are sent as 1 byte [cmd] by default (matching
	 * mainline synaptics_s3908 and required by aftermarket OnePlus 8T panels
	 * that ignore 3-byte [cmd 00 00] writes). tcm_hw_init() falls back to
	 * 3-byte headers automatically if a firmware revision requires them.
	 */
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = (payload_len || tcm->zero_len_cmd_3byte) ?
		  (sizeof(*cmd) + payload_len) : 1;
	msg.buf = (u8 *)cmd;

	mutex_lock(&tcm->rw_mutex);
	ret = i2c_transfer(client->adapter, &msg, 1);
	mutex_unlock(&tcm->rw_mutex);
	if (ret == 1)
		return 0;
	else if (ret < 0)
		return ret;
	else
		return -EIO;
}

static int tcm_recv_raw(struct tcm_data *tcm, void *buf, size_t length)
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

static int tcm_read_one_message(struct tcm_data *tcm);

static int tcm_read_message_len(struct tcm_data *tcm, u8 cmd, const void *payload,
				u16 payload_len, void *buf, size_t length, u16 *out_len)
{
	struct tcm_cmd *c;
	const struct tcm_message_header *hdr;
	int ret, attempt;

	if (payload_len > TCM_MAX_REPORT_CONFIG_SIZE)
		return -EINVAL;

	mutex_lock(&tcm->cmd_mutex);

	c = (struct tcm_cmd *)tcm->cmd_buf;
	c->cmd = cmd;
	c->length = cpu_to_le16(payload_len);
	if (payload_len && payload)
		memcpy(c->data, payload, payload_len);

	reinit_completion(&tcm->response);
	ret = tcm_send_cmd(tcm, c);
	if (ret) {
		mutex_unlock(&tcm->cmd_mutex);
		return ret;
	}

	ret = wait_for_completion_timeout(&tcm->response, msecs_to_jiffies(300));
	if (ret == 0) {
		/*
		 * Fallback poll if ATTN was missed or level-latched during init.
		 */
		for (attempt = 0; attempt < 10 && !completion_done(&tcm->response); attempt++) {
			msleep(20);
			tcm_read_one_message(tcm);
		}
		if (!completion_done(&tcm->response)) {
			mutex_unlock(&tcm->cmd_mutex);
			return -ETIMEDOUT;
		}
	}

	hdr = (const struct tcm_message_header *)tcm->buf;
	if (hdr->code != REPORT_OK && hdr->code != REPORT_IDENTIFY) {
		dev_dbg(&tcm->client->dev, "command %#x returned status %#x\n",
			cmd, hdr->code);
		mutex_unlock(&tcm->cmd_mutex);
		return -EIO;
	}

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

	mutex_unlock(&tcm->cmd_mutex);
	return 0;
}

static int tcm_read_message(struct tcm_data *tcm, u8 cmd, void *buf, size_t length)
{
	return tcm_read_message_len(tcm, cmd, NULL, 0, buf, length, NULL);
}

static int tcm_get_dynamic_config(struct tcm_data *tcm, u8 id, u16 *val)
{
	u8 resp[sizeof(struct tcm_message_header) + 2] = { 0 };
	u16 out_len = 0;
	int ret;

	ret = tcm_read_message_len(tcm, TCM_GET_DYNAMIC_CONFIG, &id, sizeof(id),
				   resp, sizeof(resp), &out_len);
	if (ret)
		return ret;
	if (out_len < sizeof(resp))
		return -EIO;

	*val = get_unaligned_le16(resp + sizeof(struct tcm_message_header));
	return 0;
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

static int tcm_set_game_mode_locked(struct tcm_data *tcm, bool enable)
{
	u16 err_prio = 0;
	int ret;

	ret = tcm_get_dynamic_config(tcm, DC_ERROR_PRIORITY, &err_prio);
	if (ret) {
		dev_dbg(&tcm->client->dev, "failed to read DC_ERROR_PRIORITY: %d\n", ret);
		err_prio = 0;
	}

	if (enable) {
		tcm_set_dynamic_config(tcm, DC_ERROR_PRIORITY, err_prio | 0x01);
		tcm_set_dynamic_config(tcm, DC_NOISE_LENGTH, TCM_GAME_NOISE_LENGTH);
		ret = tcm_set_dynamic_config(tcm, DC_SET_REPORT_FRE, TCM_REPORT_RATE_GAME);
	} else {
		tcm_set_dynamic_config(tcm, DC_ERROR_PRIORITY, err_prio & 0xf0);
		if (tcm->default_noise_length)
			tcm_set_dynamic_config(tcm, DC_NOISE_LENGTH, tcm->default_noise_length);
		ret = tcm_set_dynamic_config(tcm, DC_SET_REPORT_FRE, TCM_REPORT_RATE_DEFAULT);
	}

	if (!ret)
		tcm->game_mode = enable;
	return ret;
}

static int tcm_enable_report(struct tcm_data *tcm, u8 report_type, bool enable)
{
	u8 payload = report_type;
	int ret;

	if (tcm->report_ctrl_unsupported)
		return 0;

	ret = tcm_read_message_len(tcm,
				   enable ? TCM_ENABLE_REPORT : TCM_DISABLE_REPORT,
				   &payload, sizeof(payload), NULL, 0, NULL);
	if (ret == -ETIMEDOUT || ret == -EIO)
		tcm->report_ctrl_unsupported = true;
	return ret;
}

static void tcm_power_off(void *data)
{
	struct tcm_data *tcm = data;

	gpiod_set_value_cansleep(tcm->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(tcm->supplies), tcm->supplies);
}

static int tcm_input_open(struct input_dev *dev)
{
	struct tcm_data *tcm = input_get_drvdata(dev);
	int ret;

	ret = tcm_enable_report(tcm, REPORT_TOUCH, true);
	if (ret)
		dev_warn(&tcm->client->dev, "failed to enable touch report: %d\n", ret);
	return 0;
}

static void tcm_input_close(struct input_dev *dev)
{
	struct tcm_data *tcm = input_get_drvdata(dev);
	int ret;

	ret = tcm_enable_report(tcm, REPORT_TOUCH, false);
	if (ret)
		dev_dbg(&tcm->client->dev, "failed to disable touch report: %d\n", ret);
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
		if (!major_width)
			major_width = 10;
		if (!minor_width)
			minor_width = major_width;

		dev_dbg(&tcm->client->dev, "touch report: idx %u x %u y %u\n",
			idx, objs[idx].x, objs[idx].y);

		input_mt_slot(tcm->input, idx);
		input_mt_report_slot_state(tcm->input, MT_TOOL_FINGER, true);

		touchscreen_report_pos(tcm->input, &tcm->props,
				       objs[idx].x, objs[idx].y, true);

		input_report_abs(tcm->input, ABS_MT_TOUCH_MAJOR, major_width);
		input_report_abs(tcm->input, ABS_MT_TOUCH_MINOR, minor_width);
		input_report_abs(tcm->input, ABS_MT_PRESSURE,
				 objs[idx].z ? objs[idx].z : major_width);
	}

	input_mt_sync_frame(tcm->input);
	input_sync(tcm->input);
}

static void tcm_handle_gesture_report(struct tcm_data *tcm, const u8 *payload, size_t payload_len)
{
	u8 lpwg;
	const u8 *extra;
	int gesture_id = 0;

	if (payload_len < 7)
		return;

	lpwg = payload[0];
	extra = &payload[1];

	switch (lpwg) {
	case DTAP_DETECT:
		gesture_id = GESTURE_DOUBLE_TAP;
		break;
	case STAP_DETECT:
		if (tcm->gesture_mask & BIT(GESTURE_SINGLE_TAP))
			gesture_id = GESTURE_SINGLE_TAP;
		else
			gesture_id = GESTURE_DOUBLE_TAP;
		break;
	case CIRCLE_DETECT:
		gesture_id = GESTURE_CIRCLE;
		break;
	case SWIPE_DETECT:
		switch (extra[4]) {
		case 0x41:
			gesture_id = GESTURE_LEFT_TO_RIGHT;
			break;
		case 0x42:
			gesture_id = GESTURE_RIGHT_TO_LEFT;
			break;
		case 0x44:
			gesture_id = GESTURE_UP_TO_DOWN;
			break;
		case 0x48:
			gesture_id = GESTURE_DOWN_TO_UP;
			break;
		case 0x81:
		case 0x82:
		case 0x84:
		case 0x88:
			gesture_id = GESTURE_DOUBLE_SWIPE;
			break;
		default:
			break;
		}
		break;
	case VEE_DETECT:
		switch (extra[2]) {
		case 0x02:
			gesture_id = GESTURE_UP_VEE;
			break;
		case 0x01:
			gesture_id = GESTURE_DOWN_VEE;
			break;
		case 0x08:
			gesture_id = GESTURE_LEFT_VEE;
			break;
		case 0x04:
			gesture_id = GESTURE_RIGHT_VEE;
			break;
		default:
			break;
		}
		break;
	case M_UNICODE:
		gesture_id = GESTURE_M;
		break;
	case W_UNICODE:
		gesture_id = GESTURE_W;
		break;
	case S_UNICODE:
		gesture_id = GESTURE_S;
		break;
	case HEART_DETECT:
		gesture_id = GESTURE_HEART;
		break;
	default:
		break;
	}

	if (gesture_id > 0 && gesture_id <= GESTURE_MAX_ID &&
	    (tcm->gesture_mask & BIT(gesture_id))) {
		unsigned int key = (gesture_id == GESTURE_DOUBLE_TAP) ?
				   KEY_WAKEUP : (KEY_GESTURE_START + gesture_id);

		if (gesture_id == GESTURE_DOUBLE_TAP &&
		    time_before(jiffies, tcm->last_gesture_jiffies + msecs_to_jiffies(100)))
			return;
		tcm->last_gesture_jiffies = jiffies;

		pm_wakeup_event(&tcm->client->dev, 1000);
		dev_info(&tcm->client->dev, "gesture detected: lpwg=%#x id=%d key=%u\n",
			 lpwg, gesture_id, key);
		input_report_key(tcm->input, key, 1);
		input_sync(tcm->input);
		input_report_key(tcm->input, key, 0);
		input_sync(tcm->input);
	}
}

static void tcm_strip_continued_markers(u8 *buf, size_t start,
					size_t msg_len, size_t avail_len)
{
	size_t i = start;

	if (avail_len < msg_len)
		avail_len = msg_len;

	while (i + 1 < msg_len && i + 2 <= avail_len) {
		if (buf[i] == TCM_MESSAGE_MARKER &&
		    buf[i + 1] == REPORT_CONTINUED_READ) {
			memmove(&buf[i], &buf[i + 2], avail_len - (i + 2));
			memset(&buf[avail_len - 2], 0, 2);
			continue;
		}
		i++;
	}
}

static int tcm_handle_touch_report_dynamic(struct tcm_data *tcm,
					   u8 *payload, size_t payload_len,
					   size_t avail_payload_len)
{
	struct tcm_touch_object objs[TCM_MAX_OBJECTS] = { 0 };
	const u8 *cfg = tcm->report_config;
	u16 cfg_len = tcm->report_config_len;
	unsigned int idx = 0, next = 0, end_of_foreach = 0;
	unsigned int offset = 0, obj = 0, objects = 0, active_objects = 0;
	unsigned int min_obj_bits = min_t(unsigned int, tcm->report_obj_bits, 56);
	bool active_only = false, has_num_active = false;
	size_t avail = max(payload_len, avail_payload_len);

	if (payload_len >= 9 &&
	    ((payload[0] == REPORT_CONTINUED_READ && payload[1] == TCM_MESSAGE_MARKER) ||
	     (payload[0] == TCM_MESSAGE_MARKER && payload[1] == REPORT_CONTINUED_READ))) {
		memmove(&payload[0], &payload[2], avail - 2);
		memset(&payload[avail - 2], 0, 2);
	}

	if (tcm->in_gesture_mode) {
		if (payload_len >= 7 && payload[0] != 0)
			tcm_handle_gesture_report(tcm, payload, payload_len);
		return 0;
	}

	/*
	 * Aftermarket replacement OnePlus 8T panels may insert a 2-byte
	 * 0x03 0xa5 or 0xa5 0x03 transient prefix at bytes [0..1], [5..6]
	 * (shifting Object 0 from byte 7 to byte 9), or [8..9] when
	 * (payload[7] & 0x0f) == 0. Shift across the full read chunk so
	 * trailing width bytes of Object 0 are recovered instead of zeroed.
	 */
	if (payload_len >= 12 && payload[5] == TCM_MESSAGE_MARKER &&
	    payload[6] == REPORT_CONTINUED_READ &&
	    payload[7] == 0x00 && payload[8] == 0x00) {
		memmove(&payload[5], &payload[7], avail - 7);
		memset(&payload[avail - 2], 0, 2);
	} else if (payload_len >= 10 && (payload[7] & 0x0f) == 0x00 &&
		   payload[8] == TCM_MESSAGE_MARKER &&
		   payload[9] == REPORT_CONTINUED_READ) {
		memmove(&payload[8], &payload[10], avail - 10);
		memset(&payload[avail - 2], 0, 2);
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

static int tcm_handle_touch_report(struct tcm_data *tcm, u8 *buf,
				   size_t len, size_t avail_len)
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
	avail_len = (avail_len > sizeof(struct tcm_message_header)) ?
		    (avail_len - sizeof(struct tcm_message_header)) : len;

	dev_dbg(&tcm->client->dev, "touch report len %zu\n", len);

	if (tcm->report_config_len > 0)
		return tcm_handle_touch_report_dynamic(tcm, buf, len, avail_len);

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

static int tcm_read_one_message(struct tcm_data *tcm)
{
	struct tcm_message_header *header;
	u8 *buf = tcm->irq_buf;
	u16 len;
	size_t total_len, offset;
	int ret;

	mutex_lock(&tcm->rw_mutex);

	header = (struct tcm_message_header *)buf;
	ret = tcm_recv_raw(tcm, buf, TCM_RD_CHUNK_SIZE);
	if (ret) {
		mutex_unlock(&tcm->rw_mutex);
		dev_err(&tcm->client->dev, "failed to read report: %d\n", ret);
		return ret;
	}

	if (header->marker != TCM_MESSAGE_MARKER) {
		mutex_unlock(&tcm->rw_mutex);
		dev_dbg(&tcm->client->dev, "Ignoring invalid marker %#x\n", header->marker);
		return 0;
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
		mutex_unlock(&tcm->rw_mutex);
		dev_dbg(&tcm->client->dev, "Ignoring report %#x\n", header->code);
		return 0;
	}

	len = le16_to_cpu(header->length);

	dev_dbg(&tcm->client->dev, "report %#x len %u\n", header->code, len);

	if (len > sizeof(tcm->irq_buf) - sizeof(*header)) {
		mutex_unlock(&tcm->rw_mutex);
		dev_err(&tcm->client->dev, "report too long (%u)\n", len);
		return -EMSGSIZE;
	}

	/* Reassemble payloads that span beyond the 256-byte chunk size */
	total_len = sizeof(*header) + len;
	offset = TCM_RD_CHUNK_SIZE;
	while (offset < total_len) {
		size_t remain = total_len - offset;
		size_t xfer = min_t(size_t, remain, TCM_RD_CHUNK_SIZE - 2);

		ret = tcm_recv_raw(tcm, tcm->chunk_buf, xfer + 2);
		if (ret) {
			mutex_unlock(&tcm->rw_mutex);
			dev_err(&tcm->client->dev, "failed continued read: %d\n", ret);
			return ret;
		}
		if (tcm->chunk_buf[0] != TCM_MESSAGE_MARKER ||
		    tcm->chunk_buf[1] != REPORT_CONTINUED_READ) {
			mutex_unlock(&tcm->rw_mutex);
			dev_dbg(&tcm->client->dev,
				"unexpected continued read header %#x %#x\n",
				tcm->chunk_buf[0], tcm->chunk_buf[1]);
			return -EIO;
		}
		memcpy(buf + offset, tcm->chunk_buf + 2, xfer);
		offset += xfer;
	}

	mutex_unlock(&tcm->rw_mutex);

	print_hex_dump_bytes("report: ", DUMP_PREFIX_OFFSET, buf,
			     min_t(size_t, sizeof(tcm->irq_buf), total_len));

	/*
	 * Single-transaction reads (TCM_RD_CHUNK_SIZE = 256) already hold the
	 * extra bytes shifted out by any injected 0xa5 0x03 marker.
	 */
	offset = (total_len <= TCM_RD_CHUNK_SIZE) ? TCM_RD_CHUNK_SIZE : total_len;

	/* Touch indications are always handled directly and must not wake command waiters. */
	if (header->code == REPORT_TOUCH) {
		tcm_handle_touch_report(tcm, buf, total_len, offset);
		return 0;
	}

	/* Only command responses and IDENTIFY reports should wake command waiters. */
	if (header->code != REPORT_OK && header->code != REPORT_IDENTIFY &&
	    header->code != REPORT_NOT_IMPLEMENTED && header->code != REPORT_ERROR)
		return 0;

	if (total_len > sizeof(*header))
		tcm_strip_continued_markers(buf, sizeof(*header), total_len, offset);

	if (!completion_done(&tcm->response)) {
		tcm->buf_size = total_len;
		memcpy(tcm->buf, buf, total_len);
		complete(&tcm->response);
	}

	return 0;
}

static irqreturn_t tcm_report_irq(int irq, void *data)
{
	struct tcm_data *tcm = data;

	tcm_read_one_message(tcm);
	return IRQ_HANDLED;
}

static const u8 tcm_normal_report_config[] = {
	TOUCH_GESTURE_DOUBLE_TAP, 8,
	TOUCH_REPORT_GESTURE_INFO, 48,
	TOUCH_FOREACH_ACTIVE_OBJECT,
	TOUCH_OBJECT_N_INDEX, 4,
	TOUCH_OBJECT_N_CLASSIFICATION, 4,
	TOUCH_OBJECT_N_X_POSITION, 12,
	TOUCH_OBJECT_N_Y_POSITION, 12,
	TOUCH_OBJECT_N_X_WIDTH, 12,
	TOUCH_OBJECT_N_Y_WIDTH, 12,
	TOUCH_FOREACH_END,
	TOUCH_END,
};

static void tcm_program_normal_config_locked(struct tcm_data *tcm, u16 cfg_max_len)
{
	u8 cfg_buf[sizeof(struct tcm_message_header) + TCM_MAX_REPORT_CONFIG_SIZE] = { 0 };
	u16 cfg_total = 0;
	int ret;

	if (cfg_max_len < TCM_DEFAULT_REPORT_CONFIG_SIZE)
		cfg_max_len = TCM_DEFAULT_REPORT_CONFIG_SIZE;
	cfg_max_len = clamp_t(u16, cfg_max_len, sizeof(tcm_normal_report_config),
			      TCM_MAX_REPORT_CONFIG_SIZE);
	memcpy(cfg_buf, tcm_normal_report_config, sizeof(tcm_normal_report_config));

	ret = tcm_read_message_len(tcm, TCM_SET_TOUCH_REPORT_CONFIG,
				   cfg_buf, cfg_max_len, NULL, 0, NULL);
	if (!ret) {
		memcpy(tcm->report_config, tcm_normal_report_config,
		       sizeof(tcm_normal_report_config));
		tcm->report_config_len = sizeof(tcm_normal_report_config);
		tcm->report_obj_bits = tcm_calc_obj_bits(tcm->report_config,
							 tcm->report_config_len);
		dev_info(&tcm->client->dev,
			 "programmed normal touch report config (%u bytes, %u bits/obj, %u max_obj)\n",
			 tcm->report_config_len, tcm->report_obj_bits, tcm->max_objects);
		return;
	}

	dev_warn(&tcm->client->dev,
		 "CMD_SET_TOUCH_REPORT_CONFIG failed (%d), querying device config\n", ret);
	ret = tcm_read_message_len(tcm, TCM_GET_TOUCH_REPORT_CONFIG, NULL, 0,
				   cfg_buf, sizeof(cfg_buf), &cfg_total);
	if (!ret && cfg_total > sizeof(struct tcm_message_header)) {
		u16 payload_len = min_t(u16, cfg_total - sizeof(struct tcm_message_header),
					TCM_MAX_REPORT_CONFIG_SIZE);

		memcpy(tcm->report_config,
		       cfg_buf + sizeof(struct tcm_message_header), payload_len);
		tcm->report_config_len = payload_len;
		tcm->report_obj_bits = tcm_calc_obj_bits(tcm->report_config, payload_len);
		dev_info(&tcm->client->dev,
			 "loaded device touch report config (%u bytes, %u bits/obj, %u max_obj)\n",
			 payload_len, tcm->report_obj_bits, tcm->max_objects);
	} else {
		memcpy(tcm->report_config, tcm_normal_report_config,
		       sizeof(tcm_normal_report_config));
		tcm->report_config_len = sizeof(tcm_normal_report_config);
		tcm->report_obj_bits = tcm_calc_obj_bits(tcm->report_config,
							 tcm->report_config_len);
		dev_warn(&tcm->client->dev,
			 "failed to fetch touch report config (%d), using normal layout\n",
			 ret);
	}
}

static void tcm_activate_normal_mode_locked(struct tcm_data *tcm, bool reprogram_cfg)
{
	int sleep_ret, freq_ret;

	/*
	 * Always wake the controller out of deep sleep and gesture mode on
	 * init and screen-on: panel DDIC resets or aftermarket power-on state
	 * can leave the on-cell IC asleep without setting software flags.
	 */
	sleep_ret = tcm_read_message_len(tcm, TCM_EXIT_DEEP_SLEEP, NULL, 0, NULL, 0, NULL);
	if (sleep_ret == -ETIMEDOUT) {
		tcm->zero_len_cmd_3byte = !tcm->zero_len_cmd_3byte;
		sleep_ret = tcm_read_message_len(tcm, TCM_EXIT_DEEP_SLEEP,
						 NULL, 0, NULL, 0, NULL);
		if (sleep_ret)
			tcm->zero_len_cmd_3byte = !tcm->zero_len_cmd_3byte;
		else
			dev_info(&tcm->client->dev,
				 "switched zero-len cmd header to %u-byte on EXIT_DEEP_SLEEP\n",
				 tcm->zero_len_cmd_3byte ? 3 : 1);
	}
	tcm->in_deep_sleep = false;
	msleep(20);

	if (reprogram_cfg)
		tcm_program_normal_config_locked(tcm, TCM_DEFAULT_REPORT_CONFIG_SIZE);

	tcm->in_gesture_mode = false;
	tcm_set_dynamic_config(tcm, DC_IN_WAKEUP_GESTURE_MODE, 0);
	if (!tcm->gesture_mask)
		tcm_set_dynamic_config(tcm, DC_GESTURE_MASK, 0);
	tcm_set_dynamic_config(tcm, DC_TOUCH_HOLD, 0);

	if (tcm->game_mode) {
		freq_ret = tcm_set_game_mode_locked(tcm, true);
	} else {
		freq_ret = tcm_set_dynamic_config(tcm, DC_SET_REPORT_FRE,
						  TCM_REPORT_RATE_DEFAULT);
		if (freq_ret)
			freq_ret = tcm_set_dynamic_config(tcm, DC_SET_REPORT_FRE, 2);
	}

	tcm_enable_report(tcm, REPORT_TOUCH, true);
	dev_info(&tcm->client->dev,
		 "active normal mode set (3byte=%d sleep_exit=%d freq_ret=%d)\n",
		 tcm->zero_len_cmd_3byte, sleep_ret, freq_ret);
}

static int tcm_hw_init(struct tcm_data *tcm, u16 *max_x, u16 *max_y)
{
	int ret = 0, retry;
	struct tcm_identification id = { 0 };
	struct tcm_app_info app_info = { 0 };
	u16 cfg_max_len = 0;
	u16 status = APP_STATUS_OK;
	u32 dt_x = 0, dt_y = 0;
	bool got_id = false;
	bool got_app_info = false;

	/*
	 * Prefer the automatic power-on REPORT_IDENTIFY captured when reset
	 * was released with the IRQ already armed (matching OPlus
	 * synaptics_tcm_oncell). Only send explicit TCM_IDENTIFY (0x02) /
	 * TCM_RUN_APPLICATION_FIRMWARE (0x14) if no boot identify arrived.
	 */
	if (completion_done(&tcm->response) &&
	    tcm->buf_size >= sizeof(struct tcm_message_header) + 2 &&
	    tcm->buf[0] == TCM_MESSAGE_MARKER &&
	    tcm->buf[1] == REPORT_IDENTIFY) {
		memcpy(&id, tcm->buf, min_t(size_t, sizeof(id), tcm->buf_size));
		got_id = true;
		dev_info(&tcm->client->dev,
			 "captured power-on REPORT_IDENTIFY (%.16s v%d mode %#x)\n",
			 id.part_number, id.version, id.mode);
	}

	if (!got_id) {
		for (retry = 0; retry < 2; retry++) {
			tcm->zero_len_cmd_3byte = (retry == 1);
			memset(&id, 0, sizeof(id));

			ret = tcm_read_message(tcm, TCM_IDENTIFY, &id, sizeof(id));
			if (!ret) {
				got_id = true;
				break;
			}
			if (ret != -ETIMEDOUT && ret != -EIO)
				return ret;

			dev_warn(&tcm->client->dev,
				 "TCM_IDENTIFY (%u-byte) failed (%d), trying RUN_APPLICATION_FIRMWARE\n",
				 tcm->zero_len_cmd_3byte ? 3 : 1, ret);
			ret = tcm_read_message(tcm, TCM_RUN_APPLICATION_FIRMWARE,
					       &id, sizeof(id));
			if (!ret) {
				got_id = true;
				break;
			}
			if (ret != -ETIMEDOUT && ret != -EIO)
				return ret;
		}
	}

	if (!got_id) {
		/*
		 * Some aftermarket replacement panels ACK I2C transfers cleanly
		 * and boot directly into application mode, but silently drop
		 * zero-payload 0x02/0x14 commands. Match OPlus syna_get_chip_info()
		 * by continuing into application setup instead of aborting probe.
		 */
		dev_warn(&tcm->client->dev,
			 "identify commands timed out (%d); assuming application mode\n",
			 ret);
		tcm->zero_len_cmd_3byte = false;
		id.mode = MODE_APPLICATION;
	} else {
		if (id.mode == 0) {
			/*
			 * Some panels answer TCM_RUN_APPLICATION_FIRMWARE with
			 * an empty REPORT_OK when already in application mode;
			 * query TCM_IDENTIFY to populate part_number/mode.
			 */
			msleep(20);
			if (tcm_read_message(tcm, TCM_IDENTIFY, &id, sizeof(id)))
				id.mode = MODE_APPLICATION;
		}

		if (id.mode != MODE_APPLICATION) {
			dev_info(&tcm->client->dev,
				 "controller in mode %#x, switching to application firmware\n",
				 id.mode);
			ret = tcm_read_message(tcm, TCM_RUN_APPLICATION_FIRMWARE, NULL, 0);
			if (ret) {
				dev_err(&tcm->client->dev,
					"failed to run application firmware: %d\n", ret);
				return ret;
			}
			msleep(20);
			memset(&id, 0, sizeof(id));
			ret = tcm_read_message(tcm, TCM_IDENTIFY, &id, sizeof(id));
			if (ret) {
				dev_err(&tcm->client->dev,
					"failed post-switch identify: %d\n", ret);
				return ret;
			}
		}

		dev_info(&tcm->client->dev, "Synaptics TCM %.16s v%d mode %#x\n",
			 id.part_number, id.version, id.mode);
		if (id.mode != MODE_APPLICATION) {
			/* We don't support firmware updates or anything else */
			dev_err(&tcm->client->dev, "Device is not in application mode\n");
			return -ENODEV;
		}
	}

	retry = 0;
	do {
		msleep(20);
		ret = tcm_read_message(tcm, TCM_GET_APPLICATION_INFO,
				       &app_info, sizeof(app_info));
		if (ret == -ETIMEDOUT && retry == 0) {
			tcm->zero_len_cmd_3byte = !tcm->zero_len_cmd_3byte;
			ret = tcm_read_message(tcm, TCM_GET_APPLICATION_INFO,
					       &app_info, sizeof(app_info));
			if (ret)
				tcm->zero_len_cmd_3byte = !tcm->zero_len_cmd_3byte;
		}
		if (ret) {
			if (ret == -ETIMEDOUT) {
				dev_warn(&tcm->client->dev,
					 "TCM_GET_APPLICATION_INFO timed out; using DT/fallback dimensions\n");
				break;
			}
			dev_err(&tcm->client->dev, "failed to get application info: %d\n", ret);
			return ret;
		}
		got_app_info = true;
		status = le16_to_cpu(app_info.status);
	} while ((status == APP_STATUS_BOOTING || status == APP_STATUS_UPDATING) &&
		 ++retry < 25);

	if (got_app_info) {
		dev_info(&tcm->client->dev,
			 "Application firmware v%d.%d (customer '%.16s') status %d max=%ux%u (%u-byte cmd)\n",
			 app_info.version[0], app_info.version[1],
			 app_info.customer_config_id, status,
			 le16_to_cpu(app_info.max_x), le16_to_cpu(app_info.max_y),
			 tcm->zero_len_cmd_3byte ? 3 : 1);
		*max_x = le16_to_cpu(app_info.max_x);
		*max_y = le16_to_cpu(app_info.max_y);
		tcm->max_objects = clamp_t(u16, get_unaligned_le16(app_info.max_objects),
					   1, TCM_MAX_OBJECTS);
		cfg_max_len = get_unaligned_le16(app_info.max_touch_report_config_size);
	} else {
		*max_x = 0;
		*max_y = 0;
		tcm->max_objects = TCM_MAX_OBJECTS;
		cfg_max_len = TCM_DEFAULT_REPORT_CONFIG_SIZE;
	}

	device_property_read_u32(&tcm->client->dev, "touchscreen-size-x", &dt_x);
	device_property_read_u32(&tcm->client->dev, "touchscreen-size-y", &dt_y);
	if (*max_x <= 1 || *max_x > 4096 || *max_y <= 1 || *max_y > 4096 ||
	    *max_x >= *max_y) {
		*max_x = dt_x ? dt_x : TCM_FALLBACK_MAX_X;
		*max_y = dt_y ? dt_y : TCM_FALLBACK_MAX_Y;
	}
	tcm->max_x = max_t(u16, *max_x, dt_x ? dt_x : TCM_FALLBACK_MAX_X);
	tcm->max_y = max_t(u16, *max_y, dt_y ? dt_y : TCM_FALLBACK_MAX_Y);

	mutex_lock(&tcm->mode_mutex);
	/*
	 * Program report config with firmware's reported size first (matching
	 * OPlus syna_set_normal_report_config) and transition controller out
	 * of deep sleep / gesture mode into active normal reporting.
	 */
	tcm_program_normal_config_locked(tcm, cfg_max_len);
	tcm_activate_normal_mode_locked(tcm, false);
	mutex_unlock(&tcm->mode_mutex);

	/*
	 * Best-effort disable firmware edge grip / dark zones / dozing that
	 * cause dead edges, corners, or wake latency on aftermarket panels.
	 */
	if (tcm_set_dynamic_config(tcm, DC_GRIP_ENABLED, 0))
		dev_dbg(&tcm->client->dev, "DC_GRIP_ENABLED not supported\n");
	if (tcm_set_dynamic_config(tcm, DC_DARK_ZONE_ENABLE, 0))
		dev_dbg(&tcm->client->dev, "DC_DARK_ZONE_ENABLE not supported\n");
	if (tcm_set_dynamic_config(tcm, DC_NO_DOZE, 1))
		dev_dbg(&tcm->client->dev, "DC_NO_DOZE not supported\n");
	if (tcm_get_dynamic_config(tcm, DC_NOISE_LENGTH, &tcm->default_noise_length))
		tcm->default_noise_length = 0;

	return 0;
}

static void tcm_screen_work_fn(struct work_struct *work)
{
	struct tcm_data *tcm = container_of(work, struct tcm_data, screen_work);
	bool on = READ_ONCE(tcm->screen_on);

	mutex_lock(&tcm->mode_mutex);
	if (!on) {
		if (tcm->gesture_mask != 0) {
			if (tcm->in_deep_sleep) {
				tcm_read_message_len(tcm, TCM_EXIT_DEEP_SLEEP, NULL, 0,
						     NULL, 0, NULL);
				tcm->in_deep_sleep = false;
			}
			tcm_enable_report(tcm, REPORT_TOUCH, true);
			tcm_set_dynamic_config(tcm, DC_IN_WAKEUP_GESTURE_MODE, 1);
			tcm_set_dynamic_config(tcm, DC_GESTURE_MASK, 0xffff);
			tcm->in_gesture_mode = true;
			dev_info(&tcm->client->dev,
				 "entered wakeup gesture mode (mask=0x%x)\n",
				 tcm->gesture_mask);
		} else {
			tcm->in_gesture_mode = false;
			if (!tcm->in_deep_sleep) {
				tcm_read_message_len(tcm, TCM_ENTER_DEEP_SLEEP, NULL, 0,
						     NULL, 0, NULL);
				tcm->in_deep_sleep = true;
			}
		}
	} else {
		tcm_activate_normal_mode_locked(tcm, true);
	}
	mutex_unlock(&tcm->mode_mutex);
}

void synaptics_tcm_notify_screen_state(bool on)
{
	struct tcm_data *tcm = READ_ONCE(g_tcm_data);

	if (!tcm)
		return;
	WRITE_ONCE(tcm->screen_on, on);
	schedule_work(&tcm->screen_work);
}
EXPORT_SYMBOL_GPL(synaptics_tcm_notify_screen_state);

static ssize_t tcm_proc_gesture_read(struct file *file, char __user *buf,
				     size_t count, loff_t *ppos)
{
	struct tcm_data *tcm = pde_data(file_inode(file));
	char tmp[16];
	int len;

	if (!tcm)
		return -ENODEV;
	len = scnprintf(tmp, sizeof(tmp), "%x\n", tcm->gesture_mask);
	return simple_read_from_buffer(buf, count, ppos, tmp, len);
}

static ssize_t tcm_proc_gesture_write(struct file *file, const char __user *buf,
				      size_t count, loff_t *ppos)
{
	struct tcm_data *tcm = pde_data(file_inode(file));
	char kbuf[16];
	u32 val;
	int ret;

	if (!tcm)
		return -ENODEV;
	if (count == 0 || count >= sizeof(kbuf))
		return -EINVAL;
	if (copy_from_user(kbuf, buf, count))
		return -EFAULT;
	kbuf[count] = '\0';

	ret = kstrtou32(strim(kbuf), 0, &val);
	if (ret)
		return ret;

	mutex_lock(&tcm->mode_mutex);
	tcm->gesture_mask = val;
	mutex_unlock(&tcm->mode_mutex);

	if (!READ_ONCE(tcm->screen_on))
		schedule_work(&tcm->screen_work);

	return count;
}

static const struct proc_ops tcm_proc_gesture_ops = {
	.proc_read = tcm_proc_gesture_read,
	.proc_write = tcm_proc_gesture_write,
};

static ssize_t tcm_proc_game_read(struct file *file, char __user *buf,
				  size_t count, loff_t *ppos)
{
	struct tcm_data *tcm = pde_data(file_inode(file));
	char tmp[8];
	int len;

	if (!tcm)
		return -ENODEV;
	len = scnprintf(tmp, sizeof(tmp), "%d\n", tcm->game_mode ? 1 : 0);
	return simple_read_from_buffer(buf, count, ppos, tmp, len);
}

static ssize_t tcm_proc_game_write(struct file *file, const char __user *buf,
				   size_t count, loff_t *ppos)
{
	struct tcm_data *tcm = pde_data(file_inode(file));
	char kbuf[8];
	u32 val;
	int ret;

	if (!tcm)
		return -ENODEV;
	if (count == 0 || count >= sizeof(kbuf))
		return -EINVAL;
	if (copy_from_user(kbuf, buf, count))
		return -EFAULT;
	kbuf[count] = '\0';

	ret = kstrtou32(strim(kbuf), 0, &val);
	if (ret)
		return ret;

	mutex_lock(&tcm->mode_mutex);
	if (READ_ONCE(tcm->screen_on))
		ret = tcm_set_game_mode_locked(tcm, val != 0);
	else
		tcm->game_mode = (val != 0);
	mutex_unlock(&tcm->mode_mutex);

	return ret ? ret : count;
}

static const struct proc_ops tcm_proc_game_ops = {
	.proc_read = tcm_proc_game_read,
	.proc_write = tcm_proc_game_write,
};

static void tcm_proc_cleanup(void *data)
{
	struct tcm_data *tcm = data;

	if (READ_ONCE(g_tcm_data) == tcm)
		WRITE_ONCE(g_tcm_data, NULL);
	cancel_work_sync(&tcm->screen_work);
	if (tcm->proc_gesture) {
		proc_remove(tcm->proc_gesture);
		tcm->proc_gesture = NULL;
	}
	if (tcm->proc_game) {
		proc_remove(tcm->proc_game);
		tcm->proc_game = NULL;
	}
	if (tcm->proc_dir) {
		proc_remove(tcm->proc_dir);
		tcm->proc_dir = NULL;
	}
}

static int tcm_probe(struct i2c_client *client)
{
	struct tcm_data *tcm;
	u16 max_x, max_y;
	int i, ret;

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
	mutex_init(&tcm->cmd_mutex);
	mutex_init(&tcm->rw_mutex);
	mutex_init(&tcm->mode_mutex);
	INIT_WORK(&tcm->screen_work, tcm_screen_work_fn);
	tcm->screen_on = true;

	tcm->supplies[0].supply = "vdd";
	tcm->supplies[1].supply = "vcc";
	ret = devm_regulator_bulk_get(&client->dev, ARRAY_SIZE(tcm->supplies),
				      tcm->supplies);
	if (ret)
		return ret;

	/* Hold controller in reset while enabling supplies and arming IRQ */
	tcm->reset_gpio = devm_gpiod_get(&client->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(tcm->reset_gpio))
		return PTR_ERR(tcm->reset_gpio);

	ret = regulator_bulk_enable(ARRAY_SIZE(tcm->supplies), tcm->supplies);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&client->dev, tcm_power_off, tcm);
	if (ret)
		return ret;

	usleep_range(10000, 11000);

	/*
	 * Arm IRQ before releasing reset so we capture the controller's
	 * automatic power-on REPORT_IDENTIFY (matching OPlus syna_tcm_reset).
	 */
	ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
					tcm_report_irq,
					IRQF_ONESHOT,
					"synaptics_tcm_report", tcm);
	if (ret < 0)
		return ret;

	reinit_completion(&tcm->response);
	gpiod_set_value_cansleep(tcm->reset_gpio, 0);
	if (!wait_for_completion_timeout(&tcm->response, msecs_to_jiffies(250))) {
		/* Drain one pending boot packet in case ATTN was not asserted */
		tcm_read_one_message(tcm);
	}

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

	__set_bit(EV_KEY, tcm->input->evbit);
	input_set_capability(tcm->input, EV_KEY, KEY_WAKEUP);
	for (i = 1; i <= GESTURE_MAX_ID; i++)
		input_set_capability(tcm->input, EV_KEY, KEY_GESTURE_START + i);

	touchscreen_parse_properties(tcm->input, true, &tcm->props);

	ret = input_mt_init_slots(tcm->input, 10, INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (ret)
		return ret;

	input_set_drvdata(tcm->input, tcm);

	ret = input_register_device(tcm->input);
	if (ret)
		return ret;

	device_init_wakeup(&client->dev, true);

	/*
	 * /proc/touchpanel may already be created by goodix_fp (fp_enable).
	 * Try creating under the existing directory first to avoid duplicate
	 * proc_mkdir() warnings, and only create the directory if absent.
	 */
	tcm->proc_gesture = proc_create_data("touchpanel/double_tap_enable_indep",
					     0666, NULL, &tcm_proc_gesture_ops, tcm);
	if (tcm->proc_gesture) {
		tcm->proc_game = proc_create_data("touchpanel/game_switch_enable",
						  0666, NULL, &tcm_proc_game_ops, tcm);
	} else {
		tcm->proc_dir = proc_mkdir("touchpanel", NULL);
		if (tcm->proc_dir) {
			tcm->proc_gesture = proc_create_data("double_tap_enable_indep",
							     0666, tcm->proc_dir,
							     &tcm_proc_gesture_ops, tcm);
			tcm->proc_game = proc_create_data("game_switch_enable",
							  0666, tcm->proc_dir,
							  &tcm_proc_game_ops, tcm);
		}
	}

	WRITE_ONCE(g_tcm_data, tcm);
	ret = devm_add_action_or_reset(&client->dev, tcm_proc_cleanup, tcm);
	if (ret)
		return ret;

	return 0;
}

static int tcm_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct tcm_data *tcm = i2c_get_clientdata(client);

	flush_work(&tcm->screen_work);
	if (tcm->in_gesture_mode && device_may_wakeup(dev) &&
	    !enable_irq_wake(client->irq)) {
		tcm->irq_wake_enabled = true;
	} else {
		disable_irq(client->irq);
		tcm->irq_disabled = true;
	}
	return 0;
}

static int tcm_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct tcm_data *tcm = i2c_get_clientdata(client);

	if (tcm->irq_wake_enabled) {
		disable_irq_wake(client->irq);
		tcm->irq_wake_enabled = false;
	}
	if (tcm->irq_disabled) {
		enable_irq(client->irq);
		tcm->irq_disabled = false;
	}
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
