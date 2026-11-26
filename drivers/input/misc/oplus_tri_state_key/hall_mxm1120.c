// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 *
 * MagnaChip MXM1120 I2C Hall sensor driver for OnePlus dual-Hall Alert Slider.
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

#include "oplus_tri_key.h"

#define M1120_REG_PERSINT		0x00
#define M1120_VAL_PERSINT_COUNT(n)	((n) << 4)
#define M1120_VAL_PERSINT_INTCLR	0x01

#define M1120_REG_INTSRS		0x01
#define M1120_VAL_INTSRS_INT_ON		0x80
#define M1120_VAL_INTSRS_INTTYPE_BESIDE	0x00
#define M1120_VAL_INTSRS_SRS_10BIT_0_068mT 0x00

#define M1120_REG_LTHL			0x02
#define M1120_REG_LTHH			0x03
#define M1120_REG_HTHL			0x04
#define M1120_REG_HTHH			0x05
#define M1120_REG_SRST			0x07
#define M1120_VAL_SRST_RESET		0x01

#define M1120_REG_OPF			0x08
#define M1120_VAL_OPF_FREQ_80HZ		0x40
#define M1120_VAL_OPF_BIT_8		0x02
#define M1120_VAL_OPF_BIT_10		0x00
#define M1120_VAL_OPF_HSSON_ON		0x01

#define M1120_REG_DID			0x09
#define M1120_VAL_DID			0x9c

#define M1120_REG_ST1			0x10
#define M1120_VAL_ST1_DRDY		0x01

#define M1120_I2C_MAX_LEN		16

struct mxm1120_data {
	struct i2c_client *client;
	struct mutex i2c_lock;
	struct regulator *vdd;
	struct regulator *vio;
	struct oplus_hall_sensor sensor;

	u8 persint;
	u8 intsrs;
	u8 opf;

	short last_value;
	bool has_last_value;
	bool power_on;
};

static inline struct mxm1120_data *to_mxm1120(struct oplus_hall_sensor *sensor)
{
	return container_of(sensor, struct mxm1120_data, sensor);
}

static int mxm1120_read_block(struct mxm1120_data *data, u8 reg, u8 *buf, u8 len)
{
	struct i2c_client *client = data->client;
	struct i2c_msg msgs[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};
	int ret;

	if (len == 0 || len > M1120_I2C_MAX_LEN)
		return -EINVAL;

	mutex_lock(&data->i2c_lock);
	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	mutex_unlock(&data->i2c_lock);

	if (ret < 0)
		return ret;

	return (ret == ARRAY_SIZE(msgs)) ? 0 : -EIO;
}

static int mxm1120_write_reg(struct mxm1120_data *data, u8 reg, u8 val)
{
	u8 buf[2] = { reg, val };
	int ret;

	mutex_lock(&data->i2c_lock);
	ret = i2c_master_send(data->client, buf, sizeof(buf));
	if (ret == sizeof(buf)) {
		if (reg == M1120_REG_PERSINT)
			data->persint = val;
		else if (reg == M1120_REG_INTSRS)
			data->intsrs = val;
		else if (reg == M1120_REG_OPF)
			data->opf = val;
		ret = 0;
	} else if (ret >= 0) {
		ret = -EIO;
	}
	mutex_unlock(&data->i2c_lock);

	return ret;
}

static void mxm1120_short_to_2byte(struct mxm1120_data *data, short x,
				   u8 *hbyte, u8 *lbyte)
{
	if ((data->opf & M1120_VAL_OPF_BIT_8) == M1120_VAL_OPF_BIT_8) {
		x = clamp_t(short, x, -128, 127);
		*lbyte = (x >= 0) ? (x & 0x7f) :
			 (((0x80 - (-x)) & 0x7f) | 0x80);
		*hbyte = 0x00;
	} else {
		x = clamp_t(short, x, -512, 511);
		if (x >= 0) {
			*lbyte = x & 0xff;
			*hbyte = ((x >> 8) & 0x01) << 6;
		} else {
			u16 neg = 0x0200 - (u16)(-x);

			*lbyte = neg & 0xff;
			*hbyte = (((neg >> 8) & 0x01) << 6) | 0x80;
		}
	}
}

static short mxm1120_2byte_to_short(struct mxm1120_data *data, u8 hbyte, u8 lbyte)
{
	short x;

	if ((data->opf & M1120_VAL_OPF_BIT_8) == M1120_VAL_OPF_BIT_8) {
		x = lbyte & 0x7f;
		if (lbyte & 0x80)
			x -= 0x80;
	} else {
		x = (((hbyte & 0x40) >> 6) << 8) | lbyte;
		if (hbyte & 0x80)
			x -= 0x200;
	}

	return x;
}

static int mxm1120_clear_interrupt(struct mxm1120_data *data)
{
	u8 val = data->persint | M1120_VAL_PERSINT_INTCLR;

	return mxm1120_write_reg(data, M1120_REG_PERSINT, val);
}

static int mxm1120_get_data(struct oplus_hall_sensor *sensor, short *out)
{
	struct mxm1120_data *data = to_mxm1120(sensor);
	u8 buf[3] = { 0 };
	int attempt, ret;

	for (attempt = 0; attempt < 3; attempt++) {
		ret = mxm1120_read_block(data, M1120_REG_ST1, buf, sizeof(buf));
		if (ret)
			return ret;

		if (buf[0] & M1120_VAL_ST1_DRDY) {
			short val = mxm1120_2byte_to_short(data, buf[2], buf[1]);

			data->last_value = val;
			data->has_last_value = true;
			*out = val;
			return 0;
		}

		if (data->has_last_value) {
			*out = data->last_value;
			return 0;
		}

		msleep(20);
	}

	*out = data->last_value;
	return 0;
}

static int mxm1120_update_threshold(struct oplus_hall_sensor *sensor,
				    int position, short lowthd, short highthd)
{
	struct mxm1120_data *data = to_mxm1120(sensor);
	u8 hthh, hthl, lthh, lthl;
	int ret;

	ret = mxm1120_clear_interrupt(data);
	if (ret)
		return ret;

	if (!(data->intsrs & M1120_VAL_INTSRS_INT_ON))
		return 0;

	mxm1120_short_to_2byte(data, highthd, &hthh, &hthl);
	mxm1120_short_to_2byte(data, lowthd, &lthh, &lthl);

	ret = mxm1120_write_reg(data, M1120_REG_HTHH, hthh);
	if (ret)
		return ret;
	ret = mxm1120_write_reg(data, M1120_REG_HTHL, hthl);
	if (ret)
		return ret;
	ret = mxm1120_write_reg(data, M1120_REG_LTHH, lthh);
	if (ret)
		return ret;
	return mxm1120_write_reg(data, M1120_REG_LTHL, lthl);
}

static irqreturn_t mxm1120_irq_handler(int irq, void *dev_id)
{
	struct mxm1120_data *data = dev_id;

	if (!atomic_xchg(&data->sensor.irq_masked, 1))
		disable_irq_nosync(data->sensor.irq);

	oplus_tri_key_irq_notify(&data->sensor);
	return IRQ_HANDLED;
}

static int mxm1120_enable_irq(struct oplus_hall_sensor *sensor, bool enable)
{
	if (!sensor->irq_requested)
		return 0;

	if (enable) {
		if (atomic_xchg(&sensor->irq_masked, 0))
			enable_irq(sensor->irq);
	} else {
		if (!atomic_xchg(&sensor->irq_masked, 1))
			disable_irq_nosync(sensor->irq);
	}

	return 0;
}

static int mxm1120_clear_irq(struct oplus_hall_sensor *sensor)
{
	return mxm1120_clear_interrupt(to_mxm1120(sensor));
}

static int mxm1120_set_detection_mode(struct oplus_hall_sensor *sensor, u8 mode)
{
	struct mxm1120_data *data = to_mxm1120(sensor);
	u8 val;
	int ret;

	if (mode == DETECTION_MODE_INTERRUPT) {
		if (sensor->irq_requested)
			return 0;

		if (sensor->irq <= 0) {
			dev_err(sensor->dev, "invalid IRQ %d\n", sensor->irq);
			return -EINVAL;
		}

		val = data->intsrs | M1120_VAL_INTSRS_INT_ON;
		ret = mxm1120_write_reg(data, M1120_REG_INTSRS, val);
		if (ret)
			return ret;

		ret = mxm1120_clear_interrupt(data);
		if (ret)
			return ret;

		atomic_set(&sensor->irq_masked, 0);
		ret = request_irq(sensor->irq, mxm1120_irq_handler,
				  IRQF_TRIGGER_LOW, sensor->name, data);
		if (ret) {
			dev_err(sensor->dev, "request_irq(%d) failed (%d)\n",
				sensor->irq, ret);
			return ret;
		}

		enable_irq_wake(sensor->irq);
		sensor->irq_requested = true;
	} else {
		if (!sensor->irq_requested)
			return 0;

		val = data->intsrs & ~M1120_VAL_INTSRS_INT_ON;
		mxm1120_write_reg(data, M1120_REG_INTSRS, val);

		disable_irq_wake(sensor->irq);
		if (!atomic_xchg(&sensor->irq_masked, 1))
			disable_irq(sensor->irq);
		free_irq(sensor->irq, data);
		sensor->irq_requested = false;
	}

	return 0;
}

static bool mxm1120_is_power_on(struct oplus_hall_sensor *sensor)
{
	return to_mxm1120(sensor)->power_on;
}

static const struct oplus_hall_ops mxm1120_ops = {
	.get_data = mxm1120_get_data,
	.set_detection_mode = mxm1120_set_detection_mode,
	.enable_irq = mxm1120_enable_irq,
	.clear_irq = mxm1120_clear_irq,
	.update_threshold = mxm1120_update_threshold,
	.is_power_on = mxm1120_is_power_on,
};

static int mxm1120_configure_chip(struct mxm1120_data *data)
{
	u8 id = 0;
	int ret;

	ret = mxm1120_write_reg(data, M1120_REG_SRST, M1120_VAL_SRST_RESET);
	if (ret)
		return ret;
	msleep(20);

	ret = mxm1120_read_block(data, M1120_REG_DID, &id, 1);
	if (ret || id != M1120_VAL_DID)
		return ret ? ret : -ENODEV;

	ret = mxm1120_write_reg(data, M1120_REG_PERSINT,
				M1120_VAL_PERSINT_COUNT(15));
	if (ret)
		return ret;

	ret = mxm1120_write_reg(data, M1120_REG_INTSRS,
				M1120_VAL_INTSRS_INT_ON |
				M1120_VAL_INTSRS_INTTYPE_BESIDE |
				M1120_VAL_INTSRS_SRS_10BIT_0_068mT);
	if (ret)
		return ret;

	ret = mxm1120_write_reg(data, M1120_REG_OPF,
				M1120_VAL_OPF_FREQ_80HZ |
				M1120_VAL_OPF_BIT_10 |
				M1120_VAL_OPF_HSSON_ON);
	if (ret)
		return ret;

	/* Allow first 80 Hz sample to complete */
	msleep(20);
	return 0;
}

static void mxm1120_regulator_disable(void *arg)
{
	struct mxm1120_data *data = arg;

	data->power_on = false;
	if (data->vio)
		regulator_disable(data->vio);
	if (data->vdd)
		regulator_disable(data->vdd);
}

static int mxm1120_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct mxm1120_data *data;
	struct pinctrl *pctrl;
	struct pinctrl_state *pstate;
	enum oplus_hall_id id;
	u8 dev_id = 0;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -EOPNOTSUPP;

	id = (enum oplus_hall_id)(uintptr_t)device_get_match_data(dev);

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	mutex_init(&data->i2c_lock);

	data->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(data->vdd))
		return dev_err_probe(dev, PTR_ERR(data->vdd),
				     "failed to get vdd supply\n");

	data->vio = devm_regulator_get_optional(dev, "vio");
	if (IS_ERR(data->vio)) {
		if (PTR_ERR(data->vio) == -ENODEV)
			data->vio = NULL;
		else
			return dev_err_probe(dev, PTR_ERR(data->vio),
					     "failed to get vio supply\n");
	}

	ret = regulator_enable(data->vdd);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable vdd\n");

	if (data->vio) {
		msleep(20);
		ret = regulator_enable(data->vio);
		if (ret) {
			regulator_disable(data->vdd);
			return dev_err_probe(dev, ret, "failed to enable vio\n");
		}
	}
	data->power_on = true;

	ret = devm_add_action_or_reset(dev, mxm1120_regulator_disable, data);
	if (ret)
		return ret;

	msleep(20);

	/*
	 * Verify device ID before claiming pinctrl or IRQ so second-source
	 * IST8801 boards do not hit GPIO/pinctrl contention.
	 */
	ret = mxm1120_read_block(data, M1120_REG_DID, &dev_id, 1);
	if (ret || dev_id != M1120_VAL_DID) {
		dev_dbg(dev, "MXM1120 not present (ret=%d id=0x%02x)\n",
			ret, dev_id);
		return -ENODEV;
	}

	pctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(pctrl)) {
		pstate = pinctrl_lookup_state(pctrl, "active");
		if (!IS_ERR(pstate))
			pinctrl_select_state(pctrl, pstate);
	}

	ret = mxm1120_configure_chip(data);
	if (ret) {
		dev_err(dev, "failed to configure MXM1120 (%d)\n", ret);
		return ret;
	}

	data->sensor.id = id;
	data->sensor.name = (id == DHALL_UP) ? "mxm1120_up" : "mxm1120_down";
	data->sensor.dev = dev;
	data->sensor.ops = &mxm1120_ops;
	data->sensor.irq = client->irq;
	atomic_set(&data->sensor.irq_masked, 0);

	i2c_set_clientdata(client, data);

	ret = oplus_tri_key_register_hall(&data->sensor);
	if (ret)
		return ret;

	dev_info(dev, "probed %s at 0x%02x (irq=%d)\n",
		 data->sensor.name, client->addr, client->irq);
	return 0;
}

static void mxm1120_remove(struct i2c_client *client)
{
	struct mxm1120_data *data = i2c_get_clientdata(client);

	oplus_tri_key_unregister_hall(&data->sensor);
	mxm1120_set_detection_mode(&data->sensor, DETECTION_MODE_POLLING);
	mxm1120_write_reg(data, M1120_REG_OPF, data->opf & ~M1120_VAL_OPF_HSSON_ON);
}

static const struct of_device_id mxm1120_of_match[] = {
	{ .compatible = "oplus,hall-mxm1120,down", .data = (void *)(uintptr_t)DHALL_DOWN },
	{ .compatible = "oplus,hall-mxm1120,up",   .data = (void *)(uintptr_t)DHALL_UP },
	{ }
};
MODULE_DEVICE_TABLE(of, mxm1120_of_match);

static const struct i2c_device_id mxm1120_id_table[] = {
	{ "hall_m1120_down", DHALL_DOWN },
	{ "hall_m1120_up",   DHALL_UP },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mxm1120_id_table);

struct i2c_driver mxm1120_hall_driver = {
	.driver = {
		.name = "oplus-hall-mxm1120",
		.of_match_table = mxm1120_of_match,
	},
	.probe = mxm1120_probe,
	.remove = mxm1120_remove,
	.id_table = mxm1120_id_table,
};
