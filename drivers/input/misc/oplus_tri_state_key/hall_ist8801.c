// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 *
 * iSentek IST8801 I2C Hall sensor driver for OnePlus dual-Hall Alert Slider.
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

#define IST8801_REG_PERSINT		0x00
#define IST8801_VAL_PERSINT_COUNT(n)	((n) << 4)
#define IST8801_VAL_PERSINT_INTCLR	0x01

#define IST8801_REG_INTSRS		0x01
#define IST8801_VAL_INTSRS_INT_ON	0x80
#define IST8801_VAL_INTSRS_INTTYPE_BESIDE 0x00

#define IST8801_REG_LTHL		0x02
#define IST8801_REG_LTHH		0x03
#define IST8801_REG_HTHL		0x04
#define IST8801_REG_HTHH		0x05
#define IST8801_REG_SRST		0x07
#define IST8801_VAL_SRST_RESET		0x01

#define IST8801_REG_OPF			0x08
#define IST8801_VAL_OPF_STANDBY		0x00
#define IST8801_VAL_OPF_FREQ_20HZ	0x70

#define IST8801_REG_DID			0x09
#define IST8801_VAL_DID			0x81

#define IST8801_REG_CNTL2		0x0d
#define IST8801_GAIN_2_TIME		(0x1 << 5)
#define IST8801_GAIN_4_TIME		(0x2 << 5)
#define IST8801_ADC_RES_10_BIT		(0x6 << 1)
#define IST8801_DYNAMIC_GAIN_ADC_BIT	(IST8801_GAIN_4_TIME | IST8801_ADC_RES_10_BIT)

#define IST8801_REG_ST1			0x10
#define IST8801_VAL_ST1_DRDY		0x01
#define IST8801_VAL_ST1_DORZ		0x04

#define IST8801_REG_ACTION		0x20
#define IST8801_REG_IFCNTL		0x40
#define IST8801_REG_GAINCNTL		0x54
#define IST8801_REG_OSRCNTL		0x6c
#define IST8801_REG_TSTCNTL		0x76
#define IST8801_REG_INFO		0x87

#define IST8801_I2C_MAX_LEN		16

struct ist8801_data {
	struct i2c_client *client;
	struct mutex i2c_lock;
	struct regulator *vdd;
	struct regulator *vio;
	struct oplus_hall_sensor sensor;

	u8 persint;
	u8 intsrs;
	u8 range;
	u8 origin_gain;
	u8 origin_info;

	short last_value;
	bool has_last_value;
	bool power_on;
};

static inline struct ist8801_data *to_ist8801(struct oplus_hall_sensor *sensor)
{
	return container_of(sensor, struct ist8801_data, sensor);
}

static int ist8801_read_block(struct ist8801_data *data, u8 reg, u8 *buf, u8 len)
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

	if (len == 0 || len > IST8801_I2C_MAX_LEN)
		return -EINVAL;

	mutex_lock(&data->i2c_lock);
	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	mutex_unlock(&data->i2c_lock);

	if (ret < 0)
		return ret;

	return (ret == ARRAY_SIZE(msgs)) ? 0 : -EIO;
}

static int ist8801_write_reg(struct ist8801_data *data, u8 reg, u8 val)
{
	u8 buf[2] = { reg, val };
	int ret;

	mutex_lock(&data->i2c_lock);
	ret = i2c_master_send(data->client, buf, sizeof(buf));
	if (ret == sizeof(buf)) {
		if (reg == IST8801_REG_PERSINT)
			data->persint = val;
		else if (reg == IST8801_REG_INTSRS)
			data->intsrs = val;
		ret = 0;
	} else if (ret >= 0) {
		ret = -EIO;
	}
	mutex_unlock(&data->i2c_lock);

	return ret;
}

static int ist8801_clear_interrupt(struct ist8801_data *data)
{
	u8 val;
	int ret;

	val = data->persint | IST8801_VAL_PERSINT_INTCLR;
	ret = ist8801_write_reg(data, IST8801_REG_PERSINT, val);
	if (ret)
		return ret;

	val = data->persint & ~IST8801_VAL_PERSINT_INTCLR;
	return ist8801_write_reg(data, IST8801_REG_PERSINT, val);
}

static int ist8801_get_data(struct oplus_hall_sensor *sensor, short *out)
{
	struct ist8801_data *data = to_ist8801(sensor);
	u8 buf[3] = { 0 };
	int attempt, ret;

	for (attempt = 0; attempt < 3; attempt++) {
		ret = ist8801_read_block(data, IST8801_REG_ST1, buf, sizeof(buf));
		if (ret)
			return ret;

		if (buf[0] & (IST8801_VAL_ST1_DRDY | IST8801_VAL_ST1_DORZ)) {
			short val = (short)((buf[2] << 8) | buf[1]);

			data->last_value = val;
			data->has_last_value = true;
			*out = val;
			return 0;
		}

		if (data->has_last_value) {
			*out = data->last_value;
			return 0;
		}

		msleep(25);
	}

	*out = data->last_value;
	return 0;
}

static int ist8801_update_threshold(struct oplus_hall_sensor *sensor,
				    int position, short lowthd, short highthd)
{
	struct ist8801_data *data = to_ist8801(sensor);
	u16 lo = (u16)lowthd;
	u16 hi = (u16)highthd;
	int ret;

	ret = ist8801_clear_interrupt(data);
	if (ret)
		return ret;

	if (!(data->intsrs & IST8801_VAL_INTSRS_INT_ON))
		return 0;

	ret = ist8801_write_reg(data, IST8801_REG_HTHH, (hi >> 8) & 0xff);
	if (ret)
		return ret;
	ret = ist8801_write_reg(data, IST8801_REG_HTHL, hi & 0xff);
	if (ret)
		return ret;
	ret = ist8801_write_reg(data, IST8801_REG_LTHH, (lo >> 8) & 0xff);
	if (ret)
		return ret;
	return ist8801_write_reg(data, IST8801_REG_LTHL, lo & 0xff);
}

static irqreturn_t ist8801_irq_handler(int irq, void *dev_id)
{
	struct ist8801_data *data = dev_id;

	if (!atomic_xchg(&data->sensor.irq_masked, 1))
		disable_irq_nosync(data->sensor.irq);

	oplus_tri_key_irq_notify(&data->sensor);
	return IRQ_HANDLED;
}

static int ist8801_enable_irq(struct oplus_hall_sensor *sensor, bool enable)
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

static int ist8801_clear_irq(struct oplus_hall_sensor *sensor)
{
	return ist8801_clear_interrupt(to_ist8801(sensor));
}

static int ist8801_set_detection_mode(struct oplus_hall_sensor *sensor, u8 mode)
{
	struct ist8801_data *data = to_ist8801(sensor);
	u8 val;
	int ret;

	if (mode == DETECTION_MODE_INTERRUPT) {
		if (sensor->irq_requested)
			return 0;

		if (sensor->irq <= 0) {
			dev_err(sensor->dev, "invalid IRQ %d\n", sensor->irq);
			return -EINVAL;
		}

		val = data->intsrs | IST8801_VAL_INTSRS_INT_ON;
		ret = ist8801_write_reg(data, IST8801_REG_INTSRS, val);
		if (ret)
			return ret;

		ret = ist8801_clear_interrupt(data);
		if (ret)
			return ret;

		atomic_set(&sensor->irq_masked, 0);
		ret = request_irq(sensor->irq, ist8801_irq_handler,
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

		val = data->intsrs & ~IST8801_VAL_INTSRS_INT_ON;
		ist8801_write_reg(data, IST8801_REG_INTSRS, val);

		disable_irq_wake(sensor->irq);
		if (!atomic_xchg(&sensor->irq_masked, 1))
			disable_irq(sensor->irq);
		free_irq(sensor->irq, data);
		sensor->irq_requested = false;
	}

	return 0;
}

static bool ist8801_is_power_on(struct oplus_hall_sensor *sensor)
{
	return to_ist8801(sensor)->power_on;
}

static const struct oplus_hall_ops ist8801_ops = {
	.get_data = ist8801_get_data,
	.set_detection_mode = ist8801_set_detection_mode,
	.enable_irq = ist8801_enable_irq,
	.clear_irq = ist8801_clear_irq,
	.update_threshold = ist8801_update_threshold,
	.is_power_on = ist8801_is_power_on,
};

static int ist8801_set_measurement_mode(struct ist8801_data *data)
{
	u8 ifcntl = 0;
	int ret;

	ret = ist8801_write_reg(data, IST8801_REG_ACTION, 0x00);
	if (ret)
		return ret;
	usleep_range(5000, 5100);

	ret = ist8801_write_reg(data, IST8801_REG_OPF, IST8801_VAL_OPF_STANDBY);
	if (ret)
		return ret;

	ret = ist8801_read_block(data, IST8801_REG_IFCNTL, &ifcntl, 1);
	if (ret)
		return ret;
	ret = ist8801_write_reg(data, IST8801_REG_IFCNTL, ifcntl | 0x04);
	if (ret)
		return ret;

	return ist8801_write_reg(data, IST8801_REG_OPF, IST8801_VAL_OPF_FREQ_20HZ);
}

static int ist8801_configure_chip(struct ist8801_data *data)
{
	u8 val = 0;
	u8 ifcntl = 0;
	u8 saved_opf = 0;
	int ret;

	/* Soft reset */
	ret = ist8801_write_reg(data, IST8801_REG_SRST, IST8801_VAL_SRST_RESET);
	if (ret)
		return ret;
	msleep(20);

	ret = ist8801_read_block(data, IST8801_REG_DID, &val, 1);
	if (ret || val != IST8801_VAL_DID)
		return ret ? ret : -ENODEV;

	/* Stock OPlus factory init sequence */
	ret = ist8801_write_reg(data, IST8801_REG_TSTCNTL, 0x04);
	if (ret)
		return ret;
	ret = ist8801_write_reg(data, IST8801_REG_OSRCNTL, 0x05);
	if (ret)
		return ret;

	ret = ist8801_read_block(data, IST8801_REG_GAINCNTL, &data->origin_gain, 1);
	if (ret)
		return ret;
	ret = ist8801_read_block(data, IST8801_REG_INFO, &data->origin_info, 1);
	if (ret)
		return ret;

	ret = ist8801_write_reg(data, IST8801_REG_PERSINT,
				IST8801_VAL_PERSINT_COUNT(2));
	if (ret)
		return ret;

	val = IST8801_VAL_INTSRS_INT_ON | IST8801_VAL_INTSRS_INTTYPE_BESIDE | data->range;
	ret = ist8801_write_reg(data, IST8801_REG_INTSRS, val);
	if (ret)
		return ret;

	ret = ist8801_set_measurement_mode(data);
	if (ret)
		return ret;

	/* Set 40mT sensitivity (GAIN_2_TIME, ratio 0) matching stock probe */
	ret = ist8801_read_block(data, IST8801_REG_OPF, &saved_opf, 1);
	if (ret)
		return ret;
	ret = ist8801_write_reg(data, IST8801_REG_OPF, IST8801_VAL_OPF_STANDBY);
	if (ret)
		return ret;
	ret = ist8801_read_block(data, IST8801_REG_IFCNTL, &ifcntl, 1);
	if (ret)
		return ret;
	ret = ist8801_write_reg(data, IST8801_REG_IFCNTL, ifcntl | 0x04);
	if (ret)
		return ret;

	val = (IST8801_DYNAMIC_GAIN_ADC_BIT & 0x1e) | IST8801_GAIN_2_TIME;
	ret = ist8801_write_reg(data, IST8801_REG_CNTL2, val);
	if (ret)
		return ret;
	ret = ist8801_write_reg(data, IST8801_REG_GAINCNTL, data->origin_gain);
	if (ret)
		return ret;
	ret = ist8801_write_reg(data, IST8801_REG_OPF, saved_opf);
	if (ret)
		return ret;

	/* Wait one 20 Hz conversion cycle so the initial DRDY sample is valid */
	msleep(60);
	return 0;
}

static void ist8801_regulator_disable(void *arg)
{
	struct ist8801_data *data = arg;

	data->power_on = false;
	if (data->vio)
		regulator_disable(data->vio);
	if (data->vdd)
		regulator_disable(data->vdd);
}

static int ist8801_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ist8801_data *data;
	struct pinctrl *pctrl;
	struct pinctrl_state *pstate;
	enum oplus_hall_id id;
	u32 range = 0;
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

	if (!of_property_read_u32(dev->of_node, "data-range", &range))
		data->range = (u8)range;

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

	ret = devm_add_action_or_reset(dev, ist8801_regulator_disable, data);
	if (ret)
		return ret;

	msleep(20);

	/*
	 * Verify device ID before claiming pinctrl or IRQ so second-source
	 * MXM1120 boards do not hit GPIO/pinctrl contention.
	 */
	ret = ist8801_read_block(data, IST8801_REG_DID, &dev_id, 1);
	if (ret || dev_id != IST8801_VAL_DID) {
		dev_dbg(dev, "IST8801 not present (ret=%d id=0x%02x)\n",
			ret, dev_id);
		return -ENODEV;
	}

	pctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(pctrl)) {
		pstate = pinctrl_lookup_state(pctrl, "active");
		if (!IS_ERR(pstate))
			pinctrl_select_state(pctrl, pstate);
	}

	ret = ist8801_configure_chip(data);
	if (ret) {
		dev_err(dev, "failed to configure IST8801 (%d)\n", ret);
		return ret;
	}

	data->sensor.id = id;
	data->sensor.name = (id == DHALL_UP) ? "ist8801_up" : "ist8801_down";
	data->sensor.dev = dev;
	data->sensor.ops = &ist8801_ops;
	data->sensor.irq = client->irq;
	atomic_set(&data->sensor.irq_masked, 0);

	i2c_set_clientdata(client, data);

	ret = oplus_tri_key_register_hall(&data->sensor);
	if (ret)
		return ret;

	dev_info(dev, "probed %s at 0x%02x (irq=%d, info=0x%02x, gain=%u)\n",
		 data->sensor.name, client->addr, client->irq,
		 data->origin_info, data->origin_gain);
	return 0;
}

static void ist8801_remove(struct i2c_client *client)
{
	struct ist8801_data *data = i2c_get_clientdata(client);

	oplus_tri_key_unregister_hall(&data->sensor);
	ist8801_set_detection_mode(&data->sensor, DETECTION_MODE_POLLING);
	ist8801_write_reg(data, IST8801_REG_OPF, IST8801_VAL_OPF_STANDBY);
}

static const struct of_device_id ist8801_of_match[] = {
	{ .compatible = "oplus,hall-ist8801,down", .data = (void *)(uintptr_t)DHALL_DOWN },
	{ .compatible = "oplus,hall-ist8801,up",   .data = (void *)(uintptr_t)DHALL_UP },
	{ }
};
MODULE_DEVICE_TABLE(of, ist8801_of_match);

static const struct i2c_device_id ist8801_id_table[] = {
	{ "hall_ist8801_down", DHALL_DOWN },
	{ "hall_ist8801_up",   DHALL_UP },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ist8801_id_table);

struct i2c_driver ist8801_hall_driver = {
	.driver = {
		.name = "oplus-hall-ist8801",
		.of_match_table = ist8801_of_match,
	},
	.probe = ist8801_probe,
	.remove = ist8801_remove,
	.id_table = ist8801_id_table,
};
