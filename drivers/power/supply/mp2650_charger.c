// SPDX-License-Identifier: GPL-2.0-only
/*
 * Monolithic Power Systems MP2650 2-cell Buck-Boost Battery Charger driver
 *
 * Copyright (C) 2026 The PenguinOS Project
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/devm-helpers.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#define MP2650_REG_IIN_LIM_1		0x00
#define MP2650_REG_VINDPM		0x01
#define MP2650_REG_ICHG			0x02
#define MP2650_REG_IPRE_ITERM		0x03
#define MP2650_REG_VREG			0x04
#define MP2650_REG_THERM_IR		0x05
#define MP2650_REG_OTG_VOL		0x06
#define MP2650_REG_OTG_CURR_CELL	0x07
#define MP2650_REG_CTRL0		0x08
#define MP2650_REG_CTRL1		0x09
#define MP2650_REG_NTC			0x0a
#define MP2650_REG_CTRL2		0x0b
#define MP2650_REG_IIN_LIM_2		0x0f
#define MP2650_REG_STAT			0x13
#define MP2650_REG_FAULT		0x14
#define MP2650_REG_REV			0x15
#define MP2650_REG_ADC_VBAT_LO		0x16
#define MP2650_REG_ADC_VBAT_HI		0x17
#define MP2650_REG_ADC_ICHG_LO		0x1a
#define MP2650_REG_ADC_ICHG_HI		0x1b
#define MP2650_REG_ADC_VBUS_LO		0x1c
#define MP2650_REG_ADC_VBUS_HI		0x1d
#define MP2650_REG_ADC_IBUS_LO		0x1e
#define MP2650_REG_ADC_IBUS_HI		0x1f

/* REG00 / REG0F: Input current limit (50 mA/LSB) */
#define MP2650_IIN_LIM_MASK		GENMASK(6, 0)
#define MP2650_IIN_LIM_STEP_UA		50000

/* REG01: VINDPM threshold (100 mV/LSB) */
#define MP2650_VINDPM_STEP_UV		100000

/* REG02: Fast charge current (50 mA/LSB) */
#define MP2650_ICHG_STEP_UA		50000

/* REG03: Precharge (60 mA/LSB) & Termination current (100 mA/LSB) */
#define MP2650_ITERM_MASK		GENMASK(3, 0)
#define MP2650_ITERM_STEP_UA		100000
#define MP2650_IPRE_MASK		GENMASK(7, 4)
#define MP2650_IPRE_STEP_UA		60000

/* REG04: Per-cell charge full voltage (3.7125 V + N * 12.5 mV) */
#define MP2650_VREG_MASK		GENMASK(6, 1)
#define MP2650_VREG_CELL_BASE_UV	3712500
#define MP2650_VREG_CELL_STEP_UV	12500

/* REG07: Cell count */
#define MP2650_CELL_CNT_MASK		GENMASK(7, 6)
#define MP2650_CELL_CNT_2S		0

/* REG08: Control 0 */
#define MP2650_CTRL0_CHG_EN		BIT(4)
#define MP2650_CTRL0_OTG_EN		BIT(5)
#define MP2650_CTRL0_WDT_RST		BIT(6)

/* REG09: Control 1 */
#define MP2650_CTRL1_TIMER_MASK		GENMASK(2, 1)
#define MP2650_CTRL1_TIMER_12H		FIELD_PREP(MP2650_CTRL1_TIMER_MASK, 2)
#define MP2650_CTRL1_TIMER_EN		BIT(3)
#define MP2650_CTRL1_WDT_MASK		GENMASK(5, 4)
#define MP2650_CTRL1_WDT_DISABLE	0
#define MP2650_CTRL1_TERM_EN		BIT(6)

/* REG0B: Control 2 */
#define MP2650_CTRL2_ADC_EN_MASK	GENMASK(1, 0)
#define MP2650_CTRL2_ADC_EN		0x3

/* REG13: Status (RO) */
#define MP2650_STAT_VIN_PG_N		BIT(1)
#define MP2650_STAT_CHG_MASK		GENMASK(3, 2)
#define MP2650_STAT_CHG_NONE		0
#define MP2650_STAT_CHG_PRE		1
#define MP2650_STAT_CHG_FAST		2
#define MP2650_STAT_CHG_DONE		3

/* REG14: Fault (RO) */
#define MP2650_FAULT_NTC_MASK		GENMASK(2, 0)
#define MP2650_FAULT_NTC_COLD		1
#define MP2650_FAULT_NTC_HOT		4
#define MP2650_FAULT_BAT_OVP		BIT(3)
#define MP2650_FAULT_CHG_MASK		GENMASK(5, 4)
#define MP2650_FAULT_CHG_INPUT		1
#define MP2650_FAULT_CHG_THERM		2
#define MP2650_FAULT_CHG_TIMEOUT	3

/* Safe 2S operating limits for OnePlus 8T */
#define MP2650_MIN_IIN_UA		500000
#define MP2650_MAX_IIN_UA		3000000
#define MP2650_MIN_ICHG_UA		500000
#define MP2650_MAX_ICHG_UA		3000000
#define MP2650_MIN_VREG_2S_UV		7600000
#define MP2650_MAX_VREG_2S_UV		8800000
#define MP2650_DEFAULT_VREG_2S_UV	8700000
#define MP2650_DEFAULT_ICHG_UA		2000000
#define MP2650_DEFAULT_IIN_UA		3000000

struct mp2650_charger {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct gpio_desc *otg_en_gpio;
	struct notifier_block psy_nb;
	struct delayed_work supply_work;
	struct mutex lock;

	u32 max_vreg_uv;
	u32 max_ichg_ua;
	u32 max_iin_ua;
	bool inhibit_charge;
};

static const struct regmap_config mp2650_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x3f,
};

static int mp2650_read_adc10(struct mp2650_charger *chg, unsigned int reg_lo, u32 *val)
{
	unsigned int lo, hi;
	int ret;

	ret = regmap_read(chg->regmap, reg_lo, &lo);
	if (ret)
		return ret;
	ret = regmap_read(chg->regmap, reg_lo + 1, &hi);
	if (ret)
		return ret;

	*val = ((hi & 0xff) << 2) | ((lo >> 6) & 0x03);
	return 0;
}

static int mp2650_set_input_current_limit(struct mp2650_charger *chg, u32 ua)
{
	unsigned int val;
	int ret;

	ua = clamp(ua, (u32)MP2650_MIN_IIN_UA, chg->max_iin_ua);
	val = ua / MP2650_IIN_LIM_STEP_UA;

	ret = regmap_update_bits(chg->regmap, MP2650_REG_IIN_LIM_1,
				 MP2650_IIN_LIM_MASK,
				 FIELD_PREP(MP2650_IIN_LIM_MASK, val));
	if (ret)
		return ret;

	return regmap_update_bits(chg->regmap, MP2650_REG_IIN_LIM_2,
				  MP2650_IIN_LIM_MASK,
				  FIELD_PREP(MP2650_IIN_LIM_MASK, val));
}

static int mp2650_set_vindpm(struct mp2650_charger *chg, u32 uv)
{
	unsigned int val;

	uv = clamp(uv, 4000000U, 14000000U);
	val = uv / MP2650_VINDPM_STEP_UV;
	return regmap_write(chg->regmap, MP2650_REG_VINDPM, val);
}

static int mp2650_set_charge_current(struct mp2650_charger *chg, u32 ua)
{
	unsigned int val;

	ua = clamp(ua, (u32)MP2650_MIN_ICHG_UA, chg->max_ichg_ua);
	val = ua / MP2650_ICHG_STEP_UA;
	return regmap_write(chg->regmap, MP2650_REG_ICHG, val);
}

static int mp2650_set_charge_voltage(struct mp2650_charger *chg, u32 uv_2s)
{
	u32 cell_uv;
	unsigned int val;

	uv_2s = clamp(uv_2s, (u32)MP2650_MIN_VREG_2S_UV, chg->max_vreg_uv);
	cell_uv = uv_2s / 2;
	if (cell_uv < MP2650_VREG_CELL_BASE_UV)
		cell_uv = MP2650_VREG_CELL_BASE_UV;

	val = (cell_uv - MP2650_VREG_CELL_BASE_UV) / MP2650_VREG_CELL_STEP_UV;
	return regmap_update_bits(chg->regmap, MP2650_REG_VREG,
				  MP2650_VREG_MASK,
				  FIELD_PREP(MP2650_VREG_MASK, val));
}

static int mp2650_set_charge_enable(struct mp2650_charger *chg, bool enable)
{
	return regmap_update_bits(chg->regmap, MP2650_REG_CTRL0,
				  MP2650_CTRL0_CHG_EN,
				  enable ? MP2650_CTRL0_CHG_EN : 0);
}

static int mp2650_hw_init(struct mp2650_charger *chg)
{
	unsigned int stat, rev;
	int ret;

	ret = regmap_read(chg->regmap, MP2650_REG_REV, &rev);
	if (ret)
		return ret;
	ret = regmap_read(chg->regmap, MP2650_REG_STAT, &stat);
	if (ret)
		return ret;

	dev_info(chg->dev, "MP2650 detected (rev=0x%02x, stat=0x%02x)\n", rev, stat);

	/* Ensure 2-cell pack configuration */
	ret = regmap_update_bits(chg->regmap, MP2650_REG_OTG_CURR_CELL,
				 MP2650_CELL_CNT_MASK,
				 FIELD_PREP(MP2650_CELL_CNT_MASK, MP2650_CELL_CNT_2S));
	if (ret)
		return ret;

	/* Disable watchdog reset on suspend; keep 12h safety timer & termination */
	ret = regmap_update_bits(chg->regmap, MP2650_REG_CTRL1,
				 MP2650_CTRL1_WDT_MASK | MP2650_CTRL1_TERM_EN |
				 MP2650_CTRL1_TIMER_EN | MP2650_CTRL1_TIMER_MASK,
				 MP2650_CTRL1_WDT_DISABLE | MP2650_CTRL1_TERM_EN |
				 MP2650_CTRL1_TIMER_EN | MP2650_CTRL1_TIMER_12H);
	if (ret)
		return ret;

	/* Enable continuous internal ADC telemetry */
	ret = regmap_update_bits(chg->regmap, MP2650_REG_CTRL2,
				 MP2650_CTRL2_ADC_EN_MASK, MP2650_CTRL2_ADC_EN);
	if (ret)
		return ret;

	/* Safe 2S voltage/current defaults */
	ret = mp2650_set_charge_voltage(chg, chg->max_vreg_uv);
	if (ret)
		return ret;

	ret = regmap_update_bits(chg->regmap, MP2650_REG_IPRE_ITERM,
				 MP2650_IPRE_MASK | MP2650_ITERM_MASK,
				 FIELD_PREP(MP2650_IPRE_MASK, 6) |
				 FIELD_PREP(MP2650_ITERM_MASK, 2));
	if (ret)
		return ret;

	ret = mp2650_set_vindpm(chg, 4400000);
	if (ret)
		return ret;

	ret = mp2650_set_input_current_limit(chg, chg->max_iin_ua);
	if (ret)
		return ret;

	ret = mp2650_set_charge_current(chg, min(chg->max_ichg_ua, (u32)MP2650_DEFAULT_ICHG_UA));
	if (ret)
		return ret;

	return mp2650_set_charge_enable(chg, !chg->inhibit_charge);
}

struct mp2650_tcpm_state {
	bool found;
	int online;
	int vmax_uv;
	int imax_ua;
};

static int mp2650_match_tcpm_psy(struct device *dev, void *data)
{
	struct mp2650_tcpm_state *st = data;
	struct power_supply *psy = dev_get_drvdata(dev);
	union power_supply_propval val;

	if (!psy || !psy->desc || !psy->desc->name)
		return 0;
	if (strncmp(psy->desc->name, "tcpm-source-psy-", 16) != 0)
		return 0;

	st->found = true;
	if (!power_supply_get_property(psy, POWER_SUPPLY_PROP_ONLINE, &val))
		st->online = val.intval;
	if (!power_supply_get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_MAX, &val))
		st->vmax_uv = val.intval;
	if (!power_supply_get_property(psy, POWER_SUPPLY_PROP_CURRENT_MAX, &val))
		st->imax_ua = val.intval;
	return 1;
}

static void mp2650_supply_work(struct work_struct *work)
{
	struct mp2650_charger *chg =
		container_of(work, struct mp2650_charger, supply_work.work);
	struct mp2650_tcpm_state st = {};
	u32 vindpm_uv, iin_ua, ichg_ua;
	u64 power_mw;

	power_supply_for_each_device(&st, mp2650_match_tcpm_psy);
	if (!st.found || !st.online) {
		if (chg->psy)
			power_supply_changed(chg->psy);
		return;
	}

	if (st.vmax_uv >= 11500000)
		vindpm_uv = 11000000;
	else if (st.vmax_uv >= 8500000)
		vindpm_uv = 8200000;
	else
		vindpm_uv = 4400000;

	iin_ua = st.imax_ua > 0 ? st.imax_ua : MP2650_MIN_IIN_UA;

	/*
	 * Buck-boost 2S pack charge current derived from negotiated input power
	 * (~88% efficiency at ~8.2 V pack voltage), capped at DT max.
	 */
	power_mw = div_u64((u64)max(st.vmax_uv, 5000000) * iin_ua, 1000000000ULL);
	ichg_ua = (u32)div_u64(power_mw * 880ULL, 8200ULL) * 1000U;
	ichg_ua = clamp(ichg_ua, (u32)MP2650_MIN_ICHG_UA, chg->max_ichg_ua);

	mutex_lock(&chg->lock);
	mp2650_set_vindpm(chg, vindpm_uv);
	mp2650_set_input_current_limit(chg, iin_ua);
	mp2650_set_charge_current(chg, ichg_ua);
	mp2650_set_charge_enable(chg, !chg->inhibit_charge);
	mutex_unlock(&chg->lock);

	if (chg->psy)
		power_supply_changed(chg->psy);
}

static int mp2650_psy_notifier(struct notifier_block *nb, unsigned long event, void *data)
{
	struct mp2650_charger *chg = container_of(nb, struct mp2650_charger, psy_nb);
	struct power_supply *psy = data;

	if (event != PSY_EVENT_PROP_CHANGED || !psy || psy == chg->psy)
		return NOTIFY_OK;
	if (!psy->desc || !psy->desc->name)
		return NOTIFY_OK;
	if (strncmp(psy->desc->name, "tcpm-source-psy-", 16) != 0)
		return NOTIFY_OK;

	mod_delayed_work(system_wq, &chg->supply_work, msecs_to_jiffies(100));
	return NOTIFY_OK;
}

static int mp2650_get_property(struct power_supply *psy,
			       enum power_supply_property psp,
			       union power_supply_propval *val)
{
	struct mp2650_charger *chg = power_supply_get_drvdata(psy);
	unsigned int regval;
	u32 adc;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = regmap_read(chg->regmap, MP2650_REG_STAT, &regval);
		if (ret)
			return ret;
		val->intval = !(regval & MP2650_STAT_VIN_PG_N);
		return 0;

	case POWER_SUPPLY_PROP_STATUS:
		ret = regmap_read(chg->regmap, MP2650_REG_STAT, &regval);
		if (ret)
			return ret;
		if (regval & MP2650_STAT_VIN_PG_N) {
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			return 0;
		}
		switch (FIELD_GET(MP2650_STAT_CHG_MASK, regval)) {
		case MP2650_STAT_CHG_PRE:
		case MP2650_STAT_CHG_FAST:
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
			break;
		case MP2650_STAT_CHG_DONE:
			val->intval = POWER_SUPPLY_STATUS_FULL;
			break;
		default:
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		}
		return 0;

	case POWER_SUPPLY_PROP_HEALTH:
		ret = regmap_read(chg->regmap, MP2650_REG_FAULT, &regval);
		if (ret)
			return ret;
		if (regval & MP2650_FAULT_BAT_OVP) {
			val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		} else {
			unsigned int ntc = FIELD_GET(MP2650_FAULT_NTC_MASK, regval);
			unsigned int cf = FIELD_GET(MP2650_FAULT_CHG_MASK, regval);

			if (ntc == MP2650_FAULT_NTC_HOT || cf == MP2650_FAULT_CHG_THERM)
				val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
			else if (ntc == MP2650_FAULT_NTC_COLD)
				val->intval = POWER_SUPPLY_HEALTH_COLD;
			else if (cf == MP2650_FAULT_CHG_TIMEOUT)
				val->intval = POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE;
			else
				val->intval = POWER_SUPPLY_HEALTH_GOOD;
		}
		return 0;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = mp2650_read_adc10(chg, MP2650_REG_ADC_VBUS_LO, &adc);
		if (ret)
			return ret;
		val->intval = adc * 25000;
		return 0;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = mp2650_read_adc10(chg, MP2650_REG_ADC_ICHG_LO, &adc);
		if (ret)
			return ret;
		val->intval = adc * 12500;
		return 0;

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = regmap_read(chg->regmap, MP2650_REG_IIN_LIM_1, &regval);
		if (ret)
			return ret;
		val->intval = FIELD_GET(MP2650_IIN_LIM_MASK, regval) * MP2650_IIN_LIM_STEP_UA;
		return 0;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = regmap_read(chg->regmap, MP2650_REG_ICHG, &regval);
		if (ret)
			return ret;
		val->intval = regval * MP2650_ICHG_STEP_UA;
		return 0;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = chg->max_ichg_ua;
		return 0;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = regmap_read(chg->regmap, MP2650_REG_VREG, &regval);
		if (ret)
			return ret;
		val->intval = (MP2650_VREG_CELL_BASE_UV +
			       FIELD_GET(MP2650_VREG_MASK, regval) * MP2650_VREG_CELL_STEP_UV) * 2;
		return 0;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		val->intval = chg->max_vreg_uv;
		return 0;

	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		val->intval = chg->inhibit_charge ?
			POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE :
			POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO;
		return 0;

	default:
		return -EINVAL;
	}
}

static int mp2650_set_property(struct power_supply *psy,
			       enum power_supply_property psp,
			       const union power_supply_propval *val)
{
	struct mp2650_charger *chg = power_supply_get_drvdata(psy);
	int ret;

	mutex_lock(&chg->lock);
	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = mp2650_set_input_current_limit(chg, val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = mp2650_set_charge_current(chg, val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = mp2650_set_charge_voltage(chg, val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		if (val->intval == POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO) {
			chg->inhibit_charge = false;
			ret = mp2650_set_charge_enable(chg, true);
		} else if (val->intval == POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE) {
			chg->inhibit_charge = true;
			ret = mp2650_set_charge_enable(chg, false);
		} else {
			ret = -EINVAL;
		}
		break;
	default:
		ret = -EINVAL;
		break;
	}
	mutex_unlock(&chg->lock);

	if (!ret)
		power_supply_changed(psy);
	return ret;
}

static int mp2650_property_is_writeable(struct power_supply *psy,
					enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		return 1;
	default:
		return 0;
	}
}

static enum power_supply_property mp2650_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR,
};

static const struct power_supply_desc mp2650_psy_desc = {
	.name = "mp2650-charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = mp2650_properties,
	.num_properties = ARRAY_SIZE(mp2650_properties),
	.get_property = mp2650_get_property,
	.set_property = mp2650_set_property,
	.property_is_writeable = mp2650_property_is_writeable,
	.charge_behaviours = BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO) |
			     BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE),
};

static void mp2650_unreg_notifier(void *data)
{
	struct mp2650_charger *chg = data;

	power_supply_unreg_notifier(&chg->psy_nb);
}

static int mp2650_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct power_supply_config psy_cfg = {};
	struct mp2650_charger *chg;
	int ret;

	chg = devm_kzalloc(dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	chg->dev = dev;
	mutex_init(&chg->lock);

	chg->regmap = devm_regmap_init_i2c(client, &mp2650_regmap_config);
	if (IS_ERR(chg->regmap))
		return dev_err_probe(dev, PTR_ERR(chg->regmap), "failed to init regmap\n");

	chg->otg_en_gpio = devm_gpiod_get_optional(dev, "otg-enable", GPIOD_OUT_LOW);
	if (IS_ERR(chg->otg_en_gpio))
		return dev_err_probe(dev, PTR_ERR(chg->otg_en_gpio), "failed to get otg-enable gpio\n");

	chg->max_vreg_uv = MP2650_DEFAULT_VREG_2S_UV;
	chg->max_ichg_ua = MP2650_MAX_ICHG_UA;
	chg->max_iin_ua = MP2650_DEFAULT_IIN_UA;

	device_property_read_u32(dev, "constant-charge-voltage-max-microvolt", &chg->max_vreg_uv);
	device_property_read_u32(dev, "constant-charge-current-max-microamp", &chg->max_ichg_ua);
	device_property_read_u32(dev, "input-current-limit-microamp", &chg->max_iin_ua);

	chg->max_vreg_uv = clamp(chg->max_vreg_uv, (u32)MP2650_MIN_VREG_2S_UV,
				 (u32)MP2650_MAX_VREG_2S_UV);
	chg->max_ichg_ua = clamp(chg->max_ichg_ua, (u32)MP2650_MIN_ICHG_UA,
				 (u32)MP2650_MAX_ICHG_UA);
	chg->max_iin_ua = clamp(chg->max_iin_ua, (u32)MP2650_MIN_IIN_UA,
				(u32)MP2650_MAX_IIN_UA);

	ret = mp2650_hw_init(chg);
	if (ret)
		return dev_err_probe(dev, ret, "failed to initialize MP2650\n");

	ret = devm_delayed_work_autocancel(dev, &chg->supply_work, mp2650_supply_work);
	if (ret)
		return ret;

	psy_cfg.drv_data = chg;
	psy_cfg.fwnode = dev_fwnode(dev);
	chg->psy = devm_power_supply_register(dev, &mp2650_psy_desc, &psy_cfg);
	if (IS_ERR(chg->psy))
		return dev_err_probe(dev, PTR_ERR(chg->psy), "failed to register power supply\n");

	chg->psy_nb.notifier_call = mp2650_psy_notifier;
	ret = power_supply_reg_notifier(&chg->psy_nb);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, mp2650_unreg_notifier, chg);
	if (ret)
		return ret;

	/* Sync with initial Type-C/PD state once TCPM settles */
	schedule_delayed_work(&chg->supply_work, msecs_to_jiffies(500));
	return 0;
}

static const struct of_device_id mp2650_of_match[] = {
	{ .compatible = "mps,mp2650" },
	{ }
};
MODULE_DEVICE_TABLE(of, mp2650_of_match);

static const struct i2c_device_id mp2650_i2c_id[] = {
	{ "mp2650" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mp2650_i2c_id);

static struct i2c_driver mp2650_driver = {
	.driver = {
		.name = "mp2650-charger",
		.of_match_table = mp2650_of_match,
	},
	.probe = mp2650_probe,
	.id_table = mp2650_i2c_id,
};
module_i2c_driver(mp2650_driver);

MODULE_DESCRIPTION("MPS MP2650 2S Buck-Boost Battery Charger driver");
MODULE_LICENSE("GPL");
