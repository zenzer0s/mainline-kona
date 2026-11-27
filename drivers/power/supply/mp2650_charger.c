// SPDX-License-Identifier: GPL-2.0-only
/*
 * Monolithic Power Systems MP2650 2-cell Buck-Boost Battery Charger driver
 *
 * Copyright (C) 2026 The PenguinOS Project
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/devm-helpers.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/thermal.h>
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
#define MP2650_REG_IIN_LIM_2_TIME	0x10
#define MP2650_REG_IIN_LIM_TOTAL_TIME	0x11
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
#define MP2650_REG_VSYS_RIPPLE		0x2d
#define MP2650_REG_BURST_MODE		0x3e
#define MP2650_REG_OTG_WA		0x3f
#define MP2650_REG_TEST_UNLOCK		0x53

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

/* REG04: Per-cell charge full voltage (3.7125 V + N * 12.5 mV) & recharge */
#define MP2650_RECHG_MASK		BIT(0)
#define MP2650_RECHG_100MV		0
#define MP2650_VREG_MASK		GENMASK(6, 1)
#define MP2650_VREG_CELL_BASE_UV	3712500
#define MP2650_VREG_CELL_STEP_UV	12500

/* REG06: OTG output voltage (option + 2nd step * 50 mV) */
#define MP2650_OTG_VOL_OPT_MASK		GENMASK(6, 4)
#define MP2650_OTG_VOL_OPT_4750MV	0
#define MP2650_OTG_VOL_2ND_MASK		GENMASK(3, 0)
#define MP2650_OTG_VOL_2ND_250MV	5	/* 4750 + 250 = 5000 mV */

/* REG07: Cell count, precharge threshold, OTG current limit */
#define MP2650_OTG_ILIM_MASK		GENMASK(3, 0)
#define MP2650_OTG_ILIM_1250MA		5	/* 5 * 250 mA = 1250 mA */
#define MP2650_PRECHG_THR_MASK		GENMASK(5, 4)
#define MP2650_PRECHG_THR_7200MV	3
#define MP2650_CELL_CNT_MASK		GENMASK(7, 6)
#define MP2650_CELL_CNT_2S		0

/* REG08: Control 0 */
#define MP2650_CTRL0_LEARN_EN		BIT(3)
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

/* REG0A: NTC control */
#define MP2650_NTC_CTRL_MASK		GENMASK(5, 4)
#define MP2650_NTC_CTRL_DISABLE		3

/* REG0B: Control 2 */
#define MP2650_CTRL2_ADC_EN_MASK	GENMASK(1, 0)
#define MP2650_CTRL2_ADC_DISABLE	0x0
#define MP2650_CTRL2_ADC_EN		0x3
#define MP2650_CTRL2_SW_FREQ_MASK	GENMASK(4, 3)
#define MP2650_CTRL2_SW_FREQ_800K	1

/* REG13: Status (RO) */
#define MP2650_STAT_VIN_PG		BIT(1)
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

/* Safe 2S operating limits for OnePlus 8T (BLP801 4.45V-class 2S pack) */
#define MP2650_MIN_IIN_UA		500000
#define MP2650_MAX_IIN_UA		3000000
#define MP2650_MIN_ICHG_UA		500000
#define MP2650_MAX_ICHG_UA		3000000
#define MP2650_MIN_VREG_2S_UV		7600000
#define MP2650_MAX_VREG_2S_UV		8900000
#define MP2650_DEFAULT_VREG_2S_UV	8875000	/* 4.4375 V/cell (stock 4435 mV) */
#define MP2650_WARM_VREG_2S_UV		8250000	/* 4.125 V/cell (stock warm 4130 mV) */
#define MP2650_DEFAULT_ICHG_UA		2000000
#define MP2650_DEFAULT_IIN_UA		3000000
#define MP2650_MONITOR_INTERVAL_MS	15000

/* Stock OnePlus 8T JEITA thresholds (in tenths of degree C) */
#define MP2650_JEITA_COLD_TEMP		0	/* < 0.0 C: stop */
#define MP2650_JEITA_COLD_HYST		20	/* >= 2.0 C: resume */
#define MP2650_JEITA_LITTLE_COLD_TEMP	50	/* 0.0 - 5.0 C: 950 mA */
#define MP2650_JEITA_COOL_TEMP		120	/* 5.0 - 12.0 C: 2000 mA */
#define MP2650_JEITA_WARM_HYST		430	/* < 43.0 C: exit warm */
#define MP2650_JEITA_WARM_TEMP		450	/* 45.0 - 50.0 C: 600 mA, 8.25 V */
#define MP2650_JEITA_HOT_HYST		480	/* < 48.0 C: exit hot */
#define MP2650_JEITA_HOT_TEMP		500	/* >= 50.0 C: stop */

enum mp2650_jeita_zone {
	MP2650_JEITA_NORMAL = 0,
	MP2650_JEITA_COLD,
	MP2650_JEITA_LITTLE_COLD,
	MP2650_JEITA_COOL,
	MP2650_JEITA_WARM,
	MP2650_JEITA_HOT,
};

/* Skin thermal cooling table (0 = no throttle, 5 = pause charging) */
static const u32 mp2650_cooling_ichg_ua[] = {
	3000000,
	2000000,
	1500000,
	1000000,
	500000,
	0,
};

struct mp2650_charger {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct regulator_dev *otg_rdev;
	struct thermal_cooling_device *cdev;
	struct gpio_desc *otg_en_gpio;
	struct notifier_block psy_nb;
	struct delayed_work supply_work;
	struct delayed_work monitor_work;
	struct mutex lock;

	u32 max_vreg_uv;
	u32 max_ichg_ua;
	u32 max_iin_ua;
	u32 user_vreg_uv;
	u32 user_ichg_ua;
	u32 tcpm_ichg_ua;

	enum mp2650_jeita_zone jeita_zone;
	unsigned long cool_state;
	int last_stat;

	bool online;
	bool adc_enabled;
	bool otg_enabled;
	bool inhibit_charge;
};

static const struct regmap_config mp2650_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x5f,
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

static int mp2650_set_adc_enable_locked(struct mp2650_charger *chg, bool enable)
{
	int ret;

	if (chg->adc_enabled == enable)
		return 0;

	ret = regmap_update_bits(chg->regmap, MP2650_REG_CTRL2,
				 MP2650_CTRL2_ADC_EN_MASK,
				 enable ? MP2650_CTRL2_ADC_EN : MP2650_CTRL2_ADC_DISABLE);
	if (!ret)
		chg->adc_enabled = enable;
	return ret;
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
				  MP2650_VREG_MASK | MP2650_RECHG_MASK,
				  FIELD_PREP(MP2650_VREG_MASK, val) | MP2650_RECHG_100MV);
}

static int mp2650_set_charge_enable(struct mp2650_charger *chg, bool enable)
{
	return regmap_update_bits(chg->regmap, MP2650_REG_CTRL0,
				  MP2650_CTRL0_CHG_EN,
				  enable ? MP2650_CTRL0_CHG_EN : 0);
}

static int mp2650_apply_limits_locked(struct mp2650_charger *chg)
{
	u32 jeita_vreg_uv = chg->max_vreg_uv;
	u32 jeita_ichg_ua = chg->max_ichg_ua;
	u32 cool_ichg_ua;
	u32 eff_vreg_uv, eff_ichg_ua;
	bool jeita_stop = false;
	bool chg_allow;
	int ret;

	switch (chg->jeita_zone) {
	case MP2650_JEITA_COLD:
	case MP2650_JEITA_HOT:
		jeita_stop = true;
		break;
	case MP2650_JEITA_LITTLE_COLD:
		jeita_ichg_ua = 950000;
		break;
	case MP2650_JEITA_COOL:
		jeita_ichg_ua = 2000000;
		break;
	case MP2650_JEITA_WARM:
		jeita_vreg_uv = MP2650_WARM_VREG_2S_UV;
		jeita_ichg_ua = 600000;
		break;
	case MP2650_JEITA_NORMAL:
	default:
		break;
	}

	cool_ichg_ua = mp2650_cooling_ichg_ua[chg->cool_state];
	eff_vreg_uv = min3(chg->max_vreg_uv, chg->user_vreg_uv, jeita_vreg_uv);
	eff_ichg_ua = min(min(chg->max_ichg_ua, chg->user_ichg_ua),
			  min(chg->tcpm_ichg_ua, jeita_ichg_ua));
	if (cool_ichg_ua > 0)
		eff_ichg_ua = min(eff_ichg_ua, cool_ichg_ua);

	chg_allow = !chg->otg_enabled && !chg->inhibit_charge &&
		    !jeita_stop && (cool_ichg_ua > 0);

	ret = mp2650_set_charge_voltage(chg, eff_vreg_uv);
	if (ret)
		return ret;

	ret = mp2650_set_charge_current(chg, eff_ichg_ua);
	if (ret)
		return ret;

	return mp2650_set_charge_enable(chg, chg_allow);
}

static int mp2650_burst_mode_enable_locked(struct mp2650_charger *chg, bool enable)
{
	int ret;

	ret = regmap_write(chg->regmap, MP2650_REG_TEST_UNLOCK, 0x95);
	if (ret)
		return ret;

	ret = regmap_write(chg->regmap, MP2650_REG_BURST_MODE, enable ? 0x00 : 0x80);
	regmap_write(chg->regmap, MP2650_REG_TEST_UNLOCK, 0x00);
	return ret;
}

static int mp2650_otg_enable(struct regulator_dev *rdev)
{
	struct mp2650_charger *chg = rdev_get_drvdata(rdev);
	unsigned int val;
	int ret, retry;

	mutex_lock(&chg->lock);
	if (chg->otg_enabled) {
		mutex_unlock(&chg->lock);
		return 0;
	}

	mp2650_burst_mode_enable_locked(chg, true);

	/* Ensure charging and WDT are disabled before boosting VBUS */
	mp2650_set_charge_enable(chg, false);
	regmap_update_bits(chg->regmap, MP2650_REG_CTRL1,
			   MP2650_CTRL1_WDT_MASK, MP2650_CTRL1_WDT_DISABLE);

	/* Configure 5.0 V output and 1250 mA OTG current limit */
	regmap_update_bits(chg->regmap, MP2650_REG_OTG_VOL,
			   MP2650_OTG_VOL_OPT_MASK | MP2650_OTG_VOL_2ND_MASK,
			   FIELD_PREP(MP2650_OTG_VOL_OPT_MASK, MP2650_OTG_VOL_OPT_4750MV) |
			   FIELD_PREP(MP2650_OTG_VOL_2ND_MASK, MP2650_OTG_VOL_2ND_250MV));
	regmap_update_bits(chg->regmap, MP2650_REG_OTG_CURR_CELL,
			   MP2650_OTG_ILIM_MASK,
			   FIELD_PREP(MP2650_OTG_ILIM_MASK, MP2650_OTG_ILIM_1250MA));

	/* Stock OTG startup errata: assert REG3F bit 5 while turning on OTG */
	regmap_write(chg->regmap, MP2650_REG_TEST_UNLOCK, 0x95);
	regmap_write(chg->regmap, MP2650_REG_OTG_WA, 0x20);
	regmap_write(chg->regmap, MP2650_REG_TEST_UNLOCK, 0x00);

	if (chg->otg_en_gpio)
		gpiod_set_value_cansleep(chg->otg_en_gpio, 1);

	ret = regmap_update_bits(chg->regmap, MP2650_REG_CTRL0,
				 MP2650_CTRL0_OTG_EN, MP2650_CTRL0_OTG_EN);
	if (ret) {
		if (chg->otg_en_gpio)
			gpiod_set_value_cansleep(chg->otg_en_gpio, 0);
		goto clear_wa;
	}

	msleep(10);
	chg->otg_enabled = true;

clear_wa:
	/* Must clear REG3F bit 5 after OTG ramp so charging resumes cleanly later */
	for (retry = 0; retry < 3; retry++) {
		regmap_write(chg->regmap, MP2650_REG_TEST_UNLOCK, 0x95);
		regmap_write(chg->regmap, MP2650_REG_OTG_WA, 0x00);
		if (!regmap_read(chg->regmap, MP2650_REG_OTG_WA, &val) && val == 0)
			break;
	}
	regmap_write(chg->regmap, MP2650_REG_TEST_UNLOCK, 0x00);

	mutex_unlock(&chg->lock);
	return ret;
}

static int mp2650_otg_disable(struct regulator_dev *rdev)
{
	struct mp2650_charger *chg = rdev_get_drvdata(rdev);
	int ret;

	mutex_lock(&chg->lock);
	if (chg->otg_en_gpio)
		gpiod_set_value_cansleep(chg->otg_en_gpio, 0);

	ret = regmap_update_bits(chg->regmap, MP2650_REG_CTRL0,
				 MP2650_CTRL0_OTG_EN, 0);
	chg->otg_enabled = false;
	mp2650_apply_limits_locked(chg);
	mutex_unlock(&chg->lock);

	return ret;
}

static int mp2650_otg_is_enabled(struct regulator_dev *rdev)
{
	struct mp2650_charger *chg = rdev_get_drvdata(rdev);
	unsigned int val;
	int ret;

	ret = regmap_read(chg->regmap, MP2650_REG_CTRL0, &val);
	if (ret)
		return ret;

	return !!(val & MP2650_CTRL0_OTG_EN);
}

static const struct regulator_ops mp2650_otg_reg_ops = {
	.enable = mp2650_otg_enable,
	.disable = mp2650_otg_disable,
	.is_enabled = mp2650_otg_is_enabled,
};

static const struct regulator_desc mp2650_otg_reg_desc = {
	.name = "mp2650-otg-vbus",
	.of_match = of_match_ptr("usb-otg-vbus"),
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.ops = &mp2650_otg_reg_ops,
	.fixed_uV = 5000000,
	.n_voltages = 1,
};

static const struct regulator_init_data mp2650_otg_init_data = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
};

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

	/* Stock MP2650/MP2762A errata: shortest Ilim2 duration & VSYS ripple fix */
	regmap_write(chg->regmap, MP2650_REG_IIN_LIM_2_TIME, 0x01);
	regmap_write(chg->regmap, MP2650_REG_IIN_LIM_TOTAL_TIME, 0xff);
	regmap_write(chg->regmap, MP2650_REG_VSYS_RIPPLE, 0x0f);
	mp2650_burst_mode_enable_locked(chg, true);

	/* Ensure 2S pack, 7.2 V precharge threshold, and 1250 mA OTG current limit */
	ret = regmap_update_bits(chg->regmap, MP2650_REG_OTG_CURR_CELL,
				 MP2650_CELL_CNT_MASK | MP2650_PRECHG_THR_MASK |
				 MP2650_OTG_ILIM_MASK,
				 FIELD_PREP(MP2650_CELL_CNT_MASK, MP2650_CELL_CNT_2S) |
				 FIELD_PREP(MP2650_PRECHG_THR_MASK, MP2650_PRECHG_THR_7200MV) |
				 FIELD_PREP(MP2650_OTG_ILIM_MASK, MP2650_OTG_ILIM_1250MA));
	if (ret)
		return ret;

	/* Configure 5.0 V default OTG output */
	ret = regmap_update_bits(chg->regmap, MP2650_REG_OTG_VOL,
				 MP2650_OTG_VOL_OPT_MASK | MP2650_OTG_VOL_2ND_MASK,
				 FIELD_PREP(MP2650_OTG_VOL_OPT_MASK, MP2650_OTG_VOL_OPT_4750MV) |
				 FIELD_PREP(MP2650_OTG_VOL_2ND_MASK, MP2650_OTG_VOL_2ND_250MV));
	if (ret)
		return ret;

	/* Disable WDT reset on suspend; keep 12h safety timer & termination */
	ret = regmap_update_bits(chg->regmap, MP2650_REG_CTRL1,
				 MP2650_CTRL1_WDT_MASK | MP2650_CTRL1_TERM_EN |
				 MP2650_CTRL1_TIMER_EN | MP2650_CTRL1_TIMER_MASK,
				 MP2650_CTRL1_WDT_DISABLE | MP2650_CTRL1_TERM_EN |
				 MP2650_CTRL1_TIMER_EN | MP2650_CTRL1_TIMER_12H);
	if (ret)
		return ret;

	/* Battery temp is monitored via bq28z610 fuel gauge; disable unwired NTC pin */
	ret = regmap_update_bits(chg->regmap, MP2650_REG_NTC,
				 MP2650_NTC_CTRL_MASK,
				 FIELD_PREP(MP2650_NTC_CTRL_MASK, MP2650_NTC_CTRL_DISABLE));
	if (ret)
		return ret;

	/* Set 800 kHz switching frequency; start with ADC gated off until VBUS online */
	ret = regmap_update_bits(chg->regmap, MP2650_REG_CTRL2,
				 MP2650_CTRL2_SW_FREQ_MASK | MP2650_CTRL2_ADC_EN_MASK,
				 FIELD_PREP(MP2650_CTRL2_SW_FREQ_MASK, MP2650_CTRL2_SW_FREQ_800K) |
				 MP2650_CTRL2_ADC_DISABLE);
	if (ret)
		return ret;
	chg->adc_enabled = false;

	/* Clear HIZ/learn mode and ensure OTG is off on boot */
	ret = regmap_update_bits(chg->regmap, MP2650_REG_CTRL0,
				 MP2650_CTRL0_LEARN_EN | MP2650_CTRL0_OTG_EN, 0);
	if (ret)
		return ret;

	/* Stock precharge 480 mA (8 * 60 mA) and termination 100 mA (stock 115 mA -> 1) */
	ret = regmap_update_bits(chg->regmap, MP2650_REG_IPRE_ITERM,
				 MP2650_IPRE_MASK | MP2650_ITERM_MASK,
				 FIELD_PREP(MP2650_IPRE_MASK, 8) |
				 FIELD_PREP(MP2650_ITERM_MASK, 1));
	if (ret)
		return ret;

	ret = mp2650_set_vindpm(chg, 4400000);
	if (ret)
		return ret;

	ret = mp2650_set_input_current_limit(chg, chg->max_iin_ua);
	if (ret)
		return ret;

	return mp2650_apply_limits_locked(chg);
}

struct mp2650_tcpm_state {
	bool found;
	int online;
	int vmax_uv;
	int imax_ua;
};

static int mp2650_match_tcpm_psy(struct power_supply *psy, void *data)
{
	struct mp2650_tcpm_state *st = data;
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

struct mp2650_batt_temp_query {
	struct power_supply *self;
	bool found;
	int temp_decidegc;
};

static int mp2650_match_batt_psy(struct power_supply *psy, void *data)
{
	struct mp2650_batt_temp_query *q = data;
	union power_supply_propval val;

	if (!psy || psy == q->self || !psy->desc)
		return 0;
	if (psy->desc->type != POWER_SUPPLY_TYPE_BATTERY)
		return 0;

	if (!power_supply_get_property(psy, POWER_SUPPLY_PROP_TEMP, &val)) {
		q->found = true;
		q->temp_decidegc = val.intval;
		return 1;
	}
	return 0;
}

static bool mp2650_read_batt_temp(struct mp2650_charger *chg, int *temp_decidegc)
{
	struct mp2650_batt_temp_query q = { .self = chg->psy };

	power_supply_for_each_psy(&q, mp2650_match_batt_psy);
	if (!q.found)
		return false;

	*temp_decidegc = q.temp_decidegc;
	return true;
}

static enum mp2650_jeita_zone mp2650_eval_jeita(enum mp2650_jeita_zone prev, int temp)
{
	if (temp >= MP2650_JEITA_HOT_TEMP ||
	    (prev == MP2650_JEITA_HOT && temp >= MP2650_JEITA_HOT_HYST))
		return MP2650_JEITA_HOT;

	if (temp >= MP2650_JEITA_WARM_TEMP ||
	    (prev == MP2650_JEITA_WARM && temp >= MP2650_JEITA_WARM_HYST))
		return MP2650_JEITA_WARM;

	if (temp < MP2650_JEITA_COLD_TEMP ||
	    (prev == MP2650_JEITA_COLD && temp < MP2650_JEITA_COLD_HYST))
		return MP2650_JEITA_COLD;

	if (temp < MP2650_JEITA_LITTLE_COLD_TEMP)
		return MP2650_JEITA_LITTLE_COLD;

	if (temp < MP2650_JEITA_COOL_TEMP)
		return MP2650_JEITA_COOL;

	return MP2650_JEITA_NORMAL;
}

static void mp2650_monitor_work(struct work_struct *work)
{
	struct mp2650_charger *chg =
		container_of(work, struct mp2650_charger, monitor_work.work);
	unsigned int stat = 0;
	enum mp2650_jeita_zone new_zone;
	bool vin_pg, changed = false;
	int temp;

	if (regmap_read(chg->regmap, MP2650_REG_STAT, &stat))
		return;

	vin_pg = !!(stat & MP2650_STAT_VIN_PG);

	mutex_lock(&chg->lock);
	if (!vin_pg && !chg->online) {
		mp2650_set_adc_enable_locked(chg, false);
		mutex_unlock(&chg->lock);
		return;
	}

	if (mp2650_read_batt_temp(chg, &temp)) {
		new_zone = mp2650_eval_jeita(chg->jeita_zone, temp);
		if (new_zone != chg->jeita_zone) {
			dev_info(chg->dev, "JEITA zone %d -> %d (temp=%d.%d C)\n",
				 chg->jeita_zone, new_zone, temp / 10, abs(temp % 10));
			chg->jeita_zone = new_zone;
			mp2650_apply_limits_locked(chg);
			changed = true;
		}
	}

	if ((int)(stat & (MP2650_STAT_VIN_PG | MP2650_STAT_CHG_MASK)) != chg->last_stat) {
		chg->last_stat = stat & (MP2650_STAT_VIN_PG | MP2650_STAT_CHG_MASK);
		changed = true;
	}
	mutex_unlock(&chg->lock);

	if (changed && chg->psy)
		power_supply_changed(chg->psy);

	schedule_delayed_work(&chg->monitor_work, msecs_to_jiffies(MP2650_MONITOR_INTERVAL_MS));
}

static void mp2650_supply_work(struct work_struct *work)
{
	struct mp2650_charger *chg =
		container_of(work, struct mp2650_charger, supply_work.work);
	struct mp2650_tcpm_state st = {};
	unsigned int stat = 0;
	u32 vindpm_uv, iin_ua, ichg_ua;
	u64 power_mw;
	bool vin_pg;
	int temp;

	power_supply_for_each_psy(&st, mp2650_match_tcpm_psy);
	regmap_read(chg->regmap, MP2650_REG_STAT, &stat);
	vin_pg = !!(stat & MP2650_STAT_VIN_PG);

	if ((!st.found || !st.online) && !vin_pg) {
		cancel_delayed_work_sync(&chg->monitor_work);
		mutex_lock(&chg->lock);
		chg->online = false;
		chg->jeita_zone = MP2650_JEITA_NORMAL;
		chg->tcpm_ichg_ua = min(chg->max_ichg_ua, (u32)MP2650_DEFAULT_ICHG_UA);
		chg->last_stat = stat & (MP2650_STAT_VIN_PG | MP2650_STAT_CHG_MASK);
		mp2650_set_adc_enable_locked(chg, false);
		mutex_unlock(&chg->lock);

		if (chg->psy)
			power_supply_changed(chg->psy);
		return;
	}

	if (st.found && st.online) {
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
	} else {
		vindpm_uv = 4400000;
		iin_ua = MP2650_MIN_IIN_UA;
		ichg_ua = MP2650_MIN_ICHG_UA;
	}

	mutex_lock(&chg->lock);
	chg->online = true;
	chg->tcpm_ichg_ua = ichg_ua;
	if (mp2650_read_batt_temp(chg, &temp))
		chg->jeita_zone = mp2650_eval_jeita(chg->jeita_zone, temp);
	mp2650_set_adc_enable_locked(chg, true);
	mp2650_set_vindpm(chg, vindpm_uv);
	mp2650_set_input_current_limit(chg, iin_ua);
	mp2650_apply_limits_locked(chg);
	chg->last_stat = stat & (MP2650_STAT_VIN_PG | MP2650_STAT_CHG_MASK);
	mutex_unlock(&chg->lock);

	mod_delayed_work(system_wq, &chg->monitor_work,
			 msecs_to_jiffies(MP2650_MONITOR_INTERVAL_MS));

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

static int mp2650_get_max_cooling_state(struct thermal_cooling_device *cdev,
					unsigned long *state)
{
	*state = ARRAY_SIZE(mp2650_cooling_ichg_ua) - 1;
	return 0;
}

static int mp2650_get_cur_cooling_state(struct thermal_cooling_device *cdev,
					unsigned long *state)
{
	struct mp2650_charger *chg = cdev->devdata;

	mutex_lock(&chg->lock);
	*state = chg->cool_state;
	mutex_unlock(&chg->lock);
	return 0;
}

static int mp2650_set_cur_cooling_state(struct thermal_cooling_device *cdev,
					unsigned long state)
{
	struct mp2650_charger *chg = cdev->devdata;
	int ret = 0;

	if (state >= ARRAY_SIZE(mp2650_cooling_ichg_ua))
		return -EINVAL;

	mutex_lock(&chg->lock);
	if (chg->cool_state != state) {
		chg->cool_state = state;
		ret = mp2650_apply_limits_locked(chg);
	}
	mutex_unlock(&chg->lock);

	if (!ret && chg->psy)
		power_supply_changed(chg->psy);
	return ret;
}

static const struct thermal_cooling_device_ops mp2650_cooling_ops = {
	.get_max_state = mp2650_get_max_cooling_state,
	.get_cur_state = mp2650_get_cur_cooling_state,
	.set_cur_state = mp2650_set_cur_cooling_state,
};

static ssize_t charging_enabled_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct mp2650_charger *chg = power_supply_get_drvdata(psy);
	bool enabled;

	mutex_lock(&chg->lock);
	enabled = !chg->inhibit_charge;
	mutex_unlock(&chg->lock);

	return sysfs_emit(buf, "%d\n", enabled ? 1 : 0);
}

static ssize_t charging_enabled_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct mp2650_charger *chg = power_supply_get_drvdata(psy);
	bool enable;
	int ret;

	ret = kstrtobool(buf, &enable);
	if (ret)
		return ret;

	mutex_lock(&chg->lock);
	chg->inhibit_charge = !enable;
	ret = mp2650_apply_limits_locked(chg);
	mutex_unlock(&chg->lock);

	if (ret)
		return ret;

	power_supply_changed(psy);
	return count;
}

static DEVICE_ATTR_RW(charging_enabled);

static struct attribute *mp2650_psy_attrs[] = {
	&dev_attr_charging_enabled.attr,
	NULL,
};

static const struct attribute_group mp2650_psy_attr_group = {
	.attrs = mp2650_psy_attrs,
};

static const struct attribute_group *mp2650_psy_attr_groups[] = {
	&mp2650_psy_attr_group,
	NULL,
};

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
		val->intval = !!(regval & MP2650_STAT_VIN_PG);
		return 0;

	case POWER_SUPPLY_PROP_STATUS:
		ret = regmap_read(chg->regmap, MP2650_REG_STAT, &regval);
		if (ret)
			return ret;
		if (!(regval & MP2650_STAT_VIN_PG)) {
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

			if (ntc == MP2650_FAULT_NTC_HOT || cf == MP2650_FAULT_CHG_THERM ||
			    chg->jeita_zone == MP2650_JEITA_HOT)
				val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
			else if (ntc == MP2650_FAULT_NTC_COLD ||
				 chg->jeita_zone == MP2650_JEITA_COLD)
				val->intval = POWER_SUPPLY_HEALTH_COLD;
			else if (cf == MP2650_FAULT_CHG_TIMEOUT)
				val->intval = POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE;
			else if (chg->jeita_zone == MP2650_JEITA_WARM)
				val->intval = POWER_SUPPLY_HEALTH_WARM;
			else
				val->intval = POWER_SUPPLY_HEALTH_GOOD;
		}
		return 0;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if (!chg->adc_enabled) {
			val->intval = 0;
			return 0;
		}
		ret = mp2650_read_adc10(chg, MP2650_REG_ADC_VBUS_LO, &adc);
		if (ret)
			return ret;
		val->intval = adc * 25000;
		return 0;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (!chg->adc_enabled) {
			val->intval = 0;
			return 0;
		}
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
		chg->user_ichg_ua = clamp((u32)val->intval, (u32)MP2650_MIN_ICHG_UA,
					  chg->max_ichg_ua);
		ret = mp2650_apply_limits_locked(chg);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		chg->user_vreg_uv = clamp((u32)val->intval, (u32)MP2650_MIN_VREG_2S_UV,
					  chg->max_vreg_uv);
		ret = mp2650_apply_limits_locked(chg);
		break;
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		if (val->intval == POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO) {
			chg->inhibit_charge = false;
			ret = mp2650_apply_limits_locked(chg);
		} else if (val->intval == POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE) {
			chg->inhibit_charge = true;
			ret = mp2650_apply_limits_locked(chg);
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
	struct regulator_config rcfg = {};
	struct mp2650_charger *chg;
	int ret;

	chg = devm_kzalloc(dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	chg->dev = dev;
	mutex_init(&chg->lock);
	i2c_set_clientdata(client, chg);

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
	chg->user_vreg_uv = chg->max_vreg_uv;
	chg->user_ichg_ua = chg->max_ichg_ua;
	chg->tcpm_ichg_ua = min(chg->max_ichg_ua, (u32)MP2650_DEFAULT_ICHG_UA);
	chg->last_stat = -1;

	mutex_lock(&chg->lock);
	ret = mp2650_hw_init(chg);
	mutex_unlock(&chg->lock);
	if (ret)
		return dev_err_probe(dev, ret, "failed to initialize MP2650\n");

	ret = devm_delayed_work_autocancel(dev, &chg->supply_work, mp2650_supply_work);
	if (ret)
		return ret;

	ret = devm_delayed_work_autocancel(dev, &chg->monitor_work, mp2650_monitor_work);
	if (ret)
		return ret;

	rcfg.dev = dev;
	rcfg.driver_data = chg;
	rcfg.regmap = chg->regmap;
	rcfg.init_data = &mp2650_otg_init_data;
	chg->otg_rdev = devm_regulator_register(dev, &mp2650_otg_reg_desc, &rcfg);
	if (IS_ERR(chg->otg_rdev))
		return dev_err_probe(dev, PTR_ERR(chg->otg_rdev),
				     "failed to register OTG VBUS regulator\n");

	psy_cfg.drv_data = chg;
	psy_cfg.fwnode = dev_fwnode(dev);
	psy_cfg.attr_grp = mp2650_psy_attr_groups;
	chg->psy = devm_power_supply_register(dev, &mp2650_psy_desc, &psy_cfg);
	if (IS_ERR(chg->psy))
		return dev_err_probe(dev, PTR_ERR(chg->psy), "failed to register power supply\n");

	if (IS_ENABLED(CONFIG_THERMAL)) {
		chg->cdev = devm_thermal_of_cooling_device_register(dev, dev->of_node,
								    "mp2650-chg", chg,
								    &mp2650_cooling_ops);
		if (IS_ERR(chg->cdev)) {
			dev_warn(dev, "failed to register cooling device: %ld\n",
				 PTR_ERR(chg->cdev));
			chg->cdev = NULL;
		}
	}

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

static void mp2650_shutdown(struct i2c_client *client)
{
	struct mp2650_charger *chg = i2c_get_clientdata(client);

	if (!chg)
		return;

	cancel_delayed_work_sync(&chg->supply_work);
	cancel_delayed_work_sync(&chg->monitor_work);

	mutex_lock(&chg->lock);
	if (chg->otg_en_gpio)
		gpiod_set_value_cansleep(chg->otg_en_gpio, 0);
	regmap_update_bits(chg->regmap, MP2650_REG_CTRL0,
			   MP2650_CTRL0_OTG_EN | MP2650_CTRL0_LEARN_EN, 0);
	chg->otg_enabled = false;
	mp2650_set_adc_enable_locked(chg, false);
	mutex_unlock(&chg->lock);
}

static int mp2650_suspend(struct device *dev)
{
	struct mp2650_charger *chg = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&chg->supply_work);
	cancel_delayed_work_sync(&chg->monitor_work);

	/*
	 * Gate off internal continuous ADC during s2idle suspend to eliminate
	 * quiescent charger current; hardware charging continues autonomously.
	 */
	mutex_lock(&chg->lock);
	mp2650_set_adc_enable_locked(chg, false);
	mutex_unlock(&chg->lock);

	return 0;
}

static int mp2650_resume(struct device *dev)
{
	struct mp2650_charger *chg = dev_get_drvdata(dev);
	unsigned int stat = 0;

	regmap_read(chg->regmap, MP2650_REG_STAT, &stat);
	if ((stat & MP2650_STAT_VIN_PG) || chg->online)
		schedule_delayed_work(&chg->supply_work, msecs_to_jiffies(100));

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(mp2650_pm_ops, mp2650_suspend, mp2650_resume);

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
		.pm = pm_sleep_ptr(&mp2650_pm_ops),
	},
	.probe = mp2650_probe,
	.shutdown = mp2650_shutdown,
	.id_table = mp2650_i2c_id,
};
module_i2c_driver(mp2650_driver);

MODULE_DESCRIPTION("MPS MP2650 2S Buck-Boost Battery Charger driver");
MODULE_LICENSE("GPL");
