// SPDX-License-Identifier: GPL-2.0-only
/*
 * OPlus/OnePlus SLPI sensor hardware info publisher (SMEM item 130).
 *
 * OnePlus SM8250 SLPI firmware reads struct sensor_info from Qualcomm SMEM
 * item 130 (SMEM_SENSOR) to discover which physical and virtual sensors
 * (e.g. BMI260, AKM0991X, STK33502, TCS3408, BMP380) are populated on the
 * board before probing them on the SSC buses.
 *
 * Derived from OPlus oplus_sensor_devinfo.c for OnePlus 8T (kebab).
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/soc/qcom/smem.h>
#include <linux/string.h>

#define SMEM_SENSOR		130

#define REG_NUM			10
#define PARAMETER_NUM		25
#define FEATURE_NUM		10
#define SOURCE_NUM		3
#define ALGO_PARAMETER_NUM	15
#define ALGO_FEATURE_NUM	5
#define DEFAULT_CONFIG		0xff
#define SAR_MAX_CH_NUM		5

enum oplus_sensor_id {
	OPLUS_ACCEL,
	OPLUS_GYRO,
	OPLUS_MAG,
	OPLUS_LIGHT,
	OPLUS_PROXIMITY,
	OPLUS_SAR,
	OPLUS_CCT,
	OPLUS_CCT_REAR,
	OPLUS_BAROMETER,
	OPLUS_SARS,
	OPLUS_LIGHT_REAR,
	SENSORS_NUM
};

enum oplus_sensor_algo_id {
	OPLUS_PICKUP_DETECT,
	OPLUS_LUX_AOD,
	OPLUS_TP_GESTURE,
	OPLUS_FP_DISPLAY,
	OPLUS_FREE_FALL,
	OPLUS_CAMERA_PROTECT,
	OPLUS_MAG_FUSION,
	OPLUS_MEASUREMENT,
	SENSOR_ALGO_NUM
};

struct oplus_sensor_feature {
	int reg[REG_NUM];
	int parameter[PARAMETER_NUM];
	int feature[FEATURE_NUM];
};

struct oplus_sensor_hw {
	u8 sensor_name;
	u8 bus_number;
	u8 direction;
	u8 irq_number;
	struct oplus_sensor_feature feature;
};

struct oplus_sensor_vector {
	int sensor_id;
	struct oplus_sensor_hw hw[SOURCE_NUM];
};

struct oplus_sensor_vector_old {
	int sensor_id;
	struct oplus_sensor_hw hw[SOURCE_NUM - 1];
};

struct oplus_sensor_algorithm {
	int sensor_id;
	int parameter[ALGO_PARAMETER_NUM];
	int feature[ALGO_FEATURE_NUM];
};

struct oplus_sensor_info {
	int magic_num;
	struct oplus_sensor_vector s_vector[SENSORS_NUM];
	struct oplus_sensor_algorithm a_vector[SENSOR_ALGO_NUM];
};

struct oplus_sensor_info_old {
	int magic_num;
	struct oplus_sensor_vector_old s_vector[SENSORS_NUM];
	struct oplus_sensor_algorithm a_vector[SENSOR_ALGO_NUM];
};

static void parse_physical_common(struct oplus_sensor_hw *hw,
				  struct device_node *np)
{
	u32 val;

	hw->sensor_name = of_property_read_u32(np, "sensor-name", &val) ? 0 : val;
	hw->bus_number = of_property_read_u32(np, "bus-number", &val) ? DEFAULT_CONFIG : val;
	hw->direction = of_property_read_u32(np, "sensor-direction", &val) ? DEFAULT_CONFIG : val;
	hw->irq_number = of_property_read_u32(np, "irq-number", &val) ? DEFAULT_CONFIG : val;
}

static void parse_magnetic(struct oplus_sensor_hw *hw, struct device_node *np)
{
	static const int default_mag[18] = {
		10000, 0, 0, 0, 0, 0, 0, 0, 10000,
		0, 0, 0, 0, 0, 0, 0, 10000, 0
	};
	u32 count;

	memcpy(hw->feature.parameter, default_mag, sizeof(default_mag));
	if (!of_property_read_u32(np, "parameter-number", &count) &&
	    count > 0 && count < PARAMETER_NUM) {
		of_property_read_u32_array(np, "soft-mag-parameter",
					   (u32 *)hw->feature.parameter, count);
	}
}

static void parse_proximity(struct oplus_sensor_hw *hw, struct device_node *np)
{
	static const char *const params[] = {
		"low_step", "high_step", "low_limit", "high_limit",
		"dirty_low_step", "dirty_high_step", "ps_dirty_limit",
		"ps_ir_limit", "ps_adjust_min", "ps_adjust_max",
		"sampling_count", "step_max", "step_min", "step_div",
		"anti_shake_delta", "dynamic_cali_max", "raw2offset_radio",
		"offset_max", "offset_range_min", "offset_range_max",
		"force_cali_limit", "cali_jitter_limit", "cal_offset_margin",
		"esd_gpio_use",
	};
	u32 val;
	int i;

	if (!of_property_read_u32(np, "ps-type", &val))
		hw->feature.feature[0] = val;
	if (!of_property_read_u32(np, "ps_saturation", &val))
		hw->feature.feature[1] = val;
	if (!of_property_read_u32(np, "is_need_close_pd", &val) && val == 1)
		hw->feature.feature[2] = 1;
	if (!of_property_read_u32(np, "ps_factory_cali_max", &val))
		hw->feature.feature[3] = val;

	for (i = 0; i < ARRAY_SIZE(params); i++) {
		if (!of_property_read_u32(np, params[i], &val))
			hw->feature.parameter[i] = val;
	}

	if (!of_property_read_u32(np, "parameter-number", &val) &&
	    val > 0 && val < REG_NUM - 1) {
		hw->feature.reg[0] = val;
		of_property_read_u32_array(np, "sensor-reg",
					   (u32 *)&hw->feature.reg[1], val);
	}
}

static void parse_light(struct oplus_sensor_hw *hw, struct device_node *np)
{
	static const char *const features[] = {
		"als-type", "is-unit-device", "is-als-dri", "als-factor",
		"is_als_initialed", "als_buffer_length", "normalization_value",
		"use_lb_algo", "para-matrix", "als_ratio_type",
	};
	static const char *const params[] = {
		"coef_a", "coef_b", "coef_c", "coef_d", "coef_e", "coef_ratio",
		"gold-reset-scale", "lcd_type", "linear_comp_scope",
		"linear_comp_intercept", "temp_lux", "coef", "bal_coef",
		"ir_coef_stage_0", "ir_coef_stage_1", "ir_coef_stage_2",
		"ir_coef_stage_3", "ir_coef_val_0", "ir_coef_val_1",
		"ir_coef_val_2", "ir_coef_val_3", "ir_coef_val_4",
		"lb-dbv-coefs-id", "light-cut",
	};
	u32 val;
	int i;

	for (i = 0; i < ARRAY_SIZE(features); i++) {
		if (!of_property_read_u32(np, features[i], &val))
			hw->feature.feature[i] = val;
		else if (!strncmp(features[i], "norm", 4))
			hw->feature.feature[i] = 1057;
		else if (!strcmp(features[i], "als_ratio_type"))
			hw->feature.feature[i] = 0;
	}

	for (i = 0; i < ARRAY_SIZE(params); i++) {
		if (!of_property_read_u32(np, params[i], &val))
			hw->feature.parameter[i] = val;
		else if (!strcmp(params[i], "gold-reset-scale"))
			hw->feature.parameter[i] = 1001;
		else
			hw->feature.parameter[i] = 0;
	}
}

static void parse_sar(struct oplus_sensor_hw *hw, struct device_node *np)
{
	u32 val;

	if (!of_property_read_u32(np, "parameter-number", &val) &&
	    val > 0 && val < PARAMETER_NUM) {
		of_property_read_u32_array(np, "sensor-reg",
					   (u32 *)hw->feature.parameter, val);
	}
	if (!of_property_read_u32(np, "channel-num", &val) && val < SAR_MAX_CH_NUM)
		hw->feature.feature[0] = val;
}

static void parse_cct_rear(struct oplus_sensor_hw *hw, struct device_node *np)
{
	static const char *const features[] = {
		"decoupled-driver", "publish-sensors", "is-ch-dri",
		"timer-size", "fac-cali-sensor",
	};
	static const char *const params[] = {
		"para-matrix", "atime", "first-atime", "fac-cali-atime",
		"first-again", "fac-cali-again", "fd-time", "fac-cali-fd-time",
		"first-fd-gain", "fac-cali-fd-gain",
	};
	u32 val;
	int i;

	hw->feature.feature[0] = 1;
	for (i = 0; i < ARRAY_SIZE(features); i++) {
		if (!of_property_read_u32(np, features[i], &val))
			hw->feature.feature[i] = val;
	}
	for (i = 0; i < ARRAY_SIZE(params); i++) {
		if (!of_property_read_u32(np, params[i], &val))
			hw->feature.parameter[i] = val;
	}
}

static void parse_accel(struct oplus_sensor_hw *hw, struct device_node *np)
{
	static const char *const features[] = { "use-sois", "single-acc" };
	u32 val;
	int i;

	hw->feature.feature[0] = 0;
	for (i = 0; i < ARRAY_SIZE(features); i++) {
		if (!of_property_read_u32(np, features[i], &val))
			hw->feature.feature[i] = val;
	}
}

static void parse_physical_specific(struct oplus_sensor_hw *hw,
				    struct device_node *np)
{
	if (!strncmp(np->name, "msensor", 7))
		parse_magnetic(hw, np);
	else if (!strncmp(np->name, "psensor", 7))
		parse_proximity(hw, np);
	else if (!strncmp(np->name, "lsensor", 7))
		parse_light(hw, np);
	else if (!strncmp(np->name, "ssensor", 7) || !strncmp(np->name, "sdsensor", 8))
		parse_sar(hw, np);
	else if (!strncmp(np->name, "cctrsensor", 10) || !strncmp(np->name, "cctsensor", 9))
		parse_cct_rear(hw, np);
	else if (!strncmp(np->name, "gsensor", 7))
		parse_accel(hw, np);
}

static void parse_virtual(struct oplus_sensor_algorithm *algo,
			  struct device_node *np)
{
	u32 val;

	if (!strncmp(np->name, "pickup", 6)) {
		if (!of_property_read_u32(np, "is-need-prox", &val))
			algo->feature[0] = val;
		if (!of_property_read_u32(np, "prox-type", &val))
			algo->parameter[0] = val;
	} else if (!strncmp(np->name, "lux_aod", 7)) {
		if (!of_property_read_u32(np, "thrd-low", &val))
			algo->parameter[0] = val;
		if (!of_property_read_u32(np, "thrd-high", &val))
			algo->parameter[1] = val;
		if (!of_property_read_u32(np, "als-type", &val))
			algo->parameter[2] = val;
		if (!of_property_read_u32(np, "fold-feature", &val))
			algo->feature[0] = val;
	} else if (!strncmp(np->name, "fp_display", 10)) {
		if (!of_property_read_u32(np, "prox-type", &val))
			algo->parameter[0] = val;
	} else if (!strncmp(np->name, "mag_fusion", 10)) {
		if (!of_property_read_u32(np, "fusion-type", &val))
			algo->feature[0] = val;
		if (!of_property_read_u32(np, "mag-type", &val))
			algo->feature[1] = val;
	}
}

static int oplus_sensor_devinfo_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct oplus_sensor_info *chip = NULL;
	struct oplus_sensor_info_old *chip_old = NULL;
	struct device_node *child;
	size_t smem_size = 0;
	void *smem_addr;
	int count = 0;
	int rc;

	rc = qcom_smem_alloc(QCOM_SMEM_HOST_ANY, SMEM_SENSOR,
			     ALIGN(sizeof(struct oplus_sensor_info), 4));
	if (rc == -EPROBE_DEFER)
		return rc;
	if (rc < 0 && rc != -EEXIST) {
		dev_err(dev, "qcom_smem_alloc(SMEM_SENSOR) failed: %d\n", rc);
		return rc;
	}

	smem_addr = qcom_smem_get(QCOM_SMEM_HOST_ANY, SMEM_SENSOR, &smem_size);
	if (IS_ERR(smem_addr)) {
		rc = PTR_ERR(smem_addr);
		if (rc != -EPROBE_DEFER)
			dev_err(dev, "qcom_smem_get(SMEM_SENSOR) failed: %d\n", rc);
		return rc;
	}

	if (smem_size >= sizeof(struct oplus_sensor_info)) {
		chip = smem_addr;
		memset(chip, 0, sizeof(*chip));
	} else if (smem_size >= sizeof(struct oplus_sensor_info_old)) {
		chip_old = smem_addr;
		memset(chip_old, 0, sizeof(*chip_old));
	} else {
		dev_err(dev, "unsupported SMEM_SENSOR size: %zu (need %zu or %zu)\n",
			smem_size, sizeof(struct oplus_sensor_info),
			sizeof(struct oplus_sensor_info_old));
		return -EINVAL;
	}

	for_each_child_of_node(dev->of_node, child) {
		bool is_virtual = of_property_read_bool(child, "is-virtual-sensor");
		u32 sensor_type, sensor_index;

		if (of_property_read_u32(child, "sensor-type", &sensor_type))
			continue;

		if (!is_virtual) {
			struct oplus_sensor_hw *hw;

			if (sensor_type >= SENSORS_NUM)
				continue;
			if (of_property_read_u32(child, "sensor-index", &sensor_index))
				continue;
			if (chip) {
				if (sensor_index >= SOURCE_NUM)
					continue;
				chip->s_vector[sensor_type].sensor_id = sensor_type;
				hw = &chip->s_vector[sensor_type].hw[sensor_index];
			} else {
				if (sensor_index >= SOURCE_NUM - 1)
					continue;
				chip_old->s_vector[sensor_type].sensor_id = sensor_type;
				hw = &chip_old->s_vector[sensor_type].hw[sensor_index];
			}
			parse_physical_common(hw, child);
			parse_physical_specific(hw, child);
			count++;
		} else {
			struct oplus_sensor_algorithm *algo;

			if (sensor_type >= SENSOR_ALGO_NUM)
				continue;
			if (chip) {
				chip->a_vector[sensor_type].sensor_id = sensor_type;
				algo = &chip->a_vector[sensor_type];
			} else {
				chip_old->a_vector[sensor_type].sensor_id = sensor_type;
				algo = &chip_old->a_vector[sensor_type];
			}
			parse_virtual(algo, child);
			count++;
		}
	}

	dev_info(dev, "populated SMEM_SENSOR (item %d, size %zu, %s layout) with %d entries\n",
		 SMEM_SENSOR, smem_size, chip ? "v2" : "v1", count);

	return 0;
}

static const struct of_device_id oplus_sensor_devinfo_of_match[] = {
	{ .compatible = "oplus,sensor-devinfo" },
	{ }
};
MODULE_DEVICE_TABLE(of, oplus_sensor_devinfo_of_match);

static struct platform_driver oplus_sensor_devinfo_driver = {
	.probe = oplus_sensor_devinfo_probe,
	.driver = {
		.name = "oplus-sensor-devinfo",
		.of_match_table = oplus_sensor_devinfo_of_match,
	},
};

module_platform_driver(oplus_sensor_devinfo_driver);

MODULE_DESCRIPTION("OPlus SLPI sensor hardware info SMEM publisher");
MODULE_LICENSE("GPL");
