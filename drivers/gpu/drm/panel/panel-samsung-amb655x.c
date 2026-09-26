// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026, The Linux Foundation.
// Generated with linux-mdss-dsi-panel-driver-generator from vendor device tree:
//   Copyright (C) 2018-2020 Oplus. All rights reserved.

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_crtc.h>

struct amb655x {
	struct drm_panel panel;
	struct drm_connector *connector;
	struct mipi_dsi_device *dsi;
	struct drm_dsc_config dsc;
	struct regulator_bulk_data *supplies;
	struct gpio_desc *reset_gpio;
};

static const struct regulator_bulk_data amb655x_supplies[] = {
	{ .supply = "vdd" },
	{ .supply = "vddio" },
};

static inline struct amb655x *to_amb655x(struct drm_panel *panel)
{
	return container_of_const(panel, struct amb655x, panel);
}

static int amb655x_get_current_mode(struct amb655x *ctx)
{
	struct drm_connector *connector = ctx->connector;
	struct drm_crtc_state *crtc_state;
	struct drm_display_mode *mode;

	if (!connector || !connector->state)
		return -ENODEV;

	crtc_state = connector->state->crtc->state;
	if (!crtc_state)
		return -ENODEV;

	mode = &crtc_state->mode;
	if (!mode)
		return -ENODEV;

	/* Return the current mode's vrefresh value */
	return drm_mode_vrefresh(mode);
}

static void amb655x_reset(struct amb655x *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
}

static int amb655x_on(struct amb655x *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	/* DSC Setting */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x9e,
				     0x11, 0x00, 0x00, 0x89, 0x30, 0x80, 0x09,
				     0x60, 0x04, 0x38, 0x00, 0x1e, 0x02, 0x1c,
				     0x02, 0x1c, 0x02, 0x00, 0x02, 0x0e, 0x00,
				     0x20, 0x02, 0xe3, 0x00, 0x07, 0x00, 0x0c,
				     0x03, 0x50, 0x03, 0x64, 0x18, 0x00, 0x10,
				     0xf0, 0x03, 0x0c, 0x20, 0x00, 0x06, 0x0b,
				     0x0b, 0x33, 0x0e, 0x1c, 0x2a, 0x38, 0x46,
				     0x54, 0x62, 0x69, 0x70, 0x77, 0x79, 0x7b,
				     0x7d, 0x7e, 0x01, 0x02, 0x01, 0x00, 0x09,
				     0x40, 0x09, 0xbe, 0x19, 0xfc, 0x19, 0xfa,
				     0x19, 0xf8, 0x1a, 0x38, 0x1a, 0x78, 0x1a,
				     0xb6, 0x2a, 0xf6, 0x2b, 0x34, 0x2b, 0x74,
				     0x3b, 0x74, 0x6b, 0xf4, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x9d, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);

	/* Sleep Out(11h) */
	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_usleep_range(&dsi_ctx, 11000, 12000);

	/* VLIN CURRENT LIMIT */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd5,
				     0x24, 0x9e, 0x9e, 0x00, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);

	/* OSC Select */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfc, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x16);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd1, 0x22);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd6, 0x11);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfc, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);

	/* TE ON */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_set_tear_on_multi(&dsi_ctx, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);

	/* TSP Setting */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xdf, 0x83, 0x00, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe6, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);

	/* CASET/PASET Setting */
	mipi_dsi_dcs_set_column_address_multi(&dsi_ctx, 0, 1080 - 1);
	mipi_dsi_dcs_set_page_address_multi(&dsi_ctx, 0, 2400 - 1);

	/* FD Setting */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd5, 0x8d);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x0a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd5, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);

	/* FFC Function */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfc, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe4, 0xa6, 0x75, 0xa3);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe9,
				     0x11, 0x75, 0xa6, 0x75, 0xa3, 0x4b, 0x17,
				     0xac, 0x4b, 0x17, 0xac, 0x00, 0x19, 0x19);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfc, 0xa5, 0xa5);

	/* FFC Function */
	mipi_dsi_msleep(&dsi_ctx, 61);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x06);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb7, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb7, 0x13);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb7, 0x4c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY,
				     0x20);


	/* 60hz Transition */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	int cur_vrefresh = amb655x_get_current_mode(ctx);
	if (cur_vrefresh == 120) {
		/* 120 Hz */
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x60, 0x10);
	} else {
		/* 60 Hz */
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x60, 0x00);
	}
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);

	/* ACL Mode */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_POWER_SAVE, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY,
				     0x20);

	return dsi_ctx.accum_err;
}

static int amb655x_enable(struct drm_panel *panel)
{
	struct amb655x *ctx = to_amb655x(panel);
	struct device *dev = &ctx->dsi->dev;
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	mipi_dsi_msleep(&dsi_ctx, 110);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x9f, 0x5a, 0x5a);
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_ENTER_NORMAL_MODE);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x9f, 0xa5, 0xa5);

	return dsi_ctx.accum_err;
}

static int amb655x_off(struct amb655x *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x9f, 0x5a, 0x5a);
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 20);
	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 20);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x9f, 0xa5, 0xa5);
	mipi_dsi_msleep(&dsi_ctx, 150);

	return dsi_ctx.accum_err;
}

static int amb655x_prepare(struct drm_panel *panel)
{
	struct amb655x *ctx = to_amb655x(panel);
	struct device *dev = &ctx->dsi->dev;
	struct drm_dsc_picture_parameter_set pps;
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(amb655x_supplies), ctx->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators: %d\n", ret);
		return ret;
	}

	amb655x_reset(ctx);

	ret = amb655x_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		regulator_bulk_disable(ARRAY_SIZE(amb655x_supplies), ctx->supplies);
		return ret;
	}

	drm_dsc_pps_payload_pack(&pps, &ctx->dsc);

	ret = mipi_dsi_picture_parameter_set(ctx->dsi, &pps);
	if (ret < 0) {
		dev_err(panel->dev, "failed to transmit PPS: %d\n", ret);
		return ret;
	}

	return 0;
}

static int amb655x_unprepare(struct drm_panel *panel)
{
	struct amb655x *ctx = to_amb655x(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = amb655x_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(amb655x_supplies), ctx->supplies);

	return 0;
}

static const struct drm_display_mode amb655x_120_mode = {
	.clock = (1080 + 52 + 24 + 24) * (2400 + 1208 + 4 + 8) * 120 / 1000,
	.hdisplay = 1080,
	.hsync_start = 1080 + 52,
	.hsync_end = 1080 + 52 + 24,
	.htotal = 1080 + 52 + 24 + 24,
	.vdisplay = 2400,
	.vsync_start = 2400 + 1208,
	.vsync_end = 2400 + 1208 + 4,
	.vtotal = 2400 + 1208 + 4 + 8,
	.width_mm = 70,
	.height_mm = 151,
	.type = DRM_MODE_TYPE_DRIVER,
};

static const struct drm_display_mode amb655x_60_mode = {
	.clock = (1080 + 52 + 24 + 24) * (2400 + 1208 + 4 + 8) * 60 / 1000,
	.hdisplay = 1080,
	.hsync_start = 1080 + 52,
	.hsync_end = 1080 + 52 + 24,
	.htotal = 1080 + 52 + 24 + 24,
	.vdisplay = 2400,
	.vsync_start = 2400 + 1208,
	.vsync_end = 2400 + 1208 + 4,
	.vtotal = 2400 + 1208 + 4 + 8,
	.width_mm = 70,
	.height_mm = 151,
	.type = DRM_MODE_TYPE_DRIVER,
};

static int amb655x_get_modes(struct drm_panel *panel,
			     struct drm_connector *connector)
{
	int count = 0;

	/* Initialize 120Hz first */
	count += drm_connector_helper_get_modes_fixed(connector, &amb655x_120_mode);
	count += drm_connector_helper_get_modes_fixed(connector, &amb655x_60_mode);

	return count;
}

static const struct drm_panel_funcs amb655x_panel_funcs = {
	.prepare = amb655x_prepare,
	.enable = amb655x_enable,
	.unprepare = amb655x_unprepare,
	.get_modes = amb655x_get_modes,
};

static int amb655x_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness = backlight_get_brightness(bl);
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_brightness_large(dsi, brightness);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return 0;
}

static int amb655x_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness;
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_get_display_brightness_large(dsi, &brightness);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return brightness;
}

static const struct backlight_ops amb655x_bl_ops = {
	.update_status = amb655x_bl_update_status,
	.get_brightness = amb655x_bl_get_brightness,
};

static struct backlight_device *
amb655x_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = 360,
		.max_brightness = 1023,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &amb655x_bl_ops, &props);
}

static int amb655x_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct amb655x *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct amb655x, panel,
				   &amb655x_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ret = devm_regulator_bulk_get_const(dev,
					    ARRAY_SIZE(amb655x_supplies),
					    amb655x_supplies,
					    &ctx->supplies);
	if (ret < 0)
		return ret;

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO_BURST |
			  MIPI_DSI_MODE_NO_EOT_PACKET |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS | MIPI_DSI_MODE_LPM;

	ctx->panel.prepare_prev_first = true;

	ctx->panel.backlight = amb655x_create_backlight(dsi);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "Failed to create backlight\n");

	drm_panel_add(&ctx->panel);

	/* This panel only supports DSC; unconditionally enable it */
	dsi->dsc = &ctx->dsc;

	ctx->dsc.dsc_version_major = 1;
	ctx->dsc.dsc_version_minor = 1;

	/* TODO: Pass slice_per_pkt = 2 */
	ctx->dsc.slice_height = 30;
	ctx->dsc.slice_width = 540;
	/*
	 * TODO: hdisplay should be read from the selected mode once
	 * it is passed back to drm_panel (in prepare?)
	 */
	WARN_ON(1080 % ctx->dsc.slice_width);
	ctx->dsc.slice_count = 1080 / ctx->dsc.slice_width;
	ctx->dsc.bits_per_component = 8;
	ctx->dsc.bits_per_pixel = 8 << 4; /* 4 fractional bits */
	ctx->dsc.block_pred_enable = true;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "Failed to attach to DSI host\n");
	}

	return 0;
}

static void amb655x_remove(struct mipi_dsi_device *dsi)
{
	struct amb655x *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id amb655x_of_match[] = {
	{ .compatible = "samsung,amb655x" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, amb655x_of_match);

static struct mipi_dsi_driver amb655x_driver = {
	.probe = amb655x_probe,
	.remove = amb655x_remove,
	.driver = {
		.name = "panel-amb655x",
		.of_match_table = amb655x_of_match,
	},
};
module_mipi_dsi_driver(amb655x_driver);

MODULE_AUTHOR("Robert Oscilowski <drgitx@gmail.com>");
MODULE_DESCRIPTION("DRM driver for Samsung AMB655x FHD cmd mode DSC DSI panel");
MODULE_LICENSE("GPL");
