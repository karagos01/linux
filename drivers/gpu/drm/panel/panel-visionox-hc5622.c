// SPDX-License-Identifier: GPL-2.0-only
/*
 * Visionox HC5622 DSI command-mode AMOLED panel (1080x2400, DSC 1.1)
 *
 * Copyright (c) 2026 Jan Benedikt <karagos01@gmail.com>
 *
 * Init sequence and power-up timing reverse-engineered from the vendor
 * panel-visionox-hc5622-dphy-cmd-120hz driver.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>

#include <video/mipi_display.h>

#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

/* PPS bytes up to and including the last RC range parameter */
#define HC5622_PPS_LEN	88

/* rate control parameters of the panel's DSC decoder (vendor values) */
#define RC(min, max, ofs) { min, max, (ofs) & DSC_RANGE_BPG_OFFSET_MASK }
static const struct drm_dsc_rc_range_parameters hc5622_rc_range[DSC_NUM_BUF_RANGES] = {
	RC(0, 8, 2), RC(4, 8, 0), RC(5, 9, 0), RC(5, 10, -2),
	RC(7, 11, -4), RC(7, 11, -6), RC(7, 11, -8), RC(7, 12, -8),
	RC(7, 13, -8), RC(7, 14, -10), RC(9, 14, -10), RC(9, 15, -12),
	RC(9, 15, -12), RC(13, 16, -12), RC(16, 17, -12),
};
#undef RC

static const u16 hc5622_rc_buf_thresh[DSC_NUM_BUF_RANGES - 1] = {
	896, 1792, 2688, 3584, 4480, 5376, 6272, 6720, 7168, 7616,
	7744, 7872, 8000, 8064,
};

struct visionox_hc5622 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct drm_dsc_config dsc;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *vci_gpio;	/* vldo18 (AMOLED VCI) */
	struct gpio_desc *vddi_gpio;	/* bias[0] (AMOLED VDDI) */
	struct gpio_desc *dvdd_gpio;	/* bias[1] (AMOLED DVDD) */
};

static inline struct visionox_hc5622 *to_visionox_hc5622(struct drm_panel *panel)
{
	return container_of(panel, struct visionox_hc5622, panel);
}

/*
 * The panel takes the first 88 bytes of the VESA PPS in three vendor
 * commands of 32 bytes each instead of a PPS packet.
 */
static void visionox_hc5622_send_pps(struct visionox_hc5622 *ctx,
				     struct mipi_dsi_multi_context *dsi_ctx)
{
	static const u8 cmds[] = { 0xc7, 0xc8, 0xc9 };
	struct drm_dsc_picture_parameter_set pps;
	const u8 *data = (const u8 *)&pps;
	u8 buf[1 + 32];
	int i;

	drm_dsc_pps_payload_pack(&pps, &ctx->dsc);

	for (i = 0; i < ARRAY_SIZE(cmds); i++) {
		size_t len = min_t(size_t, 32, HC5622_PPS_LEN - 32 * i);

		memset(buf, 0, sizeof(buf));
		buf[0] = cmds[i];
		memcpy(&buf[1], &data[32 * i], len);
		mipi_dsi_dcs_write_buffer_multi(dsi_ctx, buf, sizeof(buf));
	}
}

static int visionox_hc5622_on(struct visionox_hc5622 *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x9c, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfd, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x48, 0x00);	/* 60 Hz */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x53, 0xe0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_TEAR_ON, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_EXIT_SLEEP_MODE);
	mipi_dsi_msleep(&dsi_ctx, 150);

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x9f, 0x0f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xce, 0x22);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x9f, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x90);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x9f, 0x01);
	visionox_hc5622_send_pps(ctx, &dsi_ctx);

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_DISPLAY_ON);
	mipi_dsi_msleep(&dsi_ctx, 50);

	/* the brightness is (re)applied by the backlight on panel enable */
	return dsi_ctx.accum_err;
}

static void visionox_hc5622_power_on(struct visionox_hc5622 *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(5);
	gpiod_set_value_cansleep(ctx->vddi_gpio, 1);
	msleep(5);
	gpiod_set_value_cansleep(ctx->dvdd_gpio, 1);
	msleep(10);
	gpiod_set_value_cansleep(ctx->vci_gpio, 1);
	msleep(15);

	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(10);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	msleep(50);
}

static void visionox_hc5622_power_off(struct visionox_hc5622 *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(10);
	gpiod_set_value_cansleep(ctx->vci_gpio, 0);
	msleep(5);
	gpiod_set_value_cansleep(ctx->dvdd_gpio, 0);
	msleep(10);
	gpiod_set_value_cansleep(ctx->vddi_gpio, 0);
	msleep(5);
}

static int visionox_hc5622_prepare(struct drm_panel *panel)
{
	struct visionox_hc5622 *ctx = to_visionox_hc5622(panel);
	int ret;

	visionox_hc5622_power_on(ctx);

	ret = visionox_hc5622_on(ctx);
	if (ret < 0) {
		visionox_hc5622_power_off(ctx);
		return ret;
	}

	return 0;
}

static int visionox_hc5622_unprepare(struct drm_panel *panel)
{
	struct visionox_hc5622 *ctx = to_visionox_hc5622(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_DISPLAY_OFF);
	mipi_dsi_msleep(&dsi_ctx, 120);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_ENTER_SLEEP_MODE);
	mipi_dsi_msleep(&dsi_ctx, 50);

	visionox_hc5622_power_off(ctx);

	return 0;
}

#define HFP 140
#define HSA 4
#define HBP 24
#define HACT 1080
#define VFP 2448
#define VSA 4
#define VBP 12
#define VACT 2400

static const struct drm_display_mode visionox_hc5622_mode = {
	.clock = (HACT + HFP + HSA + HBP) * (VACT + VFP + VSA + VBP) * 60 / 1000,
	.hdisplay = HACT,
	.hsync_start = HACT + HFP,
	.hsync_end = HACT + HFP + HSA,
	.htotal = HACT + HFP + HSA + HBP,
	.vdisplay = VACT,
	.vsync_start = VACT + VFP,
	.vsync_end = VACT + VFP + VSA,
	.vtotal = VACT + VFP + VSA + VBP,
	.width_mm = 70,
	.height_mm = 155,
	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static int visionox_hc5622_get_modes(struct drm_panel *panel,
				     struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector,
						    &visionox_hc5622_mode);
}

/*
 * 12-bit brightness through DCS 0x51. Levels above 3800 drive the panel in
 * high-brightness mode; the vendor default is 3515 (0xdbb).
 */
#define HC5622_MAX_BRIGHTNESS		4095
#define HC5622_DEFAULT_BRIGHTNESS	3515

static int visionox_hc5622_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct visionox_hc5622 *ctx = mipi_dsi_get_drvdata(dsi);
	u16 brightness = backlight_get_brightness(bl);

	/* nothing to talk to while the panel is powered off */
	if (!ctx->panel.prepared)
		return 0;

	return mipi_dsi_dcs_set_display_brightness_large(dsi, brightness);
}

static const struct backlight_ops visionox_hc5622_bl_ops = {
	.update_status = visionox_hc5622_bl_update_status,
};

static struct backlight_device *
visionox_hc5622_create_backlight(struct mipi_dsi_device *dsi)
{
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = HC5622_DEFAULT_BRIGHTNESS,
		.max_brightness = HC5622_MAX_BRIGHTNESS,
	};

	return devm_backlight_device_register(&dsi->dev, dev_name(&dsi->dev),
					      &dsi->dev, dsi,
					      &visionox_hc5622_bl_ops, &props);
}

static const struct drm_panel_funcs visionox_hc5622_panel_funcs = {
	.prepare = visionox_hc5622_prepare,
	.unprepare = visionox_hc5622_unprepare,
	.get_modes = visionox_hc5622_get_modes,
};

static int visionox_hc5622_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct visionox_hc5622 *ctx;
	int ret, i;

	ctx = devm_drm_panel_alloc(dev, struct visionox_hc5622, panel,
				   &visionox_hc5622_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->vddi_gpio = devm_gpiod_get_index(dev, "bias", 0, GPIOD_OUT_LOW);
	if (IS_ERR(ctx->vddi_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->vddi_gpio),
				     "Failed to get bias-gpios[0] (VDDI)\n");

	ctx->dvdd_gpio = devm_gpiod_get_index(dev, "bias", 1, GPIOD_OUT_LOW);
	if (IS_ERR(ctx->dvdd_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->dvdd_gpio),
				     "Failed to get bias-gpios[1] (DVDD)\n");

	ctx->vci_gpio = devm_gpiod_get(dev, "vldo18", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->vci_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->vci_gpio),
				     "Failed to get vldo18-gpios (VCI)\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS;

	ctx->panel.prepare_prev_first = true;

	ctx->panel.backlight = visionox_hc5622_create_backlight(dsi);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "Failed to create backlight\n");

	drm_panel_add(&ctx->panel);

	/* DSC 1.1, 10 bpc, 8 bpp, two 540x20 slices per line */
	dsi->dsc = &ctx->dsc;
	ctx->dsc.dsc_version_major = 1;
	ctx->dsc.dsc_version_minor = 1;
	ctx->dsc.slice_height = 20;
	ctx->dsc.slice_width = 540;
	ctx->dsc.slice_count = HACT / ctx->dsc.slice_width;
	ctx->dsc.bits_per_component = 10;
	ctx->dsc.bits_per_pixel = 8 << 4;
	ctx->dsc.block_pred_enable = true;
	ctx->dsc.convert_rgb = true;
	ctx->dsc.line_buf_depth = 11;
	ctx->dsc.pic_width = HACT;
	ctx->dsc.pic_height = VACT;
	ctx->dsc.slice_chunk_size = ctx->dsc.slice_width *
				    ctx->dsc.bits_per_pixel / 16 / 8;
	drm_dsc_set_const_params(&ctx->dsc);
	ctx->dsc.initial_xmit_delay = 512;
	ctx->dsc.initial_dec_delay = 549;
	ctx->dsc.initial_scale_value = 32;
	ctx->dsc.scale_increment_interval = 469;
	ctx->dsc.scale_decrement_interval = 7;
	ctx->dsc.first_line_bpg_offset = 13;
	ctx->dsc.nfl_bpg_offset = 1402;
	ctx->dsc.slice_bpg_offset = 1302;
	ctx->dsc.initial_offset = 6144;
	ctx->dsc.final_offset = 4336;
	ctx->dsc.flatness_min_qp = 7;
	ctx->dsc.flatness_max_qp = 16;
	ctx->dsc.rc_quant_incr_limit0 = 15;
	ctx->dsc.rc_quant_incr_limit1 = 15;
	for (i = 0; i < DSC_NUM_BUF_RANGES - 1; i++)
		ctx->dsc.rc_buf_thresh[i] = hc5622_rc_buf_thresh[i] >> 6;
	memcpy(ctx->dsc.rc_range_params, hc5622_rc_range,
	       sizeof(hc5622_rc_range));

	ret = devm_mipi_dsi_attach(dev, dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "Failed to attach to DSI host\n");
	}

	return 0;
}

static void visionox_hc5622_remove(struct mipi_dsi_device *dsi)
{
	struct visionox_hc5622 *ctx = mipi_dsi_get_drvdata(dsi);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id visionox_hc5622_of_match[] = {
	{ .compatible = "visionox,hc5622" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, visionox_hc5622_of_match);

static struct mipi_dsi_driver visionox_hc5622_driver = {
	.probe = visionox_hc5622_probe,
	.remove = visionox_hc5622_remove,
	.driver = {
		.name = "panel-visionox-hc5622",
		.of_match_table = visionox_hc5622_of_match,
	},
};
module_mipi_dsi_driver(visionox_hc5622_driver);

MODULE_AUTHOR("Jan Benedikt <karagos01@gmail.com>");
MODULE_DESCRIPTION("Visionox HC5622 DSI command-mode panel driver");
MODULE_LICENSE("GPL");
