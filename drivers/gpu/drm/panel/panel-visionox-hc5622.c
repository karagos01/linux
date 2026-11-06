// SPDX-License-Identifier: GPL-2.0-only
/*
 * Visionox HC5622 DSI command-mode AMOLED panel (1080x2400, DSC 1.1)
 *
 * Copyright (c) 2026 Jan Benedikt <karagos01@gmail.com>
 *
 * Init sequence and power-up timing reverse-engineered from the vendor
 * panel-visionox-hc5622-dphy-cmd-120hz driver.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>

#include <video/mipi_display.h>

#include <drm/display/drm_dsc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

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
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc7,
		0x11, 0x00, 0x00, 0xab, 0x30, 0x80, 0x09, 0x60, 0x04, 0x38,
		0x00, 0x14, 0x02, 0x1c, 0x02, 0x1c, 0x02, 0x00, 0x02, 0x25,
		0x00, 0x20, 0x01, 0xd5, 0x00, 0x07, 0x00, 0x0d, 0x05, 0x7a,
		0x05, 0x16);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc8,
		0x18, 0x00, 0x10, 0xf0, 0x07, 0x10, 0x20, 0x00, 0x06, 0x0f,
		0x0f, 0x33, 0x0e, 0x1c, 0x2a, 0x38, 0x46, 0x54, 0x62, 0x69,
		0x70, 0x77, 0x79, 0x7b, 0x7d, 0x7e, 0x02, 0x02, 0x22, 0x00,
		0x2a, 0x40);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc9,
		0x2a, 0xbe, 0x3a, 0xfc, 0x3a, 0xfa, 0x3a, 0xf8, 0x3b, 0x38,
		0x3b, 0x78, 0x3b, 0xb6, 0x4b, 0xb6, 0x4b, 0xf4, 0x4b, 0xf4,
		0x6c, 0x34, 0x84, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_DISPLAY_ON);
	mipi_dsi_msleep(&dsi_ctx, 50);

	/* Force maximum brightness (12-bit via 0x51) so the panel is visible */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_DISPLAY_BRIGHTNESS,
				     0x0f, 0xff);

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

static const struct drm_panel_funcs visionox_hc5622_panel_funcs = {
	.prepare = visionox_hc5622_prepare,
	.unprepare = visionox_hc5622_unprepare,
	.get_modes = visionox_hc5622_get_modes,
};

static int visionox_hc5622_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct visionox_hc5622 *ctx;
	int ret;

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

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get backlight\n");

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
