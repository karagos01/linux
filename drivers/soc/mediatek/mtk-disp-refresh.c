// SPDX-License-Identifier: GPL-2.0-only
/*
 * Keep a bootloader-configured MediaTek command mode DSI panel refreshed.
 *
 * Command mode panels only update when the display pipeline pushes a frame.
 * The bootloader leaves OVL -> ... -> DSI set up to scan out the boot
 * framebuffer, so retriggering the display mutex and DSI periodically is
 * enough to make a simple-framebuffer usable until a real display driver
 * exists.
 *
 * Panel brightness (DCS 0x51) is exposed as a backlight device, the command
 * is sent in place of a frame and the frame command restored afterwards.
 */

#include <linux/backlight.h>
#include <linux/bits.h>
#include <linux/hrtimer.h>
#include <linux/io.h>
#include <linux/ktime.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <video/mipi_display.h>

#define DISP_REG_MUTEX_EN(n)	(0x20 + 0x20 * (n))

#define DSI_START		0x00
#define DSI_INTSTA		0x0c
#define DSI_BUSY		BIT(31)
#define DSI_CMDQ_SIZE		0x60
#define DSI_CMDQ0		0xd00
#define DSI_CMDQ1		0xd04

/* CMDQ entry for a 3 byte DCS long write sent in low power mode */
#define DSI_CMDQ_DCS_LONG3	((3 << 16) | (MIPI_DSI_DCS_LONG_WRITE << 8) | 0x2)
#define DSI_CMDQ_SIZE_SEL	BIT(15)

#define BRIGHTNESS_MAX		4095

static unsigned int refresh_hz = 60;
module_param(refresh_hz, uint, 0644);
MODULE_PARM_DESC(refresh_hz, "Panel refresh rate in Hz (0 stops refreshing)");

struct mtk_disp_refresh {
	void __iomem *mutex;
	void __iomem *dsi;
	struct hrtimer timer;
	u32 mutex_id;
	/* Frame command left by the bootloader, restored after a DCS write */
	u32 frame_cmdq0;
	u32 frame_cmdq_size;
	/* Brightness waiting to be sent, or -1 */
	int pending_brightness;
	bool cmd_in_flight;
};

static void mtk_disp_refresh_send_brightness(struct mtk_disp_refresh *r, int level)
{
	writel(DSI_CMDQ_DCS_LONG3, r->dsi + DSI_CMDQ0);
	writel(MIPI_DCS_SET_DISPLAY_BRIGHTNESS | (level >> 8) << 8 | (level & 0xff) << 16,
	       r->dsi + DSI_CMDQ1);
	writel(DSI_CMDQ_SIZE_SEL | 2, r->dsi + DSI_CMDQ_SIZE);
	writel(0, r->dsi + DSI_START);
	writel(1, r->dsi + DSI_START);
}

static enum hrtimer_restart mtk_disp_refresh_fn(struct hrtimer *t)
{
	struct mtk_disp_refresh *r = container_of(t, struct mtk_disp_refresh, timer);
	unsigned int hz = READ_ONCE(refresh_hz);

	if (!hz) {
		hrtimer_forward_now(t, ms_to_ktime(500));
		return HRTIMER_RESTART;
	}

	/* Skip this period if the previous frame or command is still being sent */
	if (!(readl(r->dsi + DSI_INTSTA) & DSI_BUSY)) {
		int level = xchg(&r->pending_brightness, -1);

		if (r->cmd_in_flight) {
			writel(r->frame_cmdq0, r->dsi + DSI_CMDQ0);
			writel(r->frame_cmdq_size, r->dsi + DSI_CMDQ_SIZE);
			r->cmd_in_flight = false;
		}

		if (level >= 0) {
			mtk_disp_refresh_send_brightness(r, level);
			r->cmd_in_flight = true;
		} else {
			writel(1, r->mutex + DISP_REG_MUTEX_EN(r->mutex_id));
			writel(0, r->dsi + DSI_START);
			writel(1, r->dsi + DSI_START);
		}
	}

	hrtimer_forward_now(t, ns_to_ktime(NSEC_PER_SEC / hz));
	return HRTIMER_RESTART;
}

static int mtk_disp_refresh_bl_update(struct backlight_device *bl)
{
	struct mtk_disp_refresh *r = bl_get_data(bl);

	WRITE_ONCE(r->pending_brightness, backlight_get_brightness(bl));

	return 0;
}

static const struct backlight_ops mtk_disp_refresh_bl_ops = {
	.update_status = mtk_disp_refresh_bl_update,
};

static int mtk_disp_refresh_probe(struct platform_device *pdev)
{
	struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.max_brightness = BRIGHTNESS_MAX,
		.brightness = BRIGHTNESS_MAX / 2,
	};
	struct device *dev = &pdev->dev;
	struct backlight_device *bl;
	struct mtk_disp_refresh *r;

	r = devm_kzalloc(dev, sizeof(*r), GFP_KERNEL);
	if (!r)
		return -ENOMEM;

	r->mutex = devm_platform_ioremap_resource_byname(pdev, "mutex");
	if (IS_ERR(r->mutex))
		return PTR_ERR(r->mutex);

	r->dsi = devm_platform_ioremap_resource_byname(pdev, "dsi");
	if (IS_ERR(r->dsi))
		return PTR_ERR(r->dsi);

	of_property_read_u32(dev->of_node, "mediatek,mutex-id", &r->mutex_id);

	r->frame_cmdq0 = readl(r->dsi + DSI_CMDQ0);
	r->frame_cmdq_size = readl(r->dsi + DSI_CMDQ_SIZE);
	r->pending_brightness = -1;

	bl = devm_backlight_device_register(dev, dev_name(dev), dev, r,
					    &mtk_disp_refresh_bl_ops, &props);
	if (IS_ERR(bl))
		return dev_err_probe(dev, PTR_ERR(bl), "Cannot register backlight\n");

	hrtimer_setup(&r->timer, mtk_disp_refresh_fn, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hrtimer_start(&r->timer, 0, HRTIMER_MODE_REL);
	platform_set_drvdata(pdev, r);

	return 0;
}

static void mtk_disp_refresh_remove(struct platform_device *pdev)
{
	struct mtk_disp_refresh *r = platform_get_drvdata(pdev);

	hrtimer_cancel(&r->timer);
}

static const struct of_device_id mtk_disp_refresh_of_match[] = {
	{ .compatible = "mediatek,mt6878-disp-refresh" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mtk_disp_refresh_of_match);

static struct platform_driver mtk_disp_refresh_driver = {
	.probe = mtk_disp_refresh_probe,
	.remove = mtk_disp_refresh_remove,
	.driver = {
		.name = "mtk-disp-refresh",
		.of_match_table = mtk_disp_refresh_of_match,
	},
};
module_platform_driver(mtk_disp_refresh_driver);

MODULE_DESCRIPTION("MediaTek bootloader command mode panel refresher");
MODULE_LICENSE("GPL");
