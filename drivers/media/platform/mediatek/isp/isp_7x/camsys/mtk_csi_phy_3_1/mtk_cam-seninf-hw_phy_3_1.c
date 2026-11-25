// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek seninf hardware layer for the CSI PHY 3.1 generation (MT6878)
 *
 * Copyright (c) 2022 MediaTek Inc.
 * Copyright (c) 2026 Jan Benedikt <karagos01@gmail.com>
 *
 * The register sequences follow the vendor mtk_csi_phy_3_1 implementation.
 * Only the D-PHY receiver is supported.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <media/mipi-csi2.h>

#include "mtk_cam-seninf.h"
#include "mtk_cam-seninf-hw.h"
#include "mtk_cam-seninf-route.h"
#include "mtk_cam-seninf-phy31-regs.h"

#define SENINF_HW_MUX_NUM	22
#define SENINF_HW_CAM_MUX_NUM	41
#define CAM_MUX_SRC_NONE	0x7f

/*
 * The PHY timings are counted in seninf clock cycles. 312 MHz is the
 * lowest seninf operating point (VCORE 0.575 V) and covers D-PHY lanes up
 * to 4 Gbps. Faster links need a higher seninf clock and therefore a
 * higher VCORE level, which is not supported yet.
 */
#define SENINF_DEFAULT_CLK_HZ	312000000ULL

/* the HS trail the PHY expects at 1 Mbps, in ns */
#define SENINF_HS_TRAIL_NS_MBPS	224000

static struct mtk_cam_seninf_cfg _seninf_cfg = {
	.seninf_num = SENINF_NUM,
	.mux_num = SENINF_HW_MUX_NUM,
	.cam_mux_num = SENINF_HW_CAM_MUX_NUM,
	.pref_mux_num = SENINF_HW_MUX_NUM,
};

struct mtk_cam_seninf_cfg *g_seninf_cfg = &_seninf_cfg;

/*
 * Each mux feeds the cam muxes through a "virtual" mux index. The camsv
 * "sat" and raw muxes carry several virtual channels each.
 */
struct seninf_mux_range {
	u8 first, last;		/* mux */
	u8 vr_first, vr_stride;	/* virtual mux */
};

static const struct seninf_mux_range mt6878_mux_ranges[] = {
	{ 0, 2, 0x00, 8 },	/* camsv sat */
	{ 4, 4, 0x20, 1 },	/* camsv normal */
	{ 6, 8, 0x22, 4 },	/* raw */
	{ 10, 10, 0x32, 1 },	/* pdp */
};

static int mux_to_mux_vr(int mux)
{
	const struct seninf_mux_range *r;
	int i;

	for (i = 0; i < ARRAY_SIZE(mt6878_mux_ranges); i++) {
		r = &mt6878_mux_ranges[i];
		if (mux >= r->first && mux <= r->last)
			return r->vr_first + (mux - r->first) * r->vr_stride;
	}

	return -EINVAL;
}

static int mux_vr_to_mux(int vr)
{
	const struct seninf_mux_range *r;
	int i, n;

	for (i = 0; i < ARRAY_SIZE(mt6878_mux_ranges); i++) {
		r = &mt6878_mux_ranges[i];
		n = (r->last - r->first + 1) * r->vr_stride;
		if (vr >= r->vr_first && vr < r->vr_first + n)
			return r->first + (vr - r->vr_first) / r->vr_stride;
	}

	return -EINVAL;
}

/* the seninf mux that can feed a given cam mux, or -EINVAL */
int mtk_cam_seninf_cammux_to_mux(int cam_mux)
{
	return mux_vr_to_mux(cam_mux);
}

static void seninf_rmw(void __iomem *base, u32 reg, u32 mask, u32 val)
{
	u32 v = readl(base + reg);

	writel((v & ~mask) | ((val << __ffs(mask)) & mask), base + reg);
}

static void seninf_set(void __iomem *base, u32 reg, u32 bits)
{
	writel(readl(base + reg) | bits, base + reg);
}

static void seninf_clr(void __iomem *base, u32 reg, u32 bits)
{
	writel(readl(base + reg) & ~bits, base + reg);
}

/* set or clear each bit of @bits in turn, low to high */
static void seninf_set_each(void __iomem *base, u32 reg, u32 bits, bool on)
{
	unsigned long b = bits;
	unsigned int i;

	for_each_set_bit(i, &b, 32) {
		if (on)
			seninf_set(base, reg, BIT(i));
		else
			seninf_clr(base, reg, BIT(i));
	}
}

static bool port_is_a(u32 port)
{
	return port >= CSI_PORT_0A && !((port - CSI_PORT_0A) & 1);
}

static bool port_is_b(u32 port)
{
	return port >= CSI_PORT_0A && ((port - CSI_PORT_0A) & 1);
}

static u64 seninf_clk_rate(struct seninf_ctx *ctx)
{
	unsigned long rate = clk_get_rate(ctx->core->clk[CLK_TOP_SENINF]);

	return rate ? rate : SENINF_DEFAULT_CLK_HZ;
}

static u32 seninf_bpp(struct seninf_ctx *ctx)
{
	switch (ctx->fmt[PAD_SINK].format.code) {
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SRGGB8_1X8:
		return 8;
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
		return 12;
	case MEDIA_BUS_FMT_SBGGR14_1X14:
	case MEDIA_BUS_FMT_SGBRG14_1X14:
	case MEDIA_BUS_FMT_SGRBG14_1X14:
	case MEDIA_BUS_FMT_SRGGB14_1X14:
		return 14;
	case MEDIA_BUS_FMT_UYVY8_1X16:
	case MEDIA_BUS_FMT_VYUY8_1X16:
	case MEDIA_BUS_FMT_YUYV8_1X16:
	case MEDIA_BUS_FMT_YVYU8_1X16:
		return 16;
	default:
		return 10;
	}
}

/* D-PHY bit rate of one data lane */
static u64 dphy_lane_rate(struct seninf_ctx *ctx)
{
	if (!ctx->num_data_lanes)
		return 0;

	return div_u64(ctx->mipi_pixel_rate * seninf_bpp(ctx),
		       ctx->num_data_lanes);
}

int mtk_cam_seninf_init_iomem(struct seninf_ctx *ctx, void __iomem *if_base,
			      void __iomem *ana_base)
{
	unsigned int i;

	for (i = 0; i < CSI_PORT_MAX_NUM; i++) {
		u32 csi = i < CSI_PORT_0A ? i : (i - CSI_PORT_0A) / 2;
		void __iomem *base = ana_base + csi * ANA_CSI_STRIDE;

		ctx->reg_ana_csi_rx[i] = base + (port_is_b(i) ? ANA_CSI_RX_B :
								ANA_CSI_RX_A);
		ctx->reg_ana_dphy_top[i] = base + ANA_DPHY_TOP;
		ctx->reg_ana_cphy_top[i] = base + ANA_CPHY_TOP;
		ctx->reg_ana_mac_top[i] = base + ANA_MAC_TOP;
		ctx->reg_ana_mac[i] = base + (port_is_b(i) ? ANA_MAC_B :
							     ANA_MAC_A);
	}

	ctx->reg_if_top = if_base;
	for (i = 0; i < SENINF_NUM; i++) {
		ctx->reg_if_ctrl[i] = if_base + IF_CTRL_BASE + i * IF_STRIDE;
		ctx->reg_if_csi2[i] = if_base + IF_CSI2_BASE + i * IF_STRIDE;
	}
	for (i = 0; i < SENINF_HW_MUX_NUM; i++)
		ctx->reg_if_mux[i] = if_base + IF_MUX_BASE + i * IF_STRIDE;
	for (i = 0; i < SENINF_HW_CAM_MUX_NUM; i++)
		ctx->reg_if_cam_mux_pcsr[i] = if_base + IF_CAM_MUX_BASE +
					      i * IF_CAM_MUX_STRIDE;
	ctx->reg_if_cam_mux_gcsr = if_base + IF_CAM_MUX_GCSR;

	return 0;
}

int mtk_cam_seninf_init_port(struct seninf_ctx *ctx, int port)
{
	u32 csi;

	if (port < 0 || port >= CSI_PORT_MAX_NUM)
		return -EINVAL;

	csi = port < CSI_PORT_0A ? port : (port - CSI_PORT_0A) / 2;
	ctx->port = port;
	ctx->port_num = csi;
	ctx->port_a = CSI_PORT_0A + csi * 2;
	ctx->port_b = ctx->port_a + 1;
	ctx->is_4d1c = port < CSI_PORT_0A;
	/* full and A ports use the even interface, B ports the odd one */
	ctx->seninf_idx = csi * 2 + port_is_b(port);

	return 0;
}

static bool cam_mux_valid(int cam_mux)
{
	return cam_mux >= 0 && cam_mux < SENINF_HW_CAM_MUX_NUM;
}

int mtk_cam_seninf_is_cammux_used(struct seninf_ctx *ctx, int cam_mux)
{
	if (!cam_mux_valid(cam_mux))
		return 0;

	return !!(readl(ctx->reg_if_cam_mux_pcsr[cam_mux] + CAM_MUX_PCSR_CTRL) &
		  CAM_MUX_PCSR_EN);
}

int mtk_cam_seninf_cammux(struct seninf_ctx *ctx, int cam_mux)
{
	void __iomem *base;

	if (!cam_mux_valid(cam_mux))
		return -EINVAL;

	base = ctx->reg_if_cam_mux_pcsr[cam_mux];
	seninf_set(base, CAM_MUX_PCSR_CTRL, CAM_MUX_PCSR_EN);
	writel(CAM_MUX_PCSR_IRQ_CLR_ALL, base + CAM_MUX_PCSR_IRQ_STATUS);

	return 0;
}

int mtk_cam_seninf_disable_cammux(struct seninf_ctx *ctx, int cam_mux)
{
	void __iomem *gcsr = ctx->reg_if_cam_mux_gcsr;
	void __iomem *base;
	u32 page;

	if (!cam_mux_valid(cam_mux))
		return -EINVAL;

	base = ctx->reg_if_cam_mux_pcsr[cam_mux];
	seninf_clr(gcsr, CAM_MUX_GCSR_VSYNC_IRQ_EN(cam_mux / 32),
		   BIT(cam_mux % 32));
	seninf_clr(gcsr, CAM_MUX_GCSR_IRQ_EN, CAM_MUX_GCSR_DROP_IRQ_EN);

	seninf_set(base, CAM_MUX_PCSR_CTRL, CAM_MUX_PCSR_NEXT_SRC_SEL);
	seninf_set(base, CAM_MUX_PCSR_CTRL, CAM_MUX_PCSR_SRC_SEL);
	seninf_clr(base, CAM_MUX_PCSR_CTRL, CAM_MUX_PCSR_EN);
	for (page = 0; page < 4; page++) {
		seninf_rmw(base, CAM_MUX_PCSR_OPT, CAM_MUX_PCSR_TAG_PAGE, page);
		writel(0, base + CAM_MUX_PCSR_TAG_VC);
		writel(0, base + CAM_MUX_PCSR_TAG_DT);
	}
	writel(CAM_MUX_PCSR_IRQ_CLR_ALL, base + CAM_MUX_PCSR_IRQ_STATUS);

	return 0;
}

int mtk_cam_seninf_disable_all_cammux(struct seninf_ctx *ctx)
{
	int i;

	for (i = 0; i < SENINF_HW_CAM_MUX_NUM; i++)
		mtk_cam_seninf_disable_cammux(ctx, i);

	return 0;
}

int mtk_cam_seninf_set_top_mux_ctrl(struct seninf_ctx *ctx, int mux_idx,
				    int seninf_src)
{
	struct seninf_core *core = ctx->core;

	if (mux_idx < 0 || mux_idx >= SENINF_HW_MUX_NUM)
		return -EINVAL;

	mutex_lock(&core->mutex);
	seninf_rmw(ctx->reg_if_top, SENINF_TOP_MUX_CTRL(mux_idx),
		   SENINF_TOP_MUX_SRC(mux_idx), seninf_src);
	mutex_unlock(&core->mutex);

	return 0;
}

int mtk_cam_seninf_get_top_mux_ctrl(struct seninf_ctx *ctx, int mux_idx)
{
	u32 val;

	if (mux_idx < 0 || mux_idx >= SENINF_HW_MUX_NUM)
		return -EINVAL;

	val = readl(ctx->reg_if_top + SENINF_TOP_MUX_CTRL(mux_idx));

	return (val & SENINF_TOP_MUX_SRC(mux_idx)) >> ((mux_idx % 4) * 8);
}

int mtk_cam_seninf_get_cammux_ctrl(struct seninf_ctx *ctx, int cam_mux)
{
	u32 val;

	if (!cam_mux_valid(cam_mux))
		return 0;

	val = readl(ctx->reg_if_cam_mux_pcsr[cam_mux] + CAM_MUX_PCSR_CTRL);

	return FIELD_GET(CAM_MUX_PCSR_SRC_SEL, val);
}

u32 mtk_cam_seninf_get_cammux_res(struct seninf_ctx *ctx, int cam_mux)
{
	if (!cam_mux_valid(cam_mux))
		return 0;

	return readl(ctx->reg_if_cam_mux_pcsr[cam_mux] + CAM_MUX_PCSR_RES);
}

int mtk_cam_seninf_set_cammux_vc(struct seninf_ctx *ctx, int cam_mux,
				 int vc_sel, int dt_sel, int vc_en, int dt_en)
{
	void __iomem *base;

	if (!cam_mux_valid(cam_mux))
		return -EINVAL;

	base = ctx->reg_if_cam_mux_pcsr[cam_mux];
	seninf_rmw(base, CAM_MUX_PCSR_OPT, CAM_MUX_PCSR_VC_SEL, vc_sel);
	seninf_rmw(base, CAM_MUX_PCSR_OPT, CAM_MUX_PCSR_DT_SEL, dt_sel);
	seninf_rmw(base, CAM_MUX_PCSR_OPT, CAM_MUX_PCSR_VC_EN, !!vc_en);
	seninf_rmw(base, CAM_MUX_PCSR_OPT, CAM_MUX_PCSR_DT_EN, !!dt_en);

	return 0;
}

/*
 * Map a VC/DT pair to a capture tag. Each register page holds four tags;
 * "first" marks the tag that starts a frame.
 */
int mtk_cam_seninf_set_cammux_tag(struct seninf_ctx *ctx, int cam_mux,
				  int vc_sel, int dt_sel, int tag, bool first)
{
	void __iomem *base;
	u32 shift;

	if (!cam_mux_valid(cam_mux) || tag < 0 || tag > 31)
		return -EINVAL;

	base = ctx->reg_if_cam_mux_pcsr[cam_mux];
	shift = (tag % 4) * 8;
	seninf_rmw(base, CAM_MUX_PCSR_OPT, CAM_MUX_PCSR_TAG_PAGE, tag / 4);
	seninf_rmw(base, CAM_MUX_PCSR_TAG_VC, GENMASK(7, 0) << shift,
		   (first ? BIT(7) : 0) | (vc_sel & 0x1f));
	seninf_rmw(base, CAM_MUX_PCSR_TAG_DT, GENMASK(7, 0) << shift,
		   (first ? BIT(7) : 0) | (dt_sel & 0x3f));

	return 0;
}

int mtk_cam_seninf_set_cammux_src(struct seninf_ctx *ctx, int src, int target)
{
	struct v4l2_mbus_framefmt *fmt = &ctx->fmt[PAD_SINK].format;
	void __iomem *base;
	int vr;

	if (!cam_mux_valid(target))
		return -EINVAL;

	vr = mux_to_mux_vr(src);
	if (vr < 0) {
		dev_err(ctx->dev, "mux %d cannot feed a cam mux\n", src);
		return vr;
	}

	base = ctx->reg_if_cam_mux_pcsr[target];
	seninf_rmw(base, CAM_MUX_PCSR_CTRL, CAM_MUX_PCSR_SRC_SEL, vr);
	seninf_rmw(base, CAM_MUX_PCSR_CHK_RES, CAM_MUX_PCSR_EXP_HSIZE,
		   fmt->width);
	seninf_rmw(base, CAM_MUX_PCSR_CHK_RES, CAM_MUX_PCSR_EXP_VSIZE,
		   fmt->height);

	return 0;
}

int mtk_cam_seninf_set_vc(struct seninf_ctx *ctx, u32 seninf_idx,
			  struct seninf_vcinfo *vcinfo)
{
	void __iomem *csi2, *mac;
	int i;

	if (seninf_idx >= SENINF_NUM)
		return -EINVAL;
	if (!vcinfo || !vcinfo->cnt)
		return 0;
	if (vcinfo->cnt > 3) {
		dev_err(ctx->dev, "%d streams, at most 3 supported\n",
			vcinfo->cnt);
		return -EINVAL;
	}

	csi2 = ctx->reg_if_csi2[seninf_idx];
	mac = ctx->reg_ana_mac[ctx->port];

	for (i = 0; i < 8; i++)
		writel(0, csi2 + SENINF_CSI2_DI_CTRL(i));
	for (i = 0; i < 4; i++)
		writel(0, csi2 + SENINF_CSI2_CH_CTRL(i));

	/* no data type remapping; force the generic types to RAW */
	writel(0, mac + CSIRX_MAC_CSI2_DT_REMAP);
	writel(0, mac + CSIRX_MAC_CSI2_DT_REMAP);
	seninf_rmw(mac, CSIRX_MAC_CSI2_FORCE_DT, GENMASK(29, 24), 0x2e);
	seninf_rmw(mac, CSIRX_MAC_CSI2_FORCE_DT, GENMASK(21, 16), 0x2d);
	seninf_rmw(mac, CSIRX_MAC_CSI2_FORCE_DT, GENMASK(13, 8), 0x2c);
	seninf_rmw(mac, CSIRX_MAC_CSI2_FORCE_DT, GENMASK(5, 0), 0x2b);

	for (i = 0; i < vcinfo->cnt; i++) {
		u32 di = SENINF_CSI2_DI_CTRL(i);

		seninf_rmw(csi2, di, SENINF_CSI2_DI_DT, vcinfo->vc[i].dt);
		seninf_rmw(csi2, di, SENINF_CSI2_DI_VC, 0);
		seninf_rmw(csi2, di, SENINF_CSI2_DI_GRP, 1);
		seninf_set(csi2, di, SENINF_CSI2_DI_EN);
		seninf_set(csi2, SENINF_CSI2_CH_CTRL(0),
			   SENINF_CSI2_CH_DI_EN(i));
	}

	return 0;
}

int mtk_cam_seninf_set_mux_ctrl(struct seninf_ctx *ctx, u32 mux, int hs_pol,
				int vs_pol, int src_sel, int pixel_mode)
{
	void __iomem *base;
	u32 val;

	if (mux >= SENINF_HW_MUX_NUM)
		return -EINVAL;

	base = ctx->reg_if_mux[mux];
	seninf_rmw(base, SENINF_MUX_CTRL_1, SENINF_MUX_SRC_SEL, src_sel);
	seninf_rmw(base, SENINF_MUX_CTRL_1, SENINF_MUX_PIX_MODE_SEL,
		   pixel_mode);
	seninf_set(base, SENINF_MUX_OPT, SENINF_MUX_CNT_INIT_EN);
	seninf_rmw(base, SENINF_MUX_OPT, SENINF_MUX_HSYNC_POL, hs_pol);
	seninf_rmw(base, SENINF_MUX_OPT, SENINF_MUX_VSYNC_POL, vs_pol);

	val = readl(base + SENINF_MUX_CTRL_0);
	writel(val | SENINF_MUX_IRQ_SW_RST | SENINF_MUX_SW_RST,
	       base + SENINF_MUX_CTRL_0);
	writel(val & ~(SENINF_MUX_IRQ_SW_RST | SENINF_MUX_SW_RST),
	       base + SENINF_MUX_CTRL_0);

	return 0;
}

int mtk_cam_seninf_update_mux_pixel_mode(struct seninf_ctx *ctx, u32 mux,
					 int pixel_mode)
{
	void __iomem *base;
	u32 val;

	if (mux >= SENINF_HW_MUX_NUM)
		return -EINVAL;

	base = ctx->reg_if_mux[mux];
	seninf_rmw(base, SENINF_MUX_CTRL_1, SENINF_MUX_PIX_MODE_SEL,
		   pixel_mode);

	val = readl(base + SENINF_MUX_CTRL_0);
	writel(val | SENINF_MUX_IRQ_SW_RST | SENINF_MUX_SW_RST,
	       base + SENINF_MUX_CTRL_0);
	writel(val & ~(SENINF_MUX_IRQ_SW_RST | SENINF_MUX_SW_RST),
	       base + SENINF_MUX_CTRL_0);

	return 0;
}

int mtk_cam_seninf_set_mux_crop(struct seninf_ctx *ctx, u32 mux, int start_x,
				int end_x, int enable)
{
	void __iomem *base;
	int start, cnt;

	if (mux >= SENINF_HW_MUX_NUM || start_x < 0 || end_x < start_x)
		return -EINVAL;

	base = ctx->reg_if_mux[mux];
	start = start_x / 8;
	cnt = DIV_ROUND_UP(end_x - start_x + 1, 8);
	seninf_rmw(base, SENINF_MUX_CROP_PIX_CTRL, SENINF_MUX_CROP_START_8PIX,
		   start);
	seninf_rmw(base, SENINF_MUX_CROP_PIX_CTRL, SENINF_MUX_CROP_END_8PIX,
		   start + cnt - 1);
	seninf_rmw(base, SENINF_MUX_CTRL_1, SENINF_MUX_CROP_EN, !!enable);

	return 0;
}

int mtk_cam_seninf_is_mux_used(struct seninf_ctx *ctx, u32 mux)
{
	if (mux >= SENINF_HW_MUX_NUM)
		return 0;

	return readl(ctx->reg_if_mux[mux] + SENINF_MUX_CTRL_0) & SENINF_MUX_EN;
}

int mtk_cam_seninf_mux(struct seninf_ctx *ctx, u32 mux)
{
	if (mux >= SENINF_HW_MUX_NUM)
		return -EINVAL;

	seninf_set(ctx->reg_if_mux[mux], SENINF_MUX_CTRL_0, SENINF_MUX_EN);

	return 0;
}

int mtk_cam_seninf_disable_mux(struct seninf_ctx *ctx, u32 mux)
{
	int i;

	if (mux >= SENINF_HW_MUX_NUM)
		return -EINVAL;

	seninf_clr(ctx->reg_if_mux[mux], SENINF_MUX_CTRL_0, SENINF_MUX_EN);

	/* release the cam muxes fed by this mux as well */
	for (i = 0; i < SENINF_HW_CAM_MUX_NUM; i++)
		if (mux_vr_to_mux(mtk_cam_seninf_get_cammux_ctrl(ctx, i)) == mux)
			mtk_cam_seninf_disable_cammux(ctx, i);

	return 0;
}

int mtk_cam_seninf_disable_all_mux(struct seninf_ctx *ctx)
{
	int i;

	for (i = 0; i < SENINF_HW_MUX_NUM; i++)
		seninf_clr(ctx->reg_if_mux[i], SENINF_MUX_CTRL_0, SENINF_MUX_EN);

	return 0;
}

int mtk_cam_seninf_set_cammux_chk_pixel_mode(struct seninf_ctx *ctx,
					     int cam_mux, int pixel_mode)
{
	if (!cam_mux_valid(cam_mux))
		return -EINVAL;

	seninf_rmw(ctx->reg_if_cam_mux_pcsr[cam_mux], CAM_MUX_PCSR_CTRL,
		   CAM_MUX_PCSR_CHK_PIX_MODE, pixel_mode);

	return 0;
}

/* --- CSI-2 receiver ----------------------------------------------------- */

static void csirx_phy_a_init(void __iomem *base)
{
	seninf_rmw(base, CDPHY_RX_ANA_1, GENMASK(2, 0), 4);
	seninf_rmw(base, CDPHY_RX_ANA_1, GENMASK(6, 4), 4);
	seninf_rmw(base, CDPHY_RX_ANA_2, GENMASK(26, 24), 4);
	seninf_rmw(base, CDPHY_RX_ANA_2, GENMASK(30, 28), 4);
	seninf_rmw(base, CDPHY_RX_ANA_1, GENMASK(11, 8), 8);
	seninf_rmw(base, CDPHY_RX_ANA_1, GENMASK(18, 16), 2);
	seninf_rmw(base, CDPHY_RX_ANA_5, GENMASK(1, 0), 1);
	seninf_rmw(base, CDPHY_RX_ANA_5, GENMASK(3, 2), 1);
	seninf_clr(base, CDPHY_RX_ANA_5, BIT(4));
	seninf_clr(base, CDPHY_RX_ANA_5, GENMASK(11, 8));
	seninf_rmw(base, CDPHY_RX_ANA_2, GENMASK(4, 0), 0x10);
	seninf_rmw(base, CDPHY_RX_ANA_2, GENMASK(12, 8), 0x10);
	seninf_rmw(base, CDPHY_RX_ANA_3, GENMASK(4, 0), 0x10);
	seninf_rmw(base, CDPHY_RX_ANA_3, GENMASK(12, 8), 0x10);
	seninf_rmw(base, CDPHY_RX_ANA_4, GENMASK(4, 0), 0x10);
	seninf_rmw(base, CDPHY_RX_ANA_4, GENMASK(12, 8), 0x10);
	seninf_clr(base, CDPHY_RX_ANA_0, BIT(21));
	seninf_clr(base, CDPHY_RX_ANA_0, BIT(22));
	seninf_clr(base, CDPHY_RX_ANA_2, BIT(16));
	seninf_clr(base, CDPHY_RX_ANA_2, BIT(17));
	seninf_rmw(base, CDPHY_RX_ANA_6, GENMASK(28, 24), 4);
	seninf_rmw(base, CDPHY_RX_ANA_7, GENMASK(28, 24), 4);
	seninf_rmw(base, CDPHY_RX_ANA_6, GENMASK(4, 0), 10);
	seninf_rmw(base, CDPHY_RX_ANA_6, GENMASK(12, 8), 10);
	seninf_rmw(base, CDPHY_RX_ANA_6, GENMASK(20, 16), 10);
	seninf_rmw(base, CDPHY_RX_ANA_7, GENMASK(4, 0), 10);
	seninf_rmw(base, CDPHY_RX_ANA_7, GENMASK(12, 8), 10);
	seninf_rmw(base, CDPHY_RX_ANA_7, GENMASK(20, 16), 10);
	seninf_set(base, CDPHY_RX_ANA_5, BIT(30));
	seninf_set(base, CDPHY_RX_ANA_14, BIT(24));
	seninf_rmw(base, CDPHY_RX_ANA_8, GENMASK(15, 0), 0x100);
}

/* the efuse holds 5 bit trims of ANA_2..4 for the A then the B half */
static void csirx_phy_a_efuse(void __iomem *base, u32 efuse, int shift)
{
	static const u32 regs[] = {
		CDPHY_RX_ANA_2, CDPHY_RX_ANA_3, CDPHY_RX_ANA_4,
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		u32 trim = (efuse >> (shift - 5 * i)) & 0x1f;

		seninf_rmw(base, regs[i], GENMASK(4, 0), trim);
		seninf_rmw(base, regs[i], GENMASK(12, 8), trim);
	}
}

static void csirx_apply_efuse(struct seninf_ctx *ctx)
{
	u32 efuse = ctx->m_csi_efuse;

	/* blank (all zeroes) or not passed on by the bootloader (all ones) */
	if (!efuse || efuse == U32_MAX)
		return;

	if (ctx->is_4d1c || ctx->port == ctx->port_a)
		csirx_phy_a_efuse(ctx->reg_ana_csi_rx[ctx->port], efuse, 27);
	if (ctx->is_4d1c)
		csirx_phy_a_efuse(ctx->reg_ana_csi_rx[ctx->port_b], efuse, 12);
	else if (ctx->port == ctx->port_b)
		csirx_phy_a_efuse(ctx->reg_ana_csi_rx[ctx->port], efuse, 12);
}

/* settle in seninf clock cycles: @ns, or the vendor default for 0 */
static u32 dphy_settle_cycles(u64 clk, u32 ns)
{
	if (ns)
		return DIV_ROUND_UP_ULL(clk * ns, NSEC_PER_SEC);

	return div_u64(clk, 10000000) - ((clk * 100) % NSEC_PER_SEC ? 5 : 6);
}

static void csirx_dphy_timing(struct seninf_ctx *ctx)
{
	void __iomem *top = ctx->reg_ana_dphy_top[ctx->port];
	bool custom = ctx->dphy_settle_delay_dt || ctx->settle_delay_ck;
	u64 clk = seninf_clk_rate(ctx);
	u32 settle_dt, settle_ck = 0, trail = 0;
	int i;

	settle_dt = dphy_settle_cycles(clk, custom ?
					    ctx->dphy_settle_delay_dt : 0);
	if (custom)
		settle_ck = dphy_settle_cycles(clk, ctx->settle_delay_ck);

	if (ctx->hs_trail_parameter) {
		u64 mbps = div_u64(dphy_lane_rate(ctx), 1000000);
		u32 ns = mbps ? div64_u64(SENINF_HS_TRAIL_NS_MBPS, mbps) : 0;

		if (ctx->hs_trail_parameter <= ns)
			trail = DIV_ROUND_UP_ULL((u64)(ns - ctx->hs_trail_parameter) *
						 clk, NSEC_PER_SEC);
	}

	for (i = 0; i < 4; i++)
		seninf_rmw(top, DPHY_RX_DATA_LANE_HS_PARAMETER(i),
			   RG_DPHY_RX_LD_HS_SETTLE, settle_dt);
	for (i = 0; i < 2; i++)
		seninf_rmw(top, DPHY_RX_CLOCK_LANE_HS_PARAMETER(i),
			   RG_DPHY_RX_LC_HS_SETTLE, settle_ck);
	for (i = 0; i < 4; i++)
		seninf_clr(top, DPHY_RX_DATA_LANE_HS_PARAMETER(i),
			   RG_DPHY_RX_LD_HS_PREPARE);
	for (i = 0; i < 4; i++)
		seninf_rmw(top, DPHY_RX_DATA_LANE_HS_PARAMETER(i),
			   RG_DPHY_RX_LD_HS_TRAIL, trail);
	for (i = 0; i < 4; i++)
		seninf_rmw(top, DPHY_RX_DATA_LANE_HS_PARAMETER(i),
			   RG_DPHY_RX_LD_HS_TRAIL_EN, !!trail);

	seninf_set(top, DPHY_RX_DESKEW_CTRL, BIT(8));
	for (i = 0; i < 4; i++)
		seninf_clr(top, DPHY_RX_DESKEW_LANE_CTRL(i),
			   RG_DPHY_RX_DESKEW_EN);
	for (i = 0; i < 4; i++)
		seninf_set(top, DPHY_RX_DATA_LANE_HS_PARAMETER(i),
			   RG_DPHY_RX_HS_SETTLE_SEL);
	for (i = 0; i < 2; i++) {
		seninf_clr(top, DPHY_RX_CLOCK_LANE_HS_PARAMETER(i),
			   GENMASK(7, 0));
		seninf_set(top, DPHY_RX_CLOCK_LANE_HS_PARAMETER(i),
			   RG_DPHY_RX_HS_SETTLE_SEL);
	}
}

static void csirx_mac_top_setting(struct seninf_ctx *ctx)
{
	void __iomem *top = ctx->reg_ana_mac_top[ctx->port];

	seninf_rmw(top, CSIRX_MAC_TOP_CTRL, CSIRX_MAC_TOP_HALF_PORT,
		   ctx->port >= CSI_PORT_0A);
	seninf_clr(top, CSIRX_MAC_TOP_PHY_CTRL, CSIRX_MAC_TOP_CPHY_SEL);
	seninf_set(top, CSIRX_MAC_TOP_PHY_CTRL, CSIRX_MAC_TOP_DPHY_SEL);
	seninf_clr(top, CSIRX_MAC_TOP_PHY_CTRL, CSIRX_MAC_TOP_CPHY_MODE);
}

static void csirx_mac_csi_opt(void __iomem *mac)
{
	seninf_set_each(mac, CSIRX_MAC_CSI2_OPT, CSIRX_MAC_CSI2_OPT_FIXED_SET,
			true);
	seninf_set_each(mac, CSIRX_MAC_CSI2_OPT, CSIRX_MAC_CSI2_OPT_FIXED_CLR,
			false);
}

static void csirx_mac_csi_fixed(void __iomem *mac)
{
	csirx_mac_csi_opt(mac);
	seninf_clr(mac, CSIRX_MAC_CSI2_RESYNC_MERGE_CTRL, GENMASK(15, 12));
	seninf_set(mac, CSIRX_MAC_CSI2_RESYNC_MERGE_CTRL, BIT(8));
	seninf_set(mac, CSIRX_MAC_CSI2_RESYNC_MERGE_CTRL, GENMASK(4, 0));
	seninf_set_each(mac, CSIRX_MAC_CSI2_ERR_CTRL, GENMASK(3, 0), true);
	seninf_clr(mac, CSIRX_MAC_CSI2_OPT, BIT(19));
	seninf_clr(mac, CSIRX_MAC_CSI2_HS_TRAIL, BIT(24));
	seninf_clr(mac, CSIRX_MAC_CSI2_HS_TRAIL, BIT(25));
	seninf_clr(mac, CSIRX_MAC_CSI2_RESYNC_CYCLE, GENMASK(7, 0));
}

static int csirx_mac_csi_setting(struct seninf_ctx *ctx)
{
	void __iomem *mac = ctx->reg_ana_mac[ctx->port];
	u32 lanes = ctx->num_data_lanes;

	if (ctx->is_4d1c) {
		csirx_mac_csi_fixed(ctx->reg_ana_mac[ctx->port_a]);
		csirx_mac_csi_fixed(ctx->reg_ana_mac[ctx->port_b]);
	} else {
		csirx_mac_csi_fixed(mac);
	}

	seninf_clr(mac, CSIRX_MAC_CSI2_OPT, CSIRX_MAC_CSI2_CPHY_SEL);

	if (port_is_b(ctx->port)) {
		/* the B half only has a single data lane */
		seninf_set(mac, CSIRX_MAC_CSI2_EN, CSIRX_MAC_CSI2_LANE_EN(0));
		seninf_clr(mac, CSIRX_MAC_CSI2_EN, CSIRX_MAC_CSI2_LANE_EN(1));
		seninf_clr(mac, CSIRX_MAC_CSI2_EN, CSIRX_MAC_CSI2_LANE_EN(2));
		seninf_clr(mac, CSIRX_MAC_CSI2_EN, CSIRX_MAC_CSI2_LANE_EN(3));
	} else if (lanes >= 1 && lanes <= (ctx->is_4d1c ? 4 : 2)) {
		int i;

		for (i = 3; i >= 0; i--)
			seninf_rmw(mac, CSIRX_MAC_CSI2_EN,
				   CSIRX_MAC_CSI2_LANE_EN(i), i < lanes);
	} else {
		dev_err(ctx->dev, "port %u can't take %u data lanes\n",
			ctx->port, lanes);
		return -EINVAL;
	}

	seninf_rmw(mac, CSIRX_MAC_CSI2_OPT, CSIRX_MAC_CSI2_HALF_PORT,
		   ctx->port >= CSI_PORT_0A);
	csirx_mac_csi_opt(mac);
	seninf_set(mac, CSIRX_MAC_CSI2_RESYNC_MERGE_CTRL, GENMASK(4, 0));
	writel(0x2b0011, mac + CSIRX_MAC_CSI2_DT_CTRL(0));
	writel(0x2b0111, mac + CSIRX_MAC_CSI2_DT_CTRL(1));
	writel(0x2b0211, mac + CSIRX_MAC_CSI2_DT_CTRL(2));
	writel(0x2c0011, mac + CSIRX_MAC_CSI2_DT_CTRL(3));
	writel(0x2d0011, mac + CSIRX_MAC_CSI2_DT_CTRL(4));
	seninf_set(mac, CSIRX_MAC_CSI2_RESYNC_MERGE_CTRL, BIT(8));
	writel(BIT(31), mac + CSIRX_MAC_CSI2_IRQ_EN);
	seninf_set(mac, CSIRX_MAC_CSI2_DBG_CTRL, BIT(17));
	seninf_clr(mac, CSIRX_MAC_CSI2_CYCLE, GENMASK(7, 0));
	seninf_clr(mac, CSIRX_MAC_CSI2_CYCLE, GENMASK(10, 8));
	seninf_clr(mac, CSIRX_MAC_CSI2_RESYNC_MERGE_CTRL, GENMASK(27, 16));
	seninf_clr(mac, CSIRX_MAC_CSI2_RESYNC_MERGE_CTRL, GENMASK(31, 28));
	seninf_clr(mac, CSIRX_MAC_CSI2_RESYNC_MERGE_CTRL, GENMASK(15, 12));
	seninf_clr(mac, CSIRX_MAC_CSI2_RESYNC_MERGE_CTRL, BIT(11));

	/* no low-rate turnaround */
	seninf_clr(mac, CSIRX_MAC_CSI2_RESYNC_MERGE_CTRL, BIT(11));
	seninf_clr(ctx->reg_ana_cphy_top[ctx->port], CPHY_RX_LRTE_CTRL, BIT(1));

	return 0;
}

static void seninf_top_ctrl2(struct seninf_ctx *ctx, bool enable)
{
	u32 bit = ctx->port_num + (port_is_b(ctx->port) ? 8 : 0);

	seninf_rmw(ctx->reg_if_top, SENINF_TOP_CTRL2, BIT(bit), enable);
}

static void csirx_phy_a_power(void __iomem *base, bool on)
{
	seninf_set_each(base, CDPHY_RX_ANA_8, RG_CSI_LANE_LDO_EN, false);
	seninf_clr(base, CDPHY_RX_ANA_0, RG_CSI_BG_LPF_EN);
	seninf_clr(base, CDPHY_RX_ANA_0, RG_CSI_BG_CORE_EN);
	udelay(200);

	if (!on)
		return;

	seninf_set(base, CDPHY_RX_ANA_0, RG_CSI_BG_CORE_EN);
	udelay(30);
	seninf_set(base, CDPHY_RX_ANA_0, RG_CSI_BG_LPF_EN);
	udelay(1);
	seninf_set_each(base, CDPHY_RX_ANA_8, RG_CSI_LANE_LDO_EN, true);
	udelay(1);
}

/* rate dependent equalizer setup of one half port */
static void csirx_phy_a_dphy_rate(void __iomem *base, u64 rate, bool full)
{
	if (rate < 2500000000ULL) {
		seninf_rmw(base, CDPHY_RX_ANA_5, GENMASK(1, 0), 1);
		seninf_rmw(base, CDPHY_RX_ANA_5, BIT(4), rate >= 1500000000ULL);
		seninf_clr(base, CDPHY_RX_ANA_5, GENMASK(11, 8));
		seninf_rmw(base, CDPHY_RX_ANA_5, GENMASK(3, 2), 2);
		seninf_set(base, CDPHY_RX_ANA_14, BIT(8));
	} else {
		if (full)
			seninf_set(base, CDPHY_RX_ANA_5, GENMASK(1, 0));
		else
			seninf_rmw(base, CDPHY_RX_ANA_5, GENMASK(1, 0), 2);
		seninf_set(base, CDPHY_RX_ANA_5, BIT(4));
		seninf_clr(base, CDPHY_RX_ANA_5, GENMASK(11, 8));
		seninf_rmw(base, CDPHY_RX_ANA_5, GENMASK(3, 2), 1);
		seninf_clr(base, CDPHY_RX_ANA_14, BIT(8));
	}
}

static void csirx_phy_a_dphy_half(void __iomem *base, u64 rate)
{
	seninf_clr(base, CDPHY_RX_ANA_0, RG_CSI_CPHY_EN);
	seninf_clr(base, CDPHY_RX_ANA_0, RG_CSI_CPHY_T0_CDR_EN);
	seninf_clr(base, CDPHY_RX_ANA_0, RG_CSI_CPHY_T1_CDR_EN);
	seninf_clr(base, CDPHY_RX_ANA_0, RG_CSI_CPHY_T2_CDR_EN);
	seninf_clr(base, CDPHY_RX_ANA_0, RG_CSI_DPHY_L0_CKMODE_EN);
	seninf_clr(base, CDPHY_RX_ANA_0, RG_CSI_DPHY_L1_CKMODE_EN);
	seninf_clr(base, CDPHY_RX_ANA_0, RG_CSI_DPHY_L2_CKMODE_EN);
	/* lane 1 carries the clock */
	seninf_clr(base, CDPHY_RX_ANA_0, RG_CSI_CPHY_T0_CDR_EN);
	seninf_set(base, CDPHY_RX_ANA_0, RG_CSI_CPHY_T1_CDR_EN);
	seninf_clr(base, CDPHY_RX_ANA_0, RG_CSI_CPHY_T2_CDR_EN);
	seninf_set(base, CDPHY_RX_ANA_0, RG_CSI_DPHY_L0_CKSEL);
	seninf_set(base, CDPHY_RX_ANA_0, RG_CSI_DPHY_L1_CKSEL);
	csirx_phy_a_dphy_rate(base, rate, false);
}

static void csirx_phy_a_dphy_4d1c(void __iomem *a, void __iomem *b, u64 rate)
{
	static const u32 cdr = RG_CSI_CPHY_T0_CDR_EN | RG_CSI_CPHY_T1_CDR_EN |
			       RG_CSI_CPHY_T2_CDR_EN;
	static const u32 ckmode = RG_CSI_DPHY_L0_CKMODE_EN |
				  RG_CSI_DPHY_L1_CKMODE_EN |
				  RG_CSI_DPHY_L2_CKMODE_EN;
	static const u32 cksel = RG_CSI_DPHY_L0_CKSEL | RG_CSI_DPHY_L1_CKSEL;

	seninf_clr(a, CDPHY_RX_ANA_0, RG_CSI_CPHY_EN);
	seninf_clr(b, CDPHY_RX_ANA_0, RG_CSI_CPHY_EN);
	seninf_set_each(a, CDPHY_RX_ANA_0, cdr, false);
	seninf_set_each(b, CDPHY_RX_ANA_0, cdr, false);
	seninf_set_each(a, CDPHY_RX_ANA_0, ckmode, true);
	seninf_set_each(b, CDPHY_RX_ANA_0, ckmode, true);
	/* lane 2 of the A half carries the clock */
	seninf_clr(a, CDPHY_RX_ANA_0, RG_CSI_CPHY_T0_CDR_EN);
	seninf_clr(a, CDPHY_RX_ANA_0, RG_CSI_CPHY_T1_CDR_EN);
	seninf_set(a, CDPHY_RX_ANA_0, RG_CSI_CPHY_T2_CDR_EN);
	seninf_set_each(b, CDPHY_RX_ANA_0, cdr, false);
	seninf_set_each(a, CDPHY_RX_ANA_0, cksel, true);
	seninf_set_each(b, CDPHY_RX_ANA_0, cksel, true);
	csirx_phy_a_dphy_rate(a, rate, true);
	csirx_phy_a_dphy_rate(b, rate, true);
}

static void csirx_phy_a_bias(void __iomem *base)
{
	seninf_rmw(base, CDPHY_RX_ANA_SETTING_1, GENMASK(29, 24), 3);
	seninf_clr(base, CDPHY_RX_ANA_SETTING_1, GENMASK(19, 16));
	seninf_set(base, CDPHY_RX_ANA_SETTING_1, BIT(30));
}

static void dphy_lanes(void __iomem *top, u32 en, const u8 sel[6])
{
	static const u32 lane_bits = DPHY_RX_LC0_EN | DPHY_RX_LC1_EN |
		DPHY_RX_LD0_EN | DPHY_RX_LD1_EN | DPHY_RX_LD2_EN |
		DPHY_RX_LD3_EN;
	unsigned long bits = lane_bits;
	unsigned int i;

	for_each_set_bit(i, &bits, 32)
		seninf_rmw(top, DPHY_RX_LANE_EN, BIT(i), !!(en & BIT(i)));
	for (i = 0; i < 6; i++)
		seninf_rmw(top, DPHY_RX_LANE_SELECT, DPHY_RX_LANE_SEL(i),
			   sel[i]);
}

static int csirx_dphy_lane_setting(struct seninf_ctx *ctx)
{
	void __iomem *a = ctx->reg_ana_csi_rx[ctx->port_a];
	void __iomem *b = ctx->reg_ana_csi_rx[ctx->port_b];
	void __iomem *top = ctx->reg_ana_dphy_top[ctx->port];
	u32 lanes = ctx->num_data_lanes;

	if (ctx->is_4d1c) {
		static const u8 sel4[6] = { 2, 0, 1, 3, 0, 4 };
		static const u8 sel2[6] = { 2, 0, 1, 3, 0, 0 };
		static const u8 sel1[6] = { 2, 0, 1, 0, 0, 0 };

		csirx_phy_a_bias(a);
		seninf_clr(a, CDPHY_RX_ANA_SETTING_1, BIT(3));
		csirx_phy_a_bias(b);

		switch (lanes) {
		case 4:
			dphy_lanes(top, DPHY_RX_LC0_EN | DPHY_RX_LD0_EN |
				   DPHY_RX_LD1_EN | DPHY_RX_LD2_EN |
				   DPHY_RX_LD3_EN, sel4);
			seninf_set(a, CDPHY_RX_ANA_64, BIT(0));
			seninf_set(a, CDPHY_RX_ANA_64, BIT(8));
			seninf_set(b, CDPHY_RX_ANA_64, BIT(0));
			seninf_set(b, CDPHY_RX_ANA_64, BIT(8));
			break;
		case 2:
			dphy_lanes(top, DPHY_RX_LC0_EN | DPHY_RX_LD0_EN |
				   DPHY_RX_LD1_EN, sel2);
			seninf_set(a, CDPHY_RX_ANA_64, BIT(8));
			seninf_set(b, CDPHY_RX_ANA_64, BIT(0));
			break;
		case 1:
			dphy_lanes(top, DPHY_RX_LC0_EN | DPHY_RX_LD0_EN, sel1);
			seninf_set(a, CDPHY_RX_ANA_64, BIT(0));
			break;
		default:
			goto err;
		}
	} else if (port_is_a(ctx->port)) {
		static const u8 sel2[6] = { 1, 0, 0, 2, 0, 0 };
		static const u8 sel1[6] = { 1, 0, 0, 0, 0, 0 };

		if (lanes == 2) {
			dphy_lanes(top, DPHY_RX_LC0_EN | DPHY_RX_LD0_EN |
				   DPHY_RX_LD1_EN, sel2);
			seninf_set(a, CDPHY_RX_ANA_64, BIT(0));
			seninf_set(a, CDPHY_RX_ANA_64, BIT(16));
		} else if (lanes == 1) {
			dphy_lanes(top, DPHY_RX_LC0_EN | DPHY_RX_LD0_EN, sel1);
			seninf_set(a, CDPHY_RX_ANA_64, BIT(0));
		} else {
			goto err;
		}
		csirx_phy_a_bias(a);
		seninf_set(a, CDPHY_RX_ANA_SETTING_1, BIT(3));
	} else {
		static const u8 sel1[6] = { 0, 4, 0, 0, 3, 0 };

		if (lanes != 1)
			goto err;
		dphy_lanes(top, DPHY_RX_LC1_EN | DPHY_RX_LD2_EN, sel1);
		seninf_set(b, CDPHY_RX_ANA_64, BIT(0));
		csirx_phy_a_bias(b);
		seninf_set(b, CDPHY_RX_ANA_SETTING_1, BIT(3));
	}

	return 0;

err:
	dev_err(ctx->dev, "port %u can't take %u data lanes\n", ctx->port,
		lanes);
	return -EINVAL;
}

static int csirx_dphy_setting(struct seninf_ctx *ctx)
{
	void __iomem *a = ctx->reg_ana_csi_rx[ctx->port_a];
	void __iomem *b = ctx->reg_ana_csi_rx[ctx->port_b];
	void __iomem *top = ctx->reg_ana_dphy_top[ctx->port];
	u32 lanes = ctx->num_data_lanes;
	u64 rate = dphy_lane_rate(ctx);
	u32 rx_en = readl(top + DPHY_RX_HS_RX_EN_SW);
	u32 lpf;
	int i, ret;

	if (rate >= 4500000000ULL) {
		dev_err(ctx->dev, "lane rate %llu bps not supported\n", rate);
		return -EINVAL;
	}

	seninf_rmw(a, CDPHY_RX_ANA_SETTING_1, GENMASK(7, 4),
		   (rate >= 4000000000ULL || (rx_en & DPHY_RX_HS_RX_EN_SW_MASK)) ?
		   0 : 4);
	seninf_rmw(a, CDPHY_RX_ANA_SETTING_1, GENMASK(23, 20),
		   lanes < 2 ? 5 : 3);
	seninf_rmw(b, CDPHY_RX_ANA_SETTING_1, GENMASK(23, 20),
		   lanes < 2 ? 5 : 3);
	seninf_set(a, CDPHY_RX_ANA_SETTING_1, BIT(2));
	seninf_rmw(a, CDPHY_RX_ANA_SETTING_1, GENMASK(19, 16), 4);
	seninf_rmw(a, CDPHY_RX_ANA_SETTING_1, GENMASK(29, 24), 4);
	seninf_rmw(a, CDPHY_RX_ANA_SETTING_0, GENMASK(7, 4), 2);

	if (ctx->is_4d1c) {
		csirx_phy_a_dphy_4d1c(a, b, rate);
		csirx_phy_a_power(a, true);
		csirx_phy_a_power(b, true);
	} else {
		csirx_phy_a_dphy_half(ctx->reg_ana_csi_rx[ctx->port], rate);
		csirx_phy_a_power(ctx->reg_ana_csi_rx[ctx->port], true);
	}

	lpf = (ctx->is_4d1c && lanes >= 2) ? 3 : 5;
	seninf_rmw(a, CDPHY_RX_ANA_SETTING_1, GENMASK(23, 20), lpf);
	seninf_rmw(b, CDPHY_RX_ANA_SETTING_1, GENMASK(23, 20), lpf);
	seninf_set_each(top, DPHY_RX_204, GENMASK(3, 0), true);
	seninf_set(a, CDPHY_RX_ANA_8, BIT(0));

	ret = csirx_dphy_lane_setting(ctx);
	if (ret)
		return ret;

	seninf_set(top, DPHY_RX_LANE_SELECT, DPHY_RX_LANE_SEL_UPDATE);
	seninf_clr(top, DPHY_RX_HS_RX_EN_SW, DPHY_RX_HS_RX_EN_SW_MASK);
	seninf_set(top, DPHY_RX_180, BIT(3));
	writel(0xf1, top + DPHY_RX_SPARE0);

	/* no deskew calibration */
	for (i = 0; i < 4; i++)
		seninf_clr(top, DPHY_RX_DESKEW_LANE_CTRL(i),
			   RG_DPHY_RX_DESKEW_EN);

	writel(0xf, top + DPHY_RX_IRQ_EN);

	return 0;
}

int mtk_cam_seninf_set_csi_mipi(struct seninf_ctx *ctx)
{
	void __iomem *seninf;
	int ret;

	if (ctx->is_cphy) {
		dev_err(ctx->dev, "C-PHY is not supported\n");
		return -EOPNOTSUPP;
	}

	csirx_phy_a_init(ctx->reg_ana_csi_rx[ctx->port]);
	if (ctx->is_4d1c)
		csirx_phy_a_init(ctx->reg_ana_csi_rx[ctx->port_b]);
	csirx_apply_efuse(ctx);
	csirx_dphy_timing(ctx);
	seninf_set(ctx->reg_ana_cphy_top[ctx->port], CPHY_RX_DETECT_CTRL_POST,
		   BIT(4));
	csirx_mac_top_setting(ctx);

	ret = csirx_mac_csi_setting(ctx);
	if (ret)
		return ret;

	seninf = ctx->reg_if_ctrl[ctx->seninf_idx];
	seninf_top_ctrl2(ctx, true);
	seninf_set(seninf, SENINF_CSI2_CTRL, SENINF_CSI2_EN);
	seninf_set(seninf, SENINF_CTRL, SENINF_EN);

	return csirx_dphy_setting(ctx);
}

int mtk_cam_seninf_poweroff(struct seninf_ctx *ctx)
{
	writel(0, ctx->reg_if_csi2[ctx->seninf_idx] + SENINF_CSI2_RX_EN);

	if (ctx->is_4d1c) {
		csirx_phy_a_power(ctx->reg_ana_csi_rx[ctx->port_a], false);
		csirx_phy_a_power(ctx->reg_ana_csi_rx[ctx->port_b], false);
	} else {
		csirx_phy_a_power(ctx->reg_ana_csi_rx[ctx->port], false);
	}

	return 0;
}

int mtk_cam_seninf_reset(struct seninf_ctx *ctx, u32 seninf_idx)
{
	void __iomem *top = ctx->reg_ana_mac_top[ctx->port];
	void __iomem *seninf;
	int i;

	if (seninf_idx >= SENINF_NUM)
		return -EINVAL;

	seninf = ctx->reg_if_ctrl[seninf_idx];
	seninf_set(seninf, SENINF_CSI2_CTRL, SENINF_CSI2_SW_RST);
	udelay(1);
	seninf_clr(seninf, SENINF_CSI2_CTRL, SENINF_CSI2_SW_RST);

	seninf_set(top, CSIRX_MAC_TOP_CTRL, CSIRX_MAC_TOP_RST);
	udelay(1);
	seninf_clr(top, CSIRX_MAC_TOP_CTRL, CSIRX_MAC_TOP_RST);

	for (i = 0; i < SENINF_HW_MUX_NUM; i++) {
		void __iomem *mux = ctx->reg_if_mux[i];

		if (mtk_cam_seninf_get_top_mux_ctrl(ctx, i) != seninf_idx ||
		    !mtk_cam_seninf_is_mux_used(ctx, i))
			continue;

		seninf_set(mux, SENINF_MUX_CTRL_0, SENINF_MUX_SW_RST);
		udelay(1);
		seninf_clr(mux, SENINF_MUX_CTRL_0, SENINF_MUX_SW_RST);
	}

	return 0;
}

int mtk_cam_seninf_set_idle(struct seninf_ctx *ctx)
{
	struct seninf_vcinfo *vcinfo = &ctx->vcinfo;
	int i;

	for (i = 0; i < vcinfo->cnt; i++) {
		struct seninf_vc *vc = &vcinfo->vc[i];

		if (!vc->enable)
			continue;

		mtk_cam_seninf_disable_mux(ctx, vc->mux);
		mtk_cam_seninf_disable_cammux(ctx, vc->cam);
		ctx->pad2cam[vc->out_pad] = 0xff;
	}

	return 0;
}

int mtk_cam_seninf_switch_to_cammux_inner_page(struct seninf_ctx *ctx,
					       bool inner)
{
	seninf_rmw(ctx->reg_if_cam_mux_gcsr, CAM_MUX_GCSR_CTRL,
		   CAM_MUX_GCSR_PAGE_SEL, !inner);

	return 0;
}

int mtk_cam_seninf_set_cammux_next_ctrl(struct seninf_ctx *ctx, int src,
					int target)
{
	int vr;

	if (!cam_mux_valid(target))
		return -EINVAL;

	/* anything but a valid mux disconnects the cam mux */
	vr = mux_to_mux_vr(src);
	if (vr < 0)
		vr = CAM_MUX_SRC_NONE;

	seninf_rmw(ctx->reg_if_cam_mux_pcsr[target], CAM_MUX_PCSR_CTRL,
		   CAM_MUX_PCSR_NEXT_SRC_SEL, vr);

	return 0;
}

int mtk_cam_seninf_irq_handler(int irq, void *data)
{
	struct seninf_core *core = data;
	void __iomem *gcsr = core->reg_if + IF_CAM_MUX_GCSR;
	int i;

	for (i = 0; i < 2; i++) {
		u32 sts = readl(gcsr + CAM_MUX_GCSR_VSYNC_IRQ_STS(i));

		if (sts)
			writel(0xffffffff, gcsr + CAM_MUX_GCSR_VSYNC_IRQ_STS(i));
	}

	return 0;
}

int mtk_cam_seninf_set_sw_cfg_busy(struct seninf_ctx *ctx, bool enable,
				   int index)
{
	seninf_rmw(ctx->reg_if_cam_mux_gcsr, CAM_MUX_GCSR_CTRL,
		   CAM_MUX_GCSR_SW_CFG_BUSY, enable);

	return 0;
}

int mtk_cam_seninf_reset_cam_mux_dyn_en(struct seninf_ctx *ctx, int index)
{
	void __iomem *gcsr = ctx->reg_if_cam_mux_gcsr;

	seninf_clr(gcsr, CAM_MUX_GCSR_CTRL, CAM_MUX_GCSR_DYN_PAGE);
	writel(0, gcsr + CAM_MUX_GCSR_DYN_EN(!!index));

	return 0;
}

int mtk_cam_seninf_enable_global_drop_irq(struct seninf_ctx *ctx, bool enable,
					  int index)
{
	seninf_rmw(ctx->reg_if_cam_mux_gcsr, CAM_MUX_GCSR_IRQ_EN,
		   CAM_MUX_GCSR_DROP_IRQ_EN, enable ? 3 : 0);

	return 0;
}

int mtk_cam_seninf_enable_cam_mux_vsync_irq(struct seninf_ctx *ctx, bool enable,
					    int cam_mux)
{
	void __iomem *gcsr = ctx->reg_if_cam_mux_gcsr;
	u32 reg;

	if (cam_mux < 0 || cam_mux >= 64)
		return 0;

	reg = CAM_MUX_GCSR_VSYNC_IRQ_EN(cam_mux / 32);
	if (enable)
		seninf_set(gcsr, reg, BIT(cam_mux % 32));
	else
		seninf_clr(gcsr, reg, BIT(cam_mux % 32));

	return 0;
}

int mtk_cam_seninf_disable_all_cam_mux_vsync_irq(struct seninf_ctx *ctx)
{
	writel(0, ctx->reg_if_cam_mux_gcsr + CAM_MUX_GCSR_VSYNC_IRQ_EN(0));
	writel(0, ctx->reg_if_cam_mux_gcsr + CAM_MUX_GCSR_VSYNC_IRQ_EN(1));

	return 0;
}

int mtk_cam_seninf_set_reg(struct seninf_ctx *ctx, u32 key, u32 val)
{
	return -EOPNOTSUPP;
}
