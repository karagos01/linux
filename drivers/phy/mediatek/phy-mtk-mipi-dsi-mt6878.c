// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2026 MediaTek Inc.
 * Copyright (c) 2026 Jan Benedikt <karagos01@gmail.com>
 *
 * MIPI D-PHY (MIPI_TX) for the MT6878 (Dimensity 7300) "N4" PHY family,
 * shared with the MT6983/MT6985/MT6989 generation.  Register sequences
 * were reverse-engineered from the vendor mtk_mipi_tx driver.
 */

#include "phy-mtk-io.h"
#include "phy-mtk-mipi-dsi.h"

#define MIPITX_LANE_CON			0x0004
#define MIPITX_VOLTAGE_SEL		0x0008
#define RG_DSI_PRD_REF_SEL			GENMASK(5, 0)
#define MIPITX_PRESERVED		0x000c

#define MIPITX_PLL_PWR			0x0028
#define AD_DSI_PLL_SDM_PWR_ON			BIT(0)
#define AD_DSI_PLL_SDM_ISO_EN			BIT(1)

#define MIPITX_PLL_CON0			0x002c
#define MIPITX_PLL_CON1			0x0030
#define RG_DSI_PLL_EN				BIT(0)
#define RG_DSI_PLL_POSDIV			GENMASK(10, 8)
#define RG_DSI_PLL_FBSEL			BIT(13)
#define RG_DSI_PLL_DIV3_EN			BIT(28)
#define MIPITX_PLL_CON4			0x003c

#define MIPITX_SW_CTRL_CON4		0x0050
#define SW_ANA_CK_EN				BIT(0)

#define MIPITX_CK_CKMODE_EN		0x0320
#define DSI_CK_CKMODE_EN			BIT(0)

/*
 * PCW encoding (N4): integer part in bits 24..30, the remaining bits are
 * the successive 1/256 fractions of the fractional part.  div3 and fbksel
 * are always 1 for this family.
 */
static u32 mtk_mipi_tx_pll_get_pcw(u32 data_rate_mhz, unsigned int pcw_ratio)
{
	u32 pcw, pcw_floor;

	data_rate_mhz >>= 1;

	pcw = data_rate_mhz * pcw_ratio / 26;
	pcw_floor = data_rate_mhz * pcw_ratio % 26;

	return ((pcw & 0xff) << 24) |
	       (((256 * pcw_floor / 26) & 0xff) << 16) |
	       (((256 * (256 * pcw_floor % 26) / 26) & 0xff) << 8) |
	       ((256 * (256 * (256 * pcw_floor % 26) % 26) / 26) & 0xff);
}

/*
 * The PLL power-up sequence sleeps (usleep_range for BG settle / SDM power-on /
 * PLL lock), so it must run as a clk .prepare op (prepare_lock mutex), NOT
 * .enable (which the CCF calls under the enable_lock spinlock -> "scheduling
 * while atomic"). clk_prepare_enable() from mtk_mipi_tx_power_on() invokes it.
 */
static int mtk_mipi_tx_pll_prepare(struct clk_hw *hw)
{
	struct mtk_mipi_tx *mipi_tx = mtk_mipi_tx_from_clk_hw(hw);
	void __iomem *base = mipi_tx->regs;
	unsigned int txdiv, txdiv0;
	u32 rate = mipi_tx->data_rate / 1000000;
	u32 pcw;

	dev_dbg(mipi_tx->dev, "prepare: %u MHz\n", rate);

	if (rate >= 2000) {
		txdiv = 1;
		txdiv0 = 0;
	} else if (rate >= 1000) {
		txdiv = 2;
		txdiv0 = 1;
	} else if (rate >= 500) {
		txdiv = 4;
		txdiv0 = 2;
	} else if (rate >= 250) {
		txdiv = 8;
		txdiv0 = 3;
	} else if (rate >= 125) {
		txdiv = 16;
		txdiv0 = 4;
	} else {
		return -EINVAL;
	}

	mtk_phy_update_field(base + MIPITX_VOLTAGE_SEL, RG_DSI_PRD_REF_SEL,
			     rate < 2500 ? 0x0 : 0x4);

	writel(rate < 2500 ? 0xffff00f0 : 0xffff0030, base + MIPITX_PRESERVED);
	writel(0x00ff12e0, base + MIPITX_PLL_CON4);

	/* BG_LPF_EN / BG_CORE_EN */
	writel(0x3fff0180, base + MIPITX_LANE_CON);
	usleep_range(500, 600);
	writel(0x3fff00c0, base + MIPITX_LANE_CON);

	/* SDM power on, release isolation */
	mtk_phy_set_bits(base + MIPITX_PLL_PWR, AD_DSI_PLL_SDM_PWR_ON);
	usleep_range(30, 100);
	mtk_phy_clear_bits(base + MIPITX_PLL_PWR, AD_DSI_PLL_SDM_ISO_EN);

	/* FBSEL = 0 */
	mtk_phy_clear_bits(base + MIPITX_PLL_CON1, RG_DSI_PLL_FBSEL);

	pcw = mtk_mipi_tx_pll_get_pcw(rate, txdiv);
	writel(pcw, base + MIPITX_PLL_CON0);

	mtk_phy_update_field(base + MIPITX_PLL_CON1, RG_DSI_PLL_POSDIV, txdiv0);
	mtk_phy_clear_bits(base + MIPITX_PLL_CON1, RG_DSI_PLL_DIV3_EN);
	mtk_phy_set_bits(base + MIPITX_PLL_CON1, RG_DSI_PLL_EN);

	usleep_range(50, 100);

	return 0;
}

static void mtk_mipi_tx_pll_unprepare(struct clk_hw *hw)
{
	struct mtk_mipi_tx *mipi_tx = mtk_mipi_tx_from_clk_hw(hw);
	void __iomem *base = mipi_tx->regs;

	mtk_phy_clear_bits(base + MIPITX_PLL_CON1, RG_DSI_PLL_EN);
	mtk_phy_clear_bits(base + MIPITX_SW_CTRL_CON4, SW_ANA_CK_EN);

	mtk_phy_set_bits(base + MIPITX_PLL_PWR, AD_DSI_PLL_SDM_ISO_EN);
	mtk_phy_clear_bits(base + MIPITX_PLL_PWR, AD_DSI_PLL_SDM_PWR_ON);

	/* BG off */
	writel(0x3fff0180, base + MIPITX_LANE_CON);
	writel(0x3fff0100, base + MIPITX_LANE_CON);
}

static int mtk_mipi_tx_pll_determine_rate(struct clk_hw *hw,
					  struct clk_rate_request *req)
{
	req->rate = clamp_val(req->rate, 125000000, 4500000000ULL);

	return 0;
}

static const struct clk_ops mtk_mipi_tx_pll_ops = {
	.prepare = mtk_mipi_tx_pll_prepare,
	.unprepare = mtk_mipi_tx_pll_unprepare,
	.determine_rate = mtk_mipi_tx_pll_determine_rate,
	.set_rate = mtk_mipi_tx_pll_set_rate,
	.recalc_rate = mtk_mipi_tx_pll_recalc_rate,
};

static void mtk_mipi_tx_power_on_signal(struct phy *phy)
{
	struct mtk_mipi_tx *mipi_tx = phy_get_drvdata(phy);

	/* Put the clock lane into clock mode; lane mapping is left at the
	 * default (0..3, CK) programmed by the bootloader.
	 */
	mtk_phy_set_bits(mipi_tx->regs + MIPITX_CK_CKMODE_EN, DSI_CK_CKMODE_EN);
}

static void mtk_mipi_tx_power_off_signal(struct phy *phy)
{
}

const struct mtk_mipitx_data mt6878_mipitx_data = {
	.mipi_tx_clk_ops = &mtk_mipi_tx_pll_ops,
	.mipi_tx_enable_signal = mtk_mipi_tx_power_on_signal,
	.mipi_tx_disable_signal = mtk_mipi_tx_power_off_signal,
};
