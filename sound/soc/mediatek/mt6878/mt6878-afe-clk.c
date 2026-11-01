// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek MT6878 AFE clock control
 *
 * Copyright (c) 2026 Jan Benedikt <karagos01@gmail.com>
 *
 * The clock bring-up follows the downstream vendor module's mt6878_init_clock()
 * and its runtime-PM clock enable/disable helpers, reconstructed from the
 * reverse-engineered snd-soc-mt6878-afe.ko:
 *
 *   - All 34 named clocks (aud_* gates, top_* muxes/dividers, *_ck_peri bus
 *     clocks) are fetched from device tree.
 *   - The apmixedsys, topckgen and infracfg syscons are looked up by phandle
 *     so the audio APLL control words can be programmed at init time.
 *   - The always-on peripheral bus clocks are enabled once; the APLL-fed muxes
 *     are parked on the 26 MHz crystal (the IPM2.0 "hopping & 26M" scheme).
 *   - runtime resume/suspend toggle the audio-interface and audio-H bus muxes
 *     and the hopping/26M clocks.
 *
 * The aud_* gates are provided by the audiosys clock controller, which shares
 * the AFE register region (see the "mediatek,mt6878-audiosys" syscon in the
 * binding); that controller is a separate driver and must be present for this
 * AFE to bind.
 */

#include <linux/arm-smccc.h>
#include <linux/clk.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

#include "mt6878-afe-clk.h"
#include "mt6878-afe-common.h"

/*
 * MediaTek audio secure-monitor call. The stock driver issues this whenever
 * the AFE clocks are turned on (sub-command 7); it hands the audio clock
 * state to the secure world.
 */
#define MT6878_AUDIO_SMC_ID		0xc2000517
#define MT6878_AUDIO_SMC_OP_CLK_ON	7

static const char *const aud_clks[MT6878_CLK_NUM] = {
	[MT6878_CLK_AUD_HOPPING]		= "aud_hopping_clk",
	[MT6878_CLK_AUD_F26M]			= "aud_f26m_clk",
	[MT6878_CLK_AUD_DL0_DAC]		= "aud_dl0_dac_clk",
	[MT6878_CLK_AUD_DL0_DAC_HIRES]		= "aud_dl0_dac_hires_clk",
	[MT6878_CLK_AUD_DL0_DAC_PREDIS]		= "aud_dl0_dac_predis_clk",
	[MT6878_CLK_AUD_UL0_ADC]		= "aud_ul0_adc_clk",
	[MT6878_CLK_AUD_UL0_ADC_HIRES]		= "aud_ul0_adc_hires_clk",
	[MT6878_CLK_AUD_UL1_ADC]		= "aud_ul1_adc_clk",
	[MT6878_CLK_AUD_UL1_ADC_HIRES]		= "aud_ul1_adc_hires_clk",
	[MT6878_CLK_AUD_APLL1]			= "aud_apll1_clk",
	[MT6878_CLK_AUD_APLL2]			= "aud_apll2_clk",
	[MT6878_CLK_AUD_APLL_TUNER1]		= "aud_apll_tuner1_clk",
	[MT6878_CLK_AUD_APLL_TUNER2]		= "aud_apll_tuner2_clk",
	[MT6878_CLK_TOP_MUX_AUDIO_INT]		= "top_mux_audio_int",
	[MT6878_CLK_TOP_MAINPLL_D4_D4]		= "top_mainpll_d4_d4",
	[MT6878_CLK_TOP_MUX_AUD_1]		= "top_mux_aud_1",
	[MT6878_CLK_TOP_APLL1_CK]		= "top_apll1_ck",
	[MT6878_CLK_TOP_MUX_AUD_2]		= "top_mux_aud_2",
	[MT6878_CLK_TOP_APLL2_CK]		= "top_apll2_ck",
	[MT6878_CLK_TOP_MUX_AUD_ENG1]		= "top_mux_aud_eng1",
	[MT6878_CLK_TOP_APLL1_D4]		= "top_apll1_d4",
	[MT6878_CLK_TOP_MUX_AUD_ENG2]		= "top_mux_aud_eng2",
	[MT6878_CLK_TOP_APLL2_D4]		= "top_apll2_d4",
	[MT6878_CLK_TOP_MUX_AUDIO_H]		= "top_mux_audio_h",
	[MT6878_CLK_TOP_APLL1_D2]		= "top_apll1_d2",
	[MT6878_CLK_TOP_APLL2_D2]		= "top_apll2_d2",
	[MT6878_CLK_TOP_I2SIN1_M_SEL]		= "top_i2sin1_m_sel",
	[MT6878_CLK_TOP_FMI2S_M_SEL]		= "top_fmi2s_m_sel",
	[MT6878_CLK_TOP_APLL12_DIV_I2SIN1]	= "top_apll12_div_i2sin1",
	[MT6878_CLK_TOP_APLL12_DIV_FMI2S]	= "top_apll12_div_fmi2s",
	[MT6878_CLK_TOP_CLK26M]			= "top_clk26m_clk",
	[MT6878_CLK_AUD_SLV_CK_PERI]		= "aud_slv_ck_peri",
	[MT6878_CLK_AUD_MST_CK_PERI]		= "aud_mst_ck_peri",
	[MT6878_CLK_AUD_INTBUS_CK_PERI]		= "aud_intbus_ck_peri",
};

static void mt6878_set_parent(struct mtk_base_afe *afe, int clk_id,
			      int parent_id)
{
	struct mt6878_afe_private *afe_priv = afe->platform_priv;
	int ret;

	ret = clk_set_parent(afe_priv->clk[clk_id], afe_priv->clk[parent_id]);
	if (ret)
		dev_info(afe->dev, "clk_set_parent %s-%s failed %d\n",
			 aud_clks[clk_id], aud_clks[parent_id], ret);
}

/* Enable the always-on peripheral bus clocks (kept on for the device life). */
static int mt6878_afe_enable_ao_clock(struct mtk_base_afe *afe)
{
	struct mt6878_afe_private *afe_priv = afe->platform_priv;
	int ret;

	ret = clk_prepare_enable(afe_priv->clk[MT6878_CLK_AUD_INTBUS_CK_PERI]);
	if (ret)
		goto err_intbus;

	ret = clk_prepare_enable(afe_priv->clk[MT6878_CLK_AUD_SLV_CK_PERI]);
	if (ret)
		goto err_slv;

	ret = clk_prepare_enable(afe_priv->clk[MT6878_CLK_AUD_MST_CK_PERI]);
	if (ret)
		goto err_mst;

	return 0;

err_mst:
	clk_disable_unprepare(afe_priv->clk[MT6878_CLK_AUD_SLV_CK_PERI]);
err_slv:
	clk_disable_unprepare(afe_priv->clk[MT6878_CLK_AUD_INTBUS_CK_PERI]);
err_intbus:
	dev_info(afe->dev, "failed to enable always-on peri clocks %d\n", ret);
	return ret;
}

/*
 * Park the APLL-fed audio muxes (aud_1, aud_2, audio_h) on the 26 MHz crystal.
 * Done once at init so the audio tree starts from a known, low-power state.
 */
static int mt6878_afe_disable_apll(struct mtk_base_afe *afe)
{
	struct mt6878_afe_private *afe_priv = afe->platform_priv;
	int ret;

	ret = clk_prepare_enable(afe_priv->clk[MT6878_CLK_TOP_MUX_AUDIO_H]);
	if (ret)
		goto err_audio_h;

	ret = clk_prepare_enable(afe_priv->clk[MT6878_CLK_TOP_MUX_AUD_1]);
	if (ret)
		goto err_aud_1;
	mt6878_set_parent(afe, MT6878_CLK_TOP_MUX_AUD_1, MT6878_CLK_TOP_CLK26M);

	ret = clk_prepare_enable(afe_priv->clk[MT6878_CLK_TOP_MUX_AUD_2]);
	if (ret)
		goto err_aud_2;
	mt6878_set_parent(afe, MT6878_CLK_TOP_MUX_AUD_2, MT6878_CLK_TOP_CLK26M);

	clk_disable_unprepare(afe_priv->clk[MT6878_CLK_TOP_MUX_AUD_1]);
	clk_disable_unprepare(afe_priv->clk[MT6878_CLK_TOP_MUX_AUD_2]);
	mt6878_set_parent(afe, MT6878_CLK_TOP_MUX_AUDIO_H, MT6878_CLK_TOP_CLK26M);
	clk_disable_unprepare(afe_priv->clk[MT6878_CLK_TOP_MUX_AUDIO_H]);

	return 0;

err_aud_2:
	clk_disable_unprepare(afe_priv->clk[MT6878_CLK_TOP_MUX_AUD_1]);
err_aud_1:
	clk_disable_unprepare(afe_priv->clk[MT6878_CLK_TOP_MUX_AUDIO_H]);
err_audio_h:
	dev_info(afe->dev, "failed to park APLL muxes on 26M %d\n", ret);
	return ret;
}

int mt6878_afe_enable_clock(struct mtk_base_afe *afe)
{
	struct mt6878_afe_private *afe_priv = afe->platform_priv;
	struct arm_smccc_res res;
	int ret;

	ret = clk_prepare_enable(afe_priv->clk[MT6878_CLK_AUD_HOPPING]);
	if (ret)
		goto err_hopping;

	ret = clk_prepare_enable(afe_priv->clk[MT6878_CLK_AUD_F26M]);
	if (ret)
		goto err_f26m;

	ret = clk_prepare_enable(afe_priv->clk[MT6878_CLK_TOP_MUX_AUDIO_INT]);
	if (ret)
		goto err_audio_int;
	mt6878_set_parent(afe, MT6878_CLK_TOP_MUX_AUDIO_INT,
			  MT6878_CLK_TOP_CLK26M);

	ret = clk_prepare_enable(afe_priv->clk[MT6878_CLK_TOP_MUX_AUDIO_H]);
	if (ret)
		goto err_audio_h;
	mt6878_set_parent(afe, MT6878_CLK_TOP_MUX_AUDIO_H, MT6878_CLK_TOP_CLK26M);

	arm_smccc_smc(MT6878_AUDIO_SMC_ID, MT6878_AUDIO_SMC_OP_CLK_ON,
		      0, 0, 0, 0, 0, 0, &res);

	return 0;

err_audio_h:
	clk_disable_unprepare(afe_priv->clk[MT6878_CLK_TOP_MUX_AUDIO_INT]);
err_audio_int:
	clk_disable_unprepare(afe_priv->clk[MT6878_CLK_AUD_F26M]);
err_f26m:
	clk_disable_unprepare(afe_priv->clk[MT6878_CLK_AUD_HOPPING]);
err_hopping:
	dev_info(afe->dev, "failed to enable AFE clocks %d\n", ret);
	return ret;
}

void mt6878_afe_disable_clock(struct mtk_base_afe *afe)
{
	struct mt6878_afe_private *afe_priv = afe->platform_priv;

	mt6878_set_parent(afe, MT6878_CLK_TOP_MUX_AUDIO_INT,
			  MT6878_CLK_TOP_CLK26M);
	clk_disable_unprepare(afe_priv->clk[MT6878_CLK_TOP_MUX_AUDIO_INT]);

	mt6878_set_parent(afe, MT6878_CLK_TOP_MUX_AUDIO_H, MT6878_CLK_TOP_CLK26M);
	clk_disable_unprepare(afe_priv->clk[MT6878_CLK_TOP_MUX_AUDIO_H]);

	clk_disable_unprepare(afe_priv->clk[MT6878_CLK_AUD_HOPPING]);
	clk_disable_unprepare(afe_priv->clk[MT6878_CLK_AUD_F26M]);
}

int mt6878_init_clock(struct mtk_base_afe *afe)
{
	struct mt6878_afe_private *afe_priv = afe->platform_priv;
	struct device_node *np = afe->dev->of_node;
	int i, ret;

	afe_priv->clk = devm_kcalloc(afe->dev, MT6878_CLK_NUM,
				     sizeof(*afe_priv->clk), GFP_KERNEL);
	if (!afe_priv->clk)
		return -ENOMEM;

	for (i = 0; i < MT6878_CLK_NUM; i++) {
		/*
		 * The I2S/FM master-clock muxes (i2sin1_m_sel, fmi2s_m_sel) and
		 * their dividers (apll12_div_i2sin1, apll12_div_fmi2s) are only
		 * needed for the external I2S/ETDM paths and are not provided by
		 * the mainline topckgen driver; treat them as optional until
		 * those DAIs are added.  The internal ADDA downlink does not use
		 * them.
		 */
		if (i == MT6878_CLK_TOP_I2SIN1_M_SEL ||
		    i == MT6878_CLK_TOP_FMI2S_M_SEL ||
		    i == MT6878_CLK_TOP_APLL12_DIV_I2SIN1 ||
		    i == MT6878_CLK_TOP_APLL12_DIV_FMI2S)
			afe_priv->clk[i] = devm_clk_get_optional(afe->dev,
								 aud_clks[i]);
		else
			afe_priv->clk[i] = devm_clk_get(afe->dev, aud_clks[i]);
		if (IS_ERR(afe_priv->clk[i]))
			return dev_err_probe(afe->dev, PTR_ERR(afe_priv->clk[i]),
					     "devm_clk_get %s failed\n",
					     aud_clks[i]);
	}

	afe_priv->topckgen = syscon_regmap_lookup_by_phandle(np,
							     "mediatek,topckgen");
	if (IS_ERR(afe_priv->topckgen))
		return dev_err_probe(afe->dev, PTR_ERR(afe_priv->topckgen),
				     "cannot find topckgen controller\n");

	afe_priv->apmixedsys = syscon_regmap_lookup_by_phandle(np,
							       "mediatek,apmixedsys");
	if (IS_ERR(afe_priv->apmixedsys))
		return dev_err_probe(afe->dev, PTR_ERR(afe_priv->apmixedsys),
				     "cannot find apmixedsys controller\n");

	afe_priv->infracfg = syscon_regmap_lookup_by_phandle(np,
							     "mediatek,infracfg");
	if (IS_ERR(afe_priv->infracfg))
		return dev_err_probe(afe->dev, PTR_ERR(afe_priv->infracfg),
				     "cannot find infracfg controller\n");

	/* program the audio APLL control words (stock init values) */
	regmap_write(afe_priv->apmixedsys, MT6878_APLL1_CON_REG,
		     MT6878_APLL1_CON_VAL);
	regmap_write(afe_priv->apmixedsys, MT6878_APLL2_CON_REG,
		     MT6878_APLL2_CON_VAL);

	ret = mt6878_afe_disable_apll(afe);
	if (ret)
		return ret;

	ret = mt6878_afe_enable_ao_clock(afe);
	if (ret)
		return ret;

	return 0;
}
