// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek MT6878 ALSA SoC AFE -- ETDM_OUT4 / I2S4 DAI (external speaker I2S)
 *
 * Copyright (c) 2026 Jan Benedikt <karagos01@gmail.com>
 *
 * ETDM_OUT4 is the SoC-side I2S output that feeds the external speaker
 * smart-amp. This driver configures ETDM_OUT4 as an I2S master (BCLK/LRCK
 * driven from the APLL2-fed i2sin1 master-clock divider) and routes a downlink
 * memif (DL1) into it through the connection matrix. The rt5512 codec and the
 * aw87xxx boost amps on the far side of the bus are brought up separately.
 *
 * Register values and the enable sequence are reverse engineered from the
 * MT8196-generation sibling (sound/soc/mediatek/mt8196/mt8196-dai-i2s.c,
 * ETDM_OUT4 path) and cross-checked against the stock snd-soc-mt6878-afe.ko.
 */

#include <linux/clk.h>
#include <linux/regmap.h>

#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "mt6878-afe-common.h"
#include "mt6878-afe-clk.h"
#include "mt6878-afe-gpio.h"

/* ETDM_OUT4 master clock: 256 * 48 kHz, divided down from APLL2 (49.152 MHz). */
#define MT6878_I2SOUT4_MCLK_RATE	12288000

enum {
	SUPPLY_SEQ_I2S_CLK,
	SUPPLY_SEQ_I2S_EN,
};

/*
 * Enable the ETDM_OUT4 master clock: park the i2sin1 master-clock mux on the
 * APLL2 path (48 kHz family) and run the i2sin1 divider at the target MCLK.
 * The ETDM block divides this down to BCLK internally from its CON registers.
 */
static int mt6878_i2sout4_clk_enable(struct mtk_base_afe *afe)
{
	struct mt6878_afe_private *afe_priv = afe->platform_priv;
	int ret;

	ret = clk_prepare_enable(afe_priv->clk[MT6878_CLK_AUD_APLL2]);
	if (ret)
		goto err_apll2;
	ret = clk_prepare_enable(afe_priv->clk[MT6878_CLK_AUD_APLL_TUNER2]);
	if (ret)
		goto err_tuner2;

	ret = clk_prepare_enable(afe_priv->clk[MT6878_CLK_TOP_MUX_AUD_2]);
	if (ret)
		goto err_aud2;
	clk_set_parent(afe_priv->clk[MT6878_CLK_TOP_MUX_AUD_2],
		       afe_priv->clk[MT6878_CLK_TOP_APLL2_CK]);

	ret = clk_prepare_enable(afe_priv->clk[MT6878_CLK_TOP_I2SIN1_M_SEL]);
	if (ret)
		goto err_i2sin1_sel;
	clk_set_parent(afe_priv->clk[MT6878_CLK_TOP_I2SIN1_M_SEL],
		       afe_priv->clk[MT6878_CLK_TOP_MUX_AUD_2]);

	clk_set_rate(afe_priv->clk[MT6878_CLK_TOP_APLL12_DIV_I2SIN1],
		     MT6878_I2SOUT4_MCLK_RATE);
	ret = clk_prepare_enable(afe_priv->clk[MT6878_CLK_TOP_APLL12_DIV_I2SIN1]);
	if (ret)
		goto err_div;

	return 0;

err_div:
	clk_disable_unprepare(afe_priv->clk[MT6878_CLK_TOP_I2SIN1_M_SEL]);
err_i2sin1_sel:
	clk_disable_unprepare(afe_priv->clk[MT6878_CLK_TOP_MUX_AUD_2]);
err_aud2:
	clk_disable_unprepare(afe_priv->clk[MT6878_CLK_AUD_APLL_TUNER2]);
err_tuner2:
	clk_disable_unprepare(afe_priv->clk[MT6878_CLK_AUD_APLL2]);
err_apll2:
	dev_info(afe->dev, "%s() failed %d\n", __func__, ret);
	return ret;
}

static void mt6878_i2sout4_clk_disable(struct mtk_base_afe *afe)
{
	struct mt6878_afe_private *afe_priv = afe->platform_priv;

	clk_disable_unprepare(afe_priv->clk[MT6878_CLK_TOP_APLL12_DIV_I2SIN1]);
	clk_disable_unprepare(afe_priv->clk[MT6878_CLK_TOP_I2SIN1_M_SEL]);
	clk_disable_unprepare(afe_priv->clk[MT6878_CLK_TOP_MUX_AUD_2]);
	clk_disable_unprepare(afe_priv->clk[MT6878_CLK_AUD_APLL_TUNER2]);
	clk_disable_unprepare(afe_priv->clk[MT6878_CLK_AUD_APLL2]);
}

static int mt6878_i2sout4_clk_event(struct snd_soc_dapm_widget *w,
				    struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* ungate the ETDM_OUT4 (and shared ETDM_IN4) sub-block clocks */
		regmap_update_bits(afe->regmap, AUDIO_TOP_CON2,
				   AUDIO_TOP_CON2_PDN_ETDM_OUT4 |
				   AUDIO_TOP_CON2_PDN_ETDM_IN4, 0);
		mt6878_i2sout4_clk_enable(afe);
		/* mux the I2S4 pins to the audio function */
		mt6878_afe_gpio_request(afe, true, MT6878_DAI_I2S_OUT4, 0);
		break;
	case SND_SOC_DAPM_POST_PMD:
		mt6878_afe_gpio_request(afe, false, MT6878_DAI_I2S_OUT4, 0);
		mt6878_i2sout4_clk_disable(afe);
		break;
	default:
		break;
	}

	return 0;
}

/* connection-matrix mixers: route the DL1 memif into ETDM_OUT4 */
static const struct snd_kcontrol_new mtk_i2sout4_ch1_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("DL1_CH1", AFE_CONN_I2SOUT4_CH1,
				    I_DL1_CH1, 1, 0),
};

static const struct snd_kcontrol_new mtk_i2sout4_ch2_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("DL1_CH2", AFE_CONN_I2SOUT4_CH2,
				    I_DL1_CH2, 1, 0),
};

static const struct snd_soc_dapm_widget mtk_dai_i2sout4_widgets[] = {
	SND_SOC_DAPM_MIXER("I2SOUT4_CH1", SND_SOC_NOPM, 0, 0,
			   mtk_i2sout4_ch1_mix, ARRAY_SIZE(mtk_i2sout4_ch1_mix)),
	SND_SOC_DAPM_MIXER("I2SOUT4_CH2", SND_SOC_NOPM, 0, 0,
			   mtk_i2sout4_ch2_mix, ARRAY_SIZE(mtk_i2sout4_ch2_mix)),

	SND_SOC_DAPM_SUPPLY_S("I2SOUT4 CLK", SUPPLY_SEQ_I2S_CLK,
			      SND_SOC_NOPM, 0, 0, mt6878_i2sout4_clk_event,
			      SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_SUPPLY_S("I2SOUT4 Enable", SUPPLY_SEQ_I2S_EN,
			      ETDM_OUT4_CON0, ETDM_OUT_CON0_EN_SFT, 0,
			      NULL, 0),
};

static const struct snd_soc_dapm_route mtk_dai_i2sout4_routes[] = {
	{"I2SOUT4_CH1", "DL1_CH1", "DL1"},
	{"I2SOUT4_CH2", "DL1_CH2", "DL1"},

	{"I2SOUT4", NULL, "I2SOUT4_CH1"},
	{"I2SOUT4", NULL, "I2SOUT4_CH2"},

	{"I2SOUT4", NULL, "I2SOUT4 Enable"},
	{"I2SOUT4 Enable", NULL, "I2SOUT4 CLK"},
};

/* dai ops */
static int mtk_dai_i2sout4_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params,
				     struct snd_soc_dai *dai)
{
	struct mtk_base_afe *afe = snd_soc_dai_get_drvdata(dai);
	struct mt6878_afe_private *afe_priv = afe->platform_priv;
	struct mtk_afe_i2s_priv *i2s_priv = afe_priv->dai_priv[dai->id];
	unsigned int rate = params_rate(params);
	snd_pcm_format_t format = params_format(params);
	int width = snd_pcm_format_physical_width(format);
	unsigned int bit_length = width - 1;		/* valid bits - 1 */
	unsigned int word_length = (width <= 16) ? 0x0f : 0x1f;	/* slot bits */
	unsigned int ch_num = params_channels(params) - 1;

	if (i2s_priv)
		i2s_priv->rate = rate;

	dev_dbg(afe->dev, "%s(), rate %u, width %d, ch %u\n",
		__func__, rate, width, params_channels(params));

	/* CON1: initial count/point + LRCK reset (RE verbatim from mt8196) */
	regmap_update_bits(afe->regmap, ETDM_OUT4_CON1,
			   ETDM_OUT_CON1_INITIAL_COUNT_MASK <<
			   ETDM_OUT_CON1_INITIAL_COUNT_SFT,
			   0x5 << ETDM_OUT_CON1_INITIAL_COUNT_SFT);
	regmap_update_bits(afe->regmap, ETDM_OUT4_CON1,
			   ETDM_OUT_CON1_INITIAL_POINT_MASK <<
			   ETDM_OUT_CON1_INITIAL_POINT_SFT,
			   0x6 << ETDM_OUT_CON1_INITIAL_POINT_SFT);
	regmap_update_bits(afe->regmap, ETDM_OUT4_CON1,
			   BIT(ETDM_OUT_CON1_LRCK_RESET_SFT),
			   BIT(ETDM_OUT_CON1_LRCK_RESET_SFT));

	/* CON2: in-to-latch time */
	regmap_update_bits(afe->regmap, ETDM_OUT4_CON2,
			   ETDM_OUT_CON2_IN2LATCH_TIME_MASK <<
			   ETDM_OUT_CON2_IN2LATCH_TIME_SFT,
			   0x6 << ETDM_OUT_CON2_IN2LATCH_TIME_SFT);

	/* CON4: FS timing, clock source (APLL), relatch enable */
	regmap_update_bits(afe->regmap, ETDM_OUT4_CON4,
			   ETDM_OUT_CON4_FS_TIMING_MASK <<
			   ETDM_OUT_CON4_FS_TIMING_SFT,
			   ETDM_RATE_48K << ETDM_OUT_CON4_FS_TIMING_SFT);
	regmap_update_bits(afe->regmap, ETDM_OUT4_CON4,
			   ETDM_OUT_CON4_CLOCK_SOURCE_MASK <<
			   ETDM_OUT_CON4_CLOCK_SOURCE_SFT,
			   ETDM_CLK_SOURCE_APLL << ETDM_OUT_CON4_CLOCK_SOURCE_SFT);
	regmap_update_bits(afe->regmap, ETDM_OUT4_CON4,
			   ETDM_OUT_CON4_RELATCH_EN_MASK <<
			   ETDM_OUT_CON4_RELATCH_EN_SFT,
			   ETDM_CONN_48K << ETDM_OUT_CON4_RELATCH_EN_SFT);

	/* CON0: format (I2S), channels, sync mode, relatch domain, word sizes */
	regmap_update_bits(afe->regmap, ETDM_OUT4_CON0,
			   ETDM_OUT_CON0_FMT_MASK << ETDM_OUT_CON0_FMT_SFT,
			   0x0 << ETDM_OUT_CON0_FMT_SFT);
	regmap_update_bits(afe->regmap, ETDM_OUT4_CON0,
			   ETDM_OUT_CON0_CH_NUM_MASK << ETDM_OUT_CON0_CH_NUM_SFT,
			   ch_num << ETDM_OUT_CON0_CH_NUM_SFT);
	regmap_update_bits(afe->regmap, ETDM_OUT4_CON0,
			   BIT(ETDM_OUT_CON0_SYNC_MODE_SFT), 0);
	regmap_update_bits(afe->regmap, ETDM_OUT4_CON0,
			   ETDM_OUT_CON0_RELATCH_DOMAIN_MASK <<
			   ETDM_OUT_CON0_RELATCH_DOMAIN_SFT,
			   ETDM_RELATCH_SEL_APLL <<
			   ETDM_OUT_CON0_RELATCH_DOMAIN_SFT);
	regmap_update_bits(afe->regmap, ETDM_OUT4_CON0,
			   ETDM_OUT_CON0_BIT_LENGTH_MASK <<
			   ETDM_OUT_CON0_BIT_LENGTH_SFT,
			   bit_length << ETDM_OUT_CON0_BIT_LENGTH_SFT);
	regmap_update_bits(afe->regmap, ETDM_OUT4_CON0,
			   ETDM_OUT_CON0_WORD_LENGTH_MASK <<
			   ETDM_OUT_CON0_WORD_LENGTH_SFT,
			   word_length << ETDM_OUT_CON0_WORD_LENGTH_SFT);

	/* COWORK: ETDM_OUT4 is its own master (slave sel = 0) */
	regmap_update_bits(afe->regmap, ETDM_4_7_COWORK_CON0,
			   ETDM_OUT4_SLAVE_SEL_MASK << ETDM_OUT4_SLAVE_SEL_SFT,
			   0x0 << ETDM_OUT4_SLAVE_SEL_SFT);

	/* I2S4 pad-top: enable the pad clock, park the output BCK latch edge */
	regmap_update_bits(afe->regmap, AUD_TOP_CFG_VLP_RG,
			   BIT(RG_I2S4_PAD_TOP_CK_EN_SFT),
			   BIT(RG_I2S4_PAD_TOP_CK_EN_SFT));
	regmap_update_bits(afe->regmap, AUD_TOP_CFG_VLP_RG,
			   BIT(RG_I2S4_OUT_BCK_NEG_EG_LATCH_SFT), 0);

	return 0;
}

static const struct snd_soc_dai_ops mtk_dai_i2sout4_ops = {
	.hw_params = mtk_dai_i2sout4_hw_params,
};

#define MTK_I2SOUT4_RATES	(SNDRV_PCM_RATE_8000_48000 |\
				 SNDRV_PCM_RATE_88200 |\
				 SNDRV_PCM_RATE_96000 |\
				 SNDRV_PCM_RATE_176400 |\
				 SNDRV_PCM_RATE_192000)

#define MTK_I2SOUT4_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE |\
				 SNDRV_PCM_FMTBIT_S24_LE |\
				 SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver mtk_dai_i2sout4_driver[] = {
	{
		.name = "I2SOUT4",
		.id = MT6878_DAI_I2S_OUT4,
		.playback = {
			.stream_name = "I2SOUT4",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MTK_I2SOUT4_RATES,
			.formats = MTK_I2SOUT4_FORMATS,
		},
		.ops = &mtk_dai_i2sout4_ops,
	},
};

static int mt6878_dai_i2s_set_priv(struct mtk_base_afe *afe)
{
	struct mt6878_afe_private *afe_priv = afe->platform_priv;
	struct mtk_afe_i2s_priv *priv;

	priv = devm_kzalloc(afe->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	afe_priv->dai_priv[MT6878_DAI_I2S_OUT4] = priv;

	return 0;
}

int mt6878_dai_i2s_register(struct mtk_base_afe *afe)
{
	struct mtk_base_afe_dai *dai;

	dai = devm_kzalloc(afe->dev, sizeof(*dai), GFP_KERNEL);
	if (!dai)
		return -ENOMEM;

	list_add(&dai->list, &afe->sub_dais);

	dai->dai_drivers = mtk_dai_i2sout4_driver;
	dai->num_dai_drivers = ARRAY_SIZE(mtk_dai_i2sout4_driver);

	dai->dapm_widgets = mtk_dai_i2sout4_widgets;
	dai->num_dapm_widgets = ARRAY_SIZE(mtk_dai_i2sout4_widgets);
	dai->dapm_routes = mtk_dai_i2sout4_routes;
	dai->num_dapm_routes = ARRAY_SIZE(mtk_dai_i2sout4_routes);

	return mt6878_dai_i2s_set_priv(afe);
}
