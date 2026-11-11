// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek MT6878 ALSA SoC AFE -- modem PCM interface
 *
 * Copyright (c) 2026 Jan Benedikt <karagos01@gmail.com>
 *
 * PCM 1 is the speech link to the modem (MD1): during a call the modem's
 * downlink arrives on the PCM 1 capture port and is routed to the ADDA
 * downlink, and the ADDA uplink (microphone) is routed to the PCM 1 playback
 * ports. The modem drives the link once its speech path is on; the AFE side
 * only needs the interface format, rate and enable. Values are reverse
 * engineered from the stock module (snd-soc-mt6878-afe.ko).
 */

#include <linux/regmap.h>
#include <sound/pcm_params.h>

#include "mt6878-afe-common.h"

static const struct snd_kcontrol_new mtk_pcm_1_playback_ch1_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("ADDA_UL_CH1", AFE_CONN_PCM_1_PB_CH1,
				    I_ADDA_UL_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("ADDA_UL_CH2", AFE_CONN_PCM_1_PB_CH1,
				    I_ADDA_UL_CH2, 1, 0),
};

static const struct snd_kcontrol_new mtk_pcm_1_playback_ch2_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("ADDA_UL_CH1", AFE_CONN_PCM_1_PB_CH2,
				    I_ADDA_UL_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("ADDA_UL_CH2", AFE_CONN_PCM_1_PB_CH2,
				    I_ADDA_UL_CH2, 1, 0),
};

static const struct snd_soc_dapm_widget mtk_dai_pcm_widgets[] = {
	SND_SOC_DAPM_MIXER("PCM_1_PB_CH1", SND_SOC_NOPM, 0, 0,
			   mtk_pcm_1_playback_ch1_mix,
			   ARRAY_SIZE(mtk_pcm_1_playback_ch1_mix)),
	SND_SOC_DAPM_MIXER("PCM_1_PB_CH2", SND_SOC_NOPM, 0, 0,
			   mtk_pcm_1_playback_ch2_mix,
			   ARRAY_SIZE(mtk_pcm_1_playback_ch2_mix)),

	SND_SOC_DAPM_SUPPLY("PCM_1_EN", PCM_INTF_CON1, PCM_INTF_CON1_EN_SFT, 0,
			    NULL, 0),

	SND_SOC_DAPM_INPUT("MD1_TO_AFE"),
	SND_SOC_DAPM_OUTPUT("AFE_TO_MD1"),
};

static const struct snd_soc_dapm_route mtk_dai_pcm_routes[] = {
	{"PCM_1_PB_CH1", "ADDA_UL_CH1", "ADDA Capture"},
	{"PCM_1_PB_CH1", "ADDA_UL_CH2", "ADDA Capture"},
	{"PCM_1_PB_CH2", "ADDA_UL_CH1", "ADDA Capture"},
	{"PCM_1_PB_CH2", "ADDA_UL_CH2", "ADDA Capture"},

	{"PCM 1 Playback", NULL, "PCM_1_PB_CH1"},
	{"PCM 1 Playback", NULL, "PCM_1_PB_CH2"},

	{"PCM 1 Playback", NULL, "PCM_1_EN"},
	{"PCM 1 Capture", NULL, "PCM_1_EN"},

	{"AFE_TO_MD1", NULL, "PCM 1 Playback"},
	{"PCM 1 Capture", NULL, "MD1_TO_AFE"},
};

static int mtk_dai_pcm_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct mtk_base_afe *afe = snd_soc_dai_get_drvdata(dai);
	struct snd_soc_dapm_widget *p = snd_soc_dai_get_widget_playback(dai);
	struct snd_soc_dapm_widget *c = snd_soc_dai_get_widget_capture(dai);
	unsigned int rate = params_rate(params);
	unsigned int mode, rate_reg;

	/* both directions share the interface: configure it once */
	if (p->active || c->active)
		return 0;

	switch (rate) {
	case 8000:
		mode = 0;
		rate_reg = MTK_AFE_IPM2P0_RATE_8K;
		break;
	case 16000:
		mode = 1;
		rate_reg = MTK_AFE_IPM2P0_RATE_16K;
		break;
	case 32000:
		mode = 2;
		rate_reg = MTK_AFE_IPM2P0_RATE_32K;
		break;
	case 48000:
		mode = 3;
		rate_reg = MTK_AFE_IPM2P0_RATE_48K;
		break;
	default:
		dev_err(afe->dev, "%s(), unsupported rate %u\n", __func__, rate);
		return -EINVAL;
	}

	dev_dbg(afe->dev, "%s(), id %d, rate %u\n", __func__, dai->id, rate);

	regmap_update_bits(afe->regmap, PCM_INTF_CON1, (u32)~BIT(PCM_INTF_CON1_EN_SFT),
			   PCM_INTF_CON1_FMT | mode << PCM_INTF_CON1_MODE_SFT);
	regmap_update_bits(afe->regmap, PCM_INTF_CON2, PCM_INTF_CON2_RATE_MASK,
			   rate_reg << PCM_INTF_CON2_RATE_SFT);
	return 0;
}

static const struct snd_soc_dai_ops mtk_dai_pcm_ops = {
	.hw_params = mtk_dai_pcm_hw_params,
};

#define MTK_PCM_RATES (SNDRV_PCM_RATE_8000 |\
		       SNDRV_PCM_RATE_16000 |\
		       SNDRV_PCM_RATE_32000 |\
		       SNDRV_PCM_RATE_48000)

#define MTK_PCM_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
			 SNDRV_PCM_FMTBIT_S24_LE |\
			 SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver mtk_dai_pcm_driver[] = {
	{
		.name = "PCM 1",
		.id = MT6878_DAI_PCM_1,
		.playback = {
			.stream_name = "PCM 1 Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MTK_PCM_RATES,
			.formats = MTK_PCM_FORMATS,
		},
		.capture = {
			.stream_name = "PCM 1 Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MTK_PCM_RATES,
			.formats = MTK_PCM_FORMATS,
		},
		.ops = &mtk_dai_pcm_ops,
		.symmetric_rate = 1,
		.symmetric_sample_bits = 1,
	},
};

int mt6878_dai_pcm_register(struct mtk_base_afe *afe)
{
	struct mtk_base_afe_dai *dai;

	dai = devm_kzalloc(afe->dev, sizeof(*dai), GFP_KERNEL);
	if (!dai)
		return -ENOMEM;

	list_add(&dai->list, &afe->sub_dais);

	dai->dai_drivers = mtk_dai_pcm_driver;
	dai->num_dai_drivers = ARRAY_SIZE(mtk_dai_pcm_driver);
	dai->dapm_widgets = mtk_dai_pcm_widgets;
	dai->num_dapm_widgets = ARRAY_SIZE(mtk_dai_pcm_widgets);
	dai->dapm_routes = mtk_dai_pcm_routes;
	dai->num_dapm_routes = ARRAY_SIZE(mtk_dai_pcm_routes);

	return 0;
}
