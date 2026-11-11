// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek MT6878 ALSA SoC AFE -- hostless speech DAI
 *
 * Copyright (c) 2026 Jan Benedikt <karagos01@gmail.com>
 *
 * During a call no audio passes through memory: the modem's PCM 1 link is
 * wired to the ADDA codec link through the interconnect. "Hostless Speech" is
 * a front end without a buffer that, while its PCM device is open and
 * running, keeps the PCM 1 and ADDA back ends powered at the call's sample
 * rate (as the stock HAL does).
 */

#include "mt6878-afe-common.h"

static const struct snd_soc_dapm_route mtk_dai_hostless_routes[] = {
	/* modem downlink -> earpiece / speaker */
	{"ADDA_DL_CH1", "PCM_1_CAP_CH1", "Hostless Speech DL"},
	{"ADDA_DL_CH2", "PCM_1_CAP_CH1", "Hostless Speech DL"},
	/* microphone -> modem uplink */
	{"PCM_1_PB_CH1", "ADDA_UL_CH1", "Hostless Speech DL"},
	{"PCM_1_PB_CH1", "ADDA_UL_CH2", "Hostless Speech DL"},
	{"PCM_1_PB_CH2", "ADDA_UL_CH1", "Hostless Speech DL"},
	{"PCM_1_PB_CH2", "ADDA_UL_CH2", "Hostless Speech DL"},

	{"Hostless Speech UL", NULL, "PCM 1 Capture"},
	{"Hostless Speech UL", NULL, "ADDA Capture"},
};

static int mtk_dai_hostless_startup(struct snd_pcm_substream *substream,
				    struct snd_soc_dai *dai)
{
	struct mtk_base_afe *afe = snd_soc_dai_get_drvdata(dai);

	return snd_soc_set_runtime_hwparams(substream, afe->mtk_afe_hardware);
}

static const struct snd_soc_dai_ops mtk_dai_hostless_ops = {
	.startup = mtk_dai_hostless_startup,
};

#define MTK_HOSTLESS_RATES (SNDRV_PCM_RATE_8000 |\
			    SNDRV_PCM_RATE_16000 |\
			    SNDRV_PCM_RATE_32000 |\
			    SNDRV_PCM_RATE_48000)

#define MTK_HOSTLESS_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
			      SNDRV_PCM_FMTBIT_S24_LE |\
			      SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver mtk_dai_hostless_driver[] = {
	{
		.name = "Hostless Speech DAI",
		.id = MT6878_DAI_HOSTLESS_SPEECH,
		.playback = {
			.stream_name = "Hostless Speech DL",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MTK_HOSTLESS_RATES,
			.formats = MTK_HOSTLESS_FORMATS,
		},
		.capture = {
			.stream_name = "Hostless Speech UL",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MTK_HOSTLESS_RATES,
			.formats = MTK_HOSTLESS_FORMATS,
		},
		.ops = &mtk_dai_hostless_ops,
	},
};

int mt6878_dai_hostless_register(struct mtk_base_afe *afe)
{
	struct mtk_base_afe_dai *dai;

	dai = devm_kzalloc(afe->dev, sizeof(*dai), GFP_KERNEL);
	if (!dai)
		return -ENOMEM;

	list_add(&dai->list, &afe->sub_dais);

	dai->dai_drivers = mtk_dai_hostless_driver;
	dai->num_dai_drivers = ARRAY_SIZE(mtk_dai_hostless_driver);
	dai->dapm_routes = mtk_dai_hostless_routes;
	dai->num_dapm_routes = ARRAY_SIZE(mtk_dai_hostless_routes);

	return 0;
}
