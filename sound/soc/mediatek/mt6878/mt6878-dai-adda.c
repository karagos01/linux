// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek MT6878 ALSA SoC AFE -- ADDA DAI (internal PMIC-codec link)
 *
 * Copyright (c) 2026 Jan Benedikt <karagos01@gmail.com>
 *
 * The ADDA block is the downlink path from the AFE memory interfaces to the
 * MT6369 PMIC analog codec over the MTKAIF serial link. This driver wires up
 * the downlink (playback) path only: the DL0 memif is routed into the ADDA
 * downlink SRC through the connection matrix, the ADDA downlink and the AFE
 * analog interface are enabled, and the data is handed to the codec.
 *
 * All register values are reverse engineered from the downstream vendor module
 * (snd-soc-mt6878-afe.ko): mtk_dai_adda_hw_params() for the downlink SRC/SDM
 * setup and mtk_adda_dl_ch{1,2}_mix for the connection-matrix mixer controls.
 * The uplink (capture), MTKAIF calibration and sidetone paths are added later.
 */

#include <linux/delay.h>
#include <linux/regmap.h>

#include <sound/pcm.h>
#include <sound/soc.h>

#include "mt6878-afe-common.h"
#include "mt6878-afe-gpio.h"
#include "../common/mtk-afe-fe-dai.h"

enum {
	SUPPLY_SEQ_ADDA_AFE_ON,
	SUPPLY_SEQ_ADDA_DL_ON,
};

/* downlink SRC2_CON0 rate word (rate field pre-positioned, RE from the .ko) */
static unsigned int mtk_adda_dl_rate_transform(struct mtk_base_afe *afe,
					       unsigned int rate)
{
	switch (rate) {
	case 8000:
		return 0x000c1820;
	case 11025:
		return 0x010c1800;
	case 12000:
		return 0x020c1800;
	case 16000:
		return 0x040c1820;
	case 22050:
		return 0x050c1800;
	case 24000:
		return 0x060c1800;
	case 32000:
		return 0x080c1800;
	case 44100:
		return 0x090c1800;
	case 48000:
		return 0x0a0c1800;
	case 96000:
		return 0x0e0c1800;
	case 192000:
		return 0x120c1800;
	default:
		dev_info(afe->dev, "%s(), rate %d invalid, use 48kHz!!!\n",
			 __func__, rate);
		return 0x0a0c1800;
	}
}

/* connection-matrix mixers: route the DL0 memif into the ADDA downlink */
static const struct snd_kcontrol_new mtk_adda_dl_ch1_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("DL0_CH1", AFE_CONN_ADDA_DL_CH1,
				    I_DL0_CH1, 1, 0),
};

static const struct snd_kcontrol_new mtk_adda_dl_ch2_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("DL0_CH2", AFE_CONN_ADDA_DL_CH2,
				    I_DL0_CH2, 1, 0),
};

static int mtk_adda_dl_event(struct snd_soc_dapm_widget *w,
			     struct snd_kcontrol *kcontrol,
			     int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);

	dev_dbg(afe->dev, "%s(), name %s, event 0x%x\n",
		__func__, w->name, event);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* mux the MTKAIF downlink data line to the audio function */
		mt6878_afe_gpio_request(afe, true, MT6878_DAI_ADDA, 0);
		break;
	case SND_SOC_DAPM_POST_PMD:
		/* let the downlink SRC drain before the pins are released */
		usleep_range(125, 150);
		mt6878_afe_gpio_request(afe, false, MT6878_DAI_ADDA, 0);
		break;
	default:
		break;
	}

	return 0;
}

static const struct snd_soc_dapm_widget mtk_dai_adda_widgets[] = {
	/* inter-connections */
	SND_SOC_DAPM_MIXER("ADDA_DL_CH1", SND_SOC_NOPM, 0, 0,
			   mtk_adda_dl_ch1_mix,
			   ARRAY_SIZE(mtk_adda_dl_ch1_mix)),
	SND_SOC_DAPM_MIXER("ADDA_DL_CH2", SND_SOC_NOPM, 0, 0,
			   mtk_adda_dl_ch2_mix,
			   ARRAY_SIZE(mtk_adda_dl_ch2_mix)),

	SND_SOC_DAPM_SUPPLY_S("ADDA Enable", SUPPLY_SEQ_ADDA_AFE_ON,
			      AFE_ADDA_UL_DL_CON0,
			      AFE_ADDA_UL_DL_CON0_ADDA_AFE_ON_SFT, 0,
			      NULL, 0),

	SND_SOC_DAPM_SUPPLY_S("ADDA Playback Enable", SUPPLY_SEQ_ADDA_DL_ON,
			      AFE_ADDA_DL_SRC2_CON0,
			      AFE_ADDA_DL_SRC2_CON0_SRC_ON_SFT, 0,
			      mtk_adda_dl_event,
			      SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_route mtk_dai_adda_routes[] = {
	/* playback */
	{"ADDA_DL_CH1", "DL0_CH1", "DL0"},
	{"ADDA_DL_CH2", "DL0_CH2", "DL0"},

	{"ADDA Playback", NULL, "ADDA_DL_CH1"},
	{"ADDA Playback", NULL, "ADDA_DL_CH2"},

	{"ADDA Playback", NULL, "ADDA Enable"},
	{"ADDA Playback", NULL, "ADDA Playback Enable"},
};

/* dai ops */
static int mtk_dai_adda_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params,
				  struct snd_soc_dai *dai)
{
	struct mtk_base_afe *afe = snd_soc_dai_get_drvdata(dai);
	struct mt6878_afe_private *afe_priv = afe->platform_priv;
	unsigned int rate = params_rate(params);
	struct mtk_afe_adda_priv *adda_priv;

	adda_priv = afe_priv->dai_priv[dai->id];
	if (!adda_priv)
		return -EINVAL;

	dev_dbg(afe->dev, "%s(), id %d, stream %d, rate %d\n",
		__func__, dai->id, substream->stream, rate);

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return 0;

	adda_priv->dl_rate = rate;

	/* clean the sigma-delta modulator path */
	regmap_write(afe->regmap, AFE_ADDA_DL_SDM_DCCOMP_CON, 0);
	regmap_write(afe->regmap, AFE_ADDA_DL_SDM_TEST, 0);

	/* set the downlink SRC rate and turn the SRC on */
	regmap_write(afe->regmap, AFE_ADDA_DL_SRC2_CON0,
		     mtk_adda_dl_rate_transform(afe, rate) |
		     BIT(AFE_ADDA_DL_SRC2_CON0_SRC_ON_SFT));
	regmap_write(afe->regmap, AFE_ADDA_DL_SRC2_CON1, 0xf74ff74f);

	/* ADDA analog interface clock; the AFE-on bit is driven by DAPM */
	regmap_update_bits(afe->regmap, AFE_ADDA_UL_DL_CON0, 0x3f, 0x1d);
	regmap_update_bits(afe->regmap, AFE_ADDA_UL_DL_CON0,
			   AFE_ADDA_UL_DL_CON0_ADDA_AFE_ON, 0);

	/* sigma-delta dither */
	regmap_write(afe->regmap, AFE_ADDA_DL_SDM_DITHER_CON, 0x190000);
	regmap_update_bits(afe->regmap, AFE_ADDA_DL_SDM_DITHER_CON, 0x1f, 0);

	return 0;
}

static const struct snd_soc_dai_ops mtk_dai_adda_ops = {
	.hw_params = mtk_dai_adda_hw_params,
};

/* dai driver */
#define MTK_ADDA_PLAYBACK_RATES (SNDRV_PCM_RATE_8000_48000 |\
				 SNDRV_PCM_RATE_96000 |\
				 SNDRV_PCM_RATE_192000)

#define MTK_ADDA_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
			  SNDRV_PCM_FMTBIT_S24_LE |\
			  SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver mtk_dai_adda_driver[] = {
	{
		.name = "ADDA",
		.id = MT6878_DAI_ADDA,
		.playback = {
			.stream_name = "ADDA Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MTK_ADDA_PLAYBACK_RATES,
			.formats = MTK_ADDA_FORMATS,
		},
		.ops = &mtk_dai_adda_ops,
	},
};

static int mt6878_dai_adda_set_priv(struct mtk_base_afe *afe)
{
	struct mt6878_afe_private *afe_priv = afe->platform_priv;
	struct mtk_afe_adda_priv *priv;

	priv = devm_kzalloc(afe->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	afe_priv->dai_priv[MT6878_DAI_ADDA] = priv;

	return 0;
}

int mt6878_dai_adda_register(struct mtk_base_afe *afe)
{
	struct mtk_base_afe_dai *dai;

	dai = devm_kzalloc(afe->dev, sizeof(*dai), GFP_KERNEL);
	if (!dai)
		return -ENOMEM;

	list_add(&dai->list, &afe->sub_dais);

	dai->dai_drivers = mtk_dai_adda_driver;
	dai->num_dai_drivers = ARRAY_SIZE(mtk_dai_adda_driver);

	dai->dapm_widgets = mtk_dai_adda_widgets;
	dai->num_dapm_widgets = ARRAY_SIZE(mtk_dai_adda_widgets);
	dai->dapm_routes = mtk_dai_adda_routes;
	dai->num_dapm_routes = ARRAY_SIZE(mtk_dai_adda_routes);

	return mt6878_dai_adda_set_priv(afe);
}
