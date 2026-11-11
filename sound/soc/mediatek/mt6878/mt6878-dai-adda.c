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
	SUPPLY_SEQ_ADDA_MTKAIF_CFG,
	SUPPLY_SEQ_ADDA_UL_MTKAIF,
	SUPPLY_SEQ_ADDA_AFE_ON,
	SUPPLY_SEQ_ADDA_DL_ON,
	SUPPLY_SEQ_ADDA_UL_ON,
};

/*
 * MTKAIF shared RX pad-top config. Captured from stock earpiece playback;
 * the AFE_MTKAIF{0,1}_CFG0 registers (0x1190/0x11f0) are left at their
 * hardware default (0) as stock does - the active link config lives in the
 * adjacent 0x1194.. registers, which already hold the correct reset values.
 */
#define MTKAIF_PAD_VAL		0x000000b9

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

/* uplink SRC_CON0 rate word (RE from the stock capture ftrace) */
static unsigned int mtk_adda_ul_rate_transform(struct mtk_base_afe *afe,
					       unsigned int rate)
{
	switch (rate) {
	case 8000:
		return 0x00080400;
	case 16000:
		return 0x00040400;
	case 32000:
		return 0x00020400;
	case 48000:
		return 0x00060400;
	case 96000:
		return 0x000e0400;
	case 192000:
		return 0x00160400;
	default:
		dev_info(afe->dev, "%s(), rate %d invalid, use 48kHz!!!\n",
			 __func__, rate);
		return 0x00060400;
	}
}

/* connection-matrix mixers: route the DL0 memif into the ADDA downlink */
static const struct snd_kcontrol_new mtk_adda_dl_ch1_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("DL0_CH1", AFE_CONN_ADDA_DL_CH1,
				    I_DL0_CH1, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("PCM_1_CAP_CH1", AFE_CONN_ADDA_DL_CH1_PCM,
				    I_PCM_1_CAP_CH1, 1, 0),
};

static const struct snd_kcontrol_new mtk_adda_dl_ch2_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("DL0_CH2", AFE_CONN_ADDA_DL_CH2,
				    I_DL0_CH2, 1, 0),
	SOC_DAPM_SINGLE_AUTODISABLE("PCM_1_CAP_CH1", AFE_CONN_ADDA_DL_CH2_PCM,
				    I_PCM_1_CAP_CH1, 1, 0),
};

/* connection-matrix mixers: route the ADDA uplink into the VUL9 memif */
static const struct snd_kcontrol_new mtk_adda_vul9_ch1_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("ADDA_UL_CH1", AFE_CONN_VUL9_CH1,
				    I_ADDA_UL_CH1, 1, 0),
};

static const struct snd_kcontrol_new mtk_adda_vul9_ch2_mix[] = {
	SOC_DAPM_SINGLE_AUTODISABLE("ADDA_UL_CH2", AFE_CONN_VUL9_CH2,
				    I_ADDA_UL_CH2, 1, 0),
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
		/* drop the downlink AFE-on, then drain before releasing pins */
		regmap_update_bits(afe->regmap, AFE_ADDA_UL_DL_CON0,
				   AFE_ADDA_UL_DL_CON0_ADDA_AFE_ON, 0);
		usleep_range(125, 150);
		mt6878_afe_gpio_request(afe, false, MT6878_DAI_ADDA, 0);
		break;
	default:
		break;
	}

	return 0;
}

/* Configure the MTKAIF RX pad that carries the samples to/from the MT6369
 * PMIC codec. Matches stock earpiece playback.
 */
static int mtk_adda_mtkaif_event(struct snd_soc_dapm_widget *w,
				 struct snd_kcontrol *kcontrol,
				 int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/*
		 * Enable both halves of the MTKAIF serial link so the downlink
		 * can clock samples to the codec. The uplink path enables this
		 * too; for a playback-only stream it must be done here or the
		 * MT6369 never receives the downlink data (silent output).
		 */
		regmap_write(afe->regmap, AFE_MTKAIF0_CFG0, AFE_MTKAIF_CFG0_CFG);
		regmap_write(afe->regmap, AFE_MTKAIF1_CFG0, AFE_MTKAIF_CFG0_CFG);
		regmap_write(afe->regmap, AFE_MTKAIF0_CFG0, AFE_MTKAIF_CFG0_EN);
		regmap_write(afe->regmap, AFE_MTKAIF1_CFG0, AFE_MTKAIF_CFG0_EN);

		regmap_write(afe->regmap, AFE_ADDA_MTKAIF_RX_CFG0, MTKAIF_PAD_VAL);
		break;
	default:
		break;
	}

	return 0;
}

static int mtk_adda_ul_event(struct snd_soc_dapm_widget *w,
			     struct snd_kcontrol *kcontrol,
			     int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);

	dev_dbg(afe->dev, "%s(), name %s, event 0x%x\n",
		__func__, w->name, event);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* mux the MTKAIF uplink data lines to the audio function */
		mt6878_afe_gpio_request(afe, true, MT6878_DAI_ADDA, 1);
		break;
	case SND_SOC_DAPM_POST_PMD:
		/* let the uplink SRC drain before the pins are released */
		usleep_range(125, 150);
		mt6878_afe_gpio_request(afe, false, MT6878_DAI_ADDA, 1);
		break;
	default:
		break;
	}

	return 0;
}

/*
 * Bring up the MTKAIF serial link for the uplink direction. RE from the stock
 * capture ftrace: enable both link halves (CFG0 bit31) and the RX channels
 * (RX_CFG2), then assert the shared RX pad enable.
 */
static int mtk_adda_ul_mtkaif_event(struct snd_soc_dapm_widget *w,
				    struct snd_kcontrol *kcontrol,
				    int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt);
	struct mt6878_afe_private *afe_priv = afe->platform_priv;
	unsigned int rx_cfg2_0 = AFE_MTKAIF_RX_CFG2_BASE;
	unsigned int rx_cfg2_1 = AFE_MTKAIF_RX_CFG2_BASE;
	int phase0 = afe_priv->mtkaif_phase_cycle[0];
	int phase1 = afe_priv->mtkaif_phase_cycle[1];

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		regmap_write(afe->regmap, AFE_MTKAIF0_CFG0, AFE_MTKAIF_CFG0_CFG);
		regmap_write(afe->regmap, AFE_MTKAIF1_CFG0, AFE_MTKAIF_CFG0_CFG);
		regmap_write(afe->regmap, AFE_MTKAIF0_CFG0, AFE_MTKAIF_CFG0_EN);
		regmap_write(afe->regmap, AFE_MTKAIF1_CFG0, AFE_MTKAIF_CFG0_EN);

		/*
		 * Program the per-MISO RX sampling delay from the calibration
		 * result instead of a fixed value: delay whichever line locked
		 * at the lower phase by the phase difference so both are sampled
		 * together. When both lock at the same phase the delay is zero.
		 */
		if (phase0 > phase1)
			rx_cfg2_1 |= AFE_MTKAIF_RXIF_DELAY_DATA |
				((phase0 - phase1) << AFE_MTKAIF_RXIF_DELAY_CYCLE_SFT);
		else if (phase1 > phase0)
			rx_cfg2_0 |= AFE_MTKAIF_RXIF_DELAY_DATA |
				((phase1 - phase0) << AFE_MTKAIF_RXIF_DELAY_CYCLE_SFT);

		regmap_write(afe->regmap, AFE_MTKAIF0_RX_CFG2, rx_cfg2_0);
		regmap_write(afe->regmap, AFE_MTKAIF1_RX_CFG2, rx_cfg2_1);
		regmap_write(afe->regmap, AFE_ADDA_MTKAIF_RX_CFG0,
			     AFE_ADDA_MTKAIF_RX_CFG0_UL);
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

	SND_SOC_DAPM_SUPPLY_S("ADDA Playback Enable", SUPPLY_SEQ_ADDA_DL_ON,
			      AFE_ADDA_DL_SRC2_CON0,
			      AFE_ADDA_DL_SRC2_CON0_SRC_ON_SFT, 0,
			      mtk_adda_dl_event,
			      SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_SUPPLY_S("MTKAIF Config", SUPPLY_SEQ_ADDA_MTKAIF_CFG,
			      SND_SOC_NOPM, 0, 0, mtk_adda_mtkaif_event,
			      SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	/* uplink (capture) inter-connections */
	SND_SOC_DAPM_MIXER("VUL9_CH1", SND_SOC_NOPM, 0, 0,
			   mtk_adda_vul9_ch1_mix,
			   ARRAY_SIZE(mtk_adda_vul9_ch1_mix)),
	SND_SOC_DAPM_MIXER("VUL9_CH2", SND_SOC_NOPM, 0, 0,
			   mtk_adda_vul9_ch2_mix,
			   ARRAY_SIZE(mtk_adda_vul9_ch2_mix)),

	SND_SOC_DAPM_SUPPLY_S("ADDA Capture Enable", SUPPLY_SEQ_ADDA_UL_ON,
			      AFE_ADDA_UL_SRC_CON0,
			      AFE_ADDA_UL_SRC_CON0_SRC_ON_SFT, 0,
			      mtk_adda_ul_event,
			      SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_SUPPLY_S("ADDA UL MTKAIF", SUPPLY_SEQ_ADDA_UL_MTKAIF,
			      SND_SOC_NOPM, 0, 0, mtk_adda_ul_mtkaif_event,
			      SND_SOC_DAPM_PRE_PMU),
};

static const struct snd_soc_dapm_route mtk_dai_adda_routes[] = {
	/* playback */
	{"ADDA_DL_CH1", "DL0_CH1", "DL0"},
	{"ADDA_DL_CH2", "DL0_CH2", "DL0"},
	{"ADDA_DL_CH1", "PCM_1_CAP_CH1", "PCM 1 Capture"},
	{"ADDA_DL_CH2", "PCM_1_CAP_CH1", "PCM 1 Capture"},

	{"ADDA Playback", NULL, "ADDA_DL_CH1"},
	{"ADDA Playback", NULL, "ADDA_DL_CH2"},

	{"ADDA Playback", NULL, "ADDA Playback Enable"},
	{"ADDA Playback", NULL, "MTKAIF Config"},

	/* capture: ADDA uplink -> connection matrix -> VUL9 memif */
	{"VUL9_CH1", "ADDA_UL_CH1", "ADDA Capture"},
	{"VUL9_CH2", "ADDA_UL_CH2", "ADDA Capture"},

	{"UL9", NULL, "VUL9_CH1"},
	{"UL9", NULL, "VUL9_CH2"},

	{"ADDA Capture", NULL, "ADDA Capture Enable"},
	{"ADDA Capture", NULL, "ADDA UL MTKAIF"},
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

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		adda_priv->ul_rate = rate;

		/*
		 * Enable the ADDA downlink SRC as well: the stock capture path
		 * turns the DL SRC on to clock the shared ADDA block even when
		 * only capturing (no DL data flows without the DL interconnect).
		 */
		regmap_write(afe->regmap, AFE_ADDA_DL_SRC2_CON0,
			     AFE_ADDA_DL_SRC2_48K |
			     BIT(AFE_ADDA_DL_SRC2_CON0_SRC_ON_SFT) | BIT(0));
		regmap_write(afe->regmap, AFE_ADDA_DL_SRC2_CON1, 0xf74ff74f);
		regmap_write(afe->regmap, AFE_ADDA_DL_SDM_DITHER_CON, 0x190000);

		/* uplink SRC gain (0 dB) - without this the SRC output is muted */
		regmap_write(afe->regmap, AFE_ADDA_UL_SRC_CON1,
			     AFE_ADDA_UL_SRC_CON1_VAL);
		regmap_write(afe->regmap, AFE_ADDA_UL_SRC2_CON1,
			     AFE_ADDA_UL_SRC_CON1_VAL);

		/* uplink SRC fixed coefficient/voice-mode setup (RE verbatim) */
		regmap_write(afe->regmap, AFE_ADDA_UL_SRC_VOICE_MODE_CFG1, 0);
		regmap_write(afe->regmap, AFE_ADDA_UL_SRC_VOICE_MODE_CFG2,
			     0x00003fb8);
		regmap_write(afe->regmap, AFE_ADDA_UL_SRC_VOICE_MODE_CFG3,
			     0x3fb80000);
		regmap_write(afe->regmap, AFE_ADDA_UL_SRC_VOICE_MODE_CFG4,
			     0x3fb80000);
		regmap_write(afe->regmap, AFE_ADDA_UL_SRC_VOICE_MODE_CFG5,
			     0x0000c048);

		/* program the UL SRC rate; the SRC-on bit is driven by DAPM */
		regmap_write(afe->regmap, AFE_ADDA_UL_SRC_CON0,
			     mtk_adda_ul_rate_transform(afe, rate));

		/* ADDA analog interface: UL clock, UL config and UL AFE-on */
		regmap_update_bits(afe->regmap, AFE_ADDA_UL_DL_CON0, 0x3f, 0x1d);
		regmap_update_bits(afe->regmap, AFE_ADDA_UL_DL_CON0,
				   AFE_ADDA_UL_DL_CON0_UL_CFG,
				   AFE_ADDA_UL_DL_CON0_UL_CFG);
		regmap_update_bits(afe->regmap, AFE_ADDA_UL_DL_CON0,
				   AFE_ADDA_UL_DL_CON0_ADDA_UL_AFE_ON,
				   AFE_ADDA_UL_DL_CON0_ADDA_UL_AFE_ON);

		return 0;
	}

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return 0;

	adda_priv->dl_rate = rate;

	/* clean the sigma-delta modulator path */
	regmap_write(afe->regmap, AFE_ADDA_DL_SDM_DCCOMP_CON, 0);
	regmap_write(afe->regmap, AFE_ADDA_DL_SDM_TEST, 0);

	/* set the downlink SRC rate and turn the SRC on (bit1) plus the
	 * downlink mute-off/enable at bit0, matching stock playback.
	 */
	regmap_write(afe->regmap, AFE_ADDA_DL_SRC2_CON0,
		     mtk_adda_dl_rate_transform(afe, rate) |
		     BIT(AFE_ADDA_DL_SRC2_CON0_SRC_ON_SFT) | BIT(0));
	regmap_write(afe->regmap, AFE_ADDA_DL_SRC2_CON1, 0xf74ff74f);

	/*
	 * ADDA analog interface clock and downlink AFE-on. The AFE-on bit must
	 * be set for the ADDA downlink to drive the MTKAIF TX link to the codec;
	 * without it the SRC runs but no samples reach the MT6369 DAC (silent
	 * receiver and speaker).
	 */
	regmap_update_bits(afe->regmap, AFE_ADDA_UL_DL_CON0, 0x3f, 0x1d);
	regmap_update_bits(afe->regmap, AFE_ADDA_UL_DL_CON0,
			   AFE_ADDA_UL_DL_CON0_ADDA_AFE_ON,
			   AFE_ADDA_UL_DL_CON0_ADDA_AFE_ON);

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
		.capture = {
			.stream_name = "ADDA Capture",
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
