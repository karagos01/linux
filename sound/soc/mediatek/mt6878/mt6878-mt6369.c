// SPDX-License-Identifier: GPL-2.0
//
// mt6878-mt6369.c  --  MT6878 MT6369 ALSA SoC machine driver
//
// Copyright (c) 2026 Jan Benedikt <karagos01@gmail.com>

#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <sound/soc.h>

#include "mt6878-afe-common.h"
#include "mt6878-afe-gpio.h"
#include "mt6878-reg.h"
#include "../../codecs/mt6369.h"

/* CKSYS audio-top calibration monitor, on the topckgen syscon. */
#define CKSYS_AUD_TOP_CFG	0x032c
#define CKSYS_AUD_TOP_MON	0x0330

SND_SOC_DAILINK_DEFS(playback_0,
	DAILINK_COMP_ARRAY(COMP_CPU("DL0")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(capture_0,
	DAILINK_COMP_ARRAY(COMP_CPU("UL9")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(primary_codec,
	DAILINK_COMP_ARRAY(COMP_CPU("ADDA")),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "mt6369-snd-codec-aif1")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

/*
 * Parked I2S speaker path (DL1 -> ETDM_OUT4/I2S4 -> dummy): the AFE I2S master
 * is verified but the Volla Plinius has no I2S smart-amp populated, so the real
 * speaker is driven through the mt6369 headphone output into the external
 * aw87xxx amplifiers. These links keep the ETDM back end exercisable with a
 * dummy codec until that work lands.
 */
SND_SOC_DAILINK_DEFS(speaker_playback,
	DAILINK_COMP_ARRAY(COMP_CPU("DL1")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(hostless_speech,
	DAILINK_COMP_ARRAY(COMP_CPU("Hostless Speech DAI")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(pcm_1,
	DAILINK_COMP_ARRAY(COMP_CPU("PCM 1")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(i2s_out4,
	DAILINK_COMP_ARRAY(COMP_CPU("I2SOUT4")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

/*
 * MTKAIF RX phase calibration between the MT6878 AFE and the MT6369 codec.
 * The AFE sends a synchronizer pulse while the codec sweeps its RX delay; the
 * phase at which the measured cycle count first changes is the lock edge.
 * Run once at card init - without it the codec does not latch the downlink
 * samples. Mirrors the stock machine driver and the mt8192 equivalent.
 */
static int mt6878_mt6369_mtkaif_calibration(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *cmpnt_afe =
		snd_soc_rtd_to_cpu(rtd, 0)->component;
	struct snd_soc_component *cmpnt_codec =
		snd_soc_rtd_to_codec(rtd, 0)->component;
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt_afe);
	struct mt6878_afe_private *afe_priv = afe->platform_priv;
	int phase, counter;
	unsigned int monitor;
	int test_done_1, test_done_2;
	int cycle_1, cycle_2;
	int prev_cycle_1 = 0, prev_cycle_2 = 0;
	int chosen_phase_1 = -1, chosen_phase_2 = -1;
	bool calib_ok = true;

	pm_runtime_get_sync(afe->dev);
	mt6878_afe_gpio_request(afe, true, MT6878_DAI_ADDA, 1);
	mt6878_afe_gpio_request(afe, true, MT6878_DAI_ADDA, 0);

	mt6369_mtkaif_calibration_enable(cmpnt_codec);

	/* enable the MTKAIF RX fifo (protocol 2) */
	regmap_update_bits(afe->regmap, AFE_ADDA_MTKAIF_RX_CFG0, 0xff, 0xb8);
	regmap_update_bits(afe->regmap, AFE_ADDA_MTKAIF_RX_CFG0, 0xff, 0xb9);
	/* set test type to synchronizer pulse */
	regmap_update_bits(afe_priv->topckgen, CKSYS_AUD_TOP_CFG, 0xffff, 0x4);

	for (phase = 0; phase <= 42 && calib_ok; phase++) {
		mt6369_set_mtkaif_calibration_phase(cmpnt_codec,
						    phase, phase, phase);
		regmap_update_bits(afe_priv->topckgen, CKSYS_AUD_TOP_CFG, 0x1, 0x1);

		test_done_1 = 0;
		test_done_2 = 0;
		cycle_1 = -1;
		cycle_2 = -1;
		counter = 0;
		while (test_done_1 == 0 || test_done_2 == 0) {
			regmap_read(afe_priv->topckgen, CKSYS_AUD_TOP_MON,
				    &monitor);
			test_done_1 = (monitor >> 28) & 0x1;
			test_done_2 = (monitor >> 29) & 0x1;
			if (test_done_1)
				cycle_1 = monitor & 0xf;
			if (test_done_2)
				cycle_2 = (monitor >> 4) & 0xf;
			if (++counter > 10000) {
				dev_err(afe->dev,
					"%s(), calibration timeout, monitor 0x%x\n",
					__func__, monitor);
				calib_ok = false;
				break;
			}
		}

		if (phase == 0) {
			prev_cycle_1 = cycle_1;
			prev_cycle_2 = cycle_2;
		}
		if (cycle_1 != prev_cycle_1 && chosen_phase_1 < 0)
			chosen_phase_1 = phase - 1;
		if (cycle_2 != prev_cycle_2 && chosen_phase_2 < 0)
			chosen_phase_2 = phase - 1;

		regmap_update_bits(afe_priv->topckgen, CKSYS_AUD_TOP_CFG, 0x1, 0x0);

		if (chosen_phase_1 >= 0 && chosen_phase_2 >= 0)
			break;
	}

	if (chosen_phase_1 < 0)
		chosen_phase_1 = 0;
	if (chosen_phase_2 < 0)
		chosen_phase_2 = 0;

	/*
	 * Keep the per-MISO lock phases so the uplink bring-up can program the
	 * matching RX sampling delay (the two lines may lock at different
	 * phases; the earlier one is delayed to realign them).
	 */
	afe_priv->mtkaif_phase_cycle[0] = chosen_phase_1;
	afe_priv->mtkaif_phase_cycle[1] = chosen_phase_2;

	mt6369_set_mtkaif_calibration_phase(cmpnt_codec,
					    chosen_phase_1, chosen_phase_2, 0);

	/* disable the RX fifo */
	regmap_update_bits(afe->regmap, AFE_ADDA_MTKAIF_RX_CFG0, 0xff, 0xb8);
	mt6369_mtkaif_calibration_disable(cmpnt_codec);

	/*
	 * The codec calibration teardown leaves the MTKAIF calibration-mode
	 * bits set in MTKAIF_CAL_CON, but the stock codec runs normal playback
	 * with those bits cleared (the register reads 0 during stock earpiece
	 * playback). Leaving them set keeps the RX interface in calibration
	 * mode, so clear them here once calibration has finished.
	 */
	snd_soc_component_update_bits(cmpnt_codec, MT6369_AUD_TOP_CKPDN_CON0,
				      0x66, 0x00);

	mt6878_afe_gpio_request(afe, false, MT6878_DAI_ADDA, 1);
	mt6878_afe_gpio_request(afe, false, MT6878_DAI_ADDA, 0);
	pm_runtime_put(afe->dev);

	dev_info(afe->dev, "%s(), chosen phase %d/%d\n",
		 __func__, chosen_phase_1, chosen_phase_2);
	return 0;
}

static int mt6878_mt6369_init(struct snd_soc_pcm_runtime *rtd)
{
	return mt6878_mt6369_mtkaif_calibration(rtd);
}

static struct snd_soc_dai_link mt6878_mt6369_dai_links[] = {
	/* front end */
	{
		.name = "Playback_0",
		.stream_name = "Playback_0",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.playback_only = 1,
		SND_SOC_DAILINK_REG(playback_0),
	},
	{
		.name = "Capture_0",
		.stream_name = "Capture_0",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.capture_only = 1,
		/* offer only the rates the ADDA uplink can run at */
		.dpcm_merged_rate = 1,
		SND_SOC_DAILINK_REG(capture_0),
	},
	{
		.name = "Speaker_Playback",
		.stream_name = "Speaker_Playback",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.playback_only = 1,
		SND_SOC_DAILINK_REG(speaker_playback),
	},
	{
		/* held open during a call: clocks the modem PCM link */
		.name = "Hostless_Speech",
		.stream_name = "Hostless_Speech",
		.trigger = {SND_SOC_DPCM_TRIGGER_PRE,
			    SND_SOC_DPCM_TRIGGER_PRE},
		.dynamic = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(hostless_speech),
	},
	/* back end */
	{
		.name = "Primary Codec",
		.no_pcm = 1,
		.ignore_suspend = 1,
		.init = mt6878_mt6369_init,
		SND_SOC_DAILINK_REG(primary_codec),
	},
	{
		.name = "I2SOUT4 BE",
		.no_pcm = 1,
		.playback_only = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(i2s_out4),
	},
	{
		.name = "PCM 1",
		.no_pcm = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(pcm_1),
	},
};

/*
 * External analog boost amplifiers (aw87565) on the mt6369 LINEOUT output.
 * They carry no DAI, so they are attached as auxiliary components. On the
 * Volla Plinius the two amps drive two different transducers: "Spk L" (@0x58)
 * is the top earpiece/receiver, "Spk R" (@0x5b) is the bottom loudspeaker.
 * They are wired to two separate codec output pins so the earpiece and the
 * loudspeaker can be enabled independently ("Receiver Switch" / "Speaker
 * Switch"); each amp powers up with its output.
 */
static struct snd_soc_aux_dev mt6878_mt6369_spk_amps[2];

static const struct snd_soc_dapm_route mt6878_mt6369_spk_routes[] = {
	{ "Spk L IN", NULL, "Receiver Amp" },
	{ "Spk R IN", NULL, "Speaker" },
};

static struct snd_soc_card mt6878_mt6369_card = {
	.name = "mt6878-mt6369",
	.owner = THIS_MODULE,
	.dai_link = mt6878_mt6369_dai_links,
	.num_links = ARRAY_SIZE(mt6878_mt6369_dai_links),
};

static int mt6878_mt6369_dev_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &mt6878_mt6369_card;
	struct device_node *platform_node, *codec_node;
	struct snd_soc_dai_link *dai_link;
	int ret, i, num_amps;

	card->dev = &pdev->dev;

	platform_node = of_parse_phandle(pdev->dev.of_node,
					 "mediatek,platform", 0);
	if (!platform_node)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "Property 'mediatek,platform' missing or invalid\n");

	for_each_card_prelinks(card, i, dai_link) {
		if (dai_link->platforms->name)
			continue;
		dai_link->platforms->of_node = platform_node;
	}

	codec_node = of_parse_phandle(pdev->dev.of_node,
				      "mediatek,audio-codec", 0);
	if (!codec_node) {
		ret = dev_err_probe(&pdev->dev, -EINVAL,
				    "Property 'mediatek,audio-codec' missing or invalid\n");
		goto put_platform_node;
	}

	for_each_card_prelinks(card, i, dai_link) {
		if (dai_link->codecs->name)
			continue;
		dai_link->codecs->of_node = codec_node;
	}

	/* Attach the external speaker amplifiers, if any, as aux components. */
	num_amps = of_count_phandle_with_args(pdev->dev.of_node,
					      "mediatek,speaker-amps", NULL);
	if (num_amps > (int)ARRAY_SIZE(mt6878_mt6369_spk_amps))
		num_amps = ARRAY_SIZE(mt6878_mt6369_spk_amps);
	for (i = 0; i < num_amps; i++)
		mt6878_mt6369_spk_amps[i].dlc.of_node =
			of_parse_phandle(pdev->dev.of_node,
					 "mediatek,speaker-amps", i);
	if (num_amps > 0) {
		card->aux_dev = mt6878_mt6369_spk_amps;
		card->num_aux_devs = num_amps;
		card->dapm_routes = mt6878_mt6369_spk_routes;
		card->num_dapm_routes = num_amps;
	}

	ret = devm_snd_soc_register_card(&pdev->dev, card);

	for (i = 0; i < num_amps; i++)
		of_node_put(mt6878_mt6369_spk_amps[i].dlc.of_node);
	of_node_put(codec_node);
put_platform_node:
	of_node_put(platform_node);
	return ret;
}

static const struct of_device_id mt6878_mt6369_dt_match[] = {
	{ .compatible = "mediatek,mt6878-mt6369-sound", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mt6878_mt6369_dt_match);

static struct platform_driver mt6878_mt6369_driver = {
	.driver = {
		.name = "mt6878-mt6369",
		.of_match_table = mt6878_mt6369_dt_match,
	},
	.probe = mt6878_mt6369_dev_probe,
};
module_platform_driver(mt6878_mt6369_driver);

MODULE_DESCRIPTION("MT6878 MT6369 ALSA SoC machine driver");
MODULE_AUTHOR("Jan Benedikt <karagos01@gmail.com>");
MODULE_LICENSE("GPL");
