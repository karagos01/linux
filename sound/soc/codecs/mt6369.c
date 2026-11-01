// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek MT6369 ALSA SoC audio codec driver
 *
 * Copyright (c) 2026 Jan Benedikt <karagos01@gmail.com>
 *
 * The MT6369 is the audio codec inside the MT6369 SPMI sub-PMIC. It is a close
 * relative of the MT6359 codec (same downlink/output topology) but is reached
 * over SPMI with single-byte registers, so it has its own register map.
 *
 * This is an incremental bring-up: the downlink earpiece (receiver) output
 * path is implemented; the headphone, line-out, uplink (microphone) and the
 * downlink digital clock/charge-pump supplies are still to come, as is the
 * 26 MHz DCXO enable that lives in the companion MT6685 clock buffer.
 */

#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>

#include "mt6369.h"

struct mt6369_priv {
	struct device *dev;
	struct regmap *regmap;
	int hp_cal;		/* HP DC-offset trim from the PMIC efuse */
};

/*
 * Zero-crossing detector. device: 2 = receiver, 1 = line-out, else headphone.
 * Recovered from the stock driver's zcd_enable().
 */
static void mt6369_zcd_enable(struct mt6369_priv *priv, bool enable, int device)
{
	if (!enable) {
		regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON20, 0x7, 0x4);
		regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON128, 0x0);
		return;
	}

	if (device != 2)
		device = (device != 1);

	regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON20, 0x7, device);
	regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON128, 0x40, 0x0);
	regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON128, 0x30, 0x0);
	regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON128, 0xe, 0xa);
	regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON128, 0x1, 0x1);
}

/* Downlink earpiece (receiver) analog output. Recovered from mt_rcv_event(). */
static int mt_rcv_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);
	struct regmap *regmap = priv->regmap;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON5, 0x1, 0x1);

		/* Negative charge pump for the class-AB amplifier. */
		regmap_update_bits(regmap, MT6369_AUDNCP_CLKDIV_CON1, 0x1, 0x1);
		regmap_write(regmap, MT6369_AUDNCP_CLKDIV_CON2, 0x2c);
		regmap_update_bits(regmap, MT6369_AUDNCP_CLKDIV_CON0, 0x1, 0x1);
		regmap_update_bits(regmap, MT6369_AUDNCP_CLKDIV_CON4, 0x3, 0x3);
		regmap_update_bits(regmap, MT6369_AUDNCP_CLKDIV_CON3, 0x1, 0x0);
		usleep_range(250, 270);

		/* Internal LDO. */
		regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON26, 0x10, 0x10);
		regmap_write(regmap, MT6369_AUDDEC_ANA_CON25, 0x55);
		regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON27, 0x1, 0x1);
		usleep_range(100, 120);

		/* Receiver bias. */
		regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON10, 0x10, 0x10);
		regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON22, 0x10, 0x0);
		regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON21, 0x70, 0x20);
		regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON23, 0xc0, 0x40);
		regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON23, 0xc, 0x4);
		regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON23, 0x3, 0x1);
		regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON23, 0x30, 0x10);

		/* Enable the receiver amplifier. */
		regmap_write(regmap, MT6369_AUDDEC_ANA_CON10, 0x90);
		regmap_write(regmap, MT6369_AUDDEC_ANA_CON17, 0x0);
		regmap_write(regmap, MT6369_AUDDEC_ANA_CON18, 0x0);
		regmap_write(regmap, MT6369_AUDDEC_ANA_CON19, 0x0);
		regmap_write(regmap, MT6369_AUDDEC_ANA_CON10, 0x92);
		regmap_write(regmap, MT6369_AUDDEC_ANA_CON10, 0x93);

		/* Receiver gain and output switch. */
		regmap_write(regmap, MT6369_AUDDEC_ANA_CON133, MT6369_HPGAIN_IDLE);
		regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON24, 0x1, 0x1);
		regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON0, 0x9, 0x9);
		regmap_write(regmap, MT6369_AUDDEC_ANA_CON16, 0x1);
		regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON10, 0xc, 0x8);

		mt6369_zcd_enable(priv, true, 2);
		break;

	case SND_SOC_DAPM_PRE_PMD:
		regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON10, 0xc, 0x0);
		regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON0, 0x9, 0x0);
		regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON24, 0x1, 0x0);
		regmap_write(regmap, MT6369_AUDDEC_ANA_CON133, MT6369_HPGAIN_IDLE);
		regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON10, 0x1, 0x0);
		regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON10, 0x2, 0x0);
		mt6369_zcd_enable(priv, false, 2);
		break;
	}

	return 0;
}

/* One-time analog configuration. Recovered from mt6369_codec_init_reg(). */
static void mt6369_codec_init_reg(struct mt6369_priv *priv)
{
	struct regmap *regmap = priv->regmap;

	/*
	 * TODO: the 26 MHz audio DCXO buffer is enabled here in the stock
	 * driver via the companion MT6685 clock chip (mt6685_set_dcxo*());
	 * that path is not wired up yet, so no downlink audio clock runs.
	 */

	regmap_update_bits(regmap, MT6369_AUDENC_ANA_CON27, 0x2, 0x0);
	regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON1, 0x10, 0x10);
	regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON1, 0x20, 0x20);
	regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON10, 0x10, 0x10);
	regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON12, 0x10, 0x10);
	regmap_update_bits(regmap, MT6369_AUDENC_ANA_CON38, 0xf0, 0x20);

	/* Analog pad drive strength. */
	regmap_update_bits(regmap, MT6369_SMT_CON1, 0xff, 0xfc);
	regmap_update_bits(regmap, MT6369_DRV_CON1, 0xf, 0xf);
	regmap_update_bits(regmap, MT6369_DRV_CON3, 0xff, 0x88);
	regmap_update_bits(regmap, MT6369_DRV_CON4, 0xff, 0x88);
	regmap_update_bits(regmap, MT6369_DRV_CON5, 0xff, 0x88);
	regmap_update_bits(regmap, MT6369_DRV_CON6, 0xff, 0x88);

	/* Audio GPIO defaults. */
	regmap_write(regmap, MT6369_GPIO_MODE2, 0x3f);
	regmap_update_bits(regmap, MT6369_GPIO_DIR0, 0xc, 0x0);
	regmap_write(regmap, MT6369_GPIO_MODE3, 0x3f);
	regmap_update_bits(regmap, MT6369_GPIO_DIR0, 0xc0, 0x0);

	regmap_update_bits(regmap, MT6369_AUD_TOP_CKPDN_CON0, 0x4, 0x0);
	regmap_update_bits(regmap, MT6369_AUD_TOP_CKPDN_CON0, 0x20, 0x0);

	mt6369_zcd_enable(priv, false, 5);

	regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON14, 0x40, 0x40);
}

static int mt6369_codec_probe(struct snd_soc_component *cmpnt)
{
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);

	snd_soc_component_init_regmap(cmpnt, priv->regmap);
	mt6369_codec_init_reg(priv);

	return 0;
}

static const struct snd_soc_dapm_widget mt6369_dapm_widgets[] = {
	/* Digital downlink from the AFE over the MTKAIF serial link. */
	SND_SOC_DAPM_AIF_IN("AIF_RX", "mt6369-snd-codec-aif1 Playback", 0,
			    SND_SOC_NOPM, 0, 0),

	/*
	 * TODO: downlink DAC and its clock / negative-charge-pump supplies
	 * (mt_sdm_event / mt_ncp_event / mt_dl_gpio_event) still to be ported;
	 * until then the receiver has no audio data and stays silent.
	 */

	/* Analog earpiece (receiver) output. */
	SND_SOC_DAPM_OUT_DRV_E("RCV", SND_SOC_NOPM, 0, 0, NULL, 0, mt_rcv_event,
			       SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_OUTPUT("Receiver"),
};

static const struct snd_soc_dapm_route mt6369_dapm_routes[] = {
	{"RCV", NULL, "AIF_RX"},
	{"Receiver", NULL, "RCV"},
};

static const struct snd_soc_component_driver mt6369_soc_component_driver = {
	.probe			= mt6369_codec_probe,
	.dapm_widgets		= mt6369_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(mt6369_dapm_widgets),
	.dapm_routes		= mt6369_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(mt6369_dapm_routes),
	.endianness		= 1,
};

static struct snd_soc_dai_driver mt6369_dai_driver[] = {
	{
		.name = "mt6369-snd-codec-aif1",
		.playback = {
			.stream_name	= "mt6369-snd-codec-aif1 Playback",
			.channels_min	= 1,
			.channels_max	= 2,
			.rates		= SNDRV_PCM_RATE_8000_192000,
			.formats	= SNDRV_PCM_FMTBIT_S16_LE |
					  SNDRV_PCM_FMTBIT_S24_LE |
					  SNDRV_PCM_FMTBIT_S32_LE,
		},
	},
};

static int mt6369_platform_driver_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mt6369_priv *priv;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	priv->regmap = dev_get_regmap(dev->parent, NULL);
	if (!priv->regmap)
		return dev_err_probe(dev, -ENODEV, "failed to get parent regmap\n");

	platform_set_drvdata(pdev, priv);

	return devm_snd_soc_register_component(dev, &mt6369_soc_component_driver,
					       mt6369_dai_driver,
					       ARRAY_SIZE(mt6369_dai_driver));
}

static const struct of_device_id mt6369_of_match[] = {
	{ .compatible = "mediatek,mt6369-sound" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mt6369_of_match);

static struct platform_driver mt6369_platform_driver = {
	.driver = {
		.name = "mt6369-sound",
		.of_match_table = mt6369_of_match,
	},
	.probe = mt6369_platform_driver_probe,
};
module_platform_driver(mt6369_platform_driver);

MODULE_DESCRIPTION("MediaTek MT6369 ALSA SoC codec driver");
MODULE_AUTHOR("Jan Benedikt <karagos01@gmail.com>");
MODULE_LICENSE("GPL");
