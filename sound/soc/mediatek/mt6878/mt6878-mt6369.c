// SPDX-License-Identifier: GPL-2.0
//
// mt6878-mt6369.c  --  MT6878 MT6369 ALSA SoC machine driver
//
// Copyright (c) 2026 Jan Benedikt <karagos01@gmail.com>

#include <linux/module.h>
#include <sound/soc.h>

#include "mt6878-afe-common.h"

SND_SOC_DAILINK_DEFS(playback_0,
	DAILINK_COMP_ARRAY(COMP_CPU("DL0")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(primary_codec,
	DAILINK_COMP_ARRAY(COMP_CPU("ADDA")),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "mt6369-snd-codec-aif1")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

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
	/* back end */
	{
		.name = "Primary Codec",
		.no_pcm = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(primary_codec),
	},
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
	int ret, i;

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

	ret = devm_snd_soc_register_card(&pdev->dev, card);

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
