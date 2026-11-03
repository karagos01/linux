// SPDX-License-Identifier: GPL-2.0
//
// rt5512.c  --  Richtek RT5512 I2S boosted Class-D speaker amplifier
//
// Minimal ASoC codec driver: brings the amplifier up as an I2S slave sink so
// an external I2S master (the MT6878 AFE ETDM_OUT4) can drive the speaker.
// The vendor smart-PA calibration path (V/I sense feedback, the MediaTek
// speaker-amp DSP class) is intentionally omitted.
//
// Copyright (c) 2026 Jan Benedikt <karagos01@gmail.com>

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#define RT5512_REG_DEVID		0x00
#define RT5512_REG_SYSTEM_CTRL		0x03
#define RT5512_REG_SERIAL_CFG1		0x10
#define RT5512_REG_TDM_CFG3		0x15
#define RT5512_REG_PLL_CFG1		0x60
#define RT5512_REG_MISC_CTRL1		0x98
#define RT5512_REG_ANA_TOP_CTRL0	0xb5

/* RT5512_REG_SYSTEM_CTRL bits */
#define RT5512_SYSTEM_CTRL_PWR_OFF	BIT(0)
#define RT5512_SYSTEM_CTRL_MUTE		BIT(1)
#define RT5512_SYSTEM_CTRL_SW_RESET	BIT(7)

/* RT5512_REG_SERIAL_CFG1: audio bit width select */
#define RT5512_SERIAL_CFG1_AUDBIT_MASK	GENMASK(7, 6)
#define RT5512_SERIAL_CFG1_AUDBIT_SHIFT	6

/* RT5512_REG_TDM_CFG3: channel word length */
#define RT5512_TDM_CFG3_WORDLEN_MASK	GENMASK(9, 4)
#define RT5512_TDM_CFG3_WORDLEN_SHIFT	4

#define RT5512_CHIP_REV_A		2
#define RT5512_CHIP_REV_B		3

struct rt5512 {
	struct device *dev;
	struct regmap *regmap;
	unsigned int chip_rev;
	u8 bst_mode;
};

static const struct regmap_config rt5512_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = 0xff,
};

/*
 * Analog tuning table applied once at component probe. These are the vendor's
 * boost-converter, Class-D, protection and V/I-sense settings (identical for
 * chip revisions A and B); the amplifier must not be driven without them, as
 * the boost stage comes out of reset unconfigured.
 */
static const struct reg_sequence rt5512_init_regs[] = {
	{ 0x69, 0x0002 }, { 0x68, 0x000d }, { 0x6c, 0x0010 }, { 0x6d, 0x0008 },
	{ 0x30, 0x0002 }, { 0xa7, 0x0a84 }, { 0x20, 0x00a2 }, { 0x8b, 0x0040 },
	/* boost efficiency */
	{ 0xad, 0x40f7 }, { 0x41, 0x0028 }, { 0x49, 0x0495 },
	{ 0x46, 0x001d }, { 0x45, 0x5292 }, { 0x4c, 0x0293 },
	{ 0xa2, 0x355d }, { 0xa5, 0x6612 }, { 0x70, 0x0021 }, { 0xa6, 0x3135 },
	/* boost THD enhance */
	{ 0x9b, 0x5f37 }, { 0x4a, 0xd755 },
	/* V/I sense enhance */
	{ 0xb3, 0x9103 }, { 0xb0, 0xd5a5 }, { 0x98, 0x8b8c }, { 0x78, 0x00f2 },
	{ 0x9d, 0x00fc },
	{ 0x38, 0x9beb }, { 0x39, 0x8bac }, { 0x3a, 0x7e7d }, { 0x3b, 0x7395 },
	{ 0x3c, 0x6a68 }, { 0x3d, 0x6295 }, { 0x3e, 0x5bd4 },
};

/* The few init entries that are read-modify-write rather than full writes. */
static int rt5512_apply_init(struct snd_soc_component *comp)
{
	int ret;

	ret = snd_soc_component_update_bits(comp, 0xa1, 0xff18, 0x5b18);
	if (ret < 0)
		return ret;
	ret = regmap_multi_reg_write(dev_get_regmap(comp->dev, NULL),
				     rt5512_init_regs,
				     ARRAY_SIZE(rt5512_init_regs));
	if (ret < 0)
		return ret;
	ret = snd_soc_component_update_bits(comp, 0xae, 0x00ff, 0x0056);
	ret |= snd_soc_component_update_bits(comp, 0xb1, 0xfff0, 0xa5aa);
	ret |= snd_soc_component_update_bits(comp, 0x12, 0x0007, 0x0006);
	ret |= snd_soc_component_update_bits(comp, 0xb6, 0x0400, 0x0000);
	return ret < 0 ? ret : 0;
}

static int rt5512_dac_event(struct snd_soc_dapm_widget *w,
			    struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *comp = snd_soc_dapm_to_component(w->dapm);

	/* un-mute once the DAC/PLL is up */
	if (event == SND_SOC_DAPM_POST_PMU)
		return snd_soc_component_update_bits(comp, RT5512_REG_SYSTEM_CTRL,
						     RT5512_SYSTEM_CTRL_MUTE, 0);
	return 0;
}

static int rt5512_classd_event(struct snd_soc_dapm_widget *w,
			       struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *comp = snd_soc_dapm_to_component(w->dapm);
	struct rt5512 *rt5512 = snd_soc_component_get_drvdata(comp);
	int ret = 0;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		mdelay(11);
		/* charge pump off, disable UVP while ramping */
		ret |= snd_soc_component_update_bits(comp, 0xb5, 0xf9fc, 0xf9fc);
		mdelay(2);
		/* boost mode */
		ret |= snd_soc_component_update_bits(comp, 0x40, 0x0003,
						     rt5512->bst_mode);
		ret |= snd_soc_component_update_bits(comp, 0x98, 0x0700, 0x0100);
		mdelay(2);
		/* charge pump enable */
		ret |= snd_soc_component_update_bits(comp, 0xb5, 0x0001, 0x0001);
		break;
	case SND_SOC_DAPM_POST_PMU:
		mdelay(15);
		ret |= snd_soc_component_update_bits(comp, 0x98, 0x0200, 0x0200);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		/* mute, drop headroom */
		ret |= snd_soc_component_update_bits(comp, RT5512_REG_SYSTEM_CTRL,
						     RT5512_SYSTEM_CTRL_MUTE,
						     RT5512_SYSTEM_CTRL_MUTE);
		ret |= snd_soc_component_update_bits(comp, 0x41, 0x00ff, 0x002f);
		ret |= snd_soc_component_update_bits(comp, 0x98, 0x0010, 0x0010);
		break;
	case SND_SOC_DAPM_POST_PMD:
		ret |= snd_soc_component_update_bits(comp, RT5512_REG_SYSTEM_CTRL,
						     RT5512_SYSTEM_CTRL_MUTE, 0);
		mdelay(2);
		/* charge pump + boost + bias off */
		ret |= snd_soc_component_update_bits(comp, 0xb5, 0x0001, 0x0000);
		mdelay(1);
		ret |= snd_soc_component_update_bits(comp, 0x40, 0x0003, 0x0000);
		ret |= snd_soc_component_update_bits(comp, 0xb5, 0xfffe, 0x0000);
		break;
	}
	return ret < 0 ? ret : 0;
}

static const struct snd_soc_dapm_widget rt5512_dapm_widgets[] = {
	SND_SOC_DAPM_DAC_E("DAC", NULL, RT5512_REG_PLL_CFG1, 0, 1,
			   rt5512_dac_event, SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_PGA("PGA", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_OUT_DRV_E("ClassD", RT5512_REG_SYSTEM_CTRL, 2, 0, NULL, 0,
			       rt5512_classd_event,
			       SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			       SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SPK("SPK", NULL),
};

static const struct snd_soc_dapm_route rt5512_dapm_routes[] = {
	{ "DAC", NULL, "AIF1 Playback" },
	{ "PGA", NULL, "DAC" },
	{ "ClassD", NULL, "PGA" },
	{ "SPK", NULL, "ClassD" },
};

static int rt5512_component_probe(struct snd_soc_component *comp)
{
	struct rt5512 *rt5512 = snd_soc_component_get_drvdata(comp);
	int ret;

	snd_soc_component_init_regmap(comp, rt5512->regmap);

	/* leave low-power mode so the analog tuning writes take effect */
	ret = snd_soc_component_update_bits(comp, RT5512_REG_SYSTEM_CTRL,
					    RT5512_SYSTEM_CTRL_PWR_OFF, 0);
	if (ret < 0)
		return ret;

	ret = rt5512_apply_init(comp);
	if (ret < 0)
		dev_err(rt5512->dev, "failed to apply init settings: %d\n", ret);
	return ret;
}

static const struct snd_soc_component_driver rt5512_component_driver = {
	.probe			= rt5512_component_probe,
	.dapm_widgets		= rt5512_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(rt5512_dapm_widgets),
	.dapm_routes		= rt5512_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(rt5512_dapm_routes),
	.idle_bias_on		= 1,
	.endianness		= 1,
};

static int rt5512_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_dapm_context *dapm =
		snd_soc_component_to_dapm(dai->component);
	int word_len = params_physical_width(params);
	int aud_bit = params_width(params);
	unsigned int audbit_sel;
	int ret;

	if (word_len < 16 || word_len > 32)
		return -EINVAL;

	switch (aud_bit) {
	case 16:
		audbit_sel = 3;
		break;
	case 18:
		audbit_sel = 2;
		break;
	case 20:
		audbit_sel = 1;
		break;
	case 24:
	case 32:
		audbit_sel = 0;
		break;
	default:
		return -EINVAL;
	}

	ret = snd_soc_component_update_bits(dai->component,
		RT5512_REG_SERIAL_CFG1, RT5512_SERIAL_CFG1_AUDBIT_MASK,
		audbit_sel << RT5512_SERIAL_CFG1_AUDBIT_SHIFT);
	if (ret < 0)
		return ret;

	ret = snd_soc_component_update_bits(dai->component,
		RT5512_REG_TDM_CFG3, RT5512_TDM_CFG3_WORDLEN_MASK,
		word_len << RT5512_TDM_CFG3_WORDLEN_SHIFT);
	if (ret < 0)
		return ret;

	return snd_soc_dapm_enable_pin(dapm, "SPK");
}

static int rt5512_hw_free(struct snd_pcm_substream *substream,
			  struct snd_soc_dai *dai)
{
	struct snd_soc_dapm_context *dapm =
		snd_soc_component_to_dapm(dai->component);
	int ret;

	ret = snd_soc_dapm_disable_pin(dapm, "SPK");
	if (ret < 0)
		return ret;
	return snd_soc_dapm_sync(dapm);
}

static const struct snd_soc_dai_ops rt5512_dai_ops = {
	.hw_params	= rt5512_hw_params,
	.hw_free	= rt5512_hw_free,
};

#define RT5512_RATES	SNDRV_PCM_RATE_8000_192000
#define RT5512_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | \
			 SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver rt5512_dai = {
	.name = "rt5512-aif",
	.playback = {
		.stream_name	= "AIF1 Playback",
		.channels_min	= 1,
		.channels_max	= 2,
		.rates		= RT5512_RATES,
		.formats	= RT5512_FORMATS,
	},
	.symmetric_rate		= 1,
	.symmetric_channels	= 1,
	.symmetric_sample_bits	= 1,
	.ops			= &rt5512_dai_ops,
};

static int rt5512_sw_reset(struct rt5512 *rt5512)
{
	int ret;

	ret = regmap_write(rt5512->regmap, RT5512_REG_SYSTEM_CTRL, 0x0000);
	if (ret)
		return ret;
	ret = regmap_write(rt5512->regmap, RT5512_REG_SYSTEM_CTRL,
			   RT5512_SYSTEM_CTRL_SW_RESET);
	if (ret)
		return ret;
	msleep(15);

	if (rt5512->chip_rev == RT5512_CHIP_REV_B) {
		ret = regmap_write(rt5512->regmap, RT5512_REG_MISC_CTRL1, 0x0000);
		if (ret)
			return ret;
		msleep(10);
		ret = regmap_write(rt5512->regmap, RT5512_REG_ANA_TOP_CTRL0,
				   0xe1fd);
		if (ret)
			return ret;
	}
	return 0;
}

static int rt5512_i2c_probe(struct i2c_client *i2c)
{
	struct rt5512 *rt5512;
	unsigned int val;
	int ret;

	rt5512 = devm_kzalloc(&i2c->dev, sizeof(*rt5512), GFP_KERNEL);
	if (!rt5512)
		return -ENOMEM;

	rt5512->dev = &i2c->dev;
	rt5512->bst_mode = 1;		/* adaptive/battery boost */
	i2c_set_clientdata(i2c, rt5512);

	rt5512->regmap = devm_regmap_init_i2c(i2c, &rt5512_regmap_config);
	if (IS_ERR(rt5512->regmap))
		return dev_err_probe(&i2c->dev, PTR_ERR(rt5512->regmap),
				     "failed to init regmap\n");

	/* wake the digital core, then read the device/revision id */
	ret = regmap_write(rt5512->regmap, RT5512_REG_SYSTEM_CTRL, 0x0000);
	if (ret)
		return dev_err_probe(&i2c->dev, ret, "device not responding\n");

	ret = regmap_read(rt5512->regmap, RT5512_REG_DEVID, &val);
	if (ret)
		return ret;
	if (val >> 8)
		return dev_err_probe(&i2c->dev, -ENODEV,
				     "unexpected device id 0x%04x\n", val);

	rt5512->chip_rev = val & 0xff;
	if (rt5512->chip_rev != RT5512_CHIP_REV_A &&
	    rt5512->chip_rev != RT5512_CHIP_REV_B)
		return dev_err_probe(&i2c->dev, -ENODEV,
				     "unsupported chip revision %u\n",
				     rt5512->chip_rev);
	dev_info(&i2c->dev, "RT5512 rev %c\n",
		 'A' + rt5512->chip_rev - RT5512_CHIP_REV_A);

	ret = rt5512_sw_reset(rt5512);
	if (ret)
		return dev_err_probe(&i2c->dev, ret, "reset failed\n");

	return devm_snd_soc_register_component(&i2c->dev,
					       &rt5512_component_driver,
					       &rt5512_dai, 1);
}

static const struct of_device_id rt5512_of_match[] = {
	{ .compatible = "richtek,rt5512" },
	{ }
};
MODULE_DEVICE_TABLE(of, rt5512_of_match);

static const struct i2c_device_id rt5512_i2c_id[] = {
	{ "rt5512", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, rt5512_i2c_id);

static struct i2c_driver rt5512_i2c_driver = {
	.driver = {
		.name = "rt5512",
		.of_match_table = rt5512_of_match,
	},
	.probe = rt5512_i2c_probe,
	.id_table = rt5512_i2c_id,
};
module_i2c_driver(rt5512_i2c_driver);

MODULE_DESCRIPTION("Richtek RT5512 speaker amplifier driver");
MODULE_AUTHOR("Jan Benedikt <karagos01@gmail.com>");
MODULE_LICENSE("GPL");
