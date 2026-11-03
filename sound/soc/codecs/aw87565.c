// SPDX-License-Identifier: GPL-2.0
//
// aw87565.c  --  Awinic AW87565 boosted audio power amplifier
//
// Minimal driver for the analog AW87565 speaker PA: it takes the codec's
// analog output and drives the loudspeaker. The amplifier is configured from
// the register profile the stock firmware ships for this board (extracted from
// the vendor aw87xxx_acf.bin) and enabled/disabled from a DAPM widget. The
// vendor's run-time profile switching and smart-PA calibration are omitted.
//
// Copyright (c) 2026 Jan Benedikt <karagos01@gmail.com>

#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#define AW87565_REG_CHIPID	0x00
#define AW87565_REG_SYSCTRL	0x01
#define AW87565_CHIP_ID		0xc2
#define AW87565_POWER_DOWN	0x00
#define AW87565_REG_MAX		0x7f

struct aw87565 {
	struct regmap *regmap;
	struct gpio_desc *reset;
};

static const struct regmap_config aw87565_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = AW87565_REG_MAX,
};

/*
 * Loudspeaker register profile, applied verbatim in the vendor order (SYSCTRL
 * first). These are the stock aw87xxx_acf.bin values for this board, so they
 * carry the manufacturer's boost, gain and protection settings for the fitted
 * speaker; writing SYSCTRL = 0 powers the amplifier back down.
 */
/*
 * "Music" (loudspeaker) scene, recovered from the stock aw87xxx_acf.bin AND
 * verified against the live register dump of the stock amp during speaker
 * playback (/sys/.../6-0058/reg, profile ">Music"). SYSCTRL (0x01) is written
 * 0x80 first to prepare, then the config run, then 0xbc last to engage the
 * class-D PA (the stock running state reads 0x01 = 0xbc; the ACF table alone
 * ends at 0xbe, which the vendor monitor then trims to 0xbc). POST_PMD writes
 * SYSCTRL = 0 to power the amplifier back down.
 */
static const struct reg_sequence aw87565_spk_profile[] = {
	{ 0x01, 0x80 }, { 0x21, 0x6b }, { 0x7e, 0xd2 }, { 0x28, 0x00 },
	{ 0x02, 0x33 }, { 0x03, 0x29 }, { 0x04, 0xfc }, { 0x05, 0x45 },
	{ 0x06, 0xc9 }, { 0x07, 0x1a }, { 0x08, 0x25 }, { 0x09, 0x4e },
	{ 0x0a, 0x7e }, { 0x0b, 0x2b }, { 0x0c, 0x25 }, { 0x0d, 0xa8 },
	{ 0x0e, 0x05 }, { 0x0f, 0x34 }, { 0x10, 0x90 }, { 0x11, 0x50 },
	{ 0x12, 0x63 }, { 0x13, 0x3f }, { 0x14, 0x41 }, { 0x15, 0x09 },
	{ 0x16, 0x53 }, { 0x17, 0x51 }, { 0x18, 0x5d }, { 0x19, 0xcb },
	{ 0x1a, 0x87 }, { 0x1b, 0x13 }, { 0x1c, 0x89 }, { 0x1d, 0x48 },
	{ 0x1e, 0x1c }, { 0x1f, 0xc4 }, { 0x20, 0x0f }, { 0x22, 0x8d },
	{ 0x23, 0x69 }, { 0x29, 0x00 }, { 0x2a, 0x00 }, { 0x2b, 0x00 },
	{ 0x2c, 0x00 }, { 0x2d, 0x2e }, { 0x2e, 0x06 }, { 0x2f, 0x10 },
	{ 0x30, 0x00 }, { 0x3e, 0x00 }, { 0x3f, 0x00 }, { 0x50, 0x0c },
	{ 0x51, 0x00 }, { 0x7f, 0x00 }, { 0x01, 0xbc },
};

static int aw87565_drv_event(struct snd_soc_dapm_widget *w,
			     struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *comp = snd_soc_dapm_to_component(w->dapm);
	struct aw87565 *aw = snd_soc_component_get_drvdata(comp);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		return regmap_multi_reg_write(aw->regmap, aw87565_spk_profile,
					      ARRAY_SIZE(aw87565_spk_profile));
	case SND_SOC_DAPM_POST_PMD:
		return regmap_write(aw->regmap, AW87565_REG_SYSCTRL,
				    AW87565_POWER_DOWN);
	}

	return 0;
}

static const struct snd_soc_dapm_widget aw87565_dapm_widgets[] = {
	SND_SOC_DAPM_INPUT("IN"),
	SND_SOC_DAPM_PGA_E("SPK PA", SND_SOC_NOPM, 0, 0, NULL, 0,
			   aw87565_drv_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_OUTPUT("OUT"),
};

static const struct snd_soc_dapm_route aw87565_dapm_routes[] = {
	{ "SPK PA", NULL, "IN" },
	{ "OUT", NULL, "SPK PA" },
};

static const struct snd_soc_component_driver aw87565_component_driver = {
	.dapm_widgets		= aw87565_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(aw87565_dapm_widgets),
	.dapm_routes		= aw87565_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(aw87565_dapm_routes),
};

static int aw87565_i2c_probe(struct i2c_client *i2c)
{
	struct aw87565 *aw;
	unsigned int id;
	int ret;

	aw = devm_kzalloc(&i2c->dev, sizeof(*aw), GFP_KERNEL);
	if (!aw)
		return -ENOMEM;
	i2c_set_clientdata(i2c, aw);

	/* Optional hardware reset, released (deasserted) to bring the PA up. */
	aw->reset = devm_gpiod_get_optional(&i2c->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(aw->reset))
		return dev_err_probe(&i2c->dev, PTR_ERR(aw->reset),
				     "failed to get reset gpio\n");
	if (aw->reset) {
		gpiod_set_value_cansleep(aw->reset, 0);
		usleep_range(2000, 2200);
	}

	aw->regmap = devm_regmap_init_i2c(i2c, &aw87565_regmap_config);
	if (IS_ERR(aw->regmap))
		return dev_err_probe(&i2c->dev, PTR_ERR(aw->regmap),
				     "failed to init regmap\n");

	ret = regmap_read(aw->regmap, AW87565_REG_CHIPID, &id);
	if (ret)
		return ret;
	if (id != AW87565_CHIP_ID)
		return dev_err_probe(&i2c->dev, -ENODEV,
				     "unexpected chip id 0x%02x\n", id);

	/* Leave the amplifier powered down until the output path is active. */
	regmap_write(aw->regmap, AW87565_REG_SYSCTRL, AW87565_POWER_DOWN);

	return devm_snd_soc_register_component(&i2c->dev,
					       &aw87565_component_driver,
					       NULL, 0);
}

static const struct of_device_id aw87565_of_match[] = {
	{ .compatible = "awinic,aw87565" },
	{ }
};
MODULE_DEVICE_TABLE(of, aw87565_of_match);

static const struct i2c_device_id aw87565_i2c_id[] = {
	{ "aw87565", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, aw87565_i2c_id);

static struct i2c_driver aw87565_i2c_driver = {
	.driver = {
		.name = "aw87565",
		.of_match_table = aw87565_of_match,
	},
	.probe = aw87565_i2c_probe,
	.id_table = aw87565_i2c_id,
};
module_i2c_driver(aw87565_i2c_driver);

MODULE_DESCRIPTION("Awinic AW87565 speaker amplifier driver");
MODULE_AUTHOR("Jan Benedikt <karagos01@gmail.com>");
MODULE_LICENSE("GPL");
