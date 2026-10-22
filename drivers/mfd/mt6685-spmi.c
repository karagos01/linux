// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek MT6685 SPMI PMIC (clock buffer, RTC) core driver
 *
 * Registers the register map of the PMIC on the SPMI bus and instantiates
 * the function blocks described as child nodes, which share it.
 */

#include <linux/delay.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>
#include <linux/spmi.h>

#define MT6685_HWCID_L		0x8

static const struct regmap_config mt6685_regmap_config = {
	.reg_bits	= 16,
	.val_bits	= 8,
	.max_register	= 0x2000,
	.fast_io	= true,
};

static int mt6685_spmi_probe(struct spmi_device *sdev)
{
	struct device *dev = &sdev->dev;
	struct regmap *regmap;
	unsigned int id;
	int i, ret;

	regmap = devm_regmap_init_spmi_ext(sdev, &mt6685_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	/*
	 * The PMIC may be asleep and the first accesses only wake it up;
	 * it can take a few of them before it answers.
	 */
	for (i = 0; i < 5; i++) {
		ret = regmap_read(regmap, MT6685_HWCID_L, &id);
		if (!ret)
			break;
		usleep_range(1000, 2000);
	}
	/* Early in boot it may still not answer: try again later */
	if (ret)
		return dev_err_probe(dev, -EPROBE_DEFER, "Cannot read chip ID (%d)\n", ret);

	dev_dbg(dev, "chip ID: 0x%02x\n", id);

	return devm_of_platform_populate(dev);
}

static const struct of_device_id mt6685_spmi_match[] = {
	{ .compatible = "mediatek,mt6685" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mt6685_spmi_match);

static struct spmi_driver mt6685_spmi_driver = {
	.driver = {
		.name = "mt6685-spmi",
		.of_match_table = mt6685_spmi_match,
	},
	.probe = mt6685_spmi_probe,
};
module_spmi_driver(mt6685_spmi_driver);

MODULE_DESCRIPTION("MediaTek MT6685 SPMI PMIC core driver");
MODULE_LICENSE("GPL");
