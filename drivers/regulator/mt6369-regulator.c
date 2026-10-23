// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek MT6369 SPMI PMIC regulator driver
 *
 * Only the LDOs feeding the connectivity subsystem and the SD card slot are
 * described for now.
 * Register layout taken from the MediaTek downstream driver.
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>
#include <linux/spmi.h>

#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>

#define MT6369_SWCID_H			0x0b

#define MT6369_LDO_VCN33_1_CON0		0x1c3f
#define MT6369_LDO_VCN33_2_CON0		0x1c4d
#define MT6369_LDO_VMCH_CON0		0x1c95
#define MT6369_LDO_VMCH_EINT		0x1ca3
#define MT6369_LDO_VMC_CON0		0x1ca4
#define MT6369_LDO_VANT18_CON0		0x1cb2
#define MT6369_VCN33_1_ANA_CON1		0x1db1
#define MT6369_VCN33_2_ANA_CON1		0x1db5
#define MT6369_VMCH_ANA_CON1		0x1da9
#define MT6369_VMC_ANA_CON1		0x1dad
#define MT6369_VANT18_ANA_CON1		0x1e09

#define MT6369_LDO_EN			BIT(0)
#define MT6369_LDO_VOSEL		GENMASK(3, 0)
#define MT6369_VMCH_EINT_EN		BIT(0)
#define MT6369_VMCH_EINT_POL		BIT(2)

enum mt6369_regulator_id {
	MT6369_ID_VCN33_1,
	MT6369_ID_VCN33_2,
	MT6369_ID_VANT18,
	MT6369_ID_VMCH,
	MT6369_ID_VMC,
};

/* Shared by VCN33_1, VCN33_2, VMCH and VMC */
static const unsigned int mt6369_vcn33_volts[] = {
	1200000, 1300000, 1500000, 1700000, 1800000, 2000000, 2500000, 2600000,
	2700000, 2800000, 2900000, 3000000, 3100000, 3300000, 3400000, 3500000,
};

static const unsigned int mt6369_vant18_volts[] = {
	600000, 700000, 800000, 900000, 1000000, 1100000, 1200000, 1300000,
	1400000, 1500000, 1600000, 1700000, 1800000, 1900000, 2000000, 2100000,
};

static const struct regulator_ops mt6369_ldo_ops = {
	.list_voltage = regulator_list_voltage_table,
	.map_voltage = regulator_map_voltage_iterate,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
};

/*
 * VMCH feeds the SD card slot. Like the stock kernel, gate it with the card
 * detect EINT of the PMIC: the LDO is on while the EINT line is high.
 */
static int mt6369_vmch_enable(struct regulator_dev *rdev)
{
	int ret;

	ret = regmap_update_bits(rdev->regmap, MT6369_LDO_VMCH_EINT,
				 MT6369_VMCH_EINT_POL, MT6369_VMCH_EINT_POL);
	if (ret)
		return ret;

	ret = regmap_set_bits(rdev->regmap, MT6369_LDO_VMCH_CON0, MT6369_LDO_EN);
	if (ret)
		return ret;

	return regmap_set_bits(rdev->regmap, MT6369_LDO_VMCH_EINT, MT6369_VMCH_EINT_EN);
}

static int mt6369_vmch_disable(struct regulator_dev *rdev)
{
	int ret;

	ret = regmap_clear_bits(rdev->regmap, MT6369_LDO_VMCH_CON0, MT6369_LDO_EN);
	if (ret)
		return ret;

	/* Let VMCH discharge */
	udelay(1500);

	return regmap_clear_bits(rdev->regmap, MT6369_LDO_VMCH_EINT, MT6369_VMCH_EINT_EN);
}

static const struct regulator_ops mt6369_vmch_ops = {
	.list_voltage = regulator_list_voltage_table,
	.map_voltage = regulator_map_voltage_iterate,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.enable = mt6369_vmch_enable,
	.disable = mt6369_vmch_disable,
	.is_enabled = regulator_is_enabled_regmap,
};

#define MT6369_LDO(_id, _name, _volts, _en_reg, _vsel_reg)	\
[_id] = {							\
	.name = _name,						\
	.of_match = of_match_ptr(_name),			\
	.regulators_node = of_match_ptr("regulators"),		\
	.ops = &mt6369_ldo_ops,					\
	.type = REGULATOR_VOLTAGE,				\
	.id = _id,						\
	.owner = THIS_MODULE,					\
	.n_voltages = ARRAY_SIZE(_volts),			\
	.volt_table = _volts,					\
	.enable_reg = _en_reg,					\
	.enable_mask = MT6369_LDO_EN,				\
	.vsel_reg = _vsel_reg,					\
	.vsel_mask = MT6369_LDO_VOSEL,				\
	.enable_time = 240,					\
}

static const struct regulator_desc mt6369_regulators[] = {
	MT6369_LDO(MT6369_ID_VCN33_1, "vcn33-1", mt6369_vcn33_volts,
		   MT6369_LDO_VCN33_1_CON0, MT6369_VCN33_1_ANA_CON1),
	MT6369_LDO(MT6369_ID_VCN33_2, "vcn33-2", mt6369_vcn33_volts,
		   MT6369_LDO_VCN33_2_CON0, MT6369_VCN33_2_ANA_CON1),
	MT6369_LDO(MT6369_ID_VANT18, "vant18", mt6369_vant18_volts,
		   MT6369_LDO_VANT18_CON0, MT6369_VANT18_ANA_CON1),
	[MT6369_ID_VMCH] = {
		.name = "vmch",
		.of_match = of_match_ptr("vmch"),
		.regulators_node = of_match_ptr("regulators"),
		.ops = &mt6369_vmch_ops,
		.type = REGULATOR_VOLTAGE,
		.id = MT6369_ID_VMCH,
		.owner = THIS_MODULE,
		.n_voltages = ARRAY_SIZE(mt6369_vcn33_volts),
		.volt_table = mt6369_vcn33_volts,
		.enable_reg = MT6369_LDO_VMCH_EINT,
		.enable_mask = MT6369_VMCH_EINT_EN,
		.vsel_reg = MT6369_VMCH_ANA_CON1,
		.vsel_mask = MT6369_LDO_VOSEL,
		.enable_time = 240,
	},
	MT6369_LDO(MT6369_ID_VMC, "vmc", mt6369_vcn33_volts,
		   MT6369_LDO_VMC_CON0, MT6369_VMC_ANA_CON1),
};

static const struct regmap_config mt6369_spmi_regmap_config = {
	.reg_bits	= 16,
	.val_bits	= 8,
	.max_register	= 0x2000,
	.fast_io	= true,
};

static int mt6369_regulator_probe(struct spmi_device *sdev)
{
	struct regulator_config config = {};
	struct regulator_dev *rdev;
	unsigned int i, val;
	int ret;

	config.regmap = devm_regmap_init_spmi_ext(sdev, &mt6369_spmi_regmap_config);
	if (IS_ERR(config.regmap))
		return PTR_ERR(config.regmap);

	/* The PMIC may be asleep: the first access only wakes it up */
	regmap_read(config.regmap, MT6369_SWCID_H, &val);
	ret = regmap_read(config.regmap, MT6369_SWCID_H, &val);
	if (ret)
		return dev_err_probe(&sdev->dev, ret, "Cannot read chip ID\n");

	config.dev = &sdev->dev;

	for (i = 0; i < ARRAY_SIZE(mt6369_regulators); i++) {
		rdev = devm_regulator_register(&sdev->dev, &mt6369_regulators[i], &config);
		if (IS_ERR(rdev))
			return dev_err_probe(&sdev->dev, PTR_ERR(rdev), "Failed to register %s\n",
					     mt6369_regulators[i].name);
	}

	/* Function blocks of this PMIC described as child nodes */
	return devm_of_platform_populate(&sdev->dev);
}

static const struct of_device_id mt6369_regulator_match[] = {
	{ .compatible = "mediatek,mt6369-regulator" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mt6369_regulator_match);

static struct spmi_driver mt6369_regulator_driver = {
	.driver = {
		.name = "mt6369-regulator",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = mt6369_regulator_match,
	},
	.probe = mt6369_regulator_probe,
};
module_spmi_driver(mt6369_regulator_driver);

MODULE_DESCRIPTION("MediaTek MT6369 PMIC Regulator Driver");
MODULE_LICENSE("GPL");
