// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6375 switching battery charger driver
 *
 * Copyright (c) 2021 MediaTek Inc.
 *
 * A lean driver for the single-cell switching charger embedded in the MT6375
 * SubPMIC. It exposes charger status, BC1.2 port detection and the input and
 * charge current/voltage limits through the power_supply framework.
 */

#include <linux/bits.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>

#define MT6375_REG_CHG_TOP1	0x120
#define MT6375_REG_CHG_AICR	0x122
#define MT6375_REG_CHG_VCHG	0x125
#define MT6375_REG_CHG_ICHG	0x126
#define MT6375_REG_CHG_STAT	0x134
#define MT6375_REG_BC12_FUNC	0x150
#define MT6375_REG_BC12_STAT	0x151
#define MT6375_REG_CHG_STAT0	0x1E0

/* CHG_STAT[3:0] - IC charging state */
enum {
	MT6375_CHG_STAT_SLEEP,
	MT6375_CHG_STAT_VBUS_RDY,
	MT6375_CHG_STAT_TRICKLE,
	MT6375_CHG_STAT_PRE,
	MT6375_CHG_STAT_FAST,
	MT6375_CHG_STAT_EOC,
	MT6375_CHG_STAT_BKGND,
	MT6375_CHG_STAT_DONE,
	MT6375_CHG_STAT_FAULT,
	MT6375_CHG_STAT_OTG = 15,
};

/* BC12_STAT[3:0] - detected USB port type */
enum {
	MT6375_PORT_NOINFO,
	MT6375_PORT_APPLE_10W = 8,
	MT6375_PORT_SAMSUNG,
	MT6375_PORT_APPLE_5W,
	MT6375_PORT_APPLE_12W,
	MT6375_PORT_UNKNOWN_TA,
	MT6375_PORT_SDP,
	MT6375_PORT_CDP,
	MT6375_PORT_DCP,
};

enum mt6375_chg_field {
	F_CHG_EN,
	F_AICR,
	F_CV,
	F_ICHG,
	F_IC_STAT,
	F_BC12_EN,
	F_PORT_STAT,
	F_ONLINE,
	F_MAX,
};

static const struct reg_field mt6375_chg_fields[F_MAX] = {
	[F_CHG_EN]	= REG_FIELD(MT6375_REG_CHG_TOP1, 0, 0),
	[F_AICR]	= REG_FIELD(MT6375_REG_CHG_AICR, 0, 6),
	[F_CV]		= REG_FIELD(MT6375_REG_CHG_VCHG, 0, 6),
	[F_ICHG]	= REG_FIELD(MT6375_REG_CHG_ICHG, 0, 5),
	[F_IC_STAT]	= REG_FIELD(MT6375_REG_CHG_STAT, 0, 3),
	[F_BC12_EN]	= REG_FIELD(MT6375_REG_BC12_FUNC, 7, 7),
	[F_PORT_STAT]	= REG_FIELD(MT6375_REG_BC12_STAT, 0, 3),
	[F_ONLINE]	= REG_FIELD(MT6375_REG_CHG_STAT0, 0, 0),
};

/* Linear register encodings: value = min + step * (reg - offset), in uA/uV */
#define MT6375_AICR_MIN_UA	100000
#define MT6375_AICR_STEP_UA	25000
#define MT6375_AICR_OFFSET	2
#define MT6375_AICR_MAX_UA	3225000

#define MT6375_ICHG_MIN_UA	300000
#define MT6375_ICHG_STEP_UA	50000
#define MT6375_ICHG_OFFSET	6
#define MT6375_ICHG_MAX_UA	3150000

#define MT6375_CV_MIN_UV	3900000
#define MT6375_CV_STEP_UV	10000
#define MT6375_CV_MAX_UV	4710000

struct mt6375_chg_data {
	struct device *dev;
	struct regmap *regmap;
	struct regmap_field *rm_field[F_MAX];
	struct power_supply *psy;
};

static int mt6375_chg_field_get(struct mt6375_chg_data *ddata,
				enum mt6375_chg_field field, unsigned int *val)
{
	return regmap_field_read(ddata->rm_field[field], val);
}

static int mt6375_chg_field_set(struct mt6375_chg_data *ddata,
				enum mt6375_chg_field field, unsigned int val)
{
	return regmap_field_write(ddata->rm_field[field], val);
}

static int mt6375_chg_get_online(struct mt6375_chg_data *ddata, int *online)
{
	unsigned int val;
	int ret;

	ret = mt6375_chg_field_get(ddata, F_ONLINE, &val);
	if (ret)
		return ret;
	*online = val ? 1 : 0;
	return 0;
}

static int mt6375_chg_get_status(struct mt6375_chg_data *ddata, int *status)
{
	unsigned int stat, chg_en;
	int ret, online;

	ret = mt6375_chg_get_online(ddata, &online);
	if (ret)
		return ret;
	if (!online) {
		*status = POWER_SUPPLY_STATUS_DISCHARGING;
		return 0;
	}

	ret = mt6375_chg_field_get(ddata, F_IC_STAT, &stat);
	if (ret)
		return ret;
	ret = mt6375_chg_field_get(ddata, F_CHG_EN, &chg_en);
	if (ret)
		return ret;

	switch (stat) {
	case MT6375_CHG_STAT_OTG:
		*status = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	case MT6375_CHG_STAT_VBUS_RDY:
	case MT6375_CHG_STAT_TRICKLE:
	case MT6375_CHG_STAT_PRE:
	case MT6375_CHG_STAT_FAST:
	case MT6375_CHG_STAT_EOC:
	case MT6375_CHG_STAT_BKGND:
		*status = chg_en ? POWER_SUPPLY_STATUS_CHARGING :
				   POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case MT6375_CHG_STAT_DONE:
		*status = POWER_SUPPLY_STATUS_FULL;
		break;
	case MT6375_CHG_STAT_SLEEP:
	case MT6375_CHG_STAT_FAULT:
		*status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	default:
		*status = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	}
	return 0;
}

static int mt6375_chg_get_charge_type(struct mt6375_chg_data *ddata, int *type)
{
	unsigned int stat;
	int ret, online;

	ret = mt6375_chg_get_online(ddata, &online);
	if (ret)
		return ret;
	if (!online) {
		*type = POWER_SUPPLY_CHARGE_TYPE_NONE;
		return 0;
	}

	ret = mt6375_chg_field_get(ddata, F_IC_STAT, &stat);
	if (ret)
		return ret;

	switch (stat) {
	case MT6375_CHG_STAT_TRICKLE:
	case MT6375_CHG_STAT_PRE:
		*type = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		break;
	case MT6375_CHG_STAT_FAST:
	case MT6375_CHG_STAT_BKGND:
	case MT6375_CHG_STAT_EOC:
		*type = POWER_SUPPLY_CHARGE_TYPE_STANDARD;
		break;
	default:
		*type = POWER_SUPPLY_CHARGE_TYPE_NONE;
		break;
	}
	return 0;
}

static int mt6375_chg_get_usb_type(struct mt6375_chg_data *ddata, int *type)
{
	unsigned int port;
	int ret;

	ret = mt6375_chg_field_get(ddata, F_PORT_STAT, &port);
	if (ret)
		return ret;

	switch (port) {
	case MT6375_PORT_SDP:
		*type = POWER_SUPPLY_USB_TYPE_SDP;
		break;
	case MT6375_PORT_CDP:
		*type = POWER_SUPPLY_USB_TYPE_CDP;
		break;
	case MT6375_PORT_DCP:
	case MT6375_PORT_APPLE_5W:
	case MT6375_PORT_APPLE_10W:
	case MT6375_PORT_APPLE_12W:
	case MT6375_PORT_SAMSUNG:
	case MT6375_PORT_UNKNOWN_TA:
		*type = POWER_SUPPLY_USB_TYPE_DCP;
		break;
	default:
		*type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		break;
	}
	return 0;
}

static int mt6375_chg_get_value(struct mt6375_chg_data *ddata,
				enum mt6375_chg_field field, int *uval)
{
	unsigned int reg;
	int ret;

	ret = mt6375_chg_field_get(ddata, field, &reg);
	if (ret)
		return ret;

	switch (field) {
	case F_AICR:
		*uval = MT6375_AICR_MIN_UA +
			MT6375_AICR_STEP_UA * (reg - MT6375_AICR_OFFSET);
		break;
	case F_ICHG:
		*uval = MT6375_ICHG_MIN_UA +
			MT6375_ICHG_STEP_UA * (reg - MT6375_ICHG_OFFSET);
		break;
	case F_CV:
		*uval = MT6375_CV_MIN_UV + MT6375_CV_STEP_UV * reg;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int mt6375_chg_set_value(struct mt6375_chg_data *ddata,
				enum mt6375_chg_field field, int uval)
{
	unsigned int reg;

	switch (field) {
	case F_AICR:
		uval = clamp(uval, MT6375_AICR_MIN_UA, MT6375_AICR_MAX_UA);
		reg = (uval - MT6375_AICR_MIN_UA) / MT6375_AICR_STEP_UA +
		      MT6375_AICR_OFFSET;
		break;
	case F_ICHG:
		uval = clamp(uval, MT6375_ICHG_MIN_UA, MT6375_ICHG_MAX_UA);
		reg = (uval - MT6375_ICHG_MIN_UA) / MT6375_ICHG_STEP_UA +
		      MT6375_ICHG_OFFSET;
		break;
	case F_CV:
		uval = clamp(uval, MT6375_CV_MIN_UV, MT6375_CV_MAX_UV);
		reg = (uval - MT6375_CV_MIN_UV) / MT6375_CV_STEP_UV;
		break;
	default:
		return -EINVAL;
	}
	return mt6375_chg_field_set(ddata, field, reg);
}

static int mt6375_chg_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct mt6375_chg_data *ddata = power_supply_get_drvdata(psy);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = mt6375_chg_get_online(ddata, &val->intval);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		ret = mt6375_chg_get_status(ddata, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		ret = mt6375_chg_get_charge_type(ddata, &val->intval);
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		ret = mt6375_chg_get_usb_type(ddata, &val->intval);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = mt6375_chg_get_value(ddata, F_AICR, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = mt6375_chg_get_value(ddata, F_ICHG, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = mt6375_chg_get_value(ddata, F_CV, &val->intval);
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

static int mt6375_chg_set_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   const union power_supply_propval *val)
{
	struct mt6375_chg_data *ddata = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		return mt6375_chg_set_value(ddata, F_AICR, val->intval);
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		return mt6375_chg_set_value(ddata, F_ICHG, val->intval);
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		return mt6375_chg_set_value(ddata, F_CV, val->intval);
	default:
		return -EINVAL;
	}
}

static int mt6375_chg_prop_writeable(struct power_supply *psy,
				     enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		return 1;
	default:
		return 0;
	}
}

static enum power_supply_property mt6375_chg_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
};

static const struct power_supply_desc mt6375_chg_psy_desc = {
	.name			= "mt6375-charger",
	.type			= POWER_SUPPLY_TYPE_USB,
	.usb_types		= BIT(POWER_SUPPLY_USB_TYPE_SDP) |
				  BIT(POWER_SUPPLY_USB_TYPE_CDP) |
				  BIT(POWER_SUPPLY_USB_TYPE_DCP) |
				  BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN),
	.properties		= mt6375_chg_properties,
	.num_properties		= ARRAY_SIZE(mt6375_chg_properties),
	.get_property		= mt6375_chg_get_property,
	.set_property		= mt6375_chg_set_property,
	.property_is_writeable	= mt6375_chg_prop_writeable,
};

static irqreturn_t mt6375_chg_irq_handler(int irq, void *data)
{
	struct mt6375_chg_data *ddata = data;

	power_supply_changed(ddata->psy);
	return IRQ_HANDLED;
}

static int mt6375_chg_request_irq(struct mt6375_chg_data *ddata,
				  const char *name)
{
	struct platform_device *pdev = to_platform_device(ddata->dev);
	int irq, ret;

	irq = platform_get_irq_byname_optional(pdev, name);
	if (irq < 0)
		return 0;

	ret = devm_request_threaded_irq(ddata->dev, irq, NULL,
					mt6375_chg_irq_handler, IRQF_ONESHOT,
					name, ddata);
	if (ret)
		return dev_err_probe(ddata->dev, ret,
				     "Failed to request %s irq\n", name);
	return 0;
}

static int mt6375_chg_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mt6375_chg_data *ddata;
	struct power_supply_config cfg = {};
	int i, ret;

	ddata = devm_kzalloc(dev, sizeof(*ddata), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	ddata->dev = dev;
	ddata->regmap = dev_get_regmap(dev->parent, NULL);
	if (!ddata->regmap)
		return dev_err_probe(dev, -ENODEV, "Failed to get regmap\n");

	for (i = 0; i < F_MAX; i++) {
		ddata->rm_field[i] = devm_regmap_field_alloc(dev, ddata->regmap,
							mt6375_chg_fields[i]);
		if (IS_ERR(ddata->rm_field[i]))
			return dev_err_probe(dev, PTR_ERR(ddata->rm_field[i]),
					     "Failed to alloc regmap field\n");
	}

	/* Let the charger run BC1.2 detection automatically on attach */
	ret = mt6375_chg_field_set(ddata, F_BC12_EN, 1);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable BC1.2\n");

	cfg.drv_data = ddata;
	cfg.fwnode = dev_fwnode(dev);

	ddata->psy = devm_power_supply_register(dev, &mt6375_chg_psy_desc, &cfg);
	if (IS_ERR(ddata->psy))
		return dev_err_probe(dev, PTR_ERR(ddata->psy),
				     "Failed to register power supply\n");

	platform_set_drvdata(pdev, ddata);

	ret = mt6375_chg_request_irq(ddata, "attach");
	if (ret)
		return ret;
	ret = mt6375_chg_request_irq(ddata, "detach");
	if (ret)
		return ret;
	ret = mt6375_chg_request_irq(ddata, "bc12_done");
	if (ret)
		return ret;

	return 0;
}

static const struct of_device_id mt6375_chg_of_match[] = {
	{ .compatible = "mediatek,mt6375-charger" },
	{}
};
MODULE_DEVICE_TABLE(of, mt6375_chg_of_match);

static struct platform_driver mt6375_chg_driver = {
	.driver = {
		.name = "mt6375-charger",
		.of_match_table = mt6375_chg_of_match,
	},
	.probe = mt6375_chg_probe,
};
module_platform_driver(mt6375_chg_driver);

MODULE_DESCRIPTION("MediaTek MT6375 Charger Driver");
MODULE_LICENSE("GPL");
