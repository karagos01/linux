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
#include <linux/notifier.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/string.h>
#include <linux/workqueue.h>

#define MT6375_REG_TM_PASSCODE	0x107
#define MT6375_REG_CHG_TOP1	0x120
#define MT6375_REG_CHG_TOP2	0x121
#define MT6375_REG_CHG_AICR	0x122
#define MT6375_REG_CHG_VCHG	0x125
#define MT6375_REG_CHG_ICHG	0x126
#define MT6375_REG_CHG_BOOST4	0x129
#define MT6375_REG_CHG_OTGV	0x131
#define MT6375_REG_CHG_OTGC	0x132
#define MT6375_REG_CHG_STAT	0x134
#define MT6375_REG_CHG_BOOST2	0x135
#define MT6375_REG_CHG_BOOST3	0x140
#define MT6375_REG_BC12_FUNC	0x150
#define MT6375_REG_BC12_STAT	0x151
#define MT6375_REG_CHG_STAT0	0x1E0

/* CHG_TOP1: bit 0 enables the buck charger, bit 2 the OTG (boost) path */
#define MT6375_OTG_EN_MASK	BIT(2)
/* Passcode that unlocks the boost tuning registers; 0 re-locks them */
#define MT6375_TM_PASSCODE_UNLOCK	0x69
/* OTG boost output voltage (CHG_OTGV[5:0]) and current limit (CHG_OTGC[2:0]) */
#define MT6375_OTGV_MASK	GENMASK(5, 0)
#define MT6375_OTGV_MIN_UV	4850000
#define MT6375_OTGV_STEP_UV	25000
#define MT6375_OTGV_MIN_SEL	20
#define MT6375_OTGV_NUM		47
#define MT6375_OTGC_MASK	GENMASK(2, 0)

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

/* Input current to fall back to when there is no Type-C/PD contract (USB2 SDP) */
#define MT6375_AICR_DEFAULT_UA	500000

struct mt6375_chg_data {
	struct device *dev;
	struct regmap *regmap;
	struct regmap_field *rm_field[F_MAX];
	struct power_supply *psy;
	struct power_supply *tcpm_psy;
	struct notifier_block psy_nb;
	struct work_struct pd_work;
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

/*
 * Follow the input current the Type-C Port Manager negotiated: when the TCPM
 * sink contract is online, use its CURRENT_MAX (the PD PDO current, or the
 * Type-C Rp current) as our input current limit, otherwise fall back to a safe
 * SDP limit so a later non-PD source is not over-drawn.
 */
static void mt6375_chg_pd_work(struct work_struct *work)
{
	struct mt6375_chg_data *ddata =
		container_of(work, struct mt6375_chg_data, pd_work);
	union power_supply_propval online, curr;
	int ret, ilim;

	if (!ddata->tcpm_psy)
		return;

	ret = power_supply_get_property(ddata->tcpm_psy,
					POWER_SUPPLY_PROP_ONLINE, &online);
	if (ret)
		return;

	if (online.intval) {
		ret = power_supply_get_property(ddata->tcpm_psy,
						POWER_SUPPLY_PROP_CURRENT_MAX,
						&curr);
		if (ret || curr.intval <= 0)
			return;
		ilim = curr.intval;
	} else {
		ilim = MT6375_AICR_DEFAULT_UA;
	}

	ret = mt6375_chg_set_value(ddata, F_AICR, ilim);
	if (ret)
		dev_err(ddata->dev, "Failed to set input current %d uA: %d\n",
			ilim, ret);
	else
		dev_dbg(ddata->dev, "Input current limit set to %d uA\n", ilim);
}

static int mt6375_chg_psy_notify(struct notifier_block *nb,
				 unsigned long evt, void *data)
{
	struct mt6375_chg_data *ddata =
		container_of(nb, struct mt6375_chg_data, psy_nb);
	struct power_supply *psy = data;

	if (evt != PSY_EVENT_PROP_CHANGED ||
	    !strstarts(psy->desc->name, "tcpm-source"))
		return NOTIFY_OK;

	ddata->tcpm_psy = psy;
	schedule_work(&ddata->pd_work);

	return NOTIFY_OK;
}

static void mt6375_chg_psy_notifier_remove(void *data)
{
	struct mt6375_chg_data *ddata = data;

	power_supply_unreg_notifier(&ddata->psy_nb);
	cancel_work_sync(&ddata->pd_work);
}

/* Selectable OTG (boost) output current limits, in uA */
static const unsigned int mt6375_chg_otg_curr[] = {
	500000, 700000, 1100000, 1300000, 1800000, 2100000, 2400000,
};

/*
 * Converter tuning that has to be switched between buck (charging) and boost
 * (OTG) operation. These registers are behind the test-mode passcode, so they
 * are only touched while it is unlocked.
 */
static const struct {
	unsigned int reg;
	unsigned int mask;
	unsigned int boost;
	unsigned int buck;
} mt6375_chg_boost_params[] = {
	{ MT6375_REG_CHG_TOP2,   GENMASK(7, 6), 0x00, 0x40 },
	{ MT6375_REG_CHG_BOOST2, GENMASK(1, 0), 0x03, 0x00 },
	{ MT6375_REG_CHG_BOOST3, GENMASK(7, 6), 0xc0, 0x40 },
	{ MT6375_REG_CHG_BOOST4, BIT(7),        0x00, 0x80 },
};

static int mt6375_chg_set_boost_params(struct mt6375_chg_data *ddata, bool boost)
{
	int i, ret;

	ret = regmap_write(ddata->regmap, MT6375_REG_TM_PASSCODE,
			   MT6375_TM_PASSCODE_UNLOCK);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(mt6375_chg_boost_params); i++) {
		unsigned int val = boost ? mt6375_chg_boost_params[i].boost :
					   mt6375_chg_boost_params[i].buck;

		ret = regmap_update_bits(ddata->regmap,
					 mt6375_chg_boost_params[i].reg,
					 mt6375_chg_boost_params[i].mask, val);
		if (ret)
			break;
	}

	/* Always re-lock, even if a write above failed */
	regmap_write(ddata->regmap, MT6375_REG_TM_PASSCODE, 0);
	return ret;
}

static int mt6375_chg_otg_enable(struct regulator_dev *rdev)
{
	struct mt6375_chg_data *ddata = rdev_get_drvdata(rdev);
	int ret;

	ret = mt6375_chg_set_boost_params(ddata, true);
	if (ret)
		return ret;

	ret = regulator_enable_regmap(rdev);
	if (ret)
		mt6375_chg_set_boost_params(ddata, false);
	return ret;
}

static int mt6375_chg_otg_disable(struct regulator_dev *rdev)
{
	struct mt6375_chg_data *ddata = rdev_get_drvdata(rdev);
	int ret;

	ret = regulator_disable_regmap(rdev);
	if (ret)
		return ret;

	return mt6375_chg_set_boost_params(ddata, false);
}

static const struct regulator_ops mt6375_chg_otg_ops = {
	.enable			= mt6375_chg_otg_enable,
	.disable		= mt6375_chg_otg_disable,
	.is_enabled		= regulator_is_enabled_regmap,
	.list_voltage		= regulator_list_voltage_linear,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_current_limit	= regulator_set_current_limit_regmap,
	.get_current_limit	= regulator_get_current_limit_regmap,
};

static const struct regulator_desc mt6375_chg_otg_rdesc = {
	.of_match		= "usb-otg-vbus-regulator",
	.name			= "mt6375-usb-otg-vbus",
	.ops			= &mt6375_chg_otg_ops,
	.owner			= THIS_MODULE,
	.type			= REGULATOR_VOLTAGE,
	.min_uV			= MT6375_OTGV_MIN_UV,
	.uV_step		= MT6375_OTGV_STEP_UV,
	.n_voltages		= MT6375_OTGV_NUM,
	.linear_min_sel		= MT6375_OTGV_MIN_SEL,
	.vsel_reg		= MT6375_REG_CHG_OTGV,
	.vsel_mask		= MT6375_OTGV_MASK,
	.enable_reg		= MT6375_REG_CHG_TOP1,
	.enable_mask		= MT6375_OTG_EN_MASK,
	.curr_table		= mt6375_chg_otg_curr,
	.n_current_limits	= ARRAY_SIZE(mt6375_chg_otg_curr),
	.csel_reg		= MT6375_REG_CHG_OTGC,
	.csel_mask		= MT6375_OTGC_MASK,
};

static int mt6375_chg_init_otg_regulator(struct mt6375_chg_data *ddata)
{
	struct regulator_config cfg = {
		.dev = ddata->dev,
		.driver_data = ddata,
		.regmap = ddata->regmap,
	};
	struct regulator_dev *rdev;

	rdev = devm_regulator_register(ddata->dev, &mt6375_chg_otg_rdesc, &cfg);
	if (IS_ERR(rdev))
		return dev_err_probe(ddata->dev, PTR_ERR(rdev),
				     "Failed to register OTG VBUS regulator\n");
	return 0;
}

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

	ret = mt6375_chg_init_otg_regulator(ddata);
	if (ret)
		return ret;

	ret = mt6375_chg_request_irq(ddata, "attach");
	if (ret)
		return ret;
	ret = mt6375_chg_request_irq(ddata, "detach");
	if (ret)
		return ret;
	ret = mt6375_chg_request_irq(ddata, "bc12_done");
	if (ret)
		return ret;

	INIT_WORK(&ddata->pd_work, mt6375_chg_pd_work);
	ddata->psy_nb.notifier_call = mt6375_chg_psy_notify;
	ret = power_supply_reg_notifier(&ddata->psy_nb);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to register power supply notifier\n");

	return devm_add_action_or_reset(dev, mt6375_chg_psy_notifier_remove,
					ddata);
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
