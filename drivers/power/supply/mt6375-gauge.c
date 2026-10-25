// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6375 fuel gauge driver
 *
 * Copyright (c) 2021 MediaTek Inc.
 *
 * A lean fuel gauge for the MT6375 SubPMIC. It reads the battery voltage and
 * current from the on-chip auxiliary ADC and derives the state of charge from
 * the open-circuit-voltage table supplied by the monitored-battery node,
 * compensating for the IR drop with the measured current. This avoids the
 * vendor's out-of-tree coulomb-counter daemon while still giving an accurate
 * reading on a mostly-idle system.
 */

#include <linux/iio/consumer.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>

#define MT6375_GAUGE_DEFAULT_RINT_UOHM	120000

struct mt6375_gauge_data {
	struct device *dev;
	struct iio_channel *vbat_chan;
	struct iio_channel *ibat_chan;
	struct power_supply *psy;
	struct power_supply_battery_info *info;
	struct power_supply *chg_psy;
};

/* Returns battery terminal voltage in microvolts */
static int mt6375_gauge_get_voltage(struct mt6375_gauge_data *ddata, int *uV)
{
	int ret, val;

	ret = iio_read_channel_processed(ddata->vbat_chan, &val);
	if (ret < 0)
		return ret;
	*uV = val;
	return 0;
}

/* Returns battery current magnitude in microamperes */
static int mt6375_gauge_get_current(struct mt6375_gauge_data *ddata, int *uA)
{
	int ret, val;

	if (!ddata->ibat_chan) {
		*uA = 0;
		return 0;
	}
	ret = iio_read_channel_processed(ddata->ibat_chan, &val);
	if (ret < 0)
		return ret;
	*uA = val;
	return 0;
}

/* True when the charger reports an input supply is present */
static bool mt6375_gauge_is_charging(struct mt6375_gauge_data *ddata)
{
	union power_supply_propval val;
	int ret;

	if (!ddata->chg_psy)
		ddata->chg_psy = power_supply_get_by_name("mt6375-charger");
	if (!ddata->chg_psy)
		return false;

	ret = power_supply_get_property(ddata->chg_psy,
					POWER_SUPPLY_PROP_ONLINE, &val);
	if (ret)
		return false;
	return val.intval != 0;
}

static int mt6375_gauge_get_capacity(struct mt6375_gauge_data *ddata, int *cap)
{
	int ret, vbat, ibat, ocv, rint;

	ret = mt6375_gauge_get_voltage(ddata, &vbat);
	if (ret)
		return ret;
	ret = mt6375_gauge_get_current(ddata, &ibat);
	if (ret)
		ibat = 0;

	rint = ddata->info->factory_internal_resistance_uohm;
	if (rint <= 0)
		rint = MT6375_GAUGE_DEFAULT_RINT_UOHM;

	/*
	 * Compensate the measured terminal voltage back to the open-circuit
	 * voltage. While charging, current flows into the cell and raises the
	 * terminal voltage, so remove the IR term; while discharging it sags,
	 * so add it back.
	 */
	ocv = vbat;
	if (mt6375_gauge_is_charging(ddata))
		ocv -= (s64)ibat * rint / 1000000;
	else
		ocv += (s64)ibat * rint / 1000000;

	ret = power_supply_batinfo_ocv2cap(ddata->info, ocv, 25);
	if (ret < 0)
		return ret;

	*cap = clamp(ret, 0, 100);
	return 0;
}

static int mt6375_gauge_get_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *val)
{
	struct mt6375_gauge_data *ddata = power_supply_get_drvdata(psy);
	struct power_supply_battery_info *info = ddata->info;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = info->technology;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = mt6375_gauge_get_voltage(ddata, &val->intval);
		if (ret)
			return ret;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = mt6375_gauge_get_current(ddata, &val->intval);
		if (ret)
			return ret;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = mt6375_gauge_get_capacity(ddata, &val->intval);
		if (ret)
			return ret;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = info->charge_full_design_uah;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = info->voltage_max_design_uv;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = info->voltage_min_design_uv;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static enum power_supply_property mt6375_gauge_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
};

static const struct power_supply_desc mt6375_gauge_psy_desc = {
	.name		= "mt6375-gauge",
	.type		= POWER_SUPPLY_TYPE_BATTERY,
	.properties	= mt6375_gauge_properties,
	.num_properties	= ARRAY_SIZE(mt6375_gauge_properties),
	.get_property	= mt6375_gauge_get_property,
};

static int mt6375_gauge_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mt6375_gauge_data *ddata;
	struct power_supply_config cfg = {};
	int ret;

	ddata = devm_kzalloc(dev, sizeof(*ddata), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	ddata->dev = dev;

	ddata->vbat_chan = devm_iio_channel_get(dev, "vbat");
	if (IS_ERR(ddata->vbat_chan))
		return dev_err_probe(dev, PTR_ERR(ddata->vbat_chan),
				     "Failed to get vbat ADC channel\n");

	ddata->ibat_chan = devm_iio_channel_get(dev, "ibat");
	if (IS_ERR(ddata->ibat_chan)) {
		if (PTR_ERR(ddata->ibat_chan) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		ddata->ibat_chan = NULL;
	}

	cfg.drv_data = ddata;
	cfg.fwnode = dev_fwnode(dev);

	ddata->psy = devm_power_supply_register(dev, &mt6375_gauge_psy_desc,
						&cfg);
	if (IS_ERR(ddata->psy))
		return dev_err_probe(dev, PTR_ERR(ddata->psy),
				     "Failed to register power supply\n");

	ret = power_supply_get_battery_info(ddata->psy, &ddata->info);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to get battery info\n");

	if (!ddata->info->ocv_table[0])
		return dev_err_probe(dev, -EINVAL,
				     "monitored-battery has no ocv-capacity-table\n");

	platform_set_drvdata(pdev, ddata);
	return 0;
}

static void mt6375_gauge_remove(struct platform_device *pdev)
{
	struct mt6375_gauge_data *ddata = platform_get_drvdata(pdev);

	if (ddata->chg_psy)
		power_supply_put(ddata->chg_psy);
	power_supply_put_battery_info(ddata->psy, ddata->info);
}

static const struct of_device_id mt6375_gauge_of_match[] = {
	{ .compatible = "mediatek,mt6375-gauge" },
	{}
};
MODULE_DEVICE_TABLE(of, mt6375_gauge_of_match);

static struct platform_driver mt6375_gauge_driver = {
	.driver = {
		.name = "mt6375-gauge",
		.of_match_table = mt6375_gauge_of_match,
	},
	.probe = mt6375_gauge_probe,
	.remove = mt6375_gauge_remove,
};
module_platform_driver(mt6375_gauge_driver);

MODULE_DESCRIPTION("MediaTek MT6375 Fuel Gauge Driver");
MODULE_LICENSE("GPL");
