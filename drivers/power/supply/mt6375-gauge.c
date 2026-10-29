// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6375 fuel gauge driver
 *
 * Copyright (c) 2021 MediaTek Inc.
 *
 * A fuel gauge for the MT6375 SubPMIC. It derives the state of charge from a
 * mix of two sources: the open-circuit voltage (OCV) table supplied by the
 * monitored-battery node, and the on-chip hardware coulomb counter (FG_CAR in
 * the BM bank). The OCV estimate is accurate only while the cell rests, so it
 * is used to anchor the gauge whenever the current is small; between anchors
 * the accumulated charge from the coulomb counter tracks the state of charge
 * under load. Battery voltage and current come from the auxiliary ADC. This
 * avoids the vendor's out-of-tree coulomb-counter daemon while still following
 * the charge accurately through a charge/discharge cycle.
 */

#include <linux/delay.h>
#include <linux/iio/consumer.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>

#define MT6375_GAUGE_DEFAULT_RINT_UOHM	120000

/*
 * BM-bank coulomb counter. FG_CAR is a free-running 32-bit charge accumulator:
 * bit 31 is the sign (set while discharging) and bits 30:0 are the magnitude in
 * units of CHARGE_LSB. One LSB is 0.085 uAh (85 nAh).
 */
#define MT6375_REG_FGADC_CAR		0x278
#define MT6375_FGADC_CAR_SIZE		4
#define MT6375_FGADC_CHARGE_LSB_NAH	85

/*
 * The accumulator is latched for a consistent read: set SW_READ_PRE, wait for
 * LATCHDATA_ST, read FG_CAR, then release with SW_CLEAR.
 */
#define MT6375_REG_FGADC_CON2		0x26F
#define MT6375_FG_LATCHDATA_ST		BIT(7)
#define MT6375_REG_FGADC_CON3		0x270
#define MT6375_FG_SW_CLEAR		BIT(3)
#define MT6375_FG_SW_READ_PRE		BIT(0)

/*
 * Below this current magnitude the cell is treated as resting, so the OCV
 * estimate is trustworthy and the coulomb tracker is re-anchored to it.
 */
#define MT6375_GAUGE_REST_UA		60000

struct mt6375_gauge_data {
	struct device *dev;
	struct regmap *regmap;
	struct iio_channel *vbat_chan;
	struct iio_channel *ibat_chan;
	struct power_supply *psy;
	struct power_supply_battery_info *info;
	struct power_supply *chg_psy;

	/* Serialises the coulomb-counter latch-read-release sequence. */
	struct mutex car_lock;

	/* Coulomb tracker anchor, captured at rest from the OCV estimate. */
	bool cc_anchored;
	int anchor_soc;
	s64 anchor_charge_uah;
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

/* Latch the fuel-gauge accumulator so FG_CAR can be read coherently. */
static int mt6375_gauge_latch(struct mt6375_gauge_data *ddata)
{
	u32 val;
	int i, ret;

	ret = regmap_read(ddata->regmap, MT6375_REG_FGADC_CON3, &val);
	if (ret)
		return ret;
	if (val & (MT6375_FG_SW_READ_PRE | MT6375_FG_SW_CLEAR))
		return -EBUSY;		/* a previous latch was left pending */

	ret = regmap_update_bits(ddata->regmap, MT6375_REG_FGADC_CON3,
				 MT6375_FG_SW_READ_PRE | MT6375_FG_SW_CLEAR,
				 MT6375_FG_SW_READ_PRE);
	if (ret)
		return ret;
	usleep_range(100, 200);

	for (i = 0; i < 3; i++) {
		ret = regmap_read(ddata->regmap, MT6375_REG_FGADC_CON2, &val);
		if (ret)
			return ret;
		if (val & MT6375_FG_LATCHDATA_ST)
			return 0;
		usleep_range(50, 100);
	}
	return -ETIMEDOUT;
}

/* Release the accumulator latch taken by mt6375_gauge_latch(). */
static void mt6375_gauge_unlatch(struct mt6375_gauge_data *ddata)
{
	u32 val;
	int i;

	if (regmap_update_bits(ddata->regmap, MT6375_REG_FGADC_CON3,
			       MT6375_FG_SW_READ_PRE | MT6375_FG_SW_CLEAR,
			       MT6375_FG_SW_CLEAR))
		return;
	usleep_range(100, 200);

	for (i = 0; i < 3; i++) {
		if (regmap_read(ddata->regmap, MT6375_REG_FGADC_CON2, &val))
			break;
		if (!(val & MT6375_FG_LATCHDATA_ST))
			break;
		usleep_range(50, 100);
	}

	regmap_update_bits(ddata->regmap, MT6375_REG_FGADC_CON3,
			   MT6375_FG_SW_CLEAR, 0);
}

/*
 * Read the hardware coulomb counter and return the accumulated charge in
 * microamp-hours. The value is signed relative to the counter's own power-on
 * reset: it rises while charging and falls while discharging, so only the
 * difference between two reads is meaningful.
 */
static int mt6375_gauge_read_charge(struct mt6375_gauge_data *ddata, s64 *uah)
{
	u8 buf[MT6375_FGADC_CAR_SIZE];
	u32 raw, mag;
	s64 lsb;
	int ret;

	mutex_lock(&ddata->car_lock);
	ret = mt6375_gauge_latch(ddata);
	if (!ret) {
		ret = regmap_bulk_read(ddata->regmap, MT6375_REG_FGADC_CAR, buf,
				       sizeof(buf));
		mt6375_gauge_unlatch(ddata);
	}
	mutex_unlock(&ddata->car_lock);
	if (ret)
		return ret;

	raw = buf[0] | buf[1] << 8 | buf[2] << 16 | (u32)buf[3] << 24;
	mag = raw & GENMASK(30, 0);

	if (mag == 0)
		lsb = 0;
	else if (raw & BIT(31))
		lsb = -(s64)(GENMASK(30, 0) - mag);	/* discharging */
	else
		lsb = mag;				/* charging */

	*uah = div_s64(lsb * MT6375_FGADC_CHARGE_LSB_NAH, 1000);
	return 0;
}

/* Open-circuit state of charge from the ADC voltage, IR-compensated. */
static int mt6375_gauge_ocv_soc(struct mt6375_gauge_data *ddata)
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

	return clamp(ret, 0, 100);
}

static int mt6375_gauge_get_capacity(struct mt6375_gauge_data *ddata, int *cap)
{
	int ocv_soc, ibat, full_uah, soc;
	s64 charge_uah;
	int ret;

	ocv_soc = mt6375_gauge_ocv_soc(ddata);
	if (ocv_soc < 0)
		return ocv_soc;

	/* Without a coulomb counter or a battery capacity, fall back to OCV. */
	full_uah = ddata->info->charge_full_design_uah;
	if (mt6375_gauge_read_charge(ddata, &charge_uah) || full_uah <= 0) {
		ddata->cc_anchored = false;
		*cap = ocv_soc;
		return 0;
	}

	ret = mt6375_gauge_get_current(ddata, &ibat);
	if (ret)
		ibat = 0;

	/*
	 * Re-anchor to the OCV estimate whenever the cell is at rest (or on the
	 * first read): OCV is trustworthy there. Under load, hold the anchor and
	 * add the charge that has flowed since, measured by the coulomb counter.
	 */
	if (!ddata->cc_anchored || abs(ibat) < MT6375_GAUGE_REST_UA) {
		ddata->anchor_soc = ocv_soc;
		ddata->anchor_charge_uah = charge_uah;
		ddata->cc_anchored = true;
		soc = ocv_soc;
	} else {
		soc = ddata->anchor_soc +
		      div_s64((charge_uah - ddata->anchor_charge_uah) * 100,
			      full_uah);
	}

	*cap = clamp(soc, 0, 100);
	return 0;
}

static int mt6375_gauge_get_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *val)
{
	struct mt6375_gauge_data *ddata = power_supply_get_drvdata(psy);
	struct power_supply_battery_info *info = ddata->info;
	s64 charge_uah;
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
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		ret = mt6375_gauge_read_charge(ddata, &charge_uah);
		if (ret)
			return ret;
		val->intval = charge_uah;
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
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
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
	mutex_init(&ddata->car_lock);

	ddata->regmap = dev_get_regmap(dev->parent, NULL);
	if (!ddata->regmap)
		return dev_err_probe(dev, -ENODEV, "Failed to get regmap\n");

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
