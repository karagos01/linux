// SPDX-License-Identifier: GPL-2.0-only
/*
 * Will Semiconductor WL28681 camera PMIC: seven I2C programmable LDOs
 *
 * LDO1 and LDO2 are low-voltage (digital core) outputs, LDO3 to LDO7
 * high-voltage (analog, I/O, actuator) outputs. Every output has an 8-bit
 * voltage code with an 8 mV step and an enable bit in a shared register.
 * The code-to-voltage mapping matches the codes the vendor camera driver
 * writes for its 1.2 V, 1.8 V and 2.8 V supplies. LDO1 and LDO2 have their
 * own input, which a board may switch separately.
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>

#define WL28681_REG_PRODUCT_ID		0x00
#define WL28681_REG_LDO_EN		0x03
#define WL28681_REG_LDO1_VOUT		0x04
#define WL28681_REG_DISCHARGE		0x10
#define WL28681_REG_MAX			0x1e

#define WL28681_PRODUCT_ID		0x0d

/* time the chip needs after its enable pin is raised before it answers */
#define WL28681_ENABLE_DELAY_US		10000

enum wl28681_regulator_id {
	WL28681_LDO1,
	WL28681_LDO2,
	WL28681_LDO3,
	WL28681_LDO4,
	WL28681_LDO5,
	WL28681_LDO6,
	WL28681_LDO7,
};

static const struct regulator_ops wl28681_ldo_ops = {
	.list_voltage = regulator_list_voltage_linear_range,
	.map_voltage = regulator_map_voltage_linear_range,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.set_active_discharge = regulator_set_active_discharge_regmap,
};

/*
 * The whole 8-bit code space maps linearly (a code of 0 is the reset value,
 * so it has to be listed); the board limits the usable range through the
 * regulator constraints.
 */

/* LDO1/2: V = (code + 1) * 8 mV, e.g. 0x95 = 1.2 V */
static const struct linear_range wl28681_low_range[] = {
	REGULATOR_LINEAR_RANGE(8000, 0x00, 0xff, 8000),
};

/* LDO3-7: V = 1.372 V + code * 8 mV, e.g. 0x36 = 1.804 V, 0xb3 = 2.804 V */
static const struct linear_range wl28681_high_range[] = {
	REGULATOR_LINEAR_RANGE(1372000, 0x00, 0xff, 8000),
};

#define WL28681_LDO(_num, _range, _supply)				\
	[WL28681_LDO##_num] = {						\
		.name = "ldo" #_num,					\
		.of_match = "ldo" #_num,				\
		.supply_name = _supply,					\
		.regulators_node = "regulators",			\
		.id = WL28681_LDO##_num,				\
		.type = REGULATOR_VOLTAGE,				\
		.owner = THIS_MODULE,					\
		.ops = &wl28681_ldo_ops,				\
		.linear_ranges = _range,				\
		.n_linear_ranges = ARRAY_SIZE(_range),			\
		.n_voltages = 256,					\
		.vsel_reg = WL28681_REG_LDO1_VOUT + (_num) - 1,		\
		.vsel_mask = 0xff,					\
		.enable_reg = WL28681_REG_LDO_EN,			\
		.enable_mask = BIT((_num) - 1),				\
		.active_discharge_reg = WL28681_REG_DISCHARGE,		\
		.active_discharge_mask = BIT((_num) - 1),		\
		.active_discharge_on = BIT((_num) - 1),			\
		.enable_time = 1000,					\
	}

static const struct regulator_desc wl28681_regulators[] = {
	WL28681_LDO(1, wl28681_low_range, "vin12"),
	WL28681_LDO(2, wl28681_low_range, "vin12"),
	WL28681_LDO(3, wl28681_high_range, NULL),
	WL28681_LDO(4, wl28681_high_range, NULL),
	WL28681_LDO(5, wl28681_high_range, NULL),
	WL28681_LDO(6, wl28681_high_range, NULL),
	WL28681_LDO(7, wl28681_high_range, NULL),
};

static const struct regmap_config wl28681_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = WL28681_REG_MAX,
};

static int wl28681_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct regulator_config config = { };
	struct gpio_desc *enable_gpio;
	struct regmap *regmap;
	unsigned int id;
	int i, ret;

	enable_gpio = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(enable_gpio))
		return dev_err_probe(dev, PTR_ERR(enable_gpio),
				     "failed to get the enable GPIO\n");
	if (enable_gpio)
		fsleep(WL28681_ENABLE_DELAY_US);

	regmap = devm_regmap_init_i2c(client, &wl28681_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap),
				     "failed to initialise the regmap\n");

	ret = regmap_read(regmap, WL28681_REG_PRODUCT_ID, &id);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read the product ID\n");
	if (id != WL28681_PRODUCT_ID)
		return dev_err_probe(dev, -ENODEV,
				     "unknown product ID 0x%02x\n", id);

	config.dev = dev;
	config.regmap = regmap;

	for (i = 0; i < ARRAY_SIZE(wl28681_regulators); i++) {
		struct regulator_dev *rdev;

		rdev = devm_regulator_register(dev, &wl28681_regulators[i],
					       &config);
		if (IS_ERR(rdev))
			return dev_err_probe(dev, PTR_ERR(rdev),
					     "failed to register %s\n",
					     wl28681_regulators[i].name);
	}

	return 0;
}

static const struct of_device_id wl28681_of_match[] = {
	{ .compatible = "willsemi,wl28681" },
	{ }
};
MODULE_DEVICE_TABLE(of, wl28681_of_match);

static const struct i2c_device_id wl28681_i2c_id[] = {
	{ "wl28681" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, wl28681_i2c_id);

static struct i2c_driver wl28681_driver = {
	.driver = {
		.name = "wl28681",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = wl28681_of_match,
	},
	.probe = wl28681_probe,
	.id_table = wl28681_i2c_id,
};
module_i2c_driver(wl28681_driver);

MODULE_DESCRIPTION("Will Semiconductor WL28681 camera PMIC regulator driver");
MODULE_LICENSE("GPL");
