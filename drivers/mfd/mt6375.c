// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6375 SubPMIC MFD core driver
 *
 * Copyright (c) 2021 MediaTek Inc.
 *
 * The MT6375 is a companion PMIC bundling a single-cell switching battery
 * charger with BC1.2 detection, an auxiliary ADC, a coulomb-counter fuel
 * gauge and a USB Type-C port controller. The die answers on six I2C slave
 * addresses that are presented to the rest of the kernel as one banked
 * 16-bit register space (see mt6375.h).
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/regmap.h>

#include <dt-bindings/mfd/mt6375.h>

#include "mt6375.h"

#define MT6375_REG_DEV_INFO	0x100
#define MT6375_REG_CHG_IRQ0	0x1D0
#define MT6375_REG_CHG_MASK0	0x1F0
#define MT6375_REG_HK2_END	0x4FF
#define MT6375_REG_RT2_START	0xF200
#define MT6375_REG_RT2_END	0xF2FF

#define MT6375_VENID_MASK	GENMASK(7, 4)
#define MT6375_CHIPREV_MASK	GENMASK(3, 0)
#define MT6375_VENID_MT6375	0x7

#define MT6375_NUM_IRQREGS	16
#define MT6375_MAX_ADDRLEN	2

static const u8 mt6375_slave_addr[MT6375_SLAVE_MAX] = {
	[MT6375_SLAVE_TCPC]	= 0x4E,
	[MT6375_SLAVE_PMU]	= 0x34,
	[MT6375_SLAVE_BM]	= 0x1A,
	[MT6375_SLAVE_HK1]	= 0x4A,
	[MT6375_SLAVE_HK2]	= 0x64,
	[MT6375_SLAVE_TM]	= 0x3F,
};

static const struct regmap_irq mt6375_irqs[] = {
	REGMAP_IRQ_REG_LINE(MT6375_FL_PWR_RDY, 8),
	REGMAP_IRQ_REG_LINE(MT6375_FL_DETACH, 8),
	REGMAP_IRQ_REG_LINE(MT6375_FL_RECHG, 8),
	REGMAP_IRQ_REG_LINE(MT6375_FL_CHG_DONE, 8),
	REGMAP_IRQ_REG_LINE(MT6375_FL_BK_CHG, 8),
	REGMAP_IRQ_REG_LINE(MT6375_FL_IEOC, 8),
	REGMAP_IRQ_REG_LINE(MT6375_FL_CHG_RDY, 8),
	REGMAP_IRQ_REG_LINE(MT6375_FL_VBUS_GD, 8),
	REGMAP_IRQ_REG_LINE(MT6375_FL_VBUS_OV, 8),
	REGMAP_IRQ_REG_LINE(MT6375_FL_CHG_BATOV, 8),
	REGMAP_IRQ_REG_LINE(MT6375_FL_CHG_SYSOV, 8),
	REGMAP_IRQ_REG_LINE(MT6375_FL_CHG_TOUT, 8),
	REGMAP_IRQ_REG_LINE(MT6375_FL_CHG_BUSUV, 8),
	REGMAP_IRQ_REG_LINE(MT6375_FL_CHG_THREG, 8),
	REGMAP_IRQ_REG_LINE(MT6375_FL_CHG_AICR, 8),
	REGMAP_IRQ_REG_LINE(MT6375_FL_CHG_MIVR, 8),
	REGMAP_IRQ_REG_LINE(MT6375_FL_SYS_SHORT, 8),
	REGMAP_IRQ_REG_LINE(MT6375_FL_SYS_MIN, 8),
	REGMAP_IRQ_REG_LINE(MT6375_FL_AICC_DONE, 8),
	REGMAP_IRQ_REG_LINE(MT6375_FL_PE_DONE, 8),
	REGMAP_IRQ_REG_LINE(MT6375_PP_PGB_EVT, 8),
	REGMAP_IRQ_REG_LINE(MT6375_FT_DIG_THR, 8),
	REGMAP_IRQ_REG_LINE(MT6375_FL_WDT, 8),
	REGMAP_IRQ_REG_LINE(MT6375_FL_OTG_FAULT, 8),
	REGMAP_IRQ_REG_LINE(MT6375_FL_OTG_BAT_LBP, 8),
	REGMAP_IRQ_REG_LINE(MT6375_FL_OTG_CC, 8),
	REGMAP_IRQ_REG_LINE(MT6375_FL_BATPRO_DONE, 8),
	REGMAP_IRQ_REG_LINE(MT6375_FL_DCD_DONE, 8),
	REGMAP_IRQ_REG_LINE(MT6375_FL_BC12_HVDCP, 8),
	REGMAP_IRQ_REG_LINE(MT6375_FL_BC12_DN, 8),
	REGMAP_IRQ_REG_LINE(MT6375_INT_CHRDET_UV, 8),
	REGMAP_IRQ_REG_LINE(MT6375_INT_CHRDET_OV, 8),
	REGMAP_IRQ_REG_LINE(MT6375_INT_CHRDET_EXT, 8),
	REGMAP_IRQ_REG_LINE(MT6375_ADC_VBAT_MON_OV, 8),
	REGMAP_IRQ_REG_LINE(MT6375_ADC_DONEI, 8),
	REGMAP_IRQ_REG_LINE(MT6375_GM30_EVT, 8),
	REGMAP_IRQ_REG_LINE(MT6375_USBID_EVT, 8),
	REGMAP_IRQ_REG_LINE(MT6375_PD_EVT, 8),
};

static const struct regmap_irq_chip mt6375_irq_chip = {
	.name		= "mt6375-irqs",
	.status_base	= MT6375_REG_CHG_IRQ0,
	.mask_base	= MT6375_REG_CHG_MASK0,
	.ack_base	= MT6375_REG_CHG_IRQ0,
	.num_regs	= MT6375_NUM_IRQREGS,
	.irqs		= mt6375_irqs,
	.num_irqs	= ARRAY_SIZE(mt6375_irqs),
};

static const struct mfd_cell mt6375_devices[] = {
	MFD_CELL_OF("mt6375-adc", NULL, NULL, 0, 0, "mediatek,mt6375-adc"),
	MFD_CELL_OF("mt6375-charger", NULL, NULL, 0, 0, "mediatek,mt6375-charger"),
	MFD_CELL_OF("mt6375-gauge", NULL, NULL, 0, 0, "mediatek,mt6375-gauge"),
	MFD_CELL_OF("mt6375-tcpc", NULL, NULL, 0, 0, "mediatek,mt6375-tcpc"),
};

static struct i2c_client *mt6375_bank_to_i2c(struct mt6375_info *info, u8 bank)
{
	if (bank == MT6375_BANK_TCPC_RT2)
		return info->i2c[MT6375_SLAVE_TCPC];
	if (bank >= MT6375_SLAVE_MAX)
		return NULL;
	return info->i2c[bank];
}

static int mt6375_regmap_read(void *context, const void *reg_buf,
			      size_t reg_size, void *val_buf, size_t val_size)
{
	struct mt6375_info *info = context;
	const u8 *u8_buf = reg_buf;
	struct i2c_client *i2c;
	int ret;

	i2c = mt6375_bank_to_i2c(info, u8_buf[0]);
	if (!i2c)
		return -EINVAL;

	if (u8_buf[0] == MT6375_BANK_TCPC_RT2) {
		struct i2c_msg msg[2] = {
			{
				.addr = i2c->addr,
				.flags = i2c->flags,
				.len = MT6375_MAX_ADDRLEN,
				.buf = (u8 *)u8_buf,
			}, {
				.addr = i2c->addr,
				.flags = i2c->flags | I2C_M_RD,
				.len = val_size,
				.buf = val_buf,
			},
		};

		ret = i2c_transfer(i2c->adapter, msg, ARRAY_SIZE(msg));
		return ret == ARRAY_SIZE(msg) ? 0 : (ret < 0 ? ret : -EIO);
	}

	ret = i2c_smbus_read_i2c_block_data(i2c, u8_buf[1], val_size, val_buf);
	if (ret < 0)
		return ret;
	return ret != val_size ? -EIO : 0;
}

static int mt6375_regmap_write(void *context, const void *data, size_t count)
{
	struct mt6375_info *info = context;
	const u8 *u8_buf = data;
	struct i2c_client *i2c;
	int len = count - MT6375_MAX_ADDRLEN;
	int ret;

	i2c = mt6375_bank_to_i2c(info, u8_buf[0]);
	if (!i2c)
		return -EINVAL;

	if (u8_buf[0] == MT6375_BANK_TCPC_RT2) {
		ret = i2c_master_send(i2c, data, count);
		return ret == count ? 0 : (ret < 0 ? ret : -EIO);
	}

	return i2c_smbus_write_i2c_block_data(i2c, u8_buf[1], len,
					      data + MT6375_MAX_ADDRLEN);
}

static const struct regmap_bus mt6375_regmap_bus = {
	.read		= mt6375_regmap_read,
	.write		= mt6375_regmap_write,
	.max_raw_read	= 16,
	.max_raw_write	= 15,
};

static bool mt6375_accessible_reg(struct device *dev, unsigned int reg)
{
	return reg <= MT6375_REG_HK2_END ||
	       (reg >= MT6375_REG_RT2_START && reg <= MT6375_REG_RT2_END);
}

static const struct regmap_config mt6375_regmap_config = {
	.reg_bits		= 16,
	.val_bits		= 8,
	.reg_format_endian	= REGMAP_ENDIAN_BIG,
	.max_register		= MT6375_REG_RT2_END,
	.readable_reg		= mt6375_accessible_reg,
	.writeable_reg		= mt6375_accessible_reg,
};

static int mt6375_check_vendor_info(struct device *dev, struct regmap *regmap)
{
	unsigned int devinfo;
	int ret, vid;

	ret = regmap_read(regmap, MT6375_REG_DEV_INFO, &devinfo);
	if (ret)
		return ret;

	vid = FIELD_GET(MT6375_VENID_MASK, devinfo);
	if (vid != MT6375_VENID_MT6375) {
		dev_err(dev, "Unknown vendor id 0x%02x\n", devinfo);
		return -ENODEV;
	}

	dev_info(dev, "MT6375 SubPMIC rev %lu\n",
		 FIELD_GET(MT6375_CHIPREV_MASK, devinfo));
	return 0;
}

static int mt6375_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct mt6375_info *info;
	struct regmap *regmap;
	int i, ret;

	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	i2c_set_clientdata(i2c, info);

	for (i = 0; i < MT6375_SLAVE_MAX; i++) {
		if (i == MT6375_SLAVE_PMU) {
			info->i2c[i] = i2c;
			continue;
		}
		info->i2c[i] = devm_i2c_new_dummy_device(dev, i2c->adapter,
							 mt6375_slave_addr[i]);
		if (IS_ERR(info->i2c[i]))
			return dev_err_probe(dev, PTR_ERR(info->i2c[i]),
					     "Failed to add I2C client 0x%02x\n",
					     mt6375_slave_addr[i]);
	}

	regmap = devm_regmap_init(dev, &mt6375_regmap_bus, info,
				  &mt6375_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap),
				     "Failed to init regmap\n");
	info->regmap = regmap;

	ret = mt6375_check_vendor_info(dev, regmap);
	if (ret)
		return ret;

	ret = devm_regmap_add_irq_chip(dev, regmap, i2c->irq, IRQF_ONESHOT, -1,
				       &mt6375_irq_chip, &info->irq_data);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to add irq chip\n");

	return devm_mfd_add_devices(dev, PLATFORM_DEVID_AUTO, mt6375_devices,
				    ARRAY_SIZE(mt6375_devices), NULL, 0,
				    regmap_irq_get_domain(info->irq_data));
}

static const struct of_device_id mt6375_of_match[] = {
	{ .compatible = "mediatek,mt6375" },
	{}
};
MODULE_DEVICE_TABLE(of, mt6375_of_match);

static struct i2c_driver mt6375_driver = {
	.driver = {
		.name = "mt6375",
		.of_match_table = mt6375_of_match,
	},
	.probe = mt6375_probe,
};
module_i2c_driver(mt6375_driver);

MODULE_DESCRIPTION("MediaTek MT6375 SubPMIC Driver");
MODULE_LICENSE("GPL");
