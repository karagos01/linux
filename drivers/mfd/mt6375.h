/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#ifndef __MFD_MT6375_H
#define __MFD_MT6375_H

#include <linux/i2c.h>

/*
 * The MT6375 is a single die exposed through six I2C slave addresses on the
 * same bus. A banked 16-bit register space selects the slave by its high
 * byte (the enum below) and the in-slave register by its low byte. The TCPC
 * "RT2" page is reached through a dedicated high-byte value.
 */
enum {
	MT6375_SLAVE_TCPC,	/* 0x4E */
	MT6375_SLAVE_PMU,	/* 0x34 - charger, ADC, BC1.2 */
	MT6375_SLAVE_BM,	/* 0x1A - fuel gauge */
	MT6375_SLAVE_HK1,	/* 0x4A */
	MT6375_SLAVE_HK2,	/* 0x64 */
	MT6375_SLAVE_TM,	/* 0x3F */
	MT6375_SLAVE_MAX,
};

#define MT6375_BANK_TCPC_RT2	0xF2

struct mt6375_info {
	struct i2c_client *i2c[MT6375_SLAVE_MAX];
	struct regmap *regmap;
	struct regmap_irq_chip_data *irq_data;
};

#endif
