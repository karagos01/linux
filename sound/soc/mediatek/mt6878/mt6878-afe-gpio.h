/* SPDX-License-Identifier: GPL-2.0 */
/*
 * mt6878-afe-gpio.h  --  MediaTek 6878 afe gpio ctrl definition
 *
 * Copyright (c) 2026 Jan Benedikt <karagos01@gmail.com>
 */

#ifndef _MT6878_AFE_GPIO_H_
#define _MT6878_AFE_GPIO_H_

struct mtk_base_afe;

int mt6878_afe_gpio_init(struct mtk_base_afe *afe);

int mt6878_afe_gpio_request(struct mtk_base_afe *afe, bool enable,
			    int dai, int uplink);

#endif
