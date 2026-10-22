/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MediaTek connectivity subsystem (connsys) power control
 */

#ifndef __SOC_MEDIATEK_MTK_CONNSYS_H
#define __SOC_MEDIATEK_MTK_CONNSYS_H

#include <linux/types.h>

struct device;
struct mtk_connsys;

enum mtk_connsys_radio {
	MTK_CONNSYS_BT,
	MTK_CONNSYS_FM,
	MTK_CONNSYS_GPS,
	MTK_CONNSYS_WIFI,
};

#if IS_ENABLED(CONFIG_MTK_CONNSYS)
struct mtk_connsys *devm_mtk_connsys_get(struct device *dev);
int mtk_connsys_power_on(struct mtk_connsys *cs, enum mtk_connsys_radio radio);
void mtk_connsys_power_off(struct mtk_connsys *cs, enum mtk_connsys_radio radio);
phys_addr_t mtk_connsys_emi_base(struct mtk_connsys *cs);
#else
static inline struct mtk_connsys *devm_mtk_connsys_get(struct device *dev)
{
	return ERR_PTR(-ENODEV);
}

static inline int mtk_connsys_power_on(struct mtk_connsys *cs,
				       enum mtk_connsys_radio radio)
{
	return -ENODEV;
}

static inline void mtk_connsys_power_off(struct mtk_connsys *cs,
					 enum mtk_connsys_radio radio)
{
}

static inline phys_addr_t mtk_connsys_emi_base(struct mtk_connsys *cs)
{
	return 0;
}
#endif

#endif /* __SOC_MEDIATEK_MTK_CONNSYS_H */
