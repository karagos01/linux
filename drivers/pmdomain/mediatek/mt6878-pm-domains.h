/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __PMDOMAIN_MEDIATEK_MT6878_PM_DOMAINS_H
#define __PMDOMAIN_MEDIATEK_MT6878_PM_DOMAINS_H

#include <dt-bindings/power/mediatek,mt6878-power.h>
#include "mtk-pm-domains.h"

/* INFRACFG_AO bus protection, in set/clr/sta register groups */
#define MT6878_TOP_AXI_PROT_EN_INFRASYS_SET_0		0xc44
#define MT6878_TOP_AXI_PROT_EN_INFRASYS_CLR_0		0xc48
#define MT6878_TOP_AXI_PROT_EN_INFRASYS_STA_0		0xc4c
#define MT6878_TOP_AXI_PROT_EN_INFRASYS_SET_1		0xc54
#define MT6878_TOP_AXI_PROT_EN_INFRASYS_CLR_1		0xc58
#define MT6878_TOP_AXI_PROT_EN_INFRASYS_STA_1		0xc5c
#define MT6878_TOP_AXI_PROT_EN_MCU_SET_0		0xc94
#define MT6878_TOP_AXI_PROT_EN_MCU_CLR_0		0xc98
#define MT6878_TOP_AXI_PROT_EN_MCU_STA_0		0xc9c

#define MT6878_TOP_AXI_PROT_EN_MCU_0_CONN		BIT(1)
#define MT6878_TOP_AXI_PROT_EN_INFRASYS_1_CONN		BIT(12)
#define MT6878_TOP_AXI_PROT_EN_MCU_0_CONN_2ND		BIT(0)
#define MT6878_TOP_AXI_PROT_EN_INFRASYS_0_CONN		BIT(8)

/*
 * MT6878 keeps the power acks in the PWR_CON register of each domain
 * itself rather than in separate status registers.
 */
#define MT6878_PWR_ACK				BIT(30)
#define MT6878_PWR_ACK_2ND			BIT(31)

static const struct scpsys_domain_data scpsys_domain_data_mt6878[] = {
	[MT6878_POWER_DOMAIN_CONN] = {
		.name = "conn",
		.sta_mask = MT6878_PWR_ACK,
		.sta2nd_mask = MT6878_PWR_ACK_2ND,
		.ctl_offs = 0xe04,
		.pwr_sta_offs = 0xe04,
		.pwr_sta2nd_offs = 0xe04,
		.bp_cfg = {
			BUS_PROT_WR_IGN(INFRA, MT6878_TOP_AXI_PROT_EN_MCU_0_CONN,
					MT6878_TOP_AXI_PROT_EN_MCU_SET_0,
					MT6878_TOP_AXI_PROT_EN_MCU_CLR_0,
					MT6878_TOP_AXI_PROT_EN_MCU_STA_0),
			BUS_PROT_WR_IGN(INFRA, MT6878_TOP_AXI_PROT_EN_INFRASYS_1_CONN,
					MT6878_TOP_AXI_PROT_EN_INFRASYS_SET_1,
					MT6878_TOP_AXI_PROT_EN_INFRASYS_CLR_1,
					MT6878_TOP_AXI_PROT_EN_INFRASYS_STA_1),
			BUS_PROT_WR_IGN(INFRA, MT6878_TOP_AXI_PROT_EN_MCU_0_CONN_2ND,
					MT6878_TOP_AXI_PROT_EN_MCU_SET_0,
					MT6878_TOP_AXI_PROT_EN_MCU_CLR_0,
					MT6878_TOP_AXI_PROT_EN_MCU_STA_0),
			BUS_PROT_WR_IGN(INFRA, MT6878_TOP_AXI_PROT_EN_INFRASYS_0_CONN,
					MT6878_TOP_AXI_PROT_EN_INFRASYS_SET_0,
					MT6878_TOP_AXI_PROT_EN_INFRASYS_CLR_0,
					MT6878_TOP_AXI_PROT_EN_INFRASYS_STA_0),
		},
		.caps = MTK_SCPD_KEEP_DEFAULT_OFF,
	},
};

static const struct scpsys_soc_data mt6878_scpsys_data = {
	.domains_data = scpsys_domain_data_mt6878,
	.num_domains = ARRAY_SIZE(scpsys_domain_data_mt6878),
};

#endif /* __PMDOMAIN_MEDIATEK_MT6878_PM_DOMAINS_H */
