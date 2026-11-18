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
#define MT6878_TOP_AXI_PROT_EN_PERISYS_SET_0		0xc84
#define MT6878_TOP_AXI_PROT_EN_PERISYS_CLR_0		0xc88
#define MT6878_TOP_AXI_PROT_EN_PERISYS_STA_0		0xc8c
#define MT6878_TOP_AXI_PROT_EN_MMSYS_SET_0		0xc14
#define MT6878_TOP_AXI_PROT_EN_MMSYS_CLR_0		0xc18
#define MT6878_TOP_AXI_PROT_EN_MMSYS_STA_0		0xc1c

/* VLPCFG bus protection, reached through the "mediatek,infracfg-nao" regmap */
#define MT6878_VLP_AXI_PROT_EN_SET			0x214
#define MT6878_VLP_AXI_PROT_EN_CLR			0x218
#define MT6878_VLP_AXI_PROT_EN_STA			0x220

#define MT6878_TOP_AXI_PROT_EN_MCU_0_CONN		BIT(1)
#define MT6878_TOP_AXI_PROT_EN_INFRASYS_1_CONN		BIT(12)
#define MT6878_TOP_AXI_PROT_EN_MCU_0_CONN_2ND		BIT(0)
#define MT6878_TOP_AXI_PROT_EN_INFRASYS_0_CONN		BIT(8)
#define MT6878_TOP_AXI_PROT_EN_PERISYS_0_AUDIO		BIT(6)
#define MT6878_TOP_AXI_PROT_EN_PERISYS_0_SSUSB		BIT(7)
#define MT6878_TOP_AXI_PROT_EN_MMSYS_0_DISP		(BIT(18) | GENMASK(1, 0))
#define MT6878_VLP_AXI_PROT_EN_MM_PROC			BIT(8)
#define MT6878_VLP_AXI_PROT_EN_MM_PROC_2ND		GENMASK(10, 9)

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
		.caps = MTK_SCPD_KEEP_DEFAULT_OFF | MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT6878_POWER_DOMAIN_AUDIO] = {
		.name = "audio",
		.sta_mask = MT6878_PWR_ACK,
		.sta2nd_mask = MT6878_PWR_ACK_2ND,
		.ctl_offs = 0xe18,
		.pwr_sta_offs = 0xe18,
		.pwr_sta2nd_offs = 0xe18,
		.sram_pdn_bits = BIT(8),
		.sram_pdn_ack_bits = BIT(12),
		.bp_cfg = {
			BUS_PROT_WR_IGN(INFRA, MT6878_TOP_AXI_PROT_EN_PERISYS_0_AUDIO,
					MT6878_TOP_AXI_PROT_EN_PERISYS_SET_0,
					MT6878_TOP_AXI_PROT_EN_PERISYS_CLR_0,
					MT6878_TOP_AXI_PROT_EN_PERISYS_STA_0),
		},
	},
	/*
	 * DISP and MM_PROC sit below MM_INFRA, which is switched by the hardware
	 * voter (see scpsys_hwv_domain_data_mt6878): their nodes name it as their
	 * parent with a "power-domains" property.
	 */
	[MT6878_POWER_DOMAIN_DISP] = {
		.name = "disp",
		.sta_mask = MT6878_PWR_ACK,
		.sta2nd_mask = MT6878_PWR_ACK_2ND,
		.ctl_offs = 0xe70,
		.pwr_sta_offs = 0xe70,
		.pwr_sta2nd_offs = 0xe70,
		.sram_pdn_bits = BIT(8),
		.sram_pdn_ack_bits = BIT(12),
		.bp_cfg = {
			BUS_PROT_WR_IGN(INFRA, MT6878_TOP_AXI_PROT_EN_MMSYS_0_DISP,
					MT6878_TOP_AXI_PROT_EN_MMSYS_SET_0,
					MT6878_TOP_AXI_PROT_EN_MMSYS_CLR_0,
					MT6878_TOP_AXI_PROT_EN_MMSYS_STA_0),
		},
	},
	/* The multimedia processor (VCP), unused by Linux: only switched off */
	[MT6878_POWER_DOMAIN_MM_PROC] = {
		.name = "mm-proc",
		.sta_mask = MT6878_PWR_ACK,
		.sta2nd_mask = MT6878_PWR_ACK_2ND,
		.ctl_offs = 0xe7c,
		.pwr_sta_offs = 0xe7c,
		.pwr_sta2nd_offs = 0xe7c,
		.sram_pdn_bits = BIT(9),
		.sram_pdn_ack_bits = BIT(13),
		.bp_cfg = {
			BUS_PROT_WR_IGN(INFRA_NAO, MT6878_VLP_AXI_PROT_EN_MM_PROC,
					MT6878_VLP_AXI_PROT_EN_SET,
					MT6878_VLP_AXI_PROT_EN_CLR,
					MT6878_VLP_AXI_PROT_EN_STA),
			BUS_PROT_WR_IGN(INFRA_NAO, MT6878_VLP_AXI_PROT_EN_MM_PROC_2ND,
					MT6878_VLP_AXI_PROT_EN_SET,
					MT6878_VLP_AXI_PROT_EN_CLR,
					MT6878_VLP_AXI_PROT_EN_STA),
		},
		.caps = MTK_SCPD_SRAM_ISO | MTK_SCPD_SRAM_PDN_INVERTED,
	},
	[MT6878_POWER_DOMAIN_SSUSB] = {
		.name = "ssusb",
		.sta_mask = MT6878_PWR_ACK,
		.sta2nd_mask = MT6878_PWR_ACK_2ND,
		.ctl_offs = 0xea8,
		.pwr_sta_offs = 0xea8,
		.pwr_sta2nd_offs = 0xea8,
		.sram_pdn_bits = BIT(8),
		.sram_pdn_ack_bits = BIT(12),
		.bp_cfg = {
			BUS_PROT_WR_IGN(INFRA, MT6878_TOP_AXI_PROT_EN_PERISYS_0_SSUSB,
					MT6878_TOP_AXI_PROT_EN_PERISYS_SET_0,
					MT6878_TOP_AXI_PROT_EN_PERISYS_CLR_0,
					MT6878_TOP_AXI_PROT_EN_PERISYS_STA_0),
		},
	},
};

/*
 * MM_INFRA has to be switched by the hardware voter: only then are its
 * registers (the ones the kernel cannot see included) saved and restored
 * around the power cycle. Switching it off resets the SoC while DISP is still
 * on, while the GCE clocks are not released through the voter, or while the
 * SMI gate of MMINFRA_CONFIG is closed.
 */
static const struct scpsys_hwv_domain_data scpsys_hwv_domain_data_mt6878[] = {
	[MT6878_HWV_POWER_DOMAIN_MM_INFRA] = {
		.name = "mm-infra",
		.set = 0x0198,
		.clr = 0x019c,
		.done = 0x141c,
		.en = 0x1410,
		.set_sta = 0x146c,
		.clr_sta = 0x1470,
		.setclr_bit = 0,
		/*
		 * The power controller of DISP and MM_PROC is a consumer whose
		 * parent device never gets a driver, so the sync_state of this
		 * one never comes. DISP keeps the domain on as long as needed.
		 */
		.caps = MTK_SCPD_INFRA_PWR_CTL | MTK_SCPD_NO_STAY_ON,
	},
};

static const struct scpsys_soc_data mt6878_scpsys_data = {
	.domains_data = scpsys_domain_data_mt6878,
	.num_domains = ARRAY_SIZE(scpsys_domain_data_mt6878),
};

static const struct scpsys_soc_data mt6878_scpsys_hwv_data = {
	.hwv_domains_data = scpsys_hwv_domain_data_mt6878,
	.num_hwv_domains = ARRAY_SIZE(scpsys_hwv_domain_data_mt6878),
	.type = SCPSYS_MTCMOS_TYPE_HW_VOTER,
};

#endif /* __PMDOMAIN_MEDIATEK_MT6878_PM_DOMAINS_H */
