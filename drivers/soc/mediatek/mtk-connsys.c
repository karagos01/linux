// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6878 connectivity subsystem (connsys) power control
 *
 * The connsys hosts the WiFi, BT, GPS and FM subsystems ("D-die") which share
 * an external RF chip (the "A-die", MT6637 here) connected through a
 * dedicated SPI master. Before any radio can start, the connsys power domain
 * has to be enabled, the A-die configured and calibrated, and the conn_infra
 * low power logic set up. This driver takes care of that and lets the radio
 * drivers power the connsys on and off.
 *
 * Register sequences follow the MediaTek downstream conninfra driver.
 */

#include <linux/arm-smccc.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/soc/mediatek/mtk-connsys.h>
#include <linux/soc/mediatek/mtk_sip_svc.h>
#include <linux/spinlock.h>

/* conn_infra_rgu_on */
#define CONN_INFRA_OFF_TOP_PWR_CTL	0x00000
#define WFSYS_ON_TOP_PWR_CTL		0x00010
#define BGFSYS_ON_TOP_PWR_CTL		0x00020
#define SYSRAM_HWCTL_PDN		0x00050
#define SYSRAM_HWCTL_SLP		0x00054
#define CO_EXT_MEM_HWCTL_PDN		0x00070
#define CO_EXT_MEM_HWCTL_SLP		0x00074
#define PWR_CTL_KEY			GENMASK(31, 16)
#define PWR_CTL_ACK_MASK		BIT(6)

/* conn_infra_cfg_on */
#define CONN_CFG_ON_ADIE_CTL		0x01010
#define ADIE_RSTB			BIT(0)
#define CONN_INFRA_CFG_PWRCTRL0		0x01200
#define PWRCTRL0_HW_CONTROL		BIT(0)
#define PWRCTRL0_RESTORE_FIRST		BIT(6)
#define PWRCTRL0_SLPPROT_RDY_CHK	GENMASK(15, 12)
#define PWRCTRL0_SLPPROT_RDY_DIS_CHK	GENMASK(19, 16)
#define CONN_INFRA_CFG_RC_CTL_0		0x01340
#define CONN_INFRA_CFG_RC_CTL_1		0x01348
#define CONN_INFRA_CFG_RC_CTL_0_GPS	0x01350
#define CONN_INFRA_CFG_RC_CTL_0_BT	0x01360
#define CONN_INFRA_CFG_RC_CTL_0_WF	0x01370
#define CONN_INFRA_CFG_RC_CTL_0_TOP	0x01380
#define CONN_INFRA_CFG_RC_CTL_1_OFS	0x4
#define CONN_INFRA_CFG_OSC_CTL_0	0x01300
#define CONN_INFRA_CFG_OSC_CTL_1	0x01304

/* conn_wt_slp_ctl_reg: A-die power control */
#define WB_SLP_CTL			0x03004
#define WB_BG_ADDR(n)			(0x03010 + 4 * ((n) - 1))
#define WB_BG_ON(n)			(0x03030 + 4 * ((n) - 1))
#define WB_BG_OFF(n)			(0x03050 + 4 * ((n) - 1))
#define WB_WF_CK_ADDR			0x03070
#define WB_WF_WAKE_ADDR			0x03074
#define WB_WF_ZPS_ADDR			0x03078
#define WB_BT_CK_ADDR			0x0307c
#define WB_BT_WAKE_ADDR			0x03080
#define WB_TOP_CK_ADDR			0x03084
#define WB_GPS_CK_ADDR			0x03088
#define WB_WF_B0_CMD_ADDR		0x0308c
#define WB_WF_B1_CMD_ADDR		0x03090
#define WB_GPS_RFBUF_ADDR		0x03094
#define WB_GPS_L5_EN_ADDR		0x03098
#define WB_SLP_TOP_CK_0			0x03120
#define TOP_CK_EN			BIT(0)
#define TOP_CK_BUSY			BIT(1)

/* conn_infra_bus_cr_on */
#define VON_BUS_TIMEOUT			0x0e024
#define ON_BUS_TIMEOUT			0x0e038
#define VON_BUS_DCM_CTL_1		0x0e104
#define OFF_BUS_DCM_CTL_1		0x0e110
#define CKSYS_CK_CTRL			0x0e120
#define BUS_TIMEOUT_EN			BIT(0)
#define BUS_TIMEOUT_VAL			GENMASK(10, 3)

/* conn_infra_cfg */
#define CONN_HW_VER			0x11000
#define CONN_EFUSE			0x11020
#define CMDBT_FETCH_START_ADDR0		0x11050
#define EMI_CTL_0			0x11100

/* conn_infra_clkgen_top */
#define CKGEN_BPLL_DIV_1		0x12000
#define CKGEN_BPLL_DIV_2		0x12004
#define CKGEN_WPLL_DIV_1		0x12008
#define CKGEN_WPLL_DIV_2		0x1200c
#define CKGEN_RFSPI_INST2_SW_MODE	0x12034
#define CKGEN_RFSPI_CK_CTRL		0x12038
#define CKGEN_BUS			0x12050
#define CKGEN_COEX_0_SET		0x12064
#define CKGEN_COEX_1_SET		0x12074

/* conn_afe_ctl */
#define AFE_RG_DIG_EN_01		0x41000
#define AFE_RG_DIG_EN_02		0x41004
#define AFE_RG_DIG_EN_03		0x41008
#define AFE_RG_DIG_TOP_01		0x4100c
#define AFE_RG_PLL_STB_TIME		0x410f4
#define AFE_RG_DIG_EN_04		0x4115c

/* conn_rf_spi_mst_reg */
#define RFSPI_STA			0x42000
#define RFSPI_FM_CTRL			0x4200c

/* conn_infra_bus_cr */
#define CONN2AP_EMI_PATH_ADDR_START	0x4b070
#define CONN2AP_EMI_PATH_ADDR_END	0x4b074
#define OFF_BUS_TIMEOUT			0x4b024
#define LIGHT_SECURITY_CTRL		0x4b374

/* conn_infra_off_debug_ctrl_ao */
#define OFF_DEBUG_CTRL0			0x4d000

/* conn_infra sysram */
#define SYSRAM_BASE			0x50000
#define SYSRAM_SIZE			0x5000
#define SYSRAM_CMDBT			0x50000
#define SYSRAM_CMDBT_FETCH		0x18050200
#define SYSRAM_A_DIE_CHIP_ID		0x53800
#define SYSRAM_A_DIE_EFUSE(n)		(0x53804 + 4 * (n))
#define SYSRAM_D_DIE_EFUSE		0x53820
#define SYSRAM_RADIO_STATUS		0x53834
#define SYSRAM_CLOCK_TYPE		0x5383c

/* conn_semaphore, master 2 (AP) */
#define SEMA_M2_OWN_STA(n)		(0x72000 + 4 * (n))
#define SEMA_M2_OWN_REL(n)		(0x72200 + 4 * (n))
#define SEMA_RFSPI			12
#define SEMA_SYSRAM			23

#define CONN_REG_SIZE			0x80000

/* SRCLKEN_RC central configuration */
#define SRCLKEN_RC_CENTRAL_CFG1		0x4
#define SRCLKEN_RC_EN			BIT(0)

/* TOPCKGEN */
#define TOPCKGEN_CONN_CK_CTRL		0x504
#define CONN_VON_HOST_CK_REQ_EN		BIT(3)

/* A-die (MT6637) registers, accessed through the RF SPI master */
#define ADIE_TOP_PULL			0x00c
#define ADIE_CHIP_ID			0x02c
#define ADIE_RG_TOP_THADC_BG		0x034
#define ADIE_RG_TOP_THADC		0x038
#define ADIE_WRI_CTR2			0x064
#define ADIE_ENCAL_WFBT_IF_SW_01	0x070
#define ADIE_SMCTK11			0x0bc
#define ADIE_EFUSE_CTRL			0x108
#define EFUSE_CTRL_VALID		BIT(29)
#define EFUSE_CTRL_KICK			BIT(30)
#define EFUSE_CTRL_ADDR			GENMASK(25, 16)
#define ADIE_EFUSE_RDATA(n)		(0x130 + 4 * (n))
#define ADIE_FMCTL4			0x160
#define ADIE_BT0_BG_TIME		0x304
#define ADIE_BT0_ISO_TIME		0x308
#define ADIE_WF0_BG			0x344
#define ADIE_WF1_RFLDO_TIME		0x358
#define ADIE_WF1_BG			0x374
#define ADIE_WF2_POS_01			0x380
#define ADIE_WF2_TIME_COUNT_01		0x384
#define ADIE_WF2_POS_02			0x390
#define ADIE_WF2_TIME_COUNT_02		0x394
#define ADIE_BT0_XOBUF_TIME		0x430
#define ADIE_BT0_CKEN_TIME		0x438
#define ADIE_WF0_TOP_01			0x754
#define ADIE_WF1_TOP_01			0x758
#define ADIE_TOP_XO_02			0xb04
#define ADIE_TOP_XO_03			0xb08
#define ADIE_TOP_XO_07			0xb18
#define ADIE_WF1_RFDIG_OFF_TIME		0xdf8
#define ADIE_WAKEUPSLEEP_PON_OFF	0xffc

#define ADIE_ID_MT6637			0x6637
#define ADIE_HW_VER_E2			0x8a01

#define CONN_HW_VER_MT6878		0x02050401

/* Secure monitor call configuring the connsys to AP memory remapping */
#define MTK_SIP_KERNEL_CONNSYS_CONTROL	MTK_SIP_SMC_CMD(0x534)
#define SMC_CONNSYS_EMI_SET_REMAPPING	70

enum mtk_connsys_spi {
	CONNSYS_SPI_WF,
	CONNSYS_SPI_BT,
	CONNSYS_SPI_TOP,
};

struct mtk_connsys_spi_ctrl {
	u32 busy;
	u32 addr;
	u32 rd_fmt;
	u32 wr_fmt;
};

static const struct mtk_connsys_spi_ctrl mtk_connsys_spi[] = {
	[CONNSYS_SPI_WF] = { BIT(1), 0x42010, 0x3000, 0x2000 },
	[CONNSYS_SPI_BT] = { BIT(2), 0x42020, 0x5000, 0x4000 },
	[CONNSYS_SPI_TOP] = { BIT(5), 0x42050, 0xb000, 0xa000 },
};

static const char * const mtk_connsys_common_supplies[] = {
	"vrfio18", "vcn13",
};

static const char * const mtk_connsys_rf_supplies[] = {
	"vcn33-1", "vcn33-2", "vant18",
};

/* conn_infra command batch program, backs up and restores conn_infra across deep sleep */
static const u32 mt6878_cmdbt[] = {
	0x16001040, 0x16011801, 0x16100001, 0x16110000, 0x31000100, 0x16000400,
	0x16011805, 0x16100a00, 0x16111805, 0x1620008a, 0x16210000, 0xcccccccc,
	0x16001044, 0x16011801, 0x1610fffe, 0x1611ffff, 0x31000100, 0x16002060,
	0x16011801, 0x1610ffff, 0x1611ffff, 0x31000100, 0x16002070, 0x16011801,
	0x1610ffff, 0x16110000, 0x31000100, 0x1600c000, 0x16011801, 0x16100000,
	0x16110000, 0x31000100, 0x1600c008, 0x16011801, 0x16100000, 0x16110000,
	0x31000100, 0x1600d00c, 0x16011801, 0x161000bf, 0x16110000, 0x31000100,
	0x160007ac, 0x16011805, 0x16100dac, 0x16111805, 0x16200033, 0x16210000,
	0xcccccccc, 0x160007ac, 0x16011805, 0x16100dac, 0x16111805, 0x16200033,
	0x16210000, 0xcccccccc, 0x16000788, 0x16011805, 0x16100d88, 0x16111805,
	0x16200009, 0x16210000, 0xcccccccc, 0x160007ac, 0x16011805, 0x16100dac,
	0x16111805, 0x16200033, 0x16210000, 0xcccccccc, 0x16000630, 0x16011805,
	0x16100c30, 0x16111805, 0x1620001b, 0x16210000, 0xcccccccc, 0x1600c0d0,
	0x16011801, 0x16100001, 0x16110000, 0x31000100, 0x1600069c, 0x16011805,
	0x16100c9c, 0x16111805, 0x1620003b, 0x16210000, 0xcccccccc, 0x16000628,
	0x16011805, 0x16100c28, 0x16111805, 0x16200002, 0x16210000, 0xcccccccc,
	0x1600d000, 0x16011804, 0x16100200, 0x16110000, 0x31000100, 0x1600d000,
	0x16011804, 0x16100200, 0x161107f4, 0x31000100, 0x1600d000, 0x16011804,
	0x16100204, 0x161107f4, 0x31000100, 0x1600d000, 0x16011804, 0x1610020c,
	0x161107f4, 0x31000100, 0x1600d000, 0x16011804, 0x1610001c, 0x161107f4,
	0x31000100, 0x06000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x16000400, 0x16011805, 0x16100a00, 0x16111805,
	0x1620008c, 0x16210000, 0xbbbbbbbb, 0x16002060, 0x16011801, 0x1610ffff,
	0x1611ffff, 0x31000100, 0x16002070, 0x16011801, 0x1610ffff, 0x16110000,
	0x31000100, 0x16000630, 0x16011805, 0x16100c30, 0x16111805, 0x16200056,
	0x16210000, 0xbbbbbbbb, 0x1600c000, 0x16011801, 0x16100000, 0x16110000,
	0x31000100, 0x1600c008, 0x16011801, 0x16100000, 0x16110000, 0x31000100,
	0x16000788, 0x16011805, 0x16100d88, 0x16111805, 0x16200009, 0x16210000,
	0xbbbbbbbb, 0x1600d00c, 0x16011801, 0x161000bf, 0x16110000, 0x31000100,
	0x160007ac, 0x16011805, 0x16100dac, 0x16111805, 0x1620003d, 0x16210000,
	0xbbbbbbbb, 0x06000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x18041014, 0x180410f4,
	0x1804100c, 0x18041040, 0x18041044, 0x18041100, 0x1804115c, 0x18041008,
	0x18011050, 0x18011100, 0x18011104, 0x18012000, 0x18012004, 0x18012008,
	0x18012020, 0x18012050, 0x1804b000, 0x1804b004, 0x1804b008, 0x1804b010,
	0x1804b024, 0x1804b070, 0x1804b074, 0x1804b350, 0x1804b354, 0x1804b358,
	0x1804b35c, 0x1804b360, 0x1804b364, 0x1804b368, 0x1804b36c, 0x1804b370,
	0x1804b374, 0x1804b378, 0x1804b37c, 0x1804b380, 0x1804b384, 0x1804b388,
	0x1804b38c, 0x1804b390, 0x1804b394, 0x1804b398, 0x1804b39c, 0x1804b3a0,
	0x1804b3a4, 0x1804b3a8, 0x1804b3ac, 0x1804b3b0, 0x1804b3b4, 0x1804b3b8,
	0x1804b3bc, 0x1804b3c0, 0x1804b3c4, 0x1804b3c8, 0x1804b3cc, 0x1804b3d0,
	0x1804b3d4, 0x1804b3d8, 0x1804b3dc, 0x1804b3e0, 0x1804b3e4, 0x1804b3e8,
	0x1804b3ec, 0x18040004, 0x18040008, 0x1804000c, 0x18040010, 0x18040014,
	0x18040018, 0x18040000, 0x1804001c, 0x18040020, 0x18040024, 0x18040028,
	0x18044004, 0x18044008, 0x1804400c, 0x18044010, 0x18044014, 0x18044018,
	0x18044000, 0x1804401c, 0x18044020, 0x18044024, 0x18044028, 0x18042008,
	0x18012030, 0x18012038, 0x18046008, 0x18012038, 0x18012034, 0x18016308,
	0x1801630c, 0x18016310, 0x18016314, 0x18016318, 0x1801631c, 0x18016320,
	0x18016324, 0x18016328, 0x1801632c, 0x18016330, 0x18016334, 0x18016338,
	0x1801633c, 0x18016340, 0x18016344, 0x18016348, 0x1801634c, 0x18016350,
	0x18016354, 0x18016358, 0x1801635c, 0x18016360, 0x18016364, 0x180030a8,
	0x18003120, 0x18003124, 0x18003128, 0x1800312c, 0x18003130, 0x18003134,
	0x180070a8, 0x18007120, 0x18007124, 0x18007128, 0x1800712c, 0x18007130,
	0x18007134, 0x18041024, 0x18041028, 0x1804102c, 0x18041030, 0x18041034,
	0x18016374, 0x18016378, 0x1801637c, 0x18016380, 0x18012060, 0x18012070,
	0x1801c000, 0x1801c004, 0x1801c008, 0x1801c0c0, 0x1801c0c8, 0x1801c0d4,
	0x1801c0d8, 0x1801c0dc, 0x1801c0e0, 0x1801c0e4, 0x1801c0e8, 0x1801c0ec,
	0x1801c0f0, 0x1801c0f4, 0x1801c0f8, 0x1801c0fc, 0x1801c100, 0x1801c104,
	0x1801c108, 0x1801c10c, 0x1801c110, 0x1801c114, 0x1801c118, 0x1801c11c,
	0x1801c120, 0x1801c124, 0x1801c128, 0x1801c0d0, 0x1801c200, 0x1801c204,
	0x1801c208, 0x1801c214, 0x1801c230, 0x1801c238, 0x1801c23c, 0x1801c2c0,
	0x1801c2c4, 0x1801c2c8, 0x1801c2cc, 0x1801c2d0, 0x1801c2d4, 0x1801c2d8,
	0x1801c2dc, 0x1801c400, 0x1801c404, 0x1801c408, 0x1801c464, 0x1801c468,
	0x1801c46c, 0x1801c4d0, 0x1801c530, 0x1801c534, 0x1801c58c, 0x1801c590,
	0x1801c594, 0x1801c598, 0x1801c59c, 0x1801c5a0, 0x1801c5a4, 0x1801c5a8,
	0x1801c5ac, 0x1801c5b0, 0x1801c5b4, 0x1801c5b8, 0x1801c5bc, 0x1801c5c0,
	0x1801c5c4, 0x1801c5c8, 0x1801c5cc, 0x1801c5d0, 0x1801c5d4, 0x1801c5d8,
	0x1801c5e0, 0x1801c5e4, 0x1801c654, 0x1801c754, 0x1801c7b0, 0x1801c7b4,
	0x1801c7c0, 0x1801c7e0, 0x1801c800, 0x1801c830, 0x1801c254, 0x1801c258,
	0x1801c294, 0x1801c298, 0x1801d00c, 0x1801d008, 0x1801d024, 0x1801d028,
	0x1801d02c, 0x1801d03c, 0x1801d04c, 0x1801d054, 0x1801d058, 0x1801d000,
	0x1801d004, 0x1801b04c, 0x1801b0bc, 0x1801b070, 0x1801b074, 0x1801b500,
	0x1801a000, 0x1801a004, 0x1801a008, 0x1801a00c, 0x1801a018, 0x1801a024,
	0x1801a028, 0x1801a02c, 0x1801a110, 0x1801a200, 0x1801a204, 0x1801a208,
	0x1801a20c, 0x1801a210, 0x1801a214, 0x1801a218, 0x1801a21c, 0x1801a228,
	0x1801a22c, 0x1801a230, 0x1801a234, 0x1801a238, 0x1801a23c, 0x1801a240,
	0x1801a244, 0x1801a248, 0x1801a304, 0x1801a30c, 0x1801a310, 0x1801a314,
	0x1801a604, 0x1801a608, 0x1801a610, 0x1801a620, 0x1801a630, 0x1801a640,
	0x1801a650, 0x1801a660, 0x1801c030, 0x1804107c, 0x18041098, 0x18041078,
	0x18041094, 0x18042320, 0x18042300, 0x18042310, 0x18042304, 0x18042314,
	0x18042308, 0x18042318, 0x1804230c, 0x1804231c, 0x18042060, 0x18042064,
};

struct mtk_connsys {
	struct device *dev;
	void __iomem *base;
	void __iomem *srclken_rc;
	struct regmap *topckgen;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_adie_cfg;
	struct pinctrl_state *pins_sleep;
	struct regulator_bulk_data common_supplies[ARRAY_SIZE(mtk_connsys_common_supplies)];
	struct regulator_bulk_data rf_supplies[ARRAY_SIZE(mtk_connsys_rf_supplies)];
	phys_addr_t emi_base;
	bool xtal_52m;

	/* Protects the radio state */
	struct mutex lock;
	unsigned long radios;
	bool wbg_calibrated;

	/* Serializes RF SPI accesses, taken with the RF SPI HW semaphore held */
	spinlock_t spi_lock;
	u32 adie_id;
};

static u32 cs_rr(struct mtk_connsys *cs, u32 reg)
{
	return readl(cs->base + reg);
}

static void cs_wr(struct mtk_connsys *cs, u32 reg, u32 val)
{
	writel(val, cs->base + reg);
}

static void cs_rmw(struct mtk_connsys *cs, u32 reg, u32 mask, u32 val)
{
	writel((readl(cs->base + reg) & ~mask) | (val & mask), cs->base + reg);
}

static void cs_set(struct mtk_connsys *cs, u32 reg, u32 bits)
{
	cs_rmw(cs, reg, bits, bits);
}

static void cs_clr(struct mtk_connsys *cs, u32 reg, u32 bits)
{
	cs_rmw(cs, reg, bits, 0);
}

static int mtk_connsys_sema_get(struct mtk_connsys *cs, int idx)
{
	u32 val;

	/* Reading the ownership status requests the semaphore */
	return readl_poll_timeout_atomic(cs->base + SEMA_M2_OWN_STA(idx), val,
					 val & BIT(0), 1, 1000);
}

static void mtk_connsys_sema_put(struct mtk_connsys *cs, int idx)
{
	cs_wr(cs, SEMA_M2_OWN_REL(idx), 1);
}

static int mtk_connsys_spi_wait(struct mtk_connsys *cs, u32 busy)
{
	u32 val;

	return readl_poll_timeout_atomic(cs->base + RFSPI_STA, val,
					 !(val & busy), 50, 5000);
}

/* Must be called with the RF SPI semaphore held */
static int mtk_connsys_spi_read(struct mtk_connsys *cs, enum mtk_connsys_spi sys,
				u32 addr, u32 *val)
{
	const struct mtk_connsys_spi_ctrl *spi = &mtk_connsys_spi[sys];
	int ret;

	ret = mtk_connsys_spi_wait(cs, spi->busy);
	if (ret)
		return ret;

	cs_wr(cs, spi->addr, spi->rd_fmt | addr);
	cs_wr(cs, spi->addr + 0x4, 0);
	udelay(1);

	ret = mtk_connsys_spi_wait(cs, spi->busy);
	if (ret)
		return ret;

	*val = cs_rr(cs, spi->addr + 0x8);

	return 0;
}

static int mtk_connsys_spi_write(struct mtk_connsys *cs, enum mtk_connsys_spi sys,
				 u32 addr, u32 val)
{
	const struct mtk_connsys_spi_ctrl *spi = &mtk_connsys_spi[sys];
	int ret;

	ret = mtk_connsys_spi_wait(cs, spi->busy);
	if (ret)
		return ret;

	cs_wr(cs, spi->addr, spi->wr_fmt | addr);
	cs_wr(cs, spi->addr + 0x4, val);
	udelay(1);

	return mtk_connsys_spi_wait(cs, spi->busy);
}

static int mtk_connsys_spi_lock(struct mtk_connsys *cs, unsigned long *flags)
{
	int ret;

	spin_lock_irqsave(&cs->spi_lock, *flags);
	ret = mtk_connsys_sema_get(cs, SEMA_RFSPI);
	if (ret) {
		spin_unlock_irqrestore(&cs->spi_lock, *flags);
		dev_err(cs->dev, "Cannot get the RF SPI semaphore\n");
	}

	return ret;
}

static void mtk_connsys_spi_unlock(struct mtk_connsys *cs, unsigned long *flags)
{
	mtk_connsys_sema_put(cs, SEMA_RFSPI);
	spin_unlock_irqrestore(&cs->spi_lock, *flags);
}

struct mtk_connsys_adie_reg {
	u16 addr;
	u32 val;
};

/* Writes a list of A-die TOP registers with the RF SPI semaphore held */
static int mtk_connsys_adie_write_list(struct mtk_connsys *cs,
				       const struct mtk_connsys_adie_reg *regs,
				       int n)
{
	int i, ret;

	for (i = 0; i < n; i++) {
		ret = mtk_connsys_spi_write(cs, CONNSYS_SPI_TOP, regs[i].addr, regs[i].val);
		if (ret)
			return ret;
	}

	return 0;
}

static int mtk_connsys_adie_top_ck(struct mtk_connsys *cs, bool on)
{
	u32 val;
	int ret;

	if (on)
		cs_set(cs, WB_SLP_TOP_CK_0, TOP_CK_EN);
	else
		cs_clr(cs, WB_SLP_TOP_CK_0, TOP_CK_EN);

	ret = readl_poll_timeout_atomic(cs->base + WB_SLP_TOP_CK_0, val,
					!(val & TOP_CK_BUSY), 5, 500);
	if (ret)
		dev_err(cs->dev, "A-die top clock %s timeout\n", on ? "on" : "off");

	return ret;
}

static int mtk_connsys_adie_write(struct mtk_connsys *cs, u16 addr, u32 val)
{
	struct mtk_connsys_adie_reg reg = { addr, val };
	unsigned long flags;
	int ret;

	ret = mtk_connsys_spi_lock(cs, &flags);
	if (ret)
		return ret;

	ret = mtk_connsys_adie_write_list(cs, &reg, 1);
	mtk_connsys_spi_unlock(cs, &flags);

	return ret;
}

static void mtk_connsys_ddie_cfg(struct mtk_connsys *cs)
{
	/* conn_von_top_host_ck_req under HW control */
	cs_rmw(cs, CKSYS_CK_CTRL, 0x3, 0x3);
	memset_io(cs->base + SYSRAM_BASE, 0, SYSRAM_SIZE);
	cs_wr(cs, CONN2AP_EMI_PATH_ADDR_START, 0x04000000);
	cs_wr(cs, CONN2AP_EMI_PATH_ADDR_END, 0x43ffffff);

	cs_wr(cs, SYSRAM_D_DIE_EFUSE, cs_rr(cs, CONN_EFUSE));

	cs_wr(cs, SYSRAM_HWCTL_PDN, 0x0);
	cs_wr(cs, SYSRAM_HWCTL_SLP, 0x3);
	cs_wr(cs, CO_EXT_MEM_HWCTL_PDN, 0x0);
	cs_wr(cs, CO_EXT_MEM_HWCTL_SLP, 0x1);
	cs_rmw(cs, CONN_INFRA_OFF_TOP_PWR_CTL, PWR_CTL_KEY | PWR_CTL_ACK_MASK,
	       FIELD_PREP(PWR_CTL_KEY, 0x494e));
}

/* A-die register addresses used by the conn_infra sleep controller */
static void mtk_connsys_wt_slp_cfg(struct mtk_connsys *cs)
{
	cs_rmw(cs, WB_WF_CK_ADDR, 0x0fff0fff, 0x0af40a04);
	cs_rmw(cs, WB_WF_WAKE_ADDR, 0x0fff0fff, 0x00a00090);
	cs_rmw(cs, WB_WF_ZPS_ADDR, 0x0fff0fff, 0x009c008c);
	cs_rmw(cs, WB_BT_CK_ADDR, 0xfff, 0xa08);
	cs_rmw(cs, WB_BT_WAKE_ADDR, 0xfff, 0x094);
	cs_rmw(cs, WB_TOP_CK_ADDR, 0xfff, 0xa00);
	cs_rmw(cs, WB_GPS_CK_ADDR, 0x0fff0fff, 0x0afc0a0c);
	cs_rmw(cs, WB_WF_B0_CMD_ADDR, 0xfff, 0x0f0);
	cs_rmw(cs, WB_WF_B1_CMD_ADDR, 0xfff, 0x0f4);
	cs_rmw(cs, WB_GPS_RFBUF_ADDR, 0xfff, 0x0fc);
	cs_rmw(cs, WB_GPS_L5_EN_ADDR, 0xfff, 0x0f8);
}

/* Replaces bits of an A-die register with a bit field of @src */
static int mtk_connsys_adie_rmw_field(struct mtk_connsys *cs, u16 addr, u32 src,
				      int reg_shift, int src_shift, int width)
{
	u32 mask = GENMASK(reg_shift + width - 1, reg_shift);
	u32 val;
	int ret;

	ret = mtk_connsys_spi_read(cs, CONNSYS_SPI_TOP, addr, &val);
	if (ret)
		return ret;

	val &= ~mask;
	val |= ((src >> src_shift) << reg_shift) & mask;

	return mtk_connsys_spi_write(cs, CONNSYS_SPI_TOP, addr, val);
}

static int mtk_connsys_adie_efuse(struct mtk_connsys *cs)
{
	u32 ctrl, efuse[4];
	int i, ret, retry;

	ret = mtk_connsys_spi_read(cs, CONNSYS_SPI_TOP, ADIE_EFUSE_CTRL, &ctrl);
	if (ret)
		return ret;
	ctrl &= ~EFUSE_CTRL_KICK;
	mtk_connsys_spi_write(cs, CONNSYS_SPI_TOP, ADIE_EFUSE_CTRL, ctrl);

	/* Read the first 16 bytes */
	mtk_connsys_spi_read(cs, CONNSYS_SPI_TOP, ADIE_EFUSE_CTRL, &ctrl);
	ctrl &= ~0x43ff00c0;
	ctrl |= EFUSE_CTRL_KICK | FIELD_PREP(EFUSE_CTRL_ADDR, 0);
	mtk_connsys_spi_write(cs, CONNSYS_SPI_TOP, ADIE_EFUSE_CTRL, ctrl);

	for (retry = 0; retry < 70; retry++) {
		ret = mtk_connsys_spi_read(cs, CONNSYS_SPI_TOP, ADIE_EFUSE_CTRL, &ctrl);
		if (ret || !(ctrl & EFUSE_CTRL_KICK))
			break;
		udelay(30);
	}

	if (ret || !(ctrl & EFUSE_CTRL_VALID)) {
		dev_warn(cs->dev, "A-die efuse is not valid\n");
		return 0;
	}

	for (i = 0; i < 4; i++) {
		mtk_connsys_spi_read(cs, CONNSYS_SPI_TOP, ADIE_EFUSE_RDATA(i), &efuse[i]);
		cs_wr(cs, SYSRAM_A_DIE_EFUSE(i), efuse[i]);
	}

	/* Thermal sensor calibration */
	if (efuse[1] & BIT(7)) {
		mtk_connsys_adie_rmw_field(cs, ADIE_RG_TOP_THADC_BG, efuse[1], 12, 3, 4);
		mtk_connsys_adie_rmw_field(cs, ADIE_RG_TOP_THADC, efuse[1], 23, 0, 3);
	}
	if (efuse[1] & BIT(15))
		mtk_connsys_adie_rmw_field(cs, ADIE_RG_TOP_THADC, efuse[1], 26, 13, 2);

	return 0;
}

static const struct mtk_connsys_adie_reg mt6637_part2[] = {
	{ ADIE_WF2_POS_01, 0x000e8002 },
	{ ADIE_WF2_POS_02, 0x000e8002 },
	{ ADIE_WF2_TIME_COUNT_01, 0x00002008 },
	{ ADIE_WF2_TIME_COUNT_02, 0x00002008 },
	{ ADIE_WF1_RFLDO_TIME, 0x000f000b },
	{ ADIE_WF1_RFDIG_OFF_TIME, 0x000c000c },
	{ ADIE_BT0_BG_TIME, 0x00020003 },
	{ ADIE_BT0_ISO_TIME, 0x0001000b },
	{ ADIE_BT0_XOBUF_TIME, 0x00010004 },
	{ ADIE_BT0_CKEN_TIME, 0x00020003 },
	{ ADIE_WAKEUPSLEEP_PON_OFF, 0x00150008 },
	{ ADIE_TOP_XO_02, 0x88000000 },
	{ ADIE_TOP_XO_03, 0x00000000 },
	{ ADIE_TOP_XO_07, 0x00000000 },
	{ ADIE_ENCAL_WFBT_IF_SW_01, 0x00000005 },
	{ ADIE_FMCTL4, 0x00000035 },
	{ ADIE_TOP_PULL, 0xa6aaaa44 },
};

static const struct mtk_connsys_adie_reg mt6637_e2_bg[] = {
	{ ADIE_WF0_BG, 0x0000e008 },
	{ ADIE_WF1_BG, 0x0000e008 },
};

static const struct mtk_connsys_adie_reg mt6637_part3[] = {
	{ ADIE_WF0_TOP_01, 0x000e440a },
	{ ADIE_WF1_TOP_01, 0x000e401a },
};

static int mtk_connsys_adie_cfg_locked(struct mtk_connsys *cs)
{
	static const struct mtk_connsys_adie_reg part1[] = {
		{ ADIE_WRI_CTR2, 0x00007700 },
		/* WiFi low power command: DBDC mode, legacy interface */
		{ ADIE_SMCTK11, 0x00000021 },
	};
	int ret;

	cs_set(cs, CONN_CFG_ON_ADIE_CTL, ADIE_RSTB);

	ret = mtk_connsys_spi_read(cs, CONNSYS_SPI_TOP, ADIE_CHIP_ID, &cs->adie_id);
	if (ret)
		return ret;

	if ((cs->adie_id >> 16) != ADIE_ID_MT6637) {
		dev_err(cs->dev, "Unsupported A-die 0x%08x\n", cs->adie_id);
		return -ENODEV;
	}
	cs_wr(cs, SYSRAM_A_DIE_CHIP_ID, cs->adie_id);

	ret = mtk_connsys_adie_write_list(cs, part1, ARRAY_SIZE(part1));
	if (ret)
		return ret;

	cs_rmw(cs, RFSPI_FM_CTRL, 0x80ff, 0);

	ret = mtk_connsys_adie_efuse(cs);
	if (ret)
		return ret;

	ret = mtk_connsys_adie_write_list(cs, mt6637_part2, ARRAY_SIZE(mt6637_part2));
	if (ret)
		return ret;

	if ((cs->adie_id & 0xffff) == ADIE_HW_VER_E2) {
		ret = mtk_connsys_adie_write_list(cs, mt6637_e2_bg, ARRAY_SIZE(mt6637_e2_bg));
		if (ret)
			return ret;
	}

	return mtk_connsys_adie_write_list(cs, mt6637_part3, ARRAY_SIZE(mt6637_part3));
}

static void mtk_connsys_adie_sleep_cfg(struct mtk_connsys *cs)
{
	static const u32 bg_addr[] = { 0xa03c, 0xa03c, 0xab00, 0xab00, 0xab00, 0xa0c8 };
	static const u32 bg_on[] = { 0, 0, 0xc0000000, 0xf8000000, 0xfc000000, 0 };
	static const u32 bg_off[] = { 0x57400000, 0x57400000, 0, 0, 0xc4000000, 0x1 };
	int i;

	cs_rmw(cs, WB_SLP_CTL, 0x1f, 0x6);
	for (i = 0; i < ARRAY_SIZE(bg_addr); i++)
		cs_wr(cs, WB_BG_ADDR(i + 1), bg_addr[i]);
	for (i = 0; i < ARRAY_SIZE(bg_on); i++)
		cs_wr(cs, WB_BG_ON(i + 1), bg_on[i]);
	for (i = 0; i < ARRAY_SIZE(bg_off); i++)
		cs_wr(cs, WB_BG_OFF(i + 1), bg_off[i]);
}

static int mtk_connsys_adie_cfg(struct mtk_connsys *cs)
{
	unsigned long flags;
	int ret;

	cs_wr(cs, SYSRAM_CLOCK_TYPE, cs->xtal_52m ? 2 : 1);

	/* CONN_WF_CTRL2 tells the A-die the clock frequency while it is reset */
	if (cs->pins_adie_cfg) {
		ret = pinctrl_select_state(cs->pinctrl, cs->pins_adie_cfg);
		if (ret)
			return ret;
	}

	ret = mtk_connsys_adie_top_ck(cs, true);
	if (ret)
		return ret;

	ret = mtk_connsys_spi_lock(cs, &flags);
	if (ret)
		goto out_ck;

	ret = mtk_connsys_adie_cfg_locked(cs);
	mtk_connsys_spi_unlock(cs, &flags);
	if (ret)
		goto out_ck;

	if (cs->pins_adie_cfg)
		pinctrl_select_state(cs->pinctrl, cs->pins_default);

	mtk_connsys_adie_sleep_cfg(cs);

	return mtk_connsys_adie_top_ck(cs, false);

out_ck:
	mtk_connsys_adie_top_ck(cs, false);
	return ret;
}

static int mtk_connsys_afe_wbg_cal(struct mtk_connsys *cs)
{
	int ret;

	ret = mtk_connsys_adie_write(cs, ADIE_ENCAL_WFBT_IF_SW_01, 0x80000000);
	if (ret)
		return ret;

	cs_set(cs, AFE_RG_DIG_EN_01, BIT(0));
	udelay(60);
	cs_clr(cs, AFE_RG_DIG_EN_01, BIT(0));

	cs_set(cs, AFE_RG_DIG_EN_03, BIT(21));
	udelay(30);
	cs_set(cs, AFE_RG_DIG_EN_03, BIT(20));
	udelay(60);

	cs_set(cs, AFE_RG_DIG_EN_01, BIT(21) | BIT(20) | BIT(19));
	udelay(800);
	cs_clr(cs, AFE_RG_DIG_EN_01, BIT(21) | BIT(20) | BIT(19));
	cs_clr(cs, AFE_RG_DIG_EN_03, BIT(21) | BIT(20));

	return mtk_connsys_adie_write(cs, ADIE_ENCAL_WFBT_IF_SW_01, 0x00000005);
}

static int mtk_connsys_afe_cfg(struct mtk_connsys *cs)
{
	int ret;

	cs_clr(cs, AFE_RG_DIG_EN_04, BIT(31));
	cs_rmw(cs, AFE_RG_DIG_EN_03, 0x7700, 0x4400);

	/* The calibration result is kept in an always-on domain */
	if (!cs->wbg_calibrated) {
		ret = mtk_connsys_afe_wbg_cal(cs);
		if (ret)
			return ret;
		cs->wbg_calibrated = true;
	}

	/* PLL settings for a 26 MHz reference */
	cs_rmw(cs, AFE_RG_PLL_STB_TIME, 0x7fff7fff, 0x03140521);
	cs_rmw(cs, AFE_RG_DIG_EN_02, 0x300cf, 0x30047);
	cs_rmw(cs, AFE_RG_DIG_TOP_01, 0x78000, 0x68000);

	return 0;
}

static int mtk_connsys_set_radio_status(struct mtk_connsys *cs, unsigned long radios)
{
	int ret;

	ret = mtk_connsys_sema_get(cs, SEMA_SYSRAM);
	if (ret)
		return ret;

	cs_rmw(cs, SYSRAM_RADIO_STATUS, 0xf, radios);
	mtk_connsys_sema_put(cs, SEMA_SYSRAM);

	return 0;
}

static int mtk_connsys_adie_wf_xobuf(struct mtk_connsys *cs, bool on)
{
	const struct mtk_connsys_adie_reg regs[] = {
		{ ADIE_TOP_XO_02, on ? 0xfe000000 : 0x88000000 },
		{ ADIE_TOP_XO_03, on ? 0xe0000000 : 0x00000000 },
	};
	unsigned long flags;
	int ret;

	ret = mtk_connsys_adie_top_ck(cs, true);
	if (ret)
		return ret;

	ret = mtk_connsys_spi_lock(cs, &flags);
	if (!ret) {
		ret = mtk_connsys_adie_write_list(cs, regs, ARRAY_SIZE(regs));
		mtk_connsys_spi_unlock(cs, &flags);
	}

	mtk_connsys_adie_top_ck(cs, false);

	return ret;
}

static bool mtk_connsys_rc_mode(struct mtk_connsys *cs)
{
	return readl(cs->srclken_rc + SRCLKEN_RC_CENTRAL_CFG1) & SRCLKEN_RC_EN;
}

static void mtk_connsys_low_power_cfg(struct mtk_connsys *cs)
{
	u32 rc_ctl_0[] = {
		CONN_INFRA_CFG_RC_CTL_0_GPS, CONN_INFRA_CFG_RC_CTL_0_BT,
		CONN_INFRA_CFG_RC_CTL_0_WF, CONN_INFRA_CFG_RC_CTL_0_TOP,
	};
	int i;

	cs_rmw(cs, CONN_INFRA_CFG_PWRCTRL0, PWRCTRL0_SLPPROT_RDY_CHK, 0x1000);
	cs_rmw(cs, CONN_INFRA_CFG_PWRCTRL0, PWRCTRL0_SLPPROT_RDY_DIS_CHK, 0xd0000);

	if (mtk_connsys_rc_mode(cs)) {
		/* Oscillator requests go through the SRCLKEN_RC */
		for (i = 0; i < ARRAY_SIZE(rc_ctl_0); i++) {
			cs_wr(cs, rc_ctl_0[i] + CONN_INFRA_CFG_RC_CTL_1_OFS, 0x02080706);
			cs_clr(cs, rc_ctl_0[i], BIT(15));
		}
		cs_set(cs, CONN_INFRA_CFG_RC_CTL_1, BIT(3));
		udelay(30);
		cs_set(cs, CONN_INFRA_CFG_RC_CTL_1, BIT(0));
		cs_set(cs, CONN_INFRA_CFG_RC_CTL_1, BIT(1));
		cs_set(cs, CONN_INFRA_CFG_RC_CTL_1, BIT(2));
		cs_set(cs, CONN_INFRA_CFG_RC_CTL_0, BIT(4));
		cs_set(cs, CONN_INFRA_CFG_RC_CTL_0, BIT(5));
		cs_set(cs, CONN_INFRA_CFG_RC_CTL_0, BIT(6));
		cs_set(cs, CONN_INFRA_CFG_RC_CTL_1, BIT(20));
		cs_clr(cs, CONN_INFRA_CFG_RC_CTL_0, BIT(31));
		cs_clr(cs, CONN_INFRA_CFG_RC_CTL_0, BIT(0));
		cs_rmw(cs, CONN_INFRA_CFG_OSC_CTL_0, 0xffffff, 0x080706);
		cs_clr(cs, CONN_INFRA_CFG_OSC_CTL_1, BIT(16));
		cs_set(cs, CONN_INFRA_CFG_RC_CTL_0, BIT(30));
		cs_rmw(cs, CONN_INFRA_CFG_RC_CTL_1, 0xf0, 0xc0);
		cs_set(cs, CONN_INFRA_CFG_RC_CTL_1, BIT(21));
	} else {
		cs_clr(cs, CONN_INFRA_CFG_RC_CTL_0, BIT(7));
		cs_rmw(cs, CONN_INFRA_CFG_OSC_CTL_0, 0xffffff, 0x080706);
		cs_clr(cs, CONN_INFRA_CFG_OSC_CTL_1, BIT(16));
	}

	cs_rmw(cs, BGFSYS_ON_TOP_PWR_CTL, PWR_CTL_KEY | PWR_CTL_ACK_MASK,
	       FIELD_PREP(PWR_CTL_KEY, 0x4254));
	cs_rmw(cs, WFSYS_ON_TOP_PWR_CTL, PWR_CTL_KEY | PWR_CTL_ACK_MASK,
	       FIELD_PREP(PWR_CTL_KEY, 0x5746));

	cs_set(cs, EMI_CTL_0, BIT(18));
	cs_rmw(cs, EMI_CTL_0, 0x7ff0, 0x3ff0);
	cs_set(cs, EMI_CTL_0, BIT(15));

	/* Bus clock gating */
	cs_set(cs, OFF_BUS_DCM_CTL_1, BIT(14) | BIT(16));
	cs_set(cs, VON_BUS_DCM_CTL_1, BIT(14) | BIT(16));

	cs_rmw(cs, LIGHT_SECURITY_CTRL, 0xc63, 0xc63);

	/* Bus timeouts */
	cs_rmw(cs, OFF_BUS_TIMEOUT, BUS_TIMEOUT_VAL, 0x240);
	cs_set(cs, OFF_BUS_TIMEOUT, BUS_TIMEOUT_EN);
	cs_rmw(cs, ON_BUS_TIMEOUT, BUS_TIMEOUT_VAL, 0x188);
	cs_set(cs, ON_BUS_TIMEOUT, BUS_TIMEOUT_EN);
	cs_rmw(cs, VON_BUS_TIMEOUT, BUS_TIMEOUT_VAL, 0x498);
	cs_set(cs, VON_BUS_TIMEOUT, BUS_TIMEOUT_EN);

	cs_set(cs, OFF_DEBUG_CTRL0, BIT(9));
	cs_rmw(cs, OFF_DEBUG_CTRL0, 0xffff0000, 0x11df0000);
	cs_set(cs, OFF_DEBUG_CTRL0, BIT(2) | BIT(3) | BIT(4));
	cs_clr(cs, OFF_DEBUG_CTRL0, BIT(9));

	/* Bus clocks from BPLL/WPLL */
	cs_set(cs, CKGEN_BPLL_DIV_1, BIT(0));
	cs_set(cs, CKGEN_BPLL_DIV_2, BIT(0));
	cs_set(cs, CKGEN_WPLL_DIV_1, BIT(0));
	cs_set(cs, CKGEN_WPLL_DIV_2, BIT(0));

	/* RF SPI clock from BPLL/13 */
	cs_clr(cs, CKGEN_RFSPI_CK_CTRL, BIT(7));
	cs_set(cs, CKGEN_RFSPI_CK_CTRL, BIT(6));
	cs_set(cs, CKGEN_RFSPI_CK_CTRL, BIT(4));
	cs_set(cs, CKGEN_RFSPI_INST2_SW_MODE, BIT(0));

	cs_wr(cs, CKGEN_COEX_0_SET, 0xffffffff);
	cs_rmw(cs, CKGEN_COEX_1_SET, 0xffff, 0xffff);

	/* conn_infra bus clock under HW control */
	cs_clr(cs, CKGEN_BUS, BIT(0));

	cs_wr(cs, CMDBT_FETCH_START_ADDR0, SYSRAM_CMDBT_FETCH);
	for (i = 0; i < ARRAY_SIZE(mt6878_cmdbt); i++)
		cs_wr(cs, SYSRAM_CMDBT + 4 * i, mt6878_cmdbt[i]);

	cs_set(cs, CONN_INFRA_CFG_PWRCTRL0, PWRCTRL0_RESTORE_FIRST);

	/* Let conn_infra enter deep sleep, it has to be woken up from now on */
	cs_set(cs, CONN_INFRA_CFG_PWRCTRL0, PWRCTRL0_HW_CONTROL);
}

static int mtk_connsys_emi_remap(struct mtk_connsys *cs)
{
	struct arm_smccc_res res;

	arm_smccc_smc(MTK_SIP_KERNEL_CONNSYS_CONTROL, SMC_CONNSYS_EMI_SET_REMAPPING,
		      0, 0, 0, 0, 0, 0, &res);
	if (res.a0) {
		dev_err(cs->dev, "EMI remapping failed: %ld\n", (long)res.a0);
		return -EIO;
	}

	return 0;
}

static int mtk_connsys_hw_power_on(struct mtk_connsys *cs)
{
	u32 val;
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(cs->common_supplies), cs->common_supplies);
	if (ret)
		return ret;

	/* Host clock request must not keep the connsys clock always on */
	regmap_clear_bits(cs->topckgen, TOPCKGEN_CONN_CK_CTRL, CONN_VON_HOST_CK_REQ_EN);

	ret = pinctrl_select_state(cs->pinctrl, cs->pins_default);
	if (ret)
		goto err_regulators;

	ret = pm_runtime_resume_and_get(cs->dev);
	if (ret)
		goto err_pins;
	mdelay(5);

	ret = readl_poll_timeout(cs->base + CONN_HW_VER, val, val == CONN_HW_VER_MT6878,
				 1000, 11000);
	if (ret) {
		dev_err(cs->dev, "conn_infra not responding (0x%08x)\n", val);
		goto err_pm;
	}

	ret = mtk_connsys_emi_remap(cs);
	if (ret)
		goto err_pm;

	mtk_connsys_ddie_cfg(cs);
	mtk_connsys_wt_slp_cfg(cs);

	ret = mtk_connsys_adie_cfg(cs);
	if (ret)
		goto err_pm;

	ret = mtk_connsys_afe_cfg(cs);
	if (ret)
		goto err_pm;

	return 0;

err_pm:
	pm_runtime_put_sync(cs->dev);
err_pins:
	pinctrl_select_state(cs->pinctrl, cs->pins_sleep);
err_regulators:
	regulator_bulk_disable(ARRAY_SIZE(cs->common_supplies), cs->common_supplies);
	return ret;
}

static void mtk_connsys_hw_power_off(struct mtk_connsys *cs)
{
	regulator_bulk_disable(ARRAY_SIZE(cs->rf_supplies), cs->rf_supplies);
	pm_runtime_put_sync(cs->dev);
	pinctrl_select_state(cs->pinctrl, cs->pins_sleep);
	msleep(1);
	regulator_bulk_disable(ARRAY_SIZE(cs->common_supplies), cs->common_supplies);
}

/**
 * mtk_connsys_power_on - power on the connsys for a radio
 * @cs: connsys handle
 * @radio: radio subsystem that is going to be used
 *
 * On return the conn_infra is configured and allowed to sleep: the radio
 * driver has to wake it up before accessing its registers.
 */
int mtk_connsys_power_on(struct mtk_connsys *cs, enum mtk_connsys_radio radio)
{
	bool first;
	int ret = 0;

	mutex_lock(&cs->lock);

	if (cs->radios & BIT(radio))
		goto out;

	if (radio != MTK_CONNSYS_WIFI && radio != MTK_CONNSYS_BT) {
		ret = -EOPNOTSUPP;
		goto out;
	}

	/*
	 * The conn_infra, A-die and the analog supplies are shared by all
	 * radios, so they are brought up for whichever radio comes up first
	 * and torn down again when the last one is released.
	 */
	first = !cs->radios;
	if (first) {
		ret = mtk_connsys_hw_power_on(cs);
		if (ret)
			goto out;
	}

	ret = mtk_connsys_set_radio_status(cs, cs->radios | BIT(radio));
	if (ret)
		goto err;

	/* The XO buffer is only needed for the WiFi subsystem */
	if (radio == MTK_CONNSYS_WIFI) {
		ret = mtk_connsys_adie_wf_xobuf(cs, true);
		if (ret)
			goto err;
	}

	if (first) {
		mtk_connsys_low_power_cfg(cs);

		ret = regulator_bulk_enable(ARRAY_SIZE(cs->rf_supplies), cs->rf_supplies);
		if (ret)
			goto err;
	}

	cs->radios |= BIT(radio);
	dev_dbg(cs->dev, "powered on for radio %d, A-die 0x%08x\n", radio, cs->adie_id);
	goto out;

err:
	/* Only unwind the shared bring-up when this radio started it */
	if (first) {
		pm_runtime_put_sync(cs->dev);
		pinctrl_select_state(cs->pinctrl, cs->pins_sleep);
		regulator_bulk_disable(ARRAY_SIZE(cs->common_supplies), cs->common_supplies);
	}
out:
	mutex_unlock(&cs->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mtk_connsys_power_on);

/**
 * mtk_connsys_power_off - release the connsys for a radio
 * @cs: connsys handle
 * @radio: radio subsystem that is not used anymore
 *
 * The radio driver must have turned its subsystem off before.
 */
void mtk_connsys_power_off(struct mtk_connsys *cs, enum mtk_connsys_radio radio)
{
	mutex_lock(&cs->lock);

	if (cs->radios & BIT(radio)) {
		cs->radios &= ~BIT(radio);
		if (!cs->radios)
			mtk_connsys_hw_power_off(cs);
	}

	mutex_unlock(&cs->lock);
}
EXPORT_SYMBOL_GPL(mtk_connsys_power_off);

/**
 * mtk_connsys_emi_base - physical address of the connsys memory
 * @cs: connsys handle
 *
 * The bootloader loads the radio firmwares there, which also use it at
 * runtime.
 */
phys_addr_t mtk_connsys_emi_base(struct mtk_connsys *cs)
{
	return cs->emi_base;
}
EXPORT_SYMBOL_GPL(mtk_connsys_emi_base);

static void mtk_connsys_put_device(void *data)
{
	put_device(data);
}

/**
 * devm_mtk_connsys_get - get the connsys referenced by a radio device
 * @dev: radio device, with a "mediatek,connsys" phandle
 */
struct mtk_connsys *devm_mtk_connsys_get(struct device *dev)
{
	struct platform_device *pdev;
	struct device_node *np;
	struct mtk_connsys *cs;
	int ret;

	np = of_parse_phandle(dev->of_node, "mediatek,connsys", 0);
	if (!np)
		return ERR_PTR(-ENODEV);

	pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!pdev)
		return ERR_PTR(-EPROBE_DEFER);

	ret = devm_add_action_or_reset(dev, mtk_connsys_put_device, &pdev->dev);
	if (ret)
		return ERR_PTR(ret);

	cs = platform_get_drvdata(pdev);
	if (!cs)
		return ERR_PTR(-EPROBE_DEFER);

	if (!device_link_add(dev, &pdev->dev, DL_FLAG_AUTOREMOVE_CONSUMER))
		return ERR_PTR(-EINVAL);

	return cs;
}
EXPORT_SYMBOL_GPL(devm_mtk_connsys_get);

static int mtk_connsys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_connsys *cs;
	struct resource res;
	struct device_node *np;
	int i, ret;

	cs = devm_kzalloc(dev, sizeof(*cs), GFP_KERNEL);
	if (!cs)
		return -ENOMEM;

	cs->dev = dev;
	mutex_init(&cs->lock);
	spin_lock_init(&cs->spi_lock);
	cs->xtal_52m = of_property_read_bool(dev->of_node, "mediatek,xtal-52mhz");

	/* Also mapped by the radio drivers, so don't request it */
	cs->base = devm_platform_get_and_ioremap_resource(pdev, 0, NULL);
	if (IS_ERR(cs->base))
		return PTR_ERR(cs->base);

	cs->srclken_rc = devm_platform_ioremap_resource_byname(pdev, "srclken-rc");
	if (IS_ERR(cs->srclken_rc))
		return PTR_ERR(cs->srclken_rc);

	cs->topckgen = syscon_regmap_lookup_by_phandle(dev->of_node, "mediatek,topckgen");
	if (IS_ERR(cs->topckgen))
		return dev_err_probe(dev, PTR_ERR(cs->topckgen), "Cannot get topckgen\n");

	np = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!np)
		return dev_err_probe(dev, -EINVAL, "No memory region\n");
	ret = of_address_to_resource(np, 0, &res);
	of_node_put(np);
	if (ret)
		return ret;
	cs->emi_base = res.start;

	for (i = 0; i < ARRAY_SIZE(cs->common_supplies); i++)
		cs->common_supplies[i].supply = mtk_connsys_common_supplies[i];
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(cs->common_supplies), cs->common_supplies);
	if (ret)
		return dev_err_probe(dev, ret, "Cannot get supplies\n");

	for (i = 0; i < ARRAY_SIZE(cs->rf_supplies); i++)
		cs->rf_supplies[i].supply = mtk_connsys_rf_supplies[i];
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(cs->rf_supplies), cs->rf_supplies);
	if (ret)
		return dev_err_probe(dev, ret, "Cannot get RF supplies\n");

	cs->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(cs->pinctrl))
		return dev_err_probe(dev, PTR_ERR(cs->pinctrl), "Cannot get pinctrl\n");

	cs->pins_default = pinctrl_lookup_state(cs->pinctrl, PINCTRL_STATE_DEFAULT);
	if (IS_ERR(cs->pins_default))
		return dev_err_probe(dev, PTR_ERR(cs->pins_default), "No default pins\n");

	cs->pins_sleep = pinctrl_lookup_state(cs->pinctrl, PINCTRL_STATE_SLEEP);
	if (IS_ERR(cs->pins_sleep))
		return dev_err_probe(dev, PTR_ERR(cs->pins_sleep), "No sleep pins\n");

	cs->pins_adie_cfg = pinctrl_lookup_state(cs->pinctrl, "adie-cfg");
	if (IS_ERR(cs->pins_adie_cfg))
		cs->pins_adie_cfg = NULL;

	/* The pins have to stay in GPIO mode while powered off */
	ret = pinctrl_select_state(cs->pinctrl, cs->pins_sleep);
	if (ret)
		return ret;

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, cs);

	return 0;
}

static const struct of_device_id mtk_connsys_of_match[] = {
	{ .compatible = "mediatek,mt6878-connsys" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mtk_connsys_of_match);

static struct platform_driver mtk_connsys_driver = {
	.probe = mtk_connsys_probe,
	.driver = {
		.name = "mtk-connsys",
		.of_match_table = mtk_connsys_of_match,
		/* Radio drivers keep a reference to the connsys */
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(mtk_connsys_driver);

MODULE_DESCRIPTION("MediaTek connectivity subsystem power control");
MODULE_LICENSE("GPL");
