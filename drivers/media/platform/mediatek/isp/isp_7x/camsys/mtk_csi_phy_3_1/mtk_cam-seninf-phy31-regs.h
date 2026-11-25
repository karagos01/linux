/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Register layout of the MediaTek CSI PHY 3.1 seninf generation (MT6878)
 *
 * Copyright (c) 2026 Jan Benedikt <karagos01@gmail.com>
 *
 * Register names follow the vendor driver where it names them; registers
 * the vendor driver only touches by offset are named after their offset.
 */
#ifndef __MTK_CAM_SENINF_PHY31_REGS_H__
#define __MTK_CAM_SENINF_PHY31_REGS_H__

#include <linux/bits.h>

/* "ana-rx" region: one 0x8000 block per CSI port */
#define ANA_CSI_STRIDE			0x8000
#define ANA_CSI_RX_A			0x0000
#define ANA_CSI_RX_B			0x1000
#define ANA_DPHY_TOP			0x2000
#define ANA_CPHY_TOP			0x3000
#define ANA_MAC_TOP			0x4000
#define ANA_MAC_A			0x5000
#define ANA_MAC_B			0x6000

/* "base" region */
#define IF_CTRL_BASE			0x0200
#define IF_CSI2_BASE			0x0a00
#define IF_MUX_BASE			0x0d00
#define IF_STRIDE			0x1000
#define IF_CAM_MUX_BASE			0x17000
#define IF_CAM_MUX_STRIDE		0x40
#define IF_CAM_MUX_GCSR			0x17f00

/* CDPHY RX analog, one per half port */
#define CDPHY_RX_ANA_0			0x0000
#define  RG_CSI_BG_CORE_EN		BIT(0)
#define  RG_CSI_BG_LPF_EN		BIT(1)
#define  RG_CSI_DPHY_L0_CKSEL		BIT(6)
#define  RG_CSI_DPHY_L1_CKSEL		BIT(7)
#define  RG_CSI_DPHY_L0_CKMODE_EN	BIT(8)
#define  RG_CSI_DPHY_L1_CKMODE_EN	BIT(9)
#define  RG_CSI_DPHY_L2_CKMODE_EN	BIT(10)
#define  RG_CSI_CPHY_T0_CDR_EN		BIT(12)
#define  RG_CSI_CPHY_T1_CDR_EN		BIT(13)
#define  RG_CSI_CPHY_T2_CDR_EN		BIT(14)
#define  RG_CSI_CPHY_EN			BIT(20)
#define CDPHY_RX_ANA_1			0x0004
#define CDPHY_RX_ANA_2			0x0008
#define CDPHY_RX_ANA_3			0x000c
#define CDPHY_RX_ANA_4			0x0010
#define CDPHY_RX_ANA_5			0x0014
#define CDPHY_RX_ANA_6			0x0018
#define CDPHY_RX_ANA_7			0x001c
#define CDPHY_RX_ANA_8			0x0020
#define  RG_CSI_LANE_LDO_EN		GENMASK(21, 16)
#define CDPHY_RX_ANA_14			0x0038
#define CDPHY_RX_ANA_SETTING_0		0x00f0
#define CDPHY_RX_ANA_SETTING_1		0x00f4
#define CDPHY_RX_ANA_64			0x0100

/* D-PHY RX digital, one per CSI port */
#define DPHY_RX_LANE_EN			0x0000
#define  DPHY_RX_LC0_EN			BIT(0)
#define  DPHY_RX_LC1_EN			BIT(1)
#define  DPHY_RX_LD0_EN			BIT(8)
#define  DPHY_RX_LD1_EN			BIT(9)
#define  DPHY_RX_LD2_EN			BIT(10)
#define  DPHY_RX_LD3_EN			BIT(11)
#define DPHY_RX_LANE_SELECT		0x0004
#define  DPHY_RX_LANE_SEL(n)		GENMASK((n) * 4 + 2, (n) * 4)
#define  DPHY_RX_LANE_SEL_UPDATE	BIT(31)
#define DPHY_RX_HS_RX_EN_SW		0x0008
#define  DPHY_RX_HS_RX_EN_SW_MASK	GENMASK(17, 12)
#define DPHY_RX_CLOCK_LANE_HS_PARAMETER(n) (0x0010 + (n) * 4)
#define  RG_DPHY_RX_LC_HS_SETTLE	GENMASK(23, 16)
#define DPHY_RX_DATA_LANE_HS_PARAMETER(n) (0x0020 + (n) * 4)
#define  RG_DPHY_RX_LD_HS_PREPARE	GENMASK(7, 0)
#define  RG_DPHY_RX_LD_HS_TRAIL		GENMASK(17, 8)
#define  RG_DPHY_RX_LD_HS_SETTLE	GENMASK(25, 18)
#define  RG_DPHY_RX_HS_SETTLE_SEL	BIT(28)
#define  RG_DPHY_RX_LD_HS_TRAIL_EN	BIT(29)
#define DPHY_RX_DESKEW_CTRL		0x0050
#define DPHY_RX_DESKEW_LANE_CTRL(n)	(0x0060 + (n) * 4)
#define  RG_DPHY_RX_DESKEW_EN		BIT(0)
#define DPHY_RX_IRQ_EN			0x0094
#define DPHY_RX_SPARE0			0x00f0
#define DPHY_RX_180			0x0180
#define DPHY_RX_204			0x0204

/* C-PHY RX digital */
#define CPHY_RX_DETECT_CTRL_POST	0x0018
#define CPHY_RX_LRTE_CTRL		0x0130

/* CSI RX MAC */
#define CSIRX_MAC_TOP_CTRL		0x0000
#define  CSIRX_MAC_TOP_RST		BIT(0)
#define  CSIRX_MAC_TOP_HALF_PORT	BIT(16)
#define CSIRX_MAC_TOP_PHY_CTRL		0x0040
#define  CSIRX_MAC_TOP_DPHY_SEL		BIT(0)
#define  CSIRX_MAC_TOP_CPHY_SEL		BIT(1)
#define  CSIRX_MAC_TOP_CPHY_MODE	GENMASK(9, 8)

#define CSIRX_MAC_CSI2_EN		0x0000
#define  CSIRX_MAC_CSI2_LANE_EN(n)	BIT(n)
#define CSIRX_MAC_CSI2_OPT		0x0004
#define  CSIRX_MAC_CSI2_CPHY_SEL	BIT(0)
#define  CSIRX_MAC_CSI2_HALF_PORT	BIT(16)
#define  CSIRX_MAC_CSI2_OPT_FIXED_SET	(BIT(1) | BIT(2) | BIT(4) | BIT(5) | BIT(7))
#define  CSIRX_MAC_CSI2_OPT_FIXED_CLR	(BIT(8) | BIT(9))
#define CSIRX_MAC_CSI2_CYCLE		0x0008
#define CSIRX_MAC_CSI2_RESYNC_MERGE_CTRL 0x0010
#define CSIRX_MAC_CSI2_HS_TRAIL		0x0014
#define CSIRX_MAC_CSI2_ERR_CTRL		0x00b8
#define CSIRX_MAC_CSI2_IRQ_EN		0x00c0
#define CSIRX_MAC_CSI2_DBG_CTRL		0x00e0
#define CSIRX_MAC_CSI2_RESYNC_CYCLE	0x00f8
#define CSIRX_MAC_CSI2_FORCE_DT		0x0104
#define CSIRX_MAC_CSI2_DT_REMAP		0x0110
#define CSIRX_MAC_CSI2_DT_CTRL(n)	(0x0300 + (n) * 0x10)

/* seninf top */
#define SENINF_TOP_CTRL2		0x0004
#define SENINF_TOP_MUX_CTRL(n)		(0x0010 + ((n) / 4) * 4)
#define  SENINF_TOP_MUX_SRC(n)		(GENMASK(4, 0) << (((n) % 4) * 8))

/* seninf interface */
#define SENINF_CTRL			0x0000
#define  SENINF_EN			BIT(0)
#define SENINF_CSI2_CTRL		0x0010
#define  SENINF_CSI2_EN			BIT(0)
#define  SENINF_CSI2_SW_RST		BIT(4)

/* seninf CSI-2 packet decoder */
#define SENINF_CSI2_RX_EN		0x0000
#define SENINF_CSI2_DI_CTRL(n)		(0x0020 + (n) * 4)
#define  SENINF_CSI2_DI_EN		BIT(0)
#define  SENINF_CSI2_DI_GRP		GENMASK(5, 4)
#define  SENINF_CSI2_DI_VC		GENMASK(12, 8)
#define  SENINF_CSI2_DI_DT		GENMASK(21, 16)
#define SENINF_CSI2_CH_CTRL(n)		(0x0060 + (n) * 4)
#define  SENINF_CSI2_CH_DI_EN(n)	BIT(8 + (n))

/* seninf mux */
#define SENINF_MUX_CTRL_0		0x0000
#define  SENINF_MUX_EN			BIT(0)
#define  SENINF_MUX_IRQ_SW_RST		BIT(1)
#define  SENINF_MUX_SW_RST		BIT(2)
#define SENINF_MUX_CTRL_1		0x0004
#define  SENINF_MUX_SRC_SEL		GENMASK(3, 0)
#define  SENINF_MUX_PIX_MODE_SEL	GENMASK(10, 8)
#define  SENINF_MUX_CROP_EN		BIT(28)
#define SENINF_MUX_OPT			0x0008
#define  SENINF_MUX_CNT_INIT_EN		BIT(3)
#define  SENINF_MUX_HSYNC_POL		BIT(16)
#define  SENINF_MUX_VSYNC_POL		BIT(17)
#define SENINF_MUX_CROP_PIX_CTRL	0x0028
#define  SENINF_MUX_CROP_START_8PIX	GENMASK(11, 0)
#define  SENINF_MUX_CROP_END_8PIX	GENMASK(27, 16)

/* cam mux, one register page per cam mux */
#define CAM_MUX_PCSR_CTRL		0x0000
#define  CAM_MUX_PCSR_SRC_SEL		GENMASK(6, 0)
#define  CAM_MUX_PCSR_EN		BIT(7)
#define  CAM_MUX_PCSR_CHK_PIX_MODE	GENMASK(10, 8)
#define  CAM_MUX_PCSR_NEXT_SRC_SEL	GENMASK(22, 16)
#define CAM_MUX_PCSR_OPT		0x0004
#define  CAM_MUX_PCSR_VC_SEL		GENMASK(4, 0)
#define  CAM_MUX_PCSR_VC_EN		BIT(7)
#define  CAM_MUX_PCSR_DT_SEL		GENMASK(13, 8)
#define  CAM_MUX_PCSR_DT_EN		BIT(15)
#define  CAM_MUX_PCSR_TAG_PAGE		GENMASK(26, 24)
#define CAM_MUX_PCSR_IRQ_STATUS		0x000c
#define  CAM_MUX_PCSR_IRQ_CLR_ALL	0x103
#define CAM_MUX_PCSR_CHK_RES		0x0014
#define  CAM_MUX_PCSR_EXP_HSIZE		GENMASK(15, 0)
#define  CAM_MUX_PCSR_EXP_VSIZE		GENMASK(31, 16)
#define CAM_MUX_PCSR_RES		0x0018
#define CAM_MUX_PCSR_TAG_VC		0x0020
#define CAM_MUX_PCSR_TAG_DT		0x0024

/* cam mux global registers */
#define CAM_MUX_GCSR_CTRL		0x0004
#define  CAM_MUX_GCSR_SW_CFG_BUSY	BIT(0)
#define  CAM_MUX_GCSR_PAGE_SEL		BIT(7)
#define  CAM_MUX_GCSR_DYN_PAGE		GENMASK(15, 14)
#define CAM_MUX_GCSR_IRQ_EN		0x0008
#define  CAM_MUX_GCSR_DROP_IRQ_EN	GENMASK(9, 8)
#define CAM_MUX_GCSR_DYN_EN(n)		(0x0090 + (n) * 8)
#define CAM_MUX_GCSR_VSYNC_IRQ_EN(n)	(0x00c0 + (n) * 4)
#define CAM_MUX_GCSR_VSYNC_IRQ_STS(n)	(0x00d8 + (n) * 4)

#endif /* __MTK_CAM_SENINF_PHY31_REGS_H__ */
