/* SPDX-License-Identifier: GPL-2.0 */
// Copyright (c) 2022 MediaTek Inc.

#ifndef __MTK_CAM_SENINF_DEF_H__
#define __MTK_CAM_SENINF_DEF_H__

#include <linux/bitfield.h>

#define SENINF_VC_MAXCNT		8
#define SENINF_DEF_PIXEL_MODE		3
#define SENINF_CLK_MARGIN_IN_PERCENT	0
#define HW_BUF_EFFECT			10
#define ISP_CLK_LOW			273000000

/* 0: let the PHY layer pick its default timing */
#define SENINF_CPHY_SETTLE_DELAY_DT	0
#define SENINF_DPHY_SETTLE_DELAY_DT	0
#define SENINF_SETTLE_DELAY_CK		0
#define SENINF_HS_TRAIL_PARAMETER	0

#define DEFAULT_WIDTH			1600
#define DEFAULT_HEIGHT			1200

enum CSI_PORT {
	CSI_PORT_0 = 0,
	CSI_PORT_1,
	CSI_PORT_2,
	CSI_PORT_3,
	CSI_PORT_4,
	CSI_PORT_5,
	CSI_PORT_0A,
	CSI_PORT_0B,
	CSI_PORT_1A,
	CSI_PORT_1B,
	CSI_PORT_2A,
	CSI_PORT_2B,
	CSI_PORT_3A,
	CSI_PORT_3B,
	CSI_PORT_4A,
	CSI_PORT_4B,
	CSI_PORT_5A,
	CSI_PORT_5B,
	CSI_PORT_MAX_NUM,
};

#define SENINF_CSI_PORT_NAMES \
	"0", "1", "2", "3", "4", "5", \
	"0A", "0B", "1A", "1B", "2A", "2B", \
	"3A", "3B", "4A", "4B", "5A", "5B"

/* sizes of the largest supported seninf block */
#define SENINF_NUM		12
#define SENINF_MUX_NUM		22
#define SENINF_CAM_MUX_NUM	41

enum SENINF_SOURCE_ENUM {
	CSI2 = 0x0,
	TEST_MODEL = 0x1,
	MIPI_SENSOR = 0x8,
};

enum {
	CLK_CAM_SENINF = 0,
	CLK_CAM_CAM,
	CLK_CAM_CAMTG,
	CLK_TOP_SENINF,
	CLK_TOP_SENINF1,
	CLK_TOP_SENINF2,
	CLK_TOP_SENINF3,
	CLK_TOP_CAMTM,
	CLK_MAXCNT,
};

#define SENINF_CLK_NAMES \
	"cam-seninf", \
	"cam-cam", \
	"cam-camtg", \
	"seninf0", \
	"seninf1", \
	"seninf2", \
	"seninf3", \
	"camtm"

#endif /* __MTK_CAM_SENINF_DEF_H__ */
