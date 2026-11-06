// SPDX-License-Identifier: GPL-2.0-only
//
// Copyright (c) 2026 MediaTek Inc.
// Copyright (c) 2026 Jan Benedikt <karagos01@gmail.com>

#include <linux/clk-provider.h>
#include <linux/platform_device.h>

#include "clk-mtk.h"
#include "clk-gate.h"

#include <dt-bindings/clock/mediatek,mt6878-clock.h>

static const struct mtk_gate_regs mm0_cg_regs = {
	.set_ofs = 0x104,
	.clr_ofs = 0x108,
	.sta_ofs = 0x100,
};

static const struct mtk_gate_regs mm1_cg_regs = {
	.set_ofs = 0x114,
	.clr_ofs = 0x118,
	.sta_ofs = 0x110,
};

#define GATE_MM0(_id, _name, _parent, _shift)	\
	GATE_MTK(_id, _name, _parent, &mm0_cg_regs, _shift, &mtk_clk_gate_ops_setclr)

#define GATE_MM1(_id, _name, _parent, _shift)	\
	GATE_MTK(_id, _name, _parent, &mm1_cg_regs, _shift, &mtk_clk_gate_ops_setclr)

static const struct mtk_gate mm_clks[] = {
	/* MM0 */
	GATE_MM0(CLK_MM_DISP_OVL0_2L, "mm_disp_ovl0_2l", "disp0_ck", 0),
	GATE_MM0(CLK_MM_DISP_OVL1_2L, "mm_disp_ovl1_2l", "disp0_ck", 1),
	GATE_MM0(CLK_MM_DISP_OVL2_2L, "mm_disp_ovl2_2l", "disp0_ck", 2),
	GATE_MM0(CLK_MM_DISP_OVL3_2L, "mm_disp_ovl3_2l", "disp0_ck", 3),
	GATE_MM0(CLK_MM_DISP_UFBC_WDMA0, "mm_disp_ufbc_wdma0", "disp0_ck", 4),
	GATE_MM0(CLK_MM_DISP_RSZ1, "mm_disp_rsz1", "disp0_ck", 5),
	GATE_MM0(CLK_MM_DISP_RSZ0, "mm_disp_rsz0", "disp0_ck", 6),
	GATE_MM0(CLK_MM_DISP_TDSHP0, "mm_disp_tdshp0", "disp0_ck", 7),
	GATE_MM0(CLK_MM_DISP_C3D0, "mm_disp_c3d0", "disp0_ck", 8),
	GATE_MM0(CLK_MM_DISP_COLOR0, "mm_disp_color0", "disp0_ck", 9),
	GATE_MM0(CLK_MM_DISP_CCORR0, "mm_disp_ccorr0", "disp0_ck", 10),
	GATE_MM0(CLK_MM_DISP_CCORR1, "mm_disp_ccorr1", "disp0_ck", 11),
	GATE_MM0(CLK_MM_DISP_AAL0, "mm_disp_aal0", "disp0_ck", 12),
	GATE_MM0(CLK_MM_DISP_GAMMA0, "mm_disp_gamma0", "disp0_ck", 13),
	GATE_MM0(CLK_MM_DISP_POSTMASK0, "mm_disp_postmask0", "disp0_ck", 14),
	GATE_MM0(CLK_MM_DISP_DITHER0, "mm_disp_dither0", "disp0_ck", 15),
	GATE_MM0(CLK_MM_DISP_TDSHP1, "mm_disp_tdshp1", "disp0_ck", 16),
	GATE_MM0(CLK_MM_DISP_C3D1, "mm_disp_c3d1", "disp0_ck", 17),
	GATE_MM0(CLK_MM_DISP_CCORR2, "mm_disp_ccorr2", "disp0_ck", 18),
	GATE_MM0(CLK_MM_DISP_CCORR3, "mm_disp_ccorr3", "disp0_ck", 19),
	GATE_MM0(CLK_MM_DISP_GAMMA1, "mm_disp_gamma1", "disp0_ck", 20),
	GATE_MM0(CLK_MM_DISP_DITHER1, "mm_disp_dither1", "disp0_ck", 21),
	GATE_MM0(CLK_MM_DISP_SPLITTER0, "mm_disp_splitter0", "disp0_ck", 22),
	GATE_MM0(CLK_MM_DISP_DSC_WRAP0, "mm_disp_dsc_wrap0", "disp0_ck", 23),
	GATE_MM0(CLK_MM_DISP_DSI0, "mm_disp_dsi0", "disp0_ck", 24),
	GATE_MM0(CLK_MM_DISP_DSI1, "mm_disp_dsi1", "disp0_ck", 25),
	GATE_MM0(CLK_MM_DISP_WDMA1, "mm_disp_wdma1", "disp0_ck", 26),
	GATE_MM0(CLK_MM_DISP_APB_BUS, "mm_disp_apb_bus", "disp0_ck", 27),
	GATE_MM0(CLK_MM_DISP_FAKE_ENG0, "mm_disp_fake_eng0", "disp0_ck", 28),
	GATE_MM0(CLK_MM_DISP_FAKE_ENG1, "mm_disp_fake_eng1", "disp0_ck", 29),
	GATE_MM0(CLK_MM_DISP_MUTEX0, "mm_disp_mutex0", "disp0_ck", 30),
	GATE_MM0(CLK_MM_SMI_COMMON, "mm_smi_common", "disp0_ck", 31),
	/* MM1 */
	GATE_MM1(CLK_MM_DSI0, "mm_dsi0_ck", "disp0_ck", 0),
	GATE_MM1(CLK_MM_DSI1, "mm_dsi1_ck", "disp0_ck", 1),
	GATE_MM1(CLK_MM_26M, "mm_26m_ck", "disp0_ck", 11),
};

static const struct mtk_clk_desc mm_desc = {
	.clks = mm_clks,
	.num_clks = ARRAY_SIZE(mm_clks),
};

static const struct platform_device_id clk_mt6878_mm_id_table[] = {
	{ .name = "clk-mt6878-mm", .driver_data = (kernel_ulong_t)&mm_desc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(platform, clk_mt6878_mm_id_table);

static struct platform_driver clk_mt6878_mm_drv = {
	.probe = mtk_clk_pdev_probe,
	.remove = mtk_clk_pdev_remove,
	.driver = {
		.name = "clk-mt6878-mm",
	},
	.id_table = clk_mt6878_mm_id_table,
};
module_platform_driver(clk_mt6878_mm_drv);

MODULE_DESCRIPTION("MediaTek MT6878 MultiMedia clocks driver");
MODULE_LICENSE("GPL");
