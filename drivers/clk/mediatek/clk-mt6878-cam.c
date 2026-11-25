// SPDX-License-Identifier: GPL-2.0-only
//
// Copyright (c) 2026 MediaTek Inc.
// Copyright (c) 2026 Jan Benedikt <karagos01@gmail.com>

#include <linux/clk-provider.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "clk-mtk.h"
#include "clk-gate.h"

#include <dt-bindings/clock/mediatek,mt6878-clock.h>

static const struct mtk_gate_regs cam_m0_cg_regs = {
	.set_ofs = 0x4,
	.clr_ofs = 0x8,
	.sta_ofs = 0x0,
};

static const struct mtk_gate_regs cam_m1_cg_regs = {
	.set_ofs = 0x50,
	.clr_ofs = 0x54,
	.sta_ofs = 0x4c,
};

#define GATE_CAM_M0(_id, _name, _parent, _shift)	\
	GATE_MTK(_id, _name, _parent, &cam_m0_cg_regs, _shift, &mtk_clk_gate_ops_setclr)

#define GATE_CAM_M1(_id, _name, _parent, _shift)	\
	GATE_MTK(_id, _name, _parent, &cam_m1_cg_regs, _shift, &mtk_clk_gate_ops_setclr)

static const struct mtk_gate cam_m_clks[] = {
	/* CAM_M0 */
	GATE_CAM_M0(CLK_CAM_MAIN_LARB13, "cam_m_larb13", "cam_ck", 0),
	GATE_CAM_M0(CLK_CAM_MAIN_LARB14, "cam_m_larb14", "cam_ck", 1),
	GATE_CAM_M0(CLK_CAM_MAIN_LARB29, "cam_m_larb29", "cam_ck", 3),
	GATE_CAM_M0(CLK_CAM_MAIN_CAM, "cam_m_cam", "cam_ck", 4),
	GATE_CAM_M0(CLK_CAM_MAIN_CAM_SUBA, "cam_m_cam_suba", "cam_ck", 5),
	GATE_CAM_M0(CLK_CAM_MAIN_CAM_SUBB, "cam_m_cam_subb", "cam_ck", 6),
	GATE_CAM_M0(CLK_CAM_MAIN_CAM_MRAW, "cam_m_cam_mraw", "cam_ck", 8),
	GATE_CAM_M0(CLK_CAM_MAIN_CAMTG, "cam_m_camtg", "camtm_ck", 9),
	GATE_CAM_M0(CLK_CAM_MAIN_SENINF, "cam_m_seninf", "cam_ck", 10),
	GATE_CAM_M0(CLK_CAM_MAIN_CAMSV_TOP, "cam_m_camsv", "cam_ck", 11),
	GATE_CAM_M0(CLK_CAM_MAIN_CAM2MM0_GALS, "cam_m_cam2mm0_gals", "cam_ck", 16),
	GATE_CAM_M0(CLK_CAM_MAIN_CAM2MM1_GALS, "cam_m_cam2mm1_gals", "cam_ck", 17),
	GATE_CAM_M0(CLK_CAM_MAIN_CCUSYS, "cam_m_ccusys", "ccusys_ck", 20),
	GATE_CAM_M0(CLK_CAM_MAIN_CAM_ASG, "cam_m_cam_asg", "cam_ck", 27),
	/* CAM_M1 */
	GATE_CAM_M1(CLK_CAM_MAIN_CAMSV_A_CON_1, "cam_m_camsv_a_con_1", "cam_ck", 0),
	GATE_CAM_M1(CLK_CAM_MAIN_CAMSV_B_CON_1, "cam_m_camsv_b_con_1", "cam_ck", 1),
	GATE_CAM_M1(CLK_CAM_MAIN_CAMSV_C_CON_1, "cam_m_camsv_c_con_1", "cam_ck", 2),
	GATE_CAM_M1(CLK_CAM_MAIN_CAMSV_D_CON_1, "cam_m_camsv_d_con_1", "cam_ck", 3),
	GATE_CAM_M1(CLK_CAM_MAIN_CAMSV_E_CON_1, "cam_m_camsv_e_con_1", "cam_ck", 4),
	GATE_CAM_M1(CLK_CAM_MAIN_CAMSV_CON_1, "cam_m_camsv_con_1", "cam_ck", 5),
};

static const struct mtk_clk_desc cam_m_desc = {
	.clks = cam_m_clks,
	.num_clks = ARRAY_SIZE(cam_m_clks),
};

static const struct of_device_id of_match_clk_mt6878_cam[] = {
	{ .compatible = "mediatek,mt6878-camsys-main", .data = &cam_m_desc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_match_clk_mt6878_cam);

static struct platform_driver clk_mt6878_cam_drv = {
	.probe = mtk_clk_simple_probe,
	.remove = mtk_clk_simple_remove,
	.driver = {
		.name = "clk-mt6878-cam",
		.of_match_table = of_match_clk_mt6878_cam,
	},
};
module_platform_driver(clk_mt6878_cam_drv);

MODULE_DESCRIPTION("MediaTek MT6878 camera subsystem clocks driver");
MODULE_LICENSE("GPL");
