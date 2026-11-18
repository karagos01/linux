// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2026 Jan Benedikt <karagos01@gmail.com>
 *
 * MT6878 MMINFRA_CONFIG clock gates of the GCE.
 *
 * The SMI gate (CG0 bit 2) is deliberately not exposed: it has to be open
 * whenever the hardware voter switches MM_INFRA off, or the SoC resets. The
 * bootloader leaves it open and the voter restores it with the domain.
 */

#include <dt-bindings/clock/mediatek,mt6878-clock.h>
#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

#include "clk-gate.h"
#include "clk-mtk.h"

static const struct mtk_gate_regs mminfra0_cg_regs = {
	.set_ofs = 0x104,
	.clr_ofs = 0x108,
	.sta_ofs = 0x100,
};

static const struct mtk_gate_regs mminfra0_hwv_regs = {
	.set_ofs = 0x0000,
	.clr_ofs = 0x0004,
	.sta_ofs = 0x1c00,
};

static const struct mtk_gate_regs mminfra1_cg_regs = {
	.set_ofs = 0x114,
	.clr_ofs = 0x118,
	.sta_ofs = 0x110,
};

static const struct mtk_gate_regs mminfra1_hwv_regs = {
	.set_ofs = 0x0008,
	.clr_ofs = 0x000c,
	.sta_ofs = 0x1c04,
};

/*
 * The registers are only reachable while MM_INFRA is on, which every consumer
 * guarantees, but the unused clock sweep does not: keep it away from them.
 *
 * The GCE gates have to be switched by the hardware voter, or it resets the
 * SoC when it is told to switch MM_INFRA off.
 */
#define GATE_HWV_MMINFRA(_id, _name, _parent, _regs, _hwv_regs, _shift) {	\
		.id = _id,						\
		.name = _name,						\
		.parent_name = _parent,					\
		.regs = _regs,						\
		.hwv_regs = _hwv_regs,					\
		.shift = _shift,					\
		.flags = CLK_IGNORE_UNUSED,				\
		.ops = &mtk_clk_gate_hwv_ops_setclr,			\
	}

static const struct mtk_gate mminfra_config_clks[] = {
	/* The GCE needs its 26M clock whenever one of its bus clocks runs */
	GATE_HWV_MMINFRA(CLK_MMINFRA_GCE_26M, "mminfra_gce_26m", "mminfra_ck",
			 &mminfra1_cg_regs, &mminfra1_hwv_regs, 17),
	GATE_HWV_MMINFRA(CLK_MMINFRA_GCE_D, "mminfra_gce_d", "mminfra_gce_26m",
			 &mminfra0_cg_regs, &mminfra0_hwv_regs, 0),
	GATE_HWV_MMINFRA(CLK_MMINFRA_GCE_M, "mminfra_gce_m", "mminfra_gce_26m",
			 &mminfra0_cg_regs, &mminfra0_hwv_regs, 1),
};

static const struct mtk_clk_desc mminfra_config_desc = {
	.clks = mminfra_config_clks,
	.num_clks = ARRAY_SIZE(mminfra_config_clks),
};

static const struct of_device_id of_match_clk_mt6878_mminfra[] = {
	{ .compatible = "mediatek,mt6878-mminfra-config", .data = &mminfra_config_desc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_match_clk_mt6878_mminfra);

static struct platform_driver clk_mt6878_mminfra_drv = {
	.probe = mtk_clk_simple_probe,
	.remove = mtk_clk_simple_remove,
	.driver = {
		.name = "clk-mt6878-mminfra",
		.of_match_table = of_match_clk_mt6878_mminfra,
	},
};
module_platform_driver(clk_mt6878_mminfra_drv);

MODULE_DESCRIPTION("MediaTek MT6878 MMINFRA_CONFIG clock controller driver");
MODULE_LICENSE("GPL");
