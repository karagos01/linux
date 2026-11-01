// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6878 audiosys (AFE) clock controller
 *
 * Copyright (c) 2026 Jan Benedikt <karagos01@gmail.com>
 *
 * The AFE audio gate clocks live in the AUDIO_TOP_CON0..CON4 registers of the
 * audio front-end register bank (0x11050000). The gate register/bit layout and
 * the parent assignments were reverse engineered from the downstream vendor
 * clock controller (clk-mt6878-adsp.ko, which matches "mediatek,mt6878-audiosys"
 * and registers the afe_clks[] gate table); the parent names are mapped onto
 * the mainline topckgen factor clocks.
 */

#include <dt-bindings/clock/mediatek,mt6878-clock.h>
#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

#include "clk-gate.h"
#include "clk-mtk.h"

/*
 * The audio gates are plain enable bits inside the AUDIO_TOP_CONx registers:
 * there are no separate set/clear registers, so all three offsets point at the
 * same CONx register and the no-setclr gate ops do a read-modify-write.
 */
static const struct mtk_gate_regs afe0_cg_regs = {
	.set_ofs = 0x0,
	.clr_ofs = 0x0,
	.sta_ofs = 0x0,
};

static const struct mtk_gate_regs afe1_cg_regs = {
	.set_ofs = 0x4,
	.clr_ofs = 0x4,
	.sta_ofs = 0x4,
};

static const struct mtk_gate_regs afe2_cg_regs = {
	.set_ofs = 0x8,
	.clr_ofs = 0x8,
	.sta_ofs = 0x8,
};

static const struct mtk_gate_regs afe3_cg_regs = {
	.set_ofs = 0xc,
	.clr_ofs = 0xc,
	.sta_ofs = 0xc,
};

static const struct mtk_gate_regs afe4_cg_regs = {
	.set_ofs = 0x10,
	.clr_ofs = 0x10,
	.sta_ofs = 0x10,
};

#define GATE_AFE0(_id, _name, _parent, _shift)			\
	GATE_MTK(_id, _name, _parent, &afe0_cg_regs, _shift,	\
		 &mtk_clk_gate_ops_no_setclr)
#define GATE_AFE1(_id, _name, _parent, _shift)			\
	GATE_MTK(_id, _name, _parent, &afe1_cg_regs, _shift,	\
		 &mtk_clk_gate_ops_no_setclr)
#define GATE_AFE2(_id, _name, _parent, _shift)			\
	GATE_MTK(_id, _name, _parent, &afe2_cg_regs, _shift,	\
		 &mtk_clk_gate_ops_no_setclr)
#define GATE_AFE3(_id, _name, _parent, _shift)			\
	GATE_MTK(_id, _name, _parent, &afe3_cg_regs, _shift,	\
		 &mtk_clk_gate_ops_no_setclr)
#define GATE_AFE4(_id, _name, _parent, _shift)			\
	GATE_MTK(_id, _name, _parent, &afe4_cg_regs, _shift,	\
		 &mtk_clk_gate_ops_no_setclr)

static const struct mtk_gate afe_clks[] = {
	/* AUDIO_TOP_CON0 */
	GATE_AFE0(CLK_AFE_DL0_DAC_TML, "afe_dl0_dac_tml", "f26m_ck", 7),
	GATE_AFE0(CLK_AFE_DL0_DAC_HIRES, "afe_dl0_dac_hires", "audio_h_ck", 8),
	GATE_AFE0(CLK_AFE_DL0_DAC, "afe_dl0_dac", "f26m_ck", 9),
	GATE_AFE0(CLK_AFE_DL0_PREDIS, "afe_dl0_predis", "f26m_ck", 10),
	GATE_AFE0(CLK_AFE_DL0_NLE, "afe_dl0_nle", "f26m_ck", 11),
	GATE_AFE0(CLK_AFE_PCM1, "afe_pcm1", "f26m_ck", 13),
	GATE_AFE0(CLK_AFE_PCM0, "afe_pcm0", "f26m_ck", 14),
	GATE_AFE0(CLK_AFE_CM1, "afe_cm1", "f26m_ck", 17),
	GATE_AFE0(CLK_AFE_CM0, "afe_cm0", "f26m_ck", 18),
	GATE_AFE0(CLK_AFE_STF, "afe_stf", "f26m_ck", 19),
	GATE_AFE0(CLK_AFE_HW_GAIN23, "afe_hw_gain23", "f26m_ck", 20),
	GATE_AFE0(CLK_AFE_HW_GAIN01, "afe_hw_gain01", "f26m_ck", 21),
	GATE_AFE0(CLK_AFE_FM_I2S, "afe_fm_i2s", "f26m_ck", 24),
	GATE_AFE0(CLK_AFE_MTKAIFV4, "afe_mtkaifv4", "f26m_ck", 25),
	/* AUDIO_TOP_CON1 */
	GATE_AFE1(CLK_AFE_UL1_ADC_HIRES_TML, "afe_ul1_aht", "audio_h_ck", 16),
	GATE_AFE1(CLK_AFE_UL1_ADC_HIRES, "afe_ul1_adc_hires", "audio_h_ck", 17),
	GATE_AFE1(CLK_AFE_UL1_TML, "afe_ul1_tml", "f26m_ck", 18),
	GATE_AFE1(CLK_AFE_UL1_ADC, "afe_ul1_adc", "f26m_ck", 19),
	GATE_AFE1(CLK_AFE_UL0_ADC_HIRES_TML, "afe_ul0_aht", "audio_h_ck", 20),
	GATE_AFE1(CLK_AFE_UL0_ADC_HIRES, "afe_ul0_adc_hires", "audio_h_ck", 21),
	GATE_AFE1(CLK_AFE_UL0_TML, "afe_ul0_tml", "f26m_ck", 22),
	GATE_AFE1(CLK_AFE_UL0_ADC, "afe_ul0_adc", "f26m_ck", 23),
	/* AUDIO_TOP_CON2 */
	GATE_AFE2(CLK_AFE_ETDM_IN4, "afe_etdm_in4", "aud_engen1_ck", 9),
	GATE_AFE2(CLK_AFE_ETDM_IN2, "afe_etdm_in2", "aud_engen1_ck", 11),
	GATE_AFE2(CLK_AFE_ETDM_IN1, "afe_etdm_in1", "aud_engen1_ck", 12),
	GATE_AFE2(CLK_AFE_ETDM_OUT4, "afe_etdm_out4", "aud_engen1_ck", 17),
	GATE_AFE2(CLK_AFE_ETDM_OUT2, "afe_etdm_out2", "aud_engen1_ck", 19),
	GATE_AFE2(CLK_AFE_ETDM_OUT1, "afe_etdm_out1", "aud_engen1_ck", 20),
	/* AUDIO_TOP_CON3 */
	GATE_AFE3(CLK_AFE_GENERAL2_ASRC, "afe_general2_asrc", "audio_h_ck", 22),
	GATE_AFE3(CLK_AFE_GENERAL1_ASRC, "afe_general1_asrc", "audio_h_ck", 23),
	GATE_AFE3(CLK_AFE_GENERAL0_ASRC, "afe_general0_asrc", "audio_h_ck", 24),
	GATE_AFE3(CLK_AFE_CONNSYS_I2S_ASRC, "afe_connsys_i2s_asrc", "audio_h_ck", 25),
	/* AUDIO_TOP_CON4 */
	GATE_AFE4(CLK_AFE_AUDIO_HOPPING, "afe_audio_hopping_ck", "f26m_ck", 0),
	GATE_AFE4(CLK_AFE_AUDIO_F26M, "afe_audio_f26m_ck", "f26m_ck", 1),
	GATE_AFE4(CLK_AFE_APLL1, "afe_apll1_ck", "aud_1_ck", 2),
	GATE_AFE4(CLK_AFE_APLL2, "afe_apll2_ck", "aud_2_ck", 3),
	GATE_AFE4(CLK_AFE_H208M, "afe_h208m_ck", "audio_h_ck", 4),
	GATE_AFE4(CLK_AFE_APLL_TUNER2, "afe_apll_tuner2", "aud_engen2_ck", 12),
	GATE_AFE4(CLK_AFE_APLL_TUNER1, "afe_apll_tuner1", "aud_engen1_ck", 13),
};

static const struct mtk_clk_desc afe_mcd = {
	.clks = afe_clks,
	.num_clks = ARRAY_SIZE(afe_clks),
};

static const struct of_device_id of_match_clk_mt6878_audiosys[] = {
	{ .compatible = "mediatek,mt6878-audiosys", .data = &afe_mcd },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_match_clk_mt6878_audiosys);

static struct platform_driver clk_mt6878_audiosys_drv = {
	.probe = mtk_clk_simple_probe,
	.remove = mtk_clk_simple_remove,
	.driver = {
		.name = "clk-mt6878-audiosys",
		.of_match_table = of_match_clk_mt6878_audiosys,
	},
};

MODULE_DESCRIPTION("MediaTek MT6878 audiosys clock controller driver");
module_platform_driver(clk_mt6878_audiosys_drv);
MODULE_LICENSE("GPL");
