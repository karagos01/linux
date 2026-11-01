/* SPDX-License-Identifier: GPL-2.0 */
/*
 * MediaTek MT6878 AFE clock-control definitions
 *
 * Copyright (c) 2026 Jan Benedikt <karagos01@gmail.com>
 *
 * The clock identifiers and their names mirror the "clock-names" the AFE node
 * requests from device tree (reverse engineered from the downstream vendor
 * module's aud_clks[] table and confirmed against the stock device tree). The
 * aud_* gates come from the audiosys controller, the top_* muxes/dividers from
 * topckgen, and the *_ck_peri bus clocks from pericfg-ao.
 */

#ifndef _MT6878_AFE_CLK_H_
#define _MT6878_AFE_CLK_H_

struct mtk_base_afe;

enum {
	/* audiosys gates */
	MT6878_CLK_AUD_HOPPING = 0,
	MT6878_CLK_AUD_F26M,
	MT6878_CLK_AUD_DL0_DAC,
	MT6878_CLK_AUD_DL0_DAC_HIRES,
	MT6878_CLK_AUD_DL0_DAC_PREDIS,
	MT6878_CLK_AUD_UL0_ADC,
	MT6878_CLK_AUD_UL0_ADC_HIRES,
	MT6878_CLK_AUD_UL1_ADC,
	MT6878_CLK_AUD_UL1_ADC_HIRES,
	MT6878_CLK_AUD_APLL1,
	MT6878_CLK_AUD_APLL2,
	MT6878_CLK_AUD_APLL_TUNER1,
	MT6878_CLK_AUD_APLL_TUNER2,
	/* topckgen muxes and dividers */
	MT6878_CLK_TOP_MUX_AUDIO_INT,
	MT6878_CLK_TOP_MAINPLL_D4_D4,
	MT6878_CLK_TOP_MUX_AUD_1,
	MT6878_CLK_TOP_APLL1_CK,
	MT6878_CLK_TOP_MUX_AUD_2,
	MT6878_CLK_TOP_APLL2_CK,
	MT6878_CLK_TOP_MUX_AUD_ENG1,
	MT6878_CLK_TOP_APLL1_D4,
	MT6878_CLK_TOP_MUX_AUD_ENG2,
	MT6878_CLK_TOP_APLL2_D4,
	MT6878_CLK_TOP_MUX_AUDIO_H,
	MT6878_CLK_TOP_APLL1_D2,
	MT6878_CLK_TOP_APLL2_D2,
	MT6878_CLK_TOP_I2SIN1_M_SEL,
	MT6878_CLK_TOP_FMI2S_M_SEL,
	MT6878_CLK_TOP_APLL12_DIV_I2SIN1,
	MT6878_CLK_TOP_APLL12_DIV_FMI2S,
	MT6878_CLK_TOP_CLK26M,
	/* pericfg-ao always-on bus clocks */
	MT6878_CLK_AUD_SLV_CK_PERI,
	MT6878_CLK_AUD_MST_CK_PERI,
	MT6878_CLK_AUD_INTBUS_CK_PERI,
	MT6878_CLK_NUM,
};

/*
 * apmixedsys APLL control words written once at init. The values are the
 * stock settings recovered from mt6878_init_clock(); they program the audio
 * APLL1/APLL2 frequencies.
 */
#define MT6878_APLL1_CON_REG		0x40
#define MT6878_APLL1_CON_VAL		0x6f28bd4d
#define MT6878_APLL2_CON_REG		0x44
#define MT6878_APLL2_CON_VAL		0x78fd5265

int mt6878_init_clock(struct mtk_base_afe *afe);
int mt6878_afe_enable_clock(struct mtk_base_afe *afe);
void mt6878_afe_disable_clock(struct mtk_base_afe *afe);

#endif /* _MT6878_AFE_CLK_H_ */
