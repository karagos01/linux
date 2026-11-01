/* SPDX-License-Identifier: GPL-2.0 */
/*
 * MediaTek MT6369 ALSA SoC audio codec driver
 *
 * Copyright (c) 2026 Jan Benedikt <karagos01@gmail.com>
 *
 * The MT6369 is the audio codec function block of the MT6369 SPMI sub-PMIC.
 * It shares the chip's SPMI register map with the other function blocks (see
 * drivers/mfd/mtk-spmi-pmic.c), which this driver obtains from its parent.
 *
 * The register addresses here were recovered from the MediaTek downstream
 * driver; unlike the pwrap-based MT6359 the MT6369 is addressed over SPMI with
 * one byte per register, so the analog audio block is a run of single-byte
 * MT6369_AUDDEC_ANA_CONn registers starting at 0x2588.
 */

#ifndef __MT6369_H__
#define __MT6369_H__

/* Top level / identification */
#define MT6369_HWCID0			0x0008

/* GPIO / analog pad drive strength (set up once at init) */
#define MT6369_SMT_CON1			0x0023
#define MT6369_DRV_CON1			0x0024
#define MT6369_DRV_CON3			0x002a
#define MT6369_DRV_CON4			0x002b
#define MT6369_DRV_CON5			0x002c
#define MT6369_DRV_CON6			0x002d
#define MT6369_GPIO_DIR0		0x008b
#define MT6369_GPIO_MODE2		0x00cc
#define MT6369_GPIO_MODE3		0x00d2

/* Audio top clock gating (RE: bits 2 and 5 cleared at init) */
#define MT6369_AUD_TOP_CKPDN_CON0	0x23d7

/* Audio microphone bias / encoder digital config (init only for now) */
#define MT6369_AUDENC_ANA_CON16		0x2508
#define MT6369_AUDENC_ANA_CON18		0x250a
#define MT6369_AUDENC_ANA_CON27		0x2514
#define MT6369_AUDENC_ANA_CON38		0x251f

/*
 * Negative charge pump / clock divider for the class-AB output amplifiers.
 * Single-byte registers at 0x248d..0x2491.
 */
#define MT6369_AUDNCP_CLKDIV_CON0	0x248d
#define MT6369_AUDNCP_CLKDIV_CON1	0x248e
#define MT6369_AUDNCP_CLKDIV_CON2	0x248f
#define MT6369_AUDNCP_CLKDIV_CON3	0x2490
#define MT6369_AUDNCP_CLKDIV_CON4	0x2491

/*
 * Analog audio downlink (DAC + HP/receiver/lineout output stage).
 * One byte per register from 0x2588; CONn == 0x2588 + n.
 */
#define MT6369_AUDDEC_ANA_CON(n)	(0x2588 + (n))
#define MT6369_AUDDEC_ANA_CON0		MT6369_AUDDEC_ANA_CON(0)	/* 0x2588 output enable */
#define MT6369_AUDDEC_ANA_CON1		MT6369_AUDDEC_ANA_CON(1)	/* 0x2589 HP aux/short */
#define MT6369_AUDDEC_ANA_CON2		MT6369_AUDDEC_ANA_CON(2)	/* 0x258a HP depop */
#define MT6369_AUDDEC_ANA_CON3		MT6369_AUDDEC_ANA_CON(3)	/* 0x258b HP gain ramp */
#define MT6369_AUDDEC_ANA_CON4		MT6369_AUDDEC_ANA_CON(4)	/* 0x258c */
#define MT6369_AUDDEC_ANA_CON5		MT6369_AUDDEC_ANA_CON(5)	/* 0x258d AUDDEC enable */
#define MT6369_AUDDEC_ANA_CON8		MT6369_AUDDEC_ANA_CON(8)	/* 0x2590 */
#define MT6369_AUDDEC_ANA_CON10		MT6369_AUDDEC_ANA_CON(10)	/* 0x2592 receiver/DACL */
#define MT6369_AUDDEC_ANA_CON12		MT6369_AUDDEC_ANA_CON(12)	/* 0x2594 HP DAC */
#define MT6369_AUDDEC_ANA_CON13		MT6369_AUDDEC_ANA_CON(13)	/* 0x2595 */
#define MT6369_AUDDEC_ANA_CON14		MT6369_AUDDEC_ANA_CON(14)	/* 0x2596 DAC bias */
#define MT6369_AUDDEC_ANA_CON15		MT6369_AUDDEC_ANA_CON(15)	/* 0x2597 */
#define MT6369_AUDDEC_ANA_CON16		MT6369_AUDDEC_ANA_CON(16)	/* 0x2598 output switch */
#define MT6369_AUDDEC_ANA_CON17		MT6369_AUDDEC_ANA_CON(17)	/* 0x2599 HP depop/gain */
#define MT6369_AUDDEC_ANA_CON18		MT6369_AUDDEC_ANA_CON(18)	/* 0x259a */
#define MT6369_AUDDEC_ANA_CON19		MT6369_AUDDEC_ANA_CON(19)	/* 0x259b */
#define MT6369_AUDDEC_ANA_CON20		MT6369_AUDDEC_ANA_CON(20)	/* 0x259c ZCD */
#define MT6369_AUDDEC_ANA_CON21		MT6369_AUDDEC_ANA_CON(21)	/* 0x259d bias */
#define MT6369_AUDDEC_ANA_CON22		MT6369_AUDDEC_ANA_CON(22)	/* 0x259e bias */
#define MT6369_AUDDEC_ANA_CON23		MT6369_AUDDEC_ANA_CON(23)	/* 0x259f bias trim */
#define MT6369_AUDDEC_ANA_CON24		MT6369_AUDDEC_ANA_CON(24)	/* 0x25a0 */
#define MT6369_AUDDEC_ANA_CON25		MT6369_AUDDEC_ANA_CON(25)	/* 0x25a1 LDO */
#define MT6369_AUDDEC_ANA_CON26		MT6369_AUDDEC_ANA_CON(26)	/* 0x25a2 LDO */
#define MT6369_AUDDEC_ANA_CON27		MT6369_AUDDEC_ANA_CON(27)	/* 0x25a3 LDO */
#define MT6369_AUDDEC_ANA_CON128	MT6369_AUDDEC_ANA_CON(128)	/* 0x2608 ZCD */
#define MT6369_AUDDEC_ANA_CON129	MT6369_AUDDEC_ANA_CON(129)	/* 0x2609 HP volume L */
#define MT6369_AUDDEC_ANA_CON130	MT6369_AUDDEC_ANA_CON(130)	/* 0x260a HP volume R */
#define MT6369_AUDDEC_ANA_CON131	MT6369_AUDDEC_ANA_CON(131)	/* 0x260b HPL gain */
#define MT6369_AUDDEC_ANA_CON132	MT6369_AUDDEC_ANA_CON(132)	/* 0x260c HPR gain */
#define MT6369_AUDDEC_ANA_CON133	MT6369_AUDDEC_ANA_CON(133)	/* 0x260d receiver gain */

/* Low-power idle value for the HP/receiver gain registers. */
#define MT6369_HPGAIN_IDLE		0x1f

/* HP DC-offset trim calibration value lives in the PMIC efuse. */
#define MT6369_HP_EFUSE_OFFSET		0x9e

#endif /* __MT6369_H__ */
