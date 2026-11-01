/* SPDX-License-Identifier: GPL-2.0 */
/*
 * MediaTek MT6878 audio front-end (AFE) register definitions
 *
 * Copyright (c) 2026 Jan Benedikt <karagos01@gmail.com>
 *
 * The MT6878 AFE is a standard MediaTek AFE built on the shared
 * sound/soc/mediatek/common/ framework (mtk-base-afe, memif/irq helpers).
 * Its register bank lives at 0x11050000 and belongs to the MT8196
 * generation: the memif, MTKAIF and interrupt blocks share that SoC's
 * offsets exactly (see sound/soc/mediatek/mt8196/). The ADDA block is laid
 * out differently on the MT6878 and its offsets come straight from the RE.
 *
 * The offsets below were recovered from the downstream vendor module
 * (snd-soc-mt6878-afe.ko): the ADDA constants come from the register writes
 * in mtk_dai_adda_hw_params()/mtk_adda_mtkaif_cfg_event(); the memif and
 * interrupt constants come from the static memif_data[]/irq_data[] tables.
 * This header currently covers the downlink (playback) path only; the
 * capture, I2S and connection-matrix groups are added as they are brought up.
 */

#ifndef _MT6878_REG_H_
#define _MT6878_REG_H_

/* ---- Top level / interrupt controller (AFE_IRQ_MCU) ---- */
#define AFE_IRQ_MCU_EN			0x0100
#define AFE_IRQ_MCU_STATUS		0x010c

/*
 * Per-interrupt control/counter registers. Each MCU IRQ line n occupies an
 * 8-byte slot: CON at 0x140 + n*8, CNT at 0x144 + n*8. The CON register holds
 * the enable bit (bit 0) and the sampling-rate field (bits [8:4], mask 0x1f);
 * the CNT register holds the 24-bit down-count (mask 0x00ffffff).
 */
#define AFE_IRQ_MCU_CON(n)		(0x0140 + (n) * 0x8)
#define AFE_IRQ_MCU_CNT(n)		(0x0144 + (n) * 0x8)
#define AFE_IRQ0_MCU_CON		AFE_IRQ_MCU_CON(0)
#define AFE_IRQ0_MCU_CNT		AFE_IRQ_MCU_CNT(0)

#define AFE_IRQ_CNT_SHIFT		0
#define AFE_IRQ_CNT_MASK		0x00ffffff
#define AFE_IRQ_FS_SHIFT		4
#define AFE_IRQ_FS_MASK			0x1f

/* ---- DRAM downlink memory interfaces (memif) ---- */
/*
 * Each DL memif occupies a 0x30-byte block. For memif n:
 *   BASE_MSB = 0x4440 + n*0x30   (upper address bits)
 *   BASE     = 0x4444 + n*0x30
 *   CUR_MSB  = 0x4448 + n*0x30
 *   CUR      = 0x444c + n*0x30
 *   END_MSB  = 0x4450 + n*0x30
 *   END      = 0x4454 + n*0x30
 *   CON0     = 0x4460 + n*0x30   (enable, mono, fs @ bits[12:8] mask 0x1f)
 */
#define AFE_DL_BLOCK_STRIDE		0x30
#define AFE_DL_BASE_MSB(n)		(0x4440 + (n) * AFE_DL_BLOCK_STRIDE)
#define AFE_DL_BASE(n)			(0x4444 + (n) * AFE_DL_BLOCK_STRIDE)
#define AFE_DL_CUR_MSB(n)		(0x4448 + (n) * AFE_DL_BLOCK_STRIDE)
#define AFE_DL_CUR(n)			(0x444c + (n) * AFE_DL_BLOCK_STRIDE)
#define AFE_DL_END_MSB(n)		(0x4450 + (n) * AFE_DL_BLOCK_STRIDE)
#define AFE_DL_END(n)			(0x4454 + (n) * AFE_DL_BLOCK_STRIDE)
#define AFE_DL_CON0(n)			(0x4460 + (n) * AFE_DL_BLOCK_STRIDE)

/* DL0 - the first playback memif (downstream memif name "DL0"). */
#define AFE_DL0_BASE_MSB		AFE_DL_BASE_MSB(0)
#define AFE_DL0_BASE			AFE_DL_BASE(0)
#define AFE_DL0_CUR_MSB			AFE_DL_CUR_MSB(0)
#define AFE_DL0_CUR			AFE_DL_CUR(0)
#define AFE_DL0_END_MSB			AFE_DL_END_MSB(0)
#define AFE_DL0_END			AFE_DL_END(0)
#define AFE_DL0_CON0			AFE_DL_CON0(0)

#define AFE_DL_CON0_ON_SHIFT		28
#define AFE_DL_CON0_MONO_SHIFT		4
#define AFE_DL_CON0_FS_SHIFT		8
#define AFE_DL_CON0_FS_MASK		0x1f

/* ---- ADDA (internal PMIC-codec link) downlink / uplink ---- */
#define AFE_ADDA_DL_SRC2_CON0		0x04d0	/* rate<<28 | 0xc1800 | enable(bit1) */
#define AFE_ADDA_DL_SRC2_CON1		0x04d4
#define AFE_ADDA_DL_SDM_DCCOMP_CON	0x04dc
#define AFE_ADDA_DL_SDM_TEST		0x04e0
#define AFE_ADDA_UL_DL_CON0		0x04ec	/* bits[5:0] clock, bit28 = AFE_ADDA on */
#define AFE_ADDA_DL_SDM_DITHER_CON	0x0510	/* RE: offset confirmed, name tentative */

#define AFE_ADDA_UL_DL_CON0_ADDA_AFE_ON	BIT(28)

/* ADDA DL_SRC2_CON0 48 kHz output-rate word (rate field already positioned). */
#define AFE_ADDA_DL_SRC2_48K		0x0a0c1800

/* ---- MTKAIF (serial link to the MT6369 PMIC codec) ---- */
/* Same offsets and naming as the MT8196-generation sibling. */
#define AFE_MTKAIF0_CFG0		0x1190
#define AFE_MTKAIF0_RX_CFG2		0x11a0
#define AFE_MTKAIF1_CFG0		0x11f0
#define AFE_MTKAIF1_RX_CFG2		0x1200

#endif /* _MT6878_REG_H_ */
