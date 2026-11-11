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

/* ---- Top level: audio sub-block clock gates and engine enable ---- */
/*
 * AUDIO_TOP_CON0..CON4 are the AFE's own clock-gate registers (a cleared bit
 * ungates the corresponding sub-block). AUDIO_ENGEN_CON0 bits 0 and 1 enable
 * the audio engine generators (26 MHz base + the second generator the downlink
 * engine needs); the stock driver sets both (reg reads 0x3 during playback),
 * while bits 2/3 are the APLL1/APLL2 engens (unused for the 26 MHz ADDA path).
 * These are driven from runtime-PM resume/suspend.
 */
#define AUDIO_TOP_CON0			0x0000
#define AUDIO_TOP_CON1			0x0004
#define AUDIO_TOP_CON2			0x0008
#define AUDIO_TOP_CON3			0x000c
#define AUDIO_TOP_CON4			0x0010
#define AUDIO_ENGEN_CON0		0x0014
#define AUDIO_ENGEN_CON0_ENGEN_ON	(BIT(0) | BIT(1))

/* ---- Top level / interrupt controller (AFE_IRQ_MCU) ---- */
#define AFE_IRQ_MCU_EN			0x0100
#define AFE_IRQ_MCU_STATUS		0x0120

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
/*
 * The DL CON0 low bits use the same layout as the VUL (uplink) CON0 and the
 * MT8196-generation sibling: HD_MODE in bits[1:0], a half-align bit at bit 2,
 * and NORMAL_MODE at bit 3. NORMAL_MODE comes up set at reset; programming the
 * HD mode at the wrong bit (3, as this header previously did) cleared it, so
 * the DL memif ran with NORMAL_MODE=0 and CON0 did not match the running stock
 * state (0x...a68 with bit 3 set). Point the HD field at bits[1:0] and add the
 * half-align bit so bit 3 is left alone.
 */
#define AFE_DL_CON0_HD_SHIFT		0	/* HD_MODE[1:0] */
#define AFE_DL_CON0_HALIGN_SHIFT	2

/* ---- DRAM uplink memory interfaces (VUL memif) ---- */
/*
 * The uplink (capture) memifs VUL0..VUL10 occupy a second 0x30-byte-strided
 * block starting at reg base 0x4d64 (confirmed by a stock capture ftrace:
 * arecord on "Capture_1" drives memif VUL9 at reg_ofs_base 0x4f14). The block
 * layout and the CON0 field positions are identical to the MT8196-generation
 * sibling (sound/soc/mediatek/mt8196/, AFE_VUL9_* == these offsets exactly):
 *   BASE_MSB = 0x4d60 + n*0x30
 *   BASE     = 0x4d64 + n*0x30
 *   CUR_MSB  = 0x4d68 + n*0x30
 *   CUR      = 0x4d6c + n*0x30
 *   END_MSB  = 0x4d70 + n*0x30
 *   END      = 0x4d74 + n*0x30
 *   CON0     = 0x4d80 + n*0x30   (ON bit28, fs[12:8], mono bit4, hd[1:0])
 * Only VUL9 (the primary ADDA capture interface) is wired up so far.
 */
#define AFE_VUL_BLOCK_STRIDE		0x30
#define AFE_VUL_BASE_MSB(n)		(0x4d60 + (n) * AFE_VUL_BLOCK_STRIDE)
#define AFE_VUL_BASE(n)			(0x4d64 + (n) * AFE_VUL_BLOCK_STRIDE)
#define AFE_VUL_CUR_MSB(n)		(0x4d68 + (n) * AFE_VUL_BLOCK_STRIDE)
#define AFE_VUL_CUR(n)			(0x4d6c + (n) * AFE_VUL_BLOCK_STRIDE)
#define AFE_VUL_END_MSB(n)		(0x4d70 + (n) * AFE_VUL_BLOCK_STRIDE)
#define AFE_VUL_END(n)			(0x4d74 + (n) * AFE_VUL_BLOCK_STRIDE)
#define AFE_VUL_CON0(n)			(0x4d80 + (n) * AFE_VUL_BLOCK_STRIDE)

/* VUL9 - the primary uplink memif (name "VUL9", reg_ofs_base 0x4f14). */
#define AFE_VUL9_BASE_MSB		AFE_VUL_BASE_MSB(9)
#define AFE_VUL9_BASE			AFE_VUL_BASE(9)
#define AFE_VUL9_CUR_MSB		AFE_VUL_CUR_MSB(9)
#define AFE_VUL9_CUR			AFE_VUL_CUR(9)
#define AFE_VUL9_END_MSB		AFE_VUL_END_MSB(9)
#define AFE_VUL9_END			AFE_VUL_END(9)
#define AFE_VUL9_CON0			AFE_VUL_CON0(9)

#define AFE_VUL_CON0_ON_SHIFT		28
#define AFE_VUL_CON0_FS_SHIFT		8
#define AFE_VUL_CON0_FS_MASK		0x1f
#define AFE_VUL_CON0_MONO_SHIFT		4
#define AFE_VUL_CON0_HD_SHIFT		0	/* HD_MODE[1:0] */
#define AFE_VUL_CON0_HALIGN_SHIFT	2

/* ---- Connection matrix (interconnect) ---- */
/*
 * Each output port O_xx has an AFE_CONN register whose bits select the input
 * ports I_xx feeding it. The downlink memif DL0 (ports I_DL0_CH1/CH2) is routed
 * into the ADDA downlink via the O_ADDA_DL_CH1/CH2 connection registers (RE from
 * the stock mtk_adda_dl_ch{1,2}_mix mixer controls).
 */
#define AFE_CONN_ADDA_DL_CH1		0x2244
#define AFE_CONN_ADDA_DL_CH2		0x2264
#define I_DL0_CH1			0
#define I_DL0_CH2			1

/*
 * Uplink interconnect: the VUL9 capture memif input ports O_VUL9_CH1/CH2 are
 * fed from the ADDA uplink input ports I_ADDA_UL_CH1/CH2 (bits 16/17). The
 * VUL9 memif reads from interconnect output ports 36/37, i.e. the CONN036/037
 * registers at 0x2500/0x2520 (matches the MT8196 generation: memif_ul9 uses
 * AFE_CONN036_0/AFE_CONN037_0). The stock capture ftrace also writes the
 * CONN020/021 pair at 0x2300/0x2320, but those feed a different memif; VUL9
 * itself only picks up data from 0x2500/0x2520.
 */
#define AFE_CONN_VUL9_CH1		0x2500
#define AFE_CONN_VUL9_CH2		0x2520
#define I_ADDA_UL_CH1			16
#define I_ADDA_UL_CH2			17

/* ---- ADDA (internal PMIC-codec link) downlink / uplink ---- */
#define AFE_ADDA_DL_SRC2_CON0		0x04d0	/* rate<<24 | 0xc1800 | enable(bit1) */
#define AFE_ADDA_DL_SRC2_CON1		0x04d4
#define AFE_ADDA_DL_SDM_DCCOMP_CON	0x04dc
#define AFE_ADDA_DL_SDM_TEST		0x04e0
#define AFE_ADDA_UL_DL_CON0		0x04ec	/* bits[5:0] clock, bit28 = AFE_ADDA on */
#define AFE_ADDA_DL_SDM_DITHER_CON	0x0510	/* RE: offset confirmed, name tentative */

#define AFE_ADDA_DL_SRC2_CON0_SRC_ON_SFT	1	/* DL SRC enable bit */
#define AFE_ADDA_UL_DL_CON0_ADDA_AFE_ON	BIT(28)
#define AFE_ADDA_UL_DL_CON0_ADDA_AFE_ON_SFT	28
/* The uplink shares AFE_ADDA_UL_DL_CON0: bit30 is the UL-side AFE-on, and
 * bits[14:12] (0x7) are the UL analog-interface config (RE from the stock
 * capture ftrace: 0x4ec = 0x4000701d = BIT(30) | 0x7000 | clk 0x1d).
 */
#define AFE_ADDA_UL_DL_CON0_ADDA_UL_AFE_ON	BIT(30)
#define AFE_ADDA_UL_DL_CON0_UL_CFG		0x7000

/* ADDA DL_SRC2_CON0 48 kHz output-rate word (rate field already positioned). */
#define AFE_ADDA_DL_SRC2_48K		0x0a0c1800

/* ---- ADDA uplink (capture) SRC ---- */
/*
 * The uplink anti-aliasing SRC. RE from the stock capture ftrace: the rate
 * word 0x60400 selects 48 kHz (bit0 is the SRC enable), and the fixed
 * coefficient/voice-mode words below are written verbatim before enable.
 */
#define AFE_ADDA_UL_SRC_CON0		0x0bd0	/* rate word | enable(bit0) */
#define AFE_ADDA_UL_SRC_CON1		0x0bd4	/* UL SRC gain (0x7fff = 0 dB) */
#define AFE_ADDA_UL_SRC2_CON1		0x0c54	/* parallel UL SRC gain */
#define AFE_ADDA_UL_SRC_CON1_VAL	0x7fff0412
#define AFE_ADDA_UL_SRC_VOICE_MODE_CFG1	0x0bec
#define AFE_ADDA_UL_SRC_VOICE_MODE_CFG2	0x0bf0
#define AFE_ADDA_UL_SRC_VOICE_MODE_CFG3	0x0bf4
#define AFE_ADDA_UL_SRC_VOICE_MODE_CFG4	0x0bf8
#define AFE_ADDA_UL_SRC_VOICE_MODE_CFG5	0x0bfc
#define AFE_ADDA_UL_SRC_CON0_SRC_ON_SFT	0	/* UL SRC enable bit */
#define AFE_ADDA_UL_SRC_48K		0x00060400

/* ---- MTKAIF RX pad-top control (uplink side) ---- */
#define AFE_ADDA_MTKAIF_RX_CFG0		0x1204

/* ---- MTKAIF (serial link to the MT6369 PMIC codec) ---- */
/* Same offsets and naming as the MT8196-generation sibling. */
#define AFE_MTKAIF0_CFG0		0x1190
#define AFE_MTKAIF0_RX_CFG2		0x11a0
#define AFE_MTKAIF1_CFG0		0x11f0
#define AFE_MTKAIF1_RX_CFG2		0x1200

/*
 * MTKAIF enable values for the uplink link, RE from the stock capture ftrace:
 * CFG0 0x10000 -> 0x80010000 (RX-enable bit31 set on top of the config).
 */
#define AFE_MTKAIF_CFG0_EN		0x80010000
#define AFE_MTKAIF_CFG0_CFG		0x00010000
#define AFE_ADDA_MTKAIF_RX_CFG0_UL	0x000000b9

/*
 * MTKAIF RX_CFG2 (one per MISO data line: MTKAIF0 = MISO0, MTKAIF1 = MISO1).
 * The base word holds the fixed sync-word config (SYNC_WORD0=3, SYNC_WORD1=6);
 * the per-line receive sampling delay is programmed on top of it from the
 * calibration result. Field layout matches the MT8196 generation:
 *   RXIF_DELAY_CYCLE = bits [15:12], RXIF_DELAY_DATA = bit 8.
 * The line that locks at the lower phase is delayed by the phase difference so
 * both MISO lines are sampled together (mirrors mt8195 mtkaif_rx_dcc_sync).
 */
#define AFE_MTKAIF_RX_CFG2_BASE		0x06300000
#define AFE_MTKAIF_RXIF_DELAY_DATA	BIT(8)
#define AFE_MTKAIF_RXIF_DELAY_CYCLE_SFT	12

/* ---- ETDM_OUT4 / I2S4 (external I2S to the speaker smart-amp) ---- */
/*
 * The ETDM_OUT4 block drives the SoC-side I2S port that feeds the external
 * speaker smart-amp. Offsets and the CON0/CON1/CON2/CON4 field layout follow
 * the MT8196-generation sibling (sound/soc/mediatek/mt8196/): ETDM_OUT4 is a
 * 0x20-byte block based at 0x1580, and all OUT instances share the OUT_REG_*
 * field positions below. These are confirmed against the stock
 * snd-soc-mt6878-afe.ko register dump (mt6878_debug_read_reg logs
 * ETDM_OUT4_CON0..CON8/MON) and verified empirically before enable.
 */
#define ETDM_OUT4_CON0			0x1580
#define ETDM_OUT4_CON1			0x1584
#define ETDM_OUT4_CON2			0x1588
#define ETDM_OUT4_CON4			0x1590
#define ETDM_OUT4_MON			0x15a8

/* ETDM_OUT4_CON0 fields */
#define ETDM_OUT_CON0_EN_SFT			0
#define ETDM_OUT_CON0_SYNC_MODE_SFT		1
#define ETDM_OUT_CON0_FMT_SFT			6	/* mask 0x7, 0 = I2S */
#define ETDM_OUT_CON0_FMT_MASK			0x7
#define ETDM_OUT_CON0_BIT_LENGTH_SFT		11	/* mask 0x1f, valid bits-1 */
#define ETDM_OUT_CON0_BIT_LENGTH_MASK		0x1f
#define ETDM_OUT_CON0_WORD_LENGTH_SFT		16	/* mask 0x1f, slot bits-1 */
#define ETDM_OUT_CON0_WORD_LENGTH_MASK		0x1f
#define ETDM_OUT_CON0_CH_NUM_SFT		23	/* mask 0x1f, channels-1 */
#define ETDM_OUT_CON0_CH_NUM_MASK		0x1f
#define ETDM_OUT_CON0_RELATCH_DOMAIN_SFT	28	/* mask 0x7, 1 = APLL */
#define ETDM_OUT_CON0_RELATCH_DOMAIN_MASK	0x7

/* ETDM_OUT4_CON1 fields */
#define ETDM_OUT_CON1_INITIAL_COUNT_SFT		0	/* mask 0x1f */
#define ETDM_OUT_CON1_INITIAL_COUNT_MASK	0x1f
#define ETDM_OUT_CON1_INITIAL_POINT_SFT		5	/* mask 0x1f */
#define ETDM_OUT_CON1_INITIAL_POINT_MASK	0x1f
#define ETDM_OUT_CON1_LRCK_RESET_SFT		15	/* mask 0x1 */

/* ETDM_OUT4_CON2 fields */
#define ETDM_OUT_CON2_IN2LATCH_TIME_SFT		0	/* mask 0x1f */
#define ETDM_OUT_CON2_IN2LATCH_TIME_MASK	0x1f

/* ETDM_OUT4_CON4 fields */
#define ETDM_OUT_CON4_FS_TIMING_SFT		0	/* mask 0x1f, rate code */
#define ETDM_OUT_CON4_FS_TIMING_MASK		0x1f
#define ETDM_OUT_CON4_CLOCK_SOURCE_SFT		6	/* mask 0x7, 1 = APLL */
#define ETDM_OUT_CON4_CLOCK_SOURCE_MASK		0x7
#define ETDM_OUT_CON4_RELATCH_EN_SFT		24	/* mask 0x1f, inconn code */
#define ETDM_OUT_CON4_RELATCH_EN_MASK		0x1f

/* ETDM 48 kHz constants (RE from mt8196 get_etdm_rate/get_etdm_inconn_rate) */
#define ETDM_RATE_48K			0x5	/* FS_TIMING_SEL @48k */
#define ETDM_CONN_48K			0xa	/* RELATCH_EN_SEL @48k */
#define ETDM_CLK_SOURCE_APLL		0x1
#define ETDM_RELATCH_SEL_APLL		0x1

/*
 * ETDM_4_7_COWORK_CON0 selects the data/sync/slave sources shared by ETDM
 * 4..7. For a stand-alone ETDM_OUT4 master only SLAVE_SEL is written (= 0,
 * ETDMIN4 is its own master).
 */
#define ETDM_4_7_COWORK_CON0		0x1690
#define ETDM_OUT4_SLAVE_SEL_SFT		8
#define ETDM_OUT4_SLAVE_SEL_MASK	0xf

/*
 * I2S4 pad-top control (shared VLP register): enable the pad clock and park
 * the output BCK latch edge. ETDM_OUT4 is the only OUT that uses the pad-top.
 */
#define AUD_TOP_CFG_VLP_RG		0x0098
#define RG_I2S4_PAD_TOP_CK_EN_SFT		5
#define RG_I2S4_OUT_BCK_NEG_EG_LATCH_SFT	20

/*
 * ETDM sub-block clock gates live in AUDIO_TOP_CON2 (PDN bits: 1 = gated).
 * ETDM_OUT4 shares the I2S4 pad with ETDM_IN4, so both gates are ungated.
 */
#define AUDIO_TOP_CON2_PDN_ETDM_IN4	BIT(9)
#define AUDIO_TOP_CON2_PDN_ETDM_OUT4	BIT(17)

/*
 * Connection matrix: route a downlink memif into the ETDM_OUT4 output ports.
 * The I2SOUT4 channels are driven by interconnect output ports 116 (CH1) and
 * 117 (CH2); the DL memifs (global input ports 32..63) land in the "_1"
 * sub-register with the 32-port offset already folded into the bit position
 * (DLn_CH1 = bit 2n, DLn_CH2 = bit 2n+1). The speaker FE uses DL1.
 */
#define AFE_CONN_I2SOUT4_CH1		0x2f04	/* AFE_CONN116_1 */
#define AFE_CONN_I2SOUT4_CH2		0x2f24	/* AFE_CONN117_1 */
#define I_DL1_CH1			2
#define I_DL1_CH2			3

/*
 * Modem PCM interface 1 (the MD1 speech link), RE from the stock
 * mtk_dai_pcm_hw_params() and mtk_dai_pcm_widgets: CON1 bit 0 enables the
 * interface, the rest is the format; CON2[25:18] carries the sample rate.
 */
#define PCM_INTF_CON1			0x00d0
#define PCM_INTF_CON1_EN_SFT		0
#define PCM_INTF_CON1_FMT		0x46	/* PCM mode B, AFIFO */
#define PCM_INTF_CON1_MODE_SFT		3	/* 0 8k, 1 16k, 2 32k, 3 48k */
#define PCM_INTF_CON2			0x00d4
#define PCM_INTF_CON2_RATE_MASK		GENMASK(25, 18)
#define PCM_INTF_CON2_RATE_SFT		18

/*
 * Speech interconnect (stock mtk_adda_dl_ch{1,2}_mix and
 * mtk_pcm_1_playback_ch{1,2}_mix): the PCM 1 capture port (modem downlink)
 * into the ADDA downlink, and the ADDA uplink into the PCM 1 playback ports
 * (modem uplink).
 */
#define AFE_CONN_ADDA_DL_CH1_PCM	0x2250
#define AFE_CONN_ADDA_DL_CH2_PCM	0x2270
#define I_PCM_1_CAP_CH1			4
#define AFE_CONN_PCM_1_PB_CH1		0x2d40
#define AFE_CONN_PCM_1_PB_CH2		0x2d60

#endif /* _MT6878_REG_H_ */
