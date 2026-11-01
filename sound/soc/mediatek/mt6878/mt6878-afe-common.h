/* SPDX-License-Identifier: GPL-2.0 */
/*
 * MediaTek MT6878 audio front-end (AFE) common definitions
 *
 * Copyright (c) 2026 Jan Benedikt <karagos01@gmail.com>
 *
 * Shared types for the MT6878 AFE platform driver. The AFE is built on the
 * generic MediaTek AFE framework in sound/soc/mediatek/common/ (struct
 * mtk_base_afe, the memif/irq helpers and mtk_audio_sram) and belongs to the
 * MT8196 generation, so this header follows sound/soc/mediatek/mt8196/.
 *
 * Reverse engineering of the downstream vendor module (snd-soc-mt6878-afe.ko)
 * confirmed: 34 memif entries (memif_data[], 0xb0 bytes each) with the
 * downlink block at reg base 0x4444 (stride 0x30, ON bit 28) and the uplink
 * block from 0x51c4; 27 MCU interrupts (irq_data[], 0x40 bytes each); the
 * IPM2.0 rate encoding below (ADDA 48 kHz == 0xa); and a 32-bit MMIO regmap
 * with a flat cache. The memif/IRQ/DAI identifier lists here are inherited
 * from the MT8196 sibling and reconciled against the memif_data[]/irq_data[]
 * tables as the platform driver (mt6878-afe-pcm.c) is brought up.
 */

#ifndef _MT6878_AFE_COMMON_H_
#define _MT6878_AFE_COMMON_H_

#include <sound/pcm.h>
#include <sound/soc.h>

#include "mt6878-reg.h"
#include "../common/mtk-base-afe.h"

/* HW IPM 2.0 sampling-rate encoding (confirmed: ADDA 48 kHz uses 0xa). */
enum {
	MTK_AFE_IPM2P0_RATE_8K = 0x0,
	MTK_AFE_IPM2P0_RATE_11K = 0x1,
	MTK_AFE_IPM2P0_RATE_12K = 0x2,
	MTK_AFE_IPM2P0_RATE_16K = 0x4,
	MTK_AFE_IPM2P0_RATE_22K = 0x5,
	MTK_AFE_IPM2P0_RATE_24K = 0x6,
	MTK_AFE_IPM2P0_RATE_32K = 0x8,
	MTK_AFE_IPM2P0_RATE_44K = 0x9,
	MTK_AFE_IPM2P0_RATE_48K = 0xa,
	MTK_AFE_IPM2P0_RATE_88K = 0xd,
	MTK_AFE_IPM2P0_RATE_96K = 0xe,
	MTK_AFE_IPM2P0_RATE_176K = 0x11,
	MTK_AFE_IPM2P0_RATE_192K = 0x12,
	MTK_AFE_IPM2P0_RATE_352K = 0x15,
	MTK_AFE_IPM2P0_RATE_384K = 0x16,
};

enum {
	MTKAIF_PROTOCOL_1,
	MTKAIF_PROTOCOL_2,
	MTKAIF_PROTOCOL_2_CLK_P2,
};

/*
 * Memory-interface identifiers. Provisional: inherited from the MT8196
 * sibling and reconciled against the RE'd 34-entry memif_data[] table when
 * mt6878-afe-pcm.c lands. Confirmed so far: DL0..DL8 at reg base 0x4444 +
 * n*0x30 (ids 0..8); further downlink and the uplink/ETDM-in/HDMI memifs
 * follow. MT6878_MEMIF_NUM is the RE'd count (34).
 */
enum {
	MT6878_MEMIF_DL0,
	MT6878_MEMIF_DL1,
	MT6878_MEMIF_DL2,
	MT6878_MEMIF_DL3,
	MT6878_MEMIF_DL4,
	MT6878_MEMIF_DL5,
	MT6878_MEMIF_DL6,
	MT6878_MEMIF_DL7,
	MT6878_MEMIF_DL8,
	/* ... remaining DL / VUL / ETDM_IN / HDMI memifs, 34 total ... */
};

#define MT6878_MEMIF_NUM	34
#define MT6878_IRQ_NUM		27

/* MCU interrupt identifiers (irq_data[] index; IRQ0 CON=0x140, CNT=0x144). */
enum {
	MT6878_IRQ_0,
	/* ... IRQ_1 .. IRQ_26, 27 total ... */
};

/* Audio clock-gate identifiers (AUDIO_ENGEN_CON0 / AUDIO_TOP_CON4). */
enum {
	MT6878_AUDIO_26M_EN_ON,
	MT6878_AUDIO_APLL1_EN_ON,
	MT6878_AUDIO_APLL2_EN_ON,
	MT6878_CG_AUDIO_HOPPING_CK,
	MT6878_CG_AUDIO_F26M_CK,
	MT6878_CG_APLL1_CK,
	MT6878_CG_APLL2_CK,
	MT6878_PDN_APLL_TUNER1,
	MT6878_PDN_APLL_TUNER2,
	MT6878_AUDIO_CG_NUM,
};

struct clk;
struct mtk_base_afe;

/*
 * SoC-specific platform private data (mtk_base_afe::platform_priv).
 *
 * pmic_regmap is the MT6369 PMIC's SPMI regmap, which the AFE probe resolves
 * (of_find_node_by_name() -> of_find_device_by_node() -> dev_get_regmap())
 * so the ADDA path can reach the analog codec.
 */
struct mt6878_afe_private {
	struct clk **clk;
	int num_clks;

	/* per-DAI private data, indexed by DAI id */
	void *dai_priv[MT6878_MEMIF_NUM];

	/* regmap of the MT6369 PMIC that carries the analog codec */
	struct regmap *pmic_regmap;

	int mtkaif_protocol;
};

#endif /* _MT6878_AFE_COMMON_H_ */
