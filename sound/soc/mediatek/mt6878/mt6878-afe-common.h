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
 * confirmed: the IPM2.0 rate encoding below (ADDA 48 kHz == 0xa); the
 * downlink memif block at reg base 0x4444 (stride 0x30, ON bit 28); the MCU
 * interrupt block (CON 0x140 + n*8, CNT 0x144 + n*8); and a 32-bit MMIO
 * regmap. The hardware has 34 memif and 27 MCU interrupts in total; this
 * driver currently exposes the nine downlink interfaces (DL0..DL8) that make
 * up the playback path. The uplink/ETDM-in/HDMI memifs and their interrupts
 * are added as those capture paths are brought up.
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
 * Memory-interface identifiers. The downlink interfaces DL0..DL8 occupy a
 * regular 0x30-byte register block from base 0x4444 (confirmed by RE). The
 * remaining downlink, uplink, ETDM-in and HDMI memifs are added later.
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
	MT6878_MEMIF_NUM,
	/*
	 * Backend DAIs continue the id space after the memifs (the DAI id is
	 * used to index dai_priv[]). Only the ADDA downlink is wired up so far.
	 */
	MT6878_DAI_ADDA = MT6878_MEMIF_NUM,
	MT6878_DAI_NUM,
};

/* MCU interrupt identifiers (irq_data[] index; IRQn CON=0x140+n*8). */
enum {
	MT6878_IRQ_0,
	MT6878_IRQ_1,
	MT6878_IRQ_2,
	MT6878_IRQ_3,
	MT6878_IRQ_4,
	MT6878_IRQ_5,
	MT6878_IRQ_6,
	MT6878_IRQ_7,
	MT6878_IRQ_8,
	MT6878_IRQ_NUM,
};

struct clk;
struct mtk_base_afe;

/* per-ADDA-DAI private data (dai_priv[]) */
struct mtk_afe_adda_priv {
	int dl_rate;
	int ul_rate;
};

/*
 * SoC-specific platform private data (mtk_base_afe::platform_priv).
 *
 * pmic_regmap is the MT6369 PMIC's SPMI regmap, which the AFE resolves
 * (of_find_node_by_name() -> of_find_device_by_node() -> dev_get_regmap())
 * so the ADDA path can reach the analog codec.
 */
struct mt6878_afe_private {
	/* audio clocks, indexed by MT6878_CLK_* (see mt6878-afe-clk.h) */
	struct clk **clk;

	/* syscon regmaps used by the clock bring-up */
	struct regmap *topckgen;
	struct regmap *apmixedsys;
	struct regmap *infracfg;

	/* per-DAI private data, indexed by DAI id */
	void *dai_priv[MT6878_DAI_NUM];

	/* regmap of the MT6369 PMIC that carries the analog codec */
	struct regmap *pmic_regmap;

	int mtkaif_protocol;
};

int mt6878_dai_adda_register(struct mtk_base_afe *afe);

#endif /* _MT6878_AFE_COMMON_H_ */
