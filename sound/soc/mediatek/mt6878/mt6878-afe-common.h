/* SPDX-License-Identifier: GPL-2.0 */
/*
 * MediaTek MT6878 audio front-end (AFE) common definitions
 *
 * Copyright (c) 2026 Jan Benedikt <karagos01@gmail.com>
 *
 * Shared types for the MT6878 AFE platform driver. The AFE is built on the
 * generic MediaTek AFE framework in sound/soc/mediatek/common/ (struct
 * mtk_base_afe, the memif/irq helpers and mtk_audio_sram), so this header
 * only carries the SoC-specific counts, identifiers and private state.
 *
 * Counts and the memif/irq table shapes were recovered from the downstream
 * vendor module (snd-soc-mt6878-afe.ko) probe: 34 memif entries (0xb0 bytes
 * each) and 27 MCU interrupt entries (0x40 bytes each). The full memif/irq
 * identifier enums are populated as each DAI group is brought up; only the
 * downlink entries needed for first-light playback are defined so far.
 */

#ifndef _MT6878_AFE_COMMON_H_
#define _MT6878_AFE_COMMON_H_

#include <linux/list.h>
#include <linux/regmap.h>

struct clk;
struct mtk_base_afe;

#define MT6878_MEMIF_NUM	34
#define MT6878_IRQ_NUM		27

/*
 * Memory-interface identifiers. The downstream table is ordered with the
 * downlink memifs first; "DL0" is index 0. Remaining identifiers (uplink,
 * VoIP, etc.) are added together with their DAI drivers.
 */
enum {
	MT6878_MEMIF_DL0 = 0,
	/* ... further memif IDs added as they are brought up ... */
};

/* MCU interrupt identifiers (irq_data[] index). */
enum {
	MT6878_IRQ_0 = 0,
	/* ... further IRQ IDs added as they are brought up ... */
};

/*
 * SoC-specific platform private data (mtk_base_afe::platform_priv).
 *
 * The downstream driver links to the MT6369 PMIC audio codec at probe time by
 * resolving the PMIC node and calling dev_get_regmap() on its child; that
 * regmap is stashed here so the ADDA path can reach the codec. The clock
 * handles are filled by mt6878_init_clock() (brought up in a later step).
 */
struct mt6878_afe_private {
	struct clk **clk;
	int num_clks;

	/* regmap of the MT6369 PMIC that carries the analog codec. */
	struct regmap *pmic_regmap;

	/* ADDA runtime state (sample rates, MTKAIF calibration) added later. */
};

#endif /* _MT6878_AFE_COMMON_H_ */
