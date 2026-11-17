/* SPDX-License-Identifier: GPL-2.0 */
/*
 * MediaTek low power manager: system resource requests towards the SPM.
 */
#ifndef __SOC_MEDIATEK_MTK_LPM_H
#define __SOC_MEDIATEK_MTK_LPM_H

#include <linux/kconfig.h>

#if IS_REACHABLE(CONFIG_MTK_LPM)
/*
 * Keep the memory path (DRAM, EMI and its clock source) available while all
 * CPUs are idle. Needed by anything that masters the bus on its own, such as
 * a USB link or a running audio stream. Calls nest and may come from atomic
 * context.
 */
void mtk_lpm_bus_hold(void);
void mtk_lpm_bus_release(void);
#else
static inline void mtk_lpm_bus_hold(void) { }
static inline void mtk_lpm_bus_release(void) { }
#endif

#endif
