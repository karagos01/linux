/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MediaTek MT6685 clock-buffer / RTC companion PMIC
 *
 * Copyright (c) 2026 Jan Benedikt <karagos01@gmail.com>
 */

#ifndef __MFD_MT6685_H
#define __MFD_MT6685_H

#include <linux/types.h>

/*
 * Audio 26 MHz DCXO buffer control. The MT6369 codec downlink needs this
 * buffer running to clock its sigma-delta DAC.
 */
void mt6685_set_dcxo(bool enable);
void mt6685_set_dcxo_mode(u8 mode);

#endif /* __MFD_MT6685_H */
