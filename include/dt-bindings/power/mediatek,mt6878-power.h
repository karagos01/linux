/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */
/*
 * Power domains of the MediaTek MT6878 System Power Manager
 */

#ifndef _DT_BINDINGS_POWER_MT6878_POWER_H
#define _DT_BINDINGS_POWER_MT6878_POWER_H

#define MT6878_POWER_DOMAIN_MD		0
#define MT6878_POWER_DOMAIN_CONN	1
#define MT6878_POWER_DOMAIN_AUDIO	2
#define MT6878_POWER_DOMAIN_SSUSB	5
#define MT6878_POWER_DOMAIN_DISP	8
#define MT6878_POWER_DOMAIN_MM_PROC	20

/* Domains switched through the hardware voter */
#define MT6878_HWV_POWER_DOMAIN_MM_INFRA	0

#endif /* _DT_BINDINGS_POWER_MT6878_POWER_H */
