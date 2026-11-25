// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the GalaxyCore GC32E1 image sensor
 *
 * The register sequences were taken from the vendor imgsensor driver of
 * the Volla Phone Plinius; only the 2x2 binned 3264x2448 mode over four
 * MIPI CSI-2 lanes is supported.
 *
 * Copyright (C) 2026 Jan Benedikt <karagos01@gmail.com>
 */
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/units.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define GC32E1_REG_TEST_PATTERN_EN	CCI_REG8(0x008c)
#define GC32E1_REG_TEST_PATTERN_IDX	CCI_REG8(0x008d)

#define GC32E1_REG_STREAMING		CCI_REG8(0x0100)

#define GC32E1_REG_EXPOSURE		CCI_REG16(0x0202)
#define GC32E1_EXPOSURE_MIN		4
#define GC32E1_EXPOSURE_MARGIN		16

#define GC32E1_REG_AGAIN		CCI_REG16(0x0204)
#define GC32E1_AGAIN_MIN		1024
#define GC32E1_AGAIN_MAX		(1024 * 32)

#define GC32E1_REG_FRAME_LENGTH		CCI_REG16(0x0340)
#define GC32E1_VTS_MAX			0xfffe

/* register reads do not auto-increment, so the ID is read bytewise */
#define GC32E1_REG_CHIP_ID_H		CCI_REG8(0x03f0)
#define GC32E1_REG_CHIP_ID_L		CCI_REG8(0x03f1)
#define GC32E1_CHIP_ID			0x32e1

#define GC32E1_XCLK_FREQ		(24 * HZ_PER_MHZ)
#define GC32E1_MBUS_CODE		MEDIA_BUS_FMT_SRGGB10_1X10
#define GC32E1_BPP			10
#define GC32E1_DATA_LANES		4
#define GC32E1_LINK_FREQ		(381600 * HZ_PER_KHZ)
/* link_freq * 2 * lanes / bpp */
#define GC32E1_PIXEL_RATE		305280000LL

#define GC32E1_WIDTH			3264
#define GC32E1_HEIGHT			2448
/*
 * The sensor runs 4276 clocks of 345.6 MHz per line; expressed in output
 * pixel periods that is 3777 for the V4L2 blanking controls.
 */
#define GC32E1_HTS			3777
#define GC32E1_VTS_DEF			2656

static const char *const gc32e1_supply_names[] = {
	"dovdd",
	"dvdd",
	"avdd",
};

static const s64 gc32e1_link_freqs[] = {
	GC32E1_LINK_FREQ,
};

static const char *const gc32e1_test_pattern_menu[] = {
	"Disabled",
	"Solid Black",
	"Colour Bars",
};

static const struct cci_reg_sequence gc32e1_init_regs[] = {
	{ CCI_REG8(0x0315), 0xc0 },
	{ CCI_REG8(0x0c10), 0x1b },
	{ CCI_REG8(0x01a7), 0x02 },
	{ CCI_REG8(0x01aa), 0x05 },
	{ CCI_REG8(0x01a8), 0x02 },
	{ CCI_REG8(0x0c0d), 0xb4 },
	{ CCI_REG8(0x0185), 0xc0 },
	{ CCI_REG8(0x0314), 0x11 },
	{ CCI_REG8(0x031a), 0x00 },
	{ CCI_REG8(0x01a1), 0x10 },
	{ CCI_REG8(0x01e3), 0x03 },
	{ CCI_REG8(0x0057), 0x03 },
	{ CCI_REG8(0x00a6), 0x06 },
	{ CCI_REG8(0x00d3), 0x30 },
	{ CCI_REG8(0x0087), 0x51 },
	{ CCI_REG8(0x01e2), 0x24 },
	{ CCI_REG8(0x01ea), 0x24 },
	{ CCI_REG8(0x0219), 0x05 },
	{ CCI_REG8(0x0c08), 0x19 },
	{ CCI_REG8(0x0c05), 0xff },
	{ CCI_REG8(0x0c07), 0x14 },
	{ CCI_REG8(0x0c41), 0x0a },
	{ CCI_REG8(0x0c45), 0xdf },
	{ CCI_REG8(0x0e15), 0x58 },
	{ CCI_REG8(0x0e6c), 0x01 },
	{ CCI_REG8(0x0e6d), 0x01 },
	{ CCI_REG8(0x03a2), 0x00 },
	{ CCI_REG8(0x0313), 0x00 },
	{ CCI_REG8(0x0a53), 0x04 },
	{ CCI_REG8(0x0a65), 0x17 },
	{ CCI_REG8(0x0a68), 0x33 },
	{ CCI_REG8(0x0a58), 0x00 },
	{ CCI_REG8(0x0a4f), 0x00 },
	{ CCI_REG8(0x0a66), 0x00 },
	{ CCI_REG8(0x0a7f), 0x07 },
	{ CCI_REG8(0x0a84), 0x0c },
};

static const struct cci_reg_sequence gc32e1_mode_3264x2448_regs[] = {
	{ CCI_REG8(0x031c), 0x60 },
	{ CCI_REG8(0x0315), 0xd3 },
	{ CCI_REG8(0x01a9), 0x35 },
	{ CCI_REG8(0x0c0c), 0x48 },
	{ CCI_REG8(0x0c0e), 0x21 },
	{ CCI_REG8(0x0311), 0xb0 },
	{ CCI_REG8(0x05a0), 0x0a },
	{ CCI_REG8(0x0059), 0x11 },
	{ CCI_REG8(0x0084), 0x30 },
	{ CCI_REG8(0x0101), 0x00 },
	{ CCI_REG8(0x0202), 0x0a },
	{ CCI_REG8(0x0203), 0x42 },
	{ CCI_REG8(0x0210), 0xa3 },
	{ CCI_REG8(0x0218), 0x03 },
	{ CCI_REG8(0x0226), 0x14 },
	{ CCI_REG8(0x0227), 0x80 },
	{ CCI_REG8(0x0340), 0x0a },
	{ CCI_REG8(0x0341), 0x60 },
	{ CCI_REG8(0x0342), 0x08 },
	{ CCI_REG8(0x0343), 0x5a },
	{ CCI_REG8(0x0346), 0x00 },
	{ CCI_REG8(0x0347), 0x20 },
	{ CCI_REG8(0x034a), 0x13 },
	{ CCI_REG8(0x034b), 0x40 },
	{ CCI_REG8(0x0c25), 0x14 },
	{ CCI_REG8(0x0c55), 0x14 },
	{ CCI_REG8(0x013e), 0x34 },
	{ CCI_REG8(0x025b), 0x00 },
	{ CCI_REG8(0x025c), 0x40 },
	{ CCI_REG8(0x02c0), 0x55 },
	{ CCI_REG8(0x02c1), 0x71 },
	{ CCI_REG8(0x02c2), 0x07 },
	{ CCI_REG8(0x02c3), 0x1d },
	{ CCI_REG8(0x0c46), 0xfd },
	{ CCI_REG8(0x0c47), 0x7f },
	{ CCI_REG8(0x0c48), 0x5b },
	{ CCI_REG8(0x0e28), 0x5a },
	{ CCI_REG8(0x0e33), 0x00 },
	{ CCI_REG8(0x0e34), 0x04 },
	{ CCI_REG8(0x0e47), 0x02 },
	{ CCI_REG8(0x0e61), 0x20 },
	{ CCI_REG8(0x0e62), 0x20 },
	{ CCI_REG8(0x0e65), 0x01 },
	{ CCI_REG8(0x0e66), 0x20 },
	{ CCI_REG8(0x0e67), 0x2f },
	{ CCI_REG8(0x0e68), 0x2f },
	{ CCI_REG8(0x0e6a), 0x54 },
	{ CCI_REG8(0x0e6b), 0x06 },
	{ CCI_REG8(0x0e6e), 0x42 },
	{ CCI_REG8(0x0e6f), 0x44 },
	{ CCI_REG8(0x0e70), 0x2a },
	{ CCI_REG8(0x0e71), 0x2c },
	{ CCI_REG8(0x0e74), 0x45 },
	{ CCI_REG8(0x0316), 0x01 },
	{ CCI_REG8(0x0a67), 0x80 },
	{ CCI_REG8(0x00a4), 0x00 },
	{ CCI_REG8(0x00a5), 0x01 },
	{ CCI_REG8(0x00a2), 0x00 },
	{ CCI_REG8(0x00a3), 0x00 },
	{ CCI_REG8(0x00ab), 0x00 },
	{ CCI_REG8(0x00ac), 0x00 },
	{ CCI_REG8(0x00a7), 0x09 },
	{ CCI_REG8(0x00a8), 0xa0 },
	{ CCI_REG8(0x00a9), 0x0c },
	{ CCI_REG8(0x00aa), 0xd0 },
	{ CCI_REG8(0x0aaa), 0x01 },
	{ CCI_REG8(0x0aab), 0x60 },
	{ CCI_REG8(0x0aac), 0x29 },
	{ CCI_REG8(0x0aad), 0xe0 },
	{ CCI_REG8(0x0ab0), 0x0f },
	{ CCI_REG8(0x0ab1), 0x26 },
	{ CCI_REG8(0x0ab2), 0xf8 },
	{ CCI_REG8(0x0a91), 0xf2 },
	{ CCI_REG8(0x0a92), 0x12 },
	{ CCI_REG8(0x0a93), 0x64 },
	{ CCI_REG8(0x0a95), 0x41 },
	{ CCI_REG8(0x0a90), 0x17 },
	{ CCI_REG8(0x0313), 0x80 },
	{ CCI_REG8(0x02db), 0x01 },
	{ CCI_REG8(0x0b00), 0xd3 },
	{ CCI_REG8(0x0b01), 0x15 },
	{ CCI_REG8(0x0b02), 0x03 },
	{ CCI_REG8(0x0b03), 0x00 },
	{ CCI_REG8(0x0b04), 0xb4 },
	{ CCI_REG8(0x0b05), 0x0d },
	{ CCI_REG8(0x0b06), 0x0c },
	{ CCI_REG8(0x0b07), 0x01 },
	{ CCI_REG8(0x0b08), 0x02 },
	{ CCI_REG8(0x0b09), 0xa7 },
	{ CCI_REG8(0x0b0a), 0x01 },
	{ CCI_REG8(0x0b0b), 0x00 },
	{ CCI_REG8(0x0b0c), 0x11 },
	{ CCI_REG8(0x0b0d), 0x14 },
	{ CCI_REG8(0x0b0e), 0x03 },
	{ CCI_REG8(0x0b0f), 0x00 },
	{ CCI_REG8(0x0b10), 0x1b },
	{ CCI_REG8(0x0b11), 0x10 },
	{ CCI_REG8(0x0b12), 0x0c },
	{ CCI_REG8(0x0b13), 0x01 },
	{ CCI_REG8(0x0b14), 0x46 },
	{ CCI_REG8(0x0b15), 0x80 },
	{ CCI_REG8(0x0b16), 0x01 },
	{ CCI_REG8(0x0b17), 0x00 },
	{ CCI_REG8(0x0b18), 0xf0 },
	{ CCI_REG8(0x0b19), 0x81 },
	{ CCI_REG8(0x0b1a), 0x01 },
	{ CCI_REG8(0x0b1b), 0x00 },
	{ CCI_REG8(0x0b1c), 0x55 },
	{ CCI_REG8(0x0b1d), 0x84 },
	{ CCI_REG8(0x0b1e), 0x01 },
	{ CCI_REG8(0x0b1f), 0x00 },
	{ CCI_REG8(0x0b20), 0x03 },
	{ CCI_REG8(0x0b21), 0xe3 },
	{ CCI_REG8(0x0b22), 0x01 },
	{ CCI_REG8(0x0b23), 0x00 },
	{ CCI_REG8(0x0b24), 0x40 },
	{ CCI_REG8(0x0b25), 0x64 },
	{ CCI_REG8(0x0b26), 0x02 },
	{ CCI_REG8(0x0b27), 0x00 },
	{ CCI_REG8(0x0b28), 0x12 },
	{ CCI_REG8(0x0b29), 0x1c },
	{ CCI_REG8(0x0b2a), 0x03 },
	{ CCI_REG8(0x0b2b), 0x00 },
	{ CCI_REG8(0x0b2c), 0x80 },
	{ CCI_REG8(0x0b2d), 0x1c },
	{ CCI_REG8(0x0b2e), 0x03 },
	{ CCI_REG8(0x0b2f), 0x00 },
	{ CCI_REG8(0x0b30), 0x10 },
	{ CCI_REG8(0x0b31), 0xfe },
	{ CCI_REG8(0x0b32), 0x03 },
	{ CCI_REG8(0x0b33), 0x00 },
	{ CCI_REG8(0x0b34), 0x00 },
	{ CCI_REG8(0x0b35), 0xfe },
	{ CCI_REG8(0x0b36), 0x03 },
	{ CCI_REG8(0x0b37), 0x00 },
	{ CCI_REG8(0x0b38), 0x9f },
	{ CCI_REG8(0x0b39), 0x1c },
	{ CCI_REG8(0x0b3a), 0x03 },
	{ CCI_REG8(0x0b3b), 0x00 },
	{ CCI_REG8(0x0b3c), 0x00 },
	{ CCI_REG8(0x0b3d), 0xfe },
	{ CCI_REG8(0x0b3e), 0x03 },
	{ CCI_REG8(0x0b3f), 0x00 },
	{ CCI_REG8(0x0b40), 0x00 },
	{ CCI_REG8(0x0b41), 0xfe },
	{ CCI_REG8(0x0b42), 0x03 },
	{ CCI_REG8(0x0b43), 0x00 },
	{ CCI_REG8(0x0b44), 0x00 },
	{ CCI_REG8(0x0b45), 0xfe },
	{ CCI_REG8(0x0b46), 0x03 },
	{ CCI_REG8(0x0b47), 0x00 },
	{ CCI_REG8(0x0b48), 0x80 },
	{ CCI_REG8(0x0b49), 0x1c },
	{ CCI_REG8(0x0b4a), 0x03 },
	{ CCI_REG8(0x0b4b), 0x00 },
	{ CCI_REG8(0x0b4c), 0x10 },
	{ CCI_REG8(0x0b4d), 0xfe },
	{ CCI_REG8(0x0b4e), 0x03 },
	{ CCI_REG8(0x0b4f), 0x00 },
	{ CCI_REG8(0x0b50), 0x00 },
	{ CCI_REG8(0x0b51), 0xfe },
	{ CCI_REG8(0x0b52), 0x03 },
	{ CCI_REG8(0x0b53), 0x00 },
	{ CCI_REG8(0x0b54), 0x9f },
	{ CCI_REG8(0x0b55), 0x1c },
	{ CCI_REG8(0x0b56), 0x03 },
	{ CCI_REG8(0x0b57), 0x00 },
	{ CCI_REG8(0x0b58), 0x99 },
	{ CCI_REG8(0x0b59), 0x02 },
	{ CCI_REG8(0x0b5a), 0x01 },
	{ CCI_REG8(0x0b5b), 0x00 },
	{ CCI_REG8(0x0b5c), 0x00 },
	{ CCI_REG8(0x0b5d), 0x64 },
	{ CCI_REG8(0x0b5e), 0x02 },
	{ CCI_REG8(0x0b5f), 0x00 },
	{ CCI_REG8(0x0b60), 0x00 },
	{ CCI_REG8(0x0b61), 0x02 },
	{ CCI_REG8(0x0b62), 0x01 },
	{ CCI_REG8(0x0b63), 0x00 },
	{ CCI_REG8(0x0b64), 0x06 },
	{ CCI_REG8(0x0b65), 0x80 },
	{ CCI_REG8(0x0b66), 0x01 },
	{ CCI_REG8(0x0b67), 0x00 },
	{ CCI_REG8(0x0b68), 0x00 },
	{ CCI_REG8(0x0b69), 0x81 },
	{ CCI_REG8(0x0b6a), 0x01 },
	{ CCI_REG8(0x0b6b), 0x00 },
	{ CCI_REG8(0x0b6c), 0x54 },
	{ CCI_REG8(0x0b6d), 0x84 },
	{ CCI_REG8(0x0b6e), 0x01 },
	{ CCI_REG8(0x0b6f), 0x00 },
	{ CCI_REG8(0x0b70), 0x60 },
	{ CCI_REG8(0x0b71), 0x1c },
	{ CCI_REG8(0x0b72), 0x03 },
	{ CCI_REG8(0x0b73), 0x00 },
	{ CCI_REG8(0x0b74), 0x02 },
	{ CCI_REG8(0x0b75), 0xe3 },
	{ CCI_REG8(0x0b76), 0x01 },
	{ CCI_REG8(0x0b77), 0x00 },
	{ CCI_REG8(0x0b78), 0x13 },
	{ CCI_REG8(0x0b79), 0x10 },
	{ CCI_REG8(0x0b7a), 0x0c },
	{ CCI_REG8(0x0b7b), 0x01 },
	{ CCI_REG8(0x0b7c), 0x01 },
	{ CCI_REG8(0x0b7d), 0x14 },
	{ CCI_REG8(0x0b7e), 0x03 },
	{ CCI_REG8(0x0b7f), 0x00 },
	{ CCI_REG8(0x0b80), 0x00 },
	{ CCI_REG8(0x0b81), 0xa7 },
	{ CCI_REG8(0x0b82), 0x01 },
	{ CCI_REG8(0x0b83), 0x00 },
	{ CCI_REG8(0x0b84), 0x34 },
	{ CCI_REG8(0x0b85), 0x0d },
	{ CCI_REG8(0x0b86), 0x0c },
	{ CCI_REG8(0x0b87), 0x01 },
	{ CCI_REG8(0x0b88), 0x53 },
	{ CCI_REG8(0x0b89), 0x15 },
	{ CCI_REG8(0x0b8a), 0x03 },
	{ CCI_REG8(0x0b8b), 0x01 },
	{ CCI_REG8(0x0aeb), 0x09 },
	{ CCI_REG8(0x0ae9), 0x17 },
	{ CCI_REG8(0x0aea), 0x23 },
	{ CCI_REG8(0x0ae8), 0xb0 },
	{ CCI_REG8(0x05a0), 0x82 },
	{ CCI_REG8(0x05ac), 0x00 },
	{ CCI_REG8(0x05ad), 0x01 },
	{ CCI_REG8(0x0597), 0x35 },
	{ CCI_REG8(0x05ab), 0x0a },
	{ CCI_REG8(0x05a3), 0x06 },
	{ CCI_REG8(0x05a4), 0x06 },
	{ CCI_REG8(0x05ae), 0x00 },
	{ CCI_REG8(0x0800), 0x0a },
	{ CCI_REG8(0x0801), 0x14 },
	{ CCI_REG8(0x0802), 0x22 },
	{ CCI_REG8(0x0803), 0x30 },
	{ CCI_REG8(0x0804), 0x42 },
	{ CCI_REG8(0x0805), 0x0e },
	{ CCI_REG8(0x0806), 0x66 },
	{ CCI_REG8(0x0807), 0x0e },
	{ CCI_REG8(0x0808), 0x65 },
	{ CCI_REG8(0x0809), 0x02 },
	{ CCI_REG8(0x080a), 0xc3 },
	{ CCI_REG8(0x080b), 0x02 },
	{ CCI_REG8(0x080c), 0xc7 },
	{ CCI_REG8(0x080d), 0x02 },
	{ CCI_REG8(0x080e), 0xcb },
	{ CCI_REG8(0x080f), 0x00 },
	{ CCI_REG8(0x0810), 0xc0 },
	{ CCI_REG8(0x0811), 0x16 },
	{ CCI_REG8(0x0812), 0x01 },
	{ CCI_REG8(0x0813), 0x1d },
	{ CCI_REG8(0x0814), 0x1d },
	{ CCI_REG8(0x0815), 0x1d },
	{ CCI_REG8(0x0816), 0x00 },
	{ CCI_REG8(0x0817), 0x08 },
	{ CCI_REG8(0x0818), 0x01 },
	{ CCI_REG8(0x0819), 0x3d },
	{ CCI_REG8(0x081a), 0x3d },
	{ CCI_REG8(0x081b), 0x3d },
	{ CCI_REG8(0x081c), 0x00 },
	{ CCI_REG8(0x081d), 0x12 },
	{ CCI_REG8(0x081e), 0x01 },
	{ CCI_REG8(0x081f), 0x05 },
	{ CCI_REG8(0x0820), 0x05 },
	{ CCI_REG8(0x0821), 0x05 },
	{ CCI_REG8(0x0822), 0x00 },
	{ CCI_REG8(0x0823), 0x0e },
	{ CCI_REG8(0x0824), 0x01 },
	{ CCI_REG8(0x0825), 0x05 },
	{ CCI_REG8(0x0826), 0x05 },
	{ CCI_REG8(0x0827), 0x05 },
	{ CCI_REG8(0x0828), 0x00 },
	{ CCI_REG8(0x0829), 0x0b },
	{ CCI_REG8(0x082a), 0x01 },
	{ CCI_REG8(0x082b), 0x05 },
	{ CCI_REG8(0x082c), 0x05 },
	{ CCI_REG8(0x082d), 0x05 },
	{ CCI_REG8(0x082e), 0x02 },
	{ CCI_REG8(0x082f), 0x06 },
	{ CCI_REG8(0x0830), 0x01 },
	{ CCI_REG8(0x0831), 0x05 },
	{ CCI_REG8(0x0832), 0x05 },
	{ CCI_REG8(0x0833), 0x05 },
	{ CCI_REG8(0x0834), 0x02 },
	{ CCI_REG8(0x0835), 0x01 },
	{ CCI_REG8(0x0836), 0x00 },
	{ CCI_REG8(0x0837), 0x00 },
	{ CCI_REG8(0x0838), 0x00 },
	{ CCI_REG8(0x0839), 0x01 },
	{ CCI_REG8(0x083a), 0x68 },
	{ CCI_REG8(0x083b), 0x00 },
	{ CCI_REG8(0x083c), 0x01 },
	{ CCI_REG8(0x083d), 0x01 },
	{ CCI_REG8(0x083e), 0xf6 },
	{ CCI_REG8(0x083f), 0x00 },
	{ CCI_REG8(0x0840), 0x02 },
	{ CCI_REG8(0x0841), 0x02 },
	{ CCI_REG8(0x0842), 0xc9 },
	{ CCI_REG8(0x0843), 0x00 },
	{ CCI_REG8(0x0844), 0x03 },
	{ CCI_REG8(0x0845), 0x03 },
	{ CCI_REG8(0x0846), 0xe2 },
	{ CCI_REG8(0x0847), 0x00 },
	{ CCI_REG8(0x0848), 0x04 },
	{ CCI_REG8(0x0849), 0x05 },
	{ CCI_REG8(0x084a), 0x8c },
	{ CCI_REG8(0x084b), 0x00 },
	{ CCI_REG8(0x084c), 0x05 },
	{ CCI_REG8(0x084d), 0x07 },
	{ CCI_REG8(0x084e), 0xba },
	{ CCI_REG8(0x084f), 0x00 },
	{ CCI_REG8(0x0850), 0x06 },
	{ CCI_REG8(0x0851), 0x0b },
	{ CCI_REG8(0x0852), 0x3e },
	{ CCI_REG8(0x0853), 0x10 },
	{ CCI_REG8(0x0854), 0x05 },
	{ CCI_REG8(0x0855), 0x0f },
	{ CCI_REG8(0x0856), 0xaa },
	{ CCI_REG8(0x0857), 0x10 },
	{ CCI_REG8(0x0858), 0x06 },
	{ CCI_REG8(0x0859), 0x1e },
	{ CCI_REG8(0x085a), 0xe0 },
	{ CCI_REG8(0x085b), 0x18 },
	{ CCI_REG8(0x085c), 0x06 },
	{ CCI_REG8(0x05a0), 0xc2 },
	{ CCI_REG8(0x05ac), 0x01 },
	{ CCI_REG8(0x05ae), 0x00 },
	{ CCI_REG8(0x0207), 0x04 },
	{ CCI_REG8(0x0070), 0x05 },
	{ CCI_REG8(0x0080), 0xd0 },
	{ CCI_REG8(0x0089), 0x83 },
	{ CCI_REG8(0x009a), 0x00 },
	{ CCI_REG8(0x00a0), 0x03 },
	{ CCI_REG8(0x0c20), 0x10 },
	{ CCI_REG8(0x0c21), 0xc8 },
	{ CCI_REG8(0x0c22), 0xc8 },
	{ CCI_REG8(0x0c50), 0x10 },
	{ CCI_REG8(0x0c51), 0xc8 },
	{ CCI_REG8(0x0c52), 0xc8 },
	{ CCI_REG8(0x0040), 0x22 },
	{ CCI_REG8(0x0041), 0x20 },
	{ CCI_REG8(0x0042), 0x20 },
	{ CCI_REG8(0x0043), 0x0f },
	{ CCI_REG8(0x0044), 0x00 },
	{ CCI_REG8(0x0046), 0x0c },
	{ CCI_REG8(0x0049), 0x06 },
	{ CCI_REG8(0x004a), 0x19 },
	{ CCI_REG8(0x004d), 0x00 },
	{ CCI_REG8(0x004e), 0x03 },
	{ CCI_REG8(0x0051), 0x26 },
	{ CCI_REG8(0x005a), 0x0c },
	{ CCI_REG8(0x005b), 0x03 },
	{ CCI_REG8(0x021a), 0x00 },
	{ CCI_REG8(0x0450), 0x02 },
	{ CCI_REG8(0x0452), 0x02 },
	{ CCI_REG8(0x0454), 0x02 },
	{ CCI_REG8(0x0456), 0x02 },
	{ CCI_REG8(0x0204), 0x04 },
	{ CCI_REG8(0x0205), 0x00 },
	{ CCI_REG8(0x0208), 0x01 },
	{ CCI_REG8(0x0209), 0x65 },
	{ CCI_REG8(0x0096), 0x81 },
	{ CCI_REG8(0x0097), 0x01 },
	{ CCI_REG8(0x0098), 0x87 },
	{ CCI_REG8(0x00c0), 0x00 },
	{ CCI_REG8(0x00c1), 0x80 },
	{ CCI_REG8(0x00c2), 0x11 },
	{ CCI_REG8(0x00c3), 0x00 },
	{ CCI_REG8(0x0480), 0x04 },
	{ CCI_REG8(0x0482), 0x06 },
	{ CCI_REG8(0x0484), 0x10 },
	{ CCI_REG8(0x0486), 0x10 },
	{ CCI_REG8(0x0488), 0x10 },
	{ CCI_REG8(0x048a), 0x0c },
	{ CCI_REG8(0x048c), 0x0c },
	{ CCI_REG8(0x048e), 0x10 },
	{ CCI_REG8(0x0481), 0x03 },
	{ CCI_REG8(0x0483), 0x04 },
	{ CCI_REG8(0x0485), 0x04 },
	{ CCI_REG8(0x0487), 0x05 },
	{ CCI_REG8(0x0489), 0x05 },
	{ CCI_REG8(0x048b), 0x06 },
	{ CCI_REG8(0x048d), 0x06 },
	{ CCI_REG8(0x048f), 0x04 },
	{ CCI_REG8(0x0490), 0x04 },
	{ CCI_REG8(0x0492), 0x10 },
	{ CCI_REG8(0x0494), 0x18 },
	{ CCI_REG8(0x0496), 0x12 },
	{ CCI_REG8(0x0498), 0x12 },
	{ CCI_REG8(0x049a), 0x30 },
	{ CCI_REG8(0x049c), 0x30 },
	{ CCI_REG8(0x049e), 0x60 },
	{ CCI_REG8(0x0491), 0x04 },
	{ CCI_REG8(0x0493), 0x05 },
	{ CCI_REG8(0x0495), 0x05 },
	{ CCI_REG8(0x0497), 0x05 },
	{ CCI_REG8(0x0499), 0x05 },
	{ CCI_REG8(0x049b), 0x05 },
	{ CCI_REG8(0x049d), 0x05 },
	{ CCI_REG8(0x049f), 0x04 },
	{ CCI_REG8(0x0351), 0x00 },
	{ CCI_REG8(0x0352), 0x08 },
	{ CCI_REG8(0x0353), 0x00 },
	{ CCI_REG8(0x0354), 0x08 },
	{ CCI_REG8(0x034c), 0x0c },
	{ CCI_REG8(0x034d), 0xc0 },
	{ CCI_REG8(0x034e), 0x09 },
	{ CCI_REG8(0x034f), 0x90 },
	{ CCI_REG8(0x0180), 0x46 },
	{ CCI_REG8(0x0181), 0xf0 },
	{ CCI_REG8(0x0182), 0x55 },
	{ CCI_REG8(0x0183), 0x55 },
	{ CCI_REG8(0x0184), 0x55 },
	{ CCI_REG8(0x0186), 0x5f },
	{ CCI_REG8(0x0187), 0x00 },
	{ CCI_REG8(0x0188), 0x00 },
	{ CCI_REG8(0x0189), 0x00 },
	{ CCI_REG8(0x0107), 0x00 },
	{ CCI_REG8(0x010b), 0x12 },
	{ CCI_REG8(0x0115), 0x00 },
	{ CCI_REG8(0x0121), 0x12 },
	{ CCI_REG8(0x0122), 0x07 },
	{ CCI_REG8(0x0123), 0x1f },
	{ CCI_REG8(0x0124), 0x02 },
	{ CCI_REG8(0x0125), 0x16 },
	{ CCI_REG8(0x0126), 0x08 },
	{ CCI_REG8(0x0127), 0x10 },
	{ CCI_REG8(0x0129), 0x07 },
	{ CCI_REG8(0x012a), 0x1f },
	{ CCI_REG8(0x012b), 0x08 },
	{ CCI_REG8(0x0084), 0x10 },
	{ CCI_REG8(0x0a93), 0x60 },
	{ CCI_REG8(0x0a90), 0x11 },
	{ CCI_REG8(0x0313), 0x80 },
	{ CCI_REG8(0x0aff), 0x00 },
	{ CCI_REG8(0x0aff), 0x00 },
	{ CCI_REG8(0x0aff), 0x00 },
	{ CCI_REG8(0x0aff), 0x00 },
	{ CCI_REG8(0x0aff), 0x00 },
	{ CCI_REG8(0x0aff), 0x00 },
	{ CCI_REG8(0x0aff), 0x00 },
	{ CCI_REG8(0x0aff), 0x00 },
	{ CCI_REG8(0x0a90), 0x00 },
	{ CCI_REG8(0x00a4), 0x80 },
	{ CCI_REG8(0x0316), 0x00 },
	{ CCI_REG8(0x0a67), 0x00 },
	{ CCI_REG8(0x031c), 0x12 },
	{ CCI_REG8(0x031c), 0x80 },
	{ CCI_REG8(0x03fe), 0x10 },
	{ CCI_REG8(0x03fe), 0x00 },
	{ CCI_REG8(0x031c), 0x9f },
	{ CCI_REG8(0x03fe), 0x00 },
	{ CCI_REG8(0x03fe), 0x00 },
	{ CCI_REG8(0x03fe), 0x00 },
	{ CCI_REG8(0x031c), 0x80 },
	{ CCI_REG8(0x03fe), 0x10 },
	{ CCI_REG8(0x03fe), 0x00 },
	{ CCI_REG8(0x031c), 0x9f },
	{ CCI_REG8(0x0102), 0x99 },
};

struct gc32e1 {
	struct device *dev;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct regmap *regmap;

	struct clk *xclk;
	struct regulator_bulk_data supplies[ARRAY_SIZE(gc32e1_supply_names)];
	struct gpio_desc *reset_gpio;

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
};

static inline struct gc32e1 *to_gc32e1(struct v4l2_subdev *sd)
{
	return container_of(sd, struct gc32e1, sd);
}

static int gc32e1_power_on(struct device *dev)
{
	struct gc32e1 *gc32e1 = to_gc32e1(dev_get_drvdata(dev));
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(gc32e1->supplies),
				    gc32e1->supplies);
	if (ret) {
		dev_err(dev, "failed to enable regulators: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(gc32e1->xclk);
	if (ret) {
		dev_err(dev, "failed to enable clock: %d\n", ret);
		regulator_bulk_disable(ARRAY_SIZE(gc32e1->supplies),
				       gc32e1->supplies);
		return ret;
	}

	fsleep(2000);
	gpiod_set_value_cansleep(gc32e1->reset_gpio, 0);
	fsleep(5000);

	return 0;
}

static int gc32e1_power_off(struct device *dev)
{
	struct gc32e1 *gc32e1 = to_gc32e1(dev_get_drvdata(dev));

	gpiod_set_value_cansleep(gc32e1->reset_gpio, 1);
	clk_disable_unprepare(gc32e1->xclk);
	regulator_bulk_disable(ARRAY_SIZE(gc32e1->supplies), gc32e1->supplies);

	return 0;
}

static int gc32e1_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct gc32e1 *gc32e1 = container_of(ctrl->handler, struct gc32e1,
					     ctrls);
	int ret = 0;

	if (ctrl->id == V4L2_CID_VBLANK) {
		s64 exposure_max = GC32E1_HEIGHT + ctrl->val -
				   GC32E1_EXPOSURE_MARGIN;

		ret = __v4l2_ctrl_modify_range(gc32e1->exposure,
					       gc32e1->exposure->minimum,
					       exposure_max,
					       gc32e1->exposure->step,
					       min(gc32e1->exposure->default_value,
						   exposure_max));
		if (ret)
			return ret;
	}

	/* the controls are applied when streaming starts */
	if (!pm_runtime_get_if_active(gc32e1->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = cci_write(gc32e1->regmap, GC32E1_REG_EXPOSURE, ctrl->val,
				NULL);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(gc32e1->regmap, GC32E1_REG_AGAIN, ctrl->val,
				NULL);
		break;
	case V4L2_CID_VBLANK:
		/* the frame length must be even */
		ret = cci_write(gc32e1->regmap, GC32E1_REG_FRAME_LENGTH,
				(GC32E1_HEIGHT + ctrl->val) & ~1, NULL);
		break;
	case V4L2_CID_TEST_PATTERN:
		cci_write(gc32e1->regmap, GC32E1_REG_TEST_PATTERN_IDX,
			  ctrl->val == 2 ? 0x10 : 0x00, &ret);
		cci_write(gc32e1->regmap, GC32E1_REG_TEST_PATTERN_EN,
			  ctrl->val ? 1 : 0, &ret);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(gc32e1->dev);

	return ret;
}

static const struct v4l2_ctrl_ops gc32e1_ctrl_ops = {
	.s_ctrl = gc32e1_set_ctrl,
};

static int gc32e1_init_controls(struct gc32e1 *gc32e1)
{
	struct v4l2_ctrl_handler *hdl = &gc32e1->ctrls;
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl *ctrl;
	int ret;

	v4l2_ctrl_handler_init(hdl, 10);

	ctrl = v4l2_ctrl_new_int_menu(hdl, &gc32e1_ctrl_ops, V4L2_CID_LINK_FREQ,
				      ARRAY_SIZE(gc32e1_link_freqs) - 1, 0,
				      gc32e1_link_freqs);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(hdl, &gc32e1_ctrl_ops, V4L2_CID_PIXEL_RATE,
			  GC32E1_PIXEL_RATE, GC32E1_PIXEL_RATE, 1,
			  GC32E1_PIXEL_RATE);

	gc32e1->hblank = v4l2_ctrl_new_std(hdl, &gc32e1_ctrl_ops,
					   V4L2_CID_HBLANK,
					   GC32E1_HTS - GC32E1_WIDTH,
					   GC32E1_HTS - GC32E1_WIDTH, 1,
					   GC32E1_HTS - GC32E1_WIDTH);
	if (gc32e1->hblank)
		gc32e1->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	gc32e1->vblank = v4l2_ctrl_new_std(hdl, &gc32e1_ctrl_ops,
					   V4L2_CID_VBLANK,
					   GC32E1_VTS_DEF - GC32E1_HEIGHT,
					   GC32E1_VTS_MAX - GC32E1_HEIGHT, 2,
					   GC32E1_VTS_DEF - GC32E1_HEIGHT);

	gc32e1->exposure = v4l2_ctrl_new_std(hdl, &gc32e1_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     GC32E1_EXPOSURE_MIN,
					     GC32E1_VTS_DEF -
					     GC32E1_EXPOSURE_MARGIN, 1,
					     GC32E1_VTS_DEF -
					     GC32E1_EXPOSURE_MARGIN);

	v4l2_ctrl_new_std(hdl, &gc32e1_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  GC32E1_AGAIN_MIN, GC32E1_AGAIN_MAX, 1,
			  GC32E1_AGAIN_MIN);

	v4l2_ctrl_new_std_menu_items(hdl, &gc32e1_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(gc32e1_test_pattern_menu) - 1,
				     0, 0, gc32e1_test_pattern_menu);

	ret = v4l2_fwnode_device_parse(gc32e1->dev, &props);
	if (ret)
		goto err_free;

	v4l2_ctrl_new_fwnode_properties(hdl, &gc32e1_ctrl_ops, &props);

	if (hdl->error) {
		ret = hdl->error;
		goto err_free;
	}

	gc32e1->sd.ctrl_handler = hdl;

	return 0;

err_free:
	v4l2_ctrl_handler_free(hdl);
	return ret;
}

static void gc32e1_fill_format(struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = GC32E1_WIDTH;
	fmt->height = GC32E1_HEIGHT;
	fmt->code = GC32E1_MBUS_CODE;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int gc32e1_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;

	code->code = GC32E1_MBUS_CODE;

	return 0;
}

static int gc32e1_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index || fse->code != GC32E1_MBUS_CODE)
		return -EINVAL;

	fse->min_width = GC32E1_WIDTH;
	fse->max_width = GC32E1_WIDTH;
	fse->min_height = GC32E1_HEIGHT;
	fse->max_height = GC32E1_HEIGHT;

	return 0;
}

static int gc32e1_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	gc32e1_fill_format(&fmt->format);
	*v4l2_subdev_state_get_format(state, 0) = fmt->format;

	return 0;
}

static int gc32e1_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_NATIVE_SIZE:
		/* the binned mode covers the full (2x) pixel array */
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = GC32E1_WIDTH * 2;
		sel->r.height = GC32E1_HEIGHT * 2;
		return 0;
	default:
		return -EINVAL;
	}
}

static int gc32e1_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	gc32e1_fill_format(v4l2_subdev_state_get_format(state, 0));

	return 0;
}

static int gc32e1_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct gc32e1 *gc32e1 = to_gc32e1(sd);
	int ret;

	ret = pm_runtime_resume_and_get(gc32e1->dev);
	if (ret)
		return ret;

	ret = cci_multi_reg_write(gc32e1->regmap, gc32e1_init_regs,
				  ARRAY_SIZE(gc32e1_init_regs), NULL);
	if (ret)
		goto err_put;

	ret = cci_multi_reg_write(gc32e1->regmap, gc32e1_mode_3264x2448_regs,
				  ARRAY_SIZE(gc32e1_mode_3264x2448_regs), NULL);
	if (ret)
		goto err_put;

	ret = __v4l2_ctrl_handler_setup(&gc32e1->ctrls);
	if (ret)
		goto err_put;

	ret = cci_write(gc32e1->regmap, GC32E1_REG_STREAMING, 1, NULL);
	if (ret)
		goto err_put;

	return 0;

err_put:
	dev_err(gc32e1->dev, "failed to start streaming: %d\n", ret);
	pm_runtime_put(gc32e1->dev);
	return ret;
}

static int gc32e1_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct gc32e1 *gc32e1 = to_gc32e1(sd);
	int ret;

	ret = cci_write(gc32e1->regmap, GC32E1_REG_STREAMING, 0, NULL);
	pm_runtime_put_autosuspend(gc32e1->dev);

	return ret;
}

static const struct v4l2_subdev_video_ops gc32e1_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops gc32e1_pad_ops = {
	.enum_mbus_code = gc32e1_enum_mbus_code,
	.enum_frame_size = gc32e1_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = gc32e1_set_fmt,
	.get_selection = gc32e1_get_selection,
	.enable_streams = gc32e1_enable_streams,
	.disable_streams = gc32e1_disable_streams,
};

static const struct v4l2_subdev_ops gc32e1_subdev_ops = {
	.video = &gc32e1_video_ops,
	.pad = &gc32e1_pad_ops,
};

static const struct v4l2_subdev_internal_ops gc32e1_internal_ops = {
	.init_state = gc32e1_init_state,
};

static int gc32e1_check_hwcfg(struct gc32e1 *gc32e1)
{
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct fwnode_handle *ep;
	unsigned long bitmap;
	int ret;

	ep = fwnode_graph_get_next_endpoint(dev_fwnode(gc32e1->dev), NULL);
	if (!ep)
		return dev_err_probe(gc32e1->dev, -ENXIO,
				     "endpoint node not found\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return dev_err_probe(gc32e1->dev, ret,
				     "failed to parse endpoint\n");

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != GC32E1_DATA_LANES) {
		ret = dev_err_probe(gc32e1->dev, -EINVAL,
				    "only %u data lanes are supported\n",
				    GC32E1_DATA_LANES);
		goto out;
	}

	ret = v4l2_link_freq_to_bitmap(gc32e1->dev, bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       gc32e1_link_freqs,
				       ARRAY_SIZE(gc32e1_link_freqs), &bitmap);
out:
	v4l2_fwnode_endpoint_free(&bus_cfg);
	return ret;
}

static int gc32e1_identify(struct gc32e1 *gc32e1)
{
	u64 hi, lo, id;
	int ret = 0;

	cci_read(gc32e1->regmap, GC32E1_REG_CHIP_ID_H, &hi, &ret);
	cci_read(gc32e1->regmap, GC32E1_REG_CHIP_ID_L, &lo, &ret);
	if (ret)
		return dev_err_probe(gc32e1->dev, ret,
				     "failed to read chip id\n");

	id = hi << 8 | lo;
	if (id != GC32E1_CHIP_ID)
		return dev_err_probe(gc32e1->dev, -ENODEV,
				     "unexpected chip id 0x%04llx\n", id);

	return 0;
}

static int gc32e1_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct gc32e1 *gc32e1;
	unsigned int i;
	int ret;

	gc32e1 = devm_kzalloc(dev, sizeof(*gc32e1), GFP_KERNEL);
	if (!gc32e1)
		return -ENOMEM;

	gc32e1->dev = dev;
	v4l2_i2c_subdev_init(&gc32e1->sd, client, &gc32e1_subdev_ops);
	gc32e1->sd.internal_ops = &gc32e1_internal_ops;

	ret = gc32e1_check_hwcfg(gc32e1);
	if (ret)
		return ret;

	gc32e1->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(gc32e1->regmap))
		return dev_err_probe(dev, PTR_ERR(gc32e1->regmap),
				     "failed to init CCI\n");

	gc32e1->xclk = devm_v4l2_sensor_clk_get_legacy(dev, NULL, true,
						       GC32E1_XCLK_FREQ);
	if (IS_ERR(gc32e1->xclk))
		return dev_err_probe(dev, PTR_ERR(gc32e1->xclk),
				     "failed to get clock\n");

	for (i = 0; i < ARRAY_SIZE(gc32e1_supply_names); i++)
		gc32e1->supplies[i].supply = gc32e1_supply_names[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(gc32e1->supplies),
				      gc32e1->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	gc32e1->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(gc32e1->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(gc32e1->reset_gpio),
				     "failed to get reset gpio\n");

	ret = gc32e1_power_on(dev);
	if (ret)
		return ret;

	ret = gc32e1_identify(gc32e1);
	if (ret)
		goto err_power_off;

	ret = gc32e1_init_controls(gc32e1);
	if (ret)
		goto err_power_off;

	gc32e1->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	gc32e1->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	gc32e1->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&gc32e1->sd.entity, 1, &gc32e1->pad);
	if (ret)
		goto err_ctrls;

	gc32e1->sd.state_lock = gc32e1->ctrls.lock;
	ret = v4l2_subdev_init_finalize(&gc32e1->sd);
	if (ret)
		goto err_entity;

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_idle(dev);

	ret = v4l2_async_register_subdev_sensor(&gc32e1->sd);
	if (ret)
		goto err_pm;

	return 0;

err_pm:
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	v4l2_subdev_cleanup(&gc32e1->sd);
err_entity:
	media_entity_cleanup(&gc32e1->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&gc32e1->ctrls);
err_power_off:
	gc32e1_power_off(dev);
	return ret;
}

static void gc32e1_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc32e1 *gc32e1 = to_gc32e1(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&gc32e1->ctrls);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		gc32e1_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

static DEFINE_RUNTIME_DEV_PM_OPS(gc32e1_pm_ops, gc32e1_power_off,
				 gc32e1_power_on, NULL);

static const struct of_device_id gc32e1_of_match[] = {
	{ .compatible = "galaxycore,gc32e1" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, gc32e1_of_match);

static struct i2c_driver gc32e1_driver = {
	.driver = {
		.name = "gc32e1",
		.of_match_table = gc32e1_of_match,
		.pm = pm_ptr(&gc32e1_pm_ops),
	},
	.probe = gc32e1_probe,
	.remove = gc32e1_remove,
};
module_i2c_driver(gc32e1_driver);

MODULE_DESCRIPTION("GalaxyCore GC32E1 image sensor driver");
MODULE_AUTHOR("Jan Benedikt <karagos01@gmail.com>");
MODULE_LICENSE("GPL");
