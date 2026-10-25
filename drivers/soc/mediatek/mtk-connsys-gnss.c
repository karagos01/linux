// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek connsys GNSS (GPS) subsystem bring-up.
 *
 * The GPS subsystem is the third radio inside the connsys combo (next to
 * WiFi and Bluetooth, sharing the MT6637 A-die). Unlike those two, the GPS
 * power-on and the sensitive conn_infra/A-die control is performed by the
 * resident TF-A through the MTK_SIP_KERNEL_GPS_CONTROL SMC service; the
 * kernel only orchestrates the sequence and talks to the GPS MCU afterwards.
 *
 * The shared conn_infra and A-die are powered through the connsys driver
 * (as for Bluetooth); the GPS-specific "common on" then runs on top of that.
 *
 * Milestone 1: prove the secure power path works from mainline. The probe
 * (gated behind the probe_power module parameter) powers the shared connsys,
 * wakes conn_infra and asks TF-A to read back the conn_infra version through
 * the GPS common_on part1 SMC. A sane version with a zero return is a live
 * confirmation that the SMC service is registered and the GPS secure power
 * path is reachable; everything is then unwound.
 *
 * Note: a full standalone conn_infra core bring-up (without WiFi/BT having
 * initialised it) is not done yet - that sequence is shared with the WiFi
 * glue and is the next milestone.
 *
 * Copyright (C) 2026 ...
 */

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/arm-smccc.h>
#include <linux/soc/mediatek/mtk-connsys.h>
#include <linux/soc/mediatek/mtk_sip_svc.h>

/* op ids of the MTK_SIP_KERNEL_GPS_CONTROL service (enum conn_smc_opid) */
#define GPS_SMC_COMMON_ON_SET_FLAG	0x0
#define GPS_SMC_COMMON_ON_PART1		0x2
#define GPS_SMC_COMMON_ON_PART2		0x3	/* EMI remap: x2 = 20msb of 36-bit phys */
#define GPS_SMC_COMMON_ON_PART3		0x4
#define GPS_SMC_COMMON_ON_PART4		0x5
#define GPS_SMC_SET_ADIE_CHIPID		0x4a	/* x2 = A-die chip id (0x6637) */
#define GPS_SMC_COMMON_ON_FAIL_HANDLER	0x8
#define GPS_SMC_COMMON_ON_INNER_FAIL	0x9
#define GPS_SMC_COMMON_ON_PART5		0xa
#define GPS_SMC_COMMON_OFF_PART1	0xf
#define GPS_SMC_COMMON_OFF_PART2	0x10
#define GPS_SMC_SW_REQUEST_EMI_USAGE	0x11
#define GPS_SMC_COMMON_OFF_PART3	0x12
#define GPS_SMC_PWR_STAT_CTRL		0xb	/* x2 = dsp_ctrl_enum */
#define GPS_SMC_SET_DSP_ON_POLL_ACK	0xc	/* x2 = link id; a0 != 0 = ACK */
#define GPS_SMC_SET_CFG_DSP_MEM_OFF	0xe	/* x2 = dsp_ctrl_enum, x3 = link */

#define GPS_SMC_USRT_CTRL		0xd	/* x2=link x3=on x4=dma x5=1byte */
#define GPS_SMC_GET_MCUB_INFO		0x13	/* a0=ret a1=flag a2=d0 a3=d1 */
#define GPS_SMC_CLEAR_MCUB_D2A		0x14	/* x2=link x3=d2a_flag */

/*
 * GPS data-link DMA transport SMCs (enum conn_smc_opid). The TF-A owns the
 * GPS EMI ring layout (it programmed the conn_infra EMI remap in common_on),
 * so these take only the channel, a relative sub-buffer offset and the ring
 * lengths - no absolute address. Arg layout of SET_DMA_START (from the stock
 * plat/v051 gps_dl_hw_dep_set_dma_start):
 *   x2 = channel, x3 = sub_buf_offset (= data_start - buf_start, bus remap),
 *   x4 = bus_id, x5 = len_to_wrap, x6 = transfer_max_len, x7 = is_1byte.
 * Lengths are in 4-byte words unless 1-byte mode is selected.
 */
#define GPS_SMC_SET_DMA_START		0x16	/* see arg layout above */
#define GPS_SMC_GET_DMA_LEFT_LEN	0x1b	/* x2=channel; a0=left len (words) */
#define GPS_SMC_SET_DMA_STOP		0x1c	/* x2=channel */
#define GPS_SMC_GET_DMA_INT_STATUS	0x1e	/* x2=channel; a0!=0 = done */

/* enum gps_dl_hal_dma_ch_index / gps_dl_hal_dma_ch_bus_id (link0 only here) */
#define GPS_DL_DMA_LINK0_A2D		0	/* AP -> DSP */
#define GPS_DL_DMA_LINK0_D2A		1	/* DSP -> AP */
#define GPS_DL_DMA_BUS_ID0_A2D		0
#define GPS_DL_DMA_BUS_ID0_D2A		1

/* v051 per-link ring size (connac3); the reserved region adds 4K padding */
#define GPS_DL_RX_BUF_SIZE		(8 * 1024)

/*
 * link0 A2D (AP->DSP) TX ring base in the GPS reserved EMI. From the v051
 * reserved_mem layout: icap(0x50000) + padding(0x1000) puts tx[0] at
 * base + 0x51000. The AP writes the TX frame here and the DMA streams it to
 * the DSP over the USRT.
 */
#define GPS_EMI_TX0_PHYS		(GPS_EMI_PHYS_BASE + 0x51000)

/*
 * MVCD "DSP code download" SMCs. The GNSS DSP RAM image lives in TF-A; the AP
 * only drives the handshake. GET_DSP_BOOT_UP_INFO returns the image geometry
 * (fragment count etc.), SEND_DSP_FRAGEMENT pushes fragment N to the DSP (no
 * data argument - TF-A sources the bytes and streams them over the USRT/DMA
 * data link, which is why the D2A ring must be draining while this runs).
 *   x2 = dsp_type (0 = L1), x3 = fragment index (for SEND).
 */
#define GPS_SMC_MVCD_GET_DSP_BOOTUP	0x1f
#define GPS_SMC_MVCD_SEND_DSP_FRAG	0x20
#define GPS_MVCD_DSP_L1			0

/* USRT flow-control SMCs used around each fragment (x2 = link id) */
#define GPS_SMC_USRT_POLL_RX_EMPTY	0x15
#define GPS_SMC_USRT_CLEAR_NODATA	0x1a

/* enum dsp_ctrl_enum / link ids passed to the TF-A DSP control SMCs */
#define GPS_L1_DSP_ON			0
#define GPS_L1_DSP_OFF			1
#define GPS_DATA_LINK_ID0		0

/* MCUB mailbox DSP->AP (D2A) flag bits (gps_dsp_fsm.h) */
#define GPS_MCUB_D2A_RAMCODE_READY	0x1
#define GPS_MCUB_D2A_REG_READ_READY	0x2
#define GPS_MCUB_D2A_REQUEST_ACTION	0x4
#define GPS_MCUB_D2A_RESET_DONE		0x8

/* magic the TF-A expects in x2 of the SET_FLAG call */
#define GPS_SMC_SET_FLAG_MAGIC		0xdb9db9

/*
 * GPS reserved EMI, carved by the (stock) bootloader at this fixed base; the
 * secure common_on programs the conn_infra EMI remap with its top 20 bits
 * (of the 36-bit physical address, aligned to a 64KB boundary). Matches the
 * stock "emi-addr" of the gps@18c00000 node.
 */
#define GPS_EMI_PHYS_BASE		0x88a00000ULL
#define GPS_EMI_REMAP_BASE_MASK		0xfffff0000ULL

/* conn_infra registers, offsets into the "conn_infra" window (0x18000000) */
#define CONN_INFRA_WAKEUP_GPS		0x601ac	/* host CSR, BIT0 = force on */
#define CONN_INFRA_CFG_VERSION		0x11000	/* reads CONN_INFRA_CFG_ID when up */
#define CONN_INFRA_CFG_ID		0x02050401
#define CONN_INFRA_CFG_PWRCTRL1		0x01210	/* BIT16 = conn_infra ready */
#define CONN_INFRA_RDY_B		BIT(16)

struct mtk_gnss {
	struct device *dev;
	struct mtk_connsys *cs;
	void __iomem *conn;
	int irq_mcub;			/* link0 MCUB mailbox IRQ (SPI 381) */
	struct completion ramcode;	/* completed when the DSP posts RAMCODE_READY */
};

/*
 * The GNSS DSP RAM-code download handshake: the DSP will not act on the MVCD
 * fragments until userspace (mnld/libmnl) has sent it an "image descriptor"
 * control frame over the A2D data link just before the fragment loop. The
 * frame is aaf0-framed (aaf0 ... aa0f) and carries the L1 image geometry -
 * code_size 0xa609, start 0x2400, exec 0x6404, cipher 0x3bf2, frag_num 106,
 * all of which TF-A reports as stable across boots - so it can be replayed
 * verbatim. Captured from stock UT (mnld write() to /dev/gpsdl0, 116 bytes).
 */
static const u8 gps_l1_init_frame[] = {
	0xaa, 0xf0, 0x6e, 0x00, 0x08, 0xfe, 0x1a, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xc3, 0x01, 0xa5, 0x02, 0x00, 0x00, 0x00, 0x00, 0x09, 0xa6, 0x00, 0x24,
	0x04, 0x64, 0xf2, 0x3b, 0x6a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00,
	0x9d, 0x00, 0x6f, 0x3c, 0x88, 0x89, 0x00, 0x00, 0x02, 0x00, 0xdc, 0x00,
	0xc1, 0x00, 0x28, 0x00, 0x5c, 0x4a, 0x1e, 0x00, 0xc5, 0x37, 0x28, 0x00,
	0x5c, 0x4a, 0x8f, 0x65, 0x07, 0x00, 0x01, 0x00, 0x28, 0x00, 0xff, 0x00,
	0x80, 0x00, 0x47, 0x00, 0x64, 0x00, 0x50, 0x00, 0xd8, 0x00, 0x50, 0x00,
	0xbb, 0x00, 0x03, 0x00, 0x3c, 0x00, 0x6f, 0x00, 0x89, 0x00, 0x88, 0x00,
	0x02, 0x00, 0xfb, 0x00, 0x00, 0x00, 0x78, 0x68, 0x00, 0x0a, 0x00, 0x00,
	0x01, 0x00, 0x4e, 0x4f, 0x43, 0x16, 0xaa, 0x0f,
};

static bool probe_power;
module_param(probe_power, bool, 0444);
MODULE_PARM_DESC(probe_power,
		 "On probe, exercise the TF-A GPS power SMC and read back the conn_infra version");

static unsigned long gps_smc(unsigned long opid, unsigned long a2,
			     unsigned long a3, unsigned long *out_a1)
{
	struct arm_smccc_res res;

	arm_smccc_smc(MTK_SIP_KERNEL_GPS_CONTROL, opid, a2, a3,
		      0, 0, 0, 0, &res);
	if (out_a1)
		*out_a1 = res.a1;
	return res.a0;
}

/* USRT control needs four args (link, on, dma_mode, 1byte_mode) */
static void gps_usrt_ctrl(unsigned long link, bool on)
{
	struct arm_smccc_res res;

	/* driver default runtime cfg: DMA mode on, not 1-byte */
	arm_smccc_smc(MTK_SIP_KERNEL_GPS_CONTROL, GPS_SMC_USRT_CTRL,
		      link, on, 1, 0, 0, 0, &res);
}

/*
 * Read the GPS MCUB mailbox (DSP->AP). TF-A returns HW_OKAY (0) in a0 and the
 * d2a flag/data words in a1..a3. Returns the flag (0 if none or on error).
 */
static u32 gps_mcub_get(unsigned long link, u32 *d0, u32 *d1)
{
	struct arm_smccc_res res;

	arm_smccc_smc(MTK_SIP_KERNEL_GPS_CONTROL, GPS_SMC_GET_MCUB_INFO,
		      link, 0, 0, 0, 0, 0, &res);
	if (res.a0 != 0)		/* HW_OKAY == 0 */
		return 0;
	if (d0)
		*d0 = res.a2;
	if (d1)
		*d1 = res.a3;
	return (u32)res.a1;
}

/* GDL ring free/wrap helpers (read index pinned at 0: pure receive window) */
#define GPS_DL_RX_WORDS		(GPS_DL_RX_BUF_SIZE / 4)

/*
 * Arm the link0 D2A (DSP->AP) receive DMA. The TF-A knows the ring base, so we
 * pass a relative sub_buf_offset (the current write offset in bytes) plus the
 * wrap and max lengths in 4-byte words (DMA runs in 4-byte mode). Returns the
 * TF-A result (a0).
 */
static unsigned long gps_dma_d2a_arm(unsigned long woff_bytes)
{
	struct arm_smccc_res res;
	unsigned long len_to_wrap = (GPS_DL_RX_BUF_SIZE - woff_bytes) / 4;
	unsigned long free_words = GPS_DL_RX_WORDS - (woff_bytes / 4);

	arm_smccc_smc(MTK_SIP_KERNEL_GPS_CONTROL, GPS_SMC_SET_DMA_START,
		      GPS_DL_DMA_LINK0_D2A, woff_bytes, GPS_DL_DMA_BUS_ID0_D2A,
		      len_to_wrap, free_words, 0 /* is_1byte */, &res);
	return res.a0;
}

static unsigned long gps_dma_left_words(unsigned long channel)
{
	/* TF-A returns the left length in a0 (the gps_smc return value) */
	return gps_smc(GPS_SMC_GET_DMA_LEFT_LEN, channel, 0, NULL);
}

static void gps_dma_stop(unsigned long channel)
{
	gps_smc(GPS_SMC_SET_DMA_STOP, channel, 0, NULL);
}

/*
 * Send the captured L1 image-descriptor control frame to the DSP over the A2D
 * (AP->DSP) data link, exactly as mnld does just before the fragment loop.
 * Place the frame bytes in the TX ring EMI, then arm the A2D DMA to stream
 * them (sub_buf_offset 0, bus_id A2D, transfer length = frame size in words)
 * and wait for it to drain. Returns 0 on success.
 */
static int gps_a2d_send_init_frame(struct mtk_gnss *gnss)
{
	struct arm_smccc_res res;
	void __iomem *tx;
	unsigned long words = sizeof(gps_l1_init_frame) / 4;	/* 116/4 = 29 */
	int i;

	tx = ioremap(GPS_EMI_TX0_PHYS, GPS_DL_RX_BUF_SIZE);
	if (!tx) {
		dev_err(gnss->dev, "GPS: cannot map TX ring\n");
		return -ENOMEM;
	}
	memcpy_toio(tx, gps_l1_init_frame, sizeof(gps_l1_init_frame));
	wmb();	/* frame visible in EMI before the DMA reads it */

	gps_smc(GPS_SMC_SW_REQUEST_EMI_USAGE, 1, 0, NULL);
	arm_smccc_smc(MTK_SIP_KERNEL_GPS_CONTROL, GPS_SMC_SET_DMA_START,
		      GPS_DL_DMA_LINK0_A2D, 0 /* sub_buf_offset */,
		      GPS_DL_DMA_BUS_ID0_A2D, GPS_DL_RX_WORDS /* len_to_wrap */,
		      words /* transfer_max_len */, 0 /* is_1byte */, &res);

	/* wait for the A2D transfer to complete (int status a0 != 0) */
	for (i = 0; i < 100; i++) {
		if (gps_smc(GPS_SMC_GET_DMA_INT_STATUS, GPS_DL_DMA_LINK0_A2D,
			    0, NULL))
			break;
		udelay(50);
	}
	gps_dma_stop(GPS_DL_DMA_LINK0_A2D);
	gps_smc(GPS_SMC_SW_REQUEST_EMI_USAGE, 0, 0, NULL);
	iounmap(tx);

	dev_info(gnss->dev, "GPS: sent %zu-byte L1 init frame (a2d done after %d polls)\n",
		 sizeof(gps_l1_init_frame), i);
	return 0;
}

/*
 * Download the GNSS L1 DSP RAM image through the MVCD handshake. TF-A owns the
 * image and streams each fragment over the USRT data link; the DSP emits a
 * small progress record per fragment over the D2A ring, which the AP must keep
 * draining (claim EMI, arm the D2A DMA at the running write offset, read how
 * much arrived, stop, release EMI, clear the USRT nodata IRQ) or the USRT RX
 * FIFO backs up and the DSP stalls. This mirrors the exact per-fragment SMC
 * cycle the stock data-link driver runs. The DSP posts RAMCODE_READY over the
 * MCUB mailbox (serviced by our IRQ) once it has the full image.
 */
static void gps_mvcd_download_l1(struct mtk_gnss *gnss)
{
	struct arm_smccc_res res;
	u32 frag_num, code_size, cipher;
	unsigned long woff = 0;		/* D2A ring write offset, bytes */
	unsigned int i;

	arm_smccc_smc(MTK_SIP_KERNEL_GPS_CONTROL, GPS_SMC_MVCD_GET_DSP_BOOTUP,
		      GPS_MVCD_DSP_L1, 0, 0, 0, 0, 0, &res);
	if (res.a1 == 0) {
		dev_err(gnss->dev, "GPS MVCD: no bootup info from TF-A\n");
		return;
	}
	frag_num  = (u32)(res.a1 & 0xffff);
	code_size = (u32)((res.a1 >> 16) & 0xffff);
	cipher    = (u32)(res.a2 & 0xffff);
	dev_info(gnss->dev,
		 "GPS MVCD: L1 image frag_num %u code_size %#x cipher %#x\n",
		 frag_num, code_size, cipher);
	if (!frag_num || frag_num > 256)
		return;

	/* make sure the USRT RX path starts from empty */
	gps_smc(GPS_SMC_USRT_POLL_RX_EMPTY, GPS_DATA_LINK_ID0, 0, NULL);

	/* the DSP needs the image-descriptor frame before it accepts frags */
	gps_a2d_send_init_frame(gnss);

	/* arm the D2A receive DMA once and leave it running for the whole
	 * download so the DSP's per-fragment records stream in without gaps
	 */
	gps_smc(GPS_SMC_SW_REQUEST_EMI_USAGE, 1, 0, NULL);
	gps_dma_d2a_arm(0);

	for (i = 0; i < frag_num; i++) {
		arm_smccc_smc(MTK_SIP_KERNEL_GPS_CONTROL,
			      GPS_SMC_MVCD_SEND_DSP_FRAG, GPS_MVCD_DSP_L1, i,
			      0, 0, 0, 0, &res);
		if (res.a1 == 1) {
			dev_err(gnss->dev,
				"GPS MVCD: fragment %u rejected by TF-A\n", i);
			return;
		}
		gps_smc(GPS_SMC_USRT_CLEAR_NODATA, GPS_DATA_LINK_ID0, 0, NULL);

		if (i < 3 || i == frag_num - 1) {
			unsigned long left =
				gps_dma_left_words(GPS_DL_DMA_LINK0_D2A);
			dev_info(gnss->dev,
				 "GPS MVCD dbg frag %u: d2a left %lu (rx %lu B)\n",
				 i, left, (GPS_DL_RX_WORDS - left) * 4);
		}
		/* pace to roughly match the stock ~1ms/fragment cadence */
		usleep_range(700, 1100);
	}

	{
		unsigned long left = gps_dma_left_words(GPS_DL_DMA_LINK0_D2A);

		woff = (GPS_DL_RX_WORDS - left) * 4;
	}
	gps_dma_stop(GPS_DL_DMA_LINK0_D2A);
	gps_smc(GPS_SMC_SW_REQUEST_EMI_USAGE, 0, 0, NULL);
	dev_info(gnss->dev, "GPS MVCD: sent %u fragment(s), d2a rx %#lx B\n",
		 frag_num, woff);

	/* dump what the DSP actually wrote into the D2A RX ring */
	{
		void __iomem *rx = ioremap(GPS_EMI_PHYS_BASE + 0x57000, 64);

		if (rx) {
			u8 buf[48];

			memcpy_fromio(buf, rx, sizeof(buf));
			print_hex_dump(KERN_INFO, "gps d2a rx: ", DUMP_PREFIX_OFFSET,
				       16, 1, buf, sizeof(buf), false);
			iounmap(rx);
		}
	}
}

/*
 * Link0 MCUB mailbox IRQ: the DSP raises it when it posts a DSP->AP (d2a)
 * event. Drain all pending flags (clearing each, as a level IRQ requires),
 * and signal completion when the DSP reports its RAM code is running. This is
 * the real-time servicing the DSP's ROM->RAM-code handshake needs, which a
 * synchronous poll cannot provide.
 */
static irqreturn_t mtk_gnss_mcub_thread(int irq, void *data)
{
	struct mtk_gnss *gnss = data;
	u32 flag, d0 = 0, d1 = 0;
	bool handled = false;

	while ((flag = gps_mcub_get(GPS_DATA_LINK_ID0, &d0, &d1))) {
		handled = true;
		dev_info(gnss->dev, "MCUB d2a: flag %#06x d0 %#06x d1 %#06x\n",
			 flag, d0, d1);
		if ((flag & 0xffff0000) == 0xdead0000) {
			dev_err(gnss->dev, "GPS DSP posted a dead flag\n");
			break;
		}
		gps_smc(GPS_SMC_CLEAR_MCUB_D2A, GPS_DATA_LINK_ID0, flag, NULL);
		if (flag & GPS_MCUB_D2A_RAMCODE_READY)
			complete(&gnss->ramcode);
	}

	return handled ? IRQ_HANDLED : IRQ_NONE;
}

/*
 * After the connsys power-on, conn_infra responds briefly but then goes back
 * to sleep. Force it awake through its always-on host CSR (as the WiFi and BT
 * drivers do for their own subsystems) and wait until the core bus answers.
 */
static int mtk_gnss_conninfra_wakeup(struct mtk_gnss *gnss)
{
	void __iomem *conn = gnss->conn;
	u32 val;
	int ret, i;

	/* Request the wake until the host CSR latches it. */
	for (i = 0; i < 10; i++) {
		writel(readl(conn + CONN_INFRA_WAKEUP_GPS) | BIT(0),
		       conn + CONN_INFRA_WAKEUP_GPS);
		if (readl(conn + CONN_INFRA_WAKEUP_GPS) & BIT(0))
			break;
		udelay(100);
	}

	ret = readl_poll_timeout(conn + CONN_INFRA_CFG_VERSION, val,
				 val == CONN_INFRA_CFG_ID, 1000, 20000);
	if (ret) {
		dev_err(gnss->dev, "conn_infra cfg version %#x\n", val);
		return ret;
	}

	ret = readl_poll_timeout(conn + CONN_INFRA_CFG_PWRCTRL1, val,
				 val & CONN_INFRA_RDY_B, 1000, 20000);
	if (ret)
		dev_err(gnss->dev, "conn_infra not ready\n");

	return ret;
}

static void mtk_gnss_conninfra_release(struct mtk_gnss *gnss)
{
	writel(readl(gnss->conn + CONN_INFRA_WAKEUP_GPS) & ~BIT(0),
	       gnss->conn + CONN_INFRA_WAKEUP_GPS);
}

/*
 * Run the full GPS "common on" secure sequence (TF-A does the sensitive work;
 * we orchestrate the SMCs in the exact order the stock data-link driver uses
 * for this SoC configuration: CONNAC2 + TFA, no in-kernel conninfra driver,
 * so the A-die chip-id and A-die top-clock steps are all handled inside TF-A
 * and drop out of the host sequence). On success the whole GPS power domain
 * is up; we then tear it back down with "common off" so the probe is a clean,
 * self-contained bring-up test.
 */
static int mtk_gnss_power_probe(struct mtk_gnss *gnss)
{
	struct device *dev = gnss->dev;
	unsigned long ver = 0;
	u32 emi_20msb;
	int err;

	/* Shared supplies and A-die (refcounted against WiFi/BT). */
	err = mtk_connsys_power_on(gnss->cs, MTK_CONNSYS_GPS);
	if (err)
		return dev_err_probe(dev, err, "failed to power on connsys\n");

	err = mtk_gnss_conninfra_wakeup(gnss);
	if (err) {
		mtk_connsys_power_off(gnss->cs, MTK_CONNSYS_GPS);
		return err;
	}

	/* tell TF-A the AP requests the GPS common domain on */
	gps_smc(GPS_SMC_COMMON_ON_SET_FLAG, GPS_SMC_SET_FLAG_MAGIC, 1, NULL);

	/* let the 3T 32k clock settle, as the downstream flow does */
	udelay(200);

	/* part1: TF-A polls conn_infra and returns its version in a1 */
	if ((int)gps_smc(GPS_SMC_COMMON_ON_PART1, 0, 0, &ver) == -1) {
		dev_err(dev, "GPS common_on part1 failed\n");
		goto err_flag;
	}
	dev_info(dev, "GPS common_on part1 ok: conn_infra ver %#010lx\n", ver);

	/* peri remap: no-op on this SoC (conn2ap GPS peri not remapped) */

	/* EMI remap (part2): program the conn_infra remap with the GPS
	 * reserved-memory base (top 20 bits of the 36-bit, 64KB-aligned addr).
	 */
	emi_20msb = (u32)((GPS_EMI_PHYS_BASE & GPS_EMI_REMAP_BASE_MASK) >> 16);
	gps_smc(GPS_SMC_COMMON_ON_PART2, emi_20msb, 0, NULL);

	/* part3 */
	gps_smc(GPS_SMC_COMMON_ON_PART3, 1, 0, NULL);

	/*
	 * Tell TF-A the A-die chip id (MT6637). The stock data-link driver built
	 * with the conninfra driver does this between part3 and part4; TF-A needs
	 * it to finish the A-die clock setup the GPS DSP runs on.
	 */
	gps_smc(GPS_SMC_SET_ADIE_CHIPID, 0x6637, 0, NULL);

	/* emi_usage_init: software bookkeeping only (EMI claimed at routing) */

	/* part4 */
	if ((int)gps_smc(GPS_SMC_COMMON_ON_PART4, 0, 0, NULL) == -1) {
		dev_err(dev, "GPS common_on part4 failed\n");
		gps_smc(GPS_SMC_COMMON_ON_INNER_FAIL, 0, 0, NULL);
		goto err_flag;
	}

	/* bus_debug_flag: no-op on this SoC */

	/* part5 */
	if ((int)gps_smc(GPS_SMC_COMMON_ON_PART5, 0, 0, NULL) == -1) {
		dev_err(dev, "GPS common_on part5 failed\n");
		gps_smc(GPS_SMC_COMMON_ON_FAIL_HANDLER, 0, 0, NULL);
		goto err_flag;
	}

	dev_info(dev,
		 "GPS common_on COMPLETE: full secure power domain up (ver %#010lx)\n",
		 ver);

	/*
	 * Bring the GPS L1 DSP up: TF-A sets its power state and polls for the
	 * DSP's ACK (a0 != 0). A successful ACK means the DSP has booted from
	 * the firmware the bootloader pre-staged in the connsys EMI and is now
	 * alive - the first end-to-end sign of life from the GPS core itself.
	 * (USRT/DMA data routing is a later step; here we just prove boot.)
	 */
	/*
	 * Enable the MCUB mailbox IRQ so the DSP's DSP->AP events (RESET_DONE,
	 * then RAMCODE_READY) are serviced in real time, claim EMI, bring the
	 * L1 DSP up, enable the USRT, and wait for the DSP to report its RAM
	 * code is running. The real IRQ servicing is what the DSP's ROM->RAM
	 * handshake needs (a synchronous poll does not drive it).
	 */
	reinit_completion(&gnss->ramcode);
	if (gnss->irq_mcub >= 0)
		enable_irq(gnss->irq_mcub);

	gps_smc(GPS_SMC_SW_REQUEST_EMI_USAGE, 1, 0, NULL);	/* claim EMI */

	gps_smc(GPS_SMC_PWR_STAT_CTRL, GPS_L1_DSP_ON, 0, NULL);
	if (gps_smc(GPS_SMC_SET_DSP_ON_POLL_ACK, GPS_DATA_LINK_ID0, 0, NULL))
		dev_info(dev, "GPS L1 DSP ON: DSP booted and ACKed\n");
	else
		dev_err(dev, "GPS L1 DSP ON: no ACK (DSP did not boot)\n");

	gps_usrt_ctrl(GPS_DATA_LINK_ID0, true);

	/*
	 * Download the L1 DSP RAM image over the data link. Each fragment is
	 * paired with a D2A receive-DMA drain cycle (see gps_mvcd_download_l1)
	 * so the DSP never back-pressures. The DSP reports RAMCODE_READY over
	 * the MCUB mailbox (serviced by the IRQ above) once the image is in.
	 */
	gps_mvcd_download_l1(gnss);

	if (wait_for_completion_timeout(&gnss->ramcode, msecs_to_jiffies(3000)))
		dev_info(dev, "GPS DSP RAMCODE_READY: GNSS firmware is running!\n");
	else
		dev_warn(dev, "GPS DSP: no RAMCODE_READY within 3s\n");

	if (gnss->irq_mcub >= 0)
		disable_irq(gnss->irq_mcub);

	/* turn the USRT and the DSP back off before unwinding the common domain */
	gps_usrt_ctrl(GPS_DATA_LINK_ID0, false);
	gps_smc(GPS_SMC_PWR_STAT_CTRL, GPS_L1_DSP_OFF, 0, NULL);
	gps_smc(GPS_SMC_SET_CFG_DSP_MEM_OFF, GPS_L1_DSP_OFF, GPS_DATA_LINK_ID0, NULL);

	/* --- common off: tear the domain back down --- */
	gps_smc(GPS_SMC_COMMON_OFF_PART1, 0, 0, NULL);
	if (gps_smc(GPS_SMC_COMMON_OFF_PART2, 0, 0, NULL) != 0)
		dev_warn(dev, "GPS common_off part2 (sleep prot) not clean\n");
	gps_smc(GPS_SMC_SW_REQUEST_EMI_USAGE, 0, 0, NULL);	/* release EMI */
	gps_smc(GPS_SMC_COMMON_OFF_PART3, 0, 0, NULL);

	gps_smc(GPS_SMC_COMMON_ON_SET_FLAG, GPS_SMC_SET_FLAG_MAGIC, 0, NULL);
	mtk_gnss_conninfra_release(gnss);
	mtk_connsys_power_off(gnss->cs, MTK_CONNSYS_GPS);
	return 0;

err_flag:
	gps_smc(GPS_SMC_COMMON_ON_SET_FLAG, GPS_SMC_SET_FLAG_MAGIC, 0, NULL);
	mtk_gnss_conninfra_release(gnss);
	mtk_connsys_power_off(gnss->cs, MTK_CONNSYS_GPS);
	return -EIO;
}

static int mtk_gnss_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_gnss *gnss;
	struct resource *res;

	gnss = devm_kzalloc(dev, sizeof(*gnss), GFP_KERNEL);
	if (!gnss)
		return -ENOMEM;
	gnss->dev = dev;

	gnss->cs = devm_mtk_connsys_get(dev);
	if (IS_ERR(gnss->cs))
		return dev_err_probe(dev, PTR_ERR(gnss->cs),
				     "failed to get connsys\n");

	/*
	 * The conn_infra window overlaps the one the connsys/WiFi/BT drivers
	 * use, so map it without claiming the region (devm_ioremap, not
	 * devm_platform_ioremap_resource, which would -EBUSY).
	 */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "conn_infra");
	if (!res)
		return dev_err_probe(dev, -EINVAL, "no conn_infra reg\n");
	gnss->conn = devm_ioremap(dev, res->start, resource_size(res));
	if (!gnss->conn)
		return dev_err_probe(dev, -ENOMEM, "failed to map conn_infra\n");

	/* link0 MCUB mailbox IRQ = DT interrupt index 2 (SPI 381) */
	init_completion(&gnss->ramcode);
	gnss->irq_mcub = platform_get_irq(pdev, 2);
	if (gnss->irq_mcub >= 0) {
		int ret = devm_request_threaded_irq(dev, gnss->irq_mcub, NULL,
						    mtk_gnss_mcub_thread,
						    IRQF_ONESHOT | IRQF_NO_AUTOEN,
						    "gps_mb0", gnss);
		if (ret)
			return dev_err_probe(dev, ret, "request MCUB irq\n");
	} else {
		dev_warn(dev, "no MCUB irq in DT; RAMCODE handshake disabled\n");
	}

	platform_set_drvdata(pdev, gnss);
	dev_info(dev, "MediaTek connsys GNSS probe (milestone 1)\n");

	if (probe_power)
		return mtk_gnss_power_probe(gnss);

	dev_info(dev, "probe_power not set; skipping SMC power test\n");
	return 0;
}

static const struct of_device_id mtk_gnss_of_match[] = {
	{ .compatible = "mediatek,mt6878-gps" },
	{ }
};
MODULE_DEVICE_TABLE(of, mtk_gnss_of_match);

static struct platform_driver mtk_gnss_driver = {
	.probe = mtk_gnss_probe,
	.driver = {
		.name = "mtk-connsys-gnss",
		.of_match_table = mtk_gnss_of_match,
	},
};
module_platform_driver(mtk_gnss_driver);

MODULE_DESCRIPTION("MediaTek connsys GNSS subsystem driver");
MODULE_LICENSE("GPL");
