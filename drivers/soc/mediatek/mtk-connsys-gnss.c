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

#include <linux/delay.h>
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
	gps_smc(GPS_SMC_PWR_STAT_CTRL, GPS_L1_DSP_ON, 0, NULL);
	if (gps_smc(GPS_SMC_SET_DSP_ON_POLL_ACK, GPS_DATA_LINK_ID0, 0, NULL))
		dev_info(dev, "GPS L1 DSP ON: DSP booted and ACKed\n");
	else
		dev_err(dev, "GPS L1 DSP ON: no ACK (DSP did not boot)\n");

	/*
	 * Enable the USRT so the DSP finishes its boot handshake, then poll the
	 * MCUB mailbox for the DSP->AP events: RESET_DONE (ROM up) and
	 * RAMCODE_READY (firmware running). Seeing them is the first bidirectional
	 * AP<->DSP communication over the data link.
	 */
	gps_smc(GPS_SMC_SW_REQUEST_EMI_USAGE, 1, 0, NULL);	/* claim EMI */
	gps_usrt_ctrl(GPS_DATA_LINK_ID0, true);
	{
		bool saw_reset = false, saw_ram = false;
		u32 flag, d0 = 0, d1 = 0;
		int i;

		for (i = 0; i < 1500; i++) {
			flag = gps_mcub_get(GPS_DATA_LINK_ID0, &d0, &d1);
			if (flag) {
				dev_info(dev,
					 "GPS DSP MCUB d2a: flag %#06x d0 %#06x d1 %#06x\n",
					 flag, d0, d1);
				if ((flag & 0xffff0000) == 0xdead0000) {
					dev_err(dev, "GPS DSP posted a dead flag\n");
					break;
				}
				gps_smc(GPS_SMC_CLEAR_MCUB_D2A,
					GPS_DATA_LINK_ID0, flag, NULL);
				if (flag & GPS_MCUB_D2A_RESET_DONE)
					saw_reset = true;
				if (flag & GPS_MCUB_D2A_RAMCODE_READY)
					saw_ram = true;
				if (saw_reset && saw_ram)
					break;
			}
			usleep_range(1000, 1100);
		}
		if (saw_reset || saw_ram)
			dev_info(dev,
				 "GPS DSP ALIVE via MCUB (reset_done=%d ramcode_ready=%d)\n",
				 saw_reset, saw_ram);
		else
			dev_warn(dev, "GPS DSP posted no MCUB boot event in 1.5s\n");
	}

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
