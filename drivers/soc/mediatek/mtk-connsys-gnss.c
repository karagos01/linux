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
#define GPS_SMC_COMMON_ON_FAIL_HANDLER	0x8

/* magic the TF-A expects in x2 of the SET_FLAG call */
#define GPS_SMC_SET_FLAG_MAGIC		0xdb9db9

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

static int mtk_gnss_power_probe(struct mtk_gnss *gnss)
{
	struct device *dev = gnss->dev;
	unsigned long ret, ver = 0;
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

	/* let the 32k clock settle, as the downstream flow does */
	udelay(200);

	/* part1 polls conn_infra and returns its version in a1 */
	ret = gps_smc(GPS_SMC_COMMON_ON_PART1, 0, 0, &ver);
	if (ret == 0)
		dev_info(dev,
			 "GPS secure power path alive: conn_infra ver %#010lx\n",
			 ver);
	else
		dev_err(dev, "GPS common_on part1 failed (smc ret %#lx)\n", ret);

	/* unwind whatever part1 powered and release the AP flag and the wake */
	gps_smc(GPS_SMC_COMMON_ON_FAIL_HANDLER, 0, 0, NULL);
	gps_smc(GPS_SMC_COMMON_ON_SET_FLAG, GPS_SMC_SET_FLAG_MAGIC, 0, NULL);
	mtk_gnss_conninfra_release(gnss);

	mtk_connsys_power_off(gnss->cs, MTK_CONNSYS_GPS);

	return ret == 0 ? 0 : -EIO;
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
