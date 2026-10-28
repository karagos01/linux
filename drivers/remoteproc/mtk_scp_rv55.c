// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek MT6878 SCP (Andes RV55 "tinysys") bring-up.
 *
 * The SCP is a RISC-V companion processor whose firmware is an MTK tinysys
 * container (not ELF), staged into reserved DRAM by the bootloader, which also
 * writes a region-info descriptor at SRAM+0x4. Unlike the Cortex-M SCPs handled
 * by mtk_scp.c, the reset/boot registers here are secure-only: the core is
 * released through the resident TF-A via an SiP SMC. This increment boots the
 * RV55 and confirms liveness by catching its first mailbox interrupt; the IPI
 * transport and rpmsg layer are not implemented yet.
 */
#include <linux/arm-smccc.h>
#include <linux/atomic.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>

/* region-info descriptor the bootloader leaves at SCP SRAM + 0x4 */
#define SCP_REGION_INFO_OFFSET		0x4

/* SiP service in the resident TF-A: MTK_SIP_TINYSYS_SCP_CONTROL (fn 0x528) */
#define MTK_SIP_TINYSYS_SCP_CONTROL	0xC2000528UL
#define SCP_KERNEL_OP_RESET_RELEASE	3

#define SCP_READY_TIMEOUT_MS		10000
#define SCP_MAX_IRQS			8

struct scp_region_info {
	u32 ap_loader_start;
	u32 ap_loader_size;
	u32 ap_firmware_start;
	u32 ap_firmware_size;
	u32 ap_dram_start;
	u32 ap_dram_size;
	u32 ap_dram_backup_start;
	u32 struct_size;
};

struct mt6878_scp {
	struct device *dev;
	void __iomem *sram;
	atomic_t ready;
	struct delayed_work report;
};

static irqreturn_t mt6878_scp_isr(int irq, void *data)
{
	struct mt6878_scp *scp = data;

	/*
	 * The mailbox line stays asserted until a full IPI handler drains it,
	 * which this increment does not implement. Mask on first fire so the
	 * level-triggered source cannot storm.
	 */
	disable_irq_nosync(irq);
	if (atomic_xchg(&scp->ready, 1) == 0)
		dev_info(scp->dev,
			 "SCP alive: first mailbox IRQ %d fired (boot confirmed)\n",
			 irq);
	return IRQ_HANDLED;
}

static void mt6878_scp_report(struct work_struct *work)
{
	struct mt6878_scp *scp =
		container_of(to_delayed_work(work), struct mt6878_scp, report);

	if (atomic_read(&scp->ready))
		dev_info(scp->dev, "SCP signalled ready\n");
	else
		dev_warn(scp->dev,
			 "SCP did not signal ready within %d ms (no mailbox IRQ)\n",
			 SCP_READY_TIMEOUT_MS);
}

static int mt6878_scp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct scp_region_info ri;
	struct arm_smccc_res res;
	struct mt6878_scp *scp;
	int i, n, ret;

	scp = devm_kzalloc(dev, sizeof(*scp), GFP_KERNEL);
	if (!scp)
		return -ENOMEM;
	scp->dev = dev;
	atomic_set(&scp->ready, 0);
	INIT_DELAYED_WORK(&scp->report, mt6878_scp_report);
	platform_set_drvdata(pdev, scp);

	scp->sram = devm_platform_ioremap_resource_byname(pdev, "sram");
	if (IS_ERR(scp->sram))
		return dev_err_probe(dev, PTR_ERR(scp->sram),
				     "cannot map SCP SRAM\n");

	memcpy_fromio(&ri, scp->sram + SCP_REGION_INFO_OFFSET, sizeof(ri));
	dev_info(dev,
		 "region_info: loader=%#x/%#x fw=%#x/%#x dram=%#x/%#x ssz=%#x\n",
		 ri.ap_loader_start, ri.ap_loader_size, ri.ap_firmware_start,
		 ri.ap_firmware_size, ri.ap_dram_start, ri.ap_dram_size,
		 ri.struct_size);
	if (ri.struct_size < 0x20 || !ri.ap_firmware_start) {
		dev_err(dev,
			"region_info invalid; bootloader did not stage SCP firmware\n");
		return -ENODEV;
	}

	/* The ready notification arrives on one of the SCP mailbox lines. */
	n = platform_irq_count(pdev);
	if (n < 0)
		return dev_err_probe(dev, n, "cannot count interrupts\n");
	for (i = 0; i < n && i < SCP_MAX_IRQS; i++) {
		int irq = platform_get_irq(pdev, i);

		if (irq < 0)
			return irq;
		ret = devm_request_irq(dev, irq, mt6878_scp_isr, 0,
				       dev_name(dev), scp);
		if (ret)
			return dev_err_probe(dev, ret,
					     "cannot request IRQ %d\n", irq);
	}
	dev_info(dev, "requested %d mailbox IRQ(s)\n", i);

	/*
	 * Release the RV55 from reset. Firmware and the region-info descriptor
	 * were placed by the bootloader; the resident TF-A programs the
	 * DRAM-reserve GPRs (these are secure-only) and deasserts core0.
	 */
	dev_info(dev, "releasing SCP (SMC %#lx op %d)\n",
		 MTK_SIP_TINYSYS_SCP_CONTROL, SCP_KERNEL_OP_RESET_RELEASE);
	arm_smccc_smc(MTK_SIP_TINYSYS_SCP_CONTROL, SCP_KERNEL_OP_RESET_RELEASE,
		      0, 0, 0, 0, 0, 0, &res);
	dev_info(dev, "reset-release SMC returned %#lx\n", res.a0);
	if (res.a0)
		dev_warn(dev, "reset-release SMC reported error %#lx\n", res.a0);

	schedule_delayed_work(&scp->report,
			      msecs_to_jiffies(SCP_READY_TIMEOUT_MS));
	return 0;
}

static void mt6878_scp_remove(struct platform_device *pdev)
{
	struct mt6878_scp *scp = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&scp->report);
}

static const struct of_device_id mt6878_scp_of_match[] = {
	{ .compatible = "mediatek,mt6878-scp" },
	{}
};
MODULE_DEVICE_TABLE(of, mt6878_scp_of_match);

static struct platform_driver mt6878_scp_driver = {
	.probe = mt6878_scp_probe,
	.remove = mt6878_scp_remove,
	.driver = {
		.name = "mt6878-scp-rv55",
		.of_match_table = mt6878_scp_of_match,
	},
};
module_platform_driver(mt6878_scp_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek MT6878 SCP (RV55 tinysys) bring-up");
