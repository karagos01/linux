// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek MT6878 SCP (Andes RV55 "tinysys") bring-up.
 *
 * The SCP is a RISC-V companion processor whose firmware is an MTK tinysys
 * container (not ELF), staged into reserved DRAM by the bootloader, which also
 * writes a region-info descriptor at SRAM+0x4. Unlike the Cortex-M SCPs handled
 * by mtk_scp.c, the reset/boot registers here are secure-only: the core is
 * released through the resident TF-A via an SiP SMC.
 *
 * This increment boots the RV55 and receives its "ready" IPI over the hardware
 * mailbox, decoding the reported image size. The full tinysys IPI/rpmsg layer
 * and the sensor hub on top are not implemented yet; inbound mailbox slots are
 * acknowledged but not otherwise consumed.
 */
#include <linux/arm-smccc.h>
#include <linux/atomic.h>
#include <linux/bits.h>
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

/* mailbox window: set-irq (AP->SCP trigger) and clr-irq (SCP->AP status/ack) */
#define SCP_MBOX_SET_IRQ		0x100
#define SCP_MBOX_CLR_IRQ		0x10c

#define SCP_NR_MBOX			5

/*
 * The "ready" IPI (IPI_IN_SCP_READY_1, tinysys channel 22) lands on mailbox 1.
 * Its slot offset and interrupt bit are derived from this firmware's pin
 * layout (verified on-device: the ready + logger bits latch as 0x02400000).
 * The one-word payload is the SCP image size.
 */
#define SCP_READY_MBOX			1
#define SCP_READY_BIT			25
#define SCP_READY_SLOT			0xc8

#define SCP_READY_TIMEOUT_MS		10000

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

struct mt6878_scp;

struct scp_mbox {
	struct mt6878_scp *scp;
	void __iomem *base;
	int id;
};

struct mt6878_scp {
	struct device *dev;
	void __iomem *sram;
	struct scp_mbox mbox[SCP_NR_MBOX];
	atomic_t ready;
	u32 image_size;
	struct delayed_work report;
};

static irqreturn_t scp_mbox_isr(int irq, void *data)
{
	struct scp_mbox *mb = data;
	struct mt6878_scp *scp = mb->scp;
	u32 status = readl(mb->base + SCP_MBOX_CLR_IRQ);

	if (!status)
		return IRQ_NONE;

	if (mb->id == SCP_READY_MBOX && (status & BIT(SCP_READY_BIT))) {
		scp->image_size = readl(mb->base + SCP_READY_SLOT);
		if (atomic_xchg(&scp->ready, 1) == 0)
			dev_info(scp->dev,
				 "SCP ready: image size %#x (mbox%d bit %d)\n",
				 scp->image_size, mb->id, SCP_READY_BIT);
	} else {
		dev_dbg(scp->dev, "mbox%d recv status %#x\n", mb->id, status);
	}

	/*
	 * The clr-irq register is level-reflecting write-1-to-clear: ack every
	 * pending bit, including pins we do not consume yet, or the source stays
	 * asserted and storms.
	 */
	writel(status, mb->base + SCP_MBOX_CLR_IRQ);
	return IRQ_HANDLED;
}

static irqreturn_t scp_ipc_isr(int irq, void *data)
{
	struct mt6878_scp *scp = data;

	/* IPC doorbells are not wired to a decoder yet; mask on first fire. */
	disable_irq_nosync(irq);
	dev_info(scp->dev, "SCP IPC doorbell IRQ %d (masked)\n", irq);
	return IRQ_HANDLED;
}

static void mt6878_scp_report(struct work_struct *work)
{
	struct mt6878_scp *scp =
		container_of(to_delayed_work(work), struct mt6878_scp, report);

	if (atomic_read(&scp->ready))
		dev_info(scp->dev, "SCP ready (image size %#x)\n",
			 scp->image_size);
	else
		dev_warn(scp->dev,
			 "SCP did not signal ready within %d ms\n",
			 SCP_READY_TIMEOUT_MS);
}

static int mt6878_scp_request_mbox(struct platform_device *pdev,
				   struct mt6878_scp *scp)
{
	struct device *dev = &pdev->dev;
	char name[8];
	int i, irq, ret;

	for (i = 0; i < SCP_NR_MBOX; i++) {
		scp->mbox[i].scp = scp;
		scp->mbox[i].id = i;

		snprintf(name, sizeof(name), "mbox%d", i);
		scp->mbox[i].base =
			devm_platform_ioremap_resource_byname(pdev, name);
		if (IS_ERR(scp->mbox[i].base))
			return dev_err_probe(dev, PTR_ERR(scp->mbox[i].base),
					     "cannot map %s\n", name);

		irq = platform_get_irq_byname(pdev, name);
		if (irq < 0)
			return irq;
		ret = devm_request_irq(dev, irq, scp_mbox_isr, 0,
				       dev_name(dev), &scp->mbox[i]);
		if (ret)
			return dev_err_probe(dev, ret, "request %s irq\n", name);
	}

	/* IPC doorbells (optional, masked on first fire) */
	for (i = 0; i < 2; i++) {
		snprintf(name, sizeof(name), "ipc%d", i);
		irq = platform_get_irq_byname(pdev, name);
		if (irq < 0)
			continue;
		ret = devm_request_irq(dev, irq, scp_ipc_isr, 0,
				       dev_name(dev), scp);
		if (ret)
			dev_warn(dev, "cannot request %s irq: %d\n", name, ret);
	}

	return 0;
}

static int mt6878_scp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct scp_region_info ri;
	struct arm_smccc_res res;
	struct mt6878_scp *scp;
	int ret;

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

	ret = mt6878_scp_request_mbox(pdev, scp);
	if (ret)
		return ret;

	/*
	 * Release the RV55 from reset. Firmware and the region-info descriptor
	 * were placed by the bootloader; the resident TF-A programs the
	 * DRAM-reserve GPRs (these are secure-only) and deasserts core0. The SCP
	 * then raises its ready IPI on the mailbox.
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
