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
 * Pins are laid out within a mailbox by the firmware's send-then-recv order;
 * for this SoC every pin's slot byte offset works out to pin_index * 8. All
 * SCP<->sensorhub traffic uses mailbox 1. Values verified on-device (the ready
 * + logger recv bits latch as 0x02400000).
 */
#define SCP_SENSOR_MBOX			1
#define SCP_PIN_OFF(bit)		((bit) * 8)

/* SCP->AP recv pins on mailbox 1 */
#define SCP_READY_BIT			25	/* IPI_IN_SCP_READY_1, payload = image size */
#define SCP_SENSOR_NOTIFY_BIT		27	/* IPI_IN_SENSOR_NOTIFY */

/* AP->SCP send pins on mailbox 1 */
#define SCP_SENSOR_NOTIFY_OUT_BIT	13	/* IPI_OUT_SENSOR_NOTIFY */

/* sensor_comm notify commands */
#define SCP_SENSOR_NOTIFY_READY		4

#define SCP_READY_TIMEOUT_MS		10000

/*
 * sensor_comm notify packet (28 bytes): a small header plus five data words.
 * The 5-byte header is covered by a CRC-8 the SCP checks, so the layout and
 * checksum must match the firmware exactly.
 */
struct scp_sensor_notify {
	u8 sequence;
	u8 sensor_type;
	u8 command;
	u8 length;
	u8 crc8;
	u8 pad[3];
	s32 value[5];
} __packed;

/* CRC-8, polynomial 0x8c (reflected), init 0 — the SCP's tiny_crc8 */
static u8 scp_crc8(const u8 *p, size_t len)
{
	static const u8 tbl[32] = {
		0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,
		0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
		0x00, 0x9d, 0x23, 0xbe, 0x46, 0xdb, 0x65, 0xf8,
		0x8c, 0x11, 0xaf, 0x32, 0xca, 0x57, 0xe9, 0x74,
	};
	u8 crc = 0;

	while (len--) {
		crc ^= *p++;
		crc = tbl[crc & 0x0f] ^ tbl[16 + ((crc >> 4) & 0x0f)];
	}
	return crc;
}

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
	atomic_t sensor_ready;
	u32 image_size;
	u8 seq;
	struct work_struct sensor_init;
	struct delayed_work report;
};

/* Write a message into a send pin's slot and ring the SCP's doorbell. */
static void scp_mbox_send(struct scp_mbox *mb, unsigned int bit,
			  const void *data, size_t len)
{
	const u32 *w = data;
	size_t i;

	for (i = 0; i < DIV_ROUND_UP(len, 4); i++)
		writel(w[i], mb->base + SCP_PIN_OFF(bit) + i * 4);
	/* flush the slot before the trigger the SCP will react to */
	wmb();
	writel(BIT(bit), mb->base + SCP_MBOX_SET_IRQ);
}

static void scp_send_sensor_ready(struct mt6878_scp *scp)
{
	struct scp_sensor_notify n = {
		.sequence = ++scp->seq,
		.sensor_type = 0,
		.command = SCP_SENSOR_NOTIFY_READY,
		.length = 0,
	};

	n.crc8 = scp_crc8((u8 *)&n, 4);
	scp_mbox_send(&scp->mbox[SCP_SENSOR_MBOX], SCP_SENSOR_NOTIFY_OUT_BIT,
		      &n, sizeof(n));
	dev_info(scp->dev, "sent sensorhub host-ready (seq %u)\n", n.sequence);
}

static void scp_sensor_init_work(struct work_struct *work)
{
	struct mt6878_scp *scp =
		container_of(work, struct mt6878_scp, sensor_init);

	scp_send_sensor_ready(scp);
}

static irqreturn_t scp_mbox_isr(int irq, void *data)
{
	struct scp_mbox *mb = data;
	struct mt6878_scp *scp = mb->scp;
	u32 status = readl(mb->base + SCP_MBOX_CLR_IRQ);

	if (!status)
		return IRQ_NONE;

	if (mb->id == SCP_SENSOR_MBOX && (status & BIT(SCP_READY_BIT))) {
		scp->image_size = readl(mb->base + SCP_PIN_OFF(SCP_READY_BIT));
		if (atomic_xchg(&scp->ready, 1) == 0) {
			dev_info(scp->dev, "SCP ready: image size %#x\n",
				 scp->image_size);
			/* the SCP is up; greet its sensor hub from a thread */
			schedule_work(&scp->sensor_init);
		}
	}

	if (mb->id == SCP_SENSOR_MBOX && (status & BIT(SCP_SENSOR_NOTIFY_BIT))) {
		struct scp_sensor_notify n;
		u32 *w = (u32 *)&n;
		int i;

		for (i = 0; i < sizeof(n) / 4; i++)
			w[i] = readl(mb->base + SCP_PIN_OFF(SCP_SENSOR_NOTIFY_BIT) + i * 4);
		dev_info(scp->dev,
			 "sensor NOTIFY: cmd %u type %u len %u val0 %d\n",
			 n.command, n.sensor_type, n.length, n.value[0]);
		if (n.command == SCP_SENSOR_NOTIFY_READY &&
		    atomic_xchg(&scp->sensor_ready, 1) == 0)
			dev_info(scp->dev, "sensor hub ready\n");
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

	if (!atomic_read(&scp->ready))
		dev_warn(scp->dev, "SCP did not signal ready within %d ms\n",
			 SCP_READY_TIMEOUT_MS);
	else if (atomic_read(&scp->sensor_ready))
		dev_info(scp->dev, "SCP ready (image %#x), sensor hub ready\n",
			 scp->image_size);
	else
		dev_info(scp->dev,
			 "SCP ready (image %#x), sensor hub did not answer\n",
			 scp->image_size);
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
	atomic_set(&scp->sensor_ready, 0);
	INIT_WORK(&scp->sensor_init, scp_sensor_init_work);
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

	cancel_work_sync(&scp->sensor_init);
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
