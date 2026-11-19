// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek APU (AI processing unit) host support for MT6878
 *
 * Nothing in this driver uses the APU for computing. It exists because the
 * SPM firmware cannot put the infra bus to sleep until the APU's auto-restore
 * engine acknowledges, and that only works after the APU firmware has booted
 * once and powered itself off. So: wake the APU power controller, boot the
 * firmware the bootloader placed in secure memory, answer its deep-idle
 * handshake and let it go back to sleep.
 */

#include <linux/arm-smccc.h>
#include <linux/clocksource.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/sched/clock.h>
#include <linux/unaligned.h>
#include <linux/soc/mediatek/mtk-lpm.h>
#include <linux/soc/mediatek/mtk_sip_svc.h>
#include <linux/workqueue.h>
#include <clocksource/arm_arch_timer.h>

#define MTK_SIP_APUSYS_CONTROL		MTK_SIP_SMC_CMD(0x51e)

/* TF-A APU operations */
#define APU_OP_DEVAPC_INIT_RCX		7
#define APU_OP_RV_RESET_MP		9
#define APU_OP_RV_SETUP_BOOT		10
#define APU_OP_RV_START_MP		11
#define APU_OP_PWR_RCX			26
#define APU_OP_RV_SETUP_APUMMU		30

#define APU_PWR_RCX_AFC_EN		0
#define APU_PWR_RCX_WAKEUP_RPC		1
#define APU_PWR_RCX_CG_EN		2

/* RPC (power controller) registers */
#define APU_RPC_STATUS_1		0x34
#define APU_RPC_STATUS_1_ON		BIT(13)
#define APU_RPC_INTF_PWR_RDY		0x44
#define APU_RPC_INTF_PWR_RDY_RCX	BIT(0)
#define APU_RPC_PWR_ACK			0x48

/* Mailbox registers */
#define APU_MBOX_INBOX			0x00
#define APU_MBOX_OUTBOX			0x20
#define APU_MBOX_HOST_CONFIG_ADDR	0x48
#define APU_MBOX_INBOX_IRQ		0xc0
#define APU_MBOX_OUTBOX_IRQ		0xc4
#define APU_MBOX_INBOX_IRQ_MASK		0xd0

#define APU_MBOX_HDR_SLOTS		4
#define APU_SHARE_BUFFER_SIZE		256
#define APU_LOG_BUF_SIZE		SZ_2M
#define APU_LOG_LBC_SIZE		0x80000

enum {
	APU_IPI_INIT = 0,
	APU_IPI_NS_SERVICE,
	APU_IPI_DEEP_IDLE,
	APU_IPI_TIMESYNC = 10,
};

#define DPIDLE_CMD_LOCK_IPI		0x5a00
#define DPIDLE_CMD_UNLOCK_IPI		0x5a01
#define DPIDLE_CMD_PDN_UNLOCK		0x5a02
#define DPIDLE_ACK_OK			0

struct apu_mbox_hdr {
	u32 id;
	u32 len;
	u32 serial_no;
	u32 csum;
};

struct apu_dpidle_msg {
	u32 cmd;
	u32 ack;
};

/* Configuration block the firmware reads at boot (layout fixed by the firmware) */
struct apu_ipi_config {
	u64 in_buf_da;
	u64 out_buf_da;
} __packed;

struct apu_logger_info {
	u32 iova;
	u32 iova_h;
	u32 aov_iova;
	u32 aov_iova_h;
	u32 aov_buf_sz;
	u32 lbc_sz;
} __packed;

struct apu_config {
	u32 header_magic;
	u32 header_rev;
	u32 entry_offset;
	u32 config_size;
	u32 ramdump_offset;
	u32 ramdump_type;
	u32 ramdump_module;
	u64 time_offset;
	u64 time_diff;
	u64 time_diff_cycle;
	u32 user_entry[6];
	struct apu_ipi_config ipi;		/* user 0 */
	u8 vpu_init_info[120];			/* user 1 */
	u32 chip_data[4];			/* user 2 */
	struct apu_logger_info logger;		/* user 3 */
	u8 reviser_info[260];			/* user 4 */
	u32 mvpu_preempt[20];			/* user 5 */
} __packed;

static unsigned long apu_smc(u32 op, u32 arg)
{
	struct arm_smccc_res res;

	arm_smccc_smc(MTK_SIP_APUSYS_CONTROL, op, arg, 0, 0, 0, 0, 0, &res);
	return res.a0;
}

/* ---- power controller ---- */

struct mtk_apu_top {
	struct device *dev;
	void __iomem *rpc;
	struct generic_pm_domain genpd;
};

static int mtk_apu_top_power_on(struct generic_pm_domain *genpd)
{
	struct mtk_apu_top *top = container_of(genpd, struct mtk_apu_top, genpd);
	u32 val;
	int ret;

	if (readl(top->rpc + APU_RPC_INTF_PWR_RDY) & APU_RPC_INTF_PWR_RDY_RCX)
		return 0;

	apu_smc(APU_OP_PWR_RCX, APU_PWR_RCX_AFC_EN);
	apu_smc(APU_OP_PWR_RCX, APU_PWR_RCX_WAKEUP_RPC);

	ret = readl_poll_timeout(top->rpc + APU_RPC_INTF_PWR_RDY, val,
				 val & APU_RPC_INTF_PWR_RDY_RCX, 50, 10000);
	if (!ret)
		ret = readl_poll_timeout(top->rpc + APU_RPC_STATUS_1, val,
					 val & APU_RPC_STATUS_1_ON, 50, 10000);
	if (!ret)
		ret = readl_poll_timeout(top->rpc + APU_RPC_PWR_ACK, val,
					 val & 0x3, 50, 10000);
	if (ret) {
		dev_err(top->dev, "power controller did not wake up (ready %#x, ack %#x)\n",
			readl(top->rpc + APU_RPC_INTF_PWR_RDY),
			readl(top->rpc + APU_RPC_PWR_ACK));
		return ret;
	}

	apu_smc(APU_OP_PWR_RCX, APU_PWR_RCX_CG_EN);
	return 0;
}

/*
 * Only the firmware can send the APU to sleep, which it does once the host
 * has acknowledged its power-down request: all there is to do is to wait.
 */
static int mtk_apu_top_power_off(struct generic_pm_domain *genpd)
{
	struct mtk_apu_top *top = container_of(genpd, struct mtk_apu_top, genpd);
	u32 val;

	if (readl_poll_timeout(top->rpc + APU_RPC_INTF_PWR_RDY, val,
			       !(val & APU_RPC_INTF_PWR_RDY_RCX), 50, 500000))
		dev_dbg(top->dev, "still powered (firmware not running?)\n");
	else
		dev_dbg(top->dev, "powered off\n");

	return 0;
}

static int mtk_apu_top_probe(struct platform_device *pdev)
{
	struct mtk_apu_top *top;
	int ret;

	top = devm_kzalloc(&pdev->dev, sizeof(*top), GFP_KERNEL);
	if (!top)
		return -ENOMEM;

	top->dev = &pdev->dev;
	top->rpc = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(top->rpc))
		return PTR_ERR(top->rpc);

	top->genpd.name = "apu-top";
	top->genpd.power_on = mtk_apu_top_power_on;
	top->genpd.power_off = mtk_apu_top_power_off;
	ret = pm_genpd_init(&top->genpd, NULL, true);
	if (ret)
		return ret;

	return of_genpd_add_provider_simple(pdev->dev.of_node, &top->genpd);
}

static const struct of_device_id mtk_apu_top_of_match[] = {
	{ .compatible = "mediatek,mt6878-apu-top" },
	{ }
};

static struct platform_driver mtk_apu_top_driver = {
	.probe = mtk_apu_top_probe,
	.driver = {
		.name = "mtk-apu-top",
		.of_match_table = mtk_apu_top_of_match,
		.suppress_bind_attrs = true,
	},
};

/* ---- firmware host ---- */

struct mtk_apu {
	struct device *dev;
	void __iomem *mbox;
	struct apu_config *conf;
	dma_addr_t conf_da;
	void *share;
	dma_addr_t share_da;
	void *recv_buf;
	void *send_buf;
	void *log_buf;
	dma_addr_t log_da;

	struct mutex send_lock;
	u32 tx_serial;
	u32 rx_serial;
	struct apu_mbox_hdr hdr;
	u8 rx[APU_SHARE_BUFFER_SIZE];

	bool powered;
	struct completion boot_done;
	struct work_struct boot_work;
};

static u32 mtk_apu_csum(const void *data, u32 len)
{
	const u8 *p = data;
	u32 csum = 0, res = 0, i;

	for (i = 0; i < len / 4; i++)
		csum += get_unaligned_le32(p + 4 * i);
	for (i = 0; i < len % 4; i++)
		res |= p[(len & ~3) + i] << (i * 8);

	return csum + res;
}

static int mtk_apu_ipi_send(struct mtk_apu *apu, u32 id, const void *data, u32 len)
{
	struct apu_mbox_hdr hdr;
	unsigned long timeout = jiffies + 3 * HZ;
	int ret = 0, i;

	mutex_lock(&apu->send_lock);

	/* Interrupt the APU when the last header word is written */
	writel(~BIT(APU_MBOX_HDR_SLOTS - 1) & 0xff, apu->mbox + APU_MBOX_INBOX_IRQ_MASK);

	/* The previous message must have been taken */
	while (readl(apu->mbox + APU_MBOX_INBOX_IRQ) &
	       ~readl(apu->mbox + APU_MBOX_INBOX_IRQ_MASK) & 0xff) {
		if (time_after(jiffies, timeout)) {
			dev_err(apu->dev, "inbox busy\n");
			ret = -EBUSY;
			goto unlock;
		}
		usleep_range(100, 200);
	}

	memcpy(apu->send_buf, data, len);
	hdr.id = id;
	hdr.len = len;
	hdr.csum = mtk_apu_csum(data, len);
	hdr.serial_no = apu->tx_serial++;
	for (i = 0; i < APU_MBOX_HDR_SLOTS; i++)
		writel(((u32 *)&hdr)[i], apu->mbox + APU_MBOX_INBOX + 4 * i);

unlock:
	mutex_unlock(&apu->send_lock);
	return ret;
}

static void mtk_apu_deep_idle(struct mtk_apu *apu, const struct apu_dpidle_msg *msg)
{
	struct apu_dpidle_msg ack = { .cmd = msg->cmd, .ack = DPIDLE_ACK_OK };

	switch (msg->cmd) {
	case DPIDLE_CMD_LOCK_IPI:
	case DPIDLE_CMD_UNLOCK_IPI:
		mtk_apu_ipi_send(apu, APU_IPI_DEEP_IDLE, &ack, sizeof(ack));
		break;
	case DPIDLE_CMD_PDN_UNLOCK:
		mtk_apu_ipi_send(apu, APU_IPI_DEEP_IDLE, &ack, sizeof(ack));
		/* The mailbox goes away with the power */
		apu->powered = false;
		pm_runtime_put_sync(apu->dev);
		dev_info(apu->dev, "firmware powered the APU off\n");
		mtk_lpm_infra_sleep_allow();
		break;
	default:
		dev_warn(apu->dev, "unknown deep idle command %#x\n", msg->cmd);
	}
}

static irqreturn_t mtk_apu_irq(int irq, void *priv)
{
	struct mtk_apu *apu = priv;
	u32 status;
	int i;

	if (!apu->powered)
		return IRQ_NONE;

	status = readl(apu->mbox + APU_MBOX_OUTBOX_IRQ);
	if (!status)
		return IRQ_NONE;

	for (i = 0; i < APU_MBOX_HDR_SLOTS; i++)
		((u32 *)&apu->hdr)[i] = readl(apu->mbox + APU_MBOX_OUTBOX + 4 * i);

	if (apu->hdr.len <= APU_SHARE_BUFFER_SIZE)
		memcpy(apu->rx, apu->recv_buf, apu->hdr.len);

	/* Acknowledge only after the payload has been copied */
	writel(status, apu->mbox + APU_MBOX_OUTBOX_IRQ);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t mtk_apu_irq_thread(int irq, void *priv)
{
	struct mtk_apu *apu = priv;
	struct apu_mbox_hdr *hdr = &apu->hdr;

	if (hdr->len > APU_SHARE_BUFFER_SIZE) {
		dev_warn(apu->dev, "IPI %u too long (%u)\n", hdr->id, hdr->len);
		return IRQ_HANDLED;
	}
	if (hdr->serial_no != apu->rx_serial)
		dev_warn(apu->dev, "IPI serial %u, expected %u\n", hdr->serial_no,
			 apu->rx_serial);
	apu->rx_serial = hdr->serial_no + 1;

	if (mtk_apu_csum(apu->rx, hdr->len) != hdr->csum) {
		dev_warn(apu->dev, "IPI %u checksum mismatch\n", hdr->id);
		return IRQ_HANDLED;
	}

	switch (hdr->id) {
	case APU_IPI_INIT:
		apu->rx[min_t(u32, hdr->len, APU_SHARE_BUFFER_SIZE - 1)] = 0;
		dev_info(apu->dev, "firmware up: %s\n", apu->rx);
		complete(&apu->boot_done);
		break;
	case APU_IPI_DEEP_IDLE:
		if (hdr->len >= sizeof(struct apu_dpidle_msg))
			mtk_apu_deep_idle(apu, (struct apu_dpidle_msg *)apu->rx);
		break;
	case APU_IPI_TIMESYNC: {
		u64 now = sched_clock();

		mtk_apu_ipi_send(apu, APU_IPI_TIMESYNC, &now, sizeof(now));
		break;
	}
	default:
		dev_dbg(apu->dev, "IPI %u len %u ignored\n", hdr->id, hdr->len);
	}

	return IRQ_HANDLED;
}

static void mtk_apu_config_init(struct mtk_apu *apu)
{
	struct apu_config *conf = apu->conf;
	u64 tick;

	memset(conf, 0, sizeof(*conf));
	conf->header_magic = 0xc0de0101;
	conf->header_rev = 1;
	conf->entry_offset = offsetof(struct apu_config, user_entry);
	conf->config_size = sizeof(*conf);
	conf->user_entry[0] = offsetof(struct apu_config, ipi);
	conf->user_entry[1] = offsetof(struct apu_config, vpu_init_info);
	conf->user_entry[2] = offsetof(struct apu_config, chip_data);
	conf->user_entry[3] = offsetof(struct apu_config, logger);
	conf->user_entry[4] = offsetof(struct apu_config, reviser_info);
	conf->user_entry[5] = offsetof(struct apu_config, mvpu_preempt);

	/* The firmware keeps time from the 13 MHz system counter */
	conf->time_offset = sched_clock();
	tick = arch_timer_read_counter();
	conf->time_diff = tick * 1000 / 13 - conf->time_offset;
	conf->time_diff_cycle = tick - conf->time_offset * 13 / 1000;

	conf->ipi.in_buf_da = apu->share_da + APU_SHARE_BUFFER_SIZE;
	conf->ipi.out_buf_da = apu->share_da;
	conf->logger.iova = lower_32_bits(apu->log_da);
	conf->logger.iova_h = upper_32_bits(apu->log_da);
	conf->logger.lbc_sz = APU_LOG_LBC_SIZE;
}

static void mtk_apu_boot_work(struct work_struct *work)
{
	struct mtk_apu *apu = container_of(work, struct mtk_apu, boot_work);
	struct device *dev = apu->dev;
	static const u32 ops[] = {
		APU_OP_DEVAPC_INIT_RCX, APU_OP_RV_SETUP_APUMMU, APU_OP_RV_RESET_MP,
		APU_OP_RV_SETUP_BOOT, APU_OP_RV_START_MP,
	};
	unsigned long ret;
	int i;

	if (pm_runtime_resume_and_get(dev) < 0) {
		dev_err(dev, "cannot power the APU on\n");
		return;
	}

	mtk_apu_config_init(apu);
	apu->tx_serial = 0;
	apu->rx_serial = 0;
	apu->powered = true;

	writel(~BIT(APU_MBOX_HDR_SLOTS - 1) & 0xff, apu->mbox + APU_MBOX_INBOX_IRQ_MASK);
	writel(readl(apu->mbox + APU_MBOX_OUTBOX_IRQ), apu->mbox + APU_MBOX_OUTBOX_IRQ);
	writel(lower_32_bits(apu->conf_da), apu->mbox + APU_MBOX_HOST_CONFIG_ADDR);

	for (i = 0; i < ARRAY_SIZE(ops); i++) {
		ret = apu_smc(ops[i], 0);
		if (ret)
			dev_warn(dev, "secure op %u returned %ld\n", ops[i], (long)ret);
	}

	if (!wait_for_completion_timeout(&apu->boot_done, 10 * HZ))
		dev_err(dev, "firmware did not boot (outbox irq %#x, inbox irq %#x)\n",
			readl(apu->mbox + APU_MBOX_OUTBOX_IRQ),
			readl(apu->mbox + APU_MBOX_INBOX_IRQ));
}

static int mtk_apu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_apu *apu;
	int irq, ret;

	apu = devm_kzalloc(dev, sizeof(*apu), GFP_KERNEL);
	if (!apu)
		return -ENOMEM;

	apu->dev = dev;
	mutex_init(&apu->send_lock);
	init_completion(&apu->boot_done);
	INIT_WORK(&apu->boot_work, mtk_apu_boot_work);

	apu->mbox = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(apu->mbox))
		return PTR_ERR(apu->mbox);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	/* Everything the firmware touches is addressed through the APU IOMMU */
	apu->conf = dmam_alloc_coherent(dev, PAGE_ALIGN(sizeof(*apu->conf)),
					&apu->conf_da, GFP_KERNEL);
	apu->share = dmam_alloc_coherent(dev, PAGE_SIZE, &apu->share_da, GFP_KERNEL);
	apu->log_buf = dmam_alloc_coherent(dev, APU_LOG_BUF_SIZE, &apu->log_da,
					   GFP_KERNEL);
	if (!apu->conf || !apu->share || !apu->log_buf)
		return -ENOMEM;

	memset(apu->share, 0, PAGE_SIZE);
	apu->recv_buf = apu->share;
	apu->send_buf = apu->share + APU_SHARE_BUFFER_SIZE;
	dev_info(dev, "config %pad, share %pad, log %pad\n", &apu->conf_da,
		 &apu->share_da, &apu->log_da);

	ret = devm_request_threaded_irq(dev, irq, mtk_apu_irq, mtk_apu_irq_thread,
					IRQF_ONESHOT, dev_name(dev), apu);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, apu);
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	schedule_work(&apu->boot_work);
	return 0;
}

static const struct of_device_id mtk_apu_of_match[] = {
	{ .compatible = "mediatek,mt6878-apu" },
	{ }
};

static struct platform_driver mtk_apu_driver = {
	.probe = mtk_apu_probe,
	.driver = {
		.name = "mtk-apu",
		.of_match_table = mtk_apu_of_match,
		.suppress_bind_attrs = true,
	},
};

static struct platform_driver * const mtk_apu_drivers[] = {
	&mtk_apu_top_driver,
	&mtk_apu_driver,
};

static int __init mtk_apu_init(void)
{
	return platform_register_drivers(mtk_apu_drivers, ARRAY_SIZE(mtk_apu_drivers));
}
module_init(mtk_apu_init);

MODULE_DESCRIPTION("MediaTek MT6878 APU host support");
MODULE_LICENSE("GPL");
