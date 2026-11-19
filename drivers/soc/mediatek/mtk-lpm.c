// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek low power manager for the SPM resource constraints.
 *
 * The SPM firmware puts the system resources (DRAM, EMI, the 1.8 V RF rail,
 * the infra bus, the 26 MHz clock, the core rail) to sleep once all CPUs are
 * off and nobody requests them. TF-A owns the SPM, but leaves its resource
 * constraints disabled until the kernel has described its wake-up interrupts
 * and validated the setup. This driver does that handshake and owns the
 * software resource requests.
 */

#include <linux/arm-smccc.h>
#include <linux/cpuidle.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeirq.h>
#include <linux/soc/mediatek/mtk-lpm.h>
#include <linux/soc/mediatek/mtk_sip_svc.h>
#include <linux/spinlock.h>

#define MTK_SIP_LPM_CONTROL		MTK_SIP_SMC_CMD(0x507)

#define LPM_ID(user, uid)		(0xda000000 | ((user) << 16) | (uid))
#define LPM_USER_CPU_PM			0
#define LPM_USER_SPM_DBG		1
#define LPM_USER_SPM			2
#define LPM_USER_CPU_PM_LP		3

#define LPM_ACT_SET			BIT(0)
#define LPM_ACT_CLR			BIT(1)
#define LPM_ACT_PUSH			BIT(3)
#define LPM_ACT_SUBMIT			BIT(5)

/* LPM_USER_CPU_PM */
#define CPU_PM_IRQ_REMAIN_ALLOC		2
#define CPU_PM_IRQ_REMAIN_ADD		3
#define CPU_PM_IRQ_REMAIN_SUBMIT	4
#define CPU_PM_VALIDATE			5
/* LPM_USER_CPU_PM_LP */
#define CPU_PM_LP_IRQ_REMAIN_NUM	1
#define CPU_PM_LP_IRQ_REMAIN_LIST	2
#define CPU_PM_LP_IRQ_REMAIN_IRQ	3
#define CPU_PM_LP_IRQ_REMAIN_CAT	4
#define CPU_PM_LP_IRQ_REMAIN_SRC	5
/* LPM_USER_SPM_DBG */
#define SPM_DBG_IDLE_PWR_CTRL		0
#define SPM_DBG_RESOURCE_CTRL		17
#define SPM_DBG_RC_SWITCH		18
/* LPM_USER_SPM */
#define SPM_IRQ_PENDING			5

/* Fields of the idle power control block (SPM_DBG_IDLE_PWR_CTRL) */
#define PW_REG_SPM_APSRC_REQ		17
#define PW_REG_SPM_INFRA_REQ		22

#define LPM_NR_RESOURCE_USERS		8
#define LPM_NR_CONSTRAINTS		3

struct mtk_lpm_irq_remain {
	u32 spi;	/* GIC SPI number */
	u32 cat;	/* wake-up category */
	u32 src;	/* SPM wake-up source bit */
};

struct mtk_lpm_data {
	const struct mtk_lpm_irq_remain *remain;
	unsigned int num_remain;
};

static DEFINE_SPINLOCK(mtk_lpm_lock);
/*
 * One reference is held on behalf of the drivers that have not probed yet:
 * a bus master that is already running (set up by the bootloader or probing
 * right now) must not lose the memory path before its driver can ask for it.
 */
static unsigned int mtk_lpm_bus_users = 1;
static bool mtk_lpm_ready;
static bool mtk_lpm_infra_blocked = true;

static unsigned long mtk_lpm_smc(u32 id, u32 act, u32 v1, u32 v2)
{
	struct arm_smccc_res res;

	arm_smccc_smc(MTK_SIP_LPM_CONTROL, id, act, v1, v2, 0, 0, 0, &res);
	return res.a0;
}

static void mtk_lpm_set_req(u32 field, bool on)
{
	mtk_lpm_smc(LPM_ID(LPM_USER_SPM_DBG, SPM_DBG_IDLE_PWR_CTRL), LPM_ACT_SET,
		    field, on);
}

/* Called with mtk_lpm_lock held */
static void mtk_lpm_update_reqs(void)
{
	mtk_lpm_set_req(PW_REG_SPM_APSRC_REQ, mtk_lpm_bus_users);
	mtk_lpm_set_req(PW_REG_SPM_INFRA_REQ,
			mtk_lpm_bus_users || mtk_lpm_infra_blocked);
}

void mtk_lpm_bus_hold(void)
{
	unsigned long flags;

	spin_lock_irqsave(&mtk_lpm_lock, flags);
	if (!mtk_lpm_bus_users++ && mtk_lpm_ready)
		mtk_lpm_update_reqs();
	spin_unlock_irqrestore(&mtk_lpm_lock, flags);
}
EXPORT_SYMBOL_GPL(mtk_lpm_bus_hold);

void mtk_lpm_bus_release(void)
{
	unsigned long flags;

	spin_lock_irqsave(&mtk_lpm_lock, flags);
	if (!WARN_ON(!mtk_lpm_bus_users) && !--mtk_lpm_bus_users && mtk_lpm_ready)
		mtk_lpm_update_reqs();
	spin_unlock_irqrestore(&mtk_lpm_lock, flags);
}
EXPORT_SYMBOL_GPL(mtk_lpm_bus_release);

void mtk_lpm_infra_sleep_allow(void)
{
	unsigned long flags;

	spin_lock_irqsave(&mtk_lpm_lock, flags);
	mtk_lpm_infra_blocked = false;
	if (mtk_lpm_ready)
		mtk_lpm_update_reqs();
	spin_unlock_irqrestore(&mtk_lpm_lock, flags);
}
EXPORT_SYMBOL_GPL(mtk_lpm_infra_sleep_allow);

static irqreturn_t mtk_lpm_spm_irq(int irq, void *data)
{
	/* TF-A acknowledges the SPM on the way out of idle; nothing to do. */
	return IRQ_HANDLED;
}

static int mtk_lpm_probe(struct platform_device *pdev)
{
	const struct mtk_lpm_data *data = device_get_match_data(&pdev->dev);
	struct device *dev = &pdev->dev;
	struct irq_data *irqd;
	unsigned long flags;
	int i, irq, ret;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	/* Interrupts that must stay able to wake the CPUs while MCUSYS is off */
	mtk_lpm_smc(LPM_ID(LPM_USER_CPU_PM, CPU_PM_IRQ_REMAIN_ALLOC),
		    data->num_remain, 0, 0);
	mtk_lpm_smc(LPM_ID(LPM_USER_CPU_PM_LP, CPU_PM_LP_IRQ_REMAIN_NUM),
		    LPM_ACT_SET, data->num_remain, 0);
	for (i = data->num_remain - 1; i >= 0; i--) {
		const struct mtk_lpm_irq_remain *r = &data->remain[i];
		u32 hwirq = r->spi + 32;

		mtk_lpm_smc(LPM_ID(LPM_USER_CPU_PM, CPU_PM_IRQ_REMAIN_ADD),
			    hwirq, r->cat, r->src);
		mtk_lpm_smc(LPM_ID(LPM_USER_CPU_PM_LP, CPU_PM_LP_IRQ_REMAIN_IRQ),
			    LPM_ACT_SET, hwirq, 0);
		mtk_lpm_smc(LPM_ID(LPM_USER_CPU_PM_LP, CPU_PM_LP_IRQ_REMAIN_CAT),
			    LPM_ACT_SET, r->cat, 0);
		mtk_lpm_smc(LPM_ID(LPM_USER_CPU_PM_LP, CPU_PM_LP_IRQ_REMAIN_SRC),
			    LPM_ACT_SET, r->src, 0);
		mtk_lpm_smc(LPM_ID(LPM_USER_CPU_PM_LP, CPU_PM_LP_IRQ_REMAIN_LIST),
			    LPM_ACT_PUSH, 0, 0);
	}
	mtk_lpm_smc(LPM_ID(LPM_USER_CPU_PM, CPU_PM_IRQ_REMAIN_SUBMIT), 0, 0, 0);
	mtk_lpm_smc(LPM_ID(LPM_USER_CPU_PM_LP, CPU_PM_LP_IRQ_REMAIN_LIST),
		    LPM_ACT_SUBMIT, 0, 0);

	/* No software user holds a resource through the resource controller */
	for (i = 0; i < LPM_NR_RESOURCE_USERS; i++)
		mtk_lpm_smc(LPM_ID(LPM_USER_SPM_DBG, SPM_DBG_RESOURCE_CTRL),
			    LPM_ACT_SET, i, 0);

	ret = devm_request_irq(dev, irq, mtk_lpm_spm_irq, 0, dev_name(dev), NULL);
	if (ret)
		return dev_err_probe(dev, ret, "cannot request the SPM interrupt\n");
	irqd = irq_get_irq_data(irq);
	mtk_lpm_smc(LPM_ID(LPM_USER_SPM, SPM_IRQ_PENDING), LPM_ACT_SET,
		    irqd_to_hwirq(irqd), 0);
	device_init_wakeup(dev, true);
	dev_pm_set_wake_irq(dev, irq);

	/*
	 * Enabling the constraints resets the software requests, and from then
	 * on the SPM acts on them: keep the CPUs out of the deep idle states
	 * until the requests are in place.
	 */
	cpuidle_pause_and_lock();

	for (i = LPM_NR_CONSTRAINTS - 1; i >= 0; i--)
		mtk_lpm_smc(LPM_ID(LPM_USER_SPM_DBG, SPM_DBG_RC_SWITCH),
			    LPM_ACT_SET, i, 0);

	/*
	 * A bus user needs the infra bus as well as the memory path. The
	 * infra bus is also kept up until the APU driver reports that the SPM
	 * firmware can put it to sleep: before the APU has gone through a
	 * power cycle its restore engine never acknowledges, the firmware
	 * waits for it forever and the watchdog resets the SoC.
	 */
	spin_lock_irqsave(&mtk_lpm_lock, flags);
	mtk_lpm_update_reqs();
	mtk_lpm_ready = true;
	spin_unlock_irqrestore(&mtk_lpm_lock, flags);

	mtk_lpm_smc(LPM_ID(LPM_USER_CPU_PM, CPU_PM_VALIDATE), LPM_ACT_SET, 0, 0);
	cpuidle_resume_and_unlock();

	return 0;
}

/* GIC SPIs the MT6878 vendor tree keeps as wake-up interrupts */
static const struct mtk_lpm_irq_remain mt6878_irq_remain[] = {
	{ 0x06e, 0, 4 },					/* keypad */
	{ 0x1cb, 0, 0 }, { 0x1cc, 0, 0 }, { 0x1cd, 0, 0 },	/* spi0..7 */
	{ 0x1ce, 0, 0 }, { 0x1cf, 0, 0 }, { 0x1d0, 0, 0 },
	{ 0x1d1, 0, 0 }, { 0x1d2, 0, 0 },
	{ 0x148, 0, 0 }, { 0x149, 0, 0 },			/* dsi0/1 */
	{ 0x236, 0, 0 }, { 0x24a, 0, 0 }, { 0x24b, 0, 0 },	/* apu */
	{ 0x254, 0, 0 }, { 0x23f, 0, 0 },
};

static const struct mtk_lpm_data mt6878_lpm_data = {
	.remain = mt6878_irq_remain,
	.num_remain = ARRAY_SIZE(mt6878_irq_remain),
};

static const struct of_device_id mtk_lpm_of_match[] = {
	{ .compatible = "mediatek,mt6878-lpm", .data = &mt6878_lpm_data },
	{ }
};
MODULE_DEVICE_TABLE(of, mtk_lpm_of_match);

static struct platform_driver mtk_lpm_driver = {
	.probe = mtk_lpm_probe,
	.driver = {
		.name = "mtk-lpm",
		.of_match_table = mtk_lpm_of_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(mtk_lpm_driver);

static int __init mtk_lpm_boot_done(void)
{
	mtk_lpm_bus_release();
	return 0;
}
late_initcall_sync(mtk_lpm_boot_done);
