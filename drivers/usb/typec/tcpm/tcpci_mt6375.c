// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6375 Type-C Port Controller Interface driver.
 *
 * The MT6375 Type-C block is a Richtek TCPCI part (vendor ID 0x29CF, the same
 * family as the MT6370) living on slave 0x4e of the MT6375 sub-PMIC.  It is
 * reached through the parent MFD's banked regmap, where the TCPC bank maps to
 * the plain TCPCI register offsets, so this driver is a thin platform glue over
 * the generic TCPCI core, mirroring tcpci_mt6370.c.
 */

#include <linux/bits.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeirq.h>
#include <linux/regmap.h>
#include <linux/usb/tcpci.h>
#include <linux/usb/tcpm.h>

/* Vendor registers in the TCPC bank (offsets within the TCPCI address space) */
#define MT6375_REG_PHYCTRL1	0x80
#define MT6375_REG_PHYCTRL2	0x81
#define MT6375_REG_PHYCTRL3	0x82
#define MT6375_REG_PHYCTRL7	0x86
#define MT6375_REG_SYSCTRL1	0x8F
#define MT6375_REG_PHYCTRL9	0xAC
#define MT6375_REG_TCPCCTRL1	0xB1
#define MT6375_REG_TCPCCTRL2	0xB2
#define MT6375_REG_TCPCCTRL3	0xB3
#define MT6375_REG_VCONCTRL3	0x8C
#define MT6375_REG_LPWRCTRL5	0xBD
#define MT6375_REG_WATCHDOGCTRL	0xBE
#define MT6375_REG_HILOCTRL9	0xC8
#define MT6375_REG_SHIELDCTRL1	0xCA

#define MT6375_VBUSDISCHG_OPT_MASK	BIT(6)	/* WATCHDOGCTRL */
#define MT6375_OPEN40MS_EN_MASK		BIT(4)	/* SHIELDCTRL1 */

/*
 * SYSCTRL1: shipping off, timeout 6.4 ms, AUTO-IDLE left DISABLED (bit3=0).
 * Auto-idle gates the TCPC alert from reaching the PMIC PD_EVT line unless the
 * vendor wakeup interrupts are also armed; keeping the controller awake is the
 * simplest way to get reliable CC-status alerts.
 */
#define MT6375_SYSCTRL1_INIT		0xB0

#define MT6375_VENDOR_ID		0x29CF
#define MT6375_PRODUCT_ID		0x6375

/*
 * Alert sources the TCPCI core actually services (this mirrors the mask it
 * computes in tcpci_init()).  Fault, the voltage alarms and the extended
 * alerts are deliberately left out -- see mt6375_tcpc_init().
 */
#define MT6375_ALERT_MASK	(TCPC_ALERT_CC_STATUS | TCPC_ALERT_POWER_STATUS | \
				 TCPC_ALERT_RX_STATUS | TCPC_ALERT_RX_HARD_RST | \
				 TCPC_ALERT_TX_SUCCESS | TCPC_ALERT_TX_FAILED | \
				 TCPC_ALERT_TX_DISCARDED)

struct mt6375_tcpc_priv {
	struct device *dev;
	struct tcpci *tcpci;
	struct tcpci_data tcpci_data;
};

/*
 * PHY/CC/TCPC tuning taken from the downstream chip init.  The 16-bit TCPCCTRL3
 * (RP duty = 308, 30%) is written as two little-endian bytes.
 */
static const struct reg_sequence mt6375_reg_init[] = {
	REG_SEQ0(MT6375_REG_PHYCTRL1, 0x74),
	REG_SEQ0(MT6375_REG_PHYCTRL2, 0x3A),
	REG_SEQ0(MT6375_REG_PHYCTRL3, 0x82),
	REG_SEQ0(MT6375_REG_PHYCTRL7, 0x36),
	REG_SEQ0(MT6375_REG_PHYCTRL9, 0x3C),
	REG_SEQ0(MT6375_REG_VCONCTRL3, 0x11),
	REG_SEQ0(MT6375_REG_TCPCCTRL1, 0x14),
	REG_SEQ0(MT6375_REG_TCPCCTRL2, 0x00),
	REG_SEQ0(MT6375_REG_TCPCCTRL3, 0x34),		/* RP duty low */
	REG_SEQ0(MT6375_REG_TCPCCTRL3 + 1, 0x01),	/* RP duty high */
	REG_SEQ0(MT6375_REG_LPWRCTRL5, 0x2F),
	REG_SEQ0(MT6375_REG_HILOCTRL9, 0xAA),
	REG_SEQ(MT6375_REG_SYSCTRL1, MT6375_SYSCTRL1_INIT, 1000),
};

static int mt6375_tcpc_init(struct tcpci *tcpci, struct tcpci_data *data)
{
	__le16 alert_mask;
	int ret;

	ret = regmap_register_patch(data->regmap, mt6375_reg_init,
				    ARRAY_SIZE(mt6375_reg_init));
	if (ret)
		return ret;

	/* Open CC 40 ms when the PMIC reports SYSUV */
	ret = regmap_set_bits(data->regmap, MT6375_REG_SHIELDCTRL1,
			      MT6375_OPEN40MS_EN_MASK);
	if (ret)
		return ret;

	/* Auto-discharge timer for low IQ */
	ret = regmap_set_bits(data->regmap, MT6375_REG_WATCHDOGCTRL,
			      MT6375_VBUSDISCHG_OPT_MASK);
	if (ret)
		return ret;

	/* Assert Alert.CCStatus when Looking4Connection changes */
	ret = regmap_set_bits(data->regmap, TCPC_TCPC_CTRL,
			      TCPC_TCPC_CTRL_EN_LK4CONN_ALRT);
	if (ret)
		return ret;

	/* Disable bleed discharge for lower IQ */
	ret = regmap_clear_bits(data->regmap, TCPC_POWER_CTRL,
				TCPC_POWER_CTRL_BLEED_DISCHARGE);
	if (ret)
		return ret;

	/*
	 * Program the hardware Alert mask.  The TCPCI core computes
	 * tcpci->alert_mask but only writes it to TCPC_ALERT_MASK from the I2C
	 * tcpci_probe() path; drivers that bind through tcpci_register_port()
	 * (like this one) never do, so the register keeps its reset default
	 * with every source -- including Fault -- left unmasked.
	 *
	 * On this board VBUS for the host/OTG role is supplied by an external
	 * boost, so the TCPC's own force-discharge cannot pull VBUS down and
	 * TCPC_FAULT_STATUS latches a fault the core never clears.  With Fault
	 * unmasked that holds Alert# permanently asserted, and because the
	 * MT6375 routes Alert# to an edge-latched PMIC interrupt (PD_EVT) the
	 * stuck-high line produces no further edges: TCPM then stops seeing CC
	 * changes and hot-plug/flip is missed.  Mask every source the core
	 * does not service so Alert# can fall again after each handled alert.
	 */
	alert_mask = cpu_to_le16(MT6375_ALERT_MASK);
	return regmap_raw_write(data->regmap, TCPC_ALERT_MASK, &alert_mask,
				sizeof(alert_mask));
}

static irqreturn_t mt6375_irq_handler(int irq, void *dev_id)
{
	struct mt6375_tcpc_priv *priv = dev_id;

	return tcpci_irq(priv->tcpci);
}

static int mt6375_check_vendor_info(struct mt6375_tcpc_priv *priv)
{
	struct regmap *regmap = priv->tcpci_data.regmap;
	u16 id;
	int ret;

	ret = regmap_raw_read(regmap, TCPC_VENDOR_ID, &id, sizeof(u16));
	if (ret)
		return ret;
	if (le16_to_cpu((__force __le16)id) != MT6375_VENDOR_ID)
		return dev_err_probe(priv->dev, -ENODEV,
				     "Unexpected vendor id 0x%04x\n", id);

	ret = regmap_raw_read(regmap, TCPC_PRODUCT_ID, &id, sizeof(u16));
	if (ret)
		return ret;
	if (le16_to_cpu((__force __le16)id) != MT6375_PRODUCT_ID)
		return dev_err_probe(priv->dev, -ENODEV,
				     "Unexpected product id 0x%04x\n", id);

	return 0;
}

static void mt6375_unregister_tcpci_port(void *tcpci)
{
	tcpci_unregister_port(tcpci);
}

static int mt6375_tcpc_probe(struct platform_device *pdev)
{
	struct mt6375_tcpc_priv *priv;
	struct device *dev = &pdev->dev;
	int irq, ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;

	priv->tcpci_data.regmap = dev_get_regmap(dev->parent, NULL);
	if (!priv->tcpci_data.regmap)
		return dev_err_probe(dev, -ENODEV, "Failed to get regmap\n");

	ret = mt6375_check_vendor_info(priv);
	if (ret)
		return ret;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	priv->tcpci_data.auto_discharge_disconnect = 1;
	priv->tcpci_data.init = mt6375_tcpc_init;

	priv->tcpci = tcpci_register_port(dev, &priv->tcpci_data);
	if (IS_ERR(priv->tcpci))
		return dev_err_probe(dev, PTR_ERR(priv->tcpci),
				     "Failed to register tcpci port\n");

	ret = devm_add_action_or_reset(dev, mt6375_unregister_tcpci_port,
				       priv->tcpci);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(dev, irq, NULL, mt6375_irq_handler,
					IRQF_ONESHOT, dev_name(dev), priv);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to request irq\n");

	device_init_wakeup(dev, true);
	dev_pm_set_wake_irq(dev, irq);

	return 0;
}

static void mt6375_tcpc_remove(struct platform_device *pdev)
{
	dev_pm_clear_wake_irq(&pdev->dev);
	device_init_wakeup(&pdev->dev, false);
}

static const struct of_device_id mt6375_tcpc_of_match[] = {
	{ .compatible = "mediatek,mt6375-tcpc" },
	{}
};
MODULE_DEVICE_TABLE(of, mt6375_tcpc_of_match);

static struct platform_driver mt6375_tcpc_driver = {
	.driver = {
		.name = "mt6375-tcpc",
		.of_match_table = mt6375_tcpc_of_match,
	},
	.probe = mt6375_tcpc_probe,
	.remove = mt6375_tcpc_remove,
};
module_platform_driver(mt6375_tcpc_driver);

MODULE_DESCRIPTION("MediaTek MT6375 USB Type-C Port Controller Interface driver");
MODULE_LICENSE("GPL");
