// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek MT6685 SPMI PMIC (clock buffer, RTC) core driver
 *
 * Registers the register map of the PMIC on the SPMI bus and instantiates
 * the function blocks described as child nodes, which share it.
 *
 * The MT6685 aggregates its sub-block interrupts (RTC alarm, ...) into one
 * SCK_TOP interrupt group and raises them on its SPMI SID line. This driver
 * demuxes that group into a small interrupt domain so the function-block
 * drivers (e.g. the RTC alarm) can request their interrupt from the device
 * tree, mirroring drivers/mfd/mtk-spmi-pmic.c for the mt6363/mt6373.
 */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>
#include <linux/spmi.h>
#include <linux/mfd/mt6685.h>

/* SCK_TOP interrupt group: 8 sources in one status/enable register. */
#define MT6685_SCK_TOP_INT_CON0		0x528	/* per-source enable */
#define MT6685_SCK_TOP_INT_STATUS0	0x534	/* per-source status, W1C */
#define MT6685_RCS_IRQ_DONE		0x41b	/* release the SPMI RCS flag */
#define MT6685_NR_INT			8

/*
 * Audio 26 MHz DCXO external buffer (DCXO_EXTBUF5_CW0). Bit layout:
 *   [1:0] BBCK5_MODE, [2] BBCK5_EN_M (manual enable),
 *   [5:3] EXTBUF_BBCK5_RSEL (output resistor select),
 *   [7:6] EXTBUF_BBCK5_HD   (drive strength).
 * Enabling bit 2 alone is not enough: the buffer only drives a usable clock
 * with RSEL and HD set. On stock those are programmed by the clkbuf driver
 * at boot (which mainline does not have), so the register reads 0xbc while
 * audio plays (MODE=0, EN=1, RSEL=7, HD=2). Program that whole config here
 * when enabling; on disable clear only EN_M so the drive config is retained.
 */
#define MT6685_DCXO_CW00		0x79e
#define MT6685_DCXO_AUD_ENABLE		BIT(2)
#define MT6685_DCXO_MODE_MASK		GENMASK(1, 0)
#define MT6685_DCXO_AUD_CFG_MASK	GENMASK(7, 2)	/* EN_M | RSEL | HD */
#define MT6685_DCXO_AUD_CFG_ON		0xbc		/* EN=1, RSEL=7, HD=2 */

static const struct regmap_config mt6685_regmap_config = {
	.reg_bits	= 16,
	.val_bits	= 8,
	.max_register	= 0x2000,
	.fast_io	= true,
};

struct mt6685 {
	struct device		*dev;
	struct regmap		*regmap;
	struct irq_domain	*domain;
	int			irq;
};

static void mt6685_irq_mask(struct irq_data *d)
{
	struct mt6685 *mt = irq_data_get_irq_chip_data(d);

	regmap_update_bits(mt->regmap, MT6685_SCK_TOP_INT_CON0,
			   BIT(irqd_to_hwirq(d)), 0);
}

static void mt6685_irq_unmask(struct irq_data *d)
{
	struct mt6685 *mt = irq_data_get_irq_chip_data(d);

	regmap_update_bits(mt->regmap, MT6685_SCK_TOP_INT_CON0,
			   BIT(irqd_to_hwirq(d)), BIT(irqd_to_hwirq(d)));
}

/* A wakeup source here has to arm the chained SPMI interrupt it comes from. */
static int mt6685_irq_set_wake(struct irq_data *d, unsigned int on)
{
	struct mt6685 *mt = irq_data_get_irq_chip_data(d);

	return irq_set_irq_wake(mt->irq, on);
}

static struct irq_chip mt6685_irq_chip = {
	.name		= "mt6685",
	.irq_mask	= mt6685_irq_mask,
	.irq_unmask	= mt6685_irq_unmask,
	.irq_set_wake	= mt6685_irq_set_wake,
};

static int mt6685_irq_map(struct irq_domain *d, unsigned int virq,
			  irq_hw_number_t hwirq)
{
	struct mt6685 *mt = d->host_data;

	irq_set_chip_data(virq, mt);
	irq_set_chip_and_handler(virq, &mt6685_irq_chip, handle_level_irq);
	irq_set_noprobe(virq);

	return 0;
}

static const struct irq_domain_ops mt6685_irq_domain_ops = {
	.map	= mt6685_irq_map,
	.xlate	= irq_domain_xlate_twocell,
};

static void mt6685_handle_chained_irq(struct irq_desc *desc)
{
	struct mt6685 *mt = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned long status;
	unsigned int val;
	int bit;

	chained_irq_enter(chip, desc);

	if (regmap_read(mt->regmap, MT6685_SCK_TOP_INT_STATUS0, &val))
		val = 0;

	/*
	 * Nothing pending: the PMIC can send a further RCS while a source
	 * is masked for its threaded handler. Only the RCS needs acking.
	 */
	if (val) {
		status = val;
		for_each_set_bit(bit, &status, MT6685_NR_INT)
			generic_handle_domain_irq_safe(mt->domain, bit);
		/* Write 1 to clear the serviced sources. */
		regmap_write(mt->regmap, MT6685_SCK_TOP_INT_STATUS0, val);
	}

	/* Release the SPMI RCS flag so the PMIC can raise the next interrupt. */
	regmap_write(mt->regmap, MT6685_RCS_IRQ_DONE, 1);

	chained_irq_exit(chip, desc);
}

static void mt6685_irq_remove(void *data)
{
	struct mt6685 *mt = data;

	irq_set_chained_handler_and_data(mt->irq, NULL, NULL);
	irq_domain_remove(mt->domain);
}

static int mt6685_irq_init(struct mt6685 *mt)
{
	struct device *dev = mt->dev;
	int ret;

	mt->irq = of_irq_get(dev->of_node, 0);
	if (mt->irq < 0)
		return dev_err_probe(dev, mt->irq, "failed to get SPMI IRQ\n");

	mt->domain = irq_domain_add_linear(dev->of_node, MT6685_NR_INT,
					   &mt6685_irq_domain_ops, mt);
	if (!mt->domain)
		return dev_err_probe(dev, -ENOMEM, "failed to add IRQ domain\n");

	ret = devm_add_action_or_reset(dev, mt6685_irq_remove, mt);
	if (ret)
		return ret;

	/* Start from a clean state: disable and clear all sources. */
	regmap_write(mt->regmap, MT6685_SCK_TOP_INT_CON0, 0);
	regmap_write(mt->regmap, MT6685_SCK_TOP_INT_STATUS0, GENMASK(7, 0));

	irq_set_chained_handler_and_data(mt->irq, mt6685_handle_chained_irq, mt);

	/* The bootloader may have left an RCS pending: release it now. */
	regmap_write(mt->regmap, MT6685_RCS_IRQ_DONE, 1);

	return 0;
}

/*
 * The audio DCXO buffer lives on the MT6685 but is driven from the MT6369
 * codec's downlink path. Keep a reference to the regmap for those helpers;
 * there is only ever one MT6685 in a system.
 */
static struct regmap *mt6685_regmap;

void mt6685_set_dcxo(bool enable)
{
	if (!mt6685_regmap)
		return;

	if (enable) {
		regmap_update_bits(mt6685_regmap, MT6685_DCXO_CW00,
				   MT6685_DCXO_AUD_CFG_MASK,
				   MT6685_DCXO_AUD_CFG_ON);
		usleep_range(400, 420);
	} else {
		regmap_update_bits(mt6685_regmap, MT6685_DCXO_CW00,
				   MT6685_DCXO_AUD_ENABLE, 0);
	}
}
EXPORT_SYMBOL_GPL(mt6685_set_dcxo);

void mt6685_set_dcxo_mode(u8 mode)
{
	if (!mt6685_regmap)
		return;

	regmap_update_bits(mt6685_regmap, MT6685_DCXO_CW00,
			   MT6685_DCXO_MODE_MASK, mode);
}
EXPORT_SYMBOL_GPL(mt6685_set_dcxo_mode);

static int mt6685_spmi_probe(struct spmi_device *sdev)
{
	struct device *dev = &sdev->dev;
	struct mt6685 *mt;
	int ret;

	mt = devm_kzalloc(dev, sizeof(*mt), GFP_KERNEL);
	if (!mt)
		return -ENOMEM;

	mt->dev = dev;
	mt->regmap = devm_regmap_init_spmi_ext(sdev, &mt6685_regmap_config);
	if (IS_ERR(mt->regmap))
		return PTR_ERR(mt->regmap);

	mt6685_regmap = mt->regmap;

	/*
	 * No chip ID check: reading the ID (0x8) from the AP times out and
	 * makes the PMIF clear its init-done flag, after which the modem
	 * cannot use its PMIF channel and hangs in its DVFS init.
	 */

	if (of_property_present(dev->of_node, "interrupts") ||
	    of_property_present(dev->of_node, "interrupts-extended")) {
		ret = mt6685_irq_init(mt);
		if (ret)
			return ret;
	}

	return devm_of_platform_populate(dev);
}

static const struct of_device_id mt6685_spmi_match[] = {
	{ .compatible = "mediatek,mt6685" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mt6685_spmi_match);

static struct spmi_driver mt6685_spmi_driver = {
	.driver = {
		.name = "mt6685-spmi",
		.of_match_table = mt6685_spmi_match,
	},
	.probe = mt6685_spmi_probe,
};
module_spmi_driver(mt6685_spmi_driver);

MODULE_DESCRIPTION("MediaTek MT6685 SPMI PMIC core driver");
MODULE_LICENSE("GPL");
