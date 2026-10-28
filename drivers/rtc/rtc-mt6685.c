// SPDX-License-Identifier: GPL-2.0
/*
 * Real time clock in the MediaTek MT6685 SPMI PMIC.
 *
 * The MT6685 is a companion PMIC on the SPMI bus (clock buffer, RTC and
 * consys helpers). Its RTC block is a separate register IP from the older
 * MediaTek PMIC RTCs (mt6358/mt6397): the registers are byte addressed with
 * a two-byte stride per 16-bit field, the block sits at a fixed offset, and
 * the RTC source clocks must be ungated before the counter registers can be
 * written. The register map is reached through the parent MT6685 MFD core
 * (drivers/mfd/mt6685-spmi.c), which owns the SPMI regmap.
 *
 * This driver provides the time-of-day counter only (read/set). The alarm
 * path needs the MT6685 interrupt to be routed, which is not wired up yet.
 */

#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/rtc.h>

/* Fixed base of the RTC block within the MT6685 SPMI register space. */
#define MT6685_RTC_BASE			0x580

/* Clock gating (absolute addresses, RTC source clocks). */
#define MT6685_CKPDN_CON0_L_SET		0x515
#define MT6685_CKPDN_CON0_L_CLR		0x516
#define MT6685_RG_RTC_MCLK_PDN		BIT(4)
#define MT6685_RG_RTC_32K_CK_PDN	BIT(5)

/* RTC registers, relative to MT6685_RTC_BASE. */
#define RTC_BBPU			0x08
#define RTC_BBPU_KEY			(0x43 << 8)
#define RTC_BBPU_CBUSY			BIT(6)
#define RTC_BBPU_RELOAD			BIT(5)
#define RTC_BBPU_PWREN			BIT(0)

#define RTC_TC_SEC			0x12
/* MIN, HOUR, DOM, DOW, MTH, YEAR follow at a two-byte stride. */
#define RTC_WRTGR			0x42

#define RTC_TC_SEC_MASK			0x3f
#define RTC_TC_MIN_MASK			0x3f
#define RTC_TC_HOU_MASK			0x1f
#define RTC_TC_DOM_MASK			0x1f
#define RTC_TC_DOW_MASK			0x07
#define RTC_TC_MTH_MASK			0x0f
#define RTC_TC_YEA_MASK			0x7f

enum {
	RTC_OFFSET_SEC,
	RTC_OFFSET_MIN,
	RTC_OFFSET_HOUR,
	RTC_OFFSET_DOM,
	RTC_OFFSET_DOW,
	RTC_OFFSET_MTH,
	RTC_OFFSET_YEAR,
	RTC_OFFSET_COUNT,
};

/* HW year 0 maps to 1968 (a leap year, like 2000). */
#define RTC_MIN_YEAR			1968
#define RTC_BASE_YEAR			1900
#define RTC_MIN_YEAR_OFFSET		(RTC_MIN_YEAR - RTC_BASE_YEAR)

#define MTK_RTC_POLL_DELAY_US		10
#define MTK_RTC_POLL_TIMEOUT		(jiffies_to_usecs(HZ))

struct mt6685_rtc {
	struct rtc_device	*rtc_dev;
	struct regmap		*regmap;
	struct mutex		lock;		/* serialise register access */
	u32			base;
};

/*
 * Each logical RTC register is 16 bits wide but exposed as two consecutive
 * byte registers on the SPMI bus. The parent regmap is byte wide, so access
 * one 16-bit field as a low/high byte pair.
 */
static int mt6685_rtc_read(struct mt6685_rtc *rtc, u32 reg, u32 *val)
{
	u8 buf[2];
	int ret;

	ret = regmap_bulk_read(rtc->regmap, rtc->base + reg, buf, 2);
	if (ret < 0)
		return ret;

	*val = buf[0] | (buf[1] << 8);
	return 0;
}

static int mt6685_rtc_write(struct mt6685_rtc *rtc, u32 reg, u32 val)
{
	u8 buf[2] = { val & 0xff, (val >> 8) & 0xff };

	return regmap_bulk_write(rtc->regmap, rtc->base + reg, buf, 2);
}

static int mt6685_rtc_write_trigger(struct mt6685_rtc *rtc)
{
	unsigned long timeout = jiffies + HZ;
	u32 data;
	int ret;

	ret = mt6685_rtc_write(rtc, RTC_WRTGR, 1);
	if (ret < 0)
		return ret;

	do {
		ret = mt6685_rtc_read(rtc, RTC_BBPU, &data);
		if (ret < 0)
			return ret;
		if (!(data & RTC_BBPU_CBUSY))
			return 0;
		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;
		cpu_relax();
	} while (1);
}

static int mt6685_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct mt6685_rtc *rtc = dev_get_drvdata(dev);
	u32 data[RTC_OFFSET_COUNT], bbpu;
	int ret, i;

	mutex_lock(&rtc->lock);

	/* Latch the free-running counter into the readable TC registers. */
	ret = mt6685_rtc_read(rtc, RTC_BBPU, &bbpu);
	if (ret < 0)
		goto out;
	ret = mt6685_rtc_write(rtc, RTC_BBPU,
			       bbpu | RTC_BBPU_KEY | RTC_BBPU_RELOAD);
	if (ret < 0)
		goto out;
	ret = mt6685_rtc_write_trigger(rtc);
	if (ret < 0)
		goto out;

	for (i = 0; i < RTC_OFFSET_COUNT; i++) {
		ret = mt6685_rtc_read(rtc, RTC_TC_SEC + i * 2, &data[i]);
		if (ret < 0)
			goto out;
	}

out:
	mutex_unlock(&rtc->lock);
	if (ret < 0)
		return ret;

	tm->tm_sec  = data[RTC_OFFSET_SEC]  & RTC_TC_SEC_MASK;
	tm->tm_min  = data[RTC_OFFSET_MIN]  & RTC_TC_MIN_MASK;
	tm->tm_hour = data[RTC_OFFSET_HOUR] & RTC_TC_HOU_MASK;
	tm->tm_mday = data[RTC_OFFSET_DOM]  & RTC_TC_DOM_MASK;
	tm->tm_mon  = (data[RTC_OFFSET_MTH] & RTC_TC_MTH_MASK) - 1;
	tm->tm_year = (data[RTC_OFFSET_YEAR] & RTC_TC_YEA_MASK) +
		      RTC_MIN_YEAR_OFFSET;

	return 0;
}

static int mt6685_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct mt6685_rtc *rtc = dev_get_drvdata(dev);
	u32 data[RTC_OFFSET_COUNT];
	int ret, i;

	data[RTC_OFFSET_SEC]  = tm->tm_sec;
	data[RTC_OFFSET_MIN]  = tm->tm_min;
	data[RTC_OFFSET_HOUR] = tm->tm_hour;
	data[RTC_OFFSET_DOM]  = tm->tm_mday;
	data[RTC_OFFSET_DOW]  = tm->tm_wday;
	data[RTC_OFFSET_MTH]  = tm->tm_mon + 1;
	data[RTC_OFFSET_YEAR] = tm->tm_year - RTC_MIN_YEAR_OFFSET;

	mutex_lock(&rtc->lock);

	for (i = 0; i < RTC_OFFSET_COUNT; i++) {
		ret = mt6685_rtc_write(rtc, RTC_TC_SEC + i * 2, data[i]);
		if (ret < 0)
			goto out;
	}

	ret = mt6685_rtc_write_trigger(rtc);
out:
	mutex_unlock(&rtc->lock);
	return ret;
}

static const struct rtc_class_ops mt6685_rtc_ops = {
	.read_time	= mt6685_rtc_read_time,
	.set_time	= mt6685_rtc_set_time,
};

static int mt6685_rtc_probe(struct platform_device *pdev)
{
	struct mt6685_rtc *rtc;
	u32 bbpu;
	int ret;

	rtc = devm_kzalloc(&pdev->dev, sizeof(*rtc), GFP_KERNEL);
	if (!rtc)
		return -ENOMEM;

	rtc->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!rtc->regmap)
		return -ENODEV;

	rtc->base = MT6685_RTC_BASE;
	mutex_init(&rtc->lock);
	platform_set_drvdata(pdev, rtc);

	/* Ungate the RTC source clocks so the counter registers are writable. */
	regmap_write(rtc->regmap, MT6685_CKPDN_CON0_L_CLR,
		     MT6685_RG_RTC_32K_CK_PDN);
	regmap_write(rtc->regmap, MT6685_CKPDN_CON0_L_CLR,
		     MT6685_RG_RTC_MCLK_PDN);

	ret = mt6685_rtc_read(rtc, RTC_BBPU, &bbpu);
	if (ret < 0)
		return dev_err_probe(&pdev->dev, ret, "failed to read RTC\n");
	if (!(bbpu & RTC_BBPU_PWREN))
		dev_warn(&pdev->dev, "RTC not powered (BBPU=0x%04x)\n", bbpu);

	rtc->rtc_dev = devm_rtc_allocate_device(&pdev->dev);
	if (IS_ERR(rtc->rtc_dev))
		return PTR_ERR(rtc->rtc_dev);

	rtc->rtc_dev->ops = &mt6685_rtc_ops;
	rtc->rtc_dev->range_min = mktime64(RTC_MIN_YEAR, 1, 1, 0, 0, 0);
	rtc->rtc_dev->range_max = mktime64(RTC_MIN_YEAR + RTC_TC_YEA_MASK,
					   12, 31, 23, 59, 59);

	return devm_rtc_register_device(rtc->rtc_dev);
}

static const struct of_device_id mt6685_rtc_of_match[] = {
	{ .compatible = "mediatek,mt6685-rtc" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mt6685_rtc_of_match);

static struct platform_driver mt6685_rtc_driver = {
	.driver = {
		.name = "mt6685-rtc",
		.of_match_table = mt6685_rtc_of_match,
	},
	.probe = mt6685_rtc_probe,
};
module_platform_driver(mt6685_rtc_driver);

MODULE_DESCRIPTION("MediaTek MT6685 SPMI PMIC RTC driver");
MODULE_LICENSE("GPL");
