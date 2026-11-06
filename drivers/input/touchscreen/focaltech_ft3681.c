// SPDX-License-Identifier: GPL-2.0-only
/*
 * FocalTech FT3681 SPI touchscreen driver
 *
 * A minimal mainline driver for the FocalTech FT3681 capacitive touch
 * controller, as found on the Volla Phone Plinius (MediaTek MT6878) with a
 * Visionox HC5622 panel. The controller runs firmware resident in the panel
 * module, so this driver only reads touch reports; it deliberately omits the
 * downstream firmware-upgrade, gesture, proximity and ESD-check machinery.
 *
 * The SPI "v2" framing and the touch-report layout were derived from the
 * FocalTech downstream focaltech_ft3681_spi driver.
 *
 * Copyright (c) 2012-2020, FocalTech Systems, Ltd. (downstream protocol)
 * Copyright (c) 2026 Jan Benedikt <karagos01@gmail.com>
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>

/* Bring-up diagnostic: poll the touch frame instead of using the IRQ. */
static bool ft3681_poll;
module_param_named(poll, ft3681_poll, bool, 0644);

#define FT3681_MAX_POINTS	10
#define FT3681_ONE_TCH_LEN_V1	6
#define FT3681_ONE_TCH_LEN_V2	8
/* Frame read from reg 0x01: up to a 4-byte header + 10 * 8-byte points. */
#define FT3681_FRAME_LEN	(FT3681_MAX_POINTS * FT3681_ONE_TCH_LEN_V2 + 4)

#define FT3681_REG_TD_ADDR	0x01	/* touch-data frame base */
#define FT3681_REG_CHIP_ID	0xa3
#define FT3681_REG_CHIP_ID2	0x9f
#define FT3681_REG_FW_VER	0xa6
#define FT3681_REG_WORKMODE	0x00
#define FT3681_REG_INT_CNT	0x8f
#define FT3681_REG_FLOW_CNT	0x91
#define FT3681_REG_POWER_MODE	0xa5
#define FT3681_CHIP_ID		0x56
#define FT3681_CHIP_ID2		0x62

/* SPI "v2" framing */
#define FT3681_SPI_READ		0x80
#define FT3681_SPI_CRC_EN	0x20
#define FT3681_SPI_DUMMY	3	/* dummy bytes after the 4-byte header */
#define FT3681_SPI_HDR		4	/* addr, ctrl, lenH, lenL */
#define FT3681_SPI_STATUS_OFF	3	/* status echoed in rx[3] */
#define FT3681_SPI_STATUS_MASK	0xa0
#define FT3681_SPI_RETRY	3
#define FT3681_CS_DELAY_US	150

/* Per-point byte offsets inside one touch entry. */
#define FT3681_OFF_E_XH		0	/* [7:6] event, [3:0] x[11:8] */
#define FT3681_OFF_XL		1	/* x[7:0] */
#define FT3681_OFF_ID_YH	2	/* [7:4] id, [3:0] y[11:8] */
#define FT3681_OFF_YL		3	/* y[7:0] */
#define FT3681_OFF_AREA		5	/* touch major */

#define FT3681_E_NUM		1	/* frame[1]: etype<<4 | event count */
#define FT3681_ETYPE_V2		0x2

/* event flag in bits [7:6] of E_XH */
#define FT3681_EVT_DOWN		0
#define FT3681_EVT_CONTACT	2

/* worst-case SPI transfer: header + dummy + frame + 2-byte CRC */
#define FT3681_BUF_LEN		(FT3681_SPI_HDR + FT3681_SPI_DUMMY + FT3681_FRAME_LEN + 2)

struct ft3681 {
	struct spi_device *spi;
	struct input_dev *input;
	struct touchscreen_properties prop;
	struct gpio_desc *reset_gpio;
	struct regulator *vdd;
	struct delayed_work poll_work;
	unsigned int poll_dbg;	/* diagnostic log budget */
	u8 prev_intc;		/* last FT3681_REG_INT_CNT seen */
	u8 *frame;		/* parsed touch frame (FT3681_FRAME_LEN) */
	u8 *tx;			/* DMA-safe SPI buffers (FT3681_BUF_LEN) */
	u8 *rx;
};

/* CRC-16/KERMIT (reflected, poly 0x1021 -> 0x8408, init 0xffff) */
static u16 ft3681_crc(const u8 *data, u32 len)
{
	u16 crc = 0xffff;
	u32 i;
	int j;

	for (i = 0; i < len; i++) {
		crc ^= data[i];
		for (j = 0; j < 8; j++)
			crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : (crc >> 1);
	}
	return crc;
}

/*
 * Read @len bytes starting at register @addr using the FT3681 SPI v2
 * protocol: tx = [addr, READ|CRC, lenH, lenL, 3 dummy, len zero bytes,
 * 2 zero CRC bytes]. The controller echoes a status byte in rx[3] and
 * returns the payload after the dummy bytes, trailed by its CRC.
 */
static int ft3681_read(struct ft3681 *ts, u8 addr, u8 *val, u16 len)
{
	u32 dp = FT3681_SPI_HDR + FT3681_SPI_DUMMY;	/* payload offset = 7 */
	u32 xlen = dp + len + 2;			/* + CRC */
	struct spi_transfer xfer = {
		.tx_buf = ts->tx,
		.rx_buf = ts->rx,
		.len = xlen,
	};
	u16 crc_calc, crc_read;
	int ret, i;

	if (xlen > FT3681_BUF_LEN)
		return -EINVAL;

	for (i = 0; i < FT3681_SPI_RETRY; i++) {
		memset(ts->tx, 0, xlen);
		memset(ts->rx, 0, xlen);
		ts->tx[0] = addr;
		ts->tx[1] = FT3681_SPI_READ | FT3681_SPI_CRC_EN;
		ts->tx[2] = (len >> 8) & 0xff;
		ts->tx[3] = len & 0xff;

		ret = spi_sync_transfer(ts->spi, &xfer, 1);
		if (ret)
			return ret;

		if (ts->rx[FT3681_SPI_STATUS_OFF] & FT3681_SPI_STATUS_MASK) {
			udelay(FT3681_CS_DELAY_US);
			continue;
		}

		crc_calc = ft3681_crc(&ts->rx[dp], len);
		crc_read = (ts->rx[dp + len + 1] << 8) | ts->rx[dp + len];
		if (crc_calc != crc_read) {
			udelay(FT3681_CS_DELAY_US);
			continue;
		}

		memcpy(val, &ts->rx[dp], len);
		udelay(FT3681_CS_DELAY_US);
		return 0;
	}

	udelay(FT3681_CS_DELAY_US);
	return -EIO;
}

static void ft3681_report(struct ft3681 *ts)
{
	const u8 *buf = ts->frame;
	struct input_dev *input = ts->input;
	unsigned int etype = (buf[FT3681_E_NUM] >> 4) & 0x0f;
	unsigned int count = buf[FT3681_E_NUM] & 0x0f;
	unsigned int tch_len, hdr, i, n;

	if (etype == FT3681_ETYPE_V2) {
		/* v2: exactly @count 8-byte entries after a 4-byte header */
		tch_len = FT3681_ONE_TCH_LEN_V2;
		hdr = 4;
		n = count;
	} else {
		/* v1 (default): up to 10 6-byte entries after a 2-byte header */
		tch_len = FT3681_ONE_TCH_LEN_V1;
		hdr = 2;
		n = FT3681_MAX_POINTS;
	}

	if (count > FT3681_MAX_POINTS)
		return;

	for (i = 0; i < n; i++) {
		const u8 *p = &buf[hdr + i * tch_len];
		unsigned int id = p[FT3681_OFF_ID_YH] >> 4;
		unsigned int flag = p[FT3681_OFF_E_XH] >> 6;
		unsigned int x, y;
		bool down;

		/* v1 marks "no more points" with an id past the last slot */
		if (id >= FT3681_MAX_POINTS)
			break;

		x = ((p[FT3681_OFF_E_XH] & 0x0f) << 8) | p[FT3681_OFF_XL];
		y = ((p[FT3681_OFF_ID_YH] & 0x0f) << 8) | p[FT3681_OFF_YL];
		down = (flag == FT3681_EVT_DOWN || flag == FT3681_EVT_CONTACT);

		input_mt_slot(input, id);
		if (input_mt_report_slot_state(input, MT_TOOL_FINGER, down)) {
			input_report_abs(input, ABS_MT_TOUCH_MAJOR,
					  p[FT3681_OFF_AREA]);
			touchscreen_report_pos(input, &ts->prop, x, y, true);
		}
	}

	input_mt_sync_frame(input);
	input_sync(input);
}

static irqreturn_t ft3681_irq(int irq, void *data)
{
	struct ft3681 *ts = data;

	if (!ft3681_read(ts, FT3681_REG_TD_ADDR, ts->frame, FT3681_FRAME_LEN))
		ft3681_report(ts);

	return IRQ_HANDLED;
}

static void ft3681_poll_work(struct work_struct *work)
{
	struct ft3681 *ts = container_of(work, struct ft3681, poll_work.work);
	u8 flow = 0, intc = 0, wmode = 0xee, pmode = 0xee;
	int ret;

	/* Diagnostics: is the FW main loop running and generating interrupts? */
	ft3681_read(ts, FT3681_REG_FLOW_CNT, &flow, 1);
	ft3681_read(ts, FT3681_REG_INT_CNT, &intc, 1);
	ft3681_read(ts, FT3681_REG_WORKMODE, &wmode, 1);
	ft3681_read(ts, FT3681_REG_POWER_MODE, &pmode, 1);

	ret = ft3681_read(ts, FT3681_REG_TD_ADDR, ts->frame, FT3681_FRAME_LEN);

	/* Log the first few polls, then only when the chip's internal IRQ
	 * counter moves (a touch) or a valid touch frame shows up.
	 */
	if (ts->poll_dbg || intc != ts->prev_intc ||
	    (ret == 0 && ts->frame[FT3681_E_NUM] != 0xff)) {
		if (ts->poll_dbg)
			ts->poll_dbg--;
		dev_info(&ts->spi->dev,
			 "diag: flow=%02x int=%02x wm=%02x pw=%02x rd=%d frame=%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
			 flow, intc, wmode, pmode, ret,
			 ts->frame[0], ts->frame[1], ts->frame[2], ts->frame[3],
			 ts->frame[4], ts->frame[5], ts->frame[6], ts->frame[7],
			 ts->frame[8], ts->frame[9]);
	}
	ts->prev_intc = intc;
	if (!ret)
		ft3681_report(ts);

	schedule_delayed_work(&ts->poll_work, msecs_to_jiffies(300));
}

static ssize_t frame_show(struct device *dev, struct device_attribute *attr,
			  char *out)
{
	struct ft3681 *ts = spi_get_drvdata(to_spi_device(dev));
	u8 flow = 0, intc = 0, wm = 0, pm = 0;
	int ret;

	ft3681_read(ts, FT3681_REG_FLOW_CNT, &flow, 1);
	ft3681_read(ts, FT3681_REG_INT_CNT, &intc, 1);
	ft3681_read(ts, FT3681_REG_WORKMODE, &wm, 1);
	ft3681_read(ts, FT3681_REG_POWER_MODE, &pm, 1);
	ret = ft3681_read(ts, FT3681_REG_TD_ADDR, ts->frame, FT3681_FRAME_LEN);

	return sysfs_emit(out, "rd=%d flow=%02x int=%02x wm=%02x pm=%02x frame=%*ph\n",
			  ret, flow, intc, wm, pm, 24, ts->frame);
}
static DEVICE_ATTR_RO(frame);

static struct attribute *ft3681_attrs[] = {
	&dev_attr_frame.attr,
	NULL,
};
static const struct attribute_group ft3681_group = {
	.attrs = ft3681_attrs,
};

static void ft3681_reset(struct ft3681 *ts)
{
	/* reset is active low: assert (line low), hold, release, wait for FW */
	gpiod_set_value_cansleep(ts->reset_gpio, 1);
	msleep(2);
	gpiod_set_value_cansleep(ts->reset_gpio, 0);
	msleep(200);
}

static int ft3681_check_id(struct ft3681 *ts)
{
	u8 id = 0, id2 = 0;
	int ret, i;

	/* The controller needs a moment after reset; retry with a re-reset. */
	for (i = 0; i < 3; i++) {
		ret = ft3681_read(ts, FT3681_REG_CHIP_ID, &id, 1);
		if (!ret)
			ret = ft3681_read(ts, FT3681_REG_CHIP_ID2, &id2, 1);
		if (!ret && id == FT3681_CHIP_ID && id2 == FT3681_CHIP_ID2) {
			dev_dbg(&ts->spi->dev, "FT3681 chip id 0x%02x%02x\n",
				id, id2);
			return 0;
		}
		dev_dbg(&ts->spi->dev,
			"chip id read try %d: ret %d id 0x%02x%02x\n",
			i, ret, id, id2);
		ft3681_reset(ts);
	}

	dev_err(&ts->spi->dev, "chip id mismatch/unreadable (0x%02x%02x)\n",
		id, id2);
	return -ENODEV;
}

static void ft3681_disable_vdd(void *data)
{
	regulator_disable(data);
}

static void ft3681_stop_poll(void *data)
{
	struct ft3681 *ts = data;

	cancel_delayed_work_sync(&ts->poll_work);
}

static int ft3681_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct ft3681 *ts;
	int ret;

	if (!spi->irq)
		return dev_err_probe(dev, -EINVAL, "no IRQ specified\n");

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->spi = spi;
	spi_set_drvdata(spi, ts);

	ts->frame = devm_kzalloc(dev, FT3681_FRAME_LEN, GFP_KERNEL);
	ts->tx = devm_kzalloc(dev, FT3681_BUF_LEN, GFP_KERNEL);
	ts->rx = devm_kzalloc(dev, FT3681_BUF_LEN, GFP_KERNEL);
	if (!ts->frame || !ts->tx || !ts->rx)
		return -ENOMEM;

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret)
		return dev_err_probe(dev, ret, "SPI setup failed\n");

	/* Panel supply; may be always-on from the bootloader, so optional. */
	ts->vdd = devm_regulator_get_optional(dev, "vdd");
	if (IS_ERR(ts->vdd)) {
		ret = PTR_ERR(ts->vdd);
		if (ret != -ENODEV)
			return dev_err_probe(dev, ret, "failed to get vdd\n");
		ts->vdd = NULL;
	}
	if (ts->vdd) {
		ret = regulator_enable(ts->vdd);
		if (ret)
			return dev_err_probe(dev, ret, "failed to enable vdd\n");
		ret = devm_add_action_or_reset(dev, ft3681_disable_vdd, ts->vdd);
		if (ret)
			return ret;
	}

	ts->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ts->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ts->reset_gpio),
				     "failed to get reset gpio\n");

	ft3681_reset(ts);

	ret = ft3681_check_id(ts);
	if (ret)
		return ret;

	{
		u8 fw = 0;

		if (!ft3681_read(ts, FT3681_REG_FW_VER, &fw, 1))
			dev_info(dev, "FT3681 firmware version 0x%02x\n", fw);
	}

	ts->input = devm_input_allocate_device(dev);
	if (!ts->input)
		return -ENOMEM;

	ts->input->name = "FocalTech FT3681";
	ts->input->phys = "input/ts";
	ts->input->id.bustype = BUS_SPI;

	input_set_abs_params(ts->input, ABS_MT_POSITION_X, 0, 0, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_Y, 0, 0, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);

	touchscreen_parse_properties(ts->input, true, &ts->prop);
	if (!ts->prop.max_x || !ts->prop.max_y)
		return dev_err_probe(dev, -EINVAL,
				     "touchscreen-size-x/y missing\n");

	ret = input_mt_init_slots(ts->input, FT3681_MAX_POINTS,
				  INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (ret)
		return dev_err_probe(dev, ret, "failed to init MT slots\n");

	ret = input_register_device(ts->input);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register input\n");

	ret = devm_device_add_group(dev, &ft3681_group);
	if (ret)
		return ret;

	ts->poll_dbg = 4;
	if (ft3681_poll) {
		INIT_DELAYED_WORK(&ts->poll_work, ft3681_poll_work);
		ret = devm_add_action_or_reset(dev, ft3681_stop_poll, ts);
		if (ret)
			return ret;
		schedule_delayed_work(&ts->poll_work, msecs_to_jiffies(100));
		dev_info(dev, "using polling mode (bring-up)\n");
	} else {
		ret = devm_request_threaded_irq(dev, spi->irq, NULL, ft3681_irq,
						IRQF_ONESHOT, dev_name(dev), ts);
		if (ret)
			return dev_err_probe(dev, ret, "failed to request irq\n");
	}

	return 0;
}

static const struct of_device_id ft3681_of_match[] = {
	{ .compatible = "focaltech,ft3681" },
	{ }
};
MODULE_DEVICE_TABLE(of, ft3681_of_match);

static const struct spi_device_id ft3681_spi_id[] = {
	{ "ft3681" },
	{ }
};
MODULE_DEVICE_TABLE(spi, ft3681_spi_id);

static struct spi_driver ft3681_driver = {
	.driver = {
		.name = "focaltech_ft3681",
		.of_match_table = ft3681_of_match,
	},
	.id_table = ft3681_spi_id,
	.probe = ft3681_probe,
};
module_spi_driver(ft3681_driver);

MODULE_AUTHOR("Jan Benedikt <karagos01@gmail.com>");
MODULE_DESCRIPTION("FocalTech FT3681 SPI touchscreen driver");
MODULE_LICENSE("GPL");
