// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek ISP7.x CAMSV raw capture driver
 *
 * CAMSV is the simple capture unit of the MediaTek camera system: it takes
 * the pixel stream of one seninf cam mux and writes it unprocessed to
 * memory. The vendor driver programs it through its SCQ command queue with
 * register lists composed in userspace; this driver writes the same
 * registers directly and uses a single tag.
 *
 * Copyright (c) 2026 Jan Benedikt <karagos01@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include <media/media-device.h>
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-dma-contig.h>

#include "kd_imgsensor_define_v4l2.h"
#include "mtk_cam-seninf-if.h"

/* central registers, per tag registers repeat every 0x40 bytes */
#define CAMSV_DMA_EN_IMG		0x0040
#define CAMSV_DMA_EN_LEN		0x0044
#define CAMSV_DCM_DIS			0x007c
#define CAMSV_SEN_MODE			0x0140
#define  SEN_MODE_CMOS_EN		BIT(0)
#define  SEN_MODE_TG_MODE_OFF		BIT(11)
#define CAMSV_SUB_PERIOD		0x0144
#define CAMSV_VF_CON			0x0148
#define  VF_CON_VFDATA_EN		BIT(0)
#define CAMSV_PATH_CFG			0x014c
#define  PATH_CFG_SYNC_VF_EN_DB_LOAD_DIS BIT(8)
#define CAMSV_ERR_CTL			0x0170
#define  ERR_CTL_GRAB_ERR_EN		BIT(5)
#define CAMSV_TIME_STAMP_CTL		0x018c
#define  TIME_STAMP_SEL			BIT(0)
#define  TIME_STP_EN			BIT(1)
#define CAMSV_MODULE_DB			0x01b4
#define  MODULE_DB_CAM_DB_EN		BIT(30)
#define CAMSV_GROUP_TAG(n)		(0x01b8 + (n) * 4)
#define CAMSV_LAST_TAG			0x01c8
#define CAMSV_FIRST_TAG			0x01cc
#define CAMSV_SW_CTL			0x01d0
#define CAMSV_DCIF_SET			0x01d4
#define CAMSV_DCIF_SEL			0x01d8
#define CAMSV_DONE_STATUS_EN		0x0344
#define CAMSV_DONE_STATUS		0x0348
#define  DONE_ST_GROUP(n)		BIT(16 + (n))
#define CAMSV_ERR_STATUS_EN		0x034c
#define CAMSV_ERR_STATUS		0x0350
#define CAMSV_SOF_STATUS_EN		0x035c
#define CAMSV_SOF_STATUS		0x0360
#define  SOF_ST_TAG(t)			BIT(2 + (t) * 4)
#define CAMSV_TAG_CHANNEL_EN		0x036c
#define CAMSV_CHANNEL_STATUS		0x0370

#define CAMSV_TAG(reg, t)		((reg) + (t) * 0x40)
#define CAMSV_FBC0			0x0540
#define  FBC0_RCNT_INC			BIT(7)
#define  FBC0_FBC_EN			BIT(15)
#define CAMSV_FBC1			0x0544
#define CAMSV_GRAB_PXL			0x0548
#define CAMSV_GRAB_LIN			0x054c
#define CAMSV_FORMAT			0x0554
#define  FORMAT_FMT			GENMASK(3, 0)
/* bit 4 keeps the 8 MSBs only, bit 6 writes LSB-aligned 16-bit words */
#define  FORMAT_UNPACK			BIT(6)
#define CAMSV_CONFIG			0x055c
#define CAMSV_FH_SPARE			0x057c

/* image DMA registers */
#define CAMSVDMA_CON1_IMG		0x0000
#define CAMSVDMA_CON2_IMG		0x0004
#define CAMSVDMA_CON3_IMG		0x0008
#define CAMSVDMA_CON4_IMG		0x000c
#define CAMSVDMA_SW_RST_CTL		0x0020
#define  DMA_SW_RST			BIT(0)
#define  DMA_SW_RST_ST			BIT(1)
#define CAMSVDMA_BASE_ADDR		0x0200
#define CAMSVDMA_BASE_ADDR_MSB		0x0204
#define CAMSVDMA_WDMA_BASIC		0x0210
#define  WDMA_MAX_BURST			GENMASK(4, 0)
#define  WDMA_STRIDE			GENMASK(31, 16)
#define CAMSVDMA_FH_EN			0x0218
#define CAMSVDMA_LEN_BASIC		0x0a10
#define CAMSVDMA_STG1_EN		0x0c00
#define CAMSVDMA_STG1_INIT		0x0c04
#define CAMSVDMA_TAG_EN			0x0f30

/* camsys main registers */
#define CAM_MAIN_SW_RST_1		0x0058
#define CAM_MAIN_LOW_POWER_CTRL		0x0390
#define CAM_MAIN_CAM_SPM_ACK		0x042c
#define  CAM_MAIN_LP_MASK		GENMASK(2, 0)

/* command queue registers */
#define CAMSVCQTOP_SW_RST_CTL		0x0010
#define CAMSVCQ_CQ_EN			0x0100
#define  CQ_RESET			BIT(16)
#define CAMSVCQ_CQ_SUB_EN		0x0130
#define  CQ_SUB_RESET			BIT(16)

/* CAMSV image formats */
#define CAMSV_FMT_RAW8			0
#define CAMSV_FMT_RAW10			1
#define CAMSV_FMT_RAW12			2

#define CAMSV_TAG_IDX			0
#define CAMSV_MIN_BUFFERS		2
#define CAMSV_MAX_WIDTH			8192
#define CAMSV_MAX_HEIGHT		8192
#define CAMSV_STRIDE_ALIGN		16

struct mtk_camsv_format {
	u32 code;
	u32 fourcc;
	u8 hw_fmt;
	u8 bpp;		/* bytes per pixel in memory */
	bool unpack;
};

static const struct mtk_camsv_format camsv_formats[] = {
	{ MEDIA_BUS_FMT_SBGGR10_1X10, V4L2_PIX_FMT_SBGGR10, CAMSV_FMT_RAW10, 2, true },
	{ MEDIA_BUS_FMT_SGBRG10_1X10, V4L2_PIX_FMT_SGBRG10, CAMSV_FMT_RAW10, 2, true },
	{ MEDIA_BUS_FMT_SGRBG10_1X10, V4L2_PIX_FMT_SGRBG10, CAMSV_FMT_RAW10, 2, true },
	{ MEDIA_BUS_FMT_SRGGB10_1X10, V4L2_PIX_FMT_SRGGB10, CAMSV_FMT_RAW10, 2, true },
	{ MEDIA_BUS_FMT_SBGGR12_1X12, V4L2_PIX_FMT_SBGGR12, CAMSV_FMT_RAW12, 2, true },
	{ MEDIA_BUS_FMT_SGBRG12_1X12, V4L2_PIX_FMT_SGBRG12, CAMSV_FMT_RAW12, 2, true },
	{ MEDIA_BUS_FMT_SGRBG12_1X12, V4L2_PIX_FMT_SGRBG12, CAMSV_FMT_RAW12, 2, true },
	{ MEDIA_BUS_FMT_SRGGB12_1X12, V4L2_PIX_FMT_SRGGB12, CAMSV_FMT_RAW12, 2, true },
	{ MEDIA_BUS_FMT_SBGGR8_1X8, V4L2_PIX_FMT_SBGGR8, CAMSV_FMT_RAW8, 1, false },
	{ MEDIA_BUS_FMT_SGBRG8_1X8, V4L2_PIX_FMT_SGBRG8, CAMSV_FMT_RAW8, 1, false },
	{ MEDIA_BUS_FMT_SGRBG8_1X8, V4L2_PIX_FMT_SGRBG8, CAMSV_FMT_RAW8, 1, false },
	{ MEDIA_BUS_FMT_SRGGB8_1X8, V4L2_PIX_FMT_SRGGB8, CAMSV_FMT_RAW8, 1, false },
};

struct mtk_camsv_buffer {
	struct vb2_v4l2_buffer vbb;
	struct list_head list;
};

struct mtk_camsv {
	struct device *dev;
	void __iomem *base;
	void __iomem *base_inner;
	void __iomem *base_dma;
	void __iomem *base_scq;
	void __iomem *camsys;
	struct clk_bulk_data *clks;
	int num_clks;
	u32 id;
	u32 cammux_id;

	struct media_device mdev;
	struct v4l2_device v4l2_dev;
	struct v4l2_async_notifier notifier;
	unsigned int num_sources;
	/* the source of the running stream */
	struct v4l2_subdev *seninf;

	struct video_device vdev;
	struct media_pad pad;
	struct vb2_queue queue;
	struct mutex lock;		/* serializes the video node */
	struct v4l2_pix_format fmt;
	const struct mtk_camsv_format *cfmt;

	spinlock_t buf_lock;		/* protects the buffer state below */
	struct list_head queued;	/* waiting to be handed to the hardware */
	struct mtk_camsv_buffer *pending; /* in the shadow registers */
	struct mtk_camsv_buffer *active;  /* being written by the hardware */
	u32 sequence;
	bool streaming;
};

static inline struct mtk_camsv_buffer *to_camsv_buf(struct vb2_v4l2_buffer *v)
{
	return container_of(v, struct mtk_camsv_buffer, vbb);
}

static void camsv_rmw(void __iomem *reg, u32 mask, u32 val)
{
	writel((readl(reg) & ~mask) | (val & mask), reg);
}

static const struct mtk_camsv_format *camsv_find_fourcc(u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(camsv_formats); i++)
		if (camsv_formats[i].fourcc == fourcc)
			return &camsv_formats[i];

	return NULL;
}

static const struct mtk_camsv_format *camsv_find_code(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(camsv_formats); i++)
		if (camsv_formats[i].code == code)
			return &camsv_formats[i];

	return NULL;
}

static void camsv_fill_fmt(struct v4l2_pix_format *pix,
			   const struct mtk_camsv_format *cfmt)
{
	pix->width = clamp(pix->width, 16U, (u32)CAMSV_MAX_WIDTH);
	pix->height = clamp(pix->height, 16U, (u32)CAMSV_MAX_HEIGHT);
	pix->width = ALIGN_DOWN(pix->width, 2);
	pix->height = ALIGN_DOWN(pix->height, 2);
	pix->pixelformat = cfmt->fourcc;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = ALIGN(pix->width * cfmt->bpp, CAMSV_STRIDE_ALIGN);
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->colorspace = V4L2_COLORSPACE_RAW;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	pix->quantization = V4L2_QUANTIZATION_DEFAULT;
	pix->xfer_func = V4L2_XFER_FUNC_NONE;
}

/* ---------------------------------------------------------------------------
 * Hardware
 */

static int camsv_reset(struct mtk_camsv *sv)
{
	u32 val;
	int ret;

	writel(0, sv->base_dma + CAMSVDMA_SW_RST_CTL);
	writel(DMA_SW_RST, sv->base_dma + CAMSVDMA_SW_RST_CTL);
	ret = readl_poll_timeout_atomic(sv->base_dma + CAMSVDMA_SW_RST_CTL, val,
					val & DMA_SW_RST_ST, 1, 100000);
	if (ret) {
		dev_err(sv->dev, "DMA reset timed out\n");
		return ret;
	}

	/* the DMA clock gating may only run while the DMA is idle */
	writel(0, sv->base + CAMSV_DCM_DIS);

	writel(0, sv->base + CAMSV_SW_CTL);
	writel(1, sv->base + CAMSV_SW_CTL);
	writel(0, sv->base_dma + CAMSVDMA_SW_RST_CTL);
	writel(0, sv->base + CAMSV_SW_CTL);

	writel(0, sv->base_scq + CAMSVCQTOP_SW_RST_CTL);
	writel(1, sv->base_scq + CAMSVCQTOP_SW_RST_CTL);
	ret = readl_poll_timeout_atomic(sv->base_scq + CAMSVCQTOP_SW_RST_CTL,
					val, val & BIT(1), 1, 100000);
	if (ret) {
		dev_err(sv->dev, "CQ reset timed out\n");
		return ret;
	}
	writel(0, sv->base_scq + CAMSVCQTOP_SW_RST_CTL);

	/* reset the CAMSV from the camsys top as well */
	writel(0, sv->camsys + CAM_MAIN_SW_RST_1);
	writel(3 << (sv->id * 2), sv->camsys + CAM_MAIN_SW_RST_1);
	writel(0, sv->camsys + CAM_MAIN_SW_RST_1);

	camsv_rmw(sv->base_scq + CAMSVCQ_CQ_EN, CQ_RESET, CQ_RESET);
	camsv_rmw(sv->base_scq + CAMSVCQ_CQ_SUB_EN, CQ_SUB_RESET, CQ_SUB_RESET);
	camsv_rmw(sv->base_scq + CAMSVCQ_CQ_SUB_EN, CQ_SUB_RESET, 0);
	camsv_rmw(sv->base_scq + CAMSVCQ_CQ_EN, CQ_RESET, 0);

	return 0;
}

/* load the shadowed registers now */
static void camsv_toggle_db(struct mtk_camsv *sv)
{
	camsv_rmw(sv->base + CAMSV_MODULE_DB, MODULE_DB_CAM_DB_EN, 0);
	camsv_rmw(sv->base + CAMSV_MODULE_DB, MODULE_DB_CAM_DB_EN,
		  MODULE_DB_CAM_DB_EN);
}

static void camsv_toggle_tg_db(struct mtk_camsv *sv)
{
	camsv_rmw(sv->base + CAMSV_PATH_CFG, PATH_CFG_SYNC_VF_EN_DB_LOAD_DIS,
		  PATH_CFG_SYNC_VF_EN_DB_LOAD_DIS);
	camsv_rmw(sv->base + CAMSV_PATH_CFG, PATH_CFG_SYNC_VF_EN_DB_LOAD_DIS, 0);
}

/* FIFO thresholds of the MT6878 CAMSV with the 352 entry image FIFO */
static void camsv_dma_config(struct mtk_camsv *sv)
{
	writel(0x80d800b4, sv->base_dma + CAMSVDMA_CON3_IMG);
	writel(0x1090006c, sv->base_dma + CAMSVDMA_CON2_IMG);
	writel(0x10480024, sv->base_dma + CAMSVDMA_CON1_IMG);
	writel(0x80240000, sv->base_dma + CAMSVDMA_CON4_IMG);
	writel(0xfff, sv->base_dma + CAMSVDMA_STG1_EN);
	writel(0xfff, sv->base_dma + CAMSVDMA_STG1_INIT);
}

static void camsv_set_addr(struct mtk_camsv *sv, struct mtk_camsv_buffer *buf)
{
	dma_addr_t addr = vb2_dma_contig_plane_dma_addr(&buf->vbb.vb2_buf, 0);
	const unsigned int t = CAMSV_TAG_IDX;

	writel(lower_32_bits(addr),
	       sv->base_dma + CAMSV_TAG(CAMSVDMA_BASE_ADDR, t));
	writel(upper_32_bits(addr) & 0xf,
	       sv->base_dma + CAMSV_TAG(CAMSVDMA_BASE_ADDR_MSB, t));
	writel(sv->sequence, sv->base + CAMSV_TAG(CAMSV_FH_SPARE, t));
	writel(FBC0_FBC_EN | FBC0_RCNT_INC, sv->base + CAMSV_TAG(CAMSV_FBC0, t));
}

static int camsv_hw_start(struct mtk_camsv *sv)
{
	const unsigned int t = CAMSV_TAG_IDX;
	const struct mtk_camsv_format *cfmt = sv->cfmt;
	u32 w = sv->fmt.width, h = sv->fmt.height, fmt;
	int ret;

	ret = camsv_reset(sv);
	if (ret)
		return ret;

	camsv_dma_config(sv);

	/* shadowing off: the setup below takes effect at once */
	camsv_rmw(sv->base + CAMSV_MODULE_DB, MODULE_DB_CAM_DB_EN, 0);
	camsv_rmw(sv->base + CAMSV_PATH_CFG, PATH_CFG_SYNC_VF_EN_DB_LOAD_DIS, 0);

	writel(TIME_STAMP_SEL | TIME_STP_EN, sv->base + CAMSV_TIME_STAMP_CTL);
	writel(ERR_CTL_GRAB_ERR_EN, sv->base + CAMSV_ERR_CTL);
	writel(0, sv->base + CAMSV_SUB_PERIOD);
	writel(0, sv->base + CAMSV_DCIF_SET);
	writel(0, sv->base + CAMSV_DCIF_SEL);

	/* one group with one tag */
	writel(BIT(t), sv->base + CAMSV_GROUP_TAG(0));
	writel(0, sv->base + CAMSV_GROUP_TAG(1));
	writel(0, sv->base + CAMSV_GROUP_TAG(2));
	writel(0, sv->base + CAMSV_GROUP_TAG(3));
	writel(BIT(t), sv->base + CAMSV_FIRST_TAG);
	writel(BIT(t), sv->base + CAMSV_LAST_TAG);

	/* tag: grab window, format, frame buffer control */
	writel((w << 16), sv->base + CAMSV_TAG(CAMSV_GRAB_PXL, t));
	writel((h << 16), sv->base + CAMSV_TAG(CAMSV_GRAB_LIN, t));
	fmt = FIELD_PREP(FORMAT_FMT, cfmt->hw_fmt);
	if (cfmt->unpack)
		fmt |= FORMAT_UNPACK;
	writel(fmt, sv->base + CAMSV_TAG(CAMSV_FORMAT, t));
	writel(1, sv->base + CAMSV_TAG(CAMSV_CONFIG, t));
	writel(FBC0_FBC_EN, sv->base + CAMSV_TAG(CAMSV_FBC0, t));

	/* tag: image DMA */
	writel(FIELD_PREP(WDMA_STRIDE, sv->fmt.bytesperline) |
	       FIELD_PREP(WDMA_MAX_BURST, 0x10),
	       sv->base_dma + CAMSV_TAG(CAMSVDMA_WDMA_BASIC, t));
	writel(0x10, sv->base_dma + CAMSV_TAG(CAMSVDMA_LEN_BASIC, t));
	writel(0, sv->base_dma + CAMSV_TAG(CAMSVDMA_FH_EN, t));

	writel(BIT(t), sv->base + CAMSV_DMA_EN_IMG);
	writel(BIT(t), sv->base + CAMSV_DMA_EN_LEN);
	writel(BIT(t) | BIT(t) << 8, sv->base_dma + CAMSVDMA_TAG_EN);

	/* interrupts */
	writel(DONE_ST_GROUP(0) | DONE_ST_GROUP(1) | DONE_ST_GROUP(2) |
	       DONE_ST_GROUP(3), sv->base + CAMSV_DONE_STATUS_EN);
	writel(BIT(0) | (0x700 << (t * 3)), sv->base + CAMSV_ERR_STATUS_EN);
	writel(SOF_ST_TAG(t), sv->base + CAMSV_SOF_STATUS_EN);
	writel(0x11111111, sv->base + CAMSV_TAG_CHANNEL_EN);

	/* the first buffer, loaded at the first start of frame */
	spin_lock_irq(&sv->buf_lock);
	sv->pending = list_first_entry_or_null(&sv->queued,
					       struct mtk_camsv_buffer, list);
	if (sv->pending) {
		list_del(&sv->pending->list);
		camsv_set_addr(sv, sv->pending);
	}
	sv->streaming = true;
	spin_unlock_irq(&sv->buf_lock);

	camsv_toggle_db(sv);

	camsv_rmw(sv->base + CAMSV_SEN_MODE, SEN_MODE_CMOS_EN, SEN_MODE_CMOS_EN);
	camsv_rmw(sv->base + CAMSV_VF_CON, VF_CON_VFDATA_EN, VF_CON_VFDATA_EN);
	camsv_toggle_tg_db(sv);

	dev_dbg(sv->dev, "started %ux%u fmt 0x%x stride %u\n", w, h, fmt,
		sv->fmt.bytesperline);

	return 0;
}

static void camsv_hw_stop(struct mtk_camsv *sv)
{
	writel(1, sv->base + CAMSV_DCM_DIS);

	writel(0, sv->base + CAMSV_DONE_STATUS_EN);
	writel(0, sv->base + CAMSV_ERR_STATUS_EN);
	writel(0, sv->base + CAMSV_SOF_STATUS_EN);

	camsv_rmw(sv->base + CAMSV_SEN_MODE, SEN_MODE_TG_MODE_OFF,
		  SEN_MODE_TG_MODE_OFF);
	camsv_rmw(sv->base + CAMSV_VF_CON, VF_CON_VFDATA_EN, 0);
	camsv_toggle_tg_db(sv);
	camsv_rmw(sv->base + CAMSV_SEN_MODE, SEN_MODE_CMOS_EN, 0);

	camsv_reset(sv);

	writel(0, sv->base + CAMSV_TAG(CAMSV_FBC0, CAMSV_TAG_IDX));
	writel(0, sv->base + CAMSV_DMA_EN_IMG);
	writel(0, sv->base + CAMSV_DCIF_SET);
	writel(0, sv->base + CAMSV_DCIF_SEL);
	camsv_toggle_db(sv);
	camsv_rmw(sv->base + CAMSV_SEN_MODE, SEN_MODE_TG_MODE_OFF, 0);
}

static irqreturn_t camsv_irq_sof(int irq, void *data)
{
	struct mtk_camsv *sv = data;
	struct mtk_camsv_buffer *next;
	u32 status;

	status = readl(sv->base + CAMSV_SOF_STATUS);
	if (!(status & SOF_ST_TAG(CAMSV_TAG_IDX)))
		return status ? IRQ_HANDLED : IRQ_NONE;

	spin_lock(&sv->buf_lock);
	if (!sv->streaming)
		goto out;

	/* the shadowed address has just been loaded */
	if (sv->pending) {
		if (sv->active) {
			/* no frame done for the previous buffer: requeue it */
			list_add(&sv->active->list, &sv->queued);
		}
		sv->active = sv->pending;
		sv->pending = NULL;
	}

	next = list_first_entry_or_null(&sv->queued, struct mtk_camsv_buffer,
					list);
	if (next) {
		list_del(&next->list);
		sv->pending = next;
		camsv_set_addr(sv, next);
	}
out:
	spin_unlock(&sv->buf_lock);

	return IRQ_HANDLED;
}

static irqreturn_t camsv_irq_done(int irq, void *data)
{
	struct mtk_camsv *sv = data;
	struct mtk_camsv_buffer *buf;
	u32 status;

	status = readl(sv->base + CAMSV_DONE_STATUS);
	if (!(status & DONE_ST_GROUP(0)))
		return status ? IRQ_HANDLED : IRQ_NONE;

	spin_lock(&sv->buf_lock);
	buf = sv->active;
	sv->active = NULL;
	if (buf) {
		buf->vbb.vb2_buf.timestamp = ktime_get_ns();
		buf->vbb.sequence = sv->sequence;
		buf->vbb.field = V4L2_FIELD_NONE;
		vb2_buffer_done(&buf->vbb.vb2_buf, VB2_BUF_STATE_DONE);
	}
	sv->sequence++;
	spin_unlock(&sv->buf_lock);

	return IRQ_HANDLED;
}

static irqreturn_t camsv_irq_err(int irq, void *data)
{
	struct mtk_camsv *sv = data;
	u32 status;

	status = readl(sv->base_inner + CAMSV_ERR_STATUS);
	if (!status)
		return IRQ_NONE;

	dev_err_ratelimited(sv->dev,
			    "error status 0x%08x fbc 0x%08x frame size 0x%08x\n",
			    status,
			    readl(sv->base_inner + CAMSV_TAG(CAMSV_FBC1, 0)),
			    readl(sv->base + 0x0188));

	return IRQ_HANDLED;
}

/* ---------------------------------------------------------------------------
 * vb2
 */

static int camsv_queue_setup(struct vb2_queue *vq, unsigned int *num_buffers,
			     unsigned int *num_planes, unsigned int sizes[],
			     struct device *alloc_devs[])
{
	struct mtk_camsv *sv = vb2_get_drv_priv(vq);

	if (*num_planes)
		return sizes[0] < sv->fmt.sizeimage ? -EINVAL : 0;

	*num_planes = 1;
	sizes[0] = sv->fmt.sizeimage;

	return 0;
}

static int camsv_buf_prepare(struct vb2_buffer *vb)
{
	struct mtk_camsv *sv = vb2_get_drv_priv(vb->vb2_queue);

	if (vb2_plane_size(vb, 0) < sv->fmt.sizeimage)
		return -EINVAL;

	vb2_set_plane_payload(vb, 0, sv->fmt.sizeimage);

	return 0;
}

static void camsv_buf_queue(struct vb2_buffer *vb)
{
	struct mtk_camsv *sv = vb2_get_drv_priv(vb->vb2_queue);
	struct mtk_camsv_buffer *buf = to_camsv_buf(to_vb2_v4l2_buffer(vb));
	unsigned long flags;

	spin_lock_irqsave(&sv->buf_lock, flags);
	list_add_tail(&buf->list, &sv->queued);
	spin_unlock_irqrestore(&sv->buf_lock, flags);
}

static void camsv_return_all(struct mtk_camsv *sv, enum vb2_buffer_state state)
{
	struct mtk_camsv_buffer *buf, *tmp;
	unsigned long flags;

	spin_lock_irqsave(&sv->buf_lock, flags);
	if (sv->active)
		list_add_tail(&sv->active->list, &sv->queued);
	if (sv->pending)
		list_add_tail(&sv->pending->list, &sv->queued);
	sv->active = NULL;
	sv->pending = NULL;
	list_for_each_entry_safe(buf, tmp, &sv->queued, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vbb.vb2_buf, state);
	}
	spin_unlock_irqrestore(&sv->buf_lock, flags);
}

/* the seninf source and the video node must agree on the format */
static int camsv_check_format(struct mtk_camsv *sv)
{
	struct v4l2_subdev_format sfmt = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.pad = PAD_SRC_RAW0,
	};
	const struct mtk_camsv_format *cfmt;
	int ret;

	ret = v4l2_subdev_call(sv->seninf, pad, get_fmt, NULL, &sfmt);
	if (ret)
		return ret;

	cfmt = camsv_find_code(sfmt.format.code);
	if (!cfmt || cfmt->fourcc != sv->fmt.pixelformat ||
	    sfmt.format.width != sv->fmt.width ||
	    sfmt.format.height != sv->fmt.height) {
		dev_dbg(sv->dev,
			"format mismatch: seninf 0x%x %ux%u, video %p4cc %ux%u\n",
			sfmt.format.code, sfmt.format.width,
			sfmt.format.height, &sv->fmt.pixelformat,
			sv->fmt.width, sv->fmt.height);
		return -EPIPE;
	}

	sv->cfmt = cfmt;

	return 0;
}

static int camsv_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct mtk_camsv *sv = vb2_get_drv_priv(vq);
	struct media_pad *remote;
	int ret;

	ret = video_device_pipeline_alloc_start(&sv->vdev);
	if (ret)
		goto err_return;

	/* stream from the seninf port whose link is enabled */
	remote = media_pad_remote_pad_unique(&sv->pad);
	if (IS_ERR(remote)) {
		ret = PTR_ERR(remote);
		goto err_pipeline;
	}
	sv->seninf = media_entity_to_v4l2_subdev(remote->entity);

	ret = camsv_check_format(sv);
	if (ret)
		goto err_pipeline;

	ret = pm_runtime_resume_and_get(sv->dev);
	if (ret)
		goto err_pipeline;

	sv->sequence = 0;
	ret = camsv_hw_start(sv);
	if (ret)
		goto err_pm;

	ret = mtk_cam_seninf_set_camtg(sv->seninf, PAD_SRC_RAW0, sv->cammux_id);
	if (ret)
		goto err_hw;

	ret = v4l2_subdev_call(sv->seninf, video, s_stream, 1);
	if (ret)
		goto err_camtg;

	return 0;

err_camtg:
	mtk_cam_seninf_set_camtg(sv->seninf, PAD_SRC_RAW0, 0xff);
err_hw:
	spin_lock_irq(&sv->buf_lock);
	sv->streaming = false;
	spin_unlock_irq(&sv->buf_lock);
	camsv_hw_stop(sv);
err_pm:
	pm_runtime_put(sv->dev);
err_pipeline:
	video_device_pipeline_stop(&sv->vdev);
err_return:
	camsv_return_all(sv, VB2_BUF_STATE_QUEUED);
	return ret;
}

static void camsv_stop_streaming(struct vb2_queue *vq)
{
	struct mtk_camsv *sv = vb2_get_drv_priv(vq);

	v4l2_subdev_call(sv->seninf, video, s_stream, 0);
	mtk_cam_seninf_set_camtg(sv->seninf, PAD_SRC_RAW0, 0xff);

	spin_lock_irq(&sv->buf_lock);
	sv->streaming = false;
	spin_unlock_irq(&sv->buf_lock);

	camsv_hw_stop(sv);
	pm_runtime_put(sv->dev);
	video_device_pipeline_stop(&sv->vdev);

	camsv_return_all(sv, VB2_BUF_STATE_ERROR);
}

static const struct vb2_ops camsv_vb2_ops = {
	.queue_setup = camsv_queue_setup,
	.buf_prepare = camsv_buf_prepare,
	.buf_queue = camsv_buf_queue,
	.start_streaming = camsv_start_streaming,
	.stop_streaming = camsv_stop_streaming,
};

/* ---------------------------------------------------------------------------
 * Video node
 */

static int camsv_querycap(struct file *file, void *fh,
			  struct v4l2_capability *cap)
{
	strscpy(cap->driver, "mtk-camsv", sizeof(cap->driver));
	strscpy(cap->card, "MediaTek CAMSV", sizeof(cap->card));

	return 0;
}

static int camsv_enum_fmt(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	unsigned int i, n = 0;

	for (i = 0; i < ARRAY_SIZE(camsv_formats); i++) {
		if (f->mbus_code && camsv_formats[i].code != f->mbus_code)
			continue;
		if (n++ == f->index) {
			f->pixelformat = camsv_formats[i].fourcc;
			return 0;
		}
	}

	return -EINVAL;
}

static int camsv_g_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct mtk_camsv *sv = video_drvdata(file);

	f->fmt.pix = sv->fmt;

	return 0;
}

static int camsv_try_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	const struct mtk_camsv_format *cfmt;

	cfmt = camsv_find_fourcc(f->fmt.pix.pixelformat);
	if (!cfmt)
		cfmt = &camsv_formats[0];
	camsv_fill_fmt(&f->fmt.pix, cfmt);

	return 0;
}

static int camsv_s_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct mtk_camsv *sv = video_drvdata(file);
	int ret;

	if (vb2_is_busy(&sv->queue))
		return -EBUSY;

	ret = camsv_try_fmt(file, fh, f);
	if (ret)
		return ret;

	sv->fmt = f->fmt.pix;
	sv->cfmt = camsv_find_fourcc(sv->fmt.pixelformat);

	return 0;
}

static int camsv_enum_framesizes(struct file *file, void *fh,
				 struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index || !camsv_find_fourcc(fsize->pixel_format))
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise.min_width = 16;
	fsize->stepwise.max_width = CAMSV_MAX_WIDTH;
	fsize->stepwise.step_width = 2;
	fsize->stepwise.min_height = 16;
	fsize->stepwise.max_height = CAMSV_MAX_HEIGHT;
	fsize->stepwise.step_height = 2;

	return 0;
}

static const struct v4l2_ioctl_ops camsv_ioctl_ops = {
	.vidioc_querycap = camsv_querycap,
	.vidioc_enum_fmt_vid_cap = camsv_enum_fmt,
	.vidioc_g_fmt_vid_cap = camsv_g_fmt,
	.vidioc_s_fmt_vid_cap = camsv_s_fmt,
	.vidioc_try_fmt_vid_cap = camsv_try_fmt,
	.vidioc_enum_framesizes = camsv_enum_framesizes,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
};

static const struct v4l2_file_operations camsv_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.unlocked_ioctl = video_ioctl2,
	.mmap = vb2_fop_mmap,
	.poll = vb2_fop_poll,
};

static int camsv_link_validate(struct media_link *link)
{
	/* the format is checked against seninf when streaming starts */
	return 0;
}

/* only one seninf port at a time can feed the cam mux */
static int camsv_link_setup(struct media_entity *entity,
			    const struct media_pad *local,
			    const struct media_pad *remote, u32 flags)
{
	struct media_link *link;

	if (!(flags & MEDIA_LNK_FL_ENABLED))
		return 0;

	for_each_media_entity_data_link(entity, link) {
		if (link->sink == local && link->source != remote &&
		    (link->flags & MEDIA_LNK_FL_ENABLED))
			return -EBUSY;
	}

	return 0;
}

static const struct media_entity_operations camsv_entity_ops = {
	.link_setup = camsv_link_setup,
	.link_validate = camsv_link_validate,
};

static int camsv_register_video(struct mtk_camsv *sv)
{
	struct video_device *vdev = &sv->vdev;
	struct vb2_queue *q = &sv->queue;
	int ret;

	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_DMABUF;
	q->drv_priv = sv;
	q->buf_struct_size = sizeof(struct mtk_camsv_buffer);
	q->ops = &camsv_vb2_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->min_queued_buffers = CAMSV_MIN_BUFFERS;
	q->lock = &sv->lock;
	q->dev = sv->dev;
	ret = vb2_queue_init(q);
	if (ret)
		return ret;

	sv->pad.flags = MEDIA_PAD_FL_SINK | MEDIA_PAD_FL_MUST_CONNECT;
	vdev->entity.ops = &camsv_entity_ops;
	ret = media_entity_pads_init(&vdev->entity, 1, &sv->pad);
	if (ret)
		return ret;

	snprintf(vdev->name, sizeof(vdev->name), "mtk-camsv%u", sv->id);
	vdev->fops = &camsv_fops;
	vdev->ioctl_ops = &camsv_ioctl_ops;
	vdev->release = video_device_release_empty;
	vdev->v4l2_dev = &sv->v4l2_dev;
	vdev->queue = q;
	vdev->lock = &sv->lock;
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING |
			    V4L2_CAP_IO_MC;
	vdev->vfl_dir = VFL_DIR_RX;
	video_set_drvdata(vdev, sv);

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		media_entity_cleanup(&vdev->entity);

	return ret;
}

/* ---------------------------------------------------------------------------
 * seninf binding
 */

static int camsv_notifier_bound(struct v4l2_async_notifier *notifier,
				struct v4l2_subdev *sd,
				struct v4l2_async_connection *asc)
{
	struct mtk_camsv *sv = container_of(notifier, struct mtk_camsv,
					    notifier);
	u32 flags = 0;

	/* the first port to bind is the default source */
	if (!sv->num_sources++)
		flags = MEDIA_LNK_FL_ENABLED;

	return media_create_pad_link(&sd->entity, PAD_SRC_RAW0,
				     &sv->vdev.entity, 0, flags);
}

static int camsv_notifier_complete(struct v4l2_async_notifier *notifier)
{
	struct mtk_camsv *sv = container_of(notifier, struct mtk_camsv,
					    notifier);
	int ret;

	ret = v4l2_device_register_subdev_nodes(&sv->v4l2_dev);
	if (ret)
		return ret;

	return media_device_register(&sv->mdev);
}

static const struct v4l2_async_notifier_operations camsv_notifier_ops = {
	.bound = camsv_notifier_bound,
	.complete = camsv_notifier_complete,
};

static int camsv_register_notifier(struct mtk_camsv *sv)
{
	struct v4l2_async_connection *asc;
	struct device_node *np;
	unsigned int i;
	int ret;

	v4l2_async_nf_init(&sv->notifier, &sv->v4l2_dev);

	/* every seninf port this camsv can be fed from */
	for (i = 0; (np = of_parse_phandle(sv->dev->of_node,
					   "mediatek,seninf", i)); i++) {
		asc = v4l2_async_nf_add_fwnode(&sv->notifier,
					       of_fwnode_handle(np),
					       struct v4l2_async_connection);
		of_node_put(np);
		if (IS_ERR(asc)) {
			v4l2_async_nf_cleanup(&sv->notifier);
			return PTR_ERR(asc);
		}
	}

	if (!i) {
		v4l2_async_nf_cleanup(&sv->notifier);
		return dev_err_probe(sv->dev, -EINVAL, "no seninf phandle\n");
	}

	sv->notifier.ops = &camsv_notifier_ops;
	ret = v4l2_async_nf_register(&sv->notifier);
	if (ret)
		v4l2_async_nf_cleanup(&sv->notifier);

	return ret;
}

/* ---------------------------------------------------------------------------
 * Platform driver
 */

static const char * const camsv_irq_names[] = { "done", "err", "sof" };
static const irq_handler_t camsv_irq_handlers[] = {
	camsv_irq_done, camsv_irq_err, camsv_irq_sof,
};

static void camsv_unlink_supplier(void *data)
{
	struct device_link *link = data;
	struct device *supplier = link->supplier;

	device_link_del(link);
	put_device(supplier);
}

/* Keep a device runtime resumed for as long as the CAMSV is */
static int camsv_link_supplier(struct mtk_camsv *sv, const char *prop)
{
	struct platform_device *pdev;
	struct device_link *link;
	struct device_node *np;

	np = of_parse_phandle(sv->dev->of_node, prop, 0);
	if (!np)
		return dev_err_probe(sv->dev, -EINVAL, "no %s\n", prop);

	pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!pdev)
		return -EPROBE_DEFER;

	if (!platform_get_drvdata(pdev)) {
		put_device(&pdev->dev);
		return -EPROBE_DEFER;
	}

	link = device_link_add(sv->dev, &pdev->dev,
			       DL_FLAG_PM_RUNTIME | DL_FLAG_STATELESS);
	if (!link) {
		put_device(&pdev->dev);
		return dev_err_probe(sv->dev, -ENODEV, "failed to link %s\n",
				     prop);
	}

	return devm_add_action_or_reset(sv->dev, camsv_unlink_supplier, link);
}

static int camsv_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_camsv *sv;
	struct resource *res;
	unsigned int i;
	int irq, ret;

	sv = devm_kzalloc(dev, sizeof(*sv), GFP_KERNEL);
	if (!sv)
		return -ENOMEM;

	sv->dev = dev;
	mutex_init(&sv->lock);
	spin_lock_init(&sv->buf_lock);
	INIT_LIST_HEAD(&sv->queued);
	platform_set_drvdata(pdev, sv);

	sv->base = devm_platform_ioremap_resource_byname(pdev, "base");
	if (IS_ERR(sv->base))
		return PTR_ERR(sv->base);
	sv->base_inner = devm_platform_ioremap_resource_byname(pdev, "inner");
	if (IS_ERR(sv->base_inner))
		return PTR_ERR(sv->base_inner);
	sv->base_dma = devm_platform_ioremap_resource_byname(pdev, "dma");
	if (IS_ERR(sv->base_dma))
		return PTR_ERR(sv->base_dma);
	sv->base_scq = devm_platform_ioremap_resource_byname(pdev, "scq");
	if (IS_ERR(sv->base_scq))
		return PTR_ERR(sv->base_scq);

	/* shared with the camsys clock controller, so not requested */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "camsys");
	if (!res)
		return dev_err_probe(dev, -EINVAL, "no camsys registers\n");
	sv->camsys = devm_ioremap(dev, res->start, resource_size(res));
	if (!sv->camsys)
		return -ENOMEM;

	ret = of_property_read_u32(dev->of_node, "mediatek,camsv-id", &sv->id);
	if (ret)
		return dev_err_probe(dev, ret, "no camsv id\n");
	ret = of_property_read_u32(dev->of_node, "mediatek,cammux-id",
				   &sv->cammux_id);
	if (ret)
		return dev_err_probe(dev, ret, "no cam mux id\n");

	sv->num_clks = devm_clk_bulk_get_all(dev, &sv->clks);
	if (sv->num_clks < 0)
		return dev_err_probe(dev, sv->num_clks, "failed to get clocks\n");

	/*
	 * The image DMA reaches the DRAM through the multimedia SMI common,
	 * which sits in the display power domain, and the multimedia
	 * infrastructure only asks the SPM for the DRAM while its GCE does.
	 * Without both the writes are dropped whenever the display is off.
	 */
	ret = camsv_link_supplier(sv, "mediatek,smi");
	if (ret)
		return ret;
	ret = camsv_link_supplier(sv, "mediatek,gce");
	if (ret)
		return ret;

	/* the SMI path is not translated yet: physical addresses only */
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(camsv_irq_names); i++) {
		irq = platform_get_irq_byname(pdev, camsv_irq_names[i]);
		if (irq < 0)
			return irq;
		ret = devm_request_irq(dev, irq, camsv_irq_handlers[i], 0,
				       dev_name(dev), sv);
		if (ret)
			return dev_err_probe(dev, ret, "failed to request irq\n");
	}

	pm_runtime_enable(dev);

	sv->mdev.dev = dev;
	strscpy(sv->mdev.model, "mtk-camsv", sizeof(sv->mdev.model));
	media_device_init(&sv->mdev);
	sv->v4l2_dev.mdev = &sv->mdev;

	ret = v4l2_device_register(dev, &sv->v4l2_dev);
	if (ret)
		goto err_media;

	sv->fmt.width = 1600;
	sv->fmt.height = 1200;
	camsv_fill_fmt(&sv->fmt, &camsv_formats[0]);
	sv->cfmt = &camsv_formats[0];

	ret = camsv_register_video(sv);
	if (ret)
		goto err_v4l2;

	ret = camsv_register_notifier(sv);
	if (ret)
		goto err_video;

	return 0;

err_video:
	video_unregister_device(&sv->vdev);
	media_entity_cleanup(&sv->vdev.entity);
err_v4l2:
	v4l2_device_unregister(&sv->v4l2_dev);
err_media:
	media_device_cleanup(&sv->mdev);
	pm_runtime_disable(dev);
	return ret;
}

static void camsv_remove(struct platform_device *pdev)
{
	struct mtk_camsv *sv = platform_get_drvdata(pdev);

	media_device_unregister(&sv->mdev);
	v4l2_async_nf_unregister(&sv->notifier);
	v4l2_async_nf_cleanup(&sv->notifier);
	video_unregister_device(&sv->vdev);
	media_entity_cleanup(&sv->vdev.entity);
	v4l2_device_unregister(&sv->v4l2_dev);
	media_device_cleanup(&sv->mdev);
	pm_runtime_disable(&pdev->dev);
}

static int camsv_runtime_suspend(struct device *dev)
{
	struct mtk_camsv *sv = dev_get_drvdata(dev);

	writel(0, sv->camsys + CAM_MAIN_LOW_POWER_CTRL);
	clk_bulk_disable_unprepare(sv->num_clks, sv->clks);

	return 0;
}

static int camsv_runtime_resume(struct device *dev)
{
	struct mtk_camsv *sv = dev_get_drvdata(dev);

	u32 ack;
	int ret;

	ret = clk_bulk_prepare_enable(sv->num_clks, sv->clks);
	if (ret)
		return ret;

	/* let the camsys request its bus and memory resources from the SPM */
	writel(CAM_MAIN_LP_MASK, sv->camsys + CAM_MAIN_LOW_POWER_CTRL);
	ret = readl_poll_timeout(sv->camsys + CAM_MAIN_CAM_SPM_ACK, ack,
				 (ack & CAM_MAIN_LP_MASK) == CAM_MAIN_LP_MASK,
				 1, 2000);
	if (ret)
		dev_warn(dev, "no SPM ack: 0x%08x\n", ack);

	return 0;
}

static const struct dev_pm_ops camsv_pm_ops = {
	RUNTIME_PM_OPS(camsv_runtime_suspend, camsv_runtime_resume, NULL)
};

static const struct of_device_id camsv_of_match[] = {
	{ .compatible = "mediatek,mt6878-camsv" },
	{ }
};
MODULE_DEVICE_TABLE(of, camsv_of_match);

static struct platform_driver camsv_driver = {
	.probe = camsv_probe,
	.remove = camsv_remove,
	.driver = {
		.name = "mtk-camsv",
		.of_match_table = camsv_of_match,
		.pm = pm_ptr(&camsv_pm_ops),
	},
};
module_platform_driver(camsv_driver);

MODULE_AUTHOR("Jan Benedikt <karagos01@gmail.com>");
MODULE_DESCRIPTION("MediaTek ISP7.x CAMSV raw capture driver");
MODULE_LICENSE("GPL");
