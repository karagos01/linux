// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek MT6878 audio front-end (AFE) platform driver
 *
 * Copyright (c) 2026 Jan Benedikt <karagos01@gmail.com>
 *
 * This is the first stage of the MT6878 AFE bring-up: it sets up the AFE
 * MMIO regmap, the downlink (playback) memif DMA interfaces, the MCU
 * interrupts and the frontend PCM DAIs, built on the generic MediaTek AFE
 * framework (sound/soc/mediatek/common/). It follows the MT8196 sibling,
 * which shares the MT6878 register generation.
 *
 * Not yet wired up: the audio clocks (mt6878_init_clock() and the runtime-PM
 * helpers are stubs for now; the real clock bring-up lands in a follow-up
 * mt6878-afe-clk.c), so the AFE does not run on hardware with this change
 * alone. The ADDA/I2S backend DAIs and the uplink memifs follow too.
 */

#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>

#include <sound/pcm.h>
#include <sound/soc.h>

#include "mt6878-afe-common.h"

#include "../common/mtk-afe-fe-dai.h"
#include "../common/mtk-afe-platform-driver.h"

static const struct snd_pcm_hardware mt6878_afe_hardware = {
	.info = (SNDRV_PCM_INFO_MMAP |
		 SNDRV_PCM_INFO_NO_PERIOD_WAKEUP |
		 SNDRV_PCM_INFO_INTERLEAVED |
		 SNDRV_PCM_INFO_MMAP_VALID),
	.formats = (SNDRV_PCM_FMTBIT_S16_LE |
		    SNDRV_PCM_FMTBIT_S24_LE |
		    SNDRV_PCM_FMTBIT_S32_LE),
	.period_bytes_min = 96,
	.period_bytes_max = 4 * 48 * 1024,
	.periods_min = 2,
	.periods_max = 256,
	.buffer_bytes_max = 256 * 1024,
	.fifo_size = 0,
};

static unsigned int mt6878_rate_transform(struct device *dev, unsigned int rate)
{
	switch (rate) {
	case 8000:
		return MTK_AFE_IPM2P0_RATE_8K;
	case 11025:
		return MTK_AFE_IPM2P0_RATE_11K;
	case 12000:
		return MTK_AFE_IPM2P0_RATE_12K;
	case 16000:
		return MTK_AFE_IPM2P0_RATE_16K;
	case 22050:
		return MTK_AFE_IPM2P0_RATE_22K;
	case 24000:
		return MTK_AFE_IPM2P0_RATE_24K;
	case 32000:
		return MTK_AFE_IPM2P0_RATE_32K;
	case 44100:
		return MTK_AFE_IPM2P0_RATE_44K;
	case 48000:
		return MTK_AFE_IPM2P0_RATE_48K;
	case 88200:
		return MTK_AFE_IPM2P0_RATE_88K;
	case 96000:
		return MTK_AFE_IPM2P0_RATE_96K;
	case 176400:
		return MTK_AFE_IPM2P0_RATE_176K;
	case 192000:
		return MTK_AFE_IPM2P0_RATE_192K;
	case 352800:
		return MTK_AFE_IPM2P0_RATE_352K;
	case 384000:
		return MTK_AFE_IPM2P0_RATE_384K;
	default:
		dev_err(dev, "rate %u invalid, use 48 kHz\n", rate);
		return MTK_AFE_IPM2P0_RATE_48K;
	}
}

static int mt6878_memif_fs(struct snd_pcm_substream *substream,
			   unsigned int rate)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_component *component =
		snd_soc_rtdcom_lookup(rtd, AFE_PCM_NAME);
	struct mtk_base_afe *afe;

	if (!component)
		return -EINVAL;
	afe = snd_soc_component_get_drvdata(component);

	return mt6878_rate_transform(afe->dev, rate);
}

static int mt6878_get_dai_fs(struct mtk_base_afe *afe, int dai_id,
			     unsigned int rate)
{
	return mt6878_rate_transform(afe->dev, rate);
}

static int mt6878_irq_fs(struct snd_pcm_substream *substream, unsigned int rate)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_component *component =
		snd_soc_rtdcom_lookup(rtd, AFE_PCM_NAME);
	struct mtk_base_afe *afe;

	if (!component)
		return -EINVAL;
	afe = snd_soc_component_get_drvdata(component);

	return mt6878_rate_transform(afe->dev, rate);
}

static int mt6878_get_memif_pbuf_size(struct snd_pcm_substream *substream)
{
	/* pbuf programming is skipped for now (memif pbuf_reg == -1). */
	return 0;
}

/* FE (frontend) DAIs: the generic AFE memif operations. */
static const struct snd_soc_dai_ops mt6878_memif_dai_ops = {
	.startup	= mtk_afe_fe_startup,
	.shutdown	= mtk_afe_fe_shutdown,
	.hw_params	= mtk_afe_fe_hw_params,
	.hw_free	= mtk_afe_fe_hw_free,
	.prepare	= mtk_afe_fe_prepare,
	.trigger	= mtk_afe_fe_trigger,
};

#define MTK_PCM_RATES (SNDRV_PCM_RATE_8000_48000 |\
		       SNDRV_PCM_RATE_88200 |\
		       SNDRV_PCM_RATE_96000 |\
		       SNDRV_PCM_RATE_176400 |\
		       SNDRV_PCM_RATE_192000)

#define MTK_PCM_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
			 SNDRV_PCM_FMTBIT_S24_LE |\
			 SNDRV_PCM_FMTBIT_S32_LE)

#define MT6878_FE_PLAYBACK_DAI(_name, _id) \
{ \
	.name = #_name, \
	.id = _id, \
	.playback = { \
		.stream_name = #_name, \
		.channels_min = 1, \
		.channels_max = 2, \
		.rates = MTK_PCM_RATES, \
		.formats = MTK_PCM_FORMATS, \
	}, \
	.ops = &mt6878_memif_dai_ops, \
}

static struct snd_soc_dai_driver mt6878_memif_dai_driver[] = {
	MT6878_FE_PLAYBACK_DAI(DL0, MT6878_MEMIF_DL0),
	MT6878_FE_PLAYBACK_DAI(DL1, MT6878_MEMIF_DL1),
	MT6878_FE_PLAYBACK_DAI(DL2, MT6878_MEMIF_DL2),
	MT6878_FE_PLAYBACK_DAI(DL3, MT6878_MEMIF_DL3),
	MT6878_FE_PLAYBACK_DAI(DL4, MT6878_MEMIF_DL4),
	MT6878_FE_PLAYBACK_DAI(DL5, MT6878_MEMIF_DL5),
	MT6878_FE_PLAYBACK_DAI(DL6, MT6878_MEMIF_DL6),
	MT6878_FE_PLAYBACK_DAI(DL7, MT6878_MEMIF_DL7),
	MT6878_FE_PLAYBACK_DAI(DL8, MT6878_MEMIF_DL8),
};

/*
 * Downlink memif descriptor. The DL0..DL8 registers form a regular 0x30-byte
 * block (BASE at AFE_DL_BASE(n), control word at AFE_DL_CON0(n)); the ON, fs,
 * mono and hd-mode fields all live in CON0. Fields that are not programmed on
 * this path are set to -1 so the framework skips them.
 */
#define MT6878_DL_MEMIF(_id, _n) \
	[MT6878_MEMIF_##_id] = { \
		.name = #_id, \
		.id = MT6878_MEMIF_##_id, \
		.reg_ofs_base = AFE_DL_BASE(_n), \
		.reg_ofs_cur = AFE_DL_CUR(_n), \
		.reg_ofs_end = AFE_DL_END(_n), \
		.reg_ofs_base_msb = AFE_DL_BASE_MSB(_n), \
		.reg_ofs_cur_msb = AFE_DL_CUR_MSB(_n), \
		.reg_ofs_end_msb = AFE_DL_END_MSB(_n), \
		.fs_reg = AFE_DL_CON0(_n), \
		.fs_shift = AFE_DL_CON0_FS_SHIFT, \
		.fs_maskbit = AFE_DL_CON0_FS_MASK, \
		.mono_reg = AFE_DL_CON0(_n), \
		.mono_shift = AFE_DL_CON0_MONO_SHIFT, \
		.mono_invert = 0, \
		.quad_ch_reg = -1, \
		.int_odd_flag_reg = -1, \
		.enable_reg = AFE_DL_CON0(_n), \
		.enable_shift = AFE_DL_CON0_ON_SHIFT, \
		.hd_reg = AFE_DL_CON0(_n), \
		.hd_shift = AFE_DL_CON0_HD_SHIFT, \
		.hd_align_reg = -1, \
		.hd_align_mshift = -1, \
		.msb_reg = -1, \
		.msb_shift = -1, \
		.msb_end_reg = -1, \
		.msb_end_shift = -1, \
		.agent_disable_reg = -1, \
		.agent_disable_shift = -1, \
		.ch_num_reg = -1, \
		.pbuf_reg = -1, \
		.minlen_reg = -1, \
	}

static const struct mtk_base_memif_data memif_data[MT6878_MEMIF_NUM] = {
	MT6878_DL_MEMIF(DL0, 0),
	MT6878_DL_MEMIF(DL1, 1),
	MT6878_DL_MEMIF(DL2, 2),
	MT6878_DL_MEMIF(DL3, 3),
	MT6878_DL_MEMIF(DL4, 4),
	MT6878_DL_MEMIF(DL5, 5),
	MT6878_DL_MEMIF(DL6, 6),
	MT6878_DL_MEMIF(DL7, 7),
	MT6878_DL_MEMIF(DL8, 8),
};

#define MT6878_AFE_IRQ(_n) \
	[MT6878_IRQ_##_n] = { \
		.id = MT6878_IRQ_##_n, \
		.irq_cnt_reg = AFE_IRQ_MCU_CNT(_n), \
		.irq_cnt_shift = AFE_IRQ_CNT_SHIFT, \
		.irq_cnt_maskbit = AFE_IRQ_CNT_MASK, \
		.irq_fs_reg = AFE_IRQ_MCU_CON(_n), \
		.irq_fs_shift = AFE_IRQ_FS_SHIFT, \
		.irq_fs_maskbit = AFE_IRQ_FS_MASK, \
		.irq_en_reg = AFE_IRQ_MCU_CON(_n), \
		.irq_en_shift = 0, \
		.irq_clr_reg = AFE_IRQ_MCU_CNT(_n), \
		.irq_clr_shift = 0, \
	}

static const struct mtk_base_irq_data irq_data[MT6878_IRQ_NUM] = {
	MT6878_AFE_IRQ(0),
	MT6878_AFE_IRQ(1),
	MT6878_AFE_IRQ(2),
	MT6878_AFE_IRQ(3),
	MT6878_AFE_IRQ(4),
	MT6878_AFE_IRQ(5),
	MT6878_AFE_IRQ(6),
	MT6878_AFE_IRQ(7),
	MT6878_AFE_IRQ(8),
};

static const int memif_irq_usage[MT6878_MEMIF_NUM] = {
	[MT6878_MEMIF_DL0] = MT6878_IRQ_0,
	[MT6878_MEMIF_DL1] = MT6878_IRQ_1,
	[MT6878_MEMIF_DL2] = MT6878_IRQ_2,
	[MT6878_MEMIF_DL3] = MT6878_IRQ_3,
	[MT6878_MEMIF_DL4] = MT6878_IRQ_4,
	[MT6878_MEMIF_DL5] = MT6878_IRQ_5,
	[MT6878_MEMIF_DL6] = MT6878_IRQ_6,
	[MT6878_MEMIF_DL7] = MT6878_IRQ_7,
	[MT6878_MEMIF_DL8] = MT6878_IRQ_8,
};

static const struct regmap_config mt6878_afe_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	/* TODO: set to the real AFE_MAX_REGISTER and switch to REGCACHE_FLAT */
	.max_register = 0x6000,
	.cache_type = REGCACHE_NONE,
};

static irqreturn_t mt6878_afe_irq_handler(int irq_id, void *dev)
{
	struct mtk_base_afe *afe = dev;
	unsigned int status, mcu_en;
	int i, ret;

	ret = regmap_read(afe->regmap, AFE_IRQ_MCU_EN, &mcu_en);
	if (ret)
		return IRQ_NONE;
	ret = regmap_read(afe->regmap, AFE_IRQ_MCU_STATUS, &status);
	if (ret)
		return IRQ_NONE;

	status &= mcu_en;
	if (!status)
		return IRQ_NONE;

	for (i = 0; i < afe->memif_size; i++) {
		struct mtk_base_afe_memif *memif = &afe->memif[i];
		struct mtk_base_afe_irq *irq;

		if (!memif->substream || memif->irq_usage < 0)
			continue;

		irq = &afe->irqs[memif->irq_usage];
		if (status & BIT(irq->irq_data->id))
			snd_pcm_period_elapsed(memif->substream);
	}

	/* TODO: confirm the interrupt-clear semantics against the hardware */
	regmap_write(afe->regmap, AFE_IRQ_MCU_STATUS, status);

	return IRQ_HANDLED;
}

/*
 * TODO: real audio clock bring-up. The AFE clock gates are registered from
 * within the driver (the audiosys block) together with the APLL/topckgen
 * muxes; until that mt6878-afe-clk.c lands these are no-ops and the AFE does
 * not run on hardware.
 */
static int mt6878_init_clock(struct mtk_base_afe *afe)
{
	return 0;
}

static int mt6878_afe_runtime_suspend(struct device *dev)
{
	return 0;
}

static int mt6878_afe_runtime_resume(struct device *dev)
{
	return 0;
}

static int mt6878_afe_component_probe(struct snd_soc_component *component)
{
	mtk_afe_add_sub_dai_control(component);
	return 0;
}

static const struct snd_soc_component_driver mt6878_afe_component = {
	.name = AFE_PCM_NAME,
	.probe = mt6878_afe_component_probe,
	.pcm_new = mtk_afe_pcm_new,
	.pointer = mtk_afe_pcm_pointer,
};

static int mt6878_dai_memif_register(struct mtk_base_afe *afe)
{
	struct mtk_base_afe_dai *dai;

	dai = devm_kzalloc(afe->dev, sizeof(*dai), GFP_KERNEL);
	if (!dai)
		return -ENOMEM;

	list_add(&dai->list, &afe->sub_dais);

	dai->dai_drivers = mt6878_memif_dai_driver;
	dai->num_dai_drivers = ARRAY_SIZE(mt6878_memif_dai_driver);

	return 0;
}

static int mt6878_afe_pcm_dev_probe(struct platform_device *pdev)
{
	struct mtk_base_afe *afe;
	struct mt6878_afe_private *afe_priv;
	struct device *dev = &pdev->dev;
	int ret, i, irq_id;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(34));
	if (ret)
		return ret;

	afe = devm_kzalloc(dev, sizeof(*afe), GFP_KERNEL);
	if (!afe)
		return -ENOMEM;
	platform_set_drvdata(pdev, afe);

	afe->platform_priv = devm_kzalloc(dev, sizeof(*afe_priv), GFP_KERNEL);
	if (!afe->platform_priv)
		return -ENOMEM;
	afe_priv = afe->platform_priv;
	afe->dev = dev;

	afe->base_addr = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(afe->base_addr))
		return dev_err_probe(dev, PTR_ERR(afe->base_addr),
				     "AFE base_addr not found\n");

	ret = mt6878_init_clock(afe);
	if (ret)
		return dev_err_probe(dev, ret, "init clock error\n");

	/* init memif */
	afe->memif_32bit_supported = 1;
	afe->memif_size = MT6878_MEMIF_NUM;
	afe->memif = devm_kcalloc(dev, afe->memif_size, sizeof(*afe->memif),
				  GFP_KERNEL);
	if (!afe->memif)
		return -ENOMEM;

	for (i = 0; i < afe->memif_size; i++) {
		afe->memif[i].data = &memif_data[i];
		afe->memif[i].irq_usage = memif_irq_usage[i];
		afe->memif[i].const_irq = 1;
	}

	mutex_init(&afe->irq_alloc_lock);

	/* init irq */
	afe->irqs_size = MT6878_IRQ_NUM;
	afe->irqs = devm_kcalloc(dev, afe->irqs_size, sizeof(*afe->irqs),
				 GFP_KERNEL);
	if (!afe->irqs)
		return -ENOMEM;

	for (i = 0; i < afe->irqs_size; i++)
		afe->irqs[i].irq_data = &irq_data[i];

	irq_id = platform_get_irq(pdev, 0);
	if (irq_id < 0)
		return irq_id;

	ret = devm_request_irq(dev, irq_id, mt6878_afe_irq_handler,
			       IRQF_TRIGGER_NONE, "Afe_ISR_Handle", afe);
	if (ret)
		return dev_err_probe(dev, ret,
				     "could not request_irq for Afe_ISR_Handle\n");

	/* init sub_dais */
	INIT_LIST_HEAD(&afe->sub_dais);

	ret = mt6878_dai_memif_register(afe);
	if (ret)
		return ret;

	ret = mtk_afe_combine_sub_dai(afe);
	if (ret)
		return dev_err_probe(dev, ret, "mtk_afe_combine_sub_dai fail\n");

	afe->mtk_afe_hardware = &mt6878_afe_hardware;
	afe->memif_fs = mt6878_memif_fs;
	afe->irq_fs = mt6878_irq_fs;
	afe->get_dai_fs = mt6878_get_dai_fs;
	afe->get_memif_pbuf_size = mt6878_get_memif_pbuf_size;
	afe->runtime_resume = mt6878_afe_runtime_resume;
	afe->runtime_suspend = mt6878_afe_runtime_suspend;

	afe->regmap = devm_regmap_init_mmio(dev, afe->base_addr,
					    &mt6878_afe_regmap_config);
	if (IS_ERR(afe->regmap))
		return dev_err_probe(dev, PTR_ERR(afe->regmap),
				     "regmap init failed\n");

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	ret = devm_snd_soc_register_component(dev, &mt6878_afe_component,
					      afe->dai_drivers,
					      afe->num_dai_drivers);
	if (ret)
		return dev_err_probe(dev, ret, "afe component register fail\n");

	return 0;
}

static const struct of_device_id mt6878_afe_pcm_dt_match[] = {
	{ .compatible = "mediatek,mt6878-sound", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, mt6878_afe_pcm_dt_match);

static const struct dev_pm_ops mt6878_afe_pm_ops = {
	SET_RUNTIME_PM_OPS(mt6878_afe_runtime_suspend,
			   mt6878_afe_runtime_resume, NULL)
};

static struct platform_driver mt6878_afe_pcm_driver = {
	.driver = {
		.name = "mt6878-afe-pcm",
		.of_match_table = mt6878_afe_pcm_dt_match,
		.pm = &mt6878_afe_pm_ops,
	},
	.probe = mt6878_afe_pcm_dev_probe,
};
module_platform_driver(mt6878_afe_pcm_driver);

MODULE_DESCRIPTION("MediaTek MT6878 ALSA SoC AFE platform driver");
MODULE_AUTHOR("Jan Benedikt <karagos01@gmail.com>");
MODULE_LICENSE("GPL");
