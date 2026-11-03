// SPDX-License-Identifier: GPL-2.0
//
// mt6878-afe-gpio.c  --  MediaTek 6878 afe gpio ctrl
//
// Copyright (c) 2026 Jan Benedikt <karagos01@gmail.com>

#include <linux/pinctrl/consumer.h>

#include "mt6878-afe-common.h"
#include "mt6878-afe-gpio.h"

static struct pinctrl *aud_pinctrl;

enum mt6878_afe_gpio {
	MT6878_AFE_GPIO_CLK_MOSI_OFF,
	MT6878_AFE_GPIO_CLK_MOSI_ON,
	MT6878_AFE_GPIO_DAT_MOSI_OFF,
	MT6878_AFE_GPIO_DAT_MOSI_ON,
	MT6878_AFE_GPIO_DAT_MISO0_OFF,
	MT6878_AFE_GPIO_DAT_MISO0_ON,
	MT6878_AFE_GPIO_DAT_MISO1_OFF,
	MT6878_AFE_GPIO_DAT_MISO1_ON,
	MT6878_AFE_GPIO_I2S_OUT4_OFF,
	MT6878_AFE_GPIO_I2S_OUT4_ON,
	MT6878_AFE_GPIO_GPIO_NUM
};

struct audio_gpio_attr {
	const char *name;
	bool gpio_prepare;
	struct pinctrl_state *gpioctrl;
};

static struct audio_gpio_attr aud_gpios[MT6878_AFE_GPIO_GPIO_NUM] = {
	[MT6878_AFE_GPIO_CLK_MOSI_OFF] = {"aud_clk_mosi_off", false, NULL},
	[MT6878_AFE_GPIO_CLK_MOSI_ON] = {"aud_clk_mosi_on", false, NULL},
	[MT6878_AFE_GPIO_DAT_MOSI_OFF] = {"aud_dat_mosi_off", false, NULL},
	[MT6878_AFE_GPIO_DAT_MOSI_ON] = {"aud_dat_mosi_on", false, NULL},
	[MT6878_AFE_GPIO_DAT_MISO0_OFF] = {"aud_dat_miso0_off", false, NULL},
	[MT6878_AFE_GPIO_DAT_MISO0_ON] = {"aud_dat_miso0_on", false, NULL},
	[MT6878_AFE_GPIO_DAT_MISO1_OFF] = {"aud_dat_miso1_off", false, NULL},
	[MT6878_AFE_GPIO_DAT_MISO1_ON] = {"aud_dat_miso1_on", false, NULL},
	[MT6878_AFE_GPIO_I2S_OUT4_OFF] = {"aud_gpio_i2s_out4_off", false, NULL},
	[MT6878_AFE_GPIO_I2S_OUT4_ON] = {"aud_gpio_i2s_out4_on", false, NULL},
};

static DEFINE_MUTEX(gpio_request_mutex);

static int mt6878_afe_gpio_select(struct mtk_base_afe *afe,
				  enum mt6878_afe_gpio type)
{
	struct device *dev = afe->dev;
	int ret;

	if (type < 0 || type >= MT6878_AFE_GPIO_GPIO_NUM) {
		dev_err(dev, "%s(), error, invalid gpio type %d\n",
			__func__, type);
		return -EINVAL;
	}

	if (!aud_gpios[type].gpio_prepare) {
		dev_warn(dev, "%s(), error, gpio type %d not prepared\n",
			 __func__, type);
		return -EIO;
	}

	ret = pinctrl_select_state(aud_pinctrl, aud_gpios[type].gpioctrl);
	if (ret)
		dev_dbg(dev, "%s(), error, can not set gpio type %d\n",
			__func__, type);

	return ret;
}

int mt6878_afe_gpio_init(struct mtk_base_afe *afe)
{
	struct device *dev = afe->dev;
	int i, ret;

	aud_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(aud_pinctrl)) {
		ret = PTR_ERR(aud_pinctrl);
		dev_err(dev, "%s(), ret %d, cannot get aud_pinctrl!\n",
			__func__, ret);
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(aud_gpios); i++) {
		aud_gpios[i].gpioctrl = pinctrl_lookup_state(aud_pinctrl,
							     aud_gpios[i].name);
		if (IS_ERR(aud_gpios[i].gpioctrl)) {
			ret = PTR_ERR(aud_gpios[i].gpioctrl);
			dev_dbg(dev, "%s(), pinctrl_lookup_state %s fail, ret %d\n",
				__func__, aud_gpios[i].name, ret);
		} else {
			aud_gpios[i].gpio_prepare = true;
		}
	}

	/*
	 * The MTKAIF clock line is driven continuously while the link is up,
	 * so park it on the audio function once and only toggle the data
	 * lines at stream time.
	 */
	mt6878_afe_gpio_select(afe, MT6878_AFE_GPIO_CLK_MOSI_ON);

	/* gpio status init */
	mt6878_afe_gpio_request(afe, false, MT6878_DAI_ADDA, 0);
	mt6878_afe_gpio_request(afe, false, MT6878_DAI_ADDA, 1);

	return 0;
}

static int mt6878_afe_gpio_adda_dl(struct mtk_base_afe *afe, bool enable)
{
	if (enable)
		return mt6878_afe_gpio_select(afe, MT6878_AFE_GPIO_DAT_MOSI_ON);
	else
		return mt6878_afe_gpio_select(afe, MT6878_AFE_GPIO_DAT_MOSI_OFF);
}

static int mt6878_afe_gpio_adda_ul(struct mtk_base_afe *afe, bool enable)
{
	if (enable) {
		mt6878_afe_gpio_select(afe, MT6878_AFE_GPIO_DAT_MISO0_ON);
		return mt6878_afe_gpio_select(afe, MT6878_AFE_GPIO_DAT_MISO1_ON);
	}

	mt6878_afe_gpio_select(afe, MT6878_AFE_GPIO_DAT_MISO0_OFF);
	return mt6878_afe_gpio_select(afe, MT6878_AFE_GPIO_DAT_MISO1_OFF);
}

/*
 * ETDM_OUT4 drives the external speaker I2S. A single pinctrl state muxes the
 * output data pin together with the shared I2S4 clock/word-select pad (named
 * i2sin4 by the pin table) so the SoC can master the bus; the clock and data
 * pads must switch in one state, as a second pinctrl_select_state() would
 * release the first.
 */
static int mt6878_afe_gpio_i2s_out4(struct mtk_base_afe *afe, bool enable)
{
	if (enable)
		return mt6878_afe_gpio_select(afe, MT6878_AFE_GPIO_I2S_OUT4_ON);

	return mt6878_afe_gpio_select(afe, MT6878_AFE_GPIO_I2S_OUT4_OFF);
}

int mt6878_afe_gpio_request(struct mtk_base_afe *afe, bool enable,
			    int dai, int uplink)
{
	guard(mutex)(&gpio_request_mutex);
	switch (dai) {
	case MT6878_DAI_ADDA:
		if (uplink)
			mt6878_afe_gpio_adda_ul(afe, enable);
		else
			mt6878_afe_gpio_adda_dl(afe, enable);
		break;
	case MT6878_DAI_I2S_OUT4:
		mt6878_afe_gpio_i2s_out4(afe, enable);
		break;
	default:
		dev_warn(afe->dev, "%s(), invalid dai %d\n", __func__, dai);
		return -EINVAL;
	}

	return 0;
}
