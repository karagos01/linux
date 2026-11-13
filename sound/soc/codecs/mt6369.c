// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek MT6369 ALSA SoC audio codec driver
 *
 * Copyright (c) 2026 Jan Benedikt <karagos01@gmail.com>
 *
 * The MT6369 is the audio codec inside the MT6369 SPMI sub-PMIC. It is a close
 * relative of the MT6359 codec (same downlink/output topology) but is reached
 * over SPMI with single-byte registers, so it has its own register map.
 *
 * This is an incremental bring-up: the downlink earpiece (receiver) output
 * path is implemented; the headphone, line-out, uplink (microphone) and the
 * downlink digital clock/charge-pump supplies are still to come, as is the
 * 26 MHz DCXO enable that lives in the companion MT6685 clock buffer.
 */

#include <linux/mfd/mt6685.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>

#include "mt6369.h"

struct mt6369_priv {
	struct device *dev;
	struct regmap *regmap;
	int hp_cal;		/* HP DC-offset trim from the PMIC efuse */
	bool ul_active;		/* capture (uplink) stream powered */
	bool dl_active;		/* playback (downlink) stream powered */
	bool spk_on;		/* lineout analog output stage powered */
	unsigned int lineout_gain;	/* LOL analog gain (ANA_CON129), 0..0x12 */
};

/*
 * Zero-crossing detector. device: 2 = receiver, 1 = line-out, else headphone.
 * Recovered from the stock driver's zcd_enable().
 */
static void mt6369_zcd_enable(struct mt6369_priv *priv, bool enable, int device)
{
	if (!enable) {
		regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON20, 0x7, 0x4);
		regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON128, 0x0);
		return;
	}

	if (device != 2)
		device = (device != 1);

	regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON20, 0x7, device);
	regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON128, 0x40, 0x0);
	regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON128, 0x30, 0x0);
	regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON128, 0xe, 0xa);
	regmap_update_bits(priv->regmap, MT6369_AUDDEC_ANA_CON128, 0x1, 0x1);
}

/* Downlink earpiece (receiver) analog output. Recovered from mt_rcv_event(). */
static int mt_rcv_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);
	struct regmap *regmap = priv->regmap;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON5, 0x1, 0x1);

		/* Negative charge pump for the class-AB amplifier. */
		regmap_update_bits(regmap, MT6369_AUDNCP_CLKDIV_CON1, 0x1, 0x1);
		regmap_write(regmap, MT6369_AUDNCP_CLKDIV_CON2, 0x2c);
		regmap_update_bits(regmap, MT6369_AUDNCP_CLKDIV_CON0, 0x1, 0x1);
		regmap_update_bits(regmap, MT6369_AUDNCP_CLKDIV_CON4, 0x3, 0x3);
		regmap_update_bits(regmap, MT6369_AUDNCP_CLKDIV_CON3, 0x1, 0x0);
		usleep_range(250, 270);

		/* Internal LDO. */
		regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON26, 0x10, 0x10);
		regmap_write(regmap, MT6369_AUDDEC_ANA_CON25, 0x55);
		regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON27, 0x1, 0x1);
		usleep_range(100, 120);

		/* Receiver bias. */
		regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON10, 0x10, 0x10);
		regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON22, 0x10, 0x0);
		regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON21, 0x70, 0x20);
		regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON23, 0xc0, 0x40);
		regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON23, 0xc, 0x4);
		regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON23, 0x3, 0x1);
		regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON23, 0x30, 0x10);

		/* Enable the receiver amplifier. */
		regmap_write(regmap, MT6369_AUDDEC_ANA_CON10, 0x90);
		regmap_write(regmap, MT6369_AUDDEC_ANA_CON17, 0x0);
		regmap_write(regmap, MT6369_AUDDEC_ANA_CON18, 0x0);
		regmap_write(regmap, MT6369_AUDDEC_ANA_CON19, 0x0);
		regmap_write(regmap, MT6369_AUDDEC_ANA_CON10, 0x92);
		regmap_write(regmap, MT6369_AUDDEC_ANA_CON10, 0x93);

		/* Receiver playback gain and output switch. The idle value only
		 * applies on power-down; using it here would keep the output
		 * muted while playing.
		 */
		regmap_write(regmap, MT6369_AUDDEC_ANA_CON133, MT6369_RCV_GAIN);
		regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON24, 0x11, 0x1);
		regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON0, 0x9, 0x9);
		regmap_write(regmap, MT6369_AUDDEC_ANA_CON16, 0x1);
		regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON10, 0xc, 0x8);

		mt6369_zcd_enable(priv, true, 2);
		break;

	case SND_SOC_DAPM_PRE_PMD:
		regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON10, 0xc, 0x0);
		regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON0, 0x9, 0x0);
		regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON24, 0x1, 0x0);
		regmap_write(regmap, MT6369_AUDDEC_ANA_CON133, MT6369_HPGAIN_IDLE);
		regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON10, 0x1, 0x0);
		regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON10, 0x2, 0x0);
		mt6369_zcd_enable(priv, false, 2);
		break;
	}

	return 0;
}

/*
 * Loudspeaker analog output: DACL/DACR -> headphone (HP) output stage in
 * "LoudSPK" mode (HP main path routed through the lineout driver), which on
 * this board feeds the external aw87xxx boost amplifiers. Ported from the
 * stock mt6369 HP/HPSPK enable sequence (charge-pump NCP, cap-less LDOs, HP
 * aux/main output ramps, then the LO sub-stage) with a fixed conservative
 * -10 dB analog gain; the full headphone volume/impedance/trim machinery is
 * omitted since the output only drives an external amplifier.
 */
#define MT6369_DL_GAIN_N_10DB	18
#define MT6369_DL_GAIN_MUTE	0x1f

/* Pull HPL/HPR down to AVSS28_AUD, stepped, to avoid a turn-on pop. */
static void __maybe_unused mt6369_hp_pull_down(struct regmap *regmap, bool enable)
{
	int i;

	if (enable)
		for (i = 0x0; i <= 0x7; i++) {
			regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON8, 0x7, i);
			usleep_range(100, 150);
		}
	else
		for (i = 0x7; i >= 0x0; i--) {
			regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON8, 0x7, i);
			usleep_range(100, 150);
		}
}

/* Ramp the HPL/HPR main output stage drive strength (CON3). */
static void __maybe_unused mt6369_hp_main_output_ramp(struct regmap *regmap, bool up)
{
	int i, stage;

	for (i = 0; i <= 7; i++) {
		stage = up ? i : 7 - i;
		regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON3, 0x7, stage);
		regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON3, 0x70,
				   stage << 4);
		usleep_range(100, 120);
	}
}

/* Ramp the HP aux feedback-loop gain (CON17 bits 4-7). */
static void __maybe_unused mt6369_hp_aux_fbloop_ramp(struct regmap *regmap, bool up)
{
	int i, stage;

	for (i = 0; i <= 0xf; i++) {
		stage = up ? i : 0xf - i;
		regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON17, 0xf0,
				   stage << 4);
		usleep_range(100, 120);
	}
}

static void mt6369_spk_enable(struct mt6369_priv *priv)
{
	struct regmap *regmap = priv->regmap;

	/* Reduce AU_REFN ESD resistance. */
	regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON5, 0x1, 0x1);

	/* Charge-pump (NCP): on, 604 kHz, toggle, 100us soft-start, enable. */
	regmap_update_bits(regmap, MT6369_AUDNCP_CLKDIV_CON1, 0x1, 0x1);
	regmap_write(regmap, MT6369_AUDNCP_CLKDIV_CON2, 0x2c);
	regmap_update_bits(regmap, MT6369_AUDNCP_CLKDIV_CON0, 0x1, 0x1);
	regmap_update_bits(regmap, MT6369_AUDNCP_CLKDIV_CON4, 0x3, 0x3);
	regmap_update_bits(regmap, MT6369_AUDNCP_CLKDIV_CON3, 0x1, 0x0);
	usleep_range(250, 270);

	/* Cap-less LDOs (1.5 V) and NV regulator (-1.2 V). */
	regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON26, 0x10, 0x10);
	regmap_write(regmap, MT6369_AUDDEC_ANA_CON25, 0x55);
	regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON27, 0x1, 0x1);
	usleep_range(100, 120);

	/* Disable ZCD (ZCD_CON0 == ANA_CON128). */
	regmap_write(regmap, MT6369_AUDDEC_ANA_CON128, 0x0);

	/*
	 * Lineout (LO) output path, driving the LINEOUT pins directly (this is
	 * the alternative to the HP/LoudSPK route; the external amp may tap the
	 * lineout rather than the headphone output). CON12 bit4 disables LO
	 * short-circuit protection, DR bias 6uA (CON22), LO/ZCD IBIAS (CON23).
	 */
	regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON12, 0x10, 0x10);
	regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON22, 0x7, 0x2);
	regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON23, 0xc0, 0x40);
	regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON23, 0x30, 0x10);

	/* LO STB enhance + CMFB/cap-size defaults. */
	regmap_write(regmap, MT6369_AUDDEC_ANA_CON13, 0x1);
	regmap_write(regmap, MT6369_AUDDEC_ANA_CON17, 0x0);
	regmap_write(regmap, MT6369_AUDDEC_ANA_CON18, 0x0);
	regmap_write(regmap, MT6369_AUDDEC_ANA_CON19, 0x0);

	/* Enable LO driver bias then core circuits. */
	regmap_write(regmap, MT6369_AUDDEC_ANA_CON12, 0x12);
	regmap_write(regmap, MT6369_AUDDEC_ANA_CON12, 0x13);

	/*
	 * Lineout gain (ZCD_CON1/2 == ANA_CON129/130). LOL (CON129) feeds the
	 * external aw87565 amps and is exposed as the "Lineout Playback Volume" control;
	 * apply the user-selected value (default 0x09 == -1 dB, matching the
	 * stock speaker playback state). LOR (CON130) is unused on this board.
	 */
	regmap_write(regmap, MT6369_AUDDEC_ANA_CON129, priv->lineout_gain);
	regmap_write(regmap, MT6369_AUDDEC_ANA_CON130, 0x00);

	/* Decoder clock, audio DAC R, low-noise mode. */
	regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON24, 0x1, 0x1);
	regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON0, 0x6, 0x6);
	regmap_write(regmap, MT6369_AUDDEC_ANA_CON16, 0x1);
	usleep_range(100, 120);

	/* Switch the LOL mux to DAC R (CON12 bits[3:2] = 1). */
	regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON12, 0xc, 0x4);
}

static void mt6369_spk_disable(struct regmap *regmap)
{
	/* LOL mux open, DAC R off, decoder clock off. */
	regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON12, 0xc, 0x0);
	regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON0, 0x6, 0x0);
	regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON24, 0x1, 0x0);

	/* Mute LO gain, then power down the LO driver core and bias. */
	regmap_write(regmap, MT6369_AUDDEC_ANA_CON129, MT6369_DL_GAIN_MUTE);
	regmap_write(regmap, MT6369_AUDDEC_ANA_CON130, MT6369_DL_GAIN_MUTE);
	regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON12, 0x1, 0x0);
	regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON12, 0x2, 0x0);

	/* Disable ZCD, restore AU_REFN ESD resistance. */
	regmap_write(regmap, MT6369_AUDDEC_ANA_CON128, 0x0);
	regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON5, 0x1, 0x0);
}

/* Loudspeaker (HP/LO) analog output driver. */
static int mt_spk_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		mt6369_spk_enable(priv);
		priv->spk_on = true;
		break;
	case SND_SOC_DAPM_POST_PMD:
		priv->spk_on = false;
		mt6369_spk_disable(priv->regmap);
		break;
	}

	return 0;
}

/* Configure the MTKAIF downlink GPIO pins. From mt6369_set_playback_gpio(). */
static void mt6369_set_playback_gpio(struct mt6369_priv *priv)
{
	struct regmap *regmap = priv->regmap;
	unsigned int val = 0;

	regmap_read(regmap, MT6369_GPIO_MODE4, &val);
	if ((val & 0x38) != 0x8) {
		regmap_write(regmap, MT6369_GPIO_MODE0_CLR, 0x38);
		regmap_write(regmap, MT6369_GPIO_MODE0_SET, 0x8);
	}
	regmap_write(regmap, MT6369_GPIO_MODE1_CLR, 0x3f);
	regmap_write(regmap, MT6369_GPIO_MODE1_SET, 0x9);
	regmap_write(regmap, MT6369_GPIO_MODE2_CLR, 0x7);
	regmap_write(regmap, MT6369_GPIO_MODE2_SET, 0x1);
}

/* Downlink MTKAIF GPIO supply. From mt_dl_gpio_event(). */
static int mt_dl_gpio_event(struct snd_soc_dapm_widget *w,
			    struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		mt6369_set_playback_gpio(priv);
		break;
	case SND_SOC_DAPM_POST_PMD:
		regmap_write(priv->regmap, MT6369_GPIO_MODE1_CLR, 0x3f);
		regmap_update_bits(priv->regmap, MT6369_GPIO_DIR1, 0xc, 0x0);
		break;
	}

	return 0;
}

/* Downlink sigma-delta modulator (DAC digital). From mt_sdm_event(). */
static int mt_sdm_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);
	struct regmap *regmap = priv->regmap;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		regmap_update_bits(regmap, MT6369_AFUNC_AUD_CON4, 0xfd, 0x6);
		regmap_write(regmap, MT6369_AFUNC_AUD_CON1, 0xcb);
		regmap_write(regmap, MT6369_AFUNC_AUD_CON0, 0xa1);
		regmap_update_bits(regmap, MT6369_AFUNC_AUD_CON4, 0xfd, 0x3);
		regmap_update_bits(regmap, MT6369_AFUNC_AUD_CON4, 0xff, 0xb);
		break;
	case SND_SOC_DAPM_POST_PMD:
		regmap_update_bits(regmap, MT6369_AFUNC_AUD_CON4, 0xfd, 0x0);
		regmap_write(regmap, MT6369_AFUNC_AUD_CON1, 0xcb);
		regmap_write(regmap, MT6369_AFUNC_AUD_CON0, 0xa0);
		break;
	}

	return 0;
}

/* 26 MHz audio DCXO buffer on the companion MT6685, from mt_dcxo_event(). */
static int mt_dcxo_event(struct snd_soc_dapm_widget *w,
			 struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);
	struct regmap *regmap = priv->regmap;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		mt6685_set_dcxo_mode(0);
		mt6685_set_dcxo(true);
		usleep_range(100, 120);
		/*
		 * PMIC-side AFE/MTKAIF downlink sub-block that is only writable
		 * while the 26 MHz audio DCXO is running, so it is configured
		 * here (not at init, where the DCXO is turned back off and these
		 * registers would reset). Values match the running stock earpiece
		 * state (full regmap dump during playback); 0x23a9/0x23aa are
		 * read-only datapath monitors and are not written.
		 */
		regmap_write(regmap, 0x23b1, 0x5c);
		regmap_write(regmap, 0x23b2, 0x03);
		regmap_write(regmap, 0x23b4, 0x00);
		regmap_write(regmap, 0x23b9, 0x14);
		regmap_write(regmap, 0x23ba, 0x14);
		regmap_write(regmap, 0x260f, 0x32);
		break;
	case SND_SOC_DAPM_POST_PMD:
		mt6685_set_dcxo(false);
		break;
	}

	return 0;
}

/* Enable the codec side of the MTKAIF serial link (protocol 2, as the stock
 * machine driver selects for MT6878+MT6369). From mt6369_mtkaif_tx_enable().
 */
static void mt6369_mtkaif_tx_enable(struct regmap *regmap)
{
	regmap_update_bits(regmap, MT6369_AFE_ADDA_MTKAIF_CFG0, 0xff, 0x10);
	regmap_update_bits(regmap, MT6369_AFE_ADDA_MTKAIF_CFG1, 0xff, 0x02);
	regmap_update_bits(regmap, MT6369_AFE_AUD_PAD_TOP, 0xff, 0x12);
	regmap_update_bits(regmap, MT6369_AFE_AUD_PAD_TOP, 0xff, 0x13);
}

/*
 * Uplink (microphone) capture path. The whole analog-mic -> ADC -> AFE-UL-SRC
 * -> MTKAIF-TX bring-up is emitted here in the exact order captured from a
 * stock capture ftrace (arecord on the primary mic): ADC clock, top clock,
 * MIC bias, the UL SRC datapath, the mic preamp (PGA) and ADC, the MTKAIF TX
 * link, then the UL SRC latch pulse. The teardown reverses it.
 */
static int mt_mic_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);
	struct regmap *regmap = priv->regmap;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		priv->ul_active = true;
		/* ADC + top audio clock */
		regmap_write(regmap, MT6369_AUDDEC_ANA_CON24, 0x00);
		regmap_write(regmap, MT6369_AUD_TOP_CKPDN, 0x3f);
		regmap_write(regmap, MT6369_AUD_TOP_CKSEL, 0x09);
		regmap_write(regmap, MT6369_AUDENC_ANA_CON12, 0x01);
		regmap_write(regmap, MT6369_AUDDEC_ANA_CON24, 0x20);
		regmap_write(regmap, MT6369_AUDDEC_ANA_CON26, 0x01);
		regmap_write(regmap, MT6369_AUDDEC_ANA_CON26, 0x05);
		regmap_write(regmap, MT6369_AUDDEC_ANA_CON26, 0x25);
		/*
		 * MTKAIF calibration-control idle for normal capture. During
		 * full duplex the downlink owns this register and leaves it at
		 * 0x00 (stock sets 0x60 for capture-only, 0x00 whenever playback
		 * is co-active), so skip the capture-only value when a playback
		 * is already running and let the downlink value stand.
		 */
		if (!priv->dl_active) {
			regmap_write(regmap, MT6369_AUD_TOP_CKPDN_CON0, 0x62);
			regmap_write(regmap, MT6369_AUD_TOP_CKPDN_CON0, 0x60);
		}
		/* MIC bias */
		regmap_write(regmap, MT6369_AUDENC_ANA_CON17, 0x20);
		regmap_write(regmap, MT6369_AUDENC_ANA_CON17, 0x21);
		/*
		 * Audio-top clock gate. 0x40 gates the downlink clock and is
		 * correct for capture-only; whenever a playback is co-active the
		 * downlink clock must stay on, so full duplex uses 0x00 (both
		 * clocks on), matching the stock full-duplex register state.
		 */
		regmap_write(regmap, MT6369_AUDIO_TOP_CON0, 0xc0);
		regmap_write(regmap, MT6369_AUDIO_TOP_CON0,
			     priv->dl_active ? 0x00 : 0x40);
		regmap_write(regmap, MT6369_AFE_UL_DL_CON0, 0x01);
		/*
		 * ADC analog bias/reference. The stock codec keeps these two
		 * registers resident (they read 0x74/0x07 even when idle); the
		 * mainline driver never programmed them, which left the ADC
		 * sampling a disconnected node and the uplink delivering only
		 * zero samples. Recovered from a stock-vs-mainline register diff
		 * of a live recording.
		 */
		regmap_write(regmap, MT6369_AUDENC_ANA_CON19, 0x74);
		regmap_write(regmap, MT6369_AUDENC_ANA_CON21, 0x07);
		/*
		 * mic preamp (PGA) and ADC, both channels. The ADC registers
		 * carry a static bit2 (0x04) in addition to the power-up bits:
		 * stock reads 0x54, and clearing bit2 (plain 0x50) leaves the
		 * converter muted.
		 */
		regmap_write(regmap, MT6369_AUDENC_ANA_CON0, 0x41);
		regmap_write(regmap, MT6369_AUDENC_ANA_CON2, 0xc1);
		regmap_write(regmap, MT6369_AUDENC_ANA_CON1, 0x54);
		regmap_write(regmap, MT6369_AUDENC_ANA_CON3, 0x54);
		/* MTKAIF TX link (protocol 2) */
		mt6369_mtkaif_tx_enable(regmap);
		/*
		 * Latch the UL SRC and leave it asserted. Despite the "pulse"
		 * naming this is a level enable: the stock codec writes CON3=1
		 * once and holds it for the whole capture (verified by a live
		 * register dump during a working stock recording, where 0x238b
		 * reads 0x01). Clearing it back to 0 unlatches the SRC and the
		 * uplink delivers only zero samples.
		 */
		regmap_write(regmap, MT6369_AFE_UL_SRC_CON1, 0x01);
		break;
	case SND_SOC_DAPM_POST_PMD:
		priv->ul_active = false;
		/* MTKAIF TX off */
		regmap_write(regmap, MT6369_AFE_AUD_PAD_TOP, 0x10);
		/* ADC + PGA off */
		regmap_write(regmap, MT6369_AUDENC_ANA_CON1, 0x40);
		regmap_write(regmap, MT6369_AUDENC_ANA_CON3, 0x40);
		regmap_write(regmap, MT6369_AUDENC_ANA_CON0, 0x40);
		regmap_write(regmap, MT6369_AUDENC_ANA_CON2, 0xc0);
		/*
		 * UL SRC off + datapath reset. 0x2388 (AFE_UL_DL_CON0 top
		 * enable) and 0x238f (AUDIO_TOP_CON0 clock gate) are shared with
		 * the downlink, so only tear them down when no playback is
		 * co-active; otherwise leave them powered for the downlink.
		 */
		if (!priv->dl_active) {
			regmap_write(regmap, MT6369_AFE_UL_DL_CON0, 0x00);
			regmap_write(regmap, MT6369_AUDIO_TOP_CON0, 0xc0);
			regmap_write(regmap, MT6369_AUDIO_TOP_CON0, 0xf5);
		}
		/* MIC bias off */
		regmap_write(regmap, MT6369_AUDENC_ANA_CON17, 0x20);
		regmap_write(regmap, MT6369_AUDENC_ANA_CON17, 0x00);
		regmap_write(regmap, MT6369_AUDENC_ANA_CON18, 0x00);
		/* ADC clock off */
		regmap_write(regmap, MT6369_AUDDEC_ANA_CON26, 0x21);
		regmap_write(regmap, MT6369_AUDDEC_ANA_CON26, 0x01);
		regmap_write(regmap, MT6369_AUDDEC_ANA_CON26, 0x00);
		regmap_write(regmap, MT6369_AUDDEC_ANA_CON24, 0x00);
		regmap_write(regmap, MT6369_AUDENC_ANA_CON12, 0x00);
		regmap_write(regmap, MT6369_AUD_TOP_CKPDN, 0x3f);
		regmap_write(regmap, MT6369_AUDDEC_ANA_CON24, 0x10);
		break;
	}

	return 0;
}

/*
 * MTKAIF RX phase calibration, driven once from the machine driver.
 * enable/disable set up and tear down the calibration mode; set_phase pokes
 * the per-channel RX delay. Recovered from the stock mt6369_mtkaif_*() helpers.
 */
void mt6369_mtkaif_calibration_enable(struct snd_soc_component *cmpnt)
{
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);
	struct regmap *regmap = priv->regmap;

	mt6369_set_playback_gpio(priv);
	regmap_write(regmap, MT6369_GPIO_MODE3_CLR, 0x3f);
	regmap_write(regmap, MT6369_GPIO_MODE3_SET, 0x09);
	mt6369_mtkaif_tx_enable(regmap);
	mt6685_set_dcxo(true);
	regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON24, 0x10, 0x0);
	regmap_update_bits(regmap, MT6369_AUDENC_ANA_CON12, 0x1, 0x1);
	regmap_update_bits(regmap, MT6369_AUD_TOP_CKPDN_CON0, 0x66, 0x0);
	regmap_update_bits(regmap, MT6369_AUDIO_DIG_CFG1, 0x80, 0x80);
	regmap_update_bits(regmap, MT6369_AUDIO_DIG_CFG0, 0x80, 0x80);
	regmap_update_bits(regmap, MT6369_AUDIO_DIG_CFG1, 0x80, 0x80);
}
EXPORT_SYMBOL_GPL(mt6369_mtkaif_calibration_enable);

void mt6369_set_mtkaif_calibration_phase(struct snd_soc_component *cmpnt,
					 int phase_1, int phase_2, int phase_3)
{
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);
	struct regmap *regmap = priv->regmap;

	regmap_update_bits(regmap, MT6369_AUDIO_DIG_CFG0, 0x7f, phase_1);
	regmap_update_bits(regmap, MT6369_AUDIO_DIG_CFG1, 0x7f, phase_2);
	regmap_update_bits(regmap, MT6369_AUDIO_DIG_CFG2, 0x7f, phase_3);
}
EXPORT_SYMBOL_GPL(mt6369_set_mtkaif_calibration_phase);

void mt6369_mtkaif_calibration_disable(struct snd_soc_component *cmpnt)
{
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);
	struct regmap *regmap = priv->regmap;

	regmap_update_bits(regmap, MT6369_AUDIO_DIG_CFG1, 0x80, 0x0);
	regmap_update_bits(regmap, MT6369_AUDIO_DIG_CFG0, 0x80, 0x0);
	regmap_update_bits(regmap, MT6369_AUDIO_DIG_CFG2, 0x80, 0x0);
	regmap_update_bits(regmap, MT6369_AUD_TOP_CKPDN_CON0, 0x66, 0x66);
	regmap_update_bits(regmap, MT6369_AUDENC_ANA_CON12, 0x1, 0x0);
	regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON24, 0x10, 0x10);
	mt6685_set_dcxo(false);
	regmap_update_bits(regmap, MT6369_AFE_AUD_PAD_TOP, 0xff, 0x10);
	regmap_write(regmap, MT6369_GPIO_MODE1_CLR, 0x3f);
	regmap_update_bits(regmap, MT6369_GPIO_DIR1, 0xc, 0x0);
	regmap_write(regmap, MT6369_GPIO_MODE3_CLR, 0x3f);
	regmap_update_bits(regmap, MT6369_GPIO_DIR1, 0xc0, 0x0);
}
EXPORT_SYMBOL_GPL(mt6369_mtkaif_calibration_disable);

/* One-time analog configuration. Recovered from mt6369_codec_init_reg(). */
static void mt6369_codec_init_reg(struct mt6369_priv *priv)
{
	struct regmap *regmap = priv->regmap;

	/*
	 * The analog registers below are clocked by the 26 MHz audio DCXO
	 * buffer on the companion MT6685, so enable it for the duration of
	 * the one-time init (the downlink path re-enables it at stream time).
	 */
	mt6685_set_dcxo_mode(0);
	mt6685_set_dcxo(true);

	/*
	 * Enable the VAUD28 analog audio supply. TODO: this LDO belongs in the
	 * mt6369 regulator driver and should be requested through the regulator
	 * framework; poke it directly for now so the output stage has power.
	 */
	regmap_update_bits(regmap, MT6369_LDO_VAUD28_CON0, 0x1, 0x1);

	regmap_update_bits(regmap, MT6369_AUDENC_ANA_CON12, 0x2, 0x0);
	regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON1, 0x10, 0x10);
	regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON1, 0x20, 0x20);
	regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON10, 0x10, 0x10);
	regmap_update_bits(regmap, MT6369_AUDDEC_ANA_CON12, 0x10, 0x10);
	regmap_update_bits(regmap, MT6369_AUDENC_ANA_CON23, 0xf0, 0x20);

	/* Analog pad drive strength. */
	regmap_update_bits(regmap, MT6369_SMT_CON2, 0xff, 0xfc);
	regmap_update_bits(regmap, MT6369_SMT_CON3, 0xf, 0xf);
	regmap_update_bits(regmap, MT6369_DRV_CON3, 0xff, 0x88);
	regmap_update_bits(regmap, MT6369_DRV_CON4, 0xff, 0x88);
	regmap_update_bits(regmap, MT6369_DRV_CON5, 0xff, 0x88);
	regmap_update_bits(regmap, MT6369_DRV_CON6, 0xff, 0x88);

	/* Audio GPIO defaults. */
	regmap_write(regmap, MT6369_GPIO_MODE1_CLR, 0x3f);
	regmap_update_bits(regmap, MT6369_GPIO_DIR1, 0xc, 0x0);
	regmap_write(regmap, MT6369_GPIO_MODE3_CLR, 0x3f);
	regmap_update_bits(regmap, MT6369_GPIO_DIR1, 0xc0, 0x0);

	regmap_update_bits(regmap, MT6369_AFE_DL_NLE_CFG, 0x4, 0x0);
	regmap_update_bits(regmap, MT6369_AFE_DL_NLE_CFG, 0x20, 0x0);

	mt6369_zcd_enable(priv, false, 5);

	/*
	 * PMIC-side AFE / MTKAIF downlink interface block. The stock codec
	 * configures this run of registers; without it the MTKAIF downlink
	 * samples never reach the sigma-delta DAC and the output is silent.
	 * Values captured verbatim from a working stock earpiece playback.
	 */
	regmap_write(regmap, 0x230f, 0x20);
	regmap_write(regmap, 0x231d, 0x06);
	regmap_write(regmap, MT6369_AFE_UL_DL_CON0, 0x01);
	regmap_write(regmap, MT6369_AFE_DL_SRC2_CON0, 0x01);
	regmap_write(regmap, MT6369_AUDIO_TOP_CON0, 0x30);
	regmap_write(regmap, 0x23c8, 0x40);
	regmap_write(regmap, MT6369_AUDIO_DIG_CFG0, 0x03);
	regmap_write(regmap, MT6369_AUDIO_DIG_CFG1, 0x03);
	regmap_write(regmap, 0x23da, 0x2c);
	regmap_write(regmap, 0x2595, 0x01);

	mt6685_set_dcxo(false);
}

static int mt6369_codec_probe(struct snd_soc_component *cmpnt)
{
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);

	snd_soc_component_init_regmap(cmpnt, priv->regmap);
	mt6369_codec_init_reg(priv);

	return 0;
}

/*
 * Virtual switch to select the loudspeaker output. Off by default so ordinary
 * downlink playback still drives only the receiver; userspace enables it to
 * route the downlink to the HP/LO speaker stage and the external amplifier.
 */
static const struct snd_kcontrol_new mt6369_spk_switch =
	SOC_DAPM_SINGLE_VIRT("Switch", 1);

static const struct snd_kcontrol_new mt6369_rcv_switch =
	SOC_DAPM_SINGLE_VIRT("Switch", 1);

static const struct snd_soc_dapm_widget mt6369_dapm_widgets[] = {
	/* Digital downlink from the AFE over the MTKAIF serial link. */
	SND_SOC_DAPM_AIF_IN("AIF_RX", "mt6369-snd-codec-aif1 Playback", 0,
			    SND_SOC_NOPM, 0, 0),

	/* Downlink digital supplies: MTKAIF pins and the sigma-delta DAC. */
	SND_SOC_DAPM_SUPPLY("DL GPIO", SND_SOC_NOPM, 0, 0, mt_dl_gpio_event,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("DL SDM", SND_SOC_NOPM, 0, 0, mt_sdm_event,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("DCXO", SND_SOC_NOPM, 0, 0, mt_dcxo_event,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_DAC("DACL", NULL, SND_SOC_NOPM, 0, 0),

	/*
	 * Analog earpiece (receiver) output. Powered whenever the downlink DAC
	 * runs: it brings up the shared output-stage bias/input the loudspeaker
	 * LO path also depends on, so stock keeps the receiver active during
	 * loudspeaker playback too (do NOT gate it off against the speaker).
	 */
	SND_SOC_DAPM_OUT_DRV_E("RCV", SND_SOC_NOPM, 0, 0, NULL, 0, mt_rcv_event,
			       SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_OUTPUT("Receiver"),

	/*
	 * Analog LINEOUT (LOL) output stage feeding the external aw87565 amps.
	 * On this board BOTH the loudspeaker and the earpiece are aw87565 amps
	 * fed from LINEOUT (the codec's own RCV pins are not wired to the
	 * earpiece transducer); the only difference is which amp is powered.
	 * The LINEOUT analog driver is therefore a shared supply that powers
	 * whenever either the "Speaker" or the "Receiver" output is active, and
	 * two switches select the loudspeaker amp, the earpiece amp, or both.
	 */
	SND_SOC_DAPM_SWITCH("Speaker Switch", SND_SOC_NOPM, 0, 0,
			    &mt6369_spk_switch),
	SND_SOC_DAPM_SWITCH("Receiver Switch", SND_SOC_NOPM, 0, 0,
			    &mt6369_rcv_switch),

	SND_SOC_DAPM_SUPPLY("Lineout Driver", SND_SOC_NOPM, 0, 0, mt_spk_event,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_OUTPUT("Speaker"),
	SND_SOC_DAPM_OUTPUT("Receiver Amp"),

	/* Digital uplink (microphone) to the AFE over the MTKAIF serial link. */
	SND_SOC_DAPM_AIF_OUT("AIF_TX", "mt6369-snd-codec-aif1 Capture", 0,
			     SND_SOC_NOPM, 0, 0),

	/* Analog mic -> ADC bring-up (full sequence) plus the 26 MHz DCXO. */
	SND_SOC_DAPM_ADC_E("Mic ADC", NULL, SND_SOC_NOPM, 0, 0, mt_mic_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_INPUT("AIN0"),
};

static const struct snd_soc_dapm_route mt6369_dapm_routes[] = {
	{"DACL", NULL, "AIF_RX"},
	{"DACL", NULL, "DL GPIO"},
	{"DACL", NULL, "DL SDM"},
	{"DACL", NULL, "DCXO"},
	{"RCV", NULL, "DACL"},
	{"Receiver", NULL, "RCV"},

	{"Speaker Switch", "Switch", "DACL"},
	{"Speaker", NULL, "Speaker Switch"},
	{"Speaker", NULL, "Lineout Driver"},

	{"Receiver Switch", "Switch", "DACL"},
	{"Receiver Amp", NULL, "Receiver Switch"},
	{"Receiver Amp", NULL, "Lineout Driver"},

	/* capture: mic -> ADC -> MTKAIF TX -> AFE */
	{"Mic ADC", NULL, "AIN0"},
	{"Mic ADC", NULL, "DCXO"},
	{"AIF_TX", NULL, "Mic ADC"},
};

/*
 * "Lineout Playback Volume": the LOL analog output gain (ANA_CON129, ZCD_CON1 lineout
 * gain field) that drives both external aw87565 amps. The 5-bit field encodes
 * ATTENUATION: register value increases as the output gets quieter (0x00 =
 * +8 dB, 0x09 = -1 dB stock default, 0x12 = -10 dB, 0x1f = mute on power-down).
 * Note this polarity is the opposite of mt6359's lineout control -- confirmed
 * on this device by an acoustic loopback (lower register = louder capture).
 *
 * The control is exposed loudness-ascending (0 = quietest, 0x12 = loudest) so
 * "turn it up" means a higher number; the register written is the inverse.
 * Cached in priv because the power sequence (re)writes CON129 on every enable
 * and mutes it on disable, so the control cannot read the live register.
 */
#define MT6369_LINEOUT_GAIN_MAX		0x12	/* register 0..0x12 (-10..+8 dB) */
static const DECLARE_TLV_DB_SCALE(lineout_tlv, -1000, 100, 0);

static int mt6369_get_lineout_gain(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_kcontrol_chip(kcontrol);
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);

	ucontrol->value.integer.value[0] =
		MT6369_LINEOUT_GAIN_MAX - priv->lineout_gain;

	return 0;
}

static int mt6369_put_lineout_gain(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_kcontrol_chip(kcontrol);
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(cmpnt);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int val = ucontrol->value.integer.value[0];
	unsigned int reg;

	if (val > mc->max)
		return -EINVAL;

	reg = MT6369_LINEOUT_GAIN_MAX - val;	/* control is loudness, reg is attenuation */
	if (reg == priv->lineout_gain)
		return 0;

	priv->lineout_gain = reg;
	/*
	 * Apply immediately only while the lineout stage is powered; otherwise
	 * mt6369_spk_enable() picks the value up on the next power-up.
	 */
	if (priv->spk_on)
		regmap_write(priv->regmap, MT6369_AUDDEC_ANA_CON129, reg);

	return 1;
}

static const struct snd_kcontrol_new mt6369_snd_controls[] = {
	SOC_SINGLE_EXT_TLV("Lineout Playback Volume", MT6369_AUDDEC_ANA_CON129, 0,
			   MT6369_LINEOUT_GAIN_MAX, 0, mt6369_get_lineout_gain,
			   mt6369_put_lineout_gain, lineout_tlv),
};

static const struct snd_soc_component_driver mt6369_soc_component_driver = {
	.probe			= mt6369_codec_probe,
	.controls		= mt6369_snd_controls,
	.num_controls		= ARRAY_SIZE(mt6369_snd_controls),
	.dapm_widgets		= mt6369_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(mt6369_dapm_widgets),
	.dapm_routes		= mt6369_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(mt6369_dapm_routes),
	.endianness		= 1,
};

/*
 * Codec downlink-enable preamble. Stock emits this on every playback (via the
 * AFE's codec DL ops), and crucially only after the AFE back end is already
 * running: the AFE DL memif is enabled, AFE_ON is set and the MTKAIF RX link
 * is clocking data. It gates the codec audio clock on (AUDENC_ANA_CON12
 * bit0), pulses AUD_TOP_CKPDN_CON0 and resets the codec DL datapath with an
 * AUDIO_TOP_CON0 b0->30 pulse, then re-asserts the DL enable bits. The
 * AUDIO_TOP_CON0 reset only latches correctly against a live MTKAIF link, so
 * this must run at stream START (after the AFE back end is up), not from a
 * codec DAPM supply which powers earlier. Captured from a stock
 * earpiece-playback regmap trace.
 */
static void mt6369_dl_enable(struct mt6369_priv *priv)
{
	struct regmap *r = priv->regmap;

	/*
	 * Downlink datapath preamble: re-assert the MTKAIF/audio clock gate and
	 * pulse the DL datapath reset so it latches against the live MTKAIF link.
	 * This must run at stream START (after the AFE back end is up), hence from
	 * the codec DAI .trigger rather than a DAPM supply. It runs in atomic
	 * context, so it only does quick register writes (no settling delays); the
	 * receiver analog stage, which needs the delays, is brought up in process
	 * context by mt_rcv_event.
	 */
	regmap_update_bits(r, MT6369_AUDENC_ANA_CON12, 0x1, 0x1);
	regmap_write(r, MT6369_AUD_TOP_CKPDN_CON0, 0x2);
	regmap_write(r, MT6369_AUD_TOP_CKPDN_CON0, 0x0);

	if (!priv->ul_active) {
		/*
		 * Playback only: reset the datapath and leave the audio-top
		 * clock gate at 0x30.
		 */
		regmap_write(r, MT6369_AUDIO_TOP_CON0, 0xb0);
		regmap_write(r, MT6369_AUDIO_TOP_CON0, 0x30);
		regmap_write(r, MT6369_AFE_UL_DL_CON0, 0x1);
		regmap_write(r, MT6369_AFE_DL_SRC2_CON0, 0x1);
		return;
	}

	/*
	 * Full duplex: a capture is already running. AUDIO_TOP_CON0 is the
	 * shared audio-top clock gate; the capture left it at 0x40 (downlink
	 * clock gated), which silences playback. Stock runs full duplex with it
	 * at 0x00 (both clocks on) and AUD_TOP_CKPDN_CON0 at 0x00, so pulse the
	 * datapath reset and settle on those shared values, then re-latch the
	 * uplink SRC that the reset tore down. The downlink SRC and the uplink
	 * SRC share this one sequence.
	 */
	regmap_write(r, MT6369_AUDIO_TOP_CON0, 0x80);
	regmap_write(r, MT6369_AUDIO_TOP_CON0, 0x00);
	regmap_write(r, MT6369_AFE_UL_DL_CON0, 0x1);
	regmap_write(r, MT6369_AFE_DL_SRC2_CON0, 0x1);
	regmap_write(r, MT6369_AFE_UL_SRC_CON1, 0x1);
}

static int mt6369_codec_dai_trigger(struct snd_pcm_substream *substream,
				    int cmd, struct snd_soc_dai *dai)
{
	struct mt6369_priv *priv = snd_soc_component_get_drvdata(dai->component);

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		priv->dl_active = true;
		mt6369_dl_enable(priv);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		/*
		 * Clear the downlink marker so a later capture-only stream uses
		 * the capture-only clock value again. A capture running now is
		 * unaffected: the shared clock stays on.
		 */
		priv->dl_active = false;
		break;
	default:
		break;
	}

	return 0;
}

static const struct snd_soc_dai_ops mt6369_codec_dai_ops = {
	.trigger = mt6369_codec_dai_trigger,
};

static struct snd_soc_dai_driver mt6369_dai_driver[] = {
	{
		.name = "mt6369-snd-codec-aif1",
		.ops = &mt6369_codec_dai_ops,
		.playback = {
			.stream_name	= "mt6369-snd-codec-aif1 Playback",
			.channels_min	= 1,
			.channels_max	= 2,
			.rates		= SNDRV_PCM_RATE_8000_192000,
			.formats	= SNDRV_PCM_FMTBIT_S16_LE |
					  SNDRV_PCM_FMTBIT_S24_LE |
					  SNDRV_PCM_FMTBIT_S32_LE,
		},
		.capture = {
			.stream_name	= "mt6369-snd-codec-aif1 Capture",
			.channels_min	= 1,
			.channels_max	= 2,
			.rates		= SNDRV_PCM_RATE_8000_192000,
			.formats	= SNDRV_PCM_FMTBIT_S16_LE |
					  SNDRV_PCM_FMTBIT_S24_LE |
					  SNDRV_PCM_FMTBIT_S32_LE,
		},
	},
};

static int mt6369_platform_driver_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mt6369_priv *priv;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	priv->lineout_gain = 0x09;	/* -1 dB, stock speaker default */
	priv->regmap = dev_get_regmap(dev->parent, NULL);
	if (!priv->regmap)
		return dev_err_probe(dev, -ENODEV, "failed to get parent regmap\n");

	platform_set_drvdata(pdev, priv);

	return devm_snd_soc_register_component(dev, &mt6369_soc_component_driver,
					       mt6369_dai_driver,
					       ARRAY_SIZE(mt6369_dai_driver));
}

static const struct of_device_id mt6369_of_match[] = {
	{ .compatible = "mediatek,mt6369-sound" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mt6369_of_match);

static struct platform_driver mt6369_platform_driver = {
	.driver = {
		.name = "mt6369-sound",
		.of_match_table = mt6369_of_match,
	},
	.probe = mt6369_platform_driver_probe,
};
module_platform_driver(mt6369_platform_driver);

MODULE_DESCRIPTION("MediaTek MT6369 ALSA SoC codec driver");
MODULE_AUTHOR("Jan Benedikt <karagos01@gmail.com>");
MODULE_LICENSE("GPL");
