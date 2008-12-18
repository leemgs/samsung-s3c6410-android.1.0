/*
 * wm8580.c  --  WM8580 ALSA Soc Audio driver
 *
 * Copyright 2008 Wolfson Microelectronics PLC.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 * Notes:
 *  The WM8580 is a multichannel codec with S/PDIF support, featuring six
 *  DAC channels and two ADC channels.
 *
 *  Currently only the primary audio interface is supported - S/PDIF and
 *  the secondary audio interfaces are not.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include <sound/initval.h>
#include <asm/div64.h>

#include "wm8580.h"

#define AUDIO_NAME "wm8580"
#define WM8580_VERSION "0.1"

/*
 * Debug
 */

#define WM8580_DEBUG 0

#ifdef WM8580_DEBUG
#define dbg(format, arg...) \
	printk(KERN_DEBUG AUDIO_NAME ": " format , ## arg)
#else
#define dbg(format, arg...) do {} while (0)
#endif
#define err(format, arg...) \
	printk(KERN_ERR AUDIO_NAME ": " format , ## arg)
#define info(format, arg...) \
	printk(KERN_INFO AUDIO_NAME ": " format , ## arg)
#define warn(format, arg...) \
	printk(KERN_WARNING AUDIO_NAME ": " format , ## arg)

struct pll_state {
	unsigned int in;
	unsigned int out;
};

/* codec private data */
struct wm8580_priv {
	struct pll_state a;
	struct pll_state b;
};

/*
 * wm8580 register cache
 * We can't read the WM8580 register space when we
 * are using 2 wire for device control, so we cache them instead.
 */
static const u16 wm8580_reg[] = {
	0x0121, 0x017e, 0x007d, 0x0014, /*R3*/
	0x0121, 0x017e, 0x007d, 0x0194, /*R7*/
	0x001c, 0x0002, 0x0002, 0x00c2, /*R11*/
	0x0182, 0x0082, 0x000a, 0x0024, /*R15*/
	0x0009, 0x0000, 0x00ff, 0x0000, /*R19*/
	0x00ff, 0x00ff, 0x00ff, 0x00ff, /*R23*/
	0x00ff, 0x00ff, 0x00ff, 0x00ff, /*R27*/
	0x01f0, 0x0040, 0x0000, 0x0000, /*R31(0x1F)*/
	0x0000, 0x0000, 0x0031, 0x000b, /*R35*/
	0x0039, 0x0000, 0x0010, 0x0032, /*R39*/
	0x0054, 0x0076, 0x0098, 0x0000, /*R43(0x2B)*/
	0x0000, 0x0000, 0x0000, 0x0000, /*R47*/
	0x0000, 0x0000, 0x005e, 0x003e, /*R51(0x33)*/
	0x0000, 0x0000 /*R53*/
};

/*
 * read wm8580 register cache
 */
static inline unsigned int wm8580_read_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg)
{
	u16 *cache = codec->reg_cache;
	BUG_ON(reg > ARRAY_SIZE(wm8580_reg));
	return cache[reg];
}

/*
 * write wm8580 register cache
 */
static inline void wm8580_write_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg, unsigned int value)
{
	u16 *cache = codec->reg_cache;

	cache[reg] = value;
}

/*
 * write to the WM8580 register space
 */
static int wm8580_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
//	printk("%s,%d : reg : 0x%x, value : 0x%x\n",__FUNCTION__,__LINE__,reg,value);
	u8 data[2];

	BUG_ON(reg > ARRAY_SIZE(wm8580_reg));

	/* Registers are 9 bits wide */
	value &= 0x1ff;

	switch (reg) {
	case WM8580_RESET:
		/* Uncached */
		break;
	default:
		if (value == wm8580_read_reg_cache(codec, reg))
			return 0;
	}

	/* data is
	 *   D15..D9 WM8580 register offset
	 *   D8...D0 register data
	 */
	data[0] = (reg << 1) | ((value >> 8) & 0x0001);
	data[1] = value & 0x00ff;

	wm8580_write_reg_cache(codec, reg, value);
	if (codec->hw_write(codec->control_data, data, 2) == 2)
		return 0;
	else
		return -EIO;
}

static inline unsigned int wm8580_read(struct snd_soc_codec *codec,
				       unsigned int reg)
{
	switch (reg) {
	default:
		return wm8580_read_reg_cache(codec, reg);
	}
}

static const DECLARE_TLV_DB_SCALE(dac_tlv, -12750, 50, 1);

static int wm8580_out_vu(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
        struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
        int reg = kcontrol->private_value & 0xff;
        int ret;
        u16 val;

	/* Clear the register cache so we write without VU set */
	wm8580_write_reg_cache(codec, reg, 0);

        ret = snd_soc_put_volsw(kcontrol, ucontrol);
        if (ret < 0)
                return ret;

	/* Now write again with the volume update bit set */
        val = wm8580_read_reg_cache(codec, reg);
        return wm8580_write(codec, reg, val | 0x0100);
}

#define SOC_WM8580_OUT_SINGLE_R_TLV(xname, reg, shift, max, invert, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ | SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw, \
	.get = snd_soc_get_volsw, .put = wm8580_out_vu, \
	.private_value = SOC_SINGLE_VALUE(reg, shift, max, invert) }

static const struct snd_kcontrol_new wm8580_snd_controls[] = {
SOC_WM8580_OUT_SINGLE_R_TLV("Left DAC1 Playback Volume",
			    WM8580_DIGITAL_ATTENUATION_DACL1,
			    0, 0xff, 0, dac_tlv),
SOC_WM8580_OUT_SINGLE_R_TLV("Right DAC1 Playback Volume",
			    WM8580_DIGITAL_ATTENUATION_DACR1,
			    0, 0xff, 0, dac_tlv),
SOC_WM8580_OUT_SINGLE_R_TLV("Left DAC2 Playback Volume",
			    WM8580_DIGITAL_ATTENUATION_DACL2,
			    0, 0xff, 0, dac_tlv),
SOC_WM8580_OUT_SINGLE_R_TLV("Right DAC2 Playback Volume",
			    WM8580_DIGITAL_ATTENUATION_DACR2,
			    0, 0xff, 0, dac_tlv),
SOC_WM8580_OUT_SINGLE_R_TLV("Left DAC3 Playback Volume",
			    WM8580_DIGITAL_ATTENUATION_DACL3,
			    0, 0xff, 0, dac_tlv),
SOC_WM8580_OUT_SINGLE_R_TLV("Right DAC3 Playback Volume",
			    WM8580_DIGITAL_ATTENUATION_DACR3,
			    0, 0xff, 0, dac_tlv),

SOC_SINGLE("DAC1 Deemphasis Switch", WM8580_DAC_CONTROL3, 0, 1, 0),
SOC_SINGLE("DAC2 Deemphasis Switch", WM8580_DAC_CONTROL3, 1, 1, 0),
SOC_SINGLE("DAC3 Deemphasis Switch", WM8580_DAC_CONTROL3, 2, 1, 0),

SOC_SINGLE("DAC1 Left Invert Switch", WM8580_DAC_CONTROL4,  0, 1, 0),
SOC_SINGLE("DAC1 Right Invert Switch", WM8580_DAC_CONTROL4, 1, 1, 0),
SOC_SINGLE("DAC2 Left Invert Switch", WM8580_DAC_CONTROL4,  2, 1, 0),
SOC_SINGLE("DAC2 Right Invert Switch", WM8580_DAC_CONTROL4, 3, 1, 0),
SOC_SINGLE("DAC3 Left Invert Switch", WM8580_DAC_CONTROL4,  4, 1, 0),
SOC_SINGLE("DAC3 Right Invert Switch", WM8580_DAC_CONTROL4, 5, 1, 0),

SOC_SINGLE("DAC ZC Switch", WM8580_DAC_CONTROL5, 5, 1, 0),
SOC_SINGLE("DAC1 Mute Switch", WM8580_DAC_CONTROL5, 0, 1, 0),
SOC_SINGLE("DAC2 Mute Switch", WM8580_DAC_CONTROL5, 1, 1, 0),
SOC_SINGLE("DAC3 Mute Switch", WM8580_DAC_CONTROL5, 2, 1, 0),

SOC_SINGLE("ADCL Mute Switch", WM8580_ADC_CONTROL1, 0, 1, 0),
SOC_SINGLE("ADCR Mute Switch", WM8580_ADC_CONTROL1, 1, 1, 0),
SOC_SINGLE("ADC High-Pass Filter Switch", WM8580_ADC_CONTROL1, 4, 1, 0),
};

/* Add non-DAPM controls */
static int wm8580_add_controls(struct snd_soc_codec *codec)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(wm8580_snd_controls); i++) {
		err = snd_ctl_add(codec->card,
				  snd_soc_cnew(&wm8580_snd_controls[i],
					       codec, NULL));
		if (err < 0)
			return err;
	}
	return 0;
}
static const struct snd_soc_dapm_widget wm8580_dapm_widgets[] = {
SND_SOC_DAPM_DAC("DAC1", "Playback", WM8580_PWRDN1, 2, 1),
SND_SOC_DAPM_DAC("DAC2", "Playback", WM8580_PWRDN1, 3, 1),
SND_SOC_DAPM_DAC("DAC3", "Playback", WM8580_PWRDN1, 4, 1),

SND_SOC_DAPM_OUTPUT("VOUT1L"),
SND_SOC_DAPM_OUTPUT("VOUT1R"),
SND_SOC_DAPM_OUTPUT("VOUT2L"),
SND_SOC_DAPM_OUTPUT("VOUT2R"),
SND_SOC_DAPM_OUTPUT("VOUT3L"),
SND_SOC_DAPM_OUTPUT("VOUT3R"),

SND_SOC_DAPM_ADC("ADC", "Capture", WM8580_PWRDN1, 1, 1),

SND_SOC_DAPM_INPUT("AINL"),
SND_SOC_DAPM_INPUT("AINR"),
};

static const char *audio_map[][3] = {
	{ "VOUT1L", NULL, "DAC1" },
	{ "VOUT1R", NULL, "DAC1" },

	{ "VOUT2L", NULL, "DAC2" },
	{ "VOUT2R", NULL, "DAC2" },

	{ "VOUT3L", NULL, "DAC3" },
	{ "VOUT3R", NULL, "DAC3" },

	{ "ADC", NULL, "AINL" },
	{ "ADC", NULL, "AINR" },

	{ NULL, NULL, NULL }
};

static int wm8580_add_widgets(struct snd_soc_codec *codec)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(wm8580_dapm_widgets); i++)
		snd_soc_dapm_new_control(codec, &wm8580_dapm_widgets[i]);

	for (i = 0; audio_map[i][0] != NULL; i++)
		snd_soc_dapm_connect_input(codec, audio_map[i][0],
					   audio_map[i][1], audio_map[i][2]);

	snd_soc_dapm_new_widgets(codec);
	return 0;
}

/* PLL divisors */
struct _pll_div {
	u32 prescale:1;
	u32 postscale:1;
	u32 freqmode:2;
	u32 n:4;
	u32 k:24;
};

/* The size in bits of the pll divide */
#define FIXED_PLL_SIZE (1 << 22)

/* PLL rate to output rate divisions */
static struct {
	unsigned int div;
	unsigned int freqmode;
	unsigned int postscale;
} post_table[] = {
	{  2,  0, 0 },
	{  4,  0, 1 },
	{  4,  1, 0 },
	{  8,  1, 1 },
	{  8,  2, 0 },
	{ 16,  2, 1 },
	{ 12,  3, 0 },
	{ 24,  3, 1 }
};

static int pll_factors(struct _pll_div *pll_div, unsigned int target,
		       unsigned int source)
{
	u64 Kpart;
	unsigned int K, Ndiv, Nmod;
	int i;

	dbg("wm8580: PLL %dHz->%dHz\n", source, target);

	/* Scale the output frequency up; the PLL should run in the
	 * region of 90-100MHz.
	 */
	for (i = 0; i < ARRAY_SIZE(post_table); i++) {
		if (target * post_table[i].div >=  90000000 &&
		    target * post_table[i].div <= 100000000) {
			pll_div->freqmode = post_table[i].freqmode;
			pll_div->postscale = post_table[i].postscale;
			target *= post_table[i].div;
			break;
		}
	}

	if (i == ARRAY_SIZE(post_table)) {
		printk(KERN_ERR "wm8580: Unable to scale output frequency "
		       "%u\n", target);
		return -EINVAL;
	}

	Ndiv = target / source;

	if (Ndiv < 5) {
		source /= 2;
		pll_div->prescale = 1;
		Ndiv = target / source;
	} else
		pll_div->prescale = 0;

	if ((Ndiv < 5) || (Ndiv > 13)) {
		printk(KERN_ERR
			"WM8580 N=%d outside supported range\n", Ndiv);
		return -EINVAL;
	}

	pll_div->n = Ndiv;
	Nmod = target % source;
	Kpart = FIXED_PLL_SIZE * (long long)Nmod;

	do_div(Kpart, source);

	K = Kpart & 0xFFFFFFFF;

	pll_div->k = K;

	dbg("PLL %x.%x prescale %d freqmode %d postscale %d\n",
	    pll_div->n, pll_div->k, pll_div->prescale, pll_div->freqmode,
	    pll_div->postscale);

	return 0;
}

static int wm8580_set_dai_pll(struct snd_soc_codec_dai *codec_dai,
		int pll_id, unsigned int freq_in, unsigned int freq_out)
{
	int offset;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct wm8580_priv *wm8580 = codec->private_data;
	struct pll_state *state;
	struct _pll_div pll_div;
	unsigned int reg;
	unsigned int pwr_mask;
	int ret;

	switch (pll_id) {
	case WM8580_PLLA:
		state = &wm8580->a;
		offset = 0;
		pwr_mask = WM8580_PWRDN2_PLLAPD;
		break;
	case WM8580_PLLB:
		state = &wm8580->b;
		offset = 4;
		pwr_mask = WM8580_PWRDN2_PLLBPD;
		break;
	default:
		return -ENODEV;
	}

	if (freq_in && freq_out) {
		ret = pll_factors(&pll_div, freq_out, freq_in);
		if (ret != 0)
			return ret;
	}

	state->in = freq_in;
	state->out = freq_out;

	/* Always disable the PLL - it is not safe to leave it running
	 * while reprogramming it.
	 */
	reg = wm8580_read(codec, WM8580_PWRDN2);
	wm8580_write(codec, WM8580_PWRDN2, reg | pwr_mask);

	if (!freq_in || !freq_out)
		return 0;

	wm8580_write(codec, WM8580_PLLA1 + offset, pll_div.k & 0x1ff);
	wm8580_write(codec, WM8580_PLLA2 + offset, (pll_div.k >> 9) & 0xff);
	wm8580_write(codec, WM8580_PLLA3 + offset,
		     (pll_div.k >> 18 & 0xf) | (pll_div.n << 4));

	reg = wm8580_read(codec, WM8580_PLLA4 + offset);
	reg &= ~0x3f;
	reg |= pll_div.prescale | pll_div.postscale << 1 |
		pll_div.freqmode << 4;
	
	wm8580_write(codec, WM8580_PLLA4 + offset, reg);

	/* All done, turn it on */
	reg = wm8580_read(codec, WM8580_PWRDN2);
	wm8580_write(codec, WM8580_PWRDN2, reg & ~pwr_mask);

	return 0;
}

/*
 * Set PCM DAI bit size and sample rate.
 */
static int wm8580_paif_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *dai = rtd->dai;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	
	u16 paifb = wm8580_read(codec, WM8580_PAIF3 + dai->codec_dai->id);
	
	paifb &= ~WM8580_AIF_LENGTH_MASK;

	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		printk(KERN_CRIT "16 bit\n");
		paifb |= WM8580_AIF_LENGTH_16;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		printk(KERN_CRIT "20 bit\n");
		paifb |= WM8580_AIF_LENGTH_20;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		printk(KERN_CRIT "24 bit\n");
		paifb |= WM8580_AIF_LENGTH_24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		printk(KERN_CRIT "32 bit\n");
		paifb |= WM8580_AIF_LENGTH_24;
		break;
	default:
		return -EINVAL;
	}

	wm8580_write(codec, WM8580_PAIF3 + dai->codec_dai->id, paifb);
	return 0;
}

static int wm8580_set_paif_dai_fmt(struct snd_soc_codec_dai *codec_dai,
				      unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	unsigned int aifa;
	unsigned int aifb;
	int can_invert_lrclk;

	aifa = wm8580_read(codec, WM8580_PAIF1 + codec_dai->id);
	aifb = wm8580_read(codec, WM8580_PAIF3 + codec_dai->id);

	aifb &= ~(WM8580_AIF_FMT_MASK | WM8580_AIF_LRP | WM8580_AIF_BCP);

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		aifa &= ~WM8580_AIF_MS;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		aifa |= WM8580_AIF_MS;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		can_invert_lrclk = 1;
		aifb |= WM8580_AIF_FMT_I2S;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		can_invert_lrclk = 1;
		aifb |= WM8580_AIF_FMT_RIGHTJ;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		can_invert_lrclk = 1;
		aifb |= WM8580_AIF_FMT_LEFTJ;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		can_invert_lrclk = 0;
		aifb |= WM8580_AIF_FMT_DSP;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		can_invert_lrclk = 0;
		aifb |= WM8580_AIF_FMT_DSP;
		aifb |= WM8580_AIF_LRP;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;

	case SND_SOC_DAIFMT_IB_IF:
		if (!can_invert_lrclk)
			return -EINVAL;
		aifb |= WM8580_AIF_BCP;
		aifb |= WM8580_AIF_LRP;
		break;

	case SND_SOC_DAIFMT_IB_NF:
		aifb |= WM8580_AIF_BCP;
		break;

	case SND_SOC_DAIFMT_NB_IF:
		if (!can_invert_lrclk)
			return -EINVAL;
		aifb |= WM8580_AIF_LRP;
		break;

	default:
		return -EINVAL;
	}

	wm8580_write(codec, WM8580_PAIF1 + codec_dai->id, aifa);
	wm8580_write(codec, WM8580_PAIF3 + codec_dai->id, aifb);

	return 0;
}

static int wm8580_set_dai_clkdiv(struct snd_soc_codec_dai *codec_dai,
				 int div_id, int div)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	unsigned int reg;

	switch (div_id) {
	case WM8580_MCLK:
		reg = wm8580_read(codec, WM8580_PLLB4);
		reg &= ~WM8580_PLLB4_MCLKOUTSRC_MASK;

		switch (div) {
		case WM8580_CLKSRC_MCLK:
			/* Input */
			break;

		case WM8580_CLKSRC_PLLA:
			reg |= WM8580_PLLB4_MCLKOUTSRC_PLLA;
			break;
		case WM8580_CLKSRC_PLLB:
			reg |= WM8580_PLLB4_MCLKOUTSRC_PLLB;
			break;

		case WM8580_CLKSRC_OSC:
			reg |= WM8580_PLLB4_MCLKOUTSRC_OSC;
			break;

		default:
			return -EINVAL;
		}
		wm8580_write(codec, WM8580_PLLB4, reg);
		break;

	case WM8580_DAC_CLKSEL:
		reg = wm8580_read(codec, WM8580_CLKSEL);
		reg &= ~WM8580_CLKSEL_DAC_CLKSEL_MASK;

		switch (div) {
		case WM8580_CLKSRC_MCLK:
			break;

		case WM8580_CLKSRC_PLLA:
			reg |= WM8580_CLKSEL_DAC_CLKSEL_PLLA;
			break;

		case WM8580_CLKSRC_PLLB:
			reg |= WM8580_CLKSEL_DAC_CLKSEL_PLLB;
			break;

		default:
			return -EINVAL;
		}
		wm8580_write(codec, WM8580_CLKSEL, reg);
		break;

	case WM8580_CLKOUTSRC:
		reg = wm8580_read(codec, WM8580_PLLB4);
		reg &= ~WM8580_PLLB4_CLKOUTSRC_MASK;

		switch (div) {
		case WM8580_CLKSRC_NONE:
			break;

		case WM8580_CLKSRC_PLLA:
			reg |= WM8580_PLLB4_CLKOUTSRC_PLLACLK;
			break;

		case WM8580_CLKSRC_PLLB:
			reg |= WM8580_PLLB4_CLKOUTSRC_PLLBCLK;
			break;

		case WM8580_CLKSRC_OSC:
			reg |= WM8580_PLLB4_CLKOUTSRC_OSCCLK;
			break;

		default:
			return -EINVAL;
		}
		wm8580_write(codec, WM8580_PLLB4, reg);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int wm8580_dapm_event(struct snd_soc_codec *codec, int event)
{
	u16 reg;
	switch (event) {
	case SNDRV_CTL_POWER_D0: /* full On */
	case SNDRV_CTL_POWER_D1: /* partial On */
	case SNDRV_CTL_POWER_D2: /* partial On */
		break;
	case SNDRV_CTL_POWER_D3hot: /* Off, with power */
		break;
	case SNDRV_CTL_POWER_D3cold: /* Off, without power */
		reg = wm8580_read(codec, WM8580_PWRDN1);
		wm8580_write(codec, WM8580_PWRDN1, reg | WM8580_PWRDN1_PWDN);
		break;
	}
	codec->dapm_state = event;
	return 0;
}

#define WM8580_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
			SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

struct snd_soc_codec_dai wm8580_dai[] = {
	{
		.name = "WM8580 PAIFRX",
		.id = 0,
		.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 6,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = WM8580_FORMATS,
		},
		.capture = {
			.stream_name = "Capture",
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = WM8580_FORMATS,
		},
		.ops = {
			 .hw_params = wm8580_paif_hw_params,
		 },
		.dai_ops = {
			 .set_fmt = wm8580_set_paif_dai_fmt,
			 .set_clkdiv = wm8580_set_dai_clkdiv,
			 .set_pll = wm8580_set_dai_pll,
		 },
	},
#if 0
	{
		.name = "WM8580 PAIFTX",
		.id = 1,
		.capture = {
			.stream_name = "Capture",
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = WM8580_FORMATS,
		},
		.ops = {
			 .hw_params = wm8580_paif_hw_params,
		 },
		.dai_ops = {
			 .set_fmt = wm8580_set_paif_dai_fmt,
			 .set_clkdiv = wm8580_set_dai_clkdiv,
			 .set_pll = wm8580_set_dai_pll,
		 },
	},
#endif
};
EXPORT_SYMBOL_GPL(wm8580_dai);

/*
 * initialise the WM8580 driver
 * register the mixer and dsp interfaces with the kernel
 */
static int wm8580_init(struct snd_soc_device *socdev)
{
	struct snd_soc_codec *codec = socdev->codec;
	int ret = 0;
	int i;

	codec->name = "WM8580";
	codec->owner = THIS_MODULE;
	codec->read = wm8580_read_reg_cache;
	codec->write = wm8580_write;
	codec->dapm_event = wm8580_dapm_event;
	codec->dai = wm8580_dai;
	codec->num_dai = ARRAY_SIZE(wm8580_dai);
	codec->reg_cache_size = ARRAY_SIZE(wm8580_reg);
	codec->reg_cache = kmemdup(wm8580_reg, sizeof(wm8580_reg),
				   GFP_KERNEL);

	if (codec->reg_cache == NULL)
		return -ENOMEM;

	/* Get the codec into a known state */
	wm8580_write(codec, WM8580_RESET, 0);

	/* Power up and get individual control of the DACs */
	wm8580_write(codec, WM8580_PWRDN1, wm8580_read(codec, WM8580_PWRDN1) &
		     ~(WM8580_PWRDN1_PWDN | WM8580_PWRDN1_ALLDACPD));

	/* Make VMID high impedence */
	wm8580_write(codec, WM8580_ADC_CONTROL1,
		     wm8580_read(codec,  WM8580_ADC_CONTROL1) & ~0x100);

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, 
			       SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "wm8580: failed to create pcms\n");
		goto pcm_err;
	}

/* Added : For Capture Path..*/
	u16 reg;
/* Line-In */
	wm8580_write(codec, WM8580_ADC_CONTROL1, wm8580_read(codec,WM8580_ADC_CONTROL1) | 0x60);
	wm8580_write(codec, WM8580_SAIF2, wm8580_read(codec,WM8580_SAIF2) & (0<<3) | 0xc2);

	wm8580_add_controls(codec);
	wm8580_add_widgets(codec);

	ret = snd_soc_register_card(socdev);
	if (ret < 0) {
		printk(KERN_ERR "wm8580: failed to register card\n");
		goto card_err;
	}
	return ret;

card_err:
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
pcm_err:
	kfree(codec->reg_cache);
	return ret;
}

/* If the i2c layer weren't so broken, we could pass this kind of data
   around */
static struct snd_soc_device *wm8580_socdev;

#if defined (CONFIG_I2C) || defined (CONFIG_I2C_MODULE)

/*
 * WM8580 2 wire address is determined by GPIO5
 * state during powerup.
 *    low  = 0x1a
 *    high = 0x1b
 */
static unsigned short normal_i2c[] = { 0, I2C_CLIENT_END };

/* Magic definition of all other variables and things */
I2C_CLIENT_INSMOD;

static struct i2c_driver wm8580_i2c_driver;
static struct i2c_client client_template;

static int wm8580_codec_probe(struct i2c_adapter *adap, int addr, int kind)
{
	struct snd_soc_device *socdev = wm8580_socdev;
	struct wm8580_setup_data *setup = socdev->codec_data;
	struct snd_soc_codec *codec = socdev->codec;
	struct i2c_client *i2c;
	int ret;

	if (addr != setup->i2c_address)
		return -ENODEV;

	client_template.adapter = adap;
	client_template.addr = addr;

	i2c =  kmemdup(&client_template, sizeof(client_template), GFP_KERNEL);
	if (i2c == NULL){
		kfree(codec);
		return -ENOMEM;
	}
	i2c_set_clientdata(i2c, codec);
	codec->control_data = i2c;

	ret = i2c_attach_client(i2c);
	if (ret < 0) {
		err("failed to attach codec at addr %x\n", addr);
		goto err;
	}

	ret = wm8580_init(socdev);
	if (ret < 0) {
		err("failed to initialise WM8580\n");
		goto err;
	}

	return ret;

err:
	kfree(codec);
	kfree(i2c);
	return ret;
}

static int wm8580_i2c_detach(struct i2c_client *client)
{
	struct snd_soc_codec *codec = i2c_get_clientdata(client);
	i2c_detach_client(client);
	kfree(codec->reg_cache);
	kfree(client);
	return 0;
}

static int wm8580_i2c_attach(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data, wm8580_codec_probe);
}

/* corgi i2c codec control layer */
static struct i2c_driver wm8580_i2c_driver = {
	.driver = {
		.name = "WM8580 I2C Codec",
		.owner = THIS_MODULE,
	},
	.id =             I2C_DRIVERID_WM8580,
	.attach_adapter = wm8580_i2c_attach,
	.detach_client =  wm8580_i2c_detach,
	.command =        NULL,
};

static struct i2c_client client_template = {
	.name =   "WM8580",
	.driver = &wm8580_i2c_driver,
};
#endif

static int wm8580_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct wm8580_setup_data *setup;
	struct snd_soc_codec *codec;
	struct wm8580_priv *wm8580;
	int ret = 0;

	info("WM8580 Audio Codec %s\n", WM8580_VERSION);

	setup = socdev->codec_data;
	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

	wm8580 = kzalloc(sizeof(struct wm8580_priv), GFP_KERNEL);
	if (wm8580 == NULL) {
		kfree(codec);
		return -ENOMEM;
	}

	codec->private_data = wm8580;
	socdev->codec = codec;
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);
	wm8580_socdev = socdev;

#if defined (CONFIG_I2C) || defined (CONFIG_I2C_MODULE)
	if (setup->i2c_address) {
		normal_i2c[0] = setup->i2c_address;
		codec->hw_write = (hw_write_t)i2c_master_send;
		ret = i2c_add_driver(&wm8580_i2c_driver);
		if (ret != 0)
			printk(KERN_ERR "can't add i2c driver");
	}
#else
		/* Add other interfaces here */
#endif
	return ret;
}

/* power down chip */
static int wm8580_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	if (codec->control_data)
		wm8580_dapm_event(codec, SNDRV_CTL_POWER_D3cold);
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
#if defined (CONFIG_I2C) || defined (CONFIG_I2C_MODULE)
	i2c_del_driver(&wm8580_i2c_driver);
#endif
	kfree(codec->private_data);
	kfree(codec);

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_wm8580 = {
	.probe = 	wm8580_probe,
	.remove = 	wm8580_remove,
};

EXPORT_SYMBOL_GPL(soc_codec_dev_wm8580);

MODULE_DESCRIPTION("ASoC WM8580 driver");
MODULE_AUTHOR("Mark Brown <broonie@opensource.wolfsonmicro.com>");
MODULE_LICENSE("GPL");
