/*
 * smdk2450_wm8580.c  --  SoC audio for Neo1973
 *
 * Copyright 2007 Wolfson Microelectronics PLC.
 * Author: Graeme Gregory
 *         graeme.gregory@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 *  Copyright (C) 2007, Ryu Euiyoul <ryu.real@gmail.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *    20th Jan 2007   Initial version.
 *    05th Feb 2007   Rename all to Neo1973
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <asm/hardware/scoop.h>
#include <asm-arm/plat-s3c24xx/regs-iis.h>

#include <asm/arch/regs-gpio.h>
#include <asm/hardware.h>
#include <asm/arch/audio.h>
#include <asm/io.h>
#include <asm/arch/spi-gpio.h>
#include <asm/arch/regs-s3c-clock.h>

#include "../codecs/wm8580.h"
#include "s3c-pcm.h"
#include "s3c-i2s.h"

/* define the scenarios */
#define SMDK_AUDIO_OFF		0
#define SMDK_CAPTURE_MIC1		3
#define SMDK_STEREO_TO_HEADPHONES	2
#define SMDK_CAPTURE_LINE_IN	1

#ifdef CONFIG_SND_DEBUG
#define s3cdbg(x...) printk(x)
#else
#define s3cdbg(x...)
#endif

static int smdk_hifi_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	//struct snd_soc_codec_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_cpu_dai *cpu_dai = rtd->dai->cpu_dai;
	unsigned int pll_out = 0;
	int ret = 0;
	unsigned int regs;
	unsigned int prescaler=0;

	s3cdbg("Entered %s, rate = %d\n", __FUNCTION__, params_rate(params));

	/* Select Clock source EPLL */
	regs = readl(S3C2443_CLKSRC);
	regs &= ~(3<<14);
	regs |= S3C2443_CLKSRC_I2S_EPLLDIV;
	regs = (regs & ~(3<<7))|(2<<7);
	writel(regs, S3C2443_CLKSRC);
	regs |= (1<<6);
	writel(regs, S3C2443_CLKSRC);

	regs = readl(S3C2443_SCLKCON);
	regs |= S3C2443_SCLKCON_I2SCLK;
	writel(regs, S3C2443_SCLKCON);

	s3cdbg("%s: %d , params = %d \n", __FUNCTION__, __LINE__, params_rate(params));

	switch (params_rate(params)) {
	case 8000:
	case 16000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
		writel(9962, S3C2450_EPLLCON_K);
		writel((49<<16)|(1<<8)|(3<<0) ,S3C2443_EPLLCON);
		break;
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		writel(10381, S3C2450_EPLLCON_K);
		writel((45<<16)|(1<<8)|(2<<0) ,S3C2443_EPLLCON);
		break;
	default:
		printk("Unsupported rate = %d\n", params_rate(params));
		break;
	}

	switch (params_rate(params)) {
	case 8000:
		pll_out = 18432000;
		prescaler = 24; 
		break;
	case 11025:
		pll_out = 16934400;
		prescaler = 32; 
		break;
	case 16000:
		pll_out = 18432000;
		prescaler = 12; 
		break;
	case 22050:
		pll_out = 16934400;
		prescaler = 16; 
		break;
	case 32000:
		pll_out = 18432000;
		prescaler = 6; 
		break;
	case 44100:
		pll_out = 16934400;
		prescaler = 8; 
		break;
	case 48000:
		pll_out = 18432000;
		prescaler = 4; 
		break;
	case 64000:
		pll_out = 18432000;
		prescaler = 3; 
		break;
	case 88200:
		pll_out = 16934400;
		prescaler = 4; 
		break;
	case 96000:
		pll_out = 18432000;
		prescaler = 2; 
		break;
	}

	/* set cpu DAI configuration */
	ret = cpu_dai->dai_ops.set_fmt(cpu_dai,
		SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		SND_SOC_DAIFMT_CBS_CFS ); 
	if (ret < 0)
		return ret;

	/* set MCLK division for sample rate */
	ret = cpu_dai->dai_ops.set_clkdiv(cpu_dai, S3C24XX_DIV_MCLK,
		S3C2410_IISMOD_32FS );
	if (ret < 0)
		return ret;

	/* set prescaler division for sample rate */
	ret = cpu_dai->dai_ops.set_clkdiv(cpu_dai, S3C24XX_DIV_PRESCALER,
	//	((prescaler/2 - 1) << 0x8));
		((prescaler - 1) << 0x8));
	if (ret < 0)
		return ret;

	return 0;

}

static int smdk_hifi_hw_free(struct snd_pcm_substream *substream)
{
//	struct snd_soc_pcm_runtime *rtd = substream->private_data;
//	struct snd_soc_codec_dai *codec_dai = rtd->dai->codec_dai;

	/* disable the PLL */
	//return codec_dai->dai_ops.set_pll(codec_dai, WM8580_PLL1, 0, 0);
	//return codec_dai->dai_ops.set_pll(codec_dai, 0, 0, 0);
	return 0; 
}

/*
 * Neo1973 WM8580 HiFi DAI opserations.
 */
static struct snd_soc_ops smdk_hifi_ops = {
	.hw_params = smdk_hifi_hw_params,
	.hw_free = smdk_hifi_hw_free,
};

static int smdk_scenario = 0;

static int smdk_get_scenario(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = smdk_scenario;
	return 0;
}

static int set_scenario_endpoints(struct snd_soc_codec *codec, int scenario)
{
	switch(smdk_scenario) {
	case SMDK_AUDIO_OFF:
		snd_soc_dapm_set_endpoint(codec, "Headphone Jack",    0);
		snd_soc_dapm_set_endpoint(codec, "Mic1 Jack",  0);
		snd_soc_dapm_set_endpoint(codec, "Line In Jack",  0);
		break;
	case SMDK_STEREO_TO_HEADPHONES:
		snd_soc_dapm_set_endpoint(codec, "Headphone Jack",    1);
		snd_soc_dapm_set_endpoint(codec, "Mic1 Jack",  0);
		snd_soc_dapm_set_endpoint(codec, "Line In Jack",  0);
		break;
	case SMDK_CAPTURE_MIC1:
		snd_soc_dapm_set_endpoint(codec, "Headphone Jack",    0);
		snd_soc_dapm_set_endpoint(codec, "Mic1 Jack",  1);
		snd_soc_dapm_set_endpoint(codec, "Line In Jack",  0);
		break;
	case SMDK_CAPTURE_LINE_IN:
		snd_soc_dapm_set_endpoint(codec, "Headphone Jack",    0);
		snd_soc_dapm_set_endpoint(codec, "Mic1 Jack",  0);
		snd_soc_dapm_set_endpoint(codec, "Line In Jack",  1);
		break;
	default:
		snd_soc_dapm_set_endpoint(codec, "Headphone Jack",    1);
		snd_soc_dapm_set_endpoint(codec, "Mic1 Jack",  1);
		snd_soc_dapm_set_endpoint(codec, "Line In Jack",  1);
		break;
	}

	snd_soc_dapm_sync_endpoints(codec);

	return 0;
}

static int smdk_set_scenario(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	if (smdk_scenario == ucontrol->value.integer.value[0])
		return 0;

	smdk_scenario = ucontrol->value.integer.value[0];
	set_scenario_endpoints(codec, smdk_scenario);
	return 1;
}

static const struct snd_soc_dapm_widget wm8580_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_MIC("Mic1 Jack", NULL),
	SND_SOC_DAPM_LINE("Line In Jack", NULL),
};


/* example machine audio_mapnections */
static const char* audio_map[][3] = {

	{"Headphone Jack", NULL, "LOUT1"},
	{"Headphone Jack", NULL, "ROUT1"},

	/* mic is connected to mic1 - with bias */
	/* mic is connected to mic1 - with bias */
	{"MIC1", NULL, "Mic1 Jack"},

	{"LINE1", NULL, "Line In Jack"},
	{"LINE2", NULL, "Line In Jack"},

	/* Connect the ALC pins */
	{"ACIN", NULL, "ACOP"},
		
	{NULL, NULL, NULL},
};

static const char *smdk_scenarios[] = {
	"Off",
	"Capture Line In",
	"Headphones",
	"Capture Mic1",
};

static const struct soc_enum smdk_scenario_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(smdk_scenarios),smdk_scenarios),
};

static const struct snd_kcontrol_new wm8580_smdk_controls[] = {
	SOC_ENUM_EXT("SMDK Mode", smdk_scenario_enum[0],
		smdk_get_scenario, smdk_set_scenario),
};

/*
 * This is an example machine initialisation for a wm8580 connected to a
 * smdk2450. It is missing logic to detect hp/mic insertions and logic
 * to re-route the audio in such an event.
 */
static int smdk_wm8580_init(struct snd_soc_codec *codec)
{
	int i, err;

	/* set endpoints to default mode */
	set_scenario_endpoints(codec, SMDK_AUDIO_OFF);

	/* Add smdk2450 specific widgets */
	for (i = 0; i < ARRAY_SIZE(wm8580_dapm_widgets); i++)
		snd_soc_dapm_new_control(codec, &wm8580_dapm_widgets[i]);

	/* add smdk2450 specific controls */
	for (i = 0; i < ARRAY_SIZE(wm8580_smdk_controls); i++) {
		err = snd_ctl_add(codec->card,
				snd_soc_cnew(&wm8580_smdk_controls[i],
				codec, NULL));
		if (err < 0)
			return err;
	}

	/* set up smdk2450 specific audio path audio_mapnects */
	for (i = 0; audio_map[i][0] != NULL; i++) {
		snd_soc_dapm_connect_input(codec, audio_map[i][0],
			audio_map[i][1], audio_map[i][2]);
	}

	/* always connected */
	snd_soc_dapm_set_endpoint(codec, "Mic1 Jack", 1);
	snd_soc_dapm_set_endpoint(codec, "Headphone Jack", 1);
	snd_soc_dapm_set_endpoint(codec, "Line In Jack", 1);

	snd_soc_dapm_sync_endpoints(codec);
	return 0;
}

static struct snd_soc_dai_link smdk_dai[] = {
{ /* Hifi Playback - for similatious use with voice below */
	.name = "WM8580",
	.stream_name = "WM8580 HiFi",
	.cpu_dai = &s3c_i2s_dai,
	.codec_dai = &wm8580_dai[0],
	.init = smdk_wm8580_init,
	.ops = &smdk_hifi_ops,
},
};

static struct snd_soc_machine smdk2450 = {
	.name = "smdk2450",
	.dai_link = smdk_dai,
	.num_links = ARRAY_SIZE(smdk_dai),
};

static struct wm8580_setup_data smdk_wm8580_setup = {
	/* wm8580 i2c address 0x36 for write*/
	.i2c_address = 0x1b,
};

static struct snd_soc_device smdk_snd_devdata = {
	.machine = &smdk2450,
	.platform = &s3c24xx_soc_platform,
	.codec_dev = &soc_codec_dev_wm8580,
	.codec_data = &smdk_wm8580_setup,
};

static struct platform_device *smdk_snd_device;

static int __init s3c_init(void)
{
	int ret;

	smdk_snd_device = platform_device_alloc("soc-audio", -1);
	if (!smdk_snd_device)
		return -ENOMEM;

	platform_set_drvdata(smdk_snd_device, &smdk_snd_devdata);
	smdk_snd_devdata.dev = &smdk_snd_device->dev;
	ret = platform_device_add(smdk_snd_device);

	if (ret)
		platform_device_put(smdk_snd_device);
	
	return ret;
}

static void __exit s3c_exit(void)
{
	platform_device_unregister(smdk_snd_device);
}

module_init(s3c_init);
module_exit(s3c_exit);

/* Module information */
MODULE_AUTHOR("Ryu Euiyoul");
MODULE_DESCRIPTION("ALSA SoC WM8580 SMDK");
MODULE_LICENSE("GPL");
