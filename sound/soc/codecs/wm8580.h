/*
 * wm8580.h  --  audio driver for WM8580
 *
 * Copyright 2008 Samsung Electronics.
 * Author: Ryu Euiyoul
 *         ryu.real@gmail.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef _WM8580_H
#define _WM8580_H

#define WM8580_PLLA  1
#define WM8580_PLLB  2

#define WM8580_MCLK       1
#define WM8580_DAC_CLKSEL 2
#define WM8580_CLKOUTSRC  3

#define WM8580_CLKSRC_MCLK 1
#define WM8580_CLKSRC_PLLA 2
#define WM8580_CLKSRC_PLLB 3
#define WM8580_CLKSRC_OSC  4
#define WM8580_CLKSRC_NONE 5

/* WM8580 register space */
#define WM8580_PLLA1                         0x00
#define WM8580_PLLA2                         0x01
#define WM8580_PLLA3                         0x02
#define WM8580_PLLA4                         0x03
#define WM8580_PLLB1                         0x04
#define WM8580_PLLB2                         0x05
#define WM8580_PLLB3                         0x06
#define WM8580_PLLB4                         0x07
#define WM8580_CLKSEL                        0x08
#define WM8580_PAIF1                         0x09
#define WM8580_PAIF2                         0x0A
#define WM8580_SAIF1                         0x0B
#define WM8580_PAIF3                         0x0C
#define WM8580_PAIF4                         0x0D
#define WM8580_SAIF2                         0x0E
#define WM8580_DAC_CONTROL1                  0x0F
#define WM8580_DAC_CONTROL2                  0x10
#define WM8580_DAC_CONTROL3                  0x11
#define WM8580_DAC_CONTROL4                  0x12
#define WM8580_DAC_CONTROL5                  0x13
#define WM8580_DIGITAL_ATTENUATION_DACL1     0x14
#define WM8580_DIGITAL_ATTENUATION_DACR1     0x15
#define WM8580_DIGITAL_ATTENUATION_DACL2     0x16
#define WM8580_DIGITAL_ATTENUATION_DACR2     0x17
#define WM8580_DIGITAL_ATTENUATION_DACL3     0x18
#define WM8580_DIGITAL_ATTENUATION_DACR3     0x19
#define WM8580_MASTER_DIGITAL_ATTENUATION    0x1C
#define WM8580_ADC_CONTROL1                  0x1D
#define WM8580_SPDTXCHAN0                    0x1E
#define WM8580_SPDTXCHAN1                    0x1F
#define WM8580_SPDTXCHAN2                    0x20
#define WM8580_SPDTXCHAN3                    0x21
#define WM8580_SPDTXCHAN4                    0x22
#define WM8580_SPDTXCHAN5                    0x23
#define WM8580_SPDMODE                       0x24
#define WM8580_INTMASK                       0x25
#define WM8580_GPO1                          0x26
#define WM8580_GPO2                          0x27
#define WM8580_GPO3                          0x28
#define WM8580_GPO4                          0x29
#define WM8580_GPO5                          0x2A
#define WM8580_INTSTAT                       0x2B
#define WM8580_SPDRXCHAN1                    0x2C
#define WM8580_SPDRXCHAN2                    0x2D
#define WM8580_SPDRXCHAN3                    0x2E
#define WM8580_SPDRXCHAN4                    0x2F
#define WM8580_SPDRXCHAN5                    0x30
#define WM8580_SPDSTAT                       0x31
#define WM8580_PWRDN1                        0x32
#define WM8580_PWRDN2                        0x33
#define WM8580_READBACK                      0x34
#define WM8580_RESET                         0x35

/* PLLB4 (register 7h) */
#define WM8580_PLLB4_MCLKOUTSRC_MASK   0x60
#define WM8580_PLLB4_MCLKOUTSRC_PLLA   0x20
#define WM8580_PLLB4_MCLKOUTSRC_PLLB   0x40
#define WM8580_PLLB4_MCLKOUTSRC_OSC    0x60

#define WM8580_PLLB4_CLKOUTSRC_MASK    0x180
#define WM8580_PLLB4_CLKOUTSRC_PLLACLK 0x080
#define WM8580_PLLB4_CLKOUTSRC_PLLBCLK 0x100
#define WM8580_PLLB4_CLKOUTSRC_OSCCLK  0x180

/* CLKSEL (register 8h) */
#define WM8580_CLKSEL_DAC_CLKSEL_MASK 0x03
#define WM8580_CLKSEL_DAC_CLKSEL_PLLA 0x01
#define WM8580_CLKSEL_DAC_CLKSEL_PLLB 0x02

/* AIF control 1 (registers 9h-bh) */
#define WM8580_AIF_RATE_MASK       0x7
#define WM8580_AIF_RATE_128        0x0
#define WM8580_AIF_RATE_192        0x1
#define WM8580_AIF_RATE_256        0x2
#define WM8580_AIF_RATE_384        0x3
#define WM8580_AIF_RATE_512        0x4
#define WM8580_AIF_RATE_768        0x5
#define WM8580_AIF_RATE_1152       0x6

#define WM8580_AIF_BCLKSEL_MASK   0x18
#define WM8580_AIF_BCLKSEL_64     0x00
#define WM8580_AIF_BCLKSEL_128    0x08
#define WM8580_AIF_BCLKSEL_256    0x10
#define WM8580_AIF_BCLKSEL_SYSCLK 0x18

#define WM8580_AIF_MS             0x20

#define WM8580_AIF_CLKSRC_MASK    0xc0
#define WM8580_AIF_CLKSRC_PLLA    0x40
#define WM8580_AIF_CLKSRC_PLLB    0x40
#define WM8580_AIF_CLKSRC_MCLK    0xc0

/* AIF control 2 (registers ch-eh) */
#define WM8580_AIF_FMT_MASK    0x03
#define WM8580_AIF_FMT_RIGHTJ  0x00
#define WM8580_AIF_FMT_LEFTJ   0x01
#define WM8580_AIF_FMT_I2S     0x02
#define WM8580_AIF_FMT_DSP     0x03

#define WM8580_AIF_LENGTH_MASK   0x0c
#define WM8580_AIF_LENGTH_16     0x00
#define WM8580_AIF_LENGTH_20     0x04
#define WM8580_AIF_LENGTH_24     0x08
#define WM8580_AIF_LENGTH_32     0x0c

#define WM8580_AIF_LRP         0x10
#define WM8580_AIF_BCP         0x20

/* Powerdown Register 1 (register 32h) */
#define WM8580_PWRDN1_PWDN     0x001
#define WM8580_PWRDN1_ALLDACPD 0x040

/* Powerdown Register 2 (register 33h) */
#define WM8580_PWRDN2_OSSCPD   0x001
#define WM8580_PWRDN2_PLLAPD   0x002
#define WM8580_PWRDN2_PLLBPD   0x004
#define WM8580_PWRDN2_SPDIFPD  0x008
#define WM8580_PWRDN2_SPDIFTXD 0x010
#define WM8580_PWRDN2_SPDIFRXD 0x020

extern struct snd_soc_codec_device soc_codec_dev_wm8580;

struct wm8580_setup_data {
	unsigned short i2c_address;
};

#define WM8580_DAI_PAIFRX 0
#define WM8580_DAI_PAIFTX 1

extern struct snd_soc_codec_dai wm8580_dai[];
extern struct snd_soc_codec_device soc_codec_dev_wm8580;

#endif

