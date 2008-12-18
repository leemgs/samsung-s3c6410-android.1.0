/*
 * drivers/video/s3c/s3c24xxfb_spi.c
 *
 * $Id: s3cfb_spi.c,v 1.4 2008/08/27 01:04:36 jsgood Exp $
 *
 * Copyright (C) 2008 Jinsung Yang <jsgood.yang@samsung.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 *	S3C Frame Buffer Driver
 *	based on skeletonfb.c, sa1100fb.h, s3c2410fb.c
 */

#include <linux/delay.h>

#include <asm/mach/map.h>
#include <asm/arch/gpio.h>
#include <asm/arch/regs-lcd.h>
#include <asm/arch/regs-s3c-clock.h>

#if defined(CONFIG_PLAT_S3C24XX)

#include <asm/arch/regs-gpio.h>
#include <asm/arch/regs-gpioj.h>

#define S3C_FB_SPI_CLK(x)	(S3C2443_GPL10 + (ch * 0))
#define S3C_FB_SPI_MOSI(x)	(S3C2443_GPL11 + (ch * 0))
#define S3C_FB_SPI_CS(x)	(S3C2443_GPL14 + (ch * 0))

static inline void s3cfb_spi_lcd_dclk(int ch, int value)
{
	s3c2410_gpio_setpin(S3C_FB_SPI_CLK(ch), value);
}

static inline void s3cfb_spi_lcd_dseri(int ch, int value)
{
	s3c2410_gpio_setpin(S3C_FB_SPI_MOSI(ch), value);
}

static inline void s3cfb_spi_lcd_den(int ch, int value)
{
	s3c2410_gpio_setpin(S3C_FB_SPI_CS(ch), value);
}

static inline void s3cfb_spi_set_lcd_data(int ch)
{
	s3c2410_gpio_cfgpin(S3C_FB_SPI_CLK(ch), 1);
	s3c2410_gpio_cfgpin(S3C_FB_SPI_MOSI(ch), 1);
	s3c2410_gpio_cfgpin(S3C_FB_SPI_CS(ch), 1);

	s3c2410_gpio_pullup(S3C_FB_SPI_CLK(ch), 2);
	s3c2410_gpio_pullup(S3C_FB_SPI_MOSI(ch), 2);
	s3c2410_gpio_pullup(S3C_FB_SPI_CS(ch), 2);
}

#elif defined(CONFIG_PLAT_S3C64XX)

#include <asm/arch/regs-gpio.h>
#include <asm/arch/regs-gpioj.h>

#define S3C_FB_SPI_CLK(x)	(S3C_GPC1 + (ch * 4))
#define S3C_FB_SPI_MOSI(x)	(S3C_GPC2 + (ch * 4))
#define S3C_FB_SPI_CS(x)	(S3C_GPC3 + (ch * 4))

inline void s3cfb_spi_lcd_dclk(int ch, int value)
{
	gpio_set_value(S3C_FB_SPI_CLK(ch), value);
}

inline void s3cfb_spi_lcd_dseri(int ch, int value)
{
	gpio_set_value(S3C_FB_SPI_MOSI(ch), value);
}

inline void s3cfb_spi_lcd_den(int ch, int value)
{
	gpio_set_value(S3C_FB_SPI_CS(ch), value);
}

inline void s3cfb_spi_set_lcd_data(int ch)
{
	gpio_direction_output(S3C_FB_SPI_CLK(ch), 1);
	gpio_direction_output(S3C_FB_SPI_MOSI(ch), 1);
	gpio_direction_output(S3C_FB_SPI_CS(ch), 1);

	gpio_pullup(S3C_FB_SPI_CLK(ch), 0);
	gpio_pullup(S3C_FB_SPI_MOSI(ch), 0);
	gpio_pullup(S3C_FB_SPI_CS(ch), 0);
}

#elif defined(CONFIG_PLAT_S5PC1XX)

#include <asm/plat-s5p/regs-gpio.h>

#define S5P_FB_SPI_MISO(x)	(S5P_GPB0 + (ch * 4))
#define S5P_FB_SPI_CLK(x)	(S5P_GPB1 + (ch * 4))
#define S5P_FB_SPI_MOSI(x)	(S5P_GPB2 + (ch * 4))
#define S5P_FB_SPI_nSS(x)	(S5P_GPB3 + (ch * 4))

inline void s3cfb_spi_lcd_dclk(int ch, int value)
{
	gpio_set_value(S5P_FB_SPI_CLK(ch), value);
}

inline void s3cfb_spi_lcd_dseri(int ch, int value)
{
	gpio_set_value(S5P_FB_SPI_MOSI(ch), value);
}

inline void s3cfb_spi_lcd_den(int ch, int value)
{
	gpio_set_value(S5P_FB_SPI_nSS(ch), value);
}

inline void s3cfb_spi_set_lcd_data(int ch)
{
	gpio_direction_output(S5P_FB_SPI_CLK(ch), 1);
	gpio_direction_output(S5P_FB_SPI_MOSI(ch), 1);
	gpio_direction_output(S5P_FB_SPI_nSS(ch), 1);

	gpio_pullup(S5P_FB_SPI_CLK(ch), S5P_GPIO_PUD_DISABLE);
	gpio_pullup(S5P_FB_SPI_MOSI(ch), S5P_GPIO_PUD_DISABLE);
	gpio_pullup(S5P_FB_SPI_nSS(ch), S5P_GPIO_PUD_DISABLE);
}

#endif

