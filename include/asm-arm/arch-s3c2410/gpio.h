/*
 * linux/include/asm-arm/arch-s3c2410/gpio.h
 *
 * S3C2410 GPIO wrappers for arch-neutral GPIO calls
 *
 * Written by Philipp Zabel <philipp.zabel@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#ifndef __ASM_ARCH_S3C2410_GPIO_H
#define __ASM_ARCH_S3C2410_GPIO_H

#include <linux/kernel.h>

#include <asm/irq.h>
#include <asm/hardware.h>

#if defined(CONFIG_PLAT_S3C24XX)

#include <asm/arch/regs-gpio.h>

static inline int gpio_request(unsigned int gpio, const char *label)
{
	return 0;
}

static inline void gpio_free(unsigned int gpio)
{
	return;
}

static inline int gpio_direction_input(unsigned int gpio)
{
	s3c2410_gpio_cfgpin(gpio, S3C2410_GPIO_INPUT);
	return 0;
}

static inline int gpio_direction_output(unsigned int gpio, int value)
{
	s3c2410_gpio_cfgpin(gpio, S3C2410_GPIO_OUTPUT);
	/* REVISIT can we write the value first, to avoid glitching? */
	s3c2410_gpio_setpin(gpio, value);
	return 0;
}

#define gpio_get_value(gpio)		s3c2410_gpio_getpin(gpio)
#define gpio_set_value(gpio,value)	s3c2410_gpio_setpin(gpio, value)

#include <asm-generic/gpio.h>			/* cansleep wrappers */

#ifdef CONFIG_CPU_S3C2400
#define gpio_to_irq(gpio)		s3c2400_gpio_getirq(gpio)
#else
#define gpio_to_irq(gpio)		s3c2410_gpio_getirq(gpio)
#endif

/* FIXME implement irq_to_gpio() */

#elif defined(CONFIG_PLAT_S3C64XX)

#include <asm/arch/regs-gpio.h>

static inline int gpio_request(unsigned int gpio, const char *label)
{
	return 0;
}

static inline void gpio_free(unsigned int gpio)
{
	return;
}

static inline int gpio_direction_input(unsigned int gpio)
{
	s3c_gpio_cfgpin(gpio, S3C_GPIO_INPUT);

	return 0;
}

static inline int gpio_direction_output(unsigned int gpio, int value)
{
	s3c_gpio_cfgpin(gpio, S3C_GPIO_OUTPUT);

	/* REVISIT can we write the value first, to avoid glitching? */
	s3c_gpio_setpin(gpio, value);

	return 0;
}

static inline void gpio_set_pin(unsigned int gpio, int function)
{
	s3c_gpio_cfgpin(gpio, function);
}

static inline int gpio_get_pin(unsigned int gpio)
{
	return s3c_gpio_getcfg(gpio);
}

static inline void gpio_pullup(unsigned int gpio, unsigned int to)
{
	s3c_gpio_pullup(gpio, to);
}

#define gpio_get_value(gpio)		s3c_gpio_getpin(gpio)
#define gpio_set_value(gpio, value)	s3c_gpio_setpin(gpio, value)

#include <asm-generic/gpio.h>		/* cansleep wrappers */

#define gpio_to_irq(gpio)		s3c_gpio_getirq(gpio)

/* FIXME implement irq_to_gpio() */

#elif defined(CONFIG_PLAT_S5PC1XX)

#include <asm/plat-s5p/regs-gpio.h>

static inline int gpio_request(unsigned int gpio, const char *label)
{
	return 0;
}

static inline void gpio_free(unsigned int gpio)
{
	return;
}

static inline int gpio_direction_input(unsigned int gpio)
{
	s5p_gpio_cfgpin(gpio, S5P_GPIO_INPUT);

	return 0;
}

static inline int gpio_direction_output(unsigned int gpio, int value)
{
	s5p_gpio_cfgpin(gpio, S5P_GPIO_OUTPUT);

	/* REVISIT can we write the value first, to avoid glitching? */
	s5p_gpio_setpin(gpio, value);

	return 0;
}

static inline void gpio_set_pin(unsigned int gpio, int function)
{
	s5p_gpio_cfgpin(gpio, function);
}

static inline void gpio_pullup(unsigned int gpio, unsigned int to)
{
	s5p_gpio_pullup(gpio, to);
}

#define gpio_get_value(gpio)		s5p_gpio_getpin(gpio)
#define gpio_set_value(gpio, value)	s5p_gpio_setpin(gpio, value)

#include <asm-generic/gpio.h>		/* cansleep wrappers */

#define gpio_to_irq(gpio)		s5p_gpio_getirq(gpio)

/* FIXME implement irq_to_gpio() */

#endif	/* end of CONFIG_PLAT_XXX */

#endif	/* end of __ASM_ARCH_S3C2410_GPIO_H */
