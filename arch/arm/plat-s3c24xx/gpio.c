/* linux/arch/arm/plat-s3c24xx/gpio.c
 *
 * Copyright (c) 2004-2005 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * S3C24XX GPIO support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/


#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>

#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/io.h>

#include <asm/arch/regs-gpio.h>

void s3c2410_gpio_cfgpin(unsigned int pin, unsigned int function)
{
	void __iomem *base = S3C24XX_GPIO_BASE(pin);
	unsigned long mask;
	unsigned long con;
	unsigned long flags;

	if (pin < S3C2410_GPIO_BANKB) {
		mask = 1 << S3C2410_GPIO_OFFSET(pin);
	} else {
		mask = 3 << S3C2410_GPIO_OFFSET(pin)*2;
	}

	switch (function) {
	case S3C2410_GPIO_LEAVE:
		mask = 0;
		function = 0;
		break;

	case S3C2410_GPIO_INPUT:
	case S3C2410_GPIO_OUTPUT:
	case S3C2410_GPIO_SFN2:
	case S3C2410_GPIO_SFN3:
		if (pin < S3C2410_GPIO_BANKB) {
			function -= 1;
			function &= 1;
			function <<= S3C2410_GPIO_OFFSET(pin);
		} else {
			function &= 3;
			function <<= S3C2410_GPIO_OFFSET(pin)*2;
		}
	}

	/* modify the specified register wwith IRQs off */

	local_irq_save(flags);
	local_irq_disable();

	con  = __raw_readl(base + 0x00);
	con &= ~mask;
	con |= function;

	__raw_writel(con, base + 0x00);

	local_irq_restore(flags);
}

EXPORT_SYMBOL(s3c2410_gpio_cfgpin);

unsigned int s3c2410_gpio_getcfg(unsigned int pin)
{
	void __iomem *base = S3C24XX_GPIO_BASE(pin);
	unsigned long val = __raw_readl(base);

	if (pin < S3C2410_GPIO_BANKB) {
		val >>= S3C2410_GPIO_OFFSET(pin);
		val &= 1;
		val += 1;
	} else {
		val >>= S3C2410_GPIO_OFFSET(pin)*2;
		val &= 3;
	}

	return val | S3C2410_GPIO_INPUT;
}

EXPORT_SYMBOL(s3c2410_gpio_getcfg);

#if defined(CONFIG_CPU_S3C2443) || defined(CONFIG_CPU_S3C2450) || defined(CONFIG_CPU_S3C2416)
void s3c2410_gpio_pullup(unsigned int pin, unsigned int to)
{
	void __iomem *base = S3C24XX_GPIO_BASE(pin);
	unsigned long offs = S3C2410_GPIO_OFFSET(pin)*2;
	unsigned long flags;
	unsigned long up;

	if (pin < S3C2410_GPIO_BANKB)
		return;

	local_irq_save(flags);
	local_irq_disable();

	up = __raw_readl(base + 0x08);
	up &= ~(0x3 << offs);
	up |= to << offs;
	__raw_writel(up, base + 0x08);

	local_irq_restore(flags);
}
#else
void s3c2410_gpio_pullup(unsigned int pin, unsigned int to)
{
	void __iomem *base = S3C24XX_GPIO_BASE(pin);
	unsigned long offs = S3C2410_GPIO_OFFSET(pin);
	unsigned long flags;
	unsigned long up;

	if (pin < S3C2410_GPIO_BANKB)
		return;

	local_irq_save(flags);
	local_irq_disable();

	up = __raw_readl(base + 0x08);
	up &= ~(1L << offs);
	up |= to << offs;
	__raw_writel(up, base + 0x08);

	local_irq_restore(flags);
}
#endif

EXPORT_SYMBOL(s3c2410_gpio_pullup);

void s3c2410_gpio_setpin(unsigned int pin, unsigned int to)
{
	void __iomem *base = S3C24XX_GPIO_BASE(pin);
	unsigned long offs = S3C2410_GPIO_OFFSET(pin);
	unsigned long flags;
	unsigned long dat;

	local_irq_save(flags);
	local_irq_disable();

	dat = __raw_readl(base + 0x04);
	dat &= ~(1 << offs);
	dat |= to << offs;
	__raw_writel(dat, base + 0x04);

	local_irq_restore(flags);
}

EXPORT_SYMBOL(s3c2410_gpio_setpin);

unsigned int s3c2410_gpio_getpin(unsigned int pin)
{
	void __iomem *base = S3C24XX_GPIO_BASE(pin);
	unsigned long offs = S3C2410_GPIO_OFFSET(pin);

	return __raw_readl(base + 0x04) & (1<< offs);
}

EXPORT_SYMBOL(s3c2410_gpio_getpin);

unsigned int s3c2410_modify_misccr(unsigned int clear, unsigned int change)
{
	unsigned long flags;
	unsigned long misccr;

	local_irq_save(flags);
	local_irq_disable();
	misccr = __raw_readl(S3C24XX_MISCCR);
	misccr &= ~clear;
	misccr ^= change;
	__raw_writel(misccr, S3C24XX_MISCCR);
	local_irq_restore(flags);

	return misccr;
}

EXPORT_SYMBOL(s3c2410_modify_misccr);

int s3c2410_gpio_getirq(unsigned int pin)
{
	if (pin < S3C2410_GPF0 || pin > S3C2410_GPG15)
		return -1;	/* not valid interrupts */

	if (pin < S3C2410_GPG0 && pin > S3C2410_GPF7)
		return -1;	/* not valid pin */

	if (pin < S3C2410_GPF4)
		return (pin - S3C2410_GPF0) + IRQ_EINT0;

	if (pin < S3C2410_GPG0)
		return (pin - S3C2410_GPF4) + IRQ_EINT4;

	return (pin - S3C2410_GPG0) + IRQ_EINT8;
}

EXPORT_SYMBOL(s3c2410_gpio_getirq);


#if defined(CONFIG_PLAT_S3C64XX) || defined(CONFIG_PLAT_S5PC1XX)
/* --------------------------------------------------------------
 *                             Set up GPIOs
 *-------------------------------------------------------------*/
static void __iomem *gpio_base_offset[]=
{
	S3C24XX_VA_GPIO + 0x00,		//GPA ,4bit
	S3C24XX_VA_GPIO + 0x20,		//GPB ,4bit
	S3C24XX_VA_GPIO + 0x40,		//GPC ,4bit
	S3C24XX_VA_GPIO + 0x60,		//GPD ,4bit
	S3C24XX_VA_GPIO + 0x80,		//GPE ,4bit
	S3C24XX_VA_GPIO + 0xA0,		//GPF ,2bit
	S3C24XX_VA_GPIO + 0xC0,		//GPG ,4bit
	S3C24XX_VA_GPIO + 0xE0,		//GPH ,4bit
	S3C24XX_VA_GPIO + 0x100,	//GPI ,2bit
	S3C24XX_VA_GPIO + 0x120,	//GPJ ,2bit
	S3C24XX_VA_GPIO + 0x140,	//GPO ,2bit
	S3C24XX_VA_GPIO + 0x160,	//GPP ,2bit
	S3C24XX_VA_GPIO + 0x180,	//GPQ ,2bit
	S3C24XX_VA_GPIO + 0x800,	//GPK ,4bit
	S3C24XX_VA_GPIO + 0x810,	//GPL ,4bit
	S3C24XX_VA_GPIO + 0x820,	//GPM ,4bit
	S3C24XX_VA_GPIO + 0x830		//GPN ,2bit
};

void s3c_gpio_cfgpin(unsigned int pin, unsigned int function)
{
	void __iomem *base = gpio_base_offset[S3C_GPIO_BASE(pin)];
	unsigned long mask;
	unsigned long con;
	unsigned long flags;
	unsigned long offs =  S3C_GPIO_OFFSET(pin);

	if ((pin < S3C_GPIO_BANKF)||((pin >=S3C_GPIO_BANKG)&&\
		(pin<S3C_GPIO_BANKI))||((pin>=S3C_GPIO_BANKK)&&\
		(pin<S3C_GPIO_BANKN)))
	{
		offs = (offs) *4;

		if((pin == S3C_GPH8)||(pin==S3C_GPH9)||\
			(pin>=S3C_GPK8&&pin<=S3C_GPK15)||\
			(pin>=S3C_GPL8&&pin<=S3C_GPL14))
		{
			base = base + 0x04;
  	           	/*  only for con1:8~14 or 15 regiter configuratio nvalue change ...*/
			offs = offs - 32; 
		}
		mask = 0xF << offs;
	}
	else
	{
	       offs = offs*2;
		mask = 0x3 << offs;
	}

	local_irq_save(flags);
	local_irq_disable();

	con  = __raw_readl(base + 0x00);
	con &= ~mask;
	con |= (function << offs);

	__raw_writel(con, base + 0x00);

	local_irq_restore(flags);
}

EXPORT_SYMBOL(s3c_gpio_cfgpin);

unsigned int s3c_gpio_getcfg(unsigned int pin)
{
	void __iomem *base = gpio_base_offset[S3C_GPIO_BASE(pin)];
	unsigned long mask;
	unsigned long shift=0;

	if ((pin < S3C_GPIO_BANKF)||((pin >=S3C_GPIO_BANKG)&&\
		(pin<S3C_GPIO_BANKI))||((pin>=S3C_GPIO_BANKK)&&\
		(pin<S3C_GPIO_BANKN)))
	{
		mask = 0xF << S3C_GPIO_OFFSET(pin)*4;
		if((pin == S3C_GPH8)||(pin==S3C_GPH9)||\
			(pin>=S3C_GPK8&&pin<=S3C_GPK15)||\
			(pin>=S3C_GPL8&&pin<=S3C_GPL14))
		{
			base = base + 0x04;
			shift = S3C_GPIO_OFFSET(pin)*4 - 32; 
			mask = 0xF << shift; 
		}

	}
	else
	{
		shift = S3C_GPIO_OFFSET(pin)*2;
		mask = 0x3 << shift;
	}

	return ((__raw_readl(base) & mask) >> shift);
}

EXPORT_SYMBOL(s3c_gpio_getcfg);

void s3c_gpio_pullup(unsigned int pin, unsigned int to)
{
	void __iomem *base = gpio_base_offset[S3C_GPIO_BASE(pin)];
	unsigned long offs = S3C_GPIO_OFFSET(pin)*2;
	unsigned long flags;
	unsigned long up;
	unsigned long mask;

	mask = 0x3 << offs;

	if((pin>=S3C_GPH0 && pin<=S3C_GPH9)||\
		(pin>=S3C_GPK0 && pin<=S3C_GPK15)||\
		(pin>=S3C_GPL0 && pin<=S3C_GPL14))
	{
		base = base+0x04;
	}

	local_irq_save(flags);
	local_irq_disable();

	up = __raw_readl(base + 0x08);
	up &= ~mask;
	up |= to << offs;
	__raw_writel(up, base + 0x08);

	local_irq_restore(flags);
}

EXPORT_SYMBOL(s3c_gpio_pullup);

void s3c_gpio_setpin(unsigned int pin, unsigned int to)
{
	void __iomem *base = gpio_base_offset[S3C_GPIO_BASE(pin)];
	unsigned long offs = S3C_GPIO_OFFSET(pin);
	unsigned long flags;
	unsigned long dat;

	if((pin>=S3C_GPH0 && pin<=S3C_GPH9)||\
		(pin>=S3C_GPK0 && pin<=S3C_GPK15)||\
		(pin>=S3C_GPL0 && pin<=S3C_GPL14))
	{
		base = base+0x04;
	}

	local_irq_save(flags);
	local_irq_disable();

	dat = __raw_readl(base + 0x04);
	dat &= ~(1 << offs);
	dat |= to << offs;
	__raw_writel(dat, base + 0x04);

	local_irq_restore(flags);
}

EXPORT_SYMBOL(s3c_gpio_setpin);

unsigned int s3c_gpio_getpin(unsigned int pin)
{
	void __iomem *base = gpio_base_offset[S3C_GPIO_BASE(pin)];
	unsigned long offs = S3C_GPIO_OFFSET(pin);

	if((pin>=S3C_GPH0 && pin<=S3C_GPH9)||\
		(pin>=S3C_GPK0 && pin<=S3C_GPK15)||\
		(pin>=S3C_GPL0 && pin<=S3C_GPL14))
	{
		base = base+0x04;
	}
	return __raw_readl(base + 0x04) & (1<< offs);
}

EXPORT_SYMBOL(s3c_gpio_getpin);

int s3c_gpio_getirq(unsigned int pin)
{

/* mandatory */
/* Implement this function */
	return 0;
}

EXPORT_SYMBOL(s3c_gpio_getirq);

int s3c_gpio_irqfilter(unsigned int pin, unsigned int on,
			   unsigned int config)
{

	return 0;
}

EXPORT_SYMBOL(s3c_gpio_irqfilter);
#endif
