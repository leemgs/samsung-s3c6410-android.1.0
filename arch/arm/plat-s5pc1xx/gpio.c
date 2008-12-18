/* linux/arch/arm/plat-s5p/gpio.c
 *
 * Copyright (c) 2008 Samsung Electronics
 *
 * S5PC1XX GPIO support
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

#include <asm/plat-s5p/regs-gpio.h>

/* GPIO bank list */
static void __iomem *gpio_base_offset[] = {
	S3C24XX_VA_GPIO + 0x00,		/* GPA0 */
	S3C24XX_VA_GPIO + 0x20,		/* GPA1 */
	S3C24XX_VA_GPIO + 0x40,		/* GPB */
	S3C24XX_VA_GPIO + 0x60,		/* GPC */
	S3C24XX_VA_GPIO + 0x80,		/* GPD */
	S3C24XX_VA_GPIO + 0xA0,		/* GPE0 */
	S3C24XX_VA_GPIO + 0xC0,		/* GPE1 */
	S3C24XX_VA_GPIO + 0xE0,		/* GPF0 */
	S3C24XX_VA_GPIO + 0x100,	/* GPF1 */
	S3C24XX_VA_GPIO + 0x120,	/* GPF2 */
	S3C24XX_VA_GPIO + 0x140,	/* GPF3 */
	S3C24XX_VA_GPIO + 0x160,	/* GPG0 */
	S3C24XX_VA_GPIO + 0x180,	/* GPG1 */
	S3C24XX_VA_GPIO + 0x1A0,	/* GPG2 */
	S3C24XX_VA_GPIO + 0x1C0,	/* GPG3 */
	S3C24XX_VA_GPIO + 0xC00,	/* GPH0 */
	S3C24XX_VA_GPIO + 0xC20,	/* GPH1 */
	S3C24XX_VA_GPIO + 0xC40,	/* GPH1 */
	S3C24XX_VA_GPIO + 0xC60,	/* GPH1 */
	S3C24XX_VA_GPIO + 0x1E0,	/* GPI */
	S3C24XX_VA_GPIO + 0x200,	/* GPJ0 */
	S3C24XX_VA_GPIO + 0x220,	/* GPJ1 */
	S3C24XX_VA_GPIO + 0x240,	/* GPJ2 */
	S3C24XX_VA_GPIO + 0x260,	/* GPJ3 */
	S3C24XX_VA_GPIO + 0x280,	/* GPJ4 */
	S3C24XX_VA_GPIO + 0x2A0,	/* GPK0 */
	S3C24XX_VA_GPIO + 0x2C0,	/* GPK1 */
	S3C24XX_VA_GPIO + 0x2E0,	/* GPK2 */
	S3C24XX_VA_GPIO + 0x300,	/* GPK3 */
	S3C24XX_VA_GPIO + 0x320,	/* MP0_0 */
	S3C24XX_VA_GPIO + 0x340,	/* MP0_1 */
	S3C24XX_VA_GPIO + 0x360,	/* MP0_2 */
	S3C24XX_VA_GPIO + 0x380,	/* MP0_3 */
	S3C24XX_VA_GPIO + 0x3A0,	/* MP0_4 */

	/* followings do not have con register */
	S3C24XX_VA_GPIO + 0x3C8,	/* MP1_0 */
	S3C24XX_VA_GPIO + 0x3E8,	/* MP1_1 */
	S3C24XX_VA_GPIO + 0x408,	/* MP1_2 */
	S3C24XX_VA_GPIO + 0x428,	/* MP1_3 */
	S3C24XX_VA_GPIO + 0x448,	/* MP1_4 */
	S3C24XX_VA_GPIO + 0x468,	/* MP1_5 */
	S3C24XX_VA_GPIO + 0x488,	/* MP1_6 */
	S3C24XX_VA_GPIO + 0x4A8,	/* MP1_7 */
	S3C24XX_VA_GPIO + 0x4C8,	/* MP1_8 */
	S3C24XX_VA_GPIO + 0x4E8,	/* ETC0 */
	S3C24XX_VA_GPIO + 0x508,	/* ETC1 */
	S3C24XX_VA_GPIO + 0x528,	/* ETC2 */
	S3C24XX_VA_GPIO + 0x548,	/* ETC3 */
	S3C24XX_VA_GPIO + 0x568,	/* ETC4 */
};

void s5p_gpio_cfgpin(unsigned int pin, unsigned int function)
{
	void __iomem *base = gpio_base_offset[S5P_GPIO_BASE(pin)];
	unsigned long offs = S5P_GPIO_OFFSET(pin) * 4;
	unsigned long con, flags;

	local_irq_save(flags);

	con = __raw_readl(base);
	con &= ~(0xf << offs);
	con |= (function << offs);	
	__raw_writel(con, base);

	local_irq_restore(flags);
}

EXPORT_SYMBOL(s5p_gpio_cfgpin);

unsigned int s5p_gpio_getcfg(unsigned int pin)
{
	void __iomem *base = gpio_base_offset[S5P_GPIO_BASE(pin)];
	unsigned long mask;

	mask = 0xf << (S5P_GPIO_OFFSET(pin) * 4);

	return __raw_readl(base) & mask;
}

EXPORT_SYMBOL(s5p_gpio_getcfg);

void s5p_gpio_setpin(unsigned int pin, unsigned int to)
{
	void __iomem *base = gpio_base_offset[S5P_GPIO_BASE(pin)];
	unsigned long offs = S5P_GPIO_OFFSET(pin);
	unsigned long dat, flags;

	local_irq_save(flags);

	dat = __raw_readl(base + 0x4);
	dat &= ~(0x1 << offs);
	dat |= (to << offs);
	__raw_writel(dat, base + 0x4);

	local_irq_restore(flags);
}

EXPORT_SYMBOL(s5p_gpio_setpin);

unsigned int s5p_gpio_getpin(unsigned int pin)
{
	void __iomem *base = gpio_base_offset[S5P_GPIO_BASE(pin)];
	unsigned long offs = S5P_GPIO_OFFSET(pin);

	return __raw_readl(base + 0x4) & (0x1 << offs);
}

EXPORT_SYMBOL(s5p_gpio_getpin);

void s5p_gpio_pullup(unsigned int pin, unsigned int to)
{
	void __iomem *base = gpio_base_offset[S5P_GPIO_BASE(pin)];
	unsigned long offs = S5P_GPIO_OFFSET(pin) * 2;
	unsigned long up, mask, flags;

	mask = 0x3 << offs;

	local_irq_save(flags);

	up = __raw_readl(base + 0x8);
	up &= ~mask;
	up |= (to << offs);
	__raw_writel(up, base + 0x8);

	local_irq_restore(flags);
}

EXPORT_SYMBOL(s5p_gpio_pullup);

void s5p_gpio_drv(unsigned int pin, unsigned int to)
{
	void __iomem *base = gpio_base_offset[S5P_GPIO_BASE(pin)];
	unsigned long offs = S5P_GPIO_OFFSET(pin) * 2;
	unsigned long up, mask, flags;

	mask = 0x3 << offs;

	local_irq_save(flags);

	up = __raw_readl(base + 0xc);
	up &= ~mask;
	up |= (to << offs);
	__raw_writel(up, base + 0xc);

	local_irq_restore(flags);
}

EXPORT_SYMBOL(s5p_gpio_drv);

void s5p_gpio_pdn_con(unsigned int pin, unsigned int to)
{
	void __iomem *base = gpio_base_offset[S5P_GPIO_BASE(pin)];
	unsigned long offs = S5P_GPIO_OFFSET(pin) * 2;
	unsigned long con, mask, flags;

	mask = 0x3 << offs;

	local_irq_save(flags);

	con = __raw_readl(base + 0x10);
	con &= ~mask;
	con |= (to << offs);
	__raw_writel(con, base + 0x10);

	local_irq_restore(flags);
}

EXPORT_SYMBOL(s5p_gpio_pdn_con);

void s5p_gpio_pdn_pullup(unsigned int pin, unsigned int to)
{
	void __iomem *base = gpio_base_offset[S5P_GPIO_BASE(pin)];
	unsigned long offs = S5P_GPIO_OFFSET(pin) * 2;
	unsigned long up, mask, flags;

	mask = 0x3 << offs;

	local_irq_save(flags);

	up = __raw_readl(base + 0x14);
	up &= ~mask;
	up |= (to << offs);
	__raw_writel(up, base + 0x14);

	local_irq_restore(flags);
}

EXPORT_SYMBOL(s5p_gpio_pdn_pullup);

void s5p_gpio_eint_cfg(unsigned int reg, unsigned int num, unsigned int to)
{
	unsigned long con, flags;

	local_irq_save(flags);
	
	con = __raw_readl(reg);
	con &= ~(0x7 << (num * 4));
	con |= (to << (num * 4));
	__raw_writel(con, reg);

	local_irq_restore(flags);
}

EXPORT_SYMBOL(s5p_gpio_eint_cfg);

