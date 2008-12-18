
/* linux/arch/arm/mach-s3c6410/s3c6410.c
 *
 * Copyright (c) 2006, Samsung Electronics
 * All rights reserved.
 *
 * Samsung S3C6410 Mobile CPU support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * derived from linux/arch/arm/mach-s3c2410/devs.c, written by
 * Ben Dooks <ben@simtec.co.uk>
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/sysdev.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/clk.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>

#include <asm/proc-fns.h>
#include <asm/arch/idle.h>

#include <asm/arch/regs-s3c-clock.h>
#include <asm/plat-s3c/regs-serial.h>
#include <asm/arch/regs-gpio.h>
#include <asm/arch/map.h>
#include <asm/plat-s3c/nand.h>

#include <asm/plat-s3c64xx/s3c6410.h>
#include <asm/plat-s3c24xx/devs.h>
#include <asm/plat-s3c24xx/cpu.h>
//#include "pm-s3c6400.h"

/* Serial port registrations */

static struct map_desc s5pc100_iodesc[] __initdata = {
//	IODESC_ENT(SROMC),
//	IODESC_ENT(TIMER),
	IODESC_ENT(LCD),
//	IODESC_ENT(HOSTIFB),
//	IODESC_ENT(OTGSFR),
//	IODESC_ENT(OTG),
};

/* s5pc100_idle
 *
 * use the standard idle call by ensuring the idle mode
 * in power config, then issuing the idle co-processor
 * instruction
*/

static void s5pc100_idle(void)
{
#if 0
	unsigned long tmp;

	/* ensure our idle mode is to go to idle */

	/* Set WFI instruction to SLEEP mode */

	tmp = __raw_readl(S3C_PWR_CFG);
	tmp &= ~(0x60<<0);
	tmp |= (0x1<<5);
	__raw_writel(tmp, S3C_PWR_CFG);

	cpu_do_idle();
#endif
}


void __init s5pc100_init_uarts(struct s3c2410_uartcfg *cfg, int no)
{
	s3c24xx_init_uartdevs("s3c-uart", s3c2410_uart_resources, cfg, no);
}

struct sysdev_class s5pc100_sysclass = {
	set_kset_name("s5pc100-core"),
};

static struct sys_device s5pc100_sysdev = {
	.cls		= &s5pc100_sysclass,
};


static int __init s5pc100_core_init(void)
{
	return sysdev_class_register(&s5pc100_sysclass);
}

core_initcall(s5pc100_core_init);

void __init s5pc100_map_io(struct map_desc *mach_desc, int size)
{
	/* register our io-tables */
	iotable_init(s5pc100_iodesc, ARRAY_SIZE(s5pc100_iodesc));
	iotable_init(mach_desc, size);
	
	/* set our idle function */
	s3c24xx_idle = s5pc100_idle;
}

int __init s5pc100_init(void)
{
	int ret;

	printk("s5pc100: Initialising architecture\n");

	ret = sysdev_register(&s5pc100_sysdev);

	if (ret != 0)
		printk(KERN_ERR "failed to register sysdev for s3c6410\n");

	/* Rename devices that are specific to S3C6410 */
	
#if defined (CONFIG_S3C_SIR)
	s3c24xx_uart_src[3]->name = "s3c-irda";
#endif
#if 0
	s3c_device_lcd.name  = "s3c-lcd";
	s3c_device_nand.name  = "s3c-nand";
#endif

	return ret;
}

