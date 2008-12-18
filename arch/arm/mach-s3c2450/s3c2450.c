/* linux/arch/arm/mach-s3c2450/s3c2450.c
 *
 * Copyright (c) 2007 Samsung Electronics
 *   
 *
 * Samsung S3C2450 Mobile CPU support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/sysdev.h>
#include <linux/clk.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>

#include <asm/arch/regs-s3c-clock.h>
#include <asm/arch/reset.h>

#include <asm/plat-s3c24xx/s3c2450.h>
#include <asm/plat-s3c24xx/devs.h>
#include <asm/plat-s3c24xx/common-smdk.h>
#include <asm/plat-s3c24xx/cpu.h>

static struct map_desc s3c2450_iodesc[] __initdata = {
	IODESC_ENT(WATCHDOG),
	IODESC_ENT(CLKPWR),
	IODESC_ENT(TIMER),
	IODESC_ENT(LCD),
	IODESC_ENT(EBI),
	IODESC_ENT(SROMC),
	IODESC_ENT(CS8900),
	IODESC_ENT(USBDEV),
};

struct sysdev_class s3c2450_sysclass = {
	set_kset_name("s3c2450-core"),
};

static struct sys_device s3c2450_sysdev = {
	.cls		= &s3c2450_sysclass,
};

static void s3c2450_hard_reset(void)
{
	__raw_writel(S3C2443_SWRST_RESET, S3C2443_SWRST);
}

int __init s3c2450_init(void)
{
	printk("S3C2450: Initialising architecture\n");

	s3c24xx_reset_hook = s3c2450_hard_reset;

#if !defined (CONFIG_MTD_NAND_S3C)
	s3c_device_nand.name = "s3c2412-nand";
#else
	s3c_device_nand.name = "s3c-nand";
#endif
	/* Rename devices that are specific to s3c2450 */
        s3c_device_lcd.name  = "s3c-lcd";

	/* change WDT IRQ number */
	s3c_device_wdt.resource[1].start = IRQ_S3C2443_WDT;
	s3c_device_wdt.resource[1].end   = IRQ_S3C2443_WDT;

	return sysdev_register(&s3c2450_sysdev);
}

void __init s3c2450_init_uarts(struct s3c2410_uartcfg *cfg, int no)
{
	s3c24xx_init_uartdevs("s3c2440-uart", s3c2410_uart_resources, cfg, no);
}

/* s3c2450_map_io
 *
 * register the standard cpu IO areas, and any passed in from the
 * machine specific initialisation.
 */

void __init s3c2450_map_io(struct map_desc *mach_desc, int mach_size)
{
	iotable_init(s3c2450_iodesc, ARRAY_SIZE(s3c2450_iodesc));
	iotable_init(mach_desc, mach_size);
}

/* need to register class before we actually register the device, and
 * we also need to ensure that it has been initialised before any of the
 * drivers even try to use it (even if not on an s3c2450 based system)
 * as a driver which may support both 2450 and 2440 may try and use it.
*/

static int __init s3c2450_core_init(void)
{
	return sysdev_class_register(&s3c2450_sysclass);
}

core_initcall(s3c2450_core_init);
