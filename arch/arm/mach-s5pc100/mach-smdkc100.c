/***********************************************************************
 *
 * linux/arch/arm/mach-s3c6410/smdk6410.c
 *
 * $Id: mach-smdkc100.c,v 1.5 2008/08/27 01:04:36 jsgood Exp $
 *
 * Copyright (C) 2005, Samsung Electronics <@samsung.com>
 * All rights reserved
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 *
 ***********************************************************************/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include <asm/setup.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>
#include <asm/mach/flash.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach-types.h>

#include <asm/plat-s3c/regs-serial.h>
#include <asm/arch/regs-gpio.h>

#include <asm/arch/regs-mem.h>
#include <asm/arch/regs-s3c-clock.h>
#include <asm/arch/regs-usb-otg-hs.h>
#include <asm/plat-s3c/nand.h>

#include <asm/plat-s5p/s5pc100.h>
#include <asm/plat-s3c24xx/devs.h>
#include <asm/plat-s3c24xx/cpu.h>
#include <asm/arch/hsmmc.h>

#include <asm/plat-s3c24xx/common-smdk.h>
#include <asm/arch-s3c2410/reserved_mem.h>

extern struct sys_timer s3c_timer;

static struct map_desc smdkc100_iodesc[] __initdata = {
//	IODESC_ENT(CS8900),
};

#define DEF_UCON S3C_UCON_DEFAULT
#define DEF_ULCON S3C_LCON_CS8 | S3C_LCON_PNONE
#define DEF_UFCON S3C_UFCON_RXTRIG8 | S3C_UFCON_FIFOMODE

static struct s3c24xx_uart_clksrc smdkc100_serial_clocks[] = {
	[0] = {
		.name		= "pclkd1",
		.divisor	= 1,
		.min_baud	= 0,
		.max_baud	= 0,
	},
	[1] = {
		.name		= "pclkd0",
		.divisor	= 1,
		.min_baud	= 0,
		.max_baud	= 0,
	}

};

static struct s3c2410_uartcfg smdkc100_uartcfgs[] = {
	[0] = {
		.hwport	     = 0,
		.flags	     = 0,
		.ucon	     = DEF_UCON,
		.ulcon	     = DEF_ULCON,
		.ufcon	     = DEF_UFCON,
		.clocks	     = smdkc100_serial_clocks,
		.clocks_size = ARRAY_SIZE(smdkc100_serial_clocks),
	},
	[1] = {
		.hwport	     = 1,
		.flags	     = 0,
		.ucon	     = DEF_UCON,
		.ulcon	     = DEF_ULCON,
		.ufcon	     = DEF_UFCON,
		.clocks	     = smdkc100_serial_clocks,
		.clocks_size = ARRAY_SIZE(smdkc100_serial_clocks),
	},
	[2] = {
		.hwport	     = 2,
		.flags	     = 0,
		.ucon	     = DEF_UCON,
		.ulcon	     = DEF_ULCON,
		.ufcon	     = DEF_UFCON,
		.clocks	     = smdkc100_serial_clocks,
		.clocks_size = ARRAY_SIZE(smdkc100_serial_clocks),
	},
	[3] = {
		.hwport	     = 3,
		.flags	     = 0,
		.ucon	     = DEF_UCON,
		.ulcon	     = DEF_ULCON,
		.ufcon	     = DEF_UFCON,
		.clocks	     = smdkc100_serial_clocks,
		.clocks_size = ARRAY_SIZE(smdkc100_serial_clocks),
	}

};

/*add devices as drivers are integrated*/
static struct platform_device *smdkc100_devices[] __initdata = {
#if 0
	&s3c_device_rtc,
	&s3c_device_ac97,
	&s3c_device_iis,
	&s3c_device_adc,
	&s3c_device_i2c,
	&s3c_device_usb,
	&s3c_device_usbgadget,
	&s3c_device_tvenc,
	&s3c_device_tvscaler,
	&s3c_device_hsmmc0,
	&s3c_device_hsmmc1,
	&s3c_device_hsmmc2,
	&s3c_device_wdt,
	&s3c_device_jpeg,
	&s3c_device_vpp,
	&s3c_device_ide,
	&s3c_device_mfc,
	&s3c_device_spi0,
	&s3c_device_spi1,
	&s3c_device_2d,
	&s3c_device_keypad,
	&s3c_device_smc911x,
	&s3c_device_camif,
	&s3c_device_g3d,
#endif
};


static void __init smdkc100_map_io(void)
{
	s3c24xx_init_io(smdkc100_iodesc, ARRAY_SIZE(smdkc100_iodesc));
	s3c24xx_init_clocks(0);
	s3c24xx_init_uarts(smdkc100_uartcfgs, ARRAY_SIZE(smdkc100_uartcfgs));
}

static void __init smdkc100_fixup (struct machine_desc *desc, struct tag *tags,
	      char **cmdline, struct meminfo *mi)
{
	/*
	 * Bank start addresses are not present in the information
	 * passed in from the boot loader.  We could potentially
	 * detect them, but instead we hard-code them.
	 */
	mi->bank[0].start = PHYS_OFFSET;

 	mi->bank[0].size = 128*1024*1024;

	mi->bank[0].node = 0;

	mi->nr_banks = 1;
}

static void __init smdkc100_machine_init (void)
{
	platform_add_devices(smdkc100_devices, ARRAY_SIZE(smdkc100_devices));
	smdk_machine_init();
}

MACHINE_START(SMDKC100, "SMDKC100")
	/* Maintainer: Samsung Electronics */
	.phys_io	= S3C24XX_PA_UART,
	.io_pg_offst	= (((u32)S3C24XX_VA_UART) >> 18) & 0xfffc,
	.boot_params	= S5PC1XX_SDRAM_PA + 0x100,

	.init_irq	= s3c_init_irq,
	.map_io		= smdkc100_map_io,
	.fixup		= smdkc100_fixup,
	.timer		= &s3c_timer,
	.init_machine	= smdkc100_machine_init,
MACHINE_END

