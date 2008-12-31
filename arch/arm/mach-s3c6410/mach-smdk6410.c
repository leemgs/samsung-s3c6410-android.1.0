/***********************************************************************
 *
 * linux/arch/arm/mach-s3c6410/smdk6410.c
 *
 * $Id: mach-smdk6410.c,v 1.56 2008/09/10 01:40:15 jsgood Exp $
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
#include <linux/i2c.h>

#include <asm/setup.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>
#include <asm/mach/flash.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach-types.h>

#include <asm/plat-s3c/iic.h>
#include <asm/plat-s3c/regs-iic.h>

#include <asm/arch/gpio.h>
#include <asm/plat-s3c/regs-serial.h>
#include <asm/arch/regs-gpio.h>

#include <asm/arch/regs-mem.h>
#include <asm/arch/regs-s3c-clock.h>
#include <asm/plat-s3c/nand.h>
#include <asm/plat-s3c/adc.h>

#include <asm/plat-s3c64xx/s3c6410.h>
#include <asm/plat-s3c24xx/devs.h>
#include <asm/plat-s3c24xx/cpu.h>
#include <asm/arch/hsmmc.h>

#include <asm/plat-s3c24xx/common-smdk.h>
#include <asm/arch-s3c2410/reserved_mem.h>

#if defined(CONFIG_RTC_DRV_S3C)
#include <asm/plat-s3c/regs-rtc.h>
#endif

#if defined(CONFIG_USB_GADGET_S3C_OTGD_HS) || defined(CONFIG_USB_OHCI_HCD)
#include <asm/arch-s3c2410/regs-usb-otg-hs.h>
#endif

#include <asm/plat-s3c/regs-adc.h>
#include <asm/plat-s3c/ts.h>

extern struct sys_timer s3c_timer;

static struct map_desc smdk6410_iodesc[] __initdata = {
	IODESC_ENT(CS8900),
};

#define DEF_UCON S3C_UCON_DEFAULT
#define DEF_ULCON S3C_LCON_CS8 | S3C_LCON_PNONE
#define DEF_UFCON S3C_UFCON_RXTRIG8 | S3C_UFCON_FIFOMODE


static struct s3c24xx_uart_clksrc smdk6410_serial_clocks[] = {
	[0] = {
		.name		= "mpll_clk_uart",
		.divisor	= 1,
		.min_baud	= 0,
		.max_baud	= 0,
	},
#if defined (CONFIG_SERIAL_S3C64XX_HS_UART)
	[2] = {
		.name		= "epll_clk_uart_192m",
		.divisor	= 1,
		.min_baud	= 0,
		.max_baud	= 4000000,
	}
#endif
};



static struct s3c2410_uartcfg smdk6410_uartcfgs[] = {
	[0] = {
		.hwport	     = 0,
		.flags	     = 0,
		.ucon	     = DEF_UCON,
		.ulcon	     = DEF_ULCON,
		.ufcon	     = DEF_UFCON,
		.clocks	     = smdk6410_serial_clocks,
		.clocks_size = ARRAY_SIZE(smdk6410_serial_clocks),
	},
	[1] = {
		.hwport	     = 1,
		.flags	     = 0,
		.ucon	     = DEF_UCON,
		.ulcon	     = DEF_ULCON,
		.ufcon	     = DEF_UFCON,
		.clocks	     = smdk6410_serial_clocks,
		.clocks_size = ARRAY_SIZE(smdk6410_serial_clocks),
	},
	[2] = {
		.hwport	     = 2,
		.flags	     = 0,
		.ucon	     = DEF_UCON,
		.ulcon	     = DEF_ULCON,
		.ufcon	     = DEF_UFCON,
		.clocks	     = smdk6410_serial_clocks,
		.clocks_size = ARRAY_SIZE(smdk6410_serial_clocks),
	},
	[3] = {
		.hwport	     = 3,
		.flags	     = 0,
		.ucon	     = DEF_UCON,
		.ulcon	     = DEF_ULCON,
		.ufcon	     = DEF_UFCON,
		.clocks	     = smdk6410_serial_clocks,
		.clocks_size = ARRAY_SIZE(smdk6410_serial_clocks),
	}

};

static struct resource s3c64xx_i2c0_resource[] = {
	[0] = {
		.start = S3C6400_PA_IIC0,
		.end   = S3C6400_PA_IIC0 + S3C24XX_SZ_IIC - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_IIC0,
		.end   = IRQ_IIC0,
		.flags = IORESOURCE_IRQ,
	}
};

static struct platform_device s3c64xx_device_i2c0 = {
	.name		= "s3c64xx-i2c",
	.id		= 0,		/* This is so the driver gets forced to use bus 0. */
	.num_resources	= ARRAY_SIZE(s3c64xx_i2c0_resource),
	.resource	= s3c64xx_i2c0_resource,
};

static struct resource s3c64xx_i2c1_resource[] = {
	[0] = {
		.start = S3C6400_PA_IIC1,
		.end   = S3C6400_PA_IIC1 + S3C24XX_SZ_IIC - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_IIC1,
		.end   = IRQ_IIC1,
		.flags = IORESOURCE_IRQ,
	}
};

static struct platform_device s3c64xx_device_i2c1 = {
	.name		= "s3c64xx-i2c",
	.id		= 1,		/* This is so the driver gets forced to use bus 0. */
	.num_resources	= ARRAY_SIZE(s3c64xx_i2c1_resource),
	.resource	= s3c64xx_i2c1_resource,
};

static struct s3c2410_platform_i2c default_i2c_data0 __initdata = {
	.flags		= 0,
	.slave_addr	= 0x10,
	.bus_freq	= 100*1000,
	.max_freq	= 400*1000,
	.sda_delay	= S3C64XX_IICLC_SDA_DELAY5 | S3C64XX_IICLC_FILTER_ON,
};

void s3c_i2c0_cfg_gpio(struct platform_device *dev)
{
	s3c_gpio_cfgpin(S3C_GPB5, S3C_GPB5_I2C_SCL0);
	s3c_gpio_cfgpin(S3C_GPB6, S3C_GPB6_I2C_SDA0);
	s3c_gpio_pullup(S3C_GPB5, 2);
	s3c_gpio_pullup(S3C_GPB6, 2);
}

void __init s3c_i2c0_set_platdata(struct s3c2410_platform_i2c *pd)
{
	struct s3c2410_platform_i2c *npd;

	if (!pd)
		pd = &default_i2c_data0;

	npd = kmemdup(pd, sizeof(struct s3c2410_platform_i2c), GFP_KERNEL);
	if (!npd)
		printk(KERN_ERR "%s: no memory for platform data\n", __func__);
	else if (!npd->cfg_gpio)
		npd->cfg_gpio = s3c_i2c0_cfg_gpio;

	s3c64xx_device_i2c0.dev.platform_data = npd;
}

static struct s3c2410_platform_i2c default_i2c_data1 __initdata = {
	.flags		= 0,
	.bus_num	= 1,
	.slave_addr	= 0x10,
	.bus_freq	= 100*1000,
	.max_freq	= 400*1000,
	.sda_delay	= S3C64XX_IICLC_SDA_DELAY5 | S3C64XX_IICLC_FILTER_ON,
};

void s3c_i2c1_cfg_gpio(struct platform_device *dev)
{
	s3c_gpio_cfgpin(S3C_GPB2, S3C_GPB2_I2C_SCL1);
	s3c_gpio_cfgpin(S3C_GPB3, S3C_GPB3_I2C_SDA1);
	s3c_gpio_pullup(S3C_GPB2, 2);
	s3c_gpio_pullup(S3C_GPB3, 2);
}

void __init s3c_i2c1_set_platdata(struct s3c2410_platform_i2c *pd)
{
	struct s3c2410_platform_i2c *npd;

	if (!pd)
		pd = &default_i2c_data1;

	npd = kmemdup(pd, sizeof(struct s3c2410_platform_i2c), GFP_KERNEL);
	if (!npd)
		printk(KERN_ERR "%s: no memory for platform data\n", __func__);
	else if (!npd->cfg_gpio)
		npd->cfg_gpio = s3c_i2c1_cfg_gpio;

	s3c64xx_device_i2c1.dev.platform_data = npd;
}

static struct i2c_board_info i2c_devs0[] __initdata = {
	{ I2C_BOARD_INFO("24c08", 0x50), },
};

static struct i2c_board_info i2c_devs1[] __initdata = {
	{ I2C_BOARD_INFO("24c128", 0x57), },
};

/* Add devices as drivers are integrated */
static struct platform_device *smdk6410_devices[] __initdata = {
	&s3c_device_rtc,
	&s3c_device_ac97,
	&s3c_device_iis,
	&s3c_device_ts,
	&s3c_device_adc,
	&s3c64xx_device_i2c0,
	&s3c64xx_device_i2c1,
	&s3c_device_usb,
	&s3c_device_usbgadget,
	&s3c_device_usb_otghcd,
	&s3c_device_tvenc,
	&s3c_device_tvscaler,
	&s3c_device_hsmmc0,
	&s3c_device_hsmmc1,
	&s3c_device_hsmmc2,
	&s3c_device_wdt,
	&s3c_device_jpeg,
	&s3c_device_vpp,
	&s3c_device_ide,
	&s3c_device_spi0,
	&s3c_device_spi1,
	&s3c_device_g2d,
	&s3c_device_keypad,
	&s3c_device_smc911x,
	&s3c_device_camif,
	&s3c_device_mfc,
	&s3c_device_g3d,
	&s3c_device_rotator,
};


static void __init smdk6410_map_io(void)
{
	s3c24xx_init_io(smdk6410_iodesc, ARRAY_SIZE(smdk6410_iodesc));
	s3c24xx_init_clocks(0);
	s3c24xx_init_uarts(smdk6410_uartcfgs, ARRAY_SIZE(smdk6410_uartcfgs));
}

void smdk6410_smc911x_set(void)
{
	unsigned int tmp;

	tmp = __raw_readl(S3C_SROM_BW);
	tmp &=~(0xF<<4);
	tmp |= (1<<7) | (1<<6) | (1<<4);
	__raw_writel(tmp, S3C_SROM_BW);

	__raw_writel((0x0<<28)|(0x4<<24)|(0xd<<16)|(0x1<<12)|(0x4<<8)|(0x6<<4)|(0x0<<0), S3C_SROM_BC1);
}

static void __init smdk6410_fixup (struct machine_desc *desc, struct tag *tags,
	      char **cmdline, struct meminfo *mi)
{
	/*
	 * Bank start addresses are not present in the information
	 * passed in from the boot loader.  We could potentially
	 * detect them, but instead we hard-code them.
	 */
	mi->bank[0].start = PHYS_OFFSET;

#if defined(CONFIG_RESERVED_MEM_JPEG)
	mi->bank[0].size = 120*1024*1024;
#elif defined(CONFIG_RESERVED_MEM_JPEG_POST)
	mi->bank[0].size = 112*1024*1024;
#elif defined(CONFIG_RESERVED_MEM_MFC)
	mi->bank[0].size = 122*1024*1024;
#elif defined(CONFIG_RESERVED_MEM_MFC_POST)
	mi->bank[0].size = 114*1024*1024;
#elif defined(CONFIG_RESERVED_MEM_JPEG_MFC_POST)
	mi->bank[0].size = 106*1024*1024;
#elif defined(CONFIG_RESERVED_MEM_JPEG_CAMERA)
	mi->bank[0].size = 105*1024*1024;
#elif defined(CONFIG_RESERVED_MEM_JPEG_POST_CAMERA)
	mi->bank[0].size = 97*1024*1024;
#elif defined(CONFIG_RESERVED_MEM_MFC_CAMERA)
	mi->bank[0].size = 107*1024*1024;
#elif defined(CONFIG_RESERVED_MEM_MFC_POST_CAMERA)
	mi->bank[0].size = 99*1024*1024;
#elif defined(CONFIG_RESERVED_MEM_JPEG_MFC_POST_CAMERA)
	mi->bank[0].size = 91*1024*1024;
#elif defined(CONFIG_RESERVED_MEM_CMM_MFC_POST)
	mi->bank[0].size = 106*1024*1024;
#elif defined(CONFIG_RESERVED_MEM_CMM_JPEG_MFC_POST_CAMERA)
	mi->bank[0].size = 83*1024*1024;
#elif defined(CONFIG_RESERVED_MEM_TV_MFC_POST_CAMERA)
	mi->bank[0].size = 91*1024*1024;
#elif defined(CONFIG_VIDEO_SAMSUNG)
	mi->bank[0].size = 113*1024*1024;
#else
 	mi->bank[0].size = 128*1024*1024;
#endif
	mi->bank[0].node = 0;

	mi->nr_banks = 1;
}


static void smdk6410_hsmmc_init (void)
{
	/* hsmmc data strength */
	writel(readl(S3C_SPCON) | (0x3 << 26), S3C_SPCON);

	/* jsgood: hsmmc0/1 card detect pin should be high before setup gpio. (GPG6 to Input) */
	writel(readl(S3C_GPGCON) & 0xf0ffffff, S3C_GPGCON);

	/* GPIO N 13 (external interrupt) : Chip detect */
	gpio_set_pin(S3C_GPN13, S3C_GPN13_EXTINT13);	/* GPN13 to EINT13 */
	gpio_pullup(S3C_GPN13, 0x2);	  	/* Pull-up Enable */

	/* jsgood: MUXmmc# to DOUTmpll for MPLL Clock Source */
	writel((readl(S3C_CLK_SRC) & ~(0x3f << 18)) | (0x15 << 18), S3C_CLK_SRC);
}

#define IS_SMDK6430_CHK15	(1<<15)

static unsigned int is_smdk6430(void)
{
	u32 reg, ret;

	reg = (u32)ioremap((unsigned long)S3C64XX_PA_CHIP_ID, 0x1);

	if((__raw_readl(reg + 0x4)& IS_SMDK6430_CHK15) == IS_SMDK6430_CHK15)
		ret = 1; /* SMDK6430 */
	else
		ret = 0; /* SMDK6410 */

	iounmap((void*)reg);

	return ret;
}

static void smdk6410_set_qos(void)
{
	u32 reg;

	/* AXI sfr */
	reg = (u32) ioremap((unsigned long) S3C6410_PA_AXI_SYS, SZ_4K);

	/* QoS override: FIMD min. latency */
	writel(0x2, S3C24XX_VA_SYSCON + 0x128);

	/* AXI QoS */
	writel(0x7, reg + 0x460);	/* (8 - MFC ch.) */
	writel(0x7ff7, reg + 0x464);

	/* Bus cacheable */
	writel(0x8ff, S3C24XX_VA_SYSCON + 0x838);
}

static void __init smdk6410_machine_init (void)
{
	smdk6410_smc911x_set();

	s3c_i2c0_set_platdata(NULL);
	s3c_i2c1_set_platdata(NULL);

	i2c_register_board_info(0, i2c_devs0, ARRAY_SIZE(i2c_devs0));
	i2c_register_board_info(1, i2c_devs1, ARRAY_SIZE(i2c_devs1));

	platform_add_devices(smdk6410_devices, ARRAY_SIZE(smdk6410_devices));

	if(is_smdk6430()) {
		platform_device_del(&s3c_device_mfc);
		platform_device_del(&s3c_device_g3d);
	}

	smdk_machine_init();

	smdk6410_hsmmc_init();
	smdk6410_set_qos();
}

#if !defined(CONFIG_MACH_SMDK6430)
MACHINE_START(SMDK6410, "SMDK6410")
#else
MACHINE_START(SMDK6430, "SMDK6430")
#endif
	/* Maintainer: Samsung Electronics */
	.phys_io    = S3C24XX_PA_UART,
	.io_pg_offst    = (((u32)S3C24XX_VA_UART) >> 18) & 0xfffc,
	.boot_params    = S3C_SDRAM_PA + 0x100,

	.init_irq   = s3c_init_irq,
	.map_io     = smdk6410_map_io,
	.fixup      = smdk6410_fixup,
	.timer      = &s3c_timer,
	.init_machine   = smdk6410_machine_init,
MACHINE_END

/*--------------------------------------------------------------
 * HS-MMC GPIO Set function
 * the location of this function must be re-considered.
 * by scsuh
 *--------------------------------------------------------------*/
void hsmmc_set_gpio (uint channel, uint width)
{
	switch (channel) {
	/* can supports 1 and 4 bit bus */
	case 0:
		/* GPIO G : Command, Clock */
		gpio_set_pin(S3C_GPG0, S3C_GPG0_MMC_CLK0);
		gpio_set_pin(S3C_GPG1, S3C_GPG1_MMC_CMD0);

		gpio_pullup(S3C_GPG0, 0x0);	  /* Pull-up/down disable */
		gpio_pullup(S3C_GPG1, 0x0);	  /* Pull-up/down disable */

		/* GPIO G : Chip detect + LED */
		gpio_set_pin(S3C_GPG6, S3C_GPG6_MMC_CD1);
		gpio_pullup(S3C_GPG6, 0x2);	  /* Pull-up Enable */

		if (width == 1) {
			/* GPIO G : MMC DATA1[0] */
			gpio_set_pin(S3C_GPG2, S3C_GPG2_MMC_DATA0_0);

			gpio_pullup(S3C_GPG2, 0x0);	  /* Pull-up/down disable */
		}
		else if (width == 4) {
			/* GPIO G : MMC DATA1[0:3] */
			gpio_set_pin(S3C_GPG2, S3C_GPG2_MMC_DATA0_0);
			gpio_set_pin(S3C_GPG3, S3C_GPG3_MMC_DATA0_1);
			gpio_set_pin(S3C_GPG4, S3C_GPG4_MMC_DATA0_2);
			gpio_set_pin(S3C_GPG5, S3C_GPG5_MMC_DATA0_3);

			gpio_pullup(S3C_GPG2, 0x0);	  /* Pull-up/down disable */
			gpio_pullup(S3C_GPG3, 0x0);	  /* Pull-up/down disable */
			gpio_pullup(S3C_GPG4, 0x0);	  /* Pull-up/down disable */
			gpio_pullup(S3C_GPG5, 0x0);	  /* Pull-up/down disable */
		}
		break;

	/* can supports 1, 4, and 8 bit bus */
	case 1:
		/* GPIO H : Command, Clock */
		gpio_set_pin(S3C_GPH0, S3C_GPH0_MMC_CLK1);
		gpio_set_pin(S3C_GPH1, S3C_GPH1_MMC_CMD1);

		gpio_pullup(S3C_GPH0, 0x0);	  /* Pull-up/down disable */
		gpio_pullup(S3C_GPH1, 0x0);	  /* Pull-up/down disable */

		/* GPIO G : Chip detect + LED */
		gpio_set_pin(S3C_GPG6, S3C_GPG6_MMC_CD1);
		gpio_pullup(S3C_GPG6, 0x2);	  /* Pull-up Enable */

		if (width == 1) {
			/* GPIO H : MMC DATA1[0] */
			gpio_set_pin(S3C_GPH2, S3C_GPH2_MMC_DATA1_0);

			gpio_pullup(S3C_GPH2, 0x0);	  /* Pull-up/down disable */
		}
		else if (width == 4) {
			/* GPIO H : MMC DATA1[0:3] */
			gpio_set_pin(S3C_GPH2, S3C_GPH2_MMC_DATA1_0);
			gpio_set_pin(S3C_GPH3, S3C_GPH3_MMC_DATA1_1);
			gpio_set_pin(S3C_GPH4, S3C_GPH4_MMC_DATA1_2);
			gpio_set_pin(S3C_GPH5, S3C_GPH5_MMC_DATA1_3);

			gpio_pullup(S3C_GPH2, 0x0);	  /* Pull-up/down disable */
			gpio_pullup(S3C_GPH3, 0x0);	  /* Pull-up/down disable */
			gpio_pullup(S3C_GPH4, 0x0);	  /* Pull-up/down disable */
			gpio_pullup(S3C_GPH5, 0x0);	  /* Pull-up/down disable */
		}
		else if (width == 8) {
			/* GPIO H : MMC DATA1[0:7] */
			gpio_set_pin(S3C_GPH2, S3C_GPH2_MMC_DATA1_0);
			gpio_set_pin(S3C_GPH3, S3C_GPH3_MMC_DATA1_1);
			gpio_set_pin(S3C_GPH4, S3C_GPH4_MMC_DATA1_2);
			gpio_set_pin(S3C_GPH5, S3C_GPH5_MMC_DATA1_3);
			gpio_set_pin(S3C_GPH6, S3C_GPH6_MMC_DATA1_4);
			gpio_set_pin(S3C_GPH7, S3C_GPH7_MMC_DATA1_5);
			gpio_set_pin(S3C_GPH8, S3C_GPH8_MMC_DATA1_6);
			gpio_set_pin(S3C_GPH9, S3C_GPH9_MMC_DATA1_7);

			gpio_pullup(S3C_GPH2, 0x0);	  /* Pull-up/down disable */
			gpio_pullup(S3C_GPH3, 0x0);	  /* Pull-up/down disable */
			gpio_pullup(S3C_GPH4, 0x0);	  /* Pull-up/down disable */
			gpio_pullup(S3C_GPH5, 0x0);	  /* Pull-up/down disable */
			gpio_pullup(S3C_GPH6, 0x0);	  /* Pull-up/down disable */
			gpio_pullup(S3C_GPH7, 0x0);	  /* Pull-up/down disable */
			gpio_pullup(S3C_GPH8, 0x0);	  /* Pull-up/down disable */
			gpio_pullup(S3C_GPH9, 0x0);	  /* Pull-up/down disable */
		}
		break;

	/* can supports 1 and 4 bit bus, no irq_cd */
	case 2:
		/* GPIO H : Command, Clock */
		gpio_set_pin(S3C_GPH0, S3C_GPH0_MMC_CLK1);
		gpio_set_pin(S3C_GPH1, S3C_GPH1_MMC_CMD1);

		gpio_pullup(S3C_GPH0, 0x0);	  /* Pull-up/down disable */
		gpio_pullup(S3C_GPH1, 0x0);	  /* Pull-up/down disable */

		/* GPIO G : Chip detect + LED */
		gpio_set_pin(S3C_GPG6, S3C_GPG6_MMC_CD1);
		gpio_pullup(S3C_GPG6, 0x2);	  /* Pull-up Enable */

		if (width == 1) {
			/* GPIO H : MMC DATA1[0] */
			gpio_set_pin(S3C_GPH6, S3C_GPH6_MMC_DATA2_0);

			gpio_pullup(S3C_GPH6, 0x0);	  /* Pull-up/down disable */
		}
		else if (width == 4) {
			/* GPIO H : MMC DATA1[0:3] */
			gpio_set_pin(S3C_GPH6, S3C_GPH6_MMC_DATA2_0);
			gpio_set_pin(S3C_GPH7, S3C_GPH7_MMC_DATA2_1);
			gpio_set_pin(S3C_GPH8, S3C_GPH8_MMC_DATA2_2);
			gpio_set_pin(S3C_GPH9, S3C_GPH9_MMC_DATA2_3);

			gpio_pullup(S3C_GPH6, 0x0);	  /* Pull-up/down disable */
			gpio_pullup(S3C_GPH7, 0x0);	  /* Pull-up/down disable */
			gpio_pullup(S3C_GPH8, 0x0);	  /* Pull-up/down disable */
			gpio_pullup(S3C_GPH9, 0x0);	  /* Pull-up/down disable */
		}

		break;

	default:
		break;
	}
}

/* For host controller's capabilities */
#define HOST_CAPS (MMC_CAP_4_BIT_DATA | MMC_CAP_MULTIWRITE | \
			MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED)


struct s3c_hsmmc_cfg s3c_hsmmc0_platform = {
	.hwport = 0,
	.enabled = 0,
	.host_caps = HOST_CAPS,
	.bus_width = 4,
	.highspeed = 0,

	/* ctrl for mmc */
	.fd_ctrl[MMC_TYPE_MMC] = {
		.ctrl2 = 0xC0004100,			/* ctrl2 for mmc */
		.ctrl3[SPEED_NORMAL] = 0x80808080,	/* ctrl3 for low speed */
		.ctrl3[SPEED_HIGH]   = 0x00008080,	/* ctrl3 for high speed */
		.ctrl4 = 0x3,
	},

	/* ctrl for sd */
	.fd_ctrl[MMC_TYPE_SD] = {
		.ctrl2 = 0xC0004100,			/* ctrl2 for sd */
		.ctrl3[SPEED_NORMAL] = 0x80808080,	/* ctrl3 for low speed */
		.ctrl3[SPEED_HIGH]   = 0x00008080,	/* ctrl3 for high speed */
		.ctrl4 = 0x3,
	},

	.clocks[0] = {
		.name = "sclk_DOUTmpll_mmc0",
		.src = 0x2,
	},
};

struct s3c_hsmmc_cfg s3c_hsmmc1_platform = {
	.hwport = 1,
	.enabled = 1,
	.host_caps = HOST_CAPS,
	.bus_width = 4,
	//.host_caps = HOST_CAPS | MMC_CAP_8_BIT_DATA,
	//.bus_width = 8,

	.highspeed = 0,

	/* ctrl for mmc */
	.fd_ctrl[MMC_TYPE_MMC] = {
		.ctrl2 = 0xC0004100,			/* ctrl2 for mmc */
		.ctrl3[SPEED_NORMAL] = 0x80808080,	/* ctrl3 for low speed */
		.ctrl3[SPEED_HIGH]   = 0x00008080,	/* ctrl3 for high speed */
		.ctrl4 = 0x3,
	},

	/* ctrl for sd */
	.fd_ctrl[MMC_TYPE_SD] = {
		.ctrl2 = 0xC0004100,			/* ctrl2 for sd */
		.ctrl3[SPEED_NORMAL] = 0x80808080,	/* ctrl3 for low speed */
		.ctrl3[SPEED_HIGH]   = 0x00008080,	/* ctrl3 for high speed */
		.ctrl4 = 0x3,
	},

	.clocks[0] = {
		.name = "sclk_DOUTmpll_mmc1",
		.src = 0x2,
	},
};

struct s3c_hsmmc_cfg s3c_hsmmc2_platform = {
	.hwport = 2,
	.enabled = 0,
	.host_caps = HOST_CAPS,
	.bus_width = 4,
	.highspeed = 0,

	/* ctrl for mmc */
	.fd_ctrl[MMC_TYPE_MMC] = {
		.ctrl2 = 0xC0004100,			/* ctrl2 for mmc */
		.ctrl3[SPEED_NORMAL] = 0x80808080,	/* ctrl3 for low speed */
		.ctrl3[SPEED_HIGH]   = 0x00008080,	/* ctrl3 for high speed */
		.ctrl4 = 0x3,
	},

	/* ctrl for sd */
	.fd_ctrl[MMC_TYPE_SD] = {
		.ctrl2 = 0xC0004100,			/* ctrl2 for sd */
		.ctrl3[SPEED_NORMAL] = 0x80808080,	/* ctrl3 for low speed */
		.ctrl3[SPEED_HIGH]   = 0x00008080,	/* ctrl3 for high speed */
		.ctrl4 = 0x3,
	},

	.clocks[0] = {
		.name = "sclk_DOUTmpll_mmc2",
		.src = 0x2,
	},
};

#if defined(CONFIG_RTC_DRV_S3C)
/* RTC common Function for samsung APs*/
unsigned int s3c_rtc_set_bit_byte(void __iomem *base, uint offset, uint val)
{
	writeb(val, base + offset);

	return 0;
}

unsigned int s3c_rtc_read_alarm_status(void __iomem *base)
{
	return 1;
}

void s3c_rtc_set_pie(void __iomem *base, uint to)
{
	unsigned int tmp;

	tmp = readw(base + S3C2410_RTCCON) & ~S3C_RTCCON_TICEN;

        if (to)
                tmp |= S3C_RTCCON_TICEN;

        writew(tmp, base + S3C2410_RTCCON);
}

void s3c_rtc_set_freq_regs(void __iomem *base, uint freq, uint *s3c_freq)
{
	unsigned int tmp;

        tmp = readw(base + S3C2410_RTCCON) & (S3C_RTCCON_TICEN | S3C2410_RTCCON_RTCEN );
        writew(tmp, base + S3C2410_RTCCON);
        *s3c_freq = freq;
        tmp = (32768 / freq)-1;
        writel(tmp, base + S3C2410_TICNT);
}

void s3c_rtc_enable_set(struct platform_device *pdev,void __iomem *base, int en)
{
	unsigned int tmp;

	if (!en) {
		tmp = readw(base + S3C2410_RTCCON);
		writew(tmp & ~ (S3C2410_RTCCON_RTCEN | S3C_RTCCON_TICEN), base + S3C2410_RTCCON);
	} else {
		/* re-enable the device, and check it is ok */
		if ((readw(base+S3C2410_RTCCON) & S3C2410_RTCCON_RTCEN) == 0){
			dev_info(&pdev->dev, "rtc disabled, re-enabling\n");

			tmp = readw(base + S3C2410_RTCCON);
			writew(tmp|S3C2410_RTCCON_RTCEN, base+S3C2410_RTCCON);
		}

		if ((readw(base + S3C2410_RTCCON) & S3C2410_RTCCON_CNTSEL)){
			dev_info(&pdev->dev, "removing RTCCON_CNTSEL\n");

			tmp = readw(base + S3C2410_RTCCON);
			writew(tmp& ~S3C2410_RTCCON_CNTSEL, base+S3C2410_RTCCON);
		}

		if ((readw(base + S3C2410_RTCCON) & S3C2410_RTCCON_CLKRST)){
			dev_info(&pdev->dev, "removing RTCCON_CLKRST\n");

			tmp = readw(base + S3C2410_RTCCON);
			writew(tmp & ~S3C2410_RTCCON_CLKRST, base+S3C2410_RTCCON);
		}
	}
}
#endif

#if defined(CONFIG_USB_GADGET_S3C_OTGD_HS) || defined(CONFIG_USB_OHCI_HCD)
/* Initializes OTG Phy. */
void otg_phy_init(u32 otg_phy_clk) {

	writel(readl(S3C_OTHERS)|S3C_OTHERS_USB_SIG_MASK, S3C_OTHERS);
	writel(0x0, S3C_USBOTG_PHYPWR);		/* Power up */
	writel(otg_phy_clk, S3C_USBOTG_PHYCLK);
	writel(0x1, S3C_USBOTG_RSTCON);

	udelay(50);
	writel(0x0, S3C_USBOTG_RSTCON);
	udelay(50);
}
#endif

#ifdef CONFIG_USB_GADGET_S3C_OTGD_HS
/* OTG PHY Power Off */
void otg_phy_off(void) {
	writel(readl(S3C_USBOTG_PHYPWR)|(0x1F<<1), S3C_USBOTG_PHYPWR);
	writel(readl(S3C_OTHERS)&~S3C_OTHERS_USB_SIG_MASK, S3C_OTHERS);
}
#endif

#ifdef CONFIG_USB_OHCI_HCD
void usb_host_clk_en(int usb_host_clksrc, u32 otg_phy_clk) {
	switch (usb_host_clksrc) {
	case 0: /* epll clk */
		writel((readl(S3C_CLK_SRC)& ~S3C_CLKSRC_UHOST_MASK)
			|S3C_CLKSRC_EPLL_CLKSEL|S3C_CLKSRC_UHOST_EPLL,
			S3C_CLK_SRC);

		/* USB host colock divider ratio is 2 */
		writel((readl(S3C_CLK_DIV1)& ~S3C_CLKDIVN_UHOST_MASK)
			|S3C_CLKDIV1_USBDIV2, S3C_CLK_DIV1);
		break;
	case 1: /* oscillator 48M clk */
		writel(readl(S3C_CLK_SRC)& ~S3C_CLKSRC_UHOST_MASK, S3C_CLK_SRC);
		otg_phy_init(otg_phy_clk);

		/* USB host colock divider ratio is 1 */
		writel(readl(S3C_CLK_DIV1)& ~S3C_CLKDIVN_UHOST_MASK, S3C_CLK_DIV1);
		break;
	default:
		printk(KERN_INFO "Unknown USB Host Clock Source\n");
		BUG();
		break;
	}

	writel(readl(S3C_HCLK_GATE)|S3C_CLKCON_HCLK_UHOST|S3C_CLKCON_HCLK_SECUR,
		S3C_HCLK_GATE);
	writel(readl(S3C_SCLK_GATE)|S3C_CLKCON_SCLK_UHOST, S3C_SCLK_GATE);

}
#endif

/* smdk6410 adc common function */

struct s3c_adc_cfg s3c_adc_platform={
	/* s3c6410 support 12-bit resolution */
	.delay	= 10000,
	.presc 	= 49,
	.resolution = 12,
};

#if defined(CONFIG_S3C_ADC)
unsigned int s3c_adc_convert(void __iomem *reg_base,unsigned int s3c_adc_port)
{
	unsigned int adc_return;
	unsigned long data0;
	unsigned long data1;

	writel(readl(reg_base+S3C2410_ADCCON)|S3C2410_ADCCON_SELMUX(s3c_adc_port), reg_base+S3C2410_ADCCON);

	udelay(10);

	writel(readl(reg_base+S3C2410_ADCCON)|S3C2410_ADCCON_ENABLE_START, reg_base+S3C2410_ADCCON);

       do {
	   	data0 = readl(reg_base+S3C2410_ADCCON);
	} while(!(data0 & S3C2410_ADCCON_ECFLG));

	data1 = readl(reg_base+S3C2410_ADCDAT0);

	adc_return = data1 & S3C_ADCDAT0_XPDATA_MASK_12BIT;

	return adc_return;
}
#endif


struct s3c_ts_mach_info s3c_ts_platform = {
                .delay = 10000,
                .presc = 49,
                .oversampling_shift = 2,
		.resol_bit = 12,
};

