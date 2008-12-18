/* linux/arch/arm/mach-s3c2450/mach-smdk2450.c
 *
 * Copyright (c) 2007 Samsung Electronics
 *
 *
 * Thanks to Samsung for the loan of an SMDK2450
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <asm/setup.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach-types.h>

#include <asm/plat-s3c/regs-serial.h>
#include <asm/plat-s3c/nand.h>
#include <asm/plat-s3c/adc.h>
#include <asm/arch/regs-gpio.h>
#include <asm/arch/regs-gpioj.h>
#include <asm/arch/regs-lcd.h>
#include <asm/arch/regs-mem.h>

#include <asm/arch/idle.h>
#include <asm/arch/fb.h>

#include <asm/plat-s3c24xx/clock.h>
#include <asm/plat-s3c24xx/devs.h>
#include <asm/plat-s3c24xx/cpu.h>

#include <asm/plat-s3c24xx/common-smdk.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include <asm/arch/hsmmc.h>

#if defined(CONFIG_RTC_DRV_S3C)
#include <asm/plat-s3c/regs-rtc.h>
#endif

#include <asm/plat-s3c/regs-adc.h>
#include <asm/plat-s3c/ts.h>

static struct map_desc smdk2450_iodesc[] __initdata = {

	/* ISA IO Space map (memory space selected by A24) */

	{
		.virtual	= (u32)S3C24XX_VA_ISA_WORD,
		.pfn		= __phys_to_pfn(S3C2410_CS2),
		.length		= 0x10000,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (u32)S3C24XX_VA_ISA_WORD + 0x10000,
		.pfn		= __phys_to_pfn(S3C2410_CS2 + (1<<24)),
		.length		= SZ_4M,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (u32)S3C24XX_VA_ISA_BYTE,
		.pfn		= __phys_to_pfn(S3C2410_CS2),
		.length		= 0x10000,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (u32)S3C24XX_VA_ISA_BYTE + 0x10000,
		.pfn		= __phys_to_pfn(S3C2410_CS2 + (1<<24)),
		.length		= SZ_4M,
		.type		= MT_DEVICE,
	}
};

#define UCON S3C2410_UCON_DEFAULT | S3C2410_UCON_UCLK
#define ULCON S3C2410_LCON_CS8 | S3C2410_LCON_PNONE | S3C2410_LCON_STOPB
#define UFCON S3C2410_UFCON_RXTRIG8 | S3C2410_UFCON_FIFOMODE

static struct s3c24xx_uart_clksrc smdk2450_serial_clocks[] = {
	[0] = {
		.name		= "pclk",
		.divisor	= 1,
		.min_baud	= 0,
		.max_baud	= 0,
	},
	[1] = {
		.name		= "esysclk",
		.divisor	= 1,
		.min_baud	= 0,
		.max_baud	= 0,
	}

};

static struct s3c2410_uartcfg smdk2450_uartcfgs[] __initdata = {
	[0] = {
		.hwport	     = 0,
		.flags	     = 0,
		.ucon	     = 0x3c5,
		.ulcon	     = 0x03,
		.ufcon	     = 0x51,
	},
	[1] = {
		.hwport	     = 1,
		.flags	     = 0,
		.ucon	     = 0xfc5,
		.ulcon	     = 0x03,
		.ufcon	     = 0x51,
		.clocks	     = smdk2450_serial_clocks,
		.clocks_size = ARRAY_SIZE(smdk2450_serial_clocks),
	},
	/* IR port */
	[2] = {
		.hwport	     = 2,
		.flags	     = 0,
		.ucon	     = 0xfc5,
		.ulcon	     = 0x43,
		.ufcon	     = 0x51,
		.clocks	     = smdk2450_serial_clocks,
		.clocks_size = ARRAY_SIZE(smdk2450_serial_clocks),
	}
};

static struct platform_device *smdk2450_devices[] __initdata = {
	&s3c_device_spi0,
	&s3c_device_spi1,
	&s3c_device_wdt,
	&s3c_device_i2c,
	&s3c_device_rtc,
	&s3c_device_adc,
	&s3c_device_ts,
	&s3c_device_iis,
	&s3c_device_usbgadget,
	&s3c_device_usb,
	&s3c_device_hsmmc0,
	&s3c_device_hsmmc1,
	&s3c_device_camif,
	&s3c_device_smc911x,
};

static void __init smdk2450_map_io(void)
{
	s3c24xx_init_io(smdk2450_iodesc, ARRAY_SIZE(smdk2450_iodesc));
	s3c24xx_init_clocks(12000000);
	s3c24xx_init_uarts(smdk2450_uartcfgs, ARRAY_SIZE(smdk2450_uartcfgs));
}

static void smdk2450_cs89x0_set(void)
{
	u32 val;

	val = readl(S3C_BANK_CFG);
	val &= ~((1<<8)|(1<<9)|(1<<10));
	writel(val, S3C_BANK_CFG);

	/* Bank1 Idle cycle ctrl. */
	writel(0xf, S3C_SSMC_SMBIDCYR1);

	/* Bank1 Read Wait State cont. = 14 clk          Tacc? */
	writel(12, S3C_SSMC_SMBWSTRDR1);

	/* Bank1 Write Wait State ctrl. */
	writel(12, S3C_SSMC_SMBWSTWRR1);

	/* Bank1 Output Enable Assertion Delay ctrl.     Tcho? */
	writel(2, S3C_SSMC_SMBWSTOENR1);

	/* Bank1 Write Enable Assertion Delay ctrl. */
	writel(2, S3C_SSMC_SMBWSTWENR1);

	/* SMWAIT active High, Read Byte Lane Enabl      WS1? */
	val = readl(S3C_SSMC_SMBCR1);

	val |=  ((1<<15)|(1<<7));
	writel(val, S3C_SSMC_SMBCR1);

	val = readl(S3C_SSMC_SMBCR1);
	val |=  ((1<<2)|(1<<0));
	writel(val, S3C_SSMC_SMBCR1);

	val = readl(S3C_SSMC_SMBCR1);
	val &= ~((3<<20)|(3<<12));
	writel(val, S3C_SSMC_SMBCR1);

	val = readl(S3C_SSMC_SMBCR1);
	val &= ~(3<<4);
	writel(val, S3C_SSMC_SMBCR1);

	val = readl(S3C_SSMC_SMBCR1);
	val |= (1<<4);

	writel(val, S3C_SSMC_SMBCR1);

}

static void smdk2450_smc911x_set(void)
{
	u32 val;

	/* Bank1 Idle cycle ctrl. */
	writel(0xf, S3C_SSMC_SMBIDCYR4);

	/* Bank1 Read Wait State cont. = 14 clk          Tacc? */
	writel(12, S3C_SSMC_SMBWSTRDR4);

	/* Bank1 Write Wait State ctrl. */
	writel(12, S3C_SSMC_SMBWSTWRR4);

	/* Bank1 Output Enable Assertion Delay ctrl.     Tcho? */
	writel(2, S3C_SSMC_SMBWSTOENR4);

	/* Bank1 Write Enable Assertion Delay ctrl. */
	writel(2, S3C_SSMC_SMBWSTWENR4);

	/* SMWAIT active High, Read Byte Lane Enabl      WS1? */
	val = readl(S3C_SSMC_SMBCR4);

	val |=  ((1<<15)|(1<<7));
	writel(val, S3C_SSMC_SMBCR4);

	val = readl(S3C_SSMC_SMBCR4);
	val |=  ((1<<2)|(1<<0));
	writel(val, S3C_SSMC_SMBCR4);

	val = readl(S3C_SSMC_SMBCR4);
	val &= ~((3<<20)|(3<<12));
	writel(val, S3C_SSMC_SMBCR4);

	val = readl(S3C_SSMC_SMBCR4);
	val &= ~(3<<4);
	writel(val, S3C_SSMC_SMBCR4);

	val = readl(S3C_SSMC_SMBCR4);
	val |= (1<<4);

	writel(val, S3C_SSMC_SMBCR4);

}


static void smdk2450_hsmmc_init (void)
{
	/* GPIO E (external interrupt) : Chip detect */
	s3c2410_gpio_cfgpin(S3C2410_GPF1, S3C2410_GPF1_EINT1);	/* GPF1 to EINT1 */
}

static void __init smdk2450_machine_init(void)
{
	smdk2450_smc911x_set();
	platform_add_devices(smdk2450_devices, ARRAY_SIZE(smdk2450_devices));
	smdk_machine_init();
	smdk2450_hsmmc_init();
}

static void __init smdk2450_fixup (struct machine_desc *desc, struct tag *tags,
	      char **cmdline, struct meminfo *mi)
{
	/*
	 * Bank start addresses are not present in the information
	 * passed in from the boot loader.  We could potentially
	 * detect them, but instead we hard-code them.
	 */
	mi->bank[0].start = PHYS_OFFSET;

#if defined(CONFIG_VIDEO_SAMSUNG)
	mi->bank[0].size = 49*1024*1024;
#else
	mi->bank[0].size = 64*1024*1024;
#endif
	mi->bank[0].node = 0;

	mi->nr_banks = 1;
}


MACHINE_START(SMDK2450, "SMDK2450")
	/* Maintainer: Ben Dooks <ben@fluff.org> */
	.phys_io	= S3C2410_PA_UART,
	.io_pg_offst	= (((u32)S3C24XX_VA_UART) >> 18) & 0xfffc,
	.boot_params	= S3C2410_SDRAM_PA + 0x100,

	.init_irq	= s3c24xx_init_irq,
	.map_io		= smdk2450_map_io,
	.fixup      	= smdk2450_fixup,
	.init_machine	= smdk2450_machine_init,
	.timer		= &s3c24xx_timer,
MACHINE_END


void hsmmc_set_gpio (uint channel, uint width)
{
	switch (channel) {

	/* can supports 1 and 4 bit bus */
	case 0:
		/* GPIO E : Command, Clock */
		s3c2410_gpio_cfgpin(S3C2410_GPE5,  S3C2450_GPE5_SD0_CLK);
		s3c2410_gpio_cfgpin(S3C2410_GPE6, S3C2450_GPE6_SD0_CMD);

		if (width == 1) {
			/* GPIO E : MMC DATA0[0] */
			s3c2410_gpio_cfgpin(S3C2410_GPE7,  S3C2450_GPE7_SD0_DAT0);
		}
		else if (width == 4) {
			/* GPIO E : MMC DATA0[0:3] */
			s3c2410_gpio_cfgpin(S3C2410_GPE7,  S3C2450_GPE7_SD0_DAT0);
			s3c2410_gpio_cfgpin(S3C2410_GPE8,  S3C2450_GPE8_SD0_DAT1);
			s3c2410_gpio_cfgpin(S3C2410_GPE9,  S3C2450_GPE9_SD0_DAT2);
			s3c2410_gpio_cfgpin(S3C2410_GPE10,  S3C2450_GPE10_SD0_DAT3);
		}
		break;

	/* can supports 1, 4, and 8 bit bus */
	case 1:
		/* GPIO L : Command, Clock */
		s3c2410_gpio_cfgpin(S3C2443_GPL8, S3C2450_GPL8_SD1CMD);
		s3c2410_gpio_cfgpin(S3C2443_GPL9, S3C2450_GPL9_SD1CLK);

		/* GPIO J : Chip detect, LED, Write Protect */
		s3c2410_gpio_cfgpin(S3C2443_GPJ13, S3C2450_GPJ13_SD1LED);
		s3c2410_gpio_cfgpin(S3C2443_GPJ14, S3C2450_GPJ14_nSD1CD);

		s3c2410_gpio_cfgpin(S3C2443_GPJ15, S3C2450_GPJ15_nSD1WP); /* write protect enable */
		s3c2410_gpio_setpin(S3C2443_GPJ15, 1);

		if (width == 1) {
			/* GPIO L : MMC DATA1[0] */
			s3c2410_gpio_cfgpin(S3C2443_GPL0, S3C2450_GPL0_SD1DAT0);
		}
		else if (width == 4) {
			/* GPIO L : MMC DATA1[0:3] */
			s3c2410_gpio_cfgpin(S3C2443_GPL0, S3C2450_GPL0_SD1DAT0);
			s3c2410_gpio_cfgpin(S3C2443_GPL1, S3C2450_GPL1_SD1DAT1);
			s3c2410_gpio_cfgpin(S3C2443_GPL2, S3C2450_GPL2_SD1DAT2);
			s3c2410_gpio_cfgpin(S3C2443_GPL3, S3C2450_GPL3_SD1DAT3);
		}
		else if (width == 8) {
			/* GPIO L : MMC DATA1[0:7] */
			s3c2410_gpio_cfgpin(S3C2443_GPL0, S3C2450_GPL0_SD1DAT0);
			s3c2410_gpio_cfgpin(S3C2443_GPL1, S3C2450_GPL1_SD1DAT1);
			s3c2410_gpio_cfgpin(S3C2443_GPL2, S3C2450_GPL2_SD1DAT2);
			s3c2410_gpio_cfgpin(S3C2443_GPL3, S3C2450_GPL3_SD1DAT3);

			s3c2410_gpio_cfgpin(S3C2443_GPL4, S3C2450_GPL4_SD1DAT4);
			s3c2410_gpio_cfgpin(S3C2443_GPL5, S3C2450_GPL5_SD1DAT5);
			s3c2410_gpio_cfgpin(S3C2443_GPL6, S3C2450_GPL6_SD1DAT6);
			s3c2410_gpio_cfgpin(S3C2443_GPL7, S3C2450_GPL7_SD1DAT7);
		}
		break;

	default:
		break;
	}
}


#define HOST_CAPS (MMC_CAP_4_BIT_DATA | MMC_CAP_MULTIWRITE | \
			MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED)

/* Channel 0 : added HS-MMC channel */
struct s3c_hsmmc_cfg s3c_hsmmc0_platform = {
	.hwport = 0,
	.enabled = 1,
	.host_caps = HOST_CAPS,
	.bus_width = 4,
	.highspeed = 0,

	/* ctrl for mmc */
	.fd_ctrl[MMC_TYPE_MMC] = {
		.ctrl2 = 0xC0000100,			/* ctrl2 for mmc */
		.ctrl3[SPEED_NORMAL] = 0,		/* ctrl3 for low speed */
		.ctrl3[SPEED_HIGH]   = 0,		/* ctrl3 for high speed */
		.ctrl4 = 0x3,
	},

	/* ctrl for sd */
	.fd_ctrl[MMC_TYPE_SD] = {
		.ctrl2 = 0xC0000100,			/* ctrl2 for sd */
		.ctrl3[SPEED_NORMAL] = 0,		/* ctrl3 for low speed */
		.ctrl3[SPEED_HIGH]   = 0,		/* ctrl3 for high speed */
		.ctrl4 = 0x3,
	},

	.clocks[0] = {
		.name = "hsmmc",
		.src = 0x1,
	},

	.clocks[1] = {
		.name = "esysclk",
		.src = 0x2,
	},

	.clocks[2] = {
		.name = "hsmmc-ext",
		.src = 0x3,
	},
};

/* Channel 1 : default HS-MMC channel */
struct s3c_hsmmc_cfg s3c_hsmmc1_platform = {
	.hwport = 1,	/* H/W Channel 1 */
	.enabled = 1,
	.host_caps = HOST_CAPS  | MMC_CAP_8_BIT_DATA,
	.bus_width = 8,
	//.host_caps = HOST_CAPS,
	//.bus_width = 4,

	.highspeed = 0,

	/* ctrl for mmc */
	.fd_ctrl[MMC_TYPE_MMC] = {
		.ctrl2 = 0xC0000100,			/* ctrl2 for mmc */
		.ctrl3[SPEED_NORMAL] = 0,		/* ctrl3 for low speed */
		.ctrl3[SPEED_HIGH]   = 0,		/* ctrl3 for high speed */
		.ctrl4 = 0x3,
	},

	/* ctrl for sd */
	.fd_ctrl[MMC_TYPE_SD] = {
		.ctrl2 = 0xC0000100,			/* ctrl2 for sd */
		.ctrl3[SPEED_NORMAL] = 0,		/* ctrl3 for low speed */
		.ctrl3[SPEED_HIGH]   = 0,		/* ctrl3 for high speed */
		.ctrl4 = 0x3,
	},

	.clocks[0] = {
		.name = "hsmmc",
		.src = 0x1,
	},

	.clocks[1] = {
		.name = "esysclk",
		.src = 0x2,
	},

	.clocks[2] = {
		.name = "hsmmc-ext",
		.src = 0x3,
	},
};

#if defined(CONFIG_RTC_DRV_S3C)
/* rtc function */
unsigned int s3c_rtc_set_bit_byte(void __iomem *base, uint offset, uint val)
{
	return 0;
}

unsigned int s3c_rtc_read_alarm_status(void __iomem *base)
{
	return (readb(base + S3C2410_RTCALM) & S3C2410_RTCALM_ALMEN);
}

void s3c_rtc_set_pie(void __iomem *base, uint to)
{
	unsigned int tmp;

	tmp = readb(base + S3C2410_TICNT) & ~S3C2410_TICNT_ENABLE;

        if (to)
                tmp |= S3C2410_TICNT_ENABLE;

        writeb(tmp, base + S3C2410_TICNT);

}

void s3c_rtc_set_freq_regs(void __iomem *base, uint freq, uint *s3c_freq)
{
	unsigned int tmp;

        tmp = readb(base + S3C2410_TICNT) & S3C2410_TICNT_ENABLE;
        writew(readw(base + S3C2410_RTCCON) & (~(1<<8)),base + S3C2410_RTCCON);
        *s3c_freq = freq;
        tmp |= (128 / freq)-1;
        writeb(tmp, base + S3C2410_TICNT);
}

void s3c_rtc_enable_set(struct platform_device *pdev,void __iomem *base, int en)
{
	unsigned int tmp;

	if (!en) {
		tmp = readb(base + S3C2410_RTCCON);
		writeb(tmp & ~S3C2410_RTCCON_RTCEN, base + S3C2410_RTCCON);

		tmp = readb(base + S3C2410_TICNT);
		writeb(tmp & ~S3C2410_TICNT_ENABLE, base + S3C2410_TICNT);
	} else {
		/* re-enable the device, and check it is ok */
		if ((readb(base+S3C2410_RTCCON) & S3C2410_RTCCON_RTCEN) == 0){
			dev_info(&pdev->dev, "rtc disabled, re-enabling\n");

			tmp = readb(base + S3C2410_RTCCON);
			writeb(tmp|S3C2410_RTCCON_RTCEN, base+S3C2410_RTCCON);
		}

		if ((readb(base + S3C2410_RTCCON) & S3C2410_RTCCON_CNTSEL)){
			dev_info(&pdev->dev, "removing RTCCON_CNTSEL\n");

			tmp = readb(base + S3C2410_RTCCON);
			writeb(tmp& ~S3C2410_RTCCON_CNTSEL, base+S3C2410_RTCCON);
		}

		if ((readb(base + S3C2410_RTCCON) & S3C2410_RTCCON_CLKRST)){
			dev_info(&pdev->dev, "removing RTCCON_CLKRST\n");

			tmp = readb(base + S3C2410_RTCCON);
			writeb(tmp & ~S3C2410_RTCCON_CLKRST, base+S3C2410_RTCCON);
		}
	}
}
#endif

struct s3c_adc_cfg s3c_adc_platform={
	/* s3c2450 support 12-bit resolution */
	.delay	= 10000,
	.presc 	= 49,
	.resolution = 12,
};

unsigned int s3c_adc_convert(void __iomem *reg_base,unsigned int s3c_adc_port)
{
	unsigned int adc_return;
	unsigned long data0;
	unsigned long data1;

	writel(S3C2443_ADCCON_SELMUX(s3c_adc_port), base_addr+S3C2443_ADCMUX);

	udelay(10);

	writel(readl(reg_base+S3C2410_ADCCON)|S3C2410_ADCCON_ENABLE_START, reg_base+S3C2410_ADCCON);

       do {
	   	data0 = readl(reg_base+S3C2410_ADCCON);
	} while(!(data0 & S3C2410_ADCCON_ECFLG));

	data1 = readl(reg_base+S3C2410_ADCDAT0);

	adc_return = data1 & S3C_ADCDAT0_XPDATA_MASK_12BIT;

	return adc_return;
}


struct s3c_ts_mach_info s3c_ts_platform = {
                .delay = 10000,
                .presc = 49,
                .oversampling_shift = 2,
		.resol_bit = 12,
};

