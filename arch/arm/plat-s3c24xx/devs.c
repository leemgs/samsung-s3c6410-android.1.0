/* linux/arch/arm/plat-s3c24xx/devs.c
 *
 * Copyright (c) 2004 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * Base S3C24XX platform device definitions
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
#include <asm/arch/fb.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>

#include <asm/plat-s3c/regs-serial.h>
#include <asm/plat-s3c24xx/udc.h>

#include <asm/plat-s3c24xx/devs.h>
#include <asm/plat-s3c24xx/common-smdk.h>
#include <asm/plat-s3c24xx/cpu.h>
#include <asm/plat-s3c24xx/regs-spi.h>

/****************************************************************************
 * common
 ***************************************************************************/

/* Serial port registrations */

static struct resource s3c2410_uart0_resource[] = {
	[0] = {
		.start = S3C2410_PA_UART0,
#if !defined(CONFIG_PLAT_S3C64XX) && !defined(CONFIG_PLAT_S5PC1XX)
		.end   = S3C2410_PA_UART0 + 0x3fff,
#else
		.end   = S3C2410_PA_UART0 + 0x3ff,
#endif
		.flags = IORESOURCE_MEM,
	},
#if !defined(CONFIG_PLAT_S3C64XX) && !defined(CONFIG_PLAT_S5PC1XX)
	[1] = {
		.start = IRQ_S3CUART_RX0,
		.end   = IRQ_S3CUART_ERR0,
		.flags = IORESOURCE_IRQ,
	}
#else
	[1] = {
		.start = IRQ_UART0,
		.end   = IRQ_UART0,
		.flags = IORESOURCE_IRQ,
	}
#endif
};

static struct resource s3c2410_uart1_resource[] = {
	[0] = {
		.start = S3C2410_PA_UART1,
#if !defined(CONFIG_PLAT_S3C64XX) && !defined(CONFIG_PLAT_S5PC1XX)
		.end   = S3C2410_PA_UART1 + 0x3fff,
#else
		.end   = S3C2410_PA_UART1 + 0x3ff,
#endif
		.flags = IORESOURCE_MEM,
	},
#if !defined(CONFIG_PLAT_S3C64XX) && !defined(CONFIG_PLAT_S5PC1XX)
	[1] = {
		.start = IRQ_S3CUART_RX1,
		.end   = IRQ_S3CUART_ERR1,
		.flags = IORESOURCE_IRQ,
	}
#else
	[1] = {
		.start = IRQ_UART1,
		.end   = IRQ_UART1,
		.flags = IORESOURCE_IRQ,
	}
#endif
};

static struct resource s3c2410_uart2_resource[] = {
	[0] = {
		.start = S3C2410_PA_UART2,
#if !defined(CONFIG_PLAT_S3C64XX) && !defined(CONFIG_PLAT_S5PC1XX)
		.end   = S3C2410_PA_UART2 + 0x3fff,
#else
		.end   = S3C2410_PA_UART2 + 0x3ff,
#endif
		.flags = IORESOURCE_MEM,
	},
#if !defined(CONFIG_PLAT_S3C64XX) && !defined(CONFIG_PLAT_S5PC1XX)
	[1] = {
		.start = IRQ_S3CUART_RX2,
		.end   = IRQ_S3CUART_ERR2,
		.flags = IORESOURCE_IRQ,
	}
#else
	[1] = {
		.start = IRQ_UART2,
		.end   = IRQ_UART2,
		.flags = IORESOURCE_IRQ,
	}
#endif
};

#if defined(CONFIG_PLAT_S3C64XX) || defined(CONFIG_PLAT_S5PC1XX)
static struct resource s3c_uart3_resource[] = {
	[0] = {
		.start = S3C2443_PA_UART3,
		.end   = S3C2443_PA_UART3 + 0x3ff,
		.flags = IORESOURCE_MEM,
	},

	[1] = {
		.start = IRQ_UART3,
		.end   = IRQ_UART3,
		.flags = IORESOURCE_IRQ,
	}
};
#endif

struct s3c24xx_uart_resources s3c2410_uart_resources[] __initdata = {
	[0] = {
		.resources	= s3c2410_uart0_resource,
		.nr_resources	= ARRAY_SIZE(s3c2410_uart0_resource),
	},
	[1] = {
		.resources	= s3c2410_uart1_resource,
		.nr_resources	= ARRAY_SIZE(s3c2410_uart1_resource),
	},
	[2] = {
		.resources	= s3c2410_uart2_resource,
		.nr_resources	= ARRAY_SIZE(s3c2410_uart2_resource),
	},
#if defined(CONFIG_PLAT_S3C64XX) || defined(CONFIG_PLAT_S5PC1XX)
	[3] = {
		.resources	= s3c_uart3_resource,
		.nr_resources	= ARRAY_SIZE(s3c_uart3_resource),
	},
#endif
};

/* uart devices */

static struct platform_device s3c24xx_uart_device0 = {
	.id		= 0,
};

static struct platform_device s3c24xx_uart_device1 = {
	.id		= 1,
};

static struct platform_device s3c24xx_uart_device2 = {
	.id		= 2,
};

#if defined(CONFIG_PLAT_S3C64XX) || defined(CONFIG_PLAT_S5PC1XX)
static struct platform_device s3c24xx_uart_device3 = {
	.id		= 3,
};
#endif

struct platform_device *s3c24xx_uart_src[] = {
	&s3c24xx_uart_device0,
	&s3c24xx_uart_device1,
	&s3c24xx_uart_device2,
#if defined(CONFIG_PLAT_S3C64XX) || defined(CONFIG_PLAT_S5PC1XX)
	&s3c24xx_uart_device3,
#endif
};

struct platform_device *s3c24xx_uart_devs[] = {
};

/* USB Host Controller */

static struct resource s3c_usb_resource[] = {
	[0] = {
		.start = S3C24XX_PA_USBHOST,
		.end   = S3C24XX_PA_USBHOST + S3C24XX_SZ_USBHOST - 1,
		.flags = IORESOURCE_MEM,
	},
#if !defined(CONFIG_PLAT_S3C64XX) && !defined(CONFIG_PLAT_S5PC1XX)
	[1] = {
		.start = IRQ_USBH,
		.end   = IRQ_USBH,
		.flags = IORESOURCE_IRQ,
	}
#else
	[1] = {
		.start = IRQ_UHOST,
		.end   = IRQ_UHOST,
		.flags = IORESOURCE_IRQ,
	}
#endif
};

static u64 s3c_device_usb_dmamask = 0xffffffffUL;

struct platform_device s3c_device_usb = {
	.name		  = "s3c2410-ohci",
	.id		  = -1,
	.num_resources	  = ARRAY_SIZE(s3c_usb_resource),
	.resource	  = s3c_usb_resource,
	.dev              = {
		.dma_mask = &s3c_device_usb_dmamask,
		.coherent_dma_mask = 0xffffffffUL
	}
};

EXPORT_SYMBOL(s3c_device_usb);

/* LCD Controller */

static struct resource s3c_lcd_resource[] = {
	[0] = {
		.start = S3C24XX_PA_LCD,
		.end   = S3C24XX_PA_LCD + S3C24XX_SZ_LCD - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
#if defined(CONFIG_CPU_S3C6400) || defined(CONFIG_CPU_S3C6410)
		.start = IRQ_LCD_VSYNC ,
		.end   = IRQ_LCD_SYSTEM,
#elif defined(CONFIG_PLAT_S3C24XX)
		.start = IRQ_LCD,
		.end   = IRQ_LCD,
#elif defined(CONFIG_PLAT_S5PC1XX)
		.start = IRQ_LCD0,
		.end   = IRQ_LCD3,
#endif
		.flags = IORESOURCE_IRQ,
	}

};

static u64 s3c_device_lcd_dmamask = 0xffffffffUL;

struct platform_device s3c_device_lcd = {
	.name		  = "s3c2410-lcd",
	.id		  = -1,
	.num_resources	  = ARRAY_SIZE(s3c_lcd_resource),
	.resource	  = s3c_lcd_resource,
	.dev              = {
		.dma_mask		= &s3c_device_lcd_dmamask,
		.coherent_dma_mask	= 0xffffffffUL
	}
};

EXPORT_SYMBOL(s3c_device_lcd);

void __init s3c24xx_fb_set_platdata(struct s3c2410fb_mach_info *pd)
{
	struct s3c2410fb_mach_info *npd;

	npd = kmalloc(sizeof(*npd), GFP_KERNEL);
	if (npd) {
		memcpy(npd, pd, sizeof(*npd));
		s3c_device_lcd.dev.platform_data = npd;
	} else {
		printk(KERN_ERR "no memory for LCD platform data\n");
	}
}

/* NAND Controller */

static struct resource s3c_nand_resource[] = {
	[0] = {
		.start = S3C24XX_PA_NAND,
		.end   = S3C24XX_PA_NAND + S3C24XX_SZ_NAND - 1,
		.flags = IORESOURCE_MEM,
	}
};

struct platform_device s3c_device_nand = {
	.name		  = "s3c2410-nand",
	.id		  = -1,
	.num_resources	  = ARRAY_SIZE(s3c_nand_resource),
	.resource	  = s3c_nand_resource,
};

EXPORT_SYMBOL(s3c_device_nand);

/* OneNAND Controller */
static struct resource s3c_onenand_resource[] = {
	[0] = {
		.start = S3C6400_PA_ONENAND,
		.end   = S3C6400_PA_ONENAND + S3C_SZ_ONENAND - 1,
		.flags = IORESOURCE_MEM,
	}
};

struct platform_device s3c_device_onenand = {
	.name		  = "onenand",
	.id		  = -1,
	.dev		= {
		.platform_data	= &s3c_onenand_data,
	},
	.num_resources	  = ARRAY_SIZE(s3c_onenand_resource),
	.resource	  = s3c_onenand_resource,
};

EXPORT_SYMBOL(s3c_device_onenand);

/* USB Device (Gadget)*/

static struct resource s3c_usbgadget_resource[] = {
#if defined(CONFIG_PLAT_S3C64XX)
	[0] = {
		.start = S3C24XX_PA_OTG,
		.end   = S3C24XX_PA_OTG+S3C24XX_SZ_OTG-1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_OTG,
		.end   = IRQ_OTG,
		.flags = IORESOURCE_IRQ,
	}
#elif defined(CONFIG_PLAT_S3C24XX)
	[0] = {
		.start = S3C24XX_PA_USBDEV,
		.end   = S3C24XX_PA_USBDEV + S3C24XX_SZ_USBDEV - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_USBD,
		.end   = IRQ_USBD,
		.flags = IORESOURCE_IRQ,
	}
#endif
};

struct platform_device s3c_device_usbgadget = {
	.name		  = "s3c2410-usbgadget",
	.id		  = -1,
	.num_resources	  = ARRAY_SIZE(s3c_usbgadget_resource),
	.resource	  = s3c_usbgadget_resource,
};

EXPORT_SYMBOL(s3c_device_usbgadget);

#ifdef CONFIG_PLAT_S3C64XX
/* USB Device (OTG hcd)*/

static struct resource s3c_usb_otghcd_resource[] = {
	[0] = {
		.start = S3C24XX_PA_OTG,
		.end   = S3C24XX_PA_OTG + S3C24XX_SZ_OTG - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_OTG,
		.end   = IRQ_OTG,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device s3c_device_usb_otghcd = {
	.name		= "s3c6410_OTGHCD",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(s3c_usb_otghcd_resource),
	.resource	= s3c_usb_otghcd_resource,
};

EXPORT_SYMBOL(s3c_device_usb_otghcd);
#endif

void __init s3c24xx_udc_set_platdata(struct s3c2410_udc_mach_info *pd)
{
	struct s3c2410_udc_mach_info *npd;

	npd = kmalloc(sizeof(*npd), GFP_KERNEL);
	if (npd) {
		memcpy(npd, pd, sizeof(*npd));
		s3c_device_usbgadget.dev.platform_data = npd;
	} else {
		printk(KERN_ERR "no memory for udc platform data\n");
	}
}


/* Watchdog */

static struct resource s3c_wdt_resource[] = {
	[0] = {
		.start = S3C24XX_PA_WATCHDOG,
		.end   = S3C24XX_PA_WATCHDOG + S3C24XX_SZ_WATCHDOG - 1,
		.flags = IORESOURCE_MEM,
	},
#if defined(CONFIG_CPU_S3C2443) || defined(CONFIG_CPU_S3C2450) || defined(CONFIG_CPU_S3C2416)
	[1] = {
		.start = IRQ_S3C2443_WDT,
		.end   = IRQ_S3C2443_WDT,
		.flags = IORESOURCE_IRQ,
	}

#else
	[1] = {
		.start = IRQ_WDT,
		.end   = IRQ_WDT,
		.flags = IORESOURCE_IRQ,
	}
#endif
};

struct platform_device s3c_device_wdt = {
	.name		  = "s3c2410-wdt",
	.id		  = -1,
	.num_resources	  = ARRAY_SIZE(s3c_wdt_resource),
	.resource	  = s3c_wdt_resource,
};

EXPORT_SYMBOL(s3c_device_wdt);

/* I2C */

static struct resource s3c_i2c_resource[] = {
	[0] = {
		.start = S3C24XX_PA_IIC,
		.end   = S3C24XX_PA_IIC + S3C24XX_SZ_IIC - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_IIC,
		.end   = IRQ_IIC,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device s3c_device_i2c = {
	.name		  = "s3c2410-i2c",
	.id		  = -1,
	.num_resources	  = ARRAY_SIZE(s3c_i2c_resource),
	.resource	  = s3c_i2c_resource,
};

EXPORT_SYMBOL(s3c_device_i2c);

/* IIS */

static struct resource s3c_iis_resource[] = {
	[0] = {
		.start = S3C24XX_PA_IIS,
		.end   = S3C24XX_PA_IIS + S3C24XX_SZ_IIS -1,
		.flags = IORESOURCE_MEM,
	},
#if defined(CONFIG_CPU_S3C6410)
	[1] = {
		.start = IRQ_IIS,
		.end   = IRQ_IIS,
		.flags = IORESOURCE_IRQ,
	}
#endif
};

static u64 s3c_device_iis_dmamask = 0xffffffffUL;

struct platform_device s3c_device_iis = {
	.name		  = "s3c2410-iis",
	.id		  = -1,
	.num_resources	  = ARRAY_SIZE(s3c_iis_resource),
	.resource	  = s3c_iis_resource,
	.dev              = {
		.dma_mask = &s3c_device_iis_dmamask,
		.coherent_dma_mask = 0xffffffffUL
	}
};

EXPORT_SYMBOL(s3c_device_iis);

/* AC97 */

static struct resource s3c_ac97_resource[] = {
	[0] = {
		.start = S3C24XX_PA_AC97,
		.end   = S3C24XX_PA_AC97 + S3C24XX_SZ_AC97 -1,
		.flags = IORESOURCE_MEM,
	}
};

static u64 s3c_device_ac97_dmamask = 0xffffffffUL;

struct platform_device s3c_device_ac97 = {
	.name		  = "s3c-ac97",
	.id		  = -1,
	.num_resources	  = ARRAY_SIZE(s3c_ac97_resource),
	.resource	  = s3c_ac97_resource,
	.dev              = {
		.dma_mask = &s3c_device_ac97_dmamask,
		.coherent_dma_mask = 0xffffffffUL
	}
};

EXPORT_SYMBOL(s3c_device_ac97);

/* RTC */

static struct resource s3c_rtc_resource[] = {
	[0] = {
		.start = S3C24XX_PA_RTC,
		.end   = S3C24XX_PA_RTC + 0xff,
		.flags = IORESOURCE_MEM,
	},
#if !defined(CONFIG_PLAT_S3C64XX) && !defined(CONFIG_PLAT_S5PC1XX)
	[1] = {
		.start = IRQ_RTC,
		.end   = IRQ_RTC,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.start = IRQ_TICK,
		.end   = IRQ_TICK,
		.flags = IORESOURCE_IRQ
	}
#else
	[1] = {
		.start = IRQ_RTC_ALARM,
		.end   = IRQ_RTC_ALARM,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.start = IRQ_RTC_TIC,
		.end   = IRQ_RTC_TIC,
		.flags = IORESOURCE_IRQ
	}
#endif
};

struct platform_device s3c_device_rtc = {
	.name		  = "s3c2410-rtc",
	.id		  = -1,
	.num_resources	  = ARRAY_SIZE(s3c_rtc_resource),
	.resource	  = s3c_rtc_resource,
};

EXPORT_SYMBOL(s3c_device_rtc);

/* ADC */
extern struct s3c_adc_cfg s3c_adc_platform;

static struct resource s3c_adc_resource[] = {
	[0] = {
		.start = S3C24XX_PA_ADC,
		.end   = S3C24XX_PA_ADC + S3C24XX_SZ_ADC - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_TC,
		.end   = IRQ_TC,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.start = IRQ_ADC,
		.end   = IRQ_ADC,
		.flags = IORESOURCE_IRQ,
	}

};

struct platform_device s3c_device_adc = {
	.name		  = "s3c-adc",
	.id		  = -1,
	.num_resources	  = ARRAY_SIZE(s3c_adc_resource),
	.resource	  = s3c_adc_resource,
	.dev		  ={
			.platform_data = &s3c_adc_platform,
	}
};
EXPORT_SYMBOL(s3c_device_adc);

/* Touch srcreen */
extern struct s3c_ts_mach_info s3c_ts_platform;
static struct resource s3c_ts_resource[] = {
	[0] = {
		.start = S3C24XX_PA_ADC,
		.end   = S3C24XX_PA_ADC + S3C24XX_SZ_ADC - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_TC,
		.end   = IRQ_ADC,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device s3c_device_ts = {
	.name		  = "s3c-ts",
	.id		  = -1,
	.num_resources	  = ARRAY_SIZE(s3c_ts_resource),
	.resource	  = s3c_ts_resource,
	.dev		  ={
			.platform_data = &s3c_ts_platform,
	}
};

/* SDI */

static struct resource s3c_sdi_resource[] = {
	[0] = {
		.start = S3C2410_PA_SDI,
		.end   = S3C2410_PA_SDI + S3C24XX_SZ_SDI - 1,
		.flags = IORESOURCE_MEM,
	},
#if !defined(CONFIG_PLAT_S3C64XX) && !defined(CONFIG_PLAT_S5PC1XX)
	[1] = {
		.start = IRQ_SDI,
		.end   = IRQ_SDI,
		.flags = IORESOURCE_IRQ,
	}
#endif
};

struct platform_device s3c_device_sdi = {
	.name		  = "s3c2410-sdi",
	.id		  = -1,
	.num_resources	  = ARRAY_SIZE(s3c_sdi_resource),
	.resource	  = s3c_sdi_resource,
};

EXPORT_SYMBOL(s3c_device_sdi);


/* SPI (0) */
#if defined(CONFIG_CPU_S3C2443) || defined(CONFIG_CPU_S3C2450) || defined(CONFIG_CPU_S3C2416)

static struct resource s3c_spi0_resource[] = {
	[0] = {
		.start = S3C_PA_SPI_0,
		.end   = S3C_PA_SPI_0 + S3C_SZ_SPI_0,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SPI0,
		.end   = IRQ_SPI0,
		.flags = IORESOURCE_IRQ,
	}

};

static u64 s3c_device_spi0_dmamask = 0xffffffffUL;

struct platform_device s3c_device_spi0 = {
	.name		  = "s3c2410-spi",
	.id		  = 0,
	.num_resources	  = ARRAY_SIZE(s3c_spi0_resource),
	.resource	  = s3c_spi0_resource,
        .dev              = {
                .dma_mask = &s3c_device_spi0_dmamask,
                .coherent_dma_mask = 0xffffffffUL
        }
};

EXPORT_SYMBOL(s3c_device_spi0);

static struct resource s3c_spi1_resource[] = {
	[0] = {
		.start = S3C_PA_SPI_0,
		.end   = S3C_PA_SPI_0 + S3C_SZ_SPI_0,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SPI1,
		.end   = IRQ_SPI1,
		.flags = IORESOURCE_IRQ,
	}

};

struct platform_device s3c_device_spi1 = {
	.name		  = "s3c-spi",
	.id		  = 1,
	.num_resources	  = ARRAY_SIZE(s3c_spi1_resource),
	.resource	  = s3c_spi1_resource,
};

EXPORT_SYMBOL(s3c_device_spi1);

#else

static struct resource s3c_spi0_resource[] = {
	[0] = {
		.start = S3C24XX_PA_SPI,
#if !defined (CONFIG_CPU_S3C6400) && !defined (CONFIG_CPU_S3C6410)
		.end   = S3C24XX_PA_SPI + 0x1f,
#else
		.end   = S3C24XX_PA_SPI + S3C24XX_SZ_SPI - 1,
#endif
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SPI0,
		.end   = IRQ_SPI0,
		.flags = IORESOURCE_IRQ,
	}

};

static u64 s3c_device_spi0_dmamask = 0xffffffffUL;

struct platform_device s3c_device_spi0 = {
	.name		  = "s3c2410-spi",
	.id		  = 0,
	.num_resources	  = ARRAY_SIZE(s3c_spi0_resource),
	.resource	  = s3c_spi0_resource,
        .dev              = {
                .dma_mask = &s3c_device_spi0_dmamask,
                .coherent_dma_mask = 0xffffffffUL
        }
};

EXPORT_SYMBOL(s3c_device_spi0);


/* SPI (1) */

static struct resource s3c_spi1_resource[] = {
	[0] = {
#if !defined (CONFIG_CPU_S3C6400) && !defined (CONFIG_CPU_S3C6410)
		.start = S3C24XX_PA_SPI + 0x20,
		.end   = S3C24XX_PA_SPI + 0x20 + 0x1f,
#else
		.start = S3C24XX_PA_SPI + S3C24XX_SZ_SPI,
		.end   = S3C24XX_PA_SPI + S3C24XX_SZ_SPI + 0xff,
#endif
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SPI1,
		.end   = IRQ_SPI1,
		.flags = IORESOURCE_IRQ,
	}

};

static u64 s3c_device_spi1_dmamask = 0xffffffffUL;

struct platform_device s3c_device_spi1 = {
	.name		  = "s3c2410-spi",
	.id		  = 1,
	.num_resources	  = ARRAY_SIZE(s3c_spi1_resource),
	.resource	  = s3c_spi1_resource,
        .dev              = {
                .dma_mask = &s3c_device_spi1_dmamask,
                .coherent_dma_mask = 0xffffffffUL
        }
};

EXPORT_SYMBOL(s3c_device_spi1);

#endif

/* pwm timer blocks */

static struct resource s3c_timer0_resource[] = {
	[0] = {
		.start = S3C24XX_PA_TIMER + 0x0C,
		.end   = S3C24XX_PA_TIMER + 0x0C + 0xB,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_TIMER0,
		.end   = IRQ_TIMER0,
		.flags = IORESOURCE_IRQ,
	}

};

struct platform_device s3c_device_timer0 = {
	.name		  = "s3c2410-timer",
	.id		  = 0,
	.num_resources	  = ARRAY_SIZE(s3c_timer0_resource),
	.resource	  = s3c_timer0_resource,
};

EXPORT_SYMBOL(s3c_device_timer0);

/* timer 1 */

static struct resource s3c_timer1_resource[] = {
	[0] = {
		.start = S3C24XX_PA_TIMER + 0x18,
		.end   = S3C24XX_PA_TIMER + 0x23,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_TIMER1,
		.end   = IRQ_TIMER1,
		.flags = IORESOURCE_IRQ,
	}

};

struct platform_device s3c_device_timer1 = {
	.name		  = "s3c2410-timer",
	.id		  = 1,
	.num_resources	  = ARRAY_SIZE(s3c_timer1_resource),
	.resource	  = s3c_timer1_resource,
};

EXPORT_SYMBOL(s3c_device_timer1);

/* timer 2 */

static struct resource s3c_timer2_resource[] = {
	[0] = {
		.start = S3C24XX_PA_TIMER + 0x24,
		.end   = S3C24XX_PA_TIMER + 0x2F,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_TIMER2,
		.end   = IRQ_TIMER2,
		.flags = IORESOURCE_IRQ,
	}

};

struct platform_device s3c_device_timer2 = {
	.name		  = "s3c2410-timer",
	.id		  = 2,
	.num_resources	  = ARRAY_SIZE(s3c_timer2_resource),
	.resource	  = s3c_timer2_resource,
};

EXPORT_SYMBOL(s3c_device_timer2);

/* timer 3 */

static struct resource s3c_timer3_resource[] = {
	[0] = {
		.start = S3C24XX_PA_TIMER + 0x30,
		.end   = S3C24XX_PA_TIMER + 0x3B,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_TIMER3,
		.end   = IRQ_TIMER3,
		.flags = IORESOURCE_IRQ,
	}

};

struct platform_device s3c_device_timer3 = {
	.name		  = "s3c2410-timer",
	.id		  = 3,
	.num_resources	  = ARRAY_SIZE(s3c_timer3_resource),
	.resource	  = s3c_timer3_resource,
};

EXPORT_SYMBOL(s3c_device_timer3);

/****************************************************************************
 * S3C2440 only: camif
 ***************************************************************************/
#if defined(CONFIG_CPU_S3C2440) && (!defined(CONFIG_CPU_S3C2443) && !defined(CONFIG_CPU_S3C2450))

/* Camif Controller */
static struct resource s3c_camif_resource[] = {
	[0] = {
		.start = S3C2440_PA_CAMIF,
		.end   = S3C2440_PA_CAMIF + S3C2440_SZ_CAMIF - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_CAM,
		.end   = IRQ_CAM,
		.flags = IORESOURCE_IRQ,
	}

};

static u64 s3c_device_camif_dmamask = 0xffffffffUL;

struct platform_device s3c_device_camif = {
	.name		  = "s3c2440-camif",
	.id		  = -1,
	.num_resources	  = ARRAY_SIZE(s3c_camif_resource),
	.resource	  = s3c_camif_resource,
	.dev              = {
		.dma_mask = &s3c_device_camif_dmamask,
		.coherent_dma_mask = 0xffffffffUL
	}
};

EXPORT_SYMBOL(s3c_device_camif);

#endif /* End of S3C2440 only */

/****************************************************************************
 * S3C2443, S3C2450 common
 ***************************************************************************/
#if defined(CONFIG_CPU_S3C2443) || defined(CONFIG_CPU_S3C2450) || defined(CONFIG_CPU_S3C2416)
#if !defined(CONFIG_CPU_S3C2416)
/* Camif controller */
static struct resource s3c_camif_resource[] = {
	[0] = {
		.start = S3C2443_PA_CAMIF,
		.end   = S3C2443_PA_CAMIF + S3C2443_SZ_CAMIF - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_S3C2440_CAM_C,
		.end   = IRQ_S3C2440_CAM_C,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.start = IRQ_S3C2440_CAM_P,
		.end   = IRQ_S3C2440_CAM_P,
		.flags = IORESOURCE_IRQ,
	}
};

static u64 s3c_device_camif_dmamask = 0xffffffffUL;

struct platform_device s3c_device_camif = {
	.name		  = "s3c-camif",
	.id		  = -1,
	.num_resources	  = ARRAY_SIZE(s3c_camif_resource),
	.resource	  = s3c_camif_resource,
	.dev              = {
		.dma_mask = &s3c_device_camif_dmamask,
		.coherent_dma_mask = 0xffffffffUL
	}
};

EXPORT_SYMBOL(s3c_device_camif);

/* Ide controller */
static struct resource s3c_ide_resource[] = {
	[0] = {
		.start = S3C_PA_CFATA,
		.end   = S3C_PA_CFATA+ S3C_SZ_CFATA,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_CFCON,
		.end   = IRQ_CFCON,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device s3c_device_ide = {
	.name		  = "s3c-ide",
	.id		  = 0,
	.num_resources	  = ARRAY_SIZE(s3c_ide_resource),
	.resource	  = s3c_ide_resource,
};

EXPORT_SYMBOL(s3c_device_ide);
#endif

#if defined(CONFIG_CPU_S3C2450) || defined(CONFIG_CPU_S3C2416)
/* HS-MMC Controller */
extern struct s3c_hsmmc_cfg s3c_hsmmc0_platform;
extern struct s3c_hsmmc_cfg s3c_hsmmc1_platform;

static struct resource s3c_hsmmc0_resource[] = {
	[0] = {
		.start = S3C_PA_HSMMC+0x400000,
		.end   = S3C_PA_HSMMC+0x400000+S3C_SZ_HSMMC,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SDI_0,
		.end   = IRQ_SDI_0,
		.flags = IORESOURCE_IRQ,
	},
	/* To detect a card inserted, use an external interrupt */
	[2] = {
		.start = IRQ_EINT1,
		.end   = IRQ_EINT1,
		.flags = IORESOURCE_IRQ,
	}
};

static struct resource s3c_hsmmc1_resource[] = {
	[0] = {
		.start = S3C_PA_HSMMC,
		.end   = S3C_PA_HSMMC+S3C_SZ_HSMMC,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SDI_1,
		.end   = IRQ_SDI_1,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device s3c_device_hsmmc0 = {
	.name		  = "s3c-hsmmc",
	.id		  = 0,
	.num_resources	  = ARRAY_SIZE(s3c_hsmmc0_resource),
	.resource	  = s3c_hsmmc0_resource,
	.dev		= {
		.platform_data = &s3c_hsmmc0_platform,
	}
};


struct platform_device s3c_device_hsmmc1 = {
	.name		  = "s3c-hsmmc",
	.id		  = 1,
	.num_resources	  = ARRAY_SIZE(s3c_hsmmc1_resource),
	.resource	  = s3c_hsmmc1_resource,
	.dev		= {
		.platform_data = &s3c_hsmmc1_platform,
	}
};

EXPORT_SYMBOL(s3c_device_hsmmc0);
EXPORT_SYMBOL(s3c_device_hsmmc1);

#else
/* HS-MMC controller */
extern struct s3c_hsmmc_cfg s3c_hsmmc_platform;

static struct resource s3c_hsmmc_resource[] = {
	[0] = {
		.start = S3C_PA_HSMMC,
		.end   = S3C_PA_HSMMC+ S3C_SZ_HSMMC,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SDI_1,
		.end   = IRQ_SDI_1,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device s3c_device_hsmmc = {
	.name		  = "s3c-hsmmc",
	.id		  = 0,
	.num_resources	  = ARRAY_SIZE(s3c_hsmmc_resource),
	.resource	  = s3c_hsmmc_resource,
	.dev		= {
		.platform_data = &s3c_hsmmc_platform,
	}
};

EXPORT_SYMBOL(s3c_device_hsmmc);
#endif

#endif /* End of S3C2443, S3C2450 common */

/****************************************************************************
 * S3C6400, S3C6410 common
 ***************************************************************************/
#if defined(CONFIG_CPU_S3C6400) || defined(CONFIG_CPU_S3C6410)

/* HS-MMC controller */
extern struct s3c_hsmmc_cfg s3c_hsmmc0_platform;
extern struct s3c_hsmmc_cfg s3c_hsmmc1_platform;
extern struct s3c_hsmmc_cfg s3c_hsmmc2_platform;

static struct resource s3c_hsmmc0_resource[] = {
	[0] = {
		.start = S3C_PA_HSMMC,
		.end   = S3C_PA_HSMMC+S3C_SZ_HSMMC-1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_HSMMC0,
		.end   = IRQ_HSMMC0,
		.flags = IORESOURCE_IRQ,
	},
	/* To detect a card inserted, use an external interrupt */
	[2] = {
		.start = IRQ_EINT13,
		.end   = IRQ_EINT13,
		.flags = IORESOURCE_IRQ,
	}
};

static struct resource s3c_hsmmc1_resource[] = {
	[0] = {
		.start = S3C_PA_HSMMC+0x100000,
		.end   = S3C_PA_HSMMC+0x100000+S3C_SZ_HSMMC-1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_HSMMC1,
		.end   = IRQ_HSMMC1,
		.flags = IORESOURCE_IRQ,
	}
};

static struct resource s3c_hsmmc2_resource[] = {
	[0] = {
		.start = S3C_PA_HSMMC+0x200000,
		.end   = S3C_PA_HSMMC+0x200000+S3C_SZ_HSMMC-1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_HSMMC2,
		.end   = IRQ_HSMMC2,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.start = IRQ_EINT15,
		.end   = IRQ_EINT15,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device s3c_device_hsmmc0 = {
	.name		  = "s3c-hsmmc",
	.id		  = 0,
	.num_resources	  = ARRAY_SIZE(s3c_hsmmc0_resource),
	.resource	  = s3c_hsmmc0_resource,
	.dev		= {
		.platform_data = &s3c_hsmmc0_platform,
	}
};

struct platform_device s3c_device_hsmmc1 = {
	.name		  = "s3c-hsmmc",
	.id		  = 1,
	.num_resources	  = ARRAY_SIZE(s3c_hsmmc1_resource),
	.resource	  = s3c_hsmmc1_resource,
	.dev		= {
		.platform_data = &s3c_hsmmc1_platform,
	}
};

struct platform_device s3c_device_hsmmc2 = {
	.name		  = "s3c-hsmmc",
	.id		  = 2,
	.num_resources	  = ARRAY_SIZE(s3c_hsmmc2_resource),
	.resource	  = s3c_hsmmc2_resource,
	.dev		= {
		.platform_data = &s3c_hsmmc2_platform,
	}
};

EXPORT_SYMBOL(s3c_device_hsmmc0);
EXPORT_SYMBOL(s3c_device_hsmmc1);
EXPORT_SYMBOL(s3c_device_hsmmc2);

/* 2D interface */
static struct resource s3c_2d_resource[] = {
	[0] = {
		.start = S3C6400_PA_2D,
		.end   = S3C6400_PA_2D + S3C_SZ_2D - 1,
		.flags = IORESOURCE_MEM,
		},
	[1] = {
                .start = IRQ_2D,
                .end   = IRQ_2D,
                .flags = IORESOURCE_IRQ,
        }
};

struct platform_device s3c_device_2d = {
        .name             = "s3c-2d",
        .id               = -1,
        .num_resources    = ARRAY_SIZE(s3c_2d_resource),
        .resource         = s3c_2d_resource
};

EXPORT_SYMBOL(s3c_device_2d);

/* rotator interface */
static struct resource s3c_rotator_resource[] = {
	[0] = {
		.start = S3C6400_PA_ROTATOR,
		.end   = S3C6400_PA_ROTATOR + S3C_SZ_ROTATOR - 1,
		.flags = IORESOURCE_MEM,
		},
	[1] = {
                .start = IRQ_ROTATOR,
                .end   = IRQ_ROTATOR,
                .flags = IORESOURCE_IRQ,
        }
};

struct platform_device s3c_device_rotator = {
        .name             = "s3c-rotator",
        .id               = -1,
        .num_resources    = ARRAY_SIZE(s3c_rotator_resource),
        .resource         = s3c_rotator_resource
};

EXPORT_SYMBOL(s3c_device_rotator);

/* TV encoder */
static struct resource s3c_tvenc_resource[] = {
	[0] = {
		.start = S3C24XX_PA_TVENC,
		.end   = S3C24XX_PA_TVENC + S3C_SZ_TVENC - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_TVENC,
		.end   = IRQ_TVENC,
		.flags = IORESOURCE_IRQ,
	}

};

struct platform_device s3c_device_tvenc = {
	.name		  = "s3c-tvenc",
	.id		  = -1,
	.num_resources	  = ARRAY_SIZE(s3c_tvenc_resource),
	.resource	  = s3c_tvenc_resource,
};

EXPORT_SYMBOL(s3c_device_tvenc);

/* TV scaler */
static struct resource s3c_tvscaler_resource[] = {
	[0] = {
		.start = S3C24XX_PA_TVSCALER,
		.end   = S3C24XX_PA_TVSCALER + S3C_SZ_TVSCALER - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SCALER,
		.end   = IRQ_SCALER,
		.flags = IORESOURCE_IRQ,
	}

};

struct platform_device s3c_device_tvscaler = {
	.name		  = "s3c-tvscaler",
	.id		  = -1,
	.num_resources	  = ARRAY_SIZE(s3c_tvscaler_resource),
	.resource	  = s3c_tvscaler_resource,
};

EXPORT_SYMBOL(s3c_device_tvscaler);

/* Camif controller */
static struct resource s3c_camif_resource[] = {
	[0] = {
		.start = S3C6400_PA_CAMIF,
		.end   = S3C6400_PA_CAMIF + S3C24XX_SZ_CAMIF - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_CAMIF_C,
		.end   = IRQ_CAMIF_C,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.start = IRQ_CAMIF_P,
		.end   = IRQ_CAMIF_P,
		.flags = IORESOURCE_IRQ,
	}

};

static u64 s3c_device_camif_dmamask = 0xffffffffUL;

struct platform_device s3c_device_camif = {
	.name		  = "s3c-camif",
	.id		  = -1,
	.num_resources	  = ARRAY_SIZE(s3c_camif_resource),
	.resource	  = s3c_camif_resource,
	.dev              = {
		.dma_mask = &s3c_device_camif_dmamask,
		.coherent_dma_mask = 0xffffffffUL
	}
};

EXPORT_SYMBOL(s3c_device_camif);

/* JPEG controller */
static struct resource s3c_jpeg_resource[] = {
	[0] = {
		.start = S3C6400_PA_JPEG,
		.end   = S3C6400_PA_JPEG + S3C_SZ_JPEG - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_JPEG,
		.end   = IRQ_JPEG,
		.flags = IORESOURCE_IRQ,
	}

};

struct platform_device s3c_device_jpeg = {
	.name		  = "s3c-jpeg",
	.id		  = -1,
	.num_resources	  = ARRAY_SIZE(s3c_jpeg_resource),
	.resource	  = s3c_jpeg_resource,
};

EXPORT_SYMBOL(s3c_device_jpeg);

/* MFC controller */
static struct resource s3c_mfc_resource[] = {
	[0] = {
		.start = S3C6400_PA_MFC,
		.end   = S3C6400_PA_MFC + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
		},
	[1] = {
                .start = IRQ_MFC,
                .end   = IRQ_MFC,
                .flags = IORESOURCE_IRQ,
        }
};

struct platform_device s3c_device_mfc = {
        .name             = "s3c-mfc",
        .id               = -1,
        .num_resources    = ARRAY_SIZE(s3c_mfc_resource),
        .resource         = s3c_mfc_resource
};

EXPORT_SYMBOL(s3c_device_mfc);

/* VPP controller */
static struct resource s3c_vpp_resource[] = {
	[0] = {
		.start = S3C6400_PA_VPP,
		.end   = S3C6400_PA_VPP + S3C_SZ_VPP - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_POST0,
		.end   = IRQ_POST0,
		.flags = IORESOURCE_IRQ,
	}

};

struct platform_device s3c_device_vpp = {
	.name		  = "s3c-vpp",
	.id		  = -1,
	.num_resources	  = ARRAY_SIZE(s3c_vpp_resource),
	.resource	  = s3c_vpp_resource,
};

EXPORT_SYMBOL(s3c_device_vpp);

/* IDE controller */
static struct resource s3c_ide_resource[] = {
	[0] = {
		.start = S3C24XX_PA_CFATA,
		.end   = S3C24XX_PA_CFATA + S3C_SZ_CFATA - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_CFCON,
		.end   = IRQ_CFCON,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device s3c_device_ide = {
	.name		  = "s3c-ide",
	.id		  = 0,
	.num_resources	  = ARRAY_SIZE(s3c_ide_resource),
	.resource	  = s3c_ide_resource,
};

EXPORT_SYMBOL(s3c_device_ide);

/* Keypad interface */
static struct resource s3c_keypad_resource[] = {
	[0] = {
		.start = S3C24XX_PA_KEYPAD,
		.end   = S3C24XX_PA_KEYPAD+ S3C24XX_SZ_KEYPAD - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_KEYPAD,
		.end   = IRQ_KEYPAD,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device s3c_device_keypad = {
	.name		  = "s3c-keypad",
	.id		  = -1,
	.num_resources	  = ARRAY_SIZE(s3c_keypad_resource),
	.resource	  = s3c_keypad_resource,
};

EXPORT_SYMBOL(s3c_device_keypad);

#if defined(CONFIG_CPU_S3C6410) /* S3C6410 only */

/* Add here for 6410 only devices */
/* 3D interface */
static struct resource s3c_g3d_resource[] = {
	[0] = {
		.start = S3C6410_PA_G3D,
		.end   = S3C6410_PA_G3D + S3C6410_SZ_G3D - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
                .start = IRQ_G3D,
                .end   = IRQ_G3D,
                .flags = IORESOURCE_IRQ,
        }
};

struct platform_device s3c_device_g3d = {
        .name             = "s3c-g3d",
        .id               = -1,
        .num_resources    = ARRAY_SIZE(s3c_g3d_resource),
        .resource         = s3c_g3d_resource
};

EXPORT_SYMBOL(s3c_device_g3d);

#endif /* End of S3C6410 only */

#endif /* End of S3C6400, S3C6410 common */

#if defined (CONFIG_CPU_S3C6410) || defined(CONFIG_CPU_S3C2450)  || defined(CONFIG_CPU_S3C2416)
static struct resource s3c_smc911x_resources[] = {
      [0] = {
              .start  = S3C_PA_SMC9115,
              .end    = S3C_PA_SMC9115 + 0x1fffffff,
              .flags  = IORESOURCE_MEM,
      },
      [1] = {
#if defined(CONFIG_CPU_S3C6410)
              .start = IRQ_EINT10,
              .end   = IRQ_EINT10,
#elif defined(CONFIG_CPU_S3C2450) || defined(CONFIG_CPU_S3C2416)
              .start = IRQ_EINT4,
              .end   = IRQ_EINT4,
#endif
              .flags = IORESOURCE_IRQ,
        },
};

struct platform_device s3c_device_smc911x = {
      .name           = "smc911x",
      .id             =  -1,
      .num_resources  = ARRAY_SIZE(s3c_smc911x_resources),
      .resource       = s3c_smc911x_resources,
};
EXPORT_SYMBOL(s3c_device_smc911x);
#endif

