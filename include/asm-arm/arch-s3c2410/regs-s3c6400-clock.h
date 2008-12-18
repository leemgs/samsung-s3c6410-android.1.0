/* linux/include/asm-arm/arch-s3c2410/regs-s3c6400-clock.h
 *
 * Copyright (c) 2003,2004,2005,2006 Simtec Electronics <linux@simtec.co.uk>
 *		      http://armlinux.simtec.co.uk/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * S3C6400 clock register definitions
*/

#ifndef __ASM_ARM_REGS_CLOCK
#define __ASM_ARM_REGS_CLOCK "$Id: regs-s3c6400-clock.h,v 1.2 2008/03/20 07:12:45 ihlee215 Exp $"

#define S3C_CLKREG(x) ((x) + S3C24XX_VA_SYSCON)

#define S3C_PLLVAL(_m,_p,_s) ((_m) << 16 | ((_p) << 8) | ((_s)))

#define S3C_APLL_LOCK		S3C_CLKREG(0x00)
#define S3C_MPLL_LOCK		S3C_CLKREG(0x04)
#define S3C_EPLL_LOCK		S3C_CLKREG(0x08)
#define S3C_APLL_CON		S3C_CLKREG(0x0C)
#define S3C_MPLL_CON		S3C_CLKREG(0x10)
#define S3C_EPLL_CON0		S3C_CLKREG(0x14)
#define S3C_EPLL_CON1		S3C_CLKREG(0x18)
#define S3C_CLK_SRC		S3C_CLKREG(0x1C)
#define S3C_CLK_SRC2		S3C_CLKREG(0x10C)
#define S3C_CLK_DIV0		S3C_CLKREG(0x20)
#define S3C_CLK_DIV1		S3C_CLKREG(0x24)
#define S3C_CLK_DIV2		S3C_CLKREG(0x28)
#define S3C_CLK_OUT		S3C_CLKREG(0x2C)
#define S3C_HCLK_GATE		S3C_CLKREG(0x30)
#define S3C_PCLK_GATE		S3C_CLKREG(0x34)
#define S3C_SCLK_GATE		S3C_CLKREG(0x38)
#define S3C_AHB_CON0		S3C_CLKREG(0x100)
#define S3C_AHB_CON1		S3C_CLKREG(0x104)
#define S3C_AHB_CON2		S3C_CLKREG(0x108)
#define S3C_SDMA_SEL		S3C_CLKREG(0x110)
#define S3C_SW_RST		S3C_CLKREG(0x114)
#define S3C_SYS_ID		S3C_CLKREG(0x118)
#define S3C_MEM_SYS_CFG		S3C_CLKREG(0x120)
#define S3C_QOS_OVERRIDE0	S3C_CLKREG(0x124)
#define S3C_QOS_OVERRIDE1	S3C_CLKREG(0x128)
#define S3C_MEM_CFG_STAT	S3C_CLKREG(0x12C)
#define S3C_PWR_CFG		S3C_CLKREG(0x804)
#define S3C_EINT_MASK		S3C_CLKREG(0x808)
#define S3C_NORMAL_CFG		S3C_CLKREG(0x810)
#define S3C_STOP_CFG		S3C_CLKREG(0x814)
#define S3C_SLEEP_CFG		S3C_CLKREG(0x818)
#define S3C_OSC_FREQ		S3C_CLKREG(0x820)
#define S3C_OSC_STABLE		S3C_CLKREG(0x824)
#define S3C_PWR_STABLE		S3C_CLKREG(0x828)
#define S3C_FPC_STABLE		S3C_CLKREG(0x82C)
#define S3C_MTC_STABLE		S3C_CLKREG(0x830)
#define S3C_OTHERS		S3C_CLKREG(0x900)
#define S3C_RST_STAT		S3C_CLKREG(0x904)
#define S3C_WAKEUP_STAT		S3C_CLKREG(0x908)
#define S3C_BLK_PWR_STAT	S3C_CLKREG(0x90C)
#define S3C_INFORM0		S3C_CLKREG(0xA00)
#define S3C_INFORM1		S3C_CLKREG(0xA04)
#define S3C_INFORM2		S3C_CLKREG(0xA08)
#define S3C_INFORM3		S3C_CLKREG(0xA0C)
#define S3C_INFORM4		S3C_CLKREG(0xA10)
#define S3C_INFORM5		S3C_CLKREG(0xA14)
#define S3C_INFORM6		S3C_CLKREG(0xA18)
#define S3C_INFORM7		S3C_CLKREG(0xA1C)

/* Block power status register bit field */
#define S3C_BLK_ETM		(1<<6)
#define S3C_BLK_S		(1<<5)
#define S3C_BLK_F		(1<<4)
#define S3C_BLK_P		(1<<3)
#define S3C_BLK_I		(1<<2)
#define S3C_BLK_V		(1<<1)
#define S3C_BLK_TOP		(1<<0)

/* Power gating registers */
#define S3C_PWRGATE_IROM        (1<<30)
#define S3C_PWRGATE_DOMAIN_ETM  (1<<16)
#define S3C_PWRGATE_DOMAIN_S    (1<<15)
#define S3C_PWRGATE_DOMAIN_F    (1<<14)
#define S3C_PWRGATE_DOMAIN_P    (1<<13)
#define S3C_PWRGATE_DOMAIN_I    (1<<12)
#define S3C_PWRGATE_DOMAIN_V    (1<<9)

/* MTC stable registers */
#define S3C_STABLE_DOMAIN_ETM   (0xF<<24)
#define S3C_STABLE_DOMAIN_S     (0xF<<20)
#define S3C_STABLE_DOMAIN_F     (0xF<<16)
#define S3C_STABLE_DOMAIN_P     (0xF<<12)
#define S3C_STABLE_DOMAIN_I     (0xF<<8)
#define S3C_STABLE_DOMAIN_V     (0xF<<4)
#define S3C_STABLE_DOMAIN_TOP   (0xF<<0)

/* HCLK GATE Registers */
#define S3C_CLKCON_HCLK_BUS	(1<<30)
#define S3C_CLKCON_HCLK_SECUR	(1<<29)
#define S3C_CLKCON_HCLK_SDMA1	(1<<28)
#define S3C_CLKCON_HCLK_SDMA2	(1<<27)
#define S3C_CLKCON_HCLK_UHOST	(1<<26)
#define S3C_CLKCON_HCLK_IROM	(1<<25)
#define S3C_CLKCON_HCLK_DDR1	(1<<24)
#define S3C_CLKCON_HCLK_DDR0	(1<<23)
#define S3C_CLKCON_HCLK_MEM1	(1<<22)
#define S3C_CLKCON_HCLK_MEM0	(1<<21)
#define S3C_CLKCON_HCLK_USB	(1<<20)
#define S3C_CLKCON_HCLK_HSMMC2	(1<<19)
#define S3C_CLKCON_HCLK_HSMMC1	(1<<18)
#define S3C_CLKCON_HCLK_HSMMC0	(1<<17)
#define S3C_CLKCON_HCLK_MDP	(1<<16)
#define S3C_CLKCON_HCLK_DHOST	(1<<15)
#define S3C_CLKCON_HCLK_IHOST	(1<<14)
#define S3C_CLKCON_HCLK_DMA1	(1<<13)
#define S3C_CLKCON_HCLK_DMA0	(1<<12)
#define S3C_CLKCON_HCLK_JPEG	(1<<11)
#define S3C_CLKCON_HCLK_CAMIF	(1<<10)
#define S3C_CLKCON_HCLK_SCALER	(1<<9)
#define S3C_CLKCON_HCLK_2D	(1<<8)
#define S3C_CLKCON_HCLK_TV	(1<<7)
#define S3C_CLKCON_HCLK_POST0	(1<<5)
#define S3C_CLKCON_HCLK_ROT	(1<<4)
#define S3C_CLKCON_HCLK_LCD	(1<<3)
#define S3C_CLKCON_HCLK_TZIC	(1<<2)
#define S3C_CLKCON_HCLK_INTC	(1<<1)
#define S3C_CLKCON_HCLK_MFC	(1<<0)

/* PCLK GATE Registers */
#define S3C_CLKCON_PCLK_IIC1	(1<<27)
#define S3C_CLKCON_PCLK_IIS2	(1<<26)
#define S3C_CLKCON_PCLK_SKEY	(1<<24)
#define S3C_CLKCON_PCLK_CHIPID	(1<<23)
#define S3C_CLKCON_PCLK_SPI1	(1<<22)
#define S3C_CLKCON_PCLK_SPI0	(1<<21)
#define S3C_CLKCON_PCLK_HSIRX	(1<<20)
#define S3C_CLKCON_PCLK_HSITX	(1<<19)
#define S3C_CLKCON_PCLK_GPIO	(1<<18)
#define S3C_CLKCON_PCLK_IIC	(1<<17)
#define S3C_CLKCON_PCLK_IIS1	(1<<16)
#define S3C_CLKCON_PCLK_IIS0	(1<<15)
#define S3C_CLKCON_PCLK_AC97	(1<<14)
#define S3C_CLKCON_PCLK_TZPC	(1<<13)
#define S3C_CLKCON_PCLK_TSADC	(1<<12)
#define S3C_CLKCON_PCLK_KEYPAD	(1<<11)
#define S3C_CLKCON_PCLK_IRDA	(1<<10)
#define S3C_CLKCON_PCLK_PCM1	(1<<9)
#define S3C_CLKCON_PCLK_PCM0	(1<<8)
#define S3C_CLKCON_PCLK_PWM	(1<<7)
#define S3C_CLKCON_PCLK_RTC	(1<<6)
#define S3C_CLKCON_PCLK_WDT	(1<<5)
#define S3C_CLKCON_PCLK_UART3	(1<<4)
#define S3C_CLKCON_PCLK_UART2	(1<<3)
#define S3C_CLKCON_PCLK_UART1	(1<<2)
#define S3C_CLKCON_PCLK_UART0	(1<<1)
#define S3C_CLKCON_PCLK_MFC	(1<<0)

/* SCLK GATE Registers */
#define S3C_CLKCON_SCLK_UHOST		(1<<30)
#define S3C_CLKCON_SCLK_MMC2_48		(1<<29)
#define S3C_CLKCON_SCLK_MMC1_48		(1<<28)
#define S3C_CLKCON_SCLK_MMC0_48		(1<<27)
#define S3C_CLKCON_SCLK_MMC2		(1<<26)
#define S3C_CLKCON_SCLK_MMC1		(1<<25)
#define S3C_CLKCON_SCLK_MMC0		(1<<24)
#define S3C_CLKCON_SCLK_SPI1_48 	(1<<23)
#define S3C_CLKCON_SCLK_SPI0_48 	(1<<22)
#define S3C_CLKCON_SCLK_SPI1		(1<<21)
#define S3C_CLKCON_SCLK_SPI0		(1<<20)
#define S3C_CLKCON_SCLK_DAC27		(1<<19)
#define S3C_CLKCON_SCLK_TV27		(1<<18)
#define S3C_CLKCON_SCLK_SCALER27	(1<<17)
#define S3C_CLKCON_SCLK_SCALER		(1<<16)
#define S3C_CLKCON_SCLK_LCD27		(1<<15)
#define S3C_CLKCON_SCLK_LCD		(1<<14)
#if defined(CONFIG_CPU_S3C6410)
#define S3C_CLKCON_SCLK_FIMC	        (1<<13)
#define S3C_CLKCON_SCLK_POST0_27	(1<<12)
#define S3C_CLKCON_SCLK_AUDIO2		(1<<11)
#else
#define S3C_CLKCON_SCLK_POST1_27	(1<<13)
#define S3C_CLKCON_SCLK_POST0_27	(1<<12)
#define S3C_CLKCON_SCLK_POST1		(1<<11)
#endif
#define S3C_CLKCON_SCLK_POST0		(1<<10)
#define S3C_CLKCON_SCLK_AUDIO1		(1<<9)
#define S3C_CLKCON_SCLK_AUDIO0		(1<<8)
#define S3C_CLKCON_SCLK_SECUR		(1<<7)
#define S3C_CLKCON_SCLK_IRDA		(1<<6)
#define S3C_CLKCON_SCLK_UART		(1<<5)
#define S3C_CLKCON_SCLK_ONENAND 	(1<<4)
#define S3C_CLKCON_SCLK_MFC		(1<<3)
#define S3C_CLKCON_SCLK_CAM		(1<<2)
#define S3C_CLKCON_SCLK_JPEG		(1<<1)

/*OTHERS Resgister */
#define S3C_OTHERS_USB_SIG_MASK		(1<<16)

/*CLK SRC BITS*/
#define S3C_CLKSRC_APLL_CLKSEL		(1<<0)
#define S3C_CLKSRC_MPLL_CLKSEL		(1<<1)
#define S3C_CLKSRC_EPLL_CLKSEL		(1<<2)
#define S3C_CLKSRC_UHOST_EPLL		(1<<5)
#define S3C_CLKSRC_UHOST_MASK		(3<<5)
#if 0
#define S3C_CLKSRC_CAM_CLKSEL_HCLK	(1<<9)
#define S3C_CLKSRC_I2SDIV_CLKSRC	(1<<10)
#define S3C_CLKSRC_I2SCLK_CLKSEL	(1<<11)
#define S3C_CLKSRC_UARTDIV_CLKSRC	(1<<12)
#define S3C_CLKSRC_SPIDIV_CLKSRC	(1<<13)
#endif

/*CLKDIV1 Reg bits */
#define S3C_CLKDIV1_USBDIV2		(1<<20)

#define S3C_CLKDIV1_HSMMCDIV2_MASK	(0xf<<8)
#define S3C_CLKDIV1_HSMMCDIV2_SHIFT	(8)
#define S3C_CLKDIV1_HSMMCDIV1_MASK	(0xf<<4)
#define S3C_CLKDIV1_HSMMCDIV1_SHIFT	(4)
#define S3C_CLKDIV1_HSMMCDIV0_MASK	(0xf<<0)
#define S3C_CLKDIV1_HSMMCDIV0_SHIFT	(0)
#define S3C_CLKDIV1_HSMMCDIV_MASK	(0xfff<<0)
#define S3C_CLKDIV1_HSMMCDIV_SHIFT	(0)

/*EPLL_CON0 Reg bits */
#define S3C_EPLL_EN	(1<<31)
#define S3C_EPLLVAL(_m,_p,_s)	((_m) << 16 | ((_p) << 8) | ((_s)))



#define S3C_CLKDIVN_APLL_MASK	(0xF<<0)
#define S3C_CLKDIVN_MPLL_MASK	(0x1<<4)
#define S3C_CLKDIVN_HCLK_MASK	(0x1<<8)
#define S3C_CLKDIVN_HCLKX2_MASK	(0x7<<9)
#define S3C_CLKDIVN_PCLK_MASK	(0xF<<12)
#define S3C_CLKDIVN_UHOST_MASK	(0xF<<20)

static inline unsigned int
s3c6400_get_pll(unsigned long  pllval, unsigned long baseclk)
{
	unsigned long  mdiv, pdiv, sdiv;

	/* To prevent overflow in calculation -JaeCheol Lee */
	baseclk /= 1000;

	mdiv = (pllval & (0x3ff << 16))>>16;
	pdiv = (pllval & (0x3f << 8))>>8;
	sdiv = (pllval & (0x03 << 0))>>0;
	return (baseclk * (mdiv)) / ((pdiv) << sdiv)*1000;
}

static inline unsigned int
s3c6400_get_epll(unsigned long baseclk)
{
	unsigned long  pllval, mdiv, pdiv, sdiv, kdiv;

	/* To prevent overflow in calculation -JaeCheol Lee */
	baseclk /= 1000;

	pllval = readl(S3C_EPLL_CON0);
	mdiv = (pllval & (0x3ff << 16))>>16;
	pdiv = (pllval & (0x3f << 8))>>8;
	sdiv = (pllval & (0x03 << 0))>>0;
	kdiv = readl(S3C_EPLL_CON1) & (0xffff);

	return (baseclk * (mdiv+kdiv/65536) / (pdiv << sdiv))*1000;
}

#endif /* __ASM_ARM_REGS_CLOCK */
