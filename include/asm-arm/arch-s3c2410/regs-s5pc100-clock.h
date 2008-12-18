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
#define __ASM_ARM_REGS_CLOCK

#define S5P_CLKREG(x) ((x) + S3C24XX_VA_SYSCON)

#define S5P_PLLVAL(_m,_p,_s) ((_m) << 16 | ((_p) << 8) | ((_s)))

#define S5P_APLL_LOCK		S5P_CLKREG(0x00)
#define S5P_MPLL_LOCK		S5P_CLKREG(0x04)
#define S5P_EPLL_LOCK		S5P_CLKREG(0x08)
#define S5P_HPLL_LOCK		S5P_CLKREG(0x0C)

#define S5P_APLL_CON		S5P_CLKREG(0x100)
#define S5P_MPLL_CON		S5P_CLKREG(0x104)
#define S5P_EPLL_CON		S5P_CLKREG(0x108)
#define S5P_HPLL_CON		S5P_CLKREG(0x10C)

#define S5P_CLK_SRC0		S5P_CLKREG(0x200)
#define S5P_CLK_SRC1		S5P_CLKREG(0x204)
#define S5P_CLK_SRC2		S5P_CLKREG(0x208)
#define S5P_CLK_SRC3		S5P_CLKREG(0x20C)

#define S5P_CLK_DIV0		S5P_CLKREG(0x300)
#define S5P_CLK_DIV1		S5P_CLKREG(0x304)
#define S5P_CLK_DIV2		S5P_CLKREG(0x308)
#define S5P_CLK_DIV3		S5P_CLKREG(0x30C)
#define S5P_CLK_DIV4		S5P_CLKREG(0x310)
#define S5P_CLK_DIV5		S5P_CLKREG(0x314)

#define S5P_CLK_OUT		S5P_CLKREG(0x400)

#define S5P_CLK_GATE_D0_0	S5P_CLKREG(0x500)
#define S5P_CLK_GATE_D0_1	S5P_CLKREG(0x504)
#define S5P_CLK_GATE_D0_2	S5P_CLKREG(0x508)

#define S5P_CLK_GATE_D1_0	S5P_CLKREG(0x520)
#define S5P_CLK_GATE_D1_1	S5P_CLKREG(0x524)
#define S5P_CLK_GATE_D1_2	S5P_CLKREG(0x528)
#define S5P_CLK_GATE_D1_3	S5P_CLKREG(0x52C)
#define S5P_CLK_GATE_D1_4	S5P_CLKREG(0x530)
#define S5P_CLK_GATE_D1_5	S5P_CLKREG(0x534)

#define S5P_CLK_GATE_D2_0	S5P_CLKREG(0x540)

#define S5P_SCLK_GATE_0		S5P_CLKREG(0x560)
#define S5P_SCLK_GATE_1		S5P_CLKREG(0x564)

#if 0
#define S5P_AHB_CON0		S5P_CLKREG(0x100)
#define S5P_AHB_CON1		S5P_CLKREG(0x104)
#define S5P_AHB_CON2		S5P_CLKREG(0x108)
#define S5P_SDMA_SEL		S5P_CLKREG(0x110)
#define S5P_SW_RST		S5P_CLKREG(0x114)
#define S5P_SYS_ID		S5P_CLKREG(0x118)
#define S5P_MEM_SYS_CFG		S5P_CLKREG(0x120)
#define S5P_QOS_OVERRIDE0	S5P_CLKREG(0x124)
#define S5P_QOS_OVERRIDE1	S5P_CLKREG(0x128)
#define S5P_MEM_CFG_STAT	S5P_CLKREG(0x12C)
#define S5P_PWR_CFG		S5P_CLKREG(0x804)
#define S5P_EINT_MASK		S5P_CLKREG(0x808)
#define S5P_NORMAL_CFG		S5P_CLKREG(0x810)
#define S5P_STOP_CFG		S5P_CLKREG(0x814)
#define S5P_SLEEP_CFG		S5P_CLKREG(0x818)
#define S5P_OSC_FREQ		S5P_CLKREG(0x820)
#define S5P_OSC_STABLE		S5P_CLKREG(0x824)
#define S5P_PWR_STABLE		S5P_CLKREG(0x828)
#define S5P_FPC_STABLE		S5P_CLKREG(0x82C)
#define S5P_MTC_STABLE		S5P_CLKREG(0x830)
#define S5P_OTHERS		S5P_CLKREG(0x900)
#define S5P_RST_STAT		S5P_CLKREG(0x904)
#define S5P_WAKEUP_STAT		S5P_CLKREG(0x908)
#define S5P_BLK_PWR_STAT	S5P_CLKREG(0x90C)
#define S5P_INFORM0		S5P_CLKREG(0xA00)
#define S5P_INFORM1		S5P_CLKREG(0xA04)
#define S5P_INFORM2		S5P_CLKREG(0xA08)
#define S5P_INFORM3		S5P_CLKREG(0xA0C)
#define S5P_INFORM4		S5P_CLKREG(0xA10)
#define S5P_INFORM5		S5P_CLKREG(0xA14)
#define S5P_INFORM6		S5P_CLKREG(0xA18)
#define S5P_INFORM7		S5P_CLKREG(0xA1C)

/* Block power status register bit field */
#define S5P_BLK_ETM		(1<<6)
#define S5P_BLK_S		(1<<5)
#define S5P_BLK_F		(1<<4)
#define S5P_BLK_P		(1<<3)
#define S5P_BLK_I		(1<<2)
#define S5P_BLK_V		(1<<1)
#define S5P_BLK_TOP		(1<<0)

/* Power gating registers */
#define S5P_PWRGATE_IROM        (1<<30)
#define S5P_PWRGATE_DOMAIN_ETM  (1<<16)
#define S5P_PWRGATE_DOMAIN_S    (1<<15)
#define S5P_PWRGATE_DOMAIN_F    (1<<14)
#define S5P_PWRGATE_DOMAIN_P    (1<<13)
#define S5P_PWRGATE_DOMAIN_I    (1<<12)
#define S5P_PWRGATE_DOMAIN_V    (1<<9)

/* MTC stable registers */
#define S5P_STABLE_DOMAIN_ETM   (0xF<<24)
#define S5P_STABLE_DOMAIN_S     (0xF<<20)
#define S5P_STABLE_DOMAIN_F     (0xF<<16)
#define S5P_STABLE_DOMAIN_P     (0xF<<12)
#define S5P_STABLE_DOMAIN_I     (0xF<<8)
#define S5P_STABLE_DOMAIN_V     (0xF<<4)
#define S5P_STABLE_DOMAIN_TOP   (0xF<<0)

/* HCLK GATE Registers */
#define S5P_CLKCON_HCLK_BUS	(1<<30)
#define S5P_CLKCON_HCLK_SECUR	(1<<29)
#define S5P_CLKCON_HCLK_SDMA1	(1<<28)
#define S5P_CLKCON_HCLK_SDMA2	(1<<27)
#define S5P_CLKCON_HCLK_UHOST	(1<<26)
#define S5P_CLKCON_HCLK_IROM	(1<<25)
#define S5P_CLKCON_HCLK_DDR1	(1<<24)
#define S5P_CLKCON_HCLK_DDR0	(1<<23)
#define S5P_CLKCON_HCLK_MEM1	(1<<22)
#define S5P_CLKCON_HCLK_MEM0	(1<<21)
#define S5P_CLKCON_HCLK_USB	(1<<20)
#define S5P_CLKCON_HCLK_HSMMC2	(1<<19)
#define S5P_CLKCON_HCLK_HSMMC1	(1<<18)
#define S5P_CLKCON_HCLK_HSMMC0	(1<<17)
#define S5P_CLKCON_HCLK_MDP	(1<<16)
#define S5P_CLKCON_HCLK_DHOST	(1<<15)
#define S5P_CLKCON_HCLK_IHOST	(1<<14)
#define S5P_CLKCON_HCLK_DMA1	(1<<13)
#define S5P_CLKCON_HCLK_DMA0	(1<<12)
#define S5P_CLKCON_HCLK_JPEG	(1<<11)
#define S5P_CLKCON_HCLK_CAMIF	(1<<10)
#define S5P_CLKCON_HCLK_SCALER	(1<<9)
#define S5P_CLKCON_HCLK_2D	(1<<8)
#define S5P_CLKCON_HCLK_TV	(1<<7)
#define S5P_CLKCON_HCLK_POST0	(1<<5)
#define S5P_CLKCON_HCLK_ROT	(1<<4)
#define S5P_CLKCON_HCLK_LCD	(1<<3)
#define S5P_CLKCON_HCLK_TZIC	(1<<2)
#define S5P_CLKCON_HCLK_INTC	(1<<1)
#define S5P_CLKCON_HCLK_MFC	(1<<0)

/* PCLK GATE Registers */
#define S5P_CLKCON_PCLK_IIC1	(1<<27)
#define S5P_CLKCON_PCLK_IIS2	(1<<26)
#define S5P_CLKCON_PCLK_SKEY	(1<<24)
#define S5P_CLKCON_PCLK_CHIPID	(1<<23)
#define S5P_CLKCON_PCLK_SPI1	(1<<22)
#define S5P_CLKCON_PCLK_SPI0	(1<<21)
#define S5P_CLKCON_PCLK_HSIRX	(1<<20)
#define S5P_CLKCON_PCLK_HSITX	(1<<19)
#define S5P_CLKCON_PCLK_GPIO	(1<<18)
#define S5P_CLKCON_PCLK_IIC	(1<<17)
#define S5P_CLKCON_PCLK_IIS1	(1<<16)
#define S5P_CLKCON_PCLK_IIS0	(1<<15)
#define S5P_CLKCON_PCLK_AC97	(1<<14)
#define S5P_CLKCON_PCLK_TZPC	(1<<13)
#define S5P_CLKCON_PCLK_TSADC	(1<<12)
#define S5P_CLKCON_PCLK_KEYPAD	(1<<11)
#define S5P_CLKCON_PCLK_IRDA	(1<<10)
#define S5P_CLKCON_PCLK_PCM1	(1<<9)
#define S5P_CLKCON_PCLK_PCM0	(1<<8)
#define S5P_CLKCON_PCLK_PWM	(1<<7)
#define S5P_CLKCON_PCLK_RTC	(1<<6)
#define S5P_CLKCON_PCLK_WDT	(1<<5)
#define S5P_CLKCON_PCLK_UART3	(1<<4)
#define S5P_CLKCON_PCLK_UART2	(1<<3)
#define S5P_CLKCON_PCLK_UART1	(1<<2)
#define S5P_CLKCON_PCLK_UART0	(1<<1)
#define S5P_CLKCON_PCLK_MFC	(1<<0)

/* SCLK GATE Registers */
#define S5P_CLKCON_SCLK_UHOST		(1<<30)
#define S5P_CLKCON_SCLK_MMC2_48		(1<<29)
#define S5P_CLKCON_SCLK_MMC1_48		(1<<28)
#define S5P_CLKCON_SCLK_MMC0_48		(1<<27)
#define S5P_CLKCON_SCLK_MMC2		(1<<26)
#define S5P_CLKCON_SCLK_MMC1		(1<<25)
#define S5P_CLKCON_SCLK_MMC0		(1<<24)
#define S5P_CLKCON_SCLK_SPI1_48 	(1<<23)
#define S5P_CLKCON_SCLK_SPI0_48 	(1<<22)
#define S5P_CLKCON_SCLK_SPI1		(1<<21)
#define S5P_CLKCON_SCLK_SPI0		(1<<20)
#define S5P_CLKCON_SCLK_DAC27		(1<<19)
#define S5P_CLKCON_SCLK_TV27		(1<<18)
#define S5P_CLKCON_SCLK_SCALER27	(1<<17)
#define S5P_CLKCON_SCLK_SCALER		(1<<16)
#define S5P_CLKCON_SCLK_LCD27		(1<<15)
#define S5P_CLKCON_SCLK_LCD		(1<<14)
#if defined(CONFIG_CPU_S3C6410)
#define S5P_CLKCON_SCLK_FIMC	        (1<<13)
#define S5P_CLKCON_SCLK_POST0_27	(1<<12)
#define S5P_CLKCON_SCLK_AUDIO2		(1<<11)
#else
#define S5P_CLKCON_SCLK_POST1_27	(1<<13)
#define S5P_CLKCON_SCLK_POST0_27	(1<<12)
#define S5P_CLKCON_SCLK_POST1		(1<<11)
#endif
#define S5P_CLKCON_SCLK_POST0		(1<<10)
#define S5P_CLKCON_SCLK_AUDIO1		(1<<9)
#define S5P_CLKCON_SCLK_AUDIO0		(1<<8)
#define S5P_CLKCON_SCLK_SECUR		(1<<7)
#define S5P_CLKCON_SCLK_IRDA		(1<<6)
#define S5P_CLKCON_SCLK_UART		(1<<5)
#define S5P_CLKCON_SCLK_ONENAND 	(1<<4)
#define S5P_CLKCON_SCLK_MFC		(1<<3)
#define S5P_CLKCON_SCLK_CAM		(1<<2)
#define S5P_CLKCON_SCLK_JPEG		(1<<1)

/*OTHERS Resgister */
#define S5P_OTHERS_USB_SIG_MASK		(1<<16)

/*CLK SRC BITS*/
#define S5P_CLKSRC_APLL_CLKSEL		(1<<0)
#define S5P_CLKSRC_MPLL_CLKSEL		(1<<1)
#define S5P_CLKSRC_EPLL_CLKSEL		(1<<2)
#define S5P_CLKSRC_UHOST_EPLL		(1<<5)
#define S5P_CLKSRC_UHOST_MASK		(3<<5)
#if 0
#define S5P_CLKSRC_CAM_CLKSEL_HCLK	(1<<9)
#define S5P_CLKSRC_I2SDIV_CLKSRC	(1<<10)
#define S5P_CLKSRC_I2SCLK_CLKSEL	(1<<11)
#define S5P_CLKSRC_UARTDIV_CLKSRC	(1<<12)
#define S5P_CLKSRC_SPIDIV_CLKSRC	(1<<13)
#endif

/*CLKDIV1 Reg bits */
#define S5P_CLKDIV1_USBDIV2		(1<<20)

#define S5P_CLKDIV1_HSMMCDIV2_MASK	(0xf<<8)
#define S5P_CLKDIV1_HSMMCDIV2_SHIFT	(8)
#define S5P_CLKDIV1_HSMMCDIV1_MASK	(0xf<<4)
#define S5P_CLKDIV1_HSMMCDIV1_SHIFT	(4)
#define S5P_CLKDIV1_HSMMCDIV0_MASK	(0xf<<0)
#define S5P_CLKDIV1_HSMMCDIV0_SHIFT	(0)
#define S5P_CLKDIV1_HSMMCDIV_MASK	(0xfff<<0)
#define S5P_CLKDIV1_HSMMCDIV_SHIFT	(0)

/*EPLL_CON0 Reg bits */
#define S5P_EPLL_EN	(1<<31)
#define S5P_EPLLVAL(_m,_p,_s)	((_m) << 16 | ((_p) << 8) | ((_s)))



#define S5P_CLKDIVN_APLL_MASK	(0xF<<0)
#define S5P_CLKDIVN_MPLL_MASK	(0x1<<4)
#define S5P_CLKDIVN_HCLK_MASK	(0x1<<8)
#define S5P_CLKDIVN_HCLKX2_MASK	(0x7<<9)
#define S5P_CLKDIVN_PCLK_MASK	(0xF<<12)
#define S5P_CLKDIVN_UHOST_MASK	(0xF<<20)

#endif

#define S5P_CLKDIV0_APLL_MASK	(0x1<<0)
#define S5P_CLKDIV0_ARM_MASK	(0x7<<4)
#define S5P_CLKDIV0_D0_MASK	(0x7<<8)
#define S5P_CLKDIV0_PCLKD0_MASK	(0x7<<12)
#define S5P_CLKDIV0_SECSS_MASK	(0x7<<16)

#define S5P_CLKDIV1_AM_MASK	(0x7<<0)
#define S5P_CLKDIV1_MPLL_MASK	(0x3<<4)
#define S5P_CLKDIV1_MPLL2_MASK	(0x1<<8)
#define S5P_CLKDIV1_D1_MASK	(0x7<<12)
#define S5P_CLKDIV1_PCLKD1_MASK	(0x7<<16)
#define S5P_CLKDIV1_ONENAND_MASK	(0x3<<20)
#define S5P_CLKDIV1_CAM_MASK	(0x1F<<24)

static inline unsigned int
s5p_get_apll(unsigned long  pllval, unsigned long baseclk)
{
	unsigned long  mdiv, pdiv, sdiv;

	baseclk /= 1000;

	mdiv = (pllval & (0x3ff << 16))>>16;
	pdiv = (pllval & (0x3f << 8))>>8;
	sdiv = (pllval & (0x03 << 0))>>0;

	return (baseclk * (mdiv)) / ((pdiv) << sdiv)*1000;
}

static inline unsigned int
s5p_get_mpll(unsigned long  pllval, unsigned long baseclk)
{
	unsigned long  mdiv, pdiv, sdiv;

	baseclk /= 1000;

	mdiv = (pllval & (0xff << 16))>>16;
	pdiv = (pllval & (0x3f << 8))>>8;
	sdiv = (pllval & (0x03 << 0))>>0;

	return (baseclk * (mdiv)) / ((pdiv) << sdiv)*1000;
}
#if 0
static inline unsigned int
s5p_get_mpll(unsigned long baseclk)
{
	unsigned long  pllval, mdiv, pdiv, sdiv, kdiv;

	baseclk /= 1000;

	pllval = readl(S5P_EPLL_CON0);
	mdiv = (pllval & (0x3ff << 16))>>16;
	pdiv = (pllval & (0x3f << 8))>>8;
	sdiv = (pllval & (0x03 << 0))>>0;
	kdiv = readl(S5P_EPLL_CON1) & (0xffff);

	return (baseclk * (mdiv+kdiv/65536) / (pdiv << sdiv))*1000;
}
#endif

#endif /* __ASM_ARM_REGS_CLOCK */
