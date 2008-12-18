/* linux/include/asm-arm/arch-s3c2410/regs-gpio.h
 *
 * "$Id: regs-gpio.h,v 1.8 2008/07/30 11:03:03 dasan Exp $"
 *
 * Copyright (c) 2003,2004 Simtec Electronics <linux@simtec.co.uk>
 *		           http://www.simtec.co.uk/products/SWLINUX/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * S3C2410 GPIO register definitions
*/


#ifndef __ASM_ARCH_REGS_GPIO_H
#define __ASM_ARCH_REGS_GPIO_H
#define S3C2410_GPIONO(bank,offset) ((bank) + (offset))

#define S3C2410_GPIO_BANKA   (32*0)
#define S3C2410_GPIO_BANKB   (32*1)
#define S3C2410_GPIO_BANKC   (32*2)
#define S3C2410_GPIO_BANKD   (32*3)
#define S3C2410_GPIO_BANKE   (32*4)
#define S3C2410_GPIO_BANKF   (32*5)
#define S3C2410_GPIO_BANKG   (32*6)
#define S3C2410_GPIO_BANKH   (32*7)

#define S3C2410_GPIO_BANKJ   (32*13)
#define S3C2410_GPIO_BANKL   (32*15)

#ifdef CONFIG_CPU_S3C2400
#define S3C24XX_GPIO_BASE(x)  S3C2400_GPIO_BASE(x)
#define S3C24XX_MISCCR        S3C2400_MISCCR
#else
#define S3C24XX_GPIO_BASE(x)  S3C2410_GPIO_BASE(x)
#define S3C24XX_MISCCR	      S3C24XX_GPIOREG2(0x80)
#define S3C24XX_MISCCR_SPISEL (1<<31)	/* SET SPI on S3C2443 */
#endif /* CONFIG_CPU_S3C2400 */


/* S3C2400 doesn't have a 1:1 mapping to S3C2410 gpio base pins */

#define S3C2400_BANKNUM(pin)     (((pin) & ~31) / 32)
#define S3C2400_BASEA2B(pin)     ((((pin) & ~31) >> 2))
#define S3C2400_BASEC2H(pin)     ((S3C2400_BANKNUM(pin) * 10) + \
                                 (2 * (S3C2400_BANKNUM(pin)-2)))

#define S3C2400_GPIO_BASE(pin)   (pin < S3C2410_GPIO_BANKC ? \
                                 S3C2400_BASEA2B(pin)+S3C24XX_VA_GPIO : \
                                 S3C2400_BASEC2H(pin)+S3C24XX_VA_GPIO)


#define S3C2410_GPIO_BASE(pin)   ((((pin) & ~31) >> 1) + S3C24XX_VA_GPIO)
#define S3C2410_GPIO_OFFSET(pin) ((pin) & 31)

/* general configuration options */

#define S3C2410_GPIO_LEAVE   (0xFFFFFFFF)
#define S3C2410_GPIO_INPUT   (0xFFFFFFF0)	/* not available on A */
#define S3C2410_GPIO_OUTPUT  (0xFFFFFFF1)
#define S3C2410_GPIO_IRQ     (0xFFFFFFF2)	/* not available for all */
#define S3C2410_GPIO_SFN2    (0xFFFFFFF2)	/* bank A => addr/cs/nand */
#define S3C2410_GPIO_SFN3    (0xFFFFFFF3)	/* not available on A */

/* register address for the GPIO registers.
 * S3C24XX_GPIOREG2 is for the second set of registers in the
 * GPIO which move between s3c2410 and s3c2412 type systems */

#define S3C2410_GPIOREG(x) ((x) + S3C24XX_VA_GPIO)
#define S3C24XX_GPIOREG2(x) ((x) + S3C24XX_VA_GPIO2)


/* configure GPIO ports A..G */

/* port A - S3C2410: 22bits, zero in bit X makes pin X output
 *          S3C2400: 18bits, zero in bit X makes pin X output
 * 1 makes port special function, this is default
*/
#define S3C2410_GPACON	   S3C2410_GPIOREG(0x00)
#define S3C2410_GPADAT	   S3C2410_GPIOREG(0x04)

#define S3C2400_GPACON	   S3C2410_GPIOREG(0x00)
#define S3C2400_GPADAT	   S3C2410_GPIOREG(0x04)

#define S3C2410_GPA0         S3C2410_GPIONO(S3C2410_GPIO_BANKA, 0)
#define S3C2410_GPA0_OUT     (0<<0)
#define S3C2410_GPA0_ADDR0   (1<<0)

#define S3C2410_GPA1         S3C2410_GPIONO(S3C2410_GPIO_BANKA, 1)
#define S3C2410_GPA1_OUT     (0<<1)
#define S3C2410_GPA1_ADDR16  (1<<1)

#define S3C2410_GPA2         S3C2410_GPIONO(S3C2410_GPIO_BANKA, 2)
#define S3C2410_GPA2_OUT     (0<<2)
#define S3C2410_GPA2_ADDR17  (1<<2)

#define S3C2410_GPA3         S3C2410_GPIONO(S3C2410_GPIO_BANKA, 3)
#define S3C2410_GPA3_OUT     (0<<3)
#define S3C2410_GPA3_ADDR18  (1<<3)

#define S3C2410_GPA4         S3C2410_GPIONO(S3C2410_GPIO_BANKA, 4)
#define S3C2410_GPA4_OUT     (0<<4)
#define S3C2410_GPA4_ADDR19  (1<<4)

#define S3C2410_GPA5         S3C2410_GPIONO(S3C2410_GPIO_BANKA, 5)
#define S3C2410_GPA5_OUT     (0<<5)
#define S3C2410_GPA5_ADDR20  (1<<5)

#define S3C2410_GPA6         S3C2410_GPIONO(S3C2410_GPIO_BANKA, 6)
#define S3C2410_GPA6_OUT     (0<<6)
#define S3C2410_GPA6_ADDR21  (1<<6)

#define S3C2410_GPA7         S3C2410_GPIONO(S3C2410_GPIO_BANKA, 7)
#define S3C2410_GPA7_OUT     (0<<7)
#define S3C2410_GPA7_ADDR22  (1<<7)

#define S3C2410_GPA8         S3C2410_GPIONO(S3C2410_GPIO_BANKA, 8)
#define S3C2410_GPA8_OUT     (0<<8)
#define S3C2410_GPA8_ADDR23  (1<<8)

#define S3C2410_GPA9         S3C2410_GPIONO(S3C2410_GPIO_BANKA, 9)
#define S3C2410_GPA9_OUT     (0<<9)
#define S3C2410_GPA9_ADDR24  (1<<9)

#define S3C2410_GPA10        S3C2410_GPIONO(S3C2410_GPIO_BANKA, 10)
#define S3C2410_GPA10_OUT    (0<<10)
#define S3C2410_GPA10_ADDR25 (1<<10)
#define S3C2400_GPA10_SCKE   (1<<10)

#define S3C2410_GPA11        S3C2410_GPIONO(S3C2410_GPIO_BANKA, 11)
#define S3C2410_GPA11_OUT    (0<<11)
#define S3C2410_GPA11_ADDR26 (1<<11)
#define S3C2400_GPA11_nCAS0  (1<<11)

#define S3C2410_GPA12        S3C2410_GPIONO(S3C2410_GPIO_BANKA, 12)
#define S3C2410_GPA12_OUT    (0<<12)
#define S3C2410_GPA12_nGCS1  (1<<12)
#define S3C2400_GPA12_nCAS1  (1<<12)

#define S3C2410_GPA13        S3C2410_GPIONO(S3C2410_GPIO_BANKA, 13)
#define S3C2410_GPA13_OUT    (0<<13)
#define S3C2410_GPA13_nGCS2  (1<<13)
#define S3C2400_GPA13_nGCS1  (1<<13)

#define S3C2410_GPA14        S3C2410_GPIONO(S3C2410_GPIO_BANKA, 14)
#define S3C2410_GPA14_OUT    (0<<14)
#define S3C2410_GPA14_nGCS3  (1<<14)
#define S3C2400_GPA14_nGCS2  (1<<14)

#define S3C2410_GPA15        S3C2410_GPIONO(S3C2410_GPIO_BANKA, 15)
#define S3C2410_GPA15_OUT    (0<<15)
#define S3C2410_GPA15_nGCS4  (1<<15)
#define S3C2400_GPA15_nGCS3  (1<<15)

#define S3C2410_GPA16        S3C2410_GPIONO(S3C2410_GPIO_BANKA, 16)
#define S3C2410_GPA16_OUT    (0<<16)
#define S3C2410_GPA16_nGCS5  (1<<16)
#define S3C2400_GPA16_nGCS4  (1<<16)

#define S3C2410_GPA17        S3C2410_GPIONO(S3C2410_GPIO_BANKA, 17)
#define S3C2410_GPA17_OUT    (0<<17)
#define S3C2410_GPA17_CLE    (1<<17)
#define S3C2400_GPA17_nGCS5  (1<<17)

#define S3C2410_GPA18        S3C2410_GPIONO(S3C2410_GPIO_BANKA, 18)
#define S3C2410_GPA18_OUT    (0<<18)
#define S3C2410_GPA18_ALE    (1<<18)

#define S3C2410_GPA19        S3C2410_GPIONO(S3C2410_GPIO_BANKA, 19)
#define S3C2410_GPA19_OUT    (0<<19)
#define S3C2410_GPA19_nFWE   (1<<19)

#define S3C2410_GPA20        S3C2410_GPIONO(S3C2410_GPIO_BANKA, 20)
#define S3C2410_GPA20_OUT    (0<<20)
#define S3C2410_GPA20_nFRE   (1<<20)

#define S3C2410_GPA21        S3C2410_GPIONO(S3C2410_GPIO_BANKA, 21)
#define S3C2410_GPA21_OUT    (0<<21)
#define S3C2410_GPA21_nRSTOUT (1<<21)

#define S3C2410_GPA22        S3C2410_GPIONO(S3C2410_GPIO_BANKA, 22)
#define S3C2410_GPA22_OUT    (0<<22)
#define S3C2410_GPA22_nFCE   (1<<22)

/* 0x08 and 0x0c are reserved on S3C2410 */

/* S3C2410:
 * GPB is 10 IO pins, each configured by 2 bits each in GPBCON.
 *   00 = input, 01 = output, 10=special function, 11=reserved

 * S3C2400:
 * GPB is 16 IO pins, each configured by 2 bits each in GPBCON.
 *   00 = input, 01 = output, 10=data, 11=special function

 * bit 0,1 = pin 0, 2,3= pin 1...
 *
 * CPBUP = pull up resistor control, 1=disabled, 0=enabled
*/

#define S3C2410_GPBCON	   S3C2410_GPIOREG(0x10)
#define S3C2410_GPBDAT	   S3C2410_GPIOREG(0x14)
#define S3C2410_GPBUP	   S3C2410_GPIOREG(0x18)

#define S3C2400_GPBCON	   S3C2410_GPIOREG(0x08)
#define S3C2400_GPBDAT	   S3C2410_GPIOREG(0x0C)
#define S3C2400_GPBUP	   S3C2410_GPIOREG(0x10)

#define S3C2450_GPBSEL	   S3C2410_GPIOREG(0x1C)

/* no i/o pin in port b can have value 3 (unless it is a s3c2443) ! */

#define S3C2410_GPB0         S3C2410_GPIONO(S3C2410_GPIO_BANKB, 0)
#define S3C2410_GPB0_INP     (0x00 << 0)
#define S3C2410_GPB0_OUTP    (0x01 << 0)
#define S3C2410_GPB0_TOUT0   (0x02 << 0)
#define S3C2400_GPB0_DATA16  (0x02 << 0)

#define S3C2410_GPB1         S3C2410_GPIONO(S3C2410_GPIO_BANKB, 1)
#define S3C2410_GPB1_INP     (0x00 << 2)
#define S3C2410_GPB1_OUTP    (0x01 << 2)
#define S3C2410_GPB1_TOUT1   (0x02 << 2)
#define S3C2400_GPB1_DATA17  (0x02 << 2)

#define S3C2410_GPB2         S3C2410_GPIONO(S3C2410_GPIO_BANKB, 2)
#define S3C2410_GPB2_INP     (0x00 << 4)
#define S3C2410_GPB2_OUTP    (0x01 << 4)
#define S3C2410_GPB2_TOUT2   (0x02 << 4)
#define S3C2400_GPB2_DATA18  (0x02 << 4)
#define S3C2400_GPB2_TCLK1   (0x03 << 4)

#define S3C2410_GPB3         S3C2410_GPIONO(S3C2410_GPIO_BANKB, 3)
#define S3C2410_GPB3_INP     (0x00 << 6)
#define S3C2410_GPB3_OUTP    (0x01 << 6)
#define S3C2410_GPB3_TOUT3   (0x02 << 6)
#define S3C2400_GPB3_DATA19  (0x02 << 6)
#define S3C2400_GPB3_TXD1    (0x03 << 6)

#define S3C2410_GPB4         S3C2410_GPIONO(S3C2410_GPIO_BANKB, 4)
#define S3C2410_GPB4_INP     (0x00 << 8)
#define S3C2410_GPB4_OUTP    (0x01 << 8)
#define S3C2410_GPB4_TCLK0   (0x02 << 8)
#define S3C2400_GPB4_DATA20  (0x02 << 8)
#define S3C2410_GPB4_MASK    (0x03 << 8)
#define S3C2400_GPB4_RXD1    (0x03 << 8)
#define S3C2400_GPB4_MASK    (0x03 << 8)

#define S3C2410_GPB5         S3C2410_GPIONO(S3C2410_GPIO_BANKB, 5)
#define S3C2410_GPB5_INP     (0x00 << 10)
#define S3C2410_GPB5_OUTP    (0x01 << 10)
#define S3C2410_GPB5_nXBACK  (0x02 << 10)
#define S3C2443_GPB5_XBACK   (0x03 << 10)
#define S3C2400_GPB5_DATA21  (0x02 << 10)
#define S3C2400_GPB5_nCTS1   (0x03 << 10)

#define S3C2410_GPB6         S3C2410_GPIONO(S3C2410_GPIO_BANKB, 6)
#define S3C2410_GPB6_INP     (0x00 << 12)
#define S3C2410_GPB6_OUTP    (0x01 << 12)
#define S3C2410_GPB6_nXBREQ  (0x02 << 12)
#define S3C2443_GPB6_XBREQ   (0x03 << 12)
#define S3C2400_GPB6_DATA22  (0x02 << 12)
#define S3C2400_GPB6_nRTS1   (0x03 << 12)

#define S3C2410_GPB7         S3C2410_GPIONO(S3C2410_GPIO_BANKB, 7)
#define S3C2410_GPB7_INP     (0x00 << 14)
#define S3C2410_GPB7_OUTP    (0x01 << 14)
#define S3C2410_GPB7_nXDACK1 (0x02 << 14)
#define S3C2443_GPB7_XDACK1  (0x03 << 14)
#define S3C2400_GPB7_DATA23  (0x02 << 14)

#define S3C2410_GPB8         S3C2410_GPIONO(S3C2410_GPIO_BANKB, 8)
#define S3C2410_GPB8_INP     (0x00 << 16)
#define S3C2410_GPB8_OUTP    (0x01 << 16)
#define S3C2410_GPB8_nXDREQ1 (0x02 << 16)
#define S3C2400_GPB8_DATA24  (0x02 << 16)

#define S3C2410_GPB9         S3C2410_GPIONO(S3C2410_GPIO_BANKB, 9)
#define S3C2410_GPB9_INP     (0x00 << 18)
#define S3C2410_GPB9_OUTP    (0x01 << 18)
#define S3C2410_GPB9_nXDACK0 (0x02 << 18)
#define S3C2443_GPB9_XDACK0  (0x03 << 18)
#define S3C2400_GPB9_DATA25  (0x02 << 18)
#define S3C2400_GPB9_I2SSDI  (0x03 << 18)

#define S3C2410_GPB10        S3C2410_GPIONO(S3C2410_GPIO_BANKB, 10)
#define S3C2410_GPB10_INP    (0x00 << 20)
#define S3C2410_GPB10_OUTP   (0x01 << 20)
#define S3C2410_GPB10_nXDRE0 (0x02 << 20)
#define S3C2443_GPB10_XDREQ0 (0x03 << 20)
#define S3C2400_GPB10_DATA26 (0x02 << 20)
#define S3C2400_GPB10_nSS    (0x03 << 20)

#define S3C2400_GPB11        S3C2410_GPIONO(S3C2410_GPIO_BANKB, 11)
#define S3C2400_GPB11_INP    (0x00 << 22)
#define S3C2400_GPB11_OUTP   (0x01 << 22)
#define S3C2400_GPB11_DATA27 (0x02 << 22)

#define S3C2400_GPB12        S3C2410_GPIONO(S3C2410_GPIO_BANKB, 12)
#define S3C2400_GPB12_INP    (0x00 << 24)
#define S3C2400_GPB12_OUTP   (0x01 << 24)
#define S3C2400_GPB12_DATA28 (0x02 << 24)

#define S3C2400_GPB13        S3C2410_GPIONO(S3C2410_GPIO_BANKB, 13)
#define S3C2400_GPB13_INP    (0x00 << 26)
#define S3C2400_GPB13_OUTP   (0x01 << 26)
#define S3C2400_GPB13_DATA29 (0x02 << 26)

#define S3C2400_GPB14        S3C2410_GPIONO(S3C2410_GPIO_BANKB, 14)
#define S3C2400_GPB14_INP    (0x00 << 28)
#define S3C2400_GPB14_OUTP   (0x01 << 28)
#define S3C2400_GPB14_DATA30 (0x02 << 28)

#define S3C2400_GPB15        S3C2410_GPIONO(S3C2410_GPIO_BANKB, 15)
#define S3C2400_GPB15_INP    (0x00 << 30)
#define S3C2400_GPB15_OUTP   (0x01 << 30)
#define S3C2400_GPB15_DATA31 (0x02 << 30)

#define S3C2410_GPB_PUPDIS(x)  (1<<(x))

/* Port C consits of 16 GPIO/Special function
 *
 * almost identical setup to port b, but the special functions are mostly
 * to do with the video system's sync/etc.
*/

#define S3C2410_GPCCON	   S3C2410_GPIOREG(0x20)
#define S3C2410_GPCDAT	   S3C2410_GPIOREG(0x24)
#define S3C2410_GPCUP	   S3C2410_GPIOREG(0x28)

#define S3C2400_GPCCON	   S3C2410_GPIOREG(0x14)
#define S3C2400_GPCDAT	   S3C2410_GPIOREG(0x18)
#define S3C2400_GPCUP	   S3C2410_GPIOREG(0x1C)

#define S3C2410_GPC0            S3C2410_GPIONO(S3C2410_GPIO_BANKC, 0)
#define S3C2410_GPC0_INP	(0x00 << 0)
#define S3C2410_GPC0_OUTP	(0x01 << 0)
#define S3C2410_GPC0_LEND	(0x02 << 0)
#define S3C2400_GPC0_VD0 	(0x02 << 0)

#define S3C2410_GPC1            S3C2410_GPIONO(S3C2410_GPIO_BANKC, 1)
#define S3C2410_GPC1_INP	(0x00 << 2)
#define S3C2410_GPC1_OUTP	(0x01 << 2)
#define S3C2410_GPC1_VCLK	(0x02 << 2)
#define S3C2400_GPC1_VD1 	(0x02 << 2)

#define S3C2410_GPC2            S3C2410_GPIONO(S3C2410_GPIO_BANKC, 2)
#define S3C2410_GPC2_INP	(0x00 << 4)
#define S3C2410_GPC2_OUTP	(0x01 << 4)
#define S3C2410_GPC2_VLINE	(0x02 << 4)
#define S3C2400_GPC2_VD2  	(0x02 << 4)

#define S3C2410_GPC3            S3C2410_GPIONO(S3C2410_GPIO_BANKC, 3)
#define S3C2410_GPC3_INP	(0x00 << 6)
#define S3C2410_GPC3_OUTP	(0x01 << 6)
#define S3C2410_GPC3_VFRAME	(0x02 << 6)
#define S3C2400_GPC3_VD3   	(0x02 << 6)

#define S3C2410_GPC4            S3C2410_GPIONO(S3C2410_GPIO_BANKC, 4)
#define S3C2410_GPC4_INP	(0x00 << 8)
#define S3C2410_GPC4_OUTP	(0x01 << 8)
#define S3C2410_GPC4_VM		(0x02 << 8)
#define S3C2400_GPC4_VD4	(0x02 << 8)

#define S3C2410_GPC5            S3C2410_GPIONO(S3C2410_GPIO_BANKC, 5)
#define S3C2410_GPC5_INP	(0x00 << 10)
#define S3C2410_GPC5_OUTP	(0x01 << 10)
#define S3C2410_GPC5_LCDVF0	(0x02 << 10)
#define S3C2400_GPC5_VD5   	(0x02 << 10)

#define S3C2410_GPC6            S3C2410_GPIONO(S3C2410_GPIO_BANKC, 6)
#define S3C2410_GPC6_INP	(0x00 << 12)
#define S3C2410_GPC6_OUTP	(0x01 << 12)
#define S3C2410_GPC6_LCDVF1	(0x02 << 12)
#define S3C2400_GPC6_VD6   	(0x02 << 12)

#define S3C2410_GPC7            S3C2410_GPIONO(S3C2410_GPIO_BANKC, 7)
#define S3C2410_GPC7_INP	(0x00 << 14)
#define S3C2410_GPC7_OUTP	(0x01 << 14)
#define S3C2410_GPC7_LCDVF2	(0x02 << 14)
#define S3C2400_GPC7_VD7   	(0x02 << 14)

#define S3C2410_GPC8            S3C2410_GPIONO(S3C2410_GPIO_BANKC, 8)
#define S3C2410_GPC8_INP	(0x00 << 16)
#define S3C2410_GPC8_OUTP	(0x01 << 16)
#define S3C2410_GPC8_VD0	(0x02 << 16)
#define S3C2400_GPC8_VD8	(0x02 << 16)

#define S3C2410_GPC9            S3C2410_GPIONO(S3C2410_GPIO_BANKC, 9)
#define S3C2410_GPC9_INP	(0x00 << 18)
#define S3C2410_GPC9_OUTP	(0x01 << 18)
#define S3C2410_GPC9_VD1	(0x02 << 18)
#define S3C2400_GPC9_VD9	(0x02 << 18)

#define S3C2410_GPC10           S3C2410_GPIONO(S3C2410_GPIO_BANKC, 10)
#define S3C2410_GPC10_INP	(0x00 << 20)
#define S3C2410_GPC10_OUTP	(0x01 << 20)
#define S3C2410_GPC10_VD2	(0x02 << 20)
#define S3C2400_GPC10_VD10	(0x02 << 20)

#define S3C2410_GPC11           S3C2410_GPIONO(S3C2410_GPIO_BANKC, 11)
#define S3C2410_GPC11_INP	(0x00 << 22)
#define S3C2410_GPC11_OUTP	(0x01 << 22)
#define S3C2410_GPC11_VD3	(0x02 << 22)
#define S3C2400_GPC11_VD11	(0x02 << 22)

#define S3C2410_GPC12           S3C2410_GPIONO(S3C2410_GPIO_BANKC, 12)
#define S3C2410_GPC12_INP	(0x00 << 24)
#define S3C2410_GPC12_OUTP	(0x01 << 24)
#define S3C2410_GPC12_VD4	(0x02 << 24)
#define S3C2400_GPC12_VD12	(0x02 << 24)

#define S3C2410_GPC13           S3C2410_GPIONO(S3C2410_GPIO_BANKC, 13)
#define S3C2410_GPC13_INP	(0x00 << 26)
#define S3C2410_GPC13_OUTP	(0x01 << 26)
#define S3C2410_GPC13_VD5	(0x02 << 26)
#define S3C2400_GPC13_VD13	(0x02 << 26)

#define S3C2410_GPC14           S3C2410_GPIONO(S3C2410_GPIO_BANKC, 14)
#define S3C2410_GPC14_INP	(0x00 << 28)
#define S3C2410_GPC14_OUTP	(0x01 << 28)
#define S3C2410_GPC14_VD6	(0x02 << 28)
#define S3C2400_GPC14_VD14	(0x02 << 28)

#define S3C2410_GPC15           S3C2410_GPIONO(S3C2410_GPIO_BANKC, 15)
#define S3C2410_GPC15_INP	(0x00 << 30)
#define S3C2410_GPC15_OUTP	(0x01 << 30)
#define S3C2410_GPC15_VD7	(0x02 << 30)
#define S3C2400_GPC15_VD15	(0x02 << 30)

#define S3C2410_GPC_PUPDIS(x)  (1<<(x))

/*
 * S3C2410: Port D consists of 16 GPIO/Special function
 *
 * almost identical setup to port b, but the special functions are mostly
 * to do with the video system's data.
 *
 * S3C2400: Port D consists of 11 GPIO/Special function
 *
 * almost identical setup to port c
*/

#define S3C2410_GPDCON	   S3C2410_GPIOREG(0x30)
#define S3C2410_GPDDAT	   S3C2410_GPIOREG(0x34)
#define S3C2410_GPDUP	   S3C2410_GPIOREG(0x38)

#define S3C2400_GPDCON	   S3C2410_GPIOREG(0x20)
#define S3C2400_GPDDAT	   S3C2410_GPIOREG(0x24)
#define S3C2400_GPDUP	   S3C2410_GPIOREG(0x28)

#define S3C2410_GPD0            S3C2410_GPIONO(S3C2410_GPIO_BANKD, 0)
#define S3C2410_GPD0_INP	(0x00 << 0)
#define S3C2410_GPD0_OUTP	(0x01 << 0)
#define S3C2410_GPD0_VD8	(0x02 << 0)
#define S3C2400_GPD0_VFRAME	(0x02 << 0)
#define S3C2442_GPD0_nSPICS1	(0x03 << 0)

#define S3C2410_GPD1            S3C2410_GPIONO(S3C2410_GPIO_BANKD, 1)
#define S3C2410_GPD1_INP	(0x00 << 2)
#define S3C2410_GPD1_OUTP	(0x01 << 2)
#define S3C2410_GPD1_VD9	(0x02 << 2)
#define S3C2400_GPD1_VM		(0x02 << 2)
#define S3C2442_GPD1_SPICLK1	(0x03 << 2)

#define S3C2410_GPD2            S3C2410_GPIONO(S3C2410_GPIO_BANKD, 2)
#define S3C2410_GPD2_INP	(0x00 << 4)
#define S3C2410_GPD2_OUTP	(0x01 << 4)
#define S3C2410_GPD2_VD10	(0x02 << 4)
#define S3C2400_GPD2_VLINE	(0x02 << 4)

#define S3C2410_GPD3            S3C2410_GPIONO(S3C2410_GPIO_BANKD, 3)
#define S3C2410_GPD3_INP	(0x00 << 6)
#define S3C2410_GPD3_OUTP	(0x01 << 6)
#define S3C2410_GPD3_VD11	(0x02 << 6)
#define S3C2400_GPD3_VCLK	(0x02 << 6)

#define S3C2410_GPD4            S3C2410_GPIONO(S3C2410_GPIO_BANKD, 4)
#define S3C2410_GPD4_INP	(0x00 << 8)
#define S3C2410_GPD4_OUTP	(0x01 << 8)
#define S3C2410_GPD4_VD12	(0x02 << 8)
#define S3C2400_GPD4_LEND	(0x02 << 8)

#define S3C2410_GPD5            S3C2410_GPIONO(S3C2410_GPIO_BANKD, 5)
#define S3C2410_GPD5_INP	(0x00 << 10)
#define S3C2410_GPD5_OUTP	(0x01 << 10)
#define S3C2410_GPD5_VD13	(0x02 << 10)
#define S3C2400_GPD5_TOUT0	(0x02 << 10)

#define S3C2410_GPD6            S3C2410_GPIONO(S3C2410_GPIO_BANKD, 6)
#define S3C2410_GPD6_INP	(0x00 << 12)
#define S3C2410_GPD6_OUTP	(0x01 << 12)
#define S3C2410_GPD6_VD14	(0x02 << 12)
#define S3C2400_GPD6_TOUT1	(0x02 << 12)

#define S3C2410_GPD7            S3C2410_GPIONO(S3C2410_GPIO_BANKD, 7)
#define S3C2410_GPD7_INP	(0x00 << 14)
#define S3C2410_GPD7_OUTP	(0x01 << 14)
#define S3C2410_GPD7_VD15	(0x02 << 14)
#define S3C2400_GPD7_TOUT2	(0x02 << 14)

#define S3C2410_GPD8            S3C2410_GPIONO(S3C2410_GPIO_BANKD, 8)
#define S3C2410_GPD8_INP	(0x00 << 16)
#define S3C2410_GPD8_OUTP	(0x01 << 16)
#define S3C2410_GPD8_VD16	(0x02 << 16)
#define S3C2400_GPD8_TOUT3	(0x02 << 16)

#define S3C2410_GPD9            S3C2410_GPIONO(S3C2410_GPIO_BANKD, 9)
#define S3C2410_GPD9_INP	(0x00 << 18)
#define S3C2410_GPD9_OUTP	(0x01 << 18)
#define S3C2410_GPD9_VD17	(0x02 << 18)
#define S3C2400_GPD9_TCLK0	(0x02 << 18)
#define S3C2410_GPD9_MASK       (0x03 << 18)

#define S3C2410_GPD10           S3C2410_GPIONO(S3C2410_GPIO_BANKD, 10)
#define S3C2410_GPD10_INP	(0x00 << 20)
#define S3C2410_GPD10_OUTP	(0x01 << 20)
#define S3C2410_GPD10_VD18	(0x02 << 20)
#define S3C2400_GPD10_nWAIT	(0x02 << 20)

#define S3C2410_GPD11           S3C2410_GPIONO(S3C2410_GPIO_BANKD, 11)
#define S3C2410_GPD11_INP	(0x00 << 22)
#define S3C2410_GPD11_OUTP	(0x01 << 22)
#define S3C2410_GPD11_VD19	(0x02 << 22)

#define S3C2410_GPD12           S3C2410_GPIONO(S3C2410_GPIO_BANKD, 12)
#define S3C2410_GPD12_INP	(0x00 << 24)
#define S3C2410_GPD12_OUTP	(0x01 << 24)
#define S3C2410_GPD12_VD20	(0x02 << 24)

#define S3C2410_GPD13           S3C2410_GPIONO(S3C2410_GPIO_BANKD, 13)
#define S3C2410_GPD13_INP	(0x00 << 26)
#define S3C2410_GPD13_OUTP	(0x01 << 26)
#define S3C2410_GPD13_VD21	(0x02 << 26)

#define S3C2410_GPD14           S3C2410_GPIONO(S3C2410_GPIO_BANKD, 14)
#define S3C2410_GPD14_INP	(0x00 << 28)
#define S3C2410_GPD14_OUTP	(0x01 << 28)
#define S3C2410_GPD14_VD22	(0x02 << 28)

#define S3C2410_GPD15           S3C2410_GPIONO(S3C2410_GPIO_BANKD, 15)
#define S3C2410_GPD15_INP	(0x00 << 30)
#define S3C2410_GPD15_OUTP	(0x01 << 30)
#define S3C2410_GPD15_VD23	(0x02 << 30)

#define S3C2410_GPD_PUPDIS(x)  (1<<(x))

/* S3C2410:
 * Port E consists of 16 GPIO/Special function
 *
 * again, the same as port B, but dealing with I2S, SDI, and
 * more miscellaneous functions
 *
 * S3C2400:
 * Port E consists of 12 GPIO/Special function
 *
 * GPIO / interrupt inputs
*/

#define S3C2410_GPECON	   S3C2410_GPIOREG(0x40)
#define S3C2410_GPEDAT	   S3C2410_GPIOREG(0x44)
#define S3C2410_GPEUP	   S3C2410_GPIOREG(0x48)

#define S3C2400_GPECON	   S3C2410_GPIOREG(0x2C)
#define S3C2400_GPEDAT	   S3C2410_GPIOREG(0x30)
#define S3C2400_GPEUP	   S3C2410_GPIOREG(0x34)

#define S3C2450_GPESEL	   S3C2410_GPIOREG(0x4C)

#define S3C2410_GPE0           S3C2410_GPIONO(S3C2410_GPIO_BANKE, 0)
#define S3C2410_GPE0_INP       (0x00 << 0)
#define S3C2410_GPE0_OUTP      (0x01 << 0)
#define S3C2410_GPE0_I2SLRCK   (0x02 << 0)
#define S3C2443_GPE0_AC_nRESET (0x03 << 0)
#define S3C2400_GPE0_EINT0     (0x02 << 0)
#define S3C2410_GPE0_MASK      (0x03 << 0)

#define S3C2410_GPE1           S3C2410_GPIONO(S3C2410_GPIO_BANKE, 1)
#define S3C2410_GPE1_INP       (0x00 << 2)
#define S3C2410_GPE1_OUTP      (0x01 << 2)
#define S3C2410_GPE1_I2SSCLK   (0x02 << 2)
#define S3C2443_GPE1_AC_SYNC   (0x03 << 2)
#define S3C2400_GPE1_EINT1     (0x02 << 2)
#define S3C2400_GPE1_nSS       (0x03 << 2)
#define S3C2410_GPE1_MASK      (0x03 << 2)

#define S3C2410_GPE2           S3C2410_GPIONO(S3C2410_GPIO_BANKE, 2)
#define S3C2410_GPE2_INP       (0x00 << 4)
#define S3C2410_GPE2_OUTP      (0x01 << 4)
#define S3C2410_GPE2_CDCLK     (0x02 << 4)
#define S3C2443_GPE2_AC_BITCLK (0x03 << 4)
#define S3C2400_GPE2_EINT2     (0x02 << 4)
#define S3C2400_GPE2_I2SSDI    (0x03 << 4)

#define S3C2410_GPE3           S3C2410_GPIONO(S3C2410_GPIO_BANKE, 3)
#define S3C2410_GPE3_INP       (0x00 << 6)
#define S3C2410_GPE3_OUTP      (0x01 << 6)
#define S3C2410_GPE3_I2SSDI    (0x02 << 6)
#define S3C2443_GPE3_AC_SDI    (0x03 << 6)
#define S3C2400_GPE3_EINT3     (0x02 << 6)
#define S3C2400_GPE3_nCTS1     (0x03 << 6)
#define S3C2410_GPE3_nSS0      (0x03 << 6)
#define S3C2410_GPE3_MASK      (0x03 << 6)

#define S3C2410_GPE4           S3C2410_GPIONO(S3C2410_GPIO_BANKE, 4)
#define S3C2410_GPE4_INP       (0x00 << 8)
#define S3C2410_GPE4_OUTP      (0x01 << 8)
#define S3C2410_GPE4_I2SSDO    (0x02 << 8)
#define S3C2443_GPE4_AC_SDO    (0x03 << 8)
#define S3C2400_GPE4_EINT4     (0x02 << 8)
#define S3C2400_GPE4_nRTS1     (0x03 << 8)
#define S3C2410_GPE4_I2SSDI    (0x03 << 8)
#define S3C2410_GPE4_MASK      (0x03 << 8)

#define S3C2410_GPE5           S3C2410_GPIONO(S3C2410_GPIO_BANKE, 5)
#define S3C2410_GPE5_INP       (0x00 << 10)
#define S3C2410_GPE5_OUTP      (0x01 << 10)
#define S3C2410_GPE5_SDCLK     (0x02 << 10)
#define S3C2443_GPE5_SD1_CLK   (0x02 << 10)
#define S3C2450_GPE5_SD0_CLK   (0x02 << 10)
#define S3C2400_GPE5_EINT5     (0x02 << 10)
#define S3C2400_GPE5_TCLK1     (0x03 << 10)

#define S3C2410_GPE6           S3C2410_GPIONO(S3C2410_GPIO_BANKE, 6)
#define S3C2410_GPE6_INP       (0x00 << 12)
#define S3C2410_GPE6_OUTP      (0x01 << 12)
#define S3C2410_GPE6_SDCMD     (0x02 << 12)
#define S3C2443_GPE6_SD1_CMD   (0x02 << 12)
#define S3C2450_GPE6_SD0_CMD   (0x02 << 12)
#define S3C2443_GPE6_AC_BITCLK (0x03 << 12)
#define S3C2400_GPE6_EINT6     (0x02 << 12)

#define S3C2410_GPE7           S3C2410_GPIONO(S3C2410_GPIO_BANKE, 7)
#define S3C2410_GPE7_INP       (0x00 << 14)
#define S3C2410_GPE7_OUTP      (0x01 << 14)
#define S3C2410_GPE7_SDDAT0    (0x02 << 14)
#define S3C2443_GPE7_SD1_DAT0  (0x02 << 14)
#define S3C2450_GPE7_SD0_DAT0  (0x02 << 14)
#define S3C2443_GPE7_AC_SDI    (0x03 << 14)
#define S3C2400_GPE7_EINT7     (0x02 << 14)

#define S3C2410_GPE8           S3C2410_GPIONO(S3C2410_GPIO_BANKE, 8)
#define S3C2410_GPE8_INP       (0x00 << 16)
#define S3C2410_GPE8_OUTP      (0x01 << 16)
#define S3C2410_GPE8_SDDAT1    (0x02 << 16)
#define S3C2443_GPE8_SD1_DAT1  (0x02 << 16)
#define S3C2450_GPE8_SD0_DAT1  (0x02 << 16)
#define S3C2443_GPE8_AC_SDO    (0x03 << 16)
#define S3C2400_GPE8_nXDACK0   (0x02 << 16)

#define S3C2410_GPE9           S3C2410_GPIONO(S3C2410_GPIO_BANKE, 9)
#define S3C2410_GPE9_INP       (0x00 << 18)
#define S3C2410_GPE9_OUTP      (0x01 << 18)
#define S3C2410_GPE9_SDDAT2    (0x02 << 18)
#define S3C2443_GPE9_SD1_DAT2  (0x02 << 18)
#define S3C2450_GPE9_SD0_DAT2  (0x02 << 18)
#define S3C2443_GPE9_AC_SYNC   (0x03 << 18)
#define S3C2400_GPE9_nXDACK1   (0x02 << 18)
#define S3C2400_GPE9_nXBACK    (0x03 << 18)

#define S3C2410_GPE10          S3C2410_GPIONO(S3C2410_GPIO_BANKE, 10)
#define S3C2410_GPE10_INP      (0x00 << 20)
#define S3C2410_GPE10_OUTP     (0x01 << 20)
#define S3C2410_GPE10_SDDAT3   (0x02 << 20)
#define S3C2443_GPE10_SD1_DAT3 (0x02 << 20)
#define S3C2450_GPE10_SD0_DAT3 (0x02 << 20)
#define S3C2443_GPE10_AC_nRESET (0x03 << 20)
#define S3C2400_GPE10_nXDREQ0  (0x02 << 20)

#define S3C2410_GPE11          S3C2410_GPIONO(S3C2410_GPIO_BANKE, 11)
#define S3C2410_GPE11_INP      (0x00 << 22)
#define S3C2410_GPE11_OUTP     (0x01 << 22)
#define S3C2410_GPE11_SPIMISO0 (0x02 << 22)
#define S3C2400_GPE11_nXDREQ1  (0x02 << 22)
#define S3C2400_GPE11_nXBREQ   (0x03 << 22)

#define S3C2410_GPE12          S3C2410_GPIONO(S3C2410_GPIO_BANKE, 12)
#define S3C2410_GPE12_INP      (0x00 << 24)
#define S3C2410_GPE12_OUTP     (0x01 << 24)
#define S3C2410_GPE12_SPIMOSI0 (0x02 << 24)

#define S3C2410_GPE13          S3C2410_GPIONO(S3C2410_GPIO_BANKE, 13)
#define S3C2410_GPE13_INP      (0x00 << 26)
#define S3C2410_GPE13_OUTP     (0x01 << 26)
#define S3C2410_GPE13_SPICLK0  (0x02 << 26)

#define S3C2410_GPE14          S3C2410_GPIONO(S3C2410_GPIO_BANKE, 14)
#define S3C2410_GPE14_INP      (0x00 << 28)
#define S3C2410_GPE14_OUTP     (0x01 << 28)
#define S3C2410_GPE14_IICSCL   (0x02 << 28)
#define S3C2410_GPE14_MASK     (0x03 << 28)

#define S3C2410_GPE15          S3C2410_GPIONO(S3C2410_GPIO_BANKE, 15)
#define S3C2410_GPE15_INP      (0x00 << 30)
#define S3C2410_GPE15_OUTP     (0x01 << 30)
#define S3C2410_GPE15_IICSDA   (0x02 << 30)
#define S3C2410_GPE15_MASK     (0x03 << 30)

#define S3C2440_GPE0_ACSYNC    (0x03 << 0)
#define S3C2440_GPE1_ACBITCLK  (0x03 << 2)
#define S3C2440_GPE2_ACRESET   (0x03 << 4)
#define S3C2440_GPE3_ACIN      (0x03 << 6)
#define S3C2440_GPE4_ACOUT     (0x03 << 8)

#define S3C2410_GPE_PUPDIS(x)  (1<<(x))

/* S3C2410:
 * Port F consists of 8 GPIO/Special function
 *
 * GPIO / interrupt inputs
 *
 * GPFCON has 2 bits for each of the input pins on port F
 *   00 = 0 input, 1 output, 2 interrupt (EINT0..7), 3 undefined
 *
 * pull up works like all other ports.
 *
 * S3C2400:
 * Port F consists of 7 GPIO/Special function
 *
 * GPIO/serial/misc pins
*/

#define S3C2410_GPFCON	   S3C2410_GPIOREG(0x50)
#define S3C2410_GPFDAT	   S3C2410_GPIOREG(0x54)
#define S3C2410_GPFUP	   S3C2410_GPIOREG(0x58)

#define S3C2400_GPFCON	   S3C2410_GPIOREG(0x38)
#define S3C2400_GPFDAT	   S3C2410_GPIOREG(0x3C)
#define S3C2400_GPFUP	   S3C2410_GPIOREG(0x40)

#define S3C2410_GPF0        S3C2410_GPIONO(S3C2410_GPIO_BANKF, 0)
#define S3C2410_GPF0_INP    (0x00 << 0)
#define S3C2410_GPF0_OUTP   (0x01 << 0)
#define S3C2410_GPF0_EINT0  (0x02 << 0)
#define S3C2400_GPF0_RXD0   (0x02 << 0)

#define S3C2410_GPF1        S3C2410_GPIONO(S3C2410_GPIO_BANKF, 1)
#define S3C2410_GPF1_INP    (0x00 << 2)
#define S3C2410_GPF1_OUTP   (0x01 << 2)
#define S3C2410_GPF1_EINT1  (0x02 << 2)
#define S3C2400_GPF1_RXD1   (0x02 << 2)
#define S3C2400_GPF1_IICSDA (0x03 << 2)

#define S3C2410_GPF2        S3C2410_GPIONO(S3C2410_GPIO_BANKF, 2)
#define S3C2410_GPF2_INP    (0x00 << 4)
#define S3C2410_GPF2_OUTP   (0x01 << 4)
#define S3C2410_GPF2_EINT2  (0x02 << 4)
#define S3C2400_GPF2_TXD0   (0x02 << 4)

#define S3C2410_GPF3        S3C2410_GPIONO(S3C2410_GPIO_BANKF, 3)
#define S3C2410_GPF3_INP    (0x00 << 6)
#define S3C2410_GPF3_OUTP   (0x01 << 6)
#define S3C2410_GPF3_EINT3  (0x02 << 6)
#define S3C2400_GPF3_TXD1   (0x02 << 6)
#define S3C2400_GPF3_IICSCL (0x03 << 6)

#define S3C2410_GPF4        S3C2410_GPIONO(S3C2410_GPIO_BANKF, 4)
#define S3C2410_GPF4_INP    (0x00 << 8)
#define S3C2410_GPF4_OUTP   (0x01 << 8)
#define S3C2410_GPF4_EINT4  (0x02 << 8)
#define S3C2400_GPF4_nRTS0  (0x02 << 8)
#define S3C2400_GPF4_nXBACK (0x03 << 8)

#define S3C2410_GPF5        S3C2410_GPIONO(S3C2410_GPIO_BANKF, 5)
#define S3C2410_GPF5_INP    (0x00 << 10)
#define S3C2410_GPF5_OUTP   (0x01 << 10)
#define S3C2410_GPF5_EINT5  (0x02 << 10)
#define S3C2400_GPF5_nCTS0  (0x02 << 10)
#define S3C2400_GPF5_nXBREQ (0x03 << 10)

#define S3C2410_GPF6        S3C2410_GPIONO(S3C2410_GPIO_BANKF, 6)
#define S3C2410_GPF6_INP    (0x00 << 12)
#define S3C2410_GPF6_OUTP   (0x01 << 12)
#define S3C2410_GPF6_EINT6  (0x02 << 12)
#define S3C2400_GPF6_CLKOUT (0x02 << 12)

#define S3C2410_GPF7        S3C2410_GPIONO(S3C2410_GPIO_BANKF, 7)
#define S3C2410_GPF7_INP    (0x00 << 14)
#define S3C2410_GPF7_OUTP   (0x01 << 14)
#define S3C2410_GPF7_EINT7  (0x02 << 14)

#define S3C2410_GPF_PUPDIS(x)  (1<<(x))

/* S3C2410:
 * Port G consists of 8 GPIO/IRQ/Special function
 *
 * GPGCON has 2 bits for each of the input pins on port F
 *   00 = 0 input, 1 output, 2 interrupt (EINT0..7), 3 special func
 *
 * pull up works like all other ports.
 *
 * S3C2400:
 * Port G consists of 10 GPIO/Special function
*/

#define S3C2410_GPGCON	   S3C2410_GPIOREG(0x60)
#define S3C2410_GPGDAT	   S3C2410_GPIOREG(0x64)
#define S3C2410_GPGUP	   S3C2410_GPIOREG(0x68)

#define S3C2400_GPGCON	   S3C2410_GPIOREG(0x44)
#define S3C2400_GPGDAT	   S3C2410_GPIOREG(0x48)
#define S3C2400_GPGUP	   S3C2410_GPIOREG(0x4C)

#define S3C2410_GPG0          S3C2410_GPIONO(S3C2410_GPIO_BANKG, 0)
#define S3C2410_GPG0_INP      (0x00 << 0)
#define S3C2410_GPG0_OUTP     (0x01 << 0)
#define S3C2410_GPG0_EINT8    (0x02 << 0)
#define S3C2400_GPG0_I2SLRCK  (0x02 << 0)

#define S3C2410_GPG1          S3C2410_GPIONO(S3C2410_GPIO_BANKG, 1)
#define S3C2410_GPG1_INP      (0x00 << 2)
#define S3C2410_GPG1_OUTP     (0x01 << 2)
#define S3C2410_GPG1_EINT9    (0x02 << 2)
#define S3C2400_GPG1_I2SSCLK  (0x02 << 2)

#define S3C2410_GPG2          S3C2410_GPIONO(S3C2410_GPIO_BANKG, 2)
#define S3C2410_GPG2_INP      (0x00 << 4)
#define S3C2410_GPG2_OUTP     (0x01 << 4)
#define S3C2410_GPG2_EINT10   (0x02 << 4)
#define S3C2400_GPG2_CDCLK    (0x02 << 4)

#define S3C2410_GPG3          S3C2410_GPIONO(S3C2410_GPIO_BANKG, 3)
#define S3C2410_GPG3_INP      (0x00 << 6)
#define S3C2410_GPG3_OUTP     (0x01 << 6)
#define S3C2410_GPG3_EINT11   (0x02 << 6)
#define S3C2400_GPG3_I2SSDO   (0x02 << 6)
#define S3C2400_GPG3_I2SSDI   (0x03 << 6)

#define S3C2410_GPG4          S3C2410_GPIONO(S3C2410_GPIO_BANKG, 4)
#define S3C2410_GPG4_INP      (0x00 << 8)
#define S3C2410_GPG4_OUTP     (0x01 << 8)
#define S3C2410_GPG4_EINT12   (0x02 << 8)
#define S3C2400_GPG4_MMCCLK   (0x02 << 8)
#define S3C2400_GPG4_I2SSDI   (0x03 << 8)
#define S3C2410_GPG4_LCDPWREN (0x03 << 8)
#define S3C2443_GPG4_LCDPWRDN (0x03 << 8)

#define S3C2410_GPG5          S3C2410_GPIONO(S3C2410_GPIO_BANKG, 5)
#define S3C2410_GPG5_INP      (0x00 << 10)
#define S3C2410_GPG5_OUTP     (0x01 << 10)
#define S3C2410_GPG5_EINT13   (0x02 << 10)
#define S3C2400_GPG5_MMCCMD   (0x02 << 10)
#define S3C2400_GPG5_IICSDA   (0x03 << 10)
#define S3C2410_GPG5_SPIMISO1 (0x03 << 10)	/* not s3c2443 */

#define S3C2410_GPG6          S3C2410_GPIONO(S3C2410_GPIO_BANKG, 6)
#define S3C2410_GPG6_INP      (0x00 << 12)
#define S3C2410_GPG6_OUTP     (0x01 << 12)
#define S3C2410_GPG6_EINT14   (0x02 << 12)
#define S3C2400_GPG6_MMCDAT   (0x02 << 12)
#define S3C2400_GPG6_IICSCL   (0x03 << 12)
#define S3C2410_GPG6_SPIMOSI1 (0x03 << 12)

#define S3C2410_GPG7          S3C2410_GPIONO(S3C2410_GPIO_BANKG, 7)
#define S3C2410_GPG7_INP      (0x00 << 14)
#define S3C2410_GPG7_OUTP     (0x01 << 14)
#define S3C2410_GPG7_EINT15   (0x02 << 14)
#define S3C2410_GPG7_SPICLK1  (0x03 << 14)
#define S3C2400_GPG7_SPIMISO  (0x02 << 14)
#define S3C2400_GPG7_IICSDA   (0x03 << 14)

#define S3C2410_GPG8          S3C2410_GPIONO(S3C2410_GPIO_BANKG, 8)
#define S3C2410_GPG8_INP      (0x00 << 16)
#define S3C2410_GPG8_OUTP     (0x01 << 16)
#define S3C2410_GPG8_EINT16   (0x02 << 16)
#define S3C2400_GPG8_SPIMOSI  (0x02 << 16)
#define S3C2400_GPG8_IICSCL   (0x03 << 16)

#define S3C2410_GPG9          S3C2410_GPIONO(S3C2410_GPIO_BANKG, 9)
#define S3C2410_GPG9_INP      (0x00 << 18)
#define S3C2410_GPG9_OUTP     (0x01 << 18)
#define S3C2410_GPG9_EINT17   (0x02 << 18)
#define S3C2400_GPG9_SPICLK   (0x02 << 18)
#define S3C2400_GPG9_MMCCLK   (0x03 << 18)

#define S3C2410_GPG10         S3C2410_GPIONO(S3C2410_GPIO_BANKG, 10)
#define S3C2410_GPG10_INP     (0x00 << 20)
#define S3C2410_GPG10_OUTP    (0x01 << 20)
#define S3C2410_GPG10_EINT18  (0x02 << 20)

#define S3C2410_GPG11         S3C2410_GPIONO(S3C2410_GPIO_BANKG, 11)
#define S3C2410_GPG11_INP     (0x00 << 22)
#define S3C2410_GPG11_OUTP    (0x01 << 22)
#define S3C2410_GPG11_EINT19  (0x02 << 22)
#define S3C2410_GPG11_TCLK1   (0x03 << 22)
#define S3C2443_GPG11_CF_nIREQ (0x03 << 22)

#define S3C2410_GPG12         S3C2410_GPIONO(S3C2410_GPIO_BANKG, 12)
#define S3C2410_GPG12_INP     (0x00 << 24)
#define S3C2410_GPG12_OUTP    (0x01 << 24)
#define S3C2410_GPG12_EINT20  (0x02 << 24)
#define S3C2410_GPG12_XMON    (0x03 << 24)
#define S3C2442_GPG12_nSPICS0 (0x03 << 24)
#define S3C2443_GPG12_nINPACK (0x03 << 24)

#define S3C2410_GPG13         S3C2410_GPIONO(S3C2410_GPIO_BANKG, 13)
#define S3C2410_GPG13_INP     (0x00 << 26)
#define S3C2410_GPG13_OUTP    (0x01 << 26)
#define S3C2410_GPG13_EINT21  (0x02 << 26)
#define S3C2410_GPG13_nXPON   (0x03 << 26)
#define S3C2443_GPG13_CF_nREG (0x03 << 26)

#define S3C2410_GPG14         S3C2410_GPIONO(S3C2410_GPIO_BANKG, 14)
#define S3C2410_GPG14_INP     (0x00 << 28)
#define S3C2410_GPG14_OUTP    (0x01 << 28)
#define S3C2410_GPG14_EINT22  (0x02 << 28)
#define S3C2410_GPG14_YMON    (0x03 << 28)
#define S3C2443_GPG14_CF_RESET (0x03 << 28)

#define S3C2410_GPG15         S3C2410_GPIONO(S3C2410_GPIO_BANKG, 15)
#define S3C2410_GPG15_INP     (0x00 << 30)
#define S3C2410_GPG15_OUTP    (0x01 << 30)
#define S3C2410_GPG15_EINT23  (0x02 << 30)
#define S3C2410_GPG15_nYPON   (0x03 << 30)
#define S3C2443_GPG15_CF_PWR  (0x03 << 30)

#define S3C2410_GPG_PUPDIS(x)  (1<<(x))

/* Port H consists of11 GPIO/serial/Misc pins
 *
 * GPGCON has 2 bits for each of the input pins on port F
 *   00 = 0 input, 1 output, 2 interrupt (EINT0..7), 3 special func
 *
 * pull up works like all other ports.
*/

#define S3C2410_GPHCON	   S3C2410_GPIOREG(0x70)
#define S3C2410_GPHDAT	   S3C2410_GPIOREG(0x74)
#define S3C2410_GPHUP	   S3C2410_GPIOREG(0x78)

#define S3C2410_GPH0        S3C2410_GPIONO(S3C2410_GPIO_BANKH, 0)
#define S3C2410_GPH0_INP    (0x00 << 0)
#define S3C2410_GPH0_OUTP   (0x01 << 0)
#define S3C2410_GPH0_nCTS0  (0x02 << 0)

#define S3C2410_GPH1        S3C2410_GPIONO(S3C2410_GPIO_BANKH, 1)
#define S3C2410_GPH1_INP    (0x00 << 2)
#define S3C2410_GPH1_OUTP   (0x01 << 2)
#define S3C2410_GPH1_nRTS0  (0x02 << 2)

#define S3C2410_GPH2        S3C2410_GPIONO(S3C2410_GPIO_BANKH, 2)
#define S3C2410_GPH2_INP    (0x00 << 4)
#define S3C2410_GPH2_OUTP   (0x01 << 4)
#define S3C2410_GPH2_TXD0   (0x02 << 4)

#define S3C2410_GPH3        S3C2410_GPIONO(S3C2410_GPIO_BANKH, 3)
#define S3C2410_GPH3_INP    (0x00 << 6)
#define S3C2410_GPH3_OUTP   (0x01 << 6)
#define S3C2410_GPH3_RXD0   (0x02 << 6)

#define S3C2410_GPH4        S3C2410_GPIONO(S3C2410_GPIO_BANKH, 4)
#define S3C2410_GPH4_INP    (0x00 << 8)
#define S3C2410_GPH4_OUTP   (0x01 << 8)
#define S3C2410_GPH4_TXD1   (0x02 << 8)

#define S3C2410_GPH5        S3C2410_GPIONO(S3C2410_GPIO_BANKH, 5)
#define S3C2410_GPH5_INP    (0x00 << 10)
#define S3C2410_GPH5_OUTP   (0x01 << 10)
#define S3C2410_GPH5_RXD1   (0x02 << 10)

#define S3C2410_GPH6        S3C2410_GPIONO(S3C2410_GPIO_BANKH, 6)
#define S3C2410_GPH6_INP    (0x00 << 12)
#define S3C2410_GPH6_OUTP   (0x01 << 12)
#define S3C2410_GPH6_TXD2   (0x02 << 12)
#define S3C2410_GPH6_nRTS1  (0x03 << 12)

#define S3C2410_GPH7        S3C2410_GPIONO(S3C2410_GPIO_BANKH, 7)
#define S3C2410_GPH7_INP    (0x00 << 14)
#define S3C2410_GPH7_OUTP   (0x01 << 14)
#define S3C2410_GPH7_RXD2   (0x02 << 14)
#define S3C2410_GPH7_nCTS1  (0x03 << 14)

#define S3C2410_GPH8        S3C2410_GPIONO(S3C2410_GPIO_BANKH, 8)
#define S3C2410_GPH8_INP    (0x00 << 16)
#define S3C2410_GPH8_OUTP   (0x01 << 16)
#define S3C2410_GPH8_UCLK   (0x02 << 16)

#define S3C2410_GPH9          S3C2410_GPIONO(S3C2410_GPIO_BANKH, 9)
#define S3C2410_GPH9_INP      (0x00 << 18)
#define S3C2410_GPH9_OUTP     (0x01 << 18)
#define S3C2410_GPH9_CLKOUT0  (0x02 << 18)
#define S3C2442_GPH9_nSPICS0  (0x03 << 18)

#define S3C2410_GPH10         S3C2410_GPIONO(S3C2410_GPIO_BANKH, 10)
#define S3C2410_GPH10_INP     (0x00 << 20)
#define S3C2410_GPH10_OUTP    (0x01 << 20)
#define S3C2410_GPH10_CLKOUT1 (0x02 << 20)

#define S3C2443_GPH13          S3C2410_GPIONO(S3C2410_GPIO_BANKH, 13)
#define S3C2443_GPH13_INP      (0x00 << 26)
#define S3C2443_GPH13_OUTP     (0x01 << 26)
#define S3C2443_GPH13_CLKOUT0  (0x02 << 26)

#define S3C2443_GPH14         S3C2410_GPIONO(S3C2410_GPIO_BANKH, 14)
#define S3C2443_GPH14_INP     (0x00 << 28)
#define S3C2443_GPH14_OUTP    (0x01 << 28)
#define S3C2443_GPH14_CLKOUT1 (0x02 << 28)



/* Port L consists of14 SPI1/Misc pins
 *
 * GPLCON has 2 bits for each of the input pins on port L
 *   00 = 0 input, 1 output, 
 *
 * pull up works like all other ports.
*/

#define S3C2410_GPLCON	   S3C2410_GPIOREG(0xf0)
#define S3C2410_GPLDAT	   S3C2410_GPIOREG(0xf4)
#define S3C2410_GPLUP	   S3C2410_GPIOREG(0xf8)

#define S3C2450_GPLSEL	   S3C2410_GPIOREG(0xfC)

#define S3C2410_GPL0        S3C2410_GPIONO(S3C2410_GPIO_BANKL, 0)
#define S3C2410_GPL0_INP    (0x00 << 0)
#define S3C2410_GPL0_OUTP   (0x01 << 0)
#define S3C2410_GPL0_SD0_DAT0  (0x02 << 0)

#define S3C2410_GPL1        S3C2410_GPIONO(S3C2410_GPIO_BANKL, 1)
#define S3C2410_GPL1_INP    (0x00 << 2)
#define S3C2410_GPL1_OUTP   (0x01 << 2)
#define S3C2410_GPL1_SD0_DAT1  (0x02 << 2)

#define S3C2410_GPL2        S3C2410_GPIONO(S3C2410_GPIO_BANKL, 2)
#define S3C2410_GPL2_INP    (0x00 << 4)
#define S3C2410_GPL2_OUTP   (0x01 << 4)
#define S3C2410_GPL2_SD0_DAT02   (0x02 << 4)

#define S3C2410_GPL3        S3C2410_GPIONO(S3C2410_GPIO_BANKL, 3)
#define S3C2410_GPL3_INP    (0x00 << 6)
#define S3C2410_GPL3_OUTP   (0x01 << 6)
#define S3C2410_GPL3_SD0_DAT3   (0x02 << 6)

#define S3C2410_GPL4        S3C2410_GPIONO(S3C2410_GPIO_BANKL, 4)
#define S3C2410_GPL4_INP    (0x00 << 8)
#define S3C2410_GPL4_OUTP   (0x01 << 8)
#define S3C2410_GPL4_SD0_DAT4   (0x02 << 8)
#define S3C2450_GPL4_I2S1_SCLK  (0x03 << 8)

#define S3C2410_GPL5        S3C2410_GPIONO(S3C2410_GPIO_BANKL, 5)
#define S3C2410_GPL5_INP    (0x00 << 10)
#define S3C2410_GPL5_OUTP   (0x01 << 10)
#define S3C2410_GPL5_SD0_DAT5   (0x02 << 10)
#define S3C2450_GPL5_I2S1_CDCLK (0x03 << 10)

#define S3C2410_GPL6        S3C2410_GPIONO(S3C2410_GPIO_BANKL, 6)
#define S3C2410_GPL6_INP    (0x00 << 12)
#define S3C2410_GPL6_OUTP   (0x01 << 12)
#define S3C2410_GPL6_TXD2   (0x02 << 12)
#define S3C2410_GPL6_SD0_DAT6  (0x03 << 12)
#define S3C2450_GPL6_I2S1_SDI  (0x03 << 12)

#define S3C2410_GPL7        S3C2410_GPIONO(S3C2410_GPIO_BANKL, 7)
#define S3C2410_GPL7_INP    (0x00 << 14)
#define S3C2410_GPL7_OUTP   (0x01 << 14)
#define S3C2410_GPL7_RXD2   (0x02 << 14)
#define S3C2410_GPL7_SD0_DAT7  (0x03 << 14)
#define S3C2450_GPL7_I2S1_SDO  (0x03 << 14)

#define S3C2410_GPL8        S3C2410_GPIONO(S3C2410_GPIO_BANKL, 8)
#define S3C2410_GPL8_INP    (0x00 << 16)
#define S3C2410_GPL8_OUTP   (0x01 << 16)
#define S3C2410_GPL8_SD0_CMD   (0x02 << 16)

#define S3C2410_GPL9          S3C2410_GPIONO(S3C2410_GPIO_BANKL, 9)
#define S3C2410_GPL9_INP      (0x00 << 18)
#define S3C2410_GPL9_OUTP     (0x01 << 18)
#define S3C2410_GPL9_SD0_CLK  (0x02 << 18)
#define S3C2442_GPL9_nSPICS0  (0x03 << 18)

#define S3C2410_GPL10         S3C2410_GPIONO(S3C2410_GPIO_BANKL, 10)
#define S3C2410_GPL10_INP     (0x00 << 20)
#define S3C2410_GPL10_OUTP    (0x01 << 20)
#define S3C2410_GPL10_SPICLK1 (0x02 << 20)

#define S3C2410_GPL11         S3C2410_GPIONO(S3C2410_GPIO_BANKL, 11)
#define S3C2410_GPL11_INP     (0x00 << 22)
#define S3C2410_GPL11_OUTP    (0x01 << 22)
#define S3C2410_GPL11_SPIMOSI1 (0x02 << 22)

#define S3C2410_GPL12         S3C2410_GPIONO(S3C2410_GPIO_BANKL, 12)
#define S3C2410_GPL12_INP     (0x00 << 24)
#define S3C2410_GPL12_OUTP    (0x01 << 24)
#define S3C2410_GPL12_SPIMISO1 (0x02 << 24)

#define S3C2410_GPL13         S3C2410_GPIONO(S3C2410_GPIO_BANKL, 13)
#define S3C2410_GPL13_INP     (0x00 << 26)
#define S3C2410_GPL13_OUTP    (0x01 << 26)
#define S3C2410_GPL13_SS0 (0x02 << 26)

#define S3C2410_GPL14         S3C2410_GPIONO(S3C2410_GPIO_BANKL, 14)
#define S3C2410_GPL14_INP     (0x00 << 28)
#define S3C2410_GPL14_OUTP    (0x01 << 28)
#define S3C2410_GPL14_SS1 (0x02 << 28)


/* The S3C2412 and S3C2413 move the GPJ register set to after
 * GPH, which means all registers after 0x80 are now offset by 0x10
 * for the 2412/2413 from the 2410/2440/2442
*/

/* miscellaneous control */
#define S3C2400_MISCCR	   S3C2410_GPIOREG(0x54)
#define S3C2410_MISCCR	   S3C2410_GPIOREG(0x80)
#define S3C2410_DCLKCON	   S3C2410_GPIOREG(0x84)

#define S3C24XX_DCLKCON	   S3C24XX_GPIOREG2(0x84)

/* see clock.h for dclk definitions */

/* pullup control on databus */
#define S3C2410_MISCCR_SPUCR_HEN    (0<<0)
#define S3C2410_MISCCR_SPUCR_HDIS   (1<<0)
#define S3C2410_MISCCR_SPUCR_LEN    (0<<1)
#define S3C2410_MISCCR_SPUCR_LDIS   (1<<1)

#define S3C2400_MISCCR_SPUCR_LEN    (0<<0)
#define S3C2400_MISCCR_SPUCR_LDIS   (1<<0)
#define S3C2400_MISCCR_SPUCR_HEN    (0<<1)
#define S3C2400_MISCCR_SPUCR_HDIS   (1<<1)

#define S3C2400_MISCCR_HZ_STOPEN    (0<<2)
#define S3C2400_MISCCR_HZ_STOPPREV  (1<<2)

#define S3C2410_MISCCR_USBDEV	    (0<<3)
#define S3C2410_MISCCR_USBHOST	    (1<<3)

#define S3C2410_MISCCR_CLK0_MPLL    (0<<4)
#define S3C2410_MISCCR_CLK0_UPLL    (1<<4)
#define S3C2410_MISCCR_CLK0_FCLK    (2<<4)
#define S3C2410_MISCCR_CLK0_HCLK    (3<<4)
#define S3C2410_MISCCR_CLK0_PCLK    (4<<4)
#define S3C2410_MISCCR_CLK0_DCLK0   (5<<4)
#define S3C2410_MISCCR_CLK0_MASK    (7<<4)

#define S3C2412_MISCCR_CLK0_RTC	    (2<<4)

#define S3C2410_MISCCR_CLK1_MPLL    (0<<8)
#define S3C2410_MISCCR_CLK1_UPLL    (1<<8)
#define S3C2410_MISCCR_CLK1_FCLK    (2<<8)
#define S3C2410_MISCCR_CLK1_HCLK    (3<<8)
#define S3C2410_MISCCR_CLK1_PCLK    (4<<8)
#define S3C2410_MISCCR_CLK1_DCLK1   (5<<8)
#define S3C2410_MISCCR_CLK1_MASK    (7<<8)

#define S3C2412_MISCCR_CLK1_CLKsrc  (0<<8)

#define S3C2410_MISCCR_USBSUSPND0   (1<<12)
#define S3C2410_MISCCR_USBSUSPND1   (1<<13)

#define S3C2410_MISCCR_nRSTCON	    (1<<16)

#define S3C2410_MISCCR_nEN_SCLK0    (1<<17)
#define S3C2410_MISCCR_nEN_SCLK1    (1<<18)
#define S3C2410_MISCCR_nEN_SCLKE    (1<<19)	/* not 2412 */
#define S3C2410_MISCCR_SDSLEEP	    (7<<17)

/* external interrupt control... */
/* S3C2410_EXTINT0 -> irq sense control for EINT0..EINT7
 * S3C2410_EXTINT1 -> irq sense control for EINT8..EINT15
 * S3C2410_EXTINT2 -> irq sense control for EINT16..EINT23
 *
 * note S3C2410_EXTINT2 has filtering options for EINT16..EINT23
 *
 * Samsung datasheet p9-25
*/
#define S3C2400_EXTINT0    S3C2410_GPIOREG(0x58)
#define S3C2410_EXTINT0	   S3C2410_GPIOREG(0x88)
#define S3C2410_EXTINT1	   S3C2410_GPIOREG(0x8C)
#define S3C2410_EXTINT2	   S3C2410_GPIOREG(0x90)

#define S3C24XX_EXTINT0	   S3C24XX_GPIOREG2(0x88)
#define S3C24XX_EXTINT1	   S3C24XX_GPIOREG2(0x8C)
#define S3C24XX_EXTINT2	   S3C24XX_GPIOREG2(0x90)

/* values for S3C2410_EXTINT0/1/2 */
#define S3C2410_EXTINT_LOWLEV	 (0x00)
#define S3C2410_EXTINT_HILEV	 (0x01)
#define S3C2410_EXTINT_FALLEDGE	 (0x02)
#define S3C2410_EXTINT_RISEEDGE	 (0x04)
#define S3C2410_EXTINT_BOTHEDGE	 (0x06)

/* interrupt filtering conrrol for EINT16..EINT23 */
#define S3C2410_EINFLT0	   S3C2410_GPIOREG(0x94)
#define S3C2410_EINFLT1	   S3C2410_GPIOREG(0x98)
#define S3C2410_EINFLT2	   S3C2410_GPIOREG(0x9C)
#define S3C2410_EINFLT3	   S3C2410_GPIOREG(0xA0)

#define S3C24XX_EINFLT0	   S3C24XX_GPIOREG2(0x94)
#define S3C24XX_EINFLT1	   S3C24XX_GPIOREG2(0x98)
#define S3C24XX_EINFLT2	   S3C24XX_GPIOREG2(0x9C)
#define S3C24XX_EINFLT3	   S3C24XX_GPIOREG2(0xA0)

/* values for interrupt filtering */
#define S3C2410_EINTFLT_PCLK		(0x00)
#define S3C2410_EINTFLT_EXTCLK		(1<<7)
#define S3C2410_EINTFLT_WIDTHMSK(x)	((x) & 0x3f)

/* removed EINTxxxx defs from here, not meant for this */

/* GSTATUS have miscellaneous information in them
 *
 * These move between s3c2410 and s3c2412 style systems.
 */

#define S3C2410_GSTATUS0   S3C2410_GPIOREG(0x0AC)
#define S3C2410_GSTATUS1   S3C2410_GPIOREG(0x0B0)
#define S3C2410_GSTATUS2   S3C2410_GPIOREG(0x0B4)
#define S3C2410_GSTATUS3   S3C2410_GPIOREG(0x0B8)
#define S3C2410_GSTATUS4   S3C2410_GPIOREG(0x0BC)

#define S3C2412_GSTATUS0   S3C2410_GPIOREG(0x0BC)
#define S3C2412_GSTATUS1   S3C2410_GPIOREG(0x0C0)
#define S3C2412_GSTATUS2   S3C2410_GPIOREG(0x0C4)
#define S3C2412_GSTATUS3   S3C2410_GPIOREG(0x0C8)
#define S3C2412_GSTATUS4   S3C2410_GPIOREG(0x0CC)

#define S3C24XX_GSTATUS0   S3C24XX_GPIOREG2(0x0AC)
#define S3C24XX_GSTATUS1   S3C24XX_GPIOREG2(0x0B0)
#define S3C24XX_GSTATUS2   S3C24XX_GPIOREG2(0x0B4)
#define S3C24XX_GSTATUS3   S3C24XX_GPIOREG2(0x0B8)
#define S3C24XX_GSTATUS4   S3C24XX_GPIOREG2(0x0BC)

#define S3C2410_GSTATUS0_nWAIT	   (1<<3)
#define S3C2410_GSTATUS0_NCON	   (1<<2)
#define S3C2410_GSTATUS0_RnB	   (1<<1)
#define S3C2410_GSTATUS0_nBATTFLT  (1<<0)

#define S3C2410_GSTATUS1_IDMASK	   (0xffff0000)
#define S3C2410_GSTATUS1_2410	   (0x32410000)
#define S3C2410_GSTATUS1_2412	   (0x32412001)
#define S3C2410_GSTATUS1_2440	   (0x32440000)
#define S3C2410_GSTATUS1_2442	   (0x32440aaa)

#define S3C2410_GSTATUS2_WTRESET   (1<<2)
#define S3C2410_GSTATUS2_OFFRESET  (1<<1)
#define S3C2410_GSTATUS2_PONRESET  (1<<0)

/* open drain control register */
#define S3C2400_OPENCR     S3C2410_GPIOREG(0x50)

#define S3C2400_OPENCR_OPC_RXD1DIS  (0<<0)
#define S3C2400_OPENCR_OPC_RXD1EN   (1<<0)
#define S3C2400_OPENCR_OPC_TXD1DIS  (0<<1)
#define S3C2400_OPENCR_OPC_TXD1EN   (1<<1)
#define S3C2400_OPENCR_OPC_CMDDIS   (0<<2)
#define S3C2400_OPENCR_OPC_CMDEN    (1<<2)
#define S3C2400_OPENCR_OPC_DATDIS   (0<<3)
#define S3C2400_OPENCR_OPC_DATEN    (1<<3)
#define S3C2400_OPENCR_OPC_MISODIS  (0<<4)
#define S3C2400_OPENCR_OPC_MISOEN   (1<<4)
#define S3C2400_OPENCR_OPC_MOSIDIS  (0<<5)
#define S3C2400_OPENCR_OPC_MOSIEN   (1<<5)

/* 2412/2413 sleep configuration registers */

#define S3C2412_GPBSLPCON	S3C2410_GPIOREG(0x1C)
#define S3C2412_GPCSLPCON	S3C2410_GPIOREG(0x2C)
#define S3C2412_GPDSLPCON	S3C2410_GPIOREG(0x3C)
#define S3C2412_GPESLPCON	S3C2410_GPIOREG(0x4C)
#define S3C2412_GPFSLPCON	S3C2410_GPIOREG(0x5C)
#define S3C2412_GPGSLPCON	S3C2410_GPIOREG(0x6C)
#define S3C2412_GPHSLPCON	S3C2410_GPIOREG(0x7C)

/* definitions for each pin bit */
#define S3C2412_SLPCON_LOW(x)	( 0x00 << ((x) * 2))
#define S3C2412_SLPCON_HIGH(x)	( 0x01 << ((x) * 2))
#define S3C2412_SLPCON_IN(x)	( 0x02 << ((x) * 2))
#define S3C2412_SLPCON_PULL(x)	( 0x03 << ((x) * 2))
#define S3C2412_SLPCON_EINT(x)	( 0x02 << ((x) * 2))  /* only IRQ pins */
#define S3C2412_SLPCON_MASK(x)	( 0x03 << ((x) * 2))

#define S3C2412_SLPCON_ALL_LOW	(0x0)
#define S3C2412_SLPCON_ALL_HIGH	(0x11111111 | 0x44444444)
#define S3C2412_SLPCON_ALL_IN  	(0x22222222 | 0x88888888)
#define S3C2412_SLPCON_ALL_PULL	(0x33333333)

#if defined (CONFIG_PLAT_S3C64XX) || defined (CONFIG_PLAT_S5PC1XX) 

/* configure GPIO ports A..G */

#define S3C_GPIOREG(x) ((x) + S3C24XX_VA_GPIO)

/* GPIO */
#define S3C_GPIONO(bank,offset) ((bank) + (offset))

#define S3C_GPIO_BANKA	 (32*0)
#define S3C_GPIO_BANKB	 (32*1)
#define S3C_GPIO_BANKC	 (32*2)
#define S3C_GPIO_BANKD	 (32*3)
#define S3C_GPIO_BANKE	 (32*4)
#define S3C_GPIO_BANKF	 (32*5)
#define S3C_GPIO_BANKG	 (32*6)
#define S3C_GPIO_BANKH	 (32*7)
#define S3C_GPIO_BANKI	 (32*8)
#define S3C_GPIO_BANKJ	 (32*9)
#define S3C_GPIO_BANKO	 (32*10)
#define S3C_GPIO_BANKP	 (32*11)
#define S3C_GPIO_BANKQ	 (32*12)

#define S3C_GPIO_BANKK	 (32*13)
#define S3C_GPIO_BANKL	 (32*14)
#define S3C_GPIO_BANKM	 (32*15)
#define S3C_GPIO_BANKN	 (32*16)

#define S3C_GPIO_INPUT		(0)
#define S3C_GPIO_OUTPUT		(1)
#define S3C_GPIO_BASE(pin)   ((pin & ~31) >> 5)
#define S3C_GPIO_OFFSET(pin) (pin & 31)

/* general configuration options */

#define S3C_GPIO_LEAVE	 (0xFFFFFFFF)

/* GPA : 8 in/out port . UART */
#define S3C_GPADAT	   S3C_GPIOREG(0x04)
#define S3C_GPACON	   S3C_GPIOREG(0x00)
#define S3C_GPAPU	   S3C_GPIOREG(0x08)
#define S3C_GPASLPCON	   S3C_GPIOREG(0x0C)

#define S3C_GPA0	   S3C_GPIONO(S3C_GPIO_BANKA, 0)
#define S3C_GPA0_INP			(0)
#define S3C_GPA0_OUTP			(1)
#define S3C_GPA0_UART_RXD0	(2)
#define S3C_GPA0_EXT_INT_G1_0	(7)

#define S3C_GPA1	   S3C_GPIONO(S3C_GPIO_BANKA, 1)
#define S3C_GPA1_INP			(0)
#define S3C_GPA1_OUTP			(1)
#define S3C_GPA1_UART_TXD0	(2)
#define S3C_GPA1_EXT_INT_G1_1	(7)

#define S3C_GPA2	   S3C_GPIONO(S3C_GPIO_BANKA, 2)
#define S3C_GPA2_INP			(0)
#define S3C_GPA2_OUTP			(1)
#define S3C_GPA2_UART_CTS0	(2)
#define S3C_GPA2_ADDR_CF0		(5)
#define S3C_GPA2_EXT_INT_G1_2	(7)

#define S3C_GPA3	   S3C_GPIONO(S3C_GPIO_BANKA, 3)
#define S3C_GPA3_INP			(0)
#define S3C_GPA3_OUTP			(1)
#define S3C_GPA3_UART_RTS0	(2)
#define S3C_GPA3_ADDR_CF1		(5)
#define S3C_GPA3_EXT_INT_G1_3	(7)

#define S3C_GPA4       S3C_GPIONO(S3C_GPIO_BANKA, 4)
#define S3C_GPA4_INP			(0)
#define S3C_GPA4_OUTP			(1)
#define S3C_GPA4_UART_RXD1	(2)
#define S3C_GPA4_EXT_INT_G1_4	(7)

#define S3C_GPA5       S3C_GPIONO(S3C_GPIO_BANKA, 5)
#define S3C_GPA5_INP			(0)
#define S3C_GPA5_OUTP			(1)
#define S3C_GPA5_UART_TXD1	(2)
#define S3C_GPA5_EXT_INT_G1_5	(7)

#define S3C_GPA6       S3C_GPIONO(S3C_GPIO_BANKA, 6)
#define S3C_GPA6_INP			(0)
#define S3C_GPA6_OUTP			(1)
#define S3C_GPA6_UART_CTS1	 (2)
#define S3C_GPA6_ADDR_CF0		(5)
#define S3C_GPA6_EXT_INT_G1_6	(7)

#define S3C_GPA7       S3C_GPIONO(S3C_GPIO_BANKA, 7)
#define S3C_GPA7_INP			(0)
#define S3C_GPA7_OUTP			(1)
#define S3C_GPA7_UART_RTS1	 (2)
#define S3C_GPA7_ADDR_CF1		(5)
#define S3C_GPA7_EXT_INT_G1_7	(7)


/* GPB : 7 in/out port . UART, IrDA, External DMA, I2C */
#define S3C_GPBDAT	   S3C_GPIOREG(0x24)
#define S3C_GPBCON	   S3C_GPIOREG(0x20)
#define S3C_GPBPU	   S3C_GPIOREG(0x28)
#define S3C_GPBSLPCON	   S3C_GPIOREG(0x2C)

#define S3C_GPB0       S3C_GPIONO(S3C_GPIO_BANKB, 0)
#define S3C_GPB0_INP			(0)
#define S3C_GPB0_OUTP			(1)
#define S3C_GPB0_UART_RXD2		(2)
#define S3C_GPB0_EXT_DMA_REQ	(3)
#define S3C_GPB0_IRDA_RXD		(4)
#define S3C_GPB0_ADDR_CF0		(5)
#define S3C_GPB0_RESERVED		(6)
#define S3C_GPB0_EXT_INT_G1_8	(7)

#define S3C_GPB1       S3C_GPIONO(S3C_GPIO_BANKB, 1)
#define S3C_GPB1_INP			(0)
#define S3C_GPB1_OUTP			(1)
#define S3C_GPB1_UART_TXD2		(2)
#define S3C_GPB1_EXT_DMA_ACK	(3)
#define S3C_GPB1_IRDA_TXD		(4)
#define S3C_GPB1_ADDR_CF1		(5)
#define S3C_GPB1_RESERVED		(6)
#define S3C_GPB1_EXT_INT_G1_9	(7)

#define S3C_GPB2       S3C_GPIONO(S3C_GPIO_BANKB, 2)
#define S3C_GPB2_INP			(0)
#define S3C_GPB2_OUTP			(1)
#define S3C_GPB2_UART_RXD3		(2)
#define S3C_GPB2_IRDA_RXD		(3)
#define S3C_GPB2_EXT_DMA_REQ	(4)
#define S3C_GPB2_ADDR_CF2		(5)
#define S3C_GPB2_RESERVED		(6)
#define S3C_GPB2_EXT_INT_G1_10	(7)

#define S3C_GPB3       S3C_GPIONO(S3C_GPIO_BANKB, 3)
#define S3C_GPB3_INP			(0)
#define S3C_GPB3_OUTP			(1)
#define S3C_GPB3_UART_TXD3		(2)
#define S3C_GPB3_IRDA_RXD		(3)
#define S3C_GPB3_EXT_DMA_ACK	(4)
#define S3C_GPB3_RESERVED1		(5)
#define S3C_GPB3_RESERVED2		(6)
#define S3C_GPB3_EXT_INT_G1_11	(7)

#define S3C_GPB4       S3C_GPIONO(S3C_GPIO_BANKB, 4)
#define S3C_GPB4_INP			(0)
#define S3C_GPB4_OUTP			(1)
#define S3C_GPB4_IRDA_SDBW		(2)
#define S3C_GPB4_RESERVED1		(3)
#define S3C_GPB4_CF_DATA_DIR	(4)
#define S3C_GPB4_RESERVED2		(5)
#define S3C_GPB4_RESERVED3		(6)
#define S3C_GPB4_EXT_INT_G1_12	(7)

#define S3C_GPB5       S3C_GPIONO(S3C_GPIO_BANKB, 5)
#define S3C_GPB5_INP			(0)
#define S3C_GPB5_OUTP			(1)
#define S3C_GPB5_I2C_SCL	(2)
#define S3C_GPB5_RESERVED1		(3)
#define S3C_GPB5_RESERVED2		(4)
#define S3C_GPB5_RESERVED3		(5)
#define S3C_GPB5_RESERVED4		(6)
#define S3C_GPB5_EXT_INT_G1_13	(7)

#define S3C_GPB6       S3C_GPIONO(S3C_GPIO_BANKB, 6)
#define S3C_GPB6_INP			(0)
#define S3C_GPB6_OUTP			(1)
#define S3C_GPB6_I2C_SDA	(2)
#define S3C_GPB6_RESERVED1		(3)
#define S3C_GPB6_RESERVED2		(4)
#define S3C_GPB6_RESERVED3		(5)
#define S3C_GPB6_RESERVED		(6)
#define S3C_GPB6_EXT_INT_G1_14	(7)


/* GPC : 8 in/out port . SPI */
#define S3C_GPCDAT	   S3C_GPIOREG(0x44)
#define S3C_GPCCON	   S3C_GPIOREG(0x40)
#define S3C_GPCPU	   S3C_GPIOREG(0x48)
#define S3C_GPCSLPCON	   S3C_GPIOREG(0x4C)

#define S3C_GPC0       S3C_GPIONO(S3C_GPIO_BANKC, 0)
#define S3C_GPC0_INP			(0)
#define S3C_GPC0_OUTP			(1)
#define S3C_GPC0_SPI_MISO0		(2)
#define S3C_GPC0_RESERVED1		(3)
#define S3C_GPC0_RESERVED2		(4)
#define S3C_GPC0_ADDR_CF0		(5)
#define S3C_GBP0_RESERVED3		(6)
#define S3C_GPC0_EXT_INT_G2_0	(7)

#define S3C_GPC1       S3C_GPIONO(S3C_GPIO_BANKC, 1)
#define S3C_GPC1_INP			(0)
#define S3C_GPC1_OUTP			(1)
#define S3C_GPC1_SPI_CLK0	(2)
#define S3C_GPC1_RESERVED1		(3)
#define S3C_GPC1_RESERVED2		(4)
#define S3C_GPC1_ADDR_CF1		(5)
#define S3C_GPC1_RESERVED3		(6)
#define S3C_GPC1_EXT_INT_G2_1	(7)

#define S3C_GPC2       S3C_GPIONO(S3C_GPIO_BANKC, 2)
#define S3C_GPC2_INP			(0)
#define S3C_GPC2_OUTP			(1)
#define S3C_GPC2_SPI_MOSI0		(2)
#define S3C_GPC2_RESERVED1		(3)
#define S3C_GPC2_RESERVED2		(4)
#define S3C_GPC2_ADDR_CF2		(5)
#define S3C_GPC2_RESERVED3		(6)
#define S3C_GPC2_EXT_INT_G2_2	(7)

#define S3C_GPC3       S3C_GPIONO(S3C_GPIO_BANKC, 3)
#define S3C_GPC3_INP			(0)
#define S3C_GPC3_OUTP			(1)
#define S3C_GPC3_SPI_CS0	(2)
#define S3C_GPC3_RESERVED1		(3)
#define S3C_GPC3_RESERVED2		(4)
#define S3C_GPC3_RESERVED3		(5)
#define S3C_GPC3_RESERVED4		(6)
#define S3C_GPC3_EXT_INT_G2_3	(7)

#define S3C_GPC4       S3C_GPIONO(S3C_GPIO_BANKC, 4)
#define S3C_GPC4_INP			(0)
#define S3C_GPC4_OUTP			(1)
#define S3C_GPC4_SPI_MISO1		(2)
#define S3C_GPC4_MMC_CMD2		(3)
#define S3C_GPC4_RESERVED1		(4)
#define S3C_GPC4_I2S_V40_DO0		(5)
#define S3C_GPC4_RESERVED3		(6)
#define S3C_GPC4_EXT_INT_G2_4	(7)

#define S3C_GPC5       S3C_GPIONO(S3C_GPIO_BANKC, 5)
#define S3C_GPC5_INP			(0)
#define S3C_GPC5_OUTP			(1)
#define S3C_GPC5_SPI_CLK1	(2)
#define S3C_GPC5_MMC_CLK2		(3)
#define S3C_GPC5_RESERVED2		(4)
#define S3C_GPC5_I2S_V40_DO1		(5)
#define S3C_GPC5_RESERVED4		(6)
#define S3C_GPC5_EXT_INT_G2_5	(7)

#define S3C_GPC6       S3C_GPIONO(S3C_GPIO_BANKC, 6)
#define S3C_GPC6_INP			(0)
#define S3C_GPC6_OUTP			(1)
#define S3C_GPC6_SPI_MOSI1		(2)
#define S3C_GPC6_RESERVED1		(3)
#define S3C_GPC6_RESERVED2		(4)
#define S3C_GPC6_RESERVED3		(5)
#define S3C_GPC6_RESERVED4		(6)
#define S3C_GPC6_EXT_INT_G2_6	(7)

#define S3C_GPC7       S3C_GPIONO(S3C_GPIO_BANKC, 7)
#define S3C_GPC7_INP			(0)
#define S3C_GPC7_OUTP			(1)
#define S3C_GPC7_SPI_CS1	(2)
#define S3C_GPC7_RESERVED1		(3)
#define S3C_GPC7_RESERVED2		(4)
#define S3C_GPC7_I2S_V40_DO2		(5)
#define S3C_GPC7_RESERVED4		(6)
#define S3C_GPC7_EXT_INT_G2_7	(7)


/* GPD : 5 in/out port . PCM, I2S, AC97 */
#define S3C_GPDDAT	   S3C_GPIOREG(0x64)
#define S3C_GPDCON	   S3C_GPIOREG(0x60)
#define S3C_GPDPU	   S3C_GPIOREG(0x68)
#define S3C_GPDSLPCON	   S3C_GPIOREG(0x6C)

#define S3C_GPD0       S3C_GPIONO(S3C_GPIO_BANKD, 0)
#define S3C_GPD0_INP			(0)
#define S3C_GPD0_OUTP			(1)
#define S3C_GPD0_PCM_DCLK0		(2)
#define S3C_GPD0_I2S_CLK0		(3)
#define S3C_GPD0_AC97_BITCLK	(4)
#define S3C_GPD0_ADDR_CF0		(5)
#define S3C_GBP0_RESERVED1		(6)
#define S3C_GPD0_EXT_INT_G3_0	(7)

#define S3C_GPD1       S3C_GPIONO(S3C_GPIO_BANKD, 1)
#define S3C_GPD1_INP			(0)
#define S3C_GPD1_OUTP			(1)
#define S3C_GPD1_PCM_EXTCLK0	(2)
#define S3C_GPD1_I2S_CDCLK0	(3)
#define S3C_GPD1_AC97_RESET	(4)
#define S3C_GPD1_ADDR_CF1		(5)
#define S3C_GPD1_RESERVED1		(6)
#define S3C_GPD1_EXT_INT_G3_1	(7)

#define S3C_GPD2       S3C_GPIONO(S3C_GPIO_BANKD, 2)
#define S3C_GPD2_INP			(0)
#define S3C_GPD2_OUTP			(1)
#define S3C_GPD2_PCM_FSYNC0	(2)
#define S3C_GPD2_I2S_LRCLK0	(3)
#define S3C_GPD2_AC97_SYNC	(4)
#define S3C_GPD2_ADDR_CF2		(5)
#define S3C_GPD2_RESERVED1		(6)
#define S3C_GPD2_EXT_INT_G3_2	(7)

#define S3C_GPD3       S3C_GPIONO(S3C_GPIO_BANKD, 3)
#define S3C_GPD3_INP			(0)
#define S3C_GPD3_OUTP			(1)
#define S3C_GPD3_PCM_SIN0	(2)
#define S3C_GPD3_I2S_DI0		(3)
#define S3C_GPD3_AC97_SDI		(4)
#define S3C_GPD3_RESERVED1		(5)
#define S3C_GPD3_RESERVED2		(6)
#define S3C_GPD3_EXT_INT_G3_3	(7)

#define S3C_GPD4       S3C_GPIONO(S3C_GPIO_BANKD, 4)
#define S3C_GPD4_INP			(0)
#define S3C_GPD4_OUTP			(1)
#define S3C_GPD4_PCM_SOUT0		(2)
#define S3C_GPD4_I2S_DO0		(3)
#define S3C_GPD4_AC97_SDO		(4)
#define S3C_GPD4_RESERVED1		(5)
#define S3C_GPD4_RESERVED2		(6)
#define S3C_GPD4_EXT_INT_G3_4	(7)


/* GPE : 5 in/out port . PCM, I2S, AC97 */
#define S3C_GPEDAT	   S3C_GPIOREG(0x84)
#define S3C_GPECON	   S3C_GPIOREG(0x80)
#define S3C_GPEPU	   S3C_GPIOREG(0x88)
#define S3C_GPESLPCON	   S3C_GPIOREG(0x8C)

#define S3C_GPE0       S3C_GPIONO(S3C_GPIO_BANKE, 0)
#define S3C_GPE0_INP			(0)
#define S3C_GPE0_OUTP			(1)
#define S3C_GPE0_PCM_DCLK1		(2)
#define S3C_GPE0_I2S_CLK1		(3)
#define S3C_GPE0_AC97_BITCLK	(4)
#define S3C_GPE0_RESERVED1		(5)
#define S3C_GPE0_RESERVED2		(6)
#define S3C_GPE0_EXT_INT_G3_5	(7)

#define S3C_GPE1       S3C_GPIONO(S3C_GPIO_BANKE, 1)
#define S3C_GPE1_INP			(0)
#define S3C_GPE1_OUTP			(1)
#define S3C_GPE1_PCM_EXTCLK1	(2)
#define S3C_GPE1_I2S_CDCLK1	(3)
#define S3C_GPE1_AC97_RESET	(4)
#define S3C_GPE1_RESERVED1		(5)
#define S3C_GPE1_RESERVED2		(6)
#define S3C_GPE1_EXT_INT_G3_6	(7)

#define S3C_GPE2       S3C_GPIONO(S3C_GPIO_BANKE, 2)
#define S3C_GPE2_INP			(0)
#define S3C_GPE2_OUTP			(1)
#define S3C_GPE2_PCM_FSYNC1	(2)
#define S3C_GPE2_I2S_LRCLK1	(3)
#define S3C_GPE2_AC97_SYNC		(4)
#define S3C_GPE2_RESERVED1		(5)
#define S3C_GPE2_RESERVED2		(6)
#define S3C_GPE2_EXT_INT_G3_7	(7)

#define S3C_GPE3       S3C_GPIONO(S3C_GPIO_BANKE, 3)
#define S3C_GPE3_INP			(0)
#define S3C_GPE3_OUTP			(1)
#define S3C_GPE3_PCM_SIN1	(2)
#define S3C_GPE3_I2S_DI1		(3)
#define S3C_GPE3_AC97_SDI		(4)
#define S3C_GPE3_RESERVED1		(5)
#define S3C_GPE3_RESERVED2		(6)
#define S3C_GPE3_EXT_INT_G3_8	(7)

#define S3C_GPE4       S3C_GPIONO(S3C_GPIO_BANKE, 4)
#define S3C_GPE4_INP			(0)
#define S3C_GPE4_OUTP			(1)
#define S3C_GPE4_PCM_SOUT1		(2)
#define S3C_GPE4_I2S_DO1		(3)
#define S3C_GPE4_AC97_SDO		(4)
#define S3C_GPE4_RESERVED1		(5)
#define S3C_GPE4_RESERVED2		(6)
#define S3C_GPE4_EXT_INT_G3_9	(7)

#define S3C_GPE11       S3C_GPIONO(S3C_GPIO_BANKE, 11)
#define S3C_GPE11_INP			(0)
#define S3C_GPE11_OUTP			(1)
#define S3C_GPE11_SPI_MISO0		(2)
#define S3C_GPE11_I2S_DO1		(3)
#define S3C_GPE11_AC97_SDO		(4)
#define S3C_GPE11_RESERVED1		(5)
#define S3C_GPE11_RESERVED2		(6)
#define S3C_GPE11_EXT_INT_G3_9		(7)

#define S3C_GPE12       S3C_GPIONO(S3C_GPIO_BANKE, 12)
#define S3C_GPE12_INP			(0)
#define S3C_GPE12_OUTP			(1)
#define S3C_GPE12_SPI_MOSI0		(2)
#define S3C_GPE12_I2S_DO1		(3)
#define S3C_GPE12_AC97_SDO		(4)
#define S3C_GPE12_RESERVED1		(5)
#define S3C_GPE12_RESERVED2		(6)
#define S3C_GPE12_EXT_INT_G3_9		(7)

#define S3C_GPE13       S3C_GPIONO(S3C_GPIO_BANKE, 13)
#define S3C_GPE13_INP			(0)
#define S3C_GPE13_OUTP			(1)
#define S3C_GPE13_SPI_CLK0		(2)
#define S3C_GPE13_I2S_DO1		(3)
#define S3C_GPE13_AC97_SDO		(4)
#define S3C_GPE13_RESERVED1		(5)
#define S3C_GPE13_RESERVED2		(6)
#define S3C_GPE13_EXT_INT_G3_9		(7)


/* GPF : 16 in/out port . Camera I/F, PWM, Clock Out */
#define S3C_GPFDAT	   S3C_GPIOREG(0xA4)
#define S3C_GPFCON	   S3C_GPIOREG(0xA0)
#define S3C_GPFPU	   S3C_GPIOREG(0xA8)
#define S3C_GPFSLPCON	   S3C_GPIOREG(0xAC)

#define S3C_GPF0       S3C_GPIONO(S3C_GPIO_BANKF, 0)
#define S3C_GPF0_INP			(0)
#define S3C_GPF0_OUTP			(1)
#define S3C_GPF0_CAMIF_CLK		(2)
#define S3C_GPF0_EXT_INT_G4_0	(3)

#define S3C_GPF1       S3C_GPIONO(S3C_GPIO_BANKF, 1)
#define S3C_GPF1_INP			(0)
#define S3C_GPF1_OUTP			(1)
#define S3C_GPF1_CAMIF_HREF		(2)
#define S3C_GPF1_EXT_INT_G4_1	(3)

#define S3C_GPF2       S3C_GPIONO(S3C_GPIO_BANKF, 2)
#define S3C_GPF2_INP			(0)
#define S3C_GPF2_OUTP			(1)
#define S3C_GPF2_CAMIF_CLK		(2)
#define S3C_GPF2_EXT_INT_G4_2	(3)

#define S3C_GPF3       S3C_GPIONO(S3C_GPIO_BANKF, 3)
#define S3C_GPF3_INP			(0)
#define S3C_GPF3_OUTP			(1)
#define S3C_GPF3_CAMIF_RST		(2)
#define S3C_GPF3_EXT_INT_G4_3	(3)

#define S3C_GPF4       S3C_GPIONO(S3C_GPIO_BANKF, 4)
#define S3C_GPF4_INP			(0)
#define S3C_GPF4_OUTP			(1)
#define S3C_GPF4_CAMIF_VSYNC		(2)
#define S3C_GPF4_EXT_INT_G4_4	(3)

#define S3C_GPF5       S3C_GPIONO(S3C_GPIO_BANKF, 5)
#define S3C_GPF5_INP			(0)
#define S3C_GPF5_OUTP			(1)
#define S3C_GPF5_CAMIF_YDATA0		(2)
#define S3C_GPF5_EXT_INT_G4_5	(3)

#define S3C_GPF6       S3C_GPIONO(S3C_GPIO_BANKF, 6)
#define S3C_GPF6_INP			(0)
#define S3C_GPF6_OUTP			(1)
#define S3C_GPF6_CAMIF_YDATA1		(2)
#define S3C_GPF6_EXT_INT_G4_6	(3)

#define S3C_GPF7       S3C_GPIONO(S3C_GPIO_BANKF, 7)
#define S3C_GPF7_INP			(0)
#define S3C_GPF7_OUTP			(1)
#define S3C_GPF7_CAMIF_YDATA2		(2)
#define S3C_GPF7_EXT_INT_G4_7	(3)

#define S3C_GPF8       S3C_GPIONO(S3C_GPIO_BANKF, 8)
#define S3C_GPF8_INP			(0)
#define S3C_GPF8_OUTP			(1)
#define S3C_GPF8_CAMIF_YDATA03		(2)
#define S3C_GPF8_EXT_INT_G4_8	(3)

#define S3C_GPF9       S3C_GPIONO(S3C_GPIO_BANKF, 9)
#define S3C_GPF9_INP			(0)
#define S3C_GPF9_OUTP			(1)
#define S3C_GPF9_CAMIF_YDATA4		(2)
#define S3C_GPF9_EXT_INT_G4_9	(3)

#define S3C_GPF10	S3C_GPIONO(S3C_GPIO_BANKF, 10)
#define S3C_GPF10_INP			(0)
#define S3C_GPF10_OUTP			(1)
#define S3C_GPF10_CAMIF_YDATA5		(2)
#define S3C_GPF10_EXT_INT_G4_10	(3)

#define S3C_GPF11	S3C_GPIONO(S3C_GPIO_BANKF, 11)
#define S3C_GPF11_INP			(0)
#define S3C_GPF11_OUTP			(1)
#define S3C_GPF11_CAMIF_YDATA06 	(2)
#define S3C_GPF11_EXT_INT_G4_11	(3)

#define S3C_GPF12	S3C_GPIONO(S3C_GPIO_BANKF, 12)
#define S3C_GPF12_INP			(0)
#define S3C_GPF12_OUTP			(1)
#define S3C_GPF12_CAMIF_YDATA7		(2)
#define S3C_GPF12_EXT_INT_G4_12	(3)

#define S3C_GPF13	S3C_GPIONO(S3C_GPIO_BANKF, 13)
#define S3C_GPF13_INP			(0)
#define S3C_GPF13_OUTP			(1)
#define S3C_GPF13_PWM_ECLK		(2)
#define S3C_GPF13_EXT_INT_G4_13	(3)

#define S3C_GPF14	S3C_GPIONO(S3C_GPIO_BANKF, 14)
#define S3C_GPF14_INP			(0)
#define S3C_GPF14_OUTP			(1)
#define S3C_GPF14_PWM_TOUT0		(2)
#define S3C_GPF14_CLKOUT0	(3)

#define S3C_GPF15	S3C_GPIONO(S3C_GPIO_BANKF, 15)
#define S3C_GPF15_INP			(0)
#define S3C_GPF15_OUTP			(1)
#define S3C_GPF15_PWM_TOUT1		(2)
#define S3C_GPF15_RESERVED		(3)


/* GPG : 7 in/out port . MMC channel 0 */
#define S3C_GPGDAT	S3C_GPIOREG(0xC4)
#define S3C_GPGCON	S3C_GPIOREG(0xC0)
#define S3C_GPGPU	S3C_GPIOREG(0xC8)
#define S3C_GPGSLPCON	S3C_GPIOREG(0xCC)

#define S3C_GPG0	S3C_GPIONO(S3C_GPIO_BANKG, 0)
#define S3C_GPG0_INP		(0)
#define S3C_GPG0_OUTP		(1)
#define S3C_GPG0_MMC_CLK0	(2)
#define S3C_GPG0_RESERVED1	(3)
#define S3C_GPG0_RESERVED2	(4)
#define S3C_GPG0_ADDR_CF0	(5)
#define S3C_GPG0_RESERVED3	(6)
#define S3C_GPG0_EXT_INT_G5_1	(7)

#define S3C_GPG1	S3C_GPIONO(S3C_GPIO_BANKG, 1)
#define S3C_GPG1_INP		(0)
#define S3C_GPG1_OUTP		(1)
#define S3C_GPG1_MMC_CMD0	(2)
#define S3C_GPG1_EXT_DMA_ACK	(3)
#define S3C_GPG1_RESERVED1	(4)
#define S3C_GPG1_ADDR_CF1	(5)
#define S3C_GPG1_RESERVED2	(6)
#define S3C_GPG1_EXT_INT_G5_2	(7)

#define S3C_GPG2	S3C_GPIONO(S3C_GPIO_BANKG, 2)
#define S3C_GPG2_INP		(0)
#define S3C_GPG2_OUTP		(1)
#define S3C_GPG2_MMC_DATA0_0	(2)
#define S3C_GPG2_RESERVED1	(3)
#define S3C_GPG2_RESERVED2	(4)
#define S3C_GPG2_RESERVED3	(5)
#define S3C_GPG2_RESERVED4	(6)
#define S3C_GPG2_EXT_INT_G5_3	(7)

#define S3C_GPG3	S3C_GPIONO(S3C_GPIO_BANKG, 3)
#define S3C_GPG3_INP		(0)
#define S3C_GPG3_OUTP		(1)
#define S3C_GPG3_MMC_DATA0_1	(2)
#define S3C_GPG3_RESERVED1	(3)
#define S3C_GPG3_RESERVED2	(4)
#define S3C_GPG3_RESERVED3	(5)
#define S3C_GPG3_RESERVED4	(6)
#define S3C_GPG3_EXT_INT_G5_4	(7)

#define S3C_GPG4	S3C_GPIONO(S3C_GPIO_BANKG, 4)
#define S3C_GPG4_INP		(0)
#define S3C_GPG4_OUTP		(1)
#define S3C_GPG4_MMC_DATA0_2	(2)
#define S3C_GPG4_RESERVED1	(3)
#define S3C_GPG4_RESERVED2	(4)
#define S3C_GPG4_RESERVED3	(5)
#define S3C_GPG4_RESERVED4	(6)
#define S3C_GPG4_EXT_INT_G5_5	(7)

#define S3C_GPG5	S3C_GPIONO(S3C_GPIO_BANKG, 5)
#define S3C_GPG5_INP		(0)
#define S3C_GPG5_OUTP		(1)
#define S3C_GPG5_MMC_DATA0_3	(2)
#define S3C_GPG5_RESERVED1	(3)
#define S3C_GPG5_RESERVED2	(4)
#define S3C_GPG5_RESERVED3	(5)
#define S3C_GPG5_RESERVED4	(6)
#define S3C_GPG5_EXT_INT_G5_6	(7)

#define S3C_GPG6	S3C_GPIONO(S3C_GPIO_BANKG, 6)
#define S3C_GPG6_INP		(0)
#define S3C_GPG6_OUTP		(1)
#define S3C_GPG6_MMC_CD0	(2)
#define S3C_GPG6_MMC_CD1	(3)
#define S3C_GPG6_RESERVED2	(4)
#define S3C_GPG6_RESERVED3	(5)
#define S3C_GPG6_RESERVED4	(6)
#define S3C_GPG6_EXT_INT_G5_7	(7)


/*GPH : 10 in/out port . MMC channel 1 */
/* Notice!!! Watch out GPIO data register address*/
#define S3C_GPHDAT	S3C_GPIOREG(0xE8)
#define S3C_GPH0CON	S3C_GPIOREG(0xE0)
#define S3C_GPH1CON	S3C_GPIOREG(0xE4)
#define S3C_GPHPU	S3C_GPIOREG(0xEC)
#define S3C_GPHSLPCON	S3C_GPIOREG(0xF0)

#define S3C_GPH0	S3C_GPIONO(S3C_GPIO_BANKH, 0)
#define S3C_GPH0_INP		(0)
#define S3C_GPH0_OUTP		(1)
#define S3C_GPH0_MMC_CLK1	(2)
#define S3C_GPH0_RESERVED1	(3)
#define S3C_GPH0_KEYPAD_COL0	(4)
#define S3C_GPH0_ADDR_CF0	(5)
#define S3C_GPH0_RESERVED2	(6)
#define S3C_GPH0_EXT_INT_G6_0	(7)

#define S3C_GPH1	S3C_GPIONO(S3C_GPIO_BANKH, 1)
#define S3C_GPH1_INP		(0)
#define S3C_GPH1_OUTP		(1)
#define S3C_GPH1_MMC_CMD1	(2)
#define S3C_GPH1_RESERVED1	(3)
#define S3C_GPH1_KEYPAD_COL1	(4)
#define S3C_GPH1_ADDR_CF1	(5)
#define S3C_GPH1_RESERVED2	(6)
#define S3C_GPH1_EXT_INT_G6_1	(7)

#define S3C_GPH2	S3C_GPIONO(S3C_GPIO_BANKH, 2)
#define S3C_GPH2_INP		(0)
#define S3C_GPH2_OUTP		(1)
#define S3C_GPH2_MMC_DATA1_0	(2)
#define S3C_GPH2_RESERVED1	(3)
#define S3C_GPH2_KEYPAD_COL2	(4)
#define S3C_GPH2_RESERVED3	(5)
#define S3C_GPH2_RESERVED4	(6)
#define S3C_GPH2_EXT_INT_G6_2	(7)

#define S3C_GPH3	S3C_GPIONO(S3C_GPIO_BANKH, 3)
#define S3C_GPH3_INP		(0)
#define S3C_GPH3_OUTP		(1)
#define S3C_GPH3_MMC_DATA1_1	(2)
#define S3C_GPH3_RESERVED1	(3)
#define S3C_GPH3_KEYPAD_COL3	(4)
#define S3C_GPH3_RESERVED3	(5)
#define S3C_GPH3_RESERVED4	(6)
#define S3C_GPH3_EXT_INT_G6_3	(7)

#define S3C_GPH4	S3C_GPIONO(S3C_GPIO_BANKH, 4)
#define S3C_GPH4_INP		(0)
#define S3C_GPH4_OUTP		(1)
#define S3C_GPH4_MMC_DATA1_2	(2)
#define S3C_GPH4_RESERVED1	(3)
#define S3C_GPH4_KEYPAD_COL4	(4)
#define S3C_GPH4_RESERVED3	(5)
#define S3C_GPH4_RESERVED4	(6)
#define S3C_GPH4_EXT_INT_G6_4	(7)

#define S3C_GPH5	S3C_GPIONO(S3C_GPIO_BANKH, 5)
#define S3C_GPH5_INP		(0)
#define S3C_GPH5_OUTP		(1)
#define S3C_GPH5_MMC_DATA1_3	(2)
#define S3C_GPH5_RESERVED1	(3)
#define S3C_GPH5_KEYPAD_COL5	(4)
#define S3C_GPH5_RESERVED3	(5)
#define S3C_GPH5_RESERVED4	(6)
#define S3C_GPH5_EXT_INT_G6_5	(7)

#define S3C_GPH6	S3C_GPIONO(S3C_GPIO_BANKH, 6)
#define S3C_GPH6_INP		(0)
#define S3C_GPH6_OUTP		(1)
#define S3C_GPH6_MMC_DATA1_4	(2)
#define S3C_GPH6_MMC_DATA2_0	(3)
#define S3C_GPH6_KEYPAD_COL6	(4)
#define S3C_GPH6_I2S_V40_BCLK	(5)
#define S3C_GPH6_ADDR_CF0	(6)
#define S3C_GPH6_EXT_INT_G6_6	(7)

#define S3C_GPH7	S3C_GPIONO(S3C_GPIO_BANKH, 7)
#define S3C_GPH7_INP		(0)
#define S3C_GPH7_OUTP		(1)
#define S3C_GPH7_MMC_DATA1_5	(2)
#define S3C_GPH7_MMC_DATA2_1	(3)
#define S3C_GPH7_KEYPAD_COL7	(4)
#define S3C_GPH7_I2S_V40_CDCLK	(5)
#define S3C_GPH7_ADDR_CF1	(6)
#define S3C_GPH7_EXT_INT_G6_7	(7)

#define S3C_GPH8	S3C_GPIONO(S3C_GPIO_BANKH, 8)
#define S3C_GPH8_INP		(0)
#define S3C_GPH8_OUTP		(1)
#define S3C_GPH8_MMC_DATA1_6	(2)
#define S3C_GPH8_MMC_DATA2_2	(3)
#define S3C_GPH8_RESERVED2	(4)
#define S3C_GPH8_I2S_V40_LRCLK	(5)
#define S3C_GPH8_ADDR_CF2	(6)
#define S3C_GPH8_EXT_INT_G6_8	(7)

#define S3C_GPH9	S3C_GPIONO(S3C_GPIO_BANKH, 9)
#define S3C_GPH9_INP		(0)
#define S3C_GPH9_OUTP		(1)
#define S3C_GPH9_MMC_DATA1_7	(2)
#define S3C_GPH9_MMC_DATA2_3	(3)
#define S3C_GPH9_RESERVED2	(4)
#define S3C_GPH9_I2S_V40_DI	(5)
#define S3C_GPH9_RESERVED4	(6)
#define S3C_GPH9_EXT_INT_G6_9	(7)


/* GPI : 16 in/out port . LCD Video Out[15:0] */
#define S3C_GPIDAT	   S3C_GPIOREG(0x104)
#define S3C_GPICON	   S3C_GPIOREG(0x100)
#define S3C_GPIPU	   S3C_GPIOREG(0x108)
#define S3C_GPISLPCON	   S3C_GPIOREG(0x10C)

#define S3C_GPI0       S3C_GPIONO(S3C_GPIO_BANKI, 0)
#define S3C_GPI0_INP			(0)
#define S3C_GPI0_OUTP			(1)
#define S3C_GPI0_LCD_VD0		(2)
#define S3C_GPI0_RESERVED		(3)

#define S3C_GPI1       S3C_GPIONO(S3C_GPIO_BANKI, 1)
#define S3C_GPI1_INP			(0)
#define S3C_GPI1_OUTP			(1)
#define S3C_GPI1_LCD_VD1		(2)
#define S3C_GPI1_RESERVED		(3)

#define S3C_GPI2       S3C_GPIONO(S3C_GPIO_BANKI, 2)
#define S3C_GPI2_INP			(0)
#define S3C_GPI2_OUTP			(1)
#define S3C_GPI2_LCD_VD2		(2)
#define S3C_GPI2_RESERVED		(3)

#define S3C_GPI3       S3C_GPIONO(S3C_GPIO_BANKI, 3)
#define S3C_GPI3_INP			(0)
#define S3C_GPI3_OUTP			(1)
#define S3C_GPI3_LCD_VD3		(2)
#define S3C_GPI3_RESERVED		(3)

#define S3C_GPI4       S3C_GPIONO(S3C_GPIO_BANKI, 4)
#define S3C_GPI4_INP			(0)
#define S3C_GPI4_OUTP			(1)
#define S3C_GPI4_LCD_VD4		(2)
#define S3C_GPI4_RESERVED		(3)

#define S3C_GPI5       S3C_GPIONO(S3C_GPIO_BANKI, 5)
#define S3C_GPI5_INP			(0)
#define S3C_GPI5_OUTP			(1)
#define S3C_GPI5_LCD_VD5		(2)
#define S3C_GPI5_RESERVED		(3)

#define S3C_GPI6       S3C_GPIONO(S3C_GPIO_BANKI, 6)
#define S3C_GPI6_INP			(0)
#define S3C_GPI6_OUTP			(1)
#define S3C_GPI6_LCD_VD6		(2)
#define S3C_GPI6_RESERVED		(3)

#define S3C_GPI7       S3C_GPIONO(S3C_GPIO_BANKI, 7)
#define S3C_GPI7_INP			(0)
#define S3C_GPI7_OUTP			(1)
#define S3C_GPI7_LCD_VD7		(2)
#define S3C_GPI7_RESERVED		(3)

#define S3C_GPI8       S3C_GPIONO(S3C_GPIO_BANKI, 8)
#define S3C_GPI8_INP			(0)
#define S3C_GPI8_OUTP			(1)
#define S3C_GPI8_LCD_VD8		(2)
#define S3C_GPI8_RESERVED		(3)

#define S3C_GPI9       S3C_GPIONO(S3C_GPIO_BANKI, 9)
#define S3C_GPI9_INP			(0)
#define S3C_GPI9_OUTP			(1)
#define S3C_GPI9_LCD_VD9		(2)
#define S3C_GPI9_RESERVED		(3)

#define S3C_GPI10	S3C_GPIONO(S3C_GPIO_BANKI, 10)
#define S3C_GPI10_INP			(0)
#define S3C_GPI10_OUTP			(1)
#define S3C_GPI10_LCD_VD10		(2)
#define S3C_GPI10_RESERVED		(3)

#define S3C_GPI11	S3C_GPIONO(S3C_GPIO_BANKI, 11)
#define S3C_GPI11_INP			(0)
#define S3C_GPI11_OUTP			(1)
#define S3C_GPI11_LCD_VD11		(2)
#define S3C_GPI11_RESERVED		(3)

#define S3C_GPI12	S3C_GPIONO(S3C_GPIO_BANKI, 12)
#define S3C_GPI12_INP			(0)
#define S3C_GPI12_OUTP			(1)
#define S3C_GPI12_LCD_VD12		(2)
#define S3C_GPI12_RESERVED		(3)

#define S3C_GPI13	S3C_GPIONO(S3C_GPIO_BANKI, 13)
#define S3C_GPI13_INP			(0)
#define S3C_GPI13_OUTP			(1)
#define S3C_GPI13_LCD_VD13		(2)
#define S3C_GPI13_RESERVED		(3)

#define S3C_GPI14	S3C_GPIONO(S3C_GPIO_BANKI, 14)
#define S3C_GPI14_INP			(0)
#define S3C_GPI14_OUTP			(1)
#define S3C_GPI14_LCD_VD14		(2)
#define S3C_GPI14_RESERVED		(3)

#define S3C_GPI15	S3C_GPIONO(S3C_GPIO_BANKI, 15)
#define S3C_GPI15_INP			(0)
#define S3C_GPI15_OUTP			(1)
#define S3C_GPI15_LCD_VD15		(2)
#define S3C_GPI15_RESERVED		(3)


/* GPJ : 12 in/out port . LCD Video Out[23:16], Control signals */
#define S3C_GPJDAT	   S3C_GPIOREG(0x124)
#define S3C_GPJCON	   S3C_GPIOREG(0x120)
#define S3C_GPJPU	   S3C_GPIOREG(0x128)
#define S3C_GPJSLPCON	   S3C_GPIOREG(0x12C)

#define S3C_GPJ0       S3C_GPIONO(S3C_GPIO_BANKJ, 0)
#define S3C_GPJ0_INP			(0)
#define S3C_GPJ0_OUTP			(1)
#define S3C_GPJ0_LCD_VD16		(2)
#define S3C_GPJ0_RESERVED		(3)

#define S3C_GPJ1       S3C_GPIONO(S3C_GPIO_BANKJ, 1)
#define S3C_GPJ1_INP			(0)
#define S3C_GPJ1_OUTP			(1)
#define S3C_GPJ1_LCD_VD17		(2)
#define S3C_GPJ1_RESERVED		(3)

#define S3C_GPJ2       S3C_GPIONO(S3C_GPIO_BANKJ, 2)
#define S3C_GPJ2_INP			(0)
#define S3C_GPJ2_OUTP			(1)
#define S3C_GPJ2_LCD_VD18		(2)
#define S3C_GPJ2_RESERVED		(3)

#define S3C_GPJ3       S3C_GPIONO(S3C_GPIO_BANKJ, 3)
#define S3C_GPJ3_INP			(0)
#define S3C_GPJ3_OUTP			(1)
#define S3C_GPJ3_LCD_VD19		(2)
#define S3C_GPJ3_RESERVED		(3)

#define S3C_GPJ4       S3C_GPIONO(S3C_GPIO_BANKJ, 4)
#define S3C_GPJ4_INP			(0)
#define S3C_GPJ4_OUTP			(1)
#define S3C_GPJ4_LCD_VD20		(2)
#define S3C_GPJ4_RESERVED		(3)

#define S3C_GPJ5       S3C_GPIONO(S3C_GPIO_BANKJ, 5)
#define S3C_GPJ5_INP			(0)
#define S3C_GPJ5_OUTP			(1)
#define S3C_GPJ5_LCD_VD21		(2)
#define S3C_GPJ5_RESERVED		(3)

#define S3C_GPJ6       S3C_GPIONO(S3C_GPIO_BANKJ, 6)
#define S3C_GPJ6_INP			(0)
#define S3C_GPJ6_OUTP			(1)
#define S3C_GPJ6_LCD_VD22		(2)
#define S3C_GPJ6_RESERVED		(3)

#define S3C_GPJ7       S3C_GPIONO(S3C_GPIO_BANKJ, 7)
#define S3C_GPJ7_INP			(0)
#define S3C_GPJ7_OUTP			(1)
#define S3C_GPJ7_LCD_VD23		(2)
#define S3C_GPJ7_RESERVED		(3)

#define S3C_GPJ8       S3C_GPIONO(S3C_GPIO_BANKJ, 8)
#define S3C_GPJ8_INP			(0)
#define S3C_GPJ8_OUTP			(1)
#define S3C_GPJ8_LCD_HSYNC		(2)
#define S3C_GPJ8_RESERVED		(3)

#define S3C_GPJ9       S3C_GPIONO(S3C_GPIO_BANKJ, 9)
#define S3C_GPJ9_INP			(0)
#define S3C_GPJ9_OUTP			(1)
#define S3C_GPJ9_LCD_VSYNC		(2)
#define S3C_GPJ9_RESERVED		(3)

#define S3C_GPJ10	S3C_GPIONO(S3C_GPIO_BANKJ, 10)
#define S3C_GPJ10_INP			(0)
#define S3C_GPJ10_OUTP			(1)
#define S3C_GPJ10_LCD_VDEN		(2)
#define S3C_GPJ10_RESERVED		(3)

#define S3C_GPJ11	S3C_GPIONO(S3C_GPIO_BANKJ, 11)
#define S3C_GPJ11_INP			(0)
#define S3C_GPJ11_OUTP			(1)
#define S3C_GPJ11_LCD_VCLK		(2)
#define S3C_GPJ11_RESERVED		(3)


/* GPK : 16 in/out port . Host I/F, HSI, Key pad I/F */
#define S3C_GPKDAT	   S3C_GPIOREG(0x808)
#define S3C_GPK0CON	   S3C_GPIOREG(0x800)
#define S3C_GPK1CON	   S3C_GPIOREG(0x804)
#define S3C_GPKPU	   S3C_GPIOREG(0x80C)

#define S3C_GPK0       S3C_GPIONO(S3C_GPIO_BANKK, 0)
#define S3C_GPK0_INP			(0)
#define S3C_GPK0_OUTP			(1)
#define S3C_GPK0_HOSTIF_DATA0		(2)
#define S3C_GPK0_HSI_RX_RDY		(3)
#define S3C_GPK0_RESERVED1		(4)
#define S3C_GPK0_DATA_CF0		(5)
#define S3C_GPK0_RESERVED2		(6)
#define S3C_GPK0_RESERVED3		(7)

#define S3C_GPK1       S3C_GPIONO(S3C_GPIO_BANKK, 1)
#define S3C_GPK1_INP			(0)
#define S3C_GPK1_OUTP			(1)
#define S3C_GPK1_HOSTIF_DATA1		(2)
#define S3C_GPK1_HSI_RX_WAKE		(3)
#define S3C_GPK1_RESERVED1		(4)
#define S3C_GPK1_DATA_CF1		(5)
#define S3C_GPK1_RESERVED2		(6)
#define S3C_GPK1_RESERVED3		(7)

#define S3C_GPK2       S3C_GPIONO(S3C_GPIO_BANKK, 2)
#define S3C_GPK2_INP			(0)
#define S3C_GPK2_OUTP			(1)
#define S3C_GPK2_HOSTIF_DATA2		(2)
#define S3C_GPK2_HSI_RX_FLAG		(3)
#define S3C_GPK2_RESERVED1		(4)
#define S3C_GPK2_DATA_CF2		(5)
#define S3C_GPK2_RESERVED2		(6)
#define S3C_GPK2_RESERVED3		(7)

#define S3C_GPK3       S3C_GPIONO(S3C_GPIO_BANKK, 3)
#define S3C_GPK3_INP			(0)
#define S3C_GPK3_OUTP			(1)
#define S3C_GPK3_HOSTIF_DATA3		(2)
#define S3C_GPK3_HSI_RX_DATA		(3)
#define S3C_GPK3_RESERVED1		(4)
#define S3C_GPK3_DATA_CF3		(5)
#define S3C_GPK3_RESERVED2		(6)
#define S3C_GPK3_RESERVED3		(7)

#define S3C_GPK4       S3C_GPIONO(S3C_GPIO_BANKK, 4)
#define S3C_GPK4_INP			(0)
#define S3C_GPK4_OUTP			(1)
#define S3C_GPK4_HOSTIF_DATA4		(2)
#define S3C_GPK4_HSI_TX_RDY		(3)
#define S3C_GPK4_RESERVED1		(4)
#define S3C_GPK4_DATA_CF4		(5)
#define S3C_GPK4_RESERVED2		(6)
#define S3C_GPK4_RESERVED3		(7)

#define S3C_GPK5       S3C_GPIONO(S3C_GPIO_BANKK, 5)
#define S3C_GPK5_INP			(0)
#define S3C_GPK5_OUTP			(1)
#define S3C_GPK5_HOSTIF_DATA5		(2)
#define S3C_GPK5_HSI_TX_WAKE		(3)
#define S3C_GPK5_RESERVED1		(4)
#define S3C_GPK5_DATA_CF5		(5)
#define S3C_GPK5_RESERVED2		(6)
#define S3C_GPK5_RESERVED3		(7)

#define S3C_GPK6       S3C_GPIONO(S3C_GPIO_BANKK, 6)
#define S3C_GPK6_INP			(0)
#define S3C_GPK6_OUTP			(1)
#define S3C_GPK6_HOSTIF_DATA6		(2)
#define S3C_GPK6_HSI_TX_FLAG		(3)
#define S3C_GPK6_RESERVED1		(4)
#define S3C_GPK6_DATA_CF6		(5)
#define S3C_GPK6_RESERVED2		(6)
#define S3C_GPK6_RESERVED3		(7)

#define S3C_GPK7       S3C_GPIONO(S3C_GPIO_BANKK, 7)
#define S3C_GPK7_INP			(0)
#define S3C_GPK7_OUTP			(1)
#define S3C_GPK7_HOSTIF_DATA7		(2)
#define S3C_GPK7_HSI_TX_DATA		(3)
#define S3C_GPK7_RESERVED1		(4)
#define S3C_GPK7_DATA_CF7		(5)
#define S3C_GPK7_RESERVED2		(6)
#define S3C_GPK7_RESERVED3		(7)

#define S3C_GPK8       S3C_GPIONO(S3C_GPIO_BANKK, 8)
#define S3C_GPK8_INP			(0)
#define S3C_GPK8_OUTP			(1)
#define S3C_GPK8_HOSTIF_DATA8		(2)
#define S3C_GPK8_KEYPAD_ROW0		(3)
#define S3C_GPK8_RESERVED1		(4)
#define S3C_GPK8_DATA_CF8		(5)
#define S3C_GPK8_RESERVED2			(6)
#define S3C_GPK8_RESERVED3		(7)

#define S3C_GPK9       S3C_GPIONO(S3C_GPIO_BANKK, 9)
#define S3C_GPK9_INP			(0)
#define S3C_GPK9_OUTP			(1)
#define S3C_GPK9_HOSTIF_DATA9		(2)
#define S3C_GPK9_KEYPAD_ROW1		(3)
#define S3C_GPK9_RESERVED1		(4)
#define S3C_GPK9_DATA_CF9		(5)
#define S3C_GPK9_RESERVED2			(6)
#define S3C_GPK9_RESERVED3		(7)

#define S3C_GPK10	S3C_GPIONO(S3C_GPIO_BANKK, 10)
#define S3C_GPK10_INP			(0)
#define S3C_GPK10_OUTP			(1)
#define S3C_GPK10_HOSTIF_DATA10 	(2)
#define S3C_GPK10_KEYPAD_ROW2		(3)
#define S3C_GPK10_RESERVED1		(4)
#define S3C_GPK10_DATA_CF9		(5)
#define S3C_GPK10_RESERVED2			(6)
#define S3C_GPK10_RESERVED3		(7)

#define S3C_GPK11	S3C_GPIONO(S3C_GPIO_BANKK, 11)
#define S3C_GPK11_INP			(0)
#define S3C_GPK11_OUTP			(1)
#define S3C_GPK11_HOSTIF_DATA11 	(2)
#define S3C_GPK11_KEYPAD_ROW3		(3)
#define S3C_GPK11_RESERVED1		(4)
#define S3C_GPK11_DATA_CF11		(5)
#define S3C_GPK11_RESERVED2			(6)
#define S3C_GPK11_RESERVED3		(7)

#define S3C_GPK12	S3C_GPIONO(S3C_GPIO_BANKK, 12)
#define S3C_GPK12_INP			(0)
#define S3C_GPK12_OUTP			(1)
#define S3C_GPK12_HOSTIF_DATA12 	(2)
#define S3C_GPK12_KEYPAD_ROW4		(3)
#define S3C_GPK12_RESERVED1		(4)
#define S3C_GPK12_DATA_CF12		(5)
#define S3C_GPK12_RESERVED2			(6)
#define S3C_GPK12_RESERVED3		(7)

#define S3C_GPK13	S3C_GPIONO(S3C_GPIO_BANKK, 13)
#define S3C_GPK13_INP			(0)
#define S3C_GPK13_OUTP			(1)
#define S3C_GPK13_HOSTIF_DATA13 	(2)
#define S3C_GPK13_KEYPAD_ROW5		(3)
#define S3C_GPK13_RESERVED1		(4)
#define S3C_GPK13_DATA_CF8		(5)
#define S3C_GPK13_RESERVED2			(6)
#define S3C_GPK13_RESERVED3		(7)

#define S3C_GPK14	S3C_GPIONO(S3C_GPIO_BANKK, 14)
#define S3C_GPK14_INP			(0)
#define S3C_GPK14_OUTP			(1)
#define S3C_GPK14_HOSTIF_DATA14 	(2)
#define S3C_GPK14_KEYPAD_ROW6		(3)
#define S3C_GPK14_RESERVED1		(4)
#define S3C_GPK14_DATA_CF14		(5)
#define S3C_GPK14_RESERVED2			(6)
#define S3C_GPK14_RESERVED3		(7)

#define S3C_GPK15	S3C_GPIONO(S3C_GPIO_BANKK, 15)
#define S3C_GPK15_INP			(0)
#define S3C_GPK15_OUTP			(1)
#define S3C_GPK15_HOSTIF_DATA15 	(2)
#define S3C_GPK15_KEYPAD_ROW7		(3)
#define S3C_GPK15_RESERVED1		(4)
#define S3C_GPK15_DATA_CF15		(5)
#define S3C_GPK15_RESERVED2			(6)
#define S3C_GPK15_RESERVED3		(7)


/* GPL : 16 in/out port . Host I/F, HSI, Key pad I/F */
#define S3C_GPLDAT	   S3C_GPIOREG(0x818)
#define S3C_GPL0CON	   S3C_GPIOREG(0x810)
#define S3C_GPL1CON	   S3C_GPIOREG(0x814)
#define S3C_GPLPU	   S3C_GPIOREG(0x81C)

#define S3C_GPL0       S3C_GPIONO(S3C_GPIO_BANKL, 0)
#define S3C_GPL0_INP			(0)
#define S3C_GPL0_OUTP			(1)
#define S3C_GPL0_HOSTIF_ADDR0		(2)
#define S3C_GPL0_KEYPAD_COL0		(3)
#define S3C_GPL0_RESERVED1		(4)
#define S3C_GPL0_RESERVED2		(5)
#define S3C_GPL0_ADDR_CF0		(6)
#define S3C_GPL0_OTG_ULPI_DATA0		(7)

#define S3C_GPL1       S3C_GPIONO(S3C_GPIO_BANKL, 1)
#define S3C_GPL1_INP			(0)
#define S3C_GPL1_OUTP			(1)
#define S3C_GPL1_HOSTIF_ADDR1		(2)
#define S3C_GPL1_KEYPAD_COL1		(3)
#define S3C_GPL1_RESERVED1		(4)
#define S3C_GPL1_RESERVED2		(5)
#define S3C_GPL1_ADDR_CF1		(6)
#define S3C_GPL1_OTG_ULPI_DATA1		(7)

#define S3C_GPL2       S3C_GPIONO(S3C_GPIO_BANKL, 2)
#define S3C_GPL2_INP			(0)
#define S3C_GPL2_OUTP			(1)
#define S3C_GPL2_HOSTIF_ADDR2		(2)
#define S3C_GPL2_KEYPAD_COL2		(3)
#define S3C_GPL2_RESERVED1		(4)
#define S3C_GPL2_RESERVED2		(5)
#define S3C_GPL2_ADDR_CF2		(6)
#define S3C_GPL2_OTG_ULPI_DATA2		(7)

#define S3C_GPL3       S3C_GPIONO(S3C_GPIO_BANKL, 3)
#define S3C_GPL3_INP			(0)
#define S3C_GPL3_OUTP			(1)
#define S3C_GPL3_HOSTIF_ADDR3		(2)
#define S3C_GPL3_KEYPAD_COL3		(3)
#define S3C_GPL3_RESERVED1		(4)
#define S3C_GPL3_RESERVED2		(5)
#define S3C_GPL3_RESERVED3		(6)
#define S3C_GPL3_OTG_ULPI_DATA3		(7)

#define S3C_GPL4       S3C_GPIONO(S3C_GPIO_BANKL, 4)
#define S3C_GPL4_INP			(0)
#define S3C_GPL4_OUTP			(1)
#define S3C_GPL4_HOSTIF_ADDR4		(2)
#define S3C_GPL4_KEYPAD_COL4		(3)
#define S3C_GPL4_RESERVED1		(4)
#define S3C_GPL4_RESERVED2		(5)
#define S3C_GPL4_RESERVED3		(6)
#define S3C_GPL4_OTG_ULPI_DATA4		(7)

#define S3C_GPL5       S3C_GPIONO(S3C_GPIO_BANKL, 5)
#define S3C_GPL5_INP			(0)
#define S3C_GPL5_OUTP			(1)
#define S3C_GPL5_HOSTIF_ADDR5		(2)
#define S3C_GPL5_KEYPAD_COL5		(3)
#define S3C_GPL5_RESERVED1		(4)
#define S3C_GPL5_RESERVED2		(5)
#define S3C_GPL5_RESERVED3		(6)
#define S3C_GPL5_OTG_ULPI_DATA5		(7)

#define S3C_GPL6       S3C_GPIONO(S3C_GPIO_BANKL, 6)
#define S3C_GPL6_INP			(0)
#define S3C_GPL6_OUTP			(1)
#define S3C_GPL6_HOSTIF_ADDR6		(2)
#define S3C_GPL6_KEYPAD_COL6		(3)
#define S3C_GPL6_RESERVED1		(4)
#define S3C_GPL6_RESERVED2		(5)
#define S3C_GPL6_RESERVED3		(6)
#define S3C_GPL6_OTG_ULPI_DATA6		(7)

#define S3C_GPL7       S3C_GPIONO(S3C_GPIO_BANKL, 7)
#define S3C_GPL7_INP			(0)
#define S3C_GPL7_OUTP			(1)
#define S3C_GPL7_HOSTIF_ADDR7		(2)
#define S3C_GPL7_KEYPAD_COL7		(3)
#define S3C_GPL7_RESERVED1		(4)
#define S3C_GPL7_RESERVED2		(5)
#define S3C_GPL7_RESERVED3		(6)
#define S3C_GPL7_OTG_ULPI_DATA		(7)

#define S3C_GPL8       S3C_GPIONO(S3C_GPIO_BANKL, 8)
#define S3C_GPL8_INP			(0)
#define S3C_GPL8_OUTP			(1)
#define S3C_GPL8_HOSTIF_ADDR8		(2)
#define S3C_GPL8_EXTINT16		(3)
#define S3C_GPL8_RESERVED1		(4)
#define S3C_GPL8_CE_CF0			(5)
#define S3C_GPL8_RESERVED2		(6)
#define S3C_GPL8_OTG_ULPI_STP		(7)

#define S3C_GPL9       S3C_GPIONO(S3C_GPIO_BANKL, 9)
#define S3C_GPL9_INP			(0)
#define S3C_GPL9_OUTP			(1)
#define S3C_GPL9_HOSTIF_ADDR9		(2)
#define S3C_GPL9_EXTINT17		(3)
#define S3C_GPL9_RESERVED1		(4)
#define S3C_GPL9_CE_CF1			(5)
#define S3C_GPL9_RESERVED2		(6)
#define S3C_GPL9_OTG_ULPI_CLK		(7)

#define S3C_GPL10	S3C_GPIONO(S3C_GPIO_BANKL, 10)
#define S3C_GPL10_INP			(0)
#define S3C_GPL10_OUTP			(1)
#define S3C_GPL10_SPI_CLK1	 	(2)
#define S3C_GPL10_EXTINT18		(3)
#define S3C_GPL10_RESERVED1		(4)
#define S3C_GPL10_IORD_CF		(5)
#define S3C_GPL10_RESERVED2		(6)
#define S3C_GPL10_OTG_ULPI_NXT		(7)

#define S3C_GPL11	S3C_GPIONO(S3C_GPIO_BANKL, 11)
#define S3C_GPL11_INP			(0)
#define S3C_GPL11_OUTP			(1)
#define S3C_GPL11_SPI_MOSI1	 	(2)
#define S3C_GPL11_EXTINT19		(3)
#define S3C_GPL11_RESERVED1		(4)
#define S3C_GPL11_IOWR_CF		(5)
#define S3C_GPL11_RESERVED2		(6)
#define S3C_GPL11_OTG_ULPI_DIR		(7)

#define S3C_GPL12	S3C_GPIONO(S3C_GPIO_BANKL, 12)
#define S3C_GPL12_INP			(0)
#define S3C_GPL12_OUTP			(1)
#define S3C_GPL12_SPI_MISO1	 	(2)
#define S3C_GPL12_EXTINT20		(3)
#define S3C_GPL12_RESERVED1		(4)
#define S3C_GPL12_IORDY_CF		(5)
#define S3C_GPL12_RESERVED2		(6)
#define S3C_GPL12_RESERVED3		(7)

#define S3C_GPL13	S3C_GPIONO(S3C_GPIO_BANKL, 13)
#define S3C_GPL13_INP			(0)
#define S3C_GPL13_OUTP			(1)
#define S3C_GPL14_nSS0 			(2)
#define S3C_GPL13_EXTINT21		(3)
#define S3C_GPL13_RESERVED1		(4)
#define S3C_GPL13_DATA_CF8		(5)
#define S3C_GPL13_RESERVED2		(6)
#define S3C_GPL13_RESERVED3		(7)

#define S3C_GPL14	S3C_GPIONO(S3C_GPIO_BANKL, 14)
#define S3C_GPL14_INP			(0)
#define S3C_GPL14_OUTP			(1)
#define S3C_GPL14_nSS1		 	(2)
#define S3C_GPL14_EXTINT22		(3)
#define S3C_GPL14_RESERVED1		(4)
#define S3C_GPL14_DATA_CF9		(5)
#define S3C_GPL14_RESERVED2		(6)
#define S3C_GPL14_RESERVED3		(7)


/* GPM : 6 in/out port . Host I/F, EINT */
#define S3C_GPMDAT	   S3C_GPIOREG(0x824)
#define S3C_GPMCON	   S3C_GPIOREG(0x820)
#define S3C_GPMPU	   S3C_GPIOREG(0x828)

#define S3C_GPM0       S3C_GPIONO(S3C_GPIO_BANKM, 0)
#define S3C_GPM0_INP			(0)
#define S3C_GPM0_OUTP			(1)
#define S3C_GPM0_HOSTIF_CS		(2)
#define S3C_GPM0_EXTINT23		(3)
#define S3C_GPM0_RESERVED1		(4)
#define S3C_GPM0_DATA_CF10		(5)
#define S3C_GPM0_CE_CF0			(6)
#define S3C_GPM0_RESERVED2		(7)

#define S3C_GPM1       S3C_GPIONO(S3C_GPIO_BANKM, 1)
#define S3C_GPM1_INP			(0)
#define S3C_GPM1_OUTP			(1)
#define S3C_GPM1_HOSTIF_CS_M		(2)
#define S3C_GPM1_EXTINT24		(3)
#define S3C_GPM1_RESERVED1		(4)
#define S3C_GPM1_DATA_CF11		(5)
#define S3C_GPM1_CE_CF1			(6)
#define S3C_GPM1_RESERVED2		(7)

#define S3C_GPM2       S3C_GPIONO(S3C_GPIO_BANKM, 2)
#define S3C_GPM2_INP			(0)
#define S3C_GPM2_OUTP			(1)
#define S3C_GPM2_HOST_IF_CS_S		(2)
#define S3C_GPM2_EXTINT25		(3)
#define S3C_GPM2_HOSTIF_MDP_VSYNC	(4)
#define S3C_GPM2_DATA_CF12		(5)
#define S3C_GPM2_IORD_CF		(6)
#define S3C_GPM2_RESERVED2		(7)

#define S3C_GPM3       S3C_GPIONO(S3C_GPIO_BANKM, 3)
#define S3C_GPM3_INP			(0)
#define S3C_GPM3_OUTP			(1)
#define S3C_GPM3_HOSTIF_WE		(2)
#define S3C_GPM3_EXTINT26		(3)
#define S3C_GPM3_RESERVED1		(4)
#define S3C_GPM3_DATA_CF13		(5)
#define S3C_GPM3_IOWR_CF		(6)
#define S3C_GPM3_RESERVED2		(7)

#define S3C_GPM4       S3C_GPIONO(S3C_GPIO_BANKM, 4)
#define S3C_GPM4_INP			(0)
#define S3C_GPM4_OUTP			(1)
#define S3C_GPM4_HOSTIF_OE		(2)
#define S3C_GPM4_EXTINT27		(3)
#define S3C_GPM4_RESERVED1		(4)
#define S3C_GPM4_DATA_CF14		(5)
#define S3C_GPM4_IORDY_CF		(6)
#define S3C_GPM4_RESERVED2		(7)

#define S3C_GPM5       S3C_GPIONO(S3C_GPIO_BANKM, 5)
#define S3C_GPM5_INP			(0)
#define S3C_GPM5_OUTP			(1)
#define S3C_GPM5_HOSTIF_INTR		(2)
#define S3C_GPM5_CF_DATA_DIR		(3)
#define S3C_GPM5_RESERVED1		(4)
#define S3C_GPM5_DATA_CF15		(5)
#define S3C_GPM5_RESERVED2		(6)
#define S3C_GPM5_RESERVED3		(7)


/* GPN : 16 in/out port . EINT */
#define S3C_GPNDAT	   S3C_GPIOREG(0x834)
#define S3C_GPNCON	   S3C_GPIOREG(0x830)
#define S3C_GPNPU	   S3C_GPIOREG(0x838)

#define S3C_GPN0       S3C_GPIONO(S3C_GPIO_BANKN, 0)
#define S3C_GPN0_INP			(0)
#define S3C_GPN0_OUTP			(1)
#define S3C_GPN0_EXTINT0		(2)
#define S3C_GPN0_KEYPAD_ROW0		(3)

#define S3C_GPN1       S3C_GPIONO(S3C_GPIO_BANKN, 1)
#define S3C_GPN1_INP			(0)
#define S3C_GPN1_OUTP			(1)
#define S3C_GPN1_EXTINT1		(2)
#define S3C_GPN1_KEYPAD_ROW1		(3)

#define S3C_GPN2       S3C_GPIONO(S3C_GPIO_BANKN, 2)
#define S3C_GPN2_INP			(0)
#define S3C_GPN2_OUTP			(1)
#define S3C_GPN2_EXTINT2		(2)
#define S3C_GPN2_KEYPAD_ROW2		(3)

#define S3C_GPN3       S3C_GPIONO(S3C_GPIO_BANKN, 3)
#define S3C_GPN3_INP			(0)
#define S3C_GPN3_OUTP			(1)
#define S3C_GPN3_EXTINT3		(2)
#define S3C_GPN3_KEYPAD_ROW3		(3)

#define S3C_GPN4       S3C_GPIONO(S3C_GPIO_BANKN, 4)
#define S3C_GPN4_INP			(0)
#define S3C_GPN4_OUTP			(1)
#define S3C_GPN4_EXTINT4		(2)
#define S3C_GPN4_KEYPAD_ROW4		(3)

#define S3C_GPN5       S3C_GPIONO(S3C_GPIO_BANKN, 5)
#define S3C_GPN5_INP			(0)
#define S3C_GPN5_OUTP			(1)
#define S3C_GPN5_EXTINT5		(2)
#define S3C_GPN5_KEYPAD_ROW5		(3)

#define S3C_GPN6       S3C_GPIONO(S3C_GPIO_BANKN, 6)
#define S3C_GPN6_INP			(0)
#define S3C_GPN6_OUTP			(1)
#define S3C_GPN6_EXTINT6		(2)
#define S3C_GPN6_KEYPAD_ROW6		(3)

#define S3C_GPN7       S3C_GPIONO(S3C_GPIO_BANKN, 7)
#define S3C_GPN7_INP			(0)
#define S3C_GPN7_OUTP			(1)
#define S3C_GPN7_EXTINT7		(2)
#define S3C_GPN7_KEYPAD_ROW7		(3)

#define S3C_GPN8       S3C_GPIONO(S3C_GPIO_BANKN, 8)
#define S3C_GPN8_INP			(0)
#define S3C_GPN8_OUTP			(1)
#define S3C_GPN8_EXTINT8		(2)
#define S3C_GPN8_ADDR_CF0		(3)

#define S3C_GPN9       S3C_GPIONO(S3C_GPIO_BANKN, 9)
#define S3C_GPN9_INP			(0)
#define S3C_GPN9_OUTP			(1)
#define S3C_GPN9_EXTINT9		(2)
#define S3C_GPN9_ADDR_CF1		(3)

#define S3C_GPN10	S3C_GPIONO(S3C_GPIO_BANKN, 10)
#define S3C_GPN10_INP			(0)
#define S3C_GPN10_OUTP			(1)
#define S3C_GPN10_EXTINT10		(2)
#define S3C_GPN10_ADDR_CF2		(3)

#define S3C_GPN11	S3C_GPIONO(S3C_GPIO_BANKN, 11)
#define S3C_GPN11_INP			(0)
#define S3C_GPN11_OUTP			(1)
#define S3C_GPN11_EXTINT11		(2)
#define S3C_GPN11_RESERVED		(3)

#define S3C_GPN12	S3C_GPIONO(S3C_GPIO_BANKN, 12)
#define S3C_GPN12_INP			(0)
#define S3C_GPN12_OUTP			(1)
#define S3C_GPN12_EXTINT12		(2)
#define S3C_GPN12_RESERVED		(3)

#define S3C_GPN13	S3C_GPIONO(S3C_GPIO_BANKN, 13)
#define S3C_GPN13_INP			(0)
#define S3C_GPN13_OUTP			(1)
#define S3C_GPN13_EXTINT13		(2)
#define S3C_GPN13_RESERVED		(3)

#define S3C_GPN14	S3C_GPIONO(S3C_GPIO_BANKN, 14)
#define S3C_GPN14_INP			(0)
#define S3C_GPN14_OUTP			(1)
#define S3C_GPN14_EXTINT14		(2)
#define S3C_GPN14_RESERVED		(3)

#define S3C_GPN15	S3C_GPIONO(S3C_GPIO_BANKN, 15)
#define S3C_GPN15_INP			(0)
#define S3C_GPN15_OUTP			(1)
#define S3C_GPN15_EXTINT15		(2)
#define S3C_GPN15_RESERVED		(3)


/* GPO : 16 in/out port . Memory Port 0 */
#define S3C_GPODAT	   S3C_GPIOREG(0x144)
#define S3C_GPOCON	   S3C_GPIOREG(0x140)
#define S3C_GPOPU	   S3C_GPIOREG(0x148)

/* GPP : 15 in/out port . Memory Port 0 */
#define S3C_GPPDAT	   S3C_GPIOREG(0x164)
#define S3C_GPPCON	   S3C_GPIOREG(0x160)
#define S3C_GPPPU	   S3C_GPIOREG(0x168)

/* GPQ : 9 in/out port . Memory Port 0 Dram part */
#define S3C_GPQDAT	   S3C_GPIOREG(0x184)
#define S3C_GPQCON	   S3C_GPIOREG(0x180)
#define S3C_GPQPU	   S3C_GPIOREG(0x188)


#define S3C_MEM0CONSTOP		S3C_GPIOREG(0x1B0)
#define S3C_MEM1CONSTOP		S3C_GPIOREG(0x1B4)
#define S3C_MEM0CONSLP0		S3C_GPIOREG(0x1C0)
#define S3C_MEM0CONSLP1		S3C_GPIOREG(0x1C4)
#define S3C_MEM1CONSLP		S3C_GPIOREG(0x1C8)
#define S3C_MEM0DRVCON		S3C_GPIOREG(0x1D0)
#define S3C_MEM1DRVCON		S3C_GPIOREG(0x1D4)

#define S3C_EINTCON0	S3C_GPIOREG(0x900)
#define S3C_EINTCON1	S3C_GPIOREG(0x904)
#define S3C_EINTFLTCON0	S3C_GPIOREG(0x910)
#define S3C_EINTFLTCON1	S3C_GPIOREG(0x914)
#define S3C_EINTFLTCON2	S3C_GPIOREG(0x918)
#define S3C_EINTFLTCON3	S3C_GPIOREG(0x91C)
#define S3C_EINTMASK	S3C_GPIOREG(0x920)
#define S3C_EINTPEND   S3C_GPIOREG(0x924)  /*  External Interrupt Pending Register   */
#define S3C_EINT12CON	S3C_GPIOREG(0x200)  /*	External Interrupt 1,2 Configuration Register	*/
#define S3C_EINT34CON	S3C_GPIOREG(0x204)  /*	External Interrupt 3,4 Configuration Register	*/
#define S3C_EINT56CON	S3C_GPIOREG(0x208)  /*	External Interrupt 5,6 Configuration Register	*/
#define S3C_EINT78CON	S3C_GPIOREG(0x20C)  /*	External Interrupt 7,8 Configuration Register	*/
#define S3C_EINT9CON	S3C_GPIOREG(0x210)  /*	External Interrupt 9 Configuration Register */
#define S3C_EINT12FLTCON    S3C_GPIOREG(0x220)	/*  External Interrupt 1,2 Filter Control Register  */
#define S3C_EINT34FLTCON    S3C_GPIOREG(0x224)	/*  External Interrupt 3,4 Filter Control Register  */
#define S3C_EINT56FLTCON    S3C_GPIOREG(0x228)	/*  External Interrupt 5,6 Filter Control Register  */
#define S3C_EINT78FLTCON    S3C_GPIOREG(0x22C)	/*  External Interrupt 7,8 Filter Control Register  */
#define S3C_EINT9FLTCON S3C_GPIOREG(0x230)  /*	External Interrupt 9 Filter Control Register	*/
#define S3C_EINT12MASK	S3C_GPIOREG(0x240)  /*	External Interrupt 1,2 Mask Register	*/
#define S3C_EINT34MASK	S3C_GPIOREG(0x244)  /*	External Interrupt 3,4 Mask Register	*/
#define S3C_EINT56MASK	S3C_GPIOREG(0x248)  /*	External Interrupt 5,6 Mask Register	*/
#define S3C_EINT78MASK	S3C_GPIOREG(0x24C)  /*	External Interrupt 7,8 Mask Register	*/
#define S3C_EINT9MASK	S3C_GPIOREG(0x250) /*  External Interrupt 9 Mask Register  */
#define S3C_EINT12PEND	S3C_GPIOREG(0x260)  /*	External Interrupt 1,2 Pending Register */
#define S3C_EINT34PEND	S3C_GPIOREG(0x264)  /*	External Interrupt 3,4 Pending Register */
#define S3C_EINT56PEND	S3C_GPIOREG(0x268)  /*	External Interrupt 5,6 Pending Register */
#define S3C_EINT78PEND	S3C_GPIOREG(0x26C)  /*	External Interrupt 7,8 Pending Register */
#define S3C_EINT9PEND	S3C_GPIOREG(0x270)  /*	External Interrupt 9 Pending Register	*/
#define S3C_PRIORITY	S3C_GPIOREG(0x280)  /*	Priority Control Register   */
#define S3C_SERVICE S3C_GPIOREG(0x284)	/*  Current Service Register	*/
#define S3C_SERVICEPEND S3C_GPIOREG(0x288)  /*	Current Service Pending Register    */

/* values for S3C_EXTINT0 */
#define S3C_EXTINT_LOWLEV	 (0x00)
#define S3C_EXTINT_HILEV	 (0x01)
#define S3C_EXTINT_FALLEDGE	 (0x02)
#define S3C_EXTINT_RISEEDGE	 (0x04)
#define S3C_EXTINT_BOTHEDGE	 (0x06)

/* Special port control register*/
#define S3C_SPCON	   S3C_GPIOREG(0x1A0)
#define S3C_SPONSLP		S3C_GPIOREG(0x880)
#define S3C_SLPEN		S3C_GPIOREG(0x934)

#endif
#endif	/* __ASM_ARCH_REGS_GPIO_H */

