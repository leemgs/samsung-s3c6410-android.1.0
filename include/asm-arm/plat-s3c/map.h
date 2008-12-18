/* linux/include/asm-arm/plat-s3c/map.h
 *
 * Copyright 2003, 2007 Simtec Electronics
 *	http://armlinux.simtec.co.uk/
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * S3C - Memory map definitions (virtual addresses)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_PLAT_MAP_H
#define __ASM_PLAT_MAP_H __FILE__

/* Fit all our registers in at 0xF4000000 upwards, trying to use as
 * little of the VA space as possible so vmalloc and friends have a
 * better chance of getting memory.
 *
 * we try to ensure stuff like the IRQ registers are available for
 * an single MOVS instruction (ie, only 8 bits of set data)
 */

#define S3C_ADDR_BASE	(0xF4000000)

#ifndef __ASSEMBLY__
#define S3C_ADDR(x)	((void __iomem __force *)S3C_ADDR_BASE + (x))
#else
#define S3C_ADDR(x)	(S3C_ADDR_BASE + (x))
#endif

#if defined(CONFIG_PLAT_S3C64XX) || defined(CONFIG_PLAT_S5PC1XX)
#define S3C_VA_IRQ	S3C_ADDR(0x00700000)	/* irq controller(s) in size of 2 or 4 MB */
#else
#define S3C_VA_IRQ	S3C_ADDR(0x00000000)	/* irq controller(s) */
#endif

#define S3C_VA_SYS	S3C_ADDR(0x00100000)	/* system control */
#define S3C_VA_MEM	S3C_ADDR(0x00200000)	/* mem control */
#define S3C_VA_TIMER	S3C_ADDR(0x00300000)	/* timer block */
#define S3C_VA_WATCHDOG	S3C_ADDR(0x00400000)	/* watchdog */
#define S3C_VA_UART	S3C_ADDR(0x01000000)	/* UART */

/* For S3C2450/S3C6400/S3C6410 */
#define S3C_VA_LCD	S3C_ADDR(0x00500000)	/* LCD */
#define S3C_VA_GPIO	S3C_ADDR(0x00600000)	/* GPIO */
#define S3C_VA_HOSTIFA	S3C_ADDR(0x00B00000)	/* Host I/F Indirect & Direct */
#define S3C_VA_HOSTIFB	S3C_ADDR(0x00C00000)	/* Host I/F Indirect & Direct */
#define S3C_VA_SYSCON	S3C_ADDR(0x02800000)	/* System Controller  */
#define S3C_VA_CS8900	S3C_ADDR(0x03600000)	/* Ethernet  */
#define S3C_VA_SROMC	S3C_ADDR(0x03700000)	/* SROM SFR */
#define S3C_VA_DMC0	S3C_ADDR(0x01f00000)	/* DRAM controller 0  */
#if !defined(CONFIG_PLAT_S5PC1XX)
#define S3C_VA_DMC1	S3C_ADDR(0x03800000)	/* DRAM controller 1  */
#endif
#define S3C_VA_OTG	S3C_ADDR(0x03900000)	/* USB OTG */
#define S3C_VA_OTGSFR	S3C_ADDR(0x03a00000)	/* USB OTG SFR */
#define S3C_VA_EBI	S3C_ADDR(0x03c00000)	/* EBI registers */
#define S3C_VA_USBDEV	S3C_ADDR(0x03d00000)	/* EBI registers */

#define S3C_VA_CHIP_ID	S3C_ADDR(0x03f00000)	/* Chip ID */
#endif /* __ASM_PLAT_MAP_H */
