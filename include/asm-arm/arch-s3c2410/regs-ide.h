/* linux/include/asm-arm/arch-s3c2410/regs-nand.h
 *
 * Copyright (c) 2004,2005 Simtec Electronics <linux@simtec.co.uk>
 *		      http://www.simtec.co.uk/products/SWLINUX/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * S3C2410 NAND register definitions
*/

#ifndef __ASM_ARM_REGS_IDE
#define __ASM_ARM_REGS_IDE "$Id: "


#define S3C_CFATA_REG(x) (x)

#define S3C_CFATA_MUX		S3C_CFATA_REG(0x1800)
#define S3C_CF_CFG		S3C_CFATA_REG(0x1820)
#define S3C_CF_INTMSK		S3C_CFATA_REG(0x1824)
#define S3C_CF_ATTR		S3C_CFATA_REG(0x1828)
#define S3C_CF_IO		S3C_CFATA_REG(0x182c)
#define S3C_CF_COMM		S3C_CFATA_REG(0x1830)

#define S3C_ATA_CTRL		S3C_CFATA_REG(0x1900)
#define S3C_ATA_STATUS		S3C_CFATA_REG(0x1904)
#define S3C_ATA_CMD		S3C_CFATA_REG(0x1908)
#define S3C_ATA_SWRST		S3C_CFATA_REG(0x190c)
#define S3C_ATA_IRQ		S3C_CFATA_REG(0x1910)
#define S3C_ATA_IRQ_MSK		S3C_CFATA_REG(0x1914)
#define S3C_ATA_CFG		S3C_CFATA_REG(0x1918)

#define S3C_ATA_PIO_TIME	S3C_CFATA_REG(0x192c)
#define S3C_ATA_UDMA_TIME	S3C_CFATA_REG(0x1930)
#define S3C_ATA_XFR_NUM		S3C_CFATA_REG(0x1934)
#define S3C_ATA_XFR_CNT		S3C_CFATA_REG(0x1938)
#define S3C_ATA_TBUF_START	S3C_CFATA_REG(0x193c)
#define S3C_ATA_TBUF_SIZE	S3C_CFATA_REG(0x1940)
#define S3C_ATA_SBUF_START	S3C_CFATA_REG(0x1944)
#define S3C_ATA_SBUF_SIZE	S3C_CFATA_REG(0x1948)
#define S3C_ATA_CADR_TBUF	S3C_CFATA_REG(0x194c)
#define S3C_ATA_CADR_SBUF	S3C_CFATA_REG(0x1950)
#define S3C_ATA_PIO_DTR		S3C_CFATA_REG(0x1954)
#define S3C_ATA_PIO_FED		S3C_CFATA_REG(0x1958)
#define S3C_ATA_PIO_SCR		S3C_CFATA_REG(0x195c)
#define S3C_ATA_PIO_LLR		S3C_CFATA_REG(0x1960)
#define S3C_ATA_PIO_LMR		S3C_CFATA_REG(0x1964)
#define S3C_ATA_PIO_LHR		S3C_CFATA_REG(0x1968)
#define S3C_ATA_PIO_DVR		S3C_CFATA_REG(0x196c)
#define S3C_ATA_PIO_CSD		S3C_CFATA_REG(0x1970)
#define S3C_ATA_PIO_DAD		S3C_CFATA_REG(0x1974)
#define S3C_ATA_PIO_RDATA	S3C_CFATA_REG(0x197c)
#define S3C_BUS_FIFO_STATUS	S3C_CFATA_REG(0x1990)
#define S3C_ATA_FIFO_STATUS	S3C_CFATA_REG(0x1994)

#endif /* __ASM_ARM_REGS_NAND */

