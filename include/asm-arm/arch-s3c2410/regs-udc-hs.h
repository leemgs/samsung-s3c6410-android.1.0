/*
 * include/asm/arch-s3c2410/regs-udc-hs.h
 *
 * $Id: regs-udc-hs.h,v 1.2 2008/03/26 07:14:30 ihlee215 Exp $*
 *
 * Copyright (C) 2007 for Samsung Electronics 
 * 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef __ASM_ARCH_REGS_UDC_HS_H
#define __ASM_ARCH_REGS_UDC_HS_H


#if defined(CONFIG_CPU_S3C2443) || defined(CONFIG_CPU_S3C2450) || defined(CONFIG_CPU_S3C2416)
#define S3C_PWRCFG		S3C2443_PWRCFG
#define S3C_RSTCON		S3C2443_RSTCON
#define S3C_RSTSTAT		S3C2443_RSTSTAT
#define S3C_PHYCTRL		S3C2443_PHYCTRL
#define S3C_PHYPWR		S3C2443_PHYPWR
#define S3C_URSTCON		S3C2443_URSTCON
#define S3C_UCLKCON		S3C2443_UCLKCON
#endif

/* USB2.0 Device Controller register */
#define S3C_USBDREG(x) ((x) + S3C24XX_VA_USBDEV)

/* Non-Indexed Registers */
#define S3C_UDC_INDEX_REG		S3C_USBDREG(0x00) /* Index register */
#define S3C_UDC_EP_INT_REG		S3C_USBDREG(0x04) /* EP Interrupt pending and clear */
#define S3C_UDC_EP_INT_EN_REG		S3C_USBDREG(0x08) /* EP Interrupt enable */
#define S3C_UDC_FUNC_ADDR_REG		S3C_USBDREG(0x0c) /* Function address */
#define S3C_UDC_FRAME_NUM_REG		S3C_USBDREG(0x10) /* Frame number */
#define S3C_UDC_EP_DIR_REG		S3C_USBDREG(0x14) /* Endpoint direction */
#define S3C_UDC_TEST_REG		S3C_USBDREG(0x18) /* Test register */
#define S3C_UDC_SYS_STATUS_REG		S3C_USBDREG(0x1c) /* System status */
#define S3C_UDC_SYS_CON_REG		S3C_USBDREG(0x20) /* System control */
#define S3C_UDC_EP0_STATUS_REG		S3C_USBDREG(0x24) /* Endpoint 0 status */
#define S3C_UDC_EP0_CON_REG		S3C_USBDREG(0x28) /* Endpoint 0 control */
#define S3C_UDC_EP0_FIFO_REG		S3C_USBDREG(0x60) /* Endpoint 0 Buffer */
#define S3C_UDC_EP1_FIFO_REG		S3C_USBDREG(0x64) /* Endpoint 1 Buffer */
#define S3C_UDC_EP2_FIFO_REG		S3C_USBDREG(0x68) /* Endpoint 2 Buffer */
#define S3C_UDC_EP3_FIFO_REG		S3C_USBDREG(0x6c) /* Endpoint 3 Buffer */
#define S3C_UDC_EP4_FIFO_REG		S3C_USBDREG(0x70) /* Endpoint 4 Buffer */
#define S3C_UDC_EP5_FIFO_REG		S3C_USBDREG(0x74) /* Endpoint 5 Buffer */
#define S3C_UDC_EP6_FIFO_REG		S3C_USBDREG(0x78) /* Endpoint 6 Buffer */
#define S3C_UDC_EP7_FIFO_REG		S3C_USBDREG(0x7c) /* Endpoint 7 Buffer */
#define S3C_UDC_EP8_FIFO_REG		S3C_USBDREG(0x80) /* Endpoint 8 Buffer */
#define S3C_UDC_FIFO_CON_REG		S3C_USBDREG(0x100) /* Burst FIFO-DMA Control */
#define S3C_UDC_FIFO_STATUS_REG		S3C_USBDREG(0x104) /* Burst FIFO Status */

/* Indexed Registers */
#define S3C_UDC_EP_STATUS_REG		S3C_USBDREG(0x2c) /* Endpoints status */
#define S3C_UDC_EP_CON_REG		S3C_USBDREG(0x30) /* Endpoints control */
#define S3C_UDC_BYTE_READ_CNT_REG	S3C_USBDREG(0x34) /* Byte read count */
#define S3C_UDC_BYTE_WRITE_CNT_REG	S3C_USBDREG(0x38) /* Byte write count */
#define S3C_UDC_MAXP_REG		S3C_USBDREG(0x3c) /* Max packet size */
#define S3C_UDC_DMA_CON_REG		S3C_USBDREG(0x40) /* DMA control */
#define S3C_UDC_DMA_CNT_REG		S3C_USBDREG(0x44) /* DMA count */
#define S3C_UDC_DMA_FIFO_CNT_REG	S3C_USBDREG(0x48) /* DMA FIFO count */
#define S3C_UDC_DMA_TOTAL_CNT1_REG	S3C_USBDREG(0x4c) /* DMA Total Transfer count1 */
#define S3C_UDC_DMA_TOTAL_CNT2_REG	S3C_USBDREG(0x50) /* DMA Total Transfer count2 */
#define S3C_UDC_DMA_IF_CON_REG		S3C_USBDREG(0x84) /* DMA interface Control */
#define S3C_UDC_DMA_MEM_BASE_ADDR_REG	S3C_USBDREG(0x88) /* Mem Base Addr */
#define S3C_UDC_DMA_MEM_CURRENT_ADDR_REG	S3C_USBDREG(0x8c) /* Mem current Addr */

#define S3C_UDC_FCON_REG	S3C_USBDREG(0x100) /* Mem current Addr */
#define S3C_UDC_FSTAT_REG	S3C_USBDREG(0x104) /* Mem current Addr */


/* EP interrupt register Bits */
#define S3C_UDC_INT_EP3		(1<<3) // R/C
#define S3C_UDC_INT_EP2		(1<<2) // R/C
#define S3C_UDC_INT_EP1		(1<<1) // R/C
#define S3C_UDC_INT_EP0		(1<<0) // R/C

/* System status register Bits */
#define S3C_UDC_INT_CHECK	(0xff8f)
#define S3C_UDC_INT_ERR		(0xff80) // R/C
#define S3C_UDC_INT_VBUSON	(1<<8) // R/C
#define S3C_UDC_INT_HSP		(1<<4) // R
#define S3C_UDC_INT_SDE		(1<<3) // R/C
#define S3C_UDC_INT_RESUME	(1<<2) // R/C
#define S3C_UDC_INT_SUSPEND	(1<<1) // R/C
#define S3C_UDC_INT_RESET	(1<<0) // R/C

/* system control register Bits */
#define S3C_UDC_DTZIEN_EN	(1<<14)
#define S3C_UDC_RRD_EN		(1<<5)
#define S3C_UDC_SUS_EN		(1<<1)
#define S3C_UDC_RST_EN		(1<<0)

/* EP0 status register Bits */
#define S3C_UDC_EP0_LWO		(1<<6)
#define S3C_UDC_EP0_STALL	(1<<4)
#define S3C_UDC_EP0_TX_SUCCESS	(1<<1)
#define S3C_UDC_EP0_RX_SUCCESS	(1<<0)

/* EP status register Bits */
#define S3C_UDC_EP_FPID		(1<<11)
#define S3C_UDC_EP_OSD 		(1<<10)
#define S3C_UDC_EP_DTCZ		(1<<9)
#define S3C_UDC_EP_SPT		(1<<8)

#define S3C_UDC_EP_DOM		(1<<7)
#define S3C_UDC_EP_FIFO_FLUSH	(1<<6)
#define S3C_UDC_EP_STALL	(1<<5)
#define S3C_UDC_EP_LWO		(1<<4)
#define S3C_UDC_EP_PSIF_ONE	(1<<2)
#define S3C_UDC_EP_PSIF_TWO	(2<<2)
#define S3C_UDC_EP_TX_SUCCESS	(1<<1)
#define S3C_UDC_EP_RX_SUCCESS	(1<<0)

// USB2.0 DMA Operation
#define S3C_DMA_AUTO_RX_ENABLE          (0x0<<5)
#define S3C_DMA_AUTO_RX_DISABLE         (0x1<<5)
#define S3C_DMA_FLY_ENABLE              (0x1<<4)
#define S3C_DMA_FLY_DISABLE             (0x0<<4)
#define S3C_DMA_DEMEND_ENABLE           (0x1<<3)
#define S3C_DMA_DEMEND_DISABLE          (0x0<<3)
#define S3C_DMA_TX_START                (0x1<<2)
#define S3C_DMA_TX_STOP                 (0x0<<2)
#define S3C_DMA_RX_START                (0x1<<1)
#define S3C_DMA_RX_STOP                 (0x0<<1)
#define S3C_USB_DMA_MODE                (0x1<<0)
#define S3C_USB_INT_MODE                (0x0<<0)

#define S3C_MAX_BURST_INCR16            (0x3<<0)
#define S3C_MAX_BURST_INCR8             (0x2<<0)
#define S3C_MAX_BURST_INCR4             (0x1<<0)
#define S3C_MAX_BURST_SINGLE            (0x0<<0)

#define S3C_DMA_ENABLE                  (0x1<<8)
#define S3C_DMA_DISABLE                 (0x0<<8)
#define S3C_DMA_TFCLR                   (0x1<<4)
#define S3C_DMA_RFCLR                   (0x1<<0)

#endif

