/* linux/include/asm/arch-s3c2460/regs-keyif.h
 *
 * $Id: regs-keypad.h,v 1.2 2008/02/21 11:18:55 jsgood Exp $
 *
 * S3C2460 Key Interface register definitions
 * 
 * Copyright (C) 2005, Sean Choi <sh428.choi@samsung.com>
 * All rights reserved.
 *   
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *   
 */


#ifndef __ASM_ARCH_REGS_KEYPAD_H
#define __ASM_ARCH_REGS_KEYPAD_H

/* 
 * Keypad Interface
 */
#define S3C_KEYPADREG(x)	(x)
//#define S3C_KEYPADREG(x)	((x) + S3C24XX_VA_KEYPAD)

#define S3C_KEYIFCON		S3C_KEYPADREG(0x00)
#define S3C_KEYIFSTSCLR		S3C_KEYPADREG(0x04)
#define S3C_KEYIFCOL		S3C_KEYPADREG(0x08)
#define S3C_KEYIFROW		S3C_KEYPADREG(0x0C)
#define S3C_KEYIFFC		S3C_KEYPADREG(0x10)

#if defined(CONFIG_CPU_S3C6400) || defined(CONFIG_CPU_S3C6410)
#define KEYCOL_DMASK		(0xff)
#define KEYROW_DMASK		(0xff)
#define	INT_F_EN		(1<<0)	/*falling edge(key-pressed) interuppt enable*/
#define	INT_R_EN		(1<<1)	/*rising edge(key-released) interuppt enable*/
#define	DF_EN			(1<<2)	/*debouncing filter enable*/
#define	FC_EN			(1<<3)	/*filter clock enable*/
#define	KEYIFCON_INIT		(KEYIFCON_CLEAR |INT_F_EN|INT_R_EN|DF_EN|FC_EN)
#define KEYIFSTSCLR_CLEAR	(0xffff)

#else
#error "Not supported S3C Configuration!!"
#endif

#endif /* __ASM_ARCH_REGS_KEYPAD_H */


