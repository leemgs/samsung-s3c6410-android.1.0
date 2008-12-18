/* linux/include/asm-arm/arch-s3c2410/timex.h
 *
 * Copyright (c) 2003-2005 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * S3C2410 - time parameters
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARCH_TIMEX_H
#define __ASM_ARCH_TIMEX_H

/* CLOCK_TICK_RATE needs to be evaluatable by the cpp, so making it
 * a variable is useless. It seems as long as we make our timers an
 * exact multiple of HZ, any value that makes a 1->1 correspondence
 * for the time conversion functions to/from jiffies is acceptable.
*/
#if defined (CONFIG_PLAT_S3C64XX) || defined (CONFIG_PLAT_S5PC1XX) 

#define PRESCALER 4
#define DIVIDER 1

#define PCLK_INPUT	51000000
/* CLOCK_TICK_RATE is the time of a timer count desence, minsung says*/
#define CLOCK_TICK_RATE  (PCLK_INPUT / ((PRESCALER + 1) * DIVIDER))


#else

#define CLOCK_TICK_RATE 12000000

#endif

#endif /* __ASM_ARCH_TIMEX_H */
