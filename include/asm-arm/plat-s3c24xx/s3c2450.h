/* linux/include/asm-arm/plat-s3c24xx/s3c2450.h
 *
 * Copyright (c) 2008 Samsung Electronics
 *	Ryu Euiyoul <ryu.real@gmail.com>
 *
 * Header file for s3c2450 cpu support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#if defined(CONFIG_CPU_S3C2450)

struct s3c2410_uartcfg;

extern  int s3c2450_init(void);

extern void s3c2450_map_io(struct map_desc *mach_desc, int size);

extern void s3c2450_init_uarts(struct s3c2410_uartcfg *cfg, int no);

extern void s3c2450_init_clocks(int xtal);

#else
#define s3c2450_init_clocks NULL
#define s3c2450_init_uarts NULL
#define s3c2450_map_io NULL
#define s3c2450_init NULL
#endif
