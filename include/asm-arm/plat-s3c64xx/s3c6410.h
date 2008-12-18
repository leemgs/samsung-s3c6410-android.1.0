/* arch/arm/plat-s3c64xx/s3c64.h
 *
 * Copyright (c) 2004-2005 Samsung Electronics
 *	
 *
 * Header file for s3c6410 cpu support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Modifications:
*/

#if defined(CONFIG_CPU_S3C6410)

extern  int s3c6410_init(void);

extern void s3c6410_map_io(struct map_desc *mach_desc, int size);

extern void s3c6410_init_uarts(struct s3c2410_uartcfg *cfg, int no);

extern void s3c6410_init_clocks(int xtal);

#else
#define s3c6410_init NULL
#define s3c6410_map_io NULL
#define s3c6410_init_uarts NULL
#define s3c6410_init_clocks NULL
#endif

