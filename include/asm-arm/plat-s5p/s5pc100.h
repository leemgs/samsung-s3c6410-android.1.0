/* arch/arm/plat-s5pc100/s3c64.h
 *
 * Copyright (c) 2004-2005 Samsung Electronics
 *	
 *
 * Header file for s5pc100 cpu support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Modifications:
*/

#if defined(CONFIG_CPU_S5PC100)

extern  int s5pc100_init(void);

extern void s5pc100_map_io(struct map_desc *mach_desc, int size);

extern void s5pc100_init_uarts(struct s3c2410_uartcfg *cfg, int no);

extern void s5pc100_init_clocks(int xtal);

#else
#define s5pc100_init NULL
#define s5pc100_map_io NULL
#define s5pc100_init_uarts NULL
#define s5pc100_init_clocks NULL
#endif

