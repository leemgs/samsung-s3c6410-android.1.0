/* arch/arm/plat-s3c/include/plat/ts.h
 *
 * Copyright (c) 2004 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARCH_TS_H
#define __ASM_ARCH_TS_H __FILE__

struct s3c_ts_mach_info {
       int             delay;
       int             presc;
       int             oversampling_shift;
       int	       resol_bit;
};

struct s3c_ts {
	struct input_dev *dev;
	long xp;
	long yp;
	int count;
	int shift;
	char phys[32];
};


#endif /* __ASM_ARCH_TS_H */
