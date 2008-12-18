/* linux/arch/arm/mach-s3c6410/pm.h
 *
 * Copyright (c) 2006 Samsung Electronics
 *      Ryu Euiyoul <steven.ryu@samsung.com>
 *
 * Copyright (c) 2004 Simtec Electronics
 *	Written by Ben Dooks, <ben@simtec.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

/* s3c6410_pm_init
 *
 * called from board at initialisation time to setup the power
 * management
*/

#ifdef CONFIG_PM

extern __init int s3c6410_pm_init(void);

#else

static inline int s3c6410_pm_init(void)
{
	return 0;
}
#endif


/* Flags for PM Control */

extern unsigned long s3c_pm_flags;

/* from sleep.S */

extern void s3c6410_cpu_suspend(unsigned long *saveblk);
extern void s3c6410_cpu_resume(void);

extern unsigned long s3c6410_sleep_save_phys;

/* sleep save info */

struct sleep_save {
	unsigned long	reg;
	unsigned long	val;
};

#define SAVE_ITEM(x) \
	{ .reg = (unsigned long)(x) }

extern void s3c6410_pm_do_save(struct sleep_save *ptr, int count);
extern void s3c6410_pm_do_restore(struct sleep_save *ptr, int count);
