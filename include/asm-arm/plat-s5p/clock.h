/* linux/include/asm-arm/plat-s3c24xx/clock.h
 * linux/arch/arm/mach-s3c2410/clock.h
 *
 * Copyright (c) 2004-2005 Simtec Electronics
 *	http://www.simtec.co.uk/products/SWLINUX/
 *	Written by Ben Dooks, <ben@simtec.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

struct clk {
	struct list_head      list;
	struct module        *owner;
	struct clk           *parent;
	const char           *name;
	int		      id;
	int		      usage;
	unsigned long         rate;
	unsigned long         ctrlbit;

	int		    (*enable)(struct clk *, int enable);
	int		    (*set_rate)(struct clk *c, unsigned long rate);
	unsigned long	    (*get_rate)(struct clk *c);
	unsigned long	    (*round_rate)(struct clk *c, unsigned long rate);
	int		    (*set_parent)(struct clk *c, struct clk *parent);
};

/* exports for arch/arm/mach-s3c2410
 *
 * Please DO NOT use these outside of arch/arm/mach-s3c2410
 */
void s3c_clk_enable (uint clocks, uint enable, ulong gate_reg);

extern int s5pc1xx_register_clock(struct clk *clk);
extern int s5pc1xx_register_clocks(struct clk **clk, int nr_clks);

extern int s5pc1xx_setup_clocks(unsigned long xtal,
				unsigned long armclk,
				unsigned long hclkd0,
				unsigned long pclkd0,
				unsigned long hclkd1,
				unsigned long pclkd1);

extern struct mutex clocks_mutex;

/* core clock support */

extern struct clk clk_xtal;
extern struct clk clk_arm;
extern struct clk clk_hd0;
extern struct clk clk_pd0;
extern struct clk clk_hd1;
extern struct clk clk_pd1;

#if 0
extern struct clk clk_epll;

extern struct clk clk_48m;
extern struct clk clk_s;
extern struct clk clk_u;
extern struct clk clk_27m;
extern struct clk clk_hx2;
#endif


