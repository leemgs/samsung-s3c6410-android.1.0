/* linux/arch/arm/mach-s3c64xx/time.c
 *
 * Copyright (C) 2006 Samsung Electronics
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
 */

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/err.h>
#include <linux/clk.h>

#include <asm/system.h>
#include <asm/leds.h>
#include <asm/mach-types.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/arch/map.h>
#include <asm/plat-s3c/regs-timer.h>
#include <asm/arch/regs-irq.h>
#include <asm/arch/regs-gpio.h>
#include <asm/mach/time.h>

#include <asm/plat-s3c24xx/clock.h>
#include <asm/plat-s3c24xx/cpu.h>

static unsigned long timer_startval;
static unsigned long timer_usec_ticks;
static unsigned long max_buffer_cnt;
unsigned long timerclock;
unsigned long s3c_latch;

//#define T32_PROBE_TEST

#define TIMER_USEC_SHIFT 16
#define DEFAULT_TIMER_VAL 0xffffffff
#define REPROGRAM_SET_FLAG 1
#define REPROGRAM_UNSET_FLAG 0

/* Global variable for Dynamic Tick */

#ifdef CONFIG_NO_IDLE_HZ
static unsigned long initial_match;
static int match_postponed;
#endif

/* we use the shifted arithmetic to work out the ratio of timer ticks
 * to usecs, as often the peripheral clock is not a nice even multiple
 * of 1MHz.
 *
 * shift of 14 and 15 are too low for the 12MHz, 16 seems to be ok
 * for the current HZ value of 200 without producing overflows.
 *
 * Original patch by Dimitry Andric, updated by Ben Dooks
*/

static inline unsigned long
timer_mask_usec_ticks(unsigned long scaler, unsigned long pclk)
{
	unsigned long den = pclk / 1000;

	return ((1000 << TIMER_USEC_SHIFT) * scaler + (den >> 1)) / den;
}

static inline unsigned long timer_ticks_to_usec(unsigned long ticks)
{
	unsigned long res;

	res = ticks * timer_usec_ticks;
	res += 1 << (TIMER_USEC_SHIFT - 4);	/* round up slightly */

	return res >> TIMER_USEC_SHIFT;
}

static unsigned long s3c_gettimeoffset (void)
{
	unsigned long timer_val;
	unsigned long tdone;
	timer_val =  __raw_readl(S3C_TCNTO(4));
	tdone = timer_startval - timer_val;

	return timer_ticks_to_usec(tdone);
}

static inline irqreturn_t _s3c_timer_interrupt(int irq, void *dev_id)
{
	write_seqlock(&xtime_lock);
#ifdef CONFIG_NO_IDLE_HZ
	if(match_postponed){
		match_postponed = REPROGRAM_UNSET_FLAG;
		__raw_writel(initial_match, S3C_TCNTB(4));
	}
#endif
	timer_tick();
	write_sequnlock(&xtime_lock);

	return IRQ_HANDLED;
}

static irqreturn_t
s3c_timer_interrupt(int irq, void *dev_id)
{

	_s3c_timer_interrupt(irq, dev_id);
	__raw_writel((0x1f & __raw_readl(S3C_TINT_CSTAT)) | S3C_TINT_CSTAT_T4INT, S3C_TINT_CSTAT);

	return IRQ_HANDLED;
}

static struct irqaction s3c_timer_irq = {
	.name		= "S3C Timer Tick",
	.flags		= IRQF_DISABLED | IRQF_TIMER,
	.handler	= s3c_timer_interrupt,
};

static void s3c_timer_setup (void)
{
	unsigned long tcon;
	unsigned long tcnt;
	unsigned long tcfg1;
	unsigned long tcfg0;
	unsigned long pclk;

	struct clk *clk;

#ifdef T32_PROBE_TEST
	gpio_direction_output(S3C_GPF13);
#endif

	tcnt = DEFAULT_TIMER_VAL;  /* default value for tcnt */

	tcfg1 = __raw_readl(S3C_TCFG1);
	tcfg0 = __raw_readl(S3C_TCFG0);

	clk = clk_get(NULL, "timers");
	if (IS_ERR(clk))
	panic("failed to get clock for system timer");

	clk_enable(clk);

	pclk = clk_get_rate(clk);
	timerclock = (pclk / ((PRESCALER + 1)*DIVIDER));
	s3c_latch = ((timerclock + HZ / 2) / HZ);

	timer_usec_ticks = timer_mask_usec_ticks(((PRESCALER + 1)*DIVIDER), pclk);

	tcfg1 &= ~S3C_TCFG1_MUX4_MASK;
	tcfg1 |= S3C_TCFG1_MUX4_DIV1;

	tcfg0 &= ~S3C_TCFG_PRESCALER1_MASK;
	tcfg0 |= (PRESCALER) << S3C_TCFG_PRESCALER1_SHIFT;

	max_buffer_cnt = (unsigned long)(DEFAULT_TIMER_VAL / s3c_latch);

	tcnt = (timerclock / HZ) - 1;

	__raw_writel(tcfg1, S3C_TCFG1);
	__raw_writel(tcfg0, S3C_TCFG0);

	timer_startval = tcnt;
	__raw_writel(tcnt, S3C_TCNTB(4));

	/* ensure timer is stopped... */

	tcon = __raw_readl(S3C_TCON);
	tcon &= ~(7<<20);
	tcon |= S3C_TCON_T4RELOAD;
	tcon |= S3C_TCON_T4MANUALUPD;

	__raw_writel(tcon, S3C_TCON);
	__raw_writel(tcnt, S3C_TCNTB(4));

	printk("timer tcon=%08lx, tcnt %04lx, tcfg %08lx,%08lx, usec %08lx\n",
	       tcon, tcnt, tcfg0, tcfg1, timer_usec_ticks);

	/* start the timer running */
	tcon |= S3C_TCON_T4START;
	tcon &= ~S3C_TCON_T4MANUALUPD;

	__raw_writel(tcon, S3C_TCON);

	/* timer4 interrupt enable */
	__raw_writel(__raw_readl(S3C_TINT_CSTAT) | S3C_TINT_CSTAT_T4INTEN, S3C_TINT_CSTAT);

}

#ifdef CONFIG_NO_IDLE_HZ
static int s3c_dyn_tick_enable_disable(void)
{
	/* nothing to do*/
	return 0;
}

static void s3c_dyn_tick_reprogram(unsigned long ticks)
{
	unsigned long timer_buff_val;

	if(ticks > 1){
		initial_match = __raw_readl(S3C_TCNTB(4));
		timer_buff_val = (initial_match + (ticks * s3c_latch));
		if(timer_buff_val & ~(DEFAULT_TIMER_VAL))
				timer_buff_val = max_buffer_cnt;
		__raw_writel(timer_buff_val,S3C_TCNTB(4));

		match_postponed = REPROGRAM_SET_FLAG;
	}
}

static irqreturn_t s3c_dyn_tick_handler(int irq, void *dev_id)
{
	if (match_postponed){
		match_postponed = REPROGRAM_UNSET_FLAG;
		__raw_writel(initial_match,S3C_TCNTB(4));
		if((unsigned long)(initial_match - __raw_readl(S3C_TCNTB(4))) <= 0)
			return s3c_timer_interrupt(irq,dev_id);
	}

	return IRQ_NONE;
}

static struct dyn_tick_timer s3c_dyn_tick = {
	.enable		= s3c_dyn_tick_enable_disable,
	.disable	= s3c_dyn_tick_enable_disable,
	.reprogram	= s3c_dyn_tick_reprogram,
	.handler	= s3c_dyn_tick_handler,
};

#endif

static void __init s3c_timer_init (void)
{
	s3c_timer_setup();
	setup_irq(IRQ_TIMER4, &s3c_timer_irq);
}

struct sys_timer s3c_timer = {
	.init		= s3c_timer_init,
	.offset		= s3c_gettimeoffset,
	.resume		= s3c_timer_setup,
#ifdef CONFIG_NO_IDLE_HZ
	.dyn_tick	= &s3c_dyn_tick,
#endif
};
