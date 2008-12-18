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

#ifdef CONFIG_NO_IDLE_HZ
static unsigned long s3c_timer_cnt_per_tick;
static unsigned long previous_dyn_ticks;
static unsigned long last_dyn_ticks;
static unsigned long sync_timer_startval;
static unsigned long last_sync_timer_count;
static unsigned long last_tick_count;
#define JIFFIES_TO_HW_TICKS(nr_jiffies, timer_cnt_per_tick)\
			((nr_jiffies) * timer_cnt_per_tick)


#endif

//#define T32_PROBE_TEST

#define TIMER_USEC_SHIFT 16

/* we use the shifted arithmetic to work out the ratio of timer ticks
 * to usecs, as often the peripheral clock is not a nice even multiple
 * of 1MHz.
 *
 * shift of 14 and 15 are too low for the 12MHz, 16 seems to be ok
 * for the current HZ value of 200 without producing overflows.
 *
 * Original patch by Dimitry Andric, updated by Ben Dooks
*/

#ifdef CONFIG_NO_IDLE_HZ

static inline unsigned long s3c_sync_timer_read(void)
{
	return __raw_readl(S3C_TCNTO(3));
}

static inline void s3c_init_sync_timer(void)
{
	unsigned long tcon;
	unsigned long tcfg1;
	unsigned long tcfg0;

	sync_timer_startval = 0xffffffff;

	tcon = __raw_readl(S3C_TCON);
	tcfg1 = __raw_readl(S3C_TCFG1);
	tcfg0 = __raw_readl(S3C_TCFG0);

	/* Sync timer setting */
	tcfg1 &= ~S3C_TCFG1_MUX3_MASK;
	tcfg1 |= S3C_TCFG1_MUX3_DIV1;
	__raw_writel(tcfg1, S3C_TCFG1);

	tcfg0 &= ~S3C_TCFG_PRESCALER1_MASK;
	tcfg0 |= (PRESCALER) << S3C_TCFG_PRESCALER1_SHIFT;
	
	__raw_writel(0xffffffff, S3C_TCNTB(3));
	tcon &= ~(0xf<<16);
	tcon |= S3C_TCON_T3RELOAD;
	tcon |= S3C_TCON_T3MANUALUPD;
	__raw_writel(tcon, S3C_TCON);


}
#endif

/* timer_mask_usec_ticks
 *
 * given a clock and divisor, make the value to pass into timer_ticks_to_usec
 * to scale the ticks into usecs
*/

static inline unsigned long
timer_mask_usec_ticks(unsigned long scaler, unsigned long pclk)
{
	unsigned long den = pclk / 1000;

	return ((1000 << TIMER_USEC_SHIFT) * scaler + (den >> 1)) / den;
}

/* timer_ticks_to_usec
 *
 * convert timer ticks to usec.
*/

static inline unsigned long timer_ticks_to_usec(unsigned long ticks)
{
	unsigned long res;

	res = ticks * timer_usec_ticks;
	res += 1 << (TIMER_USEC_SHIFT - 4);	/* round up slightly */

	return res >> TIMER_USEC_SHIFT;
}

/***
 * Returns microsecond  since last clock interrupt.  Note that interrupts
 * will have been disabled by do_gettimeoffset()
 * IRQs are disabled before entering here from do_gettimeofday()
 */

static unsigned long s3c_gettimeoffset (void)
{

	unsigned long tval;
	unsigned long tdone;
#ifdef CONFIG_NO_IDLE_HZ  
	tval = s3c_sync_timer_read();
	if(last_tick_count >= tval)
		tdone = last_tick_count - tval;
	else
		tdone = 0xffffffff - tval + last_tick_count + 1;
#else
	tval =  __raw_readl(S3C_TCNTO(4));

	tdone = timer_startval - tval;
#endif
	return timer_ticks_to_usec(tdone);

}

static inline irqreturn_t _s3c_timer_interrupt(int irq, void *dev_id)
{
#ifdef CONFIG_NO_IDLE_HZ
	unsigned long now,tmp;
	now = s3c_sync_timer_read();
	if(now <= last_tick_count) {
		tmp = last_tick_count - now;
	} else {
		tmp = 0xffffffff - now + last_tick_count + 1;
	}
	while(tmp >= s3c_timer_cnt_per_tick) {
		tmp -= s3c_timer_cnt_per_tick;
		if(last_tick_count > s3c_timer_cnt_per_tick)
			last_tick_count -= s3c_timer_cnt_per_tick;
		else
			last_tick_count = (0xffffffff - s3c_timer_cnt_per_tick + last_tick_count + 1);
		timer_tick();
	}
	//printk("z=0x%x\n",__raw_readl(S3C_TCNTO(4)));
#else
	timer_tick();
#endif
	return IRQ_HANDLED;
}
/*
 * IRQ handler for the timer
 */
static irqreturn_t
s3c_timer_interrupt(int irq, void *dev_id)
{
	

#ifdef T32_PROBE_TEST
	gpio_set_value(S3C_GPN14, 1);
#endif
	write_seqlock(&xtime_lock);
	_s3c_timer_interrupt(irq, dev_id);	
	write_sequnlock(&xtime_lock);

	 __raw_writel((0x1f & __raw_readl(S3C_TINT_CSTAT)) | S3C_TINT_CSTAT_T4INT, S3C_TINT_CSTAT);
//	 __raw_writel((0x1f & __raw_readl(S3C_TINT_CSTAT)) | S3C_TINT_CSTAT_T4INTEN, S3C_TINT_CSTAT);
#ifdef T32_PROBE_TEST
	gpio_set_value(S3C_GPN14, 0);
#endif
	return IRQ_HANDLED;
}

static struct irqaction s3c_timer_irq = {
	.name		= "S3C Timer Tick",
	.flags		= IRQF_DISABLED | IRQF_TIMER,
	.handler	= s3c_timer_interrupt,
};

#ifdef CONFIG_NO_IDLE_HZ
/*
 * Programs the next timer interrupt needed. Called when dynamic tick is
 * enabled, and to reprogram the ticks to skip from pm_idle. Note that
 * we can keep the timer continuous, and don't need to set it to run in
 * one-shot mode. This is because the timer will get reprogrammed again
 * after next interrupt.
 */
void s3c_timer_reprogram(unsigned long next_tick)
{
	unsigned long tcnt;
#ifdef T32_PROBE_TEST
	gpio_set_value(S3C_GPN15, 1);
#endif
	//printk("next_tick=%d\n",next_tick);
	if(next_tick == 0)
		next_tick = 1;
	last_dyn_ticks = next_tick;
	tcnt = JIFFIES_TO_HW_TICKS(next_tick, s3c_timer_cnt_per_tick);
	//printk("tcnt=0x%x\n",tcnt);
	__raw_writel((tcnt - 1), S3C_TCNTB(4));

	
#ifdef T32_PROBE_TEST
	gpio_set_value(S3C_GPN15, 0);
#endif

}

static int s3c_timer_enable_dyn_tick(void)
{
	return 0;
}

static int s3c_timer_disable_dyn_tick(void)
{
	unsigned long tcnt;
	
	last_dyn_ticks = 1;
	tcnt = JIFFIES_TO_HW_TICKS(1, s3c_timer_cnt_per_tick);

	__raw_writel((tcnt - 1), S3C_TCNTB(0));

	return 0;
}

static irqreturn_t s3c_dyn_tick_handler(int irq, void *dev_id)
{	
	//printk("dyntick irq%d\n",irq);
	return _s3c_timer_interrupt(irq, dev_id);
}

static struct dyn_tick_timer s3c_dyn_tick_timer = {
	.enable		= s3c_timer_enable_dyn_tick,
	.disable	= s3c_timer_disable_dyn_tick,
	.reprogram	= s3c_timer_reprogram,
	.handler	= s3c_dyn_tick_handler,
};
#endif

/*
 * Set up timer interrupt, and return the current time in seconds.
 *
 * Currently we only use timer4, as it is the only timer which has no
 * other function that can be exploited externally
 */
static void s3c_timer_setup (void)
{
	unsigned long tcon;
	unsigned long tcnt;
	unsigned long tcfg1;
	unsigned long tcfg0;
	unsigned long pclk;
	unsigned long timerclock;
	struct clk *clk;

#ifdef T32_PROBE_TEST

	gpio_direction_output(S3C_GPN14);
	gpio_direction_output(S3C_GPN15);
	gpio_set_value(S3C_GPN14, 0);
	gpio_set_value(S3C_GPN15, 0);
	/* read the current timer configuration bits */
#endif

	tcnt = 0xffffffff;  /* default value for tcnt */

	/* read the current timer configuration bits */

	tcfg1 = __raw_readl(S3C_TCFG1);
	tcfg0 = __raw_readl(S3C_TCFG0);

	/* this is used as default if no other timer can be found */

	clk = clk_get(NULL, "timers");
	if (IS_ERR(clk))
	panic("failed to get clock for system timer");

	clk_enable(clk);

	pclk = clk_get_rate(clk);
	timerclock = (pclk / ((PRESCALER + 1)*DIVIDER));

	/* configure clock tick */

	timer_usec_ticks = timer_mask_usec_ticks(((PRESCALER + 1)*DIVIDER), pclk);

	tcfg1 &= ~S3C_TCFG1_MUX4_MASK;
	tcfg1 |= S3C_TCFG1_MUX4_DIV1;

	tcfg0 &= ~S3C_TCFG_PRESCALER1_MASK;
	tcfg0 |= (PRESCALER) << S3C_TCFG_PRESCALER1_SHIFT;

	tcnt = timerclock / HZ;
	/* timers reload after counting zero, so reduce the count by 1 */

	tcnt--;

	__raw_writel(tcfg1, S3C_TCFG1);
	__raw_writel(tcfg0, S3C_TCFG0);

	timer_startval = tcnt;
	__raw_writel(tcnt, S3C_TCNTB(4));

	/* ensure timer is stopped... */
#ifdef CONFIG_NO_IDLE_HZ	
	s3c_timer_cnt_per_tick = (timerclock/HZ);
	previous_dyn_ticks = 1;
	last_dyn_ticks = 1;
	last_sync_timer_count = 0xffffffff;
	last_tick_count = 0xffffffff;
	s3c_init_sync_timer();
#endif
	tcon = __raw_readl(S3C_TCON);

	tcon &= ~(7<<20);
	tcon |= S3C_TCON_T4RELOAD;
	tcon |= S3C_TCON_T4MANUALUPD;

	__raw_writel(tcon, S3C_TCON);
	__raw_writel(tcnt, S3C_TCNTB(4));
	//__raw_writel(tcnt, S3C_TCMPB(4));

	printk("timer tcon=%08lx, tcnt %04lx, tcfg %08lx,%08lx, usec %08lx\n",
	       tcon, tcnt, tcfg0, tcfg1, timer_usec_ticks);

	/* start the timer running */
#ifdef CONFIG_NO_IDLE_HZ	
	tcon |= S3C_TCON_T4START|S3C_TCON_T3START;
	tcon &= ~(S3C_TCON_T4MANUALUPD|S3C_TCON_T3MANUALUPD);
#else
	tcon |= S3C_TCON_T4START;
	tcon &= ~S3C_TCON_T4MANUALUPD;
#endif
	__raw_writel(tcon, S3C_TCON);

	/* timer4 interrupt enable */
	__raw_writel(__raw_readl(S3C_TINT_CSTAT) | S3C_TINT_CSTAT_T4INTEN, S3C_TINT_CSTAT);
	
}

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
	.dyn_tick	= &s3c_dyn_tick_timer,
#endif		
};
