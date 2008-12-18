/* linux/drivers/input/touchscreen/s3c-ts.c
 *
 * $Id: s3c-ts2.c,v 1.2 2008/11/12 05:50:51 ihlee215 Exp $
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Copyright (c) 2004 Arnaud Patard <arnaud.patard@rtp-net.org>
 * iPAQ H1940 touchscreen support
 *
 * ChangeLog
 *
 * 2004-09-05: Herbert Potzl <herbert@13thfloor.at>
 *	- added clock (de-)allocation code
 *
 * 2005-03-06: Arnaud Patard <arnaud.patard@rtp-net.org>
 *      - h1940_ -> s3c24xx (this driver is now also used on the n30
 *        machines :P)
 *      - Debug messages are now enabled with the config option
 *        TOUCHSCREEN_S3C_DEBUG
 *      - Changed the way the value are read
 *      - Input subsystem should now work
 *      - Use ioremap and readl/writel
 *
 * 2005-03-23: Arnaud Patard <arnaud.patard@rtp-net.org>
 *      - Make use of some undocumented features of the touchscreen
 *        controller
 *
 * 2006-09-05: Ryu Euiyoul <ryu.real@gmail.com>
 *      - added power management suspend and resume code
 *
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/serio.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/hardware.h>

#include <asm/plat-s3c/regs-adc.h>
#include <asm/plat-s3c/ts.h>
#include <asm/arch/irqs.h>


#define CONFIG_TOUCHSCREEN_S3C_DEBUG
#undef CONFIG_TOUCHSCREEN_S3C_DEBUG

/* For ts->dev.id.version */
#define S3C_TSVERSION	0x0101

#define WAIT4INT(x)  (((x)<<8) | \
		     S3C2410_ADCTSC_YM_SEN | S3C2410_ADCTSC_YP_SEN | S3C2410_ADCTSC_XP_SEN | \
		     S3C2410_ADCTSC_XY_PST(3))

#define AUTOPST	     (S3C2410_ADCTSC_YM_SEN | S3C2410_ADCTSC_YP_SEN | S3C2410_ADCTSC_XP_SEN | \
		     S3C2410_ADCTSC_AUTO_PST | S3C2410_ADCTSC_XY_PST(0))


#define DEBUG_LVL    KERN_DEBUG



/* Touchscreen default configuration */
struct s3c_ts_mach_info s3c_ts_default_cfg __initdata = {
                .delay = 10000,
                .presc = 49,
                .oversampling_shift = 2,
		.resol_bit = 10,
};

/*
 * Definitions & global arrays.
 */
static char *s3c_ts_name = "S3C TouchScreen";
static void __iomem 	*ts_base;
static struct resource	*ts_mem;
static struct resource	*ts_irq;
static struct clk	*ts_clock;
static struct s3c_ts 	*ts;

static void touch_timer_fire(unsigned long data)
{
	unsigned long data0;
	unsigned long data1;
	int updown;

	data0 = readl(ts_base+S3C2410_ADCDAT0);
	data1 = readl(ts_base+S3C2410_ADCDAT1);

	updown = (!(data0 & S3C2410_ADCDAT0_UPDOWN)) && (!(data1 & S3C2410_ADCDAT1_UPDOWN));

	if (updown) {
		if (ts->count) {

#ifdef CONFIG_TOUCHSCREEN_S3C_DEBUG
			{
				struct timeval tv;
				do_gettimeofday(&tv);
				printk(KERN_INFO "T: %06d, X: %03ld, Y: %03ld\n", (int)tv.tv_usec, ts->xp, ts->yp);
			}
#endif

			input_report_abs(ts->dev, ABS_X, ts->xp);
			input_report_abs(ts->dev, ABS_Y, ts->yp);

			input_report_key(ts->dev, BTN_TOUCH, 1);
			input_report_abs(ts->dev, ABS_PRESSURE, 1);
			input_sync(ts->dev);
		}

		ts->xp = 0;
		ts->yp = 0;
		ts->count = 0;

		writel(S3C2410_ADCTSC_PULL_UP_DISABLE | AUTOPST, ts_base+S3C2410_ADCTSC);
		writel(readl(ts_base+S3C2410_ADCCON) | S3C2410_ADCCON_ENABLE_START, ts_base+S3C2410_ADCCON);
	}
	else {

		ts->count = 0;

		input_report_key(ts->dev, BTN_TOUCH, 0);
		input_report_abs(ts->dev, ABS_PRESSURE, 0);
		input_sync(ts->dev);

		writel(WAIT4INT(0), ts_base+S3C2410_ADCTSC);
	}
}

static struct timer_list touch_timer =
		TIMER_INITIALIZER(touch_timer_fire, 0, 0);

static irqreturn_t stylus_updown(int irqno, void *param)
{
	unsigned long data0;
	unsigned long data1;
	int updown;

	data0 = readl(ts_base+S3C2410_ADCDAT0);
	data1 = readl(ts_base+S3C2410_ADCDAT1);

	updown = (!(data0 & S3C2410_ADCDAT0_UPDOWN)) && (!(data1 & S3C2410_ADCDAT1_UPDOWN));

#ifdef CONFIG_TOUCHSCREEN_S3C_DEBUG
       printk(KERN_INFO "   %c\n",	updown ? 'D' : 'U');
#endif

	/* TODO we should never get an interrupt with updown set while
	 * the timer is running, but maybe we ought to verify that the
	 * timer isn't running anyways. */

	if (updown)
		touch_timer_fire(0);

#if defined(CONFIG_CPU_S3C6400) || defined(CONFIG_CPU_S3C6410)
        __raw_writel(0x0, ts_base+S3C6400_ADCCLRWK);
        __raw_writel(0x0, ts_base+S3C6400_ADCCLRINT);
#endif
        
	return IRQ_HANDLED;
}

static irqreturn_t stylus_action(int irqno, void *param)
{
	unsigned long data0;
	unsigned long data1;

	data0 = readl(ts_base+S3C2410_ADCDAT0);
	data1 = readl(ts_base+S3C2410_ADCDAT1);

#if defined(CONFIG_TOUCHSCREEN_NEW)
	ts->yp += S3C_ADCDAT0_XPDATA_MASK_12BIT - (data0 & S3C_ADCDAT0_XPDATA_MASK_12BIT);
	ts->xp += S3C_ADCDAT1_YPDATA_MASK_12BIT - (data1 & S3C_ADCDAT1_YPDATA_MASK_12BIT);

#elif defined(CONFIG_CPU_S3C2450) || defined(CONFIG_CPU_S3C2416) || defined(CONFIG_CPU_S3C6410)
	ts->xp += data0 & S3C_ADCDAT0_XPDATA_MASK_12BIT;
	ts->yp += data1 & S3C_ADCDAT1_YPDATA_MASK_12BIT;
#else
	ts->xp += data0 & S3C2410_ADCDAT0_XPDATA_MASK;
	ts->yp += data1 & S3C2410_ADCDAT1_YPDATA_MASK;
#endif

	ts->count++;

	if (ts->count < (1<<ts->shift)) {
		writel(S3C2410_ADCTSC_PULL_UP_DISABLE | AUTOPST, ts_base+S3C2410_ADCTSC);
		writel(readl(ts_base+S3C2410_ADCCON) | S3C2410_ADCCON_ENABLE_START, ts_base+S3C2410_ADCCON);
	} else {
		mod_timer(&touch_timer, jiffies+1);
		writel(WAIT4INT(1), ts_base+S3C2410_ADCTSC);
	}

#if defined(CONFIG_CPU_S3C6400) || defined(CONFIG_CPU_S3C6410)
        __raw_writel(0x0, ts_base+S3C6400_ADCCLRWK);
        __raw_writel(0x0, ts_base+S3C6400_ADCCLRINT);
#endif
	
	return IRQ_HANDLED;
}


static struct s3c_ts_mach_info *s3c_ts_get_platdata (struct device *dev)
{
	if (dev->platform_data != NULL)
		return (struct s3c_ts_mach_info *)dev->platform_data;

	return &s3c_ts_default_cfg;
}

/*
 * The functions for inserting/removing us as a module.
 */
static int __init s3c_ts_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct input_dev *input_dev;
	struct s3c_ts_mach_info * s3c_ts_cfg;
	int ret, size;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		printk(KERN_ERR "no memory resource specified\n");
		return -ENOENT;
	}

	size = (res->end - res->start) + 1;
	ts_mem = request_mem_region(res->start, size, pdev->name);
	if (ts_mem == NULL) {
		printk(KERN_ERR "failed to get memory region\n");
		ret = -ENOENT;
		goto err_req;
	}

	ts_base = ioremap(res->start, size);
	if (ts_base == NULL) {
		printk(KERN_ERR "failed to ioremap() region\n");
		ret = -EINVAL;
		goto err_map;
	}
	
	ts_clock = clk_get(&pdev->dev, "adc");
	if (IS_ERR(ts_clock)) {
		printk(KERN_ERR "failed to find watchdog clock source\n");
		ret = PTR_ERR(ts_clock);
		goto err_clk;
	}

	clk_enable(ts_clock);

	s3c_ts_cfg = s3c_ts_get_platdata(&pdev->dev);
		
	if ((s3c_ts_cfg->presc&0xff) > 0)
		writel(S3C2410_ADCCON_PRSCEN | S3C2410_ADCCON_PRSCVL(s3c_ts_cfg->presc&0xFF),\
				ts_base+S3C2410_ADCCON);
	else
		writel(0, ts_base+S3C2410_ADCCON);


	/* Initialise registers */
	if ((s3c_ts_cfg->delay&0xffff) > 0)
		writel(s3c_ts_cfg->delay & 0xffff, ts_base+S3C2410_ADCDLY);

	if (s3c_ts_cfg->resol_bit == 12) {
#if defined(CONFIG_CPU_S3C6410)
		writel(readl(ts_base+S3C2410_ADCCON)|S3C6410_ADCCON_RESSEL_12BIT, ts_base+S3C2410_ADCCON);
#elif defined(CONFIG_CPU_S3C2450) || defined(CONFIG_CPU_S3C2416)
		writel(readl(ts_base+S3C2410_ADCCON)|S3C2450_ADCCON_RESSEL_12BIT, ts_base+S3C2410_ADCCON);
#endif
	}
	
	writel(WAIT4INT(0), ts_base+S3C2410_ADCTSC);

	ts = kzalloc(sizeof(struct s3c_ts), GFP_KERNEL);
	
	input_dev = input_allocate_device();

	if (!input_dev) {
		ret = -ENOMEM;
		goto fail;
	}
	
	ts->dev = input_dev;

	ts->dev->evbit[0] = ts->dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	ts->dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	
#if defined(CONFIG_CPU_S3C2450) || defined(CONFIG_CPU_S3C6410) || defined(CONFIG_CPU_S3C2416)
	input_set_abs_params(ts->dev, ABS_X, 0, 0xFFF, 0, 0);
	input_set_abs_params(ts->dev, ABS_Y, 0, 0xFFF, 0, 0);
#else
	input_set_abs_params(ts->dev, ABS_X, 0, 0x3FF, 0, 0);
	input_set_abs_params(ts->dev, ABS_Y, 0, 0x3FF, 0, 0);
#endif
	input_set_abs_params(ts->dev, ABS_PRESSURE, 0, 1, 0, 0);

	sprintf(ts->phys, "input(ts)");

	ts->dev->name = s3c_ts_name;
	ts->dev->phys = ts->phys;
	ts->dev->id.bustype = BUS_RS232;
	ts->dev->id.vendor = 0xDEAD;
	ts->dev->id.product = 0xBEEF;
	ts->dev->id.version = S3C_TSVERSION;

	ts->shift = s3c_ts_cfg->oversampling_shift;
	
	/* Get irqs */	
	ts_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (ts_irq == NULL) {
		printk(KERN_ERR "no irq resource specified\n");
		ret = -ENOENT;
		goto err_clk;
	}

	ret = request_irq(ts_irq->start, stylus_updown, IRQF_SAMPLE_RANDOM, "s3c_updown", ts);
	if (ret != 0) {
		printk(KERN_ERR "s3c_ts.c: Could not allocate ts IRQ_TC !\n");
		ret = -EIO;
		goto err_clk;
	}
	
	ret = request_irq(ts_irq->end, stylus_action, IRQF_SAMPLE_RANDOM, "s3c_action", ts);
	if (ret != 0) {
		printk(KERN_ERR "s3c_ts.c: Could not allocate ts IRQ_ADC !\n");
		ret =  -EIO;
		goto err_irq;
	}

	printk(KERN_INFO "%s got loaded successfully : %d bits\n", s3c_ts_name, s3c_ts_cfg->resol_bit);

	/* All went ok, so register to the input system */
	ret = input_register_device(ts->dev);
	
	if(ret) {
		printk(KERN_ERR "s3c_ts.c: Could not register input device(touchscreen)!\n");
		ret = -EIO;
		goto fail;
	}

	return 0;

fail:	input_free_device(input_dev);
	kfree(ts);
	
err_irq:
	free_irq(ts_irq->start, ts->dev);
	free_irq(ts_irq->end, ts->dev);
	
err_clk:
	clk_disable(ts_clock);
	clk_put(ts_clock);
	
err_map:
	iounmap(ts_base);

err_req:
	release_resource(ts_mem);
	kfree(ts_mem);

	return ret;
}

static int s3c_ts_remove(struct platform_device *dev)
{
	printk(KERN_INFO "s3c_ts_remove() of TS called !\n");

	disable_irq(IRQ_ADC);
	disable_irq(IRQ_TC);
	free_irq(IRQ_TC, ts->dev);
	free_irq(IRQ_ADC, ts->dev);

	if (ts_clock) {
		clk_disable(ts_clock);
		clk_put(ts_clock);
		ts_clock = NULL;
	}

	input_unregister_device(ts->dev);
	iounmap(ts_base);

	return 0;
}

#ifdef CONFIG_PM
static unsigned int adccon, adctsc, adcdly;

static int s3c_ts_suspend(struct platform_device *dev, pm_message_t state)
{
	adccon = readl(ts_base+S3C2410_ADCCON);
	adctsc = readl(ts_base+S3C2410_ADCTSC);
	adcdly = readl(ts_base+S3C2410_ADCDLY);

	disable_irq(IRQ_ADC);
	disable_irq(IRQ_TC);
	
	clk_disable(ts_clock);

	return 0;
}

static int s3c_ts_resume(struct platform_device *pdev)
{
	clk_enable(ts_clock);

	writel(adccon, ts_base+S3C2410_ADCCON);
	writel(adctsc, ts_base+S3C2410_ADCTSC);
	writel(adcdly, ts_base+S3C2410_ADCDLY);
	writel(WAIT4INT(0), ts_base+S3C2410_ADCTSC);

	enable_irq(IRQ_ADC);
	enable_irq(IRQ_TC);
	return 0;
}
#else
#define s3c_ts_suspend NULL
#define s3c_ts_resume  NULL
#endif

static struct platform_driver s3c_ts_driver = {
       .probe          = s3c_ts_probe,
       .remove         = s3c_ts_remove,
       .suspend        = s3c_ts_suspend,
       .resume         = s3c_ts_resume,
       .driver		= {
		.owner	= THIS_MODULE,
		.name	= "s3c-ts",
	},
};

static char banner[] __initdata = KERN_INFO "S3C Touchscreen driver, (c) 2008 Samsung Electronics\n";

static int __init s3c_ts_init(void)
{
	printk(banner);
	return platform_driver_register(&s3c_ts_driver);
}

static void __exit s3c_ts_exit(void)
{
	platform_driver_unregister(&s3c_ts_driver);
}

module_init(s3c_ts_init);
module_exit(s3c_ts_exit);

MODULE_AUTHOR("Samsung AP");
MODULE_DESCRIPTION("S3C touchscreen driver");
MODULE_LICENSE("GPL");
