/*
 * linux/arch/arm/mach-s3c6410/leds.c
 *
 * S3C6410 LEDs dispatcher
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
#include <linux/miscdevice.h>
#include <linux/clk.h>
#include <linux/mutex.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/hardware.h>
#include <asm/leds.h>
#include <asm/mach-types.h>

#include <asm/arch/regs-gpio.h>
#include <asm/arch/irqs.h>
#include <asm/mach/irq.h>
#include "leds.h"

static irqreturn_t eint9_switch(int irq, void *dev_id, struct pt_regs *regs)
{

	printk("EINT9 interrupt occures!!!\n");
	return IRQ_HANDLED;
}


static int __init
s3c6410_leds_init(void)
{
	if (machine_is_smdk6410())
		leds_event = smdk6410_leds_event;
	else
		return -1;

	if (machine_is_smdk6410())
	{
		/*GPN12~15 used for LED*/
		/*Set GPN12~15 to output mode */
		gpio_direction_output(S3C_GPN12);
		if(gpio_get_pin(S3C_GPN12) == 0)
		{
			printk(KERN_WARNING "LED: can't set GPN12 output mode\n");
		}

		gpio_direction_output(S3C_GPN13);
		if(gpio_get_pin(S3C_GPN13) == 0)
		{
			printk(KERN_WARNING "LED: can't set GPN13 output mode\n");
		}

		gpio_direction_output(S3C_GPN14);
		if(gpio_get_pin(S3C_GPN14) == 0)
		{
			printk(KERN_WARNING "LED: can't set GPN14 output mode\n");
		}

		gpio_direction_output(S3C_GPN15);
		if(gpio_get_pin(S3C_GPN15) == 0)
		{
			printk(KERN_WARNING "LED: can't set GPN15 output mode\n");
		}

	}

	/* Get irqs */
	set_irq_type(IRQ_EINT9, IRQT_FALLING);
	gpio_pullup(S3C_GPN9, 0x0);
	if (request_irq(IRQ_EINT9, eint9_switch, SA_TRIGGER_FALLING, "EINT9", NULL)) {
		printk(KERN_ERR "leds.c: Could not allocate EINT9 !\n");
		return -EIO;
	}


	leds_event(led_start);
	return 0;
}

__initcall(s3c6410_leds_init);
