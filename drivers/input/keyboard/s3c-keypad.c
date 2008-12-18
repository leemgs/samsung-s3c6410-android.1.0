/*
 * drivers/input/keyboard/s3c-keypad.c
 * KeyPad Interface on S3C 
 *
 * $Id: s3c-keypad.c,v 1.1 2008/06/09 07:43:16 dark0351 Exp $
 * 
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive
 * for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/delay.h>
#include <asm/irq.h>

#include <asm/arch/regs-gpio.h>
#include <asm/arch/regs-keypad.h>
#include "s3c-keypad.h"

#undef S3C_KEYPAD_DEBUG 
//#define S3C_KEYPAD_DEBUG 

#ifdef S3C_KEYPAD_DEBUG
#define DPRINTK(x...) printk("S3C-Keypad " x)
#else
#define DPRINTK(x...)		/* !!!! */
#endif

#define DEVICE_NAME "s3c-keypad"

#define TRUE 1
#define FALSE 0

static struct timer_list keypad_timer;
static int is_timer_on = FALSE;


static int keypad_scan(u32 *keymask_low, u32 *keymask_high)
{
	int i,j = 0;
	u32 cval,rval;

	for (i=0; i<KEYPAD_COLUMNS; i++) {
		cval = readl(key_base+S3C_KEYIFCOL) | KEYCOL_DMASK;
		cval &= ~(1 << i);
		writel(cval, key_base+S3C_KEYIFCOL);

		udelay(KEYPAD_DELAY);

		rval = ~(readl(key_base+S3C_KEYIFROW)) & KEYROW_DMASK;
		
		if ((i*KEYPAD_ROWS) < MAX_KEYMASK_NR)
			*keymask_low |= (rval << (i * KEYPAD_ROWS));
		else {
			*keymask_high |= (rval << (j * KEYPAD_ROWS));
			j = j +1;
		}
	}

	writel(KEYIFCOL_CLEAR, key_base+S3C_KEYIFCOL);

	return 0;
}

static unsigned prevmask_low = 0, prevmask_high = 0;

static void keypad_timer_handler(unsigned long data)
{
	u32 keymask_low = 0, keymask_high = 0;
	u32 press_mask_low, press_mask_high;
	u32 release_mask_low, release_mask_high;
	int i;
	struct s3c_keypad *pdata = (struct s3c_keypad *)data;
	struct input_dev *dev = pdata->dev;

	keypad_scan(&keymask_low, &keymask_high);

	if (keymask_low != prevmask_low) {
		press_mask_low =
			((keymask_low ^ prevmask_low) & keymask_low); 
		release_mask_low =
			((keymask_low ^ prevmask_low) & prevmask_low); 

		i = 0;
		while (press_mask_low) {
			if (press_mask_low & 1) {
				input_report_key(dev,pdata->keycodes[i],1);
				DPRINTK("low Pressed  : %d\n",i);
			}
			press_mask_low >>= 1;
			i++;
		}

		i = 0;
		while (release_mask_low) {
			if (release_mask_low & 1) {
				input_report_key(dev,pdata->keycodes[i],0);
				DPRINTK("low Released : %d\n",i);
			}
			release_mask_low >>= 1;
			i++;
		}
		prevmask_low = keymask_low;
	}

	if (keymask_high != prevmask_high) {
		press_mask_high =
			((keymask_high ^ prevmask_high) & keymask_high); 
		release_mask_high =
			((keymask_high ^ prevmask_high) & prevmask_high);

		i = 0;
		while (press_mask_high) {
			if (press_mask_high & 1) {
				input_report_key(dev,pdata->keycodes[i+MAX_KEYMASK_NR],1);
				DPRINTK("high Pressed  : %d %d\n",pdata->keycodes[i+MAX_KEYMASK_NR],i);
			}
			press_mask_high >>= 1;
			i++;
		}

		i = 0;
		while (release_mask_high) {
			if (release_mask_high & 1) {
				input_report_key(dev,pdata->keycodes[i+MAX_KEYMASK_NR],0);
				DPRINTK("high Released : %d\n",pdata->keycodes[i+MAX_KEYMASK_NR]);
			}
			release_mask_high >>= 1;
			i++;
		}
		prevmask_high = keymask_high;
	}

	if (keymask_low | keymask_high) {
		mod_timer(&keypad_timer,jiffies + HZ/10);
	} else {
		writel(KEYIFCON_INIT, key_base+S3C_KEYIFCON);
		is_timer_on = FALSE;
	}	
}

static irqreturn_t s3c_keypad_isr(int irq, void *dev_id)
{

	/* disable keypad interrupt and schedule for keypad timer handler */
	writel(readl(key_base+S3C_KEYIFCON) & ~(INT_F_EN|INT_R_EN), key_base+S3C_KEYIFCON);

	keypad_timer.expires = jiffies + (HZ/100);
	if ( is_timer_on == FALSE) {
		add_timer(&keypad_timer);
		is_timer_on = TRUE;
	} else {
		mod_timer(&keypad_timer,keypad_timer.expires);
	}
	/*Clear the keypad interrupt status*/
	writel(KEYIFSTSCLR_CLEAR, key_base+S3C_KEYIFSTSCLR);

	return IRQ_HANDLED;
}



static int __init s3c_keypad_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct input_dev *input_dev;
	int key, code;
	struct s3c_keypad *s3c_keypad;

	key_base = ioremap(S3C24XX_PA_KEYPAD, S3C24XX_SZ_KEYPAD);

	if (key_base == NULL) {
		printk(KERN_ERR "Failed to remap register block\n");
		return -ENOMEM;
	}
	
	s3c_keypad = kzalloc(sizeof(struct s3c_keypad), GFP_KERNEL);
	input_dev = input_allocate_device();

	if (!s3c_keypad || !input_dev) {
		kfree(s3c_keypad);
		input_free_device(input_dev);
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, s3c_keypad);
	s3c_keypad->dev = input_dev;
	
	writel(KEYIFCON_INIT, key_base+S3C_KEYIFCON);
	writel(KEYIFFC_DIV, key_base+S3C_KEYIFFC);

	/* Set GPIO Port for keypad mode and pull-up disable*/
	writel(KEYPAD_ROW_GPIO_SET, KEYPAD_ROW_GPIOCON);
	writel(KEYPAD_COL_GPIO_SET, KEYPAD_COL_GPIOCON);

	writel(KEYPAD_ROW_GPIOPUD_DIS, KEYPAD_ROW_GPIOPUD);
	writel(KEYPAD_COL_GPIOPUD_DIS, KEYPAD_COL_GPIOPUD);

	writel(KEYIFCOL_CLEAR, key_base+S3C_KEYIFCOL);

	/* create and register the input driver */
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_REP, input_dev->evbit);
	s3c_keypad->nr_rows = KEYPAD_ROWS;
	s3c_keypad->no_cols = KEYPAD_COLUMNS;
	s3c_keypad->total_keys = MAX_KEYPAD_NR;

	for(key = 0; key < s3c_keypad->total_keys; key++){
		code = s3c_keypad->keycodes[key] = keypad_keycode[key];
		if(code<=0)
			continue;
		set_bit(code & KEY_MAX, input_dev->keybit);
	}

	input_dev->name = DEVICE_NAME;
	input_dev->phys = "s3c-keypad/input0";
	input_dev->cdev.dev = &pdev->dev;
	input_dev->private = s3c_keypad;
	
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0001;

	input_dev->keycode = keypad_keycode;
	
	ret = input_register_device(input_dev);
	if (ret) {
		printk("Unable to register s3c-keypad input device!!!\n");
		goto out;
	}


	/* Scan timer init */
	init_timer(&keypad_timer);
	keypad_timer.function = keypad_timer_handler;
	keypad_timer.data = (unsigned long)s3c_keypad;
	
	ret = request_irq(IRQ_KEYPAD, s3c_keypad_isr, IRQF_SAMPLE_RANDOM,
		DEVICE_NAME, (void *) pdev);
	if (ret) {
		printk("request_irq failed (IRQ_KEYPAD) !!!\n");
		goto out;
	}

	keypad_timer.expires = jiffies + (HZ/10);

	if (is_timer_on == FALSE) {
		add_timer(&keypad_timer);
		is_timer_on = TRUE;
	} else {
		mod_timer(&keypad_timer,keypad_timer.expires);
	}

	printk( DEVICE_NAME " Initialized\n");
	return 0;

out:
	input_unregister_device(input_dev);
	kfree(s3c_keypad);
	return ret;
}

static int s3c_keypad_remove(struct platform_device *pdev)
{
	struct input_dev *input_dev = platform_get_drvdata(pdev);
	writel(KEYIFCON_CLEAR, key_base+S3C_KEYIFCON);

	input_unregister_device(input_dev);
	kfree(pdev->dev.platform_data);
	free_irq(IRQ_KEYPAD, (void *) pdev);

	del_timer(&keypad_timer);	
	printk(DEVICE_NAME " Removed.\n");
	return 0;
}

#ifdef CONFIG_PM
static int s3c_keypad_suspend(struct platform_device *dev, pm_message_t state)
{
	/* TODO */
	return 0;
}

static int s3c_keypad_resume(struct platform_device *dev)
{
	/* TODO */
	return 0;
}
#else
#define s3c_keypad_suspend NULL
#define s3c_keypad_resume  NULL
#endif /* CONFIG_PM */

static struct platform_driver s3c_keypad_driver = {
	.probe		= s3c_keypad_probe,
	.remove		= s3c_keypad_remove,
	.suspend	= s3c_keypad_suspend,
	.resume		= s3c_keypad_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "s3c-keypad",
	},
};

static int __init s3c_keypad_init(void)
{
	int ret;

	ret = platform_driver_register(&s3c_keypad_driver);
	
	if(!ret)
	   printk(KERN_INFO "S3C Keypad Driver\n");

	return ret;
}

static void __exit s3c_keypad_exit(void)
{
	platform_driver_unregister(&s3c_keypad_driver);
}

module_init(s3c_keypad_init);
module_exit(s3c_keypad_exit);

MODULE_AUTHOR("Samsung");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("KeyPad interface for Samsung S3C");
