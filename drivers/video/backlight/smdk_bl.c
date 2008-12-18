/*
 *  Backlight Driver for SMDK(Samsung Mobile Develop Kit) board
 *
 *  Copyright (c) 2008 Jongpill Lee
 *
 *  Based on hp680 Backlight Driver
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/fb.h>
#include <linux/backlight.h>

#define SMDK_MAX_INTENSITY 100
#define SMDK_DEFAULT_INTENSITY	0

unsigned int smdk_bl_suspend_ck;
static int current_intensity = 0;

extern void s3cfb_set_brightness(int val);

static DEFINE_SPINLOCK(bl_lock);

static void smdk_bl_send_intensity(struct backlight_device *bd)
{
	unsigned long flags;
	int intensity = bd->props.brightness;

	if (bd->props.power != FB_BLANK_UNBLANK)
		intensity = 0;
	if (bd->props.fb_blank != FB_BLANK_UNBLANK)
		intensity = 0;
	if (smdk_bl_suspend_ck)
		intensity = 0;

	s3cfb_set_brightness(SMDK_MAX_INTENSITY - intensity);

	current_intensity = intensity;
}


#ifdef CONFIG_PM
static int smdk_bl_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);

	smdk_bl_suspend_ck = 1;
	smdk_bl_send_intensity(bd);

	return 0;
}

static int smdk_bl_resume(struct platform_device *pdev)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);

	smdk_bl_suspend_ck = 0;
	smdk_bl_send_intensity(bd);

	return 0;
}
#else
#define smdk_bl_suspend	NULL
#define smdk_bl_resume	NULL
#endif

static int smdk_bl_set_intensity(struct backlight_device *bd)
{
	smdk_bl_send_intensity(bd);
	return 0;
}

static int smdk_bl_get_intensity(struct backlight_device *bd)
{
	return current_intensity;
}

static struct backlight_ops smdk_bl_ops = {
	.get_brightness = smdk_bl_get_intensity,
	.update_status  = smdk_bl_set_intensity,
};

static int __init smdk_bl_probe(struct platform_device *pdev)
{
	struct backlight_device *bd;

	bd = backlight_device_register ("smdk-backlight", &pdev->dev, NULL,
		    &smdk_bl_ops);
	if (IS_ERR(bd))
		return PTR_ERR(bd);

	platform_set_drvdata(pdev, bd);

	bd->props.max_brightness = SMDK_MAX_INTENSITY;
	bd->props.brightness = SMDK_DEFAULT_INTENSITY;
	smdk_bl_send_intensity(bd);

	return 0;
}

static int smdk_bl_remove(struct platform_device *pdev)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);

	bd->props.brightness = 0;
	bd->props.power = 0;
	smdk_bl_send_intensity(bd);

	backlight_device_unregister(bd);

	return 0;
}

static struct platform_driver smdk_bl_driver = {
	.probe		= smdk_bl_probe,
	.remove		= smdk_bl_remove,
	.suspend	= smdk_bl_suspend,
	.resume		= smdk_bl_resume,
	.driver		= {
		.name	= "smdk-backlight",
	},
};

static struct platform_device *smdk_bl_device;

static int __init smdk_bl_init(void)
{
	int ret;

	printk("SMDK board LCD Backlight Device Driver (c) 2008 Samsung Electronics \n");

	ret = platform_driver_register(&smdk_bl_driver);
	if (!ret) {
		smdk_bl_device = platform_device_alloc("smdk-backlight", -1);
		if (!smdk_bl_device)
			return -ENOMEM;

		ret = platform_device_add(smdk_bl_device);

		if (ret) {
			platform_device_put(smdk_bl_device);
			platform_driver_unregister(&smdk_bl_driver);
		}
	}
	return ret;
}

static void __exit smdk_bl_exit(void)
{
	platform_device_unregister(smdk_bl_device);
 	platform_driver_unregister(&smdk_bl_driver);
}

module_init(smdk_bl_init);
module_exit(smdk_bl_exit);

MODULE_AUTHOR("Jongpill Lee <boyko.lee@samsung.com>");
MODULE_DESCRIPTION("SMDK board Backlight Driver");
MODULE_LICENSE("GPL");
