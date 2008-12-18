/* linux/arch/arm/mach-s3c6410/pm-s3c6410.c
 *
 * Copyright (c) 2004 Samsung Electronics
 *
 * S3C Power Manager (Suspend-To-RAM) support
 *
 * 2007.04.19 - Modified by JaeCheol Lee <jc.lee@samsung.com>
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
 *
 * Parts based on arch/arm/mach-pxa/pm.c
 *
 * Thanks to Dimitry Andric for debugging
*/

#include <linux/config.h>
#include <linux/init.h>
#include <linux/suspend.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/crc32.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/spinlock.h>

#include <asm/hardware.h>
#include <asm/io.h>

#include <asm/arch/registers.h>
#include <asm/mach/time.h>

#include "pm-s3c6410.h"

/* cache functions from arch/arm/mm/cache-v6.S */
extern void v6_flush_kern_cache_all(void);
/* for external use */
unsigned long s3c_pm_flags;

#define PFX "s3c-pm: "


static struct sleep_save core_save[] = {
	SAVE_ITEM(S3C_SDMA_SEL),
};

/* this lot should be really saved by the IRQ code */
/* VICXADDRESSXX initilaization to be needed */
static struct sleep_save irq_save[] = {
	SAVE_ITEM(S3C_VIC0INTSELECT),
	SAVE_ITEM(S3C_VIC1INTSELECT),
	SAVE_ITEM(S3C_VIC0INTENABLE),
	SAVE_ITEM(S3C_VIC1INTENABLE),
	SAVE_ITEM(S3C_VIC0SOFTINT),
	SAVE_ITEM(S3C_VIC1SOFTINT),
};

static struct sleep_save sromc_save[] = {
	SAVE_ITEM(S3C_SROM_BW),
	SAVE_ITEM(S3C_SROM_BC0),
	SAVE_ITEM(S3C_SROM_BC1),
	SAVE_ITEM(S3C_SROM_BC2),
	SAVE_ITEM(S3C_SROM_BC3),
	SAVE_ITEM(S3C_SROM_BC4),
	SAVE_ITEM(S3C_SROM_BC5),
};
static struct sleep_save gpio_save[] = {

	SAVE_ITEM(S3C_GPK0CON),
	SAVE_ITEM(S3C_GPK1CON),
	SAVE_ITEM(S3C_GPKDAT),
	SAVE_ITEM(S3C_GPKPU),

	SAVE_ITEM(S3C_GPL0CON),
	SAVE_ITEM(S3C_GPL1CON),
	SAVE_ITEM(S3C_GPLDAT),
	SAVE_ITEM(S3C_GPLPU),

	SAVE_ITEM(S3C_GPMCON),
	SAVE_ITEM(S3C_GPMDAT),
	SAVE_ITEM(S3C_GPMPU),
	
	SAVE_ITEM(S3C_GPNCON),
	SAVE_ITEM(S3C_GPNDAT),
	SAVE_ITEM(S3C_GPNPU),

	/* External interrupt */
	SAVE_ITEM(S3C_EINTCON0),
	SAVE_ITEM(S3C_EINTCON1),
	SAVE_ITEM(S3C_EINTFLTCON0),
	SAVE_ITEM(S3C_EINTFLTCON1),
	SAVE_ITEM(S3C_EINTFLTCON2),
	SAVE_ITEM(S3C_EINTFLTCON3),
	SAVE_ITEM(S3C_EINTMASK),
	SAVE_ITEM(S3C_EINT12CON),
	SAVE_ITEM(S3C_EINT34CON),
	SAVE_ITEM(S3C_EINT56CON),
	SAVE_ITEM(S3C_EINT78CON),
	SAVE_ITEM(S3C_EINT9CON),
	SAVE_ITEM(S3C_EINT12FLTCON),
	SAVE_ITEM(S3C_EINT34FLTCON),
	SAVE_ITEM(S3C_EINT56FLTCON),
	SAVE_ITEM(S3C_EINT78FLTCON),
	SAVE_ITEM(S3C_EINT9FLTCON),
	SAVE_ITEM(S3C_EINT12MASK),
	SAVE_ITEM(S3C_EINT34MASK),
	SAVE_ITEM(S3C_EINT56MASK),
	SAVE_ITEM(S3C_EINT78MASK),
	SAVE_ITEM(S3C_EINT9MASK),
	SAVE_ITEM(S3C_EINT34FLTCON),
	SAVE_ITEM(S3C_EINT56FLTCON),
	SAVE_ITEM(S3C_EINT78FLTCON),
	SAVE_ITEM(S3C_EINT9FLTCON),
	SAVE_ITEM(S3C_PRIORITY),

	/* GPIO memory map bug in EVT0 */
	SAVE_ITEM(S3C_GPACON),
	SAVE_ITEM(S3C_GPADAT),
	SAVE_ITEM(S3C_GPAPU),

	SAVE_ITEM(S3C_GPBCON),
	SAVE_ITEM(S3C_GPBDAT),
	SAVE_ITEM(S3C_GPBPU),

	SAVE_ITEM(S3C_GPCCON),
	SAVE_ITEM(S3C_GPCDAT),
	SAVE_ITEM(S3C_GPCPU),

	SAVE_ITEM(S3C_GPDCON),
	SAVE_ITEM(S3C_GPDDAT),
	SAVE_ITEM(S3C_GPDPU),

	SAVE_ITEM(S3C_GPECON),
	SAVE_ITEM(S3C_GPEDAT),
	SAVE_ITEM(S3C_GPEPU),

	SAVE_ITEM(S3C_GPFCON),
	SAVE_ITEM(S3C_GPFDAT),
	SAVE_ITEM(S3C_GPFPU),

	SAVE_ITEM(S3C_GPGCON),
	SAVE_ITEM(S3C_GPGDAT),
	SAVE_ITEM(S3C_GPGPU),

	SAVE_ITEM(S3C_GPH0CON),
	SAVE_ITEM(S3C_GPH1CON),
	SAVE_ITEM(S3C_GPHDAT),
	SAVE_ITEM(S3C_GPHPU),

	SAVE_ITEM(S3C_GPICON),
	SAVE_ITEM(S3C_GPIDAT),
	SAVE_ITEM(S3C_GPIPU),

	SAVE_ITEM(S3C_GPJCON),
	SAVE_ITEM(S3C_GPJDAT),
	SAVE_ITEM(S3C_GPJPU),


	SAVE_ITEM(S3C_GPOCON),
	SAVE_ITEM(S3C_GPODAT),
	SAVE_ITEM(S3C_GPOPU),

	SAVE_ITEM(S3C_GPPCON),
	SAVE_ITEM(S3C_GPPDAT),
	SAVE_ITEM(S3C_GPPPU),

	SAVE_ITEM(S3C_GPQCON),
	SAVE_ITEM(S3C_GPQDAT),
	SAVE_ITEM(S3C_GPQPU),

	/* Special register*/
	SAVE_ITEM(S3C_SPCON),

	/* Memory port control */
	SAVE_ITEM(S3C_MEM0CONSTOP),
	SAVE_ITEM(S3C_MEM1CONSTOP),
	SAVE_ITEM(S3C_MEM0CONSLP0),
	SAVE_ITEM(S3C_MEM0CONSLP1),
	SAVE_ITEM(S3C_MEM1CONSLP),
	SAVE_ITEM(S3C_MEM0DRVCON),
	SAVE_ITEM(S3C_MEM1DRVCON),	

};

static struct sleep_save ts_save[] = {
        /* Touch Screen */
        SAVE_ITEM(S3C_ADCDLY),
        SAVE_ITEM(S3C_ADCTSC),
        SAVE_ITEM(S3C_ADCCON),
};

static struct sleep_save lcd_save[] = {

};


#define SAVE_UART(va) \
	SAVE_ITEM((va) + S3C_ULCON), \
	SAVE_ITEM((va) + S3C_UCON), \
	SAVE_ITEM((va) + S3C_UFCON), \
	SAVE_ITEM((va) + S3C_UMCON), \
	SAVE_ITEM((va) + S3C_UBRDIV), \
	SAVE_ITEM((va) + S3C_UDIVSLOT), \
	SAVE_ITEM((va) + S3C_UINTMSK)


static struct sleep_save uart_save[] = {
	SAVE_UART(S3C24XX_VA_UART0),
	SAVE_UART(S3C24XX_VA_UART1),
	SAVE_UART(S3C24XX_VA_UART2),
	SAVE_UART(S3C24XX_VA_UART3),
};

#if 1


/* debug
 *
 * we send the debug to printascii() to allow it to be seen if the
 * system never wakes up from the sleep
*/

extern void printascii(const char *);

static void pm_dbg(const char *fmt, ...)
{
	va_list va;
	char buff[256];

	va_start(va, fmt);
	vsprintf(buff, fmt, va);
	va_end(va);

	printascii(buff);
}

static void s3c6410_pm_debug_init(void)
{
	unsigned long tmp = __raw_readl(S3C_PCLK_GATE);

	/* re-start uart clocks */
	tmp |= S3C_CLKCON_PCLK_UART0;
	tmp |= S3C_CLKCON_PCLK_UART1;
	tmp |= S3C_CLKCON_PCLK_UART2;

	__raw_writel(tmp, S3C_PCLK_GATE);
	udelay(10);
}

#define DBG(fmt...) pm_dbg(fmt)
#else
#define DBG(fmt...) printk(KERN_DEBUG fmt)

#define s3c6400_pm_debug_init() do { } while(0)

#endif


#if defined(CONFIG_S3C6410_PM_CHECK) && CONFIG_S3C6410_PM_CHECK_CHUNKSIZE != 0

/* suspend checking code...
 *
 * this next area does a set of crc checks over all the installed
 * memory, so the system can verify if the resume was ok.
 *
 * CONFIG_S3C6400_PM_CHECK_CHUNKSIZE defines the block-size for the CRC,
 * increasing it will mean that the area corrupted will be less easy to spot,
 * and reducing the size will cause the CRC save area to grow
*/

#define CHECK_CHUNKSIZE (CONFIG_S3C6410_PM_CHECK_CHUNKSIZE * 1024)

static u32 crc_size;	/* size needed for the crc block */
static u32 *crcs;	/* allocated over suspend/resume */

typedef u32 *(run_fn_t)(struct resource *ptr, u32 *arg);

/* s3c6410_pm_run_res
 *
 * go thorugh the given resource list, and look for system ram
*/

static void s3c6410_pm_run_res(struct resource *ptr, run_fn_t fn, u32 *arg)
{
	while (ptr != NULL) {
		if (ptr->child != NULL)
			s3c6410_pm_run_res(ptr->child, fn, arg);

		if ((ptr->flags & IORESOURCE_MEM) &&
		    strcmp(ptr->name, "System RAM") == 0) {
			DBG("Found system RAM at %08lx..%08lx\n",
			    ptr->start, ptr->end);
			arg = (fn)(ptr, arg);
		}

		ptr = ptr->sibling;
	}
}

static void s3c6410_pm_run_sysram(run_fn_t fn, u32 *arg)
{
	s3c6410_pm_run_res(&iomem_resource, fn, arg);
}

static u32 *s3c6410_pm_countram(struct resource *res, u32 *val)
{
	u32 size = (u32)(res->end - res->start)+1;

	size += CHECK_CHUNKSIZE-1;
	size /= CHECK_CHUNKSIZE;

	DBG("Area %08lx..%08lx, %d blocks\n", res->start, res->end, size);

	*val += size * sizeof(u32);
	return val;
}

/* s3c6400_pm_prepare_check
 *
 * prepare the necessary information for creating the CRCs. This
 * must be done before the final save, as it will require memory
 * allocating, and thus touching bits of the kernel we do not
 * know about.
*/

static void s3c6410_pm_check_prepare(void)
{
	crc_size = 0;

	s3c6410_pm_run_sysram(s3c6410_pm_countram, &crc_size);

	DBG("s3c6410_pm_prepare_check: %u checks needed\n", crc_size);

	crcs = kmalloc(crc_size+4, GFP_KERNEL);
	if (crcs == NULL)
		printk(KERN_ERR "Cannot allocated CRC save area\n");
}

static u32 *s3c6410_pm_makecheck(struct resource *res, u32 *val)
{
	unsigned long addr, left;

	for (addr = res->start; addr < res->end;
	     addr += CHECK_CHUNKSIZE) {
		left = res->end - addr;

		if (left > CHECK_CHUNKSIZE)
			left = CHECK_CHUNKSIZE;

		*val = crc32_le(~0, phys_to_virt(addr), left);
		val++;
	}

	return val;
}

/* s3c6410_pm_check_store
 *
 * compute the CRC values for the memory blocks before the final
 * sleep.
*/

static void s3c6410_pm_check_store(void)
{
	if (crcs != NULL)
		s3c6410_pm_run_sysram(s3c6410_pm_makecheck, crcs);
}

/* in_region
 *
 * return TRUE if the area defined by ptr..ptr+size contatins the
 * what..what+whatsz
*/

static inline int in_region(void *ptr, int size, void *what, size_t whatsz)
{
	if ((what+whatsz) < ptr)
		return 0;

	if (what > (ptr+size))
		return 0;

	return 1;
}

static u32 *s3c6410_pm_runcheck(struct resource *res, u32 *val)
{
	void *save_at = phys_to_virt(s3c6410_sleep_save_phys);
	unsigned long addr;
	unsigned long left;
	void *ptr;
	u32 calc;

	for (addr = res->start; addr < res->end;
	     addr += CHECK_CHUNKSIZE) {
		left = res->end - addr;

		if (left > CHECK_CHUNKSIZE)
			left = CHECK_CHUNKSIZE;

		ptr = phys_to_virt(addr);

		if (in_region(ptr, left, crcs, crc_size)) {
			DBG("skipping %08lx, has crc block in\n", addr);
			goto skip_check;
		}

		if (in_region(ptr, left, save_at, 32*4 )) {
			DBG("skipping %08lx, has save block in\n", addr);
			goto skip_check;
		}

		/* calculate and check the checksum */
		calc = crc32_le(~0, ptr, left);
		if (calc != *val) {
			printk(KERN_ERR PFX "Restore CRC error at "
			       "%08lx (%08x vs %08x)\n", addr, calc, *val);

			DBG("Restore CRC error at %08lx (%08x vs %08x)\n",
			    addr, calc, *val);
		}

	skip_check:
		val++;
	}

	return val;
}

/* s3c6410_pm_check_restore
 *
 * check the CRCs after the restore event and free the memory used
 * to hold them
*/

static void s3c6410_pm_check_restore(void)
{
	if (crcs != NULL) {
		s3c6410_pm_run_sysram(s3c6410_pm_runcheck, crcs);
		kfree(crcs);
		crcs = NULL;
	}
}

#else

#define s3c6410_pm_check_prepare() do { } while(0)
#define s3c6410_pm_check_restore() do { } while(0)
#define s3c6410_pm_check_store()   do { } while(0)
#endif

/* helper functions to save and restore register state */

void s3c6410_pm_do_save(struct sleep_save *ptr, int count)
{
	for (; count > 0; count--, ptr++) {
		ptr->val = __raw_readl(ptr->reg);
//		DBG("saved %08lx value %08lx\n", ptr->reg, ptr->val);
	}
}

/* s3c6410_pm_do_restore
 *
 * restore the system from the given list of saved registers
 *
 * Note, we do not use DBG() in here, as the system may not have
 * restore the UARTs state yet
*/

void s3c6410_pm_do_restore(struct sleep_save *ptr, int count)
{
	for (; count > 0; count--, ptr++) {
//		printk(KERN_DEBUG "restore %08lx (restore %08lx, was %08x)\n",
//		       ptr->reg, ptr->val, __raw_readl(ptr->reg));

		__raw_writel(ptr->val, ptr->reg);
	}
}

/* s3c6410_pm_do_restore_core
 *
 * similar to s3c6410_pm_do_restore_core
 *
 * WARNING: Do not put any debug in here that may effect memory or use
 * peripherals, as things may be changing!
*/

static void s3c6410_pm_do_restore_core(struct sleep_save *ptr, int count)
{
	for (; count > 0; count--, ptr++) {
		__raw_writel(ptr->val, ptr->reg);
	}
}

/* s3c6410_pm_show_resume_irqs
 *
 * print any IRQs asserted at resume time (ie, we woke from)
*/

static void s3c6410_pm_show_resume_irqs(int start, unsigned long which,
					unsigned long mask)
{

}

/* s3c6400_pm_check_resume_pin
 *
 * check to see if the pin is configured correctly for sleep mode, and
 * make any necessary adjustments if it is not
*/

static void s3c6410_pm_check_resume_pin(unsigned int pin, unsigned int irqoffs)
{

}

/* s3c6410_pm_configure_extint
 *
 * configure all external interrupt pins
*/

static void s3c6410_pm_configure_extint(void)
{
	__raw_writel(0xfffffff, S3C_EINTMASK);

//	__raw_writel(0xaaaaaaaa, S3C_GPNCON);	// Set EINT configuration
//	__raw_writel(0x0, S3C_EINTCON0);	// Low level detecting (EINT0~15)
//	__raw_writel(0x0, S3C_EINTCON1);	// Low level detecting (EINT16~27)

#if 0
	__raw_writel(__raw_readl(S3C_EINTPEND), S3C_EINTPEND);	// All EINT pending clear
	do {
		__raw_writel(__raw_readl(S3C_EINTPEND), S3C_EINTPEND);	// All EINT pending clear
	} while(__raw_readl(S3C_EINTPEND));
#endif
}

static void s3c6410_pm_config_gpio(void)
{
	/* To reset HS-MMC GPIO */
	__raw_writel(0x0, S3C_GPGCON);
}

static inline void s3c6410_poweroff_misc(void)
{
	unsigned int temp;
	
	__raw_writel(0x0, S3C_RTCCON);

	temp = __raw_readl(S3C_ADCCON);
	__raw_writel(temp |(1<<2), S3C_ADCCON);

	// USB Power Control
	temp = __raw_readl(S3C_SPCON);
	__raw_writel(temp|(1<<3), S3C_SPCON);

	temp = __raw_readl(S3C_OTHERS);
	__raw_writel(temp|(1<<14), S3C_OTHERS);
}

void s3c6410_setup_keypad_wakeup(void)
{

}


void s3c6410_setup_rtc_wakeup(void)
{
#define TESTYEAR	(0x5)
#define TESTMONTH	(0x12)
#define TESTDATE	(0x31)
#define TESTDAY		(0x7)
#define TESTHOUR	(0x23)
#define TESTMIN		(0x59)
#define TESTSEC		(0x59)

#define TESTYEAR2	(0x6)
#define TESTMONTH2	(0x1)
#define TESTDATE2	(0x1)
#define TESTHOUR2	(0x0)
#define TESTMIN2	(0x0)
#define TESTSEC2	(0x0)

	__raw_writel(1,S3C_RTCCON);
	__raw_writel(TESTYEAR,S3C_RTCYEAR);
	__raw_writel(TESTMONTH,S3C_RTCMON);
	__raw_writel(TESTDATE,S3C_RTCDATE);
	__raw_writel(TESTDAY,S3C_RTCDAY);
	__raw_writel(TESTHOUR,S3C_RTCHOUR);
	__raw_writel(TESTMIN,S3C_RTCMIN);
	__raw_writel(TESTSEC,S3C_RTCSEC);
	
	__raw_writel(TESTYEAR2,S3C_ALMYEAR);
	__raw_writel(TESTMONTH2,S3C_ALMMON);
	__raw_writel(TESTDATE2,S3C_ALMDATE);
	__raw_writel(TESTHOUR2,S3C_ALMHOUR);
	__raw_writel(TESTMIN2,S3C_ALMMIN);
	__raw_writel((TESTSEC2 + 0x10),S3C_ALMSEC);

	__raw_writel(0x7f,S3C_RTCALM);
	__raw_writel(0,S3C_RTCCON);
}

unsigned long sleep_phys_sp(void *sp)
{
        return virt_to_phys(sp);
}


#define any_allowed(mask, allow) (((mask) & (allow)) != (allow))


/* s3c6410_pm_enter
 *
 * central control for sleep/resume process
 */
static int s3c6410_pm_enter(suspend_state_t state)
{
	unsigned long regs_save[16];
	unsigned long clkcon, irqindex = 0;
	unsigned int pwr_cfg, sleep_cfg, normal_cfg, hclk_gate, temp, irqno;

	/* ensure the debug is initialised (if enabled) */

//	s3c6410_pm_debug_init();

	if (state != PM_SUSPEND_MEM) {
		printk(KERN_ERR PFX "error: only PM_SUSPEND_MEM supported\n");
		return -EINVAL;
	}

	/* prepare check area if configured */
	s3c6410_pm_check_prepare();

	/* store the physical address of the register recovery block */
	s3c6410_sleep_save_phys = virt_to_phys(regs_save);

	printk("s3c6400_sleep_save_phys=0x%08lx\n", s3c6400_sleep_save_phys);

	/* save all necessary core registers not covered by the drivers */
	s3c6410_pm_do_save(gpio_save, ARRAY_SIZE(gpio_save));
	s3c6410_pm_do_save(irq_save, ARRAY_SIZE(irq_save));
	s3c6410_pm_do_save(core_save, ARRAY_SIZE(core_save));
	s3c6410_pm_do_save(sromc_save, ARRAY_SIZE(sromc_save));
	s3c6410_pm_do_save(uart_save, ARRAY_SIZE(uart_save));
	s3c6410_pm_do_save(lcd_save, ARRAY_SIZE(lcd_save));
	s3c6410_pm_do_save(ts_save, ARRAY_SIZE(ts_save));

	/* ensure INF_REG0  has the resume address */
	__raw_writel(virt_to_phys(s3c6410_cpu_resume), S3C_INFORM0);

	/* set the irq configuration for wake */
	s3c6410_pm_configure_extint();

	s3c6410_poweroff_misc();
	s3c6410_pm_config_gpio();

	s3c6410_pm_check_store();

	__raw_writel(0xffffffff, S3C_VIC0INTENCLEAR);
	__raw_writel(0xffffffff, S3C_VIC1INTENCLEAR);
	__raw_writel(0xffffffff, S3C_VIC0SOFTINTCLEAR);
	__raw_writel(0xffffffff, S3C_VIC1SOFTINTCLEAR);


	s3c6410_setup_keypad_wakeup();

//	s3c6410_setup_rtc_wakeup();
 
	__raw_writel(1, S3C_OSC_STABLE);
	__raw_writel(1, S3C_PWR_STABLE);
	
	/* Set WFI instruction to SLEEP mode */

	pwr_cfg = __raw_readl(S3C_PWR_CFG);
	pwr_cfg &= ~(0x60<<0);
	pwr_cfg |= (0x3<<5);
	__raw_writel(pwr_cfg, S3C_PWR_CFG);

	sleep_cfg = __raw_readl(S3C_SLEEP_CFG);
	sleep_cfg &= ~(0x61<<0);
	__raw_writel(sleep_cfg, S3C_SLEEP_CFG);


	__raw_writel((0x0fffffff&~(1<<9)), S3C_EINT_MASK);

	__raw_writel(0x2, S3C_SLPEN);
	

	/* ALL sub block "ON" before enterring sleep mode - EVT0 bug*/
	__raw_writel(0xffffff00, S3C_NORMAL_CFG);

	/* Open all clock gate to enter sleep mode - EVT0 bug*/
	__raw_writel(0xffffffff, S3C_HCLK_GATE);
	__raw_writel(0xffffffff, S3C_PCLK_GATE);
	__raw_writel(0xffffffff, S3C_SCLK_GATE);

	
	s3c6400_cpu_suspend(regs_save);

	/* restore the cpu state */
	cpu_init();

	/* restore the system state */
	s3c6410_pm_do_restore_core(core_save, ARRAY_SIZE(core_save));
	s3c6410_pm_do_restore(sromc_save, ARRAY_SIZE(sromc_save));
	s3c6410_pm_do_restore(gpio_save, ARRAY_SIZE(gpio_save));
	s3c6410_pm_do_restore(irq_save, ARRAY_SIZE(irq_save));
	s3c6410_pm_do_restore(uart_save, ARRAY_SIZE(uart_save));
	s3c6410_pm_do_restore(lcd_save, ARRAY_SIZE(lcd_save));
	s3c6410_pm_do_restore(ts_save, ARRAY_SIZE(ts_save));

	temp = __raw_readl(S3C_EINTPEND);
	__raw_writel(temp, S3C_EINTPEND);

	/* For writing the IRQ number into the VICVECTADDR */
	for (irqno = IRQ_EINT0_3; irqno <= IRQ_LCD_SYSTEM; irqno++) {
		__raw_writel(irqno, S3C_VIC0VECTADDR0 + irqindex);
		irqindex = irqindex + 4;
	}

	irqindex = 0;
	for (irqno = IRQ_EINT12_19; irqno <= IRQ_ADC; irqno++) {
		__raw_writel(irqno, S3C_VIC1VECTADDR0 + irqindex);
		irqindex = irqindex + 4;
	}

	s3c6410_pm_debug_init();

	DBG("post sleep, preparing to return\n");

	s3c6410_pm_check_restore();

	/* ok, let's return from sleep */
	DBG("S3C6410 PM Resume (post-restore)\n");

	return 0;

}

/*
 * Called after processes are frozen, but before we shut down devices.
 */
static int s3c6410_pm_prepare(suspend_state_t state)
{
	printk(" preparing for pwrdown\n");
	return 0;
}

/*
 * Called after devices are re-setup, but before processes are thawed.
 */
static int s3c6410_pm_finish(suspend_state_t state)
{
	return 0;
}

/*
 * Set to PM_DISK_FIRMWARE so we can quickly veto suspend-to-disk.
 */
static struct pm_ops s3c6410_pm_ops = {
	.pm_disk_mode	= PM_DISK_FIRMWARE,
	.prepare	= s3c6410_pm_prepare,
	.enter		= s3c6410_pm_enter,
	.finish		= s3c6410_pm_finish,
};

int __init s3c6410_pm_init(void)
{
	printk("S3C6410 Power Management, (c) 2006 Samsung Electronics\n");

	enable_irq_wake(IRQ_EINT9);
	enable_irq_wake(IRQ_RTC_ALARM);

	pm_set_ops(&s3c6410_pm_ops);

	return 0;
}

module_init(s3c6410_pm_init);
