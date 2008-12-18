/* linux/arch/arm/mach-s3c64xx/irq-pl192.c
 *
 *
 * S3C64XX interrupt functions.
 * Added by JaeCheol Lee(jc.lee@samsung.com)
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/ptrace.h>
#include <linux/sysdev.h>

#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/io.h>

#include <asm/mach/irq.h>

#include <asm/arch/gpio.h>
#include <asm/arch/regs-irq.h>
#include <asm/arch/regs-gpio.h>
#include <asm/arch/regs-s3c-clock.h>
#include <asm/plat-s3c24xx/cpu.h>
#include <asm/plat-s3c24xx/pm.h>

#define irqdbf(x...)
#define irqdbf2(x...)


/* "irqno" interrupt acked */
static inline void
s3c_irq_ack (unsigned int irqno)
{
	if (irqno < 32) {
		__raw_writel(irqno, S3C_VIC0ADDRESS);
	} else if (irqno < 64) {
		__raw_writel(irqno, S3C_VIC0ADDRESS);
		__raw_writel(irqno, S3C_VIC1ADDRESS);
	} else {
		/* External interrupt */
		printk("ext irq %d\n", irqno);
	}
}

/* "irqno" interrupt disabled */
static inline void
s3c_irq_mask (unsigned int irqno)
{
	if (irqno < 32) {
		__raw_writel(1<<irqno, S3C_VIC0INTENCLEAR);
	} else if (irqno < 64) {
		__raw_writel(1<<(irqno-32), S3C_VIC1INTENCLEAR);
	} else {
		/* External interrupt */
		printk("ext irq %d\n", irqno);
	}
}

/* "irqno" interrupt disabled and acked */
static inline void
s3c_irq_maskack (unsigned int irqno)
{
	if (irqno < 32) {
		__raw_writel(1<<irqno, S3C_VIC0INTENCLEAR);
		__raw_writel(irqno, S3C_VIC0ADDRESS);
	} else if (irqno < 64) {
		__raw_writel(1<<(irqno-32), S3C_VIC1INTENCLEAR);
		/* XXX: can explain why this code is here by scsuh */
		__raw_writel(irqno, S3C_VIC0ADDRESS);
		__raw_writel(irqno, S3C_VIC1ADDRESS);
	} else {
		/* External interrupt */
		printk("ext irq %d\n", irqno);
	}
}

/* "irqno" interrupt enabled */
static inline void
s3c_irq_unmask (unsigned int irqno)
{
	if (irqno < 32) {
		__raw_writel(1<<irqno, S3C_VIC0INTENABLE);
	} else if (irqno < 64) {
		__raw_writel(1<<(irqno-32), S3C_VIC1INTENABLE);
	} else {
		/* External Interrupt */
		printk("ext irq %d\n", irqno);
	}
}

static inline void
s3c_irq_unmaskack(unsigned int irqno)
{
	unsigned long unmask;

	/* It is common for all interrupt sources */
	__raw_writel(1, S3C_VIC0ADDRESS);

	if (irqno < 32) {
		unmask = __raw_readl(S3C_VIC0INTENABLE);
		unmask |= (1UL << irqno);
		__raw_writel(unmask, S3C_VIC0INTENABLE);
	} else if (irqno < 64) {
		unmask = __raw_readl(S3C_VIC1INTENABLE);
		unmask |= (1UL << (irqno - 32));
		__raw_writel(unmask, S3C_VIC1INTENABLE);
		__raw_writel(1, S3C_VIC1ADDRESS);
	} else {
		/* External Interrupt */
		printk("ext irq %d\n", irqno);

	}
}

static int
s3c_irq_wake(unsigned int irqno, unsigned int flag)
{
	unsigned int pwr_cfg,bit_position;

	pwr_cfg = __raw_readl(S3C_PWR_CFG);

	switch(irqno) {
	case IRQ_RTC_TIC:
		bit_position = 11;
		break;
	case IRQ_TC:
		bit_position = 12;
		break;
	case IRQ_RTC_ALARM:
		bit_position = 10;
		break;
	case IRQ_KEYPAD:
		bit_position = 8;
		break;
	default:
		printk("irq %d : Not supporting wakeup \n", irqno);
		return -1;
	}

	if(flag) {
		pwr_cfg &=~(1 << bit_position);
	} else {
		pwr_cfg |= (1 << bit_position);
	}
	__raw_writel(pwr_cfg, S3C_PWR_CFG);

	return 0;
}

//static struct irqchip s3c_irq_level_chip = {
static struct irq_chip s3c_irq_level_chip = {
	.ack = s3c_irq_maskack,
	.mask = s3c_irq_mask,
	.unmask = s3c_irq_unmask,
	.set_wake = s3c_irq_wake
};

#if 0	/* XXX: when you want to use it, unmask it. by scsuh */
static struct irqchip s3c_irq_chip = {
	.ack = s3c_irq_mask,
	.mask = s3c_irq_mask,
	.unmask = s3c_irq_unmask,
	.set_wake = s3c_irq_wake
};
#endif

static inline void
s3c_irqext_mask(unsigned int irqno)
{
	unsigned long gpjcon;	// Workaround
	unsigned long mask;

	if (irqno >= 64) {
		gpjcon = __raw_readl(S3C_GPJCON);
		irqno -= 64;
		mask = __raw_readl(S3C_EINTMASK);
		mask |= (1UL << irqno);
		__raw_writel(mask, S3C_EINTMASK);
		__raw_writel(gpjcon, S3C_GPJCON);
	} else {
		printk("Invalid ext irq %d\n", irqno);
	}
}

static inline void
s3c_irqext_maskack(unsigned int irqno)
{
	s3c_irqext_mask(irqno);

	if (irqno <= IRQ_EINT3) {
		s3c_irq_mask(IRQ_EINT0_3);
	} else if (irqno <= IRQ_EINT11) {
		s3c_irq_mask(IRQ_EINT4_11);
	} else if (irqno <= IRQ_EINT19) {
		s3c_irq_mask(IRQ_EINT12_19);
	} else if (irqno <= IRQ_EINT27) {
		s3c_irq_mask(IRQ_EINT20_27);
	} else if (irqno > IRQ_EINT27) {
		panic("Wrong IRQ number %d \n ", irqno);
	}

}

static inline void
s3c_irqext_unmaskack(unsigned int irqno)
{
	unsigned long mask;

	if (irqno <= IRQ_EINT3) {
		s3c_irq_unmaskack(IRQ_EINT0_3);
	} else if (irqno <= IRQ_EINT11) {
		s3c_irq_unmaskack(IRQ_EINT4_11);
	} else if (irqno <= IRQ_EINT19) {
		s3c_irq_unmaskack(IRQ_EINT12_19);
	} else if (irqno <= IRQ_EINT27) {
		s3c_irq_unmaskack(IRQ_EINT20_27);
	}
	if (irqno >= 64) {
		irqno -= 64;
		__raw_writel(1UL << irqno, S3C_EINTPEND);
		mask = __raw_readl(S3C_EINTMASK);
		mask &= ~(1UL << irqno);
		__raw_writel(mask, S3C_EINTMASK);
	} else {
		printk("Invalid ext irq %d\n", irqno);
	}

}


static int
s3c_irqext_type(unsigned int irq, unsigned int type)
{

	unsigned long newvalue = 0;


	/* Set the external interrupt to pointed trigger type */
	switch (type) {
	case IRQT_NOEDGE:
		printk(KERN_WARNING "No edge setting!\n");
		break;

	case IRQT_RISING:
		newvalue = S3C_EXTINT_RISEEDGE;
		break;

	case IRQT_FALLING:
		newvalue = S3C_EXTINT_FALLEDGE;
		break;

	case IRQT_BOTHEDGE:
		newvalue = S3C_EXTINT_BOTHEDGE;
		break;

	case IRQT_LOW:
		newvalue = S3C_EXTINT_LOWLEV;
		break;

	case IRQT_HIGH:
		newvalue = S3C_EXTINT_HILEV;
		break;

	default:
		printk(KERN_ERR "No such irq type %d", type);
		return -1;
	}

	switch (irq) {
	case IRQ_EINT9:
		gpio_set_pin(S3C_GPN9, S3C_GPN9_EXTINT9);
		__raw_writel((__raw_readl(S3C_EINTCON0) & ~(0x7 << 16)) |
			     (newvalue << 16), S3C_EINTCON0);
		break;

	case IRQ_EINT10:
		gpio_set_pin(S3C_GPN10, S3C_GPN10_EXTINT10);
		__raw_writel((__raw_readl(S3C_EINTCON0) & ~(0x7 << 20)) |
			     (newvalue << 20), S3C_EINTCON0);
		break;

	case IRQ_EINT11:
		gpio_set_pin(S3C_GPN11, S3C_GPN11_EXTINT11);
		__raw_writel((__raw_readl(S3C_EINTCON0) & ~(0x7 << 20)) |
			     (newvalue << 20), S3C_EINTCON0);
		break;
	case IRQ_EINT12:
		gpio_set_pin(S3C_GPN12, S3C_GPN12_EXTINT12);
		__raw_writel((__raw_readl(S3C_EINTCON0) & ~(0x7 << 24)) |
			     (newvalue << 24), S3C_EINTCON0);
		break;

	case IRQ_EINT13:
		gpio_set_pin(S3C_GPN13, S3C_GPN13_EXTINT13);
		__raw_writel((__raw_readl(S3C_EINTCON0) & ~(0x7 << 24)) |
			     (newvalue << 24), S3C_EINTCON0);
		break;

	case IRQ_EINT14:
		gpio_set_pin(S3C_GPN14, S3C_GPN14_EXTINT14);
		__raw_writel((__raw_readl(S3C_EINTCON0) & ~(0x7 << 28)) |
			     (newvalue << 28), S3C_EINTCON0);
		break;

	case IRQ_EINT15:
		gpio_set_pin(S3C_GPN15, S3C_GPN15_EXTINT15);
		__raw_writel((__raw_readl(S3C_EINTCON0) & ~(0x7 << 28)) |
			     (newvalue << 28), S3C_EINTCON0);
		break;	
	default:
		printk(KERN_ERR
		       "s3c_irqext_type : Only support EINT9,10,11,12,13,14,15 interrupt.\n");
		break;

	}

	return 0;
}

static int s3c_irqext_wake(unsigned int irqno, unsigned int flag)
{
	unsigned int eint_mask;

	if(irqno < IRQ_EINT0)
		printk(KERN_ERR "Check external wake-up source\n");

	eint_mask = __raw_readl(S3C_EINT_MASK);

	if(flag) {
		eint_mask &= ~(1 << (irqno - IRQ_EINT0));
	} else {
		eint_mask |= (1 << (irqno - IRQ_EINT0));
	}

	__raw_writel(eint_mask, S3C_EINT_MASK);

	return 0;
}

//static struct irqchip s3c_irqext_chip = {
static struct irq_chip s3c_irqext_chip = {
	.mask = s3c_irqext_mask,
	.unmask = s3c_irqext_unmaskack,
	.ack = s3c_irqext_maskack,
	.set_type = s3c_irqext_type,
	.set_wake = s3c_irqext_wake
};


/* irq demux for EINT0_3 */
static inline void
s3c_irq_demux_eint0_3(unsigned int irq, struct irq_desc *desc)
{
	unsigned int eintpend, eintmsk;
	unsigned int irqnr = IRQ_EINT0;
	unsigned char interruptPending = 0, count = 0;
	struct irq_desc *mydesc;

	/* read the current pending interrupts, and the mask
	 * for what it is available */
	eintpend = __raw_readl(S3C_EINTPEND);
	eintmsk = __raw_readl(S3C_EINTMASK);

	eintpend &= ~eintmsk;

	interruptPending = (eintpend & 0xf);
	while (interruptPending != 0) {
		if (interruptPending & 0x1) {
			mydesc = irq_desc + (irqnr + count);
			//mydesc->handle((irqnr + count), mydesc, regs);
			desc_handle_irq((irqnr + count), mydesc);
		}
		interruptPending >>= 1;
		count++;
	}

}

/* irq demux for EINT4_11 */
static void
s3c_irq_demux_eint4_11(unsigned int irq, struct irq_desc *desc)
{
	unsigned int eintpend, eintmsk;
	unsigned int irqnr = IRQ_EINT4;
	unsigned char interruptPending = 0, count = 0;
	struct irq_desc *mydesc;

	/* read the current pending interrupts, and the mask
	 * for what it is available */
	eintpend = __raw_readl(S3C_EINTPEND);
	eintmsk = __raw_readl(S3C_EINTMASK);

	eintpend &= ~eintmsk;

	interruptPending = ((eintpend >> 4) & 0xff);
	while (interruptPending != 0) {
		if (interruptPending & 0x1) {
			mydesc = irq_desc + (irqnr + count);
			//mydesc->handle((irqnr + count), mydesc, regs);
			desc_handle_irq((irqnr + count), mydesc);
		}
		interruptPending >>= 1;
		count++;
	}
}

/* irq demux for EINT12_19 */
static void
s3c_irq_demux_eint12_19(unsigned int irq, struct irq_desc *desc)
{
	unsigned int eintpend, eintmsk;
	unsigned int irqnr = IRQ_EINT12;
	unsigned char interruptPending = 0, count = 0;
	struct irq_desc *mydesc;

	/* read the current pending interrupts, and the mask
	 * for what it is available */
	eintpend = __raw_readl(S3C_EINTPEND);
	eintmsk = __raw_readl(S3C_EINTMASK);

	eintpend &= ~eintmsk;

	interruptPending = ((eintpend >> 12) & 0xff);
	while (interruptPending != 0) {
		if (interruptPending & 0x1) {
			mydesc = irq_desc + (irqnr + count);
			//mydesc->handle((irqnr + count), mydesc, regs);
			desc_handle_irq((irqnr + count), mydesc);
		}
		interruptPending >>= 1;
		count++;
	}

}

/* irq demux for EINT20_27 */
static void
s3c_irq_demux_eint20_27 (unsigned int irq, struct irq_desc *desc)
{
	unsigned int eintpend, eintmsk;
	unsigned int irqnr = IRQ_EINT20;
	unsigned char interruptPending = 0, count = 0;
	struct irq_desc *mydesc;

	/* read the current pending interrupts, and the mask
	 * for what it is available */
	eintpend = __raw_readl(S3C_EINTPEND);
	eintmsk = __raw_readl(S3C_EINTMASK);

	eintpend &= ~eintmsk;

	interruptPending = ((eintpend >> 20) & 0xff);
	while (interruptPending != 0) {
		if (interruptPending & 0x1) {
			mydesc = irq_desc + (irqnr + count);
			//mydesc->handle((irqnr + count), mydesc, regs);
			desc_handle_irq((irqnr + count), mydesc);
		}
		interruptPending >>= 1;
		count++;
	}
}

#ifdef CONFIG_PM

static struct sleep_save irq_save[] = {
	SAVE_ITEM(S3C_VIC0INTSELECT),
	SAVE_ITEM(S3C_VIC1INTSELECT),
	SAVE_ITEM(S3C_VIC0INTENABLE),
	SAVE_ITEM(S3C_VIC1INTENABLE),
	SAVE_ITEM(S3C_VIC0SOFTINT),
	SAVE_ITEM(S3C_VIC1SOFTINT),
};

static struct sleep_save extirq_save[] = {
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
};

int s3c24xx_irq_suspend(struct sys_device *dev, pm_message_t state)
{
	s3c2410_pm_do_save(extirq_save, ARRAY_SIZE(extirq_save));
	s3c2410_pm_do_save(irq_save, ARRAY_SIZE(irq_save));
	return 0;
}

int s3c24xx_irq_resume(struct sys_device *dev)
{
	int irqno;
	int irqindex = 0;
	
	s3c2410_pm_do_restore(extirq_save, ARRAY_SIZE(extirq_save));
	s3c2410_pm_do_restore(irq_save, ARRAY_SIZE(irq_save));
	
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
	return 0;
}

#else
#define s3c24xx_irq_suspend NULL
#define s3c24xx_irq_resume  NULL
#endif


/* --------------------------------------------------
 *  s3c_init_irq
 *
 *  Initialise s3c6400 IRQ system
 * --------------------------------------------------
 */

void __init s3c_init_irq(void)
{
	int irqno;
	int irqindex = 0;

	irqdbf("s3c_init_irq: clearing interrupt status flags\n");

	/* first, clear all interrupts pending... */

	/* clear external interrupts */
	__raw_writel(0xFFFFFFFF, S3C_EINTPEND);

	/* clear all vector interrupts */
	__raw_writel(0x00000000, S3C_VIC0ADDRESS);
	__raw_writel(0x00000000, S3C_VIC1ADDRESS);


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

	/* register the main interrupts */
	irqdbf("s3c_init_irq: registering S3C64XX interrupt handlers\n");

	for (irqno = IRQ_EINT0_3; irqno <= IRQ_ADC; irqno++) {
		switch (irqno) {
			/* deal with the special IRQs in ext (cascaded) */
		case IRQ_EINT0_3:
			set_irq_chained_handler(IRQ_EINT0_3, s3c_irq_demux_eint0_3);
			break;

		case IRQ_EINT4_11:
			set_irq_chained_handler(IRQ_EINT4_11, s3c_irq_demux_eint4_11);
			break;

		case IRQ_EINT12_19:
			set_irq_chained_handler(IRQ_EINT12_19, s3c_irq_demux_eint12_19);
			break;

		case IRQ_EINT20_27:
			set_irq_chained_handler(IRQ_EINT20_27, s3c_irq_demux_eint20_27);
			break;

		default:
			irqdbf("registering irq %d (s3c irq)\n", irqno);
			set_irq_chip(irqno, &s3c_irq_level_chip);
			//set_irq_handler(irqno, do_level_IRQ);
			set_irq_handler(irqno, handle_level_irq);
			set_irq_flags(irqno, IRQF_VALID);
			break;
		}
	}

	for (irqno = IRQ_EINT0; irqno <= IRQ_EINT27; irqno++) {
		irqdbf("registering irq %d (extended s3c irq)\n", irqno);
		set_irq_chip(irqno, &s3c_irqext_chip);
		//set_irq_handler(irqno, do_level_IRQ);
		set_irq_handler(irqno, handle_level_irq);
		set_irq_flags(irqno, IRQF_VALID);
	}

	irqdbf("s3c64XX: registered interrupt handlers\n");
}
