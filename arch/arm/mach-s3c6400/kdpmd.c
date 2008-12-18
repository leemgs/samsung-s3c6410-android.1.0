#include <linux/version.h>
#include <linux/module.h>
#if defined(MODVERSIONS)
#include <linux/modversions.h>
#endif
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <asm/arch/kdpmd.h>
#include <asm/arch/pd.h>

#define _DEBUG	1
//#undef _DEBUG

#ifdef _DEBUG
#include <linux/time.h>
#endif

/* global variables for kdpmd */
DECLARE_MUTEX(kdpmd_sem);
unsigned int kdpmd_ev;
wait_queue_head_t kdpmd_wq;
unsigned int dd_ev;
wait_queue_head_t dd_wq;
unsigned int wakeup_source;

struct task_struct *kp;

void handle_timeout(void)
{
	printk(KERN_INFO "kdpmd timeout\n");
	/* dvfs code should go here */
}

/* this function should be called before device drivers turn on
   their clocks */
void handle_drvopen(struct pm_pdtype *pd)
{
	if (pd->state == PDSTATE_OFF) {
		printk("Turning on %s pd\n", pd->name);
		/* power on and device initializaion */
		pd_on(pd);
	}
	/* clock control is done by the device driver */
}

/* this function should be called after device drivers turn off
   their clocks and update pm_devtype->state value 

   If this function takes too much time, we may employ separate
   lists for RUNNING devices and IDLE devices in a power domain */
void handle_drvclose(struct pm_pdtype *pd)
{
        struct list_head *temp;
        struct pm_devtype *pdev;

	/* check this power domain */
	list_for_each(temp, &pd->devhead) {
		pdev = list_entry(temp, struct pm_devtype, entry);
		if (pdev->state == DEV_RUNNING)
			return;
	}
	/* if all devices under this power domain are not RUNNING */
	pd_off(pd);
}

/* this is the thread function that we are executing */
static int kdpmd_thread(void *data)
{
	struct pm_pdtype *pd;
	unsigned int i = 0;
#ifdef _DEBUG
	struct timeval tv1, tv2;
#endif

	printk("Kernel DPM daemon thread started\n");

	init_waitqueue_head(&kdpmd_wq);
	init_waitqueue_head(&dd_wq);

	/* an endless loop in which we are doing our work */
	for (;;) {
		i++;
		/* fall asleep, wait for other processes to write to kdpmd_ev */
		if (wait_event_interruptible(kdpmd_wq, kdpmd_ev != 0) < 0) {
			printk(KERN_INFO "kdpmd wait returned -\n");
			return -ERESTARTSYS;
		}

#ifdef _DEBUG
		do_gettimeofday(&tv1);
#endif

		/* What if some other device opens up while servicing other */
		kdpmd_lock();

		/* We need to do a memory barrier here to be sure that
		   the flags are visible on all CPUs. 
		 */
		mb();

		/* here we are back from sleep, either due to the timeout
		   (2 seconds), or because we caught a signal.
		 */
		if (kthread_should_stop())
			break;

		printk(KERN_INFO "kdpmd woke up: %3d\n", i);

		/* find out who woke me up */
		pd = pd_hashtbl[kdpmd_ev];

		/* find out why woke me up */
		switch (wakeup_source) {
		case KDPMD_TIMEOUT :
			handle_timeout();
			goto sleepagain;
		case KDPMD_DRVOPEN :
			handle_drvopen(pd);
			break;
		case KDPMD_DRVCLOSE :
			handle_drvclose(pd);
			break;
		case KDPMD_REMOVE :	// handle rmmod call
			/* wait for 2 sec hoping that kthread_should_stop() 
			   will evaluate to true */
			wait_event_timeout(kdpmd_wq, kdpmd_ev != 0, 2 * HZ);
			goto sleepagain;
		default :
			break;
		}
		dd_ev = kdpmd_ev;
		wake_up_interruptible(&dd_wq);
sleepagain:
		kdpmd_ev = 0;
		kdpmd_unlock();

#ifdef _DEBUG
		do_gettimeofday(&tv2);
		printk(KERN_DEBUG " %d sec %d usec\n\n",
		       (int)tv2.tv_sec - (int)tv1.tv_sec,
		       (int)tv2.tv_usec - (int)tv1.tv_usec);
#endif
	}
	/* here we go only in case of termination of the thread */
	/* returning from the thread here calls the exit functions */
	return (0);
}

void kdpmd_set_event(unsigned int devid, unsigned int drv_op)
{
	kdpmd_lock();
	wakeup_source = drv_op;
	kdpmd_ev = devid;
	kdpmd_unlock();
}

void kdpmd_wakeup(void)
{
	wake_up_interruptible(&kdpmd_wq);
}

int kdpmd_wait(unsigned int devid)
{
	int ret;

	ret = wait_event_interruptible(dd_wq, dd_ev==devid);
	dd_ev = 0;

	return ret;
}

int __init kdpmd_init(void)
{
	init_MUTEX(&kdpmd_sem);
	kp = kthread_run(kdpmd_thread, NULL, "kdpmd");
	printk(KERN_INFO "kdpmd created\n");

	return (0);
}

void __exit kdpmd_exit(void)
{
	printk(KERN_INFO "kdpmd being destroyed\n");
	kdpmd_set_event(KDPMD_REMOVE, KDPMD_REMOVE);
	kdpmd_wakeup();
	/* terminate the kernel thread */
	kthread_stop(kp);

	return;
}

#ifdef MODULE
module_init(kdpmd_init);
module_exit(kdpmd_exit);
#else
__initcall(kdpmd_init);
#endif

EXPORT_SYMBOL(kdpmd_set_event);
EXPORT_SYMBOL(kdpmd_wakeup);
EXPORT_SYMBOL(kdpmd_wait);

MODULE_AUTHOR("Ikhwan Lee");
MODULE_LICENSE("Dual BSD/GPL");

