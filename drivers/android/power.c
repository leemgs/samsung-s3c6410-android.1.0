/* drivers/android/power.c
 *
 * Copyright (C) 2005-2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/list.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
//#include <linux/platform_device.h>
#include <linux/sysdev.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/wait.h>
#include <linux/android_power.h>
#include <linux/suspend.h>
#include <linux/syscalls.h> // sys_sync
#include <linux/console.h>
#include <linux/kbd_kern.h>
#include <linux/vt_kern.h>
#include <linux/freezer.h>
#ifdef CONFIG_ANDROID_POWER_STAT
#include <linux/proc_fs.h>
#endif

#define ANDROID_POWER_TEST_EARLY_SUSPEND 0
#define ANDROID_POWER_PRINT_USER_WAKE_LOCKS 0

MODULE_DESCRIPTION("OMAP CSMI Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

#define ANDROID_SUSPEND_CONSOLE	(MAX_NR_CONSOLES-2)

static spinlock_t g_list_lock = SPIN_LOCK_UNLOCKED;
static DEFINE_MUTEX(g_early_suspend_lock);

wait_queue_head_t g_wait_queue;

static LIST_HEAD(g_inactive_locks);
static LIST_HEAD(g_active_partial_wake_locks);
static LIST_HEAD(g_active_full_wake_locks);
static LIST_HEAD(g_early_suspend_handlers);
static enum {
	USER_AWAKE,
	USER_NOTIFICATION,
	USER_SLEEP
} g_user_suspend_state;
static int g_current_event_num;
static struct workqueue_struct *g_suspend_work_queue;
static void android_power_suspend(struct work_struct *work);
static void android_power_wakeup_locked(int notification, ktime_t time);
static DECLARE_WORK(g_suspend_work, android_power_suspend);
static int g_max_user_lockouts = 16;

//static const char g_free_user_lockout_name[] = "free_user";
static struct {
	enum {
		USER_WAKE_LOCK_INACTIVE,
		USER_WAKE_LOCK_PARTIAL,
		USER_WAKE_LOCK_FULL
	}                       state;
	android_suspend_lock_t  suspend_lock;
	char                    name_buffer[32];
} *g_user_wake_locks;
#ifdef CONFIG_ANDROID_POWER_STAT
android_suspend_lock_t g_deleted_wake_locks;
android_suspend_lock_t g_no_wake_locks;
#endif
static struct kobject *android_power_subsys;
//static struct kset *android_power_subsys;
#ifndef CONFIG_FRAMEBUFFER_CONSOLE
static wait_queue_head_t fb_state_wq;
static spinlock_t fb_state_lock = SPIN_LOCK_UNLOCKED;
int fb_state;
#endif

#if 0
android_suspend_lock_t *android_allocate_suspend_lock(const char *debug_name)
{
	unsigned long irqflags;
	struct android_power *e;

	e = kzalloc(sizeof(*e), GFP_KERNEL);
	if(e == NULL) {
		printk("android_power_allocate: kzalloc failed\n");
		return NULL;
	}
	e->name = debug_name;
	spin_lock_irqsave(&g_list_lock, irqflags);
	list_add(&e->link, &g_allocated);
	spin_unlock_irqrestore(&g_list_lock, irqflags);
	return e;
}
#endif

int android_init_suspend_lock(android_suspend_lock_t *lock)
{
	unsigned long irqflags;

	if(lock->name == NULL) {
		printk("android_init_suspend_lock: error name=NULL, lock=%p\n", lock);
		dump_stack();
		return -EINVAL;
	}

	//printk("android_init_suspend_lock name=%s\n", lock->name);
#ifdef CONFIG_ANDROID_POWER_STAT
	lock->stat.count = 0;
	lock->stat.expire_count = 0;
	lock->stat.total_time = ktime_set(0, 0);
	lock->stat.max_time = ktime_set(0, 0);
	lock->stat.last_time = ktime_set(0, 0);
#endif
	lock->flags = 0;

	INIT_LIST_HEAD(&lock->link);
	spin_lock_irqsave(&g_list_lock, irqflags);
	list_add(&lock->link, &g_inactive_locks);
	spin_unlock_irqrestore(&g_list_lock, irqflags);	
//	if(lock->flags & ANDROID_SUSPEND_LOCK_FLAG_USER_VISIBLE_MASK) {
//		sysfs_create_file(struct kobject * k, const struct attribute * a)
//	}
	return 0;
}

void android_uninit_suspend_lock(android_suspend_lock_t *lock)
{
	unsigned long irqflags;
	//printk("android_uninit_suspend_lock name=%s\n", lock->name);
	spin_lock_irqsave(&g_list_lock, irqflags);
#ifdef CONFIG_ANDROID_POWER_STAT
	if(lock->stat.count) {
		if(g_deleted_wake_locks.stat.count == 0) {
			g_deleted_wake_locks.name = "deleted_wake_locks";
			android_init_suspend_lock(&g_deleted_wake_locks);
		}
		g_deleted_wake_locks.stat.count += lock->stat.count;
		g_deleted_wake_locks.stat.expire_count += lock->stat.expire_count;
		g_deleted_wake_locks.stat.total_time = ktime_add(g_deleted_wake_locks.stat.total_time, lock->stat.total_time);
		g_deleted_wake_locks.stat.max_time = ktime_add(g_deleted_wake_locks.stat.max_time, lock->stat.max_time);
	}
#endif
	list_del(&lock->link);
	spin_unlock_irqrestore(&g_list_lock, irqflags);	
}

void android_lock_suspend(android_suspend_lock_t *lock)
{
	unsigned long irqflags;
	//printk("android_lock_suspend name=%s\n", lock->name);
	spin_lock_irqsave(&g_list_lock, irqflags);
#ifdef CONFIG_ANDROID_POWER_STAT
	if(!(lock->flags & ANDROID_SUSPEND_LOCK_ACTIVE)) {
		lock->flags |= ANDROID_SUSPEND_LOCK_ACTIVE;
		lock->stat.last_time = ktime_get();
	}
#endif
	lock->expires = INT_MAX;
	lock->flags &= ~ANDROID_SUSPEND_LOCK_AUTO_EXPIRE;
	list_del(&lock->link);
	list_add(&lock->link, &g_active_partial_wake_locks);
	g_current_event_num++;
	spin_unlock_irqrestore(&g_list_lock, irqflags);
}

void android_lock_suspend_auto_expire(android_suspend_lock_t *lock, int timeout)
{
	unsigned long irqflags;
	//printk("android_lock_suspend name=%s\n", lock->name);
	spin_lock_irqsave(&g_list_lock, irqflags);
#ifdef CONFIG_ANDROID_POWER_STAT
	if(!(lock->flags & ANDROID_SUSPEND_LOCK_ACTIVE)) {
		lock->flags |= ANDROID_SUSPEND_LOCK_ACTIVE;
		lock->stat.last_time = ktime_get();
	}
#endif
	lock->expires = jiffies + timeout;
	lock->flags |= ANDROID_SUSPEND_LOCK_AUTO_EXPIRE;
	list_del(&lock->link);
	list_add(&lock->link, &g_active_partial_wake_locks);
	g_current_event_num++;
	wake_up(&g_wait_queue);
	spin_unlock_irqrestore(&g_list_lock, irqflags);
}

void android_lock_partial_suspend_auto_expire(android_suspend_lock_t *lock, int timeout)
{
	unsigned long irqflags;
	//printk("android_lock_suspend name=%s\n", lock->name);
	spin_lock_irqsave(&g_list_lock, irqflags);
#ifdef CONFIG_ANDROID_POWER_STAT
	if(!(lock->flags & ANDROID_SUSPEND_LOCK_ACTIVE)) {
		lock->flags |= ANDROID_SUSPEND_LOCK_ACTIVE;
		lock->stat.last_time = ktime_get();
	}
#endif
	lock->expires = jiffies + timeout;
	lock->flags |= ANDROID_SUSPEND_LOCK_AUTO_EXPIRE;
	list_del(&lock->link);
	list_add(&lock->link, &g_active_full_wake_locks);
	g_current_event_num++;
	wake_up(&g_wait_queue);
	android_power_wakeup_locked(1, ktime_get());
	spin_unlock_irqrestore(&g_list_lock, irqflags);
}

#ifdef CONFIG_ANDROID_POWER_STAT
static int print_lock_stat(char *buf, android_suspend_lock_t *lock)
{
	ktime_t active_time;
	if(lock->flags & ANDROID_SUSPEND_LOCK_ACTIVE)
		active_time = ktime_sub(ktime_get(), lock->stat.last_time);
	else
		active_time = ktime_set(0, 0);
	return sprintf(buf, "\"%s\"\t%d\t%d\t%lld\t%lld\t%lld\t%lld\n",
	               lock->name,
	               lock->stat.count, lock->stat.expire_count,
	               ktime_to_ns(active_time),
	               ktime_to_ns(lock->stat.total_time),
	               ktime_to_ns(lock->stat.max_time),
	               ktime_to_ns(lock->stat.last_time));
}


static int wakelocks_read_proc(char *page, char **start, off_t off,
                               int count, int *eof, void *data)
{
	unsigned long irqflags;
	android_suspend_lock_t *lock;
	int len = 0;
	char *p = page;

	spin_lock_irqsave(&g_list_lock, irqflags);

	p += sprintf(p, "name\tcount\texpire_count\tactive_since\ttotal_time\tmax_time\tlast_change\n");
	list_for_each_entry(lock, &g_inactive_locks, link) {
		p += print_lock_stat(p, lock);
	}
	list_for_each_entry(lock, &g_active_partial_wake_locks, link) {
		p += print_lock_stat(p, lock);
	}
	list_for_each_entry(lock, &g_active_full_wake_locks, link) {
		p += print_lock_stat(p, lock);
	}
	spin_unlock_irqrestore(&g_list_lock, irqflags);
	

	*start = page + off;

	len = p - page;
	if (len > off)
		len -= off;
	else
		len = 0;

	return len < count ? len  : count;
}

static void android_unlock_suspend_stat_locked(android_suspend_lock_t *lock)
{
	if(lock->flags & ANDROID_SUSPEND_LOCK_ACTIVE) {
		ktime_t duration;
		lock->flags &= ~ANDROID_SUSPEND_LOCK_ACTIVE;
		lock->stat.count++;
		duration = ktime_sub(ktime_get(), lock->stat.last_time);
		lock->stat.total_time = ktime_add(lock->stat.total_time, duration);
		if(ktime_to_ns(duration) > ktime_to_ns(lock->stat.max_time))
			lock->stat.max_time = duration;
		lock->stat.last_time = ktime_get();
	}
}
#endif

void android_unlock_suspend(android_suspend_lock_t *lock)
{
	int had_full_wake_locks;
	unsigned long irqflags;
	//printk("android_unlock_suspend name=%s\n", lock->name);
	spin_lock_irqsave(&g_list_lock, irqflags);
#ifdef CONFIG_ANDROID_POWER_STAT
	android_unlock_suspend_stat_locked(lock);
#endif
	lock->flags &= ~ANDROID_SUSPEND_LOCK_AUTO_EXPIRE;
	had_full_wake_locks = !list_empty(&g_active_full_wake_locks);
	list_del(&lock->link);
	list_add(&lock->link, &g_inactive_locks);
	wake_up(&g_wait_queue);
	if(had_full_wake_locks && list_empty(&g_active_full_wake_locks)) {
		printk("android_unlock_suspend: released at %lld\n", ktime_to_ns(ktime_get()));
		if(g_user_suspend_state == USER_NOTIFICATION) {
			printk("android sleep state %d->%d at %lld\n", g_user_suspend_state, USER_SLEEP, ktime_to_ns(ktime_get()));
			g_user_suspend_state = USER_SLEEP;
			queue_work(g_suspend_work_queue, &g_suspend_work);
		}
	}
	spin_unlock_irqrestore(&g_list_lock, irqflags);
}

static void android_power_wakeup_locked(int notification, ktime_t time)
{
	int new_state = (notification == 0) ? USER_AWAKE : USER_NOTIFICATION;
	if(new_state >= g_user_suspend_state) {
		return;
	}
	printk("android_power_wakeup %d->%d at %lld\n", g_user_suspend_state, new_state, ktime_to_ns(time));
	g_user_suspend_state = new_state;
	g_current_event_num++;
	wake_up(&g_wait_queue);
}

static void android_power_wakeup(void)
{
	unsigned long irqflags;

	ktime_t ktime_now;

	spin_lock_irqsave(&g_list_lock, irqflags);
	ktime_now = ktime_get();
	android_power_wakeup_locked(0, ktime_now);
	spin_unlock_irqrestore(&g_list_lock, irqflags);
}

static void android_power_request_sleep(void)
{
	unsigned long irqflags;
	int already_suspended;
	android_suspend_lock_t *lock, *next_lock;
	ktime_t ktime_now;

	ktime_now = ktime_get();
	printk("android_power_suspend: %lld\n", ktime_to_ns(ktime_now));

	spin_lock_irqsave(&g_list_lock, irqflags);
	already_suspended = g_user_suspend_state == USER_SLEEP;
	if(!already_suspended) {
		printk("android sleep state %d->%d at %lld\n", g_user_suspend_state, USER_SLEEP, ktime_to_ns(ktime_now));
		g_user_suspend_state = USER_SLEEP;
	}

	list_for_each_entry_safe(lock, next_lock, &g_active_full_wake_locks, link) {
#ifdef CONFIG_ANDROID_POWER_STAT
		android_unlock_suspend_stat_locked(lock);
#endif
		list_del(&lock->link);
		list_add(&lock->link, &g_inactive_locks);
		printk("android_power_suspend: aborted full wake lock %s\n", lock->name);
	}
	spin_unlock_irqrestore(&g_list_lock, irqflags);
	queue_work(g_suspend_work_queue, &g_suspend_work);
}

void android_register_early_suspend(android_early_suspend_t *handler)
{
	struct list_head *pos;

	mutex_lock(&g_early_suspend_lock);
	list_for_each(pos, &g_early_suspend_handlers) {
		android_early_suspend_t *e = list_entry(pos, android_early_suspend_t, link);
		if(e->level > handler->level)
			break;
	}
	list_add_tail(&handler->link, pos);
	mutex_unlock(&g_early_suspend_lock);
}

void android_unregister_early_suspend(android_early_suspend_t *handler)
{
	mutex_lock(&g_early_suspend_lock);
	list_del(&handler->link);
	mutex_unlock(&g_early_suspend_lock);
}

#ifdef CONFIG_FRAMEBUFFER_CONSOLE
static int orig_fgconsole;
static void console_early_suspend(android_early_suspend_t *h)
{
	acquire_console_sem();
	orig_fgconsole = fg_console;
	if (vc_allocate(ANDROID_SUSPEND_CONSOLE))
		goto err;
	if (set_console(ANDROID_SUSPEND_CONSOLE))
		goto err;
	release_console_sem();

	if (vt_waitactive(ANDROID_SUSPEND_CONSOLE))
		pr_warning("console_early_suspend: Can't switch VCs.\n");
	return;
err:
	pr_warning("console_early_suspend: Can't set console\n");
	release_console_sem();
}

static void console_late_resume(android_early_suspend_t *h)
{
	int ret;
	acquire_console_sem();
	ret = set_console(orig_fgconsole);
	release_console_sem();
	if (ret) {
		pr_warning("console_late_resume: Can't set console.\n");
		return;
	}

	if (vt_waitactive(orig_fgconsole))
		pr_warning("console_late_resume: Can't switch VCs.\n");
}

static android_early_suspend_t console_early_suspend_desc = {
	.level = ANDROID_EARLY_SUSPEND_LEVEL_CONSOLE_SWITCH,
	.suspend = console_early_suspend,
	.resume = console_late_resume,
};
#else
/* tell userspace to stop drawing, wait for it to stop */
static void stop_drawing_early_suspend(android_early_suspend_t *h)
{
	int ret;
	unsigned long irq_flags;

	spin_lock_irqsave(&fb_state_lock, irq_flags);
	fb_state = ANDROID_REQUEST_STOP_DRAWING;
	spin_unlock_irqrestore(&fb_state_lock, irq_flags);

	wake_up_all(&fb_state_wq);
	ret = wait_event_timeout(fb_state_wq,
				 fb_state == ANDROID_STOPPED_DRAWING,
				 HZ);
	if (unlikely(fb_state != ANDROID_STOPPED_DRAWING))
		printk(KERN_WARNING "android_power: timeout waiting for "
		       "userspace to stop drawing\n");
}

/* tell userspace to start drawing */
static void start_drawing_late_resume(android_early_suspend_t *h)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&fb_state_lock, irq_flags);
	fb_state = ANDROID_DRAWING_OK;
	spin_unlock_irqrestore(&fb_state_lock, irq_flags);
	printk("drawing ok\n");
	wake_up(&fb_state_wq);
}

static android_early_suspend_t stop_drawing_early_suspend_desc = {
	.level = ANDROID_EARLY_SUSPEND_LEVEL_CONSOLE_SWITCH,
	.suspend = stop_drawing_early_suspend,
	.resume = start_drawing_late_resume,
};
#endif

#if ANDROID_POWER_TEST_EARLY_SUSPEND

typedef struct
{
	android_early_suspend_t h;
	const char *string;
} early_suspend_test_t;

static void early_suspend_test(android_early_suspend_t *h)
{
	early_suspend_test_t *est = container_of(h, early_suspend_test_t, h);
	printk("early suspend %s (l %d)\n", est->string, h->level);
}

static void late_resume_test(android_early_suspend_t *h)
{
	early_suspend_test_t *est = container_of(h, early_suspend_test_t, h);
	printk("late resume %s (l %d)\n", est->string, h->level);
}

#define EARLY_SUSPEND_TEST_ENTRY(ilevel, istring) \
{ \
	.h = { \
		.level = ilevel, \
		.suspend = early_suspend_test, \
		.resume = late_resume_test \
	}, \
	.string = istring \
}
static early_suspend_test_t early_suspend_tests[] = {
	EARLY_SUSPEND_TEST_ENTRY(10, "1"),
	EARLY_SUSPEND_TEST_ENTRY(5, "2"),
	EARLY_SUSPEND_TEST_ENTRY(10, "3"),
	EARLY_SUSPEND_TEST_ENTRY(15, "4"),
	EARLY_SUSPEND_TEST_ENTRY(8, "5")
};

#endif

static int get_wait_timeout(int print_locks, int state, struct list_head *list_head)
{
	unsigned long irqflags;
	android_suspend_lock_t *lock, *next;
	int max_timeout = 0;

	spin_lock_irqsave(&g_list_lock, irqflags);
	list_for_each_entry_safe(lock, next, list_head, link) {
		if(lock->flags & ANDROID_SUSPEND_LOCK_AUTO_EXPIRE) {
			int timeout = lock->expires - (int)jiffies;
			if(timeout <= 0) {
				lock->flags &= ~ANDROID_SUSPEND_LOCK_AUTO_EXPIRE;
#ifdef CONFIG_ANDROID_POWER_STAT
				lock->stat.expire_count++;
				android_unlock_suspend_stat_locked(lock);
#endif
				list_del(&lock->link);
				list_add(&lock->link, &g_inactive_locks);
				if(!print_locks) // print wake locks that expire while waiting to enter sleep
					printk("expired wake lock %s\n", lock->name);
			}
			else {
				if(timeout > max_timeout)
					max_timeout = timeout;
				if(print_locks)
					printk("active wake lock %s, time left %d\n", lock->name, timeout);
			}
		}
		else {
			if(print_locks)
				printk("active wake lock %s\n", lock->name);
		}
	}
	if(g_user_suspend_state != state || list_empty(list_head))
		max_timeout = -1;
	spin_unlock_irqrestore(&g_list_lock, irqflags);
	return max_timeout;
}

#ifdef CONFIG_FRAMEBUFFER_CONSOLE
static int android_power_class_suspend(struct sys_device *sdev, pm_message_t state)
{
	int rv = 0;
	unsigned long irqflags;

	printk("android_power_suspend: enter\n");
	spin_lock_irqsave(&g_list_lock, irqflags);
	if(!list_empty(&g_active_partial_wake_locks)) {
		printk("android_power_suspend: abort for partial wakeup\n");
		rv = -EAGAIN;
	}
	if(g_user_suspend_state != USER_SLEEP) {
		printk("android_power_suspend: abort for full wakeup\n");
		rv = -EAGAIN;
	}
	spin_unlock_irqrestore(&g_list_lock, irqflags);
	return rv;
}

static int android_power_device_suspend(struct sys_device *sdev, pm_message_t state)
{
	int rv = 0;
	unsigned long irqflags;

	printk("android_power_device_suspend: enter\n");
	spin_lock_irqsave(&g_list_lock, irqflags);
	if(!list_empty(&g_active_partial_wake_locks)) {
		printk("android_power_device_suspend: abort for partial wakeup\n");
		rv = -EAGAIN;
	}
	if(g_user_suspend_state != USER_SLEEP) {
		printk("android_power_device_suspend: abort for full wakeup\n");
		rv = -EAGAIN;
	}
	spin_unlock_irqrestore(&g_list_lock, irqflags);
	return rv;
}
#endif

int android_power_is_driver_suspended(void)
{
	return (get_wait_timeout(0, USER_SLEEP, &g_active_partial_wake_locks) < 0) && (g_user_suspend_state == USER_SLEEP);
}

static void android_power_suspend(struct work_struct *work)
{
	int entry_event_num;
	int ret;
	int wait = 0;
	android_early_suspend_t *pos;
	int print_locks;
	unsigned long irqflags;

	while(g_user_suspend_state != USER_AWAKE) {
		while(g_user_suspend_state == USER_NOTIFICATION) {
			wait = get_wait_timeout(print_locks, USER_NOTIFICATION, &g_active_full_wake_locks);
			if(wait < 0)
				break;
			if(wait)
				wait_event_interruptible_timeout(g_wait_queue, get_wait_timeout(0, USER_NOTIFICATION, &g_active_full_wake_locks) != wait, wait);
		}
		spin_lock_irqsave(&g_list_lock, irqflags);
		if(g_user_suspend_state == USER_NOTIFICATION && list_empty(&g_active_full_wake_locks)) {
			printk("android sleep state %d->%d at %lld\n", g_user_suspend_state, USER_SLEEP, ktime_to_ns(ktime_get()));
			g_user_suspend_state = USER_SLEEP;
		}
		spin_unlock_irqrestore(&g_list_lock, irqflags);
		wait = 0;
		if(g_user_suspend_state == USER_AWAKE) {
			printk("android_power_suspend: suspend aborted\n");
			return;
		}
		
		mutex_lock(&g_early_suspend_lock);
		//printk("android_power_suspend: call early suspend handlers\n");
		list_for_each_entry(pos, &g_early_suspend_handlers, link) {
			if(pos->suspend != NULL)
				pos->suspend(pos);
		}
		//printk("android_power_suspend: call early suspend handlers\n");

		//printk("android_power_suspend: enter\n");

		sys_sync();

		while(g_user_suspend_state == USER_SLEEP) {
			//printk("android_power_suspend: enter wait (%d)\n", wait);
			if(wait) {
				wait_event_interruptible_timeout(g_wait_queue, g_user_suspend_state != USER_SLEEP, wait);
				wait = 0;
			}
			print_locks = 1;
			while(1) {
				wait = get_wait_timeout(print_locks, USER_SLEEP, &g_active_partial_wake_locks);
				print_locks = 0;
				if(wait < 0)
					break;
				if(wait)
					wait_event_interruptible_timeout(g_wait_queue, get_wait_timeout(0, USER_SLEEP, &g_active_partial_wake_locks) != wait, wait);
				else
					wait_event_interruptible(g_wait_queue, get_wait_timeout(0, USER_SLEEP, &g_active_partial_wake_locks) != wait);
			}
			wait = 0;
			//printk("android_power_suspend: exit wait\n");
			entry_event_num = g_current_event_num;
			if(g_user_suspend_state != USER_SLEEP)
				break;
			sys_sync();
			printk("android_power_suspend: enter suspend\n");
			ret = pm_suspend(PM_SUSPEND_MEM);
			printk("android_power_suspend: exit suspend, ret = %d\n", ret);
			if(g_current_event_num == entry_event_num) {
				printk("android_power_suspend: pm_suspend returned with no event\n");
				wait = HZ / 2;
#ifdef CONFIG_ANDROID_POWER_STAT
				if(g_no_wake_locks.stat.count == 0) {
					g_no_wake_locks.name = "unknown_wakeups";
					android_init_suspend_lock(&g_no_wake_locks);
				}
				g_no_wake_locks.stat.count++;
				g_no_wake_locks.stat.total_time = ktime_add(
					g_no_wake_locks.stat.total_time,
					ktime_set(0, 500 * NSEC_PER_MSEC));
				g_no_wake_locks.stat.max_time =
					ktime_set(0, 500 * NSEC_PER_MSEC);
#endif
			}
		}
		printk("android_power_suspend: done\n");
		//printk("android_power_suspend: call late resume handlers\n");
		list_for_each_entry_reverse(pos, &g_early_suspend_handlers, link) {
			if(pos->resume != NULL)
				pos->resume(pos);
		}
		//printk("android_power_suspend: call late resume handlers\n");
		mutex_unlock(&g_early_suspend_lock);
	}
}

#if 0
struct sysdev_class android_power_sysclass = {
	set_kset_name("android_power"),
	.suspend = android_power_class_suspend
};
static struct sysdev_class *g_android_power_sysclass = NULL;

static struct {
	struct sys_device sysdev;
//	omap_csmi_gsm_image_info_t *pdata;
} android_power_device = {
	.sysdev = {
		.id		= 0,
		.cls		= &android_power_sysclass,
//		.suspend = android_power_device_suspend
	},
//	.pdata = &g_gsm_image_info
};

struct sysdev_class *android_power_get_sysclass(void)
{
	return g_android_power_sysclass;
}
#endif

static ssize_t state_show(struct kobject *kobj, struct subsys_attribute *attr, char * buf)
{
	char * s = buf;
	unsigned long irqflags;

	spin_lock_irqsave(&g_list_lock, irqflags);
	s += sprintf(s, "%d-%d-%d\n", g_user_suspend_state, list_empty(&g_active_full_wake_locks), list_empty(&g_active_partial_wake_locks));
	spin_unlock_irqrestore(&g_list_lock, irqflags);
	return (s - buf);
}

static ssize_t state_store(struct kobject *kobj, struct subsys_attribute *attr, const char * buf, size_t n)
{
	if(n >= strlen("standby") &&
	   strncmp(buf, "standby", strlen("standby")) == 0) {
		android_power_request_sleep();
		wait_event_interruptible(g_wait_queue, g_user_suspend_state == USER_AWAKE);
		return n;
	}
	if(n >= strlen("wake") &&
	   strncmp(buf, "wake", strlen("wake")) == 0) {
		android_power_wakeup();
		return n;
	}
	printk("android_power state_store: invalid argument\n");
	return -EINVAL;
}

static ssize_t request_state_show(struct kobject *kobj, struct subsys_attribute *attr, char * buf)
{
	char * s = buf;
	unsigned long irqflags;

	spin_lock_irqsave(&g_list_lock, irqflags);
	if(g_user_suspend_state == USER_AWAKE)
		s += sprintf(s, "wake\n");
	else if(g_user_suspend_state == USER_NOTIFICATION)
		s += sprintf(s, "standby (w/full wake lock)\n");
	else
		s += sprintf(s, "standby\n");
	spin_unlock_irqrestore(&g_list_lock, irqflags);
	return (s - buf);
}

static ssize_t request_state_store(struct kobject *kobj, struct subsys_attribute *attr, const char * buf, size_t n)
{
	if(n >= strlen("standby") &&
	   strncmp(buf, "standby", strlen("standby")) == 0) {
		android_power_request_sleep();
		return n;
	}
	if(n >= strlen("wake") &&
	   strncmp(buf, "wake", strlen("wake")) == 0) {
		android_power_wakeup();
		return n;
	}
	printk("android_power state_store: invalid argument\n");
	return -EINVAL;
}


static int lookup_wake_lock_name(const char *buf, size_t n, int allocate, int *timeout)
{
	int i;
	int free_index = -1;
	int inactive_index = -1;
	int expires_index = -1;
	int expires_time = INT_MAX;
	char *tmp_buf[64];
	char name[32];
	u64 nanoseconds;
	int num_arg;

	if(n <= 0)
		return -EINVAL;
	if(n >= sizeof(tmp_buf))
		return -EOVERFLOW;
	if(n == sizeof(tmp_buf) - 1 && buf[n - 1] != '\0')
		return -EOVERFLOW;

	memcpy(tmp_buf, buf, n);
	if(tmp_buf[n - 1] != '\0')
		tmp_buf[n] = '\0';

	num_arg = sscanf(buf, "%31s %llu", name, &nanoseconds);
	if(num_arg < 1)
		return -EINVAL;

	if(strlen(name) >= sizeof(g_user_wake_locks[i].name_buffer))
		return -EOVERFLOW;

	if(timeout != NULL) {
		if(num_arg > 1) {
			do_div(nanoseconds, (NSEC_PER_SEC / HZ));
			if(nanoseconds <= 0)
				nanoseconds = 1;
			*timeout = nanoseconds;
		}
		else
			*timeout = 0;
	}

	for(i = 0; i < g_max_user_lockouts; i++) {
		if(strcmp(g_user_wake_locks[i].name_buffer, name) == 0)
			return i;
		if(g_user_wake_locks[i].name_buffer[0] == '\0')
			free_index = i;
		else if(g_user_wake_locks[i].state == USER_WAKE_LOCK_INACTIVE)
			inactive_index = i;
		else if(g_user_wake_locks[i].suspend_lock.expires < expires_time)
			expires_index = i;
	}
	if(allocate) {
		if(free_index >= 0)
			i = free_index;
		else if(inactive_index >= 0)
			i = inactive_index;
		else if(expires_index >= 0) {
			i = expires_index;
			printk("lookup_wake_lock_name: overwriting expired lock, %s\n", g_user_wake_locks[i].name_buffer);
		}
		else {
			i = 0;
			printk("lookup_wake_lock_name: overwriting active lock, %s\n", g_user_wake_locks[i].name_buffer);
		}
		strcpy(g_user_wake_locks[i].name_buffer, name);
		return i;
	}
#if ANDROID_POWER_PRINT_USER_WAKE_LOCKS
	printk("lookup_wake_lock_name: %s not found\n", name);
#endif
	return -EINVAL;
}

static ssize_t acquire_full_wake_lock_show(struct kobject *kobj, struct subsys_attribute *attr, char * buf)
{
	int i;
	char * s = buf;
	unsigned long irqflags;

	spin_lock_irqsave(&g_list_lock, irqflags);
	for(i = 0; i < g_max_user_lockouts; i++) {
		if(g_user_wake_locks[i].name_buffer[0] != '\0' && g_user_wake_locks[i].state == USER_WAKE_LOCK_FULL)
			s += sprintf(s, "%s ", g_user_wake_locks[i].name_buffer);
	}
	s += sprintf(s, "\n");
	
	spin_unlock_irqrestore(&g_list_lock, irqflags);
	return (s - buf);
}

static ssize_t acquire_full_wake_lock_store(struct kobject *kobj, struct subsys_attribute *attr, const char * buf, size_t n)
{
	int i;
	unsigned long irqflags;
	int timeout;

	spin_lock_irqsave(&g_list_lock, irqflags);
	i = lookup_wake_lock_name(buf, n, 1, &timeout);
	if(i >= 0)
		g_user_wake_locks[i].state = USER_WAKE_LOCK_FULL;
	spin_unlock_irqrestore(&g_list_lock, irqflags);
	if(i < 0)
		return i;

#if ANDROID_POWER_PRINT_USER_WAKE_LOCKS
	printk("acquire_full_wake_lock_store: %s, size %d\n", g_user_wake_locks[i].name_buffer, n);
#endif

	//android_lock_partial_suspend_auto_expire(&g_user_wake_locks[i].suspend_lock, ktime_to_timespec(g_auto_off_timeout).tv_sec * HZ);
	if(timeout == 0)
		timeout = INT_MAX;
	android_lock_partial_suspend_auto_expire(&g_user_wake_locks[i].suspend_lock, timeout);

	return n;
}

static ssize_t acquire_partial_wake_lock_show(struct kobject *kobj, struct subsys_attribute *attr, char * buf)
{
	int i;
	char * s = buf;
	unsigned long irqflags;

	spin_lock_irqsave(&g_list_lock, irqflags);
	for(i = 0; i < g_max_user_lockouts; i++) {
		if(g_user_wake_locks[i].name_buffer[0] != '\0' && g_user_wake_locks[i].state == USER_WAKE_LOCK_PARTIAL)
			s += sprintf(s, "%s ", g_user_wake_locks[i].name_buffer);
	}
	s += sprintf(s, "\n");
	
	spin_unlock_irqrestore(&g_list_lock, irqflags);
	return (s - buf);
}

static ssize_t acquire_partial_wake_lock_store(struct kobject *kobj, struct subsys_attribute *attr, const char * buf, size_t n)
{
	int i;
	unsigned long irqflags;
	int timeout;

	spin_lock_irqsave(&g_list_lock, irqflags);
	i = lookup_wake_lock_name(buf, n, 1, &timeout);
	if(i >= 0)
		g_user_wake_locks[i].state = USER_WAKE_LOCK_PARTIAL;
	spin_unlock_irqrestore(&g_list_lock, irqflags);
	if(i < 0)
		return 0;

#if ANDROID_POWER_PRINT_USER_WAKE_LOCKS
	printk("acquire_partial_wake_lock_store: %s, size %d\n", g_user_wake_locks[i].name_buffer, n);
#endif

	if(timeout)
		android_lock_suspend_auto_expire(&g_user_wake_locks[i].suspend_lock, timeout);
	else
		android_lock_suspend(&g_user_wake_locks[i].suspend_lock);

	return n;
}


static ssize_t release_wake_lock_show(struct kobject *kobj, struct subsys_attribute *attr, char * buf)
{
	int i;
	char * s = buf;
	unsigned long irqflags;

	spin_lock_irqsave(&g_list_lock, irqflags);
	for(i = 0; i < g_max_user_lockouts; i++) {
		if(g_user_wake_locks[i].name_buffer[0] != '\0' && g_user_wake_locks[i].state == USER_WAKE_LOCK_INACTIVE)
			s += sprintf(s, "%s ", g_user_wake_locks[i].name_buffer);
	}
	s += sprintf(s, "\n");
	
	spin_unlock_irqrestore(&g_list_lock, irqflags);
	return (s - buf);
}

static ssize_t release_wake_lock_store(struct kobject *kobj, struct subsys_attribute *attr, const char * buf, size_t n)
{
	int i;
	unsigned long irqflags;

	spin_lock_irqsave(&g_list_lock, irqflags);
	i = lookup_wake_lock_name(buf, n, 1, NULL);
	if(i >= 0) {
		g_user_wake_locks[i].state = USER_WAKE_LOCK_INACTIVE;
	}
	spin_unlock_irqrestore(&g_list_lock, irqflags);

	if(i < 0)
		return i;

#if ANDROID_POWER_PRINT_USER_WAKE_LOCKS
	printk("release_wake_lock_store: %s, size %d\n", g_user_wake_locks[i].name_buffer, n);
#endif

	android_unlock_suspend(&g_user_wake_locks[i].suspend_lock);
	return n;
}


#ifndef CONFIG_FRAMEBUFFER_CONSOLE
static ssize_t wait_for_fb_sleep_show(struct kobject *kobj,
				      struct subsys_attribute *attr, char *buf)
{
	char * s = buf;
	int ret;

	ret = wait_event_freezable(fb_state_wq, fb_state != ANDROID_DRAWING_OK);
	if (!ret) {
		s += sprintf(buf, "sleeping");
		return (s - buf);
	} else
		return -1;
}

static ssize_t wait_for_fb_wake_show(struct kobject *kobj,
				     struct subsys_attribute *attr, char *buf)
{
	char * s = buf;
	int ret;
	unsigned long irq_flags;
	
	spin_lock_irqsave(&fb_state_lock, irq_flags);
	if (fb_state == ANDROID_REQUEST_STOP_DRAWING) {
		fb_state = ANDROID_STOPPED_DRAWING;
		wake_up(&fb_state_wq);
	}
	spin_unlock_irqrestore(&fb_state_lock, irq_flags);

	ret = wait_event_freezable(fb_state_wq, fb_state == ANDROID_DRAWING_OK);
	if (!ret) {
		s += sprintf(buf, "awake");
		return (s - buf);
	} else
		return -1;
}
#endif

#define android_power_attr(_name) \
static struct subsys_attribute _name##_attr = {	\
	.attr	= {				\
		.name = __stringify(_name),	\
		.mode = 0664,			\
	},					\
	.show	= _name##_show,			\
	.store	= _name##_store,		\
}

#define android_power_ro_attr(_name) \
static struct subsys_attribute _name##_attr = {	\
	.attr	= {				\
		.name = __stringify(_name),	\
		.mode = 0444,			\
	},					\
	.show	= _name##_show,			\
	.store	= NULL,		\
}

android_power_attr(state);
android_power_attr(request_state);
android_power_attr(acquire_full_wake_lock);
android_power_attr(acquire_partial_wake_lock);
android_power_attr(release_wake_lock);
#ifndef CONFIG_FRAMEBUFFER_CONSOLE
android_power_ro_attr(wait_for_fb_sleep);
android_power_ro_attr(wait_for_fb_wake);
#endif

static struct attribute * g[] = {
	&state_attr.attr,
	&request_state_attr.attr,
	&acquire_full_wake_lock_attr.attr,
	&acquire_partial_wake_lock_attr.attr,
	&release_wake_lock_attr.attr,
#ifndef CONFIG_FRAMEBUFFER_CONSOLE
	&wait_for_fb_sleep_attr.attr,
	&wait_for_fb_wake_attr.attr,
#endif
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};

#if 0
// test code when there is no platform suspend

static android_suspend_lock_t test_pm_ops_suspend_lock = {
	.name = "test_pm_ops"
};

int test_pm_op_enter(suspend_state_t state)
{
	printk("test_pm_op_enter reached\n");
	android_lock_suspend(&test_pm_ops_suspend_lock);
	printk("test_pm_op_enter returned\n");
	return 0;
}

void test_pm_ops_late_resume_handler(android_early_suspend_t *h)
{
	printk("test_pm_ops_late_resume_handler reached\n");
	android_unlock_suspend(&test_pm_ops_suspend_lock);
	printk("test_pm_ops_late_resume_handler returned\n");
}

static struct pm_ops test_pm_ops = {
	.enter = test_pm_op_enter
};

static android_early_suspend_t test_pm_ops_early_suspend_handler = {
	.resume = test_pm_ops_late_resume_handler
};
#endif

static int __init android_power_init(void)
{
	int ret;
	int i;

	printk("android_power_init\n");

#if 0
	if(pm_ops == NULL) {
		printk("android_power_init no pm_ops, installing test code\n");
		pm_set_ops(&test_pm_ops);
		android_init_suspend_lock(&test_pm_ops_suspend_lock);
		android_register_early_suspend(&test_pm_ops_early_suspend_handler);
	}
#endif

#ifdef CONFIG_ANDROID_POWER_STAT
	g_deleted_wake_locks.stat.count = 0;
#endif
	init_waitqueue_head(&g_wait_queue);
#ifndef CONFIG_FRAMEBUFFER_CONSOLE
	init_waitqueue_head(&fb_state_wq);
	fb_state = ANDROID_DRAWING_OK;
#endif

	g_user_wake_locks = kzalloc(sizeof(*g_user_wake_locks) * g_max_user_lockouts, GFP_KERNEL);
	if(g_user_wake_locks == NULL) {
		ret = -ENOMEM;
		goto err1;
	}
	for(i = 0; i < g_max_user_lockouts; i++) {
		g_user_wake_locks[i].suspend_lock.name = g_user_wake_locks[i].name_buffer;
		android_init_suspend_lock(&g_user_wake_locks[i].suspend_lock);
	}	

	g_suspend_work_queue = create_workqueue("suspend");
	if(g_suspend_work_queue == NULL) {
		ret = -ENOMEM;
		goto err2;
	}
// 2008.12.10 invain: I modified kobejct api to support sysfs.
//printk(KERN_WARNING "power.c: android_power_init -kobject_create_and_add before \n");
// 		android_power_subsys = kobject_create_and_add("android_power", NULL);
//printk(KERN_WARNING "power.c: android_power_init -kobject_create_and_add after \n");
//	if (android_power_subsys == NULL) {
//		printk("android_power_init: subsystem_register failed\n");
//		ret = -ENOMEM;
//		goto err3;
//	}

        //printk(KERN_WARNING "power.c: android_power_init -subsystem_register before \n");
        ret = subsystem_register(android_power_subsys);
        if(ret) {
                printk("android_power_init: subsystem_register failed\n");
                goto err3;
        }
        //printk(KERN_WARNING "power.c: android_power_init -subsystem_register after \n");

	ret = sysfs_create_group(android_power_subsys, &attr_group);
	if(ret) {
		printk("android_power_init: sysfs_create_group failed\n");
		goto err4;
	}
#ifdef CONFIG_ANDROID_POWER_STAT
	create_proc_read_entry("wakelocks", S_IRUGO, NULL, wakelocks_read_proc, NULL);
#endif

#if ANDROID_POWER_TEST_EARLY_SUSPEND
	{
		int i;
		for(i = 0; i < sizeof(early_suspend_tests) / sizeof(early_suspend_tests[0]); i++)
			android_register_early_suspend(&early_suspend_tests[i].h);
	}
#endif
#ifdef CONFIG_FRAMEBUFFER_CONSOLE
	android_register_early_suspend(&console_early_suspend_desc);
#else
	android_register_early_suspend(&stop_drawing_early_suspend_desc);
#endif

#if 0
	ret = sysdev_class_register(&android_power_sysclass);
	if(ret) {
		printk("android_power_init: sysdev_class_register failed\n");
		goto err1;
	}
	ret = sysdev_register(&android_power_device.sysdev);
	if(ret < 0)
		goto err2;

	g_android_power_sysclass = &android_power_sysclass;
#endif
	printk("android_power_init done\n");

	return 0;

//err2:
//	sysdev_class_unregister(&android_power_sysclass);
err4:
	kobject_del(android_power_subsys);
err3:
	destroy_workqueue(g_suspend_work_queue);
err2:
	for(i = 0; i < g_max_user_lockouts; i++) {
		android_uninit_suspend_lock(&g_user_wake_locks[i].suspend_lock);
	}
	kfree(g_user_wake_locks);
err1:
	return ret;
}

static void  __exit android_power_exit(void)
{
	int i;
//	g_android_power_sysclass = NULL;
//	sysdev_unregister(&android_power_device.sysdev);
//	sysdev_class_unregister(&android_power_sysclass);
#ifdef CONFIG_FRAMEBUFFER_CONSOLE
	android_unregister_early_suspend(&console_early_suspend_desc);
#else
	android_unregister_early_suspend(&stop_drawing_early_suspend_desc);
#endif
#ifdef CONFIG_ANDROID_POWER_STAT
	remove_proc_entry("wakelocks", NULL);
#endif
	sysfs_remove_group(android_power_subsys, &attr_group);
	kobject_del(android_power_subsys);
	destroy_workqueue(g_suspend_work_queue);
	for(i = 0; i < g_max_user_lockouts; i++) {
		android_uninit_suspend_lock(&g_user_wake_locks[i].suspend_lock);
	}
	kfree(g_user_wake_locks);
}

core_initcall(android_power_init);
module_exit(android_power_exit);

//EXPORT_SYMBOL(android_power_get_sysclass);
EXPORT_SYMBOL(android_init_suspend_lock);
EXPORT_SYMBOL(android_uninit_suspend_lock);
EXPORT_SYMBOL(android_lock_suspend);
EXPORT_SYMBOL(android_lock_suspend_auto_expire);
EXPORT_SYMBOL(android_unlock_suspend);
EXPORT_SYMBOL(android_power_wakeup);
EXPORT_SYMBOLGPL(android_power_subsys);
EXPORT_SYMBOL(android_register_early_suspend);
EXPORT_SYMBOL(android_unregister_early_suspend);


