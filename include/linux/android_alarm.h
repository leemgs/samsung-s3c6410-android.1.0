/* include/linux/android_alarm.h
 *
 * Copyright (C) 2006-2007 Google, Inc.
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

#ifndef _LINUX_ANDROID_ALARM_H
#define _LINUX_ANDROID_ALARM_H

#include <asm/ioctl.h>
#include <linux/time.h>

typedef enum {
	/* return code bit numbers or set alarm arg */
	ANDROID_ALARM_RTC_WAKEUP,
	ANDROID_ALARM_RTC,
	ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
	ANDROID_ALARM_ELAPSED_REALTIME,
	ANDROID_ALARM_SYSTEMTIME,

	ANDROID_ALARM_TYPE_COUNT,
	
	/* return code bit numbers */
	/* ANDROID_ALARM_TIME_CHANGE = 16 */
} android_alarm_type_t;

typedef enum {
	ANDROID_ALARM_RTC_WAKEUP_MASK = 1U << ANDROID_ALARM_RTC_WAKEUP,
	ANDROID_ALARM_RTC_MASK = 1U << ANDROID_ALARM_RTC,
	ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP_MASK = 1U << ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
	ANDROID_ALARM_ELAPSED_REALTIME_MASK = 1U << ANDROID_ALARM_ELAPSED_REALTIME,
	ANDROID_ALARM_SYSTEMTIME_MASK = 1U << ANDROID_ALARM_SYSTEMTIME,
	ANDROID_ALARM_TIME_CHANGE_MASK = 1U << 16
} android_alarm_return_flags_t;

/* Disable alarm */
#define ANDROID_ALARM_CLEAR(type)           _IO('a', 0 | ((type) << 4))

/* Ack last alarm and wait for next */
#define ANDROID_ALARM_WAIT                  _IO('a', 1)

/* Set alarm */
#define ANDROID_ALARM_SET(type)             _IOW('a', 2 | ((type) << 4), struct timespec)
#define ANDROID_ALARM_SET_AND_WAIT(type)    _IOW('a', 3 | ((type) << 4), struct timespec)
#define ANDROID_ALARM_GET_TIME(type)        _IOW('a', 4 | ((type) << 4), struct timespec)
#define ANDROID_ALARM_SET_RTC               _IOW('a', 5, struct timespec)
#define ANDROID_ALARM_BASE_CMD(cmd) (cmd & ~(_IOC(0, 0, 0xf0, 0)))
#define ANDROID_ALARM_IOCTL_TO_TYPE(cmd) (_IOC_NR(cmd) >> 4)

#endif
