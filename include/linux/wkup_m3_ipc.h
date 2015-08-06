/*
 * TI Wakeup M3 for AMx3 SoCs Power Management Routines
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
 * Dave Gerlach <d-gerlach@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _LINUX_WKUP_M3_IPC_H
#define _LINUX_WKUP_M3_IPC_H

#include <linux/suspend.h>

#define WKUP_M3_IDLE (PM_SUSPEND_MAX + 1)

struct wkup_m3_wakeup_src {
	int irq_nr;
	char src[10];
};

void wkup_m3_set_mem_type(int mem_type);
void wkup_m3_set_resume_address(void *addr);
int wkup_m3_prepare_low_power(int state);
int wkup_m3_finish_low_power(void);
int wkup_m3_request_pm_status(void);
int wkup_m3_request_pm_status(void);
const char *wkup_m3_request_wake_src(void);
void wkup_m3_set_rtc_only_mode(void);

#endif /* _LINUX_WKUP_M3_IPC_H */
