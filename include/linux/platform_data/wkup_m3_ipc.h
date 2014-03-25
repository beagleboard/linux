/*
 * omap wkup_m3_ipc: platform data
 *
 * Copyright (C) 2015 Texas Instruments, Inc.
 *
 * Dave Gerlach <d-gerlach@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _LINUX_PLATFORM_DATA_WKUP_M3_IPC_H
#define _LINUX_PLATFORM_DATA_WKUP_M3_IPC_H

struct wkup_m3_pm_ipc_ops {
	void (*set_mem_type)(int mem_type);
	void (*set_resume_address)(void *addr);
	int (*prepare_low_power)(int state);
	int (*finish_low_power)(void);
	int (*request_pm_status)(void);
};

struct wkup_m3_ipc_data {
	void (*set_pm_ipc_ops)(struct wkup_m3_pm_ipc_ops *ops);
};

#endif /* _LINUX_PLATFORM_DATA_WKUP_M3_IPC_H */
