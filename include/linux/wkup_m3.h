/*
 * TI Wakeup M3 for AMx3 SoCs Power Management Routines
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef _LINUX_WKUP_M3_H
#define _LINUX_WKUP_M3_H

/**
 * struct wkup_m3_ops - Callbacks for allowing pm code to interact with wkup_m3.
 *
 * @txev_handler: Callback to allow pm code to react to response from wkup_m3
 *		  after pinging it using wkup_m3_ping.
 *
 * @rproc_ready: Callback invoked when the firmware has been loaded to the
 *		     m3 to allow the pm code to enable suspend/resume ops.
 */

struct wkup_m3_ops {
	void (*txev_handler)(void);
	void (*rproc_ready)(struct device *dev);
};

struct wkup_m3_wakeup_src {
	int irq_nr;
	char src[10];
};

struct wkup_m3_ipc_regs {
	u32 reg0;
	u32 reg1;
	u32 reg2;
	u32 reg3;
	u32 reg4;
	u32 reg5;
	u32 reg6;
	u32 reg7;
};

#ifdef CONFIG_WKUP_M3_RPROC
int wkup_m3_prepare(void);
unsigned long wkup_m3_copy_aux_data(const void *data, int sz);
void wkup_m3_set_ops(struct wkup_m3_ops *ops);
int wkup_m3_ping(void);
void wkup_m3_wake_src(struct wkup_m3_wakeup_src *wakeup_src);
int wkup_m3_pm_status(void);
int wkup_m3_is_valid(void);
int wkup_m3_fw_version_read(void);
void wkup_m3_set_cmd(struct wkup_m3_ipc_regs *ipc_regs);

#else

static inline int wkup_m3_prepare(void) { return -EINVAL; }
static inline
unsigned long wkup_m3_copy_aux_data(void *data, int sz) { return -1; }
static inline void wkup_m3_set_ops(struct wkup_m3_ops *ops) { }
static inline int wkup_m3_ping(void) { return -EINVAL }
static inline void wkup_m3_wake_src(struct wkup_m3_wakeup_src *wakeup_src) { }
static inline int wkup_m3_pm_status(void) { return -1; }
static inline int wkup_m3_fw_version_read(void) { return -1; }
static inline void wkup_m3_set_cmd(struct wkup_m3_ipc_regs *ipc_regs) { }
#endif /* CONFIG_WKUP_M3_RPROC */
#endif /* _LINUX_WKUP_M3_H */
