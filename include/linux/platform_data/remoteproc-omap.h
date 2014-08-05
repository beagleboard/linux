/*
 * Remote Processor - omap-specific bits
 *
 * Copyright (C) 2011-2016 Texas Instruments Incorporated - http://www.ti.com/
 * Copyright (C) 2011 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _PLAT_REMOTEPROC_H
#define _PLAT_REMOTEPROC_H

struct platform_device;
struct device_node;
struct omap_dm_timer;

/**
 * struct omap_rproc_timer_ops - platform data ops for dmtimer handlers
 * @request_timer: omap-specific handler for requesting a rproc timer
 * @release_timer: omap-specific handler for freeing a rproc timer
 * @start_timer: omap-specific handler for enabling a rproc timer
 * @stop_timer: omap-specific handler for disabling a rproc timer
 * @get_timer_irq: handler to retrieve the irq id of a OMAP DMTimer
 * @ack_timer_irq: handler to acknowledge the interrupt of a OMAP DMTimer
 */
struct omap_rproc_timer_ops {
	struct omap_dm_timer * (*request_timer)(struct device_node *np);
	int (*release_timer)(struct omap_dm_timer *timer);
	int (*start_timer)(struct omap_dm_timer *timer);
	int (*stop_timer)(struct omap_dm_timer *timer);

	/* watchdog timer specific ops */
	int (*get_timer_irq)(struct omap_dm_timer *timer);
	void (*ack_timer_irq)(struct omap_dm_timer *timer);
};

/*
 * struct omap_rproc_pdata - omap remoteproc's platform data
 * @device_enable: omap-specific handler for enabling a device
 * @device_shutdown: omap-specific handler for shutting down a device
 * @timer_ops: platform data ops for OMAP dmtimer handlers
 */
struct omap_rproc_pdata {
	int (*device_enable)(struct platform_device *pdev);
	int (*device_shutdown)(struct platform_device *pdev);

	struct omap_rproc_timer_ops *timer_ops;
};

#endif /* _PLAT_REMOTEPROC_H */
