/*
 * Copyright (C) 2016 Texas Instruments Ltd
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __TI_DSS6_H
#define __TI_DSS6_H

#include "omapdss.h"

/* DSS6 */
void dss6_ungate_dpi_clk(struct device *dev, int port);
void dss6_gate_dpi_clk(struct device *dev, int port);

/* DISPC6 */
int __init dispc6_init_platform_driver(void);
void dispc6_uninit_platform_driver(void);

int dispc6_runtime_get(void);
void dispc6_runtime_put(void);

bool dispc6_mgr_timings_ok(enum omap_channel channel,
		const struct omap_video_timings *timings);

int dispc6_vp_set_clk_rate(enum omap_channel channel, unsigned long rate);
int dispc6_vp_enable_clk(enum omap_channel channel);
void dispc6_vp_disable_clk(enum omap_channel channel);

/* DPI6 */
int dpi6_init_port(struct platform_device *pdev, struct device_node *port);
void dpi6_uninit_port(struct device_node *port);

#endif /* __TI_DSS6_H */
