/*
 * Copyright (C) 2016 Texas Instruments
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __OMAP_DRM_DSS_H
#define __OMAP_DRM_DSS_H

#include <video/omapdss.h>

void omapdss_set_is_initialized(bool set);

struct device_node *dss_of_port_get_parent_device(struct device_node *port);
u32 dss_of_port_get_port_number(struct device_node *port);

enum dss_writeback_channel {
	DSS_WB_LCD1_MGR =	0,
	DSS_WB_LCD2_MGR =	1,
	DSS_WB_TV_MGR =		2,
	DSS_WB_OVL0 =		3,
	DSS_WB_OVL1 =		4,
	DSS_WB_OVL2 =		5,
	DSS_WB_OVL3 =		6,
	DSS_WB_LCD3_MGR =	7,
};

struct dss_mgr_ops {
	int (*connect)(enum omap_channel channel,
		struct omap_dss_device *dst);
	void (*disconnect)(enum omap_channel channel,
		struct omap_dss_device *dst);

	void (*start_update)(enum omap_channel channel);
	int (*enable)(enum omap_channel channel);
	void (*disable)(enum omap_channel channel);
	void (*set_timings)(enum omap_channel channel,
			const struct omap_video_timings *timings);
	void (*set_lcd_config)(enum omap_channel channel,
			const struct dss_lcd_mgr_config *config);
	int (*register_framedone_handler)(enum omap_channel channel,
			void (*handler)(void *), void *data);
	void (*unregister_framedone_handler)(enum omap_channel channel,
			void (*handler)(void *), void *data);
};

int dss_install_mgr_ops(const struct dss_mgr_ops *mgr_ops);
void dss_uninstall_mgr_ops(void);

int dss_mgr_connect(enum omap_channel channel,
		struct omap_dss_device *dst);
void dss_mgr_disconnect(enum omap_channel channel,
		struct omap_dss_device *dst);
void dss_mgr_set_timings(enum omap_channel channel,
		const struct omap_video_timings *timings);
void dss_mgr_set_lcd_config(enum omap_channel channel,
		const struct dss_lcd_mgr_config *config);
int dss_mgr_enable(enum omap_channel channel);
void dss_mgr_disable(enum omap_channel channel);
void dss_mgr_start_update(enum omap_channel channel);
int dss_mgr_register_framedone_handler(enum omap_channel channel,
		void (*handler)(void *), void *data);
void dss_mgr_unregister_framedone_handler(enum omap_channel channel,
		void (*handler)(void *), void *data);

/* dispc ops */

struct dispc_ops {
	u32 (*read_irqstatus)(void);
	void (*clear_irqstatus)(u32 mask);
	u32 (*read_irqenable)(void);
	void (*write_irqenable)(u32 mask);

	int (*request_irq)(irq_handler_t handler, void *dev_id);
	void (*free_irq)(void *dev_id);

	int (*runtime_get)(void);
	void (*runtime_put)(void);

	int (*get_num_ovls)(void);
	int (*get_num_mgrs)(void);

	void (*mgr_enable)(enum omap_channel channel, bool enable);
	bool (*mgr_is_enabled)(enum omap_channel channel);
	u32 (*mgr_get_vsync_irq)(enum omap_channel channel);
	u32 (*mgr_get_framedone_irq)(enum omap_channel channel);
	u32 (*mgr_get_sync_lost_irq)(enum omap_channel channel);
	bool (*mgr_go_busy)(enum omap_channel channel);
	void (*mgr_go)(enum omap_channel channel);
	void (*mgr_set_lcd_config)(enum omap_channel channel,
			const struct dss_lcd_mgr_config *config);
	void (*mgr_set_timings)(enum omap_channel channel,
			const struct omap_video_timings *timings);
	void (*mgr_setup)(enum omap_channel channel,
			const struct omap_overlay_manager_info *info);
	enum omap_dss_output_id (*mgr_get_supported_outputs)(enum omap_channel channel);

	int (*ovl_enable)(enum omap_plane plane, bool enable);
	bool (*ovl_enabled)(enum omap_plane plane);
	void (*ovl_set_channel_out)(enum omap_plane plane,
			enum omap_channel channel);
	int (*ovl_setup)(enum omap_plane plane, const struct omap_overlay_info *oi,
			bool replication, const struct omap_video_timings *mgr_timings,
			bool mem_to_mem);

	enum omap_color_mode (*ovl_get_color_modes)(enum omap_plane plane);

	u32 (*wb_get_framedone_irq)(void);
	void (*wb_set_channel_in)(enum dss_writeback_channel channel);
	int (*wb_setup)(const struct omap_dss_writeback_info *wi,
		bool mem_to_mem, const struct omap_video_timings *mgr_timings);
	bool (*has_writeback)(void);
};

void dispc_set_ops(const struct dispc_ops *o);
const struct dispc_ops *dispc_get_ops(void);

#endif /* __OMAP_DRM_DSS_H */
