/*
 * hdmi-audio.c -- OMAP4+ DSS HDMI audio support library
 *
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com
 *
 * Author: Jyri Sarha <jsarha@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */

#include <video/omapdss.h>

#ifndef __OMAP_HDMI_AUDIO_H__
#define __OMAP_HDMI_AUDIO_H__

/* HDMI audio initalization data */
struct omap_hdmi_audio {
	struct device *dev;
	enum { OMAP4_HDMI, OMAP5_HDMI } hw_version;
	phys_addr_t audio_dma_addr;

	int (*audio_startup)(struct device *dev,
			     void (*abort_cb)(struct device *dev));
	int (*audio_enable)(struct device *dev, bool enable);
	int (*audio_start)(struct device *dev, bool enable);
	int (*audio_config)(struct device *dev,
			    struct omap_dss_audio *dss_audio);
};

#if IS_ENABLED(CONFIG_SND_OMAP_SOC_HDMI_AUDIO)
int omap_hdmi_audio_register(struct omap_hdmi_audio *hdmi_audio);
void omap_hdmi_audio_unregister(struct device *dev);
#else
static inline int omap_hdmi_audio_register(struct omap_hdmi_audio *hdmi_audio)
{
	return 0;
}
static inline void omap_hdmi_audio_unregister(struct device *dev)
{
}
#endif

#endif /* __OMAP_HDMI_AUDIO_H__ */
