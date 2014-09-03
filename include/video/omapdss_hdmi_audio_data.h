/*
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com
 *
 * Author: Jyri Sarha <jsarha@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#ifndef __OMAPDSS_HDMI_AUDIO_DATA_H
#define __OMAPDSS_HDMI_AUDIO_DATA_H

struct omapdss_hdmi_drvdata_hdr {
	void *audio_data;
};

static inline void *omapdss_hdmi_get_audio_data(struct device *dev)
{
	struct omapdss_hdmi_drvdata_hdr *hd = dev_get_drvdata(dev);

	return hd->audio_data;
}

static inline void omapdss_hdmi_set_audio_data(struct device *dev, void *data)
{
	struct omapdss_hdmi_drvdata_hdr *hd = dev_get_drvdata(dev);

	hd->audio_data = data;
}
#endif
