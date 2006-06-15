/*
 * drivers/media/video/omap/sensor_if.h
 *
 * Copyright (C) 2004 Texas Instruments, Inc. 
 * 
 * Sensor interface to OMAP camera capture drivers
 * Sensor driver should implement this interface
 *
 * This package is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation. 
 * 
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR 
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED 
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE. 
 */
 
#ifndef OMAP_SENSOR_IF_H
#define OMAP_SENSOR_IF_H

#define OMAP_SENSOR_NAME_LEN		31

struct omap_camera_sensor {
	unsigned int version;
	char name[OMAP_SENSOR_NAME_LEN + 1];

	void *(*init)(struct v4l2_pix_format *);
	int (*cleanup)(void *);

	int (*power_on)(void *);
	int (*power_off)(void *);

	int (*enum_pixformat)(struct v4l2_fmtdesc *, void *);
	int (*try_format)(struct v4l2_pix_format *, void *);

	unsigned long (*calc_xclk)(struct v4l2_pix_format *,
				   struct v4l2_fract *, void *);

	int (*configure)(struct v4l2_pix_format *, unsigned long,
			 struct v4l2_fract *, void *);

	int (*query_control) (struct v4l2_queryctrl *, void *);
	int (*get_control)(struct v4l2_control *, void *);
	int (*set_control)(struct v4l2_control *, void *);

};
	
#endif
