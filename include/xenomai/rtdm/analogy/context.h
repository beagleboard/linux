/*
 * Analogy for Linux, context structure / macros declarations
 *
 * Copyright (C) 1997-2000 David A. Schleef <ds@schleef.org>
 * Copyright (C) 2008 Alexis Berlemont <alexis.berlemont@free.fr>
 *
 * Xenomai is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Xenomai is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Xenomai; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#ifndef _COBALT_RTDM_ANALOGY_CONTEXT_H
#define _COBALT_RTDM_ANALOGY_CONTEXT_H

#include <rtdm/driver.h>

struct a4l_device;
struct a4l_buffer;

struct a4l_device_context {
	/* The adequate device pointer
	   (retrieved thanks to minor at open time) */
	struct a4l_device *dev;

	/* The buffer structure contains everything to transfer data
	   from asynchronous acquisition operations on a specific
	   subdevice */
	struct a4l_buffer *buffer;
};

static inline int a4l_get_minor(struct a4l_device_context *cxt)
{
	/* Get a pointer on the container structure */
	struct rtdm_fd *fd = rtdm_private_to_fd(cxt);
	/* Get the minor index */
	return rtdm_fd_minor(fd);
}

#endif /* !_COBALT_RTDM_ANALOGY_CONTEXT_H */
