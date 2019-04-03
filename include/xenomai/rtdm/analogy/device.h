/*
 * Analogy for Linux, device related features
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
#ifndef _COBALT_RTDM_ANALOGY_DEVICE_H
#define _COBALT_RTDM_ANALOGY_DEVICE_H

#include <rtdm/analogy/rtdm_helpers.h>
#include <rtdm/analogy/transfer.h>
#include <rtdm/analogy/driver.h>

#define A4L_NB_DEVICES 10

#define A4L_DEV_ATTACHED_NR 0

struct a4l_device {

	/* Spinlock for global device use */
	rtdm_lock_t lock;

	/* Device specific flags */
	unsigned long flags;

	/* Driver assigned to this device thanks to attaching
	   procedure */
	struct a4l_driver *driver;

	/* Hidden description stuff */
	struct list_head subdvsq;

	/* Context-dependent stuff */
	struct a4l_transfer transfer;

	/* Private data useful for drivers functioning */
	void *priv;
};

/* --- Devices tab related functions --- */
void a4l_init_devs(void);
int a4l_check_cleanup_devs(void);
int a4l_rdproc_devs(struct seq_file *p, void *data);

/* --- Context related function / macro --- */
void a4l_set_dev(struct a4l_device_context *cxt);
#define a4l_get_dev(x) ((x)->dev)

/* --- Upper layer functions --- */
int a4l_ioctl_devcfg(struct a4l_device_context * cxt, void *arg);
int a4l_ioctl_devinfo(struct a4l_device_context * cxt, void *arg);

#endif /* !_COBALT_RTDM_ANALOGY_DEVICE_H */
