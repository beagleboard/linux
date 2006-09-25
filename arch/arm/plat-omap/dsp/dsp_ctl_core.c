/*
 * This file is part of OMAP DSP driver (DSP Gateway version 3.3.1)
 *
 * Copyright (C) 2004-2006 Nokia Corporation. All rights reserved.
 *
 * Contact: Toshihiro Kobayashi <toshihiro.kobayashi@nokia.com>
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/major.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/init.h>
#include "dsp.h"

#define CTL_MINOR	0
#define MEM_MINOR	1
#define TWCH_MINOR	2
#define ERR_MINOR	3

static struct class *dsp_ctl_class;
extern struct file_operations dsp_ctl_fops,
			      dsp_mem_fops,
			      dsp_twch_fops,
			      dsp_err_fops;

static int dsp_ctl_core_open(struct inode *inode, struct file *file)
{
	static DEFINE_MUTEX(open_lock);
	int ret = 0;

	mutex_lock_interruptible(&open_lock);
	if (omap_dsp->initialized == 0) {
		ret = dsp_late_init();
		if (ret != 0) {
			mutex_unlock(&open_lock);
			return ret;
		}
		omap_dsp->initialized = 1;
	}
	mutex_unlock(&open_lock);

	switch (iminor(inode)) {
	case CTL_MINOR:
		file->f_op = &dsp_ctl_fops;
		break;
	case MEM_MINOR:
		file->f_op = &dsp_mem_fops;
		break;
	case TWCH_MINOR:
		file->f_op = &dsp_twch_fops;
		break;
	case ERR_MINOR:
		file->f_op = &dsp_err_fops;
		break;
	default:
		return -ENXIO;
	}
	if (file->f_op && file->f_op->open)
		return file->f_op->open(inode, file);
	return 0;
}

static struct file_operations dsp_ctl_core_fops = {
	.owner = THIS_MODULE,
	.open  = dsp_ctl_core_open,
};

static const struct dev_list {
	unsigned int	minor;
	char		*devname;
	umode_t		mode;
} dev_list[] = {
	{CTL_MINOR,  "dspctl",  S_IRUSR | S_IWUSR},
	{MEM_MINOR,  "dspmem",  S_IRUSR | S_IWUSR | S_IRGRP},
	{TWCH_MINOR, "dsptwch", S_IRUSR | S_IWUSR | S_IRGRP},
	{ERR_MINOR,  "dsperr",  S_IRUSR | S_IRGRP},
};

int __init dsp_ctl_core_init(void)
{
	int retval;
	int i;

	retval = register_chrdev(OMAP_DSP_CTL_MAJOR, "dspctl",
				 &dsp_ctl_core_fops);
	if (retval < 0) {
		printk(KERN_ERR
		       "omapdsp: failed to register dspctl device: %d\n",
		       retval);
		return retval;
	}

	dsp_ctl_class = class_create(THIS_MODULE, "dspctl");
	for (i = 0; i < ARRAY_SIZE(dev_list); i++) {
		class_device_create(dsp_ctl_class, NULL,
				    MKDEV(OMAP_DSP_CTL_MAJOR,
					  dev_list[i].minor),
				    NULL, dev_list[i].devname);
	}

	return 0;
}

void dsp_ctl_core_exit(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dev_list); i++) {
		class_device_destroy(dsp_ctl_class,
				     MKDEV(OMAP_DSP_CTL_MAJOR,
					   dev_list[i].minor));
	}
	class_destroy(dsp_ctl_class);

	unregister_chrdev(OMAP_DSP_CTL_MAJOR, "dspctl");
}
