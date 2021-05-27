/*
 * Analogy for Linux, driver related features
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
#include <linux/module.h>
#include <linux/fs.h>
#include <rtdm/analogy/device.h>

#include "proc.h"

static LIST_HEAD(a4l_drvs);

/* --- Driver list management functions --- */

int a4l_lct_drv(char *pin, struct a4l_driver ** pio)
{
	struct list_head *this;
	int ret = -EINVAL;

	__a4l_dbg(1, core_dbg, "name=%s\n", pin);

	/* Goes through the linked list so as to find
	   a driver instance with the same name */
	list_for_each(this, &a4l_drvs) {
		struct a4l_driver *drv = list_entry(this, struct a4l_driver, list);

		if (strcmp(drv->board_name, pin) == 0) {
			/* The argument pio can be NULL
			   if there is no need to retrieve the pointer */
			if (pio != NULL)
				*pio = drv;
			ret = 0;
			break;
		}
	}

	return ret;
}

int a4l_register_drv(struct a4l_driver * drv)
{
	if (!rtdm_available())
		return -ENOSYS;

	__a4l_dbg(1, core_dbg, "board name=%s\n", drv->board_name);

	if (a4l_lct_drv(drv->board_name, NULL) != 0) {
		list_add(&drv->list, &a4l_drvs);
		return 0;
	} else
		return -EINVAL;
}

int a4l_unregister_drv(struct a4l_driver * drv)
{
	__a4l_dbg(1, core_dbg, "board name=%s\n", drv->board_name);

	if (a4l_lct_drv(drv->board_name, NULL) == 0) {
		/* Here, we consider the argument is pointing
		   to a real driver struct (not a blank structure
		   with only the name field properly set */
		list_del(&drv->list);
		return 0;
	} else
		return -EINVAL;
}

#ifdef CONFIG_PROC_FS

/* --- Driver list proc section --- */

int a4l_rdproc_drvs(struct seq_file *p, void *data)
{
	int i = 0;
	struct list_head *this;

	seq_printf(p, "--  Analogy drivers --\n\n");

	seq_printf(p, "| idx | board name \n");

	list_for_each(this, &a4l_drvs) {
		struct a4l_driver *drv = list_entry(this, struct a4l_driver, list);
		seq_printf(p, "|  %02d | %s \n", i++, drv->board_name);
	}
	return 0;
}

#endif /* CONFIG_PROC_FS */
