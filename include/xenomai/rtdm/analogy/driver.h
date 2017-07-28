/**
 * @file
 * Analogy for Linux, driver facilities
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
#ifndef _COBALT_RTDM_ANALOGY_DRIVER_H
#define _COBALT_RTDM_ANALOGY_DRIVER_H

#include <linux/list.h>
#include <rtdm/analogy/rtdm_helpers.h>
#include <rtdm/analogy/context.h>
#include <rtdm/analogy/buffer.h>

struct seq_file;
struct a4l_link_desc;
struct a4l_device;

/** Structure containing driver declaration data.
 *
 *  @see rt_task_inquire()
 */
/* Analogy driver descriptor */
struct a4l_driver {

	/* List stuff */
	struct list_head list;
			   /**< List stuff */

	/* Visible description stuff */
	struct module *owner;
	               /**< Pointer to module containing the code */
	unsigned int flags;
	               /**< Type / status driver's flags */
	char *board_name;
		       /**< Board name */
	char *driver_name;
	               /**< driver name */
	int privdata_size;
		       /**< Size of the driver's private data */

	/* Init/destroy procedures */
	int (*attach) (struct a4l_device *, struct a4l_link_desc *);
								      /**< Attach procedure */
	int (*detach) (struct a4l_device *);
				   /**< Detach procedure */

};

/* Driver list related functions */

int a4l_register_drv(struct a4l_driver * drv);
int a4l_unregister_drv(struct a4l_driver * drv);
int a4l_lct_drv(char *pin, struct a4l_driver ** pio);
#ifdef CONFIG_PROC_FS
int a4l_rdproc_drvs(struct seq_file *p, void *data);
#endif /* CONFIG_PROC_FS */

#endif /* !_COBALT_RTDM_ANALOGY_DRIVER_H */
