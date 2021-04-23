/***
 *
 *  rtmac_proc.c
 *
 *  rtmac - real-time networking medium access control subsystem
 *  Copyright (C) 2002 Marc Kleine-Budde <kleine-budde@gmx.de>
 *                2004 Jan Kiszka <jan.kiszka@web.de>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/module.h>

#include <rtnet_internal.h>
#include <rtmac/rtmac_disc.h>
#include <rtmac/rtmac_vnic.h>
#include <rtmac/rtmac_proc.h>

#ifdef CONFIG_XENO_OPT_VFILE
struct xnvfile_directory rtmac_proc_root;

static struct xnvfile_regular_ops rtnet_rtmac_disciplines_vfile_ops = {
	.show = rtnet_rtmac_disciplines_show,
};

static struct xnvfile_regular rtnet_rtmac_disciplines_vfile = {
	.ops = &rtnet_rtmac_disciplines_vfile_ops,
};

static struct xnvfile_regular_ops rtnet_rtmac_vnics_vfile_ops = {
	.show = rtnet_rtmac_vnics_show,
};

static struct xnvfile_regular rtnet_rtmac_vnics_vfile = {
	.ops = &rtnet_rtmac_vnics_vfile_ops,
};

static int rtnet_rtmac_disc_show(struct xnvfile_regular_iterator *it,
				 void *data)
{
	struct rtmac_proc_entry *entry;
	entry = container_of(it->vfile, struct rtmac_proc_entry, vfile);
	return entry->handler(it, data);
}

static struct xnvfile_regular_ops rtnet_rtmac_disc_vfile_ops = {
	.show = rtnet_rtmac_disc_show,
};

int rtmac_disc_proc_register(struct rtmac_disc *disc)
{
	int i, err;
	struct rtmac_proc_entry *entry;

	for (i = 0; i < disc->nr_proc_entries; i++) {
		entry = &disc->proc_entries[i];

		entry->vfile.ops = &rtnet_rtmac_disc_vfile_ops;
		err = xnvfile_init_regular(entry->name, &entry->vfile,
					   &rtmac_proc_root);
		if (err < 0) {
			while (--i >= 0)
				xnvfile_destroy_regular(
					&disc->proc_entries[i].vfile);
			return err;
		}
	}

	return 0;
}

void rtmac_disc_proc_unregister(struct rtmac_disc *disc)
{
	int i;

	for (i = 0; i < disc->nr_proc_entries; i++)
		xnvfile_destroy_regular(&disc->proc_entries[i].vfile);
}

int rtmac_proc_register(void)
{
	int err;

	err = xnvfile_init_dir("rtmac", &rtmac_proc_root, &rtnet_proc_root);
	if (err < 0)
		goto err1;

	err = xnvfile_init_regular("disciplines",
				   &rtnet_rtmac_disciplines_vfile,
				   &rtmac_proc_root);
	if (err < 0)
		goto err2;

	err = xnvfile_init_regular("vnics", &rtnet_rtmac_vnics_vfile,
				   &rtmac_proc_root);
	if (err < 0)
		goto err3;

	return 0;

err3:
	xnvfile_destroy_regular(&rtnet_rtmac_disciplines_vfile);

err2:
	xnvfile_destroy_dir(&rtmac_proc_root);

err1:
	/*ERRMSG*/ printk("RTmac: unable to initialize /proc entries\n");
	return err;
}

void rtmac_proc_release(void)
{
	xnvfile_destroy_regular(&rtnet_rtmac_vnics_vfile);
	xnvfile_destroy_regular(&rtnet_rtmac_disciplines_vfile);
	xnvfile_destroy_dir(&rtmac_proc_root);
}

#endif /* CONFIG_XENO_OPT_VFILE */
