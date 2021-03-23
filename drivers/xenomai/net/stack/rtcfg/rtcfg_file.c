/***
 *
 *  rtcfg/rtcfg_file.c
 *
 *  Real-Time Configuration Distribution Protocol
 *
 *  Copyright (C) 2004 Jan Kiszka <jan.kiszka@web.de>
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

#include <linux/init.h>

#include <rtdm/driver.h>
#include <rtcfg_chrdev.h>
#include <rtcfg/rtcfg.h>
#include <rtcfg/rtcfg_file.h>

/* Note:
 * We don't need any special lock protection while manipulating the
 * rtcfg_files list. The list is only accessed through valid connections, and
 * connections are already lock-protected.
 */
LIST_HEAD(rtcfg_files);

struct rtcfg_file *rtcfg_get_file(const char *filename)
{
	struct list_head *entry;
	struct rtcfg_file *file;

	RTCFG_DEBUG(4, "RTcfg: looking for file %s\n", filename);

	list_for_each (entry, &rtcfg_files) {
		file = list_entry(entry, struct rtcfg_file, entry);

		if (strcmp(file->name, filename) == 0) {
			file->ref_count++;

			RTCFG_DEBUG(4,
				    "RTcfg: reusing file entry, now %d users\n",
				    file->ref_count);

			return file;
		}
	}

	return NULL;
}

void rtcfg_add_file(struct rtcfg_file *file)
{
	RTCFG_DEBUG(4, "RTcfg: adding file %s to list\n", file->name);

	file->ref_count = 1;
	list_add_tail(&file->entry, &rtcfg_files);
}

int rtcfg_release_file(struct rtcfg_file *file)
{
	if (--file->ref_count == 0) {
		RTCFG_DEBUG(4, "RTcfg: removing file %s from list\n",
			    file->name);

		list_del(&file->entry);
	}

	return file->ref_count;
}
