/***
 *
 *  include/rtcfg/rtcfg_file.h
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

#ifndef __RTCFG_FILE_H_
#define __RTCFG_FILE_H_

#include <linux/list.h>
#include <linux/types.h>

struct rtcfg_file {
	struct list_head entry;
	int ref_count;
	const char *name;
	size_t size;
	void *buffer;
};

struct rtcfg_file *rtcfg_get_file(const char *filename);
void rtcfg_add_file(struct rtcfg_file *file);
int rtcfg_release_file(struct rtcfg_file *file);

#endif /* __RTCFG_FILE_H_ */
