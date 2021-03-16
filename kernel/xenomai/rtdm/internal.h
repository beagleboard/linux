/*
 * Copyright (C) 2005-2007 Jan Kiszka <jan.kiszka@web.de>.
 * Copyright (C) 2005 Joerg Langenberg <joerg.langenberg@gmx.net>.
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

#ifndef _RTDM_INTERNAL_H
#define _RTDM_INTERNAL_H

#include <linux/types.h>
#include <linux/list.h>
#include <linux/sem.h>
#include <linux/file.h>
#include <linux/atomic.h>
#include <cobalt/kernel/tree.h>
#include <cobalt/kernel/lock.h>
#include <rtdm/driver.h>

static inline void __rtdm_get_device(struct rtdm_device *device)
{
	atomic_inc(&device->refcount);
}

void __rtdm_put_device(struct rtdm_device *device);

struct rtdm_device *__rtdm_get_namedev(const char *path);

struct rtdm_device *__rtdm_get_protodev(int protocol_family,
					int socket_type);

void __rtdm_dev_close(struct rtdm_fd *fd);

int __rtdm_dev_ioctl_core(struct rtdm_fd *fd,
			  unsigned int request, void __user *arg);

int __rtdm_mmap_from_fdop(struct rtdm_fd *fd, size_t len, off_t offset,
			  int prot, int flags, void **pptr);

int rtdm_init(void);

void rtdm_cleanup(void);

extern const struct file_operations rtdm_dumb_fops;

#endif /* _RTDM_INTERNAL_H */
