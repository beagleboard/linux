/*
 * Copyright (C) 2013 Philippe Gerum <rpm@xenomai.org>.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA.
 */
#ifndef _COBALT_UAPI_KERNEL_HEAP_H
#define _COBALT_UAPI_KERNEL_HEAP_H

#include <linux/types.h>

#define COBALT_MEMDEV_PRIVATE  "memdev-private"
#define COBALT_MEMDEV_SHARED   "memdev-shared"
#define COBALT_MEMDEV_SYS      "memdev-sys"

struct cobalt_memdev_stat {
	__u32 size;
	__u32 free;
};

#define MEMDEV_RTIOC_STAT	_IOR(RTDM_CLASS_MEMORY, 0, struct cobalt_memdev_stat)

#endif /* !_COBALT_UAPI_KERNEL_HEAP_H */
