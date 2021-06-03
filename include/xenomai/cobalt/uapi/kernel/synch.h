/*
 * Copyright (C) 2001-2013 Philippe Gerum <rpm@xenomai.org>.
 * Copyright (C) 2008, 2009 Jan Kiszka <jan.kiszka@siemens.com>.
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
#ifndef _COBALT_UAPI_KERNEL_SYNCH_H
#define _COBALT_UAPI_KERNEL_SYNCH_H

#include <cobalt/uapi/kernel/types.h>

/* Creation flags */
#define XNSYNCH_FIFO    0x0
#define XNSYNCH_PRIO    0x1
#define XNSYNCH_PI      0x2
#define XNSYNCH_DREORD  0x4
#define XNSYNCH_OWNER   0x8
#define XNSYNCH_PP      0x10

/* Fast lock API */
static inline int xnsynch_fast_is_claimed(xnhandle_t handle)
{
	return (handle & XNSYNCH_FLCLAIM) != 0;
}

static inline xnhandle_t xnsynch_fast_claimed(xnhandle_t handle)
{
	return handle | XNSYNCH_FLCLAIM;
}

static inline xnhandle_t xnsynch_fast_ceiling(xnhandle_t handle)
{
	return handle | XNSYNCH_FLCEIL;
}

static inline int
xnsynch_fast_owner_check(atomic_t *fastlock, xnhandle_t ownerh)
{
	return (xnhandle_get_id(atomic_read(fastlock)) == ownerh) ?
		0 : -EPERM;
}

static inline
int xnsynch_fast_acquire(atomic_t *fastlock, xnhandle_t new_ownerh)
{
	xnhandle_t h;

	h = atomic_cmpxchg(fastlock, XN_NO_HANDLE, new_ownerh);
	if (h != XN_NO_HANDLE) {
		if (xnhandle_get_id(h) == new_ownerh)
			return -EBUSY;

		return -EAGAIN;
	}

	return 0;
}

static inline
int xnsynch_fast_release(atomic_t *fastlock, xnhandle_t cur_ownerh)
{
	return (xnhandle_t)atomic_cmpxchg(fastlock, cur_ownerh, XN_NO_HANDLE)
		== cur_ownerh;
}

/* Local/shared property */
static inline int xnsynch_is_shared(xnhandle_t handle)
{
	return (handle & XNSYNCH_PSHARED) != 0;
}

#endif /* !_COBALT_UAPI_KERNEL_SYNCH_H */
