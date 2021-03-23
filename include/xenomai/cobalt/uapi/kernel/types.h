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
#ifndef _COBALT_UAPI_KERNEL_TYPES_H
#define _COBALT_UAPI_KERNEL_TYPES_H

#include <linux/types.h>
#include <cobalt/uapi/kernel/limits.h>

typedef __u64 xnticks_t;

typedef __s64 xnsticks_t;

typedef __u32 xnhandle_t;

#define XN_NO_HANDLE		((xnhandle_t)0)
#define XN_HANDLE_INDEX_MASK	((xnhandle_t)0xf0000000)

/* Fixed bits (part of the identifier) */
#define XNSYNCH_PSHARED		((xnhandle_t)0x40000000)

/* Transient bits (expressing a status) */
#define XNSYNCH_FLCLAIM		((xnhandle_t)0x80000000) /* Contended. */
#define XNSYNCH_FLCEIL		((xnhandle_t)0x20000000) /* Ceiling active. */

#define XN_HANDLE_TRANSIENT_MASK	(XNSYNCH_FLCLAIM|XNSYNCH_FLCEIL)

/*
 * Strip all special bits from the handle, only retaining the object
 * index value in the registry.
 */
static inline xnhandle_t xnhandle_get_index(xnhandle_t handle)
{
	return handle & ~XN_HANDLE_INDEX_MASK;
}

/*
 * Strip the transient bits from the handle, only retaining the fixed
 * part making the identifier.
 */
static inline xnhandle_t xnhandle_get_id(xnhandle_t handle)
{
	return handle & ~XN_HANDLE_TRANSIENT_MASK;
}

#endif /* !_COBALT_UAPI_KERNEL_TYPES_H */
