/*
 * Copyright (C) 2009 Wolfgang Mauerer <wolfgang.mauerer@siemens.com>.
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
#ifndef _COBALT_UAPI_KERNEL_VDSO_H
#define _COBALT_UAPI_KERNEL_VDSO_H

#include <cobalt/uapi/kernel/urw.h>

struct xnvdso_hostrt_data {
	__u64 wall_sec;
	__u64 wtom_sec;
	__u64 cycle_last;
	__u64 mask;
	__u32 wall_nsec;
	__u32 wtom_nsec;
	__u32 mult;
	__u32 shift;
	__u32 live;
	urw_t lock;
};

/*
 * Data shared between the Cobalt kernel and applications, which lives
 * in the shared memory heap (COBALT_MEMDEV_SHARED).
 * xnvdso_hostrt_data.features tells which data is present. Notice
 * that struct xnvdso may only grow, but never shrink.
 */
struct xnvdso {
	__u64 features;
	/* XNVDSO_FEAT_HOST_REALTIME */
	struct xnvdso_hostrt_data hostrt_data;
	/* XNVDSO_FEAT_WALLCLOCK_OFFSET */
	__u64 wallclock_offset;
};

/* For each shared feature, add a flag below. */

#define XNVDSO_FEAT_HOST_REALTIME	0x0000000000000001ULL
#define XNVDSO_FEAT_WALLCLOCK_OFFSET	0x0000000000000002ULL

static inline int xnvdso_test_feature(struct xnvdso *vdso,
				      __u64 feature)
{
	return (vdso->features & feature) != 0;
}

#endif /* !_COBALT_UAPI_KERNEL_VDSO_H */
