/*
 * Written by Gilles Chanteperdrix <gilles.chanteperdrix@xenomai.org>.
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
#ifndef _COBALT_UAPI_SEM_H
#define _COBALT_UAPI_SEM_H

#include <cobalt/uapi/kernel/types.h>

#define COBALT_SEM_MAGIC (0x86860707)
#define COBALT_NAMED_SEM_MAGIC (0x86860D0D)

struct cobalt_sem;

struct cobalt_sem_state {
	atomic_t value;
	__u32 flags;
};

union cobalt_sem_union {
	sem_t native_sem;
	struct cobalt_sem_shadow {
		__u32 magic;
		__s32 state_offset;
		xnhandle_t handle;
	} shadow_sem;
};

struct cobalt_sem_info {
	unsigned int value;
	int flags;
	int nrwait;
};

#define SEM_FIFO       0x1
#define SEM_PULSE      0x2
#define SEM_PSHARED    0x4
#define SEM_REPORT     0x8
#define SEM_WARNDEL    0x10
#define SEM_RAWCLOCK   0x20
#define SEM_NOBUSYDEL  0x40

#endif /* !_COBALT_UAPI_SEM_H */
