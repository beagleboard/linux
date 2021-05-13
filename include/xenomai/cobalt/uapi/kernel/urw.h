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
#ifndef _COBALT_UAPI_KERNEL_URW_H
#define _COBALT_UAPI_KERNEL_URW_H

#include <linux/types.h>

/*
 * A restricted version of the kernel seqlocks with a slightly
 * different interface, allowing for unsynced reads with concurrent
 * write detection, without serializing writers.  Caller should
 * provide for proper locking to deal with concurrent updates.
 *
 * urw_t lock = URW_INITIALIZER;
 * urwstate_t tmp;
 *
 * unsynced_read_block(&tmp, &lock) {
 *          (will redo until clean read)...
 * }
 *
 * unsynced_write_block(&tmp, &lock) {
 *          ...
 * }
 *
 * This code was inspired by Wolfgang Mauerer's linux/seqlock.h
 * adaptation for Xenomai 2.6 to support the VDSO feature.
 */

typedef struct {
	__u32 sequence;
} urw_t;

typedef struct {
	__u32 token;
	__u32 dirty;
} urwstate_t;

#define URW_INITIALIZER     { 0 }
#define DEFINE_URW(__name)  urw_t __name = URW_INITIALIZER

static inline void __try_read_start(const urw_t *urw, urwstate_t *tmp)
{
	__u32 token;
repeat:
	token = ACCESS_ONCE(urw->sequence);
	smp_rmb();
	if (token & 1) {
		cpu_relax();
		goto repeat;
	}

	tmp->token = token;
	tmp->dirty = 1;
}

static inline void __try_read_end(const urw_t *urw, urwstate_t *tmp)
{
	smp_rmb();
	if (urw->sequence != tmp->token) {
		__try_read_start(urw, tmp);
		return;
	}

	tmp->dirty = 0;
}

static inline void __do_write_start(urw_t *urw, urwstate_t *tmp)
{
	urw->sequence++;
	tmp->dirty = 1;
	smp_wmb();
}

static inline void __do_write_end(urw_t *urw, urwstate_t *tmp)
{
	smp_wmb();
	tmp->dirty = 0;
	urw->sequence++;
}

static inline void unsynced_rw_init(urw_t *urw)
{
	urw->sequence = 0;
}

#define unsynced_read_block(__tmp, __urw)		\
	for (__try_read_start(__urw, __tmp);		\
	     (__tmp)->dirty; __try_read_end(__urw, __tmp))

#define unsynced_write_block(__tmp, __urw)		\
	for (__do_write_start(__urw, __tmp);		\
	     (__tmp)->dirty; __do_write_end(__urw, __tmp))

#endif /* !_COBALT_UAPI_KERNEL_URW_H */
