/*
 * Copyright (C) 2008 Efixo <gilles.chanteperdrix@xenomai.org>
 *
 * Xenomai is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version.
 *
 * Xenomai is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Xenomai; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 * 02111-1307, USA.
 */
#ifndef _COBALT_KERNEL_SELECT_H
#define _COBALT_KERNEL_SELECT_H

#include <cobalt/kernel/list.h>
#include <cobalt/kernel/thread.h>

/**
 * @addtogroup cobalt_core_select
 * @{
 */

#define XNSELECT_READ      0
#define XNSELECT_WRITE     1
#define XNSELECT_EXCEPT    2
#define XNSELECT_MAX_TYPES 3

struct xnselector {
	struct xnsynch synchbase;
	struct fds {
		fd_set expected;
		fd_set pending;
	} fds [XNSELECT_MAX_TYPES];
	struct list_head destroy_link;
	struct list_head bindings; /* only used by xnselector_destroy */
};

#define __NFDBITS__	(8 * sizeof(unsigned long))
#define __FDSET_LONGS__	(__FD_SETSIZE/__NFDBITS__)
#define	__FDELT__(d)	((d) / __NFDBITS__)
#define	__FDMASK__(d)	(1UL << ((d) % __NFDBITS__))

static inline void __FD_SET__(unsigned long __fd, __kernel_fd_set *__fdsetp)
{
        unsigned long __tmp = __fd / __NFDBITS__;
        unsigned long __rem = __fd % __NFDBITS__;
        __fdsetp->fds_bits[__tmp] |= (1UL<<__rem);
}

static inline void __FD_CLR__(unsigned long __fd, __kernel_fd_set *__fdsetp)
{
        unsigned long __tmp = __fd / __NFDBITS__;
        unsigned long __rem = __fd % __NFDBITS__;
        __fdsetp->fds_bits[__tmp] &= ~(1UL<<__rem);
}

static inline int __FD_ISSET__(unsigned long __fd, const __kernel_fd_set *__p)
{
        unsigned long __tmp = __fd / __NFDBITS__;
        unsigned long __rem = __fd % __NFDBITS__;
        return (__p->fds_bits[__tmp] & (1UL<<__rem)) != 0;
}

static inline void __FD_ZERO__(__kernel_fd_set *__p)
{
	unsigned long *__tmp = __p->fds_bits;
	int __i;

	__i = __FDSET_LONGS__;
	while (__i) {
		__i--;
		*__tmp = 0;
		__tmp++;
	}
}

struct xnselect {
	struct list_head bindings;
};

#define DECLARE_XNSELECT(name) struct xnselect name

struct xnselect_binding {
	struct xnselector *selector;
	struct xnselect *fd;
	unsigned int type;
	unsigned int bit_index;
	struct list_head link;  /* link in selected fds list. */
	struct list_head slink; /* link in selector list */
};

void xnselect_init(struct xnselect *select_block);

int xnselect_bind(struct xnselect *select_block,
		  struct xnselect_binding *binding,
		  struct xnselector *selector,
		  unsigned int type,
		  unsigned int bit_index,
		  unsigned int state);

int __xnselect_signal(struct xnselect *select_block, unsigned int state);

/**
 * Signal a file descriptor state change.
 *
 * @param select_block pointer to an @a xnselect structure representing the file
 * descriptor whose state changed;
 * @param state new value of the state.
 *
 * @retval 1 if rescheduling is needed;
 * @retval 0 otherwise.
 */
static inline int
xnselect_signal(struct xnselect *select_block, unsigned int state)
{
	if (!list_empty(&select_block->bindings))
		return __xnselect_signal(select_block, state);

	return 0;
}

void xnselect_destroy(struct xnselect *select_block);

int xnselector_init(struct xnselector *selector);

int xnselect(struct xnselector *selector,
	     fd_set *out_fds[XNSELECT_MAX_TYPES],
	     fd_set *in_fds[XNSELECT_MAX_TYPES],
	     int nfds,
	     xnticks_t timeout, xntmode_t timeout_mode);

void xnselector_destroy(struct xnselector *selector);

int xnselect_mount(void);

int xnselect_umount(void);

/** @} */

#endif /* _COBALT_KERNEL_SELECT_H */
