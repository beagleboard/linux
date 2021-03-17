/*
 * Copyright &copy; 2006 Gilles Chanteperdrix <gch@xenomai.org>
 *
 * Xenomai is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, Inc., 675 Mass Ave, Cambridge MA 02139,
 * USA; either version 2 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#ifndef _COBALT_KERNEL_PPD_H
#define _COBALT_KERNEL_PPD_H

#include <linux/types.h>
#include <linux/atomic.h>
#include <linux/rbtree.h>
#include <cobalt/kernel/heap.h>

struct cobalt_umm {
	struct xnheap heap;
	atomic_t refcount;
	void (*release)(struct cobalt_umm *umm);
};

struct cobalt_ppd {
	struct cobalt_umm umm;
	atomic_t refcnt;
	char *exe_path;
	struct rb_root fds;
};

extern struct cobalt_ppd cobalt_kernel_ppd;

#endif /* _COBALT_KERNEL_PPD_H */
