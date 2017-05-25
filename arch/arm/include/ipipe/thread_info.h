/**
 * Copyright (C) 2012 Philippe Gerum <rpm@xenomai.org>.
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
#ifndef _COBALT_IPIPE_THREAD_INFO_H
#define _COBALT_IPIPE_THREAD_INFO_H

struct xnthread;
struct cobalt_process;

struct ipipe_threadinfo {
	/* Core thread backlink. */
	struct xnthread *thread;
	/* User process backlink. NULL for core threads. */
	struct cobalt_process *process;
};

static inline void __ipipe_init_threadinfo(struct ipipe_threadinfo *p)
{
	p->thread = NULL;
	p->process = NULL;
}

#endif /* !_COBALT_IPIPE_THREAD_INFO_H */
