/*
 * Copyright (C) 2013 Philippe Gerum <rpm@xenomai.org>.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
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
#ifndef _COBALT_POSIX_PROCESS_H
#define _COBALT_POSIX_PROCESS_H

#include <linux/list.h>
#include <linux/bitmap.h>
#include <cobalt/kernel/ppd.h>

#define KEVENT_PROPAGATE   0
#define KEVENT_STOP        1

#define NR_PERSONALITIES  4
#if BITS_PER_LONG < NR_PERSONALITIES
#error "NR_PERSONALITIES overflows internal bitmap"
#endif

struct mm_struct;
struct xnthread_personality;
struct cobalt_timer;

struct cobalt_resources {
	struct list_head condq;
	struct list_head mutexq;
	struct list_head semq;
	struct list_head monitorq;
	struct list_head eventq;
	struct list_head schedq;
};

struct cobalt_process {
	struct mm_struct *mm;
	struct hlist_node hlink;
	struct cobalt_ppd sys_ppd;
	unsigned long permap;
	struct rb_root usems;
	struct list_head sigwaiters;
	struct cobalt_resources resources;
	struct list_head thread_list;
	DECLARE_BITMAP(timers_map, CONFIG_XENO_OPT_NRTIMERS);
	struct cobalt_timer *timers[CONFIG_XENO_OPT_NRTIMERS];
	void *priv[NR_PERSONALITIES];
	int ufeatures;
	unsigned int debugged_threads;
};

struct cobalt_resnode {
	struct cobalt_resources *scope;
	struct cobalt_process *owner;
	struct list_head next;
	xnhandle_t handle;
};

int cobalt_register_personality(struct xnthread_personality *personality);

int cobalt_unregister_personality(int xid);

struct xnthread_personality *cobalt_push_personality(int xid);

void cobalt_pop_personality(struct xnthread_personality *prev);

int cobalt_bind_core(int ufeatures);

int cobalt_bind_personality(unsigned int magic);

struct cobalt_process *cobalt_search_process(struct mm_struct *mm);

int cobalt_map_user(struct xnthread *thread, __u32 __user *u_winoff);

void *cobalt_get_context(int xid);

int cobalt_yield(xnticks_t min, xnticks_t max);

int cobalt_process_init(void);

extern struct list_head cobalt_global_thread_list;

extern struct cobalt_resources cobalt_global_resources;

static inline struct cobalt_process *cobalt_current_process(void)
{
	return ipipe_current_threadinfo()->process;
}

static inline struct cobalt_process *
cobalt_set_process(struct cobalt_process *process)
{
	struct ipipe_threadinfo *p = ipipe_current_threadinfo();
	struct cobalt_process *old;

	old = p->process;
	p->process = process;

	return old;
}

static inline struct cobalt_ppd *cobalt_ppd_get(int global)
{
	struct cobalt_process *process;

	if (global || (process = cobalt_current_process()) == NULL)
		return &cobalt_kernel_ppd;

	return &process->sys_ppd;
}

static inline struct cobalt_resources *cobalt_current_resources(int pshared)
{
	struct cobalt_process *process;

	if (pshared || (process = cobalt_current_process()) == NULL)
		return &cobalt_global_resources;

	return &process->resources;
}

static inline
void __cobalt_add_resource(struct cobalt_resnode *node, int pshared)
{
	node->owner = cobalt_current_process();
	node->scope = cobalt_current_resources(pshared);
}

#define cobalt_add_resource(__node, __type, __pshared)			\
	do {								\
		__cobalt_add_resource(__node, __pshared);		\
		list_add_tail(&(__node)->next,				\
			      &((__node)->scope)->__type ## q);		\
	} while (0)

static inline
void cobalt_del_resource(struct cobalt_resnode *node)
{
	list_del(&node->next);
}

extern struct xnthread_personality *cobalt_personalities[];

extern struct xnthread_personality cobalt_personality;

#endif /* !_COBALT_POSIX_PROCESS_H */
