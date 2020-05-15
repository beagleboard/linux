/*
 * Copyright (C) 2012 Philippe Gerum <rpm@xenomai.org>
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

#ifndef _COBALT_POSIX_EVENT_H
#define _COBALT_POSIX_EVENT_H

#include <cobalt/kernel/synch.h>
#include <cobalt/uapi/event.h>
#include <xenomai/posix/syscall.h>
#include <xenomai/posix/process.h>

struct cobalt_resources;
struct cobalt_process;

struct cobalt_event {
	unsigned int magic;
	unsigned int value;
	int flags;
	struct xnsynch synch;
	struct cobalt_event_state *state;
	struct cobalt_resnode resnode;
};

int __cobalt_event_wait(struct cobalt_event_shadow __user *u_event,
			unsigned int bits,
			unsigned int __user *u_bits_r,
			int mode, const struct timespec *ts);

COBALT_SYSCALL_DECL(event_init,
		    (struct cobalt_event_shadow __user *u_evtsh,
		     unsigned int value,
		     int flags));

COBALT_SYSCALL_DECL(event_wait,
		    (struct cobalt_event_shadow __user *u_evtsh,
		     unsigned int bits,
		     unsigned int __user *u_bits_r,
		     int mode,
		     const struct timespec __user *u_ts));

COBALT_SYSCALL_DECL(event_sync,
		    (struct cobalt_event_shadow __user *u_evtsh));

COBALT_SYSCALL_DECL(event_destroy,
		    (struct cobalt_event_shadow __user *u_evtsh));

COBALT_SYSCALL_DECL(event_inquire,
		    (struct cobalt_event_shadow __user *u_event,
		     struct cobalt_event_info __user *u_info,
		     pid_t __user *u_waitlist,
		     size_t waitsz));

void cobalt_event_reclaim(struct cobalt_resnode *node,
			  spl_t s);

#endif /* !_COBALT_POSIX_EVENT_H */
