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
#include "internal.h"
#include "thread.h"
#include "clock.h"
#include "event.h"
#include <trace/events/cobalt-posix.h>

/*
 * Cobalt event notification services
 *
 * An event flag group is a synchronization object represented by a
 * regular native integer; every available bit in such word can be
 * used to map a user-defined event flag.  When a flag is set, the
 * associated event is said to have occurred.
 *
 * Xenomai threads and interrupt handlers can use event flags to
 * signal the occurrence of events to other threads; those threads can
 * either wait for the events to occur in a conjunctive manner (all
 * awaited events must have occurred to wake up), or in a disjunctive
 * way (at least one of the awaited events must have occurred to wake
 * up).
 *
 * We expose this non-POSIX feature through the internal API, as a
 * fast IPC mechanism available to the Copperplate interface.
 */

struct event_wait_context {
	struct xnthread_wait_context wc;
	unsigned int value;
	int mode;
};

COBALT_SYSCALL(event_init, current,
	       (struct cobalt_event_shadow __user *u_event,
		unsigned int value, int flags))
{
	struct cobalt_event_shadow shadow;
	struct cobalt_event_state *state;
	int pshared, synflags, ret;
	struct cobalt_event *event;
	struct cobalt_umm *umm;
	unsigned long stateoff;
	spl_t s;

	trace_cobalt_event_init(u_event, value, flags);

	event = xnmalloc(sizeof(*event));
	if (event == NULL)
		return -ENOMEM;

	pshared = (flags & COBALT_EVENT_SHARED) != 0;
	umm = &cobalt_ppd_get(pshared)->umm;
	state = cobalt_umm_alloc(umm, sizeof(*state));
	if (state == NULL) {
		xnfree(event);
		return -EAGAIN;
	}

	ret = xnregistry_enter_anon(event, &event->resnode.handle);
	if (ret) {
		cobalt_umm_free(umm, state);
		xnfree(event);
		return ret;
	}

	event->state = state;
	event->flags = flags;
	synflags = (flags & COBALT_EVENT_PRIO) ? XNSYNCH_PRIO : XNSYNCH_FIFO;
	xnsynch_init(&event->synch, synflags, NULL);
	state->value = value;
	state->flags = 0;
	state->nwaiters = 0;
	stateoff = cobalt_umm_offset(umm, state);
	XENO_BUG_ON(COBALT, stateoff != (__u32)stateoff);

	xnlock_get_irqsave(&nklock, s);
	cobalt_add_resource(&event->resnode, event, pshared);
	event->magic = COBALT_EVENT_MAGIC;
	xnlock_put_irqrestore(&nklock, s);

	shadow.flags = flags;
	shadow.handle = event->resnode.handle;
	shadow.state_offset = (__u32)stateoff;

	return cobalt_copy_to_user(u_event, &shadow, sizeof(*u_event));
}

int __cobalt_event_wait(struct cobalt_event_shadow __user *u_event,
			unsigned int bits,
			unsigned int __user *u_bits_r,
			int mode, const struct timespec *ts)
{
	unsigned int rbits = 0, testval;
	xnticks_t timeout = XN_INFINITE;
	struct cobalt_event_state *state;
	xntmode_t tmode = XN_RELATIVE;
	struct event_wait_context ewc;
	struct cobalt_event *event;
	xnhandle_t handle;
	int ret = 0, info;
	spl_t s;

	handle = cobalt_get_handle_from_user(&u_event->handle);

	if (ts) {
		timeout = ts2ns(ts);
		if (timeout) {
			timeout++;
			tmode = XN_ABSOLUTE;
		} else
			timeout = XN_NONBLOCK;
		trace_cobalt_event_timedwait(u_event, bits, mode, ts);
	} else
		trace_cobalt_event_wait(u_event, bits, mode);

	xnlock_get_irqsave(&nklock, s);

	event = xnregistry_lookup(handle, NULL);
	if (event == NULL || event->magic != COBALT_EVENT_MAGIC) {
		ret = -EINVAL;
		goto out;
	}

	state = event->state;

	if (bits == 0) {
		/*
		 * Special case: we don't wait for any event, we only
		 * return the current flag group value.
		 */
		rbits = state->value;
		goto out;
	}

	state->flags |= COBALT_EVENT_PENDED;
	rbits = state->value & bits;
	testval = mode & COBALT_EVENT_ANY ? rbits : state->value;
	if (rbits && rbits == testval)
		goto done;

	if (timeout == XN_NONBLOCK) {
		ret = -EWOULDBLOCK;
		goto done;
	}

	ewc.value = bits;
	ewc.mode = mode;
	xnthread_prepare_wait(&ewc.wc);
	state->nwaiters++;
	info = xnsynch_sleep_on(&event->synch, timeout, tmode);
	if (info & XNRMID) {
		ret = -EIDRM;
		goto out;
	}
	if (info & (XNBREAK|XNTIMEO)) {
		state->nwaiters--;
		ret = (info & XNBREAK) ? -EINTR : -ETIMEDOUT;
	} else
		rbits = ewc.value;
done:
	if (!xnsynch_pended_p(&event->synch))
		state->flags &= ~COBALT_EVENT_PENDED;
out:
	xnlock_put_irqrestore(&nklock, s);

	if (ret == 0 &&
	    cobalt_copy_to_user(u_bits_r, &rbits, sizeof(rbits)))
		return -EFAULT;

	return ret;
}

COBALT_SYSCALL(event_wait, primary,
	       (struct cobalt_event_shadow __user *u_event,
		unsigned int bits,
		unsigned int __user *u_bits_r,
		int mode, const struct timespec __user *u_ts))
{
	struct timespec ts, *tsp = NULL;
	int ret;

	if (u_ts) {
		tsp = &ts;
		ret = cobalt_copy_from_user(&ts, u_ts, sizeof(ts));
		if (ret)
			return ret;
	}

	return __cobalt_event_wait(u_event, bits, u_bits_r, mode, tsp);
}

COBALT_SYSCALL(event_sync, current,
	       (struct cobalt_event_shadow __user *u_event))
{
	unsigned int bits, waitval, testval;
	struct xnthread_wait_context *wc;
	struct cobalt_event_state *state;
	struct event_wait_context *ewc;
	struct cobalt_event *event;
	struct xnthread *p, *tmp;
	xnhandle_t handle;
	int ret = 0;
	spl_t s;

	handle = cobalt_get_handle_from_user(&u_event->handle);

	xnlock_get_irqsave(&nklock, s);

	event = xnregistry_lookup(handle, NULL);
	if (event == NULL || event->magic != COBALT_EVENT_MAGIC) {
		ret = -EINVAL;
		goto out;
	}

	/*
	 * Userland has already updated the bitmask, our job is to
	 * wake up any thread which could be satisfied by its current
	 * value.
	 */
	state = event->state;
	bits = state->value;

	xnsynch_for_each_sleeper_safe(p, tmp, &event->synch) {
		wc = xnthread_get_wait_context(p);
		ewc = container_of(wc, struct event_wait_context, wc);
		waitval = ewc->value & bits;
		testval = ewc->mode & COBALT_EVENT_ANY ? waitval : ewc->value;
		if (waitval && waitval == testval) {
			state->nwaiters--;
			ewc->value = waitval;
			xnsynch_wakeup_this_sleeper(&event->synch, p);
		}
	}

	xnsched_run();
out:
	xnlock_put_irqrestore(&nklock, s);

	return ret;
}

COBALT_SYSCALL(event_destroy, current,
	       (struct cobalt_event_shadow __user *u_event))
{
	struct cobalt_event *event;
	xnhandle_t handle;
	spl_t s;

	trace_cobalt_event_destroy(u_event);

	handle = cobalt_get_handle_from_user(&u_event->handle);

	xnlock_get_irqsave(&nklock, s);

	event = xnregistry_lookup(handle, NULL);
	if (event == NULL || event->magic != COBALT_EVENT_MAGIC) {
		xnlock_put_irqrestore(&nklock, s);
		return -EINVAL;
	}

	cobalt_event_reclaim(&event->resnode, s); /* drops lock */
	
	return 0;
}

COBALT_SYSCALL(event_inquire, current,
	       (struct cobalt_event_shadow __user *u_event,
		struct cobalt_event_info __user *u_info,
		pid_t __user *u_waitlist,
		size_t waitsz))
{
	int nrpend = 0, nrwait = 0, nrpids, ret = 0;
	unsigned long pstamp, nstamp = 0;
	struct cobalt_event_info info;
	struct cobalt_event *event;
	pid_t *t = NULL, fbuf[16];
	struct xnthread *thread;
	xnhandle_t handle;
	spl_t s;

	handle = cobalt_get_handle_from_user(&u_event->handle);

	nrpids = waitsz / sizeof(pid_t);

	xnlock_get_irqsave(&nklock, s);

	for (;;) {
		pstamp = nstamp;
		event = xnregistry_lookup(handle, &nstamp);
		if (event == NULL || event->magic != COBALT_EVENT_MAGIC) {
			xnlock_put_irqrestore(&nklock, s);
			return -EINVAL;
		}
		/*
		 * Allocate memory to return the wait list without
		 * holding any lock, then revalidate the handle.
		 */
		if (t == NULL) {
			nrpend = 0;
			if (!xnsynch_pended_p(&event->synch))
				break;
			xnsynch_for_each_sleeper(thread, &event->synch)
				nrpend++;
			if (u_waitlist == NULL)
				break;
			xnlock_put_irqrestore(&nklock, s);
			if (nrpids > nrpend)
				nrpids = nrpend;
			if (nrpend <= ARRAY_SIZE(fbuf))
				t = fbuf; /* Use fast buffer. */
			else {
				t = xnmalloc(nrpend * sizeof(pid_t));
				if (t == NULL)
					return -ENOMEM;
			}
			xnlock_get_irqsave(&nklock, s);
		} else if (pstamp == nstamp)
			break;
		else {
			xnlock_put_irqrestore(&nklock, s);
			if (t != fbuf)
				xnfree(t);
			t = NULL;
			xnlock_get_irqsave(&nklock, s);
		}
	}

	info.flags = event->flags;
	info.value = event->value;
	info.nrwait = nrpend;

	if (xnsynch_pended_p(&event->synch) && u_waitlist != NULL) {
		xnsynch_for_each_sleeper(thread, &event->synch) {
			if (nrwait >= nrpids)
				break;
			t[nrwait++] = xnthread_host_pid(thread);
		}
	}

	xnlock_put_irqrestore(&nklock, s);

	ret = cobalt_copy_to_user(u_info, &info, sizeof(info));
	if (ret == 0 && nrwait > 0)
		ret = cobalt_copy_to_user(u_waitlist, t, nrwait * sizeof(pid_t));

	if (t && t != fbuf)
		xnfree(t);

	return ret ?: nrwait;
}

void cobalt_event_reclaim(struct cobalt_resnode *node, spl_t s)
{
	struct cobalt_event *event;
	struct cobalt_umm *umm;
	int pshared;

	event = container_of(node, struct cobalt_event, resnode);
	xnregistry_remove(node->handle);
	cobalt_del_resource(node);
	xnsynch_destroy(&event->synch);
	pshared = (event->flags & COBALT_EVENT_SHARED) != 0;
	xnlock_put_irqrestore(&nklock, s);

	umm = &cobalt_ppd_get(pshared)->umm;
	cobalt_umm_free(umm, event->state);
	xnfree(event);
}
