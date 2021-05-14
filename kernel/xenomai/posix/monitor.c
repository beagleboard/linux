/*
 * Copyright (C) 2011 Philippe Gerum <rpm@xenomai.org>
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
#include "monitor.h"
#include <trace/events/cobalt-posix.h>

/*
 * The Cobalt monitor is a double-wait condition object, serializing
 * accesses through a gate. It behaves like a mutex + two condition
 * variables combo with extended signaling logic. Folding several
 * conditions and the serialization support into a single object
 * performs better on low end hw caches and allows for specific
 * optimizations, compared to using separate general-purpose mutex and
 * condvars. This object is used by the Copperplate interface
 * internally when it runs over the Cobalt core.
 *
 * Threads can wait for some resource(s) to be granted (consumer
 * side), or wait for the available resource(s) to drain (producer
 * side).  Therefore, signals are thread-directed for the grant side,
 * and monitor-directed for the drain side.
 *
 * Typically, a consumer would wait for the GRANT condition to be
 * satisfied, signaling the DRAINED condition when more resources
 * could be made available if the protocol implements output
 * contention (e.g. the write side of a message queue waiting for the
 * consumer to release message slots). Conversely, a producer would
 * wait for the DRAINED condition to be satisfied, issuing GRANT
 * signals once more resources have been made available to the
 * consumer.
 *
 * Implementation-wise, the monitor logic is shared with the Cobalt
 * thread object.
 */
COBALT_SYSCALL(monitor_init, current,
	       (struct cobalt_monitor_shadow __user *u_mon,
		clockid_t clk_id, int flags))
{
	struct cobalt_monitor_shadow shadow;
	struct cobalt_monitor_state *state;
	struct cobalt_monitor *mon;
	int pshared, tmode, ret;
	struct cobalt_umm *umm;
	unsigned long stateoff;
	spl_t s;

	tmode = clock_flag(TIMER_ABSTIME, clk_id);
	if (tmode < 0)
		return -EINVAL;

	mon = xnmalloc(sizeof(*mon));
	if (mon == NULL)
		return -ENOMEM;

	pshared = (flags & COBALT_MONITOR_SHARED) != 0;
	umm = &cobalt_ppd_get(pshared)->umm;
	state = cobalt_umm_alloc(umm, sizeof(*state));
	if (state == NULL) {
		xnfree(mon);
		return -EAGAIN;
	}

	ret = xnregistry_enter_anon(mon, &mon->resnode.handle);
	if (ret) {
		cobalt_umm_free(umm, state);
		xnfree(mon);
		return ret;
	}

	mon->state = state;
	xnsynch_init(&mon->gate, XNSYNCH_PI, &state->owner);
	xnsynch_init(&mon->drain, XNSYNCH_PRIO, NULL);
	mon->flags = flags;
	mon->tmode = tmode;
	INIT_LIST_HEAD(&mon->waiters);

	xnlock_get_irqsave(&nklock, s);
	cobalt_add_resource(&mon->resnode, monitor, pshared);
	mon->magic = COBALT_MONITOR_MAGIC;
	xnlock_put_irqrestore(&nklock, s);

	state->flags = 0;
	stateoff = cobalt_umm_offset(umm, state);
	XENO_BUG_ON(COBALT, stateoff != (__u32)stateoff);
	shadow.flags = flags;
	shadow.handle = mon->resnode.handle;
	shadow.state_offset = (__u32)stateoff;

	return cobalt_copy_to_user(u_mon, &shadow, sizeof(*u_mon));
}

/* nklock held, irqs off */
static int monitor_enter(xnhandle_t handle, struct xnthread *curr)
{
	struct cobalt_monitor *mon;
	int info;

	mon = xnregistry_lookup(handle, NULL); /* (Re)validate. */
	if (mon == NULL || mon->magic != COBALT_MONITOR_MAGIC)
		return -EINVAL;

	info = xnsynch_acquire(&mon->gate, XN_INFINITE, XN_RELATIVE);
	if (info)
		/* Break or error, no timeout possible. */
		return info & XNBREAK ? -EINTR : -EINVAL;

	mon->state->flags &= ~(COBALT_MONITOR_SIGNALED|COBALT_MONITOR_BROADCAST);

	return 0;
}

COBALT_SYSCALL(monitor_enter, primary,
	       (struct cobalt_monitor_shadow __user *u_mon))
{
	struct xnthread *curr = xnthread_current();
	xnhandle_t handle;
	int ret;
	spl_t s;

	handle = cobalt_get_handle_from_user(&u_mon->handle);

	xnlock_get_irqsave(&nklock, s);
	ret = monitor_enter(handle, curr);
	xnlock_put_irqrestore(&nklock, s);

	return ret;
}

/* nklock held, irqs off */
static void monitor_wakeup(struct cobalt_monitor *mon)
{
	struct cobalt_monitor_state *state = mon->state;
	struct cobalt_thread *thread, *tmp;
	struct xnthread *p;
	int bcast;

	/*
	 * Having the GRANT signal pending does not necessarily mean
	 * that somebody is actually waiting for it, so we have to
	 * check both conditions below.
	 */
	bcast = (state->flags & COBALT_MONITOR_BROADCAST) != 0;
	if ((state->flags & COBALT_MONITOR_GRANTED) == 0 ||
	    list_empty(&mon->waiters))
		goto drain;

	/*
	 * Unblock waiters requesting a grant, either those who
	 * received it only or all of them, depending on the broadcast
	 * bit.
	 *
	 * We update the PENDED flag to inform userland about the
	 * presence of waiters, so that it may decide not to issue any
	 * syscall for exiting the monitor if nobody else is waiting
	 * at the gate.
	 */
	list_for_each_entry_safe(thread, tmp, &mon->waiters, monitor_link) {
		p = &thread->threadbase;
		/*
		 * A thread might receive a grant signal albeit it
		 * does not wait on a monitor, or it might have timed
		 * out before we got there, so we really have to check
		 * that ->wchan does match our sleep queue.
		 */
		if (bcast ||
		    (p->u_window->grant_value && p->wchan == &thread->monitor_synch)) {
			xnsynch_wakeup_this_sleeper(&thread->monitor_synch, p);
			list_del_init(&thread->monitor_link);
		}
	}
drain:
	/*
	 * Unblock threads waiting for a drain event if that signal is
	 * pending, either one or all, depending on the broadcast
	 * flag.
	 */
	if ((state->flags & COBALT_MONITOR_DRAINED) != 0 &&
	    xnsynch_pended_p(&mon->drain)) {
		if (bcast)
			xnsynch_flush(&mon->drain, 0);
		else
			xnsynch_wakeup_one_sleeper(&mon->drain);
	}

	if (list_empty(&mon->waiters) && !xnsynch_pended_p(&mon->drain))
		state->flags &= ~COBALT_MONITOR_PENDED;
}

int __cobalt_monitor_wait(struct cobalt_monitor_shadow __user *u_mon,
			  int event, const struct timespec *ts,
			  int __user *u_ret)
{
	struct cobalt_thread *curr = cobalt_current_thread();
	struct cobalt_monitor_state *state;
	xnticks_t timeout = XN_INFINITE;
	int ret = 0, opret = 0, info;
	struct cobalt_monitor *mon;
	struct xnsynch *synch;
	xnhandle_t handle;
	xntmode_t tmode;
	spl_t s;

	handle = cobalt_get_handle_from_user(&u_mon->handle);

	if (ts)
		timeout = ts2ns(ts) + 1;

	xnlock_get_irqsave(&nklock, s);

	mon = xnregistry_lookup(handle, NULL);
	if (mon == NULL || mon->magic != COBALT_MONITOR_MAGIC) {
		ret = -EINVAL;
		goto out;
	}

	/*
	 * The current thread might have sent signals to the monitor
	 * it wants to sleep on: wake up satisfied waiters before
	 * going to sleep.
	 */
	state = mon->state;
	if (state->flags & COBALT_MONITOR_SIGNALED)
		monitor_wakeup(mon);

	synch = &curr->monitor_synch;
	if (event & COBALT_MONITOR_WAITDRAIN)
		synch = &mon->drain;
	else {
		curr->threadbase.u_window->grant_value = 0;
		list_add_tail(&curr->monitor_link, &mon->waiters);
	}

	/*
	 * Tell userland that somebody is now waiting for a signal, so
	 * that later exiting the monitor on the producer side will
	 * trigger a wakeup syscall.
	 *
	 * CAUTION: we must raise the PENDED flag while holding the
	 * gate mutex, to prevent a signal from sneaking in from a
	 * remote CPU without the producer issuing the corresponding
	 * wakeup call when dropping the gate lock.
	 */
	state->flags |= COBALT_MONITOR_PENDED;

	tmode = ts ? mon->tmode : XN_RELATIVE;

	/* Release the gate prior to waiting, all atomically. */
	xnsynch_release(&mon->gate, &curr->threadbase);

	info = xnsynch_sleep_on(synch, timeout, tmode);
	if (info) {
		if ((event & COBALT_MONITOR_WAITDRAIN) == 0 &&
		    !list_empty(&curr->monitor_link))
			list_del_init(&curr->monitor_link);

		if (list_empty(&mon->waiters) && !xnsynch_pended_p(&mon->drain))
			state->flags &= ~COBALT_MONITOR_PENDED;

		if (info & XNBREAK) {
			opret = -EINTR;
			goto out;
		}
		if (info & XNTIMEO)
			opret = -ETIMEDOUT;
	}

	ret = monitor_enter(handle, &curr->threadbase);
out:
	xnlock_put_irqrestore(&nklock, s);

	__xn_put_user(opret, u_ret);

	return ret;
}

COBALT_SYSCALL(monitor_wait, nonrestartable,
	       (struct cobalt_monitor_shadow __user *u_mon,
	       int event, const struct timespec __user *u_ts,
	       int __user *u_ret))
{
	struct timespec ts, *tsp = NULL;
	int ret;

	if (u_ts) {
		tsp = &ts;
		ret = cobalt_copy_from_user(&ts, u_ts, sizeof(ts));
		if (ret)
			return ret;
	}

	return __cobalt_monitor_wait(u_mon, event, tsp, u_ret);
}

COBALT_SYSCALL(monitor_sync, nonrestartable,
	       (struct cobalt_monitor_shadow __user *u_mon))
{
	struct cobalt_monitor *mon;
	struct xnthread *curr;
	xnhandle_t handle;
	int ret = 0;
	spl_t s;

	handle = cobalt_get_handle_from_user(&u_mon->handle);
	curr = xnthread_current();

	xnlock_get_irqsave(&nklock, s);

	mon = xnregistry_lookup(handle, NULL);
	if (mon == NULL || mon->magic != COBALT_MONITOR_MAGIC)
		ret = -EINVAL;
	else if (mon->state->flags & COBALT_MONITOR_SIGNALED) {
		monitor_wakeup(mon);
		xnsynch_release(&mon->gate, curr);
		xnsched_run();
		ret = monitor_enter(handle, curr);
	}

	xnlock_put_irqrestore(&nklock, s);

	return ret;
}

COBALT_SYSCALL(monitor_exit, primary,
	       (struct cobalt_monitor_shadow __user *u_mon))
{
	struct cobalt_monitor *mon;
	struct xnthread *curr;
	xnhandle_t handle;
	int ret = 0;
	spl_t s;

	handle = cobalt_get_handle_from_user(&u_mon->handle);
	curr = xnthread_current();

	xnlock_get_irqsave(&nklock, s);

	mon = xnregistry_lookup(handle, NULL);
	if (mon == NULL || mon->magic != COBALT_MONITOR_MAGIC)
		ret = -EINVAL;
	else {
		if (mon->state->flags & COBALT_MONITOR_SIGNALED)
			monitor_wakeup(mon);

		xnsynch_release(&mon->gate, curr);
		xnsched_run();
	}

	xnlock_put_irqrestore(&nklock, s);

	return ret;
}

COBALT_SYSCALL(monitor_destroy, primary,
	       (struct cobalt_monitor_shadow __user *u_mon))
{
	struct cobalt_monitor_state *state;
	struct cobalt_monitor *mon;
	struct xnthread *curr;
	xnhandle_t handle;
	int ret = 0;
	spl_t s;

	handle = cobalt_get_handle_from_user(&u_mon->handle);
	curr = xnthread_current();

	xnlock_get_irqsave(&nklock, s);

	mon = xnregistry_lookup(handle, NULL);
	if (mon == NULL || mon->magic != COBALT_MONITOR_MAGIC) {
		ret = -EINVAL;
		goto fail;
	}

	state = mon->state;
	if ((state->flags & COBALT_MONITOR_PENDED) != 0 ||
	    xnsynch_pended_p(&mon->drain) || !list_empty(&mon->waiters)) {
		ret = -EBUSY;
		goto fail;
	}

	/*
	 * A monitor must be destroyed by the thread currently holding
	 * its gate lock.
	 */
	if (xnsynch_owner_check(&mon->gate, curr)) {
		ret = -EPERM;
		goto fail;
	}

	cobalt_monitor_reclaim(&mon->resnode, s); /* drops lock */

	xnsched_run();

	return 0;
 fail:
	xnlock_put_irqrestore(&nklock, s);
	
	return ret;
}

void cobalt_monitor_reclaim(struct cobalt_resnode *node, spl_t s)
{
	struct cobalt_monitor *mon;
	struct cobalt_umm *umm;
	int pshared;

	mon = container_of(node, struct cobalt_monitor, resnode);
	pshared = (mon->flags & COBALT_MONITOR_SHARED) != 0;
	xnsynch_destroy(&mon->gate);
	xnsynch_destroy(&mon->drain);
	xnregistry_remove(node->handle);
	cobalt_del_resource(node);
	cobalt_mark_deleted(mon);
	xnlock_put_irqrestore(&nklock, s);

	umm = &cobalt_ppd_get(pshared)->umm;
	cobalt_umm_free(umm, mon->state);
	xnfree(mon);
}
