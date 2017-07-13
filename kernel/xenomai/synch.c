/*
 * Copyright (C) 2001-2008 Philippe Gerum <rpm@xenomai.org>.
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
#include <stdarg.h>
#include <linux/signal.h>
#include <cobalt/kernel/sched.h>
#include <cobalt/kernel/synch.h>
#include <cobalt/kernel/thread.h>
#include <cobalt/kernel/clock.h>
#include <cobalt/uapi/signal.h>
#include <trace/events/cobalt-core.h>

/**
 * @ingroup cobalt_core
 * @defgroup cobalt_core_synch Thread synchronization services
 * @{
 */

/**
 * @fn void xnsynch_init(struct xnsynch *synch, int flags,
 *                       atomic_t *fastlock)
 *
 * @brief Initialize a synchronization object.
 *
 * Initializes a synchronization object. Xenomai threads can wait on
 * and signal such objects for serializing access to resources.
 * This object has built-in support for priority inheritance.
 *
 * @param synch The address of a synchronization object descriptor the
 * nucleus will use to store the object-specific data.  This
 * descriptor must always be valid while the object is active
 * therefore it must be allocated in permanent memory.
 *
 * @param flags A set of creation flags affecting the operation. The
 * valid flags are:
 *
 * - XNSYNCH_PRIO causes the threads waiting for the resource to pend
 * in priority order. Otherwise, FIFO ordering is used (XNSYNCH_FIFO).
 *
 * - XNSYNCH_OWNER indicates that the synchronization object shall
 * track the resource ownership, allowing a single owner at most at
 * any point in time. Note that setting this flag implies the use of
 * xnsynch_acquire() and xnsynch_release() instead of
 * xnsynch_sleep_on() and xnsynch_wakeup_*().
 *
 * - XNSYNCH_PIP enables priority inheritance when a priority
 * inversion is detected among threads using this object.  XNSYNCH_PIP
 * enables XNSYNCH_OWNER and XNSYNCH_PRIO implicitly.
 *
 * - XNSYNCH_DREORD (Disable REORDering) tells the nucleus that the
 * wait queue should not be reordered whenever the priority of a
 * blocked thread it holds is changed. If this flag is not specified,
 * changing the priority of a blocked thread using
 * xnthread_set_schedparam() will cause this object's wait queue to be
 * reordered according to the new priority level, provided the
 * synchronization object makes the waiters wait by priority order on
 * the awaited resource (XNSYNCH_PRIO).
 *
 * @param fastlock Address of the fast lock word to be associated with
 * a synchronization object with ownership tracking. Therefore, a
 * valid fast-lock address is required if XNSYNCH_OWNER is set in @a
 * flags.
 *
 * @coretags{task-unrestricted}
 */
void xnsynch_init(struct xnsynch *synch, int flags, atomic_t *fastlock)
{
	if (flags & XNSYNCH_PIP)
		flags |= XNSYNCH_PRIO | XNSYNCH_OWNER;	/* Obviously... */

	synch->status = flags & ~XNSYNCH_CLAIMED;
	synch->owner = NULL;
	synch->cleanup = NULL;	/* Only works for PIP-enabled objects. */
	synch->wprio = -1;
	INIT_LIST_HEAD(&synch->pendq);

	if (flags & XNSYNCH_OWNER) {
		BUG_ON(fastlock == NULL);
		synch->fastlock = fastlock;
		atomic_set(fastlock, XN_NO_HANDLE);
	} else
		synch->fastlock = NULL;
}
EXPORT_SYMBOL_GPL(xnsynch_init);

/**
 * @fn void xnsynch_destroy(struct xnsynch *synch)
 * @brief Destroy a synchronization object.
 *
 * Destroys the synchronization object @a synch, unblocking all
 * waiters with the XNRMID status.
 *
 * @return XNSYNCH_RESCHED is returned if at least one thread is
 * unblocked, which means the caller should invoke xnsched_run() for
 * applying the new scheduling state. Otherwise, XNSYNCH_DONE is
 * returned.

 * @sideeffect Same as xnsynch_flush().
 *
 * @coretags{task-unrestricted}
 */
int xnsynch_destroy(struct xnsynch *synch)
{
	int ret;
	
	ret = xnsynch_flush(synch, XNRMID);
	XENO_BUG_ON(COBALT, synch->status & XNSYNCH_CLAIMED);

	return ret;
}
EXPORT_SYMBOL_GPL(xnsynch_destroy);

/**
 * @fn int xnsynch_sleep_on(struct xnsynch *synch, xnticks_t timeout, xntmode_t timeout_mode);
 * @brief Sleep on an ownerless synchronization object.
 *
 * Makes the calling thread sleep on the specified synchronization
 * object, waiting for it to be signaled.
 *
 * This service should be called by upper interfaces wanting the
 * current thread to pend on the given resource. It must not be used
 * with synchronization objects that are supposed to track ownership
 * (XNSYNCH_OWNER).
 *
 * @param synch The descriptor address of the synchronization object
 * to sleep on.
 *
 * @param timeout The timeout which may be used to limit the time the
 * thread pends on the resource. This value is a wait time given as a
 * count of nanoseconds. It can either be relative, absolute
 * monotonic, or absolute adjustable depending on @a
 * timeout_mode. Passing XN_INFINITE @b and setting @a mode to
 * XN_RELATIVE specifies an unbounded wait. All other values are used
 * to initialize a watchdog timer.
 *
 * @param timeout_mode The mode of the @a timeout parameter. It can
 * either be set to XN_RELATIVE, XN_ABSOLUTE, or XN_REALTIME (see also
 * xntimer_start()).
 *
 * @return A bitmask which may include zero or one information bit
 * among XNRMID, XNTIMEO and XNBREAK, which should be tested by the
 * caller, for detecting respectively: object deletion, timeout or
 * signal/unblock conditions which might have happened while waiting.
 *
 * @coretags{primary-only, might-switch}
 */
int xnsynch_sleep_on(struct xnsynch *synch, xnticks_t timeout,
		     xntmode_t timeout_mode)
{
	struct xnthread *thread;
	spl_t s;

	primary_mode_only();

	XENO_BUG_ON(COBALT, synch->status & XNSYNCH_OWNER);

	thread = xnthread_current();

	if (IS_ENABLED(CONFIG_XENO_OPT_DEBUG_MUTEX_SLEEP) &&
	    thread->res_count > 0 &&
	    xnthread_test_state(thread, XNWARN))
		xnthread_signal(thread, SIGDEBUG, SIGDEBUG_MUTEX_SLEEP);
	
	xnlock_get_irqsave(&nklock, s);

	trace_cobalt_synch_sleepon(synch, thread);

	if ((synch->status & XNSYNCH_PRIO) == 0) /* i.e. FIFO */
		list_add_tail(&thread->plink, &synch->pendq);
	else /* i.e. priority-sorted */
		list_add_priff(thread, &synch->pendq, wprio, plink);

	xnthread_suspend(thread, XNPEND, timeout, timeout_mode, synch);

	xnlock_put_irqrestore(&nklock, s);

	return xnthread_test_info(thread, XNRMID|XNTIMEO|XNBREAK);
}
EXPORT_SYMBOL_GPL(xnsynch_sleep_on);

/**
 * @fn struct xnthread *xnsynch_wakeup_one_sleeper(struct xnsynch *synch);
 * @brief Unblock the heading thread from wait.
 *
 * This service wakes up the thread which is currently leading the
 * synchronization object's pending list. The sleeping thread is
 * unblocked from its pending state, but no reschedule is performed.
 *
 * This service should be called by upper interfaces wanting to signal
 * the given resource so that a single waiter is resumed. It must not
 * be used with synchronization objects that are supposed to track
 * ownership (XNSYNCH_OWNER not set).
 *
 * @param synch The descriptor address of the synchronization object
 * whose ownership is changed.
 *
 * @return The descriptor address of the unblocked thread.
 *
 * @coretags{unrestricted}
 */
struct xnthread *xnsynch_wakeup_one_sleeper(struct xnsynch *synch)
{
	struct xnthread *thread;
	spl_t s;

	XENO_BUG_ON(COBALT, synch->status & XNSYNCH_OWNER);

	xnlock_get_irqsave(&nklock, s);

	if (list_empty(&synch->pendq)) {
		thread = NULL;
		goto out;
	}

	trace_cobalt_synch_wakeup(synch);
	thread = list_first_entry(&synch->pendq, struct xnthread, plink);
	list_del(&thread->plink);
	thread->wchan = NULL;
	xnthread_resume(thread, XNPEND);
out:
	xnlock_put_irqrestore(&nklock, s);

	return thread;
}
EXPORT_SYMBOL_GPL(xnsynch_wakeup_one_sleeper);

int xnsynch_wakeup_many_sleepers(struct xnsynch *synch, int nr)
{
	struct xnthread *thread, *tmp;
	int nwakeups = 0;
	spl_t s;

	XENO_BUG_ON(COBALT, synch->status & XNSYNCH_OWNER);

	xnlock_get_irqsave(&nklock, s);

	if (list_empty(&synch->pendq))
		goto out;

	trace_cobalt_synch_wakeup_many(synch);

	list_for_each_entry_safe(thread, tmp, &synch->pendq, plink) {
		if (nwakeups++ >= nr)
			break;
		list_del(&thread->plink);
		thread->wchan = NULL;
		xnthread_resume(thread, XNPEND);
	}
out:
	xnlock_put_irqrestore(&nklock, s);

	return nwakeups;
}
EXPORT_SYMBOL_GPL(xnsynch_wakeup_many_sleepers);

/**
 * @fn void xnsynch_wakeup_this_sleeper(struct xnsynch *synch, struct xnthread *sleeper);
 * @brief Unblock a particular thread from wait.
 *
 * This service wakes up a specific thread which is currently pending on
 * the given synchronization object. The sleeping thread is unblocked
 * from its pending state, but no reschedule is performed.
 *
 * This service should be called by upper interfaces wanting to signal
 * the given resource so that a specific waiter is resumed. It must not
 * be used with synchronization objects that are supposed to track
 * ownership (XNSYNCH_OWNER not set).
 *
 * @param synch The descriptor address of the synchronization object
 * whose ownership is changed.
 *
 * @param sleeper The thread to unblock which MUST be currently linked
 * to the synchronization object's pending queue (i.e. synch->pendq).
 *
 * @coretags{unrestricted}
 */
void xnsynch_wakeup_this_sleeper(struct xnsynch *synch, struct xnthread *sleeper)
{
	spl_t s;

	XENO_BUG_ON(COBALT, synch->status & XNSYNCH_OWNER);

	xnlock_get_irqsave(&nklock, s);

	trace_cobalt_synch_wakeup(synch);
	list_del(&sleeper->plink);
	sleeper->wchan = NULL;
	xnthread_resume(sleeper, XNPEND);

	xnlock_put_irqrestore(&nklock, s);
}
EXPORT_SYMBOL_GPL(xnsynch_wakeup_this_sleeper);

/*
 * @internal
 *
 * This service is used by the PIP code to raise/lower a thread's
 * priority. The base priority value is _not_ changed and if ready,
 * the thread is always moved at the end of its priority group.
 *
 * @note There is no point in propagating Xenomai policy/priority
 * changes to linux/libc, since doing so would be papering over a
 * basic priority inversion issue in the application code. I.e. a
 * Xenomai mutex owner shall NOT enter secondary mode until it
 * eventually drops the resource - this is even triggering a debug
 * signal-, so there is no point in boosting the scheduling
 * policy/priority settings applicable to that mode anyway.
 */
static void xnsynch_renice_thread(struct xnthread *thread,
				  struct xnthread *target)
{
	/* Apply the scheduling policy of "target" to "thread" */
	xnsched_track_policy(thread, target);

	if (thread->wchan)
		xnsynch_requeue_sleeper(thread);
}

/**
 * @fn int xnsynch_try_acquire(struct xnsynch *synch);
 * @brief Try acquiring the ownership of a synchronization object.
 *
 * This service should be called by upper interfaces wanting the
 * current thread to acquire the ownership of the given resource. If
 * the resource is already assigned to another thread, the call
 * returns with an error code.
 *
 * This service must be used only with synchronization objects that
 * track ownership (XNSYNCH_OWNER set.
 *
 * @param synch The descriptor address of the synchronization object
 * to acquire.
 *
 * @return Zero is returned if @a synch has been successfully
 * acquired. Otherwise:
 *
 * - -EDEADLK is returned if @a synch is currently held by the calling
 * thread.
 *
 * - -EBUSY is returned if @a synch is currently held by another
 * thread.
 *
 * @coretags{primary-only}
 */
int xnsynch_try_acquire(struct xnsynch *synch)
{
	struct xnthread *curr;
	atomic_t *lockp;
	xnhandle_t h;

	primary_mode_only();

	XENO_BUG_ON(COBALT, (synch->status & XNSYNCH_OWNER) == 0);

	curr = xnthread_current();
	lockp = xnsynch_fastlock(synch);
	trace_cobalt_synch_try_acquire(synch, curr);

	h = atomic_cmpxchg(lockp, XN_NO_HANDLE, curr->handle);
	if (h != XN_NO_HANDLE)
		return xnhandle_get_id(h) == curr->handle ?
			-EDEADLK : -EBUSY;

	xnsynch_set_owner(synch, curr);
	xnthread_get_resource(curr);

	return 0;
}
EXPORT_SYMBOL_GPL(xnsynch_try_acquire);

/**
 * @fn int xnsynch_acquire(struct xnsynch *synch, xnticks_t timeout, xntmode_t timeout_mode);
 * @brief Acquire the ownership of a synchronization object.
 *
 * This service should be called by upper interfaces wanting the
 * current thread to acquire the ownership of the given resource. If
 * the resource is already assigned to another thread, the caller is
 * suspended.
 *
 * This service must be used only with synchronization objects that
 * track ownership (XNSYNCH_OWNER set.
 *
 * @param synch The descriptor address of the synchronization object
 * to acquire.
 *
 * @param timeout The timeout which may be used to limit the time the
 * thread pends on the resource. This value is a wait time given as a
 * count of nanoseconds. It can either be relative, absolute
 * monotonic, or absolute adjustable depending on @a
 * timeout_mode. Passing XN_INFINITE @b and setting @a mode to
 * XN_RELATIVE specifies an unbounded wait. All other values are used
 * to initialize a watchdog timer.
 *
 * @param timeout_mode The mode of the @a timeout parameter. It can
 * either be set to XN_RELATIVE, XN_ABSOLUTE, or XN_REALTIME (see also
 * xntimer_start()).
 *
 * @return A bitmask which may include zero or one information bit
 * among XNRMID, XNTIMEO and XNBREAK, which should be tested by the
 * caller, for detecting respectively: object deletion, timeout or
 * signal/unblock conditions which might have happened while waiting.
 *
 * @coretags{primary-only, might-switch}
 *
 * @note Unlike xnsynch_try_acquire(), this call does NOT check for
 * invalid recursive locking request, which means that such request
 * will always cause a deadlock for the caller.
 */
int xnsynch_acquire(struct xnsynch *synch, xnticks_t timeout,
		    xntmode_t timeout_mode)
{
	struct xnthread *curr, *owner;
	xnhandle_t currh, h, oldh;
	atomic_t *lockp;
	spl_t s;

	primary_mode_only();

	XENO_BUG_ON(COBALT, (synch->status & XNSYNCH_OWNER) == 0);

	curr = xnthread_current();
	currh = curr->handle;
	lockp = xnsynch_fastlock(synch);
	trace_cobalt_synch_acquire(synch, curr);
redo:
	/* Basic form of xnsynch_try_acquire(). */
	h = atomic_cmpxchg(lockp, XN_NO_HANDLE, currh);
	if (likely(h == XN_NO_HANDLE)) {
		xnsynch_set_owner(synch, curr);
		xnthread_get_resource(curr);
		return 0;
	}

	xnlock_get_irqsave(&nklock, s);

	/*
	 * Set claimed bit.  In case it appears to be set already,
	 * re-read its state under nklock so that we don't miss any
	 * change between the lock-less read and here. But also try to
	 * avoid cmpxchg where possible. Only if it appears not to be
	 * set, start with cmpxchg directly.
	 */
	if (xnsynch_fast_is_claimed(h)) {
		oldh = atomic_read(lockp);
		goto test_no_owner;
	}

	do {
		oldh = atomic_cmpxchg(lockp, h, xnsynch_fast_claimed(h));
		if (likely(oldh == h))
			break;
	test_no_owner:
		if (oldh == XN_NO_HANDLE) {
			/* Mutex released from another cpu. */
			xnlock_put_irqrestore(&nklock, s);
			goto redo;
		}
		h = oldh;
	} while (!xnsynch_fast_is_claimed(h));

	owner = xnthread_lookup(h);
	if (owner == NULL) {
		/*
		 * The handle is broken, therefore pretend that the
		 * synch object was deleted to signal an error.
		 */
		xnthread_set_info(curr, XNRMID);
		goto out;
	}

	xnsynch_set_owner(synch, owner);
	xnsynch_detect_relaxed_owner(synch, curr);

	if ((synch->status & XNSYNCH_PRIO) == 0) { /* i.e. FIFO */
		list_add_tail(&curr->plink, &synch->pendq);
		goto block;
	}

	if (curr->wprio > owner->wprio) {
		if (xnthread_test_info(owner, XNWAKEN) && owner->wwake == synch) {
			/* Ownership is still pending, steal the resource. */
			synch->owner = curr;
			xnthread_clear_info(curr, XNRMID | XNTIMEO | XNBREAK);
			xnthread_set_info(owner, XNROBBED);
			goto grab;
		}

		list_add_priff(curr, &synch->pendq, wprio, plink);

		if (synch->status & XNSYNCH_PIP) {
			if (!xnthread_test_state(owner, XNBOOST)) {
				owner->bprio = owner->cprio;
				xnthread_set_state(owner, XNBOOST);
			}

			if (synch->status & XNSYNCH_CLAIMED)
				list_del(&synch->link);
			else
				synch->status |= XNSYNCH_CLAIMED;

			synch->wprio = curr->wprio;
			list_add_priff(synch, &owner->claimq, wprio, link);
			xnsynch_renice_thread(owner, curr);
		}
	} else
		list_add_priff(curr, &synch->pendq, wprio, plink);
block:
	xnthread_suspend(curr, XNPEND, timeout, timeout_mode, synch);
	curr->wwake = NULL;
	xnthread_clear_info(curr, XNWAKEN);

	if (xnthread_test_info(curr, XNRMID | XNTIMEO | XNBREAK))
		goto out;

	if (xnthread_test_info(curr, XNROBBED)) {
		/*
		 * Somebody stole us the ownership while we were ready
		 * to run, waiting for the CPU: we need to wait again
		 * for the resource.
		 */
		if (timeout_mode != XN_RELATIVE || timeout == XN_INFINITE) {
			xnlock_put_irqrestore(&nklock, s);
			goto redo;
		}
		timeout = xntimer_get_timeout_stopped(&curr->rtimer);
		if (timeout > 1) { /* Otherwise, it's too late. */
			xnlock_put_irqrestore(&nklock, s);
			goto redo;
		}
		xnthread_set_info(curr, XNTIMEO);
		goto out;
	}
 grab:
	xnthread_get_resource(curr);

	if (xnsynch_pended_p(synch))
		currh = xnsynch_fast_claimed(currh);

	/* Set new ownership for this mutex. */
	atomic_set(lockp, currh);
out:
	xnlock_put_irqrestore(&nklock, s);

	return xnthread_test_info(curr, XNRMID|XNTIMEO|XNBREAK);
}
EXPORT_SYMBOL_GPL(xnsynch_acquire);

static void clear_boost(struct xnsynch *synch, struct xnthread *owner)
{
	struct xnthread *target;
	struct xnsynch *hsynch;
	int wprio;

	list_del(&synch->link);
	synch->status &= ~XNSYNCH_CLAIMED;
	wprio = owner->bprio + owner->sched_class->weight;

	if (list_empty(&owner->claimq)) {
		xnthread_clear_state(owner, XNBOOST);
		target = owner;
	} else {
		/* Find the highest priority needed to enforce the PIP. */
		hsynch = list_first_entry(&owner->claimq, struct xnsynch, link);
		XENO_BUG_ON(COBALT, list_empty(&hsynch->pendq));
		target = list_first_entry(&hsynch->pendq, struct xnthread, plink);
		if (target->wprio > wprio)
			wprio = target->wprio;
		else
			target = owner;
	}

	if (owner->wprio != wprio &&
	    !xnthread_test_state(owner, XNZOMBIE))
		xnsynch_renice_thread(owner, target);
}

static struct xnthread *transfer_ownership(struct xnsynch *synch,
					   struct xnthread *lastowner)
{
	struct xnthread *nextowner;
	xnhandle_t nextownerh;
	atomic_t *lockp;
	spl_t s;

	xnlock_get_irqsave(&nklock, s);

	lockp = xnsynch_fastlock(synch);

	if (list_empty(&synch->pendq)) {
		synch->owner = NULL;
		atomic_set(lockp, XN_NO_HANDLE);
		xnlock_put_irqrestore(&nklock, s);
		return NULL;
	}

	nextowner = list_first_entry(&synch->pendq, struct xnthread, plink);
	list_del(&nextowner->plink);
	nextowner->wchan = NULL;
	nextowner->wwake = synch;
	synch->owner = nextowner;
	xnthread_set_info(nextowner, XNWAKEN);
	xnthread_resume(nextowner, XNPEND);

	if (synch->status & XNSYNCH_CLAIMED)
		clear_boost(synch, lastowner);

	nextownerh = xnsynch_pended_p(synch) ?
		xnsynch_fast_claimed(nextowner->handle) :
		xnsynch_fast_not_claimed(nextowner->handle);
	atomic_set(lockp, nextownerh);

	xnlock_put_irqrestore(&nklock, s);

	return nextowner;
}

/**
 * @fn struct xnthread *xnsynch_release(struct xnsynch *synch, struct xnthread *thread)
 * @brief Give the resource ownership to the next waiting thread.
 *
 * This service releases the ownership of the given synchronization
 * object. The thread which is currently leading the object's pending
 * list, if any, is unblocked from its pending state. However, no
 * reschedule is performed.
 *
 * This service must be used only with synchronization objects that
 * track ownership (XNSYNCH_OWNER set).
 *
 * @param synch The descriptor address of the synchronization object
 * whose ownership is changed.
 *
 * @param thread The descriptor address of the current owner.
 *
 * @return The descriptor address of the unblocked thread.
 *
 * @sideeffect
 *
 * - The effective priority of the previous resource owner might be
 * lowered to its base priority value as a consequence of the priority
 * inheritance boost being cleared.
 *
 * - The synchronization object ownership is transfered to the
 * unblocked thread.
 *
 * @coretags{primary-only, might-switch}
 */
struct xnthread *xnsynch_release(struct xnsynch *synch,
				 struct xnthread *thread)
{
	xnhandle_t threadh;
	atomic_t *lockp;

	XENO_BUG_ON(COBALT, (synch->status & XNSYNCH_OWNER) == 0);

	trace_cobalt_synch_release(synch);

	if (xnthread_put_resource(thread))
		return NULL;

	lockp = xnsynch_fastlock(synch);
	XENO_BUG_ON(COBALT, lockp == NULL);
	threadh = thread->handle;
	if (likely(xnsynch_fast_release(lockp, threadh)))
		return NULL;

	return transfer_ownership(synch, thread);
}
EXPORT_SYMBOL_GPL(xnsynch_release);

void xnsynch_requeue_sleeper(struct xnthread *thread)
{
	struct xnsynch *synch = thread->wchan;
	struct xnthread *owner;

	/*
	 * Update the position of a thread waiting for a lock w/ PIP
	 * enabled.
	 */
	if ((synch->status & XNSYNCH_PRIO) == 0)
		return;

	list_del(&thread->plink);
	list_add_priff(thread, &synch->pendq, wprio, plink);
	owner = synch->owner;

	if (owner == NULL || thread->wprio <= owner->wprio)
		return;

	/*
	 * The new (weighted) priority of the sleeping thread is
	 * higher than the priority of the current owner of the
	 * resource: we need to update the PI state.
	 */
	synch->wprio = thread->wprio;
	if (synch->status & XNSYNCH_CLAIMED) {
		/*
		 * The resource is already claimed, just reorder the
		 * claim queue.
		 */
		list_del(&synch->link);
		list_add_priff(synch, &owner->claimq, wprio, link);
	} else {
		/*
		 * The resource was NOT claimed, claim it now and
		 * boost the owner.
		 */
		synch->status |= XNSYNCH_CLAIMED;
		list_add_priff(synch, &owner->claimq, wprio, link);
		if (!xnthread_test_state(owner, XNBOOST)) {
			owner->bprio = owner->cprio;
			xnthread_set_state(owner, XNBOOST);
		}
	}
	/*
	 * Renice the owner thread, progressing in the PI chain as
	 * needed.
	 */
	xnsynch_renice_thread(owner, thread);
}
EXPORT_SYMBOL_GPL(xnsynch_requeue_sleeper);

/**
 * @fn struct xnthread *xnsynch_peek_pendq(struct xnsynch *synch);
 * @brief Access the thread leading a synch object wait queue.
 *
 * This services returns the descriptor address of to the thread leading a
 * synchronization object wait queue.
 *
 * @param synch The descriptor address of the target synchronization object.
 *
 * @return The descriptor address of the unblocked thread.
 *
 * @coretags{unrestricted}
 */
struct xnthread *xnsynch_peek_pendq(struct xnsynch *synch)
{
	struct xnthread *thread;
	spl_t s;

	xnlock_get_irqsave(&nklock, s);

	if (list_empty(&synch->pendq)) {
		thread = NULL;
		goto out;
	}

	thread = list_first_entry(&synch->pendq, struct xnthread, plink);
out:
	xnlock_put_irqrestore(&nklock, s);

	return thread;
}
EXPORT_SYMBOL_GPL(xnsynch_peek_pendq);

/**
 * @fn int xnsynch_flush(struct xnsynch *synch, int reason);
 * @brief Unblock all waiters pending on a resource.
 *
 * This service atomically releases all threads which currently sleep
 * on a given resource.
 *
 * This service should be called by upper interfaces under
 * circumstances requiring that the pending queue of a given resource
 * is cleared, such as before the resource is deleted.
 *
 * @param synch The descriptor address of the synchronization object
 * to be flushed.
 *
 * @param reason Some flags to set in the information mask of every
 * unblocked thread. Zero is an acceptable value. The following bits
 * are pre-defined by the nucleus:
 *
 * - XNRMID should be set to indicate that the synchronization object
 * is about to be destroyed (see xnthread_resume()).
 *
 * - XNBREAK should be set to indicate that the wait has been forcibly
 * interrupted (see xnthread_unblock()).
 *
 * @return XNSYNCH_RESCHED is returned if at least one thread is
 * unblocked, which means the caller should invoke xnsched_run() for
 * applying the new scheduling state. Otherwise, XNSYNCH_DONE is
 * returned.
 *
 * @sideeffect
 *
 * - The effective priority of the previous resource owner might be
 * lowered to its base priority value as a consequence of the priority
 * inheritance boost being cleared.
 *
 * - After this operation has completed, the synchronization object is
 * not owned by any thread.
 *
 * @coretags{unrestricted}
 */
int xnsynch_flush(struct xnsynch *synch, int reason)
{
	struct xnthread *sleeper, *tmp;
	int ret;
	spl_t s;

	xnlock_get_irqsave(&nklock, s);

	trace_cobalt_synch_flush(synch);

	if (list_empty(&synch->pendq)) {
		XENO_BUG_ON(COBALT, synch->status & XNSYNCH_CLAIMED);
		ret = XNSYNCH_DONE;
	} else {
		ret = XNSYNCH_RESCHED;
		list_for_each_entry_safe(sleeper, tmp, &synch->pendq, plink) {
			list_del(&sleeper->plink);
			xnthread_set_info(sleeper, reason);
			sleeper->wchan = NULL;
			xnthread_resume(sleeper, XNPEND);
		}
		if (synch->status & XNSYNCH_CLAIMED)
			clear_boost(synch, synch->owner);
	}

	xnlock_put_irqrestore(&nklock, s);

	return ret;
}
EXPORT_SYMBOL_GPL(xnsynch_flush);

void xnsynch_forget_sleeper(struct xnthread *thread)
{
	struct xnsynch *synch = thread->wchan, *nsynch;
	struct xnthread *owner, *target;

	/*
	 * Do all the necessary housekeeping chores to stop a thread
	 * from waiting on a given synchronization object.
	 */
	trace_cobalt_synch_forget(synch);

	xnthread_clear_state(thread, XNPEND);
	thread->wchan = NULL;
	list_del(&thread->plink);

	if ((synch->status & XNSYNCH_CLAIMED) == 0)
		return;

	/* Find the highest priority needed to enforce the PIP. */
	owner = synch->owner;

	if (list_empty(&synch->pendq)) {
		/* No more sleepers: clear the boost. */
		clear_boost(synch, owner);
		return;
	}

	target = list_first_entry(&synch->pendq, struct xnthread, plink);
	nsynch = list_first_entry(&owner->claimq, struct xnsynch, link);

	if (target->wprio == nsynch->wprio)
		return;		/* No change. */

	/*
	 * Reorder the claim queue, and lower the priority to the
	 * required minimum needed to prevent priority inversion.
	 */
	synch->wprio = target->wprio;
	list_del(&synch->link);
	list_add_priff(synch, &owner->claimq, wprio, link);

	nsynch = list_first_entry(&owner->claimq, struct xnsynch, link);
	if (nsynch->wprio < owner->wprio)
		xnsynch_renice_thread(owner, target);
}
EXPORT_SYMBOL_GPL(xnsynch_forget_sleeper);

void xnsynch_release_all_ownerships(struct xnthread *thread)
{
	struct xnsynch *synch, *tmp;

	/*
	 * Release all the ownerships obtained by a thread on
	 * synchronization objects. This routine must be entered
	 * interrupts off.
	 */
	xnthread_for_each_claimed_safe(synch, tmp, thread) {
		xnsynch_release(synch, thread);
		if (synch->cleanup)
			synch->cleanup(synch);
	}
}
EXPORT_SYMBOL_GPL(xnsynch_release_all_ownerships);

#if XENO_DEBUG(MUTEX_RELAXED)

/*
 * Detect when a thread is about to sleep on a synchronization
 * object currently owned by someone running in secondary mode.
 */
void xnsynch_detect_relaxed_owner(struct xnsynch *synch, struct xnthread *sleeper)
{
	if (xnthread_test_state(sleeper, XNWARN) &&
	    !xnthread_test_info(sleeper, XNPIALERT) &&
	    xnthread_test_state(synch->owner, XNRELAX)) {
		xnthread_set_info(sleeper, XNPIALERT);
		xnthread_signal(sleeper, SIGDEBUG,
				  SIGDEBUG_MIGRATE_PRIOINV);
	} else
		xnthread_clear_info(sleeper,  XNPIALERT);
}

/*
 * Detect when a thread is about to relax while holding a
 * synchronization object currently claimed by another thread, which
 * bears the TWARNSW bit (thus advertising a concern about potential
 * spurious relaxes and priority inversion). By relying on the claim
 * queue, we restrict the checks to PIP-enabled objects, but that
 * already covers most of the use cases anyway.
 */
void xnsynch_detect_claimed_relax(struct xnthread *owner)
{
	struct xnthread *sleeper;
	struct xnsynch *synch;
	spl_t s;

	xnlock_get_irqsave(&nklock, s);

	xnthread_for_each_claimed(synch, owner) {
		xnsynch_for_each_sleeper(sleeper, synch) {
			if (xnthread_test_state(sleeper, XNWARN)) {
				xnthread_set_info(sleeper, XNPIALERT);
				xnthread_signal(sleeper, SIGDEBUG,
						  SIGDEBUG_MIGRATE_PRIOINV);
			}
		}
	}

	xnlock_put_irqrestore(&nklock, s);
}

#endif /* XENO_DEBUG(MUTEX_RELAXED) */

/** @} */
