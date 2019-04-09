/*
 * Written by Gilles Chanteperdrix <gilles.chanteperdrix@xenomai.org>.
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
#include "mutex.h"
#include "cond.h"
#include "clock.h"
#include <trace/events/cobalt-posix.h>

static inline int
pthread_cond_init(struct cobalt_cond_shadow *cnd, const struct cobalt_condattr *attr)
{
	int synch_flags = XNSYNCH_PRIO | XNSYNCH_NOPIP, ret;
	struct cobalt_cond *cond, *old_cond;
	struct cobalt_cond_state *state;
	struct cobalt_ppd *sys_ppd;
	struct list_head *condq;
	spl_t s;

	cond = xnmalloc(sizeof(*cond));
	if (cond == NULL)
		return -ENOMEM;

	sys_ppd = cobalt_ppd_get(attr->pshared);
	state = cobalt_umm_alloc(&sys_ppd->umm, sizeof(*state));
	if (state == NULL) {
		ret = -EAGAIN;
		goto fail_umm;
	}
	cond->state = state;
	state->pending_signals = 0;
	state->mutex_state_offset = ~0U;

	xnlock_get_irqsave(&nklock, s);

	condq = &cobalt_current_resources(attr->pshared)->condq;
	if (cnd->magic == COBALT_COND_MAGIC && !list_empty(condq)) {
		old_cond = xnregistry_lookup(cnd->handle, NULL);
		if (cobalt_obj_active(old_cond, COBALT_COND_MAGIC,
				      typeof(*old_cond))) {
			ret = -EBUSY;
			goto fail_register;
		}
	}

	ret = xnregistry_enter_anon(cond, &cond->resnode.handle);
	if (ret < 0)
		goto fail_register;
	if (attr->pshared)
		cond->resnode.handle |= XNSYNCH_PSHARED;
	cond->magic = COBALT_COND_MAGIC;
	xnsynch_init(&cond->synchbase, synch_flags, NULL);
	cond->attr = *attr;
	cond->mutex = NULL;
	cobalt_add_resource(&cond->resnode, cond, attr->pshared);

	cnd->handle = cond->resnode.handle;
	cnd->state_offset = cobalt_umm_offset(&sys_ppd->umm, state);
	cnd->magic = COBALT_COND_MAGIC;

	xnlock_put_irqrestore(&nklock, s);

	return 0;
fail_register:
	xnlock_put_irqrestore(&nklock, s);
	cobalt_umm_free(&sys_ppd->umm, state);
fail_umm:
	xnfree(cond);

	return ret;
}

static inline int pthread_cond_destroy(struct cobalt_cond_shadow *cnd)
{
	struct cobalt_cond *cond;
	spl_t s;

	xnlock_get_irqsave(&nklock, s);
	cond = xnregistry_lookup(cnd->handle, NULL);
	if (cond == NULL) {
		xnlock_put_irqrestore(&nklock, s);
		return -EINVAL;
	}

	if (!cobalt_obj_active(cnd, COBALT_COND_MAGIC, struct cobalt_cond_shadow)
	    || !cobalt_obj_active(cond, COBALT_COND_MAGIC, struct cobalt_cond)) {
		xnlock_put_irqrestore(&nklock, s);
		return -EINVAL;
	}

	if (cond->resnode.scope !=
	    cobalt_current_resources(cond->attr.pshared)) {
		xnlock_put_irqrestore(&nklock, s);
		return -EPERM;
	}

	if (xnsynch_pended_p(&cond->synchbase) || cond->mutex) {
		xnlock_put_irqrestore(&nklock, s);
		return -EBUSY;
	}

	cobalt_cond_reclaim(&cond->resnode, s); /* drops lock */

	cobalt_mark_deleted(cnd);

	return 0;
}

static inline int cobalt_cond_timedwait_prologue(struct xnthread *cur,
						 struct cobalt_cond *cond,
						 struct cobalt_mutex *mutex,
						 xnticks_t abs_to)
{
	int err, ret;
	spl_t s;

	xnlock_get_irqsave(&nklock, s);

	/* If another thread waiting for cond does not use the same mutex */
	if (!cobalt_obj_active(cond, COBALT_COND_MAGIC, struct cobalt_cond)
	    || (cond->mutex && cond->mutex != mutex)) {
		err = -EINVAL;
		goto unlock_and_return;
	}

	if (IS_ENABLED(CONFIG_XENO_OPT_DEBUG_POSIX_SYNCHRO) &&
	    cond->resnode.scope !=
	    cobalt_current_resources(cond->attr.pshared)) {
		err = -EPERM;
		goto unlock_and_return;
	}

	if (mutex->attr.pshared != cond->attr.pshared) {
		err = -EINVAL;
		goto unlock_and_return;
	}

	/* Unlock mutex. */
	err = cobalt_mutex_release(cur, mutex);
	if (err < 0)
		goto unlock_and_return;

	/* err == 1 means a reschedule is needed, but do not
	   reschedule here, releasing the mutex and suspension must be
	   done atomically in pthread_cond_*wait. */

	/* Bind mutex to cond. */
	if (cond->mutex == NULL) {
		cond->mutex = mutex;
		list_add_tail(&cond->mutex_link, &mutex->conds);
	}

	/* Wait for another thread to signal the condition. */
	if (abs_to != XN_INFINITE)
		ret = xnsynch_sleep_on(&cond->synchbase, abs_to,
				       clock_flag(TIMER_ABSTIME, cond->attr.clock));
	else
		ret = xnsynch_sleep_on(&cond->synchbase, XN_INFINITE, XN_RELATIVE);

	/* There are three possible wakeup conditions :
	   - cond_signal / cond_broadcast, no status bit is set, and the function
	     should return 0 ;
	   - timeout, the status XNTIMEO is set, and the function should return
	     ETIMEDOUT ;
	   - pthread_kill, the status bit XNBREAK is set, but ignored, the
	     function simply returns EINTR (used only by the user-space
	     interface, replaced by 0 anywhere else), causing a wakeup, spurious
	     or not whether pthread_cond_signal was called between pthread_kill
	     and the moment when xnsynch_sleep_on returned ;
	 */

	err = 0;

	if (ret & XNBREAK)
		err = -EINTR;
	else if (ret & XNTIMEO)
		err = -ETIMEDOUT;

unlock_and_return:
	xnlock_put_irqrestore(&nklock, s);

	return err;
}

static inline int cobalt_cond_timedwait_epilogue(struct xnthread *cur,
						 struct cobalt_cond *cond,
						 struct cobalt_mutex *mutex)
{
	int err;
	spl_t s;

	xnlock_get_irqsave(&nklock, s);

	err = __cobalt_mutex_acquire_unchecked(cur, mutex, NULL);
	if (err == -EINTR)
		goto unlock_and_return;

	/*
	 * Unbind mutex and cond, if no other thread is waiting, if
	 * the job was not already done.
	 */
	if (!xnsynch_pended_p(&cond->synchbase) && cond->mutex == mutex) {
		cond->mutex = NULL;
		list_del(&cond->mutex_link);
	}

unlock_and_return:
	xnlock_put_irqrestore(&nklock, s);

	return err;
}

COBALT_SYSCALL(cond_init, current,
	       (struct cobalt_cond_shadow __user *u_cnd,
		const struct cobalt_condattr __user *u_attr))
{
	struct cobalt_cond_shadow cnd;
	struct cobalt_condattr attr;
	int err;

	if (cobalt_copy_from_user(&cnd, u_cnd, sizeof(cnd)))
		return -EFAULT;

	if (cobalt_copy_from_user(&attr, u_attr, sizeof(attr)))
		return -EFAULT;

	trace_cobalt_cond_init(u_cnd, &attr);

	err = pthread_cond_init(&cnd, &attr);
	if (err < 0)
		return err;

	return cobalt_copy_to_user(u_cnd, &cnd, sizeof(*u_cnd));
}

COBALT_SYSCALL(cond_destroy, current,
	       (struct cobalt_cond_shadow __user *u_cnd))
{
	struct cobalt_cond_shadow cnd;
	int err;

	if (cobalt_copy_from_user(&cnd, u_cnd, sizeof(cnd)))
		return -EFAULT;

	trace_cobalt_cond_destroy(u_cnd);

	err = pthread_cond_destroy(&cnd);
	if (err < 0)
		return err;

	return cobalt_copy_to_user(u_cnd, &cnd, sizeof(*u_cnd));
}

struct us_cond_data {
	int err;
};

static inline int cond_fetch_timeout(struct timespec *ts,
				     const void __user *u_ts)
{
	return u_ts == NULL ? -EFAULT :
		cobalt_copy_from_user(ts, u_ts, sizeof(*ts));
}

int __cobalt_cond_wait_prologue(struct cobalt_cond_shadow __user *u_cnd,
				struct cobalt_mutex_shadow __user *u_mx,
				int *u_err,
				void __user *u_ts,
				int (*fetch_timeout)(struct timespec *ts,
						     const void __user *u_ts))
{
	struct xnthread *cur = xnthread_current();
	struct cobalt_cond *cond;
	struct cobalt_mutex *mx;
	struct us_cond_data d;
	struct timespec ts;
	xnhandle_t handle;
	int err, perr = 0;
	__u32 offset;

	handle = cobalt_get_handle_from_user(&u_cnd->handle);
	cond = xnregistry_lookup(handle, NULL);

	handle = cobalt_get_handle_from_user(&u_mx->handle);
	mx = xnregistry_lookup(handle, NULL);

	if (cond->mutex == NULL) {
		__xn_get_user(offset, &u_mx->state_offset);
		cond->state->mutex_state_offset = offset;
	}

	if (fetch_timeout) {
		err = fetch_timeout(&ts, u_ts);
		if (err == 0) {
			trace_cobalt_cond_timedwait(u_cnd, u_mx, &ts);
			err = cobalt_cond_timedwait_prologue(cur, cond, mx,
							     ts2ns(&ts) + 1);
		}
	} else {
		trace_cobalt_cond_wait(u_cnd, u_mx);
		err = cobalt_cond_timedwait_prologue(cur, cond, mx, XN_INFINITE);
	}

	switch(err) {
	case 0:
	case -ETIMEDOUT:
		perr = d.err = err;
		err = cobalt_cond_timedwait_epilogue(cur, cond, mx);
		break;

	case -EINTR:
		perr = err;
		d.err = 0;	/* epilogue should return 0. */
		break;

	default:
		/* Please gcc and handle the case which will never
		   happen */
		d.err = EINVAL;
	}

	if (cond->mutex == NULL)
		cond->state->mutex_state_offset = ~0U;

	if (err == -EINTR)
		__xn_put_user(d.err, u_err);

	return err == 0 ? perr : err;
}

/* pthread_cond_wait_prologue(cond, mutex, count_ptr, timed, timeout) */
COBALT_SYSCALL(cond_wait_prologue, nonrestartable,
	       (struct cobalt_cond_shadow __user *u_cnd,
		struct cobalt_mutex_shadow __user *u_mx,
		int *u_err,
		unsigned int timed,
		struct timespec __user *u_ts))
{
	return __cobalt_cond_wait_prologue(u_cnd, u_mx, u_err, u_ts,
					   timed ? cond_fetch_timeout : NULL);
}

COBALT_SYSCALL(cond_wait_epilogue, primary,
	       (struct cobalt_cond_shadow __user *u_cnd,
		struct cobalt_mutex_shadow __user *u_mx))
{
	struct xnthread *cur = xnthread_current();
	struct cobalt_cond *cond;
	struct cobalt_mutex *mx;
	xnhandle_t handle;
	int err;

	handle = cobalt_get_handle_from_user(&u_cnd->handle);
	cond = xnregistry_lookup(handle, NULL);

	handle = cobalt_get_handle_from_user(&u_mx->handle);
	mx = xnregistry_lookup(handle, NULL);
	err = cobalt_cond_timedwait_epilogue(cur, cond, mx);

	if (cond->mutex == NULL)
		cond->state->mutex_state_offset = ~0U;

	return err;
}

int cobalt_cond_deferred_signals(struct cobalt_cond *cond)
{
	struct cobalt_cond_state *state;
	__u32 pending_signals;
	int need_resched;

	state = cond->state;
	pending_signals = state->pending_signals;

	switch(pending_signals) {
	default:
		state->pending_signals = 0;
		need_resched = xnsynch_wakeup_many_sleepers(&cond->synchbase,
							    pending_signals);
		break;

	case ~0U:
		need_resched =
			xnsynch_flush(&cond->synchbase, 0) == XNSYNCH_RESCHED;
		state->pending_signals = 0;
		break;

	case 0:
		need_resched = 0;
		break;
	}

	return need_resched;
}

void cobalt_cond_reclaim(struct cobalt_resnode *node, spl_t s)
{
	struct cobalt_cond *cond;

	cond = container_of(node, struct cobalt_cond, resnode);
	xnregistry_remove(node->handle);
	cobalt_del_resource(node);
	xnsynch_destroy(&cond->synchbase);
	cobalt_mark_deleted(cond);
	xnlock_put_irqrestore(&nklock, s);

	cobalt_umm_free(&cobalt_ppd_get(cond->attr.pshared)->umm,
			cond->state);
	xnfree(cond);
}
