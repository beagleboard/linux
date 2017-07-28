/*
 * Written by Gilles Chanteperdrix <gilles.chanteperdrix@xenomai.org>.
 * Copyright (C) 2014,2015 Philippe Gerum <rpm@xenomai.org>
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

#include <stddef.h>
#include <linux/err.h>
#include "internal.h"
#include "thread.h"
#include "clock.h"
#include "sem.h"
#include <trace/events/cobalt-posix.h>

static inline struct cobalt_resources *sem_kqueue(struct cobalt_sem *sem)
{
	int pshared = !!(sem->flags & SEM_PSHARED);
	return cobalt_current_resources(pshared);
}

static inline int sem_check(struct cobalt_sem *sem)
{
	if (sem == NULL || sem->magic != COBALT_SEM_MAGIC)
		return -EINVAL;

	if (IS_ENABLED(CONFIG_XENO_OPT_DEBUG_POSIX_SYNCHRO) &&
	    sem->resnode.scope && sem->resnode.scope != sem_kqueue(sem))
		return -EPERM;

	return 0;
}

int __cobalt_sem_destroy(xnhandle_t handle)
{
	struct cobalt_sem *sem;
	int ret = 0;
	spl_t s;

	xnlock_get_irqsave(&nklock, s);
	sem = xnregistry_lookup(handle, NULL);
	if (!cobalt_obj_active(sem, COBALT_SEM_MAGIC, typeof(*sem))) {
		ret = -EINVAL;
		goto fail;
	}

	if (--sem->refs) {
		ret = -EBUSY;
		goto fail;
	}

	cobalt_mark_deleted(sem);
	xnregistry_remove(sem->resnode.handle);
	if (!sem->pathname)
		cobalt_del_resource(&sem->resnode);
	if (xnsynch_destroy(&sem->synchbase) == XNSYNCH_RESCHED) {
		xnsched_run();
		ret = 1;
	}

	xnlock_put_irqrestore(&nklock, s);

	if (sem->pathname)
		putname(sem->pathname);

	cobalt_umm_free(&cobalt_ppd_get(!!(sem->flags & SEM_PSHARED))->umm,
			sem->state);

	xnfree(sem);

	return ret;
fail:
	xnlock_put_irqrestore(&nklock, s);

	return ret;
}

struct cobalt_sem *
__cobalt_sem_init(const char *name, struct cobalt_sem_shadow *sm,
		  int flags, unsigned int value)
{
	struct cobalt_sem_state *state;
	struct cobalt_sem *sem, *osem;
	struct cobalt_ppd *sys_ppd;
	int ret, sflags, pshared;
	struct list_head *semq;
	spl_t s;

	if ((flags & SEM_PULSE) != 0 && value > 0) {
		ret = -EINVAL;
		goto out;
	}

	sem = xnmalloc(sizeof(*sem));
	if (sem == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	pshared = !!(flags & SEM_PSHARED);
	sys_ppd = cobalt_ppd_get(pshared);
	state = cobalt_umm_alloc(&sys_ppd->umm, sizeof(*state));
	if (state == NULL) {
		ret = -EAGAIN;
		goto err_free_sem;
	}

	xnlock_get_irqsave(&nklock, s);

	semq = &cobalt_current_resources(pshared)->semq;
	if ((sm->magic == COBALT_SEM_MAGIC && !list_empty(semq)) ||
	    sm->magic == COBALT_NAMED_SEM_MAGIC) {
		osem = xnregistry_lookup(sm->handle, NULL);
		if (cobalt_obj_active(osem, COBALT_SEM_MAGIC, typeof(*osem))) {
			ret = -EBUSY;
			goto err_lock_put;
		}
	}

	if (value > (unsigned)SEM_VALUE_MAX) {
		ret = -EINVAL;
		goto err_lock_put;
	}

	ret = xnregistry_enter(name ?: "", sem, &sem->resnode.handle, NULL);
	if (ret < 0)
		goto err_lock_put;

	sem->magic = COBALT_SEM_MAGIC;
	if (!name)
		cobalt_add_resource(&sem->resnode, sem, pshared);
	else
		sem->resnode.scope = NULL;
	sflags = flags & SEM_FIFO ? 0 : XNSYNCH_PRIO;
	xnsynch_init(&sem->synchbase, sflags, NULL);

	sem->state = state;
	atomic_set(&state->value, value);
	state->flags = flags;
	sem->flags = flags;
	sem->refs = name ? 2 : 1;
	sem->pathname = NULL;

	sm->magic = name ? COBALT_NAMED_SEM_MAGIC : COBALT_SEM_MAGIC;
	sm->handle = sem->resnode.handle;
	sm->state_offset = cobalt_umm_offset(&sys_ppd->umm, state);
	if (flags & SEM_PSHARED)
		sm->state_offset = -sm->state_offset;
	xnlock_put_irqrestore(&nklock, s);

	trace_cobalt_psem_init(name ?: "anon",
			       sem->resnode.handle, flags, value);

	return sem;

err_lock_put:
	xnlock_put_irqrestore(&nklock, s);
	cobalt_umm_free(&sys_ppd->umm, state);
err_free_sem:
	xnfree(sem);
out:
	trace_cobalt_psem_init_failed(name ?: "anon", flags, value, ret);

	return ERR_PTR(ret);
}

static int sem_destroy(struct cobalt_sem_shadow *sm)
{
	struct cobalt_sem *sem;
	int warn, ret;
	spl_t s;

	xnlock_get_irqsave(&nklock, s);

	if (sm->magic != COBALT_SEM_MAGIC) {
		ret = -EINVAL;
		goto fail;
	}

	sem = xnregistry_lookup(sm->handle, NULL);
	ret = sem_check(sem);
	if (ret)
		goto fail;

	if ((sem->flags & SEM_NOBUSYDEL) != 0 &&
	    xnsynch_pended_p(&sem->synchbase)) {
		ret = -EBUSY;
		goto fail;
	}

	warn = sem->flags & SEM_WARNDEL;
	cobalt_mark_deleted(sm);

	xnlock_put_irqrestore(&nklock, s);

	ret = __cobalt_sem_destroy(sem->resnode.handle);

	return warn ? ret : 0;
fail:
	xnlock_put_irqrestore(&nklock, s);

	return ret;
}

static inline int do_trywait(struct cobalt_sem *sem)
{
	int ret;
	
	ret = sem_check(sem);
	if (ret)
		return ret;

	if (atomic_sub_return(1, &sem->state->value) < 0)
		return -EAGAIN;

	return 0;
}

static int sem_wait(xnhandle_t handle)
{
	struct cobalt_sem *sem;
	int ret, info;
	spl_t s;

	xnlock_get_irqsave(&nklock, s);

	sem = xnregistry_lookup(handle, NULL);
	ret = do_trywait(sem);
	if (ret != -EAGAIN)
		goto out;

	ret = 0;
	info = xnsynch_sleep_on(&sem->synchbase, XN_INFINITE, XN_RELATIVE);
	if (info & XNRMID) {
		ret = -EINVAL;
	} else if (info & XNBREAK) {
		atomic_inc(&sem->state->value); /* undo do_trywait() */
		ret = -EINTR;
	}
out:
	xnlock_put_irqrestore(&nklock, s);

	return ret;
}

static inline int sem_fetch_timeout(struct timespec *ts,
				    const void __user *u_ts)
{
	return u_ts == NULL ? -EFAULT :
		cobalt_copy_from_user(ts, u_ts, sizeof(*ts));
}

int __cobalt_sem_timedwait(struct cobalt_sem_shadow __user *u_sem,
			   const void __user *u_ts,
			   int (*fetch_timeout)(struct timespec *ts,
						const void __user *u_ts))
{
	struct timespec ts = { .tv_sec = 0, .tv_nsec = 0 };
	int pull_ts = 1, ret, info;
	struct cobalt_sem *sem;
	xnhandle_t handle;
	xntmode_t tmode;
	spl_t s;

	handle = cobalt_get_handle_from_user(&u_sem->handle);
	trace_cobalt_psem_timedwait(handle);

	xnlock_get_irqsave(&nklock, s);

	for (;;) {
		sem = xnregistry_lookup(handle, NULL);
		ret = do_trywait(sem);
		if (ret != -EAGAIN)
			break;

		/*
		 * POSIX states that the validity of the timeout spec
		 * _need_ not be checked if the semaphore can be
		 * locked immediately, we show this behavior despite
		 * it's actually more complex, to keep some
		 * applications ported to Linux happy.
		 */
		if (pull_ts) {
			atomic_inc(&sem->state->value);
			xnlock_put_irqrestore(&nklock, s);
			ret = fetch_timeout(&ts, u_ts);
			xnlock_get_irqsave(&nklock, s);
			if (ret)
				break;
			if (ts.tv_nsec >= ONE_BILLION) {
				ret = -EINVAL;
				break;
			}
			pull_ts = 0;
			continue;
		}

		ret = 0;
		tmode = sem->flags & SEM_RAWCLOCK ? XN_ABSOLUTE : XN_REALTIME;
		info = xnsynch_sleep_on(&sem->synchbase, ts2ns(&ts) + 1, tmode);
		if (info & XNRMID)
			ret = -EINVAL;
		else if (info & (XNBREAK|XNTIMEO)) {
			ret = (info & XNBREAK) ? -EINTR : -ETIMEDOUT;
			atomic_inc(&sem->state->value);
		}
		break;
	}

	xnlock_put_irqrestore(&nklock, s);

	return ret;
}

static int sem_post(xnhandle_t handle)
{
	struct cobalt_sem *sem;
	int ret;
	spl_t s;

	xnlock_get_irqsave(&nklock, s);

	sem = xnregistry_lookup(handle, NULL);
	ret = sem_check(sem);
	if (ret)
		goto out;

	if (atomic_read(&sem->state->value) == SEM_VALUE_MAX) {
		ret = -EINVAL;
		goto out;
	}

	if (atomic_inc_return(&sem->state->value) <= 0) {
		if (xnsynch_wakeup_one_sleeper(&sem->synchbase))
			xnsched_run();
	} else if (sem->flags & SEM_PULSE)
		atomic_set(&sem->state->value, 0);
out:	
	xnlock_put_irqrestore(&nklock, s);

	return ret;
}

static int sem_getvalue(xnhandle_t handle, int *value)
{
	struct cobalt_sem *sem;
	int ret;
	spl_t s;

	xnlock_get_irqsave(&nklock, s);

	sem = xnregistry_lookup(handle, NULL);
	ret = sem_check(sem);
	if (ret) {
		xnlock_put_irqrestore(&nklock, s);
		return ret;
	}

	*value = atomic_read(&sem->state->value);
	if ((sem->flags & SEM_REPORT) == 0 && *value < 0)
		*value = 0;

	xnlock_put_irqrestore(&nklock, s);

	return 0;
}

COBALT_SYSCALL(sem_init, current,
	       (struct cobalt_sem_shadow __user *u_sem,
		int flags, unsigned int value))
{
	struct cobalt_sem_shadow sm;
	struct cobalt_sem *sem;

	if (cobalt_copy_from_user(&sm, u_sem, sizeof(sm)))
		return -EFAULT;

	if (flags & ~(SEM_FIFO|SEM_PULSE|SEM_PSHARED|SEM_REPORT|\
		      SEM_WARNDEL|SEM_RAWCLOCK|SEM_NOBUSYDEL))
		return -EINVAL;

	sem = __cobalt_sem_init(NULL, &sm, flags, value);
	if (IS_ERR(sem))
		return PTR_ERR(sem);

	return cobalt_copy_to_user(u_sem, &sm, sizeof(*u_sem));
}

COBALT_SYSCALL(sem_post, current,
	       (struct cobalt_sem_shadow __user *u_sem))
{
	xnhandle_t handle;

	handle = cobalt_get_handle_from_user(&u_sem->handle);
	trace_cobalt_psem_post(handle);

	return sem_post(handle);
}

COBALT_SYSCALL(sem_wait, primary,
	       (struct cobalt_sem_shadow __user *u_sem))
{
	xnhandle_t handle;

	handle = cobalt_get_handle_from_user(&u_sem->handle);
	trace_cobalt_psem_wait(handle);

	return sem_wait(handle);
}

COBALT_SYSCALL(sem_timedwait, primary,
	       (struct cobalt_sem_shadow __user *u_sem,
		struct timespec __user *u_ts))
{
	return __cobalt_sem_timedwait(u_sem, u_ts, sem_fetch_timeout);
}

COBALT_SYSCALL(sem_trywait, primary,
	       (struct cobalt_sem_shadow __user *u_sem))
{
	struct cobalt_sem *sem;
	xnhandle_t handle;
	int ret;
	spl_t s;

	handle = cobalt_get_handle_from_user(&u_sem->handle);
	trace_cobalt_psem_trywait(handle);

	xnlock_get_irqsave(&nklock, s);
	sem = xnregistry_lookup(handle, NULL);
	ret = do_trywait(sem);
	xnlock_put_irqrestore(&nklock, s);

	return ret;
}

COBALT_SYSCALL(sem_getvalue, current,
	       (struct cobalt_sem_shadow __user *u_sem,
		int __user *u_sval))
{
	int ret, sval = -1;
	xnhandle_t handle;

	handle = cobalt_get_handle_from_user(&u_sem->handle);

	ret = sem_getvalue(handle, &sval);
	trace_cobalt_psem_getvalue(handle, sval);
	if (ret)
		return ret;

	return cobalt_copy_to_user(u_sval, &sval, sizeof(sval));
}

COBALT_SYSCALL(sem_destroy, current,
	       (struct cobalt_sem_shadow __user *u_sem))
{
	struct cobalt_sem_shadow sm;
	int err;

	if (cobalt_copy_from_user(&sm, u_sem, sizeof(sm)))
		return -EFAULT;

	trace_cobalt_psem_destroy(sm.handle);

	err = sem_destroy(&sm);
	if (err < 0)
		return err;

	return cobalt_copy_to_user(u_sem, &sm, sizeof(*u_sem)) ?: err;
}

COBALT_SYSCALL(sem_broadcast_np, current,
	       (struct cobalt_sem_shadow __user *u_sem))
{
	struct cobalt_sem *sem;
	xnhandle_t handle;
	spl_t s;
	int ret;

	handle = cobalt_get_handle_from_user(&u_sem->handle);
	trace_cobalt_psem_broadcast(u_sem->handle);

	xnlock_get_irqsave(&nklock, s);

	sem = xnregistry_lookup(handle, NULL);
	ret = sem_check(sem);
	if (ret == 0 && atomic_read(&sem->state->value) < 0) {
		atomic_set(&sem->state->value, 0);
		xnsynch_flush(&sem->synchbase, 0);
		xnsched_run();
	}

	xnlock_put_irqrestore(&nklock, s);

	return ret;
}

COBALT_SYSCALL(sem_inquire, current,
	       (struct cobalt_sem_shadow __user *u_sem,
		struct cobalt_sem_info __user *u_info,
		pid_t __user *u_waitlist,
		size_t waitsz))
{
	int val = 0, nrwait = 0, nrpids, ret = 0;
	unsigned long pstamp, nstamp = 0;
	struct cobalt_sem_info info;
	pid_t *t = NULL, fbuf[16];
	struct xnthread *thread;
	struct cobalt_sem *sem;
	xnhandle_t handle;
	spl_t s;

	handle = cobalt_get_handle_from_user(&u_sem->handle);
	trace_cobalt_psem_inquire(handle);

	nrpids = waitsz / sizeof(pid_t);

	xnlock_get_irqsave(&nklock, s);

	for (;;) {
		pstamp = nstamp;
		sem = xnregistry_lookup(handle, &nstamp);
		if (sem == NULL || sem->magic != COBALT_SEM_MAGIC) {
			xnlock_put_irqrestore(&nklock, s);
			return -EINVAL;
		}
		/*
		 * Allocate memory to return the wait list without
		 * holding any lock, then revalidate the handle.
		 */
		if (t == NULL) {
			val = atomic_read(&sem->state->value);
			if (val >= 0 || u_waitlist == NULL)
				break;
			xnlock_put_irqrestore(&nklock, s);
			if (nrpids > -val)
				nrpids = -val;
			if (-val <= ARRAY_SIZE(fbuf))
				t = fbuf; /* Use fast buffer. */
			else {
				t = xnmalloc(-val * sizeof(pid_t));
				if (t == NULL)
					return -ENOMEM;
			}
			xnlock_get_irqsave(&nklock, s);
		} else if (pstamp == nstamp)
			break;
		else if (val != atomic_read(&sem->state->value)) {
			xnlock_put_irqrestore(&nklock, s);
			if (t != fbuf)
				xnfree(t);
			t = NULL;
			xnlock_get_irqsave(&nklock, s);
		}
	}

	info.flags = sem->flags;
	info.value = (sem->flags & SEM_REPORT) || val >= 0 ? val : 0;
	info.nrwait = val < 0 ? -val : 0;

	if (xnsynch_pended_p(&sem->synchbase) && u_waitlist != NULL) {
		xnsynch_for_each_sleeper(thread, &sem->synchbase) {
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

void cobalt_sem_reclaim(struct cobalt_resnode *node, spl_t s)
{
	struct cobalt_sem *sem;
	xnhandle_t handle;
	int named, ret;

	sem = container_of(node, struct cobalt_sem, resnode);
	named = (sem->flags & SEM_NAMED) != 0;
	handle = node->handle;
	xnlock_put_irqrestore(&nklock, s);
	ret = __cobalt_sem_destroy(handle);
	if (named && ret == -EBUSY)
		xnregistry_unlink(xnregistry_key(handle));
}
