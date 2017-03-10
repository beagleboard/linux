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

#include <linux/types.h>
#include <linux/cred.h>
#include <linux/jhash.h>
#include <linux/signal.h>
#include <linux/jiffies.h>
#include <linux/err.h>
#include "internal.h"
#include "thread.h"
#include "sched.h"
#include "signal.h"
#include "timer.h"
#include "clock.h"
#include "sem.h"
#define CREATE_TRACE_POINTS
#include <trace/events/cobalt-posix.h>

xnticks_t cobalt_time_slice = CONFIG_XENO_OPT_RR_QUANTUM * 1000;

#define PTHREAD_HSLOTS (1 << 8)	/* Must be a power of 2 */

/* Process-local index, pthread_t x mm_struct (cobalt_local_hkey). */
struct local_thread_hash {
	pid_t pid;
	struct cobalt_thread *thread;
	struct cobalt_local_hkey hkey;
	struct local_thread_hash *next;
};

/* System-wide index on task_pid_nr(). */
struct global_thread_hash {
	pid_t pid;
	struct cobalt_thread *thread;
	struct global_thread_hash *next;
};

static struct local_thread_hash *local_index[PTHREAD_HSLOTS];

static struct global_thread_hash *global_index[PTHREAD_HSLOTS];

static inline struct local_thread_hash *
thread_hash(const struct cobalt_local_hkey *hkey,
	    struct cobalt_thread *thread, pid_t pid)
{
	struct global_thread_hash **ghead, *gslot;
	struct local_thread_hash **lhead, *lslot;
	u32 hash;
	void *p;
	spl_t s;

	p = xnmalloc(sizeof(*lslot) + sizeof(*gslot));
	if (p == NULL)
		return NULL;

	lslot = p;
	lslot->hkey = *hkey;
	lslot->thread = thread;
	lslot->pid = pid;
	hash = jhash2((u32 *)&lslot->hkey,
		      sizeof(lslot->hkey) / sizeof(u32), 0);
	lhead = &local_index[hash & (PTHREAD_HSLOTS - 1)];

	gslot = p + sizeof(*lslot);
	gslot->pid = pid;
	gslot->thread = thread;
	hash = jhash2((u32 *)&pid, sizeof(pid) / sizeof(u32), 0);
	ghead = &global_index[hash & (PTHREAD_HSLOTS - 1)];

	xnlock_get_irqsave(&nklock, s);
	lslot->next = *lhead;
	*lhead = lslot;
	gslot->next = *ghead;
	*ghead = gslot;
	xnlock_put_irqrestore(&nklock, s);

	return lslot;
}

static inline void thread_unhash(const struct cobalt_local_hkey *hkey)
{
	struct global_thread_hash **gtail, *gslot;
	struct local_thread_hash **ltail, *lslot;
	pid_t pid;
	u32 hash;
	spl_t s;

	hash = jhash2((u32 *) hkey, sizeof(*hkey) / sizeof(u32), 0);
	ltail = &local_index[hash & (PTHREAD_HSLOTS - 1)];

	xnlock_get_irqsave(&nklock, s);

	lslot = *ltail;
	while (lslot &&
	       (lslot->hkey.u_pth != hkey->u_pth ||
		lslot->hkey.mm != hkey->mm)) {
		ltail = &lslot->next;
		lslot = *ltail;
	}

	if (lslot == NULL) {
		xnlock_put_irqrestore(&nklock, s);
		return;
	}

	*ltail = lslot->next;
	pid = lslot->pid;
	hash = jhash2((u32 *)&pid, sizeof(pid) / sizeof(u32), 0);
	gtail = &global_index[hash & (PTHREAD_HSLOTS - 1)];
	gslot = *gtail;
	while (gslot && gslot->pid != pid) {
		gtail = &gslot->next;
		gslot = *gtail;
	}
	/* gslot must be found here. */
	XENO_BUG_ON(COBALT, !(gslot && gtail));
	*gtail = gslot->next;

	xnlock_put_irqrestore(&nklock, s);

	xnfree(lslot);
}

static struct cobalt_thread *
thread_lookup(const struct cobalt_local_hkey *hkey)
{
	struct local_thread_hash *lslot;
	struct cobalt_thread *thread;
	u32 hash;
	spl_t s;

	hash = jhash2((u32 *)hkey, sizeof(*hkey) / sizeof(u32), 0);
	lslot = local_index[hash & (PTHREAD_HSLOTS - 1)];

	xnlock_get_irqsave(&nklock, s);

	while (lslot != NULL &&
	       (lslot->hkey.u_pth != hkey->u_pth || lslot->hkey.mm != hkey->mm))
		lslot = lslot->next;

	thread = lslot ? lslot->thread : NULL;

	xnlock_put_irqrestore(&nklock, s);

	return thread;
}

struct cobalt_thread *cobalt_thread_find(pid_t pid) /* nklocked, IRQs off */
{
	struct global_thread_hash *gslot;
	u32 hash;

	hash = jhash2((u32 *)&pid, sizeof(pid) / sizeof(u32), 0);

	gslot = global_index[hash & (PTHREAD_HSLOTS - 1)];
	while (gslot && gslot->pid != pid)
		gslot = gslot->next;

	return gslot ? gslot->thread : NULL;
}
EXPORT_SYMBOL_GPL(cobalt_thread_find);

struct cobalt_thread *cobalt_thread_find_local(pid_t pid) /* nklocked, IRQs off */
{
	struct cobalt_thread *thread;

	thread = cobalt_thread_find(pid);
	if (thread == NULL || thread->hkey.mm != current->mm)
		return NULL;

	return thread;
}
EXPORT_SYMBOL_GPL(cobalt_thread_find_local);

struct cobalt_thread *cobalt_thread_lookup(unsigned long pth) /* nklocked, IRQs off */
{
	struct cobalt_local_hkey hkey;

	hkey.u_pth = pth;
	hkey.mm = current->mm;
	return thread_lookup(&hkey);
}
EXPORT_SYMBOL_GPL(cobalt_thread_lookup);

void cobalt_thread_map(struct xnthread *curr)
{
	struct cobalt_thread *thread;

	thread = container_of(curr, struct cobalt_thread, threadbase);
	thread->process = cobalt_current_process();
	XENO_BUG_ON(COBALT, thread->process == NULL);
}

struct xnthread_personality *cobalt_thread_exit(struct xnthread *curr)
{
	struct cobalt_thread *thread;
	spl_t s;

	thread = container_of(curr, struct cobalt_thread, threadbase);
	/*
	 * Unhash first, to prevent further access to the TCB from
	 * userland.
	 */
	thread_unhash(&thread->hkey);
	xnlock_get_irqsave(&nklock, s);
	cobalt_mark_deleted(thread);
	list_del(&thread->next);
	xnlock_put_irqrestore(&nklock, s);
	cobalt_signal_flush(thread);
	xnsynch_destroy(&thread->monitor_synch);
	xnsynch_destroy(&thread->sigwait);

	return NULL;
}

struct xnthread_personality *cobalt_thread_finalize(struct xnthread *zombie)
{
	struct cobalt_thread *thread;

	thread = container_of(zombie, struct cobalt_thread, threadbase);
	xnfree(thread);

	return NULL;
}

int __cobalt_thread_setschedparam_ex(struct cobalt_thread *thread, int policy,
				     const struct sched_param_ex *param_ex)
{
	struct xnsched_class *sched_class;
	union xnsched_policy_param param;
	xnticks_t tslice;
	int ret = 0;
	spl_t s;

	xnlock_get_irqsave(&nklock, s);

	if (!cobalt_obj_active(thread, COBALT_THREAD_MAGIC,
			       struct cobalt_thread)) {
		ret = -ESRCH;
		goto out;
	}

	tslice = thread->threadbase.rrperiod;
	sched_class = cobalt_sched_policy_param(&param, policy,
						param_ex, &tslice);
	if (sched_class == NULL) {
		ret = -EINVAL;
		goto out;
	}
	xnthread_set_slice(&thread->threadbase, tslice);
	if (cobalt_call_extension(thread_setsched, &thread->extref, ret,
				  sched_class, &param) && ret)
		goto out;
	ret = xnthread_set_schedparam(&thread->threadbase,
				      sched_class, &param);
	xnsched_run();
out:
	xnlock_put_irqrestore(&nklock, s);

	return ret;
}

int __cobalt_thread_getschedparam_ex(struct cobalt_thread *thread,
				     int *policy_r,
				     struct sched_param_ex *param_ex)
{
	struct xnsched_class *base_class;
	struct xnthread *base_thread;
	spl_t s;

	xnlock_get_irqsave(&nklock, s);

	if (!cobalt_obj_active(thread, COBALT_THREAD_MAGIC,
			       struct cobalt_thread)) {
		xnlock_put_irqrestore(&nklock, s);
		return -ESRCH;
	}

	base_thread = &thread->threadbase;
	base_class = base_thread->base_class;
	*policy_r = base_class->policy;

	param_ex->sched_priority = xnthread_base_priority(base_thread);
	if (param_ex->sched_priority == 0) /* SCHED_FIFO/SCHED_WEAK */
		*policy_r = SCHED_NORMAL;

	if (base_class == &xnsched_class_rt) {
		if (xnthread_test_state(base_thread, XNRRB)) {
			ns2ts(&param_ex->sched_rr_quantum, base_thread->rrperiod);
			*policy_r = SCHED_RR;
		}
		goto out;
	}

#ifdef CONFIG_XENO_OPT_SCHED_WEAK
	if (base_class == &xnsched_class_weak) {
		if (*policy_r != SCHED_WEAK)
			param_ex->sched_priority = -param_ex->sched_priority;
		goto out;
	}
#endif
#ifdef CONFIG_XENO_OPT_SCHED_SPORADIC
	if (base_class == &xnsched_class_sporadic) {
		param_ex->sched_ss_low_priority = base_thread->pss->param.low_prio;
		ns2ts(&param_ex->sched_ss_repl_period, base_thread->pss->param.repl_period);
		ns2ts(&param_ex->sched_ss_init_budget, base_thread->pss->param.init_budget);
		param_ex->sched_ss_max_repl = base_thread->pss->param.max_repl;
		goto out;
	}
#endif
#ifdef CONFIG_XENO_OPT_SCHED_TP
	if (base_class == &xnsched_class_tp) {
		param_ex->sched_tp_partition =
			base_thread->tps - base_thread->sched->tp.partitions;
		goto out;
	}
#endif
#ifdef CONFIG_XENO_OPT_SCHED_QUOTA
	if (base_class == &xnsched_class_quota) {
		param_ex->sched_quota_group = base_thread->quota->tgid;
		goto out;
	}
#endif

out:
	xnlock_put_irqrestore(&nklock, s);

	return 0;
}

static int pthread_create(struct cobalt_thread **thread_p,
			  int policy,
			  const struct sched_param_ex *param_ex,
			  struct task_struct *task)
{
	struct xnsched_class *sched_class;
	union xnsched_policy_param param;
	struct xnthread_init_attr iattr;
	struct cobalt_thread *thread;
	xnticks_t tslice;
	int ret, n;
	spl_t s;

	thread = xnmalloc(sizeof(*thread));
	if (thread == NULL)
		return -EAGAIN;

	tslice = cobalt_time_slice;
	sched_class = cobalt_sched_policy_param(&param, policy,
						param_ex, &tslice);
	if (sched_class == NULL) {
		xnfree(thread);
		return -EINVAL;
	}

	iattr.name = task->comm;
	iattr.flags = XNUSER|XNFPU;
	iattr.personality = &cobalt_personality;
	iattr.affinity = CPU_MASK_ALL;
	ret = xnthread_init(&thread->threadbase, &iattr, sched_class, &param);
	if (ret) {
		xnfree(thread);
		return ret;
	}

	thread->magic = COBALT_THREAD_MAGIC;
	xnsynch_init(&thread->monitor_synch, XNSYNCH_FIFO, NULL);

	xnsynch_init(&thread->sigwait, XNSYNCH_FIFO, NULL);
	sigemptyset(&thread->sigpending);
	for (n = 0; n < _NSIG; n++)
		INIT_LIST_HEAD(thread->sigqueues + n);

	xnthread_set_slice(&thread->threadbase, tslice);
	cobalt_set_extref(&thread->extref, NULL, NULL);

	/*
	 * We need an anonymous registry entry to obtain a handle for
	 * fast mutex locking.
	 */
	ret = xnthread_register(&thread->threadbase, "");
	if (ret) {
		xnsynch_destroy(&thread->monitor_synch);
		xnsynch_destroy(&thread->sigwait);
		xnfree(thread);
		return ret;
	}

	xnlock_get_irqsave(&nklock, s);
	list_add_tail(&thread->next, &cobalt_thread_list);
	xnlock_put_irqrestore(&nklock, s);

	thread->hkey.u_pth = 0;
	thread->hkey.mm = NULL;

	*thread_p = thread;

	return 0;
}

static void pthread_discard(struct cobalt_thread *thread)
{
	spl_t s;

	xnsynch_destroy(&thread->monitor_synch);
	xnsynch_destroy(&thread->sigwait);

	xnlock_get_irqsave(&nklock, s);
	list_del(&thread->next);
	xnlock_put_irqrestore(&nklock, s);
	__xnthread_discard(&thread->threadbase);
	xnfree(thread);
}

static inline int pthread_setmode_np(int clrmask, int setmask, int *mode_r)
{
	const int valid_flags = XNLOCK|XNWARN|XNTRAPLB;
	int old;

	/*
	 * The conforming mode bit is actually zero, since jumping to
	 * this code entailed switching to primary mode already.
	 */
	if ((clrmask & ~valid_flags) != 0 || (setmask & ~valid_flags) != 0)
		return -EINVAL;

	old = xnthread_set_mode(clrmask, setmask);
	if (mode_r)
		*mode_r = old;

	if ((clrmask & ~setmask) & XNLOCK)
		/* Reschedule if the scheduler has been unlocked. */
		xnsched_run();

	return 0;
}

int cobalt_thread_setschedparam_ex(unsigned long pth,
				   int policy,
				   const struct sched_param_ex *param_ex,
				   __u32 __user *u_winoff,
				   int __user *u_promoted)
{
	struct cobalt_local_hkey hkey;
	struct cobalt_thread *thread;
	int ret, promoted = 0;

	hkey.u_pth = pth;
	hkey.mm = current->mm;
	trace_cobalt_pthread_setschedparam(pth, policy, param_ex);

	thread = thread_lookup(&hkey);
	if (thread == NULL) {
		if (u_winoff == NULL)
			return -EINVAL;
			
		thread = cobalt_thread_shadow(current, &hkey, u_winoff);
		if (IS_ERR(thread))
			return PTR_ERR(thread);

		promoted = 1;
	}

	ret = __cobalt_thread_setschedparam_ex(thread, policy, param_ex);
	if (ret)
		return ret;

	return cobalt_copy_to_user(u_promoted, &promoted, sizeof(promoted));
}

COBALT_SYSCALL(thread_setschedparam_ex, conforming,
	       (unsigned long pth,
		int policy,
		const struct sched_param_ex __user *u_param,
		__u32 __user *u_winoff,
		int __user *u_promoted))
{
	struct sched_param_ex param_ex;

	if (cobalt_copy_from_user(&param_ex, u_param, sizeof(param_ex)))
		return -EFAULT;

	return cobalt_thread_setschedparam_ex(pth, policy, &param_ex,
					      u_winoff, u_promoted);
}

int cobalt_thread_getschedparam_ex(unsigned long pth,
				   int *policy_r,
				   struct sched_param_ex *param_ex)
{
	struct cobalt_local_hkey hkey;
	struct cobalt_thread *thread;
	int ret;

	hkey.u_pth = pth;
	hkey.mm = current->mm;
	thread = thread_lookup(&hkey);
	if (thread == NULL)
		return -ESRCH;

	ret = __cobalt_thread_getschedparam_ex(thread, policy_r, param_ex);
	if (ret)
		return ret;

	trace_cobalt_pthread_getschedparam(pth, *policy_r, param_ex);

	return 0;
}

COBALT_SYSCALL(thread_getschedparam_ex, current,
	       (unsigned long pth,
		int __user *u_policy,
		struct sched_param_ex __user *u_param))
{
	struct sched_param_ex param_ex;
	int ret, policy;

	ret = cobalt_thread_getschedparam_ex(pth, &policy, &param_ex);
	if (ret)
		return ret;

	ret = cobalt_copy_to_user(u_policy, &policy, sizeof(policy));
	if (ret)
		return ret;

	return cobalt_copy_to_user(u_param, &param_ex, sizeof(param_ex));
}

int __cobalt_thread_create(unsigned long pth, int policy,
			   struct sched_param_ex *param_ex,
			   int xid, __u32 __user *u_winoff)
{
	struct cobalt_thread *thread = NULL;
	struct task_struct *p = current;
	struct cobalt_local_hkey hkey;
	int ret;

	trace_cobalt_pthread_create(pth, policy, param_ex);

	/*
	 * We have been passed the pthread_t identifier the user-space
	 * Cobalt library has assigned to our caller; we'll index our
	 * internal pthread_t descriptor in kernel space on it.
	 */
	hkey.u_pth = pth;
	hkey.mm = p->mm;

	ret = pthread_create(&thread, policy, param_ex, p);
	if (ret)
		return ret;

	ret = cobalt_map_user(&thread->threadbase, u_winoff);
	if (ret) {
		pthread_discard(thread);
		return ret;
	}

	if (!thread_hash(&hkey, thread, task_pid_vnr(p))) {
		ret = -EAGAIN;
		goto fail;
	}

	thread->hkey = hkey;

	if (xid > 0 && cobalt_push_personality(xid) == NULL) {
		ret = -EINVAL;
		goto fail;
	}

	return xnthread_harden();
fail:
	xnthread_cancel(&thread->threadbase);

	return ret;
}

COBALT_SYSCALL(thread_create, init,
	       (unsigned long pth, int policy,
		struct sched_param_ex __user *u_param,
		int xid,
		__u32 __user *u_winoff))
{
	struct sched_param_ex param_ex;
	int ret;

	ret = cobalt_copy_from_user(&param_ex, u_param, sizeof(param_ex));
	if (ret)
		return ret;

	return __cobalt_thread_create(pth, policy, &param_ex, xid, u_winoff);
}

struct cobalt_thread *
cobalt_thread_shadow(struct task_struct *p,
		     struct cobalt_local_hkey *hkey,
		     __u32 __user *u_winoff)
{
	struct cobalt_thread *thread = NULL;
	struct sched_param_ex param_ex;
	int ret;

	param_ex.sched_priority = 0;
	trace_cobalt_pthread_create(hkey->u_pth, SCHED_NORMAL, &param_ex);
	ret = pthread_create(&thread, SCHED_NORMAL, &param_ex, p);
	if (ret)
		return ERR_PTR(ret);

	ret = cobalt_map_user(&thread->threadbase, u_winoff);
	if (ret) {
		pthread_discard(thread);
		return ERR_PTR(ret);
	}

	if (!thread_hash(hkey, thread, task_pid_vnr(p))) {
		ret = -EAGAIN;
		goto fail;
	}

	thread->hkey = *hkey;

	xnthread_harden();

	return thread;
fail:
	xnthread_cancel(&thread->threadbase);

	return ERR_PTR(ret);
}

COBALT_SYSCALL(thread_setmode, primary,
	       (int clrmask, int setmask, int __user *u_mode_r))
{
	int ret, old;

	trace_cobalt_pthread_setmode(clrmask, setmask);

	ret = pthread_setmode_np(clrmask, setmask, &old);
	if (ret)
		return ret;

	if (u_mode_r && cobalt_copy_to_user(u_mode_r, &old, sizeof(old)))
		return -EFAULT;

	return 0;
}

COBALT_SYSCALL(thread_setname, current,
	       (unsigned long pth, const char __user *u_name))
{
	struct cobalt_local_hkey hkey;
	struct cobalt_thread *thread;
	char name[XNOBJECT_NAME_LEN];
	struct task_struct *p;
	spl_t s;

	if (cobalt_strncpy_from_user(name, u_name,
				     sizeof(name) - 1) < 0)
		return -EFAULT;

	name[sizeof(name) - 1] = '\0';
	hkey.u_pth = pth;
	hkey.mm = current->mm;

	trace_cobalt_pthread_setname(pth, name);

	xnlock_get_irqsave(&nklock, s);

	thread = thread_lookup(&hkey);
	if (thread == NULL) {
		xnlock_put_irqrestore(&nklock, s);
		return -ESRCH;
	}

	ksformat(thread->threadbase.name,
		 XNOBJECT_NAME_LEN - 1, "%s", name);
	p = xnthread_host_task(&thread->threadbase);
	get_task_struct(p);

	xnlock_put_irqrestore(&nklock, s);

	knamecpy(p->comm, name);
	put_task_struct(p);

	return 0;
}

COBALT_SYSCALL(thread_kill, conforming,
	       (unsigned long pth, int sig))
{
	struct cobalt_local_hkey hkey;
	struct cobalt_thread *thread;
	int ret;
	spl_t s;

	trace_cobalt_pthread_kill(pth, sig);

	xnlock_get_irqsave(&nklock, s);

	hkey.u_pth = pth;
	hkey.mm = current->mm;
	thread = thread_lookup(&hkey);
	if (thread == NULL)
		ret = -ESRCH;
	else
		ret = __cobalt_kill(thread, sig, 0);

	xnlock_put_irqrestore(&nklock, s);

	return ret;
}

COBALT_SYSCALL(thread_join, primary, (unsigned long pth))
{
	struct cobalt_local_hkey hkey;
	struct cobalt_thread *thread;
	spl_t s;

	trace_cobalt_pthread_join(pth);

	xnlock_get_irqsave(&nklock, s);

	hkey.u_pth = pth;
	hkey.mm = current->mm;
	thread = thread_lookup(&hkey);

	xnlock_put_irqrestore(&nklock, s);

	if (thread == NULL)
		return -ESRCH;

	return xnthread_join(&thread->threadbase, false);
}

COBALT_SYSCALL(thread_getpid, current, (unsigned long pth))
{
	struct cobalt_local_hkey hkey;
	struct cobalt_thread *thread;
	pid_t pid;
	spl_t s;

	trace_cobalt_pthread_pid(pth);

	xnlock_get_irqsave(&nklock, s);

	hkey.u_pth = pth;
	hkey.mm = current->mm;
	thread = thread_lookup(&hkey);
	if (thread == NULL)
		pid = -ESRCH;
	else
		pid = xnthread_host_pid(&thread->threadbase);

	xnlock_put_irqrestore(&nklock, s);

	return pid;
}

COBALT_SYSCALL(thread_getstat, current,
	       (pid_t pid, struct cobalt_threadstat __user *u_stat))
{
	struct cobalt_threadstat stat;
	struct cobalt_thread *p;
	struct xnthread *thread;
	xnticks_t xtime;
	spl_t s;

	trace_cobalt_pthread_stat(pid);

	if (pid == 0) {
		thread = xnthread_current();
		if (thread == NULL)
			return -EPERM;
		xnlock_get_irqsave(&nklock, s);
	} else {
		xnlock_get_irqsave(&nklock, s);
		p = cobalt_thread_find(pid);
		if (p == NULL) {
			xnlock_put_irqrestore(&nklock, s);
			return -ESRCH;
		}
		thread = &p->threadbase;
	}

	/* We have to hold the nklock to keep most values consistent. */
	stat.cpu = xnsched_cpu(thread->sched);
	stat.cprio = xnthread_current_priority(thread);
	xtime = xnstat_exectime_get_total(&thread->stat.account);
	if (thread->sched->curr == thread)
		xtime += xnstat_exectime_now() -
			xnstat_exectime_get_last_switch(thread->sched);
	stat.xtime = xnclock_ticks_to_ns(&nkclock, xtime);
	stat.msw = xnstat_counter_get(&thread->stat.ssw);
	stat.csw = xnstat_counter_get(&thread->stat.csw);
	stat.xsc = xnstat_counter_get(&thread->stat.xsc);
	stat.pf = xnstat_counter_get(&thread->stat.pf);
	stat.status = xnthread_get_state(thread);
	if (thread->lock_count > 0)
		stat.status |= XNLOCK;
	stat.timeout = xnthread_get_timeout(thread,
					    xnclock_read_monotonic(&nkclock));
	strcpy(stat.name, thread->name);
	strcpy(stat.personality, thread->personality->name);
	xnlock_put_irqrestore(&nklock, s);

	return cobalt_copy_to_user(u_stat, &stat, sizeof(stat));
}

#ifdef CONFIG_XENO_OPT_COBALT_EXTENSION

int cobalt_thread_extend(struct cobalt_extension *ext,
			 void *priv)
{
	struct cobalt_thread *thread = cobalt_current_thread();
	struct xnthread_personality *prev;

	trace_cobalt_pthread_extend(thread->hkey.u_pth, ext->core.name);

	prev = cobalt_push_personality(ext->core.xid);
	if (prev == NULL)
		return -EINVAL;

	cobalt_set_extref(&thread->extref, ext, priv);

	return 0;
}
EXPORT_SYMBOL_GPL(cobalt_thread_extend);

void cobalt_thread_restrict(void)
{
	struct cobalt_thread *thread = cobalt_current_thread();

	trace_cobalt_pthread_restrict(thread->hkey.u_pth,
		      thread->threadbase.personality->name);
	cobalt_pop_personality(&cobalt_personality);
	cobalt_set_extref(&thread->extref, NULL, NULL);
}
EXPORT_SYMBOL_GPL(cobalt_thread_restrict);

#endif /* !CONFIG_XENO_OPT_COBALT_EXTENSION */
