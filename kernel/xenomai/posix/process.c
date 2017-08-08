/*
 * Copyright (C) 2001-2014 Philippe Gerum <rpm@xenomai.org>.
 * Copyright (C) 2001-2014 The Xenomai project <http://www.xenomai.org>
 * Copyright (C) 2006 Gilles Chanteperdrix <gilles.chanteperdrix@xenomai.org>
 *
 * SMP support Copyright (C) 2004 The HYADES project <http://www.hyades-itea.org>
 * RTAI/fusion Copyright (C) 2004 The RTAI project <http://www.rtai.org>
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
#include <linux/unistd.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/anon_inodes.h>
#include <linux/mman.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/cred.h>
#include <linux/file.h>
#include <linux/ptrace.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/kallsyms.h>
#include <linux/ipipe.h>
#include <linux/ipipe_tickdev.h>
#include <cobalt/kernel/sched.h>
#include <cobalt/kernel/heap.h>
#include <cobalt/kernel/synch.h>
#include <cobalt/kernel/clock.h>
#include <cobalt/kernel/ppd.h>
#include <cobalt/kernel/trace.h>
#include <cobalt/kernel/stat.h>
#include <cobalt/kernel/ppd.h>
#include <cobalt/kernel/vdso.h>
#include <cobalt/kernel/thread.h>
#include <cobalt/uapi/signal.h>
#include <cobalt/uapi/syscall.h>
#include <trace/events/cobalt-core.h>
#include <rtdm/driver.h>
#include <asm/xenomai/features.h>
#include <asm/xenomai/syscall.h>
#include <asm-generic/xenomai/mayday.h>
#include "../debug.h"
#include "internal.h"
#include "thread.h"
#include "sched.h"
#include "mutex.h"
#include "cond.h"
#include "mqueue.h"
#include "sem.h"
#include "signal.h"
#include "timer.h"
#include "monitor.h"
#include "clock.h"
#include "event.h"
#include "timerfd.h"
#include "io.h"

static int gid_arg = -1;
module_param_named(allowed_group, gid_arg, int, 0644);

static DEFINE_MUTEX(personality_lock);

static struct hlist_head *process_hash;
DEFINE_PRIVATE_XNLOCK(process_hash_lock);
#define PROCESS_HASH_SIZE 13

struct xnthread_personality *cobalt_personalities[NR_PERSONALITIES];

static struct xnsynch yield_sync;

LIST_HEAD(cobalt_thread_list);

struct cobalt_resources cobalt_global_resources = {
	.condq = LIST_HEAD_INIT(cobalt_global_resources.condq),
	.mutexq = LIST_HEAD_INIT(cobalt_global_resources.mutexq),
	.semq = LIST_HEAD_INIT(cobalt_global_resources.semq),
	.monitorq = LIST_HEAD_INIT(cobalt_global_resources.monitorq),
	.eventq = LIST_HEAD_INIT(cobalt_global_resources.eventq),
	.schedq = LIST_HEAD_INIT(cobalt_global_resources.schedq),
};

static unsigned __attribute__((pure)) process_hash_crunch(struct mm_struct *mm)
{
	unsigned long hash = ((unsigned long)mm - PAGE_OFFSET) / sizeof(*mm);
	return hash % PROCESS_HASH_SIZE;
}

static struct cobalt_process *__process_hash_search(struct mm_struct *mm)
{
	unsigned int bucket = process_hash_crunch(mm);
	struct cobalt_process *p;

	hlist_for_each_entry(p, &process_hash[bucket], hlink)
		if (p->mm == mm)
			return p;
	
	return NULL;
}

static int process_hash_enter(struct cobalt_process *p)
{
	struct mm_struct *mm = current->mm;
	unsigned int bucket = process_hash_crunch(mm);
	int err;
	spl_t s;

	xnlock_get_irqsave(&process_hash_lock, s);
	if (__process_hash_search(mm)) {
		err = -EBUSY;
		goto out;
	}

	p->mm = mm;
	hlist_add_head(&p->hlink, &process_hash[bucket]);
	err = 0;
  out:
	xnlock_put_irqrestore(&process_hash_lock, s);
	return err;
}

static void process_hash_remove(struct cobalt_process *p)
{
	spl_t s;

	xnlock_get_irqsave(&process_hash_lock, s);
	if (p->mm)
		hlist_del(&p->hlink);
	xnlock_put_irqrestore(&process_hash_lock, s);
}

struct cobalt_process *cobalt_search_process(struct mm_struct *mm)
{
	struct cobalt_process *process;
	spl_t s;
	
	xnlock_get_irqsave(&process_hash_lock, s);
	process = __process_hash_search(mm);
	xnlock_put_irqrestore(&process_hash_lock, s);
	
	return process;
}

static void *lookup_context(int xid)
{
	struct cobalt_process *process = cobalt_current_process();
	void *priv = NULL;
	spl_t s;

	xnlock_get_irqsave(&process_hash_lock, s);
	/*
	 * First try matching the process context attached to the
	 * (usually main) thread which issued sc_cobalt_bind. If not
	 * found, try matching by mm context, which should point us
	 * back to the latter. If none match, then the current process
	 * is unbound.
	 */
	if (process == NULL && current->mm)
		process = __process_hash_search(current->mm);
	if (process)
		priv = process->priv[xid];

	xnlock_put_irqrestore(&process_hash_lock, s);

	return priv;
}

static void remove_process(struct cobalt_process *process)
{
	struct xnthread_personality *personality;
	void *priv;
	int xid;

	mutex_lock(&personality_lock);

	for (xid = NR_PERSONALITIES - 1; xid >= 0; xid--) {
		if (!__test_and_clear_bit(xid, &process->permap))
			continue;
		personality = cobalt_personalities[xid];
		priv = process->priv[xid];
		if (priv == NULL)
			continue;
		/*
		 * CAUTION: process potentially refers to stale memory
		 * upon return from detach_process() for the Cobalt
		 * personality, so don't dereference it afterwards.
		 */
		if (xid)
			process->priv[xid] = NULL;
		__clear_bit(personality->xid, &process->permap);
		personality->ops.detach_process(priv);
		atomic_dec(&personality->refcnt);
		XENO_WARN_ON(COBALT, atomic_read(&personality->refcnt) < 0);
		if (personality->module)
			module_put(personality->module);
	}

	cobalt_set_process(NULL);

	mutex_unlock(&personality_lock);
}

static void post_ppd_release(struct cobalt_umm *umm)
{
	struct cobalt_process *process;

	process = container_of(umm, struct cobalt_process, sys_ppd.umm);
	kfree(process);
}

static inline char *get_exe_path(struct task_struct *p)
{
	struct file *exe_file;
	char *pathname, *buf;
	struct mm_struct *mm;
	struct path path;

	/*
	 * PATH_MAX is fairly large, and in any case won't fit on the
	 * caller's stack happily; since we are mapping a shadow,
	 * which is a heavyweight operation anyway, let's pick the
	 * memory from the page allocator.
	 */
	buf = (char *)__get_free_page(GFP_TEMPORARY);
	if (buf == NULL)
		return ERR_PTR(-ENOMEM);

	mm = get_task_mm(p);
	if (mm == NULL) {
		pathname = "vmlinux";
		goto copy;	/* kernel thread */
	}

	exe_file = get_mm_exe_file(mm);
	mmput(mm);
	if (exe_file == NULL) {
		pathname = ERR_PTR(-ENOENT);
		goto out;	/* no luck. */
	}

	path = exe_file->f_path;
	path_get(&exe_file->f_path);
	fput(exe_file);
	pathname = d_path(&path, buf, PATH_MAX);
	path_put(&path);
	if (IS_ERR(pathname))
		goto out;	/* mmmh... */
copy:
	/* caution: d_path() may start writing anywhere in the buffer. */
	pathname = kstrdup(pathname, GFP_KERNEL);
out:
	free_page((unsigned long)buf);

	return pathname;
}

static inline int raise_cap(int cap)
{
	struct cred *new;

	new = prepare_creds();
	if (new == NULL)
		return -ENOMEM;

	cap_raise(new->cap_effective, cap);

	return commit_creds(new);
}

static int bind_personality(struct xnthread_personality *personality)
{
	struct cobalt_process *process;
	void *priv;

	/*
	 * We also check capabilities for stacking a Cobalt extension,
	 * in case the process dropped the supervisor privileges after
	 * a successful initial binding to the Cobalt interface.
	 */
	if (!capable(CAP_SYS_NICE) &&
	    (gid_arg == -1 || !in_group_p(KGIDT_INIT(gid_arg))))
		return -EPERM;
	/*
	 * Protect from the same process binding to the same interface
	 * several times.
	 */
	priv = lookup_context(personality->xid);
	if (priv)
		return 0;

	priv = personality->ops.attach_process();
	if (IS_ERR(priv))
		return PTR_ERR(priv);

	process = cobalt_current_process();
	/*
	 * We are still covered by the personality_lock, so we may
	 * safely bump the module refcount after the attach handler
	 * has returned.
	 */
	if (personality->module && !try_module_get(personality->module)) {
		personality->ops.detach_process(priv);
		return -EAGAIN;
	}

	__set_bit(personality->xid, &process->permap);
	atomic_inc(&personality->refcnt);
	process->priv[personality->xid] = priv;

	raise_cap(CAP_SYS_NICE);
	raise_cap(CAP_IPC_LOCK);
	raise_cap(CAP_SYS_RAWIO);

	return 0;
}

int cobalt_bind_personality(unsigned int magic)
{
	struct xnthread_personality *personality;
	int xid, ret = -ESRCH;

	mutex_lock(&personality_lock);

	for (xid = 1; xid < NR_PERSONALITIES; xid++) {
		personality = cobalt_personalities[xid];
		if (personality && personality->magic == magic) {
			ret = bind_personality(personality);
			break;
		}
	}

	mutex_unlock(&personality_lock);

	return ret ?: xid;
}

int cobalt_bind_core(void)
{
	int ret;

	mutex_lock(&personality_lock);
	ret = bind_personality(&cobalt_personality);
	mutex_unlock(&personality_lock);

	return ret;
}

/**
 * @fn int cobalt_register_personality(struct xnthread_personality *personality)
 * @internal
 * @brief Register a new interface personality.
 *
 * - personality->ops.attach_process() is called when a user-space
 *   process binds to the personality, on behalf of one of its
 *   threads. The attach_process() handler may return:
 *
 *   . an opaque pointer, representing the context of the calling
 *   process for this personality;
 *
 *   . a NULL pointer, meaning that no per-process structure should be
 *   attached to this process for this personality;
 *
 *   . ERR_PTR(negative value) indicating an error, the binding
 *   process will then abort.
 *
 * - personality->ops.detach_process() is called on behalf of an
 *   exiting user-space process which has previously attached to the
 *   personality. This handler is passed a pointer to the per-process
 *   data received earlier from the ops->attach_process() handler.
 *
 * @return the personality (extension) identifier.
 *
 * @note cobalt_get_context() is NULL when ops.detach_process() is
 * invoked for the personality the caller detaches from.
 *
 * @coretags{secondary-only}
 */
int cobalt_register_personality(struct xnthread_personality *personality)
{
	int xid;

	mutex_lock(&personality_lock);

	for (xid = 0; xid < NR_PERSONALITIES; xid++) {
		if (cobalt_personalities[xid] == NULL) {
			personality->xid = xid;
			atomic_set(&personality->refcnt, 0);
			cobalt_personalities[xid] = personality;
			goto out;
		}
	}

	xid = -EAGAIN;
out:
	mutex_unlock(&personality_lock);

	return xid;
}
EXPORT_SYMBOL_GPL(cobalt_register_personality);

/*
 * @brief Unregister an interface personality.
 *
 * @coretags{secondary-only}
 */
int cobalt_unregister_personality(int xid)
{
	struct xnthread_personality *personality;
	int ret = 0;

	if (xid < 0 || xid >= NR_PERSONALITIES)
		return -EINVAL;

	mutex_lock(&personality_lock);

	personality = cobalt_personalities[xid];
	if (atomic_read(&personality->refcnt) > 0)
		ret = -EBUSY;
	else
		cobalt_personalities[xid] = NULL;

	mutex_unlock(&personality_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(cobalt_unregister_personality);

/**
 * Stack a new personality over Cobalt for the current thread.
 *
 * This service registers the current thread as a member of the
 * additional personality identified by @a xid. If the current thread
 * is already assigned this personality, the call returns successfully
 * with no effect.
 *
 * @param xid the identifier of the additional personality.
 *
 * @return A handle to the previous personality. The caller should
 * save this handle for unstacking @a xid when applicable via a call
 * to cobalt_pop_personality().
 *
 * @coretags{secondary-only}
 */
struct xnthread_personality *
cobalt_push_personality(int xid)
{
	struct ipipe_threadinfo *p = ipipe_current_threadinfo();
	struct xnthread_personality *prev, *next;
	struct xnthread *thread = p->thread;

	secondary_mode_only();

	mutex_lock(&personality_lock);

	if (xid < 0 || xid >= NR_PERSONALITIES ||
	    p->process == NULL || !test_bit(xid, &p->process->permap)) {
		mutex_unlock(&personality_lock);
		return NULL;
	}

	next = cobalt_personalities[xid];
	prev = thread->personality;
	if (next == prev) {
		mutex_unlock(&personality_lock);
		return prev;
	}

	thread->personality = next;
	mutex_unlock(&personality_lock);
	xnthread_run_handler(thread, map_thread);

	return prev;
}
EXPORT_SYMBOL_GPL(cobalt_push_personality);

/**
 * Pop the topmost personality from the current thread.
 *
 * This service pops the topmost personality off the current thread.
 *
 * @param prev the previous personality which was returned by the
 * latest call to cobalt_push_personality() for the current thread.
 *
 * @coretags{secondary-only}
 */
void cobalt_pop_personality(struct xnthread_personality *prev)
{
	struct ipipe_threadinfo *p = ipipe_current_threadinfo();
	struct xnthread *thread = p->thread;

	secondary_mode_only();
	thread->personality = prev;
}
EXPORT_SYMBOL_GPL(cobalt_pop_personality);

/**
 * Return the per-process data attached to the calling user process.
 *
 * This service returns the per-process data attached to the calling
 * user process for the personality whose xid is @a xid.
 *
 * The per-process data was obtained from the ->attach_process()
 * handler defined for the personality @a xid refers to.
 *
 * See cobalt_register_personality() documentation for information on
 * the way to attach a per-process data to a process.
 *
 * @param xid the personality identifier.
 *
 * @return the per-process data if the current context is a user-space
 * process; @return NULL otherwise. As a special case,
 * cobalt_get_context(0) returns the current Cobalt process
 * descriptor, which is strictly identical to calling
 * cobalt_current_process().
 *
 * @coretags{task-unrestricted}
 */
void *cobalt_get_context(int xid)
{
	return lookup_context(xid);
}
EXPORT_SYMBOL_GPL(cobalt_get_context);

int cobalt_yield(xnticks_t min, xnticks_t max)
{
	xnticks_t start;
	int ret;

	start = xnclock_read_monotonic(&nkclock);
	max += start;
	min += start;

	do {
		ret = xnsynch_sleep_on(&yield_sync, max, XN_ABSOLUTE);
		if (ret & XNBREAK)
			return -EINTR;
	} while (ret == 0 && xnclock_read_monotonic(&nkclock) < min);

	return 0;
}
EXPORT_SYMBOL_GPL(cobalt_yield);

static inline void init_uthread_info(struct xnthread *thread)
{
	struct ipipe_threadinfo *p;

	p = ipipe_current_threadinfo();
	p->thread = thread;
	p->process = cobalt_search_process(current->mm);
}

static inline void clear_threadinfo(void)
{
	struct ipipe_threadinfo *p = ipipe_current_threadinfo();
	p->thread = NULL;
	p->process = NULL;
}

#ifdef CONFIG_MMU

static inline int disable_ondemand_memory(void)
{
	struct task_struct *p = current;
	siginfo_t si;

	if ((p->mm->def_flags & VM_LOCKED) == 0) {
		memset(&si, 0, sizeof(si));
		si.si_signo = SIGDEBUG;
		si.si_code = SI_QUEUE;
		si.si_int = SIGDEBUG_NOMLOCK | sigdebug_marker;
		send_sig_info(SIGDEBUG, &si, p);
		return 0;
	}

	return __ipipe_disable_ondemand_mappings(p);
}

static inline int get_mayday_prot(void)
{
	return PROT_READ|PROT_EXEC;
}

#else /* !CONFIG_MMU */

static inline int disable_ondemand_memory(void)
{
	return 0;
}

static inline int get_mayday_prot(void)
{
	/*
	 * Until we stop backing /dev/mem with the mayday page, we
	 * can't ask for PROT_EXEC since the former does not define
	 * mmap capabilities, and default ones won't allow an
	 * executable mapping with MAP_SHARED. In the NOMMU case, this
	 * is (currently) not an issue.
	 */
	return PROT_READ;
}

#endif /* !CONFIG_MMU */

/**
 * @fn int cobalt_map_user(struct xnthread *thread, __u32 __user *u_winoff)
 * @internal
 * @brief Create a shadow thread context over a user task.
 *
 * This call maps a Xenomai thread to the current regular Linux task
 * running in userland.  The priority and scheduling class of the
 * underlying Linux task are not affected; it is assumed that the
 * interface library did set them appropriately before issuing the
 * shadow mapping request.
 *
 * @param thread The descriptor address of the new shadow thread to be
 * mapped to current. This descriptor must have been previously
 * initialized by a call to xnthread_init().
 *
 * @param u_winoff will receive the offset of the per-thread
 * "u_window" structure in the global heap associated to @a
 * thread. This structure reflects thread state information visible
 * from userland through a shared memory window.
 *
 * @return 0 is returned on success. Otherwise:
 *
 * - -EINVAL is returned if the thread control block does not bear the
 * XNUSER bit.
 *
 * - -EBUSY is returned if either the current Linux task or the
 * associated shadow thread is already involved in a shadow mapping.
 *
 * @coretags{secondary-only}
 */
int cobalt_map_user(struct xnthread *thread, __u32 __user *u_winoff)
{
	struct xnthread_user_window *u_window;
	struct xnthread_start_attr attr;
	struct cobalt_ppd *sys_ppd;
	struct cobalt_umm *umm;
	int ret;

	if (!xnthread_test_state(thread, XNUSER))
		return -EINVAL;

	if (xnthread_current() || xnthread_test_state(thread, XNMAPPED))
		return -EBUSY;

	if (!access_wok(u_winoff, sizeof(*u_winoff)))
		return -EFAULT;

	ret = disable_ondemand_memory();
	if (ret)
		return ret;

	umm = &cobalt_kernel_ppd.umm;
	u_window = cobalt_umm_alloc(umm, sizeof(*u_window));
	if (u_window == NULL)
		return -ENOMEM;

	thread->u_window = u_window;
	__xn_put_user(cobalt_umm_offset(umm, u_window), u_winoff);
	xnthread_pin_initial(thread);

	trace_cobalt_shadow_map(thread);

	/*
	 * CAUTION: we enable the pipeline notifier only when our
	 * shadow TCB is consistent, so that we won't trigger false
	 * positive in debug code from handle_schedule_event() and
	 * friends.
	 */
	xnthread_init_shadow_tcb(thread);
	xnthread_suspend(thread, XNRELAX, XN_INFINITE, XN_RELATIVE, NULL);
	init_uthread_info(thread);
	xnthread_set_state(thread, XNMAPPED);
	xndebug_shadow_init(thread);
	sys_ppd = cobalt_ppd_get(0);
	atomic_inc(&sys_ppd->refcnt);
	/*
	 * ->map_thread() handler is invoked after the TCB is fully
	 * built, and when we know for sure that current will go
	 * through our task-exit handler, because it has a shadow
	 * extension and I-pipe notifications will soon be enabled for
	 * it.
	 */
	xnthread_run_handler(thread, map_thread);
	ipipe_enable_notifier(current);

	attr.mode = 0;
	attr.entry = NULL;
	attr.cookie = NULL;
	ret = xnthread_start(thread, &attr);
	if (ret)
		return ret;

	xnthread_sync_window(thread);

	xntrace_pid(xnthread_host_pid(thread),
		    xnthread_current_priority(thread));

	return 0;
}

static inline int handle_exception(struct ipipe_trap_data *d)
{
	struct xnthread *thread;
	struct xnsched *sched;

	sched = xnsched_current();
	thread = sched->curr;

	if (xnthread_test_state(thread, XNROOT))
		return 0;

	trace_cobalt_thread_fault(thread, d);

	if (xnarch_fault_fpu_p(d)) {
#ifdef CONFIG_XENO_ARCH_FPU
		spl_t s;

		/* FPU exception received in primary mode. */
		splhigh(s);
		if (xnarch_handle_fpu_fault(sched->fpuholder, thread, d)) {
			sched->fpuholder = thread;
			splexit(s);
			return 1;
		}
		splexit(s);
#endif /* CONFIG_XENO_ARCH_FPU */
		print_symbol("invalid use of FPU in Xenomai context at %s\n",
			     xnarch_fault_pc(d));
	}

	/*
	 * If we experienced a trap on behalf of a shadow thread
	 * running in primary mode, move it to the Linux domain,
	 * leaving the kernel process the exception.
	 */
#if XENO_DEBUG(COBALT) || XENO_DEBUG(USER)
	if (!user_mode(d->regs)) {
		xntrace_panic_freeze();
		printk(XENO_WARNING
		       "switching %s to secondary mode after exception #%u in "
		       "kernel-space at 0x%lx (pid %d)\n", thread->name,
		       xnarch_fault_trap(d),
		       xnarch_fault_pc(d),
		       xnthread_host_pid(thread));
		xntrace_panic_dump();
	} else if (xnarch_fault_notify(d)) /* Don't report debug traps */
		printk(XENO_WARNING
		       "switching %s to secondary mode after exception #%u from "
		       "user-space at 0x%lx (pid %d)\n", thread->name,
		       xnarch_fault_trap(d),
		       xnarch_fault_pc(d),
		       xnthread_host_pid(thread));
#endif

	if (xnarch_fault_pf_p(d))
		/*
		 * The page fault counter is not SMP-safe, but it's a
		 * simple indicator that something went wrong wrt
		 * memory locking anyway.
		 */
		xnstat_counter_inc(&thread->stat.pf);

	xnthread_relax(xnarch_fault_notify(d), SIGDEBUG_MIGRATE_FAULT);

	return 0;
}

static int handle_mayday_event(struct pt_regs *regs)
{
	struct xnthread *thread = xnthread_current();
	struct xnarchtcb *tcb = xnthread_archtcb(thread);
	struct cobalt_ppd *sys_ppd;

	XENO_BUG_ON(COBALT, !xnthread_test_state(thread, XNUSER));

	/* We enter the mayday handler with hw IRQs off. */
	sys_ppd = cobalt_ppd_get(0);

	xnarch_handle_mayday(tcb, regs, sys_ppd->mayday_tramp);

	return KEVENT_PROPAGATE;
}

int ipipe_trap_hook(struct ipipe_trap_data *data)
{
	if (data->exception == IPIPE_TRAP_MAYDAY)
		return handle_mayday_event(data->regs);

	/*
	 * No migration is possible on behalf of the head domain, so
	 * the following access is safe.
	 */
	raw_cpu_ptr(&cobalt_machine_cpudata)->faults[data->exception]++;

	if (handle_exception(data))
		return KEVENT_STOP;

	/*
	 * CAUTION: access faults must be propagated downstream
	 * whichever domain caused them, so that we don't spuriously
	 * raise a fatal error when some Linux fixup code is available
	 * to recover from the fault.
	 */
	return KEVENT_PROPAGATE;
}

#ifdef CONFIG_SMP

static int handle_setaffinity_event(struct ipipe_cpu_migration_data *d)
{
	struct task_struct *p = d->task;
	struct xnthread *thread;
	struct xnsched *sched;
	spl_t s;

	thread = xnthread_from_task(p);
	if (thread == NULL)
		return KEVENT_PROPAGATE;

	/*
	 * The CPU affinity mask is always controlled from secondary
	 * mode, therefore we progagate any change to the real-time
	 * affinity mask accordingly.
	 */
	xnlock_get_irqsave(&nklock, s);
	cpumask_and(&thread->affinity, &p->cpus_allowed, &cobalt_cpu_affinity);
	xnthread_run_handler_stack(thread, move_thread, d->dest_cpu);
	xnlock_put_irqrestore(&nklock, s);

	/*
	 * If kernel and real-time CPU affinity sets are disjoints,
	 * there might be problems ahead for this thread next time it
	 * moves back to primary mode, if it ends up switching to an
	 * unsupported CPU.
	 *
	 * Otherwise, check_affinity() will extend the CPU affinity if
	 * possible, fixing up the thread's affinity mask. This means
	 * that a thread might be allowed to run with a broken
	 * (i.e. fully cleared) affinity mask until it leaves primary
	 * mode then switches back to it, in SMP configurations.
	 */
	if (cpumask_empty(&thread->affinity))
		printk(XENO_WARNING "thread %s[%d] changed CPU affinity inconsistently\n",
		       thread->name, xnthread_host_pid(thread));
	else {
		xnlock_get_irqsave(&nklock, s);
		/*
		 * Threads running in primary mode may NOT be forcibly
		 * migrated by the regular kernel to another CPU. Such
		 * migration would have to wait until the thread
		 * switches back from secondary mode at some point
		 * later, or issues a call to xnthread_migrate().
		 */
		if (!xnthread_test_state(thread, XNMIGRATE) &&
		    xnthread_test_state(thread, XNTHREAD_BLOCK_BITS)) {
			sched = xnsched_struct(d->dest_cpu);
			xnthread_migrate_passive(thread, sched);
		}
		xnlock_put_irqrestore(&nklock, s);
	}

	return KEVENT_PROPAGATE;
}

static inline void check_affinity(struct task_struct *p) /* nklocked, IRQs off */
{
	struct xnthread *thread = xnthread_from_task(p);
	struct xnsched *sched;
	int cpu = task_cpu(p);

	/*
	 * If the task moved to another CPU while in secondary mode,
	 * migrate the companion Xenomai shadow to reflect the new
	 * situation.
	 *
	 * In the weirdest case, the thread is about to switch to
	 * primary mode on a CPU Xenomai shall not use. This is
	 * hopeless, whine and kill that thread asap.
	 */
	if (!xnsched_supported_cpu(cpu)) {
		printk(XENO_WARNING "thread %s[%d] switched to non-rt CPU%d, aborted.\n",
		       thread->name, xnthread_host_pid(thread), cpu);
		/*
		 * Can't call xnthread_cancel() from a migration
		 * point, that would break. Since we are on the wakeup
		 * path to hardening, just raise XNCANCELD to catch it
		 * in xnthread_harden().
		 */
		xnthread_set_info(thread, XNCANCELD);
		return;
	}

	sched = xnsched_struct(cpu);
	if (sched == thread->sched)
		return;

	/*
	 * The current thread moved to a supported real-time CPU,
	 * which is not part of its original affinity mask
	 * though. Assume user wants to extend this mask.
	 */
	if (!cpumask_test_cpu(cpu, &thread->affinity))
		cpumask_set_cpu(cpu, &thread->affinity);

	xnthread_migrate_passive(thread, sched);
}

#else /* !CONFIG_SMP */

struct ipipe_cpu_migration_data;

static int handle_setaffinity_event(struct ipipe_cpu_migration_data *d)
{
	return KEVENT_PROPAGATE;
}

static inline void check_affinity(struct task_struct *p) { }

#endif /* CONFIG_SMP */

void ipipe_migration_hook(struct task_struct *p) /* hw IRQs off */
{
	struct xnthread *thread = xnthread_from_task(p);

	/*
	 * We fire the handler before the thread is migrated, so that
	 * thread->sched does not change between paired invocations of
	 * relax_thread/harden_thread handlers.
	 */
	xnlock_get(&nklock);
	xnthread_run_handler_stack(thread, harden_thread);
	check_affinity(p);
	xnthread_resume(thread, XNRELAX);
	xnlock_put(&nklock);

	xnsched_run();
}

#ifdef CONFIG_XENO_OPT_HOSTRT

static IPIPE_DEFINE_SPINLOCK(__hostrtlock);

static int handle_hostrt_event(struct ipipe_hostrt_data *hostrt)
{
	unsigned long flags;
	urwstate_t tmp;

	/*
	 * The locking strategy is twofold:
	 * - The spinlock protects against concurrent updates from within the
	 *   Linux kernel and against preemption by Xenomai
	 * - The unsynced R/W block is for lockless read-only access.
	 */
	raw_spin_lock_irqsave(&__hostrtlock, flags);

	unsynced_write_block(&tmp, &nkvdso->hostrt_data.lock) {
		nkvdso->hostrt_data.live = 1;
		nkvdso->hostrt_data.cycle_last = hostrt->cycle_last;
		nkvdso->hostrt_data.mask = hostrt->mask;
		nkvdso->hostrt_data.mult = hostrt->mult;
		nkvdso->hostrt_data.shift = hostrt->shift;
		nkvdso->hostrt_data.wall_sec = hostrt->wall_time_sec;
		nkvdso->hostrt_data.wall_nsec = hostrt->wall_time_nsec;
		nkvdso->hostrt_data.wtom_sec = hostrt->wall_to_monotonic.tv_sec;
		nkvdso->hostrt_data.wtom_nsec = hostrt->wall_to_monotonic.tv_nsec;
	}

	raw_spin_unlock_irqrestore(&__hostrtlock, flags);

	return KEVENT_PROPAGATE;
}

static inline void init_hostrt(void)
{
	unsynced_rw_init(&nkvdso->hostrt_data.lock);
	nkvdso->hostrt_data.live = 0;
}

#else /* !CONFIG_XENO_OPT_HOSTRT */

struct ipipe_hostrt_data;

static inline int handle_hostrt_event(struct ipipe_hostrt_data *hostrt)
{
	return KEVENT_PROPAGATE;
}

static inline void init_hostrt(void) { }

#endif /* !CONFIG_XENO_OPT_HOSTRT */

/* called with nklock held */
static void register_debugged_thread(struct xnthread *thread)
{
	nkclock_lock++;

	xnthread_set_state(thread, XNSSTEP);
}

static void unregister_debugged_thread(struct xnthread *thread)
{
	spl_t s;

	xnlock_get_irqsave(&nklock, s);

	xnthread_clear_state(thread, XNSSTEP);

	XENO_BUG_ON(COBALT, nkclock_lock == 0);
	nkclock_lock--;

	xnlock_put_irqrestore(&nklock, s);
}

static void __handle_taskexit_event(struct task_struct *p)
{
	struct cobalt_ppd *sys_ppd;
	struct xnthread *thread;

	/*
	 * We are called for both kernel and user shadows over the
	 * root thread.
	 */
	secondary_mode_only();

	thread = xnthread_current();
	XENO_BUG_ON(COBALT, thread == NULL);
	trace_cobalt_shadow_unmap(thread);

	if (xnthread_test_state(thread, XNSSTEP))
		unregister_debugged_thread(thread);

	xnthread_run_handler_stack(thread, exit_thread);
	xnsched_run();

	if (xnthread_test_state(thread, XNUSER)) {
		cobalt_umm_free(&cobalt_kernel_ppd.umm, thread->u_window);
		thread->u_window = NULL;
		sys_ppd = cobalt_ppd_get(0);
		if (atomic_dec_and_test(&sys_ppd->refcnt))
			remove_process(cobalt_current_process());
	}
}

static int handle_taskexit_event(struct task_struct *p) /* p == current */
{
	__handle_taskexit_event(p);

	/*
	 * __xnthread_cleanup() -> ... -> finalize_thread
	 * handler. From that point, the TCB is dropped. Be careful of
	 * not treading on stale memory within @thread.
	 */
	__xnthread_cleanup(xnthread_current());

	clear_threadinfo();

	return KEVENT_PROPAGATE;
}

static inline void signal_yield(void)
{
	spl_t s;

	if (!xnsynch_pended_p(&yield_sync))
		return;

	xnlock_get_irqsave(&nklock, s);
	if (xnsynch_pended_p(&yield_sync)) {
		xnsynch_flush(&yield_sync, 0);
		xnsched_run();
	}
	xnlock_put_irqrestore(&nklock, s);
}

static int handle_schedule_event(struct task_struct *next_task)
{
	struct task_struct *prev_task;
	struct xnthread *next;
	sigset_t pending;

	signal_yield();

	prev_task = current;
	next = xnthread_from_task(next_task);
	if (next == NULL)
		goto out;

	/*
	 * Check whether we need to unlock the timers, each time a
	 * Linux task resumes from a stopped state, excluding tasks
	 * resuming shortly for entering a stopped state asap due to
	 * ptracing. To identify the latter, we need to check for
	 * SIGSTOP and SIGINT in order to encompass both the NPTL and
	 * LinuxThreads behaviours.
	 */
	if (xnthread_test_state(next, XNSSTEP)) {
		if (signal_pending(next_task)) {
			/*
			 * Do not grab the sighand lock here: it's
			 * useless, and we already own the runqueue
			 * lock, so this would expose us to deadlock
			 * situations on SMP.
			 */
			sigorsets(&pending,
				  &next_task->pending.signal,
				  &next_task->signal->shared_pending.signal);
			if (sigismember(&pending, SIGSTOP) ||
			    sigismember(&pending, SIGINT))
				goto no_ptrace;
		}
		unregister_debugged_thread(next);
	}

no_ptrace:
	/*
	 * Do basic sanity checks on the incoming thread state.
	 * NOTE: we allow ptraced threads to run shortly in order to
	 * properly recover from a stopped state.
	 */
	if (!XENO_WARN(COBALT, !xnthread_test_state(next, XNRELAX),
		       "hardened thread %s[%d] running in Linux domain?! "
		       "(status=0x%x, sig=%d, prev=%s[%d])",
		       next->name, task_pid_nr(next_task),
		       xnthread_get_state(next),
		       signal_pending(next_task),
		       prev_task->comm, task_pid_nr(prev_task)))
		XENO_WARN(COBALT,
			  !(next_task->ptrace & PT_PTRACED) &&
			   !xnthread_test_state(next, XNDORMANT)
			  && xnthread_test_state(next, XNPEND),
			  "blocked thread %s[%d] rescheduled?! "
			  "(status=0x%x, sig=%d, prev=%s[%d])",
			  next->name, task_pid_nr(next_task),
			  xnthread_get_state(next),
			  signal_pending(next_task), prev_task->comm,
			  task_pid_nr(prev_task));
out:
	return KEVENT_PROPAGATE;
}

static int handle_sigwake_event(struct task_struct *p)
{
	struct xnthread *thread;
	sigset_t pending;
	spl_t s;

	thread = xnthread_from_task(p);
	if (thread == NULL)
		return KEVENT_PROPAGATE;

	xnlock_get_irqsave(&nklock, s);

	/*
	 * CAUTION: __TASK_TRACED is not set in p->state yet. This
	 * state bit will be set right after we return, when the task
	 * is woken up.
	 */
	if ((p->ptrace & PT_PTRACED) && !xnthread_test_state(thread, XNSSTEP)) {
		/* We already own the siglock. */
		sigorsets(&pending,
			  &p->pending.signal,
			  &p->signal->shared_pending.signal);

		if (sigismember(&pending, SIGTRAP) ||
		    sigismember(&pending, SIGSTOP)
		    || sigismember(&pending, SIGINT))
			register_debugged_thread(thread);
	}

	if (xnthread_test_state(thread, XNRELAX)) {
		xnlock_put_irqrestore(&nklock, s);
		return KEVENT_PROPAGATE;
	}

	/*
	 * If kicking a shadow thread in primary mode, make sure Linux
	 * won't schedule in its mate under our feet as a result of
	 * running signal_wake_up(). The Xenomai scheduler must remain
	 * in control for now, until we explicitly relax the shadow
	 * thread to allow for processing the pending signals. Make
	 * sure we keep the additional state flags unmodified so that
	 * we don't break any undergoing ptrace.
	 */
	if (p->state & (TASK_INTERRUPTIBLE|TASK_UNINTERRUPTIBLE))
		cobalt_set_task_state(p, p->state | TASK_NOWAKEUP);

	__xnthread_kick(thread);

	xnsched_run();

	xnlock_put_irqrestore(&nklock, s);

	return KEVENT_PROPAGATE;
}

static int handle_cleanup_event(struct mm_struct *mm)
{
	struct cobalt_process *old, *process;
	struct cobalt_ppd *sys_ppd;
	struct xnthread *thread;

	/*
	 * We are NOT called for exiting kernel shadows.
	 * cobalt_current_process() is cleared if we get there after
	 * handle_task_exit(), so we need to restore this context
	 * pointer temporarily.
	 */
	process = cobalt_search_process(mm);
	old = cobalt_set_process(process);
	sys_ppd = cobalt_ppd_get(0);
	if (sys_ppd != &cobalt_kernel_ppd) {
		bool running_exec;

		/*
		 * Detect a userland shadow running exec(), i.e. still
		 * attached to the current linux task (no prior
		 * clear_threadinfo). In this case, we emulate a task
		 * exit, since the Xenomai binding shall not survive
		 * the exec() syscall. Since the process will keep on
		 * running though, we have to disable the event
		 * notifier manually for it.
		 */
		thread = xnthread_current();
		running_exec = thread && (current->flags & PF_EXITING) == 0;
		if (running_exec) {
			__handle_taskexit_event(current);
			ipipe_disable_notifier(current);
		}
		if (atomic_dec_and_test(&sys_ppd->refcnt))
			remove_process(process);
		if (running_exec) {
			__xnthread_cleanup(thread);
			clear_threadinfo();
		}
	}

	/*
	 * CAUTION: Do not override a state change caused by
	 * remove_process().
	 */
	if (cobalt_current_process() == process)
		cobalt_set_process(old);

	return KEVENT_PROPAGATE;
}

static inline int handle_clockfreq_event(unsigned int *p)
{
	unsigned int newfreq = *p;

	xnclock_update_freq(newfreq);

	return KEVENT_PROPAGATE;
}

int ipipe_kevent_hook(int kevent, void *data)
{
	int ret;

	switch (kevent) {
	case IPIPE_KEVT_SCHEDULE:
		ret = handle_schedule_event(data);
		break;
	case IPIPE_KEVT_SIGWAKE:
		ret = handle_sigwake_event(data);
		break;
	case IPIPE_KEVT_EXIT:
		ret = handle_taskexit_event(data);
		break;
	case IPIPE_KEVT_CLEANUP:
		ret = handle_cleanup_event(data);
		break;
	case IPIPE_KEVT_HOSTRT:
		ret = handle_hostrt_event(data);
		break;
	case IPIPE_KEVT_SETAFFINITY:
		ret = handle_setaffinity_event(data);
		break;
#ifdef IPIPE_KEVT_CLOCKFREQ
	case IPIPE_KEVT_CLOCKFREQ:
		ret = handle_clockfreq_event(data);
		break;
#endif
	default:
		ret = KEVENT_PROPAGATE;
	}

	return ret;
}

static inline unsigned long map_mayday_page(void)
{
	void __user *u_addr = NULL;
	void *mayday_page;
	int ret;

	mayday_page = xnarch_get_mayday_page();
	ret = rtdm_mmap_to_user(NULL, mayday_page, PAGE_SIZE,
				get_mayday_prot(), &u_addr, NULL, NULL);
	if (ret)
		return 0UL;

	return (unsigned long)u_addr;
}

static int attach_process(struct cobalt_process *process)
{
	struct cobalt_ppd *p = &process->sys_ppd;
	char *exe_path;
	int ret;

	ret = cobalt_umm_init(&p->umm, CONFIG_XENO_OPT_PRIVATE_HEAPSZ * 1024,
			      post_ppd_release);
	if (ret)
		return ret;

	cobalt_umm_set_name(&p->umm, "private heap[%d]", task_pid_nr(current));

	p->mayday_tramp = map_mayday_page();
	if (p->mayday_tramp == 0) {
		printk(XENO_WARNING
		       "%s[%d] cannot map MAYDAY page\n",
		       current->comm, task_pid_nr(current));
		ret = -ENOMEM;
		goto fail_mayday;
	}

	exe_path = get_exe_path(current);
	if (IS_ERR(exe_path)) {
		printk(XENO_WARNING
		       "%s[%d] can't find exe path\n",
		       current->comm, task_pid_nr(current));
		exe_path = NULL; /* Not lethal, but weird. */
	}
	p->exe_path = exe_path;
	xntree_init(&p->fds);
	atomic_set(&p->refcnt, 1);

	ret = process_hash_enter(process);
	if (ret)
		goto fail_hash;

	return 0;
fail_hash:
	if (p->exe_path)
		kfree(p->exe_path);
fail_mayday:
	cobalt_umm_destroy(&p->umm);

	return ret;
}

static void *cobalt_process_attach(void)
{
	struct cobalt_process *process;
	int ret;

	process = kzalloc(sizeof(*process), GFP_KERNEL);
	if (process == NULL)
		return ERR_PTR(-ENOMEM);

	ret = attach_process(process);
	if (ret) {
		kfree(process);
		return ERR_PTR(ret);
	}

	INIT_LIST_HEAD(&process->resources.condq);
	INIT_LIST_HEAD(&process->resources.mutexq);
	INIT_LIST_HEAD(&process->resources.semq);
	INIT_LIST_HEAD(&process->resources.monitorq);
	INIT_LIST_HEAD(&process->resources.eventq);
	INIT_LIST_HEAD(&process->resources.schedq);
	INIT_LIST_HEAD(&process->sigwaiters);
	xntree_init(&process->usems);
	bitmap_fill(process->timers_map, CONFIG_XENO_OPT_NRTIMERS);
	cobalt_set_process(process);

	return process;
}

static void detach_process(struct cobalt_process *process)
{
	struct cobalt_ppd *p = &process->sys_ppd;

	if (p->exe_path)
		kfree(p->exe_path);

	rtdm_fd_cleanup(p);
	process_hash_remove(process);
	/*
	 * CAUTION: the process descriptor might be immediately
	 * released as a result of calling cobalt_umm_destroy(), so we
	 * must do this last, not to tread on stale memory.
	 */
	cobalt_umm_destroy(&p->umm);
}

static void __reclaim_resource(struct cobalt_process *process,
			       void (*reclaim)(struct cobalt_resnode *node, spl_t s),
			       struct list_head *local,
			       struct list_head *global)
{
	struct cobalt_resnode *node, *tmp;
	LIST_HEAD(stash);
	spl_t s;

	xnlock_get_irqsave(&nklock, s);

	if (list_empty(global))
		goto flush_local;

	list_for_each_entry_safe(node, tmp, global, next) {
		if (node->owner == process) {
			list_del(&node->next);
			list_add(&node->next, &stash);
		}
	}
		
	list_for_each_entry_safe(node, tmp, &stash, next) {
		reclaim(node, s);
		xnlock_get_irqsave(&nklock, s);
	}

	XENO_BUG_ON(COBALT, !list_empty(&stash));

flush_local:
	if (list_empty(local))
		goto out;

	list_for_each_entry_safe(node, tmp, local, next) {
		reclaim(node, s);
		xnlock_get_irqsave(&nklock, s);
	}
out:
	xnsched_run();
	xnlock_put_irqrestore(&nklock, s);
}

#define cobalt_reclaim_resource(__process, __reclaim, __type)		\
	__reclaim_resource(__process, __reclaim,			\
			   &(__process)->resources.__type ## q,		\
			   &cobalt_global_resources.__type ## q)

static void cobalt_process_detach(void *arg)
{
	struct cobalt_process *process = arg;

	cobalt_nsem_reclaim(process);
 	cobalt_timer_reclaim(process);
 	cobalt_sched_reclaim(process);
	cobalt_reclaim_resource(process, cobalt_cond_reclaim, cond);
	cobalt_reclaim_resource(process, cobalt_mutex_reclaim, mutex);
	cobalt_reclaim_resource(process, cobalt_event_reclaim, event);
	cobalt_reclaim_resource(process, cobalt_monitor_reclaim, monitor);
	cobalt_reclaim_resource(process, cobalt_sem_reclaim, sem);
 	detach_process(process);
	/*
	 * The cobalt_process descriptor release may be deferred until
	 * the last mapping on the private heap is gone. However, this
	 * is potentially stale memory already.
	 */
}

struct xnthread_personality cobalt_personality = {
	.name = "cobalt",
	.magic = 0,
	.ops = {
		.attach_process = cobalt_process_attach,
		.detach_process = cobalt_process_detach,
		.map_thread = cobalt_thread_map,
		.exit_thread = cobalt_thread_exit,
		.finalize_thread = cobalt_thread_finalize,
	},
};
EXPORT_SYMBOL_GPL(cobalt_personality);

__init int cobalt_init(void)
{
	unsigned int i, size;
	int ret;

	size = sizeof(*process_hash) * PROCESS_HASH_SIZE;
	process_hash = kmalloc(size, GFP_KERNEL);
	if (process_hash == NULL) {
		printk(XENO_ERR "cannot allocate processes hash table\n");
		return -ENOMEM;
	}

	ret = xndebug_init();
	if (ret)
		goto fail_debug;

	/*
	 * Setup the mayday stuff early, before userland can mess with
	 * real-time ops.
	 */
	ret = xnarch_init_mayday();
	if (ret)
		goto fail_mayday;

	for (i = 0; i < PROCESS_HASH_SIZE; i++)
		INIT_HLIST_HEAD(&process_hash[i]);

	xnsynch_init(&yield_sync, XNSYNCH_FIFO, NULL);

	ret = cobalt_memdev_init();
	if (ret)
		goto fail_memdev;

	ret = cobalt_register_personality(&cobalt_personality);
	if (ret)
		goto fail_register;

	ret = cobalt_signal_init();
	if (ret)
		goto fail_siginit;

	init_hostrt();
	ipipe_set_hooks(ipipe_root_domain, IPIPE_SYSCALL|IPIPE_KEVENT);
	ipipe_set_hooks(&xnsched_realtime_domain, IPIPE_SYSCALL|IPIPE_TRAP);

	if (gid_arg != -1)
		printk(XENO_INFO "allowing access to group %d\n", gid_arg);

	return 0;
fail_siginit:
	cobalt_unregister_personality(0);
fail_register:
	cobalt_memdev_cleanup();
fail_memdev:
	xnsynch_destroy(&yield_sync);
	xnarch_cleanup_mayday();
fail_mayday:
	xndebug_cleanup();
fail_debug:
	kfree(process_hash);

	return ret;
}
