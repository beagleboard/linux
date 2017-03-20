/*
 * Copyright (C) 2005 Philippe Gerum <rpm@xenomai.org>
 * Copyright (C) 2005 Gilles Chanteperdrix <gilles.chanteperdrix@xenomai.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#include <linux/types.h>
#include <linux/err.h>
#include <linux/ipipe.h>
#include <linux/sched.h>
#include <linux/kconfig.h>
#include <cobalt/uapi/corectl.h>
#include <cobalt/kernel/tree.h>
#include <cobalt/kernel/vdso.h>
#include <cobalt/kernel/init.h>
#include <asm-generic/xenomai/mayday.h>
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
#include "corectl.h"
#include "../debug.h"
#include <trace/events/cobalt-posix.h>

/* Syscall must run into the Linux domain. */
#define __xn_exec_lostage    0x1
/* Syscall must run into the Xenomai domain. */
#define __xn_exec_histage    0x2
/* Shadow syscall: caller must be mapped. */
#define __xn_exec_shadow     0x4
/* Switch back toggle; caller must return to its original mode. */
#define __xn_exec_switchback 0x8
/* Exec in current domain. */
#define __xn_exec_current    0x10
/* Exec in conforming domain, Xenomai or Linux. */
#define __xn_exec_conforming 0x20
/* Attempt syscall restart in the opposite domain upon -ENOSYS. */
#define __xn_exec_adaptive   0x40
/* Do not restart syscall upon signal receipt. */
#define __xn_exec_norestart  0x80
/* Shorthand for shadow init syscall. */
#define __xn_exec_init       __xn_exec_lostage
/* Shorthand for shadow syscall in Xenomai space. */
#define __xn_exec_primary   (__xn_exec_shadow|__xn_exec_histage)
/* Shorthand for shadow syscall in Linux space. */
#define __xn_exec_secondary (__xn_exec_shadow|__xn_exec_lostage)
/* Shorthand for syscall in Linux space with switchback if shadow. */
#define __xn_exec_downup    (__xn_exec_lostage|__xn_exec_switchback)
/* Shorthand for non-restartable primary syscall. */
#define __xn_exec_nonrestartable (__xn_exec_primary|__xn_exec_norestart)
/* Domain probing syscall starting in conforming mode. */
#define __xn_exec_probing   (__xn_exec_conforming|__xn_exec_adaptive)
/* Hand over mode selection to syscall.  */
#define __xn_exec_handover  (__xn_exec_current|__xn_exec_adaptive)

typedef long (*cobalt_syshand)(unsigned long arg1, unsigned long arg2,
			       unsigned long arg3, unsigned long arg4,
			       unsigned long arg5);

static void prepare_for_signal(struct task_struct *p,
			       struct xnthread *thread,
			       struct pt_regs *regs,
			       int sysflags)
{
	int notify = 0;
	spl_t s;

	xnlock_get_irqsave(&nklock, s);

	if (xnthread_test_info(thread, XNKICKED)) {
		if (signal_pending(p)) {
			__xn_error_return(regs,
					  (sysflags & __xn_exec_norestart) ?
					  -EINTR : -ERESTARTSYS);
			notify = !xnthread_test_state(thread, XNSSTEP);
			xnthread_clear_info(thread, XNBREAK);
		}
		xnthread_clear_info(thread, XNKICKED);
	}

	xnlock_put_irqrestore(&nklock, s);

	xnthread_test_cancel();

	xnthread_relax(notify, SIGDEBUG_MIGRATE_SIGNAL);
}

static COBALT_SYSCALL(migrate, current, (int domain))
{
	struct xnthread *thread = xnthread_current();

	if (ipipe_root_p) {
		if (domain == COBALT_PRIMARY) {
			if (thread == NULL)
				return -EPERM;
			/*
			 * Paranoid: a corner case where userland
			 * fiddles with SIGSHADOW while the target
			 * thread is still waiting to be started.
			 */
			if (xnthread_test_state(thread, XNDORMANT))
				return 0;

			return xnthread_harden() ? : 1;
		}
		return 0;
	}

	/* ipipe_current_domain != ipipe_root_domain */
	if (domain == COBALT_SECONDARY) {
		xnthread_relax(0, 0);
		return 1;
	}

	return 0;
}

static COBALT_SYSCALL(trace, current,
		      (int op, unsigned long a1,
		       unsigned long a2, unsigned long a3))
{
	int ret = -EINVAL;

	switch (op) {
	case __xntrace_op_max_begin:
		ret = xntrace_max_begin(a1);
		break;

	case __xntrace_op_max_end:
		ret = xntrace_max_end(a1);
		break;

	case __xntrace_op_max_reset:
		ret = xntrace_max_reset();
		break;

	case __xntrace_op_user_start:
		ret = xntrace_user_start();
		break;

	case __xntrace_op_user_stop:
		ret = xntrace_user_stop(a1);
		break;

	case __xntrace_op_user_freeze:
		ret = xntrace_user_freeze(a1, a2);
		break;

	case __xntrace_op_special:
		ret = xntrace_special(a1 & 0xFF, a2);
		break;

	case __xntrace_op_special_u64:
		ret = xntrace_special_u64(a1 & 0xFF,
					  (((u64) a2) << 32) | a3);
		break;
	}
	return ret;
}

static COBALT_SYSCALL(archcall, current,
		      (unsigned long a1, unsigned long a2,
		       unsigned long a3, unsigned long a4,
		       unsigned long a5))
{
	return xnarch_local_syscall(a1, a2, a3, a4, a5);
}

static COBALT_SYSCALL(get_current, current,
		      (xnhandle_t __user *u_handle))
{
	struct xnthread *cur = xnthread_current();

	if (cur == NULL)
		return -EPERM;

	return cobalt_copy_to_user(u_handle, &cur->handle,
				      sizeof(*u_handle));
}

static COBALT_SYSCALL(backtrace, lostage,
		      (int nr, unsigned long __user *u_backtrace, int reason))
{
	unsigned long backtrace[SIGSHADOW_BACKTRACE_DEPTH];
	int ret;

	/*
	 * In case backtrace() in userland is broken or fails. We may
	 * want to know about this in kernel space however, for future
	 * use.
	 */
	if (nr <= 0)
		return 0;
	/*
	 * We may omit the older frames if we can't store the full
	 * backtrace.
	 */
	if (nr > SIGSHADOW_BACKTRACE_DEPTH)
		nr = SIGSHADOW_BACKTRACE_DEPTH;
	/*
	 * Fetch the backtrace array, filled with PC values as seen
	 * from the relaxing thread in user-space. This can't fail
	 */
	ret = cobalt_copy_from_user(backtrace, u_backtrace, nr * sizeof(long));
	if (ret)
		return ret;

	xndebug_trace_relax(nr, backtrace, reason);

	return 0;
}

static COBALT_SYSCALL(serialdbg, current,
		      (const char __user *u_msg, int len))
{
	char buf[128];
	int n;

	while (len > 0) {
		n = len;
		if (n > sizeof(buf))
			n = sizeof(buf);
		if (cobalt_copy_from_user(buf, u_msg, n))
			return -EFAULT;
		__ipipe_serial_debug("%.*s", n, buf);
		u_msg += n;
		len -= n;
	}

	return 0;
}

static COBALT_SYSCALL(mayday, current, (void))
{
	struct pt_regs *regs = task_pt_regs(current);
	struct xnthread *cur;

	cur = xnthread_current();
	if (cur == NULL) {
		printk(XENO_WARNING
		       "MAYDAY received from invalid context %s[%d]\n",
		       current->comm, task_pid_nr(current));
		return -EPERM;
	}

	/*
	 * If the thread was kicked by the watchdog, this syscall we
	 * have just forced on it via the mayday escape will cause it
	 * to relax. See handle_head_syscall().
	 */
	xnarch_fixup_mayday(xnthread_archtcb(cur), regs);

	/*
	 * Return whatever value xnarch_fixup_mayday set for this
	 * register, in order not to undo what xnarch_fixup_mayday
	 * did.
	 */
	return __xn_reg_rval(regs);
}

static void stringify_feature_set(unsigned long fset, char *buf, int size)
{
	unsigned long feature;
	int nc, nfeat;

	*buf = '\0';

	for (feature = 1, nc = nfeat = 0; fset != 0 && size > 0; feature <<= 1) {
		if (fset & feature) {
			nc = ksformat(buf, size, "%s%s",
				      nfeat > 0 ? " " : "",
				      get_feature_label(feature));
			nfeat++;
			size -= nc;
			buf += nc;
			fset &= ~feature;
		}
	}
}

static COBALT_SYSCALL(bind, lostage,
		      (struct cobalt_bindreq __user *u_breq))
{
	unsigned long featreq, featmis;
	struct cobalt_bindreq breq;
	struct cobalt_featinfo *f;
	int abirev;

	if (cobalt_copy_from_user(&breq, u_breq, sizeof(breq)))
		return -EFAULT;

	f = &breq.feat_ret;
	featreq = breq.feat_req;
	if (!realtime_core_running() && (featreq & __xn_feat_control) == 0)
		return -EPERM;

	featmis = (~XENOMAI_FEAT_DEP & (featreq & XENOMAI_FEAT_MAN));
	abirev = breq.abi_rev;

	/*
	 * Pass back the supported feature set and the ABI revision
	 * level to user-space.
	 */
	f->feat_all = XENOMAI_FEAT_DEP;
	stringify_feature_set(XENOMAI_FEAT_DEP, f->feat_all_s,
			      sizeof(f->feat_all_s));
	f->feat_man = featreq & XENOMAI_FEAT_MAN;
	stringify_feature_set(f->feat_man, f->feat_man_s,
			      sizeof(f->feat_man_s));
	f->feat_mis = featmis;
	stringify_feature_set(featmis, f->feat_mis_s,
			      sizeof(f->feat_mis_s));
	f->feat_req = featreq;
	stringify_feature_set(featreq, f->feat_req_s,
			      sizeof(f->feat_req_s));
	f->feat_abirev = XENOMAI_ABI_REV;
	collect_arch_features(f);

	f->clock_freq = cobalt_pipeline.clock_freq;
	f->vdso_offset = cobalt_umm_offset(&cobalt_ppd_get(1)->umm, nkvdso);

	if (cobalt_copy_to_user(u_breq, &breq, sizeof(breq)))
		return -EFAULT;

	/*
	 * If some mandatory features the user-space code relies on
	 * are missing at kernel level, we cannot go further.
	 */
	if (featmis)
		return -EINVAL;

	if (!check_abi_revision(abirev))
		return -ENOEXEC;

	return cobalt_bind_core();
}

static COBALT_SYSCALL(extend, lostage, (unsigned int magic))
{
	return cobalt_bind_personality(magic);
}

static int CoBaLt_ni(void)
{
	return -ENOSYS;
}

/*
 * We have a single syscall table for all ABI models, i.e. 64bit
 * native + 32bit) or plain 32bit. In the former case, we may want to
 * support several models with a single build (e.g. ia32 and x32 for
 * x86_64).
 *
 * The syscall table is set up in a single step, based on three
 * subsequent sources of initializers:
 *
 * - first, all syscall entries are defaulted to a placeholder
 * returning -ENOSYS, as the table may be sparse.
 *
 * - then __COBALT_CALL_ENTRY() produces a native call entry
 * (e.g. pure 64bit call handler for a 64bit architecture), optionally
 * followed by a set of 32bit syscall entries offset by an
 * arch-specific base index, which default to the native calls. These
 * nitty-gritty details are defined by
 * <asm/xenomai/syscall32.h>. 32bit architectures - or 64bit ones for
 * which we don't support any 32bit ABI model - will simply define
 * __COBALT_CALL32_ENTRY() as an empty macro.
 *
 * - finally, 32bit thunk entries are generated per-architecture, by
 * including <asm/xenomai/syscall32-table.h>, overriding the default
 * handlers installed during the previous step.
 *
 * For instance, with CONFIG_X86_X32 support enabled in an x86_64
 * kernel, sc_cobalt_mq_timedreceive would appear twice in the table,
 * as:
 *
 * [sc_cobalt_mq_timedreceive] = cobalt_mq_timedreceive,
 * ...
 * [sc_cobalt_mq_timedreceive + __COBALT_X32_BASE] = cobalt32x_mq_timedreceive,
 *
 * cobalt32x_mq_timedreceive() would do the required thunking for
 * dealing with the 32<->64bit conversion of arguments. On the other
 * hand, sc_cobalt_sched_yield - which do not require any thunk -
 * would also appear twice, but both entries would point at the native
 * syscall implementation:
 *
 * [sc_cobalt_sched_yield] = cobalt_sched_yield,
 * ...
 * [sc_cobalt_sched_yield + __COBALT_X32_BASE] = cobalt_sched_yield,
 *
 * Accordingly, applications targeting the x32 model (-mx32) issue
 * syscalls in the range [__COBALT_X32_BASE..__COBALT_X32_BASE +
 * __NR_COBALT_SYSCALLS-1], whilst native (32/64bit) ones issue
 * syscalls in the range [0..__NR_COBALT_SYSCALLS-1].
 *
 * In short, this is an incremental process where the arch-specific
 * code can override the 32bit syscall entries, pointing at the thunk
 * routines it may need for handing 32bit calls over their respective
 * 64bit implementation.
 *
 * By convention, there is NO pure 32bit syscall, which means that
 * each 32bit syscall defined by a compat ABI interface MUST match a
 * native (64bit) syscall. This is important as we share the call
 * modes (i.e. __xn_exec_ bits) between all ABI models.
 *
 * --rpm
 */
#define __syshand__(__name)	((cobalt_syshand)(CoBaLt_ ## __name))

#define __COBALT_NI	__syshand__(ni)

#define __COBALT_CALL_NI				\
	[0 ... __NR_COBALT_SYSCALLS-1] = __COBALT_NI,	\
	__COBALT_CALL32_INITHAND(__COBALT_NI)

#define __COBALT_CALL_NFLAGS				\
	[0 ... __NR_COBALT_SYSCALLS-1] = 0,		\
	__COBALT_CALL32_INITMODE(0)

#define __COBALT_CALL_ENTRY(__name)				\
	[sc_cobalt_ ## __name] = __syshand__(__name),		\
	__COBALT_CALL32_ENTRY(__name, __syshand__(__name))

#define __COBALT_MODE(__name, __mode)	\
	[sc_cobalt_ ## __name] = __xn_exec_##__mode,

#ifdef CONFIG_XENO_ARCH_SYS3264
#include "syscall32.h"
#endif

#include "syscall_entries.h"

static const cobalt_syshand cobalt_syscalls[] = {
	__COBALT_CALL_NI
	__COBALT_CALL_ENTRIES
#ifdef CONFIG_XENO_ARCH_SYS3264
#include <asm/xenomai/syscall32-table.h>
#endif
};

static const int cobalt_sysmodes[] = {
	__COBALT_CALL_NFLAGS
	__COBALT_CALL_MODES
};

static inline int allowed_syscall(struct cobalt_process *process,
				  struct xnthread *thread,
				  int sysflags, int nr)
{
	if (nr == sc_cobalt_bind)
		return 1;
	
	if (process == NULL)
		return 0;

	if (thread == NULL && (sysflags & __xn_exec_shadow))
		return 0;

	return cap_raised(current_cap(), CAP_SYS_NICE);
}

static int handle_head_syscall(struct ipipe_domain *ipd, struct pt_regs *regs)
{
	struct cobalt_process *process;
	int switched, sigs, sysflags;
	struct xnthread *thread;
	cobalt_syshand handler;
	struct task_struct *p;
	unsigned int nr, code;
	long ret;

	if (!__xn_syscall_p(regs))
		goto linux_syscall;

	thread = xnthread_current();
	code = __xn_syscall(regs);
	if (code >= ARRAY_SIZE(cobalt_syscalls))
		goto bad_syscall;

	nr = code & (__NR_COBALT_SYSCALLS - 1);

	trace_cobalt_head_sysentry(thread, code);

	process = cobalt_current_process();
	if (process == NULL) {
		process = cobalt_search_process(current->mm);
		cobalt_set_process(process);
	}

	handler = cobalt_syscalls[code];
	sysflags = cobalt_sysmodes[nr];

	/*
	 * Executing Cobalt services requires CAP_SYS_NICE, except for
	 * sc_cobalt_bind which does its own checks.
	 */
	if (unlikely(!allowed_syscall(process, thread, sysflags, nr))) {
		/*
		 * Exclude get_current from reporting, it is used to probe the
		 * execution context.
		 */
		if (XENO_DEBUG(COBALT) && nr != sc_cobalt_get_current)
			printk(XENO_WARNING
			       "syscall <%d> denied to %s[%d]\n",
			       nr, current->comm, task_pid_nr(current));
		__xn_error_return(regs, -EPERM);
		goto ret_handled;
	}

	if (sysflags & __xn_exec_conforming)
		/*
		 * If the conforming exec bit is set, turn the exec
		 * bitmask for the syscall into the most appropriate
		 * setup for the caller, i.e. Xenomai domain for
		 * shadow threads, Linux otherwise.
		 */
		sysflags |= (thread ? __xn_exec_histage : __xn_exec_lostage);

	/*
	 * Here we have to dispatch the syscall execution properly,
	 * depending on:
	 *
	 * o Whether the syscall must be run into the Linux or Xenomai
	 * domain, or indifferently in the current Xenomai domain.
	 *
	 * o Whether the caller currently runs in the Linux or Xenomai
	 * domain.
	 */
restart:
	/*
	 * Process adaptive syscalls by restarting them in the
	 * opposite domain upon receiving -ENOSYS from the syscall
	 * handler.
	 */
	switched = 0;
	if (sysflags & __xn_exec_lostage) {
		/*
		 * The syscall must run from the Linux domain.
		 */
		if (ipd == &xnsched_realtime_domain) {
			/*
			 * Request originates from the Xenomai domain:
			 * relax the caller then invoke the syscall
			 * handler right after.
			 */
			xnthread_relax(1, SIGDEBUG_MIGRATE_SYSCALL);
			switched = 1;
		} else
			/*
			 * Request originates from the Linux domain:
			 * propagate the event to our Linux-based
			 * handler, so that the syscall is executed
			 * from there.
			 */
			return KEVENT_PROPAGATE;
	} else if (sysflags & (__xn_exec_histage | __xn_exec_current)) {
		/*
		 * Syscall must run either from the Xenomai domain, or
		 * from the calling domain.
		 *
		 * If the request originates from the Linux domain,
		 * hand it over to our secondary-mode dispatcher.
		 * Otherwise, invoke the syscall handler immediately.
		 */
		if (ipd != &xnsched_realtime_domain)
			return KEVENT_PROPAGATE;
	}

	/*
	 * 'thread' has to be valid from that point: all syscalls
	 * regular threads may call have been pipelined to the root
	 * handler (lostage ones), or rejected by allowed_syscall().
	 */

	ret = handler(__xn_reg_arglist(regs));
	if (ret == -ENOSYS && (sysflags & __xn_exec_adaptive)) {
		if (switched) {
			ret = xnthread_harden();
			if (ret) {
				switched = 0;
				goto done;
			}
		} else /* Mark the primary -> secondary transition. */
			xnthread_set_localinfo(thread, XNDESCENT);
		sysflags ^=
		    (__xn_exec_lostage | __xn_exec_histage |
		     __xn_exec_adaptive);
		goto restart;
	}
done:
	__xn_status_return(regs, ret);
	sigs = 0;
	if (!xnsched_root_p()) {
		p = current;
		if (signal_pending(p) ||
		    xnthread_test_info(thread, XNKICKED)) {
			sigs = 1;
			prepare_for_signal(p, thread, regs, sysflags);
		} else if (xnthread_test_state(thread, XNWEAK) &&
			   thread->res_count == 0) {
			if (switched)
				switched = 0;
			else
				xnthread_relax(0, 0);
		}
	}
	if (!sigs && (sysflags & __xn_exec_switchback) && switched)
		/* -EPERM will be trapped later if needed. */
		xnthread_harden();

ret_handled:
	/* Update the stats and userland-visible state. */
	if (thread) {
		xnthread_clear_localinfo(thread, XNDESCENT);
		xnstat_counter_inc(&thread->stat.xsc);
		xnthread_sync_window(thread);
	}

	trace_cobalt_head_sysexit(thread, __xn_reg_rval(regs));

	return KEVENT_STOP;

linux_syscall:
	if (xnsched_root_p())
		/*
		 * The call originates from the Linux domain, either
		 * from a relaxed shadow or from a regular Linux task;
		 * just propagate the event so that we will fall back
		 * to handle_root_syscall().
		 */
		return KEVENT_PROPAGATE;

	/*
	 * From now on, we know that we have a valid shadow thread
	 * pointer.
	 *
	 * The current syscall will eventually fall back to the Linux
	 * syscall handler if our Linux domain handler does not
	 * intercept it. Before we let it go, ensure that the current
	 * thread has properly entered the Linux domain.
	 */
	xnthread_relax(1, SIGDEBUG_MIGRATE_SYSCALL);

	return KEVENT_PROPAGATE;

bad_syscall:
	printk(XENO_WARNING "bad syscall <%#lx>\n", __xn_syscall(regs));

	__xn_error_return(regs, -ENOSYS);

	return KEVENT_STOP;
}

static int handle_root_syscall(struct ipipe_domain *ipd, struct pt_regs *regs)
{
	int sysflags, switched, sigs;
	struct xnthread *thread;
	cobalt_syshand handler;
	struct task_struct *p;
	unsigned int nr, code;
	long ret;

	/*
	 * Catch cancellation requests pending for user shadows
	 * running mostly in secondary mode, i.e. XNWEAK. In that
	 * case, we won't run prepare_for_signal() that frequently, so
	 * check for cancellation here.
	 */
	xnthread_test_cancel();

	if (!__xn_syscall_p(regs))
		/* Fall back to Linux syscall handling. */
		return KEVENT_PROPAGATE;

	thread = xnthread_current();
	/* code has already been checked in the head domain handler. */
	code = __xn_syscall(regs);
	nr = code & (__NR_COBALT_SYSCALLS - 1);

	trace_cobalt_root_sysentry(thread, code);

	/* Processing a Xenomai syscall. */

	handler = cobalt_syscalls[code];
	sysflags = cobalt_sysmodes[nr];

	if (thread && (sysflags & __xn_exec_conforming))
		sysflags |= __xn_exec_histage;
restart:
	/*
	 * Process adaptive syscalls by restarting them in the
	 * opposite domain upon receiving -ENOSYS from the syscall
	 * handler.
	 */
	switched = 0;
	if (sysflags & __xn_exec_histage) {
		/*
		 * This request originates from the Linux domain but
		 * should run into the Xenomai domain: harden the
		 * caller before invoking the syscall handler.
		 */
		ret = xnthread_harden();
		if (ret) {
			__xn_error_return(regs, ret);
			goto ret_handled;
		}
		switched = 1;
	}

	ret = handler(__xn_reg_arglist(regs));
	if (ret == -ENOSYS && (sysflags & __xn_exec_adaptive)) {
		sysflags ^= __xn_exec_histage;
		if (switched) {
			xnthread_relax(1, SIGDEBUG_MIGRATE_SYSCALL);
			sysflags &= ~__xn_exec_adaptive;
			 /* Mark the primary -> secondary transition. */
			xnthread_set_localinfo(thread, XNDESCENT);
		}
		goto restart;
	}

	__xn_status_return(regs, ret);

	sigs = 0;
	if (!xnsched_root_p()) {
		/*
		 * We may have gained a shadow TCB from the syscall we
		 * just invoked, so make sure to fetch it.
		 */
		thread = xnthread_current();
		p = current;
		if (signal_pending(p)) {
			sigs = 1;
			prepare_for_signal(p, thread, regs, sysflags);
		} else if (xnthread_test_state(thread, XNWEAK) &&
			   thread->res_count == 0)
			sysflags |= __xn_exec_switchback;
	}
	if (!sigs && (sysflags & __xn_exec_switchback)
	    && (switched || xnsched_primary_p()))
		xnthread_relax(0, 0);

ret_handled:
	/* Update the stats and userland-visible state. */
	if (thread) {
		xnthread_clear_localinfo(thread, XNDESCENT);
		xnstat_counter_inc(&thread->stat.xsc);
		xnthread_sync_window(thread);
	}

	trace_cobalt_root_sysexit(thread, __xn_reg_rval(regs));

	return KEVENT_STOP;
}

int ipipe_syscall_hook(struct ipipe_domain *ipd, struct pt_regs *regs)
{
	if (unlikely(ipipe_root_p))
		return handle_root_syscall(ipd, regs);

	return handle_head_syscall(ipd, regs);
}

int ipipe_fastcall_hook(struct pt_regs *regs)
{
	int ret;

	ret = handle_head_syscall(&xnsched_realtime_domain, regs);
	XENO_BUG_ON(COBALT, ret == KEVENT_PROPAGATE);

	return ret;
}

long cobalt_restart_syscall_placeholder(struct restart_block *param)
{
	return -EINVAL;
}
