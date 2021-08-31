/*
 * Copyright (C) 2016 Philippe Gerum <rpm@xenomai.org>.
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
#include <linux/errno.h>
#include <linux/ipipe.h>
#include <linux/kconfig.h>
#include <linux/atomic.h>
#include <linux/printk.h>
#include <cobalt/kernel/init.h>
#include <cobalt/kernel/thread.h>
#include <xenomai/version.h>
#include <asm/xenomai/syscall.h>
#include "corectl.h"

static BLOCKING_NOTIFIER_HEAD(config_notifier_list);

static int do_conf_option(int option, void __user *u_buf, size_t u_bufsz)
{
	struct cobalt_config_vector vec;
	int ret, val = 0;

	if (option <= _CC_COBALT_GET_CORE_STATUS && u_bufsz < sizeof(val))
		return -EINVAL;

	switch (option) {
	case _CC_COBALT_GET_VERSION:
		val = XENO_VERSION_CODE;
		break;
	case _CC_COBALT_GET_NR_PIPES:
#ifdef CONFIG_XENO_OPT_PIPE
		val = CONFIG_XENO_OPT_PIPE_NRDEV;
#endif
		break;
	case _CC_COBALT_GET_NR_TIMERS:
		val = CONFIG_XENO_OPT_NRTIMERS;
		break;
	case _CC_COBALT_GET_POLICIES:
		val = _CC_COBALT_SCHED_FIFO|_CC_COBALT_SCHED_RR;
		if (IS_ENABLED(CONFIG_XENO_OPT_SCHED_WEAK))
			val |= _CC_COBALT_SCHED_WEAK;
		if (IS_ENABLED(CONFIG_XENO_OPT_SCHED_SPORADIC))
			val |= _CC_COBALT_SCHED_SPORADIC;
		if (IS_ENABLED(CONFIG_XENO_OPT_SCHED_QUOTA))
			val |= _CC_COBALT_SCHED_QUOTA;
		if (IS_ENABLED(CONFIG_XENO_OPT_SCHED_TP))
			val |= _CC_COBALT_SCHED_TP;
		break;
	case _CC_COBALT_GET_DEBUG:
		if (IS_ENABLED(CONFIG_XENO_OPT_DEBUG_COBALT))
			val |= _CC_COBALT_DEBUG_ASSERT;
		if (IS_ENABLED(CONFIG_XENO_OPT_DEBUG_CONTEXT))
			val |= _CC_COBALT_DEBUG_CONTEXT;
		if (IS_ENABLED(CONFIG_XENO_OPT_DEBUG_LOCKING))
			val |= _CC_COBALT_DEBUG_LOCKING;
		if (IS_ENABLED(CONFIG_XENO_OPT_DEBUG_USER))
			val |= _CC_COBALT_DEBUG_USER;
		if (IS_ENABLED(CONFIG_XENO_OPT_DEBUG_MUTEX_RELAXED))
			val |= _CC_COBALT_DEBUG_MUTEX_RELAXED;
		if (IS_ENABLED(CONFIG_XENO_OPT_DEBUG_MUTEX_SLEEP))
			val |= _CC_COBALT_DEBUG_MUTEX_SLEEP;
		if (IS_ENABLED(CONFIG_XENO_OPT_DEBUG_LEGACY))
			val |= _CC_COBALT_DEBUG_LEGACY;
		if (IS_ENABLED(CONFIG_XENO_OPT_DEBUG_TRACE_RELAX))
			val |= _CC_COBALT_DEBUG_TRACE_RELAX;
		if (IS_ENABLED(CONFIG_XENO_DRIVERS_RTNET_CHECKED))
			val |= _CC_COBALT_DEBUG_NET;
		break;
	case _CC_COBALT_GET_WATCHDOG:
#ifdef CONFIG_XENO_OPT_WATCHDOG
		val = CONFIG_XENO_OPT_WATCHDOG_TIMEOUT;
#endif
		break;
	case _CC_COBALT_GET_CORE_STATUS:
		val = realtime_core_state();
		break;
	default:
		if (!ipipe_root_p)
			/* Switch to secondary mode first. */
			return -ENOSYS;
		vec.u_buf = u_buf;
		vec.u_bufsz = u_bufsz;
		ret = blocking_notifier_call_chain(&config_notifier_list,
						   option, &vec);
		if (ret == NOTIFY_DONE)
			return -EINVAL; /* Nobody cared. */
		return notifier_to_errno(ret);
	}

	ret = cobalt_copy_to_user(u_buf, &val, sizeof(val));

	return ret ? -EFAULT : 0;
}

static int stop_services(const void __user *u_buf, size_t u_bufsz)
{
	const u32 final_grace_period = 3; /* seconds */
	enum cobalt_run_states state;
	__u32 grace_period;
	int ret;

	/*
	 * XXX: we don't have any syscall for unbinding a thread from
	 * the Cobalt core, so we deny real-time threads from stopping
	 * Cobalt services. i.e. _CC_COBALT_STOP_CORE must be issued
	 * from a plain regular linux thread.
	 */
	if (xnthread_current())
		return -EPERM;

	if (u_bufsz != sizeof(__u32))
		return -EINVAL;

	ret = cobalt_copy_from_user(&grace_period,
				    u_buf, sizeof(grace_period));
	if (ret)
		return ret;

	state = atomic_cmpxchg(&cobalt_runstate,
			       COBALT_STATE_RUNNING,
			       COBALT_STATE_TEARDOWN);
	switch (state) {
	case COBALT_STATE_STOPPED:
		break;
	case COBALT_STATE_RUNNING:
		/* Kill user threads. */
		ret = xnthread_killall(grace_period, XNUSER);
		if (ret) {
			set_realtime_core_state(state);
			return ret;
		}
		cobalt_call_state_chain(COBALT_STATE_TEARDOWN);
		/* Kill lingering RTDM tasks. */
		ret = xnthread_killall(final_grace_period, 0);
		if (ret == -EAGAIN)
			printk(XENO_WARNING "some RTDM tasks won't stop");
		xntimer_release_hardware();
		set_realtime_core_state(COBALT_STATE_STOPPED);
		printk(XENO_INFO "services stopped\n");
		break;
	default:
		ret = -EINPROGRESS;
	}

	return ret;
}

static int start_services(void)
{
	enum cobalt_run_states state;
	int ret = 0;

	state = atomic_cmpxchg(&cobalt_runstate,
			       COBALT_STATE_STOPPED,
			       COBALT_STATE_WARMUP);
	switch (state) {
	case COBALT_STATE_RUNNING:
		break;
	case COBALT_STATE_STOPPED:
		xntimer_grab_hardware();
		cobalt_call_state_chain(COBALT_STATE_WARMUP);
		set_realtime_core_state(COBALT_STATE_RUNNING);
		printk(XENO_INFO "services started\n");
		break;
	default:
		ret = -EINPROGRESS;
	}

	return ret;
}

COBALT_SYSCALL(corectl, probing,
	       (int request, void __user *u_buf, size_t u_bufsz))
{
	int ret;
	
	switch (request) {
	case _CC_COBALT_STOP_CORE:
		ret = stop_services(u_buf, u_bufsz);
		break;
	case _CC_COBALT_START_CORE:
		ret = start_services();
		break;
	default:
		ret = do_conf_option(request, u_buf, u_bufsz);
	}
	
	return ret;
}

void cobalt_add_config_chain(struct notifier_block *nb)
{
	blocking_notifier_chain_register(&config_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(cobalt_add_config_chain);

void cobalt_remove_config_chain(struct notifier_block *nb)
{
	blocking_notifier_chain_unregister(&config_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(cobalt_remove_config_chain);
