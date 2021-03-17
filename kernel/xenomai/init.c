/*
 * Copyright (C) 2001-2013 Philippe Gerum <rpm@xenomai.org>.
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
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ipipe_tickdev.h>
#include <xenomai/version.h>
#include <cobalt/kernel/sched.h>
#include <cobalt/kernel/clock.h>
#include <cobalt/kernel/timer.h>
#include <cobalt/kernel/heap.h>
#include <cobalt/kernel/intr.h>
#include <cobalt/kernel/apc.h>
#include <cobalt/kernel/ppd.h>
#include <cobalt/kernel/pipe.h>
#include <cobalt/kernel/select.h>
#include <cobalt/kernel/vdso.h>
#include <rtdm/fd.h>
#include "rtdm/internal.h"
#include "posix/internal.h"
#include "procfs.h"

/**
 * @defgroup cobalt Cobalt
 *
 * Cobalt supplements the native Linux kernel in dual kernel
 * configurations. It deals with all time-critical activities, such as
 * handling interrupts, and scheduling real-time threads. The Cobalt
 * kernel has higher priority over all the native kernel activities.
 *
 * Cobalt provides an implementation of the POSIX and RTDM interfaces
 * based on a set of generic RTOS building blocks.
 */

static unsigned long timerfreq_arg;
module_param_named(timerfreq, timerfreq_arg, ulong, 0444);

static unsigned long clockfreq_arg;
module_param_named(clockfreq, clockfreq_arg, ulong, 0444);

#ifdef CONFIG_SMP
static unsigned long supported_cpus_arg = -1;
module_param_named(supported_cpus, supported_cpus_arg, ulong, 0444);
#endif /* CONFIG_SMP */

static unsigned long sysheap_size_arg;
module_param_named(sysheap_size, sysheap_size_arg, ulong, 0444);

static char init_state_arg[16] = "enabled";
module_param_string(state, init_state_arg, sizeof(init_state_arg), 0444);

static BLOCKING_NOTIFIER_HEAD(state_notifier_list);

struct cobalt_pipeline cobalt_pipeline;
EXPORT_SYMBOL_GPL(cobalt_pipeline);

DEFINE_PER_CPU(struct cobalt_machine_cpudata, cobalt_machine_cpudata);
EXPORT_PER_CPU_SYMBOL_GPL(cobalt_machine_cpudata);

atomic_t cobalt_runstate = ATOMIC_INIT(COBALT_STATE_WARMUP);
EXPORT_SYMBOL_GPL(cobalt_runstate);

struct cobalt_ppd cobalt_kernel_ppd = {
	.exe_path = "vmlinux",
};
EXPORT_SYMBOL_GPL(cobalt_kernel_ppd);

#ifdef CONFIG_XENO_OPT_DEBUG
#define boot_debug_notice "[DEBUG]"
#else
#define boot_debug_notice ""
#endif

#ifdef CONFIG_IPIPE_TRACE
#define boot_lat_trace_notice "[LTRACE]"
#else
#define boot_lat_trace_notice ""
#endif

#ifdef CONFIG_ENABLE_DEFAULT_TRACERS
#define boot_evt_trace_notice "[ETRACE]"
#else
#define boot_evt_trace_notice ""
#endif

#define boot_state_notice						\
	({								\
		realtime_core_state() == COBALT_STATE_STOPPED ?		\
			"[STOPPED]" : "";				\
	})

void cobalt_add_state_chain(struct notifier_block *nb)
{
	blocking_notifier_chain_register(&state_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(cobalt_add_state_chain);

void cobalt_remove_state_chain(struct notifier_block *nb)
{
	blocking_notifier_chain_unregister(&state_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(cobalt_remove_state_chain);

void cobalt_call_state_chain(enum cobalt_run_states newstate)
{
	blocking_notifier_call_chain(&state_notifier_list, newstate, NULL);
}
EXPORT_SYMBOL_GPL(cobalt_call_state_chain);

static void sys_shutdown(void)
{
	void *membase;

	xntimer_release_hardware();
	xnsched_destroy_all();
	xnregistry_cleanup();
	membase = xnheap_get_membase(&cobalt_heap);
	xnheap_destroy(&cobalt_heap);
	xnheap_vfree(membase);
}

static int __init mach_setup(void)
{
	struct ipipe_sysinfo sysinfo;
	int ret, virq;

	ret = ipipe_select_timers(&xnsched_realtime_cpus);
	if (ret < 0)
		return ret;

	ipipe_get_sysinfo(&sysinfo);

	if (timerfreq_arg == 0)
		timerfreq_arg = sysinfo.sys_hrtimer_freq;

	if (clockfreq_arg == 0)
		clockfreq_arg = sysinfo.sys_hrclock_freq;

	if (clockfreq_arg == 0) {
		printk(XENO_ERR "null clock frequency? Aborting.\n");
		return -ENODEV;
	}

	cobalt_pipeline.timer_freq = timerfreq_arg;
	cobalt_pipeline.clock_freq = clockfreq_arg;

	if (cobalt_machine.init) {
		ret = cobalt_machine.init();
		if (ret)
			return ret;
	}

	ipipe_register_head(&xnsched_realtime_domain, "Xenomai");

	ret = -EBUSY;
	virq = ipipe_alloc_virq();
	if (virq == 0)
		goto fail_apc;

	cobalt_pipeline.apc_virq = virq;

	ipipe_request_irq(ipipe_root_domain,
			  cobalt_pipeline.apc_virq,
			  apc_dispatch,
			  NULL, NULL);

	virq = ipipe_alloc_virq();
	if (virq == 0)
		goto fail_escalate;

	cobalt_pipeline.escalate_virq = virq;

	ipipe_request_irq(&xnsched_realtime_domain,
			  cobalt_pipeline.escalate_virq,
			  (ipipe_irq_handler_t)__xnsched_run_handler,
			  NULL, NULL);

	ret = xnclock_init(cobalt_pipeline.clock_freq);
	if (ret)
		goto fail_clock;

	return 0;

fail_clock:
	ipipe_free_irq(&xnsched_realtime_domain,
		       cobalt_pipeline.escalate_virq);
	ipipe_free_virq(cobalt_pipeline.escalate_virq);
fail_escalate:
	ipipe_free_irq(ipipe_root_domain,
		       cobalt_pipeline.apc_virq);
	ipipe_free_virq(cobalt_pipeline.apc_virq);
fail_apc:
	ipipe_unregister_head(&xnsched_realtime_domain);

	if (cobalt_machine.cleanup)
		cobalt_machine.cleanup();

	return ret;
}

static inline int __init mach_late_setup(void)
{
	if (cobalt_machine.late_init)
		return cobalt_machine.late_init();

	return 0;
}

static __init void mach_cleanup(void)
{
	ipipe_unregister_head(&xnsched_realtime_domain);
	ipipe_free_irq(&xnsched_realtime_domain,
		       cobalt_pipeline.escalate_virq);
	ipipe_free_virq(cobalt_pipeline.escalate_virq);
	ipipe_timers_release();
	xnclock_cleanup();
}

static struct {
	const char *label;
	enum cobalt_run_states state;
} init_states[] __initdata = {
	{ "disabled", COBALT_STATE_DISABLED },
	{ "stopped", COBALT_STATE_STOPPED },
	{ "enabled", COBALT_STATE_WARMUP },
};
	
static void __init setup_init_state(void)
{
	static char warn_bad_state[] __initdata =
		XENO_WARNING "invalid init state '%s'\n";
	int n;

	for (n = 0; n < ARRAY_SIZE(init_states); n++)
		if (strcmp(init_states[n].label, init_state_arg) == 0) {
			set_realtime_core_state(init_states[n].state);
			return;
		}

	printk(warn_bad_state, init_state_arg);
}

static __init int sys_init(void)
{
	void *heapaddr;
	int ret;

	if (sysheap_size_arg == 0)
		sysheap_size_arg = CONFIG_XENO_OPT_SYS_HEAPSZ;

	heapaddr = xnheap_vmalloc(sysheap_size_arg * 1024);
	if (heapaddr == NULL ||
	    xnheap_init(&cobalt_heap, heapaddr, sysheap_size_arg * 1024)) {
		return -ENOMEM;
	}
	xnheap_set_name(&cobalt_heap, "system heap");

	xnsched_init_all();

	xnregistry_init();

	/*
	 * If starting in stopped mode, do all initializations, but do
	 * not enable the core timer.
	 */
	if (realtime_core_state() == COBALT_STATE_WARMUP) {
		ret = xntimer_grab_hardware();
		if (ret) {
			sys_shutdown();
			return ret;
		}
		set_realtime_core_state(COBALT_STATE_RUNNING);
	}

	return 0;
}

static int __init xenomai_init(void)
{
	int ret, __maybe_unused cpu;

	setup_init_state();

	if (!realtime_core_enabled()) {
		printk(XENO_WARNING "disabled on kernel command line\n");
		return 0;
	}

#ifdef CONFIG_SMP
	cpumask_clear(&xnsched_realtime_cpus);
	for_each_online_cpu(cpu) {
		if (supported_cpus_arg & (1UL << cpu))
			cpumask_set_cpu(cpu, &xnsched_realtime_cpus);
	}
	if (cpumask_empty(&xnsched_realtime_cpus)) {
		printk(XENO_WARNING "disabled via empty real-time CPU mask\n");
		set_realtime_core_state(COBALT_STATE_DISABLED);
		return 0;
	}
	cobalt_cpu_affinity = xnsched_realtime_cpus;
#endif /* CONFIG_SMP */

	xnsched_register_classes();

	ret = xnprocfs_init_tree();
	if (ret)
		goto fail;

	ret = mach_setup();
	if (ret)
		goto cleanup_proc;

	xnintr_mount();

	ret = xnpipe_mount();
	if (ret)
		goto cleanup_mach;

	ret = xnselect_mount();
	if (ret)
		goto cleanup_pipe;

	ret = sys_init();
	if (ret)
		goto cleanup_select;

	ret = mach_late_setup();
	if (ret)
		goto cleanup_sys;

	ret = rtdm_init();
	if (ret)
		goto cleanup_sys;

	ret = cobalt_init();
	if (ret)
		goto cleanup_rtdm;

	rtdm_fd_init();

	printk(XENO_INFO "Cobalt v%s %s%s%s%s\n",
	       XENO_VERSION_STRING,
	       boot_debug_notice,
	       boot_lat_trace_notice,
	       boot_evt_trace_notice,
	       boot_state_notice);

	return 0;

cleanup_rtdm:
	rtdm_cleanup();
cleanup_sys:
	sys_shutdown();
cleanup_select:
	xnselect_umount();
cleanup_pipe:
	xnpipe_umount();
cleanup_mach:
	mach_cleanup();
cleanup_proc:
	xnprocfs_cleanup_tree();
fail:
	set_realtime_core_state(COBALT_STATE_DISABLED);
	printk(XENO_ERR "init failed, code %d\n", ret);

	return ret;
}
device_initcall(xenomai_init);

/**
 * @ingroup cobalt
 * @defgroup cobalt_core Cobalt kernel
 *
 * The Cobalt core is a co-kernel which supplements the Linux kernel
 * for delivering real-time services with very low latency. It
 * implements a set of generic RTOS building blocks, which the
 * Cobalt/POSIX and Cobalt/RTDM APIs are based on.  Cobalt has higher
 * priority over the Linux kernel activities.
 *
 * @{
 *
 * @page cobalt-core-tags Dual kernel service tags
 *
 * The Cobalt kernel services may be restricted to particular calling
 * contexts, or entail specific side-effects. To describe this
 * information, each service documented by this section bears a set of
 * tags when applicable.
 *
 * The table below matches the tags used throughout the documentation
 * with the description of their meaning for the caller.
 *
 * @par
 * <b>Context tags</b>
 * <TABLE>
 * <TR><TH>Tag</TH> <TH>Context on entry</TH></TR>
 * <TR><TD>primary-only</TD>	<TD>Must be called from a Cobalt task in primary mode</TD></TR>
 * <TR><TD>primary-timed</TD>	<TD>Requires a Cobalt task in primary mode if timed</TD></TR>
 * <TR><TD>coreirq-only</TD>	<TD>Must be called from a Cobalt IRQ handler</TD></TR>
 * <TR><TD>secondary-only</TD>	<TD>Must be called from a Cobalt task in secondary mode or regular Linux task</TD></TR>
 * <TR><TD>rtdm-task</TD>	<TD>Must be called from a RTDM driver task</TD></TR>
 * <TR><TD>mode-unrestricted</TD>	<TD>May be called from a Cobalt task in either primary or secondary mode</TD></TR>
 * <TR><TD>task-unrestricted</TD>	<TD>May be called from a Cobalt or regular Linux task indifferently</TD></TR>
 * <TR><TD>unrestricted</TD>	<TD>May be called from any context previously described</TD></TR>
 * <TR><TD>atomic-entry</TD>	<TD>Caller must currently hold the big Cobalt kernel lock (nklock)</TD></TR>
 * </TABLE>
 *
 * @par
 * <b>Possible side-effects</b>
 * <TABLE>
 * <TR><TH>Tag</TH> <TH>Description</TH></TR>
 * <TR><TD>might-switch</TD>	<TD>The Cobalt kernel may switch context</TD></TR>
 * </TABLE>
 *
 * @}
 */
