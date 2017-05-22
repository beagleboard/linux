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
#include <cobalt/kernel/lock.h>
#include <cobalt/kernel/clock.h>
#include <cobalt/kernel/apc.h>
#include <cobalt/kernel/vfile.h>
#include <cobalt/kernel/intr.h>
#include <cobalt/kernel/heap.h>
#include <cobalt/kernel/timer.h>
#include <cobalt/kernel/sched.h>
#include <xenomai/version.h>
#include "debug.h"

#if XENO_DEBUG(LOCKING)

static int lock_vfile_show(struct xnvfile_regular_iterator *it, void *data)
{
	struct xnlockinfo lockinfo;
	spl_t s;
	int cpu;

	for_each_realtime_cpu(cpu) {
		xnlock_get_irqsave(&nklock, s);
		lockinfo = per_cpu(xnlock_stats, cpu);
		xnlock_put_irqrestore(&nklock, s);

		if (cpu > 0)
			xnvfile_printf(it, "\n");

		xnvfile_printf(it, "CPU%d:\n", cpu);

		xnvfile_printf(it,
			     "  longest locked section: %llu ns\n"
			     "  spinning time: %llu ns\n"
			     "  section entry: %s:%d (%s)\n",
			       xnclock_ticks_to_ns(&nkclock, lockinfo.lock_time),
			       xnclock_ticks_to_ns(&nkclock, lockinfo.spin_time),
			       lockinfo.file, lockinfo.line, lockinfo.function);
	}

	return 0;
}

static ssize_t lock_vfile_store(struct xnvfile_input *input)
{
	ssize_t ret;
	spl_t s;
	int cpu;

	long val;

	ret = xnvfile_get_integer(input, &val);
	if (ret < 0)
		return ret;

	if (val != 0)
		return -EINVAL;

	for_each_realtime_cpu(cpu) {
		xnlock_get_irqsave(&nklock, s);
		memset(&per_cpu(xnlock_stats, cpu), '\0', sizeof(struct xnlockinfo));
		xnlock_put_irqrestore(&nklock, s);
	}

	return ret;
}

static struct xnvfile_regular_ops lock_vfile_ops = {
	.show = lock_vfile_show,
	.store = lock_vfile_store,
};

static struct xnvfile_regular lock_vfile = {
	.ops = &lock_vfile_ops,
};

#endif /* XENO_DEBUG(LOCKING) */

static int latency_vfile_show(struct xnvfile_regular_iterator *it, void *data)
{
	xnvfile_printf(it, "%Lu\n",
		       xnclock_ticks_to_ns(&nkclock, nkclock.gravity.user));

	return 0;
}

static ssize_t latency_vfile_store(struct xnvfile_input *input)
{
	ssize_t ret;
	long val;

	ret = xnvfile_get_integer(input, &val);
	if (ret < 0)
		return ret;

	nkclock.gravity.user = xnclock_ns_to_ticks(&nkclock, val);

	return ret;
}

static struct xnvfile_regular_ops latency_vfile_ops = {
	.show = latency_vfile_show,
	.store = latency_vfile_store,
};

static struct xnvfile_regular latency_vfile = {
	.ops = &latency_vfile_ops,
};

static int version_vfile_show(struct xnvfile_regular_iterator *it, void *data)
{
	xnvfile_printf(it, "%s\n", XENO_VERSION_STRING);

	return 0;
}

static struct xnvfile_regular_ops version_vfile_ops = {
	.show = version_vfile_show,
};

static struct xnvfile_regular version_vfile = {
	.ops = &version_vfile_ops,
};

static int faults_vfile_show(struct xnvfile_regular_iterator *it, void *data)
{
	int cpu, trap;

	xnvfile_puts(it, "TRAP ");

	for_each_realtime_cpu(cpu)
		xnvfile_printf(it, "        CPU%d", cpu);

	for (trap = 0; cobalt_machine.fault_labels[trap]; trap++) {
		if (*cobalt_machine.fault_labels[trap] == '\0')
			continue;

		xnvfile_printf(it, "\n%3d: ", trap);

		for_each_realtime_cpu(cpu)
			xnvfile_printf(it, "%12u",
				       per_cpu(cobalt_machine_cpudata, cpu).faults[trap]);

		xnvfile_printf(it, "    (%s)",
			       cobalt_machine.fault_labels[trap]);
	}

	xnvfile_putc(it, '\n');

	return 0;
}

static struct xnvfile_regular_ops faults_vfile_ops = {
	.show = faults_vfile_show,
};

static struct xnvfile_regular faults_vfile = {
	.ops = &faults_vfile_ops,
};

static int apc_vfile_show(struct xnvfile_regular_iterator *it, void *data)
{
	int cpu, apc;

	/* We assume the entire output fits in a single page. */

	xnvfile_puts(it, "APC  ");

	for_each_realtime_cpu(cpu)
		xnvfile_printf(it, "        CPU%d", cpu);

	for (apc = 0; apc < BITS_PER_LONG; apc++) {
		if (!test_bit(apc, &cobalt_pipeline.apc_map))
			continue; /* Not hooked. */

		xnvfile_printf(it, "\n%3d: ", apc);

		for_each_realtime_cpu(cpu)
			xnvfile_printf(it, "%12lu",
				       per_cpu(cobalt_machine_cpudata, cpu).apc_shots[apc]);

		if (cobalt_pipeline.apc_table[apc].name)
			xnvfile_printf(it, "    (%s)",
				       cobalt_pipeline.apc_table[apc].name);
	}

	xnvfile_putc(it, '\n');

	return 0;
}

static struct xnvfile_regular_ops apc_vfile_ops = {
	.show = apc_vfile_show,
};

static struct xnvfile_regular apc_vfile = {
	.ops = &apc_vfile_ops,
};

void xnprocfs_cleanup_tree(void)
{
#if XENO_DEBUG(COBALT)
#if XENO_DEBUG(LOCKING)
	xnvfile_destroy_regular(&lock_vfile);
#endif
	xnvfile_destroy_dir(&cobalt_debug_vfroot);
#endif /* XENO_DEBUG(COBALT) */
	xnvfile_destroy_regular(&apc_vfile);
	xnvfile_destroy_regular(&faults_vfile);
	xnvfile_destroy_regular(&version_vfile);
	xnvfile_destroy_regular(&latency_vfile);
	xnintr_cleanup_proc();
	xnheap_cleanup_proc();
	xnclock_cleanup_proc();
	xnsched_cleanup_proc();
	xnvfile_destroy_root();
}

int __init xnprocfs_init_tree(void)
{
	int ret;

	ret = xnvfile_init_root();
	if (ret)
		return ret;

	ret = xnsched_init_proc();
	if (ret)
		return ret;

	xnclock_init_proc();
	xnheap_init_proc();
	xnintr_init_proc();
	xnvfile_init_regular("latency", &latency_vfile, &cobalt_vfroot);
	xnvfile_init_regular("version", &version_vfile, &cobalt_vfroot);
	xnvfile_init_regular("faults", &faults_vfile, &cobalt_vfroot);
	xnvfile_init_regular("apc", &apc_vfile, &cobalt_vfroot);
#ifdef CONFIG_XENO_OPT_DEBUG
	xnvfile_init_dir("debug", &cobalt_debug_vfroot, &cobalt_vfroot);
#if XENO_DEBUG(LOCKING)
	xnvfile_init_regular("lock", &lock_vfile, &cobalt_debug_vfroot);
#endif
#endif /* XENO_DEBUG(COBALT) */

	return 0;
}
