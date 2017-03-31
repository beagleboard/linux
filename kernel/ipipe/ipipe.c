/* -*- linux-c -*-
 * kernel/ipipe/tracer.c
 *
 * Copyright (C) 2005 Luotao Fu.
 *		 2005-2008 Jan Kiszka.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, Inc., 675 Mass Ave, Cambridge MA 02139,
 * USA; either version 2 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/kallsyms.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/ctype.h>
#include <linux/vmalloc.h>
#include <linux/pid.h>
#include <linux/vermagic.h>
#include <linux/sched.h>
#include <linux/ipipe.h>
#include <linux/ftrace.h>
#include <asm/uaccess.h>

#define IPIPE_TRACE_PATHS	    4 /* <!> Do not lower below 3 */
#define IPIPE_DEFAULT_ACTIVE	    0
#define IPIPE_DEFAULT_MAX	    1
#define IPIPE_DEFAULT_FROZEN	    2

#define IPIPE_TRACE_POINTS	    (1 << CONFIG_IPIPE_TRACE_SHIFT)
#define WRAP_POINT_NO(point)	    ((point) & (IPIPE_TRACE_POINTS-1))

#define IPIPE_DEFAULT_PRE_TRACE	    10
#define IPIPE_DEFAULT_POST_TRACE    10
#define IPIPE_DEFAULT_BACK_TRACE    100

#define IPIPE_DELAY_NOTE	    1000  /* in nanoseconds */
#define IPIPE_DELAY_WARN	    10000 /* in nanoseconds */

#define IPIPE_TFLG_NMI_LOCK	    0x0001
#define IPIPE_TFLG_NMI_HIT	    0x0002
#define IPIPE_TFLG_NMI_FREEZE_REQ   0x0004

#define IPIPE_TFLG_HWIRQ_OFF	    0x0100
#define IPIPE_TFLG_FREEZING	    0x0200
#define IPIPE_TFLG_CURRDOM_SHIFT    10	 /* bits 10..11: current domain */
#define IPIPE_TFLG_CURRDOM_MASK	    0x0C00
#define IPIPE_TFLG_DOMSTATE_SHIFT   12	 /* bits 12..15: domain stalled? */
#define IPIPE_TFLG_DOMSTATE_BITS    3

#define IPIPE_TFLG_DOMAIN_STALLED(point, n) \
	(point->flags & (1 << (n + IPIPE_TFLG_DOMSTATE_SHIFT)))
#define IPIPE_TFLG_CURRENT_DOMAIN(point) \
	((point->flags & IPIPE_TFLG_CURRDOM_MASK) >> IPIPE_TFLG_CURRDOM_SHIFT)

struct ipipe_trace_point {
	short type;
	short flags;
	unsigned long eip;
	unsigned long parent_eip;
	unsigned long v;
	unsigned long long timestamp;
};

struct ipipe_trace_path {
	volatile int flags;
	int dump_lock; /* separated from flags due to cross-cpu access */
	int trace_pos; /* next point to fill */
	int begin, end; /* finalised path begin and end */
	int post_trace; /* non-zero when in post-trace phase */
	unsigned long long length; /* max path length in cycles */
	unsigned long nmi_saved_eip; /* for deferred requests from NMIs */
	unsigned long nmi_saved_parent_eip;
	unsigned long nmi_saved_v;
	struct ipipe_trace_point point[IPIPE_TRACE_POINTS];
} ____cacheline_aligned_in_smp;

enum ipipe_trace_type
{
	IPIPE_TRACE_FUNC = 0,
	IPIPE_TRACE_BEGIN,
	IPIPE_TRACE_END,
	IPIPE_TRACE_FREEZE,
	IPIPE_TRACE_SPECIAL,
	IPIPE_TRACE_PID,
	IPIPE_TRACE_EVENT,
};

#define IPIPE_TYPE_MASK		    0x0007
#define IPIPE_TYPE_BITS		    3

#ifdef CONFIG_IPIPE_TRACE_VMALLOC
static DEFINE_PER_CPU(struct ipipe_trace_path *, trace_path);
#else /* !CONFIG_IPIPE_TRACE_VMALLOC */
static DEFINE_PER_CPU(struct ipipe_trace_path, trace_path[IPIPE_TRACE_PATHS]) =
	{ [0 ... IPIPE_TRACE_PATHS-1] = { .begin = -1, .end = -1 } };
#endif /* CONFIG_IPIPE_TRACE_VMALLOC */

int ipipe_trace_enable = 0;

static DEFINE_PER_CPU(int, active_path) = { IPIPE_DEFAULT_ACTIVE };
static DEFINE_PER_CPU(int, max_path) = { IPIPE_DEFAULT_MAX };
static DEFINE_PER_CPU(int, frozen_path) = { IPIPE_DEFAULT_FROZEN };
static IPIPE_DEFINE_SPINLOCK(global_path_lock);
static int pre_trace = IPIPE_DEFAULT_PRE_TRACE;
static int post_trace = IPIPE_DEFAULT_POST_TRACE;
static int back_trace = IPIPE_DEFAULT_BACK_TRACE;
static int verbose_trace = 1;
static unsigned long trace_overhead;

static unsigned long trigger_begin;
static unsigned long trigger_end;

static DEFINE_MUTEX(out_mutex);
static struct ipipe_trace_path *print_path;
#ifdef CONFIG_IPIPE_TRACE_PANIC
static struct ipipe_trace_path *panic_path;
#endif /* CONFIG_IPIPE_TRACE_PANIC */
static int print_pre_trace;
static int print_post_trace;


static long __ipipe_signed_tsc2us(long long tsc);
static void
__ipipe_trace_point_type(char *buf, struct ipipe_trace_point *point);
static void __ipipe_print_symname(struct seq_file *m, unsigned long eip);

static inline void store_states(struct ipipe_domain *ipd,
				struct ipipe_trace_point *point, int pos)
{
	if (test_bit(IPIPE_STALL_FLAG, &ipipe_this_cpu_context(ipd)->status))
		point->flags |= 1 << (pos + IPIPE_TFLG_DOMSTATE_SHIFT);

	if (ipd == __ipipe_current_domain)
		point->flags |= pos << IPIPE_TFLG_CURRDOM_SHIFT;
}

static notrace void
__ipipe_store_domain_states(struct ipipe_trace_point *point)
{
	store_states(ipipe_root_domain, point, 0);
	if (ipipe_head_domain != ipipe_root_domain)
		store_states(ipipe_head_domain, point, 1);
}

static notrace int __ipipe_get_free_trace_path(int old, int cpu)
{
	int new_active = old;
	struct ipipe_trace_path *tp;

	do {
		if (++new_active == IPIPE_TRACE_PATHS)
			new_active = 0;
		tp = &per_cpu(trace_path, cpu)[new_active];
	} while (new_active == per_cpu(max_path, cpu) ||
		 new_active == per_cpu(frozen_path, cpu) ||
		 tp->dump_lock);

	return new_active;
}

static notrace void
__ipipe_migrate_pre_trace(struct ipipe_trace_path *new_tp,
			  struct ipipe_trace_path *old_tp, int old_pos)
{
	int i;

	new_tp->trace_pos = pre_trace+1;

	for (i = new_tp->trace_pos; i > 0; i--)
		memcpy(&new_tp->point[WRAP_POINT_NO(new_tp->trace_pos-i)],
		       &old_tp->point[WRAP_POINT_NO(old_pos-i)],
		       sizeof(struct ipipe_trace_point));

	/* mark the end (i.e. the point before point[0]) invalid */
	new_tp->point[IPIPE_TRACE_POINTS-1].eip = 0;
}

static notrace struct ipipe_trace_path *
__ipipe_trace_end(int cpu, struct ipipe_trace_path *tp, int pos)
{
	struct ipipe_trace_path *old_tp = tp;
	long active = per_cpu(active_path, cpu);
	unsigned long long length;

	/* do we have a new worst case? */
	length = tp->point[tp->end].timestamp -
		 tp->point[tp->begin].timestamp;
	if (length > per_cpu(trace_path, cpu)[per_cpu(max_path, cpu)].length) {
		/* we need protection here against other cpus trying
		   to start a proc dump */
		raw_spin_lock(&global_path_lock);

		/* active path holds new worst case */
		tp->length = length;
		per_cpu(max_path, cpu) = active;

		/* find next unused trace path */
		active = __ipipe_get_free_trace_path(active, cpu);

		raw_spin_unlock(&global_path_lock);

		tp = &per_cpu(trace_path, cpu)[active];

		/* migrate last entries for pre-tracing */
		__ipipe_migrate_pre_trace(tp, old_tp, pos);
	}

	return tp;
}

static notrace struct ipipe_trace_path *
__ipipe_trace_freeze(int cpu, struct ipipe_trace_path *tp, int pos)
{
	struct ipipe_trace_path *old_tp = tp;
	long active = per_cpu(active_path, cpu);
	int n;

	/* frozen paths have no core (begin=end) */
	tp->begin = tp->end;

	/* we need protection here against other cpus trying
	 * to set their frozen path or to start a proc dump */
	raw_spin_lock(&global_path_lock);

	per_cpu(frozen_path, cpu) = active;

	/* find next unused trace path */
	active = __ipipe_get_free_trace_path(active, cpu);

	/* check if this is the first frozen path */
	for_each_possible_cpu(n) {
		if (n != cpu &&
		    per_cpu(trace_path, n)[per_cpu(frozen_path, n)].end >= 0)
			tp->end = -1;
	}

	raw_spin_unlock(&global_path_lock);

	tp = &per_cpu(trace_path, cpu)[active];

	/* migrate last entries for pre-tracing */
	__ipipe_migrate_pre_trace(tp, old_tp, pos);

	return tp;
}

void notrace
__ipipe_trace(enum ipipe_trace_type type, unsigned long eip,
	      unsigned long parent_eip, unsigned long v)
{
	struct ipipe_trace_path *tp, *old_tp;
	int pos, next_pos, begin;
	struct ipipe_trace_point *point;
	unsigned long flags;
	int cpu;

	flags = hard_local_irq_save_notrace();

	cpu = ipipe_processor_id();
 restart:
	tp = old_tp = &per_cpu(trace_path, cpu)[per_cpu(active_path, cpu)];

	/* here starts a race window with NMIs - catched below */

	/* check for NMI recursion */
	if (unlikely(tp->flags & IPIPE_TFLG_NMI_LOCK)) {
		tp->flags |= IPIPE_TFLG_NMI_HIT;

		/* first freeze request from NMI context? */
		if ((type == IPIPE_TRACE_FREEZE) &&
		    !(tp->flags & IPIPE_TFLG_NMI_FREEZE_REQ)) {
			/* save arguments and mark deferred freezing */
			tp->flags |= IPIPE_TFLG_NMI_FREEZE_REQ;
			tp->nmi_saved_eip = eip;
			tp->nmi_saved_parent_eip = parent_eip;
			tp->nmi_saved_v = v;
		}
		return; /* no need for restoring flags inside IRQ */
	}

	/* clear NMI events and set lock (atomically per cpu) */
	tp->flags = (tp->flags & ~(IPIPE_TFLG_NMI_HIT |
				   IPIPE_TFLG_NMI_FREEZE_REQ))
			       | IPIPE_TFLG_NMI_LOCK;

	/* check active_path again - some nasty NMI may have switched
	 * it meanwhile */
	if (unlikely(tp !=
		     &per_cpu(trace_path, cpu)[per_cpu(active_path, cpu)])) {
		/* release lock on wrong path and restart */
		tp->flags &= ~IPIPE_TFLG_NMI_LOCK;

		/* there is no chance that the NMI got deferred
		 * => no need to check for pending freeze requests */
		goto restart;
	}

	/* get the point buffer */
	pos = tp->trace_pos;
	point = &tp->point[pos];

	/* store all trace point data */
	point->type = type;
	point->flags = hard_irqs_disabled_flags(flags) ? IPIPE_TFLG_HWIRQ_OFF : 0;
	point->eip = eip;
	point->parent_eip = parent_eip;
	point->v = v;
	ipipe_read_tsc(point->timestamp);

	__ipipe_store_domain_states(point);

	/* forward to next point buffer */
	next_pos = WRAP_POINT_NO(pos+1);
	tp->trace_pos = next_pos;

	/* only mark beginning if we haven't started yet */
	begin = tp->begin;
	if (unlikely(type == IPIPE_TRACE_BEGIN) && (begin < 0))
		tp->begin = pos;

	/* end of critical path, start post-trace if not already started */
	if (unlikely(type == IPIPE_TRACE_END) &&
	    (begin >= 0) && !tp->post_trace)
		tp->post_trace = post_trace + 1;

	/* freeze only if the slot is free and we are not already freezing */
	if ((unlikely(type == IPIPE_TRACE_FREEZE) ||
	     (unlikely(eip >= trigger_begin && eip <= trigger_end) &&
	     type == IPIPE_TRACE_FUNC)) &&
	    per_cpu(trace_path, cpu)[per_cpu(frozen_path, cpu)].begin < 0 &&
	    !(tp->flags & IPIPE_TFLG_FREEZING)) {
		tp->post_trace = post_trace + 1;
		tp->flags |= IPIPE_TFLG_FREEZING;
	}

	/* enforce end of trace in case of overflow */
	if (unlikely(WRAP_POINT_NO(next_pos + 1) == begin)) {
		tp->end = pos;
		goto enforce_end;
	}

	/* stop tracing this path if we are in post-trace and
	 *  a) that phase is over now or
	 *  b) a new TRACE_BEGIN came in but we are not freezing this path */
	if (unlikely((tp->post_trace > 0) && ((--tp->post_trace == 0) ||
		     ((type == IPIPE_TRACE_BEGIN) &&
		      !(tp->flags & IPIPE_TFLG_FREEZING))))) {
		/* store the path's end (i.e. excluding post-trace) */
		tp->end = WRAP_POINT_NO(pos - post_trace + tp->post_trace);

 enforce_end:
		if (tp->flags & IPIPE_TFLG_FREEZING)
			tp = __ipipe_trace_freeze(cpu, tp, pos);
		else
			tp = __ipipe_trace_end(cpu, tp, pos);

		/* reset the active path, maybe already start a new one */
		tp->begin = (type == IPIPE_TRACE_BEGIN) ?
			WRAP_POINT_NO(tp->trace_pos - 1) : -1;
		tp->end = -1;
		tp->post_trace = 0;
		tp->flags = 0;

		/* update active_path not earlier to avoid races with NMIs */
		per_cpu(active_path, cpu) = tp - per_cpu(trace_path, cpu);
	}

	/* we still have old_tp and point,
	 * let's reset NMI lock and check for catches */
	old_tp->flags &= ~IPIPE_TFLG_NMI_LOCK;
	if (unlikely(old_tp->flags & IPIPE_TFLG_NMI_HIT)) {
		/* well, this late tagging may not immediately be visible for
		 * other cpus already dumping this path - a minor issue */
		point->flags |= IPIPE_TFLG_NMI_HIT;

		/* handle deferred freezing from NMI context */
		if (old_tp->flags & IPIPE_TFLG_NMI_FREEZE_REQ)
			__ipipe_trace(IPIPE_TRACE_FREEZE, old_tp->nmi_saved_eip,
				      old_tp->nmi_saved_parent_eip,
				      old_tp->nmi_saved_v);
	}

	hard_local_irq_restore_notrace(flags);
}

static unsigned long __ipipe_global_path_lock(void)
{
	unsigned long flags;
	int cpu;
	struct ipipe_trace_path *tp;

	raw_spin_lock_irqsave(&global_path_lock, flags);

	cpu = ipipe_processor_id();
 restart:
	tp = &per_cpu(trace_path, cpu)[per_cpu(active_path, cpu)];

	/* here is small race window with NMIs - catched below */

	/* clear NMI events and set lock (atomically per cpu) */
	tp->flags = (tp->flags & ~(IPIPE_TFLG_NMI_HIT |
				   IPIPE_TFLG_NMI_FREEZE_REQ))
			       | IPIPE_TFLG_NMI_LOCK;

	/* check active_path again - some nasty NMI may have switched
	 * it meanwhile */
	if (tp != &per_cpu(trace_path, cpu)[per_cpu(active_path, cpu)]) {
		/* release lock on wrong path and restart */
		tp->flags &= ~IPIPE_TFLG_NMI_LOCK;

		/* there is no chance that the NMI got deferred
		 * => no need to check for pending freeze requests */
		goto restart;
	}

	return flags;
}

static void __ipipe_global_path_unlock(unsigned long flags)
{
	int cpu;
	struct ipipe_trace_path *tp;

	/* release spinlock first - it's not involved in the NMI issue */
	__ipipe_spin_unlock_irqbegin(&global_path_lock);

	cpu = ipipe_processor_id();
	tp = &per_cpu(trace_path, cpu)[per_cpu(active_path, cpu)];

	tp->flags &= ~IPIPE_TFLG_NMI_LOCK;

	/* handle deferred freezing from NMI context */
	if (tp->flags & IPIPE_TFLG_NMI_FREEZE_REQ)
		__ipipe_trace(IPIPE_TRACE_FREEZE, tp->nmi_saved_eip,
			      tp->nmi_saved_parent_eip, tp->nmi_saved_v);

	/* See __ipipe_spin_lock_irqsave() and friends. */
	__ipipe_spin_unlock_irqcomplete(flags);
}

void notrace asmlinkage
ipipe_trace_asm(enum ipipe_trace_type type, unsigned long eip,
		unsigned long parent_eip, unsigned long v)
{
	if (!ipipe_trace_enable)
		return;
	__ipipe_trace(type, eip, parent_eip, v);
}

void notrace ipipe_trace_begin(unsigned long v)
{
	if (!ipipe_trace_enable)
		return;
	__ipipe_trace(IPIPE_TRACE_BEGIN, __BUILTIN_RETURN_ADDRESS0,
		      __BUILTIN_RETURN_ADDRESS1, v);
}
EXPORT_SYMBOL_GPL(ipipe_trace_begin);

void notrace ipipe_trace_end(unsigned long v)
{
	if (!ipipe_trace_enable)
		return;
	__ipipe_trace(IPIPE_TRACE_END, __BUILTIN_RETURN_ADDRESS0,
		      __BUILTIN_RETURN_ADDRESS1, v);
}
EXPORT_SYMBOL_GPL(ipipe_trace_end);

void notrace ipipe_trace_irqbegin(int irq, struct pt_regs *regs)
{
	if (!ipipe_trace_enable)
		return;
	__ipipe_trace(IPIPE_TRACE_BEGIN, regs->ip,
		      __BUILTIN_RETURN_ADDRESS1, irq);
}
EXPORT_SYMBOL_GPL(ipipe_trace_irqbegin);

void notrace ipipe_trace_irqend(int irq, struct pt_regs *regs)
{
	if (!ipipe_trace_enable)
		return;
	__ipipe_trace(IPIPE_TRACE_END, regs->ip,
		      __BUILTIN_RETURN_ADDRESS1, irq);
}
EXPORT_SYMBOL_GPL(ipipe_trace_irqend);

void notrace ipipe_trace_freeze(unsigned long v)
{
	if (!ipipe_trace_enable)
		return;
	__ipipe_trace(IPIPE_TRACE_FREEZE, __BUILTIN_RETURN_ADDRESS0,
		      __BUILTIN_RETURN_ADDRESS1, v);
}
EXPORT_SYMBOL_GPL(ipipe_trace_freeze);

void notrace ipipe_trace_special(unsigned char id, unsigned long v)
{
	if (!ipipe_trace_enable)
		return;
	__ipipe_trace(IPIPE_TRACE_SPECIAL | (id << IPIPE_TYPE_BITS),
		      __BUILTIN_RETURN_ADDRESS0,
		      __BUILTIN_RETURN_ADDRESS1, v);
}
EXPORT_SYMBOL_GPL(ipipe_trace_special);

void notrace ipipe_trace_pid(pid_t pid, short prio)
{
	if (!ipipe_trace_enable)
		return;
	__ipipe_trace(IPIPE_TRACE_PID | (prio << IPIPE_TYPE_BITS),
		      __BUILTIN_RETURN_ADDRESS0,
		      __BUILTIN_RETURN_ADDRESS1, pid);
}
EXPORT_SYMBOL_GPL(ipipe_trace_pid);

void notrace ipipe_trace_event(unsigned char id, unsigned long delay_tsc)
{
	if (!ipipe_trace_enable)
		return;
	__ipipe_trace(IPIPE_TRACE_EVENT | (id << IPIPE_TYPE_BITS),
		      __BUILTIN_RETURN_ADDRESS0,
		      __BUILTIN_RETURN_ADDRESS1, delay_tsc);
}
EXPORT_SYMBOL_GPL(ipipe_trace_event);

int ipipe_trace_max_reset(void)
{
	int cpu;
	unsigned long flags;
	struct ipipe_trace_path *path;
	int ret = 0;

	flags = __ipipe_global_path_lock();

	for_each_possible_cpu(cpu) {
		path = &per_cpu(trace_path, cpu)[per_cpu(max_path, cpu)];

		if (path->dump_lock) {
			ret = -EBUSY;
			break;
		}

		path->begin	= -1;
		path->end	= -1;
		path->trace_pos = 0;
		path->length	= 0;
	}

	__ipipe_global_path_unlock(flags);

	return ret;
}
EXPORT_SYMBOL_GPL(ipipe_trace_max_reset);

int ipipe_trace_frozen_reset(void)
{
	int cpu;
	unsigned long flags;
	struct ipipe_trace_path *path;
	int ret = 0;

	flags = __ipipe_global_path_lock();

	for_each_online_cpu(cpu) {
		path = &per_cpu(trace_path, cpu)[per_cpu(frozen_path, cpu)];

		if (path->dump_lock) {
			ret = -EBUSY;
			break;
		}

		path->begin = -1;
		path->end = -1;
		path->trace_pos = 0;
		path->length	= 0;
	}

	__ipipe_global_path_unlock(flags);

	return ret;
}
EXPORT_SYMBOL_GPL(ipipe_trace_frozen_reset);

static void
__ipipe_get_task_info(char *task_info, struct ipipe_trace_point *point,
		      int trylock)
{
	struct task_struct *task = NULL;
	char buf[8];
	int i;
	int locked = 1;

	if (trylock) {
		if (!read_trylock(&tasklist_lock))
			locked = 0;
	} else
		read_lock(&tasklist_lock);

	if (locked)
		task = find_task_by_pid_ns((pid_t)point->v, &init_pid_ns);

	if (task)
		strncpy(task_info, task->comm, 11);
	else
		strcpy(task_info, "-<?>-");

	if (locked)
		read_unlock(&tasklist_lock);

	for (i = strlen(task_info); i < 11; i++)
		task_info[i] = ' ';

	sprintf(buf, " %d ", point->type >> IPIPE_TYPE_BITS);
	strcpy(task_info + (11 - strlen(buf)), buf);
}

static void
__ipipe_get_event_date(char *buf,struct ipipe_trace_path *path,
		       struct ipipe_trace_point *point)
{
	long time;
	int type;

	time = __ipipe_signed_tsc2us(point->timestamp -
				     path->point[path->begin].timestamp + point->v);
	type = point->type >> IPIPE_TYPE_BITS;

	if (type == 0)
		/*
		 * Event type #0 is predefined, stands for the next
		 * timer tick.
		 */
		sprintf(buf, "tick@%-6ld", time);
	else
		sprintf(buf, "%3d@%-7ld", type, time);
}

#ifdef CONFIG_IPIPE_TRACE_PANIC

void ipipe_trace_panic_freeze(void)
{
	unsigned long flags;
	int cpu;

	if (!ipipe_trace_enable)
		return;

	ipipe_trace_enable = 0;
	flags = hard_local_irq_save_notrace();

	cpu = ipipe_processor_id();

	panic_path = &per_cpu(trace_path, cpu)[per_cpu(active_path, cpu)];

	hard_local_irq_restore(flags);
}
EXPORT_SYMBOL_GPL(ipipe_trace_panic_freeze);

void ipipe_trace_panic_dump(void)
{
	int cnt = back_trace;
	int start, pos;
	char buf[16];

	if (!panic_path)
		return;

	ipipe_context_check_off();

	printk("I-pipe tracer log (%d points):\n", cnt);

	start = pos = WRAP_POINT_NO(panic_path->trace_pos-1);

	while (cnt-- > 0) {
		struct ipipe_trace_point *point = &panic_path->point[pos];
		long time;
		char info[16];
		int i;

		printk(" %c",
		       (point->flags & IPIPE_TFLG_HWIRQ_OFF) ? '|' : ' ');

		for (i = IPIPE_TFLG_DOMSTATE_BITS; i >= 0; i--)
			printk("%c",
			       (IPIPE_TFLG_CURRENT_DOMAIN(point) == i) ?
				(IPIPE_TFLG_DOMAIN_STALLED(point, i) ?
					'#' : '+') :
				(IPIPE_TFLG_DOMAIN_STALLED(point, i) ?
					'*' : ' '));

		if (!point->eip)
			printk("-<invalid>-\n");
		else {
			__ipipe_trace_point_type(buf, point);
			printk("%s", buf);

			switch (point->type & IPIPE_TYPE_MASK) {
				case IPIPE_TRACE_FUNC:
					printk("	   ");
					break;

				case IPIPE_TRACE_PID:
					__ipipe_get_task_info(info,
							      point, 1);
					printk("%s", info);
					break;

				case IPIPE_TRACE_EVENT:
					__ipipe_get_event_date(info,
							       panic_path, point);
					printk("%s", info);
					break;

				default:
					printk("0x%08lx ", point->v);
			}

			time = __ipipe_signed_tsc2us(point->timestamp -
				panic_path->point[start].timestamp);
			printk(" %5ld ", time);

			__ipipe_print_symname(NULL, point->eip);
			printk(" (");
			__ipipe_print_symname(NULL, point->parent_eip);
			printk(")\n");
		}
		pos = WRAP_POINT_NO(pos - 1);
	}

	panic_path = NULL;
}
EXPORT_SYMBOL_GPL(ipipe_trace_panic_dump);

#endif /* CONFIG_IPIPE_TRACE_PANIC */


/* --- /proc output --- */

static notrace int __ipipe_in_critical_trpath(long point_no)
{
	return ((WRAP_POINT_NO(point_no-print_path->begin) <
		 WRAP_POINT_NO(print_path->end-print_path->begin)) ||
		((print_path->end == print_path->begin) &&
		 (WRAP_POINT_NO(point_no-print_path->end) >
		  print_post_trace)));
}

static long __ipipe_signed_tsc2us(long long tsc)
{
	unsigned long long abs_tsc;
	long us;

	if (!__ipipe_hrclock_ok())
		return 0;

	/* ipipe_tsc2us works on unsigned => handle sign separately */
	abs_tsc = (tsc >= 0) ? tsc : -tsc;
	us = ipipe_tsc2us(abs_tsc);
	if (tsc < 0)
		return -us;
	else
		return us;
}

static void
__ipipe_trace_point_type(char *buf, struct ipipe_trace_point *point)
{
	switch (point->type & IPIPE_TYPE_MASK) {
		case IPIPE_TRACE_FUNC:
			strcpy(buf, "func    ");
			break;

		case IPIPE_TRACE_BEGIN:
			strcpy(buf, "begin   ");
			break;

		case IPIPE_TRACE_END:
			strcpy(buf, "end     ");
			break;

		case IPIPE_TRACE_FREEZE:
			strcpy(buf, "freeze  ");
			break;

		case IPIPE_TRACE_SPECIAL:
			sprintf(buf, "(0x%02x)	",
				point->type >> IPIPE_TYPE_BITS);
			break;

		case IPIPE_TRACE_PID:
			sprintf(buf, "[%5d] ", (pid_t)point->v);
			break;

		case IPIPE_TRACE_EVENT:
			sprintf(buf, "event   ");
			break;
	}
}

static void
__ipipe_print_pathmark(struct seq_file *m, struct ipipe_trace_point *point)
{
	char mark = ' ';
	int point_no = point - print_path->point;
	int i;

	if (print_path->end == point_no)
		mark = '<';
	else if (print_path->begin == point_no)
		mark = '>';
	else if (__ipipe_in_critical_trpath(point_no))
		mark = ':';
	seq_printf(m, "%c%c", mark,
		   (point->flags & IPIPE_TFLG_HWIRQ_OFF) ? '|' : ' ');

	if (!verbose_trace)
		return;

	for (i = IPIPE_TFLG_DOMSTATE_BITS; i >= 0; i--)
		seq_printf(m, "%c",
			(IPIPE_TFLG_CURRENT_DOMAIN(point) == i) ?
			    (IPIPE_TFLG_DOMAIN_STALLED(point, i) ?
				'#' : '+') :
			(IPIPE_TFLG_DOMAIN_STALLED(point, i) ? '*' : ' '));
}

static void
__ipipe_print_delay(struct seq_file *m, struct ipipe_trace_point *point)
{
	unsigned long delay = 0;
	int next;
	char *mark = "	";

	next = WRAP_POINT_NO(point+1 - print_path->point);

	if (next != print_path->trace_pos)
		delay = ipipe_tsc2ns(print_path->point[next].timestamp -
				     point->timestamp);

	if (__ipipe_in_critical_trpath(point - print_path->point)) {
		if (delay > IPIPE_DELAY_WARN)
			mark = "! ";
		else if (delay > IPIPE_DELAY_NOTE)
			mark = "+ ";
	}
	seq_puts(m, mark);

	if (verbose_trace)
		seq_printf(m, "%3lu.%03lu%c ", delay/1000, delay%1000,
			   (point->flags & IPIPE_TFLG_NMI_HIT) ? 'N' : ' ');
	else
		seq_puts(m, " ");
}

static void __ipipe_print_symname(struct seq_file *m, unsigned long eip)
{
	char namebuf[KSYM_NAME_LEN+1];
	unsigned long size, offset;
	const char *sym_name;
	char *modname;

	sym_name = kallsyms_lookup(eip, &size, &offset, &modname, namebuf);

#ifdef CONFIG_IPIPE_TRACE_PANIC
	if (!m) {
		/* panic dump */
		if (sym_name) {
			printk("%s+0x%lx", sym_name, offset);
			if (modname)
				printk(" [%s]", modname);
		} else
			printk("<%08lx>", eip);
	} else
#endif /* CONFIG_IPIPE_TRACE_PANIC */
	{
		if (sym_name) {
			if (verbose_trace) {
				seq_printf(m, "%s+0x%lx", sym_name, offset);
				if (modname)
					seq_printf(m, " [%s]", modname);
			} else
				seq_puts(m, sym_name);
		} else
			seq_printf(m, "<%08lx>", eip);
	}
}

static void __ipipe_print_headline(struct seq_file *m)
{
	const char *name[2];

	seq_printf(m, "Calibrated minimum trace-point overhead: %lu.%03lu "
		   "us\n\n", trace_overhead/1000, trace_overhead%1000);

	if (verbose_trace) {
		name[0] = ipipe_root_domain->name;
		if (ipipe_head_domain != ipipe_root_domain)
			name[1] = ipipe_head_domain->name;
		else
			name[1] = "<unused>";

		seq_printf(m,
			   " +----- Hard IRQs ('|': locked)\n"
			   " |+-- %s\n"
			   " ||+- %s%s\n"
			   " |||			  +---------- "
			       "Delay flag ('+': > %d us, '!': > %d us)\n"
			   " |||			  |	   +- "
			       "NMI noise ('N')\n"
			   " |||			  |	   |\n"
			   "	  Type	  User Val.   Time    Delay  Function "
			       "(Parent)\n",
			   name[1], name[0],
			   " ('*': domain stalled, '+': current, "
			   "'#': current+stalled)",
			   IPIPE_DELAY_NOTE/1000, IPIPE_DELAY_WARN/1000);
	} else
		seq_printf(m,
			   " +--------------- Hard IRQs ('|': locked)\n"
			   " |		   +- Delay flag "
			       "('+': > %d us, '!': > %d us)\n"
			   " |		   |\n"
			   "  Type     Time   Function (Parent)\n",
			   IPIPE_DELAY_NOTE/1000, IPIPE_DELAY_WARN/1000);
}

static void *__ipipe_max_prtrace_start(struct seq_file *m, loff_t *pos)
{
	loff_t n = *pos;

	mutex_lock(&out_mutex);

	if (!n) {
		struct ipipe_trace_path *tp;
		unsigned long length_usecs;
		int points, cpu;
		unsigned long flags;

		/* protect against max_path/frozen_path updates while we
		 * haven't locked our target path, also avoid recursively
		 * taking global_path_lock from NMI context */
		flags = __ipipe_global_path_lock();

		/* find the longest of all per-cpu paths */
		print_path = NULL;
		for_each_online_cpu(cpu) {
			tp = &per_cpu(trace_path, cpu)[per_cpu(max_path, cpu)];
			if ((print_path == NULL) ||
			    (tp->length > print_path->length)) {
				print_path = tp;
				break;
			}
		}
		print_path->dump_lock = 1;

		__ipipe_global_path_unlock(flags);

		if (!__ipipe_hrclock_ok()) {
			seq_printf(m, "No hrclock available, dumping traces disabled\n");
			return NULL;
		}

		/* does this path actually contain data? */
		if (print_path->end == print_path->begin)
			return NULL;

		/* number of points inside the critical path */
		points = WRAP_POINT_NO(print_path->end-print_path->begin+1);

		/* pre- and post-tracing length, post-trace length was frozen
		   in __ipipe_trace, pre-trace may have to be reduced due to
		   buffer overrun */
		print_pre_trace	 = pre_trace;
		print_post_trace = WRAP_POINT_NO(print_path->trace_pos -
						 print_path->end - 1);
		if (points+pre_trace+print_post_trace > IPIPE_TRACE_POINTS - 1)
			print_pre_trace = IPIPE_TRACE_POINTS - 1 - points -
				print_post_trace;

		length_usecs = ipipe_tsc2us(print_path->length);
		seq_printf(m, "I-pipe worst-case tracing service on %s/ipipe release #%d\n"
			   "-------------------------------------------------------------\n",
			UTS_RELEASE, IPIPE_CORE_RELEASE);
		seq_printf(m, "CPU: %d, Begin: %lld cycles, Trace Points: "
			"%d (-%d/+%d), Length: %lu us\n",
			cpu, print_path->point[print_path->begin].timestamp,
			points, print_pre_trace, print_post_trace, length_usecs);
		__ipipe_print_headline(m);
	}

	/* check if we are inside the trace range */
	if (n >= WRAP_POINT_NO(print_path->end - print_path->begin + 1 +
			       print_pre_trace + print_post_trace))
		return NULL;

	/* return the next point to be shown */
	return &print_path->point[WRAP_POINT_NO(print_path->begin -
						print_pre_trace + n)];
}

static void *__ipipe_prtrace_next(struct seq_file *m, void *p, loff_t *pos)
{
	loff_t n = ++*pos;

	/* check if we are inside the trace range with the next entry */
	if (n >= WRAP_POINT_NO(print_path->end - print_path->begin + 1 +
			       print_pre_trace + print_post_trace))
		return NULL;

	/* return the next point to be shown */
	return &print_path->point[WRAP_POINT_NO(print_path->begin -
						print_pre_trace + *pos)];
}

static void __ipipe_prtrace_stop(struct seq_file *m, void *p)
{
	if (print_path)
		print_path->dump_lock = 0;
	mutex_unlock(&out_mutex);
}

static int __ipipe_prtrace_show(struct seq_file *m, void *p)
{
	long time;
	struct ipipe_trace_point *point = p;
	char buf[16];

	if (!point->eip) {
		seq_puts(m, "-<invalid>-\n");
		return 0;
	}

	__ipipe_print_pathmark(m, point);
	__ipipe_trace_point_type(buf, point);
	seq_puts(m, buf);
	if (verbose_trace)
		switch (point->type & IPIPE_TYPE_MASK) {
			case IPIPE_TRACE_FUNC:
				seq_puts(m, "           ");
				break;

			case IPIPE_TRACE_PID:
				__ipipe_get_task_info(buf, point, 0);
				seq_puts(m, buf);
				break;

			case IPIPE_TRACE_EVENT:
				__ipipe_get_event_date(buf, print_path, point);
				seq_puts(m, buf);
				break;

			default:
				seq_printf(m, "0x%08lx ", point->v);
		}

	time = __ipipe_signed_tsc2us(point->timestamp -
		print_path->point[print_path->begin].timestamp);
	seq_printf(m, "%5ld", time);

	__ipipe_print_delay(m, point);
	__ipipe_print_symname(m, point->eip);
	seq_puts(m, " (");
	__ipipe_print_symname(m, point->parent_eip);
	seq_puts(m, ")\n");

	return 0;
}

static struct seq_operations __ipipe_max_ptrace_ops = {
	.start = __ipipe_max_prtrace_start,
	.next  = __ipipe_prtrace_next,
	.stop  = __ipipe_prtrace_stop,
	.show  = __ipipe_prtrace_show
};

static int __ipipe_max_prtrace_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &__ipipe_max_ptrace_ops);
}

static ssize_t
__ipipe_max_reset(struct file *file, const char __user *pbuffer,
		  size_t count, loff_t *data)
{
	mutex_lock(&out_mutex);
	ipipe_trace_max_reset();
	mutex_unlock(&out_mutex);

	return count;
}

static const struct file_operations __ipipe_max_prtrace_fops = {
	.open	    = __ipipe_max_prtrace_open,
	.read	    = seq_read,
	.write	    = __ipipe_max_reset,
	.llseek	    = seq_lseek,
	.release    = seq_release,
};

static void *__ipipe_frozen_prtrace_start(struct seq_file *m, loff_t *pos)
{
	loff_t n = *pos;

	mutex_lock(&out_mutex);

	if (!n) {
		struct ipipe_trace_path *tp;
		int cpu;
		unsigned long flags;

		/* protect against max_path/frozen_path updates while we
		 * haven't locked our target path, also avoid recursively
		 * taking global_path_lock from NMI context */
		flags = __ipipe_global_path_lock();

		/* find the first of all per-cpu frozen paths */
		print_path = NULL;
		for_each_online_cpu(cpu) {
			tp = &per_cpu(trace_path, cpu)[per_cpu(frozen_path, cpu)];
			if (tp->end >= 0) {
				print_path = tp;
				break;
			}
		}
		if (print_path)
			print_path->dump_lock = 1;

		__ipipe_global_path_unlock(flags);

		if (!print_path)
			return NULL;

		if (!__ipipe_hrclock_ok()) {
			seq_printf(m, "No hrclock available, dumping traces disabled\n");
			return NULL;
		}

		/* back- and post-tracing length, post-trace length was frozen
		   in __ipipe_trace, back-trace may have to be reduced due to
		   buffer overrun */
		print_pre_trace	 = back_trace-1; /* substract freeze point */
		print_post_trace = WRAP_POINT_NO(print_path->trace_pos -
						 print_path->end - 1);
		if (1+pre_trace+print_post_trace > IPIPE_TRACE_POINTS - 1)
			print_pre_trace = IPIPE_TRACE_POINTS - 2 -
				print_post_trace;

		seq_printf(m, "I-pipe frozen back-tracing service on %s/ipipe release #%d\n"
			      "------------------------------------------------------------\n",
			   UTS_RELEASE, IPIPE_CORE_RELEASE);
		seq_printf(m, "CPU: %d, Freeze: %lld cycles, Trace Points: %d (+%d)\n",
			cpu, print_path->point[print_path->begin].timestamp,
			print_pre_trace+1, print_post_trace);
		__ipipe_print_headline(m);
	}

	/* check if we are inside the trace range */
	if (n >= print_pre_trace + 1 + print_post_trace)
		return NULL;

	/* return the next point to be shown */
	return &print_path->point[WRAP_POINT_NO(print_path->begin-
						print_pre_trace+n)];
}

static struct seq_operations __ipipe_frozen_ptrace_ops = {
	.start = __ipipe_frozen_prtrace_start,
	.next  = __ipipe_prtrace_next,
	.stop  = __ipipe_prtrace_stop,
	.show  = __ipipe_prtrace_show
};

static int __ipipe_frozen_prtrace_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &__ipipe_frozen_ptrace_ops);
}

static ssize_t
__ipipe_frozen_ctrl(struct file *file, const char __user *pbuffer,
		    size_t count, loff_t *data)
{
	char *end, buf[16];
	int val;
	int n;

	n = (count > sizeof(buf) - 1) ? sizeof(buf) - 1 : count;

	if (copy_from_user(buf, pbuffer, n))
		return -EFAULT;

	buf[n] = '\0';
	val = simple_strtol(buf, &end, 0);

	if (((*end != '\0') && !isspace(*end)) || (val < 0))
		return -EINVAL;

	mutex_lock(&out_mutex);
	ipipe_trace_frozen_reset();
	if (val > 0)
		ipipe_trace_freeze(-1);
	mutex_unlock(&out_mutex);

	return count;
}

static const struct file_operations __ipipe_frozen_prtrace_fops = {
	.open	    = __ipipe_frozen_prtrace_open,
	.read	    = seq_read,
	.write	    = __ipipe_frozen_ctrl,
	.llseek	    = seq_lseek,
	.release    = seq_release,
};

static int __ipipe_rd_proc_val(struct seq_file *p, void *data)
{
	seq_printf(p, "%u\n", *(int *)p->private);
	return 0;
}

static ssize_t
__ipipe_wr_proc_val(struct file *file, const char __user *buffer,
		    size_t count, loff_t *data)
{
	struct seq_file *p = file->private_data;
	char *end, buf[16];
	int val;
	int n;

	n = (count > sizeof(buf) - 1) ? sizeof(buf) - 1 : count;

	if (copy_from_user(buf, buffer, n))
		return -EFAULT;

	buf[n] = '\0';
	val = simple_strtol(buf, &end, 0);

	if (((*end != '\0') && !isspace(*end)) || (val < 0))
		return -EINVAL;

	mutex_lock(&out_mutex);
	*(int *)p->private = val;
	mutex_unlock(&out_mutex);

	return count;
}

static int __ipipe_rw_proc_val_open(struct inode *inode, struct file *file)
{
	return single_open(file, __ipipe_rd_proc_val, PDE_DATA(inode));
}

static const struct file_operations __ipipe_rw_proc_val_ops = {
	.open		= __ipipe_rw_proc_val_open,
	.read		= seq_read,
	.write		= __ipipe_wr_proc_val,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void __init
__ipipe_create_trace_proc_val(struct proc_dir_entry *trace_dir,
			      const char *name, int *value_ptr)
{
	proc_create_data(name, 0644, trace_dir, &__ipipe_rw_proc_val_ops,
			 value_ptr);
}

static int __ipipe_rd_trigger(struct seq_file *p, void *data)
{
	char str[KSYM_SYMBOL_LEN];

	if (trigger_begin) {
		sprint_symbol(str, trigger_begin);
		seq_printf(p, "%s\n", str);
	}
	return 0;
}

static ssize_t
__ipipe_wr_trigger(struct file *file, const char __user *buffer,
		   size_t count, loff_t *data)
{
	char buf[KSYM_SYMBOL_LEN];
	unsigned long begin, end;

	if (count > sizeof(buf) - 1)
		count = sizeof(buf) - 1;
	if (copy_from_user(buf, buffer, count))
		return -EFAULT;
	buf[count] = 0;
	if (buf[count-1] == '\n')
		buf[count-1] = 0;

	begin = kallsyms_lookup_name(buf);
	if (!begin || !kallsyms_lookup_size_offset(begin, &end, NULL))
		return -ENOENT;
	end += begin - 1;

	mutex_lock(&out_mutex);
	/* invalidate the current range before setting a new one */
	trigger_end = 0;
	wmb();
	ipipe_trace_frozen_reset();

	/* set new range */
	trigger_begin = begin;
	wmb();
	trigger_end = end;
	mutex_unlock(&out_mutex);

	return count;
}

static int __ipipe_rw_trigger_open(struct inode *inode, struct file *file)
{
	return single_open(file, __ipipe_rd_trigger, NULL);
}

static const struct file_operations __ipipe_rw_trigger_ops = {
	.open		= __ipipe_rw_trigger_open,
	.read		= seq_read,
	.write		= __ipipe_wr_trigger,
	.llseek		= seq_lseek,
	.release	= single_release,
};


#ifdef CONFIG_IPIPE_TRACE_MCOUNT
static void notrace
ipipe_trace_function(unsigned long ip, unsigned long parent_ip,
		     struct ftrace_ops *op, struct pt_regs *regs)
{
	if (!ipipe_trace_enable)
		return;
	__ipipe_trace(IPIPE_TRACE_FUNC, ip, parent_ip, 0);
}

static struct ftrace_ops ipipe_trace_ops = {
	.func = ipipe_trace_function,
	.flags = FTRACE_OPS_FL_IPIPE_EXCLUSIVE,
};

static ssize_t __ipipe_wr_enable(struct file *file, const char __user *buffer,
				 size_t count, loff_t *data)
{
	char *end, buf[16];
	int val;
	int n;

	n = (count > sizeof(buf) - 1) ? sizeof(buf) - 1 : count;

	if (copy_from_user(buf, buffer, n))
		return -EFAULT;

	buf[n] = '\0';
	val = simple_strtol(buf, &end, 0);

	if (((*end != '\0') && !isspace(*end)) || (val < 0))
		return -EINVAL;

	mutex_lock(&out_mutex);

	if (ipipe_trace_enable) {
		if (!val)
			unregister_ftrace_function(&ipipe_trace_ops);
	} else if (val)
		register_ftrace_function(&ipipe_trace_ops);

	ipipe_trace_enable = val;

	mutex_unlock(&out_mutex);

	return count;
}

static const struct file_operations __ipipe_rw_enable_ops = {
	.open		= __ipipe_rw_proc_val_open,
	.read		= seq_read,
	.write		= __ipipe_wr_enable,
	.llseek		= seq_lseek,
	.release	= single_release,
};
#endif /* CONFIG_IPIPE_TRACE_MCOUNT */

extern struct proc_dir_entry *ipipe_proc_root;

void __init __ipipe_tracer_hrclock_initialized(void)
{
	unsigned long long start, end, min = ULLONG_MAX;
	int i;

#ifdef CONFIG_IPIPE_TRACE_VMALLOC
	if (!per_cpu(trace_path, 0))
		return;
#endif
	/* Calculate minimum overhead of __ipipe_trace() */
	hard_local_irq_disable();
	for (i = 0; i < 100; i++) {
		ipipe_read_tsc(start);
		__ipipe_trace(IPIPE_TRACE_FUNC, __BUILTIN_RETURN_ADDRESS0,
			      __BUILTIN_RETURN_ADDRESS1, 0);
		ipipe_read_tsc(end);

		end -= start;
		if (end < min)
			min = end;
	}
	hard_local_irq_enable();
	trace_overhead = ipipe_tsc2ns(min);
}

void __init __ipipe_init_tracer(void)
{
	struct proc_dir_entry *trace_dir;
#ifdef CONFIG_IPIPE_TRACE_VMALLOC
	int cpu, path;
#endif /* CONFIG_IPIPE_TRACE_VMALLOC */

#ifdef CONFIG_IPIPE_TRACE_VMALLOC
	for_each_possible_cpu(cpu) {
		struct ipipe_trace_path *tp_buf;

		tp_buf = vmalloc_node(sizeof(struct ipipe_trace_path) *
				      IPIPE_TRACE_PATHS, cpu_to_node(cpu));
		if (!tp_buf) {
			pr_err("I-pipe: "
			       "insufficient memory for trace buffer.\n");
			return;
		}
		memset(tp_buf, 0,
		       sizeof(struct ipipe_trace_path) * IPIPE_TRACE_PATHS);
		for (path = 0; path < IPIPE_TRACE_PATHS; path++) {
			tp_buf[path].begin = -1;
			tp_buf[path].end   = -1;
		}
		per_cpu(trace_path, cpu) = tp_buf;
	}
#endif /* CONFIG_IPIPE_TRACE_VMALLOC */

	if (__ipipe_hrclock_ok() && !trace_overhead)
		__ipipe_tracer_hrclock_initialized();

#ifdef CONFIG_IPIPE_TRACE_ENABLE
	ipipe_trace_enable = 1;
#ifdef CONFIG_IPIPE_TRACE_MCOUNT
	ftrace_enabled = 1;
	register_ftrace_function(&ipipe_trace_ops);
#endif /* CONFIG_IPIPE_TRACE_MCOUNT */
#endif /* CONFIG_IPIPE_TRACE_ENABLE */

	trace_dir = proc_mkdir("trace", ipipe_proc_root);

	proc_create("max", 0644, trace_dir, &__ipipe_max_prtrace_fops);
	proc_create("frozen", 0644, trace_dir, &__ipipe_frozen_prtrace_fops);

	proc_create("trigger", 0644, trace_dir, &__ipipe_rw_trigger_ops);

	__ipipe_create_trace_proc_val(trace_dir, "pre_trace_points",
				      &pre_trace);
	__ipipe_create_trace_proc_val(trace_dir, "post_trace_points",
				      &post_trace);
	__ipipe_create_trace_proc_val(trace_dir, "back_trace_points",
				      &back_trace);
	__ipipe_create_trace_proc_val(trace_dir, "verbose",
				      &verbose_trace);
#ifdef CONFIG_IPIPE_TRACE_MCOUNT
	proc_create_data("enable", 0644, trace_dir, &__ipipe_rw_enable_ops,
			 &ipipe_trace_enable);
#else /* !CONFIG_IPIPE_TRACE_MCOUNT */
	__ipipe_create_trace_proc_val(trace_dir, "enable",
				      &ipipe_trace_enable);
#endif /* !CONFIG_IPIPE_TRACE_MCOUNT */
}
