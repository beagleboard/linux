/*
 * Copyright (C) 2008 Philippe Gerum <rpm@xenomai.org>.
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
#include <cobalt/kernel/sched.h>

static void xnsched_rt_init(struct xnsched *sched)
{
	xnsched_initq(&sched->rt.runnable);
}

static void xnsched_rt_requeue(struct xnthread *thread)
{
	/*
	 * Put back at same place: i.e. requeue to head of current
	 * priority group (i.e. LIFO, used for preemption handling).
	 */
	__xnsched_rt_requeue(thread);
}

static void xnsched_rt_enqueue(struct xnthread *thread)
{
	/*
	 * Enqueue for next pick: i.e. move to end of current priority
	 * group (i.e. FIFO).
	 */
	__xnsched_rt_enqueue(thread);
}

static void xnsched_rt_dequeue(struct xnthread *thread)
{
	/*
	 * Pull from the runnable thread queue.
	 */
	__xnsched_rt_dequeue(thread);
}

static void xnsched_rt_rotate(struct xnsched *sched,
			      const union xnsched_policy_param *p)
{
	struct xnthread *thread, *curr;

	if (xnsched_emptyq_p(&sched->rt.runnable))
		return;	/* No runnable thread in this class. */

	curr = sched->curr;

	if (p->rt.prio == XNSCHED_RUNPRIO)
		thread = curr;
	else {
		thread = xnsched_findq(&sched->rt.runnable, p->rt.prio);
		if (thread == NULL)
			return;
	}

	/*
	 * In case we picked the current thread, we have to make sure
	 * not to move it back to the runnable queue if it was blocked
	 * before we were called. The same goes if the current thread
	 * holds the scheduler lock.
	 */
	if (thread != curr ||
	    (!xnthread_test_state(curr, XNTHREAD_BLOCK_BITS) &&
	     curr->lock_count == 0))
		xnsched_putback(thread);
}

void xnsched_rt_tick(struct xnsched *sched)
{
	/*
	 * The round-robin time credit is only consumed by a running
	 * thread that neither holds the scheduler lock nor was
	 * blocked before entering this callback. As the time slice is
	 * exhausted for the running thread, move it back to the
	 * runnable queue at the end of its priority group.
	 */
	xnsched_putback(sched->curr);
}

void xnsched_rt_setparam(struct xnthread *thread,
			 const union xnsched_policy_param *p)
{
	__xnsched_rt_setparam(thread, p);
}

void xnsched_rt_getparam(struct xnthread *thread,
			 union xnsched_policy_param *p)
{
	__xnsched_rt_getparam(thread, p);
}

void xnsched_rt_trackprio(struct xnthread *thread,
			  const union xnsched_policy_param *p)
{
	__xnsched_rt_trackprio(thread, p);
}

#ifdef CONFIG_XENO_OPT_VFILE

struct xnvfile_directory sched_rt_vfroot;

struct vfile_sched_rt_priv {
	struct xnthread *curr;
};

struct vfile_sched_rt_data {
	int cpu;
	pid_t pid;
	char name[XNOBJECT_NAME_LEN];
	xnticks_t period;
	int cprio;
};

static struct xnvfile_snapshot_ops vfile_sched_rt_ops;

static struct xnvfile_snapshot vfile_sched_rt = {
	.privsz = sizeof(struct vfile_sched_rt_priv),
	.datasz = sizeof(struct vfile_sched_rt_data),
	.tag = &nkthreadlist_tag,
	.ops = &vfile_sched_rt_ops,
};

static int vfile_sched_rt_rewind(struct xnvfile_snapshot_iterator *it)
{
	struct vfile_sched_rt_priv *priv = xnvfile_iterator_priv(it);
	int nrthreads = xnsched_class_rt.nthreads;

	if (nrthreads == 0)
		return -ESRCH;

	priv->curr = list_first_entry(&nkthreadq, struct xnthread, glink);

	return nrthreads;
}

static int vfile_sched_rt_next(struct xnvfile_snapshot_iterator *it,
			       void *data)
{
	struct vfile_sched_rt_priv *priv = xnvfile_iterator_priv(it);
	struct vfile_sched_rt_data *p = data;
	struct xnthread *thread;

	if (priv->curr == NULL)
		return 0;	/* All done. */

	thread = priv->curr;
	if (list_is_last(&thread->glink, &nkthreadq))
		priv->curr = NULL;
	else
		priv->curr = list_next_entry(thread, glink);

	if (thread->base_class != &xnsched_class_rt ||
	    xnthread_test_state(thread, XNWEAK))
		return VFILE_SEQ_SKIP;

	p->cpu = xnsched_cpu(thread->sched);
	p->pid = xnthread_host_pid(thread);
	memcpy(p->name, thread->name, sizeof(p->name));
	p->cprio = thread->cprio;
	p->period = xnthread_get_period(thread);

	return 1;
}

static int vfile_sched_rt_show(struct xnvfile_snapshot_iterator *it,
			       void *data)
{
	struct vfile_sched_rt_data *p = data;
	char pribuf[16], ptbuf[16];

	if (p == NULL)
		xnvfile_printf(it, "%-3s  %-6s %-8s %-10s %s\n",
			       "CPU", "PID", "PRI", "PERIOD", "NAME");
	else {
		ksformat(pribuf, sizeof(pribuf), "%3d", p->cprio);
		xntimer_format_time(p->period, ptbuf, sizeof(ptbuf));
		xnvfile_printf(it, "%3u  %-6d %-8s %-10s %s\n",
			       p->cpu,
			       p->pid,
			       pribuf,
			       ptbuf,
			       p->name);
	}

	return 0;
}

static struct xnvfile_snapshot_ops vfile_sched_rt_ops = {
	.rewind = vfile_sched_rt_rewind,
	.next = vfile_sched_rt_next,
	.show = vfile_sched_rt_show,
};

static int xnsched_rt_init_vfile(struct xnsched_class *schedclass,
				 struct xnvfile_directory *vfroot)
{
	int ret;

	ret = xnvfile_init_dir(schedclass->name, &sched_rt_vfroot, vfroot);
	if (ret)
		return ret;

	return xnvfile_init_snapshot("threads", &vfile_sched_rt,
				     &sched_rt_vfroot);
}

static void xnsched_rt_cleanup_vfile(struct xnsched_class *schedclass)
{
	xnvfile_destroy_snapshot(&vfile_sched_rt);
	xnvfile_destroy_dir(&sched_rt_vfroot);
}

#endif /* CONFIG_XENO_OPT_VFILE */

struct xnsched_class xnsched_class_rt = {
	.sched_init		=	xnsched_rt_init,
	.sched_enqueue		=	xnsched_rt_enqueue,
	.sched_dequeue		=	xnsched_rt_dequeue,
	.sched_requeue		=	xnsched_rt_requeue,
	.sched_pick		=	xnsched_rt_pick,
	.sched_tick		=	xnsched_rt_tick,
	.sched_rotate		=	xnsched_rt_rotate,
	.sched_forget		=	NULL,
	.sched_kick		=	NULL,
	.sched_declare		=	NULL,
	.sched_setparam		=	xnsched_rt_setparam,
	.sched_trackprio	=	xnsched_rt_trackprio,
	.sched_getparam		=	xnsched_rt_getparam,
#ifdef CONFIG_XENO_OPT_VFILE
	.sched_init_vfile	=	xnsched_rt_init_vfile,
	.sched_cleanup_vfile	=	xnsched_rt_cleanup_vfile,
#endif
	.weight			=	XNSCHED_CLASS_WEIGHT(4),
	.policy			=	SCHED_FIFO,
	.name			=	"rt"
};
EXPORT_SYMBOL_GPL(xnsched_class_rt);
