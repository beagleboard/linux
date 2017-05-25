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
#include <cobalt/kernel/heap.h>
#include <cobalt/uapi/sched.h>

static void tp_schedule_next(struct xnsched_tp *tp)
{
	struct xnsched_tp_window *w;
	struct xnsched *sched;
	int p_next, ret;
	xnticks_t t;

	for (;;) {
		/*
		 * Switch to the next partition. Time holes in a
		 * global time frame are defined as partition windows
		 * assigned to part# -1, in which case the (always
		 * empty) idle queue will be polled for runnable
		 * threads.  Therefore, we may assume that a window
		 * begins immediately after the previous one ends,
		 * which simplifies the implementation a lot.
		 */
		w = &tp->gps->pwins[tp->wnext];
		p_next = w->w_part;
		tp->tps = p_next < 0 ? &tp->idle : &tp->partitions[p_next];

		/* Schedule tick to advance to the next window. */
		tp->wnext = (tp->wnext + 1) % tp->gps->pwin_nr;
		w = &tp->gps->pwins[tp->wnext];
		t = tp->tf_start + w->w_offset;

		ret = xntimer_start(&tp->tf_timer, t, XN_INFINITE, XN_ABSOLUTE);
		if (ret != -ETIMEDOUT)
			break;
		/*
		 * We are late, make sure to remain within the bounds
		 * of a valid time frame before advancing to the next
		 * window. Otherwise, fix up by advancing to the next
		 * time frame immediately.
		 */
		for (;;) {
			t = tp->tf_start + tp->gps->tf_duration;
			if (xnclock_read_monotonic(&nkclock) > t) {
				tp->tf_start = t;
				tp->wnext = 0;
			} else
				break;
		}
	}

	sched = container_of(tp, struct xnsched, tp);
	xnsched_set_resched(sched);
}

static void tp_tick_handler(struct xntimer *timer)
{
	struct xnsched_tp *tp = container_of(timer, struct xnsched_tp, tf_timer);
	/*
	 * Advance beginning date of time frame by a full period if we
	 * are processing the last window.
	 */
	if (tp->wnext + 1 == tp->gps->pwin_nr)
		tp->tf_start += tp->gps->tf_duration;

	tp_schedule_next(tp);
}

static void xnsched_tp_init(struct xnsched *sched)
{
	struct xnsched_tp *tp = &sched->tp;
	char timer_name[XNOBJECT_NAME_LEN];
	int n;

	for (n = 0; n < CONFIG_XENO_OPT_SCHED_TP_NRPART; n++)
		xnsched_initq(&tp->partitions[n].runnable);

	xnsched_initq(&tp->idle.runnable);

#ifdef CONFIG_SMP
	ksformat(timer_name, sizeof(timer_name), "[tp-tick/%u]", sched->cpu);
#else
	strcpy(timer_name, "[tp-tick]");
#endif
	tp->tps = NULL;
	tp->gps = NULL;
	INIT_LIST_HEAD(&tp->threads);
	xntimer_init(&tp->tf_timer, &nkclock, tp_tick_handler,
		     sched, XNTIMER_NOBLCK|XNTIMER_IGRAVITY);
	xntimer_set_sched(&tp->tf_timer, sched);
	xntimer_set_name(&tp->tf_timer, timer_name);
}

static void xnsched_tp_setparam(struct xnthread *thread,
				const union xnsched_policy_param *p)
{
	struct xnsched *sched = thread->sched;

	xnthread_clear_state(thread, XNWEAK);
	thread->tps = &sched->tp.partitions[p->tp.ptid];
	thread->cprio = p->tp.prio;
}

static void xnsched_tp_getparam(struct xnthread *thread,
				union xnsched_policy_param *p)
{
	p->tp.prio = thread->cprio;
	p->tp.ptid = thread->tps - thread->sched->tp.partitions;
}

static void xnsched_tp_trackprio(struct xnthread *thread,
				 const union xnsched_policy_param *p)
{
	/*
	 * The assigned partition never changes internally due to PIP
	 * (see xnsched_track_policy), since this would be pretty
	 * wrong with respect to TP scheduling: i.e. we may not allow
	 * a thread from another partition to consume CPU time from
	 * the current one, despite this would help enforcing PIP
	 * (*). In any case, introducing resource contention between
	 * threads that belong to different partitions is utterly
	 * wrong in the first place.  Only an explicit call to
	 * xnsched_set_policy() may change the partition assigned to a
	 * thread. For that reason, a policy reset action only boils
	 * down to reinstating the base priority.
	 *
	 * (*) However, we do allow threads from lower scheduling
	 * classes to consume CPU time from the current window as a
	 * result of a PIP boost, since this is aimed at speeding up
	 * the release of a synchronization object a TP thread needs.
	 */
	if (p) {
		/* We should never cross partition boundaries. */
		XENO_WARN_ON(COBALT,
			   thread->base_class == &xnsched_class_tp &&
			   thread->tps - thread->sched->tp.partitions != p->tp.ptid);
		thread->cprio = p->tp.prio;
	} else
		thread->cprio = thread->bprio;
}

static int xnsched_tp_declare(struct xnthread *thread,
			      const union xnsched_policy_param *p)
{
	struct xnsched *sched = thread->sched;
	struct xnsched_tp *tp = &sched->tp;

	if (tp->gps == NULL ||
	    p->tp.prio < XNSCHED_TP_MIN_PRIO ||
	    p->tp.prio > XNSCHED_TP_MAX_PRIO)
		return -EINVAL;

	list_add_tail(&thread->tp_link, &sched->tp.threads);

	return 0;
}

static void xnsched_tp_forget(struct xnthread *thread)
{
	list_del(&thread->tp_link);
	thread->tps = NULL;
}

static void xnsched_tp_enqueue(struct xnthread *thread)
{
	xnsched_addq_tail(&thread->tps->runnable, thread);
}

static void xnsched_tp_dequeue(struct xnthread *thread)
{
	xnsched_delq(&thread->tps->runnable, thread);
}

static void xnsched_tp_requeue(struct xnthread *thread)
{
	xnsched_addq(&thread->tps->runnable, thread);
}

static struct xnthread *xnsched_tp_pick(struct xnsched *sched)
{
	/* Never pick a thread if we don't schedule partitions. */
	if (!xntimer_running_p(&sched->tp.tf_timer))
		return NULL;

	return xnsched_getq(&sched->tp.tps->runnable);
}

static void xnsched_tp_migrate(struct xnthread *thread, struct xnsched *sched)
{
	union xnsched_policy_param param;
	/*
	 * Since our partition schedule is a per-scheduler property,
	 * it cannot apply to a thread that moves to another CPU
	 * anymore. So we upgrade that thread to the RT class when a
	 * CPU migration occurs. A subsequent call to
	 * xnsched_set_policy() may move it back to TP scheduling,
	 * with a partition assignment that fits the remote CPU's
	 * partition schedule.
	 */
	param.rt.prio = thread->cprio;
	xnsched_set_policy(thread, &xnsched_class_rt, &param);
}

void xnsched_tp_start_schedule(struct xnsched *sched)
{
	struct xnsched_tp *tp = &sched->tp;

	if (tp->gps == NULL)
		return;

	tp->wnext = 0;
	tp->tf_start = xnclock_read_monotonic(&nkclock);
	tp_schedule_next(tp);
}
EXPORT_SYMBOL_GPL(xnsched_tp_start_schedule);

void xnsched_tp_stop_schedule(struct xnsched *sched)
{
	struct xnsched_tp *tp = &sched->tp;

	if (tp->gps)
		xntimer_stop(&tp->tf_timer);
}
EXPORT_SYMBOL_GPL(xnsched_tp_stop_schedule);

struct xnsched_tp_schedule *
xnsched_tp_set_schedule(struct xnsched *sched,
			struct xnsched_tp_schedule *gps)
{
	struct xnsched_tp_schedule *old_gps;
	struct xnsched_tp *tp = &sched->tp;
	union xnsched_policy_param param;
	struct xnthread *thread, *tmp;

	XENO_BUG_ON(COBALT, gps != NULL &&
		   (gps->pwin_nr <= 0 || gps->pwins[0].w_offset != 0));

	xnsched_tp_stop_schedule(sched);

	/*
	 * Move all TP threads on this scheduler to the RT class,
	 * until we call xnsched_set_policy() for them again.
	 */
	if (list_empty(&tp->threads))
		goto done;

	list_for_each_entry_safe(thread, tmp, &tp->threads, tp_link) {
		param.rt.prio = thread->cprio;
		xnsched_set_policy(thread, &xnsched_class_rt, &param);
	}
done:
	old_gps = tp->gps;
	tp->gps = gps;

	return old_gps;
}
EXPORT_SYMBOL_GPL(xnsched_tp_set_schedule);

struct xnsched_tp_schedule *
xnsched_tp_get_schedule(struct xnsched *sched)
{
	struct xnsched_tp_schedule *gps;

	gps = sched->tp.gps;
	if (gps == NULL)
		return NULL;

	atomic_inc(&gps->refcount);

	return gps;
}
EXPORT_SYMBOL_GPL(xnsched_tp_get_schedule);

void xnsched_tp_put_schedule(struct xnsched_tp_schedule *gps)
{
	if (atomic_dec_and_test(&gps->refcount))
		xnfree(gps);
}
EXPORT_SYMBOL_GPL(xnsched_tp_put_schedule);

int xnsched_tp_get_partition(struct xnsched *sched)
{
	struct xnsched_tp *tp = &sched->tp;

	if (tp->tps == NULL || tp->tps == &tp->idle)
		return -1;

	return tp->tps - tp->partitions;
}
EXPORT_SYMBOL_GPL(xnsched_tp_get_partition);

#ifdef CONFIG_XENO_OPT_VFILE

struct xnvfile_directory sched_tp_vfroot;

struct vfile_sched_tp_priv {
	struct xnthread *curr;
};

struct vfile_sched_tp_data {
	int cpu;
	pid_t pid;
	char name[XNOBJECT_NAME_LEN];
	int prio;
	int ptid;
};

static struct xnvfile_snapshot_ops vfile_sched_tp_ops;

static struct xnvfile_snapshot vfile_sched_tp = {
	.privsz = sizeof(struct vfile_sched_tp_priv),
	.datasz = sizeof(struct vfile_sched_tp_data),
	.tag = &nkthreadlist_tag,
	.ops = &vfile_sched_tp_ops,
};

static int vfile_sched_tp_rewind(struct xnvfile_snapshot_iterator *it)
{
	struct vfile_sched_tp_priv *priv = xnvfile_iterator_priv(it);
	int nrthreads = xnsched_class_tp.nthreads;

	if (nrthreads == 0)
		return -ESRCH;

	priv->curr = list_first_entry(&nkthreadq, struct xnthread, glink);

	return nrthreads;
}

static int vfile_sched_tp_next(struct xnvfile_snapshot_iterator *it,
			       void *data)
{
	struct vfile_sched_tp_priv *priv = xnvfile_iterator_priv(it);
	struct vfile_sched_tp_data *p = data;
	struct xnthread *thread;

	if (priv->curr == NULL)
		return 0;	/* All done. */

	thread = priv->curr;
	if (list_is_last(&thread->glink, &nkthreadq))
		priv->curr = NULL;
	else
		priv->curr = list_next_entry(thread, glink);

	if (thread->base_class != &xnsched_class_tp)
		return VFILE_SEQ_SKIP;

	p->cpu = xnsched_cpu(thread->sched);
	p->pid = xnthread_host_pid(thread);
	memcpy(p->name, thread->name, sizeof(p->name));
	p->ptid = thread->tps - thread->sched->tp.partitions;
	p->prio = thread->cprio;

	return 1;
}

static int vfile_sched_tp_show(struct xnvfile_snapshot_iterator *it,
			       void *data)
{
	struct vfile_sched_tp_data *p = data;

	if (p == NULL)
		xnvfile_printf(it, "%-3s  %-6s %-4s %-4s  %s\n",
			       "CPU", "PID", "PTID", "PRI", "NAME");
	else
		xnvfile_printf(it, "%3u  %-6d %-4d %-4d  %s\n",
			       p->cpu,
			       p->pid,
			       p->ptid,
			       p->prio,
			       p->name);

	return 0;
}

static struct xnvfile_snapshot_ops vfile_sched_tp_ops = {
	.rewind = vfile_sched_tp_rewind,
	.next = vfile_sched_tp_next,
	.show = vfile_sched_tp_show,
};

static int xnsched_tp_init_vfile(struct xnsched_class *schedclass,
				 struct xnvfile_directory *vfroot)
{
	int ret;

	ret = xnvfile_init_dir(schedclass->name, &sched_tp_vfroot, vfroot);
	if (ret)
		return ret;

	return xnvfile_init_snapshot("threads", &vfile_sched_tp,
				     &sched_tp_vfroot);
}

static void xnsched_tp_cleanup_vfile(struct xnsched_class *schedclass)
{
	xnvfile_destroy_snapshot(&vfile_sched_tp);
	xnvfile_destroy_dir(&sched_tp_vfroot);
}

#endif /* CONFIG_XENO_OPT_VFILE */

struct xnsched_class xnsched_class_tp = {
	.sched_init		=	xnsched_tp_init,
	.sched_enqueue		=	xnsched_tp_enqueue,
	.sched_dequeue		=	xnsched_tp_dequeue,
	.sched_requeue		=	xnsched_tp_requeue,
	.sched_pick		=	xnsched_tp_pick,
	.sched_tick		=	NULL,
	.sched_rotate		=	NULL,
	.sched_migrate		=	xnsched_tp_migrate,
	.sched_setparam		=	xnsched_tp_setparam,
	.sched_getparam		=	xnsched_tp_getparam,
	.sched_trackprio	=	xnsched_tp_trackprio,
	.sched_declare		=	xnsched_tp_declare,
	.sched_forget		=	xnsched_tp_forget,
	.sched_kick		=	NULL,
#ifdef CONFIG_XENO_OPT_VFILE
	.sched_init_vfile	=	xnsched_tp_init_vfile,
	.sched_cleanup_vfile	=	xnsched_tp_cleanup_vfile,
#endif
	.weight			=	XNSCHED_CLASS_WEIGHT(2),
	.policy			=	SCHED_TP,
	.name			=	"tp"
};
EXPORT_SYMBOL_GPL(xnsched_class_tp);
