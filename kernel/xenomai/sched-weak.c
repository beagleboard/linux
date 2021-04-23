/*
 * Copyright (C) 2013 Philippe Gerum <rpm@xenomai.org>.
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
#include <cobalt/uapi/sched.h>

static void xnsched_weak_init(struct xnsched *sched)
{
	xnsched_initq(&sched->weak.runnable);
}

static void xnsched_weak_requeue(struct xnthread *thread)
{
	xnsched_addq(&thread->sched->weak.runnable, thread);
}

static void xnsched_weak_enqueue(struct xnthread *thread)
{
	xnsched_addq_tail(&thread->sched->weak.runnable, thread);
}

static void xnsched_weak_dequeue(struct xnthread *thread)
{
	xnsched_delq(&thread->sched->weak.runnable, thread);
}

static struct xnthread *xnsched_weak_pick(struct xnsched *sched)
{
	return xnsched_getq(&sched->weak.runnable);
}

static bool xnsched_weak_setparam(struct xnthread *thread,
				  const union xnsched_policy_param *p)
{
	if (!xnthread_test_state(thread, XNBOOST))
		xnthread_set_state(thread, XNWEAK);

	return xnsched_set_effective_priority(thread, p->weak.prio);
}

static void xnsched_weak_getparam(struct xnthread *thread,
				  union xnsched_policy_param *p)
{
	p->weak.prio = thread->cprio;
}

static void xnsched_weak_trackprio(struct xnthread *thread,
				   const union xnsched_policy_param *p)
{
	if (p)
		thread->cprio = p->weak.prio;
	else
		thread->cprio = thread->bprio;
}

static void xnsched_weak_protectprio(struct xnthread *thread, int prio)
{
  	if (prio > XNSCHED_WEAK_MAX_PRIO)
		prio = XNSCHED_WEAK_MAX_PRIO;

	thread->cprio = prio;
}

static int xnsched_weak_chkparam(struct xnthread *thread,
				 const union xnsched_policy_param *p)
{
	if (p->weak.prio < XNSCHED_WEAK_MIN_PRIO ||
	    p->weak.prio > XNSCHED_WEAK_MAX_PRIO)
		return -EINVAL;

	return 0;
}

#ifdef CONFIG_XENO_OPT_VFILE

struct xnvfile_directory sched_weak_vfroot;

struct vfile_sched_weak_priv {
	struct xnthread *curr;
};

struct vfile_sched_weak_data {
	int cpu;
	pid_t pid;
	char name[XNOBJECT_NAME_LEN];
	int cprio;
};

static struct xnvfile_snapshot_ops vfile_sched_weak_ops;

static struct xnvfile_snapshot vfile_sched_weak = {
	.privsz = sizeof(struct vfile_sched_weak_priv),
	.datasz = sizeof(struct vfile_sched_weak_data),
	.tag = &nkthreadlist_tag,
	.ops = &vfile_sched_weak_ops,
};

static int vfile_sched_weak_rewind(struct xnvfile_snapshot_iterator *it)
{
	struct vfile_sched_weak_priv *priv = xnvfile_iterator_priv(it);
	int nrthreads = xnsched_class_weak.nthreads;

	if (nrthreads == 0)
		return -ESRCH;

	priv->curr = list_first_entry(&nkthreadq, struct xnthread, glink);

	return nrthreads;
}

static int vfile_sched_weak_next(struct xnvfile_snapshot_iterator *it,
				 void *data)
{
	struct vfile_sched_weak_priv *priv = xnvfile_iterator_priv(it);
	struct vfile_sched_weak_data *p = data;
	struct xnthread *thread;

	if (priv->curr == NULL)
		return 0;	/* All done. */

	thread = priv->curr;
	if (list_is_last(&thread->glink, &nkthreadq))
		priv->curr = NULL;
	else
		priv->curr = list_next_entry(thread, glink);

	if (thread->base_class != &xnsched_class_weak)
		return VFILE_SEQ_SKIP;

	p->cpu = xnsched_cpu(thread->sched);
	p->pid = xnthread_host_pid(thread);
	memcpy(p->name, thread->name, sizeof(p->name));
	p->cprio = thread->cprio;

	return 1;
}

static int vfile_sched_weak_show(struct xnvfile_snapshot_iterator *it,
				 void *data)
{
	struct vfile_sched_weak_data *p = data;
	char pribuf[16];

	if (p == NULL)
		xnvfile_printf(it, "%-3s  %-6s %-4s %s\n",
			       "CPU", "PID", "PRI", "NAME");
	else {
		ksformat(pribuf, sizeof(pribuf), "%3d", p->cprio);
		xnvfile_printf(it, "%3u  %-6d %-4s %s\n",
			       p->cpu,
			       p->pid,
			       pribuf,
			       p->name);
	}

	return 0;
}

static struct xnvfile_snapshot_ops vfile_sched_weak_ops = {
	.rewind = vfile_sched_weak_rewind,
	.next = vfile_sched_weak_next,
	.show = vfile_sched_weak_show,
};

static int xnsched_weak_init_vfile(struct xnsched_class *schedclass,
				   struct xnvfile_directory *vfroot)
{
	int ret;

	ret = xnvfile_init_dir(schedclass->name, &sched_weak_vfroot, vfroot);
	if (ret)
		return ret;

	return xnvfile_init_snapshot("threads", &vfile_sched_weak,
				     &sched_weak_vfroot);
}

static void xnsched_weak_cleanup_vfile(struct xnsched_class *schedclass)
{
	xnvfile_destroy_snapshot(&vfile_sched_weak);
	xnvfile_destroy_dir(&sched_weak_vfroot);
}

#endif /* CONFIG_XENO_OPT_VFILE */

struct xnsched_class xnsched_class_weak = {
	.sched_init		=	xnsched_weak_init,
	.sched_enqueue		=	xnsched_weak_enqueue,
	.sched_dequeue		=	xnsched_weak_dequeue,
	.sched_requeue		=	xnsched_weak_requeue,
	.sched_pick		=	xnsched_weak_pick,
	.sched_tick		=	NULL,
	.sched_rotate		=	NULL,
	.sched_forget		=	NULL,
	.sched_kick		=	NULL,
	.sched_chkparam		=	xnsched_weak_chkparam,
	.sched_setparam		=	xnsched_weak_setparam,
	.sched_trackprio	=	xnsched_weak_trackprio,
	.sched_protectprio	=	xnsched_weak_protectprio,
	.sched_getparam		=	xnsched_weak_getparam,
#ifdef CONFIG_XENO_OPT_VFILE
	.sched_init_vfile	=	xnsched_weak_init_vfile,
	.sched_cleanup_vfile	=	xnsched_weak_cleanup_vfile,
#endif
	.weight			=	XNSCHED_CLASS_WEIGHT(1),
	.policy			=	SCHED_WEAK,
	.name			=	"weak"
};
EXPORT_SYMBOL_GPL(xnsched_class_weak);
