/*
 * Copyright (C) 2010 Gilles Chanteperdrix <gilles.chanteperdrix@xenomai.org>.
 *
 * Xenomai is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Xenomai is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Xenomai; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#include <linux/module.h>
#include <linux/vmalloc.h>
#include <linux/semaphore.h>
#include <cobalt/kernel/sched.h>
#include <cobalt/kernel/synch.h>
#include <cobalt/kernel/thread.h>
#include <cobalt/kernel/trace.h>
#include <rtdm/testing.h>
#include <rtdm/driver.h>
#include <asm/xenomai/fptest.h>

MODULE_DESCRIPTION("Cobalt context switch test helper");
MODULE_AUTHOR("Gilles Chanteperdrix <gilles.chanteperdrix@xenomai.org>");
MODULE_VERSION("0.1.1");
MODULE_LICENSE("GPL");

#define RTSWITCH_RT      0x10000
#define RTSWITCH_NRT     0
#define RTSWITCH_KERNEL  0x20000

struct rtswitch_task {
	struct rttst_swtest_task base;
	rtdm_event_t rt_synch;
	struct semaphore nrt_synch;
	struct xnthread ktask;          /* For kernel-space real-time tasks. */
	unsigned int last_switch;
};

struct rtswitch_context {
	struct rtswitch_task *tasks;
	unsigned int tasks_count;
	unsigned int next_index;
	struct semaphore lock;
	unsigned int cpu;
	unsigned int switches_count;

	unsigned long pause_us;
	unsigned int next_task;
	rtdm_timer_t wake_up_delay;

	unsigned int failed;
	struct rttst_swtest_error error;

	struct rtswitch_task *utask;
	rtdm_nrtsig_t wake_utask;
};

static int fp_features;

static int report(const char *fmt, ...)
{
	va_list ap;
	int ret;

	va_start(ap, fmt);
	ret = vprintk(fmt, ap);
	va_end(ap);

	return ret;
}

static void handle_ktask_error(struct rtswitch_context *ctx, unsigned int fp_val)
{
	struct rtswitch_task *cur = &ctx->tasks[ctx->error.last_switch.to];
	unsigned int i;

	ctx->failed = 1;
	ctx->error.fp_val = fp_val;

	if ((cur->base.flags & RTSWITCH_RT) == RTSWITCH_RT)
		for (i = 0; i < ctx->tasks_count; i++) {
			struct rtswitch_task *task = &ctx->tasks[i];

			/* Find the first non kernel-space task. */
			if ((task->base.flags & RTSWITCH_KERNEL))
				continue;

			/* Unblock it. */
			switch(task->base.flags & RTSWITCH_RT) {
			case RTSWITCH_NRT:
				ctx->utask = task;
				rtdm_nrtsig_pend(&ctx->wake_utask);
				break;

			case RTSWITCH_RT:
				rtdm_event_signal(&task->rt_synch);
				break;
			}

			xnthread_suspend(&cur->ktask,
					 XNSUSP, XN_INFINITE, XN_RELATIVE, NULL);
		}
}

static int rtswitch_pend_rt(struct rtswitch_context *ctx,
			    unsigned int idx)
{
	struct rtswitch_task *task;
	int rc;

	if (idx > ctx->tasks_count)
		return -EINVAL;

	task = &ctx->tasks[idx];
	task->base.flags |= RTSWITCH_RT;

	rc = rtdm_event_wait(&task->rt_synch);
	if (rc < 0)
		return rc;

	if (ctx->failed)
		return 1;

	return 0;
}

static void timed_wake_up(rtdm_timer_t *timer)
{
	struct rtswitch_context *ctx =
		container_of(timer, struct rtswitch_context, wake_up_delay);
	struct rtswitch_task *task;

	task = &ctx->tasks[ctx->next_task];

	switch (task->base.flags & RTSWITCH_RT) {
	case RTSWITCH_NRT:
		ctx->utask = task;
		rtdm_nrtsig_pend(&ctx->wake_utask);
		break;

	case RTSWITCH_RT:
		rtdm_event_signal(&task->rt_synch);
	}
}

static int rtswitch_to_rt(struct rtswitch_context *ctx,
			  unsigned int from_idx,
			  unsigned int to_idx)
{
	struct rtswitch_task *from, *to;
	int rc;

	if (from_idx > ctx->tasks_count || to_idx > ctx->tasks_count)
		return -EINVAL;

	/* to == from is a special case which means
	   "return to the previous task". */
	if (to_idx == from_idx)
		to_idx = ctx->error.last_switch.from;

	from = &ctx->tasks[from_idx];
	to = &ctx->tasks[to_idx];

	from->base.flags |= RTSWITCH_RT;
	from->last_switch = ++ctx->switches_count;
	ctx->error.last_switch.from = from_idx;
	ctx->error.last_switch.to = to_idx;
	barrier();

	if (ctx->pause_us) {
		ctx->next_task = to_idx;
		barrier();
		rtdm_timer_start(&ctx->wake_up_delay,
				 ctx->pause_us * 1000, 0,
				 RTDM_TIMERMODE_RELATIVE);
		xnsched_lock();
	} else
		switch (to->base.flags & RTSWITCH_RT) {
		case RTSWITCH_NRT:
			ctx->utask = to;
			barrier();
			rtdm_nrtsig_pend(&ctx->wake_utask);
			xnsched_lock();
			break;

		case RTSWITCH_RT:
			xnsched_lock();
			rtdm_event_signal(&to->rt_synch);
			break;

		default:
			return -EINVAL;
		}

	rc = rtdm_event_wait(&from->rt_synch);
	xnsched_unlock();

	if (rc < 0)
		return rc;

	if (ctx->failed)
		return 1;

	return 0;
}

static int rtswitch_pend_nrt(struct rtswitch_context *ctx,
			     unsigned int idx)
{
	struct rtswitch_task *task;

	if (idx > ctx->tasks_count)
		return -EINVAL;

	task = &ctx->tasks[idx];

	task->base.flags &= ~RTSWITCH_RT;

	if (down_interruptible(&task->nrt_synch))
		return -EINTR;

	if (ctx->failed)
		return 1;

	return 0;
}

static int rtswitch_to_nrt(struct rtswitch_context *ctx,
			   unsigned int from_idx,
			   unsigned int to_idx)
{
	struct rtswitch_task *from, *to;
	unsigned int expected, fp_val;
	int fp_check;

	if (from_idx > ctx->tasks_count || to_idx > ctx->tasks_count)
		return -EINVAL;

	/* to == from is a special case which means
	   "return to the previous task". */
	if (to_idx == from_idx)
		to_idx = ctx->error.last_switch.from;

	from = &ctx->tasks[from_idx];
	to = &ctx->tasks[to_idx];

	fp_check = ctx->switches_count == from->last_switch + 1
		&& ctx->error.last_switch.from == to_idx
		&& ctx->error.last_switch.to == from_idx;

	from->base.flags &= ~RTSWITCH_RT;
	from->last_switch = ++ctx->switches_count;
	ctx->error.last_switch.from = from_idx;
	ctx->error.last_switch.to = to_idx;
	barrier();

	if (ctx->pause_us) {
		ctx->next_task = to_idx;
		barrier();
		rtdm_timer_start(&ctx->wake_up_delay,
				 ctx->pause_us * 1000, 0,
				 RTDM_TIMERMODE_RELATIVE);
	} else
		switch (to->base.flags & RTSWITCH_RT) {
		case RTSWITCH_NRT:
		switch_to_nrt:
			up(&to->nrt_synch);
			break;

		case RTSWITCH_RT:

			if (!fp_check || fp_linux_begin() < 0) {
				fp_check = 0;
				goto signal_nofp;
			}

			expected = from_idx + 500 +
				(ctx->switches_count % 4000000) * 1000;

			fp_regs_set(fp_features, expected);
			rtdm_event_signal(&to->rt_synch);
			fp_val = fp_regs_check(fp_features, expected, report);
			fp_linux_end();

			if(down_interruptible(&from->nrt_synch))
				return -EINTR;
			if (ctx->failed)
				return 1;
			if (fp_val != expected) {
				handle_ktask_error(ctx, fp_val);
				return 1;
			}

			from->base.flags &= ~RTSWITCH_RT;
			from->last_switch = ++ctx->switches_count;
			ctx->error.last_switch.from = from_idx;
			ctx->error.last_switch.to = to_idx;
			if ((to->base.flags & RTSWITCH_RT) == RTSWITCH_NRT)
				goto switch_to_nrt;
			expected = from_idx + 500 +
				(ctx->switches_count % 4000000) * 1000;
			barrier();

			fp_linux_begin();
			fp_regs_set(fp_features, expected);
			rtdm_event_signal(&to->rt_synch);
			fp_val = fp_regs_check(fp_features, expected, report);
			fp_linux_end();

			if (down_interruptible(&from->nrt_synch))
				return -EINTR;
			if (ctx->failed)
				return 1;
			if (fp_val != expected) {
				handle_ktask_error(ctx, fp_val);
				return 1;
			}

			from->base.flags &= ~RTSWITCH_RT;
			from->last_switch = ++ctx->switches_count;
			ctx->error.last_switch.from = from_idx;
			ctx->error.last_switch.to = to_idx;
			barrier();
			if ((to->base.flags & RTSWITCH_RT) == RTSWITCH_NRT)
				goto switch_to_nrt;

		signal_nofp:
			rtdm_event_signal(&to->rt_synch);
			break;

		default:
			return -EINVAL;
		}

	if (down_interruptible(&from->nrt_synch))
		return -EINTR;

	if (ctx->failed)
		return 1;

	return 0;
}

static int rtswitch_set_tasks_count(struct rtswitch_context *ctx, unsigned int count)
{
	struct rtswitch_task *tasks;

	if (ctx->tasks_count == count)
		return 0;

	tasks = vmalloc(count * sizeof(*tasks));

	if (!tasks)
		return -ENOMEM;

	down(&ctx->lock);

	if (ctx->tasks)
		vfree(ctx->tasks);

	ctx->tasks = tasks;
	ctx->tasks_count = count;
	ctx->next_index = 0;

	up(&ctx->lock);

	return 0;
}

static int rtswitch_register_task(struct rtswitch_context *ctx,
				  struct rttst_swtest_task *arg)
{
	struct rtswitch_task *t;

	down(&ctx->lock);

	if (ctx->next_index == ctx->tasks_count) {
		up(&ctx->lock);
		return -EBUSY;
	}

	arg->index = ctx->next_index;
	t = &ctx->tasks[arg->index];
	ctx->next_index++;
	t->base = *arg;
	t->last_switch = 0;
	sema_init(&t->nrt_synch, 0);
	rtdm_event_init(&t->rt_synch, 0);

	up(&ctx->lock);

	return 0;
}

struct taskarg {
	struct rtswitch_context *ctx;
	struct rtswitch_task *task;
};

static void rtswitch_ktask(void *cookie)
{
	struct taskarg *arg = (struct taskarg *) cookie;
	unsigned int fp_val, expected, to, i = 0;
	struct rtswitch_context *ctx = arg->ctx;
	struct rtswitch_task *task = arg->task;

	to = task->base.index;

	rtswitch_pend_rt(ctx, task->base.index);

	while (!rtdm_task_should_stop()) {
		if (task->base.flags & RTTST_SWTEST_USE_FPU)
			fp_regs_set(fp_features, task->base.index + i * 1000);

		switch(i % 3) {
		case 0:
			/* to == from means "return to last task" */
			rtswitch_to_rt(ctx, task->base.index, task->base.index);
			break;
		case 1:
			if (++to == task->base.index)
				++to;
			if (to > ctx->tasks_count - 1)
				to = 0;
			if (to == task->base.index)
				++to;

			/* Fall through. */
		case 2:
			rtswitch_to_rt(ctx, task->base.index, to);
		}

		if (task->base.flags & RTTST_SWTEST_USE_FPU) {
			expected = task->base.index + i * 1000;
			fp_val = fp_regs_check(fp_features, expected, report);

			if (fp_val != expected) {
				if (task->base.flags & RTTST_SWTEST_FREEZE)
					xntrace_user_freeze(0, 0);
				handle_ktask_error(ctx, fp_val);
			}
		}

		if (++i == 4000000)
			i = 0;
	}
}

static int rtswitch_create_ktask(struct rtswitch_context *ctx,
				 struct rttst_swtest_task *ptask)
{
	union xnsched_policy_param param;
	struct xnthread_start_attr sattr;
	struct xnthread_init_attr iattr;
	struct rtswitch_task *task;
	struct taskarg arg;
	int init_flags;
	char name[30];
	int err;

	/*
	 * Silently disable FP tests in kernel if FPU is not supported
	 * there. Typical case is math emulation support: we can use
	 * it from userland as a synthetic FPU, but there is no sane
	 * way to use it from kernel-based threads (Xenomai or Linux).
	 */
	if (!fp_kernel_supported())
		ptask->flags &= ~RTTST_SWTEST_USE_FPU;

	ptask->flags |= RTSWITCH_KERNEL;
	err = rtswitch_register_task(ctx, ptask);

	if (err)
		return err;

	ksformat(name, sizeof(name), "rtk%d/%u", ptask->index, ctx->cpu);

	task = &ctx->tasks[ptask->index];

	arg.ctx = ctx;
	arg.task = task;

	init_flags = (ptask->flags & RTTST_SWTEST_FPU) ? XNFPU : 0;

	iattr.name = name;
	iattr.flags = init_flags;
	iattr.personality = &xenomai_personality;
	iattr.affinity = *cpumask_of(ctx->cpu);
	param.rt.prio = 1;

	set_cpus_allowed_ptr(current, cpumask_of(ctx->cpu));

	err = xnthread_init(&task->ktask,
			    &iattr, &xnsched_class_rt, &param);
	if (!err) {
		sattr.mode = 0;
		sattr.entry = rtswitch_ktask;
		sattr.cookie = &arg;
		err = xnthread_start(&task->ktask, &sattr);
	} else
		/*
		 * In order to avoid calling xnthread_cancel() for an
		 * invalid thread.
		 */
		task->base.flags = 0;
	/*
	 * Putting the argument on stack is safe, because the new
	 * thread, thanks to the above call to set_cpus_allowed_ptr(),
	 * will preempt the current thread immediately, and will
	 * suspend only once the arguments on stack are used.
	 */

	return err;
}

static void rtswitch_utask_waker(rtdm_nrtsig_t *sig, void *arg)
{
	struct rtswitch_context *ctx = (struct rtswitch_context *)arg;
	up(&ctx->utask->nrt_synch);
}

static int rtswitch_open(struct rtdm_fd *fd, int oflags)
{
	struct rtswitch_context *ctx = rtdm_fd_to_private(fd);

	ctx->tasks = NULL;
	ctx->tasks_count = ctx->next_index = ctx->cpu = ctx->switches_count = 0;
	sema_init(&ctx->lock, 1);
	ctx->failed = 0;
	ctx->error.last_switch.from = ctx->error.last_switch.to = -1;
	ctx->pause_us = 0;

	rtdm_nrtsig_init(&ctx->wake_utask, rtswitch_utask_waker, ctx);

	rtdm_timer_init(&ctx->wake_up_delay, timed_wake_up, "switchtest timer");

	return 0;
}

static void rtswitch_close(struct rtdm_fd *fd)
{
	struct rtswitch_context *ctx = rtdm_fd_to_private(fd);
	unsigned int i;

	rtdm_timer_destroy(&ctx->wake_up_delay);
	rtdm_nrtsig_destroy(&ctx->wake_utask);

	if (ctx->tasks) {
		set_cpus_allowed_ptr(current, cpumask_of(ctx->cpu));

		for (i = 0; i < ctx->next_index; i++) {
			struct rtswitch_task *task = &ctx->tasks[i];

			if (task->base.flags & RTSWITCH_KERNEL) {
				rtdm_task_destroy(&task->ktask);
				rtdm_task_join(&task->ktask);
			}
			rtdm_event_destroy(&task->rt_synch);
		}
		vfree(ctx->tasks);
	}
}

static int rtswitch_ioctl_nrt(struct rtdm_fd *fd,
			      unsigned int request,
			      void *arg)
{
	struct rtswitch_context *ctx = rtdm_fd_to_private(fd);
	struct rttst_swtest_task task;
	struct rttst_swtest_dir fromto;
	__u32 count;
	int err;

	switch (request) {
	case RTTST_RTIOC_SWTEST_SET_TASKS_COUNT:
		return rtswitch_set_tasks_count(ctx,
						(unsigned long) arg);

	case RTTST_RTIOC_SWTEST_SET_CPU:
		if ((unsigned long) arg > num_online_cpus() - 1)
			return -EINVAL;

		ctx->cpu = (unsigned long) arg;
		return 0;

	case RTTST_RTIOC_SWTEST_SET_PAUSE:
		ctx->pause_us = (unsigned long) arg;
		return 0;

	case RTTST_RTIOC_SWTEST_REGISTER_UTASK:
		if (!rtdm_rw_user_ok(fd, arg, sizeof(task)))
			return -EFAULT;

		rtdm_copy_from_user(fd, &task, arg, sizeof(task));

		err = rtswitch_register_task(ctx, &task);

		if (!err)
			rtdm_copy_to_user(fd,
					  arg,
					  &task,
					  sizeof(task));

		return err;

	case RTTST_RTIOC_SWTEST_CREATE_KTASK:
		if (!rtdm_rw_user_ok(fd, arg, sizeof(task)))
			return -EFAULT;

		rtdm_copy_from_user(fd, &task, arg, sizeof(task));

		err = rtswitch_create_ktask(ctx, &task);

		if (!err)
			rtdm_copy_to_user(fd,
					  arg,
					  &task,
					  sizeof(task));

		return err;

	case RTTST_RTIOC_SWTEST_PEND:
		if (!rtdm_read_user_ok(fd, arg, sizeof(task)))
			return -EFAULT;

		rtdm_copy_from_user(fd, &task, arg, sizeof(task));

		return rtswitch_pend_nrt(ctx, task.index);

	case RTTST_RTIOC_SWTEST_SWITCH_TO:
		if (!rtdm_read_user_ok(fd, arg, sizeof(fromto)))
			return -EFAULT;

		rtdm_copy_from_user(fd,
				    &fromto,
				    arg,
				    sizeof(fromto));

		return rtswitch_to_nrt(ctx, fromto.from, fromto.to);

	case RTTST_RTIOC_SWTEST_GET_SWITCHES_COUNT:
		if (!rtdm_rw_user_ok(fd, arg, sizeof(count)))
			return -EFAULT;

		count = ctx->switches_count;

		rtdm_copy_to_user(fd, arg, &count, sizeof(count));

		return 0;

	case RTTST_RTIOC_SWTEST_GET_LAST_ERROR:
		if (!rtdm_rw_user_ok(fd, arg, sizeof(ctx->error)))
			return -EFAULT;

		rtdm_copy_to_user(fd,
				  arg,
				  &ctx->error,
				  sizeof(ctx->error));

		return 0;

	default:
		return -ENOTTY;
	}
}

static int rtswitch_ioctl_rt(struct rtdm_fd *fd,
			     unsigned int request,
			     void *arg)
{
	struct rtswitch_context *ctx = rtdm_fd_to_private(fd);
	struct rttst_swtest_task task;
	struct rttst_swtest_dir fromto;

	switch (request) {
	case RTTST_RTIOC_SWTEST_PEND:
		if (!rtdm_read_user_ok(fd, arg, sizeof(task)))
			return -EFAULT;

		rtdm_copy_from_user(fd, &task, arg, sizeof(task));

		return rtswitch_pend_rt(ctx, task.index);

	case RTTST_RTIOC_SWTEST_SWITCH_TO:
		if (!rtdm_read_user_ok(fd, arg, sizeof(fromto)))
			return -EFAULT;

		rtdm_copy_from_user(fd,
				    &fromto,
				    arg,
				    sizeof(fromto));

		return rtswitch_to_rt(ctx, fromto.from, fromto.to);

	case RTTST_RTIOC_SWTEST_GET_LAST_ERROR:
		if (!rtdm_rw_user_ok(fd, arg, sizeof(ctx->error)))
			return -EFAULT;

		rtdm_copy_to_user(fd,
				  arg,
				  &ctx->error,
				  sizeof(ctx->error));

		return 0;

	default:
		return -ENOSYS;
	}
}

static struct rtdm_driver switchtest_driver = {
	.profile_info = RTDM_PROFILE_INFO(switchtest,
					  RTDM_CLASS_TESTING,
					  RTDM_SUBCLASS_SWITCHTEST,
					  RTTST_PROFILE_VER),
	.device_flags = RTDM_NAMED_DEVICE,
	.device_count =	1,
	.context_size = sizeof(struct rtswitch_context),
	.ops = {
		.open = rtswitch_open,
		.close = rtswitch_close,
		.ioctl_rt = rtswitch_ioctl_rt,
		.ioctl_nrt = rtswitch_ioctl_nrt,
	},
};

static struct rtdm_device device = {
	.driver = &switchtest_driver,
	.label = "switchtest",
};

int __init __switchtest_init(void)
{
	if (!realtime_core_enabled())
		return 0;

	fp_features = fp_detect();

	return rtdm_dev_register(&device);
}

void __switchtest_exit(void)
{
	if (!realtime_core_enabled())
		return;

	rtdm_dev_unregister(&device);
}

module_init(__switchtest_init);
module_exit(__switchtest_exit);
