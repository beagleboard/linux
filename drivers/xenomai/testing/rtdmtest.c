/*
 * Copyright (C) 2010 Jan Kiszka <jan.kiszka@web.de>.
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
#include <rtdm/driver.h>
#include <rtdm/testing.h>

MODULE_DESCRIPTION("RTDM test helper module");
MODULE_AUTHOR("Jan Kiszka <jan.kiszka@web.de>");
MODULE_VERSION("0.1.0");
MODULE_LICENSE("GPL");

struct rtdm_basic_context {
	rtdm_timer_t close_timer;
	unsigned long close_counter;
	unsigned long close_deferral;
};

struct rtdm_actor_context {
	rtdm_task_t actor_task;
	unsigned int request;
	rtdm_event_t run;
	rtdm_event_t done;
	union {
		__u32 cpu;
	} args;
};

static void close_timer_proc(rtdm_timer_t *timer)
{
	struct rtdm_basic_context *ctx =
		container_of(timer, struct rtdm_basic_context, close_timer);

	if (ctx->close_counter != 1)
		printk(XENO_ERR
		       "rtdmtest: %s: close_counter is %lu, should be 1!\n",
		       __FUNCTION__, ctx->close_counter);

	ctx->close_deferral = RTTST_RTDM_NORMAL_CLOSE;
	rtdm_fd_unlock(rtdm_private_to_fd(ctx));
}

static int rtdm_basic_open(struct rtdm_fd *fd, int oflags)
{
	struct rtdm_basic_context *ctx = rtdm_fd_to_private(fd);

	rtdm_timer_init(&ctx->close_timer, close_timer_proc,
			"rtdm close test");
	ctx->close_counter = 0;
	ctx->close_deferral = RTTST_RTDM_NORMAL_CLOSE;

	return 0;
}

static void rtdm_basic_close(struct rtdm_fd *fd)
{
	struct rtdm_basic_context *ctx = rtdm_fd_to_private(fd);

	ctx->close_counter++;

	switch (ctx->close_deferral) {
	case RTTST_RTDM_DEFER_CLOSE_CONTEXT:
		if (ctx->close_counter != 2) {
			printk(XENO_ERR
			       "rtdmtest: %s: close_counter is %lu, "
			       "should be 2!\n",
			       __FUNCTION__, ctx->close_counter);
			return;
		}
		rtdm_fd_unlock(fd);
		break;
	}

	rtdm_timer_destroy(&ctx->close_timer);
}

static int rtdm_basic_ioctl_rt(struct rtdm_fd *fd,
			    unsigned int request, void __user *arg)
{
	int ret, magic = RTTST_RTDM_MAGIC_PRIMARY;

	switch (request) {
	case RTTST_RTIOC_RTDM_PING_PRIMARY:
		ret = rtdm_safe_copy_to_user(fd, arg, &magic,
					     sizeof(magic));
		break;
	default:
		ret = -ENOSYS;
	}

	return ret;
}

static int rtdm_basic_ioctl_nrt(struct rtdm_fd *fd,
			    unsigned int request, void __user *arg)
{
	struct rtdm_basic_context *ctx = rtdm_fd_to_private(fd);
	int ret = 0, magic = RTTST_RTDM_MAGIC_SECONDARY;

	switch (request) {
	case RTTST_RTIOC_RTDM_DEFER_CLOSE:
		ctx->close_deferral = (unsigned long)arg;
		if (ctx->close_deferral == RTTST_RTDM_DEFER_CLOSE_CONTEXT) {
			++ctx->close_counter;
			rtdm_fd_lock(fd);
			rtdm_timer_start(&ctx->close_timer, 300000000ULL, 0,
					RTDM_TIMERMODE_RELATIVE);
		}
		break;
	case RTTST_RTIOC_RTDM_PING_SECONDARY:
		ret = rtdm_safe_copy_to_user(fd, arg, &magic,
					     sizeof(magic));
		break;
	default:
		ret = -ENOTTY;
	}

	return ret;
}

static void actor_handler(void *arg)
{
	struct rtdm_actor_context *ctx = arg;
	int ret;

	for (;;) {
		if (rtdm_task_should_stop())
			return;

		ret = rtdm_event_wait(&ctx->run);
		if (ret)
			break;

		switch (ctx->request) {
		case RTTST_RTIOC_RTDM_ACTOR_GET_CPU:
			ctx->args.cpu = task_cpu(current);
			break;
		default:
			printk(XENO_ERR "rtdmtest: bad request code %d\n",
			       ctx->request);
		}

		rtdm_event_signal(&ctx->done);
	}
}

static int rtdm_actor_open(struct rtdm_fd *fd, int oflags)
{
	struct rtdm_actor_context *ctx = rtdm_fd_to_private(fd);

	rtdm_event_init(&ctx->run, 0);
	rtdm_event_init(&ctx->done, 0);

	return rtdm_task_init(&ctx->actor_task, "rtdm_actor",
			      actor_handler, ctx,
			      RTDM_TASK_LOWEST_PRIORITY, 0);
}

static void rtdm_actor_close(struct rtdm_fd *fd)
{
	struct rtdm_actor_context *ctx = rtdm_fd_to_private(fd);

	rtdm_task_destroy(&ctx->actor_task);
	rtdm_event_destroy(&ctx->run);
	rtdm_event_destroy(&ctx->done);
}

#define ACTION_TIMEOUT 50000000ULL /* 50 ms timeout on action */

static int run_action(struct rtdm_actor_context *ctx, unsigned int request)
{
	rtdm_toseq_t toseq;

	rtdm_toseq_init(&toseq, ACTION_TIMEOUT);
	ctx->request = request;
	rtdm_event_signal(&ctx->run);
	/*
	 * XXX: The handshake mechanism is not bullet-proof against
	 * -EINTR received when waiting for the done event. Hopefully
	 * we won't restart/start a request while the action task has
	 * not yet completed the previous one we stopped waiting for
	 * abruptly.
	 */
	return rtdm_event_timedwait(&ctx->done, ACTION_TIMEOUT, &toseq);
}

static int rtdm_actor_ioctl(struct rtdm_fd *fd,
			    unsigned int request, void __user *arg)
{
	struct rtdm_actor_context *ctx = rtdm_fd_to_private(fd);
	int ret;

	switch (request) {
	case RTTST_RTIOC_RTDM_ACTOR_GET_CPU:
		ctx->args.cpu = (__u32)-EINVAL;
		ret = run_action(ctx, request);
		if (ret)
			break;
		ret = rtdm_safe_copy_to_user(fd, arg, &ctx->args.cpu,
					     sizeof(ctx->args.cpu));
		break;
	default:
		ret = -ENOTTY;
	}

	return ret;
}
      
static struct rtdm_driver rtdm_basic_driver = {
	.profile_info		= RTDM_PROFILE_INFO(rtdm_test_basic,
						    RTDM_CLASS_TESTING,
						    RTDM_SUBCLASS_RTDMTEST,
						    RTTST_PROFILE_VER),
	.device_flags		= RTDM_NAMED_DEVICE | RTDM_EXCLUSIVE,
	.device_count		= 2,
	.context_size		= sizeof(struct rtdm_basic_context),
	.ops = {
		.open		= rtdm_basic_open,
		.close		= rtdm_basic_close,
		.ioctl_rt	= rtdm_basic_ioctl_rt,
		.ioctl_nrt	= rtdm_basic_ioctl_nrt,
	},
};

static struct rtdm_driver rtdm_actor_driver = {
	.profile_info		= RTDM_PROFILE_INFO(rtdm_test_actor,
						    RTDM_CLASS_TESTING,
						    RTDM_SUBCLASS_RTDMTEST,
						    RTTST_PROFILE_VER),
	.device_flags		= RTDM_NAMED_DEVICE | RTDM_EXCLUSIVE,
	.device_count		= 1,
	.context_size		= sizeof(struct rtdm_actor_context),
	.ops = {
		.open		= rtdm_actor_open,
		.close		= rtdm_actor_close,
		.ioctl_rt	= rtdm_actor_ioctl,
	},
};

static struct rtdm_device device[3] = {
	[0 ... 1] = {
		.driver = &rtdm_basic_driver,
		.label = "rtdm%d",
	},
	[2] = {
		.driver = &rtdm_actor_driver,
		.label = "rtdmx",
	}
};

static int __init rtdm_test_init(void)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(device); i++) {
		ret = rtdm_dev_register(device + i);
		if (ret)
			goto fail;
	}

	return 0;
fail:
	while (i-- > 0)
		rtdm_dev_unregister(device + i);

	return ret;
}

static void __exit rtdm_test_exit(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(device); i++)
		rtdm_dev_unregister(device + i);
}

module_init(rtdm_test_init);
module_exit(rtdm_test_exit);
