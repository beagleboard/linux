/***
 *
 *  rtmac/tdma/tdma_dev.c
 *
 *  RTmac - real-time networking media access control subsystem
 *  Copyright (C) 2002      Marc Kleine-Budde <kleine-budde@gmx.de>
 *                2003-2006 Jan Kiszka <Jan.Kiszka@web.de>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/list.h>

#include <rtdev.h>
#include <rtmac.h>
#include <rtmac/tdma/tdma.h>

struct tdma_dev_ctx {
	rtdm_task_t *cycle_waiter;
};

static int tdma_dev_open(struct rtdm_fd *fd, int oflags)
{
	struct tdma_dev_ctx *ctx = rtdm_fd_to_private(fd);

	ctx->cycle_waiter = NULL;

	return 0;
}

static void tdma_dev_close(struct rtdm_fd *fd)
{
	struct tdma_dev_ctx *ctx = rtdm_fd_to_private(fd);
	rtdm_lockctx_t lock_ctx;

	cobalt_atomic_enter(lock_ctx);
	if (ctx->cycle_waiter)
		rtdm_task_unblock(ctx->cycle_waiter);
	cobalt_atomic_leave(lock_ctx);
}

static int wait_on_sync(struct tdma_dev_ctx *tdma_ctx, rtdm_event_t *sync_event)
{
	rtdm_lockctx_t lock_ctx;
	int ret;

	cobalt_atomic_enter(lock_ctx);
	/* keep it simple: only one waiter per device instance allowed */
	if (!tdma_ctx->cycle_waiter) {
		tdma_ctx->cycle_waiter = rtdm_task_current();
		ret = rtdm_event_wait(sync_event);
		tdma_ctx->cycle_waiter = NULL;
	} else
		ret = -EBUSY;
	cobalt_atomic_leave(lock_ctx);

	return ret;
}

static int tdma_dev_ioctl(struct rtdm_fd *fd, unsigned int request, void *arg)
{
	struct tdma_dev_ctx *ctx = rtdm_fd_to_private(fd);
	struct tdma_priv *tdma;
	rtdm_lockctx_t lock_ctx;
	int ret;

	tdma = container_of(rtdm_fd_to_context(fd)->device, struct tdma_priv,
			    api_device);

	switch (request) {
	case RTMAC_RTIOC_TIMEOFFSET: {
		nanosecs_rel_t offset;

		rtdm_lock_get_irqsave(&tdma->lock, lock_ctx);
		offset = tdma->clock_offset;
		rtdm_lock_put_irqrestore(&tdma->lock, lock_ctx);

		if (rtdm_fd_is_user(fd)) {
			if (!rtdm_rw_user_ok(fd, arg, sizeof(__s64)) ||
			    rtdm_copy_to_user(fd, arg, &offset, sizeof(__s64)))
				return -EFAULT;
		} else
			*(__s64 *)arg = offset;

		return 0;
	}
	case RTMAC_RTIOC_WAITONCYCLE:
		if (!rtdm_in_rt_context())
			return -ENOSYS;

		if ((long)arg != TDMA_WAIT_ON_SYNC)
			return -EINVAL;

		return wait_on_sync(ctx, &tdma->sync_event);

	case RTMAC_RTIOC_WAITONCYCLE_EX: {
		struct rtmac_waitinfo *waitinfo = (struct rtmac_waitinfo *)arg;
		struct rtmac_waitinfo waitinfo_buf;

#define WAITINFO_HEAD_SIZE                                                     \
	((char *)&waitinfo_buf.cycle_no - (char *)&waitinfo_buf)

		if (!rtdm_in_rt_context())
			return -ENOSYS;

		if (rtdm_fd_is_user(fd)) {
			if (!rtdm_rw_user_ok(fd, waitinfo,
					     sizeof(struct rtmac_waitinfo)) ||
			    rtdm_copy_from_user(fd, &waitinfo_buf, arg,
						WAITINFO_HEAD_SIZE))
				return -EFAULT;

			waitinfo = &waitinfo_buf;
		}

		if ((waitinfo->type != TDMA_WAIT_ON_SYNC) ||
		    (waitinfo->size < sizeof(struct rtmac_waitinfo)))
			return -EINVAL;

		ret = wait_on_sync(ctx, &tdma->sync_event);
		if (ret)
			return ret;

		rtdm_lock_get_irqsave(&tdma->lock, lock_ctx);
		waitinfo->cycle_no = tdma->current_cycle;
		waitinfo->cycle_start = tdma->current_cycle_start;
		waitinfo->clock_offset = tdma->clock_offset;
		rtdm_lock_put_irqrestore(&tdma->lock, lock_ctx);

		if (rtdm_fd_is_user(fd)) {
			if (rtdm_copy_to_user(fd, arg, &waitinfo_buf,
					      sizeof(struct rtmac_waitinfo)))
				return -EFAULT;
		}

		return 0;
	}
	default:
		return -ENOTTY;
	}
}

static struct rtdm_driver tdma_driver = { .profile_info = RTDM_PROFILE_INFO(
						  tdma, RTDM_CLASS_RTMAC,
						  RTDM_SUBCLASS_TDMA,
						  RTNET_RTDM_VER),
					  .device_flags = RTDM_NAMED_DEVICE,
					  .device_count = 1,
					  .context_size =
						  sizeof(struct tdma_dev_ctx),
					  .ops = {
						  .open = tdma_dev_open,
						  .ioctl_rt = tdma_dev_ioctl,
						  .ioctl_nrt = tdma_dev_ioctl,
						  .close = tdma_dev_close,
					  } };

int tdma_dev_init(struct rtnet_device *rtdev, struct tdma_priv *tdma)
{
	char *pos;

	strcpy(tdma->device_name, "TDMA");
	for (pos = rtdev->name + strlen(rtdev->name) - 1;
	     (pos >= rtdev->name) && ((*pos) >= '0') && (*pos <= '9'); pos--)
		;
	strncat(tdma->device_name + 4, pos + 1, IFNAMSIZ - 4);

	tdma->api_driver = tdma_driver;
	tdma->api_device.driver = &tdma->api_driver;
	tdma->api_device.label = tdma->device_name;

	return rtdm_dev_register(&tdma->api_device);
}
