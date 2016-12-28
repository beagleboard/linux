#define DEBUG
/*
 * Copyright (C) 2016 Noralf Trønnes
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <drm/drm_atomic_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/tinydrm/tinydrm.h>
#include <drm/tinydrm/tinydrm-helpers.h>
#include <linux/completion.h>
#include <linux/fs.h>
#include <linux/idr.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include <uapi/drm/utinydrm.h>


struct utinydrm_device {
	struct tinydrm_device	tdev;
	struct device dev;
	struct idr 		idr;

	struct mutex		mutex;
	wait_queue_head_t	waitq;
	struct completion	completion;

	struct utinydrm_event	*ev;
	int			event_ret;

	bool			initialized;
	bool			fbdev_fb_sent;
	struct work_struct	release_work;
};

static inline struct utinydrm_device *
udev_from_tdev(struct tinydrm_device *tdev)
{
	return container_of(tdev, struct utinydrm_device, tdev);
}

static struct miscdevice utinydrm_misc;

static int utinydrm_send_event(struct tinydrm_device *tdev, void *ev_in)
{
	struct utinydrm_device *udev = udev_from_tdev(tdev);
	struct utinydrm_event *ev = ev_in;
	unsigned long time_left;
	int ret = 0;

	ev = kmemdup(ev, ev->length, GFP_KERNEL);
	if (!ev)
		return -ENOMEM;

	dev_dbg(tdev->drm.dev, "%s(ev=%p) IN\n", __func__, ev);
	mutex_lock(&udev->tdev.dev_lock);
	reinit_completion(&udev->completion);

	ret = mutex_lock_interruptible(&udev->mutex);
	if (ret) {
		kfree(ev);
		goto out_unlock;
	}
	udev->ev = ev;
	mutex_unlock(&udev->mutex);

	dev_dbg(tdev->drm.dev, "%s: ev->type=%u, ev->length=%u\n", __func__, udev->ev->type, udev->ev->length);

	wake_up_interruptible(&udev->waitq);

	time_left = wait_for_completion_timeout(&udev->completion, 5 * HZ);
	//ret = udev->event_ret;
	//time_left = 1;
	if (!time_left) {
		dev_err(&udev->dev, "%s: timeout waiting for reply\n", __func__);
		ret =-ETIMEDOUT;
	}

out_unlock:
	mutex_unlock(&udev->tdev.dev_lock);

	dev_dbg(tdev->drm.dev, "%s OUT ret=%d, event_ret=%d\n", __func__, ret, udev->event_ret);
	return ret;
}

static int utinydrm_fb_create_event(struct drm_framebuffer *fb)
{
	struct tinydrm_device *tdev = drm_to_tinydrm(fb->dev);
	struct utinydrm_device *udev = udev_from_tdev(tdev);
	struct utinydrm_event_fb_create ev = {
		.base = {
			.type = UTINYDRM_EVENT_FB_CREATE,
			.length = sizeof(ev),
		},
	};
	struct drm_mode_fb_cmd2 *ufb = &ev.fb;
	struct drm_gem_cma_object *cma_obj;
	int ret, i;

	dev_dbg(fb->dev->dev, "%s: [FB:%d]\n", __func__, fb->base.id);

	ufb->fb_id = fb->base.id;
	ufb->width = fb->width;
	ufb->height = fb->height;
	ufb->pixel_format = fb->pixel_format;
	ufb->flags = fb->flags;

	for (i = 0; i < 4; i++) {
		cma_obj = drm_fb_cma_get_gem_obj(fb, i);
		if (!cma_obj)
			break;

		ufb->pitches[i] = fb->pitches[i];
		ufb->offsets[i] = fb->offsets[i];
		ufb->modifier[i] = fb->modifier[i];
	}

	ret = idr_alloc(&udev->idr, fb, fb->base.id, fb->base.id + 1, GFP_KERNEL);
	if (ret < 1) {
		dev_err(fb->dev->dev, "%s: [FB:%d]: failed to allocate idr %d\n", __func__, fb->base.id, ret);
		return ret;
	}

	ret = utinydrm_send_event(tdev, &ev);

	return ret;
}

static int utinydrm_fb_dirty(struct drm_framebuffer *fb,
			     struct drm_file *file_priv,
			     unsigned int flags, unsigned int color,
			     struct drm_clip_rect *clips,
			     unsigned int num_clips)
{
	struct tinydrm_device *tdev = drm_to_tinydrm(fb->dev);
	struct utinydrm_device *udev = udev_from_tdev(tdev);
	struct drm_mode_fb_dirty_cmd *dirty;
	struct utinydrm_event_fb_dirty *ev;
	struct drm_clip_rect clip;
	size_t size_clips, size;
	int ret;

	pr_debug("\n\n\n");
	if (!tinydrm_check_dirty(fb, &clips, &num_clips))
		return -EINVAL;

	/* FIXME: if (fb == tdev->fbdev_helper->fb) */
	if (tdev->fbdev_helper && !udev->fbdev_fb_sent) {
		utinydrm_fb_create_event(tdev->fbdev_helper->fb);
		udev->fbdev_fb_sent = true;
	}

	tdev->enabled = true;

	tinydrm_merge_clips(&clip, clips, num_clips, flags,
			    fb->width, fb->height);
	clip.x1 = 0;
	clip.x2 = fb->width;
	clips = &clip;
	num_clips = 1;

	size_clips = num_clips * sizeof(struct drm_clip_rect);
	size = sizeof(struct utinydrm_event_fb_dirty) + size_clips;
	ev = kzalloc(size, GFP_KERNEL);
	if (!ev)
		return -ENOMEM;

	dev_dbg(fb->dev->dev, "%s: [FB:%d]: num_clips=%u, size_clips=%zu, size=%zu\n", __func__, fb->base.id, num_clips, size_clips, size);

	ev->base.type = UTINYDRM_EVENT_FB_DIRTY;
	ev->base.length = size;
	dirty = &ev->fb_dirty_cmd;

	dirty->fb_id = fb->base.id;
	dirty->flags = flags;
	dirty->color = color;
	dirty->num_clips = num_clips;
	//dirty->clips_ptr

	if (num_clips)
		memcpy(ev->clips, clips, size_clips);

//	tinydrm_merge_clips(&clip, clips, num_clips, flags,
//			    fb->width, fb->height);
//	clip.x1 = 0;
//	clip.x2 = fb->width;

	DRM_DEBUG("Flushing [FB:%d] x1=%u, x2=%u, y1=%u, y2=%u\n", fb->base.id,
		  clip.x1, clip.x2, clip.y1, clip.y2);

	tinydrm_debugfs_dirty_begin(tdev, fb, &clip);

	ret = utinydrm_send_event(tdev, ev);

	tinydrm_debugfs_dirty_end(tdev, 0, 16);

	if (ret) {
		dev_err_once(fb->dev->dev, "Failed to update display %d\n",
			     ret);
	}

	return ret;
}

static void utinydrm_fb_destroy(struct drm_framebuffer *fb)
{
	struct tinydrm_device *tdev = drm_to_tinydrm(fb->dev);
	struct utinydrm_device *udev = udev_from_tdev(tdev);
	struct utinydrm_event_fb_destroy ev = {
		.base = {
			.type = UTINYDRM_EVENT_FB_DESTROY,
			.length = sizeof(ev),
		},
	};
	struct drm_framebuffer *iter;
	int id;

	dev_dbg(fb->dev->dev, "%s: [FB:%d]\n", __func__, fb->base.id);

	idr_for_each_entry(&udev->idr, iter, id) {
		if (fb == iter)
			break;
	}

	if (!iter) {
		dev_err(fb->dev->dev, "%s: failed to find idr\n", __func__);
		return;
	}

	ev.fb_id = id;
	idr_remove(&udev->idr, id);

	utinydrm_send_event(tdev, &ev);

	drm_fb_cma_destroy(fb);
}

static const struct drm_framebuffer_funcs utinydrm_fb_funcs = {
	.destroy	= utinydrm_fb_destroy,
	.create_handle	= drm_fb_cma_create_handle,
	.dirty		= utinydrm_fb_dirty,
};

static struct drm_framebuffer *
utinydrm_fb_create(struct drm_device *drm, struct drm_file *file_priv,
		   const struct drm_mode_fb_cmd2 *mode_cmd)
{
	struct drm_framebuffer *fb;
	int ret;

	fb = tinydrm_fb_create(drm, file_priv, mode_cmd);
	if (IS_ERR(fb))
		return fb;

	dev_dbg(drm->dev, "%s\n", __func__);
	ret = utinydrm_fb_create_event(fb);

	return fb;
}

static const struct drm_mode_config_funcs utinydrm_mode_config_funcs = {
	.fb_create = utinydrm_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static void utinydrm_pipe_enable(struct drm_simple_display_pipe *pipe,
				 struct drm_crtc_state *crtc_state)
{
	struct tinydrm_device *tdev = pipe_to_tinydrm(pipe);
	struct utinydrm_event ev = {
		.type = UTINYDRM_EVENT_PIPE_ENABLE,
		.length = sizeof(ev),
	};

	dev_dbg(tdev->drm.dev, "%s\n", __func__);
	tdev->prepared = true;
	utinydrm_send_event(tdev, &ev);
}

static void utinydrm_pipe_disable(struct drm_simple_display_pipe *pipe)
{
	struct tinydrm_device *tdev = pipe_to_tinydrm(pipe);
	struct utinydrm_event ev = {
		.type = UTINYDRM_EVENT_PIPE_DISABLE,
		.length = sizeof(ev),
	};

	dev_dbg(tdev->drm.dev, "%s\n", __func__);
	tdev->prepared = false;
	tdev->enabled = false;
	utinydrm_send_event(tdev, &ev);
}

static const struct drm_simple_display_pipe_funcs utinydrm_pipe_funcs = {
	.enable = utinydrm_pipe_enable,
	.disable = utinydrm_pipe_disable,
	.update = tinydrm_display_pipe_update,
};

static int utinydrm_prime_handle_to_fd_ioctl(struct drm_device *dev, void *data,
					     struct drm_file *file_priv)
{
	struct drm_prime_handle *args = data;

	/* FIXME: only the userspace driver should use this */

	/* check flags are valid */
	if (args->flags & ~(DRM_CLOEXEC | DRM_RDWR))
		return -EINVAL;

	return dev->driver->prime_handle_to_fd(dev, file_priv, args->handle,
					       args->flags, &args->fd);
}

static const struct drm_ioctl_desc utinydrm_ioctls[] = {
	DRM_IOCTL_DEF_DRV(UTINYDRM_PRIME_HANDLE_TO_FD, utinydrm_prime_handle_to_fd_ioctl, DRM_CONTROL_ALLOW|DRM_UNLOCKED),
};

static struct drm_driver utinydrm_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_PRIME |
				  DRIVER_ATOMIC,
	TINYDRM_GEM_DRIVER_OPS,
	.lastclose		= tinydrm_lastclose,

	.ioctls			= utinydrm_ioctls,
	.num_ioctls		= ARRAY_SIZE(utinydrm_ioctls),

	.name			= "utinydrm",
	.desc			= "Userspace driver tinydrm",
	.date			= "20161119",
	.major			= 1,
	.minor			= 0,
};

static const uint32_t utinydrm_formats[] = {
	DRM_FORMAT_RGB565,
	DRM_FORMAT_XRGB8888,
};

static int utinydrm_register(struct utinydrm_device *udev, struct utinydrm_dev_create *dev_create)
{
	struct tinydrm_device *tdev = &udev->tdev;
	struct drm_display_mode display_mode;
	struct device *dev = &udev->dev;
	struct drm_device *drm;
	struct drm_driver *drv;
	int ret;

	if (!dev->coherent_dma_mask) {
		ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(32));
		if (ret)
			dev_warn(dev, "Failed to set dma mask %d\n", ret);
	}

	drv = devm_kmalloc(&udev->dev, sizeof(*drv), GFP_KERNEL);
	if (!drv)
		return -ENOMEM;

	*drv = utinydrm_driver;
	drv->name = kstrdup(dev_create->name, GFP_KERNEL);
	if (!drv->name)
		return -ENOMEM;

	ret = devm_tinydrm_init(&udev->dev, tdev, &utinydrm_fb_funcs, drv);
	if (ret)
		return ret;

	drm = &tdev->drm;
	drm->mode_config.funcs = &utinydrm_mode_config_funcs;

	ret = drm_mode_convert_umode(&display_mode, &dev_create->mode);
	if (ret)
		return ret;

	drm_mode_debug_printmodeline(&display_mode);

	ret = tinydrm_display_pipe_init(tdev, &utinydrm_pipe_funcs,
					DRM_MODE_CONNECTOR_VIRTUAL,
					utinydrm_formats,
					ARRAY_SIZE(utinydrm_formats), &display_mode,
					0);
	if (ret)
		return ret;

	drm->mode_config.preferred_depth = 16;

	drm_mode_config_reset(drm);

	tinydrm_debugfs_dirty_init(tdev);

	DRM_DEBUG_KMS("preferred_depth=%u\n", drm->mode_config.preferred_depth);

	ret = devm_tinydrm_register(tdev);

	dev_create->index = drm->primary->index;

	return ret;
}

static void utinydrm_unregister(struct utinydrm_device *udev)
{
	struct tinydrm_device *tdev = &udev->tdev;

	devm_tinydrm_unregister(tdev);
}

/*********************************************************************************************************************************/

static void utinydrm_release_work(struct work_struct *work)
{
	struct utinydrm_device *udev = container_of(work, struct utinydrm_device,
						    release_work);
	struct drm_device *drm = &udev->tdev.drm;

	//drm_device_set_unplugged(drm);

	while (drm->open_count) {
		dev_dbg(drm->dev, "%s: open_count=%d\n", __func__, drm->open_count);
		msleep(1000);
	}

	utinydrm_unregister(udev);


	dev_dbg(&udev->dev, "%s: dev.refcount=%d\n", __func__, atomic_read(&udev->dev.kobj.kref.refcount));

	device_unregister(&udev->dev);
}

static void utinydrm_device_release(struct device *dev)
{
	//struct utinydrm_device *udev = container_of(dev, struct utinydrm_device, dev);

	dev_dbg(dev, "%s\n", __func__);
	/* FIXME: there's use after free */
	//kfree(udev);
}

static int utinydrm_open(struct inode *inode, struct file *file)
{
	struct utinydrm_device *udev;
	int ret;

	udev = kzalloc(sizeof(*udev), GFP_KERNEL);
	if (!udev)
		return -ENOMEM;

	/* FIXME: tinydrm currently needs a parent device */
	dev_set_name(&udev->dev, "utinydrm%u", 0);
	udev->dev.release = utinydrm_device_release;
	ret = device_register(&udev->dev);
	if (ret) {
		put_device(&udev->dev);
		kfree(udev);
		return ret;
	}

	mutex_init(&udev->mutex);
	init_waitqueue_head(&udev->waitq);
	init_completion(&udev->completion);
	idr_init(&udev->idr);
	INIT_WORK(&udev->release_work, utinydrm_release_work);

	file->private_data = udev;
	nonseekable_open(inode, file);

	return 0;
}

static ssize_t utinydrm_write(struct file *file, const char __user *buffer,
			   size_t count, loff_t *ppos)
{
	struct utinydrm_device *udev = file->private_data;
	int ret, event_ret;

	dev_dbg(udev->tdev.drm.dev, "%s\n", __func__);

	if (!udev->initialized)
		return -EINVAL;

	if (!count)
		return 0;

	if (count != sizeof(int))
		return -EINVAL;

	if (copy_from_user(&event_ret, buffer, sizeof(int)))
		return -EFAULT;

	ret = mutex_lock_interruptible(&udev->mutex);
	if (ret)
		return ret;

	udev->event_ret = event_ret;
	complete(&udev->completion);

	mutex_unlock(&udev->mutex);

	return count;
}

static ssize_t utinydrm_read(struct file *file, char __user *buffer, size_t count,
			  loff_t *ppos)
{
	struct utinydrm_device *udev = file->private_data;
	ssize_t ret;

	dev_dbg(&udev->dev, "%s(count=%zu)\n", __func__, count);
	if (!count)
		return 0;

	do {
		ret = mutex_lock_interruptible(&udev->mutex);
		if (ret)
			return ret;

		if (!udev->ev && (file->f_flags & O_NONBLOCK)) {
			ret = -EAGAIN;
		} else if (udev->ev) {
			dev_dbg(&udev->dev, "%s udev->ev->length=%u\n", __func__, udev->ev->length);
			if (count < udev->ev->length)
				ret = -EINVAL;
			else if (copy_to_user(buffer, udev->ev, udev->ev->length))
				ret = -EFAULT;
			else
				ret = udev->ev->length;
			kfree(udev->ev);
			udev->ev = NULL;
		}

		mutex_unlock(&udev->mutex);

		dev_dbg(&udev->dev, "%s: ret=%d\n", __func__, ret);
		if (ret)
			break;

		if (!(file->f_flags & O_NONBLOCK))
			ret = wait_event_interruptible(udev->waitq, udev->ev);
		dev_dbg(&udev->dev, "%s: while: ret=%d\n", __func__, ret);
	} while (ret == 0);

	return ret;
}

static unsigned int utinydrm_poll(struct file *file, poll_table *wait)
{
	struct utinydrm_device *udev = file->private_data;

	dev_dbg(udev->tdev.drm.dev, "%s\n", __func__);
	poll_wait(file, &udev->waitq, wait);

	if (udev->ev)
		return POLLIN | POLLRDNORM;

	return 0;
}

static int utinydrm_release(struct inode *inode, struct file *file)
{
	struct utinydrm_device *udev = file->private_data;
	int i;

	dev_dbg(&udev->dev, "%s: refcount=%d\n", __func__, atomic_read(&udev->dev.kobj.kref.refcount));

	if (udev->initialized) {
		schedule_work(&udev->release_work);
	} else {
		dev_dbg(&udev->dev, "%s: refcount=%d\n", __func__, atomic_read(&udev->dev.kobj.kref.refcount));

		device_unregister(&udev->dev);
		for (i = atomic_read(&udev->dev.kobj.kref.refcount); i > 0; i--)
			put_device(&udev->dev);
	}

	return 0;
}

static long utinydrm_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct utinydrm_device *udev = file->private_data;
	struct utinydrm_dev_create dev_create;
	int ret = -EINVAL;

	switch (cmd) {
	case UTINYDRM_DEV_CREATE:

		if (copy_from_user(&dev_create, (void __user *)arg, sizeof(dev_create)))
			return -EFAULT;

		ret = utinydrm_register(udev, &dev_create);
		if (!ret) {
			udev->initialized = true;
			if (copy_to_user((void __user *)arg, &dev_create, sizeof(dev_create)))
				ret = -EFAULT;
		}

		break;
	}

	return ret;
}

static const struct file_operations utinydrm_fops = {
	.owner		= THIS_MODULE,
	.open		= utinydrm_open,
	.release	= utinydrm_release,
	.read		= utinydrm_read,
	.write		= utinydrm_write,
	.poll		= utinydrm_poll,

	.unlocked_ioctl	= utinydrm_ioctl,
//#ifdef CONFIG_COMPAT
//	.compat_ioctl	= utinydrm_compat_ioctl,
//#endif

	.llseek		= no_llseek,
};

static struct miscdevice utinydrm_misc = {
	.fops		= &utinydrm_fops,
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= "utinydrm",
};

static int __init utinydrm_init(void)
{
	return misc_register(&utinydrm_misc);
}
module_init(utinydrm_init);

static void __exit utinydrm_exit(void)
{
	misc_deregister(&utinydrm_misc);
}
module_exit(utinydrm_exit);

MODULE_AUTHOR("Noralf Trønnes");
MODULE_DESCRIPTION("Userspace driver for tinydrm");
MODULE_LICENSE("GPL");
