/*
 * Copyright (C) 2016 Noralf Trønnes
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <drm/drmP.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/tinydrm/tinydrm.h>
#include <linux/console.h>
#include <linux/device.h>
#include <linux/dma-buf.h>

/**
 * DOC: Overview
 *
 * This library provides helpers for displays with onboard graphics memory
 * connected through a slow interface.
 *
 * In order for the display to turn off at shutdown, the device driver shutdown
 * callback has to be set. This function should call tinydrm_shutdown().
 */

/**
 * tinydrm_lastclose - DRM .lastclose() helper
 * @drm: DRM device
 *
 * This function ensures that fbdev is restored when drm_lastclose() is called
 * on the last drm_release(). If fbdev is disabled the pipeline is disabled
 * instead. tinydrm drivers should use this as their &drm_driver->lastclose
 * callback.
 */
void tinydrm_lastclose(struct drm_device *drm)
{
	struct tinydrm_device *tdev = drm_to_tinydrm(drm);

	DRM_DEBUG_KMS("\n");
	if (tdev->fbdev_used)
		drm_fbdev_cma_restore_mode(tdev->fbdev_cma);
	else
		drm_crtc_force_disable_all(drm);
}
EXPORT_SYMBOL(tinydrm_lastclose);

/**
 * tinydrm_gem_cma_free_object - free resources associated with a CMA GEM
 *                               object
 * @gem_obj: GEM object to free
 *
 * This function frees the backing memory of the CMA GEM object, cleans up the
 * GEM object state and frees the memory used to store the object itself using
 * drm_gem_cma_free_object(). It also handles PRIME buffers which has the kernel
 * virtual address set by tinydrm_gem_cma_prime_import_sg_table(). tinydrm
 * drivers should set this as their &drm_driver->gem_free_object callback.
 */
void tinydrm_gem_cma_free_object(struct drm_gem_object *gem_obj)
{
	if (gem_obj->import_attach) {
		struct drm_gem_cma_object *cma_obj;

		cma_obj = to_drm_gem_cma_obj(gem_obj);
		dma_buf_vunmap(gem_obj->import_attach->dmabuf, cma_obj->vaddr);
		cma_obj->vaddr = NULL;
	}

	drm_gem_cma_free_object(gem_obj);
}
EXPORT_SYMBOL_GPL(tinydrm_gem_cma_free_object);

/**
 * tinydrm_gem_cma_prime_import_sg_table - produce a CMA GEM object from
 *     another driver's scatter/gather table of pinned pages
 * @drm: device to import into
 * @attach: DMA-BUF attachment
 * @sgt: scatter/gather table of pinned pages
 *
 * This function imports a scatter/gather table exported via DMA-BUF by
 * another driver using drm_gem_cma_prime_import_sg_table(). It also sets the
 * kernel virtual address on the CMA object. tinydrm drivers should use this
 * as their &drm_driver->gem_prime_import_sg_table callback.
 *
 * Returns:
 * A pointer to a newly created GEM object or an ERR_PTR-encoded negative
 * error code on failure.
 */
struct drm_gem_object *
tinydrm_gem_cma_prime_import_sg_table(struct drm_device *drm,
				      struct dma_buf_attachment *attach,
				      struct sg_table *sgt)
{
	struct drm_gem_cma_object *cma_obj;
	struct drm_gem_object *obj;
	void *vaddr;

	vaddr = dma_buf_vmap(attach->dmabuf);
	if (!vaddr) {
		DRM_ERROR("Failed to vmap PRIME buffer\n");
		return ERR_PTR(-ENOMEM);
	}

	obj = drm_gem_cma_prime_import_sg_table(drm, attach, sgt);
	if (IS_ERR(obj)) {
		dma_buf_vunmap(attach->dmabuf, vaddr);
		return obj;
	}

	cma_obj = to_drm_gem_cma_obj(obj);
	cma_obj->vaddr = vaddr;

	return obj;
}
EXPORT_SYMBOL(tinydrm_gem_cma_prime_import_sg_table);

const struct file_operations tinydrm_fops = {
	.owner		= THIS_MODULE,
	.open		= drm_open,
	.release	= drm_release,
	.unlocked_ioctl	= drm_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= drm_compat_ioctl,
#endif
	.poll		= drm_poll,
	.read		= drm_read,
	.llseek		= no_llseek,
	.mmap		= drm_gem_cma_mmap,
};
EXPORT_SYMBOL(tinydrm_fops);

static const struct drm_mode_config_funcs tinydrm_mode_config_funcs = {
	.fb_create = tinydrm_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static void tinydrm_dirty_work(struct work_struct *work)
{
	struct tinydrm_device *tdev = container_of(work, struct tinydrm_device,
						   dirty_work);
	struct drm_framebuffer *fb = tdev->pipe.plane.fb;

	if (fb && fb->funcs->dirty)
		fb->funcs->dirty(fb, NULL, 0, 0, NULL, 0);
}

static int tinydrm_init(struct device *parent, struct tinydrm_device *tdev,
			const struct drm_framebuffer_funcs *fb_funcs,
			struct drm_driver *driver)
{
	struct drm_device *drm = &tdev->drm;
	int ret;

	INIT_WORK(&tdev->dirty_work, tinydrm_dirty_work);
	mutex_init(&tdev->dev_lock);
	tdev->fb_funcs = fb_funcs;

	ret = drm_dev_init(drm, driver, parent);
	if (ret)
		return ret;

	drm_mode_config_init(drm);
	drm->mode_config.funcs = &tinydrm_mode_config_funcs;

	return 0;
}

static void tinydrm_fini(struct tinydrm_device *tdev)
{
	struct drm_device *drm = &tdev->drm;

	DRM_DEBUG_KMS("\n");

	drm_mode_config_cleanup(drm);
	drm_dev_unref(drm);
	mutex_destroy(&tdev->dev_lock);
}

static void devm_tinydrm_release(struct device *dev, void *res)
{
	tinydrm_fini(*(struct tinydrm_device **)res);
}

/**
 * devm_tinydrm_init - Initialize tinydrm device
 * @parent: Parent device object
 * @tdev: tinydrm device
 * @fb_funcs: Framebuffer functions
 * @driver: DRM driver
 *
 * This function initializes @tdev, the underlying DRM device and it's
 * mode_config. Additionally it sets &drm_mode_config_funcs using
 * tinydrm_fb_create() for framebuffer creation.
 * Resources will be automatically freed on driver detach (devres) using
 * drm_mode_config_cleanup() and drm_dev_unref().
 *
 * Returns:
 * Zero on success, negative error code on failure.
 */
int devm_tinydrm_init(struct device *parent, struct tinydrm_device *tdev,
		      const struct drm_framebuffer_funcs *fb_funcs,
		      struct drm_driver *driver)
{
	struct tinydrm_device **ptr;
	int ret;

	ptr = devres_alloc(devm_tinydrm_release, sizeof(*ptr), GFP_KERNEL);
	if (!ptr)
		return -ENOMEM;

	ret = tinydrm_init(parent, tdev, fb_funcs, driver);
	if (ret) {
		devres_free(ptr);
		return ret;
	}

	*ptr = tdev;
	devres_add(parent, ptr);

	return 0;
}
EXPORT_SYMBOL(devm_tinydrm_init);

static int tinydrm_register(struct tinydrm_device *tdev)
{
	struct drm_device *drm = &tdev->drm;
	int ret;

	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;

	ret = tinydrm_fbdev_init(tdev);
	if (ret)
		DRM_ERROR("Failed to initialize fbdev: %d\n", ret);

	return 0;
}

static void tinydrm_unregister(struct tinydrm_device *tdev)
{
	struct drm_device *drm = &tdev->drm;

	DRM_DEBUG_KMS("\n");

	drm_crtc_force_disable_all(drm);
	cancel_work_sync(&tdev->dirty_work);
	tinydrm_fbdev_fini(tdev);
	drm_dev_unregister(drm);
}

static void devm_tinydrm_register_release(struct device *dev, void *res)
{
	tinydrm_unregister(*(struct tinydrm_device **)res);
}

/**
 * devm_tinydrm_register - Register tinydrm device
 * @tdev: tinydrm device
 *
 * This function registers the underlying DRM device, connectors and fbdev.
 * These resources will be automatically unregistered on driver detach (devres)
 * and the display pipeline will be disabled.
 *
 * Returns:
 * Zero on success, negative error code on failure.
 */
int devm_tinydrm_register(struct tinydrm_device *tdev)
{
	struct device *dev = tdev->drm.dev;
	struct tinydrm_device **ptr;
	int ret;

	ptr = devres_alloc(devm_tinydrm_register_release, sizeof(*ptr),
			   GFP_KERNEL);
	if (!ptr)
		return -ENOMEM;

	ret = tinydrm_register(tdev);
	if (ret) {
		devres_free(ptr);
		return ret;
	}

	*ptr = tdev;
	devres_add(dev, ptr);

	return 0;
}
EXPORT_SYMBOL(devm_tinydrm_register);

static int devm_tinydrm_register_match(struct device *dev, void *res, void *data)
{
	struct tinydrm_device **ptr = res, *tdev = data;

        return *ptr == tdev;
}

void devm_tinydrm_unregister(struct tinydrm_device *tdev)
{
	struct device *dev = tdev->drm.dev;

	if (WARN_ON(!dev))
		return;

	WARN_ON(devres_release(dev, devm_tinydrm_register_release, devm_tinydrm_register_match, tdev));
}
EXPORT_SYMBOL(devm_tinydrm_unregister);

/**
 * tinydrm_shutdown - Shutdown tinydrm
 * @tdev: tinydrm device
 *
 * This function makes sure that tinydrm is disabled and unprepared.
 * Used by drivers in their shutdown callback to turn off the display
 * on machine shutdown and reboot.
 */
void tinydrm_shutdown(struct tinydrm_device *tdev)
{
	struct drm_device *drm = &tdev->drm;

	drm_crtc_force_disable_all(drm);
}
EXPORT_SYMBOL(tinydrm_shutdown);

static void tinydrm_fbdev_set_suspend(struct tinydrm_device *tdev, int state)
{
	if (!tdev->fbdev_helper)
		return;

	console_lock();
	drm_fb_helper_set_suspend(tdev->fbdev_helper, state);
	console_unlock();
}

/**
 * tinydrm_suspend - Suspend tinydrm
 * @tdev: tinydrm device
 *
 * Used in driver PM operations to suspend tinydrm.
 * Suspends fbdev and DRM.
 * Resume with tinydrm_resume().
 *
 * Returns:
 * Zero on success, negative error code on failure.
 */
int tinydrm_suspend(struct tinydrm_device *tdev)
{
	struct drm_device *drm = &tdev->drm;
	struct drm_atomic_state *state;

	if (tdev->suspend_state) {
		DRM_ERROR("Failed to suspend: state already set\n");
		return -EINVAL;
	}

	tinydrm_fbdev_set_suspend(tdev, 1);
	state = drm_atomic_helper_suspend(drm);
	if (IS_ERR(state)) {
		tinydrm_fbdev_set_suspend(tdev, 0);
		return PTR_ERR(state);
	}

	tdev->suspend_state = state;

	return 0;
}
EXPORT_SYMBOL(tinydrm_suspend);

/**
 * tinydrm_resume - Resume tinydrm
 * @tdev: tinydrm device
 *
 * Used in driver PM operations to resume tinydrm.
 * Suspend with tinydrm_suspend().
 *
 * Returns:
 * Zero on success, negative error code on failure.
 */
int tinydrm_resume(struct tinydrm_device *tdev)
{
	struct drm_atomic_state *state = tdev->suspend_state;
	struct drm_device *drm = &tdev->drm;
	int ret;

	if (!state) {
		DRM_ERROR("Failed to resume: state is not set\n");
		return -EINVAL;
	}

	tdev->suspend_state = NULL;

	ret = drm_atomic_helper_resume(drm, state);
	if (ret) {
		DRM_ERROR("Error resuming state: %d\n", ret);
		drm_atomic_state_free(state);
		return ret;
	}

	tinydrm_fbdev_set_suspend(tdev, 0);

	return 0;
}
EXPORT_SYMBOL(tinydrm_resume);

MODULE_LICENSE("GPL");
