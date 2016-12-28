/*
 * Copyright (C) 2016 Noralf Trønnes
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/tinydrm/tinydrm.h>

/**
 * DOC: Framebuffer
 *
 * The tinydrm &drm_framebuffer is backed by a &drm_gem_cma_object buffer
 * object. Userspace creates this buffer by calling the
 * DRM_IOCTL_MODE_CREATE_DUMB ioctl. To flush the buffer to the display,
 * userpace calls the DRM_IOCTL_MODE_DIRTYFB ioctl on the framebuffer which
 * in turn calls the &drm_framebuffer_funcs->dirty callback.
 * This functionality is available by using tinydrm_fb_create() as the
 * &drm_mode_config_funcs->fb_create callback which devm_tinydrm_init() does.
 */

static unsigned int fbdefio_delay;
module_param(fbdefio_delay, uint, 0);
MODULE_PARM_DESC(fbdefio_delay, "fbdev deferred io delay in milliseconds");

/**
 * tinydrm_check_dirty - check before flushing framebuffer
 * @fb: framebuffer
 * @clips: pointer to dirty clip rectangles array
 * @num_clips: pointer to number of clips
 *
 * This function checks that the device is prepared and that @fb is the
 * framebuffer set on the plane. If the device hasn't been enabled, which
 * makes this the first flush, do flush everything.
 * Caller has to hold the dev_lock.
 *
 * Returns:
 * True if the dirty call can proceed, false otherwise.
 */
bool tinydrm_check_dirty(struct drm_framebuffer *fb,
			 struct drm_clip_rect **clips, unsigned int *num_clips)
{
	struct tinydrm_device *tdev = drm_to_tinydrm(fb->dev);

	if (!tdev->prepared)
		return false;

	/* fbdev can flush even when we're not interested */
	if (tdev->pipe.plane.fb != fb)
		return false;

	/* Make sure to flush everything the first time */
	if (!tdev->enabled) {
		*clips = NULL;
		*num_clips = 0;
	}

	return true;
}
EXPORT_SYMBOL(tinydrm_check_dirty);

/**
 * tinydrm_fb_create - tinydrm .fb_create() helper
 * @drm: DRM device
 * @file_priv: DRM file info
 * @mode_cmd: metadata from the userspace fb creation request
 *
 * Helper for the &drm_mode_config_funcs->fb_create callback.
 * It sets up a &drm_framebuffer backed by the &drm_gem_cma_object buffer
 * object provided in @mode_cmd.
 */
struct drm_framebuffer *
tinydrm_fb_create(struct drm_device *drm, struct drm_file *file_priv,
		  const struct drm_mode_fb_cmd2 *mode_cmd)
{
	struct tinydrm_device *tdev = drm_to_tinydrm(drm);
	struct drm_framebuffer *fb;

	fb = drm_fb_cma_create_with_funcs(drm, file_priv, mode_cmd,
					  tdev->fb_funcs);
	if (!IS_ERR(fb))
		DRM_DEBUG_KMS("[FB:%d] pixel_format: %s\n", fb->base.id,
			      drm_get_format_name(fb->pixel_format));

	return fb;
}
EXPORT_SYMBOL(tinydrm_fb_create);

/**
 * DOC: fbdev emulation
 *
 * tinydrm provides fbdev emulation using the drm_fb_cma_helper library.
 * It is backed by it's own &drm_framebuffer and CMA buffer object.
 * Framebuffer flushing is handled by the fb helper library which in turn
 * calls the dirty callback on the framebuffer. This callback is part of
 * &drm_framebuffer_funcs which is one of the arguments to devm_tinydrm_init().
 * fbdev support is initialized using tinydrm_fbdev_init().
 *
 * The tinydrm_lastclose() function ensures that fbdev operation is restored
 * when userspace closes the drm device.
 */

static int tinydrm_fbdev_create(struct drm_fb_helper *helper,
				struct drm_fb_helper_surface_size *sizes)
{
	struct tinydrm_device *tdev = drm_to_tinydrm(helper->dev);
	int ret;

	ret = drm_fbdev_cma_create_with_funcs(helper, sizes, tdev->fb_funcs);
	if (ret)
		return ret;

	strncpy(helper->fbdev->fix.id, helper->dev->driver->name, 16);
	tdev->fbdev_helper = helper;

	if (fbdefio_delay) {
		unsigned long delay;

		delay = msecs_to_jiffies(fbdefio_delay);
		helper->fbdev->fbdefio->delay = delay ? delay : 1;
	}

	DRM_DEBUG_KMS("fbdev: [FB:%d] pixel_format=%s, fbdefio->delay=%ums\n",
		      helper->fb->base.id,
		      drm_get_format_name(helper->fb->pixel_format),
		      jiffies_to_msecs(helper->fbdev->fbdefio->delay));

	return 0;
}

static const struct drm_fb_helper_funcs tinydrm_fb_helper_funcs = {
	.fb_probe = tinydrm_fbdev_create,
};

/**
 * tinydrm_fbdev_init - initialize tinydrm fbdev emulation
 * @tdev: tinydrm device
 *
 * Initialize tinydrm fbdev emulation. Tear down with tinydrm_fbdev_fini().
 * If &mode_config->preferred_depth is set it is used as preferred bpp.
 */
int tinydrm_fbdev_init(struct tinydrm_device *tdev)
{
	struct drm_device *drm = &tdev->drm;
	struct drm_fbdev_cma *fbdev;
	int bpp;

	DRM_DEBUG_KMS("\n");

	bpp = drm->mode_config.preferred_depth;
	fbdev = drm_fbdev_cma_init_with_funcs(drm, bpp ? bpp : 32,
					      drm->mode_config.num_crtc,
					      drm->mode_config.num_connector,
					      &tinydrm_fb_helper_funcs);
	if (IS_ERR(fbdev))
		return PTR_ERR(fbdev);

	tdev->fbdev_cma = fbdev;

	return 0;
}
EXPORT_SYMBOL(tinydrm_fbdev_init);

/**
 * tinydrm_fbdev_fini - finalize tinydrm fbdev emulation
 * @tdev: tinydrm device
 *
 * This function tears down the fbdev emulation
 */
void tinydrm_fbdev_fini(struct tinydrm_device *tdev)
{
	drm_fbdev_cma_fini(tdev->fbdev_cma);
	tdev->fbdev_cma = NULL;
	tdev->fbdev_helper = NULL;
}
EXPORT_SYMBOL(tinydrm_fbdev_fini);
