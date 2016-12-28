/*
 * Copyright (C) 2016 Noralf Trønnes
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __LINUX_TINYDRM_H
#define __LINUX_TINYDRM_H

#include <drm/drm_crtc.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_simple_kms_helper.h>

struct tinydrm_debugfs_dirty;

/**
 * struct tinydrm_device - tinydrm device
 * @drm: DRM device
 * @pipe: Display pipe structure
 * @dirty_work: framebuffer flusher
 * @dev_lock: serializes device access and protects
 *            prepared/enabled state changes
 * @prepared: device prepared state (prepared for framebuffer flushing)
 * @enabled: device enabled state (display is on)
 * @fbdev_cma: fbdev CMA structure
 * @fbdev_helper: fbdev helper (from the private fbdev CMA structure).
 * @fbdev_used: fbdev has actually been used
 * @suspend_state: atomic state when suspended
 * @debugfs_dirty: debugfs dirty file control structure
 */
struct tinydrm_device {
	struct drm_device drm;
	struct drm_simple_display_pipe pipe;
	struct work_struct dirty_work;
	struct mutex dev_lock;
	bool prepared;
	bool enabled;
	struct drm_fbdev_cma *fbdev_cma;
	struct drm_fb_helper *fbdev_helper;
	bool fbdev_used;
	struct drm_atomic_state *suspend_state;
	struct tinydrm_debugfs_dirty *debugfs_dirty;
/* private: */
	const struct drm_framebuffer_funcs *fb_funcs;
};

static inline struct tinydrm_device *
drm_to_tinydrm(struct drm_device *drm)
{
	return container_of(drm, struct tinydrm_device, drm);
}

static inline struct tinydrm_device *
pipe_to_tinydrm(struct drm_simple_display_pipe *pipe)
{
	return container_of(pipe, struct tinydrm_device, pipe);
}

/*
 * TINYDRM_GEM_DRIVER_OPS - default tinydrm gem operations
 *
 * This macro provides a shortcut for setting the tinydrm GEM operations in
 * the &drm_driver structure.
 */
#define TINYDRM_GEM_DRIVER_OPS \
	.gem_free_object	= tinydrm_gem_cma_free_object, \
	.gem_vm_ops		= &drm_gem_cma_vm_ops, \
	.prime_handle_to_fd	= drm_gem_prime_handle_to_fd, \
	.prime_fd_to_handle	= drm_gem_prime_fd_to_handle, \
	.gem_prime_import	= drm_gem_prime_import, \
	.gem_prime_export	= drm_gem_prime_export, \
	.gem_prime_get_sg_table	= drm_gem_cma_prime_get_sg_table, \
	.gem_prime_import_sg_table = tinydrm_gem_cma_prime_import_sg_table, \
	.gem_prime_vmap		= drm_gem_cma_prime_vmap, \
	.gem_prime_vunmap	= drm_gem_cma_prime_vunmap, \
	.gem_prime_mmap		= drm_gem_cma_prime_mmap, \
	.dumb_create		= drm_gem_cma_dumb_create, \
	.dumb_map_offset	= drm_gem_cma_dumb_map_offset, \
	.dumb_destroy		= drm_gem_dumb_destroy, \
	.fops			= &tinydrm_fops

/**
 * TINYDRM_MODE - tinydrm display mode
 * @hd: horizontal resolution, width
 * @vd: vertical resolution, height
 * @hd_mm: display width in millimeters
 * @vd_mm: display height in millimeters
 *
 * This macro creates a &drm_display_mode for use with tinydrm.
 */
#define TINYDRM_MODE(hd, vd, hd_mm, vd_mm) \
	.hdisplay = (hd), \
	.hsync_start = (hd), \
	.hsync_end = (hd), \
	.htotal = (hd), \
	.vdisplay = (vd), \
	.vsync_start = (vd), \
	.vsync_end = (vd), \
	.vtotal = (vd), \
	.width_mm = (hd_mm), \
	.height_mm = (vd_mm), \
	.type = DRM_MODE_TYPE_DRIVER, \
	.clock = 1 /* pass validation */

extern const struct file_operations tinydrm_fops;
void tinydrm_lastclose(struct drm_device *drm);
void tinydrm_gem_cma_free_object(struct drm_gem_object *gem_obj);
struct drm_gem_object *
tinydrm_gem_cma_prime_import_sg_table(struct drm_device *drm,
				      struct dma_buf_attachment *attach,
				      struct sg_table *sgt);
struct drm_framebuffer *
tinydrm_fb_create(struct drm_device *drm, struct drm_file *file_priv,
		  const struct drm_mode_fb_cmd2 *mode_cmd);
bool tinydrm_check_dirty(struct drm_framebuffer *fb,
			 struct drm_clip_rect **clips,
			 unsigned int *num_clips);
struct drm_connector *
tinydrm_connector_create(struct drm_device *drm,
			 const struct drm_display_mode *mode,
			 int connector_type);
void tinydrm_display_pipe_update(struct drm_simple_display_pipe *pipe,
				 struct drm_plane_state *old_state);
int
tinydrm_display_pipe_init(struct tinydrm_device *tdev,
			  const struct drm_simple_display_pipe_funcs *funcs,
			  int connector_type,
			  const uint32_t *formats,
			  unsigned int format_count,
			  const struct drm_display_mode *mode,
			  unsigned int rotation);
int devm_tinydrm_init(struct device *parent, struct tinydrm_device *tdev,
		      const struct drm_framebuffer_funcs *fb_funcs,
		      struct drm_driver *driver);
int devm_tinydrm_register(struct tinydrm_device *tdev);
void devm_tinydrm_unregister(struct tinydrm_device *tdev);
void tinydrm_shutdown(struct tinydrm_device *tdev);
int tinydrm_suspend(struct tinydrm_device *tdev);
int tinydrm_resume(struct tinydrm_device *tdev);

int tinydrm_fbdev_init(struct tinydrm_device *tdev);
void tinydrm_fbdev_fini(struct tinydrm_device *tdev);

#ifdef CONFIG_DEBUG_FS
int tinydrm_debugfs_init(struct drm_minor *minor);
void tinydrm_debugfs_dirty_begin(struct tinydrm_device *tdev,
				 struct drm_framebuffer *fb,
				 const struct drm_clip_rect *clip);
void tinydrm_debugfs_dirty_end(struct tinydrm_device *tdev, size_t len,
			       unsigned int bits_per_pixel);
void tinydrm_debugfs_cleanup(struct drm_minor *minor);
int tinydrm_debugfs_dirty_init(struct tinydrm_device *tdev);
#else
int tinydrm_debugfs_dirty_init(struct tinydrm_device *tdev)
{
	return 0;
}

void tinydrm_debugfs_dirty_begin(struct tinydrm_device *tdev,
				 struct drm_framebuffer *fb,
				 const struct drm_clip_rect *clip)
{
}

void tinydrm_debugfs_dirty_end(struct tinydrm_device *tdev, size_t len,
			       unsigned int bits_per_pixel)
{
}

#define tinydrm_debugfs_init	NULL
#define tinydrm_debugfs_cleanup	NULL
#endif /* CONFIG_DEBUG_FS */

#endif /* __LINUX_TINYDRM_H */
