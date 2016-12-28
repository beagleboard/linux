/*
 * Copyright (C) 2016 Noralf Trønnes
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <drm/drmP.h>
#include <drm/tinydrm/tinydrm.h>
#include <linux/backlight.h>
#include <linux/pm.h>
#include <linux/spi/spi.h>

/**
 * tinydrm_merge_clips - merge clip rectangles
 * @dst: destination clip rectangle
 * @src: source clip rectangle(s)
 * @num_clips: number of @src clip rectangles
 * @flags: dirty fb ioctl flags
 * @max_width: maximum width of @dst
 * @max_height: maximum height of @dst
 *
 * This function merges @src clip rectangle(s) into @dst. If @src is NULL,
 * @max_width and @min_width is used to set a full @dst clip rectangle.
 */
void tinydrm_merge_clips(struct drm_clip_rect *dst,
			 struct drm_clip_rect *src, unsigned int num_clips,
			 unsigned int flags, u32 max_width, u32 max_height)
{
	unsigned int i;

	if (!src || !num_clips) {
		dst->x1 = 0;
		dst->x2 = max_width;
		dst->y1 = 0;
		dst->y2 = max_height;
		return;
	}

	dst->x1 = ~0;
	dst->y1 = ~0;
	dst->x2 = 0;
	dst->y2 = 0;

	for (i = 0; i < num_clips; i++) {
		if (flags & DRM_MODE_FB_DIRTY_ANNOTATE_COPY)
			i++;
		dst->x1 = min(dst->x1, src[i].x1);
		dst->x2 = max(dst->x2, src[i].x2);
		dst->y1 = min(dst->y1, src[i].y1);
		dst->y2 = max(dst->y2, src[i].y2);
	}

	if (dst->x2 > max_width || dst->y2 > max_height ||
	    dst->x1 >= dst->x2 || dst->y1 >= dst->y2) {
		DRM_DEBUG_KMS("Illegal clip: x1=%u, x2=%u, y1=%u, y2=%u\n",
			      dst->x1, dst->x2, dst->y1, dst->y2);
		dst->x1 = 0;
		dst->y1 = 0;
		dst->x2 = max_width;
		dst->y2 = max_height;
	}
}
EXPORT_SYMBOL(tinydrm_merge_clips);

/**
 * tinydrm_xrgb8888_to_rgb565 - convert xrgb8888 to rgb565
 * @src: xrgb8888 source buffer
 * @dst: rgb565 destination buffer
 * @num_pixels: number of pixels to copy
 *
 * Drivers can use this function for rgb565 devices that don't natively
 * support xrgb8888.
 */
void tinydrm_xrgb8888_to_rgb565(u32 *src, u16 *dst, unsigned int num_pixels)
{
	int i;

	for (i = 0; i < num_pixels; i++) {
		*dst = ((*src & 0x00F80000) >> 8) |
		       ((*src & 0x0000FC00) >> 5) |
		       ((*src & 0x000000F8) >> 3);
		src++;
		dst++;
	}
}
EXPORT_SYMBOL(tinydrm_xrgb8888_to_rgb565);

#ifdef CONFIG_BACKLIGHT_CLASS_DEVICE
/**
 * tinydrm_of_find_backlight - find backlight device in device-tree
 * @dev: device
 *
 * This function looks for a DT node pointed to by a property named 'backlight'
 * and uses of_find_backlight_by_node() to get the backlight device.
 * Additionally if the brightness property is zero, it is set to
 * max_brightness.
 *
 * Returns:
 * NULL if there's no backlight property.
 * Error pointer -EPROBE_DEFER if the DT node is found, but no backlight device
 * is found.
 * If the backlight device is found, a pointer to the structure is returned.
 */
struct backlight_device *tinydrm_of_find_backlight(struct device *dev)
{
	struct backlight_device *backlight;
	struct device_node *np;

	np = of_parse_phandle(dev->of_node, "backlight", 0);
	if (!np)
		return NULL;

	backlight = of_find_backlight_by_node(np);
	of_node_put(np);

	if (!backlight)
		return ERR_PTR(-EPROBE_DEFER);

	if (!backlight->props.brightness) {
		backlight->props.brightness = backlight->props.max_brightness;
		DRM_DEBUG_KMS("Backlight brightness set to %d\n",
			      backlight->props.brightness);
	}

	return backlight;
}
EXPORT_SYMBOL(tinydrm_of_find_backlight);

/**
 * tinydrm_enable_backlight - enable backlight helper
 * @backlight: backlight device
 *
 * Helper to enable backlight for use in &tinydrm_funcs ->enable callback
 * functions.
 */
int tinydrm_enable_backlight(struct backlight_device *backlight)
{
	unsigned int old_state;

	if (!backlight)
		return 0;

	old_state = backlight->props.state;
	backlight->props.state &= ~BL_CORE_SUSPENDED;
	DRM_DEBUG_KMS("Backlight state: 0x%x -> 0x%x\n", old_state,
		      backlight->props.state);

	return backlight_update_status(backlight);
}
EXPORT_SYMBOL(tinydrm_enable_backlight);

/**
 * tinydrm_disable_backlight - disable backlight helper
 * @backlight: backlight device
 *
 * Helper to disable backlight for use in &tinydrm_funcs ->disable callback
 * functions.
 */
void tinydrm_disable_backlight(struct backlight_device *backlight)
{
	unsigned int old_state;
	int ret;

	if (!backlight)
		return;

	old_state = backlight->props.state;
	backlight->props.state |= BL_CORE_SUSPENDED;
	DRM_DEBUG_KMS("Backlight state: 0x%x -> 0x%x\n", old_state,
		      backlight->props.state);
	ret = backlight_update_status(backlight);
	if (ret)
		DRM_ERROR("Failed to disable backlight %d\n", ret);
}
EXPORT_SYMBOL(tinydrm_disable_backlight);
#endif

static int __maybe_unused tinydrm_pm_suspend(struct device *dev)
{
	struct tinydrm_device *tdev = dev_get_drvdata(dev);

	return tdev ? tinydrm_suspend(tdev) : -EINVAL;
}

static int __maybe_unused tinydrm_pm_resume(struct device *dev)
{
	struct tinydrm_device *tdev = dev_get_drvdata(dev);

	return tdev ? tinydrm_resume(tdev) : -EINVAL;
}

/*
 * tinydrm_simple_pm_ops - tinydrm simple power management operations
 *
 * This provides simple suspend/resume power management and can be assigned
 * to the drivers &device_driver->pm property. &tinydrm_device must be set
 * on the device using dev_set_drvdata() or equivalent.
 */
const struct dev_pm_ops tinydrm_simple_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(tinydrm_pm_suspend, tinydrm_pm_resume)
};
EXPORT_SYMBOL(tinydrm_simple_pm_ops);

/**
 * tinydrm_spi_shutdown - SPI driver shutdown callback helper
 * @spi: SPI device
 *
 * This is a helper function for the &spi_driver ->shutdown callback which
 * makes sure that the tinydrm device is disabled and unprepared on shutdown.
 * &tinydrm_device must be set on the device using spi_set_drvdata().
 */
void tinydrm_spi_shutdown(struct spi_device *spi)
{
	struct tinydrm_device *tdev = spi_get_drvdata(spi);

	if (tdev)
		tinydrm_shutdown(tdev);
}
EXPORT_SYMBOL(tinydrm_spi_shutdown);
