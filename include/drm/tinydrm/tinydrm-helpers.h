/*
 * Copyright (C) 2016 Noralf Trønnes
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __LINUX_TINYDRM_HELPERS_H
#define __LINUX_TINYDRM_HELPERS_H

struct backlight_device;
struct tinydrm_device;
struct drm_clip_rect;
struct dev_pm_ops;
struct spi_device;
struct device;

void tinydrm_merge_clips(struct drm_clip_rect *dst,
			 struct drm_clip_rect *src, unsigned int num_clips,
			 unsigned int flags, u32 max_width, u32 max_height);
void tinydrm_xrgb8888_to_rgb565(u32 *src, u16 *dst, unsigned int num_pixels);

#ifdef CONFIG_BACKLIGHT_CLASS_DEVICE
struct backlight_device *tinydrm_of_find_backlight(struct device *dev);
int tinydrm_enable_backlight(struct backlight_device *backlight);
void tinydrm_disable_backlight(struct backlight_device *backlight);
#else
static inline struct backlight_device *
tinydrm_of_find_backlight(struct device *dev)
{
	return NULL;
}

static inline int tinydrm_enable_backlight(struct backlight_device *backlight)
{
	return 0;
}

static inline void
tinydrm_disable_backlight(struct backlight_device *backlight)
{
}
#endif

extern const struct dev_pm_ops tinydrm_simple_pm_ops;
void tinydrm_spi_shutdown(struct spi_device *spi);

#endif /* __LINUX_TINYDRM_HELPERS_H */
