/*
 * MIPI Display Bus Interface (DBI) LCD controller support
 *
 * Copyright 2016 Noralf Trønnes
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __LINUX_MIPI_DBI_H
#define __LINUX_MIPI_DBI_H

#include <drm/tinydrm/tinydrm.h>

struct drm_gem_cma_object;
struct drm_framebuffer;
struct drm_clip_rect;
struct spi_device;
struct gpio_desc;
struct regulator;

/**
 * mipi_dbi - MIPI DBI controller
 * @tinydrm: tinydrm base
 * @reg: register map
 * @reset: Optional reset gpio
 * @rotation: initial rotation in degress Counter Clock Wise
 * @backlight: backlight device (optional)
 * @enable_delay_ms: Optional delay in milliseconds before turning on backlight
 * @regulator: power regulator (optional)
 */
struct mipi_dbi {
	struct tinydrm_device tinydrm;
	struct regmap *reg;
	struct gpio_desc *reset;
	unsigned int rotation;
	struct backlight_device *backlight;
	unsigned int enable_delay_ms;
	struct regulator *regulator;
};

static inline struct mipi_dbi *
mipi_dbi_from_tinydrm(struct tinydrm_device *tdev)
{
	return container_of(tdev, struct mipi_dbi, tinydrm);
}

struct regmap *mipi_dbi_spi_init(struct spi_device *spi, struct gpio_desc *dc,
				 bool write_only);
int mipi_dbi_init(struct device *dev, struct mipi_dbi *mipi,
		  const struct drm_simple_display_pipe_funcs *pipe_funcs,
		  struct drm_driver *driver,
		  const struct drm_display_mode *mode, unsigned int rotation);
void mipi_dbi_pipe_disable(struct drm_simple_display_pipe *pipe);
void mipi_dbi_hw_reset(struct mipi_dbi *mipi);
bool mipi_dbi_display_is_on(struct regmap *reg);

/**
 * mipi_dbi_write - Write command and optional parameter(s)
 * @cmd: Command
 * @...: Parameters
 */
#define mipi_dbi_write(reg, cmd, seq...) \
({ \
	u8 d[] = { seq }; \
	mipi_dbi_write_buf(reg, cmd, d, ARRAY_SIZE(d)); \
})

int mipi_dbi_write_buf(struct regmap *reg, unsigned int cmd,
		       const u8 *parameters, size_t num);

#ifdef CONFIG_DEBUG_FS
int mipi_dbi_debugfs_init(struct drm_minor *minor);
void mipi_dbi_debugfs_cleanup(struct drm_minor *minor);
#else
#define mipi_dbi_debugfs_init		NULL
#define mipi_dbi_debugfs_cleanup	NULL
#endif

#endif /* __LINUX_MIPI_DBI_H */
