/*
 * Copyright (C) 2016 Noralf Trønnes
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __LINUX_TINYDRM_REGMAP_H
#define __LINUX_TINYDRM_REGMAP_H

#include <linux/regmap.h>

struct drm_framebuffer;
struct drm_clip_rect;
struct spi_transfer;
struct spi_message;
struct spi_device;

int tinydrm_regmap_flush_rgb565(struct regmap *reg, u32 regnr,
				struct drm_framebuffer *fb, void *vmem,
				struct drm_clip_rect *clip);
size_t tinydrm_spi_max_transfer_size(struct spi_device *spi, size_t max_len);
bool tinydrm_spi_bpw_supported(struct spi_device *spi, u8 bpw);
int tinydrm_spi_transfer(struct spi_device *spi, u32 speed_hz,
			 struct spi_transfer *header, u8 bpw, const void *buf,
			 size_t len, u16 *swap_buf, size_t max_chunk);
void tinydrm_debug_reg_write(const void *reg, size_t reg_len, const void *val,
			     size_t val_len, size_t val_width);
void _tinydrm_dbg_spi_message(struct spi_device *spi, struct spi_message *m);

/**
 * tinydrm_get_machine_endian - Get machine endianness
 *
 * Returns:
 * REGMAP_ENDIAN_LITTLE or REGMAP_ENDIAN_BIG
 */
static inline enum regmap_endian tinydrm_get_machine_endian(void)
{
#if defined(__LITTLE_ENDIAN)
	return REGMAP_ENDIAN_LITTLE;
#else
	return REGMAP_ENDIAN_BIG;
#endif
}

#if defined(DEBUG)
/**
 * TINYDRM_DEBUG_REG_WRITE - Print info about register write
 * @reg: Register number buffer
 * @reg_len: Length of @reg buffer
 * @val: Value buffer
 * @val_len: Length of @val buffer
 * @val_width: Word width of @val buffer
 *
 * This macro prints info to the log about a register write. Can be used in
 * &regmap_bus ->gather_write functions. It's a wrapper around
 * tinydrm_debug_reg_write().
 * DEBUG has to be defined for this macro to be enabled alongside setting
 * the DRM_UT_CORE bit of drm_debug.
 */
#define TINYDRM_DEBUG_REG_WRITE(reg, reg_len, val, val_len, val_width) \
	do { \
		if (unlikely(drm_debug & DRM_UT_CORE)) \
			tinydrm_debug_reg_write(reg, reg_len, \
						val, val_len, val_width); \
	} while (0)

/**
 * tinydrm_dbg_spi_message - Dump SPI message
 * @spi: SPI device
 * @m: SPI message
 *
 * Dumps info about the transfers in a SPI message including start of buffers.
 * DEBUG has to be defined for this function to be enabled alongside setting
 * the DRM_UT_CORE bit of drm_debug.
 */
static inline void tinydrm_dbg_spi_message(struct spi_device *spi,
					   struct spi_message *m)
{
	if (drm_debug & DRM_UT_CORE)
		_tinydrm_dbg_spi_message(spi, m);
}
#else
#define TINYDRM_DEBUG_REG_WRITE(reg, reg_len, val, val_len, val_width) \
	do { \
		if (0) \
			tinydrm_debug_reg_write(reg, reg_len, \
						val, val_len, val_width); \
	} while (0)

static inline void tinydrm_dbg_spi_message(struct spi_device *spi,
					   struct spi_message *m)
{
}
#endif

#endif /* __LINUX_TINYDRM_REGMAP_H */
