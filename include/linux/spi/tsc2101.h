/*
 * include/linux/spi/tsc2101.h
 *
 * TSC2101 codec interface driver for the OMAP platform
 *
 * Copyright (C) 2004 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * History:
 *
 * 2004/11/07   Nishanth Menon - Provided common hooks for Audio and Touchscreen
 */

#ifndef __OMAP_TSC2101_H
#define __OMAP_TSC2101_H

#include <linux/spi/spi.h>

struct tsc2101_platform_data {
	int	(*init)(struct spi_device *spi);
	void	(*cleanup)(struct spi_device *spi);
	void	(*enable_mclk)(struct spi_device *spi);
	void	(*disable_mclk)(struct spi_device *spi);
};

extern int tsc2101_read_sync(struct spi_device *spi, int page, u8 address);
extern int tsc2101_reads_sync(struct spi_device *spi, int page,
			       u8 startaddress, u16 * data, int numregs);
extern int tsc2101_write_sync(struct spi_device *spi, int page, u8 address,
			       u16 data);

extern int tsc2101_enable_mclk(struct spi_device *spi);
extern void tsc2101_disable_mclk(struct spi_device *spi);

#endif

