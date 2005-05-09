/*
 * linux/drivers/ssi/omap-tsc2101.h
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

extern u16 omap_tsc2101_read(int page, u8 address);
extern void omap_tsc2101_reads(int page, u8 startaddress, u16 * data,
			       int numregs);
extern void omap_tsc2101_write(int page, u8 address, u16 data);

extern void omap_tsc2101_disable(void);
extern int omap_tsc2101_enable(void);

#endif
