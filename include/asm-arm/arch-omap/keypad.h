/*
 *  linux/include/asm-arm/arch-omap/keypad.h
 *
 *  Copyright (C) 2006 Komal Shah <komal_shah802003@yahoo.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef ASMARM_ARCH_KEYPAD_H
#define ASMARM_ARCH_KEYPAD_H

struct omap_kp_platform_data {
	int rows;
	int cols;
	int *keymap;
	unsigned int rep:1;
	/* specific to OMAP242x*/
	unsigned int *row_gpios;
	unsigned int *col_gpios;
};

#define KEY(col, row, val) (((col) << 28) | ((row) << 24) | (val))

#endif

