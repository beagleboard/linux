/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2024 - Texas Instruments Incorporated
 *
 * Aradhya Bhatia <a-bhatia1@ti.com>
 */

#ifndef __TIDSS_OLDI_H__
#define __TIDSS_OLDI_H__

#include "tidss_drv.h"

struct tidss_oldi;

/* OLDI PORTS */
#define OLDI_INPUT_PORT		0
#define OLDI_OURPUT_PORT	1

/* Control MMR Registers */

/* Register offsets */
#define OLDI_PD_CTRL            0x100
#define OLDI_LB_CTRL            0x104

/* Power control bits */
#define OLDI_PWRDOWN_TX(n)	BIT(n)

/* LVDS Bandgap reference Enable/Disable */
#define OLDI_PWRDN_BG		BIT(8)

enum tidss_oldi_link_type {
	OLDI_MODE_UNSUPPORTED,
	OLDI_MODE_SINGLE_LINK,
	OLDI_MODE_CLONE_SINGLE_LINK,
	OLDI_MODE_DUAL_LINK,
	OLDI_MODE_SECONDARY,
};

enum oldi_mode_reg_val { SPWG_18 = 0, JEIDA_24 = 1, SPWG_24 = 2 };

struct oldi_bus_format {
	u32 bus_fmt;
	u32 data_width;
	enum oldi_mode_reg_val oldi_mode_reg_val;
	u32 input_bus_fmt;
};

int tidss_oldi_init(struct tidss_device *tidss);
void tidss_oldi_deinit(struct tidss_device *tidss);

#endif /* __TIDSS_OLDI_H__ */
