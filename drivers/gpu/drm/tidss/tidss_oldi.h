/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2024 - Texas Instruments Incorporated
 *
 * Aradhya Bhatia <a-bhatia1@ti.com>
 */

#ifndef __TIDSS_OLDI_H__
#define __TIDSS_OLDI_H__

#include <linux/media-bus-format.h>

#include "tidss_drv.h"
#include "tidss_dispc.h"

struct tidss_oldi;

/* OLDI Instances */
#define OLDI(n)		n

/* OLDI PORTS */
#define OLDI_INPUT_PORT		0
#define OLDI_OURPUT_PORT	1

/* OLDI Config Bits */
#define OLDI_ENABLE		BIT(0)
#define OLDI_MAP		(BIT(1) | BIT(2) | BIT(3))
#define OLDI_SRC		BIT(4)
#define OLDI_CLONE_MODE		BIT(5)
#define OLDI_MASTERSLAVE	BIT(6)
#define OLDI_DEPOL		BIT(7)
#define OLDI_MSB		BIT(8)
#define OLDI_LBEN		BIT(9)
#define OLDI_LBDATA		BIT(10)
#define OLDI_DUALMODESYNC	BIT(11)
#define OLDI_SOFTRST		BIT(12)
#define OLDI_TPATCFG		BIT(13)

/* Control MMR Register */

/* Register offsets */
#define OLDI_PD_CTRL            0x100
#define OLDI_LB_CTRL            0x104

/* Power control bits */
#define OLDI_PWRDN_TX(n)	BIT(n)

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
