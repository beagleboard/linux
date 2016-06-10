/*
 * Copyright (C) 2016 Texas Instruments Ltd
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __OMAP2_DISPC6_REG_H
#define __OMAP2_DISPC6_REG_H

/* COMMON */

#define DISPC_REVISION			0x000
#define DISPC_SYSCONFIG			0x004
#define DISPC_SYSSTATUS			0x008

#define DISPC_IRQ_EOI			0x020
#define DISPC_IRQSTATUS_RAW		0x024
#define DISPC_IRQSTATUS			0x028
#define DISPC_IRQENABLE_SET		0x02c
#define DISPC_IRQENABLE_CLR		0x030
#define DISPC_IRQWAKEEN			0x034

#define DISPC_GLOBAL_MFLAG_ATTRIBUTE	0x040
#define DISPC_GLOBAL_BUFFER		0x044
#define DISPC_BA0_FLIPIMMEDIATE_EN	0x048

#define DISPC_DBG_CONTROL		0x04c
#define DISPC_DBG_STATUS		0x050

#define DISPC_CLKGATING_DISABLE		0x054

/* VID */

#define DISPC_VID_ACCUH_0		0x0
#define DISPC_VID_ACCUH_1		0x4
#define DISPC_VID_ACCUH2_0		0x8
#define DISPC_VID_ACCUH2_1		0xc

#define DISPC_VID_ACCUV_0		0x10
#define DISPC_VID_ACCUV_1		0x14
#define DISPC_VID_ACCUV2_0		0x18
#define DISPC_VID_ACCUV2_1		0x1c

#define DISPC_VID_ATTRIBUTES		0x20
#define DISPC_VID_ATTRIBUTES2		0x24

#define DISPC_VID_BA_0			0x28
#define DISPC_VID_BA_1			0x2c
#define DISPC_VID_BA_UV_0		0x30
#define DISPC_VID_BA_UV_1		0x34
#define DISPC_VID_BUF_SIZE_STATUS	0x38
#define DISPC_VID_BUF_THRESHOLD		0x3c

#define DISPC_VID_CONV_COEF(n)		(0x40 + (n) * 4)

#define DISPC_VID_FIRH			0x5c
#define DISPC_VID_FIRH2			0x60
#define DISPC_VID_FIRV			0x64
#define DISPC_VID_FIRV2			0x68

#define DISPC_VID_FIR_COEFS_H0		0x6c
#define DISPC_VID_FIR_COEF_H0(phase)	(0x6c + (phase) * 4)
#define DISPC_VID_FIR_COEFS_H0_C	0x90
#define DISPC_VID_FIR_COEF_H0_C(phase)	(0x90 + (phase) * 4)

#define DISPC_VID_FIR_COEFS_H12		0xb4
#define DISPC_VID_FIR_COEF_H12(phase)	(0xb4 + (phase) * 4)
#define DISPC_VID_FIR_COEFS_H12_C	0xf4
#define DISPC_VID_FIR_COEF_H12_C(phase)	(0xf4 + (phase) * 4)

#define DISPC_VID_FIR_COEFS_V0		0x134
#define DISPC_VID_FIR_COEF_V0(phase)	(0x134 + (phase) * 4)
#define DISPC_VID_FIR_COEFS_V0_C	0x158
#define DISPC_VID_FIR_COEF_V0_C(phase)	(0x158 + (phase) * 4)

#define DISPC_VID_FIR_COEFS_V12		0x17c
#define DISPC_VID_FIR_COEF_V12(phase)	(0x17c + (phase) * 4)
#define DISPC_VID_FIR_COEFS_V12_C	0x1bc
#define DISPC_VID_FIR_COEF_V12_C(phase)	(0x1bc + (phase) * 4)

#define DISPC_VID_IRQENABLE		0x200
#define DISPC_VID_IRQSTATUS		0x204

#define DISPC_VID_MFLAG_THRESHOLD	0x208
#define DISPC_VID_PICTURE_SIZE		0x20c
#define DISPC_VID_PIXEL_INC		0x210
#define DISPC_VID_POSITION		0x214
#define DISPC_VID_PRELOAD		0x218
#define DISPC_VID_ROW_INC		0x21c
#define DISPC_VID_SIZE			0x220

/* OVR */

#define DISPC_OVR_DEFAULT_COLOR		0x08
#define DISPC_OVR_DEFAULT_COLOR2	0x0c

/* VP */

#define DISPC_VP_CONFIG			0x00
#define DISPC_VP_CONTROL		0x04
#define DISPC_VP_GAMMA_TABLE		0x20
#define DISPC_VP_IRQENABLE		0x3c
#define DISPC_VP_IRQSTATUS		0x40
#define DISPC_VP_POL_FREQ		0x4c
#define DISPC_VP_SIZE_SCREEN		0x50
#define DISPC_VP_TIMING_H		0x54
#define DISPC_VP_TIMING_V		0x58

#endif
