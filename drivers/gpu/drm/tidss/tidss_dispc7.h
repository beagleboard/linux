/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2016-2018 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Jyri Sarha <jsarha@ti.com>
 */

#ifndef __TIDSS_DISPC7_H
#define __TIDSS_DISPC7_H

#define DISPC7_MAX_PORTS	4
#define DISPC7_MAX_PLANES	4

struct dispc7_features_scaling {
	u32 in_width_max_5tap_rgb;
	u32 in_width_max_3tap_rgb;
	u32 in_width_max_5tap_yuv;
	u32 in_width_max_3tap_yuv;
	u32 upscale_limit;
	u32 downscale_limit_5tap;
	u32 downscale_limit_3tap;
	u32 xinc_max;
};

enum dispc7_vp_bus_type {
	DISPC7_VP_DPI,
	DISPC7_VP_OLDI,
};

struct dispc7_features {
	/* XXX should these come from the .dts? Min pclk is not feature of DSS IP */
	unsigned long min_pclk;
	unsigned long max_pclk;

	struct dispc7_features_scaling scaling;

	u32 num_vps;
	const char *vp_name[DISPC7_MAX_PORTS]; /* Should match dt reg names */
	const char *ovr_name[DISPC7_MAX_PORTS]; /* Should match dt reg names */
	const char *vpclk_name[DISPC7_MAX_PORTS]; /* Should match dt clk names */
	const enum dispc7_vp_bus_type vp_bus_type[DISPC7_MAX_PORTS];
	u32 num_planes;
	const char *vid_name[DISPC7_MAX_PLANES]; /* Should match dt reg names */
	bool vid_lite[DISPC7_MAX_PLANES];
	u32 vid_order[DISPC7_MAX_PLANES];
};

#define DSS_REVISION			0x4
#define DSS_SYSCONFIG			0x8
#define DSS_SYSSTATUS			0x20
#define DISPC_IRQ_EOI			0x24
#define DISPC_IRQSTATUS_RAW		0x28
#define DISPC_IRQSTATUS			0x2c
#define DISPC_IRQENABLE_SET		0x30
#define DISPC_IRQENABLE_CLR		0x40
#define DISPC_VID_IRQENABLE(n)		(0x44 + (n) * 4)
#define DISPC_VID_IRQSTATUS(n)		(0x58 + (n) * 4)
#define DISPC_VP_IRQENABLE(n)		(0x70 + (n) * 4)
#define DISPC_VP_IRQSTATUS(n)		(0x7c + (n) * 4)

#define DISPC_GLOBAL_MFLAG_ATTRIBUTE	0x90
#define DISPC_GLOBAL_OUTPUT_ENABLE	0x94
#define DISPC_GLOBAL_BUFFER		0x98
#define DSS_CBA_CFG			0x9c
#define DISPC_DBG_CONTROL		0xa0
#define DISPC_DBG_STATUS		0xa4
#define DISPC_CLKGATING_DISABLE		0xa8
#define DISPC_SECURE_DISABLE		0xac

/* COMMON1 not implemented */

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
#define DISPC_VID_CSC_COEF(n)		(0x40 + (n) * 4)

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

#define DISPC_VID_GLOBAL_ALPHA		0x1fc
#define DISPC_VID_MFLAG_THRESHOLD	0x208
#define DISPC_VID_PICTURE_SIZE		0x20c
#define DISPC_VID_PIXEL_INC		0x210
#define DISPC_VID_PRELOAD		0x218
#define DISPC_VID_ROW_INC		0x21c
#define DISPC_VID_SIZE			0x220
#define DISPC_VID_BA_EXT_0		0x22c
#define DISPC_VID_BA_EXT_1		0x230
#define DISPC_VID_BA_UV_EXT_0		0x234
#define DISPC_VID_BA_UV_EXT_1		0x238
#define DISPC_VID_CSC_COEF7		0x23c
#define DISPC_VID_ROW_INC_UV		0x248
#define DISPC_VID_CLUT			0x260
#define DISPC_VID_SAFETY_ATTRIBUTES	0x2a0
#define DISPC_VID_SAFETY_CAPT_SIGNATURE	0x2a4
#define DISPC_VID_SAFETY_POSITION	0x2a8
#define DISPC_VID_SAFETY_REF_SIGNATURE	0x2ac
#define DISPC_VID_SAFETY_SIZE		0x2b0
#define DISPC_VID_SAFETY_LFSR_SEED	0x2b4
#define DISPC_VID_LUMAKEY		0x2b8

/* OVR */

#define DISPC_OVR_CONFIG		0x0
#define DISPC_OVR_DEFAULT_COLOR		0x8
#define DISPC_OVR_DEFAULT_COLOR2	0xc
#define DISPC_OVR_TRANS_COLOR_MAX	0x10
#define DISPC_OVR_TRANS_COLOR_MAX2	0x14
#define DISPC_OVR_TRANS_COLOR_MIN	0x18
#define DISPC_OVR_TRANS_COLOR_MIN2	0x1c
#define DISPC_OVR_ATTRIBUTES(n)		(0x20 + (n) * 4)

/* VP */

#define DISPC_VP_CONFIG				0x0
#define DISPC_VP_CONTROL			0x4
#define DISPC_VP_CSC_COEF0			0x8
#define DISPC_VP_CSC_COEF1			0xc
#define DISPC_VP_CSC_COEF2			0x10
#define DISPC_VP_DATA_CYCLE_0			0x14
#define DISPC_VP_DATA_CYCLE_1			0x18
#define DISPC_VP_DATA_CYCLE_2			0x1c
#define DISPC_VP_LINE_NUMBER			0x44
#define DISPC_VP_POL_FREQ			0x4c
#define DISPC_VP_SIZE_SCREEN			0x50
#define DISPC_VP_TIMING_H			0x54
#define DISPC_VP_TIMING_V			0x58
#define DISPC_VP_CSC_COEF3			0x5c
#define DISPC_VP_CSC_COEF4			0x60
#define DISPC_VP_CSC_COEF5			0x64
#define DISPC_VP_CSC_COEF6			0x68
#define DISPC_VP_CSC_COEF7			0x6c
#define DISPC_VP_SAFETY_ATTRIBUTES_0		0x70
#define DISPC_VP_SAFETY_ATTRIBUTES_1		0x74
#define DISPC_VP_SAFETY_ATTRIBUTES_2		0x78
#define DISPC_VP_SAFETY_ATTRIBUTES_3		0x7c
#define DISPC_VP_SAFETY_CAPT_SIGNATURE_0	0x90
#define DISPC_VP_SAFETY_CAPT_SIGNATURE_1	0x94
#define DISPC_VP_SAFETY_CAPT_SIGNATURE_2	0x98
#define DISPC_VP_SAFETY_CAPT_SIGNATURE_3	0x9c
#define DISPC_VP_SAFETY_POSITION_0		0xb0
#define DISPC_VP_SAFETY_POSITION_1		0xb4
#define DISPC_VP_SAFETY_POSITION_2		0xb8
#define DISPC_VP_SAFETY_POSITION_3		0xbc
#define DISPC_VP_SAFETY_REF_SIGNATURE_0		0xd0
#define DISPC_VP_SAFETY_REF_SIGNATURE_1		0xd4
#define DISPC_VP_SAFETY_REF_SIGNATURE_2		0xd8
#define DISPC_VP_SAFETY_REF_SIGNATURE_3		0xdc
#define DISPC_VP_SAFETY_SIZE_0			0xf0
#define DISPC_VP_SAFETY_SIZE_1			0xf4
#define DISPC_VP_SAFETY_SIZE_2			0xf8
#define DISPC_VP_SAFETY_SIZE_3			0xfc
#define DISPC_VP_SAFETY_LFSR_SEED		0x110
#define DISPC_VP_GAMMA_TABLE			0x120
#define DISPC_VP_DSS_OLDI_CFG			0x160
#define DISPC_VP_DSS_OLDI_STATUS		0x164
#define DISPC_VP_DSS_OLDI_LB			0x168

/* CTRL_MMR0 access trough syscon */
#define CTRLMMR0P1_OLDI_DAT0_IO_CTRL		0x41E0
#define CTRLMMR0P1_OLDI_DAT1_IO_CTRL		0x41E4
#define CTRLMMR0P1_OLDI_DAT2_IO_CTRL		0x41E8
#define CTRLMMR0P1_OLDI_DAT3_IO_CTRL		0x41EC
#define CTRLMMR0P1_OLDI_CLK_IO_CTRL		0x41F0

#define CTRLMMR0P1_OLDI_PWRDN_TX		BIT(8)

#endif /* __TIDSS_DISPC7_H */
