/* SPDX-License-Identifier: GPL-2.0 */
/*
 * VDECDD translation API's.
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Amit Makani <amit.makani@ti.com>
 *
 * Re-written for upstreamimg
 *	Sidraya Jayagond <sidraya.bj@pathpartnertech.com>
 *	Prashanth Kumar Amai <prashanth.ka@pathpartnertech.com>
 */
#ifndef __TRANSLATION_API_H__
#define __TRANSLATION_API_H__

#include "decoder.h"
#include "hw_control.h"
#include "vdecdd_defs.h"
#include "vdec_defs.h"
#include "vxd_props.h"

/*
 * This function submits a stream unit for translation
 * into a control allocation buffer used in PVDEC operation.
 */
int translation_ctrl_alloc_prepare
	(struct vdec_str_configdata *psstr_config_data,
	struct vdecdd_str_unit *psstrunit,
	struct dec_decpict *psdecpict,
	const struct vxd_coreprops *core_props,
	struct decoder_regsoffsets *regs_offset);

/*
 * TRANSLATION_FragmentPrepare.
 */
int translation_fragment_prepare(struct dec_decpict *psdecpict,
				 struct lst_t *decpic_seg_list, int eop,
				 struct dec_pict_fragment *pict_fragement);

#endif /* __TRANSLATION_API_H__ */
