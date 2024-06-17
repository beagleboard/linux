// SPDX-License-Identifier: GPL-2.0
/*
 * VXD Decoder device driver utility functions implementation
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Sunita Nadampalli <sunitan@ti.com>
 *
 * Re-written for upstream
 *	Sidraya Jayagond <sidraya.bj@pathpartnertech.com>
 *	Prashanth Kumar Amai <prashanth.ka@pathpartnertech.com>
 */

#include <linux/dma-mapping.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>

#include "bspp.h"
#include "vdecdd_utils.h"

/*
 * @Function              VDECDDUTILS_FreeStrUnit
 */
int vdecddutils_free_strunit(struct vdecdd_str_unit *str_unit)
{
	struct bspp_bitstr_seg *bstr_seg;

	/* Loop over bit stream segments */
	bstr_seg = (struct bspp_bitstr_seg *)lst_removehead(&str_unit->bstr_seg_list);
	while (bstr_seg) {
		/* Free segment. */
		kfree(bstr_seg);

		/* Get next segment. */
		bstr_seg = (struct bspp_bitstr_seg *)lst_removehead(&str_unit->bstr_seg_list);
	}

	/* Free the sequence header */
	if (str_unit->seq_hdr_info) {
		str_unit->seq_hdr_info->ref_count--;
		if (str_unit->seq_hdr_info->ref_count == 0) {
			kfree(str_unit->seq_hdr_info);
			str_unit->seq_hdr_info = NULL;
		}
	}

	/* Free the picture header... */
	if (str_unit->pict_hdr_info) {
		kfree(str_unit->pict_hdr_info->pict_sgm_data.pic_data);
		str_unit->pict_hdr_info->pict_sgm_data.pic_data = NULL;

		kfree(str_unit->pict_hdr_info);
		str_unit->pict_hdr_info = NULL;
	}

	/* Free stream unit. */
	kfree(str_unit);
	str_unit = NULL;

	/* Return success */
	return IMG_SUCCESS;
}

/*
 * @Function: VDECDDUTILS_CreateStrUnit
 * @Description: this function allocate a structure for a complete data unit
 */
int vdecddutils_create_strunit(struct vdecdd_str_unit **str_unit_handle,
			       struct lst_t *bs_list)
{
	struct vdecdd_str_unit *str_unit;
	struct bspp_bitstr_seg *bstr_seg;

	str_unit = kzalloc(sizeof(*str_unit), GFP_KERNEL);
	VDEC_ASSERT(str_unit);
	if (!str_unit)
		return IMG_ERROR_OUT_OF_MEMORY;

	if (bs_list) {
		/* copy BS list to this list */
		lst_init(&str_unit->bstr_seg_list);
		for (bstr_seg = lst_first(bs_list); bstr_seg;
			bstr_seg = lst_first(bs_list)) {
			bstr_seg = lst_removehead(bs_list);
			lst_add(&str_unit->bstr_seg_list, bstr_seg);
		}
	}

	*str_unit_handle = str_unit;

	return IMG_SUCCESS;
}
