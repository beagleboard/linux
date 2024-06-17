/* SPDX-License-Identifier: GPL-2.0 */
/*
 * topaz utility header
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Sunita Nadampalli <sunitan@ti.com>
 *
 * Re-written for upstreming
 *	Sidraya Jayagond <sidraya.bj@pathpartnertech.com>
 */

#include <linux/types.h>
#include "topaz_api.h"

/*
 * Calculates the correct number of macroblocks per kick and kicks per BU
 */
void calculate_kick_and_bu_size(unsigned int width_in_mbs,
				unsigned int height_in_mbs,
				unsigned char is_interlaced,
				unsigned int max_bu_per_frame,
				unsigned int *kick_size,
				unsigned int *kicks_per_bu,
				unsigned int *min_slice_height);

unsigned int calculate_stride(enum img_format format,
			      unsigned short requested_stride_bytes,
			      unsigned short width);

void topaz_setup_input_format(struct img_video_context *video,
			      struct img_vxe_scaler_setup *scaler_setup);

void topaz_setup_input_csc(struct img_video_context *video,
			   struct img_vxe_scaler_setup *scaler_setup,
			   struct img_vxe_csc_setup *csc_setup,
			   enum img_csc_preset csc_preset);

unsigned int topaz_get_packed_buffer_strides(unsigned short buffer_stride_bytes,
					     enum img_format format,
					     unsigned char enable_scaler,
					     unsigned char is_interlaced,
					     unsigned char is_interleaved);

void prepare_mv_estimates(struct img_enc_context *enc);

void adjust_pic_flags(struct img_enc_context *enc, struct img_rc_params *prc_params,
		      unsigned char first_pic, unsigned int *flags);

void setup_rc_data(struct img_video_context *video, struct pic_params *pic_params,
		   struct img_rc_params *rc_params);

void patch_hw_profile(struct img_video_params *video_params, struct img_video_context *video);
