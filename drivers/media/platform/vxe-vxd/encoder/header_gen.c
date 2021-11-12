// SPDX-License-Identifier: GPL-2.0
/*
 * Encoder coded header generation function implementations
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
#include "fw_headers/topazscfwif.h"
#include "fw_headers/defs.h"
#include "header_gen.h"
#include "img_errors.h"
#include "reg_headers/topazhp_core_regs.h"
#include "topaz_api.h"

#define ELEMENTS_EMPTY 9999
#define MAXNUMBERELEMENTS 32
#define _1080P_30FPS (((1920 * 1088) / 256) * 30)

void insert_element_token(struct mtx_header_params *mtx_header,
			  struct mtx_header_element **element_pointers,
			  enum header_element_type token)
{
	unsigned char offset;
	unsigned char *p;

	if (mtx_header->elements != ELEMENTS_EMPTY) {
		if (element_pointers[mtx_header->elements]->element_type ==
			ELEMENT_STARTCODE_RAWDATA ||
			element_pointers[mtx_header->elements]->element_type == ELEMENT_RAWDATA ||
			element_pointers[mtx_header->elements]->element_type ==
			ELEMENT_STARTCODE_MIDHDR) {
			/*
			 * Add a new element aligned to word boundary
			 * Find RAWBit size in bytes (rounded to word boundary))
			 * NumberofRawbits (excluding size of bit count field)+
			 * size of the bitcount field
			 */
			offset = element_pointers[mtx_header->elements]->size + 8 + 31;
			offset /= 32; /*Now contains rawbits size in words */
			offset += 1; /*Now contains rawbits+element_type size in words */
			/* Convert to number of bytes (total size of structure
			 * in bytes, aligned to word boundary).
			 */
			offset *= 4;
		} else {
			offset = 4;
		}

		mtx_header->elements++;
		p = (unsigned char *)element_pointers[mtx_header->elements - 1];
		p += offset;
		element_pointers[mtx_header->elements] = (struct mtx_header_element *)p;
	} else {
		mtx_header->elements = 0;
	}

	element_pointers[mtx_header->elements]->element_type = token;
	element_pointers[mtx_header->elements]->size = 0;
}

unsigned int write_upto_8bits_to_elements(struct mtx_header_params *mtx_header,
					  struct mtx_header_element **element_pointers,
					  unsigned int write_bits, unsigned short bit_cnt)
{
	/* This is the core function to write bits/bytes to a header stream,
	 * it writes them directly to ELEMENT structures.
	 */
	unsigned char *write_bytes;
	unsigned char *size_bits;
	union input_value {
		unsigned int input16;
		unsigned char input8[2];
	} input_val;

	unsigned char out_byte_index;
	short shift;

	if (bit_cnt == 0)
		return 0;

	/* First ensure that unused bits in ui32WriteBits are zeroed */
	write_bits &= (0x00ff >> (8 - bit_cnt));
	input_val.input16 = 0;
	/*Pointer to the bit count field */
	size_bits = &element_pointers[mtx_header->elements]->size;
	/*Pointer to the space where header bits are to be written */
	write_bytes = &element_pointers[mtx_header->elements]->bits;
	out_byte_index = (size_bits[0] / 8);

	if (!(size_bits[0] & 7)) {
		if (size_bits[0] >= 120) {
			/*Element maximum bits send to element, time to start a new one */
			mtx_header->elements++; /* Increment element index */
			/*Element pointer set to position of next element (120/8 = 15 bytes) */
			element_pointers[mtx_header->elements] =
				(struct mtx_header_element *)&write_bytes[15];
			/*Write ELEMENT_TYPE */
			element_pointers[mtx_header->elements]->element_type = ELEMENT_RAWDATA;
			/* Set new element size (bits) to zero */
			element_pointers[mtx_header->elements]->size = 0;
			/* Begin writing to the new element */
			write_upto_8bits_to_elements(mtx_header, element_pointers, write_bits,
						     bit_cnt);
			return (unsigned int)bit_cnt;
		}
		write_bytes[out_byte_index] = 0; /* Beginning a new byte, clear byte */
	}

	shift = (short)((8 - bit_cnt) - (size_bits[0] & 7));

	if (shift >= 0) {
		write_bits <<= shift;
		write_bytes[out_byte_index] |= write_bits;
		size_bits[0] = size_bits[0] + bit_cnt;
	} else {
		input_val.input8[1] = (unsigned char)write_bits + 256;
		input_val.input16 >>= -shift;
		write_bytes[out_byte_index] |= input_val.input8[1];

		size_bits[0] = size_bits[0] + bit_cnt;
		size_bits[0] = size_bits[0] - ((unsigned char)-shift);
		input_val.input8[0] = input_val.input8[0] >> (8 + shift);
		write_upto_8bits_to_elements(mtx_header, element_pointers, input_val.input8[0],
					     (unsigned short)-shift);
	}

	return (unsigned int)bit_cnt;
}

unsigned int write_upto_32bits_to_elements(struct mtx_header_params *mtx_header,
					   struct mtx_header_element **element_pointers,
					   unsigned int write_bits, unsigned int bit_cnt)
{
	unsigned int bit_lp;
	unsigned int end_byte;
	unsigned char bytes[4];

	for (bit_lp = 0; bit_lp < 4; bit_lp++) {
		bytes[bit_lp] = (unsigned char)(write_bits & 255);
		write_bits = write_bits >> 8;
	}

	end_byte = ((bit_cnt + 7) / 8);
	if ((bit_cnt) % 8)
		write_upto_8bits_to_elements(mtx_header, element_pointers, bytes[end_byte - 1],
					     (unsigned char)((bit_cnt) % 8));
	else
		write_upto_8bits_to_elements(mtx_header, element_pointers, bytes[end_byte - 1], 8);

	if (end_byte > 1)
		for (bit_lp = end_byte - 1; bit_lp > 0; bit_lp--)
			write_upto_8bits_to_elements(mtx_header, element_pointers,
						     bytes[bit_lp - 1], 8);

	return bit_cnt;
}

void h264_write_bits_startcode_prefix_element(struct mtx_header_params *mtx_header,
					      struct mtx_header_element **element_pointers,
	unsigned int byte_size)
{
	/* GENERATES THE FIRST ELEMENT OF THE H264_SEQUENCE_HEADER() STRUCTURE */
	unsigned int lp;
	/*
	 * Byte aligned (bit 0)
	 * (3 bytes in slice header when slice is first in
	 * a picture without sequence/picture_header before picture
	 */
	for (lp = 0; lp < byte_size - 1; lp++)
		write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 8);

	write_upto_8bits_to_elements(mtx_header, element_pointers, 1, 8);
	/* Byte aligned (bit 32 or 24) */
}

unsigned int generate_ue(struct mtx_header_params *mtx_header,
			 struct mtx_header_element **element_pointers, unsigned int val)
{
	unsigned int lp;
	unsigned char zeros;
	unsigned int chunk;
	unsigned int bit_cnter = 0;

	for (lp = 1, zeros = 0; (lp - 1) < val; lp = lp + lp, zeros++)
		val = val - lp;

	/*
	 * zeros = number of preceding zeros required
	 * Val = value to append after zeros and 1 bit
	 * Write preceding zeros
	 */
	for (lp = (unsigned int)zeros; lp + 1 > 8; lp -= 8)
		bit_cnter += write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 8);

	/* Write zeros and 1 bit set */
	bit_cnter +=
		write_upto_8bits_to_elements(mtx_header, element_pointers, (unsigned char)1,
					     (unsigned char)(lp + 1));

	/* Write Numeric part  */
	while (zeros > 8) {
		zeros -= 8;
		chunk = (val >> zeros);
		bit_cnter += write_upto_8bits_to_elements(mtx_header, element_pointers,
				(unsigned char)chunk, 8);
		val = val - (chunk << zeros);
	}

	bit_cnter += write_upto_8bits_to_elements(mtx_header,
				element_pointers, (unsigned char)val, zeros);

	return bit_cnter;
}

unsigned int generate_se(struct mtx_header_params *mtx_header,
			 struct mtx_header_element **element_pointers, int val)
{
	unsigned int bit_cnter;
	unsigned int code_num;

	bit_cnter = 0;

	if (val > 0)
		code_num = (unsigned int)(val + val - 1);
	else
		code_num = (unsigned int)(-val - val);

	bit_cnter = generate_ue(mtx_header, element_pointers, code_num);

	return bit_cnter;
}

void h264_write_bits_scaling_lists(struct mtx_header_params *mtx_header,
				   struct mtx_header_element **element_pointers,
				   struct h264_scaling_matrix_params *scaling_matrix,
				   unsigned char write_8x8)
{
	/* Used by H264_WriteBits_SequenceHeader and H264_WriteBits_PictureHeader */
	unsigned int list, index;
	int cur_scale, delta_scale;

	if (!scaling_matrix) {
		insert_element_token(mtx_header, element_pointers, ELEMENT_CUSTOM_QUANT);
		return;
	}

	for (list = 0; list < 6; list++) {
		/* seq_scaling_list_present_flag[ui32List] = 1 */
		if (scaling_matrix->list_mask & (1 << list)) {
			write_upto_8bits_to_elements(mtx_header, element_pointers, 1, 1);

			cur_scale = 8;
			for (index = 0; index < 16; index++) {
				delta_scale =
					((int)scaling_matrix->scaling_lists4x4[list][index]) -
					cur_scale;
				cur_scale += delta_scale;
				/* delta_scale */
				generate_se(mtx_header, element_pointers, delta_scale);
			}
		} else {
			/* seq_scaling_list_present_flag[ui32List] = 0 */
			write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 1);
		}
	}

	if (!write_8x8)
		return;

	for (; list < 8; list++) {
		/* seq_scaling_list_present_flag[ui32List] = 1 */
		if (scaling_matrix->list_mask & (1 << list)) {
			write_upto_8bits_to_elements(mtx_header, element_pointers, 1, 1);

			cur_scale = 8;
			for (index = 0; index < 64; index++) {
				delta_scale =
					((int)scaling_matrix->scaling_lists8x8[list - 6][index]) -
					cur_scale;
				cur_scale += delta_scale;
				/* delta_scale */
				generate_se(mtx_header, element_pointers, delta_scale);
			}
		} else {
			/* seq_scaling_list_present_flag[ui32List] = 0 */
			write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 1);
		}
	}
}

void h264_write_bits_vui_params(struct mtx_header_params *mtx_header,
				struct mtx_header_element **element_pointers,
				struct h264_vui_params *vui_params)
{
	/* Builds VUI Params for the Sequence Header (only present in the 1st sequence of stream) */

	if (vui_params->aspect_ratio_info_present_flag == 1) {
		/* aspect_ratio_info_present_flag = 1 in Topaz */
		write_upto_8bits_to_elements(mtx_header, element_pointers,
					     vui_params->aspect_ratio_info_present_flag, 1);
		/* aspect_ratio_idc (8 bits) = vui_params->aspect_ratio_idc in Topaz */
		write_upto_8bits_to_elements(mtx_header, element_pointers,
					     vui_params->aspect_ratio_idc, 8);

		if (vui_params->aspect_ratio_idc == 255) {
			write_upto_8bits_to_elements(mtx_header, element_pointers,
						     (vui_params->sar_width >> 8), 8);
			write_upto_8bits_to_elements(mtx_header, element_pointers,
						     vui_params->sar_width, 8);
			write_upto_8bits_to_elements(mtx_header, element_pointers,
						     (vui_params->sar_height >> 8), 8);
			write_upto_8bits_to_elements(mtx_header, element_pointers,
						     vui_params->sar_height, 8);
		}
	} else {
		/* aspect_ratio_info_present_flag = 0 in Topaz */
		write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 1);
	}

	write_upto_8bits_to_elements(mtx_header, element_pointers,
				     /* overscan_info_present_flag (1 bit) = 0 in Topaz */
				     (0 << 3) |
				     /* video_signal_type_present_flag (1 bit) = 0 in Topaz */
				     (0 << 2) |
				     /* chroma_loc_info_present_flag (1 bit) = 0 in Topaz */
				     (0 << 1) |
				     /* timing_info_present_flag (1 bit) = 1 in Topaz */
				     (1),
				     /* num_units_in_tick (32 bits) = 1 in Topaz */
				     4);
	write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 8);
	write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 8);
	write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 8);
	write_upto_8bits_to_elements(mtx_header, element_pointers, 1, 8);

	/* time_scale (32 bits) = frame rate */
	write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 8);
	write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 8);
	write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 8);
	write_upto_8bits_to_elements(mtx_header, element_pointers,
				     (unsigned char)vui_params->time_scale, 8);
	/* fixed_frame_rate_flag (1 bit) = 1 in Topaz */
	write_upto_8bits_to_elements(mtx_header, element_pointers, 1, 1);
	/* nal_hrd_parameters_present_flag (1 bit) = 1 in Topaz */
	write_upto_8bits_to_elements(mtx_header, element_pointers, 1, 1);
	/* Definitions for nal_hrd_parameters() contained in VUI structure for Topaz
	 * cpb_cnt_minus1 ue(v) = 0 in Topaz = 1b
	 */
	write_upto_8bits_to_elements(mtx_header, element_pointers, 1, 1);
	/* bit_rate_scale (4 bits) = 0 in Topaz */
	write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 4);
	/* cpb_size_scale (4 bits) = 2 in Topaz */
	write_upto_8bits_to_elements(mtx_header, element_pointers, 2, 4);
	/* bit_rate_value_minus1[0] ue(v) = (Bitrate/64)-1 [RANGE:0 to (2^32)-2] */
	generate_ue(mtx_header, element_pointers, vui_params->bit_rate_value_minus1);
	/* cpb_size_value_minus1[0] ue(v) = (CPB_Bits_Size/16)-
	 * 1 where CPB_Bits_Size = 1.5 * Bitrate  [RANGE:0 to (2^32)-2]
	 */
	generate_ue(mtx_header, element_pointers, vui_params->cbp_size_value_minus1);
	/* cbr_flag[0] (1 bit) = 0 for VBR, 1 for CBR */
	write_upto_8bits_to_elements(mtx_header, element_pointers, vui_params->cbr, 1);
	/*initial_cpb_removal_delay_length_minus1 (5 bits) */
	write_upto_8bits_to_elements(mtx_header, element_pointers,
				     vui_params->initial_cpb_removal_delay_length_minus1, 5);
	/* cpb_removal_delay_length_minus1 (5 bits) */
	write_upto_8bits_to_elements(mtx_header, element_pointers,
				     vui_params->cpb_removal_delay_length_minus1, 5);
	/* dpb_output_delay_length_minus1 (5 bits) */
	write_upto_8bits_to_elements(mtx_header, element_pointers,
				     vui_params->dpb_output_delay_length_minus1, 5);
	/* time_offst_length (5 bits) */
	write_upto_8bits_to_elements(mtx_header, element_pointers, vui_params->time_offset_length,
				     5);

	/* End of nal_hrd_parameters() */
	write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 1);
	/* low_delay_hrd_flag */
	write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 1);
	/* pic_struct_present_flag (1 bit) = 0 in Topaz */
	write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 1);

	write_upto_8bits_to_elements(mtx_header, element_pointers,
				     /* bitstream_restriction_flag (1 bit) = 1 in Topaz */
				     (1 << 1) |
				     /* motion_vectors_over_pic_boundaries_flag (1 bit) = 1
				      * in Topaz;
				      */
				     (1 << 0),
				     2);
	/* max_bytes_per_pic_denom ue(v) = 0 */
	generate_ue(mtx_header, element_pointers, 0);
	/* max_bits_per_mb_denom ue(v) = 0 */
	generate_ue(mtx_header, element_pointers, 0);
	/* log2_max_mv_length_horizontal ue(v) = 9(max horizontal vector is 128 integer samples) */
	generate_ue(mtx_header, element_pointers, 9);
	/* log2_max_mv_length_vertical ue(v) = 9 (max vertical vecotr is 103 integer samples) */
	generate_ue(mtx_header, element_pointers, 9);
	/* num_reorder_frames ue(v) = 0 */
	generate_ue(mtx_header, element_pointers, vui_params->num_reorder_frames);
	/* max_dec_frame_buffering ue(v) = 0 */
	generate_ue(mtx_header, element_pointers, vui_params->max_dec_frame_buffering);
}

void h264_write_bits_sequence_header(struct mtx_header_params *mtx_header,
				     struct mtx_header_element **element_pointers,
				     struct h264_sequence_header_params *sh_params,
				     struct h264_crop_params *crop,
				     struct h264_scaling_matrix_params *scaling_matrix,
				     unsigned char aso)
{
	/* calculate some of the VUI parameters here */
	if (sh_params->profile == SH_PROFILE_BP) {
		/* for Baseline profile we never re-roder frames */
		sh_params->vui_params.num_reorder_frames = 0;
		sh_params->vui_params.max_dec_frame_buffering = sh_params->max_num_ref_frames;
	} else {
		/* in higher profiles we can do up to 3 level hierarchical B frames */
		if (!sh_params->vui_params.num_reorder_frames)
			sh_params->vui_params.num_reorder_frames = sh_params->max_num_ref_frames;
		sh_params->vui_params.max_dec_frame_buffering =
						max(sh_params->max_num_ref_frames,
						    sh_params->vui_params.num_reorder_frames);
	}

	insert_element_token(mtx_header, element_pointers, ELEMENT_STARTCODE_RAWDATA);
	h264_write_bits_startcode_prefix_element(mtx_header, element_pointers, 4);

	/* GENERATES THE FIRST ELEMENT OF THE H264_SEQUENCE_HEADER() STRUCTURE */
	/*
	 * 4 Byte StartCodePrefix Pregenerated in: H264_WriteBits_StartCodePrefix_Element()
	 * Byte aligned (bit 32)
	 * forbidden_zero_bit=0
	 */
	write_upto_8bits_to_elements(mtx_header, element_pointers, (0 << 7) |
		(0x3 << 5) |		/* nal_ref_idc=01 (may be 11) */
		(7),			/* nal_unit_type=00111 */
		8);

	/* Byte aligned (bit 40) */
	switch (sh_params->profile) {
	case SH_PROFILE_BP:
		/* profile_idc = 8 bits = 66 for BP (PROFILE_IDC_BP) */
		write_upto_8bits_to_elements(mtx_header, element_pointers, 66, 8);

		/* Byte	aligned	(bit 48) */
		write_upto_8bits_to_elements(mtx_header, element_pointers,
					     /* constraint_set0_flag = 1 for BP constra ints */
					     (1 << 7) |
					     /* constraint_set1_flag = 1 for MP constraints */
					     ((aso ? 0 : 1) << 6) |
					     /* constraint_set2_flag = 1 for EP constra ints */
					     (1 << 5) |
					     /* constraint_set3_flag = 1
					      * for level 1b, 0 for others
					      */
					     ((sh_params->level == SH_LEVEL_1B ? 1 : 0) << 4),
					     /* reserved_zero_4bits = 0 */
					     8);
		break;

	case SH_PROFILE_MP:
		/* profile_idc = 8 bits = 77 for MP (PROFILE_IDC_MP) */
		write_upto_8bits_to_elements(mtx_header, element_pointers, 77, 8);

		/* Byte	aligned	(bit 48) */
		write_upto_8bits_to_elements(mtx_header, element_pointers,
					     /* constraint_set0_flag = 0 for no BP constraints */
					     (0 << 7) |
					     /* constraint_set1_flag = 1 for MP constraints */
					     (1 << 6) |
					     /* constraint_set2_flag = 1 for EP constraints */
					     (1 << 5) |
					     /* constraint_set3_flag = 1
					      * for level 1b, 0 for others
					      */
					     ((sh_params->level == SH_LEVEL_1B ? 1 : 0) << 4),
					     /* reserved_zero_4bits = 0 */
					     8);
		break;

	case SH_PROFILE_HP:
		/* profile_idc = 8 bits = 100 for HP (PROFILE_IDC_HP) */
		write_upto_8bits_to_elements(mtx_header, element_pointers, 100, 8);

		/* Byte aligned (bit 48) */
		write_upto_8bits_to_elements(mtx_header, element_pointers,
					     /* constraint_set0_flag = 0 for no BP constraints */
					     (0 << 7) |
					     /* constraint_set1_flag = 0 for no MP constraints */
					     (0 << 6) |
					     /* constraint_set2_flag = 0 for no EP constraints */
					     (0 << 5) |
					     /* constraint_set3_flag = 0 */
					     (0 << 4),
					     /* reserved_zero_4bits = 0 */
					     8);
		break;

	case SH_PROFILE_H444P:
		/* profile_idc = 8 bits = 244 for H444P (PROFILE_IDC_H444P) */
		write_upto_8bits_to_elements(mtx_header, element_pointers, 244, 8);

		/* Byte aligned (bit 48) */
		write_upto_8bits_to_elements(mtx_header, element_pointers,
					     /* constraint_set0_flag = 0 for no BP constraints */
					     (0 << 7) |
					     /* constraint_set1_flag = 0 for no MP constraints */
					     (0 << 6) |
					     /* constraint_set2_flag = 0 for no EP constraints */
					     (0 << 5) |
					     /* constraint_set3_flag = 0 */
					     (0 << 4),
					     /* reserved_zero_4bits = 0 */
					     8);
		break;

	default:
		break;
	}

	/*
	 * Byte aligned (bit 56)
	 * level_idc should be set to 9 in the sps in case of
	 * level is Level 1B and the profile is High or High 4:4:4 Profile
	 */
	if (sh_params->profile == SH_PROFILE_HP || sh_params->profile == SH_PROFILE_H444P)
		/* level_idc (8 bits) = 9 for 1b, 10xlevel for others */
		write_upto_8bits_to_elements(mtx_header, element_pointers,
					     (sh_params->level == SH_LEVEL_1B) ? 9 :
					     (unsigned char)sh_params->level, 8);

	else
		/* level_idc (8 bits) = 11 for 1b, 10xlevel for others */
		write_upto_8bits_to_elements(mtx_header, element_pointers,
					     (sh_params->level == SH_LEVEL_1B) ? 11 :
					     (unsigned char)sh_params->level, 8);

	generate_ue(mtx_header, element_pointers, 0);           /* seq_parameter_set_id = 0 */

	if (sh_params->profile == SH_PROFILE_HP || sh_params->profile == SH_PROFILE_H444P) {
		generate_ue(mtx_header, element_pointers, 1);           /* chroma_format_idc = 1 */
		/* bit_depth_luma_minus8 = 0 */
		generate_ue(mtx_header, element_pointers, 0);
		/* bit_depth_chroma_minus8 = 0 */
		generate_ue(mtx_header, element_pointers, 0);

		write_upto_8bits_to_elements(mtx_header, element_pointers,
					     /* qpprime_y_zero_transform_bypass_flag = 1
					      * if lossless
					      */
					     sh_params->is_lossless ? 1 : 0, 1);

		if (sh_params->use_default_scaling_list ||
		    sh_params->seq_scaling_matrix_present_flag) {
			/* seq_scaling_matrix_present_flag */
			write_upto_8bits_to_elements(mtx_header, element_pointers, 1, 1);
			if (!sh_params->use_default_scaling_list) {
				h264_write_bits_scaling_lists(mtx_header, element_pointers,
							      scaling_matrix, TRUE);
				insert_element_token(mtx_header, element_pointers, ELEMENT_RAWDATA);
			} else {
				/* seq_scaling_list_present_flag[i] = 0; 0 < i < 8 */
				write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 8);
			}
		} else {
			/* seq_scaling_matrix_present_flag */
			write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 1);
		}
	}

	generate_ue(mtx_header, element_pointers, 1);           /* log2_max_frame_num_minus4 = 1 */
	generate_ue(mtx_header, element_pointers, 0);           /* pic_order_cnt_type = 0 */
	/* log2_max_pic_order_cnt_Isb_minus4 = 2 */
	generate_ue(mtx_header, element_pointers, sh_params->log2_max_pic_order_cnt - 4);
	/*num_ref_frames ue(2), typically 2 */
	generate_ue(mtx_header, element_pointers, sh_params->max_num_ref_frames);

	/* Bytes aligned (bit 72) */
	/* gaps_in_frame_num_value_allowed_Flag      - (1 bit) */
	write_upto_8bits_to_elements(mtx_header, element_pointers,
				     (sh_params->gaps_in_frame_num_value), 1);

	/*
	 * GENERATES THE SECOND, VARIABLE LENGTH, ELEMENT OF THE H264_SEQUENCE_HEADER() STRUCTURE
	 * ELEMENT BITCOUNT: xx
	 */
	/*pic_width_in_mbs_minus1: ue(v) from 10 to 44 (176 to 720 pixel per row) */
	generate_ue(mtx_header, element_pointers, sh_params->width_in_mbs_minus1);
	/* pic_height_in_maps_units_minus1:
	 * ue(v) Value from 8 to 35 (144 to 576 pixels per column)
	 */
	generate_ue(mtx_header, element_pointers, sh_params->height_in_maps_units_minus1);
	/* We don't know the alignment at this point, so will have to use bit writing functions */
	/* frame_mb_only_flag 1=frame encoding, 0=field encoding */
	write_upto_8bits_to_elements(mtx_header, element_pointers, sh_params->frame_mbs_only_flag,
				     1);

	if (!sh_params->frame_mbs_only_flag) /* in the case of interlaced encoding */
	/* mb_adaptive_frame_field_flag = 0 in Topaz(field encoding at the sequence level) */
		write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 1);

	/* direct_8x8_inference_flag=1 in Topaz */
	write_upto_8bits_to_elements(mtx_header, element_pointers, 1, 1);

	if (crop->clip) {
		write_upto_8bits_to_elements(mtx_header, element_pointers, 1, 1);
		generate_ue(mtx_header, element_pointers, crop->left_crop_offset);
		generate_ue(mtx_header, element_pointers, crop->right_crop_offset);
		generate_ue(mtx_header, element_pointers, crop->top_crop_offset);
		generate_ue(mtx_header, element_pointers, crop->bottom_crop_offset);
	} else {
		write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 1);
	}

	/*
	 * GENERATES THE THIRD ELEMENT OF THE H264_SEQUENCE_HEADER() STRUCTURE
	 * ELEMENT BITCOUNT: xx
	 */
	/* vui_parameters_present_flag (VUI only in 1st sequence of stream) */
	write_upto_8bits_to_elements(mtx_header, element_pointers,
				     (sh_params->vui_params_present),
		1);
	if (sh_params->vui_params_present > 0)
		h264_write_bits_vui_params(mtx_header, element_pointers, &sh_params->vui_params);

	/* Finally we need to align to the next byte */
	/* Tell MTX to insert the byte align field (we don't know
	 * final stream size for alignment at this point)
	 */
	insert_element_token(mtx_header, element_pointers, ELEMENT_INSERTBYTEALIGN_H264);
}

/*
 * Prepare an H264 SPS in a form for the MTX to encode into a bitstream.
 */
void h264_prepare_sequence_header(struct mtx_header_params *mtx_header,
				  unsigned int pic_width_in_mbs,
				  unsigned int pic_height_in_mbs,
				  unsigned char vui_params_present,
				  struct h264_vui_params *params,
				  struct h264_crop_params *crop,
				  struct h264_sequence_header_params *sh_params,
				  unsigned char aso)
{
	struct mtx_header_element *this_element;
	struct mtx_header_element *element_pointers[MAXNUMBERELEMENTS];

	/*
	 * Builds a sequence, picture and slice header with from the given inputs
	 * parameters (start of new frame) Essential we initialise our header
	 * structures before building
	 */
	mtx_header->elements = ELEMENTS_EMPTY;
	this_element = (struct mtx_header_element *)mtx_header->element_stream;
	element_pointers[0] = this_element;

	h264_write_bits_sequence_header(mtx_header, element_pointers, sh_params, crop, NULL, aso);
	/*Has been used as an index, so need to add 1 for a valid element count */
	mtx_header->elements++;
}

void h264_write_bits_picture_header(struct mtx_header_params *mtx_header,
				    struct mtx_header_element **element_pointers,
				    struct h264_picture_header_params *ph_params,
				    struct h264_scaling_matrix_params *scaling_matrix)
{
	/* Begin building the picture header element */
	insert_element_token(mtx_header, element_pointers, ELEMENT_STARTCODE_RAWDATA);

	h264_write_bits_startcode_prefix_element(mtx_header, element_pointers, 4);

	/* GENERATES THE FIRST (STATIC) ELEMENT OF THE H264_PICTURE_HEADER() STRUCTURE
	 * ELEMENT BITCOUNT: 18
	 * 4 Byte StartCodePrefix Pregenerated in: H264_WriteBits_StartCodePrefix_Element()
	 * Byte aligned (bit 32)
	 */
	write_upto_8bits_to_elements(mtx_header, element_pointers,
				     (0 << 7) | /* forbidden_zero_bit */
				     (1 << 5) | /* nal_ref_idc (2 bits) = 1 */
				     (8),       /* nal_unit_tpye (5 bits) = 8 */
				     8);

	/* Byte aligned (bit 40) */
	/* pic_parameter_set_id ue(v) */
	generate_ue(mtx_header, element_pointers, ph_params->pic_parameter_set_id);
	/* seq_parameter_set_id ue(v) */
	generate_ue(mtx_header, element_pointers, ph_params->seq_parameter_set_id);

	/* entropy_coding_mode_flag (1 bit) 0 for CAVLC */
	write_upto_8bits_to_elements(mtx_header, element_pointers,
				     (ph_params->entropy_coding_mode_flag << 4) |
				     (0 << 3) |	/* pic_order_present_flag (1 bit) = 0 */
				     (1 << 2) |	/* num_slice_group_minus1 ue(v) = 0 in Topaz */
				     (1 << 1) | /* num_ref_idx_l0_active_minus1 ue(v) = 0 in Topaz*/
				     (1),/* num_ref_idx_l1_active_minus1 ue(v) = 0 in Topaz */
				     5);

	/* WEIGHTED PREDICTION */
	write_upto_8bits_to_elements(mtx_header, element_pointers,
				     /* weighted_pred_flag (1 bit) */
				     (ph_params->weighted_pred_flag << 2) |
				     /* weighted_bipred_flag (2 bits) */
				     (ph_params->weighted_bipred_idc), 3);

	/*MTX fills this value in */
	insert_element_token(mtx_header, element_pointers, ELEMENT_QP);
	insert_element_token(mtx_header, element_pointers, ELEMENT_RAWDATA);

	/*
	 * GENERATES THE SECOND ELEMENT OF THE H264_PICTURE_HEADER() STRUCTURE
	 * ELEMENT BITCOUNT: 5
	 * The following field will be generated as a special case by MTX - so not here
	 * Generate_se(mtx_header, ph_params->pic_init_qp_minus26); pic_int_qp_minus26
	 * se(v) = -26 to 25 in Topaz
	 */
	generate_se(mtx_header, element_pointers, 0); /* pic_int_qs_minus26 se(v) = 0 in Topaz */
	/* chroma_qp_index_offset se(v) = 0 in Topaz */
	generate_se(mtx_header, element_pointers, ph_params->chroma_qp_index_offset);
	/* deblocking_filter_control_present_flag (1 bit) = 1 in Topaz */
	write_upto_8bits_to_elements(mtx_header, element_pointers, (1 << 2) |
			/* constrained_intra_pred_Flag (1 bit) = 0 in Topaz */
			(ph_params->constrained_intra_pred_flag << 1) |
			/* redundant_pic_cnt_present_flag (1 bit) = 0 in Topaz */
			(0),
			3);

	if (ph_params->transform_8x8_mode_flag ||
	    ph_params->second_chroma_qp_index_offset != ph_params->chroma_qp_index_offset ||
		ph_params->pic_scaling_matrix_present_flag) {
		/* 8x8 transform flag */
		write_upto_8bits_to_elements(mtx_header, element_pointers,
					     ph_params->transform_8x8_mode_flag, 1);
		if (ph_params->pic_scaling_matrix_present_flag) {
			/* pic_scaling_matrix_present_flag */
			write_upto_8bits_to_elements(mtx_header, element_pointers, 1, 1);
			if (!ph_params->use_default_scaling_list) {
				h264_write_bits_scaling_lists(mtx_header, element_pointers,
							      scaling_matrix,
							      ph_params->transform_8x8_mode_flag);
				insert_element_token(mtx_header, element_pointers, ELEMENT_RAWDATA);
			} else {
				unsigned char scaling_list_size =
					ph_params->transform_8x8_mode_flag ? 8 : 6;

				/* pic_scaling_list_present_flag[i] = 0;
				 * 0 < i < 6 (+ 2 ( +4 for chroma444) for 8x8)
				 */
				write_upto_8bits_to_elements(mtx_header, element_pointers, 0,
							     scaling_list_size);
			}
		} else {
			/* pic_scaling_matrix_present_flag */
			write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 1);
		}
		/* second_chroma_qp_index_offset se(v) = 0 in Topaz */
		generate_se(mtx_header, element_pointers, ph_params->second_chroma_qp_index_offset);
	}
	/* Tell MTX to insert the byte align field (we don't know final
	 * stream size for alignment at this point)
	 */
	insert_element_token(mtx_header, element_pointers, ELEMENT_INSERTBYTEALIGN_H264);
}

/*
 * Prepare an H264 PPS in a form for the MTX to encode into a bitstream
 */
void h264_prepare_picture_header(struct mtx_header_params *mtx_header,
				 unsigned char cabac_enabled,
				 unsigned char transform_8x8,
				 unsigned char intra_constrained,
				 signed char cqp_offset,
				 unsigned char weighted_prediction,
				 unsigned char weighted_bi_pred,
				 unsigned char mvc_pps,
				 unsigned char scaling_matrix,
				 unsigned char scaling_lists)
{
	/*
	 * Builds a picture header with from the given inputs parameters (start of new frame)
	 * Essential we initialise our header structures before building
	 */
	struct h264_picture_header_params ph_params;
	struct mtx_header_element *this_element;
	struct mtx_header_element *element_pointers[MAXNUMBERELEMENTS];

	mtx_header->elements = ELEMENTS_EMPTY;
	this_element = (struct mtx_header_element *)mtx_header->element_stream;
	element_pointers[0] = this_element;

	ph_params.pic_parameter_set_id = mvc_pps ? MVC_PPS_ID : 0;
	ph_params.seq_parameter_set_id = mvc_pps ? MVC_SPS_ID : 0;
	ph_params.entropy_coding_mode_flag = cabac_enabled ? 1 : 0;
	ph_params.weighted_pred_flag = weighted_prediction;
	ph_params.weighted_bipred_idc = weighted_bi_pred;
	ph_params.chroma_qp_index_offset = cqp_offset;
	ph_params.constrained_intra_pred_flag = intra_constrained ? 1 : 0;
	ph_params.transform_8x8_mode_flag = transform_8x8 ? 1 : 0;
	ph_params.pic_scaling_matrix_present_flag = scaling_matrix ? 1 : 0;
	ph_params.use_default_scaling_list = !scaling_lists;
	ph_params.second_chroma_qp_index_offset = cqp_offset;

	h264_write_bits_picture_header(mtx_header, element_pointers, &ph_params, NULL);
	/*Has been used as an index, so need to add 1 for a valid element count */
	mtx_header->elements++;
}

/* SEI_INSERTION */
void h264_write_bits_aud_header(struct mtx_header_params *mtx_header,
				struct mtx_header_element **element_pointers)
{
	/* Essential we insert the element before we try to fill it! */
	insert_element_token(mtx_header, element_pointers, ELEMENT_STARTCODE_RAWDATA);
	/* 00 00 00 01 start code prefix */
	h264_write_bits_startcode_prefix_element(mtx_header, element_pointers, 4);
	/* AUD nal_unit_type = 09 */
	write_upto_8bits_to_elements(mtx_header, element_pointers, 9, 8);

	/* primary_pic_type	 u(3) 0=I slice, 1=P or I slice, 2=P,B or I slice */
	write_upto_8bits_to_elements(mtx_header, element_pointers, 2, 3);
	/* rbsp_trailing_bits */
	write_upto_8bits_to_elements(mtx_header, element_pointers, 1 << 4, 5);

	/* Write terminator */
	write_upto_8bits_to_elements(mtx_header, element_pointers, 0x80, 8);
}

void h264_prepare_aud_header(struct mtx_header_params *mtx_header)
{
	/* Essential we initialise our header structures before building */
	struct mtx_header_element *this_element;
	struct mtx_header_element *element_pointers[MAXNUMBERELEMENTS];

	mtx_header->elements = ELEMENTS_EMPTY;
	this_element = (struct mtx_header_element *)mtx_header->element_stream;
	element_pointers[0] = this_element;

	h264_write_bits_aud_header(mtx_header, element_pointers);
	/*Has been used as an index, so need to add 1 for a valid element count */
	mtx_header->elements++;
}

static void insert_prefix_nal_header(struct mtx_header_params *mtx_header,
				     struct mtx_header_element **element_pointers,
				     struct h264_slice_header_params *slh_params,
				     unsigned char cabac_enabled)
{
	insert_element_token(mtx_header, element_pointers, ELEMENT_STARTCODE_RAWDATA);
	/*Can be 3 or 4 bytes - always 4 bytes in our implementations */
	h264_write_bits_startcode_prefix_element(mtx_header, element_pointers,
						 slh_params->startcode_prefix_size_bytes);

	insert_element_token(mtx_header, element_pointers, ELEMENT_RAWDATA);
	/* forbidden_zero_bit */
	write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 1);
	/*MTX fills this value in */
	insert_element_token(mtx_header, element_pointers, ELEMENT_REFERENCE);
	insert_element_token(mtx_header, element_pointers, ELEMENT_RAWDATA);
	/* nal unit type */
	write_upto_8bits_to_elements(mtx_header, element_pointers, 14, 5);
	/* SVC extension flag */
	write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 1);
	/* non_idr_flag flag */
	if (slh_params->slice_frame_type == SLHP_IDR_SLICEFRAME_TYPE)
		write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 1);
	else
		/* non_idr_flag flag */
		write_upto_8bits_to_elements(mtx_header, element_pointers, 1, 1);
	/* priority_id flag */
	write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 6);
	/* view_id flag */
	write_upto_32bits_to_elements(mtx_header, element_pointers, 0, 10);
	/* temporal_id flag */
	insert_element_token(mtx_header, element_pointers, ELEMENT_TEMPORAL_ID);
	/* anchor_pic_flag */
	insert_element_token(mtx_header, element_pointers, ELEMENT_ANCHOR_PIC_FLAG);

	insert_element_token(mtx_header, element_pointers, ELEMENT_RAWDATA);
	/* interview flag */
	if (slh_params->slice_frame_type == SLHP_B_SLICEFRAME_TYPE)
		write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 1);
	else
		/* interview flag */
		write_upto_8bits_to_elements(mtx_header, element_pointers, 1, 1);
	/* reserved one bit */
	write_upto_8bits_to_elements(mtx_header, element_pointers, 1, 1);
}

/* helper function to start new raw data block */
static unsigned char start_next_rawdata_element = FALSE;
static void check_start_rawdata_element(struct mtx_header_params *mtx_header,
					struct mtx_header_element **element_pointers)
{
	if (start_next_rawdata_element) {
		insert_element_token(mtx_header, element_pointers, ELEMENT_RAWDATA);
		start_next_rawdata_element = FALSE;
	}
}

void h264_write_bits_extension_slice_header(struct mtx_header_params *mtx_header,
					    struct mtx_header_element **element_pointers,
					    struct h264_slice_header_params *slh_params,
					    unsigned char cabac_enabled,
					    unsigned char is_idr)
{
	start_next_rawdata_element = FALSE;

	insert_element_token(mtx_header, element_pointers, ELEMENT_STARTCODE_RAWDATA);
	/*Can be 3 or 4 bytes - always 4 bytes in our implementations */
	h264_write_bits_startcode_prefix_element(mtx_header, element_pointers,
						 slh_params->startcode_prefix_size_bytes);

	/* GENERATES THE FIRST ELEMENT OF THE H264_SLICE_HEADER() STRUCTURE
	 * ELEMENT BITCOUNT: 8

	 * StartCodePrefix Pregenerated in: Build_H264_4Byte_StartCodePrefix_Element()
	 * (4 or 3 bytes) (3 bytes when slice is first in a picture without
	 * sequence/picture_header before picture Byte aligned (bit 32 or 24)
	 * NOTE: Slice_Type and Frame_Type are always the same, hence slice_frame_type
	 */
	 /* forbidden_zero_bit */
	write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 1);
	/*MTX fills this value in */
	insert_element_token(mtx_header, element_pointers, ELEMENT_REFERENCE);
	insert_element_token(mtx_header, element_pointers, ELEMENT_RAWDATA);
	/* nal_unit_type for coded_slice_extension */
	write_upto_8bits_to_elements(mtx_header, element_pointers, 20, 5);
	/* SVC extension flag */
	write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 1);

	if (slh_params->slice_frame_type == SLHP_IDR_SLICEFRAME_TYPE)
		write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 1);
		/* non_idr_flag flag */
	else if ((slh_params->slice_frame_type == SLHP_P_SLICEFRAME_TYPE) && is_idr)
		/* non_idr_flag flag */
		write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 1);
	else
		/* non_idr_flag flag */
		write_upto_8bits_to_elements(mtx_header, element_pointers, 1, 1);
	/* priority_id flag */
	write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 6);
	/* view_id = hardcoded to 1 for dependent view */
	write_upto_32bits_to_elements(mtx_header, element_pointers, 1, 10);
	/* temporal_id flag */
	insert_element_token(mtx_header, element_pointers, ELEMENT_TEMPORAL_ID);
	/* anchor_pic_flag */
	insert_element_token(mtx_header, element_pointers, ELEMENT_ANCHOR_PIC_FLAG);

	insert_element_token(mtx_header, element_pointers, ELEMENT_RAWDATA);
	/* interview flag is always FALSE for dependent frames */
	write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 1);
	/* reserved one bit */
	write_upto_8bits_to_elements(mtx_header, element_pointers, 1, 1);

	/* slice header */
	/*MTX fills this value in */
	insert_element_token(mtx_header, element_pointers, ELEMENT_CURRMBNR);

	insert_element_token(mtx_header, element_pointers, ELEMENT_RAWDATA);

	/* GENERATES THE SECOND ELEMENT OF THE H264_SLICE_HEADER() STRUCTURE */

	generate_ue(mtx_header, element_pointers,
		    (unsigned int)((slh_params->slice_frame_type ==
				   SLHP_IDR_SLICEFRAME_TYPE) ? SLHP_I_SLICEFRAME_TYPE :
				   slh_params->slice_frame_type));
	/*slice_type ue(v): 0 for P-slice, 1 for B-slice, 2 for I-slice */

	/* pic_parameter_set_id = 1 for dependent view */
	generate_ue(mtx_header, element_pointers, 1);

	/* Insert token to tell MTX to insert frame_num */
	insert_element_token(mtx_header, element_pointers, ELEMENT_FRAME_NUM);
	start_next_rawdata_element = TRUE;

	if (slh_params->pic_interlace ||
	    slh_params->slice_frame_type == SLHP_IDR_SLICEFRAME_TYPE) {
		/* interlaced encoding */
		if (slh_params->pic_interlace) {
			check_start_rawdata_element(mtx_header, element_pointers);
			/* field_pic_flag = 1 */
			write_upto_8bits_to_elements(mtx_header, element_pointers, 1, 1);
			/* Insert token to tell MTX to insert BOTTOM_FIELD flag if required */
			insert_element_token(mtx_header, element_pointers, ELEMENT_BOTTOM_FIELD);
			start_next_rawdata_element = TRUE;
		}
	}

	if (slh_params->slice_frame_type == SLHP_IDR_SLICEFRAME_TYPE || (is_idr)) {
		check_start_rawdata_element(mtx_header, element_pointers);
		/* idr_pic_id ue(v) = 0 (1b) in Topaz */
		write_upto_8bits_to_elements(mtx_header, element_pointers, 1, 1);
	}
	/* Insert token to tell MTX to insert pic_order_cnt_lsb */
	insert_element_token(mtx_header, element_pointers, ELEMENT_PIC_ORDER_CNT);
	start_next_rawdata_element = TRUE;

	if (slh_params->slice_frame_type == SLHP_B_SLICEFRAME_TYPE)
		/* Insert token to tell MTX to insert direct_spatial_mv_pred_flag */
		insert_element_token(mtx_header, element_pointers, ELEMENT_DIRECT_SPATIAL_MV_FLAG);

	if (slh_params->slice_frame_type == SLHP_P_SLICEFRAME_TYPE) {
		insert_element_token(mtx_header, element_pointers, ELEMENT_NUM_REF_IDX_ACTIVE);
		start_next_rawdata_element = TRUE;
	} else if (slh_params->slice_frame_type == SLHP_B_SLICEFRAME_TYPE) {
		check_start_rawdata_element(mtx_header, element_pointers);
		/* num_ref_idx_active_override_flag (1 bit) = 0 in Topaz */
		write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 1);
	}

	/* reference picture list modification */
	if (slh_params->slice_frame_type != SLHP_I_SLICEFRAME_TYPE &&
	    slh_params->slice_frame_type != SLHP_IDR_SLICEFRAME_TYPE) {
		/* Insert token to tell MTX to insert BOTTOM_FIELD flag if required */
		insert_element_token(mtx_header, element_pointers, ELEMENT_REORDER_L0);
		start_next_rawdata_element = TRUE;
	}

	if (slh_params->slice_frame_type == SLHP_B_SLICEFRAME_TYPE) {
		check_start_rawdata_element(mtx_header, element_pointers);
		/* ref_pic_list_ordering_flag_l1 (1 bit) = 0, no reference
		 * picture ordering in Topaz
		 */
		write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 1);
	}

	if (slh_params->slice_frame_type == SLHP_IDR_SLICEFRAME_TYPE || (is_idr)) {
		check_start_rawdata_element(mtx_header, element_pointers);
		/* no_output_of_prior_pics_flag (1 bit) = 0 */
		write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 1);
		/* long_term_reference_flag (1 bit) = 0 */
		write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 1);
	} else {
		/*MTX fills this value in */
		insert_element_token(mtx_header, element_pointers, ELEMENT_ADAPTIVE);
		start_next_rawdata_element = TRUE;
	}

	if (cabac_enabled && (slh_params->slice_frame_type == SLHP_P_SLICEFRAME_TYPE ||
			      slh_params->slice_frame_type == SLHP_B_SLICEFRAME_TYPE)) {
		check_start_rawdata_element(mtx_header, element_pointers);
		/* hard code cabac_init_idc value of 0 */
		generate_ue(mtx_header, element_pointers, 0);
	}

	/*MTX fills this value in */
	insert_element_token(mtx_header, element_pointers, ELEMENT_SQP);
	insert_element_token(mtx_header, element_pointers, ELEMENT_RAWDATA);

	/* GENERATES ELEMENT OF THE H264_SLICE_HEADER() STRUCTURE
	 * ELEMENT BITCOUNT: 11
	 */
	/*disable_deblocking_filter_idc ue(v) = 2? */
	generate_ue(mtx_header, element_pointers, slh_params->disable_deblocking_filter_idc);
	if (slh_params->disable_deblocking_filter_idc != 1) {
		/*slice_alpha_c0_offset_div2 se(v) = 0 (1b) in Topaz */
		generate_se(mtx_header, element_pointers, slh_params->deb_alpha_offset_div2);
		/*slice_beta_offset_div2 se(v) = 0 (1b) in Topaz */
		generate_se(mtx_header, element_pointers, slh_params->deb_beta_offset_div2);
	}
	/*
	 * num_slice_groups_minus1 ==0 in Topaz, so no slice_group_change_cycle field here
	 * no byte alignment at end of slice headers
	 */
}

void h264_write_bits_slice_header(struct mtx_header_params *mtx_header,
				  struct mtx_header_element **element_pointers,
				  struct h264_slice_header_params *slh_params,
				  unsigned char cabac_enabled, unsigned char is_idr)
{
	start_next_rawdata_element = FALSE;
	if (slh_params->mvc_view_idx == (unsigned short)(NON_MVC_VIEW)) {
		insert_element_token(mtx_header, element_pointers, ELEMENT_STARTCODE_RAWDATA);
	} else if (slh_params->mvc_view_idx == MVC_BASE_VIEW_IDX) {
		insert_prefix_nal_header(mtx_header, element_pointers, slh_params, cabac_enabled);
		insert_element_token(mtx_header, element_pointers, ELEMENT_STARTCODE_MIDHDR);
	} else {
		/*Insert  */
		h264_write_bits_extension_slice_header(mtx_header, element_pointers,
						       slh_params, cabac_enabled, is_idr);
		return;
	}

	h264_write_bits_startcode_prefix_element(mtx_header, element_pointers,
						 /*Can be 3 or 4 bytes - always 4
						  * bytes in our implementations
						  */
						 slh_params->startcode_prefix_size_bytes);

	/* GENERATES THE FIRST ELEMENT OF THE H264_SLICE_HEADER() STRUCTURE
	 * ELEMENT BITCOUNT: 8

	 * StartCodePrefix Pregenerated in: Build_H264_4Byte_StartCodePrefix_Element()
	 * (4 or 3 bytes) (3 bytes when slice is first in a picture without
	 * sequence/picture_header before picture Byte aligned (bit 32 or 24)
	 * NOTE: Slice_Type and Frame_Type are always the same, hence slice_frame_type
	 */
	write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 1);   /* forbidden_zero_bit */

	/*MTX fills this value in */
	insert_element_token(mtx_header, element_pointers, ELEMENT_REFERENCE);
	insert_element_token(mtx_header, element_pointers, ELEMENT_RAWDATA);

	write_upto_8bits_to_elements(mtx_header, element_pointers,
				     /* nal_unit_tpye (5 bits) = I-frame IDR, and 1 for  rest */
				     ((slh_params->slice_frame_type == SLHP_IDR_SLICEFRAME_TYPE ?
				       5 : 1)), 5);

	/*MTX fills this value in */
	insert_element_token(mtx_header, element_pointers, ELEMENT_CURRMBNR);

	insert_element_token(mtx_header, element_pointers, ELEMENT_RAWDATA);

	/* GENERATES THE SECOND ELEMENT OF THE H264_SLICE_HEADER() STRUCTURE */

	generate_ue(mtx_header, element_pointers,
		    (unsigned int)((slh_params->slice_frame_type == SLHP_IDR_SLICEFRAME_TYPE) ?
			/*slice_type ue(v): 0 for P-slice, 1 for B-slice, 2 for I-slice */
				   SLHP_I_SLICEFRAME_TYPE : slh_params->slice_frame_type));

	if (slh_params->mvc_view_idx != (unsigned short)(NON_MVC_VIEW))
		/* pic_parameter_set_id = 0 */
		generate_ue(mtx_header, element_pointers, slh_params->mvc_view_idx);
	else
		generate_ue(mtx_header, element_pointers, 0);  /* pic_parameter_set_id = 0 */
	/* Insert token to tell MTX to insert frame_num */
	insert_element_token(mtx_header, element_pointers, ELEMENT_FRAME_NUM);

	if (slh_params->pic_interlace ||
	    slh_params->slice_frame_type == SLHP_IDR_SLICEFRAME_TYPE) {
		insert_element_token(mtx_header, element_pointers, ELEMENT_RAWDATA);
		/* interlaced encoding */
		if (slh_params->pic_interlace) {
			/* field_pic_flag = 1 */
			write_upto_8bits_to_elements(mtx_header, element_pointers, 1, 1);
			/* Insert token to tell MTX to insert BOTTOM_FIELD flag if required */
			insert_element_token(mtx_header, element_pointers, ELEMENT_BOTTOM_FIELD);
			insert_element_token(mtx_header, element_pointers, ELEMENT_RAWDATA);
		}

		if (slh_params->slice_frame_type == SLHP_IDR_SLICEFRAME_TYPE)
			/* idr_pic_id ue(v) */
			insert_element_token(mtx_header, element_pointers, ELEMENT_IDR_PIC_ID);
	}
	/* Insert token to tell MTX to insert pic_order_cnt_lsb */
	insert_element_token(mtx_header, element_pointers, ELEMENT_PIC_ORDER_CNT);

	if (slh_params->slice_frame_type == SLHP_B_SLICEFRAME_TYPE)
		/* Insert token to tell MTX to insert direct_spatial_mv_pred_flag */
		insert_element_token(mtx_header, element_pointers, ELEMENT_DIRECT_SPATIAL_MV_FLAG);

	if (slh_params->slice_frame_type == SLHP_P_SLICEFRAME_TYPE) {
		/* Insert token to tell MTX to insert override for number of active references */
		insert_element_token(mtx_header, element_pointers, ELEMENT_NUM_REF_IDX_ACTIVE);
	} else if (slh_params->slice_frame_type == SLHP_B_SLICEFRAME_TYPE) {
		insert_element_token(mtx_header, element_pointers, ELEMENT_RAWDATA);
		/* num_ref_idx_active_override_flag (1 bit) = 0 */
		write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 1);
	}

	if (slh_params->slice_frame_type != SLHP_I_SLICEFRAME_TYPE &&
	    slh_params->slice_frame_type != SLHP_IDR_SLICEFRAME_TYPE) {
		/* Insert token to tell MTX to insert reference list 0 reordering */
		insert_element_token(mtx_header, element_pointers, ELEMENT_REORDER_L0);

		if (slh_params->slice_frame_type == SLHP_B_SLICEFRAME_TYPE)
			/* Insert token to tell MTX to insert reference list 1 reordering */
			insert_element_token(mtx_header, element_pointers, ELEMENT_REORDER_L1);
	}

	/* WEIGHTED PREDICTION */
	insert_element_token(mtx_header, element_pointers, ELEMENT_SLICEWEIGHTEDPREDICTIONSTRUCT);
	insert_element_token(mtx_header, element_pointers, ELEMENT_RAWDATA);

	if (slh_params->reference_picture && slh_params->is_longterm_ref) {
		/* adaptive_ref_pic_marking_mode_flag (1 bit) = 0 */
		write_upto_8bits_to_elements(mtx_header, element_pointers, 1, 1);

		/* Clear any existing long-term reference */
		/* memory_management_control_operation */
		generate_ue(mtx_header, element_pointers, 5);

		/* Allow a single long-term reference */
		/* memory_management_control_operation */
		generate_ue(mtx_header, element_pointers, 4);
		/* max_long_term_frame_idx_plus1 */
		generate_ue(mtx_header, element_pointers, 1);

		/* Set current picture as the long-term reference */
		/* memory_management_control_operation */
		generate_ue(mtx_header, element_pointers, 6);
		/* long_term_frame_idx */
		generate_ue(mtx_header, element_pointers, 0);

		/* End */
		/* memory_management_control_operation */
		generate_ue(mtx_header, element_pointers, 0);
	} else {
		/*MTX fills this value in */
		insert_element_token(mtx_header, element_pointers, ELEMENT_ADAPTIVE);
		insert_element_token(mtx_header, element_pointers, ELEMENT_RAWDATA);
	}

	if (cabac_enabled && (slh_params->slice_frame_type == SLHP_P_SLICEFRAME_TYPE ||
			      slh_params->slice_frame_type == SLHP_B_SLICEFRAME_TYPE))
		/* hard code cabac_init_idc value of 0 */
		generate_ue(mtx_header, element_pointers, 0);

	/*MTX fills this value in */
	insert_element_token(mtx_header, element_pointers, ELEMENT_SQP);
	insert_element_token(mtx_header, element_pointers, ELEMENT_RAWDATA);

	/* GENERATES ELEMENT OF THE H264_SLICE_HEADER() STRUCTURE
	 * ELEMENT BITCOUNT: 11
	 */
		/*disable_deblocking_filter_idc ue(v) = 2? */
	generate_ue(mtx_header, element_pointers, slh_params->disable_deblocking_filter_idc);
	if (slh_params->disable_deblocking_filter_idc != 1) {
		/*slice_alpha_c0_offset_div2 se(v) = 0 (1b) in Topaz */
		generate_se(mtx_header, element_pointers, slh_params->deb_alpha_offset_div2);
		/*slice_beta_offset_div2 se(v) = 0 (1b) in Topaz */
		generate_se(mtx_header, element_pointers, slh_params->deb_beta_offset_div2);
	}

	/*
	 * num_slice_groups_minus1 ==0 in Topaz, so no slice_group_change_cycle field here
	 * no byte alignment at end of slice headers
	 */
}

/*
 * Prepare an H264 slice header in a form for the MTX to encode into a
 * bitstream.
 */
void h264_prepare_slice_header(struct mtx_header_params *mtx_header,
			       unsigned char intra_slice, unsigned char inter_b_slice,
			       unsigned char disable_deblocking_filter_idc,
			       unsigned int first_mb_address, unsigned int mb_skip_run,
			       unsigned char cabac_enabled, unsigned char is_interlaced,
			       unsigned char is_idr, unsigned short mvc_view_idx,
			       unsigned char is_longterm_ref)
{
	struct h264_slice_header_params slh_params;
	struct mtx_header_element *this_element;
	struct mtx_header_element *element_pointers[MAXNUMBERELEMENTS];

	slh_params.startcode_prefix_size_bytes  = 4;
	/* pcb -  I think that this is more correct now  -- This should also
	 * work for IDR-P frames which will be marked as SLHP_P_SLICEFRAME_TYPE
	 */
	slh_params.slice_frame_type = intra_slice ? (is_idr ? SLHP_IDR_SLICEFRAME_TYPE :
						     SLHP_I_SLICEFRAME_TYPE) :
						     (inter_b_slice ? SLHP_B_SLICEFRAME_TYPE :
						      SLHP_P_SLICEFRAME_TYPE);

	slh_params.first_mb_address = first_mb_address;
	slh_params.disable_deblocking_filter_idc = (unsigned char)disable_deblocking_filter_idc;
	slh_params.pic_interlace = is_interlaced;
	slh_params.deb_alpha_offset_div2 = 0;
	slh_params.deb_beta_offset_div2 = 0;
	/* setup the new flags used for B frame as reference */
	slh_params.reference_picture = inter_b_slice ? 0 : 1;
	slh_params.mvc_view_idx = mvc_view_idx;
	slh_params.is_longterm_ref = is_longterm_ref;
	slh_params.log2_max_pic_order_cnt = 2;
	slh_params.longterm_ref_num = 0;
	slh_params.ref_is_longterm_ref[0] = 0;
	slh_params.ref_longterm_ref_num[0] = 0;
	slh_params.ref_is_longterm_ref[1] = 0;
	slh_params.ref_longterm_ref_num[1] = 0;
	/*
	 * Builds a single slice header from the given parameters (mid frame)
	 * Essential we initialise our header structures before building
	 */
	mtx_header->elements = ELEMENTS_EMPTY;
	this_element = (struct mtx_header_element *)mtx_header->element_stream;
	element_pointers[0] = this_element;

	h264_write_bits_slice_header(mtx_header, element_pointers, &slh_params, cabac_enabled,
				     is_idr);
	/*Has been used as an index, so need to add 1 for a valid element count */
	mtx_header->elements++;
}

/*
 * PrepareEncodeSliceParams
 */
unsigned int prepare_encode_slice_params(void *enc_ctx, struct slice_params *slice_params,
					 unsigned char is_intra, unsigned short current_row,
					 unsigned char deblock_idc, unsigned short slice_height,
					 unsigned char is_bpicture, unsigned char field_mode,
					 int fine_y_search_size)
{
	struct img_enc_context *enc;
	struct img_video_context *video;
	unsigned int frame_store_format;
	unsigned char swap_chromas;
	unsigned int mbs_per_kick, kicks_per_slice;
	unsigned int ipe_control;
	enum img_ipe_minblock_size blk_sz;
	struct img_mtx_video_context *mtx_enc_context = NULL;
	unsigned char restrict_4x4_search_size;
	unsigned int lritc_boundary;

	if (!enc_ctx)
		return IMG_ERROR_INVALID_CONTEXT;

	enc = (struct img_enc_context *)enc_ctx;
	video = enc->video;

	if (video->mtx_enc_ctx_mem.cpu_virt)
		mtx_enc_context = (struct img_mtx_video_context *)(&video->mtx_enc_ctx_mem);

	/* We want multiple ones of these so we can submit multiple
	 * slices without having to wait for the next
	 */
	slice_params->flags = 0;
	ipe_control = video->ipe_control;

	/* extract block size */
	blk_sz = (enum img_ipe_minblock_size)F_EXTRACT(ipe_control, TOPAZHP_CR_IPE_BLOCKSIZE);

	/* mask-out the block size bits from ipe_control */
	ipe_control &= ~(F_MASK(TOPAZHP_CR_IPE_BLOCKSIZE));

	switch (video->standard) {
	case IMG_STANDARD_H264:
		if (blk_sz > 2)
			blk_sz = (enum img_ipe_minblock_size)2;
		if (is_bpicture && blk_sz > 1)
			blk_sz = (enum img_ipe_minblock_size)1;

		if (video->mbps >= _1080P_30FPS)
			ipe_control |= F_ENCODE(fine_y_search_size, TOPAZHP_CR_IPE_LRITC_BOUNDARY) |
				F_ENCODE(fine_y_search_size, TOPAZHP_CR_IPE_Y_FINE_SEARCH);
		else
			ipe_control |= F_ENCODE(fine_y_search_size + 1,
					TOPAZHP_CR_IPE_LRITC_BOUNDARY) |
				F_ENCODE(fine_y_search_size, TOPAZHP_CR_IPE_Y_FINE_SEARCH);

		if (video->limit_num_vectors)
			ipe_control |= F_ENCODE(1, TOPAZHP_CR_IPE_MV_NUMBER_RESTRICTION);
		break;
	default:
		break;
	}

	if (video->mbps >= _1080P_30FPS)
		restrict_4x4_search_size = 1;
	else
		restrict_4x4_search_size = 0;

	ipe_control |= F_ENCODE(blk_sz, TOPAZHP_CR_IPE_BLOCKSIZE);

	lritc_boundary =
		(blk_sz !=
		BLK_SZ_16x16) ? (fine_y_search_size + (restrict_4x4_search_size ? 0 : 1)) : 1;
	if (lritc_boundary > 3)
		IMG_DBG_ASSERT(0);

	/* Minimum sub block size to calculate motion vectors for. 0=16x16, 1=8x8, 2=4x4 */
	ipe_control = F_INSERT(ipe_control, blk_sz, TOPAZHP_CR_IPE_BLOCKSIZE);
	ipe_control = F_INSERT(ipe_control, fine_y_search_size, TOPAZHP_CR_IPE_Y_FINE_SEARCH);
	ipe_control = F_INSERT(ipe_control, video->limit_num_vectors,
			       TOPAZHP_CR_IPE_MV_NUMBER_RESTRICTION);

	/* 8x8 search  */
	ipe_control = F_INSERT(ipe_control, lritc_boundary, TOPAZHP_CR_IPE_LRITC_BOUNDARY);
	ipe_control = F_INSERT(ipe_control, restrict_4x4_search_size ? 0 : 1,
			       TOPAZHP_CR_IPE_4X4_SEARCH);

	ipe_control = F_INSERT(ipe_control, video->high_latency, TOPAZHP_CR_IPE_HIGH_LATENCY);

	slice_params->ipe_control = ipe_control;

	if (!is_intra) {
		if (is_bpicture)
			slice_params->flags |= ISINTERB_FLAGS;
		else
			slice_params->flags |= ISINTERP_FLAGS;
	}

	if (video->multi_reference_p && !(is_intra || is_bpicture))
		slice_params->flags |= ISMULTIREF_FLAGS;

	if (video->spatial_direct && is_bpicture)
		slice_params->flags |= SPATIALDIRECT_FLAGS;

	if (is_intra) {
		slice_params->slice_config = F_ENCODE(TOPAZHP_CR_SLICE_TYPE_I_SLICE,
						      TOPAZHP_CR_SLICE_TYPE);
	} else {
		if (is_bpicture)
			slice_params->slice_config = F_ENCODE(TOPAZHP_CR_SLICE_TYPE_B_SLICE,
							      TOPAZHP_CR_SLICE_TYPE);
		else /* p frame */
			slice_params->slice_config = F_ENCODE(TOPAZHP_CR_SLICE_TYPE_P_SLICE,
							      TOPAZHP_CR_SLICE_TYPE);
	}

	mbs_per_kick = video->kick_size;

	/*
	 * we need to figure out the number of kicks and mb's per kick to use.
	 * on H.264 we will use a MB's per kick of basic unit
	 * on other rc varients we will use mb's per kick of width
	 */
	kicks_per_slice = ((slice_height / 16) * (video->width / 16)) / mbs_per_kick;

	IMG_DBG_ASSERT((kicks_per_slice * mbs_per_kick) ==
		((slice_height / 16) * (video->width / 16)));

	/*
	 * need some sensible ones don't look to be implemented yet...
	 * change per stream
	 */
	if (video->format == IMG_CODEC_UY0VY1_8888 || video->format == IMG_CODEC_VY0UY1_8888)
		frame_store_format = 3;
	else if ((video->format == IMG_CODEC_Y0UY1V_8888) ||
		 (video->format == IMG_CODEC_Y0VY1U_8888))
		frame_store_format = 2;
	else if (video->format == IMG_CODEC_420_PL12 || video->format == IMG_CODEC_422_PL12 ||
		 video->format == IMG_CODEC_420_PL12_PACKED ||
		 video->format == IMG_CODEC_420_PL21_PACKED ||
		 video->format == IMG_CODEC_420_PL21 || video->format == IMG_CODEC_422_PL21)
		frame_store_format = 1;
	else
		frame_store_format = 0;

	if (video->format == IMG_CODEC_VY0UY1_8888 || video->format == IMG_CODEC_Y0VY1U_8888 ||
	    ((video->format == IMG_CODEC_420_PL21 ||
	      video->format == IMG_CODEC_420_PL21_PACKED) && mtx_enc_context &&
	     mtx_enc_context->scaler_setup.scaler_control == 0))
		swap_chromas = 1;
	else
		swap_chromas = 0;

	switch (video->standard) {
	case IMG_STANDARD_H264:
		/* H264 */
		slice_params->seq_config = F_ENCODE(0, TOPAZHP_CR_TEMPORAL_PIC0_BELOW_IN_VALID) |
			F_ENCODE(0, TOPAZHP_CR_TEMPORAL_PIC1_BELOW_IN_VALID) |
			F_ENCODE(0, TOPAZHP_CR_ABOVE_OUT_OF_SLICE_VALID) |
			F_ENCODE(1, TOPAZHP_CR_WRITE_TEMPORAL_PIC0_BELOW_VALID) |
			F_ENCODE(0, TOPAZHP_CR_REF_PIC0_VALID) |
			F_ENCODE(0, TOPAZHP_CR_REF_PIC1_VALID) |
			F_ENCODE(!is_bpicture, TOPAZHP_CR_REF_PIC1_EQUAL_PIC0) |
			F_ENCODE(field_mode ? 1 : 0, TOPAZHP_CR_FIELD_MODE) |
			F_ENCODE(swap_chromas, TOPAZHP_CR_FRAME_STORE_CHROMA_SWAP) |
			F_ENCODE(frame_store_format, TOPAZHP_CR_FRAME_STORE_FORMAT) |
			F_ENCODE(TOPAZHP_CR_ENCODER_STANDARD_H264, TOPAZHP_CR_ENCODER_STANDARD) |
			F_ENCODE(deblock_idc == 1 ? 0 : 1, TOPAZHP_CR_DEBLOCK_ENABLE);

		if (video->rc_params.bframes) {
			slice_params->seq_config |=
				F_ENCODE(1, TOPAZHP_CR_WRITE_TEMPORAL_COL_VALID);
			if ((slice_params->flags & ISINTERB_FLAGS) == ISINTERB_FLAGS)
				slice_params->seq_config |= F_ENCODE(1,
						TOPAZHP_CR_TEMPORAL_COL_IN_VALID);
		}
		if (!is_bpicture)
			slice_params->seq_config |=
				F_ENCODE(1, TOPAZHP_CR_WRITE_TEMPORAL_COL_VALID);
		break;

	default:
		break;
	}

	if (is_bpicture) {
		slice_params->seq_config |= F_ENCODE(0, TOPAZHP_CR_TEMPORAL_PIC1_BELOW_IN_VALID) |
			F_ENCODE(0, TOPAZHP_CR_WRITE_TEMPORAL_PIC1_BELOW_VALID) |
			F_ENCODE(1, TOPAZHP_CR_REF_PIC1_VALID) |
			F_ENCODE(1, TOPAZHP_CR_TEMPORAL_COL_IN_VALID);
	}

	if (video->enable_sel_stats_flags & ESF_FIRST_STAGE_STATS)
		slice_params->seq_config |= F_ENCODE(1, TOPAZHP_CR_WRITE_MB_FIRST_STAGE_VALID);

	if (video->enable_sel_stats_flags & ESF_MP_BEST_MB_DECISION_STATS ||
	    video->enable_sel_stats_flags & ESF_MP_BEST_MOTION_VECTOR_STATS) {
		slice_params->seq_config |= F_ENCODE(1, TOPAZHP_CR_BEST_MULTIPASS_OUT_VALID);

		if (!(video->enable_sel_stats_flags & ESF_MP_BEST_MOTION_VECTOR_STATS))
			/* 64 Byte Best Multipass Motion Vector output disabled by default */
			slice_params->seq_config |= F_ENCODE(1, TOPAZHP_CR_BEST_MVS_OUT_DISABLE);
	}

	if (video->enable_inp_ctrl)
		slice_params->seq_config |= F_ENCODE(1, TOPAZHP_CR_MB_CONTROL_IN_VALID);

	return 0;
}

/*
 * Generates the slice params template
 */
void generate_slice_params_template(struct img_enc_context *enc,
				    struct vidio_ddbufinfo *mem_info,
				    enum img_frame_template_type slice_type,
				    unsigned char is_interlaced, int fine_y_search_size)
{
	unsigned char is_intra = ((slice_type == IMG_FRAME_IDR) || (slice_type == IMG_FRAME_INTRA));
	unsigned char is_bframe = (slice_type == IMG_FRAME_INTER_B);
	unsigned char is_idr = ((slice_type == IMG_FRAME_IDR) ||
			(slice_type == IMG_FRAME_INTER_P_IDR));
	struct img_video_context *video = enc->video;
	unsigned short mvc_view_idx = (unsigned short)(NON_MVC_VIEW);
	/* Initialize Slice Params */
	struct slice_params *slice_params_dest;
	unsigned int slice_height = video->picture_height / video->slices_per_picture;

	slice_height &= ~15;

	slice_params_dest = (struct slice_params *)(mem_info->cpu_virt);

	mvc_view_idx = video->mvc_view_idx;

	prepare_encode_slice_params(enc, slice_params_dest, is_intra,
				    0, video->deblock_idc, slice_height, is_bframe,
				    is_interlaced, fine_y_search_size);

	slice_params_dest->template_type = slice_type;

	/* Prepare Slice Header Template */
	switch (video->standard) {
	case IMG_STANDARD_H264:
		h264_prepare_slice_header(&slice_params_dest->slice_hdr_tmpl, is_intra,
					  is_bframe, video->deblock_idc, 0, 0, video->cabac_enabled,
					  is_interlaced, is_idr, mvc_view_idx, FALSE);
		break;

	default:
		break;
	}
}

void h264_write_bits_mvc_sequence_header(struct mtx_header_params *mtx_header,
					 struct mtx_header_element **element_pointers,
					 struct h264_sequence_header_params *sh_params,
					 struct h264_crop_params *crop,
					 struct h264_scaling_matrix_params *scaling_matrix)
{
	int view_idx = 0;
	int num_views = MAX_MVC_VIEWS;

	insert_element_token(mtx_header, element_pointers, ELEMENT_STARTCODE_RAWDATA);
	h264_write_bits_startcode_prefix_element(mtx_header, element_pointers, 4);

	/*
	 * 4 Byte StartCodePrefix Pregenerated in: H264_WriteBits_StartCodePrefix_Element()
	 * Byte aligned (bit 32)
	 */
	/* forbidden_zero_bit=0 */
	write_upto_8bits_to_elements(mtx_header, element_pointers, (0 << 7) |
		(0x3 << 5) |				/* nal_ref_idc=01 (may be 11) */
		(15),					/* nal_unit_type=15 */
		8);

	/*
	 * Byte aligned (bit 40)
	 * profile_idc = 8 bits = 66 for BP (PROFILE_IDC_BP), 77 for MP (PROFILE_IDC_MP)
	 */
	write_upto_8bits_to_elements(mtx_header, element_pointers, 118, 8);

	/* Byte aligned (bit 48) */
	/* constrain_set0_flag = 1 for MP + BP constraints */
	write_upto_8bits_to_elements(mtx_header, element_pointers, (0 << 7) |
		(0 << 6) |		/* constrain_set1_flag  = 1 for MP + BP constraints */
		(0 << 5) |		/* constrain_set2_flag = always 0 in BP/MP */
		(0 << 4),		/* constrain_set3_flag = 1 for level 1b, 0 for others */
		/* reserved_zero_4bits = 0 */
		8);

	/*
	 * Byte aligned (bit 56)
	 * level_idc should be set to 9 in the sps in case of level is Level 1B and the profile
	 * is Multiview High or Stereo High profiles
	 */
	/* level_idc (8 bits) = 9 for 1b, 10xlevel for others */
	write_upto_8bits_to_elements(mtx_header, element_pointers,
				     (sh_params->level == SH_LEVEL_1B) ? 9 :
				     (unsigned char)sh_params->level, 8);

	/* seq_parameter_Set_id = 1 FOR subset-SPS */
	generate_ue(mtx_header, element_pointers, MVC_SPS_ID);
	generate_ue(mtx_header, element_pointers, 1);           /* chroma_format_idc = 1 */
	generate_ue(mtx_header, element_pointers, 0);           /* bit_depth_luma_minus8 = 0 */
	generate_ue(mtx_header, element_pointers, 0);           /* bit_depth_chroma_minus8 = 0 */
	write_upto_8bits_to_elements(mtx_header, element_pointers, sh_params->is_lossless ? 1 : 0,
				     1);       /* qpprime_y_zero_transform_bypass_flag = 0 */

	if (sh_params->use_default_scaling_list || sh_params->seq_scaling_matrix_present_flag) {
		/* seq_scaling_matrix_present_flag */
		write_upto_8bits_to_elements(mtx_header, element_pointers, 1, 1);
		if (!sh_params->use_default_scaling_list) {
			h264_write_bits_scaling_lists(mtx_header, element_pointers, scaling_matrix,
						      TRUE);
			insert_element_token(mtx_header, element_pointers, ELEMENT_RAWDATA);
		} else {
			/* seq_scaling_list_present_flag[i] = 0; 0 < i < 8 */
			write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 8);
		}
	} else {
		/* seq_scaling_matrix_present_flag */
		write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 1);
	}

	generate_ue(mtx_header, element_pointers, 1);           /* log2_max_frame_num_minus4 = 1 */
	generate_ue(mtx_header, element_pointers, 0);           /* pic_order_cnt_type = 0 */
	/* log2_max_pic_order_cnt_Isb_minus4 = 2 */
	generate_ue(mtx_header, element_pointers, 2);

	/*num_ref_frames ue(2), typically 2 */
	generate_ue(mtx_header, element_pointers, sh_params->max_num_ref_frames);
	/* Bytes aligned (bit 72) */
	/* gaps_in_frame_num_value_allowed_Flag   - (1 bit) */
	write_upto_8bits_to_elements(mtx_header, element_pointers,
				     sh_params->gaps_in_frame_num_value, 1);

	/*
	 * GENERATES THE SECOND, VARIABLE LENGTH, ELEMENT OF THE H264_SEQUENCE_HEADER() STRUCTURE
	 * ELEMENT BITCOUNT: xx
	 */
	/*pic_width_in_mbs_minus1: ue(v) from 10 to 44 (176 to 720 pixel per row) */
	generate_ue(mtx_header, element_pointers, sh_params->width_in_mbs_minus1);
	/*pic_height_in_maps_units_minus1: ue(v) Value from 8 to 35 (144 to 576 pixels per column)
	 */
	generate_ue(mtx_header, element_pointers, sh_params->height_in_maps_units_minus1);
	/* We don't know the alignment at this point, so will have to use bit writing functions */
	/* frame_mb_only_flag 1=frame encoding, 0=field encoding */
	write_upto_8bits_to_elements(mtx_header, element_pointers, sh_params->frame_mbs_only_flag,
				     1);

	if (!sh_params->frame_mbs_only_flag) /* in the case of interlaced encoding */
	/* mb_adaptive_frame_field_flag = 0 in Topaz(field encoding at the sequence level) */
		write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 1);

	/* direct_8x8_inference_flag=1 in Topaz */
	write_upto_8bits_to_elements(mtx_header, element_pointers, 1, 1);

	if (crop->clip) {
		write_upto_8bits_to_elements(mtx_header, element_pointers, 1, 1);
		generate_ue(mtx_header, element_pointers, crop->left_crop_offset);
		generate_ue(mtx_header, element_pointers, crop->right_crop_offset);
		generate_ue(mtx_header, element_pointers, crop->top_crop_offset);
		generate_ue(mtx_header, element_pointers, crop->bottom_crop_offset);

	} else {
		write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 1);
	}

	/*
	 * GENERATES THE THIRD ELEMENT OF THE H264_SEQUENCE_HEADER() STRUCTURE
	 * ELEMENT BITCOUNT: xx
	 * vui_parameters_present_flag (VUI only in 1st sequence of stream)
	 */
	write_upto_8bits_to_elements(mtx_header, element_pointers,
				     (sh_params->vui_params_present), 1);
	if (sh_params->vui_params_present > 0)
		h264_write_bits_vui_params(mtx_header, element_pointers, &sh_params->vui_params);

	write_upto_8bits_to_elements(mtx_header, element_pointers, 1, 1); /*bit_equal_to_one */

	/* sequence parameter set MVC extension */
	generate_ue(mtx_header, element_pointers, (num_views - 1));     /*num_views_minus1 */
	for (view_idx = 0; view_idx < num_views; view_idx++)
		generate_ue(mtx_header, element_pointers, view_idx);

	/* anchor references */
	for (view_idx = 1; view_idx < num_views; view_idx++) {
		/* num_anchor_refs_l0  = 1; view-1 refers to view-0 */
		generate_ue(mtx_header, element_pointers, 1);
		generate_ue(mtx_header, element_pointers, 0);     /* anchor_ref_l0 = 0 */
		generate_ue(mtx_header, element_pointers, 0);     /* num_anchor_refs_l1  = 0 */
	}

	/* non-anchor references */
	for (view_idx = 1; view_idx < num_views; view_idx++) {
		generate_ue(mtx_header, element_pointers, 1);     /* num_non_anchor_refs_l0  = 0 */
		generate_ue(mtx_header, element_pointers, 0);     /* non_anchor_refs_l0  = 0 */
		generate_ue(mtx_header, element_pointers, 0);     /* num_non_anchor_refs_l1  = 0 */
	}

	generate_ue(mtx_header, element_pointers, 0);/* num_level_values_signaled_minus1  = 0 */

	/* level_idc should be set to 9 in the sps in case of level is
	 * Level 1B and the profile is Multiview High or Stereo High profiles
	 */
	write_upto_8bits_to_elements(mtx_header, element_pointers,
				     (sh_params->level == SH_LEVEL_1B) ? 9 :
				     (unsigned char)sh_params->level,
				     8);/* level_idc (8 bits) = 9 for 1b, 10xlevel for others */
	generate_ue(mtx_header, element_pointers, 0);/* num_applicable_ops_minus1  = 0 */
	/* applicable_ops_temporal_id  = 0 */
	write_upto_8bits_to_elements(mtx_header, element_pointers, 0, 3);
	/* applicable_op_num_target_views_minus1  = 0 */
	generate_ue(mtx_header, element_pointers, 0);
	generate_ue(mtx_header, element_pointers, 0);	/* applicable_op_target_view_id  = 0 */
	generate_ue(mtx_header, element_pointers, 0);	/* applicable_op_num_views_minus1  = 0 */

	write_upto_8bits_to_elements(mtx_header, element_pointers,
				     0,                    /* mvc_vui_parameters_present_flag =0 */
				     1);

	write_upto_8bits_to_elements(mtx_header, element_pointers,
				     0,                 /* additional_extension2_flag =0 */
				     1);

	/* Finally we need to align to the next byte */
	/* Tell MTX to insert the byte align field
	 *  (we don't know final stream size for alignment at this point)
	 */
	insert_element_token(mtx_header, element_pointers, ELEMENT_INSERTBYTEALIGN_H264);
}

/*
 * Prepare an H264 SPS in a form for the MTX to encode into a bitstream.
 */
void h264_prepare_mvc_sequence_header(struct mtx_header_params *mtx_header,
				      unsigned int pic_width_in_mbs,
				      unsigned int pic_height_in_mbs,
				      unsigned char vui_params_present,
				      struct h264_vui_params *params,
				      struct h264_crop_params *crop,
				      struct h264_sequence_header_params *sh_params)
{
	struct mtx_header_element *this_element;
	struct mtx_header_element *element_pointers[MAXNUMBERELEMENTS];

	/*
	 * Builds a sequence, picture and slice header with from the given inputs
	 * parameters (start of new frame) Essential we initialise our header
	 * structures before building
	 */
	mtx_header->elements = ELEMENTS_EMPTY;
	this_element = (struct mtx_header_element *)mtx_header->element_stream;
	element_pointers[0] = this_element;

	h264_write_bits_mvc_sequence_header(mtx_header, element_pointers, sh_params, crop, NULL);
	/* Has been used as an index, so need to add 1 for a valid element count */
	mtx_header->elements++;
}
