// SPDX-License-Identifier: GPL-2.0
/*
 * hevc secure data unit parsing API.
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Angela Stegmaier <angelabaker@ti.com>
 * Re-written for upstreming
 *	Prashanth Kumar Amai <prashanth.ka@pathpartnertech.com>
 *	Sidraya Jayagond <sidraya.bj@pathpartnertech.com>
 */

#include <linux/dma-mapping.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>

#include "bspp_int.h"
#include "hevc_secure_parser.h"
#include "hevcfw_data.h"
#include "pixel_api.h"
#include "swsr.h"
#include "vdec_defs.h"
#include "vdecdd_utils.h"

#if defined(DEBUG_DECODER_DRIVER)
#define BSPP_HEVC_SYNTAX(fmt, ...)      pr_info("[hevc] " fmt, ## __VA_ARGS__)

#else

#define BSPP_HEVC_SYNTAX(fmt, ...)
#endif

static void HEVC_SWSR_U1(unsigned char *what, unsigned char *where, void *swsr_ctx)
{
	*where = swsr_read_bits(swsr_ctx, 1);
#ifdef DEBUG_DECODER_DRIVER
	pr_info("%s, u(1) : %u", what, *where);
#endif
}

static void HEVC_SWSR_UN(unsigned char *what, unsigned int *where,
			 unsigned char numbits, void *swsr_ctx)
{
	*where = swsr_read_bits(swsr_ctx, numbits);
#ifdef DEBUG_DECODER_DRIVER
	pr_info("%s, u(%u) : %u", what, numbits, *where);
#endif
}

static void HEVC_SWSR_UE(unsigned char *what, unsigned int *where, void *swsr_ctx)
{
	*where = swsr_read_unsigned_expgoulomb(swsr_ctx);
#ifdef DEBUG_DECODER_DRIVER
	pr_info("%s, ue(v) : %u", what, *where);
#endif
}

static void HEVC_SWSR_SE(unsigned char *what, int *where, void *swsr_ctx)
{
	*where = swsr_read_signed_expgoulomb(swsr_ctx);
#ifdef DEBUG_DECODER_DRIVER
	pr_info("%s, se(v) : %u", what, *where);
#endif
}

static void HEVC_SWSR_FN(unsigned char *what, unsigned char *where,
			 unsigned char numbits, unsigned char pattern,
			 enum bspp_error_type *bspperror, void *swsr_ctx)
{
	*where = swsr_read_bits(swsr_ctx, numbits);
#ifdef DEBUG_DECODER_DRIVER
	pr_info("%s, f(%u) : %u", what, numbits, *where);
#endif
	if (*where != pattern) {
		*bspperror |= BSPP_ERROR_INVALID_VALUE;
		pr_warn("Invalid value of %s (f(%u), expected: %u, got: %u)",
			what, numbits, pattern, *where);
	}
}

static void HEVC_UCHECK(unsigned char *what, unsigned int val,
			unsigned int expected,
			enum bspp_error_type *bspperror)
{
	if (val != expected) {
		*bspperror |= BSPP_ERROR_INVALID_VALUE;
		pr_warn("Invalid value of %s (expected: %u, got: %u)",
			what, expected, val);
	}
}

static void HEVC_RANGEUCHECK(unsigned char *what, unsigned int val,
			     unsigned int min, unsigned int max,
	enum bspp_error_type *bspperror)
{
	if ((min > 0 && val < min) || val > max) {
		*bspperror |= BSPP_ERROR_INVALID_VALUE;
		pr_warn("Value of %s out of range (expected: [%u, %u], got: %u)",
			what, min, max, val);
	}
}

static void HEVC_RANGESCHECK(unsigned char *what, int val, int min, int max,
			     enum bspp_error_type *bspperror)
{
	if (val < min || val > max) {
		*bspperror |= BSPP_ERROR_INVALID_VALUE;
		pr_warn("Value of %s out of range (expected: [%d, %d], got: %d)",
			what, min, max, val);
	}
}

#define HEVC_STATIC_ASSERT(expr) ((void)sizeof(unsigned char[1 - 2 * !(expr)]))

#define HEVC_MIN(a, b, type) ({ \
		type __a = a; \
		type __b = b; \
		(((__a) <= (__b)) ? (__a) : (__b)); })
#define HEVC_MAX(a, b, type) ({ \
		type __a = a; \
		type __b = b; \
		(((__a) >= (__b)) ? (__a) : (__b)); })
#define HEVC_ALIGN(_val, _alignment, type) ({ \
		type val = _val; \
		type alignment = _alignment; \
		(((val) + (alignment) - 1) & ~((alignment) - 1)); })

static const enum pixel_fmt_idc pixelformat_idc[] = {
	PIXEL_FORMAT_MONO,
	PIXEL_FORMAT_420,
	PIXEL_FORMAT_422,
	PIXEL_FORMAT_444
};

static enum bspp_error_type bspp_hevc_parse_vps(void *sr_ctx, struct bspp_hevc_vps *vps);

static void bspp_hevc_sublayhrdparams(void *sr_ctx,
				      struct bspp_hevc_hrd_parameters *hrdparams,
				      unsigned char sublayer_id);

static void bspp_hevc_parsehrdparams(void *sr_ctx,
				     struct bspp_hevc_hrd_parameters *hrdparams,
				     unsigned char common_infpresent,
				     unsigned char max_numsublayers_minus1);

static enum bspp_error_type bspp_hevc_parsesps(void *sr_ctx,
					       void *str_res,
					       struct bspp_hevc_sps *sps);

static enum bspp_error_type bspp_hevc_parsepps(void *sr_ctx, void *str_res,
					       struct bspp_hevc_pps *pps);

static int bspp_hevc_reset_ppsinfo(void *secure_ppsinfo);

static void bspp_hevc_dotilecalculations(struct bspp_hevc_sps *sps,
					 struct bspp_hevc_pps *pps);

static enum bspp_error_type bspp_hevc_parse_slicesegmentheader
		(void *sr_ctx, void *str_res,
		 struct bspp_hevc_slice_segment_header *ssh,
		 unsigned char nalunit_type,
		 struct bspp_vps_info **vpsinfo,
		 struct bspp_sequence_hdr_info **spsinfo,
		 struct bspp_pps_info **ppsinfo);

static enum bspp_error_type bspp_hevc_parse_profiletierlevel
			(void *sr_ctx,
			 struct bspp_hevc_profile_tierlevel *ptl,
			 unsigned char vps_maxsublayers_minus1);

static void bspp_hevc_getdefault_scalinglist(unsigned char size_id, unsigned char matrix_id,
					     const unsigned char **default_scalinglist,
					     unsigned int *size);

static enum bspp_error_type bspp_hevc_parse_scalinglistdata
				(void *sr_ctx,
				 struct bspp_hevc_scalinglist_data *scaling_listdata);

static void bspp_hevc_usedefault_scalinglists(struct bspp_hevc_scalinglist_data *scaling_listdata);

static enum bspp_error_type bspp_hevc_parse_shortterm_refpicset
		(void *sr_ctx,
		 struct bspp_hevc_shortterm_refpicset *st_refpicset,
		 unsigned char st_rps_idx,
		 unsigned char in_slice_header);

static void bspp_hevc_fillcommonseqhdr(struct bspp_hevc_sps *sps,
				       struct vdec_comsequ_hdrinfo *common_seq);

static void bspp_hevc_fillpicturehdr(struct vdec_comsequ_hdrinfo *common_seq,
				     enum hevc_nalunittype nalunit_type,
				     struct bspp_pict_hdr_info *picture_hdr,
				     struct bspp_hevc_sps *sps,
				     struct bspp_hevc_pps *pps,
				     struct bspp_hevc_vps *vps);

static void bspp_hevc_fill_fwsps(struct bspp_hevc_sps *sps,
				 struct hevcfw_sequence_ps *fwsps);

static void bspp_hevc_fill_fwst_rps(struct bspp_hevc_shortterm_refpicset *strps,
				    struct hevcfw_short_term_ref_picset *fwstrps);

static void bspp_hevc_fill_fwpps(struct bspp_hevc_pps *pps,
				 struct hevcfw_picture_ps *fw_pps);

static void bspp_hevc_fill_fw_scaling_lists(struct bspp_hevc_pps *pps,
					    struct bspp_hevc_sps *sps,
					    struct hevcfw_picture_ps *fw_pps);

static unsigned int bspp_ceil_log2(unsigned int linear_val);

static unsigned char bspp_hevc_picture_is_irap(enum hevc_nalunittype nalunit_type);

static unsigned char bspp_hevc_picture_is_cra(enum hevc_nalunittype nalunit_type);

static unsigned char bspp_hevc_picture_is_idr(enum hevc_nalunittype nalunit_type);

static unsigned char bspp_hevc_picture_is_bla(enum hevc_nalunittype nalunit_type);

static unsigned char bspp_hevc_picture_getnorasl_outputflag
						(enum hevc_nalunittype nalunit_type,
						 struct bspp_hevc_inter_pict_ctx *inter_pict_ctx);

static unsigned char bspp_hevc_range_extensions_is_enabled
					(struct bspp_hevc_profile_tierlevel *profile_tierlevel);

static int bspp_hevc_unitparser(void *swsr_ctx, struct bspp_unit_data *unitdata)
{
	void *sr_ctx = swsr_ctx;
	int result = 0;
	enum bspp_error_type parse_err = BSPP_ERROR_NONE;
	struct bspp_inter_pict_data *inter_pict_ctx =
				unitdata->parse_state->inter_pict_ctx;
	unsigned char forbidden_zero_bit = 0;
	unsigned char nal_unit_type = 0;
	unsigned char nuh_layer_id = 0;
	unsigned char nuh_temporal_id_plus1 = 0;

	HEVC_SWSR_FN("forbidden_zero_bit", &forbidden_zero_bit, 1, 0, &parse_err, sr_ctx);
	HEVC_SWSR_UN("nal_unit_type", (unsigned int *)&nal_unit_type, 6, sr_ctx);
	/* for current version of HEVC nuh_layer_id "shall be equal to 0" */
	HEVC_SWSR_FN("nuh_layer_id", &nuh_layer_id, 6, 0, &parse_err, sr_ctx);
	HEVC_SWSR_UN("nuh_temporal_id_plus1", (unsigned int *)&nuh_temporal_id_plus1, 3, sr_ctx);

	switch (unitdata->unit_type) {
	case BSPP_UNIT_VPS:
	{
		struct bspp_hevc_vps *vps =
			(struct bspp_hevc_vps *)unitdata->out.vps_info->secure_vpsinfo;

		unitdata->parse_error |= bspp_hevc_parse_vps(sr_ctx, vps);
		unitdata->out.vps_info->vps_id =
			vps->vps_video_parameter_set_id;
	}
	break;

	case BSPP_UNIT_SEQUENCE:
	{
		struct bspp_ddbuf_array_info *tmp;
		struct hevcfw_sequence_ps *fwsps;
		struct vdec_comsequ_hdrinfo *common_seq;
		struct bspp_hevc_sps *sps =
			(struct bspp_hevc_sps *)unitdata->out.sequ_hdr_info->secure_sequence_info;

		unitdata->parse_error |= bspp_hevc_parsesps(sr_ctx,
				unitdata->str_res_handle,
				sps);
		unitdata->out.sequ_hdr_info->sequ_hdr_info.sequ_hdr_id =
			sps->sps_seq_parameter_set_id;

		tmp = &unitdata->out.sequ_hdr_info->fw_sequence;
		/* handle firmware headers */
		fwsps =
		(struct hevcfw_sequence_ps *)((unsigned char *)tmp->ddbuf_info.cpu_virt_addr +
			tmp->buf_offset);

		bspp_hevc_fill_fwsps(sps, fwsps);

		/* handle common sequence header */
		common_seq =
			&unitdata->out.sequ_hdr_info->sequ_hdr_info.com_sequ_hdr_info;

		bspp_hevc_fillcommonseqhdr(sps, common_seq);
	}
	break;

	case BSPP_UNIT_PPS:
	{
		struct bspp_ddbuf_array_info *tmp;
		struct hevcfw_picture_ps *fw_pps;
		struct bspp_hevc_pps *pps =
			(struct bspp_hevc_pps *)unitdata->out.pps_info->secure_pps_info;

		unitdata->parse_error |= bspp_hevc_parsepps(sr_ctx,
				unitdata->str_res_handle,
				pps);
		unitdata->out.pps_info->pps_id = pps->pps_pic_parameter_set_id;

		tmp = &unitdata->out.pps_info->fw_pps;
		/* handle firmware headers */
		fw_pps =
		(struct hevcfw_picture_ps *)((unsigned char *)tmp->ddbuf_info.cpu_virt_addr +
			tmp->buf_offset);
		bspp_hevc_fill_fwpps(pps, fw_pps);
	}
	break;

	case BSPP_UNIT_PICTURE:
	{
		struct bspp_hevc_slice_segment_header ssh;
		struct bspp_vps_info *vps_info = NULL;
		struct bspp_sequence_hdr_info *sequ_hdr_info = NULL;
		struct bspp_hevc_sps *hevc_sps = NULL;
		struct bspp_pps_info *ppsinfo = NULL;
		enum bspp_error_type parse_error;
		struct bspp_ddbuf_array_info *tmp;
		struct hevcfw_picture_ps *fw_pps;
		struct bspp_pict_data *pictdata;
		struct bspp_hevc_pps *pps;

		/*
		 * EOS has to be attached to picture data, so it can be used
		 * for NoRaslOutputFlag calculation in FW
		 */
		inter_pict_ctx->hevc_ctx.eos_detected = 0;
		if (nal_unit_type == HEVC_NALTYPE_EOS) {
			inter_pict_ctx->hevc_ctx.eos_detected = 1;
			break;
		}

		parse_error = bspp_hevc_parse_slicesegmentheader(sr_ctx,
								 unitdata->str_res_handle,
								 &ssh,
								 nal_unit_type,
								 &vps_info,
								 &sequ_hdr_info,
								 &ppsinfo);
		unitdata->parse_error |= parse_error;
		unitdata->slice = 1;

		if (parse_error != BSPP_ERROR_NONE &&
		    parse_error != BSPP_ERROR_CORRECTION_VALIDVALUE) {
			result = IMG_ERROR_CANCELLED;
			break;
		}

		/* if we just started new picture. */
		if (ssh.first_slice_segment_in_pic_flag) {
			tmp = &ppsinfo->fw_pps;
			/* handle firmware headers */
			fw_pps =
			(struct hevcfw_picture_ps *)((unsigned char *)tmp->ddbuf_info.cpu_virt_addr
				+ tmp->buf_offset);

			inter_pict_ctx->hevc_ctx.first_after_eos = 0;
			if (inter_pict_ctx->hevc_ctx.eos_detected) {
				inter_pict_ctx->hevc_ctx.first_after_eos = 1;
				inter_pict_ctx->hevc_ctx.eos_detected = 0;
			}

			/* fill common picture header */
			bspp_hevc_fillpicturehdr(&sequ_hdr_info->sequ_hdr_info.com_sequ_hdr_info,
						 (enum hevc_nalunittype)nal_unit_type,
						 unitdata->out.pict_hdr_info,
						 (struct bspp_hevc_sps *)
						 sequ_hdr_info->secure_sequence_info,
						 (struct bspp_hevc_pps *)ppsinfo->secure_pps_info,
						 (struct bspp_hevc_vps *)vps_info->secure_vpsinfo);

			bspp_hevc_fill_fw_scaling_lists(ppsinfo->secure_pps_info,
							sequ_hdr_info->secure_sequence_info,
							fw_pps);

			pictdata = &unitdata->out.pict_hdr_info->pict_aux_data;
			/*
			 * We have no container for the PPS that passes down
			 * to the kernel, for this reason the hevc secure parser
			 * needs to populate that info into the picture
			 * header PictAuxData.
			 */
			pictdata->bufmap_id = ppsinfo->bufmap_id;
			pictdata->buf_offset = ppsinfo->buf_offset;
			pictdata->pic_data = fw_pps;
			pictdata->id = fw_pps->pps_pic_parameter_set_id;
			pictdata->size = sizeof(*fw_pps);

			ppsinfo->ref_count++;

			/* new Coded Video Sequence indication */
			if (nal_unit_type == HEVC_NALTYPE_IDR_W_RADL ||
			    nal_unit_type == HEVC_NALTYPE_IDR_N_LP ||
			    nal_unit_type == HEVC_NALTYPE_BLA_N_LP ||
			    nal_unit_type == HEVC_NALTYPE_BLA_W_RADL ||
			    nal_unit_type == HEVC_NALTYPE_BLA_W_LP ||
			    nal_unit_type == HEVC_NALTYPE_CRA) {
				unitdata->new_closed_gop = 1;
				inter_pict_ctx->hevc_ctx.seq_pic_count = 0;
			}

			/* Attach SEI data to the picture. */
	if (!inter_pict_ctx->hevc_ctx.sei_info_attached_to_pic) {
				/*
				 *  If there is already a non-empty SEI list
				 *  available
				 */
		if (inter_pict_ctx->hevc_ctx.sei_rawdata_list) {
			/* attach it to the picture header. */
			unitdata->out.pict_hdr_info->hevc_pict_hdr_info.raw_sei_datalist_firstfield
					=
				(void *)inter_pict_ctx->hevc_ctx.sei_rawdata_list;
			inter_pict_ctx->hevc_ctx.sei_info_attached_to_pic = 1;
		} else {
				/* Otherwise expose a handle a picture header field to
				 * attach SEI list later.
				 */
			inter_pict_ctx->hevc_ctx.hndl_pichdr_sei_rawdata_list =
		&unitdata->out.pict_hdr_info->hevc_pict_hdr_info.raw_sei_datalist_firstfield;
			}
	}

			/* Attach raw VUI data to the picture header. */
			hevc_sps = (struct bspp_hevc_sps *)sequ_hdr_info->secure_sequence_info;
			if (hevc_sps->vui_raw_data) {
				hevc_sps->vui_raw_data->ref_count++;
				unitdata->out.pict_hdr_info->hevc_pict_hdr_info.raw_vui_data =
					(void *)hevc_sps->vui_raw_data;
			}

			inter_pict_ctx->hevc_ctx.seq_pic_count++;

			/* NoOutputOfPriorPicsFlag */
			inter_pict_ctx->not_dpb_flush = 0;
			if (unitdata->new_closed_gop &&
			    bspp_hevc_picture_is_irap((enum hevc_nalunittype)nal_unit_type) &&
			    bspp_hevc_picture_getnorasl_outputflag((enum hevc_nalunittype)
								   nal_unit_type,
								   &inter_pict_ctx->hevc_ctx)) {
				if (bspp_hevc_picture_is_cra((enum hevc_nalunittype)nal_unit_type))
					inter_pict_ctx->not_dpb_flush = 1;
				else
					inter_pict_ctx->not_dpb_flush =
						ssh.no_output_of_prior_pics_flag;
			}

			unitdata->parse_state->next_pic_is_new = 0;
		}

		pps = (struct bspp_hevc_pps *)ppsinfo->secure_pps_info;
		unitdata->pict_sequ_hdr_id = pps->pps_seq_parameter_set_id;
	}
	break;

	case BSPP_UNIT_UNCLASSIFIED:
	case BSPP_UNIT_NON_PICTURE:
	case BSPP_UNIT_UNSUPPORTED:
		break;

	default:
		VDEC_ASSERT("Unknown BSPP Unit Type" == NULL);
		break;
	}

	return result;
}

static void bspp_hevc_initialiseparsing(struct bspp_parse_state *parse_state)
{
	/* Indicate that SEI info has not yet been attached to this picture. */
	parse_state->inter_pict_ctx->hevc_ctx.sei_info_attached_to_pic = 0;
}

static void bspp_hevc_finaliseparsing(void *str_alloc, struct bspp_parse_state *parse_state)
{
	/*
	 * If SEI info has not yet been attached to the picture and
	 * there is anything to be attached.
	 */
	if (!parse_state->inter_pict_ctx->hevc_ctx.sei_info_attached_to_pic &&
	    parse_state->inter_pict_ctx->hevc_ctx.sei_rawdata_list) {
		/* attach the SEI list if there is a handle provided for that. */
		if (parse_state->inter_pict_ctx->hevc_ctx.hndl_pichdr_sei_rawdata_list) {
			/* Attach the raw SEI list to the picture. */
			*parse_state->inter_pict_ctx->hevc_ctx.hndl_pichdr_sei_rawdata_list =
				(void *)parse_state->inter_pict_ctx->hevc_ctx.sei_rawdata_list;
			/* Reset the inter-picture data. */
			parse_state->inter_pict_ctx->hevc_ctx.hndl_pichdr_sei_rawdata_list = NULL;
		} else {
			/* Nowhere to attach the raw SEI list, so just free it. */
			bspp_freeraw_sei_datalist
				(str_alloc, parse_state->inter_pict_ctx->hevc_ctx.sei_rawdata_list);
		}
	}

	/* Indicate that SEI info has been attached to the picture. */
	parse_state->inter_pict_ctx->hevc_ctx.sei_info_attached_to_pic = 1;
	/* Reset the inter-picture SEI list. */
	parse_state->inter_pict_ctx->hevc_ctx.sei_rawdata_list = NULL;
}

static enum bspp_error_type bspp_hevc_parse_vps(void *sr_ctx, struct bspp_hevc_vps *vps)
{
	unsigned int parse_err = BSPP_ERROR_NONE;
	unsigned int i, j;

	VDEC_ASSERT(vps);
	VDEC_ASSERT(sr_ctx);

	memset(vps, 0, sizeof(struct bspp_hevc_vps));

	HEVC_SWSR_UN("vps_video_parameter_set_id",
		     (unsigned int *)&vps->vps_video_parameter_set_id, 4, sr_ctx);
	HEVC_SWSR_UN("vps_reserved_three_2bits",
		     (unsigned int *)&vps->vps_reserved_three_2bits, 2, sr_ctx);
	HEVC_SWSR_UN("vps_max_layers_minus1",
		     (unsigned int *)&vps->vps_max_layers_minus1, 6, sr_ctx);
	HEVC_SWSR_UN("vps_max_sub_layers_minus1",
		     (unsigned int *)&vps->vps_max_sub_layers_minus1, 3, sr_ctx);
	HEVC_RANGEUCHECK("vps_max_sub_layers_minus1", vps->vps_max_sub_layers_minus1, 0,
			 HEVC_MAX_NUM_SUBLAYERS - 1, &parse_err);
	HEVC_SWSR_U1("vps_temporal_id_nesting_flag",
		     &vps->vps_temporal_id_nesting_flag, sr_ctx);
	HEVC_SWSR_UN("vps_reserved_0xffff_16bits",
		     (unsigned int *)&vps->vps_reserved_0xffff_16bits, 16, sr_ctx);

	if (vps->vps_max_sub_layers_minus1 == 0)
		HEVC_UCHECK("vps_temporal_id_nesting_flag",
			    vps->vps_temporal_id_nesting_flag, 1, &parse_err);

	parse_err |= bspp_hevc_parse_profiletierlevel(sr_ctx, &vps->profiletierlevel,
						      vps->vps_max_sub_layers_minus1);

	HEVC_SWSR_U1("vps_sub_layer_ordering_info_present_flag",
		     &vps->vps_sub_layer_ordering_info_present_flag, sr_ctx);
	for (i = vps->vps_sub_layer_ordering_info_present_flag ?
		0 : vps->vps_max_sub_layers_minus1;
		i <= vps->vps_max_sub_layers_minus1; ++i) {
		HEVC_SWSR_UE("vps_max_dec_pic_buffering_minus1",
			     (unsigned int *)&vps->vps_max_dec_pic_buffering_minus1[i], sr_ctx);
		HEVC_SWSR_UE("vps_max_num_reorder_pics",
			     (unsigned int *)&vps->vps_max_num_reorder_pics[i], sr_ctx);
		HEVC_SWSR_UE("vps_max_latency_increase_plus1",
			     (unsigned int *)&vps->vps_max_latency_increase_plus1[i], sr_ctx);
	}

	HEVC_SWSR_UN("vps_max_layer_id", (unsigned int *)&vps->vps_max_layer_id, 6, sr_ctx);
	HEVC_SWSR_UE("vps_num_layer_sets_minus1",
		     (unsigned int *)&vps->vps_num_layer_sets_minus1, sr_ctx);

	for (i = 1; i <= vps->vps_num_layer_sets_minus1; ++i) {
		for (j = 0; j <= vps->vps_max_layer_id; ++j) {
			HEVC_SWSR_U1("layer_id_included_flag",
				     &vps->layer_id_included_flag[i][j], sr_ctx);
		}
	}

	HEVC_SWSR_U1("vps_timing_info_present_flag", &vps->vps_timing_info_present_flag, sr_ctx);
	if (vps->vps_timing_info_present_flag) {
		HEVC_SWSR_UN("vps_num_units_in_tick",
			     (unsigned int *)&vps->vps_num_units_in_tick, 32, sr_ctx);
		HEVC_SWSR_UN("vps_time_scale",
			     (unsigned int *)&vps->vps_time_scale, 32, sr_ctx);
		HEVC_SWSR_U1("vps_poc_proportional_to_timing_flag",
			     &vps->vps_poc_proportional_to_timing_flag, sr_ctx);
		if (vps->vps_poc_proportional_to_timing_flag)
			HEVC_SWSR_UE("vps_num_ticks_poc_diff_one_minus1",
				     (unsigned int *)&vps->vps_num_ticks_poc_diff_one_minus1,
				     sr_ctx);

		HEVC_SWSR_UE("vps_num_hrd_parameters",
			     (unsigned int *)&vps->vps_num_hrd_parameters, sr_ctx);

		/* consume hrd_parameters */
		for (i = 0; i < vps->vps_num_hrd_parameters; i++) {
			unsigned short hrd_layer_set_idx;
			unsigned char cprms_present_flag = 1;
			struct bspp_hevc_hrd_parameters hrdparams;

			HEVC_SWSR_UE("hrd_layer_set_idx",
				     (unsigned int *)&hrd_layer_set_idx, sr_ctx);
			if (i > 0)
				HEVC_SWSR_U1("cprms_present_flag", &cprms_present_flag, sr_ctx);

			bspp_hevc_parsehrdparams(sr_ctx, &hrdparams,
						 cprms_present_flag,
						 vps->vps_max_sub_layers_minus1);
		}
	}
	HEVC_SWSR_U1("vps_extension_flag", &vps->vps_extension_flag, sr_ctx);

	return (enum bspp_error_type)parse_err;
}

static void bspp_hevc_sublayhrdparams(void *sr_ctx,
				      struct bspp_hevc_hrd_parameters *hrdparams,
				      unsigned char sublayer_id)
{
	unsigned char i;
	unsigned char cpb_cnt = hrdparams->cpb_cnt_minus1[sublayer_id];
	struct bspp_hevc_sublayer_hrd_parameters *sublay_hrdparams =
		&hrdparams->sublayhrdparams[sublayer_id];

	VDEC_ASSERT(sr_ctx);
	VDEC_ASSERT(hrdparams);
	VDEC_ASSERT(cpb_cnt < HEVC_MAX_CPB_COUNT);
	VDEC_ASSERT(sublayer_id < HEVC_MAX_NUM_SUBLAYERS);

	for (i = 0; i <= cpb_cnt; i++) {
		HEVC_SWSR_UE("bit_rate_value_minus1",
			     (unsigned int *)&sublay_hrdparams->bit_rate_value_minus1[i], sr_ctx);
		HEVC_SWSR_UE("cpb_size_value_minus1",
			     (unsigned int *)&sublay_hrdparams->cpb_size_value_minus1[i], sr_ctx);
		if (hrdparams->sub_pic_hrd_params_present_flag) {
			HEVC_SWSR_UE("cpb_size_du_value_minus1",
				     (unsigned int *)
				     &sublay_hrdparams->cpb_size_du_value_minus1[i],
				     sr_ctx);
			HEVC_SWSR_UE("bit_rate_du_value_minus1",
				     (unsigned int *)
				     &sublay_hrdparams->bit_rate_du_value_minus1[i],
				     sr_ctx);
		}
		HEVC_SWSR_U1("cbr_flag", &sublay_hrdparams->cbr_flag[i], sr_ctx);
	}
}

static void bspp_hevc_parsehrdparams(void *sr_ctx,
				     struct bspp_hevc_hrd_parameters *hrdparams,
				     unsigned char common_infpresent,
				     unsigned char max_numsublayers_minus1)
{
	unsigned char i;

	VDEC_ASSERT(sr_ctx);
	VDEC_ASSERT(hrdparams);
	VDEC_ASSERT(max_numsublayers_minus1 < HEVC_MAX_NUM_SUBLAYERS);

	memset(hrdparams, 0, sizeof(struct bspp_hevc_hrd_parameters));

	if (common_infpresent) {
		HEVC_SWSR_U1("nal_hrd_parameters_present_flag",
			     &hrdparams->nal_hrd_parameters_present_flag, sr_ctx);
		HEVC_SWSR_U1("vcl_hrd_parameters_present_flag",
			     &hrdparams->vcl_hrd_parameters_present_flag, sr_ctx);
		if (hrdparams->nal_hrd_parameters_present_flag ||
		    hrdparams->vcl_hrd_parameters_present_flag) {
			HEVC_SWSR_U1("sub_pic_hrd_params_present_flag",
				     &hrdparams->sub_pic_hrd_params_present_flag,
				     sr_ctx);
			if (hrdparams->sub_pic_hrd_params_present_flag) {
				HEVC_SWSR_UN("tick_divisor_minus2",
					     (unsigned int *)&hrdparams->tick_divisor_minus2,
					     8, sr_ctx);
				HEVC_SWSR_UN
				("du_cpb_removal_delay_increment_length_minus1",
				 (unsigned int *)
				 &hrdparams->du_cpb_removal_delay_increment_length_minus1,
				 5, sr_ctx);
				HEVC_SWSR_U1("sub_pic_cpb_params_in_pic_timing_sei_flag",
					     &hrdparams->sub_pic_cpb_params_in_pic_timing_sei_flag,
					     sr_ctx);
				HEVC_SWSR_UN("dpb_output_delay_du_length_minus1",
					     (unsigned int *)
					     &hrdparams->dpb_output_delay_du_length_minus1,
					     5, sr_ctx);
			}
			HEVC_SWSR_UN("bit_rate_scale",
				     (unsigned int *)&hrdparams->bit_rate_scale, 4, sr_ctx);
			HEVC_SWSR_UN("cpb_size_scale",
				     (unsigned int *)&hrdparams->cpb_size_scale, 4, sr_ctx);
			if (hrdparams->sub_pic_hrd_params_present_flag)
				HEVC_SWSR_UN("cpb_size_du_scale",
					     (unsigned int *)&hrdparams->cpb_size_du_scale,
					     4, sr_ctx);

			HEVC_SWSR_UN("initial_cpb_removal_delay_length_minus1",
				     (unsigned int *)
				     &hrdparams->initial_cpb_removal_delay_length_minus1,
				     5, sr_ctx);
			HEVC_SWSR_UN("au_cpb_removal_delay_length_minus1",
				     (unsigned int *)&hrdparams->au_cpb_removal_delay_length_minus1,
				     5, sr_ctx);
			HEVC_SWSR_UN("dpb_output_delay_length_minus1",
				     (unsigned int *)&hrdparams->dpb_output_delay_length_minus1,
				     5, sr_ctx);
		}
	}
	for (i = 0; i <= max_numsublayers_minus1; i++) {
		HEVC_SWSR_U1("fixed_pic_rate_general_flag",
			     &hrdparams->fixed_pic_rate_general_flag[i], sr_ctx);
		hrdparams->fixed_pic_rate_within_cvs_flag[i] =
			hrdparams->fixed_pic_rate_general_flag[i];
		if (!hrdparams->fixed_pic_rate_general_flag[i])
			HEVC_SWSR_U1("fixed_pic_rate_within_cvs_flag",
				     &hrdparams->fixed_pic_rate_within_cvs_flag[i],
				     sr_ctx);

		if (hrdparams->fixed_pic_rate_within_cvs_flag[i])
			HEVC_SWSR_UE("elemental_duration_in_tc_minus1",
				     (unsigned int *)&hrdparams->elemental_duration_in_tc_minus1[i],
				     sr_ctx);
		else
			HEVC_SWSR_U1("low_delay_hrd_flag",
				     &hrdparams->low_delay_hrd_flag[i], sr_ctx);

		if (!hrdparams->low_delay_hrd_flag[i])
			HEVC_SWSR_UE("cpb_cnt_minus1",
				     (unsigned int *)&hrdparams->cpb_cnt_minus1[i], sr_ctx);

		if (hrdparams->nal_hrd_parameters_present_flag)
			bspp_hevc_sublayhrdparams(sr_ctx, hrdparams, i);

		if (hrdparams->vcl_hrd_parameters_present_flag)
			bspp_hevc_sublayhrdparams(sr_ctx, hrdparams, i);
	}
}

static enum bspp_error_type bspp_hevc_parsevui_parameters
			(void *sr_ctx,
			 struct bspp_hevc_vui_params *vui_params,
			 unsigned char sps_max_sub_layers_minus1)
{
	enum bspp_error_type parse_err = BSPP_ERROR_NONE;

	VDEC_ASSERT(sr_ctx);
	VDEC_ASSERT(vui_params);

	memset(vui_params, 0, sizeof(struct bspp_hevc_vui_params));

	HEVC_SWSR_U1("aspect_ratio_info_present_flag",
		     &vui_params->aspect_ratio_info_present_flag, sr_ctx);
	if (vui_params->aspect_ratio_info_present_flag) {
		HEVC_SWSR_UN("aspect_ratio_idc",
			     (unsigned int *)&vui_params->aspect_ratio_idc, 8, sr_ctx);
		if (vui_params->aspect_ratio_idc == HEVC_EXTENDED_SAR) {
			HEVC_SWSR_UN("sar_width",
				     (unsigned int *)&vui_params->sar_width, 16, sr_ctx);
			HEVC_SWSR_UN("sar_height",
				     (unsigned int *)&vui_params->sar_height, 16, sr_ctx);
		}
	}
	HEVC_SWSR_U1("overscan_info_present_flag",
		     &vui_params->overscan_info_present_flag, sr_ctx);

	if (vui_params->overscan_info_present_flag)
		HEVC_SWSR_U1("overscan_appropriate_flag",
			     &vui_params->overscan_appropriate_flag, sr_ctx);

	HEVC_SWSR_U1("video_signal_type_present_flag",
		     &vui_params->video_signal_type_present_flag, sr_ctx);

	if (vui_params->video_signal_type_present_flag) {
		HEVC_SWSR_UN("video_format",
			     (unsigned int *)&vui_params->video_format, 3, sr_ctx);
		HEVC_SWSR_U1("video_full_range_flag",
			     &vui_params->video_full_range_flag, sr_ctx);
		HEVC_SWSR_U1("colour_description_present_flag",
			     &vui_params->colour_description_present_flag,
			     sr_ctx);
		if (vui_params->colour_description_present_flag) {
			HEVC_SWSR_UN("colour_primaries",
				     (unsigned int *)&vui_params->colour_primaries, 8, sr_ctx);
			HEVC_SWSR_UN("transfer_characteristics",
				     (unsigned int *)&vui_params->transfer_characteristics,
				     8, sr_ctx);
			HEVC_SWSR_UN("matrix_coeffs",
				     (unsigned int *)&vui_params->matrix_coeffs, 8, sr_ctx);
		}
	}

	HEVC_SWSR_U1("chroma_loc_info_present_flag",
		     &vui_params->chroma_loc_info_present_flag, sr_ctx);
	if (vui_params->chroma_loc_info_present_flag) {
		HEVC_SWSR_UE("chroma_sample_loc_type_top_field",
			     (unsigned int *)&vui_params->chroma_sample_loc_type_top_field,
			     sr_ctx);
		HEVC_RANGEUCHECK("chroma_sample_loc_type_top_field",
				 vui_params->chroma_sample_loc_type_top_field,
				 0, 5, &parse_err);
		HEVC_SWSR_UE("chroma_sample_loc_type_bottom_field",
			     (unsigned int *)&vui_params->chroma_sample_loc_type_bottom_field,
			     sr_ctx);
		HEVC_RANGEUCHECK("chroma_sample_loc_type_bottom_field",
				 vui_params->chroma_sample_loc_type_bottom_field,
				 0, 5, &parse_err);
	}
	HEVC_SWSR_U1("neutral_chroma_indication_flag",
		     &vui_params->neutral_chroma_indication_flag, sr_ctx);
	HEVC_SWSR_U1("field_seq_flag",
		     &vui_params->field_seq_flag, sr_ctx);
	HEVC_SWSR_U1("frame_field_info_present_flag",
		     &vui_params->frame_field_info_present_flag, sr_ctx);
	HEVC_SWSR_U1("default_display_window_flag",
		     &vui_params->default_display_window_flag, sr_ctx);
	if (vui_params->default_display_window_flag) {
		HEVC_SWSR_UE("def_disp_win_left_offset",
			     (unsigned int *)&vui_params->def_disp_win_left_offset, sr_ctx);
		HEVC_SWSR_UE("def_disp_win_right_offset",
			     (unsigned int *)&vui_params->def_disp_win_right_offset, sr_ctx);
		HEVC_SWSR_UE("def_disp_win_top_offset",
			     (unsigned int *)&vui_params->def_disp_win_top_offset, sr_ctx);
		HEVC_SWSR_UE("def_disp_win_bottom_offset",
			     (unsigned int *)&vui_params->def_disp_win_bottom_offset, sr_ctx);
	}
	HEVC_SWSR_U1("vui_timing_info_present_flag",
		     &vui_params->vui_timing_info_present_flag, sr_ctx);
	if (vui_params->vui_timing_info_present_flag) {
		HEVC_SWSR_UN("vui_num_units_in_tick",
			     (unsigned int *)&vui_params->vui_num_units_in_tick, 32, sr_ctx);
		HEVC_SWSR_UN("vui_time_scale",
			     (unsigned int *)&vui_params->vui_time_scale, 32, sr_ctx);
		HEVC_SWSR_U1("vui_poc_proportional_to_timing_flag",
			     &vui_params->vui_poc_proportional_to_timing_flag,
			     sr_ctx);
		if (vui_params->vui_poc_proportional_to_timing_flag)
			HEVC_SWSR_UE("vui_num_ticks_poc_diff_one_minus1",
				     (unsigned int *)&vui_params->vui_num_ticks_poc_diff_one_minus1,
				     sr_ctx);

		HEVC_SWSR_U1("vui_hrd_parameters_present_flag",
			     &vui_params->vui_hrd_parameters_present_flag,
			     sr_ctx);
		if (vui_params->vui_hrd_parameters_present_flag)
			bspp_hevc_parsehrdparams(sr_ctx, &vui_params->vui_hrd_params,
						 1, sps_max_sub_layers_minus1);
	}
	HEVC_SWSR_U1("bitstream_restriction_flag",
		     &vui_params->bitstream_restriction_flag, sr_ctx);

	if (vui_params->bitstream_restriction_flag) {
		HEVC_SWSR_U1("tiles_fixed_structure_flag",
			     &vui_params->tiles_fixed_structure_flag, sr_ctx);
		HEVC_SWSR_U1("motion_vectors_over_pic_boundaries_flag",
			     &vui_params->motion_vectors_over_pic_boundaries_flag,
			     sr_ctx);
		HEVC_SWSR_U1("restricted_ref_pic_lists_flag",
			     &vui_params->restricted_ref_pic_lists_flag, sr_ctx);

		HEVC_SWSR_UE("min_spatial_segmentation_idc",
			     (unsigned int *)&vui_params->min_spatial_segmentation_idc, sr_ctx);
		HEVC_RANGEUCHECK("min_spatial_segmentation_idc",
				 vui_params->min_spatial_segmentation_idc,
				 0, 4095, &parse_err);

		HEVC_SWSR_UE("max_bytes_per_pic_denom",
			     (unsigned int *)&vui_params->max_bytes_per_pic_denom, sr_ctx);
		HEVC_RANGEUCHECK("max_bytes_per_pic_denom", vui_params->max_bytes_per_pic_denom,
				 0, 16, &parse_err);

		HEVC_SWSR_UE("max_bits_per_min_cu_denom",
			     (unsigned int *)&vui_params->max_bits_per_min_cu_denom, sr_ctx);
		HEVC_RANGEUCHECK("max_bits_per_min_cu_denom", vui_params->max_bits_per_min_cu_denom,
				 0, 16, &parse_err);

		HEVC_SWSR_UE("log2_max_mv_length_horizontal",
			     (unsigned int *)&vui_params->log2_max_mv_length_horizontal, sr_ctx);
		HEVC_RANGEUCHECK("log2_max_mv_length_horizontal",
				 vui_params->log2_max_mv_length_horizontal,
				 0, 16, &parse_err);

		HEVC_SWSR_UE("log2_max_mv_length_vertical",
			     (unsigned int *)&vui_params->log2_max_mv_length_vertical, sr_ctx);
		HEVC_RANGEUCHECK("log2_max_mv_length_vertical",
				 vui_params->log2_max_mv_length_vertical,
				 0, 15, &parse_err);
	}

	return parse_err;
}

static enum bspp_error_type bspp_hevc_parse_spsrange_extensions
					(void *sr_ctx,
					 struct bspp_hevc_sps_range_exts *range_exts)
{
	enum bspp_error_type parse_err = BSPP_ERROR_NONE;

	VDEC_ASSERT(sr_ctx);
	VDEC_ASSERT(range_exts);

	memset(range_exts, 0, sizeof(struct bspp_hevc_sps_range_exts));

	HEVC_SWSR_U1("transform_skip_rotation_enabled_flag",
		     &range_exts->transform_skip_rotation_enabled_flag, sr_ctx);
	HEVC_SWSR_U1("transform_skip_context_enabled_flag",
		     &range_exts->transform_skip_context_enabled_flag, sr_ctx);
	HEVC_SWSR_U1("implicit_rdpcm_enabled_flag",
		     &range_exts->implicit_rdpcm_enabled_flag, sr_ctx);
	HEVC_SWSR_U1("explicit_rdpcm_enabled_flag",
		     &range_exts->explicit_rdpcm_enabled_flag, sr_ctx);
	HEVC_SWSR_U1("extended_precision_processing_flag",
		     &range_exts->extended_precision_processing_flag, sr_ctx);
	HEVC_UCHECK("extended_precision_processing_flag",
		    range_exts->extended_precision_processing_flag,
		    0, &parse_err);
	HEVC_SWSR_U1("intra_smoothing_disabled_flag",
		     &range_exts->intra_smoothing_disabled_flag, sr_ctx);
	HEVC_SWSR_U1("high_precision_offsets_enabled_flag",
		     &range_exts->high_precision_offsets_enabled_flag, sr_ctx);
	HEVC_SWSR_U1("persistent_rice_adaptation_enabled_flag",
		     &range_exts->persistent_rice_adaptation_enabled_flag,
		     sr_ctx);
	HEVC_SWSR_U1("cabac_bypass_alignment_enabled_flag",
		     &range_exts->cabac_bypass_alignment_enabled_flag, sr_ctx);

	return parse_err;
}

static unsigned char
bspp_hevc_checksps_range_extensions(struct bspp_hevc_sps_range_exts *range_exts)
{
	VDEC_ASSERT(range_exts);

	if (range_exts->transform_skip_rotation_enabled_flag ||
	    range_exts->transform_skip_context_enabled_flag ||
	    range_exts->implicit_rdpcm_enabled_flag ||
	    range_exts->explicit_rdpcm_enabled_flag ||
	    range_exts->extended_precision_processing_flag ||
	    range_exts->intra_smoothing_disabled_flag ||
	    range_exts->persistent_rice_adaptation_enabled_flag ||
	    range_exts->cabac_bypass_alignment_enabled_flag)
		return 1;
	/*
	 *  Note: high_precision_offsets_enabled_flag is supported even
	 * if hw capabilities (bHevcRangeExt is not set)
	 */
	return 0;
}

static enum bspp_error_type bspp_hevc_parsesps(void *sr_ctx,
					       void *str_res,
					       struct bspp_hevc_sps *sps)
{
	enum bspp_error_type parse_err = BSPP_ERROR_NONE;
	unsigned char i;
	unsigned int min_cblog2_size_y;

	if (!sr_ctx || !sps) {
		VDEC_ASSERT(0);
		return BSPP_ERROR_INVALID_VALUE;
	}

	memset(sps, 0, sizeof(struct bspp_hevc_sps));

	HEVC_SWSR_UN("sps_video_parameter_set_id",
		     (unsigned int *)&sps->sps_video_parameter_set_id, 4, sr_ctx);
	HEVC_SWSR_UN("sps_max_sub_layers_minus1",
		     (unsigned int *)&sps->sps_max_sub_layers_minus1, 3, sr_ctx);
	HEVC_RANGEUCHECK("sps_max_sub_layers_minus1", sps->sps_max_sub_layers_minus1, 0,
			 HEVC_MAX_NUM_SUBLAYERS - 1, &parse_err);
	HEVC_SWSR_U1("sps_temporal_id_nesting_flag",
		     &sps->sps_temporal_id_nesting_flag, sr_ctx);

	if (sps->sps_max_sub_layers_minus1 == 0)
		HEVC_UCHECK("sps_temporal_id_nesting_flag",
			    sps->sps_temporal_id_nesting_flag, 1, &parse_err);

	parse_err |= bspp_hevc_parse_profiletierlevel
				(sr_ctx, &sps->profile_tier_level,
				 sps->sps_max_sub_layers_minus1);

	HEVC_SWSR_UE("sps_seq_parameter_set_id",
		     (unsigned int *)&sps->sps_seq_parameter_set_id, sr_ctx);
	HEVC_RANGEUCHECK("sps_seq_parameter_set_id", sps->sps_seq_parameter_set_id, 0,
			 HEVC_MAX_SPS_COUNT - 1, &parse_err);

	HEVC_SWSR_UE("chroma_format_idc", (unsigned int *)&sps->chroma_format_idc, sr_ctx);
	HEVC_RANGEUCHECK("chroma_format_idc", sps->chroma_format_idc, 0, 3, &parse_err);

	if (sps->chroma_format_idc == 3)
		HEVC_SWSR_U1("separate_colour_plane_flag",
			     &sps->separate_colour_plane_flag, sr_ctx);

	HEVC_SWSR_UE("pic_width_in_luma_samples",
		     (unsigned int *)&sps->pic_width_in_luma_samples, sr_ctx);
	HEVC_SWSR_UE("pic_height_in_luma_samples",
		     (unsigned int *)&sps->pic_height_in_luma_samples, sr_ctx);

	HEVC_SWSR_U1("conformance_window_flag", &sps->conformance_window_flag, sr_ctx);

	if (sps->pic_width_in_luma_samples == 0 ||
	    sps->pic_height_in_luma_samples == 0) {
		pr_warn("Invalid video dimensions (%u, %u)",
			sps->pic_width_in_luma_samples,
			sps->pic_height_in_luma_samples);
		parse_err |= BSPP_ERROR_UNRECOVERABLE;
	}

	if (sps->conformance_window_flag) {
		HEVC_SWSR_UE("conf_win_left_offset",
			     (unsigned int *)&sps->conf_win_left_offset, sr_ctx);
		HEVC_SWSR_UE("conf_win_right_offset",
			     (unsigned int *)&sps->conf_win_right_offset, sr_ctx);
		HEVC_SWSR_UE("conf_win_top_offset",
			     (unsigned int *)&sps->conf_win_top_offset, sr_ctx);
		HEVC_SWSR_UE("conf_win_bottom_offset",
			     (unsigned int *)&sps->conf_win_bottom_offset, sr_ctx);
	}

	HEVC_SWSR_UE("bit_depth_luma_minus8",
		     (unsigned int *)&sps->bit_depth_luma_minus8, sr_ctx);
	HEVC_RANGEUCHECK("bit_depth_luma_minus8",
			 sps->bit_depth_luma_minus8, 0, 6, &parse_err);
	HEVC_SWSR_UE("bit_depth_chroma_minus8",
		     (unsigned int *)&sps->bit_depth_chroma_minus8, sr_ctx);
	HEVC_RANGEUCHECK("bit_depth_chroma_minus8", sps->bit_depth_chroma_minus8,
			 0, 6, &parse_err);

	HEVC_SWSR_UE("log2_max_pic_order_cnt_lsb_minus4",
		     (unsigned int *)&sps->log2_max_pic_order_cnt_lsb_minus4, sr_ctx);
	HEVC_RANGEUCHECK("log2_max_pic_order_cnt_lsb_minus4",
			 sps->log2_max_pic_order_cnt_lsb_minus4,
			 0, 12, &parse_err);

	HEVC_SWSR_U1("sps_sub_layer_ordering_info_present_flag",
		     &sps->sps_sub_layer_ordering_info_present_flag, sr_ctx);
	for (i = (sps->sps_sub_layer_ordering_info_present_flag ?
		0 : sps->sps_max_sub_layers_minus1);
		i <= sps->sps_max_sub_layers_minus1; ++i) {
		HEVC_SWSR_UE("sps_max_dec_pic_buffering_minus1",
			     (unsigned int *)&sps->sps_max_dec_pic_buffering_minus1[i], sr_ctx);
		HEVC_SWSR_UE("sps_max_num_reorder_pics",
			     (unsigned int *)&sps->sps_max_num_reorder_pics[i], sr_ctx);
		HEVC_SWSR_UE("sps_max_latency_increase_plus1",
			     (unsigned int *)&sps->sps_max_latency_increase_plus1[i], sr_ctx);
	}

	HEVC_SWSR_UE("log2_min_luma_coding_block_size_minus3",
		     (unsigned int *)&sps->log2_min_luma_coding_block_size_minus3, sr_ctx);
	HEVC_SWSR_UE("log2_diff_max_min_luma_coding_block_size",
		     (unsigned int *)&sps->log2_diff_max_min_luma_coding_block_size, sr_ctx);
	HEVC_SWSR_UE("log2_min_transform_block_size_minus2",
		     (unsigned int *)&sps->log2_min_transform_block_size_minus2, sr_ctx);
	HEVC_SWSR_UE("log2_diff_max_min_transform_block_size",
		     (unsigned int *)&sps->log2_diff_max_min_transform_block_size, sr_ctx);
	HEVC_SWSR_UE("max_transform_hierarchy_depth_inter",
		     (unsigned int *)&sps->max_transform_hierarchy_depth_inter, sr_ctx);
	HEVC_SWSR_UE("max_transform_hierarchy_depth_intra",
		     (unsigned int *)&sps->max_transform_hierarchy_depth_intra, sr_ctx);

	HEVC_SWSR_U1("scaling_list_enabled_flag", &sps->scaling_list_enabled_flag, sr_ctx);

	if (sps->scaling_list_enabled_flag) {
		HEVC_SWSR_U1("sps_scaling_list_data_present_flag",
			     &sps->sps_scaling_list_data_present_flag, sr_ctx);
		if (sps->sps_scaling_list_data_present_flag)
			parse_err |= bspp_hevc_parse_scalinglistdata(sr_ctx,
								     &sps->scalinglist_data);
		else
			bspp_hevc_usedefault_scalinglists(&sps->scalinglist_data);
	}

	HEVC_SWSR_U1("amp_enabled_flag", &sps->amp_enabled_flag, sr_ctx);
	HEVC_SWSR_U1("sample_adaptive_offset_enabled_flag",
		     &sps->sample_adaptive_offset_enabled_flag, sr_ctx);
	HEVC_SWSR_U1("pcm_enabled_flag", &sps->pcm_enabled_flag, sr_ctx);

	if (sps->pcm_enabled_flag) {
		HEVC_SWSR_UN("pcm_sample_bit_depth_luma_minus1",
			     (unsigned int *)&sps->pcm_sample_bit_depth_luma_minus1,
			     4, sr_ctx);
		HEVC_SWSR_UN("pcm_sample_bit_depth_chroma_minus1",
			     (unsigned int *)&sps->pcm_sample_bit_depth_chroma_minus1,
			     4, sr_ctx);
		HEVC_SWSR_UE("log2_min_pcm_luma_coding_block_size_minus3",
			     (unsigned int *)&sps->log2_min_pcm_luma_coding_block_size_minus3,
			     sr_ctx);
		HEVC_SWSR_UE("log2_diff_max_min_pcm_luma_coding_block_size",
			     (unsigned int *)&sps->log2_diff_max_min_pcm_luma_coding_block_size,
			     sr_ctx);
		HEVC_SWSR_U1("pcm_loop_filter_disabled_flag",
			     &sps->pcm_loop_filter_disabled_flag, sr_ctx);
	} else {
		sps->pcm_sample_bit_depth_luma_minus1 = 7;
		sps->pcm_sample_bit_depth_chroma_minus1 = 7;
		sps->log2_min_pcm_luma_coding_block_size_minus3 = 0;
		sps->log2_diff_max_min_pcm_luma_coding_block_size = 2;
	}

	HEVC_SWSR_UE("num_short_term_ref_pic_sets",
		     (unsigned int *)&sps->num_short_term_ref_pic_sets, sr_ctx);
	HEVC_RANGEUCHECK("num_short_term_ref_pic_sets", sps->num_short_term_ref_pic_sets, 0,
			 HEVC_MAX_NUM_ST_REF_PIC_SETS - 1, &parse_err);

	for (i = 0; i < sps->num_short_term_ref_pic_sets; ++i) {
		parse_err |= bspp_hevc_parse_shortterm_refpicset(sr_ctx,
				sps->rps_list,
				i,
				0);
	}

	HEVC_SWSR_U1("long_term_ref_pics_present_flag",
		     &sps->long_term_ref_pics_present_flag, sr_ctx);
	if (sps->long_term_ref_pics_present_flag) {
		HEVC_SWSR_UE("num_long_term_ref_pics_sps",
			     (unsigned int *)&sps->num_long_term_ref_pics_sps, sr_ctx);
		HEVC_RANGEUCHECK("num_long_term_ref_pics_sps",
				 sps->num_long_term_ref_pics_sps, 0,
				 HEVC_MAX_NUM_LT_REF_PICS, &parse_err);
		for (i = 0; i < sps->num_long_term_ref_pics_sps; ++i) {
			HEVC_SWSR_UN("lt_ref_pic_poc_lsb_sps",
				     (unsigned int *)&sps->lt_ref_pic_poc_lsb_sps[i],
				     sps->log2_max_pic_order_cnt_lsb_minus4 + 4,
				     sr_ctx);
			HEVC_SWSR_U1("used_by_curr_pic_lt_sps_flag",
				     &sps->used_by_curr_pic_lt_sps_flag[i],
				     sr_ctx);
		}
	}

	HEVC_SWSR_U1("sps_temporal_mvp_enabled_flag", &sps->sps_temporal_mvp_enabled_flag, sr_ctx);
	HEVC_SWSR_U1("strong_intra_smoothing_enabled_flag",
		     &sps->strong_intra_smoothing_enabled_flag, sr_ctx);
	HEVC_SWSR_U1("vui_parameters_present_flag", &sps->vui_parameters_present_flag, sr_ctx);

	if (sps->vui_parameters_present_flag)
		bspp_hevc_parsevui_parameters(sr_ctx, &sps->vui_params,
					      sps->sps_max_sub_layers_minus1);

	HEVC_SWSR_U1("sps_extension_present_flag", &sps->sps_extension_present_flag, sr_ctx);
	if (sps->sps_extension_present_flag &&
	    bspp_hevc_range_extensions_is_enabled(&sps->profile_tier_level)) {
		HEVC_SWSR_U1("sps_range_extensions_flag", &sps->sps_range_extensions_flag, sr_ctx);

		HEVC_SWSR_UN("sps_extension_7bits", (unsigned int *)&sps->sps_extension_7bits, 7,
			     sr_ctx);
		/*
		 *  ignore extension data. Although we inform
		 * if some non-zero data was found
		 */
		HEVC_UCHECK("sps_extension_7bits", sps->sps_extension_7bits, 0, &parse_err);
		/*
		 * TODO ?: the newest HEVC spec (10/2014) splits
		 * "sps_extension_7bits" to * sps_multilayer_extension_flag (1)
		 * sps_extension_6bits (6)
		 */
		if (sps->sps_range_extensions_flag)
			parse_err |= bspp_hevc_parse_spsrange_extensions
						(sr_ctx, &sps->range_exts);
	}
	/*
	 * calculate "derived" variables needed further in the parsing process
	 * (of other headers) and save them for later use
	 */
	sps->sub_width_c = 1;
	sps->sub_height_c = 1;
	if (sps->chroma_format_idc == 2) {
		sps->sub_width_c = 2;
	} else if (sps->chroma_format_idc == 1) {
		sps->sub_width_c = 2;
		sps->sub_height_c = 2;
	}

	min_cblog2_size_y = sps->log2_min_luma_coding_block_size_minus3 + 3;
	sps->ctb_log2size_y =
		min_cblog2_size_y + sps->log2_diff_max_min_luma_coding_block_size;
	sps->ctb_size_y = 1 << sps->ctb_log2size_y;

	if (sps->ctb_size_y > 0) {
		/* use integer division with rounding up */
		sps->pic_width_in_ctbs_y =
			(sps->pic_width_in_luma_samples + sps->ctb_size_y - 1)
			/ sps->ctb_size_y;
		sps->pic_height_in_ctbs_y =
			(sps->pic_height_in_luma_samples + sps->ctb_size_y - 1)
			/ sps->ctb_size_y;
	} else {
		parse_err |= BSPP_ERROR_INVALID_VALUE;
	}

	sps->pic_size_in_ctbs_y =
		sps->pic_width_in_ctbs_y * sps->pic_height_in_ctbs_y;

	sps->max_pic_order_cnt_lsb =
		1 << (sps->log2_max_pic_order_cnt_lsb_minus4 + 4);

	for (i = 0; i <= sps->sps_max_sub_layers_minus1; ++i) {
		sps->sps_max_latency_pictures[i] =
			sps->sps_max_num_reorder_pics[i] +
			sps->sps_max_latency_increase_plus1[i] - 1;
	}

	BSPP_HEVC_SYNTAX("ctb_size_y: %u", sps->ctb_size_y);
	BSPP_HEVC_SYNTAX("pic_width_in_ctbs_y: %u", sps->pic_width_in_ctbs_y);
	BSPP_HEVC_SYNTAX("pic_height_in_ctbs_y: %u", sps->pic_height_in_ctbs_y);
	BSPP_HEVC_SYNTAX("pic_size_in_ctbs_y: %u", sps->pic_size_in_ctbs_y);

	return parse_err;
}

static int bspp_hevc_release_sequhdrinfo(void *str_alloc, void *secure_spsinfo)
{
	struct bspp_hevc_sps *hevc_sps = (struct bspp_hevc_sps *)secure_spsinfo;

	if (!hevc_sps)
		return IMG_ERROR_INVALID_PARAMETERS;

	/* Release the raw VIU data. */
	bspp_streamrelese_rawbstrdataplain(str_alloc, (void *)hevc_sps->vui_raw_data);
	return 0;
}

static int bspp_hevc_releasedata(void *str_alloc, enum bspp_unit_type data_type,
				 void *data_handle)
{
	int result = 0;

	if (!data_handle)
		return IMG_ERROR_INVALID_PARAMETERS;

	switch (data_type) {
	case BSPP_UNIT_SEQUENCE:
		result = bspp_hevc_release_sequhdrinfo(str_alloc, data_handle);
		break;
	default:
		break;
	}

	return result;
}

static int bspp_hevc_reset_ppsinfo(void *secure_ppsinfo)
{
	struct bspp_hevc_pps *hevc_pps = NULL;

	if (!secure_ppsinfo)
		return IMG_ERROR_INVALID_PARAMETERS;

	hevc_pps = (struct bspp_hevc_pps *)secure_ppsinfo;

	memset(hevc_pps, 0, sizeof(*hevc_pps));

	return 0;
}

static int bspp_hevc_resetdata(enum bspp_unit_type data_type, void *data_handle)
{
	int result = 0;

	switch (data_type) {
	case BSPP_UNIT_PPS:
		result = bspp_hevc_reset_ppsinfo(data_handle);
		break;
	default:
		break;
	}
	return result;
}

static enum bspp_error_type bspp_hevc_parsepps_range_extensions
			(void *sr_ctx,
			 struct bspp_hevc_pps_range_exts *range_exts,
			 unsigned char transform_skip_enabled_flag,
			 unsigned char log2_diff_max_min_luma_coding_block_size)
{
	enum bspp_error_type parse_err = BSPP_ERROR_NONE;

	VDEC_ASSERT(sr_ctx);
	VDEC_ASSERT(range_exts);

	memset(range_exts, 0, sizeof(struct bspp_hevc_pps_range_exts));

	if (transform_skip_enabled_flag)
		HEVC_SWSR_UE("log2_max_transform_skip_block_size_minus2",
			     (unsigned int *)&range_exts->log2_max_transform_skip_block_size_minus2,
			     sr_ctx);

	HEVC_SWSR_U1("cross_component_prediction_enabled_flag",
		     &range_exts->cross_component_prediction_enabled_flag,
		     sr_ctx);
	HEVC_UCHECK("cross_component_prediction_enabled_flag",
		    range_exts->cross_component_prediction_enabled_flag, 0,
		    &parse_err);

	HEVC_SWSR_U1("chroma_qp_offset_list_enabled_flag",
		     &range_exts->chroma_qp_offset_list_enabled_flag, sr_ctx);

	if (range_exts->chroma_qp_offset_list_enabled_flag) {
		unsigned char i;

		HEVC_SWSR_UE("diff_cu_chroma_qp_offset_depth",
			     (unsigned int *)&range_exts->diff_cu_chroma_qp_offset_depth,
			     sr_ctx);
		HEVC_RANGEUCHECK("diff_cu_chroma_qp_offset_depth",
				 range_exts->diff_cu_chroma_qp_offset_depth, 0,
				 log2_diff_max_min_luma_coding_block_size,
				 &parse_err);

		HEVC_SWSR_UE("chroma_qp_offset_list_len_minus1",
			     (unsigned int *)&range_exts->chroma_qp_offset_list_len_minus1,
			     sr_ctx);
		HEVC_RANGEUCHECK("chroma_qp_offset_list_len_minus1",
				 range_exts->chroma_qp_offset_list_len_minus1,
				 0, HEVC_MAX_CHROMA_QP - 1, &parse_err);
		for (i = 0; i <= range_exts->chroma_qp_offset_list_len_minus1; i++) {
			HEVC_SWSR_SE("cb_qp_offset_list",
				     (int *)&range_exts->cb_qp_offset_list[i], sr_ctx);
			HEVC_RANGESCHECK("cb_qp_offset_list", range_exts->cb_qp_offset_list[i],
					 -12, 12, &parse_err);
			HEVC_SWSR_SE("cr_qp_offset_list",
				     (int *)&range_exts->cr_qp_offset_list[i], sr_ctx);
			HEVC_RANGESCHECK("cr_qp_offset_list", range_exts->cr_qp_offset_list[i],
					 -12, 12, &parse_err);
		}
	}
	HEVC_SWSR_UE("log2_sao_offset_scale_luma",
		     (unsigned int *)&range_exts->log2_sao_offset_scale_luma, sr_ctx);
	HEVC_UCHECK("log2_sao_offset_scale_luma",
		    range_exts->log2_sao_offset_scale_luma, 0, &parse_err);
	HEVC_SWSR_UE("log2_sao_offset_scale_chroma",
		     (unsigned int *)&range_exts->log2_sao_offset_scale_chroma, sr_ctx);
	HEVC_UCHECK("log2_sao_offset_scale_chroma",
		    range_exts->log2_sao_offset_scale_chroma, 0, &parse_err);

	return parse_err;
}

static unsigned char bspp_hevc_checkppsrangeextensions
				(struct bspp_hevc_pps_range_exts *range_exts)
{
	VDEC_ASSERT(range_exts);

	if (range_exts->log2_max_transform_skip_block_size_minus2 ||
	    range_exts->cross_component_prediction_enabled_flag)
		return 1;
	/*
	 * Note: chroma_qp_offset_list_enabled_flag is supported even
	 * if hw capabilities (bHevcRangeExt is not set)
	 */
	return 0;
}

static enum bspp_error_type bspp_hevc_parsepps
			(void *sr_ctx, void *str_res,
			 struct bspp_hevc_pps *pps)
{
	enum bspp_error_type parse_err = BSPP_ERROR_NONE;
	struct bspp_sequence_hdr_info *spsinfo = NULL;
	struct bspp_hevc_sps *sps = NULL;

	VDEC_ASSERT(sr_ctx);
	VDEC_ASSERT(pps);
	memset(pps, 0, sizeof(struct bspp_hevc_pps));

	HEVC_SWSR_UE("pps_pic_parameter_set_id",
		     (unsigned int *)&pps->pps_pic_parameter_set_id, sr_ctx);
	HEVC_RANGEUCHECK("pps_pic_parameter_set_id", pps->pps_pic_parameter_set_id, 0,
			 HEVC_MAX_PPS_COUNT - 1, &parse_err);
	HEVC_SWSR_UE("pps_seq_parameter_set_id",
		     (unsigned int *)&pps->pps_seq_parameter_set_id, sr_ctx);
	HEVC_RANGEUCHECK("pps_seq_parameter_set_id", pps->pps_seq_parameter_set_id, 0,
			 HEVC_MAX_SPS_COUNT - 1, &parse_err);

	spsinfo = bspp_get_sequ_hdr(str_res, pps->pps_seq_parameter_set_id);
	if (!spsinfo) {
		parse_err |= BSPP_ERROR_NO_SEQUENCE_HDR;
	} else {
		sps = (struct bspp_hevc_sps *)spsinfo->secure_sequence_info;
		VDEC_ASSERT(sps->sps_seq_parameter_set_id ==
			pps->pps_seq_parameter_set_id);
	}

	HEVC_SWSR_U1("dependent_slice_segments_enabled_flag",
		     &pps->dependent_slice_segments_enabled_flag, sr_ctx);
	HEVC_SWSR_U1("output_flag_present_flag",
		     &pps->output_flag_present_flag, sr_ctx);
	HEVC_SWSR_UN("num_extra_slice_header_bits",
		     (unsigned int *)&pps->num_extra_slice_header_bits, 3, sr_ctx);
	HEVC_SWSR_U1("sign_data_hiding_enabled_flag", &pps->sign_data_hiding_enabled_flag, sr_ctx);
	HEVC_SWSR_U1("cabac_init_present_flag", &pps->cabac_init_present_flag, sr_ctx);
	HEVC_SWSR_UE("num_ref_idx_l0_default_active_minus1",
		     (unsigned int *)&pps->num_ref_idx_l0_default_active_minus1, sr_ctx);
	HEVC_RANGEUCHECK("num_ref_idx_l0_default_active_minus1",
			 pps->num_ref_idx_l0_default_active_minus1, 0, 14, &parse_err);
	HEVC_SWSR_UE("num_ref_idx_l1_default_active_minus1",
		     (unsigned int *)&pps->num_ref_idx_l1_default_active_minus1, sr_ctx);
	HEVC_RANGEUCHECK("num_ref_idx_l1_default_active_minus1",
			 pps->num_ref_idx_l1_default_active_minus1, 0, 14, &parse_err);
	HEVC_SWSR_SE("init_qp_minus26", (int *)&pps->init_qp_minus26, sr_ctx);

	if (sps)
		HEVC_RANGESCHECK("init_qp_minus26", pps->init_qp_minus26,
				 -(26 + (6 * sps->bit_depth_luma_minus8)), 25, &parse_err);

	HEVC_SWSR_U1("constrained_intra_pred_flag", &pps->constrained_intra_pred_flag, sr_ctx);
	HEVC_SWSR_U1("transform_skip_enabled_flag", &pps->transform_skip_enabled_flag, sr_ctx);

	HEVC_SWSR_U1("cu_qp_delta_enabled_flag", &pps->cu_qp_delta_enabled_flag, sr_ctx);

	if (pps->cu_qp_delta_enabled_flag)
		HEVC_SWSR_UE("diff_cu_qp_delta_depth",
			     (unsigned int *)&pps->diff_cu_qp_delta_depth, sr_ctx);

	HEVC_SWSR_SE("pps_cb_qp_offset", (int *)&pps->pps_cb_qp_offset, sr_ctx);
	HEVC_RANGESCHECK("pps_cb_qp_offset", pps->pps_cb_qp_offset, -12, 12, &parse_err);
	HEVC_SWSR_SE("pps_cr_qp_offset", (int *)&pps->pps_cr_qp_offset, sr_ctx);
	HEVC_RANGESCHECK("pps_cr_qp_offset", pps->pps_cr_qp_offset, -12, 12, &parse_err);
	HEVC_SWSR_U1("pps_slice_chroma_qp_offsets_present_flag",
		     &pps->pps_slice_chroma_qp_offsets_present_flag, sr_ctx);
	HEVC_SWSR_U1("weighted_pred_flag", &pps->weighted_pred_flag, sr_ctx);
	HEVC_SWSR_U1("weighted_bipred_flag", &pps->weighted_bipred_flag, sr_ctx);
	HEVC_SWSR_U1("transquant_bypass_enabled_flag",
		     &pps->transquant_bypass_enabled_flag, sr_ctx);
	HEVC_SWSR_U1("tiles_enabled_flag", &pps->tiles_enabled_flag, sr_ctx);
	HEVC_SWSR_U1("entropy_coding_sync_enabled_flag",
		     &pps->entropy_coding_sync_enabled_flag, sr_ctx);

	if (pps->tiles_enabled_flag) {
		HEVC_SWSR_UE("num_tile_columns_minus1",
			     (unsigned int *)&pps->num_tile_columns_minus1, sr_ctx);
		HEVC_RANGEUCHECK("num_tile_columns_minus1", pps->num_tile_columns_minus1, 0,
				 HEVC_MAX_TILE_COLS - 1, &parse_err);

		if (pps->num_tile_columns_minus1 > HEVC_MAX_TILE_COLS)
			pps->num_tile_columns_minus1 = HEVC_MAX_TILE_COLS;

		HEVC_SWSR_UE("num_tile_rows_minus1", (unsigned int *)&pps->num_tile_rows_minus1,
			     sr_ctx);
		HEVC_RANGEUCHECK("num_tile_rows_minus1", pps->num_tile_rows_minus1, 0,
				 HEVC_MAX_TILE_ROWS - 1, &parse_err);

		if (pps->num_tile_rows_minus1 > HEVC_MAX_TILE_ROWS)
			pps->num_tile_rows_minus1 = HEVC_MAX_TILE_ROWS;

		HEVC_SWSR_U1("uniform_spacing_flag", &pps->uniform_spacing_flag, sr_ctx);

		if (!pps->uniform_spacing_flag) {
			unsigned char i = 0;

			for (i = 0; i < pps->num_tile_columns_minus1; ++i)
				HEVC_SWSR_UE("column_width_minus1",
					     (unsigned int *)&pps->column_width_minus1[i],
					     sr_ctx);

			for (i = 0; i < pps->num_tile_rows_minus1; ++i)
				HEVC_SWSR_UE("row_height_minus1",
					     (unsigned int *)&pps->row_height_minus1[i],
					     sr_ctx);
		}
		HEVC_SWSR_U1("loop_filter_across_tiles_enabled_flag",
			     &pps->loop_filter_across_tiles_enabled_flag, sr_ctx);
	} else {
		pps->loop_filter_across_tiles_enabled_flag = 1;
	}

	HEVC_SWSR_U1("pps_loop_filter_across_slices_enabled_flag",
		     &pps->pps_loop_filter_across_slices_enabled_flag, sr_ctx);

	HEVC_SWSR_U1("deblocking_filter_control_present_flag",
		     &pps->deblocking_filter_control_present_flag, sr_ctx);

	if (pps->deblocking_filter_control_present_flag) {
		HEVC_SWSR_U1("deblocking_filter_override_enabled_flag",
			     &pps->deblocking_filter_override_enabled_flag, sr_ctx);
		HEVC_SWSR_U1("pps_deblocking_filter_disabled_flag",
			     &pps->pps_deblocking_filter_disabled_flag, sr_ctx);
		if (!pps->pps_deblocking_filter_disabled_flag) {
			HEVC_SWSR_SE("pps_beta_offset_div2", (int *)&pps->pps_beta_offset_div2,
				     sr_ctx);
			HEVC_RANGESCHECK("pps_beta_offset_div2", pps->pps_beta_offset_div2, -6, 6,
					 &parse_err);
			HEVC_SWSR_SE("pps_tc_offset_div2", (int *)&pps->pps_tc_offset_div2, sr_ctx);
			HEVC_RANGESCHECK("pps_tc_offset_div2", pps->pps_tc_offset_div2, -6, 6,
					 &parse_err);
		}
	}

	HEVC_SWSR_U1("pps_scaling_list_data_present_flag",
		     &pps->pps_scaling_list_data_present_flag, sr_ctx);
	if (pps->pps_scaling_list_data_present_flag)
		parse_err |= bspp_hevc_parse_scalinglistdata(sr_ctx, &pps->scaling_list);

	HEVC_SWSR_U1("lists_modification_present_flag",
		     &pps->lists_modification_present_flag, sr_ctx);
	HEVC_SWSR_UE("log2_parallel_merge_level_minus2",
		     (unsigned int *)&pps->log2_parallel_merge_level_minus2, sr_ctx);
	HEVC_SWSR_U1("slice_segment_header_extension_present_flag",
		     &pps->slice_segment_header_extension_present_flag, sr_ctx);

	HEVC_SWSR_U1("pps_extension_present_flag", &pps->pps_extension_present_flag, sr_ctx);
	if (pps->pps_extension_present_flag &&
	    bspp_hevc_range_extensions_is_enabled(&sps->profile_tier_level)) {
		HEVC_SWSR_U1("pps_range_extensions_flag",
			     &pps->pps_range_extensions_flag, sr_ctx);
		HEVC_SWSR_UN("pps_extension_7bits",
			     (unsigned int *)&pps->pps_extension_7bits, 7, sr_ctx);
		/*
		 * ignore extension data. Although we inform
		 * if some non-zero data was found
		 */
		HEVC_UCHECK("pps_extension_7bits", pps->pps_extension_7bits, 0, &parse_err);

		/*
		 * TODO ?: the newest HEVC spec (10/2014) splits "pps_extension_7bits" to
		 * pps_multilayer_extension_flag (1)
		 * pps_extension_6bits (6)
		 */
		if (pps->pps_range_extensions_flag && sps) {
			parse_err |= bspp_hevc_parsepps_range_extensions
					(sr_ctx,
					 &pps->range_exts,
					 pps->transform_skip_enabled_flag,
					 sps->log2_diff_max_min_luma_coding_block_size);
		}
	}

	/* calculate derived elements */
	if (pps->tiles_enabled_flag && sps)
		bspp_hevc_dotilecalculations(sps, pps);

	return parse_err;
}

static void bspp_hevc_dotilecalculations(struct bspp_hevc_sps *sps,
					 struct bspp_hevc_pps *pps)
{
	unsigned short colwidth[HEVC_MAX_TILE_COLS];
	unsigned short rowheight[HEVC_MAX_TILE_ROWS];
	unsigned char i;

	if (!pps->tiles_enabled_flag) {
		pps->max_tile_height_in_ctbs_y = sps->pic_height_in_ctbs_y;
		return;
	}

	if (pps->uniform_spacing_flag) {
		for (i = 0; i <= pps->num_tile_columns_minus1; ++i) {
			colwidth[i] = ((i + 1) * sps->pic_width_in_ctbs_y) /
				(pps->num_tile_columns_minus1 + 1) -
				(i * sps->pic_width_in_ctbs_y) /
				(pps->num_tile_columns_minus1 + 1);
		}

		for (i = 0; i <= pps->num_tile_rows_minus1; ++i) {
			rowheight[i] = ((i + 1) * sps->pic_height_in_ctbs_y) /
				(pps->num_tile_rows_minus1 + 1) -
				(i * sps->pic_height_in_ctbs_y) /
				(pps->num_tile_rows_minus1 + 1);
		}

		pps->max_tile_height_in_ctbs_y = rowheight[0];
	} else {
		pps->max_tile_height_in_ctbs_y = 0;

		colwidth[pps->num_tile_columns_minus1] = sps->pic_width_in_ctbs_y;
		for (i = 0; i <= pps->num_tile_columns_minus1; ++i) {
			colwidth[i] = pps->column_width_minus1[i] + 1;
			colwidth[pps->num_tile_columns_minus1] -= colwidth[i];
		}

		rowheight[pps->num_tile_rows_minus1] = sps->pic_height_in_ctbs_y;
		for (i = 0; i <= pps->num_tile_rows_minus1; ++i) {
			rowheight[i] = pps->row_height_minus1[i] + 1;
			rowheight[pps->num_tile_rows_minus1] -= rowheight[i];

			if (rowheight[i] > pps->max_tile_height_in_ctbs_y)
				pps->max_tile_height_in_ctbs_y = rowheight[i];
		}

		if (rowheight[pps->num_tile_rows_minus1] > pps->max_tile_height_in_ctbs_y)
			pps->max_tile_height_in_ctbs_y =
					rowheight[pps->num_tile_rows_minus1];
	}

	for (i = 0; i <= pps->num_tile_columns_minus1; ++i)
		pps->col_bd[i + 1] = pps->col_bd[i] + colwidth[i];

	for (i = 0; i <= pps->num_tile_rows_minus1; ++i)
		pps->row_bd[i + 1] = pps->row_bd[i] + rowheight[i];
}

static enum bspp_error_type bspp_hevc_parse_slicesegmentheader
		(void *sr_ctx, void *str_res,
		 struct bspp_hevc_slice_segment_header *ssh,
		 unsigned char nalunit_type,
		 struct bspp_vps_info **vpsinfo,
		 struct bspp_sequence_hdr_info **spsinfo,
		 struct bspp_pps_info **ppsinfo)
{
	enum bspp_error_type parse_err = BSPP_ERROR_NONE;
	struct bspp_hevc_pps *pps = NULL;
	struct bspp_hevc_sps *sps = NULL;
	struct bspp_hevc_vps *vps = NULL;

	VDEC_ASSERT(sr_ctx);
	VDEC_ASSERT(ssh);
	VDEC_ASSERT(vpsinfo);
	VDEC_ASSERT(spsinfo);
	VDEC_ASSERT(ppsinfo);

	memset(ssh, 0, sizeof(struct bspp_hevc_slice_segment_header));

	HEVC_SWSR_U1("first_slice_segment_in_pic_flag",
		     &ssh->first_slice_segment_in_pic_flag, sr_ctx);

	if (bspp_hevc_picture_is_irap((enum hevc_nalunittype)nalunit_type))
		HEVC_SWSR_U1("no_output_of_prior_pics_flag",
			     &ssh->no_output_of_prior_pics_flag, sr_ctx);

	HEVC_SWSR_UE("slice_pic_parameter_set_id", (unsigned int *)&ssh->slice_pic_parameter_set_id,
		     sr_ctx);
	HEVC_RANGEUCHECK("slice_pic_parameter_set_id", ssh->slice_pic_parameter_set_id, 0,
			 HEVC_MAX_PPS_COUNT - 1, &parse_err);

	if (ssh->slice_pic_parameter_set_id >= HEVC_MAX_PPS_COUNT) {
		pr_warn("PPS Id invalid (%u), setting to 0",
			ssh->slice_pic_parameter_set_id);
		ssh->slice_pic_parameter_set_id = 0;
		parse_err &= ~BSPP_ERROR_INVALID_VALUE;
		parse_err |= BSPP_ERROR_CORRECTION_VALIDVALUE;
	}

	/* set PPS */
	*ppsinfo = bspp_get_pps_hdr(str_res, ssh->slice_pic_parameter_set_id);
	if (!(*ppsinfo)) {
		parse_err |= BSPP_ERROR_NO_PPS;
		goto error;
	}
	pps = (struct bspp_hevc_pps *)(*ppsinfo)->secure_pps_info;
	if (!pps) {
		parse_err |= BSPP_ERROR_NO_PPS;
		goto error;
	}
	VDEC_ASSERT(pps->pps_pic_parameter_set_id == ssh->slice_pic_parameter_set_id);

	*spsinfo = bspp_get_sequ_hdr(str_res, pps->pps_seq_parameter_set_id);
	if (!(*spsinfo)) {
		parse_err |= BSPP_ERROR_NO_SEQUENCE_HDR;
		goto error;
	}
	sps = (struct bspp_hevc_sps *)(*spsinfo)->secure_sequence_info;
	VDEC_ASSERT(sps->sps_seq_parameter_set_id == pps->pps_seq_parameter_set_id);

	*vpsinfo = bspp_get_vpshdr(str_res, sps->sps_video_parameter_set_id);
	if (!(*vpsinfo)) {
		parse_err |= BSPP_ERROR_NO_VPS;
		goto error;
	}
	vps = (struct bspp_hevc_vps *)(*vpsinfo)->secure_vpsinfo;
	VDEC_ASSERT(vps->vps_video_parameter_set_id == sps->sps_video_parameter_set_id);

	if (!ssh->first_slice_segment_in_pic_flag) {
		if (pps->dependent_slice_segments_enabled_flag)
			HEVC_SWSR_U1("dependent_slice_segment_flag",
				     &ssh->dependent_slice_segment_flag, sr_ctx);

		HEVC_SWSR_UN("slice_segment_address",
			     (unsigned int *)&ssh->slice_segment_address,
			     bspp_ceil_log2(sps->pic_size_in_ctbs_y), sr_ctx);
	}

error:
	return parse_err;
}

static enum bspp_error_type bspp_hevc_parse_profiletierlevel
		(void *sr_ctx,
		 struct bspp_hevc_profile_tierlevel *ptl,
		 unsigned char vps_maxsublayers_minus1)
{
	enum bspp_error_type parse_err = BSPP_ERROR_NONE;
	unsigned char i, j;
	unsigned int res = 0;

	VDEC_ASSERT(sr_ctx);
	VDEC_ASSERT(ptl);
	VDEC_ASSERT(vps_maxsublayers_minus1 < HEVC_MAX_NUM_SUBLAYERS);

	memset(ptl, 0, sizeof(struct bspp_hevc_profile_tierlevel));

	HEVC_SWSR_UN("general_profile_space", (unsigned int *)&ptl->general_profile_space, 2,
		     sr_ctx);
	HEVC_SWSR_U1("general_tier_flag", &ptl->general_tier_flag, sr_ctx);
	HEVC_SWSR_UN("general_profile_idc", (unsigned int *)&ptl->general_profile_idc, 5, sr_ctx);

	for (j = 0; j < HEVC_MAX_NUM_PROFILE_IDC; ++j) {
		HEVC_SWSR_U1("general_profile_compatibility_flag",
			     &ptl->general_profile_compatibility_flag[j],
			     sr_ctx);
	}

	HEVC_SWSR_U1("general_progressive_source_flag",
		     &ptl->general_progressive_source_flag, sr_ctx);
	HEVC_SWSR_U1("general_interlaced_source_flag",
		     &ptl->general_interlaced_source_flag, sr_ctx);
	HEVC_SWSR_U1("general_non_packed_constraint_flag",
		     &ptl->general_non_packed_constraint_flag, sr_ctx);
	HEVC_SWSR_U1("general_frame_only_constraint_flag",
		     &ptl->general_frame_only_constraint_flag, sr_ctx);

	if (ptl->general_profile_idc == 4 ||
	    ptl->general_profile_compatibility_flag[4]) {
		HEVC_SWSR_U1("general_max_12bit_constraint_flag",
			     &ptl->general_max_12bit_constraint_flag, sr_ctx);
		HEVC_SWSR_U1("general_max_10bit_constraint_flag",
			     &ptl->general_max_10bit_constraint_flag, sr_ctx);
		HEVC_SWSR_U1("general_max_8bit_constraint_flag",
			     &ptl->general_max_8bit_constraint_flag, sr_ctx);
		HEVC_SWSR_U1("general_max_422chroma_constraint_flag",
			     &ptl->general_max_422chroma_constraint_flag,
			     sr_ctx);
		HEVC_SWSR_U1("general_max_420chroma_constraint_flag",
			     &ptl->general_max_420chroma_constraint_flag,
			     sr_ctx);
		HEVC_SWSR_U1("general_max_monochrome_constraint_flag",
			     &ptl->general_max_monochrome_constraint_flag,
			     sr_ctx);
		HEVC_SWSR_U1("general_intra_constraint_flag",
			     &ptl->general_intra_constraint_flag, sr_ctx);
		HEVC_SWSR_U1("general_one_picture_only_constraint_flag",
			     &ptl->general_one_picture_only_constraint_flag,
			     sr_ctx);
		HEVC_SWSR_U1("general_lower_bit_rate_constraint_flag",
			     &ptl->general_lower_bit_rate_constraint_flag,
			     sr_ctx);
		HEVC_SWSR_UN("general_reserved_zero_35bits", &res, 32, sr_ctx);
		HEVC_UCHECK("general_reserved_zero_35bits", res, 0, &parse_err);
		HEVC_SWSR_UN("general_reserved_zero_35bits", &res, 3, sr_ctx);
		HEVC_UCHECK("general_reserved_zero_35bits", res, 0, &parse_err);
	} else {
		HEVC_SWSR_UN("general_reserved_zero_44bits (1)", &res, 32, sr_ctx);
		HEVC_UCHECK("general_reserved_zero_44bits (1)", res, 0, &parse_err);
		HEVC_SWSR_UN("general_reserved_zero_44bits (2)", &res, 12, sr_ctx);
		HEVC_UCHECK("general_reserved_zero_44bits (2)", res, 0, &parse_err);
	}

	HEVC_SWSR_UN("general_level_idc", (unsigned int *)&ptl->general_level_idc, 8, sr_ctx);
	HEVC_RANGEUCHECK("general_level_idc", ptl->general_level_idc,
			 HEVC_LEVEL_IDC_MIN, HEVC_LEVEL_IDC_MAX, &parse_err);

	for (i = 0; i < vps_maxsublayers_minus1; ++i) {
		HEVC_SWSR_U1("sub_layer_profile_present_flag",
			     &ptl->sub_layer_profile_present_flag[i], sr_ctx);
		HEVC_SWSR_U1("sub_layer_level_present_flag",
			     &ptl->sub_layer_level_present_flag[i], sr_ctx);
	}

	if (vps_maxsublayers_minus1 > 0) {
		for (i = vps_maxsublayers_minus1; i < 8; ++i) {
			HEVC_SWSR_UN("reserved_zero_2bits", &res, 2, sr_ctx);
			HEVC_UCHECK("reserved_zero_2bits", res, 0, &parse_err);
		}
	}

	for (i = 0; i < vps_maxsublayers_minus1; ++i) {
		if (ptl->sub_layer_profile_present_flag[i]) {
			HEVC_SWSR_UN("sub_layer_profile_space",
				     (unsigned int *)&ptl->sub_layer_profile_space[i], 2, sr_ctx);
			HEVC_SWSR_U1("sub_layer_tier_flag", &ptl->sub_layer_tier_flag[i], sr_ctx);
			HEVC_SWSR_UN("sub_layer_profile_idc",
				     (unsigned int *)&ptl->sub_layer_profile_idc[i], 5, sr_ctx);
			for (j = 0; j < HEVC_MAX_NUM_PROFILE_IDC; ++j)
				HEVC_SWSR_U1("sub_layer_profile_compatibility_flag",
					     &ptl->sub_layer_profile_compatibility_flag[i][j],
					     sr_ctx);

			HEVC_SWSR_U1("sub_layer_progressive_source_flag",
				     &ptl->sub_layer_progressive_source_flag[i],
				     sr_ctx);
			HEVC_SWSR_U1("sub_layer_interlaced_source_flag",
				     &ptl->sub_layer_interlaced_source_flag[i],
				     sr_ctx);
			HEVC_SWSR_U1("sub_layer_non_packed_constraint_flag",
				     &ptl->sub_layer_non_packed_constraint_flag[i],
				     sr_ctx);
			HEVC_SWSR_U1("sub_layer_frame_only_constraint_flag",
				     &ptl->sub_layer_frame_only_constraint_flag[i],
				     sr_ctx);

			if (ptl->sub_layer_profile_idc[i] == 4 ||
			    ptl->sub_layer_profile_compatibility_flag[i][4]) {
				HEVC_SWSR_U1("sub_layer_max_12bit_constraint_flag",
					     &ptl->sub_layer_max_12bit_constraint_flag[i],
					     sr_ctx);
				HEVC_SWSR_U1("sub_layer_max_10bit_constraint_flag",
					     &ptl->sub_layer_max_10bit_constraint_flag[i],
					     sr_ctx);
				HEVC_SWSR_U1("sub_layer_max_8bit_constraint_flag",
					     &ptl->sub_layer_max_8bit_constraint_flag[i],
					     sr_ctx);
				HEVC_SWSR_U1("sub_layer_max_422chroma_constraint_flag",
					     &ptl->sub_layer_max_422chroma_constraint_flag[i],
					     sr_ctx);
				HEVC_SWSR_U1("sub_layer_max_420chroma_constraint_flag",
					     &ptl->sub_layer_max_420chroma_constraint_flag[i],
					     sr_ctx);
				HEVC_SWSR_U1("sub_layer_max_monochrome_constraint_flag",
					     &ptl->sub_layer_max_monochrome_constraint_flag[i],
					     sr_ctx);
				HEVC_SWSR_U1("sub_layer_intra_constraint_flag",
					     &ptl->sub_layer_intra_constraint_flag[i],
					     sr_ctx);
				HEVC_SWSR_U1("sub_layer_one_picture_only_constraint_flag",
					     &ptl->sub_layer_one_picture_only_constraint_flag[i],
					     sr_ctx);
				HEVC_SWSR_U1("sub_layer_lower_bit_rate_constraint_flag",
					     &ptl->sub_layer_lower_bit_rate_constraint_flag[i],
					     sr_ctx);
				HEVC_SWSR_UN("sub_layer_reserved_zero_35bits",
					     &res, 32, sr_ctx);
				HEVC_UCHECK("sub_layer_reserved_zero_35bits",
					    res, 0, &parse_err);
				HEVC_SWSR_UN("sub_layer_reserved_zero_35bits",
					     &res, 3, sr_ctx);
				HEVC_UCHECK("sub_layer_reserved_zero_35bits",
					    res, 0, &parse_err);
			} else {
				HEVC_SWSR_UN("sub_layer_reserved_zero_44bits (1)",
					     &res, 32, sr_ctx);
				HEVC_UCHECK("sub_layer_reserved_zero_44bits (1)",
					    res, 0, &parse_err);
				HEVC_SWSR_UN("sub_layer_reserved_zero_44bits (2)",
					     &res, 12, sr_ctx);
				HEVC_UCHECK("sub_layer_reserved_zero_44bits (2)",
					    res, 0, &parse_err);
			}
		}
		if (ptl->sub_layer_level_present_flag[i])
			HEVC_SWSR_UN("sub_layer_level_idc",
				     (unsigned int *)&ptl->sub_layer_level_idc[i], 8, sr_ctx);
	}
	return parse_err;
}

/* Default scaling lists */
#define HEVC_SCALING_LIST_0_SIZE 16
#define HEVC_SCALING_LIST_123_SIZE 64

static const unsigned char def_4x4[HEVC_SCALING_LIST_0_SIZE] = {
	16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16
};

static const unsigned char def_8x8_intra[HEVC_SCALING_LIST_123_SIZE] = {
	16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 16, 17, 16, 17, 18,
	17, 18, 18, 17, 18, 21, 19, 20, 21, 20, 19, 21, 24, 22, 22, 24,
	24, 22, 22, 24, 25, 25, 27, 30, 27, 25, 25, 29, 31, 35, 35, 31,
	29, 36, 41, 44, 41, 36, 47, 54, 54, 47, 65, 70, 65, 88, 88, 115
};

static const unsigned char def_8x8_inter[HEVC_SCALING_LIST_123_SIZE] = {
	16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 18,
	18, 18, 18, 18, 18, 20, 20, 20, 20, 20, 20, 20, 24, 24, 24, 24,
	24, 24, 24, 24, 25, 25, 25, 25, 25, 25, 25, 28, 28, 28, 28, 28,
	28, 33, 33, 33, 33, 33, 41, 41, 41, 41, 54, 54, 54, 71, 71, 91
};

/*
 * Scan order mapping when translating scaling lists from bitstream order
 * to PVDEC order
 */
static const unsigned char HEVC_INV_ZZ_SCAN4[HEVC_SCALING_LIST_MATRIX_SIZE / 4]  = {
	0,  1,  2,  4,  3,  6,  7, 10,  5,  8,  9, 12, 11, 13, 14, 15
};

static const unsigned char HEVC_INV_ZZ_SCAN8[HEVC_SCALING_LIST_MATRIX_SIZE] = {
	0,  1,  2,  4,  3,  6,  7, 11,  5,  8,  9, 13, 12, 17, 18, 24,
	10, 15, 16, 22, 21, 28, 29, 36, 23, 30, 31, 38, 37, 43, 44, 49,
	14, 19, 20, 26, 25, 32, 33, 40, 27, 34, 35, 42, 41, 47, 48, 53,
	39, 45, 46, 51, 50, 54, 55, 58, 52, 56, 57, 60, 59, 61, 62, 63
};

static void bspp_hevc_getdefault_scalinglist
		(unsigned char size_id, unsigned char matrix_id,
		 const unsigned char **default_scalinglist,
		 unsigned int *size)
{
	static const unsigned char *defaultlists
	[HEVC_SCALING_LIST_NUM_SIZES][HEVC_SCALING_LIST_NUM_MATRICES] = {
		{ def_4x4, def_4x4, def_4x4, def_4x4, def_4x4, def_4x4 },
		{ def_8x8_intra, def_8x8_intra, def_8x8_intra,
		  def_8x8_inter, def_8x8_inter, def_8x8_inter },
		{ def_8x8_intra, def_8x8_intra, def_8x8_intra,
		  def_8x8_inter, def_8x8_inter, def_8x8_inter },
		{ def_8x8_intra, def_8x8_inter, NULL, NULL, NULL, NULL }
	};

	static const unsigned int lists_sizes
	[HEVC_SCALING_LIST_NUM_SIZES][HEVC_SCALING_LIST_NUM_MATRICES] = {
		{ sizeof(def_4x4), sizeof(def_4x4), sizeof(def_4x4),
		  sizeof(def_4x4), sizeof(def_4x4), sizeof(def_4x4) },
		{ sizeof(def_8x8_intra), sizeof(def_8x8_intra),
		  sizeof(def_8x8_intra), sizeof(def_8x8_inter),
		  sizeof(def_8x8_inter), sizeof(def_8x8_inter) },
		{ sizeof(def_8x8_intra), sizeof(def_8x8_intra),
		  sizeof(def_8x8_intra), sizeof(def_8x8_inter),
		  sizeof(def_8x8_inter), sizeof(def_8x8_inter) },
		{ sizeof(def_8x8_intra), sizeof(def_8x8_inter), 0, 0, 0, 0 }
	};

	/* to assert that input to this function was correct */
	VDEC_ASSERT(size_id < 4);
	VDEC_ASSERT(size_id < 3 ? (matrix_id < 6) : (matrix_id < 2));

	*default_scalinglist = defaultlists[size_id][matrix_id];
	*size = lists_sizes[size_id][matrix_id];
}

static enum bspp_error_type bspp_hevc_parse_scalinglistdata
			(void *sr_ctx,
			 struct bspp_hevc_scalinglist_data *scaling_listdata)
{
	enum bspp_error_type parse_err = BSPP_ERROR_NONE;
	unsigned char size_id, matrix_id;

	for (size_id = 0; size_id < HEVC_SCALING_LIST_NUM_SIZES; ++size_id) {
		for (matrix_id = 0; matrix_id < ((size_id == 3) ? 2 : 6);
			++matrix_id) {
			/*
			 * Select scaling list on which we will operate in
			 * the iteration
			 */
			unsigned char *scalinglist = scaling_listdata->lists[size_id][matrix_id];

			unsigned char scaling_list_pred_mode_flag = 0;

			HEVC_SWSR_U1("scaling_list_pred_mode_flag",
				     &scaling_list_pred_mode_flag, sr_ctx);
			if (!scaling_list_pred_mode_flag) {
				unsigned char scaling_list_pred_matrix_id_delta = 0;
				const unsigned char *defaultlist = NULL;
				unsigned int listsize = 0;

				HEVC_SWSR_UE("scaling_list_pred_matrixid_delta",
					     (unsigned int *)&scaling_list_pred_matrix_id_delta,
					     sr_ctx);

				bspp_hevc_getdefault_scalinglist(size_id,
								 matrix_id,
								 &defaultlist,
								 &listsize);

				if (scaling_list_pred_matrix_id_delta == 0) {
					/* use default one */
					memcpy(scalinglist, defaultlist, listsize);
					if (size_id > 1)
						scaling_listdata->dccoeffs[size_id -
						2][matrix_id] = 8 + 8;
				} else {
					unsigned char ref_matrix_id =
						matrix_id - scaling_list_pred_matrix_id_delta;
					unsigned char *refscalinglist =
						scaling_listdata->lists[size_id][ref_matrix_id];
					/*
					 *  use reference list given by
					 * scaling_list_pred_matrix_id_delta
					 */
					memcpy(scalinglist, refscalinglist, listsize);
					if (size_id > 1)
						scaling_listdata->dccoeffs[size_id - 2][matrix_id] =
							scaling_listdata->dccoeffs[size_id -
							2][ref_matrix_id];
				}
			} else {
				/*
				 * scaling list coefficients
				 * signalled explicitly
				 */
				static const short coef_startvalue = 8;
				static const unsigned char matrix_max_coef_num = 64;

				short next_coef = coef_startvalue;
				unsigned char coef_num =
					HEVC_MIN(matrix_max_coef_num,
						 (1 << (4 + (size_id << 1))), unsigned char);

				unsigned char i;

				if (size_id > 1) {
					short scaling_list_dc_coef_minus8 = 0;

					HEVC_SWSR_SE("scaling_list_dc_coef_minus8",
						     (int *)&scaling_list_dc_coef_minus8,
						     sr_ctx);
					HEVC_RANGESCHECK("scaling_list_dc_coef_minus8",
							 scaling_list_dc_coef_minus8,
							 -7, 247, &parse_err);

					next_coef = scaling_list_dc_coef_minus8 + 8;
					scaling_listdata->dccoeffs[size_id - 2][matrix_id] =
						(unsigned char)next_coef;
				}
				for (i = 0; i < coef_num; ++i) {
					short scaling_list_delta_coef = 0;

					HEVC_SWSR_SE("scaling_list_delta_coef",
						     (int *)&scaling_list_delta_coef, sr_ctx);
					HEVC_RANGESCHECK("scaling_list_delta_coef",
							 scaling_list_delta_coef, -128, 127,
							 &parse_err);

					next_coef = (next_coef + scaling_list_delta_coef + 256) &
											0xFF;
					scalinglist[i] = next_coef;
				}
			}
		}
	}

#ifdef DEBUG_DECODER_DRIVER
	/* print calculated scaling lists */
	for (size_id = 0; size_id < HEVC_SCALING_LIST_NUM_SIZES; ++size_id) {
		for (matrix_id = 0; matrix_id < ((size_id == 3) ? 2 : 6);
			++matrix_id) {
			unsigned char i = 0;
			/*
			 * Select scaling list on which we will operate
			 * in the iteration
			 */
			unsigned char *scalinglist = scaling_listdata->lists[size_id][matrix_id];

			for (; i < ((size_id == 0) ? 16 : 64); ++i) {
				BSPP_HEVC_SYNTAX("scalinglist[%u][%u][%u] = %u",
						 size_id,
						 matrix_id,
						 i,
						 scalinglist[i]);
			}
		}
	}
#endif

	return parse_err;
}

static void
bspp_hevc_usedefault_scalinglists(struct bspp_hevc_scalinglist_data *scaling_listdata)
{
	unsigned char size_id, matrix_id;

	for (size_id = 0; size_id < HEVC_SCALING_LIST_NUM_SIZES; ++size_id) {
		for (matrix_id = 0; matrix_id < ((size_id == 3) ? 2 : 6);
			++matrix_id) {
			unsigned char *list = scaling_listdata->lists[size_id][matrix_id];
			const unsigned char *defaultlist = NULL;
			unsigned int listsize = 0;

			bspp_hevc_getdefault_scalinglist(size_id, matrix_id, &defaultlist,
							 &listsize);

			memcpy(list, defaultlist, listsize);
		}
	}

	memset(scaling_listdata->dccoeffs, 8 + 8, sizeof(scaling_listdata->dccoeffs));
}

static enum bspp_error_type bspp_hevc_parse_shortterm_refpicset
				(void *sr_ctx,
				 struct bspp_hevc_shortterm_refpicset *st_refpicset,
				 unsigned char st_rps_idx,
				 unsigned char in_slice_header)
{
	/*
	 * Note: unfortunately short term ref pic set has to be
	 * "partially-decoded" and parsed at the same time because derived
	 * syntax elements are used for prediction of subsequent
	 * short term ref pic sets.
	 */
	enum bspp_error_type parse_err = BSPP_ERROR_NONE;

	struct bspp_hevc_shortterm_refpicset *strps =
		&st_refpicset[st_rps_idx];
	unsigned char inter_ref_pic_set_prediction_flag = 0;
	unsigned int i = 0;

	memset(strps, 0, sizeof(*strps));

	if (st_rps_idx != 0) {
		HEVC_SWSR_U1("inter_ref_pic_set_prediction_flag",
			     &inter_ref_pic_set_prediction_flag, sr_ctx);
	}

	if (inter_ref_pic_set_prediction_flag) {
		signed char j = 0;
		unsigned char j_8 = 0;
		unsigned char ref_rps_idx = 0;
		int delta_rps = 0;
		unsigned char i = 0;
		unsigned char delta_idx_minus1 = 0;
		unsigned char delta_rps_sign = 0;
		unsigned short abs_delta_rps_minus1 = 0;
		unsigned char used_by_curr_pic_flag[HEVC_MAX_NUM_REF_PICS];
		unsigned char use_delta_flag[HEVC_MAX_NUM_REF_PICS];

		struct bspp_hevc_shortterm_refpicset *ref_strps = NULL;

		if (in_slice_header) {
			HEVC_SWSR_UE("delta_idx_minus1", (unsigned int *)&delta_idx_minus1, sr_ctx);
			HEVC_RANGEUCHECK("delta_idx_minus1", delta_idx_minus1, 0, st_rps_idx - 1,
					 &parse_err);
		}

		HEVC_SWSR_U1("delta_rps_sign", &delta_rps_sign, sr_ctx);
		HEVC_SWSR_UE("abs_delta_rps_minus1", (unsigned int *)&abs_delta_rps_minus1, sr_ctx);
		HEVC_RANGEUCHECK("abs_delta_rps_minus1", abs_delta_rps_minus1, 0, ((1 << 15) - 1),
				 &parse_err);

		ref_rps_idx = st_rps_idx - (delta_idx_minus1 + 1);
		ref_strps = &st_refpicset[ref_rps_idx];

		memset(use_delta_flag, 1, sizeof(use_delta_flag));

		for (j_8 = 0; j_8 <= ref_strps->num_delta_pocs; ++j_8) {
			HEVC_SWSR_U1("used_by_curr_pic_flag", &used_by_curr_pic_flag[j_8], sr_ctx);
			if (!used_by_curr_pic_flag[j_8])
				HEVC_SWSR_U1("use_delta_flag", &use_delta_flag[j_8], sr_ctx);
		}

		delta_rps =
			(1 - 2 * delta_rps_sign) * (abs_delta_rps_minus1 + 1);

		/*
		 * predict delta POC values of current strps from
		 * reference strps
		 */
		for (j = ref_strps->num_positive_pics - 1; j >= 0; --j) {
			int dpoc = ref_strps->delta_poc_s1[j] + delta_rps;

			if (dpoc < 0 && use_delta_flag[ref_strps->num_negative_pics + j]) {
				strps->delta_poc_s0[i] = dpoc;
				strps->used_bycurr_pic_s0[i++] =
					used_by_curr_pic_flag[ref_strps->num_negative_pics + j];
			}
		}

		if (delta_rps < 0 && use_delta_flag[ref_strps->num_delta_pocs]) {
			strps->delta_poc_s0[i] = delta_rps;
			strps->used_bycurr_pic_s0[i++] =
				used_by_curr_pic_flag[ref_strps->num_delta_pocs];
		}

		for (j_8 = 0; j_8 < ref_strps->num_negative_pics; ++j_8) {
			int dpoc = ref_strps->delta_poc_s0[j_8] + delta_rps;

			if (dpoc < 0 && use_delta_flag[j_8]) {
				strps->delta_poc_s0[i] = dpoc;
				strps->used_bycurr_pic_s0[i++] = used_by_curr_pic_flag[j_8];
			}
		}

		strps->num_negative_pics = i;

		i = 0;
		for (j = ref_strps->num_negative_pics - 1; j >= 0; --j) {
			int dpoc = ref_strps->delta_poc_s0[j] + delta_rps;

			if (dpoc > 0 && use_delta_flag[j]) {
				strps->delta_poc_s1[i] = dpoc;
				strps->used_bycurr_pic_s1[i++] =
					used_by_curr_pic_flag[j];
			}
		}

		if (delta_rps > 0 && use_delta_flag[ref_strps->num_delta_pocs]) {
			strps->delta_poc_s1[i] = delta_rps;
			strps->used_bycurr_pic_s1[i++] =
				used_by_curr_pic_flag[ref_strps->num_delta_pocs];
		}

		for (j_8 = 0; j_8 < ref_strps->num_positive_pics; ++j_8) {
			int dpoc = ref_strps->delta_poc_s1[j_8] + delta_rps;

			if (dpoc > 0 && use_delta_flag[ref_strps->num_negative_pics + j_8]) {
				strps->delta_poc_s1[i] = dpoc;
				strps->used_bycurr_pic_s1[i++] =
					used_by_curr_pic_flag[ref_strps->num_negative_pics + j_8];
			}
		}

		strps->num_positive_pics = i;
		strps->num_delta_pocs = strps->num_negative_pics + strps->num_positive_pics;
		if (strps->num_delta_pocs > (HEVC_MAX_NUM_REF_PICS - 1)) {
			strps->num_delta_pocs = HEVC_MAX_NUM_REF_PICS - 1;
			parse_err |= BSPP_ERROR_CORRECTION_VALIDVALUE;
		}
	} else {
		unsigned char num_negative_pics = 0;
		unsigned char num_positive_pics = 0;
		unsigned short delta_poc_s0_minus1[HEVC_MAX_NUM_REF_PICS];
		unsigned char used_by_curr_pic_s0_flag[HEVC_MAX_NUM_REF_PICS];
		unsigned short delta_poc_s1_minus1[HEVC_MAX_NUM_REF_PICS];
		unsigned char used_by_curr_pic_s1_flag[HEVC_MAX_NUM_REF_PICS];
		unsigned char j = 0;

		HEVC_SWSR_UE("num_negative_pics", (unsigned int *)&num_negative_pics, sr_ctx);
		if (num_negative_pics > HEVC_MAX_NUM_REF_PICS) {
			num_negative_pics = HEVC_MAX_NUM_REF_PICS;
			parse_err |= BSPP_ERROR_CORRECTION_VALIDVALUE;
		}
		HEVC_SWSR_UE("num_positive_pics", (unsigned int *)&num_positive_pics, sr_ctx);
		if (num_positive_pics > HEVC_MAX_NUM_REF_PICS) {
			num_positive_pics = HEVC_MAX_NUM_REF_PICS;
			parse_err |= BSPP_ERROR_CORRECTION_VALIDVALUE;
		}

		for (j = 0; j < num_negative_pics; ++j) {
			HEVC_SWSR_UE("delta_poc_s0_minus1",
				     (unsigned int *)&delta_poc_s0_minus1[j], sr_ctx);
			HEVC_RANGEUCHECK("delta_poc_s0_minus1", delta_poc_s0_minus1[j], 0,
					 ((1 << 15) - 1), &parse_err);
			HEVC_SWSR_U1("used_by_curr_pic_s0_flag",
				     &used_by_curr_pic_s0_flag[j], sr_ctx);

			if (j == 0)
				strps->delta_poc_s0[j] =
					-(delta_poc_s0_minus1[j] + 1);
			else
				strps->delta_poc_s0[j] = strps->delta_poc_s0[j - 1] -
							(delta_poc_s0_minus1[j] + 1);

			strps->used_bycurr_pic_s0[j] = used_by_curr_pic_s0_flag[j];
		}

		for (j = 0; j < num_positive_pics; j++) {
			HEVC_SWSR_UE("delta_poc_s1_minus1",
				     (unsigned int *)&delta_poc_s1_minus1[j], sr_ctx);
			HEVC_RANGEUCHECK("delta_poc_s1_minus1", delta_poc_s1_minus1[j], 0,
					 ((1 << 15) - 1), &parse_err);
			HEVC_SWSR_U1("used_by_curr_pic_s1_flag",
				     &used_by_curr_pic_s1_flag[j], sr_ctx);

			if (j == 0)
				strps->delta_poc_s1[j] =
					(delta_poc_s1_minus1[j] + 1);
			else
				strps->delta_poc_s1[j] = strps->delta_poc_s1[j - 1] +
							(delta_poc_s1_minus1[j] + 1);
			strps->used_bycurr_pic_s1[j] = used_by_curr_pic_s1_flag[j];
		}

		strps->num_negative_pics = num_negative_pics;
		strps->num_positive_pics = num_positive_pics;
		strps->num_delta_pocs = strps->num_negative_pics + strps->num_positive_pics;
		if (strps->num_delta_pocs > (HEVC_MAX_NUM_REF_PICS - 1)) {
			strps->num_delta_pocs = HEVC_MAX_NUM_REF_PICS - 1;
			parse_err |= BSPP_ERROR_CORRECTION_VALIDVALUE;
		}
	}

	BSPP_HEVC_SYNTAX
		("strps[%u]: num_delta_pocs: %u (%u (num_negative_pics) + %u (num_positive_pics))",
		 st_rps_idx, strps->num_delta_pocs, strps->num_negative_pics,
		 strps->num_positive_pics);

	for (i = 0; i < strps->num_negative_pics; ++i) {
		BSPP_HEVC_SYNTAX("StRps[%u][%u]: delta_poc_s0: %d, used_bycurr_pic_s0: %u",
				 st_rps_idx, i, strps->delta_poc_s0[i],
				 strps->used_bycurr_pic_s0[i]);
	}

	for (i = 0; i < strps->num_positive_pics; ++i) {
		BSPP_HEVC_SYNTAX("StRps[%u][%u]: delta_poc_s1: %d, used_bycurr_pic_s1: %u",
				 st_rps_idx, i, strps->delta_poc_s1[i],
				 strps->used_bycurr_pic_s1[i]);
	}

	return parse_err;
}

static void bspp_hevc_fillcommonseqhdr(struct bspp_hevc_sps *sps,
				       struct vdec_comsequ_hdrinfo *common_seq)
{
	struct bspp_hevc_vui_params *vui = &sps->vui_params;
	unsigned char chroma_idc = sps->chroma_format_idc;
	struct pixel_pixinfo *pixel_info = &common_seq->pixel_info;
	unsigned int maxsub_layersmin1;
	unsigned int maxdpb_size;
	struct vdec_rect *rawdisp_region;

	common_seq->codec_profile = sps->profile_tier_level.general_profile_idc;
	common_seq->codec_level   = sps->profile_tier_level.general_level_idc;

	if (sps->vui_parameters_present_flag &&
	    vui->vui_timing_info_present_flag) {
		common_seq->frame_rate_num = vui->vui_time_scale;
		common_seq->frame_rate_den = vui->vui_num_units_in_tick;
		common_seq->frame_rate =
			1 * common_seq->frame_rate_num / common_seq->frame_rate_den;
	}

	if (vui->aspect_ratio_info_present_flag) {
		common_seq->aspect_ratio_num = vui->sar_width;
		common_seq->aspect_ratio_den = vui->sar_height;
	}

	common_seq->interlaced_frames = 0;

	/* handle pixel format definitions */
	pixel_info->chroma_fmt = chroma_idc == 0 ? 0 : 1;
	pixel_info->chroma_fmt_idc = pixelformat_idc[chroma_idc];
	pixel_info->chroma_interleave =
		chroma_idc == 0 ? PIXEL_INVALID_CI : PIXEL_UV_ORDER;
	pixel_info->bitdepth_y = sps->bit_depth_luma_minus8 + 8;
	pixel_info->bitdepth_c = sps->bit_depth_chroma_minus8 + 8;

	pixel_info->mem_pkg = (pixel_info->bitdepth_y > 8 ||
		(pixel_info->bitdepth_c > 8 && pixel_info->chroma_fmt)) ?
		PIXEL_BIT10_MSB_MP : PIXEL_BIT8_MP;
	pixel_info->num_planes =
		chroma_idc == 0 ? 1 : (sps->separate_colour_plane_flag ? 3 : 2);

	pixel_info->pixfmt = pixel_get_pixfmt(pixel_info->chroma_fmt_idc,
					      pixel_info->chroma_interleave,
					      pixel_info->mem_pkg,
					      pixel_info->bitdepth_y,
					      pixel_info->chroma_fmt ?
					      pixel_info->bitdepth_c : PIXEL_INVALID_BDC,
					      pixel_info->num_planes);

	common_seq->max_frame_size.width = sps->pic_width_in_ctbs_y * sps->ctb_size_y;
	common_seq->max_frame_size.height = sps->pic_height_in_ctbs_y * sps->ctb_size_y;

	common_seq->frame_size.width = sps->pic_width_in_luma_samples;
	common_seq->frame_size.height = sps->pic_height_in_luma_samples;

	/* Get HEVC max num ref pictures and pass to bspp info*/
	vdecddutils_ref_pic_hevc_get_maxnum(common_seq, &common_seq->max_ref_frame_num);

	common_seq->field_codec_mblocks = 0;

	maxsub_layersmin1 = sps->sps_max_sub_layers_minus1;
	maxdpb_size =
		HEVC_MAX(sps->sps_max_dec_pic_buffering_minus1[maxsub_layersmin1] + 1,
			 sps->sps_max_num_reorder_pics[maxsub_layersmin1], unsigned char);

	if (sps->sps_max_latency_increase_plus1[maxsub_layersmin1]) {
		maxdpb_size =
			HEVC_MAX(maxdpb_size,
				 sps->sps_max_latency_pictures[maxsub_layersmin1], unsigned int);
	}

	maxdpb_size = HEVC_MIN(maxdpb_size,
			       HEVC_MAX_NUM_REF_IDX_ACTIVE + 1, unsigned int);

	common_seq->min_pict_buf_num = HEVC_MAX(maxdpb_size, 6, unsigned int);

	common_seq->picture_reordering = 1;
	common_seq->post_processing = 0;

	/* handle display region calculation */
	rawdisp_region = &common_seq->raw_display_region;

	rawdisp_region->width = sps->pic_width_in_luma_samples;
	rawdisp_region->height = sps->pic_height_in_luma_samples;
	rawdisp_region->top_offset = 0;
	rawdisp_region->left_offset = 0;

	if (sps->conformance_window_flag) {
		struct vdec_rect *disp_region =
			&common_seq->orig_display_region;

		disp_region->top_offset =
			sps->sub_height_c * sps->conf_win_top_offset;
		disp_region->left_offset =
			sps->sub_width_c * sps->conf_win_left_offset;
		disp_region->width =
			sps->pic_width_in_luma_samples -
			disp_region->left_offset -
			sps->sub_width_c * sps->conf_win_right_offset;
		disp_region->height =
			sps->pic_height_in_luma_samples -
			disp_region->top_offset -
			sps->sub_height_c * sps->conf_win_bottom_offset;
	} else {
		common_seq->orig_display_region =
			common_seq->raw_display_region;
	}
}

static void bspp_hevc_fillpicturehdr(struct vdec_comsequ_hdrinfo *common_seq,
				     enum hevc_nalunittype nalunit_type,
				     struct bspp_pict_hdr_info *picture_hdr,
				     struct bspp_hevc_sps *sps,
				     struct bspp_hevc_pps *pps,
				     struct bspp_hevc_vps *vps)
{
	picture_hdr->intra_coded = (nalunit_type == HEVC_NALTYPE_IDR_W_RADL ||
		nalunit_type == HEVC_NALTYPE_IDR_N_LP);
	picture_hdr->field = 0;
	picture_hdr->post_processing = 0;
	picture_hdr->discontinuous_mbs = 0;
	picture_hdr->pict_aux_data.id = BSPP_INVALID;
	picture_hdr->second_pict_aux_data.id = BSPP_INVALID;
	picture_hdr->pict_sgm_data.id = BSPP_INVALID;
	picture_hdr->coded_frame_size.width =
		HEVC_ALIGN(sps->pic_width_in_luma_samples, HEVC_MIN_CODED_UNIT_SIZE, unsigned int);
	picture_hdr->coded_frame_size.height =
		HEVC_ALIGN(sps->pic_height_in_luma_samples, HEVC_MIN_CODED_UNIT_SIZE, unsigned int);
	picture_hdr->disp_info.enc_disp_region = common_seq->orig_display_region;
	picture_hdr->disp_info.disp_region = common_seq->orig_display_region;
	picture_hdr->disp_info.raw_disp_region = common_seq->raw_display_region;
	picture_hdr->disp_info.num_pan_scan_windows = 0;
	picture_hdr->hevc_pict_hdr_info.range_ext_present =
			(sps->profile_tier_level.general_profile_idc == 4) ||
			sps->profile_tier_level.general_profile_compatibility_flag[4];

	picture_hdr->hevc_pict_hdr_info.is_full_range_ext = 0;
	if (picture_hdr->hevc_pict_hdr_info.range_ext_present &&
	    (bspp_hevc_checkppsrangeextensions(&pps->range_exts) ||
	    bspp_hevc_checksps_range_extensions(&sps->range_exts)))
		picture_hdr->hevc_pict_hdr_info.is_full_range_ext = 1;

	memset(picture_hdr->disp_info.pan_scan_windows, 0,
	       sizeof(picture_hdr->disp_info.pan_scan_windows));
}

static void bspp_hevc_fill_fwsps(struct bspp_hevc_sps *sps, struct hevcfw_sequence_ps *fwsps)
{
	unsigned char i;

	fwsps->pic_width_in_luma_samples = sps->pic_width_in_luma_samples;
	fwsps->pic_height_in_luma_samples = sps->pic_height_in_luma_samples;
	fwsps->num_short_term_ref_pic_sets = sps->num_short_term_ref_pic_sets;
	fwsps->num_long_term_ref_pics_sps = sps->num_long_term_ref_pics_sps;
	fwsps->sps_max_sub_layers_minus1 = sps->sps_max_sub_layers_minus1;
	fwsps->max_transform_hierarchy_depth_inter =
				sps->max_transform_hierarchy_depth_inter;
	fwsps->max_transform_hierarchy_depth_intra =
				sps->max_transform_hierarchy_depth_intra;
	fwsps->log2_diff_max_min_transform_block_size =
				sps->log2_diff_max_min_transform_block_size;
	fwsps->log2_min_transform_block_size_minus2 =
				sps->log2_min_transform_block_size_minus2;
	fwsps->log2_diff_max_min_luma_coding_block_size =
				sps->log2_diff_max_min_luma_coding_block_size;
	fwsps->log2_min_luma_coding_block_size_minus3 =
				sps->log2_min_luma_coding_block_size_minus3;

	HEVC_STATIC_ASSERT(sizeof(sps->sps_max_dec_pic_buffering_minus1) ==
			   sizeof(fwsps->sps_max_dec_pic_buffering_minus1));
	memcpy(fwsps->sps_max_dec_pic_buffering_minus1, sps->sps_max_dec_pic_buffering_minus1,
	       sizeof(fwsps->sps_max_dec_pic_buffering_minus1[0]) *
	       (sps->sps_max_sub_layers_minus1 + 1));

	HEVC_STATIC_ASSERT(sizeof(sps->sps_max_num_reorder_pics) ==
			   sizeof(fwsps->sps_max_num_reorder_pics));
	memcpy(fwsps->sps_max_num_reorder_pics, sps->sps_max_num_reorder_pics,
	       sizeof(fwsps->sps_max_num_reorder_pics[0]) *
	       (sps->sps_max_sub_layers_minus1 + 1));

	HEVC_STATIC_ASSERT(sizeof(sps->sps_max_latency_increase_plus1) ==
			   sizeof(fwsps->sps_max_latency_increase_plus1));
	memcpy(fwsps->sps_max_latency_increase_plus1, sps->sps_max_latency_increase_plus1,
	       sizeof(fwsps->sps_max_latency_increase_plus1[0]) *
	       (sps->sps_max_sub_layers_minus1 + 1));

	fwsps->chroma_format_idc = sps->chroma_format_idc;
	fwsps->separate_colour_plane_flag = sps->separate_colour_plane_flag;
	fwsps->log2_max_pic_order_cnt_lsb_minus4 =
		sps->log2_max_pic_order_cnt_lsb_minus4;
	fwsps->long_term_ref_pics_present_flag =
		sps->long_term_ref_pics_present_flag;
	fwsps->sample_adaptive_offset_enabled_flag =
		sps->sample_adaptive_offset_enabled_flag;
	fwsps->sps_temporal_mvp_enabled_flag =
		sps->sps_temporal_mvp_enabled_flag;
	fwsps->bit_depth_luma_minus8 = sps->bit_depth_luma_minus8;
	fwsps->bit_depth_chroma_minus8 = sps->bit_depth_chroma_minus8;
	fwsps->pcm_sample_bit_depth_luma_minus1 =
		sps->pcm_sample_bit_depth_luma_minus1;
	fwsps->pcm_sample_bit_depth_chroma_minus1 =
		sps->pcm_sample_bit_depth_chroma_minus1;
	fwsps->log2_min_pcm_luma_coding_block_size_minus3 =
		sps->log2_min_pcm_luma_coding_block_size_minus3;
	fwsps->log2_diff_max_min_pcm_luma_coding_block_size =
		sps->log2_diff_max_min_pcm_luma_coding_block_size;
	fwsps->pcm_loop_filter_disabled_flag =
		sps->pcm_loop_filter_disabled_flag;
	fwsps->amp_enabled_flag = sps->amp_enabled_flag;
	fwsps->pcm_enabled_flag = sps->pcm_enabled_flag;
	fwsps->strong_intra_smoothing_enabled_flag =
		sps->strong_intra_smoothing_enabled_flag;
	fwsps->scaling_list_enabled_flag = sps->scaling_list_enabled_flag;
	fwsps->transform_skip_rotation_enabled_flag =
		sps->range_exts.transform_skip_rotation_enabled_flag;
	fwsps->transform_skip_context_enabled_flag =
		sps->range_exts.transform_skip_context_enabled_flag;
	fwsps->implicit_rdpcm_enabled_flag =
		sps->range_exts.implicit_rdpcm_enabled_flag;
	fwsps->explicit_rdpcm_enabled_flag =
		sps->range_exts.explicit_rdpcm_enabled_flag;
	fwsps->extended_precision_processing_flag =
		sps->range_exts.extended_precision_processing_flag;
	fwsps->intra_smoothing_disabled_flag =
		sps->range_exts.intra_smoothing_disabled_flag;
	/* high precision makes no sense for 8 bit luma & chroma,
	 * so forward this parameter only when bitdepth > 8
	 */
	if (sps->bit_depth_luma_minus8 || sps->bit_depth_chroma_minus8)
		fwsps->high_precision_offsets_enabled_flag =
			sps->range_exts.high_precision_offsets_enabled_flag;

	fwsps->persistent_rice_adaptation_enabled_flag =
		sps->range_exts.persistent_rice_adaptation_enabled_flag;
	fwsps->cabac_bypass_alignment_enabled_flag =
		sps->range_exts.cabac_bypass_alignment_enabled_flag;

	HEVC_STATIC_ASSERT(sizeof(sps->lt_ref_pic_poc_lsb_sps) ==
			   sizeof(fwsps->lt_ref_pic_poc_lsb_sps));
	HEVC_STATIC_ASSERT(sizeof(sps->used_by_curr_pic_lt_sps_flag) ==
			   sizeof(fwsps->used_by_curr_pic_lt_sps_flag));
	memcpy(fwsps->lt_ref_pic_poc_lsb_sps, sps->lt_ref_pic_poc_lsb_sps,
	       sizeof(fwsps->lt_ref_pic_poc_lsb_sps[0]) *
		sps->num_long_term_ref_pics_sps);
	memcpy(fwsps->used_by_curr_pic_lt_sps_flag, sps->used_by_curr_pic_lt_sps_flag,
	       sizeof(fwsps->used_by_curr_pic_lt_sps_flag[0]) * sps->num_long_term_ref_pics_sps);

	for (i = 0; i < sps->num_short_term_ref_pic_sets; ++i)
		bspp_hevc_fill_fwst_rps(&sps->rps_list[i], &fwsps->st_rps_list[i]);

	/* derived elements */
	fwsps->pic_size_in_ctbs_y = sps->pic_size_in_ctbs_y;
	fwsps->pic_height_in_ctbs_y = sps->pic_height_in_ctbs_y;
	fwsps->pic_width_in_ctbs_y = sps->pic_width_in_ctbs_y;
	fwsps->ctb_size_y = sps->ctb_size_y;
	fwsps->ctb_log2size_y = sps->ctb_log2size_y;
	fwsps->max_pic_order_cnt_lsb = sps->max_pic_order_cnt_lsb;

	HEVC_STATIC_ASSERT(sizeof(sps->sps_max_latency_pictures) ==
			   sizeof(fwsps->sps_max_latency_pictures));
	memcpy(fwsps->sps_max_latency_pictures, sps->sps_max_latency_pictures,
	       sizeof(fwsps->sps_max_latency_pictures[0]) *
	      (sps->sps_max_sub_layers_minus1 + 1));
}

static void bspp_hevc_fill_fwst_rps(struct bspp_hevc_shortterm_refpicset *strps,
				    struct hevcfw_short_term_ref_picset *fwstrps)
{
	fwstrps->num_delta_pocs = strps->num_delta_pocs;
	fwstrps->num_negative_pics = strps->num_negative_pics;
	fwstrps->num_positive_pics = strps->num_positive_pics;

	HEVC_STATIC_ASSERT(sizeof(strps->delta_poc_s0) ==
			   sizeof(fwstrps->delta_poc_s0));
	memcpy(fwstrps->delta_poc_s0, strps->delta_poc_s0,
	       sizeof(fwstrps->delta_poc_s0[0]) * strps->num_negative_pics);

	HEVC_STATIC_ASSERT(sizeof(strps->delta_poc_s1) ==
			   sizeof(fwstrps->delta_poc_s1));
	memcpy(fwstrps->delta_poc_s1, strps->delta_poc_s1,
	       sizeof(fwstrps->delta_poc_s1[0]) * strps->num_positive_pics);

	HEVC_STATIC_ASSERT(sizeof(strps->used_bycurr_pic_s0) ==
			   sizeof(fwstrps->used_bycurr_pic_s0));
	memcpy(fwstrps->used_bycurr_pic_s0, strps->used_bycurr_pic_s0,
	       sizeof(fwstrps->used_bycurr_pic_s0[0]) * strps->num_negative_pics);

	HEVC_STATIC_ASSERT(sizeof(strps->used_bycurr_pic_s1) ==
			   sizeof(fwstrps->used_bycurr_pic_s1));
	memcpy(fwstrps->used_bycurr_pic_s1, strps->used_bycurr_pic_s1,
	       sizeof(fwstrps->used_bycurr_pic_s1[0]) * strps->num_positive_pics);
}

static void bspp_hevc_fill_fwpps(struct bspp_hevc_pps *pps, struct hevcfw_picture_ps *fw_pps)
{
	fw_pps->pps_pic_parameter_set_id = pps->pps_pic_parameter_set_id;
	fw_pps->num_tile_columns_minus1 = pps->num_tile_columns_minus1;
	fw_pps->num_tile_rows_minus1 = pps->num_tile_rows_minus1;
	fw_pps->diff_cu_qp_delta_depth = pps->diff_cu_qp_delta_depth;
	fw_pps->init_qp_minus26 = pps->init_qp_minus26;
	fw_pps->pps_beta_offset_div2 = pps->pps_beta_offset_div2;
	fw_pps->pps_tc_offset_div2 = pps->pps_tc_offset_div2;
	fw_pps->pps_cb_qp_offset = pps->pps_cb_qp_offset;
	fw_pps->pps_cr_qp_offset = pps->pps_cr_qp_offset;
	fw_pps->log2_parallel_merge_level_minus2 =
		pps->log2_parallel_merge_level_minus2;

	fw_pps->dependent_slice_segments_enabled_flag =
		pps->dependent_slice_segments_enabled_flag;
	fw_pps->output_flag_present_flag = pps->output_flag_present_flag;
	fw_pps->num_extra_slice_header_bits = pps->num_extra_slice_header_bits;
	fw_pps->lists_modification_present_flag =
		pps->lists_modification_present_flag;
	fw_pps->cabac_init_present_flag = pps->cabac_init_present_flag;
	fw_pps->weighted_pred_flag = pps->weighted_pred_flag;
	fw_pps->weighted_bipred_flag = pps->weighted_bipred_flag;
	fw_pps->pps_slice_chroma_qp_offsets_present_flag =
		pps->pps_slice_chroma_qp_offsets_present_flag;
	fw_pps->deblocking_filter_override_enabled_flag =
		pps->deblocking_filter_override_enabled_flag;
	fw_pps->tiles_enabled_flag = pps->tiles_enabled_flag;
	fw_pps->entropy_coding_sync_enabled_flag =
		pps->entropy_coding_sync_enabled_flag;
	fw_pps->slice_segment_header_extension_present_flag =
		pps->slice_segment_header_extension_present_flag;
	fw_pps->transquant_bypass_enabled_flag =
		pps->transquant_bypass_enabled_flag;
	fw_pps->cu_qp_delta_enabled_flag = pps->cu_qp_delta_enabled_flag;
	fw_pps->transform_skip_enabled_flag = pps->transform_skip_enabled_flag;
	fw_pps->sign_data_hiding_enabled_flag =
		pps->sign_data_hiding_enabled_flag;
	fw_pps->num_ref_idx_l0_default_active_minus1 =
		pps->num_ref_idx_l0_default_active_minus1;
	fw_pps->num_ref_idx_l1_default_active_minus1 =
		pps->num_ref_idx_l1_default_active_minus1;
	fw_pps->constrained_intra_pred_flag =  pps->constrained_intra_pred_flag;
	fw_pps->pps_deblocking_filter_disabled_flag =
		pps->pps_deblocking_filter_disabled_flag;
	fw_pps->pps_loop_filter_across_slices_enabled_flag =
		pps->pps_loop_filter_across_slices_enabled_flag;
	fw_pps->loop_filter_across_tiles_enabled_flag =
		pps->loop_filter_across_tiles_enabled_flag;
	fw_pps->log2_max_transform_skip_block_size_minus2 =
		pps->range_exts.log2_max_transform_skip_block_size_minus2;
	fw_pps->cross_component_prediction_enabled_flag =
		pps->range_exts.cross_component_prediction_enabled_flag;
	fw_pps->chroma_qp_offset_list_enabled_flag =
		pps->range_exts.chroma_qp_offset_list_enabled_flag;
	fw_pps->diff_cu_chroma_qp_offset_depth =
		pps->range_exts.diff_cu_chroma_qp_offset_depth;
	fw_pps->chroma_qp_offset_list_len_minus1 =
		pps->range_exts.chroma_qp_offset_list_len_minus1;
	memcpy(fw_pps->cb_qp_offset_list, pps->range_exts.cb_qp_offset_list,
	       sizeof(pps->range_exts.cb_qp_offset_list));
	memcpy(fw_pps->cr_qp_offset_list, pps->range_exts.cr_qp_offset_list,
	       sizeof(pps->range_exts.cr_qp_offset_list));

	/* derived elements */
	HEVC_STATIC_ASSERT(sizeof(pps->col_bd) == sizeof(fw_pps->col_bd));
	HEVC_STATIC_ASSERT(sizeof(pps->row_bd) == sizeof(fw_pps->row_bd));
	memcpy(fw_pps->col_bd, pps->col_bd, sizeof(fw_pps->col_bd));
	memcpy(fw_pps->row_bd, pps->row_bd, sizeof(fw_pps->row_bd));
}

static void bspp_hevc_fill_fw_scaling_lists(struct bspp_hevc_pps *pps,
					    struct bspp_hevc_sps *sps,
					    struct hevcfw_picture_ps *fw_pps)
{
	signed char size_id, matrix_id;
	unsigned char *scalinglist;
	/*
	 * We are starting at 1 to leave space for addresses,
	 * filled by lower layer
	 */
	unsigned int *scaling_lists = &fw_pps->scaling_lists[1];
	unsigned char i;

	struct bspp_hevc_scalinglist_data *scaling_listdata =
		pps->pps_scaling_list_data_present_flag ?
		&pps->scaling_list :
		&sps->scalinglist_data;

	if (!sps->scaling_list_enabled_flag)
		return;

	fw_pps->scaling_list_enabled_flag = sps->scaling_list_enabled_flag;

	for (size_id = HEVC_SCALING_LIST_NUM_SIZES - 1;
		size_id >= 0; --size_id) {
		const unsigned char *zz =
			(size_id == 0 ? HEVC_INV_ZZ_SCAN4 : HEVC_INV_ZZ_SCAN8);

		for (matrix_id = 0; matrix_id < ((size_id == 3) ? 2 : 6);
			++matrix_id) {
			/*
			 * Select scaling list on which we will operate
			 * in the iteration
			 */
			scalinglist =
				scaling_listdata->lists[size_id][matrix_id];

			for (i = 0; i < ((size_id == 0) ? 16 : 64); i += 4) {
				*scaling_lists =
					scalinglist[zz[i + 3]] << 24 |
					scalinglist[zz[i + 2]] << 16 |
					scalinglist[zz[i + 1]] << 8 |
					scalinglist[zz[i]];
				scaling_lists += 2;
			}
		}
	}

	for (i = 0; i < 2; ++i) {
		*scaling_lists = scaling_listdata->dccoeffs[1][i];
		scaling_lists += 2;
	}

	for (i = 0; i < 6; ++i) {
		*scaling_lists = scaling_listdata->dccoeffs[0][i];
		scaling_lists += 2;
	}
}

static unsigned int bspp_ceil_log2(unsigned int linear_val)
{
	unsigned int log_val = 0;

	if (linear_val > 0)
		--linear_val;

	while (linear_val > 0) {
		linear_val >>= 1;
		++log_val;
	}

	return log_val;
}

static unsigned char bspp_hevc_picture_is_irap(enum hevc_nalunittype nalunit_type)
{
	return (nalunit_type >= HEVC_NALTYPE_BLA_W_LP) &&
	       (nalunit_type <= HEVC_NALTYPE_RSV_IRAP_VCL23);
}

static unsigned char bspp_hevc_picture_is_cra(enum hevc_nalunittype nalunit_type)
{
	return (nalunit_type == HEVC_NALTYPE_CRA);
}

static unsigned char bspp_hevc_picture_is_idr(enum hevc_nalunittype nalunit_type)
{
	return (nalunit_type == HEVC_NALTYPE_IDR_N_LP) ||
	       (nalunit_type == HEVC_NALTYPE_IDR_W_RADL);
}

static unsigned char bspp_hevc_picture_is_bla(enum hevc_nalunittype nalunit_type)
{
	return (nalunit_type >= HEVC_NALTYPE_BLA_W_LP) &&
	       (nalunit_type <= HEVC_NALTYPE_BLA_N_LP);
}

static unsigned char bspp_hevc_picture_getnorasl_outputflag
					(enum hevc_nalunittype nalunit_type,
					 struct bspp_hevc_inter_pict_ctx *inter_pict_ctx)
{
	VDEC_ASSERT(inter_pict_ctx);

	if (bspp_hevc_picture_is_idr(nalunit_type) ||
	    bspp_hevc_picture_is_bla(nalunit_type) ||
	    inter_pict_ctx->first_after_eos ||
	    (bspp_hevc_picture_is_cra(nalunit_type) && inter_pict_ctx->seq_pic_count == 1))
		return 1;

	return 0;
}

static unsigned char bspp_hevc_range_extensions_is_enabled
				(struct bspp_hevc_profile_tierlevel *profile_tierlevel)
{
	unsigned char is_enabled;

	is_enabled = profile_tierlevel->general_profile_idc >= 4 ||
		profile_tierlevel->general_profile_compatibility_flag[4];

	return is_enabled;
}

static void bspp_hevc_parse_codec_config(void *hndl_swsr_ctx, unsigned int *unit_count,
					 unsigned int *unit_array_count,
					 unsigned int *delim_length,
					 unsigned int *size_delim_length)
{
	unsigned long long value = 23;

	/*
	 * Set the shift-register up to provide next 23 bytes
	 * without emulation prevention detection.
	 */
	swsr_consume_delim(hndl_swsr_ctx, SWSR_EMPREVENT_NONE, 0, &value);
	/*
	 * Codec config header must be read for size delimited data (HEVC)
	 * to get to the start of each unit.
	 * This parsing follows section 8.3.3.1.2 of ISO/IEC 14496-15:2013.
	 */
	swsr_read_bits(hndl_swsr_ctx, 8 * 4);
	swsr_read_bits(hndl_swsr_ctx, 8 * 4);
	swsr_read_bits(hndl_swsr_ctx, 8 * 4);
	swsr_read_bits(hndl_swsr_ctx, 8 * 4);
	swsr_read_bits(hndl_swsr_ctx, 8 * 4);
	swsr_read_bits(hndl_swsr_ctx, 8);

	*delim_length = ((swsr_read_bits(hndl_swsr_ctx, 8) & 0x3) + 1) * 8;
	*unit_array_count = swsr_read_bits(hndl_swsr_ctx, 8);

	/* Size delimiter is only 2 bytes for HEVC codec configuration. */
	*size_delim_length = 2 * 8;
}

static void bspp_hevc_update_unitcounts(void *hndl_swsr_ctx, unsigned int *unit_count,
					unsigned int *unit_array_count)
{
	if (*unit_array_count != 0) {
		unsigned long long value = 3;

		if (*unit_count == 0) {
			/*
			 * Set the shift-register up to provide next 3 bytes
			 * without emulation prevention detection.
			 */
			swsr_consume_delim(hndl_swsr_ctx, SWSR_EMPREVENT_NONE, 0, &value);

			swsr_read_bits(hndl_swsr_ctx, 8);
			*unit_count = swsr_read_bits(hndl_swsr_ctx, 16);

			(*unit_array_count)--;
			(*unit_count)--;
		}
	}
}

void bspp_hevc_determine_unittype(unsigned char bitstream_unittype,
				  int disable_mvc,
				  enum bspp_unit_type *bspp_unittype)
{
	/* 6 bits for NAL Unit Type in HEVC */
	unsigned char type = (bitstream_unittype >> 1) & 0x3f;

	switch (type) {
	case HEVC_NALTYPE_VPS:
		*bspp_unittype = BSPP_UNIT_VPS;
		break;

	case HEVC_NALTYPE_SPS:
		*bspp_unittype = BSPP_UNIT_SEQUENCE;
		break;

	case HEVC_NALTYPE_PPS:
		*bspp_unittype = BSPP_UNIT_PPS;
		break;

	case HEVC_NALTYPE_TRAIL_N:
	case HEVC_NALTYPE_TRAIL_R:
	case HEVC_NALTYPE_TSA_N:
	case HEVC_NALTYPE_TSA_R:
	case HEVC_NALTYPE_STSA_N:
	case HEVC_NALTYPE_STSA_R:
	case HEVC_NALTYPE_RADL_N:
	case HEVC_NALTYPE_RADL_R:
	case HEVC_NALTYPE_RASL_N:
	case HEVC_NALTYPE_RASL_R:
	case HEVC_NALTYPE_BLA_W_LP:
	case HEVC_NALTYPE_BLA_W_RADL:
	case HEVC_NALTYPE_BLA_N_LP:
	case HEVC_NALTYPE_IDR_W_RADL:
	case HEVC_NALTYPE_IDR_N_LP:
	case HEVC_NALTYPE_CRA:
	case HEVC_NALTYPE_EOS:
		/* Attach EOS to picture data, so it can be detected in FW */
		*bspp_unittype = BSPP_UNIT_PICTURE;
		break;

	case HEVC_NALTYPE_AUD:
	case HEVC_NALTYPE_PREFIX_SEI:
	case HEVC_NALTYPE_SUFFIX_SEI:
	case HEVC_NALTYPE_EOB:
	case HEVC_NALTYPE_FD:
		*bspp_unittype = BSPP_UNIT_NON_PICTURE;
		break;

	default:
		*bspp_unittype = BSPP_UNIT_UNSUPPORTED;
		break;
	}
}

int bspp_hevc_set_parser_config(enum vdec_bstr_format bstr_format,
				struct bspp_vid_std_features *pvidstd_features,
				struct bspp_swsr_ctx *pswsr_ctx,
				struct bspp_parser_callbacks *parser_callbacks,
				struct bspp_inter_pict_data *pinterpict_data)
{
	/* set HEVC parser callbacks. */
	parser_callbacks->parse_unit_cb         = bspp_hevc_unitparser;
	parser_callbacks->release_data_cb       = bspp_hevc_releasedata;
	parser_callbacks->reset_data_cb         = bspp_hevc_resetdata;
	parser_callbacks->parse_codec_config_cb = bspp_hevc_parse_codec_config;
	parser_callbacks->update_unit_counts_cb = bspp_hevc_update_unitcounts;
	parser_callbacks->initialise_parsing_cb = bspp_hevc_initialiseparsing;
	parser_callbacks->finalise_parsing_cb   = bspp_hevc_finaliseparsing;

	/* Set HEVC specific features. */
	pvidstd_features->seq_size = sizeof(struct bspp_hevc_sequ_hdr_info);
	pvidstd_features->uses_vps  = 1;
	pvidstd_features->vps_size = sizeof(struct bspp_hevc_vps);
	pvidstd_features->uses_pps  = 1;
	pvidstd_features->pps_size = sizeof(struct bspp_hevc_pps);

	/* Set HEVC specific shift register config. */
	pswsr_ctx->emulation_prevention = SWSR_EMPREVENT_00000300;

	if (bstr_format == VDEC_BSTRFORMAT_DEMUX_BYTESTREAM ||
	    bstr_format == VDEC_BSTRFORMAT_ELEMENTARY) {
		pswsr_ctx->sr_config.delim_type = SWSR_DELIM_SCP;
		pswsr_ctx->sr_config.delim_length = 3 * 8;
		pswsr_ctx->sr_config.scp_value = 0x000001;
	} else if (bstr_format == VDEC_BSTRFORMAT_DEMUX_SIZEDELIMITED) {
		pswsr_ctx->sr_config.delim_type = SWSR_DELIM_SIZE;
		pswsr_ctx->sr_config.delim_length = 4 * 8;
	} else {
		return IMG_ERROR_NOT_SUPPORTED;
	}

	return 0;
}
