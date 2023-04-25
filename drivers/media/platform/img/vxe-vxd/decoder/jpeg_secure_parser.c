// SPDX-License-Identifier: GPL-2.0
/*
 * h.264 secure data unit parsing API.
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

#include <linux/dma-mapping.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>

#include "bspp_int.h"
#include "jpeg_secure_parser.h"
#include "jpegfw_data.h"
#include "swsr.h"

#define JPEG_MCU_SIZE 8

#define JPEG_MAX_COMPONENTS 4
#define MAX_SETS_HUFFMAN_TABLES 2
#define MAX_QUANT_TABLES 4

#define TABLE_CLASS_DC  0
#define TABLE_CLASS_AC  1
#define TABLE_CLASS_NUM 2

/* Marker Codes */
#define CODE_SOF_BASELINE 0xC0
#define CODE_SOF1         0xC1
#define CODE_SOF2         0xC2
#define CODE_SOF3         0xC3
#define CODE_SOF5         0xC5
#define CODE_SOF6         0xC6
#define CODE_SOF7         0xC7
#define CODE_SOF8         0xC8
#define CODE_SOF9         0xC9
#define CODE_SOF10        0xCA
#define CODE_SOF11        0xCB
#define CODE_SOF13        0xCD
#define CODE_SOF14        0xCE
#define CODE_SOF15        0xCF
#define CODE_DHT          0xC4
#define CODE_RST0         0xD0
#define CODE_RST1         0xD1
#define CODE_RST2         0xD2
#define CODE_RST3         0xD3
#define CODE_RST4         0xD4
#define CODE_RST5         0xD5
#define CODE_RST6         0xD6
#define CODE_RST7         0xD7
#define CODE_SOI          0xD8
#define CODE_EOI          0xD9
#define CODE_SOS          0xDA
#define CODE_DQT          0xDB
#define CODE_DRI          0xDD
#define CODE_APP0         0xE0
#define CODE_APP1         0xE1
#define CODE_APP2         0xE2
#define CODE_APP3         0xE3
#define CODE_APP4         0xE4
#define CODE_APP5         0xE5
#define CODE_APP6         0xE6
#define CODE_APP7         0xE7
#define CODE_APP8         0xE8
#define CODE_APP9         0xE9
#define CODE_APP10        0xEA
#define CODE_APP11        0xEB
#define CODE_APP12        0xEC
#define CODE_APP13        0xED
#define CODE_APP14        0xEE
#define CODE_APP15        0xEF
#define CODE_M_DAC        0xCC
#define CODE_COMMENT      0xFE

enum bspp_exception_handler {
	/* BSPP parse exception handler */
	BSPP_EXCEPTION_HANDLER_NONE = 0x00,
	/* Jump at exception (external use) */
	BSPP_EXCEPTION_HANDLER_JUMP,
	BSPP_EXCEPTION_HANDLER_FORCE32BITS = 0x7FFFFFFFU
};

struct components {
	unsigned char identifier;
	unsigned char horz_factor;
	unsigned char vert_factor;
	unsigned char quant_table;
};

struct jpeg_segment_sof {
	unsigned char precision;
	unsigned short height;
	unsigned short width;
	unsigned char component;
	struct components components[JPEG_VDEC_MAX_COMPONENTS];
};

struct jpeg_segment_header {
	unsigned char type;
	unsigned short payload_size;
};

/*
 * Read bitstream data that may LOOK like SCP
 * (but in fact is regular data and should be read as such)
 * @return 8bits read from the bitstream
 */
static unsigned char bspp_jpeg_readbyte_asdata(void *swsr_ctx)
{
	if (swsr_check_delim_or_eod(swsr_ctx) == SWSR_FOUND_DELIM) {
		swsr_consume_delim(swsr_ctx, SWSR_EMPREVENT_NONE, 8, NULL);
		return 0xFF;
	} else {
		return swsr_read_bits(swsr_ctx, 8);
	}
}

/*
 * Read bitstream data that may LOOK like SCP
 * (but in fact be regular data should be read as such)
 * @return 16bits read from the bitstream
 */
static unsigned short bspp_jpeg_readword_asdata(void *swsr_ctx)
{
	unsigned short byte1 = bspp_jpeg_readbyte_asdata(swsr_ctx);
	unsigned short byte2 = bspp_jpeg_readbyte_asdata(swsr_ctx);

	return (byte1 << 8 | byte2);
}

/*
 * Access regular bitstream data that may LOOK like SCP
 * (but in fact be regular data)
 */
static void bspp_jpeg_consume_asdata(void *swsr_ctx, int len)
{
	while (len > 0) {
		bspp_jpeg_readbyte_asdata(swsr_ctx);
		len--;
	}
}

/*
 * Parse SOF segment
 */
static enum bspp_error_type bspp_jpeg_segment_parse_sof(void *swsr_ctx,
							struct jpeg_segment_sof *sof_header)
{
	unsigned char comp_ind;

	sof_header->precision = swsr_read_bits(swsr_ctx, 8);
	if (sof_header->precision != 8) {
		pr_warn("Sample precision has invalid value %d\n",
			sof_header->precision);
		return BSPP_ERROR_INVALID_VALUE;
	}

	sof_header->height = bspp_jpeg_readword_asdata(swsr_ctx);
	sof_header->width = bspp_jpeg_readword_asdata(swsr_ctx);
	if (sof_header->height < JPEG_MCU_SIZE || sof_header->width < JPEG_MCU_SIZE) {
		pr_warn("Sample X/Y smaller than macroblock\n");
		return BSPP_ERROR_INVALID_VALUE;
	}
	sof_header->component = swsr_read_bits(swsr_ctx, 8);
	if (sof_header->component > JPEG_MAX_COMPONENTS) {
		pr_warn("Number of components (%d) is greater than max allowed\n",
			sof_header->component);
		return BSPP_ERROR_INVALID_VALUE;
	}
	/* parse the component */
	for (comp_ind = 0; comp_ind < sof_header->component; comp_ind++) {
		sof_header->components[comp_ind].identifier = swsr_read_bits(swsr_ctx, 8);
		sof_header->components[comp_ind].horz_factor = swsr_read_bits(swsr_ctx, 4);
		sof_header->components[comp_ind].vert_factor = swsr_read_bits(swsr_ctx, 4);
		sof_header->components[comp_ind].quant_table = swsr_read_bits(swsr_ctx, 8);

		pr_debug("components[%d]=(identifier=%d; horz_factor=%d; vert_factor=%d; quant_table=%d)",
			 comp_ind,
			 sof_header->components[comp_ind].identifier,
			 sof_header->components[comp_ind].horz_factor,
			 sof_header->components[comp_ind].vert_factor,
			 sof_header->components[comp_ind].quant_table);
	}

	return BSPP_ERROR_NONE;
}

/*
 * Seeks to delimeter if we're not already on one
 */
static enum swsr_found bspp_jpeg_tryseek_delimeter(void *swsr_ctx)
{
	enum swsr_found was_delim_or_eod = swsr_check_delim_or_eod(swsr_ctx);

	if (was_delim_or_eod != SWSR_FOUND_DELIM)
		was_delim_or_eod = swsr_seek_delim_or_eod(swsr_ctx);

	return was_delim_or_eod;
}

static enum swsr_found bspp_jpeg_tryconsume_delimeters(void *swsr_ctx)
{
	enum swsr_found is_delim_or_eod = swsr_check_delim_or_eod(swsr_ctx);

	while (is_delim_or_eod == SWSR_FOUND_DELIM) {
		swsr_consume_delim(swsr_ctx, SWSR_EMPREVENT_NONE, 8, NULL);
		is_delim_or_eod = swsr_check_delim_or_eod(swsr_ctx);
	}
	return is_delim_or_eod;
}

static enum swsr_found bspp_jpeg_tryseek_and_consume_delimeters(void *swsr_ctx)
{
	enum swsr_found is_delim_or_eod;

	bspp_jpeg_tryseek_delimeter(swsr_ctx);
	is_delim_or_eod = bspp_jpeg_tryconsume_delimeters(swsr_ctx);
	return is_delim_or_eod;
}

/*
 * Read segment type and size
 * @return IMG_TRUE when header is found,
 * IMG_FALSE if it has to be called again
 */
static unsigned char bspp_jpeg_segment_read_header(void *swsr_ctx,
						   struct bspp_unit_data *unit_data,
						   struct jpeg_segment_header *jpeg_segment_header)
{
	bspp_jpeg_tryconsume_delimeters(swsr_ctx);
	jpeg_segment_header->type = swsr_read_bits(swsr_ctx, 8);

	if (jpeg_segment_header->type != 0)
		pr_debug("NAL=0x%x\n", jpeg_segment_header->type);

	jpeg_segment_header->payload_size = 0;

	switch (jpeg_segment_header->type) {
	case CODE_SOS:
	case CODE_DRI:
	case CODE_SOF_BASELINE:
	case CODE_SOF1:
	case CODE_SOF2:
	case CODE_SOF3:
	case CODE_SOF5:
	case CODE_SOF6:
	case CODE_SOF7:
	case CODE_SOF8:
	case CODE_SOF9:
	case CODE_SOF10:
	case CODE_SOF11:
	case CODE_SOF13:
	case CODE_SOF14:
	case CODE_SOF15:
	case CODE_APP0:
	case CODE_APP1:
	case CODE_APP2:
	case CODE_APP3:
	case CODE_APP4:
	case CODE_APP5:
	case CODE_APP6:
	case CODE_APP7:
	case CODE_APP8:
	case CODE_APP9:
	case CODE_APP10:
	case CODE_APP11:
	case CODE_APP12:
	case CODE_APP13:
	case CODE_APP14:
	case CODE_APP15:
	case CODE_DHT:
	case CODE_DQT:
	case CODE_COMMENT:
	{
		jpeg_segment_header->payload_size =
			bspp_jpeg_readword_asdata(swsr_ctx) - 2;
	}
	break;
	case CODE_EOI:
	case CODE_SOI:
	case CODE_RST0:
	case CODE_RST1:
	case CODE_RST2:
	case CODE_RST3:
	case CODE_RST4:
	case CODE_RST5:
	case CODE_RST6:
	case CODE_RST7:
		/*
		 * jpeg_segment_header->payload_size reset to 0 previously,
		 * so just break.
		 */
		break;
	case 0:
	{
		/*
		 * Emulation prevention is OFF which means that 0 after
		 * 0xff will not be swallowed
		 * and has to be treated as data
		 */
		bspp_jpeg_tryseek_and_consume_delimeters(swsr_ctx);
		return 0;
	}
	default:
	{
		pr_err("BAD NAL=%#x\n", jpeg_segment_header->type);
		unit_data->parse_error |= BSPP_ERROR_UNRECOVERABLE;
	}
	}

	pr_debug("payloadSize=%#x\n", jpeg_segment_header->payload_size);
	return 1;
}

static void bspp_jpeg_calculate_mcus(struct jpeg_segment_sof *data_sof,
				     unsigned char *alignment_width,
				     unsigned char *alignment_height)
{
	unsigned char i;
	unsigned char max_horz_factor = 0;
	unsigned char max_vert_factor = 0;
	unsigned short mcu_width = 0;
	unsigned short mcu_height = 0;

	/* Determine maximum scale factors */
	for (i = 0; i < data_sof->component; i++) {
		unsigned char horz_factor = data_sof->components[i].horz_factor;
		unsigned char vert_factor = data_sof->components[i].vert_factor;

		max_horz_factor = horz_factor > max_horz_factor ? horz_factor : max_horz_factor;
		max_vert_factor = vert_factor > max_vert_factor ? vert_factor : max_vert_factor;
	}
	/*
	 * Alignment we want to have must be:
	 * - mutliple of VDEC_MB_DIMENSION
	 * - at least of the size that will fit whole MCUs
	 */
	*alignment_width =
		VDEC_ALIGN_SIZE((8 * max_horz_factor), VDEC_MB_DIMENSION,
				unsigned int, unsigned int);
	*alignment_height =
		VDEC_ALIGN_SIZE((8 * max_vert_factor), VDEC_MB_DIMENSION,
				unsigned int, unsigned int);

	/* Calculate dimensions in MCUs */
	mcu_width += (data_sof->width + (8 * max_horz_factor) - 1) / (8 * max_horz_factor);
	mcu_height += (data_sof->height + (8 * max_vert_factor) - 1) / (8 * max_vert_factor);

#ifdef DEBUG_DECODER_DRIVER
	pr_info("%s; w=%d; w[MCU]=%d\n", __func__, data_sof->width, mcu_width);
	pr_info("%s; h=%d; h[MCU]=%d\n", __func__, data_sof->height, mcu_height);
#endif
}

static int bspp_jpeg_common_seq_hdr_populate(struct jpeg_segment_sof *sof_header,
					     struct vdec_comsequ_hdrinfo *com_sequ_hdr_info,
					     unsigned char alignment_width,
					     unsigned char alignment_height)
{
	unsigned short i;
	int res;
	struct img_pixfmt_desc format_desc;

	memset(&format_desc, 0, sizeof(struct img_pixfmt_desc));
	memset(com_sequ_hdr_info, 0, sizeof(*com_sequ_hdr_info));

	com_sequ_hdr_info->max_frame_size.width = VDEC_ALIGN_SIZE(sof_header->width,
								  alignment_width,
								  unsigned int, unsigned int);
	com_sequ_hdr_info->max_frame_size.height = VDEC_ALIGN_SIZE(sof_header->height,
								   alignment_height, unsigned int,
								   unsigned int);
	com_sequ_hdr_info->frame_size.width = sof_header->width;
	com_sequ_hdr_info->frame_size.height = sof_header->height;
	com_sequ_hdr_info->orig_display_region.width = sof_header->width;
	com_sequ_hdr_info->orig_display_region.height = sof_header->height;

	com_sequ_hdr_info->pixel_info.bitdepth_y = 8;
	com_sequ_hdr_info->pixel_info.bitdepth_c = 8;
	com_sequ_hdr_info->pixel_info.num_planes = sof_header->component;
	/*  actually we have to set foramt accroding to the following table
	 * H1    V1    H2    V2    H3    V3    J:a:b     h/v
	 * 1     1     1     1     1     1     4:4:4     1/1
	 * 1     2     1     1     1     1     4:4:0     1/2
	 * 1     4     1     1     1     1     4:4:1*     1/4
	 * 1     4     1     2     1     2     4:4:0     1/2
	 * 2     1     1     1     1     1     4:2:2     2/1
	 * 2     2     1     1     1     1     4:2:0     2/2
	 * 2     2     2     1     2     1     4:4:0     1/2
	 * 2     4     1     1     1     1     4:2:1*     2/4
	 * 4     1     1     1     1     1     4:1:1     4/1
	 * 4     1     2     1     2     1     4:2:2     2/1
	 * 4     2     1     1     1     1     4:1:0     4/2
	 * 4     4     2     2     2     2     4:2:0     2/2
	 */
	if (sof_header->component == (JPEG_MAX_COMPONENTS - 1)) {
		com_sequ_hdr_info->pixel_info.chroma_fmt = PIXEL_MULTICHROME;
		if ((sof_header->components[1].horz_factor == 1 &&
		     sof_header->components[1].vert_factor == 1) &&
		    (sof_header->components[2].horz_factor == 1 &&
		     sof_header->components[2].vert_factor == 1)) {
			if (sof_header->components[0].horz_factor == 1 &&
			    sof_header->components[0].vert_factor == 1) {
				com_sequ_hdr_info->pixel_info.chroma_fmt_idc = PIXEL_FORMAT_444;
			} else if (sof_header->components[0].horz_factor == 2) {
				if (sof_header->components[0].vert_factor == 1) {
					com_sequ_hdr_info->pixel_info.chroma_fmt_idc =
						PIXEL_FORMAT_422;
				} else if (sof_header->components[0].vert_factor == 2) {
					com_sequ_hdr_info->pixel_info.chroma_fmt_idc =
						PIXEL_FORMAT_420;
				} else {
					com_sequ_hdr_info->pixel_info.chroma_fmt_idc =
						PIXEL_FORMAT_444;
				}
			} else if ((sof_header->components[0].horz_factor == 4) &&
				   (sof_header->components[0].vert_factor == 1)) {
				com_sequ_hdr_info->pixel_info.chroma_fmt_idc = PIXEL_FORMAT_411;
			} else {
				com_sequ_hdr_info->pixel_info.chroma_fmt_idc = PIXEL_FORMAT_444;
			}
		} else {
			com_sequ_hdr_info->pixel_info.chroma_fmt_idc = PIXEL_FORMAT_444;
		}
	} else {
		com_sequ_hdr_info->pixel_info.chroma_fmt = PIXEL_MONOCHROME;
		com_sequ_hdr_info->pixel_info.chroma_fmt_idc = PIXEL_FORMAT_MONO;
	}

	for (i = 0; (i < sof_header->component) && (i < IMG_MAX_NUM_PLANES); i++) {
		format_desc.planes[i] = 1;
		format_desc.h_numer[i] = sof_header->components[i].horz_factor;
		format_desc.v_numer[i] = sof_header->components[i].vert_factor;
	}

	res = pixel_gen_pixfmt(&com_sequ_hdr_info->pixel_info.pixfmt, &format_desc);
	if (res != 0) {
		pr_err("Failed to generate pixel format.\n");
		return res;
	}

	return 0;
}

static void bspp_jpeg_pict_hdr_populate(struct jpeg_segment_sof *sof_header,
					struct bspp_pict_hdr_info *pict_hdr_info)
{
	memset(pict_hdr_info, 0, sizeof(*pict_hdr_info));

	pict_hdr_info->intra_coded = 1;
	pict_hdr_info->ref = 0;

	pict_hdr_info->coded_frame_size.width = (unsigned int)sof_header->width;
	pict_hdr_info->coded_frame_size.height = (unsigned int)sof_header->height;
	pict_hdr_info->disp_info.enc_disp_region.width = (unsigned int)sof_header->width;
	pict_hdr_info->disp_info.enc_disp_region.height = (unsigned int)sof_header->height;

	pict_hdr_info->pict_aux_data.id = BSPP_INVALID;
	pict_hdr_info->second_pict_aux_data.id = BSPP_INVALID;
	pict_hdr_info->pict_sgm_data.id = BSPP_INVALID;
}

static int bspp_jpeg_parse_picture_unit(void *swsr_ctx,
					struct bspp_unit_data *unit_data)
{
	/* assume we'll be fine */
	unit_data->parse_error = BSPP_ERROR_NONE;

	while ((unit_data->parse_error == BSPP_ERROR_NONE) &&
	       !(unit_data->slice || unit_data->extracted_all_data)) {
		struct jpeg_segment_header segment_header;
		/*
		 *  Try hard to read segment header. The only limit we set here is EOD-
		 *  if it happens, we will get an exception, to stop this madness.
		 */
		while (!bspp_jpeg_segment_read_header(swsr_ctx, unit_data, &segment_header) &&
		       unit_data->parse_error == BSPP_ERROR_NONE)
			;

		switch (segment_header.type) {
		case CODE_SOF1:
		case CODE_SOF2:
		case CODE_SOF3:
		case CODE_SOF5:
		case CODE_SOF6:
		case CODE_SOF8:
		case CODE_SOF9:
		case CODE_SOF10:
		case CODE_SOF11:
		case CODE_SOF13:
		case CODE_SOF14:
		case CODE_SOF15:
		{
			bspp_jpeg_consume_asdata(swsr_ctx, segment_header.payload_size);
			bspp_jpeg_tryseek_delimeter(swsr_ctx);
			unit_data->extracted_all_data = 1;
			unit_data->slice = 1;
			unit_data->parse_error |= BSPP_ERROR_UNSUPPORTED;
			return IMG_ERROR_NOT_SUPPORTED;
		}
		case CODE_SOI:
		{
			/*
			 * Reinitialize context at the beginning of each image
			 */
		}
		break;
		case CODE_EOI:
		{
			/*
			 * Some more frames can be concatenated after SOI,
			 * but we'll discard it for now
			 */
			while (bspp_jpeg_tryseek_and_consume_delimeters(swsr_ctx) != SWSR_FOUND_EOD)
				;
			unit_data->extracted_all_data = 1;
			return 0;
		}
		case CODE_SOF_BASELINE:
		{
			int res;
			unsigned char alignment_width = 0;
			unsigned char alignment_height = 0;
			struct jpeg_segment_sof sof_data;

			struct bspp_sequ_hdr_info *sequ_hdr_info =
				&unit_data->impl_sequ_hdr_info->sequ_hdr_info;

			memset(&sof_data, 0, sizeof(*&sof_data));

			/* SOF is the only segment we are interested in- parse it */
			unit_data->parse_error |= bspp_jpeg_segment_parse_sof(swsr_ctx, &sof_data);
			/*
			 * to correctly allocate size for frame we need to have correct MCUs to
			 * get alignment info
			 */
			bspp_jpeg_calculate_mcus(&sof_data, &alignment_width, &alignment_height);

			/* fill in headers expected by BSPP framework */
			res = bspp_jpeg_common_seq_hdr_populate(&sof_data,
								&sequ_hdr_info->com_sequ_hdr_info,
								alignment_width,
								alignment_height);
			if (res != 0) {
				unit_data->parse_error |= BSPP_ERROR_UNRECOVERABLE;
				return res;
			}

			bspp_jpeg_pict_hdr_populate(&sof_data, unit_data->out.pict_hdr_info);

			/* fill in sequence IDs for header and picture */
			sequ_hdr_info->sequ_hdr_id = BSPP_DEFAULT_SEQUENCE_ID;
			unit_data->pict_sequ_hdr_id = BSPP_DEFAULT_SEQUENCE_ID;

			/* reset SOS fields counter value */
			unit_data->out.pict_hdr_info->sos_count = 0;
		}
		break;
		case CODE_SOS:
		{
			/* increment the SOS fields counter */
			unit_data->out.pict_hdr_info->sos_count++;

			unit_data->slice = 1;
			bspp_jpeg_consume_asdata(swsr_ctx, segment_header.payload_size);
			return 0;
		}
		case CODE_DRI:
			break;
		default:
		{
#ifdef DEBUG_DECODER_DRIVER
			pr_info("Skipping over 0x%x bytes\n", segment_header.payload_size);
#endif
			bspp_jpeg_consume_asdata(swsr_ctx, segment_header.payload_size);
		}
		break;
		}
		/*
		 * After parsing segment we should already be on delimeter.
		 * Consume it, so header parsing can be started.
		 */
		bspp_jpeg_tryseek_and_consume_delimeters(swsr_ctx);
	}
	return 0;
}

int bspp_jpeg_unit_parser(void *swsr_ctx, struct bspp_unit_data *unit_data)
{
	int retval = 0;

	switch (unit_data->unit_type) {
	case BSPP_UNIT_PICTURE:
	{
		retval = bspp_jpeg_parse_picture_unit(swsr_ctx, unit_data);
		unit_data->new_closed_gop = 1;
	}
	break;
	default:
	{
		unit_data->parse_error = BSPP_ERROR_INVALID_VALUE;
	}
	break;
	}

	return retval;
}

int bspp_jpeg_setparser_config(enum vdec_bstr_format bstr_format,
			       struct bspp_vid_std_features *pvidstd_features,
			       struct bspp_swsr_ctx *pswsr_ctx,
			       struct bspp_parser_callbacks *pparser_callbacks,
			       struct bspp_inter_pict_data *pinterpict_data)
{
	/* Set JPEG parser callbacks. */
	pparser_callbacks->parse_unit_cb = bspp_jpeg_unit_parser;

	/* Set JPEG specific features. */
	pvidstd_features->seq_size = sizeof(struct bspp_jpeg_sequ_hdr_info);
	pvidstd_features->uses_vps  = 0;
	pvidstd_features->uses_pps  = 0;

	/* Set JPEG specific shift register config. */
	pswsr_ctx->emulation_prevention = SWSR_EMPREVENT_NONE;
	pswsr_ctx->sr_config.delim_type = SWSR_DELIM_SCP;
	pswsr_ctx->sr_config.delim_length = 8;
	pswsr_ctx->sr_config.scp_value = 0xFF;

	return 0;
}

void bspp_jpeg_determine_unit_type(unsigned char bitstream_unittype,
				   int disable_mvc,
				   enum bspp_unit_type *bspp_unittype)
{
	*bspp_unittype = BSPP_UNIT_PICTURE;
}
