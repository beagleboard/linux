// SPDX-License-Identifier: GPL-2.0
/*
 * VDECDD translation APIs.
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

/* As of now we are defining HAS_H264 */
#define HAS_H264
#define VDEC_USE_PVDEC

#include <linux/types.h>
#include <linux/dma-mapping.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>

#include "fw_interface.h"
#ifdef HAS_H264
#include "h264fw_data.h"
#endif /* HAS_H264 */
#include "hw_control.h"
#include "img_errors.h"
#include "img_msvdx_cmds.h"
#include "img_msvdx_vec_regs.h"
#ifdef VDEC_USE_PVDEC
#include "pvdec_int.h"
#include "img_pvdec_core_regs.h"
#endif
#include "img_video_bus4_mmu_regs.h"
#include "lst.h"
#include "reg_io2.h"
#include "rman_api.h"
#include "translation_api.h"
#include "vdecdd_defs.h"
#include "vdecdd_utils.h"
#include "vdecfw_share.h"
#include "vxd_int.h"
#include "vxd_props.h"

#ifdef HAS_HEVC
#include "hevcfw_data.h"
#include "pvdec_entropy_regs.h"
#include "pvdec_vec_be_regs.h"
#endif

#ifdef HAS_JPEG
#include "jpegfw_data.h"
#endif /* HAS_JPEG */

#define NO_VALUE        0

/*
 * Discontinuity in layout of VEC_VLC_TABLE* registers.
 * Address of VEC_VLC_TABLE_ADDR16 does not immediately follow
 * VEC_VLC_TABLE_ADDR15, see TRM.
 */
#define VEC_VLC_TABLE_ADDR_PT1_SIZE  16 /* in 32-bit words */
#define VEC_VLC_TABLE_ADDR_DISCONT   (VEC_VLC_TABLE_ADDR_PT1_SIZE * \
	PVDECIO_VLC_IDX_ADDR_PARTS)

/*
 * now it can be done by VXD_GetCodecMode
 * Imply standard from OperatingMode.
 * As of now only H264 supported through the file.
 */
#define CODEC_MODE_JPEG     0x0
#define CODEC_MODE_H264         0x1
#define CODEC_MODE_REAL8        0x8
#define CODEC_MODE_REAL9        0x9

/*
 * This enum defines values of ENTDEC_BE_MODE field of VEC_ENTDEC_BE_CONTROL
 * register and ENTDEC_FE_MODE field of VEC_ENTDEC_FE_CONTROL register.
 */
enum decode_mode {
	/* JPEG */
	VDEC_ENTDEC_MODE_JPEG   = 0x0,
	/* H264 (MPEG4/AVC) */
	VDEC_ENTDEC_MODE_H264   = 0x1,
	VDEC_ENTDEC_MODE_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * This has all that it needs to translate a Stream Unit for a picture into a
 * transaction.
 */
static int translation_set_buffer(struct vdecdd_ddpict_buf *picbuf,
				  struct vdecfw_image_buffer *image_buffer)
{
	unsigned int i;

	for (i = 0; i < VDEC_PLANE_MAX; i++) {
		image_buffer->byte_offset[i] =
			(unsigned int)GET_HOST_ADDR(&picbuf->pict_buf->ddbuf_info) +
			picbuf->rend_info.plane_info[i].offset;
		pr_debug("%s image_buffer->byte_offset[%d] = 0x%x\n",
			 __func__, i, image_buffer->byte_offset[i]);
	}
	return IMG_SUCCESS;
}

#ifdef HAS_HEVC
/*
 * @Function              translation_hevc_header
 */
static int translation_hevc_header(struct vdecdd_picture *picture,
				   struct dec_decpict *dec_pict,
				   struct hevcfw_headerdata *header_data)
{
	translation_set_buffer(dec_pict->recon_pict, &header_data->primary);

	if (dec_pict->alt_pict)
		translation_set_buffer(dec_pict->alt_pict, &header_data->alternate);

	VDEC_ASSERT(picture);
	VDEC_ASSERT(picture->pict_res_int);
	VDEC_ASSERT(picture->pict_res_int->mb_param_buf);
	header_data->temporal_outaddr = (unsigned int)GET_HOST_ADDR
					(&picture->pict_res_int->mb_param_buf->ddbuf_info);

	return IMG_SUCCESS;
}
#endif

#ifdef HAS_H264
static int translation_h264header(struct vdecdd_picture *pspicture,
				  struct dec_decpict *dec_pict,
				  struct h264fw_header_data *psheaderdata,
				  struct vdec_str_configdata *psstrconfigdata)
{
	psheaderdata->two_pass_flag = dec_pict->pict_hdr_info->discontinuous_mbs;
	psheaderdata->disable_mvc = psstrconfigdata->disable_mvc;

	/*
	 * As of now commenting the mb params base address as we are not using,
	 * if needed in future please un comment and make the allocation for
	 * pict_res_int.
	 */
	/* Obtain the MB parameter address from the stream unit. */
	if (pspicture->pict_res_int->mb_param_buf) {
		psheaderdata->mbparams_base_address =
		(unsigned int)GET_HOST_ADDR(&pspicture->pict_res_int->mb_param_buf->ddbuf_info);
		psheaderdata->mbparams_size_per_plane =
			pspicture->pict_res_int->mb_param_buf->ddbuf_info.buf_size / 3;
	} else {
		psheaderdata->mbparams_base_address = 0;
		psheaderdata->mbparams_size_per_plane = 0;
	}
	psheaderdata->slicegroupmap_base_address =
		(unsigned int)GET_HOST_ADDR(&dec_pict->cur_pict_dec_res->h264_sgm_buf);

	translation_set_buffer(dec_pict->recon_pict, &psheaderdata->primary);

	if (dec_pict->alt_pict)
		translation_set_buffer(dec_pict->alt_pict, &psheaderdata->alternate);

	/* Signal whether we have PPS for the second field. */
	if (pspicture->dec_pict_aux_info.second_pps_id == BSPP_INVALID)
		psheaderdata->second_pps = 0;
	else
		psheaderdata->second_pps = 1;

	return IMG_SUCCESS;
}
#endif /* HAS_H264 */

#ifdef HAS_JPEG

static int translation_jpegheader(const struct bspp_sequ_hdr_info *seq,
				  const struct dec_decpict *dec_pict,
				  const struct bspp_pict_hdr_info *pict_hdrinfo,
				  struct jpegfw_header_data *header_data)
{
	unsigned int i;

	/* Output picture planes addresses */
	for (i = 0; i < seq->com_sequ_hdr_info.pixel_info.num_planes; i++) {
		header_data->plane_offsets[i] =
			(unsigned int)GET_HOST_ADDR(&dec_pict->recon_pict->pict_buf->ddbuf_info) +
			dec_pict->recon_pict->rend_info.plane_info[i].offset;
	}

	/* copy the expected SOS fields number */
	header_data->hdr_sos_count = pict_hdrinfo->sos_count;

	translation_set_buffer(dec_pict->recon_pict, &header_data->primary);

	return IMG_SUCCESS;
}
#endif /* HAS_JPEG */
/*
 * This function translates host video standard enum (VDEC_eVidStd) into
 * firmware video standard enum (VDECFW_eCodecType);
 */
static int translation_get_codec(enum vdec_vid_std evidstd,
				 enum vdecfw_codectype *pecodec)
{
	enum vdecfw_codectype ecodec = VDEC_CODEC_NONE;
	unsigned int result = IMG_ERROR_NOT_SUPPORTED;

	/* Translate from video standard to firmware codec. */
	switch (evidstd) {
	#ifdef HAS_H264
	case VDEC_STD_H264:
		ecodec = VDECFW_CODEC_H264;
		result = IMG_SUCCESS;
		break;
	#endif /* HAS_H264 */
#ifdef HAS_HEVC
	case VDEC_STD_HEVC:
		ecodec = VDECFW_CODEC_HEVC;
		result = IMG_SUCCESS;
		break;
#endif /* HAS_HEVC */
#ifdef HAS_JPEG
	case VDEC_STD_JPEG:
		ecodec = VDECFW_CODEC_JPEG;
		result = IMG_SUCCESS;
		break;
#endif
	default:
		result = IMG_ERROR_NOT_SUPPORTED;
		break;
	}
	*pecodec = ecodec;
	return result;
}

/*
 * This function is used to obtain buffer for sequence header.
 */
static int translation_get_seqhdr(struct vdecdd_str_unit *psstrunit,
				  struct dec_decpict *psdecpict,
				  unsigned int *puipseqaddr)
{
	/*
	 * ending Sequence info only if its a First Pic of Sequence, or a Start
	 * of Closed GOP
	 */
	if (psstrunit->pict_hdr_info->first_pic_of_sequence || psstrunit->closed_gop) {
		struct vdecdd_ddbuf_mapinfo *ddbuf_map_info;
		/* Get access to map info context */
		int result = rman_get_resource(psstrunit->seq_hdr_info->bufmap_id,
					       VDECDD_BUFMAP_TYPE_ID,
					       (void **)&ddbuf_map_info, NULL);
		VDEC_ASSERT(result == IMG_SUCCESS);
		if (result != IMG_SUCCESS)
			return result;

		*puipseqaddr = GET_HOST_ADDR_OFFSET(&ddbuf_map_info->ddbuf_info,
						    psstrunit->seq_hdr_info->buf_offset);
	} else {
		*puipseqaddr = 0;
	}
	return IMG_SUCCESS;
}

/*
 * This function is used to obtain buffer for picture parameter set.
 */
static int translation_get_ppshdr(struct vdecdd_str_unit *psstrunit,
				  struct dec_decpict *psdecpict,
				  unsigned int *puipppsaddr)
{
	if (psstrunit->pict_hdr_info->pict_aux_data.id != BSPP_INVALID) {
		struct vdecdd_ddbuf_mapinfo *ddbuf_map_info;
		int result;

		VDEC_ASSERT(psstrunit->pict_hdr_info->pict_aux_data.pic_data);
		/* Get access to map info context */
		result = rman_get_resource(psstrunit->pict_hdr_info->pict_aux_data.bufmap_id,
					   VDECDD_BUFMAP_TYPE_ID,
					   (void **)&ddbuf_map_info, NULL);
		VDEC_ASSERT(result == IMG_SUCCESS);

		if (result != IMG_SUCCESS)
			return result;
		*puipppsaddr =
			GET_HOST_ADDR_OFFSET(&ddbuf_map_info->ddbuf_info,
					     psstrunit->pict_hdr_info->pict_aux_data.buf_offset);
	} else {
		*puipppsaddr = 0;
	}
	return IMG_SUCCESS;
}

/*
 * This function is used to obtain buffer for second picture parameter set.
 */
static int translation_getsecond_ppshdr(struct vdecdd_str_unit *psstrunit,
					unsigned int *puisecond_ppshdr)
{
	if (psstrunit->pict_hdr_info->second_pict_aux_data.id !=
		BSPP_INVALID) {
		struct vdecdd_ddbuf_mapinfo *ddbuf_map_info;
		int result;
		void *pic_data =
			psstrunit->pict_hdr_info->second_pict_aux_data.pic_data;

		VDEC_ASSERT(pic_data);
		result = rman_get_resource(psstrunit->pict_hdr_info->second_pict_aux_data.bufmap_id,
					   VDECDD_BUFMAP_TYPE_ID,
					   (void **)&ddbuf_map_info, NULL);
		VDEC_ASSERT(result == IMG_SUCCESS);

		if (result != IMG_SUCCESS)
			return result;

		*puisecond_ppshdr =
			GET_HOST_ADDR_OFFSET
				(&ddbuf_map_info->ddbuf_info,
				 psstrunit->pict_hdr_info->second_pict_aux_data.buf_offset);
	} else {
		*puisecond_ppshdr = 0;
	}
	return IMG_SUCCESS;
}

/*
 * Returns address from which FW should download its shared context.
 */
static unsigned int translation_getctx_loadaddr(struct dec_decpict *psdecpict)
{
	if (psdecpict->prev_pict_dec_res)
		return GET_HOST_ADDR(&psdecpict->prev_pict_dec_res->fw_ctx_buf);

	/*
	 * No previous context exists, using current context leads to
	 * problems on replay so just say to FW to use clean one.
	 * This is NULL as integer to avoid pointer size warnings due
	 * to type casting.
	 */
	return 0;
}

static void translation_setup_std_header
	(struct vdec_str_configdata *str_configdata,
	struct dec_decpict *dec_pict,
	struct vdecdd_str_unit *str_unit, unsigned int *psr_hdrsize,
	struct vdecdd_picture *picture, unsigned int *picture_cmds,
	enum vdecfw_parsermode *parser_mode)
{
	switch (str_configdata->vid_std) {
#ifdef HAS_H264
	case VDEC_STD_H264:
	{
		struct h264fw_header_data *header_data =
			(struct h264fw_header_data *)
			dec_pict->hdr_info->ddbuf_info->cpu_virt;
		*parser_mode = str_unit->pict_hdr_info->parser_mode;

		if (str_unit->pict_hdr_info->parser_mode !=
			VDECFW_SCP_ONLY) {
			pr_warn("VDECFW_SCP_ONLY mode supported in PVDEC FW\n");
		}
		/* Reset header data. */
		memset(header_data, 0, sizeof(*(header_data)));

		/* Prepare active parameter sets. */
		translation_h264header(picture, dec_pict, header_data, str_configdata);

		/* Setup header size in the transaction. */
		*psr_hdrsize = sizeof(struct h264fw_header_data);
		break;
	}
#endif /* HAS_H264 */

#ifdef HAS_HEVC
	case VDEC_STD_HEVC:
	{
		struct hevcfw_headerdata *header_data =
			(struct hevcfw_headerdata *)dec_pict->hdr_info->ddbuf_info->cpu_virt;
		*parser_mode = str_unit->pict_hdr_info->parser_mode;

		/* Reset header data. */
		memset(header_data, 0, sizeof(*header_data));

		/* Prepare active parameter sets. */
		translation_hevc_header(picture, dec_pict, header_data);

		/* Setup header size in the transaction. */
		*psr_hdrsize = sizeof(struct hevcfw_headerdata);
		break;
	}
#endif
#ifdef HAS_JPEG
	case VDEC_STD_JPEG:
	{
		struct jpegfw_header_data *header_data =
			(struct jpegfw_header_data *)dec_pict->hdr_info->ddbuf_info->cpu_virt;
		const struct bspp_sequ_hdr_info *seq = str_unit->seq_hdr_info;
		const struct bspp_pict_hdr_info *pict_hdr_info = str_unit->pict_hdr_info;

		/* Reset header data. */
		memset(header_data, 0, sizeof(*(header_data)));

		/* Prepare active parameter sets. */
		translation_jpegheader(seq, dec_pict, pict_hdr_info, header_data);

		/* Setup header size in the transaction. */
		*psr_hdrsize = sizeof(struct jpegfw_header_data);
		break;
	}
#endif
	default:
		VDEC_ASSERT(NULL == "Unknown standard!");
		*psr_hdrsize = 0;
		break;
	}
}

#define VDEC_INITIAL_DEVA_DMA_CMD_SIZE 3
#define VDEC_SINLGE_DEVA_DMA_CMD_SIZE 2

#ifdef VDEC_USE_PVDEC
/*
 * Creates DEVA bitstream segments command and saves is to control allocation
 * buffer.
 */
static int translation_pvdec_adddma_transfers
	(struct lst_t *decpic_seglist, unsigned int **dma_cmdbuf,
	int cmd_bufsize, struct dec_decpict *psdecpict, int eop)
{
	/*
	 * DEVA's bitstream DMA command is made out of chunks with following
	 * layout ('+' sign is used to mark actual words in command):
	 *
	 * + Bitstream HDR, type unsigned int, consists of:
	 *	- command id (CMD_BITSTREAM_SEGMENTS),
	 *	- number of segments in this chunk,
	 *	- optional CMD_BITSTREAM_SEGMENTS_MORE_FOLLOW_MASK
	 *
	 * + Bitstream total size, type unsigned int,
	 * represents size of all segments in all chunks
	 *
	 * Segments of following type (can repeat up to
	 * CMD_BITSTREAM_SEGMENTS_MINUS1_MASK + 1 times)
	 *
	 *	+ Bitstream segment address, type unsigned int
	 *
	 *	+ Bitstream segment size, type unsigned int
	 *
	 * Subsequent chunks are present when
	 * CMD_BITSTREAM_SEGMENTS_MORE_FOLLOW_MASK flag is set in Bitstream HDR.
	 */
	struct dec_decpict_seg *dec_picseg = (struct dec_decpict_seg *)lst_first(decpic_seglist);
	unsigned int *cmd = *dma_cmdbuf;
	unsigned int *dma_hdr = cmd;
	unsigned int segcount = 0;
	unsigned int bitstream_size = 0;

	/*
	 * Two words for DMA command header (setup later as we need to find out
	 * count of BS segments).
	 */
	cmd += CMD_BITSTREAM_HDR_DW_SIZE;
	cmd_bufsize -= CMD_BITSTREAM_HDR_DW_SIZE;
	if (cmd_bufsize < 0) {
		pr_err("Buffer for DMA command too small.\n");
		return IMG_ERROR_INVALID_PARAMETERS;
	}

	if (!dec_picseg) {
		/* No segments to be send to FW: preparing fake one */
		cmd_bufsize -= VDEC_SINLGE_DEVA_DMA_CMD_SIZE;
		if (cmd_bufsize < 0) {
			pr_err("Buffer for DMA command too small.\n");
			return IMG_ERROR_INVALID_PARAMETERS;
		}
		segcount++;

		/* zeroing bitstream size and bitstream offset */
		*(cmd++) = 0;
		*(cmd++) = 0;
	}

	/* Loop through all bitstream segments */
	while (dec_picseg) {
		if (dec_picseg->bstr_seg && (dec_picseg->bstr_seg->bstr_seg_flag
			& VDECDD_BSSEG_SKIP) == 0) {
			unsigned int result;
			struct vdecdd_ddbuf_mapinfo *ddbuf_map_info;

			segcount++;
			/* Two words for each added bitstream segment */
			cmd_bufsize -= VDEC_SINLGE_DEVA_DMA_CMD_SIZE;
			if (cmd_bufsize < 0) {
				pr_err("Buffer for DMA command too small.\n");
				return IMG_ERROR_INVALID_PARAMETERS;
			}
			/* Insert SCP/SC if needed */
			if (dec_picseg->bstr_seg->bstr_seg_flag &
				VDECDD_BSSEG_INSERTSCP) {
				unsigned int startcode_length =
					psdecpict->start_code_bufinfo->buf_size;

				if (dec_picseg->bstr_seg->bstr_seg_flag &
					VDECDD_BSSEG_INSERT_STARTCODE) {
					unsigned char *start_code =
						psdecpict->start_code_bufinfo->cpu_virt;
					start_code[startcode_length - 1] =
						dec_picseg->bstr_seg->start_code_suffix;
				} else {
					startcode_length -= 1;
				}

				segcount++;
				*(cmd++) = startcode_length;
				bitstream_size += startcode_length;

				*(cmd++) = psdecpict->start_code_bufinfo->dev_virt;

				if (((segcount %
					(CMD_BITSTREAM_SEGMENTS_MINUS1_MASK + 1)) == 0))
					/*
					 * we have reached max number of
					 * bitstream segments for current
					 * command make pui32Cmd point to next
					 * BS command
					 */
					cmd += CMD_BITSTREAM_HDR_DW_SIZE;
			}
			/* Get access to map info context */
			result = rman_get_resource(dec_picseg->bstr_seg->bufmap_id,
						   VDECDD_BUFMAP_TYPE_ID,
						   (void **)&ddbuf_map_info, NULL);
			VDEC_ASSERT(result == IMG_SUCCESS);
			if (result != IMG_SUCCESS)
				return result;

			*(cmd++) = (dec_picseg->bstr_seg->data_size);
			bitstream_size += dec_picseg->bstr_seg->data_size;

			*(cmd++) = ddbuf_map_info->ddbuf_info.dev_virt +
				dec_picseg->bstr_seg->data_byte_offset;

			if (((segcount %
				(CMD_BITSTREAM_SEGMENTS_MINUS1_MASK + 1)) == 0) &&
				(lst_next(dec_picseg)))
				/*
				 * we have reached max number of bitstream
				 * segments for current command make pui32Cmd
				 * point to next BS command
				 */
				cmd += CMD_BITSTREAM_HDR_DW_SIZE;
		}
		dec_picseg = lst_next(dec_picseg);
	}

	if (segcount > CMD_BITSTREAM_SEGMENTS_MAX_NUM) {
		pr_err("Too many bitstream segments to transfer.\n");
		return IMG_ERROR_INVALID_PARAMETERS;
	}

	while (segcount > (CMD_BITSTREAM_SEGMENTS_MINUS1_MASK + 1)) {
		*dma_hdr++ = CMD_BITSTREAM_SEGMENTS |
			CMD_BITSTREAM_SEGMENTS_MORE_FOLLOW_MASK |
			CMD_BITSTREAM_SEGMENTS_MINUS1_MASK;
		*dma_hdr++ = bitstream_size;
		/*
		 * make pui32DmaHdr point to next chunk by skipping bitstream
		 * Segments
		 */
		dma_hdr += (2 * (CMD_BITSTREAM_SEGMENTS_MINUS1_MASK + 1));
		segcount -= (CMD_BITSTREAM_SEGMENTS_MINUS1_MASK + 1);
	}
	*dma_hdr = eop ? CMD_BITSTREAM_EOP_MASK : 0;
	*dma_hdr++ |= CMD_BITSTREAM_SEGMENTS | (segcount - 1);
	*dma_hdr = bitstream_size;

	/*
	 * Let caller know where we finished. Pointer to location one word after
	 * end of our command buffer
	 */
	*dma_cmdbuf = cmd;
	return IMG_SUCCESS;
}

/*
 * Creates DEVA control allocation buffer header.
 */
static void translation_pvdec_ctrl_setuphdr
	(struct ctrl_alloc_header *ctrlalloc_hdr,
	unsigned int *pic_cmds)
{
	ctrlalloc_hdr->cmd_additional_params = CMD_CTRL_ALLOC_HEADER;
	ctrlalloc_hdr->ext_opmode = pic_cmds[VDECFW_CMD_EXT_OP_MODE];
	ctrlalloc_hdr->chroma_strides =
		pic_cmds[VDECFW_CMD_CHROMA_ROW_STRIDE];
	ctrlalloc_hdr->alt_output_addr[0] =
		pic_cmds[VDECFW_CMD_LUMA_ALTERNATIVE_PICTURE_BASE_ADDRESS];
	ctrlalloc_hdr->alt_output_addr[1] =
		pic_cmds[VDECFW_CMD_CHROMA_ALTERNATIVE_PICTURE_BASE_ADDRESS];
	ctrlalloc_hdr->alt_output_flags =
		pic_cmds[VDECFW_CMD_ALTERNATIVE_OUTPUT_PICTURE_ROTATION];
}

/*
 * Creates DEVA VLC DMA command and saves is to control allocation buffer.
 */
static int translation_pvdecsetup_vlcdma
	(struct vidio_ddbufinfo *vlctables_bufinfo,
	unsigned int **dmacmd_buf, unsigned int cmdbuf_size)
{
	unsigned int cmd_dma;
	unsigned int *cmd = *dmacmd_buf;

	/* Check if VLC tables fit in one DMA transfer */
	if (vlctables_bufinfo->buf_size > CMD_DMA_DMA_SIZE_MASK) {
		pr_err("VLC tables won't fit into one DMA transfer!\n");
		return IMG_ERROR_INVALID_PARAMETERS;
	}

	/* Check if we have enough space in control allocation buffer. */
	if (cmdbuf_size < VDEC_SINLGE_DEVA_DMA_CMD_SIZE) {
		pr_err("Buffer for DMA command too small.\n");
		return IMG_ERROR_INVALID_PARAMETERS;
	}

	/* Construct DMA command */
	cmd_dma = CMD_DMA | CMD_DMA_TYPE_VLC_TABLE |
		vlctables_bufinfo->buf_size;

	/* Add command to control allocation */
	*cmd++ = cmd_dma;
	*cmd++ = vlctables_bufinfo->dev_virt;

	/*
	 * Let caller know where we finished. Pointer to location one word after
	 * end of our command buffer
	 */
	*dmacmd_buf = cmd;
	return IMG_SUCCESS;
}

/*
 * Creates DEVA commands for configuring VLC tables and saves them into
 * control allocation buffer.
 */
static int translation_pvdecsetup_vlctables
	(unsigned short vlc_index_data[][3], unsigned int num_tables,
	unsigned int **ctrl_allocbuf, unsigned int ctrl_allocsize,
	unsigned int msvdx_vecoffset)
{
	unsigned int i;
	unsigned int word_count;
	unsigned int reg_val;
	unsigned int *ctrl_allochdr;

	unsigned int *ctrl_alloc = *ctrl_allocbuf;

	/* Calculate the number of words needed for VLC control allocations. */
	/*
	 * 3 words for control allocation headers (we are writing 3 chunks:
	 * addresses, widths, opcodes)
	 */
	unsigned int req_elems = 3 +
		(ALIGN(num_tables, PVDECIO_VLC_IDX_WIDTH_PARTS) /
		PVDECIO_VLC_IDX_WIDTH_PARTS) +
		(ALIGN(num_tables, PVDECIO_VLC_IDX_ADDR_PARTS) /
		PVDECIO_VLC_IDX_ADDR_PARTS) +
		(ALIGN(num_tables, PVDECIO_VLC_IDX_OPCODE_PARTS) /
		PVDECIO_VLC_IDX_OPCODE_PARTS);

	/*
	 * Addresses chunk has to be split in two, if number of tables exceeds
	 * VEC_VLC_TABLE_ADDR_DISCONT (see layout of VEC_VLC_TABLE_ADDR*
	 * registers in TRM)
	 */
	if (num_tables > VEC_VLC_TABLE_ADDR_DISCONT)
		/* We need additional control allocation header */
		req_elems += 1;

	if (ctrl_allocsize < req_elems) {
		pr_err("Buffer for VLC IDX commands too small.\n");
		return IMG_ERROR_INVALID_PARAMETERS;
	}

	/*
	 * Write VLC IDX addresses. Chunks for VEC_VLC_TABLE_ADDR[0-15] and
	 * VEC_VLC_TABLE_ADDR[16-18] registers.
	 */
	ctrl_allochdr = ctrl_alloc++;
	*ctrl_allochdr = CMD_REGISTER_BLOCK | CMD_REGISTER_BLOCK_FLAG_VLC_DATA |
		(MSVDX_VEC_CR_VEC_VLC_TABLE_ADDR0_OFFSET + msvdx_vecoffset);
	/* Reset the word count. */
	word_count = 0;

	/* Process VLC index table. */
	i = 0;
	reg_val = 0;
	while (i < num_tables) {
		VDEC_ASSERT((vlc_index_data[i][PVDECIO_VLC_IDX_ADDR_ID] &
			~PVDECIO_VLC_IDX_ADDR_MASK) == 0);
		/* Pack the addresses into a word. */
		reg_val |= ((vlc_index_data[i][PVDECIO_VLC_IDX_ADDR_ID] &
			PVDECIO_VLC_IDX_ADDR_MASK) <<
			((i % PVDECIO_VLC_IDX_ADDR_PARTS) *
			PVDECIO_VLC_IDX_ADDR_SHIFT));

		/* If we reached the end of VEC_VLC_TABLE_ADDR[0-15] area... */
		if (i == VEC_VLC_TABLE_ADDR_DISCONT) {
			/*
			 * Finalize command header for VEC_VLC_TABLE_ADDR[0-15]
			 * register chunk.
			 */
			*ctrl_allochdr |= word_count << 16;
			/*
			 * Reserve and preset command header for
			 * VEC_VLC_TABLE_ADDR[16-18] register chunk.
			 */
			ctrl_allochdr = ctrl_alloc++;
			*ctrl_allochdr = CMD_REGISTER_BLOCK |
				CMD_REGISTER_BLOCK_FLAG_VLC_DATA |
				(MSVDX_VEC_CR_VEC_VLC_TABLE_ADDR16_OFFSET +
				msvdx_vecoffset);
			/* Reset the word count. */
			word_count = 0;
		}

		/*
		 * If all the addresses are packed in this word or that's the
		 * last iteration
		 */
		if (((i % PVDECIO_VLC_IDX_ADDR_PARTS) ==
			(PVDECIO_VLC_IDX_ADDR_PARTS - 1)) ||
			(i == (num_tables - 1))) {
			/*
			 * Add VLC table address to this chunk and increase
			 * words count.
			 */
			*ctrl_alloc++ = reg_val;
			word_count++;
			/* Reset address value. */
			reg_val = 0;
		}

		i++;
	}

	/*
	 * Finalize the current command header for VEC_VLC_TABLE_ADDR register
	 * chunk.
	 */
	*ctrl_allochdr |= word_count << 16;

	/*
	 * Start new commands chunk for VEC_VLC_TABLE_INITIAL_WIDTH[0-3]
	 * registers.
	 */

	/*
	 * Reserve and preset command header for
	 * VEC_VLC_TABLE_INITIAL_WIDTH[0-3] register chunk.
	 */
	ctrl_allochdr = ctrl_alloc++;
	*ctrl_allochdr = CMD_REGISTER_BLOCK | CMD_REGISTER_BLOCK_FLAG_VLC_DATA |
		(MSVDX_VEC_CR_VEC_VLC_TABLE_INITIAL_WIDTH0_OFFSET +
		msvdx_vecoffset);
	/* Reset the word count. */
	word_count = 0;

	/* Process VLC index table. */
	i = 0;
	reg_val = 0;

	while (i < num_tables) {
		VDEC_ASSERT((vlc_index_data[i][PVDECIO_VLC_IDX_WIDTH_ID] &
			~PVDECIO_VLC_IDX_WIDTH_MASK) == 0);
		/* Pack the widths into a word. */
		reg_val |= ((vlc_index_data[i][PVDECIO_VLC_IDX_WIDTH_ID] &
			PVDECIO_VLC_IDX_WIDTH_MASK) <<
			(i % PVDECIO_VLC_IDX_WIDTH_PARTS) *
			PVDECIO_VLC_IDX_WIDTH_SHIFT);

		/*
		 * If all the widths are packed in this word or that's the last
		 * iteration.
		 */
		if (((i % PVDECIO_VLC_IDX_WIDTH_PARTS) ==
			(PVDECIO_VLC_IDX_WIDTH_PARTS - 1)) ||
			(i == (num_tables - 1))) {
			/*
			 * Add VLC table width to this chunk and increase words
			 * count.
			 */
			*ctrl_alloc++ = reg_val;
			word_count++;
			/* Reset width value. */
			reg_val = 0;
		}
		i++;
	}

	/*
	 * Finalize command header for VEC_VLC_TABLE_INITIAL_WIDTH[0-3] register
	 * chunk.
	 */
	*ctrl_allochdr |= word_count << 16;

	/*
	 * Start new commands chunk for VEC_VLC_TABLE_INITIAL_OPCODE[0-2]
	 * registers.
	 * Reserve and preset command header for
	 * VEC_VLC_TABLE_INITIAL_OPCODE[0-2] register chunk
	 */
	ctrl_allochdr = ctrl_alloc++;
	*ctrl_allochdr = CMD_REGISTER_BLOCK | CMD_REGISTER_BLOCK_FLAG_VLC_DATA |
		(MSVDX_VEC_CR_VEC_VLC_TABLE_INITIAL_OPCODE0_OFFSET +
		msvdx_vecoffset);
	/* Reset the word count. */
	word_count = 0;

	/* Process VLC index table. */
	i = 0;
	reg_val = 0;

	while (i < num_tables) {
		VDEC_ASSERT((vlc_index_data[i][PVDECIO_VLC_IDX_OPCODE_ID] &
			~PVDECIO_VLC_IDX_OPCODE_MASK) == 0);
		/* Pack the opcodes into a word. */
		reg_val |= ((vlc_index_data[i][PVDECIO_VLC_IDX_OPCODE_ID] &
			PVDECIO_VLC_IDX_OPCODE_MASK) <<
			(i % PVDECIO_VLC_IDX_OPCODE_PARTS) *
			PVDECIO_VLC_IDX_OPCODE_SHIFT);

		/*
		 * If all the opcodes are packed in this word or that's the last
		 * iteration.
		 */
		if (((i % PVDECIO_VLC_IDX_OPCODE_PARTS) ==
			(PVDECIO_VLC_IDX_OPCODE_PARTS - 1)) ||
			(i == (num_tables - 1))) {
			/*
			 * Add VLC table opcodes to this chunk and increase
			 * words count.
			 */
			*ctrl_alloc++ = reg_val;
			word_count++;
			/* Reset width value. */
			reg_val = 0;
		}
		i++;
	}

	/*
	 * Finalize command header for VEC_VLC_TABLE_INITIAL_OPCODE[0-2]
	 * register chunk.
	 */
	*ctrl_allochdr |= word_count << 16;

	/* Update caller with current location of control allocation pointer */
	*ctrl_allocbuf = ctrl_alloc;
	return IMG_SUCCESS;
}

/*
 * fills in a rendec command chunk in the command buffer.
 */
static void fill_rendec_chunk(int num, ...)
{
	va_list valist;
	unsigned int i, j = 0;
	unsigned int chunk_word_count = 0;
	unsigned int used_word_count = 0;
	int aux_array_size = 0;
	unsigned int *pic_cmds;
	unsigned int **ctrl_allocbuf;
	unsigned int ctrl_allocsize;
	unsigned int vdmc_cmd_offset;
	unsigned int offset;
	unsigned int *buf;
	/* 5 is the fixed arguments passed to fill_rendec_chunk function */
	enum vdecfw_picture_cmds *aux_array = kmalloc((sizeof(unsigned int) *
			(num - 5)), GFP_KERNEL);
	if (!aux_array)
		return;

	/* initialize valist for num number of arguments */
	va_start(valist, num);

	pic_cmds = va_arg(valist, unsigned int *);
	ctrl_allocbuf = va_arg(valist, unsigned int **);
	ctrl_allocsize = va_arg(valist, unsigned int);
	vdmc_cmd_offset = va_arg(valist, unsigned int);
	offset = va_arg(valist, unsigned int);
	buf = *ctrl_allocbuf;

	aux_array_size = (sizeof(unsigned int) * (num - 5));
	/*
	 * access all the arguments assigned to valist, we have already
	 * read till 5
	 */
	for (i = 6, j = 0; i <= num; i++, j++)
		aux_array[j] = (enum vdecfw_picture_cmds)va_arg(valist, int);

	/* clean memory reserved for valist */
	va_end(valist);
	chunk_word_count = aux_array_size /
		sizeof(enum vdecfw_picture_cmds);
	if ((chunk_word_count + 1) > (ctrl_allocsize - used_word_count)) {
		kfree(aux_array);
		return;
	}
	if ((chunk_word_count & ~(CMD_RENDEC_WORD_COUNT_MASK >>
		CMD_RENDEC_WORD_COUNT_SHIFT)) != 0) {
		kfree(aux_array);
		return;
	}
	used_word_count += chunk_word_count + 1;
	*buf++ = CMD_RENDEC_BLOCK | (chunk_word_count << 16) |
		(vdmc_cmd_offset + offset);

	for (i = 0; i < chunk_word_count; i++)
		*buf++ = pic_cmds[aux_array[i]];

	*ctrl_allocbuf = buf;
	/* free the memory */
	kfree(aux_array);
}

/*
 * Creates DEVA commands for configuring rendec and writes them into control
 * allocation buffer.
 */
static void translation_pvdec_setup_commands(unsigned int *pic_cmds,
					     unsigned int **ctrl_allocbuf,
					     unsigned int ctrl_allocsize,
					     unsigned int vdmc_cmd_offset)
{
	unsigned int codec_mode;

	codec_mode = REGIO_READ_FIELD(pic_cmds[VDECFW_CMD_OPERATING_MODE],
				      MSVDX_CMDS, OPERATING_MODE, CODEC_MODE);

	if (codec_mode != CODEC_MODE_H264)
		/* chunk with cache settings at 0x01C */
		/*
		 * here first argument 6 says there are 6 number of arguments
		 * being passed to fill_rendec_chunk function.
		 */
		fill_rendec_chunk(6, pic_cmds, ctrl_allocbuf, ctrl_allocsize,
				  vdmc_cmd_offset,
				  MSVDX_CMDS_MC_CACHE_CONFIGURATION_OFFSET,
				  VDECFW_CMD_MC_CACHE_CONFIGURATION);

	/* chunk with extended row stride at 0x03C */
	/*
	 * here first argument 6 says there are 6 number of arguments
	 * being passed to fill_rendec_chunk function.
	 */
	fill_rendec_chunk(6, pic_cmds, ctrl_allocbuf, ctrl_allocsize,
			  vdmc_cmd_offset,
			  MSVDX_CMDS_EXTENDED_ROW_STRIDE_OFFSET,
			  VDECFW_CMD_EXTENDED_ROW_STRIDE);

	/* chunk with alternative output control at 0x1B4 */
	/*
	 * here first argument 6 says there are 6 number of arguments
	 * being passed to fill_rendec_chunk function.
	 */
	fill_rendec_chunk(6, pic_cmds, ctrl_allocbuf, ctrl_allocsize,
			  vdmc_cmd_offset,
			  MSVDX_CMDS_ALTERNATIVE_OUTPUT_CONTROL_OFFSET,
			  VDECFW_CMD_ALTERNATIVE_OUTPUT_CONTROL);

	/* scaling chunks */
	if (pic_cmds[VDECFW_CMD_SCALED_DISPLAY_SIZE]) {
		if (codec_mode != CODEC_MODE_REAL8 && codec_mode != CODEC_MODE_REAL9) {
			/*
			 * chunk with scale display size, scale H/V control at
			 * 0x0050
			 */
			/*
			 * here first argument 8 says there are 8 number of
			 * arguments being passed to fill_rendec_chunk function.
			 */
			fill_rendec_chunk(8, pic_cmds, ctrl_allocbuf,
					  ctrl_allocsize, vdmc_cmd_offset,
					  MSVDX_CMDS_SCALED_DISPLAY_SIZE_OFFSET,
					  VDECFW_CMD_SCALED_DISPLAY_SIZE,
					  VDECFW_CMD_HORIZONTAL_SCALE_CONTROL,
					  VDECFW_CMD_VERTICAL_SCALE_CONTROL);

			/* chunk with luma/chorma H/V coeffs at 0x0060 */
			/*
			 * here first argument 21 says there are 21 number of
			 * arguments being passed to fill_rendec_chunk function.
			 */
			fill_rendec_chunk(21, pic_cmds, ctrl_allocbuf,
					  ctrl_allocsize, vdmc_cmd_offset,
					  MSVDX_CMDS_HORIZONTAL_LUMA_COEFFICIENTS_OFFSET,
					  VDECFW_CMD_HORIZONTAL_LUMA_COEFFICIENTS_0,
					  VDECFW_CMD_HORIZONTAL_LUMA_COEFFICIENTS_1,
					  VDECFW_CMD_HORIZONTAL_LUMA_COEFFICIENTS_2,
					  VDECFW_CMD_HORIZONTAL_LUMA_COEFFICIENTS_3,
					  VDECFW_CMD_VERTICAL_LUMA_COEFFICIENTS_0,
					  VDECFW_CMD_VERTICAL_LUMA_COEFFICIENTS_1,
					  VDECFW_CMD_VERTICAL_LUMA_COEFFICIENTS_2,
					  VDECFW_CMD_VERTICAL_LUMA_COEFFICIENTS_3,
					  VDECFW_CMD_HORIZONTAL_CHROMA_COEFFICIENTS_0,
					  VDECFW_CMD_HORIZONTAL_CHROMA_COEFFICIENTS_1,
					  VDECFW_CMD_HORIZONTAL_CHROMA_COEFFICIENTS_2,
					  VDECFW_CMD_HORIZONTAL_CHROMA_COEFFICIENTS_3,
					  VDECFW_CMD_VERTICAL_CHROMA_COEFFICIENTS_0,
					  VDECFW_CMD_VERTICAL_CHROMA_COEFFICIENTS_1,
					  VDECFW_CMD_VERTICAL_CHROMA_COEFFICIENTS_2,
					  VDECFW_CMD_VERTICAL_CHROMA_COEFFICIENTS_3);

			/*
			 * chunk with scale output size, scale H/V chroma at
			 * 0x01B8
			 */
			/*
			 * here first argument 8 says there are 8 number of
			 * arguments being passed to fill_rendec_chunk function.
			 */
			fill_rendec_chunk(8, pic_cmds, ctrl_allocbuf,
					  ctrl_allocsize, vdmc_cmd_offset,
					  MSVDX_CMDS_SCALE_OUTPUT_SIZE_OFFSET,
					  VDECFW_CMD_SCALE_OUTPUT_SIZE,
					  VDECFW_CMD_SCALE_HORIZONTAL_CHROMA,
					  VDECFW_CMD_SCALE_VERTICAL_CHROMA);
		}
	}
}

#ifdef HAS_HEVC
/*
 * @Function		translation_pvdec_setup_pvdec_commands
 */
static int translation_pvdec_setup_pvdec_commands(struct vdecdd_picture *picture,
						  struct dec_decpict *dec_pict,
						  struct vdecdd_str_unit *str_unit,
						  struct decoder_regsoffsets *regs_offsets,
						  unsigned int **ctrl_allocbuf,
						  unsigned int ctrl_alloc_size,
						  unsigned int *mem_to_reg_host_part,
						  unsigned int *pict_cmds)
{
	const unsigned int genc_buf_cnt = 4;
	/* We have two chunks: for GENC buffers addresses and sizes*/
	const unsigned int genc_conf_items = 2;
	const unsigned int pipe = 0xf << 16; /* Instruct H/W to write to current pipe */
	/* We need to configure address and size of each GENC buffer */
	const unsigned int genc_words_cnt = genc_buf_cnt * genc_conf_items;
	struct vdecdd_ddbuf_mapinfo **genc_buffers =
		picture->pict_res_int->seq_resint->genc_buffers;
	unsigned int memto_reg_used;  /* in bytes */
	unsigned int i;
	unsigned int *ctrl_alloc = *ctrl_allocbuf;
	unsigned int *mem_to_reg = (unsigned int *)dec_pict->pvdec_info->ddbuf_info->cpu_virt;
	unsigned int reg = 0;

	if (ctrl_alloc_size < genc_words_cnt + genc_conf_items) {
		pr_err("Buffer for GENC config too small.");
		return IMG_ERROR_INVALID_PARAMETERS;
	}

	/* Insert command header for GENC buffers sizes */
	*ctrl_alloc++ = CMD_REGISTER_BLOCK | (genc_buf_cnt << 16) |
		(PVDEC_ENTROPY_CR_GENC_BUFFER_SIZE_OFFSET + regs_offsets->entropy_offset);
	for (i = 0; i < genc_buf_cnt; i++)
		*ctrl_alloc++ = genc_buffers[i]->ddbuf_info.buf_size;

	/* Insert command header for GENC buffers addresses */
	*ctrl_alloc++ = CMD_REGISTER_BLOCK | (genc_buf_cnt << 16) |
		(PVDEC_ENTROPY_CR_GENC_BUFFER_BASE_ADDRESS_OFFSET + regs_offsets->entropy_offset);
	for (i = 0; i < genc_buf_cnt; i++)
		*ctrl_alloc++ = genc_buffers[i]->ddbuf_info.dev_virt;

	/* Insert GENC fragment buffer address */
	*ctrl_alloc++ = CMD_REGISTER_BLOCK | (1 << 16) |
		(PVDEC_ENTROPY_CR_GENC_FRAGMENT_BASE_ADDRESS_OFFSET + regs_offsets->entropy_offset);
	*ctrl_alloc++ = picture->pict_res_int->genc_fragment_buf->ddbuf_info.dev_virt;

	/* Return current location in control allocation buffer to caller */
	*ctrl_allocbuf = ctrl_alloc;

	reg = 0;
	REGIO_WRITE_FIELD_LITE
		(reg,
		 MSVDX_CMDS, PVDEC_DISPLAY_PICTURE_SIZE, PVDEC_DISPLAY_PICTURE_WIDTH_MIN1,
		 str_unit->pict_hdr_info->coded_frame_size.width - 1, unsigned int);
	REGIO_WRITE_FIELD_LITE
		(reg,
		 MSVDX_CMDS, PVDEC_DISPLAY_PICTURE_SIZE, PVDEC_DISPLAY_PICTURE_HEIGHT_MIN1,
		 str_unit->pict_hdr_info->coded_frame_size.height - 1, unsigned int);

	/*
	 * Pvdec operating mode needs to be submitted before any other commands.
	 * This will be set in FW. Make sure it's the first command in Mem2Reg buffer.
	 */
	VDEC_ASSERT((unsigned int *)dec_pict->pvdec_info->ddbuf_info->cpu_virt == mem_to_reg);

	*mem_to_reg++ = pipe |
		(MSVDX_CMDS_PVDEC_OPERATING_MODE_OFFSET + regs_offsets->vdmc_cmd_offset);
	*mem_to_reg++ = 0x0; /* has to be updated in the F/W */

	*mem_to_reg++ = pipe |
		(MSVDX_CMDS_MC_CACHE_CONFIGURATION_OFFSET + regs_offsets->vdmc_cmd_offset);
	*mem_to_reg++ = 0x0; /* has to be updated in the F/W */

	*mem_to_reg++ = pipe |
		(MSVDX_CMDS_PVDEC_DISPLAY_PICTURE_SIZE_OFFSET + regs_offsets->vdmc_cmd_offset);
	*mem_to_reg++ = reg;

	*mem_to_reg++ = pipe |
		(MSVDX_CMDS_PVDEC_CODED_PICTURE_SIZE_OFFSET + regs_offsets->vdmc_cmd_offset);
	*mem_to_reg++ = reg;

	/* scaling configuration */
	if (pict_cmds[VDECFW_CMD_SCALED_DISPLAY_SIZE]) {
		*mem_to_reg++ = pipe |
			(MSVDX_CMDS_PVDEC_SCALED_DISPLAY_SIZE_OFFSET +
			regs_offsets->vdmc_cmd_offset);
		*mem_to_reg++ = pict_cmds[VDECFW_CMD_SCALED_DISPLAY_SIZE];

		*mem_to_reg++ = pipe |
			(MSVDX_CMDS_HORIZONTAL_SCALE_CONTROL_OFFSET +
			 regs_offsets->vdmc_cmd_offset);
		*mem_to_reg++ = pict_cmds[VDECFW_CMD_HORIZONTAL_SCALE_CONTROL];
		*mem_to_reg++ = pipe |
			(MSVDX_CMDS_VERTICAL_SCALE_CONTROL_OFFSET + regs_offsets->vdmc_cmd_offset);
		*mem_to_reg++ = pict_cmds[VDECFW_CMD_VERTICAL_SCALE_CONTROL];

		*mem_to_reg++ = pipe |
			(MSVDX_CMDS_SCALE_OUTPUT_SIZE_OFFSET + regs_offsets->vdmc_cmd_offset);
		*mem_to_reg++ = pict_cmds[VDECFW_CMD_SCALE_OUTPUT_SIZE];

		*mem_to_reg++ = pipe |
			(MSVDX_CMDS_SCALE_HORIZONTAL_CHROMA_OFFSET + regs_offsets->vdmc_cmd_offset);
		*mem_to_reg++ = pict_cmds[VDECFW_CMD_SCALE_HORIZONTAL_CHROMA];
		*mem_to_reg++ = pipe |
			(MSVDX_CMDS_SCALE_VERTICAL_CHROMA_OFFSET + regs_offsets->vdmc_cmd_offset);
		*mem_to_reg++ = pict_cmds[VDECFW_CMD_SCALE_VERTICAL_CHROMA];

		*mem_to_reg++ = pipe |
			(MSVDX_CMDS_HORIZONTAL_LUMA_COEFFICIENTS_OFFSET +
			regs_offsets->vdmc_cmd_offset);
		*mem_to_reg++ = pict_cmds[VDECFW_CMD_HORIZONTAL_LUMA_COEFFICIENTS_0];
		*mem_to_reg++ = pipe |
			(4 + MSVDX_CMDS_HORIZONTAL_LUMA_COEFFICIENTS_OFFSET +
			regs_offsets->vdmc_cmd_offset);
		*mem_to_reg++ = pict_cmds[VDECFW_CMD_HORIZONTAL_LUMA_COEFFICIENTS_1];
		*mem_to_reg++ = pipe |
			(8 + MSVDX_CMDS_HORIZONTAL_LUMA_COEFFICIENTS_OFFSET +
			regs_offsets->vdmc_cmd_offset);
		*mem_to_reg++ = pict_cmds[VDECFW_CMD_HORIZONTAL_LUMA_COEFFICIENTS_2];
		*mem_to_reg++ = pipe |
			(12 + MSVDX_CMDS_HORIZONTAL_LUMA_COEFFICIENTS_OFFSET +
			regs_offsets->vdmc_cmd_offset);
		*mem_to_reg++ = pict_cmds[VDECFW_CMD_HORIZONTAL_LUMA_COEFFICIENTS_3];

		*mem_to_reg++ = pipe |
			(MSVDX_CMDS_VERTICAL_LUMA_COEFFICIENTS_OFFSET +
			regs_offsets->vdmc_cmd_offset);
		*mem_to_reg++ = pict_cmds[VDECFW_CMD_VERTICAL_LUMA_COEFFICIENTS_0];
		*mem_to_reg++ = pipe |
			(4 + MSVDX_CMDS_VERTICAL_LUMA_COEFFICIENTS_OFFSET +
			regs_offsets->vdmc_cmd_offset);
		*mem_to_reg++ = pict_cmds[VDECFW_CMD_VERTICAL_LUMA_COEFFICIENTS_1];
		*mem_to_reg++ = pipe |
			(8 + MSVDX_CMDS_VERTICAL_LUMA_COEFFICIENTS_OFFSET +
			regs_offsets->vdmc_cmd_offset);
		*mem_to_reg++ = pict_cmds[VDECFW_CMD_VERTICAL_LUMA_COEFFICIENTS_2];
		*mem_to_reg++ = pipe |
			(12 + MSVDX_CMDS_VERTICAL_LUMA_COEFFICIENTS_OFFSET +
			regs_offsets->vdmc_cmd_offset);
		*mem_to_reg++ = pict_cmds[VDECFW_CMD_VERTICAL_LUMA_COEFFICIENTS_3];

		*mem_to_reg++ = pipe |
			(MSVDX_CMDS_HORIZONTAL_CHROMA_COEFFICIENTS_OFFSET +
			regs_offsets->vdmc_cmd_offset);
		*mem_to_reg++ = pict_cmds[VDECFW_CMD_HORIZONTAL_CHROMA_COEFFICIENTS_0];
		*mem_to_reg++ = pipe |
			(4 + MSVDX_CMDS_HORIZONTAL_CHROMA_COEFFICIENTS_OFFSET +
			regs_offsets->vdmc_cmd_offset);
		*mem_to_reg++ = pict_cmds[VDECFW_CMD_HORIZONTAL_CHROMA_COEFFICIENTS_1];
		*mem_to_reg++ = pipe |
			(8 + MSVDX_CMDS_HORIZONTAL_CHROMA_COEFFICIENTS_OFFSET +
			regs_offsets->vdmc_cmd_offset);
		*mem_to_reg++ = pict_cmds[VDECFW_CMD_HORIZONTAL_CHROMA_COEFFICIENTS_2];
		*mem_to_reg++ = pipe |
			(12 + MSVDX_CMDS_HORIZONTAL_CHROMA_COEFFICIENTS_OFFSET +
			regs_offsets->vdmc_cmd_offset);
		*mem_to_reg++ = pict_cmds[VDECFW_CMD_HORIZONTAL_CHROMA_COEFFICIENTS_3];

		*mem_to_reg++ = pipe |
			(MSVDX_CMDS_VERTICAL_CHROMA_COEFFICIENTS_OFFSET +
			regs_offsets->vdmc_cmd_offset);
		*mem_to_reg++ = pict_cmds[VDECFW_CMD_VERTICAL_CHROMA_COEFFICIENTS_0];
		*mem_to_reg++ = pipe |
			(4 + MSVDX_CMDS_VERTICAL_CHROMA_COEFFICIENTS_OFFSET +
			regs_offsets->vdmc_cmd_offset);
		*mem_to_reg++ = pict_cmds[VDECFW_CMD_VERTICAL_CHROMA_COEFFICIENTS_1];
		*mem_to_reg++ = pipe |
			(8 + MSVDX_CMDS_VERTICAL_CHROMA_COEFFICIENTS_OFFSET +
			regs_offsets->vdmc_cmd_offset);
		*mem_to_reg++ = pict_cmds[VDECFW_CMD_VERTICAL_CHROMA_COEFFICIENTS_2];
		*mem_to_reg++ = pipe |
			(12 + MSVDX_CMDS_VERTICAL_CHROMA_COEFFICIENTS_OFFSET +
			regs_offsets->vdmc_cmd_offset);
		*mem_to_reg++ = pict_cmds[VDECFW_CMD_VERTICAL_CHROMA_COEFFICIENTS_3];
	}

	*mem_to_reg++ = pipe |
		(MSVDX_CMDS_EXTENDED_ROW_STRIDE_OFFSET + regs_offsets->vdmc_cmd_offset);
	*mem_to_reg++ = pict_cmds[VDECFW_CMD_EXTENDED_ROW_STRIDE];

	*mem_to_reg++ = pipe |
		(MSVDX_CMDS_ALTERNATIVE_OUTPUT_CONTROL_OFFSET + regs_offsets->vdmc_cmd_offset);
	*mem_to_reg++ = pict_cmds[VDECFW_CMD_ALTERNATIVE_OUTPUT_CONTROL];

	*mem_to_reg++ = pipe |
		(MSVDX_CMDS_ALTERNATIVE_OUTPUT_PICTURE_ROTATION_OFFSET +
		regs_offsets->vdmc_cmd_offset);
	*mem_to_reg++ = pict_cmds[VDECFW_CMD_ALTERNATIVE_OUTPUT_PICTURE_ROTATION];

	*mem_to_reg++ = pipe |
		(MSVDX_CMDS_CHROMA_ROW_STRIDE_OFFSET + regs_offsets->vdmc_cmd_offset);
	*mem_to_reg++ = pict_cmds[VDECFW_CMD_CHROMA_ROW_STRIDE];

	/* Setup MEM_TO_REG buffer */
	for (i = 0; i < genc_buf_cnt; i++) {
		*mem_to_reg++ = pipe | (PVDEC_VEC_BE_CR_GENC_BUFFER_SIZE_OFFSET +
			regs_offsets->vec_be_regs_offset + i * sizeof(unsigned int));
		*mem_to_reg++ = genc_buffers[i]->ddbuf_info.buf_size;
		*mem_to_reg++ = pipe | (PVDEC_VEC_BE_CR_GENC_BUFFER_BASE_ADDRESS_OFFSET +
			regs_offsets->vec_be_regs_offset + i * sizeof(unsigned int));
		*mem_to_reg++ = genc_buffers[i]->ddbuf_info.dev_virt;
	}

	*mem_to_reg++ = pipe |
		(PVDEC_VEC_BE_CR_GENC_FRAGMENT_BASE_ADDRESS_OFFSET +
		regs_offsets->vec_be_regs_offset);
	*mem_to_reg++ = picture->pict_res_int->genc_fragment_buf->ddbuf_info.dev_virt;

	*mem_to_reg++ = pipe |
		(PVDEC_VEC_BE_CR_ABOVE_PARAM_BASE_ADDRESS_OFFSET +
		regs_offsets->vec_be_regs_offset);

	*mem_to_reg++ = dec_pict->pvdec_info->ddbuf_info->dev_virt +
		MEM_TO_REG_BUF_SIZE + SLICE_PARAMS_BUF_SIZE;

	*mem_to_reg++ = pipe |
		(MSVDX_CMDS_LUMA_RECONSTRUCTED_PICTURE_BASE_ADDRESSES_OFFSET +
		regs_offsets->vdmc_cmd_offset);
	*mem_to_reg++ = pict_cmds[VDECFW_CMD_LUMA_RECONSTRUCTED_PICTURE_BASE_ADDRESS];

	*mem_to_reg++ = pipe |
		(MSVDX_CMDS_CHROMA_RECONSTRUCTED_PICTURE_BASE_ADDRESSES_OFFSET +
		regs_offsets->vdmc_cmd_offset);
	*mem_to_reg++ = pict_cmds[VDECFW_CMD_CHROMA_RECONSTRUCTED_PICTURE_BASE_ADDRESS];

	/* alternative picture configuration */
	if (dec_pict->alt_pict) {
		*mem_to_reg++ = pipe |
			(MSVDX_CMDS_VC1_LUMA_RANGE_MAPPING_BASE_ADDRESS_OFFSET +
			regs_offsets->vdmc_cmd_offset);
		*mem_to_reg++ = pict_cmds[VDECFW_CMD_LUMA_ALTERNATIVE_PICTURE_BASE_ADDRESS];

		*mem_to_reg++ = pipe |
			(MSVDX_CMDS_VC1_CHROMA_RANGE_MAPPING_BASE_ADDRESS_OFFSET +
			regs_offsets->vdmc_cmd_offset);
		*mem_to_reg++ = pict_cmds[VDECFW_CMD_CHROMA_ALTERNATIVE_PICTURE_BASE_ADDRESS];
	}

	*mem_to_reg++ = pipe |
		(MSVDX_CMDS_AUX_LINE_BUFFER_BASE_ADDRESS_OFFSET + regs_offsets->vdmc_cmd_offset);
	*mem_to_reg++ = pict_cmds[VDECFW_CMD_AUX_LINE_BUFFER_BASE_ADDRESS];

	*mem_to_reg++ = pipe |
		(MSVDX_CMDS_INTRA_BUFFER_BASE_ADDRESS_OFFSET + regs_offsets->vdmc_cmd_offset);
	*mem_to_reg++ = pict_cmds[VDECFW_CMD_INTRA_BUFFER_BASE_ADDRESS];

	/* Make sure we fit in buffer */
	memto_reg_used = (unsigned long)mem_to_reg -
		(unsigned long)dec_pict->pvdec_info->ddbuf_info->cpu_virt;

	VDEC_ASSERT(memto_reg_used < MEM_TO_REG_BUF_SIZE);

	*mem_to_reg_host_part = memto_reg_used / sizeof(unsigned int);

	return IMG_SUCCESS;
}
#endif

/*
 * Creates DEVA commands for configuring rendec and writes them into control
 * allocation buffer.
 */
static int translation_pvdecsetup_vdecext
	(struct vdec_ext_cmd *vdec_ext,
	struct dec_decpict *dec_pict, unsigned int *pic_cmds,
	struct vdecdd_str_unit *str_unit, enum vdec_vid_std vid_std,
	enum vdecfw_parsermode parser_mode)
{
	int result;
	unsigned int trans_id = dec_pict->transaction_id;

	VDEC_ASSERT(dec_pict->recon_pict);

	vdec_ext->cmd = CMD_VDEC_EXT;
	vdec_ext->trans_id = trans_id;

	result = translation_get_seqhdr(str_unit, dec_pict, &vdec_ext->seq_addr);
	VDEC_ASSERT(result == IMG_SUCCESS);
	if (result != IMG_SUCCESS)
		return result;

	result = translation_get_ppshdr(str_unit, dec_pict, &vdec_ext->pps_addr);
	VDEC_ASSERT(result == IMG_SUCCESS);
	if (result != IMG_SUCCESS)
		return result;

	result = translation_getsecond_ppshdr(str_unit, &vdec_ext->pps_2addr);
	if (result != IMG_SUCCESS)
		return result;

	vdec_ext->hdr_addr = GET_HOST_ADDR(dec_pict->hdr_info->ddbuf_info);

	vdec_ext->ctx_load_addr = translation_getctx_loadaddr(dec_pict);
	vdec_ext->ctx_save_addr = GET_HOST_ADDR(&dec_pict->cur_pict_dec_res->fw_ctx_buf);
	vdec_ext->buf_ctrl_addr = GET_HOST_ADDR(&dec_pict->pict_ref_res->fw_ctrlbuf);
	if (dec_pict->prev_pict_dec_res) {
		/*
		 * Copy the previous firmware context to the current one in case
		 * picture management fails in firmware.
		 */
		memcpy(dec_pict->cur_pict_dec_res->fw_ctx_buf.cpu_virt,
		       dec_pict->prev_pict_dec_res->fw_ctx_buf.cpu_virt,
		       dec_pict->prev_pict_dec_res->fw_ctx_buf.buf_size);
	}

	vdec_ext->last_luma_recon =
		pic_cmds[VDECFW_CMD_LUMA_RECONSTRUCTED_PICTURE_BASE_ADDRESS];
	vdec_ext->last_chroma_recon =
		pic_cmds[VDECFW_CMD_CHROMA_RECONSTRUCTED_PICTURE_BASE_ADDRESS];

	vdec_ext->luma_err_base =
		pic_cmds[VDECFW_CMD_LUMA_ERROR_PICTURE_BASE_ADDRESS];
	vdec_ext->chroma_err_base =
		pic_cmds[VDECFW_CMD_CHROMA_ERROR_PICTURE_BASE_ADDRESS];

	vdec_ext->scaled_display_size =
		pic_cmds[VDECFW_CMD_SCALED_DISPLAY_SIZE];
	vdec_ext->horz_scale_control =
		pic_cmds[VDECFW_CMD_HORIZONTAL_SCALE_CONTROL];
	vdec_ext->vert_scale_control =
		pic_cmds[VDECFW_CMD_VERTICAL_SCALE_CONTROL];
	vdec_ext->scale_output_size = pic_cmds[VDECFW_CMD_SCALE_OUTPUT_SIZE];

	vdec_ext->intra_buf_base_addr =
		pic_cmds[VDECFW_CMD_INTRA_BUFFER_BASE_ADDRESS];
	vdec_ext->intra_buf_size_per_pipe =
		pic_cmds[VDECFW_CMD_INTRA_BUFFER_SIZE_PER_PIPE];
	vdec_ext->intra_buf_size_per_plane =
		pic_cmds[VDECFW_CMD_INTRA_BUFFER_PLANE_SIZE];
	vdec_ext->aux_line_buffer_base_addr =
		pic_cmds[VDECFW_CMD_AUX_LINE_BUFFER_BASE_ADDRESS];
	vdec_ext->aux_line_buf_size_per_pipe =
		pic_cmds[VDECFW_CMD_AUX_LINE_BUFFER_SIZE_PER_PIPE];
	vdec_ext->alt_output_pict_rotation =
		pic_cmds[VDECFW_CMD_ALTERNATIVE_OUTPUT_PICTURE_ROTATION];
	vdec_ext->chroma2reconstructed_addr =
		pic_cmds[VDECFW_CMD_CHROMA2_RECONSTRUCTED_PICTURE_BASE_ADDRESS];
	vdec_ext->luma_alt_addr =
		pic_cmds[VDECFW_CMD_LUMA_ALTERNATIVE_PICTURE_BASE_ADDRESS];
	vdec_ext->chroma_alt_addr =
		pic_cmds[VDECFW_CMD_CHROMA_ALTERNATIVE_PICTURE_BASE_ADDRESS];
	vdec_ext->chroma2alt_addr =
		pic_cmds[VDECFW_CMD_CHROMA2_ALTERNATIVE_PICTURE_BASE_ADDRESS];

	if (vid_std == VDEC_STD_VC1) {
		struct vidio_ddbufinfo *vlc_idx_tables_bufinfo =
			dec_pict->vlc_idx_tables_bufinfo;
		struct vidio_ddbufinfo *vlc_tables_bufinfo =
			dec_pict->vlc_tables_bufinfo;

		vdec_ext->vlc_idx_table_size = vlc_idx_tables_bufinfo->buf_size;
		vdec_ext->vlc_idx_table_addr = vlc_idx_tables_bufinfo->buf_size;
		vdec_ext->vlc_tables_size = vlc_tables_bufinfo->buf_size;
		vdec_ext->vlc_tables_size = vlc_tables_bufinfo->buf_size;
	} else {
		vdec_ext->vlc_idx_table_size = 0;
		vdec_ext->vlc_idx_table_addr = 0;
		vdec_ext->vlc_tables_size = 0;
		vdec_ext->vlc_tables_size = 0;
	}

	vdec_ext->display_picture_size = pic_cmds[VDECFW_CMD_DISPLAY_PICTURE];
	vdec_ext->parser_mode = parser_mode;

	/* miscellaneous flags */
	vdec_ext->is_chromainterleaved =
		REGIO_READ_FIELD(pic_cmds[VDECFW_CMD_OPERATING_MODE], MSVDX_CMDS, OPERATING_MODE,
				 CHROMA_INTERLEAVED);
	vdec_ext->is_discontinuousmbs =
		dec_pict->pict_hdr_info->discontinuous_mbs;

#ifdef HAS_HEVC
	if (dec_pict->pvdec_info) {
		vdec_ext->mem_to_reg_addr = dec_pict->pvdec_info->ddbuf_info->dev_virt;
		vdec_ext->slice_params_addr = dec_pict->pvdec_info->ddbuf_info->dev_virt +
			MEM_TO_REG_BUF_SIZE;
		vdec_ext->slice_params_size = SLICE_PARAMS_BUF_SIZE;
	}
	if (vid_std == VDEC_STD_HEVC) {
		struct vdecdd_picture *picture = (struct vdecdd_picture *)str_unit->dd_pict_data;

		VDEC_ASSERT(picture);
		/* 10-bit packed output format indicator */
		vdec_ext->is_packedformat = picture->op_config.pixel_info.mem_pkg ==
			PIXEL_BIT10_MP ? 1 : 0;
	}
#endif
	return IMG_SUCCESS;
}

/*
 * NOTE :
 * translation_configure_tiling is not supported as of now.
 */
int translation_ctrl_alloc_prepare(struct vdec_str_configdata *pstr_config_data,
				   struct vdecdd_str_unit *str_unit,
				   struct dec_decpict *dec_pict,
				   const struct vxd_coreprops *core_props,
				   struct decoder_regsoffsets *regs_offset)
{
	int result;
	unsigned int *cmd_buf;
	unsigned int hdr_size = 0;
	unsigned int pict_cmds[VDECFW_CMD_MAX];
	enum vdecfw_codectype codec;
	struct vxd_buffers buffers;
	struct vdec_ext_cmd *vdec_ext;
	enum vdecfw_parsermode parser_mode = VDECFW_SCP_ONLY;
	struct vidio_ddbufinfo *batch_msgbuf_info =
		dec_pict->batch_msginfo->ddbuf_info;
	struct lst_t *decpic_seg_list = &dec_pict->dec_pict_seg_list;
	unsigned int memto_reg_host_part = 0;

	unsigned long ctrl_alloc = (unsigned long)batch_msgbuf_info->cpu_virt;
	unsigned long ctrl_alloc_end = ctrl_alloc + batch_msgbuf_info->buf_size;

	struct vdecdd_picture *picture =
		(struct vdecdd_picture *)str_unit->dd_pict_data;

	memset(pict_cmds, 0, sizeof(pict_cmds));
	memset(&buffers, 0, sizeof(buffers));

	VDEC_ASSERT(batch_msgbuf_info->buf_size >= CTRL_ALLOC_MAX_SEGMENT_SIZE);
	memset(batch_msgbuf_info->cpu_virt, 0, batch_msgbuf_info->buf_size);

	/* Construct transaction based on new picture. */
	VDEC_ASSERT(str_unit->str_unit_type == VDECDD_STRUNIT_PICTURE_START);

	/* Obtain picture data. */
	picture = (struct vdecdd_picture *)str_unit->dd_pict_data;
	dec_pict->recon_pict = &picture->disp_pict_buf;

	result = translation_get_codec(pstr_config_data->vid_std, &codec);
	if (result != IMG_SUCCESS)
		return result;

	translation_setup_std_header(pstr_config_data, dec_pict, str_unit, &hdr_size, picture,
				     pict_cmds, &parser_mode);

	buffers.recon_pict = dec_pict->recon_pict;
	buffers.alt_pict = dec_pict->alt_pict;

#ifdef HAS_HEVC
	/* Set pipe offsets to device buffers */
	if (pstr_config_data->vid_std == VDEC_STD_HEVC) {
		/* FW in multipipe requires this buffers to be allocated per stream */
		if (picture->pict_res_int && picture->pict_res_int->seq_resint &&
		    picture->pict_res_int->seq_resint->intra_buffer &&
		    picture->pict_res_int->seq_resint->aux_buffer) {
			buffers.intra_bufinfo =
				&picture->pict_res_int->seq_resint->intra_buffer->ddbuf_info;
			buffers.auxline_bufinfo =
				&picture->pict_res_int->seq_resint->aux_buffer->ddbuf_info;
		}
	} else {
		buffers.intra_bufinfo = dec_pict->intra_bufinfo;
		buffers.auxline_bufinfo = dec_pict->auxline_bufinfo;
	}

	if (buffers.intra_bufinfo)
		buffers.intra_bufsize_per_pipe = buffers.intra_bufinfo->buf_size /
			core_props->num_pixel_pipes;
	if (buffers.auxline_bufinfo)
		buffers.auxline_bufsize_per_pipe = buffers.auxline_bufinfo->buf_size /
			core_props->num_pixel_pipes;
#endif

#ifdef ERROR_CONCEALMENT
	if (picture->pict_res_int && picture->pict_res_int->seq_resint)
		if (picture->pict_res_int->seq_resint->err_pict_buf)
			buffers.err_pict_bufinfo =
				&picture->pict_res_int->seq_resint->err_pict_buf->ddbuf_info;
#endif

	/*
	 * Prepare Reconstructed Picture Configuration
	 * Note: we are obtaining values of registers prepared basing on header
	 * files generated from MSVDX *dev files.
	 * That's allowed, as layout of registers: MSVDX_CMDS_OPERATING_MODE,
	 * MSVDX_CMDS_EXTENDED_ROW_STRIDE,
	 * MSVDX_CMDS_ALTERNATIVE_OUTPUT_PICTURE_ROTATION,
	 * MSVDX_CMDS_CHROMA_ROW_STRIDE is the same for both MSVDX and PVDEC.
	 */
	vxd_set_reconpictcmds(str_unit, pstr_config_data, &picture->op_config, core_props,
			      &buffers, pict_cmds);

	/* Alternative Picture Configuration */
	if (dec_pict->alt_pict) {
		dec_pict->twopass = picture->op_config.force_oold;
		buffers.btwopass = dec_pict->twopass;
		/*
		 * Alternative Picture Configuration
		 * Note: we are obtaining values of registers prepared basing
		 * on header files generated from MSVDX *dev files.
		 * That's allowed, as layout of registers:
		 * MSVDX_CMDS_OPERATING_MODE, MSVDX_CMDS_EXTENDED_ROW_STRIDE,
		 * MSVDX_CMDS_ALTERNATIVE_OUTPUT_PICTURE_ROTATION,
		 * MSVDX_CMDS_CHROMA_ROW_STRIDE is the same for both MSVDX and
		 * PVDEC.
		 */
		/*
		 * Configure second buffer for out-of-loop processing
		 * (e.g. scaling etc.).
		 */
		vxd_set_altpictcmds(str_unit, pstr_config_data, &picture->op_config, core_props,
				    &buffers, pict_cmds);
	}

	/*
	 * Setup initial simple bitstream configuration to be used by parser
	 * task
	 */
	cmd_buf = (unsigned int *)ctrl_alloc;
	result = translation_pvdec_adddma_transfers
			(decpic_seg_list, &cmd_buf,
			 (ctrl_alloc_end - (unsigned long)cmd_buf) / sizeof(unsigned int),
			 dec_pict, str_unit->eop);
	if (result != IMG_SUCCESS)
		return result;

	if ((unsigned long)(cmd_buf + (sizeof(struct ctrl_alloc_header) +
		sizeof(struct vdec_ext_cmd)) / sizeof(unsigned int)) >=
		ctrl_alloc_end)
		return IMG_ERROR_INVALID_PARAMETERS;

	/*
	 * Setup regular control allocation message. Start with control
	 * allocation header
	 */
	translation_pvdec_ctrl_setuphdr((struct ctrl_alloc_header *)cmd_buf, pict_cmds);
	/* Setup additional params for VP8 */
	cmd_buf += sizeof(struct ctrl_alloc_header) / sizeof(unsigned int);

	/* Reserve space for VDEC extension command and fill it */
	vdec_ext = (struct vdec_ext_cmd *)cmd_buf;
	cmd_buf += sizeof(struct vdec_ext_cmd) / sizeof(unsigned int);

	result = translation_pvdecsetup_vdecext(vdec_ext, dec_pict, pict_cmds,
						str_unit,
						pstr_config_data->vid_std,
						parser_mode);
	if (result != IMG_SUCCESS)
		return result;

	vdec_ext->hdr_size = hdr_size;

	/* Add VLC tables to control allocation, skip when VC1 */
	if (pstr_config_data->vid_std != VDEC_STD_VC1 &&
	    dec_pict->vlc_idx_tables_bufinfo &&
	    dec_pict->vlc_idx_tables_bufinfo->cpu_virt) {
		unsigned short *vlc_idx_tables = (unsigned short *)
			dec_pict->vlc_idx_tables_bufinfo->cpu_virt;
		/*
		 * Get count of elements in VLC idx table. Each element is made
		 * of 3 IMG_UINT16, see e.g. mpeg2_idx.c
		 */
		unsigned int vlc_idx_count =
			dec_pict->vlc_idx_tables_bufinfo->buf_size /
			(3 * sizeof(unsigned short));

		/* Add command to DMA VLC */
		result = translation_pvdecsetup_vlcdma
				(dec_pict->vlc_tables_bufinfo, &cmd_buf,
				(ctrl_alloc_end - (unsigned long)cmd_buf) / sizeof(unsigned int));

		if (result != IMG_SUCCESS)
			return result;

		/* Add command to configure VLC tables */
		result = translation_pvdecsetup_vlctables
				((unsigned short (*)[3])vlc_idx_tables, vlc_idx_count, &cmd_buf,
				 (ctrl_alloc_end - (unsigned long)cmd_buf) / sizeof(unsigned int),
				 regs_offset->vec_offset);

		if (result != IMG_SUCCESS)
			return result;
	}

	/* Setup commands for standards other than HEVC */
	if (pstr_config_data->vid_std != VDEC_STD_HEVC) {
		translation_pvdec_setup_commands
				(pict_cmds, &cmd_buf,
				 (ctrl_alloc_end - (unsigned long)cmd_buf) / sizeof(unsigned int),
				 regs_offset->vdmc_cmd_offset);
	}

	/* Setup commands for HEVC */
	vdec_ext->mem_to_reg_size = 0;

#ifdef HAS_HEVC
	if (pstr_config_data->vid_std == VDEC_STD_HEVC) {
		result = translation_pvdec_setup_pvdec_commands
				(picture, dec_pict, str_unit,
				 regs_offset, &cmd_buf,
				 (ctrl_alloc_end - (unsigned long)cmd_buf) / sizeof(unsigned int),
				 &memto_reg_host_part, pict_cmds);
		if (result != IMG_SUCCESS) {
			pr_err("Failed to setup VDMC & VDEB firmware commands.");
			return result;
		}

		/* Set size of MemToReg buffer in VDEC extension command */
		VDEC_ASSERT(MEM_TO_REG_BUF_SIZE <
			(MEM2REG_SIZE_BUF_TOTAL_MASK >> MEM2REG_SIZE_BUF_TOTAL_SHIFT));
		VDEC_ASSERT(memto_reg_host_part <
			(MEM2REG_SIZE_HOST_PART_MASK >> MEM2REG_SIZE_HOST_PART_SHIFT));

		vdec_ext->mem_to_reg_size = (MEM_TO_REG_BUF_SIZE << MEM2REG_SIZE_BUF_TOTAL_SHIFT) |
			(memto_reg_host_part << MEM2REG_SIZE_HOST_PART_SHIFT);

		dec_pict->genc_id = picture->pict_res_int->seq_resint->genc_buf_id;
		dec_pict->genc_bufs = picture->pict_res_int->seq_resint->genc_buffers;
	}
#endif
	/* Finally mark end of commands */
	*(cmd_buf++) = CMD_COMPLETION;

	/* Print message for debugging */
	{
		int i;

		for (i = 0; i < ((unsigned long)cmd_buf - ctrl_alloc) / sizeof(unsigned int); i++)
			pr_debug("ctrl_alloc_buf[%d] == %08x\n", i,
				 ((unsigned int *)ctrl_alloc)[i]);
	}
	/* Transfer control allocation command to device memory */
	dec_pict->ctrl_alloc_bytes = ((unsigned long)cmd_buf - ctrl_alloc);
	dec_pict->ctrl_alloc_offset = dec_pict->ctrl_alloc_bytes;
	dec_pict->operating_op = pict_cmds[VDECFW_CMD_OPERATING_MODE];

	/*
	 * NOTE : Nothing related to tiling will be used.
	 * result = translation_ConfigureTiling(psStrUnit, psDecPict,
	 * psCoreProps);
	 */

	return result;
};

int translation_fragment_prepare(struct dec_decpict *dec_pict,
				 struct lst_t *decpic_seg_list, int eop,
				 struct dec_pict_fragment *pict_fragement)
{
	int result;
	unsigned int *cmd_buf;
	struct vidio_ddbufinfo *batchmsg_bufinfo;
	unsigned long ctrl_alloc;
	unsigned long ctrl_alloc_end;

	if (!dec_pict || !dec_pict->batch_msginfo ||
	    !decpic_seg_list || !pict_fragement)
		return IMG_ERROR_INVALID_PARAMETERS;

	batchmsg_bufinfo = dec_pict->batch_msginfo->ddbuf_info;

	ctrl_alloc = (unsigned long)batchmsg_bufinfo->cpu_virt +
		dec_pict->ctrl_alloc_offset;
	ctrl_alloc_end = (unsigned long)batchmsg_bufinfo->cpu_virt +
		batchmsg_bufinfo->buf_size;

	/*
	 * Setup initial simple bitstream configuration to be used by parser
	 * task
	 */
	cmd_buf = (unsigned int *)ctrl_alloc;
	result = translation_pvdec_adddma_transfers
			(decpic_seg_list, &cmd_buf,
			(ctrl_alloc_end - (unsigned long)cmd_buf) / sizeof(unsigned int),
			dec_pict, eop);

	if (result != IMG_SUCCESS)
		return result;

	/* Finally mark end of commands */
	*(cmd_buf++) = CMD_COMPLETION;

	/* Transfer control allocation command to device memory */
	pict_fragement->ctrl_alloc_offset = dec_pict->ctrl_alloc_offset;
	pict_fragement->ctrl_alloc_bytes =
		((unsigned long)cmd_buf - ctrl_alloc);

	dec_pict->ctrl_alloc_offset += pict_fragement->ctrl_alloc_bytes;

	return result;
};
#endif /* VDEC_USE_PVDEC */
