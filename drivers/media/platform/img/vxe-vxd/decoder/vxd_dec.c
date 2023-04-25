// SPDX-License-Identifier: GPL-2.0
/*
 * IMG DEC SYSDEV and UI Interface function implementations
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

#include <linux/dma-mapping.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>

#include "core.h"
#include "h264fw_data.h"
#include "hevcfw_data.h"
#include "img_dec_common.h"
#include "vxd_pvdec_priv.h"

unsigned int get_nbuffers(enum vdec_vid_std std, int w, int h,
			  unsigned int max_num_ref_frames)
{
	unsigned int nbuffers;

	switch (std) {
	case VDEC_STD_H264:
		/*
		 * Request number of buffers from header bspp information
		 * using formula N + Display Lag
		 * Parser is passing (2*N)
		 */
		if (max_num_ref_frames == 0) {
			nbuffers = DISPLAY_LAG + min(MAX_CAPBUFS_H264,
					(184320 / ((w / 16) * (h / 16))));
		} else {
			nbuffers = max_num_ref_frames + DISPLAY_LAG;
		}
		break;
	case VDEC_STD_HEVC:
		if (max_num_ref_frames == 0) {
			if ((w * h) <= (HEVC_MAX_LUMA_PS >> 2))
				nbuffers = 16;
			else if ((w * h) <= (HEVC_MAX_LUMA_PS >> 1))
				nbuffers = 12;
			else if ((w * h) <= ((3 * HEVC_MAX_LUMA_PS) >> 2))
				nbuffers = 8;
			else
				nbuffers = 6;
			nbuffers += DISPLAY_LAG;
		} else {
			nbuffers = max_num_ref_frames + DISPLAY_LAG;
		}
		break;
#ifdef HAS_JPEG
	case VDEC_STD_JPEG:
		/*
		 * Request number of output buffers based on h264 spec
		 * + display delay
		 */
		nbuffers = DISPLAY_LAG + min(MAX_CAPBUFS_H264,
				(184320 / ((w / 16) * (h / 16))));
		break;
#endif
	default:
		nbuffers = 0;
	}

	return nbuffers;
}

int vxd_dec_alloc_bspp_resource(struct vxd_dec_ctx *ctx, enum vdec_vid_std vid_std)
{
	struct vxd_dev *vxd_dev = ctx->dev;
	struct device *dev = vxd_dev->v4l2_dev.dev;
	struct vdec_buf_info buf_info;
	struct bspp_ddbuf_array_info *fw_sequ = ctx->fw_sequ;
	struct bspp_ddbuf_array_info *fw_pps = ctx->fw_pps;
	int attributes = 0, heap_id = 0, size = 0;
	int i, ret = 0;

	attributes = SYS_MEMATTRIB_UNCACHED | SYS_MEMATTRIB_WRITECOMBINE |
		SYS_MEMATTRIB_INTERNAL | SYS_MEMATTRIB_CPU_WRITE;
	heap_id = vxd_g_internal_heap_id();

	size = vid_std == VDEC_STD_HEVC ?
		sizeof(struct hevcfw_sequence_ps) : sizeof(struct h264fw_sequence_ps);

#ifdef HAS_JPEG
	if (vid_std == VDEC_STD_JPEG)
		size = sizeof(struct vdec_jpeg_sequ_hdr_info);
#endif

	for (i = 0; i < MAX_SEQUENCES; i++) {
		ret = img_mem_alloc(vxd_dev->dev, ctx->mem_ctx, heap_id,
				    size, (enum mem_attr)attributes,
				    (int *)&fw_sequ[i].ddbuf_info.buf_id);
		if (ret) {
			dev_err(dev, "Couldn't allocate sequ buffer %d\n", i);
			return -ENOMEM;
		}
		ret = img_mem_map_km(ctx->mem_ctx, fw_sequ[i].ddbuf_info.buf_id);
		if (ret) {
			dev_err(dev, "Couldn't map sequ buffer %d\n", i);
			return -ENOMEM;
		}
		fw_sequ[i].ddbuf_info.cpu_virt_addr = img_mem_get_kptr
							(ctx->mem_ctx,
							 fw_sequ[i].ddbuf_info.buf_id);
		fw_sequ[i].buf_offset = 0;
		fw_sequ[i].buf_element_size = size;
		fw_sequ[i].ddbuf_info.buf_size = size;
		fw_sequ[i].ddbuf_info.mem_attrib = (enum sys_emem_attrib)attributes;
		memset(fw_sequ[i].ddbuf_info.cpu_virt_addr, 0, size);

		buf_info.cpu_linear_addr =
			fw_sequ[i].ddbuf_info.cpu_virt_addr;
		buf_info.buf_size = size;
		buf_info.fd = -1;
		buf_info.buf_id = fw_sequ[i].ddbuf_info.buf_id;
		buf_info.mem_attrib =
			(enum sys_emem_attrib)(SYS_MEMATTRIB_UNCACHED | SYS_MEMATTRIB_WRITECOMBINE |
			SYS_MEMATTRIB_INPUT | SYS_MEMATTRIB_CPU_WRITE);

		ret = core_stream_map_buf(ctx->res_str_id, VDEC_BUFTYPE_BITSTREAM, &buf_info,
					  &fw_sequ[i].ddbuf_info.bufmap_id);
		if (ret) {
			dev_err(dev, "sps core_stream_map_buf failed\n");
			return ret;
		}
	}

#ifdef HAS_JPEG
	if (vid_std == VDEC_STD_JPEG)
		return 0;
#endif

	size = vid_std == VDEC_STD_HEVC ?
		sizeof(struct hevcfw_picture_ps) : sizeof(struct h264fw_picture_ps);

	for (i = 0; i < MAX_PPSS; i++) {
		ret = img_mem_alloc(vxd_dev->dev, ctx->mem_ctx, heap_id, size,
				    (enum mem_attr)attributes,
				    (int *)&fw_pps[i].ddbuf_info.buf_id);
		if (ret) {
			dev_err(dev, "Couldn't allocate sequ buffer %d\n", i);
			return -ENOMEM;
		}
		ret = img_mem_map_km(ctx->mem_ctx, fw_pps[i].ddbuf_info.buf_id);
		if (ret) {
			dev_err(dev, "Couldn't map sequ buffer %d\n", i);
			return -ENOMEM;
		}
		fw_pps[i].ddbuf_info.cpu_virt_addr = img_mem_get_kptr(ctx->mem_ctx,
								      fw_pps[i].ddbuf_info.buf_id);
		fw_pps[i].buf_offset = 0;
		fw_pps[i].buf_element_size = size;
		fw_pps[i].ddbuf_info.buf_size = size;
		fw_pps[i].ddbuf_info.mem_attrib = (enum sys_emem_attrib)attributes;
		memset(fw_pps[i].ddbuf_info.cpu_virt_addr, 0, size);

		buf_info.cpu_linear_addr =
			fw_pps[i].ddbuf_info.cpu_virt_addr;
		buf_info.buf_size = size;
		buf_info.fd = -1;
		buf_info.buf_id = fw_pps[i].ddbuf_info.buf_id;
		buf_info.mem_attrib =
			(enum sys_emem_attrib)(SYS_MEMATTRIB_UNCACHED | SYS_MEMATTRIB_WRITECOMBINE |
			SYS_MEMATTRIB_INPUT | SYS_MEMATTRIB_CPU_WRITE);

		ret = core_stream_map_buf(ctx->res_str_id, VDEC_BUFTYPE_BITSTREAM, &buf_info,
					  &fw_pps[i].ddbuf_info.bufmap_id);
		if (ret) {
			dev_err(dev, "pps core_stream_map_buf failed\n");
			return ret;
		}
	}
	return 0;
}
