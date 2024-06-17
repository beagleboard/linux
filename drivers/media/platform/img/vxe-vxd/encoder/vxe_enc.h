/* SPDX-License-Identifier: GPL-2.0 */
/*
 * encoder interface header
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

#ifndef _VXE_ENC_H
#define _VXE_ENC_H

#include <linux/dma-mapping.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>
#include "topaz_api.h"

#define HW_ALIGN 64
#define MB_SIZE 16
#define VXE_INVALID_ID (-1)
#define OCM_RAM_POOL_CHUNK_SIZE (32 * 1024)

enum {
	Q_ENC_DATA_SRC         = 0,
	Q_ENC_DATA_DST         = 1,
	Q_ENC_DATA_FORCE32BITS = 0x7FFFFFFFU
};

enum {
	IMG_ENC_FMT_TYPE_CAPTURE     = 0x01,
	IMG_ENC_FMT_TYPE_OUTPUT      = 0x10,
	IMG_ENC_FMT_TYPE_FORCE32BITS = 0x7FFFFFFFU
};

enum vxe_map_flags {
	VXE_MAP_FLAG_NONE        = 0x0,
	VXE_MAP_FLAG_READ_ONLY   = 0x1,
	VXE_MAP_FLAG_WRITE_ONLY  = 0x2,
	VXE_MAP_FLAG_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * struct vxe_enc_fmt - contains info for each supported video format
 */
struct vxe_enc_fmt {
	unsigned int fourcc;
	unsigned int num_planes;
	unsigned int type;
	union {
		enum img_standard std;
		enum img_format fmt;
	};
	unsigned int min_bufs;
	unsigned int size_num[MAX_PLANES];
	unsigned int size_den[MAX_PLANES];
	unsigned int bytes_pp;
	enum img_csc_preset csc_preset;
};

/*
 * struct vxe_buffer - contains info for all buffers
 */
struct vxe_buffer {
	struct v4l2_m2m_buffer buffer;
	unsigned int index;
	unsigned int buf_map_id;
	struct vidio_ddbufinfo buf_info;
	union {
		struct img_frame src_frame;
		struct img_coded_buffer coded_buffer;
	};
	struct img_buffer y_buffer;
	struct img_buffer u_buffer;
	struct img_buffer v_buffer;
	unsigned char src_slot_num;
	unsigned char mapped;
};

/*
 * struct vxe_heap - node for heaps list
 * @id:   heap id
 * @list: Entry in <struct vxe_drv:heaps>
 */
struct vxe_heap {
	int id;
	struct list_head list;
};

/* Driver context */
struct vxe_drv_ctx {
	/* Available memory heaps. List of <struct vxe_heap> */
	struct list_head heaps;
	/* heap id for all internal allocations */
	int internal_heap_id;
	/* Memory Management context for driver */
	struct mem_ctx *mem_ctx;
	/* MMU context for driver */
	struct mmu_ctx *mmu_ctx;
	/* PTD */
	unsigned int ptd;
};

/*
 * struct vxe_dev - The struct containing encoder driver internal parameters.
 */
struct vxe_dev {
	void *dev;
	struct video_device *vfd;
	struct v4l2_device ti_vxe_dev;
	struct platform_device *plat_dev;
	struct v4l2_m2m_dev *m2m_dev;
	struct mutex *mutex;
	int module_irq;
	struct idr *streams;
	void __iomem *reg_base;
	void *topaz_dev_ctx;
	struct vxe_drv_ctx drv_ctx;
	/* dummy context for MMU mappings and allocations */
	struct vxe_enc_ctx *ctx;
	unsigned int num_pipes;

	/* The variables defined below are used in RTOS only. */
	/* This variable holds queue handler */
	void *vxe_worker_queue_handle;
	void *vxe_worker_queue_sem_handle;

	/* On Chip Memory Pool for above MB params struct */
	/* Supporting only 2 max instances (upto 1080p resolutions) to make use of this */
	void *ocm_ram_chunk[2]; //each chunk of 32KB
	void *ram_chunk_owner[2];

};

#define S_FMT_FLAG_OUT_RECV 0x1
#define S_FMT_FLAG_CAP_RECV 0x2
#define S_FMT_FLAG_STREAM_CREATED 0x4

#define VXE_ENCODER_MAX_WIDTH 1920
#define VXE_ENCODER_MIN_WIDTH 64
#define VXE_ENCODER_MAX_HEIGHT 1080
#define VXE_ENCODER_MIN_HEIGHT 64

#define VXE_ENCODER_DEFAULT_HEIGHT 240
#define VXE_ENCODER_DEFAULT_WIDTH 416
#define VXE_ENCODER_INITIAL_QP_I 18
#define VXE_ENCODER_DEFAULT_FRAMERATE 30

/*
 * struct vxe_enc_q_data - contains queue data information
 *
 * @fmt: format info
 * @width: frame width
 * @height: frame height
 * @bytesperline: bytes per line in memory
 * @size_image: image size in memory
 */
struct vxe_enc_q_data {
	struct vxe_enc_fmt *fmt;
	unsigned int width;
	unsigned int height;
	unsigned int bytesperline[MAX_PLANES];
	unsigned int size_image[MAX_PLANES];
	unsigned char streaming;
};

#ifdef ENABLE_PROFILING
struct enc_drv_latency {
	unsigned int start_time;
	unsigned int end_time;
};
#endif

/*
 * struct vxe_ctx - The struct containing stream context parameters.
 */
struct vxe_enc_ctx {
	struct v4l2_fh fh;
	struct vxe_dev *dev;
	void **enc_context;
	void *topaz_str_context;
	struct mutex *mutex;
	unsigned char core_streaming;
	struct img_enc_caps caps;
	struct img_rc_params rc;
	struct img_video_params vparams;
	struct vxe_enc_q_data out_queue;
	struct vxe_enc_q_data cap_queue;
	struct mem_ctx *mem_ctx;
	struct mmu_ctx *mmu_ctx;
	/* list open_slots*/
	unsigned char s_fmt_flags;
	struct h264_vui_params vui_params;
	struct h264_crop_params crop_params;
	struct h264_sequence_header_params sh_params;
	unsigned char eos;
	unsigned char flag_last;
	unsigned int coded_packages_per_frame; /* How many slices per frame */
	unsigned int available_coded_packages;
	unsigned int available_source_frames;
	unsigned int frames_encoding;
	unsigned int frame_num;
	unsigned int last_frame_num;
	unsigned int cap_seq;   /* sequence number on capture port */
	unsigned int out_seq;   /* sequence number on output port */

	enum v4l2_colorspace colorspace;
	enum v4l2_xfer_func xfer_func;
	enum v4l2_ycbcr_encoding ycbcr_enc;
	enum v4l2_quantization quantization;
	enum v4l2_hsv_encoding hsv_enc;

	/* The below variable used only in Rtos */
	void *mm_return_resource; /* Place holder for CB to application */
	void *stream_worker_queue_handle;
	void *stream_worker_queue_sem_handle;
	void *work;
	struct vxe_enc_q_data q_data[2];
	struct v4l2_ctrl_handler v4l2_ctrl_hdl;
	struct sg_table above_mb_params_sgt[2];

#ifdef ENABLE_PROFILING
	struct enc_drv_latency drv_lat;
#endif
};

int vxe_init_mem(struct vxe_dev *vxe);
void vxe_deinit_mem(struct vxe_dev *vxe);
void vxe_create_ctx(struct vxe_dev *vxe, struct vxe_enc_ctx *ctx);
int calculate_h264_level(unsigned int width, unsigned int height, unsigned int framerate,
			 unsigned char rc_enable, unsigned int bitrate,
			 unsigned char lossless,
			 enum sh_profile_type profile_type,
			 unsigned int max_num_ref_frames);
enum sh_profile_type find_h264_profile(unsigned char lossless,
				       unsigned char h264_use_default_scaling_list,
				       unsigned int custom_quant_mask,
				       unsigned char h264_8x8_transform,
				       unsigned char enable_mvc,
				       unsigned int b_frame_count,
				       unsigned char interlaced,
				       unsigned char h264_cabac,
				       unsigned int weighted_prediction_mode,
				       unsigned int weighted_implicit_bi_pred);
void vxe_fill_default_src_frame_params(struct vxe_buffer *buf);
void vxe_fill_default_params(struct vxe_enc_ctx *ctx);
unsigned int vxe_get_sizeimage(int w, int h, struct vxe_enc_fmt *fmt, unsigned char plane_id);
unsigned int vxe_get_stride(int w, struct vxe_enc_fmt *fmt);

#endif /* _VXE_ENC_H */
