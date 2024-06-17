// SPDX-License-Identifier: GPL-2.0
/*
 * VXD Decoder Core component function implementations
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

#include "core.h"
#include "decoder.h"
#include "img_errors.h"
#include "img_pixfmts.h"
#include "img_profiles_levels.h"
#include "lst.h"
#include "resource.h"
#include "rman_api.h"
#include "vdecdd_utils.h"
#include "vdec_mmu_wrapper.h"
#include "vxd_dec.h"

#ifdef HAS_HEVC
#define SEQ_RES_NEEDED
#define GENC_BUFF_COUNT 4
#endif

/*
 * This enum defines resource availability masks.
 * @brief  Resource Availability
 */
enum core_availability {
	CORE_AVAIL_PICTBUF      = (1 << 0),
	CORE_AVAIL_PICTRES      = (1 << 1),
	CORE_AVAIL_CORE         = (1 << 2),
	CORE_AVAIL_MAX,
	CORE_AVAIL_FORCE32BITS = 0x7FFFFFFFU
};

struct core_mbparam_alloc_info {
	unsigned char alloc_mbparam_bufs;
	unsigned int mbparam_size;
	unsigned int overalloc_mbnum;
};

static struct core_mbparam_alloc_info mbparam_allocinfo[VDEC_STD_MAX - 1] = {
	/*                AllocFlag    MBParamSize    Overalloc     */
	/* MPEG2    */ { TRUE,    0xc8,          0             },
	/* MPEG4    */ { TRUE,    0xc8,          0             },
	/* H263     */ { TRUE,    0xc8,          0             },
	/* H264     */ { TRUE,    0x80,          0             },
	/* VC1      */ { TRUE,    0x80,          (4096 * 2) / 0x80 },
	/* AVS      */ { TRUE,    0x80,          0             },
	/* REAL     */ { TRUE,    0x80,          0             },
	/* JPEG     */ { FALSE,   0x00,          0             },
	/* VP6      */ { TRUE,    0x80,          0             },
	/* VP8      */ { TRUE,    0x80,          0             },
	/* SORENSON */ { TRUE,    0xc8,          0             },
	/* HEVC     */ { TRUE,    0x40,          0             },
};

struct vxdio_mempool {
	unsigned int mem_heap_id;
	enum sys_emem_attrib mem_attrib;
};

static unsigned int global_avail_slots;
static unsigned char is_core_initialized;

/*
 * This structure contains the core Context.
 * @brief  core Context
 */
struct core_context {
	struct vdecdd_dddev_context     *dev_ctx;
	/* List of stream context structures */
	struct lst_t core_str_ctx;
	vxd_cb vxd_str_processed_cb;
};

/* Global Core Context */
static struct core_context *global_core_ctx;

/*
 * This structure contains the picture buffer size info.
 * @brief  Picture Resource Info
 */
struct core_pict_bufsize_info {
	unsigned int mbparams_bufsize;

#ifdef HAS_HEVC
	union {
		struct hevc_bufsize_pict {
			/* Size of GENC fragment buffer for HEVC */
			unsigned int genc_fragment_bufsize;
		} hevc_bufsize_pict;
	};
#endif
};

/*
 * This structure contains the sequence resource info.
 * @brief  Sequence Resource Info
 */
struct core_seq_resinfo {
	union {
#ifdef HAS_HEVC
		struct hevc_bufsize_seqres {
			unsigned int genc_bufsize; /* Size of GEN buffers for HEVC */
			unsigned int intra_bufsize; /* Size of GEN buffers for HEVC */
			unsigned int aux_bufsize;   /* Size of GEN buffers for HEVC */
		} hevc_bufsize_seqres;
#endif

#ifndef SEQ_RES_NEEDED
		unsigned int dummy;
#endif
	};
};

struct core_pict_resinfo {
	unsigned int pict_res_num;
	struct core_pict_bufsize_info size_info;
	unsigned char is_valid;
};

/*
 * This structure contains the standard specific part of plant context.
 * @brief  Standard Specific Context
 */
struct core_std_spec_context {
	union {
#ifdef HAS_HEVC
		struct hevc_ctx {
			/* Counts genc buffer allocations  */
			unsigned short genc_id_gen;
		} hevc_ctx;
#else
		unsigned int dummy;
#endif
	};
};

struct core_stream_context;

struct core_std_spec_operations {
	/* Allocates standard specific picture buffers.  */
	int (*alloc_picture_buffers)(struct core_stream_context *core_strctx,
				     struct vdecdd_pict_resint *pict_resint,
				     struct vxdio_mempool mem_pool,
				     struct core_pict_resinfo *pict_res_info);

	/* Frees standard specific picture buffers.  */
	int (*free_picture_resource)(struct core_stream_context *core_strctx,
				     struct vdecdd_pict_resint *pic_res_int);

	/* Allocates standard specific sequence buffers.  */
	int (*alloc_sequence_buffers)(struct core_stream_context *core_strctx,
				      struct vdecdd_seq_resint *seq_res_int,
				      struct vxdio_mempool mem_pool,
				      struct core_seq_resinfo *seq_res_info);

	/* Frees standard specific sequence buffers.  */
	int (*free_sequence_resource)(struct core_stream_context *core_strctx,
				      struct vdecdd_seq_resint *seq_res_int);

	/* Returns buffer's sizes (common and standard specific).  */
	int (*bufs_get_size)(struct core_stream_context *core_strctx,
			     const struct vdec_comsequ_hdrinfo *seq_hdrinfo,
			     struct vdec_pict_size *max_pict_size,
			     struct core_pict_bufsize_info *size_info,
			     struct core_seq_resinfo *seq_resinfo,
			     unsigned char *resource_needed);

	/* Checks whether resource is still suitable.  */
	unsigned char (*is_stream_resource_suitable)(struct core_pict_resinfo *pict_resinfo,
						     struct core_pict_resinfo *old_pict_resinfo,
						     struct core_seq_resinfo *seq_resinfo,
						     struct core_seq_resinfo *old_seq_resinfo);
};

/*
 * This structure contains the core Stream Context.
 * @brief  core Stream Context
 */
struct core_stream_context {
	void    **link; /* to be part of single linked list */
	struct core_context     *core_ctx;
	struct vdecdd_ddstr_ctx *dd_str_ctx;
	struct vxd_dec_ctx      *vxd_dec_context;

	/* list of picture buffers */
	struct lst_t pict_buf_list;

	/* List of picture resources allocated for this stream */
	struct lst_t pict_res_list;
	struct lst_t old_pict_res_list;

	struct lst_t aux_pict_res_list;

#ifdef SEQ_RES_NEEDED
	/* List of active sequence resources that are allocated for this stream. */
	struct lst_t seq_res_list;
	/*
	 * List of sequence resources that are allocated for this stream but no
	 * longer suitable for new sequence(s).
	 */
	struct lst_t old_seq_res_list;
#endif

	/* List of sequence header information */
	struct lst_t seq_hdr_list;
	/* Queue of stream units to be processed */
	struct lst_t str_unit_list;

	struct vdec_comsequ_hdrinfo comseq_hdr_info;
	unsigned char opcfg_set;
	/* Picture buffer layout to use for decoding. */
	struct vdecdd_ddpict_buf disp_pict_buf;
	struct vdec_str_opconfig op_cfg;
	unsigned char new_seq;
	unsigned char new_op_cfg;
	unsigned char no_prev_refs_used;
	unsigned int avail_slots;
	unsigned int res_avail;
	unsigned char stopped;
	struct core_pict_resinfo pict_resinfo;
	/* Current sequence resource info. */
	struct core_seq_resinfo seq_resinfo;

	/* Reconstructed picture buffer */
	struct vdecdd_ddpict_buf recon_pictbuf;
	/* Coded picture size of last reconfiguration */
	struct vdec_pict_size coded_pict_size;
	/* Standard specific operations. */
	struct core_std_spec_operations *std_spec_ops;
	/* Standard specific context. */
	struct core_std_spec_context std_spec_context;
};

#ifdef HAS_HEVC
static int core_free_hevc_picture_resource(struct core_stream_context *core_strctx,
					   struct vdecdd_pict_resint *pic_res_int);

static int core_free_hevc_sequence_resource(struct core_stream_context *core_strctx,
					    struct vdecdd_seq_resint *seq_res_int);

static int core_hevc_bufs_get_size(struct core_stream_context *core_str_ctx,
				   const struct vdec_comsequ_hdrinfo *seq_hdr_info,
				   struct vdec_pict_size *max_pict_size,
				   struct core_pict_bufsize_info *size_info,
				   struct core_seq_resinfo *seq_res_info,
				   unsigned char *resource_needed);

static unsigned char core_is_hevc_stream_resource_suitable
			(struct core_pict_resinfo *pict_res_info,
			 struct core_pict_resinfo *old_pict_res_info,
			 struct core_seq_resinfo *seq_res_info,
			 struct core_seq_resinfo *old_seq_res_info);

static int core_alloc_hevc_specific_seq_buffers(struct core_stream_context *core_strctx,
						struct vdecdd_seq_resint *seq_res_int,
						struct vxdio_mempool mempool,
						struct core_seq_resinfo *seq_res_info);

static int core_alloc_hevc_specific_pict_buffers(struct core_stream_context *core_strctx,
						 struct vdecdd_pict_resint *pict_res_int,
						 struct vxdio_mempool mempool,
						 struct core_pict_resinfo *pict_res_info);
#endif

static int
core_common_bufs_getsize(struct core_stream_context *core_str_ctx,
			 const struct vdec_comsequ_hdrinfo *comseq_hdrinfo,
			 struct vdec_pict_size *max_pict_size,
			 struct core_pict_bufsize_info     *size_info,
			 struct core_seq_resinfo  *seq_res_info, unsigned char *res_needed);

static struct core_std_spec_operations std_specific_ops[VDEC_STD_MAX - 1] = {
	/* AllocPicture  FreePicture  AllocSeq   FreeSeq   BufsGetSize  IsStreamResourceSuitable */
	/* MPEG2   */ { .alloc_picture_buffers = NULL,
		.free_picture_resource = NULL,
		.alloc_sequence_buffers = NULL,
		.free_sequence_resource = NULL,
		.bufs_get_size = NULL,
		.is_stream_resource_suitable = NULL},

	/* MPEG4   */ { .alloc_picture_buffers = NULL,
		.free_picture_resource = NULL,
		.alloc_sequence_buffers = NULL,
		.free_sequence_resource = NULL,
		.bufs_get_size = NULL,
		.is_stream_resource_suitable = NULL},

	/* H263   */ { .alloc_picture_buffers = NULL,
		.free_picture_resource = NULL,
		.alloc_sequence_buffers = NULL,
		.free_sequence_resource = NULL,
		.bufs_get_size = NULL,
		.is_stream_resource_suitable = NULL},

	/* H264   */ { .alloc_picture_buffers = NULL,
		.free_picture_resource = NULL,
		.alloc_sequence_buffers = NULL,
		.free_sequence_resource = NULL,
		.bufs_get_size = core_common_bufs_getsize,
		.is_stream_resource_suitable = NULL},

	/* VC1   */ { .alloc_picture_buffers = NULL,
		.free_picture_resource = NULL,
		.alloc_sequence_buffers = NULL,
		.free_sequence_resource = NULL,
		.bufs_get_size = NULL,
		.is_stream_resource_suitable = NULL},

	/* AVS   */ { .alloc_picture_buffers = NULL,
		.free_picture_resource = NULL,
		.alloc_sequence_buffers = NULL,
		.free_sequence_resource = NULL,
		.bufs_get_size = NULL,
		.is_stream_resource_suitable = NULL},

	/* REAL   */ { .alloc_picture_buffers = NULL,
		.free_picture_resource = NULL,
		.alloc_sequence_buffers = NULL,
		.free_sequence_resource = NULL,
		.bufs_get_size = NULL,
		.is_stream_resource_suitable = NULL},

	/* JPEG   */ { .alloc_picture_buffers = NULL,
		.free_picture_resource = NULL,
		.alloc_sequence_buffers = NULL,
		.free_sequence_resource = NULL,
		.bufs_get_size = NULL,
		.is_stream_resource_suitable = NULL},

	/* VP6   */ { .alloc_picture_buffers = NULL,
		.free_picture_resource = NULL,
		.alloc_sequence_buffers = NULL,
		.free_sequence_resource = NULL,
		.bufs_get_size = NULL,
		.is_stream_resource_suitable = NULL},

	/* VP8   */ { .alloc_picture_buffers = NULL,
		.free_picture_resource = NULL,
		.alloc_sequence_buffers = NULL,
		.free_sequence_resource = NULL,
		.bufs_get_size = NULL,
		.is_stream_resource_suitable = NULL},

	/* SORENSON */ { .alloc_picture_buffers = NULL,
		.free_picture_resource = NULL,
		.alloc_sequence_buffers = NULL,
		.free_sequence_resource = NULL,
		.bufs_get_size = NULL,
		.is_stream_resource_suitable = NULL},

#ifdef HAS_HEVC
	/* HEVC*/ { .alloc_picture_buffers = core_alloc_hevc_specific_pict_buffers,
		.free_picture_resource = core_free_hevc_picture_resource,
		.alloc_sequence_buffers = core_alloc_hevc_specific_seq_buffers,
		.free_sequence_resource = core_free_hevc_sequence_resource,
		.bufs_get_size = core_hevc_bufs_get_size,
		.is_stream_resource_suitable = core_is_hevc_stream_resource_suitable},
#else
	/* HEVC */ { .alloc_picture_buffers = NULL,
		.free_picture_resource = NULL,
		.alloc_sequence_buffers = NULL,
		.free_sequence_resource = NULL,
		.bufs_get_size = NULL,
		.is_stream_resource_suitable = NULL},
#endif
};

#ifdef ERROR_CONCEALMENT
/*
 * This structure contains the Error Recovery Frame Store info.
 * @brief  Error Recovery Frame Store Info
 */
struct core_err_recovery_frame_info {
	/* Flag to indicate if Error Recovery Frame Store is enabled for standard. */
	unsigned char enabled;
	/* Limitation for maximum frame size based on dimensions. */
	unsigned int max_size;
};

static struct core_err_recovery_frame_info err_recovery_frame_info[VDEC_STD_MAX - 1] = {
	/*               enabled  max_frame_size  */
	/* MPEG2    */ { TRUE,     ~0         },
	/* MPEG4    */ { TRUE,     ~0         },
	/* H263     */ { FALSE,     0         },
	/* H264     */ { TRUE,     ~0         },
	/* VC1      */ { FALSE,     0         },
	/* AVS      */ { FALSE,     0         },
	/* REAL     */ { FALSE,     0         },
	/* JPEG     */ { FALSE,     0         },
	/* VP6      */ { FALSE,     0         },
	/* VP8      */ { FALSE,     0         },
	/* SORENSON */ { FALSE,     0         },
	/* HEVC     */ { TRUE,     ~0         },
};
#endif

static void core_fw_response_cb(int res_str_id, unsigned int *msg, unsigned int msg_size,
				unsigned int msg_flags)
{
	struct core_stream_context *core_str_ctx;
	int ret;

	/* extract core_str_ctx and dec_core_ctx from res_str_id */
	VDEC_ASSERT(res_str_id);

	/* Get access to stream context.. */
	ret = rman_get_resource(res_str_id, VDECDD_STREAM_TYPE_ID, (void **)&core_str_ctx, NULL);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		pr_err("could not extract core_str_context\n");

	ret = decoder_service_firmware_response(core_str_ctx->dd_str_ctx->dec_ctx,
						msg, msg_size, msg_flags);
	VDEC_ASSERT((ret == IMG_SUCCESS) | (ret == IMG_ERROR_FATAL));
	if (ret != IMG_SUCCESS)
		pr_err("decoder_service_firmware_response failed\n");
}

/*
 * @Function core_initialise
 */
int core_initialise(void *dev_handle, unsigned int int_heap_id, void *vxd_cb_ptr)
{
	struct vdecdd_dd_devconfig dev_cfg_local;
	unsigned int num_pipes_local;
	int ret;

	if (is_core_initialized)
		return IMG_ERROR_INVALID_PARAMETERS;

	is_core_initialized = TRUE;

	global_core_ctx = kzalloc(sizeof(*global_core_ctx), GFP_KERNEL);
	if (!global_core_ctx) {
		is_core_initialized = FALSE;
		return IMG_ERROR_OUT_OF_MEMORY;
	}

	global_core_ctx->dev_ctx = kzalloc(sizeof(*global_core_ctx->dev_ctx), GFP_KERNEL);
	if (!global_core_ctx->dev_ctx) {
		kfree(global_core_ctx);
		global_core_ctx = NULL;
		is_core_initialized = FALSE;
		return IMG_ERROR_OUT_OF_MEMORY;
	}

	/* Initialise device context. */
	global_core_ctx->dev_ctx->dev_handle = dev_handle; /* v4L2 dev handle */
	global_core_ctx->vxd_str_processed_cb = (vxd_cb)vxd_cb_ptr;

	ret = decoder_initialise(global_core_ctx->dev_ctx, int_heap_id,
				 &dev_cfg_local, &num_pipes_local,
				 &global_core_ctx->dev_ctx->dec_context);
	if (ret != IMG_SUCCESS)
		goto decoder_init_error;

	global_core_ctx->dev_ctx->internal_heap_id = int_heap_id;

#ifdef DEBUG_DECODER_DRIVER
	/* Dump codec config */
	pr_info("Decode slots/core:  %d", dev_cfg_local.num_slots_per_pipe);
#endif

	lst_init(&global_core_ctx->core_str_ctx);

	/* Ensure the resource manager is initialised.. */
	ret = rman_initialise();
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		goto rman_init_error;

	/* Create resource bucket.. */
	ret = rman_create_bucket(&global_core_ctx->dev_ctx->res_buck_handle);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		goto create_bucket_error;

	return IMG_SUCCESS;

create_bucket_error:
	rman_deinitialise();

rman_init_error:
	decoder_deinitialise(global_core_ctx->dev_ctx->dec_context);

decoder_init_error:
	kfree(global_core_ctx->dev_ctx);
	global_core_ctx->dev_ctx = NULL;
	kfree(global_core_ctx);
	global_core_ctx = NULL;

	is_core_initialized = FALSE;

	return ret;
}

/*
 * @Function    core_check_decoder_support
 * @Description
 * This function determines whether Decoder supports bitstream and
 * configuration.
 */
static int
core_check_decoder_support(const struct vdecdd_dddev_context *dd_dev_ctx,
			   const struct vdec_str_configdata  *str_cfg_data,
			   const struct vdec_comsequ_hdrinfo *prev_seq_hdrinfo,
			   const struct bspp_pict_hdr_info *prev_pict_hdrinfo,
			   const struct vdecdd_mapbuf_info *map_bufinfo,
			   struct vdecdd_supp_check *supp_check)
{
	int ret;
	struct vdec_unsupp_flags unsupported;
	struct vdec_pict_rendinfo disp_pict_rend_info;

	memset(&disp_pict_rend_info, 0, sizeof(struct vdec_pict_rendinfo));

	/*
	 * If output picture buffer information is provided create another
	 * with properties required by bitstream so that it can be compared.
	 */
	if (supp_check->disp_pictbuf) {
		struct vdec_pict_rend_config pict_rend_cfg;

		memset(&pict_rend_cfg, 0, sizeof(pict_rend_cfg));

		/*
		 * Cannot validate the display picture buffer layout without
		 * knowing the pixel format required for the output and the
		 * sequence information.
		 */
		if (supp_check->comseq_hdrinfo && supp_check->op_cfg) {
			pict_rend_cfg.coded_pict_size =
				supp_check->comseq_hdrinfo->max_frame_size;

			pict_rend_cfg.byte_interleave =
					supp_check->disp_pictbuf->buf_config.byte_interleave;

			pict_rend_cfg.packed =
					supp_check->disp_pictbuf->buf_config.packed;

			pict_rend_cfg.stride_alignment =
					supp_check->disp_pictbuf->buf_config.stride_alignment;

			/*
			 * Recalculate render picture layout based upon
			 * sequence and output config.
			 */
			vdecddutils_pictbuf_getinfo(str_cfg_data,
						    &pict_rend_cfg,
						    supp_check->op_cfg,
						    &disp_pict_rend_info);
		}
	}
	/* Check that the decoder supports the picture. */
	ret = decoder_check_support(dd_dev_ctx->dec_context, str_cfg_data,
				    supp_check->op_cfg,
				    supp_check->disp_pictbuf,
				    (disp_pict_rend_info.rendered_size) ?
				    &disp_pict_rend_info : NULL,
				    supp_check->comseq_hdrinfo,
				    supp_check->pict_hdrinfo,
				    prev_seq_hdrinfo,
				    prev_pict_hdrinfo,
				    supp_check->non_cfg_req,
				    &unsupported,
				    &supp_check->features);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS) {
		if (ret == IMG_ERROR_NOT_SUPPORTED)
			supp_check->unsupp_flags = unsupported;
	}

	return ret;
}

/*
 * @Function  core_supported_features
 */
int core_supported_features(struct vdec_features *features)
{
	struct vdecdd_dddev_context *dd_dev_ctx;

	VDEC_ASSERT(global_core_ctx);

	dd_dev_ctx = global_core_ctx->dev_ctx;
	VDEC_ASSERT(dd_dev_ctx);
	if (!dd_dev_ctx)
		return IMG_ERROR_NOT_INITIALISED;

	return decoder_supported_features(dd_dev_ctx->dec_context, features);
}

/*
 * @Function core_stream_stop
 */
int core_stream_stop(unsigned int res_str_id)
{
	int ret = IMG_SUCCESS;
	struct vdecdd_str_unit *stop_unit;
	struct vdecdd_ddstr_ctx *ddstr_ctx;
	struct core_stream_context *core_str_ctx;

	/*
	 * Stream based messages without a device context
	 * must have a stream ID.
	 */
	VDEC_ASSERT(res_str_id);

	if (res_str_id == 0) {
		pr_err("Invalid params passed to %s\n", __func__);
		return IMG_ERROR_INVALID_PARAMETERS;
	}

	/* Get access to stream context.. */
	ret = rman_get_resource(res_str_id, VDECDD_STREAM_TYPE_ID,
				(void **)&core_str_ctx, NULL);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	VDEC_ASSERT(core_str_ctx);

	ddstr_ctx = core_str_ctx->dd_str_ctx;

	/* Validate input arguments */
	VDEC_ASSERT(ddstr_ctx);

	/*
	 * Disregard this stop request if the stream is currently
	 * stopped or being stopped.
	 */
	if (ddstr_ctx->dd_str_state == VDECDD_STRSTATE_PLAYING) {
		vdecddutils_create_strunit(&stop_unit, NULL);
		if (!stop_unit) {
			pr_err("Failed to allocate memory for stop unit\n");
			return IMG_ERROR_OUT_OF_MEMORY;
		}
		memset(stop_unit, 0, sizeof(*stop_unit));

		stop_unit->str_unit_type = VDECDD_STRUNIT_STOP;
		stop_unit->str_unit_tag = NULL;
		stop_unit->decode = FALSE;

		/*
		 * Since the stop is now to be passed to the decoder signal
		 * that we're stopping.
		 */
		ddstr_ctx->dd_str_state = VDECDD_STRSTATE_STOPPING;
		decoder_stream_process_unit(ddstr_ctx->dec_ctx, stop_unit);
		core_str_ctx->stopped = TRUE;
		vdecddutils_free_strunit(stop_unit);
	}

	return ret;
}

/*
 * @Function              core_is_stream_idle
 */
static unsigned char core_is_stream_idle(struct vdecdd_ddstr_ctx *dd_str_ctx)
{
	unsigned char is_stream_idle;

	is_stream_idle = decoder_is_stream_idle(dd_str_ctx->dec_ctx);

	return is_stream_idle;
}

/*
 * @Function              core_stream_destroy
 */
int core_stream_destroy(unsigned int res_str_id)
{
	struct vdecdd_ddstr_ctx *ddstr_ctx;
	struct core_stream_context *core_str_ctx;
	int ret;

	/*
	 * Stream based messages without a device context
	 * must have a stream ID.
	 */
	VDEC_ASSERT(res_str_id);

	if (res_str_id == 0) {
		pr_err("Invalid params passed to %s\n", __func__);
		return IMG_ERROR_INVALID_PARAMETERS;
	}

	/* Get access to stream context.. */
	ret = rman_get_resource(res_str_id, VDECDD_STREAM_TYPE_ID,
				(void **)&core_str_ctx, NULL);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	VDEC_ASSERT(core_str_ctx);

	ddstr_ctx = core_str_ctx->dd_str_ctx;

	/* Validate input arguments */
	VDEC_ASSERT(ddstr_ctx);

	ret = core_stream_stop(res_str_id);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	lst_remove(&global_core_ctx->core_str_ctx, core_str_ctx);

	/* Destroy stream if idle otherwise wait and do it later */
	if (core_is_stream_idle(ddstr_ctx))
		rman_free_resource(ddstr_ctx->res_handle);

	pr_debug("Core stream destroy successfully\n");
	/* Return success.. */
	return IMG_SUCCESS;
}

static int
core_picture_attach_resources(struct core_stream_context *core_str_ctx,
			      struct vdecdd_str_unit *str_unit, unsigned char check)
{
	unsigned int ret = IMG_SUCCESS;

	/*
	 * Take sequence header from cache.
	 * Note: sequence header id must be set in PICTURE_START unit
	 */
	str_unit->seq_hdr_info = resource_list_getbyid(&core_str_ctx->seq_hdr_list,
						       str_unit->seq_hdr_id);

	/* Check is not needed e.g. when freeing resources at stream destroy */
	if (check && !str_unit->seq_hdr_info) {
		pr_err("[USERSID=0x%08X] Sequence header not available for current picture while attaching",
		       core_str_ctx->dd_str_ctx->str_config_data.user_str_id);
		ret = IMG_ERROR_NOT_SUPPORTED;
	}

	return ret;
}

/*
 * @Function core_handle_processed_unit
 */
static int core_handle_processed_unit(struct core_stream_context *c_str_ctx,
				      struct vdecdd_str_unit *str_unit)
{
	struct bspp_bitstr_seg *bstr_seg;
	struct vdecdd_ddstr_ctx *dd_str_ctx = c_str_ctx->dd_str_ctx;
	int ret;
	struct core_context *g_ctx = global_core_ctx;

	pr_debug("%s stream unit type = %d\n", __func__, str_unit->str_unit_type);
	/* check for type of the unit */
	switch (str_unit->str_unit_type) {
	case VDECDD_STRUNIT_SEQUENCE_START:
		/* nothing to be done as sps is maintained till it changes */
		break;

	case VDECDD_STRUNIT_PICTURE_START:
		/* Loop over bit stream segments.. */
		bstr_seg = (struct bspp_bitstr_seg *)
			lst_removehead(&str_unit->bstr_seg_list);

		while (bstr_seg) {
			lst_add(&c_str_ctx->vxd_dec_context->seg_list, bstr_seg);
			if (bstr_seg->bstr_seg_flag & VDECDD_BSSEG_LASTINBUFF &&
			    dd_str_ctx->dd_str_state != VDECDD_STRSTATE_STOPPED) {
				struct vdecdd_ddbuf_mapinfo *ddbuf_map_info;
				/* Get access to map info context.. */
				ret = rman_get_resource(bstr_seg->bufmap_id, VDECDD_BUFMAP_TYPE_ID,
							(void **)&ddbuf_map_info, NULL);
				VDEC_ASSERT(ret == IMG_SUCCESS);
				if (ret != IMG_SUCCESS)
					return ret;

				g_ctx->vxd_str_processed_cb(c_str_ctx->vxd_dec_context,
					VXD_CB_STRUNIT_PROCESSED,
					bstr_seg->bufmap_id, 0);
			}
			/* Get next segment. */
			bstr_seg = (struct bspp_bitstr_seg *)
				lst_removehead(&str_unit->bstr_seg_list);
		}
		break;

	case VDECDD_STRUNIT_PICTURE_END:
		g_ctx->vxd_str_processed_cb(c_str_ctx->vxd_dec_context,
			VXD_CB_PICT_END, 0xFFFF, 0);
		break;

	case VDECDD_STRUNIT_STOP:
		/*
		 * Signal that the stream has been stopped in the
		 * device driver.
		 */
		dd_str_ctx->dd_str_state = VDECDD_STRSTATE_STOPPED;

		break;

	default:
		pr_err("Invalid stream unit type passed\n");
		return IMG_ERROR_GENERIC_FAILURE;
	}

#ifdef DEBUG_DECODER_DRIVER
	pr_info("[SID=0x%08X] [UTYPE=0x%08X] PROCESSED",
		dd_str_ctx->res_str_id,
		str_unit->str_unit_type);
#endif

	/* Return success.. */
	return IMG_SUCCESS;
}

static int
core_handle_decoded_picture(struct core_stream_context *core_str_ctx,
			    struct vdecdd_picture *picture, unsigned int type)
{
	/* Pick the client image buffer. */
	struct vdecdd_ddbuf_mapinfo *pictbuf_mapinfo = picture->disp_pict_buf.pict_buf;

	VDEC_ASSERT(pictbuf_mapinfo);
	if (!pictbuf_mapinfo)
		return IMG_ERROR_COULD_NOT_OBTAIN_RESOURCE;

	global_core_ctx->vxd_str_processed_cb(core_str_ctx->vxd_dec_context,
		(enum vxd_cb_type)type, pictbuf_mapinfo->buf_map_id,
		picture->dec_pict_info->err_flags);
	return IMG_SUCCESS;
}

static int core_stream_processed_cb(void *handle, int cb_type, void *cb_item)
{
	int ret;
	struct core_stream_context *core_str_ctx =
		(struct core_stream_context *)handle;
	VDEC_ASSERT(core_str_ctx);
	if (!core_str_ctx) {
		pr_err("NULL handle passed to core callback\n");
		return IMG_ERROR_GENERIC_FAILURE;
	}

	pr_debug("%s callback type = %d\n", __func__, cb_type);
	/* Based on callback type, retrieve the item */
	switch (cb_type) {
	case VXD_CB_STRUNIT_PROCESSED:
	{
		struct vdecdd_str_unit *str_unit =
			(struct vdecdd_str_unit *)cb_item;
		VDEC_ASSERT(str_unit);
		if (!str_unit) {
			pr_err("NULL item passed to core callback type STRUNIT_PROCESSED\n");
			return IMG_ERROR_GENERIC_FAILURE;
		}
		ret = core_handle_processed_unit(core_str_ctx, str_unit);
		if (ret != IMG_SUCCESS) {
			pr_err("core_handle_processed_unit returned error\n");
			return ret;
		}
		break;
	}

	case VXD_CB_PICT_DECODED:
	case VXD_CB_PICT_DISPLAY:
	case VXD_CB_PICT_RELEASE:
	{
		struct vdecdd_picture *picture = (struct vdecdd_picture *)cb_item;

		if (!picture) {
			pr_err("NULL item passed to core callback type PICTURE_DECODED\n");
			return IMG_ERROR_GENERIC_FAILURE;
		}
		ret = core_handle_decoded_picture(core_str_ctx, picture, cb_type);
		break;
	}

	case VXD_CB_STR_END:
		global_core_ctx->vxd_str_processed_cb(core_str_ctx->vxd_dec_context,
			(enum vxd_cb_type)cb_type, 0, 0);
		ret = IMG_SUCCESS;

		break;
	case VXD_CB_ERROR_FATAL:
		/*
		 * Whenever the error case occurs, we need to handle the error case.
		 * Need to forward this error to v4l2 glue layer.
		 * in this case the cb_item is the error_code as we may not have
		 * an associated picture.
		 */
		global_core_ctx->vxd_str_processed_cb(core_str_ctx->vxd_dec_context,
			(enum vxd_cb_type)cb_type, 0, *((unsigned int *)cb_item));
		ret = IMG_SUCCESS;
		break;
	default:
		return 0;
	}

	return ret;
}

static int core_decoder_queries(void *handle, int query, void *item)
{
	struct core_stream_context *core_str_ctx =
		(struct core_stream_context *)handle;
	VDEC_ASSERT(core_str_ctx);
	if (!core_str_ctx) {
		pr_err("NULL handle passed to %s callback\n", __func__);
		return IMG_ERROR_GENERIC_FAILURE;
	}

	switch (query) {
	case DECODER_CORE_GET_RES_LIMIT:
	{
		unsigned int num_img_bufs;
		unsigned int num_res;

		num_img_bufs = resource_list_getnum(&core_str_ctx->pict_buf_list);

		/* Return the number of internal resources. */
		num_res = core_str_ctx->pict_resinfo.pict_res_num;

		/* Return the minimum of the two. */
		*((unsigned int *)item) = vdec_size_min(num_img_bufs, num_res);
	}
	break;

	default:
		return IMG_ERROR_GENERIC_FAILURE;
	}
	return IMG_SUCCESS;
}

static int
core_free_common_picture_resource(struct core_stream_context *core_str_ctx,
				  struct vdecdd_pict_resint *pict_resint)
{
	int ret = IMG_SUCCESS;

	if (pict_resint->mb_param_buf && pict_resint->mb_param_buf->ddbuf_info.hndl_memory) {
#ifdef DEBUG_DECODER_DRIVER
		pr_info("mmu_free for buff_id[%d]\n",
			pict_resint->mb_param_buf->ddbuf_info.buff_id);
#endif
		ret = mmu_free_mem(core_str_ctx->dd_str_ctx->mmu_str_handle,
				   &pict_resint->mb_param_buf->ddbuf_info);
		if (ret != IMG_SUCCESS)
			pr_err("MMU_Free for MBParam buffer failed with error %u", ret);

		kfree(pict_resint->mb_param_buf);
		pict_resint->mb_param_buf = NULL;
	}
	return ret;
}

static int core_free_resbuf(struct vdecdd_ddbuf_mapinfo **buf_handle, void *mmu_handle)
{
	int ret = IMG_SUCCESS;
	struct vdecdd_ddbuf_mapinfo *buf = *buf_handle;

	if (buf) {
		if (buf->ddbuf_info.hndl_memory) {
			ret = mmu_free_mem(mmu_handle, &buf->ddbuf_info);
			VDEC_ASSERT(ret == IMG_SUCCESS);
		}
		kfree(buf);
		*buf_handle = NULL;
	}
	return ret;
}

/*
 * @Function              core_free_picture_resource
 */
static int
core_free_picture_resource(struct core_stream_context *core_strctx,
			   struct vdecdd_pict_resint *pict_resint)
{
	int result = IMG_SUCCESS;

	/* Check input arguments */
	if (!core_strctx || !pict_resint) {
		VDEC_ASSERT(0);
		return -EINVAL;
	}

	result = core_free_common_picture_resource(core_strctx, pict_resint);

	VDEC_ASSERT(core_strctx->std_spec_ops);
	if (core_strctx->std_spec_ops->free_picture_resource)
		core_strctx->std_spec_ops->free_picture_resource(core_strctx,
			pict_resint);

#ifdef SEQ_RES_NEEDED
	if (pict_resint->seq_resint) {
		resource_item_return(&pict_resint->seq_resint->ref_count);
		pict_resint->seq_resint = 0;
	}
#endif

	if (result == IMG_SUCCESS)
		kfree(pict_resint);

	return result;
}

/*
 * @Function              core_free_sequence_resource
 */
#ifdef SEQ_RES_NEEDED
static int
core_free_common_sequence_resource(struct core_stream_context *core_strctx,
				   struct vdecdd_seq_resint *seqres_int)
{
	int result;

	result = core_free_resbuf(&seqres_int->err_pict_buf,
				  core_strctx->dd_str_ctx->mmu_str_handle);
	if (result != IMG_SUCCESS)
		pr_err("MMU_Free for Error Recover Frame Store buffer failed with error %u",
		       result);

	return result;
}

static void
core_free_sequence_resource(struct core_stream_context *core_strctx,
			    struct vdecdd_seq_resint *seqres_int)
{
	VDEC_ASSERT(core_strctx->std_spec_ops);
	core_free_common_sequence_resource(core_strctx, seqres_int);

	if (core_strctx->std_spec_ops->free_sequence_resource)
		core_strctx->std_spec_ops->free_sequence_resource(core_strctx, seqres_int);

	kfree(seqres_int);
}
#endif

/*
 * @Function              core_stream_resource_deprecate
 */
static int core_stream_resource_deprecate(struct core_stream_context *core_str_ctx)
{
	struct vdecdd_pict_resint *picres_int;
	int ret;

	/* Free all "old" picture resources since these should now be unused. */
	picres_int = lst_first(&core_str_ctx->old_pict_res_list);
	while (picres_int) {
		if (picres_int->ref_cnt != 0) {
			pr_warn("[USERSID=0x%08X] Internal resource should be unused since it has been deprecated before",
				core_str_ctx->dd_str_ctx->str_config_data.user_str_id);

			picres_int = lst_next(picres_int);
		} else {
			struct vdecdd_pict_resint *picres_int_to_remove = picres_int;

			picres_int = lst_next(picres_int);

			lst_remove(&core_str_ctx->old_pict_res_list, picres_int_to_remove);
			ret = core_free_picture_resource(core_str_ctx, picres_int_to_remove);
			VDEC_ASSERT(ret == IMG_SUCCESS);
			if (ret != IMG_SUCCESS)
				return ret;
		}
	}

	/* Move all "active" picture resources to the "old" list if they are still in use. */
	picres_int = lst_removehead(&core_str_ctx->pict_res_list);
	while (picres_int) {
		/* Remove picture resource from the list. */
		ret = resource_list_remove(&core_str_ctx->aux_pict_res_list, picres_int);

		/* IMG_ERROR_COULD_NOT_OBTAIN_RESOURCE is a valid return code
		 * e.g. during reconfigure we are clearing the sPictBufferList list
		 * and then try to remove the buffers again from the same list (empty now)
		 * though core UNMAP_BUF messages
		 */
		if (ret != IMG_SUCCESS && ret != IMG_ERROR_COULD_NOT_OBTAIN_RESOURCE) {
			pr_err("[USERSID=0x%08X] Failed to remove picture resource",
			       core_str_ctx->dd_str_ctx->str_config_data.user_str_id);
			return ret;
		}
		/*
		 * If the active resource is not being used, free now.
		 * Otherwise add to the old list to be freed later.
		 */
		if (picres_int->ref_cnt == 0) {
			ret = core_free_picture_resource(core_str_ctx, picres_int);
			VDEC_ASSERT(ret == IMG_SUCCESS);
			if (ret != IMG_SUCCESS)
				return ret;
		} else {
			lst_add(&core_str_ctx->old_pict_res_list, picres_int);
		}
		picres_int = lst_removehead(&core_str_ctx->pict_res_list);
	}

	/* Reset the resource configuration. */
	memset(&core_str_ctx->pict_resinfo, 0, sizeof(core_str_ctx->pict_resinfo));

#ifdef SEQ_RES_NEEDED
	{
		struct vdecdd_seq_resint *seqres_int;

		/* Free all "old" sequence resources since these should now be unused. */
		seqres_int = lst_first(&core_str_ctx->old_seq_res_list);
		while (seqres_int) {
			if (seqres_int->ref_count != 0) {
				pr_warn("[USERSID=0x%08X] Internal sequence resource should be unused since it has been deprecated before",
					core_str_ctx->dd_str_ctx->str_config_data.user_str_id);
				seqres_int = lst_next(seqres_int);
			} else {
				struct vdecdd_seq_resint *seqres_int_to_remove = seqres_int;

				seqres_int = lst_next(seqres_int);

				lst_remove(&core_str_ctx->old_seq_res_list, seqres_int_to_remove);
				core_free_sequence_resource(core_str_ctx, seqres_int_to_remove);
			}
		}

		/* Move all "active" sequence resources to the "old"
		 * list if they are still in use.
		 */
		seqres_int = lst_removehead(&core_str_ctx->seq_res_list);
		while (seqres_int) {
			/*
			 * If the active resource is not being used, free now.
			 * Otherwise add to the old list to be freed later.
			 */
			seqres_int->ref_count == 0 ? core_free_sequence_resource(core_str_ctx,
				seqres_int) :
			lst_add(&core_str_ctx->old_seq_res_list, seqres_int);

			seqres_int = lst_removehead(&core_str_ctx->seq_res_list);
		}

		/* Reset the resource configuration. */
		memset(&core_str_ctx->seq_resinfo, 0, sizeof(core_str_ctx->seq_resinfo));
	}
#endif
	return IMG_SUCCESS;
}

/*
 * @Function              core_stream_resource_destroy
 */
static int core_stream_resource_destroy(struct core_stream_context *core_str_ctx)
{
	struct vdecdd_pict_resint *picres_int;
	int ret;

	/* Remove any "active" picture resources allocated for this stream. */
	picres_int = lst_removehead(&core_str_ctx->pict_res_list);
	while (picres_int) {
		ret = core_free_picture_resource(core_str_ctx, picres_int);
		VDEC_ASSERT(ret == IMG_SUCCESS);
		if (ret != IMG_SUCCESS)
			return ret;

		picres_int = lst_removehead(&core_str_ctx->pict_res_list);
	}

	/* Remove any "old" picture resources allocated for this stream. */
	picres_int = lst_removehead(&core_str_ctx->old_pict_res_list);
	while (picres_int) {
		ret = core_free_picture_resource(core_str_ctx, picres_int);
		VDEC_ASSERT(ret == IMG_SUCCESS);
		if (ret != IMG_SUCCESS)
			return ret;

		picres_int = lst_removehead(&core_str_ctx->old_pict_res_list);
	}

	/* Reset the resource configuration. */
	memset(&core_str_ctx->pict_resinfo, 0, sizeof(core_str_ctx->pict_resinfo));

#ifdef SEQ_RES_NEEDED
	{
		struct vdecdd_seq_resint *seqres_int;

		/* Remove any "active" sequence resources allocated for this stream. */
		seqres_int = lst_removehead(&core_str_ctx->seq_res_list);
		while (seqres_int) {
			core_free_sequence_resource(core_str_ctx, seqres_int);
			seqres_int = lst_removehead(&core_str_ctx->seq_res_list);
		}

		/* Remove any "old" sequence resources allocated for this stream. */
		seqres_int = lst_removehead(&core_str_ctx->old_seq_res_list);
		while (seqres_int) {
			core_free_sequence_resource(core_str_ctx, seqres_int);
			seqres_int = lst_removehead(&core_str_ctx->old_seq_res_list);
		}

		/* Reset the resource configuration. */
		memset(&core_str_ctx->seq_resinfo, 0, sizeof(core_str_ctx->seq_resinfo));
	}
#endif
	return IMG_SUCCESS;
}

/*
 * @Function core_fn_free_stream_unit
 */
static int core_fn_free_stream_unit(struct vdecdd_str_unit *str_unit, void *param)
{
	struct core_stream_context *core_str_ctx = (struct core_stream_context *)param;
	unsigned int ret = IMG_SUCCESS;

	/* Attach picture resources where required. */
	if (str_unit->str_unit_type == VDECDD_STRUNIT_PICTURE_START)
		/*
		 * Do not force attachment because the resources can be
		 * unattached yet, e.g. in case of not yet processed picture
		 * units
		 */
		ret = core_picture_attach_resources(core_str_ctx, str_unit, FALSE);

	str_unit->decode = FALSE;

	return ret;
}

/*
 * @Function  core_fn_free_stream
 */
static void core_fn_free_stream(void *param)
{
	int ret;
	struct vdecdd_ddstr_ctx *dd_str_context;
	struct vdecdd_dddev_context *dd_dev_ctx;
	struct core_stream_context *core_str_ctx;

	/* Validate input arguments */
	VDEC_ASSERT(param);

	core_str_ctx = (struct core_stream_context *)param;

	dd_str_context = core_str_ctx->dd_str_ctx;

	VDEC_ASSERT(dd_str_context);
	if (!dd_str_context)
		return;

	dd_dev_ctx = dd_str_context->dd_dev_context;
	VDEC_ASSERT(dd_dev_ctx);

	if (!lst_empty(&core_str_ctx->str_unit_list)) {
		/*
		 * Try and empty the list. Since this function is tearing down the core stream,
		 * test result using assert and continue to tidy-up as much as possible.
		 */
		ret = resource_list_empty(&core_str_ctx->str_unit_list, FALSE,
					  (resource_pfn_freeitem)core_fn_free_stream_unit,
					  core_str_ctx);
		VDEC_ASSERT(ret == IMG_SUCCESS);
	}

	if (!lst_empty(&core_str_ctx->pict_buf_list)) {
		/*
		 * Try and empty the list. Since this function is tearing down the core stream,
		 * test result using assert and continue to tidy-up as much as possible.
		 */
		ret = resource_list_empty(&core_str_ctx->pict_buf_list, TRUE, NULL, NULL);
		VDEC_ASSERT(ret == IMG_SUCCESS);
	}

	if (!lst_empty(&core_str_ctx->aux_pict_res_list)) {
		/*
		 * Try and empty the list. Since this function is tearing down the core stream,
		 * test result using assert and continue to tidy-up as much as possible.
		 */
		ret = resource_list_empty(&core_str_ctx->aux_pict_res_list, TRUE, NULL, NULL);
		VDEC_ASSERT(ret == IMG_SUCCESS);
	}

	if (!lst_empty(&core_str_ctx->seq_hdr_list)) {
		/*
		 * Try and empty the list. Since this function is tearing down the core stream,
		 * test result using assert and continue to tidy-up as much as possible.
		 */
		ret = resource_list_empty(&core_str_ctx->seq_hdr_list, FALSE, NULL, NULL);
		VDEC_ASSERT(ret == IMG_SUCCESS);
	}

	/* Destroy stream in the Decoder. */
	if (dd_str_context->dec_ctx) {
		ret = decoder_stream_destroy(dd_str_context->dec_ctx, FALSE);
		VDEC_ASSERT(ret == IMG_SUCCESS);
		dd_str_context->dec_ctx = NULL;
	}

	core_stream_resource_destroy(core_str_ctx);

	/* Destroy the MMU context for this stream. */
	if (dd_str_context->mmu_str_handle) {
		ret = mmu_stream_destroy(dd_str_context->mmu_str_handle);

		VDEC_ASSERT(ret == IMG_SUCCESS);
		dd_str_context->mmu_str_handle = NULL;
	}

	/* Destroy the stream resources. */
	if (dd_str_context->res_buck_handle) {
		rman_destroy_bucket(dd_str_context->res_buck_handle);
		dd_str_context->res_buck_handle = NULL;
	}

	/* Free stream context. */
	kfree(dd_str_context);

	/* Free the stream context. */
	kfree(core_str_ctx);
}

/*
 * @Function  core_is_unsupported
 */
static unsigned char core_is_unsupported(struct vdec_unsupp_flags *unsupp_flags)
{
	unsigned char unsupported = FALSE;

	if (unsupp_flags->str_cfg || unsupp_flags->seq_hdr ||
	    unsupp_flags->pict_hdr || unsupp_flags->str_opcfg ||
	    unsupp_flags->op_bufcfg)
		unsupported = TRUE;

	return unsupported;
}

int core_stream_create(void *vxd_dec_ctx_arg,
		       const struct vdec_str_configdata *str_cfg_data,
		       unsigned int *res_str_id)
{
	int ret;
	struct vdecdd_ddstr_ctx *dd_str_context;
	struct vdecdd_supp_check supp_check;
	struct vdecdd_dddev_context *dd_dev_ctx;
	struct core_stream_context *core_str_ctx;

	/* Validate input arguments */
	VDEC_ASSERT(str_cfg_data);
	VDEC_ASSERT(res_str_id);

	VDEC_ASSERT(global_core_ctx);
	dd_dev_ctx = global_core_ctx->dev_ctx;

	VDEC_ASSERT(dd_dev_ctx);
	if (!dd_dev_ctx)
		return IMG_ERROR_NOT_INITIALISED;

	/* Allocate Core Stream Context */
	core_str_ctx = kzalloc(sizeof(*core_str_ctx), GFP_KERNEL);
	if (!core_str_ctx)
		return IMG_ERROR_OUT_OF_MEMORY;

	core_str_ctx->core_ctx = global_core_ctx;
	core_str_ctx->vxd_dec_context = (struct vxd_dec_ctx *)vxd_dec_ctx_arg;

	((struct vxd_dec_ctx *)vxd_dec_ctx_arg)->dev_ctx = global_core_ctx->dev_ctx;

	/* register callback for firmware response */
	core_str_ctx->vxd_dec_context->cb = (decode_cb)core_fw_response_cb;

	lst_init(&core_str_ctx->pict_buf_list);
	lst_init(&core_str_ctx->pict_res_list);
	lst_init(&core_str_ctx->old_pict_res_list);
	lst_init(&core_str_ctx->aux_pict_res_list);
	lst_init(&core_str_ctx->seq_hdr_list);
	lst_init(&core_str_ctx->str_unit_list);

#ifdef SEQ_RES_NEEDED
	lst_init(&core_str_ctx->seq_res_list);
	lst_init(&core_str_ctx->old_seq_res_list);
#endif

	/* Allocate device stream context.. */
	dd_str_context = kzalloc(sizeof(*dd_str_context), GFP_KERNEL);
	VDEC_ASSERT(dd_str_context);
	if (!dd_str_context) {
		kfree(core_str_ctx);
		core_str_ctx = NULL;
		return IMG_ERROR_OUT_OF_MEMORY;
	}

	dd_str_context->dd_dev_context = dd_dev_ctx;
	core_str_ctx->dd_str_ctx = dd_str_context;

	/* Check stream configuration. */
	memset(&supp_check, 0x0, sizeof(supp_check));
	ret = core_check_decoder_support(dd_dev_ctx, str_cfg_data, NULL, NULL, NULL, &supp_check);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		goto error;

	if (core_is_unsupported(&supp_check.unsupp_flags)) {
		ret = IMG_ERROR_NOT_SUPPORTED;
		goto error;
	}

	/* Create a bucket for the resources.. */
	ret = rman_create_bucket(&dd_str_context->res_buck_handle);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		goto error;

	/* Register the stream as a device resource.. */
	ret = rman_register_resource(dd_dev_ctx->res_buck_handle,
				     VDECDD_STREAM_TYPE_ID,
				     core_fn_free_stream, core_str_ctx,
				     &dd_str_context->res_handle,
				     &dd_str_context->res_str_id);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		goto error;

	/* Create unique Stream Id */
	dd_str_context->km_str_id = core_str_ctx->vxd_dec_context->stream.id;

	/*
	 * Create stream in the Decoder.
	 * NOTE: this must take place first since it creates the MMU context.
	 */
	ret = decoder_stream_create(dd_dev_ctx->dec_context, *str_cfg_data,
				    dd_str_context->km_str_id,
				    &dd_str_context->mmu_str_handle,
				    core_str_ctx->vxd_dec_context,
				    core_str_ctx, &dd_str_context->dec_ctx,
				    (void *)core_stream_processed_cb,
				    (void *)core_decoder_queries);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		goto error;

	/* Setup stream context.. */
	dd_str_context->str_config_data = *str_cfg_data;
	dd_str_context->dd_str_state = VDECDD_STRSTATE_STOPPED;

#ifdef DEBUG_DECODER_DRIVER
	pr_info("[SID=0x%08X] New stream created [USERSID=0x%08X]",
		dd_str_context->res_str_id, str_cfg_data->user_str_id);
#endif

	*res_str_id = dd_str_context->res_str_id;
	if (str_cfg_data->vid_std > 0 &&  str_cfg_data->vid_std <= VDEC_STD_MAX) {
		core_str_ctx->std_spec_ops = &std_specific_ops[str_cfg_data->vid_std - 1];
	} else {
		pr_err("%s: Invalid parameters\n", __func__);
		return IMG_ERROR_INVALID_PARAMETERS;
	}

	lst_add(&global_core_ctx->core_str_ctx, core_str_ctx);

	/* Return success.. */
	return IMG_SUCCESS;

error:
	if (dd_str_context->res_handle)
		rman_free_resource(dd_str_context->res_handle);
	else
		core_fn_free_stream(core_str_ctx);

	return ret;
}

static int
core_get_resource_availability(struct core_stream_context *core_str_ctx)
{
	unsigned int avail = ~0;

	if (resource_list_getnumavail(&core_str_ctx->pict_buf_list) == 0)
		avail &= ~CORE_AVAIL_PICTBUF;

	if (resource_list_getnumavail(&core_str_ctx->aux_pict_res_list) == 0)
		avail &= ~CORE_AVAIL_PICTRES;

	if (global_avail_slots == 0)
		avail &= ~CORE_AVAIL_CORE;

	return avail;
}

static int
core_stream_set_pictbuf_config(struct vdecdd_ddstr_ctx *dd_str_ctx,
			       struct vdec_pict_bufconfig *pictbuf_cfg)
{
	int ret;

	/* Validate input arguments */
	VDEC_ASSERT(dd_str_ctx);
	VDEC_ASSERT(pictbuf_cfg);

	/*
	 * If there are no buffers mapped or the configuration is not set
	 * (only done when reconfiguring output) then calculate the output
	 * picture buffer layout.
	 */
	if (dd_str_ctx->map_buf_info.num_buf == 0 ||
	    dd_str_ctx->disp_pict_buf.buf_config.buf_size == 0) {
		struct vdecdd_supp_check supp_check;
		struct vdecdd_ddpict_buf disp_pictbuf;

		memset(&disp_pictbuf, 0, sizeof(disp_pictbuf));

		disp_pictbuf.buf_config = *pictbuf_cfg;

		/*
		 * Ensure that the external picture buffer information
		 * is compatible with the hardware and convert to internal
		 * driver representation.
		 */
		ret = vdecddutils_convert_buffer_config(&dd_str_ctx->str_config_data,
							&disp_pictbuf.buf_config,
							&disp_pictbuf.rend_info);
		VDEC_ASSERT(ret == IMG_SUCCESS);
		if (ret != IMG_SUCCESS)
			goto error;

		/*
		 * Provide the current state for validation against the new
		 * buffer configuration.
		 */
		memset(&supp_check, 0, sizeof(supp_check));
		supp_check.disp_pictbuf = &disp_pictbuf;

		if (dd_str_ctx->comseq_hdr_info.max_frame_size.width)
			supp_check.comseq_hdrinfo = &dd_str_ctx->comseq_hdr_info;

		if (dd_str_ctx->str_op_configured)
			supp_check.op_cfg = &dd_str_ctx->opconfig;

		ret = core_check_decoder_support(dd_str_ctx->dd_dev_context,
						 &dd_str_ctx->str_config_data,
						 &dd_str_ctx->prev_comseq_hdr_info,
						 &dd_str_ctx->prev_pict_hdr_info,
						 &dd_str_ctx->map_buf_info,
						 &supp_check);
		VDEC_ASSERT(ret == IMG_SUCCESS);
		if (ret != IMG_SUCCESS)
			goto error;

		if (core_is_unsupported(&supp_check.unsupp_flags)) {
			ret = IMG_ERROR_NOT_SUPPORTED;
			goto error;
		}

		dd_str_ctx->disp_pict_buf = disp_pictbuf;
	} else {
		/*
		 * Check configuration of buffer matches that for stream
		 * including any picture buffers that are already mapped.
		 */
		if (memcmp(pictbuf_cfg, &dd_str_ctx->disp_pict_buf.buf_config,
			   sizeof(*pictbuf_cfg))) {
			/*
			 * Configuration of output buffer doesn't match the
			 * rest.
			 */
			pr_err("[SID=0x%08X] All output buffers must have the same properties.",
			       dd_str_ctx->res_str_id);
			ret = IMG_ERROR_INVALID_PARAMETERS;
			goto error;
		}
	}

	/* Return success.. */
	return IMG_SUCCESS;

error:
	return ret;
}

int
core_stream_set_output_config(unsigned int res_str_id,
			      struct vdec_str_opconfig *str_opcfg,
			      struct vdec_pict_bufconfig *pict_bufcfg_handle)
{
	struct vdecdd_supp_check supp_check;
	struct vdec_pict_bufconfig pict_buf_cfg;
	struct vdec_pict_rendinfo disp_pict_rend_info;
	int ret;

	struct vdecdd_ddstr_ctx *dd_str_context;
	struct core_stream_context      *core_str_ctx;

	/*
	 * Stream based messages without a device context
	 * must have a stream ID.
	 */
	VDEC_ASSERT(res_str_id);

	/* Get access to stream context */
	ret = rman_get_resource(res_str_id, VDECDD_STREAM_TYPE_ID, (void **)&core_str_ctx, NULL);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	dd_str_context = core_str_ctx->dd_str_ctx;

	VDEC_ASSERT(dd_str_context);
	VDEC_ASSERT(str_opcfg);

	memset(&supp_check, 0, sizeof(supp_check));
	if (core_str_ctx->new_seq)
		supp_check.comseq_hdrinfo = &dd_str_context->comseq_hdr_info;
	else
		supp_check.comseq_hdrinfo = NULL;

	supp_check.op_cfg = str_opcfg;

	/*
	 * Validate stream output configuration against display
	 * buffer properties if no new picture buffer configuration
	 * is provided.
	 */
	if (!pict_bufcfg_handle) {
		VDEC_ASSERT(dd_str_context->disp_pict_buf.rend_info.rendered_size);
		supp_check.disp_pictbuf = &dd_str_context->disp_pict_buf;
	}

	/* Validate output configuration. */
	ret = core_check_decoder_support(dd_str_context->dd_dev_context,
					 &dd_str_context->str_config_data,
					 &dd_str_context->prev_comseq_hdr_info,
					 &dd_str_context->prev_pict_hdr_info,
					 &dd_str_context->map_buf_info,
					 &supp_check);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return IMG_SUCCESS;

	if (core_is_unsupported(&supp_check.unsupp_flags))
		return IMG_ERROR_NOT_SUPPORTED;

	/* Update the stream output configuration. */
	dd_str_context->opconfig = *str_opcfg;

	/* Mark output as configured. */
	dd_str_context->str_op_configured = TRUE;

	if (pict_bufcfg_handle) {
		/*
		 * Clear/invalidate the latest picture buffer configuration
		 * since it is easier to reuse the set function to calculate
		 * for this new output configuration than to determine
		 * compatibility. Keep a copy beforehand just in case the new
		 * configuration is invalid.
		 */
		if (dd_str_context->disp_pict_buf.rend_info.rendered_size != 0) {
			pict_buf_cfg = dd_str_context->disp_pict_buf.buf_config;
			disp_pict_rend_info = dd_str_context->disp_pict_buf.rend_info;

			memset(&dd_str_context->disp_pict_buf.buf_config, 0,
			       sizeof(dd_str_context->disp_pict_buf.buf_config));
			memset(&dd_str_context->disp_pict_buf.rend_info, 0,
			       sizeof(dd_str_context->disp_pict_buf.rend_info));
		}

		/*
		 * Recalculate the picture buffer internal layout from the
		 * externalconfiguration. These settings provided by the
		 * allocator should be adhered to since the display process
		 * will expect the decoder to use them.
		 * If the configuration is invalid we need to leave the
		 * decoder state as it was before.
		 */
		ret = core_stream_set_pictbuf_config(dd_str_context, pict_bufcfg_handle);
		VDEC_ASSERT(ret == IMG_SUCCESS);
		if (ret != IMG_SUCCESS && dd_str_context->disp_pict_buf.rend_info.rendered_size
		    != 0) {
			/* Restore old picture buffer configuration */
			dd_str_context->disp_pict_buf.buf_config =
				pict_buf_cfg;
			dd_str_context->disp_pict_buf.rend_info =
				disp_pict_rend_info;
			return ret;
		}
	} else if (core_is_unsupported(&supp_check.unsupp_flags)) {
		return IMG_ERROR_NOT_SUPPORTED;
	}

	/* Return success.. */
	return ret;
}

/*
 * @Function core_stream_play
 */
int core_stream_play(unsigned int res_str_id)
{
	int ret;
	struct vdecdd_ddstr_ctx *dd_str_context;
	struct core_stream_context *core_str_ctx;
	/* Picture buffer layout to use for decoding. */
	struct vdecdd_ddpict_buf *disp_pict_buf;
	struct vdec_str_opconfig *op_cfg;
	struct vdecdd_supp_check supp_check;

	/*
	 * Stream based messages without a device context
	 * must have a stream ID.
	 */
	VDEC_ASSERT(res_str_id);

	/* Get access to stream context.. */
	ret = rman_get_resource(res_str_id, VDECDD_STREAM_TYPE_ID,
				(void **)&core_str_ctx, NULL);

	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	dd_str_context = core_str_ctx->dd_str_ctx;

	VDEC_ASSERT(dd_str_context);

	/* Ensure we are stopped. */
	VDEC_ASSERT(dd_str_context->dd_str_state == VDECDD_STRSTATE_STOPPED);

	/* Set "playing". */
	dd_str_context->dd_str_state = VDECDD_STRSTATE_PLAYING;

	/* set that is it not yet in closed GOP */
	core_str_ctx->no_prev_refs_used = TRUE;

	disp_pict_buf = dd_str_context->disp_pict_buf.rend_info.rendered_size ?
		&dd_str_context->disp_pict_buf : NULL;
	op_cfg = dd_str_context->str_op_configured ?
		&dd_str_context->opconfig : NULL;

	if (disp_pict_buf && op_cfg) {
		VDEC_ASSERT(!disp_pict_buf->pict_buf);

		if (memcmp(&core_str_ctx->op_cfg, op_cfg,
			   sizeof(core_str_ctx->op_cfg)) ||
			memcmp(&core_str_ctx->disp_pict_buf, disp_pict_buf,
			       sizeof(core_str_ctx->disp_pict_buf)))
			core_str_ctx->new_op_cfg = TRUE;

		core_str_ctx->disp_pict_buf = *disp_pict_buf;
		core_str_ctx->op_cfg = *op_cfg;

		core_str_ctx->opcfg_set = TRUE;
	} else {
		core_str_ctx->opcfg_set = FALSE;
		/* Must not be decoding without output configuration */
		VDEC_ASSERT(0);
	}

	memset(&supp_check, 0, sizeof(supp_check));

	if (vdec_size_nz(core_str_ctx->comseq_hdr_info.max_frame_size))
		supp_check.comseq_hdrinfo = &core_str_ctx->comseq_hdr_info;

	if (core_str_ctx->opcfg_set) {
		supp_check.op_cfg = &core_str_ctx->op_cfg;
		supp_check.disp_pictbuf = &core_str_ctx->disp_pict_buf;
	}
	supp_check.non_cfg_req = TRUE;
	ret = core_check_decoder_support(dd_str_context->dd_dev_context,
					 &dd_str_context->str_config_data,
					 &dd_str_context->prev_comseq_hdr_info,
					 &dd_str_context->prev_pict_hdr_info,
					 &dd_str_context->map_buf_info,
					 &supp_check);
	if (ret != IMG_SUCCESS)
		return ret;

	/* Return success.. */
	return IMG_SUCCESS;
}

/*
 * @Function  core_deinitialise
 */
int core_deinitialise(void)
{
	struct vdecdd_dddev_context *dd_dev_ctx;
	int ret;

	dd_dev_ctx = global_core_ctx->dev_ctx;
	VDEC_ASSERT(dd_dev_ctx);
	if (!dd_dev_ctx)
		return IMG_ERROR_NOT_INITIALISED;

	ret = decoder_deinitialise(dd_dev_ctx->dec_context);
	VDEC_ASSERT(ret == IMG_SUCCESS);

	/* Free context resources.. */
	rman_destroy_bucket(dd_dev_ctx->res_buck_handle);

	rman_deinitialise();

	kfree(dd_dev_ctx);

	global_core_ctx->dev_ctx = NULL;

	kfree(global_core_ctx);
	global_core_ctx = NULL;

	is_core_initialized = FALSE;

	pr_debug("Core deinitialise successfully\n");
	return IMG_SUCCESS;
}

static int core_get_mb_num(unsigned int width, unsigned int height)
{
	/*
	 * Calculate the number of MBs needed for current video
	 * sequence settings.
	 */
	unsigned int width_mb  = ALIGN(width, VDEC_MB_DIMENSION) / VDEC_MB_DIMENSION;
	unsigned int height_mb = ALIGN(height, 2 * VDEC_MB_DIMENSION) / VDEC_MB_DIMENSION;

	return width_mb * height_mb;
}

static int core_common_bufs_getsize(struct core_stream_context *core_str_ctx,
				    const struct vdec_comsequ_hdrinfo *comseq_hdrinfo,
				    struct vdec_pict_size *max_pict_size,
				    struct core_pict_bufsize_info     *size_info,
				    struct core_seq_resinfo  *seq_res_info,
				    unsigned char *res_needed)
{
	enum vdec_vid_std vid_std = core_str_ctx->dd_str_ctx->str_config_data.vid_std;
	unsigned int std_idx = vid_std - 1;
	unsigned int mb_num = 0;

	if (core_str_ctx->dd_str_ctx->str_config_data.vid_std >= VDEC_STD_MAX)
		return IMG_ERROR_GENERIC_FAILURE;

	/* Reset the MB parameters buffer size. */
	size_info->mbparams_bufsize = 0;

	if (mbparam_allocinfo[std_idx].alloc_mbparam_bufs) {
		*res_needed = TRUE;

		/*
		 * Calculate the number of MBs needed for current video
		 * sequence settings.
		 */
		mb_num = core_get_mb_num(max_pict_size->width, max_pict_size->height);

		/* Calculate the final number of MBs needed. */
		mb_num += mbparam_allocinfo[std_idx].overalloc_mbnum;

		/* Calculate the MB params buffer size. */
		size_info->mbparams_bufsize = mb_num * mbparam_allocinfo[std_idx].mbparam_size;

		/* Adjust the buffer size for MSVDX. */
		vdecddutils_buf_vxd_adjust_size(&size_info->mbparams_bufsize);

		if (comseq_hdrinfo->separate_chroma_planes)
			size_info->mbparams_bufsize *= 3;
	}

	return IMG_SUCCESS;
}

/*
 * @Function core_pict_res_getinfo
 */
static int
core_pict_res_getinfo(struct core_stream_context *core_str_ctx,
		      const struct vdec_comsequ_hdrinfo *comseq_hdrinfo,
		      const struct vdec_str_opconfig *op_cfg,
		      const struct vdecdd_ddpict_buf *disp_pictbuf,
		      struct core_pict_resinfo *pict_resinfo,
		      struct core_seq_resinfo *seq_resinfo)
{
	struct vdec_pict_size coded_pict_size;
	struct dec_ctx *decctx;
	unsigned char res_needed = FALSE;
	int ret;

	/* Reset the picture resource info. */
	memset(pict_resinfo, 0, sizeof(*pict_resinfo));

	coded_pict_size = comseq_hdrinfo->max_frame_size;

	VDEC_ASSERT(core_str_ctx->std_spec_ops);
	if (core_str_ctx->std_spec_ops->bufs_get_size)
		core_str_ctx->std_spec_ops->bufs_get_size(core_str_ctx, comseq_hdrinfo,
			&coded_pict_size,
			&pict_resinfo->size_info, seq_resinfo, &res_needed);

	/* If any picture resources are needed... */
	if (res_needed) {
		/* Get the number of resources required. */
		ret = vdecddutils_get_minrequired_numpicts
							(&core_str_ctx->dd_str_ctx->str_config_data,
							  comseq_hdrinfo, op_cfg,
							  &pict_resinfo->pict_res_num);
		VDEC_ASSERT(ret == IMG_SUCCESS);
		if (ret != IMG_SUCCESS)
			return ret;

		decctx = (struct dec_ctx *)global_core_ctx->dev_ctx->dec_context;

		if (core_str_ctx->dd_str_ctx->str_config_data.vid_std == VDEC_STD_HEVC)
			pict_resinfo->pict_res_num += decctx->dev_cfg->num_slots_per_pipe - 1;
		else
			pict_resinfo->pict_res_num +=
				decctx->num_pipes * decctx->dev_cfg->num_slots_per_pipe - 1;
	}

	return IMG_SUCCESS;
}

static int core_alloc_resbuf(struct vdecdd_ddbuf_mapinfo **buf_handle,
			     unsigned int size, void *mmu_handle,
			     struct vxdio_mempool mem_pool)
{
	int ret;
	struct vdecdd_ddbuf_mapinfo *buf;

	*buf_handle = kzalloc(sizeof(**buf_handle), GFP_KERNEL);
	buf = *buf_handle;
	VDEC_ASSERT(buf);
	if (buf) {
		buf->mmuheap_id = MMU_HEAP_STREAM_BUFFERS;
#ifdef DEBUG_DECODER_DRIVER
		pr_info("%s:%d calling MMU_StreamMalloc", __func__, __LINE__);
#endif
		ret = mmu_stream_alloc(mmu_handle, buf->mmuheap_id,
				       mem_pool.mem_heap_id,
				       mem_pool.mem_attrib, size,
				       DEV_MMU_PAGE_SIZE,
				       &buf->ddbuf_info);

		VDEC_ASSERT(ret == IMG_SUCCESS);
		if (ret != IMG_SUCCESS)
			ret = IMG_ERROR_OUT_OF_MEMORY;
	} else {
		ret = IMG_ERROR_OUT_OF_MEMORY;
	}
	return ret;
}

#ifdef SEQ_RES_NEEDED
static int core_alloc_common_sequence_buffers(struct core_stream_context *core_str_ctx,
					      struct vdecdd_seq_resint *seqres_int,
					      struct vxdio_mempool mem_pool,
					      struct core_seq_resinfo *seqres_info,
					      struct core_pict_resinfo *pictres_info,
					      const struct vdec_str_opconfig *op_cfg,
					      const struct vdecdd_ddpict_buf *disp_pict_buf)
{
	int ret = IMG_SUCCESS;
#ifdef ERROR_CONCEALMENT
	enum vdec_vid_std vid_std = core_str_ctx->dd_str_ctx->str_config_data.vid_std;
	unsigned int std_idx = vid_std - 1;
	struct vidio_ddbufinfo *err_buf_info;

	/* Allocate error concealment pattern frame for current sequence */
	if (err_recovery_frame_info[std_idx].enabled) {
		struct vdec_pict_bufconfig buf_config;
		unsigned int size;

		buf_config = disp_pict_buf->buf_config;
		size = buf_config.coded_width * buf_config.coded_height;

		if (err_recovery_frame_info[std_idx].max_size > size) {
			seqres_int->err_pict_buf = kzalloc(sizeof(*seqres_int->err_pict_buf),
							   GFP_KERNEL);
			VDEC_ASSERT(seqres_int->err_pict_buf);
			if (!seqres_int->err_pict_buf)
				return IMG_ERROR_OUT_OF_MEMORY;

			seqres_int->err_pict_buf->mmuheap_id = MMU_HEAP_STREAM_BUFFERS;

#ifdef DEBUG_DECODER_DRIVER
			pr_info("===== %s:%d calling MMU_StreamMalloc", __func__, __LINE__);
#endif
			ret = mmu_stream_alloc(core_str_ctx->dd_str_ctx->mmu_str_handle,
					       seqres_int->err_pict_buf->mmuheap_id,
					       mem_pool.mem_heap_id,
					       (enum sys_emem_attrib)(mem_pool.mem_attrib |
						       SYS_MEMATTRIB_CPU_WRITE),
					       buf_config.buf_size,
					       DEV_MMU_PAGE_ALIGNMENT,
					       &seqres_int->err_pict_buf->ddbuf_info);
			if (ret != IMG_SUCCESS)
				return IMG_ERROR_OUT_OF_MEMORY;

			/* make grey pattern - luma & chroma at mid-rail */
			err_buf_info = &seqres_int->err_pict_buf->ddbuf_info;
			if (op_cfg->pixel_info.mem_pkg == PIXEL_BIT10_MP) {
				unsigned int *out = (unsigned int *)err_buf_info->cpu_virt;
				unsigned int i;

				for (i = 0; i < err_buf_info->buf_size / sizeof(unsigned int); i++)
					/* See PIXEL_BIT10_MP layout definition */
					out[i] = 0x20080200;
			} else {
				/* Note: Setting 0x80 also gives grey pattern
				 * for 10bit upacked MSB format.
				 */
				memset(err_buf_info->cpu_virt, 0x80, err_buf_info->buf_size);
			}
		}
	}
#endif
	return ret;
}
#endif

/*
 * @Function              core_do_resource_realloc
 */
static unsigned char core_do_resource_realloc(struct core_stream_context *core_str_ctx,
					      struct core_pict_resinfo *pictres_info,
					      struct core_seq_resinfo *seqres_info)
{
	VDEC_ASSERT(core_str_ctx->std_spec_ops);
	/* If buffer sizes are sufficient and only the greater number of resources is needed... */
	if (core_str_ctx->pict_resinfo.size_info.mbparams_bufsize >=
		pictres_info->size_info.mbparams_bufsize &&
		(core_str_ctx->std_spec_ops->is_stream_resource_suitable ?
		core_str_ctx->std_spec_ops->is_stream_resource_suitable(pictres_info,
		&core_str_ctx->pict_resinfo,
		seqres_info, &core_str_ctx->seq_resinfo) : TRUE) &&
		core_str_ctx->pict_resinfo.pict_res_num < pictres_info->pict_res_num)
		/* ...full internal resource reallocation is not required. */
		return FALSE;

	/* Otherwise request full internal resource reallocation. */
	return TRUE;
}

/*
 * @Function              core_is_stream_resource_suitable
 */
static unsigned char core_is_stream_resource_suitable
		(struct core_stream_context *core_str_ctx,
		 const struct vdec_comsequ_hdrinfo *comseq_hdrinfo,
		 const struct vdec_str_opconfig *op_cfg,
		 const struct vdecdd_ddpict_buf *disp_pict_buf,
		 struct core_pict_resinfo *pictres_info,
		 struct core_seq_resinfo *seqres_info_ptr)
{
	int ret;
	struct core_pict_resinfo aux_pictes_info;
	struct core_pict_resinfo    *aux_pictes_info_ptr;
	struct core_seq_resinfo seqres_info;

	/* If resource info is needed externally, just use it. Otherwise use internal structure. */
	if (pictres_info)
		aux_pictes_info_ptr = pictres_info;
	else
		aux_pictes_info_ptr = &aux_pictes_info;

	if (!seqres_info_ptr)
		seqres_info_ptr = &seqres_info;

	/* Get the resource info for current settings. */
	ret = core_pict_res_getinfo(core_str_ctx, comseq_hdrinfo, op_cfg, disp_pict_buf,
				    aux_pictes_info_ptr, seqres_info_ptr);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return FALSE;

	VDEC_ASSERT(core_str_ctx->std_spec_ops);
	if (core_str_ctx->std_spec_ops->is_stream_resource_suitable) {
		if (!core_str_ctx->std_spec_ops->is_stream_resource_suitable
					(aux_pictes_info_ptr,
					 &core_str_ctx->pict_resinfo,
					 seqres_info_ptr, &core_str_ctx->seq_resinfo))
			return FALSE;
	}

	/* Check the number of picture resources required against the current number. */
	if (aux_pictes_info_ptr->pict_res_num > core_str_ctx->pict_resinfo.pict_res_num)
		return FALSE;

	return TRUE;
}

static int core_alloc_common_pict_buffers(struct core_stream_context *core_str_ctx,
					  struct vdecdd_pict_resint *pictres_int,
					  struct vxdio_mempool mem_pool,
					  struct core_pict_resinfo *pictres_info)
{
	int ret = IMG_SUCCESS;

	/* If MB params buffers are needed... */
	if (pictres_info->size_info.mbparams_bufsize > 0)
		/* Allocate the MB parameters buffer info structure. */
		ret = core_alloc_resbuf(&pictres_int->mb_param_buf,
					pictres_info->size_info.mbparams_bufsize,
					core_str_ctx->dd_str_ctx->mmu_str_handle,
					mem_pool);

	return ret;
}

/*
 * @Function              core_stream_resource_create
 */
static int core_stream_resource_create(struct core_stream_context *core_str_ctx,
				       unsigned char closed_gop, unsigned int mem_heap_id,
				       const struct vdec_comsequ_hdrinfo *comseq_hdrinfo,
				       const struct vdec_str_opconfig *op_cfg,
				       const struct vdecdd_ddpict_buf *disp_pict_buf)
{
	struct vdecdd_pict_resint *pictres_int = NULL;
	int ret = IMG_SUCCESS;
	unsigned int i, start_cnt = 0;
	struct core_pict_resinfo pictres_info;
	struct vdecdd_seq_resint *seqres_int = NULL;
	struct core_seq_resinfo seqres_info;
	struct vxdio_mempool mem_pool;

	mem_pool.mem_heap_id = mem_heap_id;
	mem_pool.mem_attrib = (enum sys_emem_attrib)(SYS_MEMATTRIB_UNCACHED
				| SYS_MEMATTRIB_WRITECOMBINE | SYS_MEMATTRIB_INTERNAL);

#ifdef SEQ_RES_NEEDED
	seqres_int = lst_first(&core_str_ctx->seq_res_list);
#endif
	/*
	 * Clear the reconstructed picture buffer layout if the previous
	 * references are no longer used. Only under these circumstances
	 * should the bitstream resolution change.
	 */
	if (closed_gop) {
		memset(&core_str_ctx->recon_pictbuf.rend_info, 0,
		       sizeof(core_str_ctx->recon_pictbuf.rend_info));
		memset(&core_str_ctx->coded_pict_size, 0, sizeof(core_str_ctx->coded_pict_size));
	} else {
		if (vdec_size_ne(core_str_ctx->coded_pict_size, comseq_hdrinfo->max_frame_size)) {
			VDEC_ASSERT(FALSE);
			pr_err("Coded picture size changed within the closed GOP (i.e. mismatched references)");
		}
	}

	/* If current buffers are not suitable for specified VSH/Output config... */
	if (!core_is_stream_resource_suitable(core_str_ctx, comseq_hdrinfo,
					      op_cfg, disp_pict_buf, &pictres_info,
					      &seqres_info)) {
		/* If full internal resource reallocation is needed... */
		if (core_do_resource_realloc(core_str_ctx, &pictres_info, &seqres_info)) {
			/*
			 * Mark all the active resources as deprecated and
			 * free-up where no longer used.
			 */
			core_stream_resource_deprecate(core_str_ctx);
		} else {
			/* Use current buffer size settings. */
			pictres_info.size_info = core_str_ctx->pict_resinfo.size_info;
			seqres_info = core_str_ctx->seq_resinfo;

			/* Set start counter to only allocate the number of
			 * resources that are missing.
			 */
			start_cnt = core_str_ctx->pict_resinfo.pict_res_num;
		}

#ifdef SEQ_RES_NEEDED
		/* allocate sequence resources */
		{
			seqres_int = kzalloc(sizeof(*seqres_int), GFP_KERNEL);
			VDEC_ASSERT(seqres_int);
			if (!seqres_int)
				goto err_out_of_memory;

			lst_add(&core_str_ctx->seq_res_list, seqres_int);
			/* Allocate sequence buffers common for all standards. */
			ret = core_alloc_common_sequence_buffers
							(core_str_ctx, seqres_int, mem_pool,
							 &seqres_info,
							 &pictres_info, op_cfg, disp_pict_buf);
			if (ret != IMG_SUCCESS)
				goto err_out_of_memory;

			VDEC_ASSERT(core_str_ctx->std_spec_ops);
			if (core_str_ctx->std_spec_ops->alloc_sequence_buffers) {
				ret = core_str_ctx->std_spec_ops->alloc_sequence_buffers
							(core_str_ctx, seqres_int,
							 mem_pool, &seqres_info);
				if (ret != IMG_SUCCESS)
					goto err_out_of_memory;
			}
		}
#endif
		/* Allocate resources for current settings. */
		for (i = start_cnt; i < pictres_info.pict_res_num; i++) {
			/* Allocate the picture resources structure. */
			pictres_int = kzalloc(sizeof(*pictres_int), GFP_KERNEL);
			VDEC_ASSERT(pictres_int);
			if (!pictres_int)
				goto err_out_of_memory;

			/* Allocate picture buffers common for all standards. */
			ret = core_alloc_common_pict_buffers(core_str_ctx, pictres_int,
							     mem_pool, &pictres_info);
			if (ret != IMG_SUCCESS)
				goto err_out_of_memory;

			/* Allocate standard specific picture buffers. */
			VDEC_ASSERT(core_str_ctx->std_spec_ops);
			if (core_str_ctx->std_spec_ops->alloc_picture_buffers) {
				ret = core_str_ctx->std_spec_ops->alloc_picture_buffers
							(core_str_ctx, pictres_int,
							 mem_pool, &pictres_info);
				if (ret != IMG_SUCCESS)
					goto err_out_of_memory;
			}

			/* attach sequence resources */
#ifdef SEQ_RES_NEEDED
			resource_item_use(&seqres_int->ref_count);
			pictres_int->seq_resint = seqres_int;
#endif
			lst_add(&core_str_ctx->pict_res_list, pictres_int);
			core_str_ctx->pict_resinfo.pict_res_num++;
		}
	}

	/*
	 * When demand for picture resources reduces (in quantity) the extra buffers
	 * are still retained. Preserve the existing count in case the demand increases
	 * again, at which time these residual buffers won't need to be reallocated.
	 */
	pictres_info.pict_res_num = core_str_ctx->pict_resinfo.pict_res_num;

	/* Store the current resource config. */
	core_str_ctx->pict_resinfo = pictres_info;
	core_str_ctx->seq_resinfo = seqres_info;

	pictres_int = lst_first(&core_str_ctx->pict_res_list);
	while (pictres_int) {
		/*
		 * Increment the reference count to indicate that this resource is also
		 * held by plant until it is added to the Scheduler list. If the resource has
		 * not just been created it might already be in circulation.
		 */
		resource_item_use(&pictres_int->ref_cnt);
#ifdef SEQ_RES_NEEDED
		/* attach sequence resources */
		resource_item_use(&seqres_int->ref_count);
		pictres_int->seq_resint = seqres_int;
#endif
		/* Add the internal picture resources to the list. */
		ret = resource_list_add_img(&core_str_ctx->aux_pict_res_list,
					    pictres_int, 0, &pictres_int->ref_cnt);

		pictres_int = lst_next(pictres_int);
	}

	/*
	 * Set the reconstructed buffer properties if they
	 * may have been changed.
	 */
	if (core_str_ctx->recon_pictbuf.rend_info.rendered_size == 0) {
		core_str_ctx->recon_pictbuf.rend_info =
			disp_pict_buf->rend_info;
		core_str_ctx->recon_pictbuf.buf_config =
			disp_pict_buf->buf_config;
		core_str_ctx->coded_pict_size = comseq_hdrinfo->max_frame_size;
	} else {
		if (memcmp(&disp_pict_buf->rend_info,
			   &core_str_ctx->recon_pictbuf.rend_info,
			   sizeof(core_str_ctx->recon_pictbuf.rend_info))) {
			/*
			 * Reconstructed picture buffer information has changed
			 * during a closed GOP.
			 */
			VDEC_ASSERT
			("Reconstructed picture buffer information cannot change within a GOP"
				 == NULL);
			pr_err("Reconstructed picture buffer information cannot change within a GOP.");
			return IMG_ERROR_GENERIC_FAILURE;
		}
	}

	/*
	 * When demand for picture resources reduces (in quantity) the extra buffers
	 * are still retained. Preserve the existing count in case the demand increases
	 * again, at which time these residual buffers won't need to be reallocated.
	 */
	pictres_info.pict_res_num = core_str_ctx->pict_resinfo.pict_res_num;

	/* Store the current resource config. */
	core_str_ctx->pict_resinfo = pictres_info;
	core_str_ctx->seq_resinfo = seqres_info;

	return IMG_SUCCESS;

	/* Handle out of memory errors. */
err_out_of_memory:
	/* Free resources being currently allocated. */
	if (pictres_int) {
		core_free_common_picture_resource(core_str_ctx, pictres_int);
		if (core_str_ctx->std_spec_ops->free_picture_resource)
			core_str_ctx->std_spec_ops->free_picture_resource(core_str_ctx,
				pictres_int);

		kfree(pictres_int);
	}

#ifdef SEQ_RES_NEEDED
	if (seqres_int) {
		core_free_common_sequence_resource(core_str_ctx, seqres_int);

		if (core_str_ctx->std_spec_ops->free_sequence_resource)
			core_str_ctx->std_spec_ops->free_sequence_resource(core_str_ctx,
				seqres_int);

		VDEC_ASSERT(lst_last(&core_str_ctx->seq_res_list) == seqres_int);
		lst_remove(&core_str_ctx->seq_res_list, seqres_int);
		kfree(seqres_int);
	}
#endif

	/* Free all the other resources. */
	core_stream_resource_destroy(core_str_ctx);

	pr_err("[USERSID=0x%08X] Core not able to allocate stream resources due to lack of memory",
	       core_str_ctx->dd_str_ctx->str_config_data.user_str_id);

	return IMG_ERROR_OUT_OF_MEMORY;
}

static int
core_reconfigure_recon_pictbufs(struct core_stream_context *core_str_ctx,
				unsigned char no_references)
{
	struct vdecdd_ddstr_ctx *dd_str_ctx;
	int ret;

	dd_str_ctx = core_str_ctx->dd_str_ctx;
	VDEC_ASSERT(dd_str_ctx->str_op_configured);

	/* Re-configure the internal picture buffers now that none are held. */
	ret = core_stream_resource_create(core_str_ctx, no_references,
					  dd_str_ctx->dd_dev_context->internal_heap_id,
					  &dd_str_ctx->comseq_hdr_info,
					  &dd_str_ctx->opconfig,
					  &dd_str_ctx->disp_pict_buf);
	return ret;
}

/*
 * @Function              core_picture_prepare
 */
static int core_picture_prepare(struct core_stream_context *core_str_ctx,
				struct vdecdd_str_unit *str_unit)
{
	int ret = IMG_SUCCESS;
	struct vdecdd_picture *pict_local = NULL;
	unsigned int avail = 0;
	unsigned char need_pict_res;

	/*
	 * For normal decode, setup picture data.
	 * Preallocate the picture structure.
	 */
	pict_local = kzalloc(sizeof(*pict_local), GFP_KERNEL);
	if (!pict_local)
		return IMG_ERROR_OUT_OF_MEMORY;

	/* Determine whether the picture can be decoded. */
	ret = decoder_get_load(core_str_ctx->dd_str_ctx->dec_ctx, &global_avail_slots);
	if (ret != IMG_SUCCESS) {
		pr_err("No resources avaialable to decode this picture");
		ret = IMG_ERROR_COULD_NOT_OBTAIN_RESOURCE;
		goto unwind;
	}

	/*
	 * Load and availability is cached in stream context simply
	 * for status reporting.
	 */
	avail = core_get_resource_availability(core_str_ctx);

	if ((avail & CORE_AVAIL_CORE) == 0) {
		/* Return straight away if the core is not available */
		ret = IMG_ERROR_COULD_NOT_OBTAIN_RESOURCE;
		goto unwind;
	}

	if (core_str_ctx->new_op_cfg || core_str_ctx->new_seq) {
		/*
		 * Reconstructed buffers should be checked for reconfiguration
		 * under these conditions:
		 *     1. New output configuration,
		 *     2. New sequence.
		 * Core can decide to reset the reconstructed buffer properties
		 * if there are no previous reference pictures used
		 * (i.e. at a closed GOP). This code must go here because we
		 * may not stop when new sequence is found or references become
		 * unused.
		 */
		ret = core_reconfigure_recon_pictbufs(core_str_ctx,
						      core_str_ctx->no_prev_refs_used);
		VDEC_ASSERT(ret == IMG_SUCCESS);
		if (ret != IMG_SUCCESS)
			goto unwind;
	}

	/* Update the display information for this picture. */
	ret = vdecddutils_get_display_region(&str_unit->pict_hdr_info->coded_frame_size,
					     &str_unit->pict_hdr_info->disp_info.enc_disp_region,
					     &str_unit->pict_hdr_info->disp_info.disp_region);

	if (ret != IMG_SUCCESS)
		goto unwind;

	/* Clear internal state */
	core_str_ctx->new_seq = FALSE;
	core_str_ctx->new_op_cfg = FALSE;
	core_str_ctx->no_prev_refs_used = FALSE;

	/*
	 * Recalculate this since we might have just created
	 * internal resources.
	 */
	core_str_ctx->res_avail = core_get_resource_availability(core_str_ctx);

	/*
	 * If picture resources were needed for this stream, picture resources
	 * list wouldn't be empty
	 */
	need_pict_res = !lst_empty(&core_str_ctx->aux_pict_res_list);
	/* If there are resources available */
	if ((core_str_ctx->res_avail & CORE_AVAIL_PICTBUF) &&
	    (!need_pict_res || (core_str_ctx->res_avail & CORE_AVAIL_PICTRES))) {
		/* Pick internal picture resources. */
		if (need_pict_res) {
			pict_local->pict_res_int =
				resource_list_get_avail(&core_str_ctx->aux_pict_res_list);

			VDEC_ASSERT(pict_local->pict_res_int);
			if (!pict_local->pict_res_int) {
				ret = IMG_ERROR_COULD_NOT_OBTAIN_RESOURCE;
				goto unwind;
			}
		}

		/* Pick the client image buffer. */
		pict_local->disp_pict_buf.pict_buf =
			resource_list_get_avail(&core_str_ctx->pict_buf_list);
		VDEC_ASSERT(pict_local->disp_pict_buf.pict_buf);
		if (!pict_local->disp_pict_buf.pict_buf) {
			ret = IMG_ERROR_COULD_NOT_OBTAIN_RESOURCE;
			goto unwind;
		}
	} else {
		/* Need resources to process picture start. */
		ret = IMG_ERROR_COULD_NOT_OBTAIN_RESOURCE;
		goto unwind;
	}

	/* Ensure that the buffer contains layout information. */
	pict_local->disp_pict_buf.rend_info = core_str_ctx->disp_pict_buf.rend_info;
	pict_local->disp_pict_buf.buf_config = core_str_ctx->disp_pict_buf.buf_config;
	pict_local->op_config = core_str_ctx->op_cfg;
	pict_local->last_pict_in_seq = str_unit->last_pict_in_seq;

	str_unit->dd_pict_data = pict_local;

	/* Indicate that all necessary resources are now available. */
	if (core_str_ctx->res_avail != ~0) {
#ifdef DEBUG_DECODER_DRIVER
		pr_info("LAST AVAIL: 0x%08X\n", core_str_ctx->res_avail);
#endif
		core_str_ctx->res_avail = ~0;
	}

#ifdef DEBUG_DECODER_DRIVER
	/* dump decoder internal resource addresses */
	if (pict_local->pict_res_int) {
		if (pict_local->pict_res_int->mb_param_buf) {
			pr_info("[USERSID=0x%08X] MB parameter buffer device virtual address: 0x%08X",
				core_str_ctx->dd_str_ctx->str_config_data.user_str_id,
				pict_local->pict_res_int->mb_param_buf->ddbuf_info.dev_virt);
		}

		if (core_str_ctx->comseq_hdr_info.separate_chroma_planes) {
			pr_info("[USERSID=0x%08X] Display picture virtual address: LUMA 0x%08X, CHROMA 0x%08X, CHROMA2 0x%08X",
				core_str_ctx->dd_str_ctx->str_config_data.user_str_id,
				pict_local->disp_pict_buf.pict_buf->ddbuf_info.dev_virt,
				pict_local->disp_pict_buf.pict_buf->ddbuf_info.dev_virt +
				pict_local->disp_pict_buf.rend_info.plane_info
						[VDEC_PLANE_VIDEO_U].offset,
				pict_local->disp_pict_buf.pict_buf->ddbuf_info.dev_virt +
				pict_local->disp_pict_buf.rend_info.plane_info
						[VDEC_PLANE_VIDEO_V].offset);
		} else {
			pr_info("[USERSID=0x%08X] Display picture virtual address: LUMA 0x%08X, CHROMA 0x%08X",
				core_str_ctx->dd_str_ctx->str_config_data.user_str_id,
				pict_local->disp_pict_buf.pict_buf->ddbuf_info.dev_virt,
				pict_local->disp_pict_buf.pict_buf->ddbuf_info.dev_virt +
				pict_local->disp_pict_buf.rend_info.plane_info
						[VDEC_PLANE_VIDEO_UV].offset);
		}
	}
#endif

	ret = core_picture_attach_resources(core_str_ctx, str_unit, TRUE);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		goto unwind;

	return IMG_SUCCESS;

unwind:
	if (pict_local->pict_res_int) {
		resource_item_return(&pict_local->pict_res_int->ref_cnt);
		pict_local->pict_res_int = NULL;
	}
	if (pict_local->disp_pict_buf.pict_buf) {
		resource_item_return(&pict_local->disp_pict_buf.pict_buf->ddbuf_info.ref_count);
		pict_local->disp_pict_buf.pict_buf = NULL;
	}
	kfree(pict_local);
	return ret;
}

/*
 * @Function              core_validate_new_sequence
 */
static int core_validate_new_sequence(struct core_stream_context *core_str_ctx,
				      const struct vdec_comsequ_hdrinfo *comseq_hdrinfo)
{
	int ret;
	struct vdecdd_supp_check supp_check;
	struct vdecdd_ddstr_ctx *dd_str_ctx;
	unsigned int num_req_bufs_prev, num_req_bufs_cur;
	struct vdecdd_mapbuf_info mapbuf_info;

	memset(&supp_check, 0, sizeof(supp_check));

	/*
	 * Omit picture header from this setup since we can'supp_check
	 * validate this here.
	 */
	supp_check.comseq_hdrinfo = comseq_hdrinfo;

	if (core_str_ctx->opcfg_set) {
		supp_check.op_cfg = &core_str_ctx->op_cfg;
		supp_check.disp_pictbuf = &core_str_ctx->disp_pict_buf;

		ret = vdecddutils_get_minrequired_numpicts
			(&core_str_ctx->dd_str_ctx->str_config_data,
			 &core_str_ctx->comseq_hdr_info,
			 &core_str_ctx->op_cfg,
			 &num_req_bufs_prev);
		VDEC_ASSERT(ret == IMG_SUCCESS);
		if (ret != IMG_SUCCESS)
			return ret;

		ret = vdecddutils_get_minrequired_numpicts
			(&core_str_ctx->dd_str_ctx->str_config_data,
			 comseq_hdrinfo,
			 &core_str_ctx->op_cfg,
			 &num_req_bufs_cur);
		VDEC_ASSERT(ret == IMG_SUCCESS);
		if (ret != IMG_SUCCESS)
			return ret;
	}

	/* Check if the output configuration is compatible with new VSH. */
	dd_str_ctx = core_str_ctx->dd_str_ctx;
	mapbuf_info = dd_str_ctx->map_buf_info;

	/* Check the compatibility of the bitstream data and configuration */
	supp_check.non_cfg_req = TRUE;
	ret = core_check_decoder_support(dd_str_ctx->dd_dev_context,
					 &dd_str_ctx->str_config_data,
					 &dd_str_ctx->prev_comseq_hdr_info,
					 &dd_str_ctx->prev_pict_hdr_info,
					 &mapbuf_info, &supp_check);
	if (ret != IMG_SUCCESS)
		return ret;

	core_str_ctx->new_seq = TRUE;

	return IMG_SUCCESS;
}

static int
core_validate_new_picture(struct core_stream_context *core_str_ctx,
			  const struct bspp_pict_hdr_info *pict_hdrinfo,
			  unsigned int *features)
{
	int ret;
	struct vdecdd_supp_check supp_check;
	struct vdecdd_ddstr_ctx *dd_str_ctx;
	struct vdecdd_mapbuf_info mapbuf_info;

	memset(&supp_check, 0, sizeof(supp_check));
	supp_check.comseq_hdrinfo = &core_str_ctx->comseq_hdr_info;
	supp_check.pict_hdrinfo = pict_hdrinfo;

	/*
	 * They cannot become invalid during a sequence.
	 * However, output configuration may signal something that
	 * changes compatibility on a closed GOP within a sequence
	 * (e.g. resolution may significantly decrease
	 * in a GOP and scaling wouldn't be supported). This resolution shift
	 * would not be signalled in the sequence header
	 * (since that is the maximum) but only
	 * found now when validating the first picture in the GOP.
	 */
	if (core_str_ctx->opcfg_set)
		supp_check.op_cfg = &core_str_ctx->op_cfg;

	/*
	 * Check if the new picture is compatible with the
	 * current driver state.
	 */
	dd_str_ctx = core_str_ctx->dd_str_ctx;
	mapbuf_info = dd_str_ctx->map_buf_info;

	/* Check the compatibility of the bitstream data and configuration */
	supp_check.non_cfg_req = TRUE;
	ret = core_check_decoder_support(dd_str_ctx->dd_dev_context,
					 &dd_str_ctx->str_config_data,
					 &dd_str_ctx->prev_comseq_hdr_info,
					 &dd_str_ctx->prev_pict_hdr_info,
					 &mapbuf_info, &supp_check);

	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	if (supp_check.unsupp_flags.str_opcfg || supp_check.unsupp_flags.pict_hdr)
		return IMG_ERROR_NOT_SUPPORTED;

	/*
	 * Clear the reconfiguration flags unless triggered by
	 * unsupported output config.
	 */
	*features = supp_check.features;

	return IMG_SUCCESS;
}

/*
 * @Function core_stream_submit_unit
 */
int core_stream_submit_unit(unsigned int res_str_id, struct vdecdd_str_unit *str_unit)
{
	int ret;
	unsigned char process_str_unit = TRUE;

	struct vdecdd_ddstr_ctx *dd_str_context;
	struct core_stream_context *core_str_ctx;

	/*
	 * Stream based messages without a device context
	 * must have a stream ID.
	 */
	VDEC_ASSERT(res_str_id);
	VDEC_ASSERT(str_unit);

	if (res_str_id == 0 || !str_unit) {
		pr_err("Invalid params passed to %s\n", __func__);
		return IMG_ERROR_INVALID_PARAMETERS;
	}

	/* Get access to stream context.. */
	ret = rman_get_resource(res_str_id, VDECDD_STREAM_TYPE_ID, (void **)&core_str_ctx, NULL);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	VDEC_ASSERT(core_str_ctx);
	dd_str_context = core_str_ctx->dd_str_ctx;
	VDEC_ASSERT(dd_str_context);

	ret = resource_list_add_img(&core_str_ctx->str_unit_list, str_unit, 0, NULL);
	VDEC_ASSERT(ret == IMG_SUCCESS);

	pr_debug("%s stream unit type = %d\n", __func__, str_unit->str_unit_type);
	switch (str_unit->str_unit_type) {
	case VDECDD_STRUNIT_SEQUENCE_START:
		if (str_unit->seq_hdr_info) {
			/* Add sequence header to cache. */
			ret =
				resource_list_replace(&core_str_ctx->seq_hdr_list,
						      str_unit->seq_hdr_info,
						      str_unit->seq_hdr_info->sequ_hdr_id,
						      &str_unit->seq_hdr_info->ref_count,
						      NULL, NULL);

			if (ret != IMG_SUCCESS)
				pr_err("[USERSID=0x%08X] Failed to replace resource",
				       res_str_id);
		} else {
			/* ...or take from cache. */
			str_unit->seq_hdr_info =
				resource_list_getbyid(&core_str_ctx->seq_hdr_list,
						      str_unit->seq_hdr_id);
		}

		VDEC_ASSERT(str_unit->seq_hdr_info);
		if (!str_unit->seq_hdr_info) {
			pr_err("Sequence header information not available for current picture");
			break;
		}
		/*
		 * Check that this latest sequence header information is
		 * compatible with current state and then if no errors store
		 * as current.
		 */
		core_str_ctx->comseq_hdr_info = str_unit->seq_hdr_info->com_sequ_hdr_info;

		ret = core_validate_new_sequence(core_str_ctx,
						 &str_unit->seq_hdr_info->com_sequ_hdr_info);
		if (ret != IMG_SUCCESS)
			return ret;

		dd_str_context->prev_comseq_hdr_info =
			dd_str_context->comseq_hdr_info;
		dd_str_context->comseq_hdr_info =
			str_unit->seq_hdr_info->com_sequ_hdr_info;

#ifdef DEBUG_DECODER_DRIVER
		pr_info("[SID=0x%08X] VSH: Maximum Frame Resolution [%dx%d]",
			dd_str_context->res_str_id,
			dd_str_context->comseq_hdr_info.max_frame_size.width,
			dd_str_context->comseq_hdr_info.max_frame_size.height);
#endif

		break;

	case VDECDD_STRUNIT_PICTURE_START:
		/*
		 * Check that the picture configuration is compatible
		 * with the current state.
		 */
		ret = core_validate_new_picture(core_str_ctx,
						str_unit->pict_hdr_info,
						&str_unit->features);
		if (ret != IMG_SUCCESS) {
			if (ret == IMG_ERROR_NOT_SUPPORTED) {
				/*
				 * Do not process stream unit since there is
				 * something unsupported.
				 */
				process_str_unit = FALSE;
				break;
			}
		}

		/* Prepare picture for decoding. */
		ret = core_picture_prepare(core_str_ctx, str_unit);
		if (ret != IMG_SUCCESS)
			if (ret == IMG_ERROR_COULD_NOT_OBTAIN_RESOURCE ||
			    ret == IMG_ERROR_NOT_SUPPORTED)
				/*
				 * Do not process stream unit since there is
				 * something unsupported or resources are not
				 * available.
				 */
				process_str_unit = FALSE;
		break;

	default:
		/*
		 * Sequence/picture headers should only be attached to
		 * corresponding units.
		 */
		VDEC_ASSERT(!str_unit->seq_hdr_info);
		VDEC_ASSERT(!str_unit->pict_hdr_info);
		break;
	}

	if (process_str_unit) {
		/* Submit stream unit to the decoder for processing. */
		str_unit->decode = TRUE;
		ret = decoder_stream_process_unit(dd_str_context->dec_ctx,
						  str_unit);
	} else {
		ret = IMG_ERROR_GENERIC_FAILURE;
	}

	return ret;
}

/*
 * @Function  core_stream_fill_pictbuf
 */
int core_stream_fill_pictbuf(unsigned int buf_map_id)
{
	int ret;
	struct vdecdd_ddbuf_mapinfo *ddbuf_map_info;
	struct vdecdd_ddstr_ctx *dd_str_ctx;
	struct core_stream_context *core_str_ctx;

	/* Get access to map info context.. */
	ret = rman_get_resource(buf_map_id, VDECDD_BUFMAP_TYPE_ID,
				(void **)&ddbuf_map_info, NULL);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	dd_str_ctx = ddbuf_map_info->ddstr_context;

	/* Get access to stream context.. */
	ret = rman_get_resource(dd_str_ctx->res_str_id, VDECDD_STREAM_TYPE_ID,
				(void **)&core_str_ctx, NULL);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	/* Check buffer type. */
	VDEC_ASSERT(ddbuf_map_info->buf_type == VDEC_BUFTYPE_PICTURE);

	/* Add the image buffer to the list */
	ret = resource_list_add_img(&core_str_ctx->pict_buf_list, ddbuf_map_info,
				    0, &ddbuf_map_info->ddbuf_info.ref_count);

	return ret;
}

/*
 * @Function              core_fn_free_mapped
 */
static void core_fn_free_mapped(void *param)
{
	struct vdecdd_ddbuf_mapinfo *ddbuf_map_info =
		(struct vdecdd_ddbuf_mapinfo *)param;

	/* Validate input arguments */
	VDEC_ASSERT(param);

	/* Do not free the MMU mapping. It is handled by talmmu code. */
	kfree(ddbuf_map_info);
}

/*
 * @Function              core_stream_map_buf
 */
int core_stream_map_buf(unsigned int res_str_id, enum vdec_buf_type buf_type,
			struct vdec_buf_info *buf_info, unsigned int *buf_map_id)
{
	int ret;
	struct vdecdd_ddstr_ctx *dd_str_ctx;
	struct core_stream_context *core_str_ctx;
	struct vdecdd_ddbuf_mapinfo *ddbuf_map_info;

	/*
	 * Stream based messages without a device context
	 * must have a stream ID.
	 */
	VDEC_ASSERT(res_str_id);
	VDEC_ASSERT(buf_type < VDEC_BUFTYPE_MAX);
	VDEC_ASSERT(buf_info);
	VDEC_ASSERT(buf_map_id);

	/* Get access to stream context.. */
	ret = rman_get_resource(res_str_id, VDECDD_STREAM_TYPE_ID,
				(void **)&core_str_ctx, NULL);

	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	dd_str_ctx = core_str_ctx->dd_str_ctx;

	VDEC_ASSERT(dd_str_ctx);

	/* Allocate an active stream unit.. */
	ddbuf_map_info = kzalloc(sizeof(*ddbuf_map_info), GFP_KERNEL);
	VDEC_ASSERT(ddbuf_map_info);

	if (!ddbuf_map_info) {
		pr_err("[SID=0x%08X] Failed to allocate memory for DD buffer map information",
		       dd_str_ctx->res_str_id);

		return IMG_ERROR_OUT_OF_MEMORY;
	}
	memset(ddbuf_map_info, 0, sizeof(*ddbuf_map_info));

	/* Save the stream context etc. */
	ddbuf_map_info->ddstr_context   = dd_str_ctx;
	ddbuf_map_info->buf_type        = buf_type;

#ifdef DEBUG_DECODER_DRIVER
	pr_info("%s:%d vdec2plus: vxd map buff id %d", __func__, __LINE__,
		buf_info->buf_id);
#endif
	ddbuf_map_info->buf_id = buf_info->buf_id;

	/* Register the allocation as a stream resource.. */
	ret = rman_register_resource(dd_str_ctx->res_buck_handle,
				     VDECDD_BUFMAP_TYPE_ID,
				     core_fn_free_mapped,
				     ddbuf_map_info,
				     &ddbuf_map_info->res_handle,
				     buf_map_id);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		goto error;

	ddbuf_map_info->buf_map_id = *buf_map_id;

	if (buf_type == VDEC_BUFTYPE_PICTURE) {
		if (dd_str_ctx->map_buf_info.num_buf == 0) {
			dd_str_ctx->map_buf_info.buf_size = buf_info->buf_size;
			dd_str_ctx->map_buf_info.byte_interleave =
				buf_info->pictbuf_cfg.byte_interleave;
#ifdef DEBUG_DECODER_DRIVER
			pr_info("[SID=0x%08X] Mapped Buffer size: %d (bytes)",
				dd_str_ctx->res_str_id, buf_info->buf_size);
#endif
		} else {
			/*
			 * Same byte interleaved setting should be used.
			 * Convert to actual bools by comparing with zero.
			 */
			if (buf_info->pictbuf_cfg.byte_interleave !=
				dd_str_ctx->map_buf_info.byte_interleave) {
				pr_err("[SID=0x%08X] Buffer cannot be mapped since its byte interleave value (%s) is not the same as buffers already mapped (%s)",
				       dd_str_ctx->res_str_id,
				       buf_info->pictbuf_cfg.byte_interleave ?
				       "ON" : "OFF",
				       dd_str_ctx->map_buf_info.byte_interleave ?
				       "ON" : "OFF");
				ret = IMG_ERROR_INVALID_PARAMETERS;
				goto error;
			}
		}

		/* Configure the buffer.. */
		ret = core_stream_set_pictbuf_config(dd_str_ctx, &buf_info->pictbuf_cfg);
		VDEC_ASSERT(ret == IMG_SUCCESS);
		if (ret != IMG_SUCCESS)
			goto error;
	}

	/* Map heap from VDEC to MMU. */
	switch (buf_type) {
	case VDEC_BUFTYPE_BITSTREAM:
		ddbuf_map_info->mmuheap_id = MMU_HEAP_BITSTREAM_BUFFERS;
		break;

	case VDEC_BUFTYPE_PICTURE:
		mmu_get_heap(buf_info->pictbuf_cfg.stride[VDEC_PLANE_VIDEO_Y],
			     &ddbuf_map_info->mmuheap_id);
		break;

	default:
		VDEC_ASSERT(FALSE);
	}

	/* Map this buffer into the MMU. */
#ifdef DEBUG_DECODER_DRIVER
	pr_info("----- %s:%d calling MMU_StreamMapExt", __func__, __LINE__);
#endif
	ret = mmu_stream_map_ext(dd_str_ctx->mmu_str_handle,
				 (enum mmu_eheap_id)ddbuf_map_info->mmuheap_id,
				 ddbuf_map_info->buf_id,
				 buf_info->buf_size, DEV_MMU_PAGE_SIZE,
				 buf_info->mem_attrib,
				 buf_info->cpu_linear_addr,
				 &ddbuf_map_info->ddbuf_info);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		goto error;

	if (buf_type == VDEC_BUFTYPE_PICTURE)
		dd_str_ctx->map_buf_info.num_buf++;

	/*
	 * Initialise the reference count to indicate that the client
	 * still holds the buffer.
	 */
	ddbuf_map_info->ddbuf_info.ref_count = 1;

	/* Return success.. */
	return IMG_SUCCESS;

error:
	if (ddbuf_map_info) {
		if (ddbuf_map_info->res_handle)
			rman_free_resource(ddbuf_map_info->res_handle);
		else
			kfree(ddbuf_map_info);
	}

	return ret;
}

/*
 * @Function              core_stream_map_buf_sg
 */
int core_stream_map_buf_sg(unsigned int res_str_id, enum vdec_buf_type buf_type,
			   struct vdec_buf_info *buf_info,
			   void *sgt, unsigned int *buf_map_id)
{
	int ret;
	struct vdecdd_ddstr_ctx *dd_str_ctx;
	struct core_stream_context *core_str_ctx;
	struct vdecdd_ddbuf_mapinfo *ddbuf_map_info;

	/*
	 * Resource stream ID cannot be zero. If zero just warning and
	 * proceeding further will break the code. Return IMG_ERROR_INVALID_ID.
	 */
	if (res_str_id <= 0)
		return IMG_ERROR_INVALID_ID;

	VDEC_ASSERT(buf_type < VDEC_BUFTYPE_MAX);
	VDEC_ASSERT(buf_info);
	VDEC_ASSERT(buf_map_id);

	/* Get access to stream context.. */
	ret = rman_get_resource(res_str_id, VDECDD_STREAM_TYPE_ID, (void **)&core_str_ctx, NULL);

	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	dd_str_ctx = core_str_ctx->dd_str_ctx;

	VDEC_ASSERT(dd_str_ctx);

	/* Allocate an active stream unit.. */
	ddbuf_map_info = kzalloc(sizeof(*ddbuf_map_info), GFP_KERNEL);
	VDEC_ASSERT(ddbuf_map_info);

	if (!ddbuf_map_info) {
		pr_err("[SID=0x%08X] Failed to allocate memory for DD buffer map information",
		       dd_str_ctx->res_str_id);

		return IMG_ERROR_OUT_OF_MEMORY;
	}

	/* Save the stream context etc. */
	ddbuf_map_info->ddstr_context = dd_str_ctx;
	ddbuf_map_info->buf_type = buf_type;

#ifdef DEBUG_DECODER_DRIVER
	pr_info("%s:%d vdec2plus: vxd map buff id %d", __func__, __LINE__,
		buf_info->buf_id);
#endif
	ddbuf_map_info->buf_id = buf_info->buf_id;

	/* Register the allocation as a stream resource.. */
	ret = rman_register_resource(dd_str_ctx->res_buck_handle,
				     VDECDD_BUFMAP_TYPE_ID,
				     core_fn_free_mapped, ddbuf_map_info,
				     &ddbuf_map_info->res_handle, buf_map_id);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		goto error;

	ddbuf_map_info->buf_map_id = *buf_map_id;

	if (buf_type == VDEC_BUFTYPE_PICTURE) {
		if (dd_str_ctx->map_buf_info.num_buf == 0) {
			dd_str_ctx->map_buf_info.buf_size = buf_info->buf_size;

			dd_str_ctx->map_buf_info.byte_interleave =
				buf_info->pictbuf_cfg.byte_interleave;

#ifdef DEBUG_DECODER_DRIVER
			pr_info("[SID=0x%08X] Mapped Buffer size: %d (bytes)",
				dd_str_ctx->res_str_id, buf_info->buf_size);
#endif
		} else {
			/*
			 * Same byte interleaved setting should be used.
			 * Convert to actual bools by comparing with zero.
			 */
			if (buf_info->pictbuf_cfg.byte_interleave !=
				dd_str_ctx->map_buf_info.byte_interleave) {
				pr_err("[SID=0x%08X] Buffer cannot be mapped since its byte interleave value (%s) is not the same as buffers already mapped (%s)",
				       dd_str_ctx->res_str_id,
				       buf_info->pictbuf_cfg.byte_interleave ?
				       "ON" : "OFF",
				       dd_str_ctx->map_buf_info.byte_interleave ?
				       "ON" : "OFF");
				ret = IMG_ERROR_INVALID_PARAMETERS;
				goto error;
			}
		}

		/* Configure the buffer.. */
		ret = core_stream_set_pictbuf_config(dd_str_ctx, &buf_info->pictbuf_cfg);
		VDEC_ASSERT(ret == IMG_SUCCESS);
		if (ret != IMG_SUCCESS)
			goto error;
	}

	/* Map heap from VDEC to MMU. */
	switch (buf_type) {
	case VDEC_BUFTYPE_BITSTREAM:
		ddbuf_map_info->mmuheap_id = MMU_HEAP_BITSTREAM_BUFFERS;
		break;

	case VDEC_BUFTYPE_PICTURE:
		mmu_get_heap(buf_info->pictbuf_cfg.stride[VDEC_PLANE_VIDEO_Y],
			     &ddbuf_map_info->mmuheap_id);
		break;

	default:
		VDEC_ASSERT(FALSE);
	}

	/* Map this buffer into the MMU. */
#ifdef	DEBUG_DECODER_DRIVER
	pr_info("----- %s:%d calling MMU_StreamMapExt_sg", __func__, __LINE__);
#endif
	ret =
		mmu_stream_map_ext_sg(dd_str_ctx->mmu_str_handle,
				      (enum mmu_eheap_id)ddbuf_map_info->mmuheap_id,
				      sgt, buf_info->buf_size, DEV_MMU_PAGE_SIZE,
				      buf_info->mem_attrib, buf_info->cpu_linear_addr,
				      &ddbuf_map_info->ddbuf_info,
				      &ddbuf_map_info->buf_id);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		goto error;

	if (buf_type == VDEC_BUFTYPE_PICTURE)
		dd_str_ctx->map_buf_info.num_buf++;

	/*
	 * Initialise the reference count to indicate that the client
	 * still holds the buffer.
	 */
	ddbuf_map_info->ddbuf_info.ref_count = 1;

	/* Return success.. */
	return IMG_SUCCESS;

error:
	if (ddbuf_map_info->res_handle)
		rman_free_resource(ddbuf_map_info->res_handle);
	else
		kfree(ddbuf_map_info);

	return ret;
}

/*
 * @Function  core_stream_unmap_buf
 */
int core_stream_unmap_buf(unsigned int buf_map_id)
{
	int ret;
	struct vdecdd_ddbuf_mapinfo *ddbuf_map_info;
	struct vdecdd_ddstr_ctx *dd_str_ctx;
	struct core_stream_context *core_str_ctx;

	/* Get access to map info context.. */
	ret = rman_get_resource(buf_map_id, VDECDD_BUFMAP_TYPE_ID,
				(void **)&ddbuf_map_info, NULL);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	dd_str_ctx = ddbuf_map_info->ddstr_context;
	VDEC_ASSERT(dd_str_ctx);

	/* Get access to stream context.. */
	ret = rman_get_resource(dd_str_ctx->res_str_id, VDECDD_STREAM_TYPE_ID,
				(void **)&core_str_ctx, NULL);
	VDEC_ASSERT(core_str_ctx);
#ifdef DEBUG_DECODER_DRIVER
	pr_info("UNMAP: PM [0x%p] --> VM [0x%08X - 0x%08X] (%d bytes)",
		ddbuf_map_info->ddbuf_info.cpu_virt,
		ddbuf_map_info->ddbuf_info.dev_virt,
		ddbuf_map_info->ddbuf_info.dev_virt +
		ddbuf_map_info->ddbuf_info.buf_size,
		ddbuf_map_info->ddbuf_info.buf_size);
#endif

	/* Buffer should only be held by the client. */
	VDEC_ASSERT(ddbuf_map_info->ddbuf_info.ref_count == 1);
	if (ddbuf_map_info->ddbuf_info.ref_count != 1)
		return IMG_ERROR_MEMORY_IN_USE;

	ddbuf_map_info->ddbuf_info.ref_count = 0;
	if (ddbuf_map_info->buf_type == VDEC_BUFTYPE_PICTURE) {
		/* Remove this picture buffer from pictbuf list */
		ret = resource_list_remove(&core_str_ctx->pict_buf_list, ddbuf_map_info);

		VDEC_ASSERT(ret == IMG_SUCCESS || ret == IMG_ERROR_COULD_NOT_OBTAIN_RESOURCE);
		if (ret != IMG_SUCCESS && ret != IMG_ERROR_COULD_NOT_OBTAIN_RESOURCE)
			return ret;

		ddbuf_map_info->ddstr_context->map_buf_info.num_buf--;

		/* Clear some state if there are no more mapped buffers. */
		if (dd_str_ctx->map_buf_info.num_buf == 0) {
			dd_str_ctx->map_buf_info.buf_size = 0;
			dd_str_ctx->map_buf_info.byte_interleave = FALSE;
		}
	}

	/* Unmap this buffer from the MMU. */
	ret = mmu_free_mem(dd_str_ctx->mmu_str_handle, &ddbuf_map_info->ddbuf_info);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	/* Free buffer map info. */
	rman_free_resource(ddbuf_map_info->res_handle);

	/* Return success.. */
	return IMG_SUCCESS;
}

/*
 * @Function              core_stream_unmap_buf_sg
 */
int core_stream_unmap_buf_sg(unsigned int buf_map_id)
{
	int ret;
	struct vdecdd_ddbuf_mapinfo *ddbuf_map_info;
	struct vdecdd_ddstr_ctx *dd_str_ctx;
	struct core_stream_context *core_str_ctx;

	/* Get access to map info context.. */
	ret = rman_get_resource(buf_map_id, VDECDD_BUFMAP_TYPE_ID, (void **)&ddbuf_map_info, NULL);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	dd_str_ctx = ddbuf_map_info->ddstr_context;
	VDEC_ASSERT(dd_str_ctx);

	/* Get access to stream context.. */
	ret = rman_get_resource(dd_str_ctx->res_str_id, VDECDD_STREAM_TYPE_ID,
				(void **)&core_str_ctx, NULL);
	VDEC_ASSERT(core_str_ctx);

#ifdef DEBUG_DECODER_DRIVER
	pr_info("UNMAP: PM [0x%p] --> VM [0x%08X - 0x%08X] (%d bytes)",
		ddbuf_map_info->ddbuf_info.cpu_virt,
		ddbuf_map_info->ddbuf_info.dev_virt,
		ddbuf_map_info->ddbuf_info.dev_virt +
		ddbuf_map_info->ddbuf_info.buf_size,
		ddbuf_map_info->ddbuf_info.buf_size);
#endif

	/* Buffer should only be held by the client. */
	VDEC_ASSERT(ddbuf_map_info->ddbuf_info.ref_count == 1);
	if (ddbuf_map_info->ddbuf_info.ref_count != 1)
		return IMG_ERROR_MEMORY_IN_USE;

	ddbuf_map_info->ddbuf_info.ref_count = 0;

	if (ddbuf_map_info->buf_type == VDEC_BUFTYPE_PICTURE) {
		/* Remove this picture buffer from pictbuf list */
		ret = resource_list_remove(&core_str_ctx->pict_buf_list, ddbuf_map_info);

		VDEC_ASSERT(ret == IMG_SUCCESS || ret == IMG_ERROR_COULD_NOT_OBTAIN_RESOURCE);
		if (ret != IMG_SUCCESS && ret != IMG_ERROR_COULD_NOT_OBTAIN_RESOURCE)
			return ret;

		ddbuf_map_info->ddstr_context->map_buf_info.num_buf--;

		/*
		 * Clear some state if there are no more
		 * mapped buffers.
		 */
		if (dd_str_ctx->map_buf_info.num_buf == 0) {
			dd_str_ctx->map_buf_info.buf_size = 0;
			dd_str_ctx->map_buf_info.byte_interleave = FALSE;
		}
	}

	/* Unmap this buffer from the MMU. */
	ret = mmu_free_mem_sg(dd_str_ctx->mmu_str_handle, &ddbuf_map_info->ddbuf_info);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	/* Free buffer map info. */
	rman_free_resource(ddbuf_map_info->res_handle);

	/* Return success.. */
	return IMG_SUCCESS;
}

/*
 * @Function              core_stream_flush
 */
int core_stream_flush(unsigned int res_str_id, unsigned char discard_refs)
{
	struct vdecdd_ddstr_ctx *dd_str_ctx;
	struct core_stream_context *core_str_ctx;
	int ret;

	/* Get access to stream context.. */
	ret = rman_get_resource(res_str_id, VDECDD_STREAM_TYPE_ID,
				(void **)&core_str_ctx, NULL);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	dd_str_ctx = core_str_ctx->dd_str_ctx;

	VDEC_ASSERT(dd_str_ctx);
	VDEC_ASSERT(dd_str_ctx->dd_str_state == VDECDD_STRSTATE_STOPPED);

	/*
	 * If unsupported sequence is found, we need to do additional
	 * check for DPB flush condition
	 */
	if (!dd_str_ctx->comseq_hdr_info.not_dpb_flush) {
		ret = decoder_stream_flush(dd_str_ctx->dec_ctx, discard_refs);
		VDEC_ASSERT(ret == IMG_SUCCESS);
		if (ret != IMG_SUCCESS)
			return ret;
	}

	/* Return success.. */
	return IMG_SUCCESS;
}

/*
 * @Function              core_stream_release_bufs
 */
int core_stream_release_bufs(unsigned int res_str_id, enum vdec_buf_type buf_type)
{
	int ret;
	struct core_stream_context *core_str_ctx;
	struct vdecdd_ddstr_ctx *dd_str_ctx;

	/* Get access to stream context.. */
	ret = rman_get_resource(res_str_id, VDECDD_STREAM_TYPE_ID, (void **)&core_str_ctx, NULL);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	dd_str_ctx = core_str_ctx->dd_str_ctx;

	VDEC_ASSERT(dd_str_ctx);
	VDEC_ASSERT(buf_type < VDEC_BUFTYPE_MAX);

	switch (buf_type) {
	case VDEC_BUFTYPE_PICTURE:
	{
		/* Empty all the decoded picture related buffer lists. */
		ret = resource_list_empty(&core_str_ctx->pict_buf_list, TRUE, NULL, NULL);
		VDEC_ASSERT(ret == IMG_SUCCESS);
		break;
	}

	case VDEC_BUFTYPE_BITSTREAM:
	{
		/* Empty the stream unit queue. */
		ret = resource_list_empty(&core_str_ctx->str_unit_list, FALSE,
					  (resource_pfn_freeitem)core_fn_free_stream_unit,
					   core_str_ctx);
		VDEC_ASSERT(ret == IMG_SUCCESS);
		break;
	}

	case VDEC_BUFTYPE_ALL:
	{
		/* Empty all the decoded picture related buffer lists. */
		ret = resource_list_empty(&core_str_ctx->pict_buf_list, TRUE, NULL, NULL);
		VDEC_ASSERT(ret == IMG_SUCCESS);

		/* Empty the stream unit queue. */
		ret = resource_list_empty(&core_str_ctx->str_unit_list, FALSE,
					  (resource_pfn_freeitem)core_fn_free_stream_unit,
					   core_str_ctx);
		VDEC_ASSERT(ret == IMG_SUCCESS);
		break;
	}

	default:
	{
		ret = IMG_ERROR_INVALID_PARAMETERS;
		VDEC_ASSERT(FALSE);
		break;
	}
	}

	if (buf_type == VDEC_BUFTYPE_PICTURE || buf_type == VDEC_BUFTYPE_ALL) {
		ret = decoder_stream_release_buffers(dd_str_ctx->dec_ctx);
		VDEC_ASSERT(ret == IMG_SUCCESS);
		if (ret != IMG_SUCCESS)
			return ret;
	}

	/* Return success.. */
	return IMG_SUCCESS;
}

/*
 * @Function              core_stream_get_status
 */
int core_stream_get_status(unsigned int res_str_id,
			   struct vdecdd_decstr_status *str_st)
{
	int ret;
	struct core_stream_context *core_str_ctx;
	struct vdecdd_ddstr_ctx *dd_str_ctx;

	/* Get access to stream context.. */
	ret = rman_get_resource(res_str_id, VDECDD_STREAM_TYPE_ID, (void **)&core_str_ctx, NULL);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	dd_str_ctx = core_str_ctx->dd_str_ctx;

	VDEC_ASSERT(dd_str_ctx);
	VDEC_ASSERT(str_st);

	ret = decoder_stream_get_status(dd_str_ctx->dec_ctx, str_st);
	VDEC_ASSERT(ret == IMG_SUCCESS);
	if (ret != IMG_SUCCESS)
		return ret;

	/* Return success.. */
	return IMG_SUCCESS;
}

#ifdef HAS_HEVC
/*
 * @Function              core_free_hevc_picture_resource
 */
static int core_free_hevc_picture_resource(struct core_stream_context *core_strctx,
					   struct vdecdd_pict_resint *pic_res_int)
{
	int ret = IMG_SUCCESS;

	ret = core_free_resbuf(&pic_res_int->genc_fragment_buf,
			       core_strctx->dd_str_ctx->mmu_str_handle);
	if (ret != IMG_SUCCESS)
		pr_err("MMU_Free for Genc Fragment buffer failed with error %u", ret);

	return ret;
}

/*
 * @Function              core_free_hevc_sequence_resource
 */
static int core_free_hevc_sequence_resource(struct core_stream_context *core_strctx,
					    struct vdecdd_seq_resint *seq_res_int)
{
	unsigned int i;
	int local_result = IMG_SUCCESS;
	int ret = IMG_SUCCESS;

	for (i = 0; i < GENC_BUFF_COUNT; ++i) {
		local_result = core_free_resbuf(&seq_res_int->genc_buffers[i],
						core_strctx->dd_str_ctx->mmu_str_handle);
		if (local_result != IMG_SUCCESS) {
			ret = local_result;
			pr_warn("MMU_Free for GENC buffer %u failed with error %u", i,
				local_result);
		}
	}

	local_result = core_free_resbuf(&seq_res_int->intra_buffer,
					core_strctx->dd_str_ctx->mmu_str_handle);
	if (local_result != IMG_SUCCESS) {
		ret = local_result;
		pr_warn("MMU_Free for GENC buffer %u failed with error %u", i, local_result);
	}

	local_result = core_free_resbuf(&seq_res_int->aux_buffer,
					core_strctx->dd_str_ctx->mmu_str_handle);
	if (local_result != IMG_SUCCESS) {
		ret = local_result;
		pr_warn("MMU_Free for GENC buffer %u failed with error %u", i, local_result);
	}

	return ret;
}

/*
 * @Function              core_hevc_bufs_get_size
 */
static int core_hevc_bufs_get_size(struct core_stream_context *core_strctx,
				   const struct vdec_comsequ_hdrinfo *seqhdr_info,
				   struct vdec_pict_size *max_pict_size,
				   struct core_pict_bufsize_info *size_info,
				   struct core_seq_resinfo *seqres_info,
				   unsigned char *resource_needed)
{
	enum vdec_vid_std vid_std = core_strctx->dd_str_ctx->str_config_data.vid_std;
	unsigned int std_idx = vid_std - 1;

	static const unsigned short max_slice_segments_list
		[HEVC_LEVEL_MAJOR_NUM][HEVC_LEVEL_MINOR_NUM] = {
		/* level: 1.0  1.1  1.2  */
		{ 16,    0,   0, },
		/* level: 2.0  2.1  2.2  */
		{ 16,   20,   0, },
		/* level: 3.0  3.1  3.2  */
		{ 30,   40,   0, },
		/* level: 4.0  4.1  4.2  */
		{ 75,   75,   0, },
		/* level: 5.0  5.1  5.2  */
		{ 200, 200, 200, },
		/* level: 6.0  6.1  6.2  */
		{ 600, 600, 600, }
	};

	static const unsigned char max_tile_cols_list
		[HEVC_LEVEL_MAJOR_NUM][HEVC_LEVEL_MINOR_NUM] = {
		/* level: 1.0  1.1  1.2  */
		{   1,   0,   0, },
		/* level: 2.0  2.1  2.2  */
		{   1,   1,   0, },
		/* level: 3.0  3.1  3.2  */
		{   2,   3,   0, },
		/* level: 4.0  4.1  4.2  */
		{   5,   5,   0, },
		/* level: 5.0  5.1  5.2  */
		{  10,  10,  10, },
		/* level: 6.0  6.1  6.2  */
		{  20,  20,  20, }
	};

	/* TRM 3.11.11 */
	static const unsigned int total_sample_per_mb[PIXEL_FORMAT_444 + 1] = {
		256, 384, 384, 512, 768};

	static const unsigned int HEVC_LEVEL_IDC_MIN = 30;
	static const unsigned int HEVC_LEVEL_IDC_MAX = 186;
	static const unsigned int GENC_ALIGNMENT = 0x1000;
	static const unsigned int mb_size = 16;
	static const unsigned int max_mb_rows_in_ctu = 4;
	static const unsigned int bytes_per_fragment_pointer = 16;

	const unsigned int max_tile_height_in_mbs =
		seqhdr_info->max_frame_size.height / mb_size;

	signed char level_maj = seqhdr_info->codec_level / 30;
	signed char level_min = (seqhdr_info->codec_level % 30) / 3;

	/*
	 * If we are somehow able to deliver more information here (CTU size,
	 * number of tile columns/rows) then memory usage could be reduced
	 */
	const struct pixel_pixinfo *pix_info = &seqhdr_info->pixel_info;
	const unsigned int bit_depth = pix_info->bitdepth_y >= pix_info->bitdepth_c ?
		pix_info->bitdepth_y : pix_info->bitdepth_c;
	unsigned short max_slice_segments;
	unsigned char max_tile_cols;
	unsigned int raw_byte_per_mb;
	unsigned int *genc_fragment_bufsize;
	unsigned int *genc_buf_size;

	/* Reset the MB parameters buffer size. */
	size_info->mbparams_bufsize = 0;
	*resource_needed = TRUE;

	if (mbparam_allocinfo[std_idx].alloc_mbparam_bufs) {
		/* shall be == 64 (0x40)*/
		const unsigned int align = mbparam_allocinfo[std_idx].mbparam_size;
		const unsigned int dpb_width = (max_pict_size->width + align * 2 - 1) / align * 2;
		const unsigned int pic_height = (max_pict_size->height + align - 1) / align;
		const unsigned int pic_width = (max_pict_size->width + align - 1) / align;

		/* calculating for worst case: max frame size, B-frame */
		size_info->mbparams_bufsize = (align * 2) * pic_width * pic_height +
			align * dpb_width * pic_height;

		/* Adjust the buffer size for MSVDX. */
		vdecddutils_buf_vxd_adjust_size(&size_info->mbparams_bufsize);
	}

	if (seqhdr_info->codec_level > HEVC_LEVEL_IDC_MAX ||
	    seqhdr_info->codec_level < HEVC_LEVEL_IDC_MIN) {
		level_maj = 6;
		level_min = 2;
	}

	if (level_maj > 0 && level_maj <= HEVC_LEVEL_MAJOR_NUM &&
	    level_min >= 0 && level_min < HEVC_LEVEL_MINOR_NUM) {
		max_slice_segments = max_slice_segments_list[level_maj - 1][level_min];
		max_tile_cols = max_tile_cols_list[level_maj - 1][level_min];
	} else {
		pr_err("%s: Invalid parameters\n", __func__);
		return IMG_ERROR_INVALID_PARAMETERS;
	}

	raw_byte_per_mb = total_sample_per_mb[pix_info->chroma_fmt_idc] *
		VDEC_ALIGN_SIZE(bit_depth, 8, unsigned int, int) / 8;

	genc_fragment_bufsize = &size_info->hevc_bufsize_pict.genc_fragment_bufsize;
	genc_buf_size = &seqres_info->hevc_bufsize_seqres.genc_bufsize;

	*genc_fragment_bufsize = bytes_per_fragment_pointer * (seqhdr_info->max_frame_size.height /
		mb_size * max_tile_cols + max_slice_segments - 1) * max_mb_rows_in_ctu;

	/*
	 * GencBufferSize formula is taken from TRM and found by HW * CSIM teams for a sensible
	 * streams i.e. size_of_stream < size_of_output_YUV. In videostream data base it's
	 * possible to find pathological Argon streams that do not fulfill this sensible
	 * requirement. eg. #58417, #58419, #58421, #58423. To make a #58417 stream running the
	 * formula below should be changed from (2 * 384) *... ---> (3 * 384) *...
	 * This solution is applied by DEVA.
	 */
	*genc_buf_size = 2 * raw_byte_per_mb * seqhdr_info->max_frame_size.width /
		mb_size * max_tile_height_in_mbs / 4;

	*genc_buf_size = VDEC_ALIGN_SIZE(*genc_buf_size, GENC_ALIGNMENT,
					 unsigned int, unsigned int);
	*genc_fragment_bufsize = VDEC_ALIGN_SIZE(*genc_fragment_bufsize, GENC_ALIGNMENT,
						 unsigned int, unsigned int);

#ifdef DEBUG_DECODER_DRIVER
	pr_info("Sizes for GENC in HEVC: 0x%X (frag), 0x%X (x4)",
		*genc_fragment_bufsize,
		*genc_buf_size);
#endif

	seqres_info->hevc_bufsize_seqres.intra_bufsize = 4 * seqhdr_info->max_frame_size.width;
	if (seqhdr_info->pixel_info.mem_pkg != PIXEL_BIT8_MP)
		seqres_info->hevc_bufsize_seqres.intra_bufsize *= 2;

	seqres_info->hevc_bufsize_seqres.aux_bufsize = (512 * 1024);

	return IMG_SUCCESS;
}

/*
 * @Function              core_is_hevc_stream_resource_suitable
 */
static unsigned char
core_is_hevc_stream_resource_suitable(struct core_pict_resinfo *pict_res_info,
				      struct core_pict_resinfo *old_pict_res_info,
				      struct core_seq_resinfo *seq_res_info,
				      struct core_seq_resinfo *old_seq_res_info)
{
	return (seq_res_info->hevc_bufsize_seqres.genc_bufsize <=
	       old_seq_res_info->hevc_bufsize_seqres.genc_bufsize &&
	       seq_res_info->hevc_bufsize_seqres.intra_bufsize <=
	       old_seq_res_info->hevc_bufsize_seqres.intra_bufsize &&
	       seq_res_info->hevc_bufsize_seqres.aux_bufsize <=
	       old_seq_res_info->hevc_bufsize_seqres.aux_bufsize &&
	       pict_res_info->size_info.hevc_bufsize_pict.genc_fragment_bufsize <=
	       old_pict_res_info->size_info.hevc_bufsize_pict.genc_fragment_bufsize);
}

/*
 * @Function  core_alloc_hevc_specific_seq_buffers
 */
static int
core_alloc_hevc_specific_seq_buffers(struct core_stream_context *core_strctx,
				     struct vdecdd_seq_resint *seqres_int,
				     struct vxdio_mempool mempool,
				     struct core_seq_resinfo *seqres_info)
{
	unsigned int i;
	int ret = IMG_SUCCESS;

	/* Allocate GENC buffers */
	for (i = 0; i < GENC_BUFF_COUNT; ++i) {
		/* Allocate the GENC buffer info structure. */
		ret = core_alloc_resbuf(&seqres_int->genc_buffers[i],
					seqres_info->hevc_bufsize_seqres.genc_bufsize,
					core_strctx->dd_str_ctx->mmu_str_handle,
					mempool);
		if (ret != IMG_SUCCESS)
			return ret;
	}

	seqres_int->genc_buf_id = ++core_strctx->std_spec_context.hevc_ctx.genc_id_gen;

	/* Allocate the intra buffer info structure. */
	ret = core_alloc_resbuf(&seqres_int->intra_buffer,
				seqres_info->hevc_bufsize_seqres.intra_bufsize,
				core_strctx->dd_str_ctx->mmu_str_handle,
				mempool);
	if (ret != IMG_SUCCESS)
		return ret;

	/* Allocate the aux buffer info structure. */
	ret = core_alloc_resbuf(&seqres_int->aux_buffer,
				seqres_info->hevc_bufsize_seqres.aux_bufsize,
				core_strctx->dd_str_ctx->mmu_str_handle,
				mempool);
	if (ret != IMG_SUCCESS)
		return ret;

	return IMG_SUCCESS;
}

/*
 * @Function              core_alloc_hevc_specific_pict_buffers
 */
static int
core_alloc_hevc_specific_pict_buffers(struct core_stream_context *core_strctx,
				      struct vdecdd_pict_resint *pict_res_int,
				      struct vxdio_mempool mempool,
				      struct core_pict_resinfo *pict_res_info)
{
	int ret;

	/* Allocate the GENC fragment buffer. */
	ret = core_alloc_resbuf(&pict_res_int->genc_fragment_buf,
				pict_res_info->size_info.hevc_bufsize_pict.genc_fragment_bufsize,
				core_strctx->dd_str_ctx->mmu_str_handle,
				mempool);

	return ret;
}
#endif
