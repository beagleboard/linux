/* SPDX-License-Identifier: GPL-2.0 */
/*
 * IMG DEC SYSDEV and UI Interface header
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

#ifndef _VXD_DEC_H
#define _VXD_DEC_H

#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>
#include <linux/types.h>

#include "bspp.h"
#include "img_dec_common.h"
#include "img_mem_man.h"
#include "img_pixfmts.h"
#include "pixel_api.h"
#include "vdecdd_defs.h"
#include "vdec_defs.h"
#include "work_queue.h"

#define VXD_MIN_STREAM_ID 1
#define VXD_MAX_STREAMS_PER_DEV 254
#define VXD_MAX_STREAM_ID (VXD_MIN_STREAM_ID + VXD_MAX_STREAMS_PER_DEV)

#define CODEC_NONE -1
#define CODEC_H264_DEC 0
#define CODEC_MPEG4_DEC 1
#define CODEC_VP8_DEC 2
#define CODEC_VC1_DEC 3
#define CODEC_MPEG2_DEC 4
#define CODEC_JPEG_DEC 5
#define CODEC_VP9_DEC 6
#define CODEC_HEVC_DEC 7

#define MAX_SEGMENTS 6
#define HW_ALIGN 64

#define MAX_BUF_TRACE 30

#define MAX_CAPBUFS_H264 16
#define DISPLAY_LAG 3
#define HEVC_MAX_LUMA_PS 35651584

#define MAX_PLANES 3

enum {
	Q_DATA_SRC         = 0,
	Q_DATA_DST         = 1,
	Q_DATA_FORCE32BITS = 0x7FFFFFFFU
};

enum {
	IMG_DEC_FMT_TYPE_CAPTURE     = 0x01,
	IMG_DEC_FMT_TYPE_OUTPUT      = 0x10,
	IMG_DEC_FMT_TYPE_FORCE32BITS = 0x7FFFFFFFU
};

enum vxd_map_flags {
	VXD_MAP_FLAG_NONE        = 0x0,
	VXD_MAP_FLAG_READ_ONLY   = 0x1,
	VXD_MAP_FLAG_WRITE_ONLY  = 0x2,
	VXD_MAP_FLAG_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * struct vxd_fw_msg - This structure holds the information about the message
 *                     exchanged in read/write between Kernel and firmware.
 *
 * @out_flags: indicating the type of message
 * @payload_size: size of payload in bytes
 * @payload: data which is send to firmware
 */
struct vxd_fw_msg {
	unsigned int out_flags;
	unsigned int payload_size;
	unsigned int payload[0];
};

/* HW state */
struct vxd_hw_state {
	unsigned int fw_counter;
	unsigned int fe_status[VXD_MAX_PIPES];
	unsigned int be_status[VXD_MAX_PIPES];
	unsigned int dmac_status[VXD_MAX_PIPES][2]; /* Cover DMA chan 2/3*/
	unsigned int irq_status;
};

/*
 * struct vxd_state - contains VXD HW state
 *
 * @hw_state: HW state
 * @msg_id_tail: msg id of the oldest item being processed
 */
struct vxd_state {
	struct vxd_hw_state hw_state;
	unsigned short msg_id_tail;
};

/*
 * struct vxd_dec_fmt - contains info for each of the supported video format
 *
 * @fourcc: V4L2 pixel format FCC identifier
 * @num_planes: number of planes required for luma and chroma
 * @type: CAPTURE or OUTPUT
 * @std: VDEC video standard
 * @pixfmt: IMG pixel format
 * @interleave: Chroma interleave order
 * @idc: Chroma format
 * @size_num: Numberator used to calculate image size
 * @size_den: Denominator used to calculate image size
 * @bytes_pp: Bytes per pixel for this format
 */
struct vxd_dec_fmt {
	unsigned int fourcc;
	unsigned int num_planes;
	unsigned char type;
	enum vdec_vid_std std;
	enum img_pixfmt pixfmt;
	enum pixel_chroma_interleaved interleave;
	enum pixel_fmt_idc idc;
	int size_num;
	int size_den;
	int bytes_pp;
};

/*
 * struct vxd_item - contains information about the item sent to fw
 *
 * @list: item to be linked list to items_done, msgs, or pend.
 * @stream_id: stream id
 * @msg_id: message id
 * @destroy: item belongs to the stream which is destroyed
 * @msg: contains msg between kernel and fw
 */
struct vxd_item {
	struct list_head list;
	unsigned int stream_id;
	unsigned int msg_id;
	struct {
		unsigned destroy : 1;
	};
	struct vxd_fw_msg msg;
};

enum vxd_cb_type {
	VXD_CB_STRUNIT_PROCESSED,
	VXD_CB_SPS_RELEASE,
	VXD_CB_PPS_RELEASE,
	VXD_CB_PICT_DECODED,
	VXD_CB_PICT_DISPLAY,
	VXD_CB_PICT_RELEASE,
	VXD_CB_PICT_END,
	VXD_CB_STR_END,
	VXD_CB_ERROR_FATAL,
	VXD_CB_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * vxd_cb - Return a resource to vxd
 *
 * @ctx: the vxd stream context
 * @type: the type of message
 * @buf_map_id: the buf_map_id of the resource being returned
 */
typedef void (*vxd_cb)(void *ctx, enum vxd_cb_type type, unsigned int buf_map_id);

/*
 * struct vxd_return - contains information about items returning from core
 *
 * @type: Type of item being returned
 * @buf_map_id: mmu mapped id of buffer being returned
 */
struct vxd_return {
	void *work;
	struct vxd_dec_ctx *ctx;
	enum vxd_cb_type type;
	unsigned int buf_map_id;
};

/*
 * struct vxd_dec_q_data - contains queue data information
 *
 * @fmt: format info
 * @width: frame width
 * @height: frame height
 * @bytesperline: bytes per line in memory
 * @size_image: image size in memory
 */
struct vxd_dec_q_data {
	struct vxd_dec_fmt *fmt;
	unsigned int width;
	unsigned int height;
	unsigned int bytesperline[MAX_PLANES];
	unsigned int size_image[MAX_PLANES];
};

/*
 * struct time_prof - contains time taken by decoding information
 *
 * @id: id info
 * @start_time: start time
 * @end_time: end time
 */
struct time_prof {
	unsigned int id;
	long long start_time;
	long long end_time;
};

/*
 * struct vxd_dev - The struct containing decoder driver internal parameters.
 *
 * @v4l2_dev: main struct of V4L2 device drivers
 * @dev: platform device driver
 * @vfd_dec: video device structure to create and manage the V4L2 device node.
 * @plat_dev: linux platform device
 * @struct v4l2_m2m_dev: mem2mem device
 * @mutex: mutex to protect certain ongoing operation.
 * @module_irq: a threaded request IRQ for the device
 * @reg_base: base address of the IMG VXD hw registers
 * @props: contains HW properties
 * @mmu_config_addr_width: indicates the number of extended address bits
 *                         (above 32) that the external memory interface
 *                         uses, based on EXTENDED_ADDR_RANGE field of
 *                         MMU_CONFIG0
 * @rendec_buf_id: buffer id for rendec buffer allocation
 * @firmware: firmware information based on vxd_dev_fw structure
 * @firmware_loading_complete: loading completion
 * @no_fw: Just to check if firmware is present in /lib
 * @fw_refcnt: firmware reference counter
 * @hw_on: indication if hw is on or off
 * @hw_dead: indication if hw is dead
 * @lock: basic primitive for locking through spinlock
 * @state: internal state handling of vxd state
 * @msgs: linked list of msgs with vxd_item
 * @pend: linked list of pending msgs to be sent to fw
 * @msg_cnt: counter of messages submitted to VXD. Wraps every VXD_MSG_ID_MASK
 * @freq_khz: Core clock frequency measured during boot of firmware
 * @streams: unique id for the stream
 * @mem_ctx: memory management context for HW buffers
 * @dwork: use for Power Management and Watchdog
 * @work_sched_at: the time of the last work has been scheduled at
 * @emergency: indicates if emergency condition occurred
 * @dbgfs_ctx: pointer to debug FS context.
 * @hw_pm_delay: delay before performaing PM
 * @hw_dwr_period: period for checking for dwr
 * @pm_start: time, in jiffies, when core become idle
 * @dwr_start: time, in jiffies, when dwr has been started
 */
struct vxd_dev {
	struct v4l2_device v4l2_dev;
	void *dev;
	struct video_device *vfd_dec;
	struct platform_device *plat_dev;
	struct v4l2_m2m_dev *m2m_dev;
	struct mutex  *mutex; /* Per device mutex */
	struct mutex  *mutex_queue; /* Mutex for ioctl synchronization on queue */
	int module_irq;
	void __iomem *reg_base;
	struct vxd_core_props props;
	unsigned int mmu_config_addr_width;
	int rendec_buf_id;
	struct vxd_dev_fw firmware;
	void *firmware_loading_complete;
	unsigned char no_fw;
	unsigned char fw_refcnt;
	unsigned int hw_on;
	unsigned int hw_dead;
	void *lock; /* basic device level spinlock */
	struct vxd_state state;
	struct list_head msgs;
	struct list_head pend;
	int msg_cnt;
	unsigned int freq_khz;
	struct idr *streams;
	struct mem_ctx *mem_ctx;
	void *dwork;
	unsigned long long work_sched_at;
	unsigned int emergency;
	void *dbgfs_ctx;
	unsigned int hw_pm_delay;
	unsigned int hw_dwr_period;
	unsigned long long pm_start;
	unsigned long long dwr_start;
	struct time_prof time_fw[MAX_BUF_TRACE];
	struct time_prof time_drv[MAX_BUF_TRACE];

	/* The variables defined below are used in RTOS only. */
	/* This variable holds queue handler */
	void *vxd_worker_queue_handle;
	void *vxd_worker_queue_sem_handle;
};

/*
 * struct vxd_stream - holds stream-related info
 *
 * @ctx: associated vxd_dec_ctx
 * @mmu_ctx: MMU context for this stream
 * @ptd: ptd for the stream
 * @id: unique stream id
 */
struct vxd_stream {
	struct vxd_dec_ctx *ctx;
	struct mmu_ctx *mmu_ctx;
	unsigned int ptd;
	unsigned int id;
};

/*
 * struct vxd_buffer - holds per buffer info.
 * @buffer: the vb2_v4l2_buffer
 * @list: list head for gathering in linked list
 * @mapped: is this buffer mapped yet
 * @reuse: is the buffer ready for reuse
 * @buf_map_id: the mapped buffer id
 * @buf_info: the buffer info for submitting to map
 * @bstr_info: the buffer info for submitting to bspp
 * @seq_unit: the str_unit for submitting sps
 * @seq_unit: the str_unit for submitting pps and segments
 * @seq_unit: the str_unit for submitting picture_end
 */
struct vxd_buffer {
	struct v4l2_m2m_buffer buffer;
	struct list_head list;
	unsigned char mapped;
	unsigned char reuse;
	unsigned int buf_map_id;
	struct vdec_buf_info buf_info;
	struct bspp_ddbuf_info bstr_info;
	struct vdecdd_str_unit seq_unit;
	struct vdecdd_str_unit pic_unit;
	struct vdecdd_str_unit end_unit;
	struct bspp_preparsed_data preparsed_data;
};

typedef void (*decode_cb)(int res_str_id, unsigned int *msg, unsigned int msg_size,
			  unsigned int msg_flags);

/*
 * struct vxd_dec_ctx - holds per stream data. Each playback has its own
 *                      vxd_dec_ctx
 *
 * @fh: V4L2 file handler
 * @dev: pointer to the device main information.
 * @ctrl_hdl_dec: v4l2 custom control command for video decoder
 * @mem_ctx: mem context for this stream
 * @mmu_ctx: MMU context for this stream
 * @ptd: page table information
 * @items_done: linked list of items is ready
 * @width: frame width
 * @height: frame height
 * @width_orig: original frame width (before padding)
 * @height_orig: original frame height (before padding)
 * @q_data: Queue data information of src[0] and dst[1]
 * @stream: stream-related info
 * @work: work queue for message handling
 * @return_queue: list of resources returned from core
 * @out_buffers: list of all output buffers
 * @cap_buffers: list of all capture buffers except those in reuse_queue
 * @reuse_queue: list of capture buffers waiting for core to signal reuse
 * @res_str_id: Core stream id
 * @stream_created: Core stream is created
 * @stream_configured: Core stream is configured
 * @opconfig_pending: Core opconfig is pending stream_create
 * @src_streaming: V4L2 src stream is streaming
 * @dst_streaming: V4L2 dst stream is streaming
 * @core_streaming: core is streaming
 * @aborting: signal job abort on next irq
 * @str_opcfg: core output config
 * @pict_bufcfg: core picture buffer config
 * @bspp_context: BSPP Stream context handle
 * @seg_list: list of bspp_bitstr_seg for submitting to BSPP
 * @fw_sequ: BSPP sps resource
 * @fw_pps: BSPP pps resource
 * @cb: registered callback for incoming messages
 * @mutex: mutex to protect context specific state machine
 */
struct vxd_dec_ctx {
	struct v4l2_fh fh;
	struct vxd_dev *dev;
	struct mem_ctx *mem_ctx;
	struct mmu_ctx *mmu_ctx;
	unsigned int ptd;
	struct list_head items_done;
	unsigned int width;
	unsigned int height;
	unsigned int width_orig;
	unsigned int height_orig;
	struct vxd_dec_q_data q_data[2];
	struct vxd_stream stream;
	void *work;
	struct list_head return_queue;
	struct list_head out_buffers;
	struct list_head cap_buffers;
	struct list_head reuse_queue;
	unsigned int res_str_id;
	unsigned char stream_created;
	unsigned char stream_configured;
	unsigned char opconfig_pending;
	unsigned char src_streaming;
	unsigned char dst_streaming;
	unsigned char core_streaming;
	unsigned char aborting;
	unsigned char eos;
	unsigned char stop_initiated;
	unsigned char flag_last;
	unsigned char num_decoding;
	unsigned int max_num_ref_frames;
	struct vdec_str_opconfig str_opcfg;
	struct vdec_pict_bufconfig pict_bufcfg;
	void *bspp_context;
	struct bspp_bitstr_seg bstr_segments[MAX_SEGMENTS];
	struct lst_t seg_list;
	struct bspp_ddbuf_array_info fw_sequ[MAX_SEQUENCES];
	struct bspp_ddbuf_array_info fw_pps[MAX_PPSS];
	decode_cb cb;
	struct mutex *mutex; /* Per stream mutex */

	/* The below variable used only in Rtos */
	void *mm_return_resource; /* Place holder for CB to application */
	void *stream_worker_queue_handle;
	void *stream_worker_queue_sem_handle;
	// lock is used to synchronize the stream worker and process function
	void *lock;
	/* "sem_eos" this semaphore variable used to wait until all frame decoded */
	void *sem_eos;
};

irqreturn_t vxd_handle_irq(void *dev);
irqreturn_t vxd_handle_thread_irq(void *dev);
int vxd_init(void *dev, struct vxd_dev *vxd, const struct heap_config heap_configs[], int heaps);
int vxd_g_internal_heap_id(void);
void vxd_deinit(struct vxd_dev *vxd);
int vxd_prepare_fw(struct vxd_dev *vxd);
void vxd_clean_fw_resources(struct vxd_dev *vxd);
int vxd_send_msg(struct vxd_dec_ctx *ctx, struct vxd_fw_msg *msg);
int vxd_suspend_dev(void *dev);
int vxd_resume_dev(void *dev);

int vxd_create_ctx(struct vxd_dev *vxd, struct vxd_dec_ctx *ctx);
void vxd_destroy_ctx(struct vxd_dev *vxd, struct vxd_dec_ctx *ctx);

int vxd_map_buffer_sg(struct vxd_dev *vxd, struct vxd_dec_ctx *ctx,
		      unsigned int str_id, unsigned int buff_id,
		      void *sgt, unsigned int virt_addr,
		      unsigned int map_flags);
int vxd_map_buffer(struct vxd_dev *vxd, struct vxd_dec_ctx *ctx, unsigned int str_id,
		   unsigned int buff_id, unsigned int virt_addr, unsigned int map_flags);
int vxd_unmap_buffer(struct vxd_dev *vxd, struct vxd_dec_ctx *ctx,
		     unsigned int str_id, unsigned int buff_id);

unsigned int get_nbuffers(enum vdec_vid_std std, int w, int h, unsigned int max_num_ref_frames);

int vxd_dec_alloc_bspp_resource(struct vxd_dec_ctx *ctx, enum vdec_vid_std vid_std);

#ifdef ERROR_RECOVERY_SIMULATION
/* sysfs read write functions */
ssize_t vxd_sysfs_show(struct kobject *vxd_dec_kobject,
		       struct kobj_attribute *attr, char *buf);

ssize_t vxd_sysfs_store(struct kobject *vxd_dec_kobject,
			struct kobj_attribute *attr, const char *buf, unsigned long count);
#endif
#endif /* _VXD_DEC_H */
