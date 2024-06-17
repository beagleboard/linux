/* SPDX-License-Identifier: GPL-2.0 */
/*
 * topaz driver data strcutures
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

#if !defined(__TOPAZ_DEVICE_H__)
#define __TOPAZ_DEVICE_H__

#include <linux/interrupt.h>

#include "fw_headers/topazscfwif.h"
#include "fw_headers/mtx_fwif.h"
#include "topazmmu.h"
#include "vid_buf.h"
#include "topaz_api.h"

#       define CODEC_MASK_JPEG          0x0001
#       define CODEC_MASK_MPEG2         0x0002
#       define CODEC_MASK_MPEG4         0x0004
#       define CODEC_MASK_H263          0x0008
#       define CODEC_MASK_H264          0x0010
#       define CODEC_MASK_H264MVC       0x0020
#       define CODEC_MASK_VP8           0x0040
#       define CODEC_MASK_H265          0x0080
#       define CODEC_MASK_FAKE          0x007F

struct img_comm_socket;

/*!
 ****************************************************************************
 Event object structure
 ****************************************************************************
 */
struct event {
	unsigned char signalled;
};

/* prototype for callback for incoming message */
typedef void (*enc_cb)(struct img_writeback_msg *msg, void *priv);

#ifdef ENABLE_PROFILING
struct enc_fw_latency {
	unsigned int start_time;
	unsigned int end_time;
};
#endif

struct mtx_tohost_msg {
	enum mtx_message_id cmd_id;
	unsigned int input_cmd_word;
	unsigned char coded_pkg_idx;
	unsigned int wb_val;
	unsigned int data;
	struct vidio_ddbufinfo *command_data_buf;
};

struct mtx_tomtx_msg {
	enum mtx_cmd_id cmd_id;
	unsigned int data;
	struct vidio_ddbufinfo *command_data_buf;
};

/*
 * This structure contains the device context.
 */
struct topaz_dev_ctx {
	/* Parent context, needed to pass to mmu_alloc */
	void *vxe_arg;

	/* KM addresses for mem spaces */
	void *multi_core_mem_addr;
	void *hp_core_reg_addr[TOPAZHP_MAX_NUM_PIPES];
	void *vlc_reg_addr[TOPAZHP_MAX_NUM_PIPES];

	unsigned char initialized; /*!< Indicates that the device driver has been initialised */

	unsigned int used_socks;
	struct img_comm_socket *socks[TOPAZHP_MAX_POSSIBLE_STREAMS];

	unsigned int fw_uploaded;
	struct img_fw_context fw_ctx;

	void *lock; /* basic device level spinlock */
	struct mutex *comm_tx_mutex;
	struct mutex *comm_rx_mutex;

	unsigned int ptd;
	struct topaz_mmu_context topaz_mmu_ctx;
};

#define COMM_INCOMING_FIFO_SIZE (WB_FIFO_SIZE * 2)
struct img_comm_socket {
	unsigned char id;
	unsigned int low_cmd_cnt;   /* count of low-priority commands sent to TOPAZ */
	unsigned int high_cmd_cnt;  /* count of high-priority commands sent to TOPAZ */
	unsigned int last_sync;     /* Last sync value sent */
	struct  img_writeback_msg in_fifo[COMM_INCOMING_FIFO_SIZE];
	unsigned int in_fifo_consumer;
	unsigned int in_fifo_producer;
	void *work;

	enc_cb cb;                            /* User-provided callback function */
	struct topaz_stream_context *str_ctx; /* User-provided callback data */

	void *event;
	unsigned char sync_waiting;
	unsigned int sync_wb_val;
	struct mutex *sync_wb_mutex;

	unsigned int msgs_sent;
	unsigned int ack_recv;
	unsigned char is_serialized;

	unsigned int codec;

	struct topaz_dev_ctx *ctx;
#ifdef ENABLE_PROFILING
	struct enc_fw_latency fw_lat;
#endif
};

unsigned char topazdd_threaded_isr(void *data);
irqreturn_t topazdd_isr(void *data);

int topazdd_init(unsigned long long reg_base, unsigned int reg_size,
		 unsigned int mmu_flags,
		 void *vxe_arg, unsigned int ptd, void **data);
void topazdd_deinit(void *data);
unsigned int topazdd_get_num_pipes(struct topaz_dev_ctx *ctx);
unsigned int topazdd_get_core_rev(void);
unsigned int topazdd_get_core_des1(void);
unsigned char topazdd_is_idle(struct img_comm_socket *sock);

int topazdd_upload_firmware(struct topaz_dev_ctx *ctx, enum img_codec codec);
int topazdd_create_stream_context(struct topaz_dev_ctx *ctx, enum img_codec codec, enc_cb cb,
				  void *cb_priv, void **dd_str_ctx,
				  struct vidio_ddbufinfo **wb_data_info);
void topazdd_destroy_stream_ctx(void *dd_str_ctx);
int topazdd_setup_stream_ctx(void *dd_str_ctx, unsigned short height,
			     unsigned short width, unsigned char *ctx_num,
			     unsigned int *used_sock);
int topazdd_send_msg(void *dd_str_ctx, enum mtx_cmd_id cmd_id,
		     unsigned int data, struct vidio_ddbufinfo *cmd_data_buf,
		     unsigned int *wb_val);
int topazdd_send_msg_with_sync(void *dd_str_ctx, enum mtx_cmd_id cmd_id,
			       unsigned int data,
			       struct vidio_ddbufinfo *cmd_data_buf);

extern unsigned int mmu_control_val;

#endif /* __TOPAZ_DEVICE_H__	*/
