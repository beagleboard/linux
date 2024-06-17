// SPDX-License-Identifier: GPL-2.0
/*
 * VXD DEC Hardware control implementation
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

#include <linux/types.h>
#include <linux/dma-mapping.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>

#include "decoder.h"
#include "hw_control.h"
#include "img_msvdx_vdmc_regs.h"
#include "img_pvdec_core_regs.h"
#include "img_pvdec_pixel_regs.h"
#include "img_pvdec_test_regs.h"
#include "img_vdec_fw_msg.h"
#include "img_video_bus4_mmu_regs.h"
#include "img_msvdx_core_regs.h"
#include "reg_io2.h"
#include "vdecdd_defs.h"
#include "vxd_dec.h"
#include "vxd_ext.h"
#include "vxd_int.h"
#include "vxd_pvdec_priv.h"

#define MSG_GROUP_MASK  0xf0

struct hwctrl_ctx {
	unsigned int is_initialised;
	unsigned int is_on_seq_replay;
	unsigned int replay_tid;
	unsigned int num_pipes;
	struct vdecdd_dd_devconfig devconfig;
	void *hndl_vxd;
	void *dec_core;
	void *comp_init_userdata;
	struct vidio_ddbufinfo dev_ptd_bufinfo;
	struct lst_t pend_pict_list;
	struct hwctrl_msgstatus host_msg_status;
	void *hmsg_task_event;
	void *hmsg_task_kick;
	void *hmsg_task;
	unsigned int is_msg_task_active;
	struct hwctrl_state state;
	struct hwctrl_state prev_state;
	unsigned int is_prev_hw_state_set;
	unsigned int is_fatal_state;
};

struct vdeckm_context {
	unsigned int core_num;
	struct vxd_coreprops props;
	unsigned short current_msgid;
	unsigned char reader_active;
	void *comms_ram_addr;
	unsigned int state_offset;
	unsigned int state_size;
};

/*
 * Panic reason identifier.
 */
enum pvdec_panic_reason {
	PANIC_REASON_OTHER = 0,
	PANIC_REASON_WDT,
	PANIC_REASON_READ_TIMEOUT,
	PANIC_REASON_CMD_TIMEOUT,
	PANIC_REASON_MMU_FAULT,
	PANIC_REASON_MAX,
	PANIC_REASON_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * Panic reason strings.
 * NOTE: Should match the pvdec_panic_reason ids.
 */
static unsigned char *apanic_reason[PANIC_REASON_MAX] = {
	[PANIC_REASON_OTHER] = "Other",
	[PANIC_REASON_WDT] = "Watch Dog Timeout",
	[PANIC_REASON_READ_TIMEOUT] = "Read Timeout",
	[PANIC_REASON_CMD_TIMEOUT] = "Command Timeout",
	[PANIC_REASON_MMU_FAULT] = "MMU Page Fault"
};

/*
 * Maximum length of the panic reason string.
 */
#define PANIC_REASON_LEN  (255)

static struct vdeckm_context acore_ctx[VXD_MAX_CORES] = {0};

static int vdeckm_getregsoffsets(const void *hndl_vxd,
				 struct decoder_regsoffsets *regs_offsets)
{
	struct vdeckm_context *core_ctx = (struct vdeckm_context *)hndl_vxd;

	if (!core_ctx)
		return IMG_ERROR_INVALID_PARAMETERS;

	regs_offsets->vdmc_cmd_offset = MSVDX_CMD_OFFSET;
	regs_offsets->vec_offset = MSVDX_VEC_OFFSET;
	regs_offsets->entropy_offset = PVDEC_ENTROPY_OFFSET;
	regs_offsets->vec_be_regs_offset = PVDEC_VEC_BE_OFFSET;
	regs_offsets->vdec_be_codec_regs_offset = PVDEC_VEC_BE_CODEC_OFFSET;

	return IMG_SUCCESS;
}

static int vdeckm_send_message(const void *hndl_vxd,
			       struct hwctrl_to_kernel_msg *to_kernelmsg,
			       void *vxd_dec_ctx)
{
	struct vdeckm_context *core_ctx = (struct vdeckm_context *)hndl_vxd;
	unsigned int count = 0;
	unsigned int *msg;

	if (!core_ctx || !to_kernelmsg)
		return IMG_ERROR_INVALID_PARAMETERS;

	msg = kzalloc(VXD_SIZE_MSG_BUFFER, GFP_KERNEL);
	if (!msg)
		return IMG_ERROR_OUT_OF_MEMORY;

	msg[count++] = to_kernelmsg->flags;
	msg[count++] = to_kernelmsg->msg_size;

	memcpy(&msg[count], to_kernelmsg->msg_hdr, to_kernelmsg->msg_size);

	core_ctx->reader_active = 1;

	if (!(to_kernelmsg->msg_hdr)) {
		kfree(msg);
		return IMG_ERROR_INVALID_PARAMETERS;
	}

	pr_debug("[HWCTRL] adding message to vxd queue\n");
	vxd_send_msg(vxd_dec_ctx, (struct vxd_fw_msg *)msg);

	kfree(msg);

	return 0;
}

static void vdeckm_return_msg(const void *hndl_vxd,
			      struct hwctrl_to_kernel_msg *to_kernelmsg)
{
	if (to_kernelmsg)
		kfree(to_kernelmsg->msg_hdr);
}

static int vdeckm_handle_mtxtohost_msg(unsigned int *msg, struct lst_t *pend_pict_list,
				       enum vxd_msg_attr *msg_attr,
				       struct dec_decpict  **decpict,
				       unsigned char msg_type,
				       unsigned int trans_id)
{
	struct dec_decpict *pdec_pict;

	int ret = 0;
	switch (msg_type) {
	case FW_DEVA_COMPLETED:
	{
		struct dec_pict_attrs *pict_attrs = NULL;
		unsigned short error_flags = 0;
		unsigned int no_bewdts = 0;
		unsigned int mbs_dropped = 0;
		unsigned int mbs_recovered = 0;
		unsigned char flag = 0;

		pr_debug("Received message from firmware\n");
		error_flags = MEMIO_READ_FIELD(msg, FW_DEVA_COMPLETED_ERROR_FLAGS);

		no_bewdts = MEMIO_READ_FIELD(msg, FW_DEVA_COMPLETED_NUM_BEWDTS);

		mbs_dropped = MEMIO_READ_FIELD(msg, FW_DEVA_COMPLETED_NUM_MBSDROPPED);

		mbs_recovered = MEMIO_READ_FIELD(msg, FW_DEVA_COMPLETED_NUM_MBSRECOVERED);

		pdec_pict = lst_first(pend_pict_list);
		while (pdec_pict) {
			if (pdec_pict->transaction_id == trans_id)
				break;
			pdec_pict = lst_next(pdec_pict);
		}
		/*
		 * We must have a picture in the list that matches
		 * the transaction id
		 */
		if (!pdec_pict)
			return IMG_ERROR_FATAL;

		if (!(pdec_pict->first_fld_fwmsg) || !(pdec_pict->second_fld_fwmsg))
			return IMG_ERROR_FATAL;

		flag = pdec_pict->first_fld_fwmsg->pict_attrs.first_fld_rcvd;
		if (flag) {
			pict_attrs = &pdec_pict->second_fld_fwmsg->pict_attrs;
		} else {
			pict_attrs = &pdec_pict->first_fld_fwmsg->pict_attrs;
			flag = 1;
		}

		pict_attrs->fe_err = (unsigned int)error_flags;
		pict_attrs->no_be_wdt = no_bewdts;
		pict_attrs->mbs_dropped = mbs_dropped;
		pict_attrs->mbs_recovered = mbs_recovered;
		/*
		 * We may successfully replayed the picture,
		 * so reset the error flags
		 */
		pict_attrs->pict_attrs.dwrfired = 0;
		pict_attrs->pict_attrs.mmufault = 0;
		pict_attrs->pict_attrs.deverror = 0;

		*msg_attr = VXD_MSG_ATTR_DECODED;
		*decpict = pdec_pict;
		break;
	}

	case FW_DEVA_PANIC:
	{
		unsigned int panic_info =  MEMIO_READ_FIELD(msg, FW_DEVA_PANIC_ERROR_INT);
		unsigned char panic_reason[PANIC_REASON_LEN] = "Reason(s): ";
		unsigned char is_panic_reson_identified = 0;
		/*
		 * Create panic reason string.
		 */
		if (REGIO_READ_FIELD(panic_info, PVDEC_CORE, CR_PVDEC_HOST_INTERRUPT_STATUS,
				     CR_HOST_SYS_WDT)) {
			strncat(panic_reason, apanic_reason[PANIC_REASON_WDT],
				PANIC_REASON_LEN - 1);
			is_panic_reson_identified = 1;
		}
		if (REGIO_READ_FIELD(panic_info, PVDEC_CORE, CR_PVDEC_HOST_INTERRUPT_STATUS,
				     CR_HOST_READ_TIMEOUT_PROC_IRQ)) {
			strncat(panic_reason, apanic_reason[PANIC_REASON_READ_TIMEOUT],
				PANIC_REASON_LEN - 1);
			is_panic_reson_identified = 1;
		}
		if (REGIO_READ_FIELD(panic_info, PVDEC_CORE, CR_PVDEC_HOST_INTERRUPT_STATUS,
				     CR_HOST_COMMAND_TIMEOUT_PROC_IRQ)) {
			strncat(panic_reason, apanic_reason[PANIC_REASON_CMD_TIMEOUT],
				PANIC_REASON_LEN - 1);
			is_panic_reson_identified = 1;
		}
		if (!is_panic_reson_identified) {
			strncat(panic_reason, apanic_reason[PANIC_REASON_OTHER],
				PANIC_REASON_LEN - 1);
		}
		panic_reason[strlen(panic_reason) - 2] = 0;
		if (trans_id != 0)
			pr_err("TID=0x%08X [FIRMWARE PANIC %s]\n", trans_id, panic_reason);
		else
			pr_err("TID=NULL [GENERAL FIRMWARE PANIC %s]\n", panic_reason);
		ret = IMG_ERROR_FATAL;

		break;
	}

	case FW_ASSERT:
	{
		unsigned int fwfile_namehash = MEMIO_READ_FIELD(msg, FW_ASSERT_FILE_NAME_HASH);
		unsigned int fwfile_line = MEMIO_READ_FIELD(msg, FW_ASSERT_FILE_LINE);

		pr_err("ASSERT file name hash:0x%08X line number:%d\n",
		       fwfile_namehash, fwfile_line);
		ret = IMG_ERROR_FATAL;
		break;
	}

	case FW_SO:
	{
		unsigned int task_name = MEMIO_READ_FIELD(msg, FW_SO_TASK_NAME);
		unsigned char sztaskname[sizeof(unsigned int) + 1];

		sztaskname[0] = task_name >> 24;
		sztaskname[1] = (task_name >> 16) & 0xff;
		sztaskname[2] = (task_name >> 8) & 0xff;
		sztaskname[3] = task_name & 0xff;
		if (sztaskname[3] != 0)
			sztaskname[4] = 0;
		pr_warn("STACK OVERFLOW for %s task\n", sztaskname);
		break;
	}

	case FW_VXD_EMPTY_COMPL:
		/*
		 * Empty completion message sent as response to init,
		 * configure etc The architecture of vxd.ko module
		 * requires the firmware to send a reply for every
		 * message submitted by the user space.
		 */
		break;

	default:
		break;
	}

	return ret;
}

static int vdeckm_handle_hosttomtx_msg(unsigned int *msg, struct lst_t *pend_pict_list,
				       enum vxd_msg_attr *msg_attr,
				       struct dec_decpict  **decpict,
				       unsigned char msg_type,
				       unsigned int trans_id,
				       unsigned int msg_flags)
{
	struct dec_decpict *pdec_pict;

	pr_debug("Received message from HOST\n");

	switch (msg_type) {
	case FW_DEVA_PARSE:
	{
		struct dec_pict_attrs *pict_attrs = NULL;
		unsigned char flag = 0;

		pdec_pict = lst_first(pend_pict_list);
		while (pdec_pict) {
			if (pdec_pict->transaction_id == trans_id)
				break;

			pdec_pict = lst_next(pdec_pict);
		}

		/*
		 * We must have a picture in the list that matches
		 * the transaction id
		 */
		if (!pdec_pict) {
			pr_err("Firmware decoded message received\n");
			pr_err("no pending picture\n");
			return IMG_ERROR_FATAL;
		}

		if (!(pdec_pict->first_fld_fwmsg) || !(pdec_pict->second_fld_fwmsg)) {
			pr_err("invalid pending picture struct\n");
			return IMG_ERROR_FATAL;
		}

		flag = pdec_pict->first_fld_fwmsg->pict_attrs.first_fld_rcvd;
		if (flag) {
			pict_attrs = &pdec_pict->second_fld_fwmsg->pict_attrs;
		} else {
			pict_attrs = &pdec_pict->first_fld_fwmsg->pict_attrs;
			flag = 1;
		}

		/*
		 * The below info is fetched from firmware state
		 * afterwards, so just set this to zero for now.
		 */
		pict_attrs->fe_err = 0;
		pict_attrs->no_be_wdt = 0;
		pict_attrs->mbs_dropped = 0;
		pict_attrs->mbs_recovered = 0;

		vxd_get_pictattrs(msg_flags, &pict_attrs->pict_attrs);
		vxd_get_msgerrattr(msg_flags, msg_attr);

		if (*msg_attr == VXD_MSG_ATTR_FATAL)
			pr_err("[TID=0x%08X] [DECODE_FAILED]\n", trans_id);
		if (*msg_attr == VXD_MSG_ATTR_CANCELED)
			pr_err("[TID=0x%08X] [DECODE_CANCELED]\n", trans_id);

		*decpict = pdec_pict;
		break;
	}

	case FW_DEVA_PARSE_FRAGMENT:
		/*
		 * Do nothing - Picture holds the list of fragments.
		 * So, in case of any error those would be replayed
		 * anyway.
		 */
		break;
	default:
		pr_warn("Unknown message received 0x%02x\n", msg_type);
		break;
	}

	return 0;
}

static int vdeckm_process_msg(const void *hndl_vxd, unsigned int *msg,
			      struct lst_t *pend_pict_list,
			      unsigned int msg_flags,
			      enum vxd_msg_attr *msg_attr,
			      struct dec_decpict  **decpict)
{
	struct vdeckm_context *core_ctx = (struct vdeckm_context *)hndl_vxd;
	unsigned char msg_type;
	unsigned char msg_group;
	unsigned int trans_id = 0;
	int ret = 0;
	struct vdec_pict_hwcrc *pict_hwcrc = NULL;
	struct dec_decpict *pdec_pict;

	if (!core_ctx || !msg || !msg_attr || !pend_pict_list || !decpict)
		return IMG_ERROR_INVALID_PARAMETERS;

	*msg_attr = VXD_MSG_ATTR_NONE;
	*decpict = NULL;

	trans_id = MEMIO_READ_FIELD(msg, FW_DEVA_GENMSG_TRANS_ID);
	msg_type  = MEMIO_READ_FIELD(msg, FW_DEVA_GENMSG_MSG_TYPE);
	msg_group = msg_type & MSG_GROUP_MASK;

	switch (msg_group) {
	case MSG_TYPE_START_PSR_MTXHOST_MSG:
		ret = vdeckm_handle_mtxtohost_msg(msg, pend_pict_list, msg_attr,
					    decpict, msg_type, trans_id);
		break;
	/*
	 * Picture decode has been returned as unprocessed.
	 * Locate the picture with corresponding TID and mark
	 * it as decoded with errors.
	 */
	case MSG_TYPE_START_PSR_HOSTMTX_MSG:
		ret = vdeckm_handle_hosttomtx_msg(msg, pend_pict_list, msg_attr,
					    decpict, msg_type, trans_id,
					    msg_flags);
		break;

	case FW_DEVA_SIGNATURES_HEVC:
	case FW_DEVA_SIGNATURES_LEGACY:
	{
		unsigned int *signatures = msg + (FW_DEVA_SIGNATURES_SIGNATURES_OFFSET /
				sizeof(unsigned int));
		unsigned char sigcount  = MEMIO_READ_FIELD(msg, FW_DEVA_SIGNATURES_MSG_SIZE) -
			((FW_DEVA_SIGNATURES_SIZE / sizeof(unsigned int)) - 1);
		unsigned int selected = MEMIO_READ_FIELD(msg, FW_DEVA_SIGNATURES_SIGNATURE_SELECT);
		unsigned char i, j = 0;

		pdec_pict = lst_first(pend_pict_list);
		while (pdec_pict) {
			if (pdec_pict->transaction_id == trans_id)
				break;
			pdec_pict = lst_next(pdec_pict);
		}

		/* We must have a picture in the list that matches the tid */
		VDEC_ASSERT(pdec_pict);
		if (!pdec_pict) {
			pr_err("Firmware signatures message received with no pending picture\n");
			return IMG_ERROR_FATAL;
		}

		VDEC_ASSERT(pdec_pict->first_fld_fwmsg);
		VDEC_ASSERT(pdec_pict->second_fld_fwmsg);
		if (!pdec_pict->first_fld_fwmsg || !pdec_pict->second_fld_fwmsg) {
			pr_err("Invalid pending picture struct\n");
			return IMG_ERROR_FATAL;
		}
		if (pdec_pict->first_fld_fwmsg->pict_hwcrc.first_fld_rcvd) {
			pict_hwcrc = &pdec_pict->second_fld_fwmsg->pict_hwcrc;
		} else {
			pict_hwcrc = &pdec_pict->first_fld_fwmsg->pict_hwcrc;
			if (selected & (PVDEC_SIGNATURE_GROUP_20 | PVDEC_SIGNATURE_GROUP_24))
				pdec_pict->first_fld_fwmsg->pict_hwcrc.first_fld_rcvd = TRUE;
		}

		for (i = 0; i < 32; i++) {
			unsigned int group = selected & (1 << i);

			switch (group) {
			case PVDEC_SIGNATURE_GROUP_20:
				pict_hwcrc->crc_vdmc_pix_recon = signatures[j++];
				break;

			case PVDEC_SIGNATURE_GROUP_24:
				pict_hwcrc->vdeb_sysmem_wrdata = signatures[j++];
				break;

			default:
				break;
			}
		}

		/* sanity check */
		sigcount -= j;
		VDEC_ASSERT(sigcount == 0);

		/*
		 * suppress PVDEC_SIGNATURE_GROUP_1 and notify
		 * only about groups used for verification
		 */
#ifdef DEBUG_DECODER_DRIVER
		if (selected & (PVDEC_SIGNATURE_GROUP_20 | PVDEC_SIGNATURE_GROUP_24))
			pr_info("[TID=0x%08X] [SIGNATURES]\n", trans_id);
#endif

		*decpict = pdec_pict;

		break;
	}

	default: {
#ifdef DEBUG_DECODER_DRIVER
		unsigned short msg_size, i;

		pr_warn("Unknown message type received: 0x%x", msg_type);

		msg_size = MEMIO_READ_FIELD(msg, FW_DEVA_GENMSG_MSG_SIZE);

		for (i = 0; i < msg_size; i++)
			pr_info("0x%04x: 0x%08x\n", i, msg[i]);
#endif
		break;
	}
	}

	return ret;
}

static void vdeckm_vlr_copy(void *dst, void *src, unsigned int size)
{
	unsigned int *pdst = (unsigned int *)dst;
	unsigned int *psrc = (unsigned int *)src;

	size /= 4;
	while (size--)
		*pdst++ = *psrc++;
}

static int vdeckm_get_core_state(const void *hndl_vxd, struct vxd_states *state)
{
	struct vdeckm_context *core_ctx = (struct vdeckm_context *)hndl_vxd;
	struct vdecfw_pvdecfirmwarestate firmware_state;
	unsigned char pipe = 0;

#ifdef ERROR_RECOVERY_SIMULATION
	/*
	 * if disable_fw_irq_value is not zero, return error. If processed further
	 * the kernel will crash because we have ignored the interrupt, but here
	 * we will try to access comms_ram_addr which will result in crash.
	 */
	if (disable_fw_irq_value != 0)
		return IMG_ERROR_INVALID_PARAMETERS;
#endif

	if (!core_ctx || !state)
		return IMG_ERROR_INVALID_PARAMETERS;

	/*
	 * If state is requested for the first time.
	 */
	if (core_ctx->state_size == 0) {
		unsigned int regval;
		/*
		 * get the state buffer info.
		 */
		regval = *((unsigned int *)core_ctx->comms_ram_addr +
			(PVDEC_COM_RAM_STATE_BUF_SIZE_AND_OFFSET_OFFSET / sizeof(unsigned int)));
		core_ctx->state_size = PVDEC_COM_RAM_BUF_GET_SIZE(regval, STATE);
		core_ctx->state_offset = PVDEC_COM_RAM_BUF_GET_OFFSET(regval, STATE);
	}

	/*
	 * If state buffer is available.
	 */
	if (core_ctx->state_size) {
		/*
		 * Determine the latest transaction to have passed each
		 * checkpoint in the firmware.
		 * Read the firmware state from VEC Local RAM
		 */
		vdeckm_vlr_copy(&firmware_state, (unsigned char *)core_ctx->comms_ram_addr +
				core_ctx->state_offset, core_ctx->state_size);

		for (pipe = 0; pipe < core_ctx->props.num_pixel_pipes; pipe++) {
			/*
			 * Set pipe presence.
			 */
			state->fw_state.pipe_state[pipe].is_pipe_present = 1;

			/*
			 * For checkpoints copy message ids here. These will
			 * be translated into transaction ids later.
			 */
			memcpy(state->fw_state.pipe_state[pipe].acheck_point,
			       firmware_state.pipestate[pipe].check_point,
				sizeof(state->fw_state.pipe_state[pipe].acheck_point));
			state->fw_state.pipe_state[pipe].firmware_action  =
				firmware_state.pipestate[pipe].firmware_action;
			state->fw_state.pipe_state[pipe].cur_codec =
				firmware_state.pipestate[pipe].curr_codec;
			state->fw_state.pipe_state[pipe].fe_slices =
				firmware_state.pipestate[pipe].fe_slices;
			state->fw_state.pipe_state[pipe].be_slices =
				firmware_state.pipestate[pipe].be_slices;
			state->fw_state.pipe_state[pipe].fe_errored_slices =
				firmware_state.pipestate[pipe].fe_errored_slices;
			state->fw_state.pipe_state[pipe].be_errored_slices =
				firmware_state.pipestate[pipe].be_errored_slices;
			state->fw_state.pipe_state[pipe].be_mbs_dropped =
				firmware_state.pipestate[pipe].be_mbs_dropped;
			state->fw_state.pipe_state[pipe].be_mbs_recovered =
				firmware_state.pipestate[pipe].be_mbs_recovered;
			state->fw_state.pipe_state[pipe].fe_mb.x =
				firmware_state.pipestate[pipe].last_fe_mb_xy & 0xFF;
			state->fw_state.pipe_state[pipe].fe_mb.y =
				(firmware_state.pipestate[pipe].last_fe_mb_xy >> 16) & 0xFF;
			state->fw_state.pipe_state[pipe].be_mb.x =
				REGIO_READ_FIELD(firmware_state.pipestate[pipe].last_be_mb_xy,
						 MSVDX_VDMC,
						 CR_VDMC_MACROBLOCK_NUMBER,
						 CR_VDMC_MACROBLOCK_X_OFFSET);
			state->fw_state.pipe_state[pipe].be_mb.y =
				REGIO_READ_FIELD(firmware_state.pipestate[pipe].last_be_mb_xy,
						 MSVDX_VDMC,
						 CR_VDMC_MACROBLOCK_NUMBER,
						 CR_VDMC_MACROBLOCK_Y_OFFSET);
		}
	}

	return 0;
}

static int vdeckm_prepare_batch(struct vdeckm_context *core_ctx,
				const struct hwctrl_batch_msgdata *batch_msgdata,
				unsigned char **msg)
{
	unsigned char vdec_flags = 0;
	unsigned short flags = 0;
	unsigned char *pmsg = kzalloc(FW_DEVA_DECODE_SIZE, GFP_KERNEL);
	struct vidio_ddbufinfo *pbatch_msg_bufinfo = batch_msgdata->batchmsg_bufinfo;

	if (!pmsg)
		return IMG_ERROR_MALLOC_FAILED;

	if (batch_msgdata->size_delimited_mode)
		vdec_flags |= FW_VDEC_NAL_SIZE_DELIM;

	flags |= FW_DEVA_RENDER_HOST_INT;

	/*
	 * Message type and stream ID
	 */
	MEMIO_WRITE_FIELD(pmsg, FW_DEVA_GENMSG_MSG_TYPE, FW_DEVA_PARSE, unsigned char*);

	MEMIO_WRITE_FIELD(pmsg, FW_DEVA_DECODE_CTRL_ALLOC_ADDR,
			  (unsigned int)pbatch_msg_bufinfo->dev_virt, unsigned char*);

	MEMIO_WRITE_FIELD(pmsg, FW_DEVA_DECODE_BUFFER_SIZE,
			  batch_msgdata->ctrl_alloc_bytes / sizeof(unsigned int), unsigned char*);

	/*
	 * Operating mode and decode flags
	 */
	MEMIO_WRITE_FIELD(pmsg, FW_DEVA_DECODE_OPERATING_MODE, batch_msgdata->operating_mode,
			  unsigned char*);

	MEMIO_WRITE_FIELD(pmsg, FW_DEVA_DECODE_FLAGS, flags, unsigned char*);

	MEMIO_WRITE_FIELD(pmsg, FW_DEVA_DECODE_VDEC_FLAGS, vdec_flags, unsigned char*);

	MEMIO_WRITE_FIELD(pmsg, FW_DEVA_DECODE_GENC_ID, batch_msgdata->genc_id, unsigned char*);

	MEMIO_WRITE_FIELD(pmsg, FW_DEVA_DECODE_MB_LOAD, batch_msgdata->mb_load, unsigned char*);

	MEMIO_WRITE_FIELD(pmsg, FW_DEVA_DECODE_STREAMID,
			  GET_STREAM_ID(batch_msgdata->transaction_id), unsigned char*);

	MEMIO_WRITE_FIELD(pmsg, FW_DEVA_DECODE_EXT_STATE_BUFFER,
			  (unsigned int)batch_msgdata->pvdec_fwctx->dev_virt, unsigned char*);

	MEMIO_WRITE_FIELD(pmsg, FW_DEVA_DECODE_MSG_ID, ++core_ctx->current_msgid,
			  unsigned char*);

	MEMIO_WRITE_FIELD(pmsg, FW_DEVA_DECODE_TRANS_ID, batch_msgdata->transaction_id,
			  unsigned char*);

	MEMIO_WRITE_FIELD(pmsg, FW_DEVA_DECODE_TILE_CFG, batch_msgdata->tile_cfg, unsigned char*);

	/*
	 * size of message
	 */
	MEMIO_WRITE_FIELD(pmsg, FW_DEVA_GENMSG_MSG_SIZE,
			  FW_DEVA_DECODE_SIZE / sizeof(unsigned int), unsigned char*);

	*msg = pmsg;

	return 0;
}

static int vdeckm_prepare_fragment(struct vdeckm_context *core_ctx,
				   const struct hwctrl_fragment_msgdata
				   *fragment_msgdata,
				   unsigned char **msg)
{
	struct vidio_ddbufinfo *pbatch_msg_bufinfo = NULL;
	unsigned char *pmsg = NULL;

	pbatch_msg_bufinfo = fragment_msgdata->batchmsg_bufinfo;

	if (!(fragment_msgdata->batchmsg_bufinfo)) {
		pr_err("Batch message info missing!\n");
		return IMG_ERROR_INVALID_PARAMETERS;
	}

	pmsg = kzalloc(FW_DEVA_DECODE_FRAGMENT_SIZE, GFP_KERNEL);
	if (!pmsg)
		return IMG_ERROR_MALLOC_FAILED;
	/*
	 * message type and stream id
	 */
	MEMIO_WRITE_FIELD(pmsg, FW_DEVA_GENMSG_MSG_TYPE,
			  FW_DEVA_PARSE_FRAGMENT, unsigned char*);
	MEMIO_WRITE_FIELD(pmsg, FW_DEVA_DECODE_MSG_ID, ++core_ctx->current_msgid, unsigned char*);

	MEMIO_WRITE_FIELD(pmsg, FW_DEVA_DECODE_FRAGMENT_CTRL_ALLOC_ADDR,
			  (unsigned int)pbatch_msg_bufinfo->dev_virt
			  + fragment_msgdata->ctrl_alloc_offset, unsigned char*);
	MEMIO_WRITE_FIELD(pmsg, FW_DEVA_DECODE_FRAGMENT_BUFFER_SIZE,
			  fragment_msgdata->ctrl_alloc_bytes / sizeof(unsigned int),
			  unsigned char*);

	/*
	 * size of message
	 */
	MEMIO_WRITE_FIELD(pmsg, FW_DEVA_GENMSG_MSG_SIZE,
			  FW_DEVA_DECODE_FRAGMENT_SIZE / sizeof(unsigned int), unsigned char*);

	*msg = pmsg;

	return 0;
}

static int vdeckm_get_message(const void *hndl_vxd, const enum hwctrl_msgid msgid,
			      const struct hwctrl_msgdata *msgdata,
			      struct hwctrl_to_kernel_msg *to_kernelmsg)
{
	unsigned int result = 0;
	struct vdeckm_context *core_ctx = (struct vdeckm_context *)hndl_vxd;

	if (!core_ctx || !to_kernelmsg || !msgdata)
		return IMG_ERROR_INVALID_PARAMETERS;

	switch (msgid) {
	case HWCTRL_MSGID_BATCH:
		result = vdeckm_prepare_batch(core_ctx, &msgdata->batch_msgdata,
					      &to_kernelmsg->msg_hdr);
		break;

	case HWCTRL_MSGID_FRAGMENT:
		result = vdeckm_prepare_fragment(core_ctx, &msgdata->fragment_msgdata,
						 &to_kernelmsg->msg_hdr);
		vxd_set_msgflag(VXD_MSG_FLAG_DROP, &to_kernelmsg->flags);
		break;

	default:
		result = IMG_ERROR_GENERIC_FAILURE;
		pr_err("got a message that is not supported by PVDEC");
		break;
	}

	if (result == 0) {
		/* Set the stream ID for the next message to be sent. */
		to_kernelmsg->km_str_id = msgdata->km_str_id;
		to_kernelmsg->msg_size = MEMIO_READ_FIELD(to_kernelmsg->msg_hdr,
							  FW_DEVA_GENMSG_MSG_SIZE) *
							  sizeof(unsigned int);
	}

	return result;
}

static void hwctrl_dump_state(struct vxd_states *prev_state,
			      struct vxd_states *cur_state,
			      unsigned char pipe_minus1)
{
	pr_info("Back-End MbX                          [% 10d]",
		prev_state->fw_state.pipe_state[pipe_minus1].be_mb.x);
	pr_info("Back-End MbY                          [% 10d]",
		prev_state->fw_state.pipe_state[pipe_minus1].be_mb.y);
	pr_info("Front-End MbX                         [% 10d]",
		prev_state->fw_state.pipe_state[pipe_minus1].fe_mb.x);
	pr_info("Front-End MbY                         [% 10d]",
		prev_state->fw_state.pipe_state[pipe_minus1].fe_mb.y);
	pr_info("VDECFW_CHECKPOINT_BE_PICTURE_COMPLETE [0x%08X]",
		cur_state->fw_state.pipe_state[pipe_minus1].acheck_point
		[VDECFW_CHECKPOINT_BE_PICTURE_COMPLETE]);
	pr_info("VDECFW_CHECKPOINT_BE_1SLICE_DONE      [0x%08X]",
		cur_state->fw_state.pipe_state[pipe_minus1].acheck_point
		[VDECFW_CHECKPOINT_BE_1SLICE_DONE]);
	pr_info("VDECFW_CHECKPOINT_BE_PICTURE_STARTED  [0x%08X]",
		cur_state->fw_state.pipe_state[pipe_minus1].acheck_point
		[VDECFW_CHECKPOINT_BE_PICTURE_STARTED]);
	pr_info("VDECFW_CHECKPOINT_FE_PICTURE_COMPLETE [0x%08X]",
		cur_state->fw_state.pipe_state[pipe_minus1].acheck_point
		[VDECFW_CHECKPOINT_FE_PICTURE_COMPLETE]);
	pr_info("VDECFW_CHECKPOINT_FE_PARSE_DONE       [0x%08X]",
		cur_state->fw_state.pipe_state[pipe_minus1].acheck_point
		[VDECFW_CHECKPOINT_FE_PARSE_DONE]);
	pr_info("VDECFW_CHECKPOINT_FE_1SLICE_DONE      [0x%08X]",
		cur_state->fw_state.pipe_state[pipe_minus1].acheck_point
		[VDECFW_CHECKPOINT_FE_1SLICE_DONE]);
	pr_info("VDECFW_CHECKPOINT_ENTDEC_STARTED      [0x%08X]",
		cur_state->fw_state.pipe_state[pipe_minus1].acheck_point
		[VDECFW_CHECKPOINT_ENTDEC_STARTED]);
	pr_info("VDECFW_CHECKPOINT_FIRMWARE_SAVED      [0x%08X]",
		cur_state->fw_state.pipe_state[pipe_minus1].acheck_point
		[VDECFW_CHECKPOINT_FIRMWARE_SAVED]);
	pr_info("VDECFW_CHECKPOINT_PICMAN_COMPLETE     [0x%08X]",
		cur_state->fw_state.pipe_state[pipe_minus1].acheck_point
		[VDECFW_CHECKPOINT_PICMAN_COMPLETE]);
	pr_info("VDECFW_CHECKPOINT_FIRMWARE_READY      [0x%08X]",
		cur_state->fw_state.pipe_state[pipe_minus1].acheck_point
		[VDECFW_CHECKPOINT_FIRMWARE_READY]);
	pr_info("VDECFW_CHECKPOINT_PICTURE_STARTED     [0x%08X]",
		cur_state->fw_state.pipe_state[pipe_minus1].acheck_point
		[VDECFW_CHECKPOINT_PICTURE_STARTED]);
}

static unsigned int hwctrl_calculate_load(struct bspp_pict_hdr_info *pict_hdr_info)
{
	return (((pict_hdr_info->coded_frame_size.width + 15) / 16)
	       * ((pict_hdr_info->coded_frame_size.height + 15) / 16));
}

static int hwctrl_send_batch_message(struct hwctrl_ctx *hwctx,
				     struct dec_decpict *decpict,
				     void *vxd_dec_ctx)
{
	int result;
	struct hwctrl_to_kernel_msg to_kernelmsg = {0};
	struct vidio_ddbufinfo *batchmsg_bufinfo =
		decpict->batch_msginfo->ddbuf_info;
	struct hwctrl_msgdata msg_data;
	struct hwctrl_batch_msgdata *batch_msgdata = &msg_data.batch_msgdata;

	memset(&msg_data, 0, sizeof(msg_data));

	msg_data.km_str_id = GET_STREAM_ID(decpict->transaction_id);

	batch_msgdata->batchmsg_bufinfo  = batchmsg_bufinfo;

	batch_msgdata->transaction_id    = decpict->transaction_id;
	batch_msgdata->pvdec_fwctx       = decpict->str_pvdec_fw_ctxbuf;
	batch_msgdata->ctrl_alloc_bytes  = decpict->ctrl_alloc_bytes;
	batch_msgdata->operating_mode    = decpict->operating_op;
	batch_msgdata->genc_id           = decpict->genc_id;
	batch_msgdata->mb_load           = hwctrl_calculate_load(decpict->pict_hdr_info);
	batch_msgdata->size_delimited_mode =
		(decpict->pict_hdr_info->parser_mode != VDECFW_SCP_ONLY) ?
		(1) : (0);

	result = vdeckm_get_message(hwctx->hndl_vxd, HWCTRL_MSGID_BATCH,
				    &msg_data, &to_kernelmsg);
	if (result != 0) {
		pr_err("failed to get decode message\n");
		return result;
	}

	pr_debug("[HWCTRL] send batch message\n");
	result = vdeckm_send_message(hwctx->hndl_vxd, &to_kernelmsg,
				     vxd_dec_ctx);
	if (result != 0)
		return result;

	vdeckm_return_msg(hwctx->hndl_vxd, &to_kernelmsg);

	return 0;
}

int hwctrl_process_msg(void *hndl_hwctx, unsigned int msg_flags, unsigned int *msg,
		       struct dec_decpict **decpict)
{
	int result;
	struct hwctrl_ctx *hwctx;
	enum vxd_msg_attr msg_attr = VXD_MSG_ATTR_NONE;
	struct dec_decpict *pdecpict = NULL;
	unsigned int val_first = 0;
	unsigned int val_sec = 0;

	if (!hndl_hwctx || !msg || !decpict) {
		VDEC_ASSERT(0);
		return IMG_ERROR_INVALID_PARAMETERS;
	}

	hwctx = (struct hwctrl_ctx *)hndl_hwctx;

	*decpict = NULL;

	pr_debug("[HWCTRL] : process message\n");
	result = vdeckm_process_msg(hwctx->hndl_vxd, msg, &hwctx->pend_pict_list, msg_flags,
				    &msg_attr, &pdecpict);
	if (result != IMG_SUCCESS)
		return result;

	/* validate pointers before using them */
	if (!pdecpict || !pdecpict->first_fld_fwmsg || !pdecpict->second_fld_fwmsg) {
		VDEC_ASSERT(0);
		return -EIO;
	}

	val_first = pdecpict->first_fld_fwmsg->pict_attrs.pict_attrs.deverror;
	val_sec = pdecpict->second_fld_fwmsg->pict_attrs.pict_attrs.deverror;

	if (val_first || val_sec)
		pr_err("device signaled critical error!!!\n");

	if (msg_attr == VXD_MSG_ATTR_DECODED) {
		pdecpict->state = DECODER_PICTURE_STATE_DECODED;
		/*
		 * We have successfully decoded a picture as normally or
		 * after the replay.
		 * Mark HW is in good state.
		 */
		hwctx->is_fatal_state = 0;
	} else if (msg_attr == VXD_MSG_ATTR_FATAL) {
		struct hwctrl_state state;
		unsigned char pipe_minus1 = 0;

		memset(&state, 0, sizeof(state));

		result = hwctrl_get_core_status(hwctx, &state);
		if (result == 0) {
			hwctx->is_prev_hw_state_set = 1;
			memcpy(&hwctx->prev_state, &state, sizeof(struct hwctrl_state));

			for (pipe_minus1 = 0; pipe_minus1 < hwctx->num_pipes;
				pipe_minus1++) {
#ifdef DEBUG_DECODER_DRIVER
				hwctrl_dump_state(&state.core_state, &state.core_state,
						  pipe_minus1);
#endif
			}
		}
		pdecpict->state = DECODER_PICTURE_STATE_TO_DISCARD;
	}
	*decpict = pdecpict;

	return 0;
}

int hwctrl_getcore_cached_status(void *hndl_hwctx, struct hwctrl_state *state)
{
	struct hwctrl_ctx *hwctx = (struct hwctrl_ctx *)hndl_hwctx;

	if (hwctx->is_prev_hw_state_set)
		memcpy(state, &hwctx->prev_state, sizeof(struct hwctrl_state));
	else
		return IMG_ERROR_UNEXPECTED_STATE;

	return 0;
}

int hwctrl_get_core_status(void *hndl_hwctx, struct hwctrl_state *state)
{
	struct hwctrl_ctx *hwctx = (struct hwctrl_ctx *)hndl_hwctx;
	unsigned int result = IMG_ERROR_GENERIC_FAILURE;

	if (!hwctx->is_fatal_state && state) {
		struct vxd_states *pcorestate = NULL;

		pcorestate  = &state->core_state;

		memset(pcorestate, 0, sizeof(*(pcorestate)));

		result = vdeckm_get_core_state(hwctx->hndl_vxd, pcorestate);
	}

	return result;
}

int hwctrl_is_on_seq_replay(void *hndl_hwctx)
{
	struct hwctrl_ctx *hwctx = (struct hwctrl_ctx *)hndl_hwctx;

	return hwctx->is_on_seq_replay;
}

int hwctrl_picture_submitbatch(void *hndl_hwctx, struct dec_decpict  *decpict, void *vxd_dec_ctx)
{
	struct hwctrl_ctx *hwctx = (struct hwctrl_ctx *)hndl_hwctx;

	if (hwctx->is_initialised) {
		lst_add(&hwctx->pend_pict_list, decpict);
		if (!hwctx->is_on_seq_replay)
			return hwctrl_send_batch_message(hwctx, decpict, vxd_dec_ctx);
	}

	return 0;
}

int hwctrl_getpicpend_pictlist(void *hndl_hwctx, unsigned int transaction_id,
			       struct dec_decpict  **decpict)
{
	struct hwctrl_ctx *hwctx = (struct hwctrl_ctx *)hndl_hwctx;
	struct dec_decpict  *dec_pic;

	dec_pic = lst_first(&hwctx->pend_pict_list);
	while (dec_pic) {
		if (dec_pic->transaction_id == transaction_id) {
			*decpict = dec_pic;
			break;
		}
		dec_pic = lst_next(dec_pic);
	}

	if (!dec_pic)
		return IMG_ERROR_INVALID_ID;

	return 0;
}

int hwctrl_peekheadpiclist(void *hndl_hwctx, struct dec_decpict **decpict)
{
	struct hwctrl_ctx *hwctx = (struct hwctrl_ctx *)hndl_hwctx;

	if (hwctx)
		*decpict = lst_first(&hwctx->pend_pict_list);

	if (*decpict)
		return 0;

	return IMG_ERROR_GENERIC_FAILURE;
}

int hwctrl_getdecodedpicture(void *hndl_hwctx, struct dec_decpict **decpict)
{
	struct hwctrl_ctx *hwctx = (struct hwctrl_ctx *)hndl_hwctx;

	if (hwctx) {
		struct dec_decpict *cur_decpict;
		/*
		 * Ensure that this picture is in the list.
		 */
		cur_decpict = lst_first(&hwctx->pend_pict_list);
		while (cur_decpict) {
			if (cur_decpict->state == DECODER_PICTURE_STATE_DECODED) {
				*decpict = cur_decpict;
				return 0;
			}

			cur_decpict = lst_next(cur_decpict);
		}
	}

	return IMG_ERROR_VALUE_OUT_OF_RANGE;
}

void hwctrl_removefrom_piclist(void *hndl_hwctx, struct dec_decpict  *decpict)
{
	struct hwctrl_ctx *hwctx = (struct hwctrl_ctx *)hndl_hwctx;

	if (hwctx) {
		struct dec_decpict *cur_decpict;
		/*
		 * Ensure that this picture is in the list.
		 */
		cur_decpict = lst_first(&hwctx->pend_pict_list);
		while (cur_decpict) {
			if (cur_decpict == decpict) {
				lst_remove(&hwctx->pend_pict_list, decpict);
				break;
			}

			cur_decpict = lst_next(cur_decpict);
		}
	}
}

int hwctrl_getregsoffset(void *hndl_hwctx, struct decoder_regsoffsets *regs_offsets)
{
	struct hwctrl_ctx *hwctx = (struct hwctrl_ctx *)hndl_hwctx;

	return vdeckm_getregsoffsets(hwctx->hndl_vxd, regs_offsets);
}

static int pvdec_create(struct vxd_dev *vxd, struct vxd_coreprops *core_props,
			void **hndl_vdeckm_context)
{
	struct vdeckm_context  *corectx;
	struct vxd_core_props hndl_core_props;
	int result;
	int iMapSize, pageSize;
	void *phy_addr;

	if (!hndl_vdeckm_context || !core_props)
		return IMG_ERROR_INVALID_PARAMETERS;

	/*
	 * Obtain core context.
	 */
	corectx = &acore_ctx[0];

	memset(corectx, 0, sizeof(*corectx));

	corectx->core_num = 0;

	result = vxd_pvdec_get_props(vxd->dev, vxd->reg_base, &hndl_core_props);
	if (result != 0)
		return result;

	vxd_get_coreproperties(&hndl_core_props, &corectx->props);

	memcpy(core_props, &corectx->props, sizeof(*core_props));

	pageSize = PAGE_SIZE;
	/* end aligned to page (ceiling), in pages */
	iMapSize = (PVDEC_COMMS_RAM_OFFSET + PVDEC_COMMS_RAM_SIZE + pageSize - 1) / pageSize;
	/* subtract start aligned to page (floor), in pages */
	iMapSize -= PVDEC_COMMS_RAM_OFFSET / pageSize;
	/* convert to bytes */
	iMapSize *= pageSize;
	phy_addr = (void *)(0x4300000);
	phy_addr += (PVDEC_COMMS_RAM_OFFSET);
	corectx->comms_ram_addr = ioremap((phys_addr_t)phy_addr, iMapSize);
	*hndl_vdeckm_context = corectx;

	return 0;
}

int hwctrl_deinitialise(void *hndl_hwctx)
{
	struct hwctrl_ctx *hwctx = (struct hwctrl_ctx *)hndl_hwctx;

	if (hwctx->is_initialised) {
		kfree(hwctx);
		hwctx = NULL;
	}

	return 0;
}

int hwctrl_initialise(void *dec_core, void *comp_int_userdata,
		      const struct vdecdd_dd_devconfig  *dd_devconfig,
		      struct vxd_coreprops *core_props, void **hndl_hwctx)
{
	struct hwctrl_ctx *hwctx = (struct hwctrl_ctx *)*hndl_hwctx;
	int result;

	if (!hwctx) {
		hwctx = kzalloc(sizeof(*(hwctx)), GFP_KERNEL);
		if (!hwctx)
			return IMG_ERROR_OUT_OF_MEMORY;

		*hndl_hwctx = hwctx;
	}

	if (!hwctx->is_initialised) {
		hwctx->hndl_vxd = ((struct dec_core_ctx *)dec_core)->dec_ctx->dev_handle;
		result = pvdec_create(hwctx->hndl_vxd, core_props, &hwctx->hndl_vxd);
		if (result != 0)
			goto error;

		lst_init(&hwctx->pend_pict_list);

		hwctx->devconfig = *dd_devconfig;
		hwctx->num_pipes = core_props->num_pixel_pipes;
		hwctx->comp_init_userdata = comp_int_userdata;
		hwctx->dec_core = dec_core;
		hwctx->is_initialised = 1;
		hwctx->is_on_seq_replay = 0;
		hwctx->is_fatal_state = 0;
	}

	return 0;
error:
	hwctrl_deinitialise(*hndl_hwctx);

	return result;
}

static int hwctrl_send_fragment_message(struct hwctrl_ctx *hwctx,
					struct dec_pict_fragment *pict_fragment,
					struct dec_decpict *decpict,
					void *vxd_dec_ctx)
{
	int result;
	struct hwctrl_to_kernel_msg to_kernelmsg = {0};
	struct hwctrl_msgdata msg_data;
	struct hwctrl_fragment_msgdata *pfragment_msgdata =
		&msg_data.fragment_msgdata;

	msg_data.km_str_id = GET_STREAM_ID(decpict->transaction_id);

	pfragment_msgdata->ctrl_alloc_bytes = pict_fragment->ctrl_alloc_bytes;

	pfragment_msgdata->ctrl_alloc_offset = pict_fragment->ctrl_alloc_offset;

	pfragment_msgdata->batchmsg_bufinfo = decpict->batch_msginfo->ddbuf_info;

	result = vdeckm_get_message(hwctx->hndl_vxd, HWCTRL_MSGID_FRAGMENT, &msg_data,
				    &to_kernelmsg);
	if (result != 0) {
		pr_err("Failed to get decode message\n");
		return result;
	}

	result = vdeckm_send_message(hwctx->hndl_vxd, &to_kernelmsg, vxd_dec_ctx);
	if (result != 0)
		return result;

	vdeckm_return_msg(hwctx->hndl_vxd, &to_kernelmsg);

	return 0;
}

int hwctrl_picture_submit_fragment(void *hndl_hwctx,
				   struct dec_pict_fragment  *pict_fragment,
				   struct dec_decpict *decpict,
				   void *vxd_dec_ctx)
{
	struct hwctrl_ctx *hwctx = (struct hwctrl_ctx *)hndl_hwctx;
	unsigned int result = 0;

	if (hwctx->is_initialised) {
		result = hwctrl_send_fragment_message(hwctx, pict_fragment,
						      decpict, vxd_dec_ctx);
		if (result != 0)
			pr_err("Failed to send fragment message to firmware !");
	}

	return result;
}
