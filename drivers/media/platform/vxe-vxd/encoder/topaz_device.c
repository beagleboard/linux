// SPDX-License-Identifier: GPL-2.0
/*
 * Encoder device function implementations
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

#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/printk.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/dma-mapping.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>
#include <linux/delay.h>
#include <linux/time64.h>
#include <linux/jiffies.h>

#include "work_queue.h"
#include "fw_headers/defs.h"
#include "fw_headers/vxe_common.h"
#include "target.h"
#include "target_config.h"
#include "topaz_device.h"
#include "topazmmu.h"
#include "vid_buf.h"
#include "vxe_public_regdefs.h"
#include "img_errors.h"

#ifdef DEBUG_ENCODER_DRIVER
static char command_string[][38] = {
	"MTX_CMDID_NULL",
	"MTX_CMDID_SHUTDOWN",
	"MTX_CMDID_DO_HEADER",
	"MTX_CMDID_ENCODE_FRAME",
	"MTX_CMDID_START_FRAME",
	"MTX_CMDID_ENCODE_SLICE",
	"MTX_CMDID_END_FRAME",
	"MTX_CMDID_SETVIDEO",
	"MTX_CMDID_GETVIDEO",
	"MTX_CMDID_DO_CHANGE_PIPEWORK",
#if SECURE_IO_PORTS
	"MTX_CMDID_SECUREIO",
#endif
	"MTX_CMDID_PICMGMT",
	"MTX_CMDID_RC_UPDATE",
	"MTX_CMDID_PROVIDE_SOURCE_BUFFER",
	"MTX_CMDID_PROVIDE_REF_BUFFER",
	"MTX_CMDID_PROVIDE_CODEDPACKAGE_BUFFER",
	"MTX_CMDID_ABORT",
	"MTX_CMDID_SETQUANT",
	"MTX_CMDID_SETUP_INTERFACE",
	"MTX_CMDID_ISSUEBUFF",
	"MTX_CMDID_SETUP",
	"MTX_CMDID_UPDATE_SOURCE_FORMAT",
	"MTX_CMDID_UPDATE_CSC",
	"MTX_CMDID_ENDMARKER"
};
#endif

DECLARE_WAIT_QUEUE_HEAD(event_wait_queue);

#define TOPAZ_DEV_SPIN_LOCK_NAME  "topaz_dev"
/* max syncStatus value used (at least 4 * MAX_TOPAZ_CMDS_QUEUED) */
#define MAX_TOPAZ_CMD_COUNT       (0x1000)

#define COMM_WB_DATA_BUF_SIZE     (64)

/* Sempahore locks */
#define COMM_LOCK_TX          0x01
#define COMM_LOCK_RX          0x02
#define COMM_LOCK_BOTH        (COMM_LOCK_TX | COMM_LOCK_RX)

static unsigned int topaz_timeout_retries = 817000;

#define TOPAZ_TIMEOUT_JPEG    (50000)
#define TOPAZ_TIMEOUT_RETRIES (topaz_timeout_retries)

unsigned short g_load_method = MTX_LOADMETHOD_DMA; /* This is the load method used */

unsigned int g_core_rev;
unsigned int g_core_des1;
void *g_lock;

struct vidio_ddbufinfo *g_aps_wb_data_info;

static unsigned char g_pipe_usage[TOPAZHP_MAX_NUM_PIPES] = { 0 };

/* Order MUST match with topaz_mem_space_idx enum */
struct mem_space topaz_mem_space[] = {
	/* Multicore sync RAM */
	{ "REG_TOPAZHP_MULTICORE",        MEMSPACE_REGISTER,
	  {{0x00000000, 0x000003ff, TARGET_NO_IRQ}}},
	{ "REG_DMAC",                     MEMSPACE_REGISTER,
	  {{0x00000400, 0x000000ff, TARGET_NO_IRQ}}},
	{ "REG_COMMS",                    MEMSPACE_REGISTER,
	  {{0x00000500, 0x000000ff, TARGET_NO_IRQ}}},
	{ "REG_MTX",                      MEMSPACE_REGISTER,
	  {{0x00000800, 0x000003ff, TARGET_NO_IRQ}}},
	{ "REG_MMU",                      MEMSPACE_REGISTER,
	  {{0x00000C00, 0x000003ff, TARGET_NO_IRQ}}},
	{ "REG_TOPAZHP_TEST",             MEMSPACE_REGISTER,
	  {{0xFFFF0000, 0x000001ff, TARGET_NO_IRQ}}},
	{ "REGMTXRAM",                    MEMSPACE_REGISTER,
	  {{0x80000000, 0x0000ffff, TARGET_NO_IRQ}}},
	{ "REG_TOPAZHP_CORE_0",           MEMSPACE_REGISTER,
	  {{0x00001000, 0x000003ff, TARGET_NO_IRQ}}},
	{ "REG_TOPAZHP_VLC_CORE_0",       MEMSPACE_REGISTER,
	  {{0x00001400, 0x000003ff, TARGET_NO_IRQ}}},
	{ "REG_TOPAZHP_DEBLOCKER_CORE_0", MEMSPACE_REGISTER,
	  {{0x00001800, 0x000003ff, TARGET_NO_IRQ}}},
	{ "REG_TOPAZHP_COREEXT_0",        MEMSPACE_REGISTER,
	  {{0x00001C00, 0x000003ff, TARGET_NO_IRQ}}},

	{ "REG_TOPAZHP_CORE_1",           MEMSPACE_REGISTER,
	  {{0x00002000, 0x000003ff, TARGET_NO_IRQ}}},
	{ "REG_TOPAZHP_VLC_CORE_1",       MEMSPACE_REGISTER,
	  {{0x00002400, 0x000003ff, TARGET_NO_IRQ}}},
	{ "REG_TOPAZHP_DEBLOCKER_CORE_1", MEMSPACE_REGISTER,
	  {{0x00002800, 0x000003ff, TARGET_NO_IRQ}}},
	{ "REG_TOPAZHP_COREEXT_1",        MEMSPACE_REGISTER,
	  {{0x00002C00, 0x000003ff, TARGET_NO_IRQ}}},

	{ "REG_TOPAZHP_CORE_2",           MEMSPACE_REGISTER,
	  {{0x00003000, 0x000003ff, TARGET_NO_IRQ}}},
	{ "REG_TOPAZHP_VLC_CORE_2",       MEMSPACE_REGISTER,
	  {{0x00003400, 0x000003ff, TARGET_NO_IRQ}}},
	{ "REG_TOPAZHP_DEBLOCKER_CORE_2", MEMSPACE_REGISTER,
	  {{0x00003800, 0x000003ff, TARGET_NO_IRQ}}},
	{ "REG_TOPAZHP_COREEXT_2",        MEMSPACE_REGISTER,
	  {{0x00003C00, 0x000003ff, TARGET_NO_IRQ}}},

	{ "REG_TOPAZHP_CORE_3",           MEMSPACE_REGISTER,
	  {{0x00004000, 0x000003ff, TARGET_NO_IRQ}}},
	{ "REG_TOPAZHP_VLC_CORE_3",       MEMSPACE_REGISTER,
	  {{0x00004400, 0x000003ff, TARGET_NO_IRQ}}},
	{ "REG_TOPAZHP_DEBLOCKER_CORE_3", MEMSPACE_REGISTER,
	  {{0x00004800, 0x000003ff, TARGET_NO_IRQ}}},
	{ "REG_TOPAZHP_COREEXT_3",        MEMSPACE_REGISTER,
	  {{0x00004C00, 0x000003ff, TARGET_NO_IRQ}}},

	{ "FW",                           MEMSPACE_MEMORY,
	  {{0x00000000, 0x00800000, 0            }}},
	{ "SYSMEM",                       MEMSPACE_MEMORY,
	  {{0x00000000, 0,          0            }}},
	{ "MEMSYSMEM",                    MEMSPACE_MEMORY,
	  {{0x00000000, 0,          0            }}},
	{ "MEM",                          MEMSPACE_MEMORY,
	  {{0x00000000, 0,          0            }}},
	{ "FB",                           MEMSPACE_MEMORY,
	  {{0x00000000, 0,          0            }}},
	{ "MEMDMAC_00",                   MEMSPACE_MEMORY,
	  {{0x00000000, 0,          0            }}},
	{ "MEMDMAC_01",                   MEMSPACE_MEMORY,
	  {{0x00000000, 0,          0            }}},
	{ "MEMDMAC_02",                   MEMSPACE_MEMORY,
	  {{0x00000000, 0,          0            }}},
};

#define MEMORYSPACES_NUM (sizeof(topaz_mem_space) / sizeof(struct mem_space))

static struct target_config topaz_target_config = {
	MEMORYSPACES_NUM,
	&topaz_mem_space[0]
};

/*
 * topazdd_int_enable
 */
static void topazdd_int_enable(struct topaz_dev_ctx *ctx, unsigned int mask)
{
	unsigned int reg;
	unsigned long flags;

	spin_lock_irqsave(ctx->lock, flags);

	/* config interrupts on Topaz core */
	reg = VXE_RD_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_MULTICORE_HOST_INT_ENAB);

	/* set enable interrupt bits */
	reg |= mask;
	VXE_WR_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_MULTICORE_HOST_INT_ENAB, reg);

	spin_unlock_irqrestore(ctx->lock, (unsigned long)flags);
}

/*
 * topazdd_int_disable
 */
static void topazdd_int_disable(struct topaz_dev_ctx *ctx, unsigned int mask)
{
	unsigned int reg;
	unsigned long flags;

	spin_lock_irqsave(ctx->lock, flags);

	/* config interrupts on Topaz core */
	reg = VXE_RD_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_MULTICORE_HOST_INT_ENAB);

	/* clear enable interrupt bits */
	reg &= ~mask;
	VXE_WR_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_MULTICORE_HOST_INT_ENAB, reg);

	spin_unlock_irqrestore(ctx->lock, (unsigned long)flags);
}

/*
 * Get the number of pipes present
 */
unsigned int topazdd_get_num_pipes(struct topaz_dev_ctx *ctx)
{
	static unsigned int g_pipes_avail;

	if (!ctx->multi_core_mem_addr)
		return 0;

	if (g_pipes_avail == 0) {
		/* get the actual number of cores */
		g_pipes_avail = VXE_RD_REG32(ctx->multi_core_mem_addr,
					     TOPAZHP_TOP_CR_MULTICORE_HW_CFG);
		g_pipes_avail = (g_pipes_avail & MASK_TOPAZHP_TOP_CR_NUM_CORES_SUPPORTED);
		IMG_DBG_ASSERT(g_pipes_avail != 0);
	}

	return g_pipes_avail;
}

unsigned int topazdd_get_core_rev(void)
{
	return g_core_rev;
}

unsigned int topazdd_get_core_des1(void)
{
	return g_core_des1;
}

static void wbfifo_clear(struct img_comm_socket *sock)
{
	sock->in_fifo_producer = 0;
	sock->in_fifo_consumer = 0;
}

static unsigned char wbfifo_add(struct img_comm_socket *sock, struct img_writeback_msg *msg)
{
	unsigned int new_producer = sock->in_fifo_producer + 1;

	if (new_producer == COMM_INCOMING_FIFO_SIZE)
		new_producer = 0;

	if (new_producer == sock->in_fifo_consumer)
		return FALSE;

	memcpy(&sock->in_fifo[sock->in_fifo_producer], msg, sizeof(struct img_writeback_msg));

	sock->in_fifo_producer = new_producer;

	return TRUE;
}

static unsigned char wbfifo_is_empty(struct img_comm_socket *sock)
{
	return (sock->in_fifo_producer == sock->in_fifo_consumer);
}

static unsigned char wbfifo_get(struct img_comm_socket *sock, struct img_writeback_msg *msg)
{
	if (wbfifo_is_empty(sock))
		return FALSE;

	memcpy(msg, &sock->in_fifo[sock->in_fifo_consumer], sizeof(struct img_writeback_msg));

	sock->in_fifo_consumer++;

	if (sock->in_fifo_consumer == COMM_INCOMING_FIFO_SIZE)
		sock->in_fifo_consumer = 0;

	return TRUE;
}

unsigned char topazdd_is_idle(struct img_comm_socket *sock)
{
	if (sock->msgs_sent == sock->ack_recv && wbfifo_is_empty(sock))
		return TRUE;

	return FALSE;
}

static void set_auto_clock_gating(struct topaz_dev_ctx *ctx, struct img_fw_context *fw_ctx,
				  unsigned char gating)
{
	unsigned int reg;

	reg = F_ENCODE(1U, TOPAZHP_TOP_CR_WRITES_CORE_ALL);
	VXE_WR_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_MULTICORE_CORE_SEL_0, reg);

	reg = F_ENCODE(gating, TOPAZHP_CR_TOPAZHP_IPE0_AUTO_CLK_GATE) |
		F_ENCODE(gating, TOPAZHP_CR_TOPAZHP_IPE1_AUTO_CLK_GATE) |
		F_ENCODE(gating, TOPAZHP_CR_TOPAZHP_SPE0_AUTO_CLK_GATE) |
		F_ENCODE(gating, TOPAZHP_CR_TOPAZHP_SPE1_AUTO_CLK_GATE) |
		F_ENCODE(gating, TOPAZHP_CR_TOPAZHP_H264COMP4X4_AUTO_CLK_GATE) |
		F_ENCODE(gating, TOPAZHP_CR_TOPAZHP_H264COMP8X8_AUTO_CLK_GATE) |
		F_ENCODE(gating, TOPAZHP_CR_TOPAZHP_H264COMP16X16_AUTO_CLK_GATE) |
		F_ENCODE(gating, TOPAZHP_CR_TOPAZHP_JMCOMP_AUTO_CLK_GATE) |
		F_ENCODE(gating, TOPAZHP_CR_TOPAZHP_VLC_AUTO_CLK_GATE) |
		F_ENCODE(gating, TOPAZHP_CR_TOPAZHP_DEB_AUTO_CLK_GATE) |
		F_ENCODE(gating, TOPAZHP_CR_TOPAZHP_PC_DM_AUTO_CLK_GATE) |
		F_ENCODE(gating, TOPAZHP_CR_TOPAZHP_PC_DMS_AUTO_CLK_GATE) |
		F_ENCODE(gating, TOPAZHP_CR_TOPAZHP_CABAC_AUTO_CLK_GATE) |
		F_ENCODE(gating, TOPAZHP_CR_TOPAZHP_INPUT_SCALER_AUTO_CLK_GATE);

	VXE_WR_REG32(ctx->hp_core_reg_addr[0], TOPAZHP_CR_TOPAZHP_AUTO_CLOCK_GATING, reg);

	reg = 0;
	reg = VXE_RD_REG32(ctx->hp_core_reg_addr[0], TOPAZHP_CR_TOPAZHP_MAN_CLOCK_GATING);

	/* Disable LRITC clocks */
	reg = F_INSERT(reg, 1, TOPAZHP_CR_TOPAZHP_LRITC_MAN_CLK_GATE);

	VXE_WR_REG32(ctx->hp_core_reg_addr[0], TOPAZHP_CR_TOPAZHP_MAN_CLOCK_GATING, reg);

	reg = F_ENCODE(0, TOPAZHP_TOP_CR_WRITES_CORE_ALL);
	VXE_WR_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_MULTICORE_CORE_SEL_0, reg);
}

static void comm_lock(struct topaz_dev_ctx *ctx, unsigned int flags)
{
	if (flags & COMM_LOCK_TX)
		mutex_lock_nested(ctx->comm_tx_mutex, SUBCLASS_TOPAZDD_TX);
}

static void comm_unlock(struct topaz_dev_ctx *ctx, unsigned int flags)
{
	if (flags & COMM_LOCK_TX)
		mutex_unlock((struct mutex *)ctx->comm_tx_mutex);
}

int comm_prepare_fw(struct img_fw_context *fw_ctx, enum img_codec codec)
{
	if (fw_ctx->populated || fw_ctx->initialized)
		return IMG_SUCCESS;

	return mtx_populate_fw_ctx(codec, fw_ctx);
}

static unsigned int H264_RCCONFIG_TABLE_5[27] = {
	0x00000007, 0x00000006, 0x00000006, 0x00000006, 0x00000006, 0x00000005, 0x00000005,
	0x00000005, 0x00000005,
	0x00000005, 0x00000005, 0x00000004, 0x00000004,
	0x00000004, 0x00000004, 0x00000004, 0x00000004, 0x00000004, 0x00000004, 0x00000005,
	0x00000005, 0x00000005,
	0x00000005, 0x00000005, 0x00000005, 0x00000006,
	0x00000006,
};

static unsigned int H264_RCCONFIG_TABLE_6[27] = {
	0x00000018, 0x00000018, 0x00000018, 0x00000018, 0x00000018, 0x00000018, 0x00000018,
	0x00000018, 0x00000024,
	0x00000030, 0x00000030, 0x0000003c, 0x0000003c,
	0x00000048, 0x00000048, 0x00000054, 0x00000060, 0x0000006c, 0x000000c8, 0x00000144,
	0x00000180, 0x00000210,
	0x000002a0, 0x00000324, 0x0000039c, 0x00000414,
	0x00000450,
};

static unsigned int H264_RCCONFIG_TABLE_7[27] = {
	0x00000014, 0x00000014, 0x00000014, 0x00000014, 0x00000014, 0x00000014, 0x00000032,
	0x00000064, 0x000000d2,
	0x000001a4, 0x000001a4, 0x000001bd, 0x000001d6,
	0x000001ef, 0x00000208, 0x00000217, 0x00000226, 0x0000023a, 0x000002cb, 0x0000035c,
	0x00000384, 0x000003e8,
	0x000004b0, 0x00000578, 0x00000640, 0x00000708,
	0x000007d0,
};

static unsigned int MPEG_RCCONFIG_TABLE_7[17] = {
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0000003c,
	0x000000b4, 0x0000012c,
	0x000001a4, 0x0000021c, 0x00000294, 0x0000030c,
	0x00000384, 0x000003fc, 0x00000474, 0x000004ec,
};

/*
 * Load the tables for H.264
 */
void comm_load_h264_tables(struct topaz_dev_ctx *ctx)
{
	int n;
	unsigned int pipe, pipe_cnt;

	pipe_cnt = topazdd_get_num_pipes(ctx);

	for (n = 26; n >= 0; n--) {
		VXE_WR_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_RC_CONFIG_TABLE4, 0);
		VXE_WR_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_RC_CONFIG_TABLE5,
			     H264_RCCONFIG_TABLE_5[n]);
		VXE_WR_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_RC_CONFIG_TABLE6,
			     H264_RCCONFIG_TABLE_6[n]);
		VXE_WR_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_RC_CONFIG_TABLE7,
			     H264_RCCONFIG_TABLE_7[n]);
	}

	for (pipe = 0; pipe < pipe_cnt; pipe++) {
		VXE_WR_REG32(ctx->hp_core_reg_addr[pipe], TOPAZHP_CR_RC_CONFIG_REG8, 0x00000006);
		VXE_WR_REG32(ctx->hp_core_reg_addr[pipe], TOPAZHP_CR_RC_CONFIG_REG9, 0x00000406);
	}
}

/*
 * Load the tables for mpeg4
 */
void comm_load_tables(struct topaz_dev_ctx *ctx)
{
	int n;
	unsigned int pipe;

	for (n = 16; n > 0; n--) {
		VXE_WR_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_RC_CONFIG_TABLE4, 0);
		VXE_WR_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_RC_CONFIG_TABLE6, 0);
		VXE_WR_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_RC_CONFIG_TABLE7,
			     MPEG_RCCONFIG_TABLE_7[n]);
	}

	for (pipe = 0; pipe < topazdd_get_num_pipes(ctx); pipe++)
		VXE_WR_REG32(ctx->hp_core_reg_addr[pipe], TOPAZHP_CR_RC_CONFIG_REG8, 0x00000006);
}

/*
 * Load bias tables
 */
static int comm_load_bias(struct topaz_dev_ctx *ctx, unsigned int codec_mask)
{
	if ((codec_mask & CODEC_MASK_H263) || (codec_mask & CODEC_MASK_MPEG2) ||
	    (codec_mask & CODEC_MASK_MPEG4))
		comm_load_tables(ctx);

	if ((codec_mask & CODEC_MASK_H264) || (codec_mask & CODEC_MASK_H264MVC))
		comm_load_h264_tables(ctx);

	return IMG_SUCCESS;
}

/*
 * Loads MTX firmware
 */
void topaz_setup_firmware(struct topaz_dev_ctx *ctx,
			  struct img_fw_context *fw_ctx,
			  enum mtx_load_method load_method,
			  enum img_codec codec, unsigned char num_pipes)
{
	unsigned int reg;
	unsigned int secure_reg;
	int ret;

	fw_ctx->initialized = FALSE;

	/* Reset the MTXs and Upload the code. */
	/* start each MTX in turn MUST start with master to enable comms to other cores */

#if SECURE_IO_PORTS
	/* reset SECURE_CONFIG register to allow loading FW without security.
	 * Default option is secure.
	 */

	secure_reg = 0x000F0F0F;

	VXE_WR_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_SECURE_CONFIG, secure_reg);
#endif

	ret = comm_prepare_fw(fw_ctx, codec);

	if (ret != IMG_SUCCESS) {
		pr_err("Failed to populate firmware context. Error code: %i\n", ret);
		return;
	}

	/* initialise the MTX */
	mtx_initialize(ctx, fw_ctx);

	/* clear TOHOST register now so that our ISR doesn't see any
	 * intermediate value before the FW has output anything
	 */
	VXE_WR_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_FIRMWARE_REG_1 +
		     (MTX_SCRATCHREG_TOHOST << 2), 0);

	/* clear BOOTSTATUS register.  Firmware will write to
	 * this to indicate firmware boot progress
	 */
	VXE_WR_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_FIRMWARE_REG_1 +
		     (MTX_SCRATCHREG_BOOTSTATUS << 2), 0);

	/* Soft reset of MTX */
	reg = 0;
	reg = F_ENCODE(1, TOPAZHP_TOP_CR_IMG_TOPAZ_MTX_SOFT_RESET) |
		       F_ENCODE(1, TOPAZHP_TOP_CR_IMG_TOPAZ_CORE_SOFT_RESET);
	VXE_WR_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_MULTICORE_SRST, reg);
	VXE_WR_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_MULTICORE_SRST, 0x0);

	if (fw_ctx->initialized) {
		set_auto_clock_gating(ctx, fw_ctx, 1);
		mtx_load(ctx, fw_ctx, load_method);

		/* flush the command FIFO */
		reg = 0;
		reg = F_ENCODE(1, TOPAZHP_TOP_CR_CMD_FIFO_FLUSH);
		VXE_WR_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_TOPAZ_CMD_FIFO_FLUSH, reg);

		/* we do not want to run in secre FW mode so write a place holder
		 * to the FIFO that the firmware will know to ignore
		 */
		VXE_WR_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_MULTICORE_CMD_FIFO_WRITE,
			     TOPAZHP_NON_SECURE_FW_MARKER);

		/* Clear FW_IDLE_STATUS register */
		VXE_WR_REG32(ctx->multi_core_mem_addr, MTX_SCRATCHREG_IDLE, 0);

		/* turn on MTX */
		mtx_start(fw_ctx);
		/* get MTX Clk Freq */

		mtx_kick(fw_ctx, 1);

		/*
		 * We do not need to do this POLL here as it is safe to continue without it.
		 * We do it because it serves to warn us that there is a problem if the
		 * firmware doesn't start for some reason
		 */
		VXE_POLL_REG32_ISEQ(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_FIRMWARE_REG_1 +
				    (MTX_SCRATCHREG_BOOTSTATUS << 2), TOPAZHP_FW_BOOT_SIGNAL,
				    0xffffffff, TOPAZ_TIMEOUT_RETRIES);
	}
}

static int comm_send(struct img_comm_socket *sock, struct mtx_tomtx_msg *msg, unsigned int *wb_val)
{
	struct topaz_dev_ctx *ctx;
	struct img_fw_context *fw_ctx;
	unsigned int space_avail;
	unsigned int cmd_word;
	unsigned int writeback_val;
	enum mtx_cmd_id cmd_id = (enum mtx_cmd_id)(msg->cmd_id & 0x7F);

	ctx = sock->ctx;
	fw_ctx = &ctx->fw_ctx;

	/* mark the context as active in case we need to save its state later */
	fw_ctx->active_ctx_mask |= (1 << sock->id);

	space_avail = VXE_RD_REG32(ctx->multi_core_mem_addr,
				   TOPAZHP_TOP_CR_MULTICORE_CMD_FIFO_WRITE_SPACE);

	space_avail = F_DECODE(space_avail, TOPAZHP_TOP_CR_CMD_FIFO_SPACE);

	if (space_avail < 4)
		return IMG_ERROR_RETRY;

	/* Write command to FIFO */
	cmd_word = F_ENCODE(sock->id, MTX_MSG_CORE) | msg->cmd_id;

	if (msg->cmd_id & MTX_CMDID_PRIORITY) {
		/* increment the command counter */
		sock->high_cmd_cnt++;

		/* Prepare high priority command */
		cmd_word |= F_ENCODE(1, MTX_MSG_PRIORITY) |
			F_ENCODE(((sock->low_cmd_cnt - 1) & 0xff) | (sock->high_cmd_cnt << 8),
				 MTX_MSG_COUNT);
	} else {
		/* Prepare low priority command */
		cmd_word |= F_ENCODE(sock->low_cmd_cnt & 0xff, MTX_MSG_COUNT);
	}

	/* write command into FIFO */
	VXE_WR_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_MULTICORE_CMD_FIFO_WRITE, cmd_word);

	/* Write data to FIFO */
	VXE_WR_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_MULTICORE_CMD_FIFO_WRITE, msg->data);

	if (msg->command_data_buf) {
		/* Write address */
		VXE_WR_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_MULTICORE_CMD_FIFO_WRITE,
			     msg->command_data_buf->dev_virt);
	} else {
		/* Write nothing */
		VXE_WR_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_MULTICORE_CMD_FIFO_WRITE, 0);
	}

	/* Write writeback value to FIFO */

	/* prepare Writeback value */

	/* We don't actually use this value, but it may be useful to customers */
	if (msg->cmd_id & MTX_CMDID_PRIORITY) {
		/* HIGH priority command */

		writeback_val = sock->high_cmd_cnt << 24;

		VXE_WR_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_MULTICORE_CMD_FIFO_WRITE,
			     writeback_val);
	} else {
		/* LOW priority command */
		writeback_val = sock->low_cmd_cnt << 16;

		VXE_WR_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_MULTICORE_CMD_FIFO_WRITE,
			     writeback_val);

		/* increment the command counter */
		sock->low_cmd_cnt++;
	}

	if (wb_val)
		*wb_val = writeback_val;

	sock->last_sync = writeback_val;

	switch (cmd_id) {
	case MTX_CMDID_PROVIDE_CODEDPACKAGE_BUFFER:
	{
#ifdef DEBUG_ENCODER_DRIVER
		unsigned int slot;

		slot = F_DECODE(msg->data, MTX_MSG_PROVIDE_CODEDPACKAGE_BUFFER_SLOT);
		pr_debug("MSG_TX[%d]: %s(%d) %s %s cmd: %#08x cmd_word: %#08x data: %#08x: addr: 0x%p writeback_val: %#08x\n",
			 sock->id, command_string[cmd_id], slot,
			 (msg->cmd_id & MTX_CMDID_PRIORITY ? "(PRIORITY)" : "(NORMAL)"),
			 (msg->cmd_id & MTX_CMDID_WB_INTERRUPT ? "(Interrupt)" : "(NO Interrupt)"),
			 (msg->cmd_id), cmd_word, (msg->data), msg->command_data_buf,
			 writeback_val);
#endif
		break;
	}
#ifdef ENABLE_PROFILING
	case MTX_CMDID_ENCODE_FRAME:
	{
		struct timespec64 time;

		ktime_get_real_ts64(&time);

		sock->fw_lat.start_time = timespec64_to_ns((const struct timespec64 *)&time);
	}
#endif
	default:
#ifdef DEBUG_ENCODER_DRIVER
		pr_debug("MSG_TX[%d]: %s %s %s cmd: %#08x cmd_word: %#08x data: %#08x addr: 0x%p writeback_val: %#08x\n",
			 sock->id, command_string[cmd_id],
			 (msg->cmd_id & MTX_CMDID_PRIORITY ? "(PRIORITY)" : "(NORMAL)"),
			 (msg->cmd_id & MTX_CMDID_WB_INTERRUPT ? "(Interrupt)" : "(NO Interrupt)"),
			 (msg->cmd_id), cmd_word, (msg->data), msg->command_data_buf,
			 writeback_val);
#endif
		break;
	}
#ifdef DEBUG_ENCODER_DRIVER
	if (msg->command_data_buf) {
		int i;

		pr_debug("Has msg->command_data_buf cpu_virt=0x%p dev_virt=%#08x\n",
			 msg->command_data_buf->cpu_virt, msg->command_data_buf->dev_virt);

		for (i = 0; i < 350; i++) {
			pr_debug("MSG_TX %03d %#08x\n", i,
				 ((unsigned int *)msg->command_data_buf->cpu_virt)[i]);
		}
	}
#endif

	/* kick the master MTX */
	mtx_kick(fw_ctx, 1);

	sock->msgs_sent++;

	return IMG_SUCCESS;
}

int topazdd_send_msg(void *dd_str_ctx, enum mtx_cmd_id cmd_id,
		     unsigned int data, struct vidio_ddbufinfo *cmd_data_buf,
		     unsigned int *wb_val)
{
	struct mtx_tomtx_msg *msg;
	struct img_comm_socket *sock;
	int err;

	if (!dd_str_ctx)
		return IMG_ERROR_INVALID_CONTEXT;

	sock = (struct img_comm_socket *)dd_str_ctx;

	msg = kmalloc(sizeof(*msg), GFP_KERNEL);
	IMG_DBG_ASSERT(msg);
	if (!msg)
		return IMG_ERROR_UNDEFINED;

	msg->command_data_buf = cmd_data_buf;
	msg->cmd_id = cmd_id;
	msg->data = data;

	if (!wb_val) {
		comm_lock(sock->ctx, COMM_LOCK_TX);
		err = comm_send(sock, msg, NULL);
		comm_unlock(sock->ctx, COMM_LOCK_TX);
	} else {
		unsigned int ret_wb_val;

		comm_lock(sock->ctx, COMM_LOCK_TX);
		err = comm_send(sock, msg, &ret_wb_val);
		comm_unlock(sock->ctx, COMM_LOCK_TX);

		if (err == IMG_SUCCESS)
			*wb_val = ret_wb_val;
	}

	kfree(msg);
	return err;
}

#define WAIT_FOR_SYNC_RETRIES 1200
#define WAIT_FOR_SYNC_TIMEOUT 1

static int wait_event_obj(void *event, unsigned char uninterruptible, unsigned int timeout)
{
	struct event *p_event = (struct event *)event;
	int ret;

	IMG_DBG_ASSERT(event);
	if (!event)
		return IMG_ERROR_GENERIC_FAILURE;

	if (uninterruptible) {
		if (timeout == (unsigned int)(-1)) {
			ret = 0;
			wait_event(event_wait_queue, p_event->signalled);
		} else {
			ret = wait_event_timeout(event_wait_queue, p_event->signalled, timeout);
			if (!ret)
				return IMG_ERROR_TIMEOUT;
		}
	} else {
		if (timeout == (unsigned int)(-1)) {
			ret = wait_event_interruptible(event_wait_queue, p_event->signalled);
		} else {
			ret = wait_event_interruptible_timeout(event_wait_queue,
							       p_event->signalled, timeout);
			if (!ret)
				return IMG_ERROR_TIMEOUT;
		}
	}

	/* If there are signals pending... */
	if (ret == -ERESTARTSYS)
		return IMG_ERROR_INTERRUPTED;

	/* If there was no signal...*/
	IMG_DBG_ASSERT(p_event->signalled);

	/* Clear signal pending...*/
	p_event->signalled = FALSE;

	return IMG_SUCCESS;
}

static int topazdd_wait_on_sync(struct img_comm_socket *sock, unsigned int wb_val)
{
	unsigned int retries = 0;

	if (!sock)
		return IMG_ERROR_INVALID_CONTEXT;

	while (wait_event_obj(sock->event, TRUE, WAIT_FOR_SYNC_TIMEOUT) != IMG_SUCCESS) {
		if (retries == WAIT_FOR_SYNC_RETRIES) {
			/*
			 * We shouldn't wait any longer than that!
			 * If the hardware locked up, we will get stuck otherwise.
			 */
			pr_err("TIMEOUT: %s timed out waiting for writeback 0x%08x.\n",
			       __func__, sock->sync_wb_val);
			return IMG_ERROR_TIMEOUT;
		}

		msleep(WAIT_FOR_SYNC_TIMEOUT);
		retries++;
		continue;
	}

	return IMG_SUCCESS;
}

int topazdd_send_msg_with_sync(void *dd_str_ctx, enum mtx_cmd_id cmd_id,
			       unsigned int data,
			       struct vidio_ddbufinfo *cmd_data_buf)
{
	struct img_comm_socket *sock;
	unsigned int wb_val = 0;

	if (!dd_str_ctx)
		return IMG_ERROR_INVALID_CONTEXT;

	sock = (struct img_comm_socket *)dd_str_ctx;

	mutex_lock_nested(sock->sync_wb_mutex, SUBCLASS_TOPAZDD);
	topazdd_send_msg(dd_str_ctx, cmd_id, data, cmd_data_buf, &wb_val);
	sock->sync_waiting = TRUE;
	sock->sync_wb_val = wb_val;
	mutex_unlock((struct mutex *)sock->sync_wb_mutex);

	return topazdd_wait_on_sync(sock, wb_val);
}

static void stream_worker(void *work)
{
	struct img_comm_socket *sock = NULL;
	struct img_writeback_msg msg;
	struct event *p_event;

	work = get_work_buff(work, FALSE);
	sock = container_of(work, struct img_comm_socket, work);

	while (wbfifo_get(sock, &msg)) {
		if (F_DECODE(msg.cmd_word, MTX_MSG_MESSAGE_ID) == MTX_MESSAGE_ACK)
			sock->ack_recv++;

		mutex_lock_nested(sock->sync_wb_mutex, SUBCLASS_TOPAZDD);
		if (sock->sync_waiting && msg.writeback_val == sock->sync_wb_val) {
			sock->sync_waiting = FALSE;
			mutex_unlock((struct mutex *)sock->sync_wb_mutex);
			/* signal the waiting sync event */
			p_event = (struct event *)sock->event;

			IMG_DBG_ASSERT(sock->event);
			if (!sock->event)
				return;

			p_event->signalled = TRUE;
			wake_up(&event_wait_queue);
			return;
		}
		mutex_unlock((struct mutex *)sock->sync_wb_mutex);

		if (sock->cb)
			sock->cb(&msg, sock->str_ctx);
	}
}

int topazdd_create_stream_context(struct topaz_dev_ctx *ctx, enum img_codec codec,
				  enc_cb cb, void *cb_priv,
	void **dd_str_ctx, struct vidio_ddbufinfo **wb_data_info)
{
	struct img_comm_socket *p_sock;
	struct event *p_event;

	p_sock = kmalloc(sizeof(*p_sock), GFP_KERNEL);
	IMG_DBG_ASSERT(p_sock);
	if (!p_sock)
		return IMG_ERROR_OUT_OF_MEMORY;

	p_sock->sync_wb_mutex = kzalloc(sizeof(*p_sock->sync_wb_mutex), GFP_KERNEL);
	if (!p_sock->sync_wb_mutex) {
		kfree(p_sock);
		return IMG_ERROR_OUT_OF_MEMORY;
	}
	mutex_init(p_sock->sync_wb_mutex);

	/* Allocate a Sync structure...*/
	p_event = kmalloc(sizeof(struct event *), GFP_KERNEL);
	IMG_DBG_ASSERT(p_event);
	if (!p_event)
		return IMG_ERROR_OUT_OF_MEMORY;

	memset(p_event, 0, sizeof(struct event));

	p_sock->event = (void *)p_event;

	if (!p_sock->event) {
		mutex_destroy(p_sock->sync_wb_mutex);
		kfree(p_sock->sync_wb_mutex);
		p_sock->sync_wb_mutex = NULL;
		kfree(p_sock);
		return IMG_ERROR_OUT_OF_MEMORY;
	}

	p_sock->low_cmd_cnt = 0xa5a5a5a5 %  MAX_TOPAZ_CMD_COUNT;
	p_sock->high_cmd_cnt = 0;
	p_sock->msgs_sent = 0;
	p_sock->ack_recv = 0;
	p_sock->codec = codec;
	p_sock->ctx = ctx;
	p_sock->cb = cb;
	p_sock->str_ctx = (struct topaz_stream_context *)cb_priv;

	init_work(&p_sock->work, stream_worker, HWA_ENCODER);
	if (!p_sock->work) {
		mutex_destroy(p_sock->sync_wb_mutex);
		kfree(p_sock->sync_wb_mutex);
		p_sock->sync_wb_mutex = NULL;
		kfree(p_sock);
		return IMG_ERROR_OUT_OF_MEMORY;
	}

	wbfifo_clear(p_sock);

	*wb_data_info = g_aps_wb_data_info;

	*dd_str_ctx = (void *)p_sock;

#ifdef DEBUG_ENCODER_DRIVE
	pr_info("topazdd context created with codec %d\n", codec);
#endif

	return IMG_SUCCESS;
}

static int topaz_upload_firmware(struct topaz_dev_ctx *ctx, enum img_codec codec)
{
#ifdef DEBUG_ENCODER_DRIVE
	pr_info("Loading firmware.\n");
#endif
	/* Upload FW */
	/* load and start MTX cores */
	ctx->fw_ctx.load_method = (enum mtx_load_method)g_load_method;

	topaz_setup_firmware(ctx, &ctx->fw_ctx, ctx->fw_ctx.load_method,
			     codec, topazdd_get_num_pipes(ctx));

	if (!ctx->fw_ctx.initialized) {
		pr_err("\nERROR: Firmware cannot be loaded!\n");
		return IMG_ERROR_UNDEFINED;
	}

	comm_load_bias(ctx, ctx->fw_ctx.supported_codecs);
	/* initialise read offset of firmware output fifo */
	VXE_WR_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_FIRMWARE_REG_1 +
		     (MTX_SCRATCHREG_TOMTX << 2), 0);

	ctx->fw_uploaded = codec;

#ifdef DEBUG_ENCODER_DRIVE
	pr_info("firmware uploaded!\n");
#endif
	return IMG_SUCCESS;
}

int topazdd_setup_stream_ctx(void *dd_str_ctx, unsigned short height,
			     unsigned short width, unsigned char *ctx_num,
			     unsigned int *used_sock)
{
	unsigned char idx;
	struct img_fw_context *fw_ctx;
	struct img_comm_socket *sock;
	int res = IMG_ERROR_UNDEFINED;
	unsigned int codec_mask = 0;

	sock = (struct img_comm_socket *)dd_str_ctx;

	comm_lock(sock->ctx, COMM_LOCK_BOTH);

	fw_ctx = &sock->ctx->fw_ctx;

	switch (sock->codec) {
	case IMG_CODEC_JPEG:
		codec_mask = CODEC_MASK_JPEG;
		break;
	case IMG_CODEC_H264_NO_RC:
	case IMG_CODEC_H264_VBR:
	case IMG_CODEC_H264_CBR:
	case IMG_CODEC_H264_VCM:
	case IMG_CODEC_H264_ERC:
		codec_mask = CODEC_MASK_H264;
		break;
	case IMG_CODEC_H263_NO_RC:
	case IMG_CODEC_H263_VBR:
	case IMG_CODEC_H263_CBR:
	case IMG_CODEC_H263_ERC:
		codec_mask = CODEC_MASK_H263;
		break;
	case IMG_CODEC_MPEG4_NO_RC:
	case IMG_CODEC_MPEG4_VBR:
	case IMG_CODEC_MPEG4_CBR:
	case IMG_CODEC_MPEG4_ERC:
		codec_mask = CODEC_MASK_MPEG4;
		break;
	case IMG_CODEC_MPEG2_NO_RC:
	case IMG_CODEC_MPEG2_VBR:
	case IMG_CODEC_MPEG2_CBR:
	case IMG_CODEC_MPEG2_ERC:
		codec_mask = CODEC_MASK_MPEG2;
		break;

	case IMG_CODEC_H264MVC_NO_RC:
	case IMG_CODEC_H264MVC_VBR:
	case IMG_CODEC_H264MVC_CBR:
	case IMG_CODEC_H264MVC_ERC:
		codec_mask = CODEC_MASK_H264MVC;
		break;
	default:
		IMG_DBG_ASSERT("Impossible use case!\n" == NULL);
		break;
	}
	/* Only do the following checks if some other firmware is loaded */
	if (sock->ctx->fw_uploaded != IMG_CODEC_NONE &&
	    (sock->ctx->fw_uploaded != sock->codec || /* Different firmware is uploaded */
		 /* We currently only support one JPEG context to be encoded at the same time */
	     (sock->ctx->fw_uploaded == IMG_CODEC_JPEG && sock->ctx->used_socks))) {
		if (!(fw_ctx->supported_codecs & codec_mask)) {
			comm_unlock(sock->ctx, COMM_LOCK_BOTH);
			res = IMG_ERROR_UNDEFINED;
			pr_err("\nERROR: Incompatible firmware context types!. Required codec: 0x%x Loaded FW : 0x%x\n",
			       codec_mask, fw_ctx->supported_codecs);
			return res;
		}
	}

	if (fw_ctx->initialized && sock->ctx->used_socks >= fw_ctx->num_contexts) {
		/* the firmware can't support any more contexts */
		comm_unlock(sock->ctx, COMM_LOCK_BOTH);
		pr_err("\nERROR: Firmware context limit reached!\n");
		return IMG_ERROR_UNDEFINED;
	}

	/* Search for an Available socket. */
	IMG_DBG_ASSERT(TOPAZHP_MAX_POSSIBLE_STREAMS < (1 << 8));
	for (idx = 0; idx < TOPAZHP_MAX_POSSIBLE_STREAMS; idx++) {
		if (!(sock->ctx->socks[idx])) {
			unsigned int index = idx;

			sock->id = idx;
			*ctx_num = idx;
			*used_sock = index;
			sock->ctx->socks[idx] = sock;
			sock->ctx->used_socks++;
			break;
		}
	}

	if (idx == TOPAZHP_MAX_POSSIBLE_STREAMS) {
		comm_unlock(sock->ctx, COMM_LOCK_BOTH);
		return IMG_ERROR_INVALID_SIZE;
	}

	if (sock->codec == IMG_CODEC_JPEG) {
		topaz_timeout_retries = TOPAZ_TIMEOUT_JPEG;
	} else {
		unsigned int mbs_per_pic = (height * width) / 256;

		if (topaz_timeout_retries < (mbs_per_pic + 10) * 100)
			topaz_timeout_retries = (mbs_per_pic + 10) * 100;
	}

	if (sock->ctx->fw_uploaded == IMG_CODEC_NONE) {
#ifdef DEBUG_ENCODER_DRIVE
		pr_info("Loading a different firmware.\n");
#endif
		res = topaz_upload_firmware(sock->ctx, (enum img_codec)sock->codec);
		if (!res) {
			comm_unlock(sock->ctx, COMM_LOCK_BOTH);
			res = IMG_ERROR_UNDEFINED;
			pr_err("\nERROR: Firmware cannot be loaded!\n");
			return res;
		}
	}

	res = IMG_SUCCESS;

	comm_unlock(sock->ctx, COMM_LOCK_BOTH);

	return res;
}

void topazdd_destroy_stream_ctx(void *dd_str_ctx)
{
	unsigned int idx;
	struct img_comm_socket *sock;

	sock = (struct img_comm_socket *)dd_str_ctx;

	WARN_ON((!sock));
	if (!sock) {
		pr_err("topazdd_destroy_sock: invalid sock\n");
		return;
	}

	flush_work(sock->work);

	mutex_lock_nested(sock->sync_wb_mutex, SUBCLASS_TOPAZDD);
	comm_lock(sock->ctx, COMM_LOCK_BOTH);
	for (idx = 0; idx < TOPAZHP_MAX_POSSIBLE_STREAMS; idx++) {
		if (sock->ctx->socks[idx] == sock) {
			sock->ctx->used_socks--;
			break;
		}
	}

#ifdef DEBUG_ENCODER_DRIVE
	pr_info("topazdd sock context closed\n");
#endif

	/* Flush the MMU table cache (so it we can't accidentally access
	 * the freed device memory due to cache/table mismatch.)
	 */
	topaz_core_mmu_flush_cache();

	/*
	 * if nIndex == TOPAZHP_MAX_POSSIBLE_STREAMS then OpenSocket succeeded
	 * and SetupSocket failed (maybe incompatible firmware)
	 */
	if (idx != TOPAZHP_MAX_POSSIBLE_STREAMS) {
		/*
		 * Abort the stream first.
		 * This function can be called as a result of abnormal process
		 * exit, and since the hardware might be encoding some frame it
		 * means that the hardware still needs the context resources
		 * (buffers mapped to the hardware, etc), so we need to make
		 * sure that hardware encoding is aborted first before releasing
		 * the resources.
		 * This is important if you're doing several encodes
		 * simultaneously because releasing the resources too early will
		 * cause a page-fault that will halt all simultaneous encodes
		 * not just the one that caused the page-fault.
		 */
		struct mtx_tomtx_msg msg;
		unsigned int wb_val = 0;

		wbfifo_clear(sock);

		msg.cmd_id = (enum mtx_cmd_id)(MTX_CMDID_ABORT | MTX_CMDID_PRIORITY |
				MTX_CMDID_WB_INTERRUPT);
		msg.data = 0;
		msg.command_data_buf = NULL;
		comm_send(sock, &msg, &wb_val);
		sock->sync_waiting = TRUE;
		sock->sync_wb_val = wb_val;
		mutex_unlock((struct mutex *)sock->sync_wb_mutex);

		topazdd_wait_on_sync(sock, wb_val);
		/*
		 * Set it to NULL here -not any time sooner-, we need it in case
		 * we had to abort the stream.
		 */
		sock->ctx->socks[idx] = NULL;
	}

	comm_unlock(sock->ctx, COMM_LOCK_BOTH);
	kfree(sock->event);
	mutex_destroy(sock->sync_wb_mutex);
	kfree(sock->sync_wb_mutex);
	sock->sync_wb_mutex = NULL;
	kfree(sock->work);
	kfree(sock);
}

/*
 * topazdd_int_clear
 */
static void topazdd_int_clear(struct topaz_dev_ctx *ctx, unsigned int mask)
{
	unsigned long flags;

	spin_lock_irqsave(ctx->lock, flags);
	VXE_WR_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_MULTICORE_INT_CLEAR, mask);

	spin_unlock_irqrestore(ctx->lock, (unsigned long)flags);
}

unsigned char topazdd_get_pipe_usage(unsigned char pipe)
{
	IMG_DBG_ASSERT(pipe < TOPAZHP_MAX_NUM_PIPES);
	if (pipe >= TOPAZHP_MAX_NUM_PIPES)
		return 0;

	return g_pipe_usage[pipe];
}

void topazdd_set_pipe_usage(unsigned char pipe, unsigned char val)
{
	IMG_DBG_ASSERT(pipe < TOPAZHP_MAX_NUM_PIPES);
	if (pipe < TOPAZHP_MAX_NUM_PIPES)
		g_pipe_usage[pipe] = val;
}

static unsigned int comm_get_consumer(struct topaz_dev_ctx *ctx)
{
	unsigned int reg;

	reg = VXE_RD_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_FIRMWARE_REG_1 +
			   (MTX_SCRATCHREG_TOMTX << 2));

	return F_DECODE(reg, WB_CONSUMER);
}

static void comm_set_consumer(struct topaz_dev_ctx *ctx, unsigned int consumer)
{
	unsigned int reg;

	reg = VXE_RD_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_FIRMWARE_REG_1 +
			   (MTX_SCRATCHREG_TOMTX << 2));

	reg = F_INSERT(reg, consumer, WB_CONSUMER);

	VXE_WR_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_FIRMWARE_REG_1 +
		     (MTX_SCRATCHREG_TOMTX << 2), reg);
}

static unsigned int comm_get_producer(struct topaz_dev_ctx *ctx)
{
	unsigned int reg;

	reg = VXE_RD_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_FIRMWARE_REG_1 +
			   (MTX_SCRATCHREG_TOHOST << 2));

	return F_DECODE(reg, WB_PRODUCER);
}

static void comm_set_producer(struct topaz_dev_ctx *ctx, unsigned int producer)
{
	unsigned int reg;

	reg = VXE_RD_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_FIRMWARE_REG_1 +
			   (MTX_SCRATCHREG_TOHOST << 2));

	reg = F_INSERT(reg, producer, WB_PRODUCER);

	VXE_WR_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_FIRMWARE_REG_1 +
		     (MTX_SCRATCHREG_TOHOST << 2), reg);
}

static int topazdd_init_comms(struct topaz_dev_ctx *ctx, unsigned int mmu_flags)
{
	unsigned int num_cores;
	unsigned int i;
	unsigned int reg;

	num_cores = topazdd_get_num_pipes(ctx);

	for (i = 0; i < num_cores; i++) {
		unsigned int offset = REG_TOPAZHP_CORE_0 + (i * 4);

		ctx->hp_core_reg_addr[i] = (void *)topaz_mem_space[offset].cpu_addr;

		offset = REG_TOPAZHP_VLC_CORE_0 + (i * 4);
		ctx->vlc_reg_addr[i] = (void *)topaz_mem_space[offset].cpu_addr;
	}

	if (topaz_mmu_device_create(&ctx->topaz_mmu_ctx, mmu_flags) != IMG_SUCCESS) {
		pr_err("\nERROR: Could not initialize MMU with selected parameters!\n");
		return IMG_ERROR_OUT_OF_MEMORY;
	}

	/* Start up MMU support for each core (if MMU is switched on) */
	reg = (F_ENCODE(1, TOPAZHP_TOP_CR_IMG_TOPAZ_MTX_SOFT_RESET) |
		F_ENCODE(1, TOPAZHP_TOP_CR_IMG_TOPAZ_CORE_SOFT_RESET) |
		F_ENCODE(1, TOPAZHP_TOP_CR_IMG_TOPAZ_IO_SOFT_RESET));

	VXE_WR_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_MULTICORE_SRST, reg);
	VXE_WR_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_MULTICORE_SRST, 0x0);

	for (i = 0; i < num_cores; i++) {
		unsigned int reset_bits = F_ENCODE(1, TOPAZHP_CR_TOPAZHP_IPE_SOFT_RESET) |
			F_ENCODE(1, TOPAZHP_CR_TOPAZHP_SPE_SOFT_RESET) |
			F_ENCODE(1, TOPAZHP_CR_TOPAZHP_PC_SOFT_RESET) |
			F_ENCODE(1, TOPAZHP_CR_TOPAZHP_H264COMP_SOFT_RESET) |
			F_ENCODE(1, TOPAZHP_CR_TOPAZHP_JMCOMP_SOFT_RESET) |
			F_ENCODE(1, TOPAZHP_CR_TOPAZHP_PREFETCH_SOFT_RESET) |
			F_ENCODE(1, TOPAZHP_CR_TOPAZHP_VLC_SOFT_RESET) |
			F_ENCODE(1, TOPAZHP_CR_TOPAZHP_LTRITC_SOFT_RESET) |
			F_ENCODE(1, TOPAZHP_CR_TOPAZHP_DB_SOFT_RESET);

#ifdef TOPAZHP // TODO: strangely, this doesn't seem defined in the build... but we ARE topazhp...
		reset_bits |= F_ENCODE(1, MVEA_CR_IMG_MVEA_SPE_SOFT_RESET(1)) |
			F_ENCODE(1, MVEA_CR_IMG_MVEA_IPE_SOFT_RESET(1));
#endif

		VXE_WR_REG32(ctx->hp_core_reg_addr[i], TOPAZHP_CR_TOPAZHP_SRST, reset_bits);

		VXE_WR_REG32(ctx->hp_core_reg_addr[i], TOPAZHP_CR_TOPAZHP_SRST, 0);
	}

	ctx->topaz_mmu_ctx.ptd_phys_addr = ctx->ptd;
	topaz_core_mmu_hw_setup(&ctx->topaz_mmu_ctx, ctx->multi_core_mem_addr);

	ctx->fw_uploaded = IMG_CODEC_NONE;

	g_core_rev = VXE_RD_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_TOPAZHP_CORE_REV);
	g_core_rev &=
		(MASK_TOPAZHP_TOP_CR_TOPAZHP_MAINT_REV | MASK_TOPAZHP_TOP_CR_TOPAZHP_MINOR_REV |
		MASK_TOPAZHP_TOP_CR_TOPAZHP_MAJOR_REV);
	g_core_des1 = VXE_RD_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_TOPAZHP_CORE_DES1);

	ctx->comm_tx_mutex = kzalloc(sizeof(*ctx->comm_tx_mutex), GFP_KERNEL);
	if (!(ctx->comm_tx_mutex))
		return IMG_ERROR_OUT_OF_MEMORY;

	mutex_init(ctx->comm_tx_mutex);

	ctx->comm_rx_mutex = kzalloc(sizeof(*ctx->comm_rx_mutex), GFP_KERNEL);
	if (!ctx->comm_rx_mutex) {
		mutex_destroy(ctx->comm_tx_mutex);
		kfree(ctx->comm_tx_mutex);
		ctx->comm_tx_mutex = NULL;
		pr_err("Memory allocation failed for mutex\n");
		return IMG_ERROR_OUT_OF_MEMORY;
	}
	mutex_init(ctx->comm_rx_mutex);

	g_aps_wb_data_info = kmalloc(sizeof(*g_aps_wb_data_info) * WB_FIFO_SIZE, GFP_KERNEL);
	if (!g_aps_wb_data_info) {
		mutex_destroy(ctx->comm_rx_mutex);
		kfree(ctx->comm_rx_mutex);
		ctx->comm_rx_mutex = NULL;

		mutex_destroy(ctx->comm_tx_mutex);
		kfree(ctx->comm_tx_mutex);
		ctx->comm_tx_mutex = NULL;
		return IMG_ERROR_OUT_OF_MEMORY;
	}

	/* Allocate WB buffers */
	for (i = 0; i < WB_FIFO_SIZE; i++) {
		struct vidio_ddbufinfo *mem_info = &g_aps_wb_data_info[i];

		if (topaz_mmu_alloc(ctx->topaz_mmu_ctx.mmu_context_handle,
				    ctx->vxe_arg, MMU_GENERAL_HEAP_ID, 1,
			(enum sys_emem_attrib)(SYS_MEMATTRIB_UNCACHED | SYS_MEMATTRIB_WRITECOMBINE),
			COMM_WB_DATA_BUF_SIZE, 64, mem_info)) {
			pr_err("mmu_alloc failed!\n");
			kfree(g_aps_wb_data_info);
			return IMG_ERROR_OUT_OF_MEMORY;
		}
	}

	/* Initialise the COMM registers */
	comm_set_producer(ctx, 0);

	/* Must reset the Consumer register too,
	 * otherwise the COMM stack may be initialised incorrectly
	 */
	comm_set_consumer(ctx, 0);

	for (i = 0; i < TOPAZHP_MAX_POSSIBLE_STREAMS; i++)
		ctx->socks[i] = NULL;

	ctx->used_socks = 0;
	ctx->initialized = TRUE;

	return 0;
}

static void topazdd_deinit_comms(struct topaz_dev_ctx *ctx)
{
	unsigned int idx;
	struct img_fw_context *fw_ctx;

	fw_ctx = &ctx->fw_ctx;

	if (fw_ctx && fw_ctx->initialized) {
		/* Stop the MTX */
		mtx_stop(fw_ctx);
		mtx_wait_for_completion(fw_ctx);
	}

	if (g_aps_wb_data_info) {
		for (idx = 0; idx < WB_FIFO_SIZE; idx++) {
			struct vidio_ddbufinfo *mem_info = &g_aps_wb_data_info[idx];

			topaz_mmu_free(ctx->vxe_arg, mem_info);
		}
		kfree(g_aps_wb_data_info);
	}

	/* Close all of the opened sockets */
	for (idx = 0; idx < TOPAZHP_MAX_POSSIBLE_STREAMS; idx++) {
		if (ctx->socks[idx])
			topazdd_destroy_stream_ctx(ctx->socks[idx]);
	}

	mutex_destroy(ctx->comm_tx_mutex);
	kfree(ctx->comm_tx_mutex);
	ctx->comm_tx_mutex = NULL;

	mutex_destroy(ctx->comm_rx_mutex);
	kfree(ctx->comm_rx_mutex);
	ctx->comm_rx_mutex = NULL;

	if (fw_ctx && fw_ctx->initialized)
		mtx_deinitialize(fw_ctx);

	topaz_mmu_device_destroy(&ctx->topaz_mmu_ctx);

	ctx->fw_uploaded = IMG_CODEC_NONE;
	ctx->initialized = FALSE;
}

static void setup_topaz_mem(unsigned long long reg_base, unsigned int reg_size)
{
	unsigned int idx;

	/* set up the kernel virtual address for mem space access */
	for (idx = 0; idx < topaz_target_config.num_mem_spaces; idx++) {
		unsigned long long offset = topaz_target_config.mem_spaces[idx].reg.addr;

		topaz_target_config.mem_spaces[idx].cpu_addr = reg_base + offset;
	}
}

/*
 * topazdd_init
 */
int topazdd_init(unsigned long long reg_base, unsigned int reg_size, unsigned int mmu_flags,
		 void *vxe_arg, unsigned int ptd, void **data)
{
	struct topaz_dev_ctx *ctx;
	int ret;
	spinlock_t **lock; /* spinlock */

	setup_topaz_mem(reg_base, reg_size);

	/* Allocate device structure...*/
	ctx = kmalloc(sizeof(*ctx), GFP_KERNEL);
	IMG_DBG_ASSERT(ctx);
	if (!ctx)
		return IMG_ERROR_OUT_OF_MEMORY;

	memset(ctx, 0, sizeof(*ctx));

	lock = (spinlock_t **)&ctx->lock;
	*lock = kzalloc(sizeof(spinlock_t), GFP_KERNEL);

	if (!(*lock)) {
		pr_err("Memory allocation failed for spin-lock\n");
		kfree(ctx);
		return IMG_ERROR_OUT_OF_MEMORY;
	}
	spin_lock_init(*lock);
	g_lock = ctx->lock;

	*data = ctx;
	ctx->initialized = FALSE;

	ctx->multi_core_mem_addr = (void *)topaz_mem_space[REG_TOPAZHP_MULTICORE].cpu_addr;

	if (!ctx->multi_core_mem_addr) {
		kfree(&ctx->lock);
		kfree(ctx);
		return IMG_ERROR_DEVICE_NOT_FOUND;
	}

	/* Now enabled interrupts */
	topazdd_int_enable(ctx, (MASK_TOPAZHP_TOP_CR_HOST_INTEN_MTX |
		MASK_TOPAZHP_TOP_CR_HOST_TOPAZHP_MAS_INTEN |
		MASK_TOPAZHP_TOP_CR_HOST_INTEN_MMU_FAULT |
		MASK_TOPAZHP_TOP_CR_HOST_INTEN_MMU_FAULT_B));

	ctx->vxe_arg = vxe_arg;
	ctx->ptd = ptd;

	ret = topazdd_init_comms(ctx, mmu_flags);
	if (ret) {
		topazdd_int_disable(ctx, ~0);
		kfree(&ctx->lock);
		kfree(ctx);
		return ret;
	}

	comm_lock(ctx, COMM_LOCK_BOTH);
	ret = topaz_upload_firmware(ctx, IMG_CODEC_H264_NO_RC);
	comm_unlock(ctx, COMM_LOCK_BOTH);

	if (ret) {
		topazdd_deinit_comms(ctx);
		topazdd_int_disable(ctx, ~0);
		kfree(&ctx->lock);
		kfree(ctx);
		return ret;
	}

	/* Device now initailised...*/
	ctx->initialized = TRUE;

	/* Return success...*/
	return IMG_SUCCESS;
}

/*
 * topazdd_deinit
 */
void topazdd_deinit(void *data)
{
	struct topaz_dev_ctx *ctx = data;
	unsigned int reg;

	/* If the interrupt was defined then it is also safe to clear interrupts
	 * and reset the core....
	 */
	if (ctx->initialized) {
		topazdd_deinit_comms(ctx);

		/* Disable interrupts...*/
		topazdd_int_disable(ctx, ~0);

		/* disable interrupts on Topaz core */
		reg =
			VXE_RD_REG32(ctx->multi_core_mem_addr,
				     TOPAZHP_TOP_CR_MULTICORE_HOST_INT_ENAB);

		reg &= ~MASK_TOPAZHP_TOP_CR_HOST_INTEN_MTX;
		VXE_WR_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_MULTICORE_HOST_INT_ENAB, reg);

		/* clear interrupt - just in case */
		VXE_WR_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_MULTICORE_INT_CLEAR,
			     MASK_TOPAZHP_TOP_CR_INTCLR_MTX);

		g_lock = NULL;
		kfree(&ctx->lock);
	}

	kfree(data);
}

static int comm_dispatch_in_msg(struct topaz_dev_ctx *ctx)
{
	unsigned int hw_fifo_producer;
	unsigned int hw_fifo_consumer;

	hw_fifo_consumer = comm_get_consumer(ctx);
	hw_fifo_producer = comm_get_producer(ctx);

	while (hw_fifo_consumer != hw_fifo_producer) {
		struct img_writeback_msg *wb_msg;
		unsigned char conn_id;
		struct vidio_ddbufinfo *mem_info = &g_aps_wb_data_info[hw_fifo_consumer];
		enum mtx_cmd_id cmd_id;

		/* Update corresponding memory region */
		topaz_update_host_mem(ctx->vxe_arg, mem_info);
		wb_msg = (struct img_writeback_msg *)(mem_info->cpu_virt);

		/* Copy to the corresponding SW fifo */
		conn_id = F_DECODE(wb_msg->cmd_word, MTX_MSG_CORE);

		/* Find corresponding Buffer Addr */
		cmd_id = (enum mtx_cmd_id)F_DECODE(wb_msg->cmd_word, MTX_MSG_MESSAGE_ID);
#ifdef DEBUG_ENCODER_DRIVER
		if ((unsigned int)cmd_id == (unsigned int)MTX_MESSAGE_ACK) {
			pr_debug("MSG_RX[%d]: 0x%03X %s (ACK) cmd_word: %#08x data: %#08x extra_data: %#08x writeback_val: %#08x\n",
				 F_DECODE(wb_msg->cmd_word, MTX_MSG_CORE),
				 hw_fifo_producer & 0x1f,
				 command_string[wb_msg->cmd_word & 0x1f],
				 wb_msg->cmd_word, wb_msg->data,
				 wb_msg->extra_data, wb_msg->writeback_val);
		} else {
#ifdef ENABLE_PROFILING
			struct timespec64 time;

			ktime_get_real_ts64(&time);
			ctx->socks[conn_id]->fw_lat.end_time =
				timespec64_to_ns((const struct timespec64 *)&time);
			pr_err("fw encode time is %llu us for msg_id x%0x\n",
			       div_s64(ctx->socks[conn_id]->fw_lat.end_time -
				       ctx->socks[conn_id]->fw_lat.start_time, 1000),
			       wb_msg->writeback_val);
#endif
			pr_debug("MSG_RX[%d]: 0x%03X CODED_BUFFER cmd_word: %#08x coded_package_consumed: %d\n",
				 F_DECODE(wb_msg->cmd_word, MTX_MSG_CORE),
				 hw_fifo_producer & 0x1f,
				 wb_msg->cmd_word,
				 wb_msg->coded_package_consumed_idx);
		}
#endif

		/* If corresponding socket still exists, call the callback */
		if (ctx->socks[conn_id]) {
			wbfifo_add(ctx->socks[conn_id], wb_msg);
			schedule_work(ctx->socks[conn_id]->work);
		}

		/* Activate corresponding FIFO
		 * proceed to the next one
		 */
		hw_fifo_consumer++;

		if (hw_fifo_consumer == WB_FIFO_SIZE)
			hw_fifo_consumer = 0;

		comm_set_consumer(ctx, hw_fifo_consumer);

		/*
		 * We need to update the producer because we might have received a new
		 * message meanwhile. This new message won't trigger an interrupt and
		 * consequently will be lost till another message arrives
		 */
		hw_fifo_producer = comm_get_producer(ctx);
	}

	return IMG_SUCCESS;
}

/*
 * topazdd_threaded_isr
 */
unsigned char topazdd_threaded_isr(void *inst_data)
{
	struct topaz_dev_ctx *ctx = *(struct topaz_dev_ctx **)inst_data;

	/* If interrupts not defined then...*/
	if (!ctx || !ctx->initialized)
		return FALSE;

	/* Now dispatch the messages */
	comm_dispatch_in_msg(ctx);

	/* Signal this interrupt has been handled...*/
	return TRUE;
}

/*
 * topazdd_isr
 */
irqreturn_t topazdd_isr(void *inst_data)
{
	unsigned int reg;
	unsigned int mmu_fault_mask = MASK_TOPAZHP_TOP_CR_INT_STAT_MMU_FAULT;

	struct topaz_dev_ctx *ctx = *(struct topaz_dev_ctx **)inst_data;

	/* More requesters with topaz hp */
	mmu_fault_mask |= MASK_TOPAZHP_TOP_CR_INTCLR_MMU_FAULT_B;

	/* If interrupts not defined then...*/
	if (!ctx || !ctx->initialized)
		return IRQ_NONE;

	/* read device interrupt status */
	reg = VXE_RD_REG32(ctx->multi_core_mem_addr, TOPAZHP_TOP_CR_MULTICORE_INT_STAT);

	/* if interrupts enabled and fired...*/
	if (((reg & MASK_TOPAZHP_TOP_CR_INT_STAT_MTX) == (MASK_TOPAZHP_TOP_CR_INT_STAT_MTX))) {
		/* Clear interrupt source...*/
		topazdd_int_clear(ctx, MASK_TOPAZHP_TOP_CR_INTCLR_MTX);

		/* Signal this interrupt has been handled...*/
		return IRQ_WAKE_THREAD;
	}

	/* if page fault ever happenned */
	if (reg & (mmu_fault_mask)) {
		static unsigned char dump_once = TRUE;

		if (dump_once) {
			VXE_WR_REG32(ctx->multi_core_mem_addr,
				     TOPAZHP_TOP_CR_MULTICORE_HOST_INT_ENAB, 0);

			dump_once = FALSE; /* only on first page fault for readability */
		}

		/* Clear interrupt source...*/
		topazdd_int_clear(ctx, mmu_fault_mask);

		/* IT served, we might never reach that point on kernel crashes */
		return IRQ_HANDLED;
	}

	/* Signal not this device...*/
	return IRQ_NONE;
}
