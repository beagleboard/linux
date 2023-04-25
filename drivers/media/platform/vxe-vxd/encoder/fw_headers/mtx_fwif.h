/* SPDX-License-Identifier: GPL-2.0 */
/*
 * firmware header
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

#ifndef _MTX_FWIF_H_
#define _MTX_FWIF_H_

#include "vxe_common.h"
#include "topazscfwif.h"

//#define VXE_MEASURE_MTX_CLK_FREQ

/*
 * enum describing the MTX load method
 */
enum mtx_load_method {
	MTX_LOADMETHOD_NONE = 0,        /* don't load MTX code */
	MTX_LOADMETHOD_BACKDOOR,        /* backdoor - writes MTX load data direct to out.res */
	MTX_LOADMETHOD_REGIF,           /* load mtx code via register interface */
	MTX_LOADMETHOD_DMA,             /* load mtx code via DMA */
	MTX_LOADMETHOD_FORCE32BITS = 0x7FFFFFFFU

};

/*
 * defines that should come from auto generated headers
 */
#define MTX_DMA_MEMORY_BASE (0x82880000)
#define PC_START_ADDRESS    (0x80900000)

#define MTX_CORE_CODE_MEM   (0x10)
#define MTX_CORE_DATA_MEM   (0x18)

#define MTX_PC              (0x05)

/*
 * MTX Firmware Context Structure
 */

/*
 * struct img_fw_int_defines_table - contains info for the fw int defines
 *
 * @length: length of the table
 * @names: array of names of entries
 * @values: array of values of entries
 */
struct img_fw_int_defines_table {
	unsigned int length;
	unsigned char **names;
	unsigned int *values;
};

/*
 * struct img_fw_context - contains info for the context of the loaded firmware
 *
 * @initialized: TRUE if MTX core is initialized
 * @populated: TRUE if MTX firmware context had been populated with data
 * @active_ctx_mask: A bit mask of active encode contexts in the firmware
 * @dev_ctx: Pointer to the device context
 * @load_method: method used to load this MTX
 * @supported_codecs: Codec mask
 * @mtx_debug_val: Value in MTX Debug register (for RAM config)
 * @mtx_ram_size: Size of MTX RAM
 * @mtx_bank_size: Size of MTX RAM banks
 * @mtx_reg_mem_space_addr: Memspace ID for MTX registers
 * @topaz_reg_mem_space_addr: Memspace ID for TOPAZ registers
 * @topaz_multicore_reg_addr: Memspace ID for TOPAZ multicore control registers
 * @core_rev: Hardware core revision ID
 * @core_des1: Hardware core designer (feature bits)
 * @drv_has_mtx_ctrl: TRUE if driver (not DASH) has control of MTX
 * @access_control: Use to get read/write access to MTX
 * @hw_num_pipes: Number of pipes available in hardware
 * @num_pipes: Number of pipes supported by firmware
 * @num_contexts: Number of contexts supported by firmware
 * @mtx_context_data_copy: Copy of MTX Context Data during hibernate
 * @mtx_reg_copy: Copy of MTX Register block during hibernate
 * @mtx_topaz_fw_text_size: Size of MTX Firmware Text Section in words
 * @mtx_topaz_fw_text: Pointer to MTX Firmware Text Section
 * @mtx_topaz_fw_data_size: Size of MTX Firmware Data Section in words
 * @mtx_topaz_fw_data: Pointer to MTX Firmware Data Section
 * @mtx_topaz_fw_data_origin: Offset to location of Data section
 * @int_defines: table of int defines
 */
struct img_fw_context {
	unsigned short initialized;
	unsigned short populated;
	unsigned char  active_ctx_mask;

	void *dev_ctx;

	enum mtx_load_method load_method;

	unsigned int supported_codecs;

	unsigned int mtx_debug_val;
	unsigned int mtx_ram_size;
	unsigned int mtx_bank_size;

	void  *mtx_reg_mem_space_addr;
	void  *topaz_reg_mem_space_addr[TOPAZHP_MAX_NUM_PIPES];
	void  *topaz_multicore_reg_addr;
	unsigned int core_rev;
	unsigned int core_des1;

	unsigned short drv_has_mtx_ctrl;
	unsigned int access_control;

	unsigned int hw_num_pipes;
	unsigned int num_pipes;
	unsigned int num_contexts;

	struct vidio_ddbufinfo *mtx_context_data_copy[TOPAZHP_MAX_POSSIBLE_STREAMS];
	unsigned int *mtx_reg_copy;

	unsigned int mtx_topaz_fw_text_size;
	unsigned int *mtx_topaz_fw_text;

	unsigned int mtx_topaz_fw_data_size;
	unsigned int *mtx_topaz_fw_data;

	unsigned int mtx_topaz_fw_data_origin;

	struct img_fw_int_defines_table int_defines;
};

/*
 * Populates MTX context structure
 * @param    codec         : version of codec specific firmware to associate with this MTX
 * @param    fw_ctx       : Output context
 * @return   int : Standard IMG_ERRORCODE
 */
int mtx_populate_fw_ctx(enum img_codec codec,
			struct img_fw_context  *fw_ctx);

/*
 * Initialise the hardware using given (populated) MTX context structure
 * @param    fw_ctx       : Pointer to the context of the target MTX
 * @return   None
 */
void mtx_initialize(void *dev_ctx, struct img_fw_context *fw_ctx);

/*
 * Return the integer define used to compile given version of firmware.
 * @param    fw_ctx       : Pointer to the context of the target MTX
 * @param    name       : Name of a define (string)
 * @return   Value of define or -1 if not found.
 */
int mtx_get_fw_config_int(struct img_fw_context const * const fw_ctx,
			  unsigned char const * const name);

/*
 * Load text and data sections onto an MTX.
 * @param    fw_ctx         : Pointer to the context of the target MTX
 * @param    load_method      : Method to use for loading code
 * @return   None
 */
void mtx_load(void  *dev_ctx, struct img_fw_context *fw_ctx,
	      enum mtx_load_method load_method);

/*
 * Deinitialises MTX and MTX control structure
 */
void mtx_deinitialize(struct img_fw_context *fw_ctx);

/*
 * Saves MTX State -- Registers and Data Memory
 */
void mtx_save_state(struct img_fw_context *fw_ctx);

/*
 * Restores MTX State -- Registers and Data Memory
 */
void mtx_restore_state(void  *ctx, struct img_fw_context *fw_ctx);

/*
 * mtx_start
 */
void mtx_start(struct img_fw_context *fw_ctx);

/*
 * mtx_stop
 */
void mtx_stop(struct img_fw_context  *fw_ctx);

/*
 * Kicks MTX
 */
void mtx_kick(struct img_fw_context *fw_ctx, unsigned int kick_count);

/*
 * Waits for MTX to halt
 */
void mtx_wait_for_completion(struct img_fw_context *fw_ctx);

#endif
