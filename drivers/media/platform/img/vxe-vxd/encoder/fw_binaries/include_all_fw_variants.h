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

#ifndef __INCLUDE_ALL_VARIANTS_INC_INCLUDED__
#define __INCLUDE_ALL_VARIANTS_INC_INCLUDED__

#define INCLUDE_ALL_VARIANTS_TEMPLATE_VERSION (1)

#define FW_BIN_FORMAT_VERSION (2)

struct IMG_COMPILED_FW_BIN_RECORD {
	unsigned int text_size, data_size;
	unsigned int data_origin, text_origin;
	unsigned int text_reloc_size, data_reloc_size;

	unsigned int pipes;
	unsigned char *fmt, *rc_mode;
	unsigned int formats_mask, hw_config;

	unsigned int int_define_cnt;
	unsigned char **int_define_names;
	unsigned int *int_defines;

	unsigned int *text, *data;
	unsigned int *text_reloc, *data_reloc;
	unsigned int *text_reloc_full_addr, *text_reloc_type;
};

#include "ALL_CODECS_FW_ALL_pipes_2_contexts_8_hwconfig_1_bin.c"

unsigned int all_fw_binaries_cnt = 1;
struct IMG_COMPILED_FW_BIN_RECORD *all_fw_binaries[] = {
	&simg_compiled_all_codecs_fw_all_pipes_2_contexts_8_hwconfig_1,
};

#endif
