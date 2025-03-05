/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * This header provides headers for Timesync Router
 *
 * Copyright (C) 2025 Texas Instruments Incorporated - https://www.ti.com/
 */

#ifndef DTS_ARM64_TI_K3_TIMESYNC_ROUTER
#define DTS_ARM64_TI_K3_TIMESYNC_ROUTER

/*
 * The value of the input to be mapped to an output has to be written in
 * the register corresponding to the output.
 * INT_ENABLE which is BIT(16) i.e. 0x10000 has to be set in addition to
 * writing the value of the input in order to begin generating the output
 * signal.
 */
#define K3_TS_OFFSET(output_index, mask, input_index)   (((0x4 + ((output_index) * 4)))) ((mask)) ((0x10000 | (input_index)))

#endif /* DTS_ARM64_TI_K3_TIMESYNC_ROUTER */
