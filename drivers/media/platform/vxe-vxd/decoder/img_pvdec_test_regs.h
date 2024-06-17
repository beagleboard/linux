/* SPDX-License-Identifier: GPL-2.0 */
/*
 * IMG PVDEC test Registers
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Amit Makani <amit.makani@ti.com>
 *
 * Re-written for upstreamimg
 *	Sidraya Jayagond <sidraya.bj@pathpartnertech.com>
 */

#ifndef _IMG_PVDEC_TEST_REGS_H
#define _IMG_PVDEC_TEST_REGS_H

/* PVDEC_TEST, RAND_STL_MEM_RDATA_CONFIG, STALL_ENABLE_MEM_RDATA */
#define PVDEC_TEST_MEM_READ_LATENCY_OFFSET              (0x00F0)

/* PVDEC_TEST, MEM_READ_LATENCY, READ_RESPONSE_RAND_LATENCY */
#define PVDEC_TEST_MEM_WRITE_RESPONSE_LATENCY_OFFSET            (0x00F4)

/* PVDEC_TEST, MEM_WRITE_RESPONSE_LATENCY, WRITE_RESPONSE_RAND_LATENCY */
#define PVDEC_TEST_MEM_CTRL_OFFSET              (0x00F8)

/* PVDEC_TEST, RAND_STL_MEM_WDATA_CONFIG, STALL_ENABLE_MEM_WDATA */
#define PVDEC_TEST_RAND_STL_MEM_WRESP_CONFIG_OFFSET             (0x00E8)

/* PVDEC_TEST, RAND_STL_MEM_WRESP_CONFIG, STALL_ENABLE_MEM_WRESP */
#define PVDEC_TEST_RAND_STL_MEM_RDATA_CONFIG_OFFSET             (0x00EC)

/* PVDEC_TEST, MEMORY_BUS2_MONITOR_2, BUS2_ADDR */
#define PVDEC_TEST_RAND_STL_MEM_CMD_CONFIG_OFFSET               (0x00E0)

/* PVDEC_TEST, RAND_STL_MEM_CMD_CONFIG, STALL_ENABLE_MEM_CMD */
#define PVDEC_TEST_RAND_STL_MEM_WDATA_CONFIG_OFFSET             (0x00E4)

#endif /* _IMG_PVDEC_TEST_REGS_H */
