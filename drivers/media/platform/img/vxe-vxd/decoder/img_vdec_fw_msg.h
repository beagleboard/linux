/* SPDX-License-Identifier: GPL-2.0 */
/*
 * IMG VDEC firmware messages
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

#ifndef _IMG_VDEC_FW_MSG_H
#define _IMG_VDEC_FW_MSG_H

#include <linux/types.h>

/* FW_DEVA_COMPLETED     ERROR_FLAGS */
#define FW_DEVA_COMPLETED_ERROR_FLAGS_TYPE              unsigned short
#define FW_DEVA_COMPLETED_ERROR_FLAGS_MASK		(0xFFFF)
#define FW_DEVA_COMPLETED_ERROR_FLAGS_SHIFT		(0)
#define FW_DEVA_COMPLETED_ERROR_FLAGS_OFFSET		(0x000C)

/* FW_DEVA_COMPLETED     NUM_BEWDTS */
#define FW_DEVA_COMPLETED_NUM_BEWDTS_TYPE               unsigned int
#define FW_DEVA_COMPLETED_NUM_BEWDTS_MASK               (0xFFFFFFFF)
#define FW_DEVA_COMPLETED_NUM_BEWDTS_SHIFT              (0)
#define FW_DEVA_COMPLETED_NUM_BEWDTS_OFFSET		(0x0010)

/* FW_DEVA_COMPLETED     NUM_MBSDROPPED */
#define FW_DEVA_COMPLETED_NUM_MBSDROPPED_TYPE           unsigned int
#define FW_DEVA_COMPLETED_NUM_MBSDROPPED_MASK           (0xFFFFFFFF)
#define FW_DEVA_COMPLETED_NUM_MBSDROPPED_SHIFT          (0)
#define FW_DEVA_COMPLETED_NUM_MBSDROPPED_OFFSET		(0x0014)

/* FW_DEVA_COMPLETED     NUM_MBSRECOVERED */
#define FW_DEVA_COMPLETED_NUM_MBSRECOVERED_TYPE         unsigned int
#define FW_DEVA_COMPLETED_NUM_MBSRECOVERED_MASK         (0xFFFFFFFF)
#define FW_DEVA_COMPLETED_NUM_MBSRECOVERED_SHIFT        (0)
#define FW_DEVA_COMPLETED_NUM_MBSRECOVERED_OFFSET	(0x0018)

/* FW_DEVA_PANIC     ERROR_INT */
#define FW_DEVA_PANIC_ERROR_INT_TYPE            unsigned int
#define FW_DEVA_PANIC_ERROR_INT_MASK            (0xFFFFFFFF)
#define FW_DEVA_PANIC_ERROR_INT_SHIFT           (0)
#define FW_DEVA_PANIC_ERROR_INT_OFFSET          (0x000C)

/* FW_ASSERT     FILE_NAME_HASH */
#define FW_ASSERT_FILE_NAME_HASH_TYPE           unsigned int
#define FW_ASSERT_FILE_NAME_HASH_MASK           (0xFFFFFFFF)
#define FW_ASSERT_FILE_NAME_HASH_SHIFT          (0)
#define FW_ASSERT_FILE_NAME_HASH_OFFSET         (0x0004)

/* FW_ASSERT     FILE_LINE */
#define FW_ASSERT_FILE_LINE_TYPE                unsigned int
#define FW_ASSERT_FILE_LINE_MASK                (0xFFFFFFFE)
#define FW_ASSERT_FILE_LINE_SHIFT               (1)
#define FW_ASSERT_FILE_LINE_OFFSET              (0x0008)

/* FW_SO     TASK_NAME */
#define FW_SO_TASK_NAME_TYPE            unsigned int
#define FW_SO_TASK_NAME_MASK            (0xFFFFFFFF)
#define FW_SO_TASK_NAME_SHIFT           (0)
#define FW_SO_TASK_NAME_OFFSET          (0x0004)

/* FW_DEVA_GENMSG     TRANS_ID */
#define FW_DEVA_GENMSG_TRANS_ID_TYPE            unsigned int
#define FW_DEVA_GENMSG_TRANS_ID_MASK            (0xFFFFFFFF)
#define FW_DEVA_GENMSG_TRANS_ID_SHIFT           (0)
#define FW_DEVA_GENMSG_TRANS_ID_OFFSET          (0x0008)

/* FW_DEVA_GENMSG     MSG_TYPE */
#define FW_DEVA_GENMSG_MSG_TYPE_TYPE            unsigned char
#define FW_DEVA_GENMSG_MSG_TYPE_MASK            (0xFF)
#define FW_DEVA_GENMSG_MSG_TYPE_SHIFT           (0)
#define FW_DEVA_GENMSG_MSG_TYPE_OFFSET          (0x0001)

/* FW_DEVA_SIGNATURES     SIGNATURES */
#define FW_DEVA_SIGNATURES_SIGNATURES_OFFSET            (0x0010)

/* FW_DEVA_SIGNATURES     MSG_SIZE */
#define FW_DEVA_SIGNATURES_MSG_SIZE_TYPE                unsigned char
#define FW_DEVA_SIGNATURES_MSG_SIZE_MASK                (0x7F)
#define FW_DEVA_SIGNATURES_MSG_SIZE_SHIFT               (0)
#define FW_DEVA_SIGNATURES_MSG_SIZE_OFFSET              (0x0000)

/* FW_DEVA_CONTIGUITY_WARNING     BEGIN_MB_NUM */
#define FW_DEVA_SIGNATURES_SIZE         (20)

/* FW_DEVA_SIGNATURES     SIGNATURE_SELECT */
#define FW_DEVA_SIGNATURES_SIGNATURE_SELECT_TYPE                unsigned int
#define FW_DEVA_SIGNATURES_SIGNATURE_SELECT_MASK                (0xFFFFFFFF)
#define FW_DEVA_SIGNATURES_SIGNATURE_SELECT_SHIFT               (0)
#define FW_DEVA_SIGNATURES_SIGNATURE_SELECT_OFFSET              (0x000C)

/* FW_DEVA_GENMSG     TRANS_ID */
#define FW_DEVA_DECODE_SIZE             (52)

/* FW_DEVA_DECODE     CTRL_ALLOC_ADDR */
#define FW_DEVA_DECODE_CTRL_ALLOC_ADDR_TYPE             unsigned int
#define FW_DEVA_DECODE_CTRL_ALLOC_ADDR_MASK             (0xFFFFFFFF)
#define FW_DEVA_DECODE_CTRL_ALLOC_ADDR_SHIFT            (0)
#define FW_DEVA_DECODE_CTRL_ALLOC_ADDR_OFFSET           (0x0010)

/* FW_DEVA_DECODE     BUFFER_SIZE */
#define FW_DEVA_DECODE_BUFFER_SIZE_TYPE         unsigned short
#define FW_DEVA_DECODE_BUFFER_SIZE_MASK         (0xFFFF)
#define FW_DEVA_DECODE_BUFFER_SIZE_SHIFT	(0)
#define FW_DEVA_DECODE_BUFFER_SIZE_OFFSET	(0x000E)

/* FW_DEVA_DECODE     OPERATING_MODE */
#define FW_DEVA_DECODE_OPERATING_MODE_TYPE              unsigned int
#define FW_DEVA_DECODE_OPERATING_MODE_MASK              (0xFFFFFFFF)
#define FW_DEVA_DECODE_OPERATING_MODE_OFFSET            (0x0018)
#define FW_DEVA_DECODE_OPERATING_MODE_SHIFT             (0)

/* FW_DEVA_DECODE     FLAGS */
#define FW_DEVA_DECODE_FLAGS_TYPE               unsigned short
#define FW_DEVA_DECODE_FLAGS_MASK               (0xFFFF)
#define FW_DEVA_DECODE_FLAGS_SHIFT              (0)
#define FW_DEVA_DECODE_FLAGS_OFFSET             (0x000C)

/* FW_DEVA_DECODE     VDEC_FLAGS */
#define FW_DEVA_DECODE_VDEC_FLAGS_TYPE          unsigned char
#define FW_DEVA_DECODE_VDEC_FLAGS_MASK          (0xFF)
#define FW_DEVA_DECODE_VDEC_FLAGS_SHIFT         (0)
#define FW_DEVA_DECODE_VDEC_FLAGS_OFFSET	(0x001E)

/* FW_DEVA_DECODE     GENC_ID */
#define FW_DEVA_DECODE_GENC_ID_TYPE             unsigned int
#define FW_DEVA_DECODE_GENC_ID_MASK             (0xFFFFFFFF)
#define FW_DEVA_DECODE_GENC_ID_SHIFT            (0)
#define FW_DEVA_DECODE_GENC_ID_OFFSET           (0x0028)

/* FW_DEVA_DECODE     MB_LOAD */
#define FW_DEVA_DECODE_MB_LOAD_TYPE             unsigned int
#define FW_DEVA_DECODE_MB_LOAD_MASK             (0xFFFFFFFF)
#define FW_DEVA_DECODE_MB_LOAD_OFFSET           (0x0030)
#define FW_DEVA_DECODE_MB_LOAD_SHIFT            (0)
#define FW_DEVA_DECODE_FRAGMENT_SIZE            (16)

/* FW_DEVA_DECODE     STREAMID */
#define FW_DEVA_DECODE_STREAMID_TYPE            unsigned char
#define FW_DEVA_DECODE_STREAMID_MASK            (0xFF)
#define FW_DEVA_DECODE_STREAMID_OFFSET          (0x001F)
#define FW_DEVA_DECODE_STREAMID_SHIFT           (0)

/* FW_DEVA_DECODE     EXT_STATE_BUFFER */
#define FW_DEVA_DECODE_EXT_STATE_BUFFER_TYPE            unsigned int
#define FW_DEVA_DECODE_EXT_STATE_BUFFER_MASK            (0xFFFFFFFF)
#define FW_DEVA_DECODE_EXT_STATE_BUFFER_OFFSET          (0x0020)
#define FW_DEVA_DECODE_EXT_STATE_BUFFER_SHIFT           (0)

/* FW_DEVA_DECODE     MSG_ID */
#define FW_DEVA_DECODE_MSG_ID_TYPE              unsigned short
#define FW_DEVA_DECODE_MSG_ID_MASK              (0xFFFF)
#define FW_DEVA_DECODE_MSG_ID_OFFSET            (0x0002)
#define FW_DEVA_DECODE_MSG_ID_SHIFT             (0)

/* FW_DEVA_DECODE     TRANS_ID */
#define FW_DEVA_DECODE_TRANS_ID_TYPE            unsigned int
#define FW_DEVA_DECODE_TRANS_ID_MASK            (0xFFFFFFFF)
#define FW_DEVA_DECODE_TRANS_ID_OFFSET          (0x0008)
#define FW_DEVA_DECODE_TRANS_ID_SHIFT           (0)

/* FW_DEVA_DECODE     TILE_CFG */
#define FW_DEVA_DECODE_TILE_CFG_TYPE            unsigned int
#define FW_DEVA_DECODE_TILE_CFG_MASK            (0xFFFFFFFF)
#define FW_DEVA_DECODE_TILE_CFG_OFFSET          (0x0024)
#define FW_DEVA_DECODE_TILE_CFG_SHIFT           (0)

/* FW_DEVA_GENMSG     MSG_SIZE */
#define FW_DEVA_GENMSG_MSG_SIZE_TYPE            unsigned char
#define FW_DEVA_GENMSG_MSG_SIZE_MASK            (0x7F)
#define FW_DEVA_GENMSG_MSG_SIZE_OFFSET          (0x0000)
#define FW_DEVA_GENMSG_MSG_SIZE_SHIFT           (0)

/* FW_DEVA_DECODE_FRAGMENT     CTRL_ALLOC_ADDR */
#define FW_DEVA_DECODE_FRAGMENT_CTRL_ALLOC_ADDR_TYPE            unsigned int
#define FW_DEVA_DECODE_FRAGMENT_CTRL_ALLOC_ADDR_MASK            (0xFFFFFFFF)
#define FW_DEVA_DECODE_FRAGMENT_CTRL_ALLOC_ADDR_OFFSET          (0x000C)
#define FW_DEVA_DECODE_FRAGMENT_CTRL_ALLOC_ADDR_SHIFT           (0)

/* FW_DEVA_DECODE_FRAGMENT     BUFFER_SIZE */
#define FW_DEVA_DECODE_FRAGMENT_BUFFER_SIZE_TYPE                unsigned short
#define FW_DEVA_DECODE_FRAGMENT_BUFFER_SIZE_MASK                (0xFFFF)
#define FW_DEVA_DECODE_FRAGMENT_BUFFER_SIZE_OFFSET              (0x000A)
#define FW_DEVA_DECODE_FRAGMENT_BUFFER_SIZE_SHIFT               (0)

#endif /* _IMG_VDEC_FW_MSG_H */
