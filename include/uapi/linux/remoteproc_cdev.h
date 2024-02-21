/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * IOCTLs for Remoteproc's character device interface.
 *
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 */

#ifndef _UAPI_REMOTEPROC_CDEV_H_
#define _UAPI_REMOTEPROC_CDEV_H_

#include <linux/ioctl.h>
#include <linux/types.h>

#define RPROC_MAGIC	0xB7

/*
 * The RPROC_SET_SHUTDOWN_ON_RELEASE ioctl allows to enable/disable the shutdown of a remote
 * processor automatically when the controlling userpsace closes the char device interface.
 *
 * input parameter: integer
 *   0		: disable automatic shutdown
 *   other	: enable automatic shutdown
 */
#define RPROC_SET_SHUTDOWN_ON_RELEASE _IOW(RPROC_MAGIC, 1, __s32)

/*
 * The RPROC_GET_SHUTDOWN_ON_RELEASE ioctl gets information about whether the automatic shutdown of
 * a remote processor is enabled or disabled when the controlling userspace closes the char device
 * interface.
 *
 * output parameter: integer
 *   0		: automatic shutdown disable
 *   other	: automatic shutdown enable
 */
#define RPROC_GET_SHUTDOWN_ON_RELEASE _IOR(RPROC_MAGIC, 2, __s32)

/**
 * struct rproc_dma_buf_attach_data - metadata passed from userspace
 * @fd:		DMA-BUF fd
 * @da:		populated with device address of DMA-BUF
 */
struct rproc_dma_buf_attach_data {
	__u32 fd;
	__u64 da;
};

/**
 * DOC: RPROC_IOC_DMA_BUF_ATTACH - Attach and map DMA-BUF a remote processor
 *
 * Takes a rproc_dma_buf_data struct containing a fd for a physicaly contigous
 * buffer. Pins this buffer and populates phys field with the device address.
 */
#define RPROC_IOC_DMA_BUF_ATTACH _IOWR(RPROC_MAGIC, 0, struct rproc_dma_buf_attach_data)

#endif
