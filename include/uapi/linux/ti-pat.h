/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * TI PAT mapped DMA-BUF memory exporter UAPI
 *
 * Copyright (C) 2018-2019 Texas Instruments Incorporated - http://www.ti.com/
 *	Andrew F. Davis <afd@ti.com>
 */

#ifndef _UAPI_LINUX_TI_PAT_H
#define _UAPI_LINUX_TI_PAT_H

#include <linux/ioctl.h>
#include <linux/types.h>

/**
 * DOC: TI PAT Userspace API
 *
 * create a client by opening /dev/ti-pat
 * most operations handled via following ioctls
 */

/**
 * struct ti_pat_allocation_data - metadata passed from userspace for allocations
 * @fd:			populated with DMA-BUF FD for this allocation
 * @flags:		flags for the allocation
 *
 * Provided by userspace as an argument to the ioctl
 */
struct ti_pat_export_data {
	__u32 fd;
	__u32 flags;
};

#define TI_PAT_IOC_MAGIC 'P'

/**
 * DOC: TI_PAT_IOC_EXPORT - Re-export DMA-BUF through TI PAT
 *
 * Takes an ti_pat_export_data struct and returns it with the fd field
 * populated with the DMA-BUF handle for the new export.
 */
#define TI_PAT_IOC_EXPORT _IOWR(TI_PAT_IOC_MAGIC, 0, struct ti_pat_export_data)

#endif /* _UAPI_LINUX_TI_PAT_H */
