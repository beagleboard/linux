/* SPDX-License-Identifier: ((GPL-2.0-only WITH Linux-syscall-note) OR BSD-3-Clause) */
/*
 * Copyright (C) 2015-2021 Texas Instruments Incorporated - https://www.ti.com/
 */

#ifndef _UAPI_LINUX_KEYSTONE_REMOTEPROC_H_
#define _UAPI_LINUX_KEYSTONE_REMOTEPROC_H_

#include <linux/types.h>
#include <linux/ioctl.h>

/**
 * enum keystone_rproc_state - keystone remoteproc state setting values
 *
 * @KEYSTONE_RPROC_OFFLINE: request to configure the remoteproc into an offline
 *			    state
 * @KEYSTONE_RPROC_RUNNING: request to configure the remoteproc into a ready
 *			    state
 */
enum keystone_rproc_state {
	KEYSTONE_RPROC_OFFLINE,
	KEYSTONE_RPROC_RUNNING,
};

/**
 * struct keystone_rproc_set_state_params - keystone remoteproc set state
 *					    parameters structure
 *
 * @state: enumerated state value to set
 * @boot_addr: boot address/entry point for the remote processor
 */
struct keystone_rproc_set_state_params {
	enum keystone_rproc_state state;
	__u32 boot_addr;
};

/* Macros used within mmap function */
#define KEYSTONE_RPROC_UIO_MAP_INDEX_MASK	(0x7)
#define KEYSTONE_RPROC_UIO_MAP_OFFSET_SHIFT	(3)

/* IOCTL definitions */
#define KEYSTONE_RPROC_IOC_MAGIC		'I'
#define KEYSTONE_RPROC_IOC_SET_RSC_TABLE	_IOW(KEYSTONE_RPROC_IOC_MAGIC, \
						0, void *)
#define KEYSTONE_RPROC_IOC_SET_STATE	_IOW(KEYSTONE_RPROC_IOC_MAGIC, \
					1, \
					struct keystone_rproc_set_state_params)
#define KEYSTONE_RPROC_IOC_SET_LOADED_RSC_TABLE _IOW(KEYSTONE_RPROC_IOC_MAGIC, \
						2, __u32)
#define KEYSTONE_RPROC_IOC_DSP_RESET		_IO(KEYSTONE_RPROC_IOC_MAGIC, 3)
#define KEYSTONE_RPROC_IOC_DSP_BOOT		_IOW(KEYSTONE_RPROC_IOC_MAGIC, \
						4, __u32)

#define KEYSTONE_RPROC_IOC_MAXNR		(5)

#endif /* _UAPI_LINUX_KEYSTONE_REMOTEPROC_H_ */
