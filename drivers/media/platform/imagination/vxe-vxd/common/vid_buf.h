/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Low-level VXD interface component
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Angela Stegmaier <angelabaker@ti.com>
 *
 * Re-written for upstream
 *	Sidraya Jayagond <sidraya.bj@pathpartnertech.com>
 */

#ifndef _VID_BUF_H
#define _VID_BUF_H

/*
 * struct vidio_ddbufinfo - contains information about virtual address
 * @buf_size: the size of the buffer (in bytes).
 * @cpu_virt: the cpu virtual address  (mapped into the local cpu mmu)
 * @dev_virt: device virtual address (pages mapped into IMG H/W mmu)
 * @hndl_memory: handle to device mmu mapping
 * @buff_id: buffer id used in communication with interface
 * @is_internal: true, if the buffer is allocated internally
 * @ref_count: reference count (number of users)
 * @kmstr_id: stream id
 * @core_id: core id
 */
struct vidio_ddbufinfo {
	unsigned int buf_size;
	void *cpu_virt;
	unsigned int dev_virt;
	void *hndl_memory;
	unsigned int buff_id;
	unsigned int is_internal;
	unsigned int ref_count;
	unsigned int kmstr_id;
	unsigned int core_id;
};

#endif /* _VID_BUF_H */
