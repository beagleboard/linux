/* SPDX-License-Identifier: GPL-2.0 */
/*
 * target interface header
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

#if !defined(__TARGET_H__)
#define __TARGET_H__

#include <linux/types.h>

#define TARGET_NO_IRQ   (999) /* Interrupt number when no interrupt exists */

/*
 * The memory space types
 */
enum mem_space_type {
	MEMSPACE_REGISTER,  /* Memory space is mapped to device registers */
	MEMSPACE_MEMORY,    /* Memory space is mapped to device memory */
	MEMSPACE_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * This structure contains all information about a device register
 */
struct mem_space_reg {
	unsigned long long addr;     /* Base address of device registers */
	unsigned int size;     /* Size of device register block */
	unsigned int intr_num; /* The interrupt number */
};

/*
 * This structure contains all information about a device memory region
 */
struct mem_space_mem {
	unsigned long long addr;       /* Base address of memory region */
	unsigned long long size;       /* Size of memory region */
	unsigned long long guard_band; /* Memory guard band */
};

/*
 * This structure contains all information about the device memory space
 */
struct mem_space {
	unsigned char *name;               /* Memory space name */
	enum mem_space_type type;          /* Memory space type */
	union {
		struct mem_space_reg reg;  /* Device register info */
		struct mem_space_mem mem;  /* Device memory region info */
	};

	unsigned long long cpu_addr;      /* Cpu KM address for the mem space */
};

struct target_config {
	unsigned int num_mem_spaces;
	struct mem_space *mem_spaces;
};

#endif /* __TARGET_H__    */
