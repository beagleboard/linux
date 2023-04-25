/* SPDX-License-Identifier: GPL-2.0 */
/*
 * VXD PVDEC Private header file
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Amit Makani <amit.makani@ti.com>
 *
 * Re-written for upstreamimg
 *	Sidraya Jayagond <sidraya.bj@pathpartnertech.com>
 *	Prashanth Kumar Amai <prashanth.ka@pathpartnertech.com>
 */

#ifndef _VXD_PVDEC_PRIV_H
#define _VXD_PVDEC_PRIV_H
#include <linux/interrupt.h>

#include "img_dec_common.h"
#include "vxd_pvdec_regs.h"
#include "vxd_dec.h"

#ifdef ERROR_RECOVERY_SIMULATION
/* kernel object used to debug. Declared in v4l2_int.c */
extern struct kobject *vxd_dec_kobject;
extern int disable_fw_irq_value;
extern int g_module_irq;
#endif

struct vxd_boot_poll_params {
	unsigned int msleep_cycles;
};

struct vxd_ena_params {
	struct vxd_boot_poll_params boot_poll;

	unsigned long fw_buf_size;
	unsigned int fw_buf_virt_addr;
	/*
	 * VXD's MMU virtual address of a firmware
	 * buffer.
	 */
	unsigned int ptd; /* Shifted physical address of PTD */

	/* Required for firmware upload via registers. */
	struct {
		const unsigned char *buf; /* Firmware blob buffer */

	} regs_data;

	struct {
		unsigned secure : 1;        /* Secure flow indicator. */
		unsigned wait_dbg_fifo : 1; /*
					     * Indicates that fw shall use
					     * blocking mode when putting logs
					     * into debug fifo
					     */
	};

	/* Structure containing memory staller configuration */
	struct {
		unsigned int *data;          /* Configuration data array */
		unsigned char size;            /* Configuration size in dwords */

	} mem_staller;

	unsigned int fwwdt_ms;    /* Firmware software watchdog timeout value */

	unsigned int crc;         /* HW signatures to be enabled by firmware */
	unsigned int rendec_addr; /* VXD's virtual address of a rendec buffer */
	unsigned short rendec_size; /* Size of a rendec buffer in 4K pages */
};

int vxd_pvdec_init(const void *dev, void __iomem *reg_base);

int vxd_pvdec_ena(const void *dev, void __iomem *reg_base,
		  struct vxd_ena_params *ena_params, struct vxd_fw_hdr *hdr,
		  unsigned int *freq_khz);

int vxd_pvdec_dis(const void *dev, void __iomem *reg_base);

int vxd_pvdec_mmu_flush(const void *dev, void __iomem *reg_base);

int vxd_pvdec_send_msg(const void *dev, void __iomem *reg_base,
		       unsigned int *msg, unsigned long msg_size, unsigned short msg_id,
		       struct vxd_dev *ctx);

int vxd_pvdec_pend_msg_info(const void *dev, void __iomem *reg_base,
			    unsigned long *size, unsigned short *msg_id,
			    unsigned char *not_last_msg);

int vxd_pvdec_recv_msg(const void *dev, void __iomem *reg_base,
		       unsigned int *buf, unsigned long buf_size, struct vxd_dev *ctx);

int vxd_pvdec_check_fw_status(const void *dev, void __iomem *reg_base);

unsigned long vxd_pvdec_peek_mtx_fifo(const void *dev,
				      void __iomem *reg_base);

unsigned long vxd_pvdec_read_mtx_fifo(const void *dev, void __iomem *reg_base,
				      unsigned int *buf, unsigned long size);

irqreturn_t vxd_pvdec_clear_int(void __iomem *reg_base, unsigned int *irq_status);

int vxd_pvdec_check_irq(const void *dev, void __iomem *reg_base,
			unsigned int irq_status);

int vxd_pvdec_msg_fit(const void *dev, void __iomem *reg_base,
		      unsigned long msg_size);

void vxd_pvdec_get_state(const void *dev, void __iomem *reg_base,
			 unsigned int num_pipes, struct vxd_hw_state *state);

int vxd_pvdec_get_props(const void *dev, void __iomem *reg_base,
			struct vxd_core_props *props);

unsigned long vxd_pvdec_get_dbg_fifo_size(void __iomem *reg_base);

int vxd_pvdec_dump_mtx_ram(const void *dev, void __iomem *reg_base,
			   unsigned int addr, unsigned int count, unsigned int *buf);

int vxd_pvdec_dump_mtx_status(const void *dev, void __iomem *reg_base,
			      unsigned int *array, unsigned int array_size);

#endif /* _VXD_PVDEC_PRIV_H */
