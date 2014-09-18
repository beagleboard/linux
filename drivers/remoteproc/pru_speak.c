/*
 * PRU driver for TI's AM33xx series of SoCs
 *
 * Copyright (C) 2013 Pantelis Antoniou <panto@antoniou-consulting.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */


#include <linux/module.h>
#include <linux/err.h>
#include <linux/dma-mapping.h>
#include <linux/remoteproc.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/genalloc.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/pinctrl/consumer.h>
#include <linux/io.h>
#include <plat/mailbox.h>
#include <linux/virtio_ids.h>
#include <linux/elf.h>
#include <linux/byteorder/generic.h>
#include <linux/virtio.h>
#include <linux/virtio_ring.h>
#include <asm/atomic.h>

#include <linux/pwm.h>

//#include "remoteproc_internal.h"

#ifndef REMOTEPROC_INTERNAL_H
#define REMOTEPROC_INTERNAL_H

#include <linux/irqreturn.h>
#include <linux/firmware.h>

/**
 * struct pru_shm - shared memory details per PRU
 * @idx: ID of the PRU this pru_shm structure represents. [PRU0 or PRU1]
 * @vaddr : virtual address of the shared memory, for access from kernel
 * @paddr : physical address of the shared memory, for access from PRU
 * @is_valid : this is = 1 if this structure represents a valid shared memory segment
 */

struct pru_shm {
	int idx;
	void __iomem *vaddr;
	void __iomem *paddr;
	int size_in_pages;
	unsigned int is_valid :1;
};

/* max/min shared memory segments per PRU */
#define MAX_SHARED	4
#define MIN_SHARED	2

/* index for shm of botspeak code */
#define BS_CODE		0
/* index for shm of return values */
#define BS_RET		1


struct rproc;

/**
 * struct rproc_fw_ops - firmware format specific operations.
 * @find_rsc_table:	finds the resource table inside the firmware image
 * @load:		load firmeware to memory, where the remote processor
 *			expects to find it
 * @sanity_check:	sanity check the fw image
 * @get_boot_addr:	get boot address to entry point specified in firmware
 */
struct rproc_fw_ops {
	struct resource_table *(*find_rsc_table) (struct rproc *rproc,
						const struct firmware *fw,
						int *tablesz);
	int (*load)(struct rproc *rproc, const struct firmware *fw);
	int (*sanity_check)(struct rproc *rproc, const struct firmware *fw);
	u32 (*get_boot_addr)(struct rproc *rproc, const struct firmware *fw);
};

/* from remoteproc_core.c */
void rproc_release(struct kref *kref);
irqreturn_t rproc_vq_interrupt(struct rproc *rproc, int vq_id);

/* from remoteproc_virtio.c */
int rproc_add_virtio_dev(struct rproc_vdev *rvdev, int id);
void rproc_remove_virtio_dev(struct rproc_vdev *rvdev);

/* from remoteproc_debugfs.c */
void rproc_remove_trace_file(struct dentry *tfile);
struct dentry *rproc_create_trace_file(const char *name, struct rproc *rproc,
					struct rproc_mem_entry *trace);
void rproc_delete_debug_dir(struct rproc *rproc);
void rproc_create_debug_dir(struct rproc *rproc);
void rproc_init_debugfs(void);
void rproc_exit_debugfs(void);

void rproc_free_vring(struct rproc_vring *rvring);
int rproc_alloc_vring(struct rproc_vdev *rvdev, int i);

void *rproc_da_to_va(struct rproc *rproc, u64 da, int len);
int rproc_trigger_recovery(struct rproc *rproc);

static inline
int rproc_fw_sanity_check(struct rproc *rproc, const struct firmware *fw)
{
	if (rproc->fw_ops->sanity_check)
		return rproc->fw_ops->sanity_check(rproc, fw);

	return 0;
}

static inline
u32 rproc_get_boot_addr(struct rproc *rproc, const struct firmware *fw)
{
	if (rproc->fw_ops->get_boot_addr)
		return rproc->fw_ops->get_boot_addr(rproc, fw);

	return 0;
}

static inline
int rproc_load_segments(struct rproc *rproc, const struct firmware *fw)
{
	if (rproc->fw_ops->load)
		return rproc->fw_ops->load(rproc, fw);

	return -EINVAL;
}

static inline
struct resource_table *rproc_find_rsc_table(struct rproc *rproc,
				const struct firmware *fw, int *tablesz)
{
	if (rproc->fw_ops->find_rsc_table)
		return rproc->fw_ops->find_rsc_table(rproc, fw, tablesz);

	return NULL;
}

extern const struct rproc_fw_ops rproc_elf_fw_ops;

#endif /* REMOTEPROC_INTERNAL_H */


/* PRU_EVTOUT0 is halt (system call) */

/* maximum PRUs */
#define MAX_PRUS		2

/* sysevent targets (ARM + PRU) */
#define MAX_TARGETS		(1 + MAX_PRUS)

/* sysevent target ids */
#define TARGET_ARM		0
#define TARGET_PRU(x)		(1 + (x))
#define TARGET_PRU_TO_PRU_IDX(x) ({ \
	unsigned int _x = (x) - TARGET_PRU(0); \
	_x < MAX_PRUS ? _x : -1; \
	})
#define TARGET_FROM_TO_IDX(_from, _to) \
	((_from) * MAX_TARGETS + (_to))
#define TARGET_ARM_TO_PRU_IDX(x) \
	TARGET_FROM_TO_IDX(TARGET_ARM, TARGET_PRU(x))
#define TARGET_PRU_TO_ARM_IDX(x) \
	TARGET_FROM_TO_IDX(TARGET_PRU(x), TARGET_ARM)

/* maximum interrupts routed to host */
#define MAX_ARM_PRU_INTS	8

/* maximum number of sys events */
#define MAX_PRU_SYS_EVENTS	64

/* maximum number of interrupt channels */
#define MAX_PRU_CHANNELS	10

/* maximum number of host interrupts (ARM + 1 for each PRU) */
#define MIN_PRU_HOST_INT	2
#define MAX_PRU_HOST_INT	10

struct pruproc;
struct pruproc_core;

/* maximum 4 vdevs per PRU */
#define PRU_VDEV_MAX	4
#define PRU_VRING_MAX	(RVDEV_NUM_VRINGS * PRU_VDEV_MAX)

#define PRU_HALT_INSN	0x2a000000

/* down call IDs */
#define DC_PWM_CONFIG   0       /* pwm, hi, lo */
#define DC_PWM_ENABLE   1       /* pwm */
#define DC_PWM_DISABLE  2       /* pwm */
#define DC_PWM_MAX	3

/* maximum PWMs */
#define PRU_PWM_MAX	32

/* returns the initialized bin_attr struct for sysfs */
#define BIN_ATTR(_name,_mode,_size,_read,_write,_mmap) { \
       .attr = { .name  =  __stringify(_name),  .mode = _mode  },   \
       .size = _size,                                         \
       .read   = _read,                                      \
       .write  = _write,                                      \
       .mmap = _mmap,                                   \
}


struct pru_vring_info {
	struct fw_rsc_vdev_vring *rsc;
	struct vring vr;
	void __iomem *va;
	dma_addr_t pa;
	u32 da;
	struct rproc_vring *rvring;
};

/* per PRU core control structure */
struct pruproc_core {
	int idx;
	struct pruproc *pruproc;
	struct rproc *rproc;

	u32 pctrl;
	u32 pdbg;

	u32 iram, iram_sz, iram_da;		/* global, size, own */
	u32 dram, dram_sz, dram_da, dram_oda;	/* global, size, own, other */

	const char *fw_name;
	unsigned int is_elf : 1;
	u32 entry_point;

	struct resource_table *table;
	int table_size;
	/* copy of the resource table in device accessible area */
	void * __iomem dev_table_va;
	dma_addr_t dev_table_pa;

	int num_vdevs;
	struct fw_rsc_vdev *rsc_vdev[PRU_VDEV_MAX];
	int vdev_vring_start[PRU_VDEV_MAX];
	int vdev_vring_count[PRU_VDEV_MAX];

	/* total for all the vdevs */
	int num_vrings;
	struct pru_vring_info vring_info[PRU_VRING_MAX];

	/* sysevent the host generates when kicking any vring */
	int host_vring_sysev;

	/* sysevent the pru generates when kicking any vring */
	int pru_vring_sysev;

	/* boots */
	atomic_t bootcnt;

	/* downcall lock */
	struct mutex dc_lock;
	wait_queue_head_t dc_waitq;
	unsigned long dc_flags;

	/* shared memory details */
	struct pru_shm shm[MAX_SHARED];	// should I have one struct or array of these structs

#define PRU_DCF_DOWNCALL_REQ	0
#define PRU_DCF_DOWNCALL_ACK	1
#define PRU_DCF_DOWNCALL_ISSUE	2
#define PRU_DCF_DOWNCALL_DONE	3
};

struct pru_sysev_target {
	unsigned int valid : 1;	/* sysevent is valid */
	unsigned int vring : 1;	/* sysevent is vring related */
	int source;		/* 0=ARM, 1=PRU0, 2=PRU1 */
	int target;		/* 0=ARM, 1=PRU0, 2=PRU1 */
};

/* PRU control structure */
struct pruproc {
	struct platform_device *pdev;
	void __iomem *vaddr;
	dma_addr_t paddr;
	struct omap_mbox *mbox;
	struct notifier_block nb;
	u32 pintc;
	u32 pdram, pdram_sz, pdram_da;

	int num_irqs;
	int irqs[MAX_ARM_PRU_INTS];
	int events[MAX_ARM_PRU_INTS];
	int sysev_to_ch[MAX_PRU_SYS_EVENTS];
	int ch_to_host[MAX_PRU_CHANNELS];
	int target_to_sysev[MAX_TARGETS * MAX_TARGETS];

	/* map an incoming sysevent to kind and handler */
	struct pru_sysev_target sysev_to_target[MAX_PRU_SYS_EVENTS];

	/* number of prus */
	u32 num_prus;
	struct pruproc_core **pruc;
	struct pruproc_core *pru_to_pruc[MAX_PRUS];

	/* PRU clock period in ns */
	u32 clock_freq;

	/* the actual linux devices */
	struct {
		struct pwm_chip chip;
		int count;
		u32 map[PRU_PWM_MAX];	/* maximum pwm channels is 32 */
		int controller;
		u32 dc_ids[DC_PWM_MAX];
	} pwm;
};

/* global memory map (for am33xx) (almost the same as local) */
#define PRU_DATA_RAM0		0x00000
#define PRU_DATA_RAM1		0x02000
#define PRU_SHARED_DATA_RAM	0x10000
#define PRU_INTC		0x20000
#define PRU_PRU0_CONTROL	0x22000
#define PRU_PRU0_DEBUG		0x22400
#define PRU_PRU1_CONTROL	0x24000
#define PRU_PRU1_DEBUG		0x24400
#define PRU_CFG			0x26000
#define PRU_UART0		0x28000
#define PRU_IEP			0x2E000
#define PRU_ECAP0		0x30000
#define PRU_MII_RT_CFG		0x32000
#define PRU_MII_MDIO		0x32400
#define PRU_INSN_RAM0		0x34000
#define PRU_INSN_RAM1		0x38000

/* PRU CONTROL */
#define PCTRL_CONTROL		0x0000
#define  CONTROL_SOFT_RST_N	0x0001
#define  CONTROL_ENABLE		0x0002
#define  CONTROL_SLEEPING	0x0004
#define  CONTROL_COUNTER_ENABLE	0x0008
#define  CONTROL_SINGLE_STEP	0x0100
#define  CONTROL_RUNSTATE	0x4000

#define PCTRL_STATUS		0x0004
#define PCTRL_WAKEUP_EN		0x0008
#define PCTRL_CYCLE		0x000C
#define PCTRL_STALL		0x0010
#define PCTRL_CTBIR0		0x0020
#define PCTRL_CTBIR1		0x0024
#define PCTRL_CTPPR0		0x0028
#define PCTRL_CTPPR1		0x002C

/* PRU DEBUG */
#define PDBG_GPREG(x)		(0x0000 + (x) * 4)
#define PDBG_CT_REG(x)		(0x0080 + (x) * 4)

/* PRU INTC */
#define PINTC_REVID		0x0000
#define PINTC_CR		0x0004
#define PINTC_GER		0x0010
#define PINTC_GNLR		0x001C
#define PINTC_SISR		0x0020
#define PINTC_SICR		0x0024
#define PINTC_EISR		0x0028
#define PINTC_EICR		0x002C
#define PINTC_HIEISR		0x0034
#define PINTC_HIDISR		0x0038
#define PINTC_GPIR		0x0080
#define PINTC_SRSR0		0x0200
#define PINTC_SRSR1		0x0204
#define PINTC_SECR0		0x0280
#define PINTC_SECR1		0x0284
#define PINTC_ESR0		0x0300
#define PINTC_ESR1		0x0304
#define PINTC_ECR0		0x0380
#define PINTC_ECR1		0x0384
#define PINTC_CMR0		0x0400
#define PINTC_CMR1		0x0404
#define PINTC_CMR2		0x0408
#define PINTC_CMR3		0x040C
#define PINTC_CMR4		0x0410
#define PINTC_CMR5		0x0414
#define PINTC_CMR6		0x0418
#define PINTC_CMR7		0x041C
#define PINTC_CMR8		0x0420
#define PINTC_CMR9		0x0424
#define PINTC_CMR10		0x0428
#define PINTC_CMR11		0x042C
#define PINTC_CMR12		0x0430
#define PINTC_CMR13		0x0434
#define PINTC_CMR14		0x0438
#define PINTC_CMR15		0x043C
#define PINTC_HMR0		0x0800
#define PINTC_HMR1		0x0804
#define PINTC_HMR2		0x0808
#define PINTC_HIPIR0		0x0900
#define PINTC_HIPIR1		0x0904
#define PINTC_HIPIR2		0x0908
#define PINTC_HIPIR3		0x090C
#define PINTC_HIPIR4		0x0910
#define PINTC_HIPIR5		0x0914
#define PINTC_HIPIR6		0x0918
#define PINTC_HIPIR7		0x091C
#define PINTC_HIPIR8		0x0920
#define PINTC_HIPIR9		0x0924
#define PINTC_SIPR0		0x0D00
#define PINTC_SIPR1		0x0D04
#define PINTC_SITR0		0x0D80
#define PINTC_SITR1		0x0D84
#define PINTC_HINLR0		0x1100
#define PINTC_HINLR1		0x1104
#define PINTC_HINLR2		0x1108
#define PINTC_HINLR3		0x110C
#define PINTC_HINLR4		0x1110
#define PINTC_HINLR5		0x1114
#define PINTC_HINLR6		0x1118
#define PINTC_HINLR7		0x111C
#define PINTC_HINLR8		0x1120
#define PINTC_HINLR9		0x1124
#define PINTC_HIER		0x1500

#define HIPIR_NOPEND		0x80000000

static inline u32 pru_read_reg(struct pruproc *pp, unsigned int reg)
{
	return __raw_readl(pp->vaddr + reg);
}

static inline void pru_write_reg(struct pruproc *pp, unsigned int reg,
		u32 val)
{
	__raw_writel(val, pp->vaddr + reg);
}

static inline u32 pintc_read_reg(struct pruproc *pp, unsigned int reg)
{
	return pru_read_reg(pp, reg + pp->pintc);
}

static inline void pintc_write_reg(struct pruproc *pp, unsigned int reg,
		u32 val)
{
	return pru_write_reg(pp, reg + pp->pintc, val);
}

static inline u32 pcntrl_read_reg(struct pruproc_core *ppc, unsigned int reg)
{
	return pru_read_reg(ppc->pruproc, reg + ppc->pctrl);
}

static inline void pcntrl_write_reg(struct pruproc_core *ppc, unsigned int reg,
		u32 val)
{
	return pru_write_reg(ppc->pruproc, reg + ppc->pctrl, val);
}

static inline u32 pdbg_read_reg(struct pruproc_core *ppc, unsigned int reg)
{
	return pru_read_reg(ppc->pruproc, reg + ppc->pdbg);
}

static inline void pdbg_write_reg(struct pruproc_core *ppc, unsigned int reg,
		u32 val)
{
	return pru_write_reg(ppc->pruproc, reg + ppc->pdbg, val);
}

/* convert 32 bit Data device address to a virtual (and physical) host addr */
static void * __iomem pru_d_da_to_va(struct pruproc_core *ppc, u32 da,
		dma_addr_t *pa)
{
	struct pruproc *pp = ppc->pruproc;
	u32 offset;

	/* check own data ram first */
	if (da >= ppc->dram_da && da < ppc->dram_da + ppc->dram_sz)
		offset = ppc->dram + da - ppc->dram_da;
	/* check other's data ram */
	else if (da >= ppc->dram_oda && da < ppc->dram_oda + ppc->dram_sz)
		offset = ppc->dram + da - ppc->dram_oda;
	/* shared data */
	else if (da >= pp->pdram_da && da < pp->pdram_da + pp->pdram_sz)
		offset = pp->pdram + da - pp->pdram_da;
	else
		return NULL;

	if (pa)
		*pa = pp->paddr + offset;
	return pp->vaddr + offset;
}

/* convert physical address to device address */
static u32 pru_d_pa_to_da(struct pruproc_core *ppc, dma_addr_t pa)
{
	/* we don't support mapping in the GPMC space */
	if (pa < 0x00080000)
		return 0xffffffff;

	/* we also don't map internal memories */

	return (u32)pa;
}

static void * __iomem pru_i_da_to_va(struct pruproc_core *ppc, u32 da,
		dma_addr_t *pa)
{
	struct pruproc *pp = ppc->pruproc;
	u32 offset;

	/* check whether is within the iram space */
	if (da >= ppc->iram_da && da < ppc->iram-da + ppc->iram_sz)
		offset = ppc->iram + da - ppc->iram_da;
	else
		return NULL;

	if (pa)
		*pa = pp->paddr + offset;
	return pp->vaddr + offset;
}

static void * __iomem pru_d_da_to_va_block(struct pruproc_core *ppc, u32 da,
		dma_addr_t *pa, int size)
{
	void * __iomem va_start;
	void * __iomem va_end;

	/* make sure size is not bogus */
	if (size <= 0)
		return NULL;

	/* get start vaddr of device address (D) */
	va_start = pru_d_da_to_va(ppc, da, pa);
	if (va_start == NULL)
		return NULL;

	/* get end vaddr of device address (D) */
	va_end = pru_d_da_to_va(ppc, da + size - 1, NULL);
	if (va_end == NULL)
		return NULL;

	/* for the block to be valid the VAs must be consecutive */
	if ((va_end - va_start) != (size - 1))
		return NULL;

	/* all good */
	return va_start;
}

static void * __iomem pru_i_da_to_va_block(struct pruproc_core *ppc, u32 da,
		dma_addr_t *pa, int size)
{
	void * __iomem va_start;
	void * __iomem va_end;

	/* make sure size is not bogus */
	if (size <= 0)
		return NULL;

	/* get start vaddr of device address (I) */
	va_start = pru_i_da_to_va(ppc, da, pa);
	if (va_start == NULL)
		return NULL;

	/* get end vaddr of device address (I) */
	va_end = pru_i_da_to_va(ppc, da + size - 1, NULL);
	if (va_end == NULL)
		return NULL;

	/* for the block to be valid the VAs must be consecutive */
	if ((va_end - va_start) != (size - 1))
		return NULL;

	/* all good */
	return va_start;
}

static int pru_i_read_u32(struct pruproc_core *ppc, u32 da, u32 *val)
{
	void * __iomem va;

	/* verify it's a word address */
	if (da & 3)
		return -EINVAL;

	/* get vaddr of device address (I) */
	va = pru_i_da_to_va_block(ppc, da, NULL, sizeof(u32));
	if (va == NULL)
		return -EFAULT;

	if (val)
		*val = le32_to_cpu(*(u32 *)va);

	return 0;
}

static int pru_d_read_u32(struct pruproc_core *ppc, u32 da, u32 *val)
{
	void * __iomem va;

	/* verify it's a word address */
	if (da & 3)
		return -EINVAL;

	/* get vaddr of device address (I) */
	va = pru_d_da_to_va_block(ppc, da, NULL, sizeof(u32));
	if (va == NULL)
		return -EINVAL;

	if (val)
		*val = le32_to_cpu(*(u32 *)va);

	return 0;
}

static int pru_i_write_u32(struct pruproc_core *ppc, u32 da, u32 val)
{
	void * __iomem va;

	/* verify it's a word address */
	if (da & 3)
		return -EINVAL;

	/* get vaddr of device address (I) */
	va = pru_i_da_to_va_block(ppc, da, NULL, sizeof(u32));
	if (va == NULL)
		return -EINVAL;

	*(u32 *)va = cpu_to_le32(val);

	return 0;
}

static int pru_d_write_u32(struct pruproc_core *ppc, u32 da, u32 val)
{
	void * __iomem va;

	/* verify it's a word address */
	if (da & 3)
		return -EINVAL;

	/* get vaddr of device address (I) */
	va = pru_d_da_to_va_block(ppc, da, NULL, sizeof(u32));
	if (va == NULL)
		return -EINVAL;

	*(u32 *)va = cpu_to_le32(val);
	return 0;
}

static int pruproc_bin_sanity_check(struct rproc *rproc, const struct firmware *fw)
{
	return 0;
}

/* Loads the firmware to shared memory. */
static int pruproc_bin_load_segments(struct rproc *rproc, const struct firmware *fw)
{
	struct pruproc_core *ppc = rproc->priv;
	struct device *dev = &rproc->dev;
	void __iomem *va;

	pcntrl_write_reg(ppc, PCTRL_CONTROL, CONTROL_SOFT_RST_N);

	/* binary starts from 0 */
	ppc->entry_point = 0;

	va = pru_i_da_to_va_block(ppc, ppc->entry_point, NULL, fw->size);
	if (va == NULL) {
		dev_err(dev, "FW is larger than available space (%u)\n",
				fw->size);
		return -ENOMEM;
	}

	/* just copy */
	memcpy(va, fw->data, fw->size);

	return 0;
}

static int
pruproc_elf_sanity_check(struct rproc *rproc, const struct firmware *fw)
{
	const char *name = rproc->firmware;
	struct device *dev = &rproc->dev;
	struct elf32_hdr *ehdr;
	char class;

	if (!fw) {
		dev_err(dev, "failed to load %s\n", name);
		return -EINVAL;
	}

	if (fw->size < sizeof(struct elf32_hdr)) {
		dev_err(dev, "Image is too small\n");
		return -EINVAL;
	}

	ehdr = (struct elf32_hdr *)fw->data;

	/* We only support ELF32 at this point */
	class = ehdr->e_ident[EI_CLASS];
	if (class != ELFCLASS32) {
		dev_err(dev, "Unsupported class: %d\n", class);
		return -EINVAL;
	}

	/* PRU is little endian */
	if (ehdr->e_ident[EI_DATA] != ELFDATA2LSB) {
		dev_err(dev, "Unsupported firmware endianness\n");
		return -EINVAL;
	}

	if (fw->size < le32_to_cpu(ehdr->e_shoff) +
			sizeof(struct elf32_shdr)) {
		dev_err(dev, "Image is too small\n");
		return -EINVAL;
	}

	if (memcmp(ehdr->e_ident, ELFMAG, SELFMAG)) {
		dev_err(dev, "Image is corrupted (bad magic)\n");
		return -EINVAL;
	}

	if (le16_to_cpu(ehdr->e_phnum) == 0) {
		dev_err(dev, "No loadable segments\n");
		return -EINVAL;
	}

	if (le32_to_cpu(ehdr->e_phoff) > fw->size) {
		dev_err(dev, "Firmware size is too small\n");
		return -EINVAL;
	}

	return 0;
}

static int
pruproc_elf_load_segments(struct rproc *rproc, const struct firmware *fw)
{
	struct device *dev = &rproc->dev;
	struct pruproc_core *ppc = rproc->priv;
	struct elf32_hdr *ehdr;
	struct elf32_phdr *phdr;
	int i, ret = 0;
	const u8 *elf_data = fw->data;
	u32 da, memsz, filesz, offset, flags;
	void * __iomem va;

	pcntrl_write_reg(ppc, PCTRL_CONTROL, CONTROL_SOFT_RST_N);

	ehdr = (struct elf32_hdr *)elf_data;
	phdr = (struct elf32_phdr *)(elf_data + le32_to_cpu(ehdr->e_phoff));

	/* go through the available ELF segments */
	for (i = 0; i < le16_to_cpu(ehdr->e_phnum); i++, phdr++) {

		da = le32_to_cpu(phdr->p_paddr);
		memsz = le32_to_cpu(phdr->p_memsz);
		filesz = le32_to_cpu(phdr->p_filesz);
		offset = le32_to_cpu(phdr->p_offset);
		flags = le32_to_cpu(phdr->p_flags);

		if (le32_to_cpu(phdr->p_type) != PT_LOAD)
			continue;

		dev_dbg(dev, "phdr: type %d da 0x%x memsz 0x%x filesz 0x%x"
				    " flags %c%c%c\n",
					le32_to_cpu(phdr->p_type),
					da, memsz, filesz,
					(flags & PF_R) ? 'R' : '-',
					(flags & PF_W) ? 'W' : '-',
					(flags & PF_X) ? 'E' : '-');

		/* PRU is not a unified address space architecture */
		/* we need to map differently executable & data segments */

		if (filesz > memsz) {
			dev_err(dev, "bad phdr filesz 0x%x memsz 0x%x\n",
							filesz, memsz);
			ret = -EINVAL;
			break;
		}

		if (offset + filesz > fw->size) {
			dev_err(dev, "truncated fw: need 0x%x avail 0x%zx\n",
					offset + filesz, fw->size);
			ret = -EINVAL;
			break;
		}

		/* we can't use rproc_da_to_va (it relies on carveouts) */
		if (filesz == 0)
			continue;

		/* text? to code area */
		if (flags & PF_X)
			va = pru_i_da_to_va_block(ppc, da, NULL, memsz);
		else
			va = pru_d_da_to_va_block(ppc, da, NULL, memsz);

		/* check if valid section */
		if (va == NULL) {
			dev_err(dev, "fw: #%d @0x%x sz 0x%x bad\n",
					i, da, memsz);
			ret = -EINVAL;
			break;
		}

		/* put the segment where the remote processor expects it */
		if (filesz > 0)
			memcpy(va, elf_data + offset, filesz);
		else
			memset(va, 0, memsz);
	}

	ppc->entry_point = le32_to_cpu(ehdr->e_entry);

	return ret;
}

static
u32 pruproc_elf_get_boot_addr(struct rproc *rproc, const struct firmware *fw)
{
	struct elf32_hdr *ehdr  = (struct elf32_hdr *)fw->data;

	return le32_to_cpu(ehdr->e_entry);
}

/* just return the built-firmware resources */
static struct resource_table *
pruproc_find_rsc_table(struct rproc *rproc, const struct firmware *fw,
		     int *tablesz)
{
	struct pruproc_core *ppc = rproc->priv;

	/* no resource table for this PRU */
	if (ppc->table == NULL) {
		*tablesz = 0;
		return NULL;
	}

	*tablesz = ppc->table_size;
	return ppc->table;
}

void *get_resource_type(struct resource_table *res,
		int type, int idx)
{
	struct fw_rsc_hdr *rsc_hdr;
	int i, j;

	j = 0;
	for (i = 0; i < res->num; i++) {
		rsc_hdr = (void *)res + res->offset[i];
		if (type >= 0 && rsc_hdr->type != type)
			continue;
		if (j == idx)
			return &rsc_hdr->data[0];
		j++;
	}

	return NULL;
}

/* update the device's firmware resource with the allocated values */
static int update_dev_rsc_table(struct pruproc_core *ppc)
{
	struct rproc *rproc = ppc->rproc;
	struct device *dev = &rproc->dev;
	struct rproc_vdev *rvdev;
	struct rproc_vring *rvring;
	struct pru_vring_info *vri;
	struct fw_rsc_vdev *rsc_vdev;
	int vdev_idx, i, cnt, err, j, vring_start, num_vrings;

	/* no table; do nothing */
	if (ppc->table == NULL)
		return 0;

	/* copy the resource table here */
	memcpy(ppc->dev_table_va, ppc->table, ppc->table_size);

	/* everything is initialized; fill in the real da & notify ids */
	for (vdev_idx = 0; vdev_idx < ppc->num_vdevs; vdev_idx++) {
		vring_start = ppc->vdev_vring_start[vdev_idx];
		num_vrings = ppc->vdev_vring_count[vdev_idx];

		/* find rvdev */
		cnt = 0;
		list_for_each_entry(rvdev, &rproc->rvdevs, node) {
			if (cnt == vdev_idx)
				break;
			cnt++;
		}
		if (cnt != vdev_idx) {
			dev_warn(dev, "rsc_vdev not found (continuing anyway)\n");
			rvdev = NULL;
		}

		/* locate the vdev resource in the device memory */
		rsc_vdev = get_resource_type(ppc->dev_table_va, RSC_VDEV,
				vdev_idx);
		if (rsc_vdev == NULL) {
			dev_err(dev, "PRU#%d Failed to get RSV_VDEV #%d\n",
					ppc->idx, vdev_idx);
			err = -EINVAL;
			goto err_fail;
		}

		for (i = 0; i < num_vrings; i++) {
			j = vring_start + i;

			vri = &ppc->vring_info[j];

			if (rvdev != NULL)
				rvring = &rvdev->vring[i];
			else
				rvring = NULL;

			/* keep track of rproc's rvring */
			vri->rvring = rvring;

			rsc_vdev->vring[i].da = vri->da;
			if (rvring != NULL)
				rsc_vdev->vring[i].notifyid = rvring->notifyid;
			else
				rsc_vdev->vring[i].notifyid = -1;	/* mark that it's not ready yet */

			dev_info(dev, "VDEV#%d VR#%d da=0x%08x notifyid=0x%08x\n",
					vdev_idx, i, vri->da, rvring->notifyid);
		}
	}

#if 0
	j = 0;
	list_for_each_entry(rvdev, &rproc->rvdevs, node) {

		/* get copied resource's VDEV */
		rsc_vdev = get_resource_type(ppc->dev_table_va, RSC_VDEV,
				vdev_idx);
		if (rsc_vdev == NULL) {
			dev_err(dev, "Failed to get RSV_VDEV #%d\n", vdev_idx);
			err = -EINVAL;
			goto err_fail;
		}

		for (i = 0; i < ARRAY_SIZE(rvdev->vring); i++) {
			rvring = &rvdev->vring[i];

			if (j >= ARRAY_SIZE(ppc->vring_info)) {
				dev_err(dev, "Too many vrings\n");
				err = -ENOENT;
				goto err_fail;
			}

			vri = &ppc->vring_info[j];

			/* keep track of rproc's rvring */
			vri->rvring = rvring;

			rsc_vdev->vring[i].da = vri->da;
			rsc_vdev->vring[i].notifyid = rvring->notifyid;

			dev_info(dev, "VDEV#%d VR#%d da=0x%08x notifyid=0x%08x\n",
					vdev_idx, i, vri->da, rvring->notifyid);

			j++;
		}

		vdev_idx++;
	}
#endif

	return 0;
err_fail:
	return err;

}

/* PRU binary firmware handler operations */
const struct rproc_fw_ops pruproc_bin_fw_ops = {
	.find_rsc_table	= pruproc_find_rsc_table,
	.load		= pruproc_bin_load_segments,
	.sanity_check	= pruproc_bin_sanity_check,
};

/* PRU elf handler operations */
const struct rproc_fw_ops pruproc_elf_fw_ops = {
	.find_rsc_table	= pruproc_find_rsc_table,
	.load		= pruproc_elf_load_segments,
	.sanity_check	= pruproc_elf_sanity_check,
	.get_boot_addr	= pruproc_elf_get_boot_addr,
};

void dump_vring(struct vring *vring)
{
	int i;
	struct vring_desc *vd;

	pr_info("vring: num=%u desc @%p\n", vring->num, vring->desc);

	for (i = 0; i < vring->num; i++) {
		vd = &vring->desc[i];
		pr_info(" d#%2d: a 0x%016llx l 0x%08x f 0x%04x n 0x%04x\n",
				i, vd->addr, vd->len, vd->flags, vd->next);
	}
	pr_info(" avail: flags 0x%04x idx 0x%04x\n",
			vring->avail->flags, vring->avail->idx);
	for (i = 0; i < vring->num; i++) {
		pr_info(" a#%2d: ring 0x%04x\n", i,
				vring->avail->ring[i]);
	}
	pr_info(" avail: used_event_idx 0x%04x\n",
			vring->avail->ring[vring->num]);

	pr_info(" used: flags 0x%04x idx 0x%04x\n",
			vring->used->flags, vring->used->idx);
	for (i = 0; i < vring->num; i++) {
		pr_info(" u#%2d: id 0x%08x len 0x%08x\n", i,
				vring->used->ring[i].id,
				vring->used->ring[i].len);
	}
}

void dump_all_vrings(struct pruproc_core *ppc)
{
	struct device *dev = &ppc->rproc->dev;
	struct vring *vr;
	struct fw_rsc_vdev_vring *rsc_vr;
	struct pru_vring_info *vri;
	int i;

	for (i = 0; i < ppc->num_vrings; i++) {
		vri = &ppc->vring_info[i];

		vr = &vri->vr;
		rsc_vr = vri->rsc;

		dev_dbg(dev, "VRING #%d (size %d)\n", i,
				vring_size(vr->num, rsc_vr->align));
		dump_vring(vr);
		dev_dbg(dev, "\n");
		print_hex_dump(KERN_DEBUG, "", DUMP_PREFIX_OFFSET,
				16, 4, vr->desc,
				vring_size(vr->num, rsc_vr->align),
				false);
	}
}

/* Kick the modem with specified notification id */
static void pruproc_kick(struct rproc *rproc, int vqid)
{
	struct device *dev = &rproc->dev;
	struct pruproc_core *ppc = rproc->priv;
	struct pruproc *pp = ppc->pruproc;
	int sysint;

	dev_dbg(dev, "kick #%d vqid:%d\n", ppc->idx, vqid);

	sysint = ppc->pru_vring_sysev;
	if (sysint < 0) {
		dev_err(dev, "PRU#%d no vring_sysev to kick with\n", ppc->idx);
		return;
	}

	/* signal event */
	if (sysint < 32)
		pintc_write_reg(pp, PINTC_SRSR0, 1 << sysint);
	else
		pintc_write_reg(pp, PINTC_SRSR1, 1 << (sysint - 32));
}

void dump_resource_table(const struct resource_table *res)
{
	const struct fw_rsc_hdr *rsc_hdr;
	const struct fw_rsc_vdev *rsc_vdev;
	const struct fw_rsc_vdev_vring *rsc_vring;
	int i, j;

	/* do nothing if given nothing */
	if (res == NULL)
		return;

	pr_info("resource_table @%p\n", res);
	pr_info(" .ver = 0x%08x\n", res->ver);
	pr_info(" .num = 0x%08x\n", res->num);
	for (i = 0; i < res->num; i++) {
		rsc_hdr = (void *)res + res->offset[i];
		pr_info("  rsc_hdr#%d: .type = 0x%08x\n", i,
				rsc_hdr->type);
		switch (rsc_hdr->type) {
		case RSC_CARVEOUT:
			pr_info("  CARVEOUT:\n");
			break;
		case RSC_DEVMEM:
			pr_info("  DEVMEM:\n");
			break;
		case RSC_TRACE:
			pr_info("  TRACE:\n");
			break;
		case RSC_VDEV:
			pr_info("  VDEV:\n");
			rsc_vdev = (void *)&rsc_hdr->data[0];
			pr_info("   .id = 0x%08x\n", rsc_vdev->id);
			pr_info("   .notifyid = 0x%08x\n", rsc_vdev->notifyid);
			pr_info("   .dfeatures = 0x%08x\n", rsc_vdev->dfeatures);
			pr_info("   .gfeatures = 0x%08x\n", rsc_vdev->gfeatures);
			pr_info("   .config_len = 0x%08x\n", rsc_vdev->config_len);
			pr_info("   .status = 0x%02x\n", rsc_vdev->status);
			pr_info("   .num_of_vrings = 0x%02x\n", rsc_vdev->num_of_vrings);
			for (j = 0; j < rsc_vdev->num_of_vrings; j++) {
				rsc_vring = &rsc_vdev->vring[j];
				pr_info("   VRING #%d:\n", j);
				pr_info("     .da = 0x%08x\n",
						rsc_vring->da);
				pr_info("     .align = 0x%08x\n",
						rsc_vring->align);
				pr_info("     .num = 0x%08x\n",
						rsc_vring->num);
				pr_info("     .notifyid = 0x%08x\n",
						rsc_vring->notifyid);
			}
			break;
		}
	}
}

/* Start the PRU modem */
static int pruproc_start(struct rproc *rproc)
{
	struct device *dev = &rproc->dev;
	struct pruproc_core *ppc = rproc->priv;
	u32 val;
	int err;

	dev_info(dev, "PRU#%d bootcnt=%d\n",
			ppc->idx, atomic_read(&ppc->bootcnt));

	/* start the processors only when all devices have booted */
	if (ppc->num_vdevs == 0 || atomic_inc_return(&ppc->bootcnt) == 1) {

		dev_info(dev, "PRU#%d entry-point 0x%x\n",
				ppc->idx, ppc->entry_point);

#if 1
		if (ppc->table)
			dump_resource_table(ppc->table);

		err = update_dev_rsc_table(ppc);
		if (err != 0) {
			dev_err(dev, "failed to update resource table\n");
			return err;
		}

		if (ppc->dev_table_va)
			dump_resource_table(ppc->dev_table_va);

#endif


		val = CONTROL_ENABLE | ((ppc->entry_point >> 2) << 16);
		pcntrl_write_reg(ppc, PCTRL_CONTROL, val);
	}

	return 0;
}

/* Stop the PRU modem */
static int pruproc_stop(struct rproc *rproc)
{
	struct device *dev = &rproc->dev;
	struct pruproc_core *ppc = rproc->priv;
	u32 val;

	/* we have to copy the resource table and update the device addresses */
	if (1 || ppc->num_vdevs == 0 || atomic_dec_return(&ppc->bootcnt) == 0) {

		dev_info(dev, "PRU#%d stop\n", ppc->idx);

		val = pcntrl_read_reg(ppc, PCTRL_CONTROL);
		val &= ~CONTROL_ENABLE;
		pcntrl_write_reg(ppc, PCTRL_CONTROL, val);
	}

	return 0;
}

static void *pruproc_alloc_vring(struct rproc *rproc,
		const struct fw_rsc_vdev_vring *rsc_vring,
		int size, dma_addr_t *dma)
{
	struct device *dev = &rproc->dev;
	struct pruproc_core *ppc = rproc->priv;
	struct pru_vring_info *vri;
	struct vring *vring;
	void * __iomem va;
	dma_addr_t dma_tmp;
	int i;

	dev_dbg(dev, "PRU#%d alloc rsc_vring %p\n",
			ppc->idx, rsc_vring);

	/* find vring index */
	for (i = 0; i < ppc->num_vrings; i++) {
		if (rsc_vring == ppc->vring_info[i].rsc)
			break;
	}

	if (i >= ppc->num_vrings) {
		dev_err(dev, "PRU#%d could not find rsc_vring at %p\n",
				ppc->idx, rsc_vring);
		return NULL;
	}

	if (dma == NULL)
		dma = &dma_tmp;

	if (rsc_vring->da != 0) {
		dev_dbg(dev, "PRU#%d alloc vring #%d from internal memory\n",
				ppc->idx, i);
		va = pru_d_da_to_va_block(ppc, rsc_vring->da, dma, size);
	} else {
		dev_dbg(dev, "PRU#%d alloc vring #%d dma_alloc_coherent\n",
				ppc->idx, i);
		va = dma_alloc_coherent(dev->parent, PAGE_ALIGN(size),
			dma, GFP_KERNEL);
	}

	if (va == NULL) {
		dev_err(dev, "PRU#%d could not allocate vring %p\n",
				ppc->idx, rsc_vring);
		return NULL;
	}

	/* setup vring for use */
	vri = &ppc->vring_info[i];
	vring = &vri->vr;
	vring_init(vring, rsc_vring->num, va, rsc_vring->align);

	/* save VA & PA */
	vri->va = va;
	vri->pa = *dma;
	vri->da = pru_d_pa_to_da(ppc, *dma);

	dev_dbg(dev, "PRU#%d vring #%d da=0x%x, va=%p, dma=0x%llx size=%u\n",
		ppc->idx, i, rsc_vring->da, va, (unsigned long long)*dma, size);

	return va;
}

static void pruproc_free_vring(struct rproc *rproc,
		const struct fw_rsc_vdev_vring *rsc_vring,
		int size, void *va, dma_addr_t dma)
{
	struct device *dev = &rproc->dev;

	/* if DA is NULL, means we allocated via dma_alloc_coherent */
	if (rsc_vring->da == 0)
		dma_free_coherent(dev->parent, PAGE_ALIGN(size), va, dma);
}

static struct rproc_ops pruproc_ops = {
	.start		= pruproc_start,
	.stop		= pruproc_stop,
	.kick		= pruproc_kick,

	.alloc_vring	= pruproc_alloc_vring,
	.free_vring	= pruproc_free_vring,
};

static ssize_t pruproc_store_load(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pruproc *pp = platform_get_drvdata(pdev);
	struct pruproc_core *ppc;
	char *fw_name[MAX_PRUS];
	const char *s, *e, *t;
	int i, pru_idx, sz, err;
	const struct firmware *fw;
	u32 val;

	memset(fw_name, 0, sizeof(fw_name));

	s = buf;
	while (*s != '\0' && *s != '\n') {
		e = strchr(s, ',');
		if (e == NULL) {
			e = s + strlen(s);
			while (e > buf && e[-1] == '\n')
				e--;
		}
		t = strchr(s, ':');
		if (t == NULL) {
			t = s;
			pru_idx = 0;
		} else {
			t++;
			pru_idx = simple_strtoul(s, NULL, 10);
		}

		for (i = 0; i < pp->num_prus; i++) {
			ppc = pp->pruc[i];
			if (pru_idx == ppc->idx)
				break;
		}
		if (i >= pp->num_prus) {
			dev_err(dev, "Can not find PRU#%d\n", pru_idx);
			return -EINVAL;
		}

		sz = e - t;
		fw_name[pru_idx] = kzalloc(sz + 1, GFP_KERNEL);
		if (fw_name[pru_idx] == NULL)
			return -ENOMEM;
		memcpy(fw_name[pru_idx], t, sz);
		fw_name[pru_idx][sz] = '\0';

		s = e;
		if (*s == ',')
			s++;
	}

	/* first halt every PRU affected */
	for (i = 0; i < pp->num_prus; i++) {
		ppc = pp->pruc[i];

		if (fw_name[ppc->idx] == NULL)
			continue;

		/* keep it in reset */
		pcntrl_write_reg(ppc, PCTRL_CONTROL, CONTROL_SOFT_RST_N);

		dev_info(dev, "PRU#%d halted\n", ppc->idx);

	}

	/* load every PRU */
	for (i = 0; i < pp->num_prus; i++) {
		ppc = pp->pruc[i];
		if (fw_name[ppc->idx] == NULL)
			continue;

		dev_info(dev, "PRU#%d loading %s\n", ppc->idx, fw_name[ppc->idx]);

		/* note this is not the rproc device */
		err = request_firmware(&fw, fw_name[ppc->idx], dev);
		if (err != 0) {
			dev_err(dev, "PRU#%d Failed to load firmware %s\n",
					ppc->idx, fw_name[ppc->idx]);
			return err;
		}

		err = pruproc_elf_load_segments(ppc->rproc, fw);
		if (err != 0) {
			dev_err(dev, "PRU#%d Failed to update firmware %s\n",
					ppc->idx, fw_name[ppc->idx]);
			return err;
		}

		release_firmware(fw);
	}

	/* start every PRU affected */
	for (i = 0; i < pp->num_prus; i++) {
		ppc = pp->pruc[i];
		if (fw_name[ppc->idx] == NULL)
			continue;

		dev_info(dev, "PRU#%d starting %s\n", ppc->idx, fw_name[ppc->idx]);

		val = CONTROL_ENABLE | ((ppc->entry_point >> 2) << 16);
		pcntrl_write_reg(ppc, PCTRL_CONTROL, val);

		/* and free */
		kfree(fw_name[ppc->idx]);
		fw_name[ppc->idx] = NULL;
	};

	return strlen(buf);
}

static ssize_t pruproc_store_reset(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pruproc *pp = platform_get_drvdata(pdev);
	struct pruproc_core *ppc;
	int i;
	u32 val;

	/* first halt every PRU affected */
	for (i = 0; i < pp->num_prus; i++) {
		ppc = pp->pruc[i];
		/* keep it in reset */
		pcntrl_write_reg(ppc, PCTRL_CONTROL, CONTROL_SOFT_RST_N);
	}

	/* start every PRU affected */
	for (i = 0; i < pp->num_prus; i++) {
		ppc = pp->pruc[i];
		val = CONTROL_ENABLE | ((ppc->entry_point >> 2) << 16);
		pcntrl_write_reg(ppc, PCTRL_CONTROL, val);

	};

	return strlen(buf);
}

static int pru_downcall(struct pruproc_core *ppc,
		u32 nr, u32 arg0, u32 arg1, u32 arg2, u32 arg3, u32 arg4);

static int pru_downcall_idx(struct pruproc *pp, int idx,
		u32 nr, u32 arg0, u32 arg1, u32 arg2, u32 arg3, u32 arg4);

/*

static ssize_t pruproc_store_downcall(int idx,
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pruproc *pp = platform_get_drvdata(pdev);
	int ret;

	ret = pru_downcall_idx(pp, idx, 0x5, 0xaa55, 0x1234, 0x98ff, 0, 0);

	dev_info(dev, "PRU#%d downcall test - ret = %d\n", idx, ret);

	return strlen(buf);
}

static ssize_t pruproc_store_downcall0(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	return pruproc_store_downcall(0, dev, attr, buf, count);
}

static ssize_t pruproc_store_downcall1(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	return pruproc_store_downcall(1, dev, attr, buf, count);
}

*/

/*
 * syscall #0 - useful while developing driver
 * 		writing 1 => all output pins in PRU control set high
 *		writing 0 => all output pins in PRU control set low
 */
static ssize_t pru_speak_debug(int idx, struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pruproc *pp = platform_get_drvdata(pdev);
	int ret;

	if (count ==  0)
		return -1;
	if (buf[0] == '0')
		ret = pru_downcall_idx(pp, idx, 0, 0, 0, 0, 0, 0);/*pp, idx, syscall_id, 5 args*/
	else
		ret = pru_downcall_idx(pp, idx, 0, 1, 0, 0, 0, 0);

	printk( KERN_INFO "write to pru_speak_debug\n");
	return strlen(buf);
}

static ssize_t pru_speak_debug0(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return pru_speak_debug(0, dev, attr, buf, count);
}

/*
 * syscall #1 - used to initialize the shm info for both PRU and ARM
 * 		*reading* this file will give back hex value of physical address as a string
 *		at the same time PRU is also informed of the address via a downcall
 */

static ssize_t pru_speak_shm_init(int idx, struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
        struct pruproc *pp = platform_get_drvdata(pdev);
        int ret;

	//struct pru_shm shm = pp->ppc->shm[BS_CODE];
	struct pru_shm shm_code = pp->pru_to_pruc[idx]->shm[BS_CODE];
	struct pru_shm shm_ret = pp->pru_to_pruc[idx]->shm[BS_RET];

	printk("physical addr shm_code, shm_ret : %x, %x\n", (unsigned int)shm_code.paddr, (unsigned int)shm_ret.paddr);

        ret = pru_downcall_idx(pp, idx, 1, (int)shm_code.paddr, (int)shm_ret.paddr, 10, 0, 0); //pp, idx, sys call id, base addr, val, junk,.,.
	//The pru modifies the arg "10" and places "10^2" in the first loc of shm_code
        printk( KERN_INFO "pru_speak_init, pram value : 10, return value : %d, modified value : %d\n", ret, *((int *)shm_code.vaddr));

	return scnprintf(buf, PAGE_SIZE, "%x,%x", (int)shm_code.paddr, (int)shm_ret.paddr);
}

static ssize_t pru_speak_shm_init0(struct device *dev, struct device_attribute *attr, char *buf)
{
        return pru_speak_shm_init(0, dev, attr, buf);
}

/*
 * syscall #2 - ask PRU to start execution.
 *		writing 1 => start/continue execution
 *		writing 0 => pause execution
 *
 * NOTE : pru_shm_init must be triggered before this. otherwise hell will chase you.
 */

static ssize_t pru_speak_execute(int idx, struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        struct platform_device *pdev = to_platform_device(dev);
        struct pruproc *pp = platform_get_drvdata(pdev);
        int ret;

	if (count ==  0)
                return -1;
        if (buf[0] == '0')
                ret = pru_downcall_idx(pp, idx, 2, 0, 0, 0, 0, 0);/*pp, idx, syscall_id, start/pause, 4 args*/
        else
                ret = pru_downcall_idx(pp, idx, 2, 1, 0, 0, 0, 0);

        printk( KERN_INFO "write to pru_speak_execute\n");
        return strlen(buf);

}

static ssize_t pru_speak_execute0(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        return pru_speak_execute(0, dev, attr, buf, count);
}

/*
 * syscall #3 - ask pru to abort currently executing BS script
 *		triggered by write to this file
 */

static ssize_t pru_speak_abort(int idx, struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        struct platform_device *pdev = to_platform_device(dev);
        struct pruproc *pp = platform_get_drvdata(pdev);
        int ret;

        ret = pru_downcall_idx(pp, idx, 3, 0, 0, 0, 0, 0); /* pp, idx, 5 args*/

        printk( KERN_INFO "write to pru_speak_abort\n");
        return strlen(buf);

}

static ssize_t pru_speak_abort0(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        return pru_speak_abort(0, dev, attr, buf, count);
}

/*
 * syscall #4 - get status of botspeak interpreter
 *		"read" returns 0 - if no BS code is being executed
 *		returns 1 - if BS code is being executed
 */

static ssize_t pru_speak_status(int idx, struct device *dev, struct device_attribute *attr, char *buf)
{
        struct platform_device *pdev = to_platform_device(dev);
        struct pruproc *pp = platform_get_drvdata(pdev);
        int ret = pru_downcall_idx(pp, idx, 4, 0, 0, 0, 0, 0); //pp, idx, sys call id, 5 params

	printk( KERN_INFO "read to pru_speak_status\n");

	return scnprintf(buf, PAGE_SIZE, "%d", ret);
}

static ssize_t pru_speak_status0(struct device *dev, struct device_attribute *attr, char *buf)
{
        return pru_speak_status(0, dev, attr, buf);
}

/*
 * syscall #5 - ask PRU to execute single BS instruction
 *		The Byte code of the BS instruction to be executed is written
 *		to this file.
 *		return value of downcall = 4 if OK, else value = ?
 */

static ssize_t pru_speak_single_cmd(int idx, struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        struct platform_device *pdev = to_platform_device(dev);
        struct pruproc *pp = platform_get_drvdata(pdev);
        int inst = 0, ret, i;

        if (count ==  0)
                return -1;

	//buf[0] = LSB; buf[3] = MSB
	//the for loop packs the 4 incoming character into a 1 word long integer
	//viz one botspeak instruction (byte code)
	for(i = 0; i < 4; i++){
		inst |= ((int)buf[i]) << i*8;
	}

        ret = pru_downcall_idx(pp, idx, 5, inst, 0, 0, 0, 0);/*pp, idx, syscall_id, 5 args*/

        printk( KERN_INFO "write to pru_speak_single_cmd. return value of downcall : %d\n", ret);
	printk( "STRLEN(buf) : %d\n", strlen(buf)); //prints 0!!
        //return strlen(buf);
	return 4; //quick hack
}

static ssize_t pru_speak_single_cmd0(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        return pru_speak_single_cmd(0, dev, attr, buf, count);
}

/*
 * syscall #6 - ask PRU to execute single 64 bit BS instruction
 *		The Byte code of the BS instruction to be executed is written
 *		to this file.
 *		return value of downcall = 8 if OK, else value = ?
 */

static ssize_t pru_speak_single_cmd_64(int idx, struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        struct platform_device *pdev = to_platform_device(dev);
        struct pruproc *pp = platform_get_drvdata(pdev);
        int inst_a = 0, ret, i;
        int inst_b = 0;

	//buf[0] = LSB; buf[3] = MSB : for 1st word
	//buf[4] = LSB; buf[7] = MSB : for 2nd word
	//the for loop packs the 8 incoming character into two, 1 word long integers
	//viz one botspeak instruction (byte code)
	for(i = 0; i < 4; i++){
		inst_a |= ((int)buf[i]) << i*8;
	}

	for(i = 4; i < 8; i++){
		inst_b |= ((int)buf[i]) << (i-4)*8;
	}

        ret = pru_downcall_idx(pp, idx, 6, inst_a, inst_b, 0, 0, 0);/*pp, idx, syscall_id, 5 args*/

	printk("**** inst_a : %d, inst_b : %d", inst_a, inst_b);
        printk( KERN_INFO "write to pru_speak_single_cmd_64. return value of downcall : %d\n", ret);
	return 8; //quick hack
}

static ssize_t pru_speak_single_cmd0_64(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        return pru_speak_single_cmd_64(0, dev, attr, buf, count);
}

static DEVICE_ATTR(load, S_IWUSR, NULL, pruproc_store_load);
static DEVICE_ATTR(reset, S_IWUSR, NULL, pruproc_store_reset);
static DEVICE_ATTR(pru_speak_debug, S_IWUSR, NULL, pru_speak_debug0);
static DEVICE_ATTR(pru_speak_shm_init, S_IWUSR | S_IRUGO, pru_speak_shm_init0, NULL);
static DEVICE_ATTR(pru_speak_execute, S_IWUSR, NULL, pru_speak_execute0);
static DEVICE_ATTR(pru_speak_abort, S_IWUSR, NULL, pru_speak_abort0);
static DEVICE_ATTR(pru_speak_status, S_IRUGO, pru_speak_status0, NULL);
static DEVICE_ATTR(pru_speak_single_cmd, S_IWUSR, NULL, pru_speak_single_cmd0);
static DEVICE_ATTR(pru_speak_single_cmd_64, S_IWUSR, NULL, pru_speak_single_cmd0_64);

/*
 *
 *	BIN FILE SYSFS - for mmap'ing. share mem b/w userspace, kernel and PRU
 *
 */
/* Will develop this later
void pru_vma_open(struct vm_area_struct *vma)
{
	printk(KERN_NOTICE "PRU Speak VMA open, virt %lx, phys %lx\n",
            vma->vm_start, vma->vm_pgoff);
}

void pru_vma_close(struct vm_area_struct *vma)
{
	printk(KERN_NOTICE "PRU Speak VMA close.\n");
}

static struct vm_operations_struct pru_remap_vm_ops = {
	.open =  pru_vma_open,
	.close = pru_vma_close,
};

static ssize_t bin_file_read(struct file * f, struct kobject *kobj, struct bin_attribute *attr, char *buffer, loff_t pos, size_t size)
{
	printk(KERN_INFO "READ called on BIN FILE\n");
	return (ssize_t)0;
}

static ssize_t bin_file_write(struct file *f, struct kobject *kobj, struct bin_attribute *attr, char *buffer, loff_t pos, size_t size)
{
	printk(KERN_INFO "WRITE called on BIN FILE\n");
	return size;
}

static int bin_file_mmap(struct file *f, struct kobject *kobj, struct bin_attribute *attr,  struct vm_area_struct *vma)
{
	//printk(KERN_INFO "MMAP called on BIN FILE, PAGE_SHIFT value : %d\n", PAGE_SHIFT);
	if (remap_pfn_range(vma, vma->vm_start, (unsigned long)((int)shm.paddr >> PAGE_SHIFT), vma->vm_end - vma->vm_start, vma->vm_page_prot))
	{
		printk("MMAP failed\n");
		return -EAGAIN;
	}

	vma->vm_ops = &pru_remap_vm_ops;
	pru_vma_open(vma);

	printk("physical address that was to be maped %x : \n", (int)shm.paddr);
	printk("MMAP successful\n");
	return 0;
}

static struct bin_attribute pru_speak_bin_attr = BIN_ATTR(pru_speak_binfile, S_IWUSR | S_IRUGO, PAGE_SIZE, bin_file_read, bin_file_write, bin_file_mmap);
*/

/* PRU is unregistered */
static int pruproc_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pruproc *pp = platform_get_drvdata(pdev);
	struct pruproc_core *ppc;
	int i;

	dev_dbg(dev, "remove pru\n");

	device_remove_file(dev, &dev_attr_reset);
	device_remove_file(dev, &dev_attr_load);
	device_remove_file(dev, &dev_attr_pru_speak_debug);
	device_remove_file(dev, &dev_attr_pru_speak_shm_init);
	device_remove_file(dev, &dev_attr_pru_speak_execute);
	device_remove_file(dev, &dev_attr_pru_speak_abort);
	device_remove_file(dev, &dev_attr_pru_speak_status);
	device_remove_file(dev, &dev_attr_pru_speak_single_cmd);
	device_remove_file(dev, &dev_attr_pru_speak_single_cmd_64);
	//sysfs_remove_bin_file( (&dev->kobj), &pru_speak_bin_attr);


	/* Unregister as remoteproc device */
	for (i = pp->num_prus - 1; i >= 0; i--) {
		ppc = pp->pruc[i];
		rproc_del(ppc->rproc);
		rproc_put(ppc->rproc);

		if (ppc->dev_table_va != NULL)
			dma_free_coherent(dev, PAGE_ALIGN(ppc->table_size),
					ppc->dev_table_va, ppc->dev_table_pa);

		//if( (ppc->idx == shm.idx) && (shm.is_valid == 1) )
	          //      dma_free_coherent(dev, PAGE_SIZE, shm.vaddr, (dma_addr_t)shm.paddr);
		for (i=0; i < MAX_SHARED; i++){
			if(ppc->shm[i].is_valid)
				dma_free_coherent(dev, ppc->shm[i].size_in_pages * PAGE_SIZE,ppc->shm[i].vaddr,
											(dma_addr_t)ppc->shm[i].paddr);
		}

	}

	platform_set_drvdata(pdev, NULL);

	return 0;
}

#define PRU_SC_HALT	0
#define PRU_SC_PUTC	1
#define PRU_SC_EXIT	2
#define PRU_SC_PUTS	3
#define PRU_SC_GET_CFG	4
#define  PRU_SC_GET_CFG_VRING_NR 0
#define  PRU_SC_GET_CFG_VRING_INFO 1
#define  PRU_SC_GET_CFG_RESOURCE_TABLE 2

#define PRU_SC_DOWNCALL_READY       254     /* host requested a downcall, ack it, and execute */
#define PRU_SC_DOWNCALL_DONE        255     /* call is performed, inform the host */

struct pru_dev_vring_info {
	u32 paddr;
	u32 num;
	u32 align;
	u32 pad;
};

/* verify that the PRU is halted */
static int pru_is_halted(struct pruproc_core *ppc, u32 *addrp)
{
	struct pruproc *pp = ppc->pruproc;
	struct device *dev = &pp->pdev->dev;
	u32 val, addr;
	int err;

	/* check whether it's halted */
	val = pcntrl_read_reg(ppc, PCTRL_CONTROL);
	if ((val & CONTROL_RUNSTATE) != 0) {
		dev_err(dev, "PRU#%d not halted\n",
				ppc->idx);
		return -EINVAL;
	}

	/* read the instruction */
	addr = pcntrl_read_reg(ppc, PCTRL_STATUS) << 2;
	err = pru_i_read_u32(ppc, addr, &val);
	if (err != 0) {
		dev_err(dev, "PRU#%d halted PC 0x%x bad\n", ppc->idx, addr);
		return err;
	}

	/* check whether it's a halt instruction */
	if (val != PRU_HALT_INSN) {
		dev_err(dev, "PRU#%d not in halt insn (addr=0x%x val 0x%08x)\n",
				ppc->idx, addr, val);
		return -EFAULT;
	}

	if (addrp != NULL)
		*addrp = addr;

	return 0;
}

static u32 pru_read_cpu_reg(struct pruproc_core *ppc, int reg)
{
	return pdbg_read_reg(ppc, PDBG_GPREG(reg));
}

static void pru_write_cpu_reg(struct pruproc_core *ppc, int reg, u32 val)
{
	pdbg_write_reg(ppc, PDBG_GPREG(reg), val);
}

/* it is assumed that the PRU is halted */
static void pru_resume(struct pruproc_core *ppc, u32 addr)
{
	u32 val;

	val = pcntrl_read_reg(ppc, PCTRL_CONTROL);
	val &= 0xffff;
	val |= ((addr >> 2) << 16) | CONTROL_ENABLE;
	val &= ~CONTROL_SOFT_RST_N;
	pcntrl_write_reg(ppc, PCTRL_CONTROL, val);
}

/* handle a PRU syscall */
static int pru_handle_syscall(struct pruproc_core *ppc)
{
	struct pruproc *pp = ppc->pruproc;
	struct device *dev = &pp->pdev->dev;
	u32 addr, scno, arg0, arg1, arg2, ret;
	int valid_sc;
	void * __iomem va;
	struct pru_vring_info *vri;
	struct pru_dev_vring_info *dvri;

	ret = pru_is_halted(ppc, &addr);
	if (ret != 0) {
		dev_err(dev, "PRU#%d not halted\n", ppc->idx);
		return ret;
	}

	valid_sc = 0;
	scno = pru_read_cpu_reg(ppc, 14);
	arg0 = pru_read_cpu_reg(ppc, 15);
	arg1 = pru_read_cpu_reg(ppc, 16);
	arg2 = pru_read_cpu_reg(ppc, 17);
	ret  = 0;	/* by default we return 0 */

	printk(KERN_INFO "pru_handle_syscall - entering switch. value scno (r14) : %d\n", scno);

	switch (scno) {
		case PRU_SC_HALT:
			dev_info(dev, "P%d HALT\n",
				ppc->idx);
			return 1;

		case PRU_SC_PUTC:
			dev_info(dev, "P%d PUTC '%c'\n",
				ppc->idx, (char)(arg0 & 0xff));
			break;

		case PRU_SC_EXIT:
			dev_info(dev, "P%d EXIT %d\n",
				ppc->idx, (int)arg0);
			return 1;

		case PRU_SC_PUTS:
			/* pointers can only be in own data ram */
			va = pru_d_da_to_va(ppc, arg0, NULL);
			if (va == NULL) {
				dev_err(dev, "PRU#%d SC PUTS bad 0x%x\n",
						ppc->idx, arg0);
				ret = (u32)-1;
			} else {
				dev_info(dev, "P%d PUTS '%s'\n",
					ppc->idx, (char *)va);
			}
			break;

		case PRU_SC_GET_CFG:
			switch (arg0) {
				case PRU_SC_GET_CFG_VRING_NR:
					ret = ppc->num_vrings;	/* two rings */
					dev_dbg(dev, "P%d GET_CFG VRING_NR %d\n",
						ppc->idx, (int)ret);
					break;
				case PRU_SC_GET_CFG_VRING_INFO:
					if (arg1 >= ppc->num_vrings) {
						dev_err(dev, "PRU#%d SC "
							"GET_CFG_VRING_INFO "
							"bad idx %d\n",
							ppc->idx, arg1);
					}

					va = pru_d_da_to_va(ppc, arg2, NULL);
					if (va == NULL) {
						dev_err(dev, "PRU#%d SC "
							"GET_CFG_VRING_INFO "
							"bad 0x%x\n",
							ppc->idx, arg2);
						ret = (u32)-1;
						break;
					}
					dvri = va;
					vri = &ppc->vring_info[arg1];

					/* fill it in */
					dvri->paddr = vri->pa;
					dvri->num = vri->rsc->num;
					dvri->align = vri->rsc->align;
					dvri->pad = 0;

					dev_dbg(dev, "P%d GET_CFG VRING_INFO %d\n",
						ppc->idx, (int)ret);
					break;

				case PRU_SC_GET_CFG_RESOURCE_TABLE:
					/* just return the physical address */
					ret = (u32)ppc->dev_table_pa;
					dev_dbg(dev, "P%d GET_CFG RESOURCE_TABLE 0x%x\n",
						ppc->idx, ret);
					break;

				default:
					dev_err(dev, "PRU#%d SC "
						"GET_CFG bad 0x%x\n",
						ppc->idx, arg1);
					ret = (u32)-1;
					break;
			}
			break;

		case PRU_SC_DOWNCALL_READY:
			/* if we were waiting for it, wake up */

			printk(KERN_INFO "#1 Function pru_handle_syscall - pru idx:%d, dc_flags : %lu\n", ppc->idx, ppc->dc_flags);

			if (test_and_clear_bit(PRU_DCF_DOWNCALL_REQ, &ppc->dc_flags)) {
				set_bit(PRU_DCF_DOWNCALL_ACK, &ppc->dc_flags);
				wake_up_interruptible(&ppc->dc_waitq);
				printk(KERN_INFO "#1.5 Function pru_handle_syscall, dc_flags : %lu\n", ppc->dc_flags);
				return 1;
			}
			dev_err(dev, "P%d No-one expected downcall; halting\n",
					ppc->idx);
			return 1;

		case PRU_SC_DOWNCALL_DONE:
			/* if we were waiting for it, wake up */

                        printk(KERN_INFO "#2 Function pru_handle_syscall - pru idx:%d, dc_flags : %lu\n", ppc->idx, ppc->dc_flags);

			if (test_and_clear_bit(PRU_DCF_DOWNCALL_ISSUE, &ppc->dc_flags)) {
				set_bit(PRU_DCF_DOWNCALL_DONE, &ppc->dc_flags);
				wake_up_interruptible(&ppc->dc_waitq);
				printk(KERN_INFO "#2.5 Function pru_handle_syscall, dc_flags : %lu\n", ppc->dc_flags);
				return 1;
			}
			dev_err(dev, "P%d No-one expected downcall; halting\n",
					ppc->idx);
			return 1;

		default:
			dev_err(dev, "PRU#%d SC Unknown (%d)\n",
				ppc->idx, scno);
			return 1;
	}

	/* return code */
	pru_write_cpu_reg(ppc, 14, ret);

	/* skip over the HALT insn */
	pru_resume(ppc, addr + 4);

	return 0;
}

/*
 * The source of the downcall part on the PRU
 *
 *         .global sc_downcall
 *sc_downcall:
 *	MOV R0.w0, R14.w0               ;* save the pointer to the function
 *	;* first issue the downcall ready
 *	LDI R14, DOWNCALL_READY
 *	LDI R31, SYSCALL_VALUE
 *	HALT                            ;* host must save R3.w0 locally
 *	;* the host will manipulate our state so that the arguments are correct
 *	JAL R3.w0, R0.w0                ;* call
 *	MOV R0, R14                     ;* save the return code
 *	;* when we return here, we will inform the host of the result
 *	LDI R14, DOWNCALL_DONE          ;
 *	LDI R31, SYSCALL_VALUE
 *	HALT                            ;* host must return to save R3.w0
 */

/* perform the downcall */
static int pru_downcall(struct pruproc_core *ppc,
		u32 nr, u32 arg0, u32 arg1, u32 arg2, u32 arg3, u32 arg4)
{
	struct pruproc *pp = ppc->pruproc;
	struct device *dev = &pp->pdev->dev;
	int sysint;
	int ret;
	long intr;
	u32 addr, r3in;

	sysint = pp->target_to_sysev[TARGET_ARM_TO_PRU_IDX(ppc->idx)];
	if (sysint == -1)
		return -EINVAL;

	/* we might sleep, warn with a backtrace */
	might_sleep();

	mutex_lock(&ppc->dc_lock);

	/* state machine out of sync */
	if (ppc->dc_flags != 0) {
		ret = -EBUSY;
		goto ret_unlock;
	}

	printk(KERN_INFO "#1 Function pru_downcall - pru idx:%d, sysevent : %d, dc_flags : %lu\n", ppc->idx, sysint, ppc->dc_flags);

	if (test_and_set_bit(PRU_DCF_DOWNCALL_REQ, &ppc->dc_flags) != 0) {
		dev_err(dev, "PRU#%d downcall failed due to mangled req bit\n",
				ppc->idx);
		ret = -EBUSY;
		goto ret_unlock;
	}

	printk(KERN_INFO "#2 Function pru_downcall - pru idx:%d, sysevent : %d, dc_flags : %lu\n", ppc->idx, sysint, ppc->dc_flags);

	/* signal downcall event */
	if (sysint < 32)
		pintc_write_reg(pp, PINTC_SRSR0, 1 << sysint);
	else
		pintc_write_reg(pp, PINTC_SRSR1, 1 << (sysint - 32));

	/* now waiting until we get the downcall ready (maximum 100ms) */
	intr = wait_event_interruptible_timeout(ppc->dc_waitq,
		test_and_clear_bit(PRU_DCF_DOWNCALL_ACK, &ppc->dc_flags),
		HZ / 10);
	if (intr < 0) {
		ret = (int)intr;
		dev_err(dev, "PRU#%d error waiting for downcall ready (%d)\n",
				ppc->idx, ret);
		goto ret_call_failed;
	}
	if (intr == 0) {
		dev_err(dev, "PRU#%d failed to issue downcall ready in 100ms\n",
				ppc->idx);
		ret = -ETIMEDOUT;
		goto ret_call_failed;
	}
	dev_dbg(dev, "PRU#%d got downcall ready\n", ppc->idx);

	printk(KERN_INFO "#3 Function pru_downcall - Got downcall ready, dc_flags : %lu\n", ppc->dc_flags);

	ret = pru_is_halted(ppc, &addr);
	if (ret != 0) {
		dev_err(dev, "PRU#%d not halted\n",
				ppc->idx);
		ret = -EFAULT;
		goto ret_call_failed;
	}

	/* get the actual return address */
	//r3in = pru_read_cpu_reg(ppc, 3) << 2;
	r3in = (pru_read_cpu_reg(ppc, 3) >> 16) << 2;

	/* write the arguments */
	pru_write_cpu_reg(ppc, 14, nr);
	pru_write_cpu_reg(ppc, 15, arg0);
	pru_write_cpu_reg(ppc, 16, arg1);
	pru_write_cpu_reg(ppc, 17, arg2);
	pru_write_cpu_reg(ppc, 18, arg3);
	pru_write_cpu_reg(ppc, 19, arg4);

	set_bit(PRU_DCF_DOWNCALL_ISSUE, &ppc->dc_flags);

	/* skip over the HALT insn */
	pru_resume(ppc, addr + 4);

	printk(KERN_INFO "#4 Function pru_downcall - set up args done, dc_flags : %lu\n",  ppc->dc_flags);

	/* now waiting until we get the downcall ready (maximum 100ms) */
	intr = wait_event_interruptible_timeout(ppc->dc_waitq,
		test_and_clear_bit(PRU_DCF_DOWNCALL_DONE, &ppc->dc_flags),
		HZ / 10);
	if (intr < 0) {
		ret = (int)intr;
		dev_err(dev, "PRU#%d error waiting for downcall done (%d)\n",
				ppc->idx, ret);
		goto ret_call_failed;
	}
	if (intr == 0) {
		dev_err(dev, "PRU#%d failed to issue downcall done in 100ms\n",
				ppc->idx);
		ret = -ETIMEDOUT;
		goto ret_call_failed;
	}
	dev_dbg(dev, "PRU#%d got downcall done\n", ppc->idx);

	printk(KERN_INFO "#4 Function pru_downcall - downcall done. reading return value, dc_flags : %lu\n",  ppc->dc_flags);

	/* return */
	ret = pru_read_cpu_reg(ppc, 0);

	printk(KERN_INFO "RETURN value of pru_downcall : %d\n", ret);

	/* and we're done */
	pru_resume(ppc, r3in);

ret_call_failed:
	ppc->dc_flags = 0;

ret_unlock:
	mutex_unlock(&ppc->dc_lock);

	return ret;
}

static int pru_downcall_idx(struct pruproc *pp, int idx,
		u32 nr, u32 arg0, u32 arg1, u32 arg2, u32 arg3, u32 arg4)
{
	struct pruproc_core *ppc = NULL;
	int i;

	for (i = 0; i < pp->num_prus; i++) {
		ppc = pp->pruc[i];
		if (ppc->idx == idx)
			break;
	}
	if (i >= pp->num_prus)
		return -EINVAL;

	return pru_downcall(ppc, nr, arg0, arg1, arg2, arg3, arg4);
}

static irqreturn_t pru_handler(int irq, void *data)
{
	struct pruproc *pp = data;
	struct pruproc_core *ppc;
	struct pru_sysev_target *pst;
	struct rproc *rproc;
	struct device *dev = &pp->pdev->dev;
	int pru_idx, i, ev, sysint, handled, ret;
	struct pru_vring_info *vri;
	u32 val;

	/* find out which IRQ we got */
	for (i = 0; i < pp->num_irqs; i++)
		if (irq == pp->irqs[i])
			break;
	if (i >= pp->num_irqs)
		return IRQ_NONE;

	ev = pp->events[i];

	/* first, check whether the interrupt is enabled */
	val = pintc_read_reg(pp, PINTC_HIER);
	if ((val & (1 << ev)) == 0)
		return IRQ_NONE;

	/* check non-pending bit of specific event */
	val = pintc_read_reg(pp, PINTC_HIPIR0 + (ev << 2));
	if ((val & HIPIR_NOPEND) != 0)
		return IRQ_NONE;

	sysint = val & 0x3f;

	/* clear event */
	if (sysint < 32)
		pintc_write_reg(pp, PINTC_SECR0, 1 << sysint);
	else
		pintc_write_reg(pp, PINTC_SECR1, 1 << (sysint - 32));

	/* get the source of the sysevent */
	pst = &pp->sysev_to_target[sysint];
	if (pst->valid == 0 || pst->source < -1 || pst->target < -1) {
		dev_warn(dev, "sysevent not handled %d; disabling\n",
				sysint);
		goto disable_int;
	}

	/* target self? not handled */
	if (pst->source == TARGET_ARM) {
		dev_warn(dev, "sysevent %d has host as source; disabling\n",
				sysint);
		goto disable_int;
	}

	/* find out which PRU was it */
	pru_idx = TARGET_PRU_TO_PRU_IDX(pst->source);
	ppc = pp->pru_to_pruc[pru_idx];
	if (ppc == NULL) {
		dev_warn(dev, "systevent %d from bad PRU; disabling\n",
				sysint);
		goto disable_int;
	}

	/* ok, got the source PRU */
	rproc = ppc->rproc;

	handled = 0;

	/* we either handle a vring or not */
	if (!pst->vring) {
		ret = pru_handle_syscall(ppc);
		if (ret >= 0) 	/* system call handled */
			handled++;
	} else {
		/* handle any vrings action */
		for (i = 0; i < ppc->num_vrings; i++) {
			vri = &ppc->vring_info[i];
			if (vri->rvring == NULL)
				continue;
			ret = rproc_vq_interrupt(rproc, vri->rvring->notifyid);
			if (ret == IRQ_HANDLED) {
				dev_dbg(dev, "PRU#%d; vring irq handled\n",
						ppc->idx);
				handled++;
			}

		}
	}

	if (!handled) {
		dev_err(dev, "sysint not handled; disabling interrupt\n");
		return IRQ_NONE;

	}

	return IRQ_HANDLED;

disable_int:
	/* disable the interrupt */
	pintc_write_reg(pp, PINTC_HIDISR, ev);
	return IRQ_HANDLED;
}

static int build_rsc_table(struct platform_device *pdev,
		struct device_node *node, struct pruproc_core *ppc)
{
	struct device *dev = &pdev->dev;
	struct device_node *rnode = NULL;	/* resource table node */
	struct device_node *rvnode = NULL;	/* vdev node */
	struct resource_table *rsc;
	struct fw_rsc_hdr *rsc_hdr;
	struct fw_rsc_vdev *rsc_vdev;
	struct pru_vring_info *vri;
	char vring_name[16];
	u32 vring_data[4], val;
	struct fw_rsc_vdev_vring *rsc_vring;
	void *table, *p;
	int i, err, table_size, num_vrings, num_vdevs, vdev_idx, vring_start;

	/* verify OF data */

	/* first find a valid resource-table node */
	for_each_child_of_node(node, rnode) {
		if (of_property_read_bool(rnode, "resource-table"))
			break;
	}

	/* no resource node found */
	if (rnode == NULL) {
		dev_warn(dev, "No resource-table node node; slave PRU\n");
		return 0;
	}

	/* count number of vdevs & vrings */

	table_size = sizeof(struct resource_table);

	num_vdevs = 0;
	num_vrings = 0;
	for_each_child_of_node(rnode, rvnode) {

		if (!of_property_read_bool(rvnode, "vdev-rproc-serial") &&
			!of_property_read_bool(rvnode, "vdev-rpmsg"))
			continue;

		/* size per each vdev */
		table_size += sizeof(u32) +
		     sizeof(struct fw_rsc_hdr) +
		     sizeof(struct fw_rsc_vdev);

		ppc->vdev_vring_start[num_vdevs] = num_vrings;
		for (i = 0; num_vrings < ARRAY_SIZE(ppc->vring_info); i++) {
			snprintf(vring_name, sizeof(vring_name), "vring-%d", i);
			if (of_property_read_u32_array(rvnode, vring_name,
					vring_data, ARRAY_SIZE(vring_data)) != 0)
				break;
			num_vrings++;
		}
		ppc->vdev_vring_count[num_vdevs] = i;

		/* size for the rings */
		table_size += i * sizeof(struct fw_rsc_vdev_vring);

		num_vdevs++;
	}

	dev_info(dev, "PRU%d #%d vdevs, #%d vrings total\n",
			ppc->idx, num_vdevs, num_vrings);

	table = devm_kzalloc(dev, table_size, GFP_KERNEL);
	if (table == NULL) {
		dev_err(dev, "Failed to allocate resource table\n");
		err = -ENOMEM;
		goto err_fail;
	}
	ppc->table = table;
	ppc->table_size = table_size;
	ppc->num_vdevs = num_vdevs;
	ppc->num_vrings = num_vrings;

	p = table;	/* pointer at start */

	/* resource table */
	rsc = p;
	p += sizeof(*rsc);
	rsc->ver = 1;	/* resource table version 1 */
	rsc->num = ppc->num_vdevs;

	p += rsc->num * sizeof(u32);	/* point after offsets */

	vdev_idx = 0;

	/* now loop over the vdevs */
	for_each_child_of_node(rnode, rvnode) {

		if (!of_property_read_bool(rvnode, "vdev-rproc-serial") &&
			!of_property_read_bool(rvnode, "vdev-rpmsg"))
			continue;

		rsc_hdr = p;
		rsc->offset[vdev_idx] = p - table;
		/* resource header */
		p += sizeof(*rsc_hdr);

		rsc_hdr->type = RSC_VDEV;

		/* vdev */
		rsc_vdev = p;
		p += sizeof(*rsc_vdev);
		ppc->rsc_vdev[vdev_idx] = rsc_vdev;

		if (of_property_read_bool(rvnode, "vdev-rproc-serial")) {
			rsc_vdev->id = VIRTIO_ID_RPROC_SERIAL;
			/* extra configuration possible here */
		} else if (of_property_read_bool(rvnode, "vdev-rpmsg")) {
			rsc_vdev->id = VIRTIO_ID_RPMSG;
			/* extra configuration possible here */
		}

		err = of_property_read_u32(rvnode, "notifyid", &val);
		if (err != 0) {
			dev_err(dev, "no notifyid vdev property\n");
			goto err_fail;
		}
		rsc_vdev->notifyid = val;
		rsc_vdev->dfeatures = 0;
		rsc_vdev->gfeatures = 0;
		rsc_vdev->config_len = 0;
		rsc_vdev->status = 0;

		/* start and count for vrings for this vdev */
		vring_start = ppc->vdev_vring_start[vdev_idx];
		num_vrings = ppc->vdev_vring_count[vdev_idx];

		rsc_vdev->num_of_vrings = num_vrings;

		for (i = 0; i < num_vrings; i++) {
			rsc_vring = p;
			p += sizeof(*rsc_vring);

			vri = &ppc->vring_info[i + vring_start];

			vri->rsc = rsc_vring;

			snprintf(vring_name, sizeof(vring_name), "vring-%d",
					i);
			err = of_property_read_u32_array(rvnode, vring_name,
					vring_data, ARRAY_SIZE(vring_data));
			if (err != 0) {
				dev_err(dev, "no %s property\n", vring_name);
				goto err_fail;
			}
			rsc_vring->da = vring_data[0];
			rsc_vring->align = vring_data[1];
			rsc_vring->num = vring_data[2];
			rsc_vring->notifyid = vring_data[3];
		}

		vdev_idx++;
	}

	/* all done, create the device copy */
	ppc->dev_table_va = dma_alloc_coherent(dev,
			PAGE_ALIGN(ppc->table_size), &ppc->dev_table_pa,
			GFP_KERNEL);
	if (ppc->dev_table_va == NULL) {
		dev_err(dev, "Failed to dma_alloc dev resource table\n");
		err = -ENOMEM;
		goto err_fail;
	}

	err = 0;

err_fail:
	of_node_put(rnode);

	return err;
}

static int read_map_property(struct device *dev,
		struct device_node *node, const char *propname,
		int *map, int max_idx, int max_val)
{
	struct property *prop;
	int i, idx, val, cnt, proplen, err;
	u32 *arr, *p;

	/* check node & propname */
	if (node == NULL || propname == NULL) {
		dev_err(dev, "Bad arguments\n");
		return -EINVAL;
	}

	/* find property */
	prop = of_find_property(node, propname, &proplen);
	if (prop == NULL) {
		dev_err(dev, "Can't find %s property\n", propname);
		return -ENOENT;
	}

	/* verify valid size (must be pairs of u32 items) */
	if ((proplen % (sizeof(u32) * 2)) != 0) {
		dev_err(dev, "Bad length (%d) of %s property\n",
				proplen, propname);
		return -EINVAL;
	}

	/* allocate temporary buffer */
	arr = devm_kzalloc(dev, proplen, GFP_KERNEL);
	if (arr == NULL) {
		dev_err(dev, "Alloc failed on %s property\n", propname);
		return -ENOMEM;
	}

	/* the number of pairs */
	cnt = proplen / (sizeof(arr[0]) * 2);

	/* now read it */
	err = of_property_read_u32_array(node, propname, arr, cnt * 2);
	if (err != 0) {
		dev_err(dev, "Failed to read %s property\n", propname);
		return err;
	}

	/* now read pairs and fill in the map */
	for (i = 0, p = arr; i < cnt; i++, p += 2) {
		idx = p[0];
		val = p[1];
		if ((unsigned int)idx >= max_idx) {
			dev_err(dev, "%s[%d] bad map idx %d\n",
					propname, i, idx);
			return err;
		}
		if ((unsigned int)val >= max_val) {
			dev_err(dev, "%s[%d] bad map val %d\n",
					propname, i, val);
			return err;
		}
		/* fill in map */
		map[idx] = val;
		dev_info(dev, "%s [%d] <- %d\n", propname, idx, val);
	}

	devm_kfree(dev, arr);

	return 0;
}

static int configure_pintc(struct platform_device *pdev, struct pruproc *pp)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	int err, i, idx, ch, host;
	uint64_t sysevt_mask;
	uint32_t ch_mask;
	uint32_t host_mask;
	u32 val;

	/* retreive the maps */
	err = read_map_property(dev, node, "sysevent-to-channel-map",
			pp->sysev_to_ch, ARRAY_SIZE(pp->sysev_to_ch),
			MAX_PRU_CHANNELS);
	if (err != 0)
		return err;

	err = read_map_property(dev, node, "channel-to-host-interrupt-map",
			pp->ch_to_host, ARRAY_SIZE(pp->ch_to_host),
			MAX_PRU_HOST_INT);
	if (err != 0)
		return err;

	err = of_property_read_u32_array(node, "target-to-sysevent-map",
			pp->target_to_sysev, ARRAY_SIZE(pp->target_to_sysev));
	if (err != 0)
		return err;

	/* now configure the pintc appropriately */

	/* configure polarity and type (all active high & pulse) */
	pintc_write_reg(pp, PINTC_SIPR0, 0xffffffff);
	pintc_write_reg(pp, PINTC_SIPR1, 0xffffffff);

	/* clear all channel mapping registers */
	for (i = PINTC_CMR0; i <= PINTC_CMR15; i += 4)
		pintc_write_reg(pp, i, 0);

	sysevt_mask = 0;
	ch_mask = 0;
	host_mask = 0;

	/* set channel mapping registers we have */
	for (i = 0; i < ARRAY_SIZE(pp->sysev_to_ch); i++) {

		ch = pp->sysev_to_ch[i];
		if (ch < 0)
			continue;

		/* CMR format: ---CH3---CH2---CH1---CH0 */

		/* 4 settings in each register */
		idx = i / 4;

		/* update CMR entry */
		val  = pintc_read_reg(pp, PINTC_CMR0 + idx * 4);
		val |= (u32)ch << ((i & 3) * 8);
		pintc_write_reg(pp, PINTC_CMR0 + idx * 4, val);

		/* set bit in the sysevent mask */
		sysevt_mask |= 1LLU << i;
		/* set bit in the channel mask */
		ch_mask |= 1U << ch;

		dev_dbg(dev, "SYSEV%d -> CH%d (CMR%d 0x%08x)\n",
				i, ch, idx,
				pintc_read_reg(pp, PINTC_CMR0 + idx * 4));
	}

	/* clear all host mapping registers */
	for (i = PINTC_HMR0; i <= PINTC_HMR2; i += 4)
		pintc_write_reg(pp, i, 0);

	/* set host mapping registers we have */
	for (i = 0; i < ARRAY_SIZE(pp->ch_to_host); i++) {

		host = pp->ch_to_host[i];
		if (host < 0)
			continue;

		/* HMR format: ---HI3---HI2---HI1---HI0 */

		/* 4 settings in each register */
		idx = i / 4;

		/* update HMR entry */
		val  = pintc_read_reg(pp, PINTC_HMR0 + idx * 4);
		val |= (u32)host << ((i & 3) * 8);
		pintc_write_reg(pp, PINTC_HMR0 + idx * 4, val);

		/* set bit in the channel mask */
		ch_mask |= 1U << i;
		/* set bit in the sysevent mask */
		host_mask |= 1U << host;

		dev_dbg(dev, "CH%d -> HOST%d (HMR%d 0x%08x)\n",
				i, host, idx,
				pintc_read_reg(pp, PINTC_HMR0 + idx * 4));
	}

	/* configure polarity and type (all active high & pulse) */
	pintc_write_reg(pp, PINTC_SITR0, 0);
	pintc_write_reg(pp, PINTC_SITR1, 0);

	dev_dbg(dev, "sysevt_mask=0x%016llx ch_mask=0x%08x host_mask=0x%08x\n",
			sysevt_mask, ch_mask, host_mask);

	/* enable sys-events */
	pintc_write_reg(pp, PINTC_ESR0, (u32)sysevt_mask);
	pintc_write_reg(pp, PINTC_SECR0, (u32)sysevt_mask);
	pintc_write_reg(pp, PINTC_ESR1, (u32)(sysevt_mask >> 32));
	pintc_write_reg(pp, PINTC_SECR1, (u32)(sysevt_mask >> 32));

	/* enable host interrupts */
	for (i = 0; i < MAX_PRU_HOST_INT; i++) {
		if ((host_mask & (1 << i)) != 0)
			pintc_write_reg(pp, PINTC_HIEISR, i);
	}

	/* global interrupt enable */
	pintc_write_reg(pp, PINTC_GER, 1);

	return 0;
}

static int pru_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			     int duty_ns, int period_ns)
{
	struct device *dev = chip->dev;
	struct platform_device *pdev = to_platform_device(dev);
	struct pruproc *pp = platform_get_drvdata(pdev);
	int hwpwm, prupwm, ret;
	u32 hi, lo;

	hwpwm = pwm->hwpwm;
	if (hwpwm >= pp->pwm.count)
		return -EINVAL;
	prupwm = pp->pwm.map[hwpwm];

	dev_dbg(&pdev->dev, "%s (%d/%d) duty_ns=%d period_ns=%d\n", __func__,
			hwpwm, prupwm, duty_ns, period_ns);

	hi = div_u64((u64)duty_ns * pp->clock_freq, 1000000000);
	lo = div_u64((u64)(period_ns - duty_ns) * pp->clock_freq, 1000000000);
	ret = pru_downcall_idx(pp, pp->pwm.controller,
			pp->pwm.dc_ids[DC_PWM_CONFIG], prupwm, hi, lo, 0, 0);
	return ret;
}

static int pru_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct device *dev = chip->dev;
	struct platform_device *pdev = to_platform_device(dev);
	struct pruproc *pp = platform_get_drvdata(pdev);
	int hwpwm, prupwm, ret;

	hwpwm = pwm->hwpwm;
	if (hwpwm >= pp->pwm.count)
		return -EINVAL;
	prupwm = pp->pwm.map[hwpwm];

	dev_dbg(&pdev->dev, "%s (%d/%d)\n", __func__, hwpwm, prupwm);

	ret = pru_downcall_idx(pp, pp->pwm.controller,
			pp->pwm.dc_ids[DC_PWM_ENABLE], prupwm, 0, 0, 0, 0);
	return ret;
}

static void pru_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct device *dev = chip->dev;
	struct platform_device *pdev = to_platform_device(dev);
	struct pruproc *pp = platform_get_drvdata(pdev);
	int hwpwm, prupwm;

	hwpwm = pwm->hwpwm;
	if (hwpwm >= pp->pwm.count)
		return;
	prupwm = pp->pwm.map[hwpwm];

	dev_dbg(&pdev->dev, "%s (%d/%d)\n", __func__, hwpwm, prupwm);

	pru_downcall_idx(pp, pp->pwm.controller,
			pp->pwm.dc_ids[DC_PWM_DISABLE], prupwm, 0, 0, 0, 0);
}

static const struct pwm_ops pru_pwm_ops = {
	.config = pru_pwm_config,
	.enable = pru_pwm_enable,
	.disable = pru_pwm_disable,
};

static int pruproc_create_pwm_devices(struct pruproc *pp)
{
	struct platform_device *pdev = pp->pdev;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct property *prop;
	u32 val;
	int err, i, cnt, proplen;

	/* pwms */

	/* find property */
	prop = of_find_property(np, "pru-pwm-channels", &proplen);
	if (prop == NULL)
		return 0;

	cnt = proplen / sizeof(u32);
	if (cnt >= ARRAY_SIZE(pp->pwm.map)) {
		dev_err(dev, "Too many PWMs %d (max %d)\n",
				cnt, ARRAY_SIZE(pp->pwm.map));
		return -EINVAL;
	}
	pp->pwm.count = cnt;

	/* now read it */
	err = of_property_read_u32_array(np, "pru-pwm-channels",
			pp->pwm.map, cnt);
	if (err != 0) {
		dev_err(dev, "Failed to read %s property\n",
				"pru-pwm-channels");
		return err;
	}

	if (of_property_read_u32(np, "pru-pwm-controller", &val) == 0)
		pp->pwm.controller = val;
	else
		pp->pwm.controller = 0;	/* default is PRU0 */

	/* verify they are sane */
	for (i = 0; i < cnt; i++) {
		if (pp->pwm.map[i] >= ARRAY_SIZE(pp->pwm.map)) {
			dev_err(dev, "Bad PWM number\n");
			return -EINVAL;
		}
	}

	/* get the PWM DC IDs */
	err = of_property_read_u32_array(np, "pru-pwm-dc-ids",
			pp->pwm.dc_ids, ARRAY_SIZE(pp->pwm.dc_ids));
	if (err != 0) {
		dev_err(dev, "Failed to read %s array\n",
				"pru-pwm-dc-ids");
		return err;
	}

	pp->pwm.chip.dev = dev;
	pp->pwm.chip.ops = &pru_pwm_ops;
	pp->pwm.chip.base = pdev->id;	/* ? */
	pp->pwm.chip.npwm = cnt;

	/* add the pwms */
	err = pwmchip_add(&pp->pwm.chip);
	if (err != 0) {
		dev_err(dev, "pwmchip_add failed\n");
		return err;
	}

	return 0;
}

/* after all is configured create the linux devices */
static int pruproc_create_devices(struct pruproc *pp)
{
	int ret;

	ret = pruproc_create_pwm_devices(pp);

	return ret;
}


static int pruproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct device_node *pnode = NULL;
	struct pruproc *pp;
	struct pruproc_core *ppc;
	struct pru_sysev_target *pst;
	const char *fw_name;
	int pm_get = 0;
	struct rproc *rproc = NULL;
	struct resource *res;
	struct pinctrl *pinctrl;
	int err, i, j, irq, sysev, x;
	u32 tmparr[4], pru_idx, val;
	u32 tmpev[MAX_ARM_PRU_INTS];
	int shm_count = 0;

	/* get pinctrl */
	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl)) {
		err = PTR_ERR(pinctrl);
		/* deferring probe */
		if (err == -EPROBE_DEFER) {
			dev_warn(dev, "deferring proble\n");
			return err;
		}
		dev_warn(dev, "pins are not configured from the driver\n");
	}

	/* we only work on OF */
	if (node == NULL) {
		dev_err(dev, "Only OF configuration supported\n");
		err = -ENODEV;
		goto err_fail;
	}

	pm_runtime_enable(dev);
	err = pm_runtime_get_sync(dev);
	if (err != 0) {
		dev_err(dev, "pm_runtime_get_sync failed\n");
		goto err_fail;
	}
	pm_get = 1;

	err = dma_set_coherent_mask(dev, DMA_BIT_MASK(32));
	if (err) {
		dev_err(dev, "dma_set_coherent_mask: %d\n", err);
		goto err_fail;
	}

	pp = devm_kzalloc(dev, sizeof(*pp), GFP_KERNEL);
	if (pp == NULL) {
		dev_err(dev, "failed to allocate pruproc\n");
		err = -ENOMEM;
		goto err_fail;

	}

	/* link the device with the pruproc */
	platform_set_drvdata(pdev, pp);
	pp->pdev = pdev;

	/* prepare the irqs */
	for (i = 0; i < ARRAY_SIZE(pp->irqs); i++)
		pp->irqs[i] = -1;

	/* prepare the events */
	for (i = 0; i < ARRAY_SIZE(pp->events); i++)
		pp->events[i] = -1;

	/* prepare the sysevevent to channel map */
	for (i = 0; i < ARRAY_SIZE(pp->sysev_to_ch); i++)
		pp->sysev_to_ch[i] = -1;

	/* prepare the channel to hostint map */
	for (i = 0; i < ARRAY_SIZE(pp->ch_to_host); i++)
		pp->ch_to_host[i] = -1;

	/* finally register the interrupts */
	for (i = 0; i < ARRAY_SIZE(pp->irqs); i++) {

		err = platform_get_irq(pdev, i);
		if (err < 0)
			break;
		irq = err;
		pp->irqs[i] = irq;
	}
	pp->num_irqs = i;
	dev_info(dev, "#%d PRU interrupts registered\n", pp->num_irqs);

	/* make sure this fits */
	if (pp->num_irqs > ARRAY_SIZE(tmpev)) {
		dev_err(dev, "Too many irqs (%d)\n", pp->num_irqs);
		goto err_fail;
	}

	/* read the events (host ints) */
	err = of_property_read_u32_array(node, "events", tmpev, pp->num_irqs);
	if (err != 0) {
		dev_err(dev, "Failed to read events array\n");
		goto err_fail;
	}

	/* assign and check */
	for (i = 0; i < pp->num_irqs; i++) {
		if (tmpev[i] < MIN_PRU_HOST_INT ||
				tmpev[i] >= MAX_PRU_HOST_INT) {
			dev_err(dev, "Bad event property entry %u\n", tmpev[i]);
			goto err_fail;
		}
		pp->events[i] = tmpev[i];
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(dev, "failed to parse MEM resource\n");
		goto err_fail;
	}

	pp->paddr = res->start;
	pp->vaddr = devm_ioremap(dev, res->start, resource_size(res));
	if (pp->vaddr == NULL) {
		dev_err(dev, "failed to parse MEM resource\n");
		goto err_fail;
	}

	err = of_property_read_u32(node, "pintc", &pp->pintc);
	if (err != 0) {
		dev_err(dev, "no pintc property\n");
		goto err_fail;
	}

	/* read pdram property global, size, local */
	err = of_property_read_u32_array(node, "pdram", tmparr, 3);
	if (err != 0) {
		dev_err(dev, "no pintc property\n");
		goto err_fail;
	}
	pp->pdram    = tmparr[0];
	pp->pdram_sz = tmparr[1];
	pp->pdram_da = tmparr[2];

	/* get the clock frequency */
	err = of_property_read_u32(node, "clock-freq", &val);
	if (err != 0) {
		dev_warn(dev, "no clock-freq property; assuming default 200MHz\n");
		val = 200000000;
	}
	pp->clock_freq = val;

	/* configure PRU interrupt controller from DT */
	err = configure_pintc(pdev, pp);
	if (err != 0) {
		dev_err(dev, "failed to configure pintc\n");
		goto err_fail;
	}

	/* count number of child nodes with a firmwary property */
	pp->num_prus = 0;
	for_each_child_of_node(node, pnode) {
		if (of_find_property(pnode, "firmware", NULL))
			pp->num_prus++;
	}
	pnode = NULL;

	/* found any? */
	if (pp->num_prus == 0) {
		dev_err(dev, "no pru nodes found\n");
		err = -EINVAL;
		goto err_fail;
	}
	/* found too many? */
	if (pp->num_prus > MAX_PRUS) {
		dev_err(dev, "Only 2 PRU nodes are supported\n");
		err = -EINVAL;
		goto err_fail;
	}
	dev_info(dev, "found #%d PRUs\n", pp->num_prus);

	/* allocate pointers */
	pp->pruc = devm_kzalloc(dev, sizeof(*pp->pruc) * pp->num_prus,
			GFP_KERNEL);
	if (pp->pruc == NULL) {
		dev_err(dev, "Failed to allocate PRU table\n");
		err = -ENOMEM;
		goto err_fail;
	}

	/* now iterate over all the pru nodes */
	i = 0;
	for_each_child_of_node(node, pnode) {

		/* only nodes with firmware are PRU nodes */
		if (of_find_property(pnode, "firmware", NULL) == NULL)
			continue;

		/* get the hardware index of the PRU */
		err = of_property_read_u32(pnode, "pru-index", &pru_idx);
		if (err != 0) {
			dev_err(dev, "can't find property %s\n", "pru-index");
			of_node_put(pnode);
			goto err_fail;
		}

		/* verify the index */
		if (pru_idx >= MAX_PRUS) {
			dev_err(dev, "Illegal pru-index property %u\n",
					pru_idx);
			of_node_put(pnode);
			goto err_fail;
		}

		/* get the firmware */
		err = of_property_read_string(pnode, "firmware", &fw_name);
		if (err != 0) {
			dev_err(dev, "can't find property %s\n", "firmware");
			of_node_put(pnode);
			goto err_fail;
		}

		/* allocate the remote proc + our private data */
		rproc = rproc_alloc(dev, pdev->name, &pruproc_ops, fw_name,
				sizeof(*ppc));
		if (!rproc) {
			dev_err(dev, "rproc_alloc failed\n");
			err = -ENOMEM;
			goto err_fail;
		}
		ppc = rproc->priv;
		ppc->idx = pru_idx;
		ppc->pruproc = pp;
		ppc->rproc = rproc;

		atomic_set(&ppc->bootcnt, 0);

		mutex_init(&ppc->dc_lock);
		init_waitqueue_head(&ppc->dc_waitq);

		err = of_property_read_u32_array(pnode, "iram", tmparr, 3);
		if (err != 0) {
			dev_err(dev, "no iram property\n");
			goto err_fail;
		}
		ppc->iram = tmparr[0];
		ppc->iram_sz = tmparr[1];
		ppc->iram_da = tmparr[2];

		err = of_property_read_u32_array(pnode, "dram", tmparr, 4);
		if (err != 0) {
			dev_err(dev, "no dram property\n");
			goto err_fail;
		}
		ppc->dram = tmparr[0];
		ppc->dram_sz = tmparr[1];
		ppc->dram_da = tmparr[2];
		ppc->dram_oda = tmparr[3];

		err = of_property_read_u32(pnode, "pctrl", &ppc->pctrl);
		if (err != 0) {
			dev_err(dev, "no pctrl property\n");
			goto err_fail;
		}

		err = of_property_read_u32(pnode, "pdbg", &ppc->pdbg);
		if (err != 0) {
			dev_err(dev, "no pdbg property\n");
			goto err_fail;
		}

		/* read vring sysevent array */
		err = of_property_read_u32_array(pnode, "vring-sysev", tmparr, 2);
		if (err == 0) {
			/* verify */
			if (tmparr[0] >= MAX_PRU_SYS_EVENTS ||
					tmparr[1] >= MAX_PRU_SYS_EVENTS) {
				dev_err(dev, "illegal vring-sysev property\n");
				goto err_fail;
			}
			ppc->pru_vring_sysev = tmparr[0];
			ppc->host_vring_sysev = tmparr[1];
		} else {
			ppc->pru_vring_sysev = -1;
			ppc->host_vring_sysev = -1;
		}

		/* check firmware type */
		ppc->is_elf = of_property_read_bool(pnode, "firmware-elf");

		/* build the resource table from DT */
		err = build_rsc_table(pdev, pnode, ppc);
		if (err != 0) {
			dev_err(dev, "failed to build resource table\n");
			goto err_fail;
		}

		/* Set the PRU specific firmware handler */
		if (!ppc->is_elf)
			rproc->fw_ops = &pruproc_bin_fw_ops;
		else
			rproc->fw_ops = &pruproc_elf_fw_ops;

		/* PRU Speak shared memory stuff */

		printk("Entering SHM section\n");
		err = of_property_read_u32(pnode, "shm-count", &shm_count);

		if (err != 0) {
			dev_err(dev, "can't find property %s\n", "shm-count");
			of_node_put(pnode);
			goto err_fail;
		}

		/* make sure the count is within limits expected */
		if ((shm_count < MIN_SHARED) || (shm_count > MAX_SHARED)){
			dev_err(dev, "expected device count min : %d, max : %d\n", MIN_SHARED, MAX_SHARED);
			of_node_put(pnode);
			goto err_fail;
		}

		err = of_property_read_u32_array(pnode, "shm-size", tmparr, shm_count);
		if (err != 0) {
			dev_err(dev, "no shm-size property\n");
			goto err_fail;
		}
		printk("Starting to initialize SHMs\n");
		/* assign idx, shared memory size for each segment for this pru */
		for(x=0; x < shm_count; x++){
			ppc->shm[x].size_in_pages = tmparr[x];
			ppc->shm[x].idx = pru_idx;

			/* mark valid each shm segment, allocate space for it*/
			printk("Initializing shm #%d for PRU%d \n", x, pru_idx);
			ppc->shm[x].is_valid = 1;
			ppc->shm[x].vaddr = dma_zalloc_coherent(dev, ppc->shm[x].size_in_pages * PAGE_SIZE,
									(dma_addr_t *) &(ppc->shm[x].paddr), GFP_DMA);

			if(!(ppc->shm[x].vaddr)){
				ppc->shm[x].is_valid = 0;
				printk("shm init failed\n");
			}
		}

		printk("SHM initalization sequence is now complete\n");
		/* important */
		pp->pruc[i] = ppc;
		pp->pru_to_pruc[pru_idx] = ppc;
		i++;
	}
	pnode = NULL;

	/* clean up the sysev to target map */
	printk("clean up the sysev to target map\n");
	memset(pp->sysev_to_target, 0, sizeof(pp->sysev_to_target));
	for (i = 0; i < ARRAY_SIZE(pp->sysev_to_target); i++) {
		pst = &pp->sysev_to_target[i];
		pst->source = -1;
		pst->target = -1;
	}

	/* fill in the sysev target map for std. signaling */
	printk("fill in the sysev target map for std. signaling\n");
	for (i = 0; i < MAX_TARGETS; i++) {
		for (j = 0; j < MAX_TARGETS; j++) {
			/* target self? don't care*/
			if (i == j)
				continue;
			sysev = pp->target_to_sysev[i * MAX_TARGETS + j];
			if (sysev >= MAX_PRU_SYS_EVENTS) {
				dev_err(dev, "Bad SYSEV %d\n", sysev);
				goto err_fail;
			}
			pst = &pp->sysev_to_target[sysev];
			if (pst->valid) {
				dev_err(dev, "SYSEV %d overlap\n", sysev);
				goto err_fail;
			}
			pst->source = i;
			pst->target = j;
			pst->vring = 0;
			pst->valid = 1;
		}
	}

	/* fill in the sysev target map from vring info */
	printk("fill in the sysev target map from vring info\n");
	for (i = 0; i < pp->num_prus; i++) {
		ppc = pp->pruc[i];
		pru_idx = ppc->idx;
		sysev = ppc->host_vring_sysev;
		if (sysev != -1) {
			pst = &pp->sysev_to_target[sysev];
			if (pst->valid) {
				dev_err(dev, "SYSEV %d overlap\n", sysev);
				goto err_fail;
			}
			pst->source = TARGET_PRU(pru_idx);
			pst->target = TARGET_ARM;
			pst->vring = 1;
			pst->valid = 1;
		}
		sysev = ppc->pru_vring_sysev;
		if (sysev != -1) {
			pst = &pp->sysev_to_target[sysev];
			if (pst->valid) {
				dev_err(dev, "SYSEV %d overlap\n", sysev);
				goto err_fail;
			}
			pst->source = TARGET_ARM;
			pst->target = TARGET_PRU(pru_idx);
			pst->vring = 1;
			pst->valid = 1;
		}
	}

	/* dump the sysev target map */
	printk("dump the sysev target map\n");
	for (i = 0; i < ARRAY_SIZE(pp->sysev_to_target); i++) {
		pst = &pp->sysev_to_target[i];
		if (!pst->valid)
			continue;
		dev_dbg(dev, "SYSEV#%d <- VR %d SRC %d TRG %d\n",
				i, pst->vring, pst->source, pst->target);
	}

	/* register the interrupts */
	printk("register the interrupts\n");
	for (i = 0; i < pp->num_irqs; i++) {

		irq = pp->irqs[i];
		err = devm_request_irq(dev, irq, pru_handler, 0,
				dev_name(dev), pp);
		if (err != 0) {
			dev_err(dev, "Failed to register irq %d\n", irq);
			goto err_fail;
		}
	}

	/* start the remote procs */
	printk("start the remote procs\n");

	for (i = 0; i < pp->num_prus; i++) {
		ppc = pp->pruc[i];

		/* Register as a remoteproc device */
		err = rproc_add(ppc->rproc);
		if (err) {
			dev_err(dev, "rproc_add failed\n");
			goto err_fail;
		}

		/* directly boot all processors that don't have VDEVs */
		if (ppc->num_vdevs == 0) {
			err = rproc_boot(ppc->rproc);
			if (err) {
				dev_err(dev, "rproc_boot failed\n");
				goto err_fail;
			}
		}
	}

	printk("Creating sysfs entries\n");

	err = device_create_file(dev, &dev_attr_load);
	if (err != 0) {
		dev_err(dev, "device_create_file failed\n");
		goto err_fail;
	}

	err = device_create_file(dev, &dev_attr_reset);
	if (err != 0) {
		dev_err(dev, "device_create_file failed\n");
		goto err_fail;
	}
/*
	err = device_create_file(dev, &dev_attr_downcall0);
	if (err != 0) {
		dev_err(dev, "device_create_file failed\n");
		goto err_fail;
	}

	err = device_create_file(dev, &dev_attr_downcall1);
	if (err != 0) {
		dev_err(dev, "device_create_file failed\n");
		goto err_fail;
	}
*/
	err = device_create_file(dev, &dev_attr_pru_speak_debug);
	if (err != 0) {
		dev_err(dev, "device_create_file failed\n");
		goto err_fail;
	}


	err = device_create_file(dev, &dev_attr_pru_speak_shm_init);
        if (err != 0) {
                dev_err(dev, "device_create_file failed\n");
                goto err_fail;
        }

	err = device_create_file(dev, &dev_attr_pru_speak_execute);
        if (err != 0) {
                dev_err(dev, "device_create_file failed\n");
                goto err_fail;
        }

	err = device_create_file(dev, &dev_attr_pru_speak_abort);
        if (err != 0) {
                dev_err(dev, "device_create_file failed\n");
                goto err_fail;
        }

        err = device_create_file(dev, &dev_attr_pru_speak_status);
        if (err != 0) {
                dev_err(dev, "device_create_file failed\n");
                goto err_fail;
        }

        err = device_create_file(dev, &dev_attr_pru_speak_single_cmd);
        if (err != 0) {
                dev_err(dev, "device_create_file failed\n");
                goto err_fail;
        }

	err = device_create_file(dev, &dev_attr_pru_speak_single_cmd_64);
        if (err != 0) {
                dev_err(dev, "device_create_file failed\n");
                goto err_fail;
        }

/*
	err = sysfs_create_bin_filr(&(dev->kobj), &pru_speak_bin_attr);
	if (err != 0){
                printk(KERN_INFO "BIN FILE could not be created");
                goto err_fail;
        }
	else{
		printk(KERN_INFO "PRU SPEAK bin attribute created\n");
	}
*/
	dev_info(dev, "Loaded OK\n");

	/* creating devices */
	pruproc_create_devices(pp);

	(void)pru_d_read_u32;
	(void)pru_i_write_u32;
	(void)pru_d_write_u32;

	printk("Probe successful");

	return 0;
err_fail:
	/* NULL is OK */
	of_node_put(pnode);

	if (rproc)
		rproc_put(rproc);
	if (pm_get)
		pm_runtime_disable(dev);
	return err;
}

static const struct of_device_id pru_speak_dt_ids[] = {
	{ .compatible = "ti,pru_speak", .data = NULL, },
	{},
};
//MODULE_DEVICE_TABLE(of, pruss_dt_ids);

static struct platform_driver pru_speak_driver = {
	.driver	= {
		.name	= "pru_speak",
		.owner	= THIS_MODULE,
		.of_match_table = pru_speak_dt_ids,
	},
	.probe	= pruproc_probe,
	.remove	= pruproc_remove,
};

//module_platform_driver(pruproc_driver);


static int __init pru_speak_init(void)
{
	printk(KERN_INFO "pru_speak loaded\n");
	platform_driver_register(&pru_speak_driver);
	return 0;
}

static void __exit pru_speak_exit(void)
{
	printk(KERN_INFO "pru_speak unloaded\n");
	platform_driver_unregister(&pru_speak_driver);
}

module_init(pru_speak_init);
module_exit(pru_speak_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("PRU speak driver");
MODULE_AUTHOR("Pantelis Antoniou <panto@antoniou-consulting.com>");
