/*
 * PRU-ICSS remoteproc driver for various TI SoCs
 *
 * Copyright (C) 2014 Texas Instruments, Inc.
 *
 * Suman Anna <s-anna@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define DEBUG
#define pr_fmt(fmt)    "%s: " fmt, __func__

#include <linux/module.h>
#include <linux/bitops.h>
#include <linux/debugfs.h>
#include <linux/of_device.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/virtio.h>
#include <linux/dma-mapping.h>
#include <linux/remoteproc.h>
#include <linux/mailbox_client.h>
#include <linux/omap-mailbox.h>

#include <linux/platform_data/remoteproc-pruss.h>

#include "remoteproc_internal.h"
#include "pruss_remoteproc.h"

/* maximum number of system events */
#define MAX_PRU_SYS_EVENTS     64

/* maximum number of interrupt channels */
#define MAX_PRU_CHANNELS       10

/* minimum starting host interrupt number for MPU */
#define MIN_PRU_HOST_INT       2

/* maximum number of host interrupts */
#define MAX_PRU_HOST_INT       10

/* PRU_ICSS_PRU_CTRL registers */
#define PRU_CTRL_CTRL		0x0000
#define PRU_CTRL_STS		0x0004
#define PRU_CTRL_WAKEUP_EN	0x0008
#define PRU_CTRL_CYCLE		0x000C
#define PRU_CTRL_STALL		0x0010
#define PRU_CTRL_CTBIR0		0x0020
#define PRU_CTRL_CTBIR1		0x0024
#define PRU_CTRL_CTPPR0		0x0028
#define PRU_CTRL_CTPPR1		0x002C

/* CTRL register bit-fields */
#define CTRL_CTRL_SOFT_RST_N	BIT(0)
#define CTRL_CTRL_EN		BIT(1)
#define CTRL_CTRL_SLEEPING	BIT(2)
#define CTRL_CTRL_CTR_EN	BIT(3)
#define CTRL_CTRL_SINGLE_STEP	BIT(8)
#define CTRL_CTRL_RUNSTATE	BIT(15)

/* PRU_ICSS_PRU_DEBUG registers */
#define PRU_DEBUG_GPREG(x)	(0x0000 + (x) * 4)
#define PRU_DEBUG_CT_REG(x)	(0x0080 + (x) * 4)

/* PRU_ICSS_INTC registers */
#define PRU_INTC_REVID		0x0000
#define PRU_INTC_CR		0x0004
#define PRU_INTC_GER		0x0010
#define PRU_INTC_GNLR		0x001C
#define PRU_INTC_SISR		0x0020
#define PRU_INTC_SICR		0x0024
#define PRU_INTC_EISR		0x0028
#define PRU_INTC_EICR		0x002C
#define PRU_INTC_HIEISR		0x0034
#define PRU_INTC_HIDISR		0x0038
#define PRU_INTC_GPIR		0x0080
#define PRU_INTC_SRSR0		0x0200
#define PRU_INTC_SRSR1		0x0204
#define PRU_INTC_SECR0		0x0280
#define PRU_INTC_SECR1		0x0284
#define PRU_INTC_ESR0		0x0300
#define PRU_INTC_ESR1		0x0304
#define PRU_INTC_ECR0		0x0380
#define PRU_INTC_ECR1		0x0384
#define PRU_INTC_CMR(x)		(0x0400 + (x) * 4)
#define PRU_INTC_HMR(x)		(0x0800 + (x) * 4)
#define PRU_INTC_HIPIR(x)	(0x0900 + (x) * 4)
#define PRU_INTC_SIPR0		0x0D00
#define PRU_INTC_SIPR1		0x0D04
#define PRU_INTC_SITR0		0x0D80
#define PRU_INTC_SITR1		0x0D84
#define PRU_INTC_HINLR(x)	(0x1100 + (x) * 4)
#define PRU_INTC_HIER		0x1500

/* HIPIR register bit-fields */
#define INTC_HIPIR_NONE_HINT	0x80000000

/**
 * enum pruss_mem - PRUSS memory range identifiers
 */
enum pruss_mem {
	PRUSS_MEM_DRAM0 = 0,
	PRUSS_MEM_DRAM1,
	PRUSS_MEM_SHRD_RAM2,
	PRUSS_MEM_INTC,
	PRUSS_MEM_CFG,
	PRUSS_MEM_MAX,
};

/**
 * enum pru_mem - PRU core memory range identifiers
 */
enum pru_mem {
	PRU_MEM_IRAM = 0,
	PRU_MEM_CTRL,
	PRU_MEM_DEBUG,
	PRU_MEM_MAX,
};

/**
 * struct pru_private_data - PRU core private data
 * @id: PRU index
 * @fw_name: firmware name to be used for the PRU core
 */
struct pru_private_data {
	u32 id;
	const char *fw_name;
};

/**
 * struct pru_match_private_data - match private data to handle multiple instances
 * @device_name: device name of the PRU processor core instance
 * @priv_data: PRU driver private data for this PRU processor core instance
 */
struct pru_match_private_data {
	const char *device_name;
	struct pru_private_data *priv_data;
};

/**
 * struct pruss_private_data - PRUSS driver private data
 * @num_irqs: number of interrupts to MPU
 * @host_events: bit mask of PRU host interrupts that are routed to MPU
 * @aux_data: auxiliary data used for creating the child nodes
 * @has_reset: flag to indicate the presence of global module reset
 */
struct pruss_private_data {
	int num_irqs;
	int host_events;
	struct of_dev_auxdata *aux_data;
	bool has_reset;
};

/**
 * struct pruss_match_private_data - match private data to handle multiple instances
 * @device_name: device name of the PRUSS instance
 * @priv_data: PRUSS driver private data for this PRUSS instance
 */
struct pruss_match_private_data {
	const char *device_name;
	struct pruss_private_data *priv_data;
};

struct pru_rproc;

/**
 * struct pruss - PRUSS parent structure
 * @pdev: platform device
 * @mem_va: kernel virtual addresses for each of the PRUSS memory regions
 * @mem_pa: physical addresses for each of the PRUSS memory regions
 * @mem_size: size of each of the PRUSS memory regions
 * @data: pointer to store PRUSS instance private data
 * @irqs: pointer to an array of interrupts to the host processor
 * @sysev_to_ch: system events to channel mapping information
 * @ch_to_host: interrupt channel to host interrupt information
 */
struct pruss {
	struct platform_device *pdev;
	void __iomem *mem_va[PRUSS_MEM_MAX];
	phys_addr_t mem_pa[PRUSS_MEM_MAX];
	size_t mem_size[PRUSS_MEM_MAX];
	const struct pruss_private_data *data;
	int *irqs;
	int sysev_to_ch[MAX_PRU_SYS_EVENTS];
	int ch_to_host[MAX_PRU_CHANNELS];
};

/**
 * struct pru_rproc: PRU remoteproc structure
 * @id: id of the PRU core within the PRUSS
 * @pruss: back-reference to parent PRUSS structure
 * @rproc: remoteproc pointer for this PRU core
 * @mbox: mailbox channel handle used for vring signalling with MPU
 * @client: mailbox client to request the mailbox channel
 * @mem_va: kernel virtual addresses for each of the PRU memory regions
 * @mem_pa: physical addresses for each of the PRU memory regions
 * @mem_size: size of each of the PRU memory regions
 * @iram_da: device address of Instruction RAM for this PRU
 * @pdram_da: device address of primary Data RAM for this PRU
 * @sdram_da: device address of secondary Data RAM for this PRU
 * @shrdram_da: device address of shared Data RAM
 * @fw_name: name of firmware image used during loading
 * @dbg_single_step: debug flag to set PRU into single step mode
 * @dbg_continous: debug flag to restore PRU execution mode
 */
struct pru_rproc {
	int id;
	struct pruss *pruss;
	struct rproc *rproc;
	struct mbox_chan *mbox;
	struct mbox_client client;
	void __iomem *mem_va[PRU_MEM_MAX];
	phys_addr_t mem_pa[PRU_MEM_MAX];
	size_t mem_size[PRU_MEM_MAX];
	u32 iram_da;
	u32 pdram_da;
	u32 sdram_da;
	u32 shrdram_da;
	const char *fw_name;
	u32 dbg_single_step;
	u32 dbg_continuous;
};

static inline u32 pruss_intc_read_reg(struct pruss *pruss, unsigned int reg)
{
	return readl_relaxed(pruss->mem_va[PRUSS_MEM_INTC] + reg);
}

static inline
void pruss_intc_write_reg(struct pruss *pruss, unsigned int reg, u32 val)
{
	writel_relaxed(val, pruss->mem_va[PRUSS_MEM_INTC] + reg);
}

static inline u32 pru_control_read_reg(struct pru_rproc *pru, unsigned int reg)
{
	return readl_relaxed(pru->mem_va[PRU_MEM_CTRL] + reg);
}

static inline
void pru_control_write_reg(struct pru_rproc *pru, unsigned int reg, u32 val)
{
	writel_relaxed(val, pru->mem_va[PRU_MEM_CTRL] + reg);
}

static inline u32 pru_debug_read_reg(struct pru_rproc *pru, unsigned int reg)
{
	return readl_relaxed(pru->mem_va[PRU_MEM_DEBUG] + reg);
}

static inline
void pru_debug_write_reg(struct pru_rproc *pru, unsigned int reg, u32 val)
{
	writel_relaxed(val, pru->mem_va[PRU_MEM_DEBUG] + reg);
}

/*
 * Convert PRU device address (data spaces only) to kernel virtual address
 *
 * Each PRU has access to all data memories within the PRUSS, accessible at
 * different ranges. So, look through both its primary and secondary Data
 * RAMs as well as any shared Data RAM to convert a PRU device address to
 * kernel virtual address. Data RAM0 is primary Data RAM for PRU0 and Data
 * RAM1 is primary Data RAM for PRU1.
 */
static void *pru_d_da_to_va(struct pru_rproc *pru, u32 da, int len)
{
	struct pruss *pruss = pru->pruss;
	u32 offset;
	u32 index;
	size_t dram_sz = pruss->mem_size[PRUSS_MEM_DRAM0];
	size_t shrd_ram_sz = pruss->mem_size[PRUSS_MEM_SHRD_RAM2];

	WARN_ON(pruss->mem_size[PRUSS_MEM_DRAM0] !=
		pruss->mem_size[PRUSS_MEM_DRAM1]);

	if (len <= 0)
		return NULL;

	if (da >= pru->pdram_da && da + len <= pru->pdram_da + dram_sz) {
		offset = da - pru->pdram_da;
		index = pru->id % 2;
	} else if (da >= pru->sdram_da && da + len <= pru->sdram_da + dram_sz) {
		offset = da - pru->sdram_da;
		index = (pru->id + 1) % 2;
	} else if (da >= pru->shrdram_da &&
		   da + len <= pru->shrdram_da + shrd_ram_sz) {
		offset = da - pru->shrdram_da;
		index = 2;
	} else {
		return NULL;
	}

	return (__force void *)(pruss->mem_va[index] + offset);
}

/*
 * Convert PRU device address (instruction space) to kernel virtual address
 *
 * A PRU does not have an unified address space. Each PRU has its very own
 * private Instruction RAM, and its device address is identical to that of
 * its primary Data RAM device address.
 */
static void *pru_i_da_to_va(struct pru_rproc *pru, u32 da, int len)
{
	u32 offset;

	if (len > 0 && da >= pru->iram_da &&
	    da + len <= pru->iram_da + pru->mem_size[PRU_MEM_IRAM]) {
		offset = da - pru->iram_da;
		return (__force void *)(pru->mem_va[PRU_MEM_IRAM] + offset);
	}

	return NULL;
}

static int pru_rproc_debug_read_regs(struct seq_file *s, void *data)
{
	struct rproc *rproc = s->private;
	struct pru_rproc *pru = rproc->priv;
	int i, nregs = 32;
	int ret = 0;
	u32 pru_sts;
	int pru_is_running;

	ret += seq_puts(s, "============== Control Registers ==============\n");
	ret += seq_printf(s, "CTRL      := 0x%08x\n",
			  pru_control_read_reg(pru, PRU_CTRL_CTRL));
	pru_sts = pru_control_read_reg(pru, PRU_CTRL_STS);
	ret += seq_printf(s, "STS (PC)  := 0x%08x (0x%08x)\n",
			  pru_sts, pru_sts << 2);
	ret += seq_printf(s, "WAKEUP_EN := 0x%08x\n",
			  pru_control_read_reg(pru, PRU_CTRL_WAKEUP_EN));
	ret += seq_printf(s, "CYCLE     := 0x%08x\n",
			  pru_control_read_reg(pru, PRU_CTRL_CYCLE));
	ret += seq_printf(s, "STALL     := 0x%08x\n",
			  pru_control_read_reg(pru, PRU_CTRL_STALL));
	ret += seq_printf(s, "CTBIR0    := 0x%08x\n",
			  pru_control_read_reg(pru, PRU_CTRL_CTBIR0));
	ret += seq_printf(s, "CTBIR1    := 0x%08x\n",
			  pru_control_read_reg(pru, PRU_CTRL_CTBIR1));
	ret += seq_printf(s, "CTPPR0    := 0x%08x\n",
			  pru_control_read_reg(pru, PRU_CTRL_CTPPR0));
	ret += seq_printf(s, "CTPPR1    := 0x%08x\n",
			  pru_control_read_reg(pru, PRU_CTRL_CTPPR1));

	ret += seq_puts(s, "=============== Debug Registers ===============\n");
	pru_is_running = pru_control_read_reg(pru, PRU_CTRL_CTRL) &
				CTRL_CTRL_RUNSTATE;
	if (pru_is_running) {
		ret += seq_puts(s, "PRU is executing, cannot print/access debug registers.\n");
		return 0;
	}

	for (i = 0; i < nregs; i++) {
		ret += seq_printf(s, "GPREG%-2d := 0x%08x\tCT_REG%-2d := 0x%08x\n",
			    i, pru_debug_read_reg(pru, PRU_DEBUG_GPREG(i)),
			    i, pru_debug_read_reg(pru, PRU_DEBUG_CT_REG(i)));
	}

	return 0;
}

static int pru_rproc_debug_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, pru_rproc_debug_read_regs, inode->i_private);
}

static const struct file_operations pru_rproc_debug_regs_ops = {
	.open = pru_rproc_debug_regs_open,
	.read = seq_read,
	.llseek	= seq_lseek,
	.release = single_release,
};

/*
 * Control PRU single-step mode
 *
 * This is a debug helper function used for controlling the single-step
 * mode of the PRU. The PRU Debug registers are not accessible when the
 * PRU is in RUNNING state.
 *
 * Writing a non-zero value sets the PRU into single-step mode irrespective
 * of its previous state. The PRU mode is saved only on the first set into
 * a single-step mode. Writing a non-zero value will restore the PRU into
 * its original mode.
 */
static int pru_rproc_debug_ss_set(void *data, u64 val)
{
	struct rproc *rproc = data;
	struct pru_rproc *pru = rproc->priv;
	u32 reg_val;

	val = val ? 1 : 0;
	if (!val && !pru->dbg_single_step)
		return 0;

	reg_val = pru_control_read_reg(pru, PRU_CTRL_CTRL);

	if (val && !pru->dbg_single_step)
		pru->dbg_continuous = reg_val;

	if (val)
		reg_val |= CTRL_CTRL_SINGLE_STEP | CTRL_CTRL_EN;
	else
		reg_val = pru->dbg_continuous;

	pru->dbg_single_step = val;
	pru_control_write_reg(pru, PRU_CTRL_CTRL, reg_val);

	return 0;
}

static int pru_rproc_debug_ss_get(void *data, u64 *val)
{
	struct rproc *rproc = data;
	struct pru_rproc *pru = rproc->priv;

	*val = pru->dbg_single_step;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(pru_rproc_debug_ss_fops, pru_rproc_debug_ss_get,
			pru_rproc_debug_ss_set, "%llu\n");

/*
 * Create PRU-specific debugfs entries
 *
 * The entries are created only if the parent remoteproc debugfs directory
 * exists, and will be cleaned up by the remoteproc core.
 */
static void pru_rproc_create_debug_entries(struct rproc *rproc)
{
	if (!rproc->dbg_dir)
		return;

	debugfs_create_file("regs", 0400, rproc->dbg_dir,
			    rproc, &pru_rproc_debug_regs_ops);
	debugfs_create_file("single_step", 0600, rproc->dbg_dir,
			    rproc, &pru_rproc_debug_ss_fops);
}

static void pruss_init_intc(struct pruss *pruss)
{
	int i;

	/* configure polarity to active high for all system interrupts */
	pruss_intc_write_reg(pruss, PRU_INTC_SIPR0, 0xffffffff);
	pruss_intc_write_reg(pruss, PRU_INTC_SIPR1, 0xffffffff);

	/* configure type to pulse interrupt for all system interrupts */
	pruss_intc_write_reg(pruss, PRU_INTC_SITR0, 0);
	pruss_intc_write_reg(pruss, PRU_INTC_SITR1, 0);

	/* clear all interrupt channel map registers */
	for (i = PRU_INTC_CMR(0); i <= PRU_INTC_CMR(15); i += 4)
		pruss_intc_write_reg(pruss, i, 0);

	/* clear all host interrupt map registers */
	for (i = PRU_INTC_HMR(0); i <= PRU_INTC_HMR(2); i += 4)
		pruss_intc_write_reg(pruss, i, 0);
}

/*
 * Configure the PRUSS INTC appropriately
 * XXX: change the life-cycle management to per PRU core
 */
static int pruss_configure_intc(struct pruss *pruss)
{
	struct device *dev = &pruss->pdev->dev;
	int i, idx, ch, host;
	uint64_t sysevt_mask = 0;
	uint32_t ch_mask = 0;
	uint32_t host_mask = 0;
	u32 val;

	/*
	 * configure channel map registers - each register holds map info
	 * for 4 events, with each event occupying the lower nibble in
	 * a register byte address in little-endian fashion
	 */
	for (i = 0; i < ARRAY_SIZE(pruss->sysev_to_ch); i++) {
		ch = pruss->sysev_to_ch[i];
		if (ch < 0)
			continue;

		idx = i / 4;
		val  = pruss_intc_read_reg(pruss, PRU_INTC_CMR(idx));
		val |= (u32)ch << ((i & 3) * 8);
		pruss_intc_write_reg(pruss, PRU_INTC_CMR(idx), val);

		sysevt_mask |= 1LLU << i;
		ch_mask |= 1U << ch;

		dev_dbg(dev, "SYSEV%d -> CH%d (CMR%d 0x%08x)\n", i, ch, idx,
			pruss_intc_read_reg(pruss, PRU_INTC_CMR(idx)));
	}

	/*
	 * set host map registers - each register holds map info for
	 * 4 channels, with each channel occupying the lower nibble in
	 * a register byte address in little-endian fashion
	 */
	for (i = 0; i < ARRAY_SIZE(pruss->ch_to_host); i++) {
		host = pruss->ch_to_host[i];
		if (host < 0)
			continue;

		idx = i / 4;

		val  = pruss_intc_read_reg(pruss, PRU_INTC_HMR(idx));
		val |= (u32)host << ((i & 3) * 8);
		pruss_intc_write_reg(pruss, PRU_INTC_HMR(idx), val);

		ch_mask |= 1U << i;
		host_mask |= 1U << host;

		dev_dbg(dev, "CH%d -> HOST%d (HMR%d 0x%08x)\n", i, host, idx,
			pruss_intc_read_reg(pruss, PRU_INTC_HMR(idx)));
	}

	dev_info(dev, "configured system_events = 0x%016llx intr_channels = 0x%08x host_intr = 0x%08x\n",
		 sysevt_mask, ch_mask, host_mask);

	/* enable system events */
	pruss_intc_write_reg(pruss, PRU_INTC_ESR0, (u32)sysevt_mask);
	pruss_intc_write_reg(pruss, PRU_INTC_SECR0, (u32)sysevt_mask);
	pruss_intc_write_reg(pruss, PRU_INTC_ESR1, (u32)(sysevt_mask >> 32));
	pruss_intc_write_reg(pruss, PRU_INTC_SECR1, (u32)(sysevt_mask >> 32));

	/* enable host interrupts */
	for (i = 0; i < MAX_PRU_HOST_INT; i++) {
		if ((host_mask & (1 << i)))
			pruss_intc_write_reg(pruss, PRU_INTC_HIEISR, i);
	}

	/* global interrupt enable */
	pruss_intc_write_reg(pruss, PRU_INTC_GER, 1);

	return 0;
}

static void pru_trigger_interrupt(struct rproc *rproc, int sysint)
{
	struct pru_rproc *pru = rproc->priv;
	struct pruss *pruss = pru->pruss;
	struct device *dev = &rproc->dev;

	if (sysint < 0) {
		dev_err(dev, "invalid sysint %d\n", sysint);
		return;
	}

	dev_dbg(dev, "triggering sysint %d on PRU %d\n", sysint, pru->id);
	if (sysint < 32)
		pruss_intc_write_reg(pruss, PRU_INTC_SRSR0, 1 << sysint);
	else
		pruss_intc_write_reg(pruss, PRU_INTC_SRSR1, 1 << (sysint - 32));
}

/**
 * pru_rproc_mbox_callback() - inbound mailbox message handler
 * @client: mailbox client pointer used for requesting the mailbox channel
 * @data: mailbox payload
 *
 * This handler is invoked by omap's mailbox driver whenever a mailbox
 * message is received. Usually, the mailbox payload simply contains
 * the index of the virtqueue that is kicked by the PRU remote processor,
 * and we let remoteproc core handle it.
 *
 * In addition to virtqueue indices, we might also have some out-of-band
 * values that indicates different events. Those values are deliberately
 * very big so they don't coincide with virtqueue indices.
 */
static void pru_rproc_mbox_callback(struct mbox_client *client, void *data)
{
	struct pru_rproc *pru = container_of(client, struct pru_rproc, client);
	struct device *dev = &pru->rproc->dev;
	u32 msg = (u32)data;

	dev_dbg(dev, "mbox msg: 0x%x\n", msg);

	/* msg contains the index of the triggered vring */
	if (rproc_vq_interrupt(pru->rproc, msg) == IRQ_NONE)
		dev_err(dev, "no message was found in vqid %d\n", msg);
}

/* kick a virtqueue */
static void pru_rproc_kick(struct rproc *rproc, int vq_id)
{
	struct device *dev = &rproc->dev;
	struct pru_rproc *pru = rproc->priv;
	int ret;

	dev_dbg(dev, "kicking vqid %d on PRU%d\n", vq_id, pru->id);

	/* send the index of the triggered virtqueue in the mailbox payload */
	ret = mbox_send_message(pru->mbox, (void *)vq_id);
	if (ret)
		dev_err(dev, "mbox_send_message failed: %d\n", ret);
}

/* start a PRU core */
static int pru_rproc_start(struct rproc *rproc)
{
	struct device *dev = &rproc->dev;
	struct pru_rproc *pru = rproc->priv;
	u32 val;

	dev_dbg(dev, "starting PRU%d: entry-point = 0x%x\n",
		pru->id, (rproc->bootaddr >> 2));

	val = CTRL_CTRL_EN | ((rproc->bootaddr >> 2) << 16);
	pru_control_write_reg(pru, PRU_CTRL_CTRL, val);

	return 0;
}

/* stop/disable a PRU core */
static int pru_rproc_stop(struct rproc *rproc)
{
	struct device *dev = &rproc->dev;
	struct pru_rproc *pru = rproc->priv;
	u32 val;

	dev_dbg(dev, "stopping PRU%d\n", pru->id);

	val = pru_control_read_reg(pru, PRU_CTRL_CTRL);
	val &= ~CTRL_CTRL_EN;
	pru_control_write_reg(pru, PRU_CTRL_CTRL, val);

	return 0;
}

/*
 * parse the custom interrupt map resource and configure the INTC
 * appropriately
 */
static int pru_handle_custom_intrmap(struct rproc *rproc,
				     struct fw_rsc_custom_intrmap *intr_rsc)
{
	struct device *dev = rproc->dev.parent;
	struct pru_rproc *pru = rproc->priv;
	struct pruss *pruss = pru->pruss;
	struct pruss_event_chnl *event_chnl_map;
	int i, ret;
	s8 sys_evt, chnl, intr_no;

	dev_dbg(dev, "version %d event_chnl_map_size %d event_chnl_map %p\n",
		intr_rsc->version, intr_rsc->event_chnl_map_size,
		intr_rsc->event_chnl_map);

	if (intr_rsc->version != 0) {
		dev_err(dev, "only custom ints resource version 0 supported\n");
		return -EINVAL;
	}

	if (intr_rsc->event_chnl_map_size < 0 ||
	    intr_rsc->event_chnl_map_size >= MAX_PRU_SYS_EVENTS) {
		dev_err(dev, "custom ints resource has more events than present on hardware\n");
		return -EINVAL;
	}

	/*
	 * XXX: The event_chnl_map mapping is currently a pointer in device
	 * memory, evaluate if this needs to be directly in firmware file.
	 */
	event_chnl_map = pru_d_da_to_va(pru, (u32)intr_rsc->event_chnl_map,
					intr_rsc->event_chnl_map_size *
					sizeof(*event_chnl_map));
	if (!event_chnl_map) {
		dev_err(dev, "custom ints resource has inadequate event_chnl_map configuration\n");
		return -EINVAL;
	}

	/* parse and fill in system event to interrupt channel mapping */
	for (i = 0; i < intr_rsc->event_chnl_map_size; i++) {
		sys_evt = event_chnl_map[i].event;
		chnl = event_chnl_map[i].chnl;

		if (sys_evt < 0 || sys_evt >= MAX_PRU_SYS_EVENTS) {
			dev_err(dev, "[%d] bad sys event %d\n", i, sys_evt);
			return -EINVAL;
		}
		if (chnl < 0 || chnl >= MAX_PRU_CHANNELS) {
			dev_err(dev, "[%d] bad channel value %d\n", i, chnl);
			return -EINVAL;
		}

		if (pruss->sysev_to_ch[sys_evt] != -1) {
			dev_err(dev, "[%d] event %d already assigned to channel %d\n",
				i, sys_evt, pruss->sysev_to_ch[sys_evt]);
			return -EEXIST;
		}
		pruss->sysev_to_ch[sys_evt] = chnl;
		dev_dbg(dev, "sysevt-to-ch[%d] -> %d\n", sys_evt, chnl);
	}

	/* parse and handle interrupt channel-to-host interrupt mapping */
	for (i = 0; i < MAX_PRU_CHANNELS; i++) {
		intr_no = intr_rsc->chnl_host_intr_map[i];
		if (intr_no < 0) {
			dev_dbg(dev, "skip intr mapping for chnl %d\n", i);
			continue;
		}

		if (intr_no >= MAX_PRU_HOST_INT) {
			dev_err(dev, "bad intr mapping for chnl %d, intr_no %d\n",
				i, intr_no);
			return -EINVAL;
		}

		if (pruss->ch_to_host[i] != -1) {
			dev_err(dev, "channel %d already assigned to intr_no %d\n",
				i, pruss->ch_to_host[i]);
			return -EEXIST;
		}
		pruss->ch_to_host[i] = intr_no;
		dev_dbg(dev, "chnl-to-host[%d] -> %d\n", i, intr_no);
	}

	ret = pruss_configure_intc(pruss);
	if (ret)
		dev_err(dev, "failed to configure pruss intc %d\n", ret);

	return ret;
}

/* PRU-specific post loading custom resource handler */
static int pru_rproc_handle_custom_rsc(struct rproc *rproc,
				       struct fw_rsc_custom *rsc)
{
	struct device *dev = rproc->dev.parent;
	int ret = -EINVAL;

	switch (rsc->sub_type) {
	case PRUSS_RSC_INTRS:
		ret = pru_handle_custom_intrmap(rproc,
			(struct fw_rsc_custom_intrmap *)rsc->data);
		break;
	default:
		dev_err(dev, "%s: handling unknown type %d\n", __func__,
			rsc->sub_type);
	}

	return ret;
}

/* PRU-specific address translator */
static void *pru_da_to_va(struct rproc *rproc, u64 da, int len, u32 flags)
{
	struct pru_rproc *pru = rproc->priv;
	void *va;
	u32 exec_flag = 0;

	exec_flag = ((flags & RPROC_FLAGS_ELF_SHDR) ? flags & SHF_EXECINSTR :
		     ((flags & RPROC_FLAGS_ELF_PHDR) ? flags & PF_X : 0));

	if (exec_flag)
		va = pru_i_da_to_va(pru, da, len);
	else
		va = pru_d_da_to_va(pru, da, len);

	return va;
}

static struct rproc_ops pru_rproc_ops = {
	.start			= pru_rproc_start,
	.stop			= pru_rproc_stop,
	.kick			= pru_rproc_kick,
	.handle_custom_rsc	= pru_rproc_handle_custom_rsc,
	.da_to_va		= pru_da_to_va,
};

static const struct of_device_id pru_rproc_match[];

static const struct pru_private_data *pru_rproc_get_private_data(
						struct platform_device *pdev)
{
	const struct pru_match_private_data *data;
	const struct of_device_id *match;

	match = of_match_device(pru_rproc_match, &pdev->dev);
	if (!match)
		return ERR_PTR(-ENODEV);

	data = match->data;
	for (; data && data->device_name; data++) {
		if (!strcmp(dev_name(&pdev->dev), data->device_name))
			return data->priv_data;
	}

	return NULL;
}

static int pru_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct platform_device *ppdev = to_platform_device(dev->parent);
	struct pru_rproc *pru;
	const struct pru_private_data *pdata;
	struct rproc *rproc = NULL;
	struct mbox_client *client;
	struct resource *res;
	int i, ret;
	const char *mem_names[PRU_MEM_MAX] = { "iram", "control", "debug" };

	if (!np) {
		dev_err(dev, "Non-DT platform device not supported\n");
		return -ENODEV;
	}

	pdata = pru_rproc_get_private_data(pdev);
	if (IS_ERR_OR_NULL(pdata) || !pdata->fw_name) {
		dev_err(dev, "missing or incomplete PRU-private data\n");
		return -ENODEV;
	}

	rproc = rproc_alloc(dev, pdev->name, &pru_rproc_ops, pdata->fw_name,
			    sizeof(*pru));
	if (!rproc) {
		dev_err(dev, "rproc_alloc failed\n");
		return -ENOMEM;
	}
	/* error recovery is not supported for PRUs */
	rproc->recovery_disabled = true;

	pru = rproc->priv;
	pru->id = pdata->id;
	pru->pruss = platform_get_drvdata(ppdev);
	pru->rproc = rproc;
	pru->fw_name = pdata->fw_name;

	/* XXX: get this from match data if different in the future */
	pru->iram_da = 0;
	pru->pdram_da = 0;
	pru->sdram_da = 0x2000;
	pru->shrdram_da = 0x10000;

	for (i = 0; i < ARRAY_SIZE(mem_names); i++) {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   mem_names[i]);
		pru->mem_va[i] = devm_ioremap_resource(dev, res);
		pru->mem_pa[i] = res->start;
		pru->mem_size[i] = resource_size(res);
		if (IS_ERR(pru->mem_va[i])) {
			dev_err(dev, "failed to parse and map memory resource %d %s\n",
				i, mem_names[i]);
			ret = PTR_ERR(pru->mem_va[i]);
			goto free_rproc;
		}
		dev_dbg(dev, "memory %8s: pa 0x%llx size 0x%x va %p\n",
			mem_names[i], (unsigned long long)pru->mem_pa[i],
			pru->mem_size[i], pru->mem_va[i]);
	}

	platform_set_drvdata(pdev, rproc);

	client = &pru->client;
	client->dev = dev;
	client->tx_done = NULL;
	client->rx_callback = pru_rproc_mbox_callback;
	client->tx_block = false;
	client->knows_txdone = false;
	pru->mbox = mbox_request_channel(client, 0);
	if (IS_ERR(pru->mbox)) {
		ret = PTR_ERR(pru->mbox);
		dev_err(dev, "mbox_request_channel failed: %d\n", ret);
		goto free_rproc;
	}

	ret = rproc_add(pru->rproc);
	if (ret) {
		dev_err(dev, "rproc_add failed: %d\n", ret);
		goto put_mbox;
	}

	pru_rproc_create_debug_entries(rproc);

	/*
	 * rproc_add will boot the processor if the corresponding PRU
	 * has a virtio device published in its resource table. If not
	 * present, manually boot the PRU remoteproc, but only after
	 * the remoteproc core is done with loading the firmware image.
	 */
	wait_for_completion(&pru->rproc->firmware_loading_complete);
	if (list_empty(&pru->rproc->rvdevs)) {
		dev_info(dev, "booting the PRU core manually\n");
		ret = rproc_boot(pru->rproc);
		if (ret) {
			dev_err(dev, "rproc_boot failed\n");
			goto del_rproc;
		}
	}

	/* suppress unused function warning */
	(void) pru_trigger_interrupt;

	dev_info(dev, "PRU rproc node %s probed successfully\n", np->full_name);

	return 0;

del_rproc:
	rproc_del(pru->rproc);
put_mbox:
	mbox_free_channel(pru->mbox);
free_rproc:
	rproc_put(rproc);
	return ret;
}

static int pru_rproc_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rproc *rproc = platform_get_drvdata(pdev);
	struct pru_rproc *pru = rproc->priv;

	dev_info(dev, "%s: removing rproc %s\n", __func__, rproc->name);

	if (list_empty(&pru->rproc->rvdevs)) {
		dev_info(dev, "stopping the manually booted PRU core\n");
		rproc_shutdown(pru->rproc);
	}

	mbox_free_channel(pru->mbox);

	rproc_del(rproc);
	rproc_put(rproc);

	return 0;
}

/*
 * Interrupt Handler for all PRUSS MPU interrupts
 * XXX: Enhance with event listener notifications
 */
static irqreturn_t pruss_handler(int irq, void *data)
{
	struct pruss *pruss = data;
	u32 sys_evt, val;
	int intr_bit = irq - pruss->irqs[0] + MIN_PRU_HOST_INT;
	int intr_mask = (1 << intr_bit);
	static int evt_mask = MAX_PRU_SYS_EVENTS - 1;

	/* check whether the interrupt can reach MPU */
	if (!(intr_mask & pruss->data->host_events))
		return IRQ_NONE;

	/* check whether the specific host interrupt is enabled */
	val = pruss_intc_read_reg(pruss, PRU_INTC_HIER);
	if (!(val & intr_mask))
		return IRQ_NONE;

	/* check non-pending bit of specific host interrupt */
	val = pruss_intc_read_reg(pruss, PRU_INTC_HIPIR(intr_bit));
	if (val & INTC_HIPIR_NONE_HINT)
		return IRQ_NONE;

	/* clear system event */
	sys_evt = val & evt_mask;
	if (sys_evt < 32)
		pruss_intc_write_reg(pruss, PRU_INTC_SECR0, 1 << sys_evt);
	else
		pruss_intc_write_reg(pruss, PRU_INTC_SECR1,
				     1 << (sys_evt - 32));

	return IRQ_HANDLED;
}

static struct of_dev_auxdata pru_rproc_auxdata_lookup[];
static const struct of_device_id pruss_of_match[];

static const
struct pruss_private_data *pruss_get_private_data(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct pruss_match_private_data *data;
	const struct of_device_id *match;

	match = of_match_device(pruss_of_match, &pdev->dev);
	if (!match)
		return ERR_PTR(-ENODEV);

	if (of_device_is_compatible(np, "ti,am335x-pruss") ||
	    of_device_is_compatible(np, "ti,am4372-pruss"))
		return match->data;

	data = match->data;
	for (; data && data->device_name; data++) {
		if (!strcmp(dev_name(&pdev->dev), data->device_name))
			return data->priv_data;
	}

	return NULL;
}

static int pruss_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	int ret;
	struct pruss *pruss;
	struct rproc *rproc = NULL;
	struct resource *res;
	int err, i, irq, num_irqs;
	struct pruss_platform_data *pdata = dev_get_platdata(dev);
	const struct pruss_private_data *data;
	const char *mem_names[PRUSS_MEM_MAX] = {
				"dram0", "dram1", "shrdram2", "intc", "cfg" };

	if (!node) {
		dev_err(dev, "Non-DT platform device not supported\n");
		return -ENODEV;
	}

	data = pruss_get_private_data(pdev);
	if (IS_ERR_OR_NULL(data)) {
		dev_err(dev, "missing private data\n");
		return -ENODEV;
	}

	if (data->has_reset && (!pdata || !pdata->deassert_reset ||
				!pdata->assert_reset || !pdata->reset_name)) {
		dev_err(dev, "platform data (reset configuration information) missing\n");
		return -ENODEV;
	}

	err = dma_set_coherent_mask(dev, DMA_BIT_MASK(32));
	if (err) {
		dev_err(dev, "dma_set_coherent_mask: %d\n", err);
		return err;
	}

	pruss = devm_kzalloc(dev, sizeof(*pruss), GFP_KERNEL);
	if (!pruss) {
		dev_err(dev, "failed to allocate pruss\n");
		return -ENOMEM;
	}

	pruss->pdev = pdev;
	pruss->data = data;

	num_irqs = data->num_irqs;
	pruss->irqs = devm_kzalloc(dev, sizeof(*pruss->irqs) * num_irqs,
				   GFP_KERNEL);
	if (!pruss->irqs) {
		dev_err(dev, "failed to allocate memory for irqs\n");
		return -ENOMEM;
	}

	for (i = 0; i < num_irqs; i++)
		pruss->irqs[i] = -1;

	for (i = 0; i < ARRAY_SIZE(pruss->sysev_to_ch); i++)
		pruss->sysev_to_ch[i] = -1;

	for (i = 0; i < ARRAY_SIZE(pruss->ch_to_host); i++)
		pruss->ch_to_host[i] = -1;

	for (i = 0; i < num_irqs; i++) {
		pruss->irqs[i] = platform_get_irq(pdev, i);
		if (pruss->irqs[i] < 0) {
			dev_err(dev, "failed to get irq #%d ret = %d\n",
				i, pruss->irqs[i]);
			return pruss->irqs[i];
		}
	}
	dev_dbg(dev, "%d PRU interrupts parsed\n", num_irqs);

	for (i = 0; i < ARRAY_SIZE(mem_names); i++) {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   mem_names[i]);
		pruss->mem_va[i] = devm_ioremap_resource(dev, res);
		pruss->mem_pa[i] = res->start;
		pruss->mem_size[i] = resource_size(res);
		if (IS_ERR(pruss->mem_va[i])) {
			dev_err(dev, "failed to parse and map memory resource %d %s\n",
				i, mem_names[i]);
			return PTR_ERR(pruss->mem_va[i]);
		}
		dev_dbg(dev, "memory %8s: pa 0x%llx size 0x%x va %p\n",
			mem_names[i], (unsigned long long)pruss->mem_pa[i],
			pruss->mem_size[i], pruss->mem_va[i]);
	}

	if (data->has_reset) {
		err = pdata->deassert_reset(pdev, pdata->reset_name);
		if (err) {
			dev_err(dev, "deassert_reset failed: %d\n", err);
			goto err_fail;
		}
	}

	pm_runtime_enable(dev);
	err = pm_runtime_get_sync(dev);
	if (err < 0) {
		pm_runtime_put_noidle(dev);
		dev_err(dev, "pm_runtime_get_sync failed\n");
		goto err_rpm_fail;
	}

	pruss_init_intc(pruss);

	for (i = 0; i < num_irqs; i++) {
		irq = pruss->irqs[i];
		err = devm_request_irq(dev, irq, pruss_handler, 0,
				       dev_name(dev), pruss);
		if (err) {
			dev_err(dev, "failed to register irq %d\n", irq);
			goto err_irq_fail;
		}
	}

	platform_set_drvdata(pdev, pruss);

	dev_info(&pdev->dev, "creating platform devices for PRU cores\n");
	ret = of_platform_populate(node, NULL, data->aux_data, &pdev->dev);

	return ret;

err_irq_fail:
	pm_runtime_put_sync(dev);
err_rpm_fail:
	pm_runtime_disable(dev);
	if (data->has_reset)
		pdata->assert_reset(pdev, pdata->reset_name);

	if (rproc)
		rproc_put(rproc);
err_fail:
	return err;
}

static int pru_rproc_unregister(struct device *dev, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);

	of_device_unregister(pdev);

	return 0;
}

static int pruss_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pruss_platform_data *pdata = dev_get_platdata(dev);
	struct pruss *pruss = platform_get_drvdata(pdev);

	dev_info(dev, "remove platform devices for PRU cores\n");
	device_for_each_child(dev, NULL, pru_rproc_unregister);

	pm_runtime_put_sync(dev);
	pm_runtime_disable(dev);
	if (pruss->data->has_reset)
		pdata->assert_reset(pdev, pdata->reset_name);

	return 0;
}

/* PRU0 core-specific private data */
static struct pru_private_data pru0_rproc_pdata = {
	.id = 0,
	.fw_name = "rproc-pru0-fw",
};

/* PRU1 core-specific private data */
static struct pru_private_data pru1_rproc_pdata = {
	.id = 1,
	.fw_name = "rproc-pru1-fw",
};

static struct pru_private_data pru1_0_rproc_pdata = {
	.id = 0,
	.fw_name = "am57xx-pru1_0-fw",
};

static struct pru_private_data pru1_1_rproc_pdata = {
	.id = 1,
	.fw_name = "am57xx-pru1_1-fw",
};

static struct pru_private_data pru2_0_rproc_pdata = {
	.id = 0,
	.fw_name = "am57xx-pru2_0-fw",
};

static struct pru_private_data pru2_1_rproc_pdata = {
	.id = 1,
	.fw_name = "am57xx-pru2_1-fw",
};

/* platform data to be added when creating the PRU platform devices */
static struct of_dev_auxdata am335x_pru_rproc_auxdata_lookup[] = {
	OF_DEV_AUXDATA("ti,pru-rproc", 0x4a334000, "4a334000.pru0", NULL),
	OF_DEV_AUXDATA("ti,pru-rproc", 0x4a338000, "4a338000.pru1", NULL),
	{ /* sentinel */ },
};

static struct of_dev_auxdata am4372_pru_rproc_auxdata_lookup[] = {
	OF_DEV_AUXDATA("ti,pru-rproc", 0x54434000, "54434000.pru0", NULL),
	OF_DEV_AUXDATA("ti,pru-rproc", 0x54438000, "54438000.pru1", NULL),
	{ /* sentinel */ },
};

static struct of_dev_auxdata am5728_pruss1_rproc_auxdata_lookup[] = {
	OF_DEV_AUXDATA("ti,pru-rproc", 0x4b234000, "4b234000.pru0", NULL),
	OF_DEV_AUXDATA("ti,pru-rproc", 0x4b238000, "4b238000.pru1", NULL),
	{ /* sentinel */ },
};

static struct of_dev_auxdata am5728_pruss2_rproc_auxdata_lookup[] = {
	OF_DEV_AUXDATA("ti,pru-rproc", 0x4b2b4000, "4b2b4000.pru0", NULL),
	OF_DEV_AUXDATA("ti,pru-rproc", 0x4b2b8000, "4b2b8000.pru1", NULL),
	{ /* sentinel */ },
};

/*
 * A single match structure is used against a unified compatible
 * string "ti,pru-rproc" as the addresses of the different PRU cores
 * are unique across all the applicable SoCs.
 * XXX: A SoC-specific compatible string is probably a better option
 *	for the future to allow more flexibility.
 */
static struct pru_match_private_data pru_match_data[] = {
	/* AM33xx SoC-specific data */
	{
		.device_name	= "4a334000.pru0",
		.priv_data	= &pru0_rproc_pdata,
	},
	{
		.device_name	= "4a338000.pru1",
		.priv_data	= &pru1_rproc_pdata,
	},
	/* AM43xx SoC-specific data */
	{
		.device_name	= "54434000.pru0",
		.priv_data	= &pru0_rproc_pdata,
	},
	{
		.device_name	= "54438000.pru1",
		.priv_data	= &pru1_rproc_pdata,
	},
	/* AM57xx SoC-specific data */
	{
		.device_name	= "4b234000.pru0",
		.priv_data	= &pru1_0_rproc_pdata,
	},
	{
		.device_name	= "4b238000.pru1",
		.priv_data	= &pru1_1_rproc_pdata,
	},
	{
		.device_name	= "4b2b4000.pru0",
		.priv_data	= &pru2_0_rproc_pdata,
	},
	{
		.device_name	= "4b2b8000.pru1",
		.priv_data	= &pru2_1_rproc_pdata,
	},
	{
		/* sentinel */
	},
};

static const struct of_device_id pru_rproc_match[] = {
	{ .compatible = "ti,pru-rproc", .data = pru_match_data, },
	{},
};
MODULE_DEVICE_TABLE(of, pru_rproc_match);

static struct platform_driver pru_rproc_driver = {
	.driver = {
		.name   = "pru-rproc",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(pru_rproc_match),
	},
	.probe  = pru_rproc_probe,
	.remove = pru_rproc_remove,
};

/*
 * There is a one-to-one relation between PRU Host interrupts
 * and the PRU Host events. The interrupts are expected to be
 * in the increasing order of PRU Host events.
 */
static struct pruss_private_data am335x_priv_data = {
	.num_irqs = 8,
	.host_events = (BIT(2) | BIT(3) | BIT(4) | BIT(5) |
			BIT(6) | BIT(7) | BIT(8) | BIT(9)),
	.aux_data = am335x_pru_rproc_auxdata_lookup,
	.has_reset = true,
};

static struct pruss_private_data am4372_priv_data = {
	.num_irqs = 7,
	.host_events = (BIT(2) | BIT(3) | BIT(4) | BIT(5) |
			BIT(6) | BIT(8) | BIT(9)),
	.aux_data = am4372_pru_rproc_auxdata_lookup,
	.has_reset = true,
};

static struct pruss_private_data am5728_pruss1_priv_data = {
	.num_irqs = 8,
	.host_events = (BIT(2) | BIT(3) | BIT(4) | BIT(5) |
			BIT(6) | BIT(7) | BIT(8) | BIT(9)),
	.aux_data = am5728_pruss1_rproc_auxdata_lookup,
};

static struct pruss_private_data am5728_pruss2_priv_data = {
	.num_irqs = 8,
	.host_events = (BIT(2) | BIT(3) | BIT(4) | BIT(5) |
			BIT(6) | BIT(7) | BIT(8) | BIT(9)),
	.aux_data = am5728_pruss2_rproc_auxdata_lookup,
};

static struct pruss_match_private_data am5728_match_data[] = {
	{
		.device_name	= "4b200000.pruss",
		.priv_data	= &am5728_pruss1_priv_data,
	},
	{
		.device_name	= "4b280000.pruss",
		.priv_data	= &am5728_pruss2_priv_data,
	},
	{
		/* sentinel */
	},
};

static const struct of_device_id pruss_of_match[] = {
	{ .compatible = "ti,am335x-pruss", .data = &am335x_priv_data, },
	{ .compatible = "ti,am4372-pruss", .data = &am4372_priv_data, },
	{ .compatible = "ti,am5728-pruss", .data = &am5728_match_data, },
	{},
};
MODULE_DEVICE_TABLE(of, pruss_of_match);

static struct platform_driver pruss_driver = {
	.driver = {
		.name   = "pruss-rproc",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(pruss_of_match),
	},
	.probe  = pruss_probe,
	.remove = pruss_remove,
};

static int __init pruss_init(void)
{
	int ret;

	ret = platform_driver_register(&pruss_driver);
	if (ret) {
		pr_err("platform driver register failed for pruss_driver, %d\n",
		       ret);
		return ret;
	}

	ret = platform_driver_register(&pru_rproc_driver);
	if (ret) {
		pr_err("platform driver register failed for pru_rproc_driver, %d\n",
		       ret);
		platform_driver_unregister(&pruss_driver);
	}

	return 0;
}
module_init(pruss_init);

static void __exit pruss_exit(void)
{
	platform_driver_unregister(&pru_rproc_driver);
	platform_driver_unregister(&pruss_driver);
}
module_exit(pruss_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("PRU-ICSS Remote Processor driver");
MODULE_AUTHOR("Suman Anna <s-anna@ti.com>");
