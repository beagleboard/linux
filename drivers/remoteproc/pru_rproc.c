/*
 * PRU-ICSS remoteproc driver for various TI SoCs
 *
 * Copyright (C) 2014-2016 Texas Instruments Incorporated - http://www.ti.com/
 *	Suman Anna <s-anna@ti.com>
 *	Andrew F. Davis <afd@ti.com>
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

#include <linux/bitops.h>
#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/remoteproc.h>
#include <linux/pruss.h>

#include "remoteproc_internal.h"
#include "pruss.h"
#include "pru_rproc.h"

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

/* Bit-field definitions for PRU functional capabilities */
#define PRU_FUNC_CAPS_ETHERNET	BIT(0)

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
 * @caps: functional capabilities the PRU core can support
 * @fw_name: firmware name to be used for the PRU core
 * @eth_fw_name: firmware name to be used for PRUSS ethernet usecases on IDKs
 */
struct pru_private_data {
	u32 id;
	int caps;
	const char *fw_name;
	const char *eth_fw_name;
};

/**
 * struct pru_match_private_data - private data to handle multiple instances
 * @device_name: device name of the PRU processor core instance
 * @priv_data: PRU driver private data for this PRU processor core instance
 */
struct pru_match_private_data {
	const char *device_name;
	struct pru_private_data *priv_data;
};

/**
 * struct pru_rproc: PRU remoteproc structure
 * @id: id of the PRU core within the PRUSS
 * @pruss: back-reference to parent PRUSS structure
 * @rproc: remoteproc pointer for this PRU core
 * @mbox: mailbox channel handle used for vring signalling with MPU
 * @client: mailbox client to request the mailbox channel
 * @irq_ring: IRQ number to use for processing vring buffers
 * @irq_kick: IRQ number to use to perform virtio kick
 * @mem_regions: data for each of the PRU memory regions
 * @intc_config: PRU INTC configuration data
 * @rmw_lock: lock for read, modify, write operations on registers
 * @iram_da: device address of Instruction RAM for this PRU
 * @pdram_da: device address of primary Data RAM for this PRU
 * @sdram_da: device address of secondary Data RAM for this PRU
 * @shrdram_da: device address of shared Data RAM
 * @fw_name: name of firmware image used during loading
 * @dbg_single_step: debug state variable to set PRU into single step mode
 * @dbg_continuous: debug state variable to restore PRU execution mode
 * @use_eth: flag to indicate ethernet usecase functionality
 */
struct pru_rproc {
	int id;
	struct pruss *pruss;
	struct rproc *rproc;
	struct mbox_chan *mbox;
	struct mbox_client client;
	int irq_vring;
	int irq_kick;
	struct pruss_mem_region mem_regions[PRU_MEM_MAX];
	struct pruss_intc_config intc_config;
	spinlock_t rmw_lock; /* register access lock */
	u32 iram_da;
	u32 pdram_da;
	u32 sdram_da;
	u32 shrdram_da;
	const char *fw_name;
	u32 dbg_single_step;
	u32 dbg_continuous;
	bool use_eth;
};

static bool use_eth_fw = true; /* ignored for non-IDK platforms */
module_param(use_eth_fw, bool, S_IRUGO);
MODULE_PARM_DESC(use_eth_fw, "Use Ethernet firmware on applicable PRUs");

static inline u32 pru_control_read_reg(struct pru_rproc *pru, unsigned int reg)
{
	return readl_relaxed(pru->mem_regions[PRU_MEM_CTRL].va + reg);
}

static inline
void pru_control_write_reg(struct pru_rproc *pru, unsigned int reg, u32 val)
{
	writel_relaxed(val, pru->mem_regions[PRU_MEM_CTRL].va + reg);
}

static inline
void pru_control_set_reg(struct pru_rproc *pru, unsigned int reg,
			 u32 mask, u32 set)
{
	u32 val;
	unsigned long flags;

	spin_lock_irqsave(&pru->rmw_lock, flags);

	val = pru_control_read_reg(pru, reg);
	val &= ~mask;
	val |= (set & mask);
	pru_control_write_reg(pru, reg, val);

	spin_unlock_irqrestore(&pru->rmw_lock, flags);
}

/**
 * pru_rproc_set_ctable() - set the constant table index for the PRU
 * @rproc: the rproc instance of the PRU
 * @c: constant table index to set
 * @addr: physical address to set it to
 */
int pru_rproc_set_ctable(struct rproc *rproc, enum pru_ctable_idx c, u32 addr)
{
	struct pru_rproc *pru = rproc->priv;
	unsigned reg;
	u32 mask, set;
	u16 idx;
	u16 idx_mask;

	/* pointer is 16 bit and index is 8-bit so mask out the rest */
	idx_mask = (c >= PRU_C28) ? 0xFFFF : 0xFF;

	/* ctable uses bit 8 and upwards only */
	idx = (addr >> 8) & idx_mask;

	/* configurable ctable (i.e. C24) starts at PRU_CTRL_CTBIR0 */
	reg = PRU_CTRL_CTBIR0 + 4 * (c >> 1);
	mask = idx_mask << (16 * (c & 1));
	set = idx << (16 * (c & 1));

	pru_control_set_reg(pru, reg, mask, set);

	return 0;
}
EXPORT_SYMBOL_GPL(pru_rproc_set_ctable);

static inline u32 pru_debug_read_reg(struct pru_rproc *pru, unsigned int reg)
{
	return readl_relaxed(pru->mem_regions[PRU_MEM_DEBUG].va + reg);
}

static inline
void pru_debug_write_reg(struct pru_rproc *pru, unsigned int reg, u32 val)
{
	writel_relaxed(val, pru->mem_regions[PRU_MEM_DEBUG].va + reg);
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
	struct pruss_mem_region dram0, dram1, shrd_ram;
	struct pruss *pruss = pru->pruss;
	u32 offset;
	void *va = NULL;

	if (len <= 0)
		return NULL;

	dram0 = pruss->mem_regions[PRUSS_MEM_DRAM0];
	dram1 = pruss->mem_regions[PRUSS_MEM_DRAM1];
	/* PRU1 has its local RAM addresses reversed */
	if (pru->id == 1)
		swap(dram0, dram1);
	shrd_ram = pruss->mem_regions[PRUSS_MEM_SHRD_RAM2];

	if (da >= pru->pdram_da && da + len <= pru->pdram_da + dram0.size) {
		offset = da - pru->pdram_da;
		va = (__force void *)(dram0.va + offset);
	} else if (da >= pru->sdram_da &&
		   da + len <= pru->sdram_da + dram1.size) {
		offset = da - pru->sdram_da;
		va = (__force void *)(dram1.va + offset);
	} else if (da >= pru->shrdram_da &&
		   da + len <= pru->shrdram_da + shrd_ram.size) {
		offset = da - pru->shrdram_da;
		va = (__force void *)(shrd_ram.va + offset);
	}

	return va;
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
	void *va = NULL;

	if (len <= 0)
		return NULL;

	if (da >= pru->iram_da &&
	    da + len <= pru->iram_da + pru->mem_regions[PRU_MEM_IRAM].size) {
		offset = da - pru->iram_da;
		va = (__force void *)(pru->mem_regions[PRU_MEM_IRAM].va +
				      offset);
	}

	return va;
}

static int pru_rproc_debug_read_regs(struct seq_file *s, void *data)
{
	struct rproc *rproc = s->private;
	struct pru_rproc *pru = rproc->priv;
	int i, nregs = 32;
	u32 pru_sts;
	int pru_is_running;

	seq_puts(s, "============== Control Registers ==============\n");
	seq_printf(s, "CTRL      := 0x%08x\n",
		   pru_control_read_reg(pru, PRU_CTRL_CTRL));
	pru_sts = pru_control_read_reg(pru, PRU_CTRL_STS);
	seq_printf(s, "STS (PC)  := 0x%08x (0x%08x)\n", pru_sts, pru_sts << 2);
	seq_printf(s, "WAKEUP_EN := 0x%08x\n",
		   pru_control_read_reg(pru, PRU_CTRL_WAKEUP_EN));
	seq_printf(s, "CYCLE     := 0x%08x\n",
		   pru_control_read_reg(pru, PRU_CTRL_CYCLE));
	seq_printf(s, "STALL     := 0x%08x\n",
		   pru_control_read_reg(pru, PRU_CTRL_STALL));
	seq_printf(s, "CTBIR0    := 0x%08x\n",
		   pru_control_read_reg(pru, PRU_CTRL_CTBIR0));
	seq_printf(s, "CTBIR1    := 0x%08x\n",
		   pru_control_read_reg(pru, PRU_CTRL_CTBIR1));
	seq_printf(s, "CTPPR0    := 0x%08x\n",
		   pru_control_read_reg(pru, PRU_CTRL_CTPPR0));
	seq_printf(s, "CTPPR1    := 0x%08x\n",
		   pru_control_read_reg(pru, PRU_CTRL_CTPPR1));

	seq_puts(s, "=============== Debug Registers ===============\n");
	pru_is_running = pru_control_read_reg(pru, PRU_CTRL_CTRL) &
				CTRL_CTRL_RUNSTATE;
	if (pru_is_running) {
		seq_puts(s, "PRU is executing, cannot print/access debug registers.\n");
		return 0;
	}

	for (i = 0; i < nregs; i++) {
		seq_printf(s, "GPREG%-2d := 0x%08x\tCT_REG%-2d := 0x%08x\n",
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
		dev_dbg(dev, "no message was found in vqid %d\n", msg);
}

/**
 * pru_rproc_vring_interrupt() - interrupt handler for processing vrings
 * @irq: irq number associated with the PRU event MPU is listening on
 * @data: interrupt handler data, will be a PRU rproc structure
 *
 * This handler is used by the PRU remoteproc driver when using PRU system
 * events for processing the virtqueues. Unlike the mailbox IP, there is
 * no payload associated with an interrupt, so either a unique event is
 * used for each virtqueue kick, or a both virtqueues are processed on
 * a single event. The latter is chosen to conserve the usable PRU system
 * events.
 */
static irqreturn_t pru_rproc_vring_interrupt(int irq, void *data)
{
	struct pru_rproc *pru = data;

	dev_dbg(&pru->rproc->dev, "got vring irq\n");

	/* process incoming buffers on both the Rx and Tx vrings */
	rproc_vq_interrupt(pru->rproc, 0);
	rproc_vq_interrupt(pru->rproc, 1);

	return IRQ_HANDLED;
}

/* kick a virtqueue */
static void pru_rproc_kick(struct rproc *rproc, int vq_id)
{
	struct device *dev = &rproc->dev;
	struct pru_rproc *pru = rproc->priv;
	int ret;

	dev_dbg(dev, "kicking vqid %d on PRU%d\n", vq_id, pru->id);

	if (pru->mbox) {
		/*
		 * send the index of the triggered virtqueue in the mailbox
		 * payload
		 */
		ret = mbox_send_message(pru->mbox, (void *)vq_id);
		if (ret < 0)
			dev_err(dev, "mbox_send_message failed: %d\n", ret);
	} else if (pru->irq_kick > 0) {
		ret = pruss_intc_trigger(pru->irq_kick);
		if (ret < 0)
			dev_err(dev, "pruss_intc_trigger failed: %d\n", ret);
	}
}

/* start a PRU core */
static int pru_rproc_start(struct rproc *rproc)
{
	struct device *dev = &rproc->dev;
	struct pru_rproc *pru = rproc->priv;
	u32 val;
	int ret;

	dev_dbg(dev, "starting PRU%d: entry-point = 0x%x\n",
		pru->id, (rproc->bootaddr >> 2));

	if (!list_empty(&pru->rproc->rvdevs)) {
		if (!pru->mbox && (pru->irq_vring <= 0 || pru->irq_kick <= 0)) {
			dev_err(dev, "virtio vring interrupt mechanisms are not provided\n");
			ret = -EINVAL;
			goto fail;
		}

		if (!pru->mbox && pru->irq_vring > 0) {
			ret = request_threaded_irq(pru->irq_vring, NULL,
						   pru_rproc_vring_interrupt,
						   IRQF_ONESHOT, dev_name(dev),
						   pru);
			if (ret) {
				dev_err(dev, "failed to enable vring interrupt, ret = %d\n",
					ret);
				goto fail;
			}
		}
	}

	val = CTRL_CTRL_EN | ((rproc->bootaddr >> 2) << 16);
	pru_control_write_reg(pru, PRU_CTRL_CTRL, val);

	return 0;

fail:
	pruss_intc_unconfigure(pru->pruss, &pru->intc_config);
	return ret;
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

	if (!list_empty(&pru->rproc->rvdevs) &&
	    !pru->mbox && pru->irq_vring > 0)
		free_irq(pru->irq_vring, pru);

	/* undo INTC config */
	pruss_intc_unconfigure(pru->pruss, &pru->intc_config);

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

	/* init intc_config to defaults */
	for (i = 0; i < ARRAY_SIZE(pru->intc_config.sysev_to_ch); i++)
		pru->intc_config.sysev_to_ch[i] = -1;

	for (i = 0; i < ARRAY_SIZE(pru->intc_config.ch_to_host); i++)
		pru->intc_config.ch_to_host[i] = -1;

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

		pru->intc_config.sysev_to_ch[sys_evt] = chnl;
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

		pru->intc_config.ch_to_host[i] = intr_no;
		dev_dbg(dev, "chnl-to-host[%d] -> %d\n", i, intr_no);
	}

	ret = pruss_intc_configure(pruss, &pru->intc_config);
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
						(struct fw_rsc_custom_intrmap *)
						rsc->data);
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
	u32 exec_flag;

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
	struct pru_private_data *pdata = NULL;

	match = of_match_device(pru_rproc_match, &pdev->dev);
	if (!match)
		return ERR_PTR(-ENODEV);

	for (data = match->data; data && data->device_name; data++) {
		if (!strcmp(dev_name(&pdev->dev), data->device_name))
			pdata = data->priv_data;
	}

	/* fixup PRU capability differences between AM571x and AM572x IDKs */
	if (pdata && of_machine_is_compatible("ti,am5718-idk"))
		pdata->caps = PRU_FUNC_CAPS_ETHERNET;

	return pdata;
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
	bool use_eth = false;
	u32 mux_sel;

	if (!np) {
		dev_err(dev, "Non-DT platform device not supported\n");
		return -ENODEV;
	}

	pdata = pru_rproc_get_private_data(pdev);
	if (IS_ERR_OR_NULL(pdata) || !pdata->fw_name) {
		dev_err(dev, "missing or incomplete PRU-private data\n");
		return -ENODEV;
	}

	/*
	 * use a different firmware name for PRU cores supporting
	 * PRUSS ethernet on specific boards
	 */
	if (of_machine_is_compatible("ti,am3359-icev2") ||
	    of_machine_is_compatible("ti,am437x-idk-evm") ||
	    of_machine_is_compatible("ti,am5728-idk") ||
	    of_machine_is_compatible("ti,am5718-idk") ||
	    of_machine_is_compatible("ti,k2g-ice")) {
		if (use_eth_fw && (pdata->caps & PRU_FUNC_CAPS_ETHERNET))
			use_eth = true;
	}

	rproc = rproc_alloc(dev, pdev->name, &pru_rproc_ops,
			    (use_eth ? pdata->eth_fw_name : pdata->fw_name),
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
	pru->fw_name = use_eth ? pdata->eth_fw_name : pdata->fw_name;
	pru->use_eth = use_eth;
	spin_lock_init(&pru->rmw_lock);

	/* XXX: get this from match data if different in the future */
	pru->iram_da = 0;
	pru->pdram_da = 0;
	pru->sdram_da = 0x2000;
	pru->shrdram_da = 0x10000;

	for (i = 0; i < ARRAY_SIZE(mem_names); i++) {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   mem_names[i]);
		pru->mem_regions[i].va = devm_ioremap_resource(dev, res);
		if (IS_ERR(pru->mem_regions[i].va)) {
			dev_err(dev, "failed to parse and map memory resource %d %s\n",
				i, mem_names[i]);
			ret = PTR_ERR(pru->mem_regions[i].va);
			goto free_rproc;
		}
		pru->mem_regions[i].pa = res->start;
		pru->mem_regions[i].size = resource_size(res);

		dev_dbg(dev, "memory %8s: pa %pa size 0x%x va %p\n",
			mem_names[i], &pru->mem_regions[i].pa,
			pru->mem_regions[i].size, pru->mem_regions[i].va);
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
		pru->mbox = NULL;
		dev_dbg(dev, "mbox_request_channel failed: %d\n", ret);
	}

	pru->irq_vring = platform_get_irq_byname(pdev, "vring");
	if (pru->irq_vring <= 0) {
		ret = pru->irq_vring;
		if (ret == -EPROBE_DEFER)
			goto free_rproc;
		dev_dbg(dev, "unable to get vring interrupt, status = %d\n",
			ret);
	}

	pru->irq_kick = platform_get_irq_byname(pdev, "kick");
	if (pru->irq_kick <= 0) {
		ret = pru->irq_kick;
		if (ret == -EPROBE_DEFER)
			goto free_rproc;
		dev_dbg(dev, "unable to get kick interrupt, status = %d\n",
			ret);
	}

	if (pru->mbox && (pru->irq_vring > 0 || pru->irq_kick > 0))
		dev_warn(dev, "both mailbox and vring/kick system events defined\n");

	ret = rproc_add(pru->rproc);
	if (ret) {
		dev_err(dev, "rproc_add failed: %d\n", ret);
		goto put_mbox;
	}

	if ((of_machine_is_compatible("ti,am5718-idk") ||
	     of_machine_is_compatible("ti,k2g-ice")) && pru->use_eth &&
	    !of_property_read_u32(np, "ti,pruss-gp-mux-sel", &mux_sel)) {
		if (mux_sel < PRUSS_GP_MUX_SEL_GP ||
		    mux_sel >= PRUSS_GP_MUX_MAX) {
			dev_err(dev, "invalid gp_mux_sel %d\n", mux_sel);
			ret = -EINVAL;
			goto del_rproc;
		}

		ret = pruss_cfg_set_gpmux(pru->pruss, pru->id, mux_sel);
		if (ret)
			goto del_rproc;
	}

	pru_rproc_create_debug_entries(rproc);

	/*
	 * rproc_add will boot the processor if the corresponding PRU
	 * has a virtio device published in its resource table. If not
	 * present, manually boot the PRU remoteproc, but only after
	 * the remoteproc core is done with loading the firmware image.
	 */
	if (!pru->use_eth) {
		wait_for_completion(&pru->rproc->firmware_loading_complete);
		if (list_empty(&pru->rproc->rvdevs)) {
			dev_info(dev, "booting the PRU core manually\n");
			ret = rproc_boot(pru->rproc);
			if (ret) {
				dev_err(dev, "rproc_boot failed\n");
				goto del_rproc;
			}
		}
	}

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

	if (!pru->use_eth) {
		if (list_empty(&pru->rproc->rvdevs)) {
			dev_info(dev, "stopping the manually booted PRU core\n");
			rproc_shutdown(pru->rproc);
		}
	}

	mbox_free_channel(pru->mbox);

	if ((of_machine_is_compatible("ti,am5718-idk") ||
	     of_machine_is_compatible("ti,k2g-ice")) && pru->use_eth)
		pruss_cfg_set_gpmux(pru->pruss, pru->id, PRUSS_GP_MUX_SEL_GP);

	rproc_del(rproc);
	rproc_put(rproc);

	return 0;
}

/* AM33xx PRU core-specific private data */
static struct pru_private_data am335x_pru0_rproc_pdata = {
	.id = 0,
	.caps = PRU_FUNC_CAPS_ETHERNET,
	.fw_name = "am335x-pru0-fw",
	.eth_fw_name = "ti-pruss/am335x-pru0-prueth-fw.elf",
};

static struct pru_private_data am335x_pru1_rproc_pdata = {
	.id = 1,
	.caps = PRU_FUNC_CAPS_ETHERNET,
	.fw_name = "am335x-pru1-fw",
	.eth_fw_name = "ti-pruss/am335x-pru1-prueth-fw.elf",
};

/* AM437x PRUSS1 PRU core-specific private data */
static struct pru_private_data am437x_pru1_0_rproc_pdata = {
	.id = 0,
	.caps = PRU_FUNC_CAPS_ETHERNET,
	.fw_name = "am437x-pru1_0-fw",
	.eth_fw_name = "ti-pruss/am437x-pru0-prueth-fw.elf"
};

static struct pru_private_data am437x_pru1_1_rproc_pdata = {
	.id = 1,
	.caps = PRU_FUNC_CAPS_ETHERNET,
	.fw_name = "am437x-pru1_1-fw",
	.eth_fw_name = "ti-pruss/am437x-pru1-prueth-fw.elf"
};

/* AM437x PRUSS0 PRU core-specific private data */
static struct pru_private_data am437x_pru0_0_rproc_pdata = {
	.id = 0,
	.fw_name = "am437x-pru0_0-fw",
};

static struct pru_private_data am437x_pru0_1_rproc_pdata = {
	.id = 1,
	.fw_name = "am437x-pru0_1-fw",
};

/* AM57xx PRUSS1 PRU core-specific private data */
static struct pru_private_data am57xx_pru1_0_rproc_pdata = {
	.id = 0,
	.fw_name = "am57xx-pru1_0-fw",
	.eth_fw_name = "ti-pruss/am57xx-pru0-prueth-fw.elf"
};

static struct pru_private_data am57xx_pru1_1_rproc_pdata = {
	.id = 1,
	.fw_name = "am57xx-pru1_1-fw",
	.eth_fw_name = "ti-pruss/am57xx-pru1-prueth-fw.elf"
};

/* AM57xx PRUSS2 PRU core-specific private data */
static struct pru_private_data am57xx_pru2_0_rproc_pdata = {
	.id = 0,
	.caps = PRU_FUNC_CAPS_ETHERNET,
	.fw_name = "am57xx-pru2_0-fw",
	.eth_fw_name = "ti-pruss/am57xx-pru0-prueth-fw.elf"
};

static struct pru_private_data am57xx_pru2_1_rproc_pdata = {
	.id = 1,
	.caps = PRU_FUNC_CAPS_ETHERNET,
	.fw_name = "am57xx-pru2_1-fw",
	.eth_fw_name = "ti-pruss/am57xx-pru1-prueth-fw.elf"
};

/* K2G PRUSS0 PRU core-specific private data */
static struct pru_private_data k2g_pru0_0_rproc_pdata = {
	.id = 0,
	.caps = PRU_FUNC_CAPS_ETHERNET,
	.fw_name = "k2g-pru0_0-fw",
	.eth_fw_name = "ti-pruss/k2g-pru0-prueth-fw.elf"
};

static struct pru_private_data k2g_pru0_1_rproc_pdata = {
	.id = 1,
	.caps = PRU_FUNC_CAPS_ETHERNET,
	.fw_name = "k2g-pru0_1-fw",
	.eth_fw_name = "ti-pruss/k2g-pru1-prueth-fw.elf"
};

static struct pru_private_data k2g_pru1_0_rproc_pdata = {
	.id = 0,
	.caps = PRU_FUNC_CAPS_ETHERNET,
	.fw_name = "k2g-pru1_0-fw",
	.eth_fw_name = "ti-pruss/k2g-pru0-prueth-fw.elf"
};

static struct pru_private_data k2g_pru1_1_rproc_pdata = {
	.id = 1,
	.caps = PRU_FUNC_CAPS_ETHERNET,
	.fw_name = "k2g-pru1_1-fw",
	.eth_fw_name = "ti-pruss/k2g-pru1-prueth-fw.elf"
};

/* AM33xx SoC-specific PRU Device data */
static struct pru_match_private_data am335x_pru_match_data[] = {
	{
		.device_name	= "4a334000.pru0",
		.priv_data	= &am335x_pru0_rproc_pdata,
	},
	{
		.device_name	= "4a338000.pru1",
		.priv_data	= &am335x_pru1_rproc_pdata,
	},
	{
		/* sentinel */
	},
};

/* AM43xx SoC-specific PRU Device data */
static struct pru_match_private_data am437x_pru_match_data[] = {
	{
		.device_name	= "54434000.pru0",
		.priv_data	= &am437x_pru1_0_rproc_pdata,
	},
	{
		.device_name	= "54438000.pru1",
		.priv_data	= &am437x_pru1_1_rproc_pdata,
	},
	{
		.device_name    = "54474000.pru0",
		.priv_data      = &am437x_pru0_0_rproc_pdata,
	},
	{
		.device_name    = "54478000.pru1",
		.priv_data      = &am437x_pru0_1_rproc_pdata,
	},
	{
		/* sentinel */
	},
};

/* AM57xx SoC-specific PRU Device data */
static struct pru_match_private_data am57xx_pru_match_data[] = {
	{
		.device_name	= "4b234000.pru0",
		.priv_data	= &am57xx_pru1_0_rproc_pdata,
	},
	{
		.device_name	= "4b238000.pru1",
		.priv_data	= &am57xx_pru1_1_rproc_pdata,
	},
	{
		.device_name	= "4b2b4000.pru0",
		.priv_data	= &am57xx_pru2_0_rproc_pdata,
	},
	{
		.device_name	= "4b2b8000.pru1",
		.priv_data	= &am57xx_pru2_1_rproc_pdata,
	},
	{
		/* sentinel */
	},
};

/* K2G SoC-specific PRU Device data */
static struct pru_match_private_data k2g_pru_match_data[] = {
	{
		.device_name	= "20ab4000.pru0",
		.priv_data	= &k2g_pru0_0_rproc_pdata,
	},
	{
		.device_name	= "20ab8000.pru1",
		.priv_data	= &k2g_pru0_1_rproc_pdata,
	},
	{
		.device_name	= "20af4000.pru0",
		.priv_data	= &k2g_pru1_0_rproc_pdata,
	},
	{
		.device_name	= "20af8000.pru1",
		.priv_data	= &k2g_pru1_1_rproc_pdata,
	},
	{
		/* sentinel */
	},
};

static const struct of_device_id pru_rproc_match[] = {
	{ .compatible = "ti,am3352-pru", .data = am335x_pru_match_data, },
	{ .compatible = "ti,am4372-pru", .data = am437x_pru_match_data, },
	{ .compatible = "ti,am5728-pru", .data = am57xx_pru_match_data, },
	{ .compatible = "ti,k2g-pru", .data = k2g_pru_match_data, },
	{},
};
MODULE_DEVICE_TABLE(of, pru_rproc_match);

static struct platform_driver pru_rproc_driver = {
	.driver = {
		.name   = "pru-rproc",
		.of_match_table = pru_rproc_match,
	},
	.probe  = pru_rproc_probe,
	.remove = pru_rproc_remove,
};
module_platform_driver(pru_rproc_driver);

MODULE_AUTHOR("Suman Anna <s-anna@ti.com>");
MODULE_DESCRIPTION("PRU-ICSS Remote Processor Driver");
MODULE_LICENSE("GPL v2");
