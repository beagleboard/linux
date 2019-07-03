// SPDX-License-Identifier: GPL-2.0-only
/*
 * PRU-ICSS remoteproc driver for various TI SoCs
 *
 * Copyright (C) 2014-2020 Texas Instruments Incorporated - http://www.ti.com/
 *	Suman Anna <s-anna@ti.com>
 *	Andrew F. Davis <afd@ti.com>
 */

#include <linux/bitops.h>
#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/omap-mailbox.h>
#include <linux/pruss_driver.h>
#include <linux/pruss_intc.h>
#include <linux/irqchip/irq-pruss-intc.h>
#include <linux/remoteproc.h>

#include "remoteproc_internal.h"
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

/* PRU Core IRAM address masks */
#define PRU0_IRAM_ADDR_MASK	0x34000
#define PRU1_IRAM_ADDR_MASK	0x38000

/**
 * enum pru_iomem - PRU core memory/register range identifiers
 */
enum pru_iomem {
	PRU_IOMEM_IRAM = 0,
	PRU_IOMEM_CTRL,
	PRU_IOMEM_DEBUG,
	PRU_IOMEM_MAX,
};

/**
 * struct pru_rproc - PRU remoteproc structure
 * @id: id of the PRU core within the PRUSS
 * @dev: PRU core device pointer
 * @pruss: back-reference to parent PRUSS structure
 * @rproc: remoteproc pointer for this PRU core
 * @mbox: mailbox channel handle used for vring signalling with MPU
 * @client: mailbox client to request the mailbox channel
 * @irq_ring: IRQ number to use for processing vring buffers
 * @irq_kick: IRQ number to use to perform virtio kick
 * @mem_regions: data for each of the PRU memory regions
 * @intc_config: PRU INTC configuration data
 * @iram_da: device address of Instruction RAM for this PRU
 * @pdram_da: device address of primary Data RAM for this PRU
 * @sdram_da: device address of secondary Data RAM for this PRU
 * @shrdram_da: device address of shared Data RAM
 * @fw_name: name of firmware image used during loading
 * @dbg_single_step: debug state variable to set PRU into single step mode
 * @dbg_continuous: debug state variable to restore PRU execution mode
 */
struct pru_rproc {
	int id;
	struct device *dev;
	struct pruss *pruss;
	struct rproc *rproc;
	struct mbox_chan *mbox;
	struct mbox_client client;
	int irq_vring;
	int irq_kick;
	struct pruss_mem_region mem_regions[PRU_IOMEM_MAX];
	struct pruss_intc_config intc_config;
	u32 iram_da;
	u32 pdram_da;
	u32 sdram_da;
	u32 shrdram_da;
	const char *fw_name;
	u32 dbg_single_step;
	u32 dbg_continuous;
};

static void *pru_d_da_to_va(struct pru_rproc *pru, u32 da, int len);

static inline u32 pru_control_read_reg(struct pru_rproc *pru, unsigned int reg)
{
	return readl_relaxed(pru->mem_regions[PRU_IOMEM_CTRL].va + reg);
}

static inline
void pru_control_write_reg(struct pru_rproc *pru, unsigned int reg, u32 val)
{
	writel_relaxed(val, pru->mem_regions[PRU_IOMEM_CTRL].va + reg);
}

static inline u32 pru_debug_read_reg(struct pru_rproc *pru, unsigned int reg)
{
	return readl_relaxed(pru->mem_regions[PRU_IOMEM_DEBUG].va + reg);
}

static inline
void pru_debug_write_reg(struct pru_rproc *pru, unsigned int reg, u32 val)
{
	writel_relaxed(val, pru->mem_regions[PRU_IOMEM_DEBUG].va + reg);
}

static int regs_show(struct seq_file *s, void *data)
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

DEFINE_SHOW_ATTRIBUTE(regs);

/*
 * Control PRU single-step mode
 *
 * This is a debug helper function used for controlling the single-step
 * mode of the PRU. The PRU Debug registers are not accessible when the
 * PRU is in RUNNING state.
 *
 * Writing a non-zero value sets the PRU into single-step mode irrespective
 * of its previous state. The PRU mode is saved only on the first set into
 * a single-step mode. Writing a zero value will restore the PRU into its
 * original mode.
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
			    rproc, &regs_fops);
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
	u32 msg = omap_mbox_message(data);

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
 * used for each virtqueue kick, or both virtqueues are processed on a
 * single event. The latter is chosen to conserve the usable PRU system
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
	mbox_msg_t msg = (mbox_msg_t)vq_id;

	dev_dbg(dev, "kicking vqid %d on PRU%d\n", vq_id, pru->id);

	if (pru->irq_kick > 0) {
		ret = pruss_intc_trigger(pru->irq_kick);
		if (ret < 0)
			dev_err(dev, "pruss_intc_trigger failed: %d\n", ret);
	} else if (pru->mbox) {
		/*
		 * send the index of the triggered virtqueue in the mailbox
		 * payload
		 */
		ret = mbox_send_message(pru->mbox, (void *)msg);
		if (ret < 0)
			dev_err(dev, "mbox_send_message failed: %d\n", ret);
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
	pruss_intc_unconfigure(pru->dev, &pru->intc_config);
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
	pruss_intc_unconfigure(pru->dev, &pru->intc_config);

	return 0;
}

/*
 * parse the custom PRU interrupt map resource and configure the INTC
 * appropriately
 */
static int pru_handle_vendor_intrmap(struct rproc *rproc,
				     struct fw_rsc_vendor *rsc)
{
	struct device *dev = rproc->dev.parent;
	struct pru_rproc *pru = rproc->priv;
	struct pruss_event_chnl *event_chnl_map;
	struct fw_rsc_pruss_intrmap *intr_rsc =
		(struct fw_rsc_pruss_intrmap *)rsc->data;
	int i, ret;
	s8 sys_evt, chnl, intr_no;

	dev_dbg(dev, "version %d event_chnl_map_size %d event_chnl_map_addr 0x%x\n",
		rsc->u.st.st_ver, intr_rsc->event_chnl_map_size,
		intr_rsc->event_chnl_map_addr);

	if (rsc->u.st.st_ver != 0) {
		dev_err(dev, "only PRU interrupt resource version 0 supported\n");
		return -EINVAL;
	}

	if (intr_rsc->event_chnl_map_size < 0 ||
	    intr_rsc->event_chnl_map_size >= MAX_PRU_SYS_EVENTS) {
		dev_err(dev, "PRU interrupt resource has more events than present on hardware\n");
		return -EINVAL;
	}

	/*
	 * XXX: The event_chnl_map_addr mapping is currently a pointer in device
	 * memory, evaluate if this needs to be directly in firmware file.
	 */
	event_chnl_map = pru_d_da_to_va(pru, intr_rsc->event_chnl_map_addr,
					intr_rsc->event_chnl_map_size *
					sizeof(*event_chnl_map));
	if (!event_chnl_map) {
		dev_err(dev, "PRU interrupt resource has inadequate event_chnl_map configuration\n");
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

		if (intr_no >= MAX_PRU_CHANNELS) {
			dev_err(dev, "bad intr mapping for chnl %d, intr_no %d\n",
				i, intr_no);
			return -EINVAL;
		}

		pru->intc_config.ch_to_host[i] = intr_no;
		dev_dbg(dev, "chnl-to-host[%d] -> %d\n", i, intr_no);
	}

	ret = pruss_intc_configure(pru->dev, &pru->intc_config);
	if (ret)
		dev_err(dev, "failed to configure pruss intc %d\n", ret);

	return ret;
}

/* PRU-specific vendor resource handler */
static int pru_rproc_handle_vendor_rsc(struct rproc *rproc,
				       struct fw_rsc_vendor *rsc)
{
	struct device *dev = rproc->dev.parent;
	int ret = -EINVAL;

	switch (rsc->u.st.st_type) {
	case PRUSS_RSC_INTRS:
		ret = pru_handle_vendor_intrmap(rproc, rsc);
		break;
	default:
		dev_err(dev, "%s: cannot handle unknown type %d\n", __func__,
			rsc->u.st.st_type);
	}

	return ret;
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
	    da + len <= pru->iram_da + pru->mem_regions[PRU_IOMEM_IRAM].size) {
		offset = da - pru->iram_da;
		va = (__force void *)(pru->mem_regions[PRU_IOMEM_IRAM].va +
				      offset);
	}

	return va;
}

/*
 * Provide address translations for only PRU Data RAMs through the remoteproc
 * core for any PRU client drivers. The PRU Instruction RAM access is restricted
 * only to the PRU loader code.
 */
static void *pru_rproc_da_to_va(struct rproc *rproc, u64 da, int len)
{
	struct pru_rproc *pru = rproc->priv;

	return pru_d_da_to_va(pru, da, len);
}

/* PRU-specific address translator used by PRU loader */
static void *pru_da_to_va(struct rproc *rproc, u64 da, int len, bool is_iram)
{
	struct pru_rproc *pru = rproc->priv;
	void *va;

	if (is_iram)
		va = pru_i_da_to_va(pru, da, len);
	else
		va = pru_d_da_to_va(pru, da, len);

	return va;
}

static struct rproc_ops pru_rproc_ops = {
	.start			= pru_rproc_start,
	.stop			= pru_rproc_stop,
	.kick			= pru_rproc_kick,
	.handle_vendor_rsc	= pru_rproc_handle_vendor_rsc,
	.da_to_va		= pru_rproc_da_to_va,
};

static int
pru_rproc_load_elf_segments(struct rproc *rproc, const struct firmware *fw)
{
	struct device *dev = &rproc->dev;
	struct elf32_hdr *ehdr;
	struct elf32_phdr *phdr;
	int i, ret = 0;
	const u8 *elf_data = fw->data;

	ehdr = (struct elf32_hdr *)elf_data;
	phdr = (struct elf32_phdr *)(elf_data + ehdr->e_phoff);

	/* go through the available ELF segments */
	for (i = 0; i < ehdr->e_phnum; i++, phdr++) {
		u32 da = phdr->p_paddr;
		u32 memsz = phdr->p_memsz;
		u32 filesz = phdr->p_filesz;
		u32 offset = phdr->p_offset;
		bool is_iram;
		void *ptr;

		if (phdr->p_type != PT_LOAD)
			continue;

		dev_dbg(dev, "phdr: type %d da 0x%x memsz 0x%x filesz 0x%x\n",
			phdr->p_type, da, memsz, filesz);

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

		/* grab the kernel address for this device address */
		is_iram = phdr->p_flags & PF_X;
		ptr = pru_da_to_va(rproc, da, memsz, is_iram);
		if (!ptr) {
			dev_err(dev, "bad phdr da 0x%x mem 0x%x\n", da, memsz);
			ret = -EINVAL;
			break;
		}

		/* skip the memzero logic performed by remoteproc ELF loader */
		if (!phdr->p_filesz)
			continue;

		memcpy(ptr, elf_data + phdr->p_offset, filesz);
	}

	return ret;
}

/*
 * compute PRU id based on the IRAM addresses. The PRU IRAMs are
 * always at a particular offset within the PRUSS address space.
 * The other alternative is to use static data for each core (not a
 * hardware property to define it in DT), and the id can always be
 * computated using this inherent address logic.
 */
static int pru_rproc_set_id(struct pru_rproc *pru)
{
	int ret = 0;
	u32 mask1 = PRU0_IRAM_ADDR_MASK;
	u32 mask2 = PRU1_IRAM_ADDR_MASK;

	if ((pru->mem_regions[PRU_IOMEM_IRAM].pa & mask1) == mask1)
		pru->id = 0;
	else if ((pru->mem_regions[PRU_IOMEM_IRAM].pa & mask2) == mask2)
		pru->id = 1;
	else
		ret = -EINVAL;

	return ret;
}

static int pru_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct platform_device *ppdev = to_platform_device(dev->parent);
	struct pru_rproc *pru;
	const char *fw_name;
	struct rproc *rproc = NULL;
	struct mbox_client *client;
	struct resource *res;
	int i, ret;
	const char *mem_names[PRU_IOMEM_MAX] = { "iram", "control", "debug" };

	if (!np) {
		dev_err(dev, "Non-DT platform device not supported\n");
		return -ENODEV;
	}

	ret = of_property_read_string(np, "firmware-name", &fw_name);
	if (ret) {
		dev_err(dev, "unable to retrieve firmware-name %d\n", ret);
		return ret;
	}

	rproc = rproc_alloc(dev, pdev->name, &pru_rproc_ops, fw_name,
			    sizeof(*pru));
	if (!rproc) {
		dev_err(dev, "rproc_alloc failed\n");
		return -ENOMEM;
	}
	/* use a custom load function to deal with PRU-specific quirks */
	rproc->ops->load = pru_rproc_load_elf_segments;

	/* error recovery is not supported for PRUs */
	rproc->recovery_disabled = true;

	/*
	 * rproc_add will auto-boot the processor normally, but this is
	 * not desired with PRU client driven boot-flow methodology. A PRU
	 * application/client driver will boot the corresponding PRU
	 * remote-processor as part of its state machine either through
	 * the remoteproc sysfs interface or through the equivalent kernel API
	 */
	rproc->auto_boot = false;

	pru = rproc->priv;
	pru->dev = dev;
	pru->pruss = platform_get_drvdata(ppdev);
	pru->rproc = rproc;
	pru->fw_name = fw_name;

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

		dev_dbg(dev, "memory %8s: pa %pa size 0x%zx va %pK\n",
			mem_names[i], &pru->mem_regions[i].pa,
			pru->mem_regions[i].size, pru->mem_regions[i].va);
	}

	ret = pru_rproc_set_id(pru);
	if (ret < 0)
		goto free_rproc;

	platform_set_drvdata(pdev, rproc);

	/* get optional vring and kick interrupts for supporting virtio rpmsg */
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

	/*
	 * get optional mailbox for virtio rpmsg signalling if vring and kick
	 * interrupts are not specified for OMAP architecture based SoCs
	 */
	if (pru->irq_vring <= 0 && pru->irq_kick <= 0 &&
	    !of_device_is_compatible(np, "ti,k2g-pru")) {
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
			dev_dbg(dev, "unable to get mailbox channel, status = %d\n",
				ret);
		}
	}

	ret = rproc_add(pru->rproc);
	if (ret) {
		dev_err(dev, "rproc_add failed: %d\n", ret);
		goto put_mbox;
	}

	pru_rproc_create_debug_entries(rproc);

	dev_info(dev, "PRU rproc node %pOF probed successfully\n", np);

	return 0;

put_mbox:
	mbox_free_channel(pru->mbox);
free_rproc:
	rproc_free(rproc);
	return ret;
}

static int pru_rproc_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rproc *rproc = platform_get_drvdata(pdev);
	struct pru_rproc *pru = rproc->priv;

	dev_info(dev, "%s: removing rproc %s\n", __func__, rproc->name);

	mbox_free_channel(pru->mbox);

	rproc_del(rproc);
	rproc_free(rproc);

	return 0;
}

static const struct of_device_id pru_rproc_match[] = {
	{ .compatible = "ti,am3356-pru", },
	{ .compatible = "ti,am4376-pru", },
	{ .compatible = "ti,am5728-pru", },
	{ .compatible = "ti,k2g-pru",    },
	{},
};
MODULE_DEVICE_TABLE(of, pru_rproc_match);

static struct platform_driver pru_rproc_driver = {
	.driver = {
		.name   = "pru-rproc",
		.of_match_table = pru_rproc_match,
		.suppress_bind_attrs = true,
	},
	.probe  = pru_rproc_probe,
	.remove = pru_rproc_remove,
};
module_platform_driver(pru_rproc_driver);

MODULE_AUTHOR("Suman Anna <s-anna@ti.com>");
MODULE_DESCRIPTION("PRU-ICSS Remote Processor Driver");
MODULE_LICENSE("GPL v2");
