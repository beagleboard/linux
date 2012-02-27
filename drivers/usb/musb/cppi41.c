/*
 * CPPI 4.1 support
 *
 * Copyright (C) 2008-2009 MontaVista Software, Inc. <source@mvista.com>
 *
 * Based on the PAL CPPI 4.1 implementation
 * Copyright (C) 1998-2006 Texas Instruments Incorporated
 *
 * This file contains the main implementation for CPPI 4.1 common peripherals,
 * including the DMA Controllers and the Queue Managers.
 *
 * This program is free software; you can distribute it and/or modify it
 * under the terms of the GNU General Public License (Version 2) as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 */

#include <linux/io.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>

#include "cppi41.h"

#undef	CPPI41_DEBUG

#ifdef	CPPI41_DEBUG
#define DBG(format, args...) printk(format, ##args)
#else
#define DBG(format, args...)
#endif

static struct {
	void *virt_addr;
	dma_addr_t phys_addr;
	u32     size;
} linking_ram[CPPI41_NUM_QUEUE_MGR];

static u32 *allocated_queues[CPPI41_NUM_QUEUE_MGR];

/* First 32 packet descriptors are reserved for unallocated memory regions. */
static u32 next_desc_index[CPPI41_NUM_QUEUE_MGR] = { 1 << 5 };
static u8  next_mem_rgn[CPPI41_NUM_QUEUE_MGR];

static struct {
	size_t rgn_size;
	void *virt_addr;
	dma_addr_t phys_addr;
	struct cppi41_queue_obj queue_obj;
	u8 mem_rgn;
	u16 q_mgr;
	u16 q_num;
	u32 num_desc;
} dma_teardown[CPPI41_NUM_DMA_BLOCK];

struct cppi41_dma_sched_tbl_t {
	u8      pos;
	u8      dma_ch;
	u8      is_tx;
	u8      enb;
};

struct cppi41_dma_sched_tbl_t dma_sched_tbl[MAX_SCHED_TBL_ENTRY] = {
	/*pos  dma_ch#  is_tx  enb/dis*/
	{ 0,    0,      0,      1},
	{ 1,    0,      1,      1},
	{ 2,    1,      0,      1},
	{ 3,    1,      1,      1},
	{ 4,    2,      0,      1},
	{ 5,    2,      1,      1},
	{ 6,    3,      0,      1},
	{ 7,    3,      1,      1}
};

struct cppi41_queue_mgr cppi41_queue_mgr[CPPI41_NUM_QUEUE_MGR];
EXPORT_SYMBOL(cppi41_queue_mgr);

struct cppi41_dma_block cppi41_dma_block[CPPI41_NUM_DMA_BLOCK];
EXPORT_SYMBOL(cppi41_dma_block);
/******************** CPPI 4.1 Functions (External Interface) *****************/

int cppi41_queue_mgr_init(u8 q_mgr, dma_addr_t rgn0_base, u16 rgn0_size)
{
	void __iomem *q_mgr_regs;
	void *ptr;

	if (q_mgr >= cppi41_num_queue_mgr)
		return -EINVAL;

	q_mgr_regs = cppi41_queue_mgr[q_mgr].q_mgr_rgn_base;
	ptr = dma_alloc_coherent(NULL, rgn0_size * 4,
				 &linking_ram[q_mgr].phys_addr,
				 GFP_KERNEL | GFP_DMA);
	if (ptr == NULL) {
		printk(KERN_ERR "ERROR: %s: Unable to allocate "
		       "linking RAM.\n", __func__);
		return -ENOMEM;
	}
	linking_ram[q_mgr].virt_addr = ptr;
	linking_ram[q_mgr].size = rgn0_size * 4;

	cppi_writel(linking_ram[q_mgr].phys_addr,
			q_mgr_regs + QMGR_LINKING_RAM_RGN0_BASE_REG);
	DBG("Linking RAM region 0 base @ %p, value: %x\n",
	    q_mgr_regs + QMGR_LINKING_RAM_RGN0_BASE_REG,
	    cppi_readl(q_mgr_regs + QMGR_LINKING_RAM_RGN0_BASE_REG));

	cppi_writel(rgn0_size, q_mgr_regs + QMGR_LINKING_RAM_RGN0_SIZE_REG);
	DBG("Linking RAM region 0 size @ %p, value: %x\n",
	    q_mgr_regs + QMGR_LINKING_RAM_RGN0_SIZE_REG,
	    cppi_readl(q_mgr_regs + QMGR_LINKING_RAM_RGN0_SIZE_REG));

	ptr = kzalloc(BITS_TO_LONGS(cppi41_queue_mgr[q_mgr].num_queue),
		      GFP_KERNEL);
	if (ptr == NULL) {
		printk(KERN_ERR "ERROR: %s: Unable to allocate queue bitmap.\n",
		       __func__);
		dma_free_coherent(NULL, rgn0_size * 4,
				  linking_ram[q_mgr].virt_addr,
				  linking_ram[q_mgr].phys_addr);
		return -ENOMEM;
	}
	allocated_queues[q_mgr] = ptr;

	return 0;
}
EXPORT_SYMBOL(cppi41_queue_mgr_init);

int cppi41_queue_mgr_uninit(u8 q_mgr)
{
	void __iomem *q_mgr_regs;

	if (q_mgr >= cppi41_num_queue_mgr)
		return -EINVAL;

	q_mgr_regs = cppi41_queue_mgr[q_mgr].q_mgr_rgn_base;

	/* free the Queue Mgr linking ram space */
	cppi_writel(0,	q_mgr_regs + QMGR_LINKING_RAM_RGN0_BASE_REG);
	cppi_writel(0, q_mgr_regs + QMGR_LINKING_RAM_RGN0_SIZE_REG);
	dma_free_coherent(NULL, linking_ram[q_mgr].size,
			linking_ram[q_mgr].virt_addr,
			linking_ram[q_mgr].phys_addr);

	/* free the allocated queues */
	kfree(allocated_queues[q_mgr]);
	return 0;
}
EXPORT_SYMBOL(cppi41_queue_mgr_uninit);

int cppi41_dma_sched_tbl_init(u8 dma_num, u8 q_mgr,
			u32 *sched_tbl, u8 tbl_size)
{
	struct cppi41_dma_block *dma_block;
	int num_reg, k, i, val = 0;

	dma_block = (struct cppi41_dma_block *)&cppi41_dma_block[dma_num];

	num_reg = (tbl_size + 3) / 4;
	for (k = i = 0; i < num_reg; i++) {
#if 0
		for (val = j = 0; j < 4; j++, k++) {
			val >>= 8;
			if (k < tbl_size)
				val |= sched_tbl[k] << 24;
		}
#endif
		val = sched_tbl[i];
		cppi_writel(val, dma_block->sched_table_base +
			DMA_SCHED_TABLE_WORD_REG(i));
		DBG("DMA scheduler table @ %p, value written: %x\n",
		dma_block->sched_table_base + DMA_SCHED_TABLE_WORD_REG(i),
			val);
	}
	return 0;
}
EXPORT_SYMBOL(cppi41_dma_sched_tbl_init);

int cppi41_schedtbl_add_dma_ch(u8 dmanum, u8 qmgr, u8 dma_ch, u8 is_tx)
{
	struct cppi41_dma_block *dma_block;
	int num_ch, i, tbl_index = 0, j = 0, found = 0;
	u32 val;

	dma_block = (struct cppi41_dma_block *)&cppi41_dma_block[dmanum];

	val = 0;
	for (num_ch = 0, i = 0; i < MAX_SCHED_TBL_ENTRY; i++) {
		if (!found && dma_sched_tbl[i].dma_ch == dma_ch &&
			dma_sched_tbl[i].is_tx == is_tx &&
			dma_sched_tbl[i].enb == 0) {
			dma_sched_tbl[i].enb = 1;
			found = 1;
		}

		if (dma_sched_tbl[i].enb) {
			val |= ((dma_sched_tbl[i].dma_ch |
				(dma_sched_tbl[i].is_tx ? 0 : (1<<7))) << j*8);
			num_ch++;
			j++;
		}
		if (num_ch % 4 == 0) {
			cppi_writel(val, dma_block->sched_table_base +
				DMA_SCHED_TABLE_WORD_REG(tbl_index));
			tbl_index++;
			val = j = 0;
		}
	}

	if (num_ch % 4) {
		cppi_writel(val, dma_block->sched_table_base +
			DMA_SCHED_TABLE_WORD_REG(tbl_index));
	}
	return num_ch;
}
EXPORT_SYMBOL(cppi41_schedtbl_add_dma_ch);

int cppi41_schedtbl_remove_dma_ch(u8 dmanum, u8 qmgr, u8 dma_ch, u8 is_tx)
{
	struct cppi41_dma_block *dma_block;
	int num_ch, i, tbl_index = 0, j = 0, found = 0;
	u32 val;

	dma_block = (struct cppi41_dma_block *)&cppi41_dma_block[dmanum];

	val = 0;
	for (num_ch = 0, i = 0; i < MAX_SCHED_TBL_ENTRY; i++) {
		if (!found && dma_sched_tbl[i].dma_ch == dma_ch &&
			dma_sched_tbl[i].is_tx == is_tx &&
			dma_sched_tbl[i].enb == 1) {
			dma_sched_tbl[i].enb = 0;
		}

		if (dma_sched_tbl[i].enb) {
			val |= ((dma_sched_tbl[i].dma_ch |
				(dma_sched_tbl[i].is_tx ? 0 : (1<<7))) << j*8);
			num_ch++;
			j++;
		}
		if (num_ch % 4 == 0) {
			cppi_writel(val, dma_block->sched_table_base +
				DMA_SCHED_TABLE_WORD_REG(tbl_index));
			tbl_index++;
			val = j = 0;
		}
	}

	if (num_ch % 4) {
		cppi_writel(val, dma_block->sched_table_base +
			DMA_SCHED_TABLE_WORD_REG(tbl_index));
	}
	return num_ch;
}
EXPORT_SYMBOL(cppi41_schedtbl_remove_dma_ch);

int cppi41_dma_block_init(u8 dma_num, u8 q_mgr, u8 num_order,
				 u32 *sched_tbl, u8 tbl_size)
{
	const struct cppi41_dma_block *dma_block;
	unsigned num_desc, num_reg;
	void *ptr;
	int error, i;
	u16 q_num;
	u32 val;

	if (dma_num >= cppi41_num_dma_block ||
	    q_mgr >= cppi41_num_queue_mgr ||
	    !tbl_size || sched_tbl == NULL)
		return -EINVAL;

	error = cppi41_queue_alloc(CPPI41_FREE_DESC_QUEUE |
				   CPPI41_UNASSIGNED_QUEUE, q_mgr, &q_num);
	if (error) {
		printk(KERN_ERR "ERROR: %s: Unable to allocate teardown "
		       "descriptor queue.\n", __func__);
		return error;
	}
	DBG("Teardown descriptor queue %d in queue manager 0 "
	    "allocated\n", q_num);

	/*
	 * Tell the hardware about the Teardown descriptor
	 * queue manager and queue number.
	 */
	dma_block = &cppi41_dma_block[dma_num];
	cppi_writel((q_mgr << DMA_TD_DESC_QMGR_SHIFT) |
		     (q_num << DMA_TD_DESC_QNUM_SHIFT),
		     dma_block->global_ctrl_base +
		     DMA_TEARDOWN_FREE_DESC_CTRL_REG);
	DBG("Teardown free descriptor control @ %p, value: %x\n",
	    dma_block->global_ctrl_base + DMA_TEARDOWN_FREE_DESC_CTRL_REG,
	    cppi_readl(dma_block->global_ctrl_base +
			DMA_TEARDOWN_FREE_DESC_CTRL_REG));

	num_desc = 1 << num_order;
	dma_teardown[dma_num].rgn_size = num_desc *
					 sizeof(struct cppi41_teardown_desc);

	/* Pre-allocate teardown descriptors. */
	ptr = dma_alloc_coherent(NULL, dma_teardown[dma_num].rgn_size,
				 &dma_teardown[dma_num].phys_addr,
				 GFP_KERNEL | GFP_DMA);
	if (ptr == NULL) {
		printk(KERN_ERR "ERROR: %s: Unable to allocate teardown "
		       "descriptors.\n", __func__);
		error = -ENOMEM;
		goto free_queue;
	}
	dma_teardown[dma_num].virt_addr = ptr;

	error = cppi41_mem_rgn_alloc(q_mgr, dma_teardown[dma_num].phys_addr, 5,
				     num_order, &dma_teardown[dma_num].mem_rgn);
	if (error) {
		printk(KERN_ERR "ERROR: %s: Unable to allocate queue manager "
		       "memory region for teardown descriptors.\n", __func__);
		goto free_mem;
	}

	error = cppi41_queue_init(&dma_teardown[dma_num].queue_obj, 0, q_num);
	if (error) {
		printk(KERN_ERR "ERROR: %s: Unable to initialize teardown "
		       "free descriptor queue.\n", __func__);
		goto free_rgn;
	}

	dma_teardown[dma_num].q_num = q_num;
	dma_teardown[dma_num].q_mgr = q_mgr;
	dma_teardown[dma_num].num_desc = num_desc;
	/*
	 * Push all teardown descriptors to the free teardown queue
	 * for the CPPI 4.1 system.
	 */
	cppi41_init_teardown_queue(dma_num);

	/* Initialize the DMA scheduler. */
	num_reg = (tbl_size + 3) / 4;
	for (i = 0; i < num_reg; i++) {
		val = sched_tbl[i];
		cppi_writel(val, dma_block->sched_table_base +
			     DMA_SCHED_TABLE_WORD_REG(i));
		DBG("DMA scheduler table @ %p, value written: %x\n",
		    dma_block->sched_table_base + DMA_SCHED_TABLE_WORD_REG(i),
		    val);
	}

	cppi_writel((tbl_size - 1) << DMA_SCHED_LAST_ENTRY_SHIFT |
		     DMA_SCHED_ENABLE_MASK,
		     dma_block->sched_ctrl_base + DMA_SCHED_CTRL_REG);
	DBG("DMA scheduler control @ %p, value: %x\n",
	    dma_block->sched_ctrl_base + DMA_SCHED_CTRL_REG,
	    cppi_readl(dma_block->sched_ctrl_base + DMA_SCHED_CTRL_REG));

	return 0;

free_rgn:
	cppi41_mem_rgn_free(q_mgr, dma_teardown[dma_num].mem_rgn);
free_mem:
	dma_free_coherent(NULL, dma_teardown[dma_num].rgn_size,
			  dma_teardown[dma_num].virt_addr,
			  dma_teardown[dma_num].phys_addr);
free_queue:
	cppi41_queue_free(q_mgr, q_num);
	return error;
}
EXPORT_SYMBOL(cppi41_dma_block_init);

int cppi41_dma_block_uninit(u8 dma_num, u8 q_mgr, u8 num_order,
				 u32 *sched_tbl, u8 tbl_size)
{
	const struct cppi41_dma_block *dma_block;
	unsigned num_reg;
	int i;

	/* popout all teardown descriptors */
	cppi41_free_teardown_queue(dma_num);

	/* free queue mgr region */
	cppi41_mem_rgn_free(q_mgr, dma_teardown[dma_num].mem_rgn);
	/* free the allocated teardown descriptors */
	dma_free_coherent(NULL, dma_teardown[dma_num].rgn_size,
			dma_teardown[dma_num].virt_addr,
			dma_teardown[dma_num].phys_addr);

	/* free the teardown queue*/
	cppi41_queue_free(dma_teardown[dma_num].q_mgr,
			dma_teardown[dma_num].q_num);

	dma_block = (struct cppi41_dma_block *)&cppi41_dma_block[dma_num];
	/* disable the dma schedular */
	num_reg = (tbl_size + 3) / 4;
	for (i = 0; i < num_reg; i++) {
		cppi_writel(0, dma_block->sched_table_base +
			     DMA_SCHED_TABLE_WORD_REG(i));
		DBG("DMA scheduler table @ %p, value written: %x\n",
		    dma_block->sched_table_base + DMA_SCHED_TABLE_WORD_REG(i),
		    0);
	}

	cppi_writel(0,	dma_block->sched_ctrl_base + DMA_SCHED_CTRL_REG);

	return 0;
}
EXPORT_SYMBOL(cppi41_dma_block_uninit);
/*
 * cppi41_mem_rgn_alloc - allocate a memory region within the queue manager
 */
int cppi41_mem_rgn_alloc(u8 q_mgr, dma_addr_t rgn_addr, u8 size_order,
			 u8 num_order, u8 *mem_rgn)
{
	void __iomem *desc_mem_regs;
	u32 num_desc = 1 << num_order, index, ctrl;
	int rgn;

	DBG("%s called with rgn_addr = %08x, size_order = %d, num_order = %d\n",
	    __func__, rgn_addr, size_order, num_order);

	if (q_mgr >= cppi41_num_queue_mgr ||
	    size_order < 5 || size_order > 13 ||
	    num_order  < 5 || num_order  > 12 ||
	    (rgn_addr & ((1 << size_order) - 1)))
		return -EINVAL;

	rgn = next_mem_rgn[q_mgr];
	index = next_desc_index[q_mgr];
	if (rgn >= CPPI41_MAX_MEM_RGN || index + num_desc > 0x4000)
		return -ENOSPC;

	next_mem_rgn[q_mgr] = rgn + 1;
	next_desc_index[q_mgr] = index + num_desc;

	desc_mem_regs = cppi41_queue_mgr[q_mgr].desc_mem_rgn_base;

	/* Write the base register */
	cppi_writel(rgn_addr, desc_mem_regs + QMGR_MEM_RGN_BASE_REG(rgn));
	DBG("Descriptor region base @ %p, value: %x\n",
	    desc_mem_regs + QMGR_MEM_RGN_BASE_REG(rgn),
	    cppi_readl(desc_mem_regs + QMGR_MEM_RGN_BASE_REG(rgn)));

	/* Write the control register */
	ctrl = ((index << QMGR_MEM_RGN_INDEX_SHIFT) &
		QMGR_MEM_RGN_INDEX_MASK) |
	       (((size_order - 5) << QMGR_MEM_RGN_DESC_SIZE_SHIFT) &
		QMGR_MEM_RGN_DESC_SIZE_MASK) |
	       (((num_order - 5) << QMGR_MEM_RGN_SIZE_SHIFT) &
		QMGR_MEM_RGN_SIZE_MASK);
	cppi_writel(ctrl, desc_mem_regs + QMGR_MEM_RGN_CTRL_REG(rgn));
	DBG("Descriptor region control @ %p, value: %x\n",
	    desc_mem_regs + QMGR_MEM_RGN_CTRL_REG(rgn),
	    cppi_readl(desc_mem_regs + QMGR_MEM_RGN_CTRL_REG(rgn)));

	*mem_rgn = rgn;
	return 0;
}
EXPORT_SYMBOL(cppi41_mem_rgn_alloc);

/*
 * cppi41_mem_rgn_free - free the memory region within the queue manager
 */
int cppi41_mem_rgn_free(u8 q_mgr, u8 mem_rgn)
{
	void __iomem *desc_mem_regs;

	DBG("%s called.\n", __func__);

	if (q_mgr >= cppi41_num_queue_mgr || mem_rgn >= next_mem_rgn[q_mgr])
		return -EINVAL;

	desc_mem_regs = cppi41_queue_mgr[q_mgr].desc_mem_rgn_base;

	if (cppi_readl(desc_mem_regs + QMGR_MEM_RGN_BASE_REG(mem_rgn)) == 0)
		return -ENOENT;

	cppi_writel(0, desc_mem_regs + QMGR_MEM_RGN_BASE_REG(mem_rgn));
	cppi_writel(0, desc_mem_regs + QMGR_MEM_RGN_CTRL_REG(mem_rgn));

	return 0;
}
EXPORT_SYMBOL(cppi41_mem_rgn_free);

/*
 * cppi41_tx_ch_init - initialize a CPPI 4.1 Tx channel object
 *
 * Verify the channel info (range checking, etc.) and store the channel
 * information within the object structure.
 */
int cppi41_tx_ch_init(struct cppi41_dma_ch_obj *tx_ch_obj,
		      u8 dma_num, u8 ch_num)
{
	if (dma_num >= cppi41_num_dma_block ||
	    ch_num  >= cppi41_dma_block[dma_num].num_tx_ch)
		return -EINVAL;

	/* Populate the channel object structure */
	tx_ch_obj->base_addr  = cppi41_dma_block[dma_num].ch_ctrl_stat_base +
				DMA_CH_TX_GLOBAL_CFG_REG(ch_num);
	tx_ch_obj->global_cfg = cppi_readl(tx_ch_obj->base_addr);
	return 0;
}
EXPORT_SYMBOL(cppi41_tx_ch_init);

/*
 * cppi41_rx_ch_init - initialize a CPPI 4.1 Rx channel object
 *
 * Verify the channel info (range checking, etc.) and store the channel
 * information within the object structure.
 */
int cppi41_rx_ch_init(struct cppi41_dma_ch_obj *rx_ch_obj,
		      u8 dma_num, u8 ch_num)
{
	if (dma_num >= cppi41_num_dma_block ||
	    ch_num  >= cppi41_dma_block[dma_num].num_rx_ch)
		return -EINVAL;

	/* Populate the channel object structure */
	rx_ch_obj->base_addr  = cppi41_dma_block[dma_num].ch_ctrl_stat_base +
				DMA_CH_RX_GLOBAL_CFG_REG(ch_num);
	rx_ch_obj->global_cfg = cppi_readl(rx_ch_obj->base_addr);
	return 0;
}
EXPORT_SYMBOL(cppi41_rx_ch_init);

/*
 * We have to cache the last written Rx/Tx channel global configration register
 * value due to its bits other than enable/teardown being write-only. Yet there
 * is a caveat related to caching the enable bit: this bit may be automatically
 * cleared as a result of teardown, so we can't trust its cached value!
 * When modifying the write only register fields, we're making use of the fact
 * that they read back as zeros, and not clearing them explicitly...
 */

/*
 * cppi41_dma_ch_default_queue - set CPPI 4.1 channel default completion queue
 */
void cppi41_dma_ch_default_queue(struct cppi41_dma_ch_obj *dma_ch_obj,
				 u8 q_mgr, u16 q_num)
{
	u32 val = dma_ch_obj->global_cfg;

	/* Clear the fields to be modified. */
	val &= ~(DMA_CH_TX_DEFAULT_QMGR_MASK | DMA_CH_TX_DEFAULT_QNUM_MASK |
		 DMA_CH_TX_ENABLE_MASK);

	/* Set the default completion queue. */
	val |= ((q_mgr << DMA_CH_TX_DEFAULT_QMGR_SHIFT) &
		DMA_CH_TX_DEFAULT_QMGR_MASK) |
	       ((q_num << DMA_CH_TX_DEFAULT_QNUM_SHIFT) &
		DMA_CH_TX_DEFAULT_QNUM_MASK);

	/* Get the current state of the enable bit. */
	dma_ch_obj->global_cfg = val |= cppi_readl(dma_ch_obj->base_addr);
	cppi_writel(val, dma_ch_obj->base_addr);
	DBG("Channel global configuration @ %p, value written: %x, "
	    "value read: %x\n", dma_ch_obj->base_addr, val,
	    cppi_readl(dma_ch_obj->base_addr));

}
EXPORT_SYMBOL(cppi41_dma_ch_default_queue);

/*
 * cppi41_rx_ch_configure - configure CPPI 4.1 Rx channel
 */
void cppi41_rx_ch_configure(struct cppi41_dma_ch_obj *rx_ch_obj,
			    struct cppi41_rx_ch_cfg  *cfg)
{
	void __iomem *base = rx_ch_obj->base_addr;
	u32 val = cppi_readl(rx_ch_obj->base_addr);

	val |= ((cfg->sop_offset << DMA_CH_RX_SOP_OFFSET_SHIFT) &
		DMA_CH_RX_SOP_OFFSET_MASK) |
	       ((cfg->default_desc_type << DMA_CH_RX_DEFAULT_DESC_TYPE_SHIFT) &
		DMA_CH_RX_DEFAULT_DESC_TYPE_MASK) |
	       ((cfg->retry_starved << DMA_CH_RX_ERROR_HANDLING_SHIFT) &
		DMA_CH_RX_ERROR_HANDLING_MASK) |
	       ((cfg->rx_queue.q_mgr << DMA_CH_RX_DEFAULT_RQ_QMGR_SHIFT) &
		DMA_CH_RX_DEFAULT_RQ_QMGR_MASK) |
	       ((cfg->rx_queue.q_num << DMA_CH_RX_DEFAULT_RQ_QNUM_SHIFT) &
		DMA_CH_RX_DEFAULT_RQ_QNUM_MASK);

	val &= ~(0x7 << DMA_CH_RX_MAX_BUF_CNT_SHIFT);
	val |= (cfg->rx_max_buf_cnt << DMA_CH_RX_MAX_BUF_CNT_SHIFT);

	rx_ch_obj->global_cfg = val;
	cppi_writel(val, base);
	DBG("Rx channel global configuration @ %p, value written: %x, "
	    "value read: %x\n", base, val, cppi_readl(base));

	base -= DMA_CH_RX_GLOBAL_CFG_REG(0);

	/*
	 * Set up the packet configuration register
	 * based on the descriptor type...
	 */
	switch (cfg->default_desc_type) {
	case DMA_CH_RX_DEFAULT_DESC_EMBED:
		val = ((cfg->cfg.embed_pkt.fd_queue.q_mgr <<
			DMA_CH_RX_EMBED_FDQ_QMGR_SHIFT) &
		       DMA_CH_RX_EMBED_FDQ_QMGR_MASK) |
		      ((cfg->cfg.embed_pkt.fd_queue.q_num <<
			DMA_CH_RX_EMBED_FDQ_QNUM_SHIFT) &
		       DMA_CH_RX_EMBED_FDQ_QNUM_MASK) |
		      ((cfg->cfg.embed_pkt.num_buf_slot <<
			DMA_CH_RX_EMBED_NUM_SLOT_SHIFT) &
		       DMA_CH_RX_EMBED_NUM_SLOT_MASK) |
		      ((cfg->cfg.embed_pkt.sop_slot_num <<
			DMA_CH_RX_EMBED_SOP_SLOT_SHIFT) &
		       DMA_CH_RX_EMBED_SOP_SLOT_MASK);

		cppi_writel(val, base + DMA_CH_RX_EMBED_PKT_CFG_REG_B(0));
		DBG("Rx channel embedded packet configuration B @ %p, "
		    "value written: %x\n",
		    base + DMA_CH_RX_EMBED_PKT_CFG_REG_B(0), val);

		val = ((cfg->cfg.embed_pkt.free_buf_pool[0].b_pool <<
			DMA_CH_RX_EMBED_FBP_PNUM_SHIFT(0)) &
		       DMA_CH_RX_EMBED_FBP_PNUM_MASK(0)) |
		      ((cfg->cfg.embed_pkt.free_buf_pool[0].b_mgr <<
			DMA_CH_RX_EMBED_FBP_BMGR_SHIFT(0)) &
		       DMA_CH_RX_EMBED_FBP_BMGR_MASK(0)) |
		      ((cfg->cfg.embed_pkt.free_buf_pool[1].b_pool <<
			DMA_CH_RX_EMBED_FBP_PNUM_SHIFT(1)) &
		       DMA_CH_RX_EMBED_FBP_PNUM_MASK(1)) |
		      ((cfg->cfg.embed_pkt.free_buf_pool[1].b_mgr <<
			DMA_CH_RX_EMBED_FBP_BMGR_SHIFT(1)) &
		       DMA_CH_RX_EMBED_FBP_BMGR_MASK(1)) |
		      ((cfg->cfg.embed_pkt.free_buf_pool[2].b_pool <<
			DMA_CH_RX_EMBED_FBP_PNUM_SHIFT(2)) &
		       DMA_CH_RX_EMBED_FBP_PNUM_MASK(2)) |
		      ((cfg->cfg.embed_pkt.free_buf_pool[2].b_mgr <<
			DMA_CH_RX_EMBED_FBP_BMGR_SHIFT(2)) &
		       DMA_CH_RX_EMBED_FBP_BMGR_MASK(2)) |
		      ((cfg->cfg.embed_pkt.free_buf_pool[3].b_pool <<
			DMA_CH_RX_EMBED_FBP_PNUM_SHIFT(3)) &
		       DMA_CH_RX_EMBED_FBP_PNUM_MASK(3)) |
		      ((cfg->cfg.embed_pkt.free_buf_pool[3].b_mgr <<
			DMA_CH_RX_EMBED_FBP_BMGR_SHIFT(3)) &
		       DMA_CH_RX_EMBED_FBP_BMGR_MASK(3));

		cppi_writel(val, base + DMA_CH_RX_EMBED_PKT_CFG_REG_A(0));
		DBG("Rx channel embedded packet configuration A @ %p, "
		    "value written: %x\n",
		    base + DMA_CH_RX_EMBED_PKT_CFG_REG_A(0), val);
		break;
	case DMA_CH_RX_DEFAULT_DESC_HOST:
		val = ((cfg->cfg.host_pkt.fdb_queue[0].q_num <<
			DMA_CH_RX_HOST_FDQ_QNUM_SHIFT(0)) &
		       DMA_CH_RX_HOST_FDQ_QNUM_MASK(0)) |
		      ((cfg->cfg.host_pkt.fdb_queue[0].q_mgr <<
			DMA_CH_RX_HOST_FDQ_QMGR_SHIFT(0)) &
		       DMA_CH_RX_HOST_FDQ_QMGR_MASK(0)) |
		      ((cfg->cfg.host_pkt.fdb_queue[1].q_num <<
			DMA_CH_RX_HOST_FDQ_QNUM_SHIFT(1)) &
		       DMA_CH_RX_HOST_FDQ_QNUM_MASK(1)) |
		      ((cfg->cfg.host_pkt.fdb_queue[1].q_mgr <<
			DMA_CH_RX_HOST_FDQ_QMGR_SHIFT(1)) &
		       DMA_CH_RX_HOST_FDQ_QMGR_MASK(1));

		cppi_writel(val, base + DMA_CH_RX_HOST_PKT_CFG_REG_A(0));
		DBG("Rx channel host packet configuration A @ %p, "
		    "value written: %x\n",
		    base + DMA_CH_RX_HOST_PKT_CFG_REG_A(0), val);

		val = ((cfg->cfg.host_pkt.fdb_queue[2].q_num <<
			DMA_CH_RX_HOST_FDQ_QNUM_SHIFT(2)) &
		       DMA_CH_RX_HOST_FDQ_QNUM_MASK(2)) |
		      ((cfg->cfg.host_pkt.fdb_queue[2].q_mgr <<
			DMA_CH_RX_HOST_FDQ_QMGR_SHIFT(2)) &
		       DMA_CH_RX_HOST_FDQ_QMGR_MASK(2)) |
		      ((cfg->cfg.host_pkt.fdb_queue[3].q_num <<
		       DMA_CH_RX_HOST_FDQ_QNUM_SHIFT(3)) &
		       DMA_CH_RX_HOST_FDQ_QNUM_MASK(3)) |
		      ((cfg->cfg.host_pkt.fdb_queue[3].q_mgr <<
			DMA_CH_RX_HOST_FDQ_QMGR_SHIFT(3)) &
		       DMA_CH_RX_HOST_FDQ_QMGR_MASK(3));

		cppi_writel(val, base + DMA_CH_RX_HOST_PKT_CFG_REG_B(0));
		DBG("Rx channel host packet configuration B @ %p, "
		    "value written: %x\n",
		    base + DMA_CH_RX_HOST_PKT_CFG_REG_B(0), val);
		break;
	case DMA_CH_RX_DEFAULT_DESC_MONO:
		val = ((cfg->cfg.mono_pkt.fd_queue.q_num <<
			DMA_CH_RX_MONO_FDQ_QNUM_SHIFT) &
		       DMA_CH_RX_MONO_FDQ_QNUM_MASK) |
		      ((cfg->cfg.mono_pkt.fd_queue.q_mgr <<
			DMA_CH_RX_MONO_FDQ_QMGR_SHIFT) &
		       DMA_CH_RX_MONO_FDQ_QMGR_MASK) |
		      ((cfg->cfg.mono_pkt.sop_offset <<
			DMA_CH_RX_MONO_SOP_OFFSET_SHIFT) &
		       DMA_CH_RX_MONO_SOP_OFFSET_MASK);

		cppi_writel(val, base + DMA_CH_RX_MONO_PKT_CFG_REG(0));
		DBG("Rx channel monolithic packet configuration @ %p, "
		    "value written: %x\n",
		    base + DMA_CH_RX_MONO_PKT_CFG_REG(0), val);
		break;
	}
}
EXPORT_SYMBOL(cppi41_rx_ch_configure);

void cppi41_rx_ch_set_maxbufcnt(struct cppi41_dma_ch_obj *rx_ch_obj,
			    u8 rx_max_buf_cnt)
{
	void __iomem *base = rx_ch_obj->base_addr;
	u32 val = cppi_readl(rx_ch_obj->base_addr);

	val = rx_ch_obj->global_cfg;
	val &= ~(0x7 << DMA_CH_RX_MAX_BUF_CNT_SHIFT);
	val |= (rx_max_buf_cnt << DMA_CH_RX_MAX_BUF_CNT_SHIFT);

	rx_ch_obj->global_cfg = val;
	cppi_writel(val, base);

	DBG("%s: rx-global-cfg @ %p, value written: %x, "
	    "value read: %x\n", __func__, base, val, cppi_readl(base));

}
EXPORT_SYMBOL(cppi41_rx_ch_set_maxbufcnt);
/*
 * cppi41_dma_ch_teardown - teardown a given Tx/Rx channel
 */
void cppi41_dma_ch_teardown(struct cppi41_dma_ch_obj *dma_ch_obj)
{
	u32 val = cppi_readl(dma_ch_obj->base_addr);

	/* Initiate channel teardown. */
	val |= dma_ch_obj->global_cfg & ~DMA_CH_TX_ENABLE_MASK;
	dma_ch_obj->global_cfg = val |= DMA_CH_TX_TEARDOWN_MASK;
	cppi_writel(val, dma_ch_obj->base_addr);
	DBG("Tear down channel @ %p, value written: %x, value read: %x\n",
	    dma_ch_obj->base_addr, val, cppi_readl(dma_ch_obj->base_addr));
}
EXPORT_SYMBOL(cppi41_dma_ch_teardown);

/*
 * cppi41_dma_ch_enable - enable Tx/Rx DMA channel in hardware
 *
 * Makes the channel ready for data transmission/reception.
 */
void cppi41_dma_ch_enable(struct cppi41_dma_ch_obj *dma_ch_obj)
{
	u32 val = dma_ch_obj->global_cfg | DMA_CH_TX_ENABLE_MASK;

	/* Teardown bit remains set after completion, so clear it now... */
	dma_ch_obj->global_cfg = val &= ~DMA_CH_TX_TEARDOWN_MASK;
	cppi_writel(val, dma_ch_obj->base_addr);
	DBG("Enable channel @ %p, value written: %x, value read: %x\n",
	    dma_ch_obj->base_addr, val, cppi_readl(dma_ch_obj->base_addr));
}
EXPORT_SYMBOL(cppi41_dma_ch_enable);

/*
 * cppi41_dma_ch_disable - disable Tx/Rx DMA channel in hardware
 */
void cppi41_dma_ch_disable(struct cppi41_dma_ch_obj *dma_ch_obj)
{
	dma_ch_obj->global_cfg &= ~DMA_CH_TX_ENABLE_MASK;
	cppi_writel(dma_ch_obj->global_cfg, dma_ch_obj->base_addr);
	DBG("Disable channel @ %p, value written: %x, value read: %x\n",
	    dma_ch_obj->base_addr, dma_ch_obj->global_cfg,
	    cppi_readl(dma_ch_obj->base_addr));
}
EXPORT_SYMBOL(cppi41_dma_ch_disable);

void cppi41_init_teardown_queue(int dma_num)
{
	dma_addr_t td_addr;
	struct cppi41_teardown_desc *curr_td;
	u32 num_desc = dma_teardown[dma_num].num_desc;
	int i;

	curr_td = dma_teardown[dma_num].virt_addr;
	td_addr = dma_teardown[dma_num].phys_addr;

	for (i = 0; i < num_desc; i++) {
		cppi41_queue_push(&dma_teardown[dma_num].queue_obj, td_addr,
				  sizeof(*curr_td), 0);
		td_addr += sizeof(*curr_td);
	}
}
EXPORT_SYMBOL(cppi41_init_teardown_queue);

void cppi41_free_teardown_queue(int dma_num)
{
	unsigned long td_addr;
	u32 num_desc = dma_teardown[dma_num].num_desc;

	while (num_desc--) {
		td_addr = cppi41_queue_pop(&dma_teardown[dma_num].queue_obj);

		if (td_addr == 0)
			break;
	}
}
EXPORT_SYMBOL(cppi41_free_teardown_queue);

/**
 * alloc_queue - allocate a queue in the given range
 * @allocated:	pointer to the bitmap of the allocated queues
 * @excluded:	pointer to the bitmap of the queues excluded from allocation
 *		(optional)
 * @start:	starting queue number
 * @count:	number of queues available
 *
 * Returns queue number on success, -ENOSPC otherwise.
 */
static int alloc_queue(u32 *allocated, const u32 *excluded, unsigned start,
		       unsigned count)
{
	u32 bit, mask = 0;
	int index = -1;

	/*
	 * We're starting the loop as if we've just wrapped around 32 bits
	 * in order to save on preloading the bitmasks.
	 */
	for (bit = 0; count--; start++, bit <<= 1) {
		/* Have we just wrapped around 32 bits? */
		if (!bit) {
			/* Start over with the next bitmask word */
			bit = 1;
			index++;
			/* Have we just entered the loop? */
			if (!index) {
				/* Calculate the starting values */
				bit <<= start & 0x1f;
				index = start >> 5;
			}
			/*
			 * Load the next word of the allocated bitmask OR'ing
			 * it with the excluded bitmask if it's been passed.
			 */
			mask = allocated[index];
			if (excluded != NULL)
				mask |= excluded[index];
		}
		/*
		 * If the bit in the combined bitmask is zero,
		 * we've just found a free queue.
		 */
		if (!(mask & bit)) {
			allocated[index] |= bit;
			return start;
		}
	}
	return -ENOSPC;
}

/*
 * cppi41_queue_alloc - allocate a queue of a given type in the queue manager
 */
int cppi41_queue_alloc(u8 type, u8 q_mgr, u16 *q_num)
{
	int res = -ENOSPC;

	if (q_mgr >= cppi41_num_queue_mgr)
		return -EINVAL;

	/* Mask out the unsupported queue types */
	type &= cppi41_queue_mgr[q_mgr].queue_types;
	/* First see if a free descriptor queue was requested... */
	if (type & CPPI41_FREE_DESC_QUEUE)
		res = alloc_queue(allocated_queues[q_mgr], NULL,
				  cppi41_queue_mgr[q_mgr].base_fdq_num,  16);

	/* Then see if a free descriptor/buffer queue was requested... */
	if (res < 0 && (type & CPPI41_FREE_DESC_BUF_QUEUE))
		res = alloc_queue(allocated_queues[q_mgr], NULL,
				  cppi41_queue_mgr[q_mgr].base_fdbq_num, 16);

	/* Last see if an unassigned queue was requested... */
	if (res < 0 && (type & CPPI41_UNASSIGNED_QUEUE))
		res = alloc_queue(allocated_queues[q_mgr],
				  cppi41_queue_mgr[q_mgr].assigned, 0,
				  cppi41_queue_mgr[q_mgr].num_queue);

	/* See if any queue was allocated... */
	if (res < 0)
		return res;

	/* Return the queue allocated */
	*q_num = res;
	return 0;
}
EXPORT_SYMBOL(cppi41_queue_alloc);

/*
 * cppi41_queue_free - free the given queue in the queue manager
 */
int cppi41_queue_free(u8 q_mgr, u16 q_num)
{
	int index = q_num >> 5, bit = 1 << (q_num & 0x1f);

	if (allocated_queues[q_mgr] != NULL) {
		if (q_mgr >= cppi41_num_queue_mgr ||
		    q_num >= cppi41_queue_mgr[q_mgr].num_queue ||
		    !(allocated_queues[q_mgr][index] & bit))
			return -EINVAL;
		allocated_queues[q_mgr][index] &= ~bit;
	}
	return 0;
}
EXPORT_SYMBOL(cppi41_queue_free);

/*
 * cppi41_queue_init - initialize a CPPI 4.1 queue object
 */
int cppi41_queue_init(struct cppi41_queue_obj *queue_obj, u8 q_mgr, u16 q_num)
{
	if (q_mgr >= cppi41_num_queue_mgr ||
	    q_num >= cppi41_queue_mgr[q_mgr].num_queue)
		return -EINVAL;

	queue_obj->base_addr = cppi41_queue_mgr[q_mgr].q_mgmt_rgn_base +
			       QMGR_QUEUE_STATUS_REG_A(q_num);

	return 0;
}
EXPORT_SYMBOL(cppi41_queue_init);

/*
 * cppi41_queue_push - push a descriptor into the given queue
 */
void cppi41_queue_push(const struct cppi41_queue_obj *queue_obj, u32 desc_addr,
		       u32 desc_size, u32 pkt_size)
{
	u32 val;

	/*
	 * Write to the tail of the queue.
	 * TODO: Can't think of a reason why a queue to head may be required.
	 * If it is, the API may have to be extended.
	 */
#if 0
	/*
	 * Also, can't understand why packet size is required to queue up a
	 * descriptor. The spec says packet size *must* be written prior to
	 * the packet write operation.
	 */
	if (pkt_size)
		val = (pkt_size << QMGR_QUEUE_PKT_SIZE_SHIFT) &
		      QMGR_QUEUE_PKT_SIZE_MASK;
	cppi_writel(val, queue_obj->base_addr + QMGR_QUEUE_REG_C(0));
#endif

	val = (((desc_size - 24) >> (2 - QMGR_QUEUE_DESC_SIZE_SHIFT)) &
	       QMGR_QUEUE_DESC_SIZE_MASK) |
	      (desc_addr & QMGR_QUEUE_DESC_PTR_MASK);

	DBG("Pushing value %x to queue @ %p\n", val, queue_obj->base_addr);

	cppi_writel(val, queue_obj->base_addr + QMGR_QUEUE_REG_D(0));
}
EXPORT_SYMBOL(cppi41_queue_push);

/*
 * cppi41_queue_pop - pop a descriptor from a given queue
 */
unsigned long cppi41_queue_pop(const struct cppi41_queue_obj *queue_obj)
{
	u32 val = cppi_readl(queue_obj->base_addr + QMGR_QUEUE_REG_D(0));

	DBG("Popping value %x from queue @ %p\n", val, queue_obj->base_addr);

	return val & QMGR_QUEUE_DESC_PTR_MASK;
}
EXPORT_SYMBOL(cppi41_queue_pop);

/*
 * cppi41_get_teardown_info - extract information from a teardown descriptor
 */
int cppi41_get_teardown_info(unsigned long addr, u32 *info)
{
	struct cppi41_teardown_desc *desc;
	int dma_num;

	for (dma_num = 0; dma_num < cppi41_num_dma_block; dma_num++)
		if (addr >= dma_teardown[dma_num].phys_addr &&
		    addr <  dma_teardown[dma_num].phys_addr +
			    dma_teardown[dma_num].rgn_size)
			break;

	if (dma_num == cppi41_num_dma_block)
		return -EINVAL;

	desc = addr - dma_teardown[dma_num].phys_addr +
	       dma_teardown[dma_num].virt_addr;

	if ((desc->teardown_info & CPPI41_DESC_TYPE_MASK) !=
	    (CPPI41_DESC_TYPE_TEARDOWN << CPPI41_DESC_TYPE_SHIFT))
		return -EINVAL;

	*info = desc->teardown_info;
#if 1
	/* Hardware is not giving the current DMA number as of now. :-/ */
	*info |= (dma_num << CPPI41_TEARDOWN_DMA_NUM_SHIFT) &
		 CPPI41_TEARDOWN_DMA_NUM_MASK;
#else
	dma_num = (desc->teardown_info & CPPI41_TEARDOWN_DMA_NUM_MASK) >>
		 CPPI41_TEARDOWN_DMA_NUM_SHIFT;
#endif

	cppi41_queue_push(&dma_teardown[dma_num].queue_obj, addr,
			  sizeof(struct cppi41_teardown_desc), 0);

	return 0;
}
EXPORT_SYMBOL(cppi41_get_teardown_info);

/*
 * cppi41_save_context - save regsiter context before going to suspend.
 */
void cppi41_save_context(u8 dma_num)
{
	const struct cppi41_dma_block *dma_block;
	struct cppi41_dma_regs *cppi41;
	struct cppi41_queue_manager *qmgr;
	void __iomem *q_mgr_regs, *desc_mem_regs;
	u8 i, q_mgr = 0;

	dma_block = (struct cppi41_dma_block *)&cppi41_dma_block[dma_num];
	cppi41 = (struct cppi41_dma_regs *)&dma_block->cppi41_regs;
	qmgr = &cppi41->qmgr;
	q_mgr_regs = cppi41_queue_mgr[q_mgr].q_mgr_rgn_base;
	desc_mem_regs = cppi41_queue_mgr[q_mgr].desc_mem_rgn_base;

	/* popout all teardown descriptors */
	cppi41_free_teardown_queue(dma_num);

	cppi41->teardn_fdq_ctrl = cppi_readl(dma_block->global_ctrl_base +
			DMA_TEARDOWN_FREE_DESC_CTRL_REG);
	cppi41->emulation_ctrl = cppi_readl(dma_block->global_ctrl_base +
			DMA_EMULATION_CTRL_REG);

	qmgr->link_ram_rgn0_base = cppi_readl(q_mgr_regs +
				QMGR_LINKING_RAM_RGN0_BASE_REG);
	qmgr->link_ram_rgn0_size = cppi_readl(q_mgr_regs +
				QMGR_LINKING_RAM_RGN0_SIZE_REG);
	qmgr->link_ram_rgn1_base = cppi_readl(q_mgr_regs +
				QMGR_LINKING_RAM_RGN1_BASE_REG);

	for (i = 0 ; i < 8 ; i++) {
		qmgr->memr_base[i] = cppi_readl(desc_mem_regs +
				QMGR_MEM_RGN_BASE_REG(i));
		qmgr->memr_ctrl[i] = cppi_readl(desc_mem_regs +
				QMGR_MEM_RGN_CTRL_REG(i));
	}

	cppi41->sched_ctrl = cppi_readl(dma_block->sched_ctrl_base +
				DMA_SCHED_CTRL_REG);

}
EXPORT_SYMBOL(cppi41_save_context);

/*
 * cppi41_restore_context - restore regsiter context after resume.
 */
void cppi41_restore_context(u8 dma_num, u32 *sched_tbl)
{
	const struct cppi41_dma_block *dma_block;
	struct cppi41_dma_regs *cppi41;
	struct cppi41_queue_manager *qmgr;
	void __iomem *q_mgr_regs, *desc_mem_regs;
	unsigned num_reg;
	u32 val;
	u8 tbl_size;
	u8 i, q_mgr = 0;

	dma_block = (struct cppi41_dma_block *)&cppi41_dma_block[dma_num];
	cppi41 = (struct cppi41_dma_regs *)&dma_block->cppi41_regs;
	qmgr = &cppi41->qmgr;
	q_mgr_regs = cppi41_queue_mgr[q_mgr].q_mgr_rgn_base;
	desc_mem_regs = cppi41_queue_mgr[q_mgr].desc_mem_rgn_base;
	tbl_size = dma_block->num_max_ch;

	cppi_writel(cppi41->teardn_fdq_ctrl, dma_block->global_ctrl_base +
			DMA_TEARDOWN_FREE_DESC_CTRL_REG);
	cppi_writel(cppi41->emulation_ctrl, dma_block->global_ctrl_base +
			DMA_EMULATION_CTRL_REG);

	cppi_writel(qmgr->link_ram_rgn0_base, q_mgr_regs +
				QMGR_LINKING_RAM_RGN0_BASE_REG);
	cppi_writel(qmgr->link_ram_rgn0_size, q_mgr_regs +
				QMGR_LINKING_RAM_RGN0_SIZE_REG);
	cppi_writel(qmgr->link_ram_rgn1_base, q_mgr_regs +
				QMGR_LINKING_RAM_RGN1_BASE_REG);

	for (i = 0 ; i < 8 ; i++) {
		cppi_writel(qmgr->memr_base[i], desc_mem_regs +
				QMGR_MEM_RGN_BASE_REG(i));
		cppi_writel(qmgr->memr_ctrl[i], desc_mem_regs +
				QMGR_MEM_RGN_CTRL_REG(i));
	}

	/*
	 * Push all teardown descriptors to the free teardown queue
	 * for the CPPI 4.1 system.
	 */
	cppi41_init_teardown_queue(dma_num);

	/* Initialize the DMA scheduler. */
	num_reg = (tbl_size + 3) / 4;
	for (i = 0; i < num_reg; i++) {
		val = sched_tbl[i];
		cppi_writel(val, dma_block->sched_table_base +
			     DMA_SCHED_TABLE_WORD_REG(i));
	}
	cppi_writel(cppi41->sched_ctrl, dma_block->sched_ctrl_base +
				DMA_SCHED_CTRL_REG);
}
EXPORT_SYMBOL(cppi41_restore_context);

MODULE_DESCRIPTION("TI CPPI 4.1 support");
MODULE_AUTHOR("MontaVista Software");
MODULE_LICENSE("GPL");
