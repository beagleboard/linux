// SPDX-License-Identifier: GPL-2.0
/**
 * Endpoint Function Driver to implement Non-Transparent Bridge functionality
 *
 * Copyright (C) 2019 Texas Instruments
 * Author: Kishon Vijay Abraham I <kishon@ti.com>
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <linux/pci-epc.h>
#include <linux/pci-epf.h>

static struct workqueue_struct *kpcintb_workqueue;

#define COMMAND_CONFIGURE_DOORBELL	1
#define COMMAND_CONFIGURE_MW		2
#define COMMAND_LINK_UP			3

#define COMMAND_STATUS_OK		BIT(0)
#define COMMAND_STATUS_ERROR		BIT(1)
#define LINK_STATUS_UP			BIT(2)

#define SPAD_COUNT			64
#define DB_COUNT			4
#define NTB_MW_OFFSET			2
#define DB_COUNT_MASK			GENMASK(15, 0)
#define MSIX_ENABLE			BIT(16)
#define MAX_DB_COUNT			32
#define MAX_MW				4

enum epf_ntb_bar {
	BAR_CONFIG,
	BAR_PEER_SPAD,
	BAR_DB_MW1,
	BAR_MW2,
	BAR_MW3,
	BAR_MW4,
};

struct epf_ntb {
	u32 num_mws;
	u32 *mws_size;
	u32 db_count;
	u32 spad_count;
	struct pci_epf *epf;
	struct epf_ntb_epc *epc[2];
};

struct epf_ntb_epc {
	u8 func_no;
	u8 vfunc_no;
	bool linkup;
	u32 spad_size;
	struct pci_epc *epc;
	struct epf_ntb *epf_ntb;
	void __iomem *mw_addr[6];
	struct epf_ntb_ctrl *reg;
	struct pci_epf_bar *epf_bar;
	enum pci_barno epf_ntb_bar[6];
	struct delayed_work cmd_handler;
	enum pci_epc_interface_type type;
	const struct pci_epc_features *epc_features;
};

struct epf_ntb_ctrl {
	u32	command;
	u32	argument;
	u32	status;
	u32	topology;
	u64	addr;
	u32	size;
	u32	mw1_offset;
	u32	num_mws;
	u32	spad_offset;
	u32	spad_count;
	u32	db_entry_size;
	u32	db_data[MAX_DB_COUNT];
} __packed;

static struct pci_epf_header epf_ntb_header = {
	.vendorid	= PCI_ANY_ID,
	.deviceid	= PCI_ANY_ID,
	.baseclass_code	= PCI_BASE_CLASS_MEMORY,
	.interrupt_pin	= PCI_INTERRUPT_INTA,
};

static int epf_ntb_link_up(struct epf_ntb *ntb)
{
	enum pci_epc_interface_type type;
	struct epf_ntb_epc *ntb_epc;
	struct epf_ntb_ctrl *ctrl;
	u8 vfunc_no, func_no;
	int ret;

	for (type = PRIMARY_INTERFACE; type <= SECONDARY_INTERFACE; type++) {
		ntb_epc = ntb->epc[type];
		func_no = ntb_epc->func_no;
		vfunc_no = ntb_epc->vfunc_no;
		ctrl = ntb_epc->reg;
		ctrl->status |= LINK_STATUS_UP;
		ret = pci_epc_raise_irq(ntb_epc->epc, func_no, vfunc_no,
					PCI_EPC_IRQ_MSI, 1);
		if (ret < 0) {
			WARN(1, "%s intf: Failed to raise Link Up IRQ\n",
			     pci_epc_interface_string(type));
			return ret;
		}
	}

	return 0;
}

static int
epf_ntb_configure_mw(struct epf_ntb *ntb, enum pci_epc_interface_type type,
		     u32 mw)
{
	struct epf_ntb_epc *peer_ntb_epc;
	struct pci_epf_bar *peer_epf_bar;
	struct epf_ntb_epc *ntb_epc;
	enum pci_barno peer_barno;
	struct epf_ntb_ctrl *ctrl;
	phys_addr_t phys_addr;
	u8 vfunc_no, func_no;
	struct pci_epc *epc;
	u64 addr;
	u32 size;
	int ret;

	ntb_epc = ntb->epc[type];
	epc = ntb_epc->epc;

	peer_ntb_epc = ntb->epc[!type];
	peer_barno = peer_ntb_epc->epf_ntb_bar[mw + NTB_MW_OFFSET];
	peer_epf_bar = &peer_ntb_epc->epf_bar[peer_barno];

	phys_addr = peer_epf_bar->phys_addr;
	ctrl = ntb_epc->reg;
	addr = ctrl->addr;
	size = ctrl->size;
	if (mw + NTB_MW_OFFSET == BAR_DB_MW1)
		phys_addr += ctrl->mw1_offset;

	func_no = ntb_epc->func_no;
	vfunc_no = ntb_epc->vfunc_no;

	ret = pci_epc_map_addr(epc, func_no, vfunc_no, phys_addr, addr, size);
	WARN(ret < 0, "%s intf: Failed to map memory window %d address\n",
	     pci_epc_interface_string(type), mw);

	return ret;
}

static int
epf_ntb_configure_db(struct epf_ntb *ntb, enum pci_epc_interface_type type,
		     u16 db_count, bool msix)
{
	struct epf_ntb_epc *peer_ntb_epc;
	struct pci_epf_bar *peer_epf_bar;
	struct epf_ntb_ctrl *peer_ctrl;
	struct epf_ntb_epc *ntb_epc;
	enum pci_barno peer_barno;
	phys_addr_t phys_addr;
	u8 vfunc_no, func_no;
	struct pci_epc *epc;
	u32 db_entry_size;
	u32 db_data;
	int ret, i;

	if (db_count > MAX_DB_COUNT)
		return -EINVAL;

	ntb_epc = ntb->epc[type];
	epc = ntb_epc->epc;

	peer_ntb_epc = ntb->epc[!type];
	peer_barno = peer_ntb_epc->epf_ntb_bar[BAR_DB_MW1];
	peer_epf_bar = &peer_ntb_epc->epf_bar[peer_barno];
	peer_ctrl = peer_ntb_epc->reg;
	db_entry_size = peer_ctrl->db_entry_size;

	phys_addr = peer_epf_bar->phys_addr;
	func_no = ntb_epc->func_no;
	vfunc_no = ntb_epc->vfunc_no;

	ret = pci_epc_map_msi_irq(epc, func_no, vfunc_no, phys_addr, db_count,
				  db_entry_size, &db_data);
	if (ret < 0) {
		WARN(1, "%s intf: Failed to map MSI IRQ\n",
		     pci_epc_interface_string(type));
		return ret;
	}

	for (i = 0; i < db_count; i++)
		peer_ctrl->db_data[i] = db_data | i;

	return 0;
}

static void epf_ntb_cmd_handler(struct work_struct *work)
{
	enum pci_epc_interface_type type;
	struct epf_ntb_epc *ntb_epc;
	struct epf_ntb_ctrl *ctrl;
	u32 command, argument;
	struct epf_ntb *ntb;
	struct device *dev;
	u16 db_count;
	bool is_msix;
	int ret;

	ntb_epc = container_of(work, struct epf_ntb_epc, cmd_handler.work);
	ctrl = ntb_epc->reg;
	command = ctrl->command;
	if (!command)
		goto reset_handler;
	argument = ctrl->argument;

	ctrl->command = 0;
	ctrl->argument = 0;

	ctrl = ntb_epc->reg;
	type = ntb_epc->type;
	ntb = ntb_epc->epf_ntb;
	dev = &ntb->epf->dev;

	switch (command) {
	case COMMAND_CONFIGURE_DOORBELL:
		db_count = argument & DB_COUNT_MASK;
		is_msix = argument & MSIX_ENABLE;
		ret = epf_ntb_configure_db(ntb, type, db_count, is_msix);
		if (ret < 0)
			ctrl->status |= COMMAND_STATUS_ERROR;
		else
			ctrl->status |= COMMAND_STATUS_OK;
		break;
	case COMMAND_CONFIGURE_MW:
		ret = epf_ntb_configure_mw(ntb, type, argument);
		if (ret < 0)
			ctrl->status |= COMMAND_STATUS_ERROR;
		else
			ctrl->status |= COMMAND_STATUS_OK;
		break;
	case COMMAND_LINK_UP:
		ntb_epc->linkup = true;
		if (ntb->epc[PRIMARY_INTERFACE]->linkup &&
		    ntb->epc[SECONDARY_INTERFACE]->linkup) {
			ret = epf_ntb_link_up(ntb);
			if (ret < 0)
				ctrl->status |= COMMAND_STATUS_ERROR;
			else
				ctrl->status |= COMMAND_STATUS_OK;
			goto reset_handler;
		}
		ctrl->status |= COMMAND_STATUS_OK;
		break;
	default:
		dev_err(dev, "UNKNOWN command: %d\n", command);
		break;
	}

reset_handler:
	queue_delayed_work(kpcintb_workqueue, &ntb_epc->cmd_handler,
			   msecs_to_jiffies(5));
}

static void epf_ntb_peer_spad_bar_clear(struct epf_ntb_epc *ntb_epc)
{
	struct pci_epf_bar *epf_bar;
	enum pci_barno barno;
	u8 vfunc_no, func_no;
	struct pci_epc *epc;

	epc = ntb_epc->epc;
	func_no = ntb_epc->func_no;
	vfunc_no = ntb_epc->vfunc_no;
	barno = ntb_epc->epf_ntb_bar[BAR_PEER_SPAD];
	epf_bar = &ntb_epc->epf_bar[barno];
	pci_epc_clear_bar(epc, func_no, vfunc_no, epf_bar);
}

static int
epf_ntb_peer_spad_bar_set(struct epf_ntb *ntb, enum pci_epc_interface_type type)
{
	struct epf_ntb_epc *peer_ntb_epc;
	struct pci_epf_bar *peer_epf_bar;
	struct epf_ntb_epc *ntb_epc;
	struct pci_epf_bar *epf_bar;
	enum pci_barno peer_barno;
	u32 peer_spad_offset;
	enum pci_barno barno;
	u8 vfunc_no, func_no;
	struct pci_epc *epc;
	struct device *dev;
	int ret;

	dev = &ntb->epf->dev;

	peer_ntb_epc = ntb->epc[!type];
	peer_barno = peer_ntb_epc->epf_ntb_bar[BAR_CONFIG];
	peer_epf_bar = &peer_ntb_epc->epf_bar[peer_barno];

	ntb_epc = ntb->epc[type];
	barno = ntb_epc->epf_ntb_bar[BAR_PEER_SPAD];
	epf_bar = &ntb_epc->epf_bar[barno];
	func_no = ntb_epc->func_no;
	vfunc_no = ntb_epc->vfunc_no;
	epc = ntb_epc->epc;

	peer_spad_offset = peer_ntb_epc->reg->spad_offset;
	epf_bar->phys_addr = peer_epf_bar->phys_addr + peer_spad_offset;
	epf_bar->size = peer_ntb_epc->spad_size;
	epf_bar->barno = barno;
	epf_bar->flags = PCI_BASE_ADDRESS_MEM_TYPE_32;

	ret = pci_epc_set_bar(ntb_epc->epc, func_no, vfunc_no, epf_bar);
	if (ret) {
		dev_err(dev, "%s intf: peer SPAD BAR set failed\n",
			pci_epc_interface_string(type));
		return ret;
	}

	return 0;
}

static void epf_ntb_config_sspad_bar_clear(struct epf_ntb_epc *ntb_epc)
{
	struct pci_epf_bar *epf_bar;
	u8 vfunc_no, func_no;
	enum pci_barno barno;
	struct pci_epc *epc;

	epc = ntb_epc->epc;
	func_no = ntb_epc->func_no;
	vfunc_no = ntb_epc->vfunc_no;
	barno = ntb_epc->epf_ntb_bar[BAR_CONFIG];
	epf_bar = &ntb_epc->epf_bar[barno];
	pci_epc_clear_bar(epc, func_no, vfunc_no, epf_bar);
}

static int epf_ntb_config_sspad_bar_set(struct epf_ntb_epc *ntb_epc)
{
	struct pci_epf_bar *epf_bar;
	enum pci_barno barno;
	u8 vfunc_no, func_no;
	struct epf_ntb *ntb;
	struct pci_epc *epc;
	struct device *dev;
	int ret;

	ntb = ntb_epc->epf_ntb;
	dev = &ntb->epf->dev;

	epc = ntb_epc->epc;
	func_no = ntb_epc->func_no;
	vfunc_no = ntb_epc->vfunc_no;
	barno = ntb_epc->epf_ntb_bar[BAR_CONFIG];
	epf_bar = &ntb_epc->epf_bar[barno];

	ret = pci_epc_set_bar(epc, func_no, vfunc_no, epf_bar);
	if (ret) {
		dev_err(dev, "%s inft: Config/Status/SPAD BAR set failed\n",
			pci_epc_interface_string(ntb_epc->type));
		return ret;
	}

	return 0;
}

static void epf_ntb_config_spad_bar_free(struct epf_ntb *ntb)
{
	enum pci_epc_interface_type type;
	struct epf_ntb_epc *ntb_epc;
	enum pci_barno barno;
	struct pci_epf *epf;

	epf = ntb->epf;
	for (type = PRIMARY_INTERFACE; type <= SECONDARY_INTERFACE; type++) {
		ntb_epc = ntb->epc[type];
		barno = ntb_epc->epf_ntb_bar[BAR_CONFIG];
		if (ntb_epc->reg)
			pci_epf_free_space(epf, ntb_epc->reg, barno, type);
	}
}

static int
epf_ntb_config_spad_bar_alloc_interface(struct epf_ntb *ntb,
					enum pci_epc_interface_type type)
{
	const struct pci_epc_features *peer_epc_features;
	const struct pci_epc_features *epc_features;
	struct epf_ntb_epc *peer_ntb_epc;
	struct epf_ntb_epc *ntb_epc;
	struct epf_ntb_ctrl *ctrl;
	enum pci_barno peer_barno;
	struct device_node *node;
	u32 spad_size, ctrl_size;
	enum pci_barno barno;
	u64 size, peer_size;
	struct pci_epc *epc;
	struct pci_epf *epf;
	struct device *dev;
	u32 spad_count;
	size_t align;
	void *base;

	epf = ntb->epf;
	node = epf->node;
	dev = &epf->dev;
	ntb_epc = ntb->epc[type];
	epc = ntb_epc->epc;

	epc_features = ntb_epc->epc_features;
	barno = ntb_epc->epf_ntb_bar[BAR_CONFIG];
	size = epc_features->bar_fixed_size[barno];
	align = epc_features->align;

	peer_ntb_epc = ntb->epc[!type];
	peer_epc_features = peer_ntb_epc->epc_features;
	peer_barno = ntb_epc->epf_ntb_bar[BAR_PEER_SPAD];
	peer_size = peer_epc_features->bar_fixed_size[barno];

	/* Check if epc_features is populated incorrectly */
	if ((!IS_ALIGNED(size, align)))
		return -EINVAL;

	spad_count = SPAD_COUNT;
	of_property_read_u32(node, "spad-count", &spad_count);

	ctrl_size = sizeof(struct epf_ntb_ctrl);
	spad_size = spad_count * 4;

	if (!align) {
		ctrl_size = roundup_pow_of_two(ctrl_size);
		spad_size = roundup_pow_of_two(spad_size);
	} else {
		ctrl_size = ALIGN(ctrl_size, align);
		spad_size = ALIGN(spad_size, align);
	}

	if (peer_size) {
		if (peer_size < spad_size)
			spad_count = peer_size / 4;
		spad_size = peer_size;
	}

	/*
	 * In order to make sure SPAD offset is aligned to its size,
	 * expand control region size to the size of SPAD if SPAD size
	 * is greater than control region size.
	 */
	if (spad_size > ctrl_size)
		ctrl_size = spad_size;

	if (!size)
		size = ctrl_size + spad_size;
	else if (size < ctrl_size + spad_size)
		return -EINVAL;

	base = pci_epf_alloc_space(epf, size, barno, align, type);
	if (!base) {
		dev_err(dev, "%s intf: Config/Status/SPAD alloc region fail\n",
			pci_epc_interface_string(type));
		return -ENOMEM;
	}

	ntb_epc->reg = base;

	ctrl = ntb_epc->reg;
	ctrl->spad_offset = ctrl_size;
	ctrl->spad_count = spad_count;
	ctrl->num_mws = ntb->num_mws;
	ctrl->db_entry_size = align ? align : 4;
	ntb_epc->spad_size = spad_size;

	return 0;
}

static int epf_ntb_config_spad_bar_alloc(struct epf_ntb *ntb)
{
	enum pci_epc_interface_type type;
	struct device *dev;
	int ret;

	dev = &ntb->epf->dev;

	for (type = PRIMARY_INTERFACE; type <= SECONDARY_INTERFACE; type++) {
		ret = epf_ntb_config_spad_bar_alloc_interface(ntb, type);
		if (ret) {
			dev_err(dev, "%s intf: Config/SPAD BAR alloc failed\n",
				pci_epc_interface_string(type));
			goto err_config_spad_bar_alloc;
		}
	}

	return 0;

err_config_spad_bar_alloc:
	epf_ntb_config_spad_bar_free(ntb);

	return ret;
}

static void epf_ntb_free_peer_mem(struct epf_ntb_epc *ntb_epc)
{
	struct pci_epf_bar *epf_bar;
	void __iomem *mw_addr;
	phys_addr_t phys_addr;
	enum epf_ntb_bar bar;
	enum pci_barno barno;
	struct pci_epc *epc;
	size_t size;

	epc = ntb_epc->epc;

	for (bar = BAR_DB_MW1; bar < BAR_MW4; bar++) {
		barno = ntb_epc->epf_ntb_bar[bar];
		mw_addr = ntb_epc->mw_addr[barno];
		epf_bar = &ntb_epc->epf_bar[barno];
		phys_addr = epf_bar->phys_addr;
		size = epf_bar->size;
		if (mw_addr) {
			pci_epc_mem_free_addr(epc, phys_addr, mw_addr, size);
			ntb_epc->mw_addr[barno] = NULL;
		}
	}
}

static void epf_ntb_db_mw_bar_clear(struct epf_ntb_epc *ntb_epc)
{
	struct pci_epf_bar *epf_bar;
	enum epf_ntb_bar bar;
	enum pci_barno barno;
	u8 vfunc_no, func_no;
	struct pci_epc *epc;

	epc = ntb_epc->epc;

	func_no = ntb_epc->func_no;
	vfunc_no = ntb_epc->vfunc_no;

	for (bar = BAR_DB_MW1; bar < BAR_MW4; bar++) {
		barno = ntb_epc->epf_ntb_bar[bar];
		epf_bar = &ntb_epc->epf_bar[barno];
		pci_epc_clear_bar(epc, func_no, vfunc_no, epf_bar);
	}
}

static void epf_ntb_db_mw_bar_cleanup(struct epf_ntb_epc *ntb_epc,
				      struct epf_ntb_epc *peer_ntb_epc)
{
	epf_ntb_db_mw_bar_clear(ntb_epc);
	epf_ntb_free_peer_mem(peer_ntb_epc);
}

static int
epf_ntb_alloc_peer_mem(struct device *dev, struct epf_ntb_epc *ntb_epc,
		       enum epf_ntb_bar bar, struct epf_ntb_epc *peer_ntb_epc,
		       size_t size)
{
	const struct pci_epc_features *epc_features;
	struct pci_epf_bar *epf_bar;
	struct pci_epc *peer_epc;
	phys_addr_t phys_addr;
	void __iomem *mw_addr;
	enum pci_barno barno;
	size_t align;

	epc_features = ntb_epc->epc_features;
	align = epc_features->align;

	if (size < 128)
		size = 128;

	if (align)
		size = ALIGN(size, align);
	else
		size = roundup_pow_of_two(size);

	peer_epc = peer_ntb_epc->epc;
	mw_addr = pci_epc_mem_alloc_addr(peer_epc, &phys_addr, size);
	if (!mw_addr) {
		dev_err(dev, "%s intf: Failed to allocate OB address\n",
			pci_epc_interface_string(peer_ntb_epc->type));
		return -ENOMEM;
	}

	barno = ntb_epc->epf_ntb_bar[bar];
	epf_bar = &ntb_epc->epf_bar[barno];
	ntb_epc->mw_addr[barno] = mw_addr;

	epf_bar->phys_addr = phys_addr;
	epf_bar->size = size;
	epf_bar->barno = barno;
	epf_bar->flags = PCI_BASE_ADDRESS_MEM_TYPE_32;

	return 0;
}

static int epf_ntb_configure_interrupt(struct epf_ntb *ntb,
				       enum pci_epc_interface_type type)
{
	const struct pci_epc_features *epc_features;
	bool msix_capable, msi_capable;
	struct epf_ntb_epc *ntb_epc;
	struct device_node *node;
	u8 vfunc_no, func_no;
	struct pci_epc *epc;
	struct device *dev;
	u32 db_count;
	int ret;

	ntb_epc = ntb->epc[type];
	dev = &ntb->epf->dev;
	node = ntb->epf->node;

	epc_features = ntb_epc->epc_features;
	msix_capable = epc_features->msix_capable;
	msi_capable = epc_features->msi_capable;

	if (!(msix_capable || msi_capable)) {
		dev_err(dev, "MSI or MSI-X is required for doorbell\n");
		return -EINVAL;
	}

	func_no = ntb_epc->func_no;
	vfunc_no = ntb_epc->vfunc_no;

	db_count = DB_COUNT;
	of_property_read_u32(node, "db-count", &db_count);
	if (db_count > MAX_DB_COUNT) {
		dev_err(dev, "DB count cannot be more than %d\n", MAX_DB_COUNT);
		return -EINVAL;
	}

	ntb->db_count = db_count;
	epc = ntb_epc->epc;

	if (msi_capable) {
		ret = pci_epc_set_msi(epc, func_no, vfunc_no, db_count);
		if (ret) {
			dev_err(dev, "%s intf: MSI configuration failed\n",
				pci_epc_interface_string(type));
			return ret;
		}
	}

	if (msix_capable) {
		ret = pci_epc_set_msix(epc, func_no, vfunc_no, db_count);
		if (ret) {
			dev_err(dev, "MSI configuration failed\n");
			return ret;
		}
	}

	return 0;
}

static int epf_ntb_db_mw_bar_init(struct epf_ntb *ntb,
				  enum pci_epc_interface_type type)
{
	const struct pci_epc_features *epc_features;
	struct epf_ntb_epc *peer_ntb_epc;
	struct epf_ntb_epc *ntb_epc;
	struct pci_epf_bar *epf_bar;
	struct epf_ntb_ctrl *ctrl;
	enum epf_ntb_bar bar;
	u8 vfunc_no, func_no;
	enum pci_barno barno;
	struct pci_epc *epc;
	struct device *dev;
	u32 num_mws, size;
	u32 db_count;
	size_t align;
	int ret;
	int i;

	ntb_epc = ntb->epc[type];
	peer_ntb_epc = ntb->epc[!type];

	dev = &ntb->epf->dev;
	epc_features = ntb_epc->epc_features;
	align = epc_features->align;
	func_no = ntb_epc->func_no;
	vfunc_no = ntb_epc->vfunc_no;
	epc = ntb_epc->epc;
	num_mws = ntb->num_mws;
	db_count = ntb->db_count;

	for (bar = BAR_DB_MW1, i = 0; i < num_mws; bar++, i++) {
		if (bar == BAR_DB_MW1) {
			align = align ? align : 4;
			size = db_count * align;
			size = ALIGN(size, ntb->mws_size[i]);
			ctrl = ntb_epc->reg;
			ctrl->mw1_offset = size;
			size += ntb->mws_size[i];
		} else {
			size = ntb->mws_size[i];
		}

		ret = epf_ntb_alloc_peer_mem(dev, ntb_epc, bar,
					     peer_ntb_epc, size);
		if (ret)
			goto err_alloc_peer_mem;

		barno = ntb_epc->epf_ntb_bar[bar];
		epf_bar = &ntb_epc->epf_bar[barno];

		ret = pci_epc_set_bar(epc, func_no, vfunc_no, epf_bar);
		if (ret) {
			dev_err(dev, "%s intf: DoorBell BAR set failed\n",
				pci_epc_interface_string(type));
			goto err_alloc_peer_mem;
		}
	}

	return 0;

err_alloc_peer_mem:
	epf_ntb_db_mw_bar_cleanup(ntb_epc, peer_ntb_epc);

	return ret;
}

static void epf_ntb_epc_destroy(struct epf_ntb *ntb)
{
	enum pci_epc_interface_type type;
	struct epf_ntb_epc *ntb_epc;
	struct pci_epc *epc;
	struct pci_epf *epf;

	epf = ntb->epf;
	for (type = PRIMARY_INTERFACE; type <= SECONDARY_INTERFACE; type++) {
		ntb_epc = ntb->epc[type];
		if (!ntb_epc)
			return;
		epc = ntb_epc->epc;
		pci_epc_remove_epf(epc, epf, type);
		pci_epc_put(epc);
	}
}

static int
epf_ntb_epc_create_interface(struct epf_ntb *ntb, struct pci_epc *epc,
			     enum pci_epc_interface_type type)
{
	const struct pci_epc_features *epc_features;
	struct pci_epf_bar *epf_bar;
	struct epf_ntb_epc *ntb_epc;
	u8 vfunc_no, func_no;
	struct pci_epf *epf;
	struct device *dev;

	dev = &ntb->epf->dev;

	ntb_epc = devm_kzalloc(dev, sizeof(*ntb_epc), GFP_KERNEL);
	if (!ntb_epc)
		return -ENOMEM;

	epf = ntb->epf;
	if (type == PRIMARY_INTERFACE) {
		func_no = epf->func_no;
		vfunc_no = epf->vfunc_no;
		epf_bar = epf->bar;
	} else {
		func_no = epf->sec_epc_func_no;
		vfunc_no = epf->sec_epc_vfunc_no;
		epf_bar = epf->sec_epc_bar;
	}

	ntb_epc->linkup = false;
	ntb_epc->epc = epc;
	ntb_epc->func_no = func_no;
	ntb_epc->vfunc_no = vfunc_no;
	ntb_epc->type = type;
	ntb_epc->epf_bar = epf_bar;
	ntb_epc->epf_ntb = ntb;

	epc_features = pci_epc_get_features(epc, func_no, vfunc_no);
	ntb_epc->epc_features = epc_features;

	ntb->epc[type] = ntb_epc;

	return 0;
}

static int epf_ntb_epc_create(struct epf_ntb *ntb)
{
	enum pci_epc_interface_type type;
	struct device_node *node;
	const char *epc_name;
	struct pci_epc *epc;
	struct pci_epf *epf;
	struct device *dev;
	int ret;

	epf = ntb->epf;
	node = epf->node;
	dev = &epf->dev;

	for (type = PRIMARY_INTERFACE; type <= SECONDARY_INTERFACE; type++) {
		epc_name = pci_epc_interface_string(type);

		epc = of_pci_epc_get_by_name(node, epc_name);
		if (IS_ERR(epc)) {
			if (PTR_ERR(epc) != -EPROBE_DEFER)
				dev_err(dev, "%s intf: Failed to get EPC\n",
					epc_name);
			ret = PTR_ERR(epc);
			goto err_epc_get;
		}

		ret = pci_epc_add_epf(epc, epf, type);
		if (ret) {
			dev_err(dev, "%s intf: Fail to add EPF to EPC\n",
				epc_name);
			goto err_epc_get;
		}

		ret = epf_ntb_epc_create_interface(ntb, epc, type);
		if (ret) {
			dev_err(dev, "%s intf: Fail to create NTB EPC\n",
				epc_name);
			goto err_epc_get;
		}
	}

	return 0;

err_epc_get:
	epf_ntb_epc_destroy(ntb);

	return ret;
}

static int epf_ntb_init_epc_bar_interface(struct epf_ntb *ntb,
					  enum pci_epc_interface_type type)
{
	const struct pci_epc_features *epc_features;
	struct epf_ntb_epc *ntb_epc;
	enum pci_barno barno;
	enum epf_ntb_bar bar;
	struct device *dev;
	u32 num_mws;
	int i;

	barno = BAR_0;
	ntb_epc = ntb->epc[type];
	num_mws = ntb->num_mws;
	dev = &ntb->epf->dev;
	epc_features = ntb_epc->epc_features;

	/* These are required BARs which are mandatory for NTB functionality */
	for (bar = BAR_CONFIG; bar <= BAR_DB_MW1; bar++, barno++) {
		barno = pci_epc_get_next_free_bar(epc_features, barno);
		if (barno < 0) {
			dev_err(dev, "%s intf: Fail to get NTB function BAR\n",
				pci_epc_interface_string(type));
			return barno;
		}
		ntb_epc->epf_ntb_bar[bar] = barno;
	}

	/* These are optional BARs which doesn't impact NTB functionality */
	for (bar = BAR_MW2, i = 1; i < num_mws; bar++, barno++, i++) {
		barno = pci_epc_get_next_free_bar(epc_features, barno);
		if (barno < 0) {
			ntb->num_mws = i;
			dev_dbg(dev, "BAR not available for > MW%d\n", i + 1);
		}
		ntb_epc->epf_ntb_bar[bar] = barno;
	}

	return 0;
}

static int epf_ntb_init_epc_bar(struct epf_ntb *ntb)
{
	enum pci_epc_interface_type type;
	struct device *dev;
	int ret;

	dev = &ntb->epf->dev;
	for (type = PRIMARY_INTERFACE; type <= SECONDARY_INTERFACE; type++) {
		ret = epf_ntb_init_epc_bar_interface(ntb, type);
		if (ret) {
			dev_err(dev, "Fail to init EPC bar for %s interface\n",
				pci_epc_interface_string(type));
			return ret;
		}
	}

	return 0;
}

static int epf_ntb_epc_init_interface(struct epf_ntb *ntb,
				      enum pci_epc_interface_type type)
{
	struct epf_ntb_epc *ntb_epc;
	u8 vfunc_no, func_no;
	struct pci_epc *epc;
	struct pci_epf *epf;
	struct device *dev;
	int ret;

	ntb_epc = ntb->epc[type];
	epf = ntb->epf;
	dev = &epf->dev;
	epc = ntb_epc->epc;
	func_no = ntb_epc->func_no;
	vfunc_no = ntb_epc->vfunc_no;

	ret = epf_ntb_config_sspad_bar_set(ntb->epc[type]);
	if (ret) {
		dev_err(dev, "%s intf: Config/self SPAD BAR init failed\n",
			pci_epc_interface_string(type));
		return ret;
	}

	ret = epf_ntb_peer_spad_bar_set(ntb, type);
	if (ret) {
		dev_err(dev, "%s intf: Peer SPAD BAR init failed\n",
			pci_epc_interface_string(type));
		goto err_peer_spad_bar_init;
	}

	ret = epf_ntb_configure_interrupt(ntb, type);
	if (ret) {
		dev_err(dev, "%s intf: Interrupt configuration failed\n",
			pci_epc_interface_string(type));
		goto err_peer_spad_bar_init;
	}

	ret = epf_ntb_db_mw_bar_init(ntb, type);
	if (ret) {
		dev_err(dev, "%s intf: DB/MW BAR init failed\n",
			pci_epc_interface_string(type));
		goto err_db_mw_bar_init;
	}

	ret = pci_epc_write_header(epc, func_no, vfunc_no, epf->header);
	if (ret) {
		dev_err(dev, "%s intf: Configuration header write failed\n",
			pci_epc_interface_string(type));
		goto err_write_header;
	}

	INIT_DELAYED_WORK(&ntb->epc[type]->cmd_handler, epf_ntb_cmd_handler);
	queue_work(kpcintb_workqueue, &ntb->epc[type]->cmd_handler.work);

	return 0;

err_write_header:
	epf_ntb_db_mw_bar_cleanup(ntb->epc[type], ntb->epc[!type]);

err_db_mw_bar_init:
	epf_ntb_peer_spad_bar_clear(ntb->epc[type]);

err_peer_spad_bar_init:
	epf_ntb_config_sspad_bar_clear(ntb->epc[type]);

	return ret;
}

static void epf_ntb_epc_cleanup(struct epf_ntb *ntb)
{
	enum pci_epc_interface_type type;
	struct epf_ntb_epc *peer_ntb_epc;
	struct epf_ntb_epc *ntb_epc;

	for (type = PRIMARY_INTERFACE; type <= SECONDARY_INTERFACE; type++) {
		ntb_epc = ntb->epc[type];
		peer_ntb_epc = ntb->epc[!type];
		cancel_delayed_work(&ntb_epc->cmd_handler);
		epf_ntb_db_mw_bar_cleanup(ntb_epc, peer_ntb_epc);
		epf_ntb_peer_spad_bar_clear(ntb_epc);
		epf_ntb_config_sspad_bar_clear(ntb_epc);
	}
}

static int epf_ntb_epc_init(struct epf_ntb *ntb)
{
	enum pci_epc_interface_type type;
	struct device *dev;
	int ret;

	dev = &ntb->epf->dev;

	for (type = PRIMARY_INTERFACE; type <= SECONDARY_INTERFACE; type++) {
		ret = epf_ntb_epc_init_interface(ntb, type);
		if (ret) {
			dev_err(dev, "%s intf: Failed to initialize\n",
				pci_epc_interface_string(type));
			goto err_init_type;
		}
	}

	return 0;

err_init_type:
	epf_ntb_epc_cleanup(ntb);

	return ret;
}

static int epf_ntb_of_parse_mw(struct epf_ntb *ntb, struct device_node *node)
{
	struct device *dev;
	u32 *mws_size;
	u32 num_mws;
	int ret;

	dev = &ntb->epf->dev;
	ret = of_property_read_u32(node, "num-mws", &num_mws);
	if (ret) {
		dev_err(dev, "Failed to get num-mws dt property\n");
		return ret;
	}

	if (num_mws > MAX_MW) {
		dev_err(dev, "Cannot support more than 4 memory window\n");
		return ret;
	}

	mws_size = devm_kzalloc(dev, sizeof(*mws_size) * num_mws, GFP_KERNEL);
	if (!mws_size)
		return -ENOMEM;

	ret = of_property_read_u32_array(node, "mws-size", mws_size,
					 num_mws);
	if (ret) {
		dev_err(dev, "Failed to get mws-size dt property\n");
		return ret;
	}

	ntb->num_mws = num_mws;
	ntb->mws_size = mws_size;

	return 0;
}

static int pci_epf_ntb_of_parse(struct epf_ntb *ntb)
{
	struct device_node *node;
	struct pci_epf *epf;
	struct device *dev;
	int ret;

	epf = ntb->epf;
	node = epf->node;
	dev = &epf->dev;

	epf->header = &epf_ntb_header;
	pci_epc_of_parse_header(node, epf->header);

	ret = epf_ntb_of_parse_mw(ntb, node);
	if (ret) {
		dev_err(dev, "Invalid memory window configuration in DT\n");
		return ret;
	}

	return 0;
}

static int pci_epf_ntb_probe(struct pci_epf *epf)
{
	struct epf_ntb *ntb;
	struct device *dev;
	int ret;

	dev = &epf->dev;

	ntb = devm_kzalloc(dev, sizeof(*ntb), GFP_KERNEL);
	if (!ntb)
		return -ENOMEM;

	ntb->epf = epf;

	ret = pci_epf_ntb_of_parse(ntb);
	if (ret) {
		dev_err(dev, "Failed to parse NTB DT node\n");
		return ret;
	}

	ret = epf_ntb_epc_create(ntb);
	if (ret) {
		dev_err(dev, "Failed to create NTB EPC\n");
		return ret;
	}

	ret = epf_ntb_init_epc_bar(ntb);
	if (ret) {
		dev_err(dev, "Failed to create NTB EPC\n");
		goto err_bar_init;
	}

	ret = epf_ntb_config_spad_bar_alloc(ntb);
	if (ret) {
		dev_err(dev, "Failed to allocate BAR memory\n");
		goto err_bar_init;
	}

	ret = epf_ntb_epc_init(ntb);
	if (ret) {
		dev_err(dev, "Failed to initialize EPC\n");
		goto err_epc_init;
	}

	epf_set_drvdata(epf, ntb);

	return 0;

err_epc_init:
	epf_ntb_config_spad_bar_free(ntb);

err_bar_init:
	epf_ntb_epc_destroy(ntb);

	return ret;
}

static int pci_epf_ntb_remove(struct pci_epf *epf)
{
	struct epf_ntb *ntb = epf_get_drvdata(epf);

	epf_ntb_epc_cleanup(ntb);
	epf_ntb_config_spad_bar_free(ntb);
	epf_ntb_epc_destroy(ntb);

	return 0;
}

static const struct pci_epf_device_id pci_epf_ntb_ids[] = {
	{
		.name = "pci-epf-ntb",
	},
	{},
};

static struct pci_epf_driver epf_ntb_driver = {
	.driver.name	= "pci_epf_ntb",
	.probe		= pci_epf_ntb_probe,
	.remove		= pci_epf_ntb_remove,
	.id_table	= pci_epf_ntb_ids,
	.owner		= THIS_MODULE,
};

static int __init pci_epf_ntb_init(void)
{
	int ret;

	kpcintb_workqueue = alloc_workqueue("kpcintb", WQ_MEM_RECLAIM |
					    WQ_HIGHPRI, 0);
	ret = pci_epf_register_driver(&epf_ntb_driver);
	if (ret) {
		pr_err("Failed to register pci epf ntb driver --> %d\n", ret);
		return ret;
	}

	return 0;
}
module_init(pci_epf_ntb_init);

static void __exit pci_epf_ntb_exit(void)
{
	pci_epf_unregister_driver(&epf_ntb_driver);
}
module_exit(pci_epf_ntb_exit);

MODULE_DESCRIPTION("PCI EPF NTB DRIVER");
MODULE_AUTHOR("Kishon Vijay Abraham I <kishon@ti.com>");
MODULE_LICENSE("GPL v2");
