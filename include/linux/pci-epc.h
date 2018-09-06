/**
 * PCI Endpoint *Controller* (EPC) header file
 *
 * Copyright (C) 2017 Texas Instruments
 * Author: Kishon Vijay Abraham I <kishon@ti.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 of
 * the License as published by the Free Software Foundation.
 */

#ifndef __LINUX_PCI_EPC_H
#define __LINUX_PCI_EPC_H

#include <linux/pci-epf.h>

struct pci_epc;

enum pci_epc_irq_type {
	PCI_EPC_IRQ_UNKNOWN,
	PCI_EPC_IRQ_LEGACY,
	PCI_EPC_IRQ_MSI,
};

/**
 * struct pci_epc_ops - set of function pointers for performing EPC operations
 * @epf_init: ops to perform EPC specific initialization
 * @epf_exit: ops to cleanup EPF
 * @data_transfer: ops to transfer data with the remote RC
 * @write_header: ops to populate configuration space header
 * @set_bar: ops to configure the BAR
 * @clear_bar: ops to reset the BAR
 * @map_addr: ops to map CPU address to PCI address
 * @unmap_addr: ops to unmap CPU address and PCI address
 * @set_msi: ops to set the requested number of MSI interrupts in the MSI
 *	     capability register
 * @get_msi: ops to get the number of MSI interrupts allocated by the RC from
 *	     the MSI capability register
 * @raise_irq: ops to raise a legacy or MSI interrupt
 * @start: ops to start the PCI link
 * @stop: ops to stop the PCI link
 * @owner: the module owner containing the ops
 */
struct pci_epc_ops {
	int	(*epf_init)(struct pci_epc *epc, struct pci_epf *epf);
	void	(*epf_exit)(struct pci_epc *epc, struct pci_epf *epf);
	int	(*data_transfer)(struct pci_epc *epc, struct pci_epf *epf,
				 dma_addr_t dma_dst, dma_addr_t dma_src,
				 size_t len);
	int	(*write_header)(struct pci_epc *epc, u8 func_no,
				struct pci_epf_header *hdr);
	int	(*set_bar)(struct pci_epc *epc, u8 func_no,
			   struct pci_epf_bar *epf_bar);
	void	(*clear_bar)(struct pci_epc *epc, u8 func_no,
			     struct pci_epf_bar *epf_bar);
	int	(*map_addr)(struct pci_epc *epc, u8 func_no,
			    phys_addr_t addr, u64 pci_addr, size_t size);
	void	(*unmap_addr)(struct pci_epc *epc, u8 func_no,
			      phys_addr_t addr);
	int	(*set_msi)(struct pci_epc *epc, u8 func_no, u8 interrupts);
	int	(*get_msi)(struct pci_epc *epc, u8 func_no);
	int	(*raise_irq)(struct pci_epc *epc, u8 func_no,
			     enum pci_epc_irq_type type, u8 interrupt_num);
	int	(*start)(struct pci_epc *epc);
	void	(*stop)(struct pci_epc *epc);
	struct module *owner;
};

/**
 * struct pci_epc_mem - address space of the endpoint controller
 * @phys_base: physical base address of the PCI address space
 * @size: the size of the PCI address space
 * @bitmap: bitmap to manage the PCI address space
 * @pages: number of bits representing the address region
 * @page_size: size of each page
 * @lock: mutex to protect bitmap
 */
struct pci_epc_mem {
	phys_addr_t	phys_base;
	size_t		size;
	unsigned long	*bitmap;
	size_t		page_size;
	int		pages;
	/* mutex to protect against concurrent access for memory allocation*/
	struct mutex	lock;
};

/**
 * struct pci_epc - represents the PCI EPC device
 * @dev: PCI EPC device
 * @pci_epf: list of endpoint functions present in this EPC device
 * @ops: function pointers for performing endpoint operations
 * @mem: address space of the endpoint controller
 * @max_functions: max number of functions that can be configured in this EPC
 * @group: configfs group representing the PCI EPC device
 * @lock: mutex to protect pci_epc ops
 * @notifier: used to notify EPF of any EPC events (like linkup)
 */
struct pci_epc {
	struct device			dev;
	struct list_head		pci_epf;
	const struct pci_epc_ops	*ops;
	struct pci_epc_mem		*mem;
	u8				max_functions;
	struct config_group		*group;
	/* mutex to protect against concurrent access of EP controller */
	struct mutex			lock;
	struct atomic_notifier_head	notifier;
};

#define to_pci_epc(device) container_of((device), struct pci_epc, dev)

#define pci_epc_create(dev, ops)    \
		__pci_epc_create((dev), (ops), THIS_MODULE)
#define devm_pci_epc_create(dev, ops)    \
		__devm_pci_epc_create((dev), (ops), THIS_MODULE)

#define pci_epc_mem_init(epc, phys_addr, size)	\
		__pci_epc_mem_init((epc), (phys_addr), (size), PAGE_SIZE)

static inline void epc_set_drvdata(struct pci_epc *epc, void *data)
{
	dev_set_drvdata(&epc->dev, data);
}

static inline void *epc_get_drvdata(struct pci_epc *epc)
{
	return dev_get_drvdata(&epc->dev);
}

static inline int
pci_epc_register_notifier(struct pci_epc *epc, struct notifier_block *nb)
{
	return atomic_notifier_chain_register(&epc->notifier, nb);
}

struct pci_epc *
__devm_pci_epc_create(struct device *dev, const struct pci_epc_ops *ops,
		      struct module *owner);
struct pci_epc *
__pci_epc_create(struct device *dev, const struct pci_epc_ops *ops,
		 struct module *owner);
void devm_pci_epc_destroy(struct device *dev, struct pci_epc *epc);
void pci_epc_destroy(struct pci_epc *epc);
int pci_epc_add_epf(struct pci_epc *epc, struct pci_epf *epf);
void pci_epc_linkup(struct pci_epc *epc);
void pci_epc_remove_epf(struct pci_epc *epc, struct pci_epf *epf);
int pci_epc_write_header(struct pci_epc *epc, u8 func_no,
			 struct pci_epf_header *hdr);
int pci_epc_set_bar(struct pci_epc *epc, u8 func_no,
		    struct pci_epf_bar *epf_bar);
void pci_epc_clear_bar(struct pci_epc *epc, u8 func_no,
		       struct pci_epf_bar *epf_bar);
int pci_epc_map_addr(struct pci_epc *epc, u8 func_no,
		     phys_addr_t phys_addr,
		     u64 pci_addr, size_t size);
void pci_epc_unmap_addr(struct pci_epc *epc, u8 func_no,
			phys_addr_t phys_addr);
int pci_epc_set_msi(struct pci_epc *epc, u8 func_no, u8 interrupts);
int pci_epc_get_msi(struct pci_epc *epc, u8 func_no);
int pci_epc_raise_irq(struct pci_epc *epc, u8 func_no,
		      enum pci_epc_irq_type type, u8 interrupt_num);
int pci_epc_start(struct pci_epc *epc);
void pci_epc_stop(struct pci_epc *epc);
struct pci_epc *pci_epc_get(const char *epc_name);
void pci_epc_put(struct pci_epc *epc);

int __pci_epc_mem_init(struct pci_epc *epc, phys_addr_t phys_addr, size_t size,
		       size_t page_size);
void pci_epc_mem_exit(struct pci_epc *epc);
void __iomem *pci_epc_mem_alloc_addr(struct pci_epc *epc,
				     phys_addr_t *phys_addr, size_t size);
void pci_epc_mem_free_addr(struct pci_epc *epc, phys_addr_t phys_addr,
			   void __iomem *virt_addr, size_t size);
int pci_epc_epf_init(struct pci_epc *epc, struct pci_epf *epf);
void pci_epc_epf_exit(struct pci_epc *epc, struct pci_epf *epf);
#endif /* __LINUX_PCI_EPC_H */
