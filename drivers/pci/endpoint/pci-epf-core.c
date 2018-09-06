/**
 * PCI Endpoint *Function* (EPF) library
 *
 * Copyright (C) 2017 Texas Instruments
 * Author: Kishon Vijay Abraham I <kishon@ti.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 of
 * the License as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/pci-epc.h>
#include <linux/pci-epf.h>
#include <linux/pci-ep-cfs.h>

static struct bus_type pci_epf_bus_type;
static const struct device_type pci_epf_type;

static void pci_epf_dma_callback(void *param)
{
	struct pci_epf *epf = param;

	complete(&epf->transfer_complete);
}

/**
 * pci_epf_data_transfer() - Helper to use dmaengine API to transfer data
 *			     between PCIe EP and remote PCIe RC
 * @epf: the EPF device that performs the data transfer operation
 * @dma_dst: The destination address of the data transfer. It can be a physical
 *	     address given by pci_epc_mem_alloc_addr or DMA mapping APIs.
 * @dma_src: The source address of the data transfer. It can be a physical
 *	     address given by pci_epc_mem_alloc_addr or DMA mapping APIs.
 * @len: The size of the data transfer
 *
 * Helper to use dmaengine API to transfer data between PCIe EP and remote PCIe
 * RC. The source and destination address can be a physical address given by
 * pci_epc_mem_alloc_addr or the one obtained using DMA mapping APIs.
 *
 * The function returns '0' on success and negative value on failure.
 */
int pci_epf_data_transfer(struct pci_epf *epf, dma_addr_t dma_dst,
			  dma_addr_t dma_src, size_t len)
{
	enum dma_ctrl_flags flags = DMA_CTRL_ACK | DMA_PREP_INTERRUPT;
	struct dma_chan *chan = epf->dma_chan;
	struct dma_async_tx_descriptor *tx;
	struct device *dev = &epf->dev;
	dma_cookie_t cookie;
	int ret;

	if (IS_ERR_OR_NULL(epf)) {
		dev_err(dev, "Invalid EPF device\n");
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(chan)) {
		dev_err(dev, "Invalid DMA memcpy channel\n");
		return -EINVAL;
	}

	tx = dmaengine_prep_dma_memcpy(chan, dma_dst, dma_src, len, flags);
	if (!tx) {
		dev_err(dev, "Failed to prepare DMA memcpy\n");
		return -EIO;
	}

	tx->callback = pci_epf_dma_callback;
	tx->callback_param = epf;
	cookie = tx->tx_submit(tx);
	reinit_completion(&epf->transfer_complete);

	ret = dma_submit_error(cookie);
	if (ret) {
		dev_err(dev, "Failed to do DMA tx_submit %d\n", cookie);
		return -EIO;
	}

	dma_async_issue_pending(chan);
	ret = wait_for_completion_interruptible(&epf->transfer_complete);
	if (ret < 0) {
		dmaengine_terminate_sync(chan);
		dev_err(dev, "DMA wait_for_completion_timeout\n");
		return -ETIMEDOUT;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(pci_epf_data_transfer);

/**
 * pci_epf_init_dma_chan() - Helper to initialize EPF DMA channel
 * @epf: the EPF device that has to perform the data transfer operation
 *
 * Helper to initialize EPF DMA channel.
 */
int pci_epf_init_dma_chan(struct pci_epf *epf)
{
	struct device *dev = &epf->dev;
	struct dma_chan *dma_chan;
	dma_cap_mask_t mask;
	int ret;

	if (IS_ERR_OR_NULL(epf)) {
		dev_err(dev, "Invalid EPF device\n");
		return -EINVAL;
	}

	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);

	dma_chan = dma_request_chan_by_mask(&mask);
	if (IS_ERR(dma_chan)) {
		ret = PTR_ERR(dma_chan);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to get DMA channel\n");
		return ret;
	}
	init_completion(&epf->transfer_complete);

	epf->dma_chan = dma_chan;

	return 0;
}
EXPORT_SYMBOL_GPL(pci_epf_init_dma_chan);

/**
 * pci_epf_clean_dma_chan() - Helper to cleanup EPF DMA channel
 * @epf: the EPF device that performed the data transfer operation
 *
 * Helper to cleanup EPF DMA channel.
 */
void pci_epf_clean_dma_chan(struct pci_epf *epf)
{
	struct device *dev = &epf->dev;

	if (IS_ERR_OR_NULL(epf)) {
		dev_err(dev, "Invalid EPF device\n");
		return;
	}

	dma_release_channel(epf->dma_chan);
	epf->dma_chan = NULL;
}
EXPORT_SYMBOL_GPL(pci_epf_clean_dma_chan);

/**
 * pci_epf_tx() - transfer data between EPC and remote PCIe RC
 * @epf: the EPF device that performs the data transfer operation
 * @dma_dst: The destination address of the data transfer. It can be a physical
 *	     address given by pci_epc_mem_alloc_addr or DMA mapping APIs.
 * @dma_src: The source address of the data transfer. It can be a physical
 *	     address given by pci_epc_mem_alloc_addr or DMA mapping APIs.
 * @len: The size of the data transfer
 *
 * Invoke to transfer data between EPC and remote PCIe RC. The source and
 * destination address can be a physical address given by pci_epc_mem_alloc_addr
 * or the one obtained using DMA mapping APIs.
 */
int pci_epf_tx(struct pci_epf *epf, dma_addr_t dma_dst,
	       dma_addr_t dma_src, size_t len)
{
	int ret;
	struct pci_epc *epc = epf->epc;

	if (IS_ERR_OR_NULL(epc) || IS_ERR_OR_NULL(epf))
		return -EINVAL;

	if (!epc->ops->data_transfer)
		return -EINVAL;

	mutex_lock(&epf->lock);
	ret = epc->ops->data_transfer(epc, epf, dma_dst, dma_src, len);
	mutex_unlock(&epf->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(pci_epf_tx);

/**
 * pci_epf_unbind() - Notify the function driver that the binding between the
 *		      EPF device and EPC device has been lost
 * @epf: the EPF device which has lost the binding with the EPC device
 *
 * Invoke to notify the function driver that the binding between the EPF device
 * and EPC device has been lost.
 */
void pci_epf_unbind(struct pci_epf *epf)
{
	if (!epf->driver) {
		dev_WARN(&epf->dev, "epf device not bound to driver\n");
		return;
	}

	mutex_lock(&epf->lock);
	epf->driver->ops->unbind(epf);
	mutex_unlock(&epf->lock);
	module_put(epf->driver->owner);
}
EXPORT_SYMBOL_GPL(pci_epf_unbind);

/**
 * pci_epf_bind() - Notify the function driver that the EPF device has been
 *		    bound to a EPC device
 * @epf: the EPF device which has been bound to the EPC device
 *
 * Invoke to notify the function driver that it has been bound to a EPC device
 */
int pci_epf_bind(struct pci_epf *epf)
{
	int ret;

	if (!epf->driver) {
		dev_WARN(&epf->dev, "epf device not bound to driver\n");
		return -EINVAL;
	}

	if (!try_module_get(epf->driver->owner))
		return -EAGAIN;

	mutex_lock(&epf->lock);
	ret = epf->driver->ops->bind(epf);
	mutex_unlock(&epf->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(pci_epf_bind);

/**
 * pci_epf_free_space() - free the allocated PCI EPF register space
 * @addr: the virtual address of the PCI EPF register space
 * @bar: the BAR number corresponding to the register space
 *
 * Invoke to free the allocated PCI EPF register space.
 */
void pci_epf_free_space(struct pci_epf *epf, void *addr, enum pci_barno bar)
{
	struct device *dev = epf->epc->dev.parent;

	if (!addr)
		return;

	dma_free_coherent(dev, epf->bar[bar].size, addr,
			  epf->bar[bar].phys_addr);

	epf->bar[bar].phys_addr = 0;
	epf->bar[bar].size = 0;
	epf->bar[bar].barno = 0;
	epf->bar[bar].flags = 0;
}
EXPORT_SYMBOL_GPL(pci_epf_free_space);

/**
 * pci_epf_alloc_space() - allocate memory for the PCI EPF register space
 * @size: the size of the memory that has to be allocated
 * @bar: the BAR number corresponding to the allocated register space
 * @align: alignment size for the allocation region
 *
 * Invoke to allocate memory for the PCI EPF register space.
 */
void *pci_epf_alloc_space(struct pci_epf *epf, size_t size, enum pci_barno bar,
			  size_t align)
{
	void *space;
	struct device *dev = epf->epc->dev.parent;
	dma_addr_t phys_addr;

	if (size < 128)
		size = 128;

	if (align)
		size = ALIGN(size, align);
	else
		size = roundup_pow_of_two(size);

	space = dma_alloc_coherent(dev, size, &phys_addr, GFP_KERNEL);
	if (!space) {
		dev_err(dev, "failed to allocate mem space\n");
		return NULL;
	}

	epf->bar[bar].phys_addr = phys_addr;
	epf->bar[bar].size = size;
	epf->bar[bar].barno = bar;
	epf->bar[bar].flags = PCI_BASE_ADDRESS_SPACE_MEMORY;

	return space;
}
EXPORT_SYMBOL_GPL(pci_epf_alloc_space);

/**
 * pci_epf_unregister_driver() - unregister the PCI EPF driver
 * @driver: the PCI EPF driver that has to be unregistered
 *
 * Invoke to unregister the PCI EPF driver.
 */
void pci_epf_unregister_driver(struct pci_epf_driver *driver)
{
	struct config_group *group;

	list_for_each_entry(group, &driver->epf_group, group_entry)
		pci_ep_cfs_remove_epf_group(group);
	list_del(&driver->epf_group);
	driver_unregister(&driver->driver);
}
EXPORT_SYMBOL_GPL(pci_epf_unregister_driver);

/**
 * __pci_epf_register_driver() - register a new PCI EPF driver
 * @driver: structure representing PCI EPF driver
 * @owner: the owner of the module that registers the PCI EPF driver
 *
 * Invoke to register a new PCI EPF driver.
 */
int __pci_epf_register_driver(struct pci_epf_driver *driver,
			      struct module *owner)
{
	int ret;
	struct config_group *group;
	const struct pci_epf_device_id *id;

	if (!driver->ops)
		return -EINVAL;

	if (!driver->ops->bind || !driver->ops->unbind)
		return -EINVAL;

	driver->driver.bus = &pci_epf_bus_type;
	driver->driver.owner = owner;

	ret = driver_register(&driver->driver);
	if (ret)
		return ret;

	INIT_LIST_HEAD(&driver->epf_group);

	id = driver->id_table;
	while (id->name[0]) {
		group = pci_ep_cfs_add_epf_group(id->name);
		list_add_tail(&group->group_entry, &driver->epf_group);
		id++;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(__pci_epf_register_driver);

/**
 * pci_epf_destroy() - destroy the created PCI EPF device
 * @epf: the PCI EPF device that has to be destroyed.
 *
 * Invoke to destroy the PCI EPF device created by invoking pci_epf_create().
 */
void pci_epf_destroy(struct pci_epf *epf)
{
	device_unregister(&epf->dev);
}
EXPORT_SYMBOL_GPL(pci_epf_destroy);

/**
 * pci_epf_create() - create a new PCI EPF device
 * @name: the name of the PCI EPF device. This name will be used to bind the
 *	  the EPF device to a EPF driver
 *
 * Invoke to create a new PCI EPF device by providing the name of the function
 * device.
 */
struct pci_epf *pci_epf_create(const char *name)
{
	int ret;
	struct pci_epf *epf;
	struct device *dev;
	char *func_name;
	char *buf;

	epf = kzalloc(sizeof(*epf), GFP_KERNEL);
	if (!epf) {
		ret = -ENOMEM;
		goto err_ret;
	}

	buf = kstrdup(name, GFP_KERNEL);
	if (!buf) {
		ret = -ENOMEM;
		goto free_epf;
	}

	func_name = buf;
	buf = strchrnul(buf, '.');
	*buf = '\0';

	epf->name = kstrdup(func_name, GFP_KERNEL);
	if (!epf->name) {
		ret = -ENOMEM;
		goto free_func_name;
	}

	dev = &epf->dev;
	device_initialize(dev);
	dev->bus = &pci_epf_bus_type;
	dev->type = &pci_epf_type;
	mutex_init(&epf->lock);

	ret = dev_set_name(dev, "%s", name);
	if (ret)
		goto put_dev;

	ret = device_add(dev);
	if (ret)
		goto put_dev;

	kfree(func_name);
	return epf;

put_dev:
	put_device(dev);
	kfree(epf->name);

free_func_name:
	kfree(func_name);

free_epf:
	kfree(epf);

err_ret:
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(pci_epf_create);

const struct pci_epf_device_id *
pci_epf_match_device(const struct pci_epf_device_id *id, struct pci_epf *epf)
{
	if (!id || !epf)
		return NULL;

	while (*id->name) {
		if (strcmp(epf->name, id->name) == 0)
			return id;
		id++;
	}

	return NULL;
}
EXPORT_SYMBOL_GPL(pci_epf_match_device);

static void pci_epf_dev_release(struct device *dev)
{
	struct pci_epf *epf = to_pci_epf(dev);

	kfree(epf->name);
	kfree(epf);
}

static const struct device_type pci_epf_type = {
	.release	= pci_epf_dev_release,
};

static int
pci_epf_match_id(const struct pci_epf_device_id *id, const struct pci_epf *epf)
{
	while (id->name[0]) {
		if (strcmp(epf->name, id->name) == 0)
			return true;
		id++;
	}

	return false;
}

static int pci_epf_device_match(struct device *dev, struct device_driver *drv)
{
	struct pci_epf *epf = to_pci_epf(dev);
	struct pci_epf_driver *driver = to_pci_epf_driver(drv);

	if (driver->id_table)
		return pci_epf_match_id(driver->id_table, epf);

	return !strcmp(epf->name, drv->name);
}

static int pci_epf_device_probe(struct device *dev)
{
	struct pci_epf *epf = to_pci_epf(dev);
	struct pci_epf_driver *driver = to_pci_epf_driver(dev->driver);

	if (!driver->probe)
		return -ENODEV;

	epf->driver = driver;

	return driver->probe(epf);
}

static int pci_epf_device_remove(struct device *dev)
{
	int ret = 0;
	struct pci_epf *epf = to_pci_epf(dev);
	struct pci_epf_driver *driver = to_pci_epf_driver(dev->driver);

	if (driver->remove)
		ret = driver->remove(epf);
	epf->driver = NULL;

	return ret;
}

static struct bus_type pci_epf_bus_type = {
	.name		= "pci-epf",
	.match		= pci_epf_device_match,
	.probe		= pci_epf_device_probe,
	.remove		= pci_epf_device_remove,
};

static int __init pci_epf_init(void)
{
	int ret;

	ret = bus_register(&pci_epf_bus_type);
	if (ret) {
		pr_err("failed to register pci epf bus --> %d\n", ret);
		return ret;
	}

	return 0;
}
module_init(pci_epf_init);

static void __exit pci_epf_exit(void)
{
	bus_unregister(&pci_epf_bus_type);
}
module_exit(pci_epf_exit);

MODULE_DESCRIPTION("PCI EPF Library");
MODULE_AUTHOR("Kishon Vijay Abraham I <kishon@ti.com>");
MODULE_LICENSE("GPL v2");
