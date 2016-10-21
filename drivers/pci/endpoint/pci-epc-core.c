/**
 * pci-epc-core.c - PCI Endpoint *Controller* (EPC) library
 *
 * Copyright (C) 2016 Texas Instruments
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
#include <linux/slab.h>
#include <linux/module.h>

#include <linux/pci-epc.h>
#include <linux/pci-epf.h>

static struct class *pci_epc_class;

static void devm_pci_epc_release(struct device *dev, void *res)
{
	struct pci_epc *epc = *(struct pci_epc **)res;

	pci_epc_destroy(epc);
}

static int devm_pci_epc_match(struct device *dev, void *res, void *match_data)
{
	struct pci_epc **epc = res;

	return *epc == match_data;
}

/**
 * pci_epc_unbind_epf() - unbind the endpoint function and endpoint controller
 * @epf: the endpoint function which has requested for unbinding
 *
 * Invoke to unbind the endpoint function and endpoint controller
 */
void pci_epc_unbind_epf(struct pci_epf *epf)
{
	struct pci_epc *epc = epf->epc;

	pci_epf_unbind(epf);
	module_put(epc->ops->owner);
	epf->epc = NULL;
	epc->epf = NULL;
}
EXPORT_SYMBOL_GPL(pci_epc_unbind_epf);

/**
 * pci_epc_bind_epf() - bind the endpoint function with an endpoint controller
 * @epf: endpoint function which has to be bound to an endpoint controller
 *
 * Invoke to bind the endpoint function with an endpoint controller based on
 * pci_epc_name.
 */
int pci_epc_bind_epf(struct pci_epf *epf)
{
	int ret = -EINVAL;
	struct pci_epc *epc;
	struct device *dev;
	struct class_dev_iter iter;

	class_dev_iter_init(&iter, pci_epc_class, NULL, NULL);
	while ((dev = class_dev_iter_next(&iter))) {
		if (strcmp(epf->pci_epc_name, dev_name(dev)))
			continue;

		epc = to_pci_epc(dev);
		if (epc->epf) {
			ret = -EBUSY;
			goto err;
		}

		if (!try_module_get(epc->ops->owner)) {
			ret = -EINVAL;
			goto err;
		}

		epf->epc = epc;
		epc->epf = epf;

		dma_set_coherent_mask(&epf->dev, epc->dev.coherent_dma_mask);
		epf->dev.dma_mask = epc->dev.dma_mask;

		pci_epf_bind(epf);
		class_dev_iter_exit(&iter);
		return 0;
	}

err:
	class_dev_iter_exit(&iter);
	return ret;
}
EXPORT_SYMBOL_GPL(pci_epc_bind_epf);

/**
 * pci_epc_stop() - stop the PCI link
 * @epc: the link of the EPC device that has to be stopped
 *
 * Invoke to stop the PCI link
 */
void pci_epc_stop(struct pci_epc *epc)
{
	unsigned long flags;

	if (IS_ERR(epc) || !epc->ops->stop)
		return;

	spin_lock_irqsave(&epc->lock, flags);
	epc->ops->stop(epc);
	spin_unlock_irqrestore(&epc->lock, flags);
}
EXPORT_SYMBOL_GPL(pci_epc_stop);

/**
 * pci_epc_start() - start the PCI link
 * @epc: the link of *this* EPC device has to be started
 *
 * Invoke to start the PCI link
 */
int pci_epc_start(struct pci_epc *epc)
{
	int ret;
	unsigned long flags;

	if (IS_ERR(epc))
		return -EINVAL;

	if (!epc->ops->start)
		return 0;

	spin_lock_irqsave(&epc->lock, flags);
	ret = epc->ops->start(epc);
	spin_unlock_irqrestore(&epc->lock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(pci_epc_start);

/**
 * pci_epc_raise_irq() - interrupt the host system
 * @epc: the EPC device which has to interrupt the host
 * @type: specify the type of interrupt; legacy or MSI
 * @interrupt_num: the MSI interrupt number
 *
 * Invoke to raise an MSI or legacy interrupt
 */
int pci_epc_raise_irq(struct pci_epc *epc, enum pci_epc_irq_type type,
		      u8 interrupt_num)
{
	int ret;
	unsigned long flags;

	if (IS_ERR(epc))
		return -EINVAL;

	if (!epc->ops->raise_irq)
		return 0;

	spin_lock_irqsave(&epc->lock, flags);
	ret = epc->ops->raise_irq(epc, type, interrupt_num);
	spin_unlock_irqrestore(&epc->lock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(pci_epc_raise_irq);

/**
 * pci_epc_get_msi() - get the number of MSI interrupt numbers allocated
 * @epc: the EPC device to which MSI interrupts was requested
 *
 * Invoke to get the number of MSI interrupts allocated by the RC
 */
int pci_epc_get_msi(struct pci_epc *epc)
{
	int interrupt;
	unsigned long flags;

	if (IS_ERR(epc))
		return 0;

	if (!epc->ops->get_msi)
		return 0;

	spin_lock_irqsave(&epc->lock, flags);
	interrupt = epc->ops->get_msi(epc);
	spin_unlock_irqrestore(&epc->lock, flags);

	if (interrupt < 0)
		return 0;

	interrupt = 1 << interrupt;

	return interrupt;
}
EXPORT_SYMBOL_GPL(pci_epc_get_msi);

/**
 * pci_epc_set_msi() - set the number of MSI interrupt numbers required
 * @epc: the EPC device on which MSI has to be configured
 * @interrupts: number of MSI interrupts required by the EPF
 *
 * Invoke to set the required number of MSI interrupts.
 */
int pci_epc_set_msi(struct pci_epc *epc, u8 interrupts)
{
	int ret;
	u8 encode_int;
	unsigned long flags;

	if (IS_ERR(epc))
		return -EINVAL;

	if (!epc->ops->set_msi)
		return 0;

	encode_int = order_base_2(interrupts);

	spin_lock_irqsave(&epc->lock, flags);
	ret = epc->ops->set_msi(epc, encode_int);
	spin_unlock_irqrestore(&epc->lock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(pci_epc_set_msi);

/**
 * pci_epc_unmap_addr() - unmap cpu address from pci address
 * @epc: the EPC device on which address is allocated
 * @phys_addr: physical address of the local system
 *
 * Invoke to unmap the cpu address from pci address.
 */
void pci_epc_unmap_addr(struct pci_epc *epc, phys_addr_t phys_addr)
{
	unsigned long flags;

	if (IS_ERR(epc))
		return;

	if (!epc->ops->unmap_addr)
		return;

	spin_lock_irqsave(&epc->lock, flags);
	epc->ops->unmap_addr(epc, phys_addr);
	spin_unlock_irqrestore(&epc->lock, flags);
}
EXPORT_SYMBOL_GPL(pci_epc_unmap_addr);

/**
 * pci_epc_map_addr() - map cpu address to pci address
 * @epc: the EPC device on which address is allocated
 * @phys_addr: physical address of the local system
 * @pci_addr: pci address to which the physical address should be mapped
 * @size: the size of the allocation
 *
 * Invoke to map cpu address with pci address.
 */
int pci_epc_map_addr(struct pci_epc *epc, phys_addr_t phys_addr,
		     u64 pci_addr, size_t size)
{
	int ret;
	unsigned long flags;

	if (IS_ERR(epc))
		return -EINVAL;

	if (!epc->ops->map_addr)
		return 0;

	spin_lock_irqsave(&epc->lock, flags);
	ret = epc->ops->map_addr(epc, phys_addr, pci_addr, size);
	spin_unlock_irqrestore(&epc->lock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(pci_epc_map_addr);

/**
 * pci_epc_clear_bar() - reset the BAR
 * @epc: the EPC device for which the BAR has to be cleared
 * @bar: the bar number that has to be reset
 *
 * Invoke to reset the BAR of the endpoint device.
 */
void pci_epc_clear_bar(struct pci_epc *epc, int bar)
{
	unsigned long flags;

	if (IS_ERR(epc))
		return;

	if (!epc->ops->clear_bar)
		return;

	spin_lock_irqsave(&epc->lock, flags);
	epc->ops->clear_bar(epc, bar);
	spin_unlock_irqrestore(&epc->lock, flags);
}
EXPORT_SYMBOL_GPL(pci_epc_clear_bar);

/**
 * pci_epc_set_bar() - configure BAR in order for host to assign PCI addr space
 * @epc: the EPC device on which BAR has to be configured
 * @bar: the bar number that has to be configured
 * @size: the size of the addr space
 * @flags: specify memory allocation/io allocation/32bit address/64 bit address
 *
 * Invoke to configure the BAR of the endpoint device.
 */
int pci_epc_set_bar(struct pci_epc *epc, enum pci_barno bar,
		    dma_addr_t bar_phys, size_t size, int flags)
{
	int ret;
	unsigned long irq_flags;

	if (IS_ERR(epc))
		return -EINVAL;

	if (!epc->ops->set_bar)
		return 0;

	spin_lock_irqsave(&epc->lock, irq_flags);
	ret = epc->ops->set_bar(epc, bar, bar_phys, size, flags);
	spin_unlock_irqrestore(&epc->lock, irq_flags);

	return ret;
}
EXPORT_SYMBOL_GPL(pci_epc_set_bar);

/**
 * pci_epc_write_header() - write standard configuration header
 * @epc: the EPC device to which the configuration header should be written
 * @header: standard configuration header fields
 *
 * Invoke to write the configuration header to the endpoint controller. Every
 * endpoint controller will have a dedicated location to which the standard
 * configuration header would be written. The callback function should write
 * the header fields to this dedicated location.
 */
int pci_epc_write_header(struct pci_epc *epc, struct pci_epf_header *header)
{
	int ret;
	unsigned long flags;

	if (IS_ERR(epc))
		return -EINVAL;

	if (!epc->ops->write_header)
		return 0;

	spin_lock_irqsave(&epc->lock, flags);
	ret = epc->ops->write_header(epc, header);
	spin_unlock_irqrestore(&epc->lock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(pci_epc_write_header);

/**
 * pci_epc_destroy() - destroy the EPC device
 * @epc: the EPC device that has to be destroyed
 *
 * Invoke to destroy the PCI EPC device
 */
void pci_epc_destroy(struct pci_epc *epc)
{
	device_unregister(&epc->dev);
	kfree(epc);
}
EXPORT_SYMBOL_GPL(pci_epc_destroy);

/**
 * devm_pci_epc_destroy() - destroy the EPC device
 * @dev: device that wants to destroy the EPC
 * @epc: the EPC device that has to be destroyed
 *
 * Invoke to destroy the devres associated with this
 * pci_epc and destroy the EPC device.
 */
void devm_pci_epc_destroy(struct device *dev, struct pci_epc *epc)
{
	int r;

	r = devres_destroy(dev, devm_pci_epc_release, devm_pci_epc_match,
			   epc);
	dev_WARN_ONCE(dev, r, "couldn't find PCI EPC resource\n");
}
EXPORT_SYMBOL_GPL(devm_pci_epc_destroy);

/**
 * __pci_epc_create() - create a new endpoint controller (EPC) device
 * @dev: device that is creating the new EPC
 * @ops: function pointers for performing EPC operations
 * @owner: the owner of the module that creates the EPC device
 *
 * Invoke to create a new EPC device and add it to pci_epc class.
 */
struct pci_epc *
__pci_epc_create(struct device *dev, const struct pci_epc_ops *ops,
		 struct module *owner)
{
	int ret;
	struct pci_epc *epc;

	if (WARN_ON(!dev)) {
		ret = -EINVAL;
		goto err_ret;
	}

	epc = kzalloc(sizeof(*epc), GFP_KERNEL);
	if (!epc) {
		ret = -ENOMEM;
		goto err_ret;
	}

	spin_lock_init(&epc->lock);

	device_initialize(&epc->dev);
	dma_set_coherent_mask(&epc->dev, dev->coherent_dma_mask);
	epc->dev.class = pci_epc_class;
	epc->dev.dma_mask = dev->dma_mask;
	epc->ops = ops;

	ret = dev_set_name(&epc->dev, "%s", dev_name(dev));
	if (ret)
		goto put_dev;

	ret = device_add(&epc->dev);
	if (ret)
		goto put_dev;

	return epc;

put_dev:
	put_device(&epc->dev);
	kfree(epc);

err_ret:
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(__pci_epc_create);

/**
 * __devm_pci_epc_create() - create a new endpoint controller (EPC) device
 * @dev: device that is creating the new EPC
 * @ops: function pointers for performing EPC operations
 * @owner: the owner of the module that creates the EPC device
 *
 * Invoke to create a new EPC device and add it to pci_epc class.
 * While at that, it also associates the device with the pci_epc using devres.
 * On driver detach, release function is invoked on the devres data,
 * then, devres data is freed.
 */
struct pci_epc *
__devm_pci_epc_create(struct device *dev, const struct pci_epc_ops *ops,
		      struct module *owner)
{
	struct pci_epc **ptr, *epc;

	ptr = devres_alloc(devm_pci_epc_release, sizeof(*ptr), GFP_KERNEL);
	if (!ptr)
		return ERR_PTR(-ENOMEM);

	epc = __pci_epc_create(dev, ops, owner);
	if (!IS_ERR(epc)) {
		*ptr = epc;
		devres_add(dev, ptr);
	} else {
		devres_free(ptr);
	}

	return epc;
}
EXPORT_SYMBOL_GPL(__devm_pci_epc_create);

static int __init pci_epc_init(void)
{
	pci_epc_class = class_create(THIS_MODULE, "pci_epc");
	if (IS_ERR(pci_epc_class)) {
		pr_err("failed to create pci epc class --> %ld\n",
		       PTR_ERR(pci_epc_class));
		return PTR_ERR(pci_epc_class);
	}

	return 0;
}
module_init(pci_epc_init);

static void __exit pci_epc_exit(void)
{
	class_destroy(pci_epc_class);
}
module_exit(pci_epc_exit);

MODULE_DESCRIPTION("PCI EPC Library");
MODULE_AUTHOR("Kishon Vijay Abraham I <kishon@ti.com>");
MODULE_LICENSE("GPL v2");
