// SPDX-License-Identifier: GPL-2.0
/*
 * UIO driver for Inter-VM shared memory PCI device
 *
 * Copyright (c) Siemens AG, 2019
 *
 * Authors:
 *  Jan Kiszka <jan.kiszka@siemens.com>
 */

#include <linux/ivshmem.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/uio_driver.h>

#define DRV_NAME "uio_ivshmem"

struct ivshm_dev {
	struct uio_info info;
	struct pci_dev *pdev;
	struct ivshm_regs __iomem *regs;
	int vectors;
};

static irqreturn_t ivshm_irq_handler(int irq, void *dev_id)
{
	struct ivshm_dev *ivshm_dev = (struct ivshm_dev *)dev_id;

	/* nothing else to do, we configured one-shot interrupt mode */
	uio_event_notify(&ivshm_dev->info);

	return IRQ_HANDLED;
}

static u64 get_config_qword(struct pci_dev *pdev, unsigned int pos)
{
	u32 lo, hi;

	pci_read_config_dword(pdev, pos, &lo);
	pci_read_config_dword(pdev, pos + 4, &hi);
	return lo | ((u64)hi << 32);
}

static int ivshm_release(struct uio_info *info, struct inode *inode)
{
	struct ivshm_dev *ivshm_dev =
		container_of(info, struct ivshm_dev, info);

	writel(0, &ivshm_dev->regs->state);
	return 0;
}

static int ivshm_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	resource_size_t rw_section_sz, output_section_sz;
	struct ivshm_dev *ivshm_dev;
	phys_addr_t section_addr;
	int err, vendor_cap, i;
	unsigned int cap_pos;
	struct uio_mem *mem;
	char *device_name;
	u32 dword;

	ivshm_dev = devm_kzalloc(&pdev->dev, sizeof(struct ivshm_dev),
				 GFP_KERNEL);
	if (!ivshm_dev)
		return -ENOMEM;

	err = pcim_enable_device(pdev);
	if (err)
		return err;

	device_name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "%s[%s]", DRV_NAME,
				     dev_name(&pdev->dev));
	if (!device_name)
		return -ENOMEM;

	ivshm_dev->info.name = device_name;
	ivshm_dev->info.version = "1";
	ivshm_dev->info.release = ivshm_release;

	err = pcim_iomap_regions(pdev, BIT(0), device_name);
	if (err)
		return err;
	ivshm_dev->regs = pcim_iomap_table(pdev)[0];

	mem = &ivshm_dev->info.mem[0];

	mem->name = "registers";
	mem->addr = pci_resource_start(pdev, 0);
	if (!mem->addr)
		return -ENODEV;
	mem->size = PAGE_ALIGN(pci_resource_len(pdev, 0));
	mem->memtype = UIO_MEM_PHYS;

	vendor_cap = pci_find_capability(pdev, PCI_CAP_ID_VNDR);
	if (vendor_cap < 0)
		return -ENODEV;

	if (pci_resource_len(pdev, 2) > 0) {
		section_addr = pci_resource_start(pdev, 2);
	} else {
		cap_pos = vendor_cap + IVSHM_CFG_ADDRESS;
		section_addr = get_config_qword(pdev, cap_pos);
	}

	mem++;
	mem->name = "state_table";
	mem->addr = section_addr;
	cap_pos = vendor_cap + IVSHM_CFG_STATE_TAB_SZ;
	pci_read_config_dword(pdev, cap_pos, &dword);
	mem->size = dword;
	mem->memtype = UIO_MEM_IOVA;
	mem->readonly = true;
	if (!devm_request_mem_region(&pdev->dev, mem->addr, mem->size,
				     device_name))
		return -EBUSY;
	dev_info(&pdev->dev, "%s at %pa, size %pa\n", mem->name, &mem->addr,
		 &mem->size);

	cap_pos = vendor_cap + IVSHM_CFG_RW_SECTION_SZ;
	rw_section_sz = get_config_qword(pdev, cap_pos);
	if (rw_section_sz > 0) {
		section_addr += mem->size;

		mem++;
		mem->name = "rw_section";
		mem->addr = section_addr;
		mem->size = rw_section_sz;
		mem->memtype = UIO_MEM_IOVA;
		if (!devm_request_mem_region(&pdev->dev, mem->addr, mem->size,
					     device_name))
			return -EBUSY;
		dev_info(&pdev->dev, "%s at %pa, size %pa\n", mem->name,
			 &mem->addr, &mem->size);
	}

	cap_pos = vendor_cap + IVSHM_CFG_OUTPUT_SECTION_SZ;
	output_section_sz = get_config_qword(pdev, cap_pos);
	if (output_section_sz > 0) {
		section_addr += mem->size;

		mem++;
		mem->name = "input_sections";
		mem->addr = section_addr;
		mem->size =
			readl(&ivshm_dev->regs->max_peers) * output_section_sz;
		mem->memtype = UIO_MEM_IOVA;
		mem->readonly = true;
		if (!devm_request_mem_region(&pdev->dev, mem->addr, mem->size,
					     device_name))
			return -EBUSY;
		dev_info(&pdev->dev, "%s at %pa, size %pa\n", mem->name,
			 &mem->addr, &mem->size);

		mem++;
		mem->name = "output_section";
		mem->addr = section_addr +
			readl(&ivshm_dev->regs->id) * output_section_sz;
		mem->size = output_section_sz;
		mem->memtype = UIO_MEM_IOVA;
		dev_info(&pdev->dev, "%s at %pa, size %pa\n", mem->name,
			 &mem->addr, &mem->size);
	}

	pci_write_config_byte(pdev, vendor_cap + IVSHM_CFG_PRIV_CNTL,
			      IVSHM_PRIV_CNTL_ONESHOT_INT);

	/*
	 * Grab all vectors although we can only coalesce them into a single
	 * notifier. This avoids missing any event.
	 */
	ivshm_dev->vectors = pci_msix_vec_count(pdev);
	if (ivshm_dev->vectors < 0)
		ivshm_dev->vectors = 1;

	err = pci_alloc_irq_vectors(pdev, ivshm_dev->vectors,
				    ivshm_dev->vectors,
				    PCI_IRQ_LEGACY | PCI_IRQ_MSIX);
	if (err < 0)
		return err;

	for (i = 0; i < ivshm_dev->vectors; i++) {
		err = request_irq(pci_irq_vector(pdev, i), ivshm_irq_handler,
				  IRQF_SHARED, ivshm_dev->info.name, ivshm_dev);
		if (err)
			goto error;
	}

	ivshm_dev->info.irq = UIO_IRQ_CUSTOM;

	err = uio_register_device(&pdev->dev, &ivshm_dev->info);
	if (err)
		goto error;

	pci_set_master(pdev);

	pci_set_drvdata(pdev, ivshm_dev);

	return 0;

error:
	while (--i > 0)
		free_irq(pci_irq_vector(pdev, i), ivshm_dev);
	pci_free_irq_vectors(pdev);
	return err;
}

static void ivshm_remove(struct pci_dev *pdev)
{
	struct ivshm_dev *ivshm_dev = pci_get_drvdata(pdev);
	int i;

	writel(0, &ivshm_dev->regs->int_control);
	pci_clear_master(pdev);

	uio_unregister_device(&ivshm_dev->info);

	for (i = 0; i < ivshm_dev->vectors; i++)
		free_irq(pci_irq_vector(pdev, i), ivshm_dev);

	pci_free_irq_vectors(pdev);
}

static const struct pci_device_id ivshm_device_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_SIEMENS, PCI_DEVICE_ID_IVSHMEM),
	  (PCI_CLASS_OTHERS << 16) | IVSHM_PROTO_UNDEFINED, 0xffffff },
	{ 0 }
};
MODULE_DEVICE_TABLE(pci, ivshm_device_id_table);

static struct pci_driver uio_ivshm_driver = {
	.name = DRV_NAME,
	.id_table = ivshm_device_id_table,
	.probe = ivshm_probe,
	.remove = ivshm_remove,
};
module_pci_driver(uio_ivshm_driver);

MODULE_AUTHOR("Jan Kiszka <jan.kiszka@siemens.com>");
MODULE_LICENSE("GPL v2");
