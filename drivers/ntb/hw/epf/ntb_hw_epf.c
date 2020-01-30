// SPDX-License-Identifier: GPL-2.0
/**
 * Host side endpoint driver to implement Non-Transparent Bridge functionality
 *
 * Copyright (C) 2019 Texas Instruments
 * Author: Kishon Vijay Abraham I <kishon@ti.com>
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/ntb.h>

#define NTB_EPF_COMMAND		0x0
#define CMD_CONFIGURE_DOORBELL	1
#define CMD_CONFIGURE_MW	2
#define CMD_LINK_UP		3

#define NTB_EPF_ARGUMENT	0x4

#define NTB_EPF_STATUS		0x8
#define COMMAND_STATUS_OK	BIT(0)
#define COMMAND_STATUS_ERROR	BIT(1)
#define LINK_STATUS_UP		BIT(2)

#define NTB_EPF_TOPOLOGY	0xc
#define NTB_EPF_ADDR		0x10
#define NTB_EPF_SIZE		0x18
#define NTB_EPF_MW1_OFFSET	0x1c
#define NTB_EPF_MW_COUNT	0x20
#define NTB_EPF_SPAD_OFFSET	0x24
#define NTB_EPF_SPAD_COUNT	0x28
#define NTB_EPF_DB_ENTRY_SIZE	0x2c
#define NTB_EPF_DB_DATA(n)	(0x30 + (n) * 4)

#define NTB_MIN_DB_COUNT	2
#define NTB_MAX_DB_COUNT	32
#define NTB_MW_OFFSET		2

#define NTB_COMMAND_TIMEOUT	1000 /* 1 Sec */

enum pci_barno {
	BAR_0,
	BAR_1,
	BAR_2,
	BAR_3,
	BAR_4,
	BAR_5,
};

struct ntb_epf_dev {
	struct ntb_dev ntb;

	enum pci_barno ctrl_reg_bar;
	enum pci_barno peer_spad_reg_bar;
	enum pci_barno db_reg_bar;

	unsigned int mw_count;
	unsigned int spad_count;
	unsigned int db_count;

	void __iomem *ctrl_reg;
	void __iomem *db_reg;
	void __iomem *peer_spad_reg;

	unsigned int self_spad;
	unsigned int peer_spad;

	int db_val;
	u64 db_valid_mask;
};

#define ntb_ndev(__ntb) container_of(__ntb, struct ntb_epf_dev, ntb)

struct ntb_epf_data {
	/* BAR that contains both control region and self spad region */
	enum pci_barno ctrl_reg_bar;
	/* BAR that contains peer spad region */
	enum pci_barno peer_spad_reg_bar;
	/* BAR that contains Doorbell region and Memory window '1' */
	enum pci_barno db_reg_bar;
};

static inline u32 ntb_epf_ctrl_readl(struct ntb_epf_dev *ndev, u32 offset)
{
	return readl(ndev->ctrl_reg + offset);
}

static inline void ntb_epf_ctrl_writel(struct ntb_epf_dev *ndev, u32 offset,
				       u32 val)
{
	return writel(val, ndev->ctrl_reg + offset);
}

static int ntb_epf_send_command(struct ntb_epf_dev *ndev, u32 command,
				u32 argument)
{
	ktime_t timeout;
	bool timedout;
	int ret = 0;
	u32 status;

	ntb_epf_ctrl_writel(ndev, NTB_EPF_ARGUMENT, argument);
	ntb_epf_ctrl_writel(ndev, NTB_EPF_COMMAND, command);

	timeout = ktime_add_ms(ktime_get(), NTB_COMMAND_TIMEOUT);
	while (1) {
		timedout = ktime_after(ktime_get(), timeout);
		status = ntb_epf_ctrl_readl(ndev, NTB_EPF_STATUS);

		if (status & COMMAND_STATUS_ERROR) {
			ret = -EINVAL;
			break;
		}

		if (status & COMMAND_STATUS_OK)
			break;

		if (WARN_ON(timedout)) {
			ret = -ETIMEDOUT;
			break;
		}

		usleep_range(5, 10);
	}

	status &= ~(COMMAND_STATUS_ERROR | COMMAND_STATUS_OK);
	ntb_epf_ctrl_writel(ndev, NTB_EPF_STATUS, status);

	return ret;
}

static int ntb_epf_mw_to_bar(struct ntb_epf_dev *ndev, int idx)
{
	if (idx < 0 || idx > ndev->mw_count)
		return -EINVAL;

	return idx + 2;
}

static int ntb_epf_mw_count(struct ntb_dev *ntb, int pidx)
{
	if (pidx != NTB_DEF_PEER_IDX)
		return -EINVAL;

	return ntb_ndev(ntb)->mw_count;
}

static int ntb_epf_mw_get_align(struct ntb_dev *ntb, int pidx, int idx,
				resource_size_t *addr_align,
				resource_size_t *size_align,
				resource_size_t *size_max)
{
	struct ntb_epf_dev *ndev = ntb_ndev(ntb);
	int bar;

	if (pidx != NTB_DEF_PEER_IDX)
		return -EINVAL;

	bar = ntb_epf_mw_to_bar(ndev, idx);
	if (bar < 0)
		return bar;

	if (addr_align)
		*addr_align = SZ_4K;

	if (size_align)
		*size_align = 1;

	if (size_max)
		*size_max = pci_resource_len(ndev->ntb.pdev, bar);

	return 0;
}

static u64 ntb_epf_link_is_up(struct ntb_dev *ntb,
			      enum ntb_speed *speed,
			      enum ntb_width *width)
{
	struct ntb_epf_dev *ndev = ntb_ndev(ntb);
	u32 status;

	status = ntb_epf_ctrl_readl(ndev, NTB_EPF_STATUS);

	return !!(status & LINK_STATUS_UP);
}

static u32 ntb_epf_spad_read(struct ntb_dev *ntb, int idx)
{
	struct ntb_epf_dev *ndev = ntb_ndev(ntb);
	u32 offset;

	if (idx < 0 || idx >= ndev->spad_count)
		return 0;

	offset = ntb_epf_ctrl_readl(ndev, NTB_EPF_SPAD_OFFSET);
	offset += (idx << 2);

	return ntb_epf_ctrl_readl(ndev, offset);
}

static int ntb_epf_spad_write(struct ntb_dev *ntb,
			      int idx, u32 val)
{
	struct ntb_epf_dev *ndev = ntb_ndev(ntb);
	u32 offset;

	if (idx < 0 || idx >= ndev->spad_count)
		return -EINVAL;

	offset = ntb_epf_ctrl_readl(ndev, NTB_EPF_SPAD_OFFSET);
	offset += (idx << 2);
	ntb_epf_ctrl_writel(ndev, offset, val);

	return 0;
}

static u32 ntb_epf_peer_spad_read(struct ntb_dev *ntb, int pidx, int idx)
{
	struct ntb_epf_dev *ndev = ntb_ndev(ntb);
	u32 offset;

	if (idx < 0 || idx >= ndev->spad_count)
		return -EINVAL;

	offset = (idx << 2);
	return readl(ndev->peer_spad_reg + offset);
}

static int ntb_epf_peer_spad_write(struct ntb_dev *ntb, int pidx,
				   int idx, u32 val)
{
	struct ntb_epf_dev *ndev = ntb_ndev(ntb);
	u32 offset;

	if (idx < 0 || idx >= ndev->spad_count)
		return -EINVAL;

	offset = (idx << 2);
	writel(val, ndev->peer_spad_reg + offset);

	return 0;
}

static int ntb_epf_link_enable(struct ntb_dev *ntb,
			       enum ntb_speed max_speed,
			       enum ntb_width max_width)
{
	struct ntb_epf_dev *ndev = ntb_ndev(ntb);
	struct device *dev = &ntb->pdev->dev;
	int ret;

	ret = ntb_epf_send_command(ndev, CMD_LINK_UP, 0);
	if (ret) {
		dev_err(dev, "Fail to enable link\n");
		return ret;
	}

	return 0;
}

static int ntb_epf_link_disable(struct ntb_dev *ntb)
{
	return 0;
}

static irqreturn_t ndev_vec_isr(int irq, void *dev)
{
	struct ntb_epf_dev *ndev = dev;
	int irq_no;

	irq_no = irq - ndev->ntb.pdev->irq;
	ndev->db_val = irq_no + 1;

	if (irq_no == 0)
		ntb_link_event(&ndev->ntb);
	else
		ntb_db_event(&ndev->ntb, irq_no);

	return IRQ_HANDLED;
}

static int ntb_epf_init_isr(struct ntb_epf_dev *ndev, int msi_min, int msi_max)
{
	struct pci_dev *pdev = ndev->ntb.pdev;
	struct device *dev = &pdev->dev;
	int irq;
	int ret;
	int i;

	irq = pci_alloc_irq_vectors(pdev, msi_min, msi_max, PCI_IRQ_MSI);
	if (irq < 0) {
		dev_err(dev, "Failed to get MSI interrupts\n");
		return irq;
	}

	for (i = 0; i < irq; i++) {
		ret = devm_request_irq(&pdev->dev, pci_irq_vector(pdev, i),
				       ndev_vec_isr, IRQF_SHARED, "ntb_epf",
				       ndev);
		if (ret) {
			dev_err(dev, "Failed to request irq\n");
			goto err_request_irq;
		}
	}

	ndev->db_count = irq;

	ret = ntb_epf_send_command(ndev, CMD_CONFIGURE_DOORBELL, irq);
	if (ret) {
		dev_err(dev, "Failed to configure doorbell\n");
		goto err_request_irq;
	}

	return 0;

err_request_irq:
	pci_free_irq_vectors(pdev);

	return ret;
}

static int ntb_epf_peer_mw_count(struct ntb_dev *ntb)
{
	return ntb_ndev(ntb)->mw_count;
}

static int ntb_epf_spad_count(struct ntb_dev *ntb)
{
	return ntb_ndev(ntb)->spad_count;
}

static u64 ntb_epf_db_valid_mask(struct ntb_dev *ntb)
{
	return ntb_ndev(ntb)->db_valid_mask;
}

static int ntb_epf_db_set_mask(struct ntb_dev *ntb, u64 db_bits)
{
	return 0;
}

static int ntb_epf_mw_set_trans(struct ntb_dev *ntb, int pidx, int idx,
				dma_addr_t addr, resource_size_t size)
{
	struct ntb_epf_dev *ndev = ntb_ndev(ntb);
	struct device *dev = &ntb->pdev->dev;
	resource_size_t mw_size;
	int bar;

	if (pidx != NTB_DEF_PEER_IDX)
		return -EINVAL;

	bar = idx + NTB_MW_OFFSET;

	mw_size = pci_resource_len(ntb->pdev, bar);

	if (size > mw_size) {
		dev_err(dev, "Size is greater than the MW size\n");
		return -EINVAL;
	}

	ntb_epf_ctrl_writel(ndev, NTB_EPF_ADDR, lower_32_bits(addr));
	ntb_epf_ctrl_writel(ndev, NTB_EPF_ADDR + 4, upper_32_bits(addr));
	ntb_epf_ctrl_writel(ndev, NTB_EPF_SIZE, size);
	ntb_epf_send_command(ndev, CMD_CONFIGURE_MW, idx);

	return 0;
}

static int ntb_epf_peer_mw_get_addr(struct ntb_dev *ntb, int idx,
				    phys_addr_t *base, resource_size_t *size)
{
	struct ntb_epf_dev *ndev = ntb_ndev(ntb);
	u32 offset = 0;
	int bar;

	if (idx == 0)
		offset = ntb_epf_ctrl_readl(ndev, NTB_EPF_MW1_OFFSET);

	bar = idx + NTB_MW_OFFSET;

	if (base)
		*base = pci_resource_start(ndev->ntb.pdev, bar) + offset;

	if (size)
		*size = pci_resource_len(ndev->ntb.pdev, bar) - offset;

	return 0;
}

static int ntb_epf_peer_db_set(struct ntb_dev *ntb, u64 db_bits)
{
	struct ntb_epf_dev *ndev = ntb_ndev(ntb);
	u32 interrupt_num = ffs(db_bits) + 1;
	u32 db_entry_size;
	u32 db_data;

	if (interrupt_num > ndev->db_count)
		return -EINVAL;

	db_entry_size = ntb_epf_ctrl_readl(ndev, NTB_EPF_DB_ENTRY_SIZE);

	db_data = readl(ndev->ctrl_reg + NTB_EPF_DB_DATA(interrupt_num));
	writel(db_data, ndev->db_reg + (db_entry_size * interrupt_num));

	return 0;
}

static u64 ntb_epf_db_read(struct ntb_dev *ntb)
{
	struct ntb_epf_dev *ndev = ntb_ndev(ntb);

	return ndev->db_val;
}

static int ntb_epf_db_clear_mask(struct ntb_dev *ntb, u64 db_bits)
{
	return 0;
}

static int ntb_epf_db_clear(struct ntb_dev *ntb, u64 db_bits)
{
	struct ntb_epf_dev *ndev = ntb_ndev(ntb);

	ndev->db_val = 0;

	return 0;
}

static const struct ntb_dev_ops ntb_epf_ops = {
	.mw_count		= ntb_epf_mw_count,
	.spad_count		= ntb_epf_spad_count,
	.peer_mw_count		= ntb_epf_peer_mw_count,
	.db_valid_mask		= ntb_epf_db_valid_mask,
	.db_set_mask		= ntb_epf_db_set_mask,
	.mw_set_trans		= ntb_epf_mw_set_trans,
	.peer_mw_get_addr	= ntb_epf_peer_mw_get_addr,
	.link_enable		= ntb_epf_link_enable,
	.spad_read		= ntb_epf_spad_read,
	.spad_write		= ntb_epf_spad_write,
	.peer_spad_read		= ntb_epf_peer_spad_read,
	.peer_spad_write	= ntb_epf_peer_spad_write,
	.peer_db_set		= ntb_epf_peer_db_set,
	.db_read		= ntb_epf_db_read,
	.mw_get_align		= ntb_epf_mw_get_align,
	.link_is_up		= ntb_epf_link_is_up,
	.db_clear_mask		= ntb_epf_db_clear_mask,
	.db_clear		= ntb_epf_db_clear,
	.link_disable		= ntb_epf_link_disable,
};

static inline void ntb_epf_init_struct(struct ntb_epf_dev *ndev,
				       struct pci_dev *pdev)
{
	ndev->ntb.pdev = pdev;
	ndev->ntb.topo = NTB_TOPO_NONE;
	ndev->ntb.ops = &ntb_epf_ops;
}

static int ntb_epf_init_dev(struct ntb_epf_dev *ndev)
{
	struct pci_dev *pdev = ndev->ntb.pdev;
	struct device *dev = &pdev->dev;
	int ret;

	ret = ntb_epf_init_isr(ndev, NTB_MIN_DB_COUNT, NTB_MAX_DB_COUNT);
	if (ret) {
		dev_err(dev, "Failed to init ISR\n");
		return ret;
	}

	ndev->db_valid_mask = BIT_ULL(ndev->db_count) - 1;
	ndev->mw_count = ntb_epf_ctrl_readl(ndev, NTB_EPF_MW_COUNT);
	ndev->spad_count = ntb_epf_ctrl_readl(ndev, NTB_EPF_SPAD_COUNT);

	return 0;
}

static int ntb_epf_init_pci(struct ntb_epf_dev *ndev,
			    struct pci_dev *pdev)
{
	struct device *dev = &pdev->dev;
	int ret;

	pci_set_drvdata(pdev, ndev);

	ret = pci_enable_device(pdev);
	if (ret) {
		dev_err(dev, "Cannot enable PCI device\n");
		goto err_pci_enable;
	}

	ret = pci_request_regions(pdev, "ntb");
	if (ret) {
		dev_err(dev, "Cannot obtain PCI resources\n");
		goto err_pci_regions;
	}

	pci_set_master(pdev);

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (ret) {
		ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
		if (ret) {
			dev_err(dev, "Cannot set DMA mask\n");
			goto err_dma_mask;
		}
		dev_warn(&pdev->dev, "Cannot DMA highmem\n");
	}

	ret = dma_coerce_mask_and_coherent(&ndev->ntb.dev,
					   dma_get_mask(&pdev->dev));
	if (ret) {
		dev_err(dev, "Cannot set DMA mask\n");
		goto err_dma_mask;
	}

	ndev->ctrl_reg = pci_iomap(pdev, 0, 0);
	if (!ndev->ctrl_reg) {
		ret = -EIO;
		goto err_dma_mask;
	}

	ndev->peer_spad_reg = pci_iomap(pdev, 1, 0);
	if (!ndev->peer_spad_reg) {
		ret = -EIO;
		goto err_dma_mask;
	}

	ndev->db_reg = pci_iomap(pdev, 2, 0);
	if (!ndev->db_reg) {
		ret = -EIO;
		goto err_dma_mask;
	}

	return 0;

err_dma_mask:
	pci_clear_master(pdev);

err_pci_regions:
	pci_disable_device(pdev);

err_pci_enable:
	pci_set_drvdata(pdev, NULL);

	return ret;
}

static void ntb_epf_deinit_pci(struct ntb_epf_dev *ndev)
{
	struct pci_dev *pdev = ndev->ntb.pdev;

	pci_iounmap(pdev, ndev->ctrl_reg);
	pci_iounmap(pdev, ndev->peer_spad_reg);
	pci_iounmap(pdev, ndev->db_reg);

	pci_clear_master(pdev);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
	pci_set_drvdata(pdev, NULL);
}

static int ntb_epf_pci_probe(struct pci_dev *pdev,
			     const struct pci_device_id *id)
{
	enum pci_barno peer_spad_reg_bar = BAR_1;
	enum pci_barno ctrl_reg_bar = BAR_0;
	enum pci_barno db_reg_bar = BAR_2;
	struct device *dev = &pdev->dev;
	struct ntb_epf_data *data;
	struct ntb_epf_dev *ndev;
	int ret;

	if (pci_is_bridge(pdev))
		return -ENODEV;

	ndev = devm_kzalloc(dev, sizeof(*ndev), GFP_KERNEL);
	if (!ndev)
		return -ENOMEM;

	data = (struct ntb_epf_data *)id->driver_data;
	if (data) {
		if (data->peer_spad_reg_bar)
			peer_spad_reg_bar = data->peer_spad_reg_bar;
		if (data->ctrl_reg_bar)
			ctrl_reg_bar = data->ctrl_reg_bar;
		if (data->db_reg_bar)
			db_reg_bar = data->db_reg_bar;
	}

	ndev->peer_spad_reg_bar = peer_spad_reg_bar;
	ndev->ctrl_reg_bar = ctrl_reg_bar;
	ndev->db_reg_bar = db_reg_bar;

	ntb_epf_init_struct(ndev, pdev);

	ret = ntb_epf_init_pci(ndev, pdev);
	if (ret) {
		dev_err(dev, "Failed to init PCI\n");
		return ret;
	}

	ret = ntb_epf_init_dev(ndev);
	if (ret) {
		dev_err(dev, "Failed to init device\n");
		goto err_init_dev;
	}

	ret = ntb_register_device(&ndev->ntb);
	if (ret) {
		dev_err(dev, "Failed to register NTB device\n");
		goto err_register_dev;
	}

	return 0;

err_register_dev:
	pci_free_irq_vectors(pdev);

err_init_dev:
	ntb_epf_deinit_pci(ndev);

	return ret;
}

static void ntb_epf_pci_remove(struct pci_dev *pdev)
{
	struct ntb_epf_dev *ndev = pci_get_drvdata(pdev);

	ntb_unregister_device(&ndev->ntb);
	pci_free_irq_vectors(pdev);
	kfree(ndev);
}

static const struct ntb_epf_data j721e_data = {
	.ctrl_reg_bar = BAR_0,
	.peer_spad_reg_bar = BAR_1,
	.db_reg_bar = BAR_2,
};

static const struct pci_device_id ntb_epf_pci_tbl[] = {
	{
		PCI_DEVICE(PCI_VENDOR_ID_TI, PCI_DEVICE_ID_TI_J721E),
		.driver_data = (kernel_ulong_t)&j721e_data,
	},
	{ },
};
MODULE_DEVICE_TABLE(pci, ntb_epf_pci_tbl);

static struct pci_driver ntb_epf_pci_driver = {
	.name		= KBUILD_MODNAME,
	.id_table	= ntb_epf_pci_tbl,
	.probe		= ntb_epf_pci_probe,
	.remove		= ntb_epf_pci_remove,
};
module_pci_driver(ntb_epf_pci_driver);

MODULE_DESCRIPTION("PCI ENDPOINT NTB HOST DRIVER");
MODULE_AUTHOR("Kishon Vijay Abraham I <kishon@ti.com>");
MODULE_LICENSE("GPL v2");
