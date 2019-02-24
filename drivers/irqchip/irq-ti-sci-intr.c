// SPDX-License-Identifier: GPL-2.0
/*
 * Texas Instruments' K3 Interrupt Router irqchip driver
 *
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 *	Lokesh Vutla <lokeshvutla@ti.com>
 */

#include <linux/err.h>
#include <linux/io.h>
#include <linux/irqchip.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/irqdomain.h>
#include <linux/soc/ti/ti_sci_protocol.h>

#define TI_SCI_DEV_ID_MASK	0xffff
#define TI_SCI_DEV_ID_SHIFT	16
#define TI_SCI_IRQ_ID_MASK	0xffff
#define TI_SCI_IRQ_ID_SHIFT	0
#define TI_SCI_IS_EVENT_IRQ	BIT(31)

#define HWIRQ_TO_DEVID(hwirq)	(((hwirq) >> (TI_SCI_DEV_ID_SHIFT)) & \
				 (TI_SCI_DEV_ID_MASK))
#define HWIRQ_TO_IRQID(hwirq)	((hwirq) & (TI_SCI_IRQ_ID_MASK))

/**
 * struct ti_sci_intr_irq_domain - Structure representing a TISCI based
 *				   Interrupt Router IRQ domain.
 * @sci:	Pointer to TISCI handle
 * @dst_irq:	TISCI resource pointer representing destination irq controller.
 * @dst_id:	TISCI device ID of the destination irq controller.
 */
struct ti_sci_intr_irq_domain {
	const struct ti_sci_handle *sci;
	struct ti_sci_resource *dst_irq;
	u16 dst_id;
};

static struct irq_chip ti_sci_intr_irq_chip = {
	.name			= "INTR",
	.irq_eoi		= irq_chip_eoi_parent,
	.irq_mask		= irq_chip_mask_parent,
	.irq_unmask		= irq_chip_unmask_parent,
	.irq_retrigger		= irq_chip_retrigger_hierarchy,
	.irq_set_type		= irq_chip_set_type_parent,
	.irq_set_affinity	= irq_chip_set_affinity_parent,
};

/**
 * ti_sci_intr_irq_domain_translate() - Retrieve hwirq and type from
 *					IRQ firmware specific handler.
 * @domain:	Pointer to IRQ domain
 * @fwspec:	Pointer to IRQ specific firmware structure
 * @hwirq:	IRQ number identified by hardware
 * @type:	IRQ type
 *
 * Return 0 if all went ok else appropriate error.
 */
static int ti_sci_intr_irq_domain_translate(struct irq_domain *domain,
					    struct irq_fwspec *fwspec,
					    unsigned long *hwirq,
					    unsigned int *type)
{
	if (is_of_node(fwspec->fwnode)) {
		if (fwspec->param_count != 3)
			return -EINVAL;

		*hwirq = ((fwspec->param[0] & TI_SCI_DEV_ID_MASK) <<
			  TI_SCI_DEV_ID_SHIFT) |
			 (fwspec->param[1] & TI_SCI_IRQ_ID_MASK);
		*type = fwspec->param[2];

		return 0;
	}

	return -EINVAL;
}

static inline void ti_sci_intr_delete_desc(struct ti_sci_intr_irq_domain *intr,
					   u16 src_id, u16 src_index,
					   u16 dst_irq)
{
	intr->sci->ops.rm_irq_ops.free_direct_irq(intr->sci, src_id, src_index,
						  intr->dst_id, dst_irq);
}

/**
 * ti_sci_intr_irq_domain_free() - Free the specified IRQs from the domain.
 * @domain:	Domain to which the irqs belong
 * @virq:	Linux virtual IRQ to be freed.
 * @nr_irqs:	Number of continuous irqs to be freed
 */
static void ti_sci_intr_irq_domain_free(struct irq_domain *domain,
					unsigned int virq, unsigned int nr_irqs)
{
	struct ti_sci_intr_irq_domain *intr = domain->host_data;
	struct irq_data *data, *parent_data;
	u32 flags;
	int i;

	intr = domain->host_data;

	for (i = 0; i < nr_irqs; i++) {
		data = irq_domain_get_irq_data(domain, virq + i);
		flags = (u32)(u64)irq_data_get_irq_chip_data(data);
		parent_data = irq_domain_get_irq_data(domain->parent, virq + i);

		if (!(flags & TI_SCI_IS_EVENT_IRQ))
			ti_sci_intr_delete_desc(intr,
						HWIRQ_TO_DEVID(data->hwirq),
						HWIRQ_TO_IRQID(data->hwirq),
						parent_data->hwirq);
		ti_sci_release_resource(intr->dst_irq, parent_data->hwirq);
		irq_domain_free_irqs_parent(domain, virq + i, 1);
		irq_domain_reset_irq_data(data);
	}
}

/**
 * allocate_gic_irq() - Allocate GIC specific IRQ
 * @domain:	Point to the interrupt router IRQ domain
 * @dev:	TISCI device IRQ generating the IRQ
 * @irq:	IRQ offset within the device
 * @flags:	Corresponding flags to the IRQ
 *
 * Returns 0 if all went well else appropriate error pointer.
 */
static int allocate_gic_irq(struct irq_domain *domain, unsigned int virq,
			    u16 dev, u16 irq, u32 flags)
{
	struct ti_sci_intr_irq_domain *intr = domain->host_data;
	struct irq_fwspec fwspec;
	u16 dst_irq;
	int err;

	if (!irq_domain_get_of_node(domain->parent))
		return -EINVAL;

	dst_irq = ti_sci_get_free_resource(intr->dst_irq);

	fwspec.fwnode = domain->parent->fwnode;
	fwspec.param_count = 3;
	fwspec.param[0] = 0;	/* SPI */
	fwspec.param[1] = dst_irq - 32; /* SPI offset */
	fwspec.param[2] = flags & IRQ_TYPE_SENSE_MASK;

	err = irq_domain_alloc_irqs_parent(domain, virq, 1, &fwspec);
	if (err)
		goto err_irqs;

	/* If event is requested then return */
	if (flags & TI_SCI_IS_EVENT_IRQ)
		return 0;

	err = intr->sci->ops.rm_irq_ops.set_direct_irq(intr->sci, dev, irq,
						       intr->dst_id, dst_irq);
	if (err) {
		pr_err("%s: IRQ allocation failed from src = %d, src_index = %d to dst_id = %d, dst_irq = %d",
		       __func__, dev, irq, intr->dst_id, dst_irq);
		goto err_msg;
	}

	return 0;

err_msg:
	irq_domain_free_irqs_parent(domain, virq, 1);
err_irqs:
	ti_sci_release_resource(intr->dst_irq, dst_irq);
	return err;
}

/**
 * ti_sci_intr_irq_domain_alloc() - Allocate Interrupt router IRQs
 * @domain:	Point to the interrupt router IRQ domain
 * @virq:	Corresponding Linux virtual IRQ number
 * @nr_irqs:	Continuous irqs to be allocated
 * @data:	Pointer to firmware specifier
 *
 * Return 0 if all went well else appropriate error value.
 */
static int ti_sci_intr_irq_domain_alloc(struct irq_domain *domain,
					unsigned int virq, unsigned int nr_irqs,
					void *data)
{
	struct irq_fwspec *fwspec = data;
	u16 src_id, src_index;
	unsigned long hwirq;
	int i, err;
	u32 type;

	err = ti_sci_intr_irq_domain_translate(domain, fwspec, &hwirq, &type);
	if (err)
		return err;

	src_id = HWIRQ_TO_DEVID(hwirq);
	src_index = HWIRQ_TO_IRQID(hwirq);

	for (i = 0; i < nr_irqs; i++) {
		err = allocate_gic_irq(domain, virq + i, src_id, src_index + i,
				       type);
		if (err)
			goto err_irq;

		err = irq_domain_set_hwirq_and_chip(domain, virq + i, hwirq + i,
						    &ti_sci_intr_irq_chip,
						    (void *)(u64)type);
		if (err)
			goto err_irq;
	}

	return 0;
err_irq:
	ti_sci_intr_irq_domain_free(domain, virq, i);
	return err;
}

static const struct irq_domain_ops ti_sci_intr_irq_domain_ops = {
	.alloc		= ti_sci_intr_irq_domain_alloc,
	.free		= ti_sci_intr_irq_domain_free,
	.translate	= ti_sci_intr_irq_domain_translate,
};

static int ti_sci_intr_irq_domain_probe(struct platform_device *pdev)
{
	struct irq_domain *parent_domain, *domain;
	struct ti_sci_intr_irq_domain *intr;
	struct device_node *parent_node;
	struct device *dev = &pdev->dev;
	int ret;

	parent_node = of_irq_find_parent(dev_of_node(dev));
	if (!parent_node) {
		dev_err(dev, "Failed to get IRQ parent node\n");
		return -ENODEV;
	}

	parent_domain = irq_find_host(parent_node);
	if (!parent_domain) {
		dev_err(dev, "Failed to find IRQ parent domain\n");
		return -ENODEV;
	}

	intr = devm_kzalloc(dev, sizeof(*intr), GFP_KERNEL);
	if (!intr)
		return -ENOMEM;

	intr->sci = devm_ti_sci_get_by_phandle(dev, "ti,sci");
	if (IS_ERR(intr->sci)) {
		ret = PTR_ERR(intr->sci);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "ti,sci read fail %d\n", ret);
		intr->sci = NULL;
		return ret;
	}

	ret = of_property_read_u32(dev_of_node(dev), "ti,sci-dst-id",
				   (u32 *)&intr->dst_id);
	if (ret) {
		dev_err(dev, "missing 'ti,sci-dst-id' property\n");
		return -EINVAL;
	}

	intr->dst_irq = devm_ti_sci_get_of_resource(intr->sci, dev,
						    intr->dst_id,
						    "ti,sci-rm-range-girq");
	if (IS_ERR(intr->dst_irq)) {
		dev_err(dev, "Destination irq resource allocation failed\n");
		return PTR_ERR(intr->dst_irq);
	}

	domain = irq_domain_add_hierarchy(parent_domain, 0, 0, dev_of_node(dev),
					  &ti_sci_intr_irq_domain_ops, intr);
	if (!domain) {
		dev_err(dev, "Failed to allocate IRQ domain\n");
		return -ENOMEM;
	}

	return 0;
}

static const struct of_device_id ti_sci_intr_irq_domain_of_match[] = {
	{ .compatible = "ti,sci-intr", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, ti_sci_intr_irq_domain_of_match);

static struct platform_driver ti_sci_intr_irq_domain_driver = {
	.probe = ti_sci_intr_irq_domain_probe,
	.driver = {
		.name = "ti-sci-intr",
		.of_match_table = ti_sci_intr_irq_domain_of_match,
	},
};
module_platform_driver(ti_sci_intr_irq_domain_driver);

MODULE_AUTHOR("Lokesh Vutla <lokeshvutla@ticom>");
MODULE_DESCRIPTION("K3 Interrupt Router driver over TI SCI protocol");
MODULE_LICENSE("GPL v2");
