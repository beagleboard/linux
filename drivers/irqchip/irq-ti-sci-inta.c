// SPDX-License-Identifier: GPL-2.0
/*
 * Texas Instruments' K3 Interrupt Aggregator irqchip driver
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
#include "irq-ti-sci-intr.h"

#define MAX_EVENTS_PER_VINT	64

struct ti_sci_inta_irq_domain {
	const struct ti_sci_handle *sci;
	struct ti_sci_resource *vint;
	struct ti_sci_resource *global_event;
	void __iomem *base;
	u16 ia_id;
};

struct ti_sci_inta_event_desc {
	u16 global_event;
	u16 src_id;
	u16 src_index;
};

struct ti_sci_inta_vint_desc {
	u16 dst_id;
	u16 dst_host_irq;
	u16 vint;
	raw_spinlock_t lock;
	unsigned long *event_map;
	struct ti_sci_inta_event_desc events[MAX_EVENTS_PER_VINT];
};

static struct irq_chip ti_sci_inta_irq_chip = {
	.name			= "INTA",
	.irq_eoi		= irq_chip_eoi_parent,
	.irq_mask		= irq_chip_mask_parent,
	.irq_unmask		= irq_chip_unmask_parent,
	.irq_retrigger		= irq_chip_retrigger_hierarchy,
	.irq_set_type		= irq_chip_set_type_parent,
	.irq_set_affinity	= irq_chip_set_affinity_parent,
};

static int ti_sci_inta_irq_domain_translate(struct irq_domain *domain,
					    struct irq_fwspec *fwspec,
					    unsigned long *hwirq,
					    unsigned int *type)
{
	if (is_of_node(fwspec->fwnode)) {
		if (fwspec->param_count != 4)
			return -EINVAL;

		*hwirq = fwspec->param[2];
		*type = fwspec->param[3] & IRQ_TYPE_SENSE_MASK;

		return 0;
	}

	return -EINVAL;
}

static void ti_sci_free_event_irq(struct ti_sci_inta_irq_domain *inta,
				  struct ti_sci_inta_vint_desc *vint,
				  u32 event_index)
{
	struct ti_sci_inta_event_desc *event;
	unsigned long flags;

	if (event_index >= MAX_EVENTS_PER_VINT)
		return;

	event = &vint->events[event_index];
	inta->sci->ops.rm_irq_ops.free_event_irq(inta->sci,
						 event->src_id,
						 event->src_index,
						 vint->dst_id,
						 vint->dst_host_irq,
						 inta->ia_id, vint->vint,
						 event->global_event,
						 event_index);

	pr_debug("%s: Event released from src = %d, index = %d, to dst = %d, irq = %d,via ia_id = %d, vint = %d, global event = %d,status_bit = %d\n",
		 __func__, event->src_id, event->src_index, vint->dst_id,
		 vint->dst_host_irq, inta->ia_id, vint->vint,
		 event->global_event, event_index);

	raw_spin_lock_irqsave(&vint->lock, flags);
	clear_bit(event_index, vint->event_map);
	raw_spin_unlock_irqrestore(&vint->lock, flags);

	ti_sci_release_resource(inta->global_event, event->global_event);
}

static void ti_sci_inta_irq_domain_free(struct irq_domain *domain,
					unsigned int virq, unsigned int nr_irqs)
{
	struct ti_sci_inta_irq_domain *inta = domain->host_data;
	struct ti_sci_inta_vint_desc *vint_desc;
	struct irq_data *d;
	int event_index;

	d = irq_domain_get_irq_data(domain, virq);

	vint_desc = irq_data_get_irq_chip_data(d);

	/* This is the last event in the vint */
	event_index = find_first_bit(vint_desc->event_map, MAX_EVENTS_PER_VINT);
	ti_sci_free_event_irq(inta, vint_desc, event_index);
	irq_domain_free_irqs_parent(domain, virq, 1);
	irq_domain_reset_irq_data(d);
	ti_sci_release_resource(inta->vint, vint_desc->vint);
	kfree(vint_desc->event_map);
	kfree(vint_desc);
}

static int ti_sci_allocate_event_irq(struct ti_sci_inta_irq_domain *inta,
				     struct ti_sci_inta_vint_desc *vint_desc,
				     u16 src_id, u16 src_index)
{
	struct ti_sci_inta_event_desc *event;
	unsigned long flags;
	u32 free_bit;
	int err;

	raw_spin_lock_irqsave(&vint_desc->lock, flags);
	free_bit = find_first_zero_bit(vint_desc->event_map,
				       MAX_EVENTS_PER_VINT);
	if (free_bit != MAX_EVENTS_PER_VINT)
		set_bit(free_bit, vint_desc->event_map);
	raw_spin_unlock_irqrestore(&vint_desc->lock, flags);

	if (free_bit >= MAX_EVENTS_PER_VINT)
		return -ENODEV;

	event = &vint_desc->events[free_bit];

	event->src_id = src_id;
	event->src_index = src_index;
	event->global_event = ti_sci_get_free_resource(inta->global_event);

	pr_debug("%s: Event requested from src = %d, index = %d, to dst = %d,irq = %d,via ia_id = %d, vint = %d, global event = %d,status_bit = %d\n",
		 __func__, src_id, src_index, vint_desc->dst_id,
		 vint_desc->dst_host_irq, inta->ia_id, vint_desc->vint,
		 event->global_event, free_bit);

	err = inta->sci->ops.rm_irq_ops.set_event_irq(inta->sci,
						       src_id, src_index,
						       vint_desc->dst_id,
						       vint_desc->dst_host_irq,
						       inta->ia_id,
						       vint_desc->vint,
						       event->global_event,
						       free_bit);
	if (err) {
		pr_err("%s: Event allocation failed from src = %d, index = %d, to dst = %d,irq = %d,via ia_id = %d, vint = %d,global event = %d, status_bit = %d\n",
		       __func__, src_id, src_index, vint_desc->dst_id,
		       vint_desc->dst_host_irq, inta->ia_id, vint_desc->vint,
		       event->global_event, free_bit);
		return err;
	}

	return 0;
}

static struct ti_sci_inta_vint_desc *alloc_intr_irq(struct irq_domain *domain,
						    unsigned int virq,
						    u32 src_id, u32 src_index,
						    u32 vint, u32 flags)
{
	struct ti_sci_inta_irq_domain *inta = domain->host_data;
	struct ti_sci_inta_vint_desc *vint_desc;
	struct irq_fwspec fwspec;
	int err;

	if (!irq_domain_get_of_node(domain->parent))
		return ERR_PTR(-EINVAL);

	vint_desc = kzalloc(sizeof(*vint_desc), GFP_KERNEL);
	if (!vint_desc)
		return ERR_PTR(-ENOMEM);

	vint_desc->event_map = kcalloc(BITS_TO_LONGS(MAX_EVENTS_PER_VINT),
				       sizeof(*vint_desc->event_map),
				       GFP_KERNEL);
	if (!vint_desc->event_map) {
		kfree(vint_desc);
		return ERR_PTR(-ENOMEM);
	}

	vint_desc->vint = vint;
	fwspec.fwnode = domain->parent->fwnode;
	fwspec.param_count = 3;
	fwspec.param[0] = inta->ia_id;
	fwspec.param[1] = vint_desc->vint;
	fwspec.param[2] = flags | TI_SCI_IS_EVENT_IRQ;

	err = irq_domain_alloc_irqs_parent(domain, virq, 1, &fwspec);
	if (err)
		goto err_irqs;

	vint_desc->dst_id = ti_sci_intr_get_dst_id(domain->parent, virq);
	vint_desc->dst_host_irq = ti_sci_intr_get_dst_irq(domain->parent, virq);
	raw_spin_lock_init(&vint_desc->lock);

	err = ti_sci_allocate_event_irq(inta, vint_desc, src_id, src_index);
	if (err)
		goto err_events;

	return vint_desc;

err_events:
	irq_domain_free_irqs_parent(domain, virq, 1);
err_irqs:
	ti_sci_release_resource(inta->vint, vint_desc->vint);
	kfree(vint_desc);
	return ERR_PTR(err);
}

static int ti_sci_inta_irq_domain_alloc(struct irq_domain *domain,
					unsigned int virq, unsigned int nr_irqs,
					void *data)
{
	struct ti_sci_inta_vint_desc *vint_desc;
	struct irq_fwspec *fwspec = data;
	int err;

	/* Continuous allocation not yet supported */
	if (nr_irqs > 1)
		return -EINVAL;

	vint_desc = alloc_intr_irq(domain, virq, fwspec->param[0],
				   fwspec->param[1], fwspec->param[2],
				   fwspec->param[3]);
	if (IS_ERR(vint_desc))
		return PTR_ERR(vint_desc);

	err = irq_domain_set_hwirq_and_chip(domain, virq, vint_desc->vint,
					    &ti_sci_inta_irq_chip, vint_desc);
	if (err)
		return err;

	return 0;
}

static const struct irq_domain_ops ti_sci_inta_irq_domain_ops = {
	.alloc		= ti_sci_inta_irq_domain_alloc,
	.free		= ti_sci_inta_irq_domain_free,
	.translate	= ti_sci_inta_irq_domain_translate,
};

static int ti_sci_inta_irq_domain_probe(struct platform_device *pdev)
{
	struct irq_domain *parent_domain, *domain;
	struct ti_sci_inta_irq_domain *inta;
	struct device_node *parent_node;
	struct device *dev = &pdev->dev;
	struct resource *res;
	int ret;

	parent_node = of_irq_find_parent(dev_of_node(dev));
	if (!parent_node) {
		dev_err(dev, "Failed to get IRQ parent node\n");
		return -ENODEV;
	}

	parent_domain = irq_find_host(parent_node);
	if (!parent_domain)
		return -EPROBE_DEFER;

	inta = devm_kzalloc(dev, sizeof(*inta), GFP_KERNEL);
	if (!inta)
		return -ENOMEM;

	inta->sci = ti_sci_get_by_phandle(dev_of_node(dev), "ti,sci");
	if (IS_ERR(inta->sci)) {
		ret = PTR_ERR(inta->sci);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "ti,sci read fail %d\n", ret);
		inta->sci = NULL;
		return ret;
	}

	inta->vint = devm_ti_sci_get_of_resource(inta->sci, dev,
						 "ti,sci-vint-type");
	if (IS_ERR(inta->vint)) {
		dev_err(dev, "VINT resource allocation failed\n");
		return PTR_ERR(inta->vint);
	}

	inta->global_event = devm_ti_sci_get_of_resource(inta->sci, dev,
						"ti,sci-global-event-type");
	if (IS_ERR(inta->global_event)) {
		dev_err(dev, "Global event resource allocation failed\n");
		return PTR_ERR(inta->global_event);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	inta->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(inta->base))
		return -ENODEV;

	ret = of_property_read_u32(dev->of_node, "ti,sci-dev-id",
				   (u32 *)&inta->ia_id);
	if (ret) {
		dev_err(dev, "missing 'ti,sci-dev-id' property\n");
		return -EINVAL;
	}

	domain = irq_domain_add_hierarchy(parent_domain, 0, 0, dev_of_node(dev),
					  &ti_sci_inta_irq_domain_ops, inta);
	if (!domain) {
		dev_err(dev, "Failed to allocate IRQ domain\n");
		return -ENOMEM;
	}

	return 0;
}

int ti_sci_inta_register_event(struct device *dev, u16 src_id, u16 src_index,
			       unsigned int virq, u32 flags)
{
	struct ti_sci_inta_vint_desc *vint_desc;
	struct ti_sci_inta_irq_domain *inta;
	struct device_node *parent_node;
	struct irq_domain *domain;
	struct irq_fwspec fwspec;
	struct irq_data *d;
	int err;

	parent_node = of_irq_find_parent(dev->of_node);
	if (!parent_node)
		return -ENODEV;

	domain = irq_find_host(parent_node);
	if (!domain)
		return -EPROBE_DEFER;

	inta = domain->host_data;

	/* ToDo: Handle Polling mode */
	if (virq > 0) {
		/* If Group already available */
		d = irq_domain_get_irq_data(domain, virq);
		vint_desc = irq_data_get_irq_chip_data(d);

		err = ti_sci_allocate_event_irq(inta, vint_desc, src_id,
						src_index);
		if (err)
			return err;
	} else {
		fwspec.param_count = 4;
		fwspec.fwnode = domain->fwnode;
		fwspec.param[0] = src_id;
		fwspec.param[1] = src_index;
		fwspec.param[2] = ti_sci_get_free_resource(inta->vint);
		fwspec.param[3] = flags;

		virq = irq_create_fwspec_mapping(&fwspec);
		if (virq <= 0)
			ti_sci_release_resource(inta->vint, fwspec.param[2]);
	}

	return virq;
}
EXPORT_SYMBOL_GPL(ti_sci_inta_register_event);

static u8 get_event_index(struct ti_sci_inta_vint_desc *vint_desc, u16 src_id,
			  u16 src_index)
{
	int i;

	for (i = 0; i < MAX_EVENTS_PER_VINT; i++) {
		if (vint_desc->events[i].src_id == src_id &&
		    vint_desc->events[i].src_index == src_index)
			return i;
	}

	return MAX_EVENTS_PER_VINT;
}

static bool is_event_last(struct ti_sci_inta_vint_desc *vint_desc,
			  u8 event_index)
{
	unsigned long flags;
	int next_event;
	bool status;

	raw_spin_lock_irqsave(&vint_desc->lock, flags);
	next_event = find_next_bit(vint_desc->event_map, MAX_EVENTS_PER_VINT,
				   event_index + 1);
	status = (next_event != MAX_EVENTS_PER_VINT) ? false : true;
	if (event_index !=
		find_first_bit(vint_desc->event_map, MAX_EVENTS_PER_VINT))
		status = false;
	raw_spin_unlock_irqrestore(&vint_desc->lock, flags);

	return status;
}

void ti_sci_inta_unregister_event(struct device *dev, u16 src_id,
				  u16 src_index, unsigned int virq)
{
	struct ti_sci_inta_vint_desc *vint_desc;
	struct device_node *parent_node;
	struct irq_domain *domain;
	struct irq_data *d;
	u8 event_index;

	parent_node = of_irq_find_parent(dev->of_node);
	if (!parent_node)
		return;

	domain = irq_find_host(parent_node);

	d = irq_domain_get_irq_data(domain, virq);

	vint_desc = irq_data_get_irq_chip_data(d);

	event_index = get_event_index(vint_desc, src_id, src_index);
	if (event_index == MAX_EVENTS_PER_VINT)
		return;

	if (is_event_last(vint_desc, event_index))
		irq_dispose_mapping(virq);
	else
		ti_sci_free_event_irq(domain->host_data, vint_desc,
				      event_index);
}
EXPORT_SYMBOL_GPL(ti_sci_inta_unregister_event);

u8 ti_sci_inta_ack_event(struct irq_domain *domain, u16 src_id, u16 src_index,
			 unsigned int virq)
{
	struct ti_sci_inta_vint_desc *vint_desc;
	struct ti_sci_inta_irq_domain *inta;
	struct irq_data *d;
	u8 event_index;
	u64 val;

	if (!domain)
		return -ENODEV;

	inta = domain->host_data;
	d = irq_domain_get_irq_data(domain, virq);

	vint_desc = irq_data_get_irq_chip_data(d);
	event_index = get_event_index(vint_desc, src_id, src_index);
	if (event_index == MAX_EVENTS_PER_VINT)
		return -ENODEV;

	val = 1 << event_index;
	__raw_writeq(val, inta->base + (vint_desc->vint * 0x1000 + 0x18));

	return 0;
}
EXPORT_SYMBOL_GPL(ti_sci_inta_ack_event);

static const struct of_device_id ti_sci_inta_irq_domain_of_match[] = {
	{ .compatible = "ti,sci-inta", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, ti_sci_inta_irq_domain_of_match);

static struct platform_driver ti_sci_inta_irq_domain_driver = {
	.probe = ti_sci_inta_irq_domain_probe,
	.driver = {
		.name = "ti-sci-inta",
		.of_match_table = ti_sci_inta_irq_domain_of_match,
	},
};
module_platform_driver(ti_sci_inta_irq_domain_driver);

MODULE_AUTHOR("Lokesh Vutla <lokeshvutla@ticom>");
MODULE_DESCRIPTION("K3 Interrupt Aggregator driver over TI SCI protocol");
MODULE_LICENSE("GPL v2");
