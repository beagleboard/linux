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

/**
 * struct ti_sci_inta_irq_domain - Structure representing a TISCI based
 *				   Interrupt Aggregator IRQ domain.
 * @sci:	Pointer to TISCI handle
 * @vint:	TISCI resource pointer representing IA inerrupts.
 * @global_event:TISCI resource pointer representing global events.
 * @dst_irq:	TISCI resource pointer representing destination irq controller.
 * @base:	Base address of the memory mapped IO registers
 * @ia_id:	TISCI device ID of this Interrupt Aggregator.
 * @dst_id:	TISCI device ID of the destination irq controller.
 */
struct ti_sci_inta_irq_domain {
	const struct ti_sci_handle *sci;
	struct ti_sci_resource *vint;
	struct ti_sci_resource *global_event;
	struct ti_sci_resource *dst_irq;
	void __iomem *base;
	u16 ia_id;
	u16 dst_id;
};

/**
 * struct ti_sci_inta_event_desc - Description of an event coming to
 *				   Interrupt Aggregator.
 * @global_event:	Global event number corresponding to this event
 * @src_id:		TISCI device ID of the event source
 * @src_index:		Event source index within the device.
 */
struct ti_sci_inta_event_desc {
	u16 global_event;
	u16 src_id;
	u16 src_index;
};

/**
 * struct ti_sci_inta_vint_desc - Description of a virtual interrupt coming out
 *				  of Interrupt Aggregator.
 * @dst_host_irq:	Destination host IRQ.
 * @vint:		interrupt number with in the Interrupt Aggregator.
 * @lock:		lock to guard the event map
 * @event_map:		Bitmap to manage the allocation of events to vint.
 * @events:		Array of event descriptors assigned to this vint.
 */
struct ti_sci_inta_vint_desc {
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

/**
 * ti_sci_inta_irq_domain_translate() - Retrieve hwirq and type from
 *					IRQ firmware specific handler.
 * @domain:	Pointer to IRQ domain
 * @fwspec:	Pointer to IRQ specific firmware structure
 * @hwirq:	IRQ number identified by hardware
 * @type:	IRQ type
 *
 * Return 0 if all went ok else appropriate error.
 */
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

/**
 * ti_sci_free_event_irq() - Free an event from vint
 * @inta:	Pointer to Interrupt Aggregator IRQ domain
 * @vint:	Virtual interrupt descriptor containing the event.
 * @event_index:	Event Index within the vint.
 */
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
						 inta->dst_id,
						 vint->dst_host_irq,
						 inta->ia_id, vint->vint,
						 event->global_event,
						 event_index);

	pr_debug("%s: Event released from src = %d, index = %d, to dst = %d, irq = %d,via ia_id = %d, vint = %d, global event = %d,status_bit = %d\n",
		 __func__, event->src_id, event->src_index, inta->dst_id,
		 vint->dst_host_irq, inta->ia_id, vint->vint,
		 event->global_event, event_index);

	raw_spin_lock_irqsave(&vint->lock, flags);
	clear_bit(event_index, vint->event_map);
	raw_spin_unlock_irqrestore(&vint->lock, flags);

	ti_sci_release_resource(inta->global_event, event->global_event);
}

/**
 * ti_sci_inta_irq_domain_free() - Free an IRQ from the IRQ domain
 * @domain:	Domain to which the irqs belong
 * @virq:	base linux virtual IRQ to be freed.
 * @nr_irqs:	Number of continuous irqs to be freed
 */
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
	if (inta->dst_irq)
		ti_sci_release_resource(inta->dst_irq, vint_desc->dst_host_irq);
	kfree(vint_desc->event_map);
	kfree(vint_desc);
}

/**
 * ti_sci_allocate_event_irq() - Allocate an event to a IA vint.
 * @inta:	Pointer to Interrupt Aggregator IRQ domain
 * @vint_desc:	Virtual interrupt descriptor to which the event gets attached.
 * @src_id:	TISCI device id of the event source
 * @src_index:	Event index with in the device.
 *
 * Return 0 if all went ok else appropriate error value.
 */
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
		 __func__, src_id, src_index, inta->dst_id,
		 vint_desc->dst_host_irq, inta->ia_id, vint_desc->vint,
		 event->global_event, free_bit);

	err = inta->sci->ops.rm_irq_ops.set_event_irq(inta->sci,
						       src_id, src_index,
						       inta->dst_id,
						       vint_desc->dst_host_irq,
						       inta->ia_id,
						       vint_desc->vint,
						       event->global_event,
						       free_bit);
	if (err) {
		pr_err("%s: Event allocation failed from src = %d, index = %d, to dst = %d,irq = %d,via ia_id = %d, vint = %d,global event = %d, status_bit = %d\n",
		       __func__, src_id, src_index, inta->dst_id,
		       vint_desc->dst_host_irq, inta->ia_id, vint_desc->vint,
		       event->global_event, free_bit);
		return err;
	}

	return 0;
}

/**
 * alloc_parent_irq() - Allocate parent irq to Interrupt aggregator
 * @domain:	IRQ domain corresponding to Interrupt Aggregator
 * @virq:	Linux virtual IRQ number
 * @src_id:	TISCI device id of the event source
 * @src_index:	Event index with in the device.
 * @vint:	Virtual interrupt number within IA
 * @flags:	Corresponding IRQ flags
 *
 * Return pointer to vint descriptor if all went well else corresponding
 * error pointer.
 */
static struct ti_sci_inta_vint_desc *alloc_parent_irq(struct irq_domain *domain,
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
	if (!inta->dst_irq) {
		/* Interrupt parent is Interrupt Router */
		fwspec.param[0] = inta->ia_id;
		fwspec.param[1] = vint_desc->vint;
		fwspec.param[2] = flags | TI_SCI_IS_EVENT_IRQ;
	} else {
		/* Interrupt parent is GIC */
		fwspec.param[0] = 0;	/* SPI */
		fwspec.param[1] = ti_sci_get_free_resource(inta->dst_irq) - 32;
		fwspec.param[2] = flags;
		vint_desc->dst_host_irq = fwspec.param[1];
	}

	err = irq_domain_alloc_irqs_parent(domain, virq, 1, &fwspec);
	if (err)
		goto err_irqs;

	if (!inta->dst_irq)
		vint_desc->dst_host_irq =
				ti_sci_intr_get_dst_irq(domain->parent, virq);

	raw_spin_lock_init(&vint_desc->lock);

	err = ti_sci_allocate_event_irq(inta, vint_desc, src_id, src_index);
	if (err)
		goto err_events;

	return vint_desc;

err_events:
	irq_domain_free_irqs_parent(domain, virq, 1);
err_irqs:
	if (inta->dst_irq)
		ti_sci_release_resource(inta->dst_irq, vint_desc->dst_host_irq);
	ti_sci_release_resource(inta->vint, vint_desc->vint);
	kfree(vint_desc);
	return ERR_PTR(err);
}

/**
 * ti_sci_inta_irq_domain_alloc() - Allocate Interrupt aggregator IRQs
 * @domain:	Point to the interrupt aggregator IRQ domain
 * @virq:	Corresponding Linux virtual IRQ number
 * @nr_irqs:	Continuous irqs to be allocated
 * @data:	Pointer to firmware specifier
 *
 * Return 0 if all went well else appropriate error value.
 */
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

	vint_desc = alloc_parent_irq(domain, virq, fwspec->param[0],
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

	inta->dst_irq = devm_ti_sci_get_of_resource(inta->sci, dev,
						    "ti,sci-dst-irq-type");
	if (!IS_ERR(inta->dst_irq)) {
		/*
		 * If dst type is specified, assume GIC is the parent
		 * then look for dst id
		 */
		ret = of_property_read_u32(dev_of_node(dev), "ti,sci-dst-id",
					   (u32 *)&inta->dst_id);
		if (ret) {
			dev_err(dev, "missing 'ti,sci-dst-id' property\n");
			return -EINVAL;
		}
	} else {
		if (PTR_ERR(inta->dst_irq) != -EINVAL) {
			dev_err(dev, "irq resource allocation failed\n");
			return PTR_ERR(inta->dst_irq);
		}
		/* If dst is not specified, assume INTR is the parent */
		inta->dst_irq = NULL;
		inta->dst_id = ti_sci_intr_get_dst_id(parent_domain);
	}

	domain = irq_domain_add_hierarchy(parent_domain, 0, 0, dev_of_node(dev),
					  &ti_sci_inta_irq_domain_ops, inta);
	if (!domain) {
		dev_err(dev, "Failed to allocate IRQ domain\n");
		return -ENOMEM;
	}

	return 0;
}

/**
 * ti_sci_inta_register_event() - Register a event to an interrupt aggregator
 * @dev:	Device pointer to source generating the event
 * @src_id:	TISCI device ID of the event source
 * @src_index:	Event source index within the device.
 * @virq:	Linux Virtual IRQ number
 * @flags:	Corresponding IRQ flags
 *
 * Creates a new irq and attaches to IA domain if virq is not specified
 * else attaches the event to vint corresponding to virq.
 * Return virq if all went well else appropriate error value.
 */
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

/**
 * get_event_index() - Helper api to get the event index with the vint.
 * @vint_desc:	Pointer to the IA irq descriptor
 * @src_id:	TISCI device ID of the event source
 * @src_index:	Event source index within the device.
 *
 * Return the index if all went well else MAX_EVENTS_PER_VINT.
 */
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

/**
 * is_event_last() -  Helper api to determine if the event is the last
 *		      event within the vint
 * @vint_desc:	Pointer to the IA irq descriptor
 * @event_index:	Event index within the vint of IA.
 *
 * Return true if specified index is the last event else false.
 */
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

/**
 * ti_sci_inta_unregister_event() - Unregister a event from an IA.
 * @dev:	Device pointer to source generating the event
 * @src_id:	TISCI device ID of the event source
 * @src_index:	Event source index within the device.
 * @virq:	Linux Virtual IRQ number
 *
 * Deletes the event from IA interrupt descriptor. If this event is the
 * last event then delete the vint as well.
 */
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
