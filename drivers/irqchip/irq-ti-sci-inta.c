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
#include <linux/irqdomain.h>
#include <linux/soc/ti/ti_sci_protocol.h>

#define MAX_EVENTS_PER_VINT	64
#define TI_SCI_EVENT_IRQ	BIT(31)

#define VINT_ENABLE_CLR_OFFSET	0x18

/**
 * struct ti_sci_inta_irq_domain - Structure representing a TISCI based
 *				   Interrupt Aggregator IRQ domain.
 * @sci:	Pointer to TISCI handle
 * @vint:	TISCI resource pointer representing IA inerrupts.
 * @global_event:TISCI resource pointer representing global events.
 * @base:	Base address of the memory mapped IO registers
 * @ia_id:	TISCI device ID of this Interrupt Aggregator.
 * @dst_id:	TISCI device ID of the destination irq controller.
 */
struct ti_sci_inta_irq_domain {
	const struct ti_sci_handle *sci;
	struct ti_sci_resource *vint;
	struct ti_sci_resource *global_event;
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
 * @lock:		lock to guard the event map
 * @event_map:		Bitmap to manage the allocation of events to vint.
 * @events:		Array of event descriptors assigned to this vint.
 * @ack_needed:		Event needs to be acked via INTA. This is used when
 *			HW generating events cannot clear the events by itself.
 *			Assuming that only events from the same hw block are
 *			grouped. So all the events attached to vint needs
 *			an ack or none needs an ack.
 */
struct ti_sci_inta_vint_desc {
	raw_spinlock_t lock;
	unsigned long *event_map;
	struct ti_sci_inta_event_desc events[MAX_EVENTS_PER_VINT];
	bool ack_needed;
};

static void ti_sci_inta_irq_eoi(struct irq_data *data)
{
	struct ti_sci_inta_irq_domain *inta = data->domain->host_data;
	struct ti_sci_inta_vint_desc *vint_desc;
	u64 val;
	int bit;

	vint_desc = irq_data_get_irq_chip_data(data);
	if (!vint_desc->ack_needed)
		goto out;

	for_each_set_bit(bit, vint_desc->event_map, MAX_EVENTS_PER_VINT) {
		val = 1 << bit;
		__raw_writeq(val, inta->base + data->hwirq * 0x1000 +
			     VINT_ENABLE_CLR_OFFSET);
	}

out:
	irq_chip_eoi_parent(data);
}

static struct irq_chip ti_sci_inta_irq_chip = {
	.name			= "INTA",
	.irq_eoi		= ti_sci_inta_irq_eoi,
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
 * @vint_desc:	Virtual interrupt descriptor containing the event.
 * @event_index:Event Index within the vint.
 * @dst_irq:	Destination host irq
 * @vint:	Interrupt number within interrupt aggregator.
 */
static void ti_sci_free_event_irq(struct ti_sci_inta_irq_domain *inta,
				  struct ti_sci_inta_vint_desc *vint_desc,
				  u32 event_index, u16 dst_irq, u16 vint)
{
	struct ti_sci_inta_event_desc *event;
	unsigned long flags;

	if (event_index >= MAX_EVENTS_PER_VINT)
		return;

	event = &vint_desc->events[event_index];
	inta->sci->ops.rm_irq_ops.free_event_irq(inta->sci,
						 event->src_id,
						 event->src_index,
						 inta->dst_id,
						 dst_irq,
						 inta->ia_id, vint,
						 event->global_event,
						 event_index);

	raw_spin_lock_irqsave(&vint_desc->lock, flags);
	clear_bit(event_index, vint_desc->event_map);
	raw_spin_unlock_irqrestore(&vint_desc->lock, flags);

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
	struct irq_data *data, *gic_data;
	int event_index;

	data = irq_domain_get_irq_data(domain, virq);
	gic_data = irq_domain_get_irq_data(domain->parent->parent, virq);

	vint_desc = irq_data_get_irq_chip_data(data);

	/* This is the last event in the vint */
	event_index = find_first_bit(vint_desc->event_map, MAX_EVENTS_PER_VINT);
	ti_sci_free_event_irq(inta, vint_desc, event_index,
			      gic_data->hwirq, data->hwirq);
	irq_domain_free_irqs_parent(domain, virq, 1);
	ti_sci_release_resource(inta->vint, data->hwirq);
	irq_domain_reset_irq_data(data);
	kfree(vint_desc->event_map);
	kfree(vint_desc);
}

/**
 * ti_sci_allocate_event_irq() - Allocate an event to a IA vint.
 * @inta:	Pointer to Interrupt Aggregator IRQ domain
 * @vint_desc:	Virtual interrupt descriptor to which the event gets attached.
 * @src_id:	TISCI device id of the event source
 * @src_index:	Event index with in the device.
 * @dst_irq:	Destination host irq
 * @vint:	Interrupt number within interrupt aggregator.
 *
 * Return 0 if all went ok else appropriate error value.
 */
static int ti_sci_allocate_event_irq(struct ti_sci_inta_irq_domain *inta,
				     struct ti_sci_inta_vint_desc *vint_desc,
				     u16 src_id, u16 src_index, u16 dst_irq,
				     u16 vint)
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

	err = inta->sci->ops.rm_irq_ops.set_event_irq(inta->sci,
						      src_id, src_index,
						      inta->dst_id,
						      dst_irq,
						      inta->ia_id,
						      vint,
						      event->global_event,
						      free_bit);
	if (err) {
		pr_err("%s: Event allocation failed from src = %d, index = %d, to dst = %d,irq = %d,via ia_id = %d, vint = %d,global event = %d, status_bit = %d\n",
		       __func__, src_id, src_index, inta->dst_id, dst_irq,
		       inta->ia_id, vint, event->global_event, free_bit);
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
	struct irq_data *gic_data;
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

	fwspec.fwnode = domain->parent->fwnode;
	fwspec.param_count = 3;
	/* Interrupt parent is Interrupt Router */
	fwspec.param[0] = inta->ia_id;
	fwspec.param[1] = vint;
	fwspec.param[2] = flags | TI_SCI_EVENT_IRQ;

	err = irq_domain_alloc_irqs_parent(domain, virq, 1, &fwspec);
	if (err)
		goto err_irqs;

	gic_data = irq_domain_get_irq_data(domain->parent->parent, virq);

	raw_spin_lock_init(&vint_desc->lock);

	err = ti_sci_allocate_event_irq(inta, vint_desc, src_id, src_index,
					gic_data->hwirq, vint);
	if (err)
		goto err_events;

	return vint_desc;

err_events:
	irq_domain_free_irqs_parent(domain, virq, 1);
err_irqs:
	ti_sci_release_resource(inta->vint, vint);
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

	vint_desc = alloc_parent_irq(domain, virq, fwspec->param[0],
				     fwspec->param[1], fwspec->param[2],
				     fwspec->param[3]);
	if (IS_ERR(vint_desc))
		return PTR_ERR(vint_desc);

	err = irq_domain_set_hwirq_and_chip(domain, virq, fwspec->param[2],
					    &ti_sci_inta_irq_chip, vint_desc);

	return err;
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

	inta->sci = devm_ti_sci_get_by_phandle(dev, "ti,sci");
	if (IS_ERR(inta->sci)) {
		ret = PTR_ERR(inta->sci);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "ti,sci read fail %d\n", ret);
		inta->sci = NULL;
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "ti,sci-dev-id",
				   (u32 *)&inta->ia_id);
	if (ret) {
		dev_err(dev, "missing 'ti,sci-dev-id' property\n");
		return -EINVAL;
	}

	inta->vint = devm_ti_sci_get_of_resource(inta->sci, dev,
						 inta->ia_id,
						 "ti,sci-rm-range-vint");
	if (IS_ERR(inta->vint)) {
		dev_err(dev, "VINT resource allocation failed\n");
		return PTR_ERR(inta->vint);
	}

	inta->global_event =
		devm_ti_sci_get_of_resource(inta->sci, dev,
					    inta->ia_id,
					    "ti,sci-rm-range-global-event");
	if (IS_ERR(inta->global_event)) {
		dev_err(dev, "Global event resource allocation failed\n");
		return PTR_ERR(inta->global_event);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	inta->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(inta->base))
		return -ENODEV;

	ret = of_property_read_u32(parent_node, "ti,sci-dst-id",
				   (u32 *)&inta->dst_id);

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
 * @ack_needed:	If explicit clearing of event is required.
 * @flags:	Corresponding IRQ flags
 *
 * Creates a new irq and attaches to IA domain if virq is not specified
 * else attaches the event to vint corresponding to virq.
 * When using TISCI within the client drivers, source indexes are always
 * generated dynamically and cannot be represented in DT. So client
 * drivers should call this API instead of platform_get_irq().
 *
 * Return virq if all went well else appropriate error value.
 */
int ti_sci_inta_register_event(struct device *dev, u16 src_id, u16 src_index,
			       unsigned int virq, bool ack_needed, u32 flags)
{
	struct ti_sci_inta_vint_desc *vint_desc;
	struct ti_sci_inta_irq_domain *inta;
	struct irq_data *data, *gic_data;
	struct device_node *parent_node;
	struct irq_domain *domain;
	struct irq_fwspec fwspec;
	int err;

	parent_node = of_irq_find_parent(dev->of_node);
	if (!parent_node)
		return -ENODEV;

	domain = irq_find_host(parent_node);
	if (!domain)
		return -EPROBE_DEFER;

	inta = domain->host_data;

	if (virq > 0) {
		/* If Group already available */
		data = irq_domain_get_irq_data(domain, virq);
		gic_data = irq_domain_get_irq_data(domain->parent->parent,
						   virq);

		vint_desc = irq_data_get_irq_chip_data(data);

		err = ti_sci_allocate_event_irq(inta, vint_desc, src_id,
						src_index, gic_data->hwirq,
						data->hwirq);
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

		data = irq_domain_get_irq_data(domain, virq);
		vint_desc = irq_data_get_irq_chip_data(data);
		vint_desc->ack_needed = ack_needed;
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
	struct irq_data *data, *gic_data;
	struct device_node *parent_node;
	struct irq_domain *domain;
	u8 event_index;

	parent_node = of_irq_find_parent(dev->of_node);
	if (!parent_node)
		return;

	domain = irq_find_host(parent_node);

	data = irq_domain_get_irq_data(domain, virq);
	gic_data = irq_domain_get_irq_data(domain->parent->parent, virq);

	vint_desc = irq_data_get_irq_chip_data(data);

	event_index = get_event_index(vint_desc, src_id, src_index);
	if (event_index == MAX_EVENTS_PER_VINT)
		return;

	if (is_event_last(vint_desc, event_index))
		irq_dispose_mapping(virq);
	else
		ti_sci_free_event_irq(domain->host_data, vint_desc,
				      event_index, gic_data->hwirq,
				      data->hwirq);
}
EXPORT_SYMBOL_GPL(ti_sci_inta_unregister_event);

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
