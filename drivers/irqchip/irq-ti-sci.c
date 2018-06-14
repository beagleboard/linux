// SPDX-License-Identifier: GPL-2.0
/*
 * Texas Instruments' System Control Interface (TI-SCI) irqchip driver
 *
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 *	Lokesh Vutla <lokeshvutla@ti.com>
 */

#include <linux/err.h>
#include <linux/io.h>
#include <linux/irqchip.h>
#include <linux/of_platform.h>
#include <linux/irqdomain.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/soc/ti/ti_sci_protocol.h>
#include <linux/irqchip/irq-ti-sci.h>

#define TI_SCI_DEV_ID_MASK	0xffff
#define TI_SCI_DEV_ID_SHIFT	16
#define TI_SCI_IRQ_ID_MASK	0xffff
#define TI_SCI_IRQ_ID_SHIFT	0
#define HWIRQ_TO_DEVID(HWIRQ)	(((HWIRQ) >> (TI_SCI_DEV_ID_SHIFT)) & \
				 (TI_SCI_DEV_ID_MASK))
#define HWIRQ_TO_IRQID(HWIRQ)	((HWIRQ) & (TI_SCI_IRQ_ID_MASK))

/**
 * struct ti_sci_event_desc - TI SCI Event descriptor
 * @dev: Device ID of the event source peripheral.
 * @evt_src_offs: Event source number within the peripheral
 * @global_event: Global event number registered for this event, as returned
 *		  by the TISCI protocol.
 * @vint_status_bit: Status bit within the virtual interrupt for which
 *			the event has been registered.
 * @event_node: List node to be added to the list of events.
 */
struct ti_sci_event_desc {
	u16 dev;
	u16 evt_src_offs;
	u32 global_event;
	u8 vint_status_bit;
	struct list_head event_node;
};

/**
 * struct ti_sci_irq_desc - TI SCI IRQ descriptor
 * @dev: Device ID of the IRQ source peripheral. If IA flows through the
 *	 IRQ configuration route, then device ID of the Interrupt Aggregator.
 * @irq_src_offset: IRQ source number with in the peripheral
 * @host_irq: IRQ number from the host Interrupt controller
 * @group: Interrupt group to which the events are assigned
 * @virq: Linux IRQ number
 * @base: Base address of IA that irq_desc is describing.
 * @ti_sci_event_list: List of event assigned to the Interrupt group
 */
struct ti_sci_irq_desc {
	u16 dev;
	u16 irq_src_offset;
	u16 host_irq;
	u32 group;
	u32 virq;
	void __iomem *base;
	struct list_head ti_sci_event_list;
};

/**
 * struct ia_desc -  Interrupt Aggregator descriptor.
 * @dev:	Device ID of IA
 * @base:	Base address of IA.
 */
struct ia_desc {
	u32 dev;
	void __iomem *base;
};

/**
 * struct ti_sci_irq_domain - TI SCI irq domain data
 * @sci:	TI SCI handle
 * @ia:		Array of ia descriptors
 * @ia_count:	Number of IAs available in SoC.
 */
struct ti_sci_irq_domain {
	const struct ti_sci_handle *sci;
	struct ia_desc *ia;
	u32 ia_count;
};

static struct irq_chip ti_sci_irq_chip = {
	.name			= "TISCI",
	.irq_eoi		= irq_chip_eoi_parent,
	.irq_mask		= irq_chip_mask_parent,
	.irq_unmask		= irq_chip_unmask_parent,
	.irq_retrigger		= irq_chip_retrigger_hierarchy,
	.irq_set_type		= irq_chip_set_type_parent,
	.irq_set_affinity	= irq_chip_set_affinity_parent,
};

/**
 * ti_sci_request_desc() - Allocate an event/irq descriptor
 * @id:		Pointer to TISCI irqdomain host data.
 * @desc:	Pointer to TISCI irq descriptor. Needed only when the
 *		requesting event is to be grouped under this IRQ.
 * @s_host:	Host ID of the IRQ destination. An invalid value of
 *		TI_SCI_IRQ_SECONDARY_HOST_INVALID can be passed if destination
 *		host id is same as the ti-sci interface host id.
 * @dev:	IRQ/Event source peripheral ID.
 * @irq:	IRQ/Event source offset within the peripheral.
 * @share:	Enable it if the requesting event is to be share with others.
 * @poll:	Enable it if the configured IRQ shall not result in a physical
 *		interrupt
 * @virq:	Linux specific IRQ number. 0 if requesting an event.
 *
 * Sends a request to TISCI library to set an IRQ. Based on the return values
 * the following operations happen
 * - For event requests from peripheral to be added to an existing event group,
 *   a new TISCI event descriptor is created and added to list of events
 *   in the group.
 * - For event requests from peripheral to get a new IRQ, new TISCI event and
 *   IRQ descriptors gets created. The created event descriptor gets added to
 *   the list of events in the created IRQ descriptor.
 * - For direct IRQ requests from the peripherals, a new TISCI irq descriptor
 *   gets created.
 *
 * The TISCI irq descriptor will be the chip data for each irq_data created
 * in TISCI IRQ domain.
 *
 * Return: Pointer to ti_sci_irq_desc if all went well else appropriate error
 *		pointer.
 */
static struct ti_sci_irq_desc *ti_sci_request_desc(struct ti_sci_irq_domain *id,
						   struct ti_sci_irq_desc *desc,
						   u8 s_host, u16 dev,
						   u16 irq, u8 share,
						   u8 poll, u32 virq)
{
	struct ti_sci_event_desc *event_desc;
	struct ti_sci_irq_desc *irq_desc;
	u32 global_event, ia_id, vint, group;
	u8 vint_status_bit;
	u16 host_irq;
	int err, i;

	group = desc ? desc->group : TI_SCI_IRQ_GROUP_NULL;

	err = id->sci->ops.irq_ops.set_irq(id->sci, s_host, dev, irq, share,
					   poll, &group, &global_event, &ia_id,
					   &vint, &host_irq, &vint_status_bit);
	if (err)
		return ERR_PTR(err);

	/* We already have an IRQ descriptor, just insert event */
	if (desc) {
		event_desc = kzalloc(sizeof(*event_desc), GFP_KERNEL);
		if (!event_desc) {
			id->sci->ops.irq_ops.free_irq(id->sci, s_host,
						      dev, irq, global_event,
						      group);
			return ERR_PTR(-ENOMEM);
		}
		event_desc->dev = dev;
		event_desc->evt_src_offs = irq;
		event_desc->global_event = global_event;
		event_desc->vint_status_bit = vint_status_bit;

		list_add(&event_desc->event_node, &desc->ti_sci_event_list);

		return desc;
	}

	/* Create New IRQ descriptor */
	irq_desc = kzalloc(sizeof(*irq_desc), GFP_KERNEL);
	if (!irq_desc)
		return ERR_PTR(-ENOMEM);

	INIT_LIST_HEAD(&irq_desc->ti_sci_event_list);

	if (ia_id != TI_SCI_IRQ_IA_NULL) {
		/* IA is in the route, create an event descriptor. */
		event_desc = kzalloc(sizeof(*event_desc), GFP_KERNEL);
		if (!event_desc) {
			kfree(irq_desc);
			id->sci->ops.irq_ops.free_irq(id->sci, s_host,
						      dev, irq, global_event,
						      group);
			return ERR_PTR(-ENOMEM);
		}
		event_desc->dev = dev;
		event_desc->evt_src_offs = irq;
		event_desc->global_event = global_event;
		event_desc->vint_status_bit = vint_status_bit;
		list_add(&event_desc->event_node, &irq_desc->ti_sci_event_list);

		irq_desc->dev = ia_id;
		irq_desc->irq_src_offset = vint;
		if (share)
			irq_desc->group = group;
		else
			irq_desc->group = TI_SCI_IRQ_GROUP_NULL;

		irq_desc->base = NULL;
		for (i = 0; i < id->ia_count; i++)
			if (ia_id == id->ia[i].dev) {
				irq_desc->base = id->ia[i].base;
				break;
			}
		if (!irq_desc->base) {
			pr_err("%s: IA id not found for dev %d\n", __func__,
			       ia_id);
			return ERR_PTR(-ENODEV);
		}
	} else {
		/* IRQ is being requested */
		irq_desc->dev = dev;
		irq_desc->irq_src_offset = irq;
		irq_desc->group = TI_SCI_IRQ_GROUP_NULL;
		irq_desc->base = NULL;
	}
	irq_desc->host_irq = host_irq;
	irq_desc->virq = virq;

	return irq_desc;
}

/**
 * __get_event_desc() - translates (dev, event) to event descriptor
 * @desc:	Pointer to TISCI irq descriptor to which the event belongs
 * @dev:	Device ID of the event source peripheral
 * @event:	Event offset within the peripheral.
 *
 * Return: Pointer to event descriptor if all went well else NULL.
 */
static struct ti_sci_event_desc *__get_event_desc(struct ti_sci_irq_desc *desc,
						  u16 dev, u16 event)
{
	struct ti_sci_event_desc *event_desc;

	list_for_each_entry(event_desc, &desc->ti_sci_event_list, event_node) {
		if (event_desc->dev == dev &&
		    event_desc->evt_src_offs == event)
			return event_desc;
	}

	return NULL;
}

/**
 * ti_sci_delete_desc() - Delete an Event/IRQ descriptor.
 * @id:		Pointer to TISCI irqdomain host data.
 * @desc:	Pointer to TISCI irq descriptor.
 * @dev:	Device id of the Event/IRQ source peripheral
 * @irq:	Event/IRQ source offset with in the peripheral.
 */
static void ti_sci_delete_desc(struct ti_sci_irq_domain *id,
			       struct ti_sci_irq_desc *desc, u16 dev, u16 irq)
{
	u32 global_event = TI_SCI_IRQ_GLOBAL_EVENT_NULL;
	struct ti_sci_event_desc *event_desc;

	if (!desc)
		return;

	/* Events are available inside the IRQ descriptor. Delete the event */
	if (desc->dev != dev) {
		event_desc = __get_event_desc(desc, dev, irq);
		list_del(&event_desc->event_node);
		global_event = event_desc->global_event;
		kfree(event_desc);
	}

	id->sci->ops.irq_ops.free_irq(id->sci,
				      TI_SCI_IRQ_SECONDARY_HOST_INVALID,
				      dev, irq, global_event, desc->group);

	/* If no events are available, delete the IRQ descriptor */
	if (list_empty(&desc->ti_sci_event_list))
		kfree(desc);
}

/**
 * ti_sci_irq_domain_translate() - Translate DT entry to an irq
 * @domain:	Pointer to IRQ domain.
 * @fwspec:	Pointer to IRQ firmware specific structure.
 * @hwirq:	Translated hwirq number.
 * @type:	IRQ type.
 *
 * Note: hwirq is created as below:
 *	[31:16]	dev_id
 *	[15: 0]	IRQ offset within device.
 * Return: 0 if all went well, else returns appropriate error value.
 */
static int ti_sci_irq_domain_translate(struct irq_domain *domain,
				       struct irq_fwspec *fwspec,
				       unsigned long *hwirq,
				       unsigned int *type)
{
	if (fwspec->param_count < 3)
		return -EINVAL;

	*hwirq = ((fwspec->param[0] & TI_SCI_DEV_ID_MASK) <<
		  TI_SCI_DEV_ID_SHIFT) |
		 (fwspec->param[1] & TI_SCI_IRQ_ID_MASK);
	*type = fwspec->param[2] & IRQ_TYPE_SENSE_MASK;

	return 0;
}

static struct ti_sci_irq_desc *allocate_gic_irq(struct irq_domain *domain,
						unsigned int virq,
						u16 dev, u16 irq, u32 flags,
						u8 share)
{
	struct ti_sci_irq_domain *id = domain->host_data;
	struct ti_sci_irq_desc *desc;
	struct irq_fwspec fwspec;
	int err;

	if (!irq_domain_get_of_node(domain->parent))
		return ERR_PTR(-EINVAL);

	desc = ti_sci_request_desc(id, NULL, TI_SCI_IRQ_SECONDARY_HOST_INVALID,
				   dev, irq, share, TI_SCI_IRQ_POLL_DISABLE,
				   virq);
	if (IS_ERR(desc))
		return desc;

	fwspec.fwnode = domain->parent->fwnode;
	fwspec.param_count = 3;
	fwspec.param[0] = 0;	/* SPI */
	fwspec.param[1] = desc->host_irq - 32; /* SPI offset */
	fwspec.param[2] = flags;

	err = irq_domain_alloc_irqs_parent(domain, virq, 1, &fwspec);
	if (err) {
		ti_sci_delete_desc(id, desc, dev, irq);
		return ERR_PTR(err);
	}

	return desc;
}

/**
 * ti_sci_irq_domain_alloc() - Allocate a ti_sci<->irq connection
 * @domain:	Pointer to IRQ domain.
 * @virq:	Linux Virtual irq number.
 * @nr_irqs:	number of irqs to be allocated.
 * @data:	Firmware specific data.
 *
 * Return: 0 if all went well, else returns appropriate error value.
 */
static int ti_sci_irq_domain_alloc(struct irq_domain *domain, unsigned int virq,
				   unsigned int nr_irqs, void *data)
{
	struct irq_fwspec *fwspec = data;
	struct ti_sci_irq_desc *desc;
	unsigned long hwirq;
	u16 dev_id, irq_id;
	int i, err;
	u32 type;
	u8 share;

	err = ti_sci_irq_domain_translate(domain, fwspec, &hwirq, &type);
	if (err)
		return err;

	dev_id = HWIRQ_TO_DEVID(hwirq);
	irq_id = HWIRQ_TO_IRQID(hwirq);
	if (fwspec->param_count == 4)
		share = TI_SCI_IRQ_SHARE_ENABLE;
	else
		share = TI_SCI_IRQ_SHARE_DISABLE;

	for (i = 0; i < nr_irqs; i++) {
		desc = allocate_gic_irq(domain, virq + i, dev_id, irq_id + i,
					fwspec->param[2], share);
		if (IS_ERR(desc))
			/* ToDO: Clean already allocated IRQs */
			return PTR_ERR(desc);

		err = irq_domain_set_hwirq_and_chip(domain, virq + i, hwirq + i,
						    &ti_sci_irq_chip, desc);
		if (err)
			return err;
	}

	return 0;
}

/**
 * ti_sci_irq_domain_free() - unmap/free a ti_sci<->irq connection
 * @domain:	Pointer to IRQ domain
 * @virq:	Virtual irq number
 * @nr_irqs:	number of irqs to free
 */
static void ti_sci_irq_domain_free(struct irq_domain *domain, unsigned int virq,
				   unsigned int nr_irqs)
{
	struct ti_sci_irq_desc *desc;
	struct irq_data *d;
	u16 dev_id, irq_id;
	int i;

	for (i = 0; i < nr_irqs; i++) {
		d = irq_domain_get_irq_data(domain, virq + i);

		desc = irq_data_get_irq_chip_data(d);
		dev_id = HWIRQ_TO_DEVID(d->hwirq);
		irq_id = HWIRQ_TO_IRQID(d->hwirq);

		ti_sci_delete_desc(domain->host_data, desc, dev_id, irq_id);
		irq_domain_free_irqs_parent(domain, virq, 1);
		irq_domain_reset_irq_data(d);
	}
}

static const struct irq_domain_ops ti_sci_irq_domain_ops = {
	.alloc		= ti_sci_irq_domain_alloc,
	.free		= ti_sci_irq_domain_free,
	.translate	= ti_sci_irq_domain_translate,
};

/**
 * ti_sci_irq_virq_to_desc() -Translates a virtual irq to TISCI irq descriptor.
 * @dev:	Pointer to sruct device
 * @virq:	Linux virtual irq number
 *
 * Return: Pointer to TISCI irq descriptor if all went well, else
 *	   appropriate error value.
 */
static struct ti_sci_irq_desc *ti_sci_irq_virq_to_desc(struct device *dev,
						       u32 virq)
{
	struct device_node *parent_node;
	struct irq_domain *domain;
	struct irq_data *data;

	parent_node = of_irq_find_parent(dev->of_node);
	if (!parent_node)
		return ERR_PTR(-ENODEV);

	domain = irq_find_host(parent_node);
	data = irq_domain_get_irq_data(domain, virq);

	return irq_data_get_irq_chip_data(data);
}

/**
 * ti_sci_register_irq() - Registers an event/irq to irq domain.
 * @dev:	Device pointer to which irq/event is associated.
 * @desc:	Pointer to TISCI irq descriptor. Needed only when the
 *		requesting event is to be grouped under this desc.
 * @dev_id:	IRQ/Event source peripheral ID.
 * @irq:	IRQ/Event source offset within the peripheral.
 * @share:	Enable it if the requesting event is to be share with others.
 * @poll:	Enable it if the configured IRQ should not result in a physical
 *		irq.
 * @flags:	flags to specify the irq type.
 *
 * Creates an TISCI event/irq descriptor as necessary and register it in the
 * irq domain. Not all event requests will result in a host irq as some events
 * are grouped under an existing irq.
 *
 * Return: Pointer to TISCI irq descriptor if all went well, else
 *	   appropriate error value.
 */
struct ti_sci_irq_desc *ti_sci_register_irq(struct device *dev,
					    struct ti_sci_irq_desc *desc,
					    u16 dev_id, u16 irq, u8 share,
					    u8 poll, u32 flags)
{
	struct device_node *parent_node;
	struct ti_sci_irq_domain *id;
	struct irq_domain *domain;
	struct irq_fwspec fwspec;
	int err;

	parent_node = of_irq_find_parent(dev->of_node);
	if (!parent_node)
		return ERR_PTR(-ENODEV);

	domain = irq_find_host(parent_node);
	if (!domain)
		return ERR_PTR(-EPROBE_DEFER);

	id = domain->host_data;

	/* ToDo: Handle Polling mode */
	if (desc) {
		/* If Group already available */
		desc = ti_sci_request_desc(id, desc,
					   TI_SCI_IRQ_SECONDARY_HOST_INVALID,
					   dev_id, irq, TI_SCI_IRQ_SHARE_ENABLE,
					   poll, 0);
	} else {
		if (share == TI_SCI_IRQ_SHARE_ENABLE)
			fwspec.param_count = 4;
		else
			fwspec.param_count = 3;
		fwspec.fwnode = domain->fwnode;
		fwspec.param[0] = dev_id;
		fwspec.param[1] = irq;
		fwspec.param[2] = flags;

		err = irq_create_fwspec_mapping(&fwspec);
		if (!err)
			return ERR_PTR(-EINVAL);
		desc = ti_sci_irq_virq_to_desc(dev, err);
	}

	return desc;
}
EXPORT_SYMBOL_GPL(ti_sci_register_irq);

/**
 * ti_sci_unregister_irq() -  Unregisters an event/irq from irq domain
 * @dev:	Device pointer to which irq/event is associated.
 * @desc:	Pointer to TISCI irq descriptor. Needed only when the
 *		requesting event is to be grouped under this desc.
 * @dev_id:	IRQ/Event source peripheral ID.
 * @irq:	IRQ/Event source offset within the peripheral.
 *
 * Unregisters an event/irq from the irq domain. Also the host irq to linux
 * virq mapping is disposed if an irq is passed or the event passed is the
 * last event associated with the irq.
 */
void ti_sci_unregister_irq(struct device *dev, struct ti_sci_irq_desc *desc,
			   u16 dev_id, u16 irq)
{
	struct ti_sci_event_desc *event_desc;
	struct device_node *parent_node;
	struct irq_domain *domain;

	parent_node = of_irq_find_parent(dev->of_node);
	if (!parent_node)
		return;

	domain = irq_find_host(parent_node);

	if (desc->dev != dev_id) {
		event_desc = __get_event_desc(desc, dev_id, irq);
		if (!list_is_last(&desc->ti_sci_event_list,
				  &event_desc->event_node)) {
			ti_sci_delete_desc(domain->host_data, desc, dev_id,
					   irq);
			return;
		}
	}
	irq_dispose_mapping(desc->virq);
}
EXPORT_SYMBOL_GPL(ti_sci_unregister_irq);

/**
 * ti_sci_irq_desc_to_virq() - Get virtual irq from ti_sci_irq_desc
 * @desc:	Pointer to TISCI irq descriptor.
 *
 * Return: Virtual irq number if all went ok, else appropriate error value.
 */
u32 ti_sci_irq_desc_to_virq(struct ti_sci_irq_desc *desc)
{
	if (!desc)
		return 0;

	return desc->virq;
}
EXPORT_SYMBOL_GPL(ti_sci_irq_desc_to_virq);

/**
 * ti_sci_irq_desc_to_global_event() - Get global event from ti_sci_irq_desc
 * @desc:		Pointer to TISCI irq descriptor.
 * @dev:		Event source peripheral ID.
 * @event_offset:	Event source offset within the peripheral.
 *
 * Return: Global event number if all went ok, else appropriate error value.
 */
u32 ti_sci_irq_desc_to_global_event(struct ti_sci_irq_desc *desc, u16 dev,
				    u16 event_offset)
{
	struct ti_sci_event_desc *event_desc;

	if (!desc)
		return -ENODEV;

	event_desc = __get_event_desc(desc, dev, event_offset);
	if (!event_desc)
		return -ENODEV;

	return event_desc->global_event;
}
EXPORT_SYMBOL_GPL(ti_sci_irq_desc_to_global_event);

/**
 * ti_sci_irq_desc_to_virq() - Get IA id from ti_sci_irq_desc
 * @desc:	Pointer to TISCI irq descriptor.
 *
 * Return: Interrupt Aggregator ID if all went ok, else appropriate error value.
 */
u32 ti_sci_irq_desc_to_ia(struct ti_sci_irq_desc *desc)
{
	if (!desc)
		return -ENODEV;

	if (list_empty(&desc->ti_sci_event_list))
		return -ENODEV;

	return desc->dev;
}
EXPORT_SYMBOL_GPL(ti_sci_irq_desc_to_ia);

/**
 * ti_sci_irq_desc_to_vint() - Get virtual interrupt no. from ti_sci_irq_desc
 * @desc:	Pointer to TISCI irq descriptor.
 *
 * Return: Virtual interrupt no. if all went ok, else appropriate error value.
 */
u32 ti_sci_irq_desc_to_vint(struct ti_sci_irq_desc *desc)
{
	if (!desc)
		return -ENODEV;

	if (list_empty(&desc->ti_sci_event_list))
		return -ENODEV;

	return desc->irq_src_offset;
}
EXPORT_SYMBOL_GPL(ti_sci_irq_desc_to_vint);

/**
 * ti_sci_irq_desc_to_vint_status_bit() - Get virtual interrupt status bit
 *					from ti_sci_irq_desc
 * @desc:		Pointer to TISCI irq descriptor.
 * @dev:		Event source peripheral ID.
 * @event_offset:	Event source offset within the peripheral.
 *
 * Return: Virtual interrupt status bit if all went ok,
 *		else appropriate error value.
 */
u8 ti_sci_irq_desc_to_vint_status_bit(struct ti_sci_irq_desc *desc, u16 dev,
				      u16 event_offset)
{
	struct ti_sci_event_desc *event_desc;

	if (!desc)
		return -ENODEV;

	event_desc = __get_event_desc(desc, dev, event_offset);
	if (!event_desc)
		return -ENODEV;

	return event_desc->vint_status_bit;
}
EXPORT_SYMBOL_GPL(ti_sci_irq_desc_to_vint_status_bit);

/**
 * ti_sci_ack_irq() - Request to ack an IRQ corresponding to an event.
 * @desc:		Pointer to TISCI irq descriptor.
 * @dev:		IRQ/Event source peripheral ID.
 * @event_offset:	Event source offset within the peripheral.
 *
 * Return: 0 if all went ok, else appropriate error value.
 */
u8 ti_sci_ack_irq(struct ti_sci_irq_desc *desc, u16 dev, u16 event_offset)
{
	struct ti_sci_event_desc *event_desc;
	u64 val;

	if (!desc)
		return -ENODEV;

	event_desc = __get_event_desc(desc, dev, event_offset);
	if (!event_desc)
		return -ENODEV;

	val = 1 << event_desc->vint_status_bit;
	__raw_writeq(val, desc->base + (desc->irq_src_offset * 0x1000 + 0x18));

	return 0;
}
EXPORT_SYMBOL_GPL(ti_sci_ack_irq);

static int ti_sci_irq_domain_probe(struct platform_device *pdev)
{
	struct irq_domain *parent_domain, *domain;
	struct ti_sci_irq_domain *ti_sci_data;
	const struct ti_sci_handle *handle;
	struct device_node *parent_node;
	struct device *dev = &pdev->dev;
	struct resource *res;
	int ret, i;

	parent_node = of_irq_find_parent(dev->of_node);
	if (!parent_node) {
		dev_err(dev, "Failed to get IRQ parent node\n");
		return -ENODEV;
	}

	parent_domain = irq_find_host(parent_node);
	if (!parent_domain) {
		dev_err(dev, "Failed to find IRQ parent domain\n");
		return -ENODEV;
	}

	handle = devm_ti_sci_get_handle(dev);
	if (IS_ERR(handle))
		return PTR_ERR(handle);

	ti_sci_data = devm_kzalloc(dev, sizeof(*ti_sci_data), GFP_KERNEL);
	if (!ti_sci_data)
		return -ENOMEM;

	ti_sci_data->sci = handle;

	ti_sci_data->ia_count = of_property_count_elems_of_size(dev->of_node,
								"ia-ids",
								sizeof(u32));
	ti_sci_data->ia = devm_kzalloc(dev, sizeof(*ti_sci_data->ia) *
					ti_sci_data->ia_count, GFP_KERNEL);
	if (!ti_sci_data->ia)
		return -ENOMEM;

	for (i = 0; i < ti_sci_data->ia_count; i++) {
		ret = of_property_read_u32_index(dev_of_node(dev), "ia-ids", i,
						 &ti_sci_data->ia[i].dev);
		if (ret)
			return -EINVAL;

		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		ti_sci_data->ia[i].base = devm_ioremap_resource(dev, res);
		if (IS_ERR(ti_sci_data->ia[i].base))
			return -ENODEV;
	}

	domain = irq_domain_add_hierarchy(parent_domain, 0,
					  0,
					  dev_of_node(dev),
					  &ti_sci_irq_domain_ops,
					  ti_sci_data);
	if (!domain) {
		dev_err(dev, "Failed to allocate IRQ domain\n");
		return -ENOMEM;
	}

	return 0;
}

static const struct of_device_id ti_sci_irq_domain_of_match[] = {
	{ .compatible = "ti,sci-irq", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, ti_sci_irq_domain_of_match);

static struct platform_driver ti_sci_irq_domain_driver = {
	.probe = ti_sci_irq_domain_probe,
	.driver = {
		.name = "ti-sci-irq",
		.of_match_table = ti_sci_irq_domain_of_match,
	},
};
module_platform_driver(ti_sci_irq_domain_driver);

MODULE_AUTHOR("Lokesh Vutla <lokeshvutla@ticom>");
MODULE_DESCRIPTION("TI System Control Interface (TI SCI) IRQ driver");
MODULE_LICENSE("GPL v2");
