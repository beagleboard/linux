// SPDX-License-Identifier: GPL-2.0
/*
 * Texas Instruments' TISCI Interrupt Router irqchip
 *
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 *	Lokesh Vutla <lokeshvutla@ti.com>
 */

#ifndef __IRQCHIP_TI_SCI_INTR_H
#define __IRQCHIP_TI_SCI_INTR_H

#include <linux/irqdomain.h>
#include <linux/soc/ti/ti_sci_protocol.h>

#define TI_SCI_DEV_ID_MASK	0xffff
#define TI_SCI_DEV_ID_SHIFT	16
#define TI_SCI_IRQ_ID_MASK	0xffff
#define TI_SCI_IRQ_ID_SHIFT	0
#define HWIRQ_TO_DEVID(HWIRQ)	(((HWIRQ) >> (TI_SCI_DEV_ID_SHIFT)) & \
				 (TI_SCI_DEV_ID_MASK))
#define HWIRQ_TO_IRQID(HWIRQ)	((HWIRQ) & (TI_SCI_IRQ_ID_MASK))

#define TI_SCI_IS_EVENT_IRQ	BIT(31)

#if IS_ENABLED(CONFIG_TI_SCI_INTR_IRQCHIP)

u16 ti_sci_intr_get_dst_id(struct irq_domain *domain);
u16 ti_sci_intr_get_dst_irq(struct irq_domain *domain, unsigned int virq);

#else /* CONFIG_TI_SCI_INTR_IRQCHIP */

static u16 ti_sci_intr_get_dst_id(struct irq_domain *domain)
{
	return TI_SCI_RESOURCE_NULL;
}

static u16 ti_sci_intr_get_dst_irq(struct irq_domain *domain,
				   unsigned int virq)
{
	return TI_SCI_RESOURCE_NULL;
}

#endif /* CONFIG_TI_SCI_INTR_IRQCHIP */
#endif /* __IRQCHIP_TI_SCI_INTR_H */
