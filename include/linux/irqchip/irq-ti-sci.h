/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Texas Instruments' System Control Interface (TI-SCI) irqchip
 *
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 *	Lokesh Vutla <lokeshvutla@ti.com>
 */

#ifndef __INCLUDE_LINUX_IRQCHIP_TI_SCI_H
#define __INCLUDE_LINUX_IRQCHIP_TI_SCI_H

struct ti_sci_irq_desc;

#if IS_ENABLED(CONFIG_TI_SCI_IRQCHIP)
struct ti_sci_irq_desc *ti_sci_register_irq(struct device *dev,
					    struct ti_sci_irq_desc *desc,
					    u16 dev_id, u16 irq, u8 share,
					    u8 poll, u32 flags);
void ti_sci_unregister_irq(struct device *dev, struct ti_sci_irq_desc *desc,
			   u16 dev_id, u16 irq);
u32 ti_sci_irq_desc_to_virq(struct ti_sci_irq_desc *desc);
u32 ti_sci_irq_desc_to_global_event(struct ti_sci_irq_desc *desc, u16 dev,
				    u16 event_offset);
u32 ti_sci_irq_desc_to_ia(struct ti_sci_irq_desc *desc);
u32 ti_sci_irq_desc_to_vint(struct ti_sci_irq_desc *desc);
u8 ti_sci_irq_desc_to_vint_status_bit(struct ti_sci_irq_desc *desc, u16 dev,
				      u16 event_offset);
u8 ti_sci_ack_irq(struct ti_sci_irq_desc *desc, u16 dev, u16 event_offset);

#else /* CONFIG_TI_SCI_IRQCHIP */

static inline
struct ti_sci_irq_desc *ti_sci_register_irq(struct device *dev,
					    struct ti_sci_irq_desc *desc,
					    u16 dev_id, u16 irq, u8 share,
					    u8 poll, u32 flags)
{
	return ERR_PTR(-EINVAL);
}

static inline
void ti_sci_unregister_irq(struct device *dev, struct ti_sci_irq_desc *desc,
			   u16 dev_id, u16 irq)
{
}

static inline u32 ti_sci_irq_desc_to_virq(struct ti_sci_irq_desc *desc)
{
	return -EINVAL;
}

static inline
u32 ti_sci_irq_desc_to_global_event(struct ti_sci_irq_desc *desc, u16 dev,
				    u16 event_offset)
{
	return -EINVAL;
}

static inline u32 ti_sci_irq_desc_to_ia(struct ti_sci_irq_desc *desc)
{
	return -EINVAL;
}

static inline u32 ti_sci_irq_desc_to_vint(struct ti_sci_irq_desc *desc)
{
	return -EINVAL;
}

static inline
u8 ti_sci_irq_desc_to_vint_status_bit(struct ti_sci_irq_desc *desc, u16 dev,
				      u16 event_offset)
{
	return -EINVAL;
}

static inline
u8 ti_sci_ack_irq(struct ti_sci_irq_desc *desc, u16 dev, u16 event_offset)
{
	return -EINVAL;
}

#endif /* CONFIG_TI_SCI_IRQCHIP */
#endif /* __INCLUDE_LINUX_IRQCHIP_TI_SCI_H */
