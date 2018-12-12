/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Texas Instruments' System Control Interface (TI-SCI) irqchip
 *
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 *	Lokesh Vutla <lokeshvutla@ti.com>
 */

#ifndef __INCLUDE_LINUX_IRQCHIP_TI_SCI_INTA_H
#define __INCLUDE_LINUX_IRQCHIP_TI_SCI_INTA_H

#if IS_ENABLED(CONFIG_TI_SCI_INTA_IRQCHIP)
int ti_sci_inta_register_event(struct device *dev, u16 src_id, u16 src_index,
			       unsigned int virq, bool ack_needed, u32 flags);
int ti_sci_inta_unregister_event(struct device *dev, u16 src_id, u16 src_index,
				 unsigned int virq);

#else /* CONFIG_TI_SCI_INTA_IRQCHIP */

static inline int ti_sci_inta_register_event(struct device *dev, u16 src_id,
					     u16 src_index, unsigned int virq,
					     bool ack_needed, u32 flags)
{
	return -EINVAL;
}

static inline int ti_sci_inta_unregister_event(struct device *dev, u16 src_id,
					       u16 src_index, unsigned int virq)
{
	return -EINVAL;
}

#endif /* CONFIG_TI_SCI_INTA_IRQCHIP */

#endif /* __INCLUDE_LINUX_IRQCHIP_TI_SCI_INTA_H */
