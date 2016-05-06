/*
 * OPP Domain header for providers of platform OPP domain drivers
 *
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __LINUX_PM_OPP_DOMAIN__
#define __LINUX_PM_OPP_DOMAIN__

#define PM_OPPDM_VOLT_PRERATE		BIT(0)
#define PM_OPPDM_VOLT_POSTRATE		BIT(1)
#define PM_OPPDM_VOLT_ABORTRATE		BIT(2)

/**
 * struct pm_opp_domain_ops - Operations functions for opp domain device
 * @oppdm_get: (optional) invoked when opp domain supply is obtained.
 * @oppdm_put: (optional) invoked when opp domain supply is released.
 * @oppdm_latency:	(optional) compute and provide opp domain
 *			transition latency.
 * @oppdm_do_transition: (mandatory) callback for notification
 * @oppdm_is_supported_voltage: (optional) return whether or not voltage
 *				range is possible using the opp domain
 *
 * These functions provide hooks for platform OPP domain drivers to
 * override the default operations of an OPP domain which only
 * allow a single regulator per device for OPP transitions.
 */
struct pm_opp_domain_ops {
	int (*oppdm_get)(struct device *oppdm_dev,
			 struct device *request_dev,
			 struct device_node *np,
			 const char *supply,
			 void **oppdm_data);
	int (*oppdm_get_latency)(struct device *oppdm_dev, void *oppdm_data,
				 unsigned long old_uv,
				 unsigned long old_uv_min,
				 unsigned long old_uv_max,
				 unsigned long new_uv,
				 unsigned long new_uv_min,
				 unsigned long new_uv_max);
	int (*oppdm_do_transition)(struct device *oppdm_dev,
				   void *oppdm_data,
				   unsigned long clk_notifier_flags,
				   int uv, int uv_min, int uv_max);
	void (*oppdm_put)(struct device *oppdm_dev,
			  struct device *request_dev,
			  void *oppdm_data);
	bool (*oppdm_is_supported_voltage)(struct device *oppdm_dev,
					   void *oppdm_data,
					   unsigned long uV_min,
					   unsigned long uV_max);
};

/**
 * struct pm_oppdm_desc - Descriptor for the voltage domain
 * @ops:	operations for the voltage domain
 * @flags:	flags controlling the various operations
 */
struct pm_opp_domain_desc {
	const struct pm_opp_domain_ops *ops;
	u16 flags;
};

#ifdef CONFIG_PM_OPP
struct pm_opp_domain_dev
*devm_opp_domain_register(struct device *dev,
			  const struct pm_opp_domain_desc *desc);
void devm_opp_domain_unregister(struct pm_opp_domain_dev *oppdm_dev);
#else
static inline struct pm_opp_domain_dev
*devm_opp_domain_register(struct device *dev,
			  const struct pm_opp_domain_desc *desc)
{
	return -ENODEV;
}

static inline
void devm_opp_domain_unregister(struct pm_opp_domain_dev *oppdm_dev)
{
}
#endif /* CONFIG_PM_OPP */
#endif /* __LINUX_PM_OPP_DOMAIN__ */
