/*
 * Voltage Domain private header for voltage domain drivers
 *
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
 *	Nishanth Menon
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

#ifndef __PM_VOLTAGE_DOMAIN_PRIVATE__
#define __PM_VOLTAGE_DOMAIN_PRIVATE__

#include <linux/pm_voltage_domain.h>

struct pm_voltdm_dev;

/**
 * struct pm_voltdm_ops - Operations functions for voltage domain
 * @voltdm_get: (optional) invoked when notifier is to be registered.
 * @voltdm_put: (optional) invoked when notifier is unregistered.
 * @voltdm_latency:	(optional) compute and provide voltage domain
 *			transition latency.
 * @voltdm_do_transition: (mandatory) callback for notification
 *
 * np_args - is arguments to the node phandle - useful when specific voltage
 * domain is referred to using indices.
 * voltdm_data - is the voltage domain driver specific data corresponding to
 * driver information per registration (may point to domain driver specific
 * data including reference to index. This is always provided back to the
 * driver at various follow on operations.
 * clk_notifier_flags - follows the standard clk notification flags
 */
struct pm_voltdm_ops {
	int (*voltdm_get)(struct device *voltdm_dev,
			  struct device *request_dev,
			  struct device_node *np,
			  struct of_phandle_args *np_args,
			  const char *supply,
			  void **voltdm_data);
	int (*voltdm_latency)(struct device *voltdm_dev, void *voltdm_data,
			      unsigned long min_uv, unsigned long max_uv);
	int (*voltdm_do_transition)(struct device *voltdm_dev,
				    void *voltdm_data,
				    unsigned long clk_notifier_flags,
				    int uv, int tol_uv);
	void (*voltdm_put)(struct device *voltdm_dev,
			   struct device *request_dev,
			   void *voltdm_data);
};

#define VOLTAGE_DOMAIN_FLAG_NOTIFY_ALL		(0x1 << 0)

/**
 * struct pm_voltdm_desc - Descriptor for the voltage domain
 * @ops:	operations for the voltage domain
 * @flags:	flags controlling the various operations
 */
struct pm_voltdm_desc {
	const struct pm_voltdm_ops *ops;
	u16 flags;
};

#if defined(CONFIG_VOLTAGE_DOMAIN)
struct pm_voltdm_dev *devm_voltdm_register(struct device *dev,
					   const struct pm_voltdm_desc *desc);
void devm_voltdm_unregister(struct pm_voltdm_dev *vdev);
#else
static inline struct pm_voltdm_dev *devm_voltdm_register(struct device *dev,
					   const struct pm_voltdm_desc *desc)
{
	return ERR_PTR(-ENODEV);
}

static inline void devm_voltdm_unregister(struct pm_voltdm_dev *vdev)
{
}
#endif				/* VOLTAGE_DOMAIN */

#endif				/* __PM_VOLTAGE_DOMAIN_PRIVATE__ */
