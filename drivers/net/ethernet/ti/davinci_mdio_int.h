/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Davinci MDIO internal definitions
 *
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 */

#ifndef DAVINCI_MDIO_INT_H_
#define DAVINCI_MDIO_INT_H_

#include <linux/kernel.h>
#include <linux/module.h>

struct davinci_mdio_data;
struct davinci_mdio_data *davinci_mdio_create(struct device *dev,
					      struct device_node *node,
					      void __iomem *reg_base,
					      const char *clk_name);
void davinci_mdio_release(struct davinci_mdio_data *mdio);

#endif /* DAVINCI_MDIO_INT_H_ */
