// SPDX-License-Identifier: GPL-2.0
/**
 * pci-j721e - PCIe controller driver for TI's J721E SoCs
 *
 * Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com
 * Author: Kishon Vijay Abraham I <kishon@ti.com>
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/mfd/syscon.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/pci.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>

#include "../../pci.h"
#include "pcie-cadence.h"

#define EOI_REG			0x10

#define ENABLE_REG_SYS_0	0x100
#define STATUS_REG_SYS_0	0x500
#define STATUS_CLR_REG_SYS_0	0x700
#define INTx_EN(num)		(1 << (num))

#define ENABLE_REG_SYS_1	0x104
#define STATUS_REG_SYS_1	0x504
#define SYS1_INTx_EN(num)	(1 << (22 + (num)))

#define J721E_PCIE_USER_CMD_STATUS	0x4
#define LINK_TRAINING_ENABLE		BIT(0)

#define J721E_PCIE_USER_LINKSTATUS	0x14
#define LINK_STATUS			GENMASK(1, 0)

enum link_status {
	NO_RECEIVERS_DETECTED,
	LINK_TRAINING_IN_PROGRESS,
	LINK_UP_DL_IN_PROGRESS,
	LINK_UP_DL_COMPLETED,
};

#define USER_EOI_REG		0xC8
enum eoi_reg {
	EOI_DOWNSTREAM_INTERRUPT,
	EOI_FLR_INTERRUPT,
	EOI_LEGACY_INTERRUPT,
	EOI_POWER_STATE_INTERRUPT,
};

#define J721E_MODE_RC			BIT(7)
#define LANE_COUNT_MASK			BIT(8)
#define LANE_COUNT(n)			((n) << 8)

#define GENERATION_SEL_MASK		GENMASK(1, 0)

#define MAX_LANES			2

struct j721e_pcie {
	struct device		*dev;
	struct device_node	*node;
	u32			mode;
	u32			num_lanes;
	struct cdns_pcie	*cdns_pcie;
	void __iomem		*user_cfg_base;
	void __iomem		*intd_cfg_base;
	struct irq_domain	*legacy_irq_domain;
	bool			is_intc_v1;
};

enum j721e_pcie_mode {
	PCI_MODE_RC,
	PCI_MODE_EP,
};

struct j721e_pcie_data {
	enum j721e_pcie_mode	mode;
	bool			is_intc_v1;
	bool			byte_access_allowed;
	const struct cdns_pcie_ops *ops;
};

static inline u32 j721e_pcie_user_readl(struct j721e_pcie *pcie, u32 offset)
{
	return readl(pcie->user_cfg_base + offset);
}

static inline void j721e_pcie_user_writel(struct j721e_pcie *pcie, u32 offset,
					  u32 value)
{
	writel(value, pcie->user_cfg_base + offset);
}

static inline u32 j721e_pcie_intd_readl(struct j721e_pcie *pcie, u32 offset)
{
	return readl(pcie->intd_cfg_base + offset);
}

static inline void j721e_pcie_intd_writel(struct j721e_pcie *pcie, u32 offset,
					  u32 value)
{
	writel(value, pcie->intd_cfg_base + offset);
}

static void j721e_pcie_legacy_irq_handler(struct irq_desc *desc)
{
	struct j721e_pcie *pcie = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	int virq;
	u32 reg;
	int i;

	chained_irq_enter(chip, desc);

	for (i = 0; i < PCI_NUM_INTX; i++) {
		reg = j721e_pcie_intd_readl(pcie, STATUS_REG_SYS_1);
		if (!(reg & SYS1_INTx_EN(i)))
			continue;

		virq = irq_find_mapping(pcie->legacy_irq_domain, i);
		generic_handle_irq(virq);
		j721e_pcie_user_writel(pcie, USER_EOI_REG,
				       EOI_LEGACY_INTERRUPT);
	}

	chained_irq_exit(chip, desc);
}

static void j721e_pcie_v1_legacy_irq_handler(struct irq_desc *desc)
{
	int i;
	u32 reg;
	int virq;
	struct j721e_pcie *pcie = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);

	chained_irq_enter(chip, desc);

	for (i = 0; i < PCI_NUM_INTX; i++) {
		reg = j721e_pcie_intd_readl(pcie, STATUS_REG_SYS_0);
		if (!(reg & INTx_EN(i)))
			continue;

	virq = irq_find_mapping(pcie->legacy_irq_domain, 3 - i);
		generic_handle_irq(virq);
		j721e_pcie_intd_writel(pcie, STATUS_CLR_REG_SYS_0, INTx_EN(i));
		j721e_pcie_intd_writel(pcie, EOI_REG, 3 - i);
	}

	chained_irq_exit(chip, desc);
}

static int j721e_pcie_intx_map(struct irq_domain *domain, unsigned int irq,
			       irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &dummy_irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

static const struct irq_domain_ops j721e_pcie_intx_domain_ops = {
	.map = j721e_pcie_intx_map,
};

static int j721e_pcie_config_legacy_irq(struct j721e_pcie *pcie)
{
	struct device *dev = pcie->dev;
	struct irq_domain *legacy_irq_domain;
	struct device_node *node = pcie->node;
	struct device_node *intc_node;
	int irq;
	u32 reg;
	int i;

	intc_node = of_get_child_by_name(node, "legacy-interrupt-controller");
	if (!intc_node) {
		dev_WARN(dev, "legacy-interrupt-controller node is absent\n");
		return -EINVAL;
	}

	irq = irq_of_parse_and_map(intc_node, 0);
	if (!irq) {
		dev_err(dev, "Failed to parse and map legacy irq\n");
		return -EINVAL;
	}

	if (pcie->is_intc_v1) {
		irq_set_chained_handler_and_data(irq, j721e_pcie_v1_legacy_irq_handler,
						 pcie);
	} else {
		irq_set_chained_handler_and_data(irq, j721e_pcie_legacy_irq_handler,
						 pcie);
	}

	legacy_irq_domain = irq_domain_add_linear(intc_node, PCI_NUM_INTX,
						  &j721e_pcie_intx_domain_ops,
						  NULL);
	if (!legacy_irq_domain) {
		dev_err(dev, "Failed to add irq domain for legacy irqs\n");
		return -EINVAL;
	}
	pcie->legacy_irq_domain = legacy_irq_domain;

	if (pcie->is_intc_v1) {
		for (i = 0; i < PCI_NUM_INTX; i++) {
			reg = j721e_pcie_intd_readl(pcie, ENABLE_REG_SYS_0);
			reg |= INTx_EN(i);
			j721e_pcie_intd_writel(pcie, ENABLE_REG_SYS_0, reg);
		}
	} else {
		for (i = 0; i < PCI_NUM_INTX; i++) {
			reg = j721e_pcie_intd_readl(pcie, ENABLE_REG_SYS_1);
			reg |= SYS1_INTx_EN(i);
			j721e_pcie_intd_writel(pcie, ENABLE_REG_SYS_1, reg);
		}
	}

	return 0;
}

static int j721e_pcie_start_link(struct cdns_pcie *cdns_pcie)
{
	struct j721e_pcie *pcie = dev_get_drvdata(cdns_pcie->dev);
	u32 reg;

	reg = j721e_pcie_user_readl(pcie, J721E_PCIE_USER_CMD_STATUS);
	reg |= LINK_TRAINING_ENABLE;
	j721e_pcie_user_writel(pcie, J721E_PCIE_USER_CMD_STATUS, reg);

	return 0;
}

static void j721e_pcie_stop_link(struct cdns_pcie *cdns_pcie)
{
	struct j721e_pcie *pcie = dev_get_drvdata(cdns_pcie->dev);
	u32 reg;

	reg = j721e_pcie_user_readl(pcie, J721E_PCIE_USER_CMD_STATUS);
	reg &= ~LINK_TRAINING_ENABLE;
	j721e_pcie_user_writel(pcie, J721E_PCIE_USER_CMD_STATUS, reg);
}

static bool j721e_pcie_link_up(struct cdns_pcie *cdns_pcie)
{
	struct j721e_pcie *pcie = dev_get_drvdata(cdns_pcie->dev);
	u32 reg;

	reg = j721e_pcie_user_readl(pcie, J721E_PCIE_USER_LINKSTATUS);
	reg &= LINK_STATUS;
	if (reg == LINK_UP_DL_COMPLETED)
		return true;

	return false;
}

static const struct cdns_pcie_ops j721e_pcie_ops = {
	.read = cdns_platform_pcie_read32,
	.write = cdns_platform_pcie_write32,
	.start_link = j721e_pcie_start_link,
	.stop_link = j721e_pcie_stop_link,
	.link_up = j721e_pcie_link_up,
};

static const struct cdns_pcie_ops j7200_pcie_ops = {
	.start_link = j721e_pcie_start_link,
	.stop_link = j721e_pcie_stop_link,
	.link_up = j721e_pcie_link_up,
};

static int j721e_pcie_set_mode(struct j721e_pcie *pcie, struct regmap *syscon)
{
	struct device *dev = pcie->dev;
	u32 mask = J721E_MODE_RC;
	u32 mode = pcie->mode;
	u32 val = 0;
	int ret = 0;

	if (mode == PCI_MODE_RC)
		val = J721E_MODE_RC;

	ret = regmap_update_bits(syscon, 0, mask, val);
	if (ret)
		dev_err(dev, "failed to set pcie mode\n");

	return ret;
}

static int j721e_pcie_set_link_speed(struct j721e_pcie *pcie,
				     struct regmap *syscon)
{
	struct device *dev = pcie->dev;
	struct device_node *np = dev->of_node;
	int link_speed;
	u32 val = 0;
	int ret;

	link_speed = of_pci_get_max_link_speed(np);
	if (link_speed < 2)
		link_speed = 2;

	val = link_speed - 1;
	ret = regmap_update_bits(syscon, 0, GENERATION_SEL_MASK, val);
	if (ret)
		dev_err(dev, "failed to set link speed\n");

	return ret;
}

static int j721e_pcie_set_lane_count(struct j721e_pcie *pcie,
				     struct regmap *syscon)
{
	struct device *dev = pcie->dev;
	u32 lanes = pcie->num_lanes;
	u32 val = 0;
	int ret;

	val = LANE_COUNT(lanes - 1);
	ret = regmap_update_bits(syscon, 0, LANE_COUNT_MASK, val);
	if (ret)
		dev_err(dev, "failed to set link count\n");

	return ret;
}

static int j721e_pcie_ctrl_init(struct j721e_pcie *pcie)
{
	struct device *dev = pcie->dev;
	struct device_node *node = dev->of_node;
	struct regmap *syscon;
	int ret;

	syscon = syscon_regmap_lookup_by_phandle(node, "ti,syscon-pcie-ctrl");
	if (IS_ERR(syscon)) {
		dev_err(dev, "Unable to get ti,syscon-pcie-ctrl regmap\n");
		return PTR_ERR(syscon);
	}

	ret = j721e_pcie_set_mode(pcie, syscon);
	if (ret < 0) {
		dev_err(dev, "Failed to set pci mode\n");
		return ret;
	}

	ret = j721e_pcie_set_link_speed(pcie, syscon);
	if (ret < 0) {
		dev_err(dev, "Failed to set link speed\n");
		return ret;
	}

	ret = j721e_pcie_set_lane_count(pcie, syscon);
	if (ret < 0) {
		dev_err(dev, "Failed to set num-lanes\n");
		return ret;
	}

	return 0;
}

static int cdns_ti_pcie_config_read(struct pci_bus *bus, unsigned int devfn,
				    int where, int size, u32 *value)
{
	struct pci_host_bridge *bridge = pci_find_host_bridge(bus);
	struct cdns_pcie_rc *rc = pci_host_bridge_priv(bridge);
	unsigned int busn = bus->number;

	if (busn == rc->bus_range->start)
		return pci_generic_config_read32(bus, devfn, where, size,
						 value);

	return pci_generic_config_read(bus, devfn, where, size, value);
}

static int cdns_ti_pcie_config_write(struct pci_bus *bus, unsigned int devfn,
				     int where, int size, u32 value)
{
	struct pci_host_bridge *bridge = pci_find_host_bridge(bus);
	struct cdns_pcie_rc *rc = pci_host_bridge_priv(bridge);
	unsigned int busn = bus->number;

	if (busn == rc->bus_range->start)
		return pci_generic_config_write32(bus, devfn, where, size,
						  value);

	return pci_generic_config_write(bus, devfn, where, size, value);
}

static struct pci_ops cdns_ti_pcie_host_ops = {
	.map_bus	= cdns_pci_map_bus,
	.read		= cdns_ti_pcie_config_read,
	.write		= cdns_ti_pcie_config_write,
};

static const struct j721e_pcie_data j721e_pcie_rc_data = {
	.mode = PCI_MODE_RC,
	.is_intc_v1 = true,
	.byte_access_allowed = false,
	.ops = &j721e_pcie_ops,
};

static const struct j721e_pcie_data j721e_pcie_ep_data = {
	.mode = PCI_MODE_EP,
	.ops = &j721e_pcie_ops,
};

static const struct j721e_pcie_data j7200_pcie_rc_data = {
	.mode = PCI_MODE_RC,
	.is_intc_v1 = false,
	.byte_access_allowed = true,
	.ops = &j7200_pcie_ops,
};

static const struct of_device_id of_j721e_pcie_match[] = {
	{
		.compatible = "ti,j721e-pcie-host",
		.data = &j721e_pcie_rc_data,
	},
	{
		.compatible = "ti,j721e-pcie-ep",
		.data = &j721e_pcie_ep_data,
	},
	{
		.compatible = "ti,j7200-pcie-host",
		.data = &j7200_pcie_rc_data,
	},
	{},
};

static int j721e_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	const struct of_device_id *match;
	const struct cdns_pcie_ops *ops;
	struct pci_host_bridge *bridge;
	struct j721e_pcie_data *data;
	struct cdns_pcie *cdns_pcie;
	bool byte_access_allowed;
	struct j721e_pcie *pcie;
	struct cdns_pcie_rc *rc;
	struct cdns_pcie_ep *ep;
	struct gpio_desc *gpiod;
	struct resource *res;
	void __iomem *base;
	u32 num_lanes;
	u32 mode;
	int ret;

	match = of_match_device(of_match_ptr(of_j721e_pcie_match), dev);
	if (!match)
		return -EINVAL;

	data = (struct j721e_pcie_data *)match->data;
	mode = (u32)data->mode;
	byte_access_allowed = data->byte_access_allowed;
	ops = data->ops;

	pcie = devm_kzalloc(dev, sizeof(*pcie), GFP_KERNEL);
	if (!pcie)
		return -ENOMEM;

	pcie->dev = dev;
	pcie->node = node;
	pcie->mode = mode;
	pcie->is_intc_v1 = data->is_intc_v1;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "intd_cfg");
	base = devm_ioremap_resource(dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);
	pcie->intd_cfg_base = base;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "user_cfg");
	base = devm_ioremap_resource(dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);
	pcie->user_cfg_base = base;

	ret = of_property_read_u32(node, "num-lanes", &num_lanes);
	if (ret || num_lanes > MAX_LANES)
		num_lanes = 1;
	pcie->num_lanes = num_lanes;

	if (dma_set_mask_and_coherent(dev, DMA_BIT_MASK(48)))
		return -EINVAL;

	dev_set_drvdata(dev, pcie);
	pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		dev_err(dev, "pm_runtime_get_sync failed\n");
		goto err_get_sync;
	}

	ret = j721e_pcie_ctrl_init(pcie);
	if (ret < 0) {
		dev_err(dev, "pm_runtime_get_sync failed\n");
		goto err_get_sync;
	}

	switch (mode) {
	case PCI_MODE_RC:
		if (!IS_ENABLED(CONFIG_PCIE_CADENCE_HOST)) {
			ret = -ENODEV;
			goto err_get_sync;
		}

		ret = j721e_pcie_config_legacy_irq(pcie);
		if (ret < 0)
			goto err_get_sync;

		bridge = devm_pci_alloc_host_bridge(dev, sizeof(*rc));
		if (!bridge) {
			ret = -ENOMEM;
			goto err_get_sync;
		}

		if (!byte_access_allowed)
			bridge->ops = &cdns_ti_pcie_host_ops;
		rc = pci_host_bridge_priv(bridge);

		cdns_pcie = &rc->pcie;
		cdns_pcie->dev = dev;
		cdns_pcie->ops = ops;
		pcie->cdns_pcie = cdns_pcie;

		gpiod = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
		if (IS_ERR(gpiod)) {
			ret = PTR_ERR(gpiod);
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "Failed to get reset GPIO\n");
			goto err_get_sync;
		}

		ret = cdns_pcie_init_phy(dev, cdns_pcie);
		if (ret) {
			dev_err(dev, "Failed to init phy\n");
			goto err_get_sync;
		}

		/*
		 * "Power Sequencing and Reset Signal Timings" table in
		 * PCI EXPRESS CARD ELECTROMECHANICAL SPECIFICATION, REV. 3.0
		 * indicates PERST# should be deasserted after minimum of 100us
		 * once REFCLK is stable. The REFCLK to the connector in RC
		 * mode is selected while enabling the PHY. So deassert PERST#
		 * after 100 us.
		 */
		if (gpiod) {
			usleep_range(100, 200);
			gpiod_set_value_cansleep(gpiod, 1);
		}

		ret = cdns_pcie_host_setup(rc);
		if (ret < 0)
			goto err_pcie_setup;

		break;
	case PCI_MODE_EP:
		if (!IS_ENABLED(CONFIG_PCIE_CADENCE_EP)) {
			ret = -ENODEV;
			goto err_get_sync;
		}

		ep = devm_kzalloc(dev, sizeof(*ep), GFP_KERNEL);
		if (!ep) {
			ret = -ENOMEM;
			goto err_get_sync;
		}

		cdns_pcie = &ep->pcie;
		cdns_pcie->dev = dev;
		cdns_pcie->ops = ops;
		pcie->cdns_pcie = cdns_pcie;

		ret = cdns_pcie_init_phy(dev, cdns_pcie);
		if (ret) {
			dev_err(dev, "Failed to init phy\n");
			goto err_get_sync;
		}

		ret = cdns_pcie_ep_setup(ep);
		if (ret < 0)
			goto err_pcie_setup;

		break;
	default:
		dev_err(dev, "INVALID device type %d\n", mode);
	}

	return 0;

err_pcie_setup:
	cdns_pcie_disable_phy(cdns_pcie);

err_get_sync:
	pm_runtime_put(dev);
	pm_runtime_disable(dev);

	return ret;
}

static int j721e_pcie_remove(struct platform_device *pdev)
{
	struct j721e_pcie *pcie = platform_get_drvdata(pdev);
	struct cdns_pcie *cdns_pcie = pcie->cdns_pcie;
	struct device *dev = &pdev->dev;

	cdns_pcie_disable_phy(cdns_pcie);
	pm_runtime_put(dev);
	pm_runtime_disable(dev);
	of_platform_depopulate(dev);

	return 0;
}

static struct platform_driver j721e_pcie_driver = {
	.probe  = j721e_pcie_probe,
	.remove = j721e_pcie_remove,
	.driver = {
		.name	= "j721e-pcie",
		.of_match_table = of_j721e_pcie_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(j721e_pcie_driver);
