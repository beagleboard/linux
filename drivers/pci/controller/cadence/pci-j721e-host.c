// SPDX-License-Identifier: GPL-2.0
/*
 * pci-j721e-host - PCIe host controller driver for TI's J721E SoCs
 *
 * Copyright (C) 2023 Texas Instruments Incorporated - http://www.ti.com
 * Author: Kishon Vijay Abraham I <kishon@ti.com>
 */

#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/of_device.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/of_irq.h>
#include <linux/irqchip/chained_irq.h>

#include "pcie-cadence.h"
#include "pci-j721e.h"

#define CTRL_MMR_LOCK2_MASK		0xFFFFFFFF
#define CTRL_MMR_LOCK2_KICK0_UNLOCK_VAL	0x68EF3490
#define CTRL_MMR_LOCK2_KICK1_UNLOCK_VAL	0xD172BC5A
#define CTRL_MMR_LOCK_KICK_LOCK_VAL	0xFFFFFFFF
#define CTRL_MMR_ACSPCIE_PAD_MASK	0xFFFFFFFF
#define CTRL_MMR_ACSPCIE_PAD_EN		0x01000000
#define PCIE_REFCLK_CLKSEL_OUT_EN	BIT(8)
#define PCIE_REFCLK_CLKSEL_MASK	GENMASK(1, 0)

static int cdns_ti_pcie_config_read(struct pci_bus *bus, unsigned int devfn,
					int where, int size, u32 *value)
{
	if (pci_is_root_bus(bus))
		return pci_generic_config_read32(bus, devfn, where, size,
						 value);

	return pci_generic_config_read(bus, devfn, where, size, value);
}

static int cdns_ti_pcie_config_write(struct pci_bus *bus, unsigned int devfn,
					int where, int size, u32 value)
{
	if (pci_is_root_bus(bus))
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
	.quirk_retrain_flag = true,
	.byte_access_allowed = false,
	.linkdown_irq_regfield = LINK_DOWN,
	.max_lanes = 2,
	.is_intc_v1 = true,
};

static const struct j721e_pcie_data j7200_pcie_rc_data = {
	.mode = PCI_MODE_RC,
	.quirk_detect_quiet_flag = true,
	.linkdown_irq_regfield = J7200_LINK_DOWN,
	.byte_access_allowed = true,
	.max_lanes = 2,
};

static const struct j721e_pcie_data am64_pcie_rc_data = {
	.mode = PCI_MODE_RC,
	.linkdown_irq_regfield = J7200_LINK_DOWN,
	.byte_access_allowed = true,
	.max_lanes = 1,
};

static const struct j721e_pcie_data j784s4_pcie_rc_data = {
	.mode = PCI_MODE_RC,
	.quirk_retrain_flag = true,
	.byte_access_allowed = false,
	.linkdown_irq_regfield = LINK_DOWN,
	.max_lanes = 4,
};

static const struct j721e_pcie_data j722s_pcie_rc_data = {
	.mode = PCI_MODE_RC,
	.linkdown_irq_regfield = J7200_LINK_DOWN,
	.byte_access_allowed = true,
	.max_lanes = 1,
};

static const struct of_device_id of_j721e_pcie_host_match[] = {
	{
		.compatible = "ti,j721e-pcie-host",
		.data = &j721e_pcie_rc_data,
	},
	{
		.compatible = "ti,j7200-pcie-host",
		.data = &j7200_pcie_rc_data,
	},
	{
		.compatible = "ti,am64-pcie-host",
		.data = &am64_pcie_rc_data,
	},
	{
		.compatible = "ti,j784s4-pcie-host",
		.data = &j784s4_pcie_rc_data,
	},
	{
		.compatible = "ti,j722s-pcie-host",
		.data = &j722s_pcie_rc_data,
	},
	{},
};
MODULE_DEVICE_TABLE(of, of_j721e_pcie_host_match);

static void j721e_pcie_legacy_irq_handler(struct irq_desc *desc)
{
	struct j721e_pcie *pcie = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	int i, virq;
	u32 reg;

	chained_irq_enter(chip, desc);

	for (i = 0; i < PCI_NUM_INTX; i++) {
		reg = j721e_pcie_intd_readl(pcie, STATUS_REG_SYS_1);
		if (!(reg & SYS1_INTx_EN(i)))
			continue;

		virq = irq_find_mapping(pcie->legacy_irq_domain, i);
		generic_handle_irq(virq);
	}

	chained_irq_exit(chip, desc);
}

static void j721e_pcie_irq_eoi(struct irq_data *data)
{
	struct j721e_pcie *pcie = irq_data_get_irq_chip_data(data);

	j721e_pcie_user_writel(pcie, USER_EOI_REG, EOI_LEGACY_INTERRUPT);
}

struct irq_chip j721e_pcie_irq_chip = {
	.name		= "J721E-PCIE-INTX",
	.irq_eoi	= j721e_pcie_irq_eoi,
};

static void j721e_pcie_v1_legacy_irq_handler(struct irq_desc *desc)
{
	struct j721e_pcie *pcie = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	int i, virq;
	u32 reg;

	chained_irq_enter(chip, desc);

	for (i = 0; i < PCI_NUM_INTX; i++) {
		reg = j721e_pcie_intd_readl(pcie, STATUS_REG_SYS_0);
		if (!(reg & INTx_EN(i)))
			continue;

		virq = irq_find_mapping(pcie->legacy_irq_domain, 3 - i);
		generic_handle_irq(virq);
		j721e_pcie_intd_writel(pcie, STATUS_CLR_REG_SYS_0, INTx_EN(i));
	}

	chained_irq_exit(chip, desc);
}

static int j721e_pcie_intx_map(struct irq_domain *domain, unsigned int irq, irq_hw_number_t hwirq)
{
	struct j721e_pcie *pcie = domain->host_data;

	if (pcie->is_intc_v1)
		irq_set_chip_and_handler(irq, &dummy_irq_chip, handle_simple_irq);
	else
		irq_set_chip_and_handler(irq, &j721e_pcie_irq_chip, handle_fasteoi_irq);

	irq_set_chip_data(irq, pcie);

	return 0;
}

static const struct irq_domain_ops j721e_pcie_intx_domain_ops = {
	.map = j721e_pcie_intx_map,
};

static int j721e_pcie_config_legacy_irq(struct j721e_pcie *pcie)
{
	struct irq_domain *legacy_irq_domain;
	struct device *dev = pcie->cdns_pcie->dev;
	struct device_node *node = dev->of_node;
	struct device_node *intc_node;
	int irq, i;
	u32 reg;

	intc_node = of_get_child_by_name(node, "interrupt-controller");
	if (!intc_node) {
		dev_dbg(dev, "interrupt-controller node is absent. Legacy INTR not supported\n");
		return 0;
	}

	irq = irq_of_parse_and_map(intc_node, 0);
	if (!irq) {
		dev_err(dev, "Failed to parse and map legacy irq\n");
		return -EINVAL;
	}

	if (pcie->is_intc_v1)
		irq_set_chained_handler_and_data(irq, j721e_pcie_v1_legacy_irq_handler, pcie);
	else
		irq_set_chained_handler_and_data(irq, j721e_pcie_legacy_irq_handler, pcie);

	legacy_irq_domain = irq_domain_add_linear(intc_node, PCI_NUM_INTX,
						  &j721e_pcie_intx_domain_ops, pcie);
	if (!legacy_irq_domain) {
		dev_err(dev, "Failed to add irq domain for legacy irqs\n");
		return -EINVAL;
	}
	pcie->legacy_irq_domain = legacy_irq_domain;

	for (i = 0; i < PCI_NUM_INTX; i++) {
		if (pcie->is_intc_v1) {
			reg = j721e_pcie_intd_readl(pcie, ENABLE_REG_SYS_0);
			reg |= INTx_EN(i);
			j721e_pcie_intd_writel(pcie, ENABLE_REG_SYS_0, reg);
		} else {
			reg = j721e_pcie_intd_readl(pcie, ENABLE_REG_SYS_1);
			reg |= SYS1_INTx_EN(i);
			j721e_pcie_intd_writel(pcie, ENABLE_REG_SYS_1, reg);
		}
	}

	return 0;
}

static int j721e_enable_acspcie(struct j721e_pcie *pcie)
{
	struct device *dev = pcie->cdns_pcie->dev;
	struct device_node *node = dev->of_node;
	struct of_phandle_args args;
	unsigned int lock2_kick0_offset, lock2_kick1_offset;
	unsigned int acspcie_pad_offset, refclk_clksel_offset;
	unsigned int refclk_clksel_source;
	struct regmap *syscon;
	u32 val = 0, mask = 0;
	int ret;

	/*
	 * If property ti,syscon-pcie-refclk-out exists, believe PCIe connector
	 * requires PCIe ref clock from Serdes, so enable ACSPCIE pads and mux
	 * to source out PCIe ref clock else ref clock has to be supplied from
	 * on-board clock generator.
	 */
	syscon = syscon_regmap_lookup_by_phandle(node, "ti,syscon-pcie-refclk-out");
	if (IS_ERR(syscon))
		return 0;

	ret = of_parse_phandle_with_fixed_args(node, "ti,syscon-pcie-refclk-out", 5,
						0, &args);
	if (ret) {
		dev_err(dev, "Failed to read ti,syscon-pcie-refclk-out property\n");
		return ret;
	}

	lock2_kick0_offset = args.args[0];
	lock2_kick1_offset = args.args[1];
	acspcie_pad_offset = args.args[2];
	refclk_clksel_offset = args.args[3];
	refclk_clksel_source = args.args[4];

	/* Un-lock Partition 2 : 8000h to 9FFFh */
	mask = CTRL_MMR_LOCK2_MASK;
	val = CTRL_MMR_LOCK2_KICK0_UNLOCK_VAL;
	ret = regmap_update_bits(syscon, lock2_kick0_offset, mask, val);
	if (ret)
		goto err;

	val = CTRL_MMR_LOCK2_KICK1_UNLOCK_VAL;
	ret = regmap_update_bits(syscon, lock2_kick1_offset, mask, val);
	if (ret)
		goto err;

	/* Enable ACSPCIe PADS  */
	mask = CTRL_MMR_ACSPCIE_PAD_MASK;
	val = CTRL_MMR_ACSPCIE_PAD_EN;
	ret = regmap_update_bits(syscon, acspcie_pad_offset, mask, val);
	if (ret)
		goto err;

	/* PCIE_REFCLKx_CLKSEL : EN + ref_clk_source */
	mask = PCIE_REFCLK_CLKSEL_OUT_EN | PCIE_REFCLK_CLKSEL_MASK;
	val = PCIE_REFCLK_CLKSEL_OUT_EN | refclk_clksel_source;
	ret = regmap_update_bits(syscon, refclk_clksel_offset, mask, val);
	if (ret)
		goto err;

	/* Re-lock Partition 2 : 8000h to 9FFFh */
	mask = CTRL_MMR_LOCK_KICK_LOCK_VAL;
	val = CTRL_MMR_LOCK_KICK_LOCK_VAL;
	ret = regmap_update_bits(syscon, lock2_kick0_offset, mask, val);
	if (ret)
		goto err;

	ret = regmap_update_bits(syscon, lock2_kick1_offset, mask, val);
	if (ret)
		goto err;

	return 0;
err:
	dev_err(dev, "Failed to enable ref clock out\n");
	return ret;
}

static int j721e_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pci_host_bridge *bridge;
	const struct j721e_pcie_data *data;
	struct cdns_pcie *cdns_pcie;
	struct j721e_pcie *pcie;
	struct cdns_pcie_rc *rc = NULL;
	struct gpio_desc *gpiod;
	struct clk *clk;
	int ret;

	data = of_device_get_match_data(dev);
	if (!data)
		return -EINVAL;

	pcie = devm_kzalloc(dev, sizeof(*pcie), GFP_KERNEL);
	if (!pcie)
		return -ENOMEM;

	bridge = devm_pci_alloc_host_bridge(dev, sizeof(*rc));
	if (!bridge)
		return -ENOMEM;

	if (!data->byte_access_allowed)
		bridge->ops = &cdns_ti_pcie_host_ops;
	rc = pci_host_bridge_priv(bridge);
	rc->quirk_retrain_flag = data->quirk_retrain_flag;
	rc->quirk_detect_quiet_flag = data->quirk_detect_quiet_flag;

	cdns_pcie = &rc->pcie;
	cdns_pcie->dev = dev;
	cdns_pcie->ops = &j721e_pcie_ops;
	pcie->cdns_pcie = cdns_pcie;

	pcie->mode = PCI_MODE_RC;
	pcie->linkdown_irq_regfield = data->linkdown_irq_regfield;

	ret = j721e_pcie_common_init(pcie);
	if (ret)
		return ret;

	pcie->is_intc_v1 = data->is_intc_v1;
	ret = j721e_pcie_config_legacy_irq(pcie);
	if (ret < 0)
		goto err_get_sync;

	/*
	 * Enable ACSPCIe clock buffer to source out reference clock for EP
	 */
	ret = j721e_enable_acspcie(pcie);
	if (ret < 0)
		goto err_get_sync;

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

	clk = devm_clk_get_optional(dev, "pcie_refclk");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		dev_err(dev, "failed to get pcie_refclk\n");
		goto err_pcie_setup;
	}

	ret = clk_prepare_enable(clk);
	if (ret) {
		dev_err(dev, "failed to enable pcie_refclk\n");
		goto err_pcie_setup;
	}
	pcie->refclk = clk;

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
	if (ret < 0) {
		clk_disable_unprepare(pcie->refclk);
		goto err_pcie_setup;
	}

	return 0;

err_pcie_setup:
	cdns_pcie_disable_phy(cdns_pcie);

err_get_sync:
	j721e_disable_common_init(dev);

	return ret;
}

static int j721e_pcie_remove(struct platform_device *pdev)
{
	struct j721e_pcie *pcie = platform_get_drvdata(pdev);
	struct cdns_pcie *cdns_pcie = pcie->cdns_pcie;
	struct cdns_pcie_rc *rc = container_of(cdns_pcie, struct cdns_pcie_rc, pcie);
	struct device *dev = &pdev->dev;

	cdns_pcie_host_remove_setup(rc);
	j721e_pcie_remove_link_irq(pcie);

	cdns_pcie_stop_link(cdns_pcie);
	clk_disable_unprepare(pcie->refclk);

	gpiod_set_value_cansleep(pcie->gpiod, 0);
	cdns_pcie_deinit_phy(cdns_pcie);
	j721e_disable_common_init(dev);

	return 0;
}

static struct platform_driver j721e_pcie_host_driver = {
	.probe  = j721e_pcie_probe,
	.remove = j721e_pcie_remove,
	.driver = {
		.name	= "j721e-pcie-host",
		.of_match_table = of_j721e_pcie_host_match,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(j721e_pcie_host_driver);
MODULE_AUTHOR("Kishon Vijay Abraham I <kishon@ti.com>");
MODULE_LICENSE("GPL v2");
