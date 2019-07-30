// SPDX-License-Identifier: GPL-2.0
//
// Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/
//

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#define UFS_SS_CTRL		0x4
#define UFS_SS_RST_N_PCS	BIT(0)
#define UFS_SS_CLK_26MHZ	BIT(4)

static int ti_j721e_ufs_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	unsigned long clk_rate;
	void __iomem *regbase;
	struct resource *res;
	struct clk *clk;
	u32 reg = 0;
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	regbase = devm_ioremap_resource(dev, res);
	if (IS_ERR(regbase))
		return PTR_ERR(regbase);

	/* Select M-PHY refclk frequency */
	clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk)) {
		dev_err(dev, "Cannot claim M-PHY clock.\n");
		return PTR_ERR(clk);
	}
	clk_rate = clk_get_rate(clk);
	if (clk_rate == 26000000)
		reg |= UFS_SS_CLK_26MHZ;
	devm_clk_put(&pdev->dev, clk);

	/*  Take UFS slave device out of reset */
	reg |= UFS_SS_RST_N_PCS;

	writel(reg, regbase + UFS_SS_CTRL);

	ret = of_platform_populate(pdev->dev.of_node, NULL, NULL,
				   &pdev->dev);
	if (ret)
		dev_err(dev, "failed to populate child nodes %d\n", ret);

	return ret;
}

static int ti_j721e_ufs_remove(struct platform_device *pdev)
{
	of_platform_depopulate(&pdev->dev);

	return 0;
}

static const struct of_device_id ti_j721e_ufs_of_match[] = {
	{
		.compatible = "ti,j721e-ufs",
	},
	{ },
};

static struct platform_driver ti_j721e_ufs_driver = {
	.probe	= ti_j721e_ufs_probe,
	.remove	= ti_j721e_ufs_remove,
	.driver	= {
		.name   = "ti-j721e-ufs",
		.of_match_table = ti_j721e_ufs_of_match,
	},
};
module_platform_driver(ti_j721e_ufs_driver);

MODULE_AUTHOR("Vignesh Raghavendra <vigneshr@ti.com>");
MODULE_DESCRIPTION("TI UFS host controller glue driver");
MODULE_LICENSE("GPL v2");
