// SPDX-License-Identifier: GPL-2.0-only
/*
 * Memory-mapped interface driver for DW ehance-spi core
 *
 * Copyright (c) 2021, ailibaba-inc corperation.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/scatterlist.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/acpi.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include "spi-dw-quad.h"

#define DRIVER_NAME "dw_qspi_mmio"

struct dw_qspi_mmio {
	struct dw_spi  dws;
	struct clk     *clk;
	struct clk     *pclk;
	void           *priv;
};

static int dw_qspi_mmio_probe(struct platform_device *pdev)
{
	int (*init_func)(struct platform_device *pdev,
			 struct dw_qspi_mmio *dwsmmio);
	struct dw_qspi_mmio *dwsmmio;
	struct dw_spi *dws;
	int ret;
	int num_cs,rx_sample_dly;

	dwsmmio = devm_kzalloc(&pdev->dev, sizeof(struct dw_qspi_mmio),
			GFP_KERNEL);
	if (!dwsmmio)
		return -ENOMEM;

	dws = &dwsmmio->dws;

	/* Get basic io resource and map it */
	dws->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dws->regs)) {
		dev_err(&pdev->dev, "QSPI region map failed\n");
		return PTR_ERR(dws->regs);
	}

	dws->irq = platform_get_irq(pdev, 0);
	if (dws->irq < 0)
		return dws->irq; /* -ENXIO */

	dwsmmio->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(dwsmmio->clk))
		return PTR_ERR(dwsmmio->clk);
	ret = clk_prepare_enable(dwsmmio->clk);
	if (ret)
		return ret;

	/* Optional clock needed to access the registers */
	dwsmmio->pclk = devm_clk_get_optional(&pdev->dev, "pclk");
	if (IS_ERR(dwsmmio->pclk)) {
		ret = PTR_ERR(dwsmmio->pclk);
		goto out_clk;
	}
	ret = clk_prepare_enable(dwsmmio->pclk);
	if (ret)
		goto out_clk;

	/* set bus number */
	dws->bus_num = pdev->id;

	/* get supported freq in max */
	dws->max_freq = clk_get_rate(dwsmmio->clk);

	//dws->slave_cs = devm_gpiod_get_index_optional(&pdev->dev, "cs", 0,GPIOD_OUT_LOW);
	dws->slave_cs = devm_gpiod_get_optional(&pdev->dev, "cs", GPIOD_OUT_LOW);
	if (IS_ERR(dws->slave_cs))
		return PTR_ERR(dws->slave_cs);
	/* get reg width of controler */
	device_property_read_u32(&pdev->dev, "reg-io-width", &dws->reg_io_width);

	/* get chip select count of controler */
	num_cs = 1;
	device_property_read_u32(&pdev->dev, "num-cs", &num_cs);
	dws->num_cs = num_cs;
	rx_sample_dly = 4;
	device_property_read_u32(&pdev->dev, "rx-sample-dly", &rx_sample_dly);
	printk("get gpio succes %d\n",rx_sample_dly);
	dws->rx_sample_delay = rx_sample_dly;
	init_func = device_get_match_data(&pdev->dev);
	if (init_func) {
		ret = init_func(pdev, dwsmmio);
		if (ret)
			goto out;
	}

	ret = dw_qspi_add_host(&pdev->dev, dws);
	if (ret){
		goto out;
		dw_drv_log("probe failed \n");
	}
	platform_set_drvdata(pdev, dwsmmio);
	dw_drv_log("dw_qspi_probe success \n");
	return 0;

out:
	clk_disable_unprepare(dwsmmio->pclk);
out_clk:
	clk_disable_unprepare(dwsmmio->clk);
	return ret;
}

static int dw_qspi_mmio_remove(struct platform_device *pdev)
{
	struct dw_qspi_mmio *dwsmmio = platform_get_drvdata(pdev);

	dw_qspi_remove_host(&dwsmmio->dws);
	clk_disable_unprepare(dwsmmio->pclk);
	clk_disable_unprepare(dwsmmio->clk);

	return 0;
}

static const struct of_device_id dw_qspi_mmio_of_match[] = {
	{ .compatible = "snps,dw-apb-ssi-quad", },
	{ /* end of table */}
};
MODULE_DEVICE_TABLE(of, dw_qspi_mmio_of_match);

static struct platform_driver dw_qspi_mmio_driver = {
	.probe		= dw_qspi_mmio_probe,
	.remove		= dw_qspi_mmio_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.of_match_table = dw_qspi_mmio_of_match,
	},
};
module_platform_driver(dw_qspi_mmio_driver);

MODULE_AUTHOR("linghui zeng <linghui.zlh@linux.alibaba.com>");
MODULE_DESCRIPTION("Memory-mapped I/O interface driver for DW ehance-spi Core");
MODULE_LICENSE("GPL v2");
