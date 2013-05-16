/*
 * PRU driver for TI's AM33xx series of SoCs
 *
 * Copyright (C) 2013 Pantelis Antoniou <panto@antoniou-consulting.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/dma-mapping.h>
#include <linux/remoteproc.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/genalloc.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/pinctrl/consumer.h>
#include <linux/io.h>

#include "remoteproc_internal.h"

/* PRU control structure */
struct pruproc {
	struct rproc *rproc;
	struct platform_device *pdev;
	struct resource_table *table;
	void __iomem *vaddr;
	dma_addr_t paddr;
};

/* Loads the firmware to shared memory. */
static int pruproc_load_segments(struct rproc *rproc, const struct firmware *fw)
{
	struct pruproc *pruproc = rproc->priv;

	dev_dbg(&pruproc->pdev->dev, "%s\n", __func__);

	return 0;
}

/* Find the resource table inside the remote processor's firmware. */
static struct resource_table *
pruproc_find_rsc_table(struct rproc *rproc, const struct firmware *fw,
		     int *tablesz)
{
	struct pruproc *pruproc = rproc->priv;
	struct resource_table *table;

	table = devm_kzalloc(&pruproc->pdev->dev, sizeof(*table), GFP_KERNEL);

	dev_dbg(&pruproc->pdev->dev, "%s\n", __func__);

	return table;
}

static int pruproc_sanity_check(struct rproc *rproc, const struct firmware *fw)
{
	return 0;
}

/* PRU firmware handler operations */
const struct rproc_fw_ops pruproc_fw_ops = {
	.find_rsc_table	= pruproc_find_rsc_table,
	.load		= pruproc_load_segments,
	.sanity_check	= pruproc_sanity_check,
};

/* Kick the modem with specified notification id */
static void pruproc_kick(struct rproc *rproc, int vqid)
{
	struct pruproc *pruproc = rproc->priv;

	dev_dbg(&pruproc->pdev->dev, "kick vqid:%d\n", vqid);
}

/* Start the PRU modem */
static int pruproc_start(struct rproc *rproc)
{
	struct pruproc *pruproc = rproc->priv;

	dev_dbg(&pruproc->pdev->dev, "start pru\n");

	return 0;
}

/* Stop the PRU modem */
static int pruproc_stop(struct rproc *rproc)
{
	struct pruproc *pruproc = rproc->priv;

	dev_dbg(&pruproc->pdev->dev, "stop PRU\n");

	return 0;
}

static struct rproc_ops pruproc_ops = {
	.start		= pruproc_start,
	.stop		= pruproc_stop,
	.kick		= pruproc_kick,
};

/* PRU is unregistered */
static int pruproc_remove(struct platform_device *pdev)
{
	struct pruproc *pruproc = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "remove pru\n");

	/* Unregister as remoteproc device */
	rproc_del(pruproc->rproc);
	rproc_put(pruproc->rproc);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

/* Handle probe of a modem device */
static int pruproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pruproc *pruproc;
	struct rproc *rproc;
	struct resource *res;
	struct pinctrl *pinctrl;
	int err;

	dev_dbg(dev, "probe pru\n");

	/* get pinctrl */
	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl)) {
		err = PTR_ERR(pinctrl);
		/* deferring probe */
		if (err == -EPROBE_DEFER) {
			dev_warn(dev, "deferring proble\n");
			return err;
		}
		dev_warn(dev, "pins are not configured from the driver\n");
	}

	/* we only work on OF */
	if (dev->of_node == NULL) {
		dev_err(dev, "Only OF configuration supported\n");
		err = -ENODEV;
		goto fail_of_node;
	}

	pm_runtime_enable(dev);
	err = pm_runtime_get_sync(dev);
	if (err != 0) {
		dev_err(dev, "pm_runtime_get_sync failed\n");
		goto fail_pm_runtime_get_sync;
	}

	err = dma_set_coherent_mask(dev, DMA_BIT_MASK(32));
	if (err) {
		dev_err(dev, "dma_set_coherent_mask: %d\n", err);
		goto fail_dma_set_coherent_mask;
	}

	rproc = rproc_alloc(dev, pdev->name, &pruproc_ops,
			"prutest.bin", sizeof(*pruproc));
	if (!rproc) {
		dev_err(dev, "rproc_alloc failed\n");
		err = -ENOMEM;
		goto fail_rproc_alloc;
	}

	pruproc = rproc->priv;
	pruproc->pdev = pdev;
	pruproc->rproc = rproc;

	platform_set_drvdata(pdev, pruproc);

	/* Set the PRU specific firmware handler */
	rproc->fw_ops = &pruproc_fw_ops;

	/* Register as a remoteproc device */
	err = rproc_add(rproc);
	if (err) {
		dev_err(dev, "rproc_add failed\n");
		goto fail_rproc_add;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(dev, "failed to parse MEM resource\n");
		goto fail_platform_get_resource;
	}

	pruproc->vaddr = devm_ioremap(dev, res->start, resource_size(res));
	if (pruproc->vaddr == NULL) {
		dev_err(dev, "failed to parse MEM resource\n");
		goto fail_devm_ioremap;
	}

	dev_info(dev, "Loaded OK\n");

	return 0;
fail_devm_ioremap:
fail_platform_get_resource:
	rproc_del(rproc);
fail_rproc_add:
	platform_set_drvdata(pdev, NULL);
	rproc_put(rproc);
fail_rproc_alloc:
fail_dma_set_coherent_mask:
fail_of_node:
	pm_runtime_disable(dev);
fail_pm_runtime_get_sync:
	return err;
}

static const struct of_device_id pru_rproc_dt_ids[] = {
	{ .compatible = "ti,pru-rproc", .data = NULL, },
	{},
};
MODULE_DEVICE_TABLE(of, pruss_dt_ids);

static struct platform_driver pruproc_driver = {
	.driver	= {
		.name	= "pru-rproc",
		.owner	= THIS_MODULE,
		.of_match_table = pru_rproc_dt_ids,
	},
	.probe	= pruproc_probe,
	.remove	= pruproc_remove,
};

module_platform_driver(pruproc_driver);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("PRU Remote Processor control driver");
MODULE_AUTHOR("Pantelis Antoniou <panto@antoniou-consulting.com>");
