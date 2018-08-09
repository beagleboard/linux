// SPDX-License-Identifier: GPL-2.0
/*
 *  Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com
 *  Author: Peter Ujfalusi <peter.ujfalusi@ti.com>
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/soc/ti/k3-navss-psilcfg.h>
#include <linux/soc/ti/ti_sci_protocol.h>

#define PSIL_DST_THREAD_MARK		0x8000

struct k3_nav_psil_priv {
	struct device *dev;
	const struct ti_sci_handle *tisci;
	const struct ti_sci_rm_psil_ops *tisci_psil_ops;
	u32  tisci_dev_id;

	struct list_head psil_links;
	/* psil link list protection */
	struct mutex lock;
};

struct k3_nav_psil_entry {
	struct list_head node;
	struct k3_nav_psil_priv *priv;

	u32 src_thread;
	u32 dst_thread;
};

struct k3_nav_psil_entry *k3_nav_psil_request_link(
					struct device_node *psilcfg_node,
					u32 src_thread, u32 dst_thread)
{
	struct k3_nav_psil_entry *link;
	struct platform_device *pdev;
	struct k3_nav_psil_priv *priv;
	int ret;

	pdev = of_find_device_by_node(psilcfg_node);
	if (!pdev) {
		pr_err("device not found\n");
		return ERR_PTR(-EPROBE_DEFER);
	}

	priv = platform_get_drvdata(pdev);
	if (!priv) {
		pr_err("driver data not available\n");
		return ERR_PTR(-EPROBE_DEFER);
	}

	link = kzalloc(sizeof(*link), GFP_KERNEL);
	if (!link)
		return ERR_PTR(-ENOMEM);

	link->src_thread = src_thread;
	link->dst_thread = (dst_thread | PSIL_DST_THREAD_MARK);
	link->priv = priv;

	mutex_lock(&priv->lock);

	ret = priv->tisci_psil_ops->pair(priv->tisci, priv->tisci_dev_id,
					 link->src_thread, link->dst_thread);
	if (ret) {
		kfree(link);
		link = ERR_PTR(ret);
	} else {
		list_add(&link->node, &priv->psil_links);
	}

	mutex_unlock(&priv->lock);

	return link;
}
EXPORT_SYMBOL_GPL(k3_nav_psil_request_link);

int k3_nav_psil_release_link(struct k3_nav_psil_entry *link)
{
	struct k3_nav_psil_priv *priv = link->priv;
	int ret;

	mutex_lock(&priv->lock);

	list_del(&link->node);
	ret = priv->tisci_psil_ops->unpair(priv->tisci, priv->tisci_dev_id,
					   link->src_thread, link->dst_thread);
	mutex_unlock(&priv->lock);

	kfree(link);
	return ret;
}
EXPORT_SYMBOL_GPL(k3_nav_psil_release_link);

struct device_node *of_k3_nav_psil_get_by_phandle(struct device_node *np,
						  const char *property)
{
	struct device_node *psilcfg_node;
	struct platform_device *pdev;
	struct k3_nav_psil_priv *priv;

	psilcfg_node = of_parse_phandle(np, property, 0);
	if (!psilcfg_node)
		return ERR_PTR(-ENODEV);

	pdev = of_find_device_by_node(psilcfg_node);
	if (!pdev)
		return ERR_PTR(-EPROBE_DEFER);

	priv = platform_get_drvdata(pdev);
	if (!priv)
		return ERR_PTR(-EPROBE_DEFER);

	return psilcfg_node;
}
EXPORT_SYMBOL_GPL(of_k3_nav_psil_get_by_phandle);

static const struct of_device_id k3_nav_psil_of_match[] = {
	{ .compatible = "ti,k3-navss-psilcfg", },
	{},
};
MODULE_DEVICE_TABLE(of, k3_nav_psil_of_match);

static int k3_nav_psil_probe(struct platform_device *pdev)
{
	struct k3_nav_psil_priv *priv;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->tisci = ti_sci_get_by_phandle(pdev->dev.of_node, "ti,sci");
	if (IS_ERR(priv->tisci)) {
		if (PTR_ERR(priv->tisci) != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Failed to get tisci %ld\n",
				PTR_ERR(priv->tisci));
		return PTR_ERR(priv->tisci);
	}

	ret = of_property_read_u32(pdev->dev.of_node, "ti,sci-dev-id",
				   &priv->tisci_dev_id);
	if (ret) {
		dev_err(&pdev->dev, "ti,sci-dev-id read failure %d\n", ret);
		return ret;
	}

	priv->tisci_psil_ops = &priv->tisci->ops.rm_psil_ops;

	INIT_LIST_HEAD(&priv->psil_links);
	mutex_init(&priv->lock);

	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

	dev_info(&pdev->dev, "NAVSS PSI-L manager\n");

	priv->dev = &pdev->dev;
	platform_set_drvdata(pdev, priv);

	return 0;
}

static int k3_nav_psil_remove(struct platform_device *pdev)
{
	struct k3_nav_psil_priv *priv = platform_get_drvdata(pdev);

	if (!list_empty(&priv->psil_links))
		return -EBUSY;

	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

static struct platform_driver k3_nav_psil_driver = {
	.driver = {
		.name	= "k3-navss-psilcfg",
		.of_match_table = k3_nav_psil_of_match,
	},
	.probe		= k3_nav_psil_probe,
	.remove		= k3_nav_psil_remove,
};

module_platform_driver(k3_nav_psil_driver);

MODULE_ALIAS("platform:k3-navss-psilcfg");
MODULE_DESCRIPTION("TI K3 NAVSS PSI-L interface driver");
MODULE_AUTHOR("Peter Ujfalusi <peter.ujfalusi@ti.com>");
MODULE_LICENSE("GPL v2");
