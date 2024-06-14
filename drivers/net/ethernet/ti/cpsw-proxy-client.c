// SPDX-License-Identifier: GPL-2.0-only or MIT
/* Texas Instruments CPSW Proxy Client Driver
 *
 * Copyright (C) 2024 Texas Instruments Incorporated - https://www.ti.com/
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/rpmsg.h>

#include "ethfw_abi.h"

struct cpsw_proxy_priv {
	struct rpmsg_device		*rpdev;
	struct device			*dev;
};

static int cpsw_proxy_client_cb(struct rpmsg_device *rpdev, void *data,
				int len, void *priv, u32 src)
{
	struct device *dev = &rpdev->dev;

	dev_dbg(dev, "callback invoked\n");

	return 0;
}

static int cpsw_proxy_client_probe(struct rpmsg_device *rpdev)
{
	struct cpsw_proxy_priv *proxy_priv;

	proxy_priv = devm_kzalloc(&rpdev->dev, sizeof(struct cpsw_proxy_priv), GFP_KERNEL);
	if (!proxy_priv)
		return -ENOMEM;

	proxy_priv->rpdev = rpdev;
	proxy_priv->dev = &rpdev->dev;
	dev_dbg(proxy_priv->dev, "driver probed\n");

	return 0;
}

static void cpsw_proxy_client_remove(struct rpmsg_device *rpdev)
{
	struct device *dev = &rpdev->dev;

	dev_dbg(dev, "driver removed\n");
}

static struct rpmsg_device_id cpsw_proxy_client_id_table[] = {
	{
		.name = ETHFW_SERVICE_EP_NAME,
	},
	{},
};
MODULE_DEVICE_TABLE(rpmsg, cpsw_proxy_client_id_table);

static struct rpmsg_driver cpsw_proxy_client_driver = {
	.drv.name	= KBUILD_MODNAME,
	.id_table	= cpsw_proxy_client_id_table,
	.probe		= cpsw_proxy_client_probe,
	.callback	= cpsw_proxy_client_cb,
	.remove		= cpsw_proxy_client_remove,
};
module_rpmsg_driver(cpsw_proxy_client_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CPSW Proxy Client Driver");
MODULE_AUTHOR("Siddharth Vadapalli <s-vadapalli@ti.com>");
