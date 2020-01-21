// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Subhajit Paul <subahjit_paul@ti.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <linux/rpmsg.h>
#include <linux/rpmsg-remotedev/rpmsg-remotedev.h>

#include "shared/rpmsg-kdrv-transport-demo.h"
#include "rpmsg_kdrv_internal.h"

struct rpmsg_kdrv_demo_private {
	struct rpmsg_kdrv_device *kddev;
	struct rpmsg_remotedev rdev;

	void *data;
	ssize_t data_len;
};

int rpmsg_kdrv_demo_get_data(struct rpmsg_remotedev *rdev, void *data, ssize_t len)
{
	struct rpmsg_kdrv_demo_private *priv = container_of(rdev, struct rpmsg_kdrv_demo_private, rdev);

	if (!data)
		if (!len)
			return priv->data_len;
		else
			return -EINVAL;
	else if (len < priv->data_len)
		return -EINVAL;

	memcpy(data, priv->data, priv->data_len);
	return priv->data_len;
}

int rpmsg_kdrv_demo_ping(struct rpmsg_remotedev *rdev, void *ping_data, ssize_t ping_len)
{
	struct rpmsg_kdrv_demo_private *priv = container_of(rdev, struct rpmsg_kdrv_demo_private, rdev);
	struct rpmsg_kdrv_device *kddev = priv->kddev;
	struct rpmsg_device *rpdev = kddev->rpdev;
	struct rpmsg_kdrv_demodev_ping_request *req;
	struct rpmsg_kdrv_demodev_ping_response *resp;
	int ret;

	if (!ping_len)
		return 0;

	if (ping_len > RPMSG_KDRV_TP_DEMODEV_MESSAGE_DATA_LEN)
		return -EINVAL;

	req = devm_kzalloc(&kddev->dev, sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	resp = devm_kzalloc(&kddev->dev, sizeof(*resp), GFP_KERNEL);
	if (!resp) {
		devm_kfree(&kddev->dev, req);
		return -ENOMEM;
	}

	req->header.message_type = RPMSG_KDRV_TP_DEMODEV_PING_REQUEST;
	memcpy(req->data, ping_data, ping_len);

	ret = rpmsg_kdrv_send_request_with_response(rpdev, kddev->device_id, req, sizeof(*req),
			resp, sizeof(*resp));
	if (ret) {
		dev_err(&kddev->dev, "%s: rpmsg_kdrv_send_request_with_response\n", __func__);
		goto out;
	}


	if (resp->header.message_type != RPMSG_KDRV_TP_DEMODEV_PING_RESPONSE) {
		dev_err(&kddev->dev, "%s: wrong response type\n", __func__);
		ret = -EIO;
		goto out;
	}


	memcpy(ping_data, resp->data, RPMSG_KDRV_TP_DEMODEV_MESSAGE_DATA_LEN);

out:
	devm_kfree(&kddev->dev, req);
	devm_kfree(&kddev->dev, resp);
	return ret;
}

int rpmsg_kdrv_demo_c2s_message(struct rpmsg_remotedev *rdev, void *c2s_msg_data, ssize_t len)
{
	struct rpmsg_kdrv_demo_private *priv = container_of(rdev, struct rpmsg_kdrv_demo_private, rdev);
	struct rpmsg_kdrv_device *kddev = priv->kddev;
	struct rpmsg_device *rpdev = kddev->rpdev;
	struct rpmsg_kdrv_demodev_c2s_message *msg;
	int ret;

	if (!len)
		return 0;

	if (len > RPMSG_KDRV_TP_DEMODEV_MESSAGE_DATA_LEN)
		return -EINVAL;

	msg = devm_kzalloc(&kddev->dev, sizeof(*msg), GFP_KERNEL);
	if (!msg)
		return -ENOMEM;


	msg->header.message_type = RPMSG_KDRV_TP_DEMODEV_C2S_MESSAGE;
	memcpy(msg->data, c2s_msg_data, len);

	ret = rpmsg_kdrv_send_message(rpdev, kddev->device_id, msg, sizeof(*msg));
	if (ret)
		dev_err(&kddev->dev, "%s: rpmsg_kdrv_send_message\n", __func__);

	devm_kfree(&kddev->dev, msg);
	return ret;
}


struct rpmsg_remotedev_demo_ops demo_ops = {
	.get_data = rpmsg_kdrv_demo_get_data,
	.ping = rpmsg_kdrv_demo_ping,
	.c2s_message = rpmsg_kdrv_demo_c2s_message,
};

static void rpmsg_kdrv_demo_device_init(struct rpmsg_kdrv_device *dev, void *data, int len)
{
	struct rpmsg_kdrv_demo_private *priv = dev->driver_private;

	priv->data = devm_kzalloc(&dev->dev, len, GFP_KERNEL);
	if (!priv->data)
		return;

	memcpy(priv->data, data, len);
	priv->data_len = len;
}

static int rpmsg_kdrv_demo_probe(struct rpmsg_kdrv_device *dev)
{
	struct rpmsg_kdrv_demo_private *priv;

	dev_dbg(&dev->dev, "%s\n", __func__);

	priv = devm_kzalloc(&dev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->rdev.type = RPMSG_REMOTEDEV_DEMO_DEVICE;
	priv->rdev.device.demo.ops = &demo_ops;

	priv->kddev = dev;
	dev->driver_private = priv;
	dev->remotedev = &priv->rdev;

	rpmsg_kdrv_demo_device_init(dev, dev->device_data, dev->device_data_len);

	return 0;
}

static void rpmsg_kdrv_demo_remove(struct rpmsg_kdrv_device *dev)
{
	dev_dbg(&dev->dev, "%s\n", __func__);
}

static void rpmsg_kdrv_demo_handle_s2c_message(struct rpmsg_kdrv_device *dev, void *msg, ssize_t len)
{
	struct rpmsg_kdrv_demo_private *priv = dev->driver_private;
	struct rpmsg_remotedev *rdev = &priv->rdev;

	if (rdev->device.demo.cb_ops && rdev->device.demo.cb_ops->s2c_message)
		rdev->device.demo.cb_ops->s2c_message(msg, len, rdev->cb_data);
}

static int rpmsg_kdrv_demo_callback(struct rpmsg_kdrv_device *dev, void *msg, int len)
{
	struct rpmsg_kdrv_demodev_message_header *hdr = msg;

	if (hdr->message_type == RPMSG_KDRV_TP_DEMODEV_S2C_MESSAGE) {
		struct rpmsg_kdrv_demodev_s2c_message *s2c = msg;

		rpmsg_kdrv_demo_handle_s2c_message(dev, s2c->data,
				RPMSG_KDRV_TP_DEMODEV_MESSAGE_DATA_LEN);
	} else
		dev_err(&dev->dev, "%s: unknown message type (%d) for demo device\n", __func__, hdr->message_type);

	return 0;
}


struct rpmsg_kdrv_driver rpmsg_kdrv_demo = {
	.drv.name = "rpmsg-kdrv-demo",
	.device_type = RPMSG_KDRV_TP_DEVICE_TYPE_DEMO,
	.probe = rpmsg_kdrv_demo_probe,
	.remove = rpmsg_kdrv_demo_remove,
	.callback = rpmsg_kdrv_demo_callback,
};

static int __init rpmsg_kdrv_demo_driver_init(void)
{
	return rpmsg_kdrv_register_driver(&rpmsg_kdrv_demo);
}
module_init(rpmsg_kdrv_demo_driver_init);

static void rpmsg_kdrv_demo_driver_fini(void)
{
}
module_exit(rpmsg_kdrv_demo_driver_fini);

MODULE_AUTHOR("Subhajit Paul <subhajit_paul@ti.com>");
MODULE_DESCRIPTION("TI Remote-device Demo Device Driver");
MODULE_LICENSE("GPL v2");
