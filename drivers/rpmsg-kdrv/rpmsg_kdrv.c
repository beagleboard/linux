// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Subhajit Paul <subahjit_paul@ti.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/mutex.h>

#include <linux/rpmsg.h>
#include "shared/rpmsg-kdrv-transport.h"
#include "rpmsg_kdrv_internal.h"

struct rpmsg_kdrv_priv {
	struct rpmsg_device *rpdev;

	struct idr message_idr;
	struct mutex message_lock;

	int num_raw_devices;
	struct rpmsg_kdrv_init_device_info raw_devices[RPMSG_KDRV_TP_MAX_DEVICES];
	void *raw_device_data[RPMSG_KDRV_TP_MAX_DEVICES];
	int raw_device_data_size[RPMSG_KDRV_TP_MAX_DEVICES];
};

struct rpmsg_kdrv_ctx {
	struct rpmsg_device *rpdev;
	bool wait_for_response;
	request_cb_t callback;
	void *cb_data;
	bool response_recv;
	struct wait_queue_head response_wq;

	struct rpmsg_kdrv_device_header *dev_hdr;
	void *req;
	void *resp;
	int req_size;
	int resp_size;
};

static struct bus_type rpmsg_kdrv_bus;

#define to_rpmsg_kdrv_device(d) container_of(d, struct rpmsg_kdrv_device, dev)
#define to_rpmsg_kdrv_driver(d) container_of(d, struct rpmsg_kdrv_driver, drv)

static int rpmsg_kdrv_match_id(struct device *dev, const void *data)
{
	const uint32_t *idptr = data;
	struct rpmsg_kdrv_device *kddev = container_of(dev, struct rpmsg_kdrv_device, dev);

	if (kddev->device_id == *idptr)
		return 1;
	return 0;
}

static int rpmsg_kdrv_match_name(struct device *dev, const void *data)
{
	const char *name = data;
	struct rpmsg_kdrv_device *kddev = container_of(dev, struct rpmsg_kdrv_device, dev);

	if (strcmp(kddev->device_name, name) == 0)
		return 1;
	return 0;
}

int rpmsg_kdrv_register_driver(struct rpmsg_kdrv_driver *drv)
{
	int ret;

	drv->drv.bus = &rpmsg_kdrv_bus;
	drv->drv.owner = THIS_MODULE;

	ret = driver_register(&drv->drv);
	if (ret)
		pr_err("%s: driver_register failed\n", __func__);

	return ret;
}
EXPORT_SYMBOL(rpmsg_kdrv_register_driver);

static void rpmsg_kdrv_driver_handle_data(struct rpmsg_device *rpdev, void *data, int len, void *private, u32 src)
{
	struct device *dev;
	struct rpmsg_kdrv_device_header *hdr = data;
	struct rpmsg_kdrv_device *kddev = NULL;
	struct rpmsg_kdrv_driver *kddrv = NULL;
	void *message;
	int message_size;
	uint32_t msg_device_id;
	int ret;

	msg_device_id = hdr->device_id;
	dev = bus_find_device(&rpmsg_kdrv_bus, NULL, &(msg_device_id), rpmsg_kdrv_match_id);
	if (!dev) {
		dev_err(&rpdev->dev, "%s: message received for unknown device\n", __func__);
		return;
	}
	kddev = container_of(dev, struct rpmsg_kdrv_device, dev);
	kddrv = to_rpmsg_kdrv_driver(kddev->dev.driver);
	if (!kddrv) {
		dev_err(&rpdev->dev, "%s: message received for device with no driver\n", __func__);
		return;
	}

	message = (void *)(&hdr[1]);
	message_size = len - sizeof(*hdr);
	ret = kddrv->callback(kddev, message, message_size);
	if (ret)
		dev_err(&rpdev->dev, "%s: message callback returns %d\n", __func__, ret);

}

static int rpmsg_kdrv_connect(struct rpmsg_device *rpdev, struct rpmsg_kdrv_device *kddev)
{
	int ret;
	struct rpmsg_kdrv_init_connect_message *connect_req;

	connect_req = devm_kzalloc(&rpdev->dev, sizeof(*connect_req), GFP_KERNEL);
	if (!connect_req)
		return -ENOMEM;

	connect_req->header.message_type = RPMSG_KDRV_TP_INIT_CONNECT_MESSAGE;
	connect_req->device_id = kddev->device_id;

	ret = rpmsg_kdrv_send_message(rpdev, RPMSG_KDRV_TP_DEVICE_ID_INIT,
			connect_req, sizeof(*connect_req));

	devm_kfree(&rpdev->dev, connect_req);
	return ret;
}

static int rpmsg_kdrv_disconnect(struct rpmsg_device *rpdev, struct rpmsg_kdrv_device *kddev)
{
	int ret;
	struct rpmsg_kdrv_init_disconnect_message *disconnect_req;

	disconnect_req = devm_kzalloc(&rpdev->dev, sizeof(*disconnect_req), GFP_KERNEL);
	if (!disconnect_req)
		return -ENOMEM;

	disconnect_req->header.message_type = RPMSG_KDRV_TP_INIT_DISCONNECT_MESSAGE;
	disconnect_req->device_id = kddev->device_id;

	ret = rpmsg_kdrv_send_message(rpdev, RPMSG_KDRV_TP_DEVICE_ID_INIT,
			disconnect_req, sizeof(*disconnect_req));

	devm_kfree(&rpdev->dev, disconnect_req);
	return ret;
}

static void rpmsg_kdrv_release_device(struct device *dev)
{
	struct rpmsg_kdrv_device *kddev = to_rpmsg_kdrv_device(dev);

	dev_dbg(dev, "%s\n", __func__);

	devm_kfree(&kddev->rpdev->dev, kddev);
}

static struct rpmsg_kdrv_device *rpmsg_kdrv_device_create(struct rpmsg_device *rpdev, int index)
{
	struct rpmsg_kdrv_device *kddev = devm_kzalloc(&rpdev->dev, sizeof(*kddev), GFP_KERNEL);
	struct rpmsg_kdrv_priv *priv = dev_get_drvdata(&rpdev->dev);
	struct rpmsg_kdrv_init_device_info *dev = &priv->raw_devices[index];
	int ret;

	if (!kddev) {
		dev_err(&rpdev->dev, "%s: could not allocate kddev\n", __func__);
		return NULL;
	}

	kddev->rpdev = rpdev;
	kddev->device_id = dev->device_id;
	kddev->device_type = dev->device_type;
	kddev->device_data_len = priv->raw_device_data_size[index];
	kddev->device_data = priv->raw_device_data[index];
	kddev->device_name = devm_kstrdup(&rpdev->dev, dev->device_name, GFP_KERNEL);
	if (!kddev->device_name) {
		dev_err(&rpdev->dev, "%s: could not allocate device name\n", __func__);
		devm_kfree(&rpdev->dev, kddev);
		return NULL;
	}

	kddev->dev.parent = &rpdev->dev;
	kddev->dev.release = rpmsg_kdrv_release_device;
	kddev->dev.bus = &rpmsg_kdrv_bus;

	dev_set_name(&kddev->dev, "rpmsg-kdrv-%u-%s", dev->device_id, dev->device_name);

	ret = device_register(&kddev->dev);
	if (ret) {
		dev_err(&rpdev->dev, "%s: device_register failed: %d\n", __func__, ret);
		put_device(&kddev->dev);
		return NULL;
	}
	dev_dbg(&rpdev->dev, "%s: registered new device : %s\n", __func__, dev_name(&kddev->dev));

	return kddev;
}

static int rpmsg_kdrv_get_devices_cb(void *cb_data, void *req, int req_sz, void *resp, int resp_sz)
{
	int i, cnt;
	struct rpmsg_device *rpdev = cb_data;
	struct rpmsg_kdrv_priv *priv = dev_get_drvdata(&rpdev->dev);
	struct rpmsg_kdrv_init_dev_info_response *info_resp = resp;
	struct rpmsg_kdrv_init_device_info *dev;
	int ret;

	if (info_resp->header.message_type != RPMSG_KDRV_TP_INIT_DEV_INFO_RESPONSE) {
		dev_err(&rpdev->dev, "%s: wrong response type\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	for (i = 0; i < info_resp->num_devices; i++) {
		dev = &info_resp->devices[i];
		cnt = priv->num_raw_devices;

		priv->raw_device_data_size[cnt] = dev->device_data_len;
		priv->raw_device_data[cnt] = devm_kzalloc(&rpdev->dev, dev->device_data_len, GFP_KERNEL);
		if (!priv->raw_device_data[cnt])
			goto out;
		memcpy(priv->raw_device_data[cnt],
				&info_resp->device_data[dev->device_data_offset],
				dev->device_data_len);
		memcpy(&priv->raw_devices[cnt], dev, sizeof(*dev));
		priv->num_raw_devices++;

		dev_dbg(&rpdev->dev, "new device: %s\n", dev->device_name);
	}

	for (i = 0; i < priv->num_raw_devices; i++)
		rpmsg_kdrv_device_create(rpdev, i);

out:
	devm_kfree(&rpdev->dev, req);
	return ret;
}

static int rpmsg_kdrv_get_devices(struct rpmsg_device *rpdev)
{
	int ret;
	struct rpmsg_kdrv_init_dev_info_request *info_req;

	info_req = devm_kzalloc(&rpdev->dev, sizeof(*info_req), GFP_KERNEL);
	if (!info_req)
		return -ENOMEM;

	info_req->header.message_type = RPMSG_KDRV_TP_INIT_DEV_INFO_REQUEST;

	ret = rpmsg_kdrv_send_request_with_callback(rpdev, RPMSG_KDRV_TP_DEVICE_ID_INIT,
			info_req, sizeof(*info_req), rpdev, rpmsg_kdrv_get_devices_cb);
	if (ret)
		goto nosend;

	return 0;

nosend:
	devm_kfree(&rpdev->dev, info_req);
	return ret;
}

static void rpmsg_kdrv_del_packet_id(struct rpmsg_device *rpdev, int id)
{
	struct rpmsg_kdrv_priv *priv = dev_get_drvdata(&rpdev->dev);

	mutex_lock(&priv->message_lock);
	idr_remove(&priv->message_idr, id);
	mutex_unlock(&priv->message_lock);
}

static uint32_t rpmsg_kdrv_new_packet_id(struct rpmsg_device *rpdev, void *data)
{
	struct rpmsg_kdrv_priv *priv = dev_get_drvdata(&rpdev->dev);
	int id;

	mutex_lock(&priv->message_lock);
	id = idr_alloc(&priv->message_idr, data, RPMSG_KDRV_TP_PACKET_ID_FIRST, 0, GFP_KERNEL);
	mutex_unlock(&priv->message_lock);

	if (id < 0)
		return 0;

	return id;
}

static void rpmsg_kdrv_dev_hdr_delete(struct rpmsg_device *rpdev, struct rpmsg_kdrv_device_header *hdr)
{
	rpmsg_kdrv_del_packet_id(rpdev, hdr->packet_id);
	devm_kfree(&rpdev->dev, hdr);
}

static struct rpmsg_kdrv_device_header *rpmsg_kdrv_dev_hdr_alloc(struct rpmsg_device *rpdev,
		int device_id, int size, int pkt_type, int pkt_src, void *msg, int len, struct rpmsg_kdrv_ctx *ctx)
{
	struct rpmsg_kdrv_device_header *dev_hdr;
	void *dst;

	dev_hdr = devm_kzalloc(&rpdev->dev, size, GFP_KERNEL);
	if (!dev_hdr)
		return NULL;

	dev_hdr->device_id = device_id;
	dev_hdr->packet_type = pkt_type;
	dev_hdr->packet_source = pkt_src;
	dev_hdr->packet_size = size;
	dev_hdr->packet_id = RPMSG_KDRV_TP_PACKET_ID_NONE;


	dst = (void *)(&dev_hdr[1]);
	memcpy(dst, msg, len);

	if (pkt_type == RPMSG_KDRV_TP_PACKET_TYPE_MESSAGE)
		return dev_hdr;

	dev_hdr->packet_id = rpmsg_kdrv_new_packet_id(rpdev, ctx);
	if (!dev_hdr->packet_id) {
		devm_kfree(&rpdev->dev, dev_hdr);
		return NULL;
	}

	ctx->dev_hdr = dev_hdr;

	return dev_hdr;
}

static struct rpmsg_kdrv_ctx *rpmsg_kdrv_ctx_alloc(struct rpmsg_device *rpdev, bool blocking,
		request_cb_t callback, void *cb_data, void *req, int req_size, void *resp, int resp_size)
{
	struct rpmsg_kdrv_ctx *ctx;

	ctx = devm_kzalloc(&rpdev->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return NULL;

	ctx->rpdev = rpdev;
	if (blocking) {
		ctx->wait_for_response = true;
		ctx->response_recv = false;
		init_waitqueue_head(&ctx->response_wq);
	} else {
		ctx->wait_for_response = false;
		ctx->callback = callback;
	}

	ctx->cb_data = cb_data;
	ctx->req = req;
	ctx->req_size = req_size;
	ctx->resp = resp;
	ctx->resp_size = resp_size;

	return ctx;
}

static int rpmsg_kdrv_send_packet(struct rpmsg_device *rpdev, void *data, int len)
{
	return rpmsg_send(rpdev->ept, data, len);
}

/*
 * rpmsg_kdrv_send_request_with_callback
 *
 * Send a message where
 * a) the caller does not block
 * b) the caller expects multile responses
 *
 * The callback function must return
 * a) RRMSG_KDRV_CALLBACK_DONE when no more responses are expected
 * b) RPMSG_KDRV_CALLBACK_MORE when more responses are awaited
 *
 * The caller is expected to destroy message when it does not
 * expect any more responses
 */
int rpmsg_kdrv_send_request_with_callback(struct rpmsg_device *rpdev, uint32_t device_id,
		void *message, uint32_t message_size,
		void *cb_data, request_cb_t callback)
{
	struct rpmsg_kdrv_device_header *dev_hdr;
	int total_size = message_size + sizeof(*dev_hdr);
	struct rpmsg_kdrv_ctx *ctx = NULL;
	int ret;

	ctx = rpmsg_kdrv_ctx_alloc(rpdev, false, callback, cb_data, message, message_size, NULL, 0);
	if (!ctx) {
		dev_err(&rpdev->dev, "%s: ctx allocation failed\n", __func__);
		return -ENOMEM;
	}

	dev_hdr = rpmsg_kdrv_dev_hdr_alloc(rpdev, device_id, total_size,
			RPMSG_KDRV_TP_PACKET_TYPE_REQUEST,
			RPMSG_KDRV_TP_PACKET_SOURCE_CLIENT,
			message, message_size,
			ctx);
	if (!dev_hdr) {
		dev_err(&rpdev->dev, "%s: device header allocation failed\n", __func__);
		ret = -ENOMEM;
		goto dev_hdr_fail;
	}

	ret = rpmsg_kdrv_send_packet(rpdev, dev_hdr, total_size);
	if (ret) {
		dev_err(&rpdev->dev, "rpmsg_send failed: %d\n", ret);
		goto nosend;
	}

	return 0;

nosend:
	rpmsg_kdrv_dev_hdr_delete(rpdev, dev_hdr);
dev_hdr_fail:
	devm_kfree(&rpdev->dev, ctx);
	return ret;
}
EXPORT_SYMBOL(rpmsg_kdrv_send_request_with_callback);

/*
 * rpmsg_kdrv_send_request_with_response
 *
 * Send a message where the caller will block for a response
 *
 * The caller is expected to destroy message and response
 * when this function returns
 */
int rpmsg_kdrv_send_request_with_response(struct rpmsg_device *rpdev, uint32_t device_id,
		void *message, uint32_t message_size,
		void *response, uint32_t response_size)
{
	struct rpmsg_kdrv_device_header *dev_hdr;
	int total_size = message_size + sizeof(*dev_hdr);
	struct rpmsg_kdrv_ctx *ctx = NULL;
	int ret;

	ctx = rpmsg_kdrv_ctx_alloc(rpdev, true, NULL, NULL, message, message_size, response, response_size);
	if (!ctx) {
		dev_err(&rpdev->dev, "%s: ctx allocation failed\n", __func__);
		return -ENOMEM;
	}

	dev_hdr = rpmsg_kdrv_dev_hdr_alloc(rpdev, device_id, total_size,
			RPMSG_KDRV_TP_PACKET_TYPE_REQUEST,
			RPMSG_KDRV_TP_PACKET_SOURCE_CLIENT,
			message, message_size,
			ctx);
	if (!dev_hdr) {
		dev_err(&rpdev->dev, "%s: device header allocation failed\n", __func__);
		ret = -ENOMEM;
		goto dev_hdr_fail;
	}

	ret = rpmsg_kdrv_send_packet(rpdev, dev_hdr, total_size);
	if (ret) {
		dev_err(&rpdev->dev, "rpmsg_send failed: %d\n", ret);
		goto nosend;
	}

	wait_event(ctx->response_wq, ctx->response_recv == true);

nosend:
	rpmsg_kdrv_dev_hdr_delete(rpdev, dev_hdr);
dev_hdr_fail:
	devm_kfree(&rpdev->dev, ctx);
	return ret;
}
EXPORT_SYMBOL(rpmsg_kdrv_send_request_with_response);

/*
 * rpmsg_kdrv_send_message
 *
 * Send a message and dont expect a response
 *
 * The caller is expected to destroy message when
 * this function returns
 */
int rpmsg_kdrv_send_message(struct rpmsg_device *rpdev, uint32_t device_id,
		void *message, uint32_t message_size)
{
	struct rpmsg_kdrv_device_header *dev_hdr;
	int total_size = message_size + sizeof(*dev_hdr);
	int ret;

	/* We dont need a ctx for direct messages */

	dev_hdr = rpmsg_kdrv_dev_hdr_alloc(rpdev, device_id, total_size,
			RPMSG_KDRV_TP_PACKET_TYPE_MESSAGE,
			RPMSG_KDRV_TP_PACKET_SOURCE_CLIENT,
			message, message_size,
			NULL);
	if (!dev_hdr) {
		dev_err(&rpdev->dev, "%s: device header allocation failed\n", __func__);
		return -ENOMEM;
	}

	ret = rpmsg_kdrv_send_packet(rpdev, dev_hdr, total_size);
	if (ret) {
		dev_err(&rpdev->dev, "%s: rpmsg_send failed: %d\n", __func__, ret);
		goto out;
	}

out:
	rpmsg_kdrv_dev_hdr_delete(rpdev, dev_hdr);
	return ret;
}
EXPORT_SYMBOL(rpmsg_kdrv_send_message);

static int rpmsg_kdrv_cb(struct rpmsg_device *rpdev, void *data, int len,
						void *private, u32 src)
{
	struct rpmsg_kdrv_priv *priv = dev_get_drvdata(&rpdev->dev);
	struct rpmsg_kdrv_device_header *hdr = data;
	struct rpmsg_kdrv_message_header *msg;
	int msg_len;
	struct rpmsg_kdrv_ctx *ctx;
	int ret;

	if (hdr->packet_type != RPMSG_KDRV_TP_PACKET_TYPE_RESPONSE) {
		rpmsg_kdrv_driver_handle_data(rpdev, data, len, private, src);
		return 0;
	}

	mutex_lock(&priv->message_lock);
	ctx = idr_find(&priv->message_idr, hdr->packet_id);
	mutex_unlock(&priv->message_lock);

	if (!ctx) {
		dev_err(&rpdev->dev, "%s: response received with no pending request\n", __func__);
		return 0;
	}

	msg = (struct rpmsg_kdrv_message_header *)((void *)(&hdr[1]));
	msg_len = len - sizeof(*hdr);

	/* process callback if expected */
	if (ctx->callback) {
		ret = ctx->callback(ctx->cb_data, ctx->req, ctx->req_size, msg, msg_len);
		if (ret == RRMSG_KDRV_CALLBACK_DONE) {
			/* No need to keep the ctx alive */
			rpmsg_kdrv_dev_hdr_delete(rpdev, ctx->dev_hdr);
			devm_kfree(&rpdev->dev, ctx);
		}
		return 0;
	}

	/* copy the response and wake up caller, caller will destroy ctx & dev_hdr */
	memcpy(ctx->resp, msg, min(msg_len, ctx->resp_size));

	ctx->response_recv = true;
	wake_up(&ctx->response_wq);

	return 0;
}

static int rpmsg_kdrv_dev_match(struct device *dev, struct device_driver *drv)
{
	struct rpmsg_kdrv_device *kddev = to_rpmsg_kdrv_device(dev);
	struct rpmsg_kdrv_driver *kddrv = to_rpmsg_kdrv_driver(drv);

	if (kddrv->device_type == kddev->device_type) {
		dev_dbg(dev, "%s: matching with driver %s\n", __func__, drv->name);
		return 1;
	}

	dev_dbg(dev, "%s: does not match driver %s\n", __func__, drv->name);
	return 0;
}

static int rpmsg_kdrv_dev_probe(struct device *dev)
{
	struct rpmsg_kdrv_device *kddev = to_rpmsg_kdrv_device(dev);
	struct rpmsg_kdrv_driver *kddrv = to_rpmsg_kdrv_driver(kddev->dev.driver);
	int ret;

	dev_dbg(dev, "%s: probe\n", __func__);

	ret = kddrv->probe(kddev);
	if (ret) {
		dev_err(dev, "%s: child probe failed\n", __func__);
		return ret;
	}

	return 0;
}

static int rpmsg_kdrv_dev_remove(struct device *dev)
{
	struct rpmsg_kdrv_device *kddev = to_rpmsg_kdrv_device(dev);
	struct rpmsg_kdrv_driver *kddrv = to_rpmsg_kdrv_driver(kddev->dev.driver);

	dev_dbg(dev, "%s: remove\n", __func__);

	kddrv->remove(kddev);
	return 0;
}

static int rpmsg_kdrv_probe(struct rpmsg_device *rpdev)
{
	int ret;
	struct rpmsg_kdrv_priv *priv;

	dev_dbg(&rpdev->dev, "%s: probing rpmsg kdrv driver\n", __func__);

	priv = devm_kzalloc(&rpdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	dev_set_drvdata(&rpdev->dev, priv);
	priv->rpdev = rpdev;

	idr_init(&priv->message_idr);
	mutex_init(&priv->message_lock);

	dev_dbg(&rpdev->dev, "%s: sending device info request\n", __func__);
	ret = rpmsg_kdrv_get_devices(rpdev);
	if (ret) {
		dev_err(&rpdev->dev, "%s: error collecting device info\n", __func__);
		goto out;
	}

	return 0;

out:
	dev_set_drvdata(&rpdev->dev, NULL);
	devm_kfree(&rpdev->dev, priv);
	return ret;
}

static void rpmsg_kdrv_remove(struct rpmsg_device *rpdev)
{
	dev_dbg(&rpdev->dev, "removing rpmsg kdrv driver\n");

	/* TODO check for pending responses for any of the child devices */
	/* TODO disconnect them all */
}

static struct bus_type rpmsg_kdrv_bus = {
	.name		= "rpmsg_kdrv",
	.match		= rpmsg_kdrv_dev_match,
	.probe		= rpmsg_kdrv_dev_probe,
	.remove		= rpmsg_kdrv_dev_remove,
};

static struct rpmsg_device_id rpmsg_kdrv_id_table[] = {
	{ .name	= "rpmsg-kdrv" },
	{ },
};

static struct rpmsg_driver rpmsg_kdrv = {
	.drv.name	= "rpmsg-kdrv",
	.id_table	= rpmsg_kdrv_id_table,
	.probe		= rpmsg_kdrv_probe,
	.callback	= rpmsg_kdrv_cb,
	.remove		= rpmsg_kdrv_remove,
};

static int __init rpmsg_kdrv_init(void)
{
	int ret;

	ret = bus_register(&rpmsg_kdrv_bus);
	if (ret) {
		pr_err("failed to register rpmsg kdrv bus: %d\n", ret);
		goto out;
	}

	ret = register_rpmsg_driver(&rpmsg_kdrv);
	if (ret) {
		pr_err("failed to register rpmsg kdrv driver: %d\n", ret);
		goto rpdrv_fail;
	}

	pr_debug("registered rpmsg kdrv driver\n");

	return 0;

rpdrv_fail:
	bus_unregister(&rpmsg_kdrv_bus);
out:
	return ret;
}
module_init(rpmsg_kdrv_init);

static void __exit rpmsg_kdrv_fini(void)
{
	pr_debug("unregistering rpmsg kdrv driver\n");

	unregister_rpmsg_driver(&rpmsg_kdrv);
	bus_unregister(&rpmsg_kdrv_bus);
}
module_exit(rpmsg_kdrv_fini);

MODULE_AUTHOR("Subhajit Paul <subhajit_paul@ti.com>");
MODULE_DESCRIPTION("TI Remote-device framework Driver");
MODULE_LICENSE("GPL v2");
