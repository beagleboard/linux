// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Subhajit Paul <subahjit_paul@ti.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <drm/drm_fourcc.h>
#include <linux/rpmsg.h>
#include <linux/rpmsg-remotedev/rpmsg-remotedev.h>

#include "shared/rpmsg-kdrv-transport-display.h"
#include "rpmsg_kdrv_internal.h"

#define RPMSG_KDRV_DISPLAY_RES_ID_FIRST		(0x10)

struct rpmsg_kdrv_display_private {
	struct rpmsg_kdrv_device *kddev;

	struct rpmsg_remotedev rdev;

	struct idr res_idr;
	struct mutex res_lock;

};

static uint32_t check_min(uint32_t a, uint32_t b, int line)
{
	uint32_t res = min(a, b);

	if (res != b) {
		pr_err("Copy mismatch at Line %d\n", line);
		WARN_ON(1);
	}

	return res;
}

static inline enum rpmsg_kdrv_display_format rpmsg_kdrv_display_fmt_to_rpmsg_fmt(uint32_t in_fmt)
{
	switch (in_fmt) {
	case DRM_FORMAT_ARGB8888:
		return RPMSG_KDRV_TP_DISPLAY_FORMAT_ARGB8888;
	case DRM_FORMAT_XRGB8888:
		return RPMSG_KDRV_TP_DISPLAY_FORMAT_XRGB8888;
	default:
		return RPMSG_KDRV_TP_DISPLAY_FORMAT_MAX;
	}
}

static inline uint32_t rpmsg_kdrv_display_fmt_to_drm_fmt(uint32_t in_fmt)
{
	switch (in_fmt) {
	case RPMSG_KDRV_TP_DISPLAY_FORMAT_ARGB8888:
		return DRM_FORMAT_ARGB8888;
	case RPMSG_KDRV_TP_DISPLAY_FORMAT_XRGB8888:
		return DRM_FORMAT_XRGB8888;
	default:
		return 0;
	}
}

static bool rpmsg_kdrv_display_ready(struct rpmsg_remotedev *rdev)
{
	struct rpmsg_kdrv_display_private *priv = container_of(rdev, struct rpmsg_kdrv_display_private, rdev);
	struct rpmsg_kdrv_device *kddev = priv->kddev;
	struct rpmsg_device *rpdev = kddev->rpdev;
	struct rpmsg_kdrv_display_ready_query_request *req;
	struct rpmsg_kdrv_display_ready_query_response *resp;
	int ret;
	bool retval;

	req = devm_kzalloc(&kddev->dev, sizeof(*req), GFP_KERNEL);
	if (!req)
		return false;

	resp = devm_kzalloc(&kddev->dev, sizeof(*resp), GFP_KERNEL);
	if (!resp) {
		devm_kfree(&kddev->dev, req);
		return false;
	}

	req->header.message_type = RPMSG_KDRV_TP_DISPLAY_READY_QUERY_REQUEST;

	ret = rpmsg_kdrv_send_request_with_response(rpdev, kddev->device_id, req, sizeof(*req), resp, sizeof(*resp));
	if (ret) {
		dev_err(&kddev->dev, "%s: rpmsg_kdrv_send_request_with_response\n", __func__);
		retval = false;
		goto out;
	}

	if (resp->header.message_type != RPMSG_KDRV_TP_DISPLAY_READY_QUERY_RESPONSE) {
		dev_err(&kddev->dev, "%s: wrong response type\n", __func__);
		retval = false;
		goto out;
	}

	retval = resp->ready ? true : false;

out:
	devm_kfree(&kddev->dev, resp);
	devm_kfree(&kddev->dev, req);
	return retval;

}

static void rpmsg_kdrv_display_copy_vid_info(struct rpmsg_remotedev_display_pipe *dst, struct rpmsg_kdrv_display_vid_info *src)
{
	int cnt;
	uint32_t out_fmt;

	dst->pipe_id = src->id;
	dst->can_scale = src->can_scale ? true : false;
	dst->can_mod_win = src->mutable_window ? true : false;
	if (dst->can_mod_win)
		dst->fixed_win_x = dst->fixed_win_y = dst->fixed_win_w = dst->fixed_win_h = 0;
	else {
		dst->fixed_win_x = src->fixed_window_x;
		dst->fixed_win_y = src->fixed_window_y;
		dst->fixed_win_w = src->fixed_window_w;
		dst->fixed_win_h = src->fixed_window_h;
	}
	dst->initial_zorder = src->init_zorder;
	dst->num_formats = check_min(RPMSG_REMOTEDEV_DISPLAY_MAX_FORMATS, src->num_formats, __LINE__);

	dst->num_allowed_zorders = check_min(RPMSG_REMOTEDEV_DISPLAY_MAX_ZORDERS, src->num_zorders, __LINE__);

	for (cnt = 0; cnt < dst->num_formats; cnt++) {
		out_fmt = rpmsg_kdrv_display_fmt_to_drm_fmt(src->format[cnt]);
		WARN_ON(out_fmt == 0);
		dst->formats[cnt] = out_fmt;
	}

	for (cnt = 0; cnt < dst->num_allowed_zorders; cnt++)
		dst->allowed_zorders[cnt] = src->zorder[cnt];
}

static void rpmsg_kdrv_display_copy_vp_info(struct rpmsg_remotedev_display_disp *dst, struct rpmsg_kdrv_display_vp_info *src)
{
	int vidcnt;

	dst->disp_id = src->id;
	dst->width = src->width;
	dst->height = src->height;
	dst->refresh = src->refresh;
	dst->num_pipes = check_min(RPMSG_REMOTEDEV_DISPLAY_MAX_PIPES, src->num_vids, __LINE__);

	for (vidcnt = 0; vidcnt < dst->num_pipes; vidcnt++)
		rpmsg_kdrv_display_copy_vid_info(&dst->pipes[vidcnt], &src->vid[vidcnt]);
}

static int rpmsg_kdrv_display_get_res(struct rpmsg_remotedev *rdev, struct rpmsg_remotedev_display_resinfo *res)
{
	struct rpmsg_kdrv_display_private *priv = container_of(rdev, struct rpmsg_kdrv_display_private, rdev);
	struct rpmsg_kdrv_device *kddev = priv->kddev;
	struct rpmsg_device *rpdev = kddev->rpdev;
	struct rpmsg_kdrv_display_res_info_request *req;
	struct rpmsg_kdrv_display_res_info_response *resp;
	int ret, vpcnt;

	req = devm_kzalloc(&kddev->dev, sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	resp = devm_kzalloc(&kddev->dev, sizeof(*resp), GFP_KERNEL);
	if (!resp) {
		devm_kfree(&kddev->dev, req);
		return -ENOMEM;
	}

	req->header.message_type = RPMSG_KDRV_TP_DISPLAY_RES_INFO_REQUEST;

	ret = rpmsg_kdrv_send_request_with_response(rpdev, kddev->device_id, req, sizeof(*req), resp, sizeof(*resp));
	if (ret) {
		dev_err(&kddev->dev, "%s: rpmsg_kdrv_send_request_with_response\n", __func__);
		goto out;
	}

	if (resp->header.message_type != RPMSG_KDRV_TP_DISPLAY_RES_INFO_RESPONSE) {
		dev_err(&kddev->dev, "%s: wrong response type\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	res->num_disps = check_min(RPMSG_REMOTEDEV_DISPLAY_MAX_DISPS, resp->num_vps, __LINE__);

	for (vpcnt = 0; vpcnt < res->num_disps; vpcnt++)
		rpmsg_kdrv_display_copy_vp_info(&res->disps[vpcnt], &resp->vp[vpcnt]);

out:
	devm_kfree(&kddev->dev, resp);
	devm_kfree(&kddev->dev, req);
	return ret;
}

static uint32_t rpmsg_kdrv_display_res_id_new(struct rpmsg_kdrv_device *kddev, void *data)
{
	struct rpmsg_kdrv_display_private *priv = kddev->driver_private;
	int id;

	mutex_lock(&priv->res_lock);
	id = idr_alloc(&priv->res_idr, data, RPMSG_KDRV_DISPLAY_RES_ID_FIRST, 0, GFP_KERNEL);
	mutex_unlock(&priv->res_lock);

	if (id < 0)
		return 0;

	return id;
}

static void rpmsg_kdrv_display_free_res_id(struct rpmsg_kdrv_device *kddev, uint32_t id)
{
	struct rpmsg_kdrv_display_private *priv = kddev->driver_private;

	mutex_lock(&priv->res_lock);
	idr_remove(&priv->res_idr, id);
	mutex_unlock(&priv->res_lock);
}

static void rpmsg_kdrv_free_request_res(struct rpmsg_kdrv_device *kddev, struct rpmsg_kdrv_display_commit_request *req)
{
	int i;

	rpmsg_kdrv_display_free_res_id(kddev, req->commit_id);

	for (i = 0; i < req->num_vid_updates; i++)
		if (req->vid[i].enabled)
			rpmsg_kdrv_display_free_res_id(kddev, req->vid[i].buffer.buffer_id);

}

static bool rpmsg_kdrv_display_copy_buffer(struct rpmsg_kdrv_device *kddev, struct rpmsg_kdrv_display_buffer_info *dst,
		struct rpmsg_remotedev_display_buffer *src)
{
	int i;

	dst->width = src->width;
	dst->height = src->height;

	dst->format = rpmsg_kdrv_display_fmt_to_rpmsg_fmt(src->format);
	if (WARN_ON(dst->format == RPMSG_KDRV_TP_DISPLAY_FORMAT_MAX))
		return false;

	dst->num_planes = check_min(RPMSG_KDRV_TP_DISPLAY_MAX_PLANES, src->num_planes, __LINE__);
	if (dst->num_planes != src->num_planes)
		return false;

	for (i = 0; i < dst->num_planes; i++) {
		dst->plane[i] = (uint64_t)src->planes[i];
		dst->pitch[i] = src->pitches[i];
	}

	dst->buffer_id = rpmsg_kdrv_display_res_id_new(kddev, src);
	if (!dst->buffer_id)
		return false;

	return true;
}

static bool rpmsg_kdrv_display_copy_vid_commit(struct rpmsg_kdrv_device *kddev, struct rpmsg_kdrv_display_vid_update_info *dst,
		struct rpmsg_remotedev_display_pipe_update *src)
{
	dst->id = src->pipe_id;
	dst->enabled = src->enabled ? 1 : 0;
	if (dst->enabled) {
		dst->dst_w = src->dst_w;
		dst->dst_h = src->dst_h;
		dst->dst_x = src->dst_x;
		dst->dst_y = src->dst_y;

		if (!rpmsg_kdrv_display_copy_buffer(kddev, &dst->buffer, src->buffer))
			return false;
	}

	return true;
}

static bool rpmsg_kdrv_display_copy_commit(struct rpmsg_kdrv_device *kddev, struct rpmsg_kdrv_display_commit_request *dst,
		struct rpmsg_remotedev_display_commit *src)
{
	int i, copied_vids;

	dst->id = src->disp_id;
	dst->num_vid_updates = check_min(RPMSG_KDRV_TP_DISPLAY_MAX_VIDS, src->num_pipe_updates, __LINE__);

	for (i = 0, copied_vids = 0; i < dst->num_vid_updates; i++, copied_vids++)
		if (!rpmsg_kdrv_display_copy_vid_commit(kddev, &dst->vid[i], &src->pipes[i]))
			goto free_vid_res;

	dst->commit_id = rpmsg_kdrv_display_res_id_new(kddev, src);
	if (!dst->commit_id)
		goto free_vid_res;

	return true;

free_vid_res:
	for (i = 0; i < copied_vids; i++)
		if (dst->vid[i].enabled)
			rpmsg_kdrv_display_free_res_id(kddev, dst->vid[i].buffer.buffer_id);
	return false;

}

static int rpmsg_kdrv_display_commit(struct rpmsg_remotedev *rdev, struct rpmsg_remotedev_display_commit *commit)
{
	struct rpmsg_kdrv_display_private *priv = container_of(rdev, struct rpmsg_kdrv_display_private, rdev);
	struct rpmsg_kdrv_device *kddev = priv->kddev;
	struct rpmsg_device *rpdev = kddev->rpdev;
	struct rpmsg_kdrv_display_commit_request *req;
	struct rpmsg_kdrv_display_commit_response *resp;
	int ret;

	req = devm_kzalloc(&kddev->dev, sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	resp = devm_kzalloc(&kddev->dev, sizeof(*resp), GFP_KERNEL);
	if (!resp) {
		devm_kfree(&kddev->dev, req);
		return -ENOMEM;
	}

	req->header.message_type = RPMSG_KDRV_TP_DISPLAY_COMMIT_REQUEST;

	if (!rpmsg_kdrv_display_copy_commit(kddev, req, commit)) {
		dev_err(&kddev->dev, "%s: failed to copy commit request\n", __func__);
		ret = -ENOMEM;
		goto out;
	}

	ret = rpmsg_kdrv_send_request_with_response(rpdev, kddev->device_id, req, sizeof(*req),
			resp, sizeof(*resp));
	if (ret) {
		dev_err(&kddev->dev, "%s: rpmsg_kdrv_send_request_with_response\n", __func__);
		goto nosend;
	}


	if (resp->header.message_type != RPMSG_KDRV_TP_DISPLAY_COMMIT_RESPONSE) {
		dev_err(&kddev->dev, "%s: wrong response type\n", __func__);
		goto out;
	}

	ret = ((resp->status == 0) ? 0 : -EINVAL);
	goto out;

nosend:
	rpmsg_kdrv_free_request_res(kddev, req);
out:
	devm_kfree(&kddev->dev, req);
	devm_kfree(&kddev->dev, resp);
	return ret;
}


struct rpmsg_remotedev_display_ops disp_ops = {
	.ready = rpmsg_kdrv_display_ready,
	.get_res_info = rpmsg_kdrv_display_get_res,
	.commit = rpmsg_kdrv_display_commit,
};

static void rpmsg_kdrv_display_device_init(struct rpmsg_kdrv_device *kddev, void *data, int len)
{
}

static int rpmsg_kdrv_display_probe(struct rpmsg_kdrv_device *dev)
{
	struct rpmsg_kdrv_display_private *priv;

	dev_dbg(&dev->dev, "%s\n", __func__);

	priv = devm_kzalloc(&dev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->rdev.type = RPMSG_REMOTEDEV_DISPLAY_DEVICE;
	priv->rdev.device.display.ops = &disp_ops;

	mutex_init(&priv->res_lock);
	idr_init(&priv->res_idr);

	priv->kddev = dev;
	dev->driver_private = priv;
	dev->remotedev = &priv->rdev;

	rpmsg_kdrv_display_device_init(dev, dev->device_data, dev->device_data_len);

	return 0;
}

static void rpmsg_kdrv_display_remove(struct rpmsg_kdrv_device *dev)
{
	dev_dbg(&dev->dev, "%s\n", __func__);
}

static void rpmsg_kdrv_display_handle_commit(struct rpmsg_kdrv_device *dev, struct rpmsg_kdrv_display_commit_done_message *msg)
{
	struct rpmsg_kdrv_display_private *priv = dev->driver_private;
	struct rpmsg_remotedev *rdev = &priv->rdev;
	struct rpmsg_remotedev_display_commit *commit;

	mutex_lock(&priv->res_lock);
	commit = idr_find(&priv->res_idr, msg->commit_id);
	idr_remove(&priv->res_idr, msg->commit_id);
	mutex_unlock(&priv->res_lock);

	if (!commit) {
		dev_err(&dev->dev, "%s: no pending commit found\n", __func__);
		return;
	}

	if (rdev->device.display.cb_ops && rdev->device.display.cb_ops->commit_done)
		rdev->device.display.cb_ops->commit_done(commit, rdev->cb_data);
}

static void rpmsg_kdrv_display_handle_buffer(struct rpmsg_kdrv_device *dev, struct rpmsg_kdrv_display_buffer_done_message *msg)
{
	struct rpmsg_kdrv_display_private *priv = dev->driver_private;
	struct rpmsg_remotedev *rdev = &priv->rdev;
	struct rpmsg_remotedev_display_buffer *buffer;

	mutex_lock(&priv->res_lock);
	buffer = idr_find(&priv->res_idr, msg->buffer_id);
	idr_remove(&priv->res_idr, msg->buffer_id);
	mutex_unlock(&priv->res_lock);

	if (!buffer) {
		dev_err(&dev->dev, "%s: no pending buffer found\n", __func__);
		return;
	}

	if (rdev->device.display.cb_ops && rdev->device.display.cb_ops->buffer_done)
		rdev->device.display.cb_ops->buffer_done(buffer, rdev->cb_data);
}

static int rpmsg_kdrv_display_callback(struct rpmsg_kdrv_device *dev, void *msg, int len)
{
	struct rpmsg_kdrv_display_message_header *hdr = msg;

	if (hdr->message_type == RPMSG_KDRV_TP_DISPLAY_COMMIT_DONE_MESSAGE)
		rpmsg_kdrv_display_handle_commit(dev, msg);
	else if (hdr->message_type == RPMSG_KDRV_TP_DISPLAY_BUFFER_DONE_MESSAGE)
		rpmsg_kdrv_display_handle_buffer(dev, msg);

	return 0;
}


struct rpmsg_kdrv_driver rpmsg_kdrv_display = {
	.drv.name = "rpmsg-kdrv-display",
	.device_type = RPMSG_KDRV_TP_DEVICE_TYPE_DISPLAY,
	.probe = rpmsg_kdrv_display_probe,
	.remove = rpmsg_kdrv_display_remove,
	.callback = rpmsg_kdrv_display_callback,
};

static int __init rpmsg_kdrv_display_driver_init(void)
{
	return rpmsg_kdrv_register_driver(&rpmsg_kdrv_display);
}
module_init(rpmsg_kdrv_display_driver_init);

static void rpmsg_kdrv_display_driver_fini(void)
{
}
module_exit(rpmsg_kdrv_display_driver_fini);

MODULE_AUTHOR("Subhajit Paul <subhajit_paul@ti.com>");
MODULE_DESCRIPTION("TI Remote-device Virtual Display Driver");
MODULE_LICENSE("GPL v2");
