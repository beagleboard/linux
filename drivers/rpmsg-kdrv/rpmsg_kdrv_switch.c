// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Grygorii Strashko <grygorii.strashko@ti.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/rpmsg.h>
#include <linux/rpmsg-remotedev/rpmsg-remotedev.h>

#include "rpmsg_kdrv_internal.h"
#include "shared/rpmsg-kdrv-transport-switch.h"

struct rpmsg_kdrv_switch_private {
	struct rpmsg_kdrv_device *kddev;
	struct rpmsg_remotedev rdev;
	u64	session_id;
	u32	core_key;
	u32	permissions;
	u32	uart_id;
	u32	attached:1;
	u32	uart_connected:1;
};

static bool
rpmsg_kdrv_switch_check_perm(struct rpmsg_kdrv_switch_private *priv,
			     enum rpmsg_kdrv_ethswitch_message_type msg_type)
{
	struct rpmsg_kdrv_device *kddev = priv->kddev;

	if (priv->permissions & BIT(msg_type))
		return true;

	dev_err_ratelimited(&kddev->dev, "permission denied msg: 0x%02X\n",
			    msg_type);
	return false;
}

static int
rpmsg_kdrv_switch_check_resp_status(struct rpmsg_kdrv_ethswitch_common_resp_info *info)
{
	if (info->status == RPMSG_KDRV_TP_ETHSWITCH_CMDSTATUS_EAGAIN)
		return -EAGAIN;
	if (info->status == RPMSG_KDRV_TP_ETHSWITCH_CMDSTATUS_EFAIL)
		return -EIO;
	return 0;
}

static int rpmsg_kdrv_switch_ping(struct rpmsg_remotedev *rdev,
				  const u8 *data, int size)
{
	struct rpmsg_kdrv_switch_private *priv =
		container_of(rdev, struct rpmsg_kdrv_switch_private, rdev);
	struct rpmsg_kdrv_device *kddev = priv->kddev;
	struct rpmsg_device *rpdev = kddev->rpdev;
	struct rpmsg_kdrv_ethswitch_ping_req *req;
	struct rpmsg_kdrv_ethswitch_ping_resp *resp;
	int ret;

	if (!rpmsg_kdrv_switch_check_perm(priv,
					  RPMSG_KDRV_TP_ETHSWITCH_PING_REQUEST))
		return -EPERM;

	req = devm_kzalloc(&kddev->dev, sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	resp = devm_kzalloc(&kddev->dev, sizeof(*resp), GFP_KERNEL);
	if (!resp) {
		devm_kfree(&kddev->dev, req);
		return -ENOMEM;
	}

	if (size > RPMSG_KDRV_TP_ETHSWITCH_MESSAGE_DATA_LEN)
		size = RPMSG_KDRV_TP_ETHSWITCH_MESSAGE_DATA_LEN;

	req->header.message_type = RPMSG_KDRV_TP_ETHSWITCH_PING_REQUEST;
	memcpy(req->data, data, size);

	ret = rpmsg_kdrv_send_request_with_response(rpdev, kddev->device_id,
						    req, sizeof(*req),
						    resp, sizeof(*resp));
	if (ret) {
		dev_dbg(&kddev->dev, "%s: send: %d\n", __func__, ret);
		goto out;
	}

	if (memcmp(req->data, resp->data, size)) {
		dev_dbg(&kddev->dev, "%s: ping fail - data\n", __func__);
		ret = -EINVAL;
	}

out:
	devm_kfree(&kddev->dev, resp);
	devm_kfree(&kddev->dev, req);
	return ret;
}

static int
rpmsg_kdrv_switch_attach(struct rpmsg_remotedev *rdev,
			 struct rpmsg_rdev_eth_switch_attach_info *attach_info)
{
	struct rpmsg_kdrv_switch_private *priv =
		container_of(rdev, struct rpmsg_kdrv_switch_private, rdev);
	struct rpmsg_kdrv_device *kddev = priv->kddev;
	struct rpmsg_device *rpdev = kddev->rpdev;
	struct rpmsg_kdrv_ethswitch_attach_req *req;
	struct rpmsg_kdrv_ethswitch_attach_resp *resp;
	int ret;

	if (priv->attached)
		return -EBUSY;

	req = devm_kzalloc(&kddev->dev, sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	resp = devm_kzalloc(&kddev->dev, sizeof(*resp), GFP_KERNEL);
	if (!resp) {
		devm_kfree(&kddev->dev, req);
		return -ENOMEM;
	}

	req->header.message_type = RPMSG_KDRV_TP_ETHSWITCH_ATTACH;
	req->cpsw_type = RPMSG_KDRV_TP_ETHSWITCH_CPSWTYPE_MAIN_CPSW;

	ret = rpmsg_kdrv_send_request_with_response(rpdev, kddev->device_id,
						    req, sizeof(*req),
						    resp, sizeof(*resp));
	if (ret) {
		dev_dbg(&kddev->dev, "%s: send: %d\n", __func__, ret);
		goto out;
	}

	ret = rpmsg_kdrv_switch_check_resp_status(&resp->info);
	if (ret)
		goto out;

	priv->session_id = resp->id;
	priv->core_key = resp->core_key;
	priv->attached = true;

	dev_dbg(&kddev->dev, "%s: done id:%llX core_key:%08X\n",
		__func__, priv->session_id, priv->core_key);

	attach_info->rx_mtu = resp->rx_mtu;
	memcpy(attach_info->tx_mtu, resp->tx_mtu, sizeof(attach_info->tx_mtu));
	attach_info->features = resp->features;
	if (priv->uart_connected)
		attach_info->features |=
				RPMSG_KDRV_ETHSWITCH_FEATURE_DUMP_STATS;

out:
	devm_kfree(&kddev->dev, resp);
	devm_kfree(&kddev->dev, req);
	return ret;
}

static int
rpmsg_kdrv_switch_attach_ext(struct rpmsg_remotedev *rdev,
			     struct rpmsg_rdev_eth_switch_attach_ext_info *attach_ext_info)
{
	struct rpmsg_kdrv_switch_private *priv =
		container_of(rdev, struct rpmsg_kdrv_switch_private, rdev);
	struct rpmsg_kdrv_device *kddev = priv->kddev;
	struct rpmsg_device *rpdev = kddev->rpdev;
	struct rpmsg_kdrv_ethswitch_attach_extended_req *req;
	struct rpmsg_kdrv_ethswitch_attach_extended_resp *resp;
	int ret;

	if (priv->attached)
		return -EBUSY;

	req = devm_kzalloc(&kddev->dev, sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	resp = devm_kzalloc(&kddev->dev, sizeof(*resp), GFP_KERNEL);
	if (!resp) {
		devm_kfree(&kddev->dev, req);
		return -ENOMEM;
	}

	req->header.message_type = RPMSG_KDRV_TP_ETHSWITCH_ATTACH_EXT;
	req->cpsw_type = RPMSG_KDRV_TP_ETHSWITCH_CPSWTYPE_MAIN_CPSW;

	ret = rpmsg_kdrv_send_request_with_response(rpdev, kddev->device_id,
						    req, sizeof(*req),
						    resp, sizeof(*resp));
	if (ret) {
		dev_dbg(&kddev->dev, "%s: send: %d\n", __func__, ret);
		goto out;
	}

	ret = rpmsg_kdrv_switch_check_resp_status(&resp->info);
	if (ret)
		goto out;

	priv->session_id = resp->id;
	priv->core_key = resp->core_key;
	priv->attached = true;

	dev_dbg(&kddev->dev, "%s: done id:%llX core_key:%08X\n",
		__func__, priv->session_id, priv->core_key);

	attach_ext_info->rx_mtu = resp->rx_mtu;
	memcpy(attach_ext_info->tx_mtu, resp->tx_mtu,
	       sizeof(attach_ext_info->tx_mtu));
	attach_ext_info->features = resp->features;
	if (priv->uart_connected)
		attach_ext_info->features |=
				RPMSG_KDRV_ETHSWITCH_FEATURE_DUMP_STATS;
	attach_ext_info->flow_idx = resp->alloc_flow_idx;
	attach_ext_info->tx_cpsw_psil_dst_id = resp->tx_cpsw_psil_dst_id;
	ether_addr_copy(attach_ext_info->mac_addr, resp->mac_address);

out:
	devm_kfree(&kddev->dev, resp);
	devm_kfree(&kddev->dev, req);
	return ret;
}

static int rpmsg_kdrv_switch_detach(struct rpmsg_remotedev *rdev)
{
	struct rpmsg_kdrv_switch_private *priv =
		container_of(rdev, struct rpmsg_kdrv_switch_private, rdev);
	struct rpmsg_kdrv_device *kddev = priv->kddev;
	struct rpmsg_device *rpdev = kddev->rpdev;
	struct rpmsg_kdrv_ethswitch_detach_req *req;
	struct rpmsg_kdrv_ethswitch_detach_resp *resp;
	int ret;

	if (!priv->attached)
		return -EINVAL;

	req = devm_kzalloc(&kddev->dev, sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	resp = devm_kzalloc(&kddev->dev, sizeof(*resp), GFP_KERNEL);
	if (!resp) {
		devm_kfree(&kddev->dev, req);
		return -ENOMEM;
	}

	req->header.message_type = RPMSG_KDRV_TP_ETHSWITCH_DETACH;
	req->info.id = priv->session_id;
	req->info.core_key = priv->core_key;

	ret = rpmsg_kdrv_send_request_with_response(rpdev, kddev->device_id,
						    req, sizeof(*req),
						    resp, sizeof(*resp));
	if (ret) {
		dev_dbg(&kddev->dev, "%s: send: %d\n", __func__, ret);
		goto out;
	}

	ret = rpmsg_kdrv_switch_check_resp_status(&resp->info);

	priv->attached = false;

	dev_dbg(&kddev->dev, "%s: done ret:%d\n", __func__, ret);

out:
	devm_kfree(&kddev->dev, resp);
	devm_kfree(&kddev->dev, req);
	return ret;
}

static int
rpmsg_kdrv_switch_get_tx_info(struct rpmsg_remotedev *rdev,
			      struct rpmsg_rdev_eth_switch_tx_info *info)
{
	struct rpmsg_kdrv_switch_private *priv =
		container_of(rdev, struct rpmsg_kdrv_switch_private, rdev);
	struct rpmsg_kdrv_device *kddev = priv->kddev;
	struct rpmsg_device *rpdev = kddev->rpdev;
	struct rpmsg_kdrv_ethswitch_alloc_req *req;
	struct rpmsg_kdrv_ethswitch_alloc_tx_resp *resp;
	int ret;

	if (!priv->attached)
		return -EINVAL;

	req = devm_kzalloc(&kddev->dev, sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	resp = devm_kzalloc(&kddev->dev, sizeof(*resp), GFP_KERNEL);
	if (!resp) {
		devm_kfree(&kddev->dev, req);
		return -ENOMEM;
	}

	req->header.message_type = RPMSG_KDRV_TP_ETHSWITCH_ALLOC_TX;
	req->info.id = priv->session_id;
	req->info.core_key = priv->core_key;

	ret = rpmsg_kdrv_send_request_with_response(rpdev, kddev->device_id,
						    req, sizeof(*req),
						    resp, sizeof(*resp));
	if (ret) {
		dev_dbg(&kddev->dev, "%s: send: %d\n", __func__, ret);
		goto out;
	}

	ret = rpmsg_kdrv_switch_check_resp_status(&resp->info);
	if (ret)
		goto out;

	info->tx_cpsw_psil_dst_id = resp->tx_cpsw_psil_dst_id;

	dev_dbg(&kddev->dev, "%s: done\n", __func__);

out:
	devm_kfree(&kddev->dev, resp);
	devm_kfree(&kddev->dev, req);
	return ret;
}

static int
rpmsg_kdrv_switch_get_rx_info(struct rpmsg_remotedev *rdev,
			      struct rpmsg_rdev_eth_switch_rx_info *info)
{
	struct rpmsg_kdrv_switch_private *priv =
		container_of(rdev, struct rpmsg_kdrv_switch_private, rdev);
	struct rpmsg_kdrv_device *kddev = priv->kddev;
	struct rpmsg_device *rpdev = kddev->rpdev;
	struct rpmsg_kdrv_ethswitch_alloc_req *req;
	struct rpmsg_kdrv_ethswitch_alloc_rx_resp *resp;
	int ret;

	if (!priv->attached)
		return -EINVAL;

	req = devm_kzalloc(&kddev->dev, sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	resp = devm_kzalloc(&kddev->dev, sizeof(*resp), GFP_KERNEL);
	if (!resp) {
		devm_kfree(&kddev->dev, req);
		return -ENOMEM;
	}

	req->header.message_type = RPMSG_KDRV_TP_ETHSWITCH_ALLOC_RX;
	req->info.id = priv->session_id;
	req->info.core_key = priv->core_key;

	ret = rpmsg_kdrv_send_request_with_response(rpdev, kddev->device_id,
						    req, sizeof(*req),
						    resp, sizeof(*resp));
	if (ret) {
		dev_dbg(&kddev->dev, "%s: send: %d\n", __func__, ret);
		goto out;
	}

	ret = rpmsg_kdrv_switch_check_resp_status(&resp->info);
	if (ret)
		goto out;

	info->flow_idx = resp->alloc_flow_idx;

	dev_dbg(&kddev->dev, "%s: done\n", __func__);

out:
	devm_kfree(&kddev->dev, resp);
	devm_kfree(&kddev->dev, req);
	return ret;
}

static int rpmsg_kdrv_switch_get_mac(struct rpmsg_remotedev *rdev,
				     void *mac_addr)
{
	struct rpmsg_kdrv_switch_private *priv =
		container_of(rdev, struct rpmsg_kdrv_switch_private, rdev);
	struct rpmsg_kdrv_device *kddev = priv->kddev;
	struct rpmsg_device *rpdev = kddev->rpdev;
	struct rpmsg_kdrv_ethswitch_alloc_req *req;
	struct rpmsg_kdrv_ethswitch_alloc_mac_resp *resp;
	int ret;

	if (!priv->attached)
		return -EINVAL;

	req = devm_kzalloc(&kddev->dev, sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	resp = devm_kzalloc(&kddev->dev, sizeof(*resp), GFP_KERNEL);
	if (!resp) {
		devm_kfree(&kddev->dev, req);
		return -ENOMEM;
	}

	req->header.message_type = RPMSG_KDRV_TP_ETHSWITCH_ALLOC_MAC;
	req->info.id = priv->session_id;
	req->info.core_key = priv->core_key;

	ret = rpmsg_kdrv_send_request_with_response(rpdev, kddev->device_id,
						    req, sizeof(*req),
						    resp, sizeof(*resp));
	if (ret) {
		dev_dbg(&kddev->dev, "%s: send: %d\n", __func__, ret);
		goto out;
	}

	ret = rpmsg_kdrv_switch_check_resp_status(&resp->info);
	if (ret)
		goto out;

	ether_addr_copy(mac_addr, resp->mac_address);

	dev_dbg(&kddev->dev, "%s: done\n", __func__);

out:
	devm_kfree(&kddev->dev, resp);
	devm_kfree(&kddev->dev, req);
	return ret;
}

static int rpmsg_kdrv_switch_register_mac(struct rpmsg_remotedev *rdev,
					  void *mac_addr, u32 flow_idx_offset)
{
	struct rpmsg_kdrv_switch_private *priv =
		container_of(rdev, struct rpmsg_kdrv_switch_private, rdev);
	struct rpmsg_kdrv_device *kddev = priv->kddev;
	struct rpmsg_device *rpdev = kddev->rpdev;
	struct rpmsg_kdrv_ethswitch_register_mac_req *req;
	struct rpmsg_kdrv_ethswitch_register_mac_resp *resp;
	int ret;

	if (!priv->attached)
		return -EINVAL;

	req = devm_kzalloc(&kddev->dev, sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	resp = devm_kzalloc(&kddev->dev, sizeof(*resp), GFP_KERNEL);
	if (!resp) {
		devm_kfree(&kddev->dev, req);
		return -ENOMEM;
	}

	req->header.message_type = RPMSG_KDRV_TP_ETHSWITCH_REGISTER_MAC;
	req->info.id = priv->session_id;
	req->info.core_key = priv->core_key;
	ether_addr_copy(req->mac_address, mac_addr);
	req->flow_idx = flow_idx_offset;

	ret = rpmsg_kdrv_send_request_with_response(rpdev, kddev->device_id,
						    req, sizeof(*req),
						    resp, sizeof(*resp));
	if (ret) {
		dev_dbg(&kddev->dev, "%s: send: %d\n", __func__, ret);
		goto out;
	}

	ret = rpmsg_kdrv_switch_check_resp_status(&resp->info);

	dev_dbg(&kddev->dev, "%s: done\n", __func__);

out:
	devm_kfree(&kddev->dev, resp);
	devm_kfree(&kddev->dev, req);
	return ret;
}

static int
rpmsg_kdrv_switch_unregister_mac(struct rpmsg_remotedev *rdev,
				 void *mac_addr, u32 flow_idx_offset)
{
	struct rpmsg_kdrv_switch_private *priv =
		container_of(rdev, struct rpmsg_kdrv_switch_private, rdev);
	struct rpmsg_kdrv_device *kddev = priv->kddev;
	struct rpmsg_device *rpdev = kddev->rpdev;
	struct rpmsg_kdrv_ethswitch_unregister_mac_req *req;
	struct rpmsg_kdrv_ethswitch_unregister_mac_resp *resp;
	int ret;

	if (!priv->attached)
		return -EINVAL;

	req = devm_kzalloc(&kddev->dev, sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	resp = devm_kzalloc(&kddev->dev, sizeof(*resp), GFP_KERNEL);
	if (!resp) {
		devm_kfree(&kddev->dev, req);
		return -ENOMEM;
	}

	req->header.message_type = RPMSG_KDRV_TP_ETHSWITCH_UNREGISTER_MAC;
	req->info.id = priv->session_id;
	req->info.core_key = priv->core_key;
	ether_addr_copy(req->mac_address, mac_addr);
	req->flow_idx = flow_idx_offset;

	ret = rpmsg_kdrv_send_request_with_response(rpdev, kddev->device_id,
						    req, sizeof(*req),
						    resp, sizeof(*resp));
	if (ret) {
		dev_dbg(&kddev->dev, "%s: send: %d\n", __func__, ret);
		goto out;
	}

	ret = rpmsg_kdrv_switch_check_resp_status(&resp->info);

	dev_dbg(&kddev->dev, "%s: done\n", __func__);

out:
	devm_kfree(&kddev->dev, resp);
	devm_kfree(&kddev->dev, req);
	return ret;
}

static int rpmsg_kdrv_switch_reg_ipv4(struct rpmsg_remotedev *rdev,
				      void *mac_addr, __be32 ipv4)
{
	struct rpmsg_kdrv_switch_private *priv =
		container_of(rdev, struct rpmsg_kdrv_switch_private, rdev);
	struct rpmsg_kdrv_device *kddev = priv->kddev;
	struct rpmsg_device *rpdev = kddev->rpdev;
	struct rpmsg_kdrv_ethswitch_ipv4_register_mac_req *req;
	struct rpmsg_kdrv_ethswitch_ipv4_register_mac_resp *resp;
	int ret;

	if (!priv->attached)
		return -EINVAL;

	req = devm_kzalloc(&kddev->dev, sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	resp = devm_kzalloc(&kddev->dev, sizeof(*resp), GFP_KERNEL);
	if (!resp) {
		devm_kfree(&kddev->dev, req);
		return -ENOMEM;
	}

	req->header.message_type = RPMSG_KDRV_TP_ETHSWITCH_IPV4_MAC_REGISTER;
	req->info.id = priv->session_id;
	req->info.core_key = priv->core_key;
	ether_addr_copy(req->mac_address, mac_addr);
	memcpy(req->ipv4_addr, &ipv4, RPMSG_KDRV_TP_ETHSWITCH_IPV4ADDRLEN);

	ret = rpmsg_kdrv_send_request_with_response(rpdev, kddev->device_id,
						    req, sizeof(*req),
						    resp, sizeof(*resp));
	if (ret) {
		dev_dbg(&kddev->dev, "%s: send: %d\n", __func__, ret);
		goto out;
	}

	ret = rpmsg_kdrv_switch_check_resp_status(&resp->info);

	dev_dbg(&kddev->dev, "%s: done\n", __func__);

out:
	devm_kfree(&kddev->dev, resp);
	devm_kfree(&kddev->dev, req);
	return ret;
}

static int rpmsg_kdrv_switch_unreg_ipv4(struct rpmsg_remotedev *rdev,
					__be32 ipv4)
{
	struct rpmsg_kdrv_switch_private *priv =
		container_of(rdev, struct rpmsg_kdrv_switch_private, rdev);
	struct rpmsg_kdrv_device *kddev = priv->kddev;
	struct rpmsg_device *rpdev = kddev->rpdev;
	struct rpmsg_kdrv_ethswitch_ipv4_unregister_mac_req *req;
	struct rpmsg_kdrv_ethswitch_ipv4_unregister_mac_resp *resp;
	int ret;

	if (!priv->attached)
		return -EINVAL;

	req = devm_kzalloc(&kddev->dev, sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	resp = devm_kzalloc(&kddev->dev, sizeof(*resp), GFP_KERNEL);
	if (!resp) {
		devm_kfree(&kddev->dev, req);
		return -ENOMEM;
	}

	req->header.message_type = RPMSG_KDRV_TP_ETHSWITCH_IPV4_MAC_UNREGISTER;
	req->info.id = priv->session_id;
	req->info.core_key = priv->core_key;
	memcpy(req->ipv4_addr, &ipv4, RPMSG_KDRV_TP_ETHSWITCH_IPV4ADDRLEN);

	ret = rpmsg_kdrv_send_request_with_response(rpdev, kddev->device_id,
						    req, sizeof(*req),
						    resp, sizeof(*resp));
	if (ret) {
		dev_dbg(&kddev->dev, "%s: send: %d\n", __func__, ret);
		goto out;
	}

	ret = rpmsg_kdrv_switch_check_resp_status(&resp->info);

	dev_dbg(&kddev->dev, "%s: done\n", __func__);

out:
	devm_kfree(&kddev->dev, resp);
	devm_kfree(&kddev->dev, req);
	return ret;
}

static int rpmsg_kdrv_switch_reg_read(struct rpmsg_remotedev *rdev,
				      u32 reg_addr, u32 *val)
{
	struct rpmsg_kdrv_switch_private *priv =
		container_of(rdev, struct rpmsg_kdrv_switch_private, rdev);
	struct rpmsg_kdrv_device *kddev = priv->kddev;
	struct rpmsg_device *rpdev = kddev->rpdev;
	struct rpmsg_kdrv_ethswitch_regrd_req *req;
	struct rpmsg_kdrv_ethswitch_regrd_resp *resp;
	int ret;

	if (!rpmsg_kdrv_switch_check_perm(priv, RPMSG_KDRV_TP_ETHSWITCH_REGRD))
		return -EPERM;

	req = devm_kzalloc(&kddev->dev, sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	resp = devm_kzalloc(&kddev->dev, sizeof(*resp), GFP_KERNEL);
	if (!resp) {
		devm_kfree(&kddev->dev, req);
		return -ENOMEM;
	}

	req->header.message_type = RPMSG_KDRV_TP_ETHSWITCH_REGRD;
	req->regaddr = reg_addr;

	ret = rpmsg_kdrv_send_request_with_response(rpdev, kddev->device_id,
						    req, sizeof(*req),
						    resp, sizeof(*resp));
	if (ret) {
		dev_dbg(&kddev->dev, "%s: send: %d\n", __func__, ret);
		goto out;
	}

	ret = rpmsg_kdrv_switch_check_resp_status(&resp->info);
	if (ret)
		goto out;

	*val = resp->regval;
	dev_dbg(&kddev->dev, "%s: done\n", __func__);

out:
	devm_kfree(&kddev->dev, resp);
	devm_kfree(&kddev->dev, req);
	return ret;
}

static int rpmsg_kdrv_switch_c2s_dbg_dump_stats(struct rpmsg_remotedev *rdev)
{
	struct rpmsg_kdrv_switch_private *priv =
		container_of(rdev, struct rpmsg_kdrv_switch_private, rdev);
	struct rpmsg_kdrv_device *kddev = priv->kddev;
	struct rpmsg_device *rpdev = kddev->rpdev;
	struct rpmsg_kdrv_ethswitch_c2s_notify *req;
	int ret;

	if (!priv->attached)
		return -EINVAL;

	req = devm_kzalloc(&kddev->dev, sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	req->header.message_type = RPMSG_KDRV_TP_ETHSWITCH_C2S_NOTIFY;
	req->info.id = priv->session_id;
	req->info.core_key = priv->core_key;
	req->notifyid = RPMSG_KDRV_TP_ETHSWITCH_CLIENTNOTIFY_DUMPSTATS;

	ret = rpmsg_kdrv_send_message(rpdev, kddev->device_id,
				      req, sizeof(*req));

	dev_dbg(&kddev->dev, "%s: done ret:%d\n", __func__, ret);
	devm_kfree(&kddev->dev, req);
	return ret;
}

static void rpmsg_kdrv_switch_get_fw_ver(struct rpmsg_remotedev *rdev,
					 char *buf, size_t size)
{
	struct rpmsg_kdrv_switch_private *priv =
		container_of(rdev, struct rpmsg_kdrv_switch_private, rdev);
	struct rpmsg_kdrv_ethswitch_fw_version_info *fw_info;
	struct rpmsg_kdrv_ethswitch_device_data *kddev_data;

	kddev_data = priv->kddev->device_data;
	fw_info = &kddev_data->fw_ver;

	snprintf(buf, size, "%u.%u.%u %.*s/%.*s/%.*s SHA:%.*s",
		 fw_info->major, fw_info->minor, fw_info->rev,
		 RPMSG_KDRV_TP_ETHSWITCH_DATELEN, fw_info->date,
		 RPMSG_KDRV_TP_ETHSWITCH_MONTHLEN, fw_info->month,
		 RPMSG_KDRV_TP_ETHSWITCH_YEARLEN, fw_info->year,
		 RPMSG_KDRV_TP_ETHSWITCH_COMMITSHALEN, fw_info->commit_hash);
}

static struct rpmsg_remotedev_eth_switch_ops switch_ops = {
	.get_fw_ver = rpmsg_kdrv_switch_get_fw_ver,
	.attach = rpmsg_kdrv_switch_attach,
	.attach_ext = rpmsg_kdrv_switch_attach_ext,
	.detach = rpmsg_kdrv_switch_detach,
	.get_tx_info = rpmsg_kdrv_switch_get_tx_info,
	.get_rx_info = rpmsg_kdrv_switch_get_rx_info,
	.get_mac = rpmsg_kdrv_switch_get_mac,
	.register_mac = rpmsg_kdrv_switch_register_mac,
	.unregister_mac = rpmsg_kdrv_switch_unregister_mac,
	.register_ipv4 = rpmsg_kdrv_switch_reg_ipv4,
	.unregister_ipv4 = rpmsg_kdrv_switch_unreg_ipv4,
	.ping = rpmsg_kdrv_switch_ping,
	.read_reg = rpmsg_kdrv_switch_reg_read,
	.dbg_dump_stats = rpmsg_kdrv_switch_c2s_dbg_dump_stats,
};

static int rpmsg_kdrv_switch_callback(struct rpmsg_kdrv_device *dev,
				      void *msg, int len)
{
	return 0;
}

static int
rpmsg_kdrv_switch_dev_data_parse(struct rpmsg_kdrv_device *kddev,
				 void *data, int len,
				 struct rpmsg_kdrv_switch_private *priv)
{
	struct rpmsg_kdrv_ethswitch_device_data *kddev_data = data;
	struct rpmsg_kdrv_ethswitch_fw_version_info *fw_info;

	if (sizeof(*kddev_data) != len)
		return -EINVAL;

	dev_info(&kddev->dev, "Device info: permissions: %08X uart_id: %d\n",
		 kddev_data->permission_flags,
		 kddev_data->uart_connected ? kddev_data->uart_id : -1);

	fw_info = &kddev_data->fw_ver;

	dev_info(&kddev->dev, "FW ver %u.%u (rev %u) %.*s/%.*s/%.*s SHA:%.*s\n",
		 fw_info->major, fw_info->minor, fw_info->rev,
		 RPMSG_KDRV_TP_ETHSWITCH_DATELEN, fw_info->date,
		 RPMSG_KDRV_TP_ETHSWITCH_MONTHLEN, fw_info->month,
		 RPMSG_KDRV_TP_ETHSWITCH_YEARLEN, fw_info->year,
		 RPMSG_KDRV_TP_ETHSWITCH_COMMITSHALEN, fw_info->commit_hash);

	if (fw_info->major != RPMSG_KDRV_TP_ETHSWITCH_VERSION_MAJOR &&
	    fw_info->minor != RPMSG_KDRV_TP_ETHSWITCH_VERSION_MINOR) {
		dev_err(&kddev->dev, "Unsupported EthSwitch FW version\n");
		return -EOPNOTSUPP;
	}

	priv->uart_connected = kddev_data->uart_connected;
	priv->uart_id = kddev_data->uart_connected ? kddev_data->uart_id : -1;
	priv->permissions = kddev_data->permission_flags;

	return 0;
}

static int rpmsg_kdrv_switch_probe(struct rpmsg_kdrv_device *dev)
{
	struct rpmsg_kdrv_switch_private *priv;
	int ret;

	dev_dbg(&dev->dev, "%s\n", __func__);

	priv = devm_kzalloc(&dev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->rdev.type = RPMSG_REMOTEDEV_ETH_SWITCH_DEVICE;
	priv->rdev.device.eth_switch.ops = &switch_ops;

	priv->kddev = dev;

	ret = rpmsg_kdrv_switch_dev_data_parse(dev, dev->device_data,
					       dev->device_data_len, priv);
	if (ret)
		return ret;

	dev->driver_private = priv;
	dev->remotedev = &priv->rdev;

	return 0;
}

static void rpmsg_kdrv_switch_remove(struct rpmsg_kdrv_device *dev)
{
	dev->driver_private = NULL;
	dev->remotedev = NULL;

	dev_dbg(&dev->dev, "%s\n", __func__);
}

static struct rpmsg_kdrv_driver rpmsg_kdrv_switch = {
	.drv = {
		.name = "rpmsg-kdrv-eth-switch",
	},
	.device_type = RPMSG_KDRV_TP_DEVICE_TYPE_ETHSWITCH,
	.probe = rpmsg_kdrv_switch_probe,
	.remove = rpmsg_kdrv_switch_remove,
	.callback = rpmsg_kdrv_switch_callback,
};

static int __init rpmsg_kdrv_display_driver_init(void)
{
	return rpmsg_kdrv_register_driver(&rpmsg_kdrv_switch);
}
module_init(rpmsg_kdrv_display_driver_init);

static void rpmsg_kdrv_display_driver_fini(void)
{
}
module_exit(rpmsg_kdrv_display_driver_fini);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Grygorii Strashko <grygorii.strashko@ti.com>");
MODULE_DESCRIPTION("TI J721E RPMSG KDRV Ethernet switch driver");
