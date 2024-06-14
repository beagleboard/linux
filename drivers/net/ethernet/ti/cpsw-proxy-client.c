// SPDX-License-Identifier: GPL-2.0-only or MIT
/* Texas Instruments CPSW Proxy Client Driver
 *
 * Copyright (C) 2024 Texas Instruments Incorporated - https://www.ti.com/
 *
 */

#include <linux/etherdevice.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/rpmsg.h>

#include "ethfw_abi.h"

#define ETHFW_RESPONSE_TIMEOUT_MS	500

enum virtual_port_type {
	VIRT_SWITCH_PORT,
	VIRT_MAC_ONLY_PORT,
};

struct cpsw_proxy_req_params {
	struct message	req_msg;	/* Request message to be filled */
	u32		token;
	u32		client_id;
	u32		request_id;
	u32		request_type;
	u32		rx_tx_idx; /* RX or TX Channel index */
	u32		rx_flow_base; /* RX DMA Flow base */
	u32		rx_flow_offset; /* RX DMA Flow offset */
	u32		tx_thread_id; /* PSI-L Thread ID of TX Channel */
	u32		port_id; /* Virtual Port ID */
	u16		vlan_id;
	u8		mac_addr[ETH_ALEN];
	u8		ipv4_addr[ETHFW_IPV4ADDRLEN];
};

struct virtual_port {
	struct cpsw_proxy_priv		*proxy_priv;
	enum virtual_port_type		port_type;
	u32				port_id;
};

struct cpsw_proxy_priv {
	struct rpmsg_device		*rpdev;
	struct device			*dev;
	struct virtual_port		*virt_ports;
	struct cpsw_proxy_req_params	req_params;
	struct mutex			req_params_mutex; /* Request params mutex */
	struct message			resp_msg;
	struct completion		wait_for_response;
	int				resp_msg_len;
	u32				vswitch_ports; /* Bitmask of Virtual Switch Port IDs */
	u32				vmac_ports /* Bitmask of Virtual MAC Only Port IDs */;
	u32				num_switch_ports;
	u32				num_mac_ports;
	u32				num_virt_ports;
};

static int cpsw_proxy_client_cb(struct rpmsg_device *rpdev, void *data,
				int len, void *priv, u32 src)
{
	struct cpsw_proxy_priv *proxy_priv = dev_get_drvdata(&rpdev->dev);
	struct response_message_header *resp_msg_hdr;
	struct message *msg = (struct message *)data;
	struct cpsw_proxy_req_params *req_params;
	struct device *dev = &rpdev->dev;
	u32 msg_type, resp_id;

	dev_dbg(dev, "callback invoked\n");
	msg_type = msg->msg_hdr.msg_type;
	switch (msg_type) {
	case ETHFW_MSG_RESPONSE:
		resp_msg_hdr = (struct response_message_header *)msg;
		resp_id = resp_msg_hdr->response_id;
		req_params = &proxy_priv->req_params;

		if (unlikely(resp_id == req_params->request_id - 1)) {
			dev_info(dev, "ignoring late response for request: %u\n",
				 resp_id);
			return 0;
		} else if (unlikely(resp_id != req_params->request_id)) {
			dev_err(dev, "expected response id: %u but received %u\n",
				req_params->request_id, resp_id);
			return -EINVAL;
		}

		/* Share response */
		memcpy(&proxy_priv->resp_msg, msg, len);
		proxy_priv->resp_msg_len = len;
		complete(&proxy_priv->wait_for_response);
		return 0;

	default:
		dev_err(dev, "unsupported message received from EthFw\n");
		return -EOPNOTSUPP;
	}
}

static int create_request_message(struct cpsw_proxy_req_params *req_params)
{
	struct mac_register_deregister_request *mac_reg_dereg_req;
	struct ipv4_deregister_request *ipv4_dereg_req;
	struct common_request_message *common_req_msg;
	struct tx_thread_release_request *tx_free_req;
	struct tx_thread_alloc_request *tx_alloc_req;
	struct add_multicast_request *mcast_add_req;
	struct del_multicast_request *mcast_del_req;
	struct rx_flow_release_request *rx_free_req;
	struct ipv4_register_request *ipv4_reg_req;
	struct request_message_header *req_msg_hdr;
	struct rx_flow_alloc_request *rx_alloc_req;
	struct message *msg = &req_params->req_msg;
	struct mac_release_request *mac_free_req;
	struct attach_request *attach_req;
	u32 req_type;

	/* Set message header fields */
	msg->msg_hdr.token = req_params->token;
	msg->msg_hdr.client_id = req_params->client_id;
	msg->msg_hdr.msg_type = ETHFW_MSG_REQUEST;

	req_type = req_params->request_type;

	switch (req_type) {
	case ETHFW_ALLOC_RX:
		rx_alloc_req = (struct rx_flow_alloc_request *)msg;
		req_msg_hdr = &rx_alloc_req->request_msg_hdr;
		rx_alloc_req->rx_flow_idx = req_params->rx_tx_idx;
		break;

	case ETHFW_ALLOC_TX:
		tx_alloc_req = (struct tx_thread_alloc_request *)msg;
		req_msg_hdr = &tx_alloc_req->request_msg_hdr;
		tx_alloc_req->tx_chan_idx = req_params->rx_tx_idx;
		break;

	case ETHFW_VIRT_PORT_ATTACH:
		attach_req = (struct attach_request *)msg;
		req_msg_hdr = &attach_req->request_msg_hdr;
		attach_req->virt_port = req_params->port_id;
		break;

	case ETHFW_FREE_MAC:
		mac_free_req = (struct mac_release_request *)msg;
		req_msg_hdr = &mac_free_req->request_msg_hdr;
		ether_addr_copy(mac_free_req->mac_addr, req_params->mac_addr);
		break;

	case ETHFW_FREE_RX:
		rx_free_req = (struct rx_flow_release_request *)msg;
		req_msg_hdr = &rx_free_req->request_msg_hdr;
		rx_free_req->rx_flow_idx_base = req_params->rx_flow_base;
		rx_free_req->rx_flow_idx_offset = req_params->rx_flow_offset;
		break;

	case ETHFW_FREE_TX:
		tx_free_req = (struct tx_thread_release_request *)msg;
		req_msg_hdr = &tx_free_req->request_msg_hdr;
		tx_free_req->tx_psil_dest_id = req_params->tx_thread_id;
		break;

	case ETHFW_IPv4_DEREGISTER:
		ipv4_dereg_req = (struct ipv4_deregister_request *)msg;
		req_msg_hdr = &ipv4_dereg_req->request_msg_hdr;
		memcpy(&ipv4_dereg_req->ipv4_addr, req_params->ipv4_addr,
		       ETHFW_IPV4ADDRLEN);
		break;

	case ETHFW_IPv4_REGISTER:
		ipv4_reg_req = (struct ipv4_register_request *)msg;
		req_msg_hdr = &ipv4_reg_req->request_msg_hdr;
		memcpy(&ipv4_reg_req->ipv4_addr, req_params->ipv4_addr,
		       ETHFW_IPV4ADDRLEN);
		ether_addr_copy(ipv4_reg_req->mac_addr,
				req_params->mac_addr);
		break;

	case ETHFW_MAC_DEREGISTER:
	case ETHFW_MAC_REGISTER:
		mac_reg_dereg_req = (struct mac_register_deregister_request *)msg;
		req_msg_hdr = &mac_reg_dereg_req->request_msg_hdr;
		ether_addr_copy(mac_reg_dereg_req->mac_addr,
				req_params->mac_addr);
		mac_reg_dereg_req->rx_flow_idx_base = req_params->rx_flow_base;
		mac_reg_dereg_req->rx_flow_idx_offset = req_params->rx_flow_offset;
		break;

	case ETHFW_MCAST_FILTER_ADD:
		mcast_add_req = (struct add_multicast_request *)msg;
		req_msg_hdr = &mcast_add_req->request_msg_hdr;
		ether_addr_copy(mcast_add_req->mac_addr, req_params->mac_addr);
		mcast_add_req->vlan_id = req_params->vlan_id;
		mcast_add_req->rx_flow_idx_base = req_params->rx_flow_base;
		mcast_add_req->rx_flow_idx_offset = req_params->rx_flow_offset;
		break;

	case ETHFW_MCAST_FILTER_DEL:
		mcast_del_req = (struct del_multicast_request *)msg;
		req_msg_hdr = &mcast_del_req->request_msg_hdr;
		ether_addr_copy(mcast_del_req->mac_addr, req_params->mac_addr);
		mcast_del_req->vlan_id = req_params->vlan_id;
		break;

	case ETHFW_ALLOC_MAC:
	case ETHFW_TEARDOWN_COMPLETE:
	case ETHFW_VIRT_PORT_DETACH:
	case ETHFW_VIRT_PORT_INFO:
		common_req_msg = (struct common_request_message *)msg;
		req_msg_hdr = &common_req_msg->request_msg_hdr;
		break;

	default:
		return -EOPNOTSUPP;
	}

	/* Set request message header fields */
	req_msg_hdr->request_id = req_params->request_id;
	req_msg_hdr->request_type = req_params->request_type;

	return 0;
}

/* Send a request to EthFw and receive the response for request.
 * Since the response is received by the callback function, it is
 * copied to "resp_msg" member of "struct cpsw_proxy_priv" to
 * allow sharing it with the following function.
 *
 * The request parameters within proxy_priv are expected to be set
 * correctly by the caller. The caller is also expected to acquire
 * lock before invoking this function, since requests and responses
 * to/from EthFw are serialized.
 */
static int send_request_get_response(struct cpsw_proxy_priv *proxy_priv,
				     struct message *response)
{
	struct cpsw_proxy_req_params *req_params = &proxy_priv->req_params;
	struct message *send_msg = &req_params->req_msg;
	struct rpmsg_device *rpdev = proxy_priv->rpdev;
	struct response_message_header *resp_msg_hdr;
	struct device *dev = proxy_priv->dev;
	unsigned long timeout;
	u32 resp_status;
	int ret;

	ret = create_request_message(req_params);
	if (ret) {
		dev_err(dev, "failed to create request %d\n", ret);
		goto err;
	}

	/* Send request and wait for callback function to acknowledge
	 * receiving the response.
	 */
	reinit_completion(&proxy_priv->wait_for_response);
	ret = rpmsg_send(rpdev->ept, (void *)(send_msg),
			 sizeof(struct message));
	if (ret) {
		dev_err(dev, "failed to send rpmsg\n");
		goto err;
	}
	timeout = msecs_to_jiffies(ETHFW_RESPONSE_TIMEOUT_MS);
	ret = wait_for_completion_timeout(&proxy_priv->wait_for_response,
					  timeout);
	if (!ret) {
		dev_err(dev, "response timedout\n");
		ret = -ETIMEDOUT;
		goto err;
	}

	/* Store response shared by callback function */
	memcpy(response, &proxy_priv->resp_msg, proxy_priv->resp_msg_len);
	resp_msg_hdr = (struct response_message_header *)response;
	resp_status = resp_msg_hdr->response_status;
	ret = resp_status;

	/* For all return values other than ETHFW_RES_EFAIL, the caller
	 * is expected to check the return value to deal with the failure
	 * accordingly.
	 */
	if (unlikely(resp_status == ETHFW_RES_EFAIL)) {
		dev_err(dev, "bad response status: %d\n", resp_status);
		ret = -EIO;
	}

err:
	req_params->request_id++;
	return ret;
}

static int get_virtual_port_info(struct cpsw_proxy_priv *proxy_priv)
{
	struct virt_port_info_response *vpi_resp;
	struct cpsw_proxy_req_params *req_p;
	struct virtual_port *vport;
	struct message resp_msg;
	unsigned int vp_id, i;
	int ret;

	mutex_lock(&proxy_priv->req_params_mutex);
	req_p = &proxy_priv->req_params;
	req_p->request_type = ETHFW_VIRT_PORT_INFO;
	ret = send_request_get_response(proxy_priv, &resp_msg);
	mutex_unlock(&proxy_priv->req_params_mutex);

	if (ret) {
		dev_err(proxy_priv->dev, "failed to get virtual port info\n");
		return ret;
	}

	vpi_resp = (struct virt_port_info_response *)&resp_msg;
	proxy_priv->vswitch_ports = vpi_resp->switch_port_mask;
	proxy_priv->vmac_ports = vpi_resp->mac_port_mask;
	/* Number of 1s set in vswitch_ports is the count of switch ports */
	proxy_priv->num_switch_ports = hweight32(proxy_priv->vswitch_ports);
	proxy_priv->num_virt_ports = proxy_priv->num_switch_ports;
	/* Number of 1s set in vmac_ports is the count of mac ports */
	proxy_priv->num_mac_ports = hweight32(proxy_priv->vmac_ports);
	proxy_priv->num_virt_ports += proxy_priv->num_mac_ports;

	proxy_priv->virt_ports = devm_kcalloc(proxy_priv->dev,
					      proxy_priv->num_virt_ports,
					      sizeof(*proxy_priv->virt_ports),
					      GFP_KERNEL);

	vp_id = 0;
	for (i = 0; i < proxy_priv->num_switch_ports; i++) {
		vport = &proxy_priv->virt_ports[vp_id];
		vport->proxy_priv = proxy_priv;
		vport->port_type = VIRT_SWITCH_PORT;
		/* Port ID is derived from the bit set in the bitmask */
		vport->port_id = fns(proxy_priv->vswitch_ports, i);
		vp_id++;
	}

	for (i = 0; i < proxy_priv->num_mac_ports; i++) {
		vport = &proxy_priv->virt_ports[vp_id];
		vport->proxy_priv = proxy_priv;
		vport->port_type = VIRT_MAC_ONLY_PORT;
		/* Port ID is derived from the bit set in the bitmask */
		vport->port_id = fns(proxy_priv->vmac_ports, i);
		vp_id++;
	}

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
	dev_set_drvdata(proxy_priv->dev, proxy_priv);
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
