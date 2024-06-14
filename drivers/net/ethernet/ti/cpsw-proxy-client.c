// SPDX-License-Identifier: GPL-2.0-only or MIT
/* Texas Instruments CPSW Proxy Client Driver
 *
 * Copyright (C) 2024 Texas Instruments Incorporated - https://www.ti.com/
 *
 */

#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/if_vlan.h>
#include <linux/kernel.h>
#include <linux/kmemleak.h>
#include <linux/module.h>
#include <linux/rpmsg.h>
#include <linux/dma/k3-udma-glue.h>

#include "ethfw_abi.h"
#include "k3-cppi-desc-pool.h"

#define ETHFW_RESPONSE_TIMEOUT_MS	500

#define PS_DATA_SIZE	16
#define SW_DATA_SIZE	16

#define MAX_TX_DESC	500
#define MAX_RX_DESC	500
#define MAX_RX_FLOWS	1

#define MIN_PACKET_SIZE	ETH_ZLEN
#define MAX_PACKET_SIZE	(VLAN_ETH_FRAME_LEN + ETH_FCS_LEN)

#define CHAN_NAME_LEN	128

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

struct rx_dma_chan {
	struct virtual_port		*vport;
	struct device			*dev;
	struct k3_cppi_desc_pool	*desc_pool;
	struct k3_udma_glue_rx_channel	*rx_chan;
	struct napi_struct		napi_rx;
	u32				rel_chan_idx;
	u32				flow_base;
	u32				flow_offset;
	u32				thread_id;
	u32				num_descs;
	unsigned int			irq;
	char				rx_chan_name[CHAN_NAME_LEN];
	bool				in_use;
};

struct tx_dma_chan {
	struct virtual_port		*vport;
	struct device			*dev;
	struct k3_cppi_desc_pool	*desc_pool;
	struct k3_udma_glue_tx_channel	*tx_chan;
	struct napi_struct		napi_tx;
	u32				rel_chan_idx;
	u32				thread_id;
	u32				num_descs;
	unsigned int			irq;
	char				tx_chan_name[CHAN_NAME_LEN];
	bool				in_use;
};

struct vport_netdev_stats {
	u64			tx_packets;
	u64			tx_bytes;
	u64			rx_packets;
	u64			rx_bytes;
	struct u64_stats_sync	syncp;
};

struct vport_netdev_priv {
	struct vport_netdev_stats __percpu	*stats;
	struct virtual_port			*vport;
};

struct virtual_port {
	struct cpsw_proxy_priv		*proxy_priv;
	struct net_device		*ndev;
	struct rx_dma_chan		*rx_chans;
	struct tx_dma_chan		*tx_chans;
	struct completion		tdown_complete;
	enum virtual_port_type		port_type;
	atomic_t			tdown_cnt;
	u32				port_id;
	u32				port_token;
	u32				port_features;
	u32				num_rx_chan;
	u32				num_tx_chan;
	u8				mac_addr[ETH_ALEN];
	bool				mac_in_use;
};

struct cpsw_proxy_priv {
	struct rpmsg_device		*rpdev;
	struct device			*dev;
	struct device_node		*dma_node;
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
	u32				num_active_tx_chans;
	u32				num_active_rx_chans;
};

#define vport_netdev_to_priv(ndev) \
	((struct vport_netdev_priv *)netdev_priv(ndev))
#define vport_ndev_to_vport(ndev) \
	(vport_netdev_to_priv(ndev)->vport)

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
	case ETHFW_VIRT_PORT_LINK_STATUS:
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

static int attach_virtual_ports(struct cpsw_proxy_priv *proxy_priv)
{
	struct cpsw_proxy_req_params *req_p;
	struct attach_response *att_resp;
	struct rx_dma_chan *rx_chn;
	struct tx_dma_chan *tx_chn;
	struct virtual_port *vport;
	struct message resp_msg;
	unsigned int i, j;
	u32 port_id;
	int ret;

	for (i = 0; i < proxy_priv->num_virt_ports; i++) {
		vport = &proxy_priv->virt_ports[i];
		port_id = vport->port_id;

		mutex_lock(&proxy_priv->req_params_mutex);
		req_p = &proxy_priv->req_params;
		req_p->port_id = port_id;
		req_p->request_type = ETHFW_VIRT_PORT_ATTACH;
		ret = send_request_get_response(proxy_priv, &resp_msg);
		mutex_unlock(&proxy_priv->req_params_mutex);

		if (ret) {
			dev_err(proxy_priv->dev, "attaching virtual port failed\n");
			goto err;
		}

		att_resp = (struct attach_response *)&resp_msg;
		vport->port_token = att_resp->response_msg_hdr.msg_hdr.token;
		vport->port_features = att_resp->features;
		vport->num_tx_chan = att_resp->num_tx_chan;
		vport->num_rx_chan = att_resp->num_rx_flow;

		vport->rx_chans = devm_kcalloc(proxy_priv->dev,
					       vport->num_rx_chan,
					       sizeof(*vport->rx_chans),
					       GFP_KERNEL);
		for (j = 0; j < vport->num_rx_chan; j++) {
			rx_chn = &vport->rx_chans[j];
			rx_chn->vport = vport;
			rx_chn->rel_chan_idx = j;
		}

		vport->tx_chans = devm_kcalloc(proxy_priv->dev,
					       vport->num_tx_chan,
					       sizeof(*vport->tx_chans),
					       GFP_KERNEL);
		for (j = 0; j < vport->num_tx_chan; j++) {
			tx_chn = &vport->tx_chans[j];
			tx_chn->vport = vport;
			tx_chn->rel_chan_idx = j;
		}
	}

	return 0;

err:
	/* Detach virtual ports which were successfully attached */
	while (i--) {
		vport = &proxy_priv->virt_ports[i];
		port_id = vport->port_id;
		mutex_lock(&proxy_priv->req_params_mutex);
		req_p = &proxy_priv->req_params;
		req_p->request_type = ETHFW_VIRT_PORT_DETACH;
		req_p->token = vport->port_token;
		ret = send_request_get_response(proxy_priv, &resp_msg);
		mutex_unlock(&proxy_priv->req_params_mutex);
		if (ret)
			dev_err(proxy_priv->dev, "detaching virtual port %u failed\n", port_id);
	}
	return -EIO;
}

static void free_port_resources(struct cpsw_proxy_priv *proxy_priv)
{
	struct cpsw_proxy_req_params *req_p;
	struct rx_dma_chan *rx_chn;
	struct tx_dma_chan *tx_chn;
	struct virtual_port *vport;
	struct message resp_msg;
	u32 port_id, i, j;
	int ret;

	for (i = 0; i < proxy_priv->num_virt_ports; i++) {
		vport = &proxy_priv->virt_ports[i];
		port_id = vport->port_id;

		/* Free allocated MAC */
		if (vport->mac_in_use) {
			mutex_lock(&proxy_priv->req_params_mutex);
			req_p = &proxy_priv->req_params;
			req_p->request_type = ETHFW_FREE_MAC;
			req_p->token = vport->port_token;
			ether_addr_copy(req_p->mac_addr, vport->mac_addr);
			ret = send_request_get_response(proxy_priv, &resp_msg);
			mutex_unlock(&proxy_priv->req_params_mutex);
			if (ret) {
				dev_err(proxy_priv->dev,
					"failed to free MAC Address for port %u err: %d\n",
					port_id, ret);
				return;
			}
		}

		/* Free TX DMA Channels */
		for (j = 0; j < vport->num_tx_chan; j++) {
			tx_chn = &vport->tx_chans[j];
			if (!tx_chn->in_use)
				continue;
			mutex_lock(&proxy_priv->req_params_mutex);
			req_p = &proxy_priv->req_params;
			req_p->request_type = ETHFW_FREE_TX;
			req_p->token = vport->port_token;
			req_p->tx_thread_id = tx_chn->thread_id;
			ret = send_request_get_response(proxy_priv, &resp_msg);
			mutex_unlock(&proxy_priv->req_params_mutex);
			if (ret) {
				dev_err(proxy_priv->dev,
					"failed to free TX Channel for port %u err: %d\n",
					port_id, ret);
				return;
			}
		}

		/* Free RX DMA Channels */
		for (j = 0; j < vport->num_rx_chan; j++) {
			rx_chn = &vport->rx_chans[j];
			if (!rx_chn->in_use)
				continue;
			mutex_lock(&proxy_priv->req_params_mutex);
			req_p = &proxy_priv->req_params;
			req_p->request_type = ETHFW_FREE_RX;
			req_p->token = vport->port_token;
			req_p->rx_flow_base = rx_chn->flow_base;
			req_p->rx_flow_offset = rx_chn->flow_offset;
			ret = send_request_get_response(proxy_priv, &resp_msg);
			mutex_unlock(&proxy_priv->req_params_mutex);
			if (ret) {
				dev_err(proxy_priv->dev,
					"failed to free RX Channel for port %u err: %d\n",
					port_id, ret);
				return;
			}
		}
	}
}

static int allocate_port_resources(struct cpsw_proxy_priv *proxy_priv)
{
	struct tx_thread_alloc_response *tta_resp;
	struct rx_flow_alloc_response *rfa_resp;
	struct cpsw_proxy_req_params *req_p;
	struct mac_alloc_response *ma_resp;
	struct rx_dma_chan *rx_chn;
	struct tx_dma_chan *tx_chn;
	struct virtual_port *vport;
	struct message resp_msg;
	u32 port_id, i, j;
	int ret;

	for (i = 0; i < proxy_priv->num_virt_ports; i++) {
		vport = &proxy_priv->virt_ports[i];
		port_id = vport->port_id;

		/* Request RX DMA Flow allocation */
		for (j = 0; j < vport->num_rx_chan; j++) {
			mutex_lock(&proxy_priv->req_params_mutex);
			req_p = &proxy_priv->req_params;
			req_p->request_type = ETHFW_ALLOC_RX;
			req_p->token = vport->port_token;
			req_p->rx_tx_idx = j;
			ret = send_request_get_response(proxy_priv, &resp_msg);
			mutex_unlock(&proxy_priv->req_params_mutex);
			if (ret) {
				dev_err(proxy_priv->dev, "RX Alloc for port %u failed\n", port_id);
				goto err;
			}

			rfa_resp = (struct rx_flow_alloc_response *)&resp_msg;
			rx_chn = &vport->rx_chans[j];
			rx_chn->flow_base = rfa_resp->rx_flow_idx_base;
			rx_chn->flow_offset = rfa_resp->rx_flow_idx_offset;
			rx_chn->thread_id = rfa_resp->rx_psil_src_id;
			rx_chn->in_use = 1;
		}

		/* Request TX DMA Channel allocation */
		for (j = 0; j < vport->num_tx_chan; j++) {
			mutex_lock(&proxy_priv->req_params_mutex);
			req_p = &proxy_priv->req_params;
			req_p->request_type = ETHFW_ALLOC_TX;
			req_p->token = vport->port_token;
			req_p->rx_tx_idx = j;
			ret = send_request_get_response(proxy_priv, &resp_msg);
			mutex_unlock(&proxy_priv->req_params_mutex);
			if (ret) {
				dev_err(proxy_priv->dev, "TX Alloc for port %u failed\n", port_id);
				goto err;
			}

			tta_resp = (struct tx_thread_alloc_response *)&resp_msg;
			tx_chn = &vport->tx_chans[j];
			tx_chn->thread_id = tta_resp->tx_psil_dest_id;
			tx_chn->in_use = 1;
		}

		/* Request MAC allocation */
		mutex_lock(&proxy_priv->req_params_mutex);
		req_p = &proxy_priv->req_params;
		req_p->request_type = ETHFW_ALLOC_MAC;
		req_p->token = vport->port_token;
		ret = send_request_get_response(proxy_priv, &resp_msg);
		mutex_unlock(&proxy_priv->req_params_mutex);
		if (ret) {
			dev_err(proxy_priv->dev, "MAC Alloc for port %u failed\n", port_id);
			goto err;
		}

		ma_resp = (struct mac_alloc_response *)&resp_msg;
		ether_addr_copy(vport->mac_addr, ma_resp->mac_addr);
		vport->mac_in_use = 1;
	}

	return 0;

err:
	free_port_resources(proxy_priv);
	return -EIO;
}

static void free_tx_chns(void *data)
{
	struct cpsw_proxy_priv *proxy_priv = data;
	struct tx_dma_chan *tx_chn;
	struct virtual_port *vport;
	u32 i, j;

	for (i = 0; i < proxy_priv->num_virt_ports; i++) {
		vport = &proxy_priv->virt_ports[i];
		for (j = 0; j < vport->num_tx_chan; j++) {
			tx_chn = &vport->tx_chans[j];

			if (!IS_ERR_OR_NULL(tx_chn->desc_pool))
				k3_cppi_desc_pool_destroy(tx_chn->desc_pool);

			if (!IS_ERR_OR_NULL(tx_chn->tx_chan))
				k3_udma_glue_release_tx_chn(tx_chn->tx_chan);

			memset(tx_chn, 0, sizeof(*tx_chn));
		}
	}
}

static int init_tx_chans(struct cpsw_proxy_priv *proxy_priv)
{
	u32 max_desc_num = ALIGN(MAX_TX_DESC, MAX_SKB_FRAGS);
	struct k3_udma_glue_tx_channel_cfg tx_cfg = { 0 };
	struct device *dev = proxy_priv->dev;
	u32 hdesc_size, tx_chn_num, i, j;
	char tx_chn_name[CHAN_NAME_LEN];
	struct k3_ring_cfg ring_cfg = {
		.elm_size = K3_RINGACC_RING_ELSIZE_8,
		.mode = K3_RINGACC_RING_MODE_RING,
		.flags = 0
	};
	struct tx_dma_chan *tx_chn;
	struct virtual_port *vport;
	int ret = 0, ret1;

	for (i = 0; i < proxy_priv->num_virt_ports; i++) {
		vport = &proxy_priv->virt_ports[i];
		init_completion(&vport->tdown_complete);

		for (j = 0; j < vport->num_tx_chan; j++) {
			tx_chn = &vport->tx_chans[j];

			tx_chn_num = proxy_priv->num_active_tx_chans++;
			snprintf(tx_chn_name, sizeof(tx_chn_name), "tx%u-virt-port-%u",
				 tx_chn_num, vport->port_id);
			strscpy(tx_chn->tx_chan_name, tx_chn_name, sizeof(tx_chn->tx_chan_name));

			hdesc_size = cppi5_hdesc_calc_size(true, PS_DATA_SIZE, SW_DATA_SIZE);

			tx_cfg.swdata_size = SW_DATA_SIZE;
			tx_cfg.tx_cfg = ring_cfg;
			tx_cfg.txcq_cfg = ring_cfg;
			tx_cfg.tx_cfg.size = max_desc_num;
			tx_cfg.txcq_cfg.size = max_desc_num;

			tx_chn->dev = dev;
			tx_chn->num_descs = max_desc_num;
			tx_chn->desc_pool = k3_cppi_desc_pool_create_name(dev,
									  tx_chn->num_descs,
									  hdesc_size,
									  tx_chn_name);
			if (IS_ERR(tx_chn->desc_pool)) {
				ret = PTR_ERR(tx_chn->desc_pool);
				dev_err(dev, "failed to create tx pool %d\n", ret);
				goto err;
			}

			tx_chn->tx_chan =
				k3_udma_glue_request_tx_chn_for_thread_id(dev, &tx_cfg,
									  proxy_priv->dma_node,
									  tx_chn->thread_id);
			if (IS_ERR(tx_chn->tx_chan)) {
				ret = PTR_ERR(tx_chn->tx_chan);
				dev_err(dev, "Failed to request tx dma channel %d\n", ret);
				goto err;
			}

			tx_chn->irq = k3_udma_glue_tx_get_irq(tx_chn->tx_chan);
			if (tx_chn->irq <= 0) {
				dev_err(dev, "Failed to get tx dma irq %d\n", tx_chn->irq);
				ret = -ENXIO;
			}
		}
	}

err:
	ret1 = devm_add_action(dev, free_tx_chns, proxy_priv);
	if (ret1) {
		dev_err(dev, "failed to add free_tx_chns action %d", ret1);
		return ret1;
	}

	return ret;
}

static void free_rx_chns(void *data)
{
	struct cpsw_proxy_priv *proxy_priv = data;
	struct rx_dma_chan *rx_chn;
	struct virtual_port *vport;
	u32 i, j;

	for (i = 0; i < proxy_priv->num_virt_ports; i++) {
		vport = &proxy_priv->virt_ports[i];

		for (j = 0; j < vport->num_rx_chan; j++) {
			rx_chn = &vport->rx_chans[j];

			if (!IS_ERR_OR_NULL(rx_chn->desc_pool))
				k3_cppi_desc_pool_destroy(rx_chn->desc_pool);

			if (!IS_ERR_OR_NULL(rx_chn->rx_chan))
				k3_udma_glue_release_rx_chn(rx_chn->rx_chan);
		}
	}
}

static int init_rx_chans(struct cpsw_proxy_priv *proxy_priv)
{
	struct k3_udma_glue_rx_channel_cfg rx_cfg = {0};
	struct device *dev = proxy_priv->dev;
	u32 hdesc_size, rx_chn_num, i, j;
	u32  max_desc_num = MAX_RX_DESC;
	char rx_chn_name[CHAN_NAME_LEN];
	struct rx_dma_chan *rx_chn;
	struct virtual_port *vport;
	struct k3_ring_cfg rxring_cfg = {
		.elm_size = K3_RINGACC_RING_ELSIZE_8,
		.mode = K3_RINGACC_RING_MODE_MESSAGE,
		.flags = 0,
	};
	struct k3_ring_cfg fdqring_cfg = {
		.elm_size = K3_RINGACC_RING_ELSIZE_8,
		.mode = K3_RINGACC_RING_MODE_MESSAGE,
		.flags = 0,
	};
	struct k3_udma_glue_rx_flow_cfg rx_flow_cfg = {
		.rx_cfg = rxring_cfg,
		.rxfdq_cfg = fdqring_cfg,
		.ring_rxq_id = K3_RINGACC_RING_ID_ANY,
		.ring_rxfdq0_id = K3_RINGACC_RING_ID_ANY,
		.src_tag_lo_sel = K3_UDMA_GLUE_SRC_TAG_LO_USE_REMOTE_SRC_TAG,
	};
	int ret = 0, ret1;

	hdesc_size = cppi5_hdesc_calc_size(true, PS_DATA_SIZE, SW_DATA_SIZE);

	rx_cfg.swdata_size = SW_DATA_SIZE;
	rx_cfg.flow_id_num = MAX_RX_FLOWS;
	rx_cfg.remote = true;

	for (i = 0; i < proxy_priv->num_virt_ports; i++) {
		vport = &proxy_priv->virt_ports[i];

		for (j = 0; j < vport->num_rx_chan; j++) {
			rx_chn = &vport->rx_chans[j];

			rx_chn_num = proxy_priv->num_active_rx_chans++;
			snprintf(rx_chn_name, sizeof(rx_chn_name), "rx%u-virt-port-%u", rx_chn_num,
				 vport->port_id);
			strscpy(rx_chn->rx_chan_name, rx_chn_name, sizeof(rx_chn->rx_chan_name));

			rx_cfg.flow_id_base = rx_chn->flow_base + rx_chn->flow_offset;

			/* init all flows */
			rx_chn->dev = dev;
			rx_chn->num_descs = max_desc_num;
			rx_chn->desc_pool = k3_cppi_desc_pool_create_name(dev,
									  rx_chn->num_descs,
									  hdesc_size,
									  rx_chn_name);
			if (IS_ERR(rx_chn->desc_pool)) {
				ret = PTR_ERR(rx_chn->desc_pool);
				dev_err(dev, "Failed to create rx pool %d\n", ret);
				goto err;
			}

			rx_chn->rx_chan =
			k3_udma_glue_request_remote_rx_chn_for_thread_id(dev, &rx_cfg,
									 proxy_priv->dma_node,
									 rx_chn->thread_id);
			if (IS_ERR(rx_chn->rx_chan)) {
				ret = PTR_ERR(rx_chn->rx_chan);
				dev_err(dev, "Failed to request rx dma channel %d\n", ret);
				goto err;
			}

			rx_flow_cfg.rx_cfg.size = max_desc_num;
			rx_flow_cfg.rxfdq_cfg.size = max_desc_num;
			ret = k3_udma_glue_rx_flow_init(rx_chn->rx_chan,
							0, &rx_flow_cfg);
			if (ret) {
				dev_err(dev, "Failed to init rx flow %d\n", ret);
				goto err;
			}

			rx_chn->irq = k3_udma_glue_rx_get_irq(rx_chn->rx_chan, 0);
			if (rx_chn->irq <= 0) {
				ret = -ENXIO;
				dev_err(dev, "Failed to get rx dma irq %d\n", rx_chn->irq);
			}
		}
	}

err:
	ret1 = devm_add_action(dev, free_rx_chns, proxy_priv);
	if (ret1) {
		dev_err(dev, "failed to add free_rx_chns action %d", ret1);
		return ret1;
	}

	return ret;
}

static void vport_xmit_free(struct tx_dma_chan *tx_chn, struct device *dev,
			    struct cppi5_host_desc_t *desc)
{
	struct cppi5_host_desc_t *first_desc, *next_desc;
	dma_addr_t buf_dma, next_desc_dma;
	u32 buf_dma_len;

	first_desc = desc;
	next_desc = first_desc;

	cppi5_hdesc_get_obuf(first_desc, &buf_dma, &buf_dma_len);

	dma_unmap_single(dev, buf_dma, buf_dma_len,
			 DMA_TO_DEVICE);

	next_desc_dma = cppi5_hdesc_get_next_hbdesc(first_desc);
	while (next_desc_dma) {
		next_desc = k3_cppi_desc_pool_dma2virt(tx_chn->desc_pool,
						       next_desc_dma);
		cppi5_hdesc_get_obuf(next_desc, &buf_dma, &buf_dma_len);

		dma_unmap_page(dev, buf_dma, buf_dma_len,
			       DMA_TO_DEVICE);

		next_desc_dma = cppi5_hdesc_get_next_hbdesc(next_desc);

		k3_cppi_desc_pool_free(tx_chn->desc_pool, next_desc);
	}

	k3_cppi_desc_pool_free(tx_chn->desc_pool, first_desc);
}

static int tx_compl_packets(struct virtual_port *vport, unsigned int tx_chan_idx,
			    unsigned int budget, bool *tdown)
{
	struct cpsw_proxy_priv *proxy_priv = vport->proxy_priv;
	struct device *dev = proxy_priv->dev;
	struct cppi5_host_desc_t *desc_tx;
	struct netdev_queue *netif_txq;
	unsigned int total_bytes = 0;
	struct tx_dma_chan *tx_chn;
	struct net_device *ndev;
	struct sk_buff *skb;
	dma_addr_t desc_dma;
	int res, num_tx = 0;
	void **swdata;

	tx_chn = &vport->tx_chans[tx_chan_idx];

	while (budget--) {
		struct vport_netdev_priv *ndev_priv;
		struct vport_netdev_stats *stats;

		res = k3_udma_glue_pop_tx_chn(tx_chn->tx_chan, &desc_dma);
		if (res == -ENODATA)
			break;

		if (desc_dma & 0x1) {
			if (atomic_dec_and_test(&vport->tdown_cnt))
				complete(&vport->tdown_complete);
			*tdown = true;
			break;
		}

		desc_tx = k3_cppi_desc_pool_dma2virt(tx_chn->desc_pool,
						     desc_dma);
		swdata = cppi5_hdesc_get_swdata(desc_tx);
		skb = *(swdata);
		vport_xmit_free(tx_chn, dev, desc_tx);

		ndev = skb->dev;

		ndev_priv = netdev_priv(ndev);
		stats = this_cpu_ptr(ndev_priv->stats);
		u64_stats_update_begin(&stats->syncp);
		stats->tx_packets++;
		stats->tx_bytes += skb->len;
		u64_stats_update_end(&stats->syncp);

		total_bytes += skb->len;
		napi_consume_skb(skb, budget);
		num_tx++;
	}

	if (!num_tx)
		return 0;

	netif_txq = netdev_get_tx_queue(ndev, tx_chan_idx);
	netdev_tx_completed_queue(netif_txq, num_tx, total_bytes);

	if (netif_tx_queue_stopped(netif_txq)) {
		__netif_tx_lock(netif_txq, smp_processor_id());
		if (netif_running(ndev) &&
		    (k3_cppi_desc_pool_avail(tx_chn->desc_pool) >=
		     MAX_SKB_FRAGS))
			netif_tx_wake_queue(netif_txq);

		__netif_tx_unlock(netif_txq);
	}

	return num_tx;
}

static int vport_tx_poll(struct napi_struct *napi_tx, int budget)
{
	struct tx_dma_chan *tx_chn = container_of(napi_tx, struct tx_dma_chan,
							 napi_tx);
	struct virtual_port *vport = tx_chn->vport;
	bool tdown = false;
	int num_tx;

	/* process every unprocessed channel */
	num_tx = tx_compl_packets(vport, tx_chn->rel_chan_idx, budget, &tdown);

	if (num_tx >= budget)
		return budget;

	if (napi_complete_done(napi_tx, num_tx))
		enable_irq(tx_chn->irq);

	return 0;
}

/* RX psdata[2] word format - checksum information */
#define RX_PSD_CSUM_ERR		BIT(16)
#define RX_PSD_IS_FRAGMENT	BIT(17)
#define RX_PSD_IPV6_VALID	BIT(19)
#define RX_PSD_IPV4_VALID	BIT(20)

static void vport_rx_csum(struct sk_buff *skb, u32 csum_info)
{
	/* HW can verify IPv4/IPv6 TCP/UDP packets checksum
	 * csum information provides in psdata[2] word:
	 * RX_PSD_CSUM_ERR bit - indicates csum error
	 * RX_PSD_IPV6_VALID and CPSW_RX_PSD_IPV4_VALID
	 * bits - indicates IPv4/IPv6 packet
	 * RX_PSD_IS_FRAGMENT bit - indicates fragmented packet
	 * RX_PSD_CSUM_ADD has value 0xFFFF for non fragmented packets
	 * or csum value for fragmented packets if !RX_PSD_CSUM_ERR
	 */
	skb_checksum_none_assert(skb);

	if (unlikely(!(skb->dev->features & NETIF_F_RXCSUM)))
		return;

	if ((csum_info & (RX_PSD_IPV6_VALID |
			  RX_PSD_IPV4_VALID)) &&
			  !(csum_info & RX_PSD_CSUM_ERR)) {
		/* csum for fragmented packets is unsupported */
		if (!(csum_info & RX_PSD_IS_FRAGMENT))
			skb->ip_summed = CHECKSUM_UNNECESSARY;
	}
}

static int vport_rx_push(struct virtual_port *vport, struct sk_buff *skb,
			 u32 rx_chan_idx)
{
	struct rx_dma_chan *rx_chn = &vport->rx_chans[rx_chan_idx];
	struct cpsw_proxy_priv *proxy_priv = vport->proxy_priv;
	struct device *dev = proxy_priv->dev;
	struct cppi5_host_desc_t *desc_rx;
	u32 pkt_len = skb_tailroom(skb);
	dma_addr_t desc_dma;
	dma_addr_t buf_dma;
	void *swdata;

	desc_rx = k3_cppi_desc_pool_alloc(rx_chn->desc_pool);
	if (!desc_rx) {
		dev_err(dev, "Failed to allocate RXFDQ descriptor\n");
		return -ENOMEM;
	}
	desc_dma = k3_cppi_desc_pool_virt2dma(rx_chn->desc_pool, desc_rx);

	buf_dma = dma_map_single(dev, skb->data, pkt_len, DMA_FROM_DEVICE);
	if (unlikely(dma_mapping_error(dev, buf_dma))) {
		k3_cppi_desc_pool_free(rx_chn->desc_pool, desc_rx);
		dev_err(dev, "Failed to map rx skb buffer\n");
		return -EINVAL;
	}

	cppi5_hdesc_init(desc_rx, CPPI5_INFO0_HDESC_EPIB_PRESENT,
			 PS_DATA_SIZE);
	cppi5_hdesc_attach_buf(desc_rx, 0, 0, buf_dma, skb_tailroom(skb));
	swdata = cppi5_hdesc_get_swdata(desc_rx);
	*((void **)swdata) = skb;

	return k3_udma_glue_push_rx_chn(rx_chn->rx_chan, 0, desc_rx, desc_dma);
}

static int vport_rx_packets(struct virtual_port *vport, u32 rx_chan_idx)
{
	struct rx_dma_chan *rx_chn = &vport->rx_chans[rx_chan_idx];
	struct cpsw_proxy_priv *proxy_priv = vport->proxy_priv;
	u32 buf_dma_len, pkt_len, port_id = 0, csum_info;
	struct device *dev = proxy_priv->dev;
	struct vport_netdev_priv *ndev_priv;
	struct cppi5_host_desc_t *desc_rx;
	struct vport_netdev_stats *stats;
	struct sk_buff *skb, *new_skb;
	dma_addr_t desc_dma, buf_dma;
	struct net_device *ndev;
	u32 flow_idx = 0;
	void **swdata;
	int ret = 0;
	u32 *psdata;

	ret = k3_udma_glue_pop_rx_chn(rx_chn->rx_chan, flow_idx, &desc_dma);
	if (ret) {
		if (ret != -ENODATA)
			dev_err(dev, "RX: pop chn fail %d\n", ret);
		return ret;
	}

	if (desc_dma & 0x1) {
		dev_dbg(dev, "%s RX tdown flow: %u\n", __func__, flow_idx);
		return 0;
	}

	desc_rx = k3_cppi_desc_pool_dma2virt(rx_chn->desc_pool, desc_dma);
	dev_dbg(dev, "%s flow_idx: %u desc %pad\n",
		__func__, flow_idx, &desc_dma);

	swdata = cppi5_hdesc_get_swdata(desc_rx);
	skb = *swdata;
	cppi5_hdesc_get_obuf(desc_rx, &buf_dma, &buf_dma_len);
	pkt_len = cppi5_hdesc_get_pktlen(desc_rx);
	cppi5_desc_get_tags_ids(&desc_rx->hdr, &port_id, NULL);
	/* read port for dbg */
	dev_dbg(dev, "%s rx port_id:%d\n", __func__, port_id);
	ndev = vport->ndev;
	skb->dev = ndev;

	psdata = cppi5_hdesc_get_psdata(desc_rx);
	csum_info = psdata[2];
	dev_dbg(dev, "%s rx csum_info:%#x\n", __func__, csum_info);

	dma_unmap_single(dev, buf_dma, buf_dma_len, DMA_FROM_DEVICE);

	k3_cppi_desc_pool_free(rx_chn->desc_pool, desc_rx);

	if (unlikely(!netif_running(skb->dev))) {
		dev_kfree_skb_any(skb);
		return -ENODEV;
	}

	new_skb = netdev_alloc_skb_ip_align(ndev, MAX_PACKET_SIZE);
	if (new_skb) {
		skb_put(skb, pkt_len);
		skb->protocol = eth_type_trans(skb, ndev);
		vport_rx_csum(skb, csum_info);
		napi_gro_receive(&rx_chn->napi_rx, skb);

		ndev_priv = netdev_priv(ndev);
		stats = this_cpu_ptr(ndev_priv->stats);

		u64_stats_update_begin(&stats->syncp);
		stats->rx_packets++;
		stats->rx_bytes += pkt_len;
		u64_stats_update_end(&stats->syncp);
		kmemleak_not_leak(new_skb);
	} else {
		ndev->stats.rx_dropped++;
		new_skb = skb;
	}

	if (netif_dormant(ndev)) {
		dev_kfree_skb_any(new_skb);
		ndev->stats.rx_dropped++;
		return -ENODEV;
	}

	ret = vport_rx_push(vport, new_skb, rx_chn->rel_chan_idx);
	if (WARN_ON(ret < 0)) {
		dev_kfree_skb_any(new_skb);
		ndev->stats.rx_errors++;
		ndev->stats.rx_dropped++;
	}

	return ret;
}

static int vport_rx_poll(struct napi_struct *napi_rx, int budget)
{
	struct rx_dma_chan *rx_chn = container_of(napi_rx, struct rx_dma_chan,
							 napi_rx);
	struct virtual_port *vport = rx_chn->vport;
	int num_rx = 0;
	int cur_budget;
	int ret;

	/* process every flow */
	cur_budget = budget;

	while (cur_budget--) {
		ret = vport_rx_packets(vport, rx_chn->rel_chan_idx);
		if (ret)
			break;
		num_rx++;
	}

	if (num_rx < budget && napi_complete_done(napi_rx, num_rx))
		enable_irq(rx_chn->irq);

	return num_rx;
}

static u32 vport_get_link(struct net_device *ndev)
{
	struct virtual_port *vport = vport_ndev_to_vport(ndev);
	struct cpsw_proxy_priv *proxy_priv = vport->proxy_priv;
	struct port_link_status_response *pls_resp;
	struct cpsw_proxy_req_params *req_p;
	struct message resp_msg;
	bool link_up;
	int ret;

	if (vport->port_type != VIRT_MAC_ONLY_PORT)
		return ethtool_op_get_link(ndev);

	mutex_lock(&proxy_priv->req_params_mutex);
	req_p = &proxy_priv->req_params;
	req_p->request_type = ETHFW_VIRT_PORT_LINK_STATUS;
	req_p->token = vport->port_token;
	ret = send_request_get_response(proxy_priv, &resp_msg);
	mutex_unlock(&proxy_priv->req_params_mutex);
	if (ret) {
		netdev_err(ndev, "failed to get link status\n");
		/* Assume that link is down if status is unknown */
		return 0;
	}
	pls_resp = (struct port_link_status_response *)&resp_msg;
	link_up = pls_resp->link_up;

	return link_up;
}

const struct ethtool_ops cpsw_proxy_client_ethtool_ops = {
	.get_link		= vport_get_link,
};

static int register_mac(struct virtual_port *vport)
{
	struct cpsw_proxy_priv *proxy_priv = vport->proxy_priv;
	struct rx_dma_chan *rx_chn = &vport->rx_chans[0];
	struct cpsw_proxy_req_params *req_p;
	struct message resp_msg;
	int ret;

	/* Register MAC Address only for RX DMA Channel 0 */
	mutex_lock(&proxy_priv->req_params_mutex);
	req_p = &proxy_priv->req_params;
	req_p->request_type = ETHFW_MAC_REGISTER;
	req_p->token = vport->port_token;
	req_p->rx_flow_base = rx_chn->flow_base;
	req_p->rx_flow_offset = rx_chn->flow_offset;
	ether_addr_copy(req_p->mac_addr, vport->mac_addr);
	ret = send_request_get_response(proxy_priv, &resp_msg);
	mutex_unlock(&proxy_priv->req_params_mutex);
	if (ret)
		dev_err(proxy_priv->dev, "failed to register MAC Address\n");

	return ret;
}

static int deregister_mac(struct virtual_port *vport)
{
	struct cpsw_proxy_priv *proxy_priv = vport->proxy_priv;
	struct rx_dma_chan *rx_chn = &vport->rx_chans[0];
	struct cpsw_proxy_req_params *req_p;
	struct message resp_msg;
	int ret;

	mutex_lock(&proxy_priv->req_params_mutex);
	req_p = &proxy_priv->req_params;
	req_p->request_type = ETHFW_MAC_DEREGISTER;
	req_p->token = vport->port_token;
	req_p->rx_flow_base = rx_chn->flow_base;
	req_p->rx_flow_offset = rx_chn->flow_offset;
	ether_addr_copy(req_p->mac_addr, vport->mac_addr);
	ret = send_request_get_response(proxy_priv, &resp_msg);
	mutex_unlock(&proxy_priv->req_params_mutex);
	if (ret)
		dev_err(proxy_priv->dev, "failed to deregister MAC Address\n");

	return ret;
}

static void vport_tx_cleanup(void *data, dma_addr_t desc_dma)
{
	struct tx_dma_chan *tx_chn = data;
	struct cppi5_host_desc_t *desc_tx;
	struct sk_buff *skb;
	void **swdata;

	desc_tx = k3_cppi_desc_pool_dma2virt(tx_chn->desc_pool, desc_dma);
	swdata = cppi5_hdesc_get_swdata(desc_tx);
	skb = *(swdata);
	vport_xmit_free(tx_chn, tx_chn->dev, desc_tx);

	dev_kfree_skb_any(skb);
}

static void vport_rx_cleanup(void *data, dma_addr_t desc_dma)
{
	struct rx_dma_chan *rx_chn = data;
	struct cppi5_host_desc_t *desc_rx;
	struct sk_buff *skb;
	dma_addr_t buf_dma;
	u32 buf_dma_len;
	void **swdata;

	desc_rx = k3_cppi_desc_pool_dma2virt(rx_chn->desc_pool, desc_dma);
	swdata = cppi5_hdesc_get_swdata(desc_rx);
	skb = *swdata;
	cppi5_hdesc_get_obuf(desc_rx, &buf_dma, &buf_dma_len);

	dma_unmap_single(rx_chn->dev, buf_dma, buf_dma_len, DMA_FROM_DEVICE);
	k3_cppi_desc_pool_free(rx_chn->desc_pool, desc_rx);

	dev_kfree_skb_any(skb);
}

static void vport_stop(struct virtual_port *vport)
{
	struct cpsw_proxy_priv *proxy_priv = vport->proxy_priv;
	struct rx_dma_chan *rx_chn;
	struct tx_dma_chan *tx_chn;
	int i;

	/* shutdown tx channels */
	atomic_set(&vport->tdown_cnt, vport->num_tx_chan);
	/* ensure new tdown_cnt value is visible */
	smp_mb__after_atomic();
	reinit_completion(&vport->tdown_complete);

	for (i = 0; i < vport->num_tx_chan; i++)
		k3_udma_glue_tdown_tx_chn(vport->tx_chans[i].tx_chan, false);

	i = wait_for_completion_timeout(&vport->tdown_complete, msecs_to_jiffies(1000));
	if (!i)
		dev_err(proxy_priv->dev, "tx teardown timeout\n");

	for (i = 0; i < vport->num_tx_chan; i++) {
		tx_chn = &vport->tx_chans[i];
		k3_udma_glue_reset_tx_chn(tx_chn->tx_chan, tx_chn, vport_tx_cleanup);
		k3_udma_glue_disable_tx_chn(tx_chn->tx_chan);
		napi_disable(&tx_chn->napi_tx);
	}

	for (i = 0; i < vport->num_rx_chan; i++) {
		rx_chn = &vport->rx_chans[i];
		k3_udma_glue_rx_flow_disable(rx_chn->rx_chan, 0);
		/* Need some delay to process RX ring before reset */
		msleep(100);
		k3_udma_glue_reset_rx_chn(rx_chn->rx_chan, 0, rx_chn, vport_rx_cleanup,
					  false);
		napi_disable(&rx_chn->napi_rx);
	}
}

static int vport_open(struct virtual_port *vport, netdev_features_t features)
{
	struct rx_dma_chan *rx_chn;
	struct tx_dma_chan *tx_chn;
	struct sk_buff *skb;
	u32 i, j;
	int ret;

	for (i = 0; i < vport->num_rx_chan; i++) {
		rx_chn = &vport->rx_chans[i];

		for (j = 0; j < rx_chn->num_descs; j++) {
			skb = __netdev_alloc_skb_ip_align(NULL, MAX_PACKET_SIZE, GFP_KERNEL);
			if (!skb)
				return -ENOMEM;

			ret = vport_rx_push(vport, skb, i);
			if (ret < 0) {
				netdev_err(vport->ndev,
					   "cannot submit skb to rx channel\n");
				kfree_skb(skb);
				return ret;
			}
			kmemleak_not_leak(skb);
		}

		ret = k3_udma_glue_rx_flow_enable(rx_chn->rx_chan, 0);
		if (ret)
			return ret;
	}

	for (i = 0; i < vport->num_tx_chan; i++) {
		tx_chn = &vport->tx_chans[i];
		ret = k3_udma_glue_enable_tx_chn(tx_chn->tx_chan);
		if (ret)
			return ret;
		napi_enable(&tx_chn->napi_tx);
	}

	for (i = 0; i < vport->num_rx_chan; i++) {
		rx_chn = &vport->rx_chans[i];
		napi_enable(&rx_chn->napi_rx);
	}

	return 0;
}

static int vport_ndo_stop(struct net_device *ndev)
{
	struct virtual_port *vport = vport_ndev_to_vport(ndev);
	struct cpsw_proxy_priv *proxy_priv = vport->proxy_priv;
	int ret;

	netif_tx_stop_all_queues(ndev);
	netif_carrier_off(ndev);

	ret = deregister_mac(vport);
	if (ret)
		netdev_err(ndev, "failed to deregister MAC for port %u\n",
			   vport->port_id);

	vport_stop(vport);

	dev_info(proxy_priv->dev, "stopped port %u on interface %s\n",
		 vport->port_id, ndev->name);

	return 0;
}

static int vport_ndo_open(struct net_device *ndev)
{
	struct virtual_port *vport = vport_ndev_to_vport(ndev);
	struct cpsw_proxy_priv *proxy_priv = vport->proxy_priv;
	int ret;
	u32 i;

	ret = netif_set_real_num_tx_queues(ndev, vport->num_tx_chan);
	if (ret)
		return ret;

	for (i = 0; i < vport->num_tx_chan; i++)
		netdev_tx_reset_queue(netdev_get_tx_queue(ndev, i));

	ret = vport_open(vport, ndev->features);
	if (ret)
		return ret;

	ret = register_mac(vport);
	if (ret) {
		netdev_err(ndev, "failed to register MAC for port: %u\n",
			   vport->port_id);
		vport_stop(vport);
		return -EIO;
	}

	netif_tx_wake_all_queues(ndev);
	netif_carrier_on(ndev);

	dev_info(proxy_priv->dev, "started port %u on interface %s\n",
		 vport->port_id, ndev->name);

	return 0;
}

static netdev_tx_t vport_ndo_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct virtual_port *vport = vport_ndev_to_vport(ndev);
	struct cppi5_host_desc_t *first_desc, *next_desc, *cur_desc;
	struct cpsw_proxy_priv *proxy_priv = vport->proxy_priv;
	struct device *dev = proxy_priv->dev;
	struct netdev_queue *netif_txq;
	dma_addr_t desc_dma, buf_dma;
	struct tx_dma_chan *tx_chn;
	void **swdata;
	int ret, i, q;
	u32 pkt_len;
	u32 *psdata;

	/* padding enabled in hw */
	pkt_len = skb_headlen(skb);

	/* Get Queue / TX DMA Channel for the SKB */
	q = skb_get_queue_mapping(skb);
	tx_chn = &vport->tx_chans[q];
	netif_txq = netdev_get_tx_queue(ndev, q);

	/* Map the linear buffer */
	buf_dma = dma_map_single(dev, skb->data, pkt_len,
				 DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(dev, buf_dma))) {
		dev_err(dev, "Failed to map tx skb buffer\n");
		ndev->stats.tx_errors++;
		goto drop_free_skb;
	}

	first_desc = k3_cppi_desc_pool_alloc(tx_chn->desc_pool);
	if (!first_desc) {
		dev_dbg(dev, "Failed to allocate descriptor\n");
		dma_unmap_single(dev, buf_dma, pkt_len, DMA_TO_DEVICE);
		goto busy_stop_q;
	}

	cppi5_hdesc_init(first_desc, CPPI5_INFO0_HDESC_EPIB_PRESENT,
			 PS_DATA_SIZE);
	cppi5_desc_set_pktids(&first_desc->hdr, 0, 0x3FFF);
	cppi5_hdesc_set_pkttype(first_desc, 0x7);
	/* target port has to be 0 */
	cppi5_desc_set_tags_ids(&first_desc->hdr, 0, vport->port_type);

	cppi5_hdesc_attach_buf(first_desc, buf_dma, pkt_len, buf_dma, pkt_len);
	swdata = cppi5_hdesc_get_swdata(first_desc);
	*(swdata) = skb;
	psdata = cppi5_hdesc_get_psdata(first_desc);

	/* HW csum offload if enabled */
	psdata[2] = 0;
	if (likely(skb->ip_summed == CHECKSUM_PARTIAL)) {
		unsigned int cs_start, cs_offset;

		cs_start = skb_transport_offset(skb);
		cs_offset = cs_start + skb->csum_offset;
		/* HW numerates bytes starting from 1 */
		psdata[2] = ((cs_offset + 1) << 24) |
			    ((cs_start + 1) << 16) | (skb->len - cs_start);
		dev_dbg(dev, "%s tx psdata:%#x\n", __func__, psdata[2]);
	}

	if (!skb_is_nonlinear(skb))
		goto done_tx;

	dev_dbg(dev, "fragmented SKB\n");

	/* Handle the case where skb is fragmented in pages */
	cur_desc = first_desc;
	for (i = 0; i < skb_shinfo(skb)->nr_frags; i++) {
		skb_frag_t *frag = &skb_shinfo(skb)->frags[i];
		u32 frag_size = skb_frag_size(frag);

		next_desc = k3_cppi_desc_pool_alloc(tx_chn->desc_pool);
		if (!next_desc) {
			dev_err(dev, "Failed to allocate descriptor\n");
			goto busy_free_descs;
		}

		buf_dma = skb_frag_dma_map(dev, frag, 0, frag_size,
					   DMA_TO_DEVICE);
		if (unlikely(dma_mapping_error(dev, buf_dma))) {
			dev_err(dev, "Failed to map tx skb page\n");
			k3_cppi_desc_pool_free(tx_chn->desc_pool, next_desc);
			ndev->stats.tx_errors++;
			goto drop_free_descs;
		}

		cppi5_hdesc_reset_hbdesc(next_desc);
		cppi5_hdesc_attach_buf(next_desc,
				       buf_dma, frag_size, buf_dma, frag_size);

		desc_dma = k3_cppi_desc_pool_virt2dma(tx_chn->desc_pool,
						      next_desc);
		cppi5_hdesc_link_hbdesc(cur_desc, desc_dma);

		pkt_len += frag_size;
		cur_desc = next_desc;
	}
	WARN_ON(pkt_len != skb->len);

done_tx:
	skb_tx_timestamp(skb);

	/* report bql before sending packet */
	dev_dbg(dev, "push 0 %d Bytes\n", pkt_len);

	netdev_tx_sent_queue(netif_txq, pkt_len);

	cppi5_hdesc_set_pktlen(first_desc, pkt_len);
	desc_dma = k3_cppi_desc_pool_virt2dma(tx_chn->desc_pool, first_desc);
	ret = k3_udma_glue_push_tx_chn(tx_chn->tx_chan, first_desc, desc_dma);
	if (ret) {
		dev_err(dev, "can't push desc %d\n", ret);
		/* inform bql */
		netdev_tx_completed_queue(netif_txq, 1, pkt_len);
		ndev->stats.tx_errors++;
		goto drop_free_descs;
	}

	if (k3_cppi_desc_pool_avail(tx_chn->desc_pool) < MAX_SKB_FRAGS) {
		netif_tx_stop_queue(netif_txq);
		/* Barrier, so that stop_queue visible to other cpus */
		smp_mb__after_atomic();
		dev_dbg(dev, "netif_tx_stop_queue %d\n", q);

		/* re-check for smp */
		if (k3_cppi_desc_pool_avail(tx_chn->desc_pool) >=
		    MAX_SKB_FRAGS) {
			netif_tx_wake_queue(netif_txq);
			dev_dbg(dev, "netif_tx_wake_queue %d\n", q);
		}
	}

	return NETDEV_TX_OK;

drop_free_descs:
	vport_xmit_free(tx_chn, dev, first_desc);
drop_free_skb:
	ndev->stats.tx_dropped++;
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;

busy_free_descs:
	vport_xmit_free(tx_chn, dev, first_desc);
busy_stop_q:
	netif_tx_stop_queue(netif_txq);
	return NETDEV_TX_BUSY;
}

static void vport_ndo_get_stats(struct net_device *ndev,
				struct rtnl_link_stats64 *stats)
{
	struct vport_netdev_priv *ndev_priv = netdev_priv(ndev);
	unsigned int start;
	int cpu;

	for_each_possible_cpu(cpu) {
		struct vport_netdev_stats *cpu_stats;
		u64 rx_packets;
		u64 rx_bytes;
		u64 tx_packets;
		u64 tx_bytes;

		cpu_stats = per_cpu_ptr(ndev_priv->stats, cpu);
		do {
			start = u64_stats_fetch_begin(&cpu_stats->syncp);
			rx_packets = cpu_stats->rx_packets;
			rx_bytes   = cpu_stats->rx_bytes;
			tx_packets = cpu_stats->tx_packets;
			tx_bytes   = cpu_stats->tx_bytes;
		} while (u64_stats_fetch_retry(&cpu_stats->syncp, start));

		stats->rx_packets += rx_packets;
		stats->rx_bytes   += rx_bytes;
		stats->tx_packets += tx_packets;
		stats->tx_bytes   += tx_bytes;
	}

	stats->rx_errors	= ndev->stats.rx_errors;
	stats->rx_dropped	= ndev->stats.rx_dropped;
	stats->tx_dropped	= ndev->stats.tx_dropped;
}

static void vport_ndo_tx_timeout(struct net_device *ndev, unsigned int txqueue)
{
	struct virtual_port *vport = vport_ndev_to_vport(ndev);
	struct netdev_queue *netif_txq;
	struct tx_dma_chan *tx_chn;
	unsigned long trans_start;

	/* process every txq */
	netif_txq = netdev_get_tx_queue(ndev, txqueue);
	tx_chn = &vport->tx_chans[txqueue];
	trans_start = READ_ONCE(netif_txq->trans_start);

	netdev_err(ndev, "txq:%d DRV_XOFF: %d tmo: %u dql_avail:%d free_desc:%zu\n",
		   txqueue, netif_tx_queue_stopped(netif_txq),
		   jiffies_to_msecs(jiffies - trans_start),
		   dql_avail(&netif_txq->dql),
		   k3_cppi_desc_pool_avail(tx_chn->desc_pool));

	if (netif_tx_queue_stopped(netif_txq)) {
		/* try to recover if it was stopped by driver */
		txq_trans_update(netif_txq);
		netif_tx_wake_queue(netif_txq);
	}
}

static const struct net_device_ops cpsw_proxy_client_netdev_ops = {
	.ndo_open		= vport_ndo_open,
	.ndo_stop		= vport_ndo_stop,
	.ndo_start_xmit		= vport_ndo_xmit,
	.ndo_get_stats64	= vport_ndo_get_stats,
	.ndo_tx_timeout		= vport_ndo_tx_timeout,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_set_mac_address	= eth_mac_addr,
};

static int init_netdev(struct cpsw_proxy_priv *proxy_priv, struct virtual_port *vport)
{
	struct device *dev = proxy_priv->dev;
	struct vport_netdev_priv *ndev_priv;
	struct rx_dma_chan *rx_chn;
	struct tx_dma_chan *tx_chn;
	int ret = 0;
	u32 i;

	vport->ndev = devm_alloc_etherdev_mqs(dev, sizeof(struct vport_netdev_priv),
					      vport->num_tx_chan, vport->num_rx_chan);

	if (!vport->ndev) {
		dev_err(dev, "error allocating netdev for port %u\n", vport->port_id);
		return -ENOMEM;
	}

	ndev_priv = netdev_priv(vport->ndev);
	ndev_priv->vport = vport;
	SET_NETDEV_DEV(vport->ndev, dev);

	if (is_valid_ether_addr(vport->mac_addr))
		eth_hw_addr_set(vport->ndev, vport->mac_addr);

	vport->ndev->min_mtu = MIN_PACKET_SIZE;
	vport->ndev->max_mtu = MAX_PACKET_SIZE;
	vport->ndev->hw_features = NETIF_F_SG | NETIF_F_RXCSUM;
	vport->ndev->features = vport->ndev->hw_features;
	vport->ndev->vlan_features |= NETIF_F_SG;
	vport->ndev->netdev_ops = &cpsw_proxy_client_netdev_ops;
	vport->ndev->ethtool_ops = &cpsw_proxy_client_ethtool_ops;

	ndev_priv->stats = netdev_alloc_pcpu_stats(struct vport_netdev_stats);
	if (!ndev_priv->stats)
		return -ENOMEM;

	ret = devm_add_action_or_reset(dev, (void(*)(void *))free_percpu, ndev_priv->stats);
	if (ret) {
		dev_err(dev, "failed to add free_percpu action, err: %d\n", ret);
		return ret;
	}

	for (i = 0; i < vport->num_tx_chan; i++) {
		tx_chn = &vport->tx_chans[i];
		netif_napi_add_tx(vport->ndev, &tx_chn->napi_tx, vport_tx_poll);
	}

	for (i = 0; i < vport->num_rx_chan; i++) {
		rx_chn = &vport->rx_chans[i];
		netif_napi_add(vport->ndev, &rx_chn->napi_rx, vport_rx_poll);
	}

	ret = register_netdev(vport->ndev);
	if (ret)
		dev_err(dev, "error registering net device, err: %d\n", ret);

	return ret;
}

static void unreg_netdevs(struct cpsw_proxy_priv *proxy_priv)
{
	struct virtual_port *vport;
	u32 i;

	for (i = 0; i < proxy_priv->num_virt_ports; i++) {
		vport = &proxy_priv->virt_ports[i];
		if (vport->ndev)
			unregister_netdev(vport->ndev);
	}
}

static int init_netdevs(struct cpsw_proxy_priv *proxy_priv)
{
	struct virtual_port *vport;
	int ret;
	u32 i;

	for (i = 0; i < proxy_priv->num_virt_ports; i++) {
		vport = &proxy_priv->virt_ports[i];
		ret = init_netdev(proxy_priv, vport);
		if (ret) {
			dev_err(proxy_priv->dev, "failed to initialize ndev for port %u\n",
				vport->port_id);
			goto err;
		}
	}

	return 0;

err:
	unreg_netdevs(proxy_priv);
	return ret;
}

static irqreturn_t tx_irq_handler(int irq, void *dev_id)
{
	struct tx_dma_chan *tx_chn = dev_id;

	disable_irq_nosync(irq);
	napi_schedule(&tx_chn->napi_tx);

	return IRQ_HANDLED;
}

static irqreturn_t rx_irq_handler(int irq, void *dev_id)
{
	struct rx_dma_chan *rx_chn = dev_id;

	disable_irq_nosync(irq);
	napi_schedule(&rx_chn->napi_rx);

	return IRQ_HANDLED;
}

static int register_dma_irq_handlers(struct cpsw_proxy_priv *proxy_priv)
{
	struct device *dev = proxy_priv->dev;
	struct rx_dma_chan *rx_chn;
	struct tx_dma_chan *tx_chn;
	struct virtual_port *vport;
	u32 i, j;
	int ret;

	for (i = 0; i < proxy_priv->num_virt_ports; i++) {
		vport = &proxy_priv->virt_ports[i];

		for (j = 0; j < vport->num_tx_chan; j++) {
			tx_chn = &vport->tx_chans[j];

			ret = devm_request_irq(dev, tx_chn->irq, tx_irq_handler,
					       IRQF_TRIGGER_HIGH, tx_chn->tx_chan_name, tx_chn);
			if (ret) {
				dev_err(dev, "failed to request tx irq: %u, err: %d\n",
					tx_chn->irq, ret);
				return ret;
			}
		}

		for (j = 0; j < vport->num_rx_chan; j++) {
			rx_chn = &vport->rx_chans[j];

			ret = devm_request_irq(dev, rx_chn->irq, rx_irq_handler,
					       IRQF_TRIGGER_HIGH, rx_chn->rx_chan_name, rx_chn);
			if (ret) {
				dev_err(dev, "failed to request rx irq: %u, err: %d\n",
					rx_chn->irq, ret);
				return ret;
			}
		}
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
