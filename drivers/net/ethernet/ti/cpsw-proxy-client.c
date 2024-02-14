// SPDX-License-Identifier: GPL-2.0
/* Texas Instruments CPSW Proxy Client Driver
 *
 * Copyright (C) 2023 Texas Instruments Incorporated - https://www.ti.com/
 *
 */

#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/if_vlan.h>
#include <linux/interrupt.h>
#include <linux/inetdevice.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/kmemleak.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/rpmsg.h>
#include <linux/dma/ti-cppi5.h>
#include <linux/dma/k3-udma-glue.h>

#include "k3-cppi-desc-pool.h"
#include "eth_remote_msg.h"

#define CPSW_PROXY_CLIENT_MIN_PACKET_SIZE	ETH_ZLEN
#define CPSW_PROXY_CLIENT_MAX_PACKET_SIZE	(VLAN_ETH_FRAME_LEN + ETH_FCS_LEN)
#define CPSW_PROXY_CLIENT_MAX_RX_FLOWS		1
#define CPSW_PROXY_CLIENT_MAX_RX_QUEUES		64
#define CPSW_PROXY_CLIENT_MAX_TX_QUEUES		8
#define CPSW_PROXY_CLIENT_REQ_TIMEOUT_MS	500

#define CPSW_PROXY_CLIENT_MAX_TX_DESC		500
#define CPSW_PROXY_CLIENT_MAX_RX_DESC		500

#define CPSW_PROXY_CLIENT_NAV_PS_DATA_SIZE	16
#define CPSW_PROXY_CLIENT_NAV_SW_DATA_SIZE	16

#define CPSW_PROXY_CLIENT_MAX_CHAN_NAME_LEN	128

enum cpsw_virt_port_type {
	VIRT_SWITCH_PORT,
	VIRT_MAC_ONLY_PORT,
};

struct cpsw_proxy_tx_chan {
	struct device			*dev;
	struct k3_cppi_desc_pool	*desc_pool;
	struct k3_udma_glue_tx_channel	*tx_chan;
	struct cpsw_virt_port		*virt_port;
	struct hrtimer			tx_hrtimer;
	struct napi_struct		napi_tx;
	u32				rel_chan_idx;
	u32				num_descs;
	u32				tx_psil_dest_id;
	u32				tx_pace_timeout;
	unsigned int			irq;
	char				tx_chan_name[CPSW_PROXY_CLIENT_MAX_CHAN_NAME_LEN];
	bool				is_valid;
};

struct cpsw_proxy_rx_chan {
	struct device			*dev;
	struct k3_cppi_desc_pool	*desc_pool;
	struct k3_udma_glue_rx_channel	*rx_chan;
	struct cpsw_virt_port		*virt_port;
	struct hrtimer			rx_hrtimer;
	struct napi_struct		napi_rx;
	u32				rx_flow_idx_base;
	u32				rx_flow_idx_offset;
	u32				rel_chan_idx;
	u32				num_descs;
	u32				rx_psil_src_id;
	unsigned long			rx_pace_timeout;
	unsigned int			irq;
	bool				rx_irq_disabled;
	char				rx_chan_name[CPSW_PROXY_CLIENT_MAX_CHAN_NAME_LEN];
	bool				is_valid;
};

struct cpsw_virt_port {
	struct cpsw_proxy_common	*common;
	struct net_device		*ndev;
	enum cpsw_virt_port_type	virt_port_type;
	struct cpsw_proxy_tx_chan	virt_port_tx_chan[CPSW_PROXY_CLIENT_MAX_TX_QUEUES];
	struct cpsw_proxy_rx_chan	virt_port_rx_chan[CPSW_PROXY_CLIENT_MAX_RX_QUEUES];
	struct workqueue_struct		*virt_port_wq;
	struct work_struct		rx_mode_work;
	struct notifier_block		virt_port_inetaddr_nb;
	struct netdev_hw_addr_list	mcast_list;
	struct completion		tdown_complete;
	struct mutex			mcast_filter_mutex; /* Multicast add/del ops mutex */
	atomic_t			tdown_cnt;
	bool				mcast_filter;
	bool				promisc_enabled;
	bool				mac_is_valid;
	u32				virt_port_id;
	u32				virt_port_token;
	u32				num_tx_chan;
	u32				num_rx_chan;
	u32				curr_tx_chan_idx;
	u32				curr_rx_chan_idx;
	u16				vlan_id;
	u8				ipv4_addr[ETHREMOTECFG_IPV4ADDRLEN];
	u8				mcast_mac_addr[ETH_ALEN];
	u8				mac_addr[ETH_ALEN];
};

struct cpsw_virt_port_ndev_stats {
	u64 tx_packets;
	u64 tx_bytes;
	u64 rx_packets;
	u64 rx_bytes;
	struct u64_stats_sync syncp;
};

struct cpsw_virt_port_ndev_priv {
	struct cpsw_virt_port_ndev_stats __percpu *stats;
	struct cpsw_virt_port *virt_port;
};

struct cpsw_proxy_common {
	struct rpmsg_device		*rpdev;
	struct device			*dev;
	struct cpsw_virt_port		*virt_ports;
	struct completion		wait_for_response;
	struct message			send_msg;
	struct message			recv_msg;
	const char			*dma_compatible;
	struct device_node		*dma_node;
	struct mutex			request_id_msg_mutex; /* RPMsg requests mutex */
	u32				request_id;
	u32				num_virt_ports;
	u32				num_active_tx_channels;
	u32				num_active_rx_channels;
	u32				virt_mac_only_ports; /* Bitmask */
	int				num_mac_only_ports;
	u32				virt_switch_ports; /* Bitmask */
	int				num_switch_ports;
	int				recv_msg_len;
	bool				in_recovery;
	bool				error_handled;
};

#define cpsw_virt_port_ndev_to_priv(ndev) \
	((struct cpsw_virt_port_ndev_priv *)netdev_priv(ndev))
#define cpsw_virt_port_ndev_to_virt_port(ndev) (cpsw_virt_port_ndev_to_priv(ndev)->virt_port)

/* Create a request message given the request type */
static int create_request(u32 token, u32 client_id, u32 req_type, struct cpsw_proxy_common *common,
			  struct cpsw_virt_port *virt_port)
{
	struct add_mcast_vlan_rx_flow_request *mcast_add_req;
	struct del_mcast_vlan_rx_flow_request *mcast_del_req;
	struct mac_rx_flow_register_request *mac_reg_req;
	struct ipv4_deregister_request *ipv4_dereg_req;
	struct common_request_message *common_req_msg;
	struct tx_psil_release_request *tx_free_req;
	struct rx_flow_release_request *rx_free_req;
	struct ipv4_register_request *ipv4_reg_req;
	struct request_message_header *req_msg_hdr;
	struct rx_flow_alloc_request *rx_alloc_req;
	struct tx_psil_alloc_request *tx_alloc_req;
	struct mac_release_request *mac_free_req;
	struct message *msg = &common->send_msg;
	struct cpsw_proxy_rx_chan *rx_chn;
	struct cpsw_proxy_tx_chan *tx_chn;
	struct attach_request *attach_req;

	/* Set message header fields */
	msg->msg_hdr.token = token;
	msg->msg_hdr.client_id = client_id;
	msg->msg_hdr.msg_type = ETHREMOTECFG_MSG_REQUEST;

	/* Handle differently based on type of request */
	switch (req_type) {
	case ETHREMOTECFG_ALLOC_RX:
		rx_alloc_req = (struct rx_flow_alloc_request *)msg;
		req_msg_hdr = &rx_alloc_req->request_msg_hdr;

		/* Set relative index of the requested RX Flow */
		rx_alloc_req->rx_flow_idx = virt_port->curr_rx_chan_idx;
		break;

	case ETHREMOTECFG_ALLOC_TX:
		tx_alloc_req = (struct tx_psil_alloc_request *)msg;
		req_msg_hdr = &tx_alloc_req->request_msg_hdr;

		/* Set relative index of the requested TX Channel */
		tx_alloc_req->tx_chan_idx = virt_port->curr_tx_chan_idx;
		break;

	case ETHREMOTECFG_ATTACH:
	case ETHREMOTECFG_ATTACH_EXT:
		attach_req = (struct attach_request *)msg;
		req_msg_hdr = &attach_req->request_msg_hdr;

		/* Set Virtual Port ID */
		attach_req->virt_port = virt_port->virt_port_id;

		break;

	case ETHREMOTECFG_ALLOC_MAC:
	case ETHREMOTECFG_DETACH:
	case ETHREMOTECFG_PORT_LINK_STATUS:
	case ETHREMOTECFG_PROMISC_DISABLE:
	case ETHREMOTECFG_PROMISC_ENABLE:
	case ETHREMOTECFG_TEARDOWN_COMPLETE:
	case ETHREMOTECFG_VIRT_PORT_ALLOC:
		common_req_msg = (struct common_request_message *)msg;
		req_msg_hdr = &common_req_msg->request_msg_hdr;

		break;

	case ETHREMOTECFG_FILTER_MAC_ADD:
		mcast_add_req = (struct add_mcast_vlan_rx_flow_request *)msg;
		req_msg_hdr = &mcast_add_req->request_msg_hdr;
		rx_chn = &virt_port->virt_port_rx_chan[virt_port->curr_rx_chan_idx];

		/* Set Multicast MAC Address */
		ether_addr_copy(mcast_add_req->mac_addr, virt_port->mcast_mac_addr);
		/* Set VLAN id */
		mcast_add_req->vlan_id = virt_port->vlan_id;
		/* Set RX Flow Index Base and Offset */
		mcast_add_req->rx_flow_idx_base = rx_chn->rx_flow_idx_base;
		mcast_add_req->rx_flow_idx_offset = rx_chn->rx_flow_idx_offset;

		break;

	case ETHREMOTECFG_FILTER_MAC_DEL:
		mcast_del_req = (struct del_mcast_vlan_rx_flow_request *)msg;
		req_msg_hdr = &mcast_del_req->request_msg_hdr;

		/* Set Multicast MAC Address */
		ether_addr_copy(mcast_del_req->mac_addr, virt_port->mcast_mac_addr);
		/* Set VLAN id */
		mcast_del_req->vlan_id = virt_port->vlan_id;

		break;

	case ETHREMOTECFG_FREE_MAC:
		mac_free_req = (struct mac_release_request *)msg;
		req_msg_hdr = &mac_free_req->request_msg_hdr;

		/* Set MAC Address to release */
		ether_addr_copy(mac_free_req->mac_addr, virt_port->mac_addr);

		break;

	case ETHREMOTECFG_FREE_RX:
		rx_free_req = (struct rx_flow_release_request *)msg;
		req_msg_hdr = &rx_free_req->request_msg_hdr;
		rx_chn = &virt_port->virt_port_rx_chan[virt_port->curr_rx_chan_idx];

		/* Set RX Flow Index Base and Offset to release */
		rx_free_req->rx_flow_idx_base = rx_chn->rx_flow_idx_base;
		rx_free_req->rx_flow_idx_offset = rx_chn->rx_flow_idx_offset;

		break;

	case ETHREMOTECFG_FREE_TX:
		tx_free_req = (struct tx_psil_release_request *)msg;
		req_msg_hdr = &tx_free_req->request_msg_hdr;
		tx_chn = &virt_port->virt_port_tx_chan[virt_port->curr_tx_chan_idx];

		tx_free_req->tx_psil_dest_id = tx_chn->tx_psil_dest_id;

		break;

	case ETHREMOTECFG_IPv4_REGISTER:
		ipv4_reg_req = (struct ipv4_register_request *)msg;
		req_msg_hdr = &ipv4_reg_req->request_msg_hdr;

		memcpy(&ipv4_reg_req->ipv4_addr, virt_port->ipv4_addr, ETHREMOTECFG_IPV4ADDRLEN);
		ether_addr_copy(ipv4_reg_req->mac_addr, virt_port->mac_addr);

		break;

	case ETHREMOTECFG_IPv4_DEREGISTER:
		ipv4_dereg_req = (struct ipv4_deregister_request *)msg;
		req_msg_hdr = &ipv4_dereg_req->request_msg_hdr;

		memcpy(&ipv4_dereg_req->ipv4_addr, virt_port->ipv4_addr, ETHREMOTECFG_IPV4ADDRLEN);

		break;

	case ETHREMOTECFG_MAC_REGISTER:
	case ETHREMOTECFG_MAC_DEREGISTER:
		mac_reg_req = (struct mac_rx_flow_register_request *)msg;
		req_msg_hdr = &mac_reg_req->request_msg_hdr;
		rx_chn = &virt_port->virt_port_rx_chan[virt_port->curr_rx_chan_idx];

		ether_addr_copy(mac_reg_req->mac_addr, virt_port->mac_addr);
		mac_reg_req->rx_flow_idx_base = rx_chn->rx_flow_idx_base;
		mac_reg_req->rx_flow_idx_offset = rx_chn->rx_flow_idx_offset;

		break;

	case ETHREMOTECFG_SET_RX_DEFAULTFLOW:
	case ETHREMOTECFG_DEL_RX_DEFAULTFLOW:
	case ETHREMOTECFG_REGISTER_READ:
	case ETHREMOTECFG_REGISTER_WRITE:
	case ETHREMOTECFG_MATCH_ETHTYPE_REGISTER:
	case ETHREMOTECFG_MATCH_ETHTYPE_DEREGISTER:
	case ETHREMOTECFG_REMOTE_TIMER_REGISTER:
	case ETHREMOTECFG_REMOTE_TIMER_DEREGISTER:
	case ETHREMOTECFG_MESSAGE_PING:
	case ETHREMOTECFG_ETHFW_STATUS:
	case ETHREMOTECFG_CLIENT_DUMPSTATS:
	default:
		pr_err("Unsupported request type\n");
		return -EOPNOTSUPP;
	}

	/* Set request message header fields */
	req_msg_hdr->request_id = common->request_id;
	req_msg_hdr->request_type = req_type;

	return 0;
}

static int cpsw_proxy_client_send_rpmsg(struct cpsw_proxy_common *common)
{
	struct rpmsg_device *rpdev = common->rpdev;
	int ret;

	reinit_completion(&common->wait_for_response);
	ret = rpmsg_send(rpdev->ept, (void *)(&common->send_msg), sizeof(struct message));
	if (ret) {
		dev_err(common->dev, "failed to send rpmsg\n");
		return ret;
	}

	return 0;
}

static int
cpsw_proxy_client_send_request(struct cpsw_proxy_common *common, struct cpsw_virt_port *virt_port,
			       u32 token, u32 req_type, struct message *response)
{
	struct response_message_header *response_msg_hdr;
	unsigned long timeout;
	u32 response_status;
	bool retry = 0;
	int ret;

	mutex_lock(&common->request_id_msg_mutex);
	ret = create_request(token, ETHREMOTECFG_LINUX_CLIENT, req_type, common, virt_port);
	if (ret) {
		dev_err(common->dev, "failed to create request %d\n", ret);
		goto err;
	}

	ret = cpsw_proxy_client_send_rpmsg(common);
	if (ret) {
		dev_err(common->dev, "failed to send request %d\n", ret);
		goto err;
	}

	/* Ensure that response is received by the callback handler */
	timeout = msecs_to_jiffies(CPSW_PROXY_CLIENT_REQ_TIMEOUT_MS);
	ret = wait_for_completion_timeout(&common->wait_for_response, timeout);
	if (!ret) {
		dev_err(common->dev, "failed to receive response. Response Timedout\n");
		ret = -ETIMEDOUT;
		goto err;
	}

	/* Store response */
	memcpy(response, &common->recv_msg, common->recv_msg_len);
	response_msg_hdr = (struct response_message_header *)response;
	response_status = response_msg_hdr->response_status;

	if (unlikely(response_status != ETHREMOTECFG_CMDSTATUS_OK)) {
		if (response_status == ETHREMOTECFG_CMDSTATUS_EAGAIN) {
			dev_info(common->dev, "resending request\n");
			retry = 1;
			goto err;
		} else {
			dev_err(common->dev, "bad response status: %d\n", response_status);
			ret = -EIO;
			goto err;
		}
	}

	common->request_id++;
	mutex_unlock(&common->request_id_msg_mutex);

	return 0;

err:
	common->request_id++;
	mutex_unlock(&common->request_id_msg_mutex);
	if (retry)
		ret = cpsw_proxy_client_send_request(common, virt_port, token, req_type,
						     response);
	return ret;
}

static struct cpsw_virt_port *get_virt_port_from_token(struct cpsw_proxy_common *common, u32 token)
{
	struct cpsw_virt_port *virt_port;
	int i;

	for (i = 0; i < common->num_virt_ports; i++) {
		virt_port = &common->virt_ports[i];

		if (virt_port->virt_port_token == token)
			return virt_port;
	}

	return NULL;
}

static int cpsw_proxy_client_server_recovery_notify_handler(void *data)
{
	struct cpsw_virt_port *virt_port = (struct cpsw_virt_port *)data;
	struct cpsw_proxy_common *common = virt_port->common;
	struct net_device *ndev;

	ndev = virt_port->ndev;
	netif_device_attach(ndev);
	if (!netif_running(ndev)) {
		rtnl_lock();
		dev_open(ndev, NULL);
		rtnl_unlock();
	}

	common->in_recovery = 0;
	return 0;
}

static int cpsw_proxy_client_server_error_notify_handler(void *data)
{
	struct cpsw_virt_port *virt_port = (struct cpsw_virt_port *)data;
	struct cpsw_proxy_common *common = virt_port->common;
	struct net_device *ndev;
	struct message response;
	int ret;

	ndev = virt_port->ndev;
	netif_device_detach(ndev);
	if (netif_running(ndev)) {
		rtnl_lock();
		dev_close(ndev);
		rtnl_unlock();
	}

	ret = cpsw_proxy_client_send_request(common, virt_port, virt_port->virt_port_token,
					     ETHREMOTECFG_TEARDOWN_COMPLETE, &response);
	if (ret) {
		dev_err(common->dev, "failed to notify teardown completion err: %d\n", ret);
		return ret;
	}

	common->error_handled = 1;
	return 0;
}

static int cpsw_proxy_client_cb(struct rpmsg_device *rpdev, void *data,
				int len, void *priv, u32 src)
{
	struct cpsw_proxy_common *common = dev_get_drvdata(&rpdev->dev);
	struct response_message_header *response_msg_hdr;
	struct notify_message_header *notify_msg_hdr;
	struct message *msg = (struct message *)data;
	struct task_struct *server_notify_handler;
	u32 msg_type = msg->msg_hdr.msg_type;
	u32 notify_type, response_id, token;
	struct cpsw_virt_port *virt_port;
	int response_status, ret = 0;

	switch (msg_type) {
	case ETHREMOTECFG_MSG_NOTIFY:
		notify_msg_hdr = (struct notify_message_header *)msg;
		notify_type = notify_msg_hdr->notify_type;
		token = notify_msg_hdr->msg_hdr.token;

		switch (notify_type) {
		case ETHREMOTECFG_NOTIFYTYPE_HWERROR:
			common->in_recovery = 1;
			virt_port = get_virt_port_from_token(common, token);
			if (!virt_port) {
				dev_err(common->dev, "invalid notification received\n");
				return -EINVAL;
			}

			server_notify_handler =
					kthread_run(&cpsw_proxy_client_server_error_notify_handler,
						    virt_port, "hwerr_handler");

			if (IS_ERR(server_notify_handler))
				ret = PTR_ERR(server_notify_handler);
			break;

		case ETHREMOTECFG_NOTIFYTYPE_HWRECOVERY_COMPLETE:
			common->error_handled = 0;
			virt_port = get_virt_port_from_token(common, token);
			if (!virt_port) {
				dev_err(common->dev, "invalid notification received\n");
				return -EINVAL;
			}

			server_notify_handler =
				kthread_run(&cpsw_proxy_client_server_recovery_notify_handler,
					    virt_port, "recovery_handler");

			if (IS_ERR(server_notify_handler))
				ret = PTR_ERR(server_notify_handler);
			break;

		default:
			break;
		}

		break;

	case ETHREMOTECFG_MSG_RESPONSE:
		response_msg_hdr = (struct response_message_header *)msg;
		response_id = response_msg_hdr->response_id;
		response_status = response_msg_hdr->response_status;

		if (response_id == common->request_id - 1) {
			dev_info(common->dev, "ignoring delayed response for request: %u\n",
				 response_id);
			return 0;
		}

		if (response_id != common->request_id) {
			dev_err(common->dev, "expected response id: %u but received %u\n",
				common->request_id, response_id);
			return -EINVAL;
		}

		memcpy(&common->recv_msg, msg, len);
		common->recv_msg_len = len;
		complete(&common->wait_for_response);
		break;

	default:
		dev_err(common->dev, "invalid message received\n");
		return -EOPNOTSUPP;
	}

	return ret;
}

static int cpsw_virt_port_rx_push(struct cpsw_virt_port *virt_port, struct sk_buff *skb,
				  u32 rel_chan_idx)
{
	struct cpsw_proxy_rx_chan *rx_chn = &virt_port->virt_port_rx_chan[rel_chan_idx];
	struct cpsw_proxy_common *common = virt_port->common;
	struct cppi5_host_desc_t *desc_rx;
	struct device *dev = common->dev;
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
			 CPSW_PROXY_CLIENT_NAV_PS_DATA_SIZE);
	cppi5_hdesc_attach_buf(desc_rx, 0, 0, buf_dma, skb_tailroom(skb));
	swdata = cppi5_hdesc_get_swdata(desc_rx);
	*((void **)swdata) = skb;

	return k3_udma_glue_push_rx_chn(rx_chn->rx_chan, 0, desc_rx, desc_dma);
}

static int cpsw_virt_port_open(struct cpsw_virt_port *virt_port, netdev_features_t features)
{
	struct cpsw_proxy_common *common = virt_port->common;
	struct cpsw_proxy_rx_chan *rx_chn;
	struct sk_buff *skb;
	int i, j, ret;

	for (i = 0; i < virt_port->num_rx_chan; i++) {
		rx_chn = &virt_port->virt_port_rx_chan[i];

		for (j = 0; j < rx_chn->num_descs; j++) {
			skb = __netdev_alloc_skb_ip_align(NULL, CPSW_PROXY_CLIENT_MAX_PACKET_SIZE,
							  GFP_KERNEL);
			if (!skb)
				return -ENOMEM;

			ret = cpsw_virt_port_rx_push(virt_port, skb, i);
			if (ret < 0) {
				dev_err(common->dev,
					"cannot submit skb to channel rx, error %d\n",
					ret);
				kfree_skb(skb);
				return ret;
			}
			kmemleak_not_leak(skb);
		}

		ret = k3_udma_glue_rx_flow_enable(rx_chn->rx_chan, 0);
		if (ret)
			return ret;
	}

	for (i = 0; i < virt_port->num_tx_chan; i++) {
		ret = k3_udma_glue_enable_tx_chn(virt_port->virt_port_tx_chan[i].tx_chan);
		if (ret)
			return ret;
		napi_enable(&virt_port->virt_port_tx_chan[i].napi_tx);
	}

	for (i = 0; i < virt_port->num_rx_chan; i++) {
		rx_chn = &virt_port->virt_port_rx_chan[i];
		napi_enable(&rx_chn->napi_rx);
		if (rx_chn->rx_irq_disabled) {
			rx_chn->rx_irq_disabled = false;
			enable_irq(rx_chn->irq);
		}
	}

	return 0;
}

static int cpsw_proxy_client_register_mac(struct cpsw_virt_port *virt_port)
{
	struct cpsw_proxy_common *common = virt_port->common;
	struct message response;
	int ret;

	/* Register MAC address only for default RX Channel/Flow */
	virt_port->curr_rx_chan_idx = 0;
	ret = cpsw_proxy_client_send_request(common, virt_port, virt_port->virt_port_token,
					     ETHREMOTECFG_MAC_REGISTER, &response);
	if (ret)
		dev_err(common->dev, "failed to register mac err: %d\n", ret);

	return ret;
}

static int cpsw_proxy_client_deregister_mac(struct cpsw_virt_port *virt_port)
{
	struct cpsw_proxy_common *common = virt_port->common;
	struct message response;
	int ret;

	ret = cpsw_proxy_client_send_request(common, virt_port, virt_port->virt_port_token,
					     ETHREMOTECFG_MAC_DEREGISTER, &response);
	if (ret)
		dev_err(common->dev, "failed to deregister mac err: %d\n", ret);

	return ret;
}

static void cpsw_virt_port_xmit_free(struct cpsw_proxy_tx_chan *tx_chn,
				     struct device *dev,
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

static void cpsw_virt_port_tx_cleanup(void *data, dma_addr_t desc_dma)
{
	struct cpsw_proxy_tx_chan *tx_chn = data;
	struct cppi5_host_desc_t *desc_tx;
	struct sk_buff *skb;
	void **swdata;

	desc_tx = k3_cppi_desc_pool_dma2virt(tx_chn->desc_pool, desc_dma);
	swdata = cppi5_hdesc_get_swdata(desc_tx);
	skb = *(swdata);
	cpsw_virt_port_xmit_free(tx_chn, tx_chn->dev, desc_tx);

	dev_kfree_skb_any(skb);
}

static void cpsw_virt_port_rx_cleanup(void *data, dma_addr_t desc_dma)
{
	struct cpsw_proxy_rx_chan *rx_chn = data;
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

static void cpsw_virt_port_stop(struct cpsw_virt_port *virt_port)
{
	struct cpsw_proxy_common *common = virt_port->common;
	struct cpsw_proxy_rx_chan *rx_chn;
	struct cpsw_proxy_tx_chan *tx_chn;
	int i;

	/* shutdown tx channels */
	atomic_set(&virt_port->tdown_cnt, virt_port->num_tx_chan);
	/* ensure new tdown_cnt value is visible */
	smp_mb__after_atomic();
	reinit_completion(&virt_port->tdown_complete);

	for (i = 0; i < virt_port->num_tx_chan; i++)
		k3_udma_glue_tdown_tx_chn(virt_port->virt_port_tx_chan[i].tx_chan, false);

	i = wait_for_completion_timeout(&virt_port->tdown_complete, msecs_to_jiffies(1000));
	if (!i)
		dev_err(common->dev, "tx teardown timeout\n");

	for (i = 0; i < virt_port->num_tx_chan; i++) {
		tx_chn = &virt_port->virt_port_tx_chan[i];
		k3_udma_glue_reset_tx_chn(tx_chn->tx_chan, tx_chn, cpsw_virt_port_tx_cleanup);
		k3_udma_glue_disable_tx_chn(tx_chn->tx_chan);
		napi_disable(&tx_chn->napi_tx);
		hrtimer_cancel(&tx_chn->tx_hrtimer);
	}

	for (i = 0; i < virt_port->num_rx_chan; i++) {
		rx_chn = &virt_port->virt_port_rx_chan[i];
		k3_udma_glue_rx_flow_disable(rx_chn->rx_chan, 0);
		/* Need some delay to process RX ring before reset */
		msleep(100);
		k3_udma_glue_reset_rx_chn(rx_chn->rx_chan, 0, rx_chn, cpsw_virt_port_rx_cleanup,
					  false);
		napi_disable(&rx_chn->napi_rx);
		hrtimer_cancel(&rx_chn->rx_hrtimer);
	}

	cancel_work_sync(&virt_port->rx_mode_work);
}

static int cpsw_virt_port_add_mcast(struct net_device *ndev, const u8 *addr)
{
	struct cpsw_virt_port *virt_port = cpsw_virt_port_ndev_to_virt_port(ndev);
	struct cpsw_proxy_common *common = virt_port->common;
	struct message response;
	int ret;

	mutex_lock(&virt_port->mcast_filter_mutex);
	ether_addr_copy(virt_port->mcast_mac_addr, addr);
	virt_port->vlan_id = ETHREMOTECFG_ETHSWITCH_VLAN_USE_DFLT;
	/* Register Multicast MAC address only for default RX Channel/Flow */
	virt_port->curr_rx_chan_idx = 0;

	ret = cpsw_proxy_client_send_request(common, virt_port, virt_port->virt_port_token,
					     ETHREMOTECFG_FILTER_MAC_ADD, &response);
	if (ret)
		dev_err(common->dev, "failed to add multicast mac filter err: %d\n", ret);

	mutex_unlock(&virt_port->mcast_filter_mutex);
	return ret;
}

static int cpsw_virt_port_del_mcast(struct net_device *ndev, const u8 *addr)
{
	struct cpsw_virt_port *virt_port = cpsw_virt_port_ndev_to_virt_port(ndev);
	struct cpsw_proxy_common *common = virt_port->common;
	struct message response;
	int ret;

	mutex_lock(&virt_port->mcast_filter_mutex);
	ether_addr_copy(virt_port->mcast_mac_addr, addr);
	virt_port->vlan_id = ETHREMOTECFG_ETHSWITCH_VLAN_USE_DFLT;

	ret = cpsw_proxy_client_send_request(common, virt_port, virt_port->virt_port_token,
					     ETHREMOTECFG_FILTER_MAC_DEL, &response);
	if (ret)
		dev_err(common->dev, "failed to delete multicast mac filter err: %d\n", ret);

	mutex_unlock(&virt_port->mcast_filter_mutex);
	return ret;
}

static int cpsw_virt_port_ndo_stop(struct net_device *ndev)
{
	struct cpsw_virt_port *virt_port = cpsw_virt_port_ndev_to_virt_port(ndev);
	struct cpsw_proxy_common *common = virt_port->common;
	int ret;

	netif_tx_stop_all_queues(ndev);
	netif_carrier_off(ndev);

	ret = cpsw_proxy_client_deregister_mac(virt_port);
	if (ret)
		dev_err(common->dev, "failed to deregister mac for virt port %u\n",
			virt_port->virt_port_id);

	__dev_mc_unsync(ndev, cpsw_virt_port_del_mcast);
	__hw_addr_init(&virt_port->mcast_list);
	cpsw_virt_port_stop(virt_port);

	dev_info(common->dev, "stopped virt port: %u on interface: %s\n", virt_port->virt_port_id,
		 virt_port->ndev->name);
	return 0;
}

static int cpsw_virt_port_ndo_open(struct net_device *ndev)
{
	struct cpsw_virt_port *virt_port = cpsw_virt_port_ndev_to_virt_port(ndev);
	struct cpsw_proxy_common *common = virt_port->common;
	int ret, i;

	/* Number of TX DMA Channels available to transfer data */
	ret = netif_set_real_num_tx_queues(ndev, virt_port->num_tx_chan);
	if (ret)
		return ret;

	for (i = 0; i < virt_port->num_tx_chan; i++)
		netdev_tx_reset_queue(netdev_get_tx_queue(ndev, i));

	ret = cpsw_virt_port_open(virt_port, ndev->features);
	if (ret)
		return ret;

	ret = cpsw_proxy_client_register_mac(virt_port);
	if (ret) {
		dev_err(common->dev, "failed to register mac for virt port %u\n",
			virt_port->virt_port_id);
		cpsw_virt_port_stop(virt_port);
		return ret;
	}

	netif_tx_wake_all_queues(ndev);
	netif_carrier_on(ndev);

	dev_info(common->dev, "started virt port: %u on interface %s\n", virt_port->virt_port_id,
		 virt_port->ndev->name);
	return 0;
}

static netdev_tx_t cpsw_virt_port_ndo_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct cpsw_virt_port *virt_port = cpsw_virt_port_ndev_to_virt_port(ndev);
	struct cppi5_host_desc_t *first_desc, *next_desc, *cur_desc;
	struct cpsw_proxy_common *common = virt_port->common;
	struct cpsw_proxy_tx_chan *tx_chn;
	struct device *dev = common->dev;
	struct netdev_queue *netif_txq;
	dma_addr_t desc_dma, buf_dma;
	void **swdata;
	int ret, i, q;
	u32 pkt_len;
	u32 *psdata;

	/* padding enabled in hw */
	pkt_len = skb_headlen(skb);

	/* Get Queue / TX DMA Channel for the SKB */
	q = skb_get_queue_mapping(skb);
	tx_chn = &virt_port->virt_port_tx_chan[q];
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
			 CPSW_PROXY_CLIENT_NAV_PS_DATA_SIZE);
	cppi5_desc_set_pktids(&first_desc->hdr, 0, 0x3FFF);
	cppi5_hdesc_set_pkttype(first_desc, 0x7);
	/* target port has to be 0 */
	cppi5_desc_set_tags_ids(&first_desc->hdr, 0, virt_port->virt_port_type);

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
	cpsw_virt_port_xmit_free(tx_chn, dev, first_desc);
drop_free_skb:
	ndev->stats.tx_dropped++;
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;

busy_free_descs:
	cpsw_virt_port_xmit_free(tx_chn, dev, first_desc);
busy_stop_q:
	netif_tx_stop_queue(netif_txq);
	return NETDEV_TX_BUSY;
}

static void cpsw_virt_port_ndo_get_stats(struct net_device *ndev,
					 struct rtnl_link_stats64 *stats)
{
	struct cpsw_virt_port_ndev_priv *ndev_priv = netdev_priv(ndev);
	unsigned int start;
	int cpu;

	for_each_possible_cpu(cpu) {
		struct cpsw_virt_port_ndev_stats *cpu_stats;
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

static void cpsw_virt_port_host_tx_timeout(struct net_device *ndev, unsigned int txqueue)
{
	struct cpsw_virt_port *virt_port = cpsw_virt_port_ndev_to_virt_port(ndev);
	struct cpsw_proxy_tx_chan *tx_chn;
	struct netdev_queue *netif_txq;
	unsigned long trans_start;

	/* process every txq */
	netif_txq = netdev_get_tx_queue(ndev, txqueue);
	tx_chn = &virt_port->virt_port_tx_chan[txqueue];
	trans_start = READ_ONCE(netif_txq->trans_start);

	netdev_err(ndev, "txq:%d DRV_XOFF:%d tmo:%u dql_avail:%d free_desc:%zu\n",
		   txqueue,
		   netif_tx_queue_stopped(netif_txq),
		   jiffies_to_msecs(jiffies - trans_start),
		   dql_avail(&netif_txq->dql),
		   k3_cppi_desc_pool_avail(tx_chn->desc_pool));

	if (netif_tx_queue_stopped(netif_txq)) {
		/* try recover if stopped by us */
		txq_trans_update(netif_txq);
		netif_tx_wake_queue(netif_txq);
	}
}

static void cpsw_virt_port_set_rx_mode_work(struct work_struct *work)
{
	struct cpsw_virt_port *virt_port = container_of(work, struct cpsw_virt_port, rx_mode_work);
	struct cpsw_proxy_common *common = virt_port->common;
	struct net_device *ndev;
	struct message response;
	u32 promisc_req_type;
	bool promisc_status;
	int ret;

	if (virt_port->virt_port_type == VIRT_MAC_ONLY_PORT) {
		/* If current state is same as desired state, exit */
		promisc_status = virt_port->ndev->flags & IFF_PROMISC;
		if (promisc_status == virt_port->promisc_enabled)
			return;

		virt_port->promisc_enabled = promisc_status;
		if (virt_port->promisc_enabled)
			promisc_req_type = ETHREMOTECFG_PROMISC_ENABLE;
		else
			promisc_req_type = ETHREMOTECFG_PROMISC_DISABLE;

		ret = cpsw_proxy_client_send_request(common, virt_port, virt_port->virt_port_token,
						     promisc_req_type, &response);
		if (ret)
			dev_err(common->dev, "failed to send Promiscuous Mode request %d\n", ret);

	} else if (virt_port->mcast_filter) {
		ndev = virt_port->ndev;

		/* make a mc list copy */
		netif_addr_lock_bh(ndev);
		__hw_addr_sync(&virt_port->mcast_list, &ndev->mc, ndev->addr_len);
		netif_addr_unlock_bh(ndev);

		__hw_addr_sync_dev(&virt_port->mcast_list, ndev,
				   cpsw_virt_port_add_mcast, cpsw_virt_port_del_mcast);
	}
}

static void cpsw_virt_port_set_rx_mode(struct net_device *ndev)
{
	struct cpsw_virt_port *virt_port = cpsw_virt_port_ndev_to_virt_port(ndev);

	if (virt_port->virt_port_type == VIRT_MAC_ONLY_PORT || virt_port->mcast_filter)
		queue_work(virt_port->virt_port_wq, &virt_port->rx_mode_work);
}

static const struct net_device_ops cpsw_virt_port_netdev_ops = {
	.ndo_open		= cpsw_virt_port_ndo_open,
	.ndo_stop		= cpsw_virt_port_ndo_stop,
	.ndo_start_xmit		= cpsw_virt_port_ndo_xmit,
	.ndo_get_stats64        = cpsw_virt_port_ndo_get_stats,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_tx_timeout		= cpsw_virt_port_host_tx_timeout,
	.ndo_set_rx_mode	= cpsw_virt_port_set_rx_mode,
};

static int cpsw_virt_port_get_coalesce(struct net_device *ndev, struct ethtool_coalesce *coal,
				       struct kernel_ethtool_coalesce *kernel_coal,
				       struct netlink_ext_ack *extack)
{
	struct cpsw_virt_port *virt_port = cpsw_virt_port_ndev_to_virt_port(ndev);

	coal->tx_coalesce_usecs = virt_port->virt_port_tx_chan[0].tx_pace_timeout / 1000;
	coal->rx_coalesce_usecs = virt_port->virt_port_rx_chan[0].rx_pace_timeout / 1000;
	return 0;
}

static int cpsw_virt_port_set_coalesce(struct net_device *ndev, struct ethtool_coalesce *coal,
				       struct kernel_ethtool_coalesce *kernel_coal,
				       struct netlink_ext_ack *extack)
{
	struct cpsw_virt_port *virt_port = cpsw_virt_port_ndev_to_virt_port(ndev);
	struct cpsw_proxy_common *common = virt_port->common;
	int i;

	if (coal->tx_coalesce_usecs && coal->tx_coalesce_usecs < 20) {
		dev_err(common->dev, "TX coalesce must be at least 20 usecs. Defaulting to 20 usecs\n");
		coal->tx_coalesce_usecs = 20;
	}

	if (coal->rx_coalesce_usecs && coal->rx_coalesce_usecs < 20) {
		dev_err(common->dev, "RX coalesce must be at least 20 usecs. Defaulting to 20 usecs\n");
		coal->rx_coalesce_usecs = 20;
	}

	/* Since it is possible to set pacing values per TX and RX queue, if per queue value is
	 * not specified, apply it to all available TX and RX queues.
	 */

	for (i = 0; i < virt_port->num_tx_chan; i++)
		virt_port->virt_port_tx_chan[i].tx_pace_timeout = coal->tx_coalesce_usecs * 1000;

	for (i = 0; i < virt_port->num_rx_chan; i++)
		virt_port->virt_port_rx_chan[i].rx_pace_timeout = coal->rx_coalesce_usecs * 1000;

	return 0;
}

static int cpsw_virt_port_get_per_queue_coalesce(struct net_device *ndev, u32 q,
						 struct ethtool_coalesce *coal)
{
	struct cpsw_virt_port *virt_port = cpsw_virt_port_ndev_to_virt_port(ndev);

	if (q >= virt_port->num_tx_chan || q >= virt_port->num_rx_chan)
		return -EINVAL;

	coal->tx_coalesce_usecs = virt_port->virt_port_tx_chan[q].tx_pace_timeout / 1000;
	coal->rx_coalesce_usecs = virt_port->virt_port_rx_chan[q].rx_pace_timeout / 1000;

	return 0;
}

static int cpsw_virt_port_set_per_queue_coalesce(struct net_device *ndev, u32 q,
						 struct ethtool_coalesce *coal)
{
	struct cpsw_virt_port *virt_port = cpsw_virt_port_ndev_to_virt_port(ndev);
	struct cpsw_proxy_common *common = virt_port->common;

	if (q >= virt_port->num_tx_chan || q >= virt_port->num_rx_chan)
		return -EINVAL;

	if (coal->tx_coalesce_usecs && coal->tx_coalesce_usecs < 20) {
		dev_err(common->dev, "TX coalesce must be at least 20 usecs. Defaulting to 20 usecs\n");
		coal->tx_coalesce_usecs = 20;
	}

	if (coal->rx_coalesce_usecs && coal->rx_coalesce_usecs < 20) {
		dev_err(common->dev, "RX coalesce must be at least 20 usecs. Defaulting to 20 usecs\n");
		coal->rx_coalesce_usecs = 20;
	}

	virt_port->virt_port_tx_chan[q].tx_pace_timeout = coal->tx_coalesce_usecs * 1000;
	virt_port->virt_port_rx_chan[q].rx_pace_timeout = coal->rx_coalesce_usecs * 1000;

	return 0;
}

static u32 cpsw_virt_port_get_link(struct net_device *ndev)
{
	struct cpsw_virt_port *virt_port = cpsw_virt_port_ndev_to_virt_port(ndev);
	struct cpsw_proxy_common *common = virt_port->common;
	struct port_link_status_response *link_stat;
	struct message response;
	bool link_up;
	int ret;

	if (virt_port->virt_port_type != VIRT_MAC_ONLY_PORT)
		return ethtool_op_get_link(ndev);

	ret = cpsw_proxy_client_send_request(common, virt_port, virt_port->virt_port_token,
					     ETHREMOTECFG_PORT_LINK_STATUS, &response);
	if (ret) {
		dev_err(common->dev, "failed to get link status err: %d\n", ret);
		/* Assume that link is down if status is not known */
		return 0;
	}

	link_stat = (struct port_link_status_response *)&response;
	link_up = link_stat->link_up;

	return link_up;
}

const struct ethtool_ops cpsw_virt_port_ethtool_ops = {
	.get_link		= cpsw_virt_port_get_link,
	.supported_coalesce_params = ETHTOOL_COALESCE_USECS,
	.get_coalesce           = cpsw_virt_port_get_coalesce,
	.set_coalesce           = cpsw_virt_port_set_coalesce,
	.get_per_queue_coalesce = cpsw_virt_port_get_per_queue_coalesce,
	.set_per_queue_coalesce = cpsw_virt_port_set_per_queue_coalesce,
};

static enum hrtimer_restart cpsw_virt_port_tx_timer_callback(struct hrtimer *timer)
{
	struct cpsw_proxy_tx_chan *tx_chn = container_of(timer, struct cpsw_proxy_tx_chan,
							 tx_hrtimer);

	enable_irq(tx_chn->irq);
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart cpsw_virt_port_rx_timer_callback(struct hrtimer *timer)
{
	struct cpsw_proxy_rx_chan *rx_chn = container_of(timer, struct cpsw_proxy_rx_chan,
							 rx_hrtimer);

	enable_irq(rx_chn->irq);
	return HRTIMER_NORESTART;
}

static int cpsw_virt_port_tx_compl_packets(struct cpsw_virt_port *virt_port,
					   int chn, unsigned int budget, bool *tdown)
{
	struct cpsw_proxy_common *common = virt_port->common;
	struct cppi5_host_desc_t *desc_tx;
	struct cpsw_proxy_tx_chan *tx_chn;
	struct device *dev = common->dev;
	struct netdev_queue *netif_txq;
	unsigned int total_bytes = 0;
	struct net_device *ndev;
	struct sk_buff *skb;
	dma_addr_t desc_dma;
	int res, num_tx = 0;
	void **swdata;

	tx_chn = &virt_port->virt_port_tx_chan[chn];

	while (budget--) {
		struct cpsw_virt_port_ndev_priv *ndev_priv;
		struct cpsw_virt_port_ndev_stats *stats;

		res = k3_udma_glue_pop_tx_chn(tx_chn->tx_chan, &desc_dma);
		if (res == -ENODATA)
			break;

		if (desc_dma & 0x1) {
			if (atomic_dec_and_test(&virt_port->tdown_cnt))
				complete(&virt_port->tdown_complete);
			*tdown = true;
			break;
		}

		desc_tx = k3_cppi_desc_pool_dma2virt(tx_chn->desc_pool,
						     desc_dma);
		swdata = cppi5_hdesc_get_swdata(desc_tx);
		skb = *(swdata);
		cpsw_virt_port_xmit_free(tx_chn, dev, desc_tx);

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

	netif_txq = netdev_get_tx_queue(ndev, chn);

	netdev_tx_completed_queue(netif_txq, num_tx, total_bytes);
	dev_dbg(dev, "compl 0 %d Bytes\n", total_bytes);

	if (netif_tx_queue_stopped(netif_txq)) {
		/* Check whether the queue is stopped due to stalled tx dma,
		 * if the queue is stopped then wake the queue as
		 * we have free desc for tx
		 */
		__netif_tx_lock(netif_txq, smp_processor_id());
		if (netif_running(ndev) &&
		    (k3_cppi_desc_pool_avail(tx_chn->desc_pool) >=
		     MAX_SKB_FRAGS))
			netif_tx_wake_queue(netif_txq);

		__netif_tx_unlock(netif_txq);
	}
	dev_dbg(dev, "%s:%u pkt:%d\n", __func__, chn, num_tx);

	return num_tx;
}

static int cpsw_virt_port_tx_poll(struct napi_struct *napi_tx, int budget)
{
	struct cpsw_proxy_tx_chan *tx_chn = container_of(napi_tx, struct cpsw_proxy_tx_chan,
							 napi_tx);
	struct cpsw_virt_port *virt_port = tx_chn->virt_port;
	bool tdown = false;
	int num_tx;

	/* process every unprocessed channel */
	num_tx = cpsw_virt_port_tx_compl_packets(virt_port, tx_chn->rel_chan_idx, budget, &tdown);

	if (num_tx >= budget)
		return budget;

	if (napi_complete_done(napi_tx, num_tx)) {
		if (unlikely(tx_chn->tx_pace_timeout && !tdown)) {
			hrtimer_start(&tx_chn->tx_hrtimer,
				      ns_to_ktime(tx_chn->tx_pace_timeout),
				      HRTIMER_MODE_REL_PINNED);
		} else {
			enable_irq(tx_chn->irq);
		}
	}

	return 0;
}

/* RX psdata[2] word format - checksum information */
#define CPSW_RX_PSD_CSUM_ADD	GENMASK(15, 0)
#define CPSW_RX_PSD_CSUM_ERR	BIT(16)
#define CPSW_RX_PSD_IS_FRAGMENT	BIT(17)
#define CPSW_RX_PSD_IS_TCP		BIT(18)
#define CPSW_RX_PSD_IPV6_VALID	BIT(19)
#define CPSW_RX_PSD_IPV4_VALID	BIT(20)

static void cpsw_virt_port_rx_csum(struct sk_buff *skb, u32 csum_info)
{
	/* HW can verify IPv4/IPv6 TCP/UDP packets checksum
	 * csum information provides in psdata[2] word:
	 * CPSW_RX_PSD_CSUM_ERR bit - indicates csum error
	 * CPSW_RX_PSD_IPV6_VALID and CPSW_RX_PSD_IPV4_VALID
	 * bits - indicates IPv4/IPv6 packet
	 * CPSW_RX_PSD_IS_FRAGMENT bit - indicates fragmented packet
	 * CPSW_RX_PSD_CSUM_ADD has value 0xFFFF for non fragmented packets
	 * or csum value for fragmented packets if !CPSW_RX_PSD_CSUM_ERR
	 */
	skb_checksum_none_assert(skb);

	if (unlikely(!(skb->dev->features & NETIF_F_RXCSUM)))
		return;

	if ((csum_info & (CPSW_RX_PSD_IPV6_VALID |
			  CPSW_RX_PSD_IPV4_VALID)) &&
			  !(csum_info & CPSW_RX_PSD_CSUM_ERR)) {
		/* csum for fragmented packets is unsupported */
		if (!(csum_info & CPSW_RX_PSD_IS_FRAGMENT))
			skb->ip_summed = CHECKSUM_UNNECESSARY;
	}
}

static int cpsw_virt_port_rx_packets(struct cpsw_virt_port *virt_port,
				     u32 rel_chan_idx)
{
	struct cpsw_proxy_rx_chan *rx_chn = &virt_port->virt_port_rx_chan[rel_chan_idx];
	struct cpsw_proxy_common *common = virt_port->common;
	u32 buf_dma_len, pkt_len, port_id = 0, csum_info;
	struct cpsw_virt_port_ndev_priv *ndev_priv;
	struct cpsw_virt_port_ndev_stats *stats;
	struct cppi5_host_desc_t *desc_rx;
	struct device *dev = common->dev;
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
	ndev = virt_port->ndev;
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

	new_skb = netdev_alloc_skb_ip_align(ndev, CPSW_PROXY_CLIENT_MAX_PACKET_SIZE);
	if (new_skb) {
		skb_put(skb, pkt_len);
		skb->protocol = eth_type_trans(skb, ndev);
		cpsw_virt_port_rx_csum(skb, csum_info);
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

	ret = cpsw_virt_port_rx_push(virt_port, new_skb, rx_chn->rel_chan_idx);
	if (WARN_ON(ret < 0)) {
		dev_kfree_skb_any(new_skb);
		ndev->stats.rx_errors++;
		ndev->stats.rx_dropped++;
	}

	return ret;
}

static int cpsw_virt_port_rx_poll(struct napi_struct *napi_rx, int budget)
{
	struct cpsw_proxy_rx_chan *rx_chn = container_of(napi_rx, struct cpsw_proxy_rx_chan,
							 napi_rx);
	struct cpsw_virt_port *virt_port = rx_chn->virt_port;
	int num_rx = 0;
	int cur_budget;
	int ret;

	/* process every flow */
	cur_budget = budget;

	while (cur_budget--) {
		ret = cpsw_virt_port_rx_packets(virt_port, rx_chn->rel_chan_idx);
		if (ret)
			break;
		num_rx++;
	}

	if (num_rx < budget && napi_complete_done(napi_rx, num_rx)) {
		if (rx_chn->rx_irq_disabled) {
			rx_chn->rx_irq_disabled = false;
			if (unlikely(rx_chn->rx_pace_timeout)) {
				hrtimer_start(&rx_chn->rx_hrtimer,
					      ns_to_ktime(rx_chn->rx_pace_timeout),
					      HRTIMER_MODE_REL_PINNED);
			} else {
				enable_irq(rx_chn->irq);
			}
		}
	}

	return num_rx;
}

static void
cpsw_proxy_client_unreg_ndevs(struct cpsw_proxy_common *common)
{
	struct cpsw_virt_port *virt_port;
	int i;

	for (i = 0; i < common->num_virt_ports; i++) {
		virt_port = &common->virt_ports[i];
		if (virt_port->ndev)
			unregister_netdev(virt_port->ndev);
	}
}

static int
cpsw_proxy_client_init_ndev(struct cpsw_proxy_common *common, struct cpsw_virt_port *virt_port)
{
	struct cpsw_virt_port_ndev_priv *ndev_priv;
	struct cpsw_proxy_rx_chan *rx_chn;
	struct cpsw_proxy_tx_chan *tx_chn;
	struct device *dev = common->dev;
	int ret = 0, i;

	virt_port->ndev = devm_alloc_etherdev_mqs(dev, sizeof(struct cpsw_virt_port_ndev_priv),
						  virt_port->num_tx_chan, virt_port->num_rx_chan);

	if (!virt_port->ndev) {
		dev_err(dev, "error allocating net device for port: %u\n", virt_port->virt_port_id);
		return -ENOMEM;
	}

	ndev_priv = netdev_priv(virt_port->ndev);
	ndev_priv->virt_port = virt_port;
	SET_NETDEV_DEV(virt_port->ndev, dev);

	if (is_valid_ether_addr(virt_port->mac_addr))
		eth_hw_addr_set(virt_port->ndev, virt_port->mac_addr);

	virt_port->ndev->min_mtu = CPSW_PROXY_CLIENT_MIN_PACKET_SIZE;
	virt_port->ndev->max_mtu = CPSW_PROXY_CLIENT_MAX_PACKET_SIZE;
	virt_port->ndev->hw_features = NETIF_F_SG | NETIF_F_RXCSUM;
	virt_port->ndev->features = virt_port->ndev->hw_features;
	virt_port->ndev->vlan_features |=  NETIF_F_SG;
	virt_port->ndev->netdev_ops = &cpsw_virt_port_netdev_ops;
	virt_port->ndev->ethtool_ops = &cpsw_virt_port_ethtool_ops;

	ndev_priv->stats = netdev_alloc_pcpu_stats(struct cpsw_virt_port_ndev_stats);
	if (!ndev_priv->stats)
		return -ENOMEM;

	ret = devm_add_action_or_reset(dev, (void(*)(void *))free_percpu, ndev_priv->stats);
	if (ret) {
		dev_err(dev, "failed to add percpu stat free action %d", ret);
		return ret;
	}

	for (i = 0; i < virt_port->num_tx_chan; i++) {
		tx_chn = &virt_port->virt_port_tx_chan[i];
		netif_napi_add_tx(virt_port->ndev, &tx_chn->napi_tx, cpsw_virt_port_tx_poll);
		hrtimer_init(&tx_chn->tx_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL_PINNED);
		tx_chn->tx_hrtimer.function = &cpsw_virt_port_tx_timer_callback;
	}

	for (i = 0; i < virt_port->num_rx_chan; i++) {
		rx_chn = &virt_port->virt_port_rx_chan[i];
		netif_napi_add(virt_port->ndev, &rx_chn->napi_rx, cpsw_virt_port_rx_poll);
		hrtimer_init(&rx_chn->rx_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL_PINNED);
		rx_chn->rx_hrtimer.function = &cpsw_virt_port_rx_timer_callback;
	}

	ret = register_netdev(virt_port->ndev);
	if (ret)
		dev_err(dev, "error registering net device %d\n", ret);

	return ret;
}

static int cpsw_proxy_client_init_ndevs(struct cpsw_proxy_common *common)
{
	struct cpsw_virt_port *virt_port;
	char work_queue_name[IFNAMSIZ];
	int ret, i;

	for (i = 0; i < common->num_virt_ports; i++) {
		virt_port = &common->virt_ports[i];
		snprintf(work_queue_name, sizeof(work_queue_name), "virt_port_%d",
			 virt_port->virt_port_id);
		__hw_addr_init(&virt_port->mcast_list);
		INIT_WORK(&virt_port->rx_mode_work, cpsw_virt_port_set_rx_mode_work);
		virt_port->virt_port_wq = create_singlethread_workqueue(work_queue_name);
		if (!virt_port->virt_port_wq) {
			dev_err(common->dev, "failed to create workqueue %s\n", work_queue_name);
			return -ENOMEM;
		}
	}

	for (i = 0; i < common->num_virt_ports; i++) {
		virt_port = &common->virt_ports[i];
		ret = cpsw_proxy_client_init_ndev(common, virt_port);
		if (ret) {
			dev_err(common->dev, "failed to initialize netdevice: %d\n", ret);
			goto err;
		}
	}

	return 0;

err:
	cpsw_proxy_client_unreg_ndevs(common);
	return ret;
}

static void cpsw_proxy_client_destroy_wq(struct cpsw_proxy_common *common)
{
	struct cpsw_virt_port *virt_port;
	int i;

	for (i = 0; i < common->num_virt_ports; i++) {
		virt_port = &common->virt_ports[i];
		destroy_workqueue(virt_port->virt_port_wq);
	}
}

static irqreturn_t cpsw_virt_port_tx_irq(int irq, void *dev_id)
{
	struct cpsw_proxy_tx_chan *tx_chn = dev_id;

	disable_irq_nosync(irq);
	napi_schedule(&tx_chn->napi_tx);

	return IRQ_HANDLED;
}

static irqreturn_t cpsw_virt_port_rx_irq(int irq, void *dev_id)
{
	struct cpsw_proxy_rx_chan *rx_chn = dev_id;

	rx_chn->rx_irq_disabled = true;
	disable_irq_nosync(irq);
	napi_schedule(&rx_chn->napi_rx);

	return IRQ_HANDLED;
}

static int cpsw_proxy_client_setup_irq(struct cpsw_proxy_common *common)
{
	struct cpsw_proxy_rx_chan *rx_chn;
	struct cpsw_proxy_tx_chan *tx_chn;
	struct device *dev = common->dev;
	struct cpsw_virt_port *virt_port;
	int ret = 0, i, j;

	for (i = 0; i < common->num_virt_ports; i++) {
		virt_port = &common->virt_ports[i];

		for (j = 0; j < virt_port->num_tx_chan; j++) {
			tx_chn = &virt_port->virt_port_tx_chan[j];

			ret = devm_request_irq(dev, tx_chn->irq, cpsw_virt_port_tx_irq,
					       IRQF_TRIGGER_HIGH, tx_chn->tx_chan_name,
					       tx_chn);
			if (ret) {
				dev_err(dev, "failure requesting tx irq: %u err: %d\n",
					tx_chn->irq, ret);
				goto err;
			}
		}

		for (j = 0; j < virt_port->num_rx_chan; j++) {
			rx_chn = &virt_port->virt_port_rx_chan[j];
			ret = devm_request_irq(dev, rx_chn->irq, cpsw_virt_port_rx_irq,
					       IRQF_TRIGGER_HIGH, rx_chn->rx_chan_name,
					       rx_chn);
			if (ret) {
				dev_err(dev, "failure requesting rx irq: %u err: %d\n",
					rx_chn->irq, ret);
				goto err;
			}
		}
	}

err:
	return ret;
}

static void cpsw_proxy_client_detach(struct cpsw_proxy_common *common)
{
	struct cpsw_virt_port *virt_port;
	struct message response;
	u32 port_id;
	int ret, i, j;

	for (i = 0; i < common->num_virt_ports; i++) {
		virt_port = &common->virt_ports[i];
		port_id = virt_port->virt_port_id;

		/* Free MAC Request */
		if (virt_port->mac_is_valid) {
			ret = cpsw_proxy_client_send_request(common, virt_port,
							     virt_port->virt_port_token,
							     ETHREMOTECFG_FREE_MAC, &response);
			if (ret) {
				dev_err(common->dev, "failed to detach port %u err: %d\n",
					port_id, ret);
				return;
			}
		}

		/* Free TX DMA Channel */
		for (j = 0; j < virt_port->num_tx_chan; j++) {
			if (&virt_port->virt_port_tx_chan &&
			    &virt_port->virt_port_tx_chan->is_valid) {
				virt_port->curr_tx_chan_idx = j;
				ret = cpsw_proxy_client_send_request(common, virt_port,
								     virt_port->virt_port_token,
								     ETHREMOTECFG_FREE_TX,
								     &response);
				if (ret) {
					dev_err(common->dev, "failed to detach port %u err: %d\n",
						port_id, ret);
					return;
				}
			}
		}

		/* Free RX DMA Flow */
		for (j = 0; j < virt_port->num_rx_chan; j++) {
			if (&virt_port->virt_port_rx_chan &&
			    &virt_port->virt_port_rx_chan->is_valid) {
				virt_port->curr_rx_chan_idx = j;
				ret = cpsw_proxy_client_send_request(common, virt_port,
								     virt_port->virt_port_token,
								     ETHREMOTECFG_FREE_RX,
								     &response);
				if (ret) {
					dev_err(common->dev, "failed to detach port %u err: %d\n",
						port_id, ret);
					return;
				}
			}
		}

		/* Send Detach Request */
		ret = cpsw_proxy_client_send_request(common, virt_port,
						     virt_port->virt_port_token,
						     ETHREMOTECFG_DETACH, &response);
		if (ret) {
			dev_err(common->dev, "failed to detach port %u err: %d\n", port_id, ret);
			return;
		}
	}
}

static int cpsw_proxy_client_attach(struct cpsw_proxy_common *common)
{
	struct cpsw_proxy_rx_chan *rx_chn;
	struct cpsw_proxy_tx_chan *tx_chn;
	struct attach_response *att_resp;
	struct cpsw_virt_port *virt_port;
	struct message response;
	int ret, ret1, i, j;
	u32 port_id;

	for (i = 0; i < common->num_virt_ports; i++) {
		virt_port = &common->virt_ports[i];
		port_id = virt_port->virt_port_id;

		/* Send Attach Request */
		ret = cpsw_proxy_client_send_request(common, virt_port, ETHREMOTECFG_TOKEN_NONE,
						     ETHREMOTECFG_ATTACH, &response);
		if (ret) {
			dev_err(common->dev, "failed to attach port %u err: %d\n", port_id, ret);
			goto err;
		}

		att_resp = (struct attach_response *)&response;
		virt_port->virt_port_token = att_resp->response_msg_hdr.msg_hdr.token;
		virt_port->mcast_filter = att_resp->features & ETHREMOTECFG_FEATURE_MC_FILTER;
		virt_port->num_tx_chan = att_resp->num_tx_chan;
		virt_port->num_rx_chan = att_resp->num_rx_flow;

		for (j = 0; j < virt_port->num_tx_chan; j++) {
			tx_chn = &virt_port->virt_port_tx_chan[j];
			tx_chn->virt_port = virt_port;
			tx_chn->rel_chan_idx = j;
		}

		for (j = 0; j < virt_port->num_rx_chan; j++) {
			rx_chn = &virt_port->virt_port_rx_chan[j];
			rx_chn->virt_port = virt_port;
			rx_chn->rel_chan_idx = j;
		}
	}

	return 0;

err:
	while (i--) {
		virt_port = &common->virt_ports[i];
		port_id = virt_port->virt_port_id;

		/* Send Detach Request */
		ret1 = cpsw_proxy_client_send_request(common, virt_port,
						      virt_port->virt_port_token,
						      ETHREMOTECFG_DETACH, &response);
		if (ret1)
			dev_err(common->dev, "failed to DETACH port: %u err: %d\n", port_id, ret1);
	}

	return ret;
}

static int cpsw_proxy_client_dma_mac_alloc(struct cpsw_proxy_common *common)
{
	struct rx_flow_alloc_response *rx_alloc_resp;
	struct tx_psil_alloc_response *tx_alloc_resp;
	struct mac_alloc_response *mac_alloc_resp;
	struct cpsw_proxy_tx_chan *tx_chn;
	struct cpsw_proxy_rx_chan *rx_chn;
	struct cpsw_virt_port *virt_port;
	struct message response;
	int ret, i, j;
	u32 port_id;

	for (i = 0; i < common->num_virt_ports; i++) {
		virt_port = &common->virt_ports[i];
		port_id = virt_port->virt_port_id;

		for (j = 0; j < virt_port->num_rx_chan; j++) {
			/* Send RX Chan Allocation Request */
			virt_port->curr_rx_chan_idx = j;
			rx_chn = &virt_port->virt_port_rx_chan[j];

			ret = cpsw_proxy_client_send_request(common, virt_port,
							     virt_port->virt_port_token,
							     ETHREMOTECFG_ALLOC_RX, &response);
			if (ret) {
				dev_err(common->dev, "failed to alloc RX for port %u err: %d\n",
					port_id, ret);
				goto err;
			}

			rx_alloc_resp = (struct rx_flow_alloc_response *)&response;
			rx_chn->rx_flow_idx_base = rx_alloc_resp->rx_flow_idx_base;
			rx_chn->rx_flow_idx_offset = rx_alloc_resp->rx_flow_idx_offset;
			rx_chn->rx_psil_src_id = rx_alloc_resp->rx_psil_src_id;
			rx_chn->is_valid = 1;
		}

		for (j = 0; j < virt_port->num_tx_chan; j++) {
			/* Send TX Channel Allocation Request */
			virt_port->curr_tx_chan_idx = j;
			tx_chn = &virt_port->virt_port_tx_chan[j];

			ret = cpsw_proxy_client_send_request(common, virt_port,
							     virt_port->virt_port_token,
							     ETHREMOTECFG_ALLOC_TX, &response);
			if (ret) {
				dev_err(common->dev, "failed to alloc TX for port %u err: %d\n",
					port_id, ret);
				goto err;
			}

			tx_alloc_resp = (struct tx_psil_alloc_response *)&response;
			tx_chn->tx_psil_dest_id = tx_alloc_resp->tx_psil_dest_id;
			tx_chn->is_valid = 1;
		}

		/* Send MAC Allocation Request */
		ret = cpsw_proxy_client_send_request(common, virt_port, virt_port->virt_port_token,
						     ETHREMOTECFG_ALLOC_MAC, &response);
		if (ret) {
			dev_err(common->dev, "failed to alloc MAC for port %u err: %d\n", port_id,
				ret);
			goto err;
		}

		mac_alloc_resp = (struct mac_alloc_response *)&response;
		ether_addr_copy(virt_port->mac_addr, mac_alloc_resp->mac_addr);
		virt_port->mac_is_valid = 1;
	}

	return 0;

err:
	cpsw_proxy_client_detach(common);
	return ret;
}

static int cpsw_proxy_client_virt_port_init(struct cpsw_proxy_common *common)
{
	struct virt_port_alloc_response *vpa_resp;
	struct cpsw_virt_port *virt_port;
	unsigned int virt_port_idx, idx;
	struct message response;
	int ret, i;

	/* Send Virtual Port Allocation Request
	 * Since virt_port will be allocated after this request, it is not
	 * going to be used in sending the request. Therefore use NULL instead.
	 */
	ret = cpsw_proxy_client_send_request(common, NULL, ETHREMOTECFG_TOKEN_NONE,
					     ETHREMOTECFG_VIRT_PORT_ALLOC, &response);
	if (ret) {
		dev_err(common->dev, "failed to request virtual port details err: %d\n", ret);
		return ret;
	}

	vpa_resp = (struct virt_port_alloc_response *)&response;
	common->virt_switch_ports = vpa_resp->switch_port_mask;
	common->virt_mac_only_ports = vpa_resp->mac_port_mask;
	common->num_switch_ports = hweight32(common->virt_switch_ports);
	common->num_virt_ports += common->num_switch_ports;
	common->num_mac_only_ports = hweight32(common->virt_mac_only_ports);
	common->num_virt_ports += common->num_mac_only_ports;
	common->virt_ports = devm_kcalloc(common->dev, common->num_virt_ports,
					  sizeof(*common->virt_ports), GFP_KERNEL);

	/* Populate details in the virtual port structures */
	virt_port_idx = 0;
	idx = 0;
	for (i = 0; i < common->num_switch_ports; i++) {
		virt_port = &common->virt_ports[virt_port_idx];
		virt_port->common = common;
		virt_port->virt_port_type = VIRT_SWITCH_PORT;
		virt_port->virt_port_id = fns(common->virt_switch_ports, idx);
		mutex_init(&virt_port->mcast_filter_mutex);

		virt_port_idx++;
		idx++;
	}

	idx = 0;
	for (i = 0; i < common->num_mac_only_ports; i++) {
		virt_port = &common->virt_ports[virt_port_idx];
		virt_port->common = common;
		virt_port->virt_port_type = VIRT_MAC_ONLY_PORT;
		virt_port->virt_port_id = fns(common->virt_mac_only_ports, idx);
		mutex_init(&virt_port->mcast_filter_mutex);

		virt_port_idx++;
		idx++;
	}

	return 0;
}

static void cpsw_virt_port_free_tx_chns(void *data)
{
	struct cpsw_proxy_common *common = data;
	struct cpsw_proxy_tx_chan *tx_chn;
	struct cpsw_virt_port *virt_port;
	int i, j;

	for (i = 0; i < common->num_virt_ports; i++) {
		virt_port = &common->virt_ports[i];
		for (j = 0; j < virt_port->num_tx_chan; j++) {
			tx_chn = &virt_port->virt_port_tx_chan[j];

			if (!IS_ERR_OR_NULL(tx_chn->desc_pool))
				k3_cppi_desc_pool_destroy(tx_chn->desc_pool);

			if (!IS_ERR_OR_NULL(tx_chn->tx_chan))
				k3_udma_glue_release_tx_chn(tx_chn->tx_chan);

			memset(tx_chn, 0, sizeof(*tx_chn));
		}
	}
}

static int cpsw_proxy_client_init_tx_chans(struct cpsw_proxy_common *common)
{
	u32 max_desc_num = ALIGN(CPSW_PROXY_CLIENT_MAX_TX_DESC, MAX_SKB_FRAGS);
	struct k3_udma_glue_tx_channel_cfg tx_cfg = { 0 };
	struct cpsw_proxy_tx_chan *tx_chn;
	struct device *dev = common->dev;
	struct cpsw_virt_port *virt_port;
	struct k3_ring_cfg ring_cfg = {
		.elm_size = K3_RINGACC_RING_ELSIZE_8,
		.mode = K3_RINGACC_RING_MODE_RING,
		.flags = 0
	};
	char tx_chn_name[IFNAMSIZ];
	u32 hdesc_size, tx_chn_num;
	int ret = 0, ret1, i, j;

	for (i = 0; i < common->num_virt_ports; i++) {
		virt_port = &common->virt_ports[i];

		for (j = 0; j < virt_port->num_tx_chan; j++) {
			tx_chn = &virt_port->virt_port_tx_chan[j];

			tx_chn_num = common->num_active_tx_channels++;
			snprintf(tx_chn_name, sizeof(tx_chn_name), "tx%u-virt-port-%u",
				 tx_chn_num, virt_port->virt_port_id);
			strcpy(&tx_chn->tx_chan_name[0], tx_chn_name);

			init_completion(&virt_port->tdown_complete);

			hdesc_size = cppi5_hdesc_calc_size(true, CPSW_PROXY_CLIENT_NAV_PS_DATA_SIZE,
							   CPSW_PROXY_CLIENT_NAV_SW_DATA_SIZE);

			tx_cfg.swdata_size = CPSW_PROXY_CLIENT_NAV_SW_DATA_SIZE;
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
				dev_err(dev, "Failed to create tx pool %d\n", ret);
				goto err;
			}

			tx_chn->tx_chan = k3_udma_glue_request_tx_chn_by_id(dev, &tx_cfg,
									    common->dma_node,
									    tx_chn->tx_psil_dest_id);
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
	ret1 = devm_add_action(dev, cpsw_virt_port_free_tx_chns, common);
	if (ret1) {
		dev_err(dev, "failed to add free_tx_chns action %d", ret1);
		return ret1;
	}

	return ret;
}

static void cpsw_proxy_client_free_rx_chns(void *data)
{
	struct cpsw_proxy_common *common = data;
	struct cpsw_proxy_rx_chan *rx_chn;
	struct cpsw_virt_port *virt_port;
	int i, j;

	for (i = 0; i < common->num_virt_ports; i++) {
		virt_port = &common->virt_ports[i];

		for (j = 0; j < virt_port->num_rx_chan; j++) {
			rx_chn = &virt_port->virt_port_rx_chan[j];

			if (!IS_ERR_OR_NULL(rx_chn->desc_pool))
				k3_cppi_desc_pool_destroy(rx_chn->desc_pool);

			if (!IS_ERR_OR_NULL(rx_chn->rx_chan))
				k3_udma_glue_release_rx_chn(rx_chn->rx_chan);
		}
	}
}

static int cpsw_proxy_client_init_rx_chans(struct cpsw_proxy_common *common)
{
	u32  max_desc_num = CPSW_PROXY_CLIENT_MAX_RX_DESC;
	struct k3_udma_glue_rx_channel_cfg rx_cfg = {0};
	struct cpsw_proxy_rx_chan *rx_chn;
	struct device *dev = common->dev;
	struct cpsw_virt_port *virt_port;
	char rx_chn_name[IFNAMSIZ];
	u32 hdesc_size, rx_chn_num;
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
	int ret = 0, ret1, i, j;

	hdesc_size = cppi5_hdesc_calc_size(true, CPSW_PROXY_CLIENT_NAV_PS_DATA_SIZE,
					   CPSW_PROXY_CLIENT_NAV_SW_DATA_SIZE);

	rx_cfg.swdata_size = CPSW_PROXY_CLIENT_NAV_SW_DATA_SIZE;
	rx_cfg.flow_id_num = CPSW_PROXY_CLIENT_MAX_RX_FLOWS;
	rx_cfg.remote = true;

	for (i = 0; i < common->num_virt_ports; i++) {
		virt_port = &common->virt_ports[i];

		for (j = 0; j < virt_port->num_rx_chan; j++) {
			rx_chn = &virt_port->virt_port_rx_chan[j];

			rx_chn_num = common->num_active_rx_channels++;
			snprintf(rx_chn_name, sizeof(rx_chn_name), "rx%u-virt-port-%u", rx_chn_num,
				 virt_port->virt_port_id);
			strcpy(&rx_chn->rx_chan_name[0], rx_chn_name);

			rx_cfg.flow_id_base = rx_chn->rx_flow_idx_base + rx_chn->rx_flow_idx_offset;

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
				k3_udma_glue_request_remote_rx_chn_by_id(dev, common->dma_node,
									 &rx_cfg,
									 rx_chn->rx_psil_src_id);
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
	ret1 = devm_add_action(dev, cpsw_proxy_client_free_rx_chns, common);
	if (ret1) {
		dev_err(dev, "failed to add free_rx_chns action %d", ret1);
		return ret1;
	}

	return ret;
}

static int cpsw_proxy_client_register_ipv4(struct cpsw_virt_port *virt_port)
{
	struct cpsw_proxy_common *common = virt_port->common;
	struct message response;
	int ret;

	ret = cpsw_proxy_client_send_request(common, virt_port, virt_port->virt_port_token,
					     ETHREMOTECFG_IPv4_REGISTER, &response);
	if (ret) {
		dev_err(common->dev, "failed to register IPv4 Address err: %d\n", ret);
		return ret;
	}

	return 0;
}

static int cpsw_proxy_client_deregister_ipv4(struct cpsw_virt_port *virt_port)
{
	struct cpsw_proxy_common *common = virt_port->common;
	struct message response;
	int ret;

	ret = cpsw_proxy_client_send_request(common, virt_port, virt_port->virt_port_token,
					     ETHREMOTECFG_IPv4_DEREGISTER, &response);
	if (ret) {
		dev_err(common->dev, "failed to deregister IPv4 Address err: %d\n", ret);
		return ret;
	}

	return 0;
}

static bool cpsw_proxy_client_check(const struct net_device *ndev)
{
	struct cpsw_virt_port *virt_port = cpsw_virt_port_ndev_to_virt_port(ndev);

	return ndev->netdev_ops == &cpsw_virt_port_netdev_ops &&
				   virt_port->virt_port_type == VIRT_SWITCH_PORT;
}

static int cpsw_proxy_client_inetaddr_event(struct notifier_block *unused,
					    unsigned long event, void *ptr)
{
	struct in_ifaddr *ifa = (struct in_ifaddr *)ptr;
	struct cpsw_virt_port *virt_port;
	struct cpsw_proxy_common *common;
	struct net_device *ndev;
	int ret = 0;

	ndev = ifa->ifa_dev ? ifa->ifa_dev->dev : NULL;
	if (!ndev)
		return NOTIFY_DONE;

	if (!cpsw_proxy_client_check(ndev))
		return NOTIFY_DONE;

	virt_port = cpsw_virt_port_ndev_to_virt_port(ndev);
	common = virt_port->common;
	memcpy(virt_port->ipv4_addr, &ifa->ifa_address, ETHREMOTECFG_IPV4ADDRLEN);
	switch (event) {
	case NETDEV_UP:
	case NETDEV_CHANGEADDR:
		ret = cpsw_proxy_client_register_ipv4(virt_port);
		if (ret)
			dev_err(common->dev, "IPV4 register failed: %d\n", ret);
		break;

	case NETDEV_DOWN:
	case NETDEV_PRE_CHANGEADDR:
		ret = cpsw_proxy_client_deregister_ipv4(virt_port);
		if (ret)
			dev_err(common->dev, "IPV4 register failed: %d\n", ret);
		break;
	}

	return notifier_from_errno(ret);
}

static void cpsw_proxy_client_unregister_notifier(struct cpsw_proxy_common *common)
{
	struct cpsw_virt_port *virt_port;
	int i;

	for (i = 0; i < common->num_virt_ports; i++) {
		virt_port = &common->virt_ports[i];

		if (virt_port->virt_port_type == VIRT_SWITCH_PORT)
			unregister_inetaddr_notifier(&virt_port->virt_port_inetaddr_nb);
	}
}

static void cpsw_proxy_client_register_notifier(struct cpsw_proxy_common *common)
{
	struct cpsw_virt_port *virt_port;
	int i;

	for (i = 0; i < common->num_virt_ports; i++) {
		virt_port = &common->virt_ports[i];

		if (virt_port->virt_port_type == VIRT_SWITCH_PORT) {
			virt_port->virt_port_inetaddr_nb.notifier_call =
								&cpsw_proxy_client_inetaddr_event;
			register_inetaddr_notifier(&virt_port->virt_port_inetaddr_nb);
		}
	}
}

static void cpsw_proxy_client_show_info(struct cpsw_proxy_common *common)
{
	struct device *dev = common->dev;
	struct cpsw_virt_port *virt_port;
	int i;

	dev_info(dev, "%u Virtual Switch Port(s), %u Virtual MAC Only Port(s)\n",
		 common->num_switch_ports, common->num_mac_only_ports);

	for (i = 0; i < common->num_virt_ports; i++) {
		virt_port = &common->virt_ports[i];

		if (virt_port->virt_port_type == VIRT_SWITCH_PORT)
			dev_info(dev, "Virt Port: %u, Type: Switch Port, Iface: %s, Num TX: %u, Num RX: %u, Token: %u\n",
				 virt_port->virt_port_id, virt_port->ndev->name,
				 virt_port->num_tx_chan, virt_port->num_rx_chan,
				 virt_port->virt_port_token);
		else
			dev_info(dev, "Virt Port: %u, Type: MAC Port, Iface: %s, Num TX: %u, Num RX: %u, Token: %u\n",
				 virt_port->virt_port_id, virt_port->ndev->name,
				 virt_port->num_tx_chan, virt_port->num_rx_chan,
				 virt_port->virt_port_token);
	}
}

static int cpsw_proxy_client_probe(struct rpmsg_device *rpdev)
{
	struct device *dev = &rpdev->dev;
	struct cpsw_proxy_common *common;
	int ret = 0;

	common = devm_kzalloc(dev, sizeof(struct cpsw_proxy_common), GFP_KERNEL);
	if (!common)
		return -ENOMEM;

	common->rpdev = rpdev;
	common->dev = dev;
	mutex_init(&common->request_id_msg_mutex);
	common->dma_compatible = (const char *)rpdev->id.driver_data;
	common->dma_node = of_find_compatible_node(NULL, NULL, common->dma_compatible);
	if (!common->dma_node) {
		dev_err(dev, "failed to get DMA Node\n");
		return -ENODEV;
	}

	dev_set_drvdata(dev, common);

	init_completion(&common->wait_for_response);

	/* Initialize Virtual Ports */
	ret = cpsw_proxy_client_virt_port_init(common);
	if (ret)
		goto err;

	/* Request DMA Channels and MAC allocation Info for each Virtual Port */
	ret = cpsw_proxy_client_attach(common);
	if (ret)
		goto err;

	/* Request DMA Channels and MAC Address for each Virtual Port */
	ret = cpsw_proxy_client_dma_mac_alloc(common);
	if (ret)
		goto err;

	/* Set DMA Mask */
	ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(48));
	if (ret) {
		dev_err(dev, "error setting dma mask: %d\n", ret);
		goto err;
	}

	/* Initialize TX DMA Channels for each Virtual Port */
	ret = cpsw_proxy_client_init_tx_chans(common);
	if (ret)
		return ret;

	/* Initialize RX DMA Flows for each Virtual Port */
	ret = cpsw_proxy_client_init_rx_chans(common);
	if (ret)
		return ret;

	/* Create and initialize Net Device for each Virtual Port */
	ret = cpsw_proxy_client_init_ndevs(common);
	if (ret)
		return ret;

	/* Request Interrupts for TX and RX DMA Channels */
	ret = cpsw_proxy_client_setup_irq(common);
	if (ret)
		goto err;

	/* Register INETADDR events notifier for Virtual Switch Ports */
	cpsw_proxy_client_register_notifier(common);

	cpsw_proxy_client_show_info(common);

	return 0;

err:
	cpsw_proxy_client_unreg_ndevs(common);
	return ret;
}

static void cpsw_proxy_client_remove(struct rpmsg_device *rpdev)
{
	struct cpsw_proxy_common *common = dev_get_drvdata(&rpdev->dev);

	cpsw_proxy_client_unregister_notifier(common);
	cpsw_proxy_client_unreg_ndevs(common);
	cpsw_proxy_client_destroy_wq(common);
	cpsw_proxy_client_detach(common);
}

static struct rpmsg_device_id cpsw_proxy_client_id_table[] = {
	{
		.name = "ti.ethfw.ethdevice",
		.driver_data = (kernel_ulong_t)"ti,j721e-navss-main-udmap"
	},
	{ },
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
MODULE_DESCRIPTION("CPSW Virtual Client Driver");
MODULE_AUTHOR("Siddharth Vadapalli <s-vadapalli@ti.com>");
