/* SPDX-License-Identifier: GPL-2.0-only or MIT */
/* Texas Instruments Ethernet Switch Firmware (EthFw) ABIs
 *
 * Copyright (C) 2024 Texas Instruments Incorporated - https://www.ti.com/
 *
 */

#ifndef __ETHFW_ABI_H_
#define __ETHFW_ABI_H_

/* Name of the RPMsg Endpoint announced by EthFw on the RPMsg-Bus */
#define ETHFW_SERVICE_EP_NAME	"ti.ethfw.ethdevice"

/* Response status set by EthFw on the Success of a Request */
#define ETHFW_RES_OK			(0)

/* Response status set by EthFw when an operation is in progress */
#define ETHFW_RES_SINPROGRESS		(1)

/* Response status set by EthFw on the Failure of a Request */
#define ETHFW_RES_EFAIL			(-1)

/* Response status set by EthFw when Client passes wrong arguments */
#define ETHFW_RES_EBADARGS		(-2)

/* Default VLAN ID for a Virtual Port */
#define ETHFW_DFLT_VLAN			0xFFFF

/* EthFw TX Checksum Offload Capability */
#define ETHFW_TX_CSUM_OFFLOAD		BIT(0)

/* EthFw Multicast Filtering Capability */
#define ETHFW_MCAST_FILTERING		BIT(1)

/* Token corresponding to the Linux CPSW Proxy Client assigned by EthFw */
#define ETHFW_LINUX_CLIENT_TOKEN	3

/* Default token used by Virtual Port to communicate with EthFw */
#define ETHFW_TOKEN_NONE		0xFFFFFFFF

/* MAC Address length in octets */
#define ETHFW_MACADDRLEN		6

/* IPV4 Address length in octets */
#define ETHFW_IPV4ADDRLEN		4

/* Types of request messages sent to EthFw from CPSW Proxy Client */
enum request_msg_type {
	/* Request details of Virtual Ports allocated to the Client.
	 * Two types of Virtual Ports exist:
	 * 1. MAC Only Port:
	 *    The Physical MAC Port corresponding to this type of Virtual
	 *    Port does not belong to the group of MAC Ports which Switch
	 *    traffic among themselves. The Physical MAC Port is dedicated
	 *    solely to the Client which has been allocated this type of
	 *    Virtual Port.
	 *
	 * 2. Switch Port:
	 *    The Physical MAC Port corresponding to this type of Virtual
	 *    Port belongs to the group of MAC Ports which Switch traffic
	 *    among themselves. The Physical MAC Port is shared with other
	 *    Clients in terms of the traffic that is sent out of or received
	 *    on this port.
	 *
	 * EthFw responds to this request by providing a bitmask of the
	 * Virtual Port IDs for each type of Virtual Port allocated to
	 * the Client.
	 */
	ETHFW_VIRT_PORT_INFO,

	/* Request usage of a Virtual Port that has been allocated to the
	 * Client.
	 *
	 * EthFw responds with details of the supported MTU size, the number
	 * of TX DMA Channels and the number of RX DMA Flows for the specified
	 * Virtual Port.
	 */
	ETHFW_VIRT_PORT_ATTACH,

	ETHFW_UNUSED_REQUEST_1,

	/* Request disuse of a Virtual Port that was in use prior to the
	 * generation of this request.
	 */
	ETHFW_VIRT_PORT_DETACH,

	/* Request to get link status */
	ETHFW_VIRT_PORT_LINK_STATUS,

	/* Request for allocation of a TX DMA Channel for a Virtual Port.
	 * Client can request as many TX DMA Channels as have been allocated
	 * by EthFw for the specified Virtual Port.
	 *
	 * EthFw responds with the TX PSI-L Thread ID corresponding to
	 * the TX DMA Channel for the Virtual Port to transmit traffic
	 * to CPSW.
	 */
	ETHFW_ALLOC_TX,

	/* Request for allocation of an RX DMA Flow for a Virtual Port.
	 * Client can request as many RX DMA Flows as have been allocated
	 * by EthFw for the specified Virtual Port.
	 *
	 * EthFw responds with the RX PSI-L Thread ID, the base of the RX
	 * Flow index and the offset from the base of the allocated RX Flow
	 * index. The RX Flow/Channel is used to receive traffic from CPSW.
	 */
	ETHFW_ALLOC_RX,

	/* Request for allocation of the MAC Address for a Virtual Port.
	 *
	 * EthFw responds with the MAC Address corresponding to the
	 * specified Virtual Port.
	 */
	ETHFW_ALLOC_MAC,

	/* Request for release of a TX DMA Channel that had been allocated
	 * to the specified Virtual Port.
	 */
	ETHFW_FREE_TX,

	/* Request for release of an RX DMA Flow that had been allocated to
	 * the specified Virtual Port.
	 */
	ETHFW_FREE_RX,

	/* Request for release of the MAC Address that had been allocated to
	 * the specified Virtual Port.
	 */
	ETHFW_FREE_MAC,

	/* Request for usage of the specified MAC Address for the traffic
	 * sent or received on the Virtual Port for which the MAC Address
	 * has been allocated.
	 */
	ETHFW_MAC_REGISTER,

	/* Request for disuse of the specified MAC Address for the traffic
	 * sent or received on the Virtual Port for which the MAC Address
	 * had been allocated.
	 */
	ETHFW_MAC_DEREGISTER,

	/* Request for setting the default RX DMA Flow for a Virtual Port. */
	ETHFW_SET_DEFAULT_RX_FLOW,

	/* Request for deleting the default RX DMA Flow for a Virtual Port. */
	ETHFW_DEL_DEFAULT_RX_FLOW,

	/* Request for registering the IPv4 Address of the Network Interface
	 * in Linux corresponding to the specified Virtual Port.
	 */
	ETHFW_IPv4_REGISTER,

	/* Request for deregistering the IPv4 Address of the Network Interface
	 * in Linux corresponding to the specified Virtual Port that had been
	 * registered prior to this request.
	 */
	ETHFW_IPv4_DEREGISTER,

	/* Request for joining a VLAN */
	ETHFW_JOIN_VLAN,

	/* Request for leaving a VLAN */
	ETHFW_LEAVE_VLAN,

	/* Request for joining a Multicast Address group */
	ETHFW_MCAST_FILTER_ADD,

	/* Request for leaving a Multicast Address group */
	ETHFW_MCAST_FILTER_DEL,

	ETHFW_UNUSED_REQUEST_2,
	ETHFW_UNUSED_REQUEST_3,
	ETHFW_UNUSED_REQUEST_4,
	ETHFW_UNUSED_REQUEST_5,
	ETHFW_UNUSED_REQUEST_6,

	/* Request for notifying teardown completion status */
	ETHFW_TEARDOWN_COMPLETE,
};

enum notify_msg_type {
	ETHFW_NOTIFYCLIENT_FWINFO,
	ETHFW_NOTIFYCLIENT_HWPUSH,
	ETHFW_NOTIFYCLIENT_HWERROR,
	ETHFW_NOTIFYCLIENT_RECOVERED,
	ETHFW_NOTIFYCLIENT_CUSTOM,
	ETHFW_NOTIFYCLIENT_LAST,
};

enum ethfw_status {
	ETHFW_UNINIT,
	ETHFW_READY,
	ETHFW_RECOVERY,
	ETHFW_BAD,
};

enum message_type {
	ETHFW_MSG_REQUEST,
	ETHFW_MSG_NOTIFY,
	ETHFW_MSG_RESPONSE,
};

struct message_header {
	u32 token;
	u32 client_id;
	u32 msg_type;
} __packed;

struct message {
	struct message_header msg_hdr;
	u32 message_data[120];
} __packed;

struct request_message_header {
	struct message_header msg_hdr;
	u32 request_type;
	u32 request_id;
} __packed;

struct response_message_header {
	struct message_header msg_hdr;
	u32 response_type; /* Same as request_type */
	u32 response_id;
	int response_status;
} __packed;

struct notify_message_header {
	struct message_header msg_hdr;
	u32 notify_type;
} __packed;

struct common_response_message {
	struct response_message_header response_msg_hdr;
} __packed;

struct common_request_message {
	struct request_message_header request_msg_hdr;
} __packed;

struct common_notify_message {
	struct notify_message_header notify_msg_hdr;
} __packed;

struct virt_port_info_response {
	struct response_message_header response_msg_hdr;
	/* Port mask denoting absolute virtual switch ports allocated */
	u32 switch_port_mask;
	/* Port mask denoting absolute virtual MAC ports allocated */
	u32 mac_port_mask;
} __packed;

struct attach_request {
	struct request_message_header request_msg_hdr;
	/* Virtual port which needs core attach */
	u32 virt_port;
} __packed;

struct attach_response {
	struct response_message_header response_msg_hdr;
	/* MTU of RX packet */
	u32 rx_mtu;
	/* MTU of TX packet */
	u32 tx_mtu;
	/* Feature bitmask */
	u32 features;
	/* Number of TX DMA Channels available for the virtual port */
	u32 num_tx_chan;
	/* Number of RX DMA Flows available for the virtual port */
	u32 num_rx_flow;
} __packed;

struct rx_flow_alloc_request {
	struct request_message_header request_msg_hdr;
	/* Relative index of RX flow among available num_rx_flow flows */
	u32 rx_flow_idx;
} __packed;

struct rx_flow_alloc_response {
	struct response_message_header response_msg_hdr;
	/* Allocated RX flow index base */
	u32 rx_flow_idx_base;
	/* Allocated flow index offset */
	u32 rx_flow_idx_offset;
	/* RX PSIL Peer source thread id */
	u32 rx_psil_src_id;
} __packed;

struct tx_thread_alloc_request {
	struct request_message_header request_msg_hdr;
	/* Relative index of TX channel among available num_tx_chan channels */
	u32 tx_chan_idx;
} __packed;

struct tx_thread_alloc_response {
	struct response_message_header response_msg_hdr;
	/* TX PSIL peer destination thread id which should be paired with the TX UDMA channel */
	u32 tx_psil_dest_id;
} __packed;

struct mac_alloc_response {
	struct response_message_header response_msg_hdr;
	/* Allocated MAC address */
	u8 mac_addr[ETHFW_MACADDRLEN];
} __packed;

struct rx_flow_release_request {
	struct request_message_header request_msg_hdr;
	/* RX flow index base */
	u32 rx_flow_idx_base;
	/* RX flow index offset */
	u32 rx_flow_idx_offset;
} __packed;

struct tx_thread_release_request {
	struct request_message_header request_msg_hdr;
	/* TX PSIL Peer destination thread id to be freed */
	u32 tx_psil_dest_id;
} __packed;

struct mac_release_request {
	struct request_message_header request_msg_hdr;
	/* MAC address to be freed */
	u8 mac_addr[ETHFW_MACADDRLEN];
} __packed;

struct mac_register_deregister_request {
	struct request_message_header request_msg_hdr;
	/* MAC address which needs to be registered/deregistered */
	u8 mac_addr[ETHFW_MACADDRLEN];
	/* RX flow index Base */
	u32 rx_flow_idx_base;
	/* RX flow index offset */
	u32 rx_flow_idx_offset;
} __packed;

struct ipv4_register_request {
	struct request_message_header request_msg_hdr;
	/* IPv4 Address */
	u8 ipv4_addr[ETHFW_IPV4ADDRLEN];
	/* MAC address associated with the IP address which should be added to
	 * the ARP table
	 */
	u8 mac_addr[ETHFW_MACADDRLEN];
} __packed;

struct ipv4_deregister_request {
	struct request_message_header request_msg_hdr;
	/* IPv4 Address */
	u8 ipv4_addr[ETHFW_IPV4ADDRLEN];
} __packed;

struct default_rx_flow_register_request {
	struct request_message_header request_msg_hdr;
	/* RX flow index Base */
	u32 rx_flow_idx_base;
	/* RX flow index offset */
	u32 rx_flow_idx_offset;
} __packed;

struct port_link_status_response {
	struct response_message_header response_msg_hdr;
	/* Link status of the port */
	bool link_up;
	/* Link speed */
	u32 speed;
	/* Duplex mode */
	u32 duplex;
} __packed;

struct add_multicast_request {
	struct request_message_header request_msg_hdr;
	/* Multicast MAC address to be added */
	u8 mac_addr[ETHFW_MACADDRLEN];
	/* VLAN id */
	u16 vlan_id;
	/* RX flow index from which the MAC_address association will be added.
	 * It's applicable only for _exclusive multicast traffic_
	 */
	u32 rx_flow_idx_base;
	/* RX flow index offset */
	u32 rx_flow_idx_offset;
} __packed;

struct del_multicast_request {
	struct request_message_header request_msg_hdr;
	/* Multicast MAC address to be added */
	u8 mac_addr[ETHFW_MACADDRLEN];
	/* VLAN id */
	u16 vlan_id;
} __packed;

#endif /* __ETHFW_ABI_H_ */
