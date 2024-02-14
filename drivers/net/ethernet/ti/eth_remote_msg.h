/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Texas Instruments Ethernet Switch Firmware ABIs
 *
 * Copyright (C) 2023 Texas Instruments Incorporated - https://www.ti.com/
 *
 */

#ifndef __ETH_REMOTE_MSG_H__
#define __ETH_REMOTE_MSG_H__

/* Ethernet Switch Firmware Service Endpoint name */
#define ETHREMOTECFG_FRAMEWORK_SERVICE_NAME   "ti.ethfw.ethdevice"

/* Ethernet Switch Firmware response code : Success */
#define ETHREMOTECFG_CMDSTATUS_OK		0

/* Ethernet Switch Firmware response code : Try again */
#define ETHREMOTECFG_CMDSTATUS_EAGAIN		-1

/* Maximum length of message data */
#define ETHREMOTECFG_MESSAGE_DATA_LEN		486

/* Number of priorities supported by CPSW */
#define ETHREMOTECFG_PRIORITY_NUM		8

/* MAC Address length in octets */
#define ETHREMOTECFG_MACADDRLEN		6

/* IPV4 Address length in octets */
#define ETHREMOTECFG_IPV4ADDRLEN		4

/* Request Ethernet Switch Firmware to use the default VLAN id for the virtual
 * port; i.e. Virtual MAC Port or Virtual Switch Port.
 */
#define ETHREMOTECFG_ETHSWITCH_VLAN_USE_DFLT	0xFFFF

#define ETHREMOTECFG_FWDATE_YEARLEN		4
#define ETHREMOTECFG_FWDATE_MONTHLEN		3
#define ETHREMOTECFG_FWDATE_DATELEN		2
#define ETHREMOTECFG_FW_COMMITSHALEN		8

/*  Ethernet Switch Firmware Version Information
 *
 *  API version info for the Ethernet device RPC
 *  Any remote core client should check API version is compatible with this
 *  API version
 */
/* Ethernet Switch Firmware API version major version */
#define ETHREMOTECFG_FW_ETHSWITCH_VERSION_MAJOR		0
/* Ethernet Switch Firmware API version minor version */
#define ETHREMOTECFG_FW_ETHSWITCH_VERSION_MINOR		4
/* Ethernet Switch Firmware API version minor revision */
#define ETHREMOTECFG_FW_ETHSWITCH_VERSION_REVISION		0

/* TX checksum offload feature support */
#define ETHREMOTECFG_FEATURE_TXCSUM			BIT(0)

/* Note: Feature bit 1 and 2 are intentionally not used */

/* Multicast filter feature support */
#define ETHREMOTECFG_FEATURE_MC_FILTER			BIT(3)

#define ETHREMOTECFG_TOKEN_NONE                0xFFFF

#define ETHREMOTECFG_IOCTL_INARGS_LEN		480
#define ETHREMOTECFG_IOCTL_OUTARGS_LEN		480

/*!
 * \brief Ethernet RPC messages (C2S)
 */
enum request_msg_type {
	ETHREMOTECFG_VIRT_PORT_ALLOC,
	ETHREMOTECFG_ATTACH,
	ETHREMOTECFG_ATTACH_EXT,
	ETHREMOTECFG_DETACH,
	ETHREMOTECFG_PORT_LINK_STATUS,
	ETHREMOTECFG_ALLOC_TX,
	ETHREMOTECFG_ALLOC_RX,
	ETHREMOTECFG_ALLOC_MAC,
	ETHREMOTECFG_FREE_TX,
	ETHREMOTECFG_FREE_RX,
	ETHREMOTECFG_FREE_MAC,
	ETHREMOTECFG_MAC_REGISTER,
	ETHREMOTECFG_MAC_DEREGISTER,
	ETHREMOTECFG_SET_RX_DEFAULTFLOW,
	ETHREMOTECFG_DEL_RX_DEFAULTFLOW,
	ETHREMOTECFG_IPv4_REGISTER,
	ETHREMOTECFG_IPv4_DEREGISTER,
	ETHREMOTECFG_CMD_JOIN_VLAN,
	ETHREMOTECFG_CMD_LEAVE_VLAN,
	ETHREMOTECFG_FILTER_MAC_ADD,
	ETHREMOTECFG_FILTER_MAC_DEL,
	ETHREMOTECFG_PROMISC_ENABLE,
	ETHREMOTECFG_PROMISC_DISABLE,
	ETHREMOTECFG_REGISTER_READ,
	ETHREMOTECFG_REGISTER_WRITE,
	ETHREMOTECFG_MATCH_ETHTYPE_REGISTER,
	ETHREMOTECFG_MATCH_ETHTYPE_DEREGISTER,
	ETHREMOTECFG_REMOTE_TIMER_REGISTER,
	ETHREMOTECFG_REMOTE_TIMER_DEREGISTER,
	ETHREMOTECFG_MESSAGE_PING,
	ETHREMOTECFG_ETHFW_STATUS,
	ETHREMOTECFG_TEARDOWN_COMPLETE,
	ETHREMOTECFG_IOCTL,
	ETHREMOTECFG_CLIENT_DUMPSTATS,
};

enum client_token {
	ETHREMOTECFG_NA_CLIENT,
	ETHREMOTECFG_AUTOSAR_CLIENT,
	ETHREMOTECFG_RTOS_CLIENT,
	ETHREMOTECFG_LINUX_CLIENT,
	ETHREMOTECFG_QNX_CLIENT,
};

enum notify_msg_type {
	ETHREMOTECFG_NOTIFYCLIENT_FWINFO,
	ETHREMOTECFG_NOTIFYCLIENT_HWPUSH,
	ETHREMOTECFG_NOTIFYTYPE_HWERROR,
	ETHREMOTECFG_NOTIFYTYPE_HWRECOVERY_COMPLETE,
	ETHREMOTECFG_NOTIFYCLIENT_CUSTOM,
	ETHREMOTECFG_NOTIFYCLIENT_LAST,
};

enum ethfw_status {
	ETHREMOTECFG_ETHFW_INIT,
	ETHREMOTECFG_ETHFW_RECOVERY,
	ETHRC_ETHFW_DEINIT,
};

enum message_type {
	ETHREMOTECFG_MSG_REQUEST,
	ETHREMOTECFG_MSG_NOTIFY,
	ETHREMOTECFG_MSG_RESPONSE,
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

struct ethfw_rpc_version_info {
	u32 major;
	u32 minor;
	u32 rev;
	char year[ETHREMOTECFG_FWDATE_YEARLEN];
	char month[ETHREMOTECFG_FWDATE_MONTHLEN];
	char date[ETHREMOTECFG_FWDATE_DATELEN];
	char commit_hash[ETHREMOTECFG_FW_COMMITSHALEN];
} __packed;

struct ethfw_firmware_version_notify {
	struct notify_message_header notify_msg_hdr;
	struct ethfw_rpc_version_info ethfw_info;
	u64 permission_flags;
	u32 uart_connected;
	u32 uart_id;
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

struct virt_port_alloc_response {
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
	/* MTU of TX packet per priority */
	u32 tx_mtu[ETHREMOTECFG_PRIORITY_NUM];
	/* Feature bitmask based on defines ETHREMOTECFG_FEATURE_xxx */
	u32 features;
	/* Number of TX DMA Channels available for the virtual port */
	u32 num_tx_chan;
	/* Number of RX DMA Flows available for the virtual port */
	u32 num_rx_flow;
} __packed;

struct attach_ext_response {
	struct response_message_header response_msg_hdr;
	/* MTU of RX packet */
	u32 rx_mtu;
	/* MTU of TX packet per priority */
	u32 tx_mtu[ETHREMOTECFG_PRIORITY_NUM];
	/* Allocated RX flow index base */
	u32 rx_flow_idx_base;
	/* Allocated flow index offset */
	u32 rx_flow_idx_offset;
	/* TX PSIL Peer destination thread id which should be paired with the TX UDMA channel */
	u32 tx_psil_dest_id;
	/* MAC address allocated */
	u8 mac_addr[ETHREMOTECFG_MACADDRLEN];
	/* Feature bitmask based on defines ETHREMOTECFG_FEATURE_xxx */
	u32 features;
	/* RX PSIL Peer source thread id */
	u32 rx_psil_src_id;
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

struct tx_psil_alloc_request {
	struct request_message_header request_msg_hdr;
	/* Relative index of TX channel among available num_tx_chan channels */
	u32 tx_chan_idx;
} __packed;

struct tx_psil_alloc_response {
	struct response_message_header response_msg_hdr;
	/* TX PSIL peer destination thread id which should be paired with the TX UDMA channel */
	u32 tx_psil_dest_id;
} __packed;

struct mac_alloc_response {
	struct response_message_header response_msg_hdr;
	/* Allocated MAC address */
	u8 mac_addr[ETHREMOTECFG_MACADDRLEN];
} __packed;

struct rx_flow_release_request {
	struct request_message_header request_msg_hdr;
	/* RX flow index base */
	u32 rx_flow_idx_base;
	/* RX flow index offset */
	u32 rx_flow_idx_offset;
} __packed;

struct tx_psil_release_request {
	struct request_message_header request_msg_hdr;
	/* TX PSIL Peer destination thread id to be freed */
	u32 tx_psil_dest_id;
} __packed;

struct mac_release_request {
	struct request_message_header request_msg_hdr;
	/* MAC address to be freed */
	u8 mac_addr[ETHREMOTECFG_MACADDRLEN];
} __packed;

struct mac_rx_flow_register_request {
	struct request_message_header request_msg_hdr;
	/* MAC address which needs to be registered */
	u8 mac_addr[ETHREMOTECFG_MACADDRLEN];
	/* RX flow index Base */
	u32 rx_flow_idx_base;
	/* RX flow index offset to which the MAC address needs to be registered */
	u32 rx_flow_idx_offset;
} __packed;

struct ipv4_register_request {
	struct request_message_header request_msg_hdr;
	/* IPv4 Address */
	u8 ipv4_addr[ETHREMOTECFG_IPV4ADDRLEN];
	/* MAC address associated with the IP address which should be added to
	 * the ARP table
	 */
	u8 mac_addr[ETHREMOTECFG_MACADDRLEN];
} __packed;

struct ipv4_deregister_request {
	struct request_message_header request_msg_hdr;
	/* IPv4 Address */
	u8 ipv4_addr[ETHREMOTECFG_IPV4ADDRLEN];
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

struct add_ether_type_rx_flow_request {
	struct request_message_header request_msg_hdr;
	/* EtherType to be associated with the flow */
	u16 ether_type;
	/* Allocated flow's base */
	u32 rx_flow_idx_base;
	/* Allocated flow's offset */
	u32 rx_flow_idx_offset;
} __packed;

struct del_ether_type_rx_flow_request {
	struct request_message_header request_msg_hdr;
	/* EtherType associated with the flow */
	u16 ether_type;
} __packed;

struct add_mcast_vlan_rx_flow_request {
	struct request_message_header request_msg_hdr;
	/* Multicast MAC address to be added */
	u8 mac_addr[ETHREMOTECFG_MACADDRLEN];
	/* VLAN id */
	u16 vlan_id;
	/* RX flow index from which the MAC_address association will be added.
	 * It's applicable only for _exclusive multicast traffic_
	 */
	u32 rx_flow_idx_base;
	/* RX flow index offset */
	u32 rx_flow_idx_offset;
} __packed;

struct del_mcast_vlan_rx_flow_request {
	struct request_message_header request_msg_hdr;
	/* Multicast MAC address to be added */
	u8 mac_addr[ETHREMOTECFG_MACADDRLEN];
	/* VLAN id */
	u16 vlan_id;
} __packed;

struct ping_request {
	struct request_message_header request_msg_hdr;
	/* Data which will be responded back by the server */
	u32 ping_request_data[ETHREMOTECFG_MESSAGE_DATA_LEN];
} __packed;

struct ping_response {
	struct response_message_header response_msg_hdr;
	/* Data which will be responded back by the server */
	u32 ping_response_data[ETHREMOTECFG_MESSAGE_DATA_LEN];
} __packed;

struct remote_timer_register_request {
	struct request_message_header request_msg_hdr;
	/* Hardware Push Number to be used for timesync router configuration */
	u8 hw_push_number;
	/* Timer Id to be used for timesync router configuration */
	u8 timer_id;
} __packed;

struct remote_timer_deregister_request {
	struct request_message_header request_msg_hdr;
	/* Hardware Push Number to be used for timesync router configuration */
	u8 hw_push_number;
} __packed;

struct ioctl_request {
	struct request_message_header request_msg_hdr;
	/* IOCTL Command ID */
	u32 cmd;
	/* IOCTL input arguments length */
	u32 in_args_len;
	/* IOCTL input arguments */
	u64 in_args[(ETHREMOTECFG_IOCTL_INARGS_LEN / sizeof(u64))];
	/* IOTCL output arguments length */
	u32 out_args_len;
};

struct ioctl_response {
	struct request_message_header request_msg_hdr;
	/* IOCTL Command ID */
	u32 cmd;
	/* IOCTL output arguments length */
	u32 out_args_len;
	/* IOCTL output arguments */
	u64 out_args[(ETHREMOTECFG_IOCTL_OUTARGS_LEN / sizeof(u64))];
};

struct register_write_request {
	struct request_message_header request_msg_hdr;
	/* Register address */
	u32 register_address;
	/* Value which needs to be written */
	u32 value;
} __packed;

struct register_read_request {
	/* Request message common header */
	struct request_message_header request_msg_hdr;
	/* Register address */
	u32 register_address;
} __packed;

struct register_read_response {
	struct response_message_header response_msg_hdr;
	/* Value which has been read */
	u32 value;
} __packed;

struct server_status_response {
	struct response_message_header response_msg_hdr;
	/* Server status */
	u32 status;
};

struct hw_msg_push_notify_message {
	struct notify_message_header notify_msg_hdr;
	/* CPTS hardware push number */
	u32 cpts_hw_push_number;
	/* CPTS hardware push event timestamp  */
	u64 timestamp;
} __packed;

#endif /* __ETH_REMOTE_MSG_H__ */
