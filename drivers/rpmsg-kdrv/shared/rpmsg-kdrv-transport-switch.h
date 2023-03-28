/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Grygorii Strashko <grygorii.strashko@ti.com>
 */

#ifndef DRIVERS_RPMSG_KDRV_SHARED_RPMSG_KDRV_TRANSPORT_SWITCH_H_
#define DRIVERS_RPMSG_KDRV_SHARED_RPMSG_KDRV_TRANSPORT_SWITCH_H_

#include <linux/etherdevice.h>
#include "rpmsg-kdrv-transport-common.h"

#define RPMSG_KDRV_TP_ETHSWITCH_VERSION_MAJOR             (0)
#define RPMSG_KDRV_TP_ETHSWITCH_VERSION_MINOR             (1)
#define RPMSG_KDRV_TP_ETHSWITCH_VERSION_REVISION          (1)

/**
 * enum rpmsg_kdrv_ethswitch_message_type - Eth switch rpmsg protocol messages
 */
enum rpmsg_kdrv_ethswitch_message_type {
	RPMSG_KDRV_TP_ETHSWITCH_ATTACH = 0x00,
	RPMSG_KDRV_TP_ETHSWITCH_ATTACH_EXT = 0x01,
	RPMSG_KDRV_TP_ETHSWITCH_ALLOC_TX = 0x02,
	RPMSG_KDRV_TP_ETHSWITCH_ALLOC_RX = 0x03,
	RPMSG_KDRV_TP_ETHSWITCH_REGISTER_DEFAULTFLOW = 0x04,
	RPMSG_KDRV_TP_ETHSWITCH_ALLOC_MAC = 0x05,
	RPMSG_KDRV_TP_ETHSWITCH_REGISTER_MAC = 0x06,
	RPMSG_KDRV_TP_ETHSWITCH_UNREGISTER_MAC = 0x07,
	RPMSG_KDRV_TP_ETHSWITCH_UNREGISTER_DEFAULTFLOW = 0x08,
	RPMSG_KDRV_TP_ETHSWITCH_FREE_MAC = 0x09,
	RPMSG_KDRV_TP_ETHSWITCH_FREE_TX = 0x0A,
	RPMSG_KDRV_TP_ETHSWITCH_FREE_RX = 0x0B,
	RPMSG_KDRV_TP_ETHSWITCH_DETACH = 0x0C,
	RPMSG_KDRV_TP_ETHSWITCH_IOCTL = 0x0D,
	RPMSG_KDRV_TP_ETHSWITCH_REGWR = 0x0E,
	RPMSG_KDRV_TP_ETHSWITCH_REGRD = 0x0F,
	RPMSG_KDRV_TP_ETHSWITCH_IPV4_MAC_REGISTER = 0x10,
	RPMSG_KDRV_TP_ETHSWITCH_IPV6_MAC_REGISTER = 0x11,
	RPMSG_KDRV_TP_ETHSWITCH_IPV4_MAC_UNREGISTER = 0x12,
	RPMSG_KDRV_TP_ETHSWITCH_IPV6_MAC_UNREGISTER = 0x13,
	RPMSG_KDRV_TP_ETHSWITCH_PING_REQUEST = 0x14,
	RPMSG_KDRV_TP_ETHSWITCH_S2C_NOTIFY = 0x15,
	RPMSG_KDRV_TP_ETHSWITCH_C2S_NOTIFY = 0x16,
	RPMSG_KDRV_TP_ETHSWITCH_REGISTER_ETHTYPE = 0x17,
	RPMSG_KDRV_TP_ETHSWITCH_UNREGISTER_ETHTYPE = 0x18,
	RPMSG_KDRV_TP_ETHSWITCH_REGISTER_REMOTETIMER = 0x19,
	RPMSG_KDRV_TP_ETHSWITCH_UNREGISTER_REMOTETIMER = 0x1A,
	RPMSG_KDRV_TP_ETHSWITCH_SET_PROMISC_MODE = 0x1B,
	RPMSG_KDRV_TP_ETHSWITCH_FILTER_ADD_MAC = 0x1C,
	RPMSG_KDRV_TP_ETHSWITCH_FILTER_DEL_MAC = 0x1D,
	RPMSG_KDRV_TP_ETHSWITCH_MAX = 0x1E,
};

/**
 * Client to Eth switch notification events @RPMSG_KDRV_TP_ETHSWITCH_C2S_NOTIFY
 */
enum rpmsg_kdrv_ethswitch_c2s_notify_type {
	RPMSG_KDRV_TP_ETHSWITCH_CLIENTNOTIFY_DUMPSTATS = 0x00,
	RPMSG_KDRV_TP_ETHSWITCH_CLIENTNOTIFY_MAX,
};

/**
 * Eth switch HW ID
 */
enum rpmsg_kdrv_ethswitch_cpsw_type {
	RPMSG_KDRV_TP_ETHSWITCH_CPSWTYPE_MCU_CPSW,
	RPMSG_KDRV_TP_ETHSWITCH_CPSWTYPE_MAIN_CPSW,
	RPMSG_KDRV_TP_ETHSWITCH_CPSWTYPE_MAX,
};

/**
 * Response status codes returned by Eth switch FW in
 * struct @rpmsg_kdrv_ethswitch_common_resp_info
 */
#define RPMSG_KDRV_TP_ETHSWITCH_CMDSTATUS_OK		(0)
#define RPMSG_KDRV_TP_ETHSWITCH_CMDSTATUS_EAGAIN	(-1)
#define RPMSG_KDRV_TP_ETHSWITCH_CMDSTATUS_EFAIL		(-2)
#define RPMSG_KDRV_TP_ETHSWITCH_CMDSTATUS_EACCESS	(-3)

/* Maximum length of message data */
#define RPMSG_KDRV_TP_ETHSWITCH_MESSAGE_DATA_LEN	(128)

/* Number of priorities supported by CPSW */
#define RPMSG_KDRV_TP_ETHSWITCH_PRIORITY_NUM   (8)

/* IPv4 Address length in octets */
#define RPMSG_KDRV_TP_ETHSWITCH_IPV4ADDRLEN         (4)

/**
 * struct rpmsg_kdrv_ethswitch_msg_header - Message Header for outgoing messages
 *
 * @message_type: Type of messages: One of
 *	enum @rpmsg_kdrv_ethswitch_message_type values
 */
struct rpmsg_kdrv_ethswitch_msg_header {
	u8 message_type;
} __packed;

/**
 * struct rpmsg_kdrv_ethswitch_common_req_info - common request msgs data
 *
 * @id: unique handle
 * @core_key: core specific key to indicate attached core
 *
 * Common structure used for all Eth switch FW request msgs except
 * @RPMSG_KDRV_TP_ETHSWITCH_ATTACH. It has to be filled with values returned
 * by @RPMSG_KDRV_TP_ETHSWITCH_ATTACH.
 */
struct rpmsg_kdrv_ethswitch_common_req_info {
	u64 id;
	u32 core_key;
} __packed;

/**
 * struct rpmsg_kdrv_ethswitch_common_resp_info - common response data
 *
 * @status: status of request
 *
 * Common data returned by Eth switch FW in all response messages to identify
 * status of request message processing.
 */
struct rpmsg_kdrv_ethswitch_common_resp_info {
	s32 status;
} __packed;

/**
 * struct rpmsg_kdrv_ethswitch_attach_req - attach cmd client request msg
 *
 * @header: msg header
 * @cpsw_type: CPSW HW type enum @rpmsg_kdrv_ethswitch_cpsw_type
 *
 * Client attach message @RPMSG_KDRV_TP_ETHSWITCH_ATTACH. it should be always
 * sent first before other requests to Eth switch FW.
 */
struct rpmsg_kdrv_ethswitch_attach_req {
	struct rpmsg_kdrv_ethswitch_msg_header header;
	u8 cpsw_type;
} __packed;

/**
 * struct rpmsg_kdrv_ethswitch_attach_resp - attach client response msg
 *
 * @info: common response data. Status of the request processing
 * @id: unique handle used by all further CMDs
 * @core_key: core specific key to indicate attached core
 * @rx_mtu: MTU of rx packets
 * @tx_mtu: MTU of tx packet per priority
 * @features: supported features mask
 * @mac_only_port: 1-relative MAC port number for ports in MAC-only mode, 0
 *                 for switch ports.
 *
 * Attach client response msg received as response to client attach request
 * @RPMSG_KDRV_TP_ETHSWITCH_ATTACH. The @id and @core_key should be used to
 * fill struct @rpmsg_kdrv_ethswitch_common_req_info in all further request
 * messages.
 * The @info.status field is @RPMSG_KDRV_TP_ETHSWITCH_CMDSTATUS_OK on success.
 */
struct rpmsg_kdrv_ethswitch_attach_resp {
	struct rpmsg_kdrv_ethswitch_common_resp_info info;
	u64 id;
	u32 core_key;
	u32 rx_mtu;
	u32 tx_mtu[RPMSG_KDRV_TP_ETHSWITCH_PRIORITY_NUM];
	u32 features;
#define RPMSG_KDRV_TP_ETHSWITCH_FEATURE_TXCSUM BIT(0)
#define RPMSG_KDRV_ETHSWITCH_FEATURE_MAC_ONLY BIT(2)
#define RPMSG_KDRV_ETHSWITCH_FEATURE_MC_FILTER BIT(3)
	u32 mac_only_port;
} __packed;

/**
 * struct rpmsg_kdrv_ethswitch_attach_extended_req - extended attach request msg
 *
 * @header: msg header
 * @cpsw_type: CPSW HW type enum @rpmsg_kdrv_ethswitch_cpsw_type
 *
 * Client extended attach request @RPMSG_KDRV_TP_ETHSWITCH_ATTACH_EXT. It can
 * be used instead of @RPMSG_KDRV_TP_ETHSWITCH_ATTACH and has to sent first
 * before other requests to Eth switch FW.
 */
struct rpmsg_kdrv_ethswitch_attach_extended_req {
	struct rpmsg_kdrv_ethswitch_msg_header header;
	u8 cpsw_type;
} __packed;

/**
 * struct rpmsg_kdrv_ethswitch_common_response_info - extended attach resp msg
 *
 * @info: common response data. Status of the request processing
 * @id: unique handle used by all further CMDs
 * @core_key: core specific key to indicate attached core
 * @rx_mtu: MTU of rx packets
 * @tx_mtu: MTU of tx packet per priority
 * @features: supported features mask
 * @alloc_flow_idx: RX UDMA flow ID
 * @tx_cpsw_psil_dst_id: PSI-L dest thread id
 * @mac_address: default eth MAC address assigned to this client
 * @mac_only_port: 1-relative MAC port number for ports in MAC-only mode, 0
 *                 for switch ports.
 *
 * Extended attach response msg received as response to client extended attach
 * request @RPMSG_KDRV_TP_ETHSWITCH_ATTACH_EXT. The @id and @core_key should be
 * used to fill struct @rpmsg_kdrv_ethswitch_common_req_info in all further
 * request messages. In addition, it provides allocated DMA resources and
 * MAC address.
 *
 * The @info.status field is @RPMSG_KDRV_TP_ETHSWITCH_CMDSTATUS_OK on success.
 */
struct rpmsg_kdrv_ethswitch_attach_extended_resp {
	struct rpmsg_kdrv_ethswitch_common_resp_info info;
	u64 id;
	u32 core_key;
	u32 rx_mtu;
	u32 tx_mtu[RPMSG_KDRV_TP_ETHSWITCH_PRIORITY_NUM];
	u32 features;
	u32 alloc_flow_idx;
	u32 tx_cpsw_psil_dst_id;
	u8 mac_address[ETH_ALEN];
	u32 mac_only_port;
} __packed;

/**
 * struct rpmsg_kdrv_ethswitch_detach_req  - detach client request msg
 *
 * @header: msg header
 * @info: common request msgs data
 *
 * Client detach request message @RPMSG_KDRV_TP_ETHSWITCH_DETACH.
 * it should be always sent as the last message to Eth switch FW.
 */
struct rpmsg_kdrv_ethswitch_detach_req {
	struct rpmsg_kdrv_ethswitch_msg_header header;
	struct rpmsg_kdrv_ethswitch_common_req_info info;
} __packed;

/**
 * struct rpmsg_kdrv_ethswitch_detach_resp - detach client response msg
 *
 * @info: common response data. Status of the request processing
 *
 * Client detach response msg received as response to client detach request
 * @RPMSG_KDRV_TP_ETHSWITCH_DETACH.
 * The @info.status field is @RPMSG_KDRV_TP_ETHSWITCH_CMDSTATUS_OK on success.
 */
struct rpmsg_kdrv_ethswitch_detach_resp {
	struct rpmsg_kdrv_ethswitch_common_resp_info info;
} __packed;

/**
 * struct rpmsg_kdrv_ethswitch_alloc_req  - alloc resources request msg
 *
 * @header: msg header
 * @info: common request msgs data
 *
 * Client resources allocation request messages
 * @RPMSG_KDRV_TP_ETHSWITCH_ALLOC_RX: get RX DMA resources
 * @RPMSG_KDRV_TP_ETHSWITCH_ALLOC_TX: get TX DMA resources
 * @RPMSG_KDRV_TP_ETHSWITCH_ALLOC_MAC: get MAC address
 */
struct rpmsg_kdrv_ethswitch_alloc_req {
	struct rpmsg_kdrv_ethswitch_msg_header header;
	struct rpmsg_kdrv_ethswitch_common_req_info info;
} __packed;

/**
 * struct rpmsg_kdrv_ethswitch_alloc_rx_resp - alloc rx resources response msg
 *
 * @info: common response data. Status of the request processing
 * @alloc_flow_idx: RX UDMA flow ID
 *
 * Client alloc rx resources response msg received as response to request
 * @RPMSG_KDRV_TP_ETHSWITCH_ALLOC_RX. The @alloc_flow_idx is RX UDMA flow ID
 * to be used for ingress packets reception.
 * The @info.status field is @RPMSG_KDRV_TP_ETHSWITCH_CMDSTATUS_OK on success.
 */
struct rpmsg_kdrv_ethswitch_alloc_rx_resp {
	struct rpmsg_kdrv_ethswitch_common_resp_info info;
	u32 alloc_flow_idx;
} __packed;

/**
 * struct rpmsg_kdrv_ethswitch_alloc_tx_resp - alloc tx resources response msg
 *
 * @info: common response data. Status of the request processing
 * @tx_cpsw_psil_dst_id: PSI-L dest thread id
 *
 * Client alloc tx resources response msg received as response to request
 * @RPMSG_KDRV_TP_ETHSWITCH_ALLOC_TX. The @tx_cpsw_psil_dst_id is TX PSI-L dest
 * thread ID to be used for TX UDMA channel setup.
 * The @info.status field is @RPMSG_KDRV_TP_ETHSWITCH_CMDSTATUS_OK on success.
 */
struct rpmsg_kdrv_ethswitch_alloc_tx_resp {
	struct rpmsg_kdrv_ethswitch_common_resp_info info;
	u32 tx_cpsw_psil_dst_id;
} __packed;

/**
 * struct rpmsg_kdrv_ethswitch_alloc_mac_resp - alloc MAC resources response msg
 *
 * @info: common response data. Status of the request processing
 * @mac_address: default eth MAC address assigned to this client
 *
 * Client alloc MAC resources response msg received as response to request
 * @RPMSG_KDRV_TP_ETHSWITCH_ALLOC_MAC.
 * The @info.status field is @RPMSG_KDRV_TP_ETHSWITCH_CMDSTATUS_OK on success.
 */
struct rpmsg_kdrv_ethswitch_alloc_mac_resp {
	struct rpmsg_kdrv_ethswitch_common_resp_info info;
	u8 mac_address[ETH_ALEN];
} __packed;

/**
 * struct rpmsg_kdrv_ethswitch_register_mac_req - register MAC addr
 *
 * @header: msg header
 * @info: common request msgs data
 * @mac_address: eth MAC address used by client
 * @flow_idx: RX UDMA flow ID
 *
 * Client register MAC addr message @RPMSG_KDRV_TP_ETHSWITCH_REGISTER_MAC.
 * it should be sent to Eth switch FW to configure HW network traffic
 * classifiers so all network traffic directed to @mac_address will be
 * redirected to this client and can be received through allocated RX UDMA flow
 * @flow_idx.
 *
 * This message has to be sent by client when it's ready to receive network
 * traffic.
 */
struct rpmsg_kdrv_ethswitch_register_mac_req {
	struct rpmsg_kdrv_ethswitch_msg_header header;
	struct rpmsg_kdrv_ethswitch_common_req_info info;
	u8 mac_address[ETH_ALEN];
	u32 flow_idx;
} __packed;

/**
 * struct rpmsg_kdrv_ethswitch_register_mac_resp - register MAC addr response
 *
 * @info: common response data. Status of the request processing
 *
 * Client register MAC addr response msg received as response to
 * request @RPMSG_KDRV_TP_ETHSWITCH_REGISTER_MAC.
 * The @info.status field is @RPMSG_KDRV_TP_ETHSWITCH_CMDSTATUS_OK on success.
 */
struct rpmsg_kdrv_ethswitch_register_mac_resp {
	struct rpmsg_kdrv_ethswitch_common_resp_info info;
} __packed;

/**
 * struct rpmsg_kdrv_ethswitch_unregister_mac_req - unregister MAC addr
 *
 * @header: msg header
 * @info: common request msgs data
 * @mac_address: eth MAC address used by client
 * @flow_idx: RX UDMA flow ID
 *
 * Client unregister MAC addr message @RPMSG_KDRV_TP_ETHSWITCH_UNREGISTER_MAC.
 * it should be sent to Eth switch FW to disable HW network traffic
 * classifiers so all network traffic directed to @mac_address will be dropped.
 *
 * This message has to be sent by client when it does not want to receive any
 * more network traffic.
 */
struct rpmsg_kdrv_ethswitch_unregister_mac_req {
	struct rpmsg_kdrv_ethswitch_msg_header header;
	struct rpmsg_kdrv_ethswitch_common_req_info info;
	u8 mac_address[ETH_ALEN];
	u32 flow_idx;
} __packed;

/**
 * struct rpmsg_kdrv_ethswitch_unregister_mac_resp - unregister MAC addr resp
 *
 * @info: common response data. Status of the request processing
 *
 * Client unregister MAC addr response msg received as response to
 * request @RPMSG_KDRV_TP_ETHSWITCH_UNREGISTER_MAC.
 * The @info.status field is @RPMSG_KDRV_TP_ETHSWITCH_CMDSTATUS_OK on success.
 */
struct rpmsg_kdrv_ethswitch_unregister_mac_resp {
	struct rpmsg_kdrv_ethswitch_common_resp_info info;
} __packed;

/**
 * struct rpmsg_kdrv_ethswitch_ipv4_register_mac_req - register IPv4:MAC pair
 *
 * @header: msg header
 * @info: common request msgs data
 * @mac_address: eth MAC address used by client
 * @ipv4_addr: IPv4 addr
 *
 * Client register IPv4:MAC addr pair message
 * @RPMSG_KDRV_TP_ETHSWITCH_IPV4_MAC_REGISTER registers pair of IPv4 @ipv4_addr
 * and Eth MAC @mac_address addresses in Eth switch FW ARP database.
 *
 * This message has to be sent by client when there is new IPv4 addr assigned.
 */
struct rpmsg_kdrv_ethswitch_ipv4_register_mac_req {
	struct rpmsg_kdrv_ethswitch_msg_header header;
	struct rpmsg_kdrv_ethswitch_common_req_info info;
	u8 mac_address[ETH_ALEN];
	u8 ipv4_addr[RPMSG_KDRV_TP_ETHSWITCH_IPV4ADDRLEN];
} __packed;

/**
 * struct rpmsg_kdrv_ethswitch_ipv4_register_mac_resp - register IPv4:MAC pair
 *	response
 *
 * @info: common response data. Status of the request processing
 *
 * Client register IPv4:MAC addr pair response msg received as response to
 * request @RPMSG_KDRV_TP_ETHSWITCH_IPV4_MAC_REGISTER.
 * The @info.status field is @RPMSG_KDRV_TP_ETHSWITCH_CMDSTATUS_OK on success.
 */
struct rpmsg_kdrv_ethswitch_ipv4_register_mac_resp {
	struct rpmsg_kdrv_ethswitch_common_resp_info info;
} __packed;

/**
 * struct rpmsg_kdrv_ethswitch_ipv4_unregister_mac_req - unregister IPv4 addr
 *
 * @header: msg header
 * @info: common request msgs data
 * @ipv4_addr: IPv4 addr
 *
 * Client unregister IPv4 addr message
 * @RPMSG_KDRV_TP_ETHSWITCH_IPV4_MAC_UNREGISTER. It removes IPv4 @ipv4_addr
 * address from Eth switch FW ARP database.
 *
 * This message has to be sent by client when there is IPv4 addr unassigned.
 */
struct rpmsg_kdrv_ethswitch_ipv4_unregister_mac_req {
	struct rpmsg_kdrv_ethswitch_msg_header header;
	struct rpmsg_kdrv_ethswitch_common_req_info info;
	u8 ipv4_addr[RPMSG_KDRV_TP_ETHSWITCH_IPV4ADDRLEN];
} __packed;

/**
 * struct rpmsg_kdrv_ethswitch_ipv4_unregister_mac_resp - unregister IPv4 addr
 *
 * @info: common response data. Status of the request processing
 *
 * Client unregister IPv4 addr response msg received as response to
 * request @RPMSG_KDRV_TP_ETHSWITCH_IPV4_MAC_UNREGISTER.
 * The @info.status field is @RPMSG_KDRV_TP_ETHSWITCH_CMDSTATUS_OK on success.
 */
struct rpmsg_kdrv_ethswitch_ipv4_unregister_mac_resp {
	struct rpmsg_kdrv_ethswitch_common_resp_info info;
} __packed;

/**
 * struct rpmsg_kdrv_ethswitch_ping_req - ping request
 *
 * @header: msg header
 * @data: custom data
 *
 * Client ping request @RPMSG_KDRV_TP_ETHSWITCH_PING_REQUEST. The Eth switch FW
 * should return the same @data in struct @rpmsg_kdrv_ethswitch_ping_resp.
 * Can be used any time - no attach required.
 */
struct rpmsg_kdrv_ethswitch_ping_req {
	struct rpmsg_kdrv_ethswitch_msg_header header;
	u8 data[RPMSG_KDRV_TP_ETHSWITCH_MESSAGE_DATA_LEN];
} __packed;

/**
 * struct rpmsg_kdrv_ethswitch_ping_resp - ping response
 *
 * @data: custom data
 *
 * The ping response msg received as response to request
 * @RPMSG_KDRV_TP_ETHSWITCH_PING_REQUEST. The Eth switch FW should return
 * the same @data as was provided in struct @rpmsg_kdrv_ethswitch_ping_req.
 */
struct rpmsg_kdrv_ethswitch_ping_resp {
	u8 data[RPMSG_KDRV_TP_ETHSWITCH_MESSAGE_DATA_LEN];
} __packed;

/**
 * struct rpmsg_kdrv_ethswitch_regrd_req - read hw register request
 *
 * @header: msg header
 * @regaddr: phys register address
 *
 * The read hw register request @RPMSG_KDRV_TP_ETHSWITCH_REGRD.
 * The Eth switch FW should return the @regaddr register value.
 * Can be used any time - no attach required.
 */
struct rpmsg_kdrv_ethswitch_regrd_req {
	struct rpmsg_kdrv_ethswitch_msg_header header;
	u32 regaddr;
} __packed;

/**
 * struct rpmsg_kdrv_ethswitch_regrd_resp - read hw register response
 *
 * @info: common response data. Status of the request processing
 * @regval: register value
 *
 * The read hw register response received as response to request
 * @RPMSG_KDRV_TP_ETHSWITCH_REGRD. The @regval is hw register value from
 * @regaddr phys register address provided in
 * struct @rpmsg_kdrv_ethswitch_regrd_req
 *
 * The @info.status field is @RPMSG_KDRV_TP_ETHSWITCH_CMDSTATUS_OK on success.
 */
struct rpmsg_kdrv_ethswitch_regrd_resp {
	struct rpmsg_kdrv_ethswitch_common_resp_info info;
	u32 regval;
} __packed;

/**
 * struct rpmsg_kdrv_ethswitch_c2s_notify - notification request
 *
 * @header: msg header
 * @info: common request msg data
 * @notifyid: enum @rpmsg_kdrv_ethswitch_c2s_notify_type
 * @notify_info_len: length of @notify_info
 * @notify_info: notification message data
 *
 * The notification request message @RPMSG_KDRV_TP_ETHSWITCH_C2S_NOTIFY is one
 * way message to Eth switch FW without response.
 */
struct rpmsg_kdrv_ethswitch_c2s_notify {
	struct rpmsg_kdrv_ethswitch_msg_header header;
	struct rpmsg_kdrv_ethswitch_common_req_info info;
	u8 notifyid;
	u32 notify_info_len;
	u8 notify_info[RPMSG_KDRV_TP_ETHSWITCH_MESSAGE_DATA_LEN];
} __packed;

/**
 * struct rpmsg_kdrv_ethswitch_set_promisc_mode_req - set promiscuous mode
 *
 * @header: msg header
 * @info: common request msg data
 * @enable: promiscuous mode (enable or disable)
 *
 * Client message @RPMSG_KDRV_TP_ETHSWITCH_SET_PROMISC_MODE is sent to change
 * the promiscuous mode.
 */
struct rpmsg_kdrv_ethswitch_set_promisc_mode_req {
	struct rpmsg_kdrv_ethswitch_msg_header header;
	struct rpmsg_kdrv_ethswitch_common_req_info info;
	u32 enable;
} __packed;

/**
 * Set promiscuous mode response msg received as response to client's mode change
 * request @RPMSG_KDRV_TP_ETHSWITCH_SET_PROMISC_MODE.
 * The @info.status field is @RPMSG_KDRV_TP_ETHSWITCH_CMDSTATUS_OK on success.
 */
struct rpmsg_kdrv_ethswitch_set_promisc_mode_resp {
	struct rpmsg_kdrv_ethswitch_common_resp_info info;
} __packed;

/**
 * struct rpmsg_kdrv_ethswitch_filter_add_mc_req - add multicast MAC address to filter
 *
 * @header: msg header
 * @info: common request msg data
 * @mac_address: Multicast address to be added
 * @vlan_id: VLAN id
 * @flow_idx: RX UDMA flow ID (used for multicast addresses marked as 'exclusive' in
 *            switch firmware)
 *
 * Client message @RPMSG_KDRV_TP_ETHSWITCH_FILTER_ADD_MAC is sent to add a multicast
 * address to the receive filter.
 */
struct rpmsg_kdrv_ethswitch_filter_add_mc_req {
	struct rpmsg_kdrv_ethswitch_msg_header header;
	struct rpmsg_kdrv_ethswitch_common_req_info info;
	u8 mac_address[ETH_ALEN];
	u16 vlan_id;
	u32 flow_idx;
} __packed;

/**
 * Response msg received as response to client's request to add a multicas address to
 * receive filter via @RPMSG_KDRV_TP_ETHSWITCH_FILTER_ADD_MAC.
 * The @info.status field is @RPMSG_KDRV_TP_ETHSWITCH_CMDSTATUS_OK on success.
 */
struct rpmsg_kdrv_ethswitch_filter_add_mc_resp {
	struct rpmsg_kdrv_ethswitch_common_resp_info info;
} __packed;

/**
 * struct rpmsg_kdrv_ethswitch_filter_del_mac_req - delete multicast MAC address
 * from filter
 *
 * @header: msg header
 * @info: common request msg data
 * @mac_address: Multicast address to be removed
 * @vlan_id: VLAN id
 * @flow_idx: RX UDMA flow ID (used for multicast addresses marked as 'exclusive' in
 *            switch firmware)
 *
 * Client message @RPMSG_KDRV_TP_ETHSWITCH_FILTER_DEL_MAC is sent to delete a multicast
 * address from the receive filter.
 */
struct rpmsg_kdrv_ethswitch_filter_del_mc_req {
	struct rpmsg_kdrv_ethswitch_msg_header header;
	struct rpmsg_kdrv_ethswitch_common_req_info info;
	u8 mac_address[ETH_ALEN];
	u16 vlan_id;
	u32 flow_idx;
} __packed;

/**
 * Response msg received as response to client's request to delete a multicas address to
 * receive filter via @RPMSG_KDRV_TP_ETHSWITCH_FILTER_DEL_MAC.
 * The @info.status field is @RPMSG_KDRV_TP_ETHSWITCH_CMDSTATUS_OK on success.
 */
struct rpmsg_kdrv_ethswitch_filter_del_mc_resp {
	struct rpmsg_kdrv_ethswitch_common_resp_info info;
} __packed;

/**
 * struct rpmsg_kdrv_ethswitch_fw_version_info - fw version info
 *
 * @major: major
 * @minor: minor
 * @rev: revision
 * @year:
 * @month:
 * @date: build date
 * @commit_hash: commit hash
 */
struct rpmsg_kdrv_ethswitch_fw_version_info {
#define RPMSG_KDRV_TP_ETHSWITCH_YEARLEN		(4)
#define RPMSG_KDRV_TP_ETHSWITCH_MONTHLEN	(3)
#define RPMSG_KDRV_TP_ETHSWITCH_DATELEN		(2)
#define RPMSG_KDRV_TP_ETHSWITCH_COMMITSHALEN	(8)
	u32 major;
	u32 minor;
	u32 rev;
	char year[RPMSG_KDRV_TP_ETHSWITCH_YEARLEN];
	char month[RPMSG_KDRV_TP_ETHSWITCH_MONTHLEN];
	char date[RPMSG_KDRV_TP_ETHSWITCH_DATELEN];
	char commit_hash[RPMSG_KDRV_TP_ETHSWITCH_COMMITSHALEN];
} __packed;

/*
 * per-device data for ethswitch device
 */
/**
 * struct rpmsg_kdrv_ethswitch_device_data - rpmsg_kdrv_device data
 *
 * @fw_ver: fw version info
 * @permission_flags: permission enabled for each
 *	enum @rpmsg_kdrv_ethswitch_message_type command
 * @uart_connected: flag indicating if UART is connected
 * @uart_id: UART ID used by firmware for log prints
 *
 * Provided as part of RPMSG KDRV device discovery protocol
 */
struct rpmsg_kdrv_ethswitch_device_data {
	struct rpmsg_kdrv_ethswitch_fw_version_info fw_ver;
	u32 permission_flags;
	u32 uart_connected;
	u32 uart_id;
} __packed;

#endif /* DRIVERS_RPMSG_KDRV_SHARED_RPMSG_KDRV_TRANSPORT_SWITCH_H_ */
