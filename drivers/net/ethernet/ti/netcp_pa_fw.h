/*
 * Keystone NetCP PA (Packet Accelerator) firmware interface header file
 *
 * Copyright (C) 2012-2015 Texas Instruments Incorporated
 * Author: Murali Karicheri (ported to 4.1.x)
 *
 * Other contributors:	Sandeep Paulraj (Initial version of the driver)
 *			Reece Pollack (Maintenance)
 *			Sandeep Nair (Maintenance)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef NETCP_PA_FW_H
#define NETCP_PA_FW_H

/* Routed Packet Destinations */

/* Packet is discarded */
#define PA_DEST_DISCARD				3
/* packet remains in PA sub-system for more parsing and LUT1 classification */
#define PA_DEST_CONTINUE_PARSE_LUT1		4
/* packet remains in PA sub-system for more parsing and LUT2 classification */
#define PA_DEST_CONTINUE_PARSE_LUT2		5
/* host thread, Packet is routed to host */
#define PA_DEST_HOST				6
/* ethernet mac port (of the switch). Packet is routed to  EMAC */
#define PA_DEST_EMAC				7

#define PA_MAX_MULTI_ROUTE_SETS			32

#define PA_EMAC_CTRL_PORT_MASK			0x0F
#define PA_EMAC_CTRL_CRC_DISABLE		0x80
#define PA_CUSTOM_TYPE_LUT1			1
#define PA_CUSTOM_TYPE_LUT2			2
#define PA_MAX_CUSTOM_TYPES_LUT1		4
#define PA_MAX_CUSTOM_TYPES_LUT2		4
/* Packet is sent to PDSP0 */
#define PA_CMD_TX_DEST_0			0
/* Packet is sent to PDSP1 */
#define PA_CMD_TX_DEST_1			1
/* Packet is sent to PDSP2 */
#define PA_CMD_TX_DEST_2			2
/* Packet is sent to PDSP3 */
#define PA_CMD_TX_DEST_3			3
/* Packet is sent to PDSP4 */
#define PA_CMD_TX_DEST_4			4
/* Packet is sent to PDSP5 */
#define PA_CMD_TX_DEST_5			5

/* PA Command Codes */
#define PA_CMD_NONE				0
#define PA_CMD_NEXT_ROUTE			1
#define PA_CMD_CRC_OP				2
#define PA_CMD_COPY_DATA_TO_PSINFO		3
#define PA_CMD_PATCH_DATA			4
#define PA_CMD_TX_CHECKSUM			5
#define PA_CMD_MULTI_ROUTE			6
#define PA_CMD_REPORT_TX_TIMESTAMP		7
#define PA_CMD_REMOVE_HEADER			8
#define PA_CMD_REMOVE_TAIL			9
#define PA_CMD_CMDSET				10
#define PA_CMD_SA_PAYLOAD			11
#define PA_CMD_IP_FRAGMENT			12
#define PA_CMD_USR_STATS			13
#define PA_CMD_CMDSET_AND_USR_STATS		14

/* Interface based routing modes */

/* No interface based routing */
#define PA_ROUTE_INTF_NONE			0
/* Route by interface number as dest queue offset */
#define PA_ROUTE_INTF_QUEUE			1
/* Route by interface number as both dest queue & CPPI flow offset */
#define PA_ROUTE_INTF_FLOW			2

struct pa_frm_forward_host {
	/* Context returned as swInfo0 for matched packet */
	u32	context;
	/*  Control bitmap, 1 for enable, 0 for disable
	 *  /-----------------------------------------------------\
	 *  | 7           |       2     |      1      |     0     |
	 *  | Selection   |             |Flow IF Dest |           |
	 *  | 0: Priority |             |    OR       |           |
	 *  | 1: IF dest  |DSCP priority|VLAN priority| multiRoute|
	 *  \-----------------------------------------------------/
	 */
	u8	ctrl_bm;
	/* Index of the multiple destination set */
	u8	multi_idx;
	/* PA PDSP number used as multi-route router */
	u8	pa_pdsp_router;
	/* use the bits 7:4.
	 * bit 7: Disable CRC,
	 * bit 6:4 port number (0/1/2),
	 * bit 3:0 errflags = 0
	 * psFlags may be required when the packet is
	 * forwarded through QoS queue
	 */
	u8	ps_flags;
	/* optional simple command:0 means no command */
	u8	cmd[4];
}; /* 12 bytes */

#define PAFRM_MULTIROUTE_ENABLE			0x1
#define PAFRM_ROUTING_PRIORITY_DSCP_ENABLE	0x2
#define PAFRM_ROUTING_PRIORITY_VLAN_ENABLE	0x4
/* 0: queue-based only 1: queue & flow-based */
#define PAFRM_ROUTING_FLOW_IF_BASE_ENABLE	0x2
#define PAFRM_ROUTING_IF_DEST_SELECT_ENABLE	0x80

#define PAFRM_ETH_PS_FLAGS_DISABLE_CRC		0x80
#define PAFRM_ETH_PS_FLAGS_PORT_MASK		0x70
#define PAFRM_ETH_PS_FLAGS_PORT_SHIFT		4

/* Routing information used to forward packets within PA */
struct pa_frm_forward_pa {
	/* PDSP destination */
	u8	pa_dest;
	/* None, LUT1, LUT2 */
	u8	custom_type;
	/* Index of the custom type if LUT1 or LUT2 custom */
	u8	custom_idx;
	u8	rsvd2;
	u32	rsvd3;
	u32	rsvd4;
};

enum {
	/* use PAFRM_DEST_CDMA */
	PAFRM_FORWARD_TYPE_HOST,
	/* use PAFRM_DEST_CDMA */
	PAFRM_FORWARD_TYPE_SA,
	/* use pa.paDest */
	PAFRM_FORWARD_TYPE_PA,
	/* use PAFRM_DEST_ETH */
	PAFRM_FORWARD_TYPE_ETH,
	/* use PAFRM_DEST_CDMA */
	PAFRM_FORWARD_TYPE_SRIO,
	PAFRM_FORWARD_TYPE_DISCARD
};

/* Routing information used to forward packets from PA sub-system to various
 * destinations
 */
struct pa_frm_forward  {
	/* Forwarding type as defined below */
	u8 forward_type;
	/* PKTDMA flow Id, valid if forwarding via PKTDMA */
	u8 flow_id;
	/* Destination queue number, valid if forwarding via PKTDMA */
	u16 queue;

	union {
		/* Host specific routing information */
		struct pa_frm_forward_host	host;
		/* PA internal routing information */
		struct pa_frm_forward_pa	pa;
	} u;
};

/* Custom match flag bits */
#define PAFRM_LUT1_CUSTOM_MATCH_ETYPE			BIT(2)
#define PAFRM_LUT1_CUSTOM_MATCH_VLAN			BIT(3)
/* Ipv6 source and dest entries */
#define PAFRM_LUT1_CUSTOM_MATCH_MATCH			(3 << 4)
#define PAFRM_LUT1_CUSTOM_MATCH_KEY			BIT(13)
#define PAFRM_LUT1_CUSTOM_MATCH_VALID			BIT(15)

/* To add entry to LUT1 */
#define PAFRM_HW_LUT1_ENTRIES				64
/* if PA_LUT1_INDEX_LAST_FREE is used then when the command returns,
 * the value of index will be replaced with the actual index used
 */
#define PAFRM_LUT1_INDEX_LAST_FREE	PAFRM_HW_LUT1_ENTRIES

/* Standard match flag bits */
#define PAFRM_LUT1_MATCH_DMAC				BIT(0)
#define PAFRM_LUT1_MATCH_SMAC				BIT(1)
#define PAFRM_LUT1_MATCH_ETYPE				BIT(2)
#define PAFRM_LUT1_MATCH_VLAN				BIT(3)
#define PAFRM_LUT1_MATCH_SIP				BIT(4)
#define PAFRM_LUT1_MATCH_DIP				BIT(5)
#define PAFRM_LUT1_MATCH_SPI_GRE_SCTP			BIT(6)
#define PAFRM_LUT1_MATCH_FLOW				BIT(7)
#define PAFRM_LUT1_MATCH_SPORT				BIT(8)
#define PAFRM_LUT1_MATCH_DPORT				BIT(9)
#define PAFRM_LUT1_MATCH_PROTO				BIT(10)
#define PAFRM_LUT1_MATCH_TOS				BIT(11)
#define PAFRM_LUT1_MATCH_PORT				BIT(12)
#define PAFRM_LUT1_MATCH_KEY				BIT(13)
#define PAFRM_LUT1_MATCH_VALID				BIT(15)
#define PAFRM_LUT1_MATCH_MPLS		(PAFRM_LUT1_MATCH_SPORT | \
					PAFRM_LUT1_MATCH_DPORT)

/* Key values. The PDSP will set these bits as it parses the headers.
 * LUT1_1 and LUT1_2 (L3): The following bit fields are used
 */
#define PAFRM_LUT1_KEY_SPI				BIT(0)
#define PAFRM_LUT1_KEY_GRE				BIT(1)
#define PAFRM_LUT1_KEY_MPLS				BIT(2)
#define PAFRM_LUT1_KEY_IPV4				BIT(3)
#define PAFRM_LUT1_KEY_IPV6				BIT(4)
#define PAFRM_LUT1_KEY_SCTP				BIT(5)

/* LUT1: Custom  (L3) */
#define PAFRM_LUT1_KEY_CUSTOM				BIT(7)
#define PAFRM_LUT1_KEY_MAC				BIT(0)

struct pa_frm_com_l1_standard {
	/* LUT1 view 1 */
	/* Destination mac */
	u8	dmac[6];
	/* Source mac */
	u8	smac[6];
	/* Ethernrt type, Also used for the previous match PDSP number */
	u16	etype;
	/* VLAN tag, the field is also used for the previous match LUT1 index */
	u16	vlan;

	/* LUT1 view 2 */
	/* Source IP address */
	u8	src_ip[16];
	/* Destination IP address */
	u8	dst_ip[16];

	/* LUT1 view 3 */
	/* ESP or AH header Security Parameters Index. The field is also used
	 * for GRE protocol or SCTP destination port
	 */
	u32	spi;
	/* IPv6 flow label in 20 lsbs */
	u32	flow;

	union {
		/* UDP/TCP Source port (0), destination port (1) */
		u16	ports[2];
		/* mpls label in 20 Lsbs */
		u32	mpls;
	} pm;
	/* Ipv4 Protocol fields, IPv6 next */
	u8	proto_next;
	/* Ipv4 TOS, Ipv6 traffic class */
	u8	tos_tclass;
	/* reserved field: not used */
	u8	inport;
	u8	key;
	/* IP: Distinguishs spi/gre and mpls and ports
	 *  LUT1_0: MAC/SRIO,
	 *  LUT1_1/LUT1_2: custom or standard
	 */
	/* end LUT1 view 3 */
	/* lookup matching valid flags as defined below */
	u16	match_flags;
	/* reserved for alignment */
	u16	rsvd;
};

struct pa_frm_com_l1_custom {
	/* LUT1 view 1 */
	/* unused field: All zero's */
	u8	dmac[6];
	/* unused field: All zero's */
	u8	smac[6];
	/* upper link (previous match PDSP number) */
	u16	etype;
	/* upper link (previous match LUT1 index) */
	u16	vlan;

	/* LUT1 view 2 */
	/* 32 bytes to match   */
	u8	match_values[32];

	/* LUT1 view 3 - offset from start */
	/* unused field: All zero's */
	u32	rsvd0;
	/* unused field: All zero's */
	u32	rsvd1;
	/* unused field: All zero's */
	u32	rsvd2;
	/* unused field: All zero's */
	u8	rsvd3;
	/* unused field: All zero's */
	u8	rsvd4;
	/* unused field: All zero's */
	u8	inport;
	/* IP: Distinguishs spi/gre and mpls and ports
	 * LUT1_0: MAC/SRIO,
	 * LUT1_1/LUT1_2: custom or standard
	 */
	u8	key;
	/* lookup matching valid flags as defined below */
	u16	match_flags;
	/* reserved for alignment */
	u16	rsvd5;
};

enum {
	PAFRM_CONFIG_COMMAND_RSVD,
	PAFRM_CONFIG_COMMAND_ADDREP_LUT1,
	PAFRM_CONFIG_COMMAND_DEL_LUT1,
	PAFRM_CONFIG_COMMAND_ADDREP_LUT2,
	PAFRM_CONFIG_COMMAND_DEL_LUT2,
	PAFRM_CONFIG_COMMAND_CONFIG_PA,
	PAFRM_CONFIG_COMMAND_REQ_STATS,
	PAFRM_CONFIG_COMMAND_REQ_VERSION,
	PAFRM_CONFIG_COMMAND_MULTI_ROUTE,
	PAFRM_CONFIG_COMMAND_CRC_ENGINE,
	PAFRM_CONFIG_COMMAND_CMD_SET,
	PAFRM_CONFIG_COMMAND_USR_STATS,
	PAFRM_CONFIG_COMMAND_SYS_CONFIG
};

/* Command magic value */
#define PAFRM_CONFIG_COMMAND_SEC_BYTE			0xce

/* Command return values */
enum {
	PAFRM_COMMAND_RESULT_SUCCESS,
	/* Command magic value not found */
	PAFRM_COMMAND_RESULT_NO_COMMAND_MAGIC,
	/* Invalid command identifier */
	PAFRM_COMMAND_RESULT_INVALID_CMD,
	/* Add entry to LUT1 fails */
	/* Invalid type, custom or standard IP/ethernet */
	PAFRM_COMMAND_RESULT_LUT1_TYPE_INVALID,
	/* Invalid LUT1 index (0-63) or no free indices available */
	PAFRM_COMMAND_RESULT_LUT1_INDEX_INVALID,
	/* Sent a match packet to q0 on c1 or c2 - this is illegal. */
	PAFRM_COMMAND_RESULT_LUT1_MATCH_DEST_INVALID,
	/* Previous match forward info was somewhere in chunk domain */
	PAFRM_COMMAND_RESULT_LUT1_NMATCH_INVALID,
	/* Invalid combination found in the key value */
	PAFRM_COMMAND_RESULT_LUT1_INVALID_KEYS,
	/* Lut 2 entry warnings since the lut can be configured without pdsp */
	PAFRM_COMMAND_RESULT_WARN_OVER_MAX_ENTRIES,
	PAFRM_COMMAND_RESULT_WARN_NEGATIVE_ENTRY_COUNT,
	/* Lut 2 entry failures */
	/* LUT2 had a lookup and pending config */
	PAFRM_COMMAND_RESULT_LUT2_ADD_BUSY,
	/* Not enough room in stats request packet for the reply */
	PAFRM_COMMAND_RESULT_WARN_STATS_REPLY_SIZE,
	/* Command sent to PDSP which couldn't handle it */
	PAFRM_COMMAND_RESULT_INVALID_DESTINATION,
	/* Add/Delete/Read entries to multi route table */
	/* Asked to use a free entry, but none found */
	PAFRM_COMMAND_RESULT_MULTI_ROUTE_NO_FREE_ENTRIES,
	/* Illegal index value used */
	PAFRM_COMMAND_RESULT_MULTI_ROUTE_INVALID_IDX,
	/* Illegal multi route mode used */
	PAFRM_COMMAND_RESULT_MULTI_ROUTE_INVALID_MODE,

	/* Packet size didn't match command */
	PAFRM_COMMAND_RESULT_INVALID_PKT_SIZE,

	/* Coustom and Command set index */
	/* Illegal Custom LUT1 index value used */
	PAFRM_COMMAND_RESULT_INVALID_C1_CUSTOM_IDX,
	/* Illegal Custom LUT2 index value used */
	PAFRM_COMMAND_RESULT_INVALID_C2_CUSTOM_IDX,
	/* Illegal Custom Command Set index value used */
	PAFRM_COMMAND_RESULT_INVALID_CMDSET_IDX
};

#define PA_SS_TIMER_CNTRL_REG_GO		0x00000001u
#define PA_SS_TIMER_CNTRL_REG_MODE		0x00000002u
#define PA_SS_TIMER_CNTRL_REG_PSE		0x00008000u
#define PA_SS_TIMER_CNTRL_REG_PRESCALE_SHIFT	0x00000002u

/* Destination (route) values */
#define PAFRM_DEST_PDSP0			0
#define PAFRM_DEST_PDSP1			1
#define PAFRM_DEST_PDSP2			2
#define PAFRM_DEST_PDSP3			3
#define PAFRM_DEST_PDSP4			4
#define PAFRM_DEST_PDSP5			5
#define PAFRM_DEST_PKTDMA			6
#define PAFRM_DEST_ETH				7
#define PAFRM_DEST_DISCARD			10

/* Assigning names based on PDSP functions */
#define PAFRM_DEST_PA_C1_0	PAFRM_DEST_PDSP0
#define PAFRM_DEST_PA_C1_1	PAFRM_DEST_PDSP1
#define PAFRM_DEST_PA_C1_2	PAFRM_DEST_PDSP2
#define PAFRM_DEST_PA_C2	PAFRM_DEST_PDSP3
#define PAFRM_DEST_PA_M_0	PAFRM_DEST_PDSP4
#define PAFRM_DEST_PA_M_1	PAFRM_DEST_PDSP5

/* The default queue for packets that arrive at the PA and don't match in
 * classify1 (right at init time)
 */
#define PAFRM_DEFAULT_INIT_Q			0x100

/* Ethertypes recognized by the firmware. */
#define PAFRM_ETHERTYPE_IP			0x0800
#define PAFRM_ETHERTYPE_IPV6			0x86dd
#define PAFRM_ETHERTYPE_VLAN			0x8100
#define PAFRM_ETHERTYPE_SPVLAN			0x88a8
#define PAFRM_ETHERTYPE_MPLS			0x8847
#define PAFRM_ETHERTYPE_MPLS_MULTI		0x8848

/* Next header type values  */
#define PAFRM_HDR_MAC				0
#define PAFRM_HDR_VLAN				1
#define PAFRM_HDR_MPLS				2
#define PAFRM_HDR_IPV4				3
#define PAFRM_HDR_IPV6				4
#define PAFRM_HDR_IPV6_EXT_HOP			5
#define PAFRM_HDR_IPV6_EXT_ROUTE		6
#define PAFRM_HDR_IPV6_EXT_FRAG			7
#define PAFRM_HDR_IPV6_EXT_DEST			8
#define PAFRM_HDR_GRE				9
#define PAFRM_HDR_ESP				10
#define PAFRM_HDR_ESP_DECODED			11
#define PAFRM_HDR_AUTH				12
#define PAFRM_HDR_CUSTOM_C1			13
#define PAFRM_HDR_FORCE_LOOKUP			14
#define PAFRM_HDR_SCTP				15
#define PAFRM_HDR_UNKNOWN			16
#define PAFRM_HDR_UDP				17
#define PAFRM_HDR_UDP_LITE			18
#define PAFRM_HDR_TCP				19
#define PAFRM_HDR_GTPU				20
#define PAFRM_HDR_ESP_DECODED_C2		21
#define PAFRM_HDR_CUSTOM_C2			22

/* Command related definitions */
#define PAFRM_CRC_FLAG_CRC_OFFSET_VALID		0x01
#define PAFRM_CRC_FLAG_CRC_OFFSET_FROM_DESC	0x02
#define PAFRM_CHKSUM_FALG_NEGATIVE		0x01

#define PA_NEXT_ROUTE_PARAM_PRESENT		0x0001
#define PA_NEXT_ROUTE_PROC_NEXT_CMD		0x0002
#define PA_NEXT_ROUTE_PROC_MULTI_ROUTE		0x0004

/* PAFRM receive commands related definitions */

/* There are the following two groups of PAFRM receive commands:
 * PAFRM short commands which can be used as part of the routing info
 * PAFRM commands which can be used within a command set
 */

/* Dummy command */
#define PAFRM_RX_CMD_NONE			0

/* short commands */
/* Execute a command set */
#define PAFRM_RX_CMD_CMDSET			1
/* Insert up to two types at the current location */
#define PAFRM_RX_CMD_INSERT			2

/* command set commands */
/* Specify the next route */
#define PAFRM_RX_CMD_NEXT_ROUTE			3
/* CRC generation or verification */
#define PAFRM_RX_CMD_CRC_OP			4
/* Copy data to the PS Info section */
#define PAFRM_RX_CMD_COPY_DATA			5
/* Insert or pacth packet data at the specific location */
#define PAFRM_RX_CMD_PATCH_DATA			6
/* Remove the parsed packet header */
#define PAFRM_RX_CMD_REMOVE_HDR			7
/* Remove the parsed packet tail */
#define PAFRM_RX_CMD_REMOVE_TAIL		8
/* Duplicate packet to multiple destinations */
#define PAFRM_RX_CMD_MULTI_ROUTE		9

/* PASS command ID formatting
 * Bit 15 is used to distinguish the L2 table from
 * the L3 table in the command comId field
 */
#define PA_COMID_L2				0
#define PA_COMID_L3				BIT(15)
#define PA_COMID_L_MASK				BIT(15)
#define PA_COMID_IDX_MASK			(~(PA_COMID_L_MASK))

/* define LUT1 entry types */
/* MAC/IP */
#define PAFRM_COM_ADD_LUT1_STANDARD		0
/* Custom LUT1 */
#define PAFRM_COM_ADD_LUT1_CUSTOM		2

struct pa_frm_cmd_add_lut1 {
	/* LUT1 index. */
	u8	index;
	/* Custom or standard */
	u8	type;
	/* reserved for alignment */
	u8	rsvd;
	/* Valid only if type is custom */
	u8	cust_index;

	union {
		/* matching information for MAC/IP entry */
		struct	pa_frm_com_l1_standard	eth_ip;
		struct	pa_frm_com_l1_custom	custom;
	} u;

	/* Routing information when a match is found */
	struct	pa_frm_forward match;

	/* Routing information when subsequent match fails - a fragmented
	 * packet orinner route
	 */
	struct	pa_frm_forward next_fail;
};

/* CRC Engine Configuration */
#define PARAM_CRC_TABLE_SIZE			16

struct pa_frm_config_crc {
	/* Control bit maps as defined below */
	u8	ctrl_bitmap;
#define PARAM_CRC_SIZE_8			0
#define PARAM_CRC_SIZE_16			1
#define PARAM_CRC_SIZE_24			2
#define PARAM_CRC_SIZE_32			3
#define PARAM_CRC_CTRL_CRC_SIZE_MASK		0x3
#define PARAM_CRC_CTRL_LEFT_SHIFT		0x0
#define PARAM_CRC_CTRL_RIGHT_SHIFT		0x4
#define PARAM_CRC_CTRL_INV_RESULT		0x8
	/* reserved for alignment */
	u8	rsvd1;
	/* reserved for alignment */
	u16	rsvd2;
	/* Initial value to use in the CRC calcualtion */
	u32	init_val;
	/* CRC table */
	u32	crc_tbl[PARAM_CRC_TABLE_SIZE];
};

/* Commands to PA */
struct pa_frm_command {
	/* Returned to the host, ignored on entry to the PASS */
	u32	command_result;
	/* Command value */
	u8	command;
	/* Magic value */
	u8	magic;
	/* Used by the host to identify command results */
	u16	com_id;
	/* Returned in swInfo to identify packet as a command */
	u32	ret_context;
	/* Specifies the queue number for the message reply. 0xffff to toss the
	 * reply
	 */
	u16	reply_queue;
	/* Reply destination:- host0, host1, discard are only valid values */
	u8	reply_dest;
	/* Flow ID used to assign packet at reply */
	u8	flow_id;
	/* First word of the command */
	u32	cmd;
};

struct pa_cmd_next_route {
	/* Routing control information as defined at @ref routeCtrlInfo */
	u16	ctrl_bit_field;
	/* Packet destination as defined at @ref pktDest */
	int	dest;
	/* For destination EMAC, specify the EMAC control to the network */
	u8	pkt_type_emac_ctrl;
	/* For host, SA or SRIO destinations, specifies return free
	 * descriptor setup
	 */
	u8	flow_id;
	/*For host, SA or SRIO destinations, specifies the dest queue */
	u16	queue;
	/* Placed in SwInfo0 for packets to host or SA */
	u32	sw_info_0;
	/* Placed in SwInfo1 for packets to the SA */
	u32	sw_info_1;
	/* Multi-route index. It is valid in the from-network direction only */
	u16	multi_route_index;
};

struct pa_cmd_crc_op {
	/* CRC operation control information */
	u16	ctrl_bit_field;
	/* Byte location, from SOP/Protocol Header, where the CRC
	 * computation begins if frame type is not specified Byte location,
	 * from SOP/Protocol header, where the specific frame header begins
	 * if frame type is specified In to-network direction: offset from SOP
	 * In from-network direction: offset from the current parsed header
	 */
	u16	start_offset;
	/* Number of bytes covered by the CRC computation
	 * valid only if pa_CRC_OP_PAYLOAD_LENGTH_IN_HEADER is clear
	 */
	u16	len;
	/* Payload length field offset in the custom header */
	u16	len_offset;
	/* Payload length field mask */
	u16	len_mask;
	/* Payload length adjustment: valid only if
	 * PA_CRC_OP_PAYLOAD_LENGTH_IN_HEADER is set
	 */
	u16	len_adjust;
	/* Offset from SOP/Protocol Header to the CRC field In to-network
	 * direction: offset from SOP In from-network direction: offset
	 * from the current parsed header
	 */
	u16	crc_offset;
	/* Frame type valid if PA_CRC_OP_CRC_FRAME_TYPE is set */
	u16	frame_yype;
};

struct pa_tx_chksum {
	/* Byte location, from SOP, where the checksum calculation begins */
	u16	start_offset;
	/* Number of bytes covered by the checksum. Must be even */
	u16	length_bytes;
	/* Byte offset, from startOffset, to place the resulting checksum */
	u16	result_offset;
	/* Initial value of the checksum */
	u16	initial_sum;
	/* If TRUE, a computed value of 0 is written as -0 */
	u16	negative_0;
};

struct pa_cmd_copy {
	u16	ctrl_bitfield;
	u16	src_offset;
	u16	dest_offset;
	u16	num_bytes;
};

struct pa_patch_info {
	unsigned int	n_patch_bytes;
	unsigned int	total_patch_size;
	unsigned int	offset;
	u16		overwrite;
	u8		*patch_data;
};

struct pa_payload_info  {
	u16	offset;
	u16	len;
};

struct pa_cmd_multi_route {
	u16	index;
};

#define PA_MAX_CMD_SETS				8
#define PA_OK					0
#define PA_ERR_CONFIG				-10
#define PA_INSUFFICIENT_CMD_BUFFER_SIZE		-11
#define PA_INVALID_CMD_REPLY_DEST		-12

struct pa_cmd_set {
	u16	index;
	/* Command Set Index */
};

struct pa_cmd_tx_timestamp {
	/* Host queue for the tx timestamp reporting packet */
	u16	dest_queue;
	/* CPPI flow */
	u16	flow_id;
	/* 32 bit value returned in the descriptor */
	u32	sw_info0;
};

struct pa_cmd_ip_frag {
	/* Offset to the IP header. */
	u16	ip_offset;
	/* Size of the maximum transmission unit (>= 68) */
	u16	mtu_size;
};

struct pa_cmd_usr_stats {
	/* User-defined statistics index */
	u16	index;
};

struct pa_cmd_set_usr_stats {
	/* Commad Set Index */
	u16	set_index;
	/* User-defined statistics index */
	u16	stats_index;
};

struct pa_cmd_info {
	/* Specify the PA command code */
	u16	cmd;
	union {
		/* Specify nextRoute command specific parameters */
		struct pa_cmd_next_route route;
		/* Specify Tx Checksum command specific parameters */
		struct pa_tx_chksum	chksum;
		/* Specify CRC operation command specific parameters */
		struct pa_cmd_crc_op     crc_op;
		/* Specify Copy command specific parameters */
		struct pa_cmd_copy	copy;
		/* Specify Patch command specific parameters */
		struct pa_patch_info	patch;
		/* Specify the payload information required by SA */
		struct pa_payload_info	payload;
		/* Specify Command Set command specific parameters */
		struct pa_cmd_set	cmd_set;
		/* Specify Multi-route command specific parameters */
		struct pa_cmd_multi_route m_route;
		/*Specify Report Tx Timestamp command specific parameters */
		struct pa_cmd_tx_timestamp tx_ts;
		/* Specify IP fragmentation command specific parameters */
		struct pa_cmd_ip_frag	ip_frag;
		/* Specify User-defined Stats command specific parameters */
		struct pa_cmd_usr_stats usr_stats;
		struct pa_cmd_set_usr_stats cmd_set_usr_stats;
	} params;
};

struct pa_route_info {
	int	dest;
	u8	flow_id;
	u16	queue;
	int	m_route_index;
	u32	sw_info_0;
	u32	sw_info_1;
	int	custom_type;
	u8	custom_index;
	u8	pkt_type_emac_ctrl;
	u8	route_type;
	struct pa_cmd_info *pcmd;
};

/* Exception routing enumeration */
enum pa_eroutes {
	/* packet failed to match in LUT1 table */
	EROUTE_LUT1_FAIL = 0,
	/* packet exceeded maximum number of VLAN tags */
	EROUTE_VLAN_MAX_DEPTH,
	/* packet exceeded maximum number of IP headers */
	EROUTE_IP_MAX_DEPTH,
	/* packet exceeded maximum number of MPLS headers */
	EROUTE_MPLS_MAX_DEPTH,
	/* packet exceeded maximum number of GRE headers */
	EROUTE_GRE_MAX_DEPTH,
	/* packet failed to parse */
	EROUTE_PARSE_FAIL,
	/* packet failed to match in LUT2 table */
	EROUTE_LUT2_FAIL,
	/* IP fragmented packet found in classify2 lookup */
	EROUTE_IP_FRAG,
	/* Packet failed due to unsupported IPV6 option header */
	EROUTE_IPV6_OPT_FAIL,
	/* Udp lite checksum coverage invalid */
	EROUTE_UDP_LITE_FAIL,
	/* IPv4 strict source route or IPv6 routing extension header */
	EROUTE_ROUTE_OPTION,
	/* Unknown system failure - should never happen */
	EROUTE_SYSTEM_FAIL,
	/* MAC broadcast packet */
	EROUTE_MAC_BROADCAST,
	/* MAC multicast packet */
	EROUTE_MAC_MULTICAST,
	/* IP broadcast packet */
	EROUTE_IP_BROADCAST,
	/* IP multicast packet */
	EROUTE_IP_MULTICAST,
	/* GTP-U PING Request packet */
	EROUTE_GTPU_MESSAGE_TYPE_1,
	/* GTP-U PING Response packet */
	EROUTE_GTPU_MESSAGE_TYPE_2,
	/* GTP-U Error Indication packet */
	EROUTE_GTPU_MESSAGE_TYPE_26,
	/* GTP-U Supported Header Notification packet */
	EROUTE_GTPU_MESSAGE_TYPE_31,
	/* GTP-U End Markr packet */
	EROUTE_GTPU_MESSAGE_TYPE_254,
	/*failed due to GTPU parsing error or unsupported dmessage types */
	EROUTE_GTPU_FAIL,
	/* Packet failed due to PPPoE session packet parsing error */
	EROUTE_PPPOE_FAIL,
	/* PPPoE session stage non-IP packets */
	EROUTE_PPPOE_CTRL,
	/* 802.1ag Packet*/
	EROUTE_802_1ag,
	/* Packet failed due to invalid IP header */
	EROUTE_IP_FAIL,
	/* NAT-T Keep Alive packet where UDP Length = 9, data = 0xFF */
	EROUTE_NAT_T_KEEPALIVE,
	/* NAT-T control packet where UDP Length > 12 and the first 4 payload
	 * bytes are equal to 0
	 */
	EROUTE_NAT_T_CTRL,
	/* NAT-T IPSEC ESP data packet where UDP Length > 12 and the first 4
	 * payload bytes are not equal to 0
	 */
	EROUTE_NAT_T_DATA,
	/* Invalid NAT-T packet */
	EROUTE_NAT_T_FAIL,
	/* Packet failed to match GTPU */
	EROUTE_GTPU_MATCH_FAIL,
	/* Number of error routes */
	EROUTE_N_MAX
};

/* exception route configuration */
struct pa_frm_com_eroute {
	/* Exception route valid bitmap */
	u32			route_bitmap;
	/* Array of exception routing information */
	struct pa_frm_forward	eroute[EROUTE_N_MAX];
};

/* PA system configuration command */
struct pa_frm_command_sys_config_pa {
	/* system configuration code as defined below */
	u8	cfg_code;
	u8	rsvd1;
	/* reserved for alignment */
	u16	rsvd2;

	union {
		/* Exception routes configuration */
		struct pa_frm_com_eroute eroute;
	} u;
};

/* PA system configuration codes */
#define PAFRM_SYSTEM_CONFIG_CODE_EROUTE         0
#define PAFRM_SYSTEM_CONFIG_CODE_CUSTOM_LUT1    1
#define PAFRM_SYSTEM_CONFIG_CODE_CUSTOM_LUT2    2
#define PAFRM_SYSTEM_CONFIG_CODE_802_1AG        3
#define PAFRM_SYSTEM_CONFIG_CODE_IPSEC_NAT_T    4
#define PAFRM_SYSTEM_CONFIG_CODE_GTPU           5

#endif /* NETCP_PA_FW_H */
