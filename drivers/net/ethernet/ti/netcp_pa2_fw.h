/*
 * Keystone NetCP PA2 (Packet Accelerator) firmware interface header file
 *
 * Copyright (C) 2013-2015 Texas Instruments Incorporated
 * Author :	Murali Karicheri (ported to kernel 4.1.x)
 *
 * Other contributors:	Hao Zhang (Initial version of the driver)
 *			Reece Pollack (Maintenance)
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

#ifndef KEYSTONE_PA2_FW_H
#define KEYSTONE_PA2_FW_H

/* PA Timer */
enum pa2_timerstamp_scaler_factor {
	PA2_TIMESTAMP_SCALER_FACTOR_1 = -1,
	PA2_TIMESTAMP_SCALER_FACTOR_2 = 0,
	PA2_TIMESTAMP_SCALER_FACTOR_4,
	PA2_TIMESTAMP_SCALER_FACTOR_8,
	PA2_TIMESTAMP_SCALER_FACTOR_16,
	PA2_TIMESTAMP_SCALER_FACTOR_32,
	PA2_TIMESTAMP_SCALER_FACTOR_64,
	PA2_TIMESTAMP_SCALER_FACTOR_128,
	PA2_TIMESTAMP_SCALER_FACTOR_256,
	PA2_TIMESTAMP_SCALER_FACTOR_512,
	PA2_TIMESTAMP_SCALER_FACTOR_1024,
	PA2_TIMESTAMP_SCALER_FACTOR_2048,
	PA2_TIMESTAMP_SCALER_FACTOR_4096,
	PA2_TIMESTAMP_SCALER_FACTOR_8192
};

#define PA2_SS_TIMER_CNTRL_REG_GO		0x1u
#define PA2_SS_TIMER_CNTRL_REG_MODE		0x2u
#define PA2_SS_TIMER_CNTRL_REG_PSE		0x8000u
#define PA2_SS_TIMER_CNTRL_REG_PRESCALE_SHIFT	0x2u

#define PA2_SYS_TIMESTAMP_ADDR_OFFSET		0xf0

/* PA Commands */
#define PA2FRM_MAX_CMD_SET_SIZE			124

/* Routed Packet Destinations */

/* Packet is discarded */
#define PA2_DEST_DISCARD			3
/* Packet remains in PA sub-system for more parsing and LUT1 classification */
#define PA2_DEST_CONTINUE_PARSE_LUT1		4
/* Packet remains in PA sub-system for more parsing and LUT2 classification */
#define PA2_DEST_CONTINUE_PARSE_LUT2		5
/* Packet is routed to host */
#define PA2_DEST_HOST				6
/* Packet is routed to  EMAC */
#define PA2_DEST_EMAC				7
/* Packet is routed to SA */
#define PA2_DEST_SASS				8
/* Packet is routed to SRIO */
#define PA2_DEST_SRIO				9
/* Cascaded forwarding packet remains in PA sub-system for next LUT1 (IP)
 * parsing. Those packets are expected to be delivered to QoS queues based
 * on the VLAN/DSCP priority at the next stage so that some PASS actions
 * such as IP reassembly and IP fragment exception route will be disabled.
 */
#define PA2_DEST_CASCADED_FORWARDING_LUT1	10
/* Packet is routed to SA through local DMA */
#define PA2_DEST_SASS_LOC_DMA			11
/* Packet is routed to Egress Flow Path */
#define PA2_DEST_EFLOW				12
/* Reseved destination for internal usage */
#define PA2_DEST_RES_1				20
/* Reseved destination for internal usage */
#define PA2_DEST_RES_2				21

/* Multi Route */
#define PA2_NO_MULTI_ROUTE			-1
#define PA2_MAX_MULTI_ROUTE_SETS		32
#define PA2_MAX_MULTI_ROUTE_ENTRIES		8
#define PA2_MULTI_ROUTE_DESCRIPTOR_ONLY		0x01
#define PA2_MULTI_ROUTE_REPLACE_SWINFO		0x02

/* Egress FLOW Operation */
#define PA2_EF_OP_CONTROL_FLAG_FC_LOOKUP	0x0001

#define PA2_EF_OP_INFO_VALID_LVL1		0x0001
#define PA2_EF_OP_INFO_VALID_LVL2		0x0002
#define PA2_EF_OP_INFO_VALID_LVL3		0x0004
#define PA2_EF_OP_INFO_VALID_LVL4		0x0008

struct pa2_ef_op_info {
	/* Specify Egress flow control flags as defined at
	 * paEfOpInfoCtrlFlags
	 */
	u16	ctrl_flags;
	/* Specify valid parameters as defined at paEfOpInfoValidBit */
	u16	valid_bitmap;
	/* Specify egress flow level 1 record index */
	u16	lvl1_index;
	/* Specify egress flow level 2 record index */
	u16	lvl2_index;
	/* Specify egress flow level 3 record index */
	u16	lvl3_index;
	/* Specify egress flow level 4 record index */
	u16	lvl4_index;
};

enum pa2_route_pri_info {
	/* Route by using VLAN bits as pri */
	PA2_ROUTE_PRIORITY_VLAN = 1,
	/* Route by using DSCP bits as pri */
	PA2_ROUTE_PRIORITY_DSCP,
	/* Route by using EMAC port (interface) number as destination
	 * queue offset
	 */
	PA2_ROUTE_INTF,
	/* Route by using EMAC port (interface) number as both destination
	 * queue and CPPI flow offset
	 */
	PA2_ROUTE_INTF_W_FLOW
};

/* EMAC Control */
#define PA2_EMAC_CTRL_PORT_MASK			0x0F
#define PA2_EMAC_CTRL_CRC_DISABLE		0x80

/* Custom Classification Types */
#define PA2_CUSTOM_TYPE_NONE			0
#define PA2_CUSTOM_TYPE_LUT1			1
#define PA2_CUSTOM_TYPE_LUT2			2
#define PA2_MAX_CUSTOM_TYPES_LUT1		4
#define PA2_MAX_CUSTOM_TYPES_LUT2		16

/* Command Transmit Packet Destinations */

/* Packet is sent to INGRESS0/PDSP0 */
#define PA2_CMD_TX_DEST_0			0
/* Packet is sent to INGRESS1/PDSP1 */
#define PA2_CMD_TX_DEST_1			1
/* Packet is sent to INGRESS2/PDSP2 */
#define PA2_CMD_TX_DEST_2			2
/* Packet is sent to INGRESS3/PDSP3 */
#define PA2_CMD_TX_DEST_3			3
/* Packet is sent to INGRESS4/PDSP4 */
#define PA2_CMD_TX_DEST_4			4
/* Packet is sent to POST/PDSP5 */
#define PA2_CMD_TX_DEST_5			5
/* Packet is sent to EGRESS0 */
#define PA2_CMD_TX_DEST_6			6
/* Packet is sent to EGRESS1 */
#define PA2_CMD_TX_DEST_7			7
/* Packet is sent to EGRESS2 */
#define PA2_CMD_TX_DEST_8			8

/* PA Command Codes */
#define PA2_CMD_NONE				0
#define PA2_CMD_NEXT_ROUTE			1
#define PA2_CMD_CRC_OP				2
#define PA2_CMD_COPY_DATA_TO_PSINFO		3
#define PA2_CMD_PATCH_DATA			4
#define PA2_CMD_TX_CHECKSUM			5
#define PA2_CMD_MULTI_ROUTE			6
#define PA2_CMD_REPORT_TX_TIMESTAMP		7
#define PA2_CMD_REMOVE_HEADER			8
#define PA2_CMD_REMOVE_TAIL			9
#define PA2_CMD_CMDSET				10
#define PA2_CMD_SA_PAYLOAD			11
#define PA2_CMD_IP_FRAGMENT			12
#define PA2_CMD_USR_STATS			13
#define PA2_CMD_CMDSET_AND_USR_STATS		14

#define PA2_CMD_PATCH_MSG_LEN			15
#define PA2_CMD_VERIFY_PKT_ERROR		16
#define PA2_CMD_SPLIT				17
#define PA2_CMD_EF_OP				18

struct pa2_frm_forward_host {
	/* Context returned as swInfo0 for matched packet */
	u32	context;
	/* Control bitmap, 1 for enable, 0 for disable
	 *  /------------------------------------------------------------------\
	 *  | 7                 |    |       2     |      1      |     0       |
	 *  | Selection         |    |DSCP priority|VLAN priority| multiRoute  |
	 *  | 0: Priority Select|    |		   |	OR	 |             |
	 *  | 1: IF Dest Select |    |             |Flow IF Dest |             |
	 *  \------------------------------------------------------------------/
	 */
	/* True if multiple destination enabled */
	u8	ctrl_bitmap;
	/* Index of the multiple destination set */
	u8	multi_idx;
	 /* PA PDSP number used as multi-route router */
	u8	pa_pdsp_router;
	/* use the bit 7:4, bit 7: Disable CRC, bit 6:4 port number (0/1/2),
	 * bit 3:0 errflags = 0
	 */
	u8	ps_flags;
	/* optional simple command: 0 means no command */
	u8	cmd[4];
}; /* 12 bytes */

#define PA2FRM_MULTIROUTE_ENABLE		0x1
#define PA2FRM_ROUTING_PRIORITY_DSCP_ENABLE	0x2
#define PA2FRM_ROUTING_PRIORITY_VLAN_ENABLE	0x4
/* 0: queue-based only; 1: queue- and flow-based */
#define PA2FRM_ROUTING_FLOW_IF_BASE_ENABLE	0x2
#define PA2FRM_ROUTING_IF_DEST_SELECT_ENABLE	0x80

/* Routing information used to forward packets to the Ethernet port */
struct pa2_frm_forward_eth {
	/* use the bit 7:4 bit 7: Disable CRC, bit 6:4 port number (0/1/2),
	 * bit 3:0 errflags = 0
	 */
	u8	ps_flags;
	u8	priority;
	u16	rsvd2;
	u32	rsvd3;
	u8	cmd[4];
};

#define PA2FRM_ETH_PS_FLAGS_DISABLE_CRC		0x80
#define PA2FRM_ETH_PS_FLAGS_PORT_MASK		PA2_EMAC_CTRL_PORT_MASK
#define PA2FRM_ETH_PS_FLAGS_PORT_SHIFT		0

/* Routing information used to forward packets within PA */
struct pa2_frm_forward_pa {
	/* PDSP destination */
	u8	pa_dest;
	/* None, LUT1, LUT2 */
	u8	custom_type;
	/* Index of the custom type if LUT1 or LUT2 custom */
	u8	custom_idx;
	u8	flags;
	u32	rsvd1;
	u8	cmd[4];
};

#define PA2FRM_CASCADED_FORWARDING		0x01
/* Mark the entry per ACL rule */
#define PA2FRM_PA_CTRL_PKT_MARK			0x02
/* Indicate that the packet should be dropped after reassembly per ACL rule */
#define PA2FRM_PA_CTRL_PKT_DROP			0x04

/* Routing information used to forward packets in egress flow */
struct pa2_frm_forward_ef {
	/* various control flags */
	u8	ctrl_flags;
	/* Egress record valid bit map, if flow cache lookup is not enabled */
	u8	valid_bitmap;
	/* reserved for alignment */
	u16	rsvd1;
	/* Egress Flow level one record index */
	u8	lvl1_rec_idx;
	/* Egress Flow level two record index */
	u8	lvl2_rec_idx;
	/* Egress Flow level three record index */
	u8	lvl3_rec_idx;
	/* Egress Flow level four record index */
	u8	lvl4_rec_idx;
	u32	rsvd2;
};

#define PA2FRM_EF_CTRL_FC_LOOKUP		0x01	/* Flow Cache lookup */
#define PA2FRM_EF_VALID_REC_LVL1		0x01
#define PA2FRM_EF_VALID_REC_LVL2		0x02
#define PA2FRM_EF_VALID_REC_LVL3		0x04
#define PA2FRM_EF_VALID_REC_LVL4		0x08

/* Routing information used to drop the packet */
struct pa2_frm_discard {
	u32	rsvd1;
	u32	rsvd2;
	u8	cmd[4];
};

#define PA2FRM_CUSTOM_TYPE_NONE		PA2_CUSTOM_TYPE_NONE
#define PA2FRM_CUSTOM_TYPE_LUT1		PA2_CUSTOM_TYPE_LUT1
#define PA2FRM_CUSTOM_TYPE_LUT2		PA2_CUSTOM_TYPE_LUT2

/* Routing information used to forward packets fromm PA sub-system to various
 * destinations
 */
struct pa2_frm_forward  {
	u8 forward_type;	/* Forwarding type as defined below */
	u8 flow_id;		/* PKTDMA flow Id, valid if forwarding via
				 * PKTDMA
				 */
	u16 queue;		/* Destination queue number, valid if forwarding
				 * via PKTDMA
				 */
	union {
		struct pa2_frm_forward_host	host;    /* Host specific
							  * routing
							  * information
							  */
		struct pa2_frm_forward_eth	eth;     /* Ethernet specific
							  * routing
							  * information
							  */
		struct pa2_frm_forward_pa	pa;      /* PA internal routing
							  * information
							  */
		struct pa2_frm_forward_ef	ef;      /* PA Egress Flow
							  * information
							  */
		struct pa2_frm_discard		discard; /* Discard specific
							  * routing
							  * information
							  */
	} u;
};

enum {
	/* use PA2FRM_DEST_CDMA */
	PA2FRM_FORWARD_TYPE_HOST = 0,
	/* use PA2FRM_DEST_CDMA */
	PA2FRM_FORWARD_TYPE_SA,
	/* use pa.paDest */
	PA2FRM_FORWARD_TYPE_PA,
	/* use PA2FRM_DEST_ETH */
	PA2FRM_FORWARD_TYPE_ETH,
	/* use PA2FRM_DEST_CDMA */
	PA2FRM_FORWARD_TYPE_SRIO,
	/* use flowId as stream ID  */
	PA2FRM_FORWARD_TYPE_SA_DIRECT,
	PA2FRM_FORWARD_TYPE_DISCARD,
	PA2FRM_FORWARD_TYPE_EFLOW
};

/* LUT1 classification mode */
#define PA2FRM_LUT1_CLASS_NONE			0
#define PA2FRM_LUT1_CLASS_STANDARD		1
#define PA2FRM_LUT1_CLASS_IPV4			2
#define PA2FRM_LUT1_CLASS_IPV6			3
#define PA2FRM_LUT1_CLASS_IPSEC		PA2FRM_LUT1_CLASS_IPV6

/*  Care0 [31:30] */
#define PA2FRM_LUT1_CLASS_SHIFT			30

/* LUT1 Range mode for two range parameters */

/* Normal comparsion */
#define PA2FRM_LUT1_RANGE_MODE_NORMAL		0
/* Use RangeLo and RangeHi for range compare */
#define PA2FRM_LUT1_RANGE_MODE_RANGE		1
/* NOT comparison */
#define PA2FRM_LUT1_RANGE_MODE_NOT		2
/* Care0 [29:28]: Byte 42-43*/
#define PA2FRM_LUT1_CLASS_SHIFT0		28
/* Care0 [27:26]: Byte 44-45*/
#define PA2FRM_LUT1_CLASS_SHIFT1		26

/* LUT1 NOT operation for byte 41 */

/* Normal Comparsion */
#define PA2FRM_LUT1_CMP_OP_NORMAL		0
/* Not cpmpare (byte 41) */
#define PA2FRM_LUT1_CMP_OP_NOT			1
/* Care 0 [25]: Byte 41 */
#define PA2FRM_LUT1_CMP_OP_SHIFT		25

/* LUT1 Entries (MAC/SRIO/Custom) */
/* Layer 2 packet Type */
#define PA2FRM_L2_PKT_TYPE_MAC			0x80
#define PA2FRM_L2_PKT_TYPE_SRIO			0x40
#define PA2FRM_L2_PKT_TYPE_CUSTOM		0x20
/* Care0: [20:15] byte 4-9 */
#define PA2FRM_LUT1_VALID_DMAC_ALL		0x001F8000
#define PA2FRM_LUT1_VALID_DMAC_MINUS_BYTE5	0x001F0000
/* Care0: [14:9]  byte 10-15 */
#define PA2FRM_LUT1_VALID_SMAC			0x00007E00
/* Care0: [8:7]   byte 16-17 */
#define PA2FRM_LUT1_VALID_ETHERTYPE		0x00000180
/* Care0: [6:5]   byte 18-19 */
#define PA2FRM_LUT1_VALID_SESSION_ID		0x00000060
/* Care0: [4:1]   byte 20-23 */
#define PA2FRM_LUT1_VALID_MPLS			0x0000001E
/* Care0: [24:24] Byte 0 */
#define PA2FRM_LUT1_VALID_PKTFLAGS		0x01000000
/* Care0: [23:23] Byte 1 */
#define PA2FRM_LUT1_VALID_DMAC5			0x00800000

/* Care1: [20:19] Byte 36-37 */
#define PA2FRM_LUT1_VALID_VLANID1		0x00180000
/* Care1: [18:17] Byte 38-39 */
#define PA2FRM_LUT1_VALID_VLANID2		0x00060000
/* Care1: [16:16] Byte 40    */
#define PA2FRM_LUT1_VALID_PKTTYPE		0x00010000
/* Care1: [15:15] Byte 41    */
#define PA2FRM_LUT1_VALID_INPORT		0x00008000
/* Care1: [14:13] Byte 42-43 */
#define PA2FRM_LUT1_VALID_VLAN_PRI1		0x00006000
/* Care1: [12:11] Byte 44-45 */
#define PA2FRM_LUT1_VALID_VLAN_PRI2		0x00001800
/* Care1: [10: 9] Byte 46-47 */
#define PA2FRM_LUT1_VALID_SRC_VC		0x00000600

#define PA2FRM_MK_SRC_VC(pdsp, lut_index)	(((pdsp) << 10) + (lut_index))
#define PA2FRM_GET_PDSPID_FROM_LINK(lnk)	((lnk) >> 10)
#define PA2FRM_GET_LUTIDX_FROM_LINK(lnk)	((lnk) & 0x3FF)

/* Add MAC entry to LUT1 */
struct pa2_frm_com_l1_mac {
	/* LUT1 view 1 */
	/* Destination mac */
	u8	dmac[6];
	/* Source mac */
	u8	smac[6];
	/* Ethernrt type, the field is also used for the previous match
	 * PDSP number
	 */
	u16	etype;
	/* PPPoE session ID */
	u16	session_id;

	/* LUT1 view 2 */
	/* MPLS label */
	u32	mpls;
	u32	rsvd2_2;
	u32	rsvd2_3;
	u32	rsvd2_4;

	/* LUT1 view 3 */
	/* Various packet flags */
	u8	pkt_flags;
#define PA2FRM_MAC_FLAG_VLAN1			0x01
#define PA2FRM_MAC_FLAG_VLAN2			0x02
#define PA2FRM_MAC_FLAG_MCAST			0x04
#define PA2FRM_MAC_FLAG_BCAST			0x08
#define PA2FRM_MAC_FLAG_PPPOE			0x10
#define PA2FRM_MAC_FLAG_802p3			0x20
#define PA2FRM_MAC_FLAG_MPLS			0x40

	/* Destination MAC address if bitMask is required */
	u8	dst_mac5;
	u16	rsvd4;
	/* 12-bit ID of inner VLAN (0x8100) */
	u16	vlan_id1;
	/* 12-bit ID of outer VLAN */
	u16	vlan_id2;
	/* Common filed to indicate packet type */
	u8	pkt_type;
	/* One-base input EMAC port number */
	u8	in_port;
	/* 3-bit priority of inner VLAN (0x8100) */
	u16	vlan_pri1;
	/* 3-bit priority of outer VLAN */
	u16	vlan_pri2;
	/* virtual or physical link */
	u16	src_vc;
};

struct pa2_frm_com_l1_custom {
	/* LUT1 view 1 & 2 */
	u8	match[32];

	/* LUT1 view 3 - offset from start */
	u32	rsvd3_1;
	u32	rsvd3_2;
	/* Common filed to indicate packet type */
	u8	pkt_type;
	u8	rsvd3_3;
	u16	rsvd3_4;
	u16	rsvd3_5;
	/* virtual or physical link */
	u16	src_vc;
};

struct pa2_frm_com_l1_ipv4 {
	/* LUT1 view 1 */
	/* Destination IP address */
	u32	dst_ip;
	/* Source IP address */
	u32	src_ip;
	u32	rsvd1_3;
	u32	rsvd1_4;

	/* LUT1 view 2 */
	u32	rsvd2_1;
	u32	rsvd2_2;
	u32	rsvd2_3;
	u32	rsvd2_4;

	/* LUT1 view 3 */
	/* Various packet flags */
	u16	pkt_flags;
#define PA2FRM_IP_FLAG_IP_TYPE			0x8000
#define PA2FRM_IP_FLAG_V4			0x8000
#define PA2FRM_IP_FLAG_GRE			0x4000
#define PA2FRM_IP_FLAG_SCTP			0x2000
#define PA2FRM_IP_FLAG_TCP_DATA			0x1000
#define PA2FRM_IP_FLAG_OPTIONS			0x0800
#define PA2FRM_IP_FLAG_FRAG			0x0400
#define PA2FRM_IP_FLAG_CONTAIN_L4		0x0200
#define PA2FRM_IP_FLAG_HOP_LIMIT		0x0100
#define PA2FRM_IP_FLAG_IPSEC			0x0080

	u8	dscp;
	u8	rsvd3_1;
	u32	rsvd3_2;
	/* Common filed to indicate packet type */
	u8	pkt_type;
	/* Next Layer protocol */
	u8	protocol;
	/* Layer 4 source port number */
	u16	src_port;
	/* Layer 4 destination port number */
	u16	dst_port;
	/* virtual or physical link */
	u16	src_vc;
};

struct pa2_frm_com_l1_ipv6 {
	/* LUT1 view 1 */
	/* Source IP address */
	u32	src_ip0;
	u32	src_ip1;
	u32	src_ip2;
	u32	src_ip3;

	/* LUT1 view 2 */
	/* Destination IP address */
	u32	dst_ip0;
	u32	dst_ip1;
	u32	dst_ip2;
	u32	dst_ip3;

	/* LUT1 view 3 */
	/* Various packet flags */
	u16	pkt_flags;
#define PA2FRM_IP_FLAG_V6			0x4000
	u8	dscp;
	u8	rsvd8a;
	/* 20-bit Flow Label in the header */
	u32	flow_label;
	/* Common filed to indicate packet type */
	u8	pkt_type;
	/* Next Layer protocol */
	u8	protocol;
	/* Layer 4 source port number */
	u16	src_port;
	/* Layer 4 destination port number */
	u16	dst_port;
	/* virtual or physical link */
	u16	src_vc;
};

struct pa2_frm_com_l1_ipsec  {
	/* LUT1 view 1 */
	u32	rsvd1_1;
	u32	rsvd1_2;
	u32	rsvd1_3;
	u32	rsvd1_4;

	/* LUT1 view 2 */
	u32	rsvd2_1;
	u32	rsvd2_2;
	u32	rsvd2_3;
	u32	rsvd2_4;

	/* LUT1 view 3 */
	/* Various packet flags */
	u16	pkt_flags;
#define PA2FRM_IPSEC_FLAG_ESP			0x8000
#define PA2FRM_IPSEC_FLAG_AH			0x4000
	u16	rsvd3_1;
	/* SPI value */
	u32	spi;
	/* Common filed to indicate packet type */
	u8	pkt_type;
	u8	rsvd3_2;
	u16	rsvd3_3;
	u16	rsvd3_4;
	/* virtual or physical link */
	u16	src_vc;
};

enum {
	PA2FRM_CONFIG_COMMAND_RSVD	= 0,
	PA2FRM_CONFIG_COMMAND_ADDREP_LUT1,
	PA2FRM_CONFIG_COMMAND_DEL_LUT1,
	PA2FRM_CONFIG_COMMAND_ADDREP_LUT2,
	PA2FRM_CONFIG_COMMAND_DEL_LUT2,
	PA2FRM_CONFIG_COMMAND_CONFIG_PA,
	PA2FRM_CONFIG_COMMAND_REQ_STATS,
	PA2FRM_CONFIG_COMMAND_REQ_VERSION,
	PA2FRM_CONFIG_COMMAND_MULTI_ROUTE,
	PA2FRM_CONFIG_COMMAND_CRC_ENGINE,
	PA2FRM_CONFIG_COMMAND_CMD_SET,
	PA2FRM_CONFIG_COMMAND_CMD_USR_STATS,
	PA2FRM_CONFIG_COMMAND_CMD_SYS_CONFIG,
	PA2FRM_CONFIG_COMMAND_CMD_MULTI_CMDS
};

/* Command magic value */
#define PA2FRM_CONFIG_COMMAND_SEC_BYTE  0xce

/* Command return values */
enum {
	/* Must be 0 */
	PA2FRM_COMMAND_RESULT_SUCCESS = 0,
	/* Command magic value not found */
	PA2FRM_COMMAND_RESULT_NO_COMMAND_MAGIC,
	/* Invalid command identifier */
	PA2FRM_COMMAND_RESULT_INVALID_CMD,

	/* Add entry to LUT1 fails codes */

	/* Invalid type, custom or standard IP/ethernet */
	PA2FRM_COMMAND_RESULT_LUT1_TYPE_INVALID,
	/* Invalid LUT1 index (0-63) or no free indices available */
	PA2FRM_COMMAND_RESULT_LUT1_INDEX_INVALID,
	/* Sent a match packet to q0 on c1 or c2 - this is illegal. */
	PA2FRM_COMMAND_RESULT_LUT1_MATCH_DEST_INVALID,
	/* Previous match fwd info was somewhere in chunk domain */
	PA2FRM_COMMAND_RESULT_LUT1_NMATCH_INVALID,
	/* Invalid combination found in the key value */
	PA2FRM_COMMAND_RESULT_LUT1_INVALID_KEYS,

	/* Lut 2 entry warnings since the lut can be configured without pdsp */
	PA2FRM_COMMAND_RESULT_WARN_OVER_MAX_ENTRIES,
	PA2FRM_COMMAND_RESULT_WARN_NEGATIVE_ENTRY_COUNT,

	/* Lut 2 entry failures */

	/* LUT2 had a lookup and pending config */
	PA2FRM_COMMAND_RESULT_LUT2_ADD_BUSY,
	/* Not enough room in stats request packet for the reply */
	PA2FRM_COMMAND_RESULT_WARN_STATS_REPLY_SIZE,
	/* Command sent to PDSP which couldn't handle it */
	PA2FRM_COMMAND_RESULT_INVALID_DESTINATION,

	/* Add/Delete/Read entries to multi route table */

	/* Asked to use a free entry, but none found */
	PA2FRM_COMMAND_RESULT_MULTI_ROUTE_NO_FREE_ENTRIES,
	/* Illegal index value used */
	PA2FRM_COMMAND_RESULT_MULTI_ROUTE_INVALID_IDX,
	/* Illegal multi route mode used */
	PA2FRM_COMMAND_RESULT_MULTI_ROUTE_INVALID_MODE,

	/* Packet size didn't match command */
	PA2FRM_COMMAND_RESULT_INVALID_PKT_SIZE,

	/* Coustom and Command set index */

	/* Illegal Custom LUT1 index value used */
	PA2FRM_COMMAND_RESULT_INVALID_C1_CUSTOM_IDX,
	/* Illegal Custom LUT2 index value used */
	PA2FRM_COMMAND_RESULT_INVALID_C2_CUSTOM_IDX,
	/* Illegal Custom Cmd Set index value used */
	PA2FRM_COMMAND_RESULT_INVALID_CMDSET_IDX,
	/* Illegal User Stats Configuration */
	PA2FRM_COMMAND_RESULT_USR_STATS_INVALID_CONFIG
};

/* Destination (route) values */
/* Packets to Global CDMA */
#define PA2FRM_DEST_CDMA0			0
/* Packets to Local CDMA */
#define PA2FRM_DEST_CDMA1			1
/* Packets to Ethernet TX */
#define PA2FRM_DEST_ETHERNET1			2
/* Packets to Ethernet TX */
#define PA2FRM_DEST_ETHERNET2			3
/* Packets to Ethernet TX */
#define PA2FRM_DEST_ETHERNET3			4
/* Packets to Ethernet TX */
#define PA2FRM_DEST_ETHERNET4			5
/* Packets to Ethernet TX */
#define PA2FRM_DEST_ETHERNET5			6
/* Packets to Ethernet TX */
#define PA2FRM_DEST_ETHERNET6			7
/* Packets to Ethernet TX */
#define PA2FRM_DEST_ETHERNET7			8
/* Packets to Ethernet TX */
#define PA2FRM_DEST_ETHERNET8			9
/* Packets to Cluster Ingress 0 */
#define PA2FRM_DEST_INGRESS0			10
/* Packets to Cluster Ingress 1 */
#define PA2FRM_DEST_INGRESS1			11
/* Packets to Cluster Ingress 2 */
#define PA2FRM_DEST_INGRESS2			12
/* Packets to Cluster Ingress 3 */
#define PA2FRM_DEST_INGRESS3			13
/* Packets to Cluster Ingress 4 */
#define PA2FRM_DEST_INGRESS4			14
/* Packets to Cluster Post Processings */
#define PA2FRM_DEST_POST			15
/* Packets to Cluster Egress 0 */
#define PA2FRM_DEST_EGRESS0			16
/* Packets to Cluster Egress 1 */
#define PA2FRM_DEST_EGRESS1			17
/* Packets to Cluster Egress 2 */
#define PA2FRM_DEST_EGRESS2			18
/* Packets to Reasm Accelerator */
#define PA2FRM_DEST_REASM			19
/* Placeholder for model */
#define PA2FRM_DEST_ACE0			20
/* Placeholder for model */
#define PA2FRM_DEST_ACE1			21
/* Packets to Statsbloc */
#define PA2FRM_DEST_STATSBLOC			22

#define PA2FRM_DEST_PKTDMA			PA2FRM_DEST_CDMA0
#define PA2FRM_DEST_PKTDMA_LOC			PA2FRM_DEST_CDMA1
#define PA2FRM_DEST_ETH				PA2FRM_DEST_ETHERNET1

#define PA2FRM_DEST_DISCARD			0xff

/* Assigning names based on PDSP functions */
#define PA2FRM_DEST_PA_C1_0			PA2FRM_DEST_INGRESS0
#define PA2FRM_DEST_PA_C1_1			PA2FRM_DEST_INGRESS1
#define PA2FRM_DEST_PA_C1_2			PA2FRM_DEST_INGRESS4
#define PA2FRM_DEST_PA_C2			PA2FRM_DEST_INGRESS4
#define PA2FRM_DEST_PA_M_0			PA2FRM_DEST_POST
#define PA2FRM_DEST_PA_M_1			PA2FRM_DEST_EGRESS2

/* The default queue for packets that arrive at the PA and don't match in
 * classify1 (right at init time)
 */
#define PA2FRM_DEFAULT_INIT_Q			0x100

/* Ethertypes recognized by the firmware. */
#define PA2FRM_ETHERTYPE_IP			0x0800
#define PA2FRM_ETHERTYPE_IPV6			0x86dd
#define PA2FRM_ETHERTYPE_VLAN			0x8100
#define PA2FRM_ETHERTYPE_SPVLAN			0x88a8
#define PA2FRM_ETHERTYPE_MPLS			0x8847
#define PA2FRM_ETHERTYPE_MPLS_MULTI		0x8848

/* Next header type values  */
#define PA2FRM_HDR_MAC				0
#define PA2FRM_HDR_VLAN				1
#define PA2FRM_HDR_MPLS				2
#define PA2FRM_HDR_IPV4				3
#define PA2FRM_HDR_IPV6				4
#define PA2FRM_HDR_IPV6_EXT_HOP			5
#define PA2FRM_HDR_IPV6_EXT_ROUTE		6
#define PA2FRM_HDR_IPV6_EXT_FRAG		7
#define PA2FRM_HDR_IPV6_EXT_DEST		8
#define PA2FRM_HDR_GRE				9
#define PA2FRM_HDR_ESP				10
#define PA2FRM_HDR_ESP_DECODED			11
#define PA2FRM_HDR_AUTH				12
#define PA2FRM_HDR_CUSTOM_C1			13
/* A contrived header type used with custom SRIO to force a parse after looking
 * at only the RIO L0-L2
 */
#define PA2FRM_HDR_FORCE_LOOKUP			14
#define PA2FRM_HDR_SCTP				15
#define PA2FRM_HDR_UNKNOWN			16
#define PA2FRM_HDR_UDP				17
#define PA2FRM_HDR_UDP_LITE			18
#define PA2FRM_HDR_TCP				19
#define PA2FRM_HDR_GTPU				20
#define PA2FRM_HDR_ESP_DECODED_C2		21
#define PA2FRM_HDR_CUSTOM_C2			22

/* Command related definitions */
#define PA2FRM_CRC_FLAG_CRC_OFFSET_VALID	0x01
#define PA2FRM_CRC_FLAG_CRC_OFFSET_FROM_DESC	0x02
#define PA2FRM_CHKSUM_FALG_NEGATIVE		0x01

#define PA2_NEXT_ROUTE_PARAM_PRESENT		0x0001
#define PA2_NEXT_ROUTE_PROC_NEXT_CMD		0x0002
#define PA2_NEXT_ROUTE_PROC_MULTI_ROUTE		0x0004

/* PAFRM receive commands related definitions */

/* There are the following two groups of PAFRM receive commands:
 * PAFRM short commands which can be used as part of the routing info
 * PAFRM commands which can be used within a command set
 */
/* Dummy command */
#define PA2FRM_RX_CMD_NONE			0

/* short commands */
/* Execute a command set */
#define PA2FRM_RX_CMD_CMDSET			1
/* Insert up to two types at the current location */
#define PA2FRM_RX_CMD_INSERT			2
/* Increment the specific user-statistics chain */
#define PA2FRM_RX_CMD_USR_STATS			3
/* Increment User-defined Stats chain and  execute the command set */
#define PA2FRM_RX_CMD_CMDSET_USR_STATS		4

/* command set commands */
/* Specify the next route */
#define PA2FRM_RX_CMD_NEXT_ROUTE		11
/* CRC generation or verification */
#define PA2FRM_RX_CMD_CRC_OP			12
/* Copy data to the PS Info section */
#define PA2FRM_RX_CMD_COPY_DATA			13
/* Insert or patch packet data at the specific location */
#define PA2FRM_RX_CMD_PATCH_DATA		14
/* Remove the parsed packet header */
#define PA2FRM_RX_CMD_REMOVE_HDR		15
/* Remove the parsed packet tail */
#define PA2FRM_RX_CMD_REMOVE_TAIL		16
/* Duplicate packet to multiple destinations */
#define PA2FRM_RX_CMD_MULTI_ROUTE		17
/* Verify packet error based on error flags */
#define PA2FRM_RX_CMD_VERIFY_PKT_ERROR		18
/* Payload splitting */
#define PA2FRM_RX_CMD_SPLIT			19

/* define LUT1 entry types */
/* MAC/IP/IPSEC/ACL/FC */
#define PA2FRM_COM_ADD_LUT1_STANDARD		0
/* SRIO */
#define PA2FRM_COM_ADD_LUT1_SRIO		1
/* Custom LUT1 */
#define PA2FRM_COM_ADD_LUT1_CUSTOM		2
/* Standard entry with virtual Link */
#define PA2FRM_COM_ADD_LUT1_VLINK		3

/* LUT1 Entries */
/* if PA2_LUT1_INDEX_LAST_FREE is used then when the command returns,
 * the value of index will be replaced with the actual index used
 */
#define PA2FRM_HW_LUT1_ENTRIES			256
#define PA2FRM_LUT1_INDEX_LAST_FREE		PA2FRM_HW_LUT1_ENTRIES

struct pa2_frm_cmd_add_lut1 {
	/* LUT1 index. */
	u16	index;
	/* Custom or standard */
	u8	type;
	/* Valid only if type is custom */
	u8	cust_index;
	/* Virtual Link number if used */
	u16	vlink_num;
	/* entry statistics index (Flow Cache only) */
	u16	stats_index;

	/* LUT1 views */
	union {
		/* matching information for MAC/IP entry */
		struct pa2_frm_com_l1_mac	mac;
		struct pa2_frm_com_l1_custom	custom;
		/* matching information for IPv4 entry*/
		struct pa2_frm_com_l1_ipv4	ipv4;
		/* matching information for IPv6 entry */
		struct pa2_frm_com_l1_ipv6	ipv6;
		/* matching information for IPSEC entry */
		struct pa2_frm_com_l1_ipsec	ipsec;
	} u;

	/* Command header */
	/* Range High for bytes 44-45 */
	u16	range1_hi;
	/* Range High for bytes 42-43 */
	u16	range0_hi;
	/* Care Bits Word0 */
	u32	cbwords0;
	/* Care Bits Word1 */
	u32	cbwords1;
	/* BitMask for Bytes 0-1 */
	u16	bit_mask;
	/* Record priority "score", relative index */
	u16	priority;
	/* Routing information when a match is found */
	struct pa2_frm_forward match;

	/* Routing information when subsequent match fails - a fragmented
	 * packet orinner route
	 */
	struct pa2_frm_forward next_fail;
};

/* PA CRC Engine Instance Destinations */
enum pa2_crc_inst {
	/* CRC Engine between Ingress0, CDE0 and CED1 */
	PA2_CRC_INST_0_0 = 0,
	/* CRC Engine between Ingress1, CDE0 and CED1 */
	PA2_CRC_INST_1_0,
	/* Engine between Ingress4, CDE0 and CED1 */
	PA2_CRC_INST_4_0,
	/* Engine between Post, CDE0 and CED1 */
	PA2_CRC_INST_5_0,
	/* Engine between Egress0, CDE0 and CED1 */
	PA2_CRC_INST_6_0,
	/* Engine between Egress0, CDE1 and CED2 */
	PA2_CRC_INST_6_1,
	PA2_CRC_INST_MAX
};

#define PARAM_CRC_TABLE_SIZE			16

enum pa2_crc_size {
	/* 8-bit CRC */
	PA2_CRC_SIZE_8 = 0,
	/* 16-bit CRC */
	PA2_CRC_SIZE_16,
	/* 24-bit CRC */
	PA2_CRC_SIZE_24,
	/* 32-bit CRC */
	PA2_CRC_SIZE_32
};

struct pa2_crc_config {
	/* CRC configuration control info as defined below */
	u16			ctrl_bits;
	/* Set: Right shift CRC (b0 to b7), Clear: Left shift CRC (b7 to b0) */
#define PA2_CRC_CONFIG_RIGHT_SHIFT		0x0001
	/* Set: a 'NOT' operation is applied to the final CRC result */
#define PA2_CRC_CONFIG_INVERSE_RESULT		0x0002

	enum pa2_crc_size	size;
	/* CRC polynomial in the format
	 *	   of 0xabcdefgh. For example,
	 *	   x32+x28+x27+x26+x25+x23+x22+
	 *	   x20+x19+x18+x14+x13+x11+x10+
	 *	   x9+x8+x6+1 ==> 0x1EDC6F41
	 *	   x16+x15+x2+1 ==>0x80050000
	 */
	u32			polynomial;
	/* CRC initial value */
	u32			init_value;
};

struct pa2_frm_config_crc {
	/* Control bit maps as defined below */
	u8	ctrl_bitmap;
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

#define PA2FRM_CFG_CMD_STATUS_PROC		0
#define PA2FRM_CFG_CMD_STATUS_DONE		1

struct pa2_frm_command_cmd_hdr {
	/* Command Header of each command within the multiple command packet */
	u8	command;
	/* Offset to the next command, 0: Indicate the last command */
	u8	offset;
	/* general parameter used by host only */
	u16	com_id;
};

/* Commands to PA */
struct pa2_frm_command {
	/* Command Status (used by firmware only) */
	u8	status;
	/* index of the first targeted PDSP in a cluster */
	u8	pdsp_index;
	/* Returned to the host, ignored on entry to the PASS */
	u16	command_result;
	/* Used by the host to identify command results */
	u16	com_id;
	/* Command value */
	u8	command;
	/* Magic value */
	u8	magic;
	/* Returned in swInfo to identify packet as a command */
	u32	ret_context;
	/* Specifies the queue number for the message reply. 0xffff
	 * to toss the reply
	 */
	u16	reply_queue;
	/* Reply destination (host0, host1, discard are the only valid values)
	 */
	u8	reply_dest;
	/* Flow ID used to assign packet at reply */
	u8	flow_id;
	/* First word of the command */
	u32	cmd;
};

struct pa2_cmd_next_route {
	/* Routing control information as defined at routeCtrlInfo */
	u16	ctrl_bit_field;
	/* Packet destination as defined at pktDest */
	int	dest;
	/* For destination SRIO, specify the 5-bit packet type toward SRIO for
	 * destination EMAC, specify the EMAC control @ref emcOutputCtrlBits to
	 * the network
	 */
	u8	pkt_type_emac_ctrl;
	/* For host, SA or SRIO destinations, specifies return free descriptor
	 * setup
	 */
	u8	flow_id;
	/* For host, SA or SRIO destinations, specifies the dest queue */
	u16	queue;
		/* Placed in SwInfo0 for packets to host or SA */
	u32	sw_info_0;
	/* Placed in SwInfo1 for packets to the SA */
	u32	sw_info_1;
	/* Multi-route index. It is valid in the from-network direction only */
	u16	multi_route_index;
	/* Index of the first user-defined statistics to be updated. This
	 * optional parameter is valid in the to-network direction only
	 */
	u16	stats_index;
};

struct pa2_cmd_crc_op {
	/* CRC operation control information as defined at @ref crcOpCtrlInfo */
	u16	ctrl_bit_field;
	/* Byte location, from SOP/Protocol Header, where the CRC computation
	 * begins if frame type is not specified Byte location, from
	 * SOP/Protocol header, where the specific frame header begins if frame
	 * type is specified In to-network direction:
	 *	offset from SOP In from-network direction: offset from the
	 *	current parsed header
	 */
	u16	start_offset;
	/* Number of bytes covered by the CRC computation valid only if
	 * pa_CRC_OP_PAYLOAD_LENGTH_IN_HEADER is clear
	 */
	u16	len;
	/* Payload length field offset in the custom header */
	u16	len_offset;
	/* Payload length field mask */
	u16	len_mask;
	/* Payload length adjustment: valid only if
	 * PA2_CRC_OP_PAYLOAD_LENGTH_IN_HEADER is set
	 */
	u16	len_adjust;
	/* Offset from SOP/Protocol Header to the CRC field In to-network
	 * direction:
	 *	offset from SOP In from-network direction: offset from
	 *      the current parsed header
	 */
	u16	crc_offset;
	/* Size of CRC in bytes */
	u16	crc_size;
	/* Frame type @ref crcFrameTypes, valid if PA2_CRC_OP_CRC_FRAME_TYPE
	 * is set
	 */
	u16	frame_yype;
	/* CRC initial value */
	u32	init_value;
};

/* Transmit checksum configuration
 * pa_tx_chksum is used in the call to @ref Pa_formatTxRoute or @ref
 * Pa_formatTxCmd to create a tx command header that instructs the packet
 * accelerator sub-system to generate ones' complement checksums into network
 * packets. The checksums are typically used for TCP and UDP payload checksums
 * as well as IPv4 header checksums. In the case of TCP and UDP payload
 * checksums the pseudo header checksum must be pre-calculated and provided,
 * the sub-system does not calculate it.
 */
struct pa2_tx_chksum {
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

struct pa2_cmd_copy {
	/* Copy operation control information as defined at @ref copyCtrlInfo */
	u16	ctrl_bit_field;
	/* Offset from the start of current protocol header for the data copy
	 * to begin
	 */
	u16	src_offset;
	/* Offset from the top of the PSInfo for the data to be copied to */
	u16	dest_offset;
	/* Number of bytes to be copied */
	u16	num_bytes;
};

struct pa2_cmd_multi_route {
	/*  Multi-route set Index */
	u16	index;
};

/* PATCH Command in to-netweok direction */
#define PA2_MAX_PATCH_BYTES			16
/* PATCH Command within a command set */
#define PA2_MAX_RX_PATCH_BYTES			32
#define PA2_PATCH_OP_INSERT			0x0001
#define PA2_PATCH_OP_MAC_HDR			0x0002
#define PA2_PATCH_OP_DELETE			0x0004
struct pa2_patch_info {
	/* Patch operation control information as defined at @ref
	 * patchCtrlInfo
	 */
	u16	ctrl_bit_field;
	/* The number of bytes to be patched */
	u16	n_patch_bytes;
	/* The number of patch bytes in the patch cmd, must be >= to nPatchBytes
	 * and a multiple of 4 bytes
	 */
	u16	total_patch_size;
	/* Offset from the start of the packet for the patch to begin in the
	 * to-network direction Offset from the start of the current header
	 * for the patch to begin in the from-network direction
	 */
	u16	offset;
	/* Pointer to the patch data */
	u8	*patch_data;
};

/* pa_payload_info defines the packet payload information in the short format.
 * It is required by the Security Accelerator sub-system (SASS)
 *
 * pa_payload_info defines the packet parsing information in terms of payload
 * offset and payload length as described below
 * SRTP:      offset to the RTP header; RTP payload length including ICV
 * IPSEC AH:  offset to the Outer IP; IP payload length
 * IPSEC ESP: offset to the ESP header; ESP papload length including ICV
 */

struct pa2_payload_info  {
	/* The offset to where the SA packet parsing starts */
	u16	offset;
	/* The total length of the protocal payload to be processed by SA */
	u16	len;
	/* Optional supplement data such as the 32-bit CountC for some 3GPP
	 * operation modes
	 */
	u32	sup_data;
};

/* The maximum number of command sets supported */
#define PA2_MAX_CMD_SETS			64

#define PA2_OK					0
#define PA2_ERR_CONFIG				-10
#define PA2_INSUFFICIENT_CMD_BUFFER_SIZE	-11
#define PA2_INVALID_CMD_REPLY_DEST		-12

/* Command Set Command
 *
 * pa_cmd_set is used to specify the desired PA command set. The command set
 * command instructs the PASS to execute a list of commands after a LUT1 or
 * LUT2 match occurs. It is one of the command which can be embedded within
 * the @ref paRouteInfo_t.
 */
struct pa2_cmd_set {
	/*Command Set Index */
	u16	index;
};

struct pa2_cmd_tx_timestamp {
	/* Host queue for the tx timestamp reporting packet */
	u16	dest_queue;
	/* CPPI flow */
	u16	flow_id;
	/* 32 bit value returned in the descriptor */
	u32	sw_info0;
};

struct pa2_cmd_ip_frag {
	/* Offset to the IP header. */
	u16	ip_offset;
	/* Size of the maximum transmission unit (>= 68) */
	u16	mtu_size;
};

struct pa2_cmd_usr_stats {
	/* User-defined statistics index */
	u16	index;
};

struct pa2_cmd_set_usr_stats {
	/* Commad Set Index */
	u16	set_index;
	/* User-defined statistics index */
	u16	stats_index;
};

struct pa2_cmd_info {
	/* Specify the PA command code as defined at paCmdCode */
	u16	cmd;
	union {
		/* Specify nextRoute command specific parameters */
		struct pa2_cmd_next_route route;
		/* Specify Tx Checksum command specific parameters */
		struct pa2_tx_chksum	chksum;
		/* Specify CRC operation command specific parameters */
		struct pa2_cmd_crc_op	crc_op;
		/* Specify Copy command specific parameters */
		struct pa2_cmd_copy	copy;
		/* Specify Patch command specific parameters */
		struct pa2_patch_info	patch;
		/* Specify the payload info required by SA */
		struct pa2_payload_info	payload;
		/* Specify Command Set command specific parameters */
		struct pa2_cmd_set	cmd_set;
		/* Specify Multi-route cmd specific parameters */
		struct pa2_cmd_multi_route m_route;
		/* Report Tx Timestamp command specific parameters */
		struct pa2_cmd_tx_timestamp tx_ts;
		/* Specify IP fragmentation command specific parameters */
		struct pa2_cmd_ip_frag	ip_frag;
		/* User-defined stats command specific parameters */
		struct pa2_cmd_usr_stats usr_stats;
		struct pa2_cmd_set_usr_stats cmd_set_usr_stats;
	} params;
};

struct pa2_route_info {
	/* Packet destination as defined at pktDest */
	int	dest;
	/* For host, SA or SRIO destinations, specifies CPPI flow which defines
	 * free queues are used for receiving packets
	 */
	u8	flow_id;
	/* For host, SA or SRIO destinations, specifies the destination queue */
	u16	queue;
	int	m_route_index;
	/* For host, Multi-queue routing index
	 * (0 to (pa_MAX_MULTI_ROUTE_SETS - 1)) or pa_NO_MULTI_ROUTE if multi
	 * routing not used
	 */
	/* Placed in SwInfo0 for packets to host or SA; Placed in the PS Info
	 * for packets to SRIO
	 */
	u32	sw_info_0;
	/* Placed in SwInfo1 for packets to the SA; Placed in the PS Info for
	 * packets to SRIO
	 */
	u32	sw_info_1;
	int	custom_type;
	/* For CONTINUE_PARSE_LUT1/LUT2 only, specifies the custom type as
	 * defined at @ref customType
	 */
	/* For CONTINUE_PARSE_LUT1/LUT2 only, specifies the custom
	 * classification entry index
	 */
	u8	custom_index;
	/* For destination SRIO, specify the 5-bit packet type toward SRIO
	 * For destination HOST, EMAC, specify the EMAC control
	 * emcOutputCtrlBits to the network
	 */
	u8	pkt_type_emac_ctrl;
	/* Pointer to the Command info to be executed prior to the packet
	 * forwarding. Note only the following commands are supported within
	 * paRouteInfo_t - pa_CMD_PATCH_DATA (up to two bytes only) (LUT2 only)
	 *	   - pa_CMD_CMDSET
	 *	   - pa_CMD_USR_STATS
	 *	   - pa_CMD_CMDSET_AND_USR_STATS
	 */
	struct pa2_cmd_info *pcmd;
};

/* PA Route Info Valid Bit Definitions */
#define PA2_ROUTE_INFO_VALID_MROUTEINDEX	BIT(0)
#define PA2_ROUTE_INFO_VALID_PKTTYPE_EMAC	BIT(1)
#define PA2_ROUTE_INFO_VALID_PCMD		BIT(2)
#define PA2_ROUTE_INFO_VALID_PRIORITY_TYPE	BIT(3)

struct pa2_route_info2 {
	/* 32-bit valid bitmap corresponding to each optional field as defined
	 * at paRouteInfoValidBits
	 */
	u32	valid_bitmap;
	/* Packet destination as defined at pktDest */
	int	dest;
	/* For host, SA or SRIO destinations, specifies CPPI flow which defines
	 * free queues are used for receiving packets
	 */
	u8	flow_id;
	/* For host, SA or SRIO destinations, specifies the destination queue */
	u16	queue;
	int	m_route_index;
	/* For host, Multi-queue routing index
	 * (0 to (pa_MAX_MULTI_ROUTE_SETS - 1)) or pa_NO_MULTI_ROUTE if multi
	 * routing not used
	 */
	/* Placed in SwInfo0 for packets to host or SA; Placed in the PS Info
	 * for packets to SRIO
	 */
	u32	sw_info_0;
	/* Placed in SwInfo1 for packets to the SA; Placed in the PS Info for
	 * packets to SRIO
	 */
	u32	sw_info_1;
	/* For CONTINUE_PARSE_LUT1/LUT2 only, specifies the custom type as
	 * defined at @ref custom_type
	 */
	int	custom_type;
	/* For CONTINUE_PARSE_LUT1/LUT2 only, specifies the custom
	 * classification entry index
	 */
	u8	custom_index;
	/* For destination SRIO, specify the 5-bit packet type toward SRIO
	 * For destination HOST, EMAC, specify the EMAC control
	 * emcOutputCtrlBits to the network
	 */
	u8	pkt_type_emac_ctrl;
	/* Pointer to the Command info to be executed prior to the packet
	 * forwarding. Note only the following commands are
	 * supported within paRouteInfo_t
	 *   - pa_CMD_PATCH_DATA (up to two bytes only) (LUT2 only)
	 *   - pa_CMD_CMDSET
	 *   - pa_CMD_USR_STATS
	 *   - pa_CMD_CMDSET_AND_USR_STATS
	 */
	struct pa2_cmd_info *pcmd;
	/* validBitMap[t3]: For Host only, specify priority-based and/or
	 * interfcae-based routing mode as defined at paRoutePriIntf_e
	 */
	u8	priority_type;
	/* For EFLOW only, egress flow operation info (PAGG Gen2 only) */
	struct pa2_ef_op_info *ef_op;
};

struct pa2_cmd_reply {
	/* Packet destination, must be pa_DEST_HOST or PA2_DEST_DISCARD, see
	 * @ref pktDest
	 */
	int	dest;
	/*  Value placed in swinfo0 in reply packet */
	u32	reply_id;
	/*  Destination queue for destination PA2_DEST_HOST */
	u16	queue;
	u8	flow_id;
	/*  Flow ID used on command reply from PASS */
};

#endif /* KEYSTONE_PA2_FW_H */
