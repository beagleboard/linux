/*
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __ICSS_SWITCH_H
#define __ICSS_SWITCH_H

/* Basic Switch Parameters
 * Used to auto compute offset addresses on L3 OCMC RAM. Do not modify these
 * without changing firmware accordingly
 */
#define SWITCH_BUFFER_SIZE	(64 * 1024)	/* L3 buffer */
#define ICSS_BLOCK_SIZE		32		/* data bytes per BD */
#define BD_SIZE			4		/* byte buffer descriptor */

#define PORT_LINK_MASK		0x1
#define PORT_IS_HD_MASK		0x2

/* Physical Port queue size (number of BDs). Same for both ports */
#define QUEUE_1_SIZE		97	/* Network Management high */
#define QUEUE_2_SIZE		97	/* Network Management low */
#define QUEUE_3_SIZE		97	/* Protocol specific */
#define QUEUE_4_SIZE		97	/* NRT (IP,ARP, ICMP) */

/* Host queue size (number of BDs). Each BD points to data buffer of 32 bytes.
 * HOST PORT QUEUES can buffer up to 4 full sized frames per queue
 */
#define	HOST_QUEUE_1_SIZE	194	/* Protocol and VLAN priority 7 & 6 */
#define HOST_QUEUE_2_SIZE	194	/* Protocol mid */
#define HOST_QUEUE_3_SIZE	194	/* Protocol low */
#define HOST_QUEUE_4_SIZE	194	/* NRT (IP, ARP, ICMP) */

/* NRT Buffer descriptor definition
 * Each buffer descriptor points to a max 32 byte block and has 32 bit in size
 * to have atomic operation.
 * PRU can address bytewise into memory.
 * Definition of 32 bit descriptor is as follows
 *
 * Bits		Name			Meaning
 * =============================================================================
 * 0..7		Index		points to index in buffer queue, max 256 x 32
 *				byte blocks can be addressed
 * 8..12	Block_length	number of valid bytes in this specific block.
 *				Will be <=32 bytes on last block of packet
 * 13		More		"More" bit indicating that there are more blocks
 * 14		Shadow		indicates that "index" is pointing into shadow
 *				buffer
 * 15		TimeStamp	indicates that this packet has time stamp in
 *				separate buffer - only needed of PTCP runs on
 *				host
 * 16..17	Port		different meaning for ingress and egress,
 *				ingress Port=0 inidcates phy port 1 and Port = 1
 *				indicates phy port 2. Egress: 0 sends on phy
 *				port 1 and 1 sends on phy port 2. Port = 2 goes
 *				over MAC table look-up
 * 18..28	Length		11 bit of total packet length which is put into
 *				first BD only so that host access only one BD
 * 29		VlanTag		indicates that packet has Length/Type field of
 *				0x08100 with VLAN tag in following byte
 * 30		Broadcast	inidcates that packet goes out on both physical
 *				ports,  there will be two bd but only one buffer
 * 31		Error		indicates there was an error in the packet
 */
#define	PRUETH_BD_SHADOW_MASK		BIT(14)
#define	PRUETH_BD_SHADOW_SHIFT		14

#define	PRUETH_BD_PORT_MASK		GENMASK(17, 16)
#define PRUETH_BD_PORT_SHIFT		16

#define	PRUETH_BD_LENGTH_MASK		GENMASK(28, 18)
#define PRUETH_BD_LENGTH_SHIFT		18

#define	PRUETH_BD_BROADCAST_MASK	BIT(30)
#define PRUETH_BD_BROADCAST_SHIFT	30

#define	PRUETH_BD_ERROR_MASK		BIT(31)
#define PRUETH_BD_ERROR_SHIFT		31

/* The following offsets indicate which sections of the memory are used
 * for switch and EMAC internal tasks
 */
#define SWITCH_SPECIFIC_DRAM0_START_SIZE	0x100
#define SWITCH_SPECIFIC_DRAM0_START_OFFSET	0x1F00

#define SWITCH_SPECIFIC_DRAM1_START_SIZE	0x300
#define SWITCH_SPECIFIC_DRAM1_START_OFFSET	0x1D00

#define EMAC_SPECIFIC_DRAM_USAGE_SIZE		0x140
#define EMAC_SPECIFIC_DRAM_START_OFFSET		0x1EC0

#define SWITCH_SPECIFIC_SRAM_USAGE_SIZE		0x2010
/* same as EMAC. The BD offsets are common between EMAC and Switch */
#define SWITCH_SPECIFIC_SRAM_START_OFFSET	0x400

#define EMAC_SPECIFIC_SRAM_USAGE_SIZE		0x1900
#define EMAC_SPECIFIC_SRAM_START_OFFSET		0x0

/* General Purpose Statistics
 * These are present on both PRU0 and PRU1 DRAM
 */
/* base statistics offset */
#define STATISTICS_OFFSET	0x1F00
#define STAT_SIZE		0x8c

/* Offset for storing
 * 1. Storm Prevention Params
 * 2. PHY Speed Offset
 * 3. Port Status Offset
 * These are present on both PRU0 and PRU1
 */
/* 4 bytes */
#define STORM_PREVENTION_OFFSET		(STATISTICS_OFFSET + STAT_SIZE)
/* 4 bytes */
#define PHY_SPEED_OFFSET		(STATISTICS_OFFSET + STAT_SIZE + 4)
/* 1 byte */
#define PORT_STATUS_OFFSET		(STATISTICS_OFFSET + STAT_SIZE + 8)
/* 1 byte */
#define COLLISION_COUNTER		(STATISTICS_OFFSET + STAT_SIZE + 9)
/* 4 bytes */
#define RX_PKT_SIZE_OFFSET		(STATISTICS_OFFSET + STAT_SIZE + 10)
/* 4 bytes */
#define PORT_CONTROL_ADDR		(STATISTICS_OFFSET + STAT_SIZE + 14)
/* 6 bytes */
#define PORT_MAC_ADDR			(STATISTICS_OFFSET + STAT_SIZE + 18)
/* 1 byte */
#define RX_INT_STATUS_OFFSET		(STATISTICS_OFFSET + STAT_SIZE + 24)

/* DRAM1 Offsets for Switch */

/* Queue Descriptors */
/* 4 queue descriptors for port 2 (PRU0 xmt) */
#define P2_QUEUE_DESC_OFFSET		0x1EBC
/* 4 queue descriptors for port 1 (PRU1 xmt) */
#define P1_QUEUE_DESC_OFFSET		0x1E9C
/* 4 queue descriptors for port 0 (host receive) */
#define P0_QUEUE_DESC_OFFSET		0x1E7C
/* collision descriptor of port 2 */
#define P2_COL_QUEUE_DESC_OFFSET	0x1E74
/* collision descriptor of port 1 */
#define P1_COL_QUEUE_DESC_OFFSET	0x1E6C
/* collision descriptor of port 0 */
#define P0_COL_QUEUE_DESC_OFFSET	0x1E64
/* Collision Status Register, P0: bit 0 is pending flag, bit 1..2 inidicates
 * which queue,
 * P1: bit 8 is pending flag, 9..10 is queue number
 * p2: bit 16 is pending flag, 17..18 is queue number, remaining bits are 0.
 */
#define COLLISION_STATUS_ADDR		0x1E60

#define INTERFACE_MAC_ADDR	0x1E58 /* Interface MAC Address */
#define P2_MAC_ADDR		0x1E50 /* Port 2 MAC Address */
#define P1_MAC_ADDR		0x1E48 /* Port 1 MAC Address */
/* With dynamic configuration of queue size the offsets are variable.
 * For PRU to find which queue and descriptor needs to be served there
 * is a look-up table with index of port and queue number.
 * table definition to access queue size, buffer descriptor and buffer
 */

/* table offset for queue size:
 * 3 ports * 4 Queues * 1 byte offset = 24 bytes
 */
#define QUEUE_SIZE_ADDR			0x1E30
/* table offset for queue:
 * 3 ports * 4 Queues * 2 byte offset = 24 bytes
 */
#define QUEUE_OFFSET_ADDR		0x1E18
/* table offset for queue descriptors:
 * 3 ports * 4 Queues * 2 byte offset = 24 bytes
 */
#define QUEUE_DESCRIPTOR_OFFSET_ADDR	0x1E00
/* todo: remove this older definition */
#define QUEUE_SIZE_TBL_ADDR		0x1E00
/* table is filled by host before operation and structured in
 * 0x1E08: P0Q0 0x1E09: P0Q1 0x1E0A: P0Q2 0x1E0B: P0Q3
 * 0x1E04: P1Q0 0x1E05: P1Q1 0x1E06: P1Q2 0x1E07: P0Q3
 * 0x1E00: P0Q0 0x1E01: P0Q1 0x1E02: P0Q2 0x1E03: P0Q3
 */

/* 256 bytes of Configuratuin which also includes queue descriptors */
#define CONFIGURATION_OFFSET	0x1E00

/* Port 2 Rx Collision context 10 bytes */
#define COL_RX_CONTEXT_P2_OFFSET_ADDR	(COL_RX_CONTEXT_P1_OFFSET_ADDR + 12)
/* Port 1 Rx Collision context 10 bytes */
#define COL_RX_CONTEXT_P1_OFFSET_ADDR	(COL_RX_CONTEXT_P0_OFFSET_ADDR + 12)
/* Host Port Collision Context 10 bytes */
#define COL_RX_CONTEXT_P0_OFFSET_ADDR	(P2_Q4_RX_CONTEXT_OFFSET + 8)

/* Port 2 Rx Context */
#define P2_Q4_RX_CONTEXT_OFFSET		(P2_Q3_RX_CONTEXT_OFFSET + 8)
#define P2_Q3_RX_CONTEXT_OFFSET		(P2_Q2_RX_CONTEXT_OFFSET + 8)
#define P2_Q2_RX_CONTEXT_OFFSET		(P2_Q1_RX_CONTEXT_OFFSET + 8)
#define P2_Q1_RX_CONTEXT_OFFSET		RX_CONTEXT_P2_Q1_OFFSET_ADDR
#define RX_CONTEXT_P2_Q1_OFFSET_ADDR	(P1_Q4_RX_CONTEXT_OFFSET + 8)

/* Port 1 Rx Context */
#define P1_Q4_RX_CONTEXT_OFFSET		(P1_Q3_RX_CONTEXT_OFFSET + 8)
#define P1_Q3_RX_CONTEXT_OFFSET		(P1_Q2_RX_CONTEXT_OFFSET + 8)
#define P1_Q2_RX_CONTEXT_OFFSET		(P1_Q1_RX_CONTEXT_OFFSET + 8)
#define P1_Q1_RX_CONTEXT_OFFSET		RX_CONTEXT_P1_Q1_OFFSET_ADDR
#define RX_CONTEXT_P1_Q1_OFFSET_ADDR	(P0_Q4_RX_CONTEXT_OFFSET + 8)

/* Host Port Rx Context */
#define P0_Q4_RX_CONTEXT_OFFSET		(P0_Q3_RX_CONTEXT_OFFSET + 8)
#define P0_Q3_RX_CONTEXT_OFFSET		(P0_Q2_RX_CONTEXT_OFFSET + 8)
#define P0_Q2_RX_CONTEXT_OFFSET		(P0_Q1_RX_CONTEXT_OFFSET + 8)
#define P0_Q1_RX_CONTEXT_OFFSET		RX_CONTEXT_P0_Q1_OFFSET_ADDR
#define RX_CONTEXT_P0_Q1_OFFSET_ADDR	(COL_TX_CONTEXT_P2_Q1_OFFSET_ADDR + 8)

/* Port 2 Tx Collision Context */
#define COL_TX_CONTEXT_P2_Q1_OFFSET_ADDR (COL_TX_CONTEXT_P1_Q1_OFFSET_ADDR + 8)
/* Port 1 Tx Collision Context */
#define COL_TX_CONTEXT_P1_Q1_OFFSET_ADDR (P2_Q4_TX_CONTEXT_OFFSET + 8)

/* Port 2 */
#define P2_Q4_TX_CONTEXT_OFFSET		(P2_Q3_TX_CONTEXT_OFFSET + 8)
#define P2_Q3_TX_CONTEXT_OFFSET		(P2_Q2_TX_CONTEXT_OFFSET + 8)
#define P2_Q2_TX_CONTEXT_OFFSET		(P2_Q1_TX_CONTEXT_OFFSET + 8)
#define P2_Q1_TX_CONTEXT_OFFSET		TX_CONTEXT_P2_Q1_OFFSET_ADDR
#define TX_CONTEXT_P2_Q1_OFFSET_ADDR	(P1_Q4_TX_CONTEXT_OFFSET + 8)
/* Port 1 */
#define P1_Q4_TX_CONTEXT_OFFSET		(P1_Q3_TX_CONTEXT_OFFSET + 8)
#define P1_Q3_TX_CONTEXT_OFFSET		(P1_Q2_TX_CONTEXT_OFFSET + 8)
#define P1_Q2_TX_CONTEXT_OFFSET		(P1_Q1_TX_CONTEXT_OFFSET + 8)
#define P1_Q1_TX_CONTEXT_OFFSET		TX_CONTEXT_P1_Q1_OFFSET_ADDR
#define TX_CONTEXT_P1_Q1_OFFSET_ADDR	SWITCH_SPECIFIC_DRAM1_START_OFFSET

/* Shared RAM Offsets for Switch */

/* ICSS Shared RAM 12kB */
/* 6 bytes */
#define MULTICAST_FIRST_VALID_ADDR_OFFSET	(STATIC_MAC_TABLE_FWD_PORT2 + 256)
/* 6 bytes */
#define	MULTICAST_LAST_VALID_ADDR_OFFSET	(MULTICAST_FIRST_VALID_ADDR_OFFSET + 8)

#define STATIC_MAC_TABLE_FWD_PORT2	(STATIC_MAC_TABLE_RCV_PORT2  + 256)
#define STATIC_MAC_TABLE_RCV_PORT2	(STATIC_MAC_TABLE_FWD_PORT1  + 256)
#define STATIC_MAC_TABLE_FWD_PORT1	(STATIC_MAC_TABLE_RCV_PORT1  + 256)
#define STATIC_MAC_TABLE_RCV_PORT1	EOF_COL_BUFFER_BD

#define EOF_COL_BUFFER_BD	(P0_COL_BD_OFFSET + 3 * BD_SIZE * 48)
#define P2_COL_BD_OFFSET	(P1_COL_BD_OFFSET + BD_SIZE * 48)
#define P1_COL_BD_OFFSET	(P0_COL_BD_OFFSET + BD_SIZE * 48)
#define P0_COL_BD_OFFSET	EOF_48K_BUFFER_BD

/* DRAM Offsets for EMAC
 * Present on Both DRAM0 and DRAM1
 */

/* table offset for Port queue descriptors:
 * 1 ports * 4 Queues * 2 byte offset = 8 bytes
 */
#define Q4_TX_CONTEXT_OFFSET		(Q3_TX_CONTEXT_OFFSET + 8)
#define Q3_TX_CONTEXT_OFFSET		(Q2_TX_CONTEXT_OFFSET + 8)
#define Q2_TX_CONTEXT_OFFSET		(Q1_TX_CONTEXT_OFFSET + 8)
#define Q1_TX_CONTEXT_OFFSET		TX_CONTEXT_Q1_OFFSET_ADDR
#define TX_CONTEXT_Q1_OFFSET_ADDR	(PORT_QUEUE_DESC_OFFSET + 32)
/* 4 queue descriptors for port tx. 32 bytes */
#define PORT_QUEUE_DESC_OFFSET		EMAC_SPECIFIC_DRAM_START_OFFSET

/* Shared RAM offsets for EMAC */

/* Queue Descriptors */

/* 4 queue descriptors for port 0 (host receive). 32 bytes */
#define HOST_QUEUE_DESC_OFFSET		(HOST_QUEUE_SIZE_ADDR + 16)

/* table offset for queue size:
 * 3 ports * 4 Queues * 1 byte offset = 12 bytes
 */
#define HOST_QUEUE_SIZE_ADDR		(HOST_QUEUE_OFFSET_ADDR + 8)
/* table offset for queue:
 * 4 Queues * 2 byte offset = 8 bytes
 */
#define HOST_QUEUE_OFFSET_ADDR		(HOST_QUEUE_DESCRIPTOR_OFFSET_ADDR + 8)
/* table offset for Host queue descriptors:
 * 1 ports * 4 Queues * 2 byte offset = 8 bytes
 */
#define HOST_QUEUE_DESCRIPTOR_OFFSET_ADDR	(HOST_Q4_RX_CONTEXT_OFFSET + 8)

/* Host Port Rx Context */
#define HOST_Q4_RX_CONTEXT_OFFSET	(HOST_Q3_RX_CONTEXT_OFFSET + 8)
#define HOST_Q3_RX_CONTEXT_OFFSET	(HOST_Q2_RX_CONTEXT_OFFSET + 8)
#define HOST_Q2_RX_CONTEXT_OFFSET	(HOST_Q1_RX_CONTEXT_OFFSET + 8)
#define HOST_Q1_RX_CONTEXT_OFFSET	EOF_48K_BUFFER_BD

/* Shared RAM offsets for both Switch and EMAC */

/* allow for max 48k switch buffer which spans the descriptors up to 0x1800 6kB */
#define EOF_48K_BUFFER_BD	(P0_BUFFER_DESC_OFFSET + HOST_BD_SIZE + PORT_BD_SIZE)

#define HOST_BD_SIZE		((HOST_QUEUE_1_SIZE + HOST_QUEUE_2_SIZE + HOST_QUEUE_3_SIZE + HOST_QUEUE_4_SIZE) * BD_SIZE)
#define PORT_BD_SIZE		((QUEUE_1_SIZE + QUEUE_2_SIZE + QUEUE_3_SIZE + QUEUE_4_SIZE) * 2 * BD_SIZE)

#define P2_Q4_BD_OFFSET		(P2_Q3_BD_OFFSET + QUEUE_3_SIZE * BD_SIZE)
#define P2_Q3_BD_OFFSET		(P2_Q2_BD_OFFSET + QUEUE_2_SIZE * BD_SIZE)
#define P2_Q2_BD_OFFSET		(P2_Q1_BD_OFFSET + QUEUE_1_SIZE * BD_SIZE)
#define P2_Q1_BD_OFFSET		(P1_Q4_BD_OFFSET + QUEUE_4_SIZE * BD_SIZE)
#define P1_Q4_BD_OFFSET		(P1_Q3_BD_OFFSET + QUEUE_3_SIZE * BD_SIZE)
#define P1_Q3_BD_OFFSET		(P1_Q2_BD_OFFSET + QUEUE_2_SIZE * BD_SIZE)
#define P1_Q2_BD_OFFSET		(P1_Q1_BD_OFFSET + QUEUE_1_SIZE * BD_SIZE)
#define P1_Q1_BD_OFFSET		(P0_Q4_BD_OFFSET + HOST_QUEUE_4_SIZE * BD_SIZE)
#define P0_Q4_BD_OFFSET		(P0_Q3_BD_OFFSET + HOST_QUEUE_3_SIZE * BD_SIZE)
#define P0_Q3_BD_OFFSET		(P0_Q2_BD_OFFSET + HOST_QUEUE_2_SIZE * BD_SIZE)
#define P0_Q2_BD_OFFSET		(P0_Q1_BD_OFFSET + HOST_QUEUE_1_SIZE * BD_SIZE)

#define P0_Q1_BD_OFFSET		P0_BUFFER_DESC_OFFSET
#define	P0_BUFFER_DESC_OFFSET	SWITCH_SPECIFIC_SRAM_START_OFFSET

/* Memory Usage of L3 OCMC RAM */

/* L3 64KB Memory - mainly buffer Pool */
/* put collision buffer at end of L3 memory.
 * Simplifies PRU coding to be on same memory as queue buffer
 */
/* 1536 byte collision buffer for port 2 send queue */
#define P2_COL_BUFFER_OFFSET	0xFA00
/* 1536 byte collision buffer for port 1 send queue */
#define P1_COL_BUFFER_OFFSET	0xF400
/* 1536 byte collision buffer for port 0 send queue */
#define P0_COL_BUFFER_OFFSET	0xEE00

#define END_OF_BUFFER_POOL	(P2_Q4_BUFFER_OFFSET + QUEUE_4_SIZE * ICSS_BLOCK_SIZE)
#define P2_Q4_BUFFER_OFFSET	(P2_Q3_BUFFER_OFFSET + QUEUE_3_SIZE * ICSS_BLOCK_SIZE)
#define P2_Q3_BUFFER_OFFSET	(P2_Q2_BUFFER_OFFSET + QUEUE_2_SIZE * ICSS_BLOCK_SIZE)
#define P2_Q2_BUFFER_OFFSET	(P2_Q1_BUFFER_OFFSET + QUEUE_1_SIZE * ICSS_BLOCK_SIZE)
#define P2_Q1_BUFFER_OFFSET	(P1_Q4_BUFFER_OFFSET + QUEUE_4_SIZE * ICSS_BLOCK_SIZE)
#define P1_Q4_BUFFER_OFFSET	(P1_Q3_BUFFER_OFFSET + QUEUE_3_SIZE * ICSS_BLOCK_SIZE)
#define P1_Q3_BUFFER_OFFSET	(P1_Q2_BUFFER_OFFSET + QUEUE_2_SIZE * ICSS_BLOCK_SIZE)
#define P1_Q2_BUFFER_OFFSET	(P1_Q1_BUFFER_OFFSET + QUEUE_1_SIZE * ICSS_BLOCK_SIZE)
#define P1_Q1_BUFFER_OFFSET	(P0_Q4_BUFFER_OFFSET + HOST_QUEUE_4_SIZE * ICSS_BLOCK_SIZE)
#define P0_Q4_BUFFER_OFFSET	(P0_Q3_BUFFER_OFFSET + HOST_QUEUE_3_SIZE * ICSS_BLOCK_SIZE)
#define P0_Q3_BUFFER_OFFSET	(P0_Q2_BUFFER_OFFSET + HOST_QUEUE_2_SIZE * ICSS_BLOCK_SIZE)
#define P0_Q2_BUFFER_OFFSET	(P0_Q1_BUFFER_OFFSET + HOST_QUEUE_1_SIZE * ICSS_BLOCK_SIZE)
#define P0_Q1_BUFFER_OFFSET	0x0000

#endif /* __ICSS_SWITCH_H */
