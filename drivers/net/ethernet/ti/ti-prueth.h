/*
 * PRU Ethernet driver
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com
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

#ifndef __NET_TI_PRUETH_H
#define __NET_TI_PRUETH_H

#define PRUETH_NUMQUEUES	5

/**
 * struct prueth_packet_info - Info about a packet in buffer
 * @shadow: this packet is stored in the collision queue
 * @port: port packet is on
 * @length: length of packet
 * @broadcast: this packet is a broadcast packet
 * @error: this packet has an error
 */
struct prueth_packet_info {
	bool shadow;
	unsigned int port;
	unsigned int length;
	bool broadcast;
	bool error;
};

/* NRT Queue Definition
 *
 * Each port has up to 4 queues with variable length. The queue is processed
 * as ring buffer with read and write pointer. Both pointer are address
 * pointers and increment by 4 for each buffer descriptor/position.
 * Queue has a length defined in constants and a status.
 * Status is defined as described below
 *
 * Bits		Name			Meaning
 * =============================================================================
 * 0		Busy_M			This queue is busy by the master port
 *					which is the PRU receiving packets from
 *					the master
 * 1		Collision		Slave is/has written into shadow queue,
 *					both descriptors and data.
 * 2		Overflow		there was not enough space to write to
 *					queue and packet was discarded
 * 4..7		Reserved		reserved
 *
 * There is busy slave flag in different byte address to grant access to queue
 * to master in case of simultaneous access. Host will always be slave in
 * this case. The PRU which is sending the packet on PHY port will be the
 * master. When both PRUs wants to write to host queues PRU0 is master and
 * PRU1 is slave.
 *
 * Bits		Name			Meaning
 * =============================================================================
 * 0		Busy_S			This queue is busy by the master port
 *					which is the PRU receiving packets from
 *					the master
 * 1..7		Reserved
 *
 * Length is the number of 32 byte blocks per queue. max_fill_level tells the
 * minimum distance between write and read pointer. over_flow_cnt tells how
 * many times the write pointer runs into the read_pointer.
 */
struct prueth_queue_desc {
	u16 rd_ptr;	/* read BD offset */
	u16 wr_ptr;	/* write BD offset */
	u8 busy_s;
	u8 status;
	u8 max_fill_level;
	u8 overflow_cnt;
} __packed;

/**
 * struct prueth_queue - Information about queue in memory
 * @buffer_offset: buffers offset in OCMC RAM
 * @queue_desc_offset: queue descriptor offset in DRAM1 or Shared RAM
 * @buffer_desc_offset: buffer descriptors offset in Shared RAM
 * @buffer_desc_end: end address buffer descriptors in Shared RAM
 */
struct prueth_queue_info {
	u16 buffer_offset;
	u16 queue_desc_offset;
	u16 buffer_desc_offset;
	u16 buffer_desc_end;
} __packed;

struct queue {
	unsigned int queue_desc_offset;
	/* queue size in bytes */
	unsigned int queue_size;

	unsigned int buffer_offset;
	/* buffer descriptors offset in Shared RAM */
	unsigned int buffer_desc_offset;
	/* number of buffer descriptors */
	unsigned int buffer_desc_count;
};

/**
 * struct port_params - Port parameters
 * @queue: information for each queue for this port
 */
struct port_params {
	/**Queues per port*/
	struct queue queue[PRUETH_NUMQUEUES];
};

/**
 * struct port_statistics - Statistics structure for capturing statistics on PRUs
 * @tx_bcast: Number of broadcast packets sent
 * @tx_mcast:Number of multicast packets sent
 * @tx_ucast:Number of unicast packets sent
 *
 * @tx_octets:Number of undersized frames rcvd
 *
 * @rx_bcast:Number of broadcast packets rcvd
 * @rx_mcast:Number of multicast packets rcvd
 * @rx_ucast:Number of unicast packets rcvd
 *
 * @rx_octets:Number of Rx packets
 *
 * @late_coll:Number of late collisions(Half Duplex)
 * @single_coll:Number of single collisions (Half Duplex)
 * @multi_coll:Number of multiple collisions (Half Duplex)
 * @excess_coll:Number of excess collisions(Half Duplex)
 *
 * @tx_hwq_overflow:Hardware Tx Queue (on PRU) over flow count
 * @rx_misalignment_frames:Number of non multiple of 8 byte frames rcvd
 * @stormprev_counter:Number of packets dropped because of Storm Prevention
 * @mac_rxerror:Number of MAC receive errors
 * @sfd_error:Number of invalid SFD
 * @def_tx:Number of transmissions deferred
 * @mac_txerror:Number of MAC transmit errors
 * @rx_oversized_frames:Number of oversized frames rcvd
 * @rx_undersized_frames:Number of undersized frames rcvd
 * @rx_crc_frames:Number of CRC error frames rcvd
 * @dropped_packets:Number of packets dropped due to a link down on opposite port
 *
 * @tx64byte:Number of 64 byte packets sent
 * @tx65_127byte:Number of 65-127 byte packets sent
 * @tx128_255byte:Number of 128-255 byte packets sent
 * @tx256_511byte:Number of 256-511 byte packets sent
 * @tx512_1023byte:Number of 512-1023 byte packets sent
 * @tx1024byte:Number of 1024 and larger size packets sent

 * @rx64byte:Number of 64 byte packets rcvd
 * @rx65_127byte:Number of 65-127 byte packets rcvd
 * @rx128_255byte:Number of 128-255 byte packets rcvd
 * @rx256_511byte:Number of 256-511 byte packets rcvd
 * @rx512_1023byte:Number of 512-1023 byte packets rcvd
 * @rx1024byte:Number of 1024 and larger size packets rcvd
 *
 * @sqe_test_error: Number of MAC receive errors
 * @u32 cs_error: Number of carrier sense errors
 *
 * The fields here are aligned here so that it's consistent
 * with the memory layout in PRU DRAM, this is to facilitate easy
 * memcpy. Don't change the order of the fields.
 */
struct port_statistics {
	u32 tx_bcast;
	u32 tx_mcast;
	u32 tx_ucast;

	u32 tx_octets;

	u32 rx_bcast;
	u32 rx_mcast;
	u32 rx_ucast;

	u32 rx_octets;

	u32 late_coll;
	u32 single_coll;
	u32 multi_coll;
	u32 excess_coll;

	u32 tx_hwq_overflow;
	u32 rx_misalignment_frames;
	u32 stormprev_counter;
	u32 mac_rxerror;
	u32 sfd_error;
	u32 def_tx;
	u32 mac_txerror;
	u32 rx_oversized_frames;
	u32 rx_undersized_frames;
	u32 rx_crc_frames;
	u32 dropped_packets;

	u32 tx64byte;
	u32 tx65_127byte;
	u32 tx128_255byte;
	u32 tx256_511byte;
	u32 tx512_1023byte;
	u32 tx1024byte;

	u32 rx64byte;
	u32 rx65_127byte;
	u32 rx128_255byte;
	u32 rx256_511byte;
	u32 rx512_1023byte;
	u32 rx1024byte;

	u32 sqe_test_error;

	u32 cs_error;
} __packed;

#endif /* __NET_TI_PRUETH_H */
