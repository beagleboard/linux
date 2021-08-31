/* rtwlan_io.h
 *
 * Copyright (C) 2006      Daniel Gregorek <dxg@gmx.de>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef RTWLAN_IO
#define RTWLAN_IO

#include <rtnet_chrdev.h>

#define RTWLAN_TXMODE_RAW 0
#define RTWLAN_TXMODE_ACK 1
#define RTWLAN_TXMODE_MCAST 2

#define ENORTWLANDEV 0xff08

struct rtwlan_cmd {
	struct rtnet_ioctl_head head;

	union {
		struct {
			unsigned int bitrate;
			unsigned int channel;
			unsigned int retry;
			unsigned int txpower;
			unsigned int mode;
			unsigned int autoresponder;
			unsigned int dropbcast;
			unsigned int dropmcast;
			unsigned int bbpsens;
		} set;

		struct {
			unsigned int address;
			unsigned int value;
		} reg;

		struct {
			int ifindex;
			unsigned int flags;
			unsigned int bitrate;
			unsigned int channel;
			unsigned int retry;
			unsigned int txpower;
			unsigned int bbpsens;
			unsigned int mode;
			unsigned int autoresponder;
			unsigned int dropbcast;
			unsigned int dropmcast;
			unsigned int rx_packets;
			unsigned int tx_packets;
			unsigned int tx_retry;
		} info;
	} args;
};

#define RTNET_IOC_TYPE_RTWLAN 8

#define IOC_RTWLAN_IFINFO                                                      \
	_IOWR(RTNET_IOC_TYPE_RTWLAN, 0 | RTNET_IOC_NODEV_PARAM,                \
	      struct rtwlan_cmd)

#define IOC_RTWLAN_BITRATE _IOWR(RTNET_IOC_TYPE_RTWLAN, 1, struct rtwlan_cmd)

#define IOC_RTWLAN_CHANNEL _IOWR(RTNET_IOC_TYPE_RTWLAN, 2, struct rtwlan_cmd)

#define IOC_RTWLAN_TXPOWER _IOWR(RTNET_IOC_TYPE_RTWLAN, 3, struct rtwlan_cmd)

#define IOC_RTWLAN_RETRY _IOWR(RTNET_IOC_TYPE_RTWLAN, 4, struct rtwlan_cmd)

#define IOC_RTWLAN_TXMODE _IOWR(RTNET_IOC_TYPE_RTWLAN, 5, struct rtwlan_cmd)

#define IOC_RTWLAN_DROPBCAST _IOWR(RTNET_IOC_TYPE_RTWLAN, 6, struct rtwlan_cmd)

#define IOC_RTWLAN_DROPMCAST _IOWR(RTNET_IOC_TYPE_RTWLAN, 7, struct rtwlan_cmd)

#define IOC_RTWLAN_REGREAD _IOWR(RTNET_IOC_TYPE_RTWLAN, 8, struct rtwlan_cmd)

#define IOC_RTWLAN_REGWRITE _IOWR(RTNET_IOC_TYPE_RTWLAN, 9, struct rtwlan_cmd)

#define IOC_RTWLAN_BBPWRITE _IOWR(RTNET_IOC_TYPE_RTWLAN, 10, struct rtwlan_cmd)

#define IOC_RTWLAN_BBPREAD _IOWR(RTNET_IOC_TYPE_RTWLAN, 11, struct rtwlan_cmd)

#define IOC_RTWLAN_BBPSENS _IOWR(RTNET_IOC_TYPE_RTWLAN, 12, struct rtwlan_cmd)

#define IOC_RTWLAN_AUTORESP _IOWR(RTNET_IOC_TYPE_RTWLAN, 13, struct rtwlan_cmd)

#endif
