/*
 * Keystone NetCP PA (Packet Accelerator)/ Security Accelerator PDSP
 * Host packet handler/formatter definitions
 *
 * Copyright (C) 2012-2015 Texas Instruments Incorporated
 * Authors:	Murali Karicheri (Ported to v4.1.x kernel)
 *		Sandeep Paulraj (Initial version of the driver)
 *
 * Other contributors:	Reece Pollack (Maintenance)
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

#ifndef NETCP_PA_HOST_H
#define NETCP_PA_HOST_H

#define PAHO_PAMOD_CMPT_CHKSUM	0
#define PAHO_PAMOD_CMPT_CRC		1
#define PAHO_PAMOD_NROUTE		3
#define PAHO_PAMOD_REPORT_TIMESTAMP	6
#define PAHO_PAMOD_GROUP_7		7
#define PAHO_PAMOD_DUMMY		PAHO_PAMOD_GROUP_7
#define PAHO_SA_SHORT_INFO		1

#define PAHO_READ_BITFIELD(a, b, c)	(((a) >> (b)) & ((1UL << (c)) - 1))

#define PAHO_SET_BITFIELD(a, x, b, c)	((a) &= ~(((1UL << (c)) - 1) << (b)), \
				(a) |= (((x) & ((1UL << (c)) - 1)) << (b)))

#define PAHO_SET_CMDID(x, v) \
		PAHO_SET_BITFIELD((x)->word0, (v), 29, 3)

/**
 *  @ingroup paho_if_structures
 *  @brief  pahoCmdInfo_t defines the general short command information
 *
 */
struct paho_cmd_info {
	/* Control block word 0 */
	u32	word0;
};

struct paho_long_info {
	/* Control block word 0 */
	u32   word0;
	/* Control block word 1 */
	u32   word1;
	/* Control block word 2 */
	u32   word2;
	/* Control block word 3 */
	u32   word3;
	/* Control block word 4 */
	u32   word4;
};

/* Extract the offset to the level 5 header */
#define PAHO_LINFO_READ_L5_OFFSET(x) \
		PAHO_READ_BITFIELD((x)->word2, 8, 8)

struct paho_next_route {
	u32  word0;
	u32  sw_info0;
	u32  sw_info1;
	u32  word1;
};

/* Sets the E bit which indicates the extended
 * parameters (packet type) are present for SRIO
 */
#define PAHO_SET_E(x, v)	PAHO_SET_BITFIELD((x)->word0, (v), 27, 1)

/* Sets the destination of the route defined */
#define PAHO_SET_DEST(x, v)	PAHO_SET_BITFIELD((x)->word0, (v), 24, 3)

/* Specifies the flow to use for packets sent to the host */
#define PAHO_SET_FLOW(x, v)	PAHO_SET_BITFIELD((x)->word0, (v), 16, 8)

/* Specifies the queue to use for packets send to the host */
#define PAHO_SET_QUEUE(x, v)   PAHO_SET_BITFIELD((x)->word0, (v), 0,  16)

/* Specifies the packet type to use for packets send to the SRIO */
#define PAHO_SET_PKTTYPE(x, v) PAHO_SET_BITFIELD((x)->word1, (v), 24, 8)

struct paho_com_chk_crc {
	/* PAHO_chksum_command_macros */
	u32	word0;
	/* PAHO_chksum_command_macros */
	u32	word1;
	/* PAHO_chksum_command_macros */
	u32	word2;
};

/* Sets the negative 0 flag - if set a
 * checksum computed as 0 will be sent as 0xffff
 */
#define PAHO_CHKCRC_SET_NEG0(x, v) \
				PAHO_SET_BITFIELD((x)->word0, (v), 23, 1)

/* Sets the optional flags of the CRC/Checksum command */
#define PAHO_CHKCRC_SET_CTRL(x, v) \
				PAHO_SET_BITFIELD((x)->word0, (v), 16, 8)

/* Sets the start offset of the checksum/crc */
#define PAHO_CHKCRC_SET_START(x, v) \
				PAHO_SET_BITFIELD((x)->word0, (v), 0, 16)

/* Sets the length of the checksum/crc */
#define PAHO_CHKCRC_SET_LEN(x, v) \
				PAHO_SET_BITFIELD((x)->word1, (v), 16, 16)

/* Sets the offset to where to paste the checksum/crc into the packet */
#define PAHO_CHKCRC_SET_RESULT_OFF(x, v) \
				PAHO_SET_BITFIELD((x)->word1, (v), 0, 16)

/* Sets the initial value of the checksum/crc */
#define PAHO_CHKCRC_SET_INITVAL(x, v) \
				PAHO_SET_BITFIELD((x)->word2, (v), 16, 16)

struct paho_report_timestamp {
	u32	word0;
	u32	sw_info0;
};

/* Specifies the flow to use for report packets sent to the host */
#define PAHO_SET_REPORT_FLOW(x, v) PAHO_SET_BITFIELD((x)->word0, (v), 16, 8)

/* Specifies the queue to use for report packets send to the host */
#define PAHO_SET_REPORT_QUEUE(x, v)	\
				PAHO_SET_BITFIELD((x)->word0, (v), 0, 16)
#endif /* NETCP_PAHOST_H */
