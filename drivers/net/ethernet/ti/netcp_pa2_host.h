/*
 * Copyright (C) 2012-2015 Texas Instruments Incorporated
 * Authors:	Murali Karicheri (Ported to v4.1.x kernel)
 *		Hao Zhang (PA2 Initial version of the driver)
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

#ifndef NETCP_PA2_HOST_H
#define NETCP_PA2_HOST_H

#include "netcp_pa_host.h"

#define PAHO2_CONFIGURE		4

#define PAHO2_PACFG_CMD	(((u32)PAHO2_CONFIGURE << 5) << 24)

/* Sets the optional flags of the CRC/Checksum command */
#define PAHO2_CHKCRC_SET_CTRL(x, v) \
	PAHO_SET_BITFIELD((x)->word0, (v), 16, 4)

/* Sets the size of the crc in bytes */
#define PAHO2_CHKCRC_SET_CRCSIZE(x, v) \
	PAHO_SET_BITFIELD((x)->word0, (v), 8,  8)

/* Sets the start offset of the checksum/crc */
#define PAHO2_CHKCRC_SET_START(x, v) \
	PAHO_SET_BITFIELD((x)->word0, (v), 0,  8)

/* Sets the initial value of the 32-bit crc */
#define PAHO2_CHKCRC_SET_INITVAL32(x, v)	(((x)->word2) = (v))

#endif /* NETCP_PA2_HOST_H */
