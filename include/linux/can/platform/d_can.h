/*
 * D_CAN controller driver platform header
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Bosch D_CAN controller is compliant to CAN protocol version 2.0 part A and B.
 * Bosch D_CAN user manual can be obtained from:
 * http://www.semiconductors.bosch.de/media/en/pdf/ipmodules_1/can/
 * d_can_users_manual_111.pdf
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

#ifndef __CAN_PLATFORM_TI_D_CAN_H__
#define __CAN_PLATFORM_TI_D_CAN_H__

/**
 * struct d_can_platform_data - DCAN Platform Data
 *
 * @num_of_msg_objs:	Number of message objects
 * @dma_support:	DMA support is required/not
 * @ram_init:		DCAN RAM initialization
 *
 * Platform data structure to get all platform specific settings.
 * this structure also accounts the fact that the IP may have different
 * RAM and mailbox offsets for different SOC's
 */
struct d_can_platform_data {
	u32 num_of_msg_objs;
	bool dma_support;
	void (*ram_init) (unsigned int, unsigned int);
};
#endif
