/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause) */
/*
 * PRUSS Remote Processor specific types
 *
 * Copyright (C) 2014-2020 Texas Instruments Incorporated - http://www.ti.com/
 *	Suman Anna <s-anna@ti.com>
 */

#ifndef _PRU_RPROC_H_
#define _PRU_RPROC_H_

/**
 * enum pruss_rsc_types - PRU specific resource types
 *
 * @PRUSS_RSC_INTRS: Resource holding information on PRU INTC configuration
 * @PRUSS_RSC_MAX: Indicates end of known/defined PRU resource types.
 *		   This should be the last definition.
 *
 * Introduce new vendor resource types before PRUSS_RSC_MAX.
 */
enum pruss_rsc_types {
	PRUSS_RSC_INTRS	= 1,
	PRUSS_RSC_MAX	= 2,
};

/**
 * struct pruss_event_chnl - PRU system events _to_ channel mapping
 * @event: number of the system event
 * @chnl: channel number assigned to a given @event
 *
 * PRU system events are mapped to channels, and these channels are mapped
 * to host interrupts. Events can be mapped to channels in a one-to-one or
 * many-to-one ratio (multiple events per channel), and channels can be
 * mapped to host interrupts in a one-to-one or many-to-one ratio (multiple
 * channels per interrupt).
 *
 */
struct pruss_event_chnl {
	s8 event;
	s8 chnl;
};

/**
 * struct fw_rsc_pruss_intrmap - custom/vendor resource to define PRU interrupts
 * @reserved: reserved field providing padding and alignment
 * @chnl_host_intr_map: array of PRU channels to host interrupt mappings
 * @event_chnl_map_size: number of event_channel mappings defined in
 *			 @event_chnl_map_addr
 * @event_chnl_map_addr: PRU device address of pointer to array of events to
 *			 channel mappings (struct pruss_event_chnl elements)
 *
 * PRU system events are mapped to channels, and these channels are mapped
 * to host interrupts. Events can be mapped to channels in a one-to-one or
 * many-to-one ratio (multiple events per channel), and channels can be
 * mapped to host interrupts in a one-to-one or many-to-one ratio (multiple
 * channels per interrupt).
 */
struct fw_rsc_pruss_intrmap {
	u16 reserved;
	s8 chnl_host_intr_map[10];
	u32 event_chnl_map_size;
	u32 event_chnl_map_addr;
};

#endif	/* _PRU_RPROC_H_ */
