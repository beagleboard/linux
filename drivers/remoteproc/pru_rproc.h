/*
 * PRUSS Remote Processor specific types
 *
 * Copyright (C) 2014-2016 Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 * * Neither the name Texas Instruments nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _PRU_REMOTEPROC_H_
#define _PRU_REMOTEPROC_H_

/**
 * enum pruss_rsc_types - PRU specific resource types
 *
 * @PRUSS_RSC_INTRS: Resource holding information on PRU PINTC configuration
 * @PRUSS_RSC_MAX: Indicates end of known/defined PRU resource types.
 *		   This should be the last definition.
 *
 * Introduce new custom resource types before PRUSS_RSC_MAX.
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
 * struct fw_rsc_custom_intrmap - custom resource to define PRU interrupts
 * @version: version number for the resource type
 * @chnl_host_intr_map: array of PRU channels to host interrupt mappings
 * @event_chnl_map_size: number of event_channel mappings defined in
 *			 @event_chnl_map
 * @event_chnl_map: pointer to array of events to channel mappings
 *
 * PRU system events are mapped to channels, and these channels are mapped
 * to host interrupts. Events can be mapped to channels in a one-to-one or
 * many-to-one ratio (multiple events per channel), and channels can be
 * mapped to host interrupts in a one-to-one or many-to-one ratio (multiple
 * channels per interrupt).
 *
 * @da is the device address of the interrupt controller, @channel_map is
 * used to specify to which channel, if any, an event is mapped, and @host_map
 * specifies to which host, if any, a channel is mapped.
 */
struct fw_rsc_custom_intrmap {
	u16 version;
	s8 chnl_host_intr_map[10];
	u32 event_chnl_map_size;
	struct pruss_event_chnl *event_chnl_map;
};

#endif	/* _PRU_REMOTEPROC_H_ */
