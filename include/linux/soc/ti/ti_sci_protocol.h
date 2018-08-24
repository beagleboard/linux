/*
 * Texas Instruments System Control Interface Protocol
 *
 * Copyright (C) 2015-2016 Texas Instruments Incorporated - http://www.ti.com/
 *	Nishanth Menon
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __TISCI_PROTOCOL_H
#define __TISCI_PROTOCOL_H

/**
 * struct ti_sci_version_info - version information structure
 * @abi_major:	Major ABI version. Change here implies risk of backward
 *		compatibility break.
 * @abi_minor:	Minor ABI version. Change here implies new feature addition,
 *		or compatible change in ABI.
 * @firmware_revision:	Firmware revision (not usually used).
 * @firmware_description: Firmware description (not usually used).
 */
struct ti_sci_version_info {
	u8 abi_major;
	u8 abi_minor;
	u16 firmware_revision;
	char firmware_description[32];
};

struct ti_sci_handle;

/**
 * struct ti_sci_core_ops - SoC Core Operations
 * @reboot_device: Reboot the SoC
 *		Returns 0 for successful request(ideally should never return),
 *		else returns corresponding error value.
 */
struct ti_sci_core_ops {
	int (*reboot_device)(const struct ti_sci_handle *handle);
};

/**
 * struct ti_sci_dev_ops - Device control operations
 * @get_device: Command to request for device managed by TISCI
 *		Returns 0 for successful exclusive request, else returns
 *		corresponding error message.
 * @idle_device: Command to idle a device managed by TISCI
 *		Returns 0 for successful exclusive request, else returns
 *		corresponding error message.
 * @put_device:	Command to release a device managed by TISCI
 *		Returns 0 for successful release, else returns corresponding
 *		error message.
 * @is_valid:	Check if the device ID is a valid ID.
 *		Returns 0 if the ID is valid, else returns corresponding error.
 * @get_context_loss_count: Command to retrieve context loss counter - this
 *		increments every time the device looses context. Overflow
 *		is possible.
 *		- count: pointer to u32 which will retrieve counter
 *		Returns 0 for successful information request and count has
 *		proper data, else returns corresponding error message.
 * @is_idle:	Reports back about device idle state
 *		- req_state: Returns requested idle state
 *		Returns 0 for successful information request and req_state and
 *		current_state has proper data, else returns corresponding error
 *		message.
 * @is_stop:	Reports back about device stop state
 *		- req_state: Returns requested stop state
 *		- current_state: Returns current stop state
 *		Returns 0 for successful information request and req_state and
 *		current_state has proper data, else returns corresponding error
 *		message.
 * @is_on:	Reports back about device ON(or active) state
 *		- req_state: Returns requested ON state
 *		- current_state: Returns current ON state
 *		Returns 0 for successful information request and req_state and
 *		current_state has proper data, else returns corresponding error
 *		message.
 * @is_transitioning: Reports back if the device is in the middle of transition
 *		of state.
 *		-current_state: Returns 'true' if currently transitioning.
 * @set_device_resets: Command to configure resets for device managed by TISCI.
 *		-reset_state: Device specific reset bit field
 *		Returns 0 for successful request, else returns
 *		corresponding error message.
 * @get_device_resets: Command to read state of resets for device managed
 *		by TISCI.
 *		-reset_state: pointer to u32 which will retrieve resets
 *		Returns 0 for successful request, else returns
 *		corresponding error message.
 *
 * NOTE: for all these functions, the following parameters are generic in
 * nature:
 * -handle:	Pointer to TISCI handle as retrieved by *ti_sci_get_handle
 * -id:		Device Identifier
 *
 * Request for the device - NOTE: the client MUST maintain integrity of
 * usage count by balancing get_device with put_device. No refcounting is
 * managed by driver for that purpose.
 */
struct ti_sci_dev_ops {
	int (*get_device)(const struct ti_sci_handle *handle, u32 id);
	int (*idle_device)(const struct ti_sci_handle *handle, u32 id);
	int (*put_device)(const struct ti_sci_handle *handle, u32 id);
	int (*is_valid)(const struct ti_sci_handle *handle, u32 id);
	int (*get_context_loss_count)(const struct ti_sci_handle *handle,
				      u32 id, u32 *count);
	int (*is_idle)(const struct ti_sci_handle *handle, u32 id,
		       bool *requested_state);
	int (*is_stop)(const struct ti_sci_handle *handle, u32 id,
		       bool *req_state, bool *current_state);
	int (*is_on)(const struct ti_sci_handle *handle, u32 id,
		     bool *req_state, bool *current_state);
	int (*is_transitioning)(const struct ti_sci_handle *handle, u32 id,
				bool *current_state);
	int (*set_device_resets)(const struct ti_sci_handle *handle, u32 id,
				 u32 reset_state);
	int (*get_device_resets)(const struct ti_sci_handle *handle, u32 id,
				 u32 *reset_state);
};

/**
 * struct ti_sci_clk_ops - Clock control operations
 * @get_clock:	Request for activation of clock and manage by processor
 *		- needs_ssc: 'true' if Spread Spectrum clock is desired.
 *		- can_change_freq: 'true' if frequency change is desired.
 *		- enable_input_term: 'true' if input termination is desired.
 * @idle_clock:	Request for Idling a clock managed by processor
 * @put_clock:	Release the clock to be auto managed by TISCI
 * @is_auto:	Is the clock being auto managed
 *		- req_state: state indicating if the clock is auto managed
 * @is_on:	Is the clock ON
 *		- req_state: if the clock is requested to be forced ON
 *		- current_state: if the clock is currently ON
 * @is_off:	Is the clock OFF
 *		- req_state: if the clock is requested to be forced OFF
 *		- current_state: if the clock is currently Gated
 * @set_parent:	Set the clock source of a specific device clock
 *		- parent_id: Parent clock identifier to set.
 * @get_parent:	Get the current clock source of a specific device clock
 *		- parent_id: Parent clock identifier which is the parent.
 * @get_num_parents: Get the number of parents of the current clock source
 *		- num_parents: returns the number of parent clocks.
 * @get_best_match_freq: Find a best matching frequency for a frequency
 *		range.
 *		- match_freq: Best matching frequency in Hz.
 * @set_freq:	Set the Clock frequency
 * @get_freq:	Get the Clock frequency
 *		- current_freq: Frequency in Hz that the clock is at.
 *
 * NOTE: for all these functions, the following parameters are generic in
 * nature:
 * -handle:	Pointer to TISCI handle as retrieved by *ti_sci_get_handle
 * -did:	Device identifier this request is for
 * -cid:	Clock identifier for the device for this request.
 *		Each device has it's own set of clock inputs. This indexes
 *		which clock input to modify.
 * -min_freq:	The minimum allowable frequency in Hz. This is the minimum
 *		allowable programmed frequency and does not account for clock
 *		tolerances and jitter.
 * -target_freq: The target clock frequency in Hz. A frequency will be
 *		processed as close to this target frequency as possible.
 * -max_freq:	The maximum allowable frequency in Hz. This is the maximum
 *		allowable programmed frequency and does not account for clock
 *		tolerances and jitter.
 *
 * Request for the clock - NOTE: the client MUST maintain integrity of
 * usage count by balancing get_clock with put_clock. No refcounting is
 * managed by driver for that purpose.
 */
struct ti_sci_clk_ops {
	int (*get_clock)(const struct ti_sci_handle *handle, u32 did, u8 cid,
			 bool needs_ssc, bool can_change_freq,
			 bool enable_input_term);
	int (*idle_clock)(const struct ti_sci_handle *handle, u32 did, u8 cid);
	int (*put_clock)(const struct ti_sci_handle *handle, u32 did, u8 cid);
	int (*is_auto)(const struct ti_sci_handle *handle, u32 did, u8 cid,
		       bool *req_state);
	int (*is_on)(const struct ti_sci_handle *handle, u32 did, u8 cid,
		     bool *req_state, bool *current_state);
	int (*is_off)(const struct ti_sci_handle *handle, u32 did, u8 cid,
		      bool *req_state, bool *current_state);
	int (*set_parent)(const struct ti_sci_handle *handle, u32 did, u8 cid,
			  u8 parent_id);
	int (*get_parent)(const struct ti_sci_handle *handle, u32 did, u8 cid,
			  u8 *parent_id);
	int (*get_num_parents)(const struct ti_sci_handle *handle, u32 did,
			       u8 cid, u8 *num_parents);
	int (*get_best_match_freq)(const struct ti_sci_handle *handle, u32 did,
				   u8 cid, u64 min_freq, u64 target_freq,
				   u64 max_freq, u64 *match_freq);
	int (*set_freq)(const struct ti_sci_handle *handle, u32 did, u8 cid,
			u64 min_freq, u64 target_freq, u64 max_freq);
	int (*get_freq)(const struct ti_sci_handle *handle, u32 did, u8 cid,
			u64 *current_freq);
};

/**
 * struct ti_sci_rm_core_ops - Resource management core operations
 * @get_range:		Get a range of resources belonging to ti sci host.
 * get_rage_from_shost:	Get a range of resources belonging to specified host id.
 *
 * NOTE: for these functions, all the parameters are consolidated and defined
 * as below:
 * - handle:	Pointer to TISCI handle as retrieved by *ti_sci_get_handle
 * - type:	resource assignment type that is being requested for
 * - subtype:	resource assignment subtype that is being requested for
 * - s_host:	Host processing entity to which the resources are allocated
 * - range_start:	Start index of the resource range
 * - range_end:		Number of resources in the range
 */
struct ti_sci_rm_core_ops {
	int (*get_range)(const struct ti_sci_handle *handle, u16 type,
			 u8 subtype, u16 *range_start, u16 *range_num);
	int (*get_range_from_shost)(const struct ti_sci_handle *handle,
				    u16 type, u8 subtype, u8 s_host,
				    u16 *range_start, u16 *range_num);
};

#define TI_SCI_RM_NULL_U8			((u8)~0U)
#define TI_SCI_RM_NULL_U16			((u16)~0U)
#define TI_SCI_RM_NULL_U32			((u32)~0U)

/**
 * struct ti_sci_rm_irq_ops: IRQ management operations
 * @set_direct_irq:		Set Non-event Sourced direct irq to destination
 *				host(same host as ti sci interface id).
 * @set_event_irq:		Set Event based peripheral irq to destination
 *				host(same host as ti sci interface id).
 * @set_direct_irq_from_shost:	Set Non-event Sourced direct irq to a
 *				specified destination host.
 * @set_event_irq_from_shost:	Set Event based peripheral irq to a
 *				specified destination host.
 * @set_event_irq_to_poll:	Set Event based peripheral irq to polling mode.
 *				vint_status_bit is used for polling.
 * @free_direct_irq:		Free a non-event sourced direct irq to
 *				destination host(same as ti sci interface id)
 * @free_event_irq:		Free an event based peripheral irq to
 *				destination host(same as ti sci interface id)
 * @free_direct_irq_from_shost:	Free non-event based direct irq from a
 *				specified destination host.
 * @free_event_irq_from_shost:	Free event based peripheral irq from a
 *				specified destination host.
 * @free_event_irq_to_poll:	Free an event based peripheral irq that is
 *				configured in polling mode.
 *
 * NOTE: for these functions, all the parameters are consolidated and defined
 * as below:
 * - handle:	Pointer to TISCI handle as retrieved by *ti_sci_get_handle
 * - src_id:	Device ID of the IRQ source
 * - src_index:	IRQ source index within the source device
 * - dst_id:	Device ID of the IRQ destination.
 * - dst_host_irq:	IRQ number of the destination device.
 * - ia_id:	Device ID of the IA, if the IRQ flows through this IA
 * - vint:	Virtual interrupt to be used within the IA
 * - global_event:	Global event number to be used for the requesting event.
 * - vint_status_bit:	Virtual interrupt status bit to be used for the event.
 * - s_host:	Secondary host ID to which the irq/event is being requested.
 */
struct ti_sci_rm_irq_ops {
	int (*set_direct_irq)(const struct ti_sci_handle *handle, u16 src_id,
			      u16 src_index, u16 dst_id, u16 dst_host_irq);
	int (*set_event_irq)(const struct ti_sci_handle *handle, u16 src_id,
			     u16 src_index, u16 dst_id, u16 dst_host_irq,
			     u16 ia_id, u16 vint, u16 global_event,
			     u8 vint_status_bit);
	int (*set_direct_irq_from_shost)(const struct ti_sci_handle *handle,
					 u16 src_id, u16 src_index, u16 dst_id,
					 u16 dst_host_irq, u8 s_host);
	int (*set_event_irq_from_shost)(const struct ti_sci_handle *handle,
					u16 src_id, u16 src_index, u16 dst_id,
					u16 dst_host_irq, u16 ia_id, u16 vint,
					u16 global_event, u8 vint_status_bit,
					u8 s_host);
	int (*set_event_irq_to_poll)(const struct ti_sci_handle *handle,
				     u16 src_id, u16 src_index, u16 ia_id,
				     u16 vint, u16 global_event,
				     u8 vint_status_bit);
	int (*free_direct_irq)(const struct ti_sci_handle *handle, u16 src_id,
			       u16 src_index, u16 dst_id, u16 dst_host_irq);
	int (*free_event_irq)(const struct ti_sci_handle *handle, u16 src_id,
			      u16 src_index, u16 dst_id, u16 dst_host_irq,
			      u16 ia_id, u16 vint, u16 global_event,
			      u8 vint_status_bit);
	int (*free_direct_irq_from_shost)(const struct ti_sci_handle *handle,
					  u16 src_id, u16 src_index, u16 dst_id,
					  u16 dst_host_irq, u8 s_host);
	int (*free_event_irq_from_shost)(const struct ti_sci_handle *handle,
					 u16 src_id, u16 src_index, u16 dst_id,
					 u16 dst_host_irq, u16 ia_id, u16 vint,
					 u16 global_event, u8 vint_status_bit,
					 u8 s_host);
	int (*free_event_irq_to_poll)(const struct ti_sci_handle *handle,
				      u16 src_id, u16 src_index, u16 ia_id,
				      u16 vint, u16 global_event,
				      u8 vint_status_bit);
};

/**
 * struct ti_sci_proc_ops - Processor Control operations
 * @request:	Request to control a physical processor. The requesting host
 *		should be in the processor access list
 * @release:	Relinquish a physical processor control
 * @handover:	Handover a physical processor control to another host
 *		in the permitted list
 * @set_config:	Set base configuration of a processor
 * @set_control: Setup limited control flags in specific cases
 * @auth_boot:	Perform an authenticated load and boot of a processor
 * @get_status: Get the state of physical processor
 *
 * NOTE: The following paramteres are generic in nature for all these ops,
 * -handle:	Pointer to TI SCI handle as retrieved by *ti_sci_get_handle
 * -pid:	Processor ID
 * -hid:	Host ID
 */
struct ti_sci_proc_ops {
	int (*request)(const struct ti_sci_handle *handle, u8 pid);
	int (*release)(const struct ti_sci_handle *handle, u8 pid);
	int (*handover)(const struct ti_sci_handle *handle, u8 pid, u8 hid);
	int (*set_config)(const struct ti_sci_handle *handle, u8 pid,
			  u64 boot_vector, u32 cfg_set, u32 cfg_clr);
	int (*set_control)(const struct ti_sci_handle *handle, u8 pid,
			   u32 ctrl_set, u32 ctrl_clr);
	int (*auth_boot)(const struct ti_sci_handle *handle, u8 pid, u64 caddr);
	int (*get_status)(const struct ti_sci_handle *handle, u8 pid,
			  u64 *boot_vector, u32 *cfg_flags, u32 *ctrl_flags,
			  u32 *status_flags);
};

#define TI_SCI_RING_MODE_RING			(0)
#define TI_SCI_RING_MODE_MESSAGE		(1)
#define TI_SCI_RING_MODE_CREDENTIALS		(2)
#define TI_SCI_RING_MODE_QM			(3)
#define TI_SCI_RING_SHARED			(1)

#define TI_SCI_MSG_UNUSED_SECONDARY_HOST TI_SCI_RM_NULL_U8

/**
 * struct ti_sci_rm_ringacc_ops - Ring Accelerator Management operations
 * @allocate: allocate and configure SoC Navigator Subsystem Ring
 *		Accelerator rings. The API allows requesting a specific ring by
 *		passing the exact @index of the ring to be allocated and
 *		configured. Rings can also be requested dynamically, which
 *		allows the RM subsystem to select the next available ring,
 *		of a specific type (@index = TI_SCI_MSG_NULL_RING_INDEX),
 *		for allocation and configuration.
 * @free: free SoC Navigator Subsystem Ring Accelerator rings that were
 *		allocated by TISCI.
 * @reset: reset SoC Navigator Subsystem Ring Accelerator rings that were
 *		allocated by TISCI.
 * @reconfig: reconfigure the qmode of SoC Navigator Subsystem Ring
 *		Accelerator rings that were allocated by TISCI
 */
struct ti_sci_rm_ringacc_ops {
	int (*allocate)(const struct ti_sci_handle *handle, u8 s_host,
			u32 nav_id, u32 *index, u32 addr_lo, u32 addr_hi,
			u32 count, u8 mode, u8 size, u8 order_id,
			u8 share, u16 type);
	int (*free)(const struct ti_sci_handle *handle,
		    u8 s_host, u32 nav_id, u32 index);
	int (*reset)(const struct ti_sci_handle *handle,
		     u32 nav_id, u32 index);
	int (*reconfig)(const struct ti_sci_handle *handle,
			u32 nav_id, u32 index, u8 *mode,
			u32 *addr_lo, u32 *addr_hi, u32 *count,
			u8 *size, u8 *order_id);
};

/**
 * struct ti_sci_rm_psil_ops - PSI-L thread operations
 * @pair: pair PSI-L source thread to a destination thread.
 *	If the src_thread is mapped to UDMA tchan, the corresponding channel's
 *	TCHAN_THRD_ID register is updated.
 *	If the dst_thread is mapped to UDMA rchan, the corresponding channel's
 *	RCHAN_THRD_ID register is updated.
 * @unpair: unpair PSI-L source thread from a destination thread.
 *	If the src_thread is mapped to UDMA tchan, the corresponding channel's
 *	TCHAN_THRD_ID register is cleared.
 *	If the dst_thread is mapped to UDMA rchan, the corresponding channel's
 *	RCHAN_THRD_ID register is cleared.
 */
struct ti_sci_rm_psil_ops {
	int (*pair)(const struct ti_sci_handle *handle, u32 nav_id,
		    u32 src_thread, u32 dst_thread);
	int (*unpair)(const struct ti_sci_handle *handle, u32 nav_id,
		      u32 src_thread, u32 dst_thread);
};

/* UDMAP channel types */
#define TI_SCI_RM_UDMAP_CHAN_TYPE_PKT_PBRR		2
#define TI_SCI_RM_UDMAP_CHAN_TYPE_PKT_PBRR_SB		3	/* RX only */
#define TI_SCI_RM_UDMAP_CHAN_TYPE_3RDP_PBRR		10
#define TI_SCI_RM_UDMAP_CHAN_TYPE_3RDP_PBVR		11
#define TI_SCI_RM_UDMAP_CHAN_TYPE_3RDP_BCOPY_PBRR	12
#define TI_SCI_RM_UDMAP_CHAN_TYPE_3RDP_BCOPY_PBVR	13

/* UDMAP channel atypes */
#define TI_SCI_RM_UDMAP_ATYPE_PHYS			0
#define TI_SCI_RM_UDMAP_ATYPE_INTERMEDIATE		1
#define TI_SCI_RM_UDMAP_ATYPE_VIRTUAL			2

/* UDMAP channel scheduling priorities */
#define TI_SCI_RM_UDMAP_SCHED_PRIOR_HIGH		0
#define TI_SCI_RM_UDMAP_SCHED_PRIOR_MEDHIGH		1
#define TI_SCI_RM_UDMAP_SCHED_PRIOR_MEDLOW		2
#define TI_SCI_RM_UDMAP_SCHED_PRIOR_LOW			3

#define TI_SCI_RM_UDMAP_RX_FLOW_ERR_DROP		0
#define TI_SCI_RM_UDMAP_RX_FLOW_ERR_RETRY		1

#define TI_SCI_RM_UDMAP_RX_FLOW_DESC_HOST		0
#define TI_SCI_RM_UDMAP_RX_FLOW_DESC_MONO		2

/**
 * struct ti_sci_rm_udmap_tx_ch_alloc - parameters for UDMAP transmit channel
 *					allocation
 * @nav_id: SoC Navigator Subsystem device ID from which the transmit channel is
 *	allocated
 * @index: UDMAP transmit channel index.
 * @tx_pause_on_err: UDMAP transmit channel pause on error configuration
 * @tx_filt_einfo: UDMAP transmit channel extended packet information passing
 *	configuration.
 * @tx_filt_pswords: UDMAP transmit channel protocol specific word passing
 *	configuration.
 * @tx_atype: UDMAP transmit channel non Ring Accelerator access pointer
 *	interpretation. Valid values are TI_SCI_RM_UDMAP_ATYPE_*
 * @tx_chan_type: UDMAP transmit channel functional channel type and work
 *	passing mechanism configuration.  Valid types are
 *	TI_SCI_RM_UDMAP_CHAN_TYPE_*
 * @tx_supr_tdpkt: UDMAP transmit channel teardown packet generation suppression
 *	configuration.
 * @tx_fetch_size: UDMAP transmit channel number of 32-bit descriptor words to
 *	fetch configuration.
 * @tx_credit_count: UDMAP transmit channel transfer request credit count
 *	configuration.
 * @txcq_qnum: UDMAP transmit channel completion queue number.
 * @tx_priority: UDMAP transmit channel transmit priority value. Can be either
 *	NULL parameter or valid priority number.
 * @tx_qos: DMAP transmit channel transmit qos value. Can be either NULL
 *	parameter or valid QoS number.
 * @tx_orderid: UDMAP transmit channel bus order id value. Can be either NULL
 *	parameter or valid orderid number.
 * @fdepth: UDMAP transmit channel FIFO depth configuration.
 * @tx_sched_priority: UDMAP transmit channel tx scheduling priority
 *	configuration. Valid values are TI_SCI_RM_UDMAP_SCHED_PRIOR_*
 * @share: Not supported, set it to 0.
 * @type: Not supported, set it to NULL parameter.
 * @secondary_host: Specifies a host ID for which the TISCI header host ID
 *	is proxying the request for.
 *
 * For detailed information on the settings, see the UDMAP section of the TRM.
 */
struct ti_sci_rm_udmap_tx_ch_alloc {
	u32 nav_id;
	u32 index;
	u8 tx_pause_on_err;
	u8 tx_filt_einfo;
	u8 tx_filt_pswords;
	u8 tx_atype;
	u8 tx_chan_type;
	u8 tx_supr_tdpkt;
	u16 tx_fetch_size;
	u8 tx_credit_count;
	u16 txcq_qnum;
	u8 tx_priority;
	u8 tx_qos;
	u8 tx_orderid;
	u16 fdepth;
	u8 tx_sched_priority;
	u8 share;
	u8 type;
	u8 secondary_host;
};

/**
 * struct ti_sci_rm_udmap_rx_ch_alloc - parameters for UDMAP receive channel
 *					allocation
 * @nav_id: SoC Navigator Subsystem device ID from which the receive channel is
 *	allocated
 * @index: UDMAP receive channel index.
 * @rx_fetch_size: UDMAP receive channel number of 32-bit descriptor words to
 *	fetch configuration.
 * @rxcq_qnum: UDMAP receive channel completion queue number.
 * @rx_priority: UDMAP receive channel receive priority value. Can be either
 *	NULL parameter or valid priority number.
 * @rx_qos: DMAP receive channel receive qos value. Can be either NULL
 *	parameter or valid QoS number.
 * @rx_orderid: UDMAP receive channel bus order id value. Can be either NULL
 *	parameter or valid orderid number.
 * @rx_sched_priority: UDMAP receive channel tx scheduling priority
 *	configuration. Valid values are TI_SCI_RM_UDMAP_SCHED_PRIOR_*
 * @flowid_start: UDMAP receive channel additional flows starting index.
 * @flowid_cnt: UDMAP receive channel additional flows count.
 *	flowid_start is only valid when flowid_cnt is not 0. in that case if
 *	flowid_start is NULL parameter, dynamic allocation is requested. If
 *	flowid_start is not NULL and it is valid, a range of flows from the
 *	given index is going to be requested.
 * @rx_pause_on_err: UDMAP receive channel pause on error configuration
 * @rx_atype: UDMAP receive channel non Ring Accelerator access pointer
 *	interpretation. Valid values are TI_SCI_RM_UDMAP_ATYPE_*
 * @rx_chan_type: UDMAP receive channel functional channel type and work
 *	passing mechanism configuration.  Valid types are
 *	TI_SCI_RM_UDMAP_CHAN_TYPE_*
 * @rx_ignore_short: UDMAP receive channel short packet treatment configuration.
 * @rx_ignore_long: UDMAP receive channel long packet treatment configuration.
 * @share: Not supported, set it to 0.
 * @type: Not supported, set it to NULL parameter.
 * @secondary_host: Specifies a host ID for which the TISCI header host ID
 *	is proxying the request for.
 *
 * For detailed information on the settings, see the UDMAP section of the TRM.
 */
struct ti_sci_rm_udmap_rx_ch_alloc {
	u32 nav_id;
	u32 index;
	u16 rx_fetch_size;
	u16 rxcq_qnum;
	u8 rx_priority;
	u8 rx_qos;
	u8 rx_orderid;
	u8 rx_sched_priority;
	u16 flowid_start;
	u16 flowid_cnt;
	u8 rx_pause_on_err;
	u8 rx_atype;
	u8 rx_chan_type;
	u8 rx_ignore_short;
	u8 rx_ignore_long;
	u8 share;
	u8 type;
	u8 secondary_host;
};

/**
 * struct ti_sci_rm_udmap_rx_flow_cfg - parameters for UDMAP receive flow
 *					configuration
 * @nav_id: SoC Navigator Subsystem device ID from which the receive flow is
 *	allocated
 * @flow_index: UDMAP receive flow index for non-optional configuration.
 * @rx_ch_index: Specifies the index of the receive channel using the flow_index.
 * @rx_einfo_present: UDMAP receive flow extended packet info present.
 * @rx_psinfo_present: UDMAP receive flow PS words present.
 * @rx_error_handling: UDMAP receive flow error handling configuration. Valid
 *	values are TI_SCI_RM_UDMAP_RX_FLOW_ERR_DROP/RETRY.
 * @rx_desc_type: UDMAP receive flow descriptor type. It can be one of
 *	TI_SCI_RM_UDMAP_RX_FLOW_DESC_HOST/MONO.
 * @rx_sop_offset: UDMAP receive flow start of packet offset.
 * @rx_dest_qnum: UDMAP receive flow destination queue number.
 * @rx_ps_location: UDMAP receive flow PS words location.
 *	0 - end of packet descriptor
 *	1 - Beginning of the data buffer
 * @rx_src_tag_hi: UDMAP receive flow source tag high byte constant
 * @rx_src_tag_lo: UDMAP receive flow source tag low byte constant
 * @rx_dest_tag_hi: UDMAP receive flow destination tag high byte constant
 * @rx_dest_tag_lo: UDMAP receive flow destination tag low byte constant
 * @rx_src_tag_hi_sel: UDMAP receive flow source tag high byte selector
 * @rx_src_tag_lo_sel: UDMAP receive flow source tag low byte selector
 * @rx_dest_tag_hi_sel: UDMAP receive flow destination tag high byte selector
 * @rx_dest_tag_lo_sel: UDMAP receive flow destination tag low byte selector
 * @rx_size_thresh_en: UDMAP receive flow packet size based free buffer queue
 *	enable. If enabled, the ti_sci_rm_udmap_rx_flow_opt_cfg also need to be
 *	configured and sent.
 * @rx_fdq0_sz0_qnum: UDMAP receive flow free descriptor queue 0.
 * @rx_fdq1_qnum: UDMAP receive flow free descriptor queue 1.
 * @rx_fdq2_qnum: UDMAP receive flow free descriptor queue 2.
 * @rx_fdq3_qnum: UDMAP receive flow free descriptor queue 3.
 *
 * For detailed information on the settings, see the UDMAP section of the TRM.
 */
struct ti_sci_rm_udmap_rx_flow_cfg {
	u32 nav_id;
	u32 flow_index;
	u32 rx_ch_index;
	u8 rx_einfo_present;
	u8 rx_psinfo_present;
	u8 rx_error_handling;
	u8 rx_desc_type;
	u16 rx_sop_offset;
	u16 rx_dest_qnum;
	u8 rx_ps_location;
	u8 rx_src_tag_hi;
	u8 rx_src_tag_lo;
	u8 rx_dest_tag_hi;
	u8 rx_dest_tag_lo;
	u8 rx_src_tag_hi_sel;
	u8 rx_src_tag_lo_sel;
	u8 rx_dest_tag_hi_sel;
	u8 rx_dest_tag_lo_sel;
	u8 rx_size_thresh_en;
	u16 rx_fdq0_sz0_qnum;
	u16 rx_fdq1_qnum;
	u16 rx_fdq2_qnum;
	u16 rx_fdq3_qnum;
};

/**
 * struct ti_sci_rm_udmap_rx_flow_opt_cfg - parameters for UDMAP receive flow
 *					    optional configuration
 * @nav_id: SoC Navigator Subsystem device ID from which the receive flow is
 *	allocated
 * @flow_index: UDMAP receive flow index for optional configuration.
 * @rx_ch_index: Specifies the index of the receive channel using the flow_index
 * @rx_size_thresh0: UDMAP receive flow packet size threshold 0.
 * @rx_size_thresh1: UDMAP receive flow packet size threshold 1.
 * @rx_size_thresh2: UDMAP receive flow packet size threshold 2.
 * @rx_fdq0_sz1_qnum: UDMAP receive flow free descriptor queue for size
 *	threshold 1.
 * @rx_fdq0_sz2_qnum: UDMAP receive flow free descriptor queue for size
 *	threshold 2.
 * @rx_fdq0_sz3_qnum: UDMAP receive flow free descriptor queue for size
 *	threshold 3.
 *
 * For detailed information on the settings, see the UDMAP section of the TRM.
 */
struct ti_sci_rm_udmap_rx_flow_opt_cfg {
	u32 nav_id;
	u32 flow_index;
	u32 rx_ch_index;
	u16 rx_size_thresh0;
	u16 rx_size_thresh1;
	u16 rx_size_thresh2;
	u16 rx_fdq0_sz1_qnum;
	u16 rx_fdq0_sz2_qnum;
	u16 rx_fdq0_sz3_qnum;
};

/**
 * struct ti_sci_rm_udmap_ops - UDMA Management operations
 * @tx_ch_alloc: allocate and configure SoC Navigator Subsystem UDMA transmit
 *	channel. The channel is allocated based on passed @params
 *	ti_sci_rm_udmap_tx_ch_alloc parameters and the allocated channel number
 *	is returned via @index.
 * @tx_ch_free: free SoC Navigator Subsystem UDMA transmit channel that were
 *	allocated by TISCI.
 * @rx_ch_alloc: allocate and configure SoC Navigator Subsystem UDMA receive
 *	channel. The channel is allocated based on passed @params
 *	ti_sci_rm_udmap_rx_ch_alloc parameters and the allocated channel number
 *	is returned via @index, the default flow id via @def_flow_index and the
 *	information about the extra flow range via def_flow_index/rng_flow_cnt
 *	if it was requested.
 * @rx_ch_free: free SoC Navigator Subsystem UDMA receive channel that were
 *	allocated by TISCI.
 * @rx_flow_cfg: configure SoC Navigator Subsystem UDMA receive flow. The flow
 *	configuration is passed via @params ti_sci_rm_udmap_rx_flow_cfg.
 * @rx_flow_cfg: configure SoC Navigator Subsystem optional UDMA receive flow.
 *	The configuration is passed via @params ti_sci_rm_udmap_rx_flow_opt_cfg.
 */
struct ti_sci_rm_udmap_ops {
	int (*tx_ch_alloc)(const struct ti_sci_handle *handle,
			   const struct ti_sci_rm_udmap_tx_ch_alloc *params,
			   u32 *index);
	int (*tx_ch_free)(const struct ti_sci_handle *handle,
			  u8 secondary_host, u32 nav_id, u32 index);
	int (*rx_ch_alloc)(const struct ti_sci_handle *handle,
			   const struct ti_sci_rm_udmap_rx_ch_alloc *params,
			   u32 *index, u32 *def_flow_index,
			   u32 *rng_flow_start_index, u32 *rng_flow_cnt);
	int (*rx_ch_free)(const struct ti_sci_handle *handle,
			  u8 secondary_host, u32 nav_id, u32 index);
	int (*rx_flow_cfg)(const struct ti_sci_handle *handle,
			   const struct ti_sci_rm_udmap_rx_flow_cfg *params);
	int (*rx_flow_opt_cfg)(
		const struct ti_sci_handle *handle,
		const struct ti_sci_rm_udmap_rx_flow_opt_cfg *params);
};

/**
 * struct ti_sci_ops - Function support for TI SCI
 * @dev_ops:	Device specific operations
 * @clk_ops:	Clock specific operations
 * @irq_ops:	IRQ management specific operations
 * @proc_ops:	Processor Control specific operations
 * @ring_ops: Ring Accelerator Management operations
 */
struct ti_sci_ops {
	struct ti_sci_core_ops core_ops;
	struct ti_sci_dev_ops dev_ops;
	struct ti_sci_clk_ops clk_ops;
	struct ti_sci_rm_core_ops rm_core_ops;
	struct ti_sci_rm_irq_ops rm_irq_ops;
	struct ti_sci_proc_ops proc_ops;
	struct ti_sci_rm_ringacc_ops rm_ring_ops;
	struct ti_sci_rm_psil_ops rm_psil_ops;
	struct ti_sci_rm_udmap_ops rm_udmap_ops;
};

/**
 * struct ti_sci_handle - Handle returned to TI SCI clients for usage.
 * @version:	structure containing version information
 * @ops:	operations that are made available to TI SCI clients
 */
struct ti_sci_handle {
	struct ti_sci_version_info version;
	struct ti_sci_ops ops;
};

#define TI_SCI_RESOURCE_NULL	0xffff

struct ti_sci_resource_desc {
	u16 start;
	u16 max;
	unsigned long *res_map;
};

struct ti_sci_resource {
	u16 sets;
	raw_spinlock_t lock;
	struct ti_sci_resource_desc *desc;
};

#if IS_ENABLED(CONFIG_TI_SCI_PROTOCOL)
const struct ti_sci_handle *ti_sci_get_handle(struct device *dev);
int ti_sci_put_handle(const struct ti_sci_handle *handle);
const struct ti_sci_handle *devm_ti_sci_get_handle(struct device *dev);
const struct ti_sci_handle *ti_sci_get_by_phandle(struct device_node *np,
						  const char *property);
u16 ti_sci_get_free_resource(struct ti_sci_resource *res);
void ti_sci_release_resource(struct ti_sci_resource *res, u16 id);
struct ti_sci_resource *
devm_ti_sci_get_of_resource(const struct ti_sci_handle *handle,
			    struct device *dev, char *of_prop);

#else	/* CONFIG_TI_SCI_PROTOCOL */

static inline const struct ti_sci_handle *ti_sci_get_handle(struct device *dev)
{
	return ERR_PTR(-EINVAL);
}

static inline int ti_sci_put_handle(const struct ti_sci_handle *handle)
{
	return -EINVAL;
}

static inline
const struct ti_sci_handle *devm_ti_sci_get_handle(struct device *dev)
{
	return ERR_PTR(-EINVAL);
}

static inline
const struct ti_sci_handle *ti_sci_get_by_phandle(struct device_node *np,
						  const char *property)
{
	return ERR_PTR(-EINVAL);
}

static inline u16 ti_sci_get_free_resource(struct ti_sci_resource *res)
{
	return TI_SCI_RESOURCE_NULL;
}

static inline void ti_sci_release_resource(struct ti_sci_resource *res, u16 id)
{
}

static inline struct ti_sci_resource *
devm_ti_sci_get_of_resource(const struct ti_sci_handle *handle,
			    struct device *dev, char *of_prop)
{
	return ERR_PTR(-EINVAL);
}

#endif	/* CONFIG_TI_SCI_PROTOCOL */

#endif	/* __TISCI_PROTOCOL_H */
