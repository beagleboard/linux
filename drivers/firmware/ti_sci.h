/*
 * Texas Instruments System Control Interface (TISCI) Protocol
 *
 * Communication protocol with TI SCI hardware
 * The system works in a message response protocol
 * See: http://processors.wiki.ti.com/index.php/TISCI for details
 *
 * Copyright (C)  2015-2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
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
 *
 */

#ifndef __TI_SCI_H
#define __TI_SCI_H

/* Generic Messages */
#define TI_SCI_MSG_ENABLE_WDT	0x0000
#define TI_SCI_MSG_WAKE_RESET	0x0001
#define TI_SCI_MSG_VERSION	0x0002
#define TI_SCI_MSG_WAKE_REASON	0x0003
#define TI_SCI_MSG_GOODBYE	0x0004
#define TI_SCI_MSG_SYS_RESET	0x0005

/* Device requests */
#define TI_SCI_MSG_SET_DEVICE_STATE	0x0200
#define TI_SCI_MSG_GET_DEVICE_STATE	0x0201
#define TI_SCI_MSG_SET_DEVICE_RESETS	0x0202

/* Clock requests */
#define TI_SCI_MSG_SET_CLOCK_STATE	0x0100
#define TI_SCI_MSG_GET_CLOCK_STATE	0x0101
#define TI_SCI_MSG_SET_CLOCK_PARENT	0x0102
#define TI_SCI_MSG_GET_CLOCK_PARENT	0x0103
#define TI_SCI_MSG_GET_NUM_CLOCK_PARENTS 0x0104
#define TI_SCI_MSG_SET_CLOCK_FREQ	0x010c
#define TI_SCI_MSG_QUERY_CLOCK_FREQ	0x010d
#define TI_SCI_MSG_GET_CLOCK_FREQ	0x010e

/* IRQ requests */
#define TI_SCI_MSG_SET_IRQ		0x1000
#define TI_SCI_MSG_FREE_IRQ		0x1001

/* NAVSS resource management */
/* Ringacc requests */
#define TI_SCI_MSG_RM_RING_ALLOCATE		0x1100
#define TI_SCI_MSG_RM_RING_FREE			0x1101
#define TI_SCI_MSG_RM_RING_RECONFIG		0x1102
#define TI_SCI_MSG_RM_RING_RESET		0x1103

/* PSI-L requests */
#define TI_SCI_MSG_RM_PSIL_PAIR			0x1280
#define TI_SCI_MSG_RM_PSIL_UNPAIR		0x1281

#define TI_SCI_MSG_RM_UDMAP_TX_ALLOC		0x1200
#define TI_SCI_MSG_RM_UDMAP_TX_FREE		0x1201
#define TI_SCI_MSG_RM_UDMAP_RX_ALLOC		0x1210
#define TI_SCI_MSG_RM_UDMAP_RX_FREE		0x1211
#define TI_SCI_MSG_RM_UDMAP_FLOW_CFG		0x1220
#define TI_SCI_MSG_RM_UDMAP_OPT_FLOW_CFG	0x1221

/**
 * struct ti_sci_msg_hdr - Generic Message Header for All messages and responses
 * @type:	Type of messages: One of TI_SCI_MSG* values
 * @host:	Host of the message
 * @seq:	Message identifier indicating a transfer sequence
 * @flags:	Flag for the message
 */
struct ti_sci_msg_hdr {
	u16 type;
	u8 host;
	u8 seq;
#define TI_SCI_MSG_FLAG(val)			(1 << (val))
#define TI_SCI_FLAG_REQ_GENERIC_NORESPONSE	0x0
#define TI_SCI_FLAG_REQ_ACK_ON_RECEIVED		TI_SCI_MSG_FLAG(0)
#define TI_SCI_FLAG_REQ_ACK_ON_PROCESSED	TI_SCI_MSG_FLAG(1)
#define TI_SCI_FLAG_RESP_GENERIC_NACK		0x0
#define TI_SCI_FLAG_RESP_GENERIC_ACK		TI_SCI_MSG_FLAG(1)
	/* Additional Flags */
	u32 flags;
} __packed;

/**
 * struct ti_sci_msg_resp_version - Response for a message
 * @hdr:		Generic header
 * @firmware_description: String describing the firmware
 * @firmware_revision:	Firmware revision
 * @abi_major:		Major version of the ABI that firmware supports
 * @abi_minor:		Minor version of the ABI that firmware supports
 *
 * In general, ABI version changes follow the rule that minor version increments
 * are backward compatible. Major revision changes in ABI may not be
 * backward compatible.
 *
 * Response to a generic message with message type TI_SCI_MSG_VERSION
 */
struct ti_sci_msg_resp_version {
	struct ti_sci_msg_hdr hdr;
	char firmware_description[32];
	u16 firmware_revision;
	u8 abi_major;
	u8 abi_minor;
} __packed;

/**
 * struct ti_sci_msg_req_reboot - Reboot the SoC
 * @hdr:	Generic Header
 *
 * Request type is TI_SCI_MSG_SYS_RESET, responded with a generic
 * ACK/NACK message.
 */
struct ti_sci_msg_req_reboot {
	struct ti_sci_msg_hdr hdr;
} __packed;

/**
 * struct ti_sci_msg_req_set_device_state - Set the desired state of the device
 * @hdr:		Generic header
 * @id:	Indicates which device to modify
 * @reserved: Reserved space in message, must be 0 for backward compatibility
 * @state: The desired state of the device.
 *
 * Certain flags can also be set to alter the device state:
 * + MSG_FLAG_DEVICE_WAKE_ENABLED - Configure the device to be a wake source.
 * The meaning of this flag will vary slightly from device to device and from
 * SoC to SoC but it generally allows the device to wake the SoC out of deep
 * suspend states.
 * + MSG_FLAG_DEVICE_RESET_ISO - Enable reset isolation for this device.
 * + MSG_FLAG_DEVICE_EXCLUSIVE - Claim this device exclusively. When passed
 * with STATE_RETENTION or STATE_ON, it will claim the device exclusively.
 * If another host already has this device set to STATE_RETENTION or STATE_ON,
 * the message will fail. Once successful, other hosts attempting to set
 * STATE_RETENTION or STATE_ON will fail.
 *
 * Request type is TI_SCI_MSG_SET_DEVICE_STATE, responded with a generic
 * ACK/NACK message.
 */
struct ti_sci_msg_req_set_device_state {
	/* Additional hdr->flags options */
#define MSG_FLAG_DEVICE_WAKE_ENABLED	TI_SCI_MSG_FLAG(8)
#define MSG_FLAG_DEVICE_RESET_ISO	TI_SCI_MSG_FLAG(9)
#define MSG_FLAG_DEVICE_EXCLUSIVE	TI_SCI_MSG_FLAG(10)
	struct ti_sci_msg_hdr hdr;
	u32 id;
	u32 reserved;

#define MSG_DEVICE_SW_STATE_AUTO_OFF	0
#define MSG_DEVICE_SW_STATE_RETENTION	1
#define MSG_DEVICE_SW_STATE_ON		2
	u8 state;
} __packed;

/**
 * struct ti_sci_msg_req_get_device_state - Request to get device.
 * @hdr:		Generic header
 * @id:		Device Identifier
 *
 * Request type is TI_SCI_MSG_GET_DEVICE_STATE, responded device state
 * information
 */
struct ti_sci_msg_req_get_device_state {
	struct ti_sci_msg_hdr hdr;
	u32 id;
} __packed;

/**
 * struct ti_sci_msg_resp_get_device_state - Response to get device request.
 * @hdr:		Generic header
 * @context_loss_count: Indicates how many times the device has lost context. A
 *	driver can use this monotonic counter to determine if the device has
 *	lost context since the last time this message was exchanged.
 * @resets: Programmed state of the reset lines.
 * @programmed_state:	The state as programmed by set_device.
 *			- Uses the MSG_DEVICE_SW_* macros
 * @current_state:	The actual state of the hardware.
 *
 * Response to request TI_SCI_MSG_GET_DEVICE_STATE.
 */
struct ti_sci_msg_resp_get_device_state {
	struct ti_sci_msg_hdr hdr;
	u32 context_loss_count;
	u32 resets;
	u8 programmed_state;
#define MSG_DEVICE_HW_STATE_OFF		0
#define MSG_DEVICE_HW_STATE_ON		1
#define MSG_DEVICE_HW_STATE_TRANS	2
	u8 current_state;
} __packed;

/**
 * struct ti_sci_msg_req_set_device_resets - Set the desired resets
 *				configuration of the device
 * @hdr:		Generic header
 * @id:	Indicates which device to modify
 * @resets: A bit field of resets for the device. The meaning, behavior,
 *	and usage of the reset flags are device specific. 0 for a bit
 *	indicates releasing the reset represented by that bit while 1
 *	indicates keeping it held.
 *
 * Request type is TI_SCI_MSG_SET_DEVICE_RESETS, responded with a generic
 * ACK/NACK message.
 */
struct ti_sci_msg_req_set_device_resets {
	struct ti_sci_msg_hdr hdr;
	u32 id;
	u32 resets;
} __packed;

/**
 * struct ti_sci_msg_req_set_clock_state - Request to setup a Clock state
 * @hdr:	Generic Header, Certain flags can be set specific to the clocks:
 *		MSG_FLAG_CLOCK_ALLOW_SSC: Allow this clock to be modified
 *		via spread spectrum clocking.
 *		MSG_FLAG_CLOCK_ALLOW_FREQ_CHANGE: Allow this clock's
 *		frequency to be changed while it is running so long as it
 *		is within the min/max limits.
 *		MSG_FLAG_CLOCK_INPUT_TERM: Enable input termination, this
 *		is only applicable to clock inputs on the SoC pseudo-device.
 * @dev_id:	Device identifier this request is for
 * @clk_id:	Clock identifier for the device for this request.
 *		Each device has it's own set of clock inputs. This indexes
 *		which clock input to modify.
 * @request_state: Request the state for the clock to be set to.
 *		MSG_CLOCK_SW_STATE_UNREQ: The IP does not require this clock,
 *		it can be disabled, regardless of the state of the device
 *		MSG_CLOCK_SW_STATE_AUTO: Allow the System Controller to
 *		automatically manage the state of this clock. If the device
 *		is enabled, then the clock is enabled. If the device is set
 *		to off or retention, then the clock is internally set as not
 *		being required by the device.(default)
 *		MSG_CLOCK_SW_STATE_REQ:  Configure the clock to be enabled,
 *		regardless of the state of the device.
 *
 * Normally, all required clocks are managed by TISCI entity, this is used
 * only for specific control *IF* required. Auto managed state is
 * MSG_CLOCK_SW_STATE_AUTO, in other states, TISCI entity assume remote
 * will explicitly control.
 *
 * Request type is TI_SCI_MSG_SET_CLOCK_STATE, response is a generic
 * ACK or NACK message.
 */
struct ti_sci_msg_req_set_clock_state {
	/* Additional hdr->flags options */
#define MSG_FLAG_CLOCK_ALLOW_SSC		TI_SCI_MSG_FLAG(8)
#define MSG_FLAG_CLOCK_ALLOW_FREQ_CHANGE	TI_SCI_MSG_FLAG(9)
#define MSG_FLAG_CLOCK_INPUT_TERM		TI_SCI_MSG_FLAG(10)
	struct ti_sci_msg_hdr hdr;
	u32 dev_id;
	u8 clk_id;
#define MSG_CLOCK_SW_STATE_UNREQ	0
#define MSG_CLOCK_SW_STATE_AUTO		1
#define MSG_CLOCK_SW_STATE_REQ		2
	u8 request_state;
} __packed;

/**
 * struct ti_sci_msg_req_get_clock_state - Request for clock state
 * @hdr:	Generic Header
 * @dev_id:	Device identifier this request is for
 * @clk_id:	Clock identifier for the device for this request.
 *		Each device has it's own set of clock inputs. This indexes
 *		which clock input to get state of.
 *
 * Request type is TI_SCI_MSG_GET_CLOCK_STATE, response is state
 * of the clock
 */
struct ti_sci_msg_req_get_clock_state {
	struct ti_sci_msg_hdr hdr;
	u32 dev_id;
	u8 clk_id;
} __packed;

/**
 * struct ti_sci_msg_resp_get_clock_state - Response to get clock state
 * @hdr:	Generic Header
 * @programmed_state: Any programmed state of the clock. This is one of
 *		MSG_CLOCK_SW_STATE* values.
 * @current_state: Current state of the clock. This is one of:
 *		MSG_CLOCK_HW_STATE_NOT_READY: Clock is not ready
 *		MSG_CLOCK_HW_STATE_READY: Clock is ready
 *
 * Response to TI_SCI_MSG_GET_CLOCK_STATE.
 */
struct ti_sci_msg_resp_get_clock_state {
	struct ti_sci_msg_hdr hdr;
	u8 programmed_state;
#define MSG_CLOCK_HW_STATE_NOT_READY	0
#define MSG_CLOCK_HW_STATE_READY	1
	u8 current_state;
} __packed;

/**
 * struct ti_sci_msg_req_set_clock_parent - Set the clock parent
 * @hdr:	Generic Header
 * @dev_id:	Device identifier this request is for
 * @clk_id:	Clock identifier for the device for this request.
 *		Each device has it's own set of clock inputs. This indexes
 *		which clock input to modify.
 * @parent_id:	The new clock parent is selectable by an index via this
 *		parameter.
 *
 * Request type is TI_SCI_MSG_SET_CLOCK_PARENT, response is generic
 * ACK / NACK message.
 */
struct ti_sci_msg_req_set_clock_parent {
	struct ti_sci_msg_hdr hdr;
	u32 dev_id;
	u8 clk_id;
	u8 parent_id;
} __packed;

/**
 * struct ti_sci_msg_req_get_clock_parent - Get the clock parent
 * @hdr:	Generic Header
 * @dev_id:	Device identifier this request is for
 * @clk_id:	Clock identifier for the device for this request.
 *		Each device has it's own set of clock inputs. This indexes
 *		which clock input to get the parent for.
 *
 * Request type is TI_SCI_MSG_GET_CLOCK_PARENT, response is parent information
 */
struct ti_sci_msg_req_get_clock_parent {
	struct ti_sci_msg_hdr hdr;
	u32 dev_id;
	u8 clk_id;
} __packed;

/**
 * struct ti_sci_msg_resp_get_clock_parent - Response with clock parent
 * @hdr:	Generic Header
 * @parent_id:	The current clock parent
 *
 * Response to TI_SCI_MSG_GET_CLOCK_PARENT.
 */
struct ti_sci_msg_resp_get_clock_parent {
	struct ti_sci_msg_hdr hdr;
	u8 parent_id;
} __packed;

/**
 * struct ti_sci_msg_req_get_clock_num_parents - Request to get clock parents
 * @hdr:	Generic header
 * @dev_id:	Device identifier this request is for
 * @clk_id:	Clock identifier for the device for this request.
 *
 * This request provides information about how many clock parent options
 * are available for a given clock to a device. This is typically used
 * for input clocks.
 *
 * Request type is TI_SCI_MSG_GET_NUM_CLOCK_PARENTS, response is appropriate
 * message, or NACK in case of inability to satisfy request.
 */
struct ti_sci_msg_req_get_clock_num_parents {
	struct ti_sci_msg_hdr hdr;
	u32 dev_id;
	u8 clk_id;
} __packed;

/**
 * struct ti_sci_msg_resp_get_clock_num_parents - Response for get clk parents
 * @hdr:		Generic header
 * @num_parents:	Number of clock parents
 *
 * Response to TI_SCI_MSG_GET_NUM_CLOCK_PARENTS
 */
struct ti_sci_msg_resp_get_clock_num_parents {
	struct ti_sci_msg_hdr hdr;
	u8 num_parents;
} __packed;

/**
 * struct ti_sci_msg_req_query_clock_freq - Request to query a frequency
 * @hdr:	Generic Header
 * @dev_id:	Device identifier this request is for
 * @min_freq_hz: The minimum allowable frequency in Hz. This is the minimum
 *		allowable programmed frequency and does not account for clock
 *		tolerances and jitter.
 * @target_freq_hz: The target clock frequency. A frequency will be found
 *		as close to this target frequency as possible.
 * @max_freq_hz: The maximum allowable frequency in Hz. This is the maximum
 *		allowable programmed frequency and does not account for clock
 *		tolerances and jitter.
 * @clk_id:	Clock identifier for the device for this request.
 *
 * NOTE: Normally clock frequency management is automatically done by TISCI
 * entity. In case of specific requests, TISCI evaluates capability to achieve
 * requested frequency within provided range and responds with
 * result message.
 *
 * Request type is TI_SCI_MSG_QUERY_CLOCK_FREQ, response is appropriate message,
 * or NACK in case of inability to satisfy request.
 */
struct ti_sci_msg_req_query_clock_freq {
	struct ti_sci_msg_hdr hdr;
	u32 dev_id;
	u64 min_freq_hz;
	u64 target_freq_hz;
	u64 max_freq_hz;
	u8 clk_id;
} __packed;

/**
 * struct ti_sci_msg_resp_query_clock_freq - Response to a clock frequency query
 * @hdr:	Generic Header
 * @freq_hz:	Frequency that is the best match in Hz.
 *
 * Response to request type TI_SCI_MSG_QUERY_CLOCK_FREQ. NOTE: if the request
 * cannot be satisfied, the message will be of type NACK.
 */
struct ti_sci_msg_resp_query_clock_freq {
	struct ti_sci_msg_hdr hdr;
	u64 freq_hz;
} __packed;

/**
 * struct ti_sci_msg_req_set_clock_freq - Request to setup a clock frequency
 * @hdr:	Generic Header
 * @dev_id:	Device identifier this request is for
 * @min_freq_hz: The minimum allowable frequency in Hz. This is the minimum
 *		allowable programmed frequency and does not account for clock
 *		tolerances and jitter.
 * @target_freq_hz: The target clock frequency. The clock will be programmed
 *		at a rate as close to this target frequency as possible.
 * @max_freq_hz: The maximum allowable frequency in Hz. This is the maximum
 *		allowable programmed frequency and does not account for clock
 *		tolerances and jitter.
 * @clk_id:	Clock identifier for the device for this request.
 *
 * NOTE: Normally clock frequency management is automatically done by TISCI
 * entity. In case of specific requests, TISCI evaluates capability to achieve
 * requested range and responds with success/failure message.
 *
 * This sets the desired frequency for a clock within an allowable
 * range. This message will fail on an enabled clock unless
 * MSG_FLAG_CLOCK_ALLOW_FREQ_CHANGE is set for the clock. Additionally,
 * if other clocks have their frequency modified due to this message,
 * they also must have the MSG_FLAG_CLOCK_ALLOW_FREQ_CHANGE or be disabled.
 *
 * Calling set frequency on a clock input to the SoC pseudo-device will
 * inform the PMMC of that clock's frequency. Setting a frequency of
 * zero will indicate the clock is disabled.
 *
 * Calling set frequency on clock outputs from the SoC pseudo-device will
 * function similarly to setting the clock frequency on a device.
 *
 * Request type is TI_SCI_MSG_SET_CLOCK_FREQ, response is a generic ACK/NACK
 * message.
 */
struct ti_sci_msg_req_set_clock_freq {
	struct ti_sci_msg_hdr hdr;
	u32 dev_id;
	u64 min_freq_hz;
	u64 target_freq_hz;
	u64 max_freq_hz;
	u8 clk_id;
} __packed;

/**
 * struct ti_sci_msg_req_get_clock_freq - Request to get the clock frequency
 * @hdr:	Generic Header
 * @dev_id:	Device identifier this request is for
 * @clk_id:	Clock identifier for the device for this request.
 *
 * NOTE: Normally clock frequency management is automatically done by TISCI
 * entity. In some cases, clock frequencies are configured by host.
 *
 * Request type is TI_SCI_MSG_GET_CLOCK_FREQ, responded with clock frequency
 * that the clock is currently at.
 */
struct ti_sci_msg_req_get_clock_freq {
	struct ti_sci_msg_hdr hdr;
	u32 dev_id;
	u8 clk_id;
} __packed;

/**
 * struct ti_sci_msg_resp_get_clock_freq - Response of clock frequency request
 * @hdr:	Generic Header
 * @freq_hz:	Frequency that the clock is currently on, in Hz.
 *
 * Response to request type TI_SCI_MSG_GET_CLOCK_FREQ.
 */
struct ti_sci_msg_resp_get_clock_freq {
	struct ti_sci_msg_hdr hdr;
	u64 freq_hz;
} __packed;

/**
 * struct ti_sci_msg_req_set_irq - Request to configure the route between
 *				   the dev and the host.
 * @hdr:		Generic Header
 * @dev_id:		IRQ source peripheral ID.
 * @irq:		IRQ source offset within the peripheral
 * @group:		The requested IRQ will be added to this virtual
 *			interrupt group.
 *			In order to create a new virtual interrupt group
 *			this should be -1 along with @share field enabled.
 * @share:		If enabled, the resultant IRQ will be shared with
 *			other events.
 * @poll:		If non-zero, the configured IRQ will not result
 *			in a physical interrupt.
 * @secondary_host:	Host ID of the IRQ destination computing entity. This is
 *			required only when destination host id is different
 *			from ti sci interface host id, else the invalid host id
 *			value of TI_SCI_IRQ_SECONDARY_HOST_INVALID (0xFF) can be
 *			passed.
 *
 * Request type is TI_SCI_MSG_SET_IRQ. Responded with IRQ number of the host
 * interrupt controller input which is of type TI_SCI_MSG_SET_IRQ.
 */
struct ti_sci_msg_req_set_irq {
	struct ti_sci_msg_hdr hdr;
	u16 dev_id;
	u16 irq;
	u32 group;
	u8 share;
	u8 pool;
	u8 secondary_host;
} __packed;

/**
 * struct ti_sci_msg_resp_set_irq - Response to the IRQ set.
 * @hdr:		Generic header
 * @global_event:	Global event routed within IA to virtual interrupt.
 * @ia_id:		SoC dev ID of the IA programmed as part of the
 *			IRQ route to the host processor. A (-1) is returned
 *			if the route is successfully programmed but does
 *			not contain an IA within it.
 * @vint:		Virtual interrupt number of the resultant IRQ.
 * @group:		Virtual interrupt group to which the IRQ belongs.
 * @host_irq:		IRQ number as per the host interrupt controller input
 * @vint_status_bit:	Virtual interrupt status bit.
 *
 * Response to request type TI_SCI_MSG_SET_IRQ.
 */
struct ti_sci_msg_resp_set_irq {
	struct ti_sci_msg_hdr hdr;
	u32 global_event;
	u32 ia_id;
	u32 vint;
	u32 group;
	u16 host_irq;
	u8 vint_status_bit;
} __packed;

/**
 * struct ti_sci_msg_req_free_irq - Request to release irq.
 * @dev_id:		IRQ source peripheral ID.
 * @irq:		IRQ source offset within the peripheral
 * @global_event:	Global event programmed as part of IRQ route.
 * @group:		Virtual interrupt group number to which the IRQ belongs.
 * @secondary_host:	Host ID of the IRQ destination computing entity. This is
 *			required only when destination host id is different
 *			from ti sci interface host id, else the invalid host id
 *			value of TI_SCI_IRQ_SECONDARY_HOST_INVALID (0xFF) can be
 *			passed.
 *
 * Request type is TI_SCI_MSG_FREE_IRQ, response is a generic ACK/NACK
 * message.
 */
struct ti_sci_msg_req_free_irq {
	struct ti_sci_msg_hdr hdr;
	u16 dev_id;
	u16 irq;
	u32 global_event;
	u32 group;
	u8 secondary_host;
} __packed;

/**
 * struct ti_sci_msg_ring_allocate - Ring Accelerator Ring Allocate
 * @hdr: Generic Header
 * @secondary_host: Specifies a host ID for which the TISCI header host ID
 *	is proxying the request for.
 * @nav_id: Device ID of Navigator Subsystem from which the ring is allocated
 * @index: Ring index. A valid value will result in a static allocation and
 *	configuration of the specified ring.
 *	Passing @TI_SCI_RING_NULL_RING_INDEX will result in the allocation of
 *	the next free ring within the subset of rings of
 *	@ti_sci_msg_ring_allocate::type.
 * @addr_lo: The ring base address must be aligned to the "size" parameter
 *	setting when the ring is configured for RING or MESSAGE modes.
 *	For CREDENTIALS and QM modes the ring base address must be aligned to
 *	a 4k byte boundary 32 LSBs of ring base address to be programmed
 *	into the ring's RING_BA_LO register.
 * @addr_hi: See "addr_lo" description,
 * @count: Number of ring elements. Must be even if mode is CREDENTIALS or QM
 *	modes.
 * @mode: Specifies the mode the ring is to be configured as. Allowed values:
 *	@TI_SCI_RING_MODE_RING
 *	@TI_SCI_RING_MODE_MESSAGE
 *	@TI_SCI_RING_MODE_CREDENTIALS
 *	@TI_SCI_RING_MODE_QM
 * @size: Specifies the ring element size. To calculate the encoded size use
 *	the formula (log2(size_bytes) - 2), where size_bytes cannot be
 *	greater than 256.
 * @order_id: Specifies the ring's bus order ID. To use the default order ID
 *	this value must be @TI_SCI_RING_NULL_ORDER_ID. Otherwise, the specified
 *	order ID will be programmed as the ring's bus order ID.
 * @share: Specifies whether the allocated and configured ring is unshared,
 *	limited shared, or open shared. Requests for an already allocated,
 *	shared ring view this value as don't care as long as a valid value is
 *	provided.
 * @type: Specifies the ring type to be allocated for dynamic ring requests
 *	where the index is @TI_SCI_RING_NULL_RING_INDEX.
 *	NACK will be returned if both the ring index and type parameters
 *	are valid.
 *
 * Request type is TI_SCI_MSG_RM_RING_ALLOCATE, responded with the ring index
 * allocated if the request was successful.
 *
 * Allocates and configures the non-real-time registers of a Navigator
 * Subsystem ring.  The message provides two methods for ring allocation,
 * static and dynamic. Static allocation occurs when the
 * @ti_sci_msg_ring_allocate::index parameter is set to a valid ring
 * index. In this case, TISCI RM will allocate and configure the specified ring
 * index. A static request will be NACK'd if the @ti_sci_msg_ring_allocate::type
 * parameter is not set to @TI_SCI_RING_NULL_RING_TYPE. The ring type is
 * redundant in static requests since TISCI RM assumes the user knows which ring
 * type is being allocated. Dynamic allocation occurs when the index parameter
 * contains @TI_SCI_RING_NULL_RING_INDEX. In this case,
 * @ti_sci_msg_ring_allocate::type must be set to a valid ring type
 * from which TISCI RM will allocate and configure the next free ring. The
 * allocated and configured ring index will be returned in the
 * @ti_sci_msg_ring_allocate_resp message for both allocation methods.
 */
struct ti_sci_msg_ring_allocate {
	struct ti_sci_msg_hdr hdr;
	u32 nav_id;
	u32 index;
	u32 addr_lo;
	u32 addr_hi;
	u32 count;
	u8 mode;
	u8 size;
	u8 order_id;
	u8 share;
	u16 type;
	u8 secondary_host;
} __packed;

/**
 * struct ti_sci_msg_ring_allocate_resp - Ring allocate response message
 * @hdr: Generic Header
 * @index: Allocated and configured Ring Accelerator ring index. The ring will
 *	be allocated from the Navigator Subsystem specified in the allocate
 *	request @ti_sci_msg_ring_allocate::nav_id parameter. Only valid if
 *	message is ACK'd, but will be NULL anyway if NACK'd.
 *
 * Response to request type TI_SCI_MSG_RM_RING_ALLOCATE
 */
struct ti_sci_msg_ring_allocate_resp {
	struct ti_sci_msg_hdr hdr;
	u32 index;
} __packed;

/**
 * struct ti_sci_msg_ring_allocate_resp - Free and clear Ring Accelerator ring
 * @hdr: Generic Header
 * @index: ring index to be freed. Shared rings must be freed the same number of
 *	times they were allocated.
 * @secondary_host: Specifies a host ID for which the TISCI header host ID
 *	is proxying the request for.
 * @nav_id: Device ID of Navigator Subsystem from which the ring is allocated
 *
 * Free SoC Navigator Subsystem Ring Accelerator rings that were allocated
 * via @TI_SCI_MSG_RM_RING_ALLOCATE TISCI message. Freeing a ring does not reset
 * the ring registers. Firewall access to the ring's real-time registers for
 * the host sending the ring free request will be revoked.
 *
 * Request type is TI_SCI_MSG_RM_RING_FREE, response is a generic
 * ACK or NACK message.
 */
struct ti_sci_msg_ring_free {
	struct ti_sci_msg_hdr hdr;
	u32 nav_id;
	u32 index;
	u8 secondary_host;
} __packed;

/**
 * struct ti_sci_msg_ring_reset - Resets a Navigator Subsystem ring
 * @hdr:	Generic Header
 * @nav_id:	SoC Navigator Subsystem device ID from which the ring was
 *		allocated
 * @index:	Ring index. Passing @RM_TISCI_MSG_NULL_RING_INDEX will result
 *		in a NACK.
 *
 * Reset a ring allocated via @ti_sci_msg_ring_allocate.  Resetting a
 * ring resets the ring's occupancies and pointers.  The host, or a
 * supervisor of the host, who owns the ring must be the requesting host.
 *
 * Request type is TI_SCI_MSG_RM_RING_RESET, response is a generic
 * ACK or NACK message.
 */
struct ti_sci_msg_ring_reset {
	struct ti_sci_msg_hdr hdr;
	u32 nav_id;
	u32 index;
} __packed;

/**
 * struct ti_sci_msg_ring_reconfig - Reconfigure a Navigator Subsystem ring
 *
 * Reconfigures the non-real-time register fields of a ring allocated via
 * @tisci_msg_rm_ring_allocate_req.  The host, or a supervisor of the host, who
 * owns the ring must be the requesting host.  The value of the non-real-time
 * register parameters prior to the reconfiguration taking place are returned
 * in @tisci_msg_rm_ring_reconfig_resp.
 *
 * @hdr:	Generic Header
 * @nav_id:	SoC Navigator Subsystem device ID from which the ring was
 *		allocated
 * @index:	Ring index. Passing @RM_TISCI_MSG_NULL_RING_INDEX will result
 *		in a NACK.
 * @addr_hi: See "addr_lo" description,
 * @addr_lo: The ring base address must be aligned to the "size" parameter
 *	setting when the ring is configured for RING or MESSAGE modes.
 *	For CREDENTIALS and QM modes the ring base address must be aligned to
 *	a 4k byte boundary 32 LSBs of ring base address to be programmed
 *	into the ring's RING_BA_LO register.
 * @count: Number of ring elements. Must be even if mode is CREDENTIALS or QM
 *	modes.
 * @mode: Specifies the mode the ring is to be configured as. Allowed values:
 *	@TI_SCI_RING_MODE_RING
 *	@TI_SCI_RING_MODE_MESSAGE
 *	@TI_SCI_RING_MODE_CREDENTIALS
 *	@TI_SCI_RING_MODE_QM
 * @size: Specifies the ring element size. To calculate the encoded size use
 *	the formula (log2(size_bytes) - 2), where size_bytes cannot be
 *	greater than 256.
 * @order_id: Specifies the ring's bus order ID. To use the default order ID
 *	this value must be @TI_SCI_RING_NULL_ORDER_ID. Otherwise, the specified
 *	order ID will be programmed as the ring's bus order ID.
 */
struct ti_sci_msg_ring_reconfig {
	struct ti_sci_msg_hdr	hdr;
	u32			nav_id;
	u32			index;
	u32			addr_lo;
	u32			addr_hi;
	u32			count;
	u8			mode;
	u8			size;
	u8			order_id;
} __packed;

/**
 * struct ti_sci_msg_ring_reconfig_resp - Ring reconfigure response message
 *
 * Response received by host processor after RM has handled
 * @ref tisci_msg_rm_ring_reconfig_req. The response contains the ring's
 * non-real-time register values before it was reconfigured.
 *
 * @hdr:	Generic Header
 * @old_addr_lo: Ring 32 LSBs of base address prior to the reconfiguration
 *		 taking place.
 * @old_addr_hi: Ring 16 MSBs of base address prior to the reconfiguration
 *		 taking place.
 * @old_count: Ring number of elements prior to the reconfiguration taking place
 * @old_mode: Ring mode prior to the reconfiguration taking place
 * @old_size: Ring element size prior to the reconfiguration taking place
 * @old_order_id: Ring order ID prior to the reconfiguration taking place
 */
struct ti_sci_msg_ring_reconfig_resp {
	struct ti_sci_msg_hdr	hdr;
	u32			old_addr_lo;
	u32			old_addr_hi;
	u32			old_count;
	u8			old_mode;
	u8			old_size;
	u8			old_order_id;
} __packed;

/**
 * struct ti_sci_msg_psil_pair - Pairs a PSI-L source thread to a destination
 *				 thread
 * @hdr:	Generic Header
 * @nav_id:	SoC Navigator Subsystem device ID whose PSI-L config proxy is
 *		used to pair the source and destination threads.
 * @src_thread:	PSI-L source thread ID within the PSI-L System thread map.
 *
 * UDMAP transmit channels mapped to source threads will have their
 * TCHAN_THRD_ID register programmed with the destination thread if the pairing
 * is successful.

 * @dst_thread: PSI-L destination thread ID within the PSI-L System thread map.
 * PSI-L destination threads start at index 0x8000.  The request is NACK'd if
 * the destination thread is not greater than or equal to 0x8000.
 *
 * UDMAP receive channels mapped to destination threads will have their
 * RCHAN_THRD_ID register programmed with the source thread if the pairing
 * is successful.
 *
 * Request type is TI_SCI_MSG_RM_PSIL_PAIR, response is a generic ACK or NACK
 * message.
 */
struct ti_sci_msg_psil_pair {
	struct ti_sci_msg_hdr hdr;
	u32 nav_id;
	u32 src_thread;
	u32 dst_thread;
} __packed;

/**
 * struct ti_sci_msg_psil_unpair - Unpairs a PSI-L source thread from a
 *				   destination thread
 * @hdr:	Generic Header
 * @nav_id:	SoC Navigator Subsystem device ID whose PSI-L config proxy is
 *		used to unpair the source and destination threads.
 * @src_thread:	PSI-L source thread ID within the PSI-L System thread map.
 *
 * UDMAP transmit channels mapped to source threads will have their
 * TCHAN_THRD_ID register cleared if the unpairing is successful.
 *
 * @dst_thread: PSI-L destination thread ID within the PSI-L System thread map.
 * PSI-L destination threads start at index 0x8000.  The request is NACK'd if
 * the destination thread is not greater than or equal to 0x8000.
 *
 * UDMAP receive channels mapped to destination threads will have their
 * RCHAN_THRD_ID register cleared if the unpairing is successful.
 *
 * Request type is TI_SCI_MSG_RM_PSIL_UNPAIR, response is a generic ACK or NACK
 * message.
 */
struct ti_sci_msg_psil_unpair {
	struct ti_sci_msg_hdr hdr;
	u32 nav_id;
	u32 src_thread;
	u32 dst_thread;
} __packed;

struct ti_sci_msg_udmap_tx_ch_alloc {
	struct ti_sci_msg_hdr hdr;
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
} __packed;

struct ti_sci_msg_udmap_tx_ch_alloc_resp {
	struct ti_sci_msg_hdr hdr;
	u32 index;
} __packed;

struct ti_sci_msg_udmap_tx_ch_free {
	struct ti_sci_msg_hdr hdr;
	u32 nav_id;
	u32 index;
	s8 secondary_host;
} __packed;

struct ti_sci_msg_udmap_rx_ch_alloc {
	struct ti_sci_msg_hdr hdr;
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
} __packed;

struct ti_sci_msg_udmap_rx_ch_alloc_resp {
	struct ti_sci_msg_hdr hdr;
	u32 index;
	u32 def_flow_index;
	u32 rng_flow_start_index;
	u32 rng_flow_cnt;
} __packed;

struct ti_sci_msg_udmap_rx_ch_free {
	struct ti_sci_msg_hdr hdr;
	u32 nav_id;
	u32 index;
	s8 secondary_host;
} __packed;

struct ti_sci_msg_udmap_rx_flow_cfg {
	struct ti_sci_msg_hdr hdr;
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
} __packed;

struct rm_ti_sci_msg_udmap_rx_flow_opt_cfg {
	struct ti_sci_msg_hdr hdr;
	u32 nav_id;
	u32 flow_index;
	u32 rx_ch_index;
	u16 rx_size_thresh0;
	u16 rx_size_thresh1;
	u16 rx_size_thresh2;
	u16 rx_fdq0_sz1_qnum;
	u16 rx_fdq0_sz2_qnum;
	u16 rx_fdq0_sz3_qnum;
} __attribute__((__packed__));

#endif /* __TI_SCI_H */
