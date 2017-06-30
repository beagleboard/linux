/**
 * @file
 * Real-Time Driver Model for RT-Socket-CAN, CAN device profile header
 *
 * @note Copyright (C) 2006 Wolfgang Grandegger <wg@grandegger.com>
 *
 * @note Copyright (C) 2005, 2006 Sebastian Smolorz
 *                         <Sebastian.Smolorz@stud.uni-hannover.de>
 *
 * This RTDM CAN device profile header is based on:
 *
 * include/linux/can.h, include/linux/socket.h, net/can/pf_can.h in
 * linux-can.patch, a CAN socket framework for Linux
 *
 * Copyright (C) 2004, 2005,
 * Robert Schwebel, Benedikt Spranger, Marc Kleine-Budde, Pengutronix
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#ifndef _RTDM_UAPI_CAN_H
#define _RTDM_UAPI_CAN_H

/**
 * @addtogroup rtdm_can
 * @{
 */

#define RTCAN_PROFILE_VER  2

#ifndef AF_CAN

/** CAN address family */
#define AF_CAN	29

/** CAN protocol family */
#define PF_CAN	AF_CAN

#endif

/** CAN socket levels
 *
 *  Used for @ref Sockopts for the particular protocols.
 */
#define SOL_CAN_RAW  103

/** Type of CAN id (see @ref CAN_xxx_MASK and @ref CAN_xxx_FLAG) */
typedef uint32_t can_id_t;
typedef uint32_t canid_t;

/** Type of CAN error mask */
typedef can_id_t can_err_mask_t;

/*!
 * @anchor CAN_xxx_MASK @name CAN ID masks
 * Bit masks for masking CAN IDs
 * @{ */

/** Bit mask for extended CAN IDs */
#define CAN_EFF_MASK  0x1FFFFFFF

/** Bit mask for standard CAN IDs */
#define CAN_SFF_MASK  0x000007FF

/** @} */

/*!
 * @anchor CAN_xxx_FLAG @name CAN ID flags
 * Flags within a CAN ID indicating special CAN frame attributes
 * @{ */
/** Extended frame */
#define CAN_EFF_FLAG  0x80000000
/** Remote transmission frame */
#define CAN_RTR_FLAG  0x40000000
/** Error frame (see @ref Errors), not valid in struct can_filter */
#define CAN_ERR_FLAG  0x20000000
/** Invert CAN filter definition, only valid in struct can_filter */
#define CAN_INV_FILTER CAN_ERR_FLAG

/** @} */

/*!
 * @anchor CAN_PROTO @name Particular CAN protocols
 * Possible protocols for the PF_CAN protocol family
 *
 * Currently only the RAW protocol is supported.
 * @{ */
/** Raw protocol of @c PF_CAN, applicable to socket type @c SOCK_RAW */
#define CAN_RAW  1
/** @} */

#define CAN_BAUDRATE_UNKNOWN       ((uint32_t)-1)
#define CAN_BAUDRATE_UNCONFIGURED  0

/**
 * Baudrate definition in bits per second
 */
typedef uint32_t can_baudrate_t;

/**
 * Supported CAN bit-time types
 */
enum CAN_BITTIME_TYPE {
	/** Standard bit-time definition according to Bosch */
	CAN_BITTIME_STD,
	/** Hardware-specific BTR bit-time definition */
	CAN_BITTIME_BTR
};

/**
 * See @ref CAN_BITTIME_TYPE
 */
typedef enum CAN_BITTIME_TYPE can_bittime_type_t;

/**
 * Standard bit-time parameters according to Bosch
 */
struct can_bittime_std {
	uint32_t brp;		/**< Baud rate prescaler */
	uint8_t prop_seg;	/**< from 1 to 8 */
	uint8_t phase_seg1;	/**< from 1 to 8 */
	uint8_t phase_seg2;	/**< from 1 to 8 */
	uint8_t sjw:7;		/**< from 1 to 4 */
	uint8_t sam:1;		/**< 1 - enable triple sampling */
};

/**
 * Hardware-specific BTR bit-times
 */
struct can_bittime_btr {

	uint8_t btr0;		/**< Bus timing register 0 */
	uint8_t btr1;		/**< Bus timing register 1 */
};

/**
 * Custom CAN bit-time definition
 */
struct can_bittime {
	/** Type of bit-time definition */
	can_bittime_type_t type;

	union {
		/** Standard bit-time */
		struct can_bittime_std std;
		/** Hardware-spcific BTR bit-time */
		struct can_bittime_btr btr;
	};
};

/*!
 * @anchor CAN_MODE @name CAN operation modes
 * Modes into which CAN controllers can be set
 * @{ */
enum CAN_MODE {
	/*! Set controller in Stop mode (no reception / transmission possible) */
	CAN_MODE_STOP = 0,

	/*! Set controller into normal operation. @n
	 *  Coming from stopped mode or bus off, the controller begins with no
	 *  errors in @ref CAN_STATE_ACTIVE. */
	CAN_MODE_START,

	/*! Set controller into Sleep mode. @n
	 *  This is only possible if the controller is not stopped or bus-off. @n
	 *  Notice that sleep mode will only be entered when there is no bus
	 *  activity. If the controller detects bus activity while "sleeping"
	 *  it will go into operating mode again. @n
	 *  To actively leave sleep mode again trigger @c CAN_MODE_START. */
	CAN_MODE_SLEEP
};
/** @} */

/** See @ref CAN_MODE */
typedef enum CAN_MODE can_mode_t;

/*!
 * @anchor CAN_CTRLMODE @name CAN controller modes
 * Special CAN controllers modes, which can be or'ed together.
 *
 * @note These modes are hardware-dependent. Please consult the hardware
 * manual of the CAN controller for more detailed information.
 *
 * @{ */

/*! Listen-Only mode
 *
 *  In this mode the CAN controller would give no acknowledge to the CAN-bus,
 *  even if a message is received successfully and messages would not be
 *  transmitted. This mode might be useful for bus-monitoring, hot-plugging
 *  or throughput analysis. */
#define CAN_CTRLMODE_LISTENONLY 0x1

/*! Loopback mode
 *
 * In this mode the CAN controller does an internal loop-back, a message is
 * transmitted and simultaneously received. That mode can be used for self
 * test operation. */
#define CAN_CTRLMODE_LOOPBACK   0x2

/*! Triple sampling mode
 *
 * In this mode the CAN controller uses Triple sampling. */
#define CAN_CTRLMODE_3_SAMPLES  0x4

/** @} */

/** See @ref CAN_CTRLMODE */
typedef int can_ctrlmode_t;

/*!
 * @anchor CAN_STATE @name CAN controller states
 * States a CAN controller can be in.
 * @{ */
enum CAN_STATE {
	/** CAN controller is error active */
	CAN_STATE_ERROR_ACTIVE = 0,
	/** CAN controller is active */
	CAN_STATE_ACTIVE = 0,

	/** CAN controller is error active, warning level is reached */
	CAN_STATE_ERROR_WARNING = 1,
	/** CAN controller is error active, warning level is reached */
	CAN_STATE_BUS_WARNING = 1,

	/** CAN controller is error passive */
	CAN_STATE_ERROR_PASSIVE = 2,
	/** CAN controller is error passive */
	CAN_STATE_BUS_PASSIVE = 2,

	/** CAN controller went into Bus Off */
	CAN_STATE_BUS_OFF,

	/** CAN controller is scanning to get the baudrate */
	CAN_STATE_SCANNING_BAUDRATE,

	/** CAN controller is in stopped mode */
	CAN_STATE_STOPPED,

	/** CAN controller is in Sleep mode */
	CAN_STATE_SLEEPING,
};
/** @} */

/** See @ref CAN_STATE */
typedef enum CAN_STATE can_state_t;

#define CAN_STATE_OPERATING(state) ((state) < CAN_STATE_BUS_OFF)

/**
 * Filter for reception of CAN messages.
 *
 * This filter works as follows:
 * A received CAN ID is AND'ed bitwise with @c can_mask and then compared to
 * @c can_id. This also includes the @ref CAN_EFF_FLAG and @ref CAN_RTR_FLAG
 * of @ref CAN_xxx_FLAG. If this comparison is true, the message will be
 * received by the socket. The logic can be inverted with the @c can_id flag
 * @ref CAN_INV_FILTER :
 *
 * @code
 * if (can_id & CAN_INV_FILTER) {
 *    if ((received_can_id & can_mask) != (can_id & ~CAN_INV_FILTER))
 *       accept-message;
 * } else {
 *    if ((received_can_id & can_mask) == can_id)
 *       accept-message;
 * }
 * @endcode
 *
 * Multiple filters can be arranged in a filter list and set with
 * @ref Sockopts. If one of these filters matches a CAN ID upon reception
 * of a CAN frame, this frame is accepted.
 *
 */
typedef struct can_filter {
	/** CAN ID which must match with incoming IDs after passing the mask.
	 *  The filter logic can be inverted with the flag @ref CAN_INV_FILTER. */
	uint32_t can_id;

	/** Mask which is applied to incoming IDs. See @ref CAN_xxx_MASK
	 *  "CAN ID masks" if exactly one CAN ID should come through. */
	uint32_t can_mask;
} can_filter_t;

/**
 * Socket address structure for the CAN address family
 */
struct sockaddr_can {
	/** CAN address family, must be @c AF_CAN */
	sa_family_t can_family;

	/** Interface index of CAN controller. See @ref SIOCGIFINDEX. */
	int can_ifindex;
};

/**
 * Raw CAN frame
 *
 * Central structure for receiving and sending CAN frames.
 */
typedef struct can_frame {
	/** CAN ID of the frame
	 *
	 *  See @ref CAN_xxx_FLAG "CAN ID flags" for special bits.
	 */
	can_id_t can_id;

	/** Size of the payload in bytes */
	uint8_t can_dlc;

	/** Payload data bytes */
	uint8_t data[8] __attribute__ ((aligned(8)));
} can_frame_t;

/**
 * CAN interface request descriptor
 *
 * Parameter block for submitting CAN control requests.
 */
struct can_ifreq {
	union {
		char	ifrn_name[IFNAMSIZ];
	} ifr_ifrn;
	
	union {
		struct can_bittime bittime;
		can_baudrate_t baudrate;
		can_ctrlmode_t ctrlmode;
		can_mode_t mode;
		can_state_t state;
		int ifru_ivalue;
	} ifr_ifru;
};

/*!
 * @anchor RTCAN_TIMESTAMPS   @name Timestamp switches
 * Arguments to pass to @ref RTCAN_RTIOC_TAKE_TIMESTAMP
 * @{ */
#define RTCAN_TAKE_NO_TIMESTAMPS	0  /**< Switch off taking timestamps */
#define RTCAN_TAKE_TIMESTAMPS		1  /**< Do take timestamps */
/** @} */

#define RTIOC_TYPE_CAN  RTDM_CLASS_CAN

/*!
 * @anchor Rawsockopts @name RAW socket options
 * Setting and getting CAN RAW socket options.
 * @{ */

/**
 * CAN filter definition
 *
 * A CAN raw filter list with elements of struct can_filter can be installed
 * with @c setsockopt. This list is used upon reception of CAN frames to
 * decide whether the bound socket will receive a frame. An empty filter list
 * can also be defined using optlen = 0, which is recommanded for write-only
 * sockets.
 * @n
 * If the socket was already bound with @ref Bind, the old filter list
 * gets replaced with the new one. Be aware that already received, but
 * not read out CAN frames may stay in the socket buffer.
 * @n
 * @n
 * @param [in] level @b SOL_CAN_RAW
 *
 * @param [in] optname @b CAN_RAW_FILTER
 *
 * @param [in] optval Pointer to array of struct can_filter.
 *
 * @param [in] optlen Size of filter list: count * sizeof( struct can_filter).
 * @n
 * @coretags{task-unrestricted}
 * @n
 * Specific return values:
 * - -EFAULT (It was not possible to access user space memory area at the
 *            specified address.)
 * - -ENOMEM (Not enough memory to fulfill the operation)
 * - -EINVAL (Invalid length "optlen")
 * - -ENOSPC (No space to store filter list, check RT-Socket-CAN kernel
 *            parameters)
 * .
 */
#define CAN_RAW_FILTER		0x1

/**
 * CAN error mask
 *
 * A CAN error mask (see @ref Errors) can be set with @c setsockopt. This
 * mask is then used to decide if error frames are delivered to this socket
 * in case of error condidtions. The error frames are marked with the
 * @ref CAN_ERR_FLAG of @ref CAN_xxx_FLAG and must be handled by the
 * application properly. A detailed description of the errors can be
 * found in the @c can_id and the @c data fields of struct can_frame
 * (see @ref Errors for futher details).
 *
 * @n
 * @param [in] level @b SOL_CAN_RAW
 *
 * @param [in] optname @b CAN_RAW_ERR_FILTER
 *
 * @param [in] optval Pointer to error mask of type can_err_mask_t.
 *
 * @param [in] optlen Size of error mask: sizeof(can_err_mask_t).
 *
 * @coretags{task-unrestricted}
 * @n
 * Specific return values:
 * - -EFAULT (It was not possible to access user space memory area at the
 *            specified address.)
 * - -EINVAL (Invalid length "optlen")
 * .
 */
#define CAN_RAW_ERR_FILTER	0x2

/**
 * CAN TX loopback
 *
 * The TX loopback to other local sockets can be selected with this
 * @c setsockopt.
 *
 * @note The TX loopback feature must be enabled in the kernel and then
 * the loopback to other local TX sockets is enabled by default.
 *
 * @n
 * @param [in] level @b SOL_CAN_RAW
 *
 * @param [in] optname @b CAN_RAW_LOOPBACK
 *
 * @param [in] optval Pointer to integer value.
 *
 * @param [in] optlen Size of int: sizeof(int).
 *
 * @coretags{task-unrestricted}
 * @n
 * Specific return values:
 * - -EFAULT (It was not possible to access user space memory area at the
 *            specified address.)
 * - -EINVAL (Invalid length "optlen")
 * - -EOPNOTSUPP (not supported, check RT-Socket-CAN kernel parameters).
 */
#define CAN_RAW_LOOPBACK	0x3

/**
 * CAN receive own messages
 *
 * Not supported by RT-Socket-CAN, but defined for compatibility with
 * Socket-CAN.
 */
#define CAN_RAW_RECV_OWN_MSGS   0x4

/** @} */

/*!
 * @anchor CANIOCTLs @name IOCTLs
 * CAN device IOCTLs
 *
 * @deprecated Passing <TT>struct ifreq<TT> as a request descriptor
 * for CAN IOCTLs is still accepted for backward compatibility,
 * however it is recommended to switch to <TT>struct can_ifreq<TT> at
 * the first opportunity.
 *
 * @{ */

/**
 * Get CAN interface index by name
 *
 * @param [in,out] arg Pointer to interface request structure buffer
 *                     (<TT>struct can_ifreq</TT>). If
 *                     <TT>ifr_name</TT> holds a valid CAN interface
 *                     name <TT>ifr_ifindex</TT> will be filled with
 *                     the corresponding interface index.
 *
 * @return 0 on success, otherwise:
 * - -EFAULT: It was not possible to access user space memory area at the
 *            specified address.
 * - -ENODEV: No device with specified name exists.
 *
 * @coretags{task-unrestricted}
 */
#ifdef DOXYGEN_CPP /* For Doxygen only, already defined by kernel headers */
#define SIOCGIFINDEX defined_by_kernel_header_file
#endif

/**
 * Set baud rate
 *
 * The baudrate must be specified in bits per second. The driver will
 * try to calculate resonable CAN bit-timing parameters. You can use
 * @ref SIOCSCANCUSTOMBITTIME to set custom bit-timing.
 *
 * @param [in] arg Pointer to interface request structure buffer
 *                 (<TT>struct can_ifreq</TT>).
 *                 <TT>ifr_name</TT> must hold a valid CAN interface name,
 *                 <TT>ifr_ifru</TT> must be filled with an instance of
 *                 @ref can_baudrate_t.
 *
 * @return 0 on success, otherwise:
 * - -EFAULT: It was not possible to access user space memory area at the
 *            specified address.
 * - -ENODEV: No device with specified name exists.
 * - -EINVAL: No valid baud rate, see @ref can_baudrate_t.
 * - -EDOM  : Baud rate not possible.
 * - -EAGAIN: Request could not be successully fulfilled. Try again.
 *
 * @coretags{task-unrestricted, might-switch}
 *
 * @note Setting the baud rate is a configuration task. It should
 * be done deliberately or otherwise CAN messages will likely be lost.
 */
#define SIOCSCANBAUDRATE	_IOW(RTIOC_TYPE_CAN, 0x01, struct can_ifreq)

/**
 * Get baud rate
 *
 * @param [in,out] arg Pointer to interface request structure buffer
 *                    (<TT>struct can_ifreq</TT>).
 *                    <TT>ifr_name</TT> must hold a valid CAN interface name,
 *                    <TT>ifr_ifru</TT> will be filled with an instance of
 *                    @ref can_baudrate_t.
 *
 * @return 0 on success, otherwise:
 * - -EFAULT: It was not possible to access user space memory area at the
 *            specified address.
 * - -ENODEV: No device with specified name exists.
 * - -EINVAL: No baud rate was set yet.
 *
 * @coretags{task-unrestricted}
 */
#define SIOCGCANBAUDRATE	_IOWR(RTIOC_TYPE_CAN, 0x02, struct can_ifreq)

/**
 * Set custom bit time parameter
 *
 * Custem-bit time could be defined in various formats (see
 * struct can_bittime).
 *
 * @param [in] arg Pointer to interface request structure buffer
 *                 (<TT>struct can_ifreq</TT>).
 *                 <TT>ifr_name</TT> must hold a valid CAN interface name,
 *                 <TT>ifr_ifru</TT> must be filled with an instance of
 *                 struct can_bittime.
 *
 * @return 0 on success, otherwise:
 * - -EFAULT: It was not possible to access user space memory area at the
 *            specified address.
 * - -ENODEV: No device with specified name exists.
 * - -EINVAL: No valid baud rate, see @ref can_baudrate_t.
 * - -EAGAIN: Request could not be successully fulfilled. Try again.
 *
 * @coretags{task-unrestricted, might-switch}
 *
 * @note Setting the bit-time is a configuration task. It should
 * be done deliberately or otherwise CAN messages will likely be lost.
 */
#define SIOCSCANCUSTOMBITTIME	_IOW(RTIOC_TYPE_CAN, 0x03, struct can_ifreq)

/**
 * Get custom bit-time parameters
 *
 * @param [in,out] arg Pointer to interface request structure buffer
 *                    (<TT>struct can_ifreq</TT>).
 *                    <TT>ifr_name</TT> must hold a valid CAN interface name,
 *                    <TT>ifr_ifru</TT> will be filled with an instance of
 *                    struct can_bittime.
 *
 * @return 0 on success, otherwise:
 * - -EFAULT: It was not possible to access user space memory area at the
 *            specified address.
 * - -ENODEV: No device with specified name exists.
 * - -EINVAL: No baud rate was set yet.
 *
 * @coretags{task-unrestricted}
 */
#define SIOCGCANCUSTOMBITTIME	_IOWR(RTIOC_TYPE_CAN, 0x04, struct can_ifreq)

/**
 * Set operation mode of CAN controller
 *
 * See @ref CAN_MODE "CAN controller modes" for available modes.
 *
 * @param [in] arg Pointer to interface request structure buffer
 *                 (<TT>struct can_ifreq</TT>).
 *                 <TT>ifr_name</TT> must hold a valid CAN interface name,
 *                 <TT>ifr_ifru</TT> must be filled with an instance of
 *                 @ref can_mode_t.
 *
 * @return 0 on success, otherwise:
 * - -EFAULT: It was not possible to access user space memory area at the
 *            specified address.
 * - -ENODEV: No device with specified name exists.
 * - -EAGAIN: (@ref CAN_MODE_START, @ref CAN_MODE_STOP) Could not successfully
 *            set mode, hardware is busy. Try again.
 * - -EINVAL: (@ref CAN_MODE_START) Cannot start controller,
 *            set baud rate first.
 * - -ENETDOWN: (@ref CAN_MODE_SLEEP) Cannot go into sleep mode because
		controller is stopped or bus off.
 * - -EOPNOTSUPP: unknown mode
 *
 * @coretags{task-unrestricted, might-switch}
 *
 * @note Setting a CAN controller into normal operation after a bus-off can
 * take some time (128 occurrences of 11 consecutive recessive bits).
 * In such a case, although this IOCTL will return immediately with success
 * and @ref SIOCGCANSTATE will report @ref CAN_STATE_ACTIVE,
 * bus-off recovery may still be in progress. @n
 * If a controller is bus-off, setting it into stop mode will return no error
 * but the controller remains bus-off.
 */
#define SIOCSCANMODE		_IOW(RTIOC_TYPE_CAN, 0x05, struct can_ifreq)

/**
 * Get current state of CAN controller
 *
 * States are divided into main states and additional error indicators. A CAN
 * controller is always in exactly one main state. CAN bus errors are
 * registered by the CAN hardware and collected by the driver. There is one
 * error indicator (bit) per error type. If this IOCTL is triggered the error
 * types which occured since the last call of this IOCTL are reported and
 * thereafter the error indicators are cleared. See also
 * @ref CAN_STATE "CAN controller states".
 *
 * @param [in,out] arg Pointer to interface request structure buffer
 *                    (<TT>struct can_ifreq</TT>).
 *                    <TT>ifr_name</TT> must hold a valid CAN interface name,
 *                    <TT>ifr_ifru</TT> will be filled with an instance of
 *                    @ref can_mode_t.
 *
 * @return 0 on success, otherwise:
 * - -EFAULT: It was not possible to access user space memory area at the
 *            specified address.
 * - -ENODEV: No device with specified name exists.
 *
 * @coretags{task-unrestricted, might-switch}
 */
#define SIOCGCANSTATE		_IOWR(RTIOC_TYPE_CAN, 0x06, struct can_ifreq)

/**
 * Set special controller modes
 *
 * Various special controller modes could be or'ed together (see
 * @ref CAN_CTRLMODE for further information).
 *
 * @param [in] arg Pointer to interface request structure buffer
 *                 (<TT>struct can_ifreq</TT>).
 *                 <TT>ifr_name</TT> must hold a valid CAN interface name,
 *                 <TT>ifr_ifru</TT> must be filled with an instance of
 *                 @ref can_ctrlmode_t.
 *
 * @return 0 on success, otherwise:
 * - -EFAULT: It was not possible to access user space memory area at the
 *            specified address.
 * - -ENODEV: No device with specified name exists.
 * - -EINVAL: No valid baud rate, see @ref can_baudrate_t.
 * - -EAGAIN: Request could not be successully fulfilled. Try again.
 *
 * @coretags{task-unrestricted, might-switch}
 *
 * @note Setting special controller modes is a configuration task. It should
 * be done deliberately or otherwise CAN messages will likely be lost.
 */
#define SIOCSCANCTRLMODE	_IOW(RTIOC_TYPE_CAN, 0x07, struct can_ifreq)

/**
 * Get special controller modes
 *
 *
 * @param [in] arg Pointer to interface request structure buffer
 *                 (<TT>struct can_ifreq</TT>).
 *                 <TT>ifr_name</TT> must hold a valid CAN interface name,
 *                 <TT>ifr_ifru</TT> must be filled with an instance of
 *                 @ref can_ctrlmode_t.
 *
 * @return 0 on success, otherwise:
 * - -EFAULT: It was not possible to access user space memory area at the
 *            specified address.
 * - -ENODEV: No device with specified name exists.
 * - -EINVAL: No baud rate was set yet.
 *
 * @coretags{task-unrestricted, might-switch}
 */
#define SIOCGCANCTRLMODE	_IOWR(RTIOC_TYPE_CAN, 0x08, struct can_ifreq)

/**
 * Enable or disable storing a high precision timestamp upon reception of
 * a CAN frame.
 *
 * A newly created socket takes no timestamps by default.
 *
 * @param [in] arg int variable, see @ref RTCAN_TIMESTAMPS "Timestamp switches"
 *
 * @return 0 on success.
 *
 * @coretags{task-unrestricted}
 *
 * @note Activating taking timestamps only has an effect on newly received
 * CAN messages from the bus. Frames that already are in the socket buffer do
 * not have timestamps if it was deactivated before. See @ref Recv "Receive"
 * for more details.
 */
#define RTCAN_RTIOC_TAKE_TIMESTAMP _IOW(RTIOC_TYPE_CAN, 0x09, int)

/**
 * Specify a reception timeout for a socket
 *
 * Defines a timeout for all receive operations via a
 * socket which will take effect when one of the @ref Recv "receive functions"
 * is called without the @c MSG_DONTWAIT flag set.
 *
 * The default value for a newly created socket is an infinite timeout.
 *
 * @note The setting of the timeout value is not done atomically to avoid
 * locks. Please set the value before receiving messages from the socket.
 *
 * @param [in] arg Pointer to @ref nanosecs_rel_t variable. The value is
 *                interpreted as relative timeout in nanoseconds in case
 *                of a positive value.
 *                See @ref RTDM_TIMEOUT_xxx "Timeouts" for special timeouts.
 *
 * @return 0 on success, otherwise:
 * - -EFAULT: It was not possible to access user space memory area at the
 *            specified address.
 *
 * @coretags{task-unrestricted}
 */
#define RTCAN_RTIOC_RCV_TIMEOUT	_IOW(RTIOC_TYPE_CAN, 0x0A, nanosecs_rel_t)

/**
 * Specify a transmission timeout for a socket
 *
 * Defines a timeout for all send operations via a
 * socket which will take effect when one of the @ref Send "send functions"
 * is called without the @c MSG_DONTWAIT flag set.
 *
 * The default value for a newly created socket is an infinite timeout.
 *
 * @note The setting of the timeout value is not done atomically to avoid
 * locks. Please set the value before sending messages to the socket.
 *
 * @param [in] arg Pointer to @ref nanosecs_rel_t variable. The value is
 *                interpreted as relative timeout in nanoseconds in case
 *                of a positive value.
 *                See @ref RTDM_TIMEOUT_xxx "Timeouts" for special timeouts.
 *
 * @return 0 on success, otherwise:
 * - -EFAULT: It was not possible to access user space memory area at the
 *            specified address.
 *
 * @coretags{task-unrestricted}
 */
#define RTCAN_RTIOC_SND_TIMEOUT	_IOW(RTIOC_TYPE_CAN, 0x0B, nanosecs_rel_t)
/** @} */

#define CAN_ERR_DLC  8	/* dlc for error frames */

/*!
 * @anchor Errors @name Error mask
 * Error class (mask) in @c can_id field of struct can_frame to
 * be used with @ref CAN_RAW_ERR_FILTER.
 *
 * @b Note: Error reporting is hardware dependent and most CAN controllers
 * report less detailed error conditions than the SJA1000.
 *
 * @b Note: In case of a bus-off error condition (@ref CAN_ERR_BUSOFF), the
 * CAN controller is @b not restarted automatically. It is the application's
 * responsibility to react appropriately, e.g. calling @ref CAN_MODE_START.
 *
 * @b Note: Bus error interrupts (@ref CAN_ERR_BUSERROR) are enabled when an
 * application is calling a @ref Recv function on a socket listening
 * on bus errors (using @ref CAN_RAW_ERR_FILTER). After one bus error has
 * occured, the interrupt will be disabled to allow the application time for
 * error processing and to efficiently avoid bus error interrupt flooding.
 * @{ */

/** TX timeout (netdevice driver) */
#define CAN_ERR_TX_TIMEOUT	0x00000001U

/** Lost arbitration (see @ref Error0 "data[0]") */
#define CAN_ERR_LOSTARB		0x00000002U

/** Controller problems (see @ref Error1 "data[1]") */
#define CAN_ERR_CRTL		0x00000004U

/** Protocol violations (see @ref Error2 "data[2]",
			     @ref Error3 "data[3]") */
#define CAN_ERR_PROT		0x00000008U

/** Transceiver status (see @ref Error4 "data[4]")    */
#define CAN_ERR_TRX		0x00000010U

/** Received no ACK on transmission */
#define CAN_ERR_ACK		0x00000020U

/** Bus off */
#define CAN_ERR_BUSOFF		0x00000040U

/** Bus error (may flood!) */
#define CAN_ERR_BUSERROR	0x00000080U

/** Controller restarted */
#define CAN_ERR_RESTARTED	0x00000100U

/** Omit EFF, RTR, ERR flags */
#define CAN_ERR_MASK		0x1FFFFFFFU

/** @} */

/*!
 * @anchor Error0 @name Arbitration lost error
 * Error in the data[0] field of struct can_frame.
 * @{ */
/* arbitration lost in bit ... / data[0] */
#define CAN_ERR_LOSTARB_UNSPEC	0x00 /**< unspecified */
				     /**< else bit number in bitstream */
/** @} */

/*!
 * @anchor Error1 @name Controller problems
 * Error in the data[1] field of struct can_frame.
 * @{ */
/* error status of CAN-controller / data[1] */
#define CAN_ERR_CRTL_UNSPEC	 0x00 /**< unspecified */
#define CAN_ERR_CRTL_RX_OVERFLOW 0x01 /**< RX buffer overflow */
#define CAN_ERR_CRTL_TX_OVERFLOW 0x02 /**< TX buffer overflow */
#define CAN_ERR_CRTL_RX_WARNING	 0x04 /**< reached warning level for RX errors */
#define CAN_ERR_CRTL_TX_WARNING	 0x08 /**< reached warning level for TX errors */
#define CAN_ERR_CRTL_RX_PASSIVE	 0x10 /**< reached passive level for RX errors */
#define CAN_ERR_CRTL_TX_PASSIVE	 0x20 /**< reached passive level for TX errors */
/** @} */

/*!
 * @anchor Error2 @name Protocol error type
 * Error in the data[2] field of struct can_frame.
 * @{ */
/* error in CAN protocol (type) / data[2] */
#define CAN_ERR_PROT_UNSPEC	0x00 /**< unspecified */
#define CAN_ERR_PROT_BIT	0x01 /**< single bit error */
#define CAN_ERR_PROT_FORM	0x02 /**< frame format error */
#define CAN_ERR_PROT_STUFF	0x04 /**< bit stuffing error */
#define CAN_ERR_PROT_BIT0	0x08 /**< unable to send dominant bit */
#define CAN_ERR_PROT_BIT1	0x10 /**< unable to send recessive bit */
#define CAN_ERR_PROT_OVERLOAD	0x20 /**< bus overload */
#define CAN_ERR_PROT_ACTIVE	0x40 /**< active error announcement */
#define CAN_ERR_PROT_TX		0x80 /**< error occured on transmission */
/** @} */

/*!
 * @anchor Error3 @name Protocol error location
 * Error in the data[3] field of struct can_frame.
 * @{ */
/* error in CAN protocol (location) / data[3] */
#define CAN_ERR_PROT_LOC_UNSPEC	 0x00 /**< unspecified */
#define CAN_ERR_PROT_LOC_SOF	 0x03 /**< start of frame */
#define CAN_ERR_PROT_LOC_ID28_21 0x02 /**< ID bits 28 - 21 (SFF: 10 - 3) */
#define CAN_ERR_PROT_LOC_ID20_18 0x06 /**< ID bits 20 - 18 (SFF: 2 - 0 )*/
#define CAN_ERR_PROT_LOC_SRTR	 0x04 /**< substitute RTR (SFF: RTR) */
#define CAN_ERR_PROT_LOC_IDE	 0x05 /**< identifier extension */
#define CAN_ERR_PROT_LOC_ID17_13 0x07 /**< ID bits 17-13 */
#define CAN_ERR_PROT_LOC_ID12_05 0x0F /**< ID bits 12-5 */
#define CAN_ERR_PROT_LOC_ID04_00 0x0E /**< ID bits 4-0 */
#define CAN_ERR_PROT_LOC_RTR	 0x0C /**< RTR */
#define CAN_ERR_PROT_LOC_RES1	 0x0D /**< reserved bit 1 */
#define CAN_ERR_PROT_LOC_RES0	 0x09 /**< reserved bit 0 */
#define CAN_ERR_PROT_LOC_DLC	 0x0B /**< data length code */
#define CAN_ERR_PROT_LOC_DATA	 0x0A /**< data section */
#define CAN_ERR_PROT_LOC_CRC_SEQ 0x08 /**< CRC sequence */
#define CAN_ERR_PROT_LOC_CRC_DEL 0x18 /**< CRC delimiter */
#define CAN_ERR_PROT_LOC_ACK	 0x19 /**< ACK slot */
#define CAN_ERR_PROT_LOC_ACK_DEL 0x1B /**< ACK delimiter */
#define CAN_ERR_PROT_LOC_EOF	 0x1A /**< end of frame */
#define CAN_ERR_PROT_LOC_INTERM	 0x12 /**< intermission */
/** @} */

/*!
 * @anchor Error4 @name Protocol error location
 * Error in the data[4] field of struct can_frame.
 * @{ */
/* error status of CAN-transceiver / data[4] */
/*                                               CANH CANL */
#define CAN_ERR_TRX_UNSPEC		0x00 /**< 0000 0000 */
#define CAN_ERR_TRX_CANH_NO_WIRE	0x04 /**< 0000 0100 */
#define CAN_ERR_TRX_CANH_SHORT_TO_BAT	0x05 /**< 0000 0101 */
#define CAN_ERR_TRX_CANH_SHORT_TO_VCC	0x06 /**< 0000 0110 */
#define CAN_ERR_TRX_CANH_SHORT_TO_GND	0x07 /**< 0000 0111 */
#define CAN_ERR_TRX_CANL_NO_WIRE	0x40 /**< 0100 0000 */
#define CAN_ERR_TRX_CANL_SHORT_TO_BAT	0x50 /**< 0101 0000 */
#define CAN_ERR_TRX_CANL_SHORT_TO_VCC	0x60 /**< 0110 0000 */
#define CAN_ERR_TRX_CANL_SHORT_TO_GND	0x70 /**< 0111 0000 */
#define CAN_ERR_TRX_CANL_SHORT_TO_CANH	0x80 /**< 1000 0000 */
/** @} */

/** @} */

#endif /* !_RTDM_UAPI_CAN_H */
