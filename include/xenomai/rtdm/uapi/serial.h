/**
 * @file
 * Real-Time Driver Model for Xenomai, serial device profile header
 *
 * @note Copyright (C) 2005-2007 Jan Kiszka <jan.kiszka@web.de>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA.
 *
 * @ingroup rtserial
 */
#ifndef _RTDM_UAPI_SERIAL_H
#define _RTDM_UAPI_SERIAL_H

#define RTSER_PROFILE_VER		3

/*!
 * @anchor RTSER_DEF_BAUD   @name RTSER_DEF_BAUD
 * Default baud rate
 * @{ */
#define RTSER_DEF_BAUD			9600
/** @} */

/*!
 * @anchor RTSER_xxx_PARITY   @name RTSER_xxx_PARITY
 * Number of parity bits
 * @{ */
#define RTSER_NO_PARITY			0x00
#define RTSER_ODD_PARITY		0x01
#define RTSER_EVEN_PARITY		0x03
#define RTSER_DEF_PARITY		RTSER_NO_PARITY
/** @} */

/*!
 * @anchor RTSER_xxx_BITS   @name RTSER_xxx_BITS
 * Number of data bits
 * @{ */
#define RTSER_5_BITS			0x00
#define RTSER_6_BITS			0x01
#define RTSER_7_BITS			0x02
#define RTSER_8_BITS			0x03
#define RTSER_DEF_BITS			RTSER_8_BITS
/** @} */

/*!
 * @anchor RTSER_xxx_STOPB   @name RTSER_xxx_STOPB
 * Number of stop bits
 * @{ */
#define RTSER_1_STOPB			0x00
/** valid only in combination with 5 data bits */
#define RTSER_1_5_STOPB			0x01
#define RTSER_2_STOPB			0x01
#define RTSER_DEF_STOPB			RTSER_1_STOPB
/** @} */

/*!
 * @anchor RTSER_xxx_HAND   @name RTSER_xxx_HAND
 * Handshake mechanisms
 * @{ */
#define RTSER_NO_HAND			0x00
#define RTSER_RTSCTS_HAND		0x01
#define RTSER_DEF_HAND			RTSER_NO_HAND
/** @} */

/*!
 * @anchor RTSER_RS485_xxx   @name RTSER_RS485_xxx
 * RS485 mode with automatic RTS handling
 * @{ */
#define RTSER_RS485_DISABLE		0x00
#define RTSER_RS485_ENABLE		0x01
#define RTSER_DEF_RS485			RTSER_RS485_DISABLE
/** @} */

/*!
 * @anchor RTSER_FIFO_xxx   @name RTSER_FIFO_xxx
 * Reception FIFO interrupt threshold
 * @{ */
#define RTSER_FIFO_DEPTH_1		0x00
#define RTSER_FIFO_DEPTH_4		0x40
#define RTSER_FIFO_DEPTH_8		0x80
#define RTSER_FIFO_DEPTH_14		0xC0
#define RTSER_DEF_FIFO_DEPTH		RTSER_FIFO_DEPTH_1
/** @} */

/*!
 * @anchor RTSER_TIMEOUT_xxx   @name RTSER_TIMEOUT_xxx
 * Special timeout values, see also @ref RTDM_TIMEOUT_xxx
 * @{ */
#define RTSER_TIMEOUT_INFINITE		RTDM_TIMEOUT_INFINITE
#define RTSER_TIMEOUT_NONE		RTDM_TIMEOUT_NONE
#define RTSER_DEF_TIMEOUT		RTDM_TIMEOUT_INFINITE
/** @} */

/*!
 * @anchor RTSER_xxx_TIMESTAMP_HISTORY   @name RTSER_xxx_TIMESTAMP_HISTORY
 * Timestamp history control
 * @{ */
#define RTSER_RX_TIMESTAMP_HISTORY	0x01
#define RTSER_DEF_TIMESTAMP_HISTORY	0x00
/** @} */

/*!
 * @anchor RTSER_EVENT_xxx   @name RTSER_EVENT_xxx
 * Events bits
 * @{ */
#define RTSER_EVENT_RXPEND		0x01
#define RTSER_EVENT_ERRPEND		0x02
#define RTSER_EVENT_MODEMHI		0x04
#define RTSER_EVENT_MODEMLO		0x08
#define RTSER_EVENT_TXEMPTY		0x10
#define RTSER_DEF_EVENT_MASK		0x00
/** @} */


/*!
 * @anchor RTSER_SET_xxx   @name RTSER_SET_xxx
 * Configuration mask bits
 * @{ */
#define RTSER_SET_BAUD			0x0001
#define RTSER_SET_PARITY		0x0002
#define RTSER_SET_DATA_BITS		0x0004
#define RTSER_SET_STOP_BITS		0x0008
#define RTSER_SET_HANDSHAKE		0x0010
#define RTSER_SET_FIFO_DEPTH		0x0020
#define RTSER_SET_TIMEOUT_RX		0x0100
#define RTSER_SET_TIMEOUT_TX		0x0200
#define RTSER_SET_TIMEOUT_EVENT		0x0400
#define RTSER_SET_TIMESTAMP_HISTORY	0x0800
#define RTSER_SET_EVENT_MASK		0x1000
#define RTSER_SET_RS485			0x2000
/** @} */


/*!
 * @anchor RTSER_LSR_xxx   @name RTSER_LSR_xxx
 * Line status bits
 * @{ */
#define RTSER_LSR_DATA			0x01
#define RTSER_LSR_OVERRUN_ERR		0x02
#define RTSER_LSR_PARITY_ERR		0x04
#define RTSER_LSR_FRAMING_ERR		0x08
#define RTSER_LSR_BREAK_IND		0x10
#define RTSER_LSR_THR_EMTPY		0x20
#define RTSER_LSR_TRANSM_EMPTY		0x40
#define RTSER_LSR_FIFO_ERR		0x80
#define RTSER_SOFT_OVERRUN_ERR		0x0100
/** @} */


/*!
 * @anchor RTSER_MSR_xxx   @name RTSER_MSR_xxx
 * Modem status bits
 * @{ */
#define RTSER_MSR_DCTS			0x01
#define RTSER_MSR_DDSR			0x02
#define RTSER_MSR_TERI			0x04
#define RTSER_MSR_DDCD			0x08
#define RTSER_MSR_CTS			0x10
#define RTSER_MSR_DSR			0x20
#define RTSER_MSR_RI			0x40
#define RTSER_MSR_DCD			0x80
/** @} */


/*!
 * @anchor RTSER_MCR_xxx   @name RTSER_MCR_xxx
 * Modem control bits
 * @{ */
#define RTSER_MCR_DTR			0x01
#define RTSER_MCR_RTS			0x02
#define RTSER_MCR_OUT1			0x04
#define RTSER_MCR_OUT2			0x08
#define RTSER_MCR_LOOP			0x10
/** @} */


/*!
 * @anchor RTSER_BREAK_xxx   @name RTSER_BREAK_xxx
 * Break control
 * @{ */
#define RTSER_BREAK_CLR			0x00
#define RTSER_BREAK_SET			0x01


/**
 * Serial device configuration
 */
typedef struct rtser_config {
	/** mask specifying valid fields, see @ref RTSER_SET_xxx */
	int		config_mask;

	/** baud rate, default @ref RTSER_DEF_BAUD */
	int		baud_rate;

	/** number of parity bits, see @ref RTSER_xxx_PARITY */
	int		parity;

	/** number of data bits, see @ref RTSER_xxx_BITS */
	int		data_bits;

	/** number of stop bits, see @ref RTSER_xxx_STOPB */
	int		stop_bits;

	/** handshake mechanisms, see @ref RTSER_xxx_HAND */
	int		handshake;

	/** reception FIFO interrupt threshold, see @ref RTSER_FIFO_xxx */
	int		fifo_depth;

	int		reserved;

	/** reception timeout, see @ref RTSER_TIMEOUT_xxx for special
	 *  values */
	nanosecs_rel_t	rx_timeout;

	/** transmission timeout, see @ref RTSER_TIMEOUT_xxx for special
	 *  values */
	nanosecs_rel_t	tx_timeout;

	/** event timeout, see @ref RTSER_TIMEOUT_xxx for special values */
	nanosecs_rel_t	event_timeout;

	/** enable timestamp history, see @ref RTSER_xxx_TIMESTAMP_HISTORY */
	int		timestamp_history;

	/** event mask to be used with @ref RTSER_RTIOC_WAIT_EVENT, see
	 *  @ref RTSER_EVENT_xxx */
	int		event_mask;

	/** enable RS485 mode, see @ref RTSER_RS485_xxx */
	int		rs485;
} rtser_config_t;

/**
 * Serial device status
 */
typedef struct rtser_status {
	/** line status register, see @ref RTSER_LSR_xxx */
	int		line_status;

	/** modem status register, see @ref RTSER_MSR_xxx */
	int		modem_status;
} rtser_status_t;

/**
 * Additional information about serial device events
 */
typedef struct rtser_event {
	/** signalled events, see @ref RTSER_EVENT_xxx */
	int		events;

	/** number of pending input characters */
	int		rx_pending;

	/** last interrupt timestamp */
	nanosecs_abs_t	last_timestamp;

	/** reception timestamp of oldest character in input queue */
	nanosecs_abs_t	rxpend_timestamp;
} rtser_event_t;


#define RTIOC_TYPE_SERIAL		RTDM_CLASS_SERIAL


/*!
 * @name Sub-Classes of RTDM_CLASS_SERIAL
 * @{ */
#define RTDM_SUBCLASS_16550A		0
/** @} */


/*!
 * @anchor SERIOCTLs @name IOCTLs
 * Serial device IOCTLs
 * @{ */

/**
 * Get serial device configuration
 *
 * @param[out] arg Pointer to configuration buffer (struct rtser_config)
 *
 * @return 0 on success, otherwise negative error code
 *
 * @coretags{task-unrestricted}
 */
#define RTSER_RTIOC_GET_CONFIG	\
	_IOR(RTIOC_TYPE_SERIAL, 0x00, struct rtser_config)

/**
 * Set serial device configuration
 *
 * @param[in] arg Pointer to configuration buffer (struct rtser_config)
 *
 * @return 0 on success, otherwise:
 *
 * - -EPERM is returned if the caller's context is invalid, see note below.
 *
 * - -ENOMEM is returned if a new history buffer for timestamps cannot be
 * allocated.
 *
 * @coretags{task-unrestricted}
 *
 * @note If rtser_config contains a valid timestamp_history and the
 * addressed device has been opened in non-real-time context, this IOCTL must
 * be issued in non-real-time context as well. Otherwise, this command will
 * fail.
 */
#define RTSER_RTIOC_SET_CONFIG	\
	_IOW(RTIOC_TYPE_SERIAL, 0x01, struct rtser_config)

/**
 * Get serial device status
 *
 * @param[out] arg Pointer to status buffer (struct rtser_status)
 *
 * @return 0 on success, otherwise negative error code
 *
 * @coretags{task-unrestricted}
 *
 * @note The error states @c RTSER_LSR_OVERRUN_ERR, @c RTSER_LSR_PARITY_ERR,
 * @c RTSER_LSR_FRAMING_ERR, and @c RTSER_SOFT_OVERRUN_ERR that may have
 * occured during previous read accesses to the device will be saved for being
 * reported via this IOCTL. Upon return from @c RTSER_RTIOC_GET_STATUS, the
 * saved state will be cleared.
 */
#define RTSER_RTIOC_GET_STATUS	\
	_IOR(RTIOC_TYPE_SERIAL, 0x02, struct rtser_status)

/**
 * Get serial device's modem contol register
 *
 * @param[out] arg Pointer to variable receiving the content (int, see
 *             @ref RTSER_MCR_xxx)
 *
 * @return 0 on success, otherwise negative error code
 *
 * @coretags{task-unrestricted}
 */
#define RTSER_RTIOC_GET_CONTROL	\
	_IOR(RTIOC_TYPE_SERIAL, 0x03, int)

/**
 * Set serial device's modem contol register
 *
 * @param[in] arg New control register content (int, see @ref RTSER_MCR_xxx)
 *
 * @return 0 on success, otherwise negative error code
 *
 * @coretags{task-unrestricted}
 */
#define RTSER_RTIOC_SET_CONTROL	\
	_IOW(RTIOC_TYPE_SERIAL, 0x04, int)

/**
 * Wait on serial device events according to previously set mask
 *
 * @param[out] arg Pointer to event information buffer (struct rtser_event)
 *
 * @return 0 on success, otherwise:
 *
 * - -EBUSY is returned if another task is already waiting on events of this
 * device.
 *
 * - -EBADF is returned if the file descriptor is invalid or the device has
 * just been closed.
 *
 * @coretags{mode-unrestricted}
 */
#define RTSER_RTIOC_WAIT_EVENT	\
	_IOR(RTIOC_TYPE_SERIAL, 0x05, struct rtser_event)
/** @} */

/**
 * Set or clear break on UART output line
 *
 * @param[in] arg @c RTSER_BREAK_SET or @c RTSER_BREAK_CLR (int)
 *
 * @return 0 on success, otherwise negative error code
 *
 * @coretags{task-unrestricted}
 *
 * @note A set break condition may also be cleared on UART line
 * reconfiguration.
 */
#define RTSER_RTIOC_BREAK_CTL	\
	_IOR(RTIOC_TYPE_SERIAL, 0x06, int)
/** @} */

/*!
 * @anchor SERutils @name RT Serial example and utility programs
 * @{ */
/** @example cross-link.c */
/** @} */

#endif /* !_RTDM_UAPI_SERIAL_H */
