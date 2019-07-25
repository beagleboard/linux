/**
 * @file
 * This file is part of the Xenomai project.
 *
 * @note Copyright (C) 2009 Philippe Gerum <rpm@xenomai.org>
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
 */

#ifndef _RTDM_UAPI_IPC_H
#define _RTDM_UAPI_IPC_H

/**
 * @ingroup rtdm_profiles
 * @defgroup rtdm_ipc Real-time IPC
 *
 * @b Profile @b Revision: 1
 * @n
 * @n
 * @par Device Characteristics
 * @n
 * @ref rtdm_driver_flags "Device Flags": @c RTDM_PROTOCOL_DEVICE @n
 * @n
 * @ref rtdm_driver.protocol_family "Protocol Family": @c PF_RTIPC @n
 * @n
 * @ref rtdm_driver.socket_type "Socket Type": @c SOCK_DGRAM @n
 * @n
 * @ref rtdm_driver_profile "Device Class": @c RTDM_CLASS_RTIPC @n
 * @n
 * @{
 *
 * @anchor rtipc_operations @name Supported operations
 * Standard socket operations supported by the RTIPC protocols.
 * @{
 */

/** Create an endpoint for communication in the AF_RTIPC domain.
 *
 * @param[in] domain The communication domain. Must be AF_RTIPC.
 *
 * @param[in] type The socket type. Must be SOCK_DGRAM.
 *
 * @param [in] protocol Any of @ref IPCPROTO_XDDP, @ref IPCPROTO_IDDP,
 * or @ref IPCPROTO_BUFP. @ref IPCPROTO_IPC is also valid, and refers
 * to the default RTIPC protocol, namely @ref IPCPROTO_IDDP.
 *
 * @return In addition to the standard error codes for @c socket(2),
 * the following specific error code may be returned:
 * - -ENOPROTOOPT (Protocol is known, but not compiled in the RTIPC driver).
 *   See @ref RTIPC_PROTO "RTIPC protocols"
 *   for available protocols.
 *
 * @par Calling context:
 * non-RT
 */
#ifdef DOXYGEN_CPP
int socket__AF_RTIPC(int domain =AF_RTIPC, int type =SOCK_DGRAM, int protocol);
#endif

/**
 * Close a RTIPC socket descriptor.
 *
 * Blocking calls to any of the @ref sendmsg__AF_RTIPC "sendmsg" or @ref
 * recvmsg__AF_RTIPC "recvmsg" functions will be unblocked when the socket
 * is closed and return with an error.
 *
 * @param[in] sockfd The socket descriptor to close.
 *
 * @return In addition to the standard error codes for @c close(2),
 * the following specific error code may be returned:
 * none
 *
 * @par Calling context:
 * non-RT
 */
#ifdef DOXYGEN_CPP
int close__AF_RTIPC(int sockfd);
#endif

/**
 * Bind a RTIPC socket to a port.
 *
 * Bind the socket to a destination port.
 *
 * @param[in] sockfd The RTDM file descriptor obtained from the socket
 * creation call.
 *
 * @param [in] addr The address to bind the socket to (see struct
 * sockaddr_ipc). The meaning of such address depends on the RTIPC
 * protocol in use for the socket:
 *
 * - IPCPROTO_XDDP
 *
 *   This action creates an endpoint for channelling traffic between
 *   the Xenomai and Linux domains.
 *
 *   @em sipc_family must be AF_RTIPC, @em sipc_port is either -1,
 *   or a valid free port number between 0 and
 *   CONFIG_XENO_OPT_PIPE_NRDEV-1.
 *
 *   If @em sipc_port is -1, a free port will be assigned automatically.
 *
 *   Upon success, the pseudo-device /dev/rtp@em N will be reserved
 *   for this communication channel, where @em N is the assigned port
 *   number. The non real-time side shall open this device to exchange
 *   data over the bound socket.
 *
 * @anchor xddp_label_binding
 *   If a label was assigned (see @ref XDDP_LABEL) prior to
 *   binding the socket to a port, a registry link referring to the
 *   created pseudo-device will be automatically set up as
 *   @c /proc/xenomai/registry/rtipc/xddp/@em label, where @em label is the
 *   label string passed to setsockopt() for the @ref XDDP_LABEL option.
 *
 * - IPCPROTO_IDDP
 *
 *   This action creates an endpoint for exchanging datagrams within
 *   the Xenomai domain.
 *
 *   @em sipc_family must be AF_RTIPC, @em sipc_port is either -1,
 *   or a valid free port number between 0 and
 *   CONFIG_XENO_OPT_IDDP_NRPORT-1.
 *
 *   If @em sipc_port is -1, a free port will be assigned
 *   automatically. The real-time peer shall connect to the same port
 *   for exchanging data over the bound socket.
 *
 * @anchor iddp_label_binding
 *   If a label was assigned (see @ref IDDP_LABEL) prior to binding
 *   the socket to a port, a registry link referring to the assigned
 *   port number will be automatically set up as @c
 *   /proc/xenomai/registry/rtipc/iddp/@em label, where @em label is
 *   the label string passed to setsockopt() for the @ref IDDP_LABEL
 *   option.
 *
 * - IPCPROTO_BUFP
 *
 *   This action creates an endpoint for a one-way byte
 *   stream within the Xenomai domain.
 *
 *   @em sipc_family must be AF_RTIPC, @em sipc_port is either -1,
 *   or a valid free port number between 0 and CONFIG_XENO_OPT_BUFP_NRPORT-1.
 *
 *   If @em sipc_port is -1, an available port will be assigned
 *   automatically. The real-time peer shall connect to the same port
 *   for exchanging data over the bound socket.
 *
 * @anchor bufp_label_binding
 *   If a label was assigned (see @ref BUFP_LABEL) prior to binding
 *   the socket to a port, a registry link referring to the assigned
 *   port number will be automatically set up as @c
 *   /proc/xenomai/registry/rtipc/bufp/@em label, where @em label is
 *   the label string passed to setsockopt() for the @a BUFP_LABEL
 *   option.
 *
 * @param[in] addrlen The size in bytes of the structure pointed to by
 * @a addr.
 *
 * @return In addition to the standard error codes for @c
 * bind(2), the following specific error code may be returned:
 *   - -EFAULT (Invalid data address given)
 *   - -ENOMEM (Not enough memory)
 *   - -EINVAL (Invalid parameter)
 *   - -EADDRINUSE (Socket already bound to a port, or no port available)
 *   - -EAGAIN (no registry slot available, check/raise
 *     CONFIG_XENO_OPT_REGISTRY_NRSLOTS) .
 *
 * @par Calling context:
 * non-RT
 */
#ifdef DOXYGEN_CPP
int bind__AF_RTIPC(int sockfd, const struct sockaddr_ipc *addr,
		   socklen_t addrlen);
#endif

/**
 * Initiate a connection on a RTIPC socket.
 *
 * @param[in] sockfd The RTDM file descriptor obtained from the socket
 * creation call.
 *
 * @param [in] addr The address to connect the socket to (see struct
 * sockaddr_ipc).
 *
 * - If sipc_port is a valid port for the protocol, it is used
 * verbatim and the connection succeeds immediately, regardless of
 * whether the destination is bound at the time of the call.
 *
 * - If sipc_port is -1 and a label was assigned to the socket,
 * connect() blocks for the requested amount of time (see @ref
 * SO_RCVTIMEO) until a socket is bound to the same label via @c
 * bind(2) (see @ref XDDP_LABEL, @ref IDDP_LABEL, @ref BUFP_LABEL), in
 * which case a connection is established between both endpoints.
 *
 * - If sipc_port is -1 and no label was assigned to the socket, the
 * default destination address is cleared, meaning that any subsequent
 * write to the socket will return -EDESTADDRREQ, until a valid
 * destination address is set via @c connect(2) or @c bind(2).
 *
 * @param[in] addrlen The size in bytes of the structure pointed to by
 * @a addr.
 *
 * @return In addition to the standard error codes for @c connect(2),
 * the following specific error code may be returned:
 * none.
 *
 * @par Calling context:
 * RT/non-RT
 */
#ifdef DOXYGEN_CPP
int connect__AF_RTIPC(int sockfd, const struct sockaddr_ipc *addr,
		      socklen_t addrlen);
#endif

/**
 * Set options on RTIPC sockets.
 *
 * These functions allow to set various socket options.
 * Supported Levels and Options:
 *
 * - Level @ref sockopts_socket "SOL_SOCKET"
 * - Level @ref sockopts_xddp "SOL_XDDP"
 * - Level @ref sockopts_iddp "SOL_IDDP"
 * - Level @ref sockopts_bufp "SOL_BUFP"
 * .
 *
 * @return In addition to the standard error codes for @c
 * setsockopt(2), the following specific error code may
 * be returned:
 * follow the option links above.
 *
 * @par Calling context:
 * non-RT
 */
#ifdef DOXYGEN_CPP
int setsockopt__AF_RTIPC(int sockfd, int level, int optname,
			 const void *optval, socklen_t optlen);
#endif
/**
 * Get options on RTIPC sockets.
 *
 * These functions allow to get various socket options.
 * Supported Levels and Options:
 *
 * - Level @ref sockopts_socket "SOL_SOCKET"
 * - Level @ref sockopts_xddp "SOL_XDDP"
 * - Level @ref sockopts_iddp "SOL_IDDP"
 * - Level @ref sockopts_bufp "SOL_BUFP"
 * .
 *
 * @return In addition to the standard error codes for @c
 * getsockopt(2), the following specific error code may
 * be returned:
 * follow the option links above.
 *
 * @par Calling context:
 * RT/non-RT
 */
#ifdef DOXYGEN_CPP
int getsockopt__AF_RTIPC(int sockfd, int level, int optname,
			 void *optval, socklen_t *optlen);
#endif

/**
 * Send a message on a RTIPC socket.
 *
 * @param[in] sockfd The RTDM file descriptor obtained from the socket
 * creation call.
 *
 * @param[in] msg The address of the message header conveying the
 * datagram.
 *
 * @param [in] flags Operation flags:
 *
 * - MSG_OOB Send out-of-band message.  For all RTIPC protocols except
 *   @ref IPCPROTO_BUFP, sending out-of-band data actually means
 *   pushing them to the head of the receiving queue, so that the
 *   reader will always receive them before normal messages. @ref
 *   IPCPROTO_BUFP does not support out-of-band sending.
 *
 * - MSG_DONTWAIT Non-blocking I/O operation. The caller will not be
 *   blocked whenever the message cannot be sent immediately at the
 *   time of the call (e.g. memory shortage), but will rather return
 *   with -EWOULDBLOCK. Unlike other RTIPC protocols, @ref
 *   IPCPROTO_XDDP accepts but never considers MSG_DONTWAIT since
 *   writing to a real-time XDDP endpoint is inherently a non-blocking
 *   operation.
 *
 * - MSG_MORE Accumulate data before sending. This flag is accepted by
 *   the @ref IPCPROTO_XDDP protocol only, and tells the send service
 *   to accumulate the outgoing data into an internal streaming
 *   buffer, instead of issuing a datagram immediately for it. See
 *   @ref XDDP_BUFSZ for more.
 *
 * @note No RTIPC protocol allows for short writes, and only complete
 * messages are sent to the peer.
 *
 * @return In addition to the standard error codes for @c sendmsg(2),
 * the following specific error code may be returned:
 * none.
 *
 * @par Calling context:
 * RT
 */
#ifdef DOXYGEN_CPP
ssize_t sendmsg__AF_RTIPC(int sockfd, const struct msghdr *msg, int flags);
#endif

/**
 * Receive a message from a RTIPC socket.
 *
 * @param[in] sockfd The RTDM file descriptor obtained from the socket
 * creation call.
 *
 * @param[out] msg The address the message header will be copied at.
 *
 * @param [in] flags Operation flags:
 *
 * - MSG_DONTWAIT Non-blocking I/O operation. The caller will not be
 *   blocked whenever no message is immediately available for receipt
 *   at the time of the call, but will rather return with
 *   -EWOULDBLOCK.
 *
 * @note @ref IPCPROTO_BUFP does not allow for short reads and always
 * returns the requested amount of bytes, except in one situation:
 * whenever some writer is waiting for sending data upon a buffer full
 * condition, while the caller would have to wait for receiving a
 * complete message.  This is usually the sign of a pathological use
 * of the BUFP socket, like defining an incorrect buffer size via @ref
 * BUFP_BUFSZ. In that case, a short read is allowed to prevent a
 * deadlock.
 *
 * @return In addition to the standard error codes for @c recvmsg(2),
 * the following specific error code may be returned:
 * none.
 *
 * @par Calling context:
 * RT
 */
#ifdef DOXYGEN_CPP
ssize_t recvmsg__AF_RTIPC(int sockfd, struct msghdr *msg, int flags);
#endif

/**
 * Get socket name.
 *
 * The name of the local endpoint for the socket is copied back (see
 * struct sockaddr_ipc).
 *
 * @return In addition to the standard error codes for @c getsockname(2),
 * the following specific error code may be returned:
 * none.
 *
 * @par Calling context:
 * RT/non-RT
 */
#ifdef DOXYGEN_CPP
int getsockname__AF_RTIPC(int sockfd, struct sockaddr_ipc *addr, socklen_t *addrlen);
#endif

/**
 * Get socket peer.
 *
 * The name of the remote endpoint for the socket is copied back (see
 * struct sockaddr_ipc). This is the default destination address for
 * messages sent on the socket. It can be set either explicitly via @c
 * connect(2), or implicitly via @c bind(2) if no @c connect(2) was
 * called prior to binding the socket to a port, in which case both
 * the local and remote names are equal.
 *
 * @return In addition to the standard error codes for @c getpeername(2),
 * the following specific error code may be returned:
 * none.
 *
 * @par Calling context:
 * RT/non-RT
 */
#ifdef DOXYGEN_CPP
int getpeername__AF_RTIPC(int sockfd, struct sockaddr_ipc *addr, socklen_t *addrlen);
#endif

/** @} */

#include <cobalt/uapi/kernel/types.h>
#include <cobalt/uapi/kernel/pipe.h>
#include <rtdm/rtdm.h>

/* Address family */
#define AF_RTIPC		111

/* Protocol family */
#define PF_RTIPC		AF_RTIPC

/**
 * @anchor RTIPC_PROTO @name RTIPC protocol list
 * protocols for the PF_RTIPC protocol family
 *
 * @{ */
enum {
/** Default protocol (IDDP) */
	IPCPROTO_IPC  = 0,
/**
 * Cross-domain datagram protocol (RT <-> non-RT).
 *
 * Real-time Xenomai threads and regular Linux threads may want to
 * exchange data in a way that does not require the former to leave
 * the real-time domain (i.e. primary mode). The RTDM-based XDDP
 * protocol is available for this purpose.
 *
 * On the Linux domain side, pseudo-device files named /dev/rtp@em \<minor\>
 * give regular POSIX threads access to non real-time communication
 * endpoints, via the standard character-based I/O interface. On the
 * Xenomai domain side, sockets may be bound to XDDP ports, which act
 * as proxies to send and receive data to/from the associated
 * pseudo-device files. Ports and pseudo-device minor numbers are
 * paired, meaning that e.g. socket port 7 will proxy the traffic to/from
 * /dev/rtp7.
 *
 * All data sent through a bound/connected XDDP socket via @c
 * sendto(2) or @c write(2) will be passed to the peer endpoint in the
 * Linux domain, and made available for reading via the standard @c
 * read(2) system call. Conversely, all data sent using @c write(2)
 * through the non real-time endpoint will be conveyed to the
 * real-time socket endpoint, and made available to the @c recvfrom(2)
 * or @c read(2) system calls.
 */
	IPCPROTO_XDDP = 1,
/**
 * Intra-domain datagram protocol (RT <-> RT).
 *
 * The RTDM-based IDDP protocol enables real-time threads to exchange
 * datagrams within the Xenomai domain, via socket endpoints.
 */
	IPCPROTO_IDDP = 2,
/**
 * Buffer protocol (RT <-> RT, byte-oriented).
 *
 * The RTDM-based BUFP protocol implements a lightweight,
 * byte-oriented, one-way Producer-Consumer data path. All messages
 * written are buffered into a single memory area in strict FIFO
 * order, until read by the consumer.
 *
 * This protocol always prevents short writes, and only allows short
 * reads when a potential deadlock situation arises (i.e. readers and
 * writers waiting for each other indefinitely).
 */
	IPCPROTO_BUFP = 3,
	IPCPROTO_MAX
};
/** @} */

/**
 * Port number type for the RTIPC address family.
 */
typedef int16_t rtipc_port_t;

/**
 * Port label information structure.
 */
struct rtipc_port_label {
	/** Port label string, null-terminated. */
	char label[XNOBJECT_NAME_LEN];
};

/**
 * Socket address structure for the RTIPC address family.
 */
struct sockaddr_ipc {
	/** RTIPC address family, must be @c AF_RTIPC */
	sa_family_t sipc_family;
	/** Port number. */
	rtipc_port_t sipc_port;
};

#define SOL_XDDP		311
/**
 * @anchor sockopts_xddp @name XDDP socket options
 * Setting and getting XDDP socket options.
 * @{ */
/**
 * XDDP label assignment
 *
 * ASCII label strings can be attached to XDDP ports, so that opening
 * the non-RT endpoint can be done by specifying this symbolic device
 * name rather than referring to a raw pseudo-device entry
 * (i.e. /dev/rtp@em N).
 *
 * When available, this label will be registered when binding, in
 * addition to the port number (see @ref xddp_label_binding
 * "XDDP port binding").
 *
 * It is not allowed to assign a label after the socket was
 * bound. However, multiple assignment calls are allowed prior to the
 * binding; the last label set will be used.
 *
 * @param [in] level @ref sockopts_xddp "SOL_XDDP"
 * @param [in] optname @b XDDP_LABEL
 * @param [in] optval Pointer to struct rtipc_port_label
 * @param [in] optlen sizeof(struct rtipc_port_label)
 *
 * @return 0 is returned upon success. Otherwise:
 *
 * - -EFAULT (Invalid data address given)
 * - -EALREADY (socket already bound)
 * - -EINVAL (@a optlen invalid)
 * .
 *
 * @par Calling context:
 * RT/non-RT
 */
#define XDDP_LABEL		1
/**
 * XDDP local pool size configuration
 *
 * By default, the memory needed to convey the data is pulled from
 * Xenomai's system pool. Setting a local pool size overrides this
 * default for the socket.
 *
 * If a non-zero size was configured, a local pool is allocated at
 * binding time. This pool will provide storage for pending datagrams.
 *
 * It is not allowed to configure a local pool size after the socket
 * was bound. However, multiple configuration calls are allowed prior
 * to the binding; the last value set will be used.
 *
 * @note: the pool memory is obtained from the host allocator by the
 * @ref bind__AF_RTIPC "bind call".
 *
 * @param [in] level @ref sockopts_xddp "SOL_XDDP"
 * @param [in] optname @b XDDP_POOLSZ
 * @param [in] optval Pointer to a variable of type size_t, containing
 * the required size of the local pool to reserve at binding time
 * @param [in] optlen sizeof(size_t)
 *
 * @return 0 is returned upon success. Otherwise:
 *
 * - -EFAULT (Invalid data address given)
 * - -EALREADY (socket already bound)
 * - -EINVAL (@a optlen invalid or *@a optval is zero)
 * .
 *
 * @par Calling context:
 * RT/non-RT
 */
#define XDDP_POOLSZ		2
/**
 * XDDP streaming buffer size configuration
 *
 * In addition to sending datagrams, real-time threads may stream data
 * in a byte-oriented mode through the port as well. This increases
 * the bandwidth and reduces the overhead, when the overall data to
 * send to the Linux domain is collected by bits, and keeping the
 * message boundaries is not required.
 *
 * This feature is enabled when a non-zero buffer size is set for the
 * socket. In that case, the real-time data accumulates into the
 * streaming buffer when MSG_MORE is passed to any of the @ref
 * sendmsg__AF_RTIPC "send functions", until:
 *
 * - the receiver from the Linux domain wakes up and consumes it,
 * - a different source port attempts to send data to the same
 *   destination port,
 * - MSG_MORE is absent from the send flags,
 * - the buffer is full,
 * .
 * whichever comes first.
 *
 * Setting *@a optval to zero disables the streaming buffer, in which
 * case all sendings are conveyed in separate datagrams, regardless of
 * MSG_MORE.
 *
 * @note only a single streaming buffer exists per socket. When this
 * buffer is full, the real-time data stops accumulating and sending
 * operations resume in mere datagram mode. Accumulation may happen
 * again after some or all data in the streaming buffer is consumed
 * from the Linux domain endpoint.
 *
 * The streaming buffer size may be adjusted multiple times during the
 * socket lifetime; the latest configuration change will take effect
 * when the accumulation resumes after the previous buffer was
 * flushed.
 *
 * @param [in] level @ref sockopts_xddp "SOL_XDDP"
 * @param [in] optname @b XDDP_BUFSZ
 * @param [in] optval Pointer to a variable of type size_t, containing
 * the required size of the streaming buffer
 * @param [in] optlen sizeof(size_t)
 *
 * @return 0 is returned upon success. Otherwise:
 *
 * - -EFAULT (Invalid data address given)
 * - -ENOMEM (Not enough memory)
 * - -EINVAL (@a optlen is invalid)
 * .
 *
 * @par Calling context:
 * RT/non-RT
 */
#define XDDP_BUFSZ		3
/**
 * XDDP monitoring callback
 *
 * Other RTDM drivers may install a user-defined callback via the @ref
 * rtdm_setsockopt call from the inter-driver API, in order to collect
 * particular events occurring on the channel.
 *
 * This notification mechanism is particularly useful to monitor a
 * channel asynchronously while performing other tasks.
 *
 * The user-provided routine will be passed the RTDM file descriptor
 * of the socket receiving the event, the event code, and an optional
 * argument.  Four events are currently defined, see @ref XDDP_EVENTS.
 *
 * The XDDP_EVTIN and XDDP_EVTOUT events are fired on behalf of a
 * fully atomic context; therefore, care must be taken to keep their
 * overhead low. In those cases, the Xenomai services that may be
 * called from the callback are restricted to the set allowed to a
 * real-time interrupt handler.
 *
 * @param [in] level @ref sockopts_xddp "SOL_XDDP"
 * @param [in] optname @b XDDP_MONITOR
 * @param [in] optval Pointer to a pointer to function of type int
 *             (*)(int fd, int event, long arg), containing the address of the
 *             user-defined callback.Passing a NULL callback pointer
 *             in @a optval disables monitoring.
 * @param [in] optlen sizeof(int (*)(int fd, int event, long arg))
 *
 * @return 0 is returned upon success. Otherwise:
 *
 * - -EFAULT (Invalid data address given)
 * - -EPERM (Operation not allowed from user-space)
 * - -EINVAL (@a optlen is invalid)
 * .
 *
 * @par Calling context:
 * RT/non-RT, kernel space only
 */
#define XDDP_MONITOR		4
/** @} */

/**
 * @anchor XDDP_EVENTS @name XDDP events
 * Specific events occurring on XDDP channels, which can be monitored
 * via the @ref XDDP_MONITOR socket option.
 *
 * @{ */
/**
 * @ref XDDP_MONITOR "Monitor" writes to the non real-time endpoint.
 *
 * XDDP_EVTIN is sent when data is written to the non real-time
 * endpoint the socket is bound to (i.e. via /dev/rtp@em N), which
 * means that some input is pending for the real-time endpoint. The
 * argument is the size of the incoming message.
 */
#define XDDP_EVTIN		1
/**
 * @ref XDDP_MONITOR "Monitor" reads from the non real-time endpoint.
 *
 * XDDP_EVTOUT is sent when the non real-time endpoint successfully
 * reads a complete message (i.e. via /dev/rtp@em N). The argument is
 * the size of the outgoing message.
 */
#define XDDP_EVTOUT		2
/**
 * @ref XDDP_MONITOR "Monitor" close from the non real-time endpoint.
 *
 * XDDP_EVTDOWN is sent when the non real-time endpoint is closed. The
 * argument is always 0.
 */
#define XDDP_EVTDOWN		3
/**
 * @ref XDDP_MONITOR "Monitor" memory shortage for non real-time
 * datagrams.
 *
 * XDDP_EVTNOBUF is sent when no memory is available from the pool to
 * hold the message currently sent from the non real-time
 * endpoint. The argument is the size of the failed allocation. Upon
 * return from the callback, the caller will block and retry until
 * enough space is available from the pool; during that process, the
 * callback might be invoked multiple times, each time a new attempt
 * to get the required memory fails.
 */
#define XDDP_EVTNOBUF		4
/** @} */

#define SOL_IDDP		312
/**
 * @anchor sockopts_iddp @name IDDP socket options
 * Setting and getting IDDP socket options.
 * @{ */
/**
 * IDDP label assignment
 *
 * ASCII label strings can be attached to IDDP ports, in order to
 * connect sockets to them in a more descriptive way than using plain
 * numeric port values.
 *
 * When available, this label will be registered when binding, in
 * addition to the port number (see @ref iddp_label_binding
 * "IDDP port binding").
 *
 * It is not allowed to assign a label after the socket was
 * bound. However, multiple assignment calls are allowed prior to the
 * binding; the last label set will be used.
 *
 * @param [in] level @ref sockopts_iddp "SOL_IDDP"
 * @param [in] optname @b IDDP_LABEL
 * @param [in] optval Pointer to struct rtipc_port_label
 * @param [in] optlen sizeof(struct rtipc_port_label)
 *
 * @return 0 is returned upon success. Otherwise:
 *
 * - -EFAULT (Invalid data address given)
 * - -EALREADY (socket already bound)
 * - -EINVAL (@a optlen is invalid)
 * .
 *
 * @par Calling context:
 * RT/non-RT
 */
#define IDDP_LABEL		1
/**
 * IDDP local pool size configuration
 *
 * By default, the memory needed to convey the data is pulled from
 * Xenomai's system pool. Setting a local pool size overrides this
 * default for the socket.
 *
 * If a non-zero size was configured, a local pool is allocated at
 * binding time. This pool will provide storage for pending datagrams.
 *
 * It is not allowed to configure a local pool size after the socket
 * was bound. However, multiple configuration calls are allowed prior
 * to the binding; the last value set will be used.
 *
 * @note: the pool memory is obtained from the host allocator by the
 * @ref bind__AF_RTIPC "bind call".
 *
 * @param [in] level @ref sockopts_iddp "SOL_IDDP"
 * @param [in] optname @b IDDP_POOLSZ
 * @param [in] optval Pointer to a variable of type size_t, containing
 * the required size of the local pool to reserve at binding time
 * @param [in] optlen sizeof(size_t)
 *
 * @return 0 is returned upon success. Otherwise:
 *
 * - -EFAULT (Invalid data address given)
 * - -EALREADY (socket already bound)
 * - -EINVAL (@a optlen is invalid or *@a optval is zero)
 * .
 *
 * @par Calling context:
 * RT/non-RT
 */
#define IDDP_POOLSZ		2
/** @} */

#define SOL_BUFP		313
/**
 * @anchor sockopts_bufp @name BUFP socket options
 * Setting and getting BUFP socket options.
 * @{ */
/**
 * BUFP label assignment
 *
 * ASCII label strings can be attached to BUFP ports, in order to
 * connect sockets to them in a more descriptive way than using plain
 * numeric port values.
 *
 * When available, this label will be registered when binding, in
 * addition to the port number (see @ref bufp_label_binding
 * "BUFP port binding").
 *
 * It is not allowed to assign a label after the socket was
 * bound. However, multiple assignment calls are allowed prior to the
 * binding; the last label set will be used.
 *
 * @param [in] level @ref sockopts_bufp "SOL_BUFP"
 * @param [in] optname @b BUFP_LABEL
 * @param [in] optval Pointer to struct rtipc_port_label
 * @param [in] optlen sizeof(struct rtipc_port_label)
 *
 * @return 0 is returned upon success. Otherwise:
 *
 * - -EFAULT (Invalid data address given)
 * - -EALREADY (socket already bound)
 * - -EINVAL (@a optlen is invalid)
 * .
 *
 * @par Calling context:
 * RT/non-RT
 */
#define BUFP_LABEL		1
/**
 * BUFP buffer size configuration
 *
 * All messages written to a BUFP socket are buffered in a single
 * per-socket memory area. Configuring the size of such buffer prior
 * to binding the socket to a destination port is mandatory.
 *
 * It is not allowed to configure a buffer size after the socket was
 * bound. However, multiple configuration calls are allowed prior to
 * the binding; the last value set will be used.
 *
 * @note: the buffer memory is obtained from the host allocator by the
 * @ref bind__AF_RTIPC "bind call".
 *
 * @param [in] level @ref sockopts_bufp "SOL_BUFP"
 * @param [in] optname @b BUFP_BUFSZ
 * @param [in] optval Pointer to a variable of type size_t, containing
 * the required size of the buffer to reserve at binding time
 * @param [in] optlen sizeof(size_t)
 *
 * @return 0 is returned upon success. Otherwise:
 *
 * - -EFAULT (Invalid data address given)
 * - -EALREADY (socket already bound)
 * - -EINVAL (@a optlen is invalid or *@a optval is zero)
 * .
 *
 * @par Calling context:
 * RT/non-RT
 */
#define BUFP_BUFSZ		2
/** @} */

/**
 * @anchor sockopts_socket @name Socket level options
 * Setting and getting supported standard socket level options.
 * @{ */
/**
 *
 * @ref IPCPROTO_IDDP and @ref IPCPROTO_BUFP protocols support the
 * standard SO_SNDTIMEO socket option, from the @c SOL_SOCKET level.
 *
 * @see @c setsockopt(), @c getsockopt() in IEEE Std 1003.1,
 * http://www.opengroup.org/onlinepubs/009695399/
 */
#ifdef DOXYGEN_CPP
#define SO_SNDTIMEO defined_by_kernel_header_file
#endif
/**
 *
 * All RTIPC protocols support the standard SO_RCVTIMEO socket option,
 * from the @c SOL_SOCKET level.
 *
 * @see @c setsockopt(), @c getsockopt() in IEEE Std 1003.1,
 * http://www.opengroup.org/onlinepubs/009695399/
 */
#ifdef DOXYGEN_CPP
#define SO_RCVTIMEO defined_by_kernel_header_file
#endif
/** @} */

/**
 * @anchor rtdm_ipc_examples @name RTIPC examples
 * @{ */
/** @example bufp-readwrite.c */
/** @example bufp-label.c */
/** @example iddp-label.c */
/** @example iddp-sendrecv.c */
/** @example xddp-echo.c */
/** @example xddp-label.c */
/** @example xddp-stream.c */
/** @} */

/** @} */

#endif /* !_RTDM_UAPI_IPC_H */
