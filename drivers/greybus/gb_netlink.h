/*
 * Greybus TCP/IP driver for Greybus over TCP/IP
 *
 * Released under the GPLv2 only.
 */

#ifndef __GB_NETLINK_H
#define __GB_NETLINK_H

/* Maximum packet size */
#define GB_NETLINK_MTU		2048
/* Maximum number of Cports */
#define GB_NETLINK_NUM_CPORT	32

#define GB_NL_NAME		"GREYBUS"
#define GB_NL_PID		1

enum {
	GB_NL_A_UNSPEC,
	GB_NL_A_DATA,
	GB_NL_A_CPORT,
	__GB_NL_A_MAX,
};
#define GB_NL_A_MAX (__GB_NL_A_MAX - 1)

enum {
	GB_NL_C_UNSPEC,
	GB_NL_C_MSG,
	GB_NL_C_HD_RESET,
	__GB_NL_C_MAX,
};
#define GB_NL_C_MAX (__GB_NL_C_MAX - 1)

#endif /* __GB_NETLINK_H */
