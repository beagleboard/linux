/*
 * Remote processor messaging sockets
 *
 * Copyright (C) 2011-2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Ohad Ben-Cohen <ohad@wizery.com>
 * Suman Anna <s-anna@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _UAPI_RPMSG_SOCKET_H
#define _UAPI_RPMSG_SOCKET_H

#include <linux/types.h>
#include <linux/socket.h>

/* user space needs this */
#ifndef AF_RPMSG
#define AF_RPMSG	41
#define PF_RPMSG	AF_RPMSG
#endif

struct sockaddr_rpmsg {
	__kernel_sa_family_t family;
	__u32 vproc_id;
	__u32 addr;
};

#define RPMSG_LOCALHOST ((__u32)~0UL)

#endif /* _UAPI_RPMSG_SOCKET_H */
