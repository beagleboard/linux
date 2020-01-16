/* SPDX-License-Identifier: ((GPL-2.0 WITH Linux-syscall-note) OR BSD-3-Clause) */
/*
 * Remote processor messaging sockets
 *
 * Copyright (C) 2011-2020 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Ohad Ben-Cohen <ohad@wizery.com>
 * Suman Anna <s-anna@ti.com>
 */

#ifndef _UAPI_RPMSG_SOCKET_H
#define _UAPI_RPMSG_SOCKET_H

#include <linux/types.h>
#include <linux/socket.h>

/* user space needs this */
#ifndef AF_RPMSG
#define AF_RPMSG	45
#define PF_RPMSG	AF_RPMSG
#endif

struct sockaddr_rpmsg {
	__kernel_sa_family_t family;
	__u32 vproc_id;
	__u32 addr;
};

#define RPMSG_LOCALHOST ((__u32)~0UL)

#endif /* _UAPI_RPMSG_SOCKET_H */
