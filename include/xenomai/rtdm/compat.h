/*
 * Copyright (C) 2014 Philippe Gerum <rpm@xenomai.org>
 *
 * Xenomai is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Xenomai is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#ifndef _COBALT_RTDM_COMPAT_H
#define _COBALT_RTDM_COMPAT_H

#ifdef CONFIG_XENO_ARCH_SYS3264

#include <cobalt/kernel/compat.h>
#include <rtdm/rtdm.h>

struct compat_rtdm_getsockopt_args {
	int level;
	int optname;
	compat_uptr_t optval;
	compat_uptr_t optlen;
};

struct compat_rtdm_setsockopt_args {
	int level;
	int optname;
	const compat_uptr_t optval;
	socklen_t optlen;
};

struct compat_rtdm_getsockaddr_args {
	compat_uptr_t addr;
	compat_uptr_t addrlen;
};

struct compat_rtdm_setsockaddr_args {
	const compat_uptr_t addr;
	socklen_t addrlen;
};

#define _RTIOC_GETSOCKOPT_COMPAT	_IOW(RTIOC_TYPE_COMMON, 0x20,	\
					     struct compat_rtdm_getsockopt_args)
#define _RTIOC_SETSOCKOPT_COMPAT	_IOW(RTIOC_TYPE_COMMON, 0x21,	\
					     struct compat_rtdm_setsockopt_args)
#define _RTIOC_BIND_COMPAT		_IOW(RTIOC_TYPE_COMMON, 0x22,	\
					     struct compat_rtdm_setsockaddr_args)
#define _RTIOC_CONNECT_COMPAT		_IOW(RTIOC_TYPE_COMMON, 0x23,	\
					     struct compat_rtdm_setsockaddr_args)
#define _RTIOC_ACCEPT_COMPAT		_IOW(RTIOC_TYPE_COMMON, 0x25,	\
					     struct compat_rtdm_getsockaddr_args)
#define _RTIOC_GETSOCKNAME_COMPAT	_IOW(RTIOC_TYPE_COMMON, 0x26,	\
					     struct compat_rtdm_getsockaddr_args)
#define _RTIOC_GETPEERNAME_COMPAT	_IOW(RTIOC_TYPE_COMMON, 0x27,	\
					     struct compat_rtdm_getsockaddr_args)

#define __COMPAT_CASE(__op)		: case __op

#else	/* !CONFIG_XENO_ARCH_SYS3264 */

#define __COMPAT_CASE(__op)

#endif	/* !CONFIG_XENO_ARCH_SYS3264 */

#define COMPAT_CASE(__op)	case __op __COMPAT_CASE(__op  ## _COMPAT)

#endif /* !_COBALT_RTDM_COMPAT_H */
