/***
 *
 *  RTnet - real-time networking subsystem
 *  Copyright (C) 2005-2011 Jan Kiszka <jan.kiszka@web.de>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *  As a special exception to the GNU General Public license, the RTnet
 *  project allows you to use this header file in unmodified form to produce
 *  application programs executing in user-space which use RTnet services by
 *  normal system calls. The resulting executable will not be covered by the
 *  GNU General Public License merely as a result of this header file use.
 *  Instead, this header file use will be considered normal use of RTnet and
 *  not a "derived work" in the sense of the GNU General Public License.
 *
 *  This exception does not apply when the application code is built as a
 *  static or dynamically loadable portion of the Linux kernel nor does the
 *  exception override other reasons justifying application of the GNU General
 *  Public License.
 *
 *  This exception applies only to the code released by the RTnet project
 *  under the name RTnet and bearing this exception notice. If you copy code
 *  from other sources into a copy of RTnet, the exception does not apply to
 *  the code that you add in this way.
 *
 */

#ifndef _RTDM_UAPI_NET_H
#define _RTDM_UAPI_NET_H

/* sub-classes: RTDM_CLASS_NETWORK */
#define RTDM_SUBCLASS_RTNET     0

#define RTIOC_TYPE_NETWORK      RTDM_CLASS_NETWORK

/* RTnet-specific IOCTLs */
#define RTNET_RTIOC_XMITPARAMS  _IOW(RTIOC_TYPE_NETWORK, 0x10, unsigned int)
#define RTNET_RTIOC_PRIORITY    RTNET_RTIOC_XMITPARAMS  /* legacy */
#define RTNET_RTIOC_TIMEOUT     _IOW(RTIOC_TYPE_NETWORK, 0x11, int64_t)
/* RTNET_RTIOC_CALLBACK         _IOW(RTIOC_TYPE_NETWORK, 0x12, ...
 * IOCTL only usable inside the kernel. */
/* RTNET_RTIOC_NONBLOCK         _IOW(RTIOC_TYPE_NETWORK, 0x13, unsigned int)
 * This IOCTL is no longer supported (and it was buggy anyway).
 * Use RTNET_RTIOC_TIMEOUT with any negative timeout value instead. */
#define RTNET_RTIOC_EXTPOOL     _IOW(RTIOC_TYPE_NETWORK, 0x14, unsigned int)
#define RTNET_RTIOC_SHRPOOL     _IOW(RTIOC_TYPE_NETWORK, 0x15, unsigned int)

/* socket transmission priorities */
#define SOCK_MAX_PRIO           0
#define SOCK_DEF_PRIO           SOCK_MAX_PRIO + \
				    (SOCK_MIN_PRIO-SOCK_MAX_PRIO+1)/2
#define SOCK_MIN_PRIO           SOCK_NRT_PRIO - 1
#define SOCK_NRT_PRIO           31

/* socket transmission channels */
#define SOCK_DEF_RT_CHANNEL     0           /* default rt xmit channel     */
#define SOCK_DEF_NRT_CHANNEL    1           /* default non-rt xmit channel */
#define SOCK_USER_CHANNEL       2           /* first user-defined channel  */

/* argument construction for RTNET_RTIOC_XMITPARAMS */
#define SOCK_XMIT_PARAMS(priority, channel) ((priority) | ((channel) << 16))

#endif  /* !_RTDM_UAPI_NET_H */
