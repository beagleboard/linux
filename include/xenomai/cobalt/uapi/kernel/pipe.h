/*
 * Copyright (C) 2014 Philippe Gerum <rpm@xenomai.org>.
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
#ifndef _COBALT_UAPI_KERNEL_PIPE_H
#define _COBALT_UAPI_KERNEL_PIPE_H

#define	XNPIPE_IOCTL_BASE	'p'

#define XNPIPEIOC_GET_NRDEV	_IOW(XNPIPE_IOCTL_BASE, 0, int)
#define XNPIPEIOC_IFLUSH	_IO(XNPIPE_IOCTL_BASE, 1)
#define XNPIPEIOC_OFLUSH	_IO(XNPIPE_IOCTL_BASE, 2)
#define XNPIPEIOC_FLUSH		XNPIPEIOC_OFLUSH
#define XNPIPEIOC_SETSIG	_IO(XNPIPE_IOCTL_BASE, 3)

#define XNPIPE_NORMAL	0x0
#define XNPIPE_URGENT	0x1

#define XNPIPE_IFLUSH	0x1
#define XNPIPE_OFLUSH	0x2

#define XNPIPE_MINOR_AUTO  (-1)

#endif /* !_COBALT_UAPI_KERNEL_PIPE_H */
