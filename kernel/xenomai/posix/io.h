/*
 * Copyright (C) 2005 Jan Kiszka <jan.kiszka@web.de>.
 * Copyright (C) 2005 Joerg Langenberg <joerg.langenberg@gmx.net>.
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
 * along with Xenomai; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#ifndef _COBALT_POSIX_IO_H
#define _COBALT_POSIX_IO_H

#include <rtdm/rtdm.h>
#include <xenomai/posix/syscall.h>
#include <cobalt/kernel/select.h>

int __cobalt_first_fd_valid_p(fd_set *fds[XNSELECT_MAX_TYPES], int nfds);

int __cobalt_select_bind_all(struct xnselector *selector,
			     fd_set *fds[XNSELECT_MAX_TYPES], int nfds);

COBALT_SYSCALL_DECL(open,
		    (const char __user *u_path, int oflag));

COBALT_SYSCALL_DECL(socket,
		    (int protocol_family,
		     int socket_type, int protocol));

COBALT_SYSCALL_DECL(close, (int fd));

COBALT_SYSCALL_DECL(fcntl, (int fd, int cmd, long arg));

COBALT_SYSCALL_DECL(ioctl,
		    (int fd, unsigned int request, void __user *arg));

COBALT_SYSCALL_DECL(read,
		    (int fd, void __user *buf, size_t size));

COBALT_SYSCALL_DECL(write,
		    (int fd, const void __user *buf, size_t size));

COBALT_SYSCALL_DECL(recvmsg,
		    (int fd, struct user_msghdr __user *umsg, int flags));

COBALT_SYSCALL_DECL(recvmmsg,
		    (int fd, struct mmsghdr __user *u_msgvec, unsigned int vlen,
		     unsigned int flags, struct timespec *u_timeout));

COBALT_SYSCALL_DECL(sendmsg,
		    (int fd, struct user_msghdr __user *umsg, int flags));

COBALT_SYSCALL_DECL(sendmmsg,
		    (int fd, struct mmsghdr __user *u_msgvec,
		     unsigned int vlen, unsigned int flags));

COBALT_SYSCALL_DECL(mmap,
		    (int fd, struct _rtdm_mmap_request __user *u_rma,
		     void __user * __user *u_addrp));

COBALT_SYSCALL_DECL(select,
		    (int nfds,
		     fd_set __user *u_rfds,
		     fd_set __user *u_wfds,
		     fd_set __user *u_xfds,
		     struct timeval __user *u_tv));

#endif /* !_COBALT_POSIX_IO_H */
