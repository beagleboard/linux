/*
 * Copyright (C) 2005 Jan Kiszka <jan.kiszka@web.de>.
 * Copyright (C) 2005 Joerg Langenberg <joerg.langenberg@gmx.net>.
 * Copyright (C) 2008 Gilles Chanteperdrix <gilles.chanteperdrix@xenomai.org>
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
#include <linux/err.h>
#include <linux/fs.h>
#include <cobalt/kernel/ppd.h>
#include <xenomai/rtdm/internal.h>
#include "process.h"
#include "internal.h"
#include "clock.h"
#include "io.h"

COBALT_SYSCALL(open, lostage,
	       (const char __user *u_path, int oflag))
{
	struct filename *filename;
	int ufd;

	filename = getname(u_path);
	if (IS_ERR(filename))
		return PTR_ERR(filename);

	ufd = __rtdm_dev_open(filename->name, oflag);
	putname(filename);

	return ufd;
}

COBALT_SYSCALL(socket, lostage,
	       (int protocol_family, int socket_type, int protocol))
{
	return __rtdm_dev_socket(protocol_family, socket_type, protocol);
}

COBALT_SYSCALL(close, lostage, (int fd))
{
	return rtdm_fd_close(fd, 0);
}

COBALT_SYSCALL(fcntl, current, (int fd, int cmd, int arg))
{
	return rtdm_fd_fcntl(fd, cmd, arg);
}

COBALT_SYSCALL(ioctl, handover,
	       (int fd, unsigned int request, void __user *arg))
{
	return rtdm_fd_ioctl(fd, request, arg);
}

COBALT_SYSCALL(read, handover,
	       (int fd, void __user *buf, size_t size))
{
	return rtdm_fd_read(fd, buf, size);
}

COBALT_SYSCALL(write, handover,
	       (int fd, const void __user *buf, size_t size))
{
	return rtdm_fd_write(fd, buf, size);
}

COBALT_SYSCALL(recvmsg, handover,
	       (int fd, struct user_msghdr __user *umsg, int flags))
{
	struct user_msghdr m;
	ssize_t ret;

	ret = cobalt_copy_from_user(&m, umsg, sizeof(m));
	if (ret)
		return ret;

	ret = rtdm_fd_recvmsg(fd, &m, flags);
	if (ret < 0)
		return ret;

	return cobalt_copy_to_user(umsg, &m, sizeof(*umsg)) ?: ret;
}

COBALT_SYSCALL(sendmsg, handover,
	       (int fd, struct user_msghdr __user *umsg, int flags))
{
	struct user_msghdr m;
	int ret;

	ret = cobalt_copy_from_user(&m, umsg, sizeof(m));

	return ret ?: rtdm_fd_sendmsg(fd, &m, flags);
}

COBALT_SYSCALL(mmap, lostage,
	       (int fd, struct _rtdm_mmap_request __user *u_rma,
	        void __user **u_addrp))
{
	struct _rtdm_mmap_request rma;
	void *u_addr = NULL;
	int ret;

	ret = cobalt_copy_from_user(&rma, u_rma, sizeof(rma));
	if (ret)
		return ret;

	ret = rtdm_fd_mmap(fd, &rma, &u_addr);
	if (ret)
		return ret;

	return cobalt_copy_to_user(u_addrp, &u_addr, sizeof(u_addr));
}

int __cobalt_first_fd_valid_p(fd_set *fds[XNSELECT_MAX_TYPES], int nfds)
{
	int i, fd;

	for (i = 0; i < XNSELECT_MAX_TYPES; i++)
		if (fds[i]
		    && (fd = find_first_bit(fds[i]->fds_bits, nfds)) < nfds)
			return rtdm_fd_valid_p(fd);

	/* All empty is correct, used as a "sleep" mechanism by strange
	   applications. */
	return 1;
}

static int select_bind_one(struct xnselector *selector, unsigned type, int fd)
{
	int rc;

	rc = rtdm_fd_select(fd, selector, type);
	if (rc != -ENOENT)
		return rc;

	return -EBADF;
}

int __cobalt_select_bind_all(struct xnselector *selector,
			     fd_set *fds[XNSELECT_MAX_TYPES], int nfds)
{
	unsigned fd, type;
	int err;

	for (type = 0; type < XNSELECT_MAX_TYPES; type++) {
		fd_set *set = fds[type];
		if (set)
			for (fd = find_first_bit(set->fds_bits, nfds);
			     fd < nfds;
			     fd = find_next_bit(set->fds_bits, nfds, fd + 1)) {
				err = select_bind_one(selector, type, fd);
				if (err)
					return err;
			}
	}

	return 0;
}

/* int select(int, fd_set *, fd_set *, fd_set *, struct timeval *) */
COBALT_SYSCALL(select, primary,
	       (int nfds,
		fd_set __user *u_rfds,
		fd_set __user *u_wfds,
		fd_set __user *u_xfds,
		struct timeval __user *u_tv))
{
	fd_set __user *ufd_sets[XNSELECT_MAX_TYPES] = {
		[XNSELECT_READ] = u_rfds,
		[XNSELECT_WRITE] = u_wfds,
		[XNSELECT_EXCEPT] = u_xfds
	};
	fd_set *in_fds[XNSELECT_MAX_TYPES] = {NULL, NULL, NULL};
	fd_set *out_fds[XNSELECT_MAX_TYPES] = {NULL, NULL, NULL};
	fd_set in_fds_storage[XNSELECT_MAX_TYPES],
		out_fds_storage[XNSELECT_MAX_TYPES];
	xnticks_t timeout = XN_INFINITE;
	struct restart_block *restart;
	xntmode_t mode = XN_RELATIVE;
	struct xnselector *selector;
	struct xnthread *curr;
	struct timeval tv;
	size_t fds_size;
	int i, err;

	curr = xnthread_current();

	if (u_tv) {
		if (xnthread_test_localinfo(curr, XNSYSRST)) {
			xnthread_clear_localinfo(curr, XNSYSRST);

			restart = cobalt_get_restart_block(current);
			timeout = restart->nanosleep.expires;

			if (restart->fn != cobalt_restart_syscall_placeholder) {
				err = -EINTR;
				goto out;
			}
		} else {
			if (!access_wok(u_tv, sizeof(tv))
			    || cobalt_copy_from_user(&tv, u_tv, sizeof(tv)))
				return -EFAULT;

			if (tv.tv_usec > 1000000)
				return -EINVAL;

			timeout = clock_get_ticks(CLOCK_MONOTONIC) + tv2ns(&tv);
		}

		mode = XN_ABSOLUTE;
	}

	fds_size = __FDELT__(nfds + __NFDBITS__ - 1) * sizeof(long);

	for (i = 0; i < XNSELECT_MAX_TYPES; i++)
		if (ufd_sets[i]) {
			in_fds[i] = &in_fds_storage[i];
			out_fds[i] = & out_fds_storage[i];
			if (!access_wok((void __user *) ufd_sets[i],
					sizeof(fd_set))
			    || cobalt_copy_from_user(in_fds[i],
						     (void __user *) ufd_sets[i],
						     fds_size))
				return -EFAULT;
		}

	selector = curr->selector;
	if (!selector) {
		/* This function may be called from pure Linux fd_sets, we want
		   to avoid the xnselector allocation in this case, so, we do a
		   simple test: test if the first file descriptor we find in the
		   fd_set is an RTDM descriptor or a message queue descriptor. */
		if (!__cobalt_first_fd_valid_p(in_fds, nfds))
			return -EBADF;

		selector = xnmalloc(sizeof(*curr->selector));
		if (selector == NULL)
			return -ENOMEM;
		xnselector_init(selector);
		curr->selector = selector;

		/* Bind directly the file descriptors, we do not need to go
		   through xnselect returning -ECHRNG */
		if ((err = __cobalt_select_bind_all(selector, in_fds, nfds)))
			return err;
	}

	do {
		err = xnselect(selector, out_fds, in_fds, nfds, timeout, mode);

		if (err == -ECHRNG) {
			int err = __cobalt_select_bind_all(selector, out_fds, nfds);
			if (err)
				return err;
		}
	} while (err == -ECHRNG);

	if (err == -EINTR && signal_pending(current)) {
		xnthread_set_localinfo(curr, XNSYSRST);

		restart = cobalt_get_restart_block(current);
		restart->fn = cobalt_restart_syscall_placeholder;
		restart->nanosleep.expires = timeout;

		return -ERESTARTSYS;
	}

out:
	if (u_tv && (err > 0 || err == -EINTR)) {
		xnsticks_t diff = timeout - clock_get_ticks(CLOCK_MONOTONIC);
		if (diff > 0)
			ticks2tv(&tv, diff);
		else
			tv.tv_sec = tv.tv_usec = 0;

		if (cobalt_copy_to_user(u_tv, &tv, sizeof(tv)))
			return -EFAULT;
	}

	if (err >= 0)
		for (i = 0; i < XNSELECT_MAX_TYPES; i++)
			if (ufd_sets[i]
			    && cobalt_copy_to_user((void __user *) ufd_sets[i],
						   out_fds[i], sizeof(fd_set)))
				return -EFAULT;
	return err;
}
