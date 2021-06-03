/***
 *
 *  stack/iovec.c
 *
 *  RTnet - real-time networking subsystem
 *  Copyright (C) 1999,2000 Zentropic Computing, LLC
 *                2002 Ulrich Marx <marx@kammer.uni-hannover.de>
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
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <rtdm/driver.h>
#include <rtnet_iovec.h>
#include <rtnet_socket.h>

ssize_t rtnet_write_to_iov(struct rtdm_fd *fd, struct iovec *iov, int iovlen,
			   const void *data, size_t len)
{
	ssize_t ret = 0;
	size_t nbytes;
	int n;

	for (n = 0; len > 0 && n < iovlen; n++, iov++) {
		if (iov->iov_len == 0)
			continue;

		nbytes = iov->iov_len;
		if (nbytes > len)
			nbytes = len;

		ret = rtnet_put_arg(fd, iov->iov_base, data, nbytes);
		if (ret)
			break;

		len -= nbytes;
		data += nbytes;
		iov->iov_len -= nbytes;
		iov->iov_base += nbytes;
		ret += nbytes;
		if (ret < 0) {
			ret = -EINVAL;
			break;
		}
	}

	return ret;
}
EXPORT_SYMBOL_GPL(rtnet_write_to_iov);

ssize_t rtnet_read_from_iov(struct rtdm_fd *fd, struct iovec *iov, int iovlen,
			    void *data, size_t len)
{
	ssize_t ret = 0;
	size_t nbytes;
	int n;

	for (n = 0; len > 0 && n < iovlen; n++, iov++) {
		if (iov->iov_len == 0)
			continue;

		nbytes = iov->iov_len;
		if (nbytes > len)
			nbytes = len;

		if (!rtdm_fd_is_user(fd))
			memcpy(data, iov->iov_base, nbytes);
		else {
			ret = rtdm_copy_from_user(fd, data, iov->iov_base,
						  nbytes);
			if (ret)
				break;
		}

		len -= nbytes;
		data += nbytes;
		iov->iov_len -= nbytes;
		iov->iov_base += nbytes;
		ret += nbytes;
		if (ret < 0) {
			ret = -EINVAL;
			break;
		}
	}

	return ret;
}
EXPORT_SYMBOL_GPL(rtnet_read_from_iov);
