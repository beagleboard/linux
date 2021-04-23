/* rtnet_iovec.h
 *
 * RTnet - real-time networking subsystem
 * Copyright (C) 1999,2000 Zentropic Computing, LLC
 *               2002 Ulrich Marx <marx@kammer.uni-hannover.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#ifndef __RTNET_IOVEC_H_
#define __RTNET_IOVEC_H_

#ifdef __KERNEL__

#include <linux/uio.h>

struct user_msghdr;
struct rtdm_fd;

ssize_t rtnet_write_to_iov(struct rtdm_fd *fd, struct iovec *iov, int iovlen,
			   const void *data, size_t len);

ssize_t rtnet_read_from_iov(struct rtdm_fd *fd, struct iovec *iov, int iovlen,
			    void *data, size_t len);
#endif /* __KERNEL__ */

#endif /* __RTNET_IOVEC_H_ */
