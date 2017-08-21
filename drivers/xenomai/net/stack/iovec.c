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

#include <rtnet_iovec.h>


/***
 *  rt_memcpy_tokerneliovec
 */
void rt_memcpy_tokerneliovec(struct iovec *iov, unsigned char *kdata, int len)
{
    while (len > 0)
    {
        if (iov->iov_len)
        {
            int copy = min_t(unsigned int, iov->iov_len, len);

            memcpy(iov->iov_base, kdata, copy);
            kdata+=copy;
            len-=copy;
            iov->iov_len-=copy;
            iov->iov_base+=copy;
        }
        iov++;
    }
}


/***
 *  rt_memcpy_fromkerneliovec
 */
void rt_memcpy_fromkerneliovec(unsigned char *kdata, struct iovec *iov,int len)
{
    while (len > 0)
    {
        if (iov->iov_len)
        {
            int copy=min_t(unsigned int, len, iov->iov_len);

            memcpy(kdata, iov->iov_base, copy);
            len-=copy;
            kdata+=copy;
            iov->iov_base+=copy;
            iov->iov_len-=copy;
        }
        iov++;
    }
}


EXPORT_SYMBOL_GPL(rt_memcpy_tokerneliovec);
EXPORT_SYMBOL_GPL(rt_memcpy_fromkerneliovec);
