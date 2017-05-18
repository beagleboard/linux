/* ethernet/eth.h
 *
 * RTnet - real-time networking subsystem
 * Copyright (C) 2002 Ulrich Marx <marx@kammer.uni-hannover.de>
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
#ifndef __RTNET_ETH_H_
#define __RTNET_ETH_H_


#include <rtskb.h>
#include <rtdev.h>

extern int rt_eth_header(struct rtskb *skb,struct rtnet_device *rtdev, 
			 unsigned short type,void *daddr,void *saddr,unsigned int len);
extern unsigned short rt_eth_type_trans(struct rtskb *skb, struct rtnet_device *dev);


#endif  /* __RTNET_ETH_H_ */
