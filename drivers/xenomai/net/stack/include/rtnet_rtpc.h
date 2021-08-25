/***
 *
 *  include/rtnet_rtpc.h
 *
 *  RTnet - real-time networking subsystem
 *
 *  Copyright (C) 2003, 2004 Jan Kiszka <jan.kiszka@web.de>
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

#ifndef __RTNET_RTPC_H_
#define __RTNET_RTPC_H_

#include <linux/init.h>

#include <rtnet_internal.h>

struct rt_proc_call;

typedef int (*rtpc_proc)(struct rt_proc_call *call);
typedef void (*rtpc_copy_back_proc)(struct rt_proc_call *call, void *priv_data);
typedef void (*rtpc_cleanup_proc)(void *priv_data);

struct rt_proc_call {
	struct list_head list_entry;
	int processed;
	rtpc_proc proc;
	int result;
	atomic_t ref_count;
	wait_queue_head_t call_wq;
	rtpc_cleanup_proc cleanup_handler;
	char priv_data[0] __attribute__((aligned(8)));
};

#define CALL_PENDING 1000 /* result value for blocked calls */

int rtnet_rtpc_dispatch_call(rtpc_proc rt_proc, unsigned int timeout,
			     void *priv_data, size_t priv_data_size,
			     rtpc_copy_back_proc copy_back_handler,
			     rtpc_cleanup_proc cleanup_handler);

void rtnet_rtpc_complete_call(struct rt_proc_call *call, int result);
void rtnet_rtpc_complete_call_nrt(struct rt_proc_call *call, int result);

#define rtpc_dispatch_call rtnet_rtpc_dispatch_call
#define rtpc_complete_call rtnet_rtpc_complete_call
#define rtpc_complete_call_nrt rtnet_rtpc_complete_call_nrt

#define rtpc_get_priv(call, type) (type *)(call->priv_data)
#define rtpc_get_result(call) call->result
#define rtpc_set_result(call, new_result) call->result = new_result
#define rtpc_set_cleanup_handler(call, handler) call->cleanup_handler = handler;

int __init rtpc_init(void);
void rtpc_cleanup(void);

#endif /* __RTNET_RTPC_H_ */
