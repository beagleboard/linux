/***
 *
 *  stack/rtdev_mgr.c - device error manager
 *
 *  Copyright (C) 2002 Ulrich Marx <marx@kammer.uni-hannover.de>
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

#include <linux/netdevice.h>

#include <rtdev.h>
#include <rtdm/net.h>
#include <rtnet_internal.h>

/***
 *  rtnetif_err_rx: will be called from the  driver
 *
 *
 *  @rtdev - the network-device
 */
void rtnetif_err_rx(struct rtnet_device *rtdev)
{
}

/***
 *  rtnetif_err_tx: will be called from the  driver
 *
 *
 *  @rtdev - the network-device
 */
void rtnetif_err_tx(struct rtnet_device *rtdev)
{
}

/***
 *  do_rtdev_task
 */
/*static void do_rtdev_task(int mgr_id)
{
    struct rtnet_msg msg;
    struct rtnet_mgr *mgr = (struct rtnet_mgr *)mgr_id;

    while (1) {
        rt_mbx_receive(&(mgr->mbx), &msg, sizeof(struct rtnet_msg));
        if (msg.rtdev) {
            rt_printk("RTnet: error on rtdev %s\n", msg.rtdev->name);
        }
    }
}*/

/***
 *  rt_rtdev_connect
 */
void rt_rtdev_connect(struct rtnet_device *rtdev, struct rtnet_mgr *mgr)
{
	/*    rtdev->rtdev_mbx=&(mgr->mbx);*/
}

/***
 *  rt_rtdev_disconnect
 */
void rt_rtdev_disconnect(struct rtnet_device *rtdev)
{
	/*    rtdev->rtdev_mbx=NULL;*/
}

/***
 *  rt_rtdev_mgr_start
 */
int rt_rtdev_mgr_start(struct rtnet_mgr *mgr)
{
	return /*(rt_task_resume(&(mgr->task)))*/ 0;
}

/***
 *  rt_rtdev_mgr_stop
 */
int rt_rtdev_mgr_stop(struct rtnet_mgr *mgr)
{
	return /*(rt_task_suspend(&(mgr->task)))*/ 0;
}

/***
 *  rt_rtdev_mgr_init
 */
int rt_rtdev_mgr_init(struct rtnet_mgr *mgr)
{
	int ret = 0;

	/*    if ( (ret=rt_mbx_init (&(mgr->mbx), sizeof(struct rtnet_msg))) )
        return ret;
    if ( (ret=rt_task_init(&(mgr->task), &do_rtdev_task, (int)mgr, 4096, RTNET_RTDEV_PRIORITY, 0, 0)) )
        return ret;
    if ( (ret=rt_task_resume(&(mgr->task))) )
        return ret;*/

	return (ret);
}

/***
 *  rt_rtdev_mgr_delete
 */
void rt_rtdev_mgr_delete(struct rtnet_mgr *mgr)
{
	/*    rt_task_delete(&(mgr->task));
    rt_mbx_delete(&(mgr->mbx));*/
}

EXPORT_SYMBOL_GPL(rtnetif_err_rx);
EXPORT_SYMBOL_GPL(rtnetif_err_tx);

EXPORT_SYMBOL_GPL(rt_rtdev_connect);
EXPORT_SYMBOL_GPL(rt_rtdev_disconnect);
