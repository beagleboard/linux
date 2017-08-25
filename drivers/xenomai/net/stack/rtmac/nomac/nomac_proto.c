/***
 *
 *  rtmac/nomac/nomac_proto.c
 *
 *  RTmac - real-time networking media access control subsystem
 *  Copyright (C) 2002      Marc Kleine-Budde <kleine-budde@gmx.de>,
 *                2003-2005 Jan Kiszka <Jan.Kiszka@web.de>
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

#include <linux/init.h>

#include <rtdev.h>
#include <rtmac/rtmac_proto.h>
#include <rtmac/nomac/nomac.h>


static struct rtskb_queue   nrt_rtskb_queue;
static rtdm_task_t          wrapper_task;
static rtdm_event_t         wakeup_sem;


int nomac_rt_packet_tx(struct rtskb *rtskb, struct rtnet_device *rtdev)
{
    /* unused here, just to demonstrate access to the discipline state
    struct nomac_priv   *nomac =
        (struct nomac_priv *)rtdev->mac_priv->disc_priv; */
    int                 ret;


    rtcap_mark_rtmac_enqueue(rtskb);

    /* no MAC: we simply transmit the packet under xmit_lock */
    rtdm_mutex_lock(&rtdev->xmit_mutex);
    ret = rtmac_xmit(rtskb);
    rtdm_mutex_unlock(&rtdev->xmit_mutex);

    return ret;
}



int nomac_nrt_packet_tx(struct rtskb *rtskb)
{
    struct rtnet_device *rtdev = rtskb->rtdev;
    /* unused here, just to demonstrate access to the discipline state
    struct nomac_priv   *nomac =
        (struct nomac_priv *)rtdev->mac_priv->disc_priv; */
    int                 ret;


    rtcap_mark_rtmac_enqueue(rtskb);

    /* note: this routine may be called both in rt and non-rt context
     *       => detect and wrap the context if necessary */
    if (!rtdm_in_rt_context()) {
        rtskb_queue_tail(&nrt_rtskb_queue, rtskb);
        rtdm_event_signal(&wakeup_sem);
        return 0;
    } else {
        /* no MAC: we simply transmit the packet under xmit_lock */
        rtdm_mutex_lock(&rtdev->xmit_mutex);
        ret = rtmac_xmit(rtskb);
        rtdm_mutex_unlock(&rtdev->xmit_mutex);

        return ret;
    }
}



void nrt_xmit_task(void *arg)
{
    struct rtskb        *rtskb;
    struct rtnet_device *rtdev;


    while (!rtdm_task_should_stop()) {
	if (rtdm_event_wait(&wakeup_sem) < 0)
	    break;

        while ((rtskb = rtskb_dequeue(&nrt_rtskb_queue))) {
            rtdev = rtskb->rtdev;

            /* no MAC: we simply transmit the packet under xmit_lock */
            rtdm_mutex_lock(&rtdev->xmit_mutex);
            rtmac_xmit(rtskb);
            rtdm_mutex_unlock(&rtdev->xmit_mutex);
        }
    }
}



int nomac_packet_rx(struct rtskb *rtskb)
{
    /* actually, NoMAC doesn't expect any control packet */
    kfree_rtskb(rtskb);

    return 0;
}



int __init nomac_proto_init(void)
{
    int ret;


    rtskb_queue_init(&nrt_rtskb_queue);
    rtdm_event_init(&wakeup_sem, 0);

    ret = rtdm_task_init(&wrapper_task, "rtnet-nomac", nrt_xmit_task, 0,
                         RTDM_TASK_LOWEST_PRIORITY, 0);
    if (ret < 0) {
        rtdm_event_destroy(&wakeup_sem);
        return ret;
    }

    return 0;
}



void nomac_proto_cleanup(void)
{
    rtdm_task_destroy(&wrapper_task);
    rtdm_event_destroy(&wakeup_sem);
}
