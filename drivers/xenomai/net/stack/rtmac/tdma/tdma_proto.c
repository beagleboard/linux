/***
 *
 *  rtmac/tdma/tdma_proto.c
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
#include "asm/div64.h"

#include <rtdev.h>
#include <rtmac/rtmac_proto.h>
#include <rtmac/tdma/tdma_proto.h>


void tdma_xmit_sync_frame(struct tdma_priv *tdma)
{
    struct rtnet_device     *rtdev = tdma->rtdev;
    struct rtskb            *rtskb;
    struct tdma_frm_sync    *sync;


    rtskb = alloc_rtskb(rtdev->hard_header_len + sizeof(struct rtmac_hdr) +
                        sizeof(struct tdma_frm_sync) + 15, &global_pool);
    if (!rtskb)
        goto err_out;

    rtskb_reserve(rtskb,
        (rtdev->hard_header_len + sizeof(struct rtmac_hdr) + 15) & ~15);

    sync = (struct tdma_frm_sync *)rtskb_put(rtskb,
                                             sizeof(struct tdma_frm_sync));

    if (rtmac_add_header(rtdev, rtdev->broadcast,
                         rtskb, RTMAC_TYPE_TDMA, 0) < 0) {
        kfree_rtskb(rtskb);
        goto err_out;
    }

    sync->head.version = __constant_htons(TDMA_FRM_VERSION);
    sync->head.id      = __constant_htons(TDMA_FRM_SYNC);

    sync->cycle_no         = htonl(tdma->current_cycle);
    sync->xmit_stamp       = tdma->clock_offset;
    sync->sched_xmit_stamp =
            cpu_to_be64(tdma->clock_offset + tdma->current_cycle_start);

    rtskb->xmit_stamp = &sync->xmit_stamp;

    rtmac_xmit(rtskb);

    /* signal local waiters */
    rtdm_event_pulse(&tdma->sync_event);

    return;

  err_out:
    /*ERROR*/rtdm_printk("TDMA: Failed to transmit sync frame!\n");
    return;
}



int tdma_xmit_request_cal_frame(struct tdma_priv *tdma, u32 reply_cycle,
                                u64 reply_slot_offset)
{
    struct rtnet_device     *rtdev = tdma->rtdev;
    struct rtskb            *rtskb;
    struct tdma_frm_req_cal *req_cal;
    int                     ret;


    rtskb = alloc_rtskb(rtdev->hard_header_len + sizeof(struct rtmac_hdr) +
                        sizeof(struct tdma_frm_req_cal) + 15, &global_pool);
    ret = -ENOMEM;
    if (!rtskb)
        goto err_out;

    rtskb_reserve(rtskb,
        (rtdev->hard_header_len + sizeof(struct rtmac_hdr) + 15) & ~15);

    req_cal = (struct tdma_frm_req_cal *)
        rtskb_put(rtskb, sizeof(struct tdma_frm_req_cal));

    if ((ret = rtmac_add_header(rtdev, tdma->master_hw_addr,
                                rtskb, RTMAC_TYPE_TDMA, 0)) < 0) {
        kfree_rtskb(rtskb);
        goto err_out;
    }

    req_cal->head.version = __constant_htons(TDMA_FRM_VERSION);
    req_cal->head.id      = __constant_htons(TDMA_FRM_REQ_CAL);

    req_cal->xmit_stamp        = 0;
    req_cal->reply_cycle       = htonl(reply_cycle);
    req_cal->reply_slot_offset = cpu_to_be64(reply_slot_offset);

    rtskb->xmit_stamp = &req_cal->xmit_stamp;

    ret = rtmac_xmit(rtskb);
    if (ret < 0)
        goto err_out;

    return 0;

  err_out:
    /*ERROR*/rtdm_printk("TDMA: Failed to transmit request calibration "
                         "frame!\n");
    return ret;
}



int tdma_rt_packet_tx(struct rtskb *rtskb, struct rtnet_device *rtdev)
{
    struct tdma_priv    *tdma;
    rtdm_lockctx_t      context;
    struct tdma_slot    *slot;
    int                 ret = 0;


    tdma = (struct tdma_priv *)rtdev->mac_priv->disc_priv;

    rtcap_mark_rtmac_enqueue(rtskb);

    rtdm_lock_get_irqsave(&tdma->lock, context);

    slot = tdma->slot_table[(rtskb->priority & RTSKB_CHANNEL_MASK) >>
                            RTSKB_CHANNEL_SHIFT];

    if (unlikely(!slot)) {
        ret = -EAGAIN;
        goto err_out;
    }

    if (unlikely(rtskb->len > slot->size)) {
        ret = -EMSGSIZE;
        goto err_out;
    }

    __rtskb_prio_queue_tail(slot->queue, rtskb);

  err_out:
    rtdm_lock_put_irqrestore(&tdma->lock, context);

    return ret;
}



int tdma_nrt_packet_tx(struct rtskb *rtskb)
{
    struct tdma_priv    *tdma;
    rtdm_lockctx_t      context;
    struct tdma_slot    *slot;
    int                 ret = 0;


    tdma = (struct tdma_priv *)rtskb->rtdev->mac_priv->disc_priv;

    rtcap_mark_rtmac_enqueue(rtskb);

    rtskb->priority = RTSKB_PRIO_VALUE(QUEUE_MIN_PRIO, DEFAULT_NRT_SLOT);

    rtdm_lock_get_irqsave(&tdma->lock, context);

    slot = tdma->slot_table[DEFAULT_NRT_SLOT];

    if (unlikely(!slot)) {
        ret = -EAGAIN;
        goto err_out;
    }

    if (unlikely(rtskb->len > slot->size)) {
        ret = -EMSGSIZE;
        goto err_out;
    }

    __rtskb_prio_queue_tail(slot->queue, rtskb);

  err_out:
    rtdm_lock_put_irqrestore(&tdma->lock, context);

    return ret;
}



int tdma_packet_rx(struct rtskb *rtskb)
{
    struct tdma_priv        *tdma;
    struct tdma_frm_head    *head;
    u64                     delay;
    u64                     cycle_start;
    nanosecs_rel_t          clock_offset;
    struct rt_proc_call     *call;
    struct tdma_request_cal *req_cal_job;
    rtdm_lockctx_t          context;
#ifdef CONFIG_XENO_DRIVERS_NET_TDMA_MASTER
    struct rtskb            *reply_rtskb;
    struct rtnet_device     *rtdev;
    struct tdma_frm_rpl_cal *rpl_cal_frm;
    struct tdma_reply_cal   *rpl_cal_job;
    struct tdma_job         *job;
#endif


    tdma = (struct tdma_priv *)rtskb->rtdev->mac_priv->disc_priv;

    head = (struct tdma_frm_head *)rtskb->data;

    if (head->version != __constant_htons(TDMA_FRM_VERSION))
        goto kfree_out;

    switch (head->id) {
        case __constant_htons(TDMA_FRM_SYNC):
            rtskb_pull(rtskb, sizeof(struct tdma_frm_sync));

            /* see "Time Arithmetics" in the TDMA specification */
            clock_offset = be64_to_cpu(SYNC_FRM(head)->xmit_stamp) +
                    tdma->master_packet_delay_ns;
            clock_offset -= rtskb->time_stamp;

            cycle_start = be64_to_cpu(SYNC_FRM(head)->sched_xmit_stamp) -
                    clock_offset;

            rtdm_lock_get_irqsave(&tdma->lock, context);
            tdma->current_cycle       = ntohl(SYNC_FRM(head)->cycle_no);
            tdma->current_cycle_start = cycle_start;
            tdma->clock_offset        = clock_offset;
            rtdm_lock_put_irqrestore(&tdma->lock, context);

            /* note: Ethernet-specific! */
            memcpy(tdma->master_hw_addr, rtskb->mac.ethernet->h_source,
                   ETH_ALEN);

            set_bit(TDMA_FLAG_RECEIVED_SYNC, &tdma->flags);

            rtdm_event_pulse(&tdma->sync_event);
            break;

#ifdef CONFIG_XENO_DRIVERS_NET_TDMA_MASTER
        case __constant_htons(TDMA_FRM_REQ_CAL):
            RTNET_ASSERT(test_bit(TDMA_FLAG_MASTER, &tdma->flags) &&
                         test_bit(TDMA_FLAG_CALIBRATED, &tdma->flags),
                         break;);

            rtskb_pull(rtskb, sizeof(struct tdma_frm_req_cal));

            rtdev = rtskb->rtdev;

            reply_rtskb = alloc_rtskb(rtdev->hard_header_len +
                                      sizeof(struct rtmac_hdr) +
                                      sizeof(struct tdma_frm_rpl_cal) + 15,
                                      &tdma->cal_rtskb_pool);
            if (unlikely(!reply_rtskb)) {
                /*ERROR*/rtdm_printk("TDMA: Too many calibration requests "
                                     "pending!\n");
                break;
            }

            rtskb_reserve(reply_rtskb, (rtdev->hard_header_len +
                          sizeof(struct rtmac_hdr) + 15) & ~15);

            rpl_cal_frm = (struct tdma_frm_rpl_cal *)
                rtskb_put(reply_rtskb, sizeof(struct tdma_frm_rpl_cal));

            /* note: Ethernet-specific! */
            if (unlikely(rtmac_add_header(rtdev, rtskb->mac.ethernet->h_source,
                                          reply_rtskb, RTMAC_TYPE_TDMA,
                                          0) < 0)) {
                kfree_rtskb(reply_rtskb);
                break;
            }

            rpl_cal_frm->head.version = __constant_htons(TDMA_FRM_VERSION);
            rpl_cal_frm->head.id      = __constant_htons(TDMA_FRM_RPL_CAL);

            rpl_cal_frm->request_xmit_stamp = REQ_CAL_FRM(head)->xmit_stamp;
            rpl_cal_frm->reception_stamp    = cpu_to_be64(rtskb->time_stamp);
            rpl_cal_frm->xmit_stamp         = 0;

            reply_rtskb->xmit_stamp = &rpl_cal_frm->xmit_stamp;

            /* use reply_rtskb memory behind the frame as job buffer */
            rpl_cal_job = (struct tdma_reply_cal *)reply_rtskb->tail;
            RTNET_ASSERT(reply_rtskb->tail +
                sizeof(struct tdma_reply_cal) <= reply_rtskb->buf_end,
                rtskb_over_panic(reply_rtskb, sizeof(struct tdma_reply_cal),
                                 current_text_addr()););

            rpl_cal_job->head.id        = XMIT_RPL_CAL;
            rpl_cal_job->head.ref_count = 0;
            rpl_cal_job->reply_cycle    =
                    ntohl(REQ_CAL_FRM(head)->reply_cycle);
            rpl_cal_job->reply_rtskb    = reply_rtskb;
            rpl_cal_job->reply_offset   =
                    be64_to_cpu(REQ_CAL_FRM(head)->reply_slot_offset);

            rtdm_lock_get_irqsave(&tdma->lock, context);

            job = tdma->current_job;
            while (1) {
                job = list_entry(job->entry.prev, struct tdma_job, entry);
                if ((job == tdma->first_job) ||
                    ((job->id >= 0) &&
                     (SLOT_JOB(job)->offset < rpl_cal_job->reply_offset)) ||
                    ((job->id == XMIT_RPL_CAL) &&
                     (REPLY_CAL_JOB(job)->reply_offset <
                            rpl_cal_job->reply_offset)))
                    break;
            }
            list_add(&rpl_cal_job->head.entry, &job->entry);
            tdma->job_list_revision++;

            rtdm_lock_put_irqrestore(&tdma->lock, context);

            break;
#endif

        case __constant_htons(TDMA_FRM_RPL_CAL):
            rtskb_pull(rtskb, sizeof(struct tdma_frm_rpl_cal));

            /* see "Time Arithmetics" in the TDMA specification */
            delay = (rtskb->time_stamp -
                     be64_to_cpu(RPL_CAL_FRM(head)->request_xmit_stamp)) -
                    (be64_to_cpu(RPL_CAL_FRM(head)->xmit_stamp) -
                     be64_to_cpu(RPL_CAL_FRM(head)->reception_stamp));
            delay = (delay + 1) >> 1;

            rtdm_lock_get_irqsave(&tdma->lock, context);

            call = tdma->calibration_call;
            if (call == NULL) {
                rtdm_lock_put_irqrestore(&tdma->lock, context);
                break;
            }
            req_cal_job = rtpc_get_priv(call, struct tdma_request_cal);

            req_cal_job->result_buffer[--req_cal_job->cal_rounds] = delay;

            if (req_cal_job->cal_rounds > 0) {
                tdma->job_list_revision++;
                list_add(&req_cal_job->head.entry, &tdma->first_job->entry);

                rtdm_lock_put_irqrestore(&tdma->lock, context);

            } else {
                tdma->calibration_call = NULL;

                rtdm_lock_put_irqrestore(&tdma->lock, context);

                rtpc_complete_call(call, 0);
            }

            break;

        default:
            /*ERROR*/rtdm_printk("TDMA: Unknown frame %d!\n", ntohs(head->id));
    }

  kfree_out:
    kfree_rtskb(rtskb);
    return 0;
}



unsigned int tdma_get_mtu(struct rtnet_device *rtdev, unsigned int priority)
{
    struct tdma_priv    *tdma;
    rtdm_lockctx_t      context;
    struct tdma_slot    *slot;
    unsigned int        mtu;


    tdma = (struct tdma_priv *)rtdev->mac_priv->disc_priv;

    rtdm_lock_get_irqsave(&tdma->lock, context);

    slot = tdma->slot_table[(priority & RTSKB_CHANNEL_MASK) >>
                            RTSKB_CHANNEL_SHIFT];

    if (unlikely(!slot)) {
        mtu = rtdev->mtu;
        goto out;
    }

    mtu = slot->mtu;

  out:
    rtdm_lock_put_irqrestore(&tdma->lock, context);

    return mtu;
}
