/***
 *
 *  rtmac/tdma/tdma_worker.c
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

#include <rtmac/rtmac_proto.h>
#include <rtmac/tdma/tdma_proto.h>


static void do_slot_job(struct tdma_priv *tdma, struct tdma_slot *job,
                        rtdm_lockctx_t lockctx)
{
    struct rtskb *rtskb;

    if ((job->period != 1) &&
        (tdma->current_cycle % job->period != job->phasing))
        return;

    rtdm_lock_put_irqrestore(&tdma->lock, lockctx);

    /* wait for slot begin, then send one pending packet */
    rtdm_task_sleep_abs(tdma->current_cycle_start + SLOT_JOB(job)->offset,
                        RTDM_TIMERMODE_REALTIME);

    rtdm_lock_get_irqsave(&tdma->lock, lockctx);
    rtskb = __rtskb_prio_dequeue(SLOT_JOB(job)->queue);
    if (!rtskb)
        return;
    rtdm_lock_put_irqrestore(&tdma->lock, lockctx);

    rtmac_xmit(rtskb);

    rtdm_lock_get_irqsave(&tdma->lock, lockctx);
}

static void do_xmit_sync_job(struct tdma_priv *tdma, rtdm_lockctx_t lockctx)
{
    rtdm_lock_put_irqrestore(&tdma->lock, lockctx);

    /* wait for beginning of next cycle, then send sync */
    rtdm_task_sleep_abs(tdma->current_cycle_start + tdma->cycle_period,
                        RTDM_TIMERMODE_REALTIME);
    rtdm_lock_get_irqsave(&tdma->lock, lockctx);
    tdma->current_cycle++;
    tdma->current_cycle_start += tdma->cycle_period;
    rtdm_lock_put_irqrestore(&tdma->lock, lockctx);

    tdma_xmit_sync_frame(tdma);

    rtdm_lock_get_irqsave(&tdma->lock, lockctx);
}

static void do_backup_sync_job(struct tdma_priv *tdma, rtdm_lockctx_t lockctx)
{
    rtdm_lock_put_irqrestore(&tdma->lock, lockctx);

    /* wait for backup slot */
    rtdm_task_sleep_abs(tdma->current_cycle_start + tdma->backup_sync_inc,
                        RTDM_TIMERMODE_REALTIME);

    /* take over sync transmission if all earlier masters failed */
    if (!test_and_clear_bit(TDMA_FLAG_RECEIVED_SYNC, &tdma->flags)) {
        rtdm_lock_get_irqsave(&tdma->lock, lockctx);
        tdma->current_cycle++;
        tdma->current_cycle_start += tdma->cycle_period;
        rtdm_lock_put_irqrestore(&tdma->lock, lockctx);

        tdma_xmit_sync_frame(tdma);

        set_bit(TDMA_FLAG_BACKUP_ACTIVE, &tdma->flags);
    } else
        clear_bit(TDMA_FLAG_BACKUP_ACTIVE, &tdma->flags);

    rtdm_lock_get_irqsave(&tdma->lock, lockctx);
}

static struct tdma_job *do_request_cal_job(struct tdma_priv *tdma,
                                           struct tdma_request_cal *job,
                                           rtdm_lockctx_t lockctx)
{
    struct rt_proc_call *call;
    struct tdma_job     *prev_job;
    int                 err;

    if ((job->period != 1) &&
        (tdma->current_cycle % job->period != job->phasing))
        return &job->head;

    /* remove job until we get a reply */
    __list_del(job->head.entry.prev, job->head.entry.next);
    job->head.ref_count--;
    prev_job = tdma->current_job =
        list_entry(job->head.entry.prev, struct tdma_job, entry);
    prev_job->ref_count++;
    tdma->job_list_revision++;

    rtdm_lock_put_irqrestore(&tdma->lock, lockctx);

    rtdm_task_sleep_abs(tdma->current_cycle_start + job->offset,
                        RTDM_TIMERMODE_REALTIME);
    err = tdma_xmit_request_cal_frame(tdma,
            tdma->current_cycle + job->period, job->offset);

    rtdm_lock_get_irqsave(&tdma->lock, lockctx);

    /* terminate call on error */
    if (err < 0) {
        call = tdma->calibration_call;
        tdma->calibration_call = NULL;

        if (call) {
            rtdm_lock_put_irqrestore(&tdma->lock, lockctx);
            rtpc_complete_call(call, err);
            rtdm_lock_get_irqsave(&tdma->lock, lockctx);
        }
    }

    return prev_job;
}

static struct tdma_job *do_reply_cal_job(struct tdma_priv *tdma,
                                         struct tdma_reply_cal *job,
                                         rtdm_lockctx_t lockctx)
{
    struct tdma_job *prev_job;

    if (job->reply_cycle > tdma->current_cycle)
        return &job->head;

    /* remove the job */
    __list_del(job->head.entry.prev, job->head.entry.next);
    job->head.ref_count--;
    prev_job = tdma->current_job =
        list_entry(job->head.entry.prev, struct tdma_job, entry);
    prev_job->ref_count++;
    tdma->job_list_revision++;

    rtdm_lock_put_irqrestore(&tdma->lock, lockctx);

    if (job->reply_cycle == tdma->current_cycle) {
        /* send reply in the assigned slot */
        rtdm_task_sleep_abs(tdma->current_cycle_start + job->reply_offset,
                            RTDM_TIMERMODE_REALTIME);
        rtmac_xmit(job->reply_rtskb);
    } else {
        /* cleanup if cycle already passed */
        kfree_rtskb(job->reply_rtskb);
    }

    rtdm_lock_get_irqsave(&tdma->lock, lockctx);

    return prev_job;
}

void tdma_worker(void *arg)
{
    struct tdma_priv    *tdma = (struct tdma_priv *)arg;
    struct tdma_job     *job;
    rtdm_lockctx_t      lockctx;


    rtdm_event_wait(&tdma->worker_wakeup);

    rtdm_lock_get_irqsave(&tdma->lock, lockctx);

    job = tdma->first_job;

    while (!rtdm_task_should_stop()) {
        job->ref_count++;
        switch (job->id) {
            case WAIT_ON_SYNC:
                rtdm_lock_put_irqrestore(&tdma->lock, lockctx);
                rtdm_event_wait(&tdma->sync_event);
                rtdm_lock_get_irqsave(&tdma->lock, lockctx);
                break;

            case XMIT_REQ_CAL:
                job = do_request_cal_job(tdma, REQUEST_CAL_JOB(job), lockctx);
                break;

#ifdef CONFIG_XENO_DRIVERS_NET_TDMA_MASTER
            case XMIT_SYNC:
                do_xmit_sync_job(tdma, lockctx);
                break;

            case BACKUP_SYNC:
                do_backup_sync_job(tdma, lockctx);
                break;

            case XMIT_RPL_CAL:
                job = do_reply_cal_job(tdma, REPLY_CAL_JOB(job), lockctx);
                break;
#endif /* CONFIG_XENO_DRIVERS_NET_TDMA_MASTER */

            default:
                do_slot_job(tdma, SLOT_JOB(job), lockctx);
                break;
        }
        job->ref_count--;

        job = tdma->current_job =
            list_entry(job->entry.next, struct tdma_job, entry);
    }

    rtdm_lock_put_irqrestore(&tdma->lock, lockctx);
}
