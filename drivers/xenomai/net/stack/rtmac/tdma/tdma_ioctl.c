/***
 *
 *  rtmac/tdma/tdma_ioctl.c
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

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <asm/div64.h>

#include <tdma_chrdev.h>
#include <rtmac/rtmac_vnic.h>
#include <rtmac/tdma/tdma.h>

#ifdef CONFIG_XENO_DRIVERS_NET_TDMA_MASTER
static int tdma_ioctl_master(struct rtnet_device *rtdev,
			     struct tdma_config *cfg)
{
	struct tdma_priv *tdma;
	u64 cycle_ms;
	unsigned int table_size;
	int ret;

	if (rtdev->mac_priv == NULL) {
		ret = rtmac_disc_attach(rtdev, &tdma_disc);
		if (ret < 0)
			return ret;
	}

	tdma = (struct tdma_priv *)rtdev->mac_priv->disc_priv;
	if (tdma->magic != TDMA_MAGIC) {
		/* note: we don't clean up an unknown discipline */
		return -ENOTTY;
	}

	if (test_bit(TDMA_FLAG_ATTACHED, &tdma->flags)) {
		/* already attached */
		return -EBUSY;
	}

	set_bit(TDMA_FLAG_MASTER, &tdma->flags);

	tdma->cal_rounds = cfg->args.master.cal_rounds;

	/* search at least 3 cycle periods for other masters */
	cycle_ms = cfg->args.master.cycle_period;
	do_div(cycle_ms, 1000000);
	if (cycle_ms == 0)
		cycle_ms = 1;
	msleep(3 * cycle_ms);

	if (rtskb_module_pool_init(&tdma->cal_rtskb_pool,
				   cfg->args.master.max_cal_requests) !=
	    cfg->args.master.max_cal_requests) {
		ret = -ENOMEM;
		goto err_out;
	}

	table_size = sizeof(struct tdma_slot *) *
		     ((cfg->args.master.max_slot_id >= 1) ?
			      cfg->args.master.max_slot_id + 1 :
			      2);

	tdma->slot_table = (struct tdma_slot **)kmalloc(table_size, GFP_KERNEL);
	if (!tdma->slot_table) {
		ret = -ENOMEM;
		goto err_out;
	}
	tdma->max_slot_id = cfg->args.master.max_slot_id;
	memset(tdma->slot_table, 0, table_size);

	tdma->cycle_period = cfg->args.master.cycle_period;
	tdma->sync_job.ref_count = 0;
	INIT_LIST_HEAD(&tdma->sync_job.entry);

	if (cfg->args.master.backup_sync_offset == 0)
		tdma->sync_job.id = XMIT_SYNC;
	else {
		set_bit(TDMA_FLAG_BACKUP_MASTER, &tdma->flags);
		tdma->sync_job.id = BACKUP_SYNC;
		tdma->backup_sync_inc = cfg->args.master.backup_sync_offset +
					tdma->cycle_period;
	}

	/* did we detect another active master? */
	if (test_bit(TDMA_FLAG_RECEIVED_SYNC, &tdma->flags)) {
		/* become a slave, we need to calibrate first */
		tdma->sync_job.id = WAIT_ON_SYNC;
	} else {
		if (test_bit(TDMA_FLAG_BACKUP_MASTER, &tdma->flags))
			printk("TDMA: warning, no primary master detected!\n");
		set_bit(TDMA_FLAG_CALIBRATED, &tdma->flags);
		tdma->current_cycle_start = rtdm_clock_read();
	}

	tdma->first_job = tdma->current_job = &tdma->sync_job;

	rtdm_event_signal(&tdma->worker_wakeup);

	set_bit(TDMA_FLAG_ATTACHED, &tdma->flags);

	return 0;

err_out:
	rtmac_disc_detach(rtdev);
	return ret;
}
#endif /* CONFIG_XENO_DRIVERS_NET_TDMA_MASTER */

static int tdma_ioctl_slave(struct rtnet_device *rtdev, struct tdma_config *cfg)
{
	struct tdma_priv *tdma;
	unsigned int table_size;
	int ret;

	if (rtdev->mac_priv == NULL) {
		ret = rtmac_disc_attach(rtdev, &tdma_disc);
		if (ret < 0)
			return ret;
	}

	tdma = (struct tdma_priv *)rtdev->mac_priv->disc_priv;
	if (tdma->magic != TDMA_MAGIC) {
		/* note: we don't clean up an unknown discipline */
		return -ENOTTY;
	}

	if (test_bit(TDMA_FLAG_ATTACHED, &tdma->flags)) {
		/* already attached */
		return -EBUSY;
	}

	tdma->cal_rounds = cfg->args.slave.cal_rounds;
	if (tdma->cal_rounds == 0)
		set_bit(TDMA_FLAG_CALIBRATED, &tdma->flags);

	table_size = sizeof(struct tdma_slot *) *
		     ((cfg->args.slave.max_slot_id >= 1) ?
			      cfg->args.slave.max_slot_id + 1 :
			      2);

	tdma->slot_table = (struct tdma_slot **)kmalloc(table_size, GFP_KERNEL);
	if (!tdma->slot_table) {
		ret = -ENOMEM;
		goto err_out;
	}
	tdma->max_slot_id = cfg->args.slave.max_slot_id;
	memset(tdma->slot_table, 0, table_size);

	tdma->sync_job.id = WAIT_ON_SYNC;
	tdma->sync_job.ref_count = 0;
	INIT_LIST_HEAD(&tdma->sync_job.entry);

	tdma->first_job = tdma->current_job = &tdma->sync_job;

	rtdm_event_signal(&tdma->worker_wakeup);

	set_bit(TDMA_FLAG_ATTACHED, &tdma->flags);

	return 0;

err_out:
	rtmac_disc_detach(rtdev);
	return ret;
}

static int tdma_ioctl_cal_result_size(struct rtnet_device *rtdev,
				      struct tdma_config *cfg)
{
	struct tdma_priv *tdma;

	if (rtdev->mac_priv == NULL)
		return -ENOTTY;

	tdma = (struct tdma_priv *)rtdev->mac_priv->disc_priv;
	if (tdma->magic != TDMA_MAGIC)
		return -ENOTTY;

	if (!test_bit(TDMA_FLAG_CALIBRATED, &tdma->flags))
		return tdma->cal_rounds;
	else
		return 0;
}

int start_calibration(struct rt_proc_call *call)
{
	struct tdma_request_cal *req_cal;
	struct tdma_priv *tdma;
	rtdm_lockctx_t context;

	req_cal = rtpc_get_priv(call, struct tdma_request_cal);
	tdma = req_cal->tdma;

	/* there are no slots yet, simply add this job after first_job */
	rtdm_lock_get_irqsave(&tdma->lock, context);
	tdma->calibration_call = call;
	tdma->job_list_revision++;
	list_add(&req_cal->head.entry, &tdma->first_job->entry);
	rtdm_lock_put_irqrestore(&tdma->lock, context);

	return -CALL_PENDING;
}

void copyback_calibration(struct rt_proc_call *call, void *priv_data)
{
	struct tdma_request_cal *req_cal;
	struct tdma_priv *tdma;
	int i;
	u64 value;
	u64 average = 0;
	u64 min = 0x7FFFFFFFFFFFFFFFLL;
	u64 max = 0;

	req_cal = rtpc_get_priv(call, struct tdma_request_cal);
	tdma = req_cal->tdma;

	for (i = 0; i < tdma->cal_rounds; i++) {
		value = req_cal->result_buffer[i];
		average += value;
		if (value < min)
			min = value;
		if (value > max)
			max = value;
		if ((req_cal->cal_results) &&
		    (copy_to_user(&req_cal->cal_results[i], &value,
				  sizeof(value)) != 0))
			rtpc_set_result(call, -EFAULT);
	}
	do_div(average, tdma->cal_rounds);
	tdma->master_packet_delay_ns = average;

	average += 500;
	do_div(average, 1000);
	min += 500;
	do_div(min, 1000);
	max += 500;
	do_div(max, 1000);
	printk("TDMA: calibrated master-to-slave packet delay: "
	       "%ld us (min/max: %ld/%ld us)\n",
	       (unsigned long)average, (unsigned long)min, (unsigned long)max);
}

void cleanup_calibration(void *priv_data)
{
	struct tdma_request_cal *req_cal;

	req_cal = (struct tdma_request_cal *)priv_data;
	kfree(req_cal->result_buffer);
}

static int tdma_ioctl_set_slot(struct rtnet_device *rtdev,
			       struct tdma_config *cfg)
{
	struct tdma_priv *tdma;
	int id;
	int jnt_id;
	struct tdma_slot *slot, *old_slot;
	struct tdma_job *job, *prev_job;
	struct tdma_request_cal req_cal;
	struct rtskb *rtskb;
	unsigned int job_list_revision;
	rtdm_lockctx_t context;
	int ret;

	if (rtdev->mac_priv == NULL)
		return -ENOTTY;

	tdma = (struct tdma_priv *)rtdev->mac_priv->disc_priv;
	if (tdma->magic != TDMA_MAGIC)
		return -ENOTTY;

	id = cfg->args.set_slot.id;
	if (id > tdma->max_slot_id)
		return -EINVAL;

	if (cfg->args.set_slot.size == 0)
		cfg->args.set_slot.size = rtdev->mtu;
	else if (cfg->args.set_slot.size > rtdev->mtu)
		return -EINVAL;

	jnt_id = cfg->args.set_slot.joint_slot;
	if ((jnt_id >= 0) &&
	    ((jnt_id >= tdma->max_slot_id) || (tdma->slot_table[jnt_id] == 0) ||
	     (tdma->slot_table[jnt_id]->mtu != cfg->args.set_slot.size)))
		return -EINVAL;

	slot = (struct tdma_slot *)kmalloc(sizeof(struct tdma_slot),
					   GFP_KERNEL);
	if (!slot)
		return -ENOMEM;

	if (!test_bit(TDMA_FLAG_CALIBRATED, &tdma->flags)) {
		req_cal.head.id = XMIT_REQ_CAL;
		req_cal.head.ref_count = 0;
		req_cal.tdma = tdma;
		req_cal.offset = cfg->args.set_slot.offset;
		req_cal.period = cfg->args.set_slot.period;
		req_cal.phasing = cfg->args.set_slot.phasing;
		req_cal.cal_rounds = tdma->cal_rounds;
		req_cal.cal_results = cfg->args.set_slot.cal_results;

		req_cal.result_buffer =
			kmalloc(req_cal.cal_rounds * sizeof(u64), GFP_KERNEL);
		if (!req_cal.result_buffer) {
			kfree(slot);
			return -ENOMEM;
		}

		ret = rtpc_dispatch_call(start_calibration, 0, &req_cal,
					 sizeof(req_cal), copyback_calibration,
					 cleanup_calibration);
		if (ret < 0) {
			/* kick out any pending calibration job before returning */
			rtdm_lock_get_irqsave(&tdma->lock, context);

			job = list_entry(tdma->first_job->entry.next,
					 struct tdma_job, entry);
			if (job != tdma->first_job) {
				__list_del(job->entry.prev, job->entry.next);

				while (job->ref_count > 0) {
					rtdm_lock_put_irqrestore(&tdma->lock,
								 context);
					msleep(100);
					rtdm_lock_get_irqsave(&tdma->lock,
							      context);
				}
			}

			rtdm_lock_put_irqrestore(&tdma->lock, context);

			kfree(slot);
			return ret;
		}

#ifdef CONFIG_XENO_DRIVERS_NET_TDMA_MASTER
		if (test_bit(TDMA_FLAG_MASTER, &tdma->flags)) {
			u32 cycle_no = (volatile u32)tdma->current_cycle;
			u64 cycle_ms;

			/* switch back to [backup] master mode */
			if (test_bit(TDMA_FLAG_BACKUP_MASTER, &tdma->flags))
				tdma->sync_job.id = BACKUP_SYNC;
			else
				tdma->sync_job.id = XMIT_SYNC;

			/* wait 2 cycle periods for the mode switch */
			cycle_ms = tdma->cycle_period;
			do_div(cycle_ms, 1000000);
			if (cycle_ms == 0)
				cycle_ms = 1;
			msleep(2 * cycle_ms);

			/* catch the very unlikely case that the current master died
               while we just switched the mode */
			if (cycle_no == (volatile u32)tdma->current_cycle) {
				kfree(slot);
				return -ETIME;
			}
		}
#endif /* CONFIG_XENO_DRIVERS_NET_TDMA_MASTER */

		set_bit(TDMA_FLAG_CALIBRATED, &tdma->flags);
	}

	slot->head.id = id;
	slot->head.ref_count = 0;
	slot->period = cfg->args.set_slot.period;
	slot->phasing = cfg->args.set_slot.phasing;
	slot->mtu = cfg->args.set_slot.size;
	slot->size = cfg->args.set_slot.size + rtdev->hard_header_len;
	slot->offset = cfg->args.set_slot.offset;
	slot->queue = &slot->local_queue;
	rtskb_prio_queue_init(&slot->local_queue);

	if (jnt_id >= 0) /* all other validation tests performed above */
		slot->queue = tdma->slot_table[jnt_id]->queue;

	old_slot = tdma->slot_table[id];
	if ((id == DEFAULT_NRT_SLOT) &&
	    (old_slot == tdma->slot_table[DEFAULT_SLOT]))
		old_slot = NULL;

restart:
	job_list_revision = tdma->job_list_revision;

	if (!old_slot) {
		job = tdma->first_job;
		while (1) {
			prev_job = job;
			job = list_entry(job->entry.next, struct tdma_job,
					 entry);
			if (((job->id >= 0) &&
			     ((slot->offset < SLOT_JOB(job)->offset) ||
			      ((slot->offset == SLOT_JOB(job)->offset) &&
			       (slot->head.id <= SLOT_JOB(job)->head.id)))) ||
#ifdef CONFIG_XENO_DRIVERS_NET_TDMA_MASTER
			    ((job->id == XMIT_RPL_CAL) &&
			     (slot->offset <
			      REPLY_CAL_JOB(job)->reply_offset)) ||
#endif /* CONFIG_XENO_DRIVERS_NET_TDMA_MASTER */
			    (job == tdma->first_job))
				break;
		}

	} else
		prev_job = list_entry(old_slot->head.entry.prev,
				      struct tdma_job, entry);

	rtdm_lock_get_irqsave(&tdma->lock, context);

	if (job_list_revision != tdma->job_list_revision) {
		rtdm_lock_put_irqrestore(&tdma->lock, context);

		msleep(100);
		goto restart;
	}

	if (old_slot)
		__list_del(old_slot->head.entry.prev,
			   old_slot->head.entry.next);

	list_add(&slot->head.entry, &prev_job->entry);
	tdma->slot_table[id] = slot;
	if ((id == DEFAULT_SLOT) &&
	    (tdma->slot_table[DEFAULT_NRT_SLOT] == old_slot))
		tdma->slot_table[DEFAULT_NRT_SLOT] = slot;

	if (old_slot) {
		while (old_slot->head.ref_count > 0) {
			rtdm_lock_put_irqrestore(&tdma->lock, context);
			msleep(100);
			rtdm_lock_get_irqsave(&tdma->lock, context);
		}

		rtdm_lock_put_irqrestore(&tdma->lock, context);

		/* search for other slots linked to the old one */
		for (jnt_id = 0; jnt_id < tdma->max_slot_id; jnt_id++)
			if ((tdma->slot_table[jnt_id] != 0) &&
			    (tdma->slot_table[jnt_id]->queue ==
			     &old_slot->local_queue)) {
				/* found a joint slot, move or detach it now */
				rtdm_lock_get_irqsave(&tdma->lock, context);

				while (tdma->slot_table[jnt_id]->head.ref_count >
				       0) {
					rtdm_lock_put_irqrestore(&tdma->lock,
								 context);
					msleep(100);
					rtdm_lock_get_irqsave(&tdma->lock,
							      context);
				}

				/* If the new slot size is larger, detach the other slot,
                 * update it otherwise. */
				if (slot->mtu > tdma->slot_table[jnt_id]->mtu)
					tdma->slot_table[jnt_id]->queue =
						&tdma->slot_table[jnt_id]
							 ->local_queue;
				else {
					tdma->slot_table[jnt_id]->mtu =
						slot->mtu;
					tdma->slot_table[jnt_id]->queue =
						slot->queue;
				}

				rtdm_lock_put_irqrestore(&tdma->lock, context);
			}
	} else
		rtdm_lock_put_irqrestore(&tdma->lock, context);

	rtmac_vnic_set_max_mtu(rtdev, cfg->args.set_slot.size);

	if (old_slot) {
		/* avoid that the formerly joint queue gets purged */
		old_slot->queue = &old_slot->local_queue;

		/* Without any reference to the old job and no joint slots we can
         * safely purge its queue without lock protection.
         * NOTE: Reconfiguring a slot during runtime may lead to packet
         *       drops! */
		while ((rtskb = __rtskb_prio_dequeue(old_slot->queue)))
			kfree_rtskb(rtskb);

		kfree(old_slot);
	}

	return 0;
}

int tdma_cleanup_slot(struct tdma_priv *tdma, struct tdma_slot *slot)
{
	struct rtskb *rtskb;
	unsigned int id, jnt_id;
	rtdm_lockctx_t context;

	if (!slot)
		return -EINVAL;

	id = slot->head.id;

	rtdm_lock_get_irqsave(&tdma->lock, context);

	__list_del(slot->head.entry.prev, slot->head.entry.next);

	if (id == DEFAULT_NRT_SLOT)
		tdma->slot_table[DEFAULT_NRT_SLOT] =
			tdma->slot_table[DEFAULT_SLOT];
	else {
		if ((id == DEFAULT_SLOT) &&
		    (tdma->slot_table[DEFAULT_NRT_SLOT] == slot))
			tdma->slot_table[DEFAULT_NRT_SLOT] = NULL;
		tdma->slot_table[id] = NULL;
	}

	while (slot->head.ref_count > 0) {
		rtdm_lock_put_irqrestore(&tdma->lock, context);
		msleep(100);
		rtdm_lock_get_irqsave(&tdma->lock, context);
	}

	rtdm_lock_put_irqrestore(&tdma->lock, context);

	/* search for other slots linked to this one */
	for (jnt_id = 0; jnt_id < tdma->max_slot_id; jnt_id++)
		if ((tdma->slot_table[jnt_id] != 0) &&
		    (tdma->slot_table[jnt_id]->queue == &slot->local_queue)) {
			/* found a joint slot, detach it now under lock protection */
			rtdm_lock_get_irqsave(&tdma->lock, context);

			while (tdma->slot_table[jnt_id]->head.ref_count > 0) {
				rtdm_lock_put_irqrestore(&tdma->lock, context);
				msleep(100);
				rtdm_lock_get_irqsave(&tdma->lock, context);
			}
			tdma->slot_table[jnt_id]->queue =
				&tdma->slot_table[jnt_id]->local_queue;

			rtdm_lock_put_irqrestore(&tdma->lock, context);
		}

	/* avoid that the formerly joint queue gets purged */
	slot->queue = &slot->local_queue;

	/* No need to protect the queue access here -
     * no one is referring to this job anymore
     * (ref_count == 0, all joint slots detached). */
	while ((rtskb = __rtskb_prio_dequeue(slot->queue)))
		kfree_rtskb(rtskb);

	kfree(slot);

	return 0;
}

static int tdma_ioctl_remove_slot(struct rtnet_device *rtdev,
				  struct tdma_config *cfg)
{
	struct tdma_priv *tdma;
	int id;

	if (rtdev->mac_priv == NULL)
		return -ENOTTY;

	tdma = (struct tdma_priv *)rtdev->mac_priv->disc_priv;
	if (tdma->magic != TDMA_MAGIC)
		return -ENOTTY;

	id = cfg->args.remove_slot.id;
	if (id > tdma->max_slot_id)
		return -EINVAL;

	if ((id == DEFAULT_NRT_SLOT) && (tdma->slot_table[DEFAULT_NRT_SLOT] ==
					 tdma->slot_table[DEFAULT_SLOT]))
		return -EINVAL;

	return tdma_cleanup_slot(tdma, tdma->slot_table[id]);
}

static int tdma_ioctl_detach(struct rtnet_device *rtdev)
{
	struct tdma_priv *tdma;
	int ret;

	if (rtdev->mac_priv == NULL)
		return -ENOTTY;

	tdma = (struct tdma_priv *)rtdev->mac_priv->disc_priv;
	if (tdma->magic != TDMA_MAGIC)
		return -ENOTTY;

	ret = rtmac_disc_detach(rtdev);

	return ret;
}

int tdma_ioctl(struct rtnet_device *rtdev, unsigned int request,
	       unsigned long arg)
{
	struct tdma_config cfg;
	int ret;

	ret = copy_from_user(&cfg, (void *)arg, sizeof(cfg));
	if (ret != 0)
		return -EFAULT;

	if (mutex_lock_interruptible(&rtdev->nrt_lock))
		return -ERESTARTSYS;

	switch (request) {
#ifdef CONFIG_XENO_DRIVERS_NET_TDMA_MASTER
	case TDMA_IOC_MASTER:
		ret = tdma_ioctl_master(rtdev, &cfg);
		break;
#endif
	case TDMA_IOC_SLAVE:
		ret = tdma_ioctl_slave(rtdev, &cfg);
		break;

	case TDMA_IOC_CAL_RESULT_SIZE:
		ret = tdma_ioctl_cal_result_size(rtdev, &cfg);
		break;

	case TDMA_IOC_SET_SLOT:
		ret = tdma_ioctl_set_slot(rtdev, &cfg);
		break;

	case TDMA_IOC_REMOVE_SLOT:
		ret = tdma_ioctl_remove_slot(rtdev, &cfg);
		break;

	case TDMA_IOC_DETACH:
		ret = tdma_ioctl_detach(rtdev);
		break;

	default:
		ret = -ENOTTY;
	}

	mutex_unlock(&rtdev->nrt_lock);

	return ret;
}
