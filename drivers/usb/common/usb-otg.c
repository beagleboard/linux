/**
 * drivers/usb/common/usb-otg.c - USB OTG core
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com
 * Author: Roger Quadros <rogerq@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/hrtimer.h>
#include <linux/list.h>
#include <linux/usb/otg.h>
#include <linux/usb/phy.h>	/* enum usb_otg_state */
#include <linux/usb/gadget.h>
#include <linux/workqueue.h>

#include "usb-otg.h"

/* to link timer with callback data */
struct otg_timer {
	struct hrtimer timer;
	ktime_t timeout;
	/* callback data */
	int *timeout_bit;
	struct otg_data *otgd;
};

struct otg_hcd {
	struct usb_hcd *hcd;
	unsigned int irqnum;
	unsigned long irqflags;
	struct otg_hcd_ops *ops;
};

struct otg_data {
	struct device *dev;	/* HCD & GCD's parent device */

	bool drd_only;		/* Dual-role only, no OTG features */
	struct otg_fsm fsm;
	/* HCD, GCD and usb_otg_state are present in otg_fsm->otg
	 * HCD is bus_to_hcd(fsm->otg->host)
	 * GCD is fsm->otg->gadget
	 */
	struct otg_fsm_ops fsm_ops;	/* private copy for override */
	struct usb_otg otg;		/* allocator for fsm->otg */

	struct otg_hcd primary_hcd;
	struct otg_hcd shared_hcd;

	struct otg_gadget_ops *gadget_ops; /* interface to gadget f/w */

	/* saved hooks to OTG device */
	int (*start_host)(struct otg_fsm *fsm, int on);
	int (*start_gadget)(struct otg_fsm *fsm, int on);

	struct list_head list;

	u32 flags;
#define OTG_FLAG_GADGET_RUNNING (1 << 0)
#define OTG_FLAG_HOST_RUNNING (1 << 1)

	struct work_struct work;	/* OTG FSM work */
	struct workqueue_struct *wq;

	struct otg_timer timers[NUM_OTG_FSM_TIMERS];

	bool fsm_running;
	/* use otg->fsm.lock for serializing access */
};

/* OTG device list */
LIST_HEAD(otg_list);
static DEFINE_MUTEX(otg_list_mutex);

static int usb_otg_hcd_is_primary_hcd(struct usb_hcd *hcd)
{
	if (!hcd->primary_hcd)
		return 1;
	return hcd == hcd->primary_hcd;
}

/**
 * check if device is in our OTG list and return
 * otg_data, else NULL.
 *
 * otg_list_mutex must be held.
 */
static struct otg_data *usb_otg_device_get_otgd(struct device *parent_dev)
{
	struct otg_data *otgd;

	list_for_each_entry(otgd, &otg_list, list) {
		if (otgd->dev == parent_dev)
			return otgd;
	}

	return NULL;
}

/**
 * timer callback to set timeout bit and kick FSM
 */
static enum hrtimer_restart set_tmout(struct hrtimer *data)
{
	struct otg_timer *otgtimer;

	otgtimer = container_of(data, struct otg_timer, timer);
	if (otgtimer->timeout_bit)
		*otgtimer->timeout_bit = 1;

	usb_otg_sync_inputs(&otgtimer->otgd->fsm);

	return HRTIMER_NORESTART;
}

/**
 * Initialize one OTG timer with callback, timeout and timeout bit
 */
static void otg_timer_init(enum otg_fsm_timer id, struct otg_data *otgd,
			   enum hrtimer_restart (*callback)(struct hrtimer *),
			   unsigned long expires_ms,
			   int *timeout_bit)
{
	struct otg_timer *otgtimer = &otgd->timers[id];
	struct hrtimer *timer = &otgtimer->timer;

	otgtimer->timeout = ms_to_ktime(expires_ms);
	hrtimer_init(timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	timer->function = callback;

	otgtimer->timeout_bit = timeout_bit;
	otgtimer->otgd = otgd;
}

/**
 * Initialize standard OTG timers
 */
static void usb_otg_init_timers(struct otg_data *otgd)
{
	struct otg_fsm *fsm = &otgd->fsm;

	otg_timer_init(A_WAIT_VRISE, otgd, set_tmout, TA_WAIT_VRISE,
		       &fsm->a_wait_vrise_tmout);
	otg_timer_init(A_WAIT_VFALL, otgd, set_tmout, TA_WAIT_VFALL,
		       &fsm->a_wait_vfall_tmout);
	otg_timer_init(A_WAIT_BCON, otgd, set_tmout, TA_WAIT_BCON,
		       &fsm->a_wait_bcon_tmout);
	otg_timer_init(A_AIDL_BDIS, otgd, set_tmout, TA_AIDL_BDIS,
		       &fsm->a_aidl_bdis_tmout);
	otg_timer_init(A_BIDL_ADIS, otgd, set_tmout, TA_BIDL_ADIS,
		       &fsm->a_bidl_adis_tmout);
	otg_timer_init(B_ASE0_BRST, otgd, set_tmout, TB_ASE0_BRST,
		       &fsm->b_ase0_brst_tmout);

	otg_timer_init(B_SE0_SRP, otgd, set_tmout, TB_SE0_SRP, &fsm->b_se0_srp);
	otg_timer_init(B_SRP_FAIL, otgd, set_tmout, TB_SRP_FAIL,
		       &fsm->b_srp_done);

	otg_timer_init(A_WAIT_ENUM, otgd, set_tmout, TB_SRP_FAIL, NULL);
}

/**
 * OTG FSM ops function to add timer
 */
static void usb_otg_add_timer(struct otg_fsm *fsm, enum otg_fsm_timer id)
{
	struct otg_data *otgd = container_of(fsm, struct otg_data, fsm);
	struct otg_timer *otgtimer = &otgd->timers[id];
	struct hrtimer *timer = &otgtimer->timer;

	if (!otgd->fsm_running)
		return;

	/* if timer is already active, exit */
	if (hrtimer_active(timer)) {
		dev_err(otgd->dev, "otg: timer %d is already running\n", id);
		return;
	}

	hrtimer_start(timer, otgtimer->timeout, HRTIMER_MODE_REL);
}

/**
 * OTG FSM ops function to delete timer
 */
static void usb_otg_del_timer(struct otg_fsm *fsm, enum otg_fsm_timer id)
{
	struct otg_data *otgd = container_of(fsm, struct otg_data, fsm);
	struct hrtimer *timer = &otgd->timers[id].timer;

	hrtimer_cancel(timer);
}

/**
 * OTG FSM ops function to start/stop host
 */
static int usb_otg_start_host(struct otg_fsm *fsm, int on)
{
	struct otg_data *otgd = container_of(fsm, struct otg_data, fsm);
	struct otg_hcd_ops *hcd_ops;

	dev_dbg(otgd->dev, "otg: %s %d\n", __func__, on);
	if (!fsm->otg->host) {
		WARN_ONCE(1, "otg: fsm running without host\n");
		return 0;
	}

	if (on) {
		if (otgd->flags & OTG_FLAG_HOST_RUNNING)
			return 0;
		otgd->flags |= OTG_FLAG_HOST_RUNNING;

		/* OTG device operations */
		if (otgd->start_host)
			otgd->start_host(fsm, on);

		/* start host */
		hcd_ops = otgd->primary_hcd.ops;
		hcd_ops->add(otgd->primary_hcd.hcd, otgd->primary_hcd.irqnum,
			     otgd->primary_hcd.irqflags);
		if (otgd->shared_hcd.hcd) {
			hcd_ops = otgd->shared_hcd.ops;
			hcd_ops->add(otgd->shared_hcd.hcd,
				     otgd->shared_hcd.irqnum,
				     otgd->shared_hcd.irqflags);
		}
	} else {
		if (!(otgd->flags & OTG_FLAG_HOST_RUNNING))
			return 0;
		otgd->flags &= ~OTG_FLAG_HOST_RUNNING;

		/* stop host */
		if (otgd->shared_hcd.hcd) {
			hcd_ops = otgd->shared_hcd.ops;
			hcd_ops->remove(otgd->shared_hcd.hcd);
		}

		hcd_ops = otgd->primary_hcd.ops;
		hcd_ops->remove(otgd->primary_hcd.hcd);

		/* OTG device operations */
		if (otgd->start_host)
			otgd->start_host(fsm, on);
	}

	return 0;
}

/**
 * OTG FSM ops function to start/stop gadget
 */
static int usb_otg_start_gadget(struct otg_fsm *fsm, int on)
{
	struct otg_data *otgd = container_of(fsm, struct otg_data, fsm);
	struct usb_gadget *gadget = fsm->otg->gadget;

	dev_dbg(otgd->dev, "otg: %s %d\n", __func__, on);
	if (!gadget) {
		WARN_ONCE(1, "otg: fsm running without gadget\n");
		return 0;
	}

	if (on) {
		if (otgd->flags & OTG_FLAG_GADGET_RUNNING)
			return 0;

		/* OTG device operations */
		if (otgd->start_gadget)
			otgd->start_gadget(fsm, on);

		otgd->gadget_ops->start(fsm->otg->gadget);
		otgd->flags |= OTG_FLAG_GADGET_RUNNING;
	} else {
		if (!(otgd->flags & OTG_FLAG_GADGET_RUNNING))
			return 0;

		otgd->gadget_ops->stop(fsm->otg->gadget);

		/* OTG device operations */
		if (otgd->start_gadget)
			otgd->start_gadget(fsm, on);
		otgd->flags &= ~OTG_FLAG_GADGET_RUNNING;
	}

	return 0;
}

/* Change USB protocol when there is a protocol change */
static int drd_set_protocol(struct otg_fsm *fsm, int protocol)
{
	struct otg_data *otgd = container_of(fsm, struct otg_data, fsm);
	int ret = 0;

	if (fsm->protocol != protocol) {
		dev_dbg(otgd->dev, "otg: changing role fsm->protocol= %d; new protocol= %d\n",
			fsm->protocol, protocol);
		/* stop old protocol */
		if (fsm->protocol == PROTO_HOST)
			ret = otg_start_host(fsm, 0);
		else if (fsm->protocol == PROTO_GADGET)
			ret = otg_start_gadget(fsm, 0);
		if (ret)
			return ret;

		/* start new protocol */
		if (protocol == PROTO_HOST)
			ret = otg_start_host(fsm, 1);
		else if (protocol == PROTO_GADGET)
			ret = otg_start_gadget(fsm, 1);
		if (ret)
			return ret;

		fsm->protocol = protocol;
		return 0;
	}

	return 0;
}

/* Called when entering a DRD state */
static void drd_set_state(struct otg_fsm *fsm, enum usb_otg_state new_state)
{
	struct otg_data *otgd = container_of(fsm, struct otg_data, fsm);

	if (fsm->otg->state == new_state)
		return;

	fsm->state_changed = 1;
	dev_dbg(otgd->dev, "otg: set state: %s\n",
		usb_otg_state_string(new_state));
	switch (new_state) {
	case OTG_STATE_B_IDLE:
		drd_set_protocol(fsm, PROTO_UNDEF);
		break;
	case OTG_STATE_B_PERIPHERAL:
		drd_set_protocol(fsm, PROTO_GADGET);
		break;
	case OTG_STATE_A_HOST:
		drd_set_protocol(fsm, PROTO_HOST);
		break;
	case OTG_STATE_UNDEFINED:
	case OTG_STATE_B_SRP_INIT:
	case OTG_STATE_B_WAIT_ACON:
	case OTG_STATE_B_HOST:
	case OTG_STATE_A_IDLE:
	case OTG_STATE_A_WAIT_VRISE:
	case OTG_STATE_A_WAIT_BCON:
	case OTG_STATE_A_SUSPEND:
	case OTG_STATE_A_PERIPHERAL:
	case OTG_STATE_A_WAIT_VFALL:
	case OTG_STATE_A_VBUS_ERR:
	default:
		dev_warn(otgd->dev, "%s: otg: invalid state: %s\n",
			 __func__, usb_otg_state_string(new_state));
		break;
	}

	fsm->otg->state = new_state;
}

/**
 * DRD state change judgement
 *
 * For DRD we're only interested in some of the OTG states
 * i.e. OTG_STATE_B_IDLE: both peripheral and host are stopped
 *	OTG_STATE_B_PERIPHERAL: peripheral active
 *	OTG_STATE_A_HOST: host active
 * we're only interested in the following inputs
 *	fsm->id, fsm->vbus
 */
static int drd_statemachine(struct otg_fsm *fsm)
{
	struct otg_data *otgd = container_of(fsm, struct otg_data, fsm);
	enum usb_otg_state state;

	mutex_lock(&fsm->lock);

	fsm->state_changed = 0;
	state = fsm->otg->state;

	switch (state) {
	case OTG_STATE_UNDEFINED:
		if (!fsm->id)
			drd_set_state(fsm, OTG_STATE_A_HOST);
		else if (fsm->id && fsm->vbus)
			drd_set_state(fsm, OTG_STATE_B_PERIPHERAL);
		else
			drd_set_state(fsm, OTG_STATE_B_IDLE);
		break;
	case OTG_STATE_B_IDLE:
		if (!fsm->id)
			drd_set_state(fsm, OTG_STATE_A_HOST);
		else if (fsm->vbus)
			drd_set_state(fsm, OTG_STATE_B_PERIPHERAL);
		break;
	case OTG_STATE_B_PERIPHERAL:
		if (!fsm->id)
			drd_set_state(fsm, OTG_STATE_A_HOST);
		else if (!fsm->vbus)
			drd_set_state(fsm, OTG_STATE_B_IDLE);
		break;
	case OTG_STATE_A_HOST:
		if (fsm->id && fsm->vbus)
			drd_set_state(fsm, OTG_STATE_B_PERIPHERAL);
		else if (fsm->id && !fsm->vbus)
			drd_set_state(fsm, OTG_STATE_B_IDLE);
		break;

	/* invalid states for DRD */
	case OTG_STATE_B_SRP_INIT:
	case OTG_STATE_B_WAIT_ACON:
	case OTG_STATE_B_HOST:
	case OTG_STATE_A_IDLE:
	case OTG_STATE_A_WAIT_VRISE:
	case OTG_STATE_A_WAIT_BCON:
	case OTG_STATE_A_SUSPEND:
	case OTG_STATE_A_PERIPHERAL:
	case OTG_STATE_A_WAIT_VFALL:
	case OTG_STATE_A_VBUS_ERR:
		dev_err(otgd->dev, "%s: otg: invalid usb-drd state: %s\n",
			__func__, usb_otg_state_string(state));
		drd_set_state(fsm, OTG_STATE_UNDEFINED);
	break;
	}

	mutex_unlock(&fsm->lock);
	dev_dbg(otgd->dev, "otg: quit statemachine, changed %d\n",
		fsm->state_changed);

	return fsm->state_changed;
}

/**
 * OTG FSM/DRD work function
 */
static void usb_otg_work(struct work_struct *work)
{
	struct otg_data *otgd = container_of(work, struct otg_data, work);

	/* OTG state machine */
	if (!otgd->drd_only) {
		otg_statemachine(&otgd->fsm);
		return;
	}

	/* DRD state machine */
	drd_statemachine(&otgd->fsm);
}

/**
 * usb_otg_register() - Register the OTG device to OTG core
 * @parent_device:	parent device of Host & Gadget controllers.
 * @otg_fsm_ops:	otg state machine ops.
 * @drd_only:		dual-role only. no OTG features.
 *
 * Register parent device that contains both HCD and GCD into
 * the USB OTG core. HCD and GCD will be prevented from starting
 * till both are available for use.
 *
 * Return: struct otg_fsm * if success, NULL if error.
 */
struct otg_fsm *usb_otg_register(struct device *parent_dev,
				 struct otg_fsm_ops *fsm_ops,
				 bool drd_only)
{
	struct otg_data *otgd;
	int ret = 0;

	if (!parent_dev || !fsm_ops)
		return ERR_PTR(-EINVAL);

	/* already in list? */
	mutex_lock(&otg_list_mutex);
	if (usb_otg_device_get_otgd(parent_dev)) {
		dev_err(parent_dev, "otg: %s: device already in otg list\n",
			__func__);
		ret = -EINVAL;
		goto unlock;
	}

	/* allocate and add to list */
	otgd = kzalloc(sizeof(*otgd), GFP_KERNEL);
	if (!otgd) {
		ret = -ENOMEM;
		goto unlock;
	}

	otgd->dev = parent_dev;
	INIT_WORK(&otgd->work, usb_otg_work);
	otgd->wq = create_singlethread_workqueue("usb_otg");
	if (!otgd->wq) {
		dev_err(parent_dev, "otg: %s: can't create workqueue\n",
			__func__);
		ret = -ENODEV;
		goto err_wq;
	}

	otgd->drd_only = drd_only;
	/* For DRD mode we don't need OTG timers */
	if (!drd_only) {
		usb_otg_init_timers(otgd);

		/* FIXME: we ignore caller's timer ops */
		otgd->fsm_ops.add_timer = usb_otg_add_timer;
		otgd->fsm_ops.del_timer = usb_otg_del_timer;
	}

	/* save original start host/gadget ops */
	otgd->start_host = fsm_ops->start_host;
	otgd->start_gadget = fsm_ops->start_gadget;
	/* create copy of original ops */
	otgd->fsm_ops = *fsm_ops;
	/* override ops */
	otgd->fsm_ops.start_host = usb_otg_start_host;
	otgd->fsm_ops.start_gadget = usb_otg_start_gadget;
	/* set otg ops */
	otgd->fsm.ops = &otgd->fsm_ops;
	otgd->fsm.otg = &otgd->otg;

	mutex_init(&otgd->fsm.lock);

	list_add_tail(&otgd->list, &otg_list);
	mutex_unlock(&otg_list_mutex);
	return &otgd->fsm;

err_wq:
	kfree(otgd);
unlock:
	mutex_unlock(&otg_list_mutex);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(usb_otg_register);

/**
 * usb_otg_unregister() - Unregister the OTG device from USB OTG core
 * @parent_device:	parent device of Host & Gadget controllers.
 *
 * Unregister parent OTG deviced from USB OTG core.
 * Prevents unregistering till both Host and Gadget controllers
 * have unregistered from the OTG core.
 *
 * Return: 0 on success, error value otherwise.
 */
int usb_otg_unregister(struct device *parent_dev)
{
	struct otg_data *otgd;

	mutex_lock(&otg_list_mutex);
	otgd = usb_otg_device_get_otgd(parent_dev);
	if (!otgd) {
		dev_err(parent_dev, "otg: %s: device not in otg list\n",
			__func__);
		mutex_unlock(&otg_list_mutex);
		return -EINVAL;
	}

	/* prevent unregister till both host & gadget have unregistered */
	if (otgd->fsm.otg->host || otgd->fsm.otg->gadget) {
		dev_err(parent_dev, "otg: %s: host/gadget still registered\n",
			__func__);
		return -EBUSY;
	}

	/* OTG FSM is halted when host/gadget unregistered */
	destroy_workqueue(otgd->wq);

	/* remove from otg list */
	list_del(&otgd->list);
	kfree(otgd);
	mutex_unlock(&otg_list_mutex);

	return 0;
}
EXPORT_SYMBOL_GPL(usb_otg_unregister);

/**
 * start/kick the OTG FSM if we can
 * fsm->lock must be held
 */
static void usb_otg_start_fsm(struct otg_fsm *fsm)
{
	struct otg_data *otgd = container_of(fsm, struct otg_data, fsm);

	if (otgd->fsm_running)
		goto kick_fsm;

	if (!fsm->otg->host) {
		dev_info(otgd->dev, "otg: can't start till host registers\n");
		return;
	}

	if (!fsm->otg->gadget) {
		dev_info(otgd->dev, "otg: can't start till gadget registers\n");
		return;
	}

	otgd->fsm_running = true;
kick_fsm:
	queue_work(otgd->wq, &otgd->work);
}

/**
 * stop the OTG FSM. Stops Host & Gadget controllers as well.
 * fsm->lock must be held
 */
static void usb_otg_stop_fsm(struct otg_fsm *fsm)
{
	struct otg_data *otgd = container_of(fsm, struct otg_data, fsm);
	int i;

	if (!otgd->fsm_running)
		return;

	/* no more new events queued */
	otgd->fsm_running = false;

	/* Stop state machine / timers */
	if (!otgd->drd_only) {
		for (i = 0; i < ARRAY_SIZE(otgd->timers); i++)
			hrtimer_cancel(&otgd->timers[i].timer);
	}

	flush_workqueue(otgd->wq);
	fsm->otg->state = OTG_STATE_UNDEFINED;

	/* stop host/gadget immediately */
	if (fsm->protocol == PROTO_HOST)
		otg_start_host(fsm, 0);
	else if (fsm->protocol == PROTO_GADGET)
		otg_start_gadget(fsm, 0);
	fsm->protocol = PROTO_UNDEF;
}

/**
 * usb_otg_sync_inputs - Sync OTG inputs with the OTG state machine
 * @fsm:	OTG FSM instance
 *
 * Used by the OTG driver to update the inputs to the OTG
 * state machine.
 *
 * Can be called in IRQ context.
 */
void usb_otg_sync_inputs(struct otg_fsm *fsm)
{
	struct otg_data *otgd = container_of(fsm, struct otg_data, fsm);

	/* Don't kick FSM till it has started */
	if (!otgd->fsm_running)
		return;

	/* Kick FSM */
	queue_work(otgd->wq, &otgd->work);
}
EXPORT_SYMBOL_GPL(usb_otg_sync_inputs);

/**
 * usb_otg_kick_fsm - Kick the OTG state machine
 * @hcd_gcd_device:	Host/Gadget controller device
 *
 * Used by USB host/device stack to sync OTG related
 * events to the OTG state machine.
 * e.g. change in host_bus->b_hnp_enable, gadget->b_hnp_enable
 *
 * Returns: 0 on success, error value otherwise.
 */
int usb_otg_kick_fsm(struct device *hcd_gcd_device)
{
	struct otg_data *otgd;

	mutex_lock(&otg_list_mutex);
	otgd = usb_otg_device_get_otgd(hcd_gcd_device->parent);
	if (!otgd) {
		dev_dbg(hcd_gcd_device, "otg: %s: invalid host/gadget device\n",
			__func__);
		mutex_unlock(&otg_list_mutex);
		return -ENODEV;
	}

	mutex_unlock(&otg_list_mutex);
	usb_otg_sync_inputs(&otgd->fsm);

	return 0;
}
EXPORT_SYMBOL_GPL(usb_otg_kick_fsm);

/**
 * usb_otg_register_hcd - Register Host controller to OTG core
 * @hcd:	Host controller device
 * @irqnum:	interrupt number
 * @irqflags:	interrupt flags
 * @ops:	HCD ops to add/remove the HCD
 *
 * This is used by the USB Host stack to register the Host controller
 * to the OTG core. Host controller must not be started by the
 * caller as it is left upto the OTG state machine to do so.
 *
 * Returns: 0 on success, error value otherwise.
 */
int usb_otg_register_hcd(struct usb_hcd *hcd, unsigned int irqnum,
			 unsigned long irqflags, struct otg_hcd_ops *ops)
{
	struct otg_data *otgd;
	struct device *otg_dev = hcd->self.controller->parent;

	mutex_lock(&otg_list_mutex);
	otgd = usb_otg_device_get_otgd(otg_dev);
	if (!otgd) {
		dev_dbg(otg_dev, "otg: %s: device not registered to otg core\n",
			__func__);
		mutex_unlock(&otg_list_mutex);
		return -EINVAL;
	}

	mutex_unlock(&otg_list_mutex);
	/* HCD will be started by OTG fsm when needed */
	mutex_lock(&otgd->fsm.lock);
	if (otgd->primary_hcd.hcd) {
		/* probably a shared HCD ? */
		if (usb_otg_hcd_is_primary_hcd(hcd)) {
			dev_err(otg_dev, "otg: primary host already registered\n");
			goto err;
		}

		if (hcd->shared_hcd == otgd->primary_hcd.hcd) {
			if (otgd->shared_hcd.hcd) {
				dev_err(otg_dev, "otg: shared host already registered\n");
				goto err;
			}

			otgd->shared_hcd.hcd = hcd;
			otgd->shared_hcd.irqnum = irqnum;
			otgd->shared_hcd.irqflags = irqflags;
			otgd->shared_hcd.ops = ops;
			dev_info(otg_dev, "otg: shared host %s registered\n",
				 dev_name(hcd->self.controller));
		} else {
			dev_err(otg_dev, "otg: invalid shared host %s\n",
				dev_name(hcd->self.controller));
			goto err;
		}
	} else {
		if (!usb_otg_hcd_is_primary_hcd(hcd)) {
			dev_err(otg_dev, "otg: primary host must be registered first\n");
			goto err;
		}

		otgd->primary_hcd.hcd = hcd;
		otgd->primary_hcd.irqnum = irqnum;
		otgd->primary_hcd.irqflags = irqflags;
		otgd->primary_hcd.ops = ops;
		dev_info(otg_dev, "otg: primary host %s registered\n",
			 dev_name(hcd->self.controller));
	}

	/*
	 * we're ready only if we have shared HCD
	 * or we don't need shared HCD.
	 */
	if (otgd->shared_hcd.hcd || !otgd->primary_hcd.hcd->shared_hcd) {
		otgd->fsm.otg->host = hcd_to_bus(hcd);
		/* FIXME: set bus->otg_port if this is true OTG port with HNP */

		/* start FSM */
		usb_otg_start_fsm(&otgd->fsm);
	} else {
		dev_dbg(otg_dev, "otg: can't start till shared host registers\n");
	}

	mutex_unlock(&otgd->fsm.lock);

	return 0;

err:
	mutex_unlock(&otgd->fsm.lock);
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(usb_otg_register_hcd);

/**
 * usb_otg_unregister_hcd - Unregister Host controller from OTG core
 * @hcd:	Host controller device
 *
 * This is used by the USB Host stack to unregister the Host controller
 * from the OTG core. Ensures that Host controller is not running
 * on successful return.
 *
 * Returns: 0 on success, error value otherwise.
 */
int usb_otg_unregister_hcd(struct usb_hcd *hcd)
{
	struct otg_data *otgd;
	struct usb_bus *bus = hcd_to_bus(hcd);
	struct device *otg_dev = bus->controller->parent;

	mutex_lock(&otg_list_mutex);
	otgd = usb_otg_device_get_otgd(otg_dev);
	if (!otgd) {
		dev_err(otg_dev, "otg: %s: device not registered to otg core\n",
			__func__);
		mutex_unlock(&otg_list_mutex);
		return -EINVAL;
	}

	mutex_unlock(&otg_list_mutex);

	mutex_lock(&otgd->fsm.lock);
	if (hcd == otgd->primary_hcd.hcd) {
		otgd->primary_hcd.hcd = NULL;
		dev_info(otg_dev, "otg: primary host %s unregistered\n",
			 dev_name(bus->controller));
	} else if (hcd == otgd->shared_hcd.hcd) {
		otgd->shared_hcd.hcd = NULL;
		dev_info(otg_dev, "otg: shared host %s unregistered\n",
			 dev_name(bus->controller));
	} else {
		dev_err(otg_dev, "otg: host %s wasn't registered with otg\n",
			dev_name(bus->controller));
		mutex_unlock(&otgd->fsm.lock);
		return -EINVAL;
	}

	/* stop FSM & Host */
	usb_otg_stop_fsm(&otgd->fsm);
	otgd->fsm.otg->host = NULL;

	mutex_unlock(&otgd->fsm.lock);

	return 0;
}
EXPORT_SYMBOL_GPL(usb_otg_unregister_hcd);

/**
 * usb_otg_register_gadget - Register Gadget controller to OTG core
 * @gadget:	Gadget controller
 *
 * This is used by the USB Gadget stack to register the Gadget controller
 * to the OTG core. Gadget controller must not be started by the
 * caller as it is left upto the OTG state machine to do so.
 *
 * Gadget core must call this only when all resources required for
 * gadget controller to run are available.
 * i.e. gadget function driver is available.
 *
 * Returns: 0 on success, error value otherwise.
 */
int usb_otg_register_gadget(struct usb_gadget *gadget,
			    struct otg_gadget_ops *ops)
{
	struct otg_data *otgd;
	struct device *otg_dev = gadget->dev.parent;

	mutex_lock(&otg_list_mutex);
	otgd = usb_otg_device_get_otgd(otg_dev);
	if (!otgd) {
		dev_dbg(otg_dev, "otg: %s: device not registered to otg core\n",
			__func__);
		mutex_unlock(&otg_list_mutex);
		return -EINVAL;
	}

	mutex_unlock(&otg_list_mutex);

	mutex_lock(&otgd->fsm.lock);
	if (otgd->fsm.otg->gadget) {
		dev_err(otg_dev, "otg: gadget already registered with otg\n");
		mutex_unlock(&otgd->fsm.lock);
		return -EINVAL;
	}

	otgd->fsm.otg->gadget = gadget;
	otgd->gadget_ops = ops;
	dev_info(otg_dev, "otg: gadget %s registered\n",
		 dev_name(&gadget->dev));

	/* start FSM */
	usb_otg_start_fsm(&otgd->fsm);
	mutex_unlock(&otgd->fsm.lock);

	return 0;
}
EXPORT_SYMBOL_GPL(usb_otg_register_gadget);

/**
 * usb_otg_unregister_gadget - Unregister Gadget controller from OTG core
 * @gadget:	Gadget controller
 *
 * This is used by the USB Gadget stack to unregister the Gadget controller
 * from the OTG core. Ensures that Gadget controller is not running
 * on successful return.
 *
 * Returns: 0 on success, error value otherwise.
 */
int usb_otg_unregister_gadget(struct usb_gadget *gadget)
{
	struct otg_data *otgd;
	struct device *otg_dev = gadget->dev.parent;

	mutex_lock(&otg_list_mutex);
	otgd = usb_otg_device_get_otgd(otg_dev);
	if (!otgd) {
		dev_dbg(otg_dev, "otg: %s: device not registered to otg core\n",
			__func__);
		mutex_unlock(&otg_list_mutex);
		return -EINVAL;
	}

	mutex_unlock(&otg_list_mutex);

	mutex_lock(&otgd->fsm.lock);
	if (otgd->fsm.otg->gadget != gadget) {
		dev_err(otg_dev, "otg: gadget %s wasn't registered with otg\n",
			dev_name(&gadget->dev));
		mutex_unlock(&otgd->fsm.lock);
		return -EINVAL;
	}

	/* Stop FSM & gadget */
	usb_otg_stop_fsm(&otgd->fsm);
	otgd->fsm.otg->gadget = NULL;
	mutex_unlock(&otgd->fsm.lock);

	dev_info(otg_dev, "otg: gadget %s unregistered\n",
		 dev_name(&gadget->dev));

	return 0;
}
EXPORT_SYMBOL_GPL(usb_otg_unregister_gadget);

/**
 * usb_otg_fsm_to_dev - Get OTG controller device from struct otg_fsm
 * @fsm:	otg_fsm data structure
 *
 * This is used by the OTG controller driver to get it's device node
 * from any of the otg_fsm->ops.
 */
struct device *usb_otg_fsm_to_dev(struct otg_fsm *fsm)
{
	struct otg_data *otgd = container_of(fsm, struct otg_data, fsm);

	return otgd->dev;
}
EXPORT_SYMBOL_GPL(usb_otg_fsm_to_dev);
