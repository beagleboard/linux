/*
 * OMAP Remote Processor driver
 *
 * Copyright (C) 2011-2015 Texas Instruments, Inc.
 * Copyright (C) 2011 Google, Inc.
 *
 * Ohad Ben-Cohen <ohad@wizery.com>
 * Brian Swetland <swetland@google.com>
 * Fernando Guzman Lugo <fernando.lugo@ti.com>
 * Mark Grosen <mgrosen@ti.com>
 * Suman Anna <s-anna@ti.com>
 * Hari Kanigeri <h-kanigeri2@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/remoteproc.h>
#include <linux/mailbox_client.h>
#include <linux/omap-mailbox.h>
#include <linux/omap-iommu.h>
#include <linux/pm_runtime.h>

#include <linux/platform_data/remoteproc-omap.h>

#include "omap_remoteproc.h"
#include "remoteproc_internal.h"

/* default auto-suspend delay (ms) */
#define DEFAULT_AUTOSUSPEND_DELAY 10000

/**
 * struct omap_rproc_timers_info - timers for the omap rproc
 * @odt: timer pointer
 * @irq: timer irq
 */
struct omap_rproc_timers_info {
	struct omap_dm_timer *odt;
	int irq;
};

/**
 * struct omap_rproc - omap remote processor state
 * @mbox: mailbox channel handle
 * @client: mailbox client to request the mailbox channel
 * @num_timers: number of rproc timer(s)
 * @num_wd_timers: number of rproc watchdog timers
 * @timers: timer(s) info used by rproc
 * @autosuspend_delay: auto-suspend delay value to be used for runtime pm
 * @need_resume: if true a resume is needed in the system resume callback
 * @rproc: rproc handle
 * @pm_comp: completion primitive to sync for suspend response
 * @standby_addr: kernel address of the register having module standby status
 * @suspend_acked: state machine flag to store the suspend request ack
 */
struct omap_rproc {
	struct mbox_chan *mbox;
	struct mbox_client client;
	int num_timers;
	int num_wd_timers;
	struct omap_rproc_timers_info *timers;
	int autosuspend_delay;
	bool need_resume;
	struct rproc *rproc;
	struct completion pm_comp;
	void __iomem *standby_addr;
	bool suspend_acked;
};

/**
 * struct omap_rproc_fw_data - firmware data for the omap remote processor
 * @device_name: device name of the remote processor
 * @fw_name: firmware name to use
 * @autosuspend_delay: custom auto-suspend delay value in milliseconds
 */
struct omap_rproc_fw_data {
	const char *device_name;
	const char *fw_name;
	int autosuspend_delay;
};

/**
 * omap_rproc_watchdog_isr - Watchdog ISR handler for remoteproc device
 * @irq: IRQ number associated with a watchdog timer
 * @data: IRQ handler data
 *
 * This ISR routine executes the required necessary low-level code to
 * acknowledge a watchdog timer interrupt. There can be multiple watchdog
 * timers associated with a rproc (like IPUs which have 2 watchdog timers,
 * one per Cortex M3/M4 core), so a lookup has to be performed to identify
 * the timer to acknowledge its interrupt.
 *
 * The function also invokes rproc_report_crash to report the watchdog event
 * to the remoteproc driver core, to trigger a recovery.
 *
 * Return: IRQ_HANDLED or IRQ_NONE
 */
static irqreturn_t omap_rproc_watchdog_isr(int irq, void *data)
{
	struct platform_device *pdev = data;
	struct rproc *rproc = platform_get_drvdata(pdev);
	struct omap_rproc *oproc = rproc->priv;
	struct device *dev = &pdev->dev;
	struct omap_rproc_pdata *pdata = dev->platform_data;
	struct omap_rproc_timer_ops *timer_ops = pdata->timer_ops;
	struct omap_rproc_timers_info *timers = oproc->timers;
	struct omap_dm_timer *timer = NULL;
	int num_timers = oproc->num_timers + oproc->num_wd_timers;
	int i;

	for (i = oproc->num_timers; i < num_timers; i++) {
		if (timers[i].irq > 0 && irq == timers[i].irq) {
			timer = timers[i].odt;
			break;
		}
	}

	if (!timer) {
		dev_err(dev, "invalid timer\n");
		return IRQ_NONE;
	}

	timer_ops->ack_timer_irq(timer);

	rproc_report_crash(rproc, RPROC_WATCHDOG);

	return IRQ_HANDLED;
}

/**
 * omap_rproc_enable_timers - enable the timers for a remoteproc
 * @pdev - the remoteproc platform device
 * @configure - boolean flag used to acquire and configure the timer handle
 *
 * This function is used primarily to enable the timers associated with
 * a remoteproc. The configure flag is provided to allow the remoteproc
 * driver core to either acquire and start a timer (during device
 * initialization) or to just start a timer (during a resume operation).
 */
static int
omap_rproc_enable_timers(struct platform_device *pdev, bool configure)
{
	int i;
	int ret = 0;
	struct rproc *rproc = platform_get_drvdata(pdev);
	struct omap_rproc *oproc = rproc->priv;
	struct omap_rproc_pdata *pdata = pdev->dev.platform_data;
	struct omap_rproc_timer_ops *timer_ops = pdata->timer_ops;
	struct omap_rproc_timers_info *timers = oproc->timers;
	struct device *dev = &pdev->dev;
	struct device_node *np = NULL;
	int num_timers = oproc->num_timers + oproc->num_wd_timers;

	if (num_timers <= 0)
		return 0;

	if (!configure)
		goto start_timers;

	for (i = 0; i < num_timers; i++) {
		if (i < oproc->num_timers)
			np = of_parse_phandle(dev->of_node, "timers", i);
		else
			np = of_parse_phandle(dev->of_node, "watchdog-timers",
					      (i - oproc->num_timers));
		if (!np) {
			ret = -ENXIO;
			dev_err(dev, "device node lookup for timer at index %d failed: %d\n",
				i < oproc->num_timers ? i :
				i - oproc->num_timers, ret);
			goto free_timers;
		}

		timers[i].irq = -1;
		timers[i].odt = timer_ops->request_timer(np);
		of_node_put(np);
		if (IS_ERR(timers[i].odt)) {
			dev_err(dev, "request for timer %p failed: %ld\n", np,
				PTR_ERR(timers[i].odt));
			ret = -EBUSY;
			goto free_timers;
		}

		if (i >= oproc->num_timers) {
			timers[i].irq = timer_ops->get_timer_irq(timers[i].odt);
			if (timers[i].irq < 0) {
				dev_err(dev, "get_irq for timer %p failed: %d\n",
					np, timers[i].irq);
				ret = -EBUSY;
				goto free_timers;
			}

			ret = request_irq(timers[i].irq,
					  omap_rproc_watchdog_isr, IRQF_SHARED,
					  "rproc-wdt", pdev);
			if (ret) {
				dev_err(&pdev->dev, "error requesting irq for timer %p\n",
					np);
				timer_ops->release_timer(timers[i].odt);
				timers[i].odt = NULL;
				timers[i].irq = -1;
				goto free_timers;
			}
		}
	}

start_timers:
	for (i = 0; i < num_timers; i++)
		timer_ops->start_timer(timers[i].odt);
	return 0;

free_timers:
	while (i--) {
		if (i >= oproc->num_timers)
			free_irq(timers[i].irq, pdev);
		timer_ops->release_timer(timers[i].odt);
		timers[i].odt = NULL;
		timers[i].irq = -1;
	}

	return ret;
}

/**
 * omap_rproc_disable_timers - disable the timers for a remoteproc
 * @pdev - the remoteproc platform device
 * @configure - boolean flag used to release the timer handle
 *
 * This function is used primarily to disable the timers associated with
 * a remoteproc. The configure flag is provided to allow the remoteproc
 * driver core to either stop and release a timer (during device shutdown)
 * or to just stop a timer (during a suspend operation).
 */
static int
omap_rproc_disable_timers(struct platform_device *pdev, bool configure)
{
	int i;
	struct rproc *rproc = platform_get_drvdata(pdev);
	struct omap_rproc *oproc = rproc->priv;
	struct omap_rproc_pdata *pdata = pdev->dev.platform_data;
	struct omap_rproc_timer_ops *timer_ops = pdata->timer_ops;
	struct omap_rproc_timers_info *timers = oproc->timers;
	int num_timers = oproc->num_timers + oproc->num_wd_timers;

	if (num_timers <= 0)
		return 0;

	for (i = 0; i < num_timers; i++) {
		timer_ops->stop_timer(timers[i].odt);
		if (configure) {
			if (i >= oproc->num_timers)
				free_irq(timers[i].irq, pdev);
			timer_ops->release_timer(timers[i].odt);
			timers[i].odt = NULL;
			timers[i].irq = -1;
		}
	}

	return 0;
}

/**
 * omap_rproc_mbox_callback() - inbound mailbox message handler
 * @client: mailbox client pointer used for requesting the mailbox channel
 * @data: mailbox payload
 *
 * This handler is invoked by omap's mailbox driver whenever a mailbox
 * message is received. Usually, the mailbox payload simply contains
 * the index of the virtqueue that is kicked by the remote processor,
 * and we let remoteproc core handle it.
 *
 * In addition to virtqueue indices, we also have some out-of-band values
 * that indicates different events. Those values are deliberately very
 * big so they don't coincide with virtqueue indices.
 */
static void omap_rproc_mbox_callback(struct mbox_client *client, void *data)
{
	struct omap_rproc *oproc = container_of(client, struct omap_rproc,
						client);
	struct device *dev = oproc->rproc->dev.parent;
	const char *name = oproc->rproc->name;
	u32 msg = (u32)data;

	dev_dbg(dev, "mbox msg: 0x%x\n", msg);

	switch (msg) {
	case RP_MBOX_CRASH:
		/*
		 * remoteproc detected an exception, notify the rproc core.
		 * The remoteproc core will handle the recovery.
		 */
		dev_err(dev, "omap rproc %s crashed\n", name);
		rproc_report_crash(oproc->rproc, RPROC_EXCEPTION);
		break;
	case RP_MBOX_ECHO_REPLY:
		dev_info(dev, "received echo reply from %s\n", name);
		break;
	case RP_MBOX_SUSPEND_ACK:
	case RP_MBOX_SUSPEND_CANCEL:
		oproc->suspend_acked = msg == RP_MBOX_SUSPEND_ACK;
		complete(&oproc->pm_comp);
		break;
	default:
		if (msg >= RP_MBOX_END_MSG) {
			dev_err(dev, "dropping unknown message %x", msg);
			return;
		}
		/* msg contains the index of the triggered vring */
		if (rproc_vq_interrupt(oproc->rproc, msg) == IRQ_NONE)
			dev_dbg(dev, "no message was found in vqid %d\n", msg);
	}
}

/* kick a virtqueue */
static void omap_rproc_kick(struct rproc *rproc, int vqid)
{
	struct omap_rproc *oproc = rproc->priv;
	struct device *dev = rproc->dev.parent;
	int ret;

	/* wake up the rproc before kicking it */
	ret = pm_runtime_get_sync(dev);
	if (WARN_ON(ret < 0)) {
		dev_err(dev, "pm_runtime_get_sync() failed during kick, ret = %d\n",
			ret);
		pm_runtime_put_noidle(dev);
		return;
	}

	/* send the index of the triggered virtqueue in the mailbox payload */
	ret = mbox_send_message(oproc->mbox, (void *)vqid);
	if (ret < 0)
		dev_err(dev, "mbox_send_message failed: %d\n", ret);

	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);
}

/*
 * Power up the remote processor.
 *
 * This function will be invoked only after the firmware for this rproc
 * was loaded, parsed successfully, and all of its resource requirements
 * were met.
 */
static int omap_rproc_start(struct rproc *rproc)
{
	struct omap_rproc *oproc = rproc->priv;
	struct device *dev = rproc->dev.parent;
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_rproc_pdata *pdata = pdev->dev.platform_data;
	int ret;
	struct mbox_client *client = &oproc->client;

	if (pdata->set_bootaddr)
		pdata->set_bootaddr(rproc->bootaddr);

	client->dev = dev;
	client->tx_done = NULL;
	client->rx_callback = omap_rproc_mbox_callback;
	client->tx_block = false;
	client->knows_txdone = false;

	oproc->mbox = mbox_request_channel(client, 0);
	if (IS_ERR(oproc->mbox)) {
		ret = -EBUSY;
		dev_err(dev, "mbox_request_channel failed: %ld\n",
			PTR_ERR(oproc->mbox));
		return ret;
	}

	/*
	 * Ping the remote processor. this is only for sanity-sake;
	 * there is no functional effect whatsoever.
	 *
	 * Note that the reply will _not_ arrive immediately: this message
	 * will wait in the mailbox fifo until the remote processor is booted.
	 */
	ret = mbox_send_message(oproc->mbox, (void *)RP_MBOX_ECHO_REQUEST);
	if (ret < 0) {
		dev_err(dev, "mbox_send_message failed: %d\n", ret);
		goto put_mbox;
	}

	ret = omap_rproc_enable_timers(pdev, true);
	if (ret) {
		dev_err(dev, "omap_rproc_enable_timers failed: %d\n", ret);
		goto put_mbox;
	}

	ret = pdata->device_enable(pdev);
	if (ret) {
		dev_err(dev, "omap_device_enable failed: %d\n", ret);
		goto reset_timers;
	}

	/*
	 * remote processor is up, so update the runtime pm status and
	 * enable the auto-suspend. The device usage count is incremented
	 * manually for balancing it for auto-suspend
	 */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_get_noresume(dev);
	pm_runtime_set_autosuspend_delay(dev, oproc->autosuspend_delay);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return 0;

reset_timers:
	omap_rproc_disable_timers(pdev, true);
put_mbox:
	mbox_free_channel(oproc->mbox);
	return ret;
}

/* power off the remote processor */
static int omap_rproc_stop(struct rproc *rproc)
{
	struct device *dev = rproc->dev.parent;
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_rproc_pdata *pdata = pdev->dev.platform_data;
	struct omap_rproc *oproc = rproc->priv;
	int ret;

	/*
	 * cancel any possible scheduled runtime suspend by incrementing
	 * the device usage count, and resuming the device. The remoteproc
	 * also needs to be woken up if suspended, to avoid the remoteproc
	 * OS to continue to remember any context that it has saved, and
	 * avoid potential issues in misindentifying a subsequent device
	 * reboot as a power restore boot
	 */
	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		pm_runtime_put_noidle(dev);
		return ret;
	}

	ret = pdata->device_shutdown(pdev);
	if (ret)
		goto out;

	ret = omap_rproc_disable_timers(pdev, true);
	if (ret)
		goto enable_device;

	mbox_free_channel(oproc->mbox);

	/*
	 * update the runtime pm states and status now that the remoteproc
	 * has stopped
	 */
	pm_runtime_dont_use_autosuspend(dev);
	pm_runtime_put_noidle(dev);
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);

	return 0;

enable_device:
	pdata->device_enable(pdev);
out:
	/* schedule the next auto-suspend */
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);
	return ret;

}

static struct rproc_ops omap_rproc_ops = {
	.start		= omap_rproc_start,
	.stop		= omap_rproc_stop,
	.kick		= omap_rproc_kick,
};

#ifdef CONFIG_PM
static bool _is_rproc_in_standby(struct omap_rproc *oproc)
{
	static int standby_mask = (1 << 18);

	return readl(oproc->standby_addr) & standby_mask;
}

/* 1 sec is long enough time to let the remoteproc side suspend the device */
#define DEF_SUSPEND_TIMEOUT 1000
static int _suspend(struct rproc *rproc, bool auto_suspend)
{
	struct device *dev = rproc->dev.parent;
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_rproc_pdata *pdata = dev_get_platdata(dev);
	struct omap_rproc *oproc = rproc->priv;
	unsigned long to = msecs_to_jiffies(DEF_SUSPEND_TIMEOUT);
	unsigned long ta = jiffies + to;
	u32 suspend_msg = auto_suspend ?
				RP_MBOX_SUSPEND_AUTO : RP_MBOX_SUSPEND_SYSTEM;
	int ret;

	reinit_completion(&oproc->pm_comp);
	oproc->suspend_acked = false;
	ret = mbox_send_message(oproc->mbox, (void *)suspend_msg);
	if (ret < 0) {
		dev_err(dev, "PM mbox_send_message failed: %d\n", ret);
		return ret;
	}

	ret = wait_for_completion_timeout(&oproc->pm_comp, to);
	if (!oproc->suspend_acked)
		return -EBUSY;

	/*
	 * The remoteproc side is returning the ACK message before saving the
	 * context, because the context saving is performed within a SYS/BIOS
	 * function, and it cannot have any inter-dependencies against the IPC
	 * layer. Also, as the SYS/BIOS needs to preserve properly the processor
	 * register set, sending this ACK or signalling the completion of the
	 * context save through a shared memory variable can never be the
	 * absolute last thing to be executed on the remoteproc side, and the
	 * MPU cannot use the ACK message as a sync point to put the remoteproc
	 * into reset. The only way to ensure that the remote processor has
	 * completed saving the context is to check that the  module has reached
	 * STANDBY state (after saving the context, the SYS/BIOS executes the
	 * appropriate target-specific WFI instruction causing the module to
	 * enter STANDBY).
	 */
	while (!_is_rproc_in_standby(oproc)) {
		if (time_after(jiffies, ta))
			return -ETIME;
		schedule();
	}

	ret = pdata->device_shutdown(pdev);
	if (ret)
		return ret;

	ret = omap_rproc_disable_timers(pdev, false);
	if (ret) {
		dev_err(dev, "disabling timers during suspend failed %d\n",
			ret);
		goto enable_device;
	}

	ret = omap_iommu_domain_suspend(rproc->domain, auto_suspend);
	if (ret) {
		dev_err(dev, "iommu domain suspend failed %d\n", ret);
		goto enable_timers;
	}

	return 0;

enable_timers:
	/* ignore errors on re-enabling code */
	omap_rproc_enable_timers(pdev, false);
enable_device:
	pdata->device_enable(pdev);
	return ret;
}

static int _omap_rproc_suspend(struct rproc *rproc, bool auto_suspend)
{
	struct omap_rproc *oproc = rproc->priv;

	/*
	 * do not even attempt suspend if the remote processor is not
	 * idled for auto-suspend
	 */
	if (auto_suspend && !_is_rproc_in_standby(oproc))
		return -EBUSY;

	return _suspend(rproc, auto_suspend);
}

static int _omap_rproc_resume(struct rproc *rproc, bool auto_suspend)
{
	struct device *dev = rproc->dev.parent;
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_rproc_pdata *pdata = dev_get_platdata(dev);
	int ret;

	ret = omap_iommu_domain_resume(rproc->domain, auto_suspend);
	if (ret) {
		dev_err(dev, "omap_iommu resume failed %d\n", ret);
		goto out;
	}

	/* boot address could be lost after suspend, so restore it */
	if (pdata->set_bootaddr)
		pdata->set_bootaddr(rproc->bootaddr);

	ret = omap_rproc_enable_timers(pdev, false);
	if (ret) {
		dev_err(dev, "enabling timers during resume failed %d\n",
			ret);
		goto suspend_iommu;
	}

	ret = pdata->device_enable(pdev);
	if (ret)
		goto disable_timers;

	return 0;

disable_timers:
	omap_rproc_disable_timers(pdev, false);
suspend_iommu:
	omap_iommu_domain_suspend(rproc->domain, auto_suspend);
out:
	return ret;
}

static int omap_rproc_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rproc *rproc = platform_get_drvdata(pdev);
	struct omap_rproc *oproc = rproc->priv;
	int ret = 0;

	mutex_lock(&rproc->lock);
	if (rproc->state == RPROC_OFFLINE)
		goto out;

	if (rproc->state == RPROC_SUSPENDED)
		goto out;

	if (rproc->state != RPROC_RUNNING) {
		ret = -EBUSY;
		goto out;
	}

	ret = _omap_rproc_suspend(rproc, false);
	if (ret) {
		dev_err(dev, "suspend failed %d\n", ret);
		goto out;
	}

	/*
	 * remoteproc is running at the time of system suspend, so remember
	 * it so as to wake it up during system resume
	 */
	oproc->need_resume = 1;
	rproc->state = RPROC_SUSPENDED;

	/*
	 * update the runtime pm status to be suspended, without decrementing
	 * the device usage count
	 */
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	pm_runtime_enable(dev);
out:
	mutex_unlock(&rproc->lock);
	return ret;
}

static int omap_rproc_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rproc *rproc = platform_get_drvdata(pdev);
	struct omap_rproc *oproc = rproc->priv;
	int ret = 0;

	mutex_lock(&rproc->lock);
	if (rproc->state == RPROC_OFFLINE)
		goto out;

	if (rproc->state != RPROC_SUSPENDED) {
		ret = -EBUSY;
		goto out;
	}

	/*
	 * remoteproc was auto-suspended at the time of system suspend,
	 * so no need to wake-up the processor (leave it in suspended
	 * state, will be woken up during a subsequent runtime_resume)
	 */
	if (!oproc->need_resume)
		goto out;

	ret = _omap_rproc_resume(rproc, false);
	if (ret) {
		dev_err(dev, "resume failed %d\n", ret);
		goto out;
	}
	oproc->need_resume = false;

	rproc->state = RPROC_RUNNING;

	/*
	 * update the runtime pm status to be active, without incrementing
	 * the device usage count
	 */
	pm_runtime_disable(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_mark_last_busy(dev);
out:
	mutex_unlock(&rproc->lock);
	return ret;
}

static int omap_rproc_runtime_suspend(struct device *dev)
{
	struct rproc *rproc = dev_get_drvdata(dev);
	int ret;

	if (WARN_ON(rproc->state != RPROC_RUNNING)) {
		dev_err(dev, "rproc cannot be runtime suspended when not running!\n");
		return -EBUSY;
	}

	ret = _omap_rproc_suspend(rproc, true);
	if (ret)
		goto abort;

	rproc->state = RPROC_SUSPENDED;
	return 0;

abort:
	pm_runtime_mark_last_busy(dev);
	return ret;
}

static int omap_rproc_runtime_resume(struct device *dev)
{
	struct rproc *rproc = dev_get_drvdata(dev);
	int ret;

	if (WARN_ON(rproc->state != RPROC_SUSPENDED)) {
		dev_err(dev, "rproc cannot be runtime resumed if not suspended!\n");
		return -EBUSY;
	}

	ret = _omap_rproc_resume(rproc, true);
	if (ret) {
		dev_err(dev, "runtime resume failed %d\n", ret);
		return ret;
	}

	rproc->state = RPROC_RUNNING;
	return 0;
}
#endif /* CONFIG_PM */

static const struct omap_rproc_fw_data omap4_dsp_fw_data = {
	.device_name	= "dsp",
	.fw_name	= "tesla-dsp.xe64T",
};

static const struct omap_rproc_fw_data omap4_ipu_fw_data = {
	.device_name	= "ipu",
	.fw_name	= "ducati-m3-core0.xem3",
};

static const struct omap_rproc_fw_data dra7_rproc_fw_data[] = {
	{
		.device_name	= "40800000.dsp",
		.fw_name	= "dra7-dsp1-fw.xe66",
	},
	{
		.device_name	= "41000000.dsp",
		.fw_name	= "dra7-dsp2-fw.xe66",
	},
	{
		.device_name	= "55020000.ipu",
		.fw_name	= "dra7-ipu2-fw.xem4",
	},
	{
		.device_name	= "58820000.ipu",
		.fw_name	= "dra7-ipu1-fw.xem4",
	},
	{
		/* sentinel */
	},
};

static const struct of_device_id omap_rproc_of_match[] = {
	{
		.compatible     = "ti,omap4-rproc-dsp",
		.data           = &omap4_dsp_fw_data,
	},
	{
		.compatible     = "ti,omap4-rproc-ipu",
		.data           = &omap4_ipu_fw_data,
	},
	{
		.compatible     = "ti,omap5-rproc-dsp",
		.data           = &omap4_dsp_fw_data,
	},
	{
		.compatible     = "ti,omap5-rproc-ipu",
		.data           = &omap4_ipu_fw_data,
	},
	{
		.compatible     = "ti,dra7-rproc-dsp",
		.data           = dra7_rproc_fw_data,
	},
	{
		.compatible     = "ti,dra7-rproc-ipu",
		.data           = dra7_rproc_fw_data,
	},
	{
		/* end */
	},
};
MODULE_DEVICE_TABLE(of, omap_rproc_of_match);

static int omap_rproc_get_autosuspend_delay(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct omap_rproc_fw_data *data;
	const struct of_device_id *match;
	int delay = -EINVAL;

	match = of_match_device(omap_rproc_of_match, &pdev->dev);
	if (!match)
		return -ENODEV;

	data = match->data;

	if (!of_device_is_compatible(np, "ti,dra7-rproc-dsp") &&
	    !of_device_is_compatible(np, "ti,dra7-rproc-ipu")) {
		delay = data->autosuspend_delay;
		goto out;
	}

	for (; data && data->device_name; data++) {
		if (!strcmp(dev_name(&pdev->dev), data->device_name)) {
			delay = data->autosuspend_delay;
			break;
		}
	}

out:
	return (delay > 0) ? delay : DEFAULT_AUTOSUSPEND_DELAY;
}

static const char *omap_rproc_get_firmware(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct omap_rproc_fw_data *data;
	const struct of_device_id *match;

	match = of_match_device(omap_rproc_of_match, &pdev->dev);
	if (!match)
		return ERR_PTR(-ENODEV);

	data = match->data;

	if (!of_device_is_compatible(np, "ti,dra7-rproc-dsp") &&
	    !of_device_is_compatible(np, "ti,dra7-rproc-ipu"))
		return data->fw_name;

	for (; data && data->device_name; data++) {
		if (!strcmp(dev_name(&pdev->dev), data->device_name))
			return data->fw_name;
	}

	return NULL;
}

static int omap_rproc_probe(struct platform_device *pdev)
{
	struct omap_rproc_pdata *pdata = pdev->dev.platform_data;
	struct device_node *np = pdev->dev.of_node;
	struct omap_rproc_timer_ops *timer_ops;
	struct omap_rproc *oproc;
	struct rproc *rproc;
	const char *firmware;
	u32 standby_addr = 0;
	int num_timers;
	int ret;

	if (!np) {
		dev_err(&pdev->dev, "only DT-based devices are supported\n");
		return -ENODEV;
	}

	/*
	 * self-manage the ordering dependencies between omap_device_enable/idle
	 * and omap_device_assert/deassert_hardreset API during runtime suspend
	 * and resume, rather than relying on the order in omap_device layer.
	 */
	if (pdev->dev.pm_domain) {
		dev_dbg(&pdev->dev, "device pm_domain is being reset for this remoteproc device\n");
		pdev->dev.pm_domain = NULL;
	}

	if (!pdata || !pdata->device_enable || !pdata->device_shutdown) {
		dev_err(&pdev->dev, "platform data is either missing or incomplete\n");
		return -ENODEV;
	}

	firmware = omap_rproc_get_firmware(pdev);
	if (IS_ERR(firmware))
		return PTR_ERR(firmware);

	ret = dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&pdev->dev, "dma_set_coherent_mask: %d\n", ret);
		return ret;
	}

	rproc = rproc_alloc(&pdev->dev, dev_name(&pdev->dev), &omap_rproc_ops,
			    firmware, sizeof(*oproc));
	if (!rproc)
		return -ENOMEM;

	oproc = rproc->priv;
	oproc->rproc = rproc;
	/*
	 * All existing OMAP IPU and DSP processors do have an MMU, and
	 * are expected to use MMU, so this statement suffices.
	 * XXX: Replace this logic if and when a need arises.
	 */
	rproc->has_iommu = true;

	timer_ops = pdata->timer_ops;
	/*
	 * Timer nodes are directly used in client nodes as phandles, so
	 * retrieve the count using NULL as cells-name.
	 * XXX: Use the much simpler of_property_count_elems_of_size
	 * if available
	 */
	oproc->num_timers = of_count_phandle_with_args(np, "timers", NULL);
	if (oproc->num_timers <= 0) {
		dev_dbg(&pdev->dev, "device does not have timers, status = %d\n",
			oproc->num_timers);
		oproc->num_timers = 0;
	} else {
		if (!timer_ops || !timer_ops->request_timer ||
		    !timer_ops->release_timer || !timer_ops->start_timer ||
		    !timer_ops->stop_timer) {
			dev_err(&pdev->dev, "device does not have required timer ops\n");
			ret = -ENODEV;
			goto free_rproc;
		}
	}

#ifdef CONFIG_OMAP_REMOTEPROC_WATCHDOG
	oproc->num_wd_timers = of_count_phandle_with_args(np, "watchdog-timers",
							  NULL);
	if (oproc->num_wd_timers <= 0) {
		dev_dbg(&pdev->dev, "device does not have watchdog timers, status = %d\n",
			oproc->num_wd_timers);
		oproc->num_wd_timers = 0;
	} else {
		if (!timer_ops || !timer_ops->get_timer_irq ||
		    !timer_ops->ack_timer_irq) {
			dev_err(&pdev->dev, "device does not have required watchdog timer ops\n");
			ret = -ENODEV;
			goto free_rproc;
		}
	}
#endif

	if (oproc->num_timers || oproc->num_wd_timers) {
		num_timers = oproc->num_timers + oproc->num_wd_timers;
		oproc->timers = devm_kzalloc(&pdev->dev, sizeof(*oproc->timers)
					     * num_timers, GFP_KERNEL);
		if (!oproc->timers) {
			ret = -ENOMEM;
			goto free_rproc;
		}

		dev_dbg(&pdev->dev, "device has %d tick timers and %d watchdog timers\n",
			oproc->num_timers, oproc->num_wd_timers);
	}

	init_completion(&oproc->pm_comp);
	oproc->autosuspend_delay = omap_rproc_get_autosuspend_delay(pdev);
	if (oproc->autosuspend_delay < 0)
		goto free_rproc;

	ret = of_property_read_u32(np, "ti,rproc-standby-info", &standby_addr);
	if (ret || !standby_addr)
		goto free_rproc;

	oproc->standby_addr = devm_ioremap(&pdev->dev, standby_addr,
					   sizeof(u32));
	if (!oproc->standby_addr)
		goto free_rproc;

	platform_set_drvdata(pdev, rproc);

	ret = rproc_add(rproc);
	if (ret)
		goto free_rproc;

	if (rproc_get_alias_id(rproc) < 0)
		dev_warn(&pdev->dev, "device does not have an alias id\n");

	return 0;

free_rproc:
	rproc_put(rproc);
	return ret;
}

static int omap_rproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);

	rproc_del(rproc);
	rproc_put(rproc);

	return 0;
}

static const struct dev_pm_ops omap_rproc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(omap_rproc_suspend, omap_rproc_resume)
	SET_RUNTIME_PM_OPS(omap_rproc_runtime_suspend,
			   omap_rproc_runtime_resume, NULL)
};

static struct platform_driver omap_rproc_driver = {
	.probe = omap_rproc_probe,
	.remove = omap_rproc_remove,
	.driver = {
		.name = "omap-rproc",
		.owner = THIS_MODULE,
		.pm = &omap_rproc_pm_ops,
		.of_match_table = of_match_ptr(omap_rproc_of_match),
	},
};

module_platform_driver(omap_rproc_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("OMAP Remote Processor control driver");
