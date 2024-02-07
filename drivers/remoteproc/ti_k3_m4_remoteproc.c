// SPDX-License-Identifier: GPL-2.0-only
/*
 * TI K3 Cortex-M4 Remote Processor(s) driver
 *
 * Copyright (C) 2021 Texas Instruments Incorporated - https://www.ti.com/
 *	Hari Nagalla <hnagalla@ti.com>
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/omap-mailbox.h>
#include <linux/platform_device.h>
#include <linux/remoteproc.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/suspend.h>

#include "omap_remoteproc.h"
#include "remoteproc_internal.h"
#include "ti_sci_proc.h"
#include "ti_k3_common.h"

/**
 * k3_m4_rproc_mbox_callback() - inbound mailbox message handler
 * @client: mailbox client pointer used for requesting the mailbox channel
 * @data: mailbox payload
 *
 * This handler is invoked by the OMAP mailbox driver whenever a mailbox
 * message is received. Usually, the mailbox payload simply contains
 * the index of the virtqueue that is kicked by the remote processor,
 * and we let remoteproc core handle it.
 *
 * In addition to virtqueue indices, we also have some out-of-band values
 * that indicate different events. Those values are deliberately very
 * large so they don't coincide with virtqueue indices.
 */
static void k3_m4_rproc_mbox_callback(struct mbox_client *client, void *data)
{
	struct k3_rproc *kproc = container_of(client, struct k3_rproc,
						  client);
	struct device *dev = kproc->rproc->dev.parent;
	const char *name = kproc->rproc->name;
	u32 msg = omap_mbox_message(data);

	switch (msg) {
	case RP_MBOX_CRASH:
		/*
		 * remoteproc detected an exception, but error recovery is not
		 * supported. So, just log this for now
		 */
		dev_err(dev, "K3 M4 rproc %s crashed\n", name);
		break;
	case RP_MBOX_ECHO_REPLY:
		dev_info(dev, "received echo reply from %s\n", name);
		break;
	case RP_MBOX_SHUTDOWN_ACK:
		dev_dbg(dev, "received shutdown_ack from %s\n", name);
		complete(&kproc->shut_comp);
		break;
	case RP_MBOX_SUSPEND_ACK:
		dev_dbg(dev, "received suspend_ack from %s\n", name);
		complete(&kproc->suspend_comp);
		break;
	default:
		/* silently handle all other valid messages */
		if (msg >= RP_MBOX_READY && msg < RP_MBOX_END_MSG)
			return;
		if (msg > kproc->rproc->max_notifyid) {
			dev_dbg(dev, "dropping unknown message 0x%x", msg);
			return;
		}
		/* msg contains the index of the triggered vring */
		if (rproc_vq_interrupt(kproc->rproc, msg) == IRQ_NONE)
			dev_dbg(dev, "no message was found in vqid %d\n", msg);
	}
}

static int k3_m4_rproc_request_mbox(struct rproc *rproc)
{
	struct k3_rproc *kproc = rproc->priv;
	struct mbox_client *client = &kproc->client;
	struct device *dev = kproc->dev;
	int ret;

	client->dev = dev;
	client->tx_done = NULL;
	client->rx_callback = k3_m4_rproc_mbox_callback;
	client->tx_block = false;
	client->knows_txdone = false;

	kproc->mbox = mbox_request_channel(client, 0);
	if (IS_ERR(kproc->mbox)) {
		ret = -EBUSY;
		dev_err(dev, "mbox_request_channel failed: %ld\n",
			PTR_ERR(kproc->mbox));
		return ret;
	}

	/*
	 * Ping the remote processor, this is only for sanity-sake for now;
	 * there is no functional effect whatsoever.
	 *
	 * Note that the reply will _not_ arrive immediately: this message
	 * will wait in the mailbox fifo until the remote processor is booted.
	 */
	ret = mbox_send_message(kproc->mbox, (void *)RP_MBOX_ECHO_REQUEST);
	if (ret < 0) {
		dev_err(dev, "mbox_send_message failed: %d\n", ret);
		mbox_free_channel(kproc->mbox);
		return ret;
	}

	return 0;
}

/*
 * The M4F cores have a local reset that affects only the CPU, and a
 * generic module reset that powers on the device and allows the M4 internal
 * memories to be accessed while the local reset is asserted. This function is
 * used to release the global reset on M4F to allow loading into the M4F
 * internal RAMs. The .prepare() ops is invoked by remoteproc core before any
 * firmware loading, and is followed by the .start() ops after loading to
 * actually let the M4F core run.
 */
static int k3_m4_rproc_prepare(struct rproc *rproc)
{
	struct k3_rproc *kproc = rproc->priv;
	struct device *dev = kproc->dev;
	int ret;

	/* IPC-only mode does not require the core to be released from reset */
	if (kproc->ipc_only)
		return 0;

	ret = kproc->ti_sci->ops.dev_ops.get_device(kproc->ti_sci,
						    kproc->ti_sci_id);
	if (ret)
		dev_err(dev, "module-reset deassert failed, cannot enable internal RAM loading, ret = %d\n",
			ret);

	return ret;
}

/*
 * This function implements the .unprepare() ops and performs the complimentary
 * operations to that of the .prepare() ops. The function is used to assert the
 * global reset on applicable M4F cores. This completes the second portion of
 * powering down the M4F cores. The cores themselves are only halted in the
 * .stop() callback through the local reset, and the .unprepare() ops is invoked
 * by the remoteproc core after the remoteproc is stopped to balance the global
 * reset.
 */
static int k3_m4_rproc_unprepare(struct rproc *rproc)
{
	struct k3_rproc *kproc = rproc->priv;
	struct device *dev = kproc->dev;
	int ret;

	/* do not put back the cores into reset in IPC-only mode */
	if (kproc->ipc_only)
		return 0;

	ret = kproc->ti_sci->ops.dev_ops.put_device(kproc->ti_sci,
						    kproc->ti_sci_id);
	if (ret)
		dev_err(dev, "module-reset assert failed, ret = %d\n", ret);

	return ret;
}

/*
 * PM notifier call.
 * This is a callback function for PM notifications. On a resume completion
 * i.e after all the resume driver calls are handled on PM_POST_SUSPEND,
 * on a deep sleep the remote core is rebooted.
 */
static int m4f_pm_notifier_call(struct notifier_block *bl,
				unsigned long state, void *unused)
{
	struct k3_rproc *kproc = container_of(bl, struct k3_rproc, pm_notifier);
	unsigned long msg = RP_MBOX_SUSPEND_SYSTEM;
	unsigned long to = msecs_to_jiffies(3000);
	int ret;

	switch (state) {
	case PM_HIBERNATION_PREPARE:
	case PM_RESTORE_PREPARE:
	case PM_SUSPEND_PREPARE:
		if (!device_may_wakeup(kproc->dev)) {
			rproc_shutdown(kproc->rproc);
		} else {
			reinit_completion(&kproc->suspend_comp);
			ret = mbox_send_message(kproc->mbox, (void *)msg);
			if (ret < 0) {
				dev_err(kproc->dev, "PM mbox_send_message failed: %d\n", ret);
				return ret;
			}
			ret = wait_for_completion_timeout(&kproc->suspend_comp, to);
			if (ret == 0) {
				dev_err(kproc->dev,
					"%s: timedout waiting for rproc completion event\n", __func__);
				return -EBUSY;
			};
		}
		kproc->rproc->state = RPROC_SUSPENDED;
		break;
	case PM_POST_HIBERNATION:
	case PM_POST_RESTORE:
	case PM_POST_SUSPEND:
		if (kproc->rproc->state == RPROC_SUSPENDED) {
			if (!device_may_wakeup(kproc->dev)) {
				rproc_boot(kproc->rproc);
			} else {
				msg = RP_MBOX_ECHO_REQUEST;
				ret = mbox_send_message(kproc->mbox, (void *)msg);
				if (ret < 0) {
					dev_err(kproc->dev,
						"PM mbox_send_message failed: %d\n", ret);
					return ret;
				}
			}
			kproc->rproc->state = RPROC_RUNNING;
		}
		break;
	}
	return 0;
}

/*
 * Power up the M4F remote processor.
 *
 * This function will be invoked only after the firmware for this rproc
 * was loaded, parsed successfully, and all of its resource requirements
 * were met.
 */
static int k3_m4_rproc_start(struct rproc *rproc)
{
	struct k3_rproc *kproc = rproc->priv;
	struct device *dev = kproc->dev;
	int ret;

	if (kproc->ipc_only) {
		dev_err(dev, "%s cannot be invoked in IPC-only mode\n",
			__func__);
		return -EINVAL;
	}

	ret = k3_m4_rproc_request_mbox(rproc);
	if (ret)
		return ret;

	ret = k3_rproc_release(kproc);
	if (ret)
		goto put_mbox;

	return 0;

put_mbox:
	mbox_free_channel(kproc->mbox);
	return ret;
}

/*
 * Stop the M4 remote processor.
 *
 * This function puts the M4 processor into reset, and finishes processing
 * of any pending messages.
 */
static int k3_m4_rproc_stop(struct rproc *rproc)
{
	unsigned long to = msecs_to_jiffies(3000);
	struct k3_rproc *kproc = rproc->priv;
	struct device *dev = kproc->dev;
	unsigned long msg = RP_MBOX_SHUTDOWN;
	int ret;

	if (kproc->ipc_only) {
		dev_err(dev, "%s cannot be invoked in IPC-only mode\n",
			__func__);
		return -EINVAL;
	}

	reinit_completion(&kproc->shut_comp);
	ret = mbox_send_message(kproc->mbox, (void *)msg);
	if (ret < 0) {
		dev_err(dev, "PM mbox_send_message failed: %d\n", ret);
		return ret;
	}

	ret = wait_for_completion_timeout(&kproc->shut_comp, to);
	if (ret == 0) {
		dev_err(dev, "%s: timedout waiting for rproc completion event\n", __func__);
		return -EBUSY;
	};

	mbox_free_channel(kproc->mbox);
	/* allow some time for the remote core to enter quiescent state
	 * (ex:wfi) after sending SHUTDOWN ack. Typically, remote core is
	 * expected to enter 'wfi' right after sending the ack. 1 ms is
	 * more than sufficient for this prupose
	 */
	msleep(1);

	k3_rproc_reset(kproc);

	return 0;
}

/*
 * Attach to a running M4 remote processor (IPC-only mode)
 *
 * This rproc attach callback only needs to request the mailbox, the remote
 * processor is already booted, so there is no need to issue any TI-SCI
 * commands to boot the M4 core.
 */
static int k3_m4_rproc_attach(struct rproc *rproc)
{
	struct k3_rproc *kproc = rproc->priv;
	struct device *dev = kproc->dev;
	int ret;

	if (!kproc->ipc_only || rproc->state != RPROC_DETACHED) {
		dev_err(dev, "M4 is expected to be in IPC-only mode and RPROC_DETACHED state\n");
		return -EINVAL;
	}

	ret = k3_m4_rproc_request_mbox(rproc);
	if (ret)
		return ret;

	dev_err(dev, "M4 initialized in IPC-only mode\n");
	return 0;
}

/*
 * Detach from a running M4 remote processor (IPC-only mode)
 *
 * This rproc detach callback performs the opposite operation to attach callback
 * and only needs to release the mailbox, the M4 core is not stopped and will
 * be left to continue to run its booted firmware.
 */
static int k3_m4_rproc_detach(struct rproc *rproc)
{
	struct k3_rproc *kproc = rproc->priv;
	struct device *dev = kproc->dev;

	if (!kproc->ipc_only || rproc->state != RPROC_ATTACHED) {
		dev_err(dev, "M4 is expected to be in IPC-only mode and RPROC_ATTACHED state\n");
		return -EINVAL;
	}

	mbox_free_channel(kproc->mbox);
	dev_err(dev, "M4 deinitialized in IPC-only mode\n");
	return 0;
}



static const struct rproc_ops k3_m4_rproc_ops = {
	.start		= k3_m4_rproc_start,
	.stop		= k3_m4_rproc_stop,
	.attach		= k3_m4_rproc_attach,
	.detach		= k3_m4_rproc_detach,
	.kick		= k3_rproc_kick,
	.da_to_va	= k3_rproc_da_to_va,
	.get_loaded_rsc_table = k3_get_loaded_rsc_table,
};

static int k3_m4_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	const struct k3_rproc_dev_data *data;
	struct k3_rproc *kproc;
	struct rproc *rproc;
	const char *fw_name;
	bool r_state = false;
	bool p_state = false;
	int ret = 0;
	int ret1;

	data = of_device_get_match_data(dev);
	if (!data)
		return -ENODEV;

	ret = rproc_of_parse_firmware(dev, 0, &fw_name);
	if (ret) {
		dev_err(dev, "failed to parse firmware-name property, ret = %d\n",
			ret);
		return ret;
	}

	rproc = rproc_alloc(dev, dev_name(dev), &k3_m4_rproc_ops, fw_name,
			    sizeof(*kproc));
	if (!rproc)
		return -ENOMEM;

	rproc->has_iommu = false;
	rproc->recovery_disabled = true;
	if (data->uses_lreset) {
		rproc->ops->prepare = k3_m4_rproc_prepare;
		rproc->ops->unprepare = k3_m4_rproc_unprepare;
	}
	kproc = rproc->priv;
	kproc->rproc = rproc;
	kproc->dev = dev;
	kproc->data = data;

	kproc->ti_sci = ti_sci_get_by_phandle(np, "ti,sci");
	if (IS_ERR(kproc->ti_sci)) {
		ret = PTR_ERR(kproc->ti_sci);
		if (ret != -EPROBE_DEFER) {
			dev_err(dev, "failed to get ti-sci handle, ret = %d\n",
				ret);
		}
		kproc->ti_sci = NULL;
		goto free_rproc;
	}

	ret = of_property_read_u32(np, "ti,sci-dev-id", &kproc->ti_sci_id);
	if (ret) {
		dev_err(dev, "missing 'ti,sci-dev-id' property\n");
		goto put_sci;
	}

	if (device_property_present(dev, "wakeup-source")) {
		dev_dbg(dev, "registering as wakeup source\n");
		device_set_wakeup_capable(dev, true);
	}

	kproc->reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(kproc->reset)) {
		ret = PTR_ERR(kproc->reset);
		dev_err(dev, "failed to get reset, status = %d\n", ret);
		goto put_sci;
	}

	kproc->tsp = k3_rproc_of_get_tsp(dev, kproc->ti_sci);
	if (IS_ERR(kproc->tsp)) {
		dev_err(dev, "failed to construct ti-sci proc control, ret = %d\n",
			ret);
		ret = PTR_ERR(kproc->tsp);
		goto put_sci;
	}

	init_completion(&kproc->shut_comp);
	init_completion(&kproc->suspend_comp);

	ret = ti_sci_proc_request(kproc->tsp);
	if (ret < 0) {
		dev_err(dev, "ti_sci_proc_request failed, ret = %d\n", ret);
		goto free_tsp;
	}

	ret = k3_rproc_of_get_memories(pdev, kproc);
	if (ret)
		goto release_tsp;

	ret = k3_rproc_of_get_sram_memories(pdev, kproc);
	if (ret)
		goto release_tsp;

	ret = k3_reserved_mem_init(kproc);
	if (ret) {
		dev_err(dev, "reserved memory init failed, ret = %d\n", ret);
		goto release_tsp;
	}

	ret = kproc->ti_sci->ops.dev_ops.is_on(kproc->ti_sci, kproc->ti_sci_id,
					       &r_state, &p_state);
	if (ret) {
		dev_err(dev, "failed to get initial state, mode cannot be determined, ret = %d\n",
			ret);
		goto release_mem;
	}

	/* configure devices for either remoteproc or IPC-only mode */
	if (p_state) {
		dev_err(dev, "configured M4 for IPC-only mode\n");
		rproc->state = RPROC_DETACHED;
		kproc->ipc_only = true;
	} else {
		dev_err(dev, "configured M4 for remoteproc mode\n");
		/*
		 * ensure the M4 local reset is asserted to ensure the core
		 * doesn't execute bogus code in .prepare() when the module
		 * reset is released.
		 */
		if (data->uses_lreset) {
			ret = reset_control_status(kproc->reset);
			if (ret < 0) {
				dev_err(dev, "failed to get reset status, status = %d\n",
					ret);
				goto release_mem;
			} else if (ret == 0) {
				dev_warn(dev, "local reset is deasserted for device\n");
				k3_rproc_reset(kproc);
			}
		}
	}

	ret = rproc_add(rproc);
	if (ret) {
		dev_err(dev, "failed to add register device with remoteproc core, status = %d\n",
			ret);
		goto release_mem;
	}

	platform_set_drvdata(pdev, kproc);

	/* configure pm notifier calls */
	kproc->pm_notifier.notifier_call = m4f_pm_notifier_call;
	register_pm_notifier(&kproc->pm_notifier);

	return 0;

release_mem:
	k3_reserved_mem_exit(kproc);
release_tsp:
	ret1 = ti_sci_proc_release(kproc->tsp);
	if (ret1)
		dev_err(dev, "failed to release proc, ret = %d\n", ret1);
free_tsp:
	kfree(kproc->tsp);
put_sci:
	ret1 = ti_sci_put_handle(kproc->ti_sci);
	if (ret1)
		dev_err(dev, "failed to put ti_sci handle, ret = %d\n", ret1);
free_rproc:
	rproc_free(rproc);
	return ret;
}

static int k3_m4_rproc_remove(struct platform_device *pdev)
{
	struct k3_rproc *kproc = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	int ret;

	rproc_del(kproc->rproc);

	ret = ti_sci_proc_release(kproc->tsp);
	if (ret)
		dev_err(dev, "failed to release proc, ret = %d\n", ret);

	kfree(kproc->tsp);

	ret = ti_sci_put_handle(kproc->ti_sci);
	if (ret)
		dev_err(dev, "failed to put ti_sci handle, ret = %d\n", ret);

	k3_reserved_mem_exit(kproc);

	unregister_pm_notifier(&kproc->pm_notifier);

	rproc_free(kproc->rproc);

	return 0;
}

static const struct k3_rproc_mem_data am64_m4_mems[] = {
	{ .name = "iram", .dev_addr = 0x0 },
	{ .name = "dram", .dev_addr = 0x30000 },
};

static const struct k3_rproc_dev_data am64_m4_data = {
	.mems = am64_m4_mems,
	.num_mems = ARRAY_SIZE(am64_m4_mems),
	.boot_align_addr = SZ_1K,
	.uses_lreset = true,
};

static const struct of_device_id k3_m4_of_match[] = {
	{ .compatible = "ti,am64-m4fss", .data = &am64_m4_data, },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, k3_m4_of_match);

static struct platform_driver k3_m4_rproc_driver = {
	.probe	= k3_m4_rproc_probe,
	.remove	= k3_m4_rproc_remove,
	.driver	= {
		.name = "k3-m4-rproc",
		.of_match_table = k3_m4_of_match,
	},
};

module_platform_driver(k3_m4_rproc_driver);

MODULE_AUTHOR("Hari Nagalla <hnagalla@ti.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("TI K3 M4 Remoteproc driver");
