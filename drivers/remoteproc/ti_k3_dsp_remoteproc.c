// SPDX-License-Identifier: GPL-2.0-only
/*
 * TI K3 DSP Remote Processor(s) driver
 *
 * Copyright (C) 2018-2022 Texas Instruments Incorporated - https://www.ti.com/
 *	Suman Anna <s-anna@ti.com>
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
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

/* C7x TI-SCI Processor Status Flags */
#define PROC_BOOT_STATUS_FLAG_CPU_WFE                   0x00000001
#define PROC_BOOT_STATUS_FLAG_CPU_WFI                   0x00000002

/**
 * is_core_in_wfi - local utility function to check core status
 * @kproc: remote core pointer used for checking core status
 *
 * This utility function is invoked by the shutdown sequence to ensure
 * the remote core is in wfi, before asserting a reset.
 */
static int is_core_in_wfi(struct k3_rproc *core)
{
	int ret;
	u64 boot_vec;
	u32 cfg, ctrl, stat;

	ret = ti_sci_proc_get_status(core->tsp, &boot_vec, &cfg, &ctrl, &stat);

	if (ret < 0)
		return 0;

	return (stat & PROC_BOOT_STATUS_FLAG_CPU_WFI);
}

/**
 * k3_dsp_rproc_mbox_callback() - inbound mailbox message handler
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
static void k3_dsp_rproc_mbox_callback(struct mbox_client *client, void *data)
{
	struct k3_rproc *kproc = container_of(client, struct k3_rproc,
						  client);
	struct device *dev = kproc->rproc->dev.parent;
	const char *name = kproc->rproc->name;
	u32 msg = omap_mbox_message(data);

	dev_dbg(dev, "mbox msg: 0x%x\n", msg);

	switch (msg) {
	case RP_MBOX_CRASH:
		/*
		 * remoteproc detected an exception, but error recovery is not
		 * supported. So, just log this for now
		 */
		dev_err(dev, "K3 DSP rproc %s crashed\n", name);
		break;
	case RP_MBOX_ECHO_REPLY:
		dev_info(dev, "received echo reply from %s\n", name);
		break;
	case RP_MBOX_SHUTDOWN_ACK:
		dev_dbg(dev, "received shutdown_ack from %s\n", name);
		complete(&kproc->shut_comp);
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

static int k3_dsp_rproc_request_mbox(struct rproc *rproc)
{
	struct k3_rproc *kproc = rproc->priv;
	struct mbox_client *client = &kproc->client;
	struct device *dev = kproc->dev;
	int ret;

	client->dev = dev;
	client->tx_done = NULL;
	client->rx_callback = k3_dsp_rproc_mbox_callback;
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
 * The C66x DSP cores have a local reset that affects only the CPU, and a
 * generic module reset that powers on the device and allows the DSP internal
 * memories to be accessed while the local reset is asserted. This function is
 * used to release the global reset on C66x DSPs to allow loading into the DSP
 * internal RAMs. The .prepare() ops is invoked by remoteproc core before any
 * firmware loading, and is followed by the .start() ops after loading to
 * actually let the C66x DSP cores run. This callback is invoked only in
 * remoteproc mode.
 */
static int k3_dsp_rproc_prepare(struct rproc *rproc)
{
	struct k3_rproc *kproc = rproc->priv;
	struct device *dev = kproc->dev;
	int ret;

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
 * global reset on applicable C66x cores. This completes the second portion of
 * powering down the C66x DSP cores. The cores themselves are only halted in the
 * .stop() callback through the local reset, and the .unprepare() ops is invoked
 * by the remoteproc core after the remoteproc is stopped to balance the global
 * reset. This callback is invoked only in remoteproc mode.
 */
static int k3_dsp_rproc_unprepare(struct rproc *rproc)
{
	struct k3_rproc *kproc = rproc->priv;
	struct device *dev = kproc->dev;
	int ret;

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
static int dsp_pm_notifier_call(struct notifier_block *bl,
				unsigned long state, void *unused)
{
	struct k3_rproc *kproc = container_of(bl, struct k3_rproc, pm_notifier);
	struct device *dev = kproc->dev;

	switch (state) {
	case PM_HIBERNATION_PREPARE:
	case PM_RESTORE_PREPARE:
	case PM_SUSPEND_PREPARE:
		if (!device_may_wakeup(kproc->dev))
			if (rproc_shutdown(kproc->rproc)) {
				dev_err(dev, "failed to shutdown remote core\n");
				return NOTIFY_BAD;
			}
		kproc->rproc->state = RPROC_SUSPENDED;
	break;
	case PM_POST_HIBERNATION:
	case PM_POST_RESTORE:
	case PM_POST_SUSPEND:
		if (kproc->rproc->state == RPROC_SUSPENDED) {
			if (!device_may_wakeup(kproc->dev))
				rproc_boot(kproc->rproc);
			kproc->rproc->state = RPROC_RUNNING;
		}
	break;
	}
	return 0;
}

/*
 * Power up the DSP remote processor.
 *
 * This function will be invoked only after the firmware for this rproc
 * was loaded, parsed successfully, and all of its resource requirements
 * were met. This callback is invoked only in remoteproc mode.
 */
static int k3_dsp_rproc_start(struct rproc *rproc)
{
	struct k3_rproc *kproc = rproc->priv;
	struct device *dev = kproc->dev;
	u32 boot_addr;
	int ret;

	ret = k3_dsp_rproc_request_mbox(rproc);
	if (ret)
		return ret;

	boot_addr = rproc->bootaddr;
	if (boot_addr & (kproc->data->boot_align_addr - 1)) {
		dev_err(dev, "invalid boot address 0x%x, must be aligned on a 0x%x boundary\n",
			boot_addr, kproc->data->boot_align_addr);
		ret = -EINVAL;
		goto put_mbox;
	}

	dev_err(dev, "booting DSP core using boot addr = 0x%x\n", boot_addr);
	ret = ti_sci_proc_set_config(kproc->tsp, boot_addr, 0, 0);
	if (ret)
		goto put_mbox;

	ret = k3_rproc_release(kproc);
	if (ret)
		goto put_mbox;

	return 0;

put_mbox:
	mbox_free_channel(kproc->mbox);
	return ret;
}

/*
 * Stop the DSP remote processor.
 *
 * This function puts the DSP processor into reset, and finishes processing
 * of any pending messages. This callback is invoked only in remoteproc mode.
 */
static int k3_dsp_rproc_stop(struct rproc *rproc)
{
	unsigned long to = msecs_to_jiffies(3000);
	struct k3_rproc *kproc = rproc->priv;
	struct device *dev = kproc->dev;
	unsigned long msg = RP_MBOX_SHUTDOWN;
	u32 stat = 0;
	int ret;

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

	/* poll remote core's status, with a timeout of 2ms and 200us sleeps in between */
	ret = readx_poll_timeout(is_core_in_wfi, kproc, stat, stat, 200, 2000);
	if (ret)
		return ret;

	k3_rproc_reset(kproc);

	return 0;
}

/*
 * Attach to a running DSP remote processor (IPC-only mode)
 *
 * This rproc attach callback only needs to request the mailbox, the remote
 * processor is already booted, so there is no need to issue any TI-SCI
 * commands to boot the DSP core. This callback is invoked only in IPC-only
 * mode.
 */
static int k3_dsp_rproc_attach(struct rproc *rproc)
{
	struct k3_rproc *kproc = rproc->priv;
	struct device *dev = kproc->dev;
	int ret;

	ret = k3_dsp_rproc_request_mbox(rproc);
	if (ret)
		return ret;

	dev_info(dev, "DSP initialized in IPC-only mode\n");
	return 0;
}

/*
 * Detach from a running DSP remote processor (IPC-only mode)
 *
 * This rproc detach callback performs the opposite operation to attach callback
 * and only needs to release the mailbox, the DSP core is not stopped and will
 * be left to continue to run its booted firmware. This callback is invoked only
 * in IPC-only mode.
 */
static int k3_dsp_rproc_detach(struct rproc *rproc)
{
	struct k3_rproc *kproc = rproc->priv;
	struct device *dev = kproc->dev;

	mbox_free_channel(kproc->mbox);
	dev_info(dev, "DSP deinitialized in IPC-only mode\n");
	return 0;
}


static const struct rproc_ops k3_dsp_rproc_ops = {
	.start		= k3_dsp_rproc_start,
	.stop		= k3_dsp_rproc_stop,
	.kick		= k3_rproc_kick,
	.da_to_va	= k3_rproc_da_to_va,
};

static int k3_dsp_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	const struct k3_rproc_dev_data *data;
	struct k3_rproc *kproc;
	struct rproc *rproc;
	const char *fw_name;
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

	rproc = rproc_alloc(dev, dev_name(dev), &k3_dsp_rproc_ops, fw_name,
			    sizeof(*kproc));
	if (!rproc)
		return -ENOMEM;

	rproc->has_iommu = false;
	rproc->recovery_disabled = true;
	if (data->uses_lreset) {
		rproc->ops->prepare = k3_dsp_rproc_prepare;
		rproc->ops->unprepare = k3_dsp_rproc_unprepare;
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

	ret = ti_sci_proc_request(kproc->tsp);
	if (ret < 0) {
		dev_err(dev, "ti_sci_proc_request failed, ret = %d\n", ret);
		goto free_tsp;
	}

	ret = k3_rproc_of_get_memories(pdev, kproc);
	if (ret)
		goto release_tsp;

	ret = k3_reserved_mem_init(kproc);
	if (ret) {
		dev_err(dev, "reserved memory init failed, ret = %d\n", ret);
		goto release_tsp;
	}

	ret = kproc->ti_sci->ops.dev_ops.is_on(kproc->ti_sci, kproc->ti_sci_id,
					       NULL, &p_state);
	if (ret) {
		dev_err(dev, "failed to get initial state, mode cannot be determined, ret = %d\n",
			ret);
		goto release_mem;
	}

	/* configure J721E devices for either remoteproc or IPC-only mode */
	if (p_state) {
		dev_info(dev, "configured DSP for IPC-only mode\n");
		rproc->state = RPROC_DETACHED;
		/* override rproc ops with only required IPC-only mode ops */
		rproc->ops->prepare = NULL;
		rproc->ops->unprepare = NULL;
		rproc->ops->start = NULL;
		rproc->ops->stop = NULL;
		rproc->ops->attach = k3_dsp_rproc_attach;
		rproc->ops->detach = k3_dsp_rproc_detach;
		rproc->ops->get_loaded_rsc_table = k3_get_loaded_rsc_table;
	} else {
		dev_info(dev, "configured DSP for remoteproc mode\n");
		/*
		 * ensure the DSP local reset is asserted to ensure the DSP
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

	/* Register pm notifiers */
	dev_info(dev, "register pm nitifiers in remoteproc mode\n");
	init_completion(&kproc->shut_comp);
	kproc->pm_notifier.notifier_call = dsp_pm_notifier_call;
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

static int k3_dsp_rproc_remove(struct platform_device *pdev)
{
	struct k3_rproc *kproc = platform_get_drvdata(pdev);
	struct rproc *rproc = kproc->rproc;
	struct device *dev = &pdev->dev;
	int ret;

	if (rproc->state == RPROC_ATTACHED) {
		ret = rproc_detach(rproc);
		if (ret) {
			dev_err(dev, "failed to detach proc, ret = %d\n", ret);
			return ret;
		}
	}

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

static const struct k3_rproc_mem_data c66_mems[] = {
	{ .name = "l2sram", .dev_addr = 0x800000 },
	{ .name = "l1pram", .dev_addr = 0xe00000 },
	{ .name = "l1dram", .dev_addr = 0xf00000 },
};

/* C71x cores only have a L1P Cache, there are no L1P SRAMs */
static const struct k3_rproc_mem_data c71_mems[] = {
	{ .name = "l2sram", .dev_addr = 0x800000 },
	{ .name = "l1dram", .dev_addr = 0xe00000 },
};

static const struct k3_rproc_mem_data c7xv_mems[] = {
	{ .name = "l2sram", .dev_addr = 0x800000 },
};

static const struct k3_rproc_dev_data c66_data = {
	.mems = c66_mems,
	.num_mems = ARRAY_SIZE(c66_mems),
	.boot_align_addr = SZ_1K,
	.uses_lreset = true,
};

static const struct k3_rproc_dev_data c71_data = {
	.mems = c71_mems,
	.num_mems = ARRAY_SIZE(c71_mems),
	.boot_align_addr = SZ_2M,
	.uses_lreset = false,
};

static const struct k3_rproc_dev_data c7xv_data = {
	.mems = c7xv_mems,
	.num_mems = ARRAY_SIZE(c7xv_mems),
	.boot_align_addr = SZ_2M,
	.uses_lreset = false,
};

static const struct of_device_id k3_dsp_of_match[] = {
	{ .compatible = "ti,j721e-c66-dsp", .data = &c66_data, },
	{ .compatible = "ti,j721e-c71-dsp", .data = &c71_data, },
	{ .compatible = "ti,j721s2-c71-dsp", .data = &c71_data, },
	{ .compatible = "ti,am62a-c7xv-dsp", .data = &c7xv_data, },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, k3_dsp_of_match);

static struct platform_driver k3_dsp_rproc_driver = {
	.probe	= k3_dsp_rproc_probe,
	.remove	= k3_dsp_rproc_remove,
	.driver	= {
		.name = "k3-dsp-rproc",
		.of_match_table = k3_dsp_of_match,
	},
};

module_platform_driver(k3_dsp_rproc_driver);

MODULE_AUTHOR("Suman Anna <s-anna@ti.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("TI K3 DSP Remoteproc driver");
