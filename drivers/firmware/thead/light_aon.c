// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2021 Alibaba Group Holding Limited.
 */

#include <linux/err.h>
#include <linux/firmware/thead/ipc.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#define MAX_RX_TIMEOUT		(msecs_to_jiffies(300))
#define MAX_TX_TIMEOUT		(msecs_to_jiffies(500))

struct light_aon_chan {
	struct light_aon_ipc *aon_ipc;

	struct mbox_client cl;
	struct mbox_chan *ch;
	struct completion tx_done;
};

struct light_aon_ipc {
	struct light_aon_chan chans;
	struct device *dev;
	struct mutex lock;
	struct completion done;
	u32 *msg;
};

/*
 * This type is used to indicate error response for most functions.
 */
enum light_aon_error_codes {
	LIGHT_AON_ERR_NONE = 0,	/* Success */
	LIGHT_AON_ERR_VERSION = 1,	/* Incompatible API version */
	LIGHT_AON_ERR_CONFIG = 2,	/* Configuration error */
	LIGHT_AON_ERR_PARM = 3,	/* Bad parameter */
	LIGHT_AON_ERR_NOACCESS = 4,	/* Permission error (no access) */
	LIGHT_AON_ERR_LOCKED = 5,	/* Permission error (locked) */
	LIGHT_AON_ERR_UNAVAILABLE = 6,	/* Unavailable (out of resources) */
	LIGHT_AON_ERR_NOTFOUND = 7,	/* Not found */
	LIGHT_AON_ERR_NOPOWER = 8,	/* No power */
	LIGHT_AON_ERR_IPC = 9,		/* Generic IPC error */
	LIGHT_AON_ERR_BUSY = 10,	/* Resource is currently busy/active */
	LIGHT_AON_ERR_FAIL = 11,	/* General I/O failure */
	LIGHT_AON_ERR_LAST
};

static int light_aon_linux_errmap[LIGHT_AON_ERR_LAST] = {
	0,	 /* LIGHT_AON_ERR_NONE */
	-EINVAL, /* LIGHT_AON_ERR_VERSION */
	-EINVAL, /* LIGHT_AON_ERR_CONFIG */
	-EINVAL, /* LIGHT_AON_ERR_PARM */
	-EACCES, /* LIGHT_AON_ERR_NOACCESS */
	-EACCES, /* LIGHT_AON_ERR_LOCKED */
	-ERANGE, /* LIGHT_AON_ERR_UNAVAILABLE */
	-EEXIST, /* LIGHT_AON_ERR_NOTFOUND */
	-EPERM,	 /* LIGHT_AON_ERR_NOPOWER */
	-EPIPE,	 /* LIGHT_AON_ERR_IPC */
	-EBUSY,	 /* LIGHT_AON_ERR_BUSY */
	-EIO,	 /* LIGHT_AON_ERR_FAIL */
};

static struct light_aon_ipc *light_aon_ipc_handle;

static inline int light_aon_to_linux_errno(int errno)
{
	if (errno >= LIGHT_AON_ERR_NONE && errno < LIGHT_AON_ERR_LAST)
		return light_aon_linux_errmap[errno];
	return -EIO;
}

/*
 * Get the default handle used by SCU
 */
int light_aon_get_handle(struct light_aon_ipc **ipc)
{
	if (!light_aon_ipc_handle)
		return -EPROBE_DEFER;

	*ipc = light_aon_ipc_handle;
	return 0;
}
EXPORT_SYMBOL(light_aon_get_handle);

static void light_aon_tx_done(struct mbox_client *cl, void *mssg, int r)
{
	struct light_aon_chan *aon_chan = container_of(cl, struct light_aon_chan, cl);

	complete(&aon_chan->tx_done);
}

static void light_aon_rx_callback(struct mbox_client *c, void *msg)
{
	struct light_aon_chan *aon_chan = container_of(c, struct light_aon_chan, cl);
	struct light_aon_ipc *aon_ipc = aon_chan->aon_ipc;

	memcpy(aon_ipc->msg, msg, LIGHT_AON_RPC_MSG_NUM * sizeof(u32));
	dev_dbg(aon_ipc->dev, "msg head: 0x%x\n", *((u32 *)msg));
	complete(&aon_ipc->done);
}

static int light_aon_ipc_write(struct light_aon_ipc *aon_ipc, void *msg)
{
	struct light_aon_rpc_msg_hdr *hdr = msg;
	struct light_aon_chan *aon_chan;
	u32 *data = msg;
	int ret;

	/* check size, currently it requires 7 MSG in one transfer */
	if (hdr->size != LIGHT_AON_RPC_MSG_NUM)
		return -EINVAL;

	dev_dbg(aon_ipc->dev, "RPC SVC %u FUNC %u SIZE %u\n", hdr->svc,
		hdr->func, hdr->size);

	aon_chan = &aon_ipc->chans;

	if (!wait_for_completion_timeout(&aon_chan->tx_done,
					 MAX_TX_TIMEOUT)) {
		dev_err(aon_ipc->dev, "tx_done timeout\n");
		return -ETIMEDOUT;
	}
	reinit_completion(&aon_chan->tx_done);

	ret = mbox_send_message(aon_chan->ch, data);
	if (ret < 0)
		return ret;

	return 0;
}

/*
 * RPC command/response
 */
int light_aon_call_rpc(struct light_aon_ipc *aon_ipc, void *msg, bool have_resp)
{
	struct light_aon_rpc_msg_hdr *hdr;
	int ret;

	if (WARN_ON(!aon_ipc || !msg))
		return -EINVAL;

	mutex_lock(&aon_ipc->lock);
	reinit_completion(&aon_ipc->done);

	if (have_resp)
		aon_ipc->msg = msg;

	ret = light_aon_ipc_write(aon_ipc, msg);
	if (ret < 0) {
		dev_err(aon_ipc->dev, "RPC send msg failed: %d\n", ret);
		goto out;
	}

	if (have_resp) {
		if (!wait_for_completion_timeout(&aon_ipc->done,
						 MAX_RX_TIMEOUT)) {
			dev_err(aon_ipc->dev, "RPC send msg timeout\n");
			mutex_unlock(&aon_ipc->lock);
			return -ETIMEDOUT;
		}

		/* response status is stored in hdr->func field */
		hdr = msg;
		ret = hdr->func;
	}

out:
	mutex_unlock(&aon_ipc->lock);

	dev_dbg(aon_ipc->dev, "RPC SVC done\n");

	return light_aon_to_linux_errno(ret);
}
EXPORT_SYMBOL(light_aon_call_rpc);

static int light_aon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct light_aon_ipc *aon_ipc;
	struct light_aon_chan *aon_chan;
	struct mbox_client *cl;
	int ret;

	aon_ipc = devm_kzalloc(dev, sizeof(*aon_ipc), GFP_KERNEL);
	if (!aon_ipc)
		return -ENOMEM;

	aon_chan = &aon_ipc->chans;
	cl = &aon_chan->cl;
	cl->dev = dev;
	cl->tx_block = false;
	cl->knows_txdone = true;
	cl->rx_callback = light_aon_rx_callback;

	/* Initial tx_done completion as "done" */
	cl->tx_done = light_aon_tx_done;
	init_completion(&aon_chan->tx_done);
	complete(&aon_chan->tx_done);

	aon_chan->aon_ipc = aon_ipc;
	aon_chan->ch = mbox_request_channel_byname(cl, "aon");
	if (IS_ERR(aon_chan->ch)) {
		ret = PTR_ERR(aon_chan->ch);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to request aon mbox chan ret %d\n", ret);
		return ret;
	}

	dev_dbg(dev, "request thead mbox chan: aon\n");

	aon_ipc->dev = dev;
	mutex_init(&aon_ipc->lock);
	init_completion(&aon_ipc->done);

	light_aon_ipc_handle = aon_ipc;

	return devm_of_platform_populate(dev);
}

static const struct of_device_id light_aon_match[] = {
	{ .compatible = "thead,light-aon", },
	{ /* Sentinel */ }
};

static struct platform_driver light_aon_driver = {
	.driver = {
		.name = "light-aon",
		.of_match_table = light_aon_match,
	},
	.probe = light_aon_probe,
};
builtin_platform_driver(light_aon_driver);

MODULE_AUTHOR("fugang.duan <duanfugang.dfg@linux.alibaba.com>");
MODULE_DESCRIPTION("Thead Light firmware protocol driver");
MODULE_LICENSE("GPL v2");
