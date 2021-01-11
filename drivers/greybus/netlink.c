/*
 * Greybus Netlink driver for userspace controller
 *
 * Copyright (c) 2017 BayLibre SAS
 *
 * Released under the GPLv2 only.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <net/genetlink.h>
#include <linux/greybus.h>

#include "gb_netlink.h"

static dev_t major_dev;
static struct class *gb_nl_class;
static struct genl_family gb_nl_family;
static struct gb_host_device *gb_nl_hd;

#define VERSION_NR	1

#define DEVICE_NAME	"gb_netlink"
#define CLASS_NAME	"gb_netlink"

static int _gb_netlink_init(struct device *dev);
static void _gb_netlink_exit(void);

static int gb_netlink_msg(struct sk_buff *skb, struct genl_info *info)
{
	struct nlattr *na;
	u16 cport_id;
	void *data;

	if (!info)
		return -EPROTO;

	na = info->attrs[GB_NL_A_CPORT];
	if (!na) {
		dev_err(&gb_nl_hd->dev,
			"Received message without cport id attribute\n");
		return -EPROTO;
	}

	cport_id = nla_get_u32(na);
	if (!cport_id_valid(gb_nl_hd, cport_id)) {
		dev_err(&gb_nl_hd->dev, "invalid cport id %u received",
			cport_id);
		return -EINVAL;
	}

	na = info->attrs[GB_NL_A_DATA];
	if (!na) {
		dev_err(&gb_nl_hd->dev,
			"Received message without data attribute\n");
		return -EPROTO;
	}

	data = nla_data(na);
	if (!data) {
		dev_err(&gb_nl_hd->dev,
			"Received message without data\n");
		return -EINVAL;
	}

	greybus_data_rcvd(gb_nl_hd, cport_id, data, nla_len(na));

	return 0;
}

static int gb_netlink_hd_reset(struct sk_buff *skb, struct genl_info *info)
{
	struct device *dev;
	struct gb_host_device *hd = gb_nl_hd;

	dev = hd->dev.parent;
	_gb_netlink_exit();
	_gb_netlink_init(dev);

	return 0;
}

static struct nla_policy gb_nl_policy[GB_NL_A_MAX + 1] = {
	[GB_NL_A_DATA] = { .type = NLA_BINARY, .len = GB_NETLINK_MTU },
	[GB_NL_A_CPORT] = { .type = NLA_U32},
};

static struct genl_ops gb_nl_ops[] = {
	{
		.cmd = GB_NL_C_MSG,
		.doit = gb_netlink_msg,
	},
	{
		.cmd = GB_NL_C_HD_RESET,
		.doit = gb_netlink_hd_reset,
	},
};

static struct genl_family gb_nl_family = {
	.hdrsize = 0,
	.name = GB_NL_NAME,
	.version = VERSION_NR,
	.maxattr = GB_NL_A_MAX,
	.ops = gb_nl_ops,
	.n_ops = ARRAY_SIZE(gb_nl_ops),
        .policy = gb_nl_policy,
};

static int message_send(struct gb_host_device *hd, u16 cport_id,
			struct gb_message *message, gfp_t gfp_mask)
{
	struct nl_msg *nl_msg;
	struct sk_buff *skb;
	int retval = -ENOMEM;

	skb = genlmsg_new(sizeof(*message->header) + sizeof(u32) +
			  message->payload_size, GFP_KERNEL);
	if (!skb)
		goto err_out;

	nl_msg = genlmsg_put(skb, GB_NL_PID, 0,
			     &gb_nl_family, 0, GB_NL_C_MSG);
	if (!nl_msg)
		goto err_free;

	retval = nla_put_u32(skb, GB_NL_A_CPORT, cport_id);
	if (retval)
		goto err_cancel;

	retval = nla_put(skb, GB_NL_A_DATA,
			 sizeof(*message->header) + message->payload_size,
			 message->header);
	if (retval)
		goto err_cancel;

	genlmsg_end(skb, nl_msg);

	retval = genlmsg_unicast(&init_net, skb, GB_NL_PID);
	if (retval)
		goto err_cancel;

	greybus_message_sent(hd, message, 0);

	return 0;

err_cancel:
	genlmsg_cancel(skb, nl_msg);
err_free:
	nlmsg_free(skb);
err_out:
	return retval;
}

static void message_cancel(struct gb_message *message)
{
}

static struct gb_hd_driver tcpip_driver = {
	.message_send		= message_send,
	.message_cancel		= message_cancel,
};

static void _gb_netlink_exit(void)
{
	if (!gb_nl_hd)
		return;

	gb_hd_del(gb_nl_hd);
	gb_hd_put(gb_nl_hd);

	gb_nl_hd = NULL;
}

static void __exit gb_netlink_exit(void)
{
	_gb_netlink_exit();

	unregister_chrdev_region(major_dev, 1);
	device_destroy(gb_nl_class, major_dev);
	class_destroy(gb_nl_class);

	genl_unregister_family(&gb_nl_family);
}

static int _gb_netlink_init(struct device *dev)
{
	int retval;

	gb_nl_hd = gb_hd_create(&tcpip_driver, dev, GB_NETLINK_MTU,
				GB_NETLINK_NUM_CPORT);
	if (IS_ERR(gb_nl_hd))
		return PTR_ERR(gb_nl_hd);

	retval = gb_hd_add(gb_nl_hd);
	if (retval)
		goto err_gb_hd_del;

	return 0;

err_gb_hd_del:
	gb_hd_del(gb_nl_hd);
	gb_hd_put(gb_nl_hd);

	return retval;
}

static int __init gb_netlink_init(void)
{
	int retval;
	struct device *dev;

	retval = genl_register_family(&gb_nl_family);
	if (retval)
		return retval;

	retval = alloc_chrdev_region(&major_dev, 0, 1, DEVICE_NAME);
	if (retval)
		goto err_genl_unregister;

	gb_nl_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(gb_nl_class)) {
		retval = PTR_ERR(gb_nl_class);
		goto err_chrdev_unregister;
	}

	dev = device_create(gb_nl_class, NULL, major_dev, NULL, DEVICE_NAME);
	if (IS_ERR(dev)) {
		retval = PTR_ERR(dev);
		goto err_class_destroy;
	}

	retval = _gb_netlink_init(dev);
	if (retval)
		goto err_device_destroy;

	return 0;

err_device_destroy:
	device_destroy(gb_nl_class, major_dev);
err_chrdev_unregister:
	unregister_chrdev_region(major_dev, 1);
err_class_destroy:
	class_destroy(gb_nl_class);
err_genl_unregister:
	genl_unregister_family(&gb_nl_family);

	return retval;
}

module_init(gb_netlink_init);
module_exit(gb_netlink_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Alexandre Bailon <abailon@baylibre.com>");
