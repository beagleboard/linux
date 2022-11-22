// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2021 Alibaba Group Holding Limited.
 */

#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/firmware/thead/ipc.h>

#define MBOX_MAX_MSG_LEN		28

static struct dentry *root_debugfs_dir;

struct light_aon_msg_req_misc_set_ctrl {
	struct light_aon_rpc_msg_hdr hdr;
	u32 ctrl;
	u32 val;
	u16 resource;
	u16 reserved[7];
} __packed __aligned(4);

struct light_aon_msg_req_misc_get_ctrl {
	struct light_aon_rpc_msg_hdr hdr;
	u32 ctrl;
	u16 resource;
	u16 reserved[9];
} __packed __aligned(4);

struct light_aon_msg_resp_misc_get_ctrl {
	struct light_aon_rpc_msg_hdr hdr;
	u32 val;
	u32 reserved[5];
} __packed __aligned(4);

struct light_aon_device {
	struct device		*dev;
	char			*test_buf;
	struct light_aon_ipc	*ipc_handle;
};

static ssize_t light_aon_test_buf_write(struct file *filp,
				      const char __user *userbuf,
				      size_t count, loff_t *ppos)
{
	struct light_aon_device *tdev = filp->private_data;
	int ret;

	if (count > MBOX_MAX_MSG_LEN)
		count = MBOX_MAX_MSG_LEN;

	ret = copy_from_user(tdev->test_buf, userbuf, count);
	if (ret) {
		ret = -EFAULT;
		goto out;
	}

	ret = light_aon_misc_set_control(tdev->ipc_handle, 0x1, 0x2, 0x3);
	ret |= light_aon_misc_set_control(tdev->ipc_handle, 0x11, 0x12, 0x13);
	ret |= light_aon_misc_set_control(tdev->ipc_handle, 0x21, 0x22, 0x23);
	ret |= light_aon_misc_set_control(tdev->ipc_handle, 0x31, 0x32, 0x33);
	if (ret)
		dev_err(tdev->dev, "failed to set control\n");

	//print_hex_dump(KERN_INFO, __func__, DUMP_PREFIX_NONE, 16, 1, tdev->test_buf, MBOX_MAX_MSG_LEN, true);

out:
	return ret < 0 ? ret : count;
}

static ssize_t light_aon_test_buf_read(struct file *filp,
				      char __user *userbuf,
				      size_t count, loff_t *ppos)
{
	struct light_aon_device *tdev = filp->private_data;

	//print_hex_dump(KERN_INFO, __func__, DUMP_PREFIX_NONE, 16, 1, tdev->test_buf, MBOX_MAX_MSG_LEN, true);
	memset(tdev->test_buf, 0, MBOX_MAX_MSG_LEN);

	return MBOX_MAX_MSG_LEN;
}

static const struct file_operations light_aon_test_buf_ops = {
	.write	= light_aon_test_buf_write,
	.read	= light_aon_test_buf_read,
	.open	= simple_open,
	.llseek	= generic_file_llseek,
};

static int light_aon_add_debugfs(struct platform_device *pdev, struct light_aon_device *tdev)
{
	root_debugfs_dir = debugfs_create_dir("light_aon",NULL);
	if (!root_debugfs_dir) {
		dev_err(&pdev->dev, "Failed to create light_aon_test debugfs\n");
		return -EINVAL;
	}

	debugfs_create_file("test", 0600, root_debugfs_dir, tdev, &light_aon_test_buf_ops);
	return 0;
}

static int light_aon_probe(struct platform_device *pdev)
{
	struct light_aon_device *tdev;
	int ret;

	tdev = devm_kzalloc(&pdev->dev, sizeof(*tdev), GFP_KERNEL);
	if (!tdev)
		return -ENOMEM;

	tdev->dev = &pdev->dev;
	platform_set_drvdata(pdev, tdev);

	tdev->test_buf = devm_kzalloc(&pdev->dev, MBOX_MAX_MSG_LEN, GFP_KERNEL);
	if (!tdev->test_buf)
		return -ENOMEM;

	ret = light_aon_get_handle(&(tdev->ipc_handle));
	if (ret) {
		dev_err(&pdev->dev, "failed to get ipc_handle\n");
		return ret;
	}

	ret = light_aon_add_debugfs(pdev, tdev);
	if (ret)
		return ret;

	dev_info(&pdev->dev, "Successfully registered\n");

	return 0;
}

static int light_aon_remove(struct platform_device *pdev)
{
	debugfs_remove_recursive(root_debugfs_dir);
	return 0;
}

static const struct of_device_id light_aon_match[] = {
	{ .compatible = "thead,light-aon-test" },
	{},
};

static struct platform_driver light_aon_driver = {
	.driver = {
		.name = "thead,light-aon-test",
		.of_match_table = light_aon_match,
	},
	.probe  = light_aon_probe,
	.remove = light_aon_remove,
};
module_platform_driver(light_aon_driver);

MODULE_AUTHOR("fugang.duan <duanfugang.dfg@linux.alibaba.com>");
MODULE_DESCRIPTION("Thead Light firmware protocol test driver");
MODULE_LICENSE("GPL v2");
