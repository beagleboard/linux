/*
 * This file is part of TI BLE over SDIO
 *
 * Copyright (C) 2022 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */

#include <linux/debugfs.h>
#include <linux/slab.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#include "btti_drv.h"

struct btti_debugfs_dir {
	struct dentry *config_dir;
	struct dentry *status_dir;
};

int btti_debugfs_if_prepare_command(u8 cmd_type,\
		struct btti_private *private_data);


#define DEBUGFS_FORMAT_BUFFER_SIZE 256

int btti_format_buffer(char __user *userbuf, size_t count,
			 loff_t *ppos, char *fmt, ...)
{
	va_list args;
	char buf[DEBUGFS_FORMAT_BUFFER_SIZE];
	int res;

	va_start(args, fmt);
	res = vscnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);

	return simple_read_from_buffer(userbuf, count, ppos, buf, res);
}

//ble_enable
static ssize_t ble_enable_read(struct file *file, char __user *userbuf,
						size_t count, loff_t *ppos)
{
	struct btti_private *private_data = file->private_data;
	char buf[16];
	int ret;
	BT_INFO("[bt sdio] ble enable read");

	ret = snprintf(buf, sizeof(buf) - 1, "%d\n", \
			private_data->hci_adapter->ble_enable);

	return btti_format_buffer(userbuf, count,  ppos, "%d\n", \
			private_data->hci_adapter->ble_enable);
}

static ssize_t ble_enable_write(struct file *file,
				const char __user *user_buf,
				size_t count, loff_t *ppos)
{
	struct btti_private *private_data = file->private_data;
	unsigned long value;
	int ret;

	ret = kstrtoul_from_user(user_buf, count, 0, &value);
	BT_INFO("[bt sdio] ble enable write: %ld", value);

	if(!private_data || !private_data->hci_adapter){
		BT_ERR("[bt sdio] driver is not ready");
	}

	if (private_data->hci_adapter->ble_enable) {
		BT_WARN("[bt sdio] ble_enable is already %d",\
				private_data->hci_adapter->ble_enable);
		return -EINVAL;
	}

	if (value != 1) {
		BT_WARN("illegal value in ble_enable "\
				"(only value allowed is is 1)");
		BT_WARN("ble_enable cant be disabled after being enabled.");
		return -EINVAL;
	}

	ret = btti_debugfs_if_prepare_command((u8)CMD_TYPE_BLE_ENABLE,\
			private_data);

	return count;
}



//ble_enable

static const struct file_operations ble_enable_ops = {
	.read = ble_enable_read,
	.write = ble_enable_write,
	.open = simple_open,
	.llseek = default_llseek,
};


void btti_debugfs_init(struct hci_dev *hdev)
{
	struct btti_private *private_data = hci_get_drvdata(hdev);
	struct btti_debugfs_dir *dbg;
	struct dentry *root_cfg_dir;

	if (!hdev->debugfs)
		return;

	dbg = kzalloc(sizeof(*dbg), GFP_KERNEL);
	private_data->debugfs_dir_vals = dbg;

	if (!dbg) {
		BT_ERR("Can not allocate memory for btti_debugfs_dir.");
		return;
	}

	dbg->config_dir = debugfs_create_dir("config", hdev->debugfs);
	root_cfg_dir = dbg->config_dir;

	debugfs_create_file("ble_enable", 0744, root_cfg_dir,  private_data,\
			&ble_enable_ops);

	//dbg->status_dir = debugfs_create_dir("cc3xx_status", hdev->debugfs);
}

void btti_debugfs_remove(struct hci_dev *hdev)
{
	struct btti_private *private_data = hci_get_drvdata(hdev);
	struct btti_debugfs_dir *dbgfs = private_data->debugfs_dir_vals;

	if (!dbgfs)
		return;

	if(dbgfs->config_dir)
	{
		debugfs_remove_recursive(dbgfs->config_dir);
	}
	if(dbgfs->status_dir)
	{
		debugfs_remove_recursive(dbgfs->status_dir);
	}

	kfree(dbgfs);
}
