// SPDX-License-Identifier: GPL-2.0-only
/*
 * This file is part of wlcore
 *
 * Copyright (C) 2013 Texas Instruments Inc.
 */

#include "acx.h"
#include "wlcore.h"
#include "debug.h"
#include "sysfs.h"


static ssize_t cc33xx_sysfs_read_fwlog(struct file *filp, struct kobject *kobj,
				       struct bin_attribute *bin_attr,
				       char *buffer, loff_t pos, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct cc33xx *wl = dev_get_drvdata(dev);
	ssize_t len;
	int ret;

	ret = mutex_lock_interruptible(&wl->mutex);
	if (ret < 0)
		return -ERESTARTSYS;

	/* Check if the fwlog is still valid */
	if (wl->fwlog_size < 0) {
		mutex_unlock(&wl->mutex);
		return 0;
	}

	/* Seeking is not supported - old logs are not kept. Disregard pos. */
	len = min_t(size_t, count, wl->fwlog_size);
	wl->fwlog_size -= len;
	memcpy(buffer, wl->fwlog, len);

	/* Make room for new messages */
	memmove(wl->fwlog, wl->fwlog + len, wl->fwlog_size);

	mutex_unlock(&wl->mutex);

	return len;
}

static const struct bin_attribute fwlog_attr = {
	.attr = { .name = "fwlog", .mode = 0400 },
	.read = cc33xx_sysfs_read_fwlog,
};

int wlcore_sysfs_init(struct cc33xx *wl)
{
	int ret;


	/* Create sysfs file for the FW log */
	ret = device_create_bin_file(wl->dev, &fwlog_attr);
	if (ret < 0) {
		cc33xx_error("failed to create sysfs file fwlog");
	}

	return ret;
}

void wlcore_sysfs_free(struct cc33xx *wl)
{
	device_remove_bin_file(wl->dev, &fwlog_attr);
}
