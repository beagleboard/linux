/*
 * This file is part of hci_h4p bluetooth driver
 *
 * Copyright (C) 2005, 2006 Nokia Corporation.
 *
 * Contact: Ville Tervo <ville.tervo@nokia.com>
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#include "hci_h4p.h"

#ifdef CONFIG_SYSFS

static ssize_t hci_h4p_store_bdaddr(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct hci_h4p_info *info = (struct hci_h4p_info*)dev_get_drvdata(dev);
	unsigned int bdaddr[6];
	int ret, i;

	ret = sscanf(buf, "%2x:%2x:%2x:%2x:%2x:%2x\n",
			&bdaddr[0], &bdaddr[1], &bdaddr[2],
			&bdaddr[3], &bdaddr[4], &bdaddr[5]);

	if (ret != 6) {
		return -EINVAL;
	}

	for (i = 0; i < 6; i++)
		info->bdaddr[i] = bdaddr[i] & 0xff;

	return count;
}

static ssize_t hci_h4p_show_bdaddr(struct device *dev, struct device_attribute *attr,
				   char *buf)
{
	struct hci_h4p_info *info = (struct hci_h4p_info*)dev_get_drvdata(dev);

	return sprintf(buf, "%.2x:%.2x:%.2x:%.2x:%.2x:%.2x\n",
		       info->bdaddr[0],
		       info->bdaddr[1],
		       info->bdaddr[2],
		       info->bdaddr[3],
		       info->bdaddr[4],
		       info->bdaddr[5]);
}

static DEVICE_ATTR(bdaddr, S_IRUGO | S_IWUSR, hci_h4p_show_bdaddr, hci_h4p_store_bdaddr);
int hci_h4p_sysfs_create_files(struct device *dev)
{
	return device_create_file(dev, &dev_attr_bdaddr);
}

#endif
