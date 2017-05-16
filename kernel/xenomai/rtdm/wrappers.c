/*
 * Copyright (c) 2013  Hauke Mehrtens <hauke@hauke-m.de>
 * Copyright (c) 2013  Hannes Frederic Sowa <hannes@stressinduktion.org>
 * Copyright (c) 2014  Luis R. Rodriguez <mcgrof@do-not-panic.com>
 *
 * Backport functionality introduced in Linux 3.13.
 *
 * Copyright (c) 2014  Hauke Mehrtens <hauke@hauke-m.de>
 *
 * Backport functionality introduced in Linux 3.14.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/hwmon.h>
#include <asm/xenomai/wrappers.h>

/*
 * Same rules as kernel/cobalt/include/asm-generic/xenomai/wrappers.h
 * apply to reduce #ifdefery.
 */

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,14,0)
#ifdef CONFIG_PCI_MSI
int pci_enable_msix_range(struct pci_dev *dev,
			struct msix_entry *entries,
			int minvec, int maxvec)
{
	int nvec = maxvec;
	int rc;

	if (maxvec < minvec)
		return -ERANGE;

	do {
		rc = pci_enable_msix(dev, entries, nvec);
		if (rc < 0) {
			return rc;
		} else if (rc > 0) {
			if (rc < minvec)
				return -ENOSPC;
			nvec = rc;
		}
	} while (rc);

	return nvec;
}
EXPORT_SYMBOL(pci_enable_msix_range);
#endif
#endif /* < 3.14 */

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,13,0)
#ifdef CONFIG_HWMON
struct device*
hwmon_device_register_with_groups(struct device *dev, const char *name,
				void *drvdata,
				const struct attribute_group **groups)
{
	struct device *hwdev;

	hwdev = hwmon_device_register(dev);
	hwdev->groups = groups;
	dev_set_drvdata(hwdev, drvdata);
	return hwdev;
}

static void devm_hwmon_release(struct device *dev, void *res)
{
	struct device *hwdev = *(struct device **)res;

	hwmon_device_unregister(hwdev);
}

struct device *
devm_hwmon_device_register_with_groups(struct device *dev, const char *name,
				void *drvdata,
				const struct attribute_group **groups)
{
	struct device **ptr, *hwdev;

	if (!dev)
		return ERR_PTR(-EINVAL);

	ptr = devres_alloc(devm_hwmon_release, sizeof(*ptr), GFP_KERNEL);
	if (!ptr)
		return ERR_PTR(-ENOMEM);

	hwdev = hwmon_device_register_with_groups(dev, name, drvdata, groups);
	if (IS_ERR(hwdev))
		goto error;

	*ptr = hwdev;
	devres_add(dev, ptr);
	return hwdev;

error:
	devres_free(ptr);
	return hwdev;
}
EXPORT_SYMBOL_GPL(devm_hwmon_device_register_with_groups);
#endif
#endif /* < 3.13 */
