/*
 * Device overlay manager
 *
 * Copyright (C) 2015 Konsulko Group
 * Pantelis Antoniou <pantelis.antoniou@konsulko.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */
#include <linux/ctype.h>
#include <linux/cpu.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/spinlock.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/configfs.h>
#include <linux/types.h>
#include <linux/stat.h>
#include <linux/limits.h>
#include <linux/file.h>
#include <linux/vmalloc.h>
#include <linux/firmware.h>
#include <linux/pci.h>
#include <linux/usb.h>
#include <linux/mod_devicetable.h>
#include <linux/workqueue.h>
#include <linux/firmware.h>

enum dovmgr_type {
	ITEM_PCI,
	ITEM_USB
};

struct dovmgr_item;

struct dovmgr_dev_item {
	struct dovmgr_item *item;
	struct list_head node;
	struct device *dev;
	const struct firmware *fw;
	struct device_node *overlay;
	int overlay_id;
	struct work_struct work;
};

struct dovmgr_item {
	struct config_item item;
	char *path;
	bool enable;
	char *overlay_name;
	struct mutex dev_item_mutex;
	struct list_head dev_item_list;
	enum dovmgr_type type;
	union {
		struct pci_device_id pci;
		struct usb_device_id usb;
	};
};

struct config_group dovmgr_pci_group;
struct config_group dovmgr_usb_group;

static inline struct dovmgr_item *to_dovmgr_item(struct config_item *cfsitem)
{
	if (!cfsitem)
		return NULL;

	return container_of(cfsitem, struct dovmgr_item, item);
}

static int dovmgr_notifier_action(struct config_group *group,
		unsigned long action, struct device *dev,
		int (*do_match)(struct dovmgr_item *item, struct device *dev),
		int (*do_action)(struct dovmgr_item *item, unsigned long action,
			struct device *dev))
{
	struct config_item *cfsitem;
	struct dovmgr_item *item;
	int ret;

	/* only handle device notifiers */
	if (action != BUS_NOTIFY_ADD_DEVICE &&
		action != BUS_NOTIFY_DEL_DEVICE &&
		action != BUS_NOTIFY_REMOVED_DEVICE)
		return 0;

	ret = 0;
	mutex_lock(&group->cg_subsys->su_mutex);
	list_for_each_entry(cfsitem, &group->cg_children, ci_entry) {
		item = to_dovmgr_item(cfsitem);
		if (!item->enable || !(*do_match)(item, dev))
			continue;

		ret = (*do_action)(item, action, dev);
		if (ret != 0)
			break;
	}
	mutex_unlock(&group->cg_subsys->su_mutex);
	return ret;
}

#if IS_ENABLED(CONFIG_PCI)
/* copy of drivers/pci/pci.h */
static inline const struct pci_device_id *
pci_match_one_device(const struct pci_device_id *id, const struct pci_dev *dev)
{
	if ((id->vendor == PCI_ANY_ID || id->vendor == dev->vendor) &&
	    (id->device == PCI_ANY_ID || id->device == dev->device) &&
	    (id->subvendor == PCI_ANY_ID ||
		id->subvendor == dev->subsystem_vendor) &&
	    (id->subdevice == PCI_ANY_ID ||
		id->subdevice == dev->subsystem_device) &&
	    !((id->class ^ dev->class) & id->class_mask))
		return id;
	return NULL;
}

static int dovmgr_pci_item_match(struct dovmgr_item *item, struct device *dev)
{
	struct pci_dev *pdev;

	BUG_ON(item->type != ITEM_PCI);
	pdev = to_pci_dev(dev);

	return pci_match_one_device(&item->pci, pdev) != NULL;
}
#endif

#if IS_ENABLED(CONFIG_USB)
/* in drivers/usb/core/driver.c */
extern int usb_match_device(struct usb_device *dev,
		const struct usb_device_id *id);

static int dovmgr_usb_item_match(struct dovmgr_item *item, struct device *dev)
{
	struct usb_device *udev;

	BUG_ON(item->type != ITEM_USB);
	udev = to_usb_device(dev);

	return usb_match_device(udev, &item->usb);
}
#endif

static struct dovmgr_dev_item *dovmgr_lookup_dev_item(struct dovmgr_item *item,
		struct device *dev)
{
	struct dovmgr_dev_item *ditem;

	list_for_each_entry(ditem, &item->dev_item_list, node)
		if (ditem->dev == dev)
			return ditem;
	return NULL;
}

static void dovmgr_item_work_func(struct work_struct *work)
{
	struct dovmgr_dev_item *ditem = container_of(work,
			struct dovmgr_dev_item, work);
	struct dovmgr_item *item = ditem->item;
	struct device *dev;
	struct device_node *np;
	int err;

	mutex_lock(&item->dev_item_mutex);

	dev = ditem->dev;
	np = dev->of_node;
	if (!dev || !np || !item->overlay_name || ditem->overlay_id >= 0)
		goto out_unlock;

	pr_info("%s: %s %s\n", __func__,
		kobject_name(&dev->kobj), of_node_full_name(np));

	err = request_firmware_direct(&ditem->fw, item->overlay_name, dev);
	if (err != 0) {
		pr_err("%s: %s failed to load firmware '%s'\n", __func__,
			kobject_name(&dev->kobj), item->overlay_name);
		goto out_unlock;
	}

	of_fdt_unflatten_tree((void *)ditem->fw->data, &ditem->overlay);
	if (ditem->overlay == NULL) {
		pr_err("%s: %s failed to load firmware '%s'\n", __func__,
			kobject_name(&dev->kobj), item->overlay_name);
		goto out_release_fw;
	}

	/* mark it as detached */
	of_node_set_flag(ditem->overlay, OF_DETACHED);

	/* perform resolution */
	err = of_resolve_phandles(ditem->overlay);
	if (err != 0) {
		pr_err("%s: %s failed to resolve tree\n", __func__,
			kobject_name(&dev->kobj));
		goto out_release_overlay;
	}

	err = of_overlay_create_target_root(ditem->overlay, np);
	if (err < 0) {
		pr_err("%s: %s failed to create overlay\n", __func__,
			kobject_name(&dev->kobj));
		goto out_release_overlay;
	}
	ditem->overlay_id = err;

out_unlock:
	mutex_unlock(&item->dev_item_mutex);
	return;

out_release_overlay:
	/* TODO: free the overlay, we can't right now cause
	 * the unflatten method does not track it */
	ditem->overlay = NULL;
out_release_fw:
	release_firmware(ditem->fw);
	ditem->fw = NULL;
	goto out_unlock;
}

/* dev item list mutex lock must be held */
static int dovmgr_add_dev_item(struct dovmgr_item *item, struct device *dev)
{
	struct dovmgr_dev_item *ditem;

	/* first make sure there's no duplicate */
	if (dovmgr_lookup_dev_item(item, dev))
		return -EEXIST;

	/* add the device item */
	ditem = kzalloc(sizeof(*ditem), GFP_KERNEL);
	if (!ditem)
		return -ENOMEM;
	ditem->overlay_id = -1;
	ditem->dev = get_device(dev);
	INIT_WORK(&ditem->work, dovmgr_item_work_func);
	ditem->item = item;

	list_add_tail(&ditem->node, &item->dev_item_list);

	pr_info("%s: added device %s from item's %s list\n", __func__,
			kobject_name(&dev->kobj),
			config_item_name(&item->item));

	/* now schedule the overlay application */
	if (item->overlay_name)
		schedule_work(&ditem->work);

	return 0;
}

static int dovmgr_remove_dev_item(struct dovmgr_item *item, struct device *dev)
{
	struct dovmgr_dev_item *ditem;

	/* find it */
	ditem = dovmgr_lookup_dev_item(item, dev);
	if (!ditem)
		return -ENODEV;

	if (work_pending(&ditem->work))
		cancel_work_sync(&ditem->work);

	if (ditem->overlay_id >= 0) {
		of_overlay_destroy(ditem->overlay_id);
		ditem->overlay_id = -1;

	}

	if (ditem->overlay) {
		/* TODO: free the overlay, we can't right now cause
		* the unflatten method does not track it */
		ditem->overlay = NULL;
	}

	if (ditem->fw) {
		/* TODO release_firmware(ditem->fw); */
		release_firmware(ditem->fw);
		ditem->fw = NULL;
	}

	put_device(ditem->dev);
	list_del(&ditem->node);

	kfree(ditem);

	pr_info("%s: removed device %s from item's %s list\n", __func__,
			kobject_name(&dev->kobj),
			config_item_name(&item->item));

	return 0;
}

static int dovmgr_item_notify(struct dovmgr_item *item,
		unsigned long action, struct device *dev)
{
	int ret;

	ret = 0;
	mutex_lock(&item->dev_item_mutex);

	switch (action) {
	case BUS_NOTIFY_ADD_DEVICE:
		pr_info("%s: BUS_NOTIFY_ADD_DEVICE for %s\n", __func__,
				kobject_name(&dev->kobj));

		ret = dovmgr_add_dev_item(item, dev);
		if (ret != 0)
			goto out_unlock;

		break;

	case BUS_NOTIFY_DEL_DEVICE:
		pr_info("%s: BUS_NOTIFY_DEL_DEVICE for %s\n", __func__,
				kobject_name(&dev->kobj));
		break;

	case BUS_NOTIFY_REMOVED_DEVICE:
		pr_info("%s: BUS_NOTIFY_REMOVE_DEVICE for %s\n", __func__,
				kobject_name(&dev->kobj));

		ret = dovmgr_remove_dev_item(item, dev);
		if (ret != 0)
			goto out_unlock;

		break;
	}

out_unlock:
	mutex_unlock(&item->dev_item_mutex);

	return ret;
}

#if IS_ENABLED(CONFIG_PCI)
static int dovmgr_pci_add_iterator(struct device *dev, void *data)
{
	struct dovmgr_item *item = data;

	/* do add match */
	if (!item->enable || !dovmgr_pci_item_match(item, dev))
		return 0;

	pr_info("%s: dev=%s\n", __func__, kobject_name(&dev->kobj));

	return dovmgr_item_notify(item, BUS_NOTIFY_ADD_DEVICE, dev);
}

static int dovmgr_pci_removed_iterator(struct device *dev, void *data)
{
	struct dovmgr_item *item = data;

	/* do add match */
	if (item->enable || !dovmgr_pci_item_match(item, dev))
		return 0;

	pr_info("%s: dev=%s\n", __func__, kobject_name(&dev->kobj));

	return dovmgr_item_notify(item, BUS_NOTIFY_REMOVED_DEVICE, dev);
}
#endif

#if IS_ENABLED(CONFIG_USB)
static int dovmgr_usb_add_iterator(struct device *dev, void *data)
{
	struct dovmgr_item *item = data;

	/* do add match */
	if (item->enable || !dovmgr_usb_item_match(item, dev))
		return 0;

	pr_info("%s: dev=%s\n", __func__, kobject_name(&dev->kobj));

	return dovmgr_item_notify(item, BUS_NOTIFY_ADD_DEVICE, dev);
}

static int dovmgr_usb_removed_iterator(struct device *dev, void *data)
{
	struct dovmgr_item *item = data;

	/* do add match */
	if (!item->enable || !dovmgr_usb_item_match(item, dev))
		return 0;

	pr_info("%s: dev=%s\n", __func__, kobject_name(&dev->kobj));

	return dovmgr_item_notify(item, BUS_NOTIFY_REMOVED_DEVICE, dev);
}
#endif

static int dovmgr_item_set_enable(struct dovmgr_item *item, bool new_enable)
{
	int ret;

	if (new_enable == item->enable)
		return 0;

	item->enable = new_enable;
	switch (item->type) {
#if IS_ENABLED(CONFIG_PCI)
	case ITEM_PCI:
		ret = bus_for_each_dev(&pci_bus_type, NULL, item,
			new_enable ? dovmgr_pci_add_iterator :
					dovmgr_pci_removed_iterator);
		if (ret != 0)
			return ret;
		break;
#endif
#if IS_ENABLED(CONFIG_USB)
	case ITEM_USB:
		ret = bus_for_each_dev(&usb_bus_type, NULL, item,
			new_enable ? dovmgr_usb_add_iterator :
					dovmgr_usb_removed_iterator);
		if (ret != 0)
			return ret;
		break;
#endif
	default:
		break;
	}
	return 0;
}


static ssize_t dovmgr_item_str_show(struct dovmgr_item *item,
		char *page, char **strp)
{
	return snprintf(page, PAGE_SIZE, "%s\n",
			*strp ? *strp : "");
}

static ssize_t dovmgr_item_str_store(struct dovmgr_item *item,
		const char *page, size_t count, char **strp)
{
	const char *s;
	int len;

	/* copy to path buffer (and make sure it's always zero terminated */
	len = strnlen(page, PAGE_SIZE);
	if (len >= PAGE_SIZE)
		return -EINVAL;
	s = page + len;
	while (len > 0 && *--s == '\n')
		len--;
	if (len == 0)
		return -EINVAL;

	if (*strp)
		kfree(*strp);
	*strp = kmalloc(len + 1, GFP_KERNEL);
	if (!*strp)
		return -ENOMEM;
	memcpy(*strp, page, len);
	(*strp)[len + 1] = '\0';

	return count;
}

static ssize_t dovmgr_item_path_show(struct config_item *citem, char *page)
{
	struct dovmgr_item *item = to_dovmgr_item(citem);
	return dovmgr_item_str_show(item, page, &item->path);
}

static ssize_t dovmgr_item_path_store(struct config_item *citem,
		const char *page, size_t count)
{
	struct dovmgr_item *item = to_dovmgr_item(citem);
	return dovmgr_item_str_store(item, page, count, &item->path);
}

static ssize_t dovmgr_item_enable_show(struct config_item *citem, char *page)
{
	struct dovmgr_item *item = to_dovmgr_item(citem);
	return snprintf(page, PAGE_SIZE, "%u\n", !!item->enable);
}

static ssize_t dovmgr_item_enable_store(struct config_item *citem,
		const char *page, size_t count)
{
	struct dovmgr_item *item = to_dovmgr_item(citem);
	int ret;
	unsigned int val;

	ret = kstrtouint(page, 0, &val);
	if (ret != 0)
		return ret;

	ret = dovmgr_item_set_enable(item, !!val);
	if (ret != 0)
		return ret;

	return count;
}

static ssize_t dovmgr_item_overlay_show(struct config_item *citem, char *page)
{
	struct dovmgr_item *item = to_dovmgr_item(citem);
	ssize_t ret;

	mutex_lock(&item->dev_item_mutex);
	ret = snprintf(page, PAGE_SIZE, "%s\n", item->overlay_name ?
			item->overlay_name : "");
	mutex_unlock(&item->dev_item_mutex);
	return ret;
};


static ssize_t dovmgr_item_overlay_store(struct config_item *citem,
		const char *page, size_t count)
{
	struct dovmgr_item *item = to_dovmgr_item(citem);
	ssize_t ret;

	mutex_lock(&item->dev_item_mutex);
	kfree(item->overlay_name);
	item->overlay_name = kstrndup(page, PAGE_SIZE, GFP_KERNEL);
	if (!item->overlay_name)
		ret = -ENOMEM;
	else
		ret = count;
	mutex_unlock(&item->dev_item_mutex);
	return ret;
}

static ssize_t dovmgr_item_status_show(struct config_item *citem, char *page)
{
	struct dovmgr_item *item = to_dovmgr_item(citem);
	struct dovmgr_dev_item *ditem;
	char *p, *e;
	int len;

	p = page;
	e = page + PAGE_SIZE;

	mutex_lock(&item->dev_item_mutex);
	list_for_each_entry(ditem, &item->dev_item_list, node) {
		len = snprintf(p, e - p, "%s:%s:%d\n",
			kobject_name(&ditem->dev->kobj),
			of_node_full_name(ditem->dev->of_node),
			ditem->overlay_id);
		p += len;
		if (p >= e - 1)
			break;
	}
	mutex_unlock(&item->dev_item_mutex);

	return p - page;
}

CONFIGFS_ATTR(dovmgr_item_, path);
CONFIGFS_ATTR_RO(dovmgr_item_, status);
CONFIGFS_ATTR(dovmgr_item_, enable);
CONFIGFS_ATTR(dovmgr_item_, overlay);

#if IS_ENABLED(CONFIG_PCI)
static ssize_t dovmgr_item_pci_device_show(struct config_item *citem,
		char *page)
{
	struct dovmgr_item *item = to_dovmgr_item(citem);
	return snprintf(page, PAGE_SIZE, "0x%04x\n", item->pci.device);
}

static ssize_t dovmgr_item_pci_device_store(struct config_item *citem,
		const char *page, size_t count)
{
	struct dovmgr_item *item = to_dovmgr_item(citem);
	int ret;
	unsigned int val;

	ret = kstrtouint(page, 0, &val);
	if (ret != 0)
		return ret;
	item->pci.device = val;
	return count;
}

static ssize_t dovmgr_item_pci_vendor_show(struct config_item *citem,
		char *page)
{
	struct dovmgr_item *item = to_dovmgr_item(citem);
	return snprintf(page, PAGE_SIZE, "0x%04x\n", item->pci.vendor);
}

static ssize_t dovmgr_item_pci_vendor_store(struct config_item *citem,
		const char *page, size_t count)
{
	struct dovmgr_item *item = to_dovmgr_item(citem);
	int ret;
	unsigned int val;

	/* cannot modify when item is enabled */
	if (item->enable)
		return -EBUSY;

	ret = kstrtouint(page, 0, &val);
	if (ret != 0)
		return ret;
	item->pci.vendor = val;
	return count;
}

static ssize_t dovmgr_item_pci_subdevice_show(struct config_item *citem,
		char *page)
{
	struct dovmgr_item *item = to_dovmgr_item(citem);
	return snprintf(page, PAGE_SIZE, "0x%08x\n", item->pci.subdevice);
}

static ssize_t dovmgr_item_pci_subdevice_store(struct config_item *citem,
		const char *page, size_t count)
{
	struct dovmgr_item *item = to_dovmgr_item(citem);
	int ret;
	unsigned int val;

	/* cannot modify when item is enabled */
	if (item->enable)
		return -EBUSY;

	ret = kstrtouint(page, 0, &val);
	if (ret != 0)
		return ret;
	item->pci.subdevice = val;
	return count;
}

static ssize_t dovmgr_item_pci_subvendor_show(struct config_item *citem,
		char *page)
{
	struct dovmgr_item *item = to_dovmgr_item(citem);
	return snprintf(page, PAGE_SIZE, "0x%08x\n", item->pci.subvendor);
}

static ssize_t dovmgr_item_pci_subvendor_store(struct config_item *citem,
		const char *page, size_t count)
{
	struct dovmgr_item *item = to_dovmgr_item(citem);
	int ret;
	unsigned int val;

	/* cannot modify when item is enabled */
	if (item->enable)
		return -EBUSY;

	ret = kstrtouint(page, 0, &val);
	if (ret != 0)
		return ret;
	item->pci.subvendor = val;
	return count;
}

static ssize_t dovmgr_item_pci_class_show(struct config_item *citem, char *page)
{
	struct dovmgr_item *item = to_dovmgr_item(citem);
	return snprintf(page, PAGE_SIZE, "0x%04x\n", item->pci.class);
}

static ssize_t dovmgr_item_pci_class_store(struct config_item *citem,
		const char *page, size_t count)
{
	struct dovmgr_item *item = to_dovmgr_item(citem);
	int ret;
	unsigned int val;

	/* cannot modify when item is enabled */
	if (item->enable)
		return -EBUSY;

	ret = kstrtouint(page, 0, &val);
	if (ret != 0)
		return ret;
	item->pci.class = val;
	return count;
}

static ssize_t dovmgr_item_pci_class_mask_show(struct config_item *citem,
		char *page)
{
	struct dovmgr_item *item = to_dovmgr_item(citem);
	return snprintf(page, PAGE_SIZE, "0x%04x\n", item->pci.class_mask);
}

static ssize_t dovmgr_item_pci_class_mask_store(struct config_item *citem,
		const char *page, size_t count)
{
	struct dovmgr_item *item = to_dovmgr_item(citem);
	int ret;
	unsigned int val;

	/* cannot modify when item is enabled */
	if (item->enable)
		return -EBUSY;

	ret = kstrtouint(page, 0, &val);
	if (ret != 0)
		return ret;
	item->pci.class_mask = val;
	return count;
}

CONFIGFS_ATTR(dovmgr_item_pci_, device);
CONFIGFS_ATTR(dovmgr_item_pci_, vendor);
CONFIGFS_ATTR(dovmgr_item_pci_, subdevice);
CONFIGFS_ATTR(dovmgr_item_pci_, subvendor);
CONFIGFS_ATTR(dovmgr_item_pci_, class);
CONFIGFS_ATTR(dovmgr_item_pci_, class_mask);
#endif

#if IS_ENABLED(CONFIG_USB)
static ssize_t dovmgr_item_usb_idProduct_show(struct config_item *citem,
		char *page)
{
	struct dovmgr_item *item = to_dovmgr_item(citem);
	return snprintf(page, PAGE_SIZE, "0x%04x\n",
			item->usb.idProduct);
}

static ssize_t dovmgr_item_usb_idProduct_store(struct config_item *citem,
		const char *page, size_t count)
{
	struct dovmgr_item *item = to_dovmgr_item(citem);
	int ret;
	unsigned int val;

	/* cannot modify when item is enabled */
	if (item->enable)
		return -EBUSY;

	ret = kstrtouint(page, 0, &val);
	if (ret != 0)
		return ret;
	item->usb.idProduct = val;
	return count;
}

static ssize_t dovmgr_item_usb_idVendor_show(struct config_item *citem,
		char *page)
{
	struct dovmgr_item *item = to_dovmgr_item(citem);
	return snprintf(page, PAGE_SIZE, "0x%04x\n",
			item->usb.idVendor);
}

static ssize_t dovmgr_item_usb_idVendor_store(struct config_item *citem,
		const char *page, size_t count)
{
	struct dovmgr_item *item = to_dovmgr_item(citem);
	int ret;
	unsigned int val;

	/* cannot modify when item is enabled */
	if (item->enable)
		return -EBUSY;

	ret = kstrtouint(page, 0, &val);
	if (ret != 0)
		return ret;
	item->usb.idVendor = val;
	return count;
}

CONFIGFS_ATTR(dovmgr_item_usb_, idProduct);
CONFIGFS_ATTR(dovmgr_item_usb_, idVendor);
#endif

#if IS_ENABLED(CONFIG_PCI)
static struct configfs_attribute *dovmgr_pci_attrs[] = {
	&dovmgr_item_attr_path,
	&dovmgr_item_attr_status,
	&dovmgr_item_attr_enable,
	&dovmgr_item_attr_overlay,
	&dovmgr_item_pci_attr_device,
	&dovmgr_item_pci_attr_vendor,
	&dovmgr_item_pci_attr_subdevice,
	&dovmgr_item_pci_attr_subvendor,
	&dovmgr_item_pci_attr_class,
	&dovmgr_item_pci_attr_class_mask,
	NULL,
};
#endif

#if IS_ENABLED(CONFIG_USB)
static struct configfs_attribute *dovmgr_usb_attrs[] = {
	&dovmgr_item_attr_path,
	&dovmgr_item_attr_enable,
	&dovmgr_item_attr_status,
	&dovmgr_item_attr_overlay,
	&dovmgr_item_usb_attr_idVendor,
	&dovmgr_item_usb_attr_idProduct,
	NULL,
};
#endif

static void dovmgr_release(struct config_item *cfsitem)
{
	struct dovmgr_item *item = to_dovmgr_item(cfsitem);

	/* disable item (this removes the overlay and all) */
	dovmgr_item_set_enable(item, false);

	kfree(item->path);
	kfree(item);
}

static struct configfs_item_operations dovmgr_item_ops = {
	.release		= dovmgr_release,
};

#if IS_ENABLED(CONFIG_PCI)
static struct config_item_type dovmgr_pci_item_type = {
	.ct_item_ops	= &dovmgr_item_ops,
	.ct_attrs	= dovmgr_pci_attrs,
	.ct_owner	= THIS_MODULE,
};
#endif

#if IS_ENABLED(CONFIG_USB)
static struct config_item_type dovmgr_usb_item_type = {
	.ct_item_ops	= &dovmgr_item_ops,
	.ct_attrs	= dovmgr_usb_attrs,
	.ct_owner	= THIS_MODULE,
};
#endif

static struct config_item *dovmgr_group_make_item(
		struct config_group *group, const char *name,
		enum dovmgr_type type)
{
	struct dovmgr_item *item;
	struct config_item_type *item_type;

	switch (type) {
#if IS_ENABLED(CONFIG_PCI)
	case ITEM_PCI:
		item_type = &dovmgr_pci_item_type;
		break;
#endif
#if IS_ENABLED(CONFIG_USB)
	case ITEM_USB:
		item_type = &dovmgr_usb_item_type;
		break;
#endif
	default:
		return ERR_PTR(-EINVAL);
	};

	item = kzalloc(sizeof(*item), GFP_KERNEL);
	if (!item)
		return ERR_PTR(-ENOMEM);

	item->type = type;
	item->enable = false;
	mutex_init(&item->dev_item_mutex);
	INIT_LIST_HEAD(&item->dev_item_list);

	switch (type) {
#if IS_ENABLED(CONFIG_PCI)
	case ITEM_PCI:
		/* default for matching device/vendor */
		item->pci.vendor = PCI_ANY_ID;
		item->pci.device = PCI_ANY_ID;
		item->pci.subvendor = PCI_ANY_ID;
		item->pci.subdevice = PCI_ANY_ID;
		item->pci.class = 0;
		item->pci.class_mask = 0;
		break;
#endif
#if IS_ENABLED(CONFIG_USB)
	case ITEM_USB:
		/* default */
		item->usb.match_flags = USB_DEVICE_ID_MATCH_DEVICE;
		break;
#endif
	default:
		return ERR_PTR(-EINVAL);
	};

	config_item_init_type_name(&item->item, name, item_type);
	return &item->item;
}

#if IS_ENABLED(CONFIG_PCI)
static struct config_item *dovmgr_group_pci_make_item(
		struct config_group *group, const char *name)
{
	return dovmgr_group_make_item(group, name, ITEM_PCI);
}
#endif

#if IS_ENABLED(CONFIG_USB)
static struct config_item *dovmgr_group_usb_make_item(
		struct config_group *group, const char *name)
{
	return dovmgr_group_make_item(group, name, ITEM_USB);
}
#endif

static void dovmgr_group_drop_item(struct config_group *group,
		struct config_item *cfsitem)
{
	struct dovmgr_item *item = to_dovmgr_item(cfsitem);

	switch (item->type) {
#if IS_ENABLED(CONFIG_PCI)
	case ITEM_PCI:
		break;
#endif
#if IS_ENABLED(CONFIG_USB)
	case ITEM_USB:
		break;
#endif
	default:
		break;
	}
	config_item_put(&item->item);
}

#if IS_ENABLED(CONFIG_PCI)
static struct configfs_group_operations dovmgr_pci_group_ops = {
	.make_item	= dovmgr_group_pci_make_item,
	.drop_item	= dovmgr_group_drop_item,
};

static struct config_item_type dovmgr_pci_type = {
	.ct_group_ops   = &dovmgr_pci_group_ops,
	.ct_owner       = THIS_MODULE,
};
#endif

#if IS_ENABLED(CONFIG_USB)
static struct configfs_group_operations dovmgr_usb_group_ops = {
	.make_item	= dovmgr_group_usb_make_item,
	.drop_item	= dovmgr_group_drop_item,
};

static struct config_item_type dovmgr_usb_type = {
	.ct_group_ops   = &dovmgr_usb_group_ops,
	.ct_owner       = THIS_MODULE,
};
#endif

static struct configfs_group_operations dovmgr_ops = {
	/* empty - we don't allow anything to be created */
};

static struct config_item_type dovmgr_type = {
	.ct_group_ops   = &dovmgr_ops,
	.ct_owner       = THIS_MODULE,
};

struct config_group *dovmgr_def_groups[] = {
#if IS_ENABLED(CONFIG_PCI)
	&dovmgr_pci_group,
#endif
#if IS_ENABLED(CONFIG_USB)
	&dovmgr_usb_group,
#endif
	NULL
};

static struct configfs_subsystem dovmgr_subsys = {
	.su_group = {
		.cg_item = {
			.ci_namebuf = "dovmgr",
			.ci_type = &dovmgr_type,
		},
		.default_groups = dovmgr_def_groups,
	},
	.su_mutex = __MUTEX_INITIALIZER(dovmgr_subsys.su_mutex),
};

#if IS_ENABLED(CONFIG_PCI)
static int pci_dev_instantiate(struct pci_dev *pdev)
{
	struct device *dev = &pdev->dev;
	struct device *bus_dev;
	struct of_changeset cset;
	struct device_node *np, *npb;
	int ret;

	npb = NULL;

	/* already instantiated */
	if (dev->of_node) {
		pr_debug("%s: dev=%s of_node=%s\n", __func__,
			kobject_name(&dev->kobj),
			of_node_full_name(dev->of_node));
		return 0;
	}

	bus_dev = &pdev->bus->dev;

	pr_debug("%s: %s: %02x:%02x.%02x - node %s%s\n", __func__,
			kobject_name(&dev->kobj),
			pdev->bus->number, PCI_SLOT(pdev->devfn), PCI_FUNC(pdev->devfn),
			bus_dev->of_node ? of_node_full_name(bus_dev->of_node) : "<NULL>",
			pci_is_bridge(pdev) ? " bridge" : "");

	/* to create the node, the bus must be present */
	if (!bus_dev->of_node) {
		pr_err("%s: No node for %s because no bus device node\n",
			__func__, kobject_name(&dev->kobj));
		return 0;
	}

	of_changeset_init(&cset);

	np = of_changeset_create_device_node(&cset, bus_dev->of_node,
		"%s/pci-%04x-%02x-%02x.%d",
		of_node_full_name(bus_dev->of_node),
		pci_domain_nr(pdev->bus), pdev->bus->number,
		PCI_SLOT(pdev->devfn), PCI_FUNC(pdev->devfn));
	if (IS_ERR(np)) {
		ret = PTR_ERR(np);
		goto out_cset_fail;
	}

	ret = of_changeset_add_property_stringf(&cset, np, "compatible",
			"pciclass,%04x", (pdev->class >> 8) & 0xffffff);
	if (ret != 0)
		goto out_cset_fail;

	ret = of_changeset_add_property_u32(&cset, np, "vendor",
			pdev->vendor);
	if (ret != 0)
		goto out_cset_fail;

	ret = of_changeset_add_property_u32(&cset, np, "device",
			pdev->device);
	if (ret != 0)
		goto out_cset_fail;

	ret = of_changeset_add_property_string(&cset, np, "status", "okay");
	if (ret != 0)
		goto out_cset_fail;

	ret = of_changeset_add_property_bool(&cset, np, "auto-generated");
	if (ret != 0)
		goto out_cset_fail;

	ret = of_changeset_attach_node(&cset, np);
	if (ret != 0)
		goto out_cset_fail;

	/* are we creating a bridge; swell */
	npb = NULL;
	if (pci_is_bridge(pdev) && !pdev->subordinate->dev.of_node) {

		pr_debug("%s: %s: bus->dev=%s subordinate=%s\n", __func__,
			kobject_name(&dev->kobj),
			kobject_name(&pdev->bus->dev.kobj),
			kobject_name(&pdev->subordinate->dev.kobj));

		npb = of_changeset_create_device_node(&cset, bus_dev->of_node,
			"%s/pci-%04x-%02x",
			of_node_full_name(bus_dev->of_node),
			pci_domain_nr(pdev->subordinate),
			pdev->subordinate->number);
		if (IS_ERR(npb)) {
			ret = PTR_ERR(npb);
			goto out_cset_fail;
		}

		ret = of_changeset_add_property_string(&cset, npb, "compatible", "generic,pci-bus");
		if (ret != 0)
			goto out_cset_fail;

		ret = of_changeset_add_property_string(&cset, npb, "device_type", "pci");
		if (ret != 0)
			goto out_cset_fail;

		ret = of_changeset_add_property_string(&cset, npb, "status", "okay");
		if (ret != 0)
			goto out_cset_fail;

		ret = of_changeset_add_property_bool(&cset, npb, "auto-generated");
		if (ret != 0)
			goto out_cset_fail;

		ret = of_changeset_attach_node(&cset, npb);
		if (ret != 0)
			goto out_cset_fail;
	}

	ret = __of_changeset_apply(&cset);
	if (ret != 0)
		goto out_cset_fail;

	/* permanently commit */
	of_changeset_destroy(&cset);

	/* bind the node to the device */
	dev->of_node = np;
	ret = sysfs_create_link(&dev->kobj, &dev->of_node->kobj,
			"of_node");
	if (ret)
		pr_warn("%s: %s Error %d creating of_node link\n",
				__func__, kobject_name(&dev->kobj), ret);

	if (npb) {
		pdev->subordinate->dev.of_node = npb;
		ret = sysfs_create_link(&pdev->subordinate->dev.kobj, &npb->kobj,
				"of_node");
		if (ret)
			pr_warn("%s: %s Error %d creating of_node link\n",
					__func__, kobject_name(&dev->kobj), ret);
	}


	return 0;

out_cset_fail:
	pr_err("%s: %s Failed to apply changeset (err=%d)\n", __func__,
		kobject_name(&dev->kobj), ret);
	of_changeset_destroy(&cset);
	return ret;
}

static int pci_dev_uninstantiate(struct pci_dev *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np, *npb;
	struct of_changeset cset;
	int ret;

	/* device node must exist */
	np = dev->of_node;
	if (!np)
		return 0;

	/* and the auto-generated property */
	if (!of_property_read_bool(np, "auto-generated"))
		return 0;

	of_changeset_init(&cset);

	ret = of_changeset_detach_node(&cset, np);
	if (ret != 0)
		goto out_cset_fail;

	npb = NULL;
	if (pci_is_bridge(pdev))
		npb = pdev->subordinate->dev.of_node;

	if (npb != NULL) {
		ret = of_changeset_detach_node(&cset, npb);
		if (ret != 0)
			goto out_cset_fail;
	}

	ret = __of_changeset_apply(&cset);
	if (ret != 0)
		goto out_cset_fail;

	dev->of_node = NULL;
	if (npb != NULL)
		pdev->subordinate->dev.of_node = NULL;

	pr_debug("%s: %s: %02x:%02x.%02x\n", __func__,
			kobject_name(&dev->kobj),
			pdev->bus->number, PCI_SLOT(pdev->devfn), PCI_FUNC(pdev->devfn));

	/* TODO iterate over the properties and free */

	return 0;

out_cset_fail:
	of_changeset_destroy(&cset);

	return ret;
}

static int dovmgr_pci_notify(struct notifier_block *nb,
				unsigned long action, void *arg)
{
	int ret;

	if (action == BUS_NOTIFY_ADD_DEVICE)
		pci_dev_instantiate(to_pci_dev(arg));

	ret = dovmgr_notifier_action(&dovmgr_pci_group, action, arg,
			dovmgr_pci_item_match, dovmgr_item_notify);

	if (action == BUS_NOTIFY_REMOVED_DEVICE)
		pci_dev_uninstantiate(to_pci_dev(arg));

	return ret;
}

static struct notifier_block dovmgr_pci_notifier = {
	.notifier_call = dovmgr_pci_notify,
};

static int pci_instantiate_iterator(struct device *dev, void *data)
{
	return pci_dev_instantiate(to_pci_dev(dev));
}

static int dovmgr_pci_init(void)
{
	int ret;

	config_group_init_type_name(&dovmgr_pci_group, "pci", &dovmgr_pci_type);
	ret = bus_register_notifier(&pci_bus_type, &dovmgr_pci_notifier);
	if (ret != 0) {
		pr_err("%s: bus_register_notifier() failed\n", __func__);
		return ret;
	}

	ret = bus_for_each_dev(&pci_bus_type, NULL, NULL,
			pci_instantiate_iterator);
	if (ret != 0) {
		pr_err("%s: bus_for_each_dev() failed\n", __func__);
		return ret;
	}

	return 0;
}

static void dovmgr_pci_cleanup(void)
{
	bus_unregister_notifier(&pci_bus_type, &dovmgr_pci_notifier);
}
#endif

#if IS_ENABLED(CONFIG_USB)
static int dovmgr_usb_notify(struct notifier_block *nb,
				unsigned long action, void *arg)
{
	return dovmgr_notifier_action(&dovmgr_usb_group, action, arg,
			dovmgr_usb_item_match, dovmgr_item_notify);
}

static struct notifier_block dovmgr_usb_notifier = {
	.notifier_call = dovmgr_usb_notify,
};

static int dovmgr_usb_init(void)
{
	int ret;

	config_group_init_type_name(&dovmgr_usb_group, "usb", &dovmgr_usb_type);
	ret = bus_register_notifier(&usb_bus_type, &dovmgr_usb_notifier);
	if (ret != 0) {
		pr_err("%s: bus_register_notifier() failed\n", __func__);
		return ret;
	}
	return 0;
}

static void dovmgr_usb_cleanup(void)
{
	bus_unregister_notifier(&usb_bus_type, &dovmgr_usb_notifier);
}
#endif

static int __init dovmgr_init(void)
{
	int ret;

	config_group_init(&dovmgr_subsys.su_group);

#if IS_ENABLED(CONFIG_PCI)
	ret = dovmgr_pci_init();
	if (ret != 0)
		goto err_no_pci_init;
#endif
#if IS_ENABLED(CONFIG_USB)
	ret = dovmgr_usb_init();
	if (ret != 0)
		goto err_no_usb_init;
#endif

	ret = configfs_register_subsystem(&dovmgr_subsys);
	if (ret != 0) {
		pr_err("%s: failed to register subsys\n", __func__);
		goto err_no_configfs;
	}
	pr_info("%s: OK\n", __func__);
	return 0;

err_no_configfs:
#if IS_ENABLED(CONFIG_USB)
	dovmgr_usb_cleanup();
err_no_usb_init:
#endif
#if IS_ENABLED(CONFIG_PCI)
	dovmgr_pci_cleanup();
err_no_pci_init:
#endif
	return ret;
}
late_initcall(dovmgr_init);
