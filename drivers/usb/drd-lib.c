/**
 * drd-lib.c - USB DRD library functions
 *
 * Copyright (C) 2014 Texas Instruments
 * Author: George Cherian <george.cherian@ti.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2  of
 * the License as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/usb.h>

#include <linux/usb/hcd.h>
#include <linux/usb/gadget.h>
#include <linux/usb/drd.h>

/**
 * struct usb_drd - describes one dual role device
 * @host - the HOST controller device of this drd
 * @gadget - the gadget of drd
 * @parent - the device to the actual controller
 * @list - for use by the drd lib
 * @state - specifies the current state
 *
 * This represents the internal data structure which is used by the UDC-class
 * to hold information about udc driver and gadget together.
*/
struct usb_drd {
	struct usb_drd_host	*host;
	struct usb_drd_gadget	*gadget;
	struct device		*parent;
	struct list_head	list;
	unsigned int		state;
};

static LIST_HEAD(drd_list);
static DEFINE_SPINLOCK(drd_lock);

static struct usb_drd *usb_drd_get_dev(struct device *parent)
{
	struct usb_drd *drd;

	spin_lock(&drd_lock);
	list_for_each_entry(drd, &drd_list, list)
		if (drd->parent == parent)
			goto out;
	drd = NULL;
out:
	spin_unlock(&drd_lock);

	return drd;
}

int usb_drd_get_state(struct device *parent)
{
	struct usb_drd	*drd;

	drd = usb_drd_get_dev(parent);
	if (!drd)
		return -ENODEV;

	return drd->state;
}
EXPORT_SYMBOL_GPL(usb_drd_get_state);

int usb_drd_release(struct device *parent)
{
	struct usb_drd	*drd;
	int ret;

	spin_lock(&drd_lock);
	list_for_each_entry(drd, &drd_list, list) {
		if (drd->parent == parent) {
			kfree(drd);
			ret = 0;
			goto out;
		}
	}
	ret = -ENODEV;
out:
	spin_unlock(&drd_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(usb_drd_release);

int usb_drd_add(struct device *parent)
{
	struct usb_drd	*drd;

	drd = kzalloc(sizeof(*drd), GFP_KERNEL);
	if (!drd)
		return -ENOMEM;

	spin_lock(&drd_lock);
	drd->parent = parent;
	list_add_tail(&drd->list, &drd_list);
	drd->state = DRD_UNREGISTERED;

	spin_unlock(&drd_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(usb_drd_add);

int usb_drd_register_hcd(struct device *parent, struct usb_drd_host *host)
{
	struct usb_drd	*drd;

	drd = usb_drd_get_dev(parent);
	if (!drd)
		return -ENODEV;

	spin_lock(&drd_lock);
	drd->host = host;
	drd->state |= DRD_HOST_REGISTERED | DRD_HOST_ACTIVE;
	spin_unlock(&drd_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(usb_drd_register_hcd);

int usb_drd_unregister_hcd(struct device *parent)
{
	struct usb_drd	*drd;

	drd = usb_drd_get_dev(parent);
	if (!drd)
		return -ENODEV;

	spin_lock(&drd_lock);
	drd->state &= ~(DRD_HOST_REGISTERED | DRD_HOST_ACTIVE);
	spin_unlock(&drd_lock);
	kfree(drd->host);

	return 0;
}
EXPORT_SYMBOL_GPL(usb_drd_unregister_hcd);

int usb_drd_start_hcd(struct device *parent)
{
	struct usb_drd	*drd;
	struct usb_drd_setup *setup;

	drd = usb_drd_get_dev(parent);
	if (!drd)
		return -ENODEV;

	if (WARN_ON(!(drd->state & DRD_HOST_REGISTERED)))
		return -EINVAL;

	setup = drd->host->host_setup;
	if (setup && setup->ll_start)
		setup->ll_start(setup->data);

	usb_add_hcd(drd->host->main_hcd,
		    drd->host->hcd_irq, IRQF_SHARED);
	if (drd->host->shared_hcd)
		usb_add_hcd(drd->host->shared_hcd,
			    drd->host->hcd_irq, IRQF_SHARED);

	spin_lock(&drd_lock);
	drd->state |= DRD_HOST_ACTIVE;
	spin_unlock(&drd_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(usb_drd_start_hcd);

int usb_drd_stop_hcd(struct device *parent)
{
	struct usb_drd	*drd;
	struct usb_drd_setup *setup;

	drd = usb_drd_get_dev(parent);
	if (!drd)
		return -ENODEV;

	if (WARN_ON(!(drd->state & DRD_HOST_ACTIVE)))
		return -EINVAL;

	setup = drd->host->host_setup;
	if (setup && setup->ll_stop)
		setup->ll_stop(setup->data);
	if (drd->host->shared_hcd)
		usb_remove_hcd(drd->host->shared_hcd);

	usb_remove_hcd(drd->host->main_hcd);

	spin_lock(&drd_lock);
	drd->state = drd->state & ~DRD_HOST_ACTIVE;
	spin_unlock(&drd_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(usb_drd_stop_hcd);

int usb_drd_register_udc(struct device *parent, struct usb_drd_gadget *gadget)
{
	struct usb_drd	*drd;

	drd = usb_drd_get_dev(parent);
	if (!drd)
		return -ENODEV;

	spin_lock(&drd_lock);
	drd->gadget = gadget;
	drd->state |= DRD_DEVICE_REGISTERED;
	if (drd->gadget->g_driver)
		drd->state |= DRD_DEVICE_ACTIVE;

	spin_unlock(&drd_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(usb_drd_register_udc);

int usb_drd_register_udc_driver(struct device *parent,
				struct usb_gadget_driver *driver)
{
	struct usb_drd	*drd;

	drd = usb_drd_get_dev(parent);
	if (!drd)
		return -ENODEV;

	spin_lock(&drd_lock);
	drd->gadget->g_driver = driver;
	drd->state |= DRD_DEVICE_ACTIVE;
	spin_unlock(&drd_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(usb_drd_register_udc_driver);

int usb_drd_unregister_udc(struct device *parent)
{
	struct usb_drd	*drd;

	drd = usb_drd_get_dev(parent);
	if (!drd)
		return -ENODEV;

	spin_lock(&drd_lock);
	drd->state &= ~(DRD_DEVICE_REGISTERED | DRD_DEVICE_ACTIVE);
	spin_unlock(&drd_lock);
	kfree(drd->gadget->gadget_setup);
	kfree(drd->gadget);

	return 0;
}
EXPORT_SYMBOL_GPL(usb_drd_unregister_udc);

int usb_drd_unregister_udc_driver(struct device *parent)
{
	struct usb_drd	*drd;
	struct usb_drd_gadget *drd_gadget;

	drd = usb_drd_get_dev(parent);
	if (!drd)
		return -ENODEV;
	drd_gadget = drd->gadget;

	spin_lock(&drd_lock);
	drd->state &= ~DRD_DEVICE_ACTIVE;
	drd_gadget->g_driver = NULL;
	spin_unlock(&drd_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(usb_drd_unregister_udc_driver);

int usb_drd_start_udc(struct device *parent)
{
	struct usb_drd	*drd;
	struct usb_drd_gadget *drd_gadget;
	struct usb_drd_setup *setup;

	drd = usb_drd_get_dev(parent);
	if (!drd)
		return -ENODEV;

	if (WARN_ON(!(drd->state & DRD_DEVICE_REGISTERED)))
		return -EINVAL;

	drd_gadget = drd->gadget;
	setup = drd_gadget->gadget_setup;

	if (setup && setup->ll_start)
		setup->ll_start(setup->data);

	usb_add_gadget_udc_release(parent, drd_gadget->gadget,
				   setup->ll_release);
	spin_lock(&drd_lock);
	drd->state |= DRD_DEVICE_ACTIVE;
	spin_unlock(&drd_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(usb_drd_start_udc);

int usb_drd_stop_udc(struct device *parent)
{
	struct usb_drd	*drd;
	struct usb_drd_gadget *drd_gadget;
	struct usb_drd_setup *setup;

	drd = usb_drd_get_dev(parent);
	if (!drd)
		return -ENODEV;

	if (WARN_ON(!(drd->state & DRD_DEVICE_REGISTERED)))
		return -EINVAL;

	drd_gadget = drd->gadget;
	setup = drd_gadget->gadget_setup;
	if (setup && setup->ll_stop)
		setup->ll_stop(setup->data);

	usb_del_gadget_udc(drd_gadget->gadget);

	spin_lock(&drd_lock);
	drd->state = drd->state & ~DRD_DEVICE_ACTIVE;
	spin_unlock(&drd_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(usb_drd_stop_udc);

MODULE_DESCRIPTION("USB-DRD Library");
MODULE_AUTHOR("George Cherian <george.cherian@ti.com>");
MODULE_LICENSE("GPL v2");
