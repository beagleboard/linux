// SPDX-License-Identifier: GPL-2.0
/*
 * Cadence USBSS DRD Driver.
 *
 * Copyright (C) 2018-2019 Cadence.
 * Copyright (C) 2017-2018 NXP
 *
 * Author: Peter Chen <peter.chen@nxp.com>
 *         Pawel Laszczak <pawell@cadence.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/pm_runtime.h>

#include "gadget.h"
#include "core.h"
#include "host-export.h"
#include "gadget-export.h"
#include "drd.h"
#include "debug.h"

/**
 * cdns3_handshake - spin reading  until handshake completes or fails
 * @ptr: address of device controller register to be read
 * @mask: bits to look at in result of read
 * @done: value of those bits when handshake succeeds
 * @usec: timeout in microseconds
 *
 * Returns negative errno, or zero on success
 *
 * Success happens when the "mask" bits have the specified value (hardware
 * handshake done). There are two failure modes: "usec" have passed (major
 * hardware flakeout), or the register reads as all-ones (hardware removed).
 */
int cdns3_handshake(void __iomem *ptr, u32 mask, u32 done, int usec)
{
	u32 result;

	do {
		result = readl(ptr);
		if (result == ~(u32)0)  /* card removed */
			return -ENODEV;

		result &= mask;
		if (result == done)
			return 0;

		udelay(1);
		usec--;
	} while (usec > 0);

	return -ETIMEDOUT;
}

static inline
struct cdns3_role_driver *cdns3_get_current_role_driver(struct cdns3 *cdns)
{
	WARN_ON(cdns->role >= CDNS3_ROLE_END || !cdns->roles[cdns->role]);
	return cdns->roles[cdns->role];
}

static int cdns3_role_start(struct cdns3 *cdns, enum cdns3_roles role)
{
	int ret;

	if (WARN_ON(role >= CDNS3_ROLE_END))
		return 0;

	if (!cdns->roles[role])
		return -ENXIO;

	if (cdns->roles[role]->state == CDNS3_ROLE_STATE_ACTIVE)
		return 0;

	mutex_lock(&cdns->mutex);
	cdns->role = role;
	ret = cdns->roles[role]->start(cdns);
	if (!ret)
		cdns->roles[role]->state = CDNS3_ROLE_STATE_ACTIVE;
	mutex_unlock(&cdns->mutex);
	return ret;
}

void cdns3_role_stop(struct cdns3 *cdns)
{
	enum cdns3_roles role = cdns->role;

	if (role >= CDNS3_ROLE_END) {
		WARN_ON(role > CDNS3_ROLE_END);
		return;
	}

	if (cdns->roles[role]->state == CDNS3_ROLE_STATE_INACTIVE)
		return;

	mutex_lock(&cdns->mutex);
	cdns->roles[role]->stop(cdns);
	cdns->roles[role]->state = CDNS3_ROLE_STATE_INACTIVE;
	mutex_unlock(&cdns->mutex);
}

/*
 * cdns->role gets from cdns3_get_initial_role, and this API tells role at the
 * runtime.
 * If both roles are supported, the role is selected based on vbus/id.
 * It could be read from OTG register or external connector.
 * If only single role is supported, only one role structure
 * is allocated, cdns->roles[CDNS3_ROLE_HOST] or cdns->roles[CDNS3_ROLE_GADGET].
 */
static enum cdns3_roles cdns3_get_initial_role(struct cdns3 *cdns)
{
	if (cdns->roles[CDNS3_ROLE_HOST] && cdns->roles[CDNS3_ROLE_GADGET]) {
		if (cdns3_is_host(cdns))
			return CDNS3_ROLE_HOST;
		if (cdns3_is_device(cdns))
			return CDNS3_ROLE_GADGET;
	}
	return cdns->roles[CDNS3_ROLE_HOST]
		? CDNS3_ROLE_HOST
		: CDNS3_ROLE_GADGET;
}

static void cdns3_exit_roles(struct cdns3 *cdns)
{
	cdns3_role_stop(cdns);
	cdns3_drd_exit(cdns);
}

/**
 * cdns3_core_init_role - initialize role of operation
 * @cdns: Pointer to cdns3 structure
 *
 * Returns 0 on success otherwise negative errno
 */
static int cdns3_core_init_role(struct cdns3 *cdns)
{
	struct device *dev = cdns->dev;
	enum usb_dr_mode best_dr_mode;
	enum usb_dr_mode dr_mode;
	int ret = 0;

	dr_mode = usb_get_dr_mode(dev);
	cdns->role = CDNS3_ROLE_END;

	/*
	 * If driver can't read mode by means of usb_get_dr_mode function then
	 * chooses mode according with Kernel configuration. This setting
	 * can be restricted later depending on strap pin configuration.
	 */
	if (dr_mode == USB_DR_MODE_UNKNOWN) {
		if (IS_ENABLED(CONFIG_USB_CDNS3_HOST) &&
		    IS_ENABLED(CONFIG_USB_CDNS3_GADGET))
			dr_mode = USB_DR_MODE_OTG;
		else if (IS_ENABLED(CONFIG_USB_CDNS3_HOST))
			dr_mode = USB_DR_MODE_HOST;
		else if (IS_ENABLED(CONFIG_USB_CDNS3_GADGET))
			dr_mode = USB_DR_MODE_PERIPHERAL;
	}

	/*
	 * At this point cdns->dr_mode contains strap configuration.
	 * Driver try update this setting considering kernel configuration
	 */
	best_dr_mode = cdns->dr_mode;

	if (dr_mode == USB_DR_MODE_OTG) {
		best_dr_mode = cdns->dr_mode;
	} else if (cdns->dr_mode == USB_DR_MODE_OTG) {
		best_dr_mode = dr_mode;
	} else if (cdns->dr_mode != dr_mode) {
		dev_err(dev, "Incorrect DRD configuration\n");
		return -EINVAL;
	}

	dr_mode = best_dr_mode;

	if (dr_mode == USB_DR_MODE_OTG || dr_mode == USB_DR_MODE_HOST) {
		ret = cdns3_host_init(cdns);
		if (ret) {
			dev_err(dev, "Host initialization failed with %d\n",
				ret);
			goto err;
		}
	}

	if (dr_mode == USB_DR_MODE_OTG || dr_mode == USB_DR_MODE_PERIPHERAL) {
		ret = cdns3_gadget_init(cdns);
		if (ret) {
			dev_err(dev, "Device initialization failed with %d\n",
				ret);
			goto err;
		}
	}

	cdns->desired_dr_mode = dr_mode;
	cdns->dr_mode = dr_mode;

	/*
	 * desired_dr_mode might have changed so we need to update
	 * the controller configuration"?
	 */
	ret = cdns3_drd_update_mode(cdns);
	if (ret)
		goto err;

	cdns->role = cdns3_get_initial_role(cdns);

	ret = cdns3_role_start(cdns, cdns->role);
	if (ret) {
		dev_err(dev, "can't start %s role\n",
			cdns3_get_current_role_driver(cdns)->name);
		goto err;
	}

	return ret;
err:
	cdns3_exit_roles(cdns);
	return ret;
}

/**
 * cdsn3_get_real_role - get real role of controller based on hardware settings.
 * @cdns: Pointer to cdns3 structure
 *
 * Returns role
 */
enum cdns3_roles cdsn3_get_real_role(struct cdns3 *cdns)
{
	enum cdns3_roles role = CDNS3_ROLE_END;

	if (cdns->current_dr_mode == USB_DR_MODE_OTG) {
		if (cdns3_get_id(cdns))
			role = CDNS3_ROLE_GADGET;
		else
			role = CDNS3_ROLE_HOST;
	} else {
		if (cdns3_is_host(cdns))
			role = CDNS3_ROLE_HOST;
		if (cdns3_is_device(cdns))
			role = CDNS3_ROLE_GADGET;
	}

	return role;
}

/**
 * cdns3_role_switch - work queue handler for role switch
 *
 * @work: work queue item structure
 *
 * Handles below events:
 * - Role switch for dual-role devices
 * - CDNS3_ROLE_GADGET <--> CDNS3_ROLE_END for peripheral-only devices
 */
static void cdns3_role_switch(struct work_struct *work)
{
	enum cdns3_roles role = CDNS3_ROLE_END;
	struct cdns3_role_driver *role_drv;
	enum cdns3_roles current_role;
	struct cdns3 *cdns;
	int ret = 0;

	cdns = container_of(work, struct cdns3, role_switch_wq);

	pm_runtime_get_sync(cdns->dev);

	role = cdsn3_get_real_role(cdns);

	role_drv = cdns3_get_current_role_driver(cdns);

	/* Disable current role if requested from debugfs */
	if (cdns->debug_disable && role_drv->state == CDNS3_ROLE_STATE_ACTIVE) {
		cdns3_role_stop(cdns);
		goto exit;
	}

	/* Do nothing if nothing changed */
	if (cdns->role == role && role_drv->state == CDNS3_ROLE_STATE_ACTIVE)
		goto exit;

	cdns3_role_stop(cdns);

	role = cdsn3_get_real_role(cdns);

	current_role = cdns->role;
	dev_dbg(cdns->dev, "Switching role");

	ret = cdns3_role_start(cdns, role);
	if (ret) {
		/* Back to current role */
		dev_err(cdns->dev, "set %d has failed, back to %d\n",
			role, current_role);
		ret = cdns3_role_start(cdns, current_role);
		if (ret)
			dev_err(cdns->dev, "back to %d failed too\n",
				current_role);
	}
exit:
	pm_runtime_put_sync(cdns->dev);
}

/**
 * cdns3_probe - probe for cdns3 core device
 * @pdev: Pointer to cdns3 core platform device
 *
 * Returns 0 on success otherwise negative errno
 */
static int cdns3_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource	*res;
	struct cdns3 *cdns;
	void __iomem *regs;
	int ret;

	cdns = devm_kzalloc(dev, sizeof(*cdns), GFP_KERNEL);
	if (!cdns)
		return -ENOMEM;

	cdns->dev = dev;

	platform_set_drvdata(pdev, cdns);

	res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "host");
	if (!res) {
		platform_get_resource_byname(pdev, IORESOURCE_IRQ, 0);
		if (!res) {
			dev_err(dev, "missing host IRQ\n");
			return -ENODEV;
		}
	}

	cdns->xhci_res[0] = *res;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "xhci");
	if (!res) {
		dev_err(dev, "couldn't get xhci resource\n");
		return -ENXIO;
	}

	cdns->xhci_res[1] = *res;

	cdns->dev_irq = platform_get_irq_byname(pdev, "peripheral");
	if (cdns->dev_irq == -EPROBE_DEFER)
		return cdns->dev_irq;

	if (cdns->dev_irq < 0) {
		cdns->dev_irq = platform_get_irq(pdev, 0);
		if (cdns->dev_irq < 0) {
			if (cdns->dev_irq != -EPROBE_DEFER)
				dev_err(dev, "couldn't get peripheral irq\n");
			return cdns->dev_irq;
		}
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dev");
	regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(regs)) {
		dev_err(dev, "couldn't iomap dev resource\n");
		return PTR_ERR(regs);
	}
	cdns->dev_regs	= regs;

	cdns->otg_irq = platform_get_irq_byname(pdev, "otg");
	if (cdns->otg_irq == -EPROBE_DEFER)
		return cdns->otg_irq;

	if (cdns->otg_irq < 0) {
		cdns->otg_irq = platform_get_irq(pdev, 0);
		if (cdns->otg_irq < 0) {
			if (cdns->otg_irq != -EPROBE_DEFER)
				dev_err(dev, "couldn't get otg irq\n");
			return cdns->otg_irq;
		}
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "otg");
	if (!res) {
		dev_err(dev, "couldn't get otg resource\n");
		return -ENXIO;
	}

	cdns->otg_res = *res;

	mutex_init(&cdns->mutex);

	cdns->phy = devm_phy_optional_get(dev, "cdns3,usbphy");
	if (IS_ERR(cdns->phy))
		return PTR_ERR(cdns->phy);

	ret = phy_init(cdns->phy);
	if (ret) {
		dev_err(dev, "phy_init error\n");
		return ret;
	}

	ret = phy_power_on(cdns->phy);
	if (ret) {
		dev_err(dev, "phy_power_on error\n");
		phy_exit(cdns->phy);
		return ret;
	}

	INIT_WORK(&cdns->role_switch_wq, cdns3_role_switch);

	ret = cdns3_drd_init(cdns);
	if (ret)
		goto err;

	ret = cdns3_core_init_role(cdns);
	if (ret)
		goto err;

	cdns3_debugfs_init(cdns);
	device_set_wakeup_capable(dev, true);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	/*
	 * The controller needs less time between bus and controller suspend,
	 * and we also needs a small delay to avoid frequently entering low
	 * power mode.
	 */
	pm_runtime_set_autosuspend_delay(dev, 20);
	pm_runtime_mark_last_busy(dev);
	pm_runtime_use_autosuspend(dev);
	dev_dbg(dev, "Cadence USB3 core: probe succeed\n");

	return 0;

err:
	phy_power_off(cdns->phy);
	phy_exit(cdns->phy);
	return ret;
}

/**
 * cdns3_remove - unbind drd driver and clean up
 * @pdev: Pointer to Linux platform device
 *
 * Returns 0 on success otherwise negative errno
 */
static int cdns3_remove(struct platform_device *pdev)
{
	struct cdns3 *cdns = platform_get_drvdata(pdev);

	pm_runtime_get_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	pm_runtime_put_noidle(&pdev->dev);
	cdns3_debugfs_exit(cdns);
	cdns3_exit_roles(cdns);
	phy_power_off(cdns->phy);
	phy_exit(cdns->phy);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id of_cdns3_match[] = {
	{ .compatible = "cdns,usb3-1.0.0" },
	{ .compatible = "cdns,usb3-1.0.1" },
	{ },
};
MODULE_DEVICE_TABLE(of, of_cdns3_match);
#endif

static struct platform_driver cdns3_driver = {
	.probe		= cdns3_probe,
	.remove		= cdns3_remove,
	.driver		= {
		.name	= "cdns-usb3",
		.of_match_table	= of_match_ptr(of_cdns3_match),
	},
};

module_platform_driver(cdns3_driver);

MODULE_ALIAS("platform:cdns3");
MODULE_AUTHOR("Pawel Laszczak <pawell@cadence.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Cadence USB3 DRD Controller Driver");
