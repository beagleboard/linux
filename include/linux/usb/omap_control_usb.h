/*
 * omap_control_usb.h - Header file for the USB part of control module.
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Author: Kishon Vijay Abraham I <kishon@ti.com>
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __OMAP_CONTROL_USB_H__
#define __OMAP_CONTROL_USB_H__

struct omap_control_usb {
	struct device *dev;

	u32 __iomem *dev_conf;
	u32 __iomem *otghs_control;

	u8 has_mailbox:1;
};

struct omap_control_usb_platform_data {
	u8 has_mailbox:1;
};

#define	PHY_PD		BIT(0)

#define	AVALID		BIT(0)
#define	BVALID		BIT(1)
#define	VBUSVALID	BIT(2)
#define	SESSEND		BIT(3)
#define	IDDIG		BIT(4)

#if (defined(CONFIG_OMAP_CONTROL_USB) || \
				defined(CONFIG_OMAP_CONTROL_USB_MODULE))
extern struct device *get_omap_control_dev(void);
extern void omap_control_usb_phy_power(struct device *dev, int on);
extern void omap_control_usb_host_mode(struct device *dev);
extern void omap_control_usb_device_mode(struct device *dev);
extern void omap_control_usb_set_sessionend(struct device *dev);
#else
static inline struct device *get_omap_control_dev()
{
	return ERR_PTR(-ENODEV);
}

static inline void omap_control_usb_phy_power(struct device *dev, int on)
{
}

static inline void omap_control_usb_host_mode(struct device *dev)
{
}

static inline void omap_control_usb_device_mode(struct device *dev)
{
}

static inline void omap_control_usb_set_sessionend(struct device *dev)
{
}
#endif

#endif	/* __OMAP_CONTROL_USB_H__ */
