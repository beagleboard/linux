/***
 *
 *  rtmac/nomac/nomac_ioctl.c
 *
 *  RTmac - real-time networking media access control subsystem
 *  Copyright (C) 2002       Marc Kleine-Budde <kleine-budde@gmx.de>,
 *                2003, 2004 Jan Kiszka <Jan.Kiszka@web.de>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/module.h>
#include <linux/uaccess.h>

#include <nomac_chrdev.h>
#include <rtmac/nomac/nomac.h>

static int nomac_ioctl_attach(struct rtnet_device *rtdev)
{
	struct nomac_priv *nomac;
	int ret;

	if (rtdev->mac_priv == NULL) {
		ret = rtmac_disc_attach(rtdev, &nomac_disc);
		if (ret < 0)
			return ret;
	}

	nomac = (struct nomac_priv *)rtdev->mac_priv->disc_priv;
	if (nomac->magic != NOMAC_MAGIC)
		return -ENOTTY;

	/* ... */

	return 0;
}

static int nomac_ioctl_detach(struct rtnet_device *rtdev)
{
	struct nomac_priv *nomac;
	int ret;

	if (rtdev->mac_priv == NULL)
		return -ENOTTY;

	nomac = (struct nomac_priv *)rtdev->mac_priv->disc_priv;
	if (nomac->magic != NOMAC_MAGIC)
		return -ENOTTY;

	ret = rtmac_disc_detach(rtdev);

	/* ... */

	return ret;
}

int nomac_ioctl(struct rtnet_device *rtdev, unsigned int request,
		unsigned long arg)
{
	struct nomac_config cfg;
	int ret;

	ret = copy_from_user(&cfg, (void *)arg, sizeof(cfg));
	if (ret != 0)
		return -EFAULT;

	if (mutex_lock_interruptible(&rtdev->nrt_lock))
		return -ERESTARTSYS;

	switch (request) {
	case NOMAC_IOC_ATTACH:
		ret = nomac_ioctl_attach(rtdev);
		break;

	case NOMAC_IOC_DETACH:
		ret = nomac_ioctl_detach(rtdev);
		break;

	default:
		ret = -ENOTTY;
	}

	mutex_unlock(&rtdev->nrt_lock);

	return ret;
}
