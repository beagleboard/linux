/***
 *
 *  rtmac/nomac/nomac_dev.c
 *
 *  RTmac - real-time networking media access control subsystem
 *  Copyright (C) 2002      Marc Kleine-Budde <kleine-budde@gmx.de>
 *                2003-2005 Jan Kiszka <Jan.Kiszka@web.de>
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

#include <linux/list.h>

#include <rtdev.h>
#include <rtmac.h>
#include <rtmac/nomac/nomac.h>

static int nomac_dev_openclose(void)
{
	return 0;
}

static int nomac_dev_ioctl(struct rtdm_fd *fd, unsigned int request, void *arg)
{
	struct nomac_priv *nomac;

	nomac = container_of(rtdm_fd_to_context(fd)->device, struct nomac_priv,
			     api_device);

	switch (request) {
	case RTMAC_RTIOC_TIMEOFFSET:

	case RTMAC_RTIOC_WAITONCYCLE:

	default:
		return -ENOTTY;
	}
}

static struct rtdm_driver
	nomac_driver = { .profile_info = RTDM_PROFILE_INFO(
				 nomac, RTDM_CLASS_RTMAC,
				 RTDM_SUBCLASS_UNMANAGED, RTNET_RTDM_VER),
			 .device_flags = RTDM_NAMED_DEVICE,
			 .device_count = 1,
			 .context_size = 0,
			 .ops = {
				 .open = (typeof(nomac_driver.ops.open))
					 nomac_dev_openclose,
				 .ioctl_rt = nomac_dev_ioctl,
				 .ioctl_nrt = nomac_dev_ioctl,
				 .close = (typeof(nomac_driver.ops.close))
					 nomac_dev_openclose,
			 } };

int nomac_dev_init(struct rtnet_device *rtdev, struct nomac_priv *nomac)
{
	char *pos;

	strcpy(nomac->device_name, "NOMAC");
	for (pos = rtdev->name + strlen(rtdev->name) - 1;
	     (pos >= rtdev->name) && ((*pos) >= '0') && (*pos <= '9'); pos--)
		;
	strncat(nomac->device_name + 5, pos + 1, IFNAMSIZ - 5);

	nomac->api_driver = nomac_driver;
	nomac->api_device.driver = &nomac->api_driver;
	nomac->api_device.label = nomac->device_name;

	return rtdm_dev_register(&nomac->api_device);
}
