/* rtmac_module.c
 *
 * rtmac - real-time networking media access control subsystem
 * Copyright (C) 2002 Marc Kleine-Budde <kleine-budde@gmx.de>,
 *               2003 Jan Kiszka <Jan.Kiszka@web.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

#include <rtdm/driver.h>

#include <rtmac/rtmac_disc.h>
#include <rtmac/rtmac_proc.h>
#include <rtmac/rtmac_proto.h>
#include <rtmac/rtmac_vnic.h>

int __init rtmac_init(void)
{
	int ret = 0;

	printk("RTmac: init realtime media access control\n");

#ifdef CONFIG_XENO_OPT_VFILE
	ret = rtmac_proc_register();
	if (ret < 0)
		return ret;
#endif

	ret = rtmac_vnic_module_init();
	if (ret < 0)
		goto error1;

	ret = rtmac_proto_init();
	if (ret < 0)
		goto error2;

	return 0;

error2:
	rtmac_vnic_module_cleanup();

error1:
#ifdef CONFIG_XENO_OPT_VFILE
	rtmac_proc_release();
#endif
	return ret;
}

void rtmac_release(void)
{
	rtmac_proto_release();
	rtmac_vnic_module_cleanup();
#ifdef CONFIG_XENO_OPT_VFILE
	rtmac_proc_release();
#endif

	printk("RTmac: unloaded\n");
}

module_init(rtmac_init);
module_exit(rtmac_release);

MODULE_AUTHOR("Marc Kleine-Budde, Jan Kiszka");
MODULE_LICENSE("GPL");
