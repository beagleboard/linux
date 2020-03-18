/* rtmac_syms.c
 *
 * rtmac - real-time networking media access control subsystem
 * Copyright (C) 2002 Marc Kleine-Budde <kleine-budde@gmx.de>
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

#include <linux/kernel.h>
#include <linux/module.h>

#include <rtmac/rtmac_disc.h>
#include <rtmac/rtmac_vnic.h>


EXPORT_SYMBOL_GPL(__rtmac_disc_register);
EXPORT_SYMBOL_GPL(rtmac_disc_deregister);

EXPORT_SYMBOL_GPL(rtmac_disc_attach);
EXPORT_SYMBOL_GPL(rtmac_disc_detach);

EXPORT_SYMBOL_GPL(rtmac_vnic_set_max_mtu);

EXPORT_SYMBOL_GPL(rtmac_vnic_xmit);
