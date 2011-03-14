/*
 * cbus.h - CBUS platform_data definition
 *
 * Copyright (C) 2004 - 2009 Nokia Corporation
 *
 * Written by Felipe Balbi <felipe.balbi@nokia.com>
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __PLAT_CBUS_H
#define __PLAT_CBUS_H

#define CBUS_RETU_DEVICE_ID	0x01
#define CBUS_TAHVO_DEVICE_ID	0x02

struct cbus_host_platform_data {
	int	dat_gpio;
	int	clk_gpio;
	int	sel_gpio;
};

struct cbus_retu_platform_data {
	int	irq_base;
	int	irq_end;
	int	devid;
};

#endif /* __PLAT_CBUS_H */
