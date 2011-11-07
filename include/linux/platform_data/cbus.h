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

#ifndef __INCLUDE_LINUX_CBUS_H
#define __INCLUDE_LINUX_CBUS_H

struct cbus_host_platform_data {
	int	dat_gpio;
	int	clk_gpio;
	int	sel_gpio;
};

#endif /* __INCLUDE_LINUX_CBUS_H */
