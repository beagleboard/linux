/*
 * drivers/cbus/cbus.h
 *
 * Copyright (C) 2004, 2005 Nokia Corporation
 *
 * Written by Juha Yrjölä <juha.yrjola@nokia.com> and
 *	      David Weinehall <david.weinehall@nokia.com>
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

#ifndef __DRIVERS_CBUS_CBUS_H
#define __DRIVERS_CBUS_CBUS_H

#define CBUS_RETU_DEVICE_ID	0x01
#define CBUS_TAHVO_DEVICE_ID	0x02

extern int cbus_read_reg(struct device *, unsigned dev, unsigned reg);
extern int cbus_write_reg(struct device *, unsigned dev, unsigned reg,
		unsigned val);

#endif /* __DRIVERS_CBUS_CBUS_H */
