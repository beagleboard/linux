/**
 * drivers/cbus/user_retu_tahvo.h
 *
 * Copyright (C) 2004, 2005 Nokia Corporation
 *
 * Written by Mikko Ylinen <mikko.k.ylinen@nokia.com>
 *
 * Definitions and types used by both retu-user and tahvo-user.
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef _USER_RETU_TAHVO_H
#define _USER_RETU_TAHVO_H

/* Chip IDs */
#define CHIP_RETU	1
#define CHIP_TAHVO	2

/* Register access type bits */
#define READ_ONLY		1
#define WRITE_ONLY		2
#define READ_WRITE		3
#define TOGGLE			4

#define MASK(field)		((u16)(field & 0xFFFF))
#define REG(field)		((u16)((field >> 16) & 0x3F))

/*** IOCTL definitions. These should be kept in sync with user space **********/

#define URT_IOC_MAGIC '`'

/*
 * IOCTL function naming conventions:
 * ==================================
 *  0 -- No argument and return value
 *  S -- Set through a pointer
 *  T -- Tell directly with the argument value
 *  G -- Reply by setting through a pointer
 *  Q -- response is on the return value
 *  X -- S and G atomically
 *  H -- T and Q atomically
 */

/* General */
#define URT_IOCT_IRQ_SUBSCR		_IO(URT_IOC_MAGIC, 0)

/* RETU */
#define RETU_IOCH_READ			_IO(URT_IOC_MAGIC, 1)
#define RETU_IOCX_WRITE			_IO(URT_IOC_MAGIC, 2)
#define RETU_IOCH_ADC_READ		_IO(URT_IOC_MAGIC, 3)

/* TAHVO */
#define TAHVO_IOCH_READ			_IO(URT_IOC_MAGIC, 4)
#define TAHVO_IOCX_WRITE		_IO(URT_IOC_MAGIC, 5)

/* This structure is used for writing RETU/TAHVO registers */
struct retu_tahvo_write_parms {
    u32	field;
    u16	value;
    u8	result;
};

#endif
