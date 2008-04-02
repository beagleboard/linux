/*
 * drivers/input/keyboard/twl4030-keypad.h
 *
 * Copyright (C) 2006-2007 Texas Instruments, Inc.
 *
 * Intial Code:
 *	Syed Mohammed Khasim <x0khasim@ti.com>
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef __TWL4030_KEYPAD_H__
#define __TWL4030_KEYPAD_H__

/* Register Definitions */
#define KEYP_CTRL				0x00
#define KEYP_DEB				0x01
#define KEYP_LONG_KEY				0x02
#define KEYP_LK_PTV				0x03
#define KEYP_TIMEOUT_L				0x04
#define KEYP_TIMEOUT_H				0x05
#define KEYP_FULL_CODE_7_0			0x09
#define KEYP_ISR1				0x11
#define KEYP_IMR1				0x12
#define KEYP_EDR				0x16
#define KEYP_SIH_CTRL				0x17

/* KEYP_CTRL_REG Fields */
#define KEYP_CTRL_SOFT_NRST			0x01
#define KEYP_CTRL_SOFTMODEN			0x02
#define KEYP_CTRL_LK_EN				0x04
#define KEYP_CTRL_TOE_EN			0x08
#define KEYP_CTRL_TOLE_EN			0x10
#define KEYP_CTRL_RP_EN				0x20
#define KEYP_CTRL_KBD_ON			0x40


#define KEYP_CTRL_NOAUTORPT			(KEYP_CTRL_SOFT_NRST |	\
						 KEYP_CTRL_SOFTMODEN |	\
						 KEYP_CTRL_TOE_EN |	\
						 KEYP_CTRL_KBD_ON)

/* KEYP_DEB, KEYP_LONG_KEY, KEYP_TIMEOUT_x*/
#define KEYP_PERIOD_US(T, prescale)		(T / (31 << (prescale + 1)) - 1)

/* KEYP_LK_PTV_REG Fields */
#define KEYP_LK_PTV_PTV_SHIFT			5

/* KEYP_IMR1 Fields */
#define KEYP_IMR1_MIS				0x08
#define KEYP_IMR1_TO				0x04
#define KEYP_IMR1_LK				0x02
#define KEYP_IMR1_KP				0x01

/* KEYP_EDR Fields */
#define KEYP_EDR_KP_FALLING			0x01
#define KEYP_EDR_KP_RISING			0x02
#define KEYP_EDR_KP_BOTH			0x03
#define KEYP_EDR_LK_FALLING			0x04
#define KEYP_EDR_LK_RISING			0x08
#define KEYP_EDR_TO_FALLING			0x10
#define KEYP_EDR_TO_RISING			0x20
#define KEYP_EDR_MIS_FALLING			0x40
#define KEYP_EDR_MIS_RISING			0x80

/* KEYP_SIH_CTRL Fields */
#define KEYP_SIH_CTRL_COR			0x04
#define KEYP_SIH_CTRL_PEND_DIS			0x02
#define KEYP_SIH_CTRL_EXCL_EN			0x01

#endif	/* End of __TWL4030-KEYPAD_H__ */
