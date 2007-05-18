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
#define REG_KEYP_CTRL_REG			(0x0)
#define REG_KEY_DEB_REG				(0x1)
#define REG_LK_PTV_REG				(0x3)
#define REG_FULL_CODE_7_0			(0x9)
#define REG_KEYP_ISR1				(0x11)
#define REG_KEYP_IMR1				(0x12)
#define REG_KEYP_EDR				(0x16)
#define REG_KEYP_SIH_CTRL			(0x17)

/* KEYP_CTRL_REG Fields */
#define BIT_KEYP_CTRL_REG_SOFT_NRST_MASK	(0x00000001)
#define BIT_KEYP_CTRL_REG_SOFTMODEN_MASK	(0x00000002)
#define BIT_KEYP_CTRL_REG_LK_EN_MASK		(0x00000004)
#define BIT_KEYP_CTRL_REG_TOE_EN_MASK		(0x00000008)
#define BIT_KEYP_CTRL_REG_TOLE_EN_MASK		(0x00000010)
#define BIT_KEYP_CTRL_REG_RP_EN_MASK		(0x00000020)
#define BIT_KEYP_CTRL_REG_KBD_ON_MASK		(0x00000040)


#define KEYP_CTRL_REG_MASK_NOAUTORPT		BIT_KEYP_CTRL_REG_SOFT_NRST_MASK |\
						BIT_KEYP_CTRL_REG_SOFTMODEN_MASK |\
						BIT_KEYP_CTRL_REG_KBD_ON_MASK

/* LK_PTV_REG Fields */
#define BIT_LK_PTV_REG_PTV			(0x005)
#define BIT_LK_PTV_REG_PTV_MASK			(0x000000E0)
#define BIT_PTV_REG_PTV4			(0x4)

/* KEYP_IMR1 Fields */
#define KEYP_IMR1_MASK				(0x0F)
#define KEYP_IMR1_UNMASK			(0x00)

/* KEYP_EDR Fields */
#define BIT_KEYP_EDR_ITKPFALLING_MASK		(0x00000001)
#define BIT_KEYP_EDR_ITKPRISING_MASK		(0x00000002)
#define BIT_KEYP_EDR_ITLKFALLING_MASK		(0x00000004)
#define BIT_KEYP_EDR_ITLKRISING_MASK		(0x00000008)
#define BIT_KEYP_EDR_ITTOFALLING_MASK		(0x00000010)
#define BIT_KEYP_EDR_ITTORISING_MASK		(0x00000020)
#define BIT_KEYP_EDR_ITMISFALLING_MASK		(0x00000040)
#define BIT_KEYP_EDR_ITMISRISING_MASK		(0x00000080)

#define KEYP_EDR_MASK				BIT_KEYP_EDR_ITKPFALLING_MASK |\
						BIT_KEYP_EDR_ITLKFALLING_MASK |\
						BIT_KEYP_EDR_ITTOFALLING_MASK |\
						BIT_KEYP_EDR_ITMISFALLING_MASK
/* KEYP_SIH_CTRL Fields */
#define KEYP_SIH_CTRL_MASK			(0x04)

#endif	/* End of __TWL4030-KEYPAD_H__ */
