/*
 * twl4030.h - header for TWL4030 PM and audio CODEC device
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
 *
 * Based on tlv320aic23.c:
 * Copyright (c) by Kai Svahn <kai.svahn@nokia.com>
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#ifndef __TWL4030_H_
#define __TWL4030_H_

/* USB ID */
#define TWL4030_MODULE_USB		0x00
/* AUD ID */
#define TWL4030_MODULE_AUDIO_VOICE	0x01
#define TWL4030_MODULE_GPIO		0x02
#define TWL4030_MODULE_INTBR		0x03
#define TWL4030_MODULE_PIH		0x04
#define TWL4030_MODULE_TEST		0x05
/* AUX ID */
#define TWL4030_MODULE_KEYPAD		0x06
#define TWL4030_MODULE_MADC		0x07
#define TWL4030_MODULE_INTERRUPTS	0x08
#define TWL4030_MODULE_LED		0x09
#define TWL4030_MODULE_MAIN_CHARGE	0x0A
#define TWL4030_MODULE_PRECHARGE	0x0B
#define TWL4030_MODULE_PWM0		0x0C
#define TWL4030_MODULE_PWM1		0x0D
#define TWL4030_MODULE_PWMA		0x0E
#define TWL4030_MODULE_PWMB		0x0F
/* POWER ID */
#define TWL4030_MODULE_BACKUP		0x10
#define TWL4030_MODULE_INT		0x11
#define TWL4030_MODULE_PM_MASTER	0x12
#define TWL4030_MODULE_PM_RECIEVER	0x13
#define TWL4030_MODULE_RTC		0x14
#define TWL4030_MODULE_SECURED_REG	0x15

/* IRQ information-need base */
#include <asm/arch/irqs.h>
/* TWL4030 interrupts */

#define TWL4030_MODIRQ_GPIO		(IH_TWL4030_BASE + 0)
#define TWL4030_MODIRQ_KEYPAD		(IH_TWL4030_BASE + 1)
#define TWL4030_MODIRQ_BCI		(IH_TWL4030_BASE + 2)
#define TWL4030_MODIRQ_MADC		(IH_TWL4030_BASE + 3)
#define TWL4030_MODIRQ_USB		(IH_TWL4030_BASE + 4)
#define TWL4030_MODIRQ_PWR		(IH_TWL4030_BASE + 5)
/* Rest are unsued currently*/

/* Offsets to Power Registers */
#define TWL4030_VDAC_DEV_GRP		0x3B
#define TWL4030_VDAC_DEDICATED		0x3E
#define TWL4030_VAUX2_DEV_GRP		0x1B
#define TWL4030_VAUX2_DEDICATED		0x1E
#define TWL4030_VAUX3_DEV_GRP		0x1F
#define TWL4030_VAUX3_DEDICATED		0x22

/* Functions to read and write from TWL4030 */

/*
 * IMP NOTE:
 * The base address of the module will be added by the triton driver
 * It is the caller's responsibility to ensure sane values
 */
int twl4030_i2c_write_u8(u8 mod_no, u8 val, u8 reg);
int twl4030_i2c_read_u8(u8 mod_no, u8* val, u8 reg);

 /*
  * i2c_write: IMPORTANT - Allocate value num_bytes+1 and valid data starts at
  *		Offset 1.
  */
int twl4030_i2c_write(u8 mod_no, u8 * value, u8 reg, u8 num_bytes);
int twl4030_i2c_read(u8 mod_no, u8 * value, u8 reg, u8 num_bytes);

#endif /* End of __TWL4030_H */
