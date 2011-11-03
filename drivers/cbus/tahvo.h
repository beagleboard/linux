/*
 * drivers/cbus/tahvo.h
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

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __DRIVERS_CBUS_TAHVO_H
#define __DRIVERS_CBUS_TAHVO_H

#include <linux/types.h>

/* Registers */
#define TAHVO_REG_ASICR		0x00	/* ASIC ID & revision */
#define TAHVO_REG_IDR		0x01	/* Interrupt ID */
#define TAHVO_REG_IDSR		0x02	/* Interrupt status */
#define TAHVO_REG_IMR		0x03	/* Interrupt mask */
#define TAHVO_REG_CHGCURR	0x04	/* Charge current control PWM (8-bit) */
#define TAHVO_REG_LEDPWMR	0x05	/* LED PWM */
#define TAHVO_REG_USBR		0x06	/* USB control */
#define TAHVO_REG_CHGCTL	0x08	/* Charge control register */
#define  TAHVO_REG_CHGCTL_EN		0x0001	/* Global charge enable */
#define  TAHVO_REG_CHGCTL_PWMOVR	0x0004	/* PWM override. Force charge PWM to 0%/100% duty cycle. */
#define  TAHVO_REG_CHGCTL_PWMOVRZERO	0x0008	/* If set, PWM override is 0% (If unset -> 100%) */
#define  TAHVO_REG_CHGCTL_CURMEAS	0x0040	/* Enable battery current measurement. */
#define  TAHVO_REG_CHGCTL_CURTIMRST	0x0080	/* Current measure timer reset. */
#define TAHVO_REG_BATCURRTIMER	0x0c	/* Battery current measure timer (8-bit) */
#define TAHVO_REG_BATCURR	0x0d	/* Battery (dis)charge current (signed 16-bit) */

#define TAHVO_REG_MAX		0x0d

/* Interrupt sources */
#define TAHVO_INT_VBUSON	0
#define TAHVO_INT_BATCURR	7 /* Battery current measure timer */

#define MAX_TAHVO_IRQ_HANDLERS	8

int tahvo_read_reg(struct device *child, unsigned reg);
void tahvo_write_reg(struct device *child, unsigned reg, u16 val);
void tahvo_set_clear_reg_bits(struct device *child, unsigned reg, u16 set,
		u16 clear);

#endif /* __DRIVERS_CBUS_TAHVO_H */
