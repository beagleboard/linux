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
#define TAHVO_REG_LEDPWMR	0x05	/* LED PWM */
#define TAHVO_REG_USBR		0x06	/* USB control */
#define TAHVO_REG_MAX		0x0d

/* Interrupt sources */
#define TAHVO_INT_VBUSON	0

#define MAX_TAHVO_IRQ_HANDLERS	8

int tahvo_read_reg(int reg);
void tahvo_write_reg(int reg, u16 val);
void tahvo_set_clear_reg_bits(int reg, u16 set, u16 clear);
int tahvo_request_irq(int id, void *irq_handler, unsigned long arg, char *name);
void tahvo_free_irq(int id);
void tahvo_enable_irq(int id);
void tahvo_disable_irq(int id);
void tahvo_ack_irq(int id);
int tahvo_get_backlight_level(void);
int tahvo_get_max_backlight_level(void);
void tahvo_set_backlight_level(int level);

#ifdef CONFIG_CBUS_TAHVO_USER
int tahvo_user_init(void);
void tahvo_user_cleanup(void);
#endif

extern spinlock_t tahvo_lock;

#endif /* __DRIVERS_CBUS_TAHVO_H */
