/**
 * drivers/cbus/retu.h
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

#ifndef __DRIVERS_CBUS_RETU_H
#define __DRIVERS_CBUS_RETU_H

#include <linux/types.h>

/* Registers */
#define RETU_REG_ASICR		0x00	/* ASIC ID & revision */
#define RETU_REG_IDR		0x01	/* Interrupt ID */
#define RETU_REG_IMR		0x02	/* Interrupt mask */
#define RETU_REG_RTCDSR		0x03	/* RTC seconds register */
#define RETU_REG_RTCHMR		0x04	/* RTC hours and minutes register */
#define RETU_REG_RTCHMAR	0x05	/* RTC hours and minutes alarm and time set register */
#define RETU_REG_RTCCALR	0x06	/* RTC calibration register */
#define RETU_REG_ADCR		0x08	/* ADC result */
#define RETU_REG_ADCSCR		0x09	/* ADC sample ctrl */
#define RETU_REG_CC1		0x0d	/* Common control register 1 */
#define RETU_REG_CC2		0x0e	/* Common control register 2 */
#define RETU_REG_CTRL_CLR	0x0f	/* Regulator clear register */
#define RETU_REG_CTRL_SET	0x10	/* Regulator set register */
#define RETU_REG_STATUS		0x16	/* Status register */
#define RETU_REG_WATCHDOG	0x17	/* Watchdog register */
#define RETU_REG_AUDTXR		0x18	/* Audio Codec Tx register */
#define RETU_REG_MAX		0x1f

/* Interrupt sources */
#define RETU_INT_PWR		0
#define RETU_INT_CHAR		1
#define RETU_INT_RTCS		2
#define RETU_INT_RTCM		3
#define RETU_INT_RTCD		4
#define RETU_INT_RTCA		5
#define RETU_INT_HOOK		6
#define RETU_INT_HEAD		7
#define RETU_INT_ADCS		8

#define	MAX_RETU_IRQ_HANDLERS	16

int retu_read_reg(int reg);
void retu_write_reg(int reg, u16 val);
void retu_set_clear_reg_bits(int reg, u16 set, u16 clear);
int retu_read_adc(int channel);
int retu_request_irq(int id, void *irq_handler, unsigned long arg, char *name);
void retu_free_irq(int id);
void retu_enable_irq(int id);
void retu_disable_irq(int id);
void retu_ack_irq(int id);

#ifdef CONFIG_CBUS_RETU_USER
int retu_user_init(void);
void retu_user_cleanup(void);
#endif

extern spinlock_t retu_lock;

#endif /* __DRIVERS_CBUS_RETU_H */
