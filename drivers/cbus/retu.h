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
#define  RETU_REG_STATUS_BATAVAIL	0x0100 /* Battery available */
#define  RETU_REG_STATUS_CHGPLUG	0x1000 /* Charger is plugged in */
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

/* ADC channels */
#define RETU_ADC_GND		0x00 /* Ground */
#define RETU_ADC_BSI		0x01 /* Battery Size Indicator */
#define RETU_ADC_BATTEMP	0x02 /* Battery temperature */
#define RETU_ADC_CHGVOLT	0x03 /* Charger voltage */
#define RETU_ADC_HEADSET	0x04 /* Headset detection */
#define RETU_ADC_HOOKDET	0x05 /* Hook detection */
#define RETU_ADC_RFGP		0x06 /* RF GP */
#define RETU_ADC_WBTX		0x07 /* Wideband Tx detection */
#define RETU_ADC_BATTVOLT	0x08 /* Battery voltage measurement */
#define RETU_ADC_GND2		0x09 /* Ground */
#define RETU_ADC_LIGHTSENS	0x0A /* Light sensor */
#define RETU_ADC_LIGHTTEMP	0x0B /* Light sensor temperature */
#define RETU_ADC_BKUPVOLT	0x0C /* Backup battery voltage */
#define RETU_ADC_TEMP		0x0D /* RETU temperature */


int retu_read_reg(struct device *child, unsigned reg);
void retu_write_reg(struct device *child, unsigned reg, u16 val);
void retu_set_clear_reg_bits(struct device *child, unsigned reg, u16 set,
		u16 clear);
int retu_read_adc(struct device *child, int channel);

#endif /* __DRIVERS_CBUS_RETU_H */
