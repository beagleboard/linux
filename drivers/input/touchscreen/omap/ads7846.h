/*
 * ads7846.h - header file for ADS7846 touchscreen controller
 * 
 * Copyright 2002 MontaVista Software Inc.
 * Author: MontaVista Software, Inc.
 *         	stevel@mvista.com or source@mvista.com
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef __ADS7846_H
#define __ADS7846_H

// ADS7846 Control Byte bit defines
#define ADS7846_S         (1<<7)
#define ADS7846_ADDR_BIT  4
#define ADS7846_ADDR_MASK (0x7<<ADS7846_ADDR_BIT)
#define   ADS7846_MEASURE_X  (0x5<<ADS7846_ADDR_BIT)
#define   ADS7846_MEASURE_Y  (0x1<<ADS7846_ADDR_BIT)
#define   ADS7846_MEASURE_Z1 (0x3<<ADS7846_ADDR_BIT)
#define   ADS7846_MEASURE_Z2 (0x4<<ADS7846_ADDR_BIT)
#define ADS7846_8BITS     (1<<3)
#define ADS7846_12BITS    0
#define ADS7846_SER       (1<<2)
#define ADS7846_DFR       0
#define ADS7846_PWR_BIT   0
#define   ADS7846_PD      0
#define   ADS7846_ADC_ON  (0x1<<ADS7846_PWR_BIT)
#define   ADS7846_REF_ON  (0x2<<ADS7846_PWR_BIT)
#define   ADS7846_REF_ADC_ON (0x3<<ADS7846_PWR_BIT)

#define MEASURE_12BIT_X \
  (ADS7846_S | ADS7846_MEASURE_X | ADS7846_12BITS | ADS7846_DFR | ADS7846_PD)
#define MEASURE_12BIT_Y \
  (ADS7846_S | ADS7846_MEASURE_Y | ADS7846_12BITS | ADS7846_DFR | ADS7846_PD)
#define MEASURE_12BIT_Z1 \
  (ADS7846_S | ADS7846_MEASURE_Z1 | ADS7846_12BITS | ADS7846_DFR | ADS7846_PD)
#define MEASURE_12BIT_Z2 \
  (ADS7846_S | ADS7846_MEASURE_Z2 | ADS7846_12BITS | ADS7846_DFR | ADS7846_PD)

#endif /* __ADS7846_H */
