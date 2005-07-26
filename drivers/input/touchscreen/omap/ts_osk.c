/*
 * ts_osk.c - touchscreen support for OMAP OSK board
 * 
 * Copyright 2002 MontaVista Software Inc.
 * Author: MontaVista Software, Inc.
 *         	stevel@mvista.com or source@mvista.com
 *
 * The touchscreen hardware on the OSK uses OMAP5912 uWire interface,
 * GPIO4 (/PENIRQ) and GPIO6 (BUSY) to connect to an ADS7846 
 * touch screen controller. GPIO6 doesn't seem to be necessary here.
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

#include <linux/input.h>
#include <linux/device.h>
#include <asm/arch/mux.h>
#include <asm/arch/gpio.h>
#include <asm/mach-types.h>

#include "../drivers/ssi/omap-uwire.h"

#include "omap_ts.h"
#include "ads7846.h"

// /PENIRQ on GPIO4 on OSK
#define PEN_IRQ		 OMAP_GPIO_IRQ(4)

// ADS7846 is on OSK uWire CS 0
#define ADS7846_UWIRE_CS	0
#define UWIRE_LEAVE_CS		1

#define X_PLATE_OHMS 419
#define Y_PLATE_OHMS 486

static int osk_ts_penup(void);
static int osk_ts_probe(struct omap_ts_t *ts);
static void osk_ts_read(u16 * data);
static void osk_ts_enable(void);
static void osk_ts_disable(void);
#ifdef MODULE
static void osk_ts_remove(void);
#endif

struct ts_device osk_ts = {
        .probe   = osk_ts_probe,
        .read    = osk_ts_read,
        .enable  = osk_ts_enable,
        .disable = osk_ts_disable,
        .remove  = __exit_p(osk_ts_remove),
	.penup   = osk_ts_penup,
};

static u16 ads7846_do(u8 cmd)
{
	u16 val = 0;
	
	// send the command to the ADS7846, leave CS active after this
	omap_uwire_data_transfer(ADS7846_UWIRE_CS, cmd, 8, 0, NULL, UWIRE_LEAVE_CS);

	// now read returned data
        omap_uwire_data_transfer(ADS7846_UWIRE_CS, 0, 0, 16, &val, !UWIRE_LEAVE_CS);
	
	return val;
}

static int osk_ts_penup(void)
{
	return (omap_get_gpio_datain(4));
}

static int  __init osk_ts_probe(struct omap_ts_t *ts)
{
#ifdef	CONFIG_OMAP_OSK_MISTRAL
	if (!machine_is_omap_osk())
		return -ENODEV;

        /* Configure GPIO4 (pin M17 ZDY) as /PENIRQ interrupt input */
        omap_cfg_reg(P20_1610_GPIO4);
	omap_request_gpio(4);
	omap_set_gpio_direction(4, 1);
	set_irq_type(OMAP_GPIO_IRQ(4), IRQT_FALLING);

	ts->irq = PEN_IRQ;

	/* Configure uWire interface. ADS7846 is on CS0 */
	omap_uwire_configure_mode(ADS7846_UWIRE_CS, UWIRE_READ_RISING_EDGE |
		                                    UWIRE_WRITE_RISING_EDGE |
		                                    UWIRE_CS_ACTIVE_LOW |
				                    UWIRE_FREQ_DIV_2);

	/* FIXME verify there's really a Mistral board:
	 * see if the AD7846 chip responds.
	 */

	/* NOTE:  no VREF; must ignore the temp, VBAT, and AUX sensors */
	return 0;
#else
	return -ENODEV;
#endif
}

static void osk_ts_read(u16 *data)
{
	unsigned int Rt = 0;

	data[0] = ads7846_do(MEASURE_12BIT_X);
	data[1] = ads7846_do(MEASURE_12BIT_Y); 
	data[2] = ads7846_do(MEASURE_12BIT_Z1); 
	data[3] = ads7846_do(MEASURE_12BIT_Z2); 

	// Calculate touch pressure resistance
	if (data[2]) {
		Rt = (X_PLATE_OHMS * (u32)data[0] *
		     ((u32)data[3] - (u32)data[2])) / (u32)data[2];

		Rt = (Rt + 2048) >> 12; // round up to nearest ohm
	}

	/* 
	 * Raw OSK touchscreen data values are between ~4000 and
	 * ~60000. This seems to be to large for calibration 
	 * systems (e.g. tslib). Make the values smaller.
	 */
	data[0] = data[0] >> 4;
	data[1] = data[1] >> 4;

	data[2] = Rt;
}

static void osk_ts_enable(void)
{

}

static void osk_ts_disable(void)
{

}

#ifdef MODULE
static void __exit osk_ts_remove(void)
{
	omap_free_gpio(4);
}
#endif
