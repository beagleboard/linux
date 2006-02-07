/*
 * input/touchscreen/omap/ts_hx.c
 * touchscreen support for OMAP H3 and H2  boards
 *
 * Copyright (c) 2002 MontaVista Software Inc.
 * Copyright (c) 2004 Texas Instruments, Inc.
 *
 * Assembled using driver code copyright the companies above.
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
 *
 * History:
 * 9/12/2004  	Srinath Modified and integrated  H2 and H3 code
 *
 */

#include <linux/input.h>
#include <linux/device.h>
#include <asm/mach-types.h>
#include <asm/arch/gpio.h>
#include <asm/arch/mux.h>
#include <asm/arch/hardware.h>
#include <asm/hardware/tsc2101.h>

#include "../drivers/ssi/omap-tsc2101.h"
#include "omap_ts.h"

#define	H2_GPIO_NUM		4
#define	H3_GPIO_NUM		48

#define OMAP_TSC2101_XRES		       500
#define TOUCHSCREEN_DATA_REGISTERS_PAGE	 0x0
#define TOUCHSCREEN_CONTROL_REGISTERS_PAGE      0x1
#define OMAP_TSC2101_READ_MAX		   0x4
#define TSC2101_GETSTATUS(ret)		  (((ret) >> 11) & 0x1)
#define TSC2101_MASKVAL			 0xFFF
#define TSC2101_PRESSUREVAL(x)		  ((x) << 12)

static int hx_ts_penup(void);
static int hx_ts_probe(struct omap_ts_t *ts);
static void hx_ts_read(u16 * data);
static void hx_ts_enable(void);
static void hx_ts_disable(void);
#ifdef	MODULE
static void hx_ts_remove(void);
#endif

struct ts_device hx_ts = {
	.probe 		= hx_ts_probe,
	.read 		= hx_ts_read,
	.enable 	= hx_ts_enable,
	.disable 	= hx_ts_disable,
	.remove 	= __exit_p(hx_ts_remove),
	.penup 		= hx_ts_penup,
};

static int hx_ts_penup(void)
{
	int ret = 0;
	/* Read the status register */
	ret = omap_tsc2101_read(TOUCHSCREEN_CONTROL_REGISTERS_PAGE,
				TSC2101_TS_STATUS);
	/* Check for availability of data in status register */
	ret = TSC2101_GETSTATUS(ret);
	return !ret;

}

static int __init hx_ts_probe(struct omap_ts_t *ts)
{
	unsigned	gpio;

	if (machine_is_omap_h2()) {
		gpio = H2_GPIO_NUM;
		omap_cfg_reg(P20_1610_GPIO4);
	} else if (machine_is_omap_h3()) {
		gpio = H3_GPIO_NUM;
		omap_cfg_reg(W19_1610_GPIO48);
	} else
		return -ENODEV;

	ts->irq = OMAP_GPIO_IRQ(gpio);
	if (omap_request_gpio(gpio) != 0) {
		printk(KERN_ERR "hX_ts_init.c: Could not reserve GPIO!\n");
		return -EINVAL;
	};

	omap_set_gpio_direction(gpio, 1);
	ts->irq_type = SA_TRIGGER_FALLING;
	return 0;
}

static void hx_ts_read(u16 * values)
{
	s32 t, p = 0;
	int i;

	/* Read X, Y, Z1 and Z2 */
	omap_tsc2101_reads(TOUCHSCREEN_DATA_REGISTERS_PAGE, TSC2101_TS_X,
			   values, OMAP_TSC2101_READ_MAX);

	for (i = 0; i < OMAP_TSC2101_READ_MAX; i++)
		values[i] &= TSC2101_MASKVAL;

	/* Calculate Pressure */
	if (values[TSC2101_TS_Z1] != 0) {
		t = ((OMAP_TSC2101_XRES * values[TSC2101_TS_X]) *
		     (values[TSC2101_TS_Z2] - values[TSC2101_TS_Z1]));
		p = t / (u32) (TSC2101_PRESSUREVAL(values[TSC2101_TS_Z1]));
		if (p < 0)
			p = 0;
	}

	values[TSC2101_TS_Z1] = p;
}

static void hx_ts_enable(void)
{
	int ret = omap_tsc2101_enable();
	if (ret) {
		printk(KERN_ERR "FAILED TO INITIALIZE TSC CODEC\n");
		return;
	}

	/* PINTDAV is data available only */
	omap_tsc2101_write(TOUCHSCREEN_CONTROL_REGISTERS_PAGE,
			   TSC2101_TS_STATUS, TSC2101_DATA_AVAILABLE);
	/* disable buffer mode */
	omap_tsc2101_write(TOUCHSCREEN_CONTROL_REGISTERS_PAGE,
			   TSC2101_TS_BUFFER_CTRL, TSC2101_BUFFERMODE_DISABLE);
	/* use internal reference, 100 usec power-up delay,
	 *	  * power down between conversions, 1.25V internal reference */
	omap_tsc2101_write(TOUCHSCREEN_CONTROL_REGISTERS_PAGE,
			   TSC2101_TS_REF_CTRL, TSC2101_REF_POWERUP);
	/* enable touch detection, 84usec precharge time, 32 usec sense time */
	omap_tsc2101_write(TOUCHSCREEN_CONTROL_REGISTERS_PAGE,
			   TSC2101_TS_CONFIG_CTRL, TSC2101_ENABLE_TOUCHDETECT);
	/* 3 msec conversion delays  */
	omap_tsc2101_write(TOUCHSCREEN_CONTROL_REGISTERS_PAGE,
			   TSC2101_TS_PROG_DELAY, TSC2101_PRG_DELAY);
	/*
	 * TSC2101-controlled conversions
	 * 12-bit samples
	 * continuous X,Y,Z1,Z2 scan mode
	 * average (mean) 4 samples per coordinate
	 * 1 MHz internal conversion clock
	 * 500 usec panel voltage stabilization delay
	 */
	omap_tsc2101_write(TOUCHSCREEN_CONTROL_REGISTERS_PAGE,
			   TSC2101_TS_ADC_CTRL, TSC2101_ADC_CONTROL);

	return;

}

static void hx_ts_disable(void)
{
	/* stop conversions and power down */
	omap_tsc2101_write(TOUCHSCREEN_CONTROL_REGISTERS_PAGE,
			   TSC2101_TS_ADC_CTRL, TSC2101_ADC_POWERDOWN);
	omap_tsc2101_disable();
}

#ifdef	MODULE
static void __exit hx_ts_remove(void)
{
	if (machine_is_omap_h2())
		omap_free_gpio(H2_GPIO_NUM);
	else if (machine_is_omap_h3())
		omap_free_gpio(H3_GPIO_NUM);
}
#endif
