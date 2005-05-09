/*
 * ts_inn1510.c - touchscreen support for OMAP1510 Innovator board
 * 
 * Copyright 2002 MontaVista Software Inc.
 * Author: MontaVista Software, Inc.
 *         	stevel@mvista.com or source@mvista.com
 *
 * The touchscreen hardware on the Innovator consists of an FPGA
 * register which is bit-banged to generate SPI-like transactions
 * to an ADS7846 touch screen controller.
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
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/arch/fpga.h>

#include "omap_ts.h"
#include "ads7846.h"

// The Touch Screen Register on Innovator FPGA
#define FPGA_TS_BCLK     (1<<0)
#define FPGA_TS_BDIN     (1<<1)
#define FPGA_TS_BCS      (1<<2)
#define FPGA_TS_BBUSY    (1<<3)
#define FPGA_TS_BOUT     (1<<4)
#define FPGA_TS_BPENUP   (1<<5)

#define X_PLATE_OHMS 419
#define Y_PLATE_OHMS 486

static int inn1510_ts_penup(void);
static int inn1510_ts_probe(struct omap_ts_t *ts);
static void inn1510_ts_read(u16 * data);
static void inn1510_ts_enable(void);
static void inn1510_ts_disable(void);
#ifdef MODULE
static void inn1510_ts_remove(void);
#endif

struct ts_device innovator1510_ts = {
        .probe   = inn1510_ts_probe,
        .read    = inn1510_ts_read,
        .enable  = inn1510_ts_enable,
        .disable = inn1510_ts_disable,
        .remove  = __exit_p(inn1510_ts_remove),
	.penup   = inn1510_ts_penup,
};

static inline u8 fpga_ts_read(void)
{
	return fpga_read(OMAP1510_FPGA_TOUCHSCREEN);
}

static inline void fpga_ts_write(u8 val)
{
	fpga_write(val, OMAP1510_FPGA_TOUCHSCREEN);
}

static inline void fpga_ts_set_bits(u8 mask)
{
	fpga_ts_write(fpga_ts_read() | mask);
}

static inline void fpga_ts_clear_bits(u8 mask)
{
	fpga_ts_write(fpga_ts_read() & ~mask);
}

static inline void CS_H(void)
{
	// EPLD inverts active low signals.
	fpga_ts_clear_bits(FPGA_TS_BCS);
}

static inline void CS_L(void)
{
	fpga_ts_set_bits(FPGA_TS_BCS);
}

static inline void SCLK_L(void)
{
	fpga_ts_clear_bits(FPGA_TS_BCLK);
}

static inline void SCLK_H(void)
{
	fpga_ts_set_bits(FPGA_TS_BCLK);
}

static inline void SDI_L(void)
{
	fpga_ts_clear_bits(FPGA_TS_BDIN);
}

static inline void SDI_H(void)
{
	fpga_ts_set_bits(FPGA_TS_BDIN);
}

static inline int BUSY(void)
{
	return (((fpga_ts_read() & FPGA_TS_BBUSY) == 0) ? 1 : 0) ;
}

static inline u8 DOUT(void)
{	 
	return ((fpga_ts_read() & FPGA_TS_BOUT) ? 1 : 0) ;
}

static u16 ads7846_do(u8 cmd)
{  
	int i;
	u16 val=0;

	SCLK_L() ;
	SDI_L();
	CS_L() ;	// enable the chip select

	// send the command to the ADS7846
	for (i=0; i<8; i++ ) {
		if (cmd & 0x80)
			SDI_H();
		else
			SDI_L();   // prepare the data on line sdi OR din

		SCLK_H() ;      // clk in the data
		cmd <<= 1 ;
		SCLK_L() ;
	}

	SDI_L();
	while (BUSY())
		;

	// now read returned data
	for (i=0 ; i<16 ; i++ ) {
		SCLK_L() ;
		
		if (i < 12) {
			val <<= 1 ;
			val |= DOUT();
		}
		SCLK_H() ;
	}

	SCLK_L() ;
	CS_H() ;   // disable the chip select

	return val;
}

static int inn1510_ts_penup(void)
{
	return ((fpga_ts_read() & FPGA_TS_BPENUP) ? 0 : 1) ;
}

static int __init inn1510_ts_probe(struct omap_ts_t *ts)
{
	if (!cpu_is_omap15xx() || !machine_is_omap_innovator())
		return -ENODEV;

	ts->irq = OMAP1510_INT_FPGA_TS;
	
	return 0;
}

static void inn1510_ts_read(u16 *data)
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

	data[2] = Rt;
}

static void inn1510_ts_enable(void)
{

}

static void inn1510_ts_disable(void)
{

}

#ifdef MODULE
static void __exit inn1510_ts_remove(void)
{
	/* Nothing to do here */
}
#endif
