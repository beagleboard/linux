/*
 * FB driver for the ILI9225 LCD Controller
 *
 * Copyright (C) 2013 Noralf Tronnes
 * Copyright (C) 2016 David Lechner <david@lechnology.com>
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
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>

#include "fbtft.h"

#define DRVNAME		"fb_ili9225"
#define WIDTH		176
#define HEIGHT		220
#define DEFAULT_GAMMA	"00 0E 5 1 0 4 B 6 A C\n" \
			"0E 00 1 5 C A 6 B 4 0"

static unsigned read_devicecode(struct fbtft_par *par)
{
	int ret;
	u8 rxbuf[8] = {0, };

	write_reg(par, 0x00);
	ret = par->fbtftops.read(par, rxbuf, 4);
	return (rxbuf[2] << 8) | rxbuf[3];
}

static int init_display(struct fbtft_par *par)
{
	unsigned devcode;

	par->fbtftops.reset(par);

	devcode = read_devicecode(par);
	fbtft_par_dbg(DEBUG_INIT_DISPLAY, par, "Device code: 0x%04X\n",
		devcode);
	if ((devcode != 0x0000) && (devcode != 0x9225))
		dev_warn(par->info->device,
			"Unrecognized Device code: 0x%04X (expected 0x9225)\n",
			devcode);

	/* 1.2 CPT 2.0 inch init sequence from ILI9225 Application Notes */

	/* *********** Start Initial Sequence ********* */
	/* set SS and NL bit */
	write_reg(par, 0x01, 0x01, 0x1C);

	/* set 1 line inversion */
	write_reg(par, 0x02, 0x01, 0x00);

	/* set GRAM write direction and BGR=1 */
	write_reg(par, 0x03, 0x10, 0x30);

	/* set the back and front porch */
	write_reg(par, 0x08, 0x08, 0x08);

	/* set RGB interface setting */
	write_reg(par, 0x0C, 0x00, 0x00);

	/* set frame rate */
	write_reg(par, 0x0F, 0x08, 0x01);

	/* GRAM horizontal Address */
	write_reg(par, 0x20, 0x00, 0x00);

	/* GRAM Vertical Address */
	write_reg(par, 0x21, 0x00, 0x00);

	/* ***********Power On sequence *************** */
	mdelay(50);

	/* SAP, BT[3:0], AP, DSTB, SLP, STB */
	write_reg(par, 0x10, 0x0A, 0x00);

	/* DC1[2:0], DC0[2:0], VC[2:0] */
	write_reg(par, 0x11, 0x10, 0x3B);

	mdelay(50);

	/* Iernal reference voltage= Vci */
	write_reg(par, 0x12, 0x11, 0x21);

	/* Set GVDD */
	write_reg(par, 0x13, 0x00, 0x66);

	/* set VCOMH/VCOML voltage */
	write_reg(par, 0x14, 0x5F, 0x60);

	/* ------------------ Set GRAM area --------------- */
	/* Gate Scan Control Register */
	write_reg(par, 0x30, 0x00, 0x00);

	/* Vertical Scroll Control 1 Register */
	write_reg(par, 0x31, 0x00, 0xDB);

	/* Vertical Scroll Control 2 Register */
	write_reg(par, 0x32, 0x00, 0x00);

	/* Vertical Scroll Control 3 Register */
	write_reg(par, 0x33, 0x00, 0x00);

	/* Partial Driving Position 1 Register */
	write_reg(par, 0x34, 0x00, 0xDB);

	/* Partial Driving Position 2 Register */
	write_reg(par, 0x35, 0x00, 0x00);

	/* Horizontal Address Start Position */
	write_reg(par, 0x36, 0x00, 0xAF);

	/* Horizontal Address End Position */
	write_reg(par, 0x37, 0x00, 0x00);

	/* Vertical Address Start Position */
	write_reg(par, 0x38, 0x00, 0xDB);

	/* Vertical Address End Position */
	write_reg(par, 0x39, 0x00, 0x00);

	mdelay(50);

	/* Display Control 1 */
	write_reg(par, 0x07, 0x10, 0x17);

	return 0;
}

#define SPLIT(n) (((n) >> 8) & 0xFF), ((n) & 0xFF)
static void set_addr_win(struct fbtft_par *par, int xs, int ys, int xe, int ye)
{
	switch (par->info->var.rotate) {
	/* R20h = Horizontal GRAM Start Address */
	/* R21h = Vertical GRAM Start Address */
	case 0:
		write_reg(par, 0x20, SPLIT(xs));
		write_reg(par, 0x21, SPLIT(ys));
		break;
	case 180:
		write_reg(par, 0x20, SPLIT(WIDTH - 1 - xs));
		write_reg(par, 0x21, SPLIT(HEIGHT - 1 - ys));
		break;
	case 270:
		write_reg(par, 0x20, SPLIT(WIDTH - 1 - ys));
		write_reg(par, 0x21, SPLIT(xs));
		break;
	case 90:
		write_reg(par, 0x20, SPLIT(ys));
		write_reg(par, 0x21, SPLIT(HEIGHT - 1 - xs));
		break;
	}
	write_reg(par, 0x22); /* Write Data to GRAM */
}
#undef SPLIT

static int set_var(struct fbtft_par *par)
{
	switch (par->info->var.rotate) {
	case 0:
		write_reg(par, 0x3, (par->bgr << 4), 0x30);
		break;
	case 270:
		write_reg(par, 0x3, (par->bgr << 4), 0x28);
		break;
	case 180:
		write_reg(par, 0x3, (par->bgr << 4), 0x00);
		break;
	case 90:
		write_reg(par, 0x3, (par->bgr << 4), 0x18);
		break;
	}
	return 0;
}

/*
  Gamma string format:
    VRP0 VRP1 RP0 RP1 KP0 KP1 KP2 KP3 KP4 KP5
    VRN0 VRN1 RN0 RN1 KN0 KN1 KN2 KN3 KN4 KN5
*/
#define CURVE(num, idx)  curves[num * par->gamma.num_values + idx]
static int set_gamma(struct fbtft_par *par, unsigned long *curves)
{
	unsigned long mask[] = {
		0x1f, 0x1f, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F,
		0x1f, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F,
	};
	int i, j;

	/* apply mask */
	for (i = 0; i < 2; i++)
		for (j = 0; j < 10; j++)
			CURVE(i, j) &= mask[i * par->gamma.num_values + j];

	write_reg(par, 0x50, CURVE(0, 5), CURVE(0, 4));
	write_reg(par, 0x51, CURVE(0, 7), CURVE(0, 6));
	write_reg(par, 0x52, CURVE(0, 9), CURVE(0, 8));
	write_reg(par, 0x53, CURVE(0, 3), CURVE(0, 2));
	write_reg(par, 0x58, CURVE(0, 1), CURVE(0, 0));

	write_reg(par, 0x54, CURVE(1, 5), CURVE(1, 4));
	write_reg(par, 0x55, CURVE(1, 7), CURVE(1, 6));
	write_reg(par, 0x56, CURVE(1, 9), CURVE(1, 8));
	write_reg(par, 0x57, CURVE(1, 3), CURVE(1, 2));
	write_reg(par, 0x59, CURVE(1, 1), CURVE(1, 0));

	return 0;
}
#undef CURVE

static struct fbtft_display display = {
	.buswidth = 8,
	.regwidth = 8,
	.width = WIDTH,
	.height = HEIGHT,
	.gamma_num = 2,
	.gamma_len = 10,
	.gamma = DEFAULT_GAMMA,
	.fbtftops = {
		.init_display = init_display,
		.set_addr_win = set_addr_win,
		.set_var = set_var,
		.set_gamma = set_gamma,
	},
};

FBTFT_REGISTER_DRIVER(DRVNAME, "ilitek,ili9225", &display);

MODULE_ALIAS("spi:" DRVNAME);
MODULE_ALIAS("platform:" DRVNAME);
MODULE_ALIAS("spi:ili9225");
MODULE_ALIAS("platform:ili9225");

MODULE_DESCRIPTION("FB driver for the ILI9225 LCD Controller");
MODULE_AUTHOR("David Lechner <david@lechnology.com>");
MODULE_LICENSE("GPL");
