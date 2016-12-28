/*
 * ST7735R LCD controller
 *
 * Copyright 2016 Noralf Trønnes
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __LINUX_ST7735R_H
#define __LINUX_ST7735R_H

#define ST7735R_FRMCTR1		0xB1
#define ST7735R_FRMCTR2		0xB2
#define ST7735R_FRMCTR3		0xB3
#define ST7735R_INVCTR		0xB4

#define ST7735R_PWCTR1		0xC0
#define ST7735R_PWCTR2		0xC1
#define ST7735R_PWCTR3		0xC2
#define ST7735R_PWCTR4		0xC3
#define ST7735R_PWCTR5		0xC4
#define ST7735R_VMCTR1		0xC5
#define ST7735R_VMOFCTR		0xC7

#define ST7735R_WRID2		0xD1
#define ST7735R_WRID3		0xD2
#define ST7735R_NVCTR1		0xD9
#define ST7735R_RDID1		0xDA
#define ST7735R_RDID2		0xDB
#define ST7735R_RDID3		0xDC
#define ST7735R_NVCTR2		0xDE
#define ST7735R_NVCTR3		0xDF

#define ST7735R_GAMCTRP1	0xE0
#define ST7735R_GAMCTRN1	0xE1

#define ST7735R_MADCTL_MV	BIT(5)
#define ST7735R_MADCTL_MX	BIT(6)
#define ST7735R_MADCTL_MY	BIT(7)

#endif /* __LINUX_ST7735R_H */
