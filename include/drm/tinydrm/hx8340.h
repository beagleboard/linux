/*
 * HX8340 LCD controller
 *
 * Copyright 2016 Noralf Trønnes
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __LINUX_HX8340_H
#define __LINUX_HX8340_H

#define HX8340_SETOSC      0xB0
#define HX8340_SETPWCTR1   0xB1
#define HX8340_SETPWCTR2   0xB2
#define HX8340_SETPWCTR3   0xB3
#define HX8340_SETPWCTR4   0xB4
#define HX8340_SETPWCTR5   0xB5
#define HX8340_SETDISCTRL  0xB6
#define HX8340_SETFRMCTRL  0xB7
#define HX8340_SETDISCYCC  0xB8
#define HX8340_SETINVCTRL  0xB9
#define HX8340_RGBBPCTR    0xBA
#define HX8340_SETRGBIF    0xBB
#define HX8340_SETDODC     0xBC
#define HX8340_SETINTMODE  0xBD
#define HX8340_SETPANEL    0xBE

#define HX8340_SETONOFF    0xC0
#define HX8340_SETEXTCMD   0xC1
#define HX8340_SETGAMMAP   0xC2
#define HX8340_SETGAMMAN   0xC3
#define HX8340_SETOTP      0xC7

#define HX8340_RDID1       0xDA
#define HX8340_RDID2       0xDB
#define HX8340_RDID3       0xDC

#define HX8340_MADCTL_BGR  BIT(3)
#define HX8340_MADCTL_MV   BIT(5)
#define HX8340_MADCTL_MX   BIT(6)
#define HX8340_MADCTL_MY   BIT(7)

#endif /* __LINUX_HX8340_H */
