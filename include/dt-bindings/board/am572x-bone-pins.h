/*
 * Copyright (C) 2020 Deepak Khatri <deepaklorkhatri7@gmail.com>
 * See Cape Interface Spec page for more info on Bone Buses
 * https://elinux.org/Beagleboard:BeagleBone_cape_interface_spec
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _DT_BINDINGS_BOARD_AM572X_BONE_PINS_H
#define _DT_BINDINGS_BOARD_AM572X_BONE_PINS_H

#define bb_device 1
#define board_soc AM572X

#define gpio_P8_03 &gpio1 24
#define gpio_P8_04 &gpio1 25
#define gpio_P8_05 &gpio7 1
#define gpio_P8_06 &gpio7 2
#define gpio_P8_07 &gpio6 5
#define gpio_P8_08 &gpio6 6
#define gpio_P8_09 &gpio6 18
#define gpio_P8_10 &gpio6 4
#define gpio_P8_11 &gpio3 11
#define gpio_P8_12 &gpio3 10
#define gpio_P8_13 &gpio4 11
#define gpio_P8_14 &gpio4 13
#define gpio_P8_15 &gpio4 3
#define gpio_P8_16 &gpio4 29
#define gpio_P8_17 &gpio8 18
#define gpio_P8_18 &gpio4 9
#define gpio_P8_19 &gpio4 10
#define gpio_P8_20 &gpio6 30
#define gpio_P8_21 &gpio6 29
#define gpio_P8_22 &gpio1 23
#define gpio_P8_23 &gpio1 22
#define gpio_P8_24 &gpio7 0
#define gpio_P8_25 &gpio6 31
#define gpio_P8_26 &gpio4 28
#define gpio_P8_27 &gpio4 23
#define gpio_P8_28 &gpio4 19
#define gpio_P8_29 &gpio4 22
#define gpio_P8_30 &gpio4 20
#define gpio_P8_31 &gpio8 14
#define gpio_P8_32 &gpio8 15
#define gpio_P8_33 &gpio8 13
#define gpio_P8_34 &gpio8 11
#define gpio_P8_35 &gpio8 12
#define gpio_P8_36 &gpio8 10
#define gpio_P8_37 &gpio8 8
#define gpio_P8_38 &gpio8 9
#define gpio_P8_39 &gpio8 6
#define gpio_P8_40 &gpio8 7
#define gpio_P8_41 &gpio8 4
#define gpio_P8_42 &gpio8 5
#define gpio_P8_43 &gpio8 2
#define gpio_P8_44 &gpio8 3
#define gpio_P8_45 &gpio8 0
#define gpio_P8_46 &gpio8 1
#define gpio_P9_11 &gpio8 17
#define gpio_P9_12 &gpio5 0
#define gpio_P9_13 &gpio6 12
#define gpio_P9_14 &gpio4 25
#define gpio_P9_15 &gpio3 12
#define gpio_P9_16 &gpio4 26
#define gpio_P9_17 &gpio7 17
#define gpio_P9_18 &gpio7 16
#define gpio_P9_19 &gpio7 3
#define gpio_P9_20 &gpio7 4
#define gpio_P9_21 &gpio3 3
#define gpio_P9_22 &gpio6 19
#define gpio_P9_23 &gpio7 11
#define gpio_P9_24 &gpio6 15
#define gpio_P9_25 &gpio6 17
#define gpio_P9_26 &gpio6 14
#define gpio_P9_27 &gpio4 15
#define gpio_P9_28 &gpio4 17
#define gpio_P9_29 &gpio5 11
#define gpio_P9_30 &gpio5 12
#define gpio_P9_31 &gpio5 10
#define gpio_P9_41 &gpio6 20
#define gpio_P9_42 &gpio4 18

#define P8_03(mode) DRA7XX_CORE_IOPAD(0x379C, mode)  /* AB8: mmc3_dat6 */
#define P8_04(mode) DRA7XX_CORE_IOPAD(0x37A0, mode)  /* AB5: mmc3_dat7 */
#define P8_05(mode) DRA7XX_CORE_IOPAD(0x378C, mode)  /* AC9: mmc3_dat2 */
#define P8_06(mode) DRA7XX_CORE_IOPAD(0x3790, mode)  /* AC3: mmc3_dat3 */
#define P8_07(mode) DRA7XX_CORE_IOPAD(0x36EC, mode)  /* G14: mcasp1_axr14 */
#define P8_08(mode) DRA7XX_CORE_IOPAD(0x36F0, mode)  /* F14: mcasp1_axr15 */
#define P8_09(mode) DRA7XX_CORE_IOPAD(0x3698, mode)  /* E17: xref_clk1 */
#define P8_10(mode) DRA7XX_CORE_IOPAD(0x36E8, mode)  /* A13: mcasp1_axr13 */
#define P8_11(mode) DRA7XX_CORE_IOPAD(0x3510, mode)  /* AH4: vin1a_d7 */
#define P8_12(mode) DRA7XX_CORE_IOPAD(0x350C, mode)  /* AG6: vin1a_d6 */
#define P8_13(mode) DRA7XX_CORE_IOPAD(0x3590, mode)  /* D3: vin2a_d10 */
#define P8_14(mode) DRA7XX_CORE_IOPAD(0x3598, mode)  /* D5: vin2a_d12 */
#define P8_15A(mode) DRA7XX_CORE_IOPAD(0x3570, mode) /* D1: vin2a_d2 */
#define P8_15B(mode) DRA7XX_CORE_IOPAD(0x35B4, mode) /* A3: vin2a_d19 */
#define P8_16(mode) DRA7XX_CORE_IOPAD(0x35BC, mode)  /* B4: vin2a_d21 */
#define P8_17(mode) DRA7XX_CORE_IOPAD(0x3624, mode)  /* A7: vout1_d18 */
#define P8_18(mode) DRA7XX_CORE_IOPAD(0x3588, mode)  /* F5: vin2a_d8 */
#define P8_19(mode) DRA7XX_CORE_IOPAD(0x358C, mode)  /* E6: vin2a_d9 */
#define P8_20(mode) DRA7XX_CORE_IOPAD(0x3780, mode)  /* AC4: mmc3_cmd */
#define P8_21(mode) DRA7XX_CORE_IOPAD(0x377C, mode)  /* AD4: mmc3_clk */
#define P8_22(mode) DRA7XX_CORE_IOPAD(0x3798, mode)  /* AD6: mmc3_dat5 */
#define P8_23(mode) DRA7XX_CORE_IOPAD(0x3794, mode)  /* AC8: mmc3_dat4 */
#define P8_24(mode) DRA7XX_CORE_IOPAD(0x3788, mode)  /* AC6: mmc3_dat1 */
#define P8_25(mode) DRA7XX_CORE_IOPAD(0x3784, mode)  /* AC7: mmc3_dat0 */
#define P8_26(mode) DRA7XX_CORE_IOPAD(0x35B8, mode)  /* B3: vin2a_d20 */
#define P8_27A(mode) DRA7XX_CORE_IOPAD(0x35D8, mode) /* E11: vout1_vsync */
#define P8_27B(mode) DRA7XX_CORE_IOPAD(0x3628, mode) /* A8: vout1_d19 */
#define P8_28A(mode) DRA7XX_CORE_IOPAD(0x35C8, mode) /* D11: vout1_clk */
#define P8_28B(mode) DRA7XX_CORE_IOPAD(0x362C, mode) /* C9: vout1_d20 */
#define P8_29A(mode) DRA7XX_CORE_IOPAD(0x35D4, mode) /* C11: vout1_hsync */
#define P8_29B(mode) DRA7XX_CORE_IOPAD(0x3630, mode) /* A9: vout1_d21 */
#define P8_30A(mode) DRA7XX_CORE_IOPAD(0x35CC, mode) /* B10: vout1_de */
#define P8_30B(mode) DRA7XX_CORE_IOPAD(0x3634, mode) /* B9: vout1_d22 */
#define P8_31A(mode) DRA7XX_CORE_IOPAD(0x3614, mode) /* C8: vout1_d14 */
#define P8_31B(mode) DRA7XX_CORE_IOPAD(0x373C, mode) /* G16: mcasp4_axr0 */
#define P8_32A(mode) DRA7XX_CORE_IOPAD(0x3618, mode) /* C7: vout1_d15 */
#define P8_32B(mode) DRA7XX_CORE_IOPAD(0x3740, mode) /* D17: mcasp4_axr1 */
#define P8_33A(mode) DRA7XX_CORE_IOPAD(0x3610, mode) /* C6: vout1_d13 */
#define P8_33B(mode) DRA7XX_CORE_IOPAD(0x34E8, mode) /* AF9: vin1a_fld0 */
#define P8_34A(mode) DRA7XX_CORE_IOPAD(0x3608, mode) /* D8: vout1_d11 */
#define P8_34B(mode) DRA7XX_CORE_IOPAD(0x3564, mode) /* G6: vin2a_vsync0 */
#define P8_35A(mode) DRA7XX_CORE_IOPAD(0x360C, mode) /* A5: vout1_d12 */
#define P8_35B(mode) DRA7XX_CORE_IOPAD(0x34E4, mode) /* AD9: vin1a_de0 */
#define P8_36A(mode) DRA7XX_CORE_IOPAD(0x3604, mode) /* D7: vout1_d10 */
#define P8_36B(mode) DRA7XX_CORE_IOPAD(0x3568, mode) /* F2: vin2a_d0 */
#define P8_37A(mode) DRA7XX_CORE_IOPAD(0x35FC, mode) /* E8: vout1_d8 */
#define P8_37B(mode) DRA7XX_CORE_IOPAD(0x3738, mode) /* A21: mcasp4_fsx */
#define P8_38A(mode) DRA7XX_CORE_IOPAD(0x3600, mode) /* D9: vout1_d9 */
#define P8_38B(mode) DRA7XX_CORE_IOPAD(0x3734, mode) /* C18: mcasp4_aclkx */
#define P8_39(mode) DRA7XX_CORE_IOPAD(0x35F4, mode)  /* F8: vout1_d6 */
#define P8_40(mode) DRA7XX_CORE_IOPAD(0x35F8, mode)  /* E7: vout1_d7 */
#define P8_41(mode) DRA7XX_CORE_IOPAD(0x35EC, mode)  /* E9: vout1_d4 */
#define P8_42(mode) DRA7XX_CORE_IOPAD(0x35F0, mode)  /* F9: vout1_d5 */
#define P8_43(mode) DRA7XX_CORE_IOPAD(0x35E4, mode)  /* F10: vout1_d2 */
#define P8_44(mode) DRA7XX_CORE_IOPAD(0x35E8, mode)  /* G11: vout1_d3 */
#define P8_45A(mode) DRA7XX_CORE_IOPAD(0x35DC, mode) /* F11: vout1_d0 */
#define P8_45B(mode) DRA7XX_CORE_IOPAD(0x361C, mode) /* B7: vout1_d16 */
#define P8_46A(mode) DRA7XX_CORE_IOPAD(0x35E0, mode) /* G10: vout1_d1 */
#define P8_46B(mode) DRA7XX_CORE_IOPAD(0x3638, mode) /* A10: vout1_d23 */
#define P9_11A(mode) DRA7XX_CORE_IOPAD(0x372C, mode) /* B19: mcasp3_axr0 */
#define P9_11B(mode) DRA7XX_CORE_IOPAD(0x3620, mode) /* B8: vout1_d17 */
#define P9_12(mode) DRA7XX_CORE_IOPAD(0x36AC, mode)  /* B14: mcasp1_aclkr */
#define P9_13A(mode) DRA7XX_CORE_IOPAD(0x3730, mode)  /* C17: mcasp3_axr1 */
#define P9_13B(mode) DRA7XX_CORE_IOPAD(0x3680, mode)  /* AB10: usb1_drvvbus */
#define P9_14(mode) DRA7XX_CORE_IOPAD(0x35AC, mode)  /* D6: vin2a_d17 */
#define P9_15(mode) DRA7XX_CORE_IOPAD(0x3514, mode)  /* AG4: vin1a_d8 */
#define P9_16(mode) DRA7XX_CORE_IOPAD(0x35B0, mode)  /* C5: vin2a_d18 */
#define P9_17A(mode) DRA7XX_CORE_IOPAD(0x37CC, mode) /* B24: spi2_cs0 */
#define P9_17B(mode) DRA7XX_CORE_IOPAD(0x36B8, mode) /* F12: mcasp1_axr1 */
#define P9_18A(mode) DRA7XX_CORE_IOPAD(0x37C8, mode) /* G17: spi2_d0 */
#define P9_18B(mode) DRA7XX_CORE_IOPAD(0x36B4, mode) /* G12: mcasp1_axr0 */
#define P9_19A(mode) DRA7XX_CORE_IOPAD(0x3440, mode) /* R6: gpmc_a0.i2c4_scl */
#define P9_19B(mode) DRA7XX_CORE_IOPAD(0x357C, mode) /* F4: vin2a_d5.pr1_pru1_gpi2 */
#define P9_20A(mode) DRA7XX_CORE_IOPAD(0x3444, mode) /* T9: gpmc_a1.i2c4_sda */
#define P9_20B(mode) DRA7XX_CORE_IOPAD(0x3578, mode) /* D2: vin2a_d4.pr1_pru1_gpi1 */
#define P9_21A(mode) DRA7XX_CORE_IOPAD(0x34F0, mode) /* AF8: vin1a_vsync0 */
#define P9_21B(mode) DRA7XX_CORE_IOPAD(0x37C4, mode) /* B22: spi2_d1 */
#define P9_22A(mode) DRA7XX_CORE_IOPAD(0x369C, mode) /* B26: xref_clk2 */
#define P9_22B(mode) DRA7XX_CORE_IOPAD(0x37C0, mode) /* A26: spi2_sclk */
#define P9_23(mode) DRA7XX_CORE_IOPAD(0x37B4, mode)  /* A22: spi1_cs1 */
#define P9_24(mode) DRA7XX_CORE_IOPAD(0x368C, mode)  /* F20: gpio6_15 */
#define P9_25(mode) DRA7XX_CORE_IOPAD(0x3694, mode)  /* D18: xref_clk0 */
#define P9_26A(mode) DRA7XX_CORE_IOPAD(0x3688, mode) /* E21: gpio6_14 */
#define P9_26B(mode) DRA7XX_CORE_IOPAD(0x3544, mode) /* AE2: vin1a_d20 */
#define P9_27A(mode) DRA7XX_CORE_IOPAD(0x35A0, mode) /* C3: vin2a_d14 */
#define P9_27B(mode) DRA7XX_CORE_IOPAD(0x36B0, mode) /* J14: mcasp1_fsr */
#define P9_28(mode) DRA7XX_CORE_IOPAD(0x36E0, mode)  /* A12: mcasp1_axr11 */
#define P9_29A(mode) DRA7XX_CORE_IOPAD(0x36D8, mode) /* A11: mcasp1_axr9 */
#define P9_29B(mode) DRA7XX_CORE_IOPAD(0x36A8, mode) /* D14: mcasp1_fsx */
#define P9_30(mode) DRA7XX_CORE_IOPAD(0x36DC, mode)  /* B13: mcasp1_axr10 */
#define P9_31A(mode) DRA7XX_CORE_IOPAD(0x36D4, mode) /* B12: mcasp1_axr8 */
#define P9_31B(mode) DRA7XX_CORE_IOPAD(0x36A4, mode) /* C14: mcasp1_aclkx */
#define P9_41A(mode) DRA7XX_CORE_IOPAD(0x36A0, mode) /* C23: xref_clk3 */
#define P9_41B(mode) DRA7XX_CORE_IOPAD(0x3580, mode) /* C1: vin2a_d6 */
#define P9_42A(mode) DRA7XX_CORE_IOPAD(0x36E4, mode) /* E14: mcasp1_axr12 */
#define P9_42B(mode) DRA7XX_CORE_IOPAD(0x359C, mode) /* C2: vin2a_d13 */

#endif