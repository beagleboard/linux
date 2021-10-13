// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Deepak Khatri <deepaklorkhatri7@gmail.com>
 * Copyright (C) 2021 Jason Kridner <jkridner@beagleboard.org>
 * See Cape Interface Spec page for more info on Bone Buses
 * https://elinux.org/Beagleboard:BeagleBone_cape_interface_spec
 */

#ifndef _DT_BINDINGS_BOARD_TDA4VM_BONE_PINS_H
#define _DT_BINDINGS_BOARD_TDA4VM_BONE_PINS_H

#define bb_device 1
#define board_soc TDA4VM

#define gpio_P8_03 &main_gpio0 20
#define gpio_P8_04 &main_gpio0 48
#define gpio_P8_05 &main_gpio0 33
#define gpio_P8_06 &main_gpio0 34
#define gpio_P8_07 &main_gpio0 15
#define gpio_P8_08 &main_gpio0 14
#define gpio_P8_09 &main_gpio0 17
#define gpio_P8_10 &main_gpio0 16
#define gpio_P8_11 &main_gpio0 60
#define gpio_P8_12 &main_gpio0 59
#define gpio_P8_13 &main_gpio0 89
#define gpio_P8_14 &main_gpio0 75
#define gpio_P8_15 &main_gpio0 61
#define gpio_P8_16 &main_gpio0 62
#define gpio_P8_17 &main_gpio0 3
#define gpio_P8_18 &main_gpio0 4
#define gpio_P8_19 &main_gpio0 88
#define gpio_P8_20 &main_gpio0 76
#define gpio_P8_21 &main_gpio0 30
#define gpio_P8_22 &main_gpio0 5
#define gpio_P8_23 &main_gpio0 31
#define gpio_P8_24 &main_gpio0 6
#define gpio_P8_25 &main_gpio0 35
#define gpio_P8_26 &main_gpio0 51
#define gpio_P8_27 &main_gpio0 71
#define gpio_P8_28 &main_gpio0 72
#define gpio_P8_29 &main_gpio0 73
#define gpio_P8_30 &main_gpio0 74
#define gpio_P8_31 &main_gpio0 32
#define gpio_P8_32 &main_gpio0 26
#define gpio_P8_33 &main_gpio0 25
#define gpio_P8_34 &main_gpio0 7
#define gpio_P8_35 &main_gpio0 24
#define gpio_P8_36 &main_gpio0 8
#define gpio_P8_37 &main_gpio0 106
#define gpio_P8_38 &main_gpio0 105
#define gpio_P8_39 &main_gpio0 69
#define gpio_P8_40 &main_gpio0 70
#define gpio_P8_41 &main_gpio0 67
#define gpio_P8_42 &main_gpio0 68
#define gpio_P8_43 &main_gpio0 65
#define gpio_P8_44 &main_gpio0 66
#define gpio_P8_45 &main_gpio0 79
#define gpio_P8_46 &main_gpio0 80
#define gpio_P9_11 &main_gpio0 1
#define gpio_P9_12 &main_gpio0 45
#define gpio_P9_13 &main_gpio0 2
#define gpio_P9_14 &main_gpio0 93
#define gpio_P9_15 &main_gpio0 47
#define gpio_P9_16 &main_gpio0 94
#define gpio_P9_17 &main_gpio0 28
#define gpio_P9_18 &main_gpio0 40
#define gpio_P9_19 &main_gpio1 1
#define gpio_P9_20 &main_gpio1 2
#define gpio_P9_21 &main_gpio0 39
#define gpio_P9_22 &main_gpio0 38
#define gpio_P9_23 &main_gpio0 10
#define gpio_P9_24 &main_gpio0 13
#define gpio_P9_25 &main_gpio0 127
#define gpio_P9_26 &main_gpio0 12
#define gpio_P9_27 &main_gpio0 46
#define gpio_P9_28 &main_gpio1 11
#define gpio_P9_29 &main_gpio0 53
#define gpio_P9_30 &main_gpio0 44
#define gpio_P9_31 &main_gpio0 52
#define gpio_P9_33 &main_gpio0 50
#define gpio_P9_35 &main_gpio0 55
#define gpio_P9_36 &main_gpio0 56
#define gpio_P9_37 &main_gpio0 57
#define gpio_P9_38 &main_gpio0 58
#define gpio_P9_39 &main_gpio0 54
#define gpio_P9_40 &main_gpio0 81
#define gpio_P9_41 &main_gpio1 0
#define gpio_P9_42 &main_gpio0 123

#define P8_03(mode, mux)  J721E_IOPAD(0x054, mode, mux)       /* AH21 */
#define P8_04(mode, mux)  J721E_IOPAD(0x0C4, mode, mux)       /* AC29 - BOOTMODE2 */
#define P8_05(mode, mux)  J721E_IOPAD(0x088, mode, mux)       /* AH25 */
#define P8_06(mode, mux)  J721E_IOPAD(0x08C, mode, mux)       /* AG25 */
#define P8_07(mode, mux)  J721E_IOPAD(0x03C, mode, mux)       /* AD24 */
#define P8_08(mode, mux)  J721E_IOPAD(0x038, mode, mux)       /* AG24 */
#define P8_09(mode, mux)  J721E_IOPAD(0x044, mode, mux)       /* AE24 */
#define P8_10(mode, mux)  J721E_IOPAD(0x040, mode, mux)       /* AC24 */
#define P8_11(mode, mux)  J721E_IOPAD(0x0F4, mode, mux)       /* AB24 - BOOTMODE7 */
#define P8_12(mode, mux)  J721E_IOPAD(0x0F0, mode, mux)       /* AH28 */
#define P8_13(mode, mux)  J721E_IOPAD(0x168, mode, mux)       /* V27 */
#define P8_14(mode, mux)  J721E_IOPAD(0x130, mode, mux)       /* AF27 */
#define P8_15(mode, mux)  J721E_IOPAD(0x0F8, mode, mux)       /* AB29 */
#define P8_16(mode, mux)  J721E_IOPAD(0x0FC, mode, mux)       /* AB28 */
#define P8_17(mode, mux)  J721E_IOPAD(0x00C, mode, mux)       /* AF22 */
#define P8_18(mode, mux)  J721E_IOPAD(0x010, mode, mux)       /* AJ23 */
#define P8_19(mode, mux)  J721E_IOPAD(0x164, mode, mux)       /* V29 */
#define P8_20(mode, mux)  J721E_IOPAD(0x134, mode, mux)       /* AF26 */
#define P8_21(mode, mux)  J721E_IOPAD(0x07C, mode, mux)       /* AF21 */
#define P8_22(mode, mux)  J721E_IOPAD(0x014, mode, mux)       /* AH23 */
#define P8_23(mode, mux)  J721E_IOPAD(0x080, mode, mux)       /* AB23 */
#define P8_24(mode, mux)  J721E_IOPAD(0x018, mode, mux)       /* AD20 - BOOTMODE0 */
#define P8_25(mode, mux)  J721E_IOPAD(0x090, mode, mux)       /* AH26 */
#define P8_26(mode, mux)  J721E_IOPAD(0x0D0, mode, mux)       /* AC27 */
#define P8_27(mode, mux)  J721E_IOPAD(0x120, mode, mux)       /* AA28 */
#define P8_28(mode, mux)  J721E_IOPAD(0x124, mode, mux)       /* Y24 */
#define P8_29(mode, mux)  J721E_IOPAD(0x128, mode, mux)       /* AA25 */
#define P8_30(mode, mux)  J721E_IOPAD(0x12C, mode, mux)       /* AG26 */
#define P8_31A(mode, mux) J721E_IOPAD(0x084, mode, mux)       /* AJ25 */
#define P8_31B(mode, mux) J721E_IOPAD(0x100, mode, mux)       /* AE29 */
#define P8_32A(mode, mux) J721E_IOPAD(0x06C, mode, mux)       /* AG21 */
#define P8_32B(mode, mux) J721E_IOPAD(0x104, mode, mux)       /* AD28 */
#define P8_33A(mode, mux) J721E_IOPAD(0x068, mode, mux)       /* AH24 */
#define P8_33B(mode, mux) J721E_IOPAD(0x1C0, mode, mux)       /* AA2 */
#define P8_34(mode, mux)  J721E_IOPAD(0x01C, mode, mux)       /* AD22 */
#define P8_35A(mode, mux) J721E_IOPAD(0x064, mode, mux)       /* AD23 */
#define P8_35B(mode, mux) J721E_IOPAD(0x1D4, mode, mux)       /* Y3 */
#define P8_36(mode, mux)  J721E_IOPAD(0x020, mode, mux)       /* AE20 */
#define P8_37A(mode, mux) J721E_IOPAD(0x02C, mode, mux)       /* AD21 */
#define P8_37B(mode, mux) J721E_IOPAD(0x1AC, mode, mux)       /* Y27 */
#define P8_38A(mode, mux) J721E_IOPAD(0x024, mode, mux)       /* AJ20 */
#define P8_38B(mode, mux) J721E_IOPAD(0x1A8, mode, mux)       /* Y29 */
#define P8_39(mode, mux)  J721E_IOPAD(0x118, mode, mux)       /* AC26 */
#define P8_40(mode, mux)  J721E_IOPAD(0x11C, mode, mux)       /* AA24 */
#define P8_41(mode, mux)  J721E_IOPAD(0x110, mode, mux)       /* AD29 */
#define P8_42(mode, mux)  J721E_IOPAD(0x114, mode, mux)       /* AB27 - BOOTMODE6 */
#define P8_43(mode, mux)  J721E_IOPAD(0x108, mode, mux)       /* AD27 */
#define P8_44(mode, mux)  J721E_IOPAD(0x10C, mode, mux)       /* AC25 */
#define P8_45(mode, mux)  J721E_IOPAD(0x140, mode, mux)       /* AG29 */
#define P8_46(mode, mux)  J721E_IOPAD(0x144, mode, mux)       /* Y25 - BOOTMODE3 */
#define P9_11(mode, mux)  J721E_IOPAD(0x004, mode, mux)       /* AC23 */
#define P9_12(mode, mux)  J721E_IOPAD(0x0B8, mode, mux)       /* AE27 */
#define P9_13(mode, mux)  J721E_IOPAD(0x008, mode, mux)       /* AG22 */
#define P9_14(mode, mux)  J721E_IOPAD(0x178, mode, mux)       /* U27 */
#define P9_15(mode, mux)  J721E_IOPAD(0x0C0, mode, mux)       /* AD25 */
#define P9_16(mode, mux)  J721E_IOPAD(0x17C, mode, mux)       /* U24 */
#define P9_17A(mode, mux) J721E_IOPAD(0x074, mode, mux)       /* AC21 */
#define P9_17B(mode, mux) J721E_IOPAD(0x1D0, mode, mux)       /* AA3 */
#define P9_18A(mode, mux) J721E_IOPAD(0x0A4, mode, mux)       /* AH22 */
#define P9_18B(mode, mux) J721E_IOPAD(0x1E4, mode, mux)       /* Y2 */
#define P9_19A(mode, mux) J721E_IOPAD(0x208, mode, mux)       /* W5 */
#define P9_19B(mode, mux) J721E_IOPAD(0x13C, mode, mux)       /* AF29 */
#define P9_20A(mode, mux) J721E_IOPAD(0x20C, mode, mux)       /* W6 */
#define P9_20B(mode, mux) J721E_IOPAD(0x138, mode, mux)       /* AE25 */
#define P9_21A(mode, mux) J721E_IOPAD(0x0A0, mode, mux)       /* AJ22 */
#define P9_21B(mode, mux) J721E_IOPAD(0x16C, mode, mux)       /* U28 */
#define P9_22A(mode, mux) J721E_IOPAD(0x09C, mode, mux)       /* AC22 */
#define P9_22B(mode, mux) J721E_IOPAD(0x170, mode, mux)       /* U29 */
#define P9_23(mode, mux)  J721E_IOPAD(0x02B, mode, mux)       /* AG20 */
#define P9_24A(mode, mux) J721E_IOPAD(0x1E0, mode, mux)       /* Y5 */
#define P9_24B(mode, mux) J721E_IOPAD(0x034, mode, mux)       /* AJ24 */
#define P9_25A(mode, mux) J721E_IOPAD(0x200, mode, mux)       /* AC4 */
#define P9_25B(mode, mux) J721E_IOPAD(0x1A4, mode, mux)       /* W26 */
#define P9_26A(mode, mux) J721E_IOPAD(0x1DC, mode, mux)       /* Y1 */
#define P9_26B(mode, mux) J721E_IOPAD(0x030, mode, mux)       /* AF24 */
#define P9_27A(mode, mux) J721E_IOPAD(0x0BC, mode, mux)       /* AD26 */
#define P9_27B(mode, mux) J721E_IOPAD(0x1F4, mode, mux)       /* AB1 */
#define P9_28A(mode, mux) J721E_IOPAD(0x230, mode, mux)       /* U2 */
#define P9_28B(mode, mux) J721E_IOPAD(0x0B0, mode, mux)       /* AF28 */
#define P9_29A(mode, mux) J721E_IOPAD(0x23C, mode, mux)       /* V5 */
#define P9_29B(mode, mux) J721E_IOPAD(0x0D8, mode, mux)       /* AB25 */
#define P9_30A(mode, mux) J721E_IOPAD(0x238, mode, mux)       /* V6 */
#define P9_30B(mode, mux) J721E_IOPAD(0x0B4, mode, mux)       /* AE28 */
#define P9_31A(mode, mux) J721E_IOPAD(0x234, mode, mux)       /* U3 */
#define P9_31B(mode, mux) J721E_IOPAD(0x0D4, mode, mux)       /* AB26 */
#define P9_33A(mode, mux) J721E_WKUP_IOPAD(0x140, mode, mux)  /* K24 */
#define P9_33B(mode, mux) J721E_IOPAD(0x0CC, mode, mux)       /* AC28 */
#define P9_35A(mode, mux) J721E_WKUP_IOPAD(0x148, mode, mux)  /* K29 */
#define P9_35B(mode, mux) J721E_IOPAD(0x0E0, mode, mux)       /* AH27 */
#define P9_36A(mode, mux) J721E_WKUP_IOPAD(0x144, mode, mux)  /* K27 */
#define P9_36B(mode, mux) J721E_IOPAD(0x0E4, mode, mux)       /* AH29 */
#define P9_37A(mode, mux) J721E_WKUP_IOPAD(0x138, mode, mux)  /* K28 */
#define P9_37B(mode, mux) J721E_IOPAD(0x0E8, mode, mux)       /* AG28 */
#define P9_38A(mode, mux) J721E_WKUP_IOPAD(0x13C, mode, mux)  /* L28 */
#define P9_38B(mode, mux) J721E_IOPAD(0x0EC, mode, mux)       /* AG27 */
#define P9_39A(mode, mux) J721E_WKUP_IOPAD(0x130, mode, mux)  /* K25 */
#define P9_39B(mode, mux) J721E_IOPAD(0x0DC, mode, mux)       /* AJ28 */
#define P9_40A(mode, mux) J721E_WKUP_IOPAD(0x134, mode, mux)  /* K26 */
#define P9_40B(mode, mux) J721E_IOPAD(0x148, mode, mux)       /* AA26 */
#define P9_41(mode, mux)  J721E_IOPAD(0x204, mode, mux)       /* AD5 */
#define P9_42A(mode, mux) J721E_IOPAD(0x1F0, mode, mux)       /* AC2 */
#define P9_42B(mode, mux) J721E_IOPAD(0x048, mode, mux)       /* AJ21 */

#endif
