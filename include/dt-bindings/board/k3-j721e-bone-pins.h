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

#define gpio_P8_03 &main_gpio0 20	/* AH21: PRG1_PRU0_GPO19 AH21_MCAN6_TX */
#define gpio_P8_04 &main_gpio0 48	/* AC29: PRG0_PRU0_GPO5 AC29_SYS_BOOTMODE2 */
#define gpio_P8_05 &main_gpio0 33	/* AH25: PRG1_PRU1_GPO12 AH25_MCAN7_TX */
#define gpio_P8_06 &main_gpio0 34	/* AG25: PRG1_PRU1_GPO13 AG25_MCAN7_RX */
#define gpio_P8_07 &main_gpio0 15	/* AD24: PRG1_PRU0_GPO14 AD24_MCAN5_RX */
#define gpio_P8_08 &main_gpio0 14	/* AG24: PRG1_PRU0_GPO13 AG24_MCAN5_TX */
#define gpio_P8_09 &main_gpio0 17	/* AE24: PRG1_PRU0_GPO16 AE24_MCAN6_RX */
#define gpio_P8_10 &main_gpio0 16	/* AC24: PRG1_PRU0_GPO15 AC24_MCAN6_TX */
#define gpio_P8_11 &main_gpio0 60	/* AB24: PRG0_PRU0_GPO17 AB24_SYS_BOOTMODE7 */
#define gpio_P8_12 &main_gpio0 59	/* AH28: PRG0_PRU0_GPO16 AH28_PRG0_PWM0_A2 */
#define gpio_P8_13 &main_gpio0 89	/* V27: RGMII5_TD1 V27_EHRPWM0_B */
#define gpio_P8_14 &main_gpio0 75	/* AF27: PRG0_PRU1_GPO12 AF27_PRG0_PWM1_A0 */
#define gpio_P8_15 &main_gpio0 61	/* AB29: PRG0_PRU0_GPO18 AB29_PRG0_ECAP0_IN_APWM_OUT */
#define gpio_P8_16 &main_gpio0 62	/* AB28: PRG0_PRU0_GPO19 AB28_PRG0_PWM0_TZ_OUT */
#define gpio_P8_17 &main_gpio0 3	/* AF22: PRG1_PRU0_GPO2 AF22_PRG1_PWM2_A0 */
#define gpio_P8_18 &main_gpio0 4	/* AJ23: PRG1_PRU0_GPO3 AJ23_PRG1_PWM3_A2 */
#define gpio_P8_19 &main_gpio0 88	/* V29: RGMII5_TD2 V29_EHRPWM0_A */
#define gpio_P8_20 &main_gpio0 76	/* AF26: PRG0_PRU1_GPO13 AF26_PRG0_PWM1_B0 */
#define gpio_P8_21 &main_gpio0 30	/* AF21: PRG1_PRU1_GPO9 AF21_MCAN8_TX */
#define gpio_P8_22 &main_gpio0 5	/* AH23: PRG1_PRU0_GPO4 AH23_UART2_RXD */
#define gpio_P8_23 &main_gpio0 31	/* AB23: PRG1_PRU1_GPO10 AB23_MCAN8_RX */
#define gpio_P8_24 &main_gpio0 6	/* AD20: PRG1_PRU0_GPO5 AD20_SYS_BOOTMODE0 */
#define gpio_P8_25 &main_gpio0 35	/* AH26: PRG1_PRU1_GPO14 AH26_PRG1_PRU1_GPO14 */
#define gpio_P8_26 &main_gpio0 51	/* AC27: PRG0_PRU0_GPO8 AC27_PRG0_PWM2_A1 */
#define gpio_P8_27 &main_gpio0 71	/* AA28: PRG0_PRU1_GPO8 AA28_PRG0_PRU1_GPO8 */
#define gpio_P8_28 &main_gpio0 72	/* Y24: PRG0_PRU1_GPO9 Y24_PRG0_UART0_RXD */
#define gpio_P8_29 &main_gpio0 73	/* AA25: PRG0_PRU1_GPO10 AA25_PRG0_UART0_TXD */
#define gpio_P8_30 &main_gpio0 74	/* AG26: PRG0_PRU1_GPO11 AG26_PRG0_PRU1_GPO11 */
#define gpio_P8_31 gpio_P8_31A
#define gpio_P8_31A &main_gpio0 32	/* AJ25: PRG1_PRU1_GPO11 AJ25_AE29 */
#define gpio_P8_31B &main_gpio0 63	/* AE29: PRG0_PRU1_GPO0 AJ25_AE29 */
#define gpio_P8_32 gpio_P8_32A
#define gpio_P8_32A &main_gpio0 26	/* AG21: PRG1_PRU1_GPO5 AG21_AD28 */
#define gpio_P8_32B &main_gpio0 64	/* AD28: PRG0_PRU1_GPO1 AG21_AD28 */
#define gpio_P8_33 gpio_P8_33A
#define gpio_P8_33A &main_gpio0 25	/* AH24: PRG1_PRU1_GPO4 AH24_AA2 */
#define gpio_P8_33B &main_gpio0 111	/* AA2: SPI0_CS0 AH24_AA2 */
#define gpio_P8_34 &main_gpio0 7	/* AD22: PRG1_PRU0_GPO6 AD22_UART2_TXD */
#define gpio_P8_35 gpio_P8_35A
#define gpio_P8_35A &main_gpio0 24	/* AD23: PRG1_PRU1_GPO3 AD23_Y3 */
#define gpio_P8_35B &main_gpio0 116	/* Y3: SPI1_CS0 AD23_Y3 */
#define gpio_P8_36 &main_gpio0 8	/* AE20: PRG1_PRU0_GPO7 AE20_MCAN4_TX */
#define gpio_P8_37 gpio_P8_37A
#define gpio_P8_37A &main_gpio0 106	/* Y27: RGMII6_RD2 Y27_AD21 */
#define gpio_P8_37B &main_gpio0 11	/* AD21: PRG1_PRU0_GPO10 Y27_AD21 */
#define gpio_P8_38 gpio_P8_38A
#define gpio_P8_38A &main_gpio0 9	/* AJ20: PRG1_PRU0_GPO8 Y29_AJ20 */
#define gpio_P8_38B &main_gpio0 105	/* Y29: RGMII6_RD3 Y29_AJ20 */
#define gpio_P8_39 &main_gpio0 69	/* AC26: PRG0_PRU1_GPO6 AC26_PRG0_PRU1_GPO6 */
#define gpio_P8_40 &main_gpio0 70	/* AA24: PRG0_PRU1_GPO7 AA24_PRG0_PRU1_GPO7 */
#define gpio_P8_41 &main_gpio0 67	/* AD29: PRG0_PRU1_GPO4 AD29_PRG0_PRU1_GPO4 */
#define gpio_P8_42 &main_gpio0 68	/* AB27: PRG0_PRU1_GPO5 AB27_SYS_BOOTMODE6 */
#define gpio_P8_43 &main_gpio0 65	/* AD27: PRG0_PRU1_GPO2 AD27_PRG0_PRU1_GPO2 */
#define gpio_P8_44 &main_gpio0 66	/* AC25: PRG0_PRU1_GPO3 AC25_PRG0_PRU1_GPO3 */
#define gpio_P8_45 &main_gpio0 79	/* AG29: PRG0_PRU1_GPO16 AG29_PRG0_PRU1_GPO16 */
#define gpio_P8_46 &main_gpio0 80	/* Y25: PRG0_PRU1_GPO17 Y25_SYS_BOOTMODE3 */
#define gpio_P9_11 &main_gpio0 1	/* AC23: PRG1_PRU0_GPO0 AC23_UART0_RXD */
#define gpio_P9_12 &main_gpio0 45	/* AE27: PRG0_PRU0_GPO2 AE27_MCASP0_ACLKR */
#define gpio_P9_13 &main_gpio0 2	/* AG22: PRG1_PRU0_GPO1 AG22_UART0_TXD */
#define gpio_P9_14 &main_gpio0 93	/* U27: RGMII5_RD3 U27_EHRPWM2_A */
#define gpio_P9_15 &main_gpio0 47	/* AD25: PRG0_PRU0_GPO4 AD25_PRG0_PRU0_GPO4 */
#define gpio_P9_16 &main_gpio0 94	/* U24: RGMII5_RD2 U24_EHRPWM2_B */
#define gpio_P9_17 gpio_P9_17A
#define gpio_P9_17A &main_gpio0 28	/* AC21: PRG1_PRU1_GPO7 AC21_AA3 */
#define gpio_P9_17B &main_gpio0 115	/* AA3: SPI0_D1 AC21_AA3 */
#define gpio_P9_18 gpio_P9_18A
#define gpio_P9_18A &main_gpio0 40	/* AH22: PRG1_PRU1_GPO19 AH22_Y2 */
#define gpio_P9_18B &main_gpio0 120	/* Y2: SPI1_D1 AH22_Y2 */
#define gpio_P9_19 gpio_P9_19A
#define gpio_P9_19A &main_gpio1 1	/* W5: MCAN0_RX W5_AF29 */
#define gpio_P9_19B &main_gpio0 78	/* AF29: PRG0_PRU1_GPO15 W5_AF29 */
#define gpio_P9_20 gpio_P9_20A
#define gpio_P9_20A &main_gpio1 2	/* W6: MCAN0_TX W6_AE25 */
#define gpio_P9_20B &main_gpio0 77	/* AE25: PRG0_PRU1_GPO14 W6_AE25 */
#define gpio_P9_21 gpio_P9_21A
#define gpio_P9_21A &main_gpio0 39	/* AJ22: PRG1_PRU1_GPO18 AJ22_U28 */
#define gpio_P9_21B &main_gpio0 90	/* U28: RGMII5_TD0 AJ22_U28 */
#define gpio_P9_22 gpio_P9_22A
#define gpio_P9_22A &main_gpio0 38	/* AC22: PRG1_PRU1_GPO17 AC22_U29 */
#define gpio_P9_22B &main_gpio0 91	/* U29: RGMII5_TXC AC22_U29 */
#define gpio_P9_23 &main_gpio0 10	/* AG20: PRG1_PRU0_GPO9 AG20_SPI6_CS1 */
#define gpio_P9_24 gpio_P9_24A
#define gpio_P9_24A &main_gpio0 119	/* Y5: SPI1_D0 Y5_AJ24 */
#define gpio_P9_24B &main_gpio0 13	/* AJ24: PRG1_PRU0_GPO12 Y5_AJ24 */
#define gpio_P9_25 gpio_P9_25A
#define gpio_P9_25A &main_gpio0 127	/* AC4: UART1_CTSn AC4_W26 */
#define gpio_P9_25B &main_gpio0 104	/* W26: RGMII6_RXC AC4_W26 */
#define gpio_P9_26 gpio_P9_26A
#define gpio_P9_26A &main_gpio0 118	/* Y1: SPI1_CLK Y1_AF24 */
#define gpio_P9_26B &main_gpio0 12	/* AF24: PRG1_PRU0_GPO11 Y1_AF24 */
#define gpio_P9_27 gpio_P9_27A
#define gpio_P9_27A &main_gpio0 46	/* AD26: PRG0_PRU0_GPO3 AD26_AB1 */
#define gpio_P9_27B &main_gpio0 124	/* AB1: UART0_RTSn AD26_AB1 */
#define gpio_P9_28 gpio_P9_28A
#define gpio_P9_28A &main_gpio1 11	/* U2: ECAP0_IN_APWM_OUT U2_AF28 */
#define gpio_P9_28B &main_gpio0 43	/* AF28: PRG0_PRU0_GPO0 U2_AF28 */
#define gpio_P9_29 gpio_P9_29A
#define gpio_P9_29A &main_gpio1 14	/* V5: TIMER_IO1 V5_AB25 */
#define gpio_P9_29B &main_gpio0 53	/* AB25: PRG0_PRU0_GPO10 V5_AB25 */
#define gpio_P9_30 gpio_P9_30A
#define gpio_P9_30A &main_gpio1 13	/* V6: TIMER_IO0 V6_AE28 */
#define gpio_P9_30B &main_gpio0 44	/* AE28: PRG0_PRU0_GPO1 V6_AE28 */
#define gpio_P9_31 gpio_P9_31A
#define gpio_P9_31A &main_gpio1 12	/* U3: EXT_REFCLK1 U3_AB26 */
#define gpio_P9_31B &main_gpio0 52	/* AB26: PRG0_PRU0_GPO9 U3_AB26 */
#define gpio_P9_33 &main_gpio0 50
#define gpio_P9_33B &main_gpio0 50	/* AC28: PRG0_PRU0_GPO7 K24_AC28 */
#define gpio_P9_35 &main_gpio0 55
#define gpio_P9_35B &main_gpio0 55	/* AH27: PRG0_PRU0_GPO12 K29_AH27 */
#define gpio_P9_36 &main_gpio0 56
#define gpio_P9_36B &main_gpio0 56	/* AH29: PRG0_PRU0_GPO13 K27_AH29 */
#define gpio_P9_37 &main_gpio0 57
#define gpio_P9_37B &main_gpio0 57	/* AG28: PRG0_PRU0_GPO14 K28_AG28 */
#define gpio_P9_38 &main_gpio0 58
#define gpio_P9_38B &main_gpio0 58	/* AG27: PRG0_PRU0_GPO15 L28_AG27 */
#define gpio_P9_39 &main_gpio0 54
#define gpio_P9_39B &main_gpio0 54	/* AJ28: PRG0_PRU0_GPO11 K25_AJ28 */
#define gpio_P9_40 &main_gpio0 81
#define gpio_P9_40B &main_gpio0 81	/* AA26: PRG0_PRU1_GPO18 K26_AA26 */
#define gpio_P9_41 &main_gpio1 0	/* AD5: UART1_RTSn AD5_EQEP0_I */
#define gpio_P9_42 gpio_P9_42A
#define gpio_P9_42A &main_gpio0 123	/* AC2: UART0_CTSn AC2_AJ21 */
#define gpio_P9_42B &main_gpio0 18	/* AJ21: PRG1_PRU0_GPO17 AC2_AJ21 */

#define P8_03(mode, mux) J721E_IOPAD(0x54, mode, mux)	/* AH21: PRG1_PRU0_GPO19 AH21_MCAN6_TX */
#define P8_04(mode, mux) J721E_IOPAD(0xC4, mode, mux)	/* AC29: PRG0_PRU0_GPO5 AC29_SYS_BOOTMODE2 */
#define P8_05(mode, mux) J721E_IOPAD(0x88, mode, mux)	/* AH25: PRG1_PRU1_GPO12 AH25_MCAN7_TX */
#define P8_06(mode, mux) J721E_IOPAD(0x8C, mode, mux)	/* AG25: PRG1_PRU1_GPO13 AG25_MCAN7_RX */
#define P8_07(mode, mux) J721E_IOPAD(0x3C, mode, mux)	/* AD24: PRG1_PRU0_GPO14 AD24_MCAN5_RX */
#define P8_08(mode, mux) J721E_IOPAD(0x38, mode, mux)	/* AG24: PRG1_PRU0_GPO13 AG24_MCAN5_TX */
#define P8_09(mode, mux) J721E_IOPAD(0x44, mode, mux)	/* AE24: PRG1_PRU0_GPO16 AE24_MCAN6_RX */
#define P8_10(mode, mux) J721E_IOPAD(0x40, mode, mux)	/* AC24: PRG1_PRU0_GPO15 AC24_MCAN6_TX */
#define P8_11(mode, mux) J721E_IOPAD(0xF4, mode, mux)	/* AB24: PRG0_PRU0_GPO17 AB24_SYS_BOOTMODE7 */
#define P8_12(mode, mux) J721E_IOPAD(0xF0, mode, mux)	/* AH28: PRG0_PRU0_GPO16 AH28_PRG0_PWM0_A2 */
#define P8_13(mode, mux) J721E_IOPAD(0x168, mode, mux)	/* V27: RGMII5_TD1 V27_EHRPWM0_B */
#define P8_14(mode, mux) J721E_IOPAD(0x130, mode, mux)	/* AF27: PRG0_PRU1_GPO12 AF27_PRG0_PWM1_A0 */
#define P8_15(mode, mux) J721E_IOPAD(0xF8, mode, mux)	/* AB29: PRG0_PRU0_GPO18 AB29_PRG0_ECAP0_IN_APWM_OUT */
#define P8_16(mode, mux) J721E_IOPAD(0xFC, mode, mux)	/* AB28: PRG0_PRU0_GPO19 AB28_PRG0_PWM0_TZ_OUT */
#define P8_17(mode, mux) J721E_IOPAD(0xC, mode, mux)	/* AF22: PRG1_PRU0_GPO2 AF22_PRG1_PWM2_A0 */
#define P8_18(mode, mux) J721E_IOPAD(0x10, mode, mux)	/* AJ23: PRG1_PRU0_GPO3 AJ23_PRG1_PWM3_A2 */
#define P8_19(mode, mux) J721E_IOPAD(0x164, mode, mux)	/* V29: RGMII5_TD2 V29_EHRPWM0_A */
#define P8_20(mode, mux) J721E_IOPAD(0x134, mode, mux)	/* AF26: PRG0_PRU1_GPO13 AF26_PRG0_PWM1_B0 */
#define P8_21(mode, mux) J721E_IOPAD(0x7C, mode, mux)	/* AF21: PRG1_PRU1_GPO9 AF21_MCAN8_TX */
#define P8_22(mode, mux) J721E_IOPAD(0x14, mode, mux)	/* AH23: PRG1_PRU0_GPO4 AH23_UART2_RXD */
#define P8_23(mode, mux) J721E_IOPAD(0x80, mode, mux)	/* AB23: PRG1_PRU1_GPO10 AB23_MCAN8_RX */
#define P8_24(mode, mux) J721E_IOPAD(0x18, mode, mux)	/* AD20: PRG1_PRU0_GPO5 AD20_SYS_BOOTMODE0 */
#define P8_25(mode, mux) J721E_IOPAD(0x90, mode, mux)	/* AH26: PRG1_PRU1_GPO14 AH26_PRG1_PRU1_GPO14 */
#define P8_26(mode, mux) J721E_IOPAD(0xD0, mode, mux)	/* AC27: PRG0_PRU0_GPO8 AC27_PRG0_PWM2_A1 */
#define P8_27(mode, mux) J721E_IOPAD(0x120, mode, mux)	/* AA28: PRG0_PRU1_GPO8 AA28_PRG0_PRU1_GPO8 */
#define P8_28(mode, mux) J721E_IOPAD(0x124, mode, mux)	/* Y24: PRG0_PRU1_GPO9 Y24_PRG0_UART0_RXD */
#define P8_29(mode, mux) J721E_IOPAD(0x128, mode, mux)	/* AA25: PRG0_PRU1_GPO10 AA25_PRG0_UART0_TXD */
#define P8_30(mode, mux) J721E_IOPAD(0x12C, mode, mux)	/* AG26: PRG0_PRU1_GPO11 AG26_PRG0_PRU1_GPO11 */
#define P8_31A(mode, mux) J721E_IOPAD(0x84, mode, mux)	/* AJ25: PRG1_PRU1_GPO11 AJ25_AE29 */
#define P8_31B(mode, mux) J721E_IOPAD(0x100, mode, mux)	/* AE29: PRG0_PRU1_GPO0 AJ25_AE29 */
#define P8_32A(mode, mux) J721E_IOPAD(0x6C, mode, mux)	/* AG21: PRG1_PRU1_GPO5 AG21_AD28 */
#define P8_32B(mode, mux) J721E_IOPAD(0x104, mode, mux)	/* AD28: PRG0_PRU1_GPO1 AG21_AD28 */
#define P8_33A(mode, mux) J721E_IOPAD(0x68, mode, mux)	/* AH24: PRG1_PRU1_GPO4 AH24_AA2 */
#define P8_33B(mode, mux) J721E_IOPAD(0x1C0, mode, mux)	/* AA2: SPI0_CS0 AH24_AA2 */
#define P8_34(mode, mux) J721E_IOPAD(0x1C, mode, mux)	/* AD22: PRG1_PRU0_GPO6 AD22_UART2_TXD */
#define P8_35A(mode, mux) J721E_IOPAD(0x64, mode, mux)	/* AD23: PRG1_PRU1_GPO3 AD23_Y3 */
#define P8_35B(mode, mux) J721E_IOPAD(0x1D4, mode, mux)	/* Y3: SPI1_CS0 AD23_Y3 */
#define P8_36(mode, mux) J721E_IOPAD(0x20, mode, mux)	/* AE20: PRG1_PRU0_GPO7 AE20_MCAN4_TX */
#define P8_37A(mode, mux) J721E_IOPAD(0x1AC, mode, mux)	/* Y27: RGMII6_RD2 Y27_AD21 */
#define P8_37B(mode, mux) J721E_IOPAD(0x2C, mode, mux)	/* AD21: PRG1_PRU0_GPO10 Y27_AD21 */
#define P8_38A(mode, mux) J721E_IOPAD(0x24, mode, mux)	/* AJ20: PRG1_PRU0_GPO8 Y29_AJ20 */
#define P8_38B(mode, mux) J721E_IOPAD(0x1A8, mode, mux)	/* Y29: RGMII6_RD3 Y29_AJ20 */
#define P8_39(mode, mux) J721E_IOPAD(0x118, mode, mux)	/* AC26: PRG0_PRU1_GPO6 AC26_PRG0_PRU1_GPO6 */
#define P8_40(mode, mux) J721E_IOPAD(0x11C, mode, mux)	/* AA24: PRG0_PRU1_GPO7 AA24_PRG0_PRU1_GPO7 */
#define P8_41(mode, mux) J721E_IOPAD(0x110, mode, mux)	/* AD29: PRG0_PRU1_GPO4 AD29_PRG0_PRU1_GPO4 */
#define P8_42(mode, mux) J721E_IOPAD(0x114, mode, mux)	/* AB27: PRG0_PRU1_GPO5 AB27_SYS_BOOTMODE6 */
#define P8_43(mode, mux) J721E_IOPAD(0x108, mode, mux)	/* AD27: PRG0_PRU1_GPO2 AD27_PRG0_PRU1_GPO2 */
#define P8_44(mode, mux) J721E_IOPAD(0x10C, mode, mux)	/* AC25: PRG0_PRU1_GPO3 AC25_PRG0_PRU1_GPO3 */
#define P8_45(mode, mux) J721E_IOPAD(0x140, mode, mux)	/* AG29: PRG0_PRU1_GPO16 AG29_PRG0_PRU1_GPO16 */
#define P8_46(mode, mux) J721E_IOPAD(0x144, mode, mux)	/* Y25: PRG0_PRU1_GPO17 Y25_SYS_BOOTMODE3 */
#define P9_11(mode, mux) J721E_IOPAD(0x4, mode, mux)	/* AC23: PRG1_PRU0_GPO0 AC23_UART0_RXD */
#define P9_12(mode, mux) J721E_IOPAD(0xB8, mode, mux)	/* AE27: PRG0_PRU0_GPO2 AE27_MCASP0_ACLKR */
#define P9_13(mode, mux) J721E_IOPAD(0x8, mode, mux)	/* AG22: PRG1_PRU0_GPO1 AG22_UART0_TXD */
#define P9_14(mode, mux) J721E_IOPAD(0x178, mode, mux)	/* U27: RGMII5_RD3 U27_EHRPWM2_A */
#define P9_15(mode, mux) J721E_IOPAD(0xC0, mode, mux)	/* AD25: PRG0_PRU0_GPO4 AD25_PRG0_PRU0_GPO4 */
#define P9_16(mode, mux) J721E_IOPAD(0x17C, mode, mux)	/* U24: RGMII5_RD2 U24_EHRPWM2_B */
#define P9_17A(mode, mux) J721E_IOPAD(0x74, mode, mux)	/* AC21: PRG1_PRU1_GPO7 AC21_AA3 */
#define P9_17B(mode, mux) J721E_IOPAD(0x1D0, mode, mux)	/* AA3: SPI0_D1 AC21_AA3 */
#define P9_18A(mode, mux) J721E_IOPAD(0xA4, mode, mux)	/* AH22: PRG1_PRU1_GPO19 AH22_Y2 */
#define P9_18B(mode, mux) J721E_IOPAD(0x1E4, mode, mux)	/* Y2: SPI1_D1 AH22_Y2 */
#define P9_19A(mode, mux) J721E_IOPAD(0x208, mode, mux)	/* W5: MCAN0_RX W5_AF29 */
#define P9_19B(mode, mux) J721E_IOPAD(0x13C, mode, mux)	/* AF29: PRG0_PRU1_GPO15 W5_AF29 */
#define P9_20A(mode, mux) J721E_IOPAD(0x20C, mode, mux)	/* W6: MCAN0_TX W6_AE25 */
#define P9_20B(mode, mux) J721E_IOPAD(0x138, mode, mux)	/* AE25: PRG0_PRU1_GPO14 W6_AE25 */
#define P9_21A(mode, mux) J721E_IOPAD(0xA0, mode, mux)	/* AJ22: PRG1_PRU1_GPO18 AJ22_U28 */
#define P9_21B(mode, mux) J721E_IOPAD(0x16C, mode, mux)	/* U28: RGMII5_TD0 AJ22_U28 */
#define P9_22A(mode, mux) J721E_IOPAD(0x9C, mode, mux)	/* AC22: PRG1_PRU1_GPO17 AC22_U29 */
#define P9_22B(mode, mux) J721E_IOPAD(0x170, mode, mux)	/* U29: RGMII5_TXC AC22_U29 */
#define P9_23(mode, mux) J721E_IOPAD(0x28, mode, mux)	/* AG20: PRG1_PRU0_GPO9 AG20_SPI6_CS1 */
#define P9_24A(mode, mux) J721E_IOPAD(0x1E0, mode, mux)	/* Y5: SPI1_D0 Y5_AJ24 */
#define P9_24B(mode, mux) J721E_IOPAD(0x34, mode, mux)	/* AJ24: PRG1_PRU0_GPO12 Y5_AJ24 */
#define P9_25A(mode, mux) J721E_IOPAD(0x200, mode, mux)	/* AC4: UART1_CTSn AC4_W26 */
#define P9_25B(mode, mux) J721E_IOPAD(0x1A4, mode, mux)	/* W26: RGMII6_RXC AC4_W26 */
#define P9_26A(mode, mux) J721E_IOPAD(0x1DC, mode, mux)	/* Y1: SPI1_CLK Y1_AF24 */
#define P9_26B(mode, mux) J721E_IOPAD(0x30, mode, mux)	/* AF24: PRG1_PRU0_GPO11 Y1_AF24 */
#define P9_27A(mode, mux) J721E_IOPAD(0xBC, mode, mux)	/* AD26: PRG0_PRU0_GPO3 AD26_AB1 */
#define P9_27B(mode, mux) J721E_IOPAD(0x1F4, mode, mux)	/* AB1: UART0_RTSn AD26_AB1 */
#define P9_28A(mode, mux) J721E_IOPAD(0x230, mode, mux)	/* U2: ECAP0_IN_APWM_OUT U2_AF28 */
#define P9_28B(mode, mux) J721E_IOPAD(0xB0, mode, mux)	/* AF28: PRG0_PRU0_GPO0 U2_AF28 */
#define P9_29A(mode, mux) J721E_IOPAD(0x23C, mode, mux)	/* V5: TIMER_IO1 V5_AB25 */
#define P9_29B(mode, mux) J721E_IOPAD(0xD8, mode, mux)	/* AB25: PRG0_PRU0_GPO10 V5_AB25 */
#define P9_30A(mode, mux) J721E_IOPAD(0x238, mode, mux)	/* V6: TIMER_IO0 V6_AE28 */
#define P9_30B(mode, mux) J721E_IOPAD(0xB4, mode, mux)	/* AE28: PRG0_PRU0_GPO1 V6_AE28 */
#define P9_31A(mode, mux) J721E_IOPAD(0x234, mode, mux)	/* U3: EXT_REFCLK1 U3_AB26 */
#define P9_31B(mode, mux) J721E_IOPAD(0xD4, mode, mux)	/* AB26: PRG0_PRU0_GPO9 U3_AB26 */
#define P9_33A(mode, mux) J721E_WKUP_IOPAD(0x140, mode, mux)	/* K24: MCU_ADC0_AIN4 K24_AC28 */
#define P9_33B(mode, mux) J721E_IOPAD(0xCC, mode, mux)	/* AC28: PRG0_PRU0_GPO7 K24_AC28 */
#define P9_35A(mode, mux) J721E_WKUP_IOPAD(0x148, mode, mux)	/* K29: MCU_ADC0_AIN6 K29_AH27 */
#define P9_35B(mode, mux) J721E_IOPAD(0xE0, mode, mux)	/* AH27: PRG0_PRU0_GPO12 K29_AH27 */
#define P9_36A(mode, mux) J721E_WKUP_IOPAD(0x144, mode, mux)	/* K27: MCU_ADC0_AIN5 K27_AH29 */
#define P9_36B(mode, mux) J721E_IOPAD(0xE4, mode, mux)	/* AH29: PRG0_PRU0_GPO13 K27_AH29 */
#define P9_37A(mode, mux) J721E_WKUP_IOPAD(0x138, mode, mux)	/* K28: MCU_ADC0_AIN2 K28_AG28 */
#define P9_37B(mode, mux) J721E_IOPAD(0xE8, mode, mux)	/* AG28: PRG0_PRU0_GPO14 K28_AG28 */
#define P9_38A(mode, mux) J721E_WKUP_IOPAD(0x13C, mode, mux)	/* L28: MCU_ADC0_AIN3 L28_AG27 */
#define P9_38B(mode, mux) J721E_IOPAD(0xEC, mode, mux)	/* AG27: PRG0_PRU0_GPO15 L28_AG27 */
#define P9_39A(mode, mux) J721E_WKUP_IOPAD(0x130, mode, mux)	/* K25: MCU_ADC0_AIN0 K25_AJ28 */
#define P9_39B(mode, mux) J721E_IOPAD(0xDC, mode, mux)	/* AJ28: PRG0_PRU0_GPO11 K25_AJ28 */
#define P9_40A(mode, mux) J721E_WKUP_IOPAD(0x134, mode, mux)	/* K26: MCU_ADC0_AIN1 K26_AA26 */
#define P9_40B(mode, mux) J721E_IOPAD(0x148, mode, mux)	/* AA26: PRG0_PRU1_GPO18 K26_AA26 */
#define P9_41(mode, mux) J721E_IOPAD(0x204, mode, mux)	/* AD5: UART1_RTSn AD5_EQEP0_I */
#define P9_42A(mode, mux) J721E_IOPAD(0x1F0, mode, mux)	/* AC2: UART0_CTSn AC2_AJ21 */
#define P9_42B(mode, mux) J721E_IOPAD(0x4C, mode, mux)	/* AJ21: PRG1_PRU0_GPO17 AC2_AJ21 */

#endif
