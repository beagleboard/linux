// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Deepak Khatri <deepaklorkhatri7@gmail.com>
 * See Cape Interface Spec page for more info on Bone Buses
 * https://elinux.org/Beagleboard:BeagleBone_cape_interface_spec
 */

#ifndef _DT_BINDINGS_BOARD_AM335X_BONE_PINS_H
#define _DT_BINDINGS_BOARD_AM335X_BONE_PINS_H

#define bb_device 0
#define board_soc AM335X

#define gpio_P8_03 &gpio1 6
#define gpio_P8_04 &gpio1 7
#define gpio_P8_05 &gpio1 2
#define gpio_P8_06 &gpio1 3
#define gpio_P8_07 &gpio2 2
#define gpio_P8_08 &gpio2 3
#define gpio_P8_09 &gpio2 5
#define gpio_P8_10 &gpio2 4
#define gpio_P8_11 &gpio1 13
#define gpio_P8_12 &gpio1 12
#define gpio_P8_13 &gpio0 23
#define gpio_P8_14 &gpio0 26
#define gpio_P8_15 &gpio1 15
#define gpio_P8_16 &gpio1 14
#define gpio_P8_17 &gpio0 27
#define gpio_P8_18 &gpio2 1
#define gpio_P8_19 &gpio0 22
#define gpio_P8_20 &gpio1 31
#define gpio_P8_21 &gpio1 30
#define gpio_P8_22 &gpio1 5
#define gpio_P8_23 &gpio1 4
#define gpio_P8_24 &gpio1 1
#define gpio_P8_25 &gpio1 0
#define gpio_P8_26 &gpio1 29
#define gpio_P8_27 &gpio2 22
#define gpio_P8_28 &gpio2 24
#define gpio_P8_29 &gpio2 23
#define gpio_P8_30 &gpio2 25
#define gpio_P8_31 &gpio0 10
#define gpio_P8_32 &gpio0 11
#define gpio_P8_33 &gpio0 9
#define gpio_P8_34 &gpio2 17
#define gpio_P8_35 &gpio0 8
#define gpio_P8_36 &gpio2 16
#define gpio_P8_37 &gpio2 14
#define gpio_P8_38 &gpio2 15
#define gpio_P8_39 &gpio2 12
#define gpio_P8_40 &gpio2 13
#define gpio_P8_41 &gpio2 10
#define gpio_P8_42 &gpio2 11
#define gpio_P8_43 &gpio2 8
#define gpio_P8_44 &gpio2 9
#define gpio_P8_45 &gpio2 6
#define gpio_P8_46 &gpio2 7
#define gpio_P9_11 &gpio0 30
#define gpio_P9_12 &gpio1 28
#define gpio_P9_13 &gpio0 31
#define gpio_P9_14 &gpio1 18
#define gpio_P9_15 &gpio1 16
#define gpio_P9_16 &gpio1 19
#define gpio_P9_17 &gpio0 5
#define gpio_P9_18 &gpio0 4
#define gpio_P9_19 &gpio0 13
#define gpio_P9_20 &gpio0 12
#define gpio_P9_21 &gpio0 3
#define gpio_P9_22 &gpio0 2
#define gpio_P9_23 &gpio1 17
#define gpio_P9_24 &gpio0 15
#define gpio_P9_25 &gpio3 21
#define gpio_P9_26 &gpio0 14
#define gpio_P9_27 &gpio3 19
#define gpio_P9_28 &gpio3 17
#define gpio_P9_29 &gpio3 15
#define gpio_P9_30 &gpio3 16
#define gpio_P9_31 &gpio3 14
#define gpio_P9_41 &gpio0 20
#define gpio_P9_41A &gpio0 20
#define gpio_P9_41B &gpio3 20
#define gpio_P9_91 &gpio3 20
#define gpio_P9_42 &gpio0 7
#define gpio_P9_42A &gpio0 7
#define gpio_P9_42B &gpio3 18
#define gpio_P9_92 &gpio3 18
#define gpio_A15 &gpio0 19

#define P8_03(mode) AM33XX_IOPAD(0x0818, mode)  /* R9: gpmc_ad6 */
#define P8_04(mode) AM33XX_IOPAD(0x081c, mode)  /* T9: gpmc_ad7 */
#define P8_05(mode) AM33XX_IOPAD(0x0808, mode)  /* R8: gpmc_ad2 */
#define P8_06(mode) AM33XX_IOPAD(0x080c, mode)  /* T8: gpmc_ad3 */
#define P8_07(mode) AM33XX_IOPAD(0x0890, mode)  /* R7: gpmc_advn_ale */
#define P8_08(mode) AM33XX_IOPAD(0x0894, mode)  /* T7: gpmc_oen_ren */
#define P8_09(mode) AM33XX_IOPAD(0x089c, mode)  /* T6: gpmc_be0n_cle */
#define P8_10(mode) AM33XX_IOPAD(0x0898, mode)  /* U6: gpmc_wen */
#define P8_11(mode) AM33XX_IOPAD(0x0834, mode)  /* R12: gpmc_ad13 */
#define P8_12(mode) AM33XX_IOPAD(0x0830, mode)  /* T12: gpmc_ad12 */
#define P8_13(mode) AM33XX_IOPAD(0x0824, mode)  /* T10: gpmc_ad9 */
#define P8_14(mode) AM33XX_IOPAD(0x0828, mode)  /* T11: gpmc_ad10 */
#define P8_15(mode) AM33XX_IOPAD(0x083c, mode)  /* U13: gpmc_ad15 */
#define P8_16(mode) AM33XX_IOPAD(0x0838, mode)  /* V13: gpmc_ad14 */
#define P8_17(mode) AM33XX_IOPAD(0x082c, mode)  /* U12: gpmc_ad11 */
#define P8_18(mode) AM33XX_IOPAD(0x088c, mode)  /* V12: gpmc_clk */
#define P8_19(mode) AM33XX_IOPAD(0x0820, mode)  /* U10: gpmc_ad8 */
#define P8_20(mode) AM33XX_IOPAD(0x0884, mode)  /* V9: gpmc_csn2 */
#define P8_21(mode) AM33XX_IOPAD(0x0880, mode)  /* U9: gpmc_csn1 */
#define P8_22(mode) AM33XX_IOPAD(0x0814, mode)  /* V8: gpmc_ad5 */
#define P8_23(mode) AM33XX_IOPAD(0x0810, mode)  /* U8: gpmc_ad4 */
#define P8_24(mode) AM33XX_IOPAD(0x0804, mode)  /* V7: gpmc_ad1 */
#define P8_25(mode) AM33XX_IOPAD(0x0800, mode)  /* U7: gpmc_ad0 */
#define P8_26(mode) AM33XX_IOPAD(0x087c, mode)  /* V6: gpmc_csn0 */
#define P8_27(mode) AM33XX_IOPAD(0x08e0, mode)  /* U5: lcd_vsync */
#define P8_28(mode) AM33XX_IOPAD(0x08e8, mode)  /* V5: lcd_pclk */
#define P8_29(mode) AM33XX_IOPAD(0x08e4, mode)  /* R5: lcd_hsync */
#define P8_30(mode) AM33XX_IOPAD(0x08ec, mode)  /* R6: lcd_ac_bias_en */
#define P8_31(mode) AM33XX_IOPAD(0x08d8, mode)  /* V4: lcd_data14 */
#define P8_32(mode) AM33XX_IOPAD(0x08dc, mode)  /* T5: lcd_data15 */
#define P8_33(mode) AM33XX_IOPAD(0x08d4, mode)  /* V3: lcd_data13 */
#define P8_34(mode) AM33XX_IOPAD(0x08cc, mode)  /* U4: lcd_data11 */
#define P8_35(mode) AM33XX_IOPAD(0x08d0, mode)  /* V2: lcd_data12 */
#define P8_36(mode) AM33XX_IOPAD(0x08c8, mode)  /* U3: lcd_data10 */
#define P8_37(mode) AM33XX_IOPAD(0x08c0, mode)  /* U1: lcd_data8 */
#define P8_38(mode) AM33XX_IOPAD(0x08c4, mode)  /* U2: lcd_data9 */
#define P8_39(mode) AM33XX_IOPAD(0x08b8, mode)  /* T3: lcd_data6 */
#define P8_40(mode) AM33XX_IOPAD(0x08bc, mode)  /* T4: lcd_data7 */
#define P8_41(mode) AM33XX_IOPAD(0x08b0, mode)  /* T1: lcd_data4 */
#define P8_42(mode) AM33XX_IOPAD(0x08b4, mode)  /* T2: lcd_data5 */
#define P8_43(mode) AM33XX_IOPAD(0x08a8, mode)  /* R3: lcd_data2 */
#define P8_44(mode) AM33XX_IOPAD(0x08ac, mode)  /* R4: lcd_data3 */
#define P8_45(mode) AM33XX_IOPAD(0x08a0, mode)  /* R1: lcd_data0 */
#define P8_46(mode) AM33XX_IOPAD(0x08a4, mode)  /* R2: lcd_data1 */
#define P9_11(mode) AM33XX_IOPAD(0x0870, mode)  /* T17: gpmc_wait0 */
#define P9_12(mode) AM33XX_IOPAD(0x0878, mode)  /* U18: gpmc_be1n */
#define P9_13(mode) AM33XX_IOPAD(0x0874, mode)  /* U17: gpmc_wpn */
#define P9_14(mode) AM33XX_IOPAD(0x0848, mode)  /* U14: gpmc_a2 */
#define P9_15(mode) AM33XX_IOPAD(0x0840, mode)  /* R13: gpmc_a0 */
#define P9_16(mode) AM33XX_IOPAD(0x084c, mode)  /* T14: gpmc_a3 */
#define P9_17(mode) AM33XX_IOPAD(0x095c, mode)  /* A16: spi0_cs0 */
#define P9_18(mode) AM33XX_IOPAD(0x0958, mode)  /* B16: spi0_d1 */
#define P9_19(mode) AM33XX_IOPAD(0x097c, mode)  /* D17: uart1_rtsn */
#define P9_20(mode) AM33XX_IOPAD(0x0978, mode)  /* D18: uart1_ctsn */
#define P9_21(mode) AM33XX_IOPAD(0x0954, mode)  /* B17: spi0_d0 */
#define P9_22(mode) AM33XX_IOPAD(0x0950, mode)  /* A17: spi0_sclk */
#define P9_23(mode) AM33XX_IOPAD(0x0844, mode)  /* V14: gpmc_a1 */
#define P9_24(mode) AM33XX_IOPAD(0x0984, mode)  /* D15: uart1_txd */
#define P9_25(mode) AM33XX_IOPAD(0x09ac, mode)  /* A14: mcasp0_ahclkx */
#define P9_26(mode) AM33XX_IOPAD(0x0980, mode)  /* D16: uart1_rxd */
#define P9_27(mode) AM33XX_IOPAD(0x09a4, mode)  /* C13: mcasp0_fsr */
#define P9_28(mode) AM33XX_IOPAD(0x099c, mode)  /* C12: mcasp0_ahclkr */
#define P9_29(mode) AM33XX_IOPAD(0x0994, mode)  /* B13: mcasp0_fsx */
#define P9_30(mode) AM33XX_IOPAD(0x0998, mode)  /* D12: mcasp0_axr0 */
#define P9_31(mode) AM33XX_IOPAD(0x0990, mode)  /* A13: mcasp0_aclkx */
#define P9_41(mode) AM33XX_IOPAD(0x09b4, mode)  /* D14: xdma_event_intr1 */
#define P9_41A(mode) AM33XX_IOPAD(0x09b4, mode) /* D14: xdma_event_intr1 */
#define P9_41B(mode) AM33XX_IOPAD(0x09a8, mode) /* D13: mcasp0_axr1 */
#define P9_91(mode) AM33XX_IOPAD(0x09a8, mode) /* D13: mcasp0_axr1 */
#define P9_42(mode) AM33XX_IOPAD(0x0964, mode) /* C18: P0_in_PWM0_out */
#define P9_42A(mode) AM33XX_IOPAD(0x0964, mode) /* C18: P0_in_PWM0_out */
#define P9_42B(mode) AM33XX_IOPAD(0x09a0, mode) /* B12: mcasp0_aclkr */
#define P9_92(mode) AM33XX_IOPAD(0x09a0, mode) /* B12: mcasp0_aclkr */

#define gpio_P1_02 &gpio2 23
#define gpio_P1_04 &gpio2 25
#define gpio_P1_06 &gpio0 5
#define gpio_P1_08 &gpio0 2
#define gpio_P1_10 &gpio0 3
#define gpio_P1_12 &gpio0 4
#define gpio_P1_20 &gpio0 20
#define gpio_P1_26 &gpio0 12
#define gpio_P1_28 &gpio0 13
#define gpio_P1_29 &gpio3 21
#define gpio_P1_30 &gpio1 11
#define gpio_P1_31 &gpio3 18
#define gpio_P1_32 &gpio1 10
#define gpio_P1_33 &gpio3 15
#define gpio_P1_34 &gpio0 26
#define gpio_P1_35 &gpio2 24
#define gpio_P1_36 &gpio3 14
#define gpio_P2_01 &gpio1 18
#define gpio_P2_02 &gpio1 27
#define gpio_P2_03 &gpio0 23
#define gpio_P2_04 &gpio1 26
#define gpio_P2_05 &gpio0 30
#define gpio_P2_06 &gpio1 25
#define gpio_P2_07 &gpio0 31
#define gpio_P2_08 &gpio1 28
#define gpio_P2_09 &gpio0 15
#define gpio_P2_10 &gpio1 20
#define gpio_P2_11 &gpio0 14
#define gpio_P2_17 &gpio2 1
#define gpio_P2_18 &gpio1 15
#define gpio_P2_19 &gpio0 27
#define gpio_P2_20 &gpio2 0
#define gpio_P2_22 &gpio1 14
#define gpio_P2_24 &gpio1 12
#define gpio_P2_25 &gpio1 9
#define gpio_P2_27 &gpio1 8
#define gpio_P2_28 &gpio3 20
#define gpio_P2_29 &gpio0 7
#define gpio_P2_30 &gpio3 17
#define gpio_P2_31 &gpio0 19
#define gpio_P2_32 &gpio3 16
#define gpio_P2_33 &gpio1 13
#define gpio_P2_34 &gpio3 19
#define gpio_P2_35 &gpio2 22

#define P1_02(mode) AM33XX_IOPAD(0x08e4, mode)  /* R5: lcd_hsync */
#define P1_04(mode) AM33XX_IOPAD(0x08ec, mode)  /* R6: lcd_ac_bias_en */
#define P1_06(mode) AM33XX_IOPAD(0x095c, mode)  /* A16: spi0_cs0 */
#define P1_08(mode) AM33XX_IOPAD(0x0950, mode)  /* A17: spi0_sclk */
#define P1_10(mode) AM33XX_IOPAD(0x0954, mode)  /* B17: spi0_d0 */
#define P1_12(mode) AM33XX_IOPAD(0x0958, mode)  /* B16: spi0_d1 */
#define P1_20(mode) AM33XX_IOPAD(0x09b4, mode)  /* D14: xdma_event_intr1 */
#define P1_26(mode) AM33XX_IOPAD(0x0978, mode)  /* D18: uart1_ctsn */
#define P1_28(mode) AM33XX_IOPAD(0x097c, mode)  /* D17: uart1_rtsn */
#define P1_29(mode) AM33XX_IOPAD(0x09ac, mode)  /* A14: mcasp0_ahclkx */
#define P1_30(mode) AM33XX_IOPAD(0x0974, mode)  /* E16: uart0_txd */
#define P1_31(mode) AM33XX_IOPAD(0x09a0, mode)  /* B12: mcasp0_aclkr */
#define P1_32(mode) AM33XX_IOPAD(0x0970, mode)  /* E15: uart0_rxd */
#define P1_33(mode) AM33XX_IOPAD(0x0994, mode)  /* B13: mcasp0_fsx */
#define P1_34(mode) AM33XX_IOPAD(0x0828, mode)  /* T11: gpmc_ad10 */
#define P1_35(mode) AM33XX_IOPAD(0x08e8, mode)  /* V5: lcd_pclk */
#define P1_36(mode) AM33XX_IOPAD(0x0990, mode)  /* A13: mcasp0_aclkx */
#define P2_01(mode) AM33XX_IOPAD(0x0848, mode)  /* U14: gpmc_a2 */
#define P2_02(mode) AM33XX_IOPAD(0x086c, mode)  /* V17: gpmc_a11 */
#define P2_03(mode) AM33XX_IOPAD(0x0824, mode)  /* T10: gpmc_ad9 */
#define P2_04(mode) AM33XX_IOPAD(0x0868, mode)  /* T16: gpmc_a10 */
#define P2_05(mode) AM33XX_IOPAD(0x0870, mode)  /* T17: gpmc_wait0 */
#define P2_06(mode) AM33XX_IOPAD(0x0864, mode)  /* U16: gpmc_a9 */
#define P2_07(mode) AM33XX_IOPAD(0x0874, mode)  /* U17: gpmc_wpn */
#define P2_08(mode) AM33XX_IOPAD(0x0878, mode)  /* U18: gpmc_be1n */
#define P2_09(mode) AM33XX_IOPAD(0x0984, mode)  /* D15: uart1_txd */
#define P2_10(mode) AM33XX_IOPAD(0x0850, mode)  /* R14: gpmc_a4 */
#define P2_11(mode) AM33XX_IOPAD(0x0980, mode)  /* D16: uart1_rxd */
#define P2_17(mode) AM33XX_IOPAD(0x088c, mode)  /* V12: gpmc_clk */
#define P2_18(mode) AM33XX_IOPAD(0x083c, mode)  /* U13: gpmc_ad15 */
#define P2_19(mode) AM33XX_IOPAD(0x082c, mode)  /* U12: gpmc_ad11 */
#define P2_20(mode) AM33XX_IOPAD(0x0888, mode)  /* T13: gpmc_csn3 */
#define P2_22(mode) AM33XX_IOPAD(0x0838, mode)  /* V13: gpmc_ad14 */
#define P2_24(mode) AM33XX_IOPAD(0x0830, mode)  /* T12: gpmc_ad12 */
#define P2_25(mode) AM33XX_IOPAD(0x096c, mode)  /* E17: uart0_rtsn */
#define P2_27(mode) AM33XX_IOPAD(0x0968, mode)  /* E18: uart0_ctsn */
#define P2_28(mode) AM33XX_IOPAD(0x09a8, mode)  /* D13: mcasp0_axr1 */
#define P2_29(mode) AM33XX_IOPAD(0x0964, mode)  /* C18: eCAP0_in_PWM0_out */
#define P2_30(mode) AM33XX_IOPAD(0x099c, mode)  /* C12: mcasp0_ahclkr */
#define P2_31(mode) AM33XX_IOPAD(0x09b0, mode)  /* A15: xdma_event_intr0 */
#define P2_32(mode) AM33XX_IOPAD(0x0998, mode)  /* D12: mcasp0_axr0 */
#define P2_33(mode) AM33XX_IOPAD(0x0834, mode)  /* R12: gpmc_ad13 */
#define P2_34(mode) AM33XX_IOPAD(0x09a4, mode)  /* C13: mcasp0_fsr */
#define P2_35(mode) AM33XX_IOPAD(0x08e0, mode)  /* U5: lcd_vsync */

#endif
