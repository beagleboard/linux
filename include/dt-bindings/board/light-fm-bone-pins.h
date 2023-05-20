// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 - 2023 BeagleBoard.org - https://beagleboard.org/
 * Copyright (C) 2020 - 2023 Deepak Khatri <lorforlinux@beagleboard.org>
 * Copyright (C) 2021 Jason Kridner <jkridner@beagleboard.org>
 * See Cape Interface Spec page for more info on Bone Buses
 * https://docs.beagleboard.org/0.0/boards/capes/cape-interface-spec.html
 */

#ifndef _DT_BINDINGS_BOARD_TH1520_BONE_PINS_H
#define _DT_BINDINGS_BOARD_TH1520_BONE_PINS_H

#define bb_device 1
#define board_soc TH1520

/*
* GPIO port macros
* 
* gpio0: &gpio0_porta
* gpio1: &gpio1_porta
* gpio2: &gpio2_porta
* gpio3: &gpio3_porta
* ao_gpio4: &ao_gpio4_porta
* ao_gpio: &ao_gpio_porta
*
*/

// P8 cape header
#define gpio_P8_03 &gpio1_porta 21
#define gpio_P8_04 &gpio1_porta 22
#define gpio_P8_05 &gpio1_porta 23
#define gpio_P8_06 &gpio1_porta 24
#define gpio_P8_07 &gpio1_porta 25
#define gpio_P8_08 &gpio1_porta 26
#define gpio_P8_09 &gpio1_porta 27
#define gpio_P8_10 &gpio1_porta 28
#define gpio_P8_11 &gpio1_porta 29
#define gpio_P8_12 &gpio1_porta 30
#define gpio_P8_13 &gpio3_porta 2
#define gpio_P8_14 &gpio1_porta 20
#define gpio_P8_15 &gpio3_porta 0
#define gpio_P8_16 &gpio0_porta 20
#define gpio_P8_17 &gpio3_porta 1
#define gpio_P8_18 &gpio1_porta 5
#define gpio_P8_19 &gpio3_porta 3
#define gpio_P8_20 &gpio1_porta 6
#define gpio_P8_21 &gpio1_porta 7
#define gpio_P8_22 &gpio1_porta 8
#define gpio_P8_23 &gpio1_porta 9
#define gpio_P8_24 &gpio1_porta 10
#define gpio_P8_25 &gpio1_porta 11
#define gpio_P8_26 &gpio1_porta 12
#define gpio_P8_27 &gpio1_porta 15
#define gpio_P8_28 &gpio1_porta 16
#define gpio_P8_29 &gpio1_porta 14
#define gpio_P8_30 &gpio1_porta 13
#define gpio_P8_31 &gpio1_porta 3
#define gpio_P8_32 &gpio1_porta 4
#define gpio_P8_33 &gpio1_porta 2
#define gpio_P8_34 &gpio1_porta 0
#define gpio_P8_35 &gpio1_porta 1
#define gpio_P8_36 &gpio0_porta 31
#define gpio_P8_37 &gpio0_porta 29
#define gpio_P8_38 &gpio0_porta 30
#define gpio_P8_39 &gpio0_porta 27
#define gpio_P8_40 &gpio0_porta 28
#define gpio_P8_41 &gpio0_porta 25
#define gpio_P8_42 &gpio0_porta 26
#define gpio_P8_43 &gpio0_porta 23
#define gpio_P8_44 &gpio0_porta 24
#define gpio_P8_45 &gpio0_porta 21
#define gpio_P8_46 &gpio0_porta 22

// P9 cape header
#define gpio_P9_11 &gpio0_porta 10
#define gpio_P9_12 &gpio2_porta 3
#define gpio_P9_13 &gpio0_porta 11
#define gpio_P9_14 &gpio2_porta 6
#define gpio_P9_15 &gpio2_porta 7
#define gpio_P9_16 &gpio2_porta 5
#define gpio_P9_17 &gpio0_porta 1
#define gpio_P9_18 &gpio0_porta 2
#define gpio_P9_19 &gpio2_porta 9
#define gpio_P9_20 &gpio2_porta 10
#define gpio_P9_21 &gpio0_porta 3
#define gpio_P9_22 &gpio0_porta 0
#define gpio_P9_23 &gpio2_porta 8
#define gpio_P9_24 &gpio0_porta 4
#define gpio_P9_25 &gpio2_porta 18
#define gpio_P9_26 &gpio0_porta 5
#define gpio_P9_27 &gpio2_porta 19
#define gpio_P9_28 &gpio2_porta 15
#define gpio_P9_29 &gpio2_porta 17
#define gpio_P9_30 &gpio2_porta 16
#define gpio_P9_31 &gpio2_porta 14
#define gpio_P9_41 &gpio2_porta 13
#define gpio_P9_42 &gpio2_porta 2

// mikroBus port
#define gpio_mb_pwm  &gpio2_porta 4
#define gpio_mb_rst  &ao_gpio4_porta 3
#define gpio_mb_int  &gpio2_porta 21
#define gpio_mb_rxd  &gpio0_porta 17
#define gpio_mb_txd  &gpio0_porta 16
#define gpio_mb_cs   &gpio2_porta 20 
#define gpio_mb_sck  &gpio2_porta 14
#define gpio_mb_miso &gpio2_porta 17
#define gpio_mb_mosi &gpio2_porta 16
#define gpio_mb_scl  &gpio0_porta 18
#define gpio_mb_sda  &gpio0_porta 19

// AP6203BM
#define gpio_bt_shutdown &gpio2_porta 28
#define gpio_bt_device_wakeup &gpio2_porta 29
#define gpio_bt_host_wakeup &gpio2_porta 30

/*
* padctrl macros
*
* left-pinctrl (gpio0,gpio1): &light_padctrl1
* right-pinctrl (gpio2,gpio3): &light_padctrl0
* aon-pinctrl (ao_gpio4,ao_gpio): &light_padctrl
*
*/

// P8 cape header
#define padctrl_P8_03 &light_padctrl1
#define padctrl_P8_04 &light_padctrl1
#define padctrl_P8_05 &light_padctrl1
#define padctrl_P8_06 &light_padctrl1
#define padctrl_P8_07 &light_padctrl1
#define padctrl_P8_08 &light_padctrl1
#define padctrl_P8_09 &light_padctrl1
#define padctrl_P8_10 &light_padctrl1
#define padctrl_P8_11 &light_padctrl1
#define padctrl_P8_12 &light_padctrl1
#define padctrl_P8_13 &light_padctrl0
#define padctrl_P8_14 &light_padctrl1
#define padctrl_P8_15 &light_padctrl0
#define padctrl_P8_16 &light_padctrl1
#define padctrl_P8_17 &light_padctrl0
#define padctrl_P8_18 &light_padctrl1
#define padctrl_P8_19 &light_padctrl0
#define padctrl_P8_20 &light_padctrl1
#define padctrl_P8_21 &light_padctrl1
#define padctrl_P8_22 &light_padctrl1
#define padctrl_P8_23 &light_padctrl1
#define padctrl_P8_24 &light_padctrl1
#define padctrl_P8_25 &light_padctrl1
#define padctrl_P8_26 &light_padctrl1
#define padctrl_P8_27 &light_padctrl1
#define padctrl_P8_28 &light_padctrl1
#define padctrl_P8_29 &light_padctrl1
#define padctrl_P8_30 &light_padctrl1
#define padctrl_P8_31 &light_padctrl1
#define padctrl_P8_32 &light_padctrl1
#define padctrl_P8_33 &light_padctrl1
#define padctrl_P8_34 &light_padctrl1
#define padctrl_P8_35 &light_padctrl1
#define padctrl_P8_36 &light_padctrl1
#define padctrl_P8_37 &light_padctrl1
#define padctrl_P8_38 &light_padctrl1
#define padctrl_P8_39 &light_padctrl1
#define padctrl_P8_40 &light_padctrl1
#define padctrl_P8_41 &light_padctrl1
#define padctrl_P8_42 &light_padctrl1
#define padctrl_P8_43 &light_padctrl1
#define padctrl_P8_44 &light_padctrl1
#define padctrl_P8_45 &light_padctrl1
#define padctrl_P8_46 &light_padctrl1

// P9 cape header
#define padctrl_P9_11 &light_padctrl1
#define padctrl_P9_12 &light_padctrl0
#define padctrl_P9_13 &light_padctrl1
#define padctrl_P9_14 &light_padctrl0
#define padctrl_P9_15 &light_padctrl0
#define padctrl_P9_16 &light_padctrl0
#define padctrl_P9_17 &light_padctrl1
#define padctrl_P9_18 &light_padctrl1
#define padctrl_P9_19 &light_padctrl0
#define padctrl_P9_20 &light_padctrl0
#define padctrl_P9_21 &light_padctrl1
#define padctrl_P9_22 &light_padctrl1
#define padctrl_P9_23 &light_padctrl0
#define padctrl_P9_24 &light_padctrl1
#define padctrl_P9_25 &light_padctrl0
#define padctrl_P9_26 &light_padctrl1
#define padctrl_P9_27 &light_padctrl0
#define padctrl_P9_28 &light_padctrl0
#define padctrl_P9_29 &light_padctrl0
#define padctrl_P9_30 &light_padctrl0
#define padctrl_P9_31 &light_padctrl0
#define padctrl_P9_41 &light_padctrl0
#define padctrl_P9_42 &light_padctrl0

// mikroBus port
#define padctrl_mb_pwm  &light_padctrl0
#define padctrl_mb_rst  &light_padctrl
#define padctrl_mb_int  &light_padctrl0
#define padctrl_mb_rxd  &light_padctrl1
#define padctrl_mb_txd  &light_padctrl1
#define padctrl_mb_cs   &light_padctrl0 
#define padctrl_mb_sck  &light_padctrl0
#define padctrl_mb_miso &light_padctrl0
#define padctrl_mb_mosi &light_padctrl0
#define padctrl_mb_scl  &light_padctrl1
#define padctrl_mb_sda  &light_padctrl1

// AM6203BM
#define padctrl_bt_shutdown &light_padctrl0
#define padctrl_bt_device_wakeup &light_padctrl0
#define padctrl_bt_host_wakeup &light_padctrl0

/*
* Cape compatibility PinMuxing macros
*/

// P8 cape header
#define P8_03(muxmode, config) FM_GPIO1_21 muxmode config
#define P8_04(muxmode, config) FM_GPIO1_22 muxmode config
#define P8_05(muxmode, config) FM_GPIO1_23 muxmode config
#define P8_06(muxmode, config) FM_GPIO1_24 muxmode config
#define P8_07(muxmode, config) FM_GPIO1_25 muxmode config
#define P8_08(muxmode, config) FM_GPIO1_26 muxmode config
#define P8_09(muxmode, config) FM_GPIO1_27 muxmode config
#define P8_10(muxmode, config) FM_GPIO1_28 muxmode config
#define P8_11(muxmode, config) FM_GPIO1_29 muxmode config
#define P8_12(muxmode, config) FM_GPIO1_30 muxmode config
#define P8_13(muxmode, config) FM_GPIO3_2 muxmode config
#define P8_14(muxmode, config) FM_CLK_OUT_3 muxmode config
#define P8_15(muxmode, config) FM_GPIO3_0 muxmode config
#define P8_16(muxmode, config) FM_GPIO0_20 muxmode config
#define P8_17(muxmode, config) FM_GPIO3_1 muxmode config
#define P8_18(muxmode, config) FM_GPIO1_5 muxmode config
#define P8_19(muxmode, config) FM_GPIO3_3 muxmode config
#define P8_20(muxmode, config) FM_GPIO1_6 muxmode config
#define P8_21(muxmode, config) FM_GPIO1_7 muxmode config
#define P8_22(muxmode, config) FM_GPIO1_8 muxmode config
#define P8_23(muxmode, config) FM_GPIO1_9 muxmode config
#define P8_24(muxmode, config) FM_GPIO1_10 muxmode config
#define P8_25(muxmode, config) FM_GPIO1_11 muxmode config
#define P8_26(muxmode, config) FM_GPIO1_12 muxmode config
#define P8_27(muxmode, config) FM_GPIO1_15 muxmode config
#define P8_28(muxmode, config) FM_GPIO1_16 muxmode config
#define P8_29(muxmode, config) FM_GPIO1_14 muxmode config
#define P8_30(muxmode, config) FM_GPIO1_13 muxmode config
#define P8_31(muxmode, config) FM_GPIO1_3 muxmode config
#define P8_32(muxmode, config) FM_GPIO1_4 muxmode config
#define P8_33(muxmode, config) FM_GPIO1_2 muxmode config
#define P8_34(muxmode, config) FM_GPIO1_0 muxmode config
#define P8_35(muxmode, config) FM_GPIO1_1 muxmode config
#define P8_36(muxmode, config) FM_GPIO0_31 muxmode config
#define P8_37(muxmode, config) FM_GPIO0_29 muxmode config
#define P8_38(muxmode, config) FM_GPIO0_30 muxmode config
#define P8_39(muxmode, config) FM_GPIO0_27 muxmode config
#define P8_40(muxmode, config) FM_GPIO0_28 muxmode config
#define P8_41(muxmode, config) FM_GPIO0_25 muxmode config
#define P8_42(muxmode, config) FM_GPIO0_26 muxmode config
#define P8_43(muxmode, config) FM_GPIO0_23 muxmode config
#define P8_44(muxmode, config) FM_GPIO0_24 muxmode config
#define P8_45(muxmode, config) FM_GPIO0_21 muxmode config
#define P8_46(muxmode, config) FM_GPIO0_22 muxmode config

// P9 cape header
#define P9_11(muxmode, config) FM_UART1_TXD muxmode config
#define P9_12(muxmode, config) FM_QSPI0_CSN0 muxmode config
#define P9_13(muxmode, config) FM_UART1_RXD muxmode config
#define P9_14(muxmode, config) FM_QSPI0_D1_MISO muxmode config
#define P9_15(muxmode, config) FM_QSPI0_D2_WP muxmode config
#define P9_16(muxmode, config) FM_QSPI0_D0_MOSI muxmode config
#define P9_17(muxmode, config) FM_QSPI1_CSN0 muxmode config
#define P9_18(muxmode, config) FM_QSPI1_D0_MOSI muxmode config
#define P9_19(muxmode, config) FM_I2C2_SCL muxmode config
#define P9_20(muxmode, config) FM_I2C2_SDA muxmode config
#define P9_21(muxmode, config) FM_QSPI1_D1_MISO muxmode config
#define P9_22(muxmode, config) FM_QSPI1_SCLK muxmode config
#define P9_23(muxmode, config) FM_QSPI0_D3_HOLD muxmode config
#define P9_24(muxmode, config) FM_QSPI1_D2_WP muxmode config
#define P9_25(muxmode, config) FM_GPIO2_18 muxmode config
#define P9_26(muxmode, config) FM_QSPI1_D3_HOLD muxmode config
#define P9_27(muxmode, config) FM_GPIO2_19 muxmode config
#define P9_28(muxmode, config) FM_SPI_CSN muxmode config
#define P9_29(muxmode, config) FM_SPI_MISO muxmode config
#define P9_30(muxmode, config) FM_SPI_MOSI muxmode config
#define P9_31(muxmode, config) FM_SPI_SCLK muxmode config
#define P9_41(muxmode, config) FM_GPIO2_13 muxmode config
#define P9_42(muxmode, config) FM_QSPI0_SCLK muxmode config

// mikroBus port
#define mb_pwm(muxmode, config)  FM_QSPI0_CSN1 muxmode config
#define mb_rst(muxmode, config)  FM_AUDIO_PA3 muxmode config
#define mb_int(muxmode, config)  FM_GPIO2_21 muxmode config
#define mb_rxd(muxmode, config)  FM_UART3_RXD muxmode config
#define mb_txd(muxmode, config)  FM_UART3_TXD muxmode config
#define mb_cs(muxmode, config)   FM_GPIO2_20 muxmode config 
#define mb_sck(muxmode, config)  FM_SPI_SCLK muxmode config
#define mb_miso(muxmode, config) FM_SPI_MISO muxmode config
#define mb_mosi(muxmode, config) FM_SPI_MOSI muxmode config
#define mb_scl(muxmode, config)  FM_GPIO0_18 muxmode config
#define mb_sda(muxmode, config)  FM_GPIO0_19 muxmode config

// AM6203BM
#define bt_shutdown(muxmode, config) FM_SDIO1_WPRTN muxmode config
#define bt_device_wakeup(muxmode, config) FM_SDIO1_DETN muxmode config
#define bt_host_wakeup(muxmode, config) FM_GPIO2_30 muxmode config

#endif
