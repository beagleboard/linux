/*
 * This file is part of TI BLE over SDIO
 *
 * Copyright (C) 2022 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */


#define SDIO_HEADER_LEN			4

/* SD block size can not bigger than 64 due to buf size limit in firmware */
/* define SD block size for data Tx/Rx */
#define SDIO_BLOCK_SIZE			128

#define TIDRV_BT_RX_PACKET_BUFFER_SIZE 	(HCI_MAX_FRAME_SIZE)
#define TIDRV_SIZE_OF_CMD_BUFFER         (251)

#define MAX_RX_PACKET_SIZE	(((min_t (int, TIDRV_BT_RX_PACKET_BUFFER_SIZE, \
			TIDRV_SIZE_OF_CMD_BUFFER) + SDIO_HEADER_LEN \
			+ SDIO_BLOCK_SIZE - 1) / SDIO_BLOCK_SIZE) \
			* SDIO_BLOCK_SIZE)


/* Max retry number of CMD53 write */
#define MAX_SDIO_TX_RETRY		2

/* register bitmasks */
#define HOST_POWER_UP_MASK			1 // 1 to power up BLE SDIO firmware wake up bit */

#define INTRPT_ENABLE_MASk		    1 //interrupt set

struct btti_sdio_dev_reg_map {
	u8 sdio_wup_ble;
	u8 sdio_enable_int;
	u8 sdio_rt_data;
	u8 sdio_cl_int;
	u8 bt_mode_status;
	u8 sdio_pc_rrt;//ack interrupt
};

struct btti_sdio_dev {
	struct sdio_func *func;
	const struct btti_sdio_dev_reg_map *reg_map;
	u8 bt_mode_status;/* the status read from 0x20 address */
	struct btti_private *private_data;
	u8 sdio_header[SDIO_HEADER_LEN];
	bool Intrpt_triggered;
};

struct btti_sdio_device {
	const struct btti_sdio_dev_reg_map *reg_map;
};

struct btti_vendor_event {
	u8  event_code;//0xFF
	u8  total_length;//0x2
	u16 event_opcode;// EOGF  = 1  ; ESG = 0 ;  CMD
	u8  param_00[2];//Data
} __packed;

//BTTI_BLE_FIRMWARE_UP: EOGF  = 1  ; ESG = 0 ;  CMD= 42 => opcode  = 0x042A
#define BTTI_BLE_FIRMWARE_UP 0x2A04 //on the packet it is 0x042A

// CC33xx HW FIFOs must be accessed with 32 bit alignment
#define BTSDIO_RX_ALIGN		    4
#define BTSDIO_TX_ALIGN		    4


/* Macros for Data Alignment : size */
#define ALIGN_SZ(p, a)	\
	(((p) + ((a) - 1)) & ~((a) - 1))
