/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * This file is part of wl1271
 *
 * Copyright (C) 2008-2009 Nokia Corporation
 *
 * Contact: Luciano Coelho <luciano.coelho@nokia.com>
 */

#ifndef __BOOT_H__
#define __BOOT_H__

#include "wlcore.h"

int cc33xx_init_fw(struct wl1271 *wl);

void cc33xx_handle_boot_irqs(struct wl1271 *wl, u32 pending_interrupts);

#define WL1271_NO_SUBBANDS 8
#define WL1271_NO_POWER_LEVELS 4
#define WL1271_FW_VERSION_MAX_LEN 20

#define SECOND_LOADER_NAME "ti-connectivity/cc33xx_2nd_loader.bin"
#define FW_NAME "ti-connectivity/cc33xx_fw.bin"

struct wl1271_static_data {
	u8 mac_address[ETH_ALEN];
	u8 padding[2];
	u8 fw_version[WL1271_FW_VERSION_MAX_LEN];
	u32 hw_version;
	u8 tx_power_table[WL1271_NO_SUBBANDS][WL1271_NO_POWER_LEVELS];
	u8 priv[0];
};

struct cc33xx_fw_download {
	atomic_t pending_irqs;
	struct completion wait_on_irq;
	size_t max_transfer_size;
};

/* number of times we try to read the INIT interrupt */
#define INIT_LOOP 20000

/* delay between retries */
#define INIT_LOOP_DELAY 50

#define WU_COUNTER_PAUSE_VAL 0x3FF
#define WELP_ARM_COMMAND_VAL 0x4

#endif
