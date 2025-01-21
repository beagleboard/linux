/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (C) 2022-2024 Texas Instruments Incorporated - https://www.ti.com/
 */

#ifndef __BOOT_H__
#define __BOOT_H__

#include "cc33xx.h"

int cc33xx_init_fw(struct cc33xx *cc);

void cc33xx_handle_boot_irqs(struct cc33xx *cc, u32 pending_interrupts);

#define SECOND_LOADER_NAME "ti-connectivity/cc33xx_2nd_loader.bin"
#define FW_NAME "ti-connectivity/cc33xx_fw.bin"

struct cc33xx_fw_download {
	atomic_t pending_irqs;
	struct completion wait_on_irq;
	size_t max_transfer_size;
};

#endif /* __BOOT_H__ */
