/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (C) 2022-2024 Texas Instruments Incorporated - https://www.ti.com/
 */

#ifndef __IO_H__
#define __IO_H__

struct cc33xx;

void cc33xx_disable_interrupts_nosync(struct cc33xx *cc);
void cc33xx_enable_interrupts(struct cc33xx *cc);
void cc33xx_io_reset(struct cc33xx *cc);
void cc33xx_io_init(struct cc33xx *cc);
int __must_check cc33xx_raw_read(struct cc33xx *cc, int addr,
				 void *buf, size_t len, bool fixed);
int __must_check cc33xx_write(struct cc33xx *cc, int addr,
			      void *buf, size_t len, bool fixed);
void claim_core_status_lock(struct cc33xx *cc);
void release_core_status_lock(struct cc33xx *cc);
void cc33xx_power_off(struct cc33xx *cc);
int cc33xx_power_on(struct cc33xx *cc);
int cc33xx_translate_addr(struct cc33xx *cc, int addr);
bool cc33xx_set_block_size(struct cc33xx *cc);

#endif /* __IO_H__ */
