/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * This file is part of cc33xx
 *
 * Copyright (C) 1998-2009 Texas Instruments. All rights reserved.
 * Copyright (C) 2008-2010 Nokia Corporation
 *
 * Contact: Luciano Coelho <luciano.coelho@nokia.com>
 */

#ifndef __IO_H__
#define __IO_H__


struct cc33xx;

void wlcore_disable_interrupts_nosync(struct cc33xx *wl);
void wlcore_enable_interrupts(struct cc33xx *wl);
void cc33xx_io_reset(struct cc33xx *wl);
void cc33xx_io_init(struct cc33xx *wl);
int __must_check wlcore_raw_read(struct cc33xx *wl, int addr,
				 void *buf, size_t len, bool fixed);
int __must_check wlcore_write(struct cc33xx *wl, int addr,
			      void *buf, size_t len, bool fixed);
void claim_core_status_lock(struct cc33xx *wl);
void release_core_status_lock(struct cc33xx *wl);
void cc33xx_power_off(struct cc33xx *wl);
int cc33xx_power_on(struct cc33xx *wl);
int wlcore_translate_addr(struct cc33xx *wl, int addr);
bool cc33xx_set_block_size(struct cc33xx *wl);


#endif /* __IO_H__ */
