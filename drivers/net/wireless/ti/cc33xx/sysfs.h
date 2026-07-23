/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * This file is part of wlcore
 *
 * Copyright (C) 2013 Texas Instruments Inc.
 */

#ifndef _CC33XX_SYSFS_H_
#define _CC33XX_SYSFS_H_

int cc33xx_sysfs_init(struct cc33xx *cc);
void cc33xx_sysfs_free(struct cc33xx *cc);

#endif