/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (C) 2022-2024 Texas Instruments Incorporated - https://www.ti.com/
 */

#ifndef __INIT_H__
#define __INIT_H__

#include "cc33xx.h"

int cc33xx_hw_init(struct cc33xx *cc);
int cc33xx_download_ini_params_and_wait(struct cc33xx *cc);
int cc33xx_init_vif_specific(struct cc33xx *cc, struct ieee80211_vif *vif);

#endif /* __INIT_H__ */
