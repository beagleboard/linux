/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * This file is part of cc33xx
 *
 * Copyright (C) 2009 Nokia Corporation
 *
 * Contact: Luciano Coelho <luciano.coelho@nokia.com>
 */

#ifndef __INIT_H__
#define __INIT_H__

#include "wlcore.h"

int cc33xx_init_energy_detection(struct cc33xx *wl);
int cc33xx_hw_init(struct cc33xx *wl);
int cc33xx_download_ini_params_and_wait(struct cc33xx *wl);
int cc33xx_init_vif_specific(struct cc33xx *wl, struct ieee80211_vif *vif);
int cc33xx_ap_init_templates(struct cc33xx *wl, struct ieee80211_vif *vif);
int cc33xx_sta_hw_init(struct cc33xx *wl, struct cc33xx_vif *wlvif);
int download_static_calibration_data(struct cc33xx *wl);

#endif
