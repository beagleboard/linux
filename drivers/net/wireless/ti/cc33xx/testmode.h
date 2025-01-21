/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (C) 2022-2024 Texas Instruments Incorporated - https://www.ti.com/
 */

#ifndef __TESTMODE_H__
#define __TESTMODE_H__

int cc33xx_tm_cmd(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		  void *data, int len);

#endif /* __TESTMODE_H__ */
