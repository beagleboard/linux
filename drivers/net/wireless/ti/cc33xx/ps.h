/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (C) 2022-2024 Texas Instruments Incorporated - https://www.ti.com/
 */

#ifndef __PS_H__
#define __PS_H__

#include "acx.h"

int cc33xx_ps_set_mode(struct cc33xx *cc, struct cc33xx_vif *wlvif,
		       enum cc33xx_cmd_ps_mode_e mode);
void cc33xx_ps_link_start(struct cc33xx *cc, struct cc33xx_vif *wlvif,
			  u8 hlid, bool clean_queues);

#endif /* __PS_H__ */
