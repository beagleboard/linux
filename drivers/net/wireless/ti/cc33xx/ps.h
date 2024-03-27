/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * This file is part of cc33xx
 *
 * Copyright (C) 2008-2009 Nokia Corporation
 *
 * Contact: Luciano Coelho <luciano.coelho@nokia.com>
 */

#ifndef __PS_H__
#define __PS_H__

#include "wlcore.h"
#include "acx.h"

int cc33xx_ps_set_mode(struct cc33xx *wl, struct cc33xx_vif *wlvif,
		       enum cc33xx_cmd_ps_mode mode);
void cc33xx_ps_link_start(struct cc33xx *wl, struct cc33xx_vif *wlvif,
			  u8 hlid, bool clean_queues);
void cc33xx_ps_link_end(struct cc33xx *wl, struct cc33xx_vif *wlvif, u8 hlid);

#endif /* __WL1271_PS_H__ */
