/* SPDX-License-Identifier: GPL-2.0 */
/*
 * TI j721e Cadence MHDP DP wrapper
 *
 * Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Jyri Sarha <jsarha@ti.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef CDNS_MHDP_J721E_H
#define CDNS_MHDP_J721E_H

#include <linux/platform_device.h>
#include "cdns-mhdp.h"

struct cdns_mhdp_j721e_wrap;

#ifdef CONFIG_DRM_CDNS_MHDP_J721E

int cdns_mhdp_j721e_init(struct cdns_mhdp_device *mhdp);

void cdns_mhdp_j721e_fini(struct cdns_mhdp_device *mhdp);

void cdns_mhdp_j721e_enable(struct cdns_mhdp_device *mhdp);

void cdns_mhdp_j721e_disable(struct cdns_mhdp_device *mhdp);

#else

static inline
int cdns_mhdp_j721e_init(struct cdns_mhdp_device *mhdp)
{
	return 0;
}

static inline
void cdns_mhdp_j721e_fini(struct cdns_mhdp_device *mhdp)
{
}

static inline
void cdns_mhdp_j721e_sst_enable(struct cdns_mhdp_device *mhdp);
{
}

static inline
void cdns_mhdp_j721e_sst_disable(struct cdns_mhdp_device *mhdp)
{
}
#endif /* CONFIG_DRM_CDNS_MHDP_J721E */

#endif /* !CDNS_MHDP_J721E_H */
