/*
 * Header file for LCD controller
 *
 * Copyright (C) {2011} Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
 * GNU General Public License for more details.
 **/

#ifndef __OMAP2_LCDC_H
#define __OMAP2_LCDC_H

struct platform_device *am33xx_register_lcdc(
		struct da8xx_lcdc_platform_data *pdata);
#endif
