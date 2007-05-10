/*
 * Copyright (C) 2005-2006 by Texas Instruments
 *
 * The Inventra Controller Driver for Linux is free software; you
 * can redistribute it and/or modify it under the terms of the GNU
 * General Public License version 2 as published by the Free Software
 * Foundation.
 */

#ifndef __MUSB_OMAP243X_H__
#define __MUSB_OMAP243X_H__

#ifdef CONFIG_ARCH_OMAP2430

#include <asm/arch/usb.h>

/*
 * OMAP2430-specific definitions
 */

#define MENTOR_BASE_OFFSET	0
#define HS_OTG(offset)		(OMAP243X_HS_BASE + (offset))
#define OTG_REVISION		HS_OTG(0x400)
#define OTG_SYSCONFIG		HS_OTG(0x404)
#define OTG_SYSSTATUS		HS_OTG(0x408)
#define OTG_INTERFSEL		HS_OTG(0x40c)
#define OTG_SIMENABLE		HS_OTG(0x410)

#endif	/* CONFIG_ARCH_OMAP2430 */

#endif	/* __MUSB_OMAP243X_H__ */
