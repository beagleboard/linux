/*
 * File: drivers/video/omap_new/debug.c
 *
 * Debug support for the omapfb driver
 *
 * Copyright (C) 2004 Nokia Corporation
 * Author: Imre Deak <imre.deak@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#ifndef __OMAPFB_DEBUG_H
#define __OMAPFB_DEBUG_H

#include <asm/io.h>

#ifdef OMAPFB_DBG

#define DBGPRINT(level, fmt, ...) if (level <= OMAPFB_DBG) do { \
					printk(KERN_DEBUG "%s: "fmt, \
						__FUNCTION__, ## __VA_ARGS__); \
				  } while (0)
#define DBGENTER(level) DBGPRINT(level, "Enter\n")
#define DBGLEAVE(level)	DBGPRINT(level, "Leave\n")

static inline void dump_dma_regs(int lch)
{
#ifdef CONFIG_ARCH_OMAP1
#define _R(x) __REG16(OMAP_DMA_##x(lch))

	DBGPRINT(4, "\nCSDP  :%#06x CCR      :%#06x CSSA_U  :%#06x "
		    "\nCDSA_L:%#06x CDSA_U   :%#06x CEN     :%#06x "
		    "\nCFN   :%#06x CSFI     :%#06x CSEI    :%#06x "
		    "\nCSAC  :%#06x CICR     :%#06x CSR     :%04x "
		    "\nCSSA_L:%#06x CDAC     :%#06x CDEI    :%#06x "
		    "\nCDFI  :%#06x COLOR_L  :%#06x COLOR_U :%#06x "
		    "\nCCR2  :%#06x CLNK_CTRL:%#06x LCH_CTRL:%#06x\n",
		    _R(CSDP), _R(CCR), _R(CSSA_U),
		    _R(CDSA_L), _R(CDSA_U), _R(CEN),
		    _R(CFN), _R(CSFI), _R(CSEI),
		    _R(CSAC), _R(CICR), 0, /* _R(CSR), */
		    _R(CSSA_L), _R(CDAC), _R(CDEI),
		    _R(CDFI), _R(COLOR_L), _R(COLOR_U),
		    _R(CCR2), _R(CLNK_CTRL), _R(LCH_CTRL));
#undef _R
#endif
}

#define DUMP_DMA_REGS(lch) dump_dma_regs(lch)

#else	/* OMAPFB_DBG */

#define DBGPRINT(level, format, ...)
#define DBGENTER(level)
#define DBGLEAVE(level)
#define DUMP_DMA_REGS(lch)

#endif	/* OMAPFB_DBG */

#endif /* __OMAPFB_DEBUG_H */
