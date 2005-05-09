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

#ifdef OMAPFB_DBG

#define DBG_BUF_SIZE		2048
#define MAX_DBG_INDENT_LEVEL	5
#define DBG_INDENT_SIZE		3
#define MAX_DBG_MESSAGES	0

static int dbg_indent;
static int dbg_cnt;
static char dbg_buf[DBG_BUF_SIZE];
static spinlock_t dbg_spinlock = SPIN_LOCK_UNLOCKED;

static inline void dbg_print(int level, const char *fmt, ...)
{
	if (level <= OMAPFB_DBG) {
		if (!MAX_DBG_MESSAGES || dbg_cnt < MAX_DBG_MESSAGES) {
			va_list args;
			int	ind = dbg_indent;
			unsigned long flags;

			spin_lock_irqsave(&dbg_spinlock, flags);
			dbg_cnt++;
			if (ind > MAX_DBG_INDENT_LEVEL)
				ind = MAX_DBG_INDENT_LEVEL;

			printk("%*s", ind * DBG_INDENT_SIZE, "");
			va_start(args, fmt);
			vsnprintf(dbg_buf, sizeof(dbg_buf), fmt, args);
			printk(dbg_buf);
			va_end(args);
			spin_unlock_irqrestore(&dbg_spinlock, flags);
		}
	}
}

#define DBGPRINT	dbg_print

#define DBGENTER(level)	do { \
		dbg_print(level, "%s: Enter\n", __FUNCTION__); \
		dbg_indent++; \
	} while (0)

#define DBGLEAVE(level)	do { \
		dbg_indent--; \
		dbg_print(level, "%s: Leave\n", __FUNCTION__); \
	} while (0)

static inline void dump_dma_regs(int lch)
{
#define _R(x) __REG16(OMAP_DMA_##x(lch))

	dbg_print(4, "\nCSDP  :%#06x CCR      :%#06x CSSA_U  :%#06x "
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
}

#define DUMP_DMA_REGS(lch) dump_dma_regs(lch)

#else	/* OMAPFB_DBG */

#define DBGPRINT(level, format, ...)
#define DBGENTER(level)
#define DBGLEAVE(level)
#define DUMP_DMA_REGS(lch)

#endif	/* OMAPFB_DBG */

#endif /* __OMAPFB_DEBUG_H */
