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

#define DBGPRINT(level, fmt, ...) if (level <= OMAPFB_DBG) do { \
					printk(KERN_DEBUG "%s: "fmt, \
						__FUNCTION__, ## __VA_ARGS__); \
				  } while (0)
#define DBGENTER(level) DBGPRINT(level, "Enter\n")
#define DBGLEAVE(level)	DBGPRINT(level, "Leave\n")

#else	/* OMAPFB_DBG */

#define DBGPRINT(level, format, ...)
#define DBGENTER(level)
#define DBGLEAVE(level)

#endif	/* OMAPFB_DBG */

#endif /* __OMAPFB_DEBUG_H */
