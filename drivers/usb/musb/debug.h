/******************************************************************
 * Copyright 2005 Mentor Graphics Corporation
 * Copyright (C) 2005-2006 by Texas Instruments
 *
 * This file is part of the Inventra Controller Driver for Linux.
 *
 * The Inventra Controller Driver for Linux is free software; you
 * can redistribute it and/or modify it under the terms of the GNU
 * General Public License version 2 as published by the Free Software
 * Foundation.
 *
 * The Inventra Controller Driver for Linux is distributed in
 * the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with The Inventra Controller Driver for Linux ; if not,
 * write to the Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA  02111-1307  USA
 *
 * ANY DOWNLOAD, USE, REPRODUCTION, MODIFICATION OR DISTRIBUTION
 * OF THIS DRIVER INDICATES YOUR COMPLETE AND UNCONDITIONAL ACCEPTANCE
 * OF THOSE TERMS.THIS DRIVER IS PROVIDED "AS IS" AND MENTOR GRAPHICS
 * MAKES NO WARRANTIES, EXPRESS OR IMPLIED, RELATED TO THIS DRIVER.
 * MENTOR GRAPHICS SPECIFICALLY DISCLAIMS ALL IMPLIED WARRANTIES
 * OF MERCHANTABILITY; FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT.  MENTOR GRAPHICS DOES NOT PROVIDE SUPPORT
 * SERVICES OR UPDATES FOR THIS DRIVER, EVEN IF YOU ARE A MENTOR
 * GRAPHICS SUPPORT CUSTOMER.
 ******************************************************************/

#ifndef __MUSB_LINUX_DEBUG_H__
#define __MUSB_LINUX_DEBUG_H__

#define yprintk(facility, format, args...) \
	do { printk(facility "%s %d: " format , \
	__FUNCTION__, __LINE__ , ## args); } while (0)
#define WARN(fmt, args...) yprintk(KERN_WARNING,fmt, ## args)
#define INFO(fmt,args...) yprintk(KERN_INFO,fmt, ## args)
#define ERR(fmt,args...) yprintk(KERN_ERR,fmt, ## args)

#define xprintk(level, facility, format, args...) do { \
	if ( _dbg_level(level) ) { \
		printk(facility "%s %d: " format , \
				__FUNCTION__, __LINE__ , ## args); \
	} } while (0)

#if MUSB_DEBUG > 0
extern unsigned debug;
#else
#define debug	0
#endif

static inline int _dbg_level(unsigned l)
{
	return debug >= l;
}

#define DBG(level,fmt,args...) xprintk(level,KERN_DEBUG,fmt, ## args)

#endif				//  __MUSB_LINUX_DEBUG_H__
