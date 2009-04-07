/*
 * lp5521.h - header for LP5521 LED driver
 *
 * Copyright (C) 2009 Nokia Corporation
 *
 * Contact: Felipe Balbi <felipe.balbi@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#ifndef __LP5521_H_
#define __LP5521_H_

enum lp5521_mode {
	LP5521_MODE_LOAD,
	LP5521_MODE_RUN,
	LP5521_MODE_DIRECT_CONTROL,
};

struct lp5521_platform_data {
	enum lp5521_mode	mode;

	unsigned		red_present:1;
	unsigned		green_present:1;
	unsigned		blue_present:1;

	const char		*label;
};

#endif /* End of __LP5521_H */
