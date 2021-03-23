/**
 * @note Copyright (C) 2016 Philippe Gerum <rpm@xenomai.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#ifndef _RTDM_UAPI_GPIO_H
#define _RTDM_UAPI_GPIO_H

struct rtdm_gpio_readout {
	nanosecs_abs_t timestamp;
	__s32 value;
};

#define GPIO_RTIOC_DIR_OUT	_IOW(RTDM_CLASS_GPIO, 0, int)
#define GPIO_RTIOC_DIR_IN	_IO(RTDM_CLASS_GPIO, 1)
#define GPIO_RTIOC_IRQEN	_IOW(RTDM_CLASS_GPIO, 2, int) /* GPIO trigger */
#define GPIO_RTIOC_IRQDIS	_IO(RTDM_CLASS_GPIO, 3)
#define GPIO_RTIOC_REQS		_IO(RTDM_CLASS_GPIO, 4)
#define GPIO_RTIOC_RELS		_IO(RTDM_CLASS_GPIO, 5)
#define GPIO_RTIOC_TS		_IOR(RTDM_CLASS_GPIO, 7, int)

#define GPIO_TRIGGER_NONE		0x0 /* unspecified */
#define GPIO_TRIGGER_EDGE_RISING	0x1
#define GPIO_TRIGGER_EDGE_FALLING	0x2
#define GPIO_TRIGGER_LEVEL_HIGH		0x4
#define GPIO_TRIGGER_LEVEL_LOW		0x8
#define GPIO_TRIGGER_MASK		0xf

#endif /* !_RTDM_UAPI_GPIO_H */
