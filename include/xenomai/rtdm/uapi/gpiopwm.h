/**
 * @file
 * Real-Time Driver Model for Xenomai, pwm header
 *
 * @note Copyright (C) 2015 Jorge Ramirez <jro@xenomai.org>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA.
 *
 * @ingroup rttesting
 */
#ifndef _RTDM_UAPI_PWM_H
#define _RTDM_UAPI_PWM_H

#include <linux/types.h>

#define RTPWM_PROFILE_VER			1

struct gpiopwm {
	unsigned int duty_cycle;
	unsigned int range_min;
	unsigned int range_max;
	unsigned int period;
	unsigned int gpio;
};

#define RTIOC_TYPE_PWM		RTDM_CLASS_PWM

#define GPIOPWM_RTIOC_SET_CONFIG \
	_IOW(RTIOC_TYPE_PWM, 0x00, struct gpiopwm)

#define GPIOPWM_RTIOC_GET_CONFIG \
	_IOR(RTIOC_TYPE_PWM, 0x10, struct gpiopwm)

#define GPIOPWM_RTIOC_START \
	_IO(RTIOC_TYPE_PWM, 0x20)

#define GPIOPWM_RTIOC_STOP \
	_IO(RTIOC_TYPE_PWM, 0x30)

#define GPIOPWM_RTIOC_CHANGE_DUTY_CYCLE \
	_IOW(RTIOC_TYPE_PWM, 0x40, unsigned int)


#endif /* !_RTDM_UAPI_TESTING_H */
