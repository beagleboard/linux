/*
 * This file is part of the Xenomai project.
 *
 * Copyright (C) 2014 Philippe Gerum <rpm@xenomai.org>
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
 */
#ifndef _RTDM_UAPI_AUTOTUNE_H
#define _RTDM_UAPI_AUTOTUNE_H

#include <linux/types.h>

#define RTDM_CLASS_AUTOTUNE		RTDM_CLASS_MISC
#define RTDM_SUBCLASS_AUTOTUNE		0

struct autotune_setup {
	__u32 period;
	__u32 quiet;
};

#define AUTOTUNE_RTIOC_IRQ		_IOW(RTDM_CLASS_AUTOTUNE, 0, struct autotune_setup)
#define AUTOTUNE_RTIOC_KERN		_IOW(RTDM_CLASS_AUTOTUNE, 1, struct autotune_setup)
#define AUTOTUNE_RTIOC_USER		_IOW(RTDM_CLASS_AUTOTUNE, 2, struct autotune_setup)
#define AUTOTUNE_RTIOC_PULSE		_IOW(RTDM_CLASS_AUTOTUNE, 3, __u64)
#define AUTOTUNE_RTIOC_RUN		_IOR(RTDM_CLASS_AUTOTUNE, 4, __u32)
#define AUTOTUNE_RTIOC_RESET		_IO(RTDM_CLASS_AUTOTUNE, 5)

#endif /* !_RTDM_UAPI_AUTOTUNE_H */
