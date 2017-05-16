/*
 * Copyright (C) 2014 Philippe Gerum <rpm@xenomai.org>.
 *
 * Xenomai is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version.
 *
 * Xenomai is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Xenomai; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 * 02111-1307, USA.
 */
#ifndef _COBALT_KERNEL_ANCILLARIES_H
#define _COBALT_KERNEL_ANCILLARIES_H

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/uidgid.h>
#include <cobalt/uapi/kernel/limits.h>

#define ksformat(__dst, __len, __fmt, __args...)			\
	({								\
		size_t __ret;						\
		__ret = snprintf(__dst, __len, __fmt, ##__args);	\
		if (__ret >= __len)					\
			__dst[__len-1] = '\0';				\
		__ret;							\
	})

#define kasformat(__fmt, __args...)					\
	({								\
		kasprintf(GFP_KERNEL, __fmt, ##__args);			\
	})

#define kvsformat(__dst, __len, __fmt, __ap)				\
	({								\
		size_t __ret;						\
		__ret = vsnprintf(__dst, __len, __fmt, __ap);		\
		if (__ret >= __len)					\
			__dst[__len-1] = '\0';				\
		__ret;							\
	})

#define kvasformat(__fmt, __ap)						\
	({								\
		kvasprintf(GFP_KERNEL, __fmt, __ap);			\
	})

void __knamecpy_requires_character_array_as_destination(void);

#define knamecpy(__dst, __src)						\
	({								\
		if (!__builtin_types_compatible_p(typeof(__dst), char[])) \
			__knamecpy_requires_character_array_as_destination();	\
		strncpy((__dst), __src, sizeof(__dst) - 1);		\
		__dst[sizeof(__dst) - 1] = '\0';			\
		__dst;							\
	 })

#define get_current_uuid() from_kuid_munged(current_user_ns(), current_uid())

#endif /* !_COBALT_KERNEL_ANCILLARIES_H */
