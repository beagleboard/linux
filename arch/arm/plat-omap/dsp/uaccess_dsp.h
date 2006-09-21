/*
 * This file is part of OMAP DSP driver (DSP Gateway version 3.3.1)
 *
 * Copyright (C) 2002-2006 Nokia Corporation. All rights reserved.
 *
 * Contact: Toshihiro Kobayashi <toshihiro.kobayashi@nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#ifndef _OMAP_DSP_UACCESS_DSP_H
#define _OMAP_DSP_UACCESS_DSP_H

#include <asm/uaccess.h>
#include "dsp_common.h"

#define HAVE_ASM_COPY_FROM_USER_DSP_2B

#ifdef HAVE_ASM_COPY_FROM_USER_DSP_2B
extern unsigned long __copy_from_user_dsp_2b(void *to,
						  const void __user *from);
extern unsigned long __copy_to_user_dsp_2b(void __user *to,
						const void *from);
#endif

#ifndef HAVE_ASM_COPY_FROM_USER_DSP_2B
static __inline__ unsigned long copy_from_user_dsp_2b(void *to,
						      const void *from)
{
	unsigned short tmp;

	if (__copy_from_user(&tmp, from, 2))
		return 2;
	/* expecting compiler to generate "strh" instruction */
	*((unsigned short *)to) = tmp;
	return 0;
}
#endif

/*
 * @n must be multiple of 2
 */
static __inline__ unsigned long copy_from_user_dsp(void *to, const void *from,
						   unsigned long n)
{
	if (access_ok(VERIFY_READ, from, n)) {
		if ((is_dsp_internal_mem(to)) &&
		    (((unsigned long)to & 2) || (n & 2))) {
			/*
			 * DARAM/SARAM with odd word alignment
			 */
			unsigned long n4;
			unsigned long last_n;

			/* dest not aligned -- copy 2 bytes */
			if (((unsigned long)to & 2) && (n >= 2)) {
#ifdef HAVE_ASM_COPY_FROM_USER_DSP_2B
				if (__copy_from_user_dsp_2b(to, from))
#else
				if (copy_from_user_dsp_2b(to, from))
#endif
					return n;
				to += 2;
				from += 2;
				n -= 2;
			}
			/* middle 4*n bytes */
			last_n = n & 2;
			n4 = n - last_n;
			if ((n = __copy_from_user(to, from, n4)) != 0)
				return n + last_n;
			/* last 2 bytes */
			if (last_n) {
				to += n4;
				from += n4;
#ifdef HAVE_ASM_COPY_FROM_USER_DSP_2B
				if (__copy_from_user_dsp_2b(to, from))
#else
				if (copy_from_user_dsp_2b(to, from))
#endif
					return 2;
				n = 0;
			}
		} else {
			/*
			 * DARAM/SARAM with 4-byte alignment or
			 * external memory
			 */
			n = __copy_from_user(to, from, n);
		}
	}
	else	/* security hole - plug it */
		memzero(to, n);
	return n;
}

#ifndef HAVE_ASM_COPY_FROM_USER_DSP_2B
static __inline__ unsigned long copy_to_user_dsp_2b(void *to, const void *from)
{
	/* expecting compiler to generate "strh" instruction */
	unsigned short tmp = *(unsigned short *)from;

	return __copy_to_user(to, &tmp, 2);
}
#endif

/*
 * @n must be multiple of 2
 */
static __inline__ unsigned long copy_to_user_dsp(void *to, const void *from,
						 unsigned long n)
{
	if (access_ok(VERIFY_WRITE, to, n)) {
		if ((is_dsp_internal_mem(from)) &&
		    (((unsigned long)to & 2) || (n & 2))) {
			/*
			 * DARAM/SARAM with odd word alignment
			 */
			unsigned long n4;
			unsigned long last_n;

			/* dest not aligned -- copy 2 bytes */
			if (((unsigned long)to & 2) && (n >= 2)) {
#ifdef HAVE_ASM_COPY_FROM_USER_DSP_2B
				if (__copy_to_user_dsp_2b(to, from))
#else
				if (copy_to_user_dsp_2b(to, from))
#endif
					return n;
				to += 2;
				from += 2;
				n -= 2;
			}
			/* middle 4*n bytes */
			last_n = n & 2;
			n4 = n - last_n;
			if ((n = __copy_to_user(to, from, n4)) != 0)
				return n + last_n;
			/* last 2 bytes */
			if (last_n) {
				to += n4;
				from += n4;
#ifdef HAVE_ASM_COPY_FROM_USER_DSP_2B
				if (__copy_to_user_dsp_2b(to, from))
#else
				if (copy_to_user_dsp_2b(to, from))
#endif
					return 2;
				n = 0;
			}
		} else {
			/*
			 * DARAM/SARAM with 4-byte alignment or
			 * external memory
			 */
			n = __copy_to_user(to, from, n);
		}
	}
	return n;
}

#endif /* _OMAP_DSP_UACCESS_DSP_H */
