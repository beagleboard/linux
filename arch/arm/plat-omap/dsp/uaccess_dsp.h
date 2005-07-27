/*
 * linux/arch/arm/mach-omap/dsp/uaccess_dsp.h
 *
 * Header for user access functions for DSP driver
 *
 * Copyright (C) 2002-2005 Nokia Corporation
 *
 * Modified from linux/include/asm-arm/uaccess.h
 * by Toshihiro Kobayashi <toshihiro.kobayashi@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * 2004/06/29:  DSP Gateway version 3.3
 */

#ifndef _OMAP_DSP_UACCESS_DSP_H
#define _OMAP_DSP_UACCESS_DSP_H

#include <asm/uaccess.h>

#define HAVE_ASM_COPY_FROM_USER_DSP_2B

#ifdef HAVE_ASM_COPY_FROM_USER_DSP_2B
extern unsigned long __arch_copy_from_user_dsp_2b(void *to,
						  const void __user *from);
extern unsigned long __arch_copy_to_user_dsp_2b(void __user *to,
						const void *from);
#endif

extern unsigned long dspmem_base, dspmem_size;
#define is_dsp_internal_mem(va) \
	(((unsigned long)(va) >= dspmem_base) &&  \
	 ((unsigned long)(va) < dspmem_base + dspmem_size))


#ifndef HAVE_ASM_COPY_FROM_USER_DSP_2B
static __inline__ unsigned long copy_from_user_dsp_2b(void *to,
						      const void *from)
{
	unsigned short tmp;

	if (__arch_copy_from_user(&tmp, from, 2))
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
				if (__arch_copy_from_user_dsp_2b(to, from))
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
			if ((n = __arch_copy_from_user(to, from, n4)) != 0)
				return n + last_n;
			/* last 2 bytes */
			if (last_n) {
				to += n4;
				from += n4;
#ifdef HAVE_ASM_COPY_FROM_USER_DSP_2B
				if (__arch_copy_from_user_dsp_2b(to, from))
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
			n = __arch_copy_from_user(to, from, n);
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

	return __arch_copy_to_user(to, &tmp, 2);
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
				if (__arch_copy_to_user_dsp_2b(to, from))
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
			if ((n = __arch_copy_to_user(to, from, n4)) != 0)
				return n + last_n;
			/* last 2 bytes */
			if (last_n) {
				to += n4;
				from += n4;
#ifdef HAVE_ASM_COPY_FROM_USER_DSP_2B
				if (__arch_copy_to_user_dsp_2b(to, from))
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
			n = __arch_copy_to_user(to, from, n);
		}
	}
	return n;
}

#undef is_dsp_internal_mem

#endif /* _OMAP_DSP_UACCESS_DSP_H */
