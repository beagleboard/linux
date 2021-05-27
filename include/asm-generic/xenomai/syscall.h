/*
 * Copyright (C) 2001,2002,2003,2004,2005 Philippe Gerum <rpm@xenomai.org>.
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
#ifndef _COBALT_ASM_GENERIC_SYSCALL_H
#define _COBALT_ASM_GENERIC_SYSCALL_H

#include <linux/types.h>
#include <linux/version.h>
#include <linux/uaccess.h>
#include <asm/xenomai/features.h>
#include <asm/xenomai/wrappers.h>
#include <asm/xenomai/machine.h>
#include <cobalt/uapi/asm-generic/syscall.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 0, 0)
#define access_rok(addr, size)	access_ok((addr), (size))
#define access_wok(addr, size)	access_ok((addr), (size))
#else
#define access_rok(addr, size)	access_ok(VERIFY_READ, (addr), (size))
#define access_wok(addr, size)	access_ok(VERIFY_WRITE, (addr), (size))
#endif

#define __xn_reg_arglist(regs)	\
	__xn_reg_arg1(regs),	\
	__xn_reg_arg2(regs),	\
	__xn_reg_arg3(regs),	\
	__xn_reg_arg4(regs),	\
	__xn_reg_arg5(regs)

#define __xn_copy_from_user(dstP, srcP, n)	raw_copy_from_user(dstP, srcP, n)
#define __xn_copy_to_user(dstP, srcP, n)	raw_copy_to_user(dstP, srcP, n)
#define __xn_put_user(src, dstP)		__put_user(src, dstP)
#define __xn_get_user(dst, srcP)		__get_user(dst, srcP)
#define __xn_strncpy_from_user(dstP, srcP, n)	strncpy_from_user(dstP, srcP, n)

static inline int cobalt_copy_from_user(void *dst, const void __user *src,
					size_t size)
{
	size_t remaining = size;

	if (likely(access_rok(src, size)))
		remaining = __xn_copy_from_user(dst, src, size);

	if (unlikely(remaining > 0)) {
		memset(dst + (size - remaining), 0, remaining);
		return -EFAULT;
	}
	return 0;
}

static inline int cobalt_copy_to_user(void __user *dst, const void *src,
				      size_t size)
{
	if (unlikely(!access_wok(dst, size) ||
	    __xn_copy_to_user(dst, src, size)))
		return -EFAULT;
	return 0;
}

static inline int cobalt_strncpy_from_user(char *dst, const char __user *src,
					   size_t count)
{
	if (unlikely(!access_rok(src, 1)))
		return -EFAULT;

	return __xn_strncpy_from_user(dst, src, count);
}

/* 32bit syscall emulation */
#define __COBALT_COMPAT_BIT	0x1
/* 32bit syscall emulation - extended form */
#define __COBALT_COMPATX_BIT	0x2

#endif /* !_COBALT_ASM_GENERIC_SYSCALL_H */
