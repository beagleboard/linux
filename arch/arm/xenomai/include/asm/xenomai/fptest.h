/*
 * Copyright (C) 2006 Gilles Chanteperdrix <gilles.chanteperdrix@xenomai.org>.
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
#ifndef _COBALT_ARM_ASM_FPTEST_H
#define _COBALT_ARM_ASM_FPTEST_H

#include <linux/errno.h>
#include <asm/hwcap.h>

#ifdef CONFIG_VFP
#define have_vfp (elf_hwcap & HWCAP_VFP)
#else /* !CONFIG_VFP */
#define have_vfp 0
#endif /* !CONFIG_VFP */

#include <asm/xenomai/uapi/fptest.h>

static inline int fp_kernel_supported(void)
{
	return 1;
}

static inline int fp_linux_begin(void)
{
	return -ENOSYS;
}

static inline void fp_linux_end(void)
{
}

static inline int fp_detect(void)
{
	return have_vfp ? __COBALT_HAVE_VFP : 0;
}

#endif /* _COBALT_ARM_ASM_FPTEST_H */
