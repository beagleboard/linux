/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2012 ARM Limited
 * Copyright (C) 2014 Regents of the University of California
 * Copyright (C) 2017 SiFive
 */

#ifndef _ASM_RISCV_VDSO_H
#define _ASM_RISCV_VDSO_H

#include <linux/types.h>
#include <generated/vdso-offsets.h>

#ifndef CONFIG_GENERIC_TIME_VSYSCALL
struct vdso_data {
};
#endif

#define VDSO_SYMBOL(base, name)							\
	(void __user *)((unsigned long)(base) + __vdso_##name##_offset)

#ifdef CONFIG_COMPAT
#include <generated/compat_vdso-offsets.h>

#define COMPAT_VDSO_SYMBOL(base, name)						\
	(void __user *)((unsigned long)(base) + compat__vdso_##name##_offset)

#endif /* CONFIG_COMPAT */

asmlinkage long sys_riscv_flush_icache(uintptr_t, uintptr_t, uintptr_t);

#endif /* _ASM_RISCV_VDSO_H */
