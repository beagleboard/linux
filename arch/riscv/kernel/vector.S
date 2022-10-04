/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2012 Regents of the University of California
 * Copyright (C) 2017 SiFive
 * Copyright (C) 2019 T-HEAD
 *
 *   This program is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU General Public License
 *   as published by the Free Software Foundation, version 2.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 */

#include <linux/linkage.h>

#include <asm/asm.h>
#include <asm/csr.h>
#include <asm/asm-offsets.h>

#if (defined(CONFIG_VECTOR_1_0) && defined(__THEAD_VERSION__))
#define V_ST	vse8.v
#define V_LD	vle8.v
#else
#define V_ST	vsb.v
#define V_LD	vlb.v
#endif

ENTRY(__vstate_save)
	li	a2,  TASK_THREAD_V0
	add	a0,  a0, a2

	li	t1, (SR_VS | SR_FS)
	csrs	sstatus, t1

	csrr	t0,  CSR_VSTART
	sd	t0,  TASK_THREAD_VSTART_V0(a0)
	csrr	t0,  CSR_VXSAT
	sd	t0,  TASK_THREAD_VXSAT_V0(a0)
	csrr	t0,  CSR_VXRM
	sd	t0,  TASK_THREAD_VXRM_V0(a0)
	csrr	t0,  CSR_VL
	sd	t0,  TASK_THREAD_VL_V0(a0)
	csrr	t0,  CSR_VTYPE
	sd	t0,  TASK_THREAD_VTYPE_V0(a0)

	vsetvli	t0, x0, e8,m8
	V_ST	v0,  (a0)
	addi	a0, a0, RISCV_VECTOR_VLENB*8
	V_ST	v8,  (a0)
	addi	a0, a0, RISCV_VECTOR_VLENB*8
	V_ST	v16, (a0)
	addi	a0, a0, RISCV_VECTOR_VLENB*8
	V_ST	v24, (a0)

	csrc	sstatus, t1
	ret
ENDPROC(__vstate_save)

ENTRY(__vstate_restore)
	li	a2,  TASK_THREAD_V0
	add	a0,  a0, a2
	mv	t2,  a0

	li	t1, (SR_VS | SR_FS)
	csrs	sstatus, t1

	vsetvli	t0, x0, e8,m8
	V_LD	v0,  (a0)
	addi	a0, a0, RISCV_VECTOR_VLENB*8
	V_LD	v8,  (a0)
	addi	a0, a0, RISCV_VECTOR_VLENB*8
	V_LD	v16, (a0)
	addi	a0, a0, RISCV_VECTOR_VLENB*8
	V_LD	v24, (a0)

	mv	a0,  t2
	ld	t0,  TASK_THREAD_VSTART_V0(a0)
	csrw	CSR_VSTART, t0
	ld	t0,  TASK_THREAD_VXSAT_V0(a0)
	csrw	CSR_VXSAT, t0
	ld	t0,  TASK_THREAD_VXRM_V0(a0)
	csrw	CSR_VXRM, t0
	ld	t0,  TASK_THREAD_VL_V0(a0)
	ld	t2,  TASK_THREAD_VTYPE_V0(a0)
#ifdef CONFIG_VECTOR_EMU
	srli	t3, t2, 63
	bne	t3,zero,1f
#endif
	vsetvl	t3, t0, t2
#ifdef CONFIG_VECTOR_EMU
	j	2f
1:	vsetvli zero,zero,e64,m2,d1
2:
#endif
	csrc	sstatus, t1
	ret
ENDPROC(__vstate_restore)
