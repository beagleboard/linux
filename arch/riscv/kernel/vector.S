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

	.word 0x003072d7 	/* vsetvli	t0, x0, e8,m8 	*/
	.word 0x02050027 	/* vsb.v	v0,  (a0)	*/
	addi	a0, a0, RISCV_VECTOR_VLENB*8
	.word 0x02050427 	/* vsb.v	v8,  (a0) 	*/
	addi	a0, a0, RISCV_VECTOR_VLENB*8
	.word 0x02050827 	/* vsb.v	v16, (a0) 	*/
	addi	a0, a0, RISCV_VECTOR_VLENB*8
	.word 0x02050c27 	/* vsb.v	v24, (a0) 	*/

	csrc	sstatus, t1
	ret
ENDPROC(__vstate_save)

ENTRY(__vstate_restore)
	li	a2,  TASK_THREAD_V0
	add	a0,  a0, a2
	mv	t2,  a0

	li	t1, (SR_VS | SR_FS)
	csrs	sstatus, t1

	.word 0x003072d7	/* vsetvli	t0, x0, e8,m8 	*/
	.word 0x12050007	/* vlb.v	v0,  (a0) 	*/
	addi	a0, a0, RISCV_VECTOR_VLENB*8
	.word 0x12050407	/* vlb.v	v8,  (a0)	*/
	addi	a0, a0, RISCV_VECTOR_VLENB*8
	.word 0x12050807 	/* vlb.v	v16, (a0)	*/
	addi	a0, a0, RISCV_VECTOR_VLENB*8
	.word 0x12050c07	/* vlb.v	v24, (a0)	*/

	mv	a0,  t2
	ld	t0,  TASK_THREAD_VSTART_V0(a0)
	csrw	CSR_VSTART, t0
	ld	t0,  TASK_THREAD_VXSAT_V0(a0)
	csrw	CSR_VXSAT, t0
	ld	t0,  TASK_THREAD_VXRM_V0(a0)
	csrw	CSR_VXRM, t0
	ld	t0,  TASK_THREAD_VL_V0(a0)
	ld	t2,  TASK_THREAD_VTYPE_V0(a0)

	.word 0x8072fe57 	/* vsetvl	t3, t0, t2	*/

	csrc	sstatus, t1
	ret
ENDPROC(__vstate_restore)
