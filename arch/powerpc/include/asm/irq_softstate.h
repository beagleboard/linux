#ifndef _ASM_POWERPC_IRQ_SOFTSTATE_H
#define _ASM_POWERPC_IRQ_SOFTSTATE_H

#ifdef __ASSEMBLY__

.macro HARD_ENABLE_INTS tmp=r10
#ifdef CONFIG_PPC_BOOK3E
	wrteei	1
#else
	li	\tmp,MSR_RI
	ori	\tmp,\tmp,MSR_EE
	mtmsrd	\tmp,1
#endif /* CONFIG_PPC_BOOK3E */
.endm

.macro HARD_DISABLE_INTS tmp=r10
#ifdef CONFIG_PPC_BOOK3E
	wrteei	0
#else
	li	\tmp,0
	mtmsrd	\tmp,1		  /* Update machine state */
#endif /* CONFIG_PPC_BOOK3E */
.endm

.macro HARD_DISABLE_INTS_RI tmp=r10
#ifdef CONFIG_PPC_BOOK3E
	wrteei	0
#else
	/*
	 * CAUTION: using r9-r11 the way they are is assumed by the
	 * caller.
	 */
	li	\tmp,MSR_RI
	mtmsrd	\tmp,1		  /* Update machine state */
#endif /* CONFIG_PPC_BOOK3E */
.endm

#ifdef CONFIG_IPIPE

  /* Do NOT alter Rc(eq) in this code;  our caller uses it. */
#define __COPY_SOFTISTATE(mreg)			\
	ld	mreg,PACAROOTPCPU(r13);		\
	ld	mreg,0(mreg);			\
	nor	mreg,mreg,mreg;			\
	clrldi	mreg,mreg,63;			\

/* Do NOT alter Rc(eq) in this code;  our caller uses it. */
#define COPY_SOFTISTATE(mreg)			\
	__COPY_SOFTISTATE(mreg);		\
	std	mreg,SOFTE(r1)

#ifdef CONFIG_PPC_BOOK3E
#define SPECIAL_SAVE_SOFTISTATE(mreg)		\
	__COPY_SOFTISTATE(mreg);		\
	SPECIAL_EXC_STORE(mreg, SOFTE)
#endif

#define EXC_SAVE_SOFTISTATE(mreg)		\
	COPY_SOFTISTATE(mreg)

#define RECONCILE_IRQ_STATE(__rA, __rB)	HARD_DISABLE_INTS __rA

#else /* !CONFIG_IPIPE */

#define COPY_SOFTISTATE(mreg)			\
	li	mreg,1
	std	mreg,SOFTE(r1)

#ifdef CONFIG_PPC_BOOK3E
#define SPECIAL_SAVE_SOFTISTATE(mreg)		\
	lbz	mreg,PACASOFTIRQEN(r13);	\
	SPECIAL_EXC_STORE(mreg, SOFTE)
#endif

#define EXC_SAVE_SOFTISTATE(mreg)		\
	COPY_SOFTISTATE(mreg)

 /*
 * This is used by assembly code to soft-disable interrupts first and
 * reconcile irq state.
 *
 * NB: This may call C code, so the caller must be prepared for volatiles to
 * be clobbered.
 */
#ifdef CONFIG_TRACE_IRQFLAGS
#define RECONCILE_IRQ_STATE(__rA, __rB)		\
	lbz	__rA,PACASOFTIRQEN(r13);	\
	lbz	__rB,PACAIRQHAPPENED(r13);	\
	cmpwi	cr0,__rA,0;			\
	li	__rA,0;				\
	ori	__rB,__rB,PACA_IRQ_HARD_DIS;	\
	stb	__rB,PACAIRQHAPPENED(r13);	\
	beq	44f;				\
	stb	__rA,PACASOFTIRQEN(r13);	\
	TRACE_DISABLE_INTS;			\
44:
#else
#define RECONCILE_IRQ_STATE(__rA, __rB)		\
	lbz	__rA,PACAIRQHAPPENED(r13);	\
	li	__rB,0;				\
	ori	__rA,__rA,PACA_IRQ_HARD_DIS;	\
	stb	__rB,PACASOFTIRQEN(r13);	\
	stb	__rA,PACAIRQHAPPENED(r13)
#endif /* !CONFIG_TRACE_IRQFLAGS */

#endif /* !CONFIG_IPIPE */

#endif /* __ASSEMBLY__ */

#endif /* _ASM_POWERPC_IRQ_SOFTSTATE_H */
