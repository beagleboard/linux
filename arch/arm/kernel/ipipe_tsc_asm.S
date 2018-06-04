#include <asm/assembler.h>
#include <asm/asm-offsets.h>
#include <asm/glue.h>

	.macro	usr_ret, reg
#ifdef CONFIG_ARM_THUMB
	bx	\reg
#else
	mov	pc, \reg
#endif
	.endm

	.macro	usr_reteq, reg
#ifdef CONFIG_ARM_THUMB
	bxeq	\reg
#else
	moveq	 pc, \reg
#endif
	.endm

	.macro	myldrd, rd1, rd2, rtmp, label
#if __LINUX_ARM_ARCH__ < 5
	adr	\rtmp, \label
	ldm	\rtmp, { \rd1, \rd2 }
#else
	ldrd	\rd1, \label
#endif
	.endm

/*
	We use the same mechanism as Linux user helpers to store
	variables and functions related to TSC emulation, so that they
	can also be used in user-space.

	The function ipipe_tsc_register will copy the proper
	implemntation to the vectors page. We repeat the data area so
	that the PC relative operations are computed correctly.
*/

	.section	.init.text, "ax", %progbits
 THUMB(	.arm	)

	.align 5
	.rep	7
	.word	0
	.endr
.LCfr64_cntr_addr:
	.word 	0

	.align 5
	.globl 	__ipipe_freerunning_64
__ipipe_freerunning_64:
	ldr	r0, .LCfr64_cntr_addr
/* User-space entry-point: r0 is the hardware counter virtual address */
	mov 	r2, r0
#ifndef CONFIG_CPU_BIG_ENDIAN
/* Little endian */
	ldr 	r1, [r2, #4]
1:	ldr	r0, [r2]
	ldr	r3, [r2, #4]
	cmp	r3, r1
	usr_reteq lr
	mov	r1, r3
	b	1b
#else /* Big endian */
	ldr 	r0, [r2]
1:	ldr	r1, [r2, #4]
	ldr	r3, [r2]
	cmp	r3, r0
	usr_reteq lr
	mov	r0, r3
	b	1b
#endif /* Big endian */

	.align 5
.LCfr32_last_tsc:
	.rep	7
	.word	0
	.endr
.LCfr32_cntr_addr:
	.word 	0

	.align 5
	.globl 	__ipipe_freerunning_32
__ipipe_freerunning_32:
	ldr	r0, .LCfr32_cntr_addr
/* User-space entry-point: r0 is the hardware counter virtual address */
	myldrd	r2, r3, r1, .LCfr32_last_tsc
#ifndef CONFIG_CPU_BIG_ENDIAN
/* Little endian */
	ldr	r0, [r0]
	cmp	r2, r0
	adc	r1, r3, #0
#else /* Big endian */
	ldr	r1, [r0]
	cmp	r3, r1
	adc	r0, r2, #0
#endif /* Big endian */
	usr_ret lr

	.align 5
.LCfrcd32_last_tsc:
	.rep	7
	.word	0
	.endr
.LCfrcd32_cntr_addr:
	.word 	0

	.align 5
	.globl __ipipe_freerunning_countdown_32
__ipipe_freerunning_countdown_32:
	ldr	r0, .LCfrcd32_cntr_addr
/* User-space entry-point: r0 is the hardware counter virtual address */
	myldrd	r2, r3, r1, .LCfrcd32_last_tsc
#ifndef CONFIG_CPU_BIG_ENDIAN
/* Little endian */
	ldr	r0, [r0]
	mvn	r0, r0
	cmp	r2, r0
	adc	r1, r3, #0
#else /* Big endian */
	ldr	r1, [r0]
	mvn	r1, r1
	cmp	r3, r1
	adc	r0, r2, #0
#endif /* Big endian */
	usr_ret lr

	.align 5
.LCfr16_last_tsc:
	.rep	7
	.word	0
	.endr
.LCfr16_cntr_addr:
	.word 	0

	.align 5
	.globl __ipipe_freerunning_16
__ipipe_freerunning_16:
	ldr	r0, .LCfr16_cntr_addr
/* User-space entry-point: r0 is the hardware counter virtual address */
1:	myldrd	r2, r3, r1, .LCfr16_last_tsc
	ldrh	ip, [r0]
#ifndef CONFIG_CPU_BIG_ENDIAN
/* Little endian */
	ldr	r1, .LCfr16_last_tsc
	cmp	r1, r2
	mov	r1, r2, lsr #16
	bne	1b
	orr	r0, ip, r1, lsl #16
	cmp	r2, r0
	addhis	r0, r0, #0x10000
	adc	r1, r3, #0
#else /* Big endian */
	ldr	r1, .LCfr16_last_tsc + 4
	cmp	r1, r3
	mov	r1, r3, lsr #16
	bne	1b
	orr	r1, ip, r1, lsl #16
	cmp	r3, r1
	addhis	r1, r1, #0x10000
	adc	r0, r2, #0
#endif /* Big endian */
	usr_ret lr

	.align 5
.LCfrcd16_last_tsc:
	.rep	7
	.word	0
	.endr
.LCfrcd16_cntr_addr:
	.word 	0

	.align 5
	.globl __ipipe_freerunning_countdown_16
__ipipe_freerunning_countdown_16:
	ldr	r0, .LCfrcd16_cntr_addr
/* User-space entry-point: r0 is the hardware counter virtual address */
1:	myldrd	r2, r3, r1, .LCfrcd16_last_tsc
	ldrh	ip, [r0]
#ifndef CONFIG_CPU_BIG_ENDIAN
/* Little endian */
	ldr	r1, .LCfrcd16_last_tsc
	rsb	ip, ip, #0x10000
	cmp	r1, r2
	mov	r1, r2, lsr #16
	bne	1b
	orr	r0, ip, r1, lsl #16
	cmp	r2, r0
	addhis	r0, r0, #0x10000
	adc	r1, r3, #0
#else /* Big endian */
	ldr	r1, .LCfrcd16_last_tsc + 4
	rsb	ip, ip, #0x10000
	cmp	r1, r3
	mov	r1, r3, lsr #16
	bne	1b
	orr	r1, ip, r1, lsl #16
	cmp	r3, r1
	addhis	r1, r1, #0x10000
	adc	r0, r2, #0
#endif /* Big endian */
	usr_ret lr

	.align 5
.LCfrt16_last_tsc:
	.rep	7
	.word	0
	.endr
.LCfrt16_cntr_addr:
	.word 	0

	.align 5
	.globl __ipipe_freerunning_twice_16
__ipipe_freerunning_twice_16:
	ldr	r0, .LCfrt16_cntr_addr
/* User-space entry-point: r0 is the hardware counter virtual address */
1:	myldrd	r2, r3, r1, .LCfrt16_last_tsc
2:	ldrh	ip, [r0]
	ldrh	r1, [r0]
	cmp	r1, ip
	bne	2b
#ifndef CONFIG_CPU_BIG_ENDIAN
/* Little endian */
	ldr	r1, .LCfrt16_last_tsc
	cmp	r1, r2
	mov	r1, r2, lsr #16
	bne	1b
	orr	r0, ip, r1, lsl #16
	cmp	r2, r0
	addhis	r0, r0, #0x10000
	adc	r1, r3, #0
#else /* Big endian */
	ldr	r1, .LCfrt16_last_tsc + 4
	cmp	r1, r3
	mov	r1, r3, lsr #16
	bne	1b
	orr	r1, ip, r1, lsl #16
	cmp	r3, r1
	addhis	r1, r1, #0x10000
	adc	r0, r2, #0
#endif /* Big endian */
	usr_ret lr

	.align 5
.LCdec16_last_tsc:
	.rep	2
	.word	0
	.endr
.LCdec16_last_cnt:
	.rep	5
	.word	0
	.endr
.LCdec16_cntr_addr:
	.word 	0

	.align 5
	.globl __ipipe_decrementer_16
__ipipe_decrementer_16:
	ldr	r0, .LCdec16_cntr_addr
/* User-space entry-point: r0 is the hardware counter virtual address */
#ifndef CONFIG_CPU_BIG_ENDIAN
/* Little endian */
1:	ldr	r1, .LCdec16_last_tsc
	ldrh	ip, [r0]
	ldr	r2, .LCdec16_last_cnt
	subs 	ip, r2, ip
	addcc	ip, ip, #0x10000
	myldrd	r2, r3, r3, .LCdec16_last_tsc
	cmp	r1, r2
	bne	1b
	adds	r0, ip, r2
	adc	r1, r3, #0
#else /* Big endian */
1:	ldr	r1, .LCdec16_last_tsc + 4
	ldrh	ip, [r0]
	ldr	r2, .LCdec16_last_cnt
	subs 	ip, r2, ip
	addcc	ip, ip, #0x10000
	myldrd	r2, r3, r3, .LCdec16_last_tsc
	cmp	r1, r3
	bne	1b
	adds	r1, ip, r3
	adc	r0, r2, #0
#endif /* Big endian */
	usr_ret	lr

	.align 5
	.globl __ipipe_freerunning_arch
__ipipe_freerunning_arch:
	nop
#ifdef CONFIG_ARM_ARCH_TIMER
	mrrc	p15, 0, r0, r1, c14
#else
	mov	r0, #0
	mov	r1, #0
#endif
	usr_ret	lr
