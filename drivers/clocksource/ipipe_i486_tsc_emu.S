#include <linux/linkage.h>

#define PIT_MODE	0x43
#define PIT_CH2		0x42
#define PIT_COUNT2LATCH 0xfffe

.macro SAVE reg
	pushl %\reg
.endm

.macro RESTORE reg
	popl %\reg
.endm

ENTRY(__ipipe_get_8253_tsc)
	mov	$0xd8, %al
	out	%al, $(PIT_MODE)
	in	$(PIT_CH2), %al
	xor	%ecx, %ecx
	mov	%al, %cl
	in	$(PIT_CH2), %al
	mov	%al, %ch

	mov	__ipipe_last_8253_counter2, %eax
	mov	__ipipe_cs_last_tsc + 4, %edx
	sub	%ecx, %eax
	mov	%ecx, __ipipe_last_8253_counter2
	test	%eax, %eax
	mov	__ipipe_cs_last_tsc, %ecx
	jg	1f
	add	$(PIT_COUNT2LATCH), %eax
1:	add	%ecx, %eax
	adc	$0, %edx
	mov	%eax, __ipipe_cs_last_tsc
	mov	%edx, __ipipe_cs_last_tsc + 4

	ret

ENDPROC(__ipipe_get_8253_tsc)

ENTRY(__ipipe_get_cs_tsc)
	SAVE	ecx

	pushfl
	cli

	mov	__ipipe_cs_mask + 4, %ecx
	mov	__ipipe_cs_mask, %edx
	cmp	$0xffffffff, %ecx
	mov	__ipipe_cs, %eax
	jne	1f

	/* 64 bits clocksource */
	call	*__ipipe_cs_read
	jmp	4f

1:	cmp	$0xffffffff, %edx
	jne	2f

	/* 32 bits clocksource */
	call 	*__ipipe_cs_read

	mov	__ipipe_cs_last_tsc + 4, %edx
	cmp	__ipipe_cs_last_tsc, %eax
	adc	$0, %edx

	jmp	4f

	/* n bits (< 32) clocksource */
2:	SAVE	ebx

	mov	%edx, %ebx
	call 	*__ipipe_cs_read

	mov	__ipipe_cs_last_tsc, %ecx
	and	%ebx, %eax
	mov	%ebx, %edx
	and	%ecx, %ebx
	not 	%edx
	cmp	%ebx, %eax
	jae	3f
	sub	%edx, %eax
3:	and	%edx, %ecx
	mov	__ipipe_cs_last_tsc + 4, %edx
	add	%ecx, %eax
	adc	$0, %edx

	RESTORE	ebx

4:	mov	%eax, __ipipe_cs_last_tsc
	mov	%edx, __ipipe_cs_last_tsc + 4
	popfl
	RESTORE	ecx
	ret

	/* n bits clocksource with 32 < n < 64, not supported. */
ENDPROC(__ipipe_get_cs_tsc)
