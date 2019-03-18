#ifndef __ARM_MMU_H
#define __ARM_MMU_H

#ifdef CONFIG_MMU

typedef struct {
#ifdef CONFIG_CPU_HAS_ASID
	atomic64_t	id;
#else
	int		switch_pending;
#endif
#ifdef CONFIG_ARM_FCSE
	struct {
		unsigned long pid;
#ifdef CONFIG_ARM_FCSE_BEST_EFFORT
		unsigned shared_dirty_pages;
		unsigned large : 1;
		unsigned high_pages;
		unsigned long highest_pid;
#endif /* CONFIG_ARM_FCSE_BEST_EFFORT */
	} fcse;
#endif /* CONFIG_ARM_FCSE */
	unsigned int	vmalloc_seq;
	unsigned long	sigpage;
#ifdef CONFIG_VDSO
	unsigned long	vdso;
#endif
} mm_context_t;

#ifdef CONFIG_CPU_HAS_ASID
#define ASID_BITS	8
#define ASID_MASK	((~0ULL) << ASID_BITS)
#define ASID(mm)	((unsigned int)((mm)->context.id.counter & ~ASID_MASK))
#else
#define ASID(mm)	(0)
#endif

#else

/*
 * From nommu.h:
 *  Copyright (C) 2002, David McCullough <davidm@snapgear.com>
 *  modified for 2.6 by Hyok S. Choi <hyok.choi@samsung.com>
 */
typedef struct {
	unsigned long	end_brk;
} mm_context_t;

#endif

#endif
