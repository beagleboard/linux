/*   -*- linux-c -*-
 *   include/asm-blackfin/ipipe_base.h
 *
 *   Copyright (C) 2007 Philippe Gerum.
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, Inc., 675 Mass Ave, Cambridge MA 02139,
 *   USA; either version 2 of the License, or (at your option) any later
 *   version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __ASM_BLACKFIN_IPIPE_BASE_H
#define __ASM_BLACKFIN_IPIPE_BASE_H

#ifdef CONFIG_IPIPE

#include <linux/bitops.h>
#include <asm/bitsperlong.h>
#include <asm/irq.h>

#define IPIPE_NR_XIRQS		NR_IRQS

/* Blackfin-specific, per-cpu pipeline status */
#define IPIPE_SYNCDEFER_FLAG	15
#define IPIPE_SYNCDEFER_MASK	(1L << IPIPE_SYNCDEFER_MASK)

/*
 * Blackfin traps -- i.e. exception vector numbers, we leave a gap
 * after VEC_ILL_RES.
 */
#define IPIPE_TRAP_MAYDAY	52	/* Internal recovery trap */
#define IPIPE_NR_FAULTS		53

#ifndef __ASSEMBLY__

extern unsigned long __ipipe_root_status;

void ipipe_stall_root(void);

unsigned long ipipe_test_and_stall_root(void);

unsigned long ipipe_test_root(void);

void __ipipe_lock_root(void);

void __ipipe_unlock_root(void);

int __ipipe_do_sync_check(void);
#define __ipipe_sync_check __ipipe_do_sync_check()

static inline unsigned long __ipipe_ffnz(unsigned long ul)
{
	return ffs(ul) - 1;
}

#define __ipipe_run_irqtail(irq)  /* Must be a macro */			\
	do {								\
		unsigned long __pending;				\
		CSYNC();						\
		__pending = bfin_read_IPEND();				\
		if (__pending & 0x8000) {				\
			__pending &= ~0x8010;				\
			if (__pending && (__pending & (__pending - 1)) == 0) \
				__ipipe_call_irqtail(__ipipe_irq_tail_hook); \
		}							\
	} while (0)

#define __ipipe_syscall_watched_p(p, sc)	\
	(ipipe_notifier_enabled_p(p) || (unsigned long)sc >= NR_syscalls)

#endif /* !__ASSEMBLY__ */

#endif /* CONFIG_IPIPE */

#endif /* !__ASM_BLACKFIN_IPIPE_BASE_H */
