/* Generic task switch macro wrapper.
 *
 * It should be possible to use these on really simple architectures,
 * but it serves more as a starting point for new ports.
 *
 * Copyright (C) 2007 Red Hat, Inc. All Rights Reserved.
 * Written by David Howells (dhowells@redhat.com)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public Licence
 * as published by the Free Software Foundation; either version
 * 2 of the Licence, or (at your option) any later version.
 */
#ifndef __ASM_GENERIC_SWITCH_TO_H
#define __ASM_GENERIC_SWITCH_TO_H

#include <linux/thread_info.h>

/*
 * Context switching is now performed out-of-line in switch_to.S
 */
extern struct task_struct *__switch_to(struct task_struct *,
				       struct task_struct *);
#ifdef CONFIG_IPIPE_WANT_PREEMPTIBLE_SWITCH
#define switch_to(prev, next, last)					\
	do {								\
		hard_cond_local_irq_disable();                                  \
		((last) = __switch_to((prev), (next)));			\
		hard_cond_local_irq_enable();                                   \
	} while (0)
#else /* !CONFIG_IPIPE_WANT_PREEMPTIBLE_SWITCH */
#define switch_to(prev, next, last)					\
	do {								\
		((last) = __switch_to((prev), (next)));			\
	} while (0)
#endif /* !CONFIG_IPIPE_WANT_PREEMPTIBLE_SWITCH */
#endif /* __ASM_GENERIC_SWITCH_TO_H */
