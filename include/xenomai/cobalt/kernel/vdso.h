/*
 * Copyright (C) 2009 Wolfgang Mauerer <wolfgang.mauerer@siemens.com>.
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
#ifndef _COBALT_KERNEL_VDSO_H
#define _COBALT_KERNEL_VDSO_H

#include <linux/time.h>
#include <asm/barrier.h>
#include <asm/atomic.h>
#include <asm/processor.h>
#include <cobalt/uapi/kernel/vdso.h>

/*
 * Define the available feature set here. We have a single feature
 * defined for now.
 */
#ifdef CONFIG_XENO_OPT_HOSTRT
#define XNVDSO_FEATURES XNVDSO_FEAT_HOST_REALTIME
#else
#define XNVDSO_FEATURES 0
#endif /* CONFIG_XENO_OPT_HOSTRT */

extern struct xnvdso *nkvdso;

static inline struct xnvdso_hostrt_data *get_hostrt_data(void)
{
	return &nkvdso->hostrt_data;
}

#endif /* _COBALT_KERNEL_VDSO_H */
