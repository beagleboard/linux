/*
 * This file is part of the Xenomai project.
 *
 * Copyright (C) 2014 Philippe Gerum <rpm@xenomai.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#ifndef _COBALT_POSIX_MEMORY_H
#define _COBALT_POSIX_MEMORY_H

#include <cobalt/kernel/ppd.h>

#define cobalt_umm_set_name(__umm, __fmt, __args...)	\
	xnheap_set_name(&(__umm)->heap, (__fmt), ## __args)

static inline
void *cobalt_umm_alloc(struct cobalt_umm *umm, __u32 size)
{
	return xnheap_alloc(&umm->heap, size);
}

static inline
void *cobalt_umm_zalloc(struct cobalt_umm *umm, __u32 size)
{
	return xnheap_zalloc(&umm->heap, size);
}

static inline
void cobalt_umm_free(struct cobalt_umm *umm, void *p)
{
	xnheap_free(&umm->heap, p);
}

static inline
__u32 cobalt_umm_offset(struct cobalt_umm *umm, void *p)
{
	return p - xnheap_get_membase(&umm->heap);
}

int cobalt_memdev_init(void);

void cobalt_memdev_cleanup(void);

int cobalt_umm_init(struct cobalt_umm *umm, u32 size,
		    void (*release)(struct cobalt_umm *umm));

void cobalt_umm_destroy(struct cobalt_umm *umm);

#endif /* !_COBALT_POSIX_MEMORY_H */
