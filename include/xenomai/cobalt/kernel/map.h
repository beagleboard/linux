/*
 * Copyright (C) 2007 Philippe Gerum <rpm@xenomai.org>.
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
#ifndef _COBALT_KERNEL_MAP_H
#define _COBALT_KERNEL_MAP_H

#include <asm/bitsperlong.h>

/**
 * @addtogroup cobalt_core_map
 * @{
 */

#define XNMAP_MAX_KEYS	(BITS_PER_LONG * BITS_PER_LONG)

struct xnmap {
    int nkeys;
    int ukeys;
    int offset;
    unsigned long himask;
    unsigned long himap;
#define __IDMAP_LONGS	((XNMAP_MAX_KEYS+BITS_PER_LONG-1)/BITS_PER_LONG)
    unsigned long lomap[__IDMAP_LONGS];
#undef __IDMAP_LONGS
    void *objarray[1];
};

struct xnmap *xnmap_create(int nkeys,
			   int reserve,
			   int offset);

void xnmap_delete(struct xnmap *map);

int xnmap_enter(struct xnmap *map,
		int key,
		void *objaddr);

int xnmap_remove(struct xnmap *map,
		 int key);

static inline void *xnmap_fetch_nocheck(struct xnmap *map, int key)
{
	int ofkey = key - map->offset;
	return map->objarray[ofkey];
}

static inline void *xnmap_fetch(struct xnmap *map, int key)
{
	int ofkey = key - map->offset;

	if (ofkey < 0 || ofkey >= map->nkeys)
		return NULL;

	return map->objarray[ofkey];
}

/** @} */

#endif /* !_COBALT_KERNEL_MAP_H */
