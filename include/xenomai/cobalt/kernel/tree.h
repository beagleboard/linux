/*
 * Copyright (C) 2014 Gilles Chanteperdrix <gilles.chanteperdrix@xenomai.org>.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#ifndef _COBALT_KERNEL_TREE_H
#define _COBALT_KERNEL_TREE_H

#include <linux/errno.h>
#include <linux/rbtree.h>
#include <cobalt/kernel/assert.h>

typedef unsigned long long xnkey_t;

static inline xnkey_t PTR_KEY(void *p)
{
	return (xnkey_t)(long)p;
}

struct xnid {
	xnkey_t key;
	struct rb_node link;
};

#define xnid_entry(ptr, type, member)					\
	({								\
		typeof(ptr) _ptr = (ptr);				\
		(_ptr ? container_of(_ptr, type, member.link) : NULL);	\
	})

#define xnid_next_entry(ptr, member)				\
	xnid_entry(rb_next(&ptr->member.link), typeof(*ptr), member)

static inline void xntree_init(struct rb_root *t)
{
	*t = RB_ROOT;
}

#define xntree_for_each_entry(pos, root, member)			\
	for (pos = xnid_entry(rb_first(root), typeof(*pos), member);	\
	     pos; pos = xnid_next_entry(pos, member))

void xntree_cleanup(struct rb_root *t, void *cookie,
		void (*destroy)(void *cookie, struct xnid *id));

int xnid_enter(struct rb_root *t, struct xnid *xnid, xnkey_t key);

static inline xnkey_t xnid_key(struct xnid *i)
{
	return i->key;
}

static inline
struct xnid *xnid_fetch(struct rb_root *t, xnkey_t key)
{
	struct rb_node *node = t->rb_node;

	while (node) {
		struct xnid *i = container_of(node, struct xnid, link);

		if (key < i->key)
			node = node->rb_left;
		else if (key > i->key)
			node = node->rb_right;
		else
			return i;
	}

	return NULL;
}

static inline int xnid_remove(struct rb_root *t, struct xnid *xnid)
{
#ifdef CONFIG_XENO_OPT_DEBUG_COBALT
	if (xnid_fetch(t, xnid->key) != xnid)
		return -ENOENT;
#endif
	rb_erase(&xnid->link, t);
	return 0;
}

#endif /* _COBALT_KERNEL_TREE_H */
