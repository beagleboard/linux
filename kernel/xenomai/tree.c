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
#include <cobalt/kernel/tree.h>

void xntree_cleanup(struct rb_root *t, void *cookie,
		void (*destroy)(void *cookie, struct xnid *id))
{
	struct rb_node *node, *next;

	node = rb_first(t);
	while (node) {
		next = rb_next(node);

		/* destroy is expected to remove the node from the rbtree */
		destroy(cookie, container_of(node, struct xnid, link));

		node = next;
	}
}

int xnid_enter(struct rb_root *t, struct xnid *xnid, xnkey_t key)
{
	struct rb_node **new = &t->rb_node, *parent = NULL;

	while (*new) {
		struct xnid *i = container_of(*new, struct xnid, link);

		parent = *new;
		if (key < i->key)
			new = &((*new)->rb_left);
		else if (key > i->key)
			new = &((*new)->rb_right);
		else
			return -EEXIST;
	}

	xnid->key = key;
	rb_link_node(&xnid->link, parent, new);
	rb_insert_color(&xnid->link, t);

	return 0;
}
