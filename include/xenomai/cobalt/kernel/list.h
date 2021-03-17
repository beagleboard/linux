/*
 * Copyright (C) 2013 Philippe Gerum <rpm@xenomai.org>.
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
#ifndef _COBALT_KERNEL_LIST_H
#define _COBALT_KERNEL_LIST_H

#include <linux/list.h>

#define __list_add_pri(__new, __head, __member_pri, __member_next, __relop)	\
do {										\
	typeof(*__new) *__pos;							\
	if (list_empty(__head))							\
		list_add(&(__new)->__member_next, __head);		 	\
	else {									\
		list_for_each_entry_reverse(__pos, __head, __member_next) {	\
			if ((__new)->__member_pri __relop __pos->__member_pri)	\
				break;						\
		}								\
		list_add(&(__new)->__member_next, &__pos->__member_next); 	\
	}									\
} while (0)

#define list_add_priff(__new, __head, __member_pri, __member_next)		\
	__list_add_pri(__new, __head, __member_pri, __member_next, <=)

#define list_add_prilf(__new, __head, __member_pri, __member_next)		\
	__list_add_pri(__new, __head, __member_pri, __member_next, <)

#define list_get_entry(__head, __type, __member)		\
  ({								\
	  __type *__item;					\
	  __item = list_first_entry(__head, __type, __member);	\
	  list_del(&__item->__member);				\
	  __item;						\
  })

#define list_get_entry_init(__head, __type, __member)		\
  ({								\
	  __type *__item;					\
	  __item = list_first_entry(__head, __type, __member);	\
	  list_del_init(&__item->__member);			\
	  __item;						\
  })

#ifndef list_next_entry
#define list_next_entry(__item, __member)			\
	list_entry((__item)->__member.next, typeof(*(__item)), __member)
#endif

#endif /* !_COBALT_KERNEL_LIST_H_ */
