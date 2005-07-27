/*
 * linux/arch/arm/mach-omap/dsp/proclist.h
 *
 * Linux task list handler
 *
 * Copyright (C) 2004,2005 Nokia Corporation
 *
 * Written by Toshihiro Kobayashi <toshihiro.kobayashi@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * 2004/11/22:  DSP Gateway version 3.3
 */

struct proc_list {
	struct list_head list_head;
	struct task_struct *tsk;
	unsigned int cnt;
};

static __inline__ void proc_list_add(struct list_head *list,
				     struct task_struct *tsk)
{
	struct list_head *ptr;
	struct proc_list *pl;
	struct proc_list *new;

	list_for_each(ptr, list) {
		pl = list_entry(ptr, struct proc_list, list_head);
		if (pl->tsk == tsk) {
			/*
			 * this process has opened DSP devices multi time
			 */
			pl->cnt++;
			return;
		}
	}

	new = kmalloc(sizeof(struct proc_list), GFP_KERNEL);
	new->tsk = tsk;
	new->cnt = 1;
	list_add_tail(&new->list_head, list);
}

static __inline__ void proc_list_del(struct list_head *list,
				     struct task_struct *tsk)
{
	struct list_head *ptr;
	struct proc_list *pl;

	list_for_each(ptr, list) {
		pl = list_entry(ptr, struct proc_list, list_head);
		if (pl->tsk == tsk) {
			if (--pl->cnt == 0) {
				list_del(&pl->list_head);
				kfree(pl);
			}
			return;
		}
	}
}

static __inline__ void proc_list_flush(struct list_head *list)
{
	struct proc_list *pl;

	while (!list_empty(list)) {
		pl = list_entry(list->next, struct proc_list, list_head);
		list_del(&pl->list_head);
		kfree(pl);
	}
}
