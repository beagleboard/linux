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
	pid_t pid;
	unsigned int cnt;
};

static __inline__ void proc_list_add(struct list_head *list,
				     struct task_struct *tsk)
{
	struct proc_list *pl;
	struct proc_list *new;

	list_for_each_entry(pl, list, list_head) {
		if (pl->pid == tsk->pid) {
			/*
			 * this process has opened DSP devices multi time
			 */
			pl->cnt++;
			return;
		}
	}

	new = kmalloc(sizeof(struct proc_list), GFP_KERNEL);
	new->pid = tsk->pid;
	new->cnt = 1;
	list_add_tail(&new->list_head, list);
}

static __inline__ void proc_list_del(struct list_head *list,
				     struct task_struct *tsk)
{
	struct proc_list *pl, *next;

	list_for_each_entry(pl, list, list_head) {
		if (pl->pid == tsk->pid) {
			if (--pl->cnt == 0) {
				list_del(&pl->list_head);
				kfree(pl);
			}
			return;
		}
	}

	/*
	 * correspinding pid wasn't found in the list
	 * -- this means the caller of proc_list_del is different from
	 * the proc_list_add's caller. in this case, the parent is
	 * cleaning up the context of a killed child.
	 * let's delete exiting task from the list.
	 */
	/* need to lock tasklist_lock before calling find_task_by_pid_type. */
	read_lock(&tasklist_lock);
	list_for_each_entry_safe(pl, next, list, list_head) {
		if (find_task_by_pid_type(PIDTYPE_PID, pl->pid) == NULL) {
			list_del(&pl->list_head);
			kfree(pl);
		}
	}
	read_unlock(&tasklist_lock);
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
