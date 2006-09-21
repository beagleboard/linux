/*
 * This file is part of OMAP DSP driver (DSP Gateway version 3.3.1)
 *
 * Copyright (C) 2004-2006 Nokia Corporation. All rights reserved.
 *
 * Contact: Toshihiro Kobayashi <toshihiro.kobayashi@nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

struct proc_list {
	struct list_head list_head;
	pid_t pid;
	struct file *file;
};

static __inline__ void proc_list_add(spinlock_t *lock, struct list_head *list,
				     struct task_struct *tsk, struct file *file)
{
	struct proc_list *new;

	new = kmalloc(sizeof(struct proc_list), GFP_KERNEL);
	new->pid = tsk->pid;
	new->file = file;
	spin_lock(lock);
	list_add_tail(&new->list_head, list);
	spin_unlock(lock);
}

static __inline__ void proc_list_del(spinlock_t *lock, struct list_head *list,
				     struct task_struct *tsk, struct file *file)
{
	struct proc_list *pl;

	spin_lock(lock);
	list_for_each_entry(pl, list, list_head) {
		if (pl->file == file) {
			list_del(&pl->list_head);
			kfree(pl);
			spin_unlock(lock);
			return;
		}
	}

	/* correspinding file struct isn't found in the list ???  */
	printk(KERN_ERR "proc_list_del(): proc_list is inconsistent!\n"
			"struct file (%p) not found\n", file);
	printk(KERN_ERR "listing proc_list...\n");
	list_for_each_entry(pl, list, list_head)
		printk(KERN_ERR "  pid:%d file:%p\n", pl->pid, pl->file);
	spin_unlock(lock);
}

static __inline__ void proc_list_flush(spinlock_t *lock, struct list_head *list)
{
	struct proc_list *pl;

	spin_lock(lock);
	while (!list_empty(list)) {
		pl = list_entry(list->next, struct proc_list, list_head);
		list_del(&pl->list_head);
		kfree(pl);
	}
	spin_unlock(lock);
}
