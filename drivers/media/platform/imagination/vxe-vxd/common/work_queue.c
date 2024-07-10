// SPDX-License-Identifier: GPL-2.0
/*
 * Work Queue Handling for Linux
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Lakshmi Sankar <lakshmisankar-t@ti.com>
 *
 * Re-written for upstream
 *	Prashanth Kumar Amai <prashanth.ka@pathpartnertech.com>
 */

#include <linux/slab.h>
#include <linux/printk.h>
#include <linux/mutex.h>

#include "work_queue.h"

/* Defining and initilizing mutex
 */
DEFINE_MUTEX(mutex);

#define false 0
#define true 1

struct node {
	void **key;
	struct node *next;
};

struct node *work_head;
struct node *delayed_work_head;

void init_work(void **work_args, void *work_fn, uint8_t hwa_id)
{
	struct work_struct **work = (struct work_struct **)work_args;
	//create a link
	struct node *link = kmalloc(sizeof(*link), GFP_KERNEL);

	*work = kzalloc(sizeof(struct work_struct), GFP_KERNEL);
	if (!(*work)) {
		pr_err("Memory allocation failed for work_queue\n");
		return;
	}
	INIT_WORK(*work, work_fn);

	link->key = (void **)work;
	mutex_lock(&mutex);
	//point it to old first node
	link->next = work_head;

	//point first to new first node
	work_head = link;
	mutex_unlock(&mutex);
}

void init_delayed_work(void **work_args, void *work_fn, uint8_t hwa_id)
{
	struct delayed_work **work = (struct delayed_work **)work_args;
	//create a link
	struct node *link = kmalloc(sizeof(*link), GFP_KERNEL);

	*work = kzalloc(sizeof(struct delayed_work), GFP_KERNEL);
	if (!(*work)) {
		pr_err("Memory allocation failed for delayed_work_queue\n");
		return;
	}
	INIT_DELAYED_WORK(*work, work_fn);

	link->key = (void **)work;
	mutex_lock(&mutex);
	//point it to old first node
	link->next = delayed_work_head;

	//point first to new first node
	delayed_work_head = link;
	mutex_unlock(&mutex);
}

/**
 * get_work_buff - get_work_buff
 * @key: key value
 * @flag: flag
 */

void *get_work_buff(void *key, signed char flag)
{
	struct node *data = NULL;
	void *work_new = NULL;
	struct node *temp = NULL;
	struct node *previous = NULL;
	struct work_struct **work = NULL;

	//start from the first link
	mutex_lock(&mutex);
	temp = work_head;

	//if list is empty
	if (!work_head) {
		mutex_unlock(&mutex);
		return NULL;
	}

	work = ((struct work_struct **)(temp->key));
	//navigate through list
	while (*work != key) {
		//if it is last node
		if (!temp->next) {
			mutex_unlock(&mutex);
			return NULL;
		}
			//store reference to current link
			previous = temp;
			//move to next link
			temp = temp->next;
			work = ((struct work_struct **)(temp->key));
	}

	if (flag) {
		//found a match, update the link
		if (temp == work_head) {
			//change first to point to next link
			work_head = work_head->next;
		} else {
			//bypass the current link
			previous->next = temp->next;
		}
	}

	mutex_unlock(&mutex);
	//return temp;
	data = temp;
	if (data) {
		work_new = data->key;
		if (flag)
			kfree(data);
	}
	return work_new;
}

void *get_delayed_work_buff(void *key, signed char flag)
{
	struct node *data = NULL;
	void *dwork_new = NULL;
	struct node *temp = NULL;
	struct node *previous = NULL;
	struct delayed_work **dwork = NULL;

	if (flag) {
		/* This Condition is true when kernel module is removed */
		return delayed_work_head;
	}
	//start from the first link
	mutex_lock(&mutex);
	temp = delayed_work_head;

	//if list is empty
	if (!delayed_work_head) {
		mutex_unlock(&mutex);
		return NULL;
	}

	dwork = ((struct delayed_work **)(temp->key));
	//navigate through list
	while (&(*dwork)->work != key) {
		//if it is last node
		if (!temp->next) {
			mutex_unlock(&mutex);
			return NULL;
		}
			//store reference to current link
			previous = temp;
			//move to next link
			temp = temp->next;
			dwork = ((struct delayed_work **)(temp->key));
	}

	mutex_unlock(&mutex);
	data = temp;
	if (data) {
		dwork_new = data->key;
		if (flag)
			kfree(data);
	}
	return dwork_new;
}
