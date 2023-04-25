/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Work Queue Related Definitions
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

#ifndef WORKQUEUE_H_
#define WORKQUEUE_H_

#include <linux/types.h>

enum {
	HWA_DECODER   = 0,
	HWA_ENCODER    = 1,
	HWA_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * init_work - This function provides the necessary initialization
 * and saving given pointer(work_args) in linked list.
 * @work_args: structure for the initialization
 * @work_fn: work function pointer
 *
 * This function provides the necessary initialization
 * and setting of the handler function (passed by the user).
 */
void init_work(void **work_args, void *work_fn, uint8_t hwa_id);

/*
 * init_delayed_work - This function provides the necessary initialization.
 * and saving given pointer(work_args) in linked list.
 * @work_args: structure for the initialization
 * @work_fn: work function pointer
 *
 * This function provides the necessary initialization
 * and setting of the handler function (passed by the user).
 */
void init_delayed_work(void **work_args, void *work_fn, uint8_t hwa_id);

/*
 * get_delayed_work_buff - This function return base address of given pointer
 * @key: The given work struct pointer
 * @flag: If TRUE, delete the node from the linked list.
 *
 * Return: Base address of the given input buffer.
 */
void *get_delayed_work_buff(void *key, signed char flag);

/**
 * get_work_buff - This function return base address of given pointer
 * @key: The given work struct pointer
 * @flag: If TRUE, delete the node from the linked list.
 *
 * Return: Base address of the given input buffer.
 */
void *get_work_buff(void *key, signed char flag);

#endif /* WORKQUEUE_H_ */
