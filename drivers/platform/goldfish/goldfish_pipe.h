/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2018 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef GOLDFISH_PIPE_H
#define GOLDFISH_PIPE_H

#define DEVICE_NAME "goldfish_pipe"

struct goldfish_pipe_dev_base {
	/* the destructor, the pointer is set in init */
	int (*deinit)(void *pipe_dev, struct platform_device *pdev);
};

/* The entry point to the pipe v1 driver */
int goldfish_pipe_device_v1_init(struct platform_device *pdev,
				 void __iomem *base,
				 int irq);

/* The entry point to the pipe v2 driver */
int goldfish_pipe_device_v2_init(struct platform_device *pdev,
				 char __iomem *base,
				 int irq);

#endif /* GOLDFISH_PIPE_H */
