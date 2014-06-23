/*
 * Include file for the BeagleLogic kernel module [glue code for pru_rproc
 *
 * Copyright (C) 2014 Kumar Abhishek <abhishek@theembeddedkitchen.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#ifndef BEAGLELOGIC_GLUE_H_
#define BEAGLELOGIC_GLUE_H_

/* IRQ requests handled by pru_rproc are rerouted to beaglelogic
 * after a rename.
 *
 * Currently only two interrupts, buffer ready and cleanup after stop */
#define BL_IRQ_BUFREADY	0
#define BL_IRQ_CLEANUP	1

struct beaglelogic_glue {
	/* Misc device descriptor */
	struct miscdevice miscdev;

	/* Imported functions */
	int (*downcall_idx)(int, u32, u32, u32, u32, u32, u32);
	void __iomem *(*d_da_to_va)(int, u32);
	int (*pru_start)(int);
	void (*pru_request_stop)(void);

	/* Exported functions */
	int (*serve_irq)(int, void *);

	/* Core clock frequency: Required for configuring sample rates */
	u32 coreclockfreq;
};

/* Bind and unbind requests */
extern int pruproc_beaglelogic_request_bind(struct beaglelogic_glue *g);
extern void pruproc_beaglelogic_request_unbind(void);

#endif /* BEAGLELOGIC_KERNEL_H_ */
