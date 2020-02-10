/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * PRU-ICSS sub-system private interfaces
 *
 * Copyright (C) 2019-2020 Texas Instruments Incorporated - http://www.ti.com/
 *	Suman Anna <s-anna@ti.com>
 */

#ifndef __LINUX_IRQ_PRUSS_INTC_H
#define __LINUX_IRQ_PRUSS_INTC_H

#include <linux/types.h>
#include <linux/errno.h>

/* maximum number of system events */
#define MAX_PRU_SYS_EVENTS	160

/* maximum number of interrupt channels */
#define MAX_PRU_CHANNELS	20

/* use -1 to mark unassigned events and channels */
#define PRU_INTC_FREE		-1

struct device;

/**
 * struct pruss_intc_config - INTC configuration info
 * @sysev_to_ch: system events to channel mapping information
 * @ch_to_host: interrupt channel to host interrupt information
 */
struct pruss_intc_config {
	s8 sysev_to_ch[MAX_PRU_SYS_EVENTS];
	s8 ch_to_host[MAX_PRU_CHANNELS];
};

#if IS_ENABLED(CONFIG_TI_PRUSS_INTC)

int pruss_intc_configure(struct device *dev,
			 struct pruss_intc_config *intc_config);
int pruss_intc_unconfigure(struct device *dev,
			   struct pruss_intc_config *intc_config);

#else

static inline int pruss_intc_configure(struct device *dev,
				       struct pruss_intc_config *intc_config)
{
	return -ENOTSUPP;
}

static inline int pruss_intc_unconfigure(struct device *dev,
					 struct pruss_intc_config *intc_config)
{
	return -ENOTSUPP;
}

#endif /* CONFIG_TI_PRUSS_INTC */

#endif	/* __LINUX_IRQ_PRUSS_INTC_H */
