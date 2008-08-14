/*
 * This file is part of OMAP DSP driver (DSP Gateway version 3.3.1)
 *
 * Copyright (C) 2004-2006 Nokia Corporation. All rights reserved.
 *
 * Contact: Toshihiro Kobayashi <toshihiro.kobayashi@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef ASM_ARCH_DSP_COMMON_H
#define ASM_ARCH_DSP_COMMON_H

#include <linux/clk.h>

struct dsp_kfunc_device {
	char		*name;
	struct clk	*fck;
	struct clk	*ick;;
	spinlock_t	 lock;
	int		 enabled;
	int		 type;
#define DSP_KFUNC_DEV_TYPE_COMMON	1
#define DSP_KFUNC_DEV_TYPE_AUDIO	2

	struct list_head	entry;

	int	(*probe)(struct dsp_kfunc_device *, int);
	int	(*remove)(struct dsp_kfunc_device *, int);
	int	(*enable)(struct dsp_kfunc_device *, int);
	int	(*disable)(struct dsp_kfunc_device *, int);
};

extern int dsp_kfunc_device_register(struct dsp_kfunc_device *);

struct dsp_platform_data {
	struct list_head kdev_list;
};

struct omap_dsp {
	struct mutex		lock;
	int			enabled;	/* stored peripheral status */
	struct omap_mmu		*mmu;
	struct omap_mbox	*mbox;
	struct device		*dev;
	struct list_head	*kdev_list;
	int			initialized;
};

#if defined(CONFIG_ARCH_OMAP1) && defined(CONFIG_OMAP_MMU_FWK)
extern void omap_dsp_request_mpui(void);
extern void omap_dsp_release_mpui(void);
extern int omap_dsp_request_mem(void);
extern int omap_dsp_release_mem(void);
#else
static inline int omap_dsp_request_mem(void)
{
	return 0;
}
#define omap_dsp_release_mem()	do {} while (0)
#endif

#endif /* ASM_ARCH_DSP_COMMON_H */
