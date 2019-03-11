/*
 * omap iommu: simple virtual address space management
 *
 * Copyright (C) 2008-2009 Nokia Corporation
 *
 * Written by Hiroshi DOYU <Hiroshi.DOYU@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _OMAP_IOMMU_H_
#define _OMAP_IOMMU_H_

#include <linux/errno.h>

struct iommu_domain;

#ifdef CONFIG_OMAP_IOMMU
extern void omap_iommu_save_ctx(struct device *dev);
extern void omap_iommu_restore_ctx(struct device *dev);

int omap_iommu_domain_deactivate(struct iommu_domain *domain);
int omap_iommu_domain_activate(struct iommu_domain *domain);
#else
static inline void omap_iommu_save_ctx(struct device *dev) {}
static inline void omap_iommu_restore_ctx(struct device *dev) {}

static inline int omap_iommu_domain_deactivate(struct iommu_domain *domain)
{
	return -ENOTSUPP;
}

static inline int omap_iommu_domain_activate(struct iommu_domain *domain)
{
	return -ENOTSUPP;
}
#endif

#endif
