/* SPDX-License-Identifier: GPL-2.0 */
/*
 * CPPI5 descriptors pool
 *
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com
 */

#ifndef K3_NAVSS_DESC_POOL_H_
#define K3_NAVSS_DESC_POOL_H_

struct k3_knav_desc_pool;

void k3_knav_pool_destroy(struct k3_knav_desc_pool *pool);
struct k3_knav_desc_pool *k3_knav_pool_create_name(struct device *dev,
						   size_t size,
						   size_t desc_size,
						   const char *name);
#define k3_knav_pool_create(dev, size, desc_size) \
		k3_knav_pool_create_name(dev, size, desc_size, NULL)
dma_addr_t k3_knav_pool_virt2dma(struct k3_knav_desc_pool *pool, void *addr);
void *k3_knav_pool_dma2virt(struct k3_knav_desc_pool *pool, dma_addr_t dma);
void *k3_knav_pool_alloc(struct k3_knav_desc_pool *pool);
void k3_knav_pool_free(struct k3_knav_desc_pool *pool, void *addr);
size_t k3_knav_pool_avail(struct k3_knav_desc_pool *pool);

#endif /* K3_NAVSS_DESC_POOL_H_ */
