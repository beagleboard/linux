/*
 * MMIO based nvmem providers.
 *
 * Copyright (C) 2015 Srinivas Kandagatla <srinivas.kandagatla@linaro.org>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef _LINUX_NVMEM_MMIO_H
#define _LINUX_NVMEM_MMIO_H

#include <linux/platform_device.h>
#include <linux/nvmem-provider.h>
#include <linux/regmap.h>

struct nvmem_mmio_data {
	struct regmap_config *regmap_config;
	struct nvmem_config *nvmem_config;
};

#if IS_ENABLED(CONFIG_NVMEM)

int nvmem_mmio_probe(struct platform_device *pdev);
int nvmem_mmio_remove(struct platform_device *pdev);

#else

static inline int nvmem_mmio_probe(struct platform_device *pdev)
{
	return -ENOSYS;
}

static inline int nvmem_mmio_remove(struct platform_device *pdev)
{
	return -ENOSYS;
}
#endif

#endif  /* ifndef _LINUX_NVMEM_MMIO_H */
