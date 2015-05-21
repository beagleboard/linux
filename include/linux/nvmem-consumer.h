/*
 * nvmem framework consumer.
 *
 * Copyright (C) 2015 Srinivas Kandagatla <srinivas.kandagatla@linaro.org>
 * Copyright (C) 2013 Maxime Ripard <maxime.ripard@free-electrons.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef _LINUX_NVMEM_CONSUMER_H
#define _LINUX_NVMEM_CONSUMER_H

/* consumer cookie */
struct nvmem_cell;
struct nvmem_device;

struct nvmem_cell_info {
	const char		*name;
	int			offset;
	int			bytes;
	int			bit_offset;
	int			nbits;
};

#if IS_ENABLED(CONFIG_NVMEM)

/* Cell based interface */
struct nvmem_cell *nvmem_cell_get(struct device *dev, const char *name);
void nvmem_cell_put(struct nvmem_cell *cell);
void *nvmem_cell_read(struct nvmem_cell *cell, ssize_t *len);
int nvmem_cell_write(struct nvmem_cell *cell, void *buf, ssize_t len);

/* direct nvmem device read/write interface */
struct nvmem_device *nvmem_device_get(struct device *dev, const char *name);
void nvmem_device_put(struct nvmem_device *nvmem);
int nvmem_device_read(struct nvmem_device *nvmem, unsigned int offset,
		      size_t bytes, void *buf);
int nvmem_device_write(struct nvmem_device *nvmem, unsigned int offset,
		       size_t bytes, void *buf);
int nvmem_device_cell_read(struct nvmem_device *nvmem,
			   struct nvmem_cell_info *info, void *buf);
int nvmem_device_cell_write(struct nvmem_device *nvmem,
			    struct nvmem_cell_info *info, void *buf);

#else

struct nvmem_cell *nvmem_cell_get(struct device *dev, const char *name)
{
	return ERR_PTR(-ENOSYS);
}

static inline void nvmem_cell_put(struct nvmem_cell *cell)
{
}

static inline char *nvmem_cell_read(struct nvmem_cell *cell, ssize_t *len)
{
	return ERR_PTR(-ENOSYS);
}

static inline int nvmem_cell_write(struct nvmem_cell *cell,
				    const char *buf, ssize_t len)
{
	return -ENOSYS;
}

static inline struct nvmem_device *nvmem_device_get(struct device *dev,
						    const char *name)
{
	return ERR_PTR(-ENOSYS);
}

static inline void nvmem_device_put(struct nvmem_device *nvmem)
{
}

static inline int nvmem_device_cell_read(struct nvmem_device *nvmem,
					 struct nvmem_cell_info *info,
					 void *buf)
{
	return -ENOSYS;
}

static inline int nvmem_device_cell_write(struct nvmem_device *nvmem,
					  struct nvmem_cell_info *info,
					  void *buf)
{
	return -ENOSYS;
}

static inline int nvmem_device_read(struct nvmem_device *nvmem,
				    unsigned int offset, size_t bytes, void *buf)
{
	return -ENOSYS;
}

static inline int nvmem_device_write(struct nvmem_device *nvmem,
				     unsigned int offset, size_t bytes, void *buf)
{
	return -ENOSYS;
}
#endif /* CONFIG_NVMEM */

#endif  /* ifndef _LINUX_NVMEM_CONSUMER_H */
