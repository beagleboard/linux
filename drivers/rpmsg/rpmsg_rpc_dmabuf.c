// SPDX-License-Identifier: GPL-2.0-only
/*
 * Remote Processor Procedure Call Driver
 *
 * Copyright (C) 2012-2020 Texas Instruments Incorporated - http://www.ti.com/
 *	Erik Rainey <erik.rainey@ti.com>
 *	Suman Anna <s-anna@ti.com>
 */

#include <linux/dma-buf.h>
#include <linux/rpmsg_rpc.h>

#include "rpmsg_rpc_internal.h"

#if defined(CONFIG_ARCH_OMAP4) || defined(CONFIG_SOC_OMAP5) || \
	defined(CONFIG_SOC_DRA7XX)
/*
 * TODO: Remove tiler_stride_from_region & rppc_recalc_off from here, and
 *	 rely on OMAPDRM/TILER code for OMAP dependencies
 */

/**
 * tiler_stride_from_region() - calculate stride value for OMAP TILER
 * @localphys:	The local physical address.
 *
 * Returns the stride value as seen by remote processors based on the local
 * address given to the function. This stride value is calculated based on the
 * actual bus address, and is assumed that the TILER regions are mapped in a
 * in a linear fashion.
 *
 * The physical address range decoding of local addresses is as follows:
 *
 * 0x60000000 - 0x67FFFFFF : 8-bit region (Stride is 16K bytes)
 * 0x68000000 - 0x6FFFFFFF : 16-bit region (Stride is 32K bytes)
 * 0x70000000 - 0x77FFFFFF : 32-bit region (Stride is 32K bytes)
 * 0x78000000 - 0x7FFFFFFF : Page mode region (Stride is 0 bytes)
 *
 * Return: stride value
 */
static long tiler_stride_from_region(phys_addr_t localphys)
{
	switch (localphys & 0xf8000000) {
	case 0x60000000:
		return 0x4000;
	case 0x68000000:
	case 0x70000000:
		return 0x8000;
	default:
		return 0;
	}
}

/**
 * rppc_recalc_off() - Recalculate the unsigned offset in a buffer due to
 *		       it's location in the TILER.
 * @lpa:	local physical address
 * @uoff:	unsigned offset
 *
 * Return: adjusted offset accounting for TILER region
 */
static long rppc_recalc_off(phys_addr_t lpa, long uoff)
{
	long stride = tiler_stride_from_region(lpa);

	return (stride != 0) ? (stride * (uoff / PAGE_SIZE)) +
				(uoff & (PAGE_SIZE - 1)) : uoff;
}
#else
static inline long rppc_recalc_off(phys_addr_t lpa, long uoff)
{
	return uoff;
}
#endif

/**
 * rppc_alloc_dmabuf - import a buffer and store in a rppc buffer descriptor
 * @rpc - rppc instance handle
 * @fd - dma_buf file descriptor
 * @autoreg: flag indicating the mode of creation
 *
 * This function primarily imports a buffer into the driver and holds
 * a reference to the buffer on behalf of the remote processor. The
 * buffer to be imported is represented by a dma-buf file descriptor,
 * and as such is agnostic of the buffer allocator and/or exporter.
 * The buffer is imported using the dma-buf api, and a driver specific
 * buffer descriptor is used to store the imported buffer properties.
 * The imported buffers are all stored in a rppc instance specific
 * idr, to be used for looking up and cleaning up the driver buffer
 * descriptors.
 *
 * The @autoreg field is used to dictate the manner in which the buffer
 * is imported. The user-side can pre-register the buffers with the driver
 * (which will import the buffers) if the application is going to use
 * these repeatedly in consecutive function invocations. The buffers
 * are auto-imported if the user-side has not registered them previously
 * and are un-imported once the remote function call returns.
 *
 * This function is to be called only after checking that buffer has
 * not been imported already (see rppc_find_dmabuf).
 *
 * Return: allocated rppc_dma_buf or error
 */
struct rppc_dma_buf *rppc_alloc_dmabuf(struct rppc_instance *rpc, int fd,
				       bool autoreg)
{
	struct rppc_device *rppcdev = rpc->rppcdev;
	struct rppc_dma_buf *dma;
	void *ret;
	int id;

	dma = kzalloc(sizeof(*dma), GFP_KERNEL);
	if (!dma)
		return ERR_PTR(-ENOMEM);

	dma->fd = fd;
	dma->autoreg = !!autoreg;
	dma->buf = dma_buf_get(dma->fd);
	if (IS_ERR(dma->buf)) {
		ret = dma->buf;
		goto free_dma;
	}

	dma->attach = dma_buf_attach(dma->buf, rppcdev->dev);
	if (IS_ERR(dma->attach)) {
		ret = dma->attach;
		goto put_buf;
	}

	dma->sgt = dma_buf_map_attachment(dma->attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(dma->sgt)) {
		ret = dma->sgt;
		goto detach_buf;
	}

	dma->pa = sg_dma_address(dma->sgt->sgl);
	mutex_lock(&rpc->lock);
	id = idr_alloc(&rpc->dma_idr, dma, 0, 0, GFP_KERNEL);
	dma->id = id;
	mutex_unlock(&rpc->lock);
	if (id < 0) {
		ret = ERR_PTR(id);
		goto unmap_buf;
	}

	return dma;

unmap_buf:
	dma_buf_unmap_attachment(dma->attach, dma->sgt, DMA_BIDIRECTIONAL);
detach_buf:
	dma_buf_detach(dma->buf, dma->attach);
put_buf:
	dma_buf_put(dma->buf);
free_dma:
	kfree(dma);

	return ret;
}

/**
 * rppc_free_dmabuf - release the imported buffer
 * @id: idr index of the imported buffer descriptor
 * @p: imported buffer descriptor allocated during rppc_alloc_dmabuf
 * @data: rpc instance handle
 *
 * This function is used to release a buffer that has been previously
 * imported through a rppc_alloc_dmabuf call. The function can be used
 * either individually for releasing a specific buffer or in a loop iterator
 * for releasing all the buffers associated with a remote function call, or
 * during cleanup of the rpc instance.
 *
 * Return: 0 on success, and -ENOENT if invalid pointers passed in
 */
int rppc_free_dmabuf(int id, void *p, void *data)
{
	struct rppc_dma_buf *dma = p;
	struct rppc_instance *rpc = data;

	if (!dma || !rpc)
		return -ENOENT;

	dma_buf_unmap_attachment(dma->attach, dma->sgt, DMA_BIDIRECTIONAL);
	dma_buf_detach(dma->buf, dma->attach);
	dma_buf_put(dma->buf);
	WARN_ON(id != dma->id);
	idr_remove(&rpc->dma_idr, id);
	kfree(dma);

	return 0;
}

/**
 * rppc_free_auto_dmabuf - release an auto-registered imported buffer
 * @id: idr index of the imported buffer descriptor
 * @p: imported buffer descriptor allocated during the rppc_alloc_dmabuf
 * @data: rpc instance handle
 *
 * This function is used to release a buffer that has been previously
 * imported automatically in the remote function invocation path (for
 * rppc_alloc_dmabuf invocations with autoreg set as true). The function
 * is used as a loop iterator for releasing all such buffers associated
 * with a remote function call, and is called after processing the
 * translations while handling the return message of an executed function
 * call.
 *
 * Return: 0 on success or if the buffer is not auto-imported, and -ENOENT
 *	   if invalid pointers passed in
 */
static int rppc_free_auto_dmabuf(int id, void *p, void *data)
{
	struct rppc_dma_buf *dma = p;
	struct rppc_instance *rpc = data;

	if (WARN_ON(!dma || !rpc))
		return -ENOENT;

	if (!dma->autoreg)
		return 0;

	rppc_free_dmabuf(id, p, data);
	return 0;
}

/**
 * find_dma_by_fd - find the allocated buffer descriptor
 * @id: idr loop index
 * @p: imported buffer descriptor associated with each idr index @id
 * @data: dma-buf file descriptor of the buffer
 *
 * This is a idr iterator helper function, used for checking if a buffer
 * has been imported before and present within the rpc instance's idr.
 *
 * Return: rpc buffer descriptor if file descriptor matches, and 0 otherwise
 */
static int find_dma_by_fd(int id, void *p, void *data)
{
	struct rppc_dma_buf *dma = p;
	int fd = (int)data;

	if (dma->fd == fd)
		return (int)p;

	return 0;
}

/**
 * rppc_find_dmabuf - find and return the rppc buffer descriptor of an imported
 *		      buffer
 * @rpc: rpc instance
 * @fd: dma-buf file descriptor of the buffer
 *
 * This function is used to find and return the rppc buffer descriptor of an
 * imported buffer. The function is used to check if ia buffer has already
 * been imported (during manual registration to return an error), and to return
 * the rppc buffer descriptor to be used for freeing (during manual
 * deregistration). It is also used during auto-registration to see if the
 * buffer needs to be imported through a rppc_alloc_dmabuf if not found.
 *
 * Return: rppc buffer descriptor of the buffer if it has already been imported,
 *	   or NULL otherwise.
 */
struct rppc_dma_buf *rppc_find_dmabuf(struct rppc_instance *rpc, int fd)
{
	struct rppc_dma_buf *node = NULL;
	void *data = (void *)fd;

	dev_dbg(rpc->rppcdev->dev, "looking for fd %u\n", fd);

	mutex_lock(&rpc->lock);
	node = (struct rppc_dma_buf *)
			idr_for_each(&rpc->dma_idr, find_dma_by_fd, data);
	mutex_unlock(&rpc->lock);

	dev_dbg(rpc->rppcdev->dev, "returning node %p for fd %u\n",
		node, fd);

	return node;
}

/**
 * rppc_map_page - import and map a kernel page in a dma_buf
 * @rpc - rppc instance handle
 * @fd: file descriptor of the dma_buf to import
 * @offset: offset of the translate location within the buffer
 * @base_ptr: pointer for returning mapped kernel address
 * @dmabuf: pointer for returning the imported dma_buf
 *
 * A helper function to import the dma_buf buffer and map into kernel
 * the page containing the offset within the buffer. The function is
 * called by rppc_xlate_buffers and returns the pointers to the kernel
 * mapped address and the imported dma_buf handle in arguments. The
 * mapping is used for performing in-place translation of the user
 * provided pointer at location @offset within the buffer.
 *
 * The mapping is achieved through the appropriate dma_buf ops, and
 * the page will be unmapped after performing the translation. See
 * also rppc_unmap_page.
 *
 * Return: 0 on success, or an appropriate failure code otherwise
 */
static int rppc_map_page(struct rppc_instance *rpc, int fd, u32 offset,
			 u8 **base_ptr, struct dma_buf **dmabuf)
{
	int ret = 0;
	u8 *ptr = NULL;
	struct dma_buf *dbuf = NULL;
	u32 pg_offset;
	unsigned long pg_num;
	size_t begin, end = PAGE_SIZE;
	struct device *dev = rpc->rppcdev->dev;

	if (!base_ptr || !dmabuf)
		return -EINVAL;

	pg_offset = (offset & (PAGE_SIZE - 1));
	begin = offset & PAGE_MASK;
	pg_num = offset >> PAGE_SHIFT;

	dbuf = dma_buf_get(fd);
	if (IS_ERR(dbuf)) {
		ret = PTR_ERR(dbuf);
		dev_err(dev, "invalid dma_buf file descriptor passed! fd = %d ret = %d\n",
			fd, ret);
		goto out;
	}

	ret = dma_buf_begin_cpu_access(dbuf, DMA_BIDIRECTIONAL);
	if (ret < 0) {
		dev_err(dev, "failed to acquire cpu access to the dma buf fd = %d offset = 0x%x, ret = %d\n",
			fd, offset, ret);
		goto put_dmabuf;
	}

	ptr = dma_buf_kmap(dbuf, pg_num);
	if (!ptr) {
		ret = -ENOBUFS;
		dev_err(dev, "failed to map the page containing the translation into kernel fd = %d offset = 0x%x\n",
			fd, offset);
		goto end_cpuaccess;
	}

	*base_ptr = ptr;
	*dmabuf = dbuf;
	dev_dbg(dev, "kmap'd base_ptr = %p buf = %p into kernel from %zu for %zu bytes, pg_offset = 0x%x\n",
		ptr, dbuf, begin, end, pg_offset);
	return 0;

end_cpuaccess:
	dma_buf_end_cpu_access(dbuf, DMA_BIDIRECTIONAL);
put_dmabuf:
	dma_buf_put(dbuf);
out:
	return ret;
}

/**
 * rppc_unmap_page - unmap and release a previously mapped page
 * @rpc - rppc instance handle
 * @offset: offset of the translate location within the buffer
 * @base_ptr: kernel mapped address for the page to be unmapped
 * @dmabuf: imported dma_buf to be released
 *
 * This function is called by rppc_xlate_buffers to unmap the
 * page and release the imported buffer. It essentially undoes
 * the functionality of rppc_map_page.
 */
static void rppc_unmap_page(struct rppc_instance *rpc, u32 offset,
			    u8 *base_ptr, struct dma_buf *dmabuf)
{
	u32 pg_offset;
	unsigned long pg_num;
	size_t begin, end = PAGE_SIZE;
	struct device *dev = rpc->rppcdev->dev;

	if (!base_ptr || !dmabuf)
		return;

	pg_offset = (offset & (PAGE_SIZE - 1));
	begin = offset & PAGE_MASK;
	pg_num = offset >> PAGE_SHIFT;

	dev_dbg(dev, "Unkmaping base_ptr = %p of buf = %p from %zu to %zu bytes\n",
		base_ptr, dmabuf, begin, end);
	dma_buf_kunmap(dmabuf, pg_num, base_ptr);
	dma_buf_end_cpu_access(dmabuf, DMA_BIDIRECTIONAL);
	dma_buf_put(dmabuf);
}

/**
 * rppc_buffer_lookup - convert a buffer pointer to a remote processor pointer
 * @rpc: rpc instance
 * @uva: buffer pointer that needs to be translated
 * @buva: base pointer of the allocated buffer
 * @fd: dma-buf file descriptor of the allocated buffer
 *
 * This function is used for converting a pointer value in the function
 * arguments to its appropriate remote processor device address value.
 * The @uva and @buva are used for identifying the offset of the function
 * argument pointer in an original allocation. This supports the cases where
 * an offset pointer (eg: alignment, packed buffers etc) needs to be passed
 * as the argument rather than the actual allocated pointer.
 *
 * The remote processor device address is done by retrieving the base physical
 * address of the buffer by importing the buffer and converting it to the
 * remote processor device address using a remoteproc api, with adjustments
 * to the offset.
 *
 * The offset is specifically adjusted for OMAP TILER to account for the stride
 * and mapping onto the remote processor.
 *
 * Return: remote processor device address, 0 on failure (implies invalid
 *	   arguments)
 */
dev_addr_t rppc_buffer_lookup(struct rppc_instance *rpc, virt_addr_t uva,
			      virt_addr_t buva, int fd)
{
	phys_addr_t lpa = 0;
	dev_addr_t rda = 0;
	long uoff = uva - buva;
	struct device *dev = rpc->rppcdev->dev;
	struct rppc_dma_buf *buf;

	dev_dbg(dev, "buva = %p uva = %p offset = %ld [0x%016lx] fd = %d\n",
		(void *)buva, (void *)uva, uoff, (ulong)uoff, fd);

	if (uoff < 0) {
		dev_err(dev, "invalid pointer values for uva = %p from buva = %p\n",
			(void *)uva, (void *)buva);
		return rda;
	}

	buf = rppc_find_dmabuf(rpc, fd);
	if (IS_ERR_OR_NULL(buf)) {
		buf = rppc_alloc_dmabuf(rpc, fd, true);
		if (IS_ERR(buf))
			goto out;
	}

	lpa = buf->pa;
	WARN_ON(lpa != sg_dma_address(buf->sgt->sgl));
	uoff = rppc_recalc_off(lpa, uoff);
	lpa += uoff;
	rda = rppc_local_to_remote_da(rpc, lpa);

out:
	dev_dbg(dev, "host uva %p == host pa %pa => remote da %p (fd %d)\n",
		(void *)uva, &lpa, (void *)rda, fd);
	return rda;
}

/**
 * rppc_xlate_buffers - translate argument pointers in the marshalled packet
 * @rpc: rppc instance
 * @func: rppc function packet being acted upon
 * @direction: direction of translation
 *
 * This function translates all the pointers within the function call packet
 * structure, based on the translation descriptor structures. The translation
 * replaces the pointers to the appropriate pointers based on the direction.
 * The function is invoked in preparing the packet to be sent to the remote
 * processor-side and replaces the pointers to the remote processor device
 * address pointers; and in processing the packet back after executing the
 * function and replacing back the remote processor device addresses with
 * the original pointers.
 *
 * Return: 0 on success, or an appropriate failure code otherwise
 */
int rppc_xlate_buffers(struct rppc_instance *rpc, struct rppc_function *func,
		       int direction)
{
	u8 *base_ptr = NULL;
	struct dma_buf *dbuf = NULL;
	struct device *dev = rpc->rppcdev->dev;
	u32 ptr_idx, pri_offset, sec_offset, offset, pg_offset, size;
	int i, limit, inc = 1;
	virt_addr_t kva, uva, buva;
	dev_addr_t rda;
	int ret = 0, final_ret = 0;
	int xlate_fd;

	limit = func->num_translations;
	if (WARN_ON(!limit))
		return 0;

	dev_dbg(dev, "operating on %d pointers\n", func->num_translations);

	/* sanity check the translation elements */
	for (i = 0; i < limit; i++) {
		ptr_idx = func->translations[i].index;

		if (ptr_idx >= RPPC_MAX_PARAMETERS) {
			dev_err(dev, "xlate[%d] - invalid parameter pointer index %u\n",
				i, ptr_idx);
			return -EINVAL;
		}
		if (func->params[ptr_idx].type != RPPC_PARAM_TYPE_PTR) {
			dev_err(dev, "xlate[%d] - parameter index %u is not a pointer (type %u)\n",
				i, ptr_idx, func->params[ptr_idx].type);
			return -EINVAL;
		}
		if (func->params[ptr_idx].data == 0) {
			dev_err(dev, "xlate[%d] - supplied user pointer is NULL!\n",
				i);
			return -EINVAL;
		}

		pri_offset = func->params[ptr_idx].data -
					func->params[ptr_idx].base;
		sec_offset = func->translations[i].offset;
		size = func->params[ptr_idx].size;

		if (sec_offset > (size - sizeof(virt_addr_t))) {
			dev_err(dev, "xlate[%d] offset is larger than data area! (sec_offset = %u size = %u)\n",
				i, sec_offset, size);
			return -ENOSPC;
		}
	}

	/*
	 * we may have a failure during translation, in which case use the same
	 * loop to unwind the whole operation
	 */
	for (i = 0; i != limit; i += inc) {
		dev_dbg(dev, "starting translation %d of %d by %d\n",
			i, limit, inc);

		ptr_idx = func->translations[i].index;
		pri_offset = func->params[ptr_idx].data -
						func->params[ptr_idx].base;
		sec_offset = func->translations[i].offset;
		offset = pri_offset + sec_offset;
		pg_offset = (offset & (PAGE_SIZE - 1));

		/*
		 * map into kernel the page containing the offset, where the
		 * pointer needs to be translated.
		 */
		ret = rppc_map_page(rpc, func->params[ptr_idx].fd, offset,
				    &base_ptr, &dbuf);
		if (ret) {
			dev_err(dev, "rppc_map_page failed, translation = %d param_index = %d fd = %d ret = %d\n",
				i, ptr_idx, func->params[ptr_idx].fd, ret);
			goto unwind;
		}

		/*
		 * perform the actual translation as per the direction.
		 */
		if (direction == RPPC_UVA_TO_RPA) {
			kva = (virt_addr_t)&base_ptr[pg_offset];
			if (kva & 0x3) {
				dev_err(dev, "kernel virtual address %p is not aligned for translation = %d\n",
					(void *)kva, i);
				ret = -EADDRNOTAVAIL;
				goto unmap;
			}

			uva = *(virt_addr_t *)kva;
			if (!uva) {
				dev_err(dev, "user pointer in the translated offset location is NULL for translation = %d\n",
					i);
				print_hex_dump(KERN_DEBUG, "KMAP: ",
					       DUMP_PREFIX_NONE, 16, 1,
					       base_ptr, PAGE_SIZE, true);
				ret = -EADDRNOTAVAIL;
				goto unmap;
			}

			buva = (virt_addr_t)func->translations[i].base;
			xlate_fd = func->translations[i].fd;

			dev_dbg(dev, "replacing UVA %p at KVA %p prt_idx = %u pg_offset = 0x%x fd = %d\n",
				(void *)uva, (void *)kva, ptr_idx,
				pg_offset, xlate_fd);

			/* compute the corresponding remote device address */
			rda = rppc_buffer_lookup(rpc, uva, buva, xlate_fd);
			if (!rda) {
				ret = -ENODATA;
				goto unmap;
			}

			/*
			 * replace the pointer, save the old value for replacing
			 * it back on the function return path
			 */
			func->translations[i].fd = (int32_t)uva;
			*(virt_addr_t *)kva = rda;
			dev_dbg(dev, "replaced UVA %p with RDA %p at KVA %p\n",
				(void *)uva, (void *)rda, (void *)kva);
		} else if (direction == RPPC_RPA_TO_UVA) {
			kva = (virt_addr_t)&base_ptr[pg_offset];
			if (kva & 0x3) {
				ret = -EADDRNOTAVAIL;
				goto unmap;
			}

			rda = *(virt_addr_t *)kva;
			uva = (virt_addr_t)func->translations[i].fd;
			WARN_ON(!uva);
			*(virt_addr_t *)kva = uva;

			dev_dbg(dev, "replaced RDA %p with UVA %p at KVA %p\n",
				(void *)rda, (void *)uva, (void *)kva);
		}

unmap:
		/*
		 * unmap the page containing the translation from kernel, the
		 * next translation acting on the same fd might be in a
		 * different page altogether from the current one
		 */
		rppc_unmap_page(rpc, offset, base_ptr, dbuf);
		dbuf = NULL;
		base_ptr = NULL;

		if (!ret)
			continue;

unwind:
		/*
		 * unwind all the previous translations if the failure occurs
		 * while sending a message to the remote-side. There's nothing
		 * to do but to continue if the failure occurs during the
		 * processing of a function response.
		 */
		if (direction == RPPC_UVA_TO_RPA) {
			dev_err(dev, "unwinding UVA to RDA translations! translation = %d\n",
				i);
			direction = RPPC_RPA_TO_UVA;
			inc = -1;
			limit = -1;
		} else if (direction == RPPC_RPA_TO_UVA) {
			dev_err(dev, "error during UVA to RDA translations!! current translation = %d\n",
				i);
		}
		/*
		 * store away the return value to return back to caller
		 * in case of an error, record only the first error
		 */
		if (!final_ret)
			final_ret = ret;
	}

	/*
	 * all the in-place pointer replacements are done, release all the
	 * imported buffers during the remote function return path
	 */
	if (direction == RPPC_RPA_TO_UVA) {
		mutex_lock(&rpc->lock);
		idr_for_each(&rpc->dma_idr, rppc_free_auto_dmabuf, rpc);
		mutex_unlock(&rpc->lock);
	}

	return final_ret;
}
