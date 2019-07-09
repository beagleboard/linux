/*
 * Copyright (C) 2009 Philippe Gerum <rpm@xenomai.org>.
 *
 * Xenomai is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Xenomai is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Xenomai; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 * 02111-1307, USA.
 */
#include <cobalt/kernel/heap.h>
#include <cobalt/kernel/sched.h>
#include <cobalt/kernel/bufd.h>
#include <cobalt/kernel/assert.h>
#include <asm/xenomai/syscall.h>

/**
 * @ingroup cobalt_core
 * @defgroup cobalt_core_bufd Buffer descriptor
 *
 * Abstraction for copying data to/from different address spaces
 *
 * A buffer descriptor is a simple abstraction dealing with copy
 * operations to/from memory buffers which may belong to different
 * address spaces.
 *
 * To this end, the buffer descriptor library provides a small set of
 * copy routines which are aware of address space restrictions when
 * moving data, and a generic container type which can hold a
 * reference to - or cover - a particular memory area, either present
 * in kernel space, or in any of the existing user memory contexts.
 *
 * The goal of the buffer descriptor abstraction is to hide address
 * space specifics from Xenomai services dealing with memory areas,
 * allowing them to operate on multiple address spaces seamlessly.
 *
 * The common usage patterns are as follows:
 *
 * - Implementing a Xenomai syscall returning a bulk of data to the
 *   caller, which may have to be copied back to either kernel or user
 *   space:
 *
 *   @code
 *   [Syscall implementation]
 *   ssize_t rt_bulk_read_inner(struct xnbufd *bufd)
 *   {
 *       ssize_t ret;
 *       size_t len;
 *       void *bulk;
 *
 *       bulk = get_next_readable_bulk(&len);
 *       ret = xnbufd_copy_from_kmem(bufd, bulk, min(bufd->b_len, len));
 *       free_bulk(bulk);
 *
 *       ret = this_may_fail();
 *       if (ret)
 *	       xnbufd_invalidate(bufd);
 *
 *       return ret;
 *   }
 *
 *   [Kernel wrapper for in-kernel calls]
 *   int rt_bulk_read(void *ptr, size_t len)
 *   {
 *       struct xnbufd bufd;
 *       ssize_t ret;
 *
 *       xnbufd_map_kwrite(&bufd, ptr, len);
 *       ret = rt_bulk_read_inner(&bufd);
 *       xnbufd_unmap_kwrite(&bufd);
 *
 *       return ret;
 *   }
 *
 *   [Userland trampoline for user syscalls]
 *   int __rt_bulk_read(struct pt_regs *regs)
 *   {
 *       struct xnbufd bufd;
 *       void __user *ptr;
 *       ssize_t ret;
 *       size_t len;
 *
 *       ptr = (void __user *)__xn_reg_arg1(regs);
 *       len = __xn_reg_arg2(regs);
 *
 *       xnbufd_map_uwrite(&bufd, ptr, len);
 *       ret = rt_bulk_read_inner(&bufd);
 *       xnbufd_unmap_uwrite(&bufd);
 *
 *       return ret;
 *   }
 *   @endcode
 *
 * - Implementing a Xenomai syscall receiving a bulk of data from the
 *   caller, which may have to be read from either kernel or user
 *   space:
 *
 *   @code
 *   [Syscall implementation]
 *   ssize_t rt_bulk_write_inner(struct xnbufd *bufd)
 *   {
 *       void *bulk = get_free_bulk(bufd->b_len);
 *       return xnbufd_copy_to_kmem(bulk, bufd, bufd->b_len);
 *   }
 *
 *   [Kernel wrapper for in-kernel calls]
 *   int rt_bulk_write(const void *ptr, size_t len)
 *   {
 *       struct xnbufd bufd;
 *       ssize_t ret;
 *
 *       xnbufd_map_kread(&bufd, ptr, len);
 *       ret = rt_bulk_write_inner(&bufd);
 *       xnbufd_unmap_kread(&bufd);
 *
 *       return ret;
 *   }
 *
 *   [Userland trampoline for user syscalls]
 *   int __rt_bulk_write(struct pt_regs *regs)
 *   {
 *       struct xnbufd bufd;
 *       void __user *ptr;
 *       ssize_t ret;
 *       size_t len;
 *
 *       ptr = (void __user *)__xn_reg_arg1(regs);
 *       len = __xn_reg_arg2(regs);
 *
 *       xnbufd_map_uread(&bufd, ptr, len);
 *       ret = rt_bulk_write_inner(&bufd);
 *       xnbufd_unmap_uread(&bufd);
 *
 *       return ret;
 *   }
 *   @endcode
 *
 *@{*/

/**
 * @fn void xnbufd_map_kread(struct xnbufd *bufd, const void *ptr, size_t len)
 * @brief Initialize a buffer descriptor for reading from kernel memory.
 *
 * The new buffer descriptor may be used to copy data from kernel
 * memory. This routine should be used in pair with
 * xnbufd_unmap_kread().
 *
 * @param bufd The address of the buffer descriptor which will map a
 * @a len bytes kernel memory area, starting from @a ptr.
 *
 * @param ptr The start of the kernel buffer to map.
 *
 * @param len The length of the kernel buffer starting at @a ptr.
 *
 * @coretags{unrestricted}
 */

/**
 * @fn void xnbufd_map_kwrite(struct xnbufd *bufd, void *ptr, size_t len)
 * @brief Initialize a buffer descriptor for writing to kernel memory.
 *
 * The new buffer descriptor may be used to copy data to kernel
 * memory. This routine should be used in pair with
 * xnbufd_unmap_kwrite().
 *
 * @param bufd The address of the buffer descriptor which will map a
 * @a len bytes kernel memory area, starting from @a ptr.
 *
 * @param ptr The start of the kernel buffer to map.
 *
 * @param len The length of the kernel buffer starting at @a ptr.
 *
 * @coretags{unrestricted}
 */
void xnbufd_map_kmem(struct xnbufd *bufd, void *ptr, size_t len)
{
	bufd->b_ptr = ptr;
	bufd->b_len = len;
	bufd->b_mm = NULL;
	bufd->b_off = 0;
	bufd->b_carry = NULL;
}
EXPORT_SYMBOL_GPL(xnbufd_map_kmem);

/**
 * @fn void xnbufd_map_uread(struct xnbufd *bufd, const void __user *ptr, size_t len)
 * @brief Initialize a buffer descriptor for reading from user memory.
 *
 * The new buffer descriptor may be used to copy data from user
 * memory. This routine should be used in pair with
 * xnbufd_unmap_uread().
 *
 * @param bufd The address of the buffer descriptor which will map a
 * @a len bytes user memory area, starting from @a ptr. @a ptr is
 * never dereferenced directly, since it may refer to a buffer that
 * lives in another address space.
 *
 * @param ptr The start of the user buffer to map.
 *
 * @param len The length of the user buffer starting at @a ptr.
 *
 * @coretags{task-unrestricted}
 */

/**
 * @fn void xnbufd_map_uwrite(struct xnbufd *bufd, void __user *ptr, size_t len)
 * @brief Initialize a buffer descriptor for writing to user memory.
 *
 * The new buffer descriptor may be used to copy data to user
 * memory. This routine should be used in pair with
 * xnbufd_unmap_uwrite().
 *
 * @param bufd The address of the buffer descriptor which will map a
 * @a len bytes user memory area, starting from @a ptr. @a ptr is
 * never dereferenced directly, since it may refer to a buffer that
 * lives in another address space.
 *
 * @param ptr The start of the user buffer to map.
 *
 * @param len The length of the user buffer starting at @a ptr.
 *
 * @coretags{task-unrestricted}
 */

void xnbufd_map_umem(struct xnbufd *bufd, void __user *ptr, size_t len)
{
	bufd->b_ptr = ptr;
	bufd->b_len = len;
	bufd->b_mm = current->mm;
	bufd->b_off = 0;
	bufd->b_carry = NULL;
}
EXPORT_SYMBOL_GPL(xnbufd_map_umem);

/**
 * @fn ssize_t xnbufd_copy_to_kmem(void *to, struct xnbufd *bufd, size_t len)
 * @brief Copy memory covered by a buffer descriptor to kernel memory.
 *
 * This routine copies @a len bytes from the area referred to by the
 * buffer descriptor @a bufd to the kernel memory area @a to.
 * xnbufd_copy_to_kmem() tracks the read offset within the source
 * memory internally, so that it may be called several times in a
 * loop, until the entire memory area is loaded.
 *
 * The source address space is dealt with, according to the following
 * rules:
 *
 * - if @a bufd refers to readable kernel area (i.e. see
 *   xnbufd_map_kread()), the copy is immediately and fully performed
 *   with no restriction.
 *
 * - if @a bufd refers to a readable user area (i.e. see
 *   xnbufd_map_uread()), the copy is performed only if that area
 *   lives in the currently active address space, and only if the
 *   caller may sleep Linux-wise to process any potential page fault
 *   which may arise while reading from that memory.
 *
 * - any attempt to read from @a bufd from a non-suitable context is
 *   considered as a bug, and will raise a panic assertion when the
 *   nucleus is compiled in debug mode.
 *
 * @param to The start address of the kernel memory to copy to.
 *
 * @param bufd The address of the buffer descriptor covering the user
 * memory to copy data from.
 *
 * @param len The length of the user memory to copy from @a bufd.
 *
 * @return The number of bytes read so far from the memory area
 * covered by @a ubufd. Otherwise:
 *
 * - -EINVAL is returned upon attempt to read from the user area from
 *   an invalid context. This error is only returned when the debug
 *   mode is disabled; otherwise a panic assertion is raised.
 *
 * @coretags{task-unrestricted}
 *
 * @note Calling this routine while holding the nklock and/or running
 * with interrupts disabled is invalid, and doing so will trigger a
 * debug assertion.
 *
 * This routine may switch the caller to secondary mode if a page
 * fault occurs while reading from the user area. For that reason,
 * xnbufd_copy_to_kmem() may only be called from a preemptible section
 * (Linux-wise).
 */
ssize_t xnbufd_copy_to_kmem(void *to, struct xnbufd *bufd, size_t len)
{
	caddr_t from;

	thread_only();

	if (len == 0)
		goto out;

	from = bufd->b_ptr + bufd->b_off;

	/*
	 * If the descriptor covers a source buffer living in the
	 * kernel address space, we may read from it directly.
	 */
	if (bufd->b_mm == NULL) {
		memcpy(to, from, len);
		goto advance_offset;
	}

	/*
	 * We want to read data from user-space, check whether:
	 * 1) the source buffer lies in the current address space,
	 * 2) we may fault while reading from the buffer directly.
	 *
	 * If we can't reach the buffer, or the current context may
	 * not fault while reading data from it, copy_from_user() is
	 * not an option and we have a bug somewhere, since there is
	 * no way we could fetch the data to kernel space immediately.
	 *
	 * Note that we don't check for non-preemptible Linux context
	 * here, since the source buffer would live in kernel space in
	 * such a case.
	 */
	if (current->mm == bufd->b_mm) {
		preemptible_only();
		if (cobalt_copy_from_user(to, (void __user *)from, len))
			return -EFAULT;
		goto advance_offset;
	}

	XENO_BUG(COBALT);

	return -EINVAL;

advance_offset:
	bufd->b_off += len;
out:
	return (ssize_t)bufd->b_off;
}
EXPORT_SYMBOL_GPL(xnbufd_copy_to_kmem);

/**
 * @fn ssize_t xnbufd_copy_from_kmem(struct xnbufd *bufd, void *from, size_t len)
 * @brief Copy kernel memory to the area covered by a buffer descriptor.
 *
 * This routine copies @a len bytes from the kernel memory starting at
 * @a from to the area referred to by the buffer descriptor @a
 * bufd. xnbufd_copy_from_kmem() tracks the write offset within the
 * destination memory internally, so that it may be called several
 * times in a loop, until the entire memory area is stored.
 *
 * The destination address space is dealt with, according to the
 * following rules:
 *
 * - if @a bufd refers to a writable kernel area (i.e. see
 *   xnbufd_map_kwrite()), the copy is immediatly and fully performed
 *   with no restriction.
 *
 * - if @a bufd refers to a writable user area (i.e. see
 *   xnbufd_map_uwrite()), the copy is performed only if that area
 *   lives in the currently active address space, and only if the
 *   caller may sleep Linux-wise to process any potential page fault
 *   which may arise while writing to that memory.
 *
 * - if @a bufd refers to a user area which may not be immediately
 *   written to from the current context, the copy is postponed until
 *   xnbufd_unmap_uwrite() is invoked for @a ubufd, at which point the
 *   copy will take place. In such a case, the source memory is
 *   transferred to a carry over buffer allocated internally; this
 *   operation may lead to request dynamic memory from the nucleus
 *   heap if @a len is greater than 64 bytes.
 *
 * @param bufd The address of the buffer descriptor covering the user
 * memory to copy data to.
 *
 * @param from The start address of the kernel memory to copy from.
 *
 * @param len The length of the kernel memory to copy to @a bufd.
 *
 * @return The number of bytes written so far to the memory area
 * covered by @a ubufd. Otherwise,
 *
 * - -ENOMEM is returned when no memory is available from the nucleus
 *    heap to allocate the carry over buffer.
 *
 * @coretags{unrestricted}
 *
 * @note Calling this routine while holding the nklock and/or running
 * with interrupts disabled is invalid, and doing so will trigger a
 * debug assertion.
 *
 * This routine may switch the caller to secondary mode if a page
 * fault occurs while reading from the user area. For that reason,
 * xnbufd_copy_to_kmem() may only be called from a preemptible section
 * (Linux-wise).
 */
ssize_t xnbufd_copy_from_kmem(struct xnbufd *bufd, void *from, size_t len)
{
	caddr_t to;

	thread_only();

	if (len == 0)
		goto out;

	to = bufd->b_ptr + bufd->b_off;

	/*
	 * If the descriptor covers a destination buffer living in the
	 * kernel address space, we may copy to it directly.
	 */
	if (bufd->b_mm == NULL)
		goto direct_copy;

	/*
	 * We want to pass data to user-space, check whether:
	 * 1) the destination buffer lies in the current address space,
	 * 2) we may fault while writing to the buffer directly.
	 *
	 * If we can't reach the buffer, or the current context may
	 * not fault while copying data to it, copy_to_user() is not
	 * an option and we have to convey the data from kernel memory
	 * through the carry over buffer.
	 *
	 * Note that we don't check for non-preemptible Linux context
	 * here: feeding a RT activity with data from a non-RT context
	 * is wrong in the first place, so never mind.
	 */
	if (current->mm == bufd->b_mm) {
		preemptible_only();
		if (cobalt_copy_to_user((void __user *)to, from, len))
			return -EFAULT;
		goto advance_offset;
	}

	/*
	 * We need a carry over buffer to convey the data to
	 * user-space. xnbufd_unmap_uwrite() should be called on the
	 * way back to user-space to update the destination buffer
	 * from the carry over area.
	 */
	if (bufd->b_carry == NULL) {
		/*
		 * Try to use the fast carry over area available
		 * directly from the descriptor for short messages, to
		 * save a dynamic allocation request.
		 */
		if (bufd->b_len <= sizeof(bufd->b_buf))
			bufd->b_carry = bufd->b_buf;
		else {
			bufd->b_carry = xnmalloc(bufd->b_len);
			if (bufd->b_carry == NULL)
				return -ENOMEM;
		}
		to = bufd->b_carry;
	} else
		to = bufd->b_carry + bufd->b_off;

direct_copy:
	memcpy(to, from, len);

advance_offset:
	bufd->b_off += len;
out:
	return (ssize_t)bufd->b_off;
}
EXPORT_SYMBOL_GPL(xnbufd_copy_from_kmem);

/**
 * @fn void xnbufd_unmap_uread(struct xnbufd *bufd)
 * @brief Finalize a buffer descriptor obtained from xnbufd_map_uread().
 *
 * This routine finalizes a buffer descriptor previously initialized
 * by a call to xnbufd_map_uread(), to read data from a user area.
 *
 * @param bufd The address of the buffer descriptor to finalize.
 *
 * @return The number of bytes read so far from the memory area
 * covered by @a ubufd.
 *
 * @coretags{task-unrestricted}
 *
 * @note Calling this routine while holding the nklock and/or running
 * with interrupts disabled is invalid, and doing so will trigger a
 * debug assertion.
 */
ssize_t xnbufd_unmap_uread(struct xnbufd *bufd)
{
	preemptible_only();

#ifdef CONFIG_XENO_OPT_DEBUG_COBALT
	bufd->b_ptr = (caddr_t)-1;
#endif
	return bufd->b_off;
}
EXPORT_SYMBOL_GPL(xnbufd_unmap_uread);

/**
 * @fn void xnbufd_unmap_uwrite(struct xnbufd *bufd)
 * @brief Finalize a buffer descriptor obtained from xnbufd_map_uwrite().
 *
 * This routine finalizes a buffer descriptor previously initialized
 * by a call to xnbufd_map_uwrite(), to write data to a user area.
 *
 * The main action taken is to write the contents of the kernel memory
 * area passed to xnbufd_copy_from_kmem() whenever the copy operation
 * was postponed at that time; the carry over buffer is eventually
 * released as needed. If xnbufd_copy_from_kmem() was allowed to copy
 * to the destination user memory at once, then xnbufd_unmap_uwrite()
 * leads to a no-op.
 *
 * @param bufd The address of the buffer descriptor to finalize.
 *
 * @return The number of bytes written so far to the memory area
 * covered by @a ubufd.
 *
 * @coretags{task-unrestricted}
 *
 * @note Calling this routine while holding the nklock and/or running
 * with interrupts disabled is invalid, and doing so will trigger a
 * debug assertion.
 */
ssize_t xnbufd_unmap_uwrite(struct xnbufd *bufd)
{
	ssize_t ret = 0;
	void __user *to;
	void *from;
	size_t len;

	preemptible_only();

	len = bufd->b_off;

	if (bufd->b_carry == NULL)
		/* Copy took place directly. Fine. */
		goto done;

	/*
	 * Something was written to the carry over area, copy the
	 * contents to user-space, then release the area if needed.
	 */
	to = (void __user *)bufd->b_ptr;
	from = bufd->b_carry;
	ret = cobalt_copy_to_user(to, from, len);

	if (bufd->b_len > sizeof(bufd->b_buf))
		xnfree(bufd->b_carry);
done:
#ifdef CONFIG_XENO_OPT_DEBUG_COBALT
	bufd->b_ptr = (caddr_t)-1;
#endif
	return ret ?: (ssize_t)len;
}
EXPORT_SYMBOL_GPL(xnbufd_unmap_uwrite);

/**
 * @fn void xnbufd_reset(struct xnbufd *bufd)
 * @brief Reset a buffer descriptor.
 *
 * The buffer descriptor is reset, so that all data already copied is
 * forgotten. Any carry over buffer allocated is kept, though.
 *
 * @param bufd The address of the buffer descriptor to reset.
 *
 * @coretags{unrestricted}
 */

/**
 * @fn void xnbufd_invalidate(struct xnbufd *bufd)
 * @brief Invalidate a buffer descriptor.
 *
 * The buffer descriptor is invalidated, making it unusable for
 * further copy operations. If an outstanding carry over buffer was
 * allocated by a previous call to xnbufd_copy_from_kmem(), it is
 * immediately freed so that no data transfer will happen when the
 * descriptor is finalized.
 *
 * The only action that may subsequently be performed on an
 * invalidated descriptor is calling the relevant unmapping routine
 * for it. For that reason, xnbufd_invalidate() should be invoked on
 * the error path when data may have been transferred to the carry
 * over buffer.
 *
 * @param bufd The address of the buffer descriptor to invalidate.
 *
 * @coretags{unrestricted}
 */
void xnbufd_invalidate(struct xnbufd *bufd)
{
#ifdef CONFIG_XENO_OPT_DEBUG_COBALT
	bufd->b_ptr = (caddr_t)-1;
#endif
	if (bufd->b_carry) {
		if (bufd->b_len > sizeof(bufd->b_buf))
			xnfree(bufd->b_carry);
		bufd->b_carry = NULL;
	}
	bufd->b_off = 0;
}
EXPORT_SYMBOL_GPL(xnbufd_invalidate);

/**
 * @fn void xnbufd_unmap_kread(struct xnbufd *bufd)
 * @brief Finalize a buffer descriptor obtained from xnbufd_map_kread().
 *
 * This routine finalizes a buffer descriptor previously initialized
 * by a call to xnbufd_map_kread(), to read data from a kernel area.
 *
 * @param bufd The address of the buffer descriptor to finalize.
 *
 * @return The number of bytes read so far from the memory area
 * covered by @a ubufd.
 *
 * @coretags{task-unrestricted}
 */
ssize_t xnbufd_unmap_kread(struct xnbufd *bufd)
{
#ifdef CONFIG_XENO_OPT_DEBUG_COBALT
	bufd->b_ptr = (caddr_t)-1;
#endif
	return bufd->b_off;
}
EXPORT_SYMBOL_GPL(xnbufd_unmap_kread);

/**
 * @fn void xnbufd_unmap_kwrite(struct xnbufd *bufd)
 * @brief Finalize a buffer descriptor obtained from xnbufd_map_kwrite().
 *
 * This routine finalizes a buffer descriptor previously initialized
 * by a call to xnbufd_map_kwrite(), to write data to a kernel area.
 *
 * @param bufd The address of the buffer descriptor to finalize.
 *
 * @return The number of bytes written so far to the memory area
 * covered by @a ubufd.
 *
 * @coretags{task-unrestricted}
 */
ssize_t xnbufd_unmap_kwrite(struct xnbufd *bufd)
{
#ifdef CONFIG_XENO_OPT_DEBUG_COBALT
	bufd->b_ptr = (caddr_t)-1;
#endif
	return bufd->b_off;
}
EXPORT_SYMBOL_GPL(xnbufd_unmap_kwrite);

/** @} */
