// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2011 Google, Inc.
 * Copyright (C) 2012 Intel, Inc.
 * Copyright (C) 2013 Intel, Inc.
 * Copyright (C) 2014 Linaro Limited
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

/* This source file contains the implementation of the legacy version of
 * a goldfish pipe device driver. See goldfish_pipe_v2.c for the current
 * version.
 */

#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/bitops.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/mm.h>
#include <linux/bug.h>
#include <linux/goldfish.h>

#include "goldfish_pipe_qemu.h"
#include "goldfish_pipe.h"

#define MAX_PAGES_TO_GRAB 32

/* A value that will not be set by qemu emulator */
#define INITIAL_BATCH_RESULT (0xdeadbeaf)

struct goldfish_pipe_dev;

/* This data type models a given pipe instance */
struct goldfish_pipe {
	struct goldfish_pipe_dev *dev;

	/* The wake flags pipe is waiting for
	 * Note: not protected with any lock, uses atomic operations
	 *  and barriers to make it thread-safe.
	 */
	unsigned long flags;

	wait_queue_head_t wake_queue;

	/* protects access to the pipe */
	struct mutex lock;
};

struct access_params {
	unsigned long channel;
	u32 size;
	unsigned long address;
	u32 cmd;
	u32 result;
	/* reserved for future extension */
	u32 flags;
};

/* The driver state. Holds a reference to the i/o page used to
 * communicate with the emulator, and a wake queue for blocked tasks
 * waiting to be awoken.
 */
struct goldfish_pipe_dev {
	/* Needed for the 'remove' call */
	struct goldfish_pipe_dev_base super;

	/* ptr to platform device's device struct */
	struct device *pdev_dev;

	/* the base address for MMIO */
	char __iomem *base;

	struct access_params *aps;

	struct miscdevice miscdev;

	/* Global device spinlock */
	spinlock_t lock;
};

static int goldfish_pipe_device_deinit(void *raw_dev,
				       struct platform_device *pdev);

static u32 goldfish_cmd_status(struct goldfish_pipe *pipe, u32 cmd)
{
	unsigned long flags;
	u32 status;
	struct goldfish_pipe_dev *dev = pipe->dev;

	spin_lock_irqsave(&dev->lock, flags);
	gf_write_ptr(pipe, dev->base + PIPE_V1_REG_CHANNEL,
		     dev->base + PIPE_V1_REG_CHANNEL_HIGH);
	writel(cmd, dev->base + PIPE_V1_REG_COMMAND);
	status = readl(dev->base + PIPE_V1_REG_STATUS);
	spin_unlock_irqrestore(&dev->lock, flags);
	return status;
}

static void goldfish_cmd(struct goldfish_pipe *pipe, u32 cmd)
{
	unsigned long flags;
	struct goldfish_pipe_dev *dev = pipe->dev;

	spin_lock_irqsave(&dev->lock, flags);
	gf_write_ptr(pipe, dev->base + PIPE_V1_REG_CHANNEL,
		     dev->base + PIPE_V1_REG_CHANNEL_HIGH);
	writel(cmd, dev->base + PIPE_V1_REG_COMMAND);
	spin_unlock_irqrestore(&dev->lock, flags);
}

/* This function converts an error code returned by the emulator through
 * the PIPE_V1_REG_STATUS i/o register into a valid negative errno value.
 */
static int goldfish_pipe_error_convert(int status)
{
	switch (status) {
	case PIPE_ERROR_AGAIN:
		return -EAGAIN;
	case PIPE_ERROR_NOMEM:
		return -ENOMEM;
	case PIPE_ERROR_IO:
		return -EIO;
	default:
		return -EINVAL;
	}
}

/*
 * Notice: QEMU will return 0 for un-known register access, indicating
 * access_params is supported or not
 */
static int valid_batchbuffer_addr(struct goldfish_pipe_dev *dev,
				  struct access_params *aps)
{
	u32 aph, apl;
	u64 paddr;

	aph = readl(dev->base + PIPE_V1_REG_PARAMS_ADDR_HIGH);
	apl = readl(dev->base + PIPE_V1_REG_PARAMS_ADDR_LOW);

	paddr = ((u64)aph << 32) | apl;
	return paddr == (__pa(aps));
}

static int setup_access_params_addr(struct platform_device *pdev,
				    struct goldfish_pipe_dev *dev)
{
	u64 paddr;
	struct access_params *aps;

	aps = devm_kzalloc(&pdev->dev, sizeof(struct access_params),
			   GFP_KERNEL);
	if (!aps)
		return -ENOMEM;

	paddr = __pa(aps);
	writel((u32)(paddr >> 32), dev->base + PIPE_V1_REG_PARAMS_ADDR_HIGH);
	writel((u32)paddr, dev->base + PIPE_V1_REG_PARAMS_ADDR_LOW);

	if (valid_batchbuffer_addr(dev, aps)) {
		dev->aps = aps;
		return 0;
	}

	devm_kfree(&pdev->dev, aps);
	return -EFAULT;
}

static int access_with_param(struct goldfish_pipe_dev *dev, const int cmd,
			     unsigned long address, unsigned long avail,
			     struct goldfish_pipe *pipe, int *status)
{
	struct access_params *aps = dev->aps;

	if (!aps)
		return -EINVAL;

	aps->result = INITIAL_BATCH_RESULT;
	aps->channel = (unsigned long)pipe;
	aps->size = avail;
	aps->address = address;
	aps->cmd = cmd;
	writel(cmd, dev->base + PIPE_V1_REG_ACCESS_PARAMS);

	/*
	 * If the aps->result has not changed, that means
	 * that the batch command failed
	 */
	if (aps->result == INITIAL_BATCH_RESULT)
		return -EINVAL;

	*status = aps->result;
	return 0;
}

static int transfer_pages(struct goldfish_pipe_dev *dev,
			  struct goldfish_pipe *pipe,
			  int cmd,
			  unsigned long xaddr,
			  unsigned long size)
{
	unsigned long irq_flags;
	int status = 0;

	spin_lock_irqsave(&dev->lock, irq_flags);
	if (access_with_param(dev, cmd, xaddr, size, pipe, &status)) {
		gf_write_ptr(pipe, dev->base + PIPE_V1_REG_CHANNEL,
			     dev->base + PIPE_V1_REG_CHANNEL_HIGH);

		writel(size, dev->base + PIPE_V1_REG_SIZE);

		gf_write_ptr((void *)xaddr,
			     dev->base + PIPE_V1_REG_ADDRESS,
			     dev->base + PIPE_V1_REG_ADDRESS_HIGH);

		writel(cmd, dev->base + PIPE_V1_REG_COMMAND);

		status = readl(dev->base + PIPE_V1_REG_STATUS);
	}
	spin_unlock_irqrestore(&dev->lock, irq_flags);

	return status;
}

static unsigned long translate_address(const struct page *page,
				       unsigned long addr)
{
	return page_to_phys(page) | (addr & ~PAGE_MASK);
}

static ssize_t goldfish_pipe_read_write(struct file *filp, char __user *buffer,
					size_t bufflen, int is_write)
{
	struct goldfish_pipe *pipe = filp->private_data;
	struct goldfish_pipe_dev *dev = pipe->dev;
	unsigned long address;
	unsigned long address_end;
	const int wake_bit = is_write ? BIT_WAKE_ON_WRITE : BIT_WAKE_ON_READ;
	const int pipe_cmd = is_write ? PIPE_CMD_WRITE : PIPE_CMD_READ;
	int count = 0;
	int ret = -EINVAL;

	/* If the emulator already closed the pipe, no need to go further */
	if (test_bit(BIT_CLOSED_ON_HOST, &pipe->flags))
		return -EIO;

	/* Null reads or writes succeeds */
	if (unlikely(bufflen == 0))
		return 0;

	/* Check the buffer range for access */
	if (!access_ok(buffer, bufflen))
		return -EFAULT;

	address = (unsigned long)buffer;
	address_end = address + bufflen;

	/* Serialize access to the pipe */
	if (mutex_lock_interruptible(&pipe->lock))
		return -ERESTARTSYS;

	while (address < address_end) {
		struct page *pages[MAX_PAGES_TO_GRAB];
		unsigned long page_end = (address & PAGE_MASK) + PAGE_SIZE;
		unsigned long avail;
		unsigned long xaddr;
		unsigned long xaddr_prev;
		long first_page;
		long last_page;
		long requested_pages;
		int status;
		int n_pages;
		int page_i;
		int num_contiguous_pages;

		/*
		 * Attempt to grab multiple physically contiguous pages.
		 */
		first_page = address & PAGE_MASK;
		last_page = (address_end - 1) & PAGE_MASK;
		requested_pages =
			min(((last_page - first_page) >> PAGE_SHIFT) + 1,
			    (long)MAX_PAGES_TO_GRAB);

		ret = get_user_pages_fast(first_page, requested_pages,
					  !is_write, pages);
		if (ret < 0) {
			dev_err(dev->pdev_dev,
				"%s: get_user_pages_fast failed: %d\n",
				__func__, ret);
			break;
		} else if (!ret) {
			dev_err(dev->pdev_dev,
				"%s: error: no pages returned, requested %ld\n",
				__func__, requested_pages);
			break;
		}

		n_pages = ret;
		xaddr = translate_address(pages[0], address);
		xaddr_prev = xaddr;
		num_contiguous_pages = 1;
		for (page_i = 1; page_i < n_pages; page_i++) {
			unsigned long xaddr_i;

			xaddr_i = translate_address(pages[page_i], address);
			if (xaddr_i == xaddr_prev + PAGE_SIZE) {
				page_end += PAGE_SIZE;
				xaddr_prev = xaddr_i;
				num_contiguous_pages++;
			} else {
				dev_err(dev->pdev_dev,
					"%s: discontinuous page boundary: %d "
					"pages instead\n",
					__func__, page_i);
				break;
			}
		}
		avail = min(page_end, address_end) - address;

		status = transfer_pages(dev, pipe, pipe_cmd, xaddr, avail);

		for (page_i = 0; page_i < n_pages; page_i++) {
			if (status > 0 && !is_write &&
			    page_i < num_contiguous_pages)
				set_page_dirty(pages[page_i]);

			put_page(pages[page_i]);
		}

		if (status > 0) { /* Correct transfer */
			count += status;
			address += status;
			continue;
		} else if (status == 0) { /* EOF */
			ret = 0;
			break;
		} else if (status < 0 && count > 0) {
			/*
			 * An error occurred and we already transferred
			 * something on one of the previous pages.
			 * Just return what we already copied and log this
			 * err.
			 *
			 * Note: This seems like an incorrect approach but
			 * cannot change it until we check if any user space
			 * ABI relies on this behavior.
			 */
			if (status != PIPE_ERROR_AGAIN)
				dev_err_ratelimited(dev->pdev_dev,
					"backend returned error %d on %s\n",
					status, is_write ? "write" : "read");
			ret = 0;
			break;
		}

		/*
		 * If the error is not PIPE_ERROR_AGAIN, or if we are not in
		 * non-blocking mode, just return the error code.
		 */
		if (status != PIPE_ERROR_AGAIN ||
		    (filp->f_flags & O_NONBLOCK) != 0) {
			ret = goldfish_pipe_error_convert(status);
			break;
		}

		/*
		 * The backend blocked the read/write, wait until the backend
		 * tells us it's ready to process more data.
		 */
		set_bit(wake_bit, &pipe->flags);

		/* Tell the emulator we're going to wait for a wake event */
		goldfish_cmd(pipe, pipe_cmd);

		/* Unlock the pipe, then wait for the wake signal */
		mutex_unlock(&pipe->lock);

		while (test_bit(wake_bit, &pipe->flags)) {
			if (wait_event_interruptible(pipe->wake_queue,
					!test_bit(wake_bit, &pipe->flags)))
				return -ERESTARTSYS;

			if (test_bit(BIT_CLOSED_ON_HOST, &pipe->flags))
				return -EIO;
		}

		/* Try to re-acquire the lock */
		if (mutex_lock_interruptible(&pipe->lock))
			return -ERESTARTSYS;
	}
	mutex_unlock(&pipe->lock);

	return (ret < 0) ? ret : count;
}

static ssize_t goldfish_pipe_read(struct file *filp, char __user *buffer,
				  size_t bufflen, loff_t *ppos)
{
	return goldfish_pipe_read_write(filp, buffer, bufflen,
					/* is_write */ 0);
}

static ssize_t goldfish_pipe_write(struct file *filp,
				   const char __user *buffer, size_t bufflen,
				   loff_t *ppos)
{
	return goldfish_pipe_read_write(filp, (char __user *)buffer,
					bufflen, /* is_write */ 1);
}

static unsigned int goldfish_pipe_poll(struct file *filp, poll_table *wait)
{
	struct goldfish_pipe *pipe = filp->private_data;
	unsigned int mask = 0;
	int status;

	if (mutex_lock_interruptible(&pipe->lock))
		return -ERESTARTSYS;

	poll_wait(filp, &pipe->wake_queue, wait);

	status = goldfish_cmd_status(pipe, PIPE_CMD_POLL);

	mutex_unlock(&pipe->lock);

	if (status & PIPE_POLL_IN)
		mask |= POLLIN | POLLRDNORM;

	if (status & PIPE_POLL_OUT)
		mask |= POLLOUT | POLLWRNORM;

	if (status & PIPE_POLL_HUP)
		mask |= POLLHUP;

	if (test_bit(BIT_CLOSED_ON_HOST, &pipe->flags))
		mask |= POLLERR;

	return mask;
}

static irqreturn_t goldfish_pipe_interrupt(int irq, void *dev_id)
{
	struct goldfish_pipe_dev *dev = dev_id;
	unsigned long irq_flags;
	int count = 0;

	/*
	 * We're going to read from the emulator a list of (channel,flags)
	 * pairs corresponding to the wake events that occurred on each
	 * blocked pipe (i.e. channel).
	 */
	spin_lock_irqsave(&dev->lock, irq_flags);
	for (;;) {
		/* First read the channel, 0 means the end of the list */
		struct goldfish_pipe *pipe;
		unsigned long wakes;
		unsigned long channel = 0;

#ifdef CONFIG_64BIT
		channel =
			(u64)readl(dev->base + PIPE_V1_REG_CHANNEL_HIGH) << 32;
#endif
		channel |= readl(dev->base + PIPE_V1_REG_CHANNEL);
		if (!channel)
			break;

		/* Convert channel to struct pipe pointer + read wake flags */
		wakes = readl(dev->base + PIPE_V1_REG_WAKES);
		pipe  = (struct goldfish_pipe *)(ptrdiff_t)channel;

		/* Did the emulator just closed a pipe? */
		if (wakes & PIPE_WAKE_CLOSED) {
			set_bit(BIT_CLOSED_ON_HOST, &pipe->flags);
			wakes |= PIPE_WAKE_READ | PIPE_WAKE_WRITE;
		}
		if (wakes & PIPE_WAKE_READ)
			clear_bit(BIT_WAKE_ON_READ, &pipe->flags);
		if (wakes & PIPE_WAKE_WRITE)
			clear_bit(BIT_WAKE_ON_WRITE, &pipe->flags);

		wake_up_interruptible(&pipe->wake_queue);
		count++;
	}
	spin_unlock_irqrestore(&dev->lock, irq_flags);

	return (count == 0) ? IRQ_NONE : IRQ_HANDLED;
}

/* A helper function to get the instance of goldfish_pipe_dev from file */
static struct goldfish_pipe_dev *to_goldfish_pipe_dev(struct file *file)
{
	struct miscdevice *miscdev = file->private_data;

	return container_of(miscdev, struct goldfish_pipe_dev, miscdev);
}

/**
 *	goldfish_pipe_open - open a channel to the AVD
 *	@inode: inode of device
 *	@file: file struct of opener
 *
 *	Create a new pipe link between the emulator and the use application.
 *	Each new request produces a new pipe.
 *
 *	Note: we use the pipe ID as a mux. All goldfish emulations are 32bit
 *	right now so this is fine. A move to 64bit will need this addressing
 */
static int goldfish_pipe_open(struct inode *inode, struct file *file)
{
	struct goldfish_pipe_dev *dev = to_goldfish_pipe_dev(file);
	struct goldfish_pipe *pipe;
	int status;

	/* Allocate new pipe kernel object */
	pipe = kzalloc(sizeof(*pipe), GFP_KERNEL);
	if (!pipe)
		return -ENOMEM;

	pipe->dev = dev;
	init_waitqueue_head(&pipe->wake_queue);
	mutex_init(&pipe->lock);

	/*
	 * Now, tell the emulator we're opening a new pipe. We use the
	 * pipe object's address as the channel identifier for simplicity.
	 */

	status = goldfish_cmd_status(pipe, PIPE_CMD_OPEN);
	if (status < 0) {
		kfree(pipe);
		return status;
	}

	/* All is done, save the pipe into the file's private data field */
	file->private_data = pipe;
	return 0;
}

static int goldfish_pipe_release(struct inode *inode, struct file *filp)
{
	struct goldfish_pipe *pipe = filp->private_data;

	pr_debug("%s: call. pipe=%p file=%p\n", __func__, pipe, filp);
	/* The guest is closing the channel, so tell the emulator right now */
	goldfish_cmd(pipe, PIPE_CMD_CLOSE);
	kfree(pipe);
	filp->private_data = NULL;
	return 0;
}

static const struct file_operations goldfish_pipe_fops = {
	.owner = THIS_MODULE,
	.read = goldfish_pipe_read,
	.write = goldfish_pipe_write,
	.poll = goldfish_pipe_poll,
	.open = goldfish_pipe_open,
	.release = goldfish_pipe_release,
};

static void init_miscdevice(struct miscdevice *miscdev)
{
	memset(miscdev, 0, sizeof(*miscdev));

	miscdev->minor = MISC_DYNAMIC_MINOR;
	miscdev->name = DEVICE_NAME;
	miscdev->fops = &goldfish_pipe_fops;
};

static int goldfish_pipe_device_deinit(void *raw_dev,
				       struct platform_device *pdev);

int goldfish_pipe_device_v1_init(struct platform_device *pdev,
				 void __iomem *base,
				 int irq)
{
	struct goldfish_pipe_dev *dev;
	int err;

	pr_info("%s--start\n", __FUNCTION__);
	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->super.deinit = &goldfish_pipe_device_deinit;
	dev->pdev_dev = &pdev->dev;
	spin_lock_init(&dev->lock);

	err = devm_request_irq(&pdev->dev, irq,
			       &goldfish_pipe_interrupt, IRQF_SHARED,
			       DEVICE_NAME, dev);
	if (err) {
		dev_err(&pdev->dev, "unable to allocate IRQ for v1\n");
		return err;
	}

	init_miscdevice(&dev->miscdev);
	err = misc_register(&dev->miscdev);
	if (err) {
		dev_err(&pdev->dev, "unable to register v1 device\n");
		return err;
	}

	setup_access_params_addr(pdev, dev);

	platform_set_drvdata(pdev, dev);
	return 0;
}

static int goldfish_pipe_device_deinit(void *raw_dev,
				       struct platform_device *pdev)
{
	struct goldfish_pipe_dev *dev = raw_dev;

	misc_deregister(&dev->miscdev);
	return 0;
}
