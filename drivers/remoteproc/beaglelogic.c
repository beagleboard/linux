/*
 * Kernel module for BeagleLogic - a logic analyzer for the BeagleBone [Black]
 * Designed to be used in conjunction with a modified pru_rproc driver
 *
 * Copyright (C) 2014 Kumar Abhishek <abhishek@theembeddedkitchen.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include <asm/atomic.h>
#include <asm/uaccess.h>

#include <linux/module.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/wait.h>

#include <linux/platform_device.h>
#include <linux/miscdevice.h>

#include <linux/io.h>
#include <linux/irqreturn.h>
#include <linux/slab.h>
#include <linux/genalloc.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>

#include <linux/kobject.h>
#include <linux/string.h>

#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_device.h>

#include <linux/sysfs.h>
#include <linux/fs.h>

#include "beaglelogic.h"
#include "beaglelogic_glue.h"

/* Buffer states */
enum bufstates {
	STATE_BL_BUF_ALLOC,
	STATE_BL_BUF_MAPPED,
	STATE_BL_BUF_UNMAPPED,
	STATE_BL_BUF_DROPPED
};

/* PRU Downcall API */
#define BL_DC_GET_VERSION	0   /* Firmware */
#define BL_DC_GET_MAX_SG	1   /* Get the Max number of SG entries */
#define BL_DC_GET_CXT_PTR	2   /* Get the context pointer */
#define BL_DC_SM_RATE		3   /* Get/set rate = (200 / n) MHz, n = 2... */
#define BL_DC_SM_TRIGGER	4   /* RFU */
#define BL_DC_SM_ARM		7   /* Arm the LA (start sampling) */

/* PRU-side sample buffer descriptor */
typedef struct prusamplebuf {
	u32 dma_start_addr;
	u32 dma_end_addr;
} buflist;

/* Shared structure containing PRU attributes */
typedef struct capture_context {
	/* Firmware context structure magic bytes */
#define BL_FW_MAGIC	0xBEA61E10
	u32 magic;
	u32 errorCode;

	u32 interrupt1count;

	buflist list_head; /* This is really an array on the PRU side */
} ccontext;

/* Forward declration */
static const struct file_operations pru_beaglelogic_fops;

/* Buffers are arranged as an array but are
 * also circularly linked to simplify reads */
typedef struct databuf logic_buffer;
typedef struct databuf {
	void *buf;
	dma_addr_t phys_addr;
	size_t size;

	unsigned short state;
	unsigned short index;

	logic_buffer *next;
} logic_buffer;

struct beaglelogicdev {
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

	/* Private data */
	struct device *p_dev; /* Parent platform device */

	/* Locks */
	struct mutex mutex;

	/* Buffer management */
	logic_buffer *buffers;
	logic_buffer *lastbufready;
	logic_buffer *bufbeingread;
	u32 bufcount;
	wait_queue_head_t wait;

	/* ISR Bookkeeping */
	u32 previntcount;	/* Previous interrupt count read from PRU */

	/* Firmware capabilities */
	ccontext *cxt_pru;

	/* Device capabilities */
	u32 bufunitsize;  	/* Size of 1 Allocation unit */
	u32 maxbufcount;	/* Max buffer count supported by the PRU FW */
	u32 samplerate; 	/* Sample rate = 100 / n MHz, n = 2+ (int) */
	u32 triggerflags;	/* bit 0 : 1shot/!continuous */
	u32 sampleunit; 	/* 0:8bits, 1:16bits */

	/* State */
	u32 state;
	u32 lasterror;
};

typedef struct bufreader {
	struct beaglelogicdev *bldev;
	logic_buffer *buf;

	u32 pos;
	u32 remaining;
} logic_buffer_reader;

#define to_beaglelogicdev(dev)	container_of((dev), \
		struct beaglelogicdev, miscdev)

#define DRV_NAME	"beaglelogic"
#define DRV_VERSION	"1.0"

/* Begin Buffer Management section */
static int bufunitsize = 4 * 1024 * 1024;
module_param(bufunitsize, int, S_IRUGO);
MODULE_PARM_DESC(bufunitsize, " Size of each buffer unit [default 4 MB]");

/* Allocate DMA buffers for the PRU
 * This method acquires & releases the device mutex */
static int beaglelogic_memalloc(struct device *dev, u32 bufsize)
{
	struct beaglelogicdev *bldev = dev_get_drvdata(dev);
	int i, cnt;
	void *buf;

	/* Check if BL is in use */
	if (!mutex_trylock(&bldev->mutex))
		return -EBUSY;

	/* Compute no. of buffers to allocate, round up
	 * We need at least two buffers for ping-pong action */
	cnt = max(DIV_ROUND_UP(bufsize, bldev->bufunitsize), (u32)2);

	/* Too large? */
	if (cnt > bldev->maxbufcount) {
		dev_err(dev, "Not enough memory\n");
		return -ENOMEM;
	}

	bldev->bufcount = cnt;

	/* Allocate buffer list */
	bldev->buffers = devm_kzalloc(dev, sizeof(logic_buffer) * (cnt),
			GFP_KERNEL);
	if (!bldev->buffers)
		goto failnomem;

	/* Allocate DMA buffers */
	for (i = 0; i < cnt; i++) {
		buf = kmalloc(bldev->bufunitsize, GFP_KERNEL);
		if (!buf)
			goto failrelease;

		/* Fill with 0xFF */
		memset(buf, 0xFF, bldev->bufunitsize);

		/* Set the buffers */
		bldev->buffers[i].buf = buf;
		bldev->buffers[i].phys_addr = virt_to_phys(buf);
		bldev->buffers[i].size = bldev->bufunitsize;
		bldev->buffers[i].index = i;

		/* Circularly link the buffers */
		bldev->buffers[i].next = &bldev->buffers[(i + 1) % cnt];
	}

	/* Write log and unlock */
	dev_info(dev, "Successfully allocated %d bytes of memory.\n",
			cnt * bldev->bufunitsize);

	mutex_unlock(&bldev->mutex);

	/* Done */
	return 0;
failrelease:
	for (i = 0; i < cnt; i++) {
		if (bldev->buffers[i].buf)
			kfree(bldev->buffers[i].buf);
	}
	devm_kfree(dev, bldev->buffers);
	dev_err(dev, "Sample buffer allocation:");
failnomem:
	dev_err(dev, "Not enough memory\n");
	mutex_unlock(&bldev->mutex);
	return -ENOMEM;
}

/* Frees the DMA buffers and the bufferlist */
static void beaglelogic_memfree(struct device *dev)
{
	struct beaglelogicdev *bldev = dev_get_drvdata(dev);
	int i;

	mutex_lock(&bldev->mutex);
	for (i = 0; i < bldev->bufcount; i++)
		kfree(bldev->buffers[i].buf);

	if (bldev->buffers)
		devm_kfree(dev, bldev->buffers);
	mutex_unlock(&bldev->mutex);
}

/* No argument checking for the map/unmap functions */
static int beaglelogic_map_buffer(struct device *dev, logic_buffer *buf)
{
	dma_addr_t dma_addr;

	/* If already mapped, do nothing */
	if (buf->state == STATE_BL_BUF_MAPPED)
		return 0;

	dma_addr = dma_map_single(dev, buf->buf, buf->size, DMA_FROM_DEVICE);
	if (dma_mapping_error(dev, dma_addr))
		goto fail;
	else {
		buf->phys_addr = dma_addr;
		buf->state = STATE_BL_BUF_MAPPED;
	}

	return 0;
fail:
	dev_err(dev, "DMA Mapping error. \n");
	return -1;
}

static void beaglelogic_unmap_buffer(struct device *dev, logic_buffer *buf)
{
	dma_unmap_single(dev, buf->phys_addr, buf->size, DMA_FROM_DEVICE);
	buf->state = STATE_BL_BUF_UNMAPPED;
}

/* Fill the sample buffer with a pattern of increasing 32-bit ints
 * This can be studied to watch out for dropped bytes/buffers */
static void beaglelogic_fill_buffer_testpattern(struct device *dev)
{
	struct beaglelogicdev *bldev = dev_get_drvdata(dev);
	int i, j;
	u32 cnt = 0, *addr;

	mutex_lock(&bldev->mutex);
	for (i = 0; i < bldev->bufcount; i++) {
		addr = bldev->buffers[i].buf;

		for (j = 0; j < bldev->buffers[i].size / sizeof(cnt); j++)
			*addr++ = cnt++;
	}
	mutex_unlock(&bldev->mutex);
}

/* Map all the buffers. This is done just before beginning a sample operation
 * NOTE: PRUs are halted at this time */
static int beaglelogic_map_and_submit_all_buffers(struct device *dev)
{
	struct beaglelogicdev *bldev = dev_get_drvdata(dev);
	buflist *pru_buflist = &bldev->cxt_pru->list_head;
	int i, j;
	dma_addr_t addr;

	if (!pru_buflist)
		return -1;

	for (i = 0; i < bldev->bufcount;i++) {
		if (beaglelogic_map_buffer(dev, &bldev->buffers[i]))
			goto fail;
	}

	/* Write buffer table to the PRU memory, and null terminate */
	for (i = 0; i < bldev->bufcount; i++) {
		addr = bldev->buffers[i].phys_addr;
		pru_buflist[i].dma_start_addr = addr;
		pru_buflist[i].dma_end_addr = addr + bldev->buffers[i].size;
	}
	pru_buflist[i].dma_start_addr = 0;
	pru_buflist[i].dma_end_addr = 0;

	/* Update state to ready */
	if (i)
		bldev->state = STATE_BL_ARMED;

	return 0;
fail:
	/* Unmap the buffers */
	for (j = 0; j < i; j++)
		beaglelogic_unmap_buffer(dev, &bldev->buffers[i]);

	dev_err(dev, "DMA Mapping failed at i=%d\n", i);

	bldev->state = STATE_BL_ERROR;
	return 1;
}

/* End Buffer Management section */

/* Begin Device Attributes Configuration Section
 * All set operations lock and unlock the device mutex */

u32 beaglelogic_get_samplerate(struct device *dev)
{
	struct beaglelogicdev *bldev = dev_get_drvdata(dev);
	return bldev->samplerate;
}

int beaglelogic_set_samplerate(struct device *dev, u32 samplerate)
{
	struct beaglelogicdev *bldev = dev_get_drvdata(dev);
	if (samplerate > bldev->coreclockfreq / 2 || samplerate < 100000)
		return -EINVAL;

	if (mutex_trylock(&bldev->mutex)) {
		/* Get sample rate nearest to divisor */
		bldev->samplerate = (bldev->coreclockfreq / 2) /
				((bldev->coreclockfreq / 2)/ samplerate);
		mutex_unlock(&bldev->mutex);
		return 0;
	}
	return -EBUSY;
}

u32 beaglelogic_get_sampleunit(struct device *dev)
{
	struct beaglelogicdev *bldev = dev_get_drvdata(dev);
	return bldev->sampleunit;
}

int beaglelogic_set_sampleunit(struct device *dev, u32 sampleunit)
{
	struct beaglelogicdev *bldev = dev_get_drvdata(dev);
	if (sampleunit > 2)
		return -EINVAL;

	if (mutex_trylock(&bldev->mutex)) {
		bldev->sampleunit = sampleunit;
		mutex_unlock(&bldev->mutex);

		return 0;
	}
	return -EBUSY;
}

u32 beaglelogic_get_triggerflags(struct device *dev)
{
	struct beaglelogicdev *bldev = dev_get_drvdata(dev);
	return bldev->triggerflags;
}

int beaglelogic_set_triggerflags(struct device *dev, u32 triggerflags)
{
	struct beaglelogicdev *bldev = dev_get_drvdata(dev);
	if (triggerflags > 1)
		return -EINVAL;

	if (mutex_trylock(&bldev->mutex)) {
		bldev->triggerflags = triggerflags;
		mutex_unlock(&bldev->mutex);

		return 0;
	}
	return -EBUSY;
}

/* End Device Attributes Configuration Section */

/* This is [to be] called from a threaded IRQ handler */
int beaglelogic_serve_irq(int irqno, void *data)
{
	struct beaglelogicdev *bldev = data;
	struct device *dev = bldev->miscdev.this_device;
	u32 state = bldev->state;

	dev_dbg(dev, "Beaglelogic IRQ #%d\n", irqno);
	if (irqno == BL_IRQ_BUFREADY) {
		/* Manage the buffers */
		beaglelogic_unmap_buffer(dev,
			bldev->lastbufready = bldev->bufbeingread);

		/* Avoid a false buffer overrun warning on the last run */
		if (bldev->triggerflags != BL_TRIGGERFLAGS_ONESHOT ||
			bldev->bufbeingread->next->index != 0) {
			beaglelogic_map_buffer(dev,
				bldev->bufbeingread = bldev->bufbeingread->next);
		}
		wake_up_interruptible(&bldev->wait);
	} else if (irqno == BL_IRQ_CLEANUP) {
		/* This interrupt occurs twice:
		 *  1. After a successful configuration of PRU capture
		 *  2. After the last buffer transferred  */
		state = bldev->state;
		if (state <= STATE_BL_ARMED) {
			dev_dbg(dev, "config written, BeagleLogic ready\n");
			return IRQ_HANDLED;
		}
		else if (state != STATE_BL_REQUEST_STOP &&
				state != STATE_BL_RUNNING) {
			dev_err(dev, "Unexpected stop request \n");
			bldev->state = STATE_BL_ERROR;
			return IRQ_HANDLED;
		}
		bldev->state = STATE_BL_INITIALIZED;
		wake_up_interruptible(&bldev->wait);
	}

	return IRQ_HANDLED;
}

/* Write configuration into the PRU [via downcall] (assume mutex is held) */
int beaglelogic_write_configuration(struct device *dev)
{
	struct beaglelogicdev *bldev = dev_get_drvdata(dev);
	int i;

	/* Do a downcall and hand over the settings */
	i = bldev->downcall_idx(0, BL_DC_SM_TRIGGER,
			(bldev->coreclockfreq / 2) / bldev->samplerate,
			bldev->sampleunit, bldev->triggerflags, 0, 0);

	dev_dbg(dev, "PRU Config written, err code = %d\n", i);
	return 0;
}

/* Begin the sampling operation [This takes the mutex] */
int beaglelogic_start(struct device *dev)
{
	struct beaglelogicdev *bldev = dev_get_drvdata(dev);

	/* This mutex will be locked for the entire duration BeagleLogic runs */
	mutex_lock(&bldev->mutex);
	if (beaglelogic_write_configuration(dev)) {
		mutex_unlock(&bldev->mutex);
		return -1;
	}
	bldev->bufbeingread = &bldev->buffers[0];
	bldev->pru_start(0);
	bldev->downcall_idx(0, BL_DC_SM_ARM, 0, 0, 0, 0, 0);

	/* All set now. Start the PRUs and wait for IRQs */
	bldev->state = STATE_BL_RUNNING;
	bldev->lasterror = 0;

	dev_info(dev, "capture started with sample rate=%d Hz, sampleunit=%d, "\
			"triggerflags=%d",
			bldev->samplerate,
			bldev->sampleunit,
			bldev->triggerflags);
	return 0;
}

/* Request stop. Stop will effect only after the last buffer is written out */
void beaglelogic_stop(struct device *dev)
{
	struct beaglelogicdev *bldev = dev_get_drvdata(dev);

	if (mutex_is_locked(&bldev->mutex)) {
		bldev->pru_request_stop();
		bldev->state = STATE_BL_REQUEST_STOP;

		/* Wait for the PRU to signal completion */
		wait_event_interruptible_timeout(bldev->wait,
				bldev->state == STATE_BL_INITIALIZED, HZ / 100);

		/* Release */
		mutex_unlock(&bldev->mutex);

		dev_info(dev, "capture session ended\n");
	}
}

/* fops */
static int beaglelogic_f_open(struct inode *inode, struct file *filp)
{
	logic_buffer_reader *reader;
	struct beaglelogicdev *bldev = to_beaglelogicdev(filp->private_data);
	struct device *dev = bldev->miscdev.this_device;

	if (bldev->bufcount == 0)
		return -ENOMEM;

	reader = devm_kzalloc(dev, sizeof(*reader), GFP_KERNEL);
	reader->bldev = bldev;
	reader->buf = NULL;
	reader->pos = 0;
	reader->remaining = 0;

	filp->private_data = reader;

	/* Map and submit all the buffers */
	if (beaglelogic_map_and_submit_all_buffers(dev))
		return -ENOMEM;

	return 0;
}

/* Read the sample (ring) buffer. TODO Implement Nonblock */
ssize_t beaglelogic_f_read (struct file *filp, char __user *buf,
                          size_t sz, loff_t *offset)
{
	int count;
	logic_buffer_reader *reader = filp->private_data;
	struct beaglelogicdev *bldev = reader->bldev;
	struct device *dev = bldev->miscdev.this_device;

	if (bldev->state == STATE_BL_ERROR)
		return -EIO;

	if (reader->remaining > 0)
		goto perform_copy;

	if (reader->buf) {
		if (bldev->state == STATE_BL_INITIALIZED &&
				bldev->lastbufready == reader->buf)
			return 0;

		reader->buf = reader->buf->next;
	}
	else {
		/* (re)trigger */
		if (beaglelogic_start(dev))
			return -ENOEXEC;

		reader->buf = &bldev->buffers[0];
	}
	reader->pos = 0;
	reader->remaining = reader->buf->size;

	dev_dbg(dev, "waiting for IRQ\n");
	wait_event_interruptible(bldev->wait,
			reader->buf->state == STATE_BL_BUF_UNMAPPED);
	dev_dbg(dev, "got IRQ\n");
perform_copy:
	count = min(reader->remaining, sz);

	/* Detect buffer drop */
	if (reader->buf->state == STATE_BL_BUF_MAPPED) {
		dev_warn(dev, "buffer dropped at index %d \n",
				reader->buf->index);
		reader->buf->state = STATE_BL_BUF_DROPPED;
		bldev->lasterror = 0x10000 | reader->buf->index;
	}

	if (copy_to_user(buf, reader->buf->buf + reader->pos, count))
		return -EFAULT;

	reader->pos += count;
	reader->remaining -= count;

	return count;
}

/* Map the PRU buffers to user space [cache coherency managed by driver] */
int beaglelogic_f_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int i, ret;
	logic_buffer_reader *reader = filp->private_data;
	struct beaglelogicdev *bldev = reader->bldev;

	unsigned long addr = vma->vm_start;

	if (vma->vm_end - vma->vm_start > bldev->bufunitsize * bldev->bufcount)
		return -EINVAL;

	for (i = 0; i < bldev->bufcount; i++) {
		ret = remap_pfn_range(vma, addr,
				(bldev->buffers[i].phys_addr) >> PAGE_SHIFT,
				bldev->buffers[i].size,
				vma->vm_page_prot);

		if (ret)
			return -EINVAL;

		addr += bldev->buffers[i].size;
	}
	return 0;
}

/* Configuration through ioctl */
static long beaglelogic_f_ioctl(struct file *filp, unsigned int cmd,
		  unsigned long arg)
{
	logic_buffer_reader *reader = filp->private_data;
	struct beaglelogicdev *bldev = reader->bldev;
	struct device *dev = bldev->miscdev.this_device;

	u32 val;

	dev_info(dev, "BeagleLogic: IOCTL called cmd = %08X, "\
			"arg = %08lX\n", cmd, arg);

	switch (cmd) {
		case IOCTL_BL_GET_VERSION:
			return 0;

		case IOCTL_BL_GET_SAMPLE_RATE:
			if (copy_to_user((void * __user)arg,
					&bldev->samplerate,
					sizeof(bldev->samplerate)))
				return -EFAULT;
			return 0;

		case IOCTL_BL_SET_SAMPLE_RATE:
			if (beaglelogic_set_samplerate(dev, (u32)arg))
				return -EFAULT;
			return 0;

		case IOCTL_BL_GET_SAMPLE_UNIT:
			if (copy_to_user((void * __user)arg,
					&bldev->sampleunit,
					sizeof(bldev->sampleunit)))
				return -EFAULT;
			return 0;

		case IOCTL_BL_SET_SAMPLE_UNIT:
			if (beaglelogic_set_sampleunit(dev, (u32)arg))
				return -EFAULT;
			return 0;

		case IOCTL_BL_GET_TRIGGER_FLAGS:
			if (copy_to_user((void * __user)arg,
					&bldev->triggerflags,
					sizeof(bldev->triggerflags)))
				return -EFAULT;
			return 0;

		case IOCTL_BL_SET_TRIGGER_FLAGS:
			if (beaglelogic_set_triggerflags(dev, (u32)arg))
				return -EFAULT;
			return 0;

		case IOCTL_BL_GET_CUR_INDEX:
			if (copy_to_user((void * __user)arg,
					&bldev->bufbeingread->index,
					sizeof(bldev->bufbeingread->index)))
				return -EFAULT;
			return 0;

		case IOCTL_BL_CACHE_INVALIDATE:
			for (val = 0; val < bldev->bufcount; val++) {
				beaglelogic_unmap_buffer(dev,
						&bldev->buffers[val]);
			}
			return 0;

		case IOCTL_BL_GET_BUFFER_SIZE:
			val = bldev->bufunitsize * bldev->bufcount;
			if (copy_to_user((void * __user)arg,
					&val,
					sizeof(val)))
				return -EFAULT;
			return 0;

		case IOCTL_BL_SET_BUFFER_SIZE:
			beaglelogic_memfree(dev);
			val = beaglelogic_memalloc(dev, arg);
			if (!val)
				return beaglelogic_map_and_submit_all_buffers(dev);
			return 0;

		case IOCTL_BL_GET_BUFUNIT_SIZE:
			if (copy_to_user((void * __user)arg,
					&bldev->bufunitsize,
					sizeof(bldev->bufunitsize)))
				return -EFAULT;
			return 0;

		case IOCTL_BL_FILL_TEST_PATTERN:
			beaglelogic_fill_buffer_testpattern(dev);
			return 0;

		case IOCTL_BL_START:
			beaglelogic_start(dev);
			return 0;

		case IOCTL_BL_STOP:
			beaglelogic_stop(dev);
			return 0;

	}
	return -ENOTTY;
}

/* llseek to offset zero resets the LA */
static loff_t beaglelogic_f_llseek(struct file *filp, loff_t offset, int whence)
{
	logic_buffer_reader *reader = filp->private_data;
	struct device *dev = reader->bldev->miscdev.this_device;
	if (whence == SEEK_SET && offset == 0) {
		/* The next read triggers the LA */
		reader->buf = NULL;
		reader->pos = 0;
		reader->remaining = 0;

		/* Stop and map the first buffer */
		beaglelogic_stop(dev);
		beaglelogic_map_buffer(dev, &reader->bldev->buffers[0]);
	}
	return -EINVAL;
}

/* Device file close handler */
static int beaglelogic_f_release(struct inode *inode, struct file *filp)
{
	logic_buffer_reader *reader = filp->private_data;
	struct beaglelogicdev *bldev = reader->bldev;
	struct device *dev = bldev->miscdev.this_device;

	/* Stop & Release */
	beaglelogic_stop(dev);
	devm_kfree(dev, reader);

	return 0;
}

/* File operations struct */
static const struct file_operations pru_beaglelogic_fops = {
	.owner = THIS_MODULE,
	.open = beaglelogic_f_open,
	.unlocked_ioctl = beaglelogic_f_ioctl,
	.read = beaglelogic_f_read,
	.llseek = beaglelogic_f_llseek,
	.mmap = beaglelogic_f_mmap,
	.release = beaglelogic_f_release,
};
/* fops */

/* begin sysfs attrs */
static ssize_t bl_memalloc_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	struct beaglelogicdev *bldev = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			bldev->bufcount * bldev->bufunitsize);
}

static ssize_t bl_memalloc_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	struct beaglelogicdev *bldev = dev_get_drvdata(dev);
	u32 val;
	int ret;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	/* Check value of memory to reserve */
	if (val > bldev->maxbufcount * bldev->bufunitsize)
		return -EINVAL;

	/* Free buffers and reallocate */
	beaglelogic_memfree(dev);
	ret = beaglelogic_memalloc(dev, val);

	if (!ret && val)
		beaglelogic_map_and_submit_all_buffers(dev);

	return count;
}

static ssize_t bl_samplerate_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n",
			beaglelogic_get_samplerate(dev));
}

static ssize_t bl_samplerate_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	/* Check value of sample rate - 100 kHz to 100MHz */
	if (beaglelogic_set_samplerate(dev, val))
		return -EINVAL;

	return count;
}

static ssize_t bl_sampleunit_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u32 ret = beaglelogic_get_sampleunit(dev);
	int cnt = scnprintf(buf, PAGE_SIZE, "%d:", ret);

	switch (ret)
	{
		case 0:
			cnt += scnprintf(buf, PAGE_SIZE, "8bits,norle\n");
			break;

		case 1:
			cnt += scnprintf(buf, PAGE_SIZE, "16bit,norle\n");
			break;
	}
	return cnt;
}

static ssize_t bl_sampleunit_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int err;
	u32 val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	/* Check value of sample unit - only 0 or 1 currently */
	if ((err = beaglelogic_set_sampleunit(dev, val)))
		return err;

	return count;
}

static ssize_t bl_triggerflags_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	switch (beaglelogic_get_triggerflags(dev)) {
		case BL_TRIGGERFLAGS_ONESHOT:
			return scnprintf(buf, PAGE_SIZE, "0:oneshot\n");

		case BL_TRIGGERFLAGS_CONTINUOUS:
			return scnprintf(buf, PAGE_SIZE, "1:continuous\n");
	}
	return 0;
}

static ssize_t bl_triggerflags_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	int err;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	if ((err = beaglelogic_set_triggerflags(dev, val)))
		return err;

	return count;
}

static ssize_t bl_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct beaglelogicdev *bldev = dev_get_drvdata(dev);
	u32 state = bldev->state;
	logic_buffer *buffer = bldev->bufbeingread;

	if (state == STATE_BL_RUNNING) {
		/* State blocks and returns last buffer read */
		wait_event_interruptible(bldev->wait,
				buffer->state == STATE_BL_BUF_UNMAPPED);
		return scnprintf(buf, PAGE_SIZE, "%d\n", buffer->index);
	}

	/* Identify non-buffer debug states with a -ve value */
	return scnprintf(buf, PAGE_SIZE, "-%d\n", -bldev->state);
}

static ssize_t bl_state_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	/* State going to 1 starts the sampling operation, 0 aborts*/
	if (val > 1)
		return -EINVAL;

	if (val == 1)
		beaglelogic_start(dev);
	else
		beaglelogic_stop(dev);

	return count;
}

static ssize_t bl_buffers_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct beaglelogicdev *bldev = dev_get_drvdata(dev);
	int i, c, cnt;

	for (i = 0, c = 0, cnt = 0; i < bldev->bufcount; i++) {
		c = scnprintf(buf, PAGE_SIZE, "%08x,%u\n",
				(u32)bldev->buffers[i].phys_addr,
				bldev->buffers[i].size);
		cnt += c;
		buf += c;
	}

	return cnt;
}

static ssize_t bl_lasterror_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct beaglelogicdev *bldev = dev_get_drvdata(dev);

	wait_event_interruptible(bldev->wait,
			bldev->state != STATE_BL_RUNNING);


	return scnprintf(buf, PAGE_SIZE, "%d\n", bldev->lasterror);
}

static ssize_t bl_testpattern_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	/* Only if we get the magic number, trigger the test pattern */
	if (val == 12345678)
		beaglelogic_fill_buffer_testpattern(dev);

	return count;
}

static DEVICE_ATTR(memalloc, S_IWUSR | S_IRUGO,
		bl_memalloc_show, bl_memalloc_store);

static DEVICE_ATTR(samplerate, S_IWUSR | S_IRUGO,
		bl_samplerate_show, bl_samplerate_store);

static DEVICE_ATTR(sampleunit, S_IWUSR | S_IRUGO,
		bl_sampleunit_show, bl_sampleunit_store);

static DEVICE_ATTR(triggerflags, S_IWUSR | S_IRUGO,
		bl_triggerflags_show, bl_triggerflags_store);

static DEVICE_ATTR(state, S_IWUSR | S_IRUGO,
		bl_state_show, bl_state_store);

static DEVICE_ATTR(buffers, S_IRUGO,
		bl_buffers_show, NULL);

static DEVICE_ATTR(lasterror, S_IRUGO,
		bl_lasterror_show, NULL);

static DEVICE_ATTR(filltestpattern, S_IWUSR,
		NULL, bl_testpattern_store);

static struct attribute *beaglelogic_attributes[] = {
	&dev_attr_memalloc.attr,
	&dev_attr_samplerate.attr,
	&dev_attr_sampleunit.attr,
	&dev_attr_triggerflags.attr,
	&dev_attr_state.attr,
	&dev_attr_buffers.attr,
	&dev_attr_lasterror.attr,
	&dev_attr_filltestpattern.attr,
	NULL
};

static struct attribute_group beaglelogic_attr_group = {
	.attrs = beaglelogic_attributes
};
/* end sysfs attrs */

static int beaglelogic_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	int err, ret;
	struct beaglelogicdev *bldev;
	struct device *dev;
	u32 val;

	printk("BeagleLogic loaded and initializing\n");

	/* Allocate memory for our private structure */
	bldev = kzalloc(sizeof(*bldev), GFP_KERNEL);
	if (!bldev)
		goto fail;

	bldev->miscdev.fops = &pru_beaglelogic_fops;
	bldev->miscdev.minor = MISC_DYNAMIC_MINOR;
	bldev->miscdev.mode = S_IRUGO;
	bldev->miscdev.name = "beaglelogic";

	/* Export the IRQ handler */
	bldev->serve_irq = beaglelogic_serve_irq;

	/* Link the platform device data to our private structure */
	bldev->p_dev = &pdev->dev;
	dev_set_drvdata(bldev->p_dev, bldev);

	/* Bind to the pru_rproc module */
	err = pruproc_beaglelogic_request_bind((void *)bldev);
	if (err)
		goto fail;

	/* Once done, register our misc device and link our private data */
	err = misc_register(&bldev->miscdev);
	if (err)
		goto fail;
	dev = bldev->miscdev.this_device;
	dev_set_drvdata(dev, bldev);

	/* Set up locks */
	mutex_init(&bldev->mutex);
	init_waitqueue_head(&bldev->wait);

	/* Power on in disabled state */
	bldev->state = STATE_BL_DISABLED;

	/* Get firmware properties */
	ret = bldev->downcall_idx(0, BL_DC_GET_VERSION, 0, 0, 0, 0, 0);
	if (ret != 0) {
		dev_info(dev, "BeagleLogic PRU Firmware version: %d.%d\n",
				ret >> 8, ret & 0xFF);
	} else {
		dev_err(dev, "Firmware error!\n");
		goto faildereg;
	}

	ret = bldev->downcall_idx(0, BL_DC_GET_MAX_SG, 0, 0, 0, 0, 0);
	if (ret > 0 && ret < 256) { /* Let's be reasonable here */
		dev_info(dev, "Device supports max %d vector transfers\n", ret);
		bldev->maxbufcount = ret;
	} else {
		dev_err(dev, "Firmware error!\n");
		goto faildereg;
	}

	ret = bldev->downcall_idx(0, BL_DC_GET_CXT_PTR, 0, 0, 0, 0, 0);
	bldev->cxt_pru = bldev->d_da_to_va(0, ret);

	if (!bldev->cxt_pru) {
		dev_err(dev, "BeagleLogic: Unable to access PRU SRAM.\n");
		goto faildereg;
	}

	/* To be removed in the next iteration */
	if (bldev->cxt_pru->magic == BL_FW_MAGIC)
		dev_info(dev, "Valid PRU capture context structure "\
				"found at offset %04X\n", ret);
	else {
		dev_err(dev, "Firmware error!\n");
		goto faildereg;
	}


	/* Apply default configuration first */
	bldev->samplerate = 100 * 1000 * 1000;
	bldev->sampleunit = 1;
	bldev->bufunitsize = 4 * 1024 * 1024;
	bldev->triggerflags = 0;

	/* Override defaults with the device tree */
	if (!of_property_read_u32(node, "samplerate", &val))
		if (beaglelogic_set_samplerate(dev, val))
			dev_warn(dev, "Invalid default samplerate\n");

	if (!of_property_read_u32(node, "sampleunit", &val))
		if (beaglelogic_set_sampleunit(dev, val))
			dev_warn(dev, "Invalid default sampleunit\n");

	if (!of_property_read_u32(node, "triggerflags", &val))
		if (beaglelogic_set_triggerflags(dev, val))
			dev_warn(dev, "Invalid default triggerflags\n");

	if (bufunitsize < 2 * 1024 * 1024)
		dev_warn(dev, "WARNING:Buffer unit sizes less than "\
				"2 MB are not recommended. Using 4 MB");
	else
		bldev->bufunitsize = bufunitsize;

	/* We got configuration from PRUs, now mark device init'd */
	bldev->state = STATE_BL_INITIALIZED;

	/* Display our init'ed state */
	dev_info(dev, "Default sample rate=%d Hz, sampleunit=%d, "\
			"triggerflags=%d. Buffer in units of %d bytes each",
			bldev->samplerate,
			bldev->sampleunit,
			bldev->triggerflags,
			bldev->bufunitsize);

	/* Once done, create device files */
	err = sysfs_create_group(&dev->kobj, &beaglelogic_attr_group);
	if (err) {
		dev_err(dev, "Registration failed.\n");
		goto faildereg;
	}

	return 0;
faildereg:
	misc_deregister(&bldev->miscdev);
	kfree(bldev);
fail:
	return -1;
}

static int beaglelogic_remove(struct platform_device *pdev)
{
	struct beaglelogicdev *bldev = platform_get_drvdata(pdev);
	struct device *dev = bldev->miscdev.this_device;

	/* Unregister ourselves from the pru_rproc module */
	pruproc_beaglelogic_request_unbind();

	/* Free all buffers */
	beaglelogic_memfree(dev);

	/* Remove the sysfs attributes */
	sysfs_remove_group(&dev->kobj, &beaglelogic_attr_group);

	/* Deregister the misc device */
	misc_deregister(&bldev->miscdev);

	/* Free up memory */
	kfree(bldev);

	/* Print a log message to announce unloading */
	printk("BeagleLogic unloaded\n");
	return 0;
}

static const struct of_device_id beaglelogic_dt_ids[] = {
	{ .compatible = "beaglelogic,beaglelogic", .data = NULL, },
	{ /* sentinel */ },
};

static struct platform_driver beaglelogic_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = beaglelogic_dt_ids,
	},
	.probe = beaglelogic_probe,
	.remove = beaglelogic_remove,
};

module_platform_driver(beaglelogic_driver);

MODULE_AUTHOR("Kumar Abhishek <abhishek@theembeddedkitchen.net>");
MODULE_DESCRIPTION("Kernel Driver for BeagleLogic");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
