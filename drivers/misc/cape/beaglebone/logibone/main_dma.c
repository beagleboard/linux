#include <linux/init.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/ioctl.h>
#include <linux/time.h>
#include <asm/uaccess.h>   /* copy_to_user */
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/memory.h>
#include <linux/dma-mapping.h>
#include <linux/edma.h>
#include <linux/platform_data/edma.h>
#include <linux/delay.h>
#include <linux/mutex.h>

//device tree support
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_dma.h>
#include <linux/of_gpio.h>
#include <linux/completion.h>
#include "generic.h"
#include "drvr.h"
#include "ioctl.h"


//#define PROFILE //uncoment to enable code profile

static int dm_open(struct inode *inode, struct file *filp);
static int dm_release(struct inode *inode, struct file *filp);
static ssize_t dm_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos);
static ssize_t dm_read(struct file *filp, char *buf, size_t count, loff_t *f_pos);
static int edma_memtomemcpy(int count, unsigned long src_addr, unsigned long trgt_addr, int dma_ch);
static void dma_callback(unsigned lch, u16 ch_status, void *data);


static struct file_operations dm_ops = {
	.read =   dm_read,
	.write =  dm_write,
	.compat_ioctl =  dm_ioctl,
	.unlocked_ioctl = dm_ioctl,
	.open =   dm_open,
	.release =  dm_release,
};


static dma_addr_t dmaphysbuf = 0;
static volatile int irqraised1 = 0;
static unsigned char gDrvrMajor = 0;
static struct device * prog_device;
static struct class * drvr_class;
static struct drvr_device * drvr_devices;
static struct completion dma_comp;


#ifdef PROFILE

static struct timespec start_ts, end_ts ; //profile timer

inline void start_profile() {
	getnstimeofday(&start_ts);
}

inline void stop_profile() {
	getnstimeofday(&end_ts);
}

inline void compute_bandwidth(const unsigned int nb_byte) {
	struct timespec dt=timespec_sub(end_ts,start_ts);
	long elapsed_u_time=dt.tv_sec*1000000+dt.tv_nsec/1000;

	printk("Time=%ld us\n",elapsed_u_time);
	printk("Bandwidth=%d kBytes/s\n",1000000*(nb_byte>>10)/elapsed_u_time);
}

#endif


ssize_t writeMem(struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
	unsigned short int transfer_size;
	ssize_t transferred = 0;
	unsigned long src_addr, trgt_addr;
	struct drvr_mem * mem_to_write = &(((struct drvr_device *) filp->private_data)->data);
/*
	if (count % 2 != 0) {
		printk("%s: write: Transfer must be 16bits aligned.\n", DEVICE_NAME);

		return -1;
	}
*/
	if (count < MAX_DMA_TRANSFER_IN_BYTES) {
		transfer_size = count;
	} else {
		transfer_size = MAX_DMA_TRANSFER_IN_BYTES;
	}

	if (mem_to_write->dma_buf == NULL) {
		printk("failed to allocate DMA buffer \n");

		return -1;
	}

	trgt_addr = (unsigned long) &(mem_to_write->base_addr[(*f_pos)]);
	src_addr = (unsigned long) dmaphysbuf;

	if (copy_from_user(mem_to_write->dma_buf, buf, transfer_size)) {
		return -1;
	}

	while (transferred < count) {

#ifdef PROFILE
		printk("Write \n");
		start_profile();
#endif

		if (edma_memtomemcpy(transfer_size, src_addr, trgt_addr, mem_to_write->dma_chan) < 0) {
			printk("%s: write: Failed to trigger EDMA transfer.\n", DEVICE_NAME);

			return -1;
		}

#ifdef PROFILE
		stop_profile();
		compute_bandwidth(transfer_size);
#endif

		trgt_addr += transfer_size;
		transferred += transfer_size;

		if ((count - transferred) < MAX_DMA_TRANSFER_IN_BYTES) {
			transfer_size = count - transferred;
		} else {
			transfer_size = MAX_DMA_TRANSFER_IN_BYTES;
		}

		if (copy_from_user(mem_to_write->dma_buf, &buf[transferred], transfer_size)) {
			return -1;
		}
	}

	return transferred;
}

ssize_t readMem(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	unsigned short int transfer_size;
	ssize_t transferred = 0;
	unsigned long src_addr, trgt_addr;

	struct drvr_mem * mem_to_read = &(((struct drvr_device *) filp->private_data)->data);
/*
	if (count % 2 != 0) {
		printk("%s: read: Transfer must be 16bits aligned.\n", DEVICE_NAME);

		return -1;
	}
*/
	if (count < MAX_DMA_TRANSFER_IN_BYTES) {
		transfer_size = count;
	} else {
		transfer_size = MAX_DMA_TRANSFER_IN_BYTES;
	}

	if (mem_to_read->dma_buf == NULL) {
		printk("failed to allocate DMA buffer \n");

		return -1;
	}

	src_addr = (unsigned long) &(mem_to_read->base_addr[(*f_pos)]);
	trgt_addr = (unsigned long) dmaphysbuf;

	while (transferred < count) {

#ifdef PROFILE
		printk("Read \n");
		start_profile();
#endif

		if (edma_memtomemcpy(transfer_size, src_addr, trgt_addr, mem_to_read->dma_chan) < 0) {

			printk("%s: read: Failed to trigger EDMA transfer.\n", DEVICE_NAME);

			return -1;
		}

		if (copy_to_user(&buf[transferred], mem_to_read->dma_buf, transfer_size)) {
			return -1;
		}

#ifdef PROFILE
		stop_profile();
		compute_bandwidth(transfer_size);
#endif

		src_addr += transfer_size;
		transferred += transfer_size;

		if ((count - transferred) < MAX_DMA_TRANSFER_IN_BYTES) {
			transfer_size = (count - transferred);
		} else {
			transfer_size = MAX_DMA_TRANSFER_IN_BYTES;
		}
	}

	return transferred;
}

static int dm_open(struct inode *inode, struct file *filp)
{
	struct drvr_device * dev = container_of(inode->i_cdev, struct drvr_device, cdev);
	struct drvr_mem* mem_dev ;

	filp->private_data = dev; /* for other methods */

	if (dev == NULL) {
		printk("%s: Failed to retrieve driver structure !\n", DEVICE_NAME);

		return -1;
	}

	if (dev->opened == 1) {
		printk("%s: module already opened\n", DEVICE_NAME);

		return 0;
	}

	mem_dev = &(dev->data);
	request_mem_region((unsigned long) mem_dev->base_addr, FPGA_MEM_SIZE, DEVICE_NAME);
	mem_dev->virt_addr = ioremap_nocache(((unsigned long) mem_dev->base_addr), FPGA_MEM_SIZE);
	mem_dev->dma_chan = edma_alloc_channel(EDMA_CHANNEL_ANY, dma_callback, NULL, EVENTQ_0);
	mem_dev->dma_buf = (unsigned char *) dma_alloc_coherent(NULL, MAX_DMA_TRANSFER_IN_BYTES, &dmaphysbuf, 0);
	printk("EDMA channel %d reserved \n", mem_dev->dma_chan);

	if (mem_dev->dma_chan < 0) {
		printk("edma_alloc_channel failed for dma_ch, error: %d\n", mem_dev->dma_chan);

		return -1;
	}

	printk("mem interface opened \n");

	dev->opened = 1;

	return 0;
}

static int dm_release(struct inode *inode, struct file *filp)
{
	struct drvr_device
	* dev = container_of(inode->i_cdev, struct drvr_device, cdev);

	if (dev->opened == 0) {
		printk("%s: module already released\n", DEVICE_NAME);

		return 0;
	}

	iounmap((dev->data).virt_addr);
	release_mem_region(((unsigned long) (dev->data).base_addr), FPGA_MEM_SIZE);
	printk("%s: Release: module released\n", DEVICE_NAME);
	dma_free_coherent(NULL, MAX_DMA_TRANSFER_IN_BYTES, (dev->data).dma_buf, dmaphysbuf);
	edma_free_channel((dev->data).dma_chan);


	dev->opened = 0;

	return 0;
}

static ssize_t dm_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
	return writeMem(filp, buf, count, f_pos);
}

static ssize_t dm_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	return readMem(filp, buf, count, f_pos);

}

static void dm_exit(void)
{
	dev_t devno = MKDEV(gDrvrMajor, 0);

	/* Get rid of our char dev entries */
	if (drvr_devices) {
		int i = 0;

		device_destroy(drvr_class, MKDEV(gDrvrMajor, i));
		cdev_del(&drvr_devices[i].cdev);
		kfree(drvr_devices);
	}

	class_destroy(drvr_class);
	/* cleanup_module is never called if registering failed */
	unregister_chrdev_region(devno, 2);

	ioctl_exit();
}

static int dm_init(void)
{
	int result;
	int devno;
	struct drvr_mem * memDev;

	dev_t dev = 0;
	result = alloc_chrdev_region(&dev, 0, 1, DEVICE_NAME);
	gDrvrMajor = MAJOR(dev);

	if (result < 0) {
		printk(KERN_ALERT "Registering char device failed with %d\n", gDrvrMajor);
		return result;
	}

	drvr_devices = kmalloc(sizeof(struct drvr_device), GFP_KERNEL);

	if (!drvr_devices) {
		dm_exit();
		return -ENOMEM;
	}

	drvr_class = class_create(THIS_MODULE, DEVICE_NAME);
	memset(drvr_devices, 0, sizeof(struct drvr_device));

	//printk(KERN_INFO "'mknod /dev/%s c %d %d'.\n", DEVICE_NAME, gDrvrMajor, 0);
	/* Initialize each device. */
	devno = MKDEV(gDrvrMajor, 0);
	memDev = &(drvr_devices[0].data);
	memDev->base_addr = (unsigned short *) (FPGA_BASE_ADDR);
	device_create(drvr_class, prog_device, devno, NULL, DEVICE_NAME_MEM);
	cdev_init(&(drvr_devices[0].cdev), &dm_ops);
	(drvr_devices[0].cdev).owner = THIS_MODULE;
	(drvr_devices[0].cdev).ops = &dm_ops;
	cdev_add(&(drvr_devices[0].cdev), devno, 1);
	drvr_devices[0].opened = 0;
	init_completion(&dma_comp);
	return ioctl_init();
}

static int edma_memtomemcpy(int count, unsigned long src_addr, unsigned long trgt_addr, int dma_ch)
{
	int result = 0;
	struct edmacc_param param_set;

	edma_set_src(dma_ch, src_addr, INCR, W256BIT);
	edma_set_dest(dma_ch, trgt_addr, INCR, W256BIT);
	edma_set_src_index(dma_ch, 1, 1);
	edma_set_dest_index(dma_ch, 1, 1);
	/* A Sync Transfer Mode */
	edma_set_transfer_params(dma_ch, count, 1, 1, 1, ASYNC); //one block of one frame of one array of count bytes

	/* Enable the Interrupts on Channel 1 */
	edma_read_slot(dma_ch, &param_set);
	param_set.opt |= ITCINTEN;
	param_set.opt |= TCINTEN;
	param_set.opt |= EDMA_TCC(EDMA_CHAN_SLOT(dma_ch));
	edma_write_slot(dma_ch, &param_set);
	irqraised1 = 0u;
	dma_comp.done = 0;
	result = edma_start(dma_ch);

	if (result != 0) {
		printk("%s: edma copy failed \n", DEVICE_NAME);
	}

	wait_for_completion(&dma_comp);

	/* Check the status of the completed transfer */
	if (irqraised1 < 0) {
		printk("%s: edma copy: Event Miss Occured!!!\n", DEVICE_NAME);
		edma_stop(dma_ch);
		result = -EAGAIN;
	}

	return result;
}

static void dma_callback(unsigned lch, u16 ch_status, void *data)
{
	switch (ch_status) {
		case DMA_COMPLETE:
			irqraised1 = 1;
			break;

		case DMA_CC_ERROR:
			irqraised1 = -1;
			break;

		default:
			irqraised1 = -1;
			break;
	}

	complete(&dma_comp);
}

static const struct of_device_id drvr_of_match[] = {
	{ .compatible = DEVICE_NAME, },
	{ },
};

MODULE_DEVICE_TABLE(of, drvr_of_match);
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Jonathan Piat <piat.jonathan@gmail.com>");
MODULE_AUTHOR("Martin Schmitt <test051102@hotmail.com>");

module_init(dm_init);
module_exit(dm_exit);
