/*
 * PRU driver for TI's AM33xx series of SoCs
 *
 * Copyright (C) 2013 Pantelis Antoniou <panto@antoniou-consulting.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <asm/atomic.h>
#include <asm/uaccess.h>

#include <linux/module.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/wait.h>
#include <linux/poll.h>

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

#include <linux/sysfs.h>
#include <linux/fs.h>

#include "external_glue.h"

#define PS_SHM_CODE_SIZE	1
#define PS_SHM_CODE_IDX		0
#define PS_SHM_RET_SIZE		1
#define PS_SHM_RET_IDX		0

/**
 * struct pru_shm - shared memory details per PRU
 * @idx: ID of the PRU this pru_shm structure represents. [PRU0 or PRU1]
 * @vaddr : virtual address of the shared memory, for access from kernel
 * @paddr : physical address of the shared memory, for access from PRU
 * @is_valid : this is = 1 if this structure represents a valid shared memory segment
 */

struct pru_shm {
	int idx;
	void __iomem *vaddr;
	void __iomem *paddr;
	int size_in_pages;
	unsigned int is_valid :1;
};

struct pru_speak_dev {
	/* Misc device descriptor */
	struct miscdevice miscdev;

	/* Imported members */
	int (*downcall_idx)(int, u32, u32, u32, u32, u32, u32);

	/* Data */
	struct device *p_dev; /* parent platform device */
	struct pru_shm shm_code;
	struct pru_shm shm_ret;
};

static const struct file_operations pru_speak_fops;

static ssize_t pru_speak_store_load(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	return pru_external_load(dev, attr, buf, count);
}

static ssize_t pru_speak_store_reset(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	return pru_external_reset(dev, attr, buf, count);
}

/*
 * syscall #0 - useful while developing driver
 * 		writing 1 => all output pins in PRU control set high
 *		writing 0 => all output pins in PRU control set low
 */
static ssize_t pru_speak_debug(int idx, struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pru_speak_dev *pp = platform_get_drvdata(pdev);
	int ret;

	if (count ==  0)
		return -1;
	if (buf[0] == '0')
		ret = pp->downcall_idx(idx, 0, 0, 0, 0, 0, 0);/* idx, syscall_id, 5 args*/
	else
		ret = pp->downcall_idx(idx, 0, 1, 0, 0, 0, 0);

	printk( KERN_INFO "write to pru_speak_debug\n");
	return strlen(buf);
}

static ssize_t pru_speak_debug0(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return pru_speak_debug(0, dev, attr, buf, count);
}

/*
 * syscall #1 - used to initialize the shm info for both PRU and ARM
 * 		*reading* this file will give back hex value of physical address as a string
 *		at the same time PRU is also informed of the address via a downcall
 */

static ssize_t pru_speak_shm_init(int idx, struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
        struct pru_speak_dev *pp = platform_get_drvdata(pdev);
        int ret;

	printk("physical addr shm_code, shm_ret : %x, %x\n", (unsigned int)pp->shm_code.paddr, (unsigned int)pp->shm_ret.paddr);

        ret = pp->downcall_idx(idx, 1, (int)pp->shm_code.paddr, (int)pp->shm_ret.paddr, 10, 0, 0); //pp, idx, sys call id, base addr, val, junk,.,.
	//The pru modifies the arg "10" and places "10^2" in the first loc of shm_code
        printk( KERN_INFO "pru_speak_init, pram value : 10, return value : %d, modified value : %d\n", ret, *((int *)pp->shm_code.vaddr));

	return scnprintf(buf, PAGE_SIZE, "%x,%x", (int)pp->shm_code.paddr, (int)pp->shm_ret.paddr);
}

static ssize_t pru_speak_shm_init0(struct device *dev, struct device_attribute *attr, char *buf)
{
        return pru_speak_shm_init(0, dev, attr, buf);
}

/*
 * syscall #2 - ask PRU to start execution.
 *		writing 1 => start/continue execution
 *		writing 0 => pause execution
 *
 * NOTE : pru_shm_init must be triggered before this. otherwise hell will chase you.
 */

static ssize_t pru_speak_execute(int idx, struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        struct platform_device *pdev = to_platform_device(dev);
        struct pru_speak_dev *pp = platform_get_drvdata(pdev);
        int ret;

	if (count ==  0)
                return -1;
        if (buf[0] == '0')
                ret = pp->downcall_idx(idx, 2, 0, 0, 0, 0, 0);/* idx, syscall_id, start/pause, 4 args*/
        else
                ret = pp->downcall_idx(idx, 2, 1, 0, 0, 0, 0);

        printk( KERN_INFO "write to pru_speak_execute\n");
        return strlen(buf);

}

static ssize_t pru_speak_execute0(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        return pru_speak_execute(0, dev, attr, buf, count);
}

/*
 * syscall #3 - ask pru to abort currently executing BS script
 *		triggered by write to this file
 */

static ssize_t pru_speak_abort(int idx, struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        struct platform_device *pdev = to_platform_device(dev);
        struct pru_speak_dev *pp = platform_get_drvdata(pdev);
        int ret;

        ret = pp->downcall_idx(idx, 3, 0, 0, 0, 0, 0); /* idx, 5 args*/

        printk( KERN_INFO "write to pru_speak_abort\n");
        return strlen(buf);

}

static ssize_t pru_speak_abort0(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        return pru_speak_abort(0, dev, attr, buf, count);
}

/*
 * syscall #4 - get status of botspeak interpreter
 *		"read" returns 0 - if no BS code is being executed
 *		returns 1 - if BS code is being executed
 */

static ssize_t pru_speak_status(int idx, struct device *dev, struct device_attribute *attr, char *buf)
{
        struct platform_device *pdev = to_platform_device(dev);
        struct pru_speak_dev *pp = platform_get_drvdata(pdev);
        int ret = pp->downcall_idx(idx, 4, 0, 0, 0, 0, 0); // idx, sys call id, 5 params

	printk( KERN_INFO "read to pru_speak_status\n");

	return scnprintf(buf, PAGE_SIZE, "%d", ret);
}

static ssize_t pru_speak_status0(struct device *dev, struct device_attribute *attr, char *buf)
{
        return pru_speak_status(0, dev, attr, buf);
}

/*
 * syscall #5 - ask PRU to execute single BS instruction
 *		The Byte code of the BS instruction to be executed is written
 *		to this file.
 *		return value of downcall = 4 if OK, else value = ?
 */

static ssize_t pru_speak_single_cmd(int idx, struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        struct platform_device *pdev = to_platform_device(dev);
        struct pru_speak_dev *pp = platform_get_drvdata(pdev);
        int inst = 0, ret, i;

        if (count ==  0)
                return -1;

	//buf[0] = LSB; buf[3] = MSB
	//the for loop packs the 4 incoming character into a 1 word long integer
	//viz one botspeak instruction (byte code)
	for(i = 0; i < 4; i++){
		inst |= ((int)buf[i]) << i*8;
	}

        ret = pp->downcall_idx(idx, 5, inst, 0, 0, 0, 0);/* idx, syscall_id, 5 args*/

        printk( KERN_INFO "write to pru_speak_single_cmd. return value of downcall : %d\n", ret);
	printk( "STRLEN(buf) : %d\n", strlen(buf)); //prints 0!!
        //return strlen(buf);
	return 4; //quick hack
}

static ssize_t pru_speak_single_cmd0(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        return pru_speak_single_cmd(0, dev, attr, buf, count);
}

/*
 * syscall #6 - ask PRU to execute single 64 bit BS instruction
 *		The Byte code of the BS instruction to be executed is written
 *		to this file.
 *		return value of downcall = 8 if OK, else value = ?
 */

static ssize_t pru_speak_single_cmd_64(int idx, struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        struct platform_device *pdev = to_platform_device(dev);
        struct pru_speak_dev *pp = platform_get_drvdata(pdev);
        int inst_a = 0, ret, i;
        int inst_b = 0;

	//buf[0] = LSB; buf[3] = MSB : for 1st word
	//buf[4] = LSB; buf[7] = MSB : for 2nd word
	//the for loop packs the 8 incoming character into two, 1 word long integers
	//viz one botspeak instruction (byte code)
	for(i = 0; i < 4; i++){
		inst_a |= ((int)buf[i]) << i*8;
	}

	for(i = 4; i < 8; i++){
		inst_b |= ((int)buf[i]) << (i-4)*8;
	}

        ret = pp->downcall_idx(idx, 6, inst_a, inst_b, 0, 0, 0);/* idx, syscall_id, 5 args*/

	printk("**** inst_a : %d, inst_b : %d", inst_a, inst_b);
        printk( KERN_INFO "write to pru_speak_single_cmd_64. return value of downcall : %d\n", ret);
	return 8; //quick hack
}

static ssize_t pru_speak_single_cmd0_64(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        return pru_speak_single_cmd_64(0, dev, attr, buf, count);
}

static DEVICE_ATTR(load, S_IWUSR, NULL, pru_speak_store_load);
static DEVICE_ATTR(reset, S_IWUSR, NULL, pru_speak_store_reset);
static DEVICE_ATTR(pru_speak_debug, S_IWUSR, NULL, pru_speak_debug0);
static DEVICE_ATTR(pru_speak_shm_init, S_IWUSR | S_IRUGO, pru_speak_shm_init0, NULL);
static DEVICE_ATTR(pru_speak_execute, S_IWUSR, NULL, pru_speak_execute0);
static DEVICE_ATTR(pru_speak_abort, S_IWUSR, NULL, pru_speak_abort0);
static DEVICE_ATTR(pru_speak_status, S_IRUGO, pru_speak_status0, NULL);
static DEVICE_ATTR(pru_speak_single_cmd, S_IWUSR, NULL, pru_speak_single_cmd0);
static DEVICE_ATTR(pru_speak_single_cmd_64, S_IWUSR, NULL, pru_speak_single_cmd0_64);

static const struct file_operations pru_beaglelogic_fops = {
	.owner = THIS_MODULE,
};

static int pru_speak_probe(struct platform_device *pdev)
{
	struct pru_speak_dev *pp;
	struct pru_rproc_external_glue g;
	struct device *dev;
	int err;

	/* Allocate memory for our private structure */
	pp = kzalloc(sizeof(*pp), GFP_KERNEL);
	if (!pp)
		goto fail;

	pp->miscdev.fops = &pru_speak_fops;
	pp->miscdev.minor = MISC_DYNAMIC_MINOR;
	pp->miscdev.mode = S_IRUGO;
	pp->miscdev.name = "pru_speak";

	/* Link the platform device data to our private structure */
	pp->p_dev = &pdev->dev;
	dev_set_drvdata(pp->p_dev, pp);

	/* Bind to the pru_rproc module */
	err = pruproc_external_request_bind(&g);
	if (err)
		goto fail;
	pp->downcall_idx = g.downcall_idx;

	/* Once done, register our misc device and link our private data */
	err = misc_register(&pp->miscdev);
	if (err)
		goto fail;
	dev = pp->miscdev.this_device;
	dev_set_drvdata(dev, pp);

	printk("Initializing pru_speak shared memory for code\n");
	pp->shm_code.size_in_pages = PS_SHM_CODE_SIZE;
	pp->shm_code.idx = PS_SHM_CODE_IDX;
	pp->shm_code.is_valid = 1;
	dma_set_coherent_mask(dev, 0xFFFFFFFF);
	pp->shm_code.vaddr = dma_zalloc_coherent(dev, pp->shm_code.size_in_pages * PAGE_SIZE,
							(dma_addr_t *) &(pp->shm_code.paddr), GFP_DMA);
	if(!(pp->shm_code.vaddr)){
		pp->shm_code.is_valid = 0;
		printk("shm_code init failed\n");
	}

	printk("Initializing pru_speak shared memory for return values\n");
	pp->shm_ret.size_in_pages = PS_SHM_RET_SIZE;
	pp->shm_ret.idx = PS_SHM_RET_IDX;
	pp->shm_ret.is_valid = 1;
	pp->shm_ret.vaddr = dma_zalloc_coherent(dev, pp->shm_ret.size_in_pages * PAGE_SIZE,
							(dma_addr_t *) &(pp->shm_ret.paddr), GFP_DMA);
	if(!(pp->shm_ret.vaddr)){
		pp->shm_ret.is_valid = 0;
		printk("shm_ret init failed\n");
	}

	printk("Creating sysfs entries\n");

	err = device_create_file(dev, &dev_attr_load);
	if (err != 0) {
		dev_err(dev, "device_create_file failed\n");
		goto err_fail;
	}

	err = device_create_file(dev, &dev_attr_reset);
	if (err != 0) {
		dev_err(dev, "device_create_file failed\n");
		goto err_fail;
	}

	err = device_create_file(dev, &dev_attr_pru_speak_debug);
	if (err != 0) {
		dev_err(dev, "device_create_file failed\n");
		goto err_fail;
	}

	err = device_create_file(dev, &dev_attr_pru_speak_shm_init);
	if (err != 0) {
		dev_err(dev, "device_create_file failed\n");
		goto err_fail;
	}

	err = device_create_file(dev, &dev_attr_pru_speak_execute);
	if (err != 0) {
		dev_err(dev, "device_create_file failed\n");
		goto err_fail;
	}

	err = device_create_file(dev, &dev_attr_pru_speak_abort);
	if (err != 0) {
		dev_err(dev, "device_create_file failed\n");
		goto err_fail;
	}

	err = device_create_file(dev, &dev_attr_pru_speak_status);
	if (err != 0) {
		dev_err(dev, "device_create_file failed\n");
		goto err_fail;
	}

	err = device_create_file(dev, &dev_attr_pru_speak_single_cmd);
        if (err != 0) {
                dev_err(dev, "device_create_file failed\n");
                goto err_fail;
        }

	err = device_create_file(dev, &dev_attr_pru_speak_single_cmd_64);
        if (err != 0) {
                dev_err(dev, "device_create_file failed\n");
                goto err_fail;
        }

	dev_info(dev, "Loaded OK\n");

	printk("Probe successful");

err_fail:
	return err;
fail:
	return -1;
}

static int pru_speak_remove(struct platform_device *pdev)
{
	struct pru_speak_dev *pp = platform_get_drvdata(pdev);
	struct device *dev = pp->miscdev.this_device;

	/* TODO: Unregister ourselves from the pru_rproc module */

	/* TODO: deallocate memory */

	device_remove_file(dev, &dev_attr_reset);
	device_remove_file(dev, &dev_attr_load);
	device_remove_file(dev, &dev_attr_pru_speak_debug);
	device_remove_file(dev, &dev_attr_pru_speak_shm_init);
	device_remove_file(dev, &dev_attr_pru_speak_execute);
	device_remove_file(dev, &dev_attr_pru_speak_abort);
	device_remove_file(dev, &dev_attr_pru_speak_status);
	device_remove_file(dev, &dev_attr_pru_speak_single_cmd);
	device_remove_file(dev, &dev_attr_pru_speak_single_cmd_64);

	platform_set_drvdata(pdev, NULL);

	/* Print a log message to announce unloading */
	printk("PRU Speak unloaded\n");
	return 0;
}

static const struct of_device_id pru_speak_dt_ids[] = {
	{ .compatible = "ti,pru_speak", .data = NULL, },
	{},
};
//MODULE_DEVICE_TABLE(of, pruss_dt_ids);

static struct platform_driver pru_speak_driver = {
	.driver	= {
		.name	= "pru_speak",
		.owner	= THIS_MODULE,
		.of_match_table = pru_speak_dt_ids,
	},
	.probe	= pru_speak_probe,
	.remove	= pru_speak_remove,
};

//module_platform_driver(pruproc_driver);

static int __init pru_speak_init(void)
{
	printk(KERN_INFO "pru_speak loaded\n");
	platform_driver_register(&pru_speak_driver);
	return 0;
}

static void __exit pru_speak_exit(void)
{
	printk(KERN_INFO "pru_speak unloaded\n");
	platform_driver_unregister(&pru_speak_driver);
}

module_init(pru_speak_init);
module_exit(pru_speak_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("BotSpeak interpreter for AM335x PRU");
MODULE_AUTHOR("Jason Kridner <jdk@ti.com>");
