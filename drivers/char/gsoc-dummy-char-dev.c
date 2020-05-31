/*
 * GSoC Dummy Driver for warm up exercise
 *
 * Author:	Niklas Wantrupp <niklaswantrupp@web.de>
 *        	based on
 * 				Dummy character driver by
 *				John Madieu <john.madieu@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/version.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>

static int gsoc_char_dev_open(struct inode * inode, struct file * file);
static int gsoc_char_dev_close(struct inode * inode, struct file * file);
static ssize_t gsoc_char_dev_read(struct file *file, char __user * buf, size_t count,
                                loff_t * offset);
static ssize_t gsoc_char_dev_write(struct file * file, const char __user * buf, size_t count,
                                loff_t * offset);
static int gsoc_char_dev_uevent(struct device *dev, struct kobj_uevent_env *env);

static unsigned int major;
static struct class *gsoc_dummy_class;
static struct cdev gsoc_dummy_device;

static const struct file_operations gsoc_dummy_dev_file_ops = {
    .open = gsoc_char_dev_open,
    .release = gsoc_char_dev_close,
    .read = gsoc_char_dev_read,
    .write = gsoc_char_dev_write,
};

static int __init gsoc_dummy_char_dev_init_module(void)
{
    struct device *dummy_device;
    int err;
    dev_t gsoc_dev;

    // register a range of char dev numbers 0 to 1 in this case
    err = alloc_chrdev_region(&gsoc_dev, 0, 1, "gsoc_dummy_char_dev");
    if (err < 0) {
        pr_err("Can't get major number\n");
        return err;
    }
    major = MAJOR(gsoc_dev);
    pr_info("gsoc_dummy_char_dev major number = %d\n",major);

    // Create sysfs class
    gsoc_dummy_class = class_create(THIS_MODULE, "gsoc_dummy_char_dev_class");

    if (IS_ERR(gsoc_dummy_class)) {
        pr_err("GSoC char device class could not be created.\n");
        unregister_chrdev_region(MKDEV(major, 0), 1);
        return PTR_ERR(gsoc_dummy_class);
    }

    gsoc_dummy_class->dev_uevent = gsoc_char_dev_uevent;

    // Create device and bind file ops
    cdev_init(&gsoc_dummy_device, &gsoc_dummy_dev_file_ops);
    gsoc_dummy_device.owner = THIS_MODULE;

    // add device to system
    cdev_add(&gsoc_dummy_device, gsoc_dev, 1);

    // create device node
    dummy_device = device_create(gsoc_dummy_class, NULL, gsoc_dev, NULL, "gsoc_dummy_char_dev"); 

    if (IS_ERR(dummy_device)) {
        pr_err("GSoC char device could not be created.\n");
        class_destroy(gsoc_dummy_class);
        unregister_chrdev_region(gsoc_dev, 1);
        return -1;
    }

    pr_info("loaded GSoC dummy driver\n");
    return 0;
}

static void __exit gsoc_dummy_char_dev_exit_module(void)
{
    unregister_chrdev_region(MKDEV(major, 0), 1);
    device_destroy(gsoc_dummy_class, MKDEV(major, 0));
    cdev_del(&gsoc_dummy_device);
    class_destroy(gsoc_dummy_class);

    pr_info("unloaded GSoC dummy driver\n");
}

static int gsoc_char_dev_open(struct inode * inode, struct file * file)
{
    pr_info("Open function of GSoC dummy driver called.\n");
    return 0;
}

static int gsoc_char_dev_close(struct inode * inode, struct file * file)
{
    pr_info("Close function of GSoC dummy driver called.\n");
    return 0;
}

static ssize_t gsoc_char_dev_read(struct file *file, char __user * buf, size_t count,
                                loff_t * offset)
{
    pr_info("Read function of GSoC dummy driver called.\n");
    return 0;
}

static ssize_t gsoc_char_dev_write(struct file * file, const char __user * buf, size_t count,
                                loff_t * offset)
{
    pr_info("Write function of GSoC dummy driver called.\n");
    return count;
}

static int gsoc_char_dev_uevent(struct device *dev, struct kobj_uevent_env *env)
{
    add_uevent_var(env, "DEVMODE=%#o", 0666);
    return 0;
}

module_init(gsoc_dummy_char_dev_init_module);
module_exit(gsoc_dummy_char_dev_exit_module);

MODULE_AUTHOR("Niklas Wantrupp <niklaswantrupp@web.de>");
MODULE_DESCRIPTION("GSoC warm up dummy char device driver");
MODULE_LICENSE("GPL");