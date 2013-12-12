/* -*- linux-c -*- */

/* Linux Kernel Module for Breakaway Systems UPS control.
 *
 * PUBLIC DOMAIN
 */

#include <linux/syscalls.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/reboot.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/mount.h>
#include <linux/workqueue.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>

/* Module to sync file systems leaving them mounted read-only,
 * then signal the UPS that it is safe to remove
 * power, and finally halt the processor.
 * Also to allow kicking the watchdog from user mode.
 */

#define N_GPIOS 9		/* Total number of GPIOS used */

#define REQ_GPIO_IDX 0		/* Indices got GPIOS */
#define ACK_GPIO_IDX 1
#define WDG_GPIO_IDX 2
#define LED1_GREEN_IDX 3
#define LED1_RED_IDX 4
#define LED2_GREEN_IDX 5
#define LED2_RED_IDX 6
#define GEN_OUT1_IDX 7
#define GEN_OUT2_IDX 8

static struct argus_ups_info {	/* As there is only one UPS device we can make this static */
	struct fasync_struct *async_queue; /* asynchronous readers */
	struct platform_device *pdev;
	struct pwm_device *pwm_dev;
	struct gpio gpios[N_GPIOS];
} info = {NULL, NULL, NULL, /* Some fields filled in by device tree, probe, etc. */
     {
	     {-1, GPIOF_IN, "Powerdown request"},
	     {-1, GPIOF_OUT_INIT_LOW, "Powerdown acknowledge" },
	     {-1, GPIOF_OUT_INIT_LOW, "Watchdog"},
	     {-1, GPIOF_OUT_INIT_LOW, "LED 1 Green"},
	     {-1, GPIOF_OUT_INIT_LOW, "LED 1 Red"},
	     {-1, GPIOF_OUT_INIT_LOW, "LED 2 Green"},
	     {-1, GPIOF_OUT_INIT_LOW, "LED 2 Red"},
	     {-1, GPIOF_OUT_INIT_LOW, "General Output #1"},
	     {-1, GPIOF_OUT_INIT_LOW, "General Output #2"}
     },
};


static const struct of_device_id argus_ups_of_ids[] = {
	{ .compatible = "argus-ups" },
	{ }
};

static int argus_ups_major;     /* Major device number */

static struct class *argus_ups_class; /* /sys/class */

dev_t argus_ups_dev;            /* Device number */

static struct cdev *argus_ups_cdev; /* Character device details */

static void argus_ups_function(struct work_struct *ignored); /* Work function */

static DECLARE_DELAYED_WORK(argus_ups_work, argus_ups_function); /* Kernel workqueue glue */

static struct workqueue_struct *argus_ups_workqueue; /* Kernel workqueue */

static int debug = 0;
module_param(debug, int, S_IRUGO);
MODULE_PARM_DESC(debug, "Debug flag");

static int shutdown = 1;
module_param(shutdown, int, S_IRUGO);
MODULE_PARM_DESC(shutdown, "Shutdown flag");

static char* fs_type_names[] = {"vfat", "ext4"}; /* File system names that may need syncing. */

/* Just kick watchdog */

static ssize_t argus_ups_write(struct file *filp, const char __user *buf, size_t count,
                loff_t *f_pos)
{
	int i;
        if (debug >= 3) {
            printk("Writing to watchdog - count:%d\n", count);
        }
	for (i = 0; i < count; i++) {
		gpio_set_value(info.gpios[WDG_GPIO_IDX].gpio, 1); /* Set it */
		msleep(10);                       /* Wait */
		gpio_set_value(info.gpios[WDG_GPIO_IDX].gpio, 0); /* End clearing it */
		msleep(10);
	}
	return count;                     /* Always returns what we sent, regardsless */
}

static long argus_ups_ioctl(struct file *file,
			   unsigned int ioctl,
			   unsigned long param)
{
	if (debug >= 4) {
		printk(KERN_ERR "ioctl: %d, param: %ld\n", ioctl, param);
	}
	switch(ioctl) {
	case 10001: {
		debug = param;
		printk("Debug set to %d\n", debug);
		break;
	}
	case 10002: {
		unsigned char value = param & 0x0F;
		unsigned char mask = (param >> 4) & 0x0F;
		int i;		/* Loop iterator */
		if (mask == 0) {
			printk(KERN_ERR "Pointless mask of zero!\n");
		}
		for (i = 0; i < 4; i++) { /* For all four LEDS */
			if (mask & (1 << i)) { /* Only masked values */
				if (value & (1 << i)) { /* On - so gpio is hi */
					if (debug >= 4) {
						printk("Setting %d hi, ",
						       info.gpios[LED1_GREEN_IDX + i].gpio);
					}
					gpio_set_value(info.gpios[LED1_GREEN_IDX + i].gpio, 1);
				}
				else {	/* Off - so gpio is lo */
					if (debug >= 4) {
						printk("Setting %d lo, ",
						       info.gpios[LED1_GREEN_IDX + i].gpio);
					}
					gpio_set_value(info.gpios[LED1_GREEN_IDX + i].gpio, 0);
				}
			}
		}
		if (debug >= 4) {
			printk("\n");
		}
		break;
	}
	case 10003: {
		gpio_set_value(info.gpios[GEN_OUT1_IDX].gpio, param & 1);
		break;
	}
	case 10004: {
		gpio_set_value(info.gpios[GEN_OUT2_IDX].gpio, param & 1);
		break;
	}
	default:
	{
		printk(KERN_ERR "Invalid ioctl %d\n", ioctl);
		return -1;
	}
	}
	return 0;
}

static int argus_ups_fasync(int fd, struct file *filp, int mode)
{
	printk(KERN_ERR "In argus_ups_fasync() fd:%d, filp:%p, mode:%d\n", fd, filp, mode);
	return fasync_helper(fd, filp, mode, &info.async_queue);
}

static struct file_operations argus_ups_fops = { /* Only file operation is to kick watchdog via a write */
	.owner =    THIS_MODULE,
	.llseek =   NULL,
	.read =     NULL,
	.unlocked_ioctl = argus_ups_ioctl,
	.write =    argus_ups_write,
	.open =     NULL,
	.release =  NULL,
	.fasync = argus_ups_fasync,
};


static void remount_sb(struct super_block *sb)
{
	int flags =  MS_RDONLY;
	int result = sb->s_op->remount_fs(sb, &flags, "");
	if (debug) {
		printk("Processing superblock %p\n", sb);
		printk("Remount operation returned %d\n", result);
	}
}


static void argus_ups_function(struct work_struct *ignored)
{
	static int testdata = 0;       /* Data for test */
	int i;                      /* Iterator */
	testdata++;
	if (!gpio_get_value(info.gpios[REQ_GPIO_IDX].gpio)) {
                queue_delayed_work(argus_ups_workqueue, &argus_ups_work, HZ/100); /* Re-queue in 10mS*/
		return;
        }
	printk(KERN_ERR "Request received\n");
	if (debug) {
		printk("Shutdown request received from UPS\n");
	}
	if (!shutdown) {
		printk("Shutdown request ignored\n");
		return;
	}

	if (debug) {
		printk("Sending async kill SIGIO to %p\n", info.async_queue);
	}
	if (info.async_queue) { /* Try and tell usermode to halt system */
		kill_fasync(&info.async_queue, SIGIO, POLL_IN);
	}
	gpio_set_value(info.gpios[LED1_GREEN_IDX].gpio, 0); /* Turn off green LED1 */
	for (i = 0; i < 300; i++) { /* Toggle acknowledge at 10 Hz for 15 seconds */
		if (debug >= 2) {
			printk("Waiting for first shutdown request:%d\n", i);
		}
		gpio_set_value(info.gpios[ACK_GPIO_IDX].gpio, i & 1); /* Toggle acknowledge */
		gpio_set_value(info.gpios[LED1_RED_IDX].gpio, i & 1); /* and LED1 red */
		msleep(50); /* Wait in 50ms increments */
	}

	{
		char *argv[] = { "/sbin/halt", NULL };
		static char *envp[] = {
			"HOME=/",
			"TERM=linux",
			"PATH=/usr/local/bin:/usr/bin:/bin:/usr/local/sbin:/usr/sbin:/sbin", NULL };

		call_usermodehelper( argv[0], argv, envp, UMH_WAIT_PROC );
	}
	for (i = 0; i < 300; i++) { /* Toggle acknowledge at 10 Hz for 15 more seconds */
		if (debug >= 2) {
			printk("Waiting for second shutdown request:%d\n", i);
		}
		gpio_set_value(info.gpios[ACK_GPIO_IDX].gpio, i & 1); /* Toggle acknowledge */
		gpio_set_value(info.gpios[LED1_RED_IDX].gpio, i & 1); /* and LED1 red */
		msleep(50); /* Wait in 50ms increments */
	}
	printk(KERN_ERR "Usermode failed to halt system\n");
	kernel_halt();	       /* Last resort - may give some oopss */
}


static int argus_ups_probe(struct platform_device *pdev) /* Entry point */
{
	struct pinctrl *pinctrl;
	struct device_node *pnode = pdev->dev.of_node;
	int i;
	int ret;
        printk("Init UPS module - debug=%d, shutdown=%d\n",
	       debug, shutdown);
	platform_set_drvdata(pdev, &info);
	info.pdev = pdev;
	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(pinctrl)) {
		dev_warn(&pdev->dev,
			"pins are not configured from the driver\n");
		return -1;
	}
	ret = of_property_read_u32(pnode, "debug", &debug);
	if (ret != 0) {
		dev_err(&pdev->dev, "Unable to read debug parameter\n");
	}
	else {
		printk("Debug parameter read from DT:%d\n", debug);
	}

	ret = of_property_read_u32(pnode, "shutdown", &shutdown);
	if (ret != 0) {
		dev_err(&pdev->dev, "Unable to read shutdown parameter\n");
	}
	else {
		printk("Shutdown parameter read from DT:%d\n", shutdown);
	}

	ret = of_gpio_count(pnode);

	if (ret != N_GPIOS) {
		printk(KERN_ERR "Wrong number of gpios");
		return -1;
	}

	for (i = 0; i < of_gpio_count(pnode); i++) {
		ret = of_get_gpio_flags(pnode, i, NULL);
		if (debug) {
			printk("GPIO#%d:%d\n", i, ret);
		}
		if (IS_ERR_VALUE(ret)) {
			dev_err(&pdev->dev, "unable to get GPIO %d\n", i);
			goto err_no_gpio;
		}
		info.gpios[i].gpio = ret;
	}


        ret = alloc_chrdev_region(&argus_ups_dev, 0, 2, "argus_ups");
        argus_ups_major = MAJOR(argus_ups_dev);
        if (ret) {
		printk(KERN_ERR "Error %d adding argus_ups\n", ret);
		return -1;
        }
	if (debug) {
		printk("argus_ups major: %d\n", argus_ups_major);
	}
        argus_ups_cdev = cdev_alloc(); /* Make this a character device */
        argus_ups_cdev->ops = &argus_ups_fops; /* File operations */
        argus_ups_cdev->owner = THIS_MODULE;   /* Top level device */
        ret = cdev_add(argus_ups_cdev, argus_ups_dev, 1); /* Add it to the kernel */
        if (ret) {
		printk(KERN_ERR "cdev_add returned %d\n", ret);
		unregister_chrdev_region(0, 1);
		return -1;
	}
        ret = gpio_request_array(info.gpios, N_GPIOS);
	if (ret) {
		printk(KERN_ERR "Error %d requesting GPIOs\n", ret);
		unregister_chrdev_region(0, 1);
		return -1;
        }

        argus_ups_class = class_create(THIS_MODULE, "argus_ups"); /* /sys/class entry for udev */
        if (IS_ERR(argus_ups_class)) {
		printk(KERN_ERR "Error creating argus_ups_class\n");
		unregister_chrdev_region(0, 1);
		return -1;
	}
	device_create(argus_ups_class, NULL, MKDEV(argus_ups_major, 0), NULL, "argus_ups");
        argus_ups_workqueue = create_singlethread_workqueue("argus_ups");
        INIT_DELAYED_WORK(&argus_ups_work, argus_ups_function);
        queue_delayed_work(argus_ups_workqueue, &argus_ups_work, 0); /* Start work immediately */

        return 0;
err_no_gpio:
	return ret;

}


static void argus_ups_cleanup(void)
{
	printk("Module cleanup called\n");
        while (cancel_delayed_work(&argus_ups_work) == 0) {
		flush_workqueue(argus_ups_workqueue); /* Make sure all work is completed */
	}
	destroy_workqueue(argus_ups_workqueue);
	gpio_free_array(info.gpios, N_GPIOS);
	device_destroy(argus_ups_class, argus_ups_dev);
	class_destroy(argus_ups_class);
        unregister_chrdev_region(argus_ups_dev, 1);
        cdev_del(argus_ups_cdev);
}



static int argus_ups_remove(struct platform_device *pdev)
{
	printk("In argus_ups_remove()\n");
	argus_ups_cleanup();
	printk("After cleanup\n");
	return 0;
}

#define ARGUS_UPS_PM_OPS NULL

struct platform_driver argus_ups_driver = {
	.probe		= argus_ups_probe,
	.remove		= argus_ups_remove,
	.driver = {
		.name		= "argus-ups",
		.owner		= THIS_MODULE,
		.pm		= ARGUS_UPS_PM_OPS,
		.of_match_table = argus_ups_of_ids,
	},
};


static int __init argus_ups_init(void)
{
	return platform_driver_probe(&argus_ups_driver,
				     argus_ups_probe);
}

static void __exit argus_ups_exit(void)
{
	platform_driver_unregister(&argus_ups_driver);
	printk("After driver unregister\n");
}

module_init(argus_ups_init);
module_exit(argus_ups_exit);

/*
 * Get rid of taint message.
 */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Lambert");	/* Who wrote this module? */
MODULE_DESCRIPTION("Argus UPS control"); /* What does this module do */
MODULE_ALIAS("platform:argus-ups");
MODULE_DEVICE_TABLE(of, argus_ups_of_ids);
