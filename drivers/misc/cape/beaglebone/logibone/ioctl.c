#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <asm/uaccess.h>   /* copy_to_user */
#include "ioctl.h"
#include "drvr.h"


long ioctl_init() {
	return 0;
}

void ioctl_exit() {
}

long dm_ioctl(struct file *filp, unsigned int cmd, unsigned long arg){
	printk("ioctl failed \n");

	return -ENOTTY;
}
