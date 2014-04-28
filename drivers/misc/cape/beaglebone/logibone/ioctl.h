#ifndef __IOCTL_H__
#define __IOCTL_H__

#include <linux/stddef.h>
#include <linux/ioctl.h>

#define MAJOR_NUM 100

long ioctl_init(void);
void ioctl_exit(void);
long dm_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

#endif
