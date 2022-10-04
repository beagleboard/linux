/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _LINUX_MMIO_KHV_H
#define _LINUX_MMIO_KHV_H

#define KHV_STAT_OFF    0
#define KHV_ADDR_OFF    8
#define KHV_VAL_OFF     16

/* REQ type*/
#define INVAL           0
#define START           1
#define READY           2
#define ACK             3
#define IS_WRITE        BIT(7)

#define WIDTH_8         0
#define WIDTH_16        1
#define WIDTH_32        2
#define WIDTH_64        3

struct mmio_khv_device {
	void __iomem *base;

	volatile unsigned char __iomem *notify_host_reg;
	u32 notify_host_reg_val;
	volatile unsigned char __iomem *notify_guest_reg;
	u32 notify_guest_reg_clr;
};

u64 readx(u32 offset, u32 bw);
void writex(u64 val, u32 offset, u32 bw);
void khv_shutdown(void);

#endif
