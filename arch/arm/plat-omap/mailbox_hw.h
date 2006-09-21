/*
 * Header for OMAP mailbox driver
 *
 * Copyright (C) 2006 Nokia Corporation. All rights reserved.
 *
 * Contact: Toshihiro Kobayashi <toshihiro.kobayashi@nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <asm/hardware.h>

#if defined(CONFIG_ARCH_OMAP1)

#define MAILBOX_BASE			(0xfffcf000)
#define MAILBOX_ARM2DSP1		(MAILBOX_BASE + 0x00)
#define MAILBOX_ARM2DSP1b		(MAILBOX_BASE + 0x04)
#define MAILBOX_DSP2ARM1		(MAILBOX_BASE + 0x08)
#define MAILBOX_DSP2ARM1b		(MAILBOX_BASE + 0x0c)
#define MAILBOX_DSP2ARM2		(MAILBOX_BASE + 0x10)
#define MAILBOX_DSP2ARM2b		(MAILBOX_BASE + 0x14)
#define MAILBOX_ARM2DSP1_Flag		(MAILBOX_BASE + 0x18)
#define MAILBOX_DSP2ARM1_Flag		(MAILBOX_BASE + 0x1c)
#define MAILBOX_DSP2ARM2_Flag		(MAILBOX_BASE + 0x20)

#elif defined(CONFIG_ARCH_OMAP2)

/*
 * Mailbox: L4 peripheral -- use omap_readX(), omap_writeX()
 */
#define OMAP24XX_MAILBOX_BASE		(L4_24XX_BASE + 0x94000)

#define MAILBOX_REVISION		(OMAP24XX_MAILBOX_BASE + 0x00)
#define MAILBOX_SYSCONFIG		(OMAP24XX_MAILBOX_BASE + 0x10)
#define MAILBOX_SYSSTATUS		(OMAP24XX_MAILBOX_BASE + 0x14)
#define MAILBOX_MESSAGE_0		(OMAP24XX_MAILBOX_BASE + 0x40)
#define MAILBOX_MESSAGE_1		(OMAP24XX_MAILBOX_BASE + 0x44)
#define MAILBOX_MESSAGE_2		(OMAP24XX_MAILBOX_BASE + 0x48)
#define MAILBOX_MESSAGE_3		(OMAP24XX_MAILBOX_BASE + 0x4c)
#define MAILBOX_MESSAGE_4		(OMAP24XX_MAILBOX_BASE + 0x50)
#define MAILBOX_MESSAGE_5		(OMAP24XX_MAILBOX_BASE + 0x54)
#define MAILBOX_FIFOSTATUS_0		(OMAP24XX_MAILBOX_BASE + 0x80)
#define MAILBOX_FIFOSTATUS_1		(OMAP24XX_MAILBOX_BASE + 0x84)
#define MAILBOX_FIFOSTATUS_2		(OMAP24XX_MAILBOX_BASE + 0x88)
#define MAILBOX_FIFOSTATUS_3		(OMAP24XX_MAILBOX_BASE + 0x8c)
#define MAILBOX_FIFOSTATUS_4		(OMAP24XX_MAILBOX_BASE + 0x90)
#define MAILBOX_FIFOSTATUS_5		(OMAP24XX_MAILBOX_BASE + 0x94)
#define MAILBOX_MSGSTATUS_0		(OMAP24XX_MAILBOX_BASE + 0xc0)
#define MAILBOX_MSGSTATUS_1		(OMAP24XX_MAILBOX_BASE + 0xc4)
#define MAILBOX_MSGSTATUS_2		(OMAP24XX_MAILBOX_BASE + 0xc8)
#define MAILBOX_MSGSTATUS_3		(OMAP24XX_MAILBOX_BASE + 0xcc)
#define MAILBOX_MSGSTATUS_4		(OMAP24XX_MAILBOX_BASE + 0xd0)
#define MAILBOX_MSGSTATUS_5		(OMAP24XX_MAILBOX_BASE + 0xd4)
#define MAILBOX_IRQSTATUS_0		(OMAP24XX_MAILBOX_BASE + 0x100)
#define MAILBOX_IRQENABLE_0		(OMAP24XX_MAILBOX_BASE + 0x104)
#define MAILBOX_IRQSTATUS_1		(OMAP24XX_MAILBOX_BASE + 0x108)
#define MAILBOX_IRQENABLE_1		(OMAP24XX_MAILBOX_BASE + 0x10c)
#define MAILBOX_IRQSTATUS_2		(OMAP24XX_MAILBOX_BASE + 0x110)
#define MAILBOX_IRQENABLE_2		(OMAP24XX_MAILBOX_BASE + 0x114)
#define MAILBOX_IRQSTATUS_3		(OMAP24XX_MAILBOX_BASE + 0x118)
#define MAILBOX_IRQENABLE_3		(OMAP24XX_MAILBOX_BASE + 0x11c)

#define MAILBOX_IRQ_NOTFULL(n)		(1<<(2*(n)+1))
#define MAILBOX_IRQ_NEWMSG(n)		(1<<(2*(n)))

#endif /* CONFIG_ARCH_OMAP2 */
