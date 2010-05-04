/*
 * arch/xtensa/platforms/xtavnet/include/platform/hardware.h
 *
 * This file contains the hardware configuration of Tensilica Avnet boards
 * (XT-AV60, XT-AV110, XT-AV200, derived from Avnet LX60, LX110, LX200
 * boards respectively).
 *
 * Copyright (C) 2006-2010 Tensilica Inc.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#ifndef __XTAVNET_HARDWARE_H
#define __XTAVNET_HARDWARE_H

#include <variant/core.h>

/* Memory configuration. */

#define PLATFORM_DEFAULT_MEM_START	0x00000000
#define PLATFORM_DEFAULT_MEM_SIZE	0x04000000

/* Interrupt configuration. */

#define PLATFORM_NR_IRQS		2

/* 
 * Default assignment of XTAVnet devices to external interrupts. 
 *
 * CONFIG_ARCH_HAS_SMP means the hardware supports SMP, ie. is Xtensa MX.
 *
 * Systems with SMP support (MX) have an External Interrupt Distributor
 * between external interrupts and the core's interrupts.  The first three
 * core interrupts are used for IPI (interprocessor interrupts), so
 * external (board device) interrupts end up shifted up by 3.
 */

/*  UART interrupt: */
#ifdef CONFIG_ARCH_HAS_SMP
#define UART_INTNUM		XCHAL_EXTINT3_NUM
#else
#define UART_INTNUM		XCHAL_EXTINT0_NUM
#endif

/*  Ethernet interrupt:  */
#ifdef CONFIG_ARCH_HAS_SMP
#define OETH_IRQ		XCHAL_EXTINT4_NUM
#else
#define OETH_IRQ		XCHAL_EXTINT1_NUM
#endif

/*
 *  Device addresses and parameters.
 */

/* UART */
#define UART_PADDR		0xFD050020

/* LCD instruction and data virt. addresses. */
#define LCD_INSTR_ADDR		(char*)(0xFD040000)
#define LCD_DATA_ADDR		(char*)(0xFD040004)

/* Misc. */
#define XTAVNET_FPGAREGS_VADDR	0xFD020000
/* Clock frequency in Hz (read-only):  */
#define XTAVNET_CLKFRQ_VADDR	(XTAVNET_FPGAREGS_VADDR + 0x04)
/* Setting of 8 DIP switches:  */
#define DIP_SWITCHES_VADDR	(XTAVNET_FPGAREGS_VADDR + 0x0C)
/* Software reset (write 0xdead):  */
#define XTAVNET_SWRST_VADDR	(XTAVNET_FPGAREGS_VADDR + 0x10)

/*  OpenCores Ethernet controller:  */
#define OETH_REGS_PADDR		IOADDR(0xD030000)	/* regs + RX/TX descriptors */
#define OETH_REGS_VADDR		0xFD030000
#define OETH_REGS_SIZE  	0x1000  
#define OETH_SRAMBUFF_PADDR	0xFD800000
#define OETH_SRAMBUFF_SIZE	(5*0x600 + 5*0x600)	/* 5*rx buffs + 5*tx buffs */
/*#define OETH_SRAMBUFF_SIZE	0x3C00*/		/* probably 0x4000 ? */
/* The MAC address for these boards is 00:50:c2:13:6f:xx.
   The last byte (here as zero) is read from the DIP switches on the board.  */
#define OETH_MACADDR		0x00, 0x50, 0xc2, 0x13, 0x6f, 0

#endif /* __XTAVNET_HARDWARE_H */
