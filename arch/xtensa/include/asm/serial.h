/*
 * arch/xtensa/include/asm/serial.h
 *
 * Configuration details for 8250, 16450, 16550, etc. serial ports
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001-2010 Tensilica Inc.
 */

#ifndef _XTENSA_SERIAL_H
#define _XTENSA_SERIAL_H

#include <asm/irq.h>
#include <platform/serial.h>

/* The 8250 driver treats IRQ 0 as absent.  For the Xtensa architecture,
   interrupt 0 is valid, must compare against NO_IRQ instead. */
#ifdef is_real_interrupt
#undef is_real_interrupt
#define is_real_interrupt(irq)	((irq) != NO_IRQ)
#endif

#endif	/* _XTENSA_SERIAL_H */
