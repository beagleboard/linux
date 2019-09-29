/**
 *   Copyright &copy; 2002-2004 Philippe Gerum.
 *
 *   ARM port
 *     Copyright (C) 2005 Stelian Pop
 *
 *   Xenomai is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU General Public License as
 *   published by the Free Software Foundation, Inc., 675 Mass Ave,
 *   Cambridge MA 02139, USA; either version 2 of the License, or (at
 *   your option) any later version.
 *
 *   Xenomai is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with Xenomai; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 *   02111-1307, USA.
 */
#ifndef _COBALT_ARM_ASM_MACHINE_H
#define _COBALT_ARM_ASM_MACHINE_H

#include <linux/version.h>
#include <asm/byteorder.h>

#define XNARCH_HOST_TICK_IRQ __ipipe_hrtimer_irq

#include <asm/barrier.h>
#include <asm/compiler.h>
#include <asm/cmpxchg.h>
#include <asm/switch_to.h>
#include <asm/system_info.h>
#include <asm/system_misc.h>
#include <asm/timex.h>
#include <asm/processor.h>
#include <asm/ipipe.h>
#include <asm/mach/irq.h>
#include <asm/cacheflush.h>

#define xnarch_cache_aliasing() cache_is_vivt()

#if __LINUX_ARM_ARCH__ < 5
static inline __attribute_const__ unsigned long ffnz(unsigned long x)
{
	int r = 0;

	if (!x)
		return 0;
	if (!(x & 0xffff)) {
		x >>= 16;
		r += 16;
	}
	if (!(x & 0xff)) {
		x >>= 8;
		r += 8;
	}
	if (!(x & 0xf)) {
		x >>= 4;
		r += 4;
	}
	if (!(x & 3)) {
		x >>= 2;
		r += 2;
	}
	if (!(x & 1)) {
		x >>= 1;
		r += 1;
	}
	return r;
}
#else
static inline __attribute_const__ unsigned long ffnz(unsigned long ul)
{
	int __r;
	__asm__("clz\t%0, %1" : "=r" (__r) : "r"(ul & (-ul)) : "cc");
	return 31 - __r;
}
#endif

#include <asm-generic/xenomai/machine.h>

#endif /* !_COBALT_ARM_ASM_MACHINE_H */
