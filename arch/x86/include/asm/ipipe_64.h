/*   -*- linux-c -*-
 *   arch/x86/include/asm/ipipe_64.h
 *
 *   Copyright (C) 2007-2012 Philippe Gerum.
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, Inc., 675 Mass Ave, Cambridge MA 02139,
 *   USA; either version 2 of the License, or (at your option) any later
 *   version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __X86_IPIPE_64_H
#define __X86_IPIPE_64_H

#define ipipe_read_tsc(t)  do {		\
	unsigned int __a,__d;			\
	asm volatile("rdtsc" : "=a" (__a), "=d" (__d)); \
	(t) = ((unsigned long)__a) | (((unsigned long)__d)<<32); \
} while(0)

extern unsigned int cpu_khz;
#define __ipipe_cpu_freq	({ unsigned long long __freq = (1000ULL * cpu_khz); __freq; })
#define __ipipe_hrclock_freq	__ipipe_cpu_freq

#define ipipe_tsc2ns(t)	(((t) * 1000UL) / (__ipipe_hrclock_freq / 1000000UL))
#define ipipe_tsc2us(t)	((t) / (__ipipe_hrclock_freq / 1000000UL))

static inline const char *ipipe_clock_name(void)
{
	return "tsc";
}

/* Private interface -- Internal use only */

static inline unsigned long __ipipe_ffnz(unsigned long ul)
{
      __asm__("bsrq %1, %0":"=r"(ul)
	      :	"rm"(ul));
      return ul;
}

#endif	/* !__X86_IPIPE_64_H */
