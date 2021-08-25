/*
 * Copyright &copy; 2005 Gilles Chanteperdrix.
 *
 * Xenomai is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Xenomai is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Xenomai; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 * 02111-1307, USA.
 */
#include <linux/module.h>

/**
 * @ingroup cobalt_core
 * @defgroup cobalt_core_arith In-kernel arithmetics
 *
 * A collection of helpers performing arithmetics not implicitly
 * available from kernel context via GCC helpers. Many of these
 * routines enable 64bit arithmetics on 32bit systems. Xenomai
 * architecture ports normally implement the performance critical ones
 * in hand-crafted assembly code (see
 * kernel/cobalt/arch/\<arch\>/include/asm/xenomai/uapi/arith.h).
 * @{
 */

/**
 * Architecture-independent div64 operation with remainder.
 *
 * @param a dividend
 *
 * @param b divisor
 *
 * @param rem if non-NULL, a pointer to a 64bit variable for
 * collecting the remainder from the division.
 */
unsigned long long xnarch_generic_full_divmod64(unsigned long long a,
						unsigned long long b,
						unsigned long long *rem)
{
	unsigned long long q = 0, r = a;
	int i;

	for (i = fls(a >> 32) - fls(b >> 32), b <<= i; i >= 0; i--, b >>= 1) {
		q <<= 1;
		if (b <= r) {
			r -= b;
			q++;
		}
	}

	if (rem)
		*rem = r;
	return q;
}
EXPORT_SYMBOL_GPL(xnarch_generic_full_divmod64);

/** @} */
