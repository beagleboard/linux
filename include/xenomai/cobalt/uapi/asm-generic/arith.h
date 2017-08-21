/**
 *   Generic arithmetic/conversion routines.
 *   Copyright &copy; 2005 Stelian Pop.
 *   Copyright &copy; 2005 Gilles Chanteperdrix.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA.
 */
#ifndef _COBALT_UAPI_ASM_GENERIC_ARITH_H
#define _COBALT_UAPI_ASM_GENERIC_ARITH_H

#ifndef xnarch_u64tou32
#define xnarch_u64tou32(ull, h, l) ({		\
      union {					\
	      unsigned long long _ull;		\
	      struct endianstruct _s;		\
      } _u;					\
      _u._ull = (ull);				\
      (h) = _u._s._h;				\
      (l) = _u._s._l;				\
})
#endif /* !xnarch_u64tou32 */

#ifndef xnarch_u64fromu32
#define xnarch_u64fromu32(h, l) ({		\
	union {					\
		unsigned long long _ull;	\
		struct endianstruct _s;		\
	} _u;					\
	_u._s._h = (h);				\
	_u._s._l = (l);				\
	_u._ull;				\
})
#endif /* !xnarch_u64fromu32 */

#ifndef xnarch_ullmul
static inline __attribute__((__const__)) unsigned long long
xnarch_generic_ullmul(const unsigned m0, const unsigned m1)
{
	return (unsigned long long) m0 * m1;
}
#define xnarch_ullmul(m0,m1) xnarch_generic_ullmul((m0),(m1))
#endif /* !xnarch_ullmul */

#ifndef xnarch_ulldiv
static inline unsigned long long xnarch_generic_ulldiv (unsigned long long ull,
							const unsigned uld,
							unsigned long *const rp)
{
	const unsigned r = do_div(ull, uld);

	if (rp)
		*rp = r;

	return ull;
}
#define xnarch_ulldiv(ull,uld,rp) xnarch_generic_ulldiv((ull),(uld),(rp))
#endif /* !xnarch_ulldiv */

#ifndef xnarch_uldivrem
#define xnarch_uldivrem(ull,ul,rp) ((unsigned) xnarch_ulldiv((ull),(ul),(rp)))
#endif /* !xnarch_uldivrem */

#ifndef xnarch_divmod64
static inline unsigned long long
xnarch_generic_divmod64(unsigned long long a,
			unsigned long long b,
			unsigned long long *rem)
{
	unsigned long long q;
#if defined(__KERNEL__) && BITS_PER_LONG < 64
	unsigned long long
		xnarch_generic_full_divmod64(unsigned long long a,
					     unsigned long long b,
					     unsigned long long *rem);
	if (b <= 0xffffffffULL) {
		unsigned long r;
		q = xnarch_ulldiv(a, b, &r);
		if (rem)
			*rem = r;
	} else {
		if (a < b) {
			if (rem)
				*rem = a;
			return 0;
		}

		return xnarch_generic_full_divmod64(a, b, rem);
	}
#else /* !(__KERNEL__ && BITS_PER_LONG < 64) */
	q = a / b;
	if (rem)
		*rem = a % b;
#endif  /* !(__KERNEL__ && BITS_PER_LONG < 64) */
	return q;
}
#define xnarch_divmod64(a,b,rp) xnarch_generic_divmod64((a),(b),(rp))
#endif /* !xnarch_divmod64 */

#ifndef xnarch_imuldiv
static inline __attribute__((__const__)) int xnarch_generic_imuldiv(int i,
								    int mult,
								    int div)
{
	/* (int)i = (unsigned long long)i*(unsigned)(mult)/(unsigned)div. */
	const unsigned long long ull = xnarch_ullmul(i, mult);
	return xnarch_uldivrem(ull, div, NULL);
}
#define xnarch_imuldiv(i,m,d) xnarch_generic_imuldiv((i),(m),(d))
#endif /* !xnarch_imuldiv */

#ifndef xnarch_imuldiv_ceil
static inline __attribute__((__const__)) int xnarch_generic_imuldiv_ceil(int i,
									 int mult,
									 int div)
{
	/* Same as xnarch_generic_imuldiv, rounding up. */
	const unsigned long long ull = xnarch_ullmul(i, mult);
	return xnarch_uldivrem(ull + (unsigned)div - 1, div, NULL);
}
#define xnarch_imuldiv_ceil(i,m,d) xnarch_generic_imuldiv_ceil((i),(m),(d))
#endif /* !xnarch_imuldiv_ceil */

/* Division of an unsigned 96 bits ((h << 32) + l) by an unsigned 32 bits.
   Building block for llimd. Without const qualifiers, gcc reload registers
   after each call to uldivrem. */
static inline unsigned long long
xnarch_generic_div96by32(const unsigned long long h,
			 const unsigned l,
			 const unsigned d,
			 unsigned long *const rp)
{
	unsigned long rh;
	const unsigned qh = xnarch_uldivrem(h, d, &rh);
	const unsigned long long t = xnarch_u64fromu32(rh, l);
	const unsigned ql = xnarch_uldivrem(t, d, rp);

	return xnarch_u64fromu32(qh, ql);
}

#ifndef xnarch_llimd
static inline __attribute__((__const__))
unsigned long long xnarch_generic_ullimd(const unsigned long long op,
					 const unsigned m,
					 const unsigned d)
{
	unsigned int oph, opl, tlh, tll;
	unsigned long long th, tl;

	xnarch_u64tou32(op, oph, opl);
	tl = xnarch_ullmul(opl, m);
	xnarch_u64tou32(tl, tlh, tll);
	th = xnarch_ullmul(oph, m);
	th += tlh;

	return xnarch_generic_div96by32(th, tll, d, NULL);
}

static inline __attribute__((__const__)) long long
xnarch_generic_llimd (long long op, unsigned m, unsigned d)
{
	long long ret;
	int sign = 0;

	if (op < 0LL) {
		sign = 1;
		op = -op;
	}
	ret = xnarch_generic_ullimd(op, m, d);

	return sign ? -ret : ret;
}
#define xnarch_llimd(ll,m,d) xnarch_generic_llimd((ll),(m),(d))
#endif /* !xnarch_llimd */

#ifndef _xnarch_u96shift
#define xnarch_u96shift(h, m, l, s) ({		\
	unsigned int _l = (l);			\
	unsigned int _m = (m);			\
	unsigned int _s = (s);			\
	_l >>= _s;				\
	_l |= (_m << (32 - _s));		\
	_m >>= _s;				\
	_m |= ((h) << (32 - _s));		\
	xnarch_u64fromu32(_m, _l);		\
})
#endif /* !xnarch_u96shift */

static inline long long xnarch_llmi(int i, int j)
{
	/* Fast 32x32->64 signed multiplication */
	return (long long) i * j;
}

#ifndef xnarch_llmulshft
/* Fast scaled-math-based replacement for long long multiply-divide */
static inline long long
xnarch_generic_llmulshft(const long long op,
			  const unsigned m,
			  const unsigned s)
{
	unsigned int oph, opl, tlh, tll, thh, thl;
	unsigned long long th, tl;

	xnarch_u64tou32(op, oph, opl);
	tl = xnarch_ullmul(opl, m);
	xnarch_u64tou32(tl, tlh, tll);
	th = xnarch_llmi(oph, m);
	th += tlh;
	xnarch_u64tou32(th, thh, thl);

	return xnarch_u96shift(thh, thl, tll, s);
}
#define xnarch_llmulshft(ll, m, s) xnarch_generic_llmulshft((ll), (m), (s))
#endif /* !xnarch_llmulshft */

#ifdef XNARCH_HAVE_NODIV_LLIMD

/* Representation of a 32 bits fraction. */
struct xnarch_u32frac {
	unsigned long long frac;
	unsigned integ;
};

static inline void xnarch_init_u32frac(struct xnarch_u32frac *const f,
				       const unsigned m,
				       const unsigned d)
{
	/*
	 * Avoid clever compiler optimizations to occur when d is
	 * known at compile-time. The performance of this function is
	 * not critical since it is only called at init time.
	 */
	volatile unsigned vol_d = d;
	f->integ = m / d;
	f->frac = xnarch_generic_div96by32
		(xnarch_u64fromu32(m % d, 0), 0, vol_d, NULL);
}

#ifndef xnarch_nodiv_imuldiv
static inline __attribute__((__const__)) unsigned
xnarch_generic_nodiv_imuldiv(unsigned op, const struct xnarch_u32frac f)
{
	return (xnarch_ullmul(op, f.frac >> 32) >> 32) + f.integ * op;
}
#define xnarch_nodiv_imuldiv(op, f) xnarch_generic_nodiv_imuldiv((op),(f))
#endif /* xnarch_nodiv_imuldiv */

#ifndef xnarch_nodiv_imuldiv_ceil
static inline __attribute__((__const__)) unsigned
xnarch_generic_nodiv_imuldiv_ceil(unsigned op, const struct xnarch_u32frac f)
{
	unsigned long long full = xnarch_ullmul(op, f.frac >> 32) + ~0U;
	return (full >> 32) + f.integ * op;
}
#define xnarch_nodiv_imuldiv_ceil(op, f) \
	xnarch_generic_nodiv_imuldiv_ceil((op),(f))
#endif /* xnarch_nodiv_imuldiv_ceil */

#ifndef xnarch_nodiv_ullimd

#ifndef xnarch_add96and64
#error "xnarch_add96and64 must be implemented."
#endif

static inline __attribute__((__const__)) unsigned long long
xnarch_mul64by64_high(const unsigned long long op, const unsigned long long m)
{
	/* Compute high 64 bits of multiplication 64 bits x 64 bits. */
	register unsigned long long t0, t1, t2, t3;
	register unsigned int oph, opl, mh, ml, t0h, t0l, t1h, t1l, t2h, t2l, t3h, t3l;

	xnarch_u64tou32(op, oph, opl);
	xnarch_u64tou32(m, mh, ml);
	t0 = xnarch_ullmul(opl, ml);
	xnarch_u64tou32(t0, t0h, t0l);
	t3 = xnarch_ullmul(oph, mh);
	xnarch_u64tou32(t3, t3h, t3l);
	xnarch_add96and64(t3h, t3l, t0h, 0, t0l >> 31);
	t1 = xnarch_ullmul(oph, ml);
	xnarch_u64tou32(t1, t1h, t1l);
	xnarch_add96and64(t3h, t3l, t0h, t1h, t1l);
	t2 = xnarch_ullmul(opl, mh);
	xnarch_u64tou32(t2, t2h, t2l);
	xnarch_add96and64(t3h, t3l, t0h, t2h, t2l);

	return xnarch_u64fromu32(t3h, t3l);
}

static inline unsigned long long
xnarch_generic_nodiv_ullimd(const unsigned long long op,
			    const unsigned long long frac,
			    unsigned int integ)
{
	return xnarch_mul64by64_high(op, frac) + integ * op;
}
#define xnarch_nodiv_ullimd(op, f, i)  xnarch_generic_nodiv_ullimd((op),(f), (i))
#endif /* !xnarch_nodiv_ullimd */

#ifndef xnarch_nodiv_llimd
static inline __attribute__((__const__)) long long
xnarch_generic_nodiv_llimd(long long op, unsigned long long frac,
			   unsigned int integ)
{
	long long ret;
	int sign = 0;

	if (op < 0LL) {
		sign = 1;
		op = -op;
	}
	ret = xnarch_nodiv_ullimd(op, frac, integ);

	return sign ? -ret : ret;
}
#define xnarch_nodiv_llimd(ll,frac,integ) xnarch_generic_nodiv_llimd((ll),(frac),(integ))
#endif /* !xnarch_nodiv_llimd */

#endif /* XNARCH_HAVE_NODIV_LLIMD */

static inline void xnarch_init_llmulshft(const unsigned m_in,
					 const unsigned d_in,
					 unsigned *m_out,
					 unsigned *s_out)
{
	/*
	 * Avoid clever compiler optimizations to occur when d is
	 * known at compile-time. The performance of this function is
	 * not critical since it is only called at init time.
	 */
	volatile unsigned int vol_d = d_in;
	unsigned long long mult;

	*s_out = 31;
	while (1) {
		mult = ((unsigned long long)m_in) << *s_out;
		do_div(mult, vol_d);
		if (mult <= 0x7FFFFFFF)
			break;
		(*s_out)--;
	}
	*m_out = (unsigned int)mult;
}

#define xnarch_ullmod(ull,uld,rem)   ({ xnarch_ulldiv(ull,uld,rem); (*rem); })
#define xnarch_uldiv(ull, d)         xnarch_uldivrem(ull, d, NULL)
#define xnarch_ulmod(ull, d)         ({ unsigned long _rem;	\
					xnarch_uldivrem(ull,d,&_rem); _rem; })

#define xnarch_div64(a,b)            xnarch_divmod64((a),(b),NULL)
#define xnarch_mod64(a,b)            ({ unsigned long long _rem; \
					xnarch_divmod64((a),(b),&_rem); _rem; })

#endif /* _COBALT_UAPI_ASM_GENERIC_ARITH_H */
