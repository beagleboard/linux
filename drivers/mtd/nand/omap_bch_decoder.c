/*
 * drivers/mtd/nand/omap_omap_bch_decoder.c
 *
 * Whole BCH ECC Decoder (Post hardware generated syndrome decoding)
 *
 * Copyright (c) 2007 Texas Instruments
 *
 * Author: Sukumar Ghorai <s-ghorai@ti.com
 *		   Michael Fillinger <m-fillinger@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#undef DEBUG

#include <linux/kernel.h>
#include <linux/module.h>

#define mm		13
#define kk_shorten	4096
#define nn		8191	/* Length of codeword, n = 2**mm - 1 */

#define PPP	0x201B	/* Primary Polynomial : x^13 + x^4 + x^3 + x + 1 */
#define P	0x001B	/* With omitted x^13 */
#define POLY	12	/* degree of the primary Polynomial less one */

/**
 * mpy_mod_gf - GALOIS field multiplier
 * Input  : A(x), B(x)
 * Output : A(x)*B(x) mod P(x)
 */
static unsigned int mpy_mod_gf(unsigned int a, unsigned int b)
{
	unsigned int R = 0;
	unsigned int R1 = 0;
	unsigned int k = 0;

	for (k = 0; k < mm; k++) {

		R = (R << 1) & 0x1FFE;
		if (R1 == 1)
			R ^= P;

		if (((a >> (POLY - k)) & 1) == 1)
			R ^= b;

		if (k < POLY)
			R1 = (R >> POLY) & 1;
	}
	return R;
}

/**
 * chien - CHIEN search
 *
 * @location - Error location vector pointer
 *
 * Inputs  : ELP(z)
 *	     No. of found errors
 *	     Size of input codeword
 * Outputs : Up to 8 locations
 *	     No. of errors
 */
static int chien(unsigned int select_4_8, int err_nums,
				unsigned int err[], unsigned int *location)
{
	int i, count; /* Number of dectected errors */
	/* Contains accumulation of evaluation at x^i (i:1->8) */
	unsigned int gammas[8] = {0};
	unsigned int alpha;
	unsigned int bit, ecc_bits;
	unsigned int elp_sum;

	ecc_bits = (select_4_8 == 0) ? 52 : 104;

	/* Start evaluation at Alpha**8192 and decreasing */
	for (i = 0; i < 8; i++)
		gammas[i] = err[i];

	count = 0;
	for (i = 1; (i <= nn) && (count < err_nums); i++) {

		/* Result of evaluation at root */
		elp_sum = 1 ^ gammas[0] ^ gammas[1] ^
				gammas[2] ^ gammas[3] ^
				gammas[4] ^ gammas[5] ^
				gammas[6] ^ gammas[7];

		alpha = PPP >> 1;
		gammas[0] = mpy_mod_gf(gammas[0], alpha);
		alpha = mpy_mod_gf(alpha, (PPP >> 1));	/* x alphha^-2 */
		gammas[1] = mpy_mod_gf(gammas[1], alpha);
		alpha = mpy_mod_gf(alpha, (PPP >> 1));	/* x alphha^-2 */
		gammas[2] = mpy_mod_gf(gammas[2], alpha);
		alpha = mpy_mod_gf(alpha, (PPP >> 1));	/* x alphha^-3 */
		gammas[3] = mpy_mod_gf(gammas[3], alpha);
		alpha = mpy_mod_gf(alpha, (PPP >> 1));	/* x alphha^-4 */
		gammas[4] = mpy_mod_gf(gammas[4], alpha);
		alpha = mpy_mod_gf(alpha, (PPP >> 1));	/* x alphha^-5 */
		gammas[5] = mpy_mod_gf(gammas[5], alpha);
		alpha = mpy_mod_gf(alpha, (PPP >> 1));	/* x alphha^-6 */
		gammas[6] = mpy_mod_gf(gammas[6], alpha);
		alpha = mpy_mod_gf(alpha, (PPP >> 1));	/* x alphha^-7 */
		gammas[7] = mpy_mod_gf(gammas[7], alpha);

		if (elp_sum == 0) {
			/* calculate bit position in main data area */
			bit = ((i-1) & ~7)|(7-((i-1) & 7));
			if (i >= 2 * ecc_bits)
				location[count++] =
					kk_shorten - (bit - 2 * ecc_bits) - 1;
		}
	}

	/* Failure: No. of detected errors != No. or corrected errors */
	if (count != err_nums) {
		count = -1;
		printk(KERN_ERR "BCH decoding failed\n");
	}
	for (i = 0; i < count; i++)
		pr_debug("%d ", location[i]);

	return count;
}

/* synd : 16 Syndromes
 * return: gamaas - Coefficients to the error polynomial
 * return: : Number of detected errors
*/
static unsigned int berlekamp(unsigned int select_4_8,
			unsigned int synd[], unsigned int err[])
{
	int loop, iteration;
	unsigned int LL = 0;		/* Detected errors */
	unsigned int d = 0;	/* Distance between Syndromes and ELP[n](z) */
	unsigned int invd = 0;		/* Inverse of d */
	/* Intermediate ELP[n](z).
	 * Final ELP[n](z) is Error Location Polynomial
	 */
	unsigned int gammas[16] = {0};
	/* Intermediate normalized ELP[n](z) : D[n](z) */
	unsigned int D[16] = {0};
	/* Temporary value that holds an ELP[n](z) coefficient */
	unsigned int next_gamma = 0;

	int e = 0;
	unsigned int sign = 0;
	unsigned int u = 0;
	unsigned int v = 0;
	unsigned int C1 = 0, C2 = 0;
	unsigned int ss = 0;
	unsigned int tmp_v = 0, tmp_s = 0;
	unsigned int tmp_poly;

	/*-------------- Step 0 ------------------*/
	for (loop = 0; loop < 16; loop++)
		gammas[loop] = 0;
	gammas[0] = 1;
	D[1] = 1;

	iteration = 0;
	LL = 0;
	while ((iteration < ((select_4_8+1)*2*4)) &&
			(LL <= ((select_4_8+1)*4))) {

		pr_debug("\nIteration.............%d\n", iteration);
		d = 0;
		/* Step: 0 */
		for (loop = 0; loop <= LL; loop++) {
			tmp_poly = mpy_mod_gf(
					gammas[loop], synd[iteration - loop]);
			d ^= tmp_poly;
			pr_debug("%02d. s=0 LL=%x poly %x\n",
					loop, LL, tmp_poly);
		}

		/* Step 1: 1 cycle only to perform inversion */
		v = d << 1;
		e = -1;
		sign = 1;
		ss = 0x2000;
		invd = 0;
		u = PPP;
		for (loop = 0; (d != 0) && (loop <= (2 * POLY)); loop++) {
			pr_debug("%02d. s=1 LL=%x poly NULL\n",
						loop, LL);
			C1 = (v >> 13) & 1;
			C2 = C1 & sign;

			sign ^= C2 ^ (e == 0);

			tmp_v = v;
			tmp_s = ss;

			if (C1 == 1) {
				v ^= u;
				ss ^= invd;
			}
			v = (v << 1) & 0x3FFF;
			if (C2 == 1) {
				u = tmp_v;
				invd = tmp_s;
				e = -e;
			}
			invd >>= 1;
			e--;
		}

		for (loop = 0; (d != 0) && (loop <= (iteration + 1)); loop++) {
			/* Step 2
			 * Interleaved with Step 3, if L<(n-k)
			 * invd: Update of ELP[n](z) = ELP[n-1](z) - d.D[n-1](z)
			 */

			/* Holds value of ELP coefficient until precedent
			 * value does not have to be used anymore
			 */
			tmp_poly = mpy_mod_gf(d, D[loop]);
			pr_debug("%02d. s=2 LL=%x poly %x\n",
						loop, LL, tmp_poly);

			next_gamma = gammas[loop] ^ tmp_poly;
			if ((2 * LL) < (iteration + 1)) {
				/* Interleaving with Step 3
				 * for parallelized update of ELP(z) and D(z)
				 */
			} else {
				/* Update of ELP(z) only -> stay in Step 2 */
				gammas[loop] = next_gamma;
				if (loop == (iteration + 1)) {
					/* to step 4 */
					break;
				}
			}

			/* Step 3
			 * Always interleaved with Step 2 (case when L<(n-k))
			 * Update of D[n-1](z) = ELP[n-1](z)/d
			 */
			D[loop] = mpy_mod_gf(gammas[loop], invd);
			pr_debug("%02d. s=3 LL=%x poly %x\n",
					loop, LL, D[loop]);

			/* Can safely update ELP[n](z) */
			gammas[loop] = next_gamma;

			if (loop == (iteration + 1)) {
				/* If update finished */
				LL = iteration - LL + 1;
				/* to step 4 */
				break;
			}
			/* Else, interleaving to step 2*/
		}

		/* Step 4: Update D(z): i:0->L */
		/* Final update of D[n](z) = D[n](z).z*/
		for (loop = 0; loop < 15; loop++) /* Left Shift */
			D[15 - loop] = D[14 - loop];

		D[0] = 0;

		iteration++;
	} /* while */

	/* Processing finished, copy ELP to final registers : 0->2t-1*/
	for (loop = 0; loop < 8; loop++)
		err[loop] = gammas[loop+1];

	pr_debug("\n Err poly:");
	for (loop = 0; loop < 8; loop++)
		pr_debug("0x%x ", err[loop]);

	return LL;
}

/*
 * syndrome - Generate syndrome components from hw generate syndrome
 * r(x) = c(x) + e(x)
 * s(x) = c(x) mod g(x) + e(x) mod g(x) =  e(x) mod g(x)
 * so receiver checks if the syndrome s(x) = r(x) mod g(x) is equal to zero.
 * unsigned int s[16]; - Syndromes
 */
static void syndrome(unsigned int select_4_8,
					unsigned char *ecc, unsigned int syn[])
{
	unsigned int k, l, t;
	unsigned int alpha_bit, R_bit;
	int ecc_pos, ecc_min;

	/* 2t-1 = 15 (for t=8) minimal polynomials of the first 15 powers of a
	 * primitive elemmants of GF(m); Even powers minimal polynomials are
	 * duplicate of odd powers' minimal polynomials.
	 * Odd powers of alpha (1 to 15)
	 */
	unsigned int pow_alpha[8] = {0x0002, 0x0008, 0x0020, 0x0080,
				 0x0200, 0x0800, 0x001B, 0x006C};

	pr_debug("\n ECC[0..n]: ");
	for (k = 0; k < 13; k++)
		pr_debug("0x%x ", ecc[k]);

	if (select_4_8 == 0) {
		t = 4;
		ecc_pos = 55; /* bits(52-bits): 55->4 */
		ecc_min = 4;
	} else {
		t = 8;
		ecc_pos = 103; /* bits: 103->0 */
		ecc_min = 0;
	}

	/* total numbber of syndrom to be used is 2t */
	/* Step1: calculate the odd syndrome(s) */
	R_bit = ((ecc[ecc_pos/8] >> (7 - ecc_pos%8)) & 1);
	ecc_pos--;
	for (k = 0; k < t; k++)
		syn[2 * k] = R_bit;

	while (ecc_pos >= ecc_min) {
		R_bit = ((ecc[ecc_pos/8] >> (7 - ecc_pos%8)) & 1);
		ecc_pos--;

		for (k = 0; k < t; k++) {
			/* Accumulate value of x^i at alpha^(2k+1) */
			if (R_bit == 1)
				syn[2*k] ^= pow_alpha[k];

			/* Compute a**(2k+1), using LSFR */
			for (l = 0; l < (2 * k + 1); l++) {
				alpha_bit = (pow_alpha[k] >> POLY) & 1;
				pow_alpha[k] = (pow_alpha[k] << 1) & 0x1FFF;
				if (alpha_bit == 1)
					pow_alpha[k] ^= P;
			}
		}
	}

	/* Step2: calculate the even syndrome(s)
	 * Compute S(a), where a is an even power of alpha
	 * Evenry even power of primitive element has the same minimal
	 * polynomial as some odd power of elemets.
	 * And based on S(a^2) = S^2(a)
	 */
	for (k = 0; k < t; k++)
		syn[2*k+1] = mpy_mod_gf(syn[k], syn[k]);

	pr_debug("\n Syndromes: ");
	for (k = 0; k < 16; k++)
		pr_debug("0x%x ", syn[k]);
}

/**
 * decode_bch - BCH decoder for 4- and 8-bit error correction
 *
 * @ecc - ECC syndrome generated by hw BCH engine
 * @err_loc - pointer to error location array
 *
 * This function does post sydrome generation (hw generated) decoding
 * for:-
 * Dimension of Galoise Field: m = 13
 * Length of codeword: n = 2**m - 1
 * Number of errors that can be corrected: 4- or 8-bits
 * Length of information bit: kk = nn - rr
 */
int decode_bch(int select_4_8, unsigned char *ecc, unsigned int *err_loc)
{
	int no_of_err;
	unsigned int syn[16] = {0,};	/* 16 Syndromes */
	unsigned int err_poly[8] = {0,};
	/* Coefficients to the error polynomial
	 * ELP(x) = 1 + err0.x + err1.x^2 + ... + err7.x^8
	 */

	/* Decoting involes three steps
	 * 1. Compute the syndrom from teh received codeword,
	 * 2. Find the error location polynomial from a set of equations
	 *     derived from the syndrome,
	 * 3. Use the error location polynomial to identify errants bits,
	 *
	 * And correcttion done by bit flips using error locaiton and expected
	 * to be outseide of this implementation.
	 */
	syndrome(select_4_8, ecc, syn);
	no_of_err = berlekamp(select_4_8, syn, err_poly);
	if (no_of_err <= (4 << select_4_8))
		no_of_err = chien(select_4_8, no_of_err, err_poly, err_loc);

	return no_of_err;
}
EXPORT_SYMBOL(decode_bch);
MODULE_LICENSE("GPL");
