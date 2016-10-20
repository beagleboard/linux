/*
 * Calc mult/shift coefficients for cycles2ns conversation
 *
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com
 * Author: Grygorii Strashko <grygorii.strashko@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>

#include <asm/types.h>
#include <limits.h>

#define NSEC_PER_SEC	1000000000L
static int verbose;

static __u64 cyclecounter_cyc2ns(
		      __u64 cycles, __u64 mask, __u32 mult, __u32 shift)
{
	__u64 ns = (__u64)cycles;

	ns = (ns * mult);
	return ns >> shift;
}

/**
 * clocks_calc_mult_shift - calculate mult/shift factors for scaled math of
 *		 clocks
 * @mult:	pointer to mult variable
 * @shift:	pointer to shift variable
 * @from:	frequency to convert from
 * @to:		frequency to convert to
 * @maxsec:	guaranteed runtime conversion range in seconds
 *
 * The function evaluates the shift/mult pair for the scaled math
 * operations of clocksources and clockevents.
 *
 * @to and @from are frequency values in HZ. For clock sources @to is
 * NSEC_PER_SEC == 1GHz and @from is the counter frequency. For clock
 * event @to is the counter frequency and @from is NSEC_PER_SEC.
 *
 * The @maxsec conversion range argument controls the time frame in
 * seconds which must be covered by the runtime conversion with the
 * calculated mult and shift factors. This guarantees that no 64bit
 * overflow happens when the input value of the conversion is
 * multiplied with the calculated mult factor. Larger ranges may
 * reduce the conversion accuracy by chosing smaller mult and shift
 * factors.
 */
static void
clocks_calc_mult_shift(__u32 *mult, __u32 *shift, __u32 from, __u32 to,
		       __u32 maxsec, __u64 mask)
{
	__u64 tmp;
	__u32 sft, sftacc = 32;
	__u64 res_ns;
	int i;
	int is_good;

	printf("start calc from:%u to:%u maxsec:%u\n ", from, to, maxsec);

	/*
	 * Calculate the shift factor which is limiting the conversion
	 * range:
	 */
	tmp = ((__u64)maxsec * from) >> 32;
	while (tmp) {
		tmp >>= 1;
		sftacc--;
	}

	printf("sftacc: %u\n", sftacc);

	/*
	* Find the conversion shift/mult pair which has the best
	* accuracy and fits the maxsec conversion range:
	*/
	for (sft = 32; sft > 0; sft--) {
		tmp = (__u64)to << sft;
		tmp += from / 2;
		tmp /=  from;
		if ((tmp >> sftacc) != 0)
			continue;
		res_ns = cyclecounter_cyc2ns(from, mask, (__u32)tmp, sft);
		is_good = res_ns == to;
		if (is_good || verbose)
			printf("sft:%u \t\tmult:%llu \tns: %llu \t%s \t%s\n",
			       sft, tmp, res_ns,
			       (tmp >> sftacc) == 0 ? "+" : "-",
			       is_good ? "good" : "bad");
		if (res_ns < to) {
			i = 1;
			while (res_ns < to) {
				tmp += 1;
				res_ns = cyclecounter_cyc2ns(from, mask,
							     (__u32)tmp, sft);
				is_good = res_ns == to;
				if (is_good || verbose)
					printf("sft:%u try:%u \tmult:%llu \tns: %llu \t%s \t%s\n",
					       sft, i, tmp, res_ns,
					       (tmp >> sftacc) == 0 ? "+" : "-",
					       is_good ? "good" : "bad");
				i++;
			}
		} else if (res_ns > to) {
			i = 1;
			while (res_ns > to) {
				tmp -= 1;
				res_ns = cyclecounter_cyc2ns(from, mask,
							     (__u32)tmp, sft);
				is_good = res_ns == to;
				if (is_good || verbose)
					printf("sft:%u try:%u \tmult:%llu \tns: %llu \t%s \t%s\n",
					       sft, i, tmp, res_ns,
					       (tmp >> sftacc) == 0 ? "+" : "-",
					       is_good ? "good" : "bad");
				i++;
			}
		}
	}
	*mult = tmp;
	*shift = sft;
}

#define CLOCKSOURCE_MASK(bits) (__u64)((bits) < 64 ? ((1ULL << (bits)) - 1) : -1)

static void clocksource_update_freq_scale(__u32 freq, __u32 width)
{
	__u64 sec;
	__u64 mask;
	__u32 mult;
	__u32 shift;
	/*
	 * Calc the maximum number of seconds which we can run before
	 * wrapping around. For clocksources which have a mask > 32-bit
	 * we need to limit the max sleep time to have a good
	 * conversion precision. 10 minutes is still a reasonable
	 * amount. That results in a shift value of 24 for a
	 * clocksource with mask >= 40-bit and f >= 4GHz. That maps to
	 * ~ 0.06ppm granularity for NTP.
	 */
	mask = CLOCKSOURCE_MASK(width);
	printf(" width: %u mask: %016llx\n", width, mask);
	sec = mask;
	sec /= freq;
	if (!sec)
		sec = 1;
	else if (sec > 600 && mask > UINT_MAX)
		sec = 600;

	clocks_calc_mult_shift(&mult, &shift, freq,
			       NSEC_PER_SEC, sec, mask);
}

static void usage(char *progname)
{
	fprintf(stderr,
		"Calc mult/shift coefficients to be used by timecounter/cyclecounter\n"
		"for cycles2ns conversation:\n"
		"  ns = (cycles * mult) >> shift\n\n"
		"usage: %s [options]\n"
		" -f freq	frequency\n"
		" -w width	Counter width (optional), default 32 bit\n"
		" -h		prints this message\n"
		" -v		verbose output\n",
		progname);
}

int main(int argc, char **argv)
{
	char *progname;
	__u32 freq = 0;
	__u32 width = 32;
	int c;

	progname = strrchr(argv[0], '/');
	progname = progname ? 1 + progname : argv[0];
	while (EOF != (c = getopt(argc, argv, "f:w:hv"))) {
		switch (c) {
		case 'f':
			freq = atoi(optarg);
			break;
		case 'w':
			width = atoi(optarg);
			break;
		case 'v':
			verbose = 1;
			break;
		case 'h':
			usage(progname);
			return 0;
		case '?':
		default:
			usage(progname);
			return -1;
		}
	}

	if (!freq) {
		fprintf(stderr, "missed -f option\n");
		usage(progname);
		return -1;
	}

	clocksource_update_freq_scale(freq, width);

	return 0;
}
