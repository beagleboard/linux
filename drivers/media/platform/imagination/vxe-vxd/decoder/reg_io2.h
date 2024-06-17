/* SPDX-License-Identifier: GPL-2.0 */
/*
 * IMG MSVDX core Registers
 * This file contains the MSVDX_CORE_REGS_H Definitions
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Amit Makani <amit.makani@ti.com>
 *
 * Re-written for upstreamimg
 *	Sidraya Jayagond <sidraya.bj@pathpartnertech.com>
 */

#ifndef REG_IO2_H_
#define REG_IO2_H_

#define IMG_ASSERT(expected) \
	((void)((expected) || \
	       (pr_err("Assertion failed: %s, file %s, line %d\n", \
			   #expected, __FILE__, __LINE__), dump_stack(), 0)))

/* This macro is used to extract a field from a register. */
#define REGIO_READ_FIELD(regval, group, reg, field)     \
	(((regval) & group ## _ ## reg ## _ ## field ## _MASK) >> \
	 group ## _ ## reg ## _ ## field ## _SHIFT)

#if (defined WIN32 || defined __linux__) && !defined NO_REGIO_CHECK_FIELD_VALUE
/*
 * Only provide register field range checking for Windows and
 * Linux builds
 * Simple range check that ensures that if bits outside the valid field
 * range are set, that the provided value is at least consistent with a
 * negative value (i.e.: all top bits are set to 1).
 * Cannot perform more comprehensive testing without knowing
 * whether field
 * should be interpreted as signed or unsigned.
 */
#define REGIO_CHECK_VALUE_FITS_WITHIN_FIELD(group, reg, field, value, type) \
	{ \
		type __value = 0; \
		unsigned int temp = 0; \
		__value = value; \
		temp = (unsigned int)(__value); \
		if (temp > group ## _ ## reg ## _ ## field ## _LSBMASK) { \
			IMG_ASSERT((((unsigned int)__value) & \
			(unsigned int)~(group ## _ ## reg ## _ ## field ## _LSBMASK)) == \
			(unsigned int)~(group ## _ ## reg ## _ ## field ## _LSBMASK));  \
		}                                                       \
	}
#else
#define REGIO_CHECK_VALUE_FITS_WITHIN_FIELD(group, reg, field, value, type)
#endif

/* This macro is used to update the value of a field in a register. */
#define REGIO_WRITE_FIELD(regval, group, reg, field, value, reg_type, val_type) \
	{                                                               \
		reg_type __regval = regval; \
		val_type __value = value; \
		REGIO_CHECK_VALUE_FITS_WITHIN_FIELD(group, reg, field, __value, val_type); \
		(regval) =                                                      \
		((__regval) & ~(group ## _ ## reg ## _ ## field ## _MASK)) |              \
		(((unsigned int)(__value) << (group ## _ ## reg ## _ ## field ## _SHIFT)) & \
		(group ## _ ## reg ## _ ## field ## _MASK));      \
	}

/* This macro is used to update the value of a field in a register. */
#define REGIO_WRITE_FIELD_LITE(regval, group, reg, field, value, type)  \
{                                                                       \
	type __value = value; \
	REGIO_CHECK_VALUE_FITS_WITHIN_FIELD(group, reg, field, __value, type); \
	(regval) |= ((unsigned int)(__value) << (group ## _ ## reg ## _ ## field ## _SHIFT)); \
}

#endif /* REG_IO2_H_ */
