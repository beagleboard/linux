// SPDX-License-Identifier: GPL-2.0
/*
 * Simple KUnit suite for math helper funcs that are always enabled.
 *
 * Copyright (C) 2020, Google LLC.
 * Author: Daniel Latypov <dlatypov@google.com>
 */

#include <kunit/test.h>
#include <linux/gcd.h>
#include <linux/lcm.h>
#include <linux/math.h>
#include <linux/module.h>
#include <linux/reciprocal_div.h>
#include <linux/types.h>

static void abs_test(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, abs((char)0), (char)0);
	KUNIT_EXPECT_EQ(test, abs((char)42), (char)42);
	KUNIT_EXPECT_EQ(test, abs((char)-42), (char)42);

	/* The expression in the macro is actually promoted to an int. */
	KUNIT_EXPECT_EQ(test, abs((short)0),  0);
	KUNIT_EXPECT_EQ(test, abs((short)42),  42);
	KUNIT_EXPECT_EQ(test, abs((short)-42),  42);

	KUNIT_EXPECT_EQ(test, abs(0),  0);
	KUNIT_EXPECT_EQ(test, abs(42),  42);
	KUNIT_EXPECT_EQ(test, abs(-42),  42);

	KUNIT_EXPECT_EQ(test, abs(0L), 0L);
	KUNIT_EXPECT_EQ(test, abs(42L), 42L);
	KUNIT_EXPECT_EQ(test, abs(-42L), 42L);

	KUNIT_EXPECT_EQ(test, abs(0LL), 0LL);
	KUNIT_EXPECT_EQ(test, abs(42LL), 42LL);
	KUNIT_EXPECT_EQ(test, abs(-42LL), 42LL);

	/* Unsigned types get casted to signed. */
	KUNIT_EXPECT_EQ(test, abs(0ULL), 0LL);
	KUNIT_EXPECT_EQ(test, abs(42ULL), 42LL);
}

static void int_sqrt_test(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, int_sqrt(0UL), 0UL);
	KUNIT_EXPECT_EQ(test, int_sqrt(1UL), 1UL);
	KUNIT_EXPECT_EQ(test, int_sqrt(4UL), 2UL);
	KUNIT_EXPECT_EQ(test, int_sqrt(5UL), 2UL);
	KUNIT_EXPECT_EQ(test, int_sqrt(8UL), 2UL);
	KUNIT_EXPECT_EQ(test, int_sqrt(1UL << 30), 1UL << 15);
}

static void round_up_test(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, round_up(0, 1), 0);
	KUNIT_EXPECT_EQ(test, round_up(1, 2), 2);
	KUNIT_EXPECT_EQ(test, round_up(3, 2), 4);
	KUNIT_EXPECT_EQ(test, round_up((1 << 30) - 1, 2), 1 << 30);
	KUNIT_EXPECT_EQ(test, round_up((1 << 30) - 1, 1 << 29), 1 << 30);
}

static void round_down_test(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, round_down(0, 1), 0);
	KUNIT_EXPECT_EQ(test, round_down(1, 2), 0);
	KUNIT_EXPECT_EQ(test, round_down(3, 2), 2);
	KUNIT_EXPECT_EQ(test, round_down((1 << 30) - 1, 2), (1 << 30) - 2);
	KUNIT_EXPECT_EQ(test, round_down((1 << 30) - 1, 1 << 29), 1 << 29);
}

/* These versions can round to numbers that aren't a power of two */
static void roundup_test(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, roundup(0, 1), 0);
	KUNIT_EXPECT_EQ(test, roundup(1, 2), 2);
	KUNIT_EXPECT_EQ(test, roundup(3, 2), 4);
	KUNIT_EXPECT_EQ(test, roundup((1 << 30) - 1, 2), 1 << 30);
	KUNIT_EXPECT_EQ(test, roundup((1 << 30) - 1, 1 << 29), 1 << 30);

	KUNIT_EXPECT_EQ(test, roundup(3, 2), 4);
	KUNIT_EXPECT_EQ(test, roundup(4, 3), 6);
}

static void rounddown_test(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, rounddown(0, 1), 0);
	KUNIT_EXPECT_EQ(test, rounddown(1, 2), 0);
	KUNIT_EXPECT_EQ(test, rounddown(3, 2), 2);
	KUNIT_EXPECT_EQ(test, rounddown((1 << 30) - 1, 2), (1 << 30) - 2);
	KUNIT_EXPECT_EQ(test, rounddown((1 << 30) - 1, 1 << 29), 1 << 29);

	KUNIT_EXPECT_EQ(test, rounddown(3, 2), 2);
	KUNIT_EXPECT_EQ(test, rounddown(4, 3), 3);
}

static void div_round_up_test(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, DIV_ROUND_UP(0, 1), 0);
	KUNIT_EXPECT_EQ(test, DIV_ROUND_UP(20, 10), 2);
	KUNIT_EXPECT_EQ(test, DIV_ROUND_UP(21, 10), 3);
	KUNIT_EXPECT_EQ(test, DIV_ROUND_UP(21, 20), 2);
	KUNIT_EXPECT_EQ(test, DIV_ROUND_UP(21, 99), 1);
}

static void div_round_closest_test(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, DIV_ROUND_CLOSEST(0, 1), 0);
	KUNIT_EXPECT_EQ(test, DIV_ROUND_CLOSEST(20, 10), 2);
	KUNIT_EXPECT_EQ(test, DIV_ROUND_CLOSEST(21, 10), 2);
	KUNIT_EXPECT_EQ(test, DIV_ROUND_CLOSEST(25, 10), 3);
}

/* Generic test case for unsigned long inputs. */
struct test_case {
	unsigned long a, b;
	unsigned long result;
};

static struct test_case gcd_cases[] = {
	{
		.a = 0, .b = 0,
		.result = 0,
	},
	{
		.a = 0, .b = 1,
		.result = 1,
	},
	{
		.a = 2, .b = 2,
		.result = 2,
	},
	{
		.a = 2, .b = 4,
		.result = 2,
	},
	{
		.a = 3, .b = 5,
		.result = 1,
	},
	{
		.a = 3 * 9, .b = 3 * 5,
		.result = 3,
	},
	{
		.a = 3 * 5 * 7, .b = 3 * 5 * 11,
		.result = 15,
	},
	{
		.a = 1 << 21,
		.b = (1 << 21) - 1,
		.result = 1,
	},
};

KUNIT_ARRAY_PARAM(gcd, gcd_cases, NULL);

static void gcd_test(struct kunit *test)
{
	const char *message_fmt = "gcd(%lu, %lu)";
	const struct test_case *test_param = test->param_value;

	KUNIT_EXPECT_EQ_MSG(test, test_param->result,
			    gcd(test_param->a, test_param->b),
			    message_fmt, test_param->a,
			    test_param->b);

	if (test_param->a == test_param->b)
		return;

	/* gcd(a,b) == gcd(b,a) */
	KUNIT_EXPECT_EQ_MSG(test, test_param->result,
			    gcd(test_param->b, test_param->a),
			    message_fmt, test_param->b,
			    test_param->a);
}

static struct test_case lcm_cases[] = {
	{
		.a = 0, .b = 0,
		.result = 0,
	},
	{
		.a = 0, .b = 1,
		.result = 0,
	},
	{
		.a = 1, .b = 2,
		.result = 2,
	},
	{
		.a = 2, .b = 2,
		.result = 2,
	},
	{
		.a = 3 * 5, .b = 3 * 7,
		.result = 3 * 5 * 7,
	},
};

KUNIT_ARRAY_PARAM(lcm, lcm_cases, NULL);

static void lcm_test(struct kunit *test)
{
	const char *message_fmt = "lcm(%lu, %lu)";
	const struct test_case *test_param = test->param_value;

	KUNIT_EXPECT_EQ_MSG(test, test_param->result,
			    lcm(test_param->a, test_param->b),
			    message_fmt, test_param->a,
			    test_param->b);

	if (test_param->a == test_param->b)
		return;

	/* lcm(a,b) == lcm(b,a) */
	KUNIT_EXPECT_EQ_MSG(test, test_param->result,
			    lcm(test_param->b, test_param->a),
			    message_fmt, test_param->b,
			    test_param->a);
}

struct u32_test_case {
	u32 a, b;
	u32 result;
};

static struct u32_test_case reciprocal_div_cases[] = {
	{
		.a = 0, .b = 1,
		.result = 0,
	},
	{
		.a = 42, .b = 20,
		.result = 2,
	},
	{
		.a = 42, .b = 9999,
		.result = 0,
	},
	{
		.a = (1 << 16), .b = (1 << 14),
		.result = 1 << 2,
	},
};

KUNIT_ARRAY_PARAM(reciprocal_div, reciprocal_div_cases, NULL);

static void reciprocal_div_test(struct kunit *test)
{
	const struct u32_test_case *test_param = test->param_value;
	struct reciprocal_value rv = reciprocal_value(test_param->b);

	KUNIT_EXPECT_EQ_MSG(test, test_param->result,
			    reciprocal_divide(test_param->a, rv),
			    "reciprocal_divide(%u, %u)",
			    test_param->a, test_param->b);
}

static void reciprocal_scale_test(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, reciprocal_scale(0u, 100), 0u);
	KUNIT_EXPECT_EQ(test, reciprocal_scale(1u, 100), 0u);
	KUNIT_EXPECT_EQ(test, reciprocal_scale(1u << 4, 1 << 28), 1u);
	KUNIT_EXPECT_EQ(test, reciprocal_scale(1u << 16, 1 << 28), 1u << 12);
	KUNIT_EXPECT_EQ(test, reciprocal_scale(~0u, 1 << 28), (1u << 28) - 1);
}

static struct kunit_case math_test_cases[] = {
	KUNIT_CASE(abs_test),
	KUNIT_CASE(int_sqrt_test),
	KUNIT_CASE(round_up_test),
	KUNIT_CASE(round_down_test),
	KUNIT_CASE(roundup_test),
	KUNIT_CASE(rounddown_test),
	KUNIT_CASE(div_round_up_test),
	KUNIT_CASE(div_round_closest_test),
	KUNIT_CASE_PARAM(gcd_test, gcd_gen_params),
	KUNIT_CASE_PARAM(lcm_test, lcm_gen_params),
	KUNIT_CASE_PARAM(reciprocal_div_test, reciprocal_div_gen_params),
	KUNIT_CASE(reciprocal_scale_test),
	{}
};

static struct kunit_suite math_test_suite = {
	.name = "lib-math",
	.test_cases = math_test_cases,
};

kunit_test_suites(&math_test_suite);

MODULE_DESCRIPTION("Math helper functions kunit test");
MODULE_LICENSE("GPL");
