#ifndef __TWL4030_GENERIC_SCRIPTS_H
#define __TWL4030_GENERIC_SCRIPTS_H

#include <linux/i2c/twl4030.h>

#ifdef CONFIG_TWL4030_POWER
extern struct twl4030_power_data generic3430_t2scripts_data;
#define GENERIC3430_T2SCRIPTS_DATA &generic3430_t2scripts_data
#else
#define GENERIC3430_T2SCRIPTS_DATA NULL
#endif /* CONFIG_TWL4030_POWER */

#endif
