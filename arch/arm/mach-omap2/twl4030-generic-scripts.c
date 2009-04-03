/*
 * arch/arm/mach-omap2/twl4030-generic-scripts.c
 *
 * Generic power control scripts for TWL4030
 *
 * Copyright (C) 2008 Nokia Corporation
 * Copyright (C) 2006 Texas Instruments, Inc
 *
 * Written by 	Kalle Jokiniemi
 *		Peter De Schrijver <peter.de-schrijver@nokia.com>
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifdef CONFIG_TWL4030_POWER

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/i2c/twl4030.h>

/*
 * This script instructs twl4030 to first put the Reset and Control (RC)
 * resources to sleep and then all the other resources.
 */

static struct twl4030_ins sleep_on_seq[] __initdata = {
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_RC, RES_TYPE_ALL, RES_TYPE2_R0,
			RES_STATE_SLEEP), 4},
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_ALL, RES_TYPE2_R0,
			RES_STATE_SLEEP), 4},
};

static struct twl4030_script sleep_on_script __initdata = {
	.script = sleep_on_seq,
	.size   = ARRAY_SIZE(sleep_on_seq),
	.flags  = TRITON_SLEEP_SCRIPT,
};

/*
 * This script instructs twl4030 to first enable CLKEN, then wakeup the
 * regulators and then all other resources.
 */

static struct twl4030_ins wakeup_seq[] __initdata = {
	{MSG_SINGULAR(DEV_GRP_NULL, 0x17, RES_STATE_ACTIVE), 0x30},
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_PP_PR, RES_TYPE_ALL, RES_TYPE2_R0,
			RES_STATE_ACTIVE), 0x37},
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_ALL, RES_TYPE2_R0,
			RES_STATE_ACTIVE), 0x2},
};

static struct twl4030_script wakeup_script __initdata = {
	.script = wakeup_seq,
	.size   = ARRAY_SIZE(wakeup_seq),
	.flags  = TRITON_WAKEUP12_SCRIPT | TRITON_WAKEUP3_SCRIPT,
};

static struct twl4030_script *twl4030_scripts[] __initdata = {
	&sleep_on_script,
	&wakeup_script,
};

struct twl4030_power_data generic3430_t2scripts_data __initdata = {
	.scripts        = twl4030_scripts,
	.size           = ARRAY_SIZE(twl4030_scripts),
};


#endif /* CONFIG_TWL4030_POWER */
