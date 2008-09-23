/*
 * linux/drivers/i2c/chips/twl4030-power.c
 *
 * Handle TWL4030 Power initialization
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

#include <linux/module.h>
#include <linux/pm.h>
#include <linux/i2c/twl4030.h>

#include <asm/mach-types.h>

#define PWR_P1_SW_EVENTS	0x10
#define PWR_DEVOFF	(1<<0)

#define PHY_TO_OFF_PM_MASTER(p)		(p - 0x36)
#define PHY_TO_OFF_PM_RECIEVER(p)	(p - 0x5b)

/* resource - hfclk */
#define R_HFCLKOUT_DEV_GRP 	PHY_TO_OFF_PM_RECIEVER(0xe6)

/* PM events */
#define R_P1_SW_EVENTS		PHY_TO_OFF_PM_MASTER(0x46)
#define R_P2_SW_EVENTS		PHY_TO_OFF_PM_MASTER(0x47)
#define R_P3_SW_EVENTS		PHY_TO_OFF_PM_MASTER(0x48)
#define R_CFG_P1_TRANSITION	PHY_TO_OFF_PM_MASTER(0x36)
#define R_CFG_P2_TRANSITION	PHY_TO_OFF_PM_MASTER(0x37)
#define R_CFG_P3_TRANSITION	PHY_TO_OFF_PM_MASTER(0x38)

#define LVL_WAKEUP	0x08

#define ENABLE_WARMRESET (1<<4)

/* sequence script */

#define END_OF_SCRIPT		0x3f

#define R_SEQ_ADD_A2S		PHY_TO_OFF_PM_MASTER(0x55)
#define R_SEQ_ADD_SA12		PHY_TO_OFF_PM_MASTER(0x56)
#define	R_SEQ_ADD_S2A3		PHY_TO_OFF_PM_MASTER(0x57)
#define	R_SEQ_ADD_WARM		PHY_TO_OFF_PM_MASTER(0x58)
#define R_MEMORY_ADDRESS	PHY_TO_OFF_PM_MASTER(0x59)
#define R_MEMORY_DATA		PHY_TO_OFF_PM_MASTER(0x5a)

/* Power bus message definitions */

#define DEV_GRP_NULL		0x0
#define DEV_GRP_P1		0x1
#define DEV_GRP_P2		0x2
#define DEV_GRP_P3		0x4

#define RES_GRP_RES		0x0
#define RES_GRP_PP		0x1
#define RES_GRP_RC		0x2
#define RES_GRP_PP_RC		0x3
#define RES_GRP_PR		0x4
#define RES_GRP_PP_PR		0x5
#define RES_GRP_RC_PR		0x6
#define RES_GRP_ALL		0x7

#define RES_TYPE2_R0		0x0

#define RES_TYPE_ALL		0x7

#define RES_STATE_WRST		0xF
#define RES_STATE_ACTIVE	0xE
#define RES_STATE_SLEEP		0x8
#define RES_STATE_OFF		0x0

/*
*	Power Bus Message Format
*
*	Broadcast Message (16 Bits)
*	DEV_GRP[15:13] MT[12]  RES_GRP[11:9]  RES_TYPE2[8:7] RES_TYPE[6:4]
*	RES_STATE[3:0]
*
*	Singular Message (16 Bits)
*	DEV_GRP[15:13] MT[12]  RES_ID[11:4]  RES_STATE[3:0]
*
*/

#define MSG_BROADCAST(devgrp, grp, type, type2, state) \
	(devgrp << 13 | 1 << 12 | grp << 9 | type2 << 7 | type << 4 | state)

#define MSG_SINGULAR(devgrp, id, state) \
	(devgrp << 13 | 0 << 12 | id << 4 | state)

#define R_PROTECT_KEY		0x0E
#define KEY_1			0xC0
#define KEY_2			0x0C

struct triton_ins {
	u16 pmb_message;
	u8 delay;
};


#define CONFIG_DISABLE_HFCLK	1

#if defined(CONFIG_MACH_OMAP_3430SDP) || defined(CONFIG_MACH_OMAP_3430LABRADOR)

struct triton_ins sleep_on_seq[] __initdata = {
	{MSG_SINGULAR(DEV_GRP_P1, 0xf, RES_STATE_OFF), 4},
	{MSG_SINGULAR(DEV_GRP_P1, 0x10, RES_STATE_OFF), 2},
#ifdef CONFIG_DISABLE_HFCLK
	{MSG_SINGULAR(DEV_GRP_P1, 0x7, RES_STATE_OFF), 3},
	{MSG_SINGULAR(DEV_GRP_P1, 0x19, RES_STATE_OFF), 3},
#endif /* #ifdef CONFIG_DISABLE_HFCLK */
};

struct triton_ins sleep_off_seq[] __initdata = {
#ifndef CONFIG_DISABLE_HFCLK
	{MSG_SINGULAR(DEV_GRP_P1, 0xf, RES_STATE_ACTIVE), 4},
	{MSG_SINGULAR(DEV_GRP_P1, 0x10, RES_STATE_ACTIVE), 2},
#else
	{MSG_SINGULAR(DEV_GRP_P1, 0x7, RES_STATE_ACTIVE), 0x30},
	{MSG_SINGULAR(DEV_GRP_P1, 0xf, RES_STATE_ACTIVE), 0x30},
	{MSG_SINGULAR(DEV_GRP_P1, 0x10, RES_STATE_ACTIVE), 0x37},
	{MSG_SINGULAR(DEV_GRP_P1, 0x19, RES_STATE_ACTIVE), 3},
#endif /* #ifndef CONFIG_DISABLE_HFCLK */
};

struct triton_ins t2_wrst_seq[] __initdata = {
	{MSG_SINGULAR(DEV_GRP_NULL, 0x1b, RES_STATE_OFF), 2},
	{MSG_SINGULAR(DEV_GRP_P1, 0xf, RES_STATE_WRST), 15},
	{MSG_SINGULAR(DEV_GRP_P1, 0x10, RES_STATE_WRST), 15},
	{MSG_SINGULAR(DEV_GRP_P1, 0x7, RES_STATE_WRST), 0x60},
	{MSG_SINGULAR(DEV_GRP_P1, 0x19, RES_STATE_ACTIVE), 2},
	{MSG_SINGULAR(DEV_GRP_NULL, 0x1b, RES_STATE_ACTIVE), 2},
};
#else
struct triton_ins sleep_on_seq[] __initdata = {
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_RC, RES_TYPE_ALL, RES_TYPE2_R0,
			RES_STATE_SLEEP), 4},
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_ALL, RES_TYPE2_R0,
			RES_STATE_SLEEP), 4},
};

struct triton_ins sleep_off_seq[] __initdata = {
	{MSG_SINGULAR(DEV_GRP_NULL, 0x17, RES_STATE_ACTIVE), 0x30},
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_PP_PR, RES_TYPE_ALL, RES_TYPE2_R0,
			RES_STATE_ACTIVE), 0x37},
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_ALL, RES_TYPE2_R0,
			RES_STATE_ACTIVE), 0x2},
};

struct triton_ins t2_wrst_seq[] __initdata = { };

#endif

static int __init twl4030_write_script_byte(u8 address, u8 byte)
{
	int err;

	err = twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, address,
					R_MEMORY_ADDRESS);
	err |= twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, byte,
					R_MEMORY_DATA);

	return err;
}

static int __init twl4030_write_script_ins(u8 address, u16 pmb_message,
						u8 delay, u8 next)
{
	int err = 0;

	address *= 4;
	err |= twl4030_write_script_byte(address++, pmb_message >> 8);
	err |= twl4030_write_script_byte(address++, pmb_message & 0xff);
	err |= twl4030_write_script_byte(address++, delay);
	err |= twl4030_write_script_byte(address++, next);

	return err;
}

static int __init twl4030_write_script(u8 address, struct triton_ins *script,
					int len)
{
	int err = 0;

	for (; len; len--, address++, script++) {
		if (len == 1)
			err |= twl4030_write_script_ins(address,
							script->pmb_message,
							script->delay,
							END_OF_SCRIPT);
		else
			err |= twl4030_write_script_ins(address,
							script->pmb_message,
							script->delay,
							address + 1);
	}

	return err;
}

static int __init config_sleep_wake_sequence(void)
{
	int err = 0;

	/*
	 * CLKREQ is pulled high on the 2430SDP, therefore, we need to take
	 * it out of the HFCLKOUT DEV_GRP for P1 else HFCLKOUT can't be stopped.
	 */

	err |= twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				  0x20, R_HFCLKOUT_DEV_GRP);

	/* Set ACTIVE to SLEEP SEQ address in T2 memory*/
	err |= twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, 0x2B,
				  R_SEQ_ADD_A2S);

	/* Set SLEEP to ACTIVE SEQ address for P1 and P2 */
	err |= twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, 0x2F,
				  R_SEQ_ADD_SA12);

	/* Set SLEEP to ACTIVE SEQ address for P3 */
	err |= twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, 0x2F,
				  R_SEQ_ADD_S2A3);

	/* Install Active->Sleep (A2S) sequence */
	err |= twl4030_write_script(0x2B, sleep_on_seq,
					ARRAY_SIZE(sleep_on_seq));

	/* Install Sleep->Active (S2A) sequence */
	err |= twl4030_write_script(0x2F, sleep_off_seq,
					ARRAY_SIZE(sleep_off_seq));

	if (machine_is_omap_3430sdp() || machine_is_omap_ldp()) {
		u8 data;
		/* Disabling AC charger effect on sleep-active transitions */
		err |= twl4030_i2c_read_u8(TWL4030_MODULE_PM_MASTER, &data,
						R_CFG_P1_TRANSITION);
		data &= ~(1<<1);
		err |= twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, data ,
						R_CFG_P1_TRANSITION);
	}

	/* P1/P2/P3 LVL_WAKEUP should be on LEVEL */
	err |= twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, LVL_WAKEUP,
					R_P1_SW_EVENTS);
	err |= twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, LVL_WAKEUP,
					R_P2_SW_EVENTS);
	err |= twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, LVL_WAKEUP,
				R_P3_SW_EVENTS);

	if (err)
		printk(KERN_ERR "TWL4030 sleep-wake sequence config error\n");

	return err;
}


/* Programming the WARMRESET Sequence on TRITON */
static int __init config_warmreset_sequence(void)
{

	int e = 0;
	u8 rd_data;

	if (!ARRAY_SIZE(t2_wrst_seq))
		return 0;

	/* Set WARM RESET SEQ address for P1 */
	e |= twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, 0x38,
				R_SEQ_ADD_WARM);

	/* Install Warm Reset sequence */
	e |= twl4030_write_script(0x38, t2_wrst_seq,
					ARRAY_SIZE(t2_wrst_seq));

	/* P1/P2/P3 enable WARMRESET */
	e |= twl4030_i2c_read_u8(TWL4030_MODULE_PM_MASTER, &rd_data,
				R_P1_SW_EVENTS);
	rd_data |= ENABLE_WARMRESET;
	e |= twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, rd_data,
				R_P1_SW_EVENTS);

	e |= twl4030_i2c_read_u8(TWL4030_MODULE_PM_MASTER, &rd_data,
				R_P2_SW_EVENTS);
	rd_data |= ENABLE_WARMRESET;
	e |= twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, rd_data,
				R_P2_SW_EVENTS);

	e |= twl4030_i2c_read_u8(TWL4030_MODULE_PM_MASTER, &rd_data,
				R_P3_SW_EVENTS);
	rd_data |= ENABLE_WARMRESET;
	e |= twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, rd_data,
				R_P3_SW_EVENTS);

	if (e)
		printk(KERN_ERR
			"TWL4030 Power Companion Warmreset seq config error\n");
	return e;
}

static int __init twl4030_power_init(void)
{
	int err = 0;

	err = twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, KEY_1,
				R_PROTECT_KEY);
	err |= twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, KEY_2,
				R_PROTECT_KEY);

	if (err)
		return err;

	err = config_sleep_wake_sequence();
	if (err)
		return err;

	err = config_warmreset_sequence();
	if (err)
		return err;

	err = twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, 0, R_PROTECT_KEY);

	return err;

}

module_init(twl4030_power_init);
