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
#include <linux/platform_device.h>

#include <asm/mach-types.h>

static u8 triton_next_free_address = 0x2b;

#define PWR_P1_SW_EVENTS	0x10
#define PWR_DEVOFF	(1<<0)

#define PHY_TO_OFF_PM_MASTER(p)		(p - 0x36)
#define PHY_TO_OFF_PM_RECEIVER(p)	(p - 0x5b)

/* resource - hfclk */
#define R_HFCLKOUT_DEV_GRP 	PHY_TO_OFF_PM_RECEIVER(0xe6)

/* PM events */
#define R_P1_SW_EVENTS		PHY_TO_OFF_PM_MASTER(0x46)
#define R_P2_SW_EVENTS		PHY_TO_OFF_PM_MASTER(0x47)
#define R_P3_SW_EVENTS		PHY_TO_OFF_PM_MASTER(0x48)
#define R_CFG_P1_TRANSITION	PHY_TO_OFF_PM_MASTER(0x36)
#define R_CFG_P2_TRANSITION	PHY_TO_OFF_PM_MASTER(0x37)
#define R_CFG_P3_TRANSITION	PHY_TO_OFF_PM_MASTER(0x38)

#define LVL_WAKEUP	0x08

#define ENABLE_WARMRESET (1<<4)

#define END_OF_SCRIPT		0x3f

#define R_SEQ_ADD_A2S		PHY_TO_OFF_PM_MASTER(0x55)
#define R_SEQ_ADD_SA12		PHY_TO_OFF_PM_MASTER(0x56)
#define	R_SEQ_ADD_S2A3		PHY_TO_OFF_PM_MASTER(0x57)
#define	R_SEQ_ADD_WARM		PHY_TO_OFF_PM_MASTER(0x58)
#define R_MEMORY_ADDRESS	PHY_TO_OFF_PM_MASTER(0x59)
#define R_MEMORY_DATA		PHY_TO_OFF_PM_MASTER(0x5a)

#define R_PROTECT_KEY		0x0E
#define KEY_1			0xC0
#define KEY_2			0x0C

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

static int __init twl4030_write_script(u8 address, struct twl4030_ins *script,
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

static int __init config_wakeup3_sequence(u8 address)
{

	int err = 0;

	/* Set SLEEP to ACTIVE SEQ address for P3 */
	err |= twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, address,
				  R_SEQ_ADD_S2A3);

	err |= twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, LVL_WAKEUP,
					R_P3_SW_EVENTS);
	if (err)
		printk(KERN_ERR "TWL4030 wakeup sequence for P3" \
				"config error\n");

	return err;
}

static int __init config_wakeup12_sequence(u8 address)
{
	int err = 0;

	/* Set SLEEP to ACTIVE SEQ address for P1 and P2 */
	err |= twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, address,
				  R_SEQ_ADD_SA12);

	/* P1/P2/P3 LVL_WAKEUP should be on LEVEL */
	err |= twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, LVL_WAKEUP,
					R_P1_SW_EVENTS);
	err |= twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, LVL_WAKEUP,
					R_P2_SW_EVENTS);

	if (machine_is_omap_3430sdp() || machine_is_omap_ldp()) {
		u8 data;
		/* Disabling AC charger effect on sleep-active transitions */
		err |= twl4030_i2c_read_u8(TWL4030_MODULE_PM_MASTER, &data,
						R_CFG_P1_TRANSITION);
		data &= ~(1<<1);
		err |= twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, data ,
						R_CFG_P1_TRANSITION);
	}

	if (err)
		printk(KERN_ERR "TWL4030 wakeup sequence for P1 and P2" \
				"config error\n");

	return err;
}

static int __init config_sleep_sequence(u8 address)
{
	int err = 0;

	/*
	 * CLKREQ is pulled high on the 2430SDP, therefore, we need to take
	 * it out of the HFCLKOUT DEV_GRP for P1 else HFCLKOUT can't be stopped.
	 */

	err |= twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				  0x20, R_HFCLKOUT_DEV_GRP);

	/* Set ACTIVE to SLEEP SEQ address in T2 memory*/
	err |= twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, address,
				  R_SEQ_ADD_A2S);

	if (err)
		printk(KERN_ERR "TWL4030 sleep sequence config error\n");

	return err;
}

static int __init config_warmreset_sequence(u8 address)
{

	int err = 0;
	u8 rd_data;

	/* Set WARM RESET SEQ address for P1 */
	err |= twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, address,
					R_SEQ_ADD_WARM);

	/* P1/P2/P3 enable WARMRESET */
	err |= twl4030_i2c_read_u8(TWL4030_MODULE_PM_MASTER, &rd_data,
					R_P1_SW_EVENTS);
	rd_data |= ENABLE_WARMRESET;
	err |= twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, rd_data,
					R_P1_SW_EVENTS);

	err |= twl4030_i2c_read_u8(TWL4030_MODULE_PM_MASTER, &rd_data,
					R_P2_SW_EVENTS);
	rd_data |= ENABLE_WARMRESET;
	err |= twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, rd_data,
					R_P2_SW_EVENTS);

	err |= twl4030_i2c_read_u8(TWL4030_MODULE_PM_MASTER, &rd_data,
					R_P3_SW_EVENTS);
	rd_data |= ENABLE_WARMRESET;
	err |= twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, rd_data,
					R_P3_SW_EVENTS);

	if (err)
		printk(KERN_ERR
			"TWL4030 warmreset seq config error\n");
	return err;
}

static int __init load_triton_script(struct twl4030_script *tscript)
{
	u8 address = triton_next_free_address;
	int err;

	err = twl4030_write_script(address, tscript->script, tscript->size);
	if (err)
		return err;

	triton_next_free_address += tscript->size;

	if (tscript->flags & TRITON_WRST_SCRIPT)
		err |= config_warmreset_sequence(address);

	if (tscript->flags & TRITON_WAKEUP12_SCRIPT)
		err |= config_wakeup12_sequence(address);

	if (tscript->flags & TRITON_WAKEUP3_SCRIPT)
		err |= config_wakeup3_sequence(address);

	if (tscript->flags & TRITON_SLEEP_SCRIPT)
		err |= config_sleep_sequence(address);

	return err;
}

void __init twl4030_power_init(struct twl4030_power_data *triton2_scripts)
{
	int err = 0;
	int i;

	err = twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, KEY_1,
				R_PROTECT_KEY);
	err |= twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, KEY_2,
				R_PROTECT_KEY);
	if (err)
		printk(KERN_ERR
			"TWL4030 Unable to unlock registers\n");

	for (i = 0; i < triton2_scripts->size; i++) {
		err = load_triton_script(triton2_scripts->scripts[i]);
		if (err)
			break;
	}

	if (twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, 0, R_PROTECT_KEY))
		printk(KERN_ERR
			"TWL4030 Unable to relock registers\n");
}
