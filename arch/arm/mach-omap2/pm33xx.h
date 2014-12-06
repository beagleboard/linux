/*
 * AM33XX Power Management Routines
 *
 * Copyright (C) 2012-2014 Texas Instruments Inc.
 *	Vaibhav Bedia, Dave Gerlach
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
#ifndef __ARCH_ARM_MACH_OMAP2_PM33XX_H
#define __ARCH_ARM_MACH_OMAP2_PM33XX_H

#ifndef __ASSEMBLER__

#include <linux/wkup_m3.h>

struct am33xx_pm_ops {
	int (*init)(void);
	void (*pre_suspend)(unsigned int state);
	void (*post_suspend)(unsigned int state);
	void (*save_context)(void);
	void (*restore_context)(void);
};

struct am33xx_pm_context {
	struct wkup_m3_ipc_regs	ipc;
	struct firmware		*firmware;
	struct omap_mbox	*mbox;
	struct am33xx_pm_ops	*ops;
	u8			state;
	u32			ver;
	u32			m3_i2c_sequence_offsets;
	const char		*sd_fw_name;
};

/*
 * Params passed to suspend routine
 *
 * These are used to load into registers by suspend code,
 * entries here must always be in sync with the suspend code
 * in arm/mach-omap2/sleep33xx.S
 */
struct am33xx_suspend_params {
	void __iomem *emif_addr_virt;
	u32 wfi_flags;
	void __iomem *dram_sync;
	void __iomem *rtc_base;
	void __iomem *l2_base_virt;
	void __iomem *cke_override_virt;
};

void wkup_m3_reset_data_pos(void);
int wkup_m3_copy_data(const u8 *data, size_t size);
int am33xx_do_sram_cpuidle(u32, u32);
void omap_rtc_power_off_program(void);
void __iomem *omap_rtc_get_base_addr(void);

#endif /*__ASSEMBLER__ */

#define IPC_CMD_RTC_ONLY		0x1
#define	IPC_CMD_DS0			0x4
#define	IPC_CMD_STANDBY			0xc
#define	IPC_CMD_IDLE			0xd
#define IPC_CMD_RESET			0xe
#define DS_IPC_DEFAULT			0xffffffff
#define M3_VERSION_UNKNOWN		0x0000ffff
#define M3_BASELINE_VERSION		0x189

#define M3_STATE_UNKNOWN		0
#define M3_STATE_RESET			1
#define M3_STATE_INITED			2
#define M3_STATE_MSG_FOR_LP		3
#define M3_STATE_MSG_FOR_RESET		4

#define AM33XX_OCMC_END			0x40310000
#define AM33XX_EMIF_BASE		0x4C000000

#define AM43XX_CM_BASE			0x44DF0000

#define AM43XX_CTRL_CKE_OVERRIDE	0x44E1131C

#define AM43XX_CM_REGADDR(inst, reg)				\
	AM33XX_L4_WK_IO_ADDRESS(AM43XX_CM_BASE + (inst) + (reg))

#define AM43XX_PM_MPU_PWRSTCTRL AM43XX_CM_REGADDR(0x0300, 0x00)
#define AM43XX_CM_MPU_CLKSTCTRL AM43XX_CM_REGADDR(0x8300, 0x00)
#define AM43XX_CM_MPU_MPU_CLKCTRL AM43XX_CM_REGADDR(0x8300, 0x20)
#define AM43XX_CM_PER_EMIF_CLKCTRL  AM43XX_CM_REGADDR(0x8800, 0x0720)

#define AM43XX_CM_PER_EMIF_CLKCTRL_OFFSET 0x0720
#define AM43XX_PRM_EMIF_CTRL_OFFSET	0x30

#define MEM_TYPE_DDR2		2
#define MEM_TYPE_DDR3		3

#define WFI_MEM_TYPE_DDR2	(1 << 0)
#define WFI_MEM_TYPE_DDR3	(1 << 1)
#define WFI_SELF_REFRESH	(1 << 2)
#define WFI_SAVE_EMIF		(1 << 3)
#define WFI_WAKE_M3		(1 << 4)
#define WFI_DISABLE_EMIF	(1 << 7)
#define WFI_RTC_ONLY		(1 << 8)

/*
 * 9-4 = VTT GPIO PIN (6 Bits)
 *   3 = VTT Status (1 Bit)
 * 2-0 = Memory Type (3 Bits)
*/
#define MEM_TYPE_SHIFT		(0x0)
#define MEM_TYPE_MASK		(0x7 << 0)
#define VTT_STAT_SHIFT		(0x3)
#define VTT_STAT_MASK		(0x1 << 3)
#define VTT_GPIO_PIN_SHIFT	(0x4)
#define VTT_GPIO_PIN_MASK	(0x3f << 4)
#define IO_ISOLATION_STAT_SHIFT (10)
#define IO_ISOLATION_STAT_MASK  (0x1 << 10)

#define MPU_WAKE		0x800

#define MEM_BANK_RET_ST_OFF		0x0
#define MEM_BANK_RET_ST_RET		0x1

#define M3_PARAM2_MPU_STATE_SHIFT	0
#define M3_PARAM2_MPU_RAM_RET_SHIFT	2
#define M3_PARAM2_MPU_L1_RET_SHIFT	3
#define M3_PARAM2_MPU_L2_RET_SHIFT	4
#define M3_PARAM2_PER_STATE_SHIFT	7
#define M3_PARAM2_WAKE_SOURCES_SHIFT	18

#endif /* __ARCH_ARM_MACH_OMAP2_PM33XX_H */
