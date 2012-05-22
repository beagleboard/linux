/*
 * AM33XX Power Management Routines
 *
 * Copyright (C) 2012 Texas Instruments Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ARCH_ARM_MACH_OMAP2_PM33XX_H
#define __ARCH_ARM_MACH_OMAP2_PM33XX_H

#include <mach/hardware.h>	/* XXX Is this the right one to include? */

#ifndef __ASSEMBLER__
extern void __iomem *am33xx_get_ram_base(void);

struct a8_wkup_m3_ipc_data {
	int resume_addr;
	int sleep_mode;
	int ipc_data1;
	int ipc_data2;
} am33xx_lp_ipc;

struct am33xx_padconf {
	int	mii1_col;
	int	mii1_crs;
	int	mii1_rxerr;
	int	mii1_txen;
	int	mii1_rxdv;
	int	mii1_txd3;
	int	mii1_txd2;
	int	mii1_txd1;
	int	mii1_txd0;
	int	mii1_txclk;
	int	mii1_rxclk;
	int	mii1_rxd3;
	int	mii1_rxd2;
	int	mii1_rxd1;
	int	mii1_rxd0;
	int	rmii1_refclk;
	int	mdio_data;
	int	mdio_clk;
};
#endif /* ASSEMBLER */

#define M3_TXEV_EOI			(AM33XX_CTRL_BASE + 0x1324)
#define A8_M3_IPC_REGS			(AM33XX_CTRL_BASE + 0x1328)
#define DS_RESUME_BASE			0x40300000
#define DS_IPC_DEFAULT			0xffffffff
#define M3_UMEM				0x44D00000

#define	DS0_ID				0x3
#define DS1_ID				0x5

#define M3_STATE_UNKNOWN		-1
#define M3_STATE_RESET			0
#define M3_STATE_INITED			1
#define M3_STATE_MSG_FOR_LP		2
#define M3_STATE_MSG_FOR_RESET		3

/* DDR offsets */
#define DDR_CMD0_IOCTRL			(AM33XX_CTRL_BASE + 0x1404)
#define DDR_CMD1_IOCTRL			(AM33XX_CTRL_BASE + 0x1408)
#define DDR_CMD2_IOCTRL			(AM33XX_CTRL_BASE + 0x140C)
#define DDR_DATA0_IOCTRL		(AM33XX_CTRL_BASE + 0x1440)
#define DDR_DATA1_IOCTRL		(AM33XX_CTRL_BASE + 0x1444)

#define DDR_IO_CTRL			(AM33XX_CTRL_BASE + 0x0E04)
#define VTP0_CTRL_REG			(AM33XX_CTRL_BASE + 0x0E0C)
#define DDR_CKE_CTRL			(AM33XX_CTRL_BASE + 0x131C)
#define DDR_PHY_BASE_ADDR		(AM33XX_CTRL_BASE + 0x2000)

#define CMD0_CTRL_SLAVE_RATIO_0		(DDR_PHY_BASE_ADDR + 0x01C)
#define CMD0_CTRL_SLAVE_FORCE_0		(DDR_PHY_BASE_ADDR + 0x020)
#define CMD0_CTRL_SLAVE_DELAY_0		(DDR_PHY_BASE_ADDR + 0x024)
#define CMD0_DLL_LOCK_DIFF_0		(DDR_PHY_BASE_ADDR + 0x028)
#define CMD0_INVERT_CLKOUT_0		(DDR_PHY_BASE_ADDR + 0x02C)

#define CMD1_CTRL_SLAVE_RATIO_0		(DDR_PHY_BASE_ADDR + 0x050)
#define CMD1_CTRL_SLAVE_FORCE_0		(DDR_PHY_BASE_ADDR + 0x054)
#define CMD1_CTRL_SLAVE_DELAY_0		(DDR_PHY_BASE_ADDR + 0x058)
#define CMD1_DLL_LOCK_DIFF_0		(DDR_PHY_BASE_ADDR + 0x05C)
#define CMD1_INVERT_CLKOUT_0		(DDR_PHY_BASE_ADDR + 0x060)

#define CMD2_CTRL_SLAVE_RATIO_0		(DDR_PHY_BASE_ADDR + 0x084)
#define CMD2_CTRL_SLAVE_FORCE_0		(DDR_PHY_BASE_ADDR + 0x088)
#define CMD2_CTRL_SLAVE_DELAY_0		(DDR_PHY_BASE_ADDR + 0x08C)
#define CMD2_DLL_LOCK_DIFF_0		(DDR_PHY_BASE_ADDR + 0x090)
#define CMD2_INVERT_CLKOUT_0		(DDR_PHY_BASE_ADDR + 0x094)

#define DATA0_RD_DQS_SLAVE_RATIO_0	(DDR_PHY_BASE_ADDR + 0x0C8)
#define DATA0_RD_DQS_SLAVE_RATIO_1	(DDR_PHY_BASE_ADDR + 0x0CC)

#define DATA0_WR_DQS_SLAVE_RATIO_0	(DDR_PHY_BASE_ADDR + 0x0DC)
#define DATA0_WR_DQS_SLAVE_RATIO_1	(DDR_PHY_BASE_ADDR + 0x0E0)

#define DATA0_WRLVL_INIT_RATIO_0	(DDR_PHY_BASE_ADDR + 0x0F0)
#define DATA0_WRLVL_INIT_RATIO_1	(DDR_PHY_BASE_ADDR + 0x0F4)

#define DATA0_GATELVL_INIT_RATIO_0	(DDR_PHY_BASE_ADDR + 0x0FC)
#define DATA0_GATELVL_INIT_RATIO_1	(DDR_PHY_BASE_ADDR + 0x100)

#define DATA0_FIFO_WE_SLAVE_RATIO_0	(DDR_PHY_BASE_ADDR + 0x108)
#define DATA0_FIFO_WE_SLAVE_RATIO_1	(DDR_PHY_BASE_ADDR + 0x10C)

#define DATA0_WR_DATA_SLAVE_RATIO_0	(DDR_PHY_BASE_ADDR + 0x120)
#define DATA0_WR_DATA_SLAVE_RATIO_1	(DDR_PHY_BASE_ADDR + 0x124)

#define DATA0_DLL_LOCK_DIFF_0		(DDR_PHY_BASE_ADDR + 0x138)

#define DATA0_RANK0_DELAYS_0		(DDR_PHY_BASE_ADDR + 0x134)
#define DATA1_RANK0_DELAYS_0		(DDR_PHY_BASE_ADDR + 0x1D8)

/* Temp placeholder for the values we want in the registers */
#define EMIF_READ_LATENCY	0x100005	/* Enable Dynamic Power Down */
#define EMIF_TIM1		0x0666B3C9
#define EMIF_TIM2		0x243631CA
#define EMIF_TIM3		0x0000033F
#define EMIF_SDCFG		0x41805332
#define EMIF_SDREF		0x0000081a
#define EMIF_SDMGT		0x80000000
#define EMIF_SDRAM		0x00004650
#define EMIF_PHYCFG		0x2

#define DDR2_DLL_LOCK_DIFF	0x0
#define DDR2_RD_DQS		0x12
#define DDR2_PHY_FIFO_WE	0x80

#define DDR_PHY_RESET		(0x1 << 10)
#define DDR_PHY_READY		(0x1 << 2)
#define DDR2_RATIO		0x80
#define CMD_FORCE		0x00
#define CMD_DELAY		0x00

#define DDR2_INVERT_CLKOUT	0x00
#define DDR2_WR_DQS		0x00
#define DDR2_PHY_WRLVL		0x00
#define DDR2_PHY_GATELVL	0x00
#define DDR2_PHY_WR_DATA	0x40
#define PHY_RANK0_DELAY		0x01
#define PHY_DLL_LOCK_DIFF	0x0
#define DDR_IOCTRL_VALUE	0x18B

#define VTP_CTRL_READY		(0x1 << 5)
#define VTP_CTRL_ENABLE		(0x1 << 6)
#define VTP_CTRL_LOCK_EN	(0x1 << 4)
#define VTP_CTRL_START_EN	(0x1)

#endif
