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

#define VTP_CTRL_READY		(0x1 << 5)
#define VTP_CTRL_ENABLE		(0x1 << 6)
#define VTP_CTRL_LOCK_EN	(0x1 << 4)
#define VTP_CTRL_START_EN	(0x1)

#define DDR_IO_CTRL			(AM33XX_CTRL_BASE + 0x0E04)
#define VTP0_CTRL_REG			(AM33XX_CTRL_BASE + 0x0E0C)

#endif
