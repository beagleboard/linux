/*
 * linux/arch/arm/mach-omap3/smartreflex.c
 *
 * OMAP34XX SmartReflex Voltage Control
 *
 * Copyright (C) 2008 Nokia Corporation
 * Kalle Jokiniemi
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Lesly A M <x0080970@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/i2c/twl4030.h>
#include <linux/io.h>

#include <asm/arch/omap34xx.h>
#include <asm/arch/control.h>
#include <asm/arch/clock.h>

#include "prm.h"
#include "smartreflex.h"
#include "prm-regbits-34xx.h"

/* XXX: These should be relocated where-ever the OPP implementation will be */
u32 current_vdd1_opp;
u32 current_vdd2_opp;

struct omap_sr {
	int		srid;
	int		is_sr_reset;
	int		is_autocomp_active;
	struct clk 	*clk;
	u32 		clk_length;
	u32 		req_opp_no;
	u32 		opp1_nvalue, opp2_nvalue, opp3_nvalue, opp4_nvalue;
	u32		opp5_nvalue;
	u32		senp_mod, senn_mod;
	u32		srbase_addr;
	u32		vpbase_addr;
};

/* Custom clocks to enable SR specific enable/disable functions. */
struct sr_custom_clk {
	struct clk 	clk;  /* meta-clock with custom enable/disable calls */
	struct clk 	*fck; /* actual functional clock */
	struct omap_sr 	*sr;
};

static inline void sr_write_reg(struct omap_sr *sr, int offset, u32 value)
{
	__raw_writel(value, sr->srbase_addr + offset);
}

static inline void sr_modify_reg(struct omap_sr *sr, int offset, u32 mask,
								u32 value)
{
	u32 reg_val;

	reg_val = __raw_readl(sr->srbase_addr + offset);
	reg_val &= ~mask;
	reg_val |= value;

	__raw_writel(reg_val, sr->srbase_addr + offset);
}

static inline u32 sr_read_reg(struct omap_sr *sr, int offset)
{
	return __raw_readl(sr->srbase_addr + offset);
}

/* Custom clock handling functions */
static int sr_clk_enable(struct clk *clk)
{
	struct sr_custom_clk *sr_clk = container_of(clk, struct sr_custom_clk,
									clk);

	if (clk_enable(sr_clk->fck) != 0) {
		printk(KERN_ERR "Could not enable %s\n", sr_clk->fck->name);
		goto clk_enable_err;
	}

	/* set fclk- active , iclk- idle */
	sr_modify_reg(sr_clk->sr, ERRCONFIG, SR_CLKACTIVITY_MASK,
						SR_CLKACTIVITY_IOFF_FON);

	return 0;

clk_enable_err:
	return -1;
}

static void sr_clk_disable(struct clk *clk)
{
	struct sr_custom_clk *sr_clk = container_of(clk, struct sr_custom_clk,
									clk);

	/* set fclk, iclk- idle */
	sr_modify_reg(sr_clk->sr, ERRCONFIG, SR_CLKACTIVITY_MASK,
						SR_CLKACTIVITY_IOFF_FOFF);

	clk_disable(sr_clk->fck);
	sr_clk->sr->is_sr_reset = 1;
}

static struct omap_sr sr1 = {
	.srid			= SR1,
	.is_sr_reset		= 1,
	.is_autocomp_active	= 0,
	.clk_length		= 0,
	.srbase_addr		= OMAP2_IO_ADDRESS(OMAP34XX_SR1_BASE),
};

static struct omap_sr sr2 = {
	.srid			= SR2,
	.is_sr_reset		= 1,
	.is_autocomp_active	= 0,
	.clk_length		= 0,
	.srbase_addr		= OMAP2_IO_ADDRESS(OMAP34XX_SR2_BASE),
};

static struct sr_custom_clk sr1_custom_clk = {
	.clk = {
			.name 		= "sr1_custom_clk",
			.enable		= sr_clk_enable,
			.disable	= sr_clk_disable,
	},
	.sr	= &sr1,
};

static struct sr_custom_clk sr2_custom_clk = {
	.clk = {
			.name 		= "sr2_custom_clk",
			.enable		= sr_clk_enable,
			.disable	= sr_clk_disable,
	},
	.sr	= &sr2,
};

static void cal_reciprocal(u32 sensor, u32 *sengain, u32 *rnsen)
{
	u32 gn, rn, mul;

	for (gn = 0; gn < GAIN_MAXLIMIT; gn++) {
		mul = 1 << (gn + 8);
		rn = mul / sensor;
		if (rn < R_MAXLIMIT) {
			*sengain = gn;
			*rnsen = rn;
		}
	}
}

static u32 cal_test_nvalue(u32 sennval, u32 senpval)
{
	u32 senpgain, senngain;
	u32 rnsenp, rnsenn;

	/* Calculating the gain and reciprocal of the SenN and SenP values */
	cal_reciprocal(senpval, &senpgain, &rnsenp);
	cal_reciprocal(sennval, &senngain, &rnsenn);

	return ((senpgain << NVALUERECIPROCAL_SENPGAIN_SHIFT) |
		(senngain << NVALUERECIPROCAL_SENNGAIN_SHIFT) |
		(rnsenp << NVALUERECIPROCAL_RNSENP_SHIFT) |
		(rnsenn << NVALUERECIPROCAL_RNSENN_SHIFT));
}

static void sr_clk_init(struct sr_custom_clk *sr_clk)
{
	if (sr_clk->sr->srid == SR1) {
		sr_clk->fck = clk_get(NULL, "sr1_fck");
		if (IS_ERR(sr_clk->fck))
			printk(KERN_ERR "Could not get sr1_fck\n");
	} else if (sr_clk->sr->srid == SR2) {
		sr_clk->fck = clk_get(NULL, "sr2_fck");
		if (IS_ERR(sr_clk->fck))
			printk(KERN_ERR "Could not get sr2_fck\n");
	}
	clk_register(&sr_clk->clk);
}

static void sr_set_clk_length(struct omap_sr *sr)
{
	struct clk *osc_sys_ck;
	u32 sys_clk = 0;

	osc_sys_ck = clk_get(NULL, "osc_sys_ck");
	sys_clk = clk_get_rate(osc_sys_ck);
	clk_put(osc_sys_ck);

	switch (sys_clk) {
	case 12000000:
		sr->clk_length = SRCLKLENGTH_12MHZ_SYSCLK;
		break;
	case 13000000:
		sr->clk_length = SRCLKLENGTH_13MHZ_SYSCLK;
		break;
	case 19200000:
		sr->clk_length = SRCLKLENGTH_19MHZ_SYSCLK;
		break;
	case 26000000:
		sr->clk_length = SRCLKLENGTH_26MHZ_SYSCLK;
		break;
	case 38400000:
		sr->clk_length = SRCLKLENGTH_38MHZ_SYSCLK;
		break;
	default :
		printk(KERN_ERR "Invalid sysclk value: %d\n", sys_clk);
		break;
	}
}

static void sr_set_efuse_nvalues(struct omap_sr *sr)
{
	if (sr->srid == SR1) {
		sr->senn_mod = (omap_ctrl_readl(OMAP343X_CONTROL_FUSE_SR) &
					OMAP343X_SR1_SENNENABLE_MASK) >>
					OMAP343X_SR1_SENNENABLE_SHIFT;

		sr->senp_mod = (omap_ctrl_readl(OMAP343X_CONTROL_FUSE_SR) &
					OMAP343X_SR1_SENPENABLE_MASK) >>
					OMAP343X_SR1_SENPENABLE_SHIFT;

		sr->opp5_nvalue = omap_ctrl_readl(
					OMAP343X_CONTROL_FUSE_OPP5_VDD1);
		sr->opp4_nvalue = omap_ctrl_readl(
					OMAP343X_CONTROL_FUSE_OPP4_VDD1);
		sr->opp3_nvalue = omap_ctrl_readl(
					OMAP343X_CONTROL_FUSE_OPP3_VDD1);
		sr->opp2_nvalue = omap_ctrl_readl(
					OMAP343X_CONTROL_FUSE_OPP2_VDD1);
		sr->opp1_nvalue = omap_ctrl_readl(
					OMAP343X_CONTROL_FUSE_OPP1_VDD1);
	} else if (sr->srid == SR2) {
		sr->senn_mod = (omap_ctrl_readl(OMAP343X_CONTROL_FUSE_SR) &
					OMAP343X_SR2_SENNENABLE_MASK) >>
					OMAP343X_SR2_SENNENABLE_SHIFT;

		sr->senp_mod = (omap_ctrl_readl(OMAP343X_CONTROL_FUSE_SR) &
					OMAP343X_SR2_SENPENABLE_MASK) >>
					OMAP343X_SR2_SENPENABLE_SHIFT;

		sr->opp3_nvalue = omap_ctrl_readl(
					OMAP343X_CONTROL_FUSE_OPP3_VDD2);
		sr->opp2_nvalue = omap_ctrl_readl(
					OMAP343X_CONTROL_FUSE_OPP2_VDD2);
		sr->opp1_nvalue = omap_ctrl_readl(
					OMAP343X_CONTROL_FUSE_OPP1_VDD2);
	}
}

/* Hard coded nvalues for testing purposes, may cause device to hang! */
static void sr_set_testing_nvalues(struct omap_sr *sr)
{
	if (sr->srid == SR1) {
		sr->senp_mod = 0x03;	/* SenN-M5 enabled */
		sr->senn_mod = 0x03;

		/* calculate nvalues for each opp */
		sr->opp5_nvalue = cal_test_nvalue(0xacd + 0x330, 0x848 + 0x330);
		sr->opp4_nvalue = cal_test_nvalue(0x964 + 0x2a0, 0x727 + 0x2a0);
		sr->opp3_nvalue = cal_test_nvalue(0x85b + 0x200, 0x655 + 0x200);
		sr->opp2_nvalue = cal_test_nvalue(0x506 + 0x1a0, 0x3be + 0x1a0);
		sr->opp1_nvalue = cal_test_nvalue(0x373 + 0x100, 0x28c + 0x100);
	} else if (sr->srid == SR2) {
		sr->senp_mod = 0x03;
		sr->senn_mod = 0x03;

		sr->opp3_nvalue = cal_test_nvalue(0x76f + 0x200, 0x579 + 0x200);
		sr->opp2_nvalue = cal_test_nvalue(0x4f5 + 0x1c0, 0x390 + 0x1c0);
		sr->opp1_nvalue = cal_test_nvalue(0x359, 0x25d);
	}

}

static void sr_set_nvalues(struct omap_sr *sr)
{
	if (SR_TESTING_NVALUES)
		sr_set_testing_nvalues(sr);
	else
		sr_set_efuse_nvalues(sr);
}

static void sr_configure_vp(int srid)
{
	u32 vpconfig;

	if (srid == SR1) {
		vpconfig = PRM_VP1_CONFIG_ERROROFFSET | PRM_VP1_CONFIG_ERRORGAIN
					| PRM_VP1_CONFIG_INITVOLTAGE
					| PRM_VP1_CONFIG_TIMEOUTEN;

		prm_write_mod_reg(vpconfig, OMAP3430_GR_MOD,
					OMAP3_PRM_VP1_CONFIG_OFFSET);
		prm_write_mod_reg(PRM_VP1_VSTEPMIN_SMPSWAITTIMEMIN |
					PRM_VP1_VSTEPMIN_VSTEPMIN,
					OMAP3430_GR_MOD,
					OMAP3_PRM_VP1_VSTEPMIN_OFFSET);

		prm_write_mod_reg(PRM_VP1_VSTEPMAX_SMPSWAITTIMEMAX |
					PRM_VP1_VSTEPMAX_VSTEPMAX,
					OMAP3430_GR_MOD,
					OMAP3_PRM_VP1_VSTEPMAX_OFFSET);

		prm_write_mod_reg(PRM_VP1_VLIMITTO_VDDMAX |
					PRM_VP1_VLIMITTO_VDDMIN |
					PRM_VP1_VLIMITTO_TIMEOUT,
					OMAP3430_GR_MOD,
					OMAP3_PRM_VP1_VLIMITTO_OFFSET);

		/* Trigger initVDD value copy to voltage processor */
		prm_set_mod_reg_bits(PRM_VP1_CONFIG_INITVDD, OMAP3430_GR_MOD,
					OMAP3_PRM_VP1_CONFIG_OFFSET);
		/* Clear initVDD copy trigger bit */
		prm_clear_mod_reg_bits(PRM_VP1_CONFIG_INITVDD, OMAP3430_GR_MOD,
					OMAP3_PRM_VP1_CONFIG_OFFSET);

	} else if (srid == SR2) {
		vpconfig = PRM_VP2_CONFIG_ERROROFFSET | PRM_VP2_CONFIG_ERRORGAIN
					| PRM_VP2_CONFIG_INITVOLTAGE
					| PRM_VP2_CONFIG_TIMEOUTEN;

		prm_write_mod_reg(vpconfig, OMAP3430_GR_MOD,
					OMAP3_PRM_VP2_CONFIG_OFFSET);
		prm_write_mod_reg(PRM_VP2_VSTEPMIN_SMPSWAITTIMEMIN |
					PRM_VP2_VSTEPMIN_VSTEPMIN,
					OMAP3430_GR_MOD,
					OMAP3_PRM_VP2_VSTEPMIN_OFFSET);

		prm_write_mod_reg(PRM_VP2_VSTEPMAX_SMPSWAITTIMEMAX |
					PRM_VP2_VSTEPMAX_VSTEPMAX,
					OMAP3430_GR_MOD,
					OMAP3_PRM_VP2_VSTEPMAX_OFFSET);

		prm_write_mod_reg(PRM_VP2_VLIMITTO_VDDMAX |
					PRM_VP2_VLIMITTO_VDDMIN |
					PRM_VP2_VLIMITTO_TIMEOUT,
					OMAP3430_GR_MOD,
					OMAP3_PRM_VP2_VLIMITTO_OFFSET);

		/* Trigger initVDD value copy to voltage processor */
		prm_set_mod_reg_bits(PRM_VP2_CONFIG_INITVDD, OMAP3430_GR_MOD,
					OMAP3_PRM_VP2_CONFIG_OFFSET);
		/* Reset initVDD copy trigger bit */
		prm_clear_mod_reg_bits(PRM_VP2_CONFIG_INITVDD, OMAP3430_GR_MOD,
					OMAP3_PRM_VP2_CONFIG_OFFSET);

	}
}

static void sr_configure_vc(void)
{
	prm_write_mod_reg((R_SRI2C_SLAVE_ADDR << OMAP3430_SMPS_SA1_SHIFT) |
			(R_SRI2C_SLAVE_ADDR << OMAP3430_SMPS_SA0_SHIFT),
			OMAP3430_GR_MOD, OMAP3_PRM_VC_SMPS_SA_OFFSET);

	prm_write_mod_reg((R_VDD2_SR_CONTROL << OMAP3430_VOLRA1_SHIFT) |
			(R_VDD1_SR_CONTROL << OMAP3430_VOLRA0_SHIFT),
			OMAP3430_GR_MOD, OMAP3_PRM_VC_SMPS_VOL_RA_OFFSET);

	prm_write_mod_reg((OMAP3430_VC_CMD_VAL0_ON <<
		OMAP3430_VC_CMD_ON_SHIFT) |
		(OMAP3430_VC_CMD_VAL0_ONLP << OMAP3430_VC_CMD_ONLP_SHIFT) |
		(OMAP3430_VC_CMD_VAL0_RET << OMAP3430_VC_CMD_RET_SHIFT) |
		(OMAP3430_VC_CMD_VAL0_OFF << OMAP3430_VC_CMD_OFF_SHIFT),
		OMAP3430_GR_MOD, OMAP3_PRM_VC_CMD_VAL_0_OFFSET);

	prm_write_mod_reg((OMAP3430_VC_CMD_VAL1_ON <<
		OMAP3430_VC_CMD_ON_SHIFT) |
		(OMAP3430_VC_CMD_VAL1_ONLP << OMAP3430_VC_CMD_ONLP_SHIFT) |
		(OMAP3430_VC_CMD_VAL1_RET << OMAP3430_VC_CMD_RET_SHIFT) |
		(OMAP3430_VC_CMD_VAL1_OFF << OMAP3430_VC_CMD_OFF_SHIFT),
		OMAP3430_GR_MOD, OMAP3_PRM_VC_CMD_VAL_1_OFFSET);

	prm_write_mod_reg(OMAP3430_CMD1 | OMAP3430_RAV1,
				OMAP3430_GR_MOD,
				OMAP3_PRM_VC_CH_CONF_OFFSET);

	prm_write_mod_reg(OMAP3430_MCODE_SHIFT | OMAP3430_HSEN | OMAP3430_SREN,
				OMAP3430_GR_MOD,
				OMAP3_PRM_VC_I2C_CFG_OFFSET);

	/* Setup voltctrl and other setup times */
	/* XXX CONFIG_SYSOFFMODE has not been implemented yet */
#ifdef CONFIG_SYSOFFMODE
	prm_write_mod_reg(OMAP3430_AUTO_OFF | OMAP3430_AUTO_RET,
			OMAP3430_GR_MOD,
			OMAP3_PRM_VOLTCTRL_OFFSET);

	prm_write_mod_reg(OMAP3430_CLKSETUP_DURATION, OMAP3430_GR_MOD,
			OMAP3_PRM_CLKSETUP_OFFSET);
	prm_write_mod_reg((OMAP3430_VOLTSETUP_TIME2 <<
			OMAP3430_VOLTSETUP_TIME2_OFFSET) |
			(OMAP3430_VOLTSETUP_TIME1 <<
			OMAP3430_VOLTSETUP_TIME1_OFFSET),
			OMAP3430_GR_MOD, OMAP3_PRM_VOLTSETUP1_OFFSET);

	prm_write_mod_reg(OMAP3430_VOLTOFFSET_DURATION, OMAP3430_GR_MOD,
			OMAP3_PRM_VOLTOFFSET_OFFSET);
	prm_write_mod_reg(OMAP3430_VOLTSETUP2_DURATION, OMAP3430_GR_MOD,
			OMAP3_PRM_VOLTSETUP2_OFFSET);
#else
	prm_set_mod_reg_bits(OMAP3430_AUTO_RET, OMAP3430_GR_MOD,
			OMAP3_PRM_VOLTCTRL_OFFSET);
#endif

}

static void sr_configure(struct omap_sr *sr)
{
	u32 sr_config;
	u32 senp_en , senn_en;

	if (sr->clk_length == 0)
		sr_set_clk_length(sr);

	senp_en = sr->senp_mod;
	senn_en = sr->senn_mod;
	if (sr->srid == SR1) {
		sr_config = SR1_SRCONFIG_ACCUMDATA |
			(sr->clk_length << SRCONFIG_SRCLKLENGTH_SHIFT) |
			SRCONFIG_SENENABLE | SRCONFIG_ERRGEN_EN |
			SRCONFIG_MINMAXAVG_EN |
			(senn_en << SRCONFIG_SENNENABLE_SHIFT) |
			(senp_en << SRCONFIG_SENPENABLE_SHIFT) |
			SRCONFIG_DELAYCTRL;

		sr_write_reg(sr, SRCONFIG, sr_config);
		sr_write_reg(sr, AVGWEIGHT, SR1_AVGWEIGHT_SENPAVGWEIGHT |
					SR1_AVGWEIGHT_SENNAVGWEIGHT);

		sr_modify_reg(sr, ERRCONFIG, (SR_ERRWEIGHT_MASK |
			SR_ERRMAXLIMIT_MASK | SR_ERRMINLIMIT_MASK),
			(SR1_ERRWEIGHT | SR1_ERRMAXLIMIT | SR1_ERRMINLIMIT));

	} else if (sr->srid == SR2) {
		sr_config = SR2_SRCONFIG_ACCUMDATA |
			(sr->clk_length << SRCONFIG_SRCLKLENGTH_SHIFT) |
			SRCONFIG_SENENABLE | SRCONFIG_ERRGEN_EN |
			SRCONFIG_MINMAXAVG_EN |
			(senn_en << SRCONFIG_SENNENABLE_SHIFT) |
			(senp_en << SRCONFIG_SENPENABLE_SHIFT) |
			SRCONFIG_DELAYCTRL;

		sr_write_reg(sr, SRCONFIG, sr_config);
		sr_write_reg(sr, AVGWEIGHT, SR2_AVGWEIGHT_SENPAVGWEIGHT |
					SR2_AVGWEIGHT_SENNAVGWEIGHT);
		sr_modify_reg(sr, ERRCONFIG, (SR_ERRWEIGHT_MASK |
			SR_ERRMAXLIMIT_MASK | SR_ERRMINLIMIT_MASK),
			(SR2_ERRWEIGHT | SR2_ERRMAXLIMIT | SR2_ERRMINLIMIT));

	}
	sr->is_sr_reset = 0;
}

static int sr_enable(struct omap_sr *sr, u32 target_opp_no)
{
	u32 nvalue_reciprocal;

	sr->req_opp_no = target_opp_no;

	if (sr->srid == SR1) {
		switch (target_opp_no) {
		case 5:
			nvalue_reciprocal = sr->opp5_nvalue;
			break;
		case 4:
			nvalue_reciprocal = sr->opp4_nvalue;
			break;
		case 3:
			nvalue_reciprocal = sr->opp3_nvalue;
			break;
		case 2:
			nvalue_reciprocal = sr->opp2_nvalue;
			break;
		case 1:
			nvalue_reciprocal = sr->opp1_nvalue;
			break;
		default:
			nvalue_reciprocal = sr->opp3_nvalue;
			break;
		}
	} else {
		switch (target_opp_no) {
		case 3:
			nvalue_reciprocal = sr->opp3_nvalue;
			break;
		case 2:
			nvalue_reciprocal = sr->opp2_nvalue;
			break;
		case 1:
			nvalue_reciprocal = sr->opp1_nvalue;
			break;
		default:
			nvalue_reciprocal = sr->opp3_nvalue;
			break;
		}
	}

	if (nvalue_reciprocal == 0) {
		printk(KERN_NOTICE "OPP%d doesn't support SmartReflex\n",
								target_opp_no);
		return SR_FALSE;
	}

	sr_write_reg(sr, NVALUERECIPROCAL, nvalue_reciprocal);

	/* Enable the interrupt */
	sr_modify_reg(sr, ERRCONFIG,
			(ERRCONFIG_VPBOUNDINTEN | ERRCONFIG_VPBOUNDINTST),
			(ERRCONFIG_VPBOUNDINTEN | ERRCONFIG_VPBOUNDINTST));
	if (sr->srid == SR1) {
		/* Enable VP1 */
		prm_set_mod_reg_bits(PRM_VP1_CONFIG_VPENABLE, OMAP3430_GR_MOD,
				OMAP3_PRM_VP1_CONFIG_OFFSET);
	} else if (sr->srid == SR2) {
		/* Enable VP2 */
		prm_set_mod_reg_bits(PRM_VP2_CONFIG_VPENABLE, OMAP3430_GR_MOD,
				OMAP3_PRM_VP2_CONFIG_OFFSET);
	}

	/* SRCONFIG - enable SR */
	sr_modify_reg(sr, SRCONFIG, SRCONFIG_SRENABLE, SRCONFIG_SRENABLE);
	return SR_TRUE;
}

static void sr_disable(struct omap_sr *sr)
{
	sr->is_sr_reset = 1;

	/* SRCONFIG - disable SR */
	sr_modify_reg(sr, SRCONFIG, SRCONFIG_SRENABLE, ~SRCONFIG_SRENABLE);

	if (sr->srid == SR1) {
		/* Disable VP1 */
		prm_clear_mod_reg_bits(PRM_VP1_CONFIG_VPENABLE, OMAP3430_GR_MOD,
					OMAP3_PRM_VP1_CONFIG_OFFSET);
	} else if (sr->srid == SR2) {
		/* Disable VP2 */
		prm_clear_mod_reg_bits(PRM_VP2_CONFIG_VPENABLE, OMAP3430_GR_MOD,
					OMAP3_PRM_VP2_CONFIG_OFFSET);
	}
}


void sr_start_vddautocomap(int srid, u32 target_opp_no)
{
	struct omap_sr *sr = NULL;

	if (srid == SR1)
		sr = &sr1;
	else if (srid == SR2)
		sr = &sr2;

	if (sr->is_sr_reset == 1) {
		clk_enable(sr->clk);
		sr_configure(sr);
	}

	if (sr->is_autocomp_active == 1)
		printk(KERN_WARNING "SR%d: VDD autocomp is already active\n",
									srid);

	sr->is_autocomp_active = 1;
	if (!sr_enable(sr, target_opp_no)) {
		printk(KERN_WARNING "SR%d: VDD autocomp not activated\n", srid);
		sr->is_autocomp_active = 0;
		if (sr->is_sr_reset == 1)
			clk_disable(sr->clk);
	}
}
EXPORT_SYMBOL(sr_start_vddautocomap);

int sr_stop_vddautocomap(int srid)
{
	struct omap_sr *sr = NULL;

	if (srid == SR1)
		sr = &sr1;
	else if (srid == SR2)
		sr = &sr2;

	if (sr->is_autocomp_active == 1) {
		sr_disable(sr);
		clk_disable(sr->clk);
		sr->is_autocomp_active = 0;
		return SR_TRUE;
	} else {
		printk(KERN_WARNING "SR%d: VDD autocomp is not active\n",
								srid);
		return SR_FALSE;
	}

}
EXPORT_SYMBOL(sr_stop_vddautocomap);

void enable_smartreflex(int srid)
{
	u32 target_opp_no = 0;
	struct omap_sr *sr = NULL;

	if (srid == SR1)
		sr = &sr1;
	else if (srid == SR2)
		sr = &sr2;

	if (sr->is_autocomp_active == 1) {
		if (sr->is_sr_reset == 1) {
			/* Enable SR clks */
			clk_enable(sr->clk);

			if (srid == SR1)
				target_opp_no = get_opp_no(current_vdd1_opp);
			else if (srid == SR2)
				target_opp_no = get_opp_no(current_vdd2_opp);

			sr_configure(sr);

			if (!sr_enable(sr, target_opp_no))
				clk_disable(sr->clk);
		}
	}
}

void disable_smartreflex(int srid)
{
	struct omap_sr *sr = NULL;

	if (srid == SR1)
		sr = &sr1;
	else if (srid == SR2)
		sr = &sr2;

	if (sr->is_autocomp_active == 1) {
		if (sr->is_sr_reset == 0) {

			sr->is_sr_reset = 1;
			/* SRCONFIG - disable SR */
			sr_modify_reg(sr, SRCONFIG, SRCONFIG_SRENABLE,
							~SRCONFIG_SRENABLE);

			/* Disable SR clk */
			clk_disable(sr->clk);
			if (sr->srid == SR1) {
				/* Disable VP1 */
				prm_clear_mod_reg_bits(PRM_VP1_CONFIG_VPENABLE,
						OMAP3430_GR_MOD,
						OMAP3_PRM_VP1_CONFIG_OFFSET);
			} else if (sr->srid == SR2) {
				/* Disable VP2 */
				prm_clear_mod_reg_bits(PRM_VP2_CONFIG_VPENABLE,
						OMAP3430_GR_MOD,
						OMAP3_PRM_VP2_CONFIG_OFFSET);
			}
		}
	}
}

/* Voltage Scaling using SR VCBYPASS */
int sr_voltagescale_vcbypass(u32 target_opp, u8 vsel)
{
	int sr_status = 0;
	u32 vdd, target_opp_no;
	u32 vc_bypass_value;
	u32 reg_addr = 0;
	u32 loop_cnt = 0, retries_cnt = 0;

	vdd = get_vdd(target_opp);
	target_opp_no = get_opp_no(target_opp);

	if (vdd == PRCM_VDD1) {
		sr_status = sr_stop_vddautocomap(SR1);

		prm_rmw_mod_reg_bits(OMAP3430_VC_CMD_ON_MASK,
					(vsel << OMAP3430_VC_CMD_ON_SHIFT),
					OMAP3430_GR_MOD,
					OMAP3_PRM_VC_CMD_VAL_0_OFFSET);
		reg_addr = R_VDD1_SR_CONTROL;

	} else if (vdd == PRCM_VDD2) {
		sr_status = sr_stop_vddautocomap(SR2);

		prm_rmw_mod_reg_bits(OMAP3430_VC_CMD_ON_MASK,
					(vsel << OMAP3430_VC_CMD_ON_SHIFT),
					OMAP3430_GR_MOD,
					OMAP3_PRM_VC_CMD_VAL_1_OFFSET);
		reg_addr = R_VDD2_SR_CONTROL;
	}

	vc_bypass_value = (vsel << OMAP3430_DATA_SHIFT) |
			(reg_addr << OMAP3430_REGADDR_SHIFT) |
			(R_SRI2C_SLAVE_ADDR << OMAP3430_SLAVEADDR_SHIFT);

	prm_write_mod_reg(vc_bypass_value, OMAP3430_GR_MOD,
			OMAP3_PRM_VC_BYPASS_VAL_OFFSET);

	vc_bypass_value = prm_set_mod_reg_bits(OMAP3430_VALID, OMAP3430_GR_MOD,
					OMAP3_PRM_VC_BYPASS_VAL_OFFSET);

	while ((vc_bypass_value & OMAP3430_VALID) != 0x0) {
		loop_cnt++;
		if (retries_cnt > 10) {
			printk(KERN_INFO "Loop count exceeded in check SR I2C"
								"write\n");
			return SR_FAIL;
		}
		if (loop_cnt > 50) {
			retries_cnt++;
			loop_cnt = 0;
			udelay(10);
		}
		vc_bypass_value = prm_read_mod_reg(OMAP3430_GR_MOD,
					OMAP3_PRM_VC_BYPASS_VAL_OFFSET);
	}

	udelay(T2_SMPS_UPDATE_DELAY);

	if (sr_status) {
		if (vdd == PRCM_VDD1)
			sr_start_vddautocomap(SR1, target_opp_no);
		else if (vdd == PRCM_VDD2)
			sr_start_vddautocomap(SR2, target_opp_no);
	}

	return SR_PASS;
}

/* Sysfs interface to select SR VDD1 auto compensation */
static ssize_t omap_sr_vdd1_autocomp_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sr1.is_autocomp_active);
}

static ssize_t omap_sr_vdd1_autocomp_store(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t n)
{
	u32 current_vdd1opp_no;
	unsigned short value;

	if (sscanf(buf, "%hu", &value) != 1 || (value > 1)) {
		printk(KERN_ERR "sr_vdd1_autocomp: Invalid value\n");
		return -EINVAL;
	}

	current_vdd1opp_no = get_opp_no(current_vdd1_opp);

	if (value == 0)
		sr_stop_vddautocomap(SR1);
	else
		sr_start_vddautocomap(SR1, current_vdd1opp_no);

	return n;
}

static struct kobj_attribute sr_vdd1_autocomp = {
	.attr = {
	.name = __stringify(sr_vdd1_autocomp),
	.mode = 0644,
	},
	.show = omap_sr_vdd1_autocomp_show,
	.store = omap_sr_vdd1_autocomp_store,
};

/* Sysfs interface to select SR VDD2 auto compensation */
static ssize_t omap_sr_vdd2_autocomp_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sr2.is_autocomp_active);
}

static ssize_t omap_sr_vdd2_autocomp_store(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t n)
{
	u32 current_vdd2opp_no;
	unsigned short value;

	if (sscanf(buf, "%hu", &value) != 1 || (value > 1)) {
		printk(KERN_ERR "sr_vdd2_autocomp: Invalid value\n");
		return -EINVAL;
	}

	current_vdd2opp_no = get_opp_no(current_vdd2_opp);

	if (value == 0)
		sr_stop_vddautocomap(SR2);
	else
		sr_start_vddautocomap(SR2, current_vdd2opp_no);

	return n;
}

static struct kobj_attribute sr_vdd2_autocomp = {
	.attr = {
	.name = __stringify(sr_vdd2_autocomp),
	.mode = 0644,
	},
	.show = omap_sr_vdd2_autocomp_show,
	.store = omap_sr_vdd2_autocomp_store,
};



static int __init omap3_sr_init(void)
{
	int ret = 0;
	u8 RdReg;

	if (is_sil_rev_greater_than(OMAP3430_REV_ES1_0)) {
		current_vdd1_opp = PRCM_VDD1_OPP3;
		current_vdd2_opp = PRCM_VDD2_OPP3;
	} else {
		current_vdd1_opp = PRCM_VDD1_OPP1;
		current_vdd2_opp = PRCM_VDD1_OPP1;
	}
	if (cpu_is_omap34xx()) {
		sr_clk_init(&sr1_custom_clk);
		sr_clk_init(&sr2_custom_clk);
		sr1.clk = clk_get(NULL, "sr1_custom_clk");
		sr2.clk = clk_get(NULL, "sr2_custom_clk");
	}
	sr_set_clk_length(&sr1);
	sr_set_clk_length(&sr2);

	/* Call the VPConfig, VCConfig, set N Values. */
	sr_set_nvalues(&sr1);
	sr_configure_vp(SR1);

	sr_set_nvalues(&sr2);
	sr_configure_vp(SR2);

	sr_configure_vc();

	/* Enable SR on T2 */
	ret = twl4030_i2c_read_u8(TWL4030_MODULE_PM_RECEIVER, &RdReg,
					R_DCDC_GLOBAL_CFG);

	RdReg |= DCDC_GLOBAL_CFG_ENABLE_SRFLX;
	ret |= twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, RdReg,
					R_DCDC_GLOBAL_CFG);

	printk(KERN_INFO "SmartReflex driver initialized\n");

	ret = sysfs_create_file(power_kobj, &sr_vdd1_autocomp.attr);
	if (ret)
		printk(KERN_ERR "sysfs_create_file failed: %d\n", ret);

	ret = sysfs_create_file(power_kobj, &sr_vdd2_autocomp.attr);
	if (ret)
		printk(KERN_ERR "sysfs_create_file failed: %d\n", ret);

	return 0;
}

late_initcall(omap3_sr_init);
