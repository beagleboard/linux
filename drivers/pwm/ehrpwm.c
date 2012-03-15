/*
 * eHRPWM driver for simple PWM output generation
 *
 * Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed .as is. WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/pwm/pwm.h>
#include <linux/pwm/ehrpwm.h>
#include <linux/pm_runtime.h>

#include <plat/clock.h>
#include <plat/config_pwm.h>

#ifdef DEBUG
#define debug(format, args...) printk(format, ##args)
#else
#define debug(format, args...) { }
#endif

/******************** Time base sub module*****************************/
#define TBCTL				0x0
#define TBSTS				0x2
#define TBPHS				0x6
#define TBCTR				0x8
#define TBPRD				0xA

#define TBCTL_CLKDIV_MASK		(BIT(12) | BIT(11) | BIT(10))
#define TBCTL_HSPCLKDIV_MASK		(BIT(9) | BIT(8) | BIT(7))
#define TBCTL_SYNCOSEL_MASK		(BIT(5) | BIT(4))
#define TBCTL_CTRMODE_MASK		(BIT(1) | BIT(0))

#define	TBCTL_CLKDIV_POS		0xA
#define TBCTL_HSPCLKDIV_POS		0x7
#define TBCTL_PHSEN_POS			0x2
#define TBCTL_SYNCOSEL_POS		0x4
#define TBCTL_PHSDIR_POS		0xD
#define TBCTL_FRC_SYC_POS		0x6
#define TBCTL_LOAD_MD_POS		0x3

#define TBCTL_FREERUN_FREE		0x2
#define TBCTL_CTRMOD_CTRUP		0x0

/******************* Counter-Compare Sub Module ***********************/
#define CMPCTL				0xE
#define CMPA				0x12
#define CMPB				0x14

#define CMPCTL_LDBMODE_MASK		(BIT(3) | BIT(2))
#define CMPCTL_LDAMODE_MASK		(BIT(1) | BIT(0))

#define CMPCTL_SHDAMODE_POS		0x4
#define CMPCTL_SHDBMODE_POS		0x6
#define CMPCTL_LDBMODE_POS		0x2

/*********************** Action Control Sub module ********************/
#define AQCTLA				0x16
#define AQCTLB				0x18
#define AQSFRC				0x1A
#define AQCSFRC				0x1c

#define ACTCTL_CBD_MASK			(BIT(11) | BIT(10))
#define ACTCTL_CBU_MASK			(BIT(9) | BIT(8))
#define ACTCTL_CAD_MASK			(BIT(7) | BIT(6))
#define ACTCTL_CAU_MASK			(BIT(5) | BIT(4))
#define ACTCTL_CPRD_MASK		(BIT(3) | BIT(2))
#define ACTCTL_CZRO_MASK		(BIT(1) | BIT(0))

#define ACTCTL_CTREQPRD_POS		0x2
#define ACTCTL_CTREQCMPAUP_POS		0x4
#define ACTCTL_CTREQCMPADN_POS		0x6
#define ACTCTL_CTREQCMPBUP_POS		0x8
#define ACTCTL_CTREQCMPBDN_POS		0xA

#define ACTCTL_CTREQCMP_LOW		0x1
#define ACTCTL_CTREQCMP_HIGH		0x2
#define ACTCTL_CTREQZRO_LOW		0x1
#define ACTCTL_CTREQZRO_HIGH		0x2

#define AQSFRC_ACTA_MASK		(BIT(1) | BIT(0))
#define AQSFRC_ACTB_MASK		(BIT(4) | BIT(3))
#define AQCSFRC_CFRC_LOAD_MASK		(BIT(7) | BIT(6))
#define AQCSFRC_OUTB_MASK		(BIT(3) | BIT(2))
#define AQCSFRC_OUTA_MASK		(BIT(1) | BIT(0))

#define AQSFRC_ACTB_POS			0x3
#define AQSFRC_OTFRCA_POS		0x2
#define AQSFRC_OTFRCB_POS		0x5
#define AQSFRC_LDMD_POS			0x6

#define AQCSFRC_OUTB_POS		0x2

/******************** Dead Band Generator Sub module *******************/
#define DBCTL				0x1E
#define DBRED				0x20
#define DBFED				0x22

#define DBCTL_INMODE_MASK		(BIT(5) | BIT(4))
#define DBCTL_PLSEL_MASK		(BIT(3) | BIT(2))
#define DBCTL_OUTMODE_MASK		(BIT(1) | BIT(0))

#define DBCTL_INMODE_POS		0x4
#define DBCTL_POLSEL_POS		0x2

/********************** PWM Chopper Sub module ************************/
#define PCCTL				0x3C

#define PCCTL_CHPDUTY_MASK		(BIT(10) | BIT(9) | BIT(8))
#define PCCTL_CHPFREQ_MASK		(BIT(7) | BIT(6) | BIT(5))
#define PCCTL_OSHTWTH_MASK		(BIT(4) | BIT(3) | BIT(2) | BIT(1))

#define PCCTL_CHPDUTY_POS		0x8
#define PCCTL_CHPFRQ_POS		0x5
#define PCCTL_OSTWID_POS		0x1

/*************************Trip-zone submodule **************************/
#define TZSEL				0x24
#define TZCTL				0x28
#define TZEINT				0x2A
#define TZFLG				0x2C
#define TZCLR				0x2E
#define TZFRC				0x30

#define TZCTL_ACTA_MASK			(BIT(1) | BIT(0))
#define TZCTL_ACTB_MASK			(BIT(3) | BIT(2))

#define TZCTL_ACTB_POS			0x2

#define TZEINT_OSHTEVT_POS		0x2
#define TZEINT_CBCEVT_POS		0x1

/*************************Event-Trigger submodule registers**************/
#define ETSEL				0x32
#define ETPS				0x34
#define ETFLG				0x36
#define ETCLR				0x38
#define ETFRC				0x3A

#define ETSEL_INTSEL_MASK		(BIT(2) | BIT(1) | BIT(0))
#define ETPS_INTCNT_MASK		(BIT(3) | BIT(2))
#define ETPS_INTPRD_MASK		(BIT(1) | BIT(0))

#define ETSEL_EN_INT_EN_POS		0x3

/**********************High Resolution Registers ********************/
#define TBPHSHR				0x4
#define CMPAHR				0x10
#define HRCNFG				0x1040

#define AM335X_HRCNFG			0x40

#define HRCNFG_EDGEMD_MASK		(BIT(1) | BIT(0))
#define HRCNFG_LDMD_POS			0x3
#define HRCNFG_CTLMD_POS		0x2

struct ehrpwm_suspend_params {
	struct pwm_device *pch;
	unsigned long req_delay_cycles;
	unsigned long act_delay;
} ehrpwm_suspend_params;

static inline unsigned short ehrpwm_read(struct ehrpwm_pwm *ehrpwm,
	unsigned int offset)
{
	return readw(ehrpwm->mmio_base + offset);
}

static inline void ehrpwm_write(struct ehrpwm_pwm *ehrpwm, unsigned int offset,
	unsigned short val)
{
	writew(val, ehrpwm->mmio_base + offset);
}

static void ehrpwm_reg_config(struct ehrpwm_pwm *ehrpwm, unsigned int offset,
	       unsigned short val, unsigned short mask)
{
	unsigned short read_val;

	read_val = ehrpwm_read(ehrpwm, offset);
	read_val = read_val & ~mask;
	read_val = read_val | val;
	ehrpwm_write(ehrpwm, offset, read_val);
}

static inline struct ehrpwm_pwm *to_ehrpwm_pwm(const struct pwm_device *p)
{
	return pwm_get_drvdata(p);
}

/* Time Base Module Configurations */
int ehrpwm_tb_set_prescalar_val(struct pwm_device *p, unsigned char clkdiv,
	unsigned char hspclkdiv)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);

	if (clkdiv > 0x7 || hspclkdiv > 0x7)
		return -EINVAL;

	ehrpwm_reg_config(ehrpwm, TBCTL, clkdiv << TBCTL_CLKDIV_POS,
		       TBCTL_CLKDIV_MASK);
	ehrpwm_reg_config(ehrpwm, TBCTL, hspclkdiv << TBCTL_HSPCLKDIV_POS,
		       TBCTL_HSPCLKDIV_MASK);

	return 0;
}
EXPORT_SYMBOL(ehrpwm_tb_set_prescalar_val);

int ehrpwm_tb_config_sync(struct pwm_device *p, unsigned char phsen,
	unsigned char syncosel)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);

	if (phsen > 1 || syncosel > 0x3)
		return -EINVAL;

	ehrpwm_reg_config(ehrpwm, TBCTL, phsen << TBCTL_PHSEN_POS, BIT(2));
	ehrpwm_reg_config(ehrpwm, TBCTL, syncosel << TBCTL_SYNCOSEL_POS,
		       TBCTL_SYNCOSEL_MASK);

	return 0;
}
EXPORT_SYMBOL(ehrpwm_tb_config_sync);

int ehrpwm_tb_set_counter_mode(struct pwm_device *p, unsigned char ctrmode,
	       unsigned char phsdir)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);

	if (ctrmode > 0x3 || phsdir > 1)
		return -EINVAL;

	ehrpwm_reg_config(ehrpwm, TBCTL, phsdir << TBCTL_PHSDIR_POS, BIT(13));
	ehrpwm_reg_config(ehrpwm, TBCTL, ctrmode, TBCTL_CTRMODE_MASK);

	return 0;
}
EXPORT_SYMBOL(ehrpwm_tb_set_counter_mode);

int ehrpwm_tb_force_sync(struct pwm_device *p)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);

	ehrpwm_reg_config(ehrpwm, TBCTL, ENABLE << TBCTL_FRC_SYC_POS, BIT(6));

	return 0;
}
EXPORT_SYMBOL(ehrpwm_tb_force_sync);

int ehrpwm_tb_set_periodload(struct pwm_device *p, unsigned char loadmode)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);

	if (loadmode > 0x1)
		return -EINVAL;

	ehrpwm_reg_config(ehrpwm, TBCTL, loadmode << TBCTL_LOAD_MD_POS, BIT(3));

	return 0;
}
EXPORT_SYMBOL(ehrpwm_tb_set_periodload);

int ehrpwm_tb_read_status(struct pwm_device *p, unsigned short *val)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);

	*val = ehrpwm_read(ehrpwm, TBSTS);

	return 0;
}
EXPORT_SYMBOL(ehrpwm_tb_read_status);

int ehrpwm_tb_read_counter(struct pwm_device *p, unsigned short *val)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);

	*val = ehrpwm_read(ehrpwm, TBCTR);

	return 0;
}
EXPORT_SYMBOL(ehrpwm_tb_read_counter);

int ehrpwm_tb_set_period(struct pwm_device *p,	unsigned short val)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);

	ehrpwm_write(ehrpwm, TBPRD, val);

	return 0;
}
EXPORT_SYMBOL(ehrpwm_tb_set_period);

int ehrpwm_tb_set_phase(struct pwm_device *p, unsigned short val)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);

	ehrpwm_write(ehrpwm, TBPHS, val);

	return 0;
}
EXPORT_SYMBOL(ehrpwm_tb_set_phase);

int ehrpwm_cmp_set_cmp_ctl(struct pwm_device *p, unsigned char shdwamode,
	unsigned char shdwbmode, unsigned char loadamode,
	unsigned char loadbmode)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);

	if (shdwamode > 0x1 || shdwbmode > 0x1 || loadamode > 0x3 ||
		loadbmode > 0x3)
		return -EINVAL;

	ehrpwm_reg_config(ehrpwm, CMPCTL, shdwamode << CMPCTL_SHDAMODE_POS,
		       BIT(4));
	ehrpwm_reg_config(ehrpwm, CMPCTL, shdwbmode << CMPCTL_SHDBMODE_POS,
		       BIT(6));
	ehrpwm_reg_config(ehrpwm, CMPCTL, loadamode, CMPCTL_LDAMODE_MASK);
	ehrpwm_reg_config(ehrpwm, CMPCTL, loadbmode << CMPCTL_LDBMODE_POS,
		       CMPCTL_LDBMODE_MASK);

	return 0;
}
EXPORT_SYMBOL(ehrpwm_cmp_set_cmp_ctl);

int ehrpwm_cmp_set_cmp_val(struct pwm_device *p, unsigned char reg,
	unsigned short val)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);
	unsigned char offset;

	if (reg > 0x1)
		return -EINVAL;

	if (reg == 0)
		offset = CMPA;
	else
		offset = CMPB;

	ehrpwm_write(ehrpwm, offset, val);

	return 0;
}
EXPORT_SYMBOL(ehrpwm_cmp_set_cmp_val);

int ehrpwm_aq_set_act_ctrl(struct pwm_device *p, struct aq_config_params *cfg)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);
	unsigned char reg;

	if (!cfg)
		return -EINVAL;

	if (cfg->ch > 1 || cfg->ctreqzro > 3 || cfg->ctreqprd > 3 ||
		cfg->ctreqcmpaup > 3 || cfg->ctreqcmpadown > 3 ||
		cfg->ctreqcmpbup > 3 || cfg->ctreqcmpbdown > 3)
		return -EINVAL;

	if (cfg->ch == 0)
		reg = AQCTLA;
	else
		reg = AQCTLB;

	ehrpwm_reg_config(ehrpwm, reg, cfg->ctreqzro, ACTCTL_CZRO_MASK);
	ehrpwm_reg_config(ehrpwm, reg, cfg->ctreqprd << ACTCTL_CTREQPRD_POS,
		       ACTCTL_CPRD_MASK);
	ehrpwm_reg_config(ehrpwm, reg, cfg->ctreqcmpaup <<
		       ACTCTL_CTREQCMPAUP_POS, ACTCTL_CAU_MASK);
	ehrpwm_reg_config(ehrpwm, reg, cfg->ctreqcmpadown <<
		       ACTCTL_CTREQCMPADN_POS, ACTCTL_CAD_MASK);
	ehrpwm_reg_config(ehrpwm, reg, cfg->ctreqcmpbup <<
		       ACTCTL_CTREQCMPBUP_POS, ACTCTL_CBU_MASK);
	ehrpwm_reg_config(ehrpwm, reg, cfg->ctreqcmpbdown <<
		       ACTCTL_CTREQCMPBDN_POS, ACTCTL_CBD_MASK);

	return 0;
}
EXPORT_SYMBOL(ehrpwm_aq_set_act_ctrl);

int ehrpwm_aq_set_one_shot_act(struct pwm_device  *p, unsigned char ch,
	unsigned char act)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);

	if (ch > 1 || act > 3)
		return -EINVAL;

	if (ch == 0)
		ehrpwm_reg_config(ehrpwm, AQSFRC, act, AQSFRC_ACTA_MASK);
	else
		ehrpwm_reg_config(ehrpwm, AQSFRC, act << AQSFRC_ACTB_POS,
			       AQSFRC_ACTB_MASK);

	return 0;
}
EXPORT_SYMBOL(ehrpwm_aq_set_one_shot_act);

int ehrpwm_aq_ot_frc(struct pwm_device *p, unsigned char ch)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);

	if (ch > 1)
		return -EINVAL;

	if (ch == 0)
		ehrpwm_reg_config(ehrpwm, AQSFRC, ENABLE << AQSFRC_OTFRCA_POS,
			       BIT(2));
	else
		ehrpwm_reg_config(ehrpwm, AQSFRC, ENABLE << AQSFRC_OTFRCB_POS,
			       BIT(5));

	return 0;
}
EXPORT_SYMBOL(ehrpwm_aq_ot_frc);

int ehrpwm_aq_set_csfrc_load_mode(struct pwm_device *p, unsigned char loadmode)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);

	if (loadmode > 0x3)
		return -EINVAL;

	ehrpwm_reg_config(ehrpwm, AQSFRC, loadmode << AQSFRC_LDMD_POS,
		       AQCSFRC_CFRC_LOAD_MASK);

	return 0;
}
EXPORT_SYMBOL(ehrpwm_aq_set_csfrc_load_mode);

int ehrpwm_aq_continuous_frc(struct pwm_device *p, unsigned char ch,
	unsigned char act)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);

	if (ch > 1)
		return -EINVAL;

	if (ch == 0)
		ehrpwm_reg_config(ehrpwm, AQCSFRC, act, AQCSFRC_OUTA_MASK);
	else
		ehrpwm_reg_config(ehrpwm, AQCSFRC, act << AQCSFRC_OUTB_POS,
			       AQCSFRC_OUTB_MASK);

	return 0;
}
EXPORT_SYMBOL(ehrpwm_aq_continuous_frc);

int ehrpwm_db_get_max_delay(struct pwm_device *p, enum config_mask cfgmask,
	unsigned long *delay_val)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);
	unsigned long delay_ns;
	unsigned long max_ticks;

	if (cfgmask == CONFIG_NS) {
		max_ticks = 0x3ff * ehrpwm->prescale_val;
		delay_ns = pwm_ticks_to_ns(p, max_ticks);
		*delay_val = delay_ns;
	} else if (cfgmask == CONFIG_TICKS) {
		*delay_val = 0x3ff * ehrpwm->prescale_val;
	} else {
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(ehrpwm_db_get_max_delay);

int ehrpwm_db_get_delay(struct pwm_device *p, unsigned char edge,
	enum config_mask cfgmask, unsigned long *delay_val)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);
	unsigned long delay_ns;
	unsigned long delay_ticks;
	unsigned char offset;

	if (!ehrpwm)
		return -EINVAL;

	if (edge == RISING_EDGE_DELAY)
		offset = DBRED;
	else if (edge == FALLING_EDGE_DELAY)
		offset = DBFED;
	else
		return -EINVAL;

	delay_ticks = ehrpwm_read(ehrpwm, offset);
	/* Only least 10 bits are required */
	delay_ticks = delay_ticks & 0x3ff;
	if (cfgmask == CONFIG_TICKS) {
		*delay_val = delay_ticks * ehrpwm->prescale_val;
	} else if (cfgmask == CONFIG_NS) {
		delay_ticks = delay_ticks * ehrpwm->prescale_val;
		delay_ns = pwm_ticks_to_ns(p, delay_ticks);
		debug("\n delay ns value is %lu", delay_ns);
		*delay_val = delay_ns;
	} else {
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(ehrpwm_db_get_delay);

int ehrpwm_db_set_delay(struct pwm_device *p, unsigned char edge,
		enum config_mask cfgmask, unsigned long delay)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);
	unsigned long delay_ticks;
	unsigned char offset = 0;

	if (!ehrpwm)
		return -EINVAL;

	if (edge == RISING_EDGE_DELAY)
		offset = DBRED;
	else if (edge == FALLING_EDGE_DELAY)
		offset = DBFED;
	else
		return -EINVAL;

	if (cfgmask == CONFIG_TICKS) {
		delay = delay / ehrpwm->prescale_val;
		if (delay > 0x3ff)
			return -EINVAL;
		ehrpwm_write(ehrpwm, offset, delay);
	} else if (cfgmask == CONFIG_NS) {
		delay_ticks = pwm_ns_to_ticks(p, delay);
		delay_ticks = delay_ticks / ehrpwm->prescale_val;
		if (delay_ticks > 0x3ff) {
			ehrpwm_db_get_max_delay(p, CONFIG_NS, &delay_ticks);
			dev_dbg(p->dev, "%s: Expected delay cannot be"
			" attained setting the maximum possible delay of"
			" %lu ns", __func__, delay_ticks);
			delay_ticks = 0x3ff;
		}
		debug("\n delay ticks is %lu", delay_ticks);
		ehrpwm_write(ehrpwm, offset, delay_ticks);
	} else {
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(ehrpwm_db_set_delay);

/* Dead Band Configuration functions */
int ehrpwm_db_set_mode(struct pwm_device *p, unsigned char inmode,
		unsigned char polsel, unsigned char outmode)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);

	if (inmode > 0x3 || polsel > 0x3 || outmode > 0x3)
		return -EINVAL;

	ehrpwm_reg_config(ehrpwm, DBCTL, inmode << DBCTL_INMODE_POS,
		       DBCTL_INMODE_MASK);
	ehrpwm_reg_config(ehrpwm, DBCTL, polsel << DBCTL_POLSEL_POS,
		       DBCTL_PLSEL_MASK);
	ehrpwm_reg_config(ehrpwm, DBCTL, outmode, DBCTL_OUTMODE_MASK);

	return 0;
}
EXPORT_SYMBOL(ehrpwm_db_set_mode);

/* PWM chopper Configuration functions */
int ehrpwm_pc_configure(struct pwm_device *p, unsigned char chpduty,
		unsigned char chpfreq, unsigned char oshtwidth)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);

	if (chpduty > 0x7 || chpfreq > 0x7 || oshtwidth > 0xf)
		return -EINVAL;

	ehrpwm_reg_config(ehrpwm, PCCTL, chpduty << PCCTL_CHPDUTY_POS,
		       PCCTL_CHPDUTY_MASK);
	ehrpwm_reg_config(ehrpwm, PCCTL, chpfreq << PCCTL_CHPFRQ_POS,
		       PCCTL_CHPFREQ_MASK);
	ehrpwm_reg_config(ehrpwm, PCCTL, oshtwidth << PCCTL_OSTWID_POS,
		       PCCTL_OSHTWTH_MASK);

	return 0;
}
EXPORT_SYMBOL(ehrpwm_pc_configure);

int ehrpwm_pc_en_dis(struct pwm_device *p, unsigned char chpen)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);

	if (chpen > 1)
		return -EINVAL;

	ehrpwm_reg_config(ehrpwm, PCCTL, chpen, BIT(0));

	return 0;
}
EXPORT_SYMBOL(ehrpwm_pc_en_dis);

/* Trip Zone configuration functions */
int ehrpwm_tz_sel_event(struct pwm_device *p, unsigned char input,
	       enum tz_event evt)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);

	unsigned short val = 0;
	unsigned short mask;
	unsigned short pos;

	if (evt > 4 || input > 7)
		return -EINVAL;

	switch (evt) {
	case TZ_ONE_SHOT_EVENT:
		pos = input + 8;
		mask = BIT((pos)) | BIT(input);
		ehrpwm_reg_config(ehrpwm, TZSEL, 1 << pos, mask);
		break;

	case TZ_CYCLE_BY_CYCLE:
		pos = input;
		mask = BIT(pos) | BIT((pos + 8));
		ehrpwm_reg_config(ehrpwm, TZSEL, 1 << pos, mask);
		break;

	case TZ_OSHT_CBC:
	case TZ_DIS_EVT:
		if (evt == TZ_OSHT_CBC)
			val = 1;
		else
			val = 0;

		pos = input + 8;
		mask = BIT((pos));
		ehrpwm_reg_config(ehrpwm, TZSEL, val << pos, mask);
		pos = input;
		mask = BIT((pos));
		ehrpwm_reg_config(ehrpwm, TZSEL, val << pos, mask);
		break;

	default:
		dev_dbg(p->dev, "%s: Invalid command", __func__);
		return -EINVAL;
	}
	debug("\n TZ_sel val is %0x", ehrpwm_read(ehrpwm, TZSEL));

	return 0;
}
EXPORT_SYMBOL(ehrpwm_tz_sel_event);

int ehrpwm_tz_set_action(struct pwm_device *p, unsigned char ch,
	unsigned char act)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);

	if (act > 0x3 || ch > 1)
		return -EINVAL;

	if (ch == 0)
		ehrpwm_reg_config(ehrpwm, TZCTL, act, TZCTL_ACTA_MASK);
	else
		ehrpwm_reg_config(ehrpwm, TZCTL, act << TZCTL_ACTB_POS,
			       TZCTL_ACTB_MASK);

	debug("\n TZCTL reg val is %0x", ehrpwm_read(ehrpwm, TZCTL));

	return 0;
}
EXPORT_SYMBOL(ehrpwm_tz_set_action);

int ehrpwm_tz_set_int_en_dis(struct pwm_device *p, enum tz_event event,
		unsigned char int_en_dis)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);

	if (event == TZ_ONE_SHOT_EVENT)
		ehrpwm_reg_config(ehrpwm, TZEINT, int_en_dis <<
			       TZEINT_OSHTEVT_POS, BIT(2));
	else if (event == TZ_CYCLE_BY_CYCLE)
		ehrpwm_reg_config(ehrpwm, TZEINT, int_en_dis <<
			       TZEINT_CBCEVT_POS, BIT(1));
	else
		return -EINVAL;

	debug("\n TZEINT reg val is %0x", ehrpwm_read(ehrpwm, TZEINT));

	return 0;
}
EXPORT_SYMBOL(ehrpwm_tz_set_int_en_dis);

int ehrpwm_tz_force_evt(struct pwm_device *p, enum tz_event event)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);

	if (event == TZ_ONE_SHOT_EVENT)
		ehrpwm_write(ehrpwm, TZFRC, 0x4);
	else if (event == TZ_CYCLE_BY_CYCLE)
		ehrpwm_write(ehrpwm, TZFRC, 0x2);
	else
		return -EINVAL;

	return 0;
}
EXPORT_SYMBOL(ehrpwm_tz_force_evt);

inline int ehrpwm_tz_read_status(struct pwm_device *p, unsigned short *status)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);

	*status = ehrpwm_read(ehrpwm, TZFLG);

	return 0;
}
EXPORT_SYMBOL(ehrpwm_tz_read_status);

inline int ehrpwm_tz_clr_evt_status(struct pwm_device *p)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);
	unsigned short ret;

	ret = ehrpwm_read(ehrpwm, TZFLG);
	ehrpwm_write(ehrpwm, TZCLR, ret & ~0x1);

	return 0;
}
EXPORT_SYMBOL(ehrpwm_tz_clr_evt_status);

inline int ehrpwm_tz_clr_int_status(struct pwm_device *p)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);

	ehrpwm_write(ehrpwm, TZCLR, 0x1);

	return 0;
}
EXPORT_SYMBOL(ehrpwm_tz_clr_int_status);

/* Event Trigger Configuration functions */
int ehrpwm_et_set_sel_evt(struct pwm_device *p, unsigned char evt,
		unsigned char prd)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);

	if (evt > 0x7 || prd > 0x3)
		return -EINVAL;

	ehrpwm_reg_config(ehrpwm, ETSEL, evt, ETSEL_INTSEL_MASK);
	ehrpwm_reg_config(ehrpwm, ETPS, prd, ETPS_INTPRD_MASK);

	return 0;
}
EXPORT_SYMBOL(ehrpwm_et_set_sel_evt);

inline int ehrpwm_et_int_en_dis(struct pwm_device *p, unsigned char en_dis)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);

	ehrpwm_reg_config(ehrpwm, ETSEL, en_dis << ETSEL_EN_INT_EN_POS, BIT(3));

	return 0;
}
EXPORT_SYMBOL(ehrpwm_et_int_en_dis);

inline int ehrpwm_et_read_evt_cnt(struct pwm_device *p,
	unsigned long *evtcnt)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);

	*evtcnt = ehrpwm_read(ehrpwm, ETPS) & ETPS_INTCNT_MASK;
	*evtcnt >>= 0x2;

	return 0;
}
EXPORT_SYMBOL(ehrpwm_et_read_evt_cnt);

inline int ehrpwm_et_read_int_status(struct pwm_device *p,
	unsigned long *status)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);

	*status = ehrpwm_read(ehrpwm, ETFLG) & BIT(0);

	return 0;
}
EXPORT_SYMBOL(ehrpwm_et_read_int_status);

inline int ehrpwm_et_frc_int(struct pwm_device *p)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);

	ehrpwm_write(ehrpwm, ETFRC, ENABLE);

	return 0;
}
EXPORT_SYMBOL(ehrpwm_et_frc_int);

inline int ehrpwm_et_clr_int(struct pwm_device *p)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);

	ehrpwm_write(ehrpwm, ETCLR, ENABLE);

	return 0;
}
EXPORT_SYMBOL(ehrpwm_et_clr_int);

/* High Resolution Module configuration */
inline int ehrpwm_hr_set_phase(struct pwm_device *p, unsigned char val)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);

	ehrpwm_write(ehrpwm, TBPHSHR, val << 8);

	return 0;
}
EXPORT_SYMBOL(ehrpwm_hr_set_phase);

inline int ehrpwm_hr_set_cmpval(struct pwm_device *p, unsigned char val)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);

	ehrpwm_write(ehrpwm, CMPAHR, val << 8);

	return 0;
}
EXPORT_SYMBOL(ehrpwm_hr_set_cmpval);

int ehrpwm_hr_config(struct pwm_device *p, unsigned char loadmode,
		unsigned char ctlmode, unsigned char edgemode)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);

	if (loadmode > 1 || ctlmode > 1 || edgemode > 3)
		return -EINVAL;

	if (ehrpwm->version == PWM_VERSION_1) {
		ehrpwm_reg_config(ehrpwm, AM335X_HRCNFG,
				loadmode << HRCNFG_LDMD_POS, BIT(3));
		ehrpwm_reg_config(ehrpwm, AM335X_HRCNFG,
				ctlmode << HRCNFG_CTLMD_POS, BIT(2));
		ehrpwm_reg_config(ehrpwm, AM335X_HRCNFG,
				edgemode, HRCNFG_EDGEMD_MASK);
	} else {
		ehrpwm_reg_config(ehrpwm, HRCNFG,
				loadmode << HRCNFG_LDMD_POS, BIT(3));
		ehrpwm_reg_config(ehrpwm, HRCNFG,
				ctlmode << HRCNFG_CTLMD_POS, BIT(2));
		ehrpwm_reg_config(ehrpwm, HRCNFG,
				edgemode, HRCNFG_EDGEMD_MASK);
	}

	return 0;
}
EXPORT_SYMBOL(ehrpwm_hr_config);

inline int ehrpwm_reg_read(struct pwm_device *p, unsigned int reg,
	       unsigned short *val)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);

	if (!(ehrpwm->version == PWM_VERSION_1)) {
		if (reg > HRCNFG)
			return -EINVAL;
	}

	*val = ehrpwm_read(ehrpwm, reg);

	return 0;
}
EXPORT_SYMBOL(ehrpwm_reg_read);

inline int ehrpwm_reg_write(struct pwm_device *p, unsigned int reg,
	       unsigned short val)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);

	if (!(ehrpwm->version == PWM_VERSION_1)) {
		if (reg > HRCNFG)
			return -EINVAL;
	}

	ehrpwm_write(ehrpwm, reg, val);

	return 0;
}
EXPORT_SYMBOL(ehrpwm_reg_write);

static int ehrpwm_pwm_set_pol(struct pwm_device *p)
{
	unsigned int act_ctrl_reg;
	unsigned int cmp_reg;
	unsigned int ctreqcmp_mask;
	unsigned int ctreqcmp;
	unsigned short val;
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);
	int chan;

	chan = p - &ehrpwm->pwm[0];
	if (!chan) {
		act_ctrl_reg = AQCTLA;
		cmp_reg = CMPA;
		ctreqcmp_mask = ACTCTL_CAU_MASK;
		ctreqcmp = 4;
	} else {
		act_ctrl_reg = AQCTLB;
		cmp_reg = CMPB;
		ctreqcmp_mask = ACTCTL_CBU_MASK;
		ctreqcmp = 8;
	}


	pm_runtime_get_sync(ehrpwm->dev);
	val = ((p->active_high ? ACTCTL_CTREQCMP_HIGH : ACTCTL_CTREQCMP_LOW)
		 << ctreqcmp) | (p->active_high ? ACTCTL_CTREQZRO_LOW :
			ACTCTL_CTREQZRO_HIGH);
	ehrpwm_write(ehrpwm, act_ctrl_reg, val);
	pm_runtime_put_sync(ehrpwm->dev);
	return 0;
}

static int ehrpwm_pwm_start(struct pwm_device *p)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);
	unsigned short val;
	unsigned short read_val1;
	unsigned short read_val2;
	int chan;


	/* Trying to start a running PWM, not allowed */
	if (pwm_is_running(p))
		return -EPERM;

	/* For PWM clock should be enabled on start */
	pm_runtime_get_sync(ehrpwm->dev);

	ehrpwm_pwm_set_pol(p);
	chan = p - &ehrpwm->pwm[0];
	val = ehrpwm_read(ehrpwm, TBCTL);
	val = (val & ~TBCTL_CTRMODE_MASK) | (TBCTL_CTRMOD_CTRUP |
		 TBCTL_FREERUN_FREE << 14);
	ehrpwm_write(ehrpwm, TBCTL, val);
	ehrpwm_tz_set_action(p, chan, 0x3);
	read_val1 = ehrpwm_read(ehrpwm, TZFLG);
	read_val2 = ehrpwm_read(ehrpwm, TZCTL);
	/*
	 * State of the other channel is determined by reading the
	 * TZCTL register. If the other channel is also in running state,
	 * one shot event status is cleared, otherwise one shot action for
	 * this channel is set to "DO NOTHING.
	 */
	read_val2 = read_val2 & (chan ? 0x3 : (0x3 << 2));
	read_val2 = chan ? read_val2 : (read_val2 >> 2);
	if (!(read_val1 & 0x4) || (read_val2 == 0x3))
		ehrpwm_tz_clr_evt_status(p);

	set_bit(FLAG_RUNNING, &p->flags);
	return 0;
}

/*
 * Stop function is implemented using the Trip Zone module. Action for the
 * corresponding channel is set to low and the one shot software force
 * event is triggered.
 */
static int ehrpwm_pwm_stop(struct pwm_device *p)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);
	unsigned short read_val;
	int chan;

	/* Trying to stop a non-running PWM, not allowed */
	if (!pwm_is_running(p))
		return -EPERM;

	chan = p - &ehrpwm->pwm[0];
	/* Set the Trip Zone Action to low */
	ehrpwm_tz_set_action(p, chan, 0x2);
	read_val = ehrpwm_read(ehrpwm, TZFLG);
	/*
	 * If the channel is already in stop state, Trip Zone software force is
	 * not required
	 */
	if (!(read_val & 0x4)) {
		ehrpwm_tz_clr_evt_status(p);
		ehrpwm_tz_force_evt(p, TZ_ONE_SHOT_EVENT);
	}

	/* For PWM clock should be disabled on stop */
	pm_runtime_put_sync(ehrpwm->dev);
	clear_bit(FLAG_RUNNING, &p->flags);
	return 0;
}

/*
 * Prescalar is used when the period value exceeds the maximum value
 * of the 16 bit period register. We always look for the minimum prescalar
 * value as it would result in wide range of duty control
 */
static char get_divider_val(unsigned int desired_ps_val, unsigned int
*ps_div_val, unsigned int *tb_div_val)
{
	char i = 0;
	char j = 0;

	for (i = 0; i <= 7; i++) {
		for (j = 0; j <= 7; j++) {
			if (((1 << i) * (j ? (j * 2) : 1)) >= desired_ps_val) {
				*ps_div_val = (1 << i) * (j ? (j * 2) : 1);
				*tb_div_val = (i << 10) | (j << 7) ;
				return 0;
			}
		}
	}

	return -1;
}

static int ehrpwm_pwm_set_prd(struct pwm_device *p)
{
	unsigned int ps_div_val = 1;
	unsigned int tb_div_val = 0;
	char ret;
	unsigned short val;
	unsigned short period_ticks;
	struct pwm_device *temp;
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);
	int chan = 0;
	/*
	 * Since the device has a singe period register, copy the period
	 * value to the other channel also.
	 */
	chan = p - &ehrpwm->pwm[0];
	temp = &ehrpwm->pwm[!chan];
	temp->period_ticks = p->period_ticks;
	temp->period_ns = p->period_ns;
	debug("\n period_ticks is %lu", p->period_ticks);

	if (p->period_ticks > 65535) {
		ret = get_divider_val(p->period_ticks / 65535 + 1, &ps_div_val,
				&tb_div_val);
		if (ret) {
			dev_err(p->dev, "failed to get the divider value");
			return -EINVAL;
		}
	}

	pm_runtime_get_sync(ehrpwm->dev);
	val = ehrpwm_read(ehrpwm, TBCTL);
	val = (val & ~TBCTL_CLKDIV_MASK & ~TBCTL_HSPCLKDIV_MASK) | tb_div_val;
	ehrpwm_write(ehrpwm, TBCTL, val);
	period_ticks = p->period_ticks / ps_div_val;

	if (period_ticks <= 1) {
		dev_err(p->dev, "Required period/frequency cannot be obtained");
		pm_runtime_put_sync(ehrpwm->dev);
		return -EINVAL;
	}
	/*
	 * Program the period register with 1 less than the actual value since
	 * the module generates waveform with period always 1 greater
	 * the programmed value.
	 */
	ehrpwm_write(ehrpwm, TBPRD, (unsigned short)(period_ticks - 1));
	pm_runtime_put_sync(ehrpwm->dev);
	debug("\n period_ticks is %d", period_ticks);
	ehrpwm->prescale_val = ps_div_val;
	debug("\n Prescaler value is %d", ehrpwm->prescale_val);

	return 0;
}

static int ehrpwm_hr_duty_config(struct pwm_device *p)
{
	unsigned char no_of_mepsteps;
	unsigned short cmphr_val;
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);

	if (!p->tick_hz) {
		dev_dbg(p->dev, "%s: p->tick_hz is zero\n", __func__);
		return -EINVAL;
	}

	/*
	 * Calculate the no of MEP steps. Assume system clock
	 * is in the order of MHZ.
	 */
	no_of_mepsteps = USEC_PER_SEC / ((p->tick_hz / USEC_PER_SEC) * 63);

	pm_runtime_get_sync(ehrpwm->dev);
	/* Calculate the CMPHR Value */
	cmphr_val = p->tick_hz / USEC_PER_SEC;
	cmphr_val = (p->duty_ns * cmphr_val) % MSEC_PER_SEC;
	cmphr_val = (cmphr_val * no_of_mepsteps) / 1000;
	cmphr_val = (cmphr_val << 8) + 0x180;
	ehrpwm_write(ehrpwm, CMPAHR, cmphr_val);

	if (ehrpwm->version == PWM_VERSION_1)
		ehrpwm_write(ehrpwm, AM335X_HRCNFG, 0x2);
	else
		ehrpwm_write(ehrpwm, HRCNFG, 0x2);

	pm_runtime_put_sync(ehrpwm->dev);
	return 0;
}

static int ehrpwm_pwm_set_dty(struct pwm_device *p)
{
	unsigned short duty_ticks = 0;
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);
	int ret = 0;
	int chan;

	chan = p - &ehrpwm->pwm[0];

	if (!ehrpwm->prescale_val) {
		dev_dbg(p->dev, "%s: prescale_val is zero\n", __func__);
		return -EINVAL;
	}

	duty_ticks = p->duty_ticks / ehrpwm->prescale_val;
	debug("\n Prescaler value is %d", ehrpwm->prescale_val);
	debug("\n duty ticks is %d", duty_ticks);
	pm_runtime_get_sync(ehrpwm->dev);
	/* High resolution module */
	if (chan && ehrpwm->prescale_val <= 1) {
		ret = ehrpwm_hr_duty_config(p);
		if (ehrpwm->version == PWM_VERSION_1)
			ehrpwm_write(ehrpwm, AM335X_HRCNFG, 0x2);
		else
			ehrpwm_write(ehrpwm, HRCNFG, 0x2);
	}

	ehrpwm_write(ehrpwm, (chan ? CMPB : CMPA), duty_ticks);
	pm_runtime_put_sync(ehrpwm->dev);
	return ret;
}

int ehrpwm_et_cb_register(struct pwm_device *p, void *data,
	p_fcallback cb)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);
	unsigned long flags;

	spin_lock_irqsave(&ehrpwm->lock, flags);
	ehrpwm->st_etint.data = data;
	ehrpwm->st_etint.pcallback = cb;
	spin_unlock_irqrestore(&ehrpwm->lock, flags);

	return 0;
}
EXPORT_SYMBOL(ehrpwm_et_cb_register);

int ehrpwm_tz_cb_register(struct pwm_device *p, void *data,
	p_fcallback cb)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);
	unsigned long flags;

	spin_lock_irqsave(&ehrpwm->lock, flags);
	ehrpwm->st_tzint.data = data;
	ehrpwm->st_tzint.pcallback = cb;
	spin_unlock_irqrestore(&ehrpwm->lock, flags);

	return 0;
}
EXPORT_SYMBOL(ehrpwm_tz_cb_register);

static int ehrpwm_pwm_suspend_cb(struct ehrpwm_pwm *ehrpwm, void *data)
{
	struct ehrpwm_suspend_params *pwm_suspend_params =
				(struct ehrpwm_suspend_params *)data;

	if (pwm_suspend_params->act_delay++ >= pwm_suspend_params->
			req_delay_cycles) {
		pwm_start(pwm_suspend_params->pch);
		ehrpwm_et_cb_register(pwm_suspend_params->pch, NULL, NULL);
		ehrpwm_et_int_en_dis(pwm_suspend_params->pch, DISABLE);
	}

	return 0;
}

int ehrpwm_pwm_suspend(struct pwm_device *p, enum config_mask config_mask,
	       unsigned long val)
{
	unsigned long long req_cycles = 0;

	if (!p->period_ns)
		return -EINVAL;

	ehrpwm_pwm_stop(p);
	/* Calculate the delay in terms of cycles */
	if (config_mask == CONFIG_NS)
		req_cycles =  val / p->period_ns;
	else if (config_mask == CONFIG_TICKS)
		req_cycles = val;
	else
		return -EINVAL;

	/* Configute the event interrupt */
	ehrpwm_et_set_sel_evt(p, 0x2, 0x1);
	ehrpwm_suspend_params.pch = p;
	ehrpwm_suspend_params.req_delay_cycles = req_cycles;
	ehrpwm_suspend_params.act_delay = 0;
	ehrpwm_et_cb_register(p, &ehrpwm_suspend_params,
		ehrpwm_pwm_suspend_cb);
	ehrpwm_et_int_en_dis(p, ENABLE);

	return 0;
}
EXPORT_SYMBOL(ehrpwm_pwm_suspend);

static irqreturn_t ehrpwm_trip_zone_irq_handler(int irq, void *data)
{
	struct ehrpwm_pwm *ehrpwm = (struct ehrpwm_pwm *)data;
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&ehrpwm->lock, flags);
	 ret = ehrpwm_read(ehrpwm, TZFLG);
	if (!(ret & 0x1))
		return IRQ_NONE;

	if (ehrpwm->st_tzint.pcallback)
		ret = ehrpwm->st_tzint.pcallback(ehrpwm, ehrpwm->st_tzint.data);

	ret = ehrpwm_read(ehrpwm, TZFLG);
	ehrpwm_write(ehrpwm, TZCLR, ret & ~0x1);
	ehrpwm_write(ehrpwm, TZCLR, 0x1);
	spin_unlock_irqrestore(&ehrpwm->lock, flags);

	return IRQ_HANDLED;
}

static irqreturn_t ehrpwm_event_irq_handler(int irq, void *data)
{
	struct ehrpwm_pwm *ehrpwm = (struct ehrpwm_pwm *)data;
	unsigned long flags;

	spin_lock_irqsave(&ehrpwm->lock, flags);

	if (ehrpwm->st_etint.pcallback)
		ehrpwm->st_etint.pcallback(ehrpwm, ehrpwm->st_etint.data);

	ehrpwm_write(ehrpwm, ETCLR, 0x1);

	spin_unlock_irqrestore(&ehrpwm->lock, flags);

	return IRQ_HANDLED;
}

static int ehrpwm_pwm_config(struct pwm_device *p,
	struct pwm_config *c)
{
	int ret = 0;

	switch (c->config_mask) {
	case BIT(PWM_CONFIG_PERIOD_TICKS):
		if (p->max_period_ticks &&
				(p->max_period_ticks >= c->period_ticks))
			p->period_ticks = p->max_period_ticks;
		else
			p->period_ticks = c->period_ticks;

		ret = ehrpwm_pwm_set_prd(p);
		break;

	case BIT(PWM_CONFIG_DUTY_TICKS):
		p->duty_ticks = c->duty_ticks;
		ret = ehrpwm_pwm_set_dty(p);
		break;

	case BIT(PWM_CONFIG_POLARITY):
		p->active_high = c->polarity;
		ret = ehrpwm_pwm_set_pol(p);
		break;

	case BIT(PWM_CONFIG_START):
		 ret = ehrpwm_pwm_start(p);
		 break;

	case BIT(PWM_CONFIG_STOP):
		ret = ehrpwm_pwm_stop(p);
		break;

	default:
		dev_dbg(p->dev, "%s: Invalid configuration\n", __func__);
		ret = -EINVAL;
	}

	return ret;
}

static int ehrpwm_pwm_request(struct pwm_device *p)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);
	int chan;

	chan = p - &ehrpwm->pwm[0];

	p->tick_hz = clk_get_rate(ehrpwm->clk);
	debug("\n The clk freq is %lu", p->tick_hz);
	ehrpwm_pwm_stop(p);

	return 0;
}

static int ehrpwm_freq_transition_cb(struct pwm_device *p)
{
	struct ehrpwm_pwm *ehrpwm = to_ehrpwm_pwm(p);
	unsigned long duty_ns;

	p->tick_hz = clk_get_rate(ehrpwm->clk);
	duty_ns = p->duty_ns;
	if (pwm_is_running(p)) {
		pwm_stop(p);
		pwm_set_duty_ns(p, 0);
		pwm_set_period_ns(p, p->period_ns);
		pwm_set_duty_ns(p, duty_ns);
		pwm_start(p);
	} else {
		pwm_set_duty_ns(p, 0);
		pwm_set_period_ns(p, p->period_ns);
		pwm_set_duty_ns(p, duty_ns);
	}
		return 0;
}

static int ehrpwm_probe(struct platform_device *pdev)
{
	struct ehrpwm_pwm *ehrpwm = NULL;
	struct resource *r;
	int ret = 0;
	int chan = 0;
	struct pwmss_platform_data *pdata = (&pdev->dev)->platform_data;
	int ch_mask = 0;
	int val;
	char con_id[PWM_CON_ID_STRING_LENGTH] = "epwmss";

	ehrpwm = kzalloc(sizeof(*ehrpwm), GFP_KERNEL);
	if (!ehrpwm) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		ret = -ENOMEM;
		goto err_mem_failure;
	}

	ehrpwm->version = pdata->version;

	if (ehrpwm->version == PWM_VERSION_1) {
		sprintf(con_id, "%s%d_%s", con_id, pdev->id, "fck");
		ehrpwm->clk = clk_get(&pdev->dev, con_id);
	} else
		ehrpwm->clk = clk_get(&pdev->dev, "ehrpwm");

	pm_runtime_enable(&pdev->dev);
	ehrpwm->dev = &pdev->dev;
	if (IS_ERR(ehrpwm->clk)) {
		ret = PTR_ERR(ehrpwm->clk);
		goto err_clock_failure;
	}

	if (ehrpwm->version == PWM_VERSION_1) {
		r = platform_get_resource(pdev, IORESOURCE_MEM, 0);

		if (!r) {
			dev_err(&pdev->dev, "no memory resource defined\n");
			ret = -ENOMEM;
			goto err_resource_mem_failure;
		}

		ehrpwm->config_mem_base = ioremap(r->start, resource_size(r));

		if (!ehrpwm->config_mem_base) {

			dev_err(&pdev->dev, "failed to ioremap() registers\n");
			ret = -ENODEV;
			goto err_free_mem_config;
		}

		pm_runtime_get_sync(ehrpwm->dev);
		val = readw(ehrpwm->config_mem_base + PWMSS_CLKCONFIG);
		val |= BIT(EPWM_CLK_EN);
		writew(val, ehrpwm->config_mem_base + PWMSS_CLKCONFIG);
		pm_runtime_put_sync(ehrpwm->dev);
	} else
		ch_mask = pdata->channel_mask;

	spin_lock_init(&ehrpwm->lock);
	ehrpwm->ops.config = ehrpwm_pwm_config;
	ehrpwm->ops.request = ehrpwm_pwm_request;
	ehrpwm->ops.freq_transition_notifier_cb = ehrpwm_freq_transition_cb;
	ehrpwm->prescale_val = 1;

	if (ehrpwm->version == PWM_VERSION_1)
		r = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	else
		r = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (!r) {
		dev_err(&pdev->dev, "no memory resource defined\n");
		ret = -ENODEV;
		goto err_resource_mem2_failiure;
	}

	r = request_mem_region(r->start, resource_size(r), pdev->name);
	if (!r) {
		dev_err(&pdev->dev, "failed to request memory resource\n");
		ret = -EBUSY;
		goto err_request_mem2_failure;
	}

	ehrpwm->mmio_base = ioremap(r->start, resource_size(r));
	if (!ehrpwm->mmio_base) {
		dev_err(&pdev->dev, "failed to ioremap() registers\n");
		ret = -ENODEV;
		goto err_free_mem2;
	}

	ehrpwm->irq[0] = platform_get_irq(pdev, 0);
	if (ehrpwm->irq[0] == -ENXIO) {
		dev_err(&pdev->dev, "No IRQ resource\n");
		ret = -ENXIO;
		goto err_free_io;
	}

	ret = request_irq(ehrpwm->irq[0], ehrpwm_trip_zone_irq_handler,
				0, "ehrpwmTZ", ehrpwm);
	if (ret)
		goto err_free_io;

	ehrpwm->irq[1] = platform_get_irq(pdev, 1);
	if (ehrpwm->irq[1] == -ENXIO) {
		dev_err(&pdev->dev, "No IRQ resource\n");
		ret = -ENXIO;
		goto err_request_irq;
	}

	ret = request_irq(ehrpwm->irq[1], ehrpwm_event_irq_handler,
				0, "ehrpwm_evt", ehrpwm);
	if (ret)
		goto err_request_irq;

	for (chan = 0; chan < NCHAN; chan++) {
		ehrpwm->pwm[chan].ops = &ehrpwm->ops;
		pwm_set_drvdata(&ehrpwm->pwm[chan], ehrpwm);
		ehrpwm->pwm[chan].tick_hz = clk_get_rate(ehrpwm->clk);

		if (pdata->chan_attrib[chan].max_freq) {
			int period_ns = NSEC_PER_SEC
				/ pdata->chan_attrib[chan].max_freq;

			ehrpwm->pwm[chan].max_period_ticks =
				pwm_ns_to_ticks(&ehrpwm->pwm[chan], period_ns);
		}

		if (!(ehrpwm->version == PWM_VERSION_1)) {
			if (!(ch_mask & (0x1 << chan)))
				continue;
		}

		ret =  pwm_register(&ehrpwm->pwm[chan], &pdev->dev, chan);
		if (ret)
			goto err_pwm_register;
	}

		platform_set_drvdata(pdev, ehrpwm);
	return 0;

err_pwm_register:
	for (chan = 0; chan < NCHAN; chan++) {
		if (pwm_is_registered(&ehrpwm->pwm[chan]))
			pwm_unregister(&ehrpwm->pwm[chan]);
	}

err_request_irq:
	if (ehrpwm->irq[0] != -ENXIO)
		free_irq(ehrpwm->irq[0], ehrpwm);
err_free_io:
	iounmap(ehrpwm->mmio_base);
err_free_mem2:
	release_mem_region(r->start, resource_size(r));
err_request_mem2_failure:
err_resource_mem2_failiure:
	if (ehrpwm->version == PWM_VERSION_1) {
		iounmap(ehrpwm->config_mem_base);
		ehrpwm->config_mem_base = NULL;
	}
err_free_mem_config:
err_resource_mem_failure:
	clk_put(ehrpwm->clk);
	pm_runtime_disable(ehrpwm->dev);
err_clock_failure:
	kfree(ehrpwm);
err_mem_failure:
	return ret;
}

#ifdef CONFIG_PM

void ehrpwm_context_save(struct ehrpwm_pwm *ehrpwm,
		struct ehrpwm_context *ehrpwm_ctx)
{
	pm_runtime_get_sync(ehrpwm->dev);
	ehrpwm_ctx->tbctl = ehrpwm_read(ehrpwm, TBCTL);
	ehrpwm_ctx->tbprd = ehrpwm_read(ehrpwm, TBPRD);
	if (ehrpwm->version == PWM_VERSION_1)
		ehrpwm_ctx->hrcfg = ehrpwm_read(ehrpwm, AM335X_HRCNFG);
	else
		ehrpwm_ctx->hrcfg = ehrpwm_read(ehrpwm, HRCNFG);
	ehrpwm_ctx->aqctla = ehrpwm_read(ehrpwm, AQCTLA);
	ehrpwm_ctx->aqctlb = ehrpwm_read(ehrpwm, AQCTLB);
	ehrpwm_ctx->cmpa = ehrpwm_read(ehrpwm, CMPA);
	ehrpwm_ctx->cmpb = ehrpwm_read(ehrpwm, CMPB);
	ehrpwm_ctx->tzctl = ehrpwm_read(ehrpwm, TZCTL);
	ehrpwm_ctx->tzflg = ehrpwm_read(ehrpwm, TZFLG);
	ehrpwm_ctx->tzclr = ehrpwm_read(ehrpwm, TZCLR);
	ehrpwm_ctx->tzfrc = ehrpwm_read(ehrpwm, TZFRC);
	pm_runtime_put_sync(ehrpwm->dev);
}

void ehrpwm_context_restore(struct ehrpwm_pwm *ehrpwm,
		struct ehrpwm_context *ehrpwm_ctx)
{
	ehrpwm_write(ehrpwm, TBCTL, ehrpwm_ctx->tbctl);
	ehrpwm_write(ehrpwm, TBPRD, ehrpwm_ctx->tbprd);
	if (ehrpwm->version == PWM_VERSION_1)
		ehrpwm_write(ehrpwm, AM335X_HRCNFG, ehrpwm_ctx->hrcfg);
	else
		ehrpwm_write(ehrpwm, HRCNFG, ehrpwm_ctx->hrcfg);
	ehrpwm_write(ehrpwm, AQCTLA, ehrpwm_ctx->aqctla);
	ehrpwm_write(ehrpwm, AQCTLB, ehrpwm_ctx->aqctlb);
	ehrpwm_write(ehrpwm, CMPA, ehrpwm_ctx->cmpa);
	ehrpwm_write(ehrpwm, CMPB, ehrpwm_ctx->cmpb);
	ehrpwm_write(ehrpwm, TZCTL, ehrpwm_ctx->tzctl);
	ehrpwm_write(ehrpwm, TZFLG, ehrpwm_ctx->tzflg);
	ehrpwm_write(ehrpwm, TZCLR, ehrpwm_ctx->tzclr);
	ehrpwm_write(ehrpwm, TZFRC, ehrpwm_ctx->tzfrc);
}

static int ehrpwm_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct ehrpwm_pwm *ehrpwm = platform_get_drvdata(pdev);

	ehrpwm_context_save(ehrpwm, &ehrpwm->ctx);
	pm_runtime_put_sync(ehrpwm->dev);

	return 0;
}

static int ehrpwm_resume(struct platform_device *pdev)
{
	struct ehrpwm_pwm *ehrpwm = platform_get_drvdata(pdev);

	pm_runtime_get_sync(ehrpwm->dev);
	ehrpwm_context_restore(ehrpwm, &ehrpwm->ctx);

	return 0;
}

#else
#define ehrpwm_suspend NULL
#define ehrpwm_resume NULL
#endif /* CONFIG_PM */

static int __devexit ehrpwm_remove(struct platform_device *pdev)
{
	struct ehrpwm_pwm *ehrpwm = platform_get_drvdata(pdev);
	struct resource *r;
	unsigned char i;
	int val;
	struct pwmss_platform_data *pdata;

	if (ehrpwm->version == PWM_VERSION_1) {
		pdata = (&pdev->dev)->platform_data;
		val = readw(ehrpwm->config_mem_base + PWMSS_CLKCONFIG);
		val &= ~BIT(EPWM_CLK_EN);
		writew(val, ehrpwm->config_mem_base + PWMSS_CLKCONFIG);
		iounmap(ehrpwm->config_mem_base);
		ehrpwm->config_mem_base = NULL;
	}

	for (i = 0; i < NCHAN; i++) {
		if (pwm_is_registered(&ehrpwm->pwm[i]))
			pwm_unregister(&ehrpwm->pwm[i]);
	}

	for (i = 0; i < 2; i++)
		if (ehrpwm->irq[i] != -ENXIO)
			free_irq(ehrpwm->irq[i], ehrpwm);
	iounmap(ehrpwm->mmio_base);

	if (ehrpwm->version == PWM_VERSION_1)
		r = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	else
		r = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	release_mem_region(r->start, resource_size(r));
	platform_set_drvdata(pdev, NULL);
	clk_put(ehrpwm->clk);
	pm_runtime_disable(ehrpwm->dev);
	kfree(ehrpwm);

	return 0;
}

static struct platform_driver ehrpwm_driver = {
	.driver	= {
		.name	= "ehrpwm",
		.owner	= THIS_MODULE,
	},
	.probe		= ehrpwm_probe,
	.remove		= __devexit_p(ehrpwm_remove),
	.suspend	= ehrpwm_suspend,
	.resume		= ehrpwm_resume,
};

static int __init ehrpwm_init(void)
{
	return platform_driver_register(&ehrpwm_driver);
}
module_init(ehrpwm_init);

static void __exit ehrpwm_exit(void)
{
	platform_driver_unregister(&ehrpwm_driver);
}
module_exit(ehrpwm_exit);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("Driver for Davinci eHRPWM peripheral");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ehrpwm");
