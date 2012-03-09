#ifndef __EHRPWM_H__
#define __EHRPWM_H__

#include <linux/pwm/pwm.h>

#define NCHAN 2

struct ehrpwm_pwm;

typedef int (*p_fcallback) (struct ehrpwm_pwm *, void *data);

struct et_int {
	void *data;
	p_fcallback pcallback;
};

struct tz_int {
	void *data;
	p_fcallback pcallback;
};

struct ehrpwm_context {
	u32 tbctl;
	u32 tbprd;
	u32 hrcfg;
	u32 aqctla;
	u32 aqctlb;
	u32 cmpa;
	u32 cmpb;
	u32 tzctl;
	u32 tzflg;
	u32 tzclr;
	u32 tzfrc;
};

struct ehrpwm_pwm {
	struct pwm_device pwm[NCHAN];
	struct pwm_device_ops ops;
	spinlock_t      lock;
	struct clk      *clk;
	void __iomem    *mmio_base;
	unsigned short prescale_val;
	int irq[2];
	struct et_int st_etint;
	struct tz_int st_tzint;
	u8 version;
	void __iomem *config_mem_base;
	struct device *dev;
	struct ehrpwm_context ctx;
};

enum tz_event {
	TZ_ONE_SHOT_EVENT = 0,
	TZ_CYCLE_BY_CYCLE,
	TZ_OSHT_CBC,
	TZ_DIS_EVT,
};

enum config_mask {
	CONFIG_NS,
	CONFIG_TICKS,
};

enum db_edge_delay {
	RISING_EDGE_DELAY,
	FALLING_EDGE_DELAY,
};

struct aq_config_params {
	unsigned char ch;
	unsigned char ctreqzro;
	unsigned char ctreqprd;
	unsigned char ctreqcmpaup;
	unsigned char ctreqcmpadown;
	unsigned char ctreqcmpbup;
	unsigned char ctreqcmpbdown;
};

int ehrpwm_tb_set_prescalar_val(struct pwm_device *p, unsigned char
	clkdiv, unsigned char hspclkdiv);

int ehrpwm_tb_config_sync(struct pwm_device *p, unsigned char phsen,
	unsigned char syncosel);

int ehrpwm_tb_set_counter_mode(struct pwm_device *p, unsigned char
	ctrmode, unsigned char phsdir);

int ehrpwm_tb_force_sync(struct pwm_device *p);

int ehrpwm_tb_set_periodload(struct pwm_device *p, unsigned char
	       loadmode);

int ehrpwm_tb_read_status(struct pwm_device *p, unsigned short *val);

int ehrpwm_tb_read_counter(struct pwm_device *p, unsigned short *val);

int ehrpwm_tb_set_period(struct pwm_device *p,	unsigned short val);

int ehrpwm_tb_set_phase(struct pwm_device *p, unsigned short val);

int ehrpwm_cmp_set_cmp_ctl(struct pwm_device *p, unsigned char
	       shdwamode, unsigned char shdwbmode, unsigned char loadamode,
	unsigned char loadbmode);

int ehrpwm_cmp_set_cmp_val(struct pwm_device *p, unsigned char reg,
	unsigned short val);

int ehrpwm_aq_set_act_ctrl(struct pwm_device *p,
	       struct aq_config_params *cfg);

int ehrpwm_aq_set_one_shot_act(struct pwm_device  *p, unsigned char ch,
	unsigned char act);

int ehrpwm_aq_ot_frc(struct pwm_device *p, unsigned char ch);

int ehrpwm_aq_set_csfrc_load_mode(struct pwm_device *p, unsigned char
	       loadmode);

int ehrpwm_aq_continuous_frc(struct pwm_device *p, unsigned char ch,
	unsigned char act);

int ehrpwm_db_get_max_delay(struct pwm_device *p,
	       enum config_mask cfgmask, unsigned long *delay_val);

int ehrpwm_db_get_delay(struct pwm_device *p, unsigned char edge,
	enum config_mask cfgmask, unsigned long *delay_val);

int ehrpwm_db_set_delay(struct pwm_device *p, unsigned char edge,
		enum config_mask cfgmask, unsigned long delay);

int ehrpwm_db_set_mode(struct pwm_device *p, unsigned char inmode,
		unsigned char polsel, unsigned char outmode);

int ehrpwm_pc_configure(struct pwm_device *p, unsigned char chpduty,
		unsigned char chpfreq, unsigned char oshtwidth);

int ehrpwm_pc_en_dis(struct pwm_device *p, unsigned char chpen);

int ehrpwm_tz_sel_event(struct pwm_device *p, unsigned char input,
	       enum tz_event evt);

int ehrpwm_tz_set_action(struct pwm_device *p, unsigned char ch,
	unsigned char act);

int ehrpwm_tz_set_int_en_dis(struct pwm_device *p, enum tz_event event,
		unsigned char int_en_dis);

int ehrpwm_tz_force_evt(struct pwm_device *p, enum tz_event event);

int ehrpwm_tz_read_status(struct pwm_device *p, unsigned short *status);

int ehrpwm_tz_clr_evt_status(struct pwm_device *p);

int ehrpwm_tz_clr_int_status(struct pwm_device *p);

int ehrpwm_et_set_sel_evt(struct pwm_device *p, unsigned char evt,
		unsigned char prd);

int ehrpwm_et_int_en_dis(struct pwm_device *p, unsigned char en_dis);

int ehrpwm_et_read_evt_cnt(struct pwm_device *p, unsigned long *evtcnt);

int pwm_et_read_int_status(struct pwm_device *p,
	unsigned long *status);

int ehrpwm_et_frc_int(struct pwm_device *p);

int ehrpwm_et_clr_int(struct pwm_device *p);

int ehrpwm_hr_set_phase(struct pwm_device *p, unsigned char val);

int ehrpwm_hr_set_cmpval(struct pwm_device *p, unsigned char val);

int ehrpwm_hr_config(struct pwm_device *p, unsigned char loadmode,
		unsigned char ctlmode, unsigned char edgemode);

int ehrpwm_et_cb_register(struct pwm_device *p, void *data,
	p_fcallback cb);

int ehrpwm_tz_cb_register(struct pwm_device *p, void *data,
	p_fcallback cb);

int ehrpwm_pwm_suspend(struct pwm_device *p, enum
	       config_mask config_mask,
	       unsigned long val);

#define ENABLE 1
#define DISABLE 0

#endif
