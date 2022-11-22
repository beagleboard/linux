/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2021 Alibaba Group Holding Limited.
 */

#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/firmware/thead/ipc.h>

#define MBOX_MAX_MSG_LEN	28

#define CONFIG_AON_REG_DEBUG	1

struct rpc_msg_regu_vol_set {
	u16 regu_id;                    ///< virtual regu id
	u16 is_dual_rail;               ///< whether this regu has dual rails
	u32 dc1;                        ///< voltage uinit in uv for single rail or first rail of dual rail
	u32 dc2;                        ///< voltage uinit in uv for second rail of dual rail,ignore it if "is_dual_rail" is false
	u16 reserved[6];
} __packed __aligned(4);

struct rpc_msg_regu_vol_get {
	u16 regu_id;                    ///< virtual regu id
	u16 is_dual_rail;               ///< whether this regu has dual rails
	u32 dc1;                        ///< voltage uinit in uv for single rail or first rail of dual rail
	u32 dc2;                        ///< voltage uinit in uv for second rail of dual rail,ignore it if "is_dual_rail" is false
	u16 reserved[6];

} __packed __aligned(4);

struct rpc_msg_regu_pwr_set {
	u16 regu_id;                    ///< virtual regu id
	u16 status;                     ///< 0: power off; 1: powr on
	u32 reserved[5];
} __packed __aligned(4);

struct rpc_msg_regu_pwr_get {
	u16 regu_id;                    ///< virtual regu id
	u16 status;                     ///< 0: power off; 1: powr on
	u32 reserved[5];

} __packed __aligned(4);

struct light_aon_msg_regulator_ctrl {
	struct light_aon_rpc_msg_hdr hdr;
	union rpc_func_t {
	struct rpc_msg_regu_vol_set    regu_vol_set;
	struct rpc_msg_regu_vol_get    regu_vol_get;
	struct rpc_msg_regu_pwr_set    regu_pwr_set;
	struct rpc_msg_regu_pwr_get    regu_pwr_get;
	} __packed __aligned(4) rpc;
} __packed __aligned(4);

enum pm_resource {
	SOC_DVDD18_AON,      /*da9063:  ldo-3 */
	SOC_AVDD33_USB3,     /*da9063:  ldo-9 */
	SOC_DVDD08_AON,      /*da9063:  ldo-2 */
	SOC_APCPU_DVDD_DVDDM,/*da9063:  vbcore1 & vbcore2*/
	SOC_DVDD08_DDR,      /*da9063:  buckperi */
	SOC_VDD_DDR_1V8,     /*da9063:  ldo-4  */
	SOC_VDD_DDR_1V1,     /*da9063:  buckmem & buckio  */
	SOC_VDD_DDR_0V6,     /*da9063:  buckpro */
	SOC_DVDD18_AP,       /*da9063:  ldo-11 */
	SOC_DVDD08_AP,       /*da9121:  da9121_ex */
	SOC_AVDD08_MIPI_HDMI,/*da9063:  ldo-1  */
	SOC_AVDD18_MIPI_HDMI,/*da9063:  ldo-5  */
	SOC_DVDD33_EMMC,     /*da9063:  ldo-10 */
	SOC_DVDD18_EMMC,     /*slg51000:ldo-3 */
	SOC_DOVDD18_SCAN,    /*da9063:  ldo-6 */
	SOC_VEXT_2V8,        /*da9063:  ldo-7 */
	SOC_DVDD12_SCAN,     /*da9063:  ld0-8 */
	SOC_AVDD28_SCAN_EN,  /*da9063: gpio4 */
	SOC_AVDD28_RGB,      /*slg51000:ldo-1 */
	SOC_DOVDD18_RGB,     /*slg51000:ldo-4 */
	SOC_DVDD12_RGB,      /*slg51000:ldo-5 */
	SOC_AVDD25_IR,       /*slg51000:ldo-2 */
	SOC_DOVDD18_IR,      /*slg51000:ldo-7 */
	SOC_DVDD12_IR,       /*slg51000:ldo-6 */
	SOC_ADC_VREF,
	SOC_LCD0_EN,
	SOC_VEXT_1V8,


	SOC_REGU_MAX
};

struct apcpu_vol_set {
	u32 vdd;               ///< cpu core voltage
	u32 vddm;              ///< cpu core-mem voltage
};

struct aon_regu_desc {
	struct regulator_desc *regu_desc;            ///< discription of regulator
	u32                    regu_num;             ///< element number of regulators,which point to regu-dsc-array
};

struct aon_regu_info {
	struct device              *dev;
	const struct apcpu_vol_set *cpu_vol;         ///< signed-off voltage of cpu
	u32                        vddm;             ///< cpu-mem voltage
	struct aon_regu_desc       *regu_desc;       ///< regu-desc set
	struct light_aon_ipc       *ipc_handle;      ///< handle of mail-box
};

static struct aon_regu_info light_aon_pmic_info;

#define APCPU_VOL_DEF(_vdd, _vddm) \
	{					\
		.vdd = _vdd,		\
		.vddm = _vddm,		\
	}

static const struct apcpu_vol_set apcpu_volts[] = {
	/*300Mhz*/
	APCPU_VOL_DEF(600000U, 750000U),
	APCPU_VOL_DEF(600000U, 800000U),
	APCPU_VOL_DEF(650000U, 800000U),
	APCPU_VOL_DEF(720000U, 770000U),
	/*800Mhz*/
	APCPU_VOL_DEF(700000U,800000U),
	APCPU_VOL_DEF(720000U,820000U),
	/*1500Mhz*/
	APCPU_VOL_DEF(800000U,800000U),
	APCPU_VOL_DEF(820000U,820000U),
	/*1850Mhz*/
	APCPU_VOL_DEF(1000000U,1000000U),
};

/* dc2 is valid when is_dual_rail is true
 *
 * dual-rail regulator means that a virtual regulator involes two hw-regulators
 */
static int aon_set_regulator(struct light_aon_ipc *ipc, u16 regu_id,
			    u32 dc1, u32 dc2, u16 is_dual_rail)
{
	struct light_aon_msg_regulator_ctrl msg = {0};
	struct light_aon_rpc_msg_hdr *hdr = &msg.hdr;

	hdr->ver = LIGHT_AON_RPC_VERSION;
	hdr->svc = (uint8_t)LIGHT_AON_RPC_SVC_PM;
	hdr->func = (uint8_t)LIGHT_AON_PM_FUNC_SET_RESOURCE_REGULATOR;
	hdr->size = LIGHT_AON_RPC_MSG_NUM;

	msg.rpc.regu_vol_set.regu_id = regu_id;
	msg.rpc.regu_vol_set.is_dual_rail = is_dual_rail;
	msg.rpc.regu_vol_set.dc1 = dc1;
	msg.rpc.regu_vol_set.dc2 = dc2;

	return light_aon_call_rpc(ipc, &msg, true);
}

/* dc2 is valid when is_dual_rail is true
 *
 * dual-rail regulator means that a virtual regulator involes two hw-regulators
 */
static int aon_get_regulator(struct light_aon_ipc *ipc, u16 regu_id,
			    u32 *dc1, u32 *dc2, u16 is_dual_rail)
{
	struct light_aon_msg_regulator_ctrl msg = {0};
	struct light_aon_rpc_msg_hdr *hdr = &msg.hdr;
	int ret;

	hdr->ver = LIGHT_AON_RPC_VERSION;
	hdr->svc = (uint8_t)LIGHT_AON_RPC_SVC_PM;
	hdr->func = (uint8_t)LIGHT_AON_PM_FUNC_GET_RESOURCE_REGULATOR;
	hdr->size = LIGHT_AON_RPC_MSG_NUM;
	msg.rpc.regu_vol_get.regu_id  = regu_id;
	msg.rpc.regu_vol_get.is_dual_rail = is_dual_rail;

	ret = light_aon_call_rpc(ipc, &msg, true);
	if (ret)
		return ret;

	if (dc1 != NULL)
		*dc1 = msg.rpc.regu_vol_get.dc1;

	if (dc2 != NULL)
		*dc2 = msg.rpc.regu_vol_get.dc2;

	return 0;
}

static int aon_regu_power_ctrl(struct light_aon_ipc *ipc,u32 regu_id,u16 pwr_on)
{
	struct light_aon_msg_regulator_ctrl msg = {0};
	struct light_aon_rpc_msg_hdr *hdr = &msg.hdr;

	hdr->ver = LIGHT_AON_RPC_VERSION;
	hdr->svc = (uint8_t)LIGHT_AON_RPC_SVC_PM;
	hdr->func = (uint8_t)LIGHT_AON_PM_FUNC_PWR_SET;
	hdr->size = LIGHT_AON_RPC_MSG_NUM;

	msg.rpc.regu_pwr_set.regu_id = regu_id;
	msg.rpc.regu_pwr_set.status  = pwr_on;
	return light_aon_call_rpc(ipc, &msg, true);
}
static int aon_regu_dummy_enable(struct regulator_dev *reg)
{
	return 0;
}
static int aon_regu_dummy_disable(struct regulator_dev *reg)
{
	return 0;
}
static int aon_regu_dummy_is_enabled(struct regulator_dev *reg)
{
	return 0;
}
static int aon_regu_enable(struct regulator_dev *reg)
{
	u16 regu_id =(u16) rdev_get_id(reg);
	return aon_regu_power_ctrl(light_aon_pmic_info.ipc_handle, regu_id, 1);
}

static int aon_regu_disable(struct regulator_dev *reg)
{
	u16 regu_id =(u16) rdev_get_id(reg);
	return aon_regu_power_ctrl(light_aon_pmic_info.ipc_handle, regu_id, 0);
}

static int aon_regu_is_enabled(struct regulator_dev *reg)
{
	struct light_aon_msg_regulator_ctrl msg = {0};
	struct light_aon_rpc_msg_hdr *hdr = &msg.hdr;
	int ret;
	u16 regu_id =(u16) rdev_get_id(reg);

	hdr->ver = LIGHT_AON_RPC_VERSION;
	hdr->svc = (uint8_t)LIGHT_AON_RPC_SVC_PM;
	hdr->func = (uint8_t)LIGHT_AON_PM_FUNC_PWR_GET;
	hdr->size = LIGHT_AON_RPC_MSG_NUM;

	msg.rpc.regu_pwr_get.regu_id = regu_id;
	ret = light_aon_call_rpc(light_aon_pmic_info.ipc_handle, &msg, true);
	if (ret < 0) {
		return ret;
	}

	return (int) msg.rpc.regu_pwr_get.status;
}

static int aon_regu_set_voltage(struct regulator_dev *reg,
				int minuV, int uV, unsigned *selector)
{
	u16 regu_id =(u16) rdev_get_id(reg);
	u32 voltage = minuV; /* uV */
	int err;

	pr_debug("[%s,%d]minuV = %d, uV = %d\n", __func__, __LINE__, minuV, uV);

	err = aon_set_regulator(light_aon_pmic_info.ipc_handle, regu_id,
				       voltage, 0, 0);
	if (err) {
		pr_err("failed to set Voltages to %d!\n", minuV);
		return -EINVAL;
	}

	return 0;
}

static int aon_regu_get_voltage(struct regulator_dev *reg)
{
	u16 regu_id = (u16) rdev_get_id(reg);
	int voltage, ret;

	ret = aon_get_regulator(light_aon_pmic_info.ipc_handle, regu_id,
				&voltage, NULL, 0);
	if (ret) {
		pr_err("failed to get voltage\n");
		return -EINVAL;
	}

	pr_debug("[%s,%d]voltage = %d\n", __func__, __LINE__, voltage);

	return voltage;
}

static const struct apcpu_vol_set *apcpu_get_matched_signed_off_voltage(u32 vdd, u32 vddm)
{
	int vol_count = ARRAY_SIZE(apcpu_volts);
	int i;

	for (i = 0; i < vol_count; i++)
		if ((vdd == apcpu_volts[i].vdd) &&
		    (vddm == apcpu_volts[i].vddm))
			return &apcpu_volts[i];

#ifdef CONFIG_AON_REG_DEBUG
	return &apcpu_volts[2];
#else
	return NULL;
#endif
}

static int apcpu_set_vdd_vddm_voltage(struct regulator_dev *reg,
			      int minuV, int uV, unsigned *selector)
{
	struct aon_regu_info *info = rdev_get_drvdata(reg);
	const struct apcpu_vol_set *cpu_vol;
	u32 vol = minuV; /* uV */
	u32 dc1, dc2;
	int err;

	cpu_vol = apcpu_get_matched_signed_off_voltage(vol, light_aon_pmic_info.vddm);
	if (!cpu_vol) {
		dev_err(info->dev, "failed to find bcore1/bcore2 matching table\n");
#ifndef CONFIG_AON_REG_DEBUG
		return -EINVAL;
#endif
	}

	dc1 = cpu_vol->vdd;
	dc2 = cpu_vol->vddm;
	info->cpu_vol = cpu_vol;
	info->vddm = cpu_vol->vddm;

	err = aon_set_regulator(light_aon_pmic_info.ipc_handle, (u16)SOC_APCPU_DVDD_DVDDM,
				       dc1, dc2, 1);
	if (err) {
		dev_err(info->dev, "failed to set Voltages to %d!\n", uV);
		return -EINVAL;
	}

	return 0;
}

static int apcpu_set_vddm_voltage(struct regulator_dev *reg,
			      int minuV, int uV, unsigned *selector)
{
	struct aon_regu_info *info = rdev_get_drvdata(reg);
	int bcore_table_count = ARRAY_SIZE(apcpu_volts);
	u32 vol = minuV; /* uV */
	int i;

	for (i = 0; i < bcore_table_count; i++)
		if (vol == apcpu_volts[i].vddm)
			break;

	if (i >= bcore_table_count) {
		dev_err(info->dev, "The vol is not existed in matching table\n");
#ifndef CONFIG_AON_REG_DEBUG
		return -EINVAL;
#endif
	}

	/* update the vddm */
	info->vddm = vol;
	return 0;
}

static int apcpu_get_voltage(struct regulator_dev *reg, bool is_vdd)
{
	struct aon_regu_info *info = rdev_get_drvdata(reg);
	const struct apcpu_vol_set *cpu_vol;
	u32 dc1, dc2;
	int err;

	err = aon_get_regulator(light_aon_pmic_info.ipc_handle, SOC_APCPU_DVDD_DVDDM,
				       &dc1, &dc2, 1);
	if (err) {
		dev_err(info->dev, "failed to get Voltages!\n");
		return -EINVAL;
	}
	cpu_vol = apcpu_get_matched_signed_off_voltage(dc1, dc2);
	if (!cpu_vol) {
		dev_err(info->dev, "Voltage [%d:%d] is not existing in matching table\n", dc1, dc2);
		return -EINVAL;
	}

	info->cpu_vol = cpu_vol;

	return is_vdd ? cpu_vol->vdd : cpu_vol->vddm;
}

static int apcpu_get_vdd_voltage(struct regulator_dev *reg)
{
	return apcpu_get_voltage(reg, true);
}

static int apcpu_get_vddm_voltage(struct regulator_dev *reg)
{
	return apcpu_get_voltage(reg, false);
}

static const struct regulator_ops regu_common_ops = {
	.enable =        aon_regu_enable,
	.disable =       aon_regu_disable,
	.is_enabled =    aon_regu_is_enabled,
	.list_voltage =  regulator_list_voltage_linear,
	.set_voltage =   aon_regu_set_voltage,
	.get_voltage =   aon_regu_get_voltage,
};
static const struct regulator_ops apcpu_dvdd_ops = {
	.enable =        aon_regu_dummy_enable,
	.disable =       aon_regu_dummy_disable,
	.is_enabled =    aon_regu_dummy_is_enabled,
	.list_voltage =  regulator_list_voltage_linear,
	.set_voltage =   apcpu_set_vdd_vddm_voltage,
	.get_voltage =   apcpu_get_vdd_voltage,
};

static const struct regulator_ops apcpu_dvddm_ops = {
	.enable =        aon_regu_dummy_enable,
	.disable =       aon_regu_dummy_disable,
	.is_enabled =    aon_regu_dummy_is_enabled,
	.list_voltage =  regulator_list_voltage_linear,
	.set_voltage =   apcpu_set_vddm_voltage,
	.get_voltage =   apcpu_get_vddm_voltage,
};

/* Macros for voltage DC/DC converters (BUCKs) for cpu */
#define REGU_DSC_DEF(regu_id, of_math_name) \
	.id = regu_id, \
	.name = #regu_id, \
	.of_match = of_match_ptr(__stringify(of_math_name)), \
	.ops = &regu_common_ops, \
	.type = REGULATOR_VOLTAGE, \
	.owner = THIS_MODULE

#define BUCK_APCPU_DVDD(regu_id,min_mV, step_mV, max_mV) \
	.id = regu_id, \
	.name = "APCPU_DVDD", \
	.of_match = of_match_ptr("appcpu_dvdd"), \
	.ops = &apcpu_dvdd_ops, \
	.min_uV = (min_mV), \
	.uV_step = (step_mV), \
	.n_voltages = ((max_mV) - (min_mV))/(step_mV) + 1, \
	.type = REGULATOR_VOLTAGE, \
	.owner = THIS_MODULE

#define BUCK_APCPU_DVDDM(regu_id, min_mV, step_mV, max_mV) \
	.id = regu_id, \
	.name = "APCPU_DVDDM", \
	.of_match = of_match_ptr("appcpu_dvddm"), \
	.ops = &apcpu_dvddm_ops, \
	.min_uV = (min_mV) , \
	.uV_step = (step_mV), \
	.n_voltages = ((max_mV) - (min_mV))/(step_mV) + 1, \
	.type = REGULATOR_VOLTAGE, \
	.owner = THIS_MODULE

/* regulator desc for dialog */
static struct regulator_desc light_dialog_ant_regu_desc[] = {
	/*cpu vdd vddm regulators, used to adjust vol dynamicaly */
	{
		BUCK_APCPU_DVDD(SOC_APCPU_DVDD_DVDDM, 300000, 10000, 1570000),
	},
	{
		BUCK_APCPU_DVDDM(SOC_APCPU_DVDD_DVDDM, 300000, 10000, 1570000),
	},

	/*common regu ,no need to adjust vol dynamicaly */
	{
		REGU_DSC_DEF(SOC_DVDD18_AON,soc_dvdd18_aon),
	},
	{
		REGU_DSC_DEF(SOC_AVDD33_USB3,soc_avdd33_usb3),
	},
	{
		REGU_DSC_DEF(SOC_DVDD08_AON,soc_dvdd08_aon),
	},
	{
		REGU_DSC_DEF(SOC_DVDD08_DDR,soc_dvdd08_ddr),
	},
	{
		REGU_DSC_DEF(SOC_VDD_DDR_1V8,soc_vdd_ddr_1v8),
	},
	{
		REGU_DSC_DEF(SOC_VDD_DDR_1V1,soc_vdd_ddr_1v1),
	},
	{
		REGU_DSC_DEF(SOC_VDD_DDR_0V6,soc_vdd_ddr_0v6),
	},
	{
		REGU_DSC_DEF(SOC_DVDD18_AP,soc_dvdd18_ap),
	},
	{
		REGU_DSC_DEF(SOC_DVDD08_AP,soc_dvdd08_ap),
	},
	{
		REGU_DSC_DEF(SOC_AVDD08_MIPI_HDMI,soc_avdd08_mipi_hdmi),
	},
	{
		REGU_DSC_DEF(SOC_AVDD18_MIPI_HDMI,soc_avdd18_mipi_hdmi),
	},
	{
		REGU_DSC_DEF(SOC_DVDD33_EMMC,soc_dvdd33_emmc),
	},
	{
		REGU_DSC_DEF(SOC_DVDD18_EMMC,soc_dvdd18_emmc),
	},
	{
		REGU_DSC_DEF(SOC_DOVDD18_SCAN,soc_dovdd18_scan),
	},
	{
		REGU_DSC_DEF(SOC_DVDD12_SCAN,soc_dvdd12_scan),
	},
	{
		REGU_DSC_DEF(SOC_AVDD28_SCAN_EN,soc_avdd28_scan_en),
	},
};

static struct regulator_desc light_dialog_regu_desc[] = {
	/*cpu vdd vddm regulators, used to adjust vol dynamicaly */
	{
		BUCK_APCPU_DVDD(SOC_APCPU_DVDD_DVDDM, 300000, 10000, 1570000),
	},
	{
		BUCK_APCPU_DVDDM(SOC_APCPU_DVDD_DVDDM, 300000, 10000, 1570000),
	},

	/*common regu ,no need to adjust vol dynamicaly */
	{
		REGU_DSC_DEF(SOC_DVDD18_AON,soc_dvdd18_aon),
	},
	{
		REGU_DSC_DEF(SOC_AVDD33_USB3,soc_avdd33_usb3),
	},
	{
		REGU_DSC_DEF(SOC_DVDD08_AON,soc_dvdd08_aon),
	},
	{
		REGU_DSC_DEF(SOC_DVDD08_DDR,soc_dvdd08_ddr),
	},
	{
		REGU_DSC_DEF(SOC_VDD_DDR_1V8,soc_vdd_ddr_1v8),
	},
	{
		REGU_DSC_DEF(SOC_VDD_DDR_1V1,soc_vdd_ddr_1v1),
	},
	{
		REGU_DSC_DEF(SOC_VDD_DDR_0V6,soc_vdd_ddr_0v6),
	},
	{
		REGU_DSC_DEF(SOC_DVDD18_AP,soc_dvdd18_ap),
	},
	{
		REGU_DSC_DEF(SOC_DVDD08_AP,soc_dvdd08_ap),
	},
	{
		REGU_DSC_DEF(SOC_AVDD08_MIPI_HDMI,soc_avdd08_mipi_hdmi),
	},
	{
		REGU_DSC_DEF(SOC_AVDD18_MIPI_HDMI,soc_avdd18_mipi_hdmi),
	},
	{
		REGU_DSC_DEF(SOC_DVDD33_EMMC,soc_dvdd33_emmc),
	},
	{
		REGU_DSC_DEF(SOC_DVDD18_EMMC,soc_dvdd18_emmc),
	},
	{
		REGU_DSC_DEF(SOC_DOVDD18_SCAN,soc_dovdd18_scan),
	},
	{
		REGU_DSC_DEF(SOC_VEXT_2V8,soc_vext_2v8),
	},
	{
		REGU_DSC_DEF(SOC_DVDD12_SCAN,soc_dvdd12_scan),
	},
	{
		REGU_DSC_DEF(SOC_AVDD28_SCAN_EN,soc_avdd28_scan_en),
	},
	{
		REGU_DSC_DEF(SOC_AVDD28_RGB,soc_avdd28_rgb),
	},
	{
		REGU_DSC_DEF(SOC_DOVDD18_RGB,soc_dovdd18_rgb),
	},
	{
		REGU_DSC_DEF(SOC_DVDD12_RGB,soc_dvdd12_rgb),
	},
	{
		REGU_DSC_DEF(SOC_AVDD25_IR,soc_avdd25_ir),
	},
	{
		REGU_DSC_DEF(SOC_DOVDD18_IR,soc_dovdd18_ir),
	},
	{
		REGU_DSC_DEF(SOC_DVDD12_IR,soc_dvdd12_ir),
	},
};

/* regulator desc for ricoh */
static struct regulator_desc light_ricoh_regu_desc[] = {
    /*cpu vdd vddm regulators, used to adjust vol dynamicaly */
	{
		BUCK_APCPU_DVDD(SOC_APCPU_DVDD_DVDDM, 600000, 12500, 1500000),
	},
	{
		BUCK_APCPU_DVDDM(SOC_APCPU_DVDD_DVDDM, 600000, 12500, 1500000),
	},

	/*common regu ,no need to adjust vol dynamicaly */
	{
		REGU_DSC_DEF(SOC_DVDD18_AON,soc_dvdd18_aon),
	},
	{
		REGU_DSC_DEF(SOC_AVDD33_USB3,soc_avdd33_usb3),
	},
	{
		REGU_DSC_DEF(SOC_DVDD08_AON,soc_dvdd08_aon),
	},
	{
		REGU_DSC_DEF(SOC_DVDD08_DDR,soc_dvdd08_ddr),
	},
	{
		REGU_DSC_DEF(SOC_VDD_DDR_1V8,soc_vdd_ddr_1v8),
	},
	{
		REGU_DSC_DEF(SOC_VDD_DDR_1V1,soc_vdd_ddr_1v1),
	},
	{
		REGU_DSC_DEF(SOC_VDD_DDR_0V6,soc_vdd_ddr_0v6),
	},
	{
		REGU_DSC_DEF(SOC_DVDD18_AP,soc_dvdd18_ap),
	},
	{
		REGU_DSC_DEF(SOC_DVDD08_AP,soc_dvdd08_ap),
	},
	{
		REGU_DSC_DEF(SOC_AVDD08_MIPI_HDMI,soc_avdd08_mipi_hdmi),
	},
	{
		REGU_DSC_DEF(SOC_AVDD18_MIPI_HDMI,soc_avdd18_mipi_hdmi),
	},
	{
		REGU_DSC_DEF(SOC_DVDD33_EMMC,soc_dvdd33_emmc),
	},
	{
		REGU_DSC_DEF(SOC_DVDD18_EMMC,soc_dvdd18_emmc),
	},
	{
		REGU_DSC_DEF(SOC_LCD0_EN,soc_lcd0_en),
	},
	{
		REGU_DSC_DEF(SOC_VEXT_1V8,soc_vext_1v8),
	},
};

#define GEN_REGISTER_SHOW(x, y)	\
static ssize_t x##_registers_show(struct device *dev,				\
				     struct device_attribute *attr,		\
				     char *buf)					\
{										\
	struct platform_device *pdev = to_platform_device(dev);			\
	struct aon_regu_info *info = platform_get_drvdata(pdev);	\
	u32 dc1, dc2;								\
	ssize_t ret;								\
										\
	ret = aon_get_regulator(light_aon_pmic_info.ipc_handle, y,		\
				       &dc1, &dc2, 0);				\
	if (ret) {								\
		dev_err(info->dev, "failed to get Voltages!\n");		\
		return -EINVAL;							\
	}									\
										\
	ret = sprintf(buf, "%u\n", dc1);					\
	return ret;								\
}

#define GEN_REGISTER_STORE(x, y)	\
static ssize_t x##_register_store(struct device *dev,			\
				      struct device_attribute *attr,		\
				      const char *buf, size_t count)		\
{										\
	struct platform_device *pdev = to_platform_device(dev);			\
	struct aon_regu_info *info = platform_get_drvdata(pdev);	\
	unsigned long dc1, dc2 = 0;						\
	int err;								\
										\
	if (kstrtoul(buf, 0, &dc1))						\
		return -EINVAL;							\
										\
	err = aon_set_regulator(light_aon_pmic_info.ipc_handle, y,		\
				       dc1, dc2, 0);				\
	if (err) {								\
		dev_err(info->dev, "failed to set Voltages to [%lu]!\n", dc1);	\
		return -EINVAL;							\
	}									\
										\
	return count;								\
}

static ssize_t soc_apcpu_dvdd_dvddm_registers_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct aon_regu_info *info = platform_get_drvdata(pdev);
	size_t bufpos = 0, count = 26;
	const struct apcpu_vol_set *cpu_vol;
	u32 dc1, dc2;
	int i = 0;
	int err;
	err = aon_get_regulator(light_aon_pmic_info.ipc_handle, SOC_APCPU_DVDD_DVDDM,
				       &dc1, &dc2, 1);
	if (err) {
		dev_err(info->dev, "failed to get Voltages!\n");
		return -EINVAL;
	}
	cpu_vol = apcpu_get_matched_signed_off_voltage(dc1, dc2);
	if (!cpu_vol)
		dev_err(info->dev, "Read [%d:%d] is not existing in matching table\n", dc1, dc2);
	snprintf(buf + bufpos, count - bufpos, "%.*x: ", 2, i);
	bufpos += 4;
	snprintf(buf + bufpos, count - bufpos, "%u", dc1);
	bufpos += 8;
	buf[bufpos++] = '\n';
	i++;
	snprintf(buf + bufpos, count - bufpos, "%.*x: ", 2, i);
	bufpos += 4;
	snprintf(buf + bufpos, count - bufpos, "%u", dc2);
	bufpos += 8;
	buf[bufpos++] = '\n';
	return bufpos;
}
static ssize_t soc_apcpu_dvdd_dvddm_register_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct aon_regu_info *info = platform_get_drvdata(pdev);
	const struct apcpu_vol_set *cpu_vol;
	char *start = (char *)buf;
	unsigned long dc1, dc2;
	int err;
	while (*start == ' ')
		start++;
	dc1 = simple_strtoul(start, &start, 0);
	while (*start == ' ')
		start++;
	if (kstrtoul(start, 0, &dc2))
		return -EINVAL;
	cpu_vol = apcpu_get_matched_signed_off_voltage(dc1, dc2);
	if (!cpu_vol) {
		dev_err(info->dev, "failed to find bcore1/bcore2 matching table\n");
#ifndef CONFIG_AON_REG_DEBUG
		return -EINVAL;
#endif
	}
	info->cpu_vol = cpu_vol;
	info->vddm = cpu_vol->vddm;
	err = aon_set_regulator(light_aon_pmic_info.ipc_handle, SOC_APCPU_DVDD_DVDDM,
				       dc1, dc2, 1);
	if (err) {
		dev_err(info->dev, "failed to set Voltages to [%lu,%lu]!\n", dc1, dc2);
#ifndef CONFIG_AON_REG_DEBUG
		return -EINVAL;
#endif
	}
	return size;
}

GEN_REGISTER_SHOW(soc_dvdd18_aon, SOC_DVDD18_AON)
GEN_REGISTER_STORE(soc_dvdd18_aon, SOC_DVDD18_AON)
GEN_REGISTER_SHOW(soc_dvdd08_ap, SOC_DVDD08_AP)
GEN_REGISTER_STORE(soc_dvdd08_ap, SOC_DVDD08_AP)
GEN_REGISTER_SHOW(soc_dvdd18_emmc, SOC_DVDD18_EMMC)
GEN_REGISTER_STORE(soc_dvdd18_emmc, SOC_DVDD18_EMMC)
GEN_REGISTER_SHOW(soc_dvdd33_emmc, SOC_DVDD33_EMMC)
GEN_REGISTER_STORE(soc_dvdd33_emmc, SOC_DVDD33_EMMC)

static DEVICE_ATTR(soc_dvdd18_aon_regs, 0644, soc_dvdd18_aon_registers_show, soc_dvdd18_aon_register_store);
static DEVICE_ATTR(soc_dvdd08_ap_regs, 0644, soc_dvdd08_ap_registers_show, soc_dvdd08_ap_register_store);
static DEVICE_ATTR(soc_dvdd33_emmc_regs, 0644, soc_dvdd33_emmc_registers_show, soc_dvdd33_emmc_register_store);
static DEVICE_ATTR(soc_dvdd18_emmc_regs, 0644, soc_dvdd18_emmc_registers_show, soc_dvdd18_emmc_register_store);
static DEVICE_ATTR(soc_apcpu_dvdd_dvddm_regs, 0644, soc_apcpu_dvdd_dvddm_registers_show, soc_apcpu_dvdd_dvddm_register_store);

static struct attribute *aon_regs_sysfs_entries[] = {
	&dev_attr_soc_dvdd18_aon_regs.attr,
	&dev_attr_soc_dvdd08_ap_regs.attr,
	&dev_attr_soc_dvdd33_emmc_regs.attr,
	&dev_attr_soc_dvdd18_emmc_regs.attr,
	&dev_attr_soc_apcpu_dvdd_dvddm_regs.attr,
	NULL
};
static const struct attribute_group dev_attr_aon_regs_group = {
	.attrs = aon_regs_sysfs_entries,
};


static const struct aon_regu_desc light_dialog_regus = {
    .regu_desc = (struct regulator_desc*) &light_dialog_regu_desc,
    .regu_num  = ARRAY_SIZE(light_dialog_regu_desc),
};

static const struct aon_regu_desc light_dialog_ant_regus = {
    .regu_desc = (struct regulator_desc*) &light_dialog_ant_regu_desc,
    .regu_num  = ARRAY_SIZE(light_dialog_ant_regu_desc),
};

static const struct aon_regu_desc light_ricoh_regus = {
    .regu_desc = (struct regulator_desc*)&light_ricoh_regu_desc,
    .regu_num  = ARRAY_SIZE(light_ricoh_regu_desc),
};

static int light_aon_regulator_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct regulator_config config = { };
	int i;
	int ret;
	struct aon_regu_desc *regus_set = NULL;

	if (!np)
		return -ENODEV;

	regus_set = (struct aon_regu_desc*)of_device_get_match_data(&pdev->dev);
	if (!regus_set) {
		return -ENODEV;
	}

	/*get ipc handle */
	ret = light_aon_get_handle(&(light_aon_pmic_info.ipc_handle));
	if (ret) {
		dev_err(&pdev->dev, "failed to get ipc_handle\n");
		return ret;
	}

	/*init private drv data */
	light_aon_pmic_info.dev = &pdev->dev;
	light_aon_pmic_info.regu_desc = regus_set;
	light_aon_pmic_info.cpu_vol = &apcpu_volts[2]; /* pmic default voltages */
	light_aon_pmic_info.vddm = light_aon_pmic_info.cpu_vol->vddm;

	/*register all regulators*/
	config.dev = &pdev->dev;
	config.driver_data = &light_aon_pmic_info;
	for (i = 0; i < regus_set->regu_num; i++) {
		struct regulator_dev *rdev;
		struct regulator_desc *desc;

		desc = &regus_set->regu_desc[i];
		rdev = devm_regulator_register(&pdev->dev, desc, &config);
		if (IS_ERR(rdev)) {
			dev_err(&pdev->dev,
				"Failed to initialize regulator-%d\n", i);
			return PTR_ERR(rdev);
		}
	}

	i = sysfs_create_group(&config.dev->kobj, &dev_attr_aon_regs_group);
	if (i) {
		dev_err(&pdev->dev, "Failed to create aon regs debug sysfs.\n");
		return i;
	}

	platform_set_drvdata(pdev, &light_aon_pmic_info);

	return 0;
}

static const struct of_device_id light_pmic_dev_id[] = {
	{ .compatible = "thead,light-dialog-pmic-ant", .data = &light_dialog_ant_regus},
	{ .compatible = "thead,light-dialog-pmic", .data = &light_dialog_regus},
	{ .compatible = "thead,light-ricoh-pmic",  .data = &light_ricoh_regus},
	{},
};
MODULE_DEVICE_TABLE(of, light_pmic_dev_id);

static struct platform_driver light_aon_regulator_driver = {
	.driver = {
		   .name = "light-aon-reg",
		   .owner = THIS_MODULE,
		   .of_match_table = light_pmic_dev_id,
	},
	.probe = light_aon_regulator_probe,
};

module_platform_driver(light_aon_regulator_driver);

MODULE_AUTHOR("fugang.duan <duanfugang.dfg@linux.alibaba.com>");
MODULE_AUTHOR("linghui.zlh <linghui.zlh@linux.alibaba.com>");
MODULE_DESCRIPTION("Thead Light Aon regulator virtual driver");
MODULE_LICENSE("GPL v2");
