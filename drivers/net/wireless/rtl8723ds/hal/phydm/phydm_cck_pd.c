/******************************************************************************
 *
 * Copyright(c) 2007 - 2017  Realtek Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * The full GNU General Public License is included in this distribution in the
 * file called LICENSE.
 *
 * Contact Information:
 * wlanfae <wlanfae@realtek.com>
 * Realtek Corporation, No. 2, Innovation Road II, Hsinchu Science Park,
 * Hsinchu 300, Taiwan.
 *
 * Larry Finger <Larry.Finger@lwfinger.net>
 *
 *****************************************************************************/

/*@************************************************************
 * include files
 ************************************************************/

#include "mp_precomp.h"
#include "phydm_precomp.h"

#ifdef PHYDM_SUPPORT_CCKPD
#ifdef PHYDM_COMPILE_CCKPD_TYPE1
void phydm_write_cck_pd_type1(void *dm_void, u8 cca_th)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_cckpd_struct *cckpd_t = &dm->dm_cckpd_table;

	PHYDM_DBG(dm, DBG_CCKPD, "[%s] cck_cca_th=((0x%x))\n",
		  __func__, cca_th);

	odm_write_1byte(dm, R_0xa0a, cca_th);
	cckpd_t->cur_cck_cca_thres = cca_th;
}

void phydm_set_cckpd_lv_type1(void *dm_void, enum cckpd_lv lv)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_cckpd_struct *cckpd_t = &dm->dm_cckpd_table;
	u8 pd_th = 0;

	PHYDM_DBG(dm, DBG_CCKPD, "%s ======>\n", __func__);
	PHYDM_DBG(dm, DBG_CCKPD, "lv: (%d) -> (%d)\n", cckpd_t->cck_pd_lv, lv);

	if (cckpd_t->cck_pd_lv == lv) {
		PHYDM_DBG(dm, DBG_CCKPD, "stay in lv=%d\n", lv);
		return;
	}

	cckpd_t->cck_pd_lv = lv;
	cckpd_t->cck_fa_ma = CCK_FA_MA_RESET;

	if (lv == CCK_PD_LV_4)
		pd_th = 0xed;
	else if (lv == CCK_PD_LV_3)
		pd_th = 0xdd;
	else if (lv == CCK_PD_LV_2)
		pd_th = 0xcd;
	else if (lv == CCK_PD_LV_1)
		pd_th = 0x83;
	else if (lv == CCK_PD_LV_0)
		pd_th = 0x40;

	phydm_write_cck_pd_type1(dm, pd_th);
}

void phydm_cckpd_type1(void *dm_void)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_dig_struct *dig_t = &dm->dm_dig_table;
	struct phydm_cckpd_struct *cckpd_t = &dm->dm_cckpd_table;
	enum cckpd_lv lv = CCK_PD_LV_INIT;
	boolean is_update = true;

	if (dm->is_linked) {
	#if (DM_ODM_SUPPORT_TYPE & (ODM_WIN | ODM_CE))
		if (dm->rssi_min > 60) {
			lv = CCK_PD_LV_3;
		} else if (dm->rssi_min > 35) {
			lv = CCK_PD_LV_2;
		} else if (dm->rssi_min > 20) {
			if (cckpd_t->cck_fa_ma > 500)
				lv = CCK_PD_LV_2;
			else if (cckpd_t->cck_fa_ma < 250)
				lv = CCK_PD_LV_1;
			else
				is_update = false;
		} else { /*RSSI < 20*/
			lv = CCK_PD_LV_1;
		}
	#else /*ODM_AP*/
		if (dig_t->cur_ig_value > 0x32)
			lv = CCK_PD_LV_4;
		else if (dig_t->cur_ig_value > 0x2a)
			lv = CCK_PD_LV_3;
		else if (dig_t->cur_ig_value > 0x24)
			lv = CCK_PD_LV_2;
		else
			lv = CCK_PD_LV_1;
	#endif
	} else {
		if (cckpd_t->cck_fa_ma > 1000)
			lv = CCK_PD_LV_1;
		else if (cckpd_t->cck_fa_ma < 500)
			lv = CCK_PD_LV_0;
		else
			is_update = false;
	}

	/*[Abnormal case] =================================================*/
	#if (DM_ODM_SUPPORT_TYPE & ODM_WIN)
	/*@HP 22B LPS power consumption issue & [PCIE-1596]*/
	if (dm->hp_hw_id && dm->traffic_load == TRAFFIC_ULTRA_LOW) {
		lv = CCK_PD_LV_0;
		PHYDM_DBG(dm, DBG_CCKPD, "CCKPD Abnormal case1\n");
	} else if ((dm->p_advance_ota & PHYDM_ASUS_OTA_SETTING) &&
	    cckpd_t->cck_fa_ma > 200 && dm->rssi_min <= 20) {
		lv = CCK_PD_LV_1;
		cckpd_t->cck_pd_lv = lv;
		phydm_write_cck_pd_type1(dm, 0xc3); /*@for ASUS OTA test*/
		is_update = false;
		PHYDM_DBG(dm, DBG_CCKPD, "CCKPD Abnormal case2\n");
	}
	#elif (DM_ODM_SUPPORT_TYPE & (ODM_AP))
		#ifdef MCR_WIRELESS_EXTEND
		lv = CCK_PD_LV_2;
		cckpd_t->cck_pd_lv = lv;
		phydm_write_cck_pd_type1(dm, 0x43);
		is_update = false;
		PHYDM_DBG(dm, DBG_CCKPD, "CCKPD Abnormal case3\n");
		#endif
	#endif
	/*=================================================================*/

	if (is_update)
		phydm_set_cckpd_lv_type1(dm, lv);

	PHYDM_DBG(dm, DBG_CCKPD, "is_linked=%d, lv=%d, pd_th=0x%x\n\n",
		  dm->is_linked, cckpd_t->cck_pd_lv,
		  cckpd_t->cur_cck_cca_thres);
}
#endif /*#ifdef PHYDM_COMPILE_CCKPD_TYPE1*/

#ifdef PHYDM_COMPILE_CCKPD_TYPE2
void phydm_write_cck_pd_type2(void *dm_void, u8 cca_th, u8 cca_th_aaa)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_cckpd_struct *cckpd_t = &dm->dm_cckpd_table;

	PHYDM_DBG(dm, DBG_CCKPD, "[%s] pd_th=0x%x, cs_ratio=0x%x\n",
		  __func__, cca_th, cca_th_aaa);

	odm_set_bb_reg(dm, R_0xa08, 0x3f0000, cca_th);
	odm_set_bb_reg(dm, R_0xaa8, 0x1f0000, cca_th_aaa);
	cckpd_t->cur_cck_cca_thres = cca_th;
	cckpd_t->cck_cca_th_aaa = cca_th_aaa;
}

void phydm_set_cckpd_lv_type2(void *dm_void, enum cckpd_lv lv)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_cckpd_struct *cckpd_t = &dm->dm_cckpd_table;
	u8 pd_th = 0, cs_ratio = 0, cs_2r_offset = 0;
	u8 cck_n_rx = 1;

	PHYDM_DBG(dm, DBG_CCKPD, "%s ======>\n", __func__);
	PHYDM_DBG(dm, DBG_CCKPD, "lv: (%d) -> (%d)\n", cckpd_t->cck_pd_lv, lv);

	/*@r_mrx & r_cca_mrc*/
	cck_n_rx = (odm_get_bb_reg(dm, R_0xa2c, BIT(18)) &&
		    odm_get_bb_reg(dm, R_0xa2c, BIT(22))) ? 2 : 1;

	if (cckpd_t->cck_pd_lv == lv && cckpd_t->cck_n_rx == cck_n_rx) {
		PHYDM_DBG(dm, DBG_CCKPD, "stay in lv=%d\n", lv);
		return;
	}

	cckpd_t->cck_n_rx = cck_n_rx;
	cckpd_t->cck_pd_lv = lv;
	cckpd_t->cck_fa_ma = CCK_FA_MA_RESET;

	if (lv == CCK_PD_LV_4) {
		cs_ratio = cckpd_t->aaa_default + 8;
		cs_2r_offset = 5;
		pd_th = 0xd;
	} else if (lv == CCK_PD_LV_3) {
		cs_ratio = cckpd_t->aaa_default + 6;
		cs_2r_offset = 4;
		pd_th = 0xd;
	} else if (lv == CCK_PD_LV_2) {
		cs_ratio = cckpd_t->aaa_default + 4;
		cs_2r_offset = 3;
		pd_th = 0xd;
	} else if (lv == CCK_PD_LV_1) {
		cs_ratio = cckpd_t->aaa_default + 2;
		cs_2r_offset = 1;
		pd_th = 0x7;
	} else if (lv == CCK_PD_LV_0) {
		cs_ratio = cckpd_t->aaa_default;
		cs_2r_offset = 0;
		pd_th = 0x3;
	}

	if (cckpd_t->cck_n_rx == 2) {
		if (cs_ratio >= cs_2r_offset)
			cs_ratio = cs_ratio - cs_2r_offset;
		else
			cs_ratio = 0;
	}
	phydm_write_cck_pd_type2(dm, pd_th, cs_ratio);
}

#if 0
void phydm_set_cckpd_lv_type2_bcn(void *dm_void, enum cckpd_lv lv)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_cckpd_struct *cckpd_t = &dm->dm_cckpd_table;
	u8 pd_th = 0, cs_ratio = 0, cs_2r_offset = 0;
	u8 cck_n_rx = 1;
	u8 cs_ratio_pre = 0;
	u8 bcn_cnt = dm->phy_dbg_info.beacon_cnt_in_period; //BCN CNT
	u8 ofst = 0;
	u8 ofst_direc = 0; //0:+, 1:-

	PHYDM_DBG(dm, DBG_CCKPD, "%s ======>\n", __func__);
	PHYDM_DBG(dm, DBG_CCKPD, "lv: (%d) -> (%d)\n", cckpd_t->cck_pd_lv, lv);

	/*@r_mrx & r_cca_mrc*/
	cck_n_rx = (odm_get_bb_reg(dm, R_0xa2c, BIT(18)) &&
		    odm_get_bb_reg(dm, R_0xa2c, BIT(22))) ? 2 : 1;
	cs_ratio_pre = (u8)((odm_get_bb_reg(dm, R_0xaa8, 0x1f0000)));
	PHYDM_DBG(dm, DBG_CCKPD, "BCN: %d, pre CS ratio: 0x%x\n", bcn_cnt,
		  cs_ratio_pre);

	if (cckpd_t->cck_pd_lv == lv && cckpd_t->cck_n_rx == cck_n_rx &&
	    (bcn_cnt >= 10 && bcn_cnt < 14)) {
		PHYDM_DBG(dm, DBG_CCKPD, "BCN ok, stay lv=%d, cs ratio=0x%x\n",
			  lv, cs_ratio_pre);
		return;
	}

	cckpd_t->cck_n_rx = cck_n_rx;
	cckpd_t->cck_pd_lv = lv;
	cckpd_t->cck_fa_ma = CCK_FA_MA_RESET;

	if (lv == CCK_PD_LV_4) {
		cs_ratio = cckpd_t->aaa_default + 8;
		cs_2r_offset = 5;
		pd_th = 0xd;
	} else if (lv == CCK_PD_LV_3) {
		cs_ratio = cckpd_t->aaa_default + 6;
		cs_2r_offset = 4;
		pd_th = 0xd;
	} else if (lv == CCK_PD_LV_2) {
		cs_ratio = cckpd_t->aaa_default + 4;
		cs_2r_offset = 3;
		pd_th = 0xd;
	} else if (lv == CCK_PD_LV_1) {
		cs_ratio = cckpd_t->aaa_default + 2;
		cs_2r_offset = 1;
		pd_th = 0x7;
	} else if (lv == CCK_PD_LV_0) {
		cs_ratio = cckpd_t->aaa_default;
		cs_2r_offset = 0;
		pd_th = 0x3;
	}

	if (cckpd_t->cck_n_rx == 2) {
		if (cs_ratio >= cs_2r_offset)
			cs_ratio = cs_ratio - cs_2r_offset;
		else
			cs_ratio = 0;
	}

	if (bcn_cnt >= 18) {
		ofst_direc = 0;
		ofst = 0x2;
	} else if (bcn_cnt >= 14) {
		ofst_direc = 0;
		ofst = 0x1;
	} else if (bcn_cnt >= 10) {
		ofst_direc = 0;
		ofst = 0x0;
	} else if (bcn_cnt >= 5) {
		ofst_direc = 1;
		ofst = 0x3;
	} else {
		ofst_direc = 1;
		ofst = 0x4;
	}
	PHYDM_DBG(dm, DBG_CCKPD, "bcn:(%d), ofst:(%s%d)\n", bcn_cnt,
		  ((ofst_direc) ? "-" : "+"), ofst);

	if (ofst_direc == 0)
		cs_ratio = cs_ratio + ofst;
	else
		cs_ratio = cs_ratio - ofst;

	if (cs_ratio == cs_ratio_pre) {
		PHYDM_DBG(dm, DBG_CCKPD, "Same cs ratio, lv=%d cs_ratio=0x%x\n",
			  lv, cs_ratio);
		return;
	}
	phydm_write_cck_pd_type2(dm, pd_th, cs_ratio);
}
#endif

void phydm_cckpd_type2(void *dm_void)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_dig_struct *dig_t = &dm->dm_dig_table;
	struct phydm_cckpd_struct *cckpd_t = &dm->dm_cckpd_table;
	enum cckpd_lv lv = CCK_PD_LV_INIT;
	u8 igi = dig_t->cur_ig_value;
	u8 rssi_min = dm->rssi_min;
	boolean is_update = true;

	PHYDM_DBG(dm, DBG_CCKPD, "%s ======>\n", __func__);

	if (dm->is_linked) {
		if (igi > 0x38 && rssi_min > 32) {
			lv = CCK_PD_LV_4;
		} else if (igi > 0x2a && rssi_min > 32) {
			lv = CCK_PD_LV_3;
		} else if (igi > 0x24 || (rssi_min > 24 && rssi_min <= 30)) {
			lv = CCK_PD_LV_2;
		} else if (igi <= 0x24 || rssi_min < 22) {
			if (cckpd_t->cck_fa_ma > 1000) {
				lv = CCK_PD_LV_1;
			} else if (cckpd_t->cck_fa_ma < 500) {
				lv = CCK_PD_LV_0;
			} else {
				is_update = false;
			}
		} else {
			is_update = false;
		}
	} else {
		if (cckpd_t->cck_fa_ma > 1000) {
			lv = CCK_PD_LV_1;
		} else if (cckpd_t->cck_fa_ma < 500) {
			lv = CCK_PD_LV_0;
		} else {
			is_update = false;
		}
	}

	/*[Abnormal case] =================================================*/
	#if (DM_ODM_SUPPORT_TYPE & ODM_WIN)
	/*@21C Miracast lag issue & [PCIE-3298]*/
	if (dm->support_ic_type & ODM_RTL8821C && rssi_min > 60) {
		lv = CCK_PD_LV_4;
		cckpd_t->cck_pd_lv = lv;
		phydm_write_cck_pd_type2(dm, 0x1d, (cckpd_t->aaa_default + 8));
		is_update = false;
		PHYDM_DBG(dm, DBG_CCKPD, "CCKPD Abnormal case1\n");
	}
	#endif
	/*=================================================================*/

	if (is_update) {
		phydm_set_cckpd_lv_type2(dm, lv);
	}

	PHYDM_DBG(dm, DBG_CCKPD,
		  "is_linked=%d, lv=%d, n_rx=%d, cs_ratio=0x%x, pd_th=0x%x\n\n",
		  dm->is_linked, cckpd_t->cck_pd_lv, cckpd_t->cck_n_rx,
		  cckpd_t->cck_cca_th_aaa, cckpd_t->cur_cck_cca_thres);
}
#endif /*#ifdef PHYDM_COMPILE_CCKPD_TYPE2*/

#ifdef PHYDM_COMPILE_CCKPD_TYPE3
void phydm_write_cck_pd_type3(void *dm_void, u8 pd_th, u8 cs_ratio,
			      enum cckpd_mode mode)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_cckpd_struct *cckpd_t = &dm->dm_cckpd_table;

	PHYDM_DBG(dm, DBG_CCKPD,
		  "[%s] mode=%d, pd_th=0x%x, cs_ratio=0x%x\n", __func__,
		  mode, pd_th, cs_ratio);

	switch (mode) {
	case CCK_BW20_1R: /*RFBW20_1R*/
	{
		cckpd_t->cur_cck_pd_20m_1r = pd_th;
		cckpd_t->cur_cck_cs_ratio_20m_1r = cs_ratio;
		odm_set_bb_reg(dm, R_0xac8, 0xff, pd_th);
		odm_set_bb_reg(dm, R_0xad0, 0x1f, cs_ratio);
	} break;
	case CCK_BW20_2R: /*RFBW20_2R*/
	{
		cckpd_t->cur_cck_pd_20m_2r = pd_th;
		cckpd_t->cur_cck_cs_ratio_20m_2r = cs_ratio;
		odm_set_bb_reg(dm, R_0xac8, 0xff00, pd_th);
		odm_set_bb_reg(dm, R_0xad0, 0x3e0, cs_ratio);
	} break;
	case CCK_BW40_1R: /*RFBW40_1R*/
	{
		cckpd_t->cur_cck_pd_40m_1r = pd_th;
		cckpd_t->cur_cck_cs_ratio_40m_1r = cs_ratio;
		odm_set_bb_reg(dm, R_0xacc, 0xff, pd_th);
		odm_set_bb_reg(dm, R_0xad0, 0x1f00000, cs_ratio);
	} break;
	case CCK_BW40_2R: /*RFBW40_2R*/
	{
		cckpd_t->cur_cck_pd_40m_2r = pd_th;
		cckpd_t->cur_cck_cs_ratio_40m_2r = cs_ratio;
		odm_set_bb_reg(dm, R_0xacc, 0xff00, pd_th);
		odm_set_bb_reg(dm, R_0xad0, 0x3e000000, cs_ratio);
	} break;

	default:
		/*@pr_debug("[%s] warning!\n", __func__);*/
		break;
	}
}

void phydm_set_cckpd_lv_type3(void *dm_void, enum cckpd_lv lv)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_cckpd_struct *cckpd_t = &dm->dm_cckpd_table;
	enum cckpd_mode cck_mode = CCK_BW20_2R;
	enum channel_width cck_bw = CHANNEL_WIDTH_20;
	u8 cck_n_rx = 1;
	u8 pd_th;
	u8 cs_ratio;

	PHYDM_DBG(dm, DBG_CCKPD, "%s ======>\n", __func__);
	PHYDM_DBG(dm, DBG_CCKPD, "lv: (%d) -> (%d)\n", cckpd_t->cck_pd_lv, lv);

	/*[Check Nrx]*/
	cck_n_rx = (odm_get_bb_reg(dm, R_0xa2c, BIT(17))) ? 2 : 1;

	/*[Check BW]*/
	if (odm_get_bb_reg(dm, R_0x800, BIT(0)))
		cck_bw = CHANNEL_WIDTH_40;
	else
		cck_bw = CHANNEL_WIDTH_20;

	/*[Check LV]*/
	if (cckpd_t->cck_pd_lv == lv &&
	    cckpd_t->cck_n_rx == cck_n_rx &&
	    cckpd_t->cck_bw == cck_bw) {
		PHYDM_DBG(dm, DBG_CCKPD, "stay in lv=%d\n", lv);
		return;
	}

	cckpd_t->cck_bw = cck_bw;
	cckpd_t->cck_n_rx = cck_n_rx;
	cckpd_t->cck_pd_lv = lv;
	cckpd_t->cck_fa_ma = CCK_FA_MA_RESET;

	if (cck_n_rx == 2) {
		if (cck_bw == CHANNEL_WIDTH_20) {
			pd_th = cckpd_t->cck_pd_20m_2r;
			cs_ratio = cckpd_t->cck_cs_ratio_20m_2r;
			cck_mode = CCK_BW20_2R;
		} else {
			pd_th = cckpd_t->cck_pd_40m_2r;
			cs_ratio = cckpd_t->cck_cs_ratio_40m_2r;
			cck_mode = CCK_BW40_2R;
		}
	} else {
		if (cck_bw == CHANNEL_WIDTH_20) {
			pd_th = cckpd_t->cck_pd_20m_1r;
			cs_ratio = cckpd_t->cck_cs_ratio_20m_1r;
			cck_mode = CCK_BW20_1R;
		} else {
			pd_th = cckpd_t->cck_pd_40m_1r;
			cs_ratio = cckpd_t->cck_cs_ratio_40m_1r;
			cck_mode = CCK_BW40_1R;
		}
	}

	if (lv == CCK_PD_LV_4) {
		if (cck_n_rx == 2) {
			pd_th += 4;
			cs_ratio += 2;
		} else {
			pd_th += 4;
			cs_ratio += 3;
		}
	} else if (lv == CCK_PD_LV_3) {
		if (cck_n_rx == 2) {
			pd_th += 3;
			cs_ratio += 1;
		} else {
			pd_th += 3;
			cs_ratio += 2;
		}
	} else if (lv == CCK_PD_LV_2) {
		pd_th += 2;
		cs_ratio += 1;
	} else if (lv == CCK_PD_LV_1) {
		pd_th += 1;
		cs_ratio += 1;
	}

	phydm_write_cck_pd_type3(dm, pd_th, cs_ratio, cck_mode);
}

void phydm_cckpd_type3(void *dm_void)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_cckpd_struct *cckpd_t = &dm->dm_cckpd_table;
	enum cckpd_lv lv = CCK_PD_LV_INIT;
	u8 igi = dm->dm_dig_table.cur_ig_value;
	boolean is_update = true;
	u8 pd_th = 0;
	u8 cs_ratio = 0;

	PHYDM_DBG(dm, DBG_CCKPD, "%s ======>\n", __func__);

	if (dm->is_linked) {
		if (igi > 0x38 && dm->rssi_min > 32) {
			lv = CCK_PD_LV_4;
		} else if ((igi > 0x2a) && (dm->rssi_min > 32)) {
			lv = CCK_PD_LV_3;
		} else if ((igi > 0x24) ||
			   (dm->rssi_min > 24 && dm->rssi_min <= 30)) {
			lv = CCK_PD_LV_2;
		} else if ((igi <= 0x24) || (dm->rssi_min < 22)) {
			if (cckpd_t->cck_fa_ma > 1000)
				lv = CCK_PD_LV_1;
			else if (cckpd_t->cck_fa_ma < 500)
				lv = CCK_PD_LV_0;
			else
				is_update = false;
		}
        if (igi >= 0x20 && dm->rssi_min >= 27) {
			//printf(">>>>>TUYA CCK FA CNT = %d, RSSI = %d, IGI =%d \n", cckpd_t->cck_fa_ma, dm->rssi_min, igi);
			is_update = false;
			odm_set_bb_reg(dm, R_0xa08, BIT(21) | BIT(20), 0x2);
			//odm_set_bb_reg(dm, R_0xac8, 0xff, 0x18);
		}
		else {
			//printf("CCK FA CNT = %d, RSSI = %d, IGI =%d \n", cckpd_t->cck_fa_ma, dm->rssi_min, igi);
			odm_set_bb_reg(dm, R_0xa08, BIT(21) | BIT(20), cckpd_t->cck_din_shift_opt);
			//odm_set_bb_reg(dm, R_0xac8, 0xff, cckpd_t->cck_pd_20m_1r);
		}
	} else {
		if (cckpd_t->cck_fa_ma > 1000)
			lv = CCK_PD_LV_1;
		else if (cckpd_t->cck_fa_ma < 500)
			lv = CCK_PD_LV_0;
		else
			is_update = false;
	}

	if (is_update)
		phydm_set_cckpd_lv_type3(dm, lv);

	if (cckpd_t->cck_n_rx == 2) {
		if (cckpd_t->cck_bw == CHANNEL_WIDTH_20) {
			pd_th = cckpd_t->cur_cck_pd_20m_2r;
			cs_ratio = cckpd_t->cur_cck_cs_ratio_20m_2r;
		} else {
			pd_th = cckpd_t->cur_cck_pd_40m_2r;
			cs_ratio = cckpd_t->cur_cck_cs_ratio_40m_2r;
		}
	} else {
		if (cckpd_t->cck_bw == CHANNEL_WIDTH_20) {
			pd_th = cckpd_t->cur_cck_pd_20m_1r;
			cs_ratio = cckpd_t->cur_cck_cs_ratio_20m_1r;
		} else {
			pd_th = cckpd_t->cur_cck_pd_40m_1r;
			cs_ratio = cckpd_t->cur_cck_cs_ratio_40m_1r;
		}
	}
	PHYDM_DBG(dm, DBG_CCKPD,
		  "[%dR][%dM] is_linked=%d, lv=%d, cs_ratio=0x%x, pd_th=0x%x\n\n",
		  cckpd_t->cck_n_rx, 20 << cckpd_t->cck_bw, dm->is_linked,
		  cckpd_t->cck_pd_lv, cs_ratio, pd_th);
}

void phydm_cck_pd_init_type3(void *dm_void)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_cckpd_struct *cckpd_t = &dm->dm_cckpd_table;
	u32 reg_tmp = 0;

	/*Get Default value*/
	cckpd_t->cck_pd_20m_1r = (u8)odm_get_bb_reg(dm, R_0xac8, 0xff);
	cckpd_t->cck_pd_20m_2r = (u8)odm_get_bb_reg(dm, R_0xac8, 0xff00);
	cckpd_t->cck_pd_40m_1r = (u8)odm_get_bb_reg(dm, R_0xacc, 0xff);
	cckpd_t->cck_pd_40m_2r = (u8)odm_get_bb_reg(dm, R_0xacc, 0xff00);
    cckpd_t->cck_din_shift_opt = (u8)odm_get_bb_reg(dm, R_0xa08, BIT(21) | BIT(20));

	reg_tmp = odm_get_bb_reg(dm, R_0xad0, MASKDWORD);
	cckpd_t->cck_cs_ratio_20m_1r = (u8)(reg_tmp & 0x1f);
	cckpd_t->cck_cs_ratio_20m_2r = (u8)((reg_tmp & 0x3e0) >> 5);
	cckpd_t->cck_cs_ratio_40m_1r = (u8)((reg_tmp & 0x1f00000) >> 20);
	cckpd_t->cck_cs_ratio_40m_2r = (u8)((reg_tmp & 0x3e000000) >> 25);
}
#endif /*#ifdef PHYDM_COMPILE_CCKPD_TYPE3*/

#ifdef PHYDM_COMPILE_CCKPD_TYPE4
void phydm_write_cck_pd_type4(void *dm_void, enum cckpd_lv lv,
			      enum cckpd_mode mode)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_cckpd_struct *cckpd_t = &dm->dm_cckpd_table;
	u32 val = 0;

	PHYDM_DBG(dm, DBG_CCKPD, "write CCK CCA parameters(CS_ratio & PD)\n");
	switch (mode) {
	case CCK_BW20_1R: /*RFBW20_1R*/
	{
		val = cckpd_t->cckpd_jgr3[0][0][0][lv];
		odm_set_bb_reg(dm, R_0x1ac8, 0xff, val);
		val = cckpd_t->cckpd_jgr3[0][0][1][lv];
		odm_set_bb_reg(dm, R_0x1ad0, 0x1f, val);
	} break;
	case CCK_BW40_1R: /*RFBW40_1R*/
	{
		val = cckpd_t->cckpd_jgr3[1][0][0][lv];
		odm_set_bb_reg(dm, R_0x1acc, 0xff, val);
		val = cckpd_t->cckpd_jgr3[1][0][1][lv];
		odm_set_bb_reg(dm, R_0x1ad0, 0x01F00000, val);
	} break;
	#if (defined(PHYDM_COMPILE_ABOVE_2SS))
	case CCK_BW20_2R: /*RFBW20_2R*/
	{
		val = cckpd_t->cckpd_jgr3[0][1][0][lv];
		odm_set_bb_reg(dm, R_0x1ac8, 0xff00, val);
		val = cckpd_t->cckpd_jgr3[0][1][1][lv];
		odm_set_bb_reg(dm, R_0x1ad0, 0x3e0, val);
	} break;
	case CCK_BW40_2R: /*RFBW40_2R*/
	{
		val = cckpd_t->cckpd_jgr3[1][1][0][lv];
		odm_set_bb_reg(dm, R_0x1acc, 0xff00, val);
		val = cckpd_t->cckpd_jgr3[1][1][1][lv];
		odm_set_bb_reg(dm, R_0x1ad0, 0x3E000000, val);
	} break;
	#endif
	#if (defined(PHYDM_COMPILE_ABOVE_3SS))
	case CCK_BW20_3R: /*RFBW20_3R*/
	{
		val = cckpd_t->cckpd_jgr3[0][2][0][lv];
		odm_set_bb_reg(dm, R_0x1ac8, 0xff0000, val);
		val = cckpd_t->cckpd_jgr3[0][2][1][lv];
		odm_set_bb_reg(dm, R_0x1ad0, 0x7c00, val);
	} break;
	case CCK_BW40_3R: /*RFBW40_3R*/
	{
		val = cckpd_t->cckpd_jgr3[1][2][0][lv];
		odm_set_bb_reg(dm, R_0x1acc, 0xff0000, val);
		val = cckpd_t->cckpd_jgr3[1][2][1][lv] & 0x3;
		odm_set_bb_reg(dm, R_0x1ad0, 0xC0000000, val);
		val = (cckpd_t->cckpd_jgr3[1][2][1][lv] & 0x1c) >> 2;
		odm_set_bb_reg(dm, R_0x1ad4, 0x7, val);
	} break;
	#endif
	#if (defined(PHYDM_COMPILE_ABOVE_4SS))
	case CCK_BW20_4R: /*RFBW20_4R*/
	{
		val = cckpd_t->cckpd_jgr3[0][3][0][lv];
		odm_set_bb_reg(dm, R_0x1ac8, 0xff000000, val);
		val = cckpd_t->cckpd_jgr3[0][3][1][lv];
		odm_set_bb_reg(dm, R_0x1ad0, 0xF8000, val);
	} break;
	case CCK_BW40_4R: /*RFBW40_4R*/
	{
		val = cckpd_t->cckpd_jgr3[1][3][0][lv];
		odm_set_bb_reg(dm, R_0x1acc, 0xff000000, val);
		val = cckpd_t->cckpd_jgr3[1][3][1][lv];
		odm_set_bb_reg(dm, R_0x1ad4, 0xf8, val);
	} break;
	#endif
	default:
		/*@pr_debug("[%s] warning!\n", __func__);*/
		break;
	}
}

void phydm_set_cck_pd_lv_type4(void *dm_void, enum cckpd_lv lv)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_cckpd_struct *cckpd_t = &dm->dm_cckpd_table;
	enum cckpd_mode cck_mode = CCK_BW20_2R;
	enum channel_width cck_bw = CHANNEL_WIDTH_20;
	u8 cck_n_rx = 0;
	u32 val = 0;
	/*u32 val_dbg = 0;*/

	PHYDM_DBG(dm, DBG_CCKPD, "%s ======>\n", __func__);
	PHYDM_DBG(dm, DBG_CCKPD, "lv: (%d) -> (%d)\n", cckpd_t->cck_pd_lv, lv);

	/*[Check Nrx]*/
	cck_n_rx = (u8)odm_get_bb_reg(dm, R_0x1a2c, 0x60000) + 1;

	/*[Check BW]*/
	val = odm_get_bb_reg(dm, R_0x9b0, 0xc);
	if (val == 0)
		cck_bw = CHANNEL_WIDTH_20;
	else if (val == 1)
		cck_bw = CHANNEL_WIDTH_40;
	else
		cck_bw = CHANNEL_WIDTH_80;

	/*[Check LV]*/
	if (cckpd_t->cck_pd_lv == lv &&
	    cckpd_t->cck_n_rx == cck_n_rx &&
	    cckpd_t->cck_bw == cck_bw) {
		PHYDM_DBG(dm, DBG_CCKPD, "stay in lv=%d\n", lv);
		return;
	}

	cckpd_t->cck_bw = cck_bw;
	cckpd_t->cck_n_rx = cck_n_rx;
	cckpd_t->cck_pd_lv = lv;
	cckpd_t->cck_fa_ma = CCK_FA_MA_RESET;

	switch (cck_n_rx) {
	case 1: /*1R*/
	{
		if (cck_bw == CHANNEL_WIDTH_20)
			cck_mode = CCK_BW20_1R;
		else if (cck_bw == CHANNEL_WIDTH_40)
			cck_mode = CCK_BW40_1R;
	} break;
	#if (defined(PHYDM_COMPILE_ABOVE_2SS))
	case 2: /*2R*/
	{
		if (cck_bw == CHANNEL_WIDTH_20)
			cck_mode = CCK_BW20_2R;
		else if (cck_bw == CHANNEL_WIDTH_40)
			cck_mode = CCK_BW40_2R;
	} break;
	#endif
	#if (defined(PHYDM_COMPILE_ABOVE_3SS))
	case 3: /*3R*/
	{
		if (cck_bw == CHANNEL_WIDTH_20)
			cck_mode = CCK_BW20_3R;
		else if (cck_bw == CHANNEL_WIDTH_40)
			cck_mode = CCK_BW40_3R;
	} break;
	#endif
	#if (defined(PHYDM_COMPILE_ABOVE_4SS))
	case 4: /*4R*/
	{
		if (cck_bw == CHANNEL_WIDTH_20)
			cck_mode = CCK_BW20_4R;
		else if (cck_bw == CHANNEL_WIDTH_40)
			cck_mode = CCK_BW40_4R;
	} break;
	#endif
	default:
		/*@pr_debug("[%s] warning!\n", __func__);*/
		break;
	}
	phydm_write_cck_pd_type4(dm, lv, cck_mode);
}

void phydm_read_cckpd_para_type4(void *dm_void)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_cckpd_struct *cckpd_t = &dm->dm_cckpd_table;
	u8 bw = 0; /*r_RX_RF_BW*/
	u8 n_rx = 0;
	u8 curr_cck_pd_t[2][4][2];
	u32 reg0 = 0;
	u32 reg1 = 0;
	u32 reg2 = 0;
	u32 reg3 = 0;

	if (!(dm->debug_components & DBG_CCKPD))
		return;

	bw = (u8)odm_get_bb_reg(dm, R_0x9b0, 0xc);
	n_rx = (u8)odm_get_bb_reg(dm, R_0x1a2c, 0x60000) + 1;

	reg0 = odm_get_bb_reg(dm, R_0x1ac8, MASKDWORD);
	reg1 = odm_get_bb_reg(dm, R_0x1acc, MASKDWORD);
	reg2 = odm_get_bb_reg(dm, R_0x1ad0, MASKDWORD);
	reg3 = odm_get_bb_reg(dm, R_0x1ad4, MASKDWORD);
	curr_cck_pd_t[0][0][0] = (u8)(reg0 & 0x000000ff);
	curr_cck_pd_t[1][0][0] = (u8)(reg1 & 0x000000ff);
	curr_cck_pd_t[0][0][1] = (u8)(reg2 & 0x0000001f);
	curr_cck_pd_t[1][0][1] = (u8)((reg2 & 0x01f00000) >> 20);
	#if (defined(PHYDM_COMPILE_ABOVE_2SS))
	if (dm->support_ic_type & PHYDM_IC_ABOVE_2SS) {
		curr_cck_pd_t[0][1][0] = (u8)((reg0 & 0x0000ff00) >> 8);
		curr_cck_pd_t[1][1][0] = (u8)((reg1 & 0x0000ff00) >> 8);
		curr_cck_pd_t[0][1][1] = (u8)((reg2 & 0x000003E0) >> 5);
		curr_cck_pd_t[1][1][1] = (u8)((reg2 & 0x3E000000) >> 25);
	}
	#endif
	#if (defined(PHYDM_COMPILE_ABOVE_3SS))
	if (dm->support_ic_type & PHYDM_IC_ABOVE_3SS) {
		curr_cck_pd_t[0][2][0] = (u8)((reg0 & 0x00ff0000) >> 16);
		curr_cck_pd_t[1][2][0] = (u8)((reg1 & 0x00ff0000) >> 16);
		curr_cck_pd_t[0][2][1] = (u8)((reg2 & 0x00007C00) >> 10);
		curr_cck_pd_t[1][2][1] = (u8)((reg2 & 0xC0000000) >> 30) |
					 (u8)((reg3 & 0x00000007) << 2);
	}
	#endif
	#if (defined(PHYDM_COMPILE_ABOVE_4SS))
	if (dm->support_ic_type & PHYDM_IC_ABOVE_4SS) {
		curr_cck_pd_t[0][3][0] = (u8)((reg0 & 0xff000000) >> 24);
		curr_cck_pd_t[1][3][0] = (u8)((reg1 & 0xff000000) >> 24);
		curr_cck_pd_t[0][3][1] = (u8)((reg2 & 0x000F8000) >> 15);
		curr_cck_pd_t[1][3][1] = (u8)((reg3 & 0x000000F8) >> 3);
	}
	#endif

	PHYDM_DBG(dm, DBG_CCKPD, "bw=%dM, Nrx=%d\n", 20 << bw, n_rx);
	PHYDM_DBG(dm, DBG_CCKPD, "lv=%d, readback CS_th=0x%x, PD th=0x%x\n",
		  cckpd_t->cck_pd_lv,
		  curr_cck_pd_t[bw][n_rx - 1][1],
		  curr_cck_pd_t[bw][n_rx - 1][0]);
}

void phydm_cckpd_type4(void *dm_void)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_cckpd_struct *cckpd_t = &dm->dm_cckpd_table;
	u8 igi = dm->dm_dig_table.cur_ig_value;
	enum cckpd_lv lv = 0;
	boolean is_update = true;

	PHYDM_DBG(dm, DBG_CCKPD, "%s ======>\n", __func__);

	if (dm->is_linked) {
		PHYDM_DBG(dm, DBG_CCKPD, "Linked!!!\n");
		#if (DM_ODM_SUPPORT_TYPE & (ODM_WIN | ODM_CE))
		if (dm->rssi_min > 40) {
			lv = CCK_PD_LV_4;
			PHYDM_DBG(dm, DBG_CCKPD, "Order 1\n");
		} else if (dm->rssi_min > 32) {
			lv = CCK_PD_LV_3;
			PHYDM_DBG(dm, DBG_CCKPD, "Order 2\n");
		} else if (dm->rssi_min > 24) {
			lv = CCK_PD_LV_2;
			PHYDM_DBG(dm, DBG_CCKPD, "Order 3\n");
		} else {
			if (cckpd_t->cck_fa_ma > 1000) {
				lv = CCK_PD_LV_1;
				PHYDM_DBG(dm, DBG_CCKPD, "Order 4-1\n");
			} else if (cckpd_t->cck_fa_ma < 500) {
				lv = CCK_PD_LV_0;
				PHYDM_DBG(dm, DBG_CCKPD, "Order 4-2\n");
			} else {
				is_update = false;
				PHYDM_DBG(dm, DBG_CCKPD, "Order 4-3\n");
			}
		}
		#else /*ODM_AP*/
		if (igi > 0x38 && dm->rssi_min > 32) {
			lv = CCK_PD_LV_4;
			PHYDM_DBG(dm, DBG_CCKPD, "Order 1\n");
		} else if (igi > 0x2a && dm->rssi_min > 32) {
			lv = CCK_PD_LV_3;
			PHYDM_DBG(dm, DBG_CCKPD, "Order 2\n");
		} else if (igi > 0x24 || dm->rssi_min > 24) {
			lv = CCK_PD_LV_2;
			PHYDM_DBG(dm, DBG_CCKPD, "Order 3\n");
		} else {
			if (cckpd_t->cck_fa_ma > 1000) {
				lv = CCK_PD_LV_1;
				PHYDM_DBG(dm, DBG_CCKPD, "Order 4-1\n");
			} else if (cckpd_t->cck_fa_ma < 500) {
				lv = CCK_PD_LV_0;
				PHYDM_DBG(dm, DBG_CCKPD, "Order 4-2\n");
			} else {
				is_update = false;
				PHYDM_DBG(dm, DBG_CCKPD, "Order 4-3\n");
			}
		}
		#endif
	} else {
		PHYDM_DBG(dm, DBG_CCKPD, "UnLinked!!!\n");
		if (cckpd_t->cck_fa_ma > 1000) {
			lv = CCK_PD_LV_1;
			PHYDM_DBG(dm, DBG_CCKPD, "Order 1\n");
		} else if (cckpd_t->cck_fa_ma < 500) {
			lv = CCK_PD_LV_0;
			PHYDM_DBG(dm, DBG_CCKPD, "Order 2\n");
		} else {
			is_update = false;
			PHYDM_DBG(dm, DBG_CCKPD, "Order 3\n");
		}
	}

	if (is_update) {
		phydm_set_cck_pd_lv_type4(dm, lv);

		PHYDM_DBG(dm, DBG_CCKPD, "setting CS_th = 0x%x, PD th = 0x%x\n",
			  cckpd_t->cckpd_jgr3[cckpd_t->cck_bw]
			  [cckpd_t->cck_n_rx - 1][1][lv],
			  cckpd_t->cckpd_jgr3[cckpd_t->cck_bw]
			  [cckpd_t->cck_n_rx - 1][0][lv]);
	}
	phydm_read_cckpd_para_type4(dm);
}

void phydm_cck_pd_init_type4(void *dm_void)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_cckpd_struct *cckpd_t = &dm->dm_cckpd_table;
	u32 reg0 = 0;
	u32 reg1 = 0;
	u32 reg2 = 0;
	u32 reg3 = 0;
	u8 pw_step = 0;
	u8 cs_step = 0;
	u8 cck_bw = 0; /*r_RX_RF_BW*/
	u8 cck_n_rx = 0;
	u8 val = 0;
	u8 i = 0;

	PHYDM_DBG(dm, DBG_CCKPD, "[%s]======>\n", __func__);

	#if 0
	/*@
	 *cckpd_t[0][0][0][0] =  1ac8[7:0]	r_PD_lim_RFBW20_1R
	 *cckpd_t[0][1][0][0] =  1ac8[15:8]	r_PD_lim_RFBW20_2R
	 *cckpd_t[0][2][0][0] =  1ac8[23:16]	r_PD_lim_RFBW20_3R
	 *cckpd_t[0][3][0][0] =  1ac8[31:24]	r_PD_lim_RFBW20_4R
	 *cckpd_t[1][0][0][0] =  1acc[7:0]	r_PD_lim_RFBW40_1R
	 *cckpd_t[1][1][0][0] =  1acc[15:8]	r_PD_lim_RFBW40_2R
	 *cckpd_t[1][2][0][0] =  1acc[23:16]	r_PD_lim_RFBW40_3R
	 *cckpd_t[1][3][0][0] =  1acc[31:24]	r_PD_lim_RFBW40_4R
	 *
	 *
	 *cckpd_t[0][0][1][0] =  1ad0[4:0]	r_CS_ratio_RFBW20_1R[4:0]
	 *cckpd_t[0][1][1][0] =  1ad0[9:5]	r_CS_ratio_RFBW20_2R[4:0]
	 *cckpd_t[0][2][1][0] =  1ad0[14:10]	r_CS_ratio_RFBW20_3R[4:0]
	 *cckpd_t[0][3][1][0] =  1ad0[19:15]	r_CS_ratio_RFBW20_4R[4:0]
	 *cckpd_t[1][0][1][0] =  1ad0[24:20]	r_CS_ratio_RFBW40_1R[4:0]
	 *cckpd_t[1][1][1][0] =  1ad0[29:25]	r_CS_ratio_RFBW40_2R[4:0]
	 *cckpd_t[1][2][1][0] =  1ad0[31:30]	r_CS_ratio_RFBW40_3R[1:0]
	 *			 1ad4[2:0]	r_CS_ratio_RFBW40_3R[4:2]
	 *cckpd_t[1][3][1][0] =  1ad4[7:3]	r_CS_ratio_RFBW40_4R[4:0]
	 */
	#endif
	/*[Check Nrx]*/
	cck_n_rx = (u8)odm_get_bb_reg(dm, R_0x1a2c, 0x60000) + 1;

	/*[Check BW]*/
	val = (u8)odm_get_bb_reg(dm, R_0x9b0, 0xc);
	if (val == 0)
		cck_bw = CHANNEL_WIDTH_20;
	else if (val == 1)
		cck_bw = CHANNEL_WIDTH_40;
	else
		cck_bw = CHANNEL_WIDTH_80;

	cckpd_t->cck_bw = cck_bw;
	cckpd_t->cck_n_rx = cck_n_rx;
	reg0 = odm_get_bb_reg(dm, R_0x1ac8, MASKDWORD);
	reg1 = odm_get_bb_reg(dm, R_0x1acc, MASKDWORD);
	reg2 = odm_get_bb_reg(dm, R_0x1ad0, MASKDWORD);
	reg3 = odm_get_bb_reg(dm, R_0x1ad4, MASKDWORD);

	for (i = 0 ; i < CCK_PD_LV_MAX ; i++) {
		pw_step = i * 2;
		cs_step = i * 2;

		#if (RTL8197G_SUPPORT)
		if (dm->support_ic_type & ODM_RTL8197G) {
			pw_step = i;
			cs_step = i;
			if (i > CCK_PD_LV_3) {
				pw_step = 3;
				cs_step = 3;
			}
		}
		#endif

		#if (RTL8822C_SUPPORT)
		if (dm->support_ic_type & ODM_RTL8822C) {
			if (i == CCK_PD_LV_1) {
				pw_step = 9; /*IGI-19.2:0x11=d'17*/
				cs_step = 0;
			} else if (i == CCK_PD_LV_2) {
				pw_step = 12; /*IGI-15.5:0x14=d'20*/
				cs_step = 1;
			} else if (i == CCK_PD_LV_3) {
				pw_step = 14; /*IGI-14:0x16=d'22*/
				cs_step = 1;
			} else if (i == CCK_PD_LV_4) {
				pw_step = 17; /*IGI-12:0x19=d'25*/
				cs_step = 1;
			}
		}
		#endif
		
		val = (u8)(reg0 & 0x000000ff) + pw_step;
		PHYDM_DBG(dm, DBG_CCKPD, "lvl %d val = %x\n\n", i, val);
		cckpd_t->cckpd_jgr3[0][0][0][i] = val;

		val = (u8)(reg1 & 0x000000ff) + pw_step;
		cckpd_t->cckpd_jgr3[1][0][0][i] = val;

		val = (u8)(reg2 & 0x0000001f) + cs_step;
		cckpd_t->cckpd_jgr3[0][0][1][i] = val;

		val = (u8)((reg2 & 0x01f00000) >> 20) + cs_step;
		cckpd_t->cckpd_jgr3[1][0][1][i] = val;

		#ifdef PHYDM_COMPILE_ABOVE_2SS
		if (dm->support_ic_type & PHYDM_IC_ABOVE_2SS) {
			val = (u8)((reg0 & 0x0000ff00) >> 8) + pw_step;
			cckpd_t->cckpd_jgr3[0][1][0][i] = val;

			val = (u8)((reg1 & 0x0000ff00) >> 8) + pw_step;
			cckpd_t->cckpd_jgr3[1][1][0][i] = val;

			val = (u8)((reg2 & 0x000003e0) >> 5) + cs_step;
			cckpd_t->cckpd_jgr3[0][1][1][i] = val;

			val = (u8)((reg2 & 0x3e000000) >> 25) + cs_step;
			cckpd_t->cckpd_jgr3[1][1][1][i] = val;
		}
		#endif

		#ifdef PHYDM_COMPILE_ABOVE_3SS
		if (dm->support_ic_type & PHYDM_IC_ABOVE_3SS) {
			val = (u8)((reg0 & 0x00ff0000) >> 16) + pw_step;
			cckpd_t->cckpd_jgr3[0][2][0][i] = val;

			val = (u8)((reg1 & 0x00ff0000) >> 16) + pw_step;
			cckpd_t->cckpd_jgr3[1][2][0][i] = val;
			val = (u8)((reg2 & 0x00007c00) >> 10) + cs_step;
			cckpd_t->cckpd_jgr3[0][2][1][i] = val;
			val = (u8)(((reg2 & 0xc0000000) >> 30) |
			      ((reg3 & 0x7) << 3)) + cs_step;
			cckpd_t->cckpd_jgr3[1][2][1][i] = val;
		}
		#endif

		#ifdef PHYDM_COMPILE_ABOVE_4SS
		if (dm->support_ic_type & PHYDM_IC_ABOVE_4SS) {
			val = (u8)((reg0 & 0xff000000) >> 24) + pw_step;
			cckpd_t->cckpd_jgr3[0][3][0][i] = val;

			val = (u8)((reg1 & 0xff000000) >> 24) + pw_step;
			cckpd_t->cckpd_jgr3[1][3][0][i] = val;

			val = (u8)((reg2 & 0x000f8000) >> 15) + cs_step;
			cckpd_t->cckpd_jgr3[0][3][1][i] = val;

			val = (u8)((reg3 & 0x000000f8) >> 3) + cs_step;
			cckpd_t->cckpd_jgr3[1][3][1][i] = val;
		}
		#endif
	}
}

void phydm_invalid_cckpd_type4(void *dm_void)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_cckpd_struct *cckpd_t = &dm->dm_cckpd_table;
	u8 val = 0;
	u8 i = 0;
	u8 j = 0;
	u8 k = 0;

	PHYDM_DBG(dm, DBG_CCKPD, "[%s]======>\n", __func__);

	for (i = 0; i < CCK_PD_LV_MAX; i++) {
		for (j = 0; j < 2; j++) {
			for (k = 0; k < dm->num_rf_path; k++) {
				val = cckpd_t->cckpd_jgr3[j][k][1][i];
				if (val == INVALID_CS_RATIO_0)
					cckpd_t->cckpd_jgr3[j][k][1][i] = 0x1c;
				else if (val == INVALID_CS_RATIO_1)
					cckpd_t->cckpd_jgr3[j][k][1][i] = 0x1e;
				else if (val > MAXVALID_CS_RATIO)
					cckpd_t->cckpd_jgr3[j][k][1][i] =
					MAXVALID_CS_RATIO;
			}
		}

		#if (RTL8198F_SUPPORT)
		if (dm->support_ic_type & ODM_RTL8198F) {
			val = cckpd_t->cckpd_jgr3[0][3][1][i];
			if (i == CCK_PD_LV_1 && val > 0x10)
				cckpd_t->cckpd_jgr3[0][3][1][i] = 0x10;
			else if (i == CCK_PD_LV_2 && val > 0x10)
				cckpd_t->cckpd_jgr3[0][3][1][i] = 0x10;
			else if (i == CCK_PD_LV_3 && val > 0x10)
				cckpd_t->cckpd_jgr3[0][3][1][i] = 0x10;
			else if (i == CCK_PD_LV_4 && val > 0x10)
				cckpd_t->cckpd_jgr3[0][3][1][i] = 0x10;
			val = cckpd_t->cckpd_jgr3[1][3][1][i];
			if (i == CCK_PD_LV_1 && val > 0xF)
				cckpd_t->cckpd_jgr3[1][3][1][i] = 0xF;
			else if (i == CCK_PD_LV_2 && val > 0xF)
				cckpd_t->cckpd_jgr3[1][3][1][i] = 0xF;
			else if (i == CCK_PD_LV_3 && val > 0xF)
				cckpd_t->cckpd_jgr3[1][3][1][i] = 0xF;
			else if (i == CCK_PD_LV_4 && val > 0xF)
				cckpd_t->cckpd_jgr3[1][3][1][i] = 0xF;
		}
		#endif
	}
}

#endif /*#ifdef PHYDM_COMPILE_CCKPD_TYPE4*/


#ifdef PHYDM_COMPILE_CCKPD_TYPE5
void phydm_write_cck_pd_type5(void *dm_void, enum cckpd_lv lv,
			      enum cckpd_mode mode)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_cckpd_struct *cckpd_t = &dm->dm_cckpd_table;
	u32 val = 0;

	PHYDM_DBG(dm, DBG_CCKPD, "write CCK CCA parameters(CS_ratio & PD)\n");
	switch (mode) {
	case CCK_BW20_1R: /*RFBW20_1R*/
	{
		val = cckpd_t->cck_pd_table_jgr3[0][0][0][lv];
		odm_set_bb_reg(dm, R_0x1a30, 0x1f, val);
		val = cckpd_t->cck_pd_table_jgr3[0][0][1][lv];
		odm_set_bb_reg(dm, R_0x1a20, 0x1f, val);
	} break;
	case CCK_BW40_1R: /*RFBW40_1R*/
	{
		val = cckpd_t->cck_pd_table_jgr3[1][0][0][lv];
		odm_set_bb_reg(dm, R_0x1a34, 0x1f, val);
		val = cckpd_t->cck_pd_table_jgr3[1][0][1][lv];
		odm_set_bb_reg(dm, R_0x1a24, 0x1f, val);
	} break;
	#if (defined(PHYDM_COMPILE_ABOVE_2SS))
	case CCK_BW20_2R: /*RFBW20_2R*/
	{
		val = cckpd_t->cck_pd_table_jgr3[0][1][0][lv];
		odm_set_bb_reg(dm, R_0x1a30, 0x3e0, val);
		val = cckpd_t->cck_pd_table_jgr3[0][1][1][lv];
		odm_set_bb_reg(dm, R_0x1a20, 0x3e0, val);
	} break;
	case CCK_BW40_2R: /*RFBW40_2R*/
	{
		val = cckpd_t->cck_pd_table_jgr3[1][1][0][lv];
		odm_set_bb_reg(dm, R_0x1a34, 0x3e0, val);
		val = cckpd_t->cck_pd_table_jgr3[1][1][1][lv];
		odm_set_bb_reg(dm, R_0x1a24, 0x3e0, val);
	} break;
	#endif
	#if (defined(PHYDM_COMPILE_ABOVE_3SS))
	case CCK_BW20_3R: /*RFBW20_3R*/
	{
		val = cckpd_t->cck_pd_table_jgr3[0][2][0][lv];
		odm_set_bb_reg(dm, R_0x1a30, 0x7c00, val);
		val = cckpd_t->cck_pd_table_jgr3[0][2][1][lv];
		odm_set_bb_reg(dm, R_0x1a20, 0x7c00, val);
	} break;
	case CCK_BW40_3R: /*RFBW40_3R*/
	{
		val = cckpd_t->cck_pd_table_jgr3[1][2][0][lv];
		odm_set_bb_reg(dm, R_0x1a34, 0x7c00, val);
		val = cckpd_t->cck_pd_table_jgr3[1][2][1][lv];
		odm_set_bb_reg(dm, R_0x1a24, 0x7c00, val);
	} break;
	#endif
	#if (defined(PHYDM_COMPILE_ABOVE_4SS))
	case CCK_BW20_4R: /*RFBW20_4R*/
	{
		val = cckpd_t->cck_pd_table_jgr3[0][3][0][lv];
		odm_set_bb_reg(dm, R_0x1a30, 0xF8000, val);
		val = cckpd_t->cck_pd_table_jgr3[0][3][1][lv];
		odm_set_bb_reg(dm, R_0x1a20, 0xF8000, val);
	} break;
	case CCK_BW40_4R: /*RFBW40_4R*/
	{
		val = cckpd_t->cck_pd_table_jgr3[1][3][0][lv];
		odm_set_bb_reg(dm, R_0x1a34, 0xF8000, val);
		val = cckpd_t->cck_pd_table_jgr3[1][3][1][lv];
		odm_set_bb_reg(dm, R_0x1a24, 0xF8000, val);
	} break;
	#endif
	default:
		/*@pr_debug("[%s] warning!\n", __func__);*/
		break;
	}
}


void phydm_set_cck_pd_lv_type5(void *dm_void, enum cckpd_lv lv)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_cckpd_struct *cckpd_t = &dm->dm_cckpd_table;
	enum cckpd_mode cck_mode = CCK_BW20_1R;
	enum channel_width cck_bw = CHANNEL_WIDTH_20;
	u8 cck_n_rx = 0;
	u32 val = 0;
	/*u32 val_dbg = 0;*/

	PHYDM_DBG(dm, DBG_CCKPD, "%s ======>\n", __func__);
	PHYDM_DBG(dm, DBG_CCKPD, "lv: (%d) -> (%d)\n", cckpd_t->cck_pd_lv, lv);

	/*[Check Nrx] for 8723F*/
	cck_n_rx = 1;

	/*[Check BW]*/
	val = odm_get_bb_reg(dm, R_0x9b0, 0xc);
	if (val == 0)
		cck_bw = CHANNEL_WIDTH_20;
	else if (val == 1)
		cck_bw = CHANNEL_WIDTH_40;
	else
		cck_bw = CHANNEL_WIDTH_80;

	/*[Check LV]*/
	if (cckpd_t->cck_pd_lv == lv &&
	    cckpd_t->cck_n_rx == cck_n_rx &&
	    cckpd_t->cck_bw == cck_bw) {
		PHYDM_DBG(dm, DBG_CCKPD, "stay in lv=%d\n", lv);
		return;
	}
	cckpd_t->cck_bw = cck_bw;
	cckpd_t->cck_n_rx = cck_n_rx;
	cckpd_t->cck_pd_lv = lv;
	cckpd_t->cck_fa_ma = CCK_FA_MA_RESET;

	switch (cck_n_rx) {
	case 1: /*1R*/
	{
		if (cck_bw == CHANNEL_WIDTH_20)
			cck_mode = CCK_BW20_1R;
		else if (cck_bw == CHANNEL_WIDTH_40)
			cck_mode = CCK_BW40_1R;
	} break;
	#if (defined(PHYDM_COMPILE_ABOVE_2SS))
	case 2: /*2R*/
	{
		if (cck_bw == CHANNEL_WIDTH_20)
			cck_mode = CCK_BW20_2R;
		else if (cck_bw == CHANNEL_WIDTH_40)
			cck_mode = CCK_BW40_2R;
	} break;
	#endif
	#if (defined(PHYDM_COMPILE_ABOVE_3SS))
	case 3: /*3R*/
	{
		if (cck_bw == CHANNEL_WIDTH_20)
			cck_mode = CCK_BW20_3R;
		else if (cck_bw == CHANNEL_WIDTH_40)
			cck_mode = CCK_BW40_3R;
	} break;
	#endif
	#if (defined(PHYDM_COMPILE_ABOVE_4SS))
	case 4: /*4R*/
	{
		if (cck_bw == CHANNEL_WIDTH_20)
			cck_mode = CCK_BW20_4R;
		else if (cck_bw == CHANNEL_WIDTH_40)
			cck_mode = CCK_BW40_4R;
	} break;
	#endif
	default:
		/*@pr_debug("[%s] warning!\n", __func__);*/
		break;
	}


	
phydm_write_cck_pd_type5(dm, lv, cck_mode);
}

void phydm_read_cckpd_para_type5(void *dm_void)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_cckpd_struct *cckpd_t = &dm->dm_cckpd_table;
	u8 bw = 0; /*r_RX_RF_BW*/
	u8 n_rx = 0;
	u8 curr_cck_pd_t[2][4][2];
	u32 reg0 = 0;
	u32 reg1 = 0;
	u32 reg2 = 0;
	u32 reg3 = 0;

	bw = (u8)odm_get_bb_reg(dm, R_0x9b0, 0xc);

	reg0 = odm_get_bb_reg(dm, R_0x1a30, MASKDWORD);
	reg1 = odm_get_bb_reg(dm, R_0x1a34, MASKDWORD);
	reg2 = odm_get_bb_reg(dm, R_0x1a20, MASKDWORD);
	reg3 = odm_get_bb_reg(dm, R_0x1a24, MASKDWORD);
	curr_cck_pd_t[0][0][0] = (u8)(reg0 & 0x0000001f);
	curr_cck_pd_t[1][0][0] = (u8)(reg1 & 0x0000001f);
	curr_cck_pd_t[0][0][1] = (u8)(reg2 & 0x0000001f);
	curr_cck_pd_t[1][0][1] = (u8)(reg3 & 0x0000001f);
	n_rx = 1;
	#if (defined(PHYDM_COMPILE_ABOVE_2SS))
	if (dm->support_ic_type & PHYDM_IC_ABOVE_2SS) {
		curr_cck_pd_t[0][1][0] = (u8)((reg0 & 0x000003E0) >> 5);
		curr_cck_pd_t[1][1][0] = (u8)((reg1 & 0x000003E0) >> 5);
		curr_cck_pd_t[0][1][1] = (u8)((reg2 & 0x000003E0) >> 5);
		curr_cck_pd_t[1][1][1] = (u8)((reg3 & 0x000003E0) >> 5);
		n_rx = 2;
	}
	#endif
	#if (defined(PHYDM_COMPILE_ABOVE_3SS))
	if (dm->support_ic_type & PHYDM_IC_ABOVE_3SS) {
		curr_cck_pd_t[0][2][0] = (u8)((reg0 & 0x00007C00) >> 10);
		curr_cck_pd_t[1][2][0] = (u8)((reg1 & 0x00007C00) >> 10);
		curr_cck_pd_t[0][2][1] = (u8)((reg2 & 0x00007C00) >> 10);
		curr_cck_pd_t[1][2][1] = (u8)((reg3 & 0x00007C00) >> 10);
		n_rx = 3;
	}
	#endif
	#if (defined(PHYDM_COMPILE_ABOVE_4SS))
	if (dm->support_ic_type & PHYDM_IC_ABOVE_4SS) {
		curr_cck_pd_t[0][3][0] = (u8)((reg0 & 0x000F8000) >> 15);
		curr_cck_pd_t[1][3][0] = (u8)((reg1 & 0x000F8000) >> 15);
		curr_cck_pd_t[0][3][1] = (u8)((reg2 & 0x000F8000) >> 15);
		curr_cck_pd_t[1][3][1] = (u8)((reg3 & 0x000F8000) >> 15);
		n_rx = 4;
	}
	#endif

	PHYDM_DBG(dm, DBG_CCKPD, "bw=%dM, Nrx=%d\n", 20 << bw, n_rx);
	PHYDM_DBG(dm, DBG_CCKPD, "lv=%d, readback CS_th=0x%x, PD th=0x%x\n",
		  cckpd_t->cck_pd_lv,
		  curr_cck_pd_t[bw][n_rx - 1][1],
		  curr_cck_pd_t[bw][n_rx - 1][0]);
}

void phydm_cckpd_type5(void *dm_void)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_cckpd_struct *cckpd_t = &dm->dm_cckpd_table;
	u8 igi = dm->dm_dig_table.cur_ig_value;
	enum cckpd_lv lv = 0;
	boolean is_update = true;

	PHYDM_DBG(dm, DBG_CCKPD, "%s ======>\n", __func__);

	if (dm->is_linked) {
		PHYDM_DBG(dm, DBG_CCKPD, "Linked!!!\n");
		if (dm->rssi_min > 40) {
			lv = CCK_PD_LV_4;
			PHYDM_DBG(dm, DBG_CCKPD, "Order 1\n");
		} else if (dm->rssi_min > 32) {
			lv = CCK_PD_LV_3;
			PHYDM_DBG(dm, DBG_CCKPD, "Order 2\n");
		} else if (dm->rssi_min > 24) {
			lv = CCK_PD_LV_2;
			PHYDM_DBG(dm, DBG_CCKPD, "Order 3\n");
		} else {
			if (cckpd_t->cck_fa_ma > 1000) {
				lv = CCK_PD_LV_1;
				PHYDM_DBG(dm, DBG_CCKPD, "Order 4-1\n");
			} else if (cckpd_t->cck_fa_ma < 500) {
				lv = CCK_PD_LV_0;
				PHYDM_DBG(dm, DBG_CCKPD, "Order 4-2\n");
			} else {
				is_update = false;
				PHYDM_DBG(dm, DBG_CCKPD, "Order 4-3\n");
			}
		}
	} else {
	PHYDM_DBG(dm, DBG_CCKPD, "UnLinked!!!\n");
		if (cckpd_t->cck_fa_ma > 1000) {
			lv = CCK_PD_LV_1;
			PHYDM_DBG(dm, DBG_CCKPD, "Order 1\n");
		} else if (cckpd_t->cck_fa_ma < 500) {
			lv = CCK_PD_LV_0;
			PHYDM_DBG(dm, DBG_CCKPD, "Order 2\n");
		} else {
			is_update = false;
			PHYDM_DBG(dm, DBG_CCKPD, "Order 3\n");
		}
	}

	if (is_update) {
		phydm_set_cck_pd_lv_type5(dm, lv);

		PHYDM_DBG(dm, DBG_CCKPD, "setting CS_th = 0x%x, PD th = 0x%x\n",
			  cckpd_t->cck_pd_table_jgr3[cckpd_t->cck_bw]
			  [cckpd_t->cck_n_rx - 1][1][lv],
			  cckpd_t->cck_pd_table_jgr3[cckpd_t->cck_bw]
			  [cckpd_t->cck_n_rx - 1][0][lv]);
	}

	phydm_read_cckpd_para_type5(dm);
}

void phydm_cck_pd_init_type5(void *dm_void)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_cckpd_struct *cckpd_t = &dm->dm_cckpd_table;
	u32 reg0 = 0;
	u32 reg1 = 0;
	u32 reg2 = 0;
	u32 reg3 = 0;
	u8 pw_step = 0;
	u8 cs_step = 0;
	u8 cck_bw = 0; /*r_RX_RF_BW*/
	u8 cck_n_rx = 0;
	u8 val = 0;
	u8 i = 0;

	PHYDM_DBG(dm, DBG_CCKPD, "[%s]======>\n", __func__);
	#if 0
	/*@
	 *cckpd_t[0][0][0][0] =  1a30[4:0]	r_PD_lim_RFBW20_1R
	 *cckpd_t[0][1][0][0] =  1a30[9:5]	r_PD_lim_RFBW20_2R
	 *cckpd_t[0][2][0][0] =  1a30[14:10]	r_PD_lim_RFBW20_3R
	 *cckpd_t[0][3][0][0] =  1a30[19:15]	r_PD_lim_RFBW20_4R
	 *cckpd_t[1][0][0][0] =  1a34[4:0]	r_PD_lim_RFBW40_1R
	 *cckpd_t[1][1][0][0] =  1a34[9:5]	r_PD_lim_RFBW40_2R
	 *cckpd_t[1][2][0][0] =  1a34[14:10]	r_PD_lim_RFBW40_3R
	 *cckpd_t[1][3][0][0] =  1a34[19:15]	r_PD_lim_RFBW40_4R
	 *
	 *
	 *cckpd_t[0][0][1][0] =  1a20[4:0]	r_CS_ratio_RFBW20_1R
	 *cckpd_t[0][1][1][0] =  1a20[9:5]	r_CS_ratio_RFBW20_2R
	 *cckpd_t[0][2][1][0] =  1a20[14:10]	r_CS_ratio_RFBW20_3R
	 *cckpd_t[0][3][1][0] =  1a20[19:15]	r_CS_ratio_RFBW20_4R
	 *cckpd_t[1][0][1][0] =  1a24[4:0]	r_CS_ratio_RFBW40_1R
	 *cckpd_t[1][1][1][0] =  1a24[9:5]	r_CS_ratio_RFBW40_2R
	 *cckpd_t[1][2][1][0] =  1a24[14:10]	r_CS_ratio_RFBW40_3R
	 *cckpd_t[1][3][1][0] =  1a24[19:15]	r_CS_ratio_RFBW40_4R
	 */
	#endif
	/*[Check Nrx]*/
	cck_n_rx = 1;

	/*[Check BW]*/
	val = (u8)odm_get_bb_reg(dm, R_0x9b0, 0xc);
	if (val == 0)
		cck_bw = CHANNEL_WIDTH_20;
	else if (val == 1)
		cck_bw = CHANNEL_WIDTH_40;
	else
		cck_bw = CHANNEL_WIDTH_80;

	cckpd_t->cck_bw = cck_bw;
	reg0 = odm_get_bb_reg(dm, R_0x1a30, MASKDWORD);
	reg1 = odm_get_bb_reg(dm, R_0x1a34, MASKDWORD);
	reg2 = odm_get_bb_reg(dm, R_0x1a20, MASKDWORD);
	reg3 = odm_get_bb_reg(dm, R_0x1a24, MASKDWORD);

	for (i = 0 ; i < CCK_PD_LV_MAX ; i++) {
		pw_step = i * 2;
		cs_step = i * 2;

		#if (RTL8723F_SUPPORT)
		if (dm->support_ic_type & ODM_RTL8723F) {
			if (i == CCK_PD_LV_1) {
				pw_step = 9; /*IGI-19.2:0x11=d'17*/
				cs_step = 0;
			} else if (i == CCK_PD_LV_2) {
				pw_step = 12; /*IGI-15.5:0x14=d'20*/
				cs_step = 1;
			} else if (i == CCK_PD_LV_3) {
				pw_step = 14; /*IGI-14:0x16=d'22*/
				cs_step = 1;
			} else if (i == CCK_PD_LV_4) {
				pw_step = 17; /*IGI-12:0x19=d'25*/
				cs_step = 1;
			}
		}
		#endif
		val = (u8)(reg0 & 0x0000001F) + pw_step;
		PHYDM_DBG(dm, DBG_CCKPD, "lvl %d val = %x\n\n", i, val);
		cckpd_t->cck_pd_table_jgr3[0][0][0][i] = val;

		val = (u8)(reg1 & 0x0000001F) + pw_step;
		cckpd_t->cck_pd_table_jgr3[1][0][0][i] = val;

		val = (u8)(reg2 & 0x0000001F) + cs_step;
		cckpd_t->cck_pd_table_jgr3[0][0][1][i] = val;

		val = (u8)(reg3 & 0x0000001F) + cs_step;
		cckpd_t->cck_pd_table_jgr3[1][0][1][i] = val;

		#ifdef PHYDM_COMPILE_ABOVE_2SS
		if (dm->support_ic_type & PHYDM_IC_ABOVE_2SS) {
			val = (u8)((reg0 & 0x000003E0) >> 5) + pw_step;
			cckpd_t->cck_pd_table_jgr3[0][1][0][i] = val;

			val = (u8)((reg1 & 0x000003E0) >> 5) + pw_step;
			cckpd_t->cck_pd_table_jgr3[1][1][0][i] = val;

			val = (u8)((reg2 & 0x000003E0) >> 5) + cs_step;
			cckpd_t->cck_pd_table_jgr3[0][1][1][i] = val;

			val = (u8)((reg3 & 0x000003E0) >> 5) + cs_step;
			cckpd_t->cck_pd_table_jgr3[1][1][1][i] = val;

			cck_n_rx = 2;
		}
		#endif
		#ifdef PHYDM_COMPILE_ABOVE_3SS
		if (dm->support_ic_type & PHYDM_IC_ABOVE_3SS) {
			val = (u8)((reg0 & 0x00007C00) >> 10) + pw_step;
			cckpd_t->cck_pd_table_jgr3[0][2][0][i] = val;

			val = (u8)((reg1 & 0x00007C00) >> 10) + pw_step;
			cckpd_t->cck_pd_table_jgr3[1][2][0][i] = val;

			val = (u8)((reg2 & 0x00007C00) >> 10) + cs_step;
			cckpd_t->cck_pd_table_jgr3[0][2][1][i] = val;

			val = (u8)((reg3 & 0x00007C00) >> 10) + cs_step;
			cckpd_t->cck_pd_table_jgr3[1][2][1][i] = val;

			cck_n_rx = 3;
		}
		#endif

		#ifdef PHYDM_COMPILE_ABOVE_4SS
		if (dm->support_ic_type & PHYDM_IC_ABOVE_4SS) {
			val = (u8)((reg0 & 0x000F8000) >> 15) + pw_step;
			cckpd_t->cck_pd_table_jgr3[0][3][0][i] = val;

			val = (u8)((reg1 & 0x000F8000) >> 15) + pw_step;
			cckpd_t->cck_pd_table_jgr3[1][3][0][i] = val;

			val = (u8)((reg2 & 0x000F8000) >> 15) + cs_step;
			cckpd_t->cck_pd_table_jgr3[0][3][1][i] = val;

			val = (u8)((reg3 & 0x000F8000) >> 15) + cs_step;
			cckpd_t->cck_pd_table_jgr3[1][3][1][i] = val;

			cck_n_rx = 4;
		}
		#endif
			}
	cckpd_t->cck_n_rx = cck_n_rx;
}




#endif /*#ifdef PHYDM_COMPILE_CCKPD_TYPE5*/




void phydm_set_cckpd_val(void *dm_void, u32 *val_buf, u8 val_len)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_cckpd_struct *cckpd_t = &dm->dm_cckpd_table;
	enum cckpd_lv lv;

	if (val_len != 1) {
		PHYDM_DBG(dm, ODM_COMP_API, "[Error][CCKPD]Need val_len=1\n");
		return;
	}

	lv = (enum cckpd_lv)val_buf[0];

	if (lv > CCK_PD_LV_4) {
		pr_debug("[%s] warning! lv=%d\n", __func__, lv);
		return;
	}

	switch (cckpd_t->cckpd_hw_type) {
	#ifdef PHYDM_COMPILE_CCKPD_TYPE1
	case 1:
		phydm_set_cckpd_lv_type1(dm, lv);
		break;
	#endif
	#ifdef PHYDM_COMPILE_CCKPD_TYPE2
	case 2:
		phydm_set_cckpd_lv_type2(dm, lv);
		break;
	#endif
	#ifdef PHYDM_COMPILE_CCKPD_TYPE3
	case 3:
		phydm_set_cckpd_lv_type3(dm, lv);
		break;
	#endif
	#ifdef PHYDM_COMPILE_CCKPD_TYPE4
	case 4:
		phydm_set_cck_pd_lv_type4(dm, lv);
		break;
	#endif
	#ifdef PHYDM_COMPILE_CCKPD_TYPE5
	case 5:
		phydm_set_cck_pd_lv_type5(dm, lv);
		break;
	#endif
	default:
		pr_debug("[%s]warning\n", __func__);
		break;
	}
}

boolean
phydm_stop_cck_pd_th(void *dm_void)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;

	if (!(dm->support_ability & ODM_BB_FA_CNT)) {
		PHYDM_DBG(dm, DBG_CCKPD, "Not Support:ODM_BB_FA_CNT disable\n");
		return true;
	}

	if (!(dm->support_ability & ODM_BB_CCK_PD)) {
		PHYDM_DBG(dm, DBG_CCKPD, "Not Support:ODM_BB_CCK_PD disable\n");
		return true;
	}

	if (dm->pause_ability & ODM_BB_CCK_PD) {
		PHYDM_DBG(dm, DBG_CCKPD, "Return: Pause CCKPD in LV=%d\n",
			  dm->pause_lv_table.lv_cckpd);
		return true;
	}

	if (dm->is_linked && (*dm->channel > 36)) {
		PHYDM_DBG(dm, DBG_CCKPD, "Return: 5G CH=%d\n", *dm->channel);
		return true;
	}
	return false;
}

void phydm_cck_pd_th(void *dm_void)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_fa_struct *fa_t = &dm->false_alm_cnt;
	struct phydm_cckpd_struct *cckpd_t = &dm->dm_cckpd_table;
	u32 cck_fa = fa_t->cnt_cck_fail;
	#ifdef PHYDM_TDMA_DIG_SUPPORT
	struct phydm_fa_acc_struct *fa_acc_t = &dm->false_alm_cnt_acc;
	#endif

	PHYDM_DBG(dm, DBG_CCKPD, "[%s] ======>\n", __func__);

	if (phydm_stop_cck_pd_th(dm))
		return;

	#ifdef PHYDM_TDMA_DIG_SUPPORT
	if (dm->original_dig_restore)
		cck_fa = fa_t->cnt_cck_fail;
	else
		cck_fa = fa_acc_t->cnt_cck_fail_1sec;
	#endif

	if (cckpd_t->cck_fa_ma == CCK_FA_MA_RESET)
		cckpd_t->cck_fa_ma = cck_fa;
	else
		cckpd_t->cck_fa_ma = (cckpd_t->cck_fa_ma * 3 + cck_fa) >> 2;

	PHYDM_DBG(dm, DBG_CCKPD,
		  "IGI=0x%x, rssi_min=%d, cck_fa=%d, cck_fa_ma=%d\n",
		  dm->dm_dig_table.cur_ig_value, dm->rssi_min,
		  cck_fa, cckpd_t->cck_fa_ma);

	switch (cckpd_t->cckpd_hw_type) {
	#ifdef PHYDM_COMPILE_CCKPD_TYPE1
	case 1:
		phydm_cckpd_type1(dm);
		break;
	#endif
	#ifdef PHYDM_COMPILE_CCKPD_TYPE2
	case 2:
		phydm_cckpd_type2(dm);
		break;
	#endif
	#ifdef PHYDM_COMPILE_CCKPD_TYPE3
	case 3:
		phydm_cckpd_type3(dm);
		break;
	#endif
	#ifdef PHYDM_COMPILE_CCKPD_TYPE4
	case 4:
		#ifdef PHYDM_DCC_ENHANCE
		if (dm->dm_dcc_info.dcc_en)
			phydm_cckpd_type4_dcc(dm);
		else
		#endif
			phydm_cckpd_type4(dm);
		break;
	#endif
	#ifdef PHYDM_COMPILE_CCKPD_TYPE5
	case 5:
		phydm_cckpd_type5(dm);
		break;
	#endif
	default:
		pr_debug("[%s]warning\n", __func__);
		break;
	}
}

void phydm_cck_pd_init(void *dm_void)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_cckpd_struct *cckpd_t = &dm->dm_cckpd_table;

	if (*dm->mp_mode)
		return;

	if (dm->support_ic_type & CCK_PD_IC_TYPE1)
		cckpd_t->cckpd_hw_type = 1;
	else if (dm->support_ic_type & CCK_PD_IC_TYPE2)
		cckpd_t->cckpd_hw_type = 2;
	else if (dm->support_ic_type & CCK_PD_IC_TYPE3)
		cckpd_t->cckpd_hw_type = 3;
	else if (dm->support_ic_type & CCK_PD_IC_TYPE4)
		cckpd_t->cckpd_hw_type = 4;

	if (dm->support_ic_type & CCK_PD_IC_TYPE5)
		cckpd_t->cckpd_hw_type = 5;

	PHYDM_DBG(dm, DBG_CCKPD, "[%s] cckpd_hw_type=%d\n",
		  __func__, cckpd_t->cckpd_hw_type);

	cckpd_t->cck_pd_lv = CCK_PD_LV_INIT;
	cckpd_t->cck_n_rx = 0xff;
	cckpd_t->cck_bw = CHANNEL_WIDTH_MAX;
	cckpd_t->cck_fa_th[1] = 400;
	cckpd_t->cck_fa_th[0] = 200;

	switch (cckpd_t->cckpd_hw_type) {
	#ifdef PHYDM_COMPILE_CCKPD_TYPE1
	case 1:
		phydm_set_cckpd_lv_type1(dm, CCK_PD_LV_1);
		break;
	#endif
	#ifdef PHYDM_COMPILE_CCKPD_TYPE2
	case 2:
		cckpd_t->aaa_default = odm_read_1byte(dm, 0xaaa) & 0x1f;
		phydm_set_cckpd_lv_type2(dm, CCK_PD_LV_1);
		break;
	#endif
	#ifdef PHYDM_COMPILE_CCKPD_TYPE3
	case 3:
		phydm_cck_pd_init_type3(dm);
		phydm_set_cckpd_lv_type3(dm, CCK_PD_LV_1);
		break;
	#endif
	#ifdef PHYDM_COMPILE_CCKPD_TYPE4
	case 4:
		phydm_cck_pd_init_type4(dm);
		phydm_invalid_cckpd_type4(dm);
		phydm_set_cck_pd_lv_type4(dm, CCK_PD_LV_1);
		break;
	#endif
	#ifdef PHYDM_COMPILE_CCKPD_TYPE5
	case 5:
		phydm_cck_pd_init_type5(dm);
		break;
	#endif
	default:
		pr_debug("[%s]warning\n", __func__);
		break;
	}
}

#ifdef PHYDM_DCC_ENHANCE

void phydm_cckpd_type4_dcc(void *dm_void)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_cckpd_struct *cckpd_t = &dm->dm_cckpd_table;
	enum cckpd_lv lv_curr = cckpd_t->cck_pd_lv;
	enum phydm_cck_pd_trend trend = CCKPD_STABLE;
	u8 th_ofst = 0;
	u16 lv_up_th, lv_down_th;

	PHYDM_DBG(dm, DBG_CCKPD, "%s ======>\n", __func__);

	if (!dm->is_linked)
		th_ofst = 1;

	lv_up_th = (cckpd_t->cck_fa_th[1]) << th_ofst;
	lv_down_th = (cckpd_t->cck_fa_th[0]) << th_ofst;

	PHYDM_DBG(dm, DBG_CCKPD, "th{Up, Down}: {%d, %d}\n",
		  lv_up_th, lv_down_th);

	if (cckpd_t->cck_fa_ma > lv_up_th) {
		if (lv_curr <= CCK_PD_LV_3) {
			lv_curr++;
			trend = CCKPD_INCREASING;
		} else {
			lv_curr = CCK_PD_LV_4;
		}
	} else if (cckpd_t->cck_fa_ma < lv_down_th) {
		if (lv_curr >= CCK_PD_LV_1) {
			lv_curr--;
			trend = CCKPD_DECREASING;
		} else {
			lv_curr = CCK_PD_LV_0;
		}
	}

	PHYDM_DBG(dm, DBG_CCKPD, "lv: %d->%d\n", cckpd_t->cck_pd_lv, lv_curr);
#if 1
	if (trend != CCKPD_STABLE) {
		phydm_set_cck_pd_lv_type4(dm, lv_curr);

		PHYDM_DBG(dm, DBG_CCKPD, "setting CS_th = 0x%x, PD th = 0x%x\n",
			  cckpd_t->cckpd_jgr3[cckpd_t->cck_bw]
			  [cckpd_t->cck_n_rx - 1][1][lv_curr],
			  cckpd_t->cckpd_jgr3[cckpd_t->cck_bw]
			  [cckpd_t->cck_n_rx - 1][0][lv_curr]);
	}
	phydm_read_cckpd_para_type4(dm);
#endif
}

boolean phydm_do_cckpd(void *dm_void)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_dig_struct *dig_t = &dm->dm_dig_table;

	if (dig_t->igi_trend == DIG_INCREASING)
		return false;

	return true;
}

void phydm_dig_cckpd_coex(void *dm_void)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_dcc_struct	*dcc = &dm->dm_dcc_info;

	if (*dm->channel > 36) {
		phydm_dig(dm);
		return;
	} else if (!dcc->dcc_en) {
		phydm_dig(dm);
		phydm_cck_pd_th(dm);
		return;
	}

	dcc->dig_execute_cnt++;
	PHYDM_DBG(dm, DBG_CCKPD, "DCC_cnt: %d\n", dcc->dig_execute_cnt);

	if (dcc->dig_execute_cnt % dcc->dcc_ratio) {
		PHYDM_DBG(dm, DBG_CCKPD, "DCC: DIG\n");
		phydm_dig(dm);
	} else {
		if (phydm_do_cckpd(dm)) {
			PHYDM_DBG(dm, DBG_CCKPD, "DCC: CCKPD\n");
			dcc->dcc_mode = DCC_CCK_PD;
			phydm_cck_pd_th(dm);
		} else {
			PHYDM_DBG(dm, DBG_CCKPD, "DCC: Boost_DIG\n");
			dcc->dcc_mode = DCC_DIG;
			phydm_dig(dm);
		}
	}
}

void phydm_dig_cckpd_coex_init(void *dm_void)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_dig_struct *dig_t = &dm->dm_dig_table;
	struct phydm_dcc_struct	*dcc = &dm->dm_dcc_info;

	dcc->dcc_mode = DCC_DIG;
	dcc->dcc_en = false;
	dcc->dig_execute_cnt = 0;
	dcc->dcc_ratio = 2;
}

void phydm_dig_cckpd_coex_dbg(void *dm_void, char input[][16], u32 *_used,
			      char *output, u32 *_out_len)
{
	struct dm_struct *dm = (struct dm_struct *)dm_void;
	struct phydm_cckpd_struct *cckpd_t = &dm->dm_cckpd_table;
	struct phydm_dcc_struct	*dcc = &dm->dm_dcc_info;
	char help[] = "-h";
	u32 var[10] = {0};
	u32 used = *_used;
	u32 out_len = *_out_len;
	u8 i = 0;

	for (i = 0; i < 3; i++) {
		if (input[i + 1])
			PHYDM_SSCANF(input[i + 1], DCMD_DECIMAL, &var[i]);
	}

	if ((strcmp(input[1], help) == 0)) {
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "Enable: en {0/1}\n");
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "DCC_ratio: ratio {x}\n");
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "threshold: th {Down_th} {Up_th}\n");
	} else if ((strcmp(input[1], "en") == 0)) {
		dcc->dcc_en = (var[1]) ? true : false;
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "en=%d\n", dcc->dcc_en);
	} else if ((strcmp(input[1], "ratio") == 0)) {
		dcc->dcc_ratio = (u8)var[1];
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "Ratio=%d\n", dcc->dcc_ratio);
	} else if ((strcmp(input[1], "th") == 0)) {
		cckpd_t->cck_fa_th[1] = (u16)var[2];
		cckpd_t->cck_fa_th[0] = (u16)var[1];
		PDM_SNPF(out_len, used, output + used, out_len - used,
			 "th{Down, Up}: {%d, %d}\n",
			 cckpd_t->cck_fa_th[0], cckpd_t->cck_fa_th[1]);
	}

	*_used = used;
	*_out_len = out_len;
}

#endif
#endif /*#ifdef PHYDM_SUPPORT_CCKPD*/

