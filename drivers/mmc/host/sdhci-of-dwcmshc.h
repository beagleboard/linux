/* SPDX-License-Identifier: GPL-2.0 */
/*
 * snps sdhci driver.
 *
 * Copyright (C)  2011 Renesas Solutions Corp.
 */

#ifndef _SDHCI_OF_DWCMSHC_H_
#define _SDHCI_OF_DWCMSHC_H_

#define DWC_MSHC_PTR_PHY_R  0x300
#define PHY_CNFG_R      (DWC_MSHC_PTR_PHY_R + 0x00) //32bit
#define PHY_RSTN  0x0      //1bit
#define PAD_SP    0x10     //4bit
#define PAD_SN    0x14     //4bit

#define PHY_CMDPAD_CNFG_R   (DWC_MSHC_PTR_PHY_R + 0x04) //16bit
#define PHY_DATAPAD_CNFG_R  (DWC_MSHC_PTR_PHY_R + 0x06) //16bit
#define PHY_CLKPAD_CNFG_R   (DWC_MSHC_PTR_PHY_R + 0x08) //16bit
#define PHY_STBPAD_CNFG_R   (DWC_MSHC_PTR_PHY_R + 0x0a) //16bit
#define PHY_RSTNPAD_CNFG_R  (DWC_MSHC_PTR_PHY_R + 0x0c) //16bit
#define RXSEL 0x0         //3bit
#define WEAKPULL_EN 0x3   //2bit
#define TXSLEW_CTRL_P 0x5 //4bit
#define TXSLEW_CTRL_N 0x9 //4bit

#define PHY_PADTEST_CNFG_R  (DWC_MSHC_PTR_PHY_R + 0x0e)
#define PHY_PADTEST_OUT_R   (DWC_MSHC_PTR_PHY_R + 0x10)
#define PHY_PADTEST_IN_R    (DWC_MSHC_PTR_PHY_R + 0x12)
#define PHY_PRBS_CNFG_R     (DWC_MSHC_PTR_PHY_R + 0x18)
#define PHY_PHYLBK_CNFG_R   (DWC_MSHC_PTR_PHY_R + 0x1a)
#define PHY_COMMDL_CNFG_R   (DWC_MSHC_PTR_PHY_R + 0x1c)

#define PHY_SDCLKDL_CNFG_R  (DWC_MSHC_PTR_PHY_R + 0x1d) //8bit
#define UPDATE_DC 0x4     //1bit

#define PHY_SDCLKDL_DC_R    (DWC_MSHC_PTR_PHY_R + 0x1e)
#define PHY_SMPLDL_CNFG_R   (DWC_MSHC_PTR_PHY_R + 0x20)
#define PHY_ATDL_CNFG_R     (DWC_MSHC_PTR_PHY_R + 0x21)
#define INPSEL_CNFG  2 //2bit

#define PHY_DLL_CTRL_R      (DWC_MSHC_PTR_PHY_R + 0x24)
#define DLL_EN  0x0 //1bit

#define PHY_DLL_CNFG1_R     (DWC_MSHC_PTR_PHY_R + 0x25)
#define PHY_DLL_CNFG2_R     (DWC_MSHC_PTR_PHY_R + 0x26)
#define PHY_DLLDL_CNFG_R    (DWC_MSHC_PTR_PHY_R + 0x28)
#define SLV_INPSEL 0x5 //2bit

#define PHY_DLL_OFFST_R     (DWC_MSHC_PTR_PHY_R + 0x29)
#define PHY_DLLMST_TSTDC_R  (DWC_MSHC_PTR_PHY_R + 0x2a)
#define PHY_DLLBT_CNFG_R    (DWC_MSHC_PTR_PHY_R + 0x2c)
#define PHY_DLL_STATUS_R    (DWC_MSHC_PTR_PHY_R + 0x2e)
#define PHY_DLLDBG_MLKDC_R  (DWC_MSHC_PTR_PHY_R + 0x30)
#define PHY_DLLDBG_SLKDC_R  (DWC_MSHC_PTR_PHY_R + 0x32)

#define SNPS_SDHCI_CTRL_HS400 0x7

#define P_VENDOR_SPECIFIC_AREA 0x500
#define EMMC_CTRL_R (P_VENDOR_SPECIFIC_AREA + 0x2c) //16bit
#define CARD_IS_EMMC 0x0 //1bit

#define AT_CTRL_R   (P_VENDOR_SPECIFIC_AREA + 0x40) // 32bit
#define AT_EN 0x0             //1bit
#define CI_SEL 0x1            //1bit
#define SWIN_TH_EN 0x2        //1bit
#define RPT_TUNE_ERR 0x3      //1bit
#define SW_TUNE_EN 0x4        //1bit
#define WIN_EDGE_SEL 0x8      //4bit
#define TUNE_CLK_STOP_EN 0x10 //1bit
#define PRE_CHANGE_DLY 0x11   //2bit
#define POST_CHANGE_DLY 0x13  //2bit
#define SWIN_TH_VAL 0x18      //9bit

#endif /* _SDHCI_OF_DWCMSHC_H_*/
