/*
 * Copyright (C) 2005,2006 Sebastian Smolorz
 *                        <Sebastian.Smolorz@stud.uni-hannover.de>
 *
 * Based on drivers/can/sja1000.h in linux-can.patch, a CAN socket
 * framework for Linux:
 *
 * Copyright (C) 2005, Sascha Hauer, Pengutronix
 *
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __SJA1000_REGS_H_
#define __SJA1000_REGS_H_


/* PeliCAN mode address map */

/* reset and operating mode */
#define SJA_MOD          0       /* Mode register                   */
#define SJA_CMR          1       /* Command register                */
#define SJA_SR           2       /* Status register                 */
#define SJA_IR           3       /* Interrupt register              */
#define SJA_IER          4       /* Interrupt enable register       */
#define SJA_BTR0         6       /* Bus timing register 0           */
#define SJA_BTR1         7       /* Bus timing register 1           */
#define SJA_OCR          8       /* Output control register         */
#define SJA_ALC         11       /* Arbitration lost capture        */
#define SJA_ECC         12       /* Error code capture register     */
#define SJA_RXERR       14       /* Receive error counter           */
#define SJA_TXERR       15       /* Transmit error counter          */
#define SJA_CDR         31       /* Clock divider register          */

/* reset mode */
#define SJA_ACR0        16       /* Acceptance code register 0      */
#define SJA_ACR1        17       /* Acceptance code register 1      */
#define SJA_ACR2        18       /* Acceptance code register 2      */
#define SJA_ACR3        19       /* Acceptance code register 3      */
#define SJA_AMR0        20       /* Acceptance mask register 0      */
#define SJA_AMR1        21       /* Acceptance mask register 1      */
#define SJA_AMR2        22       /* Acceptance mask register 2      */
#define SJA_AMR3        23       /* Acceptance mask register 3      */

/* operating mode */
#define SJA_FIR         16       /* Frame information register      */
#define SJA_ID1         17       /* Identifier 1                    */
#define SJA_ID2         18       /* Identifier 2                    */
#define SJA_ID3         19       /* Identifier 3 (EFF only)         */
#define SJA_ID4         20       /* Identifier 4 (EFF only)         */

#define SJA_DATA_SFF(x) (19 + (x)) /* Data registers in case of standard
				    * frame format; 0 <= x <= 7 */
#define SJA_DATA_EFF(x) (21 + (x)) /* Data registers in case of extended
				    * frame format; 0 <= x <= 7 */

/* Mode register */
enum SJA1000_PELI_MOD {
    SJA_MOD_RM           = 1,    /* Reset Mode                          */
    SJA_MOD_LOM          = 1<<1, /* Listen Only Mode                    */
    SJA_MOD_STM          = 1<<2, /* Self Test Mode                      */
    SJA_MOD_AFM          = 1<<3, /* Acceptance Filter Mode              */
    SJA_MOD_SM           = 1<<4  /* Sleep Mode                          */
};

/* Command register */
enum SJA1000_PELI_CMR {
    SJA_CMR_TR  = 1,             /* Transmission request                */
    SJA_CMR_AT  = 1<<1,          /* Abort Transmission                  */
    SJA_CMR_RRB = 1<<2,          /* Release Receive Buffer              */
    SJA_CMR_CDO = 1<<3,          /* Clear Data Overrun                  */
    SJA_CMR_SRR = 1<<4           /* Self reception request              */
};

/* Status register */
enum SJA1000_PELI_SR {
    SJA_SR_RBS           = 1,    /* Receive Buffer Status               */
    SJA_SR_DOS           = 1<<1, /* Data Overrun Status                 */
    SJA_SR_TBS           = 1<<2, /* Transmit Buffer Status              */
    SJA_SR_ES            = 1<<6, /* Error Status                        */
    SJA_SR_BS            = 1<<7  /* Bus Status                          */
};

/* Interrupt register */
enum SJA1000_PELI_IR {
    SJA_IR_RI           = 1,     /* Receive Interrupt                   */
    SJA_IR_TI           = 1<<1,  /* Transmit Interrupt                  */
    SJA_IR_EI           = 1<<2,  /* Error Warning Interrupt             */
    SJA_IR_DOI          = 1<<3,  /* Data Overrun Interrupt              */
    SJA_IR_WUI          = 1<<4,  /* Wake-Up Interrupt                   */
    SJA_IR_EPI          = 1<<5,  /* Error Passive Interrupt             */
    SJA_IR_ALI          = 1<<6,  /* Arbitration Lost Interrupt          */
    SJA_IR_BEI          = 1<<7,  /* Bus Error Interrupt                 */
};

/* Interrupt enable register */
enum SJA1000_PELI_IER {
    SJA_IER_RIE         = 1,     /* Receive Interrupt Enable            */
    SJA_IER_TIE         = 1<<1,  /* Transmit Interrupt Enable           */
    SJA_IER_EIE         = 1<<2,  /* Error Warning Interrupt Enable      */
    SJA_IER_DOIE        = 1<<3,  /* Data Overrun Interrupt Enable       */
    SJA_IER_WUIE        = 1<<4,  /* Wake-Up Interrupt Enable            */
    SJA_IER_EPIE        = 1<<5,  /* Error Passive Interrupt Enable      */
    SJA_IER_ALIE        = 1<<6,  /* Arbitration Lost Interrupt Enable   */
    SJA_IER_BEIE        = 1<<7,  /* Bus Error Interrupt Enable          */
};

/* Bus timing register 0 */
enum SJA1000_PELI_BTR0 {
    /* Period of the CAN system clock t_SCl
     * (t_CLK = time period of XTAL frequency) */
    SJA_BTR0_T_SCL_2_T_CLK  = 0,    /* t_SCl = 2 x t_CLK                 */
    SJA_BTR0_T_SCL_4_T_CLK  = 1,    /* t_SCl = 4 x t_CLK                 */
    SJA_BTR0_T_SCL_6_T_CLK  = 2,    /* t_SCl = 6 x t_CLK                 */
    SJA_BTR0_T_SCL_8_T_CLK  = 3,    /* t_SCl = 8 x t_CLK                 */
    SJA_BTR0_T_SCL_10_T_CLK = 4,    /* t_SCl = 10 x t_CLK                */
    SJA_BTR0_T_SCL_12_T_CLK = 5,    /* t_SCl = 12 x t_CLK                */
    SJA_BTR0_T_SCL_14_T_CLK = 6,    /* t_SCl = 14 x t_CLK                */
    SJA_BTR0_T_SCL_16_T_CLK = 7,    /* t_SCl = 16 x t_CLK                */
    SJA_BTR0_T_SCL_20_T_CLK = 9,    /* t_SCl = 20 x t_CLK                */
    SJA_BTR0_T_SCL_40_T_CLK = 19,   /* t_SCl = 40 x t_CLK                */
    SJA_BTR0_T_SCL_100_T_CLK = 49,  /* t_SCl = 100 x t_CLK               */

};

/* Bus timing register 1 */
enum SJA1000_PELI_BTR1 {
    /* Time segment 1 */
    SJA_BTR1_T_SEG1_1_T_SCL = 0,    /* t_SEG1 = 1 x t_SCl               */
    SJA_BTR1_T_SEG1_2_T_SCL = 1,    /* t_SEG1 = 2 x t_SCl               */
    SJA_BTR1_T_SEG1_3_T_SCL = 2,    /* t_SEG1 = 3 x t_SCl               */
    SJA_BTR1_T_SEG1_4_T_SCL = 3,    /* t_SEG1 = 4 x t_SCl               */
    SJA_BTR1_T_SEG1_5_T_SCL = 4,    /* t_SEG1 = 5 x t_SCl               */
    SJA_BTR1_T_SEG1_6_T_SCL = 5,    /* t_SEG1 = 6 x t_SCl               */
    SJA_BTR1_T_SEG1_7_T_SCL = 6,    /* t_SEG1 = 7 x t_SCl               */
    SJA_BTR1_T_SEG1_8_T_SCL = 7,    /* t_SEG1 = 8 x t_SCl               */
    /* Time segment 2 */
    SJA_BTR1_T_SEG2_1_T_SCL = 0<<4, /* t_SEG2 = 1 x t_SCl               */
    SJA_BTR1_T_SEG2_2_T_SCL = 1<<4, /* t_SEG2 = 2 x t_SCl               */
    SJA_BTR1_T_SEG2_3_T_SCL = 2<<4, /* t_SEG2 = 3 x t_SCl               */
    SJA_BTR1_T_SEG2_4_T_SCL = 3<<4, /* t_SEG2 = 4 x t_SCl               */
    SJA_BTR1_T_SEG2_5_T_SCL = 4<<4, /* t_SEG2 = 5 x t_SCl               */
    SJA_BTR1_T_SEG2_6_T_SCL = 5<<4, /* t_SEG2 = 6 x t_SCl               */
    SJA_BTR1_T_SEG2_7_T_SCL = 6<<4, /* t_SEG2 = 7 x t_SCl               */
    SJA_BTR1_T_SEG2_8_T_SCL = 7<<4, /* t_SEG2 = 8 x t_SCl               */
};

/* One bit time = t_SCl + t_SEG1 + t_SEG2 */


/* Output control register */
enum SJA1000_PELI_OCR {
    SJA_OCR_MODE_BIPHASE = 0,
    SJA_OCR_MODE_TEST    = 1,
    SJA_OCR_MODE_NORMAL  = 2,
    SJA_OCR_MODE_CLOCK   = 3,
    SJA_OCR_TX0_INVERT   = 1<<2,
    SJA_OCR_TX0_PULLDOWN = 1<<3,
    SJA_OCR_TX0_PULLUP   = 2<<3,
    SJA_OCR_TX0_PUSHPULL = 3<<3,
    SJA_OCR_TX1_INVERT   = 1<<5,
    SJA_OCR_TX1_PULLDOWN = 1<<6,
    SJA_OCR_TX1_PULLUP   = 2<<6,
    SJA_OCR_TX1_PUSHPULL = 3<<6
};

/* Error code capture register */
enum SJA1000_PELI_ECC {
    /* The segmentation field gives information about the location of
     * errors on the bus */
    SJA_ECC_SEG_MASK     = 31,   /* Segmentation field mask             */
    SJA_ECC_DIR          = 1<<5, /* Transfer direction                  */
    SJA_ECC_ERR_BIT      = 0<<6,
    SJA_ECC_ERR_FORM     = 1<<6,
    SJA_ECC_ERR_STUFF    = 2<<6,
    SJA_ECC_ERR_MASK     = 3<<6  /* Error code mask                     */
};

/* Frame information register */
enum SJA1000_PELI_FIR {
    SJA_FIR_DLC_MASK     = 15,   /* Data length code mask               */
    SJA_FIR_RTR          = 1<<6, /* Remote transmission request         */
    SJA_FIR_EFF          = 1<<7  /* Extended frame format               */
};

/* Clock divider register */
enum SJA1000_PELI_CDR {
    SJA_CDR_CLKOUT_MASK  = 0x07,
    SJA_CDR_CLK_OFF      = 1<<3, /* Clock off (CLKOUT pin)              */
    SJA_CDR_CBP          = 1<<6, /* CAN input comparator bypass         */
    SJA_CDR_CAN_MODE     = 1<<7  /* CAN mode: 1 = PeliCAN               */
};

#endif  /* __SJA1000_REGS_H_ */
