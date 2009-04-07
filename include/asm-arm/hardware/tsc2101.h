/*
 *
 * TI TSC2101 Audio CODEC and TS control registers definition 
 *          
 *
 * Copyright 2003 MontaVista Software Inc.
 * Author: MontaVista Software, Inc.
 *	   source@mvista.com
 *
 *  This program is free software; you can redistribute	 it and/or modify it
 *  under  the terms of	 the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the	License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED	  ``AS	IS'' AND   ANY	EXPRESS OR IMPLIED
 *  WARRANTIES,	  INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO	EVENT  SHALL   THE AUTHOR  BE	 LIABLE FOR ANY	  DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED	  TO, PROCUREMENT OF  SUBSTITUTE GOODS	OR SERVICES; LOSS OF
 *  USE, DATA,	OR PROFITS; OR	BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN	 CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __ASM_HARDWARE_TSC2101_H
#define __ASM_HARDWARE_TSC2101_H

/* Page 0 Touch Screen Data Registers */
#define TSC2101_TS_X                  (0x00)
#define TSC2101_TS_Y                  (0x01)
#define TSC2101_TS_Z1                 (0x02)
#define TSC2101_TS_Z2                 (0x03)
#define TSC2101_TS_BAT                (0x05)
#define TSC2101_TS_AUX1               (0x07)
#define TSC2101_TS_AUX2               (0x08)
#define TSC2101_TS_TEMP1              (0x09)
#define TSC2101_TS_TEMP2              (0x0A)

/* Page 1 Touch Screen Control registers */
#define TSC2101_TS_ADC_CTRL           (0x00)
#define TSC2101_TS_STATUS             (0x01)
#define TSC2101_TS_BUFFER_CTRL        (0x02)
#define TSC2101_TS_REF_CTRL           (0x03)
#define TSC2101_TS_RESET_CTRL         (0x04)
#define TSC2101_TS_CONFIG_CTRL        (0x05)
#define TSC2101_TS_TEMP_MAX_THRESHOLD (0x06)
#define TSC2101_TS_TEMP_MIN_THRESHOLD (0x07)
#define TSC2101_TS_AUX1_MAX_THRESHOLD (0x08)
#define TSC2101_TS_AUX1_MIN_THRESHOLD (0x09)
#define TSC2101_TS_AUX2_MAX_THRESHOLD (0x0A)
#define TSC2101_TS_AUX2_MIN_THRESHOLD (0x0B)
#define TSC2101_TS_MEASURE_CONFIG     (0x0C)
#define TSC2101_TS_PROG_DELAY         (0x0D)

/* Page 2 Audio codec Control registers */
#define TSC2101_AUDIO_CTRL_1          (0x00)
#define TSC2101_HEADSET_GAIN_CTRL     (0x01)
#define TSC2101_DAC_GAIN_CTRL         (0x02)
#define TSC2101_MIXER_PGA_CTRL        (0x03)
#define TSC2101_AUDIO_CTRL_2          (0x04)
#define TSC2101_CODEC_POWER_CTRL      (0x05)
#define TSC2101_AUDIO_CTRL_3          (0x06)
#define TSC2101_LCH_BASS_BOOST_N0     (0x07)
#define TSC2101_LCH_BASS_BOOST_N1     (0x08)
#define TSC2101_LCH_BASS_BOOST_N2     (0x09)
#define TSC2101_LCH_BASS_BOOST_N3     (0x0A)
#define TSC2101_LCH_BASS_BOOST_N4     (0x0B)
#define TSC2101_LCH_BASS_BOOST_N5     (0x0C)
#define TSC2101_LCH_BASS_BOOST_D1     (0x0D)
#define TSC2101_LCH_BASS_BOOST_D2     (0x0E)
#define TSC2101_LCH_BASS_BOOST_D4     (0x0F)
#define TSC2101_LCH_BASS_BOOST_D5     (0x10)
#define TSC2101_RCH_BASS_BOOST_N0     (0x11)
#define TSC2101_RCH_BASS_BOOST_N1     (0x12)
#define TSC2101_RCH_BASS_BOOST_N2     (0x13)
#define TSC2101_RCH_BASS_BOOST_N3     (0x14)
#define TSC2101_RCH_BASS_BOOST_N4     (0x15)
#define TSC2101_RCH_BASS_BOOST_N5     (0x16)
#define TSC2101_RCH_BASS_BOOST_D1     (0x17)
#define TSC2101_RCH_BASS_BOOST_D2     (0x18)
#define TSC2101_RCH_BASS_BOOST_D4     (0x19)
#define TSC2101_RCH_BASS_BOOST_D5     (0x1A)
#define TSC2101_PLL_PROG_1            (0x1B)
#define TSC2101_PLL_PROG_2            (0x1C)
#define TSC2101_AUDIO_CTRL_4          (0x1D)
#define TSC2101_HANDSET_GAIN_CTRL     (0x1E)
#define TSC2101_BUZZER_GAIN_CTRL      (0x1F)
#define TSC2101_AUDIO_CTRL_5          (0x20)
#define TSC2101_AUDIO_CTRL_6          (0x21)
#define TSC2101_AUDIO_CTRL_7          (0x22)
#define TSC2101_GPIO_CTRL             (0x23)
#define TSC2101_AGC_CTRL              (0x24)
#define TSC2101_POWERDOWN_STS         (0x25)
#define TSC2101_MIC_AGC_CONTROL       (0x26)
#define TSC2101_CELL_AGC_CONTROL      (0x27)

/* Bit field definitions for TS Control */
#define TSC2101_DATA_AVAILABLE         0x4000
#define TSC2101_BUFFERMODE_DISABLE     0x0
#define TSC2101_REF_POWERUP            0x16
#define TSC2101_ENABLE_TOUCHDETECT     0x08
#define TSC2101_PRG_DELAY              0x0900
#define TSC2101_ADC_CONTROL            0x8874
#define TSC2101_ADC_POWERDOWN          0x4000

/* Bit position */
#define TSC2101_BIT(ARG)    ((0x01)<<(ARG))

/* Field masks for Audio Control 1 */
#define AC1_ADCHPF(ARG)     (((ARG) & 0x03) << 14)
#define AC1_WLEN(ARG)       (((ARG) & 0x03) << 10)
#define AC1_DATFM(ARG)      (((ARG) & 0x03) << 8)
#define AC1_DACFS(ARG)      (((ARG) & 0x07) << 3)
#define AC1_ADCFS(ARG)      (((ARG) & 0x07))

/* Field masks for TSC2101_HEADSET_GAIN_CTRL */
#define HGC_ADMUT_HED       TSC2101_BIT(15)
#define HGC_ADPGA_HED(ARG)  (((ARG) & 0x7F) << 8)
#define HGC_AGCTG_HED(ARG)  (((ARG) & 0x07) << 5)
#define HGC_AGCTC_HED(ARG)  (((ARG) & 0x0F) << 1)
#define HGC_AGCEN_HED       (0x01)

/* Field masks for TSC2101_DAC_GAIN_CTRL */
#define DGC_DALMU           TSC2101_BIT(15)
#define DGC_DALVL(ARG)      (((ARG) & 0x7F) << 8)
#define DGC_DARMU           TSC2101_BIT(7)
#define DGC_DARVL(ARG)      (((ARG) & 0x7F))

/* Field masks for TSC2101_MIXER_PGA_CTRL */
#define MPC_ASTMU           TSC2101_BIT(15)
#define MPC_ASTG(ARG)       (((ARG) & 0x7F) << 8)
#define MPC_MICSEL(ARG)     (((ARG) & 0x07) << 5)
#define MPC_MICADC          TSC2101_BIT(4)
#define MPC_CPADC           TSC2101_BIT(3)
#define MPC_ASTGF           (0x01)

/* Field formats for TSC2101_AUDIO_CTRL_2 */
#define AC2_KCLEN           TSC2101_BIT(15)
#define AC2_KCLAC(ARG)      (((ARG) & 0x07) << 12)
#define AC2_APGASS          TSC2101_BIT(11)
#define AC2_KCLFRQ(ARG)     (((ARG) & 0x07) << 8)
#define AC2_KCLLN(ARG)      (((ARG) & 0x0F) << 4)
#define AC2_DLGAF           TSC2101_BIT(3)
#define AC2_DRGAF           TSC2101_BIT(2)
#define AC2_DASTC           TSC2101_BIT(1)
#define AC2_ADGAF           (0x01)

/* Field masks for TSC2101_CODEC_POWER_CTRL */
#define CPC_MBIAS_HND       TSC2101_BIT(15)
#define CPC_MBIAS_HED       TSC2101_BIT(14)
#define CPC_ASTPWD          TSC2101_BIT(13)
#define CPC_SP1PWDN         TSC2101_BIT(12)
#define CPC_SP2PWDN         TSC2101_BIT(11)
#define CPC_DAPWDN          TSC2101_BIT(10)
#define CPC_ADPWDN          TSC2101_BIT(9)
#define CPC_VGPWDN          TSC2101_BIT(8)
#define CPC_COPWDN          TSC2101_BIT(7)
#define CPC_LSPWDN          TSC2101_BIT(6)
#define CPC_ADPWDF          TSC2101_BIT(5)
#define CPC_LDAPWDF         TSC2101_BIT(4)
#define CPC_RDAPWDF         TSC2101_BIT(3)
#define CPC_ASTPWF          TSC2101_BIT(2)
#define CPC_BASSBC          TSC2101_BIT(1)
#define CPC_DEEMPF          (0x01)

/* Field masks for TSC2101_AUDIO_CTRL_3 */
#define AC3_DMSVOL(ARG)     (((ARG) & 0x03) << 14)
#define AC3_REFFS           TSC2101_BIT(13)
#define AC3_DAXFM           TSC2101_BIT(12)
#define AC3_SLVMS           TSC2101_BIT(11)
#define AC3_ADCOVF          TSC2101_BIT(8)
#define AC3_DALOVF          TSC2101_BIT(7)
#define AC3_DAROVF          TSC2101_BIT(6)
#define AC3_CLPST           TSC2101_BIT(3)
#define AC3_REVID(ARG)      (((ARG) & 0x07))

/* Field masks for TSC2101_PLL_PROG_1 */
#define PLL1_PLLSEL         TSC2101_BIT(15)
#define PLL1_QVAL(ARG)      (((ARG) & 0x0F) << 11)
#define PLL1_PVAL(ARG)      (((ARG) & 0x07) << 8)
#define PLL1_I_VAL(ARG)     (((ARG) & 0x3F) << 2)

/* Field masks of TSC2101_PLL_PROG_2 */
#define PLL2_D_VAL(ARG)     (((ARG) & 0x3FFF) << 2)

/* Field masks for TSC2101_AUDIO_CTRL_4 */
#define AC4_ADSTPD          TSC2101_BIT(15)
#define AC4_DASTPD          TSC2101_BIT(14)
#define AC4_ASSTPD          TSC2101_BIT(13)
#define AC4_CISTPD          TSC2101_BIT(12)
#define AC4_BISTPD          TSC2101_BIT(11)
#define AC4_AGCHYS(ARG)     (((ARG) & 0x03) << 9)
#define AC4_MB_HED(ARG)     (((ARG) & 0x03) << 7)
#define AC4_MB_HND          TSC2101_BIT(6)
#define AC4_SCPFL           TSC2101_BIT(1)

/* Field masks settings for TSC2101_HANDSET_GAIN_CTRL */
#define HNGC_ADMUT_HND      TSC2101_BIT(15)
#define HNGC_ADPGA_HND(ARG) (((ARG) & 0x7F) << 8)
#define HNGC_AGCTG_HND(ARG) (((ARG) & 0x07) << 5)
#define HNGC_AGCTC_HND(ARG) (((ARG) & 0x0F) << 1)
#define HNGC_AGCEN_HND      (0x01)

/* Field masks settings for TSC2101_BUZZER_GAIN_CTRL */
#define BGC_MUT_CP          TSC2101_BIT(15)
#define BGC_CPGA(ARG)       (((ARG) & 0x7F) << 8)
#define BGC_CPGF            TSC2101_BIT(7)
#define BGC_MUT_BU          TSC2101_BIT(6)
#define BGC_BPGA(ARG)       (((ARG) & 0x0F) << 2)
#define BGC_BUGF            TSC2101_BIT(1)

/* Field masks settings for TSC2101_AUDIO_CTRL_5 */
#define AC5_DIFFIN          TSC2101_BIT(15)
#define AC5_DAC2SPK1(ARG)   (((ARG) & 0x03) << 13)
#define AC5_AST2SPK1        TSC2101_BIT(12)
#define AC5_BUZ2SPK1        TSC2101_BIT(11)
#define AC5_KCL2SPK1        TSC2101_BIT(10)
#define AC5_CPI2SPK1        TSC2101_BIT(9)
#define AC5_DAC2SPK2(ARG)   (((ARG) & 0x03) << 7)
#define AC5_AST2SPK2        TSC2101_BIT(6)
#define AC5_BUZ2SPK2        TSC2101_BIT(5)
#define AC5_KCL2SPK2        TSC2101_BIT(4)
#define AC5_CPI2SPK2        TSC2101_BIT(3)
#define AC5_MUTSPK1         TSC2101_BIT(2)
#define AC5_MUTSPK2         TSC2101_BIT(1)
#define AC5_HDSCPTC         (0x01)

/* Field masks settings for TSC2101_AUDIO_CTRL_6 */
#define AC6_SPL2LSK         TSC2101_BIT(15)
#define AC6_AST2LSK         TSC2101_BIT(14)
#define AC6_BUZ2LSK         TSC2101_BIT(13)
#define AC6_KCL2LSK         TSC2101_BIT(12)
#define AC6_CPI2LSK         TSC2101_BIT(11)
#define AC6_MIC2CPO         TSC2101_BIT(10)
#define AC6_SPL2CPO         TSC2101_BIT(9)
#define AC6_SPR2CPO         TSC2101_BIT(8)
#define AC6_MUTLSPK         TSC2101_BIT(7)
#define AC6_MUTSPK2         TSC2101_BIT(6)
#define AC6_LDSCPTC         TSC2101_BIT(5)
#define AC6_VGNDSCPTC       TSC2101_BIT(4)
#define AC6_CAPINTF         TSC2101_BIT(3)

/* Field masks settings for TSC2101_AUDIO_CTRL_7 */
#define AC7_DETECT          TSC2101_BIT(15)
#define AC7_HESTYPE(ARG)    (((ARG) & 0x03) << 13)
#define AC7_HDDETFL         TSC2101_BIT(12)
#define AC7_BDETFL          TSC2101_BIT(11)
#define AC7_HDDEBNPG(ARG)   (((ARG) & 0x03) << 9)
#define AC7_BDEBNPG(ARG)    (((ARG) & 0x03) << 6)
#define AC7_DGPIO2          TSC2101_BIT(4)
#define AC7_DGPIO1          TSC2101_BIT(3)
#define AC7_CLKGPIO2        TSC2101_BIT(2)
#define AC7_ADWSF(ARG)      (((ARG) & 0x03))

/* Field masks settings for TSC2101_GPIO_CTRL */
#define GC_GPO2EN           TSC2101_BIT(15)
#define GC_GPO2SG           TSC2101_BIT(14)
#define GC_GPI2EN           TSC2101_BIT(13)
#define GC_GPI2SGF          TSC2101_BIT(12)
#define GC_GPO1EN           TSC2101_BIT(11)
#define GC_GPO1SG           TSC2101_BIT(10)
#define GC_GPI1EN           TSC2101_BIT(9)
#define GC_GPI1SGF          TSC2101_BIT(8)

/* Field masks for TSC2101_AGC_CTRL */
#define AC_AGCNF_CELL       TSC2101_BIT(14)
#define AC_AGCNL(ARG)       (((ARG) & 0x07) << 11)
#define AC_AGCHYS_CELL(ARG) (((ARG) & 0x03) << 9)
#define AC_CLPST_CELL       TSC2101_BIT(8)
#define AC_AGCTG_CELL(ARG)  (((ARG) & 0x07) << 5)
#define AC_AGCTC_CELL(ARG)  (((ARG) & 0x0F) << 1)
#define AC_AGCEN_CELL       (0x01)

/* Field masks for TSC2101_POWERDOWN_STS */
#define PS_SPK1FL            TSC2101_BIT(15)
#define PS_SPK2FL            TSC2101_BIT(14)
#define PS_HNDFL             TSC2101_BIT(13)
#define PS_VGNDFL            TSC2101_BIT(12)
#define PS_LSPKFL            TSC2101_BIT(11)
#define PS_CELLFL            TSC2101_BIT(10)
#define PS_PSEQ              TSC2101_BIT(5)
#define PS_PSTIME            TSC2101_BIT(4)

/* Field masks for Register Mic AGC Control */
#define MAC_MMPGA(ARG)       (((ARG) & 0x7F) << 9)
#define MAC_MDEBNS(ARG)      (((ARG) & 0x07) << 6)
#define MAC_MDEBSN(ARG)      (((ARG) & 0x07) << 3)

/* Field masks for Register Cellphone AGC Control */
#define CAC_CMPGA(ARG)       (((ARG) & 0x7F) << 9)
#define CAC_CDEBNS(ARG)      (((ARG) & 0x07) << 6)
#define CAC_CDEBSN(ARG)      (((ARG) & 0x07) << 3)

#endif				/* __ASM_HARDWARE_TSC2101_H */
