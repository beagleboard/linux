#ifndef _DW_HDMI_PHY_GEN2_H_
#define _DW_HDMI_PHY_GEN2_H_

#include <linux/bits.h>
#include <linux/bitfield.h>
#include <linux/kernel.h>
#include <drm/bridge/dw_hdmi.h>

/* PHY registers */
#define DW_HDMI_TX_PHY_GEN2_PWRCTRL			0x00
#define DW_HDMI_TX_PHY_GEN2_SERDIVCTRL			0x01
#define DW_HDMI_TX_PHY_GEN2_SERCKCTRL			0x02
#define DW_HDMI_TX_PHY_GEN2_SERCKKILLCTRL		0x03
#define DW_HDMI_TX_PHY_GEN2_TXRESCALCTRL		0x04
#define DW_HDMI_TX_PHY_GEN2_OPMODE_PLLCFG		0x06
#define DW_HDMI_TX_PHY_GEN2_TXCKMEASCTRL		0x07
#define DW_HDMI_TX_PHY_GEN2_TXMEASCTRL			0x08
#define DW_HDMI_TX_PHY_GEN2_CKSYMTXCTRL			0x09
#define DW_HDMI_TX_PHY_GEN2_CMPSEQCTRL			0x0A
#define DW_HDMI_TX_PHY_GEN2_CMPPWRCTRL			0x0B
#define DW_HDMI_TX_PHY_GEN2_CMPMODECTRL			0x0C
#define DW_HDMI_TX_PHY_GEN2_MEASCTRL			0x0D
#define DW_HDMI_TX_PHY_GEN2_VLEVCTRL_PLLMEASCTRL	0x0E
#define DW_HDMI_TX_PHY_GEN2_D2ACTRL			0x0F
#define DW_HDMI_TX_PHY_GEN2_PLLCURRGMPCTRL		0x10
#define DW_HDMI_TX_PHY_GEN2_PLLDIVCTRL			0x11
#define DW_HDMI_TX_PHY_GEN2_SUPPLLBGCTRL		0x12
#define DW_HDMI_TX_PHY_GEN2_PLLCONFCTRL			0x13
#define DW_HDMI_TX_PHY_GEN2_PLLCTRL			0x14
#define DW_HDMI_TX_PHY_GEN2_PLLDIGTESTCTRL		0x15
#define DW_HDMI_TX_PHY_GEN2_PLLMEASCTRL			0x16
#define DW_HDMI_TX_PHY_GEN2_PLLCLKBISTPHASE		0x17
#define DW_HDMI_TX_PHY_GEN2_COMPRCAL			0x18
#define DW_HDMI_TX_PHY_GEN2_TXTERM			0x19
#define DW_HDMI_TX_PHY_GEN2_PWRSEQ_PATGENSKIP		0x1A
#define DW_HDMI_TX_PHY_GEN2_PATTERNGEN_BISTSEL		0x1B
#define DW_HDMI_TX_PHY_GEN2_CTRL_CLKALIGN_PGMODE_BISTPG	0x1C
#define DW_HDMI_TX_PHY_GEN2_BISTCHSTATUS_CLKALIGN_RCAL	0x1D
#define DW_HDMI_TX_PHY_GEN2_DIGTXMODE			0x1E
#define DW_HDMI_TX_PHY_GEN2_CLKALIGN_RCAL_SOFTRESET	0x1F
#define DW_HDMI_TX_PHY_GEN2_PWRSEQ_PATGENSKIP_RD	0x2E
#define DW_HDMI_TX_PHY_GEN2_PLLCLKBISTPHASE_RD		0x2F
#define DW_HDMI_TX_PHY_GEN2_PLLSTATUS			0x30
#define DW_HDMI_TX_PHY_GEN2_SUPPLLBGSTATUS		0x31
#define DW_HDMI_TX_PHY_GEN2_D2ASTATUS			0x32
#define DW_HDMI_TX_PHY_GEN2_CMPMODESTATUS		0x33
#define DW_HDMI_TX_PHY_GEN2_CMPPWRSTATUS		0x34
#define DW_HDMI_TX_PHY_GEN2_CMPSEQSTATUS		0x35
#define DW_HDMI_TX_PHY_GEN2_CKSYMTXSTATUS		0x36
#define DW_HDMI_TX_PHY_GEN2_TXRESCALSTATUS		0x37
#define DW_HDMI_TX_PHY_GEN2_SERCKKILLSTATUS		0x38
#define DW_HDMI_TX_PHY_GEN2_SERCKSTATUS			0x39
#define DW_HDMI_TX_PHY_GEN2_SERDIVSTATUS		0x3A
#define DW_HDMI_TX_PHY_GEN2_PWRSTATUS			0x3B

/* PHY register bitfields */
#define PWRCTRL_OVERRIDE_0				BIT(15)
#define PWRCTRL_BIAS_ON					BIT(5)
#define PWRCTRL_TX_PWRON				BIT(4)
#define PWRCTRL_TX_PWRON0				BIT(3)
#define PWRCTRL_TX_PWRON1				BIT(2)
#define PWRCTRL_TX_PWRON2				BIT(1)
#define PWRCTRL_CK_PWRON				BIT(0)

#define OPMODE_PLLCFG_PREP_DIV				GENMASK(13, 12)
#define OPMODE_PLLCFG_MPLL_CKO_DIV			GENMASK(10,  9)
#define OPMODE_PLLCFG_OPMODE				GENMASK( 7,  6)
#define OPMODE_PLLCFG_REF_CNTRL				GENMASK( 4,  3)
#define OPMODE_PLLCFG_MPLL_N_CNTRL			GENMASK( 1,  0)

#define CKSYMTXCTRL_OVERRIDE_5				BIT(15)
#define CKSYMTXCTRL_SLOPE_BOOST				GENMASK(13, 10)
#define CKSYMTXCTRL_TX_TRAON				BIT(9)
#define CKSYMTXCTRL_TX_TRBON				BIT(8)
#define CKSYMTXCTRL_TX_SYMON				GENMASK( 7,  4)
#define CKSYMTXCTRL_CK_SYMON				GENMASK( 3,  0)

#define VLEVCTRL_PLLMEASCTRL_MPLL_MEAS_17_14		GENMASK(13, 10)
#define VLEVCTRL_PLLMEASCTRL_SUP_TX_LVL			GENMASK( 9,  5)

#define PLLCURRGMPCTRL_MPLL_GMP_CNTRL			GENMASK(13, 12)
#define PLLCURRGMPCTRL_MPLL_PROP_CNTRL			GENMASK(11,  6)
#define PLLCURRGMPCTRL_MPLL_INT_CNTRL			GENMASK( 5,  0)

#define PLLDIVCTRL_VCO_CNTRL				GENMASK(12,  9)
#define PLLDIVCTRL_MPLL_MULTIPLIER			GENMASK( 7,  0)

#define TXTERM_D_TX_TERM				GENMASK( 2,  0)

#define TXTERM_50_OHM					0x0
#define TXTERM_57_14_OHM				0x1
#define TXTERM_66_67_OHM				0x2
#define TXTERM_80_OHM					0x3
#define TXTERM_100_OHM					0x4
#define TXTERM_133_33_OHM				0x5
#define TXTERM_200_OHM					0x6
#define TXTERM_OPEN_CIRCUIT				0x7

#define OPMODE_HDMI_14		0
#define OPMODE_HDMI_20		1

struct dw_hdmi_mpll_gen_config{
	u64 pixelclock;		/* in kHz */
	u8  colordepth;
	u16 opmode:2;
	struct {
		u16 prep_div:2;
		u16 mpll_cko_div:2;
		u16 ref_cntrl:2;
		u16 mpll_n_cntrl:2;
		u16 vco_cntrl:4;
		u16 mpll_multiplier:8;
	} divider;
	struct {
		u16 gmp_cntrl:2;
		u16 prop_cntrl:6;
		u16 int_cntrl:6;
	} charge_pump;
	struct {
		u16 txterm:3;
		u16 sup_txlvl:5;
		u16 tx_traon:1;
		u16 tx_trbon:1;
		u16 tx_symon:4;
		u16 ck_symon:4;
	} voltage;
};

static const struct dw_hdmi_mpll_gen_config mpll_configs[] = {
	{
		.pixelclock = 25175,
		.colordepth = 8,
		.opmode = OPMODE_HDMI_14,
		.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x3, .vco_cntrl = 0x3, .mpll_multiplier = 0x28, },
		.charge_pump = { .gmp_cntrl = 0x0, .prop_cntrl = 0xa, .int_cntrl = 0x3, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 27000,
		.colordepth = 8,
		.opmode = OPMODE_HDMI_14,
		.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x3, .vco_cntrl = 0x3, .mpll_multiplier = 0x28, },
		.charge_pump = { .gmp_cntrl = 0x0, .prop_cntrl = 0xa, .int_cntrl = 0x3, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 31500,
		.colordepth = 8,
		.opmode = OPMODE_HDMI_14,
		.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x3, .vco_cntrl = 0x3, .mpll_multiplier = 0x28, },
		.charge_pump = { .gmp_cntrl = 0x0, .prop_cntrl = 0xa, .int_cntrl = 0x3, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 33750,
		.colordepth = 8,
		.opmode = OPMODE_HDMI_14,
		.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x3, .vco_cntrl = 0x3, .mpll_multiplier = 0x28, },
		.charge_pump = { .gmp_cntrl = 0x0, .prop_cntrl = 0xa, .int_cntrl = 0x3, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 35500,
		.colordepth = 8,
		.opmode = OPMODE_HDMI_14,
		.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x3, .vco_cntrl = 0x3, .mpll_multiplier = 0x28, },
		.charge_pump = { .gmp_cntrl = 0x0, .prop_cntrl = 0xa, .int_cntrl = 0x3, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 36000,
		.colordepth = 8,
		.opmode = OPMODE_HDMI_14,
		.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x3, .vco_cntrl = 0x1, .mpll_multiplier = 0x28, },
		.charge_pump = { .gmp_cntrl = 0x0, .prop_cntrl = 0xa, .int_cntrl = 0x5, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 40000,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_14,
		.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x3, .vco_cntrl = 0x1, .mpll_multiplier = 0x28, },
		.charge_pump = { .gmp_cntrl = 0x0, .prop_cntrl = 0xa, .int_cntrl = 0x5, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 44900,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_14,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x3, .vco_cntrl = 0x1, .mpll_multiplier = 0x28, },
		.charge_pump = { .gmp_cntrl = 0x0, .prop_cntrl = 0xa, .int_cntrl = 0x5, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 49500,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_14,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x2, .vco_cntrl = 0x3, .mpll_multiplier = 0x14, },
		.charge_pump = { .gmp_cntrl = 0x1, .prop_cntrl = 0x6, .int_cntrl = 0x3, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 50000,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_14,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x2, .vco_cntrl = 0x3, .mpll_multiplier = 0x14, },
		.charge_pump = { .gmp_cntrl = 0x1, .prop_cntrl = 0x6, .int_cntrl = 0x3, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 50350,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_14,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x2, .vco_cntrl = 0x3, .mpll_multiplier = 0x14, },
		.charge_pump = { .gmp_cntrl = 0x1, .prop_cntrl = 0x6, .int_cntrl = 0x3, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 54000,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_14,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x2, .vco_cntrl = 0x3, .mpll_multiplier = 0x14, },
		.charge_pump = { .gmp_cntrl = 0x1, .prop_cntrl = 0x6, .int_cntrl = 0x3, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 56250,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_14,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x2, .vco_cntrl = 0x3, .mpll_multiplier = 0x14, },
		.charge_pump = { .gmp_cntrl = 0x1, .prop_cntrl = 0x6, .int_cntrl = 0x3, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 59400,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_14,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x2, .vco_cntrl = 0x3, .mpll_multiplier = 0x14, },
		.charge_pump = { .gmp_cntrl = 0x1, .prop_cntrl = 0x6, .int_cntrl = 0x3, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 65000,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_14,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x2, .vco_cntrl = 0x3, .mpll_multiplier = 0x14, },
		.charge_pump = { .gmp_cntrl = 0x1, .prop_cntrl = 0x6, .int_cntrl = 0x3, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 68250,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_14,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x2, .vco_cntrl = 0x3, .mpll_multiplier = 0x14, },
		.charge_pump = { .gmp_cntrl = 0x1, .prop_cntrl = 0x6, .int_cntrl = 0x3, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 71000,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_14,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x2, .vco_cntrl = 0x3, .mpll_multiplier = 0x14, },
		.charge_pump = { .gmp_cntrl = 0x1, .prop_cntrl = 0x6, .int_cntrl = 0x3, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 72000,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_14,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x2, .vco_cntrl = 0x1, .mpll_multiplier = 0x14, },
		.charge_pump = { .gmp_cntrl = 0x1, .prop_cntrl = 0x5, .int_cntrl = 0x2, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 73250,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_14,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x2, .vco_cntrl = 0x1, .mpll_multiplier = 0x14, },
		.charge_pump = { .gmp_cntrl = 0x1, .prop_cntrl = 0x5, .int_cntrl = 0x2, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 74250,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_14,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x2, .vco_cntrl = 0x1, .mpll_multiplier = 0x14, },
		.charge_pump = { .gmp_cntrl = 0x1, .prop_cntrl = 0x5, .int_cntrl = 0x2, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 75000,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_14,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x2, .vco_cntrl = 0x1, .mpll_multiplier = 0x14, },
		.charge_pump = { .gmp_cntrl = 0x1, .prop_cntrl = 0x5, .int_cntrl = 0x2, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 78750,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_14,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x2, .vco_cntrl = 0x1, .mpll_multiplier = 0x14, },
		.charge_pump = { .gmp_cntrl = 0x1, .prop_cntrl = 0x5, .int_cntrl = 0x2, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 79500,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_14,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x2, .vco_cntrl = 0x1, .mpll_multiplier = 0x14, },
		.charge_pump = { .gmp_cntrl = 0x1, .prop_cntrl = 0x5, .int_cntrl = 0x2, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 82500,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_14,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x2, .vco_cntrl = 0x1, .mpll_multiplier = 0x14, },
		.charge_pump = { .gmp_cntrl = 0x1, .prop_cntrl = 0x5, .int_cntrl = 0x2, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 83500,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_14,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x2, .vco_cntrl = 0x1, .mpll_multiplier = 0x14, },
		.charge_pump = { .gmp_cntrl = 0x1, .prop_cntrl = 0x5, .int_cntrl = 0x2, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 85500,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_14,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x2, .vco_cntrl = 0x1, .mpll_multiplier = 0x14, },
		.charge_pump = { .gmp_cntrl = 0x1, .prop_cntrl = 0x5, .int_cntrl = 0x2, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 88750,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_14,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x2, .vco_cntrl = 0x1, .mpll_multiplier = 0x14, },
		.charge_pump = { .gmp_cntrl = 0x1, .prop_cntrl = 0x5, .int_cntrl = 0x2, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 90000,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_14,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x2, .vco_cntrl = 0x1, .mpll_multiplier = 0x14, },
		.charge_pump = { .gmp_cntrl = 0x1, .prop_cntrl = 0x5, .int_cntrl = 0x2, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 94500,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_14,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x1, .vco_cntrl = 0x3, .mpll_multiplier = 0xa, },
		.charge_pump = { .gmp_cntrl = 0x2, .prop_cntrl = 0x3, .int_cntrl = 0x0, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 99000,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_14,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x1, .vco_cntrl = 0x3, .mpll_multiplier = 0xa, },
		.charge_pump = { .gmp_cntrl = 0x2, .prop_cntrl = 0x3, .int_cntrl = 0x0, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 100700,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_14,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x1, .vco_cntrl = 0x3, .mpll_multiplier = 0xa, },
		.charge_pump = { .gmp_cntrl = 0x2, .prop_cntrl = 0x3, .int_cntrl = 0x0, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 101000,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_14,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x1, .vco_cntrl = 0x3, .mpll_multiplier = 0xa, },
		.charge_pump = { .gmp_cntrl = 0x2, .prop_cntrl = 0x3, .int_cntrl = 0x0, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 102250,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_14,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x1, .vco_cntrl = 0x3, .mpll_multiplier = 0xa, },
		.charge_pump = { .gmp_cntrl = 0x2, .prop_cntrl = 0x3, .int_cntrl = 0x0, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 106500,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_14,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x1, .vco_cntrl = 0x3, .mpll_multiplier = 0xa, },
		.charge_pump = { .gmp_cntrl = 0x2, .prop_cntrl = 0x3, .int_cntrl = 0x0, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 108000,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_14,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x1, .vco_cntrl = 0x3, .mpll_multiplier = 0xa, },
		.charge_pump = { .gmp_cntrl = 0x2, .prop_cntrl = 0x3, .int_cntrl = 0x0, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 115500,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_14,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x1, .vco_cntrl = 0x3, .mpll_multiplier = 0xa, },
		.charge_pump = { .gmp_cntrl = 0x2, .prop_cntrl = 0x3, .int_cntrl = 0x0, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 117500,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_14,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x1, .vco_cntrl = 0x3, .mpll_multiplier = 0xa, },
		.charge_pump = { .gmp_cntrl = 0x2, .prop_cntrl = 0x3, .int_cntrl = 0x0, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 118800,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_14,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x1, .vco_cntrl = 0x3, .mpll_multiplier = 0xa, },
		.charge_pump = { .gmp_cntrl = 0x2, .prop_cntrl = 0x3, .int_cntrl = 0x0, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 119000,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_14,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x1, .vco_cntrl = 0x3, .mpll_multiplier = 0xa, },
		.charge_pump = { .gmp_cntrl = 0x2, .prop_cntrl = 0x3, .int_cntrl = 0x0, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 121750,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_14,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x1, .vco_cntrl = 0x3, .mpll_multiplier = 0xa, },
		.charge_pump = { .gmp_cntrl = 0x2, .prop_cntrl = 0x3, .int_cntrl = 0x0, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 148500,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_14,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x1, .vco_cntrl = 0x1, .mpll_multiplier = 0xa, },
		.charge_pump = { .gmp_cntrl = 0x2, .prop_cntrl = 0x2, .int_cntrl = 0x0, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 154000,
		.colordepth = 8,
		.opmode = OPMODE_HDMI_14,
		.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x1, .vco_cntrl = 0x1, .mpll_multiplier = 0xa, },
		.charge_pump = { .gmp_cntrl = 0x2, .prop_cntrl = 0x2, .int_cntrl = 0x0, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 162000,
		.colordepth = 8,
		.opmode = OPMODE_HDMI_14,
		.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x1, .vco_cntrl = 0x1, .mpll_multiplier = 0xa, },
		.charge_pump = { .gmp_cntrl = 0x2, .prop_cntrl = 0x2, .int_cntrl = 0x0, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 165000,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_14,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x1, .vco_cntrl = 0x1, .mpll_multiplier = 0xa, },
		.charge_pump = { .gmp_cntrl = 0x2, .prop_cntrl = 0x2, .int_cntrl = 0x0, },
		.voltage = { .txterm = TXTERM_OPEN_CIRCUIT, .sup_txlvl = 0xd, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0x8, .ck_symon = 0x8, },
	}, {
		.pixelclock = 185625,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_14,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x0, .vco_cntrl = 0x3, .mpll_multiplier = 0x5, },
		.charge_pump = { .gmp_cntrl = 0x3, .prop_cntrl = 0x1, .int_cntrl = 0x0, },
		.voltage = { .txterm = TXTERM_100_OHM, .sup_txlvl = 0x9, .tx_traon = 0x1,
			     .tx_trbon = 0x1, .tx_symon = 0xc, .ck_symon = 0x8, },
	}, {
		.pixelclock = 198000,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_14,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x0, .vco_cntrl = 0x3, .mpll_multiplier = 0x5, },
		.charge_pump = { .gmp_cntrl = 0x3, .prop_cntrl = 0x1, .int_cntrl = 0x0, },
		.voltage = { .txterm = TXTERM_100_OHM, .sup_txlvl = 0x9, .tx_traon = 0x1,
			     .tx_trbon = 0x1, .tx_symon = 0xc, .ck_symon = 0x8, },
	}, {
		.pixelclock = 297000,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_14,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x0, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x0, .vco_cntrl = 0x1, .mpll_multiplier = 0x5, },
		.charge_pump = { .gmp_cntrl = 0x3, .prop_cntrl = 0x1, .int_cntrl = 0x1, },
		.voltage = { .txterm = TXTERM_133_33_OHM, .sup_txlvl = 0x10, .tx_traon = 0x0,
			     .tx_trbon = 0x1, .tx_symon = 0xd, .ck_symon = 0xc, },
	}, {
		.pixelclock = 371250,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_20,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x3, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x0, .vco_cntrl = 0x1, .mpll_multiplier = 0x5, },
		.charge_pump = { .gmp_cntrl = 0x3, .prop_cntrl = 0x1, .int_cntrl = 0x1, },
		.voltage = { .txterm = TXTERM_50_OHM, .sup_txlvl = 0xa, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0xf, .ck_symon = 0x6, },
	}, {
		.pixelclock = 495000,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_20,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x3, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x0, .vco_cntrl = 0x0, .mpll_multiplier = 0x5, },
		.charge_pump = { .gmp_cntrl = 0x3, .prop_cntrl = 0x2, .int_cntrl = 0x0, },
		.voltage = { .txterm = TXTERM_50_OHM, .sup_txlvl = 0xa, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0xf, .ck_symon = 0x6, },
	}, {
		.pixelclock = 594000,
	  	.colordepth = 8,
	  	.opmode = OPMODE_HDMI_20,
	  	.divider = { .prep_div = 0x0, .mpll_cko_div = 0x3, .ref_cntrl = 0x0,
			     .mpll_n_cntrl = 0x0, .vco_cntrl = 0x0, .mpll_multiplier = 0x5, },
		.charge_pump = { .gmp_cntrl = 0x3, .prop_cntrl = 0x2, .int_cntrl = 0x0, },
		.voltage = { .txterm = TXTERM_100_OHM, .sup_txlvl = 0xf, .tx_traon = 0x0,
			     .tx_trbon = 0x0, .tx_symon = 0xf, .ck_symon = 0xa, },
	},
};

static enum drm_mode_status
dw_hdmi_tx_phy_gen2_mode_valid(struct dw_hdmi *dw_hdmi, void *data,
			       const struct drm_display_info *info,
			       const struct drm_display_mode *mode)
{
	int i;

        if (mode->clock < 13500)
                return MODE_CLOCK_LOW;
        else if (mode->clock > 594000)
                return MODE_CLOCK_HIGH;

	for (i = 0; i < ARRAY_SIZE(mpll_configs); i++) {
		if (abs(mode->clock - mpll_configs[i].pixelclock) <= 100)
			return MODE_OK;
		else if (mode->clock < mpll_configs[i].pixelclock)
			break;
	}

	return MODE_NOMODE;
}

static int dw_hdmi_tx_phy_gen2_configure(struct dw_hdmi *hdmi, void *data,
					 unsigned long mpixelclock)
{
	int i;
	u16 opmode_pllcfg, pllcurrgmpctrl, plldivctrl;
	u16 txterm, vlevctrl_pllmeasctrl, cksymtxctrl;
	u64 pixclk = mpixelclock / 1000;	
	const struct dw_hdmi_mpll_gen_config *config;

	for (i = 0; i < ARRAY_SIZE(mpll_configs); i++) {
		config = &mpll_configs[i];
		/* TODO: add colordepth check later */
		if (abs(config->pixelclock - pixclk) <= 100)
			break;
	}

	if (i == ARRAY_SIZE(mpll_configs))
		return -1;

	opmode_pllcfg = FIELD_PREP(OPMODE_PLLCFG_PREP_DIV, config->divider.prep_div)		|
			FIELD_PREP(OPMODE_PLLCFG_MPLL_CKO_DIV, config->divider.mpll_cko_div)	|
			FIELD_PREP(OPMODE_PLLCFG_OPMODE, config->opmode)			|
			FIELD_PREP(OPMODE_PLLCFG_REF_CNTRL, config->divider.ref_cntrl)		|
			FIELD_PREP(OPMODE_PLLCFG_MPLL_N_CNTRL, config->divider.mpll_n_cntrl);
	dw_hdmi_phy_i2c_write(
		hdmi,
		opmode_pllcfg,
		DW_HDMI_TX_PHY_GEN2_OPMODE_PLLCFG
	);

	pllcurrgmpctrl = FIELD_PREP(PLLCURRGMPCTRL_MPLL_GMP_CNTRL, config->charge_pump.gmp_cntrl)	|
			 FIELD_PREP(PLLCURRGMPCTRL_MPLL_PROP_CNTRL, config->charge_pump.prop_cntrl)	|
			 FIELD_PREP(PLLCURRGMPCTRL_MPLL_INT_CNTRL, config->charge_pump.int_cntrl);
	dw_hdmi_phy_i2c_write(
		hdmi,
		pllcurrgmpctrl,
		DW_HDMI_TX_PHY_GEN2_PLLCURRGMPCTRL
	);

	plldivctrl = FIELD_PREP(PLLDIVCTRL_VCO_CNTRL, config->divider.vco_cntrl)	|
		     FIELD_PREP(PLLDIVCTRL_MPLL_MULTIPLIER, config->divider.mpll_multiplier);
	dw_hdmi_phy_i2c_write(
		hdmi,
		plldivctrl,
		DW_HDMI_TX_PHY_GEN2_PLLDIVCTRL
	);

	txterm = FIELD_PREP(TXTERM_D_TX_TERM, config->voltage.txterm);
	dw_hdmi_phy_i2c_write(
		hdmi,
		txterm,
		DW_HDMI_TX_PHY_GEN2_TXTERM
	);

	vlevctrl_pllmeasctrl = FIELD_PREP(VLEVCTRL_PLLMEASCTRL_SUP_TX_LVL, config->voltage.sup_txlvl);
	dw_hdmi_phy_i2c_write(
		hdmi,
		vlevctrl_pllmeasctrl,
		DW_HDMI_TX_PHY_GEN2_VLEVCTRL_PLLMEASCTRL
	);

	cksymtxctrl = FIELD_PREP(CKSYMTXCTRL_OVERRIDE_5, 0x1)				|
		      FIELD_PREP(CKSYMTXCTRL_TX_TRAON, config->voltage.tx_traon)	|
		      FIELD_PREP(CKSYMTXCTRL_TX_TRBON, config->voltage.tx_trbon)	|
		      FIELD_PREP(CKSYMTXCTRL_TX_SYMON, config->voltage.tx_symon)	|
		      FIELD_PREP(CKSYMTXCTRL_CK_SYMON, config->voltage.ck_symon);
	dw_hdmi_phy_i2c_write(
		hdmi,
		cksymtxctrl,
		DW_HDMI_TX_PHY_GEN2_CKSYMTXCTRL
	);

	return 0;
}

#endif
