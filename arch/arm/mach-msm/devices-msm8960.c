/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <linux/dma-mapping.h>
#include <mach/irqs-8960.h>
#include <mach/board.h>
#include <mach/msm_iomap.h>
#include <mach/irqs.h>

#include "devices.h"
#include "clock.h"

#define MSM_GSBI2_PHYS		0x16100000
#define MSM_UART2DM_PHYS	(MSM_GSBI2_PHYS + 0x40000)

#define MSM_GSBI5_PHYS		0x16400000
#define MSM_UART5DM_PHYS	(MSM_GSBI5_PHYS + 0x40000)

static struct resource resources_uart_gsbi2[] = {
	{
		.start	= GSBI2_UARTDM_IRQ,
		.end	= GSBI2_UARTDM_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= MSM_UART2DM_PHYS,
		.end	= MSM_UART2DM_PHYS + PAGE_SIZE - 1,
		.name	= "uart_resource",
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= MSM_GSBI2_PHYS,
		.end	= MSM_GSBI2_PHYS + PAGE_SIZE - 1,
		.name	= "gsbi_resource",
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device msm8960_device_uart_gsbi2 = {
	.name	= "msm_serial",
	.id	= 0,
	.num_resources	= ARRAY_SIZE(resources_uart_gsbi2),
	.resource	= resources_uart_gsbi2,
};

static struct resource resources_uart_gsbi5[] = {
	{
		.start	= GSBI5_UARTDM_IRQ,
		.end	= GSBI5_UARTDM_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= MSM_UART5DM_PHYS,
		.end	= MSM_UART5DM_PHYS + PAGE_SIZE - 1,
		.name	= "uart_resource",
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= MSM_GSBI5_PHYS,
		.end	= MSM_GSBI5_PHYS + PAGE_SIZE - 1,
		.name	= "gsbi_resource",
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device msm8960_device_uart_gsbi5 = {
	.name	= "msm_serial",
	.id	= 0,
	.num_resources	= ARRAY_SIZE(resources_uart_gsbi5),
	.resource	= resources_uart_gsbi5,
};

static struct resource resources_dmov[] = {
	{
		.start = MSM8960_DMOV_PHYS,
		.end = MSM8960_DMOV_PHYS + MSM8960_DMOV_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = INT_ADM0_SCSS_0_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device msm8960_device_dmov = {
	.name	= "msm_dmov",
	.id	= -1,
	.num_resources = ARRAY_SIZE(resources_dmov),
	.resource = resources_dmov,
};

struct clk_lookup msm_clocks_8960[] = {
	CLK_DUMMY("gsbi_uart_clk",	GSBI1_UART_CLK,		NULL),
	CLK_DUMMY("gsbi_uart_clk",	GSBI2_UART_CLK,		NULL),
	CLK_DUMMY("gsbi_uart_clk",	GSBI3_UART_CLK,		NULL),
	CLK_DUMMY("gsbi_uart_clk",	GSBI4_UART_CLK,		NULL),
	CLK_DUMMY("gsbi_uart_clk",	GSBI5_UART_CLK,		NULL),
	CLK_DUMMY("gsbi_uart_clk",	GSBI6_UART_CLK,		NULL),
	CLK_DUMMY("gsbi_uart_clk",	GSBI7_UART_CLK,		NULL),
	CLK_DUMMY("gsbi_uart_clk",	GSBI8_UART_CLK,		NULL),
	CLK_DUMMY("gsbi_uart_clk",	GSBI9_UART_CLK,		NULL),
	CLK_DUMMY("gsbi_uart_clk",	GSBI10_UART_CLK,	NULL),
	CLK_DUMMY("gsbi_uart_clk",	GSBI11_UART_CLK,	NULL),
	CLK_DUMMY("gsbi_uart_clk",	GSBI12_UART_CLK,	NULL),
	CLK_DUMMY("gsbi_qup_clk",	GSBI1_QUP_CLK,		NULL),
	CLK_DUMMY("gsbi_qup_clk",	GSBI2_QUP_CLK,		NULL),
	CLK_DUMMY("gsbi_qup_clk",	GSBI3_QUP_CLK,		NULL),
	CLK_DUMMY("gsbi_qup_clk",	GSBI4_QUP_CLK,		NULL),
	CLK_DUMMY("gsbi_qup_clk",	GSBI5_QUP_CLK,		NULL),
	CLK_DUMMY("gsbi_qup_clk",	GSBI6_QUP_CLK,		NULL),
	CLK_DUMMY("gsbi_qup_clk",	GSBI7_QUP_CLK,		NULL),
	CLK_DUMMY("gsbi_qup_clk",	GSBI8_QUP_CLK,		NULL),
	CLK_DUMMY("gsbi_qup_clk",	GSBI9_QUP_CLK,		NULL),
	CLK_DUMMY("gsbi_qup_clk",	GSBI10_QUP_CLK,		NULL),
	CLK_DUMMY("gsbi_qup_clk",	GSBI11_QUP_CLK,		NULL),
	CLK_DUMMY("gsbi_qup_clk",	GSBI12_QUP_CLK,		NULL),
	CLK_DUMMY("pdm_clk",		PDM_CLK,		NULL),
	CLK_DUMMY("pmem_clk",		PMEM_CLK,		NULL),
	CLK_DUMMY("prng_clk",		PRNG_CLK,		NULL),
	CLK_DUMMY("sdc_clk",		SDC1_CLK,		NULL),
	CLK_DUMMY("sdc_clk",		SDC2_CLK,		NULL),
	CLK_DUMMY("sdc_clk",		SDC3_CLK,		NULL),
	CLK_DUMMY("sdc_clk",		SDC4_CLK,		NULL),
	CLK_DUMMY("sdc_clk",		SDC5_CLK,		NULL),
	CLK_DUMMY("tsif_ref_clk",	TSIF_REF_CLK,		NULL),
	CLK_DUMMY("tssc_clk",		TSSC_CLK,		NULL),
	CLK_DUMMY("usb_hs_clk",		USB_HS1_XCVR_CLK,	NULL),
	CLK_DUMMY("usb_phy_clk",	USB_PHY0_CLK,		NULL),
	CLK_DUMMY("usb_fs_src_clk",	USB_FS1_SRC_CLK,	NULL),
	CLK_DUMMY("usb_fs_clk",		USB_FS1_XCVR_CLK,	NULL),
	CLK_DUMMY("usb_fs_sys_clk",	USB_FS1_SYS_CLK,	NULL),
	CLK_DUMMY("usb_fs_src_clk",	USB_FS2_SRC_CLK,	NULL),
	CLK_DUMMY("usb_fs_clk",		USB_FS2_XCVR_CLK,	NULL),
	CLK_DUMMY("usb_fs_sys_clk",	USB_FS2_SYS_CLK,	NULL),
	CLK_DUMMY("ce_clk",		CE2_CLK,		NULL),
	CLK_DUMMY("gsbi_pclk",		GSBI1_P_CLK,		NULL),
	CLK_DUMMY("gsbi_pclk",		GSBI2_P_CLK,		NULL),
	CLK_DUMMY("gsbi_pclk",		GSBI3_P_CLK,		NULL),
	CLK_DUMMY("gsbi_pclk",		GSBI4_P_CLK,		NULL),
	CLK_DUMMY("gsbi_pclk",		GSBI5_P_CLK,		NULL),
	CLK_DUMMY("gsbi_pclk",		GSBI6_P_CLK,		NULL),
	CLK_DUMMY("gsbi_pclk",		GSBI7_P_CLK,		NULL),
	CLK_DUMMY("gsbi_pclk",		GSBI8_P_CLK,		NULL),
	CLK_DUMMY("gsbi_pclk",		GSBI9_P_CLK,		NULL),
	CLK_DUMMY("gsbi_pclk",		GSBI10_P_CLK,		NULL),
	CLK_DUMMY("gsbi_pclk",		GSBI11_P_CLK,		NULL),
	CLK_DUMMY("gsbi_pclk",		GSBI12_P_CLK,		NULL),
	CLK_DUMMY("gsbi_pclk",		GSBI12_P_CLK,		NULL),
	CLK_DUMMY("ppss_pclk",		PPSS_P_CLK,		NULL),
	CLK_DUMMY("tsif_pclk",		TSIF_P_CLK,		NULL),
	CLK_DUMMY("usb_fs_pclk",	USB_FS1_P_CLK,		NULL),
	CLK_DUMMY("usb_fs_pclk",	USB_FS2_P_CLK,		NULL),
	CLK_DUMMY("usb_hs_pclk",	USB_HS1_P_CLK,		NULL),
	CLK_DUMMY("sdc_pclk",		SDC1_P_CLK,		NULL),
	CLK_DUMMY("sdc_pclk",		SDC2_P_CLK,		NULL),
	CLK_DUMMY("sdc_pclk",		SDC3_P_CLK,		NULL),
	CLK_DUMMY("sdc_pclk",		SDC4_P_CLK,		NULL),
	CLK_DUMMY("sdc_pclk",		SDC5_P_CLK,		NULL),
	CLK_DUMMY("adm_clk",		ADM0_CLK,		NULL),
	CLK_DUMMY("adm_pclk",		ADM0_P_CLK,		NULL),
	CLK_DUMMY("pmic_arb_pclk",	PMIC_ARB0_P_CLK,	NULL),
	CLK_DUMMY("pmic_arb_pclk",	PMIC_ARB1_P_CLK,	NULL),
	CLK_DUMMY("pmic_ssbi2",		PMIC_SSBI2_CLK,		NULL),
	CLK_DUMMY("rpm_msg_ram_pclk",	RPM_MSG_RAM_P_CLK,	NULL),
	CLK_DUMMY("amp_clk",		AMP_CLK,		NULL),
	CLK_DUMMY("cam_clk",		CAM0_CLK,		NULL),
	CLK_DUMMY("cam_clk",		CAM1_CLK,		NULL),
	CLK_DUMMY("csi_src_clk",	CSI0_SRC_CLK,		NULL),
	CLK_DUMMY("csi_src_clk",	CSI1_SRC_CLK,		NULL),
	CLK_DUMMY("csi_clk",		CSI0_CLK,		NULL),
	CLK_DUMMY("csi_clk",		CSI1_CLK,		NULL),
	CLK_DUMMY("csi_pix_clk",	CSI_PIX_CLK,		NULL),
	CLK_DUMMY("csi_rdi_clk",	CSI_RDI_CLK,		NULL),
	CLK_DUMMY("csiphy_timer_src_clk", CSIPHY_TIMER_SRC_CLK,	NULL),
	CLK_DUMMY("csi0phy_timer_clk",	CSIPHY0_TIMER_CLK,	NULL),
	CLK_DUMMY("csi1phy_timer_clk",	CSIPHY1_TIMER_CLK,	NULL),
	CLK_DUMMY("dsi_byte_div_clk",	DSI1_BYTE_CLK,		NULL),
	CLK_DUMMY("dsi_byte_div_clk",	DSI2_BYTE_CLK,		NULL),
	CLK_DUMMY("dsi_esc_clk",	DSI1_ESC_CLK,		NULL),
	CLK_DUMMY("dsi_esc_clk",	DSI2_ESC_CLK,		NULL),
	CLK_DUMMY("gfx2d0_clk",		GFX2D0_CLK,		NULL),
	CLK_DUMMY("gfx2d1_clk",		GFX2D1_CLK,		NULL),
	CLK_DUMMY("gfx3d_clk",		GFX3D_CLK,		NULL),
	CLK_DUMMY("ijpeg_axi_clk",	IJPEG_AXI_CLK,		NULL),
	CLK_DUMMY("imem_axi_clk",	IMEM_AXI_CLK,		NULL),
	CLK_DUMMY("jpegd_clk",		JPEGD_CLK,		NULL),
	CLK_DUMMY("mdp_clk",		MDP_CLK,		NULL),
	CLK_DUMMY("mdp_vsync_clk",	MDP_VSYNC_CLK,		NULL),
	CLK_DUMMY("lut_mdp",		LUT_MDP_CLK,		NULL),
	CLK_DUMMY("rot_clk",		ROT_CLK,		NULL),
	CLK_DUMMY("tv_src_clk",		TV_SRC_CLK,		NULL),
	CLK_DUMMY("tv_enc_clk",		TV_ENC_CLK,		NULL),
	CLK_DUMMY("tv_dac_clk",		TV_DAC_CLK,		NULL),
	CLK_DUMMY("vcodec_clk",		VCODEC_CLK,		NULL),
	CLK_DUMMY("mdp_tv_clk",		MDP_TV_CLK,		NULL),
	CLK_DUMMY("hdmi_clk",		HDMI_TV_CLK,		NULL),
	CLK_DUMMY("hdmi_app_clk",	HDMI_APP_CLK,		NULL),
	CLK_DUMMY("vpe_clk",		VPE_CLK,		NULL),
	CLK_DUMMY("vfe_clk",		VFE_CLK,		NULL),
	CLK_DUMMY("csi_vfe_clk",	CSI0_VFE_CLK,		NULL),
	CLK_DUMMY("vfe_axi_clk",	VFE_AXI_CLK,		NULL),
	CLK_DUMMY("ijpeg_axi_clk",	IJPEG_AXI_CLK,		NULL),
	CLK_DUMMY("mdp_axi_clk",	MDP_AXI_CLK,		NULL),
	CLK_DUMMY("rot_axi_clk",	ROT_AXI_CLK,		NULL),
	CLK_DUMMY("vcodec_axi_clk",	VCODEC_AXI_CLK,		NULL),
	CLK_DUMMY("vcodec_axi_a_clk",	VCODEC_AXI_A_CLK,	NULL),
	CLK_DUMMY("vcodec_axi_b_clk",	VCODEC_AXI_B_CLK,	NULL),
	CLK_DUMMY("vpe_axi_clk",	VPE_AXI_CLK,		NULL),
	CLK_DUMMY("amp_pclk",		AMP_P_CLK,		NULL),
	CLK_DUMMY("csi_pclk",		CSI0_P_CLK,		NULL),
	CLK_DUMMY("dsi_m_pclk",		DSI1_M_P_CLK,		NULL),
	CLK_DUMMY("dsi_s_pclk",		DSI1_S_P_CLK,		NULL),
	CLK_DUMMY("dsi_m_pclk",		DSI2_M_P_CLK,		NULL),
	CLK_DUMMY("dsi_s_pclk",		DSI2_S_P_CLK,		NULL),
	CLK_DUMMY("gfx2d0_pclk",	GFX2D0_P_CLK,		NULL),
	CLK_DUMMY("gfx2d1_pclk",	GFX2D1_P_CLK,		NULL),
	CLK_DUMMY("gfx3d_pclk",		GFX3D_P_CLK,		NULL),
	CLK_DUMMY("hdmi_m_pclk",	HDMI_M_P_CLK,		NULL),
	CLK_DUMMY("hdmi_s_pclk",	HDMI_S_P_CLK,		NULL),
	CLK_DUMMY("ijpeg_pclk",		IJPEG_P_CLK,		NULL),
	CLK_DUMMY("jpegd_pclk",		JPEGD_P_CLK,		NULL),
	CLK_DUMMY("imem_pclk",		IMEM_P_CLK,		NULL),
	CLK_DUMMY("mdp_pclk",		MDP_P_CLK,		NULL),
	CLK_DUMMY("smmu_pclk",		SMMU_P_CLK,		NULL),
	CLK_DUMMY("rotator_pclk",	ROT_P_CLK,		NULL),
	CLK_DUMMY("tv_enc_pclk",	TV_ENC_P_CLK,		NULL),
	CLK_DUMMY("vcodec_pclk",	VCODEC_P_CLK,		NULL),
	CLK_DUMMY("vfe_pclk",		VFE_P_CLK,		NULL),
	CLK_DUMMY("vpe_pclk",		VPE_P_CLK,		NULL),
	CLK_DUMMY("mi2s_osr_clk",	MI2S_OSR_CLK,		NULL),
	CLK_DUMMY("mi2s_bit_clk",	MI2S_BIT_CLK,		NULL),
	CLK_DUMMY("i2s_mic_osr_clk",	CODEC_I2S_MIC_OSR_CLK,	NULL),
	CLK_DUMMY("i2s_mic_bit_clk",	CODEC_I2S_MIC_BIT_CLK,	NULL),
	CLK_DUMMY("i2s_mic_osr_clk",	SPARE_I2S_MIC_OSR_CLK,	NULL),
	CLK_DUMMY("i2s_mic_bit_clk",	SPARE_I2S_MIC_BIT_CLK,	NULL),
	CLK_DUMMY("i2s_spkr_osr_clk",	CODEC_I2S_SPKR_OSR_CLK,	NULL),
	CLK_DUMMY("i2s_spkr_bit_clk",	CODEC_I2S_SPKR_BIT_CLK,	NULL),
	CLK_DUMMY("i2s_spkr_osr_clk",	SPARE_I2S_SPKR_OSR_CLK,	NULL),
	CLK_DUMMY("i2s_spkr_bit_clk",	SPARE_I2S_SPKR_BIT_CLK,	NULL),
	CLK_DUMMY("pcm_clk",		PCM_CLK,		NULL),
	CLK_DUMMY("iommu_clk",		JPEGD_AXI_CLK,		NULL),
	CLK_DUMMY("iommu_clk",		VFE_AXI_CLK,		NULL),
	CLK_DUMMY("iommu_clk",		VCODEC_AXI_CLK,	NULL),
	CLK_DUMMY("iommu_clk",		GFX3D_CLK,	NULL),
	CLK_DUMMY("iommu_clk",		GFX2D0_CLK,	NULL),
	CLK_DUMMY("iommu_clk",		GFX2D1_CLK,	NULL),
};

unsigned msm_num_clocks_8960 = ARRAY_SIZE(msm_clocks_8960);
