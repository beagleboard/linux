#include <linux/component.h>
#include <linux/hdmi.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <sound/asoundef.h>
#include <sound/hdmi-codec.h>
#include <linux/string.h>

#include <drm/drmP.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_atomic.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_of.h>
#include <linux/of_irq.h>

#include "it66121.h"
#define INIT_CLK_HIGH
// #define INIT_CLK_LOW
#define conn_to_it66121_priv(x) \
	container_of(x, struct it66121_priv, connector)

#define enc_to_it66121_priv(x) \
	container_of(x, struct it66121_priv, encoder)



u8  bCSCMtx_RGB2YUV_ITU601_16_235[] =
{
	0x00, 0x80, 0x00,
	0xB2, 0x04, 0x65, 0x02, 0xE9, 0x00,
	0x93, 0x3C, 0x18, 0x04, 0x55, 0x3F,
	0x49, 0x3D, 0x9F, 0x3E, 0x18, 0x04
};

u8 bCSCMtx_RGB2YUV_ITU601_0_255[] =
{
	0x10, 0x80, 0x10,
	0x09, 0x04, 0x0E, 0x02, 0xC9, 0x00,
	0x0F, 0x3D, 0x84, 0x03, 0x6D, 0x3F,
	0xAB, 0x3D, 0xD1, 0x3E, 0x84, 0x03
};

u8 bCSCMtx_RGB2YUV_ITU709_16_235[] =
{
	0x00, 0x80, 0x00,
	0xB8, 0x05, 0xB4, 0x01, 0x94, 0x00,
	0x4a, 0x3C, 0x17, 0x04, 0x9F, 0x3F,
	0xD9, 0x3C, 0x10, 0x3F, 0x17, 0x04
};

u8 bCSCMtx_RGB2YUV_ITU709_0_255[] =
{
	0x10, 0x80, 0x10,
	0xEa, 0x04, 0x77, 0x01, 0x7F, 0x00,
	0xD0, 0x3C, 0x83, 0x03, 0xAD, 0x3F,
	0x4B, 0x3D, 0x32, 0x3F, 0x83, 0x03
};


u8 bCSCMtx_YUV2RGB_ITU601_16_235[] =
{
	0x00, 0x00, 0x00,
	0x00, 0x08, 0x6B, 0x3A, 0x50, 0x3D,
	0x00, 0x08, 0xF5, 0x0A, 0x02, 0x00,
	0x00, 0x08, 0xFD, 0x3F, 0xDA, 0x0D
};

u8 bCSCMtx_YUV2RGB_ITU601_0_255[] =
{
	0x04, 0x00, 0xA7,
	0x4F, 0x09, 0x81, 0x39, 0xDD, 0x3C,
	0x4F, 0x09, 0xC4, 0x0C, 0x01, 0x00,
	0x4F, 0x09, 0xFD, 0x3F, 0x1F, 0x10
};

u8 bCSCMtx_YUV2RGB_ITU709_16_235[] =
{
	0x00, 0x00, 0x00,
	0x00, 0x08, 0x55, 0x3C, 0x88, 0x3E,
	0x00, 0x08, 0x51, 0x0C, 0x00, 0x00,
	0x00, 0x08, 0x00, 0x00, 0x84, 0x0E
};

u8 bCSCMtx_YUV2RGB_ITU709_0_255[] =
{
	0x04, 0x00, 0xA7,
	0x4F, 0x09, 0xBA, 0x3B, 0x4B, 0x3E,
	0x4F, 0x09, 0x57, 0x0E, 0x02, 0x00,
	0x4F, 0x09, 0xFE, 0x3F, 0xE8, 0x10
};

static struct a_reg_entry  it66121_init_table[] = {

	{ 0x0F, 0x40, 0x00 }, // enabling register programming.

	{ 0x62, 0x08, 0x00 }, // TMDSTXPLL18VA0 is reset.
	{ 0x64, 0x04, 0x00 }, // TMDSIPLL18VA0 is reset
	{ 0x01, 0x00, 0x00 }, //idle(100);

	{ 0x04, 0x20, 0x20 },  //Software RCLK reset.
	{ 0x04, 0x1D, 0x1D },
	{ 0x01, 0x00, 0x00 }, //idle(100);
	{ 0x0F, 0x01, 0x00 }, // bank 0 ;
#ifdef INIT_CLK_LOW
	{ 0x62, 0x90, 0x10 },
	{ 0x64, 0x89, 0x09 },
	{ 0x68, 0x10, 0x10 },
#endif

	{ 0xD1, 0x0E, 0x0C },
	{ 0x65, 0x03, 0x00 },
#ifdef NON_SEQUENTIAL_YCBCR422 // for ITE HDMIRX
	{ 0x71, 0xFC, 0x1C },
#else
	{ 0x71, 0xFC, 0x18 },
#endif

	{ 0x8D, 0xFF, CEC_I2C_SLAVE_ADDR },
	{ 0x0F, 0x08, 0x08 },

	{ 0xF8, 0xFF, 0xC3 },
	{ 0xF8, 0xFF, 0xA5 },
	{ 0x20, 0x80, 0x80 },
	{ 0x37, 0x01, 0x00 },
	{ 0x20, 0x80, 0x00 },
	{ 0xF8, 0xFF, 0xFF },

	{ 0x59, 0xD8, 0x40 | PCLKINV },
	{ 0xE1, 0x20, InvAudCLK },
	{ 0x05, 0xC0, 0x40 },
	{ REG_TX_INT_MASK1, 0xFF, ~(B_TX_RXSEN_MASK | B_TX_HPD_MASK) },
	{ REG_TX_INT_MASK2, 0xFF, ~(B_TX_KSVLISTCHK_MASK | B_TX_AUTH_DONE_MASK | B_TX_AUTH_FAIL_MASK) },
	{ REG_TX_INT_MASK3, 0xFF, ~(0x0) },
	{ 0x0C, 0xFF, 0xFF },
	{ 0x0D, 0xFF, 0xFF },
	{ 0x0E, 0x03, 0x03 },

	{ 0x0C, 0xFF, 0x00 },
	{ 0x0D, 0xFF, 0x00 },
	{ 0x0E, 0x02, 0x00 },
	{ 0x09, 0x03, 0x00 }, // Enable HPD and RxSen Interrupt
	{ 0, 0, 0 }
};

static struct a_reg_entry  it66121_default_video_table[] = {
	/* Config default output format.*/
	{ 0x72, 0xff, 0x00 },
	{ 0x70, 0xff, 0x00 },
#ifndef DEFAULT_INPUT_YCBCR
// GenCSC\RGB2YUV_ITU709_16_235.c
	{ 0x72, 0xFF, 0x02 },
	{ 0x73, 0xFF, 0x00 },
	{ 0x74, 0xFF, 0x80 },
	{ 0x75, 0xFF, 0x00 },
	{ 0x76, 0xFF, 0xB8 },
	{ 0x77, 0xFF, 0x05 },
	{ 0x78, 0xFF, 0xB4 },
	{ 0x79, 0xFF, 0x01 },
	{ 0x7A, 0xFF, 0x93 },
	{ 0x7B, 0xFF, 0x00 },
	{ 0x7C, 0xFF, 0x49 },
	{ 0x7D, 0xFF, 0x3C },
	{ 0x7E, 0xFF, 0x18 },
	{ 0x7F, 0xFF, 0x04 },
	{ 0x80, 0xFF, 0x9F },
	{ 0x81, 0xFF, 0x3F },
	{ 0x82, 0xFF, 0xD9 },
	{ 0x83, 0xFF, 0x3C },
	{ 0x84, 0xFF, 0x10 },
	{ 0x85, 0xFF, 0x3F },
	{ 0x86, 0xFF, 0x18 },
	{ 0x87, 0xFF, 0x04 },
#else
// GenCSC\YUV2RGB_ITU709_16_235.c
	{ 0x0F, 0x01, 0x00 },
	{ 0x72, 0xFF, 0x03 },
	{ 0x73, 0xFF, 0x00 },
	{ 0x74, 0xFF, 0x80 },
	{ 0x75, 0xFF, 0x00 },
	{ 0x76, 0xFF, 0x00 },
	{ 0x77, 0xFF, 0x08 },
	{ 0x78, 0xFF, 0x53 },
	{ 0x79, 0xFF, 0x3C },
	{ 0x7A, 0xFF, 0x89 },
	{ 0x7B, 0xFF, 0x3E },
	{ 0x7C, 0xFF, 0x00 },
	{ 0x7D, 0xFF, 0x08 },
	{ 0x7E, 0xFF, 0x51 },
	{ 0x7F, 0xFF, 0x0C },
	{ 0x80, 0xFF, 0x00 },
	{ 0x81, 0xFF, 0x00 },
	{ 0x82, 0xFF, 0x00 },
	{ 0x83, 0xFF, 0x08 },
	{ 0x84, 0xFF, 0x00 },
	{ 0x85, 0xFF, 0x00 },
	{ 0x86, 0xFF, 0x87 },
	{ 0x87, 0xFF, 0x0E },
#endif
	// 2012/12/20 added by Keming's suggestion test
	{ 0x88, 0xF0, 0x00 },
	//~jauchih.tseng@ite.com.tw
	{ 0x04, 0x08, 0x00 },
	{ 0, 0, 0 }
};
static struct a_reg_entry  it66121_setHDMI_table[] = {
	/* Config default HDMI Mode */
	{ 0xC0, 0x01, 0x01 },
	{ 0xC1, 0x03, 0x03 },
	{ 0xC6, 0x03, 0x03 },
	{ 0, 0, 0 }
};

static struct a_reg_entry  it66121_setDVI_table[] = {
	/* Config default DVI Mode */

	{ 0x0F, 0x01, 0x01 },
	{ 0x58, 0xFF, 0x00 },
	{ 0x0F, 0x01, 0x00 },
	{ 0xC0, 0x01, 0x00 },
	{ 0xC1, 0x03, 0x02 },
	{ 0xC6, 0x03, 0x00 },
	{ 0, 0, 0 }
};

static struct a_reg_entry  it66121_default_AVI_info_table[] = {


	/* Config default avi infoframe */

	{ 0x0F, 0x01, 0x01 },
	{ 0x58, 0xFF, 0x10 },
	{ 0x59, 0xFF, 0x08 },
	{ 0x5A, 0xFF, 0x00 },
	{ 0x5B, 0xFF, 0x00 },
	{ 0x5C, 0xFF, 0x00 },
	{ 0x5D, 0xFF, 0x57 },
	{ 0x5E, 0xFF, 0x00 },
	{ 0x5F, 0xFF, 0x00 },
	{ 0x60, 0xFF, 0x00 },
	{ 0x61, 0xFF, 0x00 },
	{ 0x62, 0xFF, 0x00 },
	{ 0x63, 0xFF, 0x00 },
	{ 0x64, 0xFF, 0x00 },
	{ 0x65, 0xFF, 0x00 },
	{ 0x0F, 0x01, 0x00 },
	{ 0xCD, 0x03, 0x03 },
	{ 0, 0, 0 }
};
static struct a_reg_entry  it66121_default_audio_info_table[] = {
	/* Config default audio infoframe */
	{ 0x0F, 0x01, 0x01 },
	{ 0x68, 0xFF, 0x00 },
	{ 0x69, 0xFF, 0x00 },
	{ 0x6A, 0xFF, 0x00 },
	{ 0x6B, 0xFF, 0x00 },
	{ 0x6C, 0xFF, 0x00 },
	{ 0x6D, 0xFF, 0x71 },
	{ 0x0F, 0x01, 0x00 },
	{ 0xCE, 0x03, 0x03 },

	{ 0, 0, 0 }
};

static struct a_reg_entry  it66121_aud_CHStatus_LPCM_20bit_48Khz[] =
{
	{ 0x0F, 0x01, 0x01 },
	{ 0x33, 0xFF, 0x00 },
	{ 0x34, 0xFF, 0x18 },
	{ 0x35, 0xFF, 0x00 },
	{ 0x91, 0xFF, 0x00 },
	{ 0x92, 0xFF, 0x00 },
	{ 0x93, 0xFF, 0x01 },
	{ 0x94, 0xFF, 0x00 },
	{ 0x98, 0xFF, 0x02 },
	{ 0x99, 0xFF, 0xDA },
	{ 0x0F, 0x01, 0x00 },
	{ 0, 0, 0 } //end of table
};

static struct a_reg_entry  it66121_AUD_SPDIF_2ch_24bit[] =
{
	{ 0x0F, 0x11, 0x00 },
	{ 0x04, 0x14, 0x04 },
	{ 0xE0, 0xFF, 0xD1 },
	{ 0xE1, 0xFF, 0x01 },
	{ 0xE2, 0xFF, 0xE4 },
	{ 0xE3, 0xFF, 0x10 },
	{ 0xE4, 0xFF, 0x00 },
	{ 0xE5, 0xFF, 0x00 },
	{ 0x04, 0x14, 0x00 },
	{ 0, 0, 0 } //end of table
};

static struct a_reg_entry  it66121_AUD_I2S_2ch_24bit[] =
{
	{ 0x0F, 0x11, 0x00 },
	{ 0x04, 0x14, 0x04 },
	{ 0xE0, 0xFF, 0xC1 },
	{ 0xE1, 0xFF, 0x01 },
	{ 0xE2, 0xFF, 0xE4 },
	{ 0xE3, 0xFF, 0x00 },
	{ 0xE4, 0xFF, 0x00 },
	{ 0xE5, 0xFF, 0x00 },
	{ 0x04, 0x14, 0x00 },
	{ 0, 0, 0 } //end of table
};

static struct a_reg_entry  it66121_default_audio_table[] = {


	/* Config default audio output format. */
	{ 0x0F, 0x21, 0x00 },
	{ 0x04, 0x14, 0x04 },
	{ 0xE0, 0xFF, 0xC1 },
	{ 0xE1, 0xFF, 0x01 },
	{ 0xE2, 0xFF, 0xE4 },
	{ 0xE3, 0xFF, 0x00 },
	{ 0xE4, 0xFF, 0x00 },
	{ 0xE5, 0xFF, 0x00 },
	{ 0x0F, 0x01, 0x01 },
	{ 0x33, 0xFF, 0x00 },
	{ 0x34, 0xFF, 0x18 },
	{ 0x35, 0xFF, 0x00 },
	{ 0x91, 0xFF, 0x00 },
	{ 0x92, 0xFF, 0x00 },
	{ 0x93, 0xFF, 0x01 },
	{ 0x94, 0xFF, 0x00 },
	{ 0x98, 0xFF, 0x02 },
	{ 0x99, 0xFF, 0xDB },
	{ 0x0F, 0x01, 0x00 },
	{ 0x04, 0x14, 0x00 },

	{ 0x00, 0x00, 0x00 } // End of Table.
};

static struct a_reg_entry  it66121_Pwr_down_table[] = {
	// Enable GRCLK
	{ 0x0F, 0x40, 0x00 },
	// PLL Reset
	{ 0x61, 0x10, 0x10 },   // DRV_RST
	{ 0x62, 0x08, 0x00 },   // XP_RESETB
	{ 0x64, 0x04, 0x00 },   // IP_RESETB
	{ 0x01, 0x00, 0x00 }, // idle(100);

	// PLL PwrDn
	{ 0x61, 0x20, 0x20 },   // PwrDn DRV
	{ 0x62, 0x44, 0x44 },   // PwrDn XPLL
	{ 0x64, 0x40, 0x40 },   // PwrDn IPLL

	// HDMITX PwrDn
	{ 0x05, 0x01, 0x01 },   // PwrDn PCLK
	{ 0x0F, 0x78, 0x78 },   // PwrDn GRCLK
	{ 0x00, 0x00, 0x00 } // End of Table.
};

static struct a_reg_entry it66121_Pwr_on_table[] = {
	{ 0x0F, 0x78, 0x38 },   // PwrOn GRCLK
	{ 0x05, 0x01, 0x00 },   // PwrOn PCLK

	// PLL PwrOn
	{ 0x61, 0x20, 0x00 },   // PwrOn DRV
	{ 0x62, 0x44, 0x00 },   // PwrOn XPLL
	{ 0x64, 0x40, 0x00 },   // PwrOn IPLL

	// PLL Reset OFF
	{ 0x61, 0x10, 0x00 },   // DRV_RST
	{ 0x62, 0x08, 0x08 },   // XP_RESETB
	{ 0x64, 0x04, 0x04 },   // IP_RESETB
	{ 0x0F, 0x78, 0x08 },   // PwrOn IACLK
	{ 0x00, 0x00, 0x00 } // End of Table.
};


static u8 reg_read(struct i2c_client *client, u8 reg) {
	struct i2c_msg msgs[2];
	int ret = -1;
	u8 buf[1];

	buf[0] = reg;

	/* Write device addr fisrt */
	msgs[0].addr	= client->addr;
	msgs[0].flags	= !I2C_M_RD;
	msgs[0].len		= 1;
	msgs[0].buf		= &buf[0];
	/* Then, begin to read data */
	msgs[1].addr	= client->addr;
	msgs[1].flags	= I2C_M_RD;
	msgs[1].len		= 1;
	msgs[1].buf		= &buf[0];

	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret != 2) dev_err(&client->dev, "I2C transfer Error! ret = %d\n", ret);

	//ErrorF("Reg%02xH: 0x%02x\n", RegAddr, buf[0]);
	return buf[0];
}

static int reg_write(struct i2c_client *client, u8 reg, u8 val) {
	struct i2c_msg msg;
	int  ret = -1;
	u8 buf[2];

	buf[0] = reg;
	buf[1] = val;

	msg.addr	= client->addr;
	msg.flags	= !I2C_M_RD;
	msg.len		= 2;
	msg.buf		= buf;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret != 1) dev_err(&client->dev, "I2C transfer Error!\n");
	return ret;
}

static int write_a_entry(struct it66121_priv *priv, u8 reg, u8 mask, u8 val) {
	u8 T;
	int ret;
	mutex_lock(&priv->mutex);
	if (mask != 0xFF) {
		T  = reg_read(priv->hdmi, reg);
		T &= (~mask);
		T |= val & mask;
	}   else {
		T = val;
	}
	ret = reg_write(priv->hdmi, reg, T);
	mutex_unlock(&priv->mutex);
	return ret;
}
static inline void it66121_switch_blank(struct it66121_priv *priv, u8 bank) {
	write_a_entry(priv, 0x0f, 1, bank);
}
static int it66121_load_reg_table(struct it66121_priv *priv, struct a_reg_entry table[]) {
	int ret = 0;
	int i;
	for (i = 0;; i++) {
		if (table[i].reg == 0 && table[i].mask == 0 && table[i].val == 0) {
			return ret;
		} else if (table[i].mask == 0 && table[i].val == 0) {
			mdelay(table[i].reg);
		} else if (table[i].mask == 0xFF) {
			ret = reg_write(priv->hdmi, table[i].reg, table[i].val);
		} else {
			ret = write_a_entry(priv, table[i].reg, table[i].mask, table[i].val);
		}
		if (ret < 0) {
			return ret;
		}
	}
	return ret;
}

static int  it66121_power_on(struct it66121_priv *priv) {
	priv->powerstatus = 1;
	return it66121_load_reg_table(priv, it66121_Pwr_on_table);
}

static int  it66121_power_down(struct it66121_priv *priv) {
	priv->powerstatus = 0;
	return it66121_load_reg_table(priv, it66121_Pwr_down_table);
}

static enum drm_connector_status
it66121_connector_detect(struct drm_connector *connector, bool force) {
	struct it66121_priv *priv = conn_to_it66121_priv(connector);
	char isconnect = reg_read(priv->hdmi, REG_TX_SYS_STATUS) & B_TX_HPDETECT;
	return isconnect ? connector_status_connected : connector_status_disconnected;
}

static int it66121_connector_dpms(struct drm_connector *connector, int mode) {
	if (drm_core_check_feature(connector->dev, DRIVER_ATOMIC)) return drm_atomic_helper_connector_dpms(connector, mode);
	else return drm_helper_connector_dpms(connector, mode);
}

static void it66121_connector_destroy(struct drm_connector *connector) {
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
}

static const struct drm_connector_funcs it66121_connector_funcs = {
	.dpms = it66121_connector_dpms,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = it66121_connector_detect,
	.destroy = it66121_connector_destroy,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};
static int it66121_connector_mode_valid(struct drm_connector *connector,
										struct drm_display_mode *mode) {
	//return drm_match_cea_mode(mode) == 0 ? MODE_BAD : MODE_OK;
	return MODE_OK;
}


static struct drm_encoder*
it66121_connector_best_encoder(struct drm_connector *connector) {
	struct it66121_priv *priv = conn_to_it66121_priv(connector);

	return &priv->encoder;
}
static void it66121_abort_DDC(struct i2c_client *client) {
	u8 CPDesire, SWReset, DDCMaster;
	u8 uc, timeout, i;
	// save the SW reset,DDC master,and CP Desire setting.
	SWReset = reg_read(client, REG_TX_SW_RST);
	CPDesire = reg_read(client, REG_TX_HDCP_DESIRE);
	DDCMaster = reg_read(client, REG_TX_DDC_MASTER_CTRL);

	reg_write(client, REG_TX_HDCP_DESIRE, CPDesire & (~B_TX_CPDESIRE));
	reg_write(client, REG_TX_SW_RST, SWReset | B_TX_HDCP_RST_HDMITX);
	reg_write(client, REG_TX_DDC_MASTER_CTRL, B_TX_MASTERDDC | B_TX_MASTERHOST);

	// 2009/01/15 modified by Jau-Chih.Tseng@ite.com.tw
	// do abort DDC twice.
	for (i = 0; i < 2; i++) {
		reg_write(client, REG_TX_DDC_CMD, CMD_DDC_ABORT);

		for (timeout = 0; timeout < 200; timeout++) {
			uc = reg_read(client, REG_TX_DDC_STATUS);
			if (uc & B_TX_DDC_DONE) {
				break; // success
			}
			if (uc & (B_TX_DDC_NOACK | B_TX_DDC_WAITBUS | B_TX_DDC_ARBILOSE)) {
				dev_err(&client->dev, "it66121_abort_DDC Fail by reg16=%02X\n", (int)uc);
				break;
			}
			mdelay(1); // delay 1 ms to stable.
		}
	}
	//~Jau-Chih.Tseng@ite.com.tw
}
/*
 * To get the EDID data, DDC master should write segment with I2C address 0x60 then ask
 * the bytes with I2C address 0xA0. (That is the major difference to burst read.) The
 * programming of EDID read should set the following registers:
 *
 * Reg11 \A8C Should set 0xA0 for EDID fetching.
 * Reg12 \A8C Set the starting offset of EDID block on current segment.
 * Reg13 \A8C Set the number of byte to read back. The data will be put in DDC FIFO,
 * 		   therefore, cannot exceed the size (32) of FIFO.
 * Reg14 \A8C The segment of EDID block to read.
 * Reg15 \A8C DDC command should be 0x03.
 *
 * After reg15 written 0x03, the command is fired and successfully when reg16[7] = '1' or
 * fail by reg16[5:3] contains any bit '1'. When EDID read done, EDID can be read from DDC
 * FIFO.
 * Note: By hardware implementation, the I2C access sequence on PCSCL/PCSDA should be
 *
 * <start>-<0x98/0x9A>-<0x17>-<Restart>-<0x99/0x9B>-<read data>-<stop>
 *
 * If the sequence is the following sequence, the FIFO read will be fail.
 *
 * <start>-<0x98/0x9A>-<0x17>-<stop>-<start>-<0x99/0x9B>-<read data>-<stop>
 */
static int it66121_read_edid_block(void *data, u8 *buf, unsigned int blk, size_t length) {
	struct it66121_priv *priv = data;
	struct i2c_client *client = priv->hdmi;

	u16 ReqCount;
	u8 bCurrOffset;
	u16 TimeOut;
	u8 *pBuff = buf;
	u8 ucdata;
	u8 bSegment;

	if (!buf) return -1;

	if (reg_read(priv->hdmi, REG_TX_INT_STAT1) & B_TX_INT_DDC_BUS_HANG) {
		dev_err(&priv->hdmi->dev, "Sorry, ddc bus is hang\n");
		it66121_abort_DDC(priv->hdmi);
	}
	/*clear the DDC FIFO*/
	reg_write(client, REG_TX_DDC_MASTER_CTRL, B_TX_MASTERDDC | B_TX_MASTERHOST);
	reg_write(client, REG_TX_DDC_CMD, CMD_FIFO_CLR);

	bCurrOffset = (blk % 2) / length;
	bSegment  = blk / length;
	it66121_switch_blank(priv, 0);

	while (length > 0) {
		ReqCount = (length > DDC_FIFO_MAXREQ) ? DDC_FIFO_MAXREQ : length;

		reg_write(client, REG_TX_DDC_MASTER_CTRL, B_TX_MASTERDDC | B_TX_MASTERHOST);
		reg_write(client, REG_TX_DDC_CMD, CMD_FIFO_CLR);

		for (TimeOut = 0; TimeOut < 200; TimeOut++) {
			ucdata = reg_read(client, REG_TX_DDC_STATUS);

			if (ucdata & B_TX_DDC_DONE) {
				break;
			}
			if ((ucdata & B_TX_DDC_ERROR) || (reg_read(client, REG_TX_INT_STAT1) & B_TX_INT_DDC_BUS_HANG)) {
				dev_err(&priv->hdmi->dev, "it66121_read_edid_block(): DDC_STATUS = %02X,fail.\n", (int)ucdata);
				/*clear the DDC FIFO*/
				reg_write(client, REG_TX_DDC_MASTER_CTRL, B_TX_MASTERDDC | B_TX_MASTERHOST);
				reg_write(client, REG_TX_DDC_CMD, CMD_FIFO_CLR);
				return -1;
			}
		}
		reg_write(client, REG_TX_DDC_MASTER_CTRL, B_TX_MASTERDDC | B_TX_MASTERHOST); //0x10
		reg_write(client, REG_TX_DDC_HEADER, DDC_EDID_ADDRESS); // for EDID ucdata get 0x11
		reg_write(client, REG_TX_DDC_REQOFF, bCurrOffset); //0x12
		reg_write(client, REG_TX_DDC_REQCOUNT, (u8)ReqCount); //0x13
		reg_write(client, REG_TX_DDC_EDIDSEG, bSegment); //0x14
		reg_write(client, REG_TX_DDC_CMD, CMD_EDID_READ); //0x15

		bCurrOffset += ReqCount;
		length -= ReqCount;

		for (TimeOut = 250; TimeOut > 0; TimeOut--) {
			mdelay(1);
			ucdata = reg_read(client, REG_TX_DDC_STATUS);
			if (ucdata & B_TX_DDC_DONE) {
				break;
			}
			if (ucdata & B_TX_DDC_ERROR) {
				dev_err(&priv->hdmi->dev, "it66121_read_edid_block(): DDC_STATUS = %02X,fail.\n", (int)ucdata);
				return -1;
			}
		}
		if (TimeOut == 0) {
			dev_err(&priv->hdmi->dev, "it66121_read_edid_block(): DDC TimeOut %d . \n", (int)ucdata);
			return -1;
		}

		do {
			*(pBuff++) = reg_read(client, REG_TX_DDC_READFIFO);
			ReqCount--;
		}while (ReqCount > 0);

	}

	return 0;
}
static int it66121_connector_get_modes(struct drm_connector *connector) {
	struct it66121_priv *priv = conn_to_it66121_priv(connector);
	struct edid *edid;
	int n;



	edid = drm_do_get_edid(connector, it66121_read_edid_block, priv);


	if (!edid) {
		dev_warn(&priv->hdmi->dev, "failed to read EDID\n");
		return 0;
	}

	drm_mode_connector_update_edid_property(connector, edid);
	n = drm_add_edid_modes(connector, edid);
	//priv->is_hdmi_sink = drm_detect_hdmi_monitor(edid);
	drm_edid_to_eld(connector, edid);

	kfree(edid);
	printk("it66121_connector_get_modes->color_formats: %x", connector->display_info.color_formats);
	return n;
}

static const struct drm_connector_helper_funcs it66121_connector_helper_funcs = {
	.get_modes = it66121_connector_get_modes,
	.mode_valid = it66121_connector_mode_valid,
	.best_encoder = it66121_connector_best_encoder,
};
static void it66121_encoder_destroy(struct drm_encoder *encoder) {
	struct it66121_priv *priv = enc_to_it66121_priv(encoder);

	drm_encoder_cleanup(encoder);
}
static const struct drm_encoder_funcs it66121_encoder_funcs = {
	.destroy = it66121_encoder_destroy,
};
static void it66121_disable_video_output(struct it66121_priv *priv) {
	struct i2c_client *client = priv->hdmi;
	u8 udata;
	udata = reg_read(client, REG_TX_SW_RST) | B_HDMITX_VID_RST;
	reg_write(client, REG_TX_SW_RST, udata);
	reg_write(client, REG_TX_AFE_DRV_CTRL, B_TX_AFE_DRV_RST | B_TX_AFE_DRV_PWD);
	write_a_entry(priv, 0x62, 0x90, 0x00);
	write_a_entry(priv, 0x64, 0x89, 0x00);

}
static void it66121_disable_audio_output(struct it66121_priv *priv) {
	write_a_entry(priv, REG_TX_SW_RST, (B_HDMITX_AUD_RST | B_TX_AREF_RST), (B_HDMITX_AUD_RST | B_TX_AREF_RST));
	write_a_entry(priv, 0x0F, 0x10, 0x10);
}

static void it66121_encoder_dpms(struct drm_encoder *encoder, int mode) {
	struct it66121_priv *priv = enc_to_it66121_priv(encoder);
	if (mode == DRM_MODE_DPMS_OFF) {
		it66121_disable_video_output(priv);
		it66121_disable_audio_output(priv);
	}
}

static void it66121_encoder_prepare(struct drm_encoder *encoder) {
	struct it66121_priv *priv = enc_to_it66121_priv(encoder);
	dev_err(&priv->hdmi->dev, "it66121_encoder_prepare\n");
}
static void it66121_dump_reg(struct it66121_priv *priv) {
	struct i2c_client *client = priv->hdmi;
	int i, j, n;
	u8 ucData;
	u8 str[128];
	n = 0;

	pr_cont("[%s]\n", __FUNCTION__);
	n += sprintf(str + n, "       ");
	for (j = 0; j < 16; j++) {
		n += sprintf(str + n, " %02X", (int)j);
		if ((j == 3) || (j == 7) || (j == 11)) {
			n += sprintf(str + n, "  ");
		}
	}
	printk("%s\n", str);
	printk("        -----------------------------------------------------\n");

	it66121_switch_blank(priv, 0);
	n = 0;
	for (i = 0; i < 0x100; i += 16) {
		n += sprintf(str + n, "[%3X]  ", i);
		for (j = 0; j < 16; j++) {
			if ((i + j) != 0x17) {
				ucData = reg_read(client, (u8)((i + j) & 0xFF));
				n += sprintf(str + n, " %02X", (int)ucData);
			} else {
				n += sprintf(str + n, " XX"); // for DDC FIFO
			}
			if ((j == 3) || (j == 7) || (j == 11)) {
				n += sprintf(str + n, " -");
			}
		}
		printk("%s\n", str);
		n = 0;
		if ((i % 0x40) == 0x30) {
			pr_cont("        -----------------------------------------------------\n");
		}
	}
	it66121_switch_blank(priv, 1);
	n = 0;
	for (i = 0x130; i < 0x200; i += 16) {
		n += sprintf(str + n, "[%3X]  ", i);
		for (j = 0; j < 16; j++) {
			ucData = reg_read(client, (u8)((i + j) & 0xFF));
			n += sprintf(str + n, " %02X", (int)ucData);
			if ((j == 3) || (j == 7) || (j == 11)) {
				n += sprintf(str + n, " -");
			}
		}
		printk("%s\n", str);
		n = 0;
		if ((i % 0x40) == 0x20) {
			pr_cont("        -----------------------------------------------------\n");
		}
	}
	pr_cont("        -----------------------------------------------------\n");
	it66121_switch_blank(priv, 0);
}

static void it66121_encoder_commit(struct drm_encoder *encoder) {
	struct it66121_priv *priv = enc_to_it66121_priv(encoder);
#if DEBUG
	//it66121_dump_reg(priv);
#endif
}
static void it66121_set_CSC_scale(struct it66121_priv *priv,
								  u8 input_color_mode) {
	struct i2c_client *client = priv->hdmi;
	u8 csc = 0;
	u8 filter = 0;
	u8 udata = 0;
	int i;
	switch (input_color_mode & F_MODE_CLRMOD_MASK) {
	case F_MODE_YUV444:
		switch (OUTPUT_COLOR_MODE & F_MODE_CLRMOD_MASK) {
		case F_MODE_YUV444:
			csc = B_HDMITX_CSC_BYPASS;
			break;

		case F_MODE_YUV422:
			if (input_color_mode & F_VIDMODE_EN_UDFILT) { // YUV444 to YUV422 need up/down filter for processing.
				filter |= B_TX_EN_UDFILTER;
			}
			csc = B_HDMITX_CSC_BYPASS;
			break;
		case F_MODE_RGB444:
			csc = B_HDMITX_CSC_YUV2RGB;
			if (input_color_mode & F_VIDMODE_EN_DITHER) { // YUV444 to RGB24 need dither
				filter |= B_TX_EN_DITHER | B_TX_DNFREE_GO;
			}
			break;
		}
		break;

	case F_MODE_YUV422:
		switch (OUTPUT_COLOR_MODE & F_MODE_CLRMOD_MASK) {
		case F_MODE_YUV444:
			csc = B_HDMITX_CSC_BYPASS;
			if (input_color_mode & F_VIDMODE_EN_UDFILT) { // YUV422 to YUV444 need up filter
				filter |= B_TX_EN_UDFILTER;
			}
			if (input_color_mode & F_VIDMODE_EN_DITHER) { // YUV422 to YUV444 need dither
				filter |= B_TX_EN_DITHER | B_TX_DNFREE_GO;
			}
			break;
		case F_MODE_YUV422:
			csc = B_HDMITX_CSC_BYPASS;

			break;

		case F_MODE_RGB444:
			csc = B_HDMITX_CSC_YUV2RGB;
			if (input_color_mode & F_VIDMODE_EN_UDFILT) { // YUV422 to RGB24 need up/dn filter.
				filter |= B_TX_EN_UDFILTER;
			}
			if (input_color_mode & F_VIDMODE_EN_DITHER) { // YUV422 to RGB24 need dither
				filter |= B_TX_EN_DITHER | B_TX_DNFREE_GO;
			}
			break;
		}
		break;

	case F_MODE_RGB444:
		switch (OUTPUT_COLOR_MODE & F_MODE_CLRMOD_MASK) {
		case F_MODE_YUV444:
			csc = B_HDMITX_CSC_RGB2YUV;

			if (INPUT_COLOR_MODE & F_VIDMODE_EN_DITHER) { // RGB24 to YUV444 need dither
				filter |= B_TX_EN_DITHER | B_TX_DNFREE_GO;
			}
			break;

		case F_MODE_YUV422:
			if (input_color_mode & F_VIDMODE_EN_UDFILT) { // RGB24 to YUV422 need down filter.
				filter |= B_TX_EN_UDFILTER;
			}
			if (input_color_mode & F_VIDMODE_EN_DITHER) { // RGB24 to YUV422 need dither
				filter |= B_TX_EN_DITHER | B_TX_DNFREE_GO;
			}
			csc = B_HDMITX_CSC_RGB2YUV;
			break;

		case F_MODE_RGB444:
			csc = B_HDMITX_CSC_BYPASS;
			break;
		}
		break;
	}

	if (csc == B_HDMITX_CSC_RGB2YUV) {
		switch (input_color_mode & (F_VIDMODE_ITU709 | F_VIDMODE_16_235)) {
		case F_VIDMODE_ITU709 | F_VIDMODE_16_235:
			for (i = 0; i < SIZEOF_CSCMTX; i++) {
				reg_write(client, REG_TX_CSC_YOFF + i, bCSCMtx_RGB2YUV_ITU709_16_235[i]);
			}
			break;
		case F_VIDMODE_ITU709 | F_VIDMODE_0_255:
			for (i = 0; i < SIZEOF_CSCMTX; i++) {
				reg_write(client, REG_TX_CSC_YOFF + i, bCSCMtx_RGB2YUV_ITU709_0_255[i]);
			}
			break;
		case F_VIDMODE_ITU601 | F_VIDMODE_16_235:
			for (i = 0; i < SIZEOF_CSCMTX; i++) {
				reg_write(client, REG_TX_CSC_YOFF + i, bCSCMtx_RGB2YUV_ITU601_16_235[i]);
			}
			break;
		case F_VIDMODE_ITU601 | F_VIDMODE_0_255:
		default:
			for (i = 0; i < SIZEOF_CSCMTX; i++) {
				reg_write(client, REG_TX_CSC_YOFF + i, bCSCMtx_RGB2YUV_ITU601_0_255[i]);
			}
			break;
		}
	}

	if (csc == B_HDMITX_CSC_YUV2RGB) {
		switch (input_color_mode & (F_VIDMODE_ITU709 | F_VIDMODE_16_235)) {
		case F_VIDMODE_ITU709 | F_VIDMODE_16_235:
			for (i = 0; i < SIZEOF_CSCMTX; i++) {
				reg_write(client, REG_TX_CSC_YOFF + i, bCSCMtx_YUV2RGB_ITU709_16_235[i]);
			}
			break;
		case F_VIDMODE_ITU709 | F_VIDMODE_0_255:
			for (i = 0; i < SIZEOF_CSCMTX; i++) {
				reg_write(client, REG_TX_CSC_YOFF + i, bCSCMtx_YUV2RGB_ITU709_0_255[i]);
			}
			break;
		case F_VIDMODE_ITU601 | F_VIDMODE_16_235:
			for (i = 0; i < SIZEOF_CSCMTX; i++) {
				reg_write(client, REG_TX_CSC_YOFF + i, bCSCMtx_YUV2RGB_ITU601_16_235[i]);
			}
			break;
		case F_VIDMODE_ITU601 | F_VIDMODE_0_255:
		default:
			for (i = 0; i < SIZEOF_CSCMTX; i++) {
				reg_write(client, REG_TX_CSC_YOFF + i, bCSCMtx_YUV2RGB_ITU601_0_255[i]);
			}
			break;
		}
	}
	if (csc == B_HDMITX_CSC_BYPASS) {
		write_a_entry(priv, 0xF, 0x10, 0x10);
	} else {
		write_a_entry(priv, 0xF, 0x10, 0x00);
	}
	udata = reg_read(client, REG_TX_CSC_CTRL) & ~(M_TX_CSC_SEL | B_TX_DNFREE_GO | B_TX_EN_DITHER | B_TX_EN_UDFILTER);
	udata |= filter | csc;

	reg_write(client, REG_TX_CSC_CTRL, udata);
}
static void it66121_setup_AFE(struct it66121_priv *priv, u8 level) {
	struct i2c_client *client = priv->hdmi;
	reg_write(client, REG_TX_AFE_DRV_CTRL, B_TX_AFE_DRV_RST); /* 0x10 */
	switch (level) {
	case 1:
		write_a_entry(priv, 0x62, 0x90, 0x80);
		write_a_entry(priv, 0x64, 0x89, 0x80);
		write_a_entry(priv, 0x68, 0x10, 0x80);
		break;
	default:
		write_a_entry(priv, 0x62, 0x90, 0x10);
		write_a_entry(priv, 0x64, 0x89, 0x09);
		write_a_entry(priv, 0x68, 0x10, 0x10);
		break;
	}
	write_a_entry(priv, REG_TX_SW_RST, B_TX_REF_RST_HDMITX | B_HDMITX_VID_RST, 0);
	reg_write(client, REG_TX_AFE_DRV_CTRL, 0);
	mdelay(1);
}

static void it66121_config_avi_info_frame(struct it66121_priv *priv,
										  u8 aspec,
										  u8 colorimetry,
										  u8 pixelrep,
										  int vic) {
	struct i2c_client *client = priv->hdmi;
	u8 AVI_DB[12];
	int checksum;
	int i;

	switch (OUTPUT_COLOR_MODE) {
	case F_MODE_YUV444:
		AVI_DB[0] = (2 << 5) | (1 << 4);
		break;
	case F_MODE_YUV422:
		AVI_DB[0] = (1 << 5) | (1 << 4);
		break;
	case F_MODE_RGB444:
	default:
		AVI_DB[0] = (0 << 5) | (1 << 4);
		break;
	}
	AVI_DB[0] |= 0x02;
	AVI_DB[1] = 8;
	AVI_DB[1] |= (aspec != HDMI_16x9) ? (1 << 4) : (2 << 4); // 4:3 or 16:9
	AVI_DB[1] |= (colorimetry != HDMI_ITU709) ? (1 << 6) : (2 << 6); // 4:3 or 16:9
	AVI_DB[2] = 0;
	AVI_DB[3] = vic;
	AVI_DB[4] =  pixelrep & 3;
	AVI_DB[5] = 0;
	AVI_DB[6] = 0;
	AVI_DB[7] = 0;
	AVI_DB[8] = 0;
	AVI_DB[9] = 0;
	AVI_DB[10] = 0;
	AVI_DB[11] = 0;
	AVI_DB[12] = 0;

	it66121_switch_blank(priv, 1);
	reg_write(client, REG_TX_AVIINFO_DB1, AVI_DB[0]);
	reg_write(client, REG_TX_AVIINFO_DB2, AVI_DB[1]);
	reg_write(client, REG_TX_AVIINFO_DB3, AVI_DB[2]);
	reg_write(client, REG_TX_AVIINFO_DB4, AVI_DB[3]);
	reg_write(client, REG_TX_AVIINFO_DB5, AVI_DB[4]);
	reg_write(client, REG_TX_AVIINFO_DB6, AVI_DB[5]);
	reg_write(client, REG_TX_AVIINFO_DB7, AVI_DB[6]);
	reg_write(client, REG_TX_AVIINFO_DB8, AVI_DB[7]);
	reg_write(client, REG_TX_AVIINFO_DB9, AVI_DB[8]);
	reg_write(client, REG_TX_AVIINFO_DB10, AVI_DB[9]);
	reg_write(client, REG_TX_AVIINFO_DB11, AVI_DB[10]);
	reg_write(client, REG_TX_AVIINFO_DB12, AVI_DB[11]);
	reg_write(client, REG_TX_AVIINFO_DB13, AVI_DB[12]);
	for (i = 0, checksum = 0; i < 13; i++) {
		checksum -= AVI_DB[i];
	}
	checksum -= AVI_INFOFRAME_VER + AVI_INFOFRAME_TYPE + AVI_INFOFRAME_LEN;
	reg_write(client, REG_TX_AVIINFO_SUM, checksum);

	it66121_switch_blank(priv, 0);
	reg_write(client, REG_TX_AVI_INFOFRM_CTRL, B_TX_ENABLE_PKT | B_TX_REPEAT_PKT);
}

/**
 * To enable the video of IT66121, the input signal type and
 * output TMDS should be programmed. The following sequence is to
 * set the video mode:
 * 1. Set regC1[0] = '1' for AVMUTE the output.
 * 2. Programming Input Signal Type
 * 3. Set color space converting by the input color space and output color space.
 * 4. Set AFE by the input video pixel clock.
 * 5. Set HDMI package or DVI mode.
 * 6. Set HDCP if necessary.
 * 7. Set Audio if necessary.
 * 8. Clear the AVMUTE by regC1[0] = '1' and regC6 = 0x03.
 *
 */
static void it66121_enable_video_output(struct it66121_priv *priv,
										struct drm_display_mode *mode) {
	struct i2c_client *client = priv->hdmi;
	int name_len = 0;
	u8 is_high_clk = 0;
	u8 pixelrep = 0;
	u8 input_color_mode = F_MODE_RGB444;
	u8 aspec = 0;
	u8 Colorimetry = 0;
	u8 udata;
	u8 vic;

	vic = drm_match_cea_mode(mode);
	name_len = strlen(mode->name);
	if (mode->name[name_len - 1] == 'i') {
		pixelrep = 1;
	}
	if ((pixelrep + 1) * (mode->clock * 1000) > 80000000L) {
		is_high_clk = 1;
	}
	if (mode->hdisplay * 9 == mode->vdisplay * 16) {
		aspec = HDMI_16x9;
		Colorimetry = HDMI_ITU709;
	}
	if (mode->hdisplay * 3 == mode->vdisplay * 4) {
		aspec = HDMI_4x3;
		Colorimetry = HDMI_ITU601;
	}
	if (Colorimetry == HDMI_ITU709) {
		input_color_mode |= F_VIDMODE_ITU709;
	} else {
		input_color_mode &= ~F_VIDMODE_ITU709;
	}
	if (pixelrep == 0 && mode->hdisplay == 640 &&
		mode->vdisplay == 480 && mode->vrefresh == 60) {
		input_color_mode |= F_VIDMODE_16_235;
	} else {
		input_color_mode &= ~F_VIDMODE_16_235;
	}

	is_high_clk ? reg_write(client, REG_TX_PLL_CTRL, 0x30 /*0xff*/) :
		reg_write(client, REG_TX_PLL_CTRL, 0x00);

	reg_write(client, REG_TX_SW_RST,	B_HDMITX_VID_RST |
			  B_HDMITX_AUD_RST |
			  B_TX_AREF_RST |
			  B_TX_HDCP_RST_HDMITX);

	// 2009/12/09 added by jau-chih.tseng@ite.com.tw
	it66121_switch_blank(priv, 1);
	reg_write(client, REG_TX_AVIINFO_DB1, 0x00);
	it66121_switch_blank(priv, 0);
	//~jau-chih.tseng@ite.com.tw

	/*Set regC1[0] = '1' for AVMUTE the output, only support hdmi mode now*/
	//write_a_entry(priv, REG_TX_GCP, B_TX_SETAVMUTE,  B_TX_SETAVMUTE);
	write_a_entry(priv, REG_TX_GCP, B_TX_SETAVMUTE,  0);
	reg_write(client, REG_TX_PKT_GENERAL_CTRL, B_TX_ENABLE_PKT | B_TX_REPEAT_PKT);

	/* Programming Input Signal Type
	 * InputColorMode: F_MODE_RGB444
	 *				   F_MODE_YUV422
	 *  			   F_MODE_YUV444
	 *
	 *bInputSignalType: T_MODE_PCLKDIV2
	 *  				T_MODE_CCIR656
	 *  				T_MODE_SYNCEMB
	 *  				T_MODE_INDDR
	*/
	udata = reg_read(client, REG_TX_INPUT_MODE);
	udata &= ~(M_TX_INCOLMOD | B_TX_2X656CLK | B_TX_SYNCEMB | B_TX_INDDR | B_TX_PCLKDIV2);
	udata |= 0x01; //input clock delay 1 for 1080P DDR
	udata |= input_color_mode & F_MODE_CLRMOD_MASK; //define in it66121.h, F_MODE_RGB444
	udata |= INPUT_SIGNAL_TYPE; //define in it66121.h,  24 bit sync seperate
	reg_write(client, REG_TX_INPUT_MODE, udata);

	/*
	 * Set color space converting by the input color space and output color space.
	*/
	it66121_set_CSC_scale(priv, input_color_mode);
	/*hdmi mode*/
	reg_write(client, REG_TX_HDMI_MODE, B_TX_HDMI_MODE);
	/*dvi mode*/
	//reg_write(client,REG_TX_HDMI_MODE, B_TX_DVI_MODE);
#ifdef INVERT_VID_LATCHEDGE
	udata = reg_read(client, REG_TX_CLK_CTRL1);
	udata |= B_TX_VDO_LATCH_EDGE;
	reg_write(client, REG_TX_CLK_CTRL1, udata);
#endif

	it66121_setup_AFE(priv, is_high_clk); // pass if High Freq request
	reg_write(client, REG_TX_SW_RST, B_HDMITX_AUD_RST |
			  B_TX_AREF_RST |
			  B_TX_HDCP_RST_HDMITX);

	/*fire APFE*/
	it66121_switch_blank(priv, 0);
	reg_write(client, REG_TX_AFE_DRV_CTRL, 0);

	it66121_config_avi_info_frame(priv, aspec, Colorimetry, pixelrep, vic);


	printk("color_formats: %x", priv->connector.display_info.color_formats);
}


static void it66121_encoder_mode_set(struct drm_encoder *encoder,
									 struct drm_display_mode *mode,
									 struct drm_display_mode *adjusted_mode) {
	struct it66121_priv *priv = enc_to_it66121_priv(encoder);
	it66121_enable_video_output(priv, mode);

}
static void it66121_encoder_save(struct drm_encoder *encoder) {
}

static void it66121_encoder_restore(struct drm_encoder *encoder) {
}
static bool t66121_encoder_mode_fixup(struct drm_encoder *encoder,
									  const struct drm_display_mode *mode,
									  struct drm_display_mode *adjusted_mode) {
	return true;
}

static const struct drm_encoder_helper_funcs it66121_encoder_helper_funcs = {
	.dpms = it66121_encoder_dpms,
	.save =  it66121_encoder_save,
	.restore = it66121_encoder_restore,
	.mode_fixup = t66121_encoder_mode_fixup,
	.prepare = it66121_encoder_prepare,
	.commit = it66121_encoder_commit,
	.mode_set = it66121_encoder_mode_set,
};

void it66121_clear_Interrupt(struct i2c_client *client) {
	char intclr3, intdata4;
	intdata4 = reg_read(client, 0xee);
	intclr3 = reg_read(client, REG_TX_SYS_STATUS);
	intclr3 = intclr3 | B_TX_CLR_AUD_CTS | B_TX_INTACTDONE;
	if (intdata4) {
		reg_write(client, 0xEE, intdata4); // clear ext interrupt ;
	}

	reg_write(client, REG_TX_INT_CLR0, 0xFF);
	reg_write(client, REG_TX_INT_CLR1, 0xFF);
	reg_write(client, REG_TX_SYS_STATUS, intclr3); // clear interrupt.
	intclr3 &= ~(B_TX_INTACTDONE);
	reg_write(client, REG_TX_SYS_STATUS, intclr3); // INTACTDONE reset to zero.
}

static irqreturn_t it66121_thread_interrupt(int irq, void *data) {
	struct it66121_priv *priv = data;
	struct i2c_client *client = priv->hdmi;
	u8 sysstat;
	u8 intdata1;
	u8 intdata2;
	u8 intdata3;
	u8 udata;
	sysstat = reg_read(priv->hdmi, REG_TX_SYS_STATUS); //0x0E


	intdata1 = reg_read(client, REG_TX_INT_STAT1); //0x06
	intdata2 = reg_read(client, REG_TX_INT_STAT2); //0x07
	intdata3 = reg_read(client, REG_TX_INT_STAT3); //0x08



	if (priv->powerstatus == 0) {
		it66121_power_on(priv);
	}
	it66121_clear_Interrupt(client);

	if (intdata1 & B_TX_INT_DDCFIFO_ERR) {
		//dev_err(&client->dev, "DDC FIFO Error.\n");
		/*clear ddc fifo*/
		reg_write(client, REG_TX_DDC_MASTER_CTRL, B_TX_MASTERDDC | B_TX_MASTERHOST);
		reg_write(client, REG_TX_DDC_CMD, CMD_FIFO_CLR);
	}
	if (intdata1 & B_TX_INT_DDC_BUS_HANG) {
		//dev_err(&client->dev, "DDC BUS HANG.\n");
		/*abort ddc*/
		it66121_abort_DDC(client);
	}
	if (intdata1 & B_TX_INT_AUD_OVERFLOW) {
		//dev_err(&client->dev, "AUDIO FIFO OVERFLOW.\n");
		write_a_entry(priv, REG_TX_SW_RST, (B_HDMITX_AUD_RST | B_TX_AREF_RST),
					  (B_HDMITX_AUD_RST | B_TX_AREF_RST));
		udata = reg_read(client, REG_TX_SW_RST);
		reg_write(client, REG_TX_SW_RST, udata & (~(B_HDMITX_AUD_RST | B_TX_AREF_RST)));
	}
	if (intdata3 & B_TX_INT_VIDSTABLE) {
		//dev_info(&client->dev, "it66121 interrupt video enabled\n");
		sysstat = reg_read(client, REG_TX_SYS_STATUS);
		if (sysstat & B_TXVIDSTABLE) {
			/*fire APFE*/
			it66121_switch_blank(priv, 0);
			reg_write(client, REG_TX_AFE_DRV_CTRL, 0);
		}
	}

	struct drm_device *dev = priv->encoder.dev;

	if (dev)
		drm_kms_helper_hotplug_event(dev);

	//printk("it66121_thread_interrupt\n");
	return IRQ_HANDLED;
}
static void it66121_free(struct it66121_priv *priv) {
	if (priv->hdmi->irq) free_irq(priv->hdmi->irq, priv);
	if (priv->audio_pdev) platform_device_unregister(priv->audio_pdev);
}
static void it66121_aud_config_aai(struct it66121_priv *priv) {

	struct i2c_client *client = priv->hdmi;
	u8 aud_db[AUDIO_INFOFRAME_LEN];


	unsigned int  checksum = 0;
	u8 i;
	aud_db[0] = 1;
	for (i = 1; i < AUDIO_INFOFRAME_LEN; i++) {
		aud_db[i] = 0;
	}
	it66121_switch_blank(priv, 1);
	checksum = 0x100 - (AUDIO_INFOFRAME_VER + AUDIO_INFOFRAME_TYPE + AUDIO_INFOFRAME_LEN);
	reg_write(client, REG_TX_PKT_AUDINFO_CC, aud_db[0]);
	checksum -= reg_read(client, REG_TX_PKT_AUDINFO_CC);
	checksum &= 0xFF;

	reg_write(client, REG_TX_PKT_AUDINFO_SF, aud_db[1]);
	checksum -= reg_read(client, REG_TX_PKT_AUDINFO_SF);
	checksum &= 0xFF;

	reg_write(client, REG_TX_PKT_AUDINFO_CA, aud_db[3]);
	checksum -= reg_read(client, REG_TX_PKT_AUDINFO_CA);
	checksum &= 0xFF;

	reg_write(client, REG_TX_PKT_AUDINFO_DM_LSV, aud_db[4]);
	checksum -= reg_read(client, REG_TX_PKT_AUDINFO_DM_LSV);
	checksum &= 0xFF;

	reg_write(client, REG_TX_PKT_AUDINFO_SUM, checksum);

	it66121_switch_blank(priv, 0);
	reg_write(client, REG_TX_AUD_INFOFRM_CTRL, B_TX_ENABLE_PKT | B_TX_REPEAT_PKT);
}
static void it66121_aud_set_fs(struct it66121_priv *priv, u8 fs) {
	u32 n;
	u32  LastCTS = 0;
	u8 HBR_mode;
	u8 udata;
	struct i2c_client *client = priv->hdmi;

	if (B_TX_HBR & reg_read(client, REG_TX_AUD_HDAUDIO)) {
		HBR_mode = 1;
	} else {
		HBR_mode = 0;
	}
	printk("HBR_mode:%d\n", HBR_mode);
	switch (fs) {
	case AUDFS_32KHz:
		n = 4096; break;
	case AUDFS_44p1KHz:
		n = 6272; break;
	case AUDFS_48KHz:
		n = 6144; break;
	case AUDFS_88p2KHz:
		n = 12544; break;
	case AUDFS_96KHz:
		n = 12288; break;
	case AUDFS_176p4KHz:
		n = 25088; break;
	case AUDFS_192KHz:
		n = 24576; break;
	case AUDFS_768KHz:
		n = 24576; break;
	default:
		n = 6144;
	}
	// tr_printf((" n = %ld\n",n));
	it66121_switch_blank(priv, 1);
	reg_write(client, REGPktAudN0, (u8)((n)&0xFF));
	reg_write(client, REGPktAudN1, (u8)((n >> 8) & 0xFF));
	reg_write(client, REGPktAudN2, (u8)((n >> 16) & 0xF));


	reg_write(client, REGPktAudCTS0, (u8)((LastCTS)&0xFF));
	reg_write(client, REGPktAudCTS1, (u8)((LastCTS >> 8) & 0xFF));
	reg_write(client, REGPktAudCTS2, (u8)((LastCTS >> 16) & 0xF));
	it66121_switch_blank(priv, 0);

	reg_write(client, 0xF8, 0xC3);
	reg_write(client, 0xF8, 0xA5);

	udata =  reg_read(client, REG_TX_PKT_SINGLE_CTRL);
	udata &= ~B_TX_SW_CTS;
	reg_write(client, REG_TX_PKT_SINGLE_CTRL, udata);

	reg_write(client, 0xF8, 0xFF);

	if (0 == HBR_mode) { //LPCM
		it66121_switch_blank(priv, 1);
		fs = AUDFS_768KHz;
		reg_write(client, REG_TX_AUDCHST_CA_FS, 0x00 | fs);
		fs = ~fs; // OFS is the one's complement of FS
		udata = (0x0f & reg_read(client, REG_TX_AUDCHST_OFS_WL));
		reg_write(client, REG_TX_AUDCHST_OFS_WL, (fs << 4) | udata);
		it66121_switch_blank(priv, 0);
	}
}
static void it66121_set_ChStat(struct it66121_priv *priv, u8 ucIEC60958ChStat[]) {
	u8 udata;
	struct i2c_client *client = priv->hdmi;

	it66121_switch_blank(priv, 1);
	udata = (ucIEC60958ChStat[0] << 1) & 0x7C;
	reg_write(client, REG_TX_AUDCHST_MODE, udata);
	reg_write(client, REG_TX_AUDCHST_CAT, ucIEC60958ChStat[1]); // 192, audio CATEGORY
	reg_write(client, REG_TX_AUDCHST_SRCNUM, ucIEC60958ChStat[2] & 0xF);
	reg_write(client, REG_TX_AUD0CHST_CHTNUM, (ucIEC60958ChStat[2] >> 4) & 0xF);
	reg_write(client, REG_TX_AUDCHST_CA_FS, ucIEC60958ChStat[3]); // choose clock
	reg_write(client, REG_TX_AUDCHST_OFS_WL, ucIEC60958ChStat[4]);
	it66121_switch_blank(priv, 0);
}
static void it66121_set_HBRAudio(struct it66121_priv *priv) {
	u8 udata;
	struct i2c_client *client = priv->hdmi;
	it66121_switch_blank(priv, 0);

	reg_write(client, REG_TX_AUDIO_CTRL1, 0x47); // regE1 bOutputAudioMode should be loaded from ROM image.
	reg_write(client, REG_TX_AUDIO_FIFOMAP, 0xE4); // default mapping.

	if (CONFIG_INPUT_AUDIO_SPDIF) {
		reg_write(client, REG_TX_AUDIO_CTRL0, M_TX_AUD_BIT | B_TX_AUD_SPDIF);
		reg_write(client, REG_TX_AUDIO_CTRL3, B_TX_CHSTSEL);
	} else {
		reg_write(client, REG_TX_AUDIO_CTRL0, M_TX_AUD_BIT);
		reg_write(client, REG_TX_AUDIO_CTRL3, 0);
	}
	reg_write(client, REG_TX_AUD_SRCVALID_FLAT, 0x08);
	reg_write(client, REG_TX_AUD_HDAUDIO, B_TX_HBR); // regE5 = 0 ;

	//uc = HDMITX_ReadI2C_Byte(client,REG_TX_CLK_CTRL1);
	//uc &= ~M_TX_AUD_DIV ;
	//HDMITX_WriteI2C_Byte(client,REG_TX_CLK_CTRL1, uc);

	if (CONFIG_INPUT_AUDIO_SPDIF) {
		u8 i;
		for (i = 0; i < 100; i++) {
			if (reg_read(client, REG_TX_CLK_STATUS2) & B_TX_OSF_LOCK) {
				break; // stable clock.
			}
		}
		reg_write(client, REG_TX_AUDIO_CTRL0, M_TX_AUD_BIT |
				  B_TX_AUD_SPDIF | B_TX_AUD_EN_SPDIF);
	} else {
		reg_write(client, REG_TX_AUDIO_CTRL0, M_TX_AUD_BIT |
				  B_TX_AUD_EN_I2S3 |
				  B_TX_AUD_EN_I2S2 |
				  B_TX_AUD_EN_I2S1 |
				  B_TX_AUD_EN_I2S0);
	}
	udata = reg_read(client, 0x5c);
	udata &= ~(1 << 6);
	reg_write(client, 0x5c, udata);

	//hdmiTxDev[0].bAudioChannelEnable = reg_read(client, REG_TX_AUDIO_CTRL0);
	// reg_write(client,REG_TX_SW_RST, rst  );
}

static void it66121_set_DSDAudio(struct i2c_client *client) {
	// to be continue
	// u8 rst;
	// rst = reg_read(client,REG_TX_SW_RST);

	//red_write(client,REG_TX_SW_RST, rst | (B_HDMITX_AUD_RST|B_TX_AREF_RST) );

	reg_write(client, REG_TX_AUDIO_CTRL1, 0x41); // regE1 bOutputAudioMode should be loaded from ROM image.
	reg_write(client, REG_TX_AUDIO_FIFOMAP, 0xE4); // default mapping.

	reg_write(client, REG_TX_AUDIO_CTRL0, M_TX_AUD_BIT);
	reg_write(client, REG_TX_AUDIO_CTRL3, 0);

	reg_write(client, REG_TX_AUD_SRCVALID_FLAT, 0x00);
	reg_write(client, REG_TX_AUD_HDAUDIO, B_TX_DSD); // regE5 = 0 ;
													 //red_write(client,REG_TX_SW_RST, rst & ~(B_HDMITX_AUD_RST|B_TX_AREF_RST) );

	//uc = reg_read(client,REG_TX_CLK_CTRL1);
	//uc &= ~M_TX_AUD_DIV ;
	//red_write(client,REG_TX_CLK_CTRL1, uc);

	reg_write(client, REG_TX_AUDIO_CTRL0, M_TX_AUD_BIT |
			  B_TX_AUD_EN_I2S3 |
			  B_TX_AUD_EN_I2S2 |
			  B_TX_AUD_EN_I2S1 |
			  B_TX_AUD_EN_I2S0);
}
static void it66121_set_NLPCMAudio(struct it66121_priv *priv) { // no Source Num, no I2S.
	u8 AudioEnable, AudioFormat;
	u8 i;
	struct i2c_client *client = priv->hdmi;
	AudioFormat = 0x01; // NLPCM must use standard I2S mode.
	if (CONFIG_INPUT_AUDIO_SPDIF) {
		AudioEnable = M_TX_AUD_BIT | B_TX_AUD_SPDIF;
	} else {
		AudioEnable = M_TX_AUD_BIT;
	}

	it66121_switch_blank(priv, 0);
	// HDMITX_WriteI2C_Byte(client,REG_TX_AUDIO_CTRL0, M_TX_AUD_24BIT|B_TX_AUD_SPDIF);
	reg_write(client, REG_TX_AUDIO_CTRL0, AudioEnable);
	//HDMITX_AndREG_Byte(REG_TX_SW_RST,~(B_HDMITX_AUD_RST|B_TX_AREF_RST));

	reg_write(client, REG_TX_AUDIO_CTRL1, 0x01); // regE1 bOutputAudioMode should be loaded from ROM image.
	reg_write(client, REG_TX_AUDIO_FIFOMAP, 0xE4); // default mapping.

#ifdef USE_SPDIF_CHSTAT
	reg_write(client, REG_TX_AUDIO_CTRL3, B_TX_CHSTSEL);
#else // not USE_SPDIF_CHSTAT
	reg_write(client, REG_TX_AUDIO_CTRL3, 0);
#endif // USE_SPDIF_CHSTAT

	reg_write(client, REG_TX_AUD_SRCVALID_FLAT, 0x00);
	reg_write(client, REG_TX_AUD_HDAUDIO, 0x00); // regE5 = 0 ;

	if (CONFIG_INPUT_AUDIO_SPDIF) {
		for (i = 0; i < 100; i++) {
			if (reg_read(client, REG_TX_CLK_STATUS2) & B_TX_OSF_LOCK) {
				break; // stable clock.
			}
		}
	}
	priv->AudioChannelEnable = AudioEnable;
	reg_write(client, REG_TX_AUDIO_CTRL0, AudioEnable | B_TX_AUD_EN_I2S0);
}
static void it66121_set_LPCMAudio(struct it66121_priv *priv,
								  u8 AudioSrcNum, u8 AudSWL) {
	struct i2c_client *client = priv->hdmi;
	u8 AudioEnable, AudioFormat;

	AudioEnable = 0;
	AudioFormat = 0;

	switch (AudSWL) {
	case 16:
		AudioEnable |= M_TX_AUD_16BIT;
		break;
	case 18:
		AudioEnable |= M_TX_AUD_18BIT;
		break;
	case 20:
		AudioEnable |= M_TX_AUD_20BIT;
		break;
	case 24:
	default:
		AudioEnable |= M_TX_AUD_24BIT;
		break;
	}
	if (CONFIG_INPUT_AUDIO_SPDIF) {
		AudioFormat &= ~0x40;
		AudioEnable |= B_TX_AUD_SPDIF | B_TX_AUD_EN_I2S0;
	} else {
		AudioFormat |= 0x40;
		switch (AudioSrcNum) {
		case 4:
			AudioEnable |= B_TX_AUD_EN_I2S3 | B_TX_AUD_EN_I2S2 | B_TX_AUD_EN_I2S1 | B_TX_AUD_EN_I2S0;
			break;

		case 3:
			AudioEnable |= B_TX_AUD_EN_I2S2 | B_TX_AUD_EN_I2S1 | B_TX_AUD_EN_I2S0;
			break;

		case 2:
			AudioEnable |= B_TX_AUD_EN_I2S1 | B_TX_AUD_EN_I2S0;
			break;

		case 1:
		default:
			AudioFormat &= ~0x40;
			AudioEnable |= B_TX_AUD_EN_I2S0;
			break;

		}
	}
	if (AudSWL != 16) {
		AudioFormat |= 0x01;
	}

	it66121_switch_blank(priv, 0);
	reg_write(client, REG_TX_AUDIO_CTRL0, AudioEnable & 0xF0);

	// regE1 bOutputAudioMode should be loaded from ROM image.
	reg_write(client, REG_TX_AUDIO_CTRL1,
			  AudioFormat |
			  B_TX_AUDFMT_DELAY_1T_TO_WS |
			  B_TX_AUDFMT_RISE_EDGE_SAMPLE_WS
			 );


	reg_write(client, REG_TX_AUDIO_FIFOMAP, 0xE4); // default mapping.
#ifdef USE_SPDIF_CHSTAT
	if (CONFIG_INPUT_AUDIO_SPDIF) {
		reg_write(client, REG_TX_AUDIO_CTRL3, B_TX_CHSTSEL);
	} else {
		reg_write(client, REG_TX_AUDIO_CTRL3, 0);
	}
#else // not USE_SPDIF_CHSTAT
	reg_write(client, REG_TX_AUDIO_CTRL3, 0);
#endif // USE_SPDIF_CHSTAT

	reg_write(client, REG_TX_AUD_SRCVALID_FLAT, 0x00);
	reg_write(client, REG_TX_AUD_HDAUDIO, 0x00); // regE5 = 0 ;
	priv->AudioChannelEnable = AudioEnable;
	if (CONFIG_INPUT_AUDIO_SPDIF) {
		u8 i;
		write_a_entry(priv, 0x5c, (1 << 6), (1 << 6));
		for (i = 0; i < 100; i++) {
			if (reg_read(client, REG_TX_CLK_STATUS2) & B_TX_OSF_LOCK) {
				break; // stable clock.
			}
		}
	}
}

static int it66121_aud_output_config(struct it66121_priv *priv,
									 struct hdmi_codec_params *param) {
	struct i2c_client *client = priv->hdmi;
	u8 udata;
	u8 fs;
	u8 ucIEC60958ChStat[8];


	write_a_entry(priv, REG_TX_SW_RST, (B_HDMITX_AUD_RST | B_TX_AREF_RST),
				  (B_HDMITX_AUD_RST | B_TX_AREF_RST));
	reg_write(client, REG_TX_CLK_CTRL0, B_TX_AUTO_OVER_SAMPLING_CLOCK | B_TX_EXT_256FS | 0x01);

	write_a_entry(priv, 0x0F, 0x10, 0x00); // power on the ACLK

	//use i2s
	udata = reg_read(client, REG_TX_AUDIO_CTRL0);
	udata &= ~B_TX_AUD_SPDIF;
	reg_write(client, REG_TX_AUDIO_CTRL0, udata);


	// one bit audio have no channel status.
	switch (param->sample_rate) {
	case  44100L:
		fs =  AUDFS_44p1KHz; break;
	case  88200L:
		fs =  AUDFS_88p2KHz; break;
	case 176400L:
		fs = AUDFS_176p4KHz; break;
	case  32000L:
		fs =    AUDFS_32KHz; break;
	case  48000L:
		fs =    AUDFS_48KHz; break;
	case  96000L:
		fs =    AUDFS_96KHz; break;
	case 192000L:
		fs =   AUDFS_192KHz; break;
	case 768000L:
		fs =   AUDFS_768KHz; break;
	default:
		//SampleFreq = 48000L;
		fs =    AUDFS_48KHz;
		break; // default, set Fs = 48KHz.
	}
	it66121_aud_set_fs(priv, fs);

	ucIEC60958ChStat[0] = 0;
	ucIEC60958ChStat[1] = 0;
	ucIEC60958ChStat[2] = (param->channels + 1) / 2;

	if (ucIEC60958ChStat[2] < 1) {
		ucIEC60958ChStat[2] = 1;
	} else if (ucIEC60958ChStat[2] > 4) {
		ucIEC60958ChStat[2] = 4;
	}
	ucIEC60958ChStat[3] = fs;
	ucIEC60958ChStat[4] = (((~fs) << 4) & 0xF0) | CHTSTS_SWCODE; // Fs | 24bit word length


	write_a_entry(priv, REG_TX_SW_RST, (B_HDMITX_AUD_RST | B_TX_AREF_RST), B_TX_AREF_RST);

	switch (CNOFIG_INPUT_AUDIO_TYPE) {
	case T_AUDIO_HBR:
		ucIEC60958ChStat[0] |= 1 << 1;
		ucIEC60958ChStat[2] = 0;
		ucIEC60958ChStat[3] &= 0xF0;
		ucIEC60958ChStat[3] |= AUDFS_768KHz;
		ucIEC60958ChStat[4] |= (((~AUDFS_768KHz) << 4) & 0xF0) | 0xB;
		it66121_set_ChStat(priv, ucIEC60958ChStat);
		it66121_set_HBRAudio(priv);

		break;
	case T_AUDIO_DSD:
		it66121_set_DSDAudio(client);
		break;
	case T_AUDIO_NLPCM:
		ucIEC60958ChStat[0] |= 1 << 1;
		it66121_set_ChStat(priv, ucIEC60958ChStat);
		it66121_set_NLPCMAudio(priv);
		break;
	case T_AUDIO_LPCM:
		ucIEC60958ChStat[0] &= ~(1 << 1);

		it66121_set_ChStat(priv, ucIEC60958ChStat);
		it66121_set_LPCMAudio(priv, (param->channels + 1) / 2, SUPPORT_AUDI_AudSWL);
		// can add auto adjust
		break;
	}
	udata = reg_read(client, REG_TX_INT_MASK1);
	udata &= ~B_TX_AUDIO_OVFLW_MASK;
	reg_write(client, REG_TX_INT_MASK1, udata);
	reg_write(client, REG_TX_AUDIO_CTRL0, priv->AudioChannelEnable);

	write_a_entry(priv, REG_TX_SW_RST, (B_HDMITX_AUD_RST | B_TX_AREF_RST), 0);
	return 0;
}

static int it66121_audio_hw_params(struct device *dev,
								   struct hdmi_codec_daifmt *daifmt,
								   struct hdmi_codec_params *params) {
	struct it66121_priv *priv = dev_get_drvdata(dev);
	struct i2c_client *client = priv->hdmi;


	dev_err(&client->dev, "%s: %u Hz, %d bit, %d channels\n", __func__,
			params->sample_rate, params->sample_width, params->channels);

	it66121_aud_config_aai(priv);
	it66121_aud_output_config(priv, params);
	it66121_dump_reg(priv);
	return 0;
}
static void it66121_audio_shutdown(struct device *dev) {
}
static int it66121_audio_digital_mute(struct device *dev, bool enable) {
	struct it66121_priv *priv = dev_get_drvdata(dev);
	if (0 == enable) {
		it66121_power_on(priv);
	}
	return 0;
}
static int it66121_audio_get_eld(struct device *dev,
								 uint8_t *buf, size_t len) {
	struct it66121_priv *priv = dev_get_drvdata(dev);
	struct drm_mode_config *config = &priv->encoder.dev->mode_config;
	struct drm_connector *connector;
	int ret = -ENODEV;

	mutex_lock(&config->mutex);
	list_for_each_entry(connector, &config->connector_list, head) {
		if (&priv->encoder == connector->encoder) {
			memcpy(buf, connector->eld,
				   min(sizeof(connector->eld), len));
			ret = 0;
		}
	}
	mutex_unlock(&config->mutex);

	return ret;
}

static const struct hdmi_codec_ops audio_codec_ops = {
	.hw_params = it66121_audio_hw_params,
	.audio_shutdown = it66121_audio_shutdown,
	.digital_mute = it66121_audio_digital_mute,
	.get_eld = it66121_audio_get_eld,
};

static int it66121_audio_codec_init(struct it66121_priv *priv,
									struct device *dev) {
	struct hdmi_codec_pdata codec_data = {
		.ops = &audio_codec_ops,
		.max_i2s_channels = 2,
		.i2s = 1,
	};
	priv->audio_pdev = platform_device_register_data(
		dev, HDMI_CODEC_DRV_NAME, PLATFORM_DEVID_AUTO,
		&codec_data, sizeof(codec_data));

	return PTR_ERR_OR_ZERO(priv->audio_pdev);
}
static int it66121_init(struct i2c_client *client, struct it66121_priv *priv) {
	int ret = 0;
	priv->hdmi = client;
	int vender_id1, vender_id2, device_id;


	mutex_init(&priv->mutex);   /* protect the range items */
	vender_id1 = reg_read(priv->hdmi, REG_TX_VENDOR_ID0);
	vender_id2 = reg_read(priv->hdmi, REG_TX_VENDOR_ID1);
	device_id = reg_read(priv->hdmi, REG_TX_DEVICE_ID0);
	if (!(vender_id1 == 0x54 &&
		  vender_id2 == 0x49 &&
		  device_id == 0x12)) {
		dev_err(&client->dev, "[it66121] Device not found!\n");
		goto err_device;
	}
	if (it66121_load_reg_table(priv, it66121_init_table) < 0) {
		goto err_device;
	}
	if (it66121_load_reg_table(priv, it66121_default_video_table) < 0) {
		goto err_device;
	}
	if (it66121_load_reg_table(priv, it66121_setHDMI_table) < 0) {
		goto err_device;
	}
	if (it66121_load_reg_table(priv, it66121_default_AVI_info_table) < 0) {
		goto err_device;
	}
	if (it66121_load_reg_table(priv, it66121_default_audio_info_table) < 0) {
		goto err_device;
	}
	if (it66121_load_reg_table(priv, it66121_aud_CHStatus_LPCM_20bit_48Khz) < 0) {
		goto err_device;
	}
	if (it66121_load_reg_table(priv, it66121_AUD_SPDIF_2ch_24bit) < 0) {
		goto err_device;
	}
	if (it66121_power_on(priv) < 0) {
		goto err_device;
	}

	if (it66121_audio_codec_init(priv, &client->dev) != 0) {
		dev_err(&priv->hdmi->dev, "fail to init hdmi audio\n");
		goto err_device;
	}

	if (request_threaded_irq(priv->irq, NULL, it66121_thread_interrupt, IRQF_TRIGGER_LOW | IRQF_ONESHOT, dev_name(&client->dev), priv) < 0) {
		dev_err(&priv->hdmi->dev, "fail to request hdmi irq\n");
		goto err_device;
	}
	return ret;

err_device:
	it66121_free(priv);
	return -ENXIO;;
}


static int it66121_bind(struct device *dev, struct device *master, void *data) {
	struct it66121_priv *priv;
	struct drm_device *drm = data;
	struct i2c_client *client =  to_i2c_client(dev);
	struct device_node *np = dev->of_node;
	int irq;
	int ret;
	u32 crtcs = 0;


	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) return -ENOMEM;

	priv->irq = irq_of_parse_and_map(np, 0);
	dev_info(&client->dev, "Get irq: %d \n", irq);


	ret = it66121_init(client, priv);
	if (ret < 0) {
		dev_err(&client->dev, "it66121_init error \n");
		return ret;
	}
	dev_set_drvdata(dev, priv);
	if (dev->of_node) crtcs = drm_of_find_possible_crtcs(drm, dev->of_node);

	/* If no CRTCs were found, fall back to our old behaviour */
	if (crtcs == 0) {
		dev_err(&client->dev, "Falling back to first CRTC\n");
		crtcs = 1 << 0;
	}

	priv->connector.interlace_allowed = 1;
	priv->connector.polled = DRM_CONNECTOR_POLL_HPD;
	priv->encoder.possible_crtcs = crtcs;


	drm_encoder_helper_add(&priv->encoder, &it66121_encoder_helper_funcs);
	ret = drm_encoder_init(drm, &priv->encoder, &it66121_encoder_funcs,
						   DRM_MODE_ENCODER_TMDS);

	drm_connector_helper_add(&priv->connector, &it66121_connector_helper_funcs);
	ret = drm_connector_init(drm, &priv->connector,
							 &it66121_connector_funcs,
							 DRM_MODE_CONNECTOR_HDMIA);
	if (ret) goto err_connector;
	ret = drm_connector_register(&priv->connector);
	if (ret) goto err_sysfs;


	drm_mode_connector_attach_encoder(&priv->connector, &priv->encoder);

	return 0;

err_sysfs:
	drm_connector_cleanup(&priv->connector);
err_connector:
	drm_encoder_cleanup(&priv->encoder);
	return ret;
}
static void it66121_unbind(struct device *dev, struct device *master,
						   void *data) {
	struct it66121_priv *priv = dev_get_drvdata(dev);

	drm_connector_unregister(&priv->connector);
	drm_connector_cleanup(&priv->connector);
	drm_encoder_cleanup(&priv->encoder);
	it66121_free(priv);
}
static const struct component_ops it66121_ops = {
	.bind = it66121_bind,
	.unbind = it66121_unbind,
};


static int it66121_probe(struct i2c_client *client, const struct i2c_device_id *id) {
	return  component_add(&client->dev, &it66121_ops);;
}

static int it66121_remove(struct i2c_client *client) {
	component_del(&client->dev, &it66121_ops);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id it66121_dt_ids[] = {
	{ .compatible = "ite,it66121", },
	{ }
};
MODULE_DEVICE_TABLE(of, it66121_dt_ids);
#endif

static struct i2c_device_id it66121_ids[] = {
	{ "it66121", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, it66121_ids);

static struct i2c_driver it66121_driver = {
	.probe = it66121_probe,
	.remove = it66121_remove,
	.driver = {
		.name = "it66121",
		.of_match_table = of_match_ptr(it66121_dt_ids),
	},
	.id_table = it66121_ids,
};

module_i2c_driver(it66121_driver);
MODULE_AUTHOR("Baozhu Zuo <zuobaozhu@gmail.com>");
MODULE_DESCRIPTION("BeagleBone Gree HDMI cape Driver");
MODULE_LICENSE("GPL v2");
