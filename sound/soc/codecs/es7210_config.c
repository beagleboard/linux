/*
 * ALSA SoC ES7210 adc driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.^M
 *
 * Notes:
 *  ES7210 is a 4-ch ADC of Everest
 */
static int es7210_read(u8 reg, u8 * rt_value, struct i2c_client *client)
{
	int ret;
	u8 read_cmd[3] = { 0 };
	u8 cmd_len = 0;

	read_cmd[0] = reg;
	cmd_len = 1;

	if (client->adapter == NULL)
		printk("es7210_read client->adapter==NULL\n");

	ret = i2c_master_send(client, read_cmd, cmd_len);
	if (ret != cmd_len) {
		printk("es7210_read error1\n");
		return -1;
	}

	ret = i2c_master_recv(client, rt_value, 1);
	if (ret != 1) {
		printk("es7210_read error2, ret = %d.\n", ret);
		return -1;
	}

	return 0;
}

static int es7210_write(u8 reg, unsigned char value, struct i2c_client *client)
{
	int ret = 0;
	u8 write_cmd[2] = { 0 };

	write_cmd[0] = reg;
	write_cmd[1] = value;

	ret = i2c_master_send(client, write_cmd, 2);
	if (ret != 2) {
		printk("es7210_write error->[REG-0x%02x,val-0x%02x]\n",
		       reg, value);
		return -1;
	}

	return 0;
}

static int es7210_update_bits(u8 reg, u8 mask, u8 value,
			      struct i2c_client *client)
{
	u8 val_old, val_new;

	es7210_read(reg, &val_old, client);
	val_new = (val_old & ~mask) | (value & mask);
	if (val_new != val_old) {
		es7210_write(reg, val_new, client);
	}

	return 0;
}

static int es7210_multi_chips_write(u8 reg, unsigned char value)
{
	u8 i;

	for (i = 0; i < ADC_DEV_MAXNUM; i++) {
		es7210_write(reg, value, i2c_clt1[i]);
	}

	return 0;
}

static int es7210_multi_chips_update_bits(u8 reg, u8 mask, u8 value)
{
	u8 i;

	for (i = 0; i < ADC_DEV_MAXNUM; i++) {
		es7210_update_bits(reg, mask, value, i2c_clt1[i]);
	}

	return 0;
}

/*
* to initialize es7210 for tdm mode
*/
static void es7210_tdm_init_codec(u8 mode)
{
	int cnt, channel;

	for (cnt = 0;
	     cnt < sizeof(es7210_tdm_reg_common_cfg1) /
	     sizeof(es7210_tdm_reg_common_cfg1[0]); cnt++) {
		es7210_multi_chips_write(es7210_tdm_reg_common_cfg1
					 [cnt].reg_addr,
					 es7210_tdm_reg_common_cfg1
					 [cnt].reg_v);
	}
	switch (mode) {
	case ES7210_TDM_1LRCK_DSPA:
		for (cnt = 0; cnt < ADC_DEV_MAXNUM; cnt++) {
			es7210_write(ES7210_SDP_CFG1_REG11,
				     0x63, i2c_clt1[cnt]);
		}
		for (cnt = 0; cnt < ADC_DEV_MAXNUM; cnt++) {
			es7210_write(ES7210_SDP_CFG2_REG12,
				     0x01, i2c_clt1[cnt]);
		}
		break;
	case ES7210_TDM_1LRCK_DSPB:
		for (cnt = 0; cnt < ADC_DEV_MAXNUM; cnt++) {
			es7210_write(ES7210_SDP_CFG1_REG11,
				     0x73, i2c_clt1[cnt]);
		}
		for (cnt = 0; cnt < ADC_DEV_MAXNUM; cnt++) {
			es7210_write(ES7210_SDP_CFG2_REG12,
				     0x01, i2c_clt1[cnt]);
		}
		break;
	case ES7210_TDM_1LRCK_I2S:
		for (cnt = 0; cnt < ADC_DEV_MAXNUM; cnt++) {
			es7210_write(ES7210_SDP_CFG1_REG11,
				     0x60, i2c_clt1[cnt]);
		}
		for (cnt = 0; cnt < ADC_DEV_MAXNUM; cnt++) {
			es7210_write(ES7210_SDP_CFG2_REG12,
				     0x02, i2c_clt1[cnt]);
		}
		break;
	case ES7210_TDM_1LRCK_LJ:
		for (cnt = 0; cnt < ADC_DEV_MAXNUM; cnt++) {
			es7210_write(ES7210_SDP_CFG1_REG11,
				     0x61, i2c_clt1[cnt]);
		}
		for (cnt = 0; cnt < ADC_DEV_MAXNUM; cnt++) {
			es7210_write(ES7210_SDP_CFG2_REG12,
				     0x02, i2c_clt1[cnt]);
		}
		break;
	case ES7210_TDM_NLRCK_DSPA:
		for (cnt = 0; cnt < ADC_DEV_MAXNUM; cnt++) {
			es7210_write(ES7210_SDP_CFG1_REG11,
				     0x63, i2c_clt1[cnt]);
		}
		for (cnt = 0; cnt < ADC_DEV_MAXNUM; cnt++) {
			if (cnt == 0) {
				es7210_write(ES7210_SDP_CFG2_REG12,
					     0x07, i2c_clt1[cnt]);
			} else {
				es7210_write(ES7210_SDP_CFG2_REG12,
					     0x03, i2c_clt1[cnt]);
			}
		}
		break;
	case ES7210_TDM_NLRCK_DSPB:
		for (cnt = 0; cnt < ADC_DEV_MAXNUM; cnt++) {
			es7210_write(ES7210_SDP_CFG1_REG11,
				     0x73, i2c_clt1[cnt]);
		}
		for (cnt = 0; cnt < ADC_DEV_MAXNUM; cnt++) {
			if (cnt == 0) {
				es7210_write(ES7210_SDP_CFG2_REG12,
					     0x07, i2c_clt1[cnt]);
			} else {
				es7210_write(ES7210_SDP_CFG2_REG12,
					     0x03, i2c_clt1[cnt]);
			}
		}
		break;
	case ES7210_TDM_NLRCK_I2S:
		for (cnt = 0; cnt < ADC_DEV_MAXNUM; cnt++) {
			es7210_write(ES7210_SDP_CFG1_REG11,
				     0x60, i2c_clt1[cnt]);
		}
		for (cnt = 0; cnt < ADC_DEV_MAXNUM; cnt++) {
			if (cnt == 0) {
				es7210_write(ES7210_SDP_CFG2_REG12,
					     0x07, i2c_clt1[cnt]);
			} else {
				es7210_write(ES7210_SDP_CFG2_REG12,
					     0x03, i2c_clt1[cnt]);
			}
		}
		break;
	case ES7210_TDM_NLRCK_LJ:
		for (cnt = 0; cnt < ADC_DEV_MAXNUM; cnt++) {
			es7210_write(ES7210_SDP_CFG1_REG11,
				     0x61, i2c_clt1[cnt]);
		}
		for (cnt = 0; cnt < ADC_DEV_MAXNUM; cnt++) {
			if (cnt == 0) {
				es7210_write(ES7210_SDP_CFG2_REG12,
					     0x07, i2c_clt1[cnt]);
			} else {
				es7210_write(ES7210_SDP_CFG2_REG12,
					     0x03, i2c_clt1[cnt]);
			}
		}
		break;
	case ES7210_NORMAL_I2S:
		es7210_multi_chips_write(ES7210_SDP_CFG1_REG11, 0x60);
		es7210_multi_chips_write(ES7210_SDP_CFG2_REG12, 0x00);
		break;
	case ES7210_NORMAL_LJ:
		es7210_multi_chips_write(ES7210_SDP_CFG1_REG11, 0x61);
		es7210_multi_chips_write(ES7210_SDP_CFG2_REG12, 0x00);
		break;
	case ES7210_NORMAL_DSPA:
		es7210_multi_chips_write(ES7210_SDP_CFG1_REG11, 0x63);
		es7210_multi_chips_write(ES7210_SDP_CFG2_REG12, 0x00);
		break;
	case ES7210_NORMAL_DSPB:
		es7210_multi_chips_write(ES7210_SDP_CFG1_REG11, 0x73);
		es7210_multi_chips_write(ES7210_SDP_CFG2_REG12, 0x00);
		break;
	default:
		es7210_multi_chips_write(ES7210_SDP_CFG1_REG11, 0x60);
		es7210_multi_chips_write(ES7210_SDP_CFG2_REG12, 0x00);
		break;

	}
	for (cnt = 0;
	     cnt < sizeof(es7210_tdm_reg_common_cfg2) /
	     sizeof(es7210_tdm_reg_common_cfg2[0]); cnt++) {
		es7210_multi_chips_write(es7210_tdm_reg_common_cfg2
					 [cnt].reg_addr,
					 es7210_tdm_reg_common_cfg2
					 [cnt].reg_v);
	}
	switch (mode) {
	case ES7210_TDM_1LRCK_DSPA:
	case ES7210_TDM_1LRCK_DSPB:
	case ES7210_TDM_1LRCK_I2S:
	case ES7210_TDM_1LRCK_LJ:
		es7210_multi_chips_write(ES7210_MCLK_CTL_REG02, 0xc1);
		break;
	case ES7210_TDM_NLRCK_DSPA:
	case ES7210_TDM_NLRCK_DSPB:
	case ES7210_TDM_NLRCK_I2S:
	case ES7210_TDM_NLRCK_LJ:
		channel = ES7210_CHANNELS_MAX / 2;
		channel &= 0x0f;
		channel = channel << 4;
		printk("ADC Analg channel register = 0x%2x\n", channel);
		es7210_multi_chips_write(ES7210_MODE_CFG_REG08, channel);
		es7210_multi_chips_write(ES7210_MCLK_CTL_REG02, 0x01);
		break;
	default:
		es7210_multi_chips_write(ES7210_MCLK_CTL_REG02, 0x41);
		break;
	}
	for (cnt = 0; cnt < sizeof(es7210_tdm_reg_common_cfg3) /
	     sizeof(es7210_tdm_reg_common_cfg3[0]); cnt++) {
		es7210_multi_chips_write(es7210_tdm_reg_common_cfg3
					 [cnt].reg_addr,
					 es7210_tdm_reg_common_cfg3
					 [cnt].reg_v);
	}
	/*
	 * Mute All ADC
	 */
	es7210_multi_chips_update_bits(ES7210_ADC34_MUTE_REG14, 0x03, 0x03);
	es7210_multi_chips_update_bits(ES7210_ADC12_MUTE_REG15, 0x03, 0x03);
	/*
	 * Set Direct DB Gain
	 */
	es7210_multi_chips_update_bits(ES7210_ALC_COM_CFG2_REG1A, 0x03, 0x00);
	es7210_multi_chips_write(ES7210_ALC1_MAX_GAIN_REG1E, 0xBF);
	es7210_multi_chips_write(ES7210_ALC2_MAX_GAIN_REG1D, 0xBF);
	es7210_multi_chips_write(ES7210_ALC3_MAX_GAIN_REG1C, 0xBF);
	es7210_multi_chips_write(ES7210_ALC4_MAX_GAIN_REG1B, 0xBF);
}
