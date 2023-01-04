// SPDX-License-Identifier: GPL-2.0+
/*
 * ILI9881D panel driver
 *
 * Copyright (c) 2020 Seeed Studio
 */
#include "panel-ili9881d.h"
#include <linux/version.h>

#define ILI9881_PAGE(_page)	DSI_DCS_WRITE(dsi, 0xff, 0x98, 0x81, _page)
#define IILI9881_COMMAND(_cmd, _data...)		DSI_DCS_WRITE(dsi, _cmd, _data)
#define DCS_CMD_READ_ID1        0xDA

#define ILI_9881D_I2C_ADAPTER	1
#define ILI_9881D_I2C_ADDR	0x45


#define GOODIX_STATUS_SIZE		2
#define GOODIX_CONTACT_SIZE		8
#define GOODIX_BUFFER_STATUS_READY	(((uint32_t)0x01) << 7)//BIT(7)
#define GOODIX_HAVE_KEY		(((uint32_t)0x01) << 4)//BIT(4)

#define TP_DEFAULT_WIDTH	1280
#define TP_DEFAULT_HEIGHT	720
#define TP_MAX_POINTS	5
#define TP_POLL_INTERVAL	15

static struct i2c_mipi_dsi *ili9881d_mipi_dsi;

static int goodix_ts_read_input_report(struct i2c_mipi_dsi *md, u8 *data)
{
	int header = GOODIX_STATUS_SIZE + GOODIX_CONTACT_SIZE;
	int i, ret, touch_num;

	for (i = 0; i < 2; i++) {
		ret = i2c_md_read(md, REG_TP_STATUS, data, header);
		if (ret < 0)
			return -EIO;

		if (data[0] & GOODIX_BUFFER_STATUS_READY) {
			touch_num = data[0] & 0x0f;
			if (touch_num > TP_MAX_POINTS)
				return -EPROTO;

			if (touch_num > 1) {
				ret = i2c_md_read(md, REG_TP_POINT, data+header, (touch_num-1)*GOODIX_CONTACT_SIZE);
				if (ret < 0)
					return -EIO;
			}
			return touch_num;
		}

		usleep_range(3000, 5000); /* Poll every 3 - 5 ms */
	}

	/*
	 * The Goodix panel will send spurious interrupts after a
	 * 'finger up' event, which will always cause a timeout.
	 */
	return -ENOMSG;
}

//TODO
//need more work for it's compatibility
static void x_y_rotate(int *x, int *y)
{
	int temp_x, temp_y;
	int temp;

	if (*x < 0 || *y < 0) {
		pr_err("%s<%d> parameter error\n", __func__, __LINE__);
		return;
	}
	//1 move rectangle center to (0,0)
	temp_x = *x - TP_DEFAULT_WIDTH/2;
	temp_y = *y - TP_DEFAULT_HEIGHT/2;

	//2 rotate the point anti-clockwise for 90 degree
	temp = temp_x;
	temp_x = temp_y;
	temp_y = temp;

	temp_x *= (-1);
	temp_y *= 1;

	//3 zoom
	temp_x = temp_x * TP_DEFAULT_WIDTH / TP_DEFAULT_HEIGHT;
	temp_y = temp_y * TP_DEFAULT_HEIGHT / TP_DEFAULT_WIDTH;

	//4 move rectangle center back to (TP_DEFAULT_WIDTH/2, TP_DEFAULT_HEIGHT/2)
	temp_x += TP_DEFAULT_WIDTH/2;
	temp_y += TP_DEFAULT_HEIGHT/2;

	*x = temp_x;
	*y = temp_y;
}

static void goodix_ts_report_touch_8b(struct i2c_mipi_dsi *md, u8 *coor_data)
{
	struct input_dev *input_dev = md->input;
	int id = coor_data[7];
	int input_x = 0;
	int input_y = 0;
	int input_w = coor_data[4];

	input_x = coor_data[1];
	input_x <<= 8;
	input_x += coor_data[0];

	input_y = coor_data[3];
	input_y <<= 8;
	input_y += coor_data[2];

	if (md->tp_point_rotate)
		x_y_rotate(&input_x, &input_y);

	input_mt_slot(input_dev, id);
	input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, true);
	touchscreen_report_pos(input_dev, &md->prop, input_x, input_y, true);
	input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, input_w);
	input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, input_w);
}

static void tp_poll_func(struct input_dev *input)
{
	struct i2c_mipi_dsi *md = (struct i2c_mipi_dsi *)input_get_drvdata(input);
	u8  point_data[GOODIX_STATUS_SIZE + TP_MAX_POINTS * GOODIX_CONTACT_SIZE] = { 0 };
	int touch_num;
	int i;

	touch_num = goodix_ts_read_input_report(md, point_data);
	if (touch_num < 0)
		return;

	for (i = 0; i < touch_num; i++)
		goodix_ts_report_touch_8b(md, &point_data[GOODIX_STATUS_SIZE + i*GOODIX_CONTACT_SIZE]);

	input_mt_sync_frame(input);
	input_sync(input);
}

int tp_init(struct i2c_mipi_dsi *md)
{
	struct i2c_client *i2c = md->i2c;
	struct device *dev = &i2c->dev;
	struct input_dev *input;
	int ret;

	input = devm_input_allocate_device(dev);
	if (!input) {
		dev_err(dev, "Failed to allocate input device\n");
		return -ENOMEM;
	}
	md->input = input;
	input_set_drvdata(input, md);

	input->dev.parent = dev;
	input->name = "seeed-tp";
	input->id.bustype = BUS_I2C;
	input->id.vendor = 0x1234;
	input->id.product = 0x1001;
	input->id.version = 0x0100;

	input_set_abs_params(input, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, TP_DEFAULT_WIDTH, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, TP_DEFAULT_HEIGHT, 0, 0);

	ret = input_mt_init_slots(input, TP_MAX_POINTS, INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (ret) {
		dev_err(dev, "could not init mt slots, %d\n", ret);
		return ret;
	}

	ret = input_setup_polling(input, tp_poll_func);
	if (ret) {
		dev_err(dev, "could not set up polling mode, %d\n", ret);
		return ret;
	}
	input_set_poll_interval(input, TP_POLL_INTERVAL);

	ret = input_register_device(input);
	if (ret) {
		dev_err(dev, "could not register input device, %d\n", ret);
		return ret;
	}

	return 0;
}

int tp_deinit(struct i2c_mipi_dsi *md)
{
	input_unregister_device(md->input);
	return 0;
}

static const struct drm_display_mode ili9881d_modes = {
	.clock		= 76000,

	.hdisplay	= 800,
	.hsync_start	= 800 + 60,
	.hsync_end	= 800 + 60 + 40,
	.htotal		= 800 + 60 + 40 + 60,

	.vdisplay	= 1280,
	.vsync_start	= 1280 + 16,
	.vsync_end	= 1280 + 16 + 8,
	.vtotal		= 1280 + 16 + 8 + 16,

	.width_mm	= 62,
	.height_mm	= 110,

	.flags          = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC,
};

static int ili9881d_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &ili9881d_modes);
	if (!mode) {
		dev_err(panel->dev, "failed to add mode %ux%u@%u\n",
			mode->hdisplay, mode->vdisplay,
			drm_mode_vrefresh(mode));
		return -ENOMEM;
	}
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_set_name(mode);

	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	drm_mode_probed_add(connector, mode);

	return 1;
}

static int ili9881d_read_id(struct mipi_dsi_device *dsi, u8 *id1)
{
	int ret;

	ret = mipi_dsi_dcs_read(dsi, DCS_CMD_READ_ID1, id1, 1);
	if (ret < 0) {
		dev_err(&dsi->dev, "could not read ID1\n");
		return ret;
	}
	dev_info(&dsi->dev, "ID1 : %02x\n", *id1);

	return 0;
}

static int ili9881d_enable(struct drm_panel *panel)
{
	struct mipi_dsi_device *dsi = ili9881d_mipi_dsi->dsi;
	int ret = 0;
	u8 id1;

	DBG_FUNC();

	if (!dsi)
		return -1;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	ILI9881_PAGE(0x00);
	mipi_dsi_set_maximum_return_packet_size(dsi, 1);

	ret = ili9881d_read_id(dsi, &id1);
	if (ret < 0) {
		dev_info(&dsi->dev, "No LCD connected,pls check your hardware! ret:%d\n", ret);
		return -ENODEV;
	}

	ILI9881_PAGE(0x01);
	IILI9881_COMMAND(0x91,0x00);
	IILI9881_COMMAND(0x92,0x00);
	IILI9881_COMMAND(0x93,0x72);
	IILI9881_COMMAND(0x94,0x00);
	IILI9881_COMMAND(0x95,0x00);
	IILI9881_COMMAND(0x96,0x09);
	IILI9881_COMMAND(0x97,0x00);
	IILI9881_COMMAND(0x98,0x00);

	IILI9881_COMMAND(0x09,0x01);
	IILI9881_COMMAND(0x0a,0x00);
	IILI9881_COMMAND(0x0b,0x00);
	IILI9881_COMMAND(0x0c,0x01);
	IILI9881_COMMAND(0x0d,0x00);
	IILI9881_COMMAND(0x0e,0x00);
	IILI9881_COMMAND(0x0f,0x1D);
	IILI9881_COMMAND(0x10,0x1D);
	IILI9881_COMMAND(0x11,0x00);
	IILI9881_COMMAND(0x12,0x00);
	IILI9881_COMMAND(0x13,0x00);
	IILI9881_COMMAND(0x14,0x00);
	IILI9881_COMMAND(0x15,0x00);
	IILI9881_COMMAND(0x16,0x00);
	IILI9881_COMMAND(0x17,0x00);
	IILI9881_COMMAND(0x18,0x00);
	IILI9881_COMMAND(0x19,0x00);
	IILI9881_COMMAND(0x1a,0x00);
	IILI9881_COMMAND(0x1b,0x00);
	IILI9881_COMMAND(0x1c,0x00);
	IILI9881_COMMAND(0x1d,0x00);
	IILI9881_COMMAND(0x1e,0xc0);
	IILI9881_COMMAND(0x1f,0x00);
	IILI9881_COMMAND(0x20,0x06);
	IILI9881_COMMAND(0x21,0x02);
	IILI9881_COMMAND(0x22,0x00);
	IILI9881_COMMAND(0x23,0x00);
	IILI9881_COMMAND(0x24,0x00);
	IILI9881_COMMAND(0x25,0x00);
	IILI9881_COMMAND(0x26,0x00);
	IILI9881_COMMAND(0x27,0x00);
	IILI9881_COMMAND(0x28,0x33);
	IILI9881_COMMAND(0x29,0x03);
	IILI9881_COMMAND(0x2a,0x00);
	IILI9881_COMMAND(0x2b,0x00);
	IILI9881_COMMAND(0x2c,0x00);
	IILI9881_COMMAND(0x2d,0x00);
	IILI9881_COMMAND(0x2e,0x00);
	IILI9881_COMMAND(0x2f,0x00);
	IILI9881_COMMAND(0x30,0x00);
	IILI9881_COMMAND(0x31,0x00);
	IILI9881_COMMAND(0x32,0x00);
	IILI9881_COMMAND(0x33,0x00);
	IILI9881_COMMAND(0x34,0x04);
	IILI9881_COMMAND(0x35,0x00);
	IILI9881_COMMAND(0x36,0x00);
	IILI9881_COMMAND(0x37,0x00);
	IILI9881_COMMAND(0x38,0x3C);
	IILI9881_COMMAND(0x39,0x07);
	IILI9881_COMMAND(0x3a,0x00);
	IILI9881_COMMAND(0x3b,0x00);
	IILI9881_COMMAND(0x3c,0x00);

	IILI9881_COMMAND(0x40,0x03);
	IILI9881_COMMAND(0x41,0x20);
	IILI9881_COMMAND(0x42,0x00);
	IILI9881_COMMAND(0x43,0x00);
	IILI9881_COMMAND(0x44,0x03);
	IILI9881_COMMAND(0x45,0x00);
	IILI9881_COMMAND(0x46,0x01);
	IILI9881_COMMAND(0x47,0x08);
	IILI9881_COMMAND(0x48,0x00);
	IILI9881_COMMAND(0x49,0x00);
	IILI9881_COMMAND(0x4a,0x00);
	IILI9881_COMMAND(0x4b,0x00);

	// ==== GL[3OUT=
	IILI9881_COMMAND(0x4c,0x01);
	IILI9881_COMMAND(0x4d,0x54);
	IILI9881_COMMAND(0x4e,0x57);
	IILI9881_COMMAND(0x4f,0x9b);
	IILI9881_COMMAND(0x50,0xf9);
	IILI9881_COMMAND(0x51,0x27);
	IILI9881_COMMAND(0x52,0x2f);
	IILI9881_COMMAND(0x53,0xf2);
	IILI9881_COMMAND(0x54,0xff);
	IILI9881_COMMAND(0x55,0xff);
	IILI9881_COMMAND(0x56,0xff);

	// ==== GR[3OUT==
	IILI9881_COMMAND(0x57,0x01);
	IILI9881_COMMAND(0x58,0x54);
	IILI9881_COMMAND(0x59,0x46);
	IILI9881_COMMAND(0x5a,0x8a);
	IILI9881_COMMAND(0x5b,0xf8);
	IILI9881_COMMAND(0x5c,0x26);
	IILI9881_COMMAND(0x5d,0x2f);
	IILI9881_COMMAND(0x5e,0xf2);
	IILI9881_COMMAND(0x5f,0xff);
	IILI9881_COMMAND(0x60,0xff);
	IILI9881_COMMAND(0x61,0xff);

	IILI9881_COMMAND(0x62,0x06);

	// == GOUT:4]_BWUTL[5:0]==
	IILI9881_COMMAND(0x63,0x01);
	IILI9881_COMMAND(0x64,0x00);
	IILI9881_COMMAND(0x65,0xa4);
	IILI9881_COMMAND(0x66,0xa5);
	IILI9881_COMMAND(0x67,0x58);
	IILI9881_COMMAND(0x68,0x5a);
	IILI9881_COMMAND(0x69,0x54);
	IILI9881_COMMAND(0x6a,0x56);
	IILI9881_COMMAND(0x6b,0x06);
	IILI9881_COMMAND(0x6c,0xff);
	IILI9881_COMMAND(0x6d,0x08);
	IILI9881_COMMAND(0x6e,0x02);
	IILI9881_COMMAND(0x6f,0xff);
	IILI9881_COMMAND(0x70,0x02);
	IILI9881_COMMAND(0x71,0x02);
	IILI9881_COMMAND(0x72,0xff);
	IILI9881_COMMAND(0x73,0xff);
	IILI9881_COMMAND(0x74,0xff);
	IILI9881_COMMAND(0x75,0xff);
	IILI9881_COMMAND(0x76,0xff);
	IILI9881_COMMAND(0x77,0xff);
	IILI9881_COMMAND(0x78,0xff);

	// == GOUT:4]_BWUTR[5:0]==
	IILI9881_COMMAND(0x79,0x01);
	IILI9881_COMMAND(0x7a,0x00);
	IILI9881_COMMAND(0x7b,0xa4);
	IILI9881_COMMAND(0x7c,0xa5);
	IILI9881_COMMAND(0x7d,0x59);
	IILI9881_COMMAND(0x7e,0x5b);
	IILI9881_COMMAND(0x7f,0x55);
	IILI9881_COMMAND(0x80,0x57);
	IILI9881_COMMAND(0x81,0x07);
	IILI9881_COMMAND(0x82,0xff);
	IILI9881_COMMAND(0x83,0x09);
	IILI9881_COMMAND(0x84,0x02);
	IILI9881_COMMAND(0x85,0xff);
	IILI9881_COMMAND(0x86,0x02);
	IILI9881_COMMAND(0x87,0x02);
	IILI9881_COMMAND(0x88,0xff);
	IILI9881_COMMAND(0x89,0xff);
	IILI9881_COMMAND(0x8a,0xff);
	IILI9881_COMMAND(0x8b,0xff);
	IILI9881_COMMAND(0x8c,0xff);
	IILI9881_COMMAND(0x8d,0xff);
	IILI9881_COMMAND(0x8e,0xff);

	IILI9881_COMMAND(0x8f,0x00);
	IILI9881_COMMAND(0x90,0x00);

	IILI9881_COMMAND(0x9d,0x00);
	IILI9881_COMMAND(0x9e,0x00);

	IILI9881_COMMAND(0xa0,0x35);
	IILI9881_COMMAND(0xa1,0x00);
	IILI9881_COMMAND(0xa2,0x00);
	IILI9881_COMMAND(0xa3,0x00);
	IILI9881_COMMAND(0xa4,0x00);
	IILI9881_COMMAND(0xa5,0x00);
	IILI9881_COMMAND(0xa6,0x08);
	IILI9881_COMMAND(0xa7,0x00);
	IILI9881_COMMAND(0xa8,0x00);
	IILI9881_COMMAND(0xa9,0x00);
	IILI9881_COMMAND(0xaa,0x00);
	IILI9881_COMMAND(0xab,0x00);
	IILI9881_COMMAND(0xac,0x00);
	IILI9881_COMMAND(0xad,0x00);
	IILI9881_COMMAND(0xae,0xff);
	IILI9881_COMMAND(0xaf,0x00);
	IILI9881_COMMAND(0xb0,0x00);

	ILI9881_PAGE(0x02);
	IILI9881_COMMAND(0x08,0x11);
	IILI9881_COMMAND(0x0a,0x0c);
	IILI9881_COMMAND(0x0f,0x06);
	IILI9881_COMMAND(0xA0,0x00,0x26,0x35,0x16,0x19,0x2C,0x1F,0x1F,0x96,0x1C,0x28,0x80,0x1A,0x18,0x4C,0x21,0x27,0x55,0x65,0x39);
	IILI9881_COMMAND(0xC0,0x00,0x26,0x35,0x16,0x19,0x2C,0x1F,0x1F,0x96,0x1C,0x28,0x80,0x1A,0x18,0x4C,0x21,0x27,0x55,0x65,0x39);

	//===== GIP code finish =====//
	IILI9881_COMMAND(0x4C,0xA4); // PS_EN on ,0x default :A4
	IILI9881_COMMAND(0x18,0xF4); // SH on ,0x default E4 

	//=========================//
	ILI9881_PAGE(0x04);
	IILI9881_COMMAND(0x5D,0xAF); // VREG1 5.5V 
	IILI9881_COMMAND(0x5E,0xAF); // VREG2 5.5V
	IILI9881_COMMAND(0x60,0x9B); // VCM1 
	IILI9881_COMMAND(0x62,0x9B); // VCM2 
	IILI9881_COMMAND(0x82,0x38); // VREF_VGH_MOD_CLPSEL 16V 
	IILI9881_COMMAND(0x84,0x38); // VREF_VGH_DC 16V     
	IILI9881_COMMAND(0x86,0x18); // VREF_VGL_CLPSEL -10V       
	IILI9881_COMMAND(0x66,0xC4); // VGH_AC x4 ,0xdefault :04
	IILI9881_COMMAND(0xC1,0xF0); // VGH_DC x4 ,0xdefault :70
	IILI9881_COMMAND(0x70,0x60);
	IILI9881_COMMAND(0x71,0x00);

	//=========================//
	IILI9881_COMMAND(0x5B,0x33); // vcore_sel Voltage
	IILI9881_COMMAND(0x6C,0x10); // vcore bias L
	IILI9881_COMMAND(0x77,0x03); // vcore_sel Voltage
	IILI9881_COMMAND(0x7B,0x02); // vcore bias R

	//=========================//
	ILI9881_PAGE(0x01);
	IILI9881_COMMAND(0xF0,0x00); // 1280 Gate NL
	IILI9881_COMMAND(0xF1,0xC8); // 1280 Gate NL

	ILI9881_PAGE(0x05);
	IILI9881_COMMAND(0x22,0x3A); // RGB to BGR

	ILI9881_PAGE(0x00);
	IILI9881_COMMAND(0x35,0x00);

	IILI9881_COMMAND(0x11);
	msleep(120);
	IILI9881_COMMAND(0x29);

	return 0;
}

static const struct drm_panel_funcs ili9881d_funcs = {
	.get_modes = ili9881d_get_modes,
	.enable = ili9881d_enable,
};

static void ili9881d_set_dsi(struct mipi_dsi_device *dsi)
{
	dsi->mode_flags = (MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_LPM);
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->lanes = 4;
}

const struct panel_data ili9881d_data = {
	.set_dsi = ili9881d_set_dsi,
	.funcs = &ili9881d_funcs,
};

static int i2c_md_read(struct i2c_mipi_dsi *md, u8 reg, u8 *buf, int len)
{
	struct i2c_client *client = md->i2c;
	struct i2c_msg msgs[1];
	u8 addr_buf[1] = { reg };
	u8 data_buf[1] = { 0, };
	int ret;

	mutex_lock(&md->mutex);

	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = ARRAY_SIZE(addr_buf);
	msgs[0].buf = addr_buf;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs)) {
		mutex_unlock(&md->mutex);
		return -EIO;
	}

	usleep_range(1000, 1500);

	/* Read data from register */
	msgs[0].addr = client->addr;
	msgs[0].flags = I2C_M_RD;
	if (buf == NULL) {
		msgs[0].len = 1;
		msgs[0].buf = data_buf;
	} else {
		msgs[0].len = len;
		msgs[0].buf = buf;
	}

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs)) {
		mutex_unlock(&md->mutex);
		return -EIO;
	}
	mutex_unlock(&md->mutex);

	if (buf == NULL)
		return data_buf[0];
	else
		return ret;
}

static void i2c_md_write(struct i2c_mipi_dsi *md, u8 reg, u8 val)
{
	struct i2c_client *client = md->i2c;
	int ret;

	mutex_lock(&md->mutex);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret)
		dev_err(&client->dev, "I2C write failed: %d\n", ret);

	usleep_range(1000, 1500);
	mutex_unlock(&md->mutex);
}

/* panel_funcs */
static int panel_prepare(struct drm_panel *panel)
{
	int ret = 0;
	struct i2c_mipi_dsi *md = panel_to_md(panel);
	const struct drm_panel_funcs *funcs = md->panel_data->funcs;

	DBG_FUNC("");

	/* i2c */
	/* reset pin */
	i2c_md_write(md, REG_LCD_RST, 0);
	msleep(20);
	i2c_md_write(md, REG_LCD_RST, 1);
	msleep(20);

	/* panel */
	if (funcs && funcs->prepare) {
		ret = funcs->prepare(panel);
		if (ret < 0) {
			i2c_md_write(md, REG_POWERON, 0);
			i2c_md_write(md, REG_LCD_RST, 0);
			i2c_md_write(md, REG_PWM, 0);
			return ret;
		}
	}

	return ret;
}

static int panel_unprepare(struct drm_panel *panel)
{
	int ret = 0;
	struct i2c_mipi_dsi *md = panel_to_md(panel);
	const struct drm_panel_funcs *funcs = md->panel_data->funcs;

	DBG_FUNC("");
	if (funcs && funcs->unprepare) {
		ret = funcs->unprepare(panel);
		if (ret < 0)
			return ret;
	}

	return ret;
}

static int panel_enable(struct drm_panel *panel)
{
	int ret = 0;
	struct i2c_mipi_dsi *md = panel_to_md(panel);
	const struct drm_panel_funcs *funcs = md->panel_data->funcs;

	DBG_FUNC("");
	/* panel */
	if (funcs && funcs->enable) {
		ret = funcs->enable(panel);
		if (ret < 0)
			return ret;
	}

	/* i2c */
	i2c_md_write(md, REG_PWM, md->brightness);

	return ret;
}

static int panel_disable(struct drm_panel *panel)
{
	int ret = 0;
	struct i2c_mipi_dsi *md = panel_to_md(panel);
	const struct drm_panel_funcs *funcs = md->panel_data->funcs;

	DBG_FUNC("");
	/* i2c */
	i2c_md_write(md, REG_PWM, 0);
	i2c_md_write(md, REG_LCD_RST, 0);

	/* panel */
	if (funcs && funcs->disable) {
		ret = funcs->disable(panel);
		if (ret < 0)
			return ret;
	}

	return ret;
}

static int panel_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
	int ret = 0;
	struct i2c_mipi_dsi *md = panel_to_md(panel);
	const struct drm_panel_funcs *funcs = md->panel_data->funcs;

	if (funcs && funcs->get_modes) {
		ret = funcs->get_modes(panel, connector);
		if (ret < 0)
			return ret;
	}

	return ret;
}

static const struct drm_panel_funcs panel_funcs = {
	.prepare = panel_prepare,
	.unprepare = panel_unprepare,
	.enable = panel_enable,
	.disable = panel_disable,
	.get_modes = panel_get_modes,
};

/* backlight */
static int backlight_update(struct backlight_device *bd)
{
	struct i2c_mipi_dsi *md = bl_get_data(bd);
	int brightness = bd->props.brightness;

	if (bd->props.power != FB_BLANK_UNBLANK ||
		bd->props.fb_blank != FB_BLANK_UNBLANK ||
		(bd->props.state & (BL_CORE_SUSPENDED | BL_CORE_FBBLANK))) {
			brightness = 0;
		}

	md->brightness = brightness;
	i2c_md_write(md, REG_PWM, brightness);

	return 0;
}

static const struct backlight_ops backlight_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status	= backlight_update,
};

static int backlight_init(struct i2c_mipi_dsi *md)
{
	struct device *dev = &md->i2c->dev;
	struct backlight_properties props;
	struct backlight_device *bd;

	memset(&props, 0, sizeof(props));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = 255;
	bd = devm_backlight_device_register(dev, dev_name(dev),
					dev, md, &backlight_ops,
					&props);
	if (IS_ERR(bd)) {
		dev_err(dev, "failed to register backlight\n");
		return PTR_ERR(bd);
	}

	bd->props.brightness = 255;
	backlight_update_status(bd);

	return 0;
}

static int i2c_md_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct device *dev = &i2c->dev;
	struct i2c_mipi_dsi *md = ili9881d_mipi_dsi;
	int ret = 0;

	DBG_FUNC("start");

	i2c_set_clientdata(i2c, md);
	mutex_init(&md->mutex);
	md->i2c = i2c;

	md->panel_data = &ili9881d_data;
	if (!md->panel_data) {
		dev_err(dev, "No valid panel data.\n");
		return -ENODEV;
	}

	ret = i2c_md_read(md, REG_ID, NULL, 0);
	if (ret != 0xC3) {
		dev_err(dev, "Unknown chip id: 0x%02x\n", ret);
		return -ENODEV;
	}
	dev_info(dev, "I2C Address:0x%x read id: 0x%x\n", i2c->addr, ret);

	/* Turn on */
	i2c_md_write(md, REG_POWERON, 1);

	DBG_FUNC("finished.");

	return 0;
}

static int i2c_md_remove(struct i2c_client *i2c)
{
	struct i2c_mipi_dsi *md = i2c_get_clientdata(i2c);

	DBG_FUNC();
	tp_deinit(md);

	/* Turn off power */
	i2c_md_write(md, REG_POWERON, 0);
	i2c_md_write(md, REG_LCD_RST, 0);
	i2c_md_write(md, REG_PWM, 0);

	mipi_dsi_detach(md->dsi);
	drm_panel_remove(&md->panel);

	return 0;
}

static void i2c_md_shutdown(struct i2c_client *i2c)
{
	struct i2c_mipi_dsi *md = i2c_get_clientdata(i2c);

	DBG_FUNC();
	tp_deinit(md);

	/* Turn off power */
	i2c_md_write(md, REG_POWERON, 0);
	i2c_md_write(md, REG_LCD_RST, 0);
	i2c_md_write(md, REG_PWM, 0);

	mipi_dsi_detach(md->dsi);
	drm_panel_remove(&md->panel);
}

static const struct of_device_id i2c_md_of_ids[] = {
	{
		.compatible = "ili9881d",
	},
	{ }
};
MODULE_DEVICE_TABLE(of, i2c_md_of_ids);

static struct i2c_driver i2c_md_driver = {
	.driver = {
		.name = "i2c_mipi_dsi",
		.of_match_table = i2c_md_of_ids,
	},
	.probe = i2c_md_probe,
	.remove = i2c_md_remove,
	.shutdown = i2c_md_shutdown,
};

static int ili9881d_hack_create_device(void)
{
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	struct i2c_board_info info = {
		.type = "ili9881d",
		.addr = ILI_9881D_I2C_ADDR,
	};

	adapter = i2c_get_adapter(ILI_9881D_I2C_ADAPTER);
	if (!adapter) {
		pr_err("%s: i2c_get_adapter(%d) failed\n", __func__,
			ILI_9881D_I2C_ADAPTER);
		return -EINVAL;
	}

	client = i2c_new_client_device(adapter, &info);
	if (IS_ERR(client)) {
		pr_err("%s: creating I2C device failed\n", __func__);
		i2c_put_adapter(adapter);
		return PTR_ERR(client);
	}

	return 0;
}

static int ili9881d_dsi_probe(struct mipi_dsi_device *dsi)
{
	int ret;
	struct i2c_mipi_dsi *ctx;

	ctx = devm_kzalloc(&dsi->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ili9881d_mipi_dsi = ctx;

	ili9881d_hack_create_device();
	ret = i2c_add_driver(&i2c_md_driver);
	if (ret < 0) {
		dev_err(&dsi->dev, "i2c_add_driver ret:%d\n", ret);
		return ret;
	}

	mipi_dsi_set_drvdata(dsi, ctx);
	ctx->dsi = dsi;

	ctx->panel_data->set_dsi(ctx->dsi);
	drm_panel_init(&ctx->panel, &dsi->dev, &panel_funcs, DRM_MODE_CONNECTOR_DSI);
	drm_panel_add(&ctx->panel);

	tp_init(ctx);
	backlight_init(ctx);

	ret = device_property_read_u32(&dsi->dev, "mcu_auto_reset_enable", &ctx->mcu_auto_reset);
	if (ret < 0)
		dev_err(&dsi->dev, "Can't get the data of mcu_auto_reset!\n");
	i2c_md_write(ctx, REG_MCU_AUTO_RESET, (ctx->mcu_auto_reset&0xff));

	ret = device_property_read_u32(&dsi->dev, "tp_point_rotate", &ctx->tp_point_rotate);
	if (ret < 0)
		dev_err(&dsi->dev, "Can't get the data of tp_point_rotate!\n");

	return mipi_dsi_attach(dsi);
}

static int ili9881d_dsi_remove(struct mipi_dsi_device *dsi)
{
	struct i2c_mipi_dsi *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);

	return 0;
}

static const struct of_device_id ili9881d_of_match[] = {
	{ .compatible = "i2c_dsi,ili9881d", },
	{ }
};
MODULE_DEVICE_TABLE(of, ili9881d_of_match);

static struct mipi_dsi_driver ili9881d_dsi_driver = {
	.probe		= ili9881d_dsi_probe,
	.remove		= ili9881d_dsi_remove,
	.driver = {
		.name		= "ili9881d-dsi",
		.of_match_table	= ili9881d_of_match,
	},
};
module_mipi_dsi_driver(ili9881d_dsi_driver);

MODULE_DESCRIPTION("Ilitek ILI9881D Controller Driver");
MODULE_LICENSE("GPL v2");
