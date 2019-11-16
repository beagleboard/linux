// SPDX-License-Identifier: GPL-2.0
/*
 * DRA7 EVM TPD12S015 HDMI ESD protection & level shifter chip driver
 *
 * Copyright (C) 2013-2018 Texas Instruments Incorporated -  http://www.ti.com/
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
 */

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/suspend.h>
#include <linux/of_platform.h>
#include <linux/pinctrl/consumer.h>
#include <linux/spinlock.h>

#include "../dss/omapdss.h"

#define SEL_I2C2	0
#define SEL_HDMI	1

/* HPD gpio debounce time in microseconds */
#define HPD_DEBOUNCE_TIME	1000

struct i2c_mux_wa {
	struct gpio_desc *i2c_ddc_gpio;

	struct pinctrl *pins;
	struct pinctrl_state *pin_state_i2c;
	struct pinctrl_state *pin_state_ddc;

	struct i2c_adapter *ddc_i2c_adapter;
};

struct panel_drv_data {
	struct omap_dss_device dssdev;
	struct omap_dss_device *src;
	void (*hpd_cb)(void *cb_data, enum drm_connector_status status);
	void *hpd_cb_data;
	struct mutex hpd_lock;

	struct gpio_desc *ct_cp_hpd_gpio;
	struct gpio_desc *ls_oe_gpio;
	struct gpio_desc *hpd_gpio;

	struct i2c_mux_wa i2c_wa;
};

/*
 * use SEL_I2C2 to configure pcf8575@26 to set/unset LS_OE and CT_HPD, and use
 * SEL_HDMI to read edid via the HDMI ddc lines
 */
static void tpd_i2c_ddc_demux(struct i2c_mux_wa *i2c_wa, int sel)
{
	/*
	 * switch to I2C2 or HDMI DDC internal pinmux gpio to low or high to
	 * select I2C2 or HDMI path respectively
	 */
	if (sel == SEL_I2C2) {
		pinctrl_select_state(i2c_wa->pins, i2c_wa->pin_state_i2c);
		gpiod_set_value(i2c_wa->i2c_ddc_gpio, 0);
	} else {
		pinctrl_select_state(i2c_wa->pins, i2c_wa->pin_state_ddc);
		gpiod_set_value(i2c_wa->i2c_ddc_gpio, 1);
	}

	/* let it propagate */
	udelay(5);
}

#define to_panel_data(x) container_of(x, struct panel_drv_data, dssdev)

static int tpd_connect(struct omap_dss_device *src,
		       struct omap_dss_device *dst)
{
	struct panel_drv_data *ddata = to_panel_data(dst);
	int r;

	r = omapdss_device_connect(dst->dss, dst, dst->next);
	if (r)
		return r;

	gpiod_set_value_cansleep(ddata->ct_cp_hpd_gpio, 1);
	gpiod_set_value_cansleep(ddata->ls_oe_gpio, 1);

	/* DC-DC converter needs at max 300us to get to 90% of 5V */
	udelay(300);

	/*
	 * The HPD GPIO debounce causes a delay until we see the real HPD state.
	 * If tpd_read_edid() or tpd_detect() are called very soon after setting
	 * the ct_cp_hpd-gpio, we could observe wrong HPD value. So sleep here
	 * until the GPIO values has become valid.
	 */
	msleep(DIV_ROUND_UP(HPD_DEBOUNCE_TIME, 1000));

	ddata->src = src;

	return 0;
}

static void tpd_disconnect(struct omap_dss_device *src,
			   struct omap_dss_device *dst)
{
	struct panel_drv_data *ddata = to_panel_data(dst);

	gpiod_set_value_cansleep(ddata->ct_cp_hpd_gpio, 0);
	gpiod_set_value_cansleep(ddata->ls_oe_gpio, 0);

	omapdss_device_disconnect(dst, dst->next);
	ddata->src = NULL;
}

static int tpd_read_edid(struct omap_dss_device *dssdev,
		u8 *edid, int len)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *src = ddata->src;
	struct i2c_mux_wa *i2c_wa = &ddata->i2c_wa;
	int r = 0;

	if (!gpiod_get_value_cansleep(ddata->hpd_gpio))
		return -ENODEV;

	i2c_lock_bus(i2c_wa->ddc_i2c_adapter, I2C_LOCK_ROOT_ADAPTER);

	tpd_i2c_ddc_demux(i2c_wa, SEL_HDMI);

	r = src->ops->read_edid(src, edid, len);

	tpd_i2c_ddc_demux(i2c_wa, SEL_I2C2);

	i2c_unlock_bus(i2c_wa->ddc_i2c_adapter, I2C_LOCK_ROOT_ADAPTER);

	return r;
}

static bool tpd_detect(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);

	return gpiod_get_value_cansleep(ddata->hpd_gpio);
}

static void tpd_register_hpd_cb(struct omap_dss_device *dssdev,
				void (*cb)(void *cb_data,
					  enum drm_connector_status status),
				void *cb_data)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);

	mutex_lock(&ddata->hpd_lock);
	ddata->hpd_cb = cb;
	ddata->hpd_cb_data = cb_data;
	mutex_unlock(&ddata->hpd_lock);
}

static void tpd_unregister_hpd_cb(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);

	mutex_lock(&ddata->hpd_lock);
	ddata->hpd_cb = NULL;
	ddata->hpd_cb_data = NULL;
	mutex_unlock(&ddata->hpd_lock);
}

static const struct omap_dss_device_ops tpd_ops = {
	.connect		= tpd_connect,
	.disconnect		= tpd_disconnect,
	.detect			= tpd_detect,
	.register_hpd_cb	= tpd_register_hpd_cb,
	.unregister_hpd_cb	= tpd_unregister_hpd_cb,
	.read_edid		= tpd_read_edid,
};

static irqreturn_t tpd_hpd_isr(int irq, void *data)
{
	struct panel_drv_data *ddata = data;

	mutex_lock(&ddata->hpd_lock);
	if (ddata->hpd_cb) {
		enum drm_connector_status status;

		if (tpd_detect(&ddata->dssdev))
			status = connector_status_connected;
		else
			status = connector_status_disconnected;

		ddata->hpd_cb(ddata->hpd_cb_data, status);
	}
	mutex_unlock(&ddata->hpd_lock);

	return IRQ_HANDLED;
}

static int tpd_init_i2c_mux(struct platform_device *pdev)
{
	struct panel_drv_data *ddata = platform_get_drvdata(pdev);
	struct i2c_mux_wa *i2c_wa = &ddata->i2c_wa;
	struct device_node *node;

	i2c_wa->i2c_ddc_gpio = devm_gpiod_get_index(&pdev->dev, NULL,
						    3, GPIOD_OUT_LOW);
	if (IS_ERR(i2c_wa->i2c_ddc_gpio))
		return PTR_ERR(i2c_wa->i2c_ddc_gpio);

	i2c_wa->pins = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(i2c_wa->pins))
		return PTR_ERR(i2c_wa->pins);

	i2c_wa->pin_state_i2c = pinctrl_lookup_state(i2c_wa->pins, "i2c");
	if (IS_ERR(i2c_wa->pin_state_i2c))
		return PTR_ERR(i2c_wa->pin_state_i2c);

	i2c_wa->pin_state_ddc = pinctrl_lookup_state(i2c_wa->pins, "ddc");
	if (IS_ERR(i2c_wa->pin_state_ddc))
		return PTR_ERR(i2c_wa->pin_state_ddc);

	node = of_parse_phandle(pdev->dev.of_node, "ddc-i2c-bus", 0);
	if (!node)
		return -ENODEV;

	i2c_wa->ddc_i2c_adapter = of_find_i2c_adapter_by_node(node);
	if (!i2c_wa->ddc_i2c_adapter)
		return -ENODEV;

	tpd_i2c_ddc_demux(i2c_wa, SEL_I2C2);

	return 0;
}

static void tpd_uninit_i2c_mux(struct platform_device *pdev)
{
	struct panel_drv_data *ddata = platform_get_drvdata(pdev);

	tpd_i2c_ddc_demux(&ddata->i2c_wa, SEL_I2C2);

	i2c_put_adapter(ddata->i2c_wa.ddc_i2c_adapter);
}

static int tpd_probe(struct platform_device *pdev)
{
	struct omap_dss_device *dssdev;
	struct panel_drv_data *ddata;
	int r;
	struct gpio_desc *gpio;

	ddata = devm_kzalloc(&pdev->dev, sizeof(*ddata), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	platform_set_drvdata(pdev, ddata);

	gpio = devm_gpiod_get_index_optional(&pdev->dev, NULL, 0,
		 GPIOD_OUT_LOW);
	if (IS_ERR(gpio))
		return PTR_ERR(gpio);

	ddata->ct_cp_hpd_gpio = gpio;

	gpio = devm_gpiod_get_index_optional(&pdev->dev, NULL, 1,
		 GPIOD_OUT_LOW);
	if (IS_ERR(gpio))
		return PTR_ERR(gpio);

	ddata->ls_oe_gpio = gpio;

	gpio = devm_gpiod_get_index(&pdev->dev, NULL, 2,
		GPIOD_IN);
	if (IS_ERR(gpio))
		return PTR_ERR(gpio);

	ddata->hpd_gpio = gpio;

	mutex_init(&ddata->hpd_lock);

	r = tpd_init_i2c_mux(pdev);
	if (r)
		return r;

	/*
	 * we see some low voltage glitches on the HPD_B line before it
	 * stabalizes to around 5V. We see the effects of this glitch on the
	 * HPD_A side, and hence on the gpio on DRA7x. The glitch is quite short
	 * in duration, but it takes a while for the voltage to go down back to
	 * 0 volts, we set a debounce value of 1 millisecond to prevent this,
	 * the reason for the glitch not being taken care of by the TPD chip
	 * needs to be investigated
	 */
	r = gpiod_set_debounce(ddata->hpd_gpio, HPD_DEBOUNCE_TIME);
	if (r)
		goto err;

	r = devm_request_threaded_irq(&pdev->dev, gpiod_to_irq(ddata->hpd_gpio),
		NULL, tpd_hpd_isr,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
		"tpd12s015 hpd", ddata);
	if (r)
		goto err;

	dssdev = &ddata->dssdev;
	dssdev->ops = &tpd_ops;
	dssdev->dev = &pdev->dev;
	dssdev->type = OMAP_DISPLAY_TYPE_HDMI;
	dssdev->owner = THIS_MODULE;
	dssdev->of_ports = BIT(1) | BIT(0);
	dssdev->ops_flags = OMAP_DSS_DEVICE_OP_DETECT
			  | OMAP_DSS_DEVICE_OP_EDID
			  | OMAP_DSS_DEVICE_OP_HPD;

	dssdev->next = omapdss_of_find_connected_device(pdev->dev.of_node, 1);
	if (IS_ERR(dssdev->next)) {
		if (PTR_ERR(dssdev->next) != -EPROBE_DEFER)
			dev_err(&pdev->dev, "failed to find video sink\n");
		r = PTR_ERR(dssdev->next);
		goto err;
	}

	omapdss_device_register(dssdev);

	return 0;
err:
	tpd_uninit_i2c_mux(pdev);
	return r;
}

static int __exit tpd_remove(struct platform_device *pdev)
{
	struct panel_drv_data *ddata = platform_get_drvdata(pdev);
	struct omap_dss_device *dssdev = &ddata->dssdev;

	tpd_uninit_i2c_mux(pdev);

	if (dssdev->next)
		omapdss_device_put(dssdev->next);
	omapdss_device_unregister(&ddata->dssdev);

	return 0;
}

static const struct of_device_id tpd_of_match[] = {
	{ .compatible = "omapdss,ti,dra7evm-tpd12s015", },
	{},
};

MODULE_DEVICE_TABLE(of, tpd_of_match);

static struct platform_driver tpd_driver = {
	.probe	= tpd_probe,
	.remove	= __exit_p(tpd_remove),
	.driver	= {
		.name	= "dra7evm-tpd12s015",
		.of_match_table = tpd_of_match,
		.suppress_bind_attrs = true,
	},
};

module_platform_driver(tpd_driver);

MODULE_AUTHOR("Tomi Valkeinen <tomi.valkeinen@ti.com>");
MODULE_DESCRIPTION("TPD12S015 driver");
MODULE_LICENSE("GPL");
