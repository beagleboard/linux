/*
 * DRA7 EVM TPD12S015 HDMI ESD protection & level shifter chip driver
 *
 * Copyright (C) 2013 Texas Instruments
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/suspend.h>
#include <linux/of_platform.h>
#include <linux/pinctrl/consumer.h>
#include <linux/spinlock.h>

#include "../dss/omapdss.h"

#define SEL_I2C2	0
#define SEL_HDMI	1

int dra7_mcasp_hdmi_gpio_get(struct platform_device *pdev);
int dra7_mcasp_hdmi_gpio_put(struct platform_device *pdev);
int dra7_mcasp_hdmi_gpio_set(struct platform_device *pdev, bool high);

/* HPD gpio debounce time in microseconds */
#define HPD_DEBOUNCE_TIME	1000

struct panel_drv_data {
	struct omap_dss_device dssdev;
	struct omap_dss_device *in;
	void (*hpd_cb)(void *cb_data, enum drm_connector_status status);
	void *hpd_cb_data;
	struct mutex hpd_lock;

	int ct_cp_hpd_gpio;
	int ls_oe_gpio;
	int hpd_gpio;

	struct omap_video_timings timings;

	struct pinctrl *pins;
	struct pinctrl_state *pin_state_i2c;
	struct pinctrl_state *pin_state_ddc;

	struct i2c_adapter *ddc_i2c_adapter;
};

static struct platform_device *mcasp;

static int hdmi_i2c2_hack_init(struct device *dev)
{
	struct device_node *node;

	node = of_parse_phandle(dev->of_node, "mcasp-gpio", 0);

	if (!node)
		return -ENODEV;

	mcasp = of_find_device_by_node(node);

	return 0;
}

static int hdmi_i2c2_hack_resume_mcasp(void)
{
	return dra7_mcasp_hdmi_gpio_get(mcasp);
}

static void hdmi_i2c2_hack_suspend_mcasp(void)
{
	dra7_mcasp_hdmi_gpio_put(mcasp);
}

static int hdmi_i2c2_hack_pm_notif(struct notifier_block *b, unsigned long v, void *d)
{
	switch (v) {
	case PM_SUSPEND_PREPARE:
		hdmi_i2c2_hack_suspend_mcasp();
		return 0;

	case PM_POST_SUSPEND:
		hdmi_i2c2_hack_resume_mcasp();
		return 0;

	default:
		return 0;
	}
}

static struct notifier_block hdmi_i2c2_hack_pm_notif_block = {
	.notifier_call = hdmi_i2c2_hack_pm_notif,
};

/*
 * use SEL_I2C2 to configure pcf8575@26 to set/unset LS_OE and CT_HPD, and use
 * SEL_HDMI to read edid via the HDMI ddc lines
 */
static void hdmi_i2c2_hack_demux(struct device *dev, int sel)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct panel_drv_data *ddata = platform_get_drvdata(pdev);

	/*
	 * switch to I2C2 or HDMI DDC internal pinmux and drive MCASP8_AXR2
	 * to low or high to select I2C2 or HDMI path respectively
	 */
	if (sel == SEL_I2C2) {
		pinctrl_select_state(ddata->pins, ddata->pin_state_i2c);
		dra7_mcasp_hdmi_gpio_set(mcasp, false);
	} else {
		pinctrl_select_state(ddata->pins, ddata->pin_state_ddc);
		dra7_mcasp_hdmi_gpio_set(mcasp, true);
	}

	/* let it propagate */
	udelay(5);
}

#define to_panel_data(x) container_of(x, struct panel_drv_data, dssdev)

static int tpd_connect(struct omap_dss_device *dssdev,
		struct omap_dss_device *dst)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	int r;

	r = in->ops.hdmi->connect(in, dssdev);
	if (r)
		return r;

	dst->src = dssdev;
	dssdev->dst = dst;

	gpio_set_value_cansleep(ddata->ct_cp_hpd_gpio, 1);
	/* DC-DC converter needs at max 300us to get to 90% of 5V */
	udelay(300);

	/*
	 * The HPD GPIO debounce causes a delay until we see the real HPD state.
	 * If tpd_read_edid() or tpd_detect() are called very soon after setting
	 * the ct_cp_hpd-gpio, we could observe wrong HPD value. So sleep here
	 * until the GPIO values has become valid.
	 */
	msleep(DIV_ROUND_UP(HPD_DEBOUNCE_TIME, 1000));

	return 0;
}

static void tpd_disconnect(struct omap_dss_device *dssdev,
		struct omap_dss_device *dst)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	WARN_ON(dst != dssdev->dst);

	if (dst != dssdev->dst)
		return;

	gpio_set_value_cansleep(ddata->ct_cp_hpd_gpio, 0);

	dst->src = NULL;
	dssdev->dst = NULL;

	in->ops.hdmi->disconnect(in, &ddata->dssdev);
}

static int tpd_enable(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	int r;

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		return 0;

	in->ops.hdmi->set_timings(in, &ddata->timings);

	r = in->ops.hdmi->enable(in);
	if (r)
		return r;

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	return r;
}

static void tpd_disable(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return;

	in->ops.hdmi->disable(in);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static void tpd_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	ddata->timings = *timings;
	dssdev->panel.timings = *timings;

	in->ops.hdmi->set_timings(in, timings);
}

static void tpd_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);

	*timings = ddata->timings;
}

static int tpd_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	int r;

	r = in->ops.hdmi->check_timings(in, timings);

	return r;
}

static int tpd_read_edid(struct omap_dss_device *dssdev,
		u8 *edid, int len)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	int r = 0;

	if (!gpio_get_value_cansleep(ddata->hpd_gpio))
		return -ENODEV;

	if (gpio_is_valid(ddata->ls_oe_gpio))
		gpio_set_value_cansleep(ddata->ls_oe_gpio, 1);

	i2c_lock_adapter(ddata->ddc_i2c_adapter);

	hdmi_i2c2_hack_demux(dssdev->dev, SEL_HDMI);

	r = in->ops.hdmi->read_edid(in, edid, len);

	hdmi_i2c2_hack_demux(dssdev->dev, SEL_I2C2);

	i2c_unlock_adapter(ddata->ddc_i2c_adapter);

	if (gpio_is_valid(ddata->ls_oe_gpio))
		gpio_set_value_cansleep(ddata->ls_oe_gpio, 0);

	return r;
}

static bool tpd_detect(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);

	return gpio_get_value_cansleep(ddata->hpd_gpio);
}

static int tpd_enable_hpd(struct omap_dss_device *dssdev,
			   void (*cb)(void *cb_data,
				      enum drm_connector_status status),
			   void *cb_data)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);

	mutex_lock(&ddata->hpd_lock);
	ddata->hpd_cb = cb;
	ddata->hpd_cb_data = cb_data;
	mutex_unlock(&ddata->hpd_lock);

	return 0;
}

static void tpd_disable_hpd(struct omap_dss_device *dssdev)
{
	tpd_enable_hpd(dssdev, NULL, NULL);
}

static int tpd_set_infoframe(struct omap_dss_device *dssdev,
		const struct hdmi_avi_infoframe *avi)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	return in->ops.hdmi->set_infoframe(in, avi);
}

static int tpd_set_hdmi_mode(struct omap_dss_device *dssdev,
		bool hdmi_mode)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	return in->ops.hdmi->set_hdmi_mode(in, hdmi_mode);
}

static const struct omapdss_hdmi_ops tpd_hdmi_ops = {
	.connect		= tpd_connect,
	.disconnect		= tpd_disconnect,

	.enable			= tpd_enable,
	.disable		= tpd_disable,

	.check_timings		= tpd_check_timings,
	.set_timings		= tpd_set_timings,
	.get_timings		= tpd_get_timings,

	.read_edid		= tpd_read_edid,
	.detect			= tpd_detect,
	.enable_hpd		= tpd_enable_hpd,
	.disable_hpd		= tpd_disable_hpd,
	.set_infoframe		= tpd_set_infoframe,
	.set_hdmi_mode		= tpd_set_hdmi_mode,
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

static int tpd_probe_of(struct platform_device *pdev)
{
	struct panel_drv_data *ddata = platform_get_drvdata(pdev);
	struct device_node *node = pdev->dev.of_node;
	struct omap_dss_device *in;
	int gpio;

	/* CT CP HPD GPIO */
	gpio = of_get_gpio(node, 0);
	if (!gpio_is_valid(gpio)) {
		dev_err(&pdev->dev, "failed to parse CT CP HPD gpio\n");
		return gpio;
	}
	ddata->ct_cp_hpd_gpio = gpio;

	/* LS OE GPIO */
	gpio = of_get_gpio(node, 1);
	if (gpio_is_valid(gpio) || gpio == -ENOENT) {
		ddata->ls_oe_gpio = gpio;
	} else {
		dev_err(&pdev->dev, "failed to parse LS OE gpio\n");
		return gpio;
	}

	/* HPD GPIO */
	gpio = of_get_gpio(node, 2);
	if (!gpio_is_valid(gpio)) {
		dev_err(&pdev->dev, "failed to parse HPD gpio\n");
		return gpio;
	}
	ddata->hpd_gpio = gpio;

	in = omapdss_of_find_source_for_first_ep(node);
	if (IS_ERR(in)) {
		dev_err(&pdev->dev, "failed to find video source\n");
		return PTR_ERR(in);
	}

	ddata->in = in;

	return 0;
}

static int tpd_init_pins(struct platform_device *pdev)
{
	struct panel_drv_data *ddata = platform_get_drvdata(pdev);
	struct device_node *node;

	ddata->pins = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(ddata->pins))
		return PTR_ERR(ddata->pins);

	ddata->pin_state_i2c = pinctrl_lookup_state(ddata->pins, "i2c");
	if (IS_ERR(ddata->pin_state_i2c))
		return PTR_ERR(ddata->pin_state_i2c);

	ddata->pin_state_ddc = pinctrl_lookup_state(ddata->pins, "ddc");
	if (IS_ERR(ddata->pin_state_ddc))
		return PTR_ERR(ddata->pin_state_ddc);

	node = of_parse_phandle(pdev->dev.of_node, "ddc-i2c-bus", 0);
	if (!node)
		return -ENODEV;

	ddata->ddc_i2c_adapter = of_find_i2c_adapter_by_node(node);
	if (!ddata->ddc_i2c_adapter)
		return -ENODEV;

	return 0;
}

static void tpd_uninit_pins(struct platform_device *pdev)
{
	struct panel_drv_data *ddata = platform_get_drvdata(pdev);

	i2c_put_adapter(ddata->ddc_i2c_adapter);
}

static int tpd_probe(struct platform_device *pdev)
{
	struct omap_dss_device *in, *dssdev;
	struct panel_drv_data *ddata;
	int r;

	ddata = devm_kzalloc(&pdev->dev, sizeof(*ddata), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	platform_set_drvdata(pdev, ddata);

	if (pdev->dev.of_node) {
		r = tpd_probe_of(pdev);
		if (r)
			return r;
	} else {
		return -ENODEV;
	}

	r = tpd_init_pins(pdev);
	if (r)
		goto err_pins;

	/*
	 * initialize the SEL_HDMI_I2C2 line going to the demux. Configure the
	 * demux to select the I2C2 bus
	 */
	r = hdmi_i2c2_hack_init(&pdev->dev);
	if (r)
		goto err_hack;

	r = hdmi_i2c2_hack_resume_mcasp();
	if (r)
		goto err_hack;

	hdmi_i2c2_hack_demux(&pdev->dev, SEL_I2C2);

	register_pm_notifier(&hdmi_i2c2_hack_pm_notif_block);

	r = devm_gpio_request_one(&pdev->dev, ddata->ct_cp_hpd_gpio,
			GPIOF_OUT_INIT_LOW, "hdmi_ct_cp_hpd");
	if (r)
		goto err_gpio;

	if (gpio_is_valid(ddata->ls_oe_gpio)) {
		r = devm_gpio_request_one(&pdev->dev, ddata->ls_oe_gpio,
				GPIOF_OUT_INIT_LOW, "hdmi_ls_oe");
		if (r)
			goto err_gpio;
	}

	r = devm_gpio_request_one(&pdev->dev, ddata->hpd_gpio,
			GPIOF_DIR_IN, "hdmi_hpd");
	if (r)
		goto err_gpio;

	/*
	 * we see some low voltage glitches on the HPD_B line before it
	 * stabalizes to around 5V. We see the effects of this glitch on the
	 * HPD_A side, and hence on the gpio on DRA7x. The glitch is quite short
	 * in duration, but it takes a while for the voltage to go down back to
	 * 0 volts, we set a debounce value of 1 millisecond to prevent this,
	 * the reason for the glitch not being taken care of by the TPD chip
	 * needs to be investigated
	 */
	r = gpio_set_debounce(ddata->hpd_gpio, HPD_DEBOUNCE_TIME);
	if (r)
		goto err_debounce;

	mutex_init(&ddata->hpd_lock);

	r = devm_request_threaded_irq(&pdev->dev, gpio_to_irq(ddata->hpd_gpio),
		NULL, tpd_hpd_isr,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
		"tpd12s015 hpd", ddata);
	if (r)
		goto err_debounce;

	dssdev = &ddata->dssdev;
	dssdev->ops.hdmi = &tpd_hdmi_ops;
	dssdev->dev = &pdev->dev;
	dssdev->type = OMAP_DISPLAY_TYPE_HDMI;
	dssdev->output_type = OMAP_DISPLAY_TYPE_HDMI;
	dssdev->owner = THIS_MODULE;
	dssdev->port_num = 1;

	in = ddata->in;

	r = omapdss_register_output(dssdev);
	if (r) {
		dev_err(&pdev->dev, "Failed to register output\n");
		goto err_reg;
	}

	return 0;
err_reg:
err_debounce:
err_gpio:
	hdmi_i2c2_hack_suspend_mcasp();
err_hack:
	tpd_uninit_pins(pdev);
err_pins:
	omap_dss_put_device(ddata->in);
	return r;
}

static int __exit tpd_remove(struct platform_device *pdev)
{
	struct panel_drv_data *ddata = platform_get_drvdata(pdev);
	struct omap_dss_device *dssdev = &ddata->dssdev;
	struct omap_dss_device *in = ddata->in;

	tpd_uninit_pins(pdev);

	unregister_pm_notifier(&hdmi_i2c2_hack_pm_notif_block);

	hdmi_i2c2_hack_suspend_mcasp();

	omapdss_unregister_output(&ddata->dssdev);

	WARN_ON(omapdss_device_is_enabled(dssdev));
	if (omapdss_device_is_enabled(dssdev))
		tpd_disable(dssdev);

	WARN_ON(omapdss_device_is_connected(dssdev));
	if (omapdss_device_is_connected(dssdev))
		tpd_disconnect(dssdev, dssdev->dst);

	omap_dss_put_device(in);

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
		.owner	= THIS_MODULE,
		.of_match_table = tpd_of_match,
	},
};

module_platform_driver(tpd_driver);

MODULE_AUTHOR("Tomi Valkeinen <tomi.valkeinen@ti.com>");
MODULE_DESCRIPTION("TPD12S015 driver");
MODULE_LICENSE("GPL");
