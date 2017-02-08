/*
 * SCI Clock driver for keystone based devices
 *
 * Copyright (C) 2015-2016 Texas Instruments Incorporated - http://www.ti.com/
 *	Tero Kristo <t-kristo@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/soc/ti/ti_sci_protocol.h>
#include <dt-bindings/clock/k2g.h>
#include <dt-bindings/genpd/k2g.h>

#define SCI_CLK_SSC_ENABLE		BIT(0)
#define SCI_CLK_ALLOW_FREQ_CHANGE	BIT(1)
#define SCI_CLK_INPUT_TERMINATION	BIT(2)

/**
 * struct sci_clk_data - TI SCI clock data
 * @dev: device index
 * @num_clks: number of clocks for this device
 * @clocks: clocks array for this device
 */
struct sci_clk_data {
	u16 dev;
	u16 num_clks;
	struct clk_hw **clocks;
};

/**
 * struct sci_clk_provider - TI SCI clock provider representation
 * @sci:    Handle to the System Control Interface protocol handler
 * @ops:    Pointer to the SCI ops to be used by the clocks
 * @dev:    Device pointer for the clock provider
 * @clocks:	Clock data
 */
struct sci_clk_provider {
	const struct ti_sci_handle *sci;
	const struct ti_sci_clk_ops *ops;
	struct device *dev;
	struct sci_clk_data *clocks;
};

/**
 * struct sci_clk - TI SCI clock representation
 * @hw:		 Hardware clock cookie for common clock framework
 * @dev_id:	 Device index
 * @clk_id:	 Clock index
 * @node:	 Clocks list link
 * @provider:	 Master clock provider
 * @flags:	 Flags for the clock
 */
struct sci_clk {
	struct clk_hw hw;
	u16 dev_id;
	u8 clk_id;
	struct list_head node;
	struct sci_clk_provider *provider;
	u8 flags;
};

#define to_sci_clk(_hw) container_of(_hw, struct sci_clk, hw)

/**
 * sci_clk_prepare - Prepare (enable) a TI SCI clock
 * @hw: clock to prepare
 *
 * Prepares a clock to be actively used. Returns the SCI protocol status.
 */
static int sci_clk_prepare(struct clk_hw *hw)
{
	struct sci_clk *clk = to_sci_clk(hw);
	bool enable_ssc = clk->flags & SCI_CLK_SSC_ENABLE;
	bool allow_freq_change = clk->flags & SCI_CLK_ALLOW_FREQ_CHANGE;
	bool input_termination = clk->flags & SCI_CLK_INPUT_TERMINATION;

	return clk->provider->ops->get_clock(clk->provider->sci, clk->dev_id,
					     clk->clk_id, enable_ssc,
					     allow_freq_change,
					     input_termination);
}

/**
 * sci_clk_unprepare - Un-prepares (disables) a TI SCI clock
 * @hw: clock to unprepare
 *
 * Un-prepares a clock from active state.
 */
static void sci_clk_unprepare(struct clk_hw *hw)
{
	struct sci_clk *clk = to_sci_clk(hw);
	int ret;

	ret = clk->provider->ops->put_clock(clk->provider->sci, clk->dev_id,
					    clk->clk_id);
	if (ret)
		dev_err(clk->provider->dev,
			"unprepare failed for dev=%d, clk=%d, ret=%d\n",
			clk->dev_id, clk->clk_id, ret);
}

/**
 * sci_clk_is_prepared - Check if a TI SCI clock is prepared or not
 * @hw: clock to check status for
 *
 * Checks if a clock is prepared (enabled) in hardware. Returns non-zero
 * value if clock is enabled, zero otherwise.
 */
static int sci_clk_is_prepared(struct clk_hw *hw)
{
	struct sci_clk *clk = to_sci_clk(hw);
	bool req_state, current_state;
	int ret;

	ret = clk->provider->ops->is_on(clk->provider->sci, clk->dev_id,
					clk->clk_id, &req_state,
					&current_state);
	if (ret) {
		dev_err(clk->provider->dev,
			"is_prepared failed for dev=%d, clk=%d, ret=%d\n",
			clk->dev_id, clk->clk_id, ret);
		return 0;
	}

	return req_state;
}

/**
 * sci_clk_recalc_rate - Get clock rate for a TI SCI clock
 * @hw: clock to get rate for
 * @parent_rate: parent rate provided by common clock framework, not used
 *
 * Gets the current clock rate of a TI SCI clock. Returns the current
 * clock rate, or zero in failure.
 */
static unsigned long sci_clk_recalc_rate(struct clk_hw *hw,
					 unsigned long parent_rate)
{
	struct sci_clk *clk = to_sci_clk(hw);
	u64 freq;
	int ret;

	ret = clk->provider->ops->get_freq(clk->provider->sci, clk->dev_id,
					   clk->clk_id, &freq);
	if (ret) {
		dev_err(clk->provider->dev,
			"recalc-rate failed for dev=%d, clk=%d, ret=%d\n",
			clk->dev_id, clk->clk_id, ret);
		return 0;
	}

	return (u32)freq;
}

/**
 * sci_clk_determine_rate - Determines a clock rate a clock can be set to
 * @hw: clock to change rate for
 * @req: requested rate configuration for the clock
 *
 * Determines a suitable clock rate and parent for a TI SCI clock.
 * The parent handling is un-used, as generally the parent clock rates
 * are not known by the kernel; instead these are internally handled
 * by the firmware. Returns 0 on success, negative error value on failure.
 */
static int sci_clk_determine_rate(struct clk_hw *hw,
				  struct clk_rate_request *req)
{
	struct sci_clk *clk = to_sci_clk(hw);
	int ret;
	u64 new_rate;

	ret = clk->provider->ops->get_best_match_freq(clk->provider->sci,
						      clk->dev_id,
						      clk->clk_id,
						      req->min_rate,
						      req->rate,
						      req->max_rate,
						      &new_rate);
	if (ret) {
		dev_err(clk->provider->dev,
			"determine-rate failed for dev=%d, clk=%d, ret=%d\n",
			clk->dev_id, clk->clk_id, ret);
		return ret;
	}

	req->rate = new_rate;

	return 0;
}

/**
 * sci_clk_set_rate - Set rate for a TI SCI clock
 * @hw: clock to change rate for
 * @rate: target rate for the clock
 * @parent_rate: rate of the clock parent, not used for TI SCI clocks
 *
 * Sets a clock frequency for a TI SCI clock. Returns the TI SCI
 * protocol status.
 */
static int sci_clk_set_rate(struct clk_hw *hw, unsigned long rate,
			    unsigned long parent_rate)
{
	struct sci_clk *clk = to_sci_clk(hw);

	return clk->provider->ops->set_freq(clk->provider->sci, clk->dev_id,
					    clk->clk_id, rate, rate, rate);
}

/**
 * sci_clk_get_parent - Get the current parent of a TI SCI clock
 * @hw: clock to get parent for
 *
 * Returns the index of the currently selected parent for a TI SCI clock.
 */
static u8 sci_clk_get_parent(struct clk_hw *hw)
{
	struct sci_clk *clk = to_sci_clk(hw);
	u8 parent_id;
	int ret;

	ret = clk->provider->ops->get_parent(clk->provider->sci, clk->dev_id,
					     clk->clk_id, &parent_id);
	if (ret) {
		dev_err(clk->provider->dev,
			"get-parent failed for dev=%d, clk=%d, ret=%d\n",
			clk->dev_id, clk->clk_id, ret);
		return 0;
	}

	return parent_id - clk->clk_id - 1;
}

/**
 * sci_clk_set_parent - Set the parent of a TI SCI clock
 * @hw: clock to set parent for
 * @index: new parent index for the clock
 *
 * Sets the parent of a TI SCI clock. Return TI SCI protocol status.
 */
static int sci_clk_set_parent(struct clk_hw *hw, u8 index)
{
	struct sci_clk *clk = to_sci_clk(hw);

	return clk->provider->ops->set_parent(clk->provider->sci, clk->dev_id,
					      clk->clk_id,
					      index + 1 + clk->clk_id);
}

static const struct clk_ops sci_clk_ops = {
	.prepare = sci_clk_prepare,
	.unprepare = sci_clk_unprepare,
	.is_prepared = sci_clk_is_prepared,
	.recalc_rate = sci_clk_recalc_rate,
	.determine_rate = sci_clk_determine_rate,
	.set_rate = sci_clk_set_rate,
	.get_parent = sci_clk_get_parent,
	.set_parent = sci_clk_set_parent,
};

/**
 * _sci_clk_get - Gets a handle for an SCI clock
 * @provider: Handle to SCI clock provider
 * @dev_id: device ID for the clock to register
 * @clk_id: clock ID for the clock to register
 *
 * Gets a handle to an existing TI SCI hw clock, or builds a new clock
 * entry and registers it with the common clock framework. Called from
 * the common clock framework, when a corresponding of_clk_get call is
 * executed, or recursively from itself when parsing parent clocks.
 * Returns a pointer to the hw clock struct, or ERR_PTR value in failure.
 */
static struct clk_hw *_sci_clk_build(struct sci_clk_provider *provider,
				     u16 dev_id, u8 clk_id)
{
	struct clk_init_data init = { NULL };
	struct sci_clk *sci_clk = NULL;
	char *name = NULL;
	char **parent_names = NULL;
	int i;
	int ret;

	sci_clk = devm_kzalloc(provider->dev, sizeof(*sci_clk), GFP_KERNEL);
	if (!sci_clk)
		return ERR_PTR(-ENOMEM);

	sci_clk->dev_id = dev_id;
	sci_clk->clk_id = clk_id;
	sci_clk->provider = provider;

	ret = provider->ops->get_num_parents(provider->sci, dev_id,
					     clk_id,
					     &init.num_parents);
	if (ret)
		goto err;

	name = kasprintf(GFP_KERNEL, "%s:%d:%d", dev_name(provider->dev),
			 sci_clk->dev_id, sci_clk->clk_id);

	init.name = name;

	if (init.num_parents < 2)
		init.num_parents = 0;

	if (init.num_parents) {
		parent_names = devm_kcalloc(provider->dev, init.num_parents,
					    sizeof(char *), GFP_KERNEL);

		if (!parent_names) {
			ret = -ENOMEM;
			goto err;
		}

		for (i = 0; i < init.num_parents; i++) {
			char *parent_name;

			parent_name = kasprintf(GFP_KERNEL, "%s:%d:%d",
						dev_name(provider->dev),
						sci_clk->dev_id,
						sci_clk->clk_id + 1 + i);
			if (!parent_name) {
				ret = -ENOMEM;
				goto err;
			}
			parent_names[i] = parent_name;
		}
		init.parent_names = (const char * const *)parent_names;
	}

	init.ops = &sci_clk_ops;
	sci_clk->hw.init = &init;

	ret = devm_clk_hw_register(provider->dev, &sci_clk->hw);
	if (ret) {
		dev_err(provider->dev, "failed clk register with %d\n", ret);
		goto err;
	}
	kfree(name);

	return &sci_clk->hw;

err:
	if (parent_names) {
		for (i = 0; i < init.num_parents; i++)
			devm_kfree(provider->dev, parent_names[i]);

		devm_kfree(provider->dev, parent_names);
	}

	devm_kfree(provider->dev, sci_clk);

	kfree(name);

	return ERR_PTR(ret);
}

/**
 * sci_clk_get - Xlate function for getting clock handles
 * @clkspec: device tree clock specifier
 * @data: pointer to the clock provider
 *
 * Xlate function for retrieving clock TI SCI hw clock handles based on
 * device tree clock specifier. Called from the common clock framework,
 * when a corresponding of_clk_get call is executed. Returns a pointer
 * to the TI SCI hw clock struct, or ERR_PTR value in failure.
 */
static struct clk_hw *sci_clk_get(struct of_phandle_args *clkspec, void *data)
{
	struct sci_clk_provider *provider = data;
	u16 dev_id;
	u8 clk_id;
	struct sci_clk_data *clks = provider->clocks;

	if (clkspec->args_count != 2)
		return ERR_PTR(-EINVAL);

	dev_id = clkspec->args[0];
	clk_id = clkspec->args[1];

	while (clks->num_clks) {
		if (clks->dev == dev_id) {
			if (clk_id >= clks->num_clks)
				return ERR_PTR(-EINVAL);

			return clks->clocks[clk_id];
		}

		clks++;
	}

	return ERR_PTR(-ENODEV);
}

int ti_sci_init_clocks(struct sci_clk_provider *p)
{
	struct sci_clk_data *data = p->clocks;
	struct clk_hw *hw;
	int i;

	while (data->num_clks) {
		data->clocks = devm_kcalloc(p->dev, data->num_clks,
					    sizeof(struct sci_clk),
					    GFP_KERNEL);
		if (!data->clocks)
			return -ENOMEM;

		for (i = 0; i < data->num_clks; i++) {
			hw = _sci_clk_build(p, data->dev, i);
			if (!IS_ERR(hw)) {
				data->clocks[i] = hw;
				continue;
			}

			/* Skip any holes in the clock lists */
			if (PTR_ERR(hw) == -ENODEV)
				continue;

			return PTR_ERR(hw);
		}
		data++;
	}

	return 0;
}

static const struct sci_clk_data k2g_clk_data[] = {
	{ .dev = K2G_DEV_PMMC0, .num_clks = K2G_DEV_PMMC_MPM_DAP_CLK + 1 },
	{ .dev = K2G_DEV_MLB0, .num_clks = K2G_DEV_MLB_MLBP_IO_CLK + 1 },
	{ .dev = K2G_DEV_DSS0, .num_clks = K2G_DEV_DSS_PI_DSS_VP_CLK + 1 },
	{ .dev = K2G_DEV_MCBSP0, .num_clks = K2G_DEV_MCBSP_CLKS_PARENT_UART_PLL + 1 },
	{ .dev = K2G_DEV_MCASP0, .num_clks = K2G_DEV_MCASP_AUX_CLK_PARENT_UART_PLL + 1 },
	{ .dev = K2G_DEV_MCASP1, .num_clks = K2G_DEV_MCASP_AUX_CLK_PARENT_UART_PLL + 1 },
	{ .dev = K2G_DEV_MCASP2, .num_clks = K2G_DEV_MCASP_AUX_CLK_PARENT_UART_PLL + 1 },
	{ .dev = K2G_DEV_DCAN0, .num_clks = K2G_DEV_DCAN_CAN_CLK + 1 },
	{ .dev = K2G_DEV_DCAN1, .num_clks = K2G_DEV_DCAN_CAN_CLK + 1 },
	{ .dev = K2G_DEV_EMIF0, .num_clks = K2G_DEV_EMIF_VBUSP_CLK + 1 },
	{ .dev = K2G_DEV_MMCHS0, .num_clks = K2G_DEV_MMCHS_CLK32K + 1 },
	{ .dev = K2G_DEV_MMCHS1, .num_clks = K2G_DEV_MMCHS_CLK32K + 1 },
	{ .dev = K2G_DEV_GPMC0, .num_clks = K2G_DEV_GPMC_GPMC_FCLK + 1 },
	{ .dev = K2G_DEV_ELM0, .num_clks = K2G_DEV_ELM_CLK + 1 },
	{ .dev = K2G_DEV_SPI0, .num_clks = K2G_DEV_SPI_VBUSP_CLK + 1 },
	{ .dev = K2G_DEV_SPI1, .num_clks = K2G_DEV_SPI_VBUSP_CLK + 1 },
	{ .dev = K2G_DEV_SPI2, .num_clks = K2G_DEV_SPI_VBUSP_CLK + 1 },
	{ .dev = K2G_DEV_SPI3, .num_clks = K2G_DEV_SPI_VBUSP_CLK + 1 },
	{ .dev = K2G_DEV_ICSS0, .num_clks = K2G_DEV_ICSS_IEPCLK_CLK + 1 },
	{ .dev = K2G_DEV_ICSS1, .num_clks = K2G_DEV_ICSS_IEPCLK_CLK + 1 },
	{ .dev = K2G_DEV_USB0, .num_clks = K2G_DEV_USB_CLKCORE + 1 },
	{ .dev = K2G_DEV_USB1, .num_clks = K2G_DEV_USB_CLKCORE + 1 },
	{ .dev = K2G_DEV_NSS0, .num_clks = K2G_DEV_NSS_RMII_MHZ_50_CLK + 1 },
	{ .dev = K2G_DEV_PCIE0, .num_clks = K2G_DEV_PCIE_VBUS_CLK + 1 },
	{ .dev = K2G_DEV_GPIO0, .num_clks = K2G_DEV_GPIO_VBUS_CLK + 1 },
	{ .dev = K2G_DEV_GPIO1, .num_clks = K2G_DEV_GPIO_VBUS_CLK + 1 },
	{ .dev = K2G_DEV_TIMER64_0, .num_clks = K2G_DEV_TIMER64_TOUTH + 1 },
	{ .dev = K2G_DEV_TIMER64_1, .num_clks = K2G_DEV_TIMER64_TOUTH + 1 },
	{ .dev = K2G_DEV_TIMER64_2, .num_clks = K2G_DEV_TIMER64_TOUTH + 1 },
	{ .dev = K2G_DEV_TIMER64_3, .num_clks = K2G_DEV_TIMER64_TOUTH + 1 },
	{ .dev = K2G_DEV_TIMER64_4, .num_clks = K2G_DEV_TIMER64_TOUTH + 1 },
	{ .dev = K2G_DEV_TIMER64_5, .num_clks = K2G_DEV_TIMER64_TOUTH + 1 },
	{ .dev = K2G_DEV_TIMER64_6, .num_clks = K2G_DEV_TIMER64_TOUTH + 1 },
	{ .dev = K2G_DEV_MSGMGR0, .num_clks = K2G_DEV_MSGMGR_VBUS_CLK + 1 },
	{ .dev = K2G_DEV_BOOTCFG0, .num_clks = K2G_DEV_BOOTCFG_VBUS_CLK + 1 },
	{ .dev = K2G_DEV_ARM_BOOTROM0, .num_clks = K2G_DEV_ARM_BOOTROM_VBUS_CLK + 1 },
	{ .dev = K2G_DEV_DSP_BOOTROM0, .num_clks = K2G_DEV_DSP_BOOTROM_VBUS_CLK + 1 },
	{ .dev = K2G_DEV_DEBUGSS0, .num_clks = K2G_DEV_DEBUGSS_STMXPT_CLK + 1 },
	{ .dev = K2G_DEV_UART0, .num_clks = K2G_DEV_UART_CBA_CLK_PI + 1 },
	{ .dev = K2G_DEV_UART1, .num_clks = K2G_DEV_UART_CBA_CLK_PI + 1 },
	{ .dev = K2G_DEV_UART2, .num_clks = K2G_DEV_UART_CBA_CLK_PI + 1 },
	{ .dev = K2G_DEV_EHRPWM0, .num_clks = K2G_DEV_EHRPWM_VBUS_CLK + 1 },
	{ .dev = K2G_DEV_EHRPWM1, .num_clks = K2G_DEV_EHRPWM_VBUS_CLK + 1 },
	{ .dev = K2G_DEV_EHRPWM2, .num_clks = K2G_DEV_EHRPWM_VBUS_CLK + 1 },
	{ .dev = K2G_DEV_EHRPWM3, .num_clks = K2G_DEV_EHRPWM_VBUS_CLK + 1 },
	{ .dev = K2G_DEV_EHRPWM4, .num_clks = K2G_DEV_EHRPWM_VBUS_CLK + 1 },
	{ .dev = K2G_DEV_EHRPWM5, .num_clks = K2G_DEV_EHRPWM_VBUS_CLK + 1 },
	{ .dev = K2G_DEV_EQEP0, .num_clks = K2G_DEV_EQEP_VBUS_CLK + 1 },
	{ .dev = K2G_DEV_EQEP1, .num_clks = K2G_DEV_EQEP_VBUS_CLK + 1 },
	{ .dev = K2G_DEV_EQEP2, .num_clks = K2G_DEV_EQEP_VBUS_CLK + 1 },
	{ .dev = K2G_DEV_ECAP0, .num_clks = K2G_DEV_ECAP_VBUS_CLK + 1 },
	{ .dev = K2G_DEV_ECAP1, .num_clks = K2G_DEV_ECAP_VBUS_CLK + 1 },
	{ .dev = K2G_DEV_I2C0, .num_clks = K2G_DEV_I2C_VBUS_CLK + 1 },
	{ .dev = K2G_DEV_I2C1, .num_clks = K2G_DEV_I2C_VBUS_CLK + 1 },
	{ .dev = K2G_DEV_I2C2, .num_clks = K2G_DEV_I2C_VBUS_CLK + 1 },
	{ .dev = K2G_DEV_EDMA0, .num_clks = K2G_DEV_EDMA_TPCC_CLK + 1 },
	{ .dev = K2G_DEV_SEMAPHORE0, .num_clks = K2G_DEV_SEMAPHORE_VBUS_CLK + 1 },
	{ .dev = K2G_DEV_INTC0, .num_clks = K2G_DEV_INTC_VBUS_CLK + 1 },
	{ .dev = K2G_DEV_GIC0, .num_clks = K2G_DEV_GIC_VBUS_CLK + 1 },
	{ .dev = K2G_DEV_QSPI0, .num_clks = K2G_DEV_QSPI_QSPI_CLK_I + 1 },
	{ .dev = K2G_DEV_ARM_64B_COUNTER0, .num_clks = K2G_DEV_ARM_64B_COUNTER_VBUSP_CLK + 1 },
	{ .dev = K2G_DEV_TETRIS0, .num_clks = K2G_DEV_TETRIS_SUBSYS_CLK + 1 },
	{ .dev = K2G_DEV_CGEM0, .num_clks = K2G_DEV_CGEM_TRACE_CLK + 1 },
	{ .dev = K2G_DEV_MSMC0, .num_clks = K2G_DEV_MSMC_VBUS_CLK + 1 },
	{ .dev = K2G_DEV_CBASS0, .num_clks = K2G_DEV_CBASS_VBUS_CLK + 1 },
	{ .dev = K2G_DEV_BOARD0, .num_clks = K2G_DEV_BOARD_TIMO_PARENT_TIMER64_5H + 1 },
	{ .dev = K2G_DEV_EDMA1, .num_clks = K2G_DEV_EDMA_TPCC_CLK + 1 },
	{ .num_clks = 0 },
};

static const struct of_device_id ti_sci_clk_of_match[] = {
	{ .compatible = "ti,k2g-sci-clk", .data = &k2g_clk_data },
	{ /* Sentinel */ },
};
MODULE_DEVICE_TABLE(of, ti_sci_clk_of_match);

/**
 * ti_sci_clk_probe - Probe function for the TI SCI clock driver
 * @pdev: platform device pointer to be probed
 *
 * Probes the TI SCI clock device. Allocates a new clock provider
 * and registers this to the common clock framework. Also applies
 * any required flags to the identified clocks via clock lists
 * supplied from DT. Returns 0 for success, negative error value
 * for failure.
 */
static int ti_sci_clk_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct sci_clk_provider *provider;
	const struct ti_sci_handle *handle;
	struct sci_clk_data *data;
	int ret;

	data = (struct sci_clk_data *)
		of_match_node(ti_sci_clk_of_match, np)->data;

	handle = devm_ti_sci_get_handle(dev);
	if (IS_ERR(handle))
		return PTR_ERR(handle);

	provider = devm_kzalloc(dev, sizeof(*provider), GFP_KERNEL);
	if (!provider)
		return -ENOMEM;

	provider->clocks = data;

	provider->sci = handle;
	provider->ops = &handle->ops.clk_ops;
	provider->dev = dev;

	ti_sci_init_clocks(provider);

	ret = of_clk_add_hw_provider(np, sci_clk_get, provider);
	if (ret)
		return ret;

	return 0;
}

/**
 * ti_sci_clk_remove - Remove TI SCI clock device
 * @pdev: platform device pointer for the device to be removed
 *
 * Removes the TI SCI device. Unregisters the clock provider registered
 * via common clock framework. Any memory allocated for the device will
 * be free'd silently via the devm framework. Returns 0 always.
 */
static int ti_sci_clk_remove(struct platform_device *pdev)
{
	of_clk_del_provider(pdev->dev.of_node);

	return 0;
}

static struct platform_driver ti_sci_clk_driver = {
	.probe = ti_sci_clk_probe,
	.remove = ti_sci_clk_remove,
	.driver = {
		.name = "ti-sci-clk",
		.of_match_table = of_match_ptr(ti_sci_clk_of_match),
	},
};
module_platform_driver(ti_sci_clk_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("TI System Control Interface(SCI) Clock driver");
MODULE_AUTHOR("Tero Kristo");
MODULE_ALIAS("platform:ti-sci-clk");
