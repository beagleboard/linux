/*
* Copyright (C) 2014 Texas Instruments Ltd
*
* This program is free software; you can redistribute it and/or modify it
* under the terms of the GNU General Public License version 2 as published by
* the Free Software Foundation.
*
* You should have received a copy of the GNU General Public License along with
* this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/sched.h>

#include <video/omapdss.h>

#include "dss.h"

static struct {
	struct regulator *pll_reg;

	struct pll_data *pll[2];
	unsigned scp_refcount[2];
	void __iomem *clk_ctrl[2];
} dpll;

#define CLK_CTRL_GET(id, start, end) \
	FLD_GET(clk_ctrl_read(id), start, end)

#define CLK_CTRL_FLD_MOD(id, val, start, end) \
	clk_ctrl_write(id, FLD_MOD(clk_ctrl_read(id), val, start, end))

#define	PLL_POWER_OFF		0x0
#define PLL_POWER_ON_ALL	0x2

static inline u32 clk_ctrl_read(int id)
{
	return __raw_readl(dpll.clk_ctrl[id]);
}

static inline void clk_ctrl_write(int id, u32 val)
{
	__raw_writel(val, dpll.clk_ctrl[id]);
}

struct pll_data *dss_dpll_get_pll_data(int id)
{
	return dpll.pll[id];
}

static void dss_dpll_disable_scp_clk(int id)
{
	unsigned *refcount;

	refcount = &dpll.scp_refcount[id];

	WARN_ON(*refcount == 0);
	if (--(*refcount) == 0)
		CLK_CTRL_FLD_MOD(id, 0, 14, 14); /* CIO_CLK_ICG */
}

static void dss_dpll_enable_scp_clk(int id)
{
	unsigned *refcount;

	refcount = &dpll.scp_refcount[id];

	if ((*refcount)++ == 0)
		CLK_CTRL_FLD_MOD(id, 1, 14, 14); /* CIO_CLK_ICG */
}

static int dss_dpll_power(int id, int state)
{
	/* PLL_PWR_CMD = enable both hsdiv and clkout*/
	CLK_CTRL_FLD_MOD(id, state, 31, 30);

	/*
	 * DRA7x has a bug when it comes to reading PLL_PWR_STATUS, we wait for
	 * 100 us here
	 */

	udelay(100);

	return 0;
}

static int dss_dpll_enable(struct pll_data *pll)
{
	int r;
	int id = pll == dpll.pll[0] ? 0 : 1;

	if (!dpll.pll_reg) {
		struct regulator *reg;

		reg = devm_regulator_get(&pll->pdev->dev, "vdda_video");

		if (IS_ERR(reg)) {
			if (PTR_ERR(reg) != -EPROBE_DEFER)
				DSSERR("can't get DPLL VDDA regulator\n");
			return PTR_ERR(reg);
		}

		dpll.pll_reg = reg;
	}

	r = dss_runtime_get();
	if (r)
		return r;

	dss_ctrl_pll_enable(id, true);

	pll_enable_clock(pll, true);

	dss_dpll_enable_scp_clk(id);

	r = regulator_enable(dpll.pll_reg);
	if (r)
		goto err_reg;

	r = pll_wait_reset(pll);
	if (r)
		goto err_reset;

	r = dss_dpll_power(id, PLL_POWER_ON_ALL);
	if (r)
		goto err_power;

	return 0;

err_power:
err_reset:
	regulator_disable(dpll.pll_reg);
err_reg:
	pll_enable_clock(pll, false);
	dss_ctrl_pll_enable(id, false);
	dss_runtime_put();

	return r;
}

static void dss_dpll_disable(struct pll_data *pll)
{
	int id = pll == dpll.pll[0] ? 0 : 1;

	dss_dpll_power(id, PLL_POWER_OFF);
	regulator_disable(dpll.pll_reg);

	dss_dpll_disable_scp_clk(id);

	dss_ctrl_pll_enable(id, false);
	pll_enable_clock(pll, false);
	dss_runtime_put();
}

static struct pll_ops dss_dpll_ops = {
	.enable = dss_dpll_enable,
	.disable = dss_dpll_disable,
};

static const char * const reg_name[] = { "pll1", "pll2" };
static const char * const clk_name[] = { "video1_clk", "video2_clk" };
static const char * const clkctrl_name[] = { "pll1_clkctrl", "pll2_clkctrl" };

int dss_dpll_init(struct platform_device *pdev, int id)
{
	struct resource *res;

	dpll.pll[id] = pll_create(pdev, reg_name[id], clk_name[id],
				DSS_PLL_TYPE_DSI, &dss_dpll_ops, 0);
	if (!dpll.pll[id]) {
		dev_err(&pdev->dev, "failed to create PLL%d instance\n", id);
		return -ENODEV;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
		clkctrl_name[id]);
	if (!res) {
		dev_err(&pdev->dev,
			"missing platform resource data for pll%d\n", id);
		return -ENODEV;
	}

	dpll.clk_ctrl[id] = devm_ioremap_resource(&pdev->dev, res);
	if (!dpll.clk_ctrl[id]) {
		dev_err(&pdev->dev, "failed to ioremap pll%d clkctrl\n", id);
		return -ENOMEM;
	}

	return 0;
}
