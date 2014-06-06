#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/wait.h>
#include <linux/slab.h>

#include <video/omapdss.h>

#include "dss.h"

#define PLL_SZ				0x20

#define PLL_CONTROL			0x0000
#define PLL_STATUS			0x0004
#define PLL_GO				0x0008
#define PLL_CONFIGURATION1		0x000C
#define PLL_CONFIGURATION2		0x0010
#define PLL_CONFIGURATION3		0x0014
#define PLL_SSC_CONFIGURATION1		0x0018
#define PLL_SSC_CONFIGURATION2		0x001C
#define PLL_CONFIGURATION4		0x0020

struct pll_features {
	unsigned long regm_max, regn_max;
	unsigned long regm_hsdiv_max;
	unsigned long fint_min, fint_max;
	unsigned long clkout_max;

	unsigned long dco_range1_min, dco_range1_max;
	unsigned long dco_range2_min, dco_range2_max;

	u8 regm_start, regm_end;
	u8 regn_start, regn_end;
	u8 regm_hsdiv_start[4], regm_hsdiv_end[4];

	bool freqsel;
	bool refsel;
	bool sysreset_fsm;
};

static inline void pll_write_reg(void __iomem *base_addr, const u16 idx,
		u32 val)
{
	__raw_writel(val, base_addr + idx);
}

static inline u32 pll_read_reg(void __iomem *base_addr, const u16 idx)
{
	return __raw_readl(base_addr + idx);
}

#define REG_FLD_MOD(base, idx, val, start, end) \
	pll_write_reg(base, idx, FLD_MOD(pll_read_reg(base, idx),\
							val, start, end))
#define REG_GET(base, idx, start, end) \
	FLD_GET(pll_read_reg(base, idx), start, end)

static inline int wait_for_bit_change(void __iomem *base, const u16 offset,
		int bitnum, int value)
{
	int t = 100;

	while (t-- > 0) {
		if (REG_GET(base, offset, bitnum, bitnum) == value)
			return value;
		udelay(1);
	}

	return !value;
}

void pll_dump(struct pll_data *pll, struct seq_file *s)
{
#define DUMPPLL(r) seq_printf(s, "%-35s %08x\n", #r,\
		pll_read_reg(pll->base, r))

	DUMPPLL(PLL_CONTROL);
	DUMPPLL(PLL_STATUS);
	DUMPPLL(PLL_GO);
	DUMPPLL(PLL_CONFIGURATION1);
	DUMPPLL(PLL_CONFIGURATION2);
	DUMPPLL(PLL_CONFIGURATION3);
	DUMPPLL(PLL_SSC_CONFIGURATION1);
	DUMPPLL(PLL_SSC_CONFIGURATION2);
	DUMPPLL(PLL_CONFIGURATION4);

#undef DUMPPLL
}

unsigned long pll_get_clkin(struct pll_data *pll)
{
	return clk_get_rate(pll->clkin);
}

unsigned long pll_get_hsdiv_rate(struct pll_data *pll, int index)
{
	return pll->params.clkout_hsdiv[index];
}

int pll_wait_hsdiv_active(struct pll_data *pll, int index)
{
	u8 bit;

	switch (index) {
	case 0:
		bit = 7; break;
	case 1:
		bit = 8; break;
	case 2:
		bit = 10; break;
	case 3:
		bit = 11; break;
	default:
		return 0;
	};

	if (wait_for_bit_change(pll->base, PLL_STATUS, bit, 1) != 1)
		return -EINVAL;

	return 0;
}

void pll_sysreset(struct pll_data *pll)
{
	REG_FLD_MOD(pll->base, PLL_CONTROL, pll->feats->sysreset_fsm, 3, 3);
}

int pll_get_reset_status(struct pll_data *pll)
{
	return REG_GET(pll->base, PLL_STATUS, 0, 0);
}

int pll_wait_reset(struct pll_data *pll)
{

	if (wait_for_bit_change(pll->base, PLL_STATUS, 0, 1) != 1) {
		DSSERR("PLL not coming out of reset.\n");
		return -ENODEV;
	}

	return 0;
}

void pll_enable_clock(struct pll_data *pll, bool enable)
{
	if (enable) {
		clk_prepare_enable(pll->clkin);
	} else {
		clk_disable_unprepare(pll->clkin);
		pll->locked = 0;
	}

	if (enable && pll->locked) {
		if (wait_for_bit_change(pll->base, PLL_STATUS, 1, 1) != 1)
			DSSERR("cannot lock PLL when enabling clocks\n");
	}
}

int pll_calc_and_check_clock_rates(struct pll_data *pll,
		struct pll_params *params)
{
	int i;
	struct pll_features *feats = pll->feats;

	if (params->regn == 0 || params->regn > feats->regn_max)
		return -EINVAL;

	if (params->regm == 0 || params->regm > feats->regm_max)
		return -EINVAL;

	for (i = 0; i < 4; i++) {
		if (params->regm_hsdiv[i] > feats->regm_hsdiv_max)
			return -EINVAL;
	}

	params->clkin = clk_get_rate(pll->clkin);
	params->fint = params->clkin / params->regn;

	if (params->fint > feats->fint_max ||
			params->fint < feats->fint_min)
		return -EINVAL;

	params->clkout = 2 * params->regm * params->fint;

	if (params->clkout > feats->clkout_max)
		return -EINVAL;

	for (i = 0; i < 4; i++) {
		if (params->regm_hsdiv[i] > 0)
			params->clkout_hsdiv[i] =
				params->clkout / params->regm_hsdiv[i];
		else
			params->clkout_hsdiv[i] = 0;
	}

	return 0;
}

bool pll_hsdiv_calc(struct pll_data *pll, unsigned long clkout,
		unsigned long hsdiv_out_min, unsigned long hsdiv_out_max,
		pll_hsdiv_calc_func func, void *data)
{
	int regm, regm_start, regm_stop;
	unsigned long hsdiv_out;

	hsdiv_out_min = hsdiv_out_min ? hsdiv_out_min : 1;

	regm_start = max(DIV_ROUND_UP(clkout, hsdiv_out_max), 1ul);
	regm_stop = min(clkout / hsdiv_out_min, pll->feats->regm_hsdiv_max);

	for (regm = regm_start; regm <= regm_stop; ++regm) {
		hsdiv_out = clkout / regm;

		if (func(regm, hsdiv_out, data))
			return true;
	}

	return false;
}

bool pll_calc(struct pll_data *pll, unsigned long clkout_min,
		unsigned long clkout_max, pll_calc_func func, void *data)
{
	struct pll_features *feats = pll->feats;
	int regn, regn_start, regn_stop;
	int regm, regm_start, regm_stop;
	unsigned long fint, clkout;
	unsigned long clkout_hw_max;
	unsigned long fint_hw_min, fint_hw_max;
	unsigned long clkin = clk_get_rate(pll->clkin);

	fint_hw_min = feats->fint_min;
	fint_hw_max = feats->fint_max;
	clkout_hw_max = feats->clkout_max;

	regn_start = max(DIV_ROUND_UP(clkin, fint_hw_max), 1ul);
	regn_stop = min(clkin / fint_hw_min, feats->regn_max);

	clkout_max = clkout_max ? clkout_max : ULONG_MAX;

	for (regn = regn_start; regn <= regn_stop; ++regn) {
		fint = clkin / regn;

		regm_start = max(DIV_ROUND_UP(DIV_ROUND_UP(clkout_min, fint),
					2), 1ul);
		regm_stop = min3(clkout_max / fint / 2,
				clkout_hw_max / fint / 2,
				feats->regm_max);

		for (regm = regm_start; regm <= regm_stop; ++regm) {
			clkout = 2 * regm * fint;

			if (func(regn, regm, fint, clkout, data))
				return true;
		}
	}

	return false;
}

int pll_set_clock_div(struct pll_data *pll, struct pll_params *params)
{
	int i, r = 0, f = 0;
	u32 l;
	struct pll_features *feats = pll->feats;

	pll->params = *params;

	/* PLL_AUTOMODE = manual */
	REG_FLD_MOD(pll->base, PLL_CONTROL, 0, 0, 0);

	l = pll_read_reg(pll->base, PLL_CONFIGURATION1);
	l = FLD_MOD(l, 1, 0, 0);		/* PLL_STOPMODE */
	/* PLL_REGN */
	l = FLD_MOD(l, params->regn - 1, feats->regn_start, feats->regn_end);
	/* PLL_REGM */
	l = FLD_MOD(l, params->regm, feats->regm_start, feats->regm_end);

	for (i = 0; i < 4; i++) {
		if (params->hsdiv_enabled[i])
			l = FLD_MOD(l, params->regm_hsdiv[i] > 0 ?
				params->regm_hsdiv[i] - 1 : 0,
				feats->regm_hsdiv_start[i],
				feats->regm_hsdiv_end[i]);
	}

	pll_write_reg(pll->base, PLL_CONFIGURATION1, l);

	WARN_ON(params->fint < feats->fint_min ||
		params->fint > feats->fint_max);

	l = pll_read_reg(pll->base, PLL_CONFIGURATION2);

	if (feats->freqsel) {
		f = params->fint < 1000000 ? 0x3 :
			params->fint < 1250000 ? 0x4 :
			params->fint < 1500000 ? 0x5 :
			params->fint < 1750000 ? 0x6 :
			0x7;

		l = FLD_MOD(l, f, 4, 1);	/* PLL_FREQSEL */
	}

	l = FLD_MOD(l, 1, 13, 13);		/* PLL_REFEN */
	l = FLD_MOD(l, 0, 14, 14);		/* CLKOUT_EN */
	l = FLD_MOD(l, 1, 20, 20);		/* HSDIVBYPASS */

	if (feats->refsel)
		l = FLD_MOD(l, 3, 22, 21);	/* REF_SYSCLK = sysclk */
	pll_write_reg(pll->base, PLL_CONFIGURATION2, l);

	l = pll_read_reg(pll->base, PLL_CONFIGURATION3);
	for (i = 2; i <= 3; i++) {
		if (params->hsdiv_enabled[i])
			l = FLD_MOD(l, params->regm_hsdiv[i] > 0 ?
				params->regm_hsdiv[i] - 1 : 0,
				feats->regm_hsdiv_start[i],
				feats->regm_hsdiv_end[i]);
	}
	pll_write_reg(pll->base, PLL_CONFIGURATION3, l);

	REG_FLD_MOD(pll->base, PLL_GO, 1, 0, 0);	/* DSI_PLL_GO */

	if (wait_for_bit_change(pll->base, PLL_GO, 0, 0) != 0) {
		DSSERR("dsi pll go bit not going down.\n");
		r = -EIO;
		goto err;
	}

	if (wait_for_bit_change(pll->base, PLL_STATUS, 1, 1) != 1) {
		DSSERR("cannot lock PLL\n");
		r = -EIO;
		goto err;
	}

	pll->locked = 1;

	l = pll_read_reg(pll->base, PLL_CONFIGURATION2);
	l = FLD_MOD(l, 0, 0, 0);	/* PLL_IDLE */
	l = FLD_MOD(l, 0, 5, 5);	/* PLL_PLLLPMODE */
	l = FLD_MOD(l, 0, 6, 6);	/* PLL_LOWCURRSTBY */
	l = FLD_MOD(l, 0, 7, 7);	/* PLL_TIGHTPHASELOCK */
	l = FLD_MOD(l, 0, 8, 8);	/* PLL_DRIFTGUARDEN */
	l = FLD_MOD(l, 0, 10, 9);	/* PLL_LOCKSEL */
	l = FLD_MOD(l, 1, 13, 13);	/* PLL_REFEN */
	l = FLD_MOD(l, 1, 14, 14);	/* PHY_CLKINEN */
	l = FLD_MOD(l, 0, 15, 15);	/* BYPASSEN */
	/* HSDIV1_CLOCK_EN */
	l = FLD_MOD(l, params->hsdiv_enabled[0], 16, 16);
	l = FLD_MOD(l, 0, 17, 17);	/* DSS_CLOCK_PWDN */
	/* HSDIV2_CLOCK_EN */
	l = FLD_MOD(l, params->hsdiv_enabled[1], 18, 18);
	l = FLD_MOD(l, 0, 19, 19);	/* PROTO_CLOCK_PWDN */
	l = FLD_MOD(l, 0, 20, 20);	/* HSDIVBYPASS */
	/* HSDIV3_CLOCK_EN */
	l = FLD_MOD(l, params->hsdiv_enabled[2], 23, 23);
	/* HSDIV4_CLOCK_EN */
	l = FLD_MOD(l, params->hsdiv_enabled[3], 25, 25);
	pll_write_reg(pll->base, PLL_CONFIGURATION2, l);

err:
	return r;
}

static struct pll_features omap34xx_pll_features = {
	.regn_max = (1 << 7) - 1,
	.regm_max = (1 << 11) - 1,
	.regm_hsdiv_max = (1 << 4) - 1,
	.fint_min = 750000,
	.fint_max = 2100000,
	.clkout_max = 1800000000UL,
	.regm_start = 18,
	.regm_end = 8,
	.regn_start = 7,
	.regn_end = 1,
	.regm_hsdiv_start = { 22, 26, 0, 0 },
	.regm_hsdiv_end = { 19, 23, 0, 0 },
	.freqsel = true,
	.refsel = false,
	.sysreset_fsm = false,
};

static struct pll_features omap36xx_pll_features = {
	.regn_max = (1 << 7) - 1,
	.regm_max = (1 << 11) - 1,
	.regm_hsdiv_max = (1 << 4) - 1,
	.fint_min = 150000,
	.fint_max = 52000000,
	.clkout_max = 1800000000UL,
	.regm_start = 18,
	.regm_end = 8,
	.regn_start = 7,
	.regn_end = 1,
	.regm_hsdiv_start = { 22, 26, 0, 0 },
	.regm_hsdiv_end = { 19, 23, 0, 0 },
	.freqsel = false,
	.refsel = false,
	.sysreset_fsm = false,
};

static struct pll_features omap44xx_pll_features = {
	.regn_max = (1 << 8) - 1,
	.regm_max = (1 << 12) - 1,
	.regm_hsdiv_max = (1 << 5) - 1,
	.fint_min = 500000,
	.fint_max = 2500000,
	.clkout_max = 1800000000UL,
	.dco_range1_min = 500000000UL,
	.dco_range1_max = 1000000000UL,
	.dco_range2_min = 1000000000UL,
	.dco_range2_max = 2000000000UL,
	.regm_start = 20,
	.regm_end = 9,
	.regn_start = 8,
	.regn_end = 1,
	.regm_hsdiv_start = { 25, 30, 4, 9 },
	.regm_hsdiv_end = { 21, 26, 0, 5 },
	.freqsel = false,
	.refsel = true,
	.sysreset_fsm = false,
};

static struct pll_features omap54xx_pll_features = {
	.regn_max = (1 << 8) - 1,
	.regm_max = (1 << 12) - 1,
	.regm_hsdiv_max = (1 << 5) - 1,
	.fint_min = 150000,
	.fint_max = 52000000,
	.clkout_max = 2500000000UL,
	.dco_range1_min = 750000000UL,
	.dco_range1_max = 1500000000UL,
	.dco_range2_min = 1250000000UL,
	.dco_range2_max = 2500000000UL,
	.regm_start = 20,
	.regm_end = 9,
	.regn_start = 8,
	.regn_end = 1,
	.regm_hsdiv_start = { 25, 30, 4, 9 },
	.regm_hsdiv_end = { 21, 26, 0, 5 },
	.freqsel = false,
	.refsel = true,
	.sysreset_fsm = true,
};

static int __init pll_init_features(struct pll_data *pll)
{

	switch (omapdss_get_version()) {
	case OMAPDSS_VER_OMAP34xx_ES1:
	case OMAPDSS_VER_OMAP34xx_ES3:
	case OMAPDSS_VER_AM35xx:
		pll->feats = &omap34xx_pll_features;
		break;

	case OMAPDSS_VER_OMAP3630:
		pll->feats = &omap36xx_pll_features;
		break;

	case OMAPDSS_VER_OMAP4430_ES1:
	case OMAPDSS_VER_OMAP4430_ES2:
	case OMAPDSS_VER_OMAP4:
		pll->feats = &omap44xx_pll_features;
		break;

	case OMAPDSS_VER_OMAP5:
		pll->feats = &omap54xx_pll_features;
		break;

	case OMAPDSS_VER_OMAP24xx:
	default:
		return -ENODEV;
	}

	return 0;
}

struct pll_data *pll_create(struct platform_device *pdev,
		const char *res_name, const char *clk_name,
		enum pll_type type, struct pll_ops *ops,
		u32 offset)
{
	struct resource *res, temp_res;
	struct pll_data *pll;
	int r;

	pll = devm_kzalloc(&pdev->dev, sizeof(*pll), GFP_KERNEL);
	if (!pll) {
		dev_err(&pdev->dev, "couldn't alloc pll_data\n");
		return ERR_PTR(-ENOMEM);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, res_name);
	if (!res) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!res) {
			DSSERR("can't get top level resource\n");
			return ERR_PTR(-EINVAL);
		}

		temp_res.start = res->start + offset;
		temp_res.end = temp_res.start + PLL_SZ - 1;
		res = &temp_res;
	}

	pll->base = devm_ioremap_resource(&pdev->dev, res);
	if (!pll->base) {
		dev_err(&pdev->dev, "failed to ioremap pll\n");
		return ERR_PTR(-ENOMEM);
	}

	pll->clkin = devm_clk_get(&pdev->dev, clk_name);
	if (IS_ERR(pll->clkin)) {
		DSSERR("can't get clock %s\n", clk_name);
		return ERR_PTR(-ENODEV);
	}

	pll->pdev = pdev;
	pll->type = type;
	pll->ops = ops;

	r = pll_init_features(pll);
	if (r)
		return ERR_PTR(r);

	return pll;
}
