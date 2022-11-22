// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2021 Alibaba Group Holding Limited.
 *
 */
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/slab.h>
#include <dt-bindings/pinctrl/light-mpw-pinctrl.h>
#include <dt-bindings/pinctrl/light-fm-right-pinctrl.h>
#include <dt-bindings/pinctrl/light-fm-left-pinctrl.h>
#include <dt-bindings/pinctrl/light-fm-aon-pinctrl.h>

#include "../core.h"

struct light_pin {
	unsigned int pin_id;
	unsigned int mux_mode;
	unsigned long config;
};

struct light_pin_group {
	const char *name;
	unsigned int npins;
	unsigned int *pin_ids;
	struct light_pin *pins;
};

struct light_pmx_func {
	const char *name;
	const char **groups;
	unsigned int num_groups;
};

struct light_pinctrl_soc_info {
	struct device *dev;
	struct pinctrl_desc *desc;
	const struct pinctrl_pin_desc *pins;
	unsigned int npins;
	struct light_pin_group *groups;
	unsigned int ngroups;
	unsigned int grp_index;
	struct light_pmx_func *functions;
	unsigned int nfunctions;

	unsigned int cfg_off;
	unsigned int mux_off;
	int (*covert_pin_off)(const unsigned int pin_id);
};

struct light_pinctrl {
	struct device *dev;
	struct pinctrl_dev *pctl;
	void __iomem *base;
	const struct light_pinctrl_soc_info *info;
};

#define LIGHT_PINCTRL_PIN(pin)	PINCTRL_PIN(pin, #pin)
#define LIGHT_PAD_CONFIG(idx)	(priv->info->cfg_off + (idx >> 1) * 4)
#define LIGHT_PAD_MUX(idx)	(priv->info->mux_off + ((idx >> 3) * 4))

static const char *pin_get_name_from_info(struct light_pinctrl_soc_info *info,
					  const unsigned int pin_id)
{
	int i;

	for (i = 0; i < info->npins; i++) {
		if (info->pins[i].number == pin_id)
			return info->pins[i].name;
	}

	return NULL;
}

static inline const struct light_pin_group *light_pinctrl_find_group_by_name(
				const struct light_pinctrl_soc_info *info,
				const char *name)
{
	const struct light_pin_group *grp = NULL;
	unsigned int i;

	for (i = 0; i < info->ngroups; i++) {
		if (!strcmp(info->groups[i].name, name)) {
			grp = &info->groups[i];
			break;
		}
	}

	return grp;
}

static int light_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct light_pinctrl *priv = pinctrl_dev_get_drvdata(pctldev);
	const struct light_pinctrl_soc_info *info = priv->info;

	return info->ngroups;
}

static const char *light_get_group_name(struct pinctrl_dev *pctldev,
				       unsigned int selector)
{
	struct light_pinctrl *priv = pinctrl_dev_get_drvdata(pctldev);
	const struct light_pinctrl_soc_info *info = priv->info;

	return info->groups[selector].name;
}

static int light_get_group_pins(struct pinctrl_dev *pctldev,
			       unsigned int selector, const unsigned int **pins,
			       unsigned int *npins)
{
	struct light_pinctrl *priv = pinctrl_dev_get_drvdata(pctldev);
	const struct light_pinctrl_soc_info *info = priv->info;

	if (selector >= info->ngroups)
		return -EINVAL;

	*pins = info->groups[selector].pin_ids;
	*npins = info->groups[selector].npins;

	return 0;
}

static void light_pin_dbg_show(struct pinctrl_dev *pctldev, struct seq_file *s,
			      unsigned int offset)
{
	seq_printf(s, "%s", dev_name(pctldev->dev));
}

static int light_dt_node_to_map(struct pinctrl_dev *pctldev,
			       struct device_node *np,
			       struct pinctrl_map **map, unsigned int *num_maps)
{
	struct light_pinctrl *priv = pinctrl_dev_get_drvdata(pctldev);
	const struct light_pinctrl_soc_info *info = priv->info;
	const struct light_pin_group *grp;
	struct pinctrl_map *new_map;
	struct device_node *parent;
	int map_num = 1;
	int i, j;

	/*
	 * first find the group of this node and check if we need create
	 * config maps for pins
	 */
	grp = light_pinctrl_find_group_by_name(info, np->name);
	if (!grp) {
		dev_err(info->dev, "unable to find group for node %s\n",
			np->name);
		return -EINVAL;
	}

	for (i = 0; i < grp->npins; i++)
		map_num++;

	new_map = kmalloc_array(map_num, sizeof(struct pinctrl_map),
				GFP_KERNEL);
	if (!new_map)
		return -ENOMEM;

	*map = new_map;
	*num_maps = map_num;

	/* create mux map */
	parent = of_get_parent(np);
	if (!parent) {
		kfree(new_map);
		return -EINVAL;
	}
	new_map[0].type = PIN_MAP_TYPE_MUX_GROUP;
	new_map[0].data.mux.function = parent->name;
	new_map[0].data.mux.group = np->name;
	of_node_put(parent);

	/* create config map */
	new_map++;
	for (i = j = 0; i < grp->npins; i++) {
		new_map[j].type = PIN_MAP_TYPE_CONFIGS_PIN;
		new_map[j].data.configs.group_or_pin =
			pin_get_name(pctldev, grp->pins[i].pin_id);
		new_map[j].data.configs.configs = &grp->pins[i].config;
		new_map[j].data.configs.num_configs = 1;
		j++;
	}

	dev_dbg(pctldev->dev, "maps: function %s group %s num %d\n",
		(*map)->data.mux.function, (*map)->data.mux.group, map_num);

	return 0;
}

static void light_dt_free_map(struct pinctrl_dev *pctldev,
			     struct pinctrl_map *map, unsigned int num_maps)
{
	kfree(map);
}

static const struct pinctrl_ops light_pctrl_ops = {
	.get_groups_count = light_get_groups_count,
	.get_group_name = light_get_group_name,
	.get_group_pins = light_get_group_pins,
	.pin_dbg_show = light_pin_dbg_show,
	.dt_node_to_map = light_dt_node_to_map,
	.dt_free_map = light_dt_free_map,

};

static int light_pmx_set(struct pinctrl_dev *pctldev, unsigned int selector,
			unsigned int group)
{
	struct light_pinctrl *priv = pinctrl_dev_get_drvdata(pctldev);
	const struct light_pinctrl_soc_info *info = priv->info;
	unsigned int npins, pin_id;
	int i;
	struct light_pin_group *grp;

	/*
	 * Configure the mux mode for each pin in the group for a specific
	 * function.
	 */
	grp = &info->groups[group];
	npins = grp->npins;

	dev_dbg(priv->dev, "enable function %s group %s\n",
		info->functions[selector].name, grp->name);

	for (i = 0; i < npins; i++) {
		struct light_pin *pin = &grp->pins[i];
		unsigned int mux, shift;

		dev_dbg(priv->dev, "pinmux set pin %s\n",
			pin_get_name(pctldev, pin->pin_id));

		pin_id = pin->pin_id;
		if (info->covert_pin_off) {
			pin_id = info->covert_pin_off(pin_id);
			if (pin_id < 0) /* no need to configure mux */
				continue;
		}

		mux = readl(priv->base + LIGHT_PAD_MUX(pin_id));
		shift = ((pin_id % 8) << 2);
		mux &= ~(0xF << shift);	/* 4 mux bits for one pad */
		mux |= (pin->mux_mode << shift);
		writel(mux, priv->base + LIGHT_PAD_MUX(pin_id));
		dev_dbg(priv->dev, "write: offset 0x%x val 0x%x\n",
			LIGHT_PAD_MUX(pin_id), mux);
	}

	return 0;
}

static int light_pmx_get_funcs_count(struct pinctrl_dev *pctldev)
{
	struct light_pinctrl *priv = pinctrl_dev_get_drvdata(pctldev);
	const struct light_pinctrl_soc_info *info = priv->info;

	return info->nfunctions;
}

static const char *light_pmx_get_func_name(struct pinctrl_dev *pctldev,
					  unsigned int selector)
{
	struct light_pinctrl *priv = pinctrl_dev_get_drvdata(pctldev);
	const struct light_pinctrl_soc_info *info = priv->info;

	return info->functions[selector].name;
}

static int light_pmx_get_groups(struct pinctrl_dev *pctldev,
			       unsigned int selector,
			       const char * const **groups,
			       unsigned int * const num_groups)
{
	struct light_pinctrl *priv = pinctrl_dev_get_drvdata(pctldev);
	const struct light_pinctrl_soc_info *info = priv->info;

	*groups = info->functions[selector].groups;
	*num_groups = info->functions[selector].num_groups;

	return 0;
}

static const struct pinmux_ops light_pmx_ops = {
	.get_functions_count = light_pmx_get_funcs_count,
	.get_function_name = light_pmx_get_func_name,
	.get_function_groups = light_pmx_get_groups,
	.set_mux = light_pmx_set,
};

static int light_pinconf_get(struct pinctrl_dev *pctldev,
			    unsigned int pin_id, unsigned long *config)
{
	struct light_pinctrl *priv = pinctrl_dev_get_drvdata(pctldev);

	*config = readl(priv->base + LIGHT_PAD_CONFIG(pin_id));

	return 0;
}

static int light_pinconf_set(struct pinctrl_dev *pctldev,
			    unsigned int pin_id, unsigned long *configs,
			    unsigned int num_configs)
{
	struct light_pinctrl *priv = pinctrl_dev_get_drvdata(pctldev);
	int i;

	dev_dbg(priv->dev, "pinconf set pin %s\n",
		pin_get_name(pctldev, pin_id));

	for (i = 0; i < num_configs; i++) {
		unsigned long config;
		unsigned int shift;

		config = readl(priv->base + LIGHT_PAD_CONFIG(pin_id));
		shift = ((pin_id % 2) << 4);
		config &= ~(0xFFFF << shift); /* 16 cfg bits for one pad */
		config |= (configs[i] << shift);
		writel(config, priv->base + LIGHT_PAD_CONFIG(pin_id));
		dev_dbg(priv->dev, "write: offset 0x%x val 0x%lx\n",
			LIGHT_PAD_CONFIG(pin_id), config);
	} /* for each config */

	return 0;
}

static void light_pinconf_dbg_show(struct pinctrl_dev *pctldev,
				  struct seq_file *s, unsigned int pin_id)
{
	struct light_pinctrl *priv = pinctrl_dev_get_drvdata(pctldev);
	unsigned long config;

	config = readl(priv->base + LIGHT_PAD_CONFIG(pin_id));
	seq_printf(s, "0x%lx", config);
}

static void light_pinconf_group_dbg_show(struct pinctrl_dev *pctldev,
					struct seq_file *s, unsigned int group)
{
	struct light_pinctrl *priv = pinctrl_dev_get_drvdata(pctldev);
	const struct light_pinctrl_soc_info *info = priv->info;
	struct light_pin_group *grp;
	unsigned long config;
	const char *name;
	int i, ret;

	if (group > info->ngroups)
		return;

	seq_puts(s, "\n");
	grp = &info->groups[group];
	for (i = 0; i < grp->npins; i++) {
		struct light_pin *pin = &grp->pins[i];

		name = pin_get_name(pctldev, pin->pin_id);
		ret = light_pinconf_get(pctldev, pin->pin_id, &config);
		if (ret)
			return;
		seq_printf(s, "%s: 0x%lx", name, config);
	}
}

static const struct pinconf_ops light_pinconf_ops = {
	.pin_config_get = light_pinconf_get,
	.pin_config_set = light_pinconf_set,
	.pin_config_dbg_show = light_pinconf_dbg_show,
	.pin_config_group_dbg_show = light_pinconf_group_dbg_show,
};

/*
 * Each pin represented in thead,pins consists:
 * - u32 PIN_FUNC_ID
 * - u32 pin mux_mode
 * - u32 pin config
 * so 12 bytes in total for each pin.
 */
#define LIGHT_PIN_SIZE 12

static int light_pinctrl_parse_groups(struct device_node *np,
				     struct light_pin_group *grp,
				     struct light_pinctrl_soc_info *info,
				     u32 index)
{
	int size, i;
	const __be32 *list;

	dev_dbg(info->dev, "group(%d): %s\n", index, np->name);

	/* Initialise group */
	grp->name = np->name;

	/*
	 * the binding format is thead,pins = <PIN MUX CONFIG>,
	 * do sanity check and calculate pins number
	 */
	list = of_get_property(np, "thead,pins", &size);
	if (!list) {
		dev_err(info->dev, "no thead,pins property in node %s\n",
			np->full_name);
		return -EINVAL;
	}

	/* we do not check return since it's safe node passed down */
	if (!size || size % LIGHT_PIN_SIZE) {
		dev_err(info->dev, "Invalid thead,pins property in node %s\n",
			np->full_name);
		return -EINVAL;
	}

	grp->npins = size / LIGHT_PIN_SIZE;
	grp->pins = devm_kzalloc(info->dev,
				 grp->npins * sizeof(struct light_pin),
				 GFP_KERNEL);
	grp->pin_ids = devm_kzalloc(info->dev,
				    grp->npins * sizeof(unsigned int),
				    GFP_KERNEL);
	if (!grp->pins || !grp->pin_ids)
		return -ENOMEM;

	for (i = 0; i < grp->npins; i++) {
		struct light_pin *pin = &grp->pins[i];

		pin->pin_id = be32_to_cpu(*list++);
		pin->mux_mode = be32_to_cpu(*list++) & 0xF;
		pin->config = be32_to_cpu(*list++) & 0xFFFF;
		grp->pin_ids[i] = grp->pins[i].pin_id;

		dev_dbg(info->dev, "%s: 0x%04x 0x%04lx",
			pin_get_name_from_info(info, pin->pin_id), pin->mux_mode, pin->config);
	}

	return 0;
}

static int light_pinctrl_parse_functions(struct device_node *np,
					struct light_pinctrl_soc_info *info,
					u32 index)
{
	struct device_node *child;
	struct light_pmx_func *func;
	struct light_pin_group *grp;
	u32 i = 0;

	dev_dbg(info->dev, "parse function(%d): %s\n", index, np->name);

	func = &info->functions[index];

	/* Initialise function */
	func->name = np->name;
	func->num_groups = of_get_child_count(np);
	if (func->num_groups == 0) {
		dev_err(info->dev, "no groups defined in %s\n", np->full_name);
		return -EINVAL;
	}
	func->groups = devm_kzalloc(info->dev,
				    func->num_groups * sizeof(char *),
				    GFP_KERNEL);

	if (!func->groups)
		return -ENOMEM;

	for_each_child_of_node(np, child) {
		func->groups[i] = child->name;
		grp = &info->groups[info->grp_index++];
		light_pinctrl_parse_groups(child, grp, info, i++);
	}

	return 0;
}

static int light_pinctrl_probe_dt(struct platform_device *pdev,
				  struct light_pinctrl_soc_info *info)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *child;
	u32 nfuncs = 0;
	u32 i = 0;

	if (!np)
		return -ENODEV;

	nfuncs = of_get_child_count(np);
	if (nfuncs <= 0) {
		dev_err(&pdev->dev, "no functions defined\n");
		return -EINVAL;
	}

	info->nfunctions = nfuncs;
	info->functions = devm_kzalloc(&pdev->dev,
				       nfuncs * sizeof(struct light_pmx_func),
				       GFP_KERNEL);
	if (!info->functions)
		return -ENOMEM;

	info->ngroups = 0;
	for_each_child_of_node(np, child)
		info->ngroups += of_get_child_count(child);
	info->groups = devm_kzalloc(&pdev->dev, info->ngroups *
				    sizeof(struct light_pin_group),
				    GFP_KERNEL);
	if (!info->groups)
		return -ENOMEM;

	for_each_child_of_node(np, child)
		light_pinctrl_parse_functions(child, info, i++);

	return 0;
}

static int light_pinctrl_probe(struct platform_device *pdev)
{
	struct light_pinctrl *priv;
	struct light_pinctrl_soc_info *info;
	struct resource *res;
	int ret;

	info = (struct light_pinctrl_soc_info *)of_device_get_match_data(&pdev->dev);
	if (!info || !info->pins || !info->npins) {
		dev_err(&pdev->dev, "wrong pinctrl info\n");
		return -EINVAL;
	}
	info->dev = &pdev->dev;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->base = devm_ioremap_resource(&pdev->dev, res);

	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	info->desc = devm_kzalloc(&pdev->dev, sizeof(*info->desc), GFP_KERNEL);
	if (!info->desc)
		return -ENOMEM;

	info->desc->name = dev_name(&pdev->dev);
	info->desc->pins = info->pins;
	info->desc->npins = info->npins;
	info->desc->pctlops = &light_pctrl_ops;
	info->desc->pmxops = &light_pmx_ops;
	info->desc->confops = &light_pinconf_ops;
	info->desc->owner = THIS_MODULE;

	ret = light_pinctrl_probe_dt(pdev, info);
	if (ret) {
		dev_err(&pdev->dev, "fail to probe dt properties\n");
		return ret;
	}

	priv->info = info;
	priv->dev = info->dev;
	platform_set_drvdata(pdev, priv);
	priv->pctl = pinctrl_register(info->desc, &pdev->dev, priv);
	if (!priv->pctl) {
		dev_err(&pdev->dev, "could not register light pinctrl driver\n");
		return -EINVAL;
	}

	dev_info(&pdev->dev, "initialized light pinctrl driver\n");

	return 0;
}

/* Pad names for the pinmux subsystem */
static const struct pinctrl_pin_desc light_mpw_pinctrl_pads[] = {
	LIGHT_PINCTRL_PIN(BOOT_SEL0),
	LIGHT_PINCTRL_PIN(BOOT_SEL1),
	LIGHT_PINCTRL_PIN(BOOT_SEL2),
	LIGHT_PINCTRL_PIN(OSC_BOOT),
	LIGHT_PINCTRL_PIN(QSPI0_SCK),
	LIGHT_PINCTRL_PIN(QSPI0_SSN0),
	LIGHT_PINCTRL_PIN(QSPI0_M0_MOSI),
	LIGHT_PINCTRL_PIN(QSPI0_M1_MISO),
	LIGHT_PINCTRL_PIN(QSPI0_M2_WP),
	LIGHT_PINCTRL_PIN(QSPI0_M3_HOLD),
	LIGHT_PINCTRL_PIN(QSPI1_SCK),
	LIGHT_PINCTRL_PIN(QSPI1_SSN0),
	LIGHT_PINCTRL_PIN(QSPI1_M0_MOSI),
	LIGHT_PINCTRL_PIN(QSPI1_M1_MISO),
	LIGHT_PINCTRL_PIN(QSPI1_M2_WP),
	LIGHT_PINCTRL_PIN(QSPI1_M3_HOLD),
	LIGHT_PINCTRL_PIN(SPI_SCK),
	LIGHT_PINCTRL_PIN(SPI_SSN0),
	LIGHT_PINCTRL_PIN(SPI_SSN1),
	LIGHT_PINCTRL_PIN(SPI_MOSI),
	LIGHT_PINCTRL_PIN(SPI_MISO),
	LIGHT_PINCTRL_PIN(I2C0_SCL),
	LIGHT_PINCTRL_PIN(I2C0_SDA),
	LIGHT_PINCTRL_PIN(I2C1_SCL),
	LIGHT_PINCTRL_PIN(I2C1_SDA),
	LIGHT_PINCTRL_PIN(I2C2_SCL),
	LIGHT_PINCTRL_PIN(I2C2_SDA),
	LIGHT_PINCTRL_PIN(I2C3_SCL),
	LIGHT_PINCTRL_PIN(I2C3_SDA),
	LIGHT_PINCTRL_PIN(UART0_RXD),
	LIGHT_PINCTRL_PIN(UART0_TXD),
	LIGHT_PINCTRL_PIN(UART1_RXD),
	LIGHT_PINCTRL_PIN(UART1_TXD),
	LIGHT_PINCTRL_PIN(UART3_RXD),
	LIGHT_PINCTRL_PIN(UART3_TXD),
	LIGHT_PINCTRL_PIN(UART4_RXD),
	LIGHT_PINCTRL_PIN(UART4_TXD),
	LIGHT_PINCTRL_PIN(UART4_CTSN),
	LIGHT_PINCTRL_PIN(UART4_RTSN),
	LIGHT_PINCTRL_PIN(GPIO1_DATA7),
	LIGHT_PINCTRL_PIN(GPIO1_DATA8),
	LIGHT_PINCTRL_PIN(GPIO1_DATA9),
	LIGHT_PINCTRL_PIN(GPIO1_DATA10),
	LIGHT_PINCTRL_PIN(GPIO1_DATA11),
	LIGHT_PINCTRL_PIN(GPIO1_DATA12),
	LIGHT_PINCTRL_PIN(GPIO1_DATA13),
	LIGHT_PINCTRL_PIN(GPIO1_DATA14),
	LIGHT_PINCTRL_PIN(GPIO1_DATA15),
	LIGHT_PINCTRL_PIN(GPIO1_DATA16),
	LIGHT_PINCTRL_PIN(GPIO1_DATA17),
	LIGHT_PINCTRL_PIN(GPIO1_DATA18),
	LIGHT_PINCTRL_PIN(GPIO1_DATA19),
	LIGHT_PINCTRL_PIN(GPIO1_DATA20),
	LIGHT_PINCTRL_PIN(GPIO1_DATA21),
	LIGHT_PINCTRL_PIN(GPIO1_DATA22),
	LIGHT_PINCTRL_PIN(GPIO1_DATA23),
	LIGHT_PINCTRL_PIN(GPIO1_DATA24),
	LIGHT_PINCTRL_PIN(GPIO1_DATA25),
	LIGHT_PINCTRL_PIN(GPIO1_DATA26),
	LIGHT_PINCTRL_PIN(EMMC0_DETN),
	LIGHT_PINCTRL_PIN(EMMC0_PWRON),
	LIGHT_PINCTRL_PIN(EMMC0_WPRTN),
	LIGHT_PINCTRL_PIN(EMMC0_VOL_STB),
	LIGHT_PINCTRL_PIN(EMMC1_DETN),
	LIGHT_PINCTRL_PIN(EMMC1_PWRON),
	LIGHT_PINCTRL_PIN(EMMC1_WPRTN),
	LIGHT_PINCTRL_PIN(EMMC1_VOL_STB),
	LIGHT_PINCTRL_PIN(C910_JTG_TCLK),
	LIGHT_PINCTRL_PIN(C910_JTG_TRST),
	LIGHT_PINCTRL_PIN(C910_JTG_TDI),
	LIGHT_PINCTRL_PIN(C910_JTG_TMS),
	LIGHT_PINCTRL_PIN(C910_JTG_TDO),
	LIGHT_PINCTRL_PIN(GPIO2_DATA8),
	LIGHT_PINCTRL_PIN(GPIO2_DATA9),
	LIGHT_PINCTRL_PIN(GPIO2_DATA10),
	LIGHT_PINCTRL_PIN(GPIO2_DATA11),
	LIGHT_PINCTRL_PIN(GPIO2_DATA12),
	LIGHT_PINCTRL_PIN(GPIO2_DATA13),
	LIGHT_PINCTRL_PIN(GPIO2_DATA14),
	LIGHT_PINCTRL_PIN(RGM_PAD_MS_RTE),
	LIGHT_PINCTRL_PIN(GMAC_EPHY_CLK),
	LIGHT_PINCTRL_PIN(GMAC_TX_CLK),
	LIGHT_PINCTRL_PIN(GMAC_RX_CLK),
	LIGHT_PINCTRL_PIN(GMAC_TXEN),
	LIGHT_PINCTRL_PIN(GMAC_TXD0),
	LIGHT_PINCTRL_PIN(GMAC_TXD1),
	LIGHT_PINCTRL_PIN(GMAC_TXD2),
	LIGHT_PINCTRL_PIN(GMAC_TXD3),
	LIGHT_PINCTRL_PIN(GMAC_RXDV),
	LIGHT_PINCTRL_PIN(GMAC_RXD0),
	LIGHT_PINCTRL_PIN(GMAC_RXD1),
	LIGHT_PINCTRL_PIN(GMAC_RXD2),
	LIGHT_PINCTRL_PIN(GMAC_RXD3),
	LIGHT_PINCTRL_PIN(GMAC_MDC),
	LIGHT_PINCTRL_PIN(GMAC_MDIO),
	LIGHT_PINCTRL_PIN(GMAC_COL),
	LIGHT_PINCTRL_PIN(GMAC_CRS),
};

static const struct pinctrl_pin_desc light_fm_right_pinctrl_pads[] = {
	LIGHT_PINCTRL_PIN(FM_UART0_TXD),
	LIGHT_PINCTRL_PIN(FM_UART0_RXD),
	LIGHT_PINCTRL_PIN(FM_QSPI0_SCLK),
	LIGHT_PINCTRL_PIN(FM_QSPI0_CSN0),
	LIGHT_PINCTRL_PIN(FM_QSPI0_CSN1),
	LIGHT_PINCTRL_PIN(FM_QSPI0_D0_MOSI),
	LIGHT_PINCTRL_PIN(FM_QSPI0_D1_MISO),
	LIGHT_PINCTRL_PIN(FM_QSPI0_D2_WP),
	LIGHT_PINCTRL_PIN(FM_QSPI0_D3_HOLD),
	LIGHT_PINCTRL_PIN(FM_I2C2_SCL),
	LIGHT_PINCTRL_PIN(FM_I2C2_SDA),
	LIGHT_PINCTRL_PIN(FM_I2C3_SCL),
	LIGHT_PINCTRL_PIN(FM_I2C3_SDA),
	LIGHT_PINCTRL_PIN(FM_GPIO2_13),
	LIGHT_PINCTRL_PIN(FM_SPI_SCLK),
	LIGHT_PINCTRL_PIN(FM_SPI_CSN),
	LIGHT_PINCTRL_PIN(FM_SPI_MOSI),
	LIGHT_PINCTRL_PIN(FM_SPI_MISO),
	LIGHT_PINCTRL_PIN(FM_GPIO2_18),
	LIGHT_PINCTRL_PIN(FM_GPIO2_19),
	LIGHT_PINCTRL_PIN(FM_GPIO2_20),
	LIGHT_PINCTRL_PIN(FM_GPIO2_21),
	LIGHT_PINCTRL_PIN(FM_GPIO2_22),
	LIGHT_PINCTRL_PIN(FM_GPIO2_23),
	LIGHT_PINCTRL_PIN(FM_GPIO2_24),
	LIGHT_PINCTRL_PIN(FM_GPIO2_25),
	LIGHT_PINCTRL_PIN(FM_SDIO0_WPRTN),
	LIGHT_PINCTRL_PIN(FM_SDIO0_DETN),
	LIGHT_PINCTRL_PIN(FM_SDIO1_WPRTN),
	LIGHT_PINCTRL_PIN(FM_SDIO1_DETN),
	LIGHT_PINCTRL_PIN(FM_GPIO2_30),
	LIGHT_PINCTRL_PIN(FM_GPIO2_31),
	LIGHT_PINCTRL_PIN(FM_GPIO3_0),
	LIGHT_PINCTRL_PIN(FM_GPIO3_1),
	LIGHT_PINCTRL_PIN(FM_GPIO3_2),
	LIGHT_PINCTRL_PIN(FM_GPIO3_3),
	LIGHT_PINCTRL_PIN(FM_HDMI_SCL),
	LIGHT_PINCTRL_PIN(FM_HDMI_SDA),
	LIGHT_PINCTRL_PIN(FM_HDMI_CEC),
	LIGHT_PINCTRL_PIN(FM_GMAC0_TX_CLK),
	LIGHT_PINCTRL_PIN(FM_GMAC0_RX_CLK),
	LIGHT_PINCTRL_PIN(FM_GMAC0_TXEN),
	LIGHT_PINCTRL_PIN(FM_GMAC0_TXD0),
	LIGHT_PINCTRL_PIN(FM_GMAC0_TXD1),
	LIGHT_PINCTRL_PIN(FM_GMAC0_TXD2),
	LIGHT_PINCTRL_PIN(FM_GMAC0_TXD3),
	LIGHT_PINCTRL_PIN(FM_GMAC0_RXDV),
	LIGHT_PINCTRL_PIN(FM_GMAC0_RXD0),
	LIGHT_PINCTRL_PIN(FM_GMAC0_RXD1),
	LIGHT_PINCTRL_PIN(FM_GMAC0_RXD2),
	LIGHT_PINCTRL_PIN(FM_GMAC0_RXD3),
	LIGHT_PINCTRL_PIN(FM_GMAC0_MDC),
	LIGHT_PINCTRL_PIN(FM_GMAC0_MDIO),
	LIGHT_PINCTRL_PIN(FM_GMAC0_COL),
	LIGHT_PINCTRL_PIN(FM_GMAC0_CRS),
};

static const struct pinctrl_pin_desc light_fm_left_pinctrl_pads[] = {
	LIGHT_PINCTRL_PIN(FM_QSPI1_SCLK),
	LIGHT_PINCTRL_PIN(FM_QSPI1_CSN0),
	LIGHT_PINCTRL_PIN(FM_QSPI1_D0_MOSI),
	LIGHT_PINCTRL_PIN(FM_QSPI1_D1_MISO),
	LIGHT_PINCTRL_PIN(FM_QSPI1_D2_WP ),
	LIGHT_PINCTRL_PIN(FM_QSPI1_D3_HOLD),
	LIGHT_PINCTRL_PIN(FM_I2C0_SCL),
	LIGHT_PINCTRL_PIN(FM_I2C0_SDA),
	LIGHT_PINCTRL_PIN(FM_I2C1_SCL),
	LIGHT_PINCTRL_PIN(FM_I2C1_SDA),
	LIGHT_PINCTRL_PIN(FM_UART1_TXD),
	LIGHT_PINCTRL_PIN(FM_UART1_RXD),
	LIGHT_PINCTRL_PIN(FM_UART4_TXD),
	LIGHT_PINCTRL_PIN(FM_UART4_RXD),
	LIGHT_PINCTRL_PIN(FM_UART4_CTSN),
	LIGHT_PINCTRL_PIN(FM_UART4_RTSN),
	LIGHT_PINCTRL_PIN(FM_UART3_TXD),
	LIGHT_PINCTRL_PIN(FM_UART3_RXD),
	LIGHT_PINCTRL_PIN(FM_GPIO0_18),
	LIGHT_PINCTRL_PIN(FM_GPIO0_19),
	LIGHT_PINCTRL_PIN(FM_GPIO0_20),
	LIGHT_PINCTRL_PIN(FM_GPIO0_21),
	LIGHT_PINCTRL_PIN(FM_GPIO0_22),
	LIGHT_PINCTRL_PIN(FM_GPIO0_23),
	LIGHT_PINCTRL_PIN(FM_GPIO0_24),
	LIGHT_PINCTRL_PIN(FM_GPIO0_25),
	LIGHT_PINCTRL_PIN(FM_GPIO0_26),
	LIGHT_PINCTRL_PIN(FM_GPIO0_27),
	LIGHT_PINCTRL_PIN(FM_GPIO0_28),
	LIGHT_PINCTRL_PIN(FM_GPIO0_29),
	LIGHT_PINCTRL_PIN(FM_GPIO0_30),
	LIGHT_PINCTRL_PIN(FM_GPIO0_31),
	LIGHT_PINCTRL_PIN(FM_GPIO1_0),
	LIGHT_PINCTRL_PIN(FM_GPIO1_1),
	LIGHT_PINCTRL_PIN(FM_GPIO1_2),
	LIGHT_PINCTRL_PIN(FM_GPIO1_3),
	LIGHT_PINCTRL_PIN(FM_GPIO1_4),
	LIGHT_PINCTRL_PIN(FM_GPIO1_5),
	LIGHT_PINCTRL_PIN(FM_GPIO1_6),
	LIGHT_PINCTRL_PIN(FM_GPIO1_7),
	LIGHT_PINCTRL_PIN(FM_GPIO1_8),
	LIGHT_PINCTRL_PIN(FM_GPIO1_9),
	LIGHT_PINCTRL_PIN(FM_GPIO1_10),
	LIGHT_PINCTRL_PIN(FM_GPIO1_11),
	LIGHT_PINCTRL_PIN(FM_GPIO1_12),
	LIGHT_PINCTRL_PIN(FM_GPIO1_13),
	LIGHT_PINCTRL_PIN(FM_GPIO1_14),
	LIGHT_PINCTRL_PIN(FM_GPIO1_15),
	LIGHT_PINCTRL_PIN(FM_GPIO1_16),
	LIGHT_PINCTRL_PIN(FM_CLK_OUT_0),
	LIGHT_PINCTRL_PIN(FM_CLK_OUT_1),
	LIGHT_PINCTRL_PIN(FM_CLK_OUT_2),
	LIGHT_PINCTRL_PIN(FM_CLK_OUT_3),
	LIGHT_PINCTRL_PIN(FM_GPIO1_21),
	LIGHT_PINCTRL_PIN(FM_GPIO1_22),
	LIGHT_PINCTRL_PIN(FM_GPIO1_23),
	LIGHT_PINCTRL_PIN(FM_GPIO1_24),
	LIGHT_PINCTRL_PIN(FM_GPIO1_25),
	LIGHT_PINCTRL_PIN(FM_GPIO1_26),
	LIGHT_PINCTRL_PIN(FM_GPIO1_27),
	LIGHT_PINCTRL_PIN(FM_GPIO1_28),
	LIGHT_PINCTRL_PIN(FM_GPIO1_29),
	LIGHT_PINCTRL_PIN(FM_GPIO1_30),
};

static const struct pinctrl_pin_desc light_fm_aon_pinctrl_pads[] = {
	LIGHT_PINCTRL_PIN(FM_OSC_CLK_IN),
	LIGHT_PINCTRL_PIN(FM_RTC_CLK_IN),
	LIGHT_PINCTRL_PIN(FM_DEBUG_MODE),
	LIGHT_PINCTRL_PIN(FM_I2C_AON_SCL),
	LIGHT_PINCTRL_PIN(FM_I2C_AON_SDA),
	LIGHT_PINCTRL_PIN(FM_CPU_JTG_TCLK),
	LIGHT_PINCTRL_PIN(FM_CPU_JTG_TMS),
	LIGHT_PINCTRL_PIN(FM_CPU_JTG_TDI),
	LIGHT_PINCTRL_PIN(FM_CPU_JTG_TDO),
	LIGHT_PINCTRL_PIN(FM_CPU_JTG_TRST),
	LIGHT_PINCTRL_PIN(FM_AOGPIO_7),
	LIGHT_PINCTRL_PIN(FM_AOGPIO_8),
	LIGHT_PINCTRL_PIN(FM_AOGPIO_9),
	LIGHT_PINCTRL_PIN(FM_AOGPIO_10),
	LIGHT_PINCTRL_PIN(FM_AOGPIO_11),
	LIGHT_PINCTRL_PIN(FM_AOGPIO_12),
	LIGHT_PINCTRL_PIN(FM_AOGPIO_13),
	LIGHT_PINCTRL_PIN(FM_AOGPIO_14),
	LIGHT_PINCTRL_PIN(FM_AOGPIO_15),
	LIGHT_PINCTRL_PIN(FM_AUDIO_PA0),
	LIGHT_PINCTRL_PIN(FM_AUDIO_PA1),
	LIGHT_PINCTRL_PIN(FM_AUDIO_PA2),
	LIGHT_PINCTRL_PIN(FM_AUDIO_PA3),
	LIGHT_PINCTRL_PIN(FM_AUDIO_PA4),
	LIGHT_PINCTRL_PIN(FM_AUDIO_PA5),
	LIGHT_PINCTRL_PIN(FM_AUDIO_PA6),
	LIGHT_PINCTRL_PIN(FM_AUDIO_PA7),
	LIGHT_PINCTRL_PIN(FM_AUDIO_PA8),
	LIGHT_PINCTRL_PIN(FM_AUDIO_PA9),
	LIGHT_PINCTRL_PIN(FM_AUDIO_PA10),
	LIGHT_PINCTRL_PIN(FM_AUDIO_PA11),
	LIGHT_PINCTRL_PIN(FM_AUDIO_PA12),
	LIGHT_PINCTRL_PIN(FM_AUDIO_PA13),
	LIGHT_PINCTRL_PIN(FM_AUDIO_PA14),
	LIGHT_PINCTRL_PIN(FM_AUDIO_PA15),
	LIGHT_PINCTRL_PIN(FM_AUDIO_PA16),
	LIGHT_PINCTRL_PIN(FM_AUDIO_PA17),
	LIGHT_PINCTRL_PIN(FM_AUDIO_PA27),
	LIGHT_PINCTRL_PIN(FM_AUDIO_PA28),
	LIGHT_PINCTRL_PIN(FM_AUDIO_PA29),
	LIGHT_PINCTRL_PIN(FM_AUDIO_PA30),
};

static int light_pinctrl_mpw_convert_pin_off(const unsigned int pin_id)
{
	unsigned int cov_pin_id = pin_id;

	if (cov_pin_id >= RGM_PAD_MS_RTE)
		cov_pin_id -= 9;

	return cov_pin_id;
}

static int light_pinctrl_fm_aon_covert_pin_off(const unsigned int pin_id)
{
	int cov_pin_id = pin_id;

	/* no need to configure mux according to spec */
	if ((pin_id >= 0 && pin_id <=6) || pin_id == 8)
		cov_pin_id = -1;

	return cov_pin_id;
}

static struct light_pinctrl_soc_info light_mpw_pinctrl_info = {
	.pins = light_mpw_pinctrl_pads,
	.npins = ARRAY_SIZE(light_mpw_pinctrl_pads),
	.cfg_off = 0x4c,
	.mux_off = 0x404,
	.covert_pin_off = light_pinctrl_mpw_convert_pin_off,
};

static struct light_pinctrl_soc_info light_fm_right_pinctrl_info = {
	.pins = light_fm_right_pinctrl_pads,
	.npins = ARRAY_SIZE(light_fm_right_pinctrl_pads),
	.cfg_off = 0x0,
	.mux_off = 0x400,
};

static struct light_pinctrl_soc_info light_fm_left_pinctrl_info = {
	.pins = light_fm_left_pinctrl_pads,
	.npins = ARRAY_SIZE(light_fm_left_pinctrl_pads),
	.cfg_off = 0x0,
	.mux_off = 0x400,
};

static struct light_pinctrl_soc_info light_fm_aon_pinctrl_info = {
	.pins = light_fm_aon_pinctrl_pads,
	.npins = ARRAY_SIZE(light_fm_aon_pinctrl_pads),
	.cfg_off = 0x0,
	.mux_off = 0x400,
	.covert_pin_off = light_pinctrl_fm_aon_covert_pin_off,
};

static const struct of_device_id light_pinctrl_of_match[] = {
	{ .compatible = "thead,light-mpw-pinctrl", .data = &light_mpw_pinctrl_info},
	{ .compatible = "thead,light-fm-right-pinctrl",	.data = &light_fm_right_pinctrl_info},
	{ .compatible = "thead,light-fm-left-pinctrl",	.data = &light_fm_left_pinctrl_info},
	{ .compatible = "thead,light-fm-aon-pinctrl",	.data = &light_fm_aon_pinctrl_info},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, light_pinctrl_of_match);

static struct platform_driver light_pinctrl_driver = {
	.driver = {
		.name = "light-pinctrl",
		.owner = THIS_MODULE,
		.of_match_table = light_pinctrl_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = light_pinctrl_probe,
};

module_platform_driver(light_pinctrl_driver);

MODULE_AUTHOR("fugang.duan <duanfugang.dfg@linux.alibaba.com>");
MODULE_DESCRIPTION("Thead light pinctrl driver");
MODULE_LICENSE("GPL v2");
