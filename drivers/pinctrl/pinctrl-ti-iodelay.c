/*
 * Support for configuration of IO Delay module found on Texas Instruments SoCs
 * such as DRA7
 *
 * Copyright (C) 2015 Texas Instruments, Inc.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#define IODELAY_REG_NAME_LEN		((sizeof(u32) * 2) + 3)
#define DRIVER_NAME			"ti-io-delay"
/* Should I change this? Abuse? */
#define IODELAY_MUX_PINS_NAME		"pinctrl-single,pins"

/* Device tree match, populated later */
static const struct of_device_id ti_iodelay_of_match[];

/**
 * struct ti_iodelay_conf_vals - Description of each configuration parameters.
 * @offset:	Configuration register offset
 * @a_delay:	Agnostic Delay (in ps)
 * @g_delay:	Gnostic Delay (in ps)
 */
struct ti_iodelay_conf_vals {
	u16 offset;
	u16 a_delay;
	u16 g_delay;
};

/**
 * struct ti_iodelay_reg_data - Describes the registers for the IOdelay instance
 * @signature_mask:	Conf reg- mask for the signature bits
 * @signature_value:	Conf reg- signature value to be written (see TRM)
 * @lock_mask:		Conf reg- mask for the lock bits
 * @lock_val:		Conf reg- lock value for the lock bits (see TRM)
 * @unlock_val:		Conf reg- unlock value for the lock bits (see TRM)
 * @binary_data_coarse_mask: Conf reg- coarse mask (see TRM)
 * @binary_data_fine_mask: Conf reg- fine mask (see TRM)
 * @reg_refclk_offset:	Refclk register offset
 * @refclk_period_mask:	Refclk mask
 * @reg_coarse_offset:	Coarse register configuration offset
 * @coarse_delay_count_mask: Coarse delay count mask
 * @coarse_ref_count_mask: Coarse ref count mask
 * @reg_fine_offset:	Fine register configuration offset
 * @fine_delay_count_mask: Fine delay count mask
 * @fine_ref_count_mask: Fine  ref count mask
 * @reg_global_lock_offset: Global(for the IOdelay module) lock register offset
 * @global_lock_mask:	Lock mask
 * @global_unlock_val:	unlock value
 * @global_lock_val:	lock value
 * @reg_start_offset:	Where does the configuration registers start?
 * @regmap_config:	Regmap configuration for the IODelay region
 */
struct ti_iodelay_reg_data {
	u32 signature_mask;
	u32 signature_value;
	u32 lock_mask;
	u32 lock_val;
	u32 unlock_val;
	u32 binary_data_coarse_mask;
	u32 binary_data_fine_mask;

	u32 reg_refclk_offset;
	u32 refclk_period_mask;

	u32 reg_coarse_offset;
	u32 coarse_delay_count_mask;
	u32 coarse_ref_count_mask;

	u32 reg_fine_offset;
	u32 fine_delay_count_mask;
	u32 fine_ref_count_mask;

	u32 reg_global_lock_offset;
	u32 global_lock_mask;
	u32 global_unlock_val;
	u32 global_lock_val;

	u32 reg_start_offset;

	struct regmap_config *regmap_config;
};

/**
 * struct ti_iodelay_reg_values - Computed io_reg configuration values (see TRM)
 * @coarse_ref_count:	Coarse reference count
 * @coarse_delay_count:	Coarse delay count
 * @fine_ref_count:	Fine reference count
 * @fine_delay_count:	Fine Delay count
 * @ref_clk_period:	Reference Clock period
 * @cdpe:		Coarse delay parameter
 * @fdpe:		Fine delay parameter
 */
struct ti_iodelay_reg_values {
	u16 coarse_ref_count;
	u16 coarse_delay_count;

	u16 fine_ref_count;
	u16 fine_delay_count;

	u16 ref_clk_period;

	u32 cdpe;
	u32 fdpe;
};

/**
 * struct ti_iodelay_pin_name - name of the pins
 * @name: name
 */
struct ti_iodelay_pin_name {
	char name[IODELAY_REG_NAME_LEN];
};

/**
 * struct ti_iodelay_pingroup - Structure that describes one group
 * @np:		Node pointer (device tree)
 * @name:	Name of the group
 * @map:	pinctrl map allocated for the group
 * @vals:	configuration values allocated for the group (from dt)
 * @nvals:	number of configuration values allocated
 * @config:	pinconf "Config" - currently a dummy value
 * @node:	list node to next group
 */
struct ti_iodelay_pingroup {
	struct device_node *np;
	const char *name;
	struct pinctrl_map *map;
	struct ti_iodelay_conf_vals *vals;
	int nvals;
	unsigned long config;
	struct list_head node;
};

/**
 * struct ti_iodelay_device - Represents information for a IOdelay instance
 * @dev: device pointer
 * @reg_base: Remapped virtual address
 * @regmap: Regmap for this IOdelay instance
 * @pctl: Pinctrl device
 * @desc: pinctrl descriptor for pctl
 * @pa: pinctrl pin wise description
 * @names: names of the pins
 * @groups: list of pinconf groups for iodelay instance
 * @ngroups: number of groups in the list
 * @mutex: mutex to protect group list modification
 * @reg_data: Register definition data for the IODelay instance
 * @reg_init_conf_values: Initial configuration values.
 */
struct ti_iodelay_device {
	struct device *dev;
	void __iomem *reg_base;
	struct regmap *regmap;

	struct pinctrl_dev *pctl;
	struct pinctrl_desc desc;
	struct pinctrl_pin_desc *pa;
	struct ti_iodelay_pin_name *names;

	struct list_head groups;
	int ngroups;
	struct mutex mutex; /* list protection */

	const struct ti_iodelay_reg_data *reg_data;
	struct ti_iodelay_reg_values reg_init_conf_values;
};

/*--- IOdelay configuration stuff ----*/

/**
 * ti_iodelay_extract() - extract bits for a field
 * @val:	register value
 * @mask:	Mask
 *
 * Return: extracted value which is appropriately shifted
 */
static inline u32 ti_iodelay_extract(u32 val, u32 mask)
{
	return (val & mask) >> __ffs(mask);
}

/**
 * ti_iodelay_compute_dpe() - Compute equation for delay parameter
 * @period:	Period to use
 * @ref:	Reference Count
 * @delay:	Delay count
 * @delay_m:	Delay multiplier
 *
 * Return: Computed delay parameter
 */
static inline u32 ti_iodelay_compute_dpe(u16 period, u16 ref, u16 delay,
					 u16 delay_m)
{
	u64 m, d;

	/* Handle overflow conditions */
	m = 10 * (u64)period * (u64)ref;
	d = 2 * (u64)delay * (u64)delay_m;

	/* Truncate result back to 32 bits */
	return div64_u64(m, d);
}

/**
 * ti_iodelay_pinconf_set() - Configure the pin configuration
 * @iod:	IODelay device
 * @val:	Configuration value
 *
 * Update the configuration register as per TRM and lockup once done.
 * *IMPORTANT NOTE* SoC TRM does recommend doing iodelay programmation only
 * while in Isolation. But, then, isolation also implies that every pin
 * on the SoC(including DDR) will be isolated out. The only benefit being
 * a glitchless configuration, However, the intent of this driver is purely
 * to support a "glitchy" configuration where applicable.
 *
 * Return: 0 in case of success, else appropriate error value
 */
static int ti_iodelay_pinconf_set(struct ti_iodelay_device *iod,
				  struct ti_iodelay_conf_vals *val)
{
	const struct ti_iodelay_reg_data *reg = iod->reg_data;
	struct ti_iodelay_reg_values *ival = &iod->reg_init_conf_values;
	struct device *dev = iod->dev;
	u32 g_delay_coarse, g_delay_fine;
	u32 a_delay_coarse, a_delay_fine;
	u32 c_elements, f_elements;
	u32 total_delay;
	u32 reg_mask, reg_val, tmp_val;
	int r;

	/* NOTE: Truncation is expected in all division below */
	g_delay_coarse = val->g_delay / 920;
	g_delay_fine = ((val->g_delay % 920) * 10) / 60;

	a_delay_coarse = val->a_delay / ival->cdpe;
	a_delay_fine = ((val->a_delay % ival->cdpe) * 10) / ival->fdpe;

	c_elements = g_delay_coarse + a_delay_coarse;
	f_elements = (g_delay_fine + a_delay_fine) / 10;

	if (f_elements > 22) {
		total_delay = c_elements * ival->cdpe + f_elements * ival->fdpe;
		c_elements = total_delay / ival->cdpe;
		f_elements = (total_delay % ival->cdpe) / ival->fdpe;
	}

	reg_mask = reg->signature_mask;
	reg_val = reg->signature_value << __ffs(reg->signature_mask);

	reg_mask |= reg->binary_data_coarse_mask;
	tmp_val = c_elements << __ffs(reg->binary_data_coarse_mask);
	if (tmp_val & ~reg->binary_data_coarse_mask) {
		dev_err(dev, "Masking overflow of coarse elements %08x\n",
			tmp_val);
		tmp_val &= reg->binary_data_coarse_mask;
	}
	reg_val |= tmp_val;

	reg_mask |= reg->binary_data_fine_mask;
	tmp_val = f_elements << __ffs(reg->binary_data_fine_mask);
	if (tmp_val & ~reg->binary_data_fine_mask) {
		dev_err(dev, "Masking overflow of fine elements %08x\n",
			tmp_val);
		tmp_val &= reg->binary_data_fine_mask;
	}
	reg_val |= tmp_val;

	/* Write with lock value - we DONOT want h/w updates */
	reg_mask |= reg->lock_mask;
	reg_val |= reg->lock_val << __ffs(reg->lock_mask);
	r = regmap_update_bits(iod->regmap, val->offset, reg_mask, reg_val);

	dev_dbg(dev, "Set reg 0x%x Delay(a=%d g=%d), Elements(C=%d F=%d)0x%x\n",
		val->offset, val->a_delay, val->g_delay, c_elements,
		f_elements, reg_val);

	return r;
}

/**
 * ti_iodelay_pinconf_init_dev() - Initialize IODelay device
 * @iod:	IODelay device
 *
 * Unlocks the IODelay region, computes the common parameters
 *
 * Return: 0 in case of success, else appropriate error value
 */
static int ti_iodelay_pinconf_init_dev(struct ti_iodelay_device *iod)
{
	const struct ti_iodelay_reg_data *reg = iod->reg_data;
	struct device *dev = iod->dev;
	struct ti_iodelay_reg_values *ival = &iod->reg_init_conf_values;
	u32 val;
	int r;

	/* unlock the IOdelay region */
	r = regmap_update_bits(iod->regmap, reg->reg_global_lock_offset,
			       reg->global_lock_mask, reg->global_unlock_val);
	if (r)
		return r;

	/* Read up Recalibration sequence done by bootloader */
	r = regmap_read(iod->regmap, reg->reg_refclk_offset, &val);
	if (r)
		return r;
	ival->ref_clk_period = ti_iodelay_extract(val, reg->refclk_period_mask);
	dev_dbg(dev, "refclk_period=0x%04x\n", ival->ref_clk_period);

	r = regmap_read(iod->regmap, reg->reg_coarse_offset, &val);
	if (r)
		return r;
	ival->coarse_ref_count =
	    ti_iodelay_extract(val, reg->coarse_ref_count_mask);
	ival->coarse_delay_count =
	    ti_iodelay_extract(val, reg->coarse_delay_count_mask);
	if (!ival->coarse_delay_count) {
		dev_err(dev, "Invalid Coarse delay count (0) (reg=0x%08x)\n",
			val);
		return -EINVAL;
	}
	ival->cdpe = ti_iodelay_compute_dpe(ival->ref_clk_period,
					    ival->coarse_ref_count,
					    ival->coarse_delay_count, 88);
	if (!ival->cdpe) {
		dev_err(dev, "Invalid cdpe computed params = %d %d %d\n",
			ival->ref_clk_period, ival->coarse_ref_count,
			ival->coarse_delay_count);
		return -EINVAL;
	}
	dev_dbg(iod->dev, "coarse: ref=0x%04x delay=0x%04x cdpe=0x%08x\n",
		ival->coarse_ref_count, ival->coarse_delay_count, ival->cdpe);

	r = regmap_read(iod->regmap, reg->reg_fine_offset, &val);
	if (r)
		return r;
	ival->fine_ref_count =
	    ti_iodelay_extract(val, reg->fine_ref_count_mask);
	ival->fine_delay_count =
	    ti_iodelay_extract(val, reg->fine_delay_count_mask);
	if (!ival->fine_delay_count) {
		dev_err(dev, "Invalid Fine delay count (0) (reg=0x%08x)\n",
			val);
		return -EINVAL;
	}
	ival->fdpe = ti_iodelay_compute_dpe(ival->ref_clk_period,
					    ival->fine_ref_count,
					    ival->fine_delay_count, 264);
	if (!ival->fdpe) {
		dev_err(dev, "Invalid fdpe(0) computed params = %d %d %d\n",
			ival->ref_clk_period, ival->fine_ref_count,
			ival->fine_delay_count);
		return -EINVAL;
	}
	dev_dbg(iod->dev, "fine: ref=0x%04x delay=0x%04x fdpe=0x%08x\n",
		ival->fine_ref_count, ival->fine_delay_count, ival->fdpe);

	return 0;
}

/**
 * ti_iodelay_pinconf_deinit_dev() - deinit the IOdelay device
 * @iod:	IODelay device
 *
 * Deinitialize the IODelay device (basically just lock the region back up.
 */
static void ti_iodelay_pinconf_deinit_dev(struct ti_iodelay_device *iod)
{
	const struct ti_iodelay_reg_data *reg = iod->reg_data;

	/* lock the IOdelay region back again */
	regmap_update_bits(iod->regmap, reg->reg_global_lock_offset,
			   reg->global_lock_mask, reg->global_lock_val);
}

/*--- Pinctrl/pinconf framework stuff ----*/

/**
 * ti_iodelay_get_group() - Find the group mapped by a group selector
 * @iod:	IODelay device
 * @gselector:	Group Selector
 *
 * Return: Corresponding group representing group selector in list of groups
 * managed in IOdelay device OR NULL if not found.
 */
static struct ti_iodelay_pingroup *ti_iodelay_get_group(struct ti_iodelay_device
							*iod,
							unsigned gselector)
{
	struct ti_iodelay_pingroup *group;
	int gid = 0;

	list_for_each_entry(group, &iod->groups, node) {
		if (gid == gselector)
			return group;
		gid++;
	}

	dev_err(iod->dev, "%s could not find pingroup %i\n", __func__,
		gselector);
	return NULL;
}

/**
 * ti_iodelay_dt_node_to_map() - Map a device tree node to appropriate group
 * @pctldev:	pinctrl device representing IODelay device
 * @np:		Node Pointer (device tree)
 * @map:	Pinctrl Map returned back to pinctrl framework
 * @num_maps:	Number of maps (1)
 *
 * Maps the device tree description into a group of configuration parameters
 * for IOdelay block entry.
 *
 * Return: 0 in case of success, else appropriate error value
 */
static int ti_iodelay_dt_node_to_map(struct pinctrl_dev *pctldev,
				     struct device_node *np,
				     struct pinctrl_map **map,
				     unsigned *num_maps)
{
	struct ti_iodelay_device *iod;
	struct device *dev;
	/* const char **pgnames; */
	int ret = 0;
	const __be32 *mux;
	struct ti_iodelay_conf_vals *vals;
	struct ti_iodelay_pingroup *group;
	int size, index, idx, rows;
	u32 offset, val;

	iod = pinctrl_dev_get_drvdata(pctldev);
	if (!iod)
		return -EINVAL;
	dev = iod->dev;

	*map = devm_kzalloc(dev, sizeof(**map), GFP_KERNEL);
	if (!*map)
		return -ENOMEM;
	*num_maps = 0;

	group = devm_kzalloc(dev, sizeof(*group), GFP_KERNEL);
	if (!group) {
		ret = -ENOMEM;
		goto free_map;
	}

	mux = of_get_property(np, IODELAY_MUX_PINS_NAME, &size);
	if ((!mux) || (size < sizeof(*mux) * 2)) {
		dev_err(dev, "bad data for mux %s\n", np->name);
		ret = -EINVAL;
		goto free_group;
	}

	size /= sizeof(*mux);	/* Number of elements in array */
	rows = size / 2;

	vals = devm_kzalloc(dev, sizeof(*vals) * rows, GFP_KERNEL);
	if (!vals) {
		ret = -ENOMEM;
		goto free_group;
	}

	index = 0;
	idx = 0;
	while (index < size) {
		offset = be32_to_cpup(mux + index++);
		val = be32_to_cpup(mux + index++);
		vals[idx].offset = offset;
		vals[idx].a_delay = val & 0xFFFF;
		vals[idx].g_delay = (val & 0xFFFF0000) >> 16;
		if (offset > iod->reg_data->regmap_config->max_register) {
			dev_err(dev, "Invalid offset for %s 0x%x\n",
				np->name, offset);
			break;
		}
		dev_dbg(dev, "%s offset=%x a_delay = %d g_delay = %d\n",
			np->name, vals[idx].offset, vals[idx].a_delay,
			vals[idx].g_delay);
		idx++;
	}

	group->name = np->name;
	group->np = np;
	group->vals = vals;
	group->nvals = idx;
	group->config = PIN_CONFIG_END;
	group->map = *map;

	/* Add to group list */
	mutex_lock(&iod->mutex);
	list_add_tail(&group->node, &iod->groups);
	iod->ngroups++;
	mutex_unlock(&iod->mutex);

	(*map)->type = PIN_MAP_TYPE_CONFIGS_GROUP;
	(*map)->data.configs.group_or_pin = np->name;
	(*map)->data.configs.configs = &group->config;
	(*map)->data.configs.num_configs = 1;
	*num_maps = 1;

	return 0;

free_group:
	devm_kfree(dev, group);
free_map:
	devm_kfree(dev, *map);
	return ret;
}

/**
 * ti_iodelay_dt_free_map() - Free map and resource alloted as per the map
 * @pctldev:	pinctrl device representing IODelay device
 * @map:	Map allocated by ti_iodelay_dt_node_to_map
 * @num_maps:	Num maps (1)
 *
 * Removes the group associated with the map and frees all resources allocated
 * for the group.
 */
static void ti_iodelay_dt_free_map(struct pinctrl_dev *pctldev,
				   struct pinctrl_map *map, unsigned num_maps)
{
	struct ti_iodelay_device *iod;
	struct device *dev;
	struct ti_iodelay_pingroup *group;
	bool found = false;

	if (!map)
		return;

	iod = pinctrl_dev_get_drvdata(pctldev);
	if (!iod)
		return;
	dev = iod->dev;

	mutex_lock(&iod->mutex);
	list_for_each_entry(group, &iod->groups, node) {
		if (group->map == map) {
			found = true;
			list_del(&group->node);
			iod->ngroups--;
			break;
		}
	}
	mutex_unlock(&iod->mutex);

	/* If some freaky pinconf framework bug... */
	if (!found)
		return;

	devm_kfree(dev, group->vals);
	devm_kfree(dev, group);
	devm_kfree(dev, map);
}

/**
 * ti_iodelay_pinctrl_get_groups_count() - Get number of groups registered
 * @pctldev:	pinctrl device representing IODelay device
 *
 * Return: number of groups mapped on the IODelay
 */
static int ti_iodelay_pinctrl_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct ti_iodelay_device *iod;
	struct device *dev;

	iod = pinctrl_dev_get_drvdata(pctldev);
	dev = iod->dev;

	return iod->ngroups;
}

/**
 * ti_iodelay_pinctrl_get_group_name() - Get the group name
 * @pctldev:	pinctrl device representing IODelay device
 * @gselector:	group selector
 *
 * Return: name of the Group given a valid gselector, else NULL.
 */
static const char *ti_iodelay_pinctrl_get_group_name(struct pinctrl_dev
						     *pctldev,
						     unsigned gselector)
{
	struct ti_iodelay_device *iod;
	struct device *dev;
	struct ti_iodelay_pingroup *group;

	iod = pinctrl_dev_get_drvdata(pctldev);
	dev = iod->dev;

	group = ti_iodelay_get_group(iod, gselector);
	if (!group)
		return NULL;

	return group->name;
}

/**
 * ti_iodelay_pinctrl_get_group_pins() - get group pins
 * @pctldev:	pinctrl device representing IODelay device
 * @gselector: Group selector
 * @pins:	pointer to the pins
 * @npins:	number of pins
 *
 * Dummy implementation since we do not track pins, we track configurations
 * Forced by pinctrl's pinctrl_check_ops()
 *
 * Return: -EINVAL
 */
static int ti_iodelay_pinctrl_get_group_pins(struct pinctrl_dev *pctldev,
					     unsigned gselector,
					     const unsigned **pins,
					     unsigned *npins)
{
	/* Dummy implementation - we dont do pin mux */
	return -EINVAL;
}

/**
 * ti_iodelay_pinconf_group_get() - Get the group configuration
 * @pctldev:	pinctrl device representing IODelay device
 * @gselector:	Group selector
 * @config:	configuration returned
 *
 * Return: The configuration if the group is valid, else returns -EINVAL
 */
static int ti_iodelay_pinconf_group_get(struct pinctrl_dev *pctldev,
					unsigned gselector,
					unsigned long *config)
{
	struct ti_iodelay_device *iod;
	struct device *dev;
	struct ti_iodelay_pingroup *group;

	iod = pinctrl_dev_get_drvdata(pctldev);
	dev = iod->dev;
	group = ti_iodelay_get_group(iod, gselector);

	if (!group)
		return -EINVAL;

	*config = group->config;
	return 0;
}

/**
 * ti_iodelay_pinconf_group_set() - Configure the groups of pins
 * @pctldev:	pinctrl device representing IODelay device
 * @gselector:	Group selector
 * @configs:	Configurations
 * @num_configs: Number of configurations
 *
 * Return: 0 if all went fine, else appropriate error value.
 */
static int ti_iodelay_pinconf_group_set(struct pinctrl_dev *pctldev,
					unsigned gselector,
					unsigned long *configs,
					unsigned num_configs)
{
	struct ti_iodelay_device *iod;
	struct device *dev;
	struct ti_iodelay_pingroup *group;
	int i;

	iod = pinctrl_dev_get_drvdata(pctldev);
	dev = iod->dev;
	group = ti_iodelay_get_group(iod, gselector);

	if (num_configs != 1) {
		dev_err(dev, "Unsupported number of configurations %d\n",
			num_configs);
		return -EINVAL;
	}

	if (*configs != PIN_CONFIG_END) {
		dev_err(dev, "Unsupported configuration\n");
		return -EINVAL;
	}

	for (i = 0; i < group->nvals; i++) {
		if (ti_iodelay_pinconf_set(iod, &group->vals[i]))
			return -ENOTSUPP;
	}

	return 0;
}

#ifdef CONFIG_DEBUG_FS
/**
 * ti_iodelay_pinconf_group_dbg_show() - show the group information
 * @pctldev:	Show the group information
 * @s:	Sequence file
 * @gselector:	group selector
 *
 * Provide the configuration information of the selected group
 */
static void ti_iodelay_pinconf_group_dbg_show(struct pinctrl_dev *pctldev,
					      struct seq_file *s,
					      unsigned gselector)
{
	struct ti_iodelay_device *iod;
	struct device *dev;
	struct ti_iodelay_pingroup *group;
	int i;

	iod = pinctrl_dev_get_drvdata(pctldev);
	dev = iod->dev;
	group = ti_iodelay_get_group(iod, gselector);
	if (!group)
		return;

	for (i = 0; i < group->nvals; i++) {
		struct ti_iodelay_conf_vals *val;
		u32 reg = 0;

		val = &group->vals[i];
		regmap_read(iod->regmap, val->offset, &reg),
		    seq_printf(s, "\n\t0x%08x = 0x%08x (%3d, %3d)",
			       val->offset, reg, val->a_delay, val->g_delay);
	}
}
#endif

static struct pinctrl_ops ti_iodelay_pinctrl_ops = {
	.dt_node_to_map = ti_iodelay_dt_node_to_map,
	.dt_free_map = ti_iodelay_dt_free_map,
	.get_groups_count = ti_iodelay_pinctrl_get_groups_count,
	.get_group_name = ti_iodelay_pinctrl_get_group_name,
	.get_group_pins = ti_iodelay_pinctrl_get_group_pins,
};

static struct pinconf_ops ti_iodelay_pinctrl_pinconf_ops = {
	.pin_config_group_get = ti_iodelay_pinconf_group_get,
	.pin_config_group_set = ti_iodelay_pinconf_group_set,
#ifdef CONFIG_DEBUG_FS
	.pin_config_group_dbg_show = ti_iodelay_pinconf_group_dbg_show,
#endif
};

/**
 * ti_iodelay_alloc_pins() - Allocate structures needed for pins for IOdelay
 * @dev:	device pointer
 * @iod:	IODelay device
 * @base_phy:	Base Physical Address
 *
 * Return: 0 if all went fine, else appropriate error value.
 */
static int ti_iodelay_alloc_pins(struct device *dev,
				 struct ti_iodelay_device *iod, u32 base_phy)
{
	const struct ti_iodelay_reg_data *r = iod->reg_data;
	struct pinctrl_pin_desc *pin;
	struct ti_iodelay_pin_name *pn;
	u32 phy_reg;
	int nr_pins, i;

	nr_pins = (r->regmap_config->max_register - r->reg_start_offset) / 4;

	dev_dbg(dev, "Allocating %i pins\n", nr_pins);

	iod->pa = devm_kzalloc(dev, sizeof(*iod->pa) * nr_pins, GFP_KERNEL);
	if (!iod->pa)
		return -ENOMEM;

	iod->names =
	    devm_kzalloc(dev, sizeof(struct ti_iodelay_pin_name) * nr_pins,
			 GFP_KERNEL);
	if (!iod->names)
		return -ENOMEM;

	iod->desc.pins = iod->pa;
	iod->desc.npins = nr_pins;

	phy_reg = r->reg_start_offset + base_phy;
	pn = iod->names;
	for (i = 0; i < nr_pins; i++, pn++, phy_reg += 4) {
		pin = &iod->pa[i];
		sprintf(pn->name, "%x.%d", phy_reg, i);
		pin->number = i;
		pin->name = pn->name;
	}

	return 0;
}

/**
 * ti_iodelay_probe() - Standard probe
 * @pdev:	platform device
 *
 * Return: 0 if all went fine, else appropriate error value.
 */
static int ti_iodelay_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = of_node_get(dev->of_node);
	const struct of_device_id *match;
	struct resource *res;
	struct ti_iodelay_device *iod;
	int ret = 0;

	if (!np) {
		ret = -EINVAL;
		dev_err(dev, "No OF node\n");
		goto exit_out;
	}

	match = of_match_device(ti_iodelay_of_match, dev);
	if (!match) {
		ret = -EINVAL;
		dev_err(dev, "No DATA match\n");
		goto exit_out;
	}

	iod = devm_kzalloc(dev, sizeof(*iod), GFP_KERNEL);
	if (!iod) {
		ret = -ENOMEM;
		goto exit_out;
	}
	iod->dev = dev;
	iod->reg_data = match->data;

	/* So far We can assume there is only 1 bank of registers */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "Missing MEM resource\n");
		ret = -ENODEV;
		goto exit_out;
	}

	iod->reg_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(iod->reg_base)) {
		ret = PTR_ERR(iod->reg_base);
		goto exit_out;
	}

	iod->regmap = devm_regmap_init_mmio(dev, iod->reg_base,
					    iod->reg_data->regmap_config);
	if (IS_ERR(iod->regmap)) {
		dev_err(dev, "Regmap MMIO init failed.\n");
		ret = PTR_ERR(iod->regmap);
		goto exit_out;
	}

	if (ti_iodelay_pinconf_init_dev(iod))
		goto exit_out;

	ret = ti_iodelay_alloc_pins(dev, iod, res->start);
	if (ret)
		goto exit_out;

	INIT_LIST_HEAD(&iod->groups);
	mutex_init(&iod->mutex);

	iod->desc.pctlops = &ti_iodelay_pinctrl_ops;
	/* no pinmux ops - we are pinconf */
	iod->desc.confops = &ti_iodelay_pinctrl_pinconf_ops;
	iod->desc.name = dev_name(dev);
	iod->desc.owner = THIS_MODULE;

	iod->pctl = pinctrl_register(&iod->desc, dev, iod);
	if (!iod->pctl) {
		dev_err(dev, "Failed to register pinctrl\n");
		ret = -ENODEV;
		goto exit_out;
	}

	platform_set_drvdata(pdev, iod);

exit_out:
	of_node_put(np);
	return ret;
}

/**
 * ti_iodelay_remove() - standard remove
 * @pdev:	platform device
 *
 * Return: 0 if all went fine, else appropriate error value.
 */
static int ti_iodelay_remove(struct platform_device *pdev)
{
	struct ti_iodelay_device *iod = platform_get_drvdata(pdev);

	if (!iod)
		return 0;
	if (iod->pctl)
		pinctrl_unregister(iod->pctl);

	ti_iodelay_pinconf_deinit_dev(iod);

	/* Expect other allocations to be freed by devm */

	return 0;
}

static struct regmap_config dra7_iodelay_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = 0xD1C,
};

static struct ti_iodelay_reg_data dra7_iodelay_data = {
	.signature_mask = 0x0003F000,
	.signature_value = 0x29,
	.lock_mask = 0x00000400,
	.lock_val = 1,
	.unlock_val = 0,
	.binary_data_coarse_mask = 0x000003E0,
	.binary_data_fine_mask = 0x0000001F,

	.reg_refclk_offset = 0x14,
	.refclk_period_mask = 0xFFFF,

	.reg_coarse_offset = 0x18,
	.coarse_delay_count_mask = 0xFFFF0000,
	.coarse_ref_count_mask = 0x0000FFFF,

	.reg_fine_offset = 0x1C,
	.fine_delay_count_mask = 0xFFFF0000,
	.fine_ref_count_mask = 0x0000FFFF,

	.reg_global_lock_offset = 0x2C,
	.global_lock_mask = 0x0000FFFF,
	.global_unlock_val = 0x0000AAAA,
	.global_lock_val = 0x0000AAAB,

	.reg_start_offset = 0x30,
	.regmap_config = &dra7_iodelay_regmap_config,
};

static const struct of_device_id ti_iodelay_of_match[] = {
	{.compatible = "ti,dra7-iodelay", .data = &dra7_iodelay_data},
	{ /* Hopefully no more.. */ },
};
MODULE_DEVICE_TABLE(of, ti_iodelay_of_match);

static struct platform_driver ti_iodelay_driver = {
	.probe = ti_iodelay_probe,
	.remove = ti_iodelay_remove,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = DRIVER_NAME,
		   .of_match_table = ti_iodelay_of_match,
		   },
};
module_platform_driver(ti_iodelay_driver);

MODULE_AUTHOR("Texas Instruments, Inc.");
MODULE_DESCRIPTION("Pinconf driver for TI's IO Delay module");
MODULE_LICENSE("GPL v2");
