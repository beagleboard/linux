/*
 * Core driver for the reset control subsystem
 *
 * Copyright (C) 2013 Pantelis Antoniou <panto@antoniou-consulting.com>
 * Based on bits of rstctl, regulator core, gpio core and clk core
 *
 * License terms: GNU General Public License (GPL) version 2
 */
#define pr_fmt(fmt) "rstctl core: " fmt

#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/sysfs.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include <linux/rstctl.h>

static inline int reset_request(struct rstctl_dev *rdev,
			const struct rstctl_line *line)
{
	/* no request op? */
	if (rdev->rdesc->ops->request == NULL)
		return 0;

	return rdev->rdesc->ops->request(rdev, line);
}

static inline int reset_release(struct rstctl_dev *rdev,
			const struct rstctl_line *line)
{
	/* no release op? */
	if (rdev->rdesc->ops->release == NULL)
		return 0;

	return rdev->rdesc->ops->release(rdev, line);
}

static inline int reset_assert(struct rstctl_dev *rdev,
			const struct rstctl_line *line)
{
	return rdev->rdesc->ops->assert(rdev, line);
}

static inline int reset_deassert(struct rstctl_dev *rdev,
			const struct rstctl_line *line)
{
	return rdev->rdesc->ops->deassert(rdev, line);
}

static inline int reset_pulse(struct rstctl_dev *rdev,
			const struct rstctl_line *line,
			unsigned long hold_ns)
{
	return rdev->rdesc->ops->pulse(rdev, line, hold_ns);
}

int rstctl_assert(struct rstctl *rctrl)
{
	if (IS_ERR_OR_NULL(rctrl))
		return -EINVAL;
	return reset_assert(rctrl->rdev, rctrl->line);
}
EXPORT_SYMBOL(rstctl_assert);

int rstctl_deassert(struct rstctl *rctrl)
{
	if (IS_ERR_OR_NULL(rctrl))
		return -EINVAL;
	return reset_deassert(rctrl->rdev, rctrl->line);
}
EXPORT_SYMBOL(rstctl_deassert);

int rstctl_pulse(struct rstctl *rctrl, unsigned long hold_ns)
{
	if (IS_ERR_OR_NULL(rctrl))
		return -EINVAL;
	return reset_pulse(rctrl->rdev, rctrl->line, hold_ns);
}
EXPORT_SYMBOL(rstctl_pulse);

/* Mutex taken by all entry points */
static DEFINE_MUTEX(rstctl_lock);

/* Global list of reset control devices (struct rstctl_dev) */
static LIST_HEAD(rstctl_dev_list);

static struct rstctl *rstctl_request_line(struct rstctl_dev *rdev,
		struct device *dev, const struct rstctl_line *line)
{
	struct rstctl *rctrl;
	int index, err;

	index = line - rdev->rdesc->lines;

	/* find if it's already requested */
	list_for_each_entry(rctrl, &rdev->handles, node) {
		/* retreive index from the point */
		if (rctrl->line == line) {
			dev_err(dev, "Reset %s:%d already requested\n",
					rdev->rdesc->name, index);
			return ERR_PTR(-EEXIST);
		}
	}

	/* request from the driver */
	err = reset_request(rdev, line);
	if (err != 0) {
		dev_err(dev, "reset_request on %s:%d failed\n",
				rdev->rdesc->name, index);
		return ERR_PTR(err);
	}

	/* allocate */
	rctrl = kzalloc(sizeof(*rctrl), GFP_KERNEL);
	if (rctrl == NULL) {
		dev_err(dev, "Out of memory on %s:%d request\n",
				rdev->rdesc->name, index);
		reset_release(rdev, line);
		return ERR_PTR(-ENOMEM);
	}

	INIT_LIST_HEAD(&rctrl->node);
	rctrl->rdev = rdev;
	rctrl->dev = dev;
	rctrl->line = line;

	/* add it to the handle list */
	list_add_tail(&rctrl->node, &rdev->handles);

	return rctrl;
}

#ifdef CONFIG_OF

static struct rstctl_dev *of_node_to_rstctl_dev(struct device_node *np)
{
	struct rstctl_dev *rdev;

	list_for_each_entry(rdev, &rstctl_dev_list, node) {
		if (rdev->dev && rdev->dev->of_node == np)
			return rdev;
	}

	return ERR_PTR(-EPROBE_DEFER);
}

static struct rstctl *of_rstctl_get(struct device *dev, const char *id)
{
	struct device_node *np;
	struct rstctl *rctrl;
	struct rstctl_dev *rdev;
	const struct rstctl_line *line;
	struct of_phandle_args args;
	int index, lineidx, err;

	/* sanity check */
	if (dev == NULL || dev->of_node == NULL)
		return NULL;
	np = dev->of_node;

	index = 0;
	if (id != NULL) {
		err = of_property_match_string(np, "reset-names", id);
		if (err < 0) {
			dev_err(dev, "of_property_match of 'reset-names' failed\n");
			rctrl = ERR_PTR(err);
			goto out;
		}
		index = err;
	}

	err = of_parse_phandle_with_args(np, "reset", "#reset-cells", index,
			&args);
	if (err != 0) {
		dev_err(dev, "of_parse_phandle_with_args of 'reset' failed\n");
		rctrl = ERR_PTR(err);
		goto out;
	}

	mutex_lock(&rstctl_lock);

	rdev = of_node_to_rstctl_dev(args.np);
	if (IS_ERR_OR_NULL(rdev)) {
		dev_err(dev, "rstctl node not found\n");
		rctrl = rdev == NULL ? ERR_PTR(-EINVAL) : (void *)rdev;
		goto out_unlock;
	}

	if (args.args_count != 2) {
		dev_err(dev, "#reset-cells not %d\n", 2);
		rctrl = ERR_PTR(-EINVAL);
		goto out_unlock;
	}

	lineidx = args.args[0];
	/* make sure it's one we handle */
	if (lineidx < 0 || lineidx >= rdev->rdesc->nlines) {
		dev_err(dev, "Illegal reset #%d\n", lineidx);
		rctrl = ERR_PTR(-EINVAL);
		goto out_unlock;
	}
	line = &rdev->rdesc->lines[lineidx];

	rctrl = rstctl_request_line(rdev, dev, line);
	if (IS_ERR(rctrl)) {
		dev_err(dev, "rstctl_request_line failed\n");
		goto out_unlock;
	}

	/* put the label in */
	err = of_property_read_string_index(np, "reset-names", index,
				&rctrl->label);
	if (err != 0)
		rctrl->label = np->name;

out_unlock:
	mutex_unlock(&rstctl_lock);
out:
	return rctrl;
}
#else
static inline struct rstctl *of_rstctl_get(struct device *dev,
		const char *id)
{
	return NULL;
}
#endif

struct rstctl *rstctl_get(struct device *dev, const char *id)
{
	struct rstctl_dev *rdev;
	struct rstctl *rctrl;
	const struct rstctl_line *line;
	int i;

	/* DT case goes through here */
	if (IS_ENABLED(CONFIG_OF) && dev && dev->of_node)
		return of_rstctl_get(dev, id);

	/* non DT case does not support id == NULL */
	if (id == NULL) {
		dev_err(dev, "No reset ID supplied\n");
		return ERR_PTR(-EINVAL);
	}

	/*
	 * We search all the rstctl devices for a match
	 * with the given label id
	 */
	mutex_lock(&rstctl_lock);
	list_for_each_entry(rdev, &rstctl_dev_list, node) {
		for (i = 0; i < rdev->rdesc->nlines; i++) {
			line = &rdev->rdesc->lines[i];
			if (strcmp(line->name, id) == 0) {
				rctrl = rstctl_request_line(rdev, dev, line);
				goto out;
			}
		}
	}
	rctrl = ERR_PTR(-ENODEV);
out:
	mutex_unlock(&rstctl_lock);

	return rctrl;
}
EXPORT_SYMBOL(rstctl_get);

void rstctl_put(struct rstctl *rctrl)
{
	struct rstctl_dev *rdev;
	struct rstctl *rctrlt;

	/* safe */
	if (IS_ERR_OR_NULL(rctrl))
		return;
	rdev = rctrl->rdev;

	mutex_lock(&rstctl_lock);
	list_for_each_entry(rctrlt, &rdev->handles, node) {
		if (rctrlt == rctrl)
			goto found;
	}
	goto out;
found:
	/* release the reset */
	reset_release(rctrl->rdev, rctrl->line);
	list_del(&rctrl->node);
	kfree(rctrl);
out:
	mutex_unlock(&rstctl_lock);
}
EXPORT_SYMBOL(rstctl_put);

struct rstctl_dev *rstctl_register(struct device *dev,
		const struct rstctl_desc *rdesc)
{
	struct rstctl_dev *rdev;

	/* sanity check */
	if (dev == NULL || rdesc == NULL || rdesc->ops == NULL)
		return ERR_PTR(-EINVAL);

	/* those three must be defined */
	if (rdesc->ops->assert == NULL || rdesc->ops->deassert == NULL
			|| rdesc->ops->pulse == NULL)
		return ERR_PTR(-EINVAL);

	rdev = kzalloc(sizeof(*rdev), GFP_KERNEL);
	if (rdev == NULL) {
		dev_err(dev, "failed to alloc struct rstctl_dev\n");
		return ERR_PTR(-ENOMEM);
	}

	INIT_LIST_HEAD(&rdev->node);
	rdev->dev = dev;
	rdev->rdesc = rdesc;
	INIT_LIST_HEAD(&rdev->handles);

	mutex_lock(&rstctl_lock);
	list_add_tail(&rdev->node, &rstctl_dev_list);
	mutex_unlock(&rstctl_lock);

	return rdev;
}

int rstctl_unregister(struct rstctl_dev *rdev)
{
	int err;

	/* guard */
	if (IS_ERR_OR_NULL(rdev))
		return -EINVAL;

	err = 0;

	mutex_lock(&rstctl_lock);

	if (!list_empty(&rdev->handles)) {
		dev_err(rdev->dev, "%s still busy\n", rdev->rdesc->name);
		err = -EBUSY;
		goto out;
	}
	list_del(&rdev->node);

	/* free */
	kfree(rdev);

	mutex_unlock(&rstctl_lock);

out:
	mutex_unlock(&rstctl_lock);
	return err;
}

static int __init rstctl_init(void)
{
	pr_info("initialized rstctl subsystem\n");
	return 0;
}

/* init early, resetting is needed pretty early */
core_initcall(rstctl_init);
