/*
 * Functions for working with device tree overlays
 *
 * Copyright (C) 2012 Pantelis Antoniou <panto@antoniou-consulting.com>
 * Copyright (C) 2012 Texas Instruments Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_i2c.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/err.h>

/**
 * Apply a single overlay node recursively.
 *
 * Property or node names that start with '-' signal that
 * the property/node is to be removed.
 *
 * All the property notifiers are appropriately called.
 * Note that the in case of an error the target node is left
 * in a inconsistent state. Error recovery should be performed
 * by recording the modification using the of notifiers.
 */
static int of_overlay_apply_one(struct device_node *target,
		const struct device_node *overlay)
{
	const char *pname, *cname;
	struct device_node *child, *tchild;
	struct property *prop, *propn, *tprop;
	int remove;
	char *full_name;
	const char *suffix;
	int ret;

	/* sanity checks */
	if (target == NULL || overlay == NULL)
		return -EINVAL;

	for_each_property_of_node(overlay, prop) {

		/* don't touch, 'name' */
		if (of_prop_cmp(prop->name, "name") == 0)
			continue;

		/* default is add */
		remove = 0;
		pname = prop->name;
		if (*pname == '-') {	/* skip, - notes removal */
			pname++;
			remove = 1;
			propn = NULL;
		} else {
			propn = __of_copy_property(prop,
					GFP_KERNEL);
			if (propn == NULL)
				return -ENOMEM;
		}

		tprop = of_find_property(target, pname, NULL);

		/* found? */
		if (tprop != NULL) {
			if (propn != NULL)
				ret = of_update_property(target, propn);
			else
				ret = of_remove_property(target, tprop);
		} else {
			if (propn != NULL)
				ret = of_add_property(target, propn);
			else
				ret = 0;
		}
		if (ret != 0)
			return ret;
	}

	__for_each_child_of_node(overlay, child) {

		/* default is add */
		remove = 0;
		cname = child->name;
		if (*cname == '-') {	/* skip, - notes removal */
			cname++;
			remove = 1;
		}

		/* special case for nodes with a suffix */
		suffix = strrchr(child->full_name, '@');
		if (suffix != NULL) {
			cname = kbasename(child->full_name);
			WARN_ON(cname == NULL);	/* sanity check */
			if (cname == NULL)
				continue;
			if (*cname == '-')
				cname++;
		}

		tchild = of_get_child_by_name(target, cname);
		if (tchild != NULL) {

			if (!remove) {

				/* apply overlay recursively */
				ret = of_overlay_apply_one(tchild, child);
				of_node_put(tchild);

				if (ret != 0)
					return ret;

			} else {

				ret = of_detach_node(tchild);
				of_node_put(tchild);
			}

		} else {

			if (!remove) {
				full_name = kasprintf(GFP_KERNEL, "%s/%s",
						target->full_name, cname);
				if (full_name == NULL)
					return -ENOMEM;

				/* create empty tree as a target */
				tchild = __of_create_empty_node(cname,
						child->type, full_name,
						child->phandle, GFP_KERNEL);

				/* free either way */
				kfree(full_name);

				if (tchild == NULL)
					return -ENOMEM;

				/* point to parent */
				tchild->parent = target;

				ret = of_attach_node(tchild);
				if (ret != 0)
					return ret;

				/* apply the overlay */
				ret = of_overlay_apply_one(tchild, child);
				if (ret != 0) {
					__of_free_tree(tchild);
					return ret;
				}
			}
		}
	}

	return 0;
}

/**
 * Lookup an overlay device entry
 */
struct of_overlay_device_entry *of_overlay_device_entry_lookup(
		struct of_overlay_info *ovinfo, struct device_node *node)
{
	struct of_overlay_device_entry *re;

	/* no need for locks, we're under the ovinfo->lock */
	list_for_each_entry(re, &ovinfo->de_list, node) {
		if (re->np == node)
			return re;
	}
	return NULL;
}

/**
 * Add an overlay log entry
 */
static int of_overlay_log_entry_entry_add(struct of_overlay_info *ovinfo,
		unsigned long action, struct device_node *dn,
		struct property *prop)
{
	struct of_overlay_log_entry *le;

	/* check */
	if (ovinfo == NULL || dn == NULL)
		return -EINVAL;

	le = kzalloc(sizeof(*le), GFP_KERNEL);
	if (le == NULL) {
		pr_err("%s: Failed to allocate\n", __func__);
		return -ENOMEM;
	}

	/* get a reference to the node */
	le->action = action;
	le->np = of_node_get(dn);
	le->prop = prop;

	if (action == OF_RECONFIG_UPDATE_PROPERTY && prop)
		le->old_prop = of_find_property(dn, prop->name, NULL);

	list_add_tail(&le->node, &ovinfo->le_list);

	return 0;
}

/**
 * Add an overlay device entry
 */
static void of_overlay_device_entry_entry_add(struct of_overlay_info *ovinfo,
		struct device_node *node, struct platform_device *pdev,
		int state)
{
	struct of_overlay_device_entry *re;
	int fresh;

	/* check */
	if (ovinfo == NULL)
		return;

	fresh = 0;
	re = of_overlay_device_entry_lookup(ovinfo, node);
	if (re == NULL) {
		re = kzalloc(sizeof(*re), GFP_KERNEL);
		if (re == NULL) {
			pr_err("%s: Failed to allocate\n", __func__);
			return;
		}
		fresh = 1;
	}

	if (re->np == NULL)
		re->np = of_node_get(node);
	if (re->pdev == NULL)
		re->pdev = of_dev_get(pdev);
	re->state = state;

	if (fresh)
		list_add_tail(&re->node, &ovinfo->de_list);
}

/**
 * Overlay OF notifier
 *
 * Called every time there's a property/node modification
 * Every modification causes a log entry addition, while
 * any modification that causes a node's state to change
 * from/to disabled to/from enabled causes a device entry
 * addition.
 */
static int of_overlay_notify(struct notifier_block *nb,
				unsigned long action, void *arg)
{
	struct of_overlay_info *ovinfo;
	struct device_node *node;
	struct property *prop, *sprop, *cprop;
	struct of_prop_reconfig *pr;
	struct platform_device *pdev;
	int state;
	int err = 0;

	ovinfo = container_of(nb, struct of_overlay_info, notifier);

	/* prep vars */
	switch (action) {
	case OF_RECONFIG_ATTACH_NODE:
	case OF_RECONFIG_DETACH_NODE:
		node = arg;
		if (node == NULL)
			return notifier_from_errno(-EINVAL);
		prop = NULL;
		break;
	case OF_RECONFIG_ADD_PROPERTY:
	case OF_RECONFIG_REMOVE_PROPERTY:
	case OF_RECONFIG_UPDATE_PROPERTY:
		pr = arg;
		if (pr == NULL)
			return notifier_from_errno(-EINVAL);
		node = pr->dn;
		if (node == NULL)
			return notifier_from_errno(-EINVAL);
		prop = pr->prop;
		if (prop == NULL)
			return notifier_from_errno(-EINVAL);
		break;
	default:
		return notifier_from_errno(0);
	}

	/* add to the log */
	err = of_overlay_log_entry_entry_add(ovinfo, action, node, prop);
	if (err != 0)
		return notifier_from_errno(err);

	/* come up with the device entry (if any) */
	pdev = NULL;
	state = 0;

	/* determine the state the node will end up */
	switch (action) {
	case OF_RECONFIG_ATTACH_NODE:
		/* we demand that a compatible node is present */
		state = of_find_property(node, "compatible", NULL) &&
			of_device_is_available(node);
		break;
	case OF_RECONFIG_DETACH_NODE:
		state = 0;
		pdev = of_find_device_by_node(node);
		break;
	case OF_RECONFIG_ADD_PROPERTY:
	case OF_RECONFIG_REMOVE_PROPERTY:
	case OF_RECONFIG_UPDATE_PROPERTY:
		/* either one cause a change in state */
		if (strcmp(prop->name, "status") != 0 &&
				strcmp(prop->name, "compatible") != 0)
			return notifier_from_errno(0);

		if (strcmp(prop->name, "status") == 0) {
			/* status */
			cprop = of_find_property(node, "compatible", NULL);
			sprop = action != OF_RECONFIG_REMOVE_PROPERTY ?
				prop : NULL;
		} else {
			/* compatible */
			sprop = of_find_property(node, "status", NULL);
			cprop = action != OF_RECONFIG_REMOVE_PROPERTY ?
				prop : NULL;
		}

		state = cprop && cprop->length > 0 &&
			    (!sprop || (sprop->length > 0 &&
				(strcmp(sprop->value, "okay") == 0 ||
				 strcmp(sprop->value, "ok") == 0)));
		break;

	default:
		return notifier_from_errno(0);
	}

	if (state == 0)
		pdev = of_find_device_by_node(node);

	of_overlay_device_entry_entry_add(ovinfo, node, pdev, state);

	return notifier_from_errno(err);
}

/**
 * Prepare for the overlay, for now it just registers the
 * notifier.
 */
static int of_overlay_prep_one(struct of_overlay_info *ovinfo)
{
	int err;

	err = of_reconfig_notifier_register(&ovinfo->notifier);
	if (err != 0) {
		pr_err("%s: failed to register notifier for '%s'\n",
			__func__, ovinfo->target->full_name);
		return err;
	}
	return 0;
}

static int of_overlay_device_entry_change(struct of_overlay_info *ovinfo,
		struct of_overlay_device_entry *re, int revert)
{
	struct i2c_adapter *adap = NULL;
	struct i2c_client *client;
	struct platform_device *pdev, *parent_pdev = NULL;
	int state;

	state = !!re->state  ^ !!revert;

	if (re->np && re->np->parent) {
		pr_debug("%s: parent is %s\n",
				__func__, re->np->parent->full_name);
		adap = of_find_i2c_adapter_by_node(re->np->parent);
		if (adap == NULL)
			parent_pdev = of_find_device_by_node(re->np->parent);
	}

	if (state) {

		/* try to see if it's an I2C client node */
		if (adap == NULL) {

			pr_debug("%s: creating new platform device "
					"new_node='%s' %p\n",
					__func__, re->np->full_name, re->np);

			pdev = of_platform_device_create(re->np, NULL,
					parent_pdev ? &parent_pdev->dev : NULL);
			if (pdev == NULL) {
				pr_warn("%s: Failed to create platform device "
						"for '%s'\n",
						__func__, re->np->full_name);
			}
		} else {
			pr_debug("%s: creating new i2c_client device "
					"new_node='%s' %p\n",
					__func__, re->np->full_name, re->np);

			client = of_i2c_register_device(adap, re->np);

			if (client == NULL) {
				pr_warn("%s: Failed to create i2c client device "
						"for '%s'\n",
						__func__, re->np->full_name);
			}
		}

	} else {

		if (re->pdev) {
			pr_debug("%s: removing pdev %s\n", __func__,
					dev_name(&re->pdev->dev));
			platform_device_unregister(re->pdev);
		}
	}

	if (adap)
		put_device(&adap->dev);

	if (parent_pdev)
		of_dev_put(parent_pdev);

	return 0;
}

/**
 * Revert one overlay
 * Either due to an error, or due to normal overlay removal.
 * Using the log entries, we revert any change to the live tree.
 * In the same manner, using the device entries we enable/disable
 * the platform devices appropriately.
 */
static void of_overlay_revert_one(struct of_overlay_info *ovinfo)
{
	struct of_overlay_device_entry *re, *ren;
	struct of_overlay_log_entry *le, *len;
	struct property *prop, **propp;
	int ret;
	unsigned long flags;

	if (!ovinfo || !ovinfo->target || !ovinfo->overlay)
		return;

	pr_debug("%s: Reverting overlay on '%s'\n", __func__,
			ovinfo->target->full_name);

	/* overlay applied correctly, now create/destroy pdevs */
	list_for_each_entry_safe_reverse(re, ren, &ovinfo->de_list, node) {

		of_overlay_device_entry_change(ovinfo, re, 1);

		of_node_put(re->np);
		list_del(&re->node);
		kfree(re);
	}

	list_for_each_entry_safe_reverse(le, len, &ovinfo->le_list, node) {

		ret = 0;
		switch (le->action) {
		case OF_RECONFIG_ATTACH_NODE:
			pr_debug("Reverting ATTACH_NODE %s\n",
					le->np->full_name);
			ret = of_detach_node(le->np);
			break;

		case OF_RECONFIG_DETACH_NODE:
			pr_debug("Reverting DETACH_NODE %s\n",
					le->np->full_name);
			ret = of_attach_node(le->np);
			break;

		case OF_RECONFIG_ADD_PROPERTY:
			pr_debug("Reverting ADD_PROPERTY %s %s\n",
					le->np->full_name, le->prop->name);
			ret = of_remove_property(le->np, le->prop);
			break;

		case OF_RECONFIG_REMOVE_PROPERTY:
		case OF_RECONFIG_UPDATE_PROPERTY:

			pr_debug("Reverting %s_PROPERTY %s %s\n",
				le->action == OF_RECONFIG_REMOVE_PROPERTY ?
					"REMOVE" : "UPDATE",
					le->np->full_name, le->prop->name);

			/* property is possibly on deadprops (avoid alloc) */
			write_lock_irqsave(&devtree_lock, flags);
			prop = le->action == OF_RECONFIG_REMOVE_PROPERTY ?
				le->prop : le->old_prop;
			propp = &le->np->deadprops;
			while (*propp != NULL) {
				if (*propp == prop)
					break;
				propp = &(*propp)->next;
			}
			if (*propp != NULL) {
				/* remove it from deadprops */
				(*propp)->next = prop->next;
				write_unlock_irqrestore(&devtree_lock, flags);
			} else {
				write_unlock_irqrestore(&devtree_lock, flags);
				/* not found, just make a copy */
				prop = __of_copy_property(prop, GFP_KERNEL);
				if (prop == NULL) {
					pr_err("%s: Failed to copy property\n",
							__func__);
					break;
				}
			}

			if (le->action == OF_RECONFIG_REMOVE_PROPERTY)
				ret = of_add_property(le->np, prop);
			else
				ret = of_update_property(le->np, prop);
			break;

		default:
			/* nothing */
			break;
		}

		if (ret != 0)
			pr_err("%s: revert on node %s Failed!\n",
					__func__, le->np->full_name);

		of_node_put(le->np);

		list_del(&le->node);

		kfree(le);
	}
}

/**
 * Perform the post overlay work.
 *
 * We unregister the notifier, and in the case on an error we
 * revert the overlay.
 * If the overlay applied correctly, we iterare over the device entries
 * and create/destroy the platform devices appropriately.
 */
static int of_overlay_post_one(struct of_overlay_info *ovinfo, int err)
{
	struct of_overlay_device_entry *re;

	of_reconfig_notifier_unregister(&ovinfo->notifier);

	if (err != 0) {
		/* revert this (possible partially applied) overlay */
		of_overlay_revert_one(ovinfo);
		return 0;
	}

	/* overlay applied correctly, now create/destroy pdevs */
	list_for_each_entry(re, &ovinfo->de_list, node)
		of_overlay_device_entry_change(ovinfo, re, 0);

	return 0;
}

/**
 * of_overlay	- Apply @count overlays pointed at by @ovinfo_tab
 * @count:	Number of of_overlay_info's
 * @ovinfo_tab:	Array of overlay_info's to apply
 *
 * Applies the overlays given, while handling all error conditions
 * appropriately. Either the operation succeeds, or if it fails the
 * live tree is reverted to the state before the attempt.
 * Returns 0, or an error if the overlay attempt failed.
 */
int of_overlay(int count, struct of_overlay_info *ovinfo_tab)
{
	struct of_overlay_info *ovinfo;
	int i, err;

	if (!ovinfo_tab)
		return -EINVAL;

	/* first we apply the overlays atomically */
	for (i = 0; i < count; i++) {

		ovinfo = &ovinfo_tab[i];

		mutex_lock(&ovinfo->lock);

		err = of_overlay_prep_one(ovinfo);
		if (err == 0)
			err = of_overlay_apply_one(ovinfo->target,
					ovinfo->overlay);
		of_overlay_post_one(ovinfo, err);

		mutex_unlock(&ovinfo->lock);

		if (err != 0) {
			pr_err("%s: overlay failed '%s'\n",
				__func__, ovinfo->target->full_name);
			goto err_fail;
		}
	}

	return 0;

err_fail:
	while (--i >= 0) {
		ovinfo = &ovinfo_tab[i];

		mutex_lock(&ovinfo->lock);
		of_overlay_revert_one(ovinfo);
		mutex_unlock(&ovinfo->lock);
	}

	return err;
}

/**
 * of_overlay_revert	- Revert a previously applied overlay
 * @count:	Number of of_overlay_info's
 * @ovinfo_tab:	Array of overlay_info's to apply
 *
 * Revert a previous overlay. The state of the live tree
 * is reverted to the one before the overlay.
 * Returns 0, or an error if the overlay table is not given.
 */
int of_overlay_revert(int count, struct of_overlay_info *ovinfo_tab)
{
	struct of_overlay_info *ovinfo;
	int i;

	if (!ovinfo_tab)
		return -EINVAL;

	/* revert the overlays in reverse */
	for (i = count - 1; i >= 0; i--) {

		ovinfo = &ovinfo_tab[i];

		mutex_lock(&ovinfo->lock);
		of_overlay_revert_one(ovinfo);
		mutex_unlock(&ovinfo->lock);

	}

	return 0;
}

/**
 * of_init_overlay_info	- Initialize a single of_overlay_info structure
 * @ovinfo:	Pointer to the overlay info structure to initialize
 *
 * Initialize a single overlay info structure.
 */
void of_init_overlay_info(struct of_overlay_info *ovinfo)
{
	memset(ovinfo, 0, sizeof(ovinfo));
	mutex_init(&ovinfo->lock);
	INIT_LIST_HEAD(&ovinfo->de_list);
	INIT_LIST_HEAD(&ovinfo->le_list);

	ovinfo->notifier.notifier_call = of_overlay_notify;
}

/**
 * of_fill_overlay_info	- Fill an overlay info structure
 * @info_node:	Device node containing the overlay
 * @ovinfo:	Pointer to the overlay info structure to fill
 *
 * Fills an overlay info structure with the overlay information
 * from a device node. This device node must have a target property
 * which contains a phandle of the overlay target node, and an
 * __overlay__ child node which has the overlay contents.
 * Both ovinfo->target & ovinfo->overlay have their references taken.
 *
 * Returns 0 on success, or a negative error value.
 */
int of_fill_overlay_info(struct device_node *info_node,
		struct of_overlay_info *ovinfo)
{
	u32 val;
	int ret;

	if (!info_node || !ovinfo)
		return -EINVAL;

	ret = of_property_read_u32(info_node, "target", &val);
	if (ret != 0)
		goto err_fail;

	ovinfo->target = of_find_node_by_phandle(val);
	if (ovinfo->target == NULL)
		goto err_fail;

	ovinfo->overlay = of_get_child_by_name(info_node, "__overlay__");
	if (ovinfo->overlay == NULL)
		goto err_fail;

	return 0;

err_fail:
	of_node_put(ovinfo->target);
	of_node_put(ovinfo->overlay);

	memset(ovinfo, 0, sizeof(*ovinfo));
	return -EINVAL;
}

/**
 * of_build_overlay_info	- Build an overlay info array
 * @tree:	Device node containing all the overlays
 * @cntp:	Pointer to where the overlay info count will be help
 * @ovinfop:	Pointer to the pointer of an overlay info structure.
 *
 * Helper function that given a tree containing overlay information,
 * allocates and builds an overlay info array containing it, ready
 * for use using of_overlay.
 *
 * Returns 0 on success with the @cntp @ovinfop pointers valid,
 * while on error a negative error value is returned.
 */
int of_build_overlay_info(struct device_node *tree,
		int *cntp, struct of_overlay_info **ovinfop)
{
	struct device_node *node;
	struct of_overlay_info *ovinfo;
	int cnt, err;

	if (tree == NULL || cntp == NULL || ovinfop == NULL)
		return -EINVAL;

	/* worst case; every child is a node */
	cnt = 0;
	for_each_child_of_node(tree, node)
		cnt++;

	ovinfo = kzalloc(cnt * sizeof(*ovinfo), GFP_KERNEL);
	if (ovinfo == NULL)
		return -ENOMEM;

	cnt = 0;
	for_each_child_of_node(tree, node) {

		of_init_overlay_info(&ovinfo[cnt]);
		err = of_fill_overlay_info(node, &ovinfo[cnt]);
		if (err == 0)
			cnt++;
	}

	/* if nothing filled, return error */
	if (cnt == 0) {
		kfree(ovinfo);
		return -ENODEV;
	}

	*cntp = cnt;
	*ovinfop = ovinfo;

	return 0;
}

/**
 * of_free_overlay_info	- Free an overlay info array
 * @count:	Number of of_overlay_info's
 * @ovinfo_tab:	Array of overlay_info's to free
 *
 * Releases the memory of a previously allocate ovinfo array
 * by of_build_overlay_info.
 * Returns 0, or an error if the arguments are bogus.
 */
int of_free_overlay_info(int count, struct of_overlay_info *ovinfo_tab)
{
	struct of_overlay_info *ovinfo;
	int i;

	if (!ovinfo_tab || count < 0)
		return -EINVAL;

	/* do it in reverse */
	for (i = count - 1; i >= 0; i--) {
		ovinfo = &ovinfo_tab[i];

		of_node_put(ovinfo->target);
		of_node_put(ovinfo->overlay);
	}
	kfree(ovinfo_tab);

	return 0;
}

