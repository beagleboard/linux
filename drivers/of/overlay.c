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

#define pr_fmt(fmt)	"OF: overlay: " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/idr.h>
#include <linux/sysfs.h>
#include <linux/atomic.h>

#include "of_private.h"

/* fwd. decl */
struct of_overlay;
struct of_overlay_info;

/* an attribute for each fragment */
struct fragment_attribute {
	struct attribute attr;
	ssize_t (*show)(struct kobject *kobj, struct fragment_attribute *fattr,
			char *buf);
	ssize_t (*store)(struct kobject *kobj, struct fragment_attribute *fattr,
			 const char *buf, size_t count);
	struct of_overlay_info *ovinfo;
};

/**
 * struct of_overlay_info - Holds a single overlay info
 * @info:	info node that contains the target and overlay
 * @target:	target of the overlay operation
 * @overlay:	pointer to the overlay contents node
 *
 * Holds a single overlay state, including all the overlay logs &
 * records.
 */
struct of_overlay_info {
	struct of_overlay *ov;
	struct device_node *info;
	struct device_node *target;
	struct device_node *overlay;
	struct attribute_group attr_group;
	struct attribute *attrs[2];
	struct fragment_attribute target_attr;
	bool is_symbols_node;
};

/**
 * struct of_overlay - Holds a complete overlay transaction
 * @node:	List on which we are located
 * @count:	Count of ovinfo structures
 * @ovinfo_tab:	Overlay info table (count sized)
 * @cset:	Changeset to be used
 *
 * Holds a complete overlay transaction
 */
struct of_overlay {
	int id;
	struct list_head node;
	int count;
	struct of_overlay_info *ovinfo_tab;
	const struct attribute_group **attr_groups;
	struct of_changeset cset;
	struct kobject kobj;
	int target_index;
	struct device_node *target_root;
};

/* master enable switch; once set to 0 can't be re-enabled */
static atomic_t ov_enable = ATOMIC_INIT(1);

static int __init of_overlay_disable_setup(char *str __always_unused)
{
	atomic_set(&ov_enable, 0);
	return 1;
}
__setup("of_overlay_disable", of_overlay_disable_setup);

static int of_overlay_apply_one(struct of_overlay *ov,
		struct device_node *target, const struct device_node *overlay,
		bool is_symbols_node);
static int overlay_removal_is_ok(struct of_overlay *ov);

static BLOCKING_NOTIFIER_HEAD(of_overlay_chain);

int of_overlay_notifier_register(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&of_overlay_chain, nb);
}
EXPORT_SYMBOL_GPL(of_overlay_notifier_register);

int of_overlay_notifier_unregister(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&of_overlay_chain, nb);
}
EXPORT_SYMBOL_GPL(of_overlay_notifier_unregister);

static int of_overlay_notify(struct of_overlay *ov,
			     enum of_overlay_notify_action action)
{
	struct of_overlay_notify_data nd;
	int i, ret;

	for (i = 0; i < ov->count; i++) {
		struct of_overlay_info *ovinfo = &ov->ovinfo_tab[i];

		nd.target = ovinfo->target;
		nd.overlay = ovinfo->overlay;

		ret = blocking_notifier_call_chain(&of_overlay_chain,
						   action, &nd);
		if (ret)
			return notifier_to_errno(ret);
	}

	return 0;
}

static struct property *dup_and_fixup_symbol_prop(struct of_overlay *ov,
		const struct property *prop)
{
	struct of_overlay_info *ovinfo;
	struct property *new;
	const char *overlay_name;
	char *label_path;
	char *symbol_path;
	const char *target_path;
	int k;
	int label_path_len;
	int overlay_name_len;
	int target_path_len;

	if (!prop->value)
		return NULL;
	symbol_path = prop->value;

	new = kzalloc(sizeof(*new), GFP_KERNEL);
	if (!new)
		return NULL;

	for (k = 0; k < ov->count; k++) {
		ovinfo = &ov->ovinfo_tab[k];
		overlay_name = ovinfo->overlay->full_name;
		overlay_name_len = strlen(overlay_name);
		if (!strncasecmp(symbol_path, overlay_name, overlay_name_len))
			break;
	}

	if (k >= ov->count)
		goto err_free;

	target_path = ovinfo->target->full_name;
	target_path_len = strlen(target_path);

	label_path = symbol_path + overlay_name_len;
	label_path_len = strlen(label_path);

	new->name = kstrdup(prop->name, GFP_KERNEL);
	new->length = target_path_len + label_path_len + 1;
	new->value = kzalloc(new->length, GFP_KERNEL);

	if (!new->name || !new->value)
		goto err_free;

	strcpy(new->value, target_path);
	strcpy(new->value + target_path_len, label_path);

	/* mark the property as dynamic */
	of_property_set_flag(new, OF_DYNAMIC);

	return new;

 err_free:
	kfree(new->name);
	kfree(new->value);
	kfree(new);
	return NULL;


}

static int of_overlay_apply_single_property(struct of_overlay *ov,
		struct device_node *target, struct property *prop,
		bool is_symbols_node)
{
	struct property *propn = NULL, *tprop;

	/* NOTE: Multiple changes of single properties not supported */
	tprop = of_find_property(target, prop->name, NULL);

	/* special properties are not meant to be updated (silent NOP) */
	if (of_prop_cmp(prop->name, "name") == 0 ||
	    of_prop_cmp(prop->name, "phandle") == 0 ||
	    of_prop_cmp(prop->name, "linux,phandle") == 0)
		return 0;

	if (is_symbols_node) {
		/* changing a property in __symbols__ node not allowed */
		if (tprop)
			return -EINVAL;
		propn = dup_and_fixup_symbol_prop(ov, prop);
	} else {
		propn = __of_prop_dup(prop, GFP_KERNEL);
	}

	if (propn == NULL)
		return -ENOMEM;

	/* not found? add */
	if (tprop == NULL)
		return of_changeset_add_property(&ov->cset, target, propn);

	/* found? update */
	return of_changeset_update_property(&ov->cset, target, propn);
}

static int of_overlay_apply_single_device_node(struct of_overlay *ov,
		struct device_node *target, struct device_node *child)
{
	const char *cname;
	struct device_node *tchild;
	int ret = 0;

	cname = kbasename(child->full_name);
	if (cname == NULL)
		return -ENOMEM;

	/* NOTE: Multiple mods of created nodes not supported */
	for_each_child_of_node(target, tchild)
		if (!of_node_cmp(cname, kbasename(tchild->full_name)))
			break;

	if (tchild != NULL) {
		/* new overlay phandle value conflicts with existing value */
		if (child->phandle)
			return -EINVAL;

		/* apply overlay recursively */
		ret = of_overlay_apply_one(ov, tchild, child, 0);
		of_node_put(tchild);
	} else {
		/* create empty tree as a target */
		tchild = __of_node_dup(child, "%pOF/%s", target, cname);
		if (!tchild)
			return -ENOMEM;

		/* point to parent */
		tchild->parent = target;

		ret = of_changeset_attach_node(&ov->cset, tchild);
		if (ret)
			return ret;

		ret = of_overlay_apply_one(ov, tchild, child, 0);
		if (ret)
			return ret;
	}

	return ret;
}

/*
 * Apply a single overlay node recursively.
 *
 * Note that the in case of an error the target node is left
 * in a inconsistent state. Error recovery should be performed
 * by using the changeset.
 */
static int of_overlay_apply_one(struct of_overlay *ov,
		struct device_node *target, const struct device_node *overlay,
		bool is_symbols_node)
{
	struct device_node *child;
	struct property *prop;
	int ret;

	for_each_property_of_node(overlay, prop) {
		ret = of_overlay_apply_single_property(ov, target, prop,
						       is_symbols_node);
		if (ret) {
			pr_err("Failed to apply prop @%pOF/%s\n",
			       target, prop->name);
			return ret;
		}
	}

	/* do not allow symbols node to have any children */
	if (is_symbols_node)
		return 0;

	for_each_child_of_node(overlay, child) {
		ret = of_overlay_apply_single_device_node(ov, target, child);
		if (ret != 0) {
			pr_err("Failed to apply single node @%pOF/%s\n",
			       target, child->name);
			of_node_put(child);
			return ret;
		}
	}

	return 0;
}

/**
 * of_overlay_apply() - Apply @count overlays pointed at by @ovinfo_tab
 * @ov:		Overlay to apply
 *
 * Applies the overlays given, while handling all error conditions
 * appropriately. Either the operation succeeds, or if it fails the
 * live tree is reverted to the state before the attempt.
 * Returns 0, or an error if the overlay attempt failed.
 */
static int of_overlay_apply(struct of_overlay *ov)
{
	int i, err;

	/* first we apply the overlays atomically */
	for (i = 0; i < ov->count; i++) {
		struct of_overlay_info *ovinfo = &ov->ovinfo_tab[i];

		err = of_overlay_apply_one(ov, ovinfo->target, ovinfo->overlay,
					   ovinfo->is_symbols_node);
		if (err != 0) {
			pr_err("apply failed '%pOF'\n", ovinfo->target);
			return err;
		}
	}

	return 0;
}

/*
 * Find the target node using a number of different strategies
 * in order of preference. Respects the target index if available.
 *
 * "target" property containing the phandle of the target
 * "target-path" property containing the path of the target
 */
static struct device_node *find_target_node(struct of_overlay *ov,
		struct device_node *info_node, int index)
{
	struct device_node *target = NULL, *np;
	const char *path;
	char *newpath;
	u32 val;
	int ret;

	/* first try to go by using the target as a phandle */
	ret = of_property_read_u32_index(info_node, "target", index, &val);
	if (ret == 0) {
		target = of_find_node_by_phandle(val);
		if (!target) {
			pr_err("%s: Could not find target phandle 0x%x\n",
					__func__, val);
			return NULL;
		}
		goto check_root;
	}

	/* failed, try to locate by path */
	ret = of_property_read_string_index(info_node, "target-path", index,
			&path);
	if (ret == 0) {

		if (!ov->target_root) {
			target = of_find_node_by_path(path);
			if (!target)
				pr_err("%s: Could not find target path \"%s\"\n",
						__func__, path);
			return target;
		}

		/* remove preceding '/' from path; relative path */
		if (*path == '/') {
			while (*path == '/')
				path++;

			newpath = kasprintf(GFP_KERNEL, "%s%s%s",
					of_node_full_name(ov->target_root),
					*path ? "/" : "", path);
			if (!newpath) {
				pr_err("%s: Could not allocate \"%s%s%s\"\n",
					__func__,
					of_node_full_name(ov->target_root),
					*path ? "/" : "", path);
				return NULL;
			}
			target = of_find_node_by_path(newpath);
			kfree(newpath);

			return target;

		}
		/* target is an alias, need to check */
		target = of_find_node_by_path(path);
		if (!target) {
			pr_err("%s: Could not find alias \"%s\"\n",
					__func__, path);
			return NULL;
		}
		goto check_root;
	}

	return NULL;

check_root:
	if (!ov->target_root)
		return target;

	/* got a target, but we have to check it's under target root */
	for (np = target; np; np = np->parent) {
		if (np == ov->target_root)
			return target;
	}
	pr_err("%s: target \"%s\" not under target_root \"%s\"\n",
			__func__, of_node_full_name(target),
			of_node_full_name(ov->target_root));
	/* target is not under target_root */
	of_node_put(target);
	return NULL;
}

/**
 * of_fill_overlay_info() - Fill an overlay info structure
 * @ov		Overlay to fill
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
static int of_fill_overlay_info(struct of_overlay *ov,
		struct device_node *info_node, struct of_overlay_info *ovinfo)
{
	ovinfo->overlay = of_get_child_by_name(info_node, "__overlay__");
	if (ovinfo->overlay == NULL)
		goto err_fail;

	ovinfo->target = find_target_node(ov, info_node, ov->target_index);
	if (ovinfo->target == NULL)
		goto err_fail;

	ovinfo->info = of_node_get(info_node);

	return 0;

err_fail:
	of_node_put(ovinfo->target);
	of_node_put(ovinfo->overlay);

	memset(ovinfo, 0, sizeof(*ovinfo));
	return -EINVAL;
}

static ssize_t target_show(struct kobject *kobj,
		struct fragment_attribute *fattr, char *buf)
{
	struct of_overlay_info *ovinfo = fattr->ovinfo;

	return snprintf(buf, PAGE_SIZE, "%s\n",
			of_node_full_name(ovinfo->target));
}

static const struct fragment_attribute target_template_attr = __ATTR_RO(target);

/**
 * of_build_overlay_info() - Build an overlay info array
 * @ov		Overlay to build
 * @tree:	Device node containing all the overlays
 *
 * Helper function that given a tree containing overlay information,
 * allocates and builds an overlay info array containing it, ready
 * for use using of_overlay_apply.
 *
 * Returns 0 on success with the @cntp @ovinfop pointers valid,
 * while on error a negative error value is returned.
 */
static int of_build_overlay_info(struct of_overlay *ov,
		struct device_node *tree)
{
	struct device_node *node;
	struct of_overlay_info *ovinfo;
	int i, cnt, err;

	/* worst case; every child is a node */
	cnt = 0;
	for_each_child_of_node(tree, node)
		cnt++;

	if (of_get_child_by_name(tree, "__symbols__"))
		cnt++;

	ovinfo = kcalloc(cnt, sizeof(*ovinfo), GFP_KERNEL);
	if (ovinfo == NULL)
		return -ENOMEM;

	cnt = 0;
	for_each_child_of_node(tree, node) {
		err = of_fill_overlay_info(ov, node, &ovinfo[cnt]);
		if (err == 0)
			cnt++;
	}

	node = of_get_child_by_name(tree, "__symbols__");
	if (node) {
		ovinfo[cnt].overlay = node;
		ovinfo[cnt].target = of_find_node_by_path("/__symbols__");
		ovinfo[cnt].is_symbols_node = 1;

		if (!ovinfo[cnt].target) {
			pr_err("no symbols in root of device tree.\n");
			return -EINVAL;
		}

		cnt++;
	}

	/* if nothing filled, return error */
	if (cnt == 0) {
		err = -ENODEV;
		goto err_free_ovinfo;
	}

	ov->count = cnt;
	ov->ovinfo_tab = ovinfo;

	ov->attr_groups = kcalloc(cnt + 1,
			sizeof(struct attribute_group *), GFP_KERNEL);
	if (ov->attr_groups == NULL) {
		err = -ENOMEM;
		goto err_free_ovinfo;
	}

	for (i = 0; i < cnt; i++) {
		ovinfo = &ov->ovinfo_tab[i];

		ov->attr_groups[i] = &ovinfo->attr_group;

		ovinfo->target_attr = target_template_attr;
		/* make lockdep happy */
		sysfs_attr_init(&ovinfo->target_attr.attr);
		ovinfo->target_attr.ovinfo = ovinfo;

		ovinfo->attrs[0] = &ovinfo->target_attr.attr;
		ovinfo->attrs[1] = NULL;

		/* NOTE: direct reference to the full_name */
		if(ovinfo->info != NULL)
			ovinfo->attr_group.name = kbasename(ovinfo->info->full_name);
		else
			ovinfo->attr_group.name = "__symbols__";

		ovinfo->attr_group.attrs = ovinfo->attrs;

	}
	ov->attr_groups[i] = NULL;

	return 0;

err_free_ovinfo:
	kfree(ovinfo);
	return err;
}

/**
 * of_free_overlay_info() - Free an overlay info array
 * @ov		Overlay to free the overlay info from
 * @ovinfo_tab:	Array of overlay_info's to free
 *
 * Releases the memory of a previously allocated ovinfo array
 * by of_build_overlay_info.
 * Returns 0, or an error if the arguments are bogus.
 */
static int of_free_overlay_info(struct of_overlay *ov)
{
	struct of_overlay_info *ovinfo;
	int i;

	/* free attribute groups space */
	kfree(ov->attr_groups);

	/* do it in reverse */
	for (i = ov->count - 1; i >= 0; i--) {
		ovinfo = &ov->ovinfo_tab[i];

		of_node_put(ovinfo->target);
		of_node_put(ovinfo->overlay);
		of_node_put(ovinfo->info);
	}
	kfree(ov->ovinfo_tab);

	return 0;
}

static int of_overlay_add_symbols(
		struct device_node *tree,
		struct of_overlay *ov)
{
	struct of_overlay_info *ovinfo;
	struct device_node *root_sym = NULL;
	struct device_node *child = NULL;
	struct property *prop;
	const char *path, *s;
	char *new_path;
	int i, len, err;

	/* both may fail (if no fixups are required) */
	root_sym = of_find_node_by_path("/__symbols__");
	child = of_get_child_by_name(tree, "__symbols__");

	err = 0;
	/* do nothing if either is NULL */
	if (!root_sym || !child)
		goto out;

	for_each_property_of_node(child, prop) {

		/* skip properties added automatically */
		if (of_prop_cmp(prop->name, "name") == 0)
			continue;

		err = of_property_read_string(child,
				prop->name, &path);
		if (err != 0) {
			pr_err("Could not find symbol '%s'\n", prop->name);
			continue;
		}

		/* now find fragment index */
		s = path;

		/* compare paths to find fragment index */
		for (i = 0, ovinfo = NULL, len = -1; i < ov->count; i++) {
			ovinfo = &ov->ovinfo_tab[i];

			pr_debug("#%d: overlay->name=%s target->name=%s\n",
					i, ovinfo->overlay->full_name,
					ovinfo->target->full_name);

			len = strlen(ovinfo->overlay->full_name);
			if (strncasecmp(path, ovinfo->overlay->full_name,
						len) == 0 && path[len] == '/')
				break;
		}

		if (i >= ov->count)
			continue;

		pr_debug("found target at #%d\n", i);
		new_path = kasprintf(GFP_KERNEL, "%s%s",
				ovinfo->target->full_name,
				path + len);
		if (!new_path) {
			pr_err("Failed to allocate propname for \"%s\"\n",
					prop->name);
			err = -ENOMEM;
			break;
		}

		err = of_changeset_add_property_string(&ov->cset, root_sym,
				prop->name, new_path);

		/* free always */
		kfree(new_path);

		if (err) {
			pr_err("Failed to add property for \"%s\"\n",
					prop->name);
			break;
		}
	}

out:
	of_node_put(child);
	of_node_put(root_sym);

	return err;
}

static LIST_HEAD(ov_list);
static DEFINE_IDR(ov_idr);

static inline struct of_overlay *kobj_to_overlay(struct kobject *kobj)
{
	return container_of(kobj, struct of_overlay, kobj);
}

void of_overlay_release(struct kobject *kobj)
{
	struct of_overlay *ov = kobj_to_overlay(kobj);

	of_node_put(ov->target_root);
	kfree(ov);
}

static ssize_t enable_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&ov_enable));
}

static ssize_t enable_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	bool new_enable;

	ret = strtobool(buf, &new_enable);
	if (ret != 0)
		return ret;
	/* if we've disabled it, no going back */
	if (atomic_read(&ov_enable) == 0)
		return -EPERM;
	atomic_set(&ov_enable, (int)new_enable);
	return count;
}

static struct kobj_attribute enable_attr = __ATTR_RW(enable);

static const struct attribute *overlay_global_attrs[] = {
	&enable_attr.attr,
	NULL
};

static ssize_t can_remove_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	struct of_overlay *ov = kobj_to_overlay(kobj);

	return snprintf(buf, PAGE_SIZE, "%d\n", overlay_removal_is_ok(ov));
}

static struct kobj_attribute can_remove_attr = __ATTR_RO(can_remove);

static struct attribute *overlay_attrs[] = {
	&can_remove_attr.attr,
	NULL
};

static struct kobj_type of_overlay_ktype = {
	.release = of_overlay_release,
	.sysfs_ops = &kobj_sysfs_ops,	/* default kobj sysfs ops */
	.default_attrs = overlay_attrs,
};

static struct kset *ov_kset;

static int __of_overlay_create(struct device_node *tree,
		int target_index, struct device_node *target_root)
{
	struct of_overlay *ov;
	int err, id;

	/* administratively disabled */
	if (!atomic_read(&ov_enable))
		return -EPERM;

	/* allocate the overlay structure */
	ov = kzalloc(sizeof(*ov), GFP_KERNEL);
	if (ov == NULL)
		return -ENOMEM;
	ov->id = -1;

	ov->target_index = target_index;
	ov->target_root = of_node_get(target_root);

	INIT_LIST_HEAD(&ov->node);

	of_changeset_init(&ov->cset);

	/* initialize kobject */
	kobject_init(&ov->kobj, &of_overlay_ktype);

	mutex_lock(&of_mutex);

	id = idr_alloc(&ov_idr, ov, 0, 0, GFP_KERNEL);
	if (id < 0) {
		err = id;
		goto err_destroy_trans;
	}
	ov->id = id;

	/* build the overlay info structures */
	err = of_build_overlay_info(ov, tree);
	if (err) {
		pr_err("of_build_overlay_info() failed for tree@%pOF\n",
		       tree);
		goto err_free_idr;
	}

	err = of_overlay_notify(ov, OF_OVERLAY_PRE_APPLY);
	if (err < 0) {
		pr_err("%s: Pre-apply notifier failed (err=%d)\n",
		       __func__, err);
		goto err_free_idr;
	}

	/* apply the overlay */
	err = of_overlay_apply(ov);
	if (err)
		goto err_abort_trans;

	err = of_overlay_add_symbols(tree, ov);
	if (err) {
		pr_err("%s: of_overlay_add_symbols() failed for tree@%s\n",
				__func__, tree->full_name);
		goto err_abort_trans;
	}

	/* apply the changeset */
	err = __of_changeset_apply(&ov->cset);
	if (err)
		goto err_revert_overlay;

	err = sysfs_create_groups(&ov->kobj, ov->attr_groups);
	if (err != 0) {
		pr_err("%s: sysfs_create_groups() failed for tree@%s\n",
				__func__, tree->full_name);
		goto err_remove_kobj;
	}

	ov->kobj.kset = ov_kset;
	err = kobject_add(&ov->kobj, NULL, "%d", id);
	if (err != 0) {
		pr_err("%s: kobject_add() failed for tree@%s\n",
				__func__, tree->full_name);
		goto err_cancel_overlay;
	}

	/* add to the tail of the overlay list */
	list_add_tail(&ov->node, &ov_list);

	of_overlay_notify(ov, OF_OVERLAY_POST_APPLY);

	mutex_unlock(&of_mutex);

	return id;

err_remove_kobj:
	kobject_put(&ov->kobj);
err_cancel_overlay:
	of_changeset_revert(&ov->cset);
err_revert_overlay:
err_abort_trans:
	of_free_overlay_info(ov);
err_free_idr:
	idr_remove(&ov_idr, ov->id);
err_destroy_trans:
	of_changeset_destroy(&ov->cset);
	of_node_put(ov->target_root);
	kfree(ov);
	mutex_unlock(&of_mutex);

	return err;
}

/**
 * of_overlay_create() - Create and apply an overlay
 * @tree:	Device node containing all the overlays
 *
 * Creates and applies an overlay while also keeping track
 * of the overlay in a list. This list can be used to prevent
 * illegal overlay removals.
 *
 * Returns the id of the created overlay, or a negative error number
 */
int of_overlay_create(struct device_node *tree)
{
	return __of_overlay_create(tree, 0, NULL);
}
EXPORT_SYMBOL_GPL(of_overlay_create);

/**
 * of_overlay_create_target_index() - Create and apply an overlay
 * @tree:	Device node containing all the overlays
 * @index:	Index to use in the target properties
 *
 * Creates and applies an overlay while also keeping track
 * of the overlay in a list. This list can be used to prevent
 * illegal overlay removals.
 *
 * Returns the id of the created overlay, or a negative error number
 */
int of_overlay_create_target_index(struct device_node *tree, int index)
{
	return __of_overlay_create(tree, index, NULL);
}
EXPORT_SYMBOL_GPL(of_overlay_create_target_index);

/**
 * of_overlay_create_target_root() - Create and apply an overlay
 *			under which will be limited to target_root
 * @tree:		Device node containing all the overlays
 * @target_root:	Target root for the overlay.
 *
 * Creates and applies an overlay while also keeping track
 * of the overlay in a list. This list can be used to prevent
 * illegal overlay removals. The overlay is only allowed to
 * target nodes under the target_root node.
 *
 * Returns the id of the created overlay, or an negative error number
 */
int of_overlay_create_target_root(struct device_node *tree,
		struct device_node *target_root)
{
	return __of_overlay_create(tree, 0, target_root);
}
EXPORT_SYMBOL_GPL(of_overlay_create_target_root);

/* check whether the given node, lies under the given tree */
static int overlay_subtree_check(struct device_node *tree,
		struct device_node *dn)
{
	struct device_node *child;

	/* match? */
	if (tree == dn)
		return 1;

	for_each_child_of_node(tree, child) {
		if (overlay_subtree_check(child, dn)) {
			of_node_put(child);
			return 1;
		}
	}

	return 0;
}

/* check whether this overlay is the topmost */
static int overlay_is_topmost(struct of_overlay *ov, struct device_node *dn)
{
	struct of_overlay *ovt;
	struct of_changeset_entry *ce;

	list_for_each_entry_reverse(ovt, &ov_list, node) {
		/* if we hit ourselves, we're done */
		if (ovt == ov)
			break;

		/* check against each subtree affected by this overlay */
		list_for_each_entry(ce, &ovt->cset.entries, node) {
			if (overlay_subtree_check(ce->np, dn)) {
				pr_err("%s: #%d clashes #%d @%pOF\n",
					__func__, ov->id, ovt->id, dn);
				return 0;
			}
		}
	}

	/* overlay is topmost */
	return 1;
}

/*
 * We can safely remove the overlay only if it's the top-most one.
 * Newly applied overlays are inserted at the tail of the overlay list,
 * so a top most overlay is the one that is closest to the tail.
 *
 * The topmost check is done by exploiting this property. For each
 * affected device node in the log list we check if this overlay is
 * the one closest to the tail. If another overlay has affected this
 * device node and is closest to the tail, then removal is not permited.
 */
static int overlay_removal_is_ok(struct of_overlay *ov)
{
	struct of_changeset_entry *ce;

	list_for_each_entry(ce, &ov->cset.entries, node) {
		if (!overlay_is_topmost(ov, ce->np)) {
			pr_err("overlay #%d is not topmost\n", ov->id);
			return 0;
		}
	}

	return 1;
}

/**
 * of_overlay_destroy() - Removes an overlay
 * @id:	Overlay id number returned by a previous call to of_overlay_create
 *
 * Removes an overlay if it is permissible.
 *
 * Returns 0 on success, or a negative error number
 */
int of_overlay_destroy(int id)
{
	struct of_overlay *ov;
	int err;

	mutex_lock(&of_mutex);

	ov = idr_find(&ov_idr, id);
	if (ov == NULL) {
		err = -ENODEV;
		pr_err("destroy: Could not find overlay #%d\n", id);
		goto out;
	}

	/* check whether the overlay is safe to remove */
	if (!overlay_removal_is_ok(ov)) {
		err = -EBUSY;
		goto out;
	}

	of_overlay_notify(ov, OF_OVERLAY_PRE_REMOVE);
	list_del(&ov->node);
	sysfs_remove_groups(&ov->kobj, ov->attr_groups);
	__of_changeset_revert(&ov->cset);
	of_overlay_notify(ov, OF_OVERLAY_POST_REMOVE);
	of_free_overlay_info(ov);
	idr_remove(&ov_idr, id);
	of_changeset_destroy(&ov->cset);

	kobject_put(&ov->kobj);

	err = 0;

out:
	mutex_unlock(&of_mutex);

	return err;
}
EXPORT_SYMBOL_GPL(of_overlay_destroy);

/**
 * of_overlay_destroy_all() - Removes all overlays from the system
 *
 * Removes all overlays from the system in the correct order.
 *
 * Returns 0 on success, or a negative error number
 */
int of_overlay_destroy_all(void)
{
	struct of_overlay *ov, *ovn;

	mutex_lock(&of_mutex);

	/* the tail of list is guaranteed to be safe to remove */
	list_for_each_entry_safe_reverse(ov, ovn, &ov_list, node) {
		list_del(&ov->node);
		__of_changeset_revert(&ov->cset);
		of_free_overlay_info(ov);
		idr_remove(&ov_idr, ov->id);
		kobject_put(&ov->kobj);
	}

	mutex_unlock(&of_mutex);

	return 0;
}
EXPORT_SYMBOL_GPL(of_overlay_destroy_all);

/* called from of_init() */
int of_overlay_init(void)
{
	int rc;

	ov_kset = kset_create_and_add("overlays", NULL, &of_kset->kobj);
	if (!ov_kset)
		return -ENOMEM;

	rc = sysfs_create_files(&ov_kset->kobj, overlay_global_attrs);
	WARN(rc, "%s: error adding global attributes\n", __func__);

	return rc;
}
