/*
 * Functions for dealing with DT resolution
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
#include <linux/of_fdt.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/slab.h>

/**
 * Find a subtree's maximum phandle value.
 */
static phandle __of_get_tree_max_phandle(struct device_node *node,
		phandle max_phandle)
{
	struct device_node *child;

	if (node->phandle != 0 && node->phandle != OF_PHANDLE_ILLEGAL &&
			node->phandle > max_phandle)
		max_phandle = node->phandle;

	__for_each_child_of_node(node, child)
		max_phandle = __of_get_tree_max_phandle(child, max_phandle);

	return max_phandle;
}

/**
 * Find live tree's maximum phandle value.
 */
static phandle of_get_tree_max_phandle(void)
{
	struct device_node *node;
	phandle phandle;

	/* get root node */
	node = of_find_node_by_path("/");
	if (node == NULL)
		return OF_PHANDLE_ILLEGAL;

	/* now search recursively */
	read_lock(&devtree_lock);
	phandle = __of_get_tree_max_phandle(node, 0);
	read_unlock(&devtree_lock);

	of_node_put(node);

	return phandle;
}

/**
 * Adjust a subtree's phandle values by a given delta.
 * Makes sure not to just adjust the device node's phandle value,
 * but modify the phandle properties values as well.
 */
static void __of_adjust_tree_phandles(struct device_node *node,
		int phandle_delta)
{
	struct device_node *child;
	struct property *prop;
	phandle phandle;

	/* first adjust the node's phandle direct value */
	if (node->phandle != 0 && node->phandle != OF_PHANDLE_ILLEGAL)
		node->phandle += phandle_delta;

	/* now adjust phandle & linux,phandle values */
	for_each_property_of_node(node, prop) {

		/* only look for these two */
		if (of_prop_cmp(prop->name, "phandle") != 0 &&
		    of_prop_cmp(prop->name, "linux,phandle") != 0)
			continue;

		/* must be big enough */
		if (prop->length < 4)
			continue;

		/* read phandle value */
		phandle = be32_to_cpu(*(uint32_t *)prop->value);
		if (phandle == OF_PHANDLE_ILLEGAL)	/* unresolved */
			continue;

		/* adjust */
		*(uint32_t *)prop->value = cpu_to_be32(node->phandle);
	}

	/* now do the children recursively */
	__for_each_child_of_node(node, child)
		__of_adjust_tree_phandles(child, phandle_delta);
}

/**
 * Adjust the local phandle references by the given phandle delta.
 * Assumes the existances of a __local_fixups__ node at the root
 * of the tree. Does not take any devtree locks so make sure you
 * call this on a tree which is at the detached state.
 */
static int __of_adjust_tree_phandle_references(struct device_node *node,
		int phandle_delta)
{
	phandle phandle;
	struct device_node *refnode, *child;
	struct property *rprop, *sprop;
	char *propval, *propcur, *propend, *nodestr, *propstr, *s;
	int offset, propcurlen;
	int err;

	/* locate the symbols & fixups nodes on resolve */
	__for_each_child_of_node(node, child)
		if (of_node_cmp(child->name, "__local_fixups__") == 0)
			break;

	/* no local fixups */
	if (child == NULL)
		return 0;

	/* find the local fixups property */
	for_each_property_of_node(child, rprop) {

		/* skip properties added automatically */
		if (of_prop_cmp(rprop->name, "name") == 0)
			continue;

		/* make a copy */
		propval = kmalloc(rprop->length, GFP_KERNEL);
		if (propval == NULL) {
			pr_err("%s: Could not copy value of '%s'\n",
					__func__, rprop->name);
			return -ENOMEM;
		}
		memcpy(propval, rprop->value, rprop->length);

		propend = propval + rprop->length;
		for (propcur = propval; propcur < propend;
				propcur += propcurlen + 1) {

			propcurlen = strlen(propcur);

			nodestr = propcur;
			s = strchr(propcur, ':');
			if (s == NULL) {
				pr_err("%s: Illegal symbol entry '%s' (1)\n",
					__func__, propcur);
				err = -EINVAL;
				goto err_fail;
			}
			*s++ = '\0';

			propstr = s;
			s = strchr(s, ':');
			if (s == NULL) {
				pr_err("%s: Illegal symbol entry '%s' (2)\n",
					__func__, (char *)rprop->value);
				err = -EINVAL;
				goto err_fail;
			}

			*s++ = '\0';
			offset = simple_strtoul(s, NULL, 10);

			/* look into the resolve node for the full path */
			refnode = __of_find_node_by_full_name(node, nodestr);
			if (refnode == NULL) {
				pr_warn("%s: Could not find refnode '%s'\n",
					__func__, (char *)rprop->value);
				continue;
			}

			/* now find the property */
			for_each_property_of_node(refnode, sprop) {
				if (of_prop_cmp(sprop->name, propstr) == 0)
					break;
			}

			if (sprop == NULL) {
				pr_err("%s: Could not find property '%s'\n",
					__func__, (char *)rprop->value);
				err = -ENOENT;
				goto err_fail;
			}

			phandle = be32_to_cpu(*(uint32_t *)
					(sprop->value + offset));
			*(uint32_t *)(sprop->value + offset) =
				cpu_to_be32(phandle + phandle_delta);
		}

		kfree(propval);
	}

	return 0;

err_fail:
	kfree(propval);
	return err;
}

/**
 * of_resolve	- Resolve the given node against the live tree.
 *
 * @resolve:	Node to resolve
 *
 * Perform dynamic Device Tree resolution against the live tree
 * to the given node to resolve. This depends on the live tree
 * having a __symbols__ node, and the resolve node the __fixups__ &
 * __local_fixups__ nodes (if needed).
 * The result of the operation is a resolve node that it's contents
 * are fit to be inserted or operate upon the live tree.
 * Returns 0 on success or a negative error value on error.
 */
int of_resolve(struct device_node *resolve)
{
	struct device_node *child, *refnode;
	struct device_node *root_sym, *resolve_sym, *resolve_fix;
	struct property *rprop, *sprop;
	const char *refpath;
	char *propval, *propcur, *propend, *nodestr, *propstr, *s;
	int offset, propcurlen;
	phandle phandle, phandle_delta;
	int err;

	/* the resolve node must exist, and be detached */
	if (resolve == NULL ||
			!of_node_check_flag(resolve, OF_DETACHED)) {
		return -EINVAL;
	}

	/* first we need to adjust the phandles */
	phandle_delta = of_get_tree_max_phandle() + 1;
	__of_adjust_tree_phandles(resolve, phandle_delta);
	err = __of_adjust_tree_phandle_references(resolve, phandle_delta);
	if (err != 0)
		return err;

	root_sym = NULL;
	resolve_sym = NULL;
	resolve_fix = NULL;

	/* this may fail (if no fixups are required) */
	root_sym = of_find_node_by_path("/__symbols__");

	/* locate the symbols & fixups nodes on resolve */
	__for_each_child_of_node(resolve, child) {

		if (resolve_sym == NULL &&
				of_node_cmp(child->name, "__symbols__") == 0)
			resolve_sym = child;

		if (resolve_fix == NULL &&
				of_node_cmp(child->name, "__fixups__") == 0)
			resolve_fix = child;

		/* both found, don't bother anymore */
		if (resolve_sym != NULL && resolve_fix != NULL)
			break;
	}

	/* we do allow for the case where no fixups are needed */
	if (resolve_fix == NULL)
		goto merge_sym;

	/* we need to fixup, but no root symbols... */
	if (root_sym == NULL)
		return -EINVAL;

	for_each_property_of_node(resolve_fix, rprop) {

		/* skip properties added automatically */
		if (of_prop_cmp(rprop->name, "name") == 0)
			continue;

		err = of_property_read_string(root_sym,
				rprop->name, &refpath);
		if (err != 0) {
			pr_err("%s: Could not find symbol '%s'\n",
					__func__, rprop->name);
			goto err_fail;
		}

		refnode = of_find_node_by_path(refpath);
		if (refnode == NULL) {
			pr_err("%s: Could not find node by path '%s'\n",
					__func__, refpath);
			err = -ENOENT;
			goto err_fail;
		}

		phandle = refnode->phandle;
		of_node_put(refnode);

		pr_debug("%s: %s phandle is 0x%08x\n",
				__func__, rprop->name, phandle);

		/* make a copy */
		propval = kmalloc(rprop->length, GFP_KERNEL);
		if (propval == NULL) {
			pr_err("%s: Could not copy value of '%s'\n",
					__func__, rprop->name);
			err = -ENOMEM;
			goto err_fail;
		}

		memcpy(propval, rprop->value, rprop->length);

		propend = propval + rprop->length;
		for (propcur = propval; propcur < propend;
				propcur += propcurlen + 1) {
			propcurlen = strlen(propcur);

			nodestr = propcur;
			s = strchr(propcur, ':');
			if (s == NULL) {
				pr_err("%s: Illegal symbol "
					"entry '%s' (1)\n",
					__func__, (char *)rprop->value);
				kfree(propval);
				err = -EINVAL;
				goto err_fail;
			}
			*s++ = '\0';

			propstr = s;
			s = strchr(s, ':');
			if (s == NULL) {
				pr_err("%s: Illegal symbol "
					"entry '%s' (2)\n",
					__func__, (char *)rprop->value);
				kfree(propval);
				err = -EINVAL;
				goto err_fail;
			}

			*s++ = '\0';
			offset = simple_strtoul(s, NULL, 10);

			/* look into the resolve node for the full path */
			refnode = __of_find_node_by_full_name(resolve,
					nodestr);
			if (refnode == NULL) {
				pr_err("%s: Could not find refnode '%s'\n",
					__func__, (char *)rprop->value);
				kfree(propval);
				err = -ENOENT;
				goto err_fail;
			}

			/* now find the property */
			for_each_property_of_node(refnode, sprop) {
				if (of_prop_cmp(sprop->name, propstr) == 0)
					break;
			}

			if (sprop == NULL) {
				pr_err("%s: Could not find property '%s'\n",
					__func__, (char *)rprop->value);
				kfree(propval);
				err = -ENOENT;
				goto err_fail;
			}

			*(uint32_t *)(sprop->value + offset) =
				cpu_to_be32(phandle);
		}

		kfree(propval);
	}

merge_sym:

	of_node_put(root_sym);

	return 0;

err_fail:

	if (root_sym != NULL)
		of_node_put(root_sym);

	return err;
}
