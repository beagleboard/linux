/*
 * Utility functions for working with device tree(s)
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
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/err.h>

/**
 * __of_free_property - release the memory of an allocated property
 * @prop:	Property to release
 *
 * Release the memory of an allocated property only after checking
 * that the property has been marked as OF_DYNAMIC.
 * Only call on known allocated properties.
 */
void __of_free_property(struct property *prop)
{
	if (prop == NULL)
		return;

	if (of_property_check_flag(prop, OF_DYNAMIC)) {
		kfree(prop->value);
		kfree(prop->name);
		kfree(prop);
	} else {
		pr_warn("%s: property %p cannot be freed; memory is gone\n",
				__func__, prop);
	}
}

/**
 * __of_free_tree - release the memory of a device tree node and
 *		    of all it's children + properties.
 * @node:	Device Tree node to release
 *
 * Release the memory of a device tree node and of all it's children.
 * Also release the properties and the dead properties.
 * Only call on detached node trees, and you better be sure that
 * no pointer exist for any properties. Only safe to do if you 
 * absolutely control the life cycle of the node.
 * Also note that the node is not removed from the all_nodes list,
 * neither from the parent's child list; this should be handled before
 * calling this function.
 */
void __of_free_tree(struct device_node *node)
{
	struct property *prop;
	struct device_node *noden;

	/* sanity check */
	if (!node)
		return;

	/* free recursively any children */
	while ((noden = node->child) != NULL) {
		node->child = noden->sibling;
		__of_free_tree(noden);
	}

	/* free every property already allocated */
	while ((prop = node->properties) != NULL) {
		node->properties = prop->next;
		__of_free_property(prop);
	}

	/* free dead properties already allocated */
	while ((prop = node->deadprops) != NULL) {
		node->deadprops = prop->next;
		__of_free_property(prop);
	}

	if (of_node_check_flag(node, OF_DYNAMIC)) {
		kfree(node->type);
		kfree(node->name);
		kfree(node);
	} else {
		pr_warn("%s: node %p cannot be freed; memory is gone\n",
				__func__, node);
	}
}

/**
 * __of_copy_property - Copy a property dynamically.
 * @prop:	Property to copy
 * @flags:	Allocation flags (typically pass GFP_KERNEL)
 *
 * Copy a property by dynamically allocating the memory of both the
 * property stucture and the property name & contents. The property's
 * flags have the OF_DYNAMIC bit set so that we can differentiate between
 * dynamically allocated properties and not.
 * Returns the newly allocated property or NULL on out of memory error.
 */
struct property *__of_copy_property(const struct property *prop, gfp_t flags)
{
	struct property *propn;

	propn = kzalloc(sizeof(*prop), flags);
	if (propn == NULL)
		return NULL;

	propn->name = kstrdup(prop->name, flags);
	if (propn->name == NULL)
		goto err_fail_name;

	if (prop->length > 0) {
		propn->value = kmalloc(prop->length, flags);
		if (propn->value == NULL)
			goto err_fail_value;
		memcpy(propn->value, prop->value, prop->length);
		propn->length = prop->length;
	}

	/* mark the property as dynamic */
	of_property_set_flag(propn, OF_DYNAMIC);

	return propn;

err_fail_value:
	kfree(propn->name);
err_fail_name:
	kfree(propn);
	return NULL;
}

/**
 * __of_create_empty_node - Create an empty device node dynamically.
 * @name:	Name of the new device node
 * @type:	Type of the new device node
 * @full_name:	Full name of the new device node
 * @phandle:	Phandle of the new device node
 * @flags:	Allocation flags (typically pass GFP_KERNEL)
 *
 * Create an empty device tree node, suitable for further modification.
 * The node data are dynamically allocated and all the node flags
 * have the OF_DYNAMIC & OF_DETACHED bits set.
 * Returns the newly allocated node or NULL on out of memory error.
 */
struct device_node *__of_create_empty_node(
		const char *name, const char *type, const char *full_name,
		phandle phandle, gfp_t flags)
{
	struct device_node *node;

	node = kzalloc(sizeof(*node), flags);
	if (node == NULL)
		return NULL;

	node->name = kstrdup(name, flags);
	if (node->name == NULL)
		goto err_return;

	node->type = kstrdup(type, flags);
	if (node->type == NULL)
		goto err_return;

	node->full_name = kstrdup(full_name, flags);
	if (node->type == NULL)
		goto err_return;

	node->phandle = phandle;
	kref_init(&node->kref);
	of_node_set_flag(node, OF_DYNAMIC);
	of_node_set_flag(node, OF_DETACHED);

	return node;

err_return:
	__of_free_tree(node);
	return NULL;
}

/**
 * __of_find_node_by_full_name - Find a node with the full name recursively
 * @node:	Root of the tree to perform the search
 * @full_name:	Full name of the node to find.
 *
 * Find a node with the give full name by recursively following any of 
 * the child node links.
 * Returns the matching node, or NULL if not found.
 * Note that the devtree lock is not taken, so this function is only
 * safe to call on either detached trees, or when devtree lock is already
 * taken.
 */
struct device_node *__of_find_node_by_full_name(struct device_node *node,
		const char *full_name)
{
	struct device_node *child, *found;

	if (node == NULL)
		return NULL;

	/* check */
	if (of_node_cmp(node->full_name, full_name) == 0)
		return node;

	__for_each_child_of_node(node, child) {
		found = __of_find_node_by_full_name(child, full_name);
		if (found != NULL)
			return found;
	}

	return NULL;
}

/**
 * of_multi_prop_cmp - Check if a property matches a value
 * @prop:	Property to check
 * @value:	Value to check against
 *
 * Check whether a property matches a value, using the standard
 * of_compat_cmp() test on each string. It is similar to the test
 * of_device_is_compatible() makes, but it can be performed without
 * taking the devtree_lock, which is required in some cases.
 * Returns 0 on a match, -1 on no match.
 */
int of_multi_prop_cmp(const struct property *prop, const char *value)
{
	const char *cp;
	int cplen, vlen, l;

	if (prop == NULL || value == NULL)
		return -1;

	cp = prop->value;
	cplen = prop->length;
	vlen = strlen(value);

	while (cplen > 0) {
		if (of_compat_cmp(cp, value, vlen) == 0)
			return 0;
		l = strlen(cp) + 1;
		cp += l;
		cplen -= l;
	}

	return -1;
}

