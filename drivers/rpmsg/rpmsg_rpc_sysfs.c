// SPDX-License-Identifier: GPL-2.0
/*
 * Remote Processor Procedure Call Driver
 *
 * Copyright (C) 2012-2019 Texas Instruments Incorporated - http://www.ti.com/
 *	Erik Rainey <erik.rainey@ti.com>
 *	Suman Anna <s-anna@ti.com>
 */

#include <linux/device.h>
#include <linux/slab.h>
#include <linux/rpmsg_rpc.h>

#include "rpmsg_rpc_internal.h"

static ssize_t show_numfuncs(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct rppc_device *rppcdev = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", rppcdev->num_funcs - 1);
}

static ssize_t set_type_c(char *buf, uint32_t len,
			  struct rppc_param_signature *psig)
{
	char *isptr = (psig->type & RPPC_PARAM_PTR ? " *" : "");

	switch (psig->type & RPPC_PARAM_MASK) {
	case RPPC_PARAM_S08:
		return snprintf(buf, len, "int8_t%s", isptr);
	case RPPC_PARAM_U08:
		return snprintf(buf, len, "uint8_t%s", isptr);
	case RPPC_PARAM_S16:
		return snprintf(buf, len, "int16_t%s", isptr);
	case RPPC_PARAM_U16:
		return snprintf(buf, len, "uint16_t%s", isptr);
	case RPPC_PARAM_S32:
		return snprintf(buf, len, "int32_t%s", isptr);
	case RPPC_PARAM_U32:
		return snprintf(buf, len, "uint32_t%s", isptr);
	case RPPC_PARAM_S64:
		return snprintf(buf, len, "int64_t%s", isptr);
	case RPPC_PARAM_U64:
		return snprintf(buf, len, "uint64_t%s", isptr);
	default:
		return snprintf(buf, len, "<unknown>%s", isptr);
	}
}

static ssize_t set_type_doxy(char *buf, uint32_t len,
			     struct rppc_param_signature *psig)
{
	char *isptr = (psig->type & RPPC_PARAM_PTR ? " *" : "");
	char dir[10];

	switch (psig->direction) {
	case RPPC_PARAMDIR_IN:
		snprintf(dir, sizeof(dir), "[in]");
		break;
	case RPPC_PARAMDIR_OUT:
		snprintf(dir, sizeof(dir), "[out]");
		break;
	case RPPC_PARAMDIR_BI:
		snprintf(dir, sizeof(dir), "[in,out]");
		break;
	default:
		snprintf(dir, sizeof(dir), "[unknown]");
		break;
	}

	switch (psig->type & RPPC_PARAM_MASK) {
	case RPPC_PARAM_S08:
		return snprintf(buf, len, "%s int8_t%s", dir, isptr);
	case RPPC_PARAM_U08:
		return snprintf(buf, len, "%s uint8_t%s", dir, isptr);
	case RPPC_PARAM_S16:
		return snprintf(buf, len, "%s int16_t%s", dir, isptr);
	case RPPC_PARAM_U16:
		return snprintf(buf, len, "%s uint16_t%s", dir, isptr);
	case RPPC_PARAM_S32:
		return snprintf(buf, len, "%s int32_t%s", dir, isptr);
	case RPPC_PARAM_U32:
		return snprintf(buf, len, "%s uint32_t%s", dir, isptr);
	case RPPC_PARAM_S64:
		return snprintf(buf, len, "%s int64_t%s", dir, isptr);
	case RPPC_PARAM_U64:
		return snprintf(buf, len, "%s uint64_t%s", dir, isptr);
	default:
		return snprintf(buf, len, "%s <unknown>%s", dir, isptr);
	}
}

static ssize_t show_c_function(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct rppc_device *rppcdev = dev_get_drvdata(dev);
	char return_value[11]; /* longest string is strlen("uintXX_t *") = 10 */
	char parameters[110]; /* longest string * 10 + 9(,) */
	char comment[300];
	int p;
	ssize_t pidx = 0;
	ssize_t cidx = 0;
	__u32 index = 0;

	if (sscanf(attr->attr.name, "c_function%u\n", &index) != 1)
		return -EIO;

	memset(return_value, 0, sizeof(return_value));
	memset(parameters, 0, sizeof(parameters));

	strcpy(return_value, "void");
	strcpy(parameters, "void");
	cidx += snprintf(&comment[cidx], sizeof(comment) - cidx, "/**\n");
	cidx += snprintf(&comment[cidx], sizeof(comment) - cidx,
		" * \\fn %s\n", rppcdev->signatures[index].name);
	for (p = 0; p < rppcdev->signatures[index].num_param; p++) {
		if (p == 0) {
			set_type_c(return_value, sizeof(return_value),
				   &rppcdev->signatures[index].params[0]);
			cidx += snprintf(&comment[cidx], sizeof(comment) - cidx,
					" * \\return %s\n", return_value);
		} else {
			pidx += set_type_c(&parameters[pidx],
					sizeof(parameters) - pidx,
					&rppcdev->signatures[index].params[p]);
			if (p != rppcdev->signatures[index].num_param - 1)
				parameters[pidx++] = ',';
			cidx += snprintf(&comment[cidx], sizeof(comment) - cidx,
						" * \\param ");
			cidx += set_type_doxy(&comment[cidx],
					sizeof(comment) - cidx,
					&rppcdev->signatures[index].params[p]);
			cidx += snprintf(&comment[cidx], sizeof(comment) - cidx,
						"\n");
		}
	}
	if (p <= 1)
		pidx += strlen("void");
	if (pidx < sizeof(parameters))
		parameters[pidx] = '\0';
	cidx += snprintf(&comment[cidx], sizeof(comment) - cidx, " */");
	return snprintf(buf, PAGE_SIZE, "%s\nextern \"C\" %s %s(%s);\n",
			comment, return_value, rppcdev->signatures[index].name,
			parameters);
}

static struct device_attribute rppc_attrs[] = {
	__ATTR(num_funcs, 0444, show_numfuncs, NULL),
};

/**
 * rppc_create_sysfs - Creates the sysfs entry structures for the instance
 * @rppcdev: the rppc device (remote server instance) handle
 *
 * Helper function to create all the sysfs entries associated with a rppc
 * device. Each device is associated with a number of remote procedure
 * functions. The number of such functions and the signatures of those
 * functions are created in sysfs. Function is invoked after querying
 * the remote side about the supported functions on this device.
 *
 * The entries are split into a set of static entries, which are common
 * between all rppc devices, and a set of dynamic entries specific to
 * each rppc device.
 *
 * Return: 0 on success, or an appropriate error code otherwise
 */
int rppc_create_sysfs(struct rppc_device *rppcdev)
{
	int i;
	int ret;

	rppcdev->sig_attr = kcalloc(rppcdev->num_funcs,
				    sizeof(*rppcdev->sig_attr), GFP_KERNEL);
	if (!rppcdev->sig_attr)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(rppc_attrs); i++) {
		ret = device_create_file(rppcdev->dev, &rppc_attrs[i]);
		if (ret) {
			dev_err(rppcdev->dev, "failed to create sysfs entry\n");
			goto clean_static_entries;
		}
	}

	for (i = 1; i < rppcdev->num_funcs; i++) {
		sysfs_attr_init(&rppcdev->sig_attr[i].attr);
		rppcdev->sig_attr[i].attr.name =
				kzalloc(RPPC_MAX_FUNC_NAMELEN, GFP_KERNEL);
		if (!rppcdev->sig_attr[i].attr.name) {
			ret = -ENOMEM;
			goto clean_dynamic_entries;
		}
		snprintf((char *)rppcdev->sig_attr[i].attr.name,
			 RPPC_MAX_FUNC_NAMELEN, "c_function%u", i);
		rppcdev->sig_attr[i].attr.mode = 0444;
		rppcdev->sig_attr[i].show = show_c_function;
		rppcdev->sig_attr[i].store = NULL;

		ret = device_create_file(rppcdev->dev, &rppcdev->sig_attr[i]);
		if (ret) {
			dev_err(rppcdev->dev, "failed to create sysfs function entry (%d)\n",
				ret);
			goto clean_dynamic_entries;
		}
	}
	return 0;

clean_dynamic_entries:
	while (i-- > 1) {
		device_remove_file(rppcdev->dev, &rppcdev->sig_attr[i]);
		kfree(rppcdev->sig_attr[i].attr.name);
	}
	i = ARRAY_SIZE(rppc_attrs);
clean_static_entries:
	while (i-- > 0)
		device_remove_file(rppcdev->dev, &rppc_attrs[i]);
	kfree(rppcdev->sig_attr);
	return ret;
}

/**
 * rppc_remove_sysfs: Removes the sysfs entry structures for the instance
 * @rppcdev: the rppc device (remote server instance) handle
 *
 * Helper function to remove all the sysfs entries associated with the
 * rppc device.
 *
 * Return: 0 on success, or an appropriate error code otherwise
 */
int rppc_remove_sysfs(struct rppc_device *rppcdev)
{
	int i;

	if (rppcdev->sig_attr) {
		for (i = 1; i < rppcdev->num_funcs; i++) {
			device_remove_file(rppcdev->dev, &rppcdev->sig_attr[i]);
			kfree(rppcdev->sig_attr[i].attr.name);
		}
	}
	kfree(rppcdev->sig_attr);

	for (i = 0; i < ARRAY_SIZE(rppc_attrs); i++)
		device_remove_file(rppcdev->dev, &rppc_attrs[i]);

	return 0;
}
