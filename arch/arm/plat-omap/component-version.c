/*
 *  linux/arch/arm/plat-omap/component-version.c
 *
 *  Copyright (C) 2005 Nokia Corporation
 *  Written by Juha Yrjölä <juha.yrjola@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/proc_fs.h>
#include <mach/board.h>
#include <mach/board-nokia.h>

static int component_version_read_proc(char *page, char **start, off_t off,
				       int count, int *eof, void *data)
{
	int len, i;
	const struct omap_version_config *ver;
	char *p;

	i = 0;
	p = page;
	while ((ver = omap_get_nr_config(OMAP_TAG_VERSION_STR,
					 struct omap_version_config, i)) != NULL) {
		p += sprintf(p, "%-12s%s\n", ver->component, ver->version);
		i++;
	}

	len = (p - page) - off;
	if (len < 0)
		len = 0;

	*eof = (len <= count) ? 1 : 0;
	*start = page + off;

	return len;
}

static int __init component_version_init(void)
{
	if (omap_get_config(OMAP_TAG_VERSION_STR, struct omap_version_config) == NULL)
		return -ENODEV;
	if (!create_proc_read_entry("component_version", S_IRUGO, NULL,
				    component_version_read_proc, NULL))
		return -ENOMEM;

	return 0;
}

static void __exit component_version_exit(void)
{
	remove_proc_entry("component_version", NULL);
}

late_initcall(component_version_init);
module_exit(component_version_exit);

MODULE_AUTHOR("Juha Yrjölä <juha.yrjola@nokia.com>");
MODULE_DESCRIPTION("Component version driver");
MODULE_LICENSE("GPL");
