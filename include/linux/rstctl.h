/*
 * Reset control subsystem
 *
 * Copyright (C) 2013 Pantelis Antoniou <panto@antoniou-consulting.com>
 *
 * License terms: GNU General Public License (GPL) version 2
 */
#ifndef __LINUX_RSTCTL_H
#define __LINUX_RSTCTL_H

#include <linux/of.h>
#include <linux/list.h>

struct rstctl_dev;

struct rstctl_line {
	const char *name;
	void *data;
};

struct rstctl_ops {
	int (*request)(struct rstctl_dev *rdev,
			const struct rstctl_line *line);
	int (*release)(struct rstctl_dev *rdev,
			const struct rstctl_line *line);
	int (*assert)(struct rstctl_dev *rdev,
			const struct rstctl_line *line);
	int (*deassert)(struct rstctl_dev *rdev,
			const struct rstctl_line *line);
	int (*pulse)(struct rstctl_dev *rdev,
			const struct rstctl_line *line,
			unsigned long hold_ns);
	struct module *owner;
};

struct rstctl_desc {
	const char *name;
	const struct rstctl_ops *ops;
	int nlines;
	const struct rstctl_line *lines;
};

struct rstctl_dev {
	struct list_head node;
	struct device *dev;
	const struct rstctl_desc *rdesc;
	struct list_head handles;
};

struct rstctl {
	struct list_head node;		/* linked all in */
	struct device *dev; 		/* the user */
	struct rstctl_dev *rdev;	/* the controler */
	const struct rstctl_line *line;
	const char *label;
};

/* driver API */
struct rstctl_dev *rstctl_register(struct device *dev,
		const struct rstctl_desc *rdesc);
int rstctl_unregister(struct rstctl_dev *rdev);

/* consumer API */
struct rstctl *rstctl_get(struct device *dev, const char *id);
void rstctl_put(struct rstctl *rctrl);

int rstctl_assert(struct rstctl *rctrl);
int rstctl_deassert(struct rstctl *rctrl);
int rstctl_pulse(struct rstctl *rctrl, unsigned long hold_ns);

#endif
