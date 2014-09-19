/*
 * Include file for the BeagleLogic kernel module [glue code for pru_rproc
 *
 * Copyright (C) 2014 Kumar Abhishek <abhishek@theembeddedkitchen.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#ifndef RPROC_EXTERNAL_GLUE_H_
#define RPROC_EXTERNAL_GLUE_H_

struct pru_rproc_external_glue {
	int (*downcall_idx)(int, u32, u32, u32, u32, u32, u32);
};

ssize_t pru_external_load(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);

ssize_t pru_external_reset(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);

int pruproc_external_request_bind(struct pru_rproc_external_glue *g);

#endif /* RPROC_EXTERNAL_GLUE_H_ */
