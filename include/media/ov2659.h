/*
 * OV2659 camera sensors driver
 *
 * Copyright (C) 2013 Benoit Parrot <bparrot@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef OV2659_H_
#define OV2659_H_

/**
 * struct ov2659_platform_data - ov2659 driver platform data
 * @mclk_frequency: the sensor's master clock frequency in Hz
 *
 * @mclk_frequency must always be specified.
 */
struct ov2659_platform_data {
	unsigned int mclk_frequency;
};
#endif /* OV2659_H_ */
