/*
 * This file is part of the coreboot project.
 *
 * Copyright 2013 Google Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef CPU_SAMSUNG_EXYNOS5420_H
#define CPU_SAMSUNG_EXYNOS5420_H

#include <soc/gpio.h>

struct soc_samsung_exynos5420_config {
	/* special magic numbers! */
	int clkval_f;
	int upper_margin;
	int lower_margin;
	int vsync;
	int left_margin;
	int right_margin;
	int hsync;

	int xres;
	int yres;
	int framebuffer_bits_per_pixel;

	int usb_vbus_gpio;
	int usb_hsic_gpio;

	u32 lcdbase;
};

#endif /* CPU_SAMSUNG_EXYNOS5420_H */
