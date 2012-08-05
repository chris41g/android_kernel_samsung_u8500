/*
 *
 * Sain touch driver
 *
 * Copyright (C) 2009 Sain, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __LINUX_ZINIX_TOUCH_H
#define __LINUX_ZINIX_TOUCH_H

#define ZINITIX_DRIVER_NAME	"zinitix_touch"
#define ZINITIX_ISP_NAME	"zinitix_isp"

/* max 8 */
#define	MAX_SUPPORTED_BUTTON_NUM		8

#define	TOUCH_V_FLIP	0x01
#define	TOUCH_H_FLIP	0x02
#define	TOUCH_XY_SWAP	0x04

struct _zinitix_reg_data {
	s16 reg_val;
	u8 valid;
};

/* Board specific touch screen initial values */
struct zinitix_touch_platform_data {
	const char *platform_name;	/* name to be used for inputs */
	u32	irq_gpio;	/* IRQ GPIO */
	u32	irq_trigger;	/* (IRQF_TRIGGER_LOW/FALLING) */
	u32	tsp_ldo_en;	/* power enable GPIO */
	u32	reset_gpio;	/* Reset GPIO, -1 if not used */
	u32	orientation;	/* to re-orientate screen coordinates */
	u32	x_max;		/* touch screen dimensions */
	u32	y_max;
	u32	num_buttons;	/* number of buttons supported */
		/* ordered mapping of buttons */
	u32	BUTTON_MAPPING_KEY[MAX_SUPPORTED_BUTTON_NUM];
	u32	num_regs;		/* number of registers on
					particular Zinitix controller */
		/* Pointer to table of register settings to configure
		controller for this product. */
	const struct _zinitix_reg_data *reg_data;
};

#endif /*  __LINUX_ZINITIX_TOUCH_H */
