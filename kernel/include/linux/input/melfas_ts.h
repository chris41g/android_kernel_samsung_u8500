/*
 * include/linux/melfas_ts.h - platform data structure for MCS Series sensor
 *
 * Copyright (C) 2010 Melfas, Inc.
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

#ifndef _LINUX_MELFAS_TS_H
#define _LINUX_MELFAS_TS_H

#define MELFAS_TS_NAME "melfas-ts"

#define MELFAS_MAX_KEYS       5
struct melfas_touchkey {
	int				key_id;
	unsigned int	code;
	const char 		*name;
};

struct melfas_tsi_platform_data {
	int x_size;
	int y_size;
	int version;

	int num_keys;
	struct melfas_touchkey touchkeys[MELFAS_MAX_KEYS];

	const char 	*regulator;
	int		pwr_en_gpio;
	int		key_bl_en_gpio;

	/* for firmware download */
	int	sda_gpio;
	int	scl_gpio;
};

#endif /* _LINUX_MELFAS_TS_H */
