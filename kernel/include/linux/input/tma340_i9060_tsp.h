/*
 * Samsung GT-I9060 Touchscreen panel driver.
 *
 * Author: Robert Teather  <robert.teather@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#ifndef _LINUX_TMA340_I9060_TSP_H
#define _LINUX_TMA340_I9060_TSP_H

#define TMA340_I9060_TSP_NAME "tma340-i9060-tsp"

struct tma340_tsp_platform_data {
	const char 		*reg_1v8;		/* NULL if no regulator used for TSP */
	const char		*reg_3v;
	int				ldo_gpio;		/* -1 if GPIO not used for LDO control */
	int				on_off_delay;	/* power on/off delay */
	int				watchdog;		/* 0 if no periodic checks on device */
	int				flipxy;			/* 0 if no flip, else invert X/Y inputs */
	unsigned int	max_x;
	unsigned int	max_y;
	int				key_led_gpio;	/* GPIO to control touchkey backlight */
	int				scl_gpio;		/* GPIO for I2C clock - used for firmware update */
	int				sda_gpio;		/* GPIO for I2C data - used for firmware update */
	void (*disable_i2c)(int disable);	/* func to disable I2C pins */
};

#endif /* _LINUX_TMA340_I9060_TSP_H */

