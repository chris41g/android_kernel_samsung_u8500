/*
 * Copyright (C) ST-Ericsson SA 2011
 *
 * License Terms: GNU General Public License, version 2
 * Author: Atul Dahiya <atul.dahiya@stericsson.com> for ST-Ericsson
 */

#ifndef __CW1200_WLAN_H
#define __CW1200_WLAN_H

/**
 * struct wlan1200_platform_data - data structure for platform specific data
 * @gpio_irq:		irq no,
 * @gpio_enable		gpio to enable
 */
struct wlan1200_platform_data {
	int			gpio_irq;
	int			gpio_enable;
};

extern void wlan1200_set_platdata(struct wlan1200_platform_data *pdata);
extern const struct wlan1200_platform_data *wlan1200_get_platform_data(void);

#endif	/*CW1200_WLAN_H*/

