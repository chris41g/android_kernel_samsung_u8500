/*
 *  Copyright (C) ST-Ericsson SA 2011
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/gfp.h>
#include <linux/err.h>
#include <linux/cw1200-wlan.h>
#include <linux/module.h>

static const struct wlan1200_platform_data *platform_data;

void wlan1200_set_platdata(struct wlan1200_platform_data *pdata)
{
	/*
	 * note, if we get a failure in allocation, we simply drop out of the
	 * function. If there is so little memory available at initialisation
	 * time then there is little chance the system is going to run.
	*/

	platform_data = kmemdup(pdata, sizeof(struct wlan1200_platform_data), GFP_KERNEL);
	if (!platform_data) {
		printk(KERN_ERR "%s: failed copying platform data\n", __func__);
		return;
	}
}
EXPORT_SYMBOL(wlan1200_set_platdata);

const struct wlan1200_platform_data *wlan1200_get_platform_data(void)
{
       if (!platform_data)
               return ERR_PTR(-ENODEV);

       return platform_data;
}
EXPORT_SYMBOL(wlan1200_get_platform_data);

