/*
 * Copyright (C) 2007 STMicroelectronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __HSIDEV_H__
#define __HSIDEV_H__

#ifdef __KERNEL__
#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/wait.h>
#include <mach/debug.h>

#define MAX_CHANNELS_MONITORED 4

#define DRIVER_NAME            "HSI LOOPBACK"
/* enables/disables debug msgs */
#define DRIVER_DEBUG            0
/* msg header represents this module */
#define DRIVER_DEBUG_PFX        DRIVER_NAME
#define DRIVER_DBG              KERN_ERR

/** struct hsidev_data - HSI loopback slave device
 * @devt - Device major and minor number for each channel
 * @hsidev_lock - Lock protecting the channel from simultaneous
 * 		 operations.
 * @wq - Wait queue for processed to sleep for event arrival
 * @hsi - Slave hsi device representing the channel
 * @device_entry - To enable insertion into loopback list
 * @xfer - Structure representing each transfer
 * @users - Number of users currently
 * @xfer_done - Flag to indicate trnasfer completion
**/
struct hsidev_data {
	dev_t devt;
	struct mutex hsidev_lock;
	wait_queue_head_t wq;
	struct hsi_device *hsi;
	struct list_head device_entry;
	struct hsi_data *xfer;
	unsigned users;
	int xfer_done;
};

#endif

#endif /* __HSIDEV_H__ */
