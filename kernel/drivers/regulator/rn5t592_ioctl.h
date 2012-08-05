/*
 * Linux device driver for RN5T592 power management IC
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2011 Samsung Electronics Co. Ltd
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


#ifndef _RN5T592_IOCTL_h_
#define	_RN5T592_IOCTL_h_


#include <linux/ioctl.h>	/* _IO() */


/* ioctl numbers for the RN5T592 miscellaneous device driver */

#define	RN5T592_DEVNAME		"rn5t592"

#define	RN5T592_IOC_MAGIC	'Z'

#define	RN5T592_IOC_TURN_ON	_IO(RN5T592_IOC_MAGIC, 0)
#define	RN5T592_IOC_TURN_OFF	_IO(RN5T592_IOC_MAGIC, 1)



#endif
