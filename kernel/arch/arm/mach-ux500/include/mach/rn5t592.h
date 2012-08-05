/*
 * RN5T592 platform-specific data.
 *
 * Copyright (c) Samsung 2011
 */

#ifndef _RN5T592_h_
#define	_RN5T592_h_


#define	RN5T592_I2C_DEVICE_NAME	"rn5t592"


struct rn5t592_platform_data
{
	unsigned int	subpmu_pwron_gpio;
};


#endif
