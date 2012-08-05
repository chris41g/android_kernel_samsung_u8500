/*
 * GP2A proximity sensor platform-specific data.
 *
 * Copyright (c) Samsung 2010
 */

#ifndef _GP2A_h_
#define	_GP2A_h_


#define	GP2A_I2C_DEVICE_NAME	"gp2a_prox"



struct gp2a_platform_data
{
	unsigned int	ps_vout_gpio;
	int	alsout;
	int (* hw_setup)( void );
	int (* hw_teardown)( void );
};


#endif
