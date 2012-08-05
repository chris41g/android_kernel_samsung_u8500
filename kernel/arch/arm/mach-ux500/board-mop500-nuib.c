/*
 * Copyright (C) 2010 ST-Ericsson SA
 *
 * License terms:GNU General Public License (GPL) version 2
 */

#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input/synaptics_i2c_rmi4.h>

#include "board-mop500.h"

/* Descriptor structure.
 * Describes the number of i2c devices on the bus that speak RMI.
 */
static struct synaptics_rmi4_platform_data rmi4_i2c_dev_platformdata = {
	.irq_number	= GPIO_TO_IRQ(84),
	.irq_type	= (IRQF_TRIGGER_FALLING | IRQF_SHARED),
#if defined(CONFIG_DISPLAY_GENERIC_DSI_PRIMARY_ROTATION_ANGLE) &&	\
		CONFIG_DISPLAY_GENERIC_DSI_PRIMARY_ROTATION_ANGLE == 270
	.x_flip		= true,
	.y_flip		= false,
#else
	.x_flip		= false,
	.y_flip		= true,
#endif
	.regulator_en	= true,
};

static struct i2c_board_info __initdata u8500_i2c3_devices_nuib[] = {
	{
		/* Touschscreen */
		I2C_BOARD_INFO("synaptics_rmi4_i2c", 0x4B),
		.platform_data = &rmi4_i2c_dev_platformdata,
	},
};

void __init mop500_nuib_init(void)
{
	i2c_register_board_info(3, u8500_i2c3_devices_nuib,
				ARRAY_SIZE(u8500_i2c3_devices_nuib));
}
