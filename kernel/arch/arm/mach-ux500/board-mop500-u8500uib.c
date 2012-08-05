/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * License terms: GNU General Public License (GPL), version 2
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/lsm303dlh.h>
#include <linux/l3g4200d.h>
#include <plat/pincfg.h>
#include "pins-db8500.h"

#include "board-mop500.h"

/* For U8500 UIB */
static struct lsm303dlh_platform_data __initdata lsm303dlh_pdata_u8500_uib = {
	.name_a = "lsm303dlh.0",
	.name_m = "lsm303dlh.1",
	.axis_map_x = 1,
	.axis_map_y = 0,
	.axis_map_z = 2,
	.negative_x = 1,
	.negative_y = 1,
	.negative_z = 1,
};

static struct l3g4200d_gyr_platform_data  __initdata l3g4200d_pdata_u8500_uib = {
	.name_gyr = "l3g4200d",
	.axis_map_x = 1,
	.axis_map_y = 0,
	.axis_map_z = 2,
	.negative_x = 0,
	.negative_y = 0,
	.negative_z = 1,
};
static struct i2c_board_info __initdata mop500_i2c2_devices_u8500_uib[] = {
	{
		/* LSM303DLH Accelerometer */
		I2C_BOARD_INFO("lsm303dlh_a", 0x18),
		.platform_data = &lsm303dlh_pdata_u8500_uib,
	},
	{
		/* LSM303DLH Magnetometer */
		I2C_BOARD_INFO("lsm303dlh_m", 0x1E),
		.platform_data = &lsm303dlh_pdata_u8500_uib,
	},
	{
		/* L3G4200D Gyroscope */
		I2C_BOARD_INFO("l3g4200d", 0x68),
		.platform_data = &l3g4200d_pdata_u8500_uib,
	},
};

void __init mop500_u8500uib_init(void)
{
	if (machine_is_hrefv60())	{
		lsm303dlh_pdata_u8500_uib.irq_a1 = HREFV60_ACCEL_INT1_GPIO;
		lsm303dlh_pdata_u8500_uib.irq_a2 = HREFV60_ACCEL_INT2_GPIO;
		lsm303dlh_pdata_u8500_uib.irq_m = HREFV60_MAGNET_DRDY_GPIO;
	} else	{
		lsm303dlh_pdata_u8500_uib.irq_a1 = EGPIO_PIN_10;
		lsm303dlh_pdata_u8500_uib.irq_a2 = EGPIO_PIN_11;
		lsm303dlh_pdata_u8500_uib.irq_m = EGPIO_PIN_1;
	}
	mop500_uib_i2c_add(2, mop500_i2c2_devices_u8500_uib,
			ARRAY_SIZE(mop500_i2c2_devices_u8500_uib));
}
