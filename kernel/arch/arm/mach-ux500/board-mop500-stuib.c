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
#include <plat/pincfg.h>
#include "pins-db8500.h"


#include "board-mop500.h"

static struct lsm303dlh_platform_data __initdata lsm303dlh_pdata_st_uib = {
	.name_a = "lsm303dlh.0",
	.name_m = "lsm303dlh.1",
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negative_x = 1,
	.negative_y = 1,
	.negative_z = 0,
};

static struct i2c_board_info __initdata mop500_i2c2_devices_st_uib[] = {
	{
		/* LSM303DLH Accelerometer */
		I2C_BOARD_INFO("lsm303dlh_a", 0x18),
		.platform_data = &lsm303dlh_pdata_st_uib,
	},
	{
		/* LSM303DLH Magnetometer */
		I2C_BOARD_INFO("lsm303dlh_m", 0x1E),
		.platform_data = &lsm303dlh_pdata_st_uib,
	},
};

void __init mop500_stuib_init(void)
{
	if (machine_is_hrefv60())	{
		lsm303dlh_pdata_st_uib.irq_a1 = HREFV60_ACCEL_INT1_GPIO;
		lsm303dlh_pdata_st_uib.irq_a2 = HREFV60_ACCEL_INT2_GPIO;
		lsm303dlh_pdata_st_uib.irq_m = HREFV60_MAGNET_DRDY_GPIO;
	} else	{
		lsm303dlh_pdata_st_uib.irq_a1 = EGPIO_PIN_10;
		lsm303dlh_pdata_st_uib.irq_a2 = EGPIO_PIN_11;
		lsm303dlh_pdata_st_uib.irq_m = EGPIO_PIN_1;
	}
	mop500_uib_i2c_add(2, mop500_i2c2_devices_st_uib,
			ARRAY_SIZE(mop500_i2c2_devices_st_uib));
}
