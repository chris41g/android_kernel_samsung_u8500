/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * License terms: GNU General Public License (GPL) version 2
 */

#ifndef __BOARD_MOP500_H
#define __BOARD_MOP500_H

#define HREFV60_TOUCH_RST_GPIO 143
#define HREFV60_PROX_SENSE_GPIO 217
#define HREFV60_HAL_SW_GPIO 145
#define HREFV60_SDMMC_EN_GPIO 169
#define HREFV60_SDMMC_1V8_3V_GPIO 5
#define HREFV60_SDMMC_CD_GPIO 95
#define HREFV60_ACCEL_INT1_GPIO 82
#define HREFV60_ACCEL_INT2_GPIO 83
#define HREFV60_MAGNET_DRDY_GPIO 32
#define HREFV60_DISP1_RST_GPIO	65
#define HREFV60_DISP2_RST_GPIO	66
#define HREFV60_MMIO_XENON_CHARGE 170
#define HREFV60_XSHUTDOWN_SECONDARY_SENSOR 140
#define XSHUTDOWN_PRIMARY_SENSOR 141
#define XSHUTDOWN_SECONDARY_SENSOR 142
struct i2c_board_info;
void mop500_nuib_init(void);
void __init mop500_u8500uib_init(void);
void __init mop500_stuib_init(void);
void mop500_uib_i2c_add(int busnum, struct i2c_board_info const *info,
		unsigned n);
void mop500_msp_init(void);
void __init mop500_pins_init(void);
void __init mop500_vibra_init(void);

int msp13_i2s_init(void);
int msp13_i2s_exit(void);

#endif
