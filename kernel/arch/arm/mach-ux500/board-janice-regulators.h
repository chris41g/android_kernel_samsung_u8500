/*
 * Copyright (C) 2009 ST-Ericsson SA
 * Copyright (C) 2010 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

#ifndef __BOARD_JANICE_REGULATORS_H
#define __BOARD_JANICE_REGULATORS_H

#include <linux/regulator/machine.h>
#include <linux/regulator/ab8500.h>

/* AB8500 regulators */
extern struct regulator_init_data janice_regulators[AB8500_NUM_REGULATORS];
extern struct ab8500_regulator_reg_init
	janice_ab8500_regulator_reg_init[AB8500_NUM_REGULATOR_REGISTERS];

/* U8500 specific regulators */
extern struct regulator_init_data janice_u8500_vape_regulator;
extern struct regulator_init_data janice_u8500_varm_regulator;
extern struct regulator_init_data janice_u8500_vmodem_regulator;
extern struct regulator_init_data janice_u8500_vpll_regulator;
extern struct regulator_init_data janice_u8500_vsmps1_regulator;
extern struct regulator_init_data janice_u8500_vsmps2_regulator;
extern struct regulator_init_data janice_u8500_vsmps3_regulator;
extern struct regulator_init_data janice_u8500_vrf1_regulator;

/* U8500 specific regulator switches */
extern struct regulator_init_data janice_u8500_svammdsp_regulator;
extern struct regulator_init_data janice_u8500_svammdspret_regulator;
extern struct regulator_init_data janice_u8500_svapipe_regulator;
extern struct regulator_init_data janice_u8500_siammdsp_regulator;
extern struct regulator_init_data janice_u8500_siammdspret_regulator;
extern struct regulator_init_data janice_u8500_siapipe_regulator;
extern struct regulator_init_data janice_u8500_sga_regulator;
extern struct regulator_init_data janice_u8500_b2r2_mcde_regulator;
extern struct regulator_init_data janice_u8500_esram12_regulator;
extern struct regulator_init_data janice_u8500_esram12ret_regulator;
extern struct regulator_init_data janice_u8500_esram34_regulator;
extern struct regulator_init_data janice_u8500_esram34ret_regulator;

#endif
