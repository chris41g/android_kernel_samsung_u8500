/*
 * Copyright (C) ST-Ericsson SA 2011
 *
 * License terms:  GNU General Public License (GPL), version 2
 *
 * U8500 board specific charger and battery initialization parameters.
 *
 * Author: Johan Palsson <johan.palsson@stericsson.com> for ST-Ericsson.
 * Author: Johan Gardsmark <johan.gardsmark@stericsson.com> for ST-Ericsson.
 *
 */

#include <linux/power_supply.h>
#include <linux/mfd/ab8500/ab8500-bm.h>
#include "board-sec-bm.h"

#ifdef CONFIG_AB8500_BATTERY_THERM_ON_BATCTRL
/*
 * These are the defined batteries that uses a NTC and ID resistor placed
 * inside of the battery pack.
 * Note that the res_to_temp table must be strictly sorted by falling resistance
 * values to work.
 */
static struct res_to_temp temp_tbl_A[] = {
	{-5, 53407},
	{ 0, 48594},
	{ 5, 43804},
	{10, 39188},
	{15, 34870},
	{20, 30933},
	{25, 27422},
	{30, 24347},
	{35, 21694},
	{40, 19431},
	{45, 17517},
	{50, 15908},
	{55, 14561},
	{60, 13437},
	{65, 12500},
};
static struct res_to_temp temp_tbl_B[] = {
	{-5, 165418},
	{ 0, 159024},
	{ 5, 151921},
	{10, 144300},
	{15, 136424},
	{20, 128565},
	{25, 120978},
	{30, 113875},
	{35, 107397},
	{40, 101629},
	{45,  96592},
	{50,  92253},
	{55,  88569},
	{60,  85461},
	{65,  82869},
};
static struct v_to_cap cap_tbl_A[] = {
	{4171,	100},
	{4114,	 95},
	{4009,	 83},
	{3947,	 74},
	{3907,	 67},
	{3863,	 59},
	{3830,	 56},
	{3813,	 53},
	{3791,	 46},
	{3771,	 33},
	{3754,	 25},
	{3735,	 20},
	{3717,	 17},
	{3681,	 13},
	{3664,	  8},
	{3651,	  6},
	{3635,	  5},
	{3560,	  3},
	{3408,    1},
	{3247,	  0},
};
static struct v_to_cap cap_tbl_B[] = {
	{4161,	100},
	{4124,	 98},
	{4044,	 90},
	{4003,	 85},
	{3966,	 80},
	{3933,	 75},
	{3888,	 67},
	{3849,	 60},
	{3813,	 55},
	{3787,	 47},
	{3772,	 30},
	{3751,	 25},
	{3718,	 20},
	{3681,	 16},
	{3660,	 14},
	{3589,	 10},
	{3546,	  7},
	{3495,	  4},
	{3404,	  2},
	{3250,	  0},
};
#endif
static struct v_to_cap cap_tbl[] = {
	{4186,	100},
	{4163,	 99},
	{4114,	 95},
	{4068,	 90},
	{3990,	 80},
	{3926,	 70},
	{3898,	 65},
	{3866,	 60},
	{3833,	 55},
	{3812,	 50},
	{3787,	 40},
	{3768,	 30},
	{3747,	 25},
	{3730,	 20},
	{3705,	 15},
	{3699,	 14},
	{3684,	 12},
	{3672,	  9},
	{3657,	  7},
	{3638,	  6},
	{3556,	  4},
	{3424,	  2},
	{3317,	  1},
	{3094,	  0},
};

/*
 * Note that the res_to_temp table must be strictly sorted by falling
 * resistance values to work.
 */
static struct res_to_temp temp_tbl[] = {
	{-5, 214834},
	{ 0, 162943},
	{ 5, 124820},
	{10,  96520},
	{15,  75306},
	{20,  59254},
	{25,  47000},
	{30,  37566},
	{35,  30245},
	{40,  24520},
	{45,  20010},
	{50,  16432},
	{55,  13576},
	{60,  11280},
	{65,   9425},
};

static struct res_to_temp adc_temp_tbl[] = {
	{-10, 650},
	{-5, 569},
	{ 0, 483},
	{ 5, 406},
	{10, 310},
	{15, 282},
	{20, 236},
	{25, 192},
	{30, 161},
	{35, 135},
	{40, 112},
	{45, 92},
	{50, 78},
	{55, 65},
	{60, 55},
	{65, 45},
};

static const struct battery_type bat_type[] = {
	[BATTERY_UNKNOWN] = {
		/* First element always represent the UNKNOWN battery */
		.name = POWER_SUPPLY_TECHNOLOGY_UNKNOWN,
		.resis_high = 0,
		.resis_low = 0,
		.battery_resistance = 300,
		.charge_full_design = 612,
		.nominal_voltage = 3700,
		.termination_vol = 4050,
		.termination_curr_1st = 240,
		.termination_curr_2nd = 200,
		.recharge_vol = 3990,
		.normal_cur_lvl = 400,
		.normal_vol_lvl = 4100,
		.maint_a_cur_lvl = 400,
		.maint_a_vol_lvl = 4050,
		.maint_a_chg_timer_h = 60,
		.maint_b_cur_lvl = 400,
		.maint_b_vol_lvl = 4000,
		.maint_b_chg_timer_h = 200,
		.low_high_cur_lvl = 300,
		.low_high_vol_lvl = 4000,
#ifdef CONFIG_MEASURE_TEMP_BY_ADC_TABLE
		.n_temp_tbl_elements = ARRAY_SIZE(adc_temp_tbl),
		.r_to_t_tbl = adc_temp_tbl,
#else
		.n_temp_tbl_elements = ARRAY_SIZE(temp_tbl),
		.r_to_t_tbl = temp_tbl,
#endif
		.n_v_cap_tbl_elements = ARRAY_SIZE(cap_tbl),
		.v_to_cap_tbl = cap_tbl,
		.timeout_chargeoff_time = 21*HZ,	// specification is 25 +/- 5 seconds. 30*HZ ,
		.initial_timeout_time =HZ*3600*6 ,	//6 hours for first charge attempt 
		.subsequent_timeout_time = HZ*60*90,	//1.5 hours for second and later attempts
		.error_charge_stoptime = HZ*60,		// After an error stop charging for a minute.
		.over_voltage_threshold =  4500 ,
	},

/*
 * These are the batteries that doesn't have an internal NTC resistor to measure
 * its temperature. The temperature in this case is measure with a NTC placed
 * near the battery but on the PCB.
 */


/*
		This battery entry is the real 1650/1500 mAh battery to be fitted to Godin identified by a 1.5K resistor 
	
*/
	{
		.name = POWER_SUPPLY_TECHNOLOGY_LION,
		.resis_high = 2550,		//1500 * 1.7 +70% 
		.resis_low = 450,		//1500 * 0.3 -70%
		.battery_resistance = 300,
		.charge_full_design = 1500, 	//950,
		.nominal_voltage = 3700,
		.termination_vol =  4180,
		.termination_curr_1st = 240, 	//100,
		.termination_curr_2nd = 200, 	//100,
		.recharge_vol = 4120,		//4130,
		.normal_cur_lvl = 600,		//was 700
		.normal_vol_lvl = 4230,		//4210,		//4200,
		.maint_a_cur_lvl = 600,
		.maint_a_vol_lvl = 4150,
		.maint_a_chg_timer_h = 60,
		.maint_b_cur_lvl = 600,
		.maint_b_vol_lvl = 4100,
		.maint_b_chg_timer_h = 200,
		.low_high_cur_lvl = 300,
		.low_high_vol_lvl = 4000,
#ifdef CONFIG_MEASURE_TEMP_BY_ADC_TABLE
		.n_temp_tbl_elements = ARRAY_SIZE(adc_temp_tbl),
		.r_to_t_tbl = adc_temp_tbl,
#else
		.n_temp_tbl_elements = ARRAY_SIZE(temp_tbl),
		.r_to_t_tbl = temp_tbl,
#endif
		.n_v_cap_tbl_elements = ARRAY_SIZE(cap_tbl),
		.v_to_cap_tbl = cap_tbl,
		.timeout_chargeoff_time = 21*HZ,	// specification is 25 +/- 5 seconds. 30*HZ ,
		.initial_timeout_time =HZ*3600*6 ,	//6 hours for first charge attempt 
		.subsequent_timeout_time = HZ*60*90,	//1.5 hours for second and later attempts
		.error_charge_stoptime = HZ*60,		// After an error stop charging for a minute.
		.over_voltage_threshold =  4500 ,
	},
};

static char *ab8500_charger_supplied_to[] = {
	"ab8500_chargalg",
	"ab8500_fg",
	"ab8500_btemp",
};

static char *ab8500_btemp_supplied_to[] = {
	"ab8500_chargalg",
	"ab8500_fg",
};

static char *ab8500_fg_supplied_to[] = {
	"ab8500_chargalg",
};

static char *ab8500_chargalg_supplied_to[] = {
	"ab8500_fg",
};

struct ab8500_charger_platform_data ab8500_charger_plat_data = {
	.supplied_to = ab8500_charger_supplied_to,
	.num_supplicants = ARRAY_SIZE(ab8500_charger_supplied_to),
};

struct ab8500_btemp_platform_data ab8500_btemp_plat_data = {
	.supplied_to = ab8500_btemp_supplied_to,
	.num_supplicants = ARRAY_SIZE(ab8500_btemp_supplied_to),
};

struct ab8500_fg_platform_data ab8500_fg_plat_data = {
	.supplied_to = ab8500_fg_supplied_to,
	.num_supplicants = ARRAY_SIZE(ab8500_fg_supplied_to),
};

struct ab8500_chargalg_platform_data ab8500_chargalg_plat_data = {
	.supplied_to = ab8500_chargalg_supplied_to,
	.num_supplicants = ARRAY_SIZE(ab8500_chargalg_supplied_to),
};

static const struct ab8500_bm_capacity_levels cap_levels = {
	.critical	= 2,
	.low		= 10,
	.normal		= 70,
	.high		= 95,
	.full		= 100,
};

static const struct ab8500_fg_parameters fg = {
	.recovery_sleep_timer = 10,
	.recovery_total_time = 100,
	.init_timer = 1,
	.init_discard_time = 5,
	.init_total_time = 40,
	.high_curr_time = 60,
	.accu_charging = 30,
	.accu_high_curr = 30,
	.high_curr_threshold = 50,
	.lowbat_threshold = 3100,
};

static const struct ab8500_maxim_parameters maxi_params = {
	.ena_maxi = true,
	.chg_curr = 910,
	.wait_cycles = 10,
	.charger_curr_step = 100,
};

static const struct ab8500_bm_charger_parameters chg = {
	.usb_volt_max		= 5500,
	.usb_curr_max		=  900, //1500,
	.ac_volt_max		= 6800,
	.ac_curr_max		=  900, //1500,
};

struct ab8500_bm_data ab8500_bm_data = {
	.temp_under		= -3,
	.temp_low		= 0,
	.temp_high		= 43,
	.temp_over		= 65,
	.ta_chg_current		= 600,
	.ta_chg_voltage		= 4230,
	.usb_chg_current	= 400,
	.usb_chg_voltage	= 4230,
	.main_safety_tmr_h	= 4,
	.usb_safety_tmr_h	= 4,
	.bkup_bat_v		= BUP_VCH_SEL_3P1V,
	.bkup_bat_i		= BUP_ICH_SEL_50UA,
	.no_maintenance		= true,
#ifdef CONFIG_AB8500_BATTERY_THERM_ON_BATCTRL
	.adc_therm		= ADC_THERM_BATCTRL,
#else
	.adc_therm		= ADC_THERM_BATTEMP,
#endif
	.chg_unknown_bat	= false,
	.enable_overshoot	= false,
	.fg_res			= 10,
	.cap_levels		= &cap_levels,
	.bat_type		= bat_type,
	.n_btypes		= ARRAY_SIZE(bat_type),
	.batt_id		= 0,
	.interval_charging	= 5,
	.interval_not_charging	= 120,
	.temp_hysteresis	= 22,		//turn back on if temp < (43 
	.low_temp_hysteresis	= 3,
	.maxi			= &maxi_params,
	.chg_params		= &chg,
	.fg_params		= &fg,
	.use_safety_timer       = 0 ,
};
