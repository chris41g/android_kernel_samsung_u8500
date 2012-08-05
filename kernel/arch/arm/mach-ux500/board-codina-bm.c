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
 * Note that the res_to_temp table must be strictly sorted by falling
 * resistance values to work.
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

/* Temporarily, we use this table */
/* 1500 mAh battery table used in Janice (OCV from STE) */
static struct v_to_cap cap_tbl[] = {
	{4162, 100},
	{4131, 99},
	{4088, 95},
	{4045, 90},
	{4024, 87},
	{3955, 80},
	{3893, 70},
	{3859, 65},
	{3825, 60},
	{3799, 55},
	{3780, 50},
	{3750, 40},
	{3731, 30},
	{3714, 25},
	{3683, 20},
	{3658, 17},
	{3648, 14},
	{3640, 12},
	{3627, 10},
	{3615, 9},
	{3566, 7},
	{3539, 6},
	{3477, 4},
	{3403, 2},
	{3361, 1},
	{3320, 0},
};

static struct v_to_cap cap_tbl_5ma[] = {
	{4326,	100},
	{4302,	99},
	{4248,	95},
	{4212,	92},
	{4176,	89},
	{4131,	85},
	{4098,	82},
	{4029,	75},
	{3985,	70},
	{3953,	66},
	{3911,	60},
	{3886,	57},
	{3843,	53},
	{3827,	50},
	{3807,	45},
	{3790,	40},
	{3778,	35},
	{3770,	30},
	{3755,	25},
	{3737,	20},
	{3728,	18},
	{3705,	15},
	{3677,	12},
	{3672,	10},
	{3669,	9},
	{3662,	7},
	{3651,	5},
	{3636,	4},
	{3604,	3},
	{3552,	2},
	{3483,	1},
	{3377,	0},
};

/* Battery voltage to Resistance table*/
static struct v_to_res res_tbl[] = {
	{3890,	176},
	{3800,	212},
	{3750,	215},
	{3700,	237},
	{3670,	252},
	{3640,	280},
	{3590,	324},
	{3560,	340},
	{3530,	395},
	{3460,	608},
	{3440,	605},
	{3420,	582},
	{3400,	572},
	{3340,	669},
	{3300,	710},
	{3250,	687},
	{3200,	714},
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
	{-10, 780},
	{-5, 660},
	{ 0, 580},
	{ 5, 483},
	{10, 416},
	{15, 348},
	{20, 282},
	{25, 233},
	{30, 195},
	{35, 160},
	{40, 131},
	{43, 122},
	{45, 111},
	{47, 100},
	{50, 93},
	{55, 77},
	{60, 66},
	{63, 60},
	{65, 55},
	{70, 48},
};

static const struct battery_type bat_type[] = {
	[BATTERY_UNKNOWN] = {
		/* First element always represent the UNKNOWN battery */
		.name = POWER_SUPPLY_TECHNOLOGY_UNKNOWN,
		.resis_high = 0,
		.resis_low = 0,
		.battery_resistance = 100,
		.line_impedance = 36,
		.charge_full_design = 1500,
		.nominal_voltage = 3820,
		.termination_vol = 4260,
		.termination_curr_1st = 240,
		.termination_curr_2nd = 200,
		.recharge_vol = 4250,
		.normal_cur_lvl = 400,
		.normal_vol_lvl = 4340,
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
		.n_v_cap_tbl_elements = ARRAY_SIZE(cap_tbl_5ma),
		.v_to_cap_tbl = cap_tbl_5ma,
		.n_v_res_tbl_elements = ARRAY_SIZE(res_tbl),
		.v_to_res_tbl = res_tbl,
			/* specification is 25 +/- 5 seconds. 30*HZ */
		.timeout_chargeoff_time = 21*HZ,
			/* 6 hours for first charge attempt */
		.initial_timeout_time = HZ*3600*6,
			/* 1.5 hours for second and later attempts */
		.subsequent_timeout_time = HZ*60*90,
			/* After an error stop charging for a minute. */
		.error_charge_stoptime = HZ*60,
		.over_voltage_threshold =  4500 ,
	},

/*
 * These are the batteries that doesn't have an internal NTC resistor to
 * measure its temperature. The temperature in this case is measure with a NTC
 *  placed near the battery but on the PCB.
 */


/*
	This battery entry is the real 1650/1500 mAh battery
	to be fitted to Codina identified by a 1.5K resistor
*/
	{
		.name = POWER_SUPPLY_TECHNOLOGY_LION,
		.resis_high = 7990,		/* 1500 * 1.7, +70% */
		.resis_low = 0,		/* 1500 * 0.3, -70% */
		.battery_resistance = 100,  /* mOhms,ESR:100+LineImpedance:36 */
		.line_impedance = 36,
		.charge_full_design = 1500,
		.nominal_voltage = 3820,
		.termination_vol =  4350,
		.termination_curr_1st = 100,	/* 100 */
		.termination_curr_2nd = 60,	/* 100 */
		.recharge_vol = 4250,		/* 4130 */
		.normal_cur_lvl = 900,		/* was 700 */
		.normal_vol_lvl = 4340,		/* 4210 */
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
		.n_v_cap_tbl_elements = ARRAY_SIZE(cap_tbl_5ma),
		.v_to_cap_tbl = cap_tbl_5ma,
		.n_v_res_tbl_elements = ARRAY_SIZE(res_tbl),
		.v_to_res_tbl = res_tbl,
			/* specification is 25 +/- 5 seconds. 30*HZ */
		.timeout_chargeoff_time = 21*HZ,
			/* 5 hours for first charge attempt */
		.initial_timeout_time = HZ*3600*5,
			/* 1.5 hours for second and later attempts */
		.subsequent_timeout_time = HZ*60*90,
			/* After an error stop charging for a minute. */
		.error_charge_stoptime = HZ*60,
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
	.accu_charging = 20,
	.accu_high_curr = 20,
	.high_curr_threshold = 50,
	.lowbat_threshold = 3300,
};

static const struct ab8500_maxim_parameters maxi_params = {
	.ena_maxi = true,
	.chg_curr = 900,
	.wait_cycles = 10,
	.charger_curr_step = 100,
};

static const struct ab8500_bm_charger_parameters chg = {
	.usb_volt_max		= 5500,
	.usb_curr_max		= 500,
	/* When power supply set as 7000mV (OVP SPEC above 6.8V)
	   SET read it as .ac_volt_max.
	   After charging is disabled, SET read the voltage
	   as .ac_volt_max_recovery.
	   This value should be modified according to the model
	   experimentally.
	   There's distinct difference between ac voltage when charging
	   and ac voltage when discharging.
	*/
	.ac_volt_max		= 6650,
	.ac_volt_max_recovery	= 6800,
	.ac_curr_max		= 600,
};

struct ab8500_bm_data ab8500_bm_data = {
	.temp_under		= -5,
	.temp_low		= 0,
	.temp_high		= 40,
	.temp_over		= 60,
	.ta_chg_current		= 800,
	.ta_chg_voltage		= 4350,
	.usb_chg_current	= 500,
	.usb_chg_voltage	= 4350,
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
	/* Please find the real setting for fg_res
	   in the ab8500_fg.c probe function  */
	.fg_res			= 133,
	.charge_state		= 1,
	.cap_levels		= &cap_levels,
	.bat_type		= bat_type,
	.n_btypes		= ARRAY_SIZE(bat_type),
	.batt_id		= 0,
	.batt_res		= 0,
	.interval_charging	= 5,
	.interval_not_charging	= 120,
	.temp_hysteresis	= 22,		/* turn back on if temp < 43 */
	.low_temp_hysteresis	= 3,
	.maxi			= &maxi_params,
	.chg_params		= &chg,
	.fg_params		= &fg,
	.use_safety_timer       = 0 ,
};
