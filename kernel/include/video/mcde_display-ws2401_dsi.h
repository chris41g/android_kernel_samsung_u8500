/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * Samsung MCDE Widechips WS2401 DCS display driver
 *
 * Author: Gareth Phillips <gareth.phillips@samsung.com>
 * for Samsung Electronics.
 *
 * License terms: GNU General Public License (GPL), version 2.
 */
#ifndef __MCDE_DISPLAY_WS2401__H__
#define __MCDE_DISPLAY_WS2401__H__

#include <linux/regulator/consumer.h>

#include "mcde_display.h"

enum display_panel_type {
	DISPLAY_NONE			= 0,
	/* gareth.phillips needs changing when we know the ID for the WS2401 */
	DISPLAY_WS2401          = 0x1b81,
};

struct  ws2401_platform_data {
	bool platform_enabled;

	/* Platform info */
	int power_on_gpio;
	bool power_on_high;
	int power_on_delay;

	int reset_gpio;
	bool reset_high;

	const char *regulator_id;
	int reset_delay; /* ms */
	int reset_low_delay; /* ms */
	int sleep_out_delay; /* ms */
	int sleep_in_delay; /* ms */
	enum display_panel_type disp_panel; /* display panel types */

	int bl_en_gpio;
	bool no_bl_ctrl;

	/* Driver data */
	bool ws2401_platform_enable;
	struct regulator *regulator;
};

#endif /* __MCDE_DISPLAY_WS2401__H__ */

