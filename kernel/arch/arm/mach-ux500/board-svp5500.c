/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * License terms: GNU General Public License (GPL) version 2
 */

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/amba/bus.h>
#include <linux/gpio.h>

#include <asm/mach/arch.h>
#include <asm/mach-types.h>

#include <plat/pincfg.h>

#include <mach/devices.h>
#include <mach/setup.h>

#include "pins-db5500.h"
#include "devices-db5500.h"

static pin_cfg_t svp5500_pins[] = {
	GPIO28_U0_TXD,
	GPIO29_U0_RXD,
};

static void __init svp5500_init_machine(void)
{
	u5500_init_devices();

	nmk_config_pins(ARRAY_AND_SIZE(svp5500_pins));

	db5500_add_uart0();
}

MACHINE_START(SVP5500, "ST-Ericsson U5500 Simulator")
	.phys_io	= U5500_UART0_BASE,
	.io_pg_offst	= (IO_ADDRESS(U5500_UART0_BASE) >> 18) & 0xfffc,
	.boot_params	= 0x00000100,
	.map_io		= u5500_map_io,
	.init_irq	= ux500_init_irq,
	.timer		= &u8500_timer,
	.init_machine	= svp5500_init_machine,
MACHINE_END
