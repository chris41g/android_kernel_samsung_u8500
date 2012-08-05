/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * License terms: GNU General Public License (GPL), version 2.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>

#include <asm/mach/arch.h>
#include <asm/mach-types.h>

#include <plat/pincfg.h>

#include <mach/devices.h>
#include <mach/hardware.h>
#include <mach/setup.h>

#include "devices-db8500.h"
#include "pins-db8500.h"

static pin_cfg_t svp8500_pins[] = {
	/* UART */
	GPIO0_U0_CTSn	| PIN_INPUT_PULLUP,
	GPIO1_U0_RTSn	| PIN_OUTPUT_HIGH,
	GPIO2_U0_RXD	| PIN_INPUT_PULLUP,
	GPIO3_U0_TXD	| PIN_OUTPUT_HIGH,
	GPIO4_U1_RXD	| PIN_INPUT_PULLUP,
	GPIO5_U1_TXD	| PIN_OUTPUT_HIGH,
	GPIO6_U1_CTSn	| PIN_INPUT_PULLUP,
	GPIO7_U1_RTSn	| PIN_OUTPUT_HIGH,
	GPIO29_U2_RXD	| PIN_INPUT_PULLUP,
	GPIO30_U2_TXD	| PIN_OUTPUT_HIGH,
	GPIO31_U2_CTSn	| PIN_INPUT_PULLUP,
	GPIO32_U2_RTSn	| PIN_OUTPUT_HIGH,
};

static void __init svp8500_uart_init(void)
{
	db8500_add_uart0(NULL);
	db8500_add_uart1();
	db8500_add_uart2();
}

static void __init svp8500_init_machine(void)
{
	nmk_config_pins(ARRAY_AND_SIZE(svp8500_pins));

	svp8500_uart_init();
}

MACHINE_START(SVP8500V1, "ST-Ericsson U8500 Simulator (V1)")
	.phys_io	= U8500_UART2_BASE,
	.io_pg_offst	= (IO_ADDRESS(U8500_UART2_BASE) >> 18) & 0xfffc,
	.boot_params	= 0x00000100,
	.map_io		= u8500_map_io,
	.init_irq	= ux500_init_irq,
	.timer		= &u8500_timer,
	.init_machine	= svp8500_init_machine,
MACHINE_END

MACHINE_START(SVP8500V2, "ST-Ericsson U8500 Simulator (V2)")
	.phys_io	= U8500_UART2_BASE,
	.io_pg_offst	= (IO_ADDRESS(U8500_UART2_BASE) >> 18) & 0xfffc,
	.boot_params	= 0x00000100,
	.map_io		= u8500_map_io,
	.init_irq	= ux500_init_irq,
	.timer		= &u8500_timer,
	.init_machine	= svp8500_init_machine,
MACHINE_END
