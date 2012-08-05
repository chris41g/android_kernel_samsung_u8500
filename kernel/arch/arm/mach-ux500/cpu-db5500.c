/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * Author: Rabin Vincent <rabin.vincent@stericsson.com> for ST-Ericsson
 * License terms: GNU General Public License (GPL) version 2
 */

#include <linux/platform_device.h>
#include <linux/amba/bus.h>
#include <linux/io.h>
#include <linux/err.h>

#include <asm/mach/map.h>

#include <mach/hardware.h>
#include <mach/devices.h>
#include <mach/setup.h>
#include <mach/irqs.h>

#include "devices-db5500.h"

static struct map_desc u5500_io_desc[] __initdata = {
	__IO_DEV_DESC(U5500_MTU0_BASE, SZ_4K),
	__IO_DEV_DESC(U5500_MTU1_BASE, SZ_4K),
	__IO_DEV_DESC(U5500_SCU_BASE, SZ_4K),
	__IO_DEV_DESC(U5500_TWD_BASE, SZ_4K),
	__IO_DEV_DESC(U5500_UART0_BASE, SZ_4K),
	__IO_DEV_DESC(U5500_UART1_BASE, SZ_4K),
	__IO_DEV_DESC(U5500_UART2_BASE, SZ_4K),
	__IO_DEV_DESC(U5500_RTC_BASE, SZ_4K),
	__IO_DEV_DESC(U5500_GPIO0_BASE, SZ_4K),
	__IO_DEV_DESC(U5500_GPIO1_BASE, SZ_4K),
	__IO_DEV_DESC(U5500_GPIO2_BASE, SZ_4K),
	__IO_DEV_DESC(U5500_GPIO3_BASE, SZ_4K),
	__IO_DEV_DESC(U5500_GPIO4_BASE, SZ_4K),
	__IO_DEV_DESC(U5500_PRCMU_BASE, SZ_4K),
	__IO_DEV_DESC(U5500_PRCMU_TCDM_BASE, SZ_4K),
	__MEM_DEV_DESC(U5500_BOOT_ROM_BASE, SZ_1M),
	__IO_DEV_DESC(U5500_BACKUPRAM0_BASE, SZ_8K),
	__IO_DEV_DESC(U5500_GIC_CPU_BASE, SZ_4K),
	__IO_DEV_DESC(U5500_GIC_DIST_BASE, SZ_4K),
	__IO_DEV_DESC(U5500_L2CC_BASE, SZ_4K),
	__IO_DEV_DESC(U5500_CLKRST1_BASE, SZ_4K),
	__IO_DEV_DESC(U5500_CLKRST2_BASE, SZ_4K),
	__IO_DEV_DESC(U5500_CLKRST3_BASE, SZ_4K),
	__IO_DEV_DESC(U5500_CLKRST5_BASE, SZ_4K),
	__IO_DEV_DESC(U5500_CLKRST6_BASE, SZ_4K),
};

static struct resource mbox0_resources[] = {
	{
		.name = "mbox_peer",
		.start = U5500_MBOX0_PEER_START,
		.end = U5500_MBOX0_PEER_END,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "mbox_local",
		.start = U5500_MBOX0_LOCAL_START,
		.end = U5500_MBOX0_LOCAL_END,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "mbox_irq",
		.start = MBOX_PAIR0_VIRT_IRQ,
		.end = MBOX_PAIR0_VIRT_IRQ,
		.flags = IORESOURCE_IRQ,
	}
};

static struct resource mbox1_resources[] = {
	{
		.name = "mbox_peer",
		.start = U5500_MBOX1_PEER_START,
		.end = U5500_MBOX1_PEER_END,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "mbox_local",
		.start = U5500_MBOX1_LOCAL_START,
		.end = U5500_MBOX1_LOCAL_END,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "mbox_irq",
		.start = MBOX_PAIR1_VIRT_IRQ,
		.end = MBOX_PAIR1_VIRT_IRQ,
		.flags = IORESOURCE_IRQ,
	}
};

static struct resource mbox2_resources[] = {
	{
		.name = "mbox_peer",
		.start = U5500_MBOX2_PEER_START,
		.end = U5500_MBOX2_PEER_END,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "mbox_local",
		.start = U5500_MBOX2_LOCAL_START,
		.end = U5500_MBOX2_LOCAL_END,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "mbox_irq",
		.start = MBOX_PAIR2_VIRT_IRQ,
		.end = MBOX_PAIR2_VIRT_IRQ,
		.flags = IORESOURCE_IRQ,
	}
};

static struct platform_device mbox0_device = {
	.id = 0,
	.name = "mbox",
	.resource = mbox0_resources,
	.num_resources = ARRAY_SIZE(mbox0_resources),
};

static struct platform_device mbox1_device = {
	.id = 1,
	.name = "mbox",
	.resource = mbox1_resources,
	.num_resources = ARRAY_SIZE(mbox1_resources),
};

static struct platform_device mbox2_device = {
	.id = 2,
	.name = "mbox",
	.resource = mbox2_resources,
	.num_resources = ARRAY_SIZE(mbox2_resources),
};

static struct platform_device *u5500_platform_devs[] __initdata = {
	&u5500_gpio_devs[0],
	&u5500_gpio_devs[1],
	&u5500_gpio_devs[2],
	&u5500_gpio_devs[3],
	&u5500_gpio_devs[4],
	&u5500_gpio_devs[5],
	&u5500_gpio_devs[6],
	&u5500_gpio_devs[7],
	&mbox0_device,
	&mbox1_device,
	&mbox2_device,
	&u5500_pwm0_device,
	&u5500_pwm1_device,
	&u5500_pwm2_device,
	&u5500_pwm3_device,
};

void __init u5500_map_io(void)
{
	iotable_init(u5500_io_desc, ARRAY_SIZE(u5500_io_desc));

	_PRCMU_BASE = __io_address(U5500_PRCMU_BASE);
}

void __init u5500_init_devices(void)
{
	struct amba_device *dev;

	ux500_init_devices();

	db5500_dma_init();

	dev = db5500_add_rtc();
	if (!IS_ERR(dev))
		device_init_wakeup(&dev->dev, true);

	platform_add_devices(u5500_platform_devs,
			     ARRAY_SIZE(u5500_platform_devs));
}
