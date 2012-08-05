/*
 * Copyright (C) 2008-2009 ST-Ericsson
 *
 * Author: Srinidhi KASAGAR <srinidhi.kasagar@stericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 *
 */
#include <linux/types.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/amba/bus.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/sys_soc.h>

#include <asm/pmu.h>
#include <asm/mach/map.h>
#include <mach/hardware.h>
#include <mach/setup.h>
#include <mach/devices.h>
#include <mach/prcmu.h>
#include <mach/reboot_reasons.h>

static struct resource db8500_pmu_resources[] = {
	[0] = {
		.start		= IRQ_DB8500_PMU,
		.end		= IRQ_DB8500_PMU,
		.flags		= IORESOURCE_IRQ,
	},
};

/*
 * The PMU IRQ lines of two cores are wired together into a single interrupt.
 * Bounce the interrupt to the other core if it's not ours.
 */
static irqreturn_t db8500_pmu_handler(int irq, void *dev, irq_handler_t handler)
{
	irqreturn_t ret = handler(irq, dev);
	int other = !smp_processor_id();

	if (ret == IRQ_NONE && cpu_online(other))
		irq_set_affinity(irq, cpumask_of(other));

	/*
	 * We should be able to get away with the amount of IRQ_NONEs we give,
	 * while still having the spurious IRQ detection code kick in if the
	 * interrupt really starts hitting spuriously.
	 */
	return ret;
}

static struct arm_pmu_platdata db8500_pmu_platdata = {
	.handle_irq		= db8500_pmu_handler,
};

static struct platform_device db8500_pmu_device = {
	.name			= "arm-pmu",
	.id			= ARM_PMU_DEVICE_CPU,
	.num_resources		= ARRAY_SIZE(db8500_pmu_resources),
	.resource		= db8500_pmu_resources,
	.dev.platform_data	= &db8500_pmu_platdata,
};

static struct platform_device *platform_devs[] __initdata = {
	&u8500_gpio_devs[0],
	&u8500_gpio_devs[1],
	&u8500_gpio_devs[2],
	&u8500_gpio_devs[3],
	&u8500_gpio_devs[4],
	&u8500_gpio_devs[5],
	&u8500_gpio_devs[6],
	&u8500_gpio_devs[7],
	&u8500_gpio_devs[8],
	&ux500_wdt_device,
	&db8500_pmu_device,
};

#include "devices-db8500.h"

static struct map_desc u8500_io_desc[] __initdata = {
	__IO_DEV_DESC(U8500_MTU0_BASE, SZ_4K),
	__IO_DEV_DESC(U8500_MTU1_BASE, SZ_4K),
	__IO_DEV_DESC(U8500_SCU_BASE, SZ_4K),
	__IO_DEV_DESC(U8500_TWD_BASE, SZ_4K),
	__IO_DEV_DESC(U8500_UART0_BASE, SZ_4K),
	__IO_DEV_DESC(U8500_UART1_BASE, SZ_4K),
	__IO_DEV_DESC(U8500_UART2_BASE, SZ_4K),
	__IO_DEV_DESC(U8500_RTC_BASE, SZ_4K),
	__IO_DEV_DESC(U8500_MSP0_BASE, SZ_4K),
	__IO_DEV_DESC(U8500_MSP1_BASE, SZ_4K),
	__IO_DEV_DESC(U8500_MSP2_BASE, SZ_4K),
	__IO_DEV_DESC(U8500_PRCMU_BASE, SZ_4K),
	__IO_DEV_DESC(U8500_GPIO0_BASE, SZ_4K),
	__IO_DEV_DESC(U8500_GPIO1_BASE, SZ_4K),
	__IO_DEV_DESC(U8500_GPIO2_BASE, SZ_4K),
	__IO_DEV_DESC(U8500_GPIO3_BASE, SZ_4K),
	__IO_DEV_DESC(U8500_PRCMU_BASE, SZ_4K),
	__IO_DEV_DESC(U8500_STM_REG_BASE, SZ_4K),
	{IO_ADDRESS(U8500_BACKUPRAM0_BASE),
		__phys_to_pfn(U8500_BACKUPRAM0_BASE), SZ_8K, MT_BACKUP_RAM},
	__MEM_DEV_DESC(U8500_BOOT_ROM_BASE, SZ_1M),
	__IO_DEV_DESC(U8500_CLKRST1_BASE, SZ_4K),
	__IO_DEV_DESC(U8500_CLKRST2_BASE, SZ_4K),
	__IO_DEV_DESC(U8500_CLKRST3_BASE, SZ_4K),
	__IO_DEV_DESC(U8500_CLKRST5_BASE, SZ_4K),
	__IO_DEV_DESC(U8500_CLKRST6_BASE, SZ_4K),
	__IO_DEV_DESC(U8500_GIC_CPU_BASE, SZ_4K),
	__IO_DEV_DESC(U8500_GIC_DIST_BASE, SZ_4K),
	__IO_DEV_DESC(U8500_L2CC_BASE, SZ_4K),
};

static struct map_desc u8500ed_io_desc[] __initdata = {
	__IO_DEV_DESC(U8500_MTU0_BASE_ED, SZ_4K),
	__IO_DEV_DESC(U8500_MTU1_BASE_ED, SZ_4K),
	__IO_DEV_DESC(U8500_CLKRST7_BASE_ED, SZ_8K),
};

static struct map_desc u8500v1_io_desc[] __initdata = {
	__IO_DEV_DESC(U8500_PRCMU_TCDM_BASE_V1, SZ_4K),
};

static struct map_desc u8500v2_io_desc[] __initdata = {
	__IO_DEV_DESC(U8500_PRCMU_TCDM_BASE, SZ_4K),
};
/*
 * Functions to differentiate between later ASICs
 * We look into the end of the ROM to locate the hardcoded ASIC ID.
 * This is only needed to differentiate between minor revisions and
 * process variants of an ASIC, the major revisions are encoded in
 * the cpuid.
 */
#define U8500_ASIC_ID_LOC_ED_V1	(U8500_BOOT_ROM_BASE + 0x1FFF4)
#define U8500_ASIC_ID_LOC_V2	(U8500_BOOT_ROM_BASE + 0x1DBF4)
#define U8500_ASIC_REV_ED	0x01
#define U8500_ASIC_REV_V10	0xA0
#define U8500_ASIC_REV_V11	0xA1
#define U8500_ASIC_REV_V20	0xB0
#define U8500_ASIC_REV_V21	0xB1
#define U8500_ASIC_REV_V22	0xB2

/**
 * struct db8500_asic_id - fields of the ASIC ID
 * @process: the manufacturing process, 0x40 is 40 nm
 *  0x00 is "standard"
 * @partnumber: hithereto 0x8500 for DB8500
 * @revision: version code in the series
 * This field definion is not formally defined but makes
 * sense.
 */
struct db8500_asic_id {
	u8 process;
	u16 partnumber;
	u8 revision;
};

/* This isn't going to change at runtime */
static struct db8500_asic_id db8500_id;

static void __init get_db8500_asic_id(void)
{
	u32 asicid;

	if (cpu_is_u8500v1() || cpu_is_u8500ed())
		asicid = readl(__io_address(U8500_ASIC_ID_LOC_ED_V1));
	else if (cpu_is_u8500v2())
		asicid = readl(__io_address(U8500_ASIC_ID_LOC_V2));
	else
		BUG();

	db8500_id.process = (asicid >> 24);
	db8500_id.partnumber = (asicid >> 16) & 0xFFFFU;
	db8500_id.revision = asicid & 0xFFU;
}

bool cpu_is_u8500v10(void)
{
	return (db8500_id.revision == U8500_ASIC_REV_V10);
}

bool cpu_is_u8500v11(void)
{
	return (db8500_id.revision == U8500_ASIC_REV_V11);
}

bool cpu_is_u8500v20(void)
{
	return (db8500_id.revision == U8500_ASIC_REV_V20);
}

bool cpu_is_u8500v21(void)
{
	return (db8500_id.revision == U8500_ASIC_REV_V21);
}

bool cpu_is_u8500v22(void)
{
	return (db8500_id.revision == U8500_ASIC_REV_V22);
}

bool cpu_is_u8500v20_or_later(void)
{
	return !cpu_is_u8500v10() && !cpu_is_u8500v11();
}

void __init u8500_map_io(void)
{
	iotable_init(u8500_io_desc, ARRAY_SIZE(u8500_io_desc));

	if (cpu_is_u8500ed())
		iotable_init(u8500ed_io_desc, ARRAY_SIZE(u8500ed_io_desc));
	else if (cpu_is_u8500v1())
		iotable_init(u8500v1_io_desc, ARRAY_SIZE(u8500v1_io_desc));
	else if (cpu_is_u8500v2())
		iotable_init(u8500v2_io_desc, ARRAY_SIZE(u8500v2_io_desc));

	_PRCMU_BASE = __io_address(U8500_PRCMU_BASE);

	/* Read out the ASIC ID as early as we can */
	get_db8500_asic_id();
}

static void __init u8500_earlydrop_fixup(void)
{
	u8500_shrm_device.resource[1].start = IRQ_CA_WAKE_REQ_ED;
	u8500_shrm_device.resource[1].end = IRQ_CA_WAKE_REQ_ED;
	u8500_shrm_device.resource[2].start = IRQ_AC_READ_NOTIFICATION_0_ED;
	u8500_shrm_device.resource[2].end = IRQ_AC_READ_NOTIFICATION_0_ED;
	u8500_shrm_device.resource[3].start = IRQ_AC_READ_NOTIFICATION_1_ED;
	u8500_shrm_device.resource[3].end = IRQ_AC_READ_NOTIFICATION_1_ED;
	u8500_shrm_device.resource[4].start = IRQ_CA_MSG_PEND_NOTIFICATION_0_ED;
	u8500_shrm_device.resource[4].end = IRQ_CA_MSG_PEND_NOTIFICATION_0_ED;
	u8500_shrm_device.resource[5].start = IRQ_CA_MSG_PEND_NOTIFICATION_1_ED;
	u8500_shrm_device.resource[5].end = IRQ_CA_MSG_PEND_NOTIFICATION_1_ED;
}
/*
 * This function is called from the board init
 */
void __init u8500_init_devices(void)
{
	struct amba_device *dev;

	ux500_init_devices();

	/* Display some ASIC boilerplate */
	pr_info("DB8500: process: %02x, revision ID: 0x%02x\n",
		db8500_id.process, db8500_id.revision);
	if (cpu_is_u8500ed())
		pr_info("DB8500: Early Drop (ED)\n");
	else if (cpu_is_u8500v10())
		pr_info("DB8500: version 1.0\n");
	else if (cpu_is_u8500v11())
		pr_info("DB8500: version 1.1\n");
	else if (cpu_is_u8500v20())
		pr_info("DB8500: version 2.0\n");
	else if (cpu_is_u8500v21())
		pr_info("DB8500: version 2.1\n");
	else if (cpu_is_u8500v22())
		pr_info("DB8500: version 2.2\n");
	else
		pr_warning("ASIC: UNKNOWN SILICON VERSION!\n");

	if (cpu_is_u8500ed())
		u8500_earlydrop_fixup();

	db8500_dma_init();
	db8500_uart_init();

	dev = db8500_add_rtc();
	if (!IS_ERR(dev))
		device_init_wakeup(&dev->dev, true);

	/* Register the platform devices */
	platform_add_devices(platform_devs, ARRAY_SIZE(platform_devs));

	return ;
}

#ifdef CONFIG_SYS_SOC
#define U8500_BB_UID_BASE (U8500_BACKUPRAM1_BASE + 0xFC0)
#define U8500_BB_UID_LENGTH 5

static ssize_t ux500_get_machine(char *buf, struct sysfs_soc_info *si)
{
	return sprintf(buf, "DB%2x00\n", db8500_id.partnumber);
}

static ssize_t ux500_get_soc_id(char *buf, struct sysfs_soc_info *si)
{
	void __iomem *uid_base;
	int i;
	ssize_t sz = 0;

	if (db8500_id.partnumber == 0x85) {
		uid_base = __io_address(U8500_BB_UID_BASE);
		for (i = 0; i < U8500_BB_UID_LENGTH; i++)
			sz += sprintf(buf + sz, "%08x", readl(uid_base + i * sizeof(u32)));
		sz += sprintf(buf + sz, "\n");
	}
	else {
		/* Don't know where it is located for U5500 */
		sz = sprintf(buf, "N/A\n");
	}

	return sz;
}

static ssize_t ux500_get_revision(char *buf, struct sysfs_soc_info *si)
{
	unsigned int rev = db8500_id.revision;

	if (rev == 0x01)
		return sprintf(buf, "%s\n", "ED");
	else if (rev >= 0xA0)
		return sprintf(buf, "%d.%d\n" , (rev >> 4) - 0xA + 1, rev & 0xf);

	return sprintf(buf, "%s", "Unknown\n");
}

static ssize_t ux500_get_process(char *buf, struct sysfs_soc_info *si)
{
	if (db8500_id.process == 0x00)
		return sprintf(buf, "Standard\n");

	return sprintf(buf, "%02xnm\n", db8500_id.process);
}

static ssize_t ux500_get_reset_code(char *buf, struct sysfs_soc_info *si)
{
	return sprintf(buf, "0x%04x\n", prcmu_get_reset_code());
}

static ssize_t ux500_get_reset_reason(char *buf, struct sysfs_soc_info *si)
{
	return sprintf(buf, "%s\n",
		reboot_reason_string(prcmu_get_reset_code()));
}

static struct sysfs_soc_info soc_info[] = {
	SYSFS_SOC_ATTR_CALLBACK("machine", ux500_get_machine),
	SYSFS_SOC_ATTR_VALUE("family", "Ux500"),
	SYSFS_SOC_ATTR_CALLBACK("soc_id", ux500_get_soc_id),
	SYSFS_SOC_ATTR_CALLBACK("revision", ux500_get_revision),
	SYSFS_SOC_ATTR_CALLBACK("process", ux500_get_process),
	SYSFS_SOC_ATTR_CALLBACK("reset_code", ux500_get_reset_code),
	SYSFS_SOC_ATTR_CALLBACK("reset_reason", ux500_get_reset_reason),
};

static int __init ux500_sys_soc_init(void)
{
	return register_sysfs_soc(soc_info, ARRAY_SIZE(soc_info));
}

module_init(ux500_sys_soc_init);
#endif

