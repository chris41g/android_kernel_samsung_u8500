/*
 * Copyright (C) ST-Ericsson SA 2011
 *
 * License Terms: GNU General Public License v2
 * Author: Johan Bjornstedt <johan.bjornstedt@stericsson.com>
 *
 * Save DBx500 registers in case of kernel crash
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/kdebug.h>

#include <mach/hardware.h>
#include <mach/db8500-regs.h>

struct dbx500_dump_info {
	char *name;
	int *data;
	int *io_addr;
	int phy_addr;
	int size;
};

static struct dbx500_dump_info db8500_dump[] = {
	{
		.name     = "prcmu_tcdm",
		.phy_addr = U8500_PRCMU_TCDM_BASE,
		.size     = 0x1000,
	},
	{
		.name     = "prcmu_non_sec_1",
		.phy_addr = U8500_PRCMU_BASE,
		.size     = 0x340,
	},
	{
		.name     = "prcmu_pmb",
		.phy_addr = (U8500_PRCMU_BASE + 0x344),
		.size     = 0xC,
	},
	{
		.name     = "prcmu_thermal",
		.phy_addr = (U8500_PRCMU_BASE + 0x3C0),
		.size     = 0x40,
	},
	{
		.name     = "prcmu_non_sec_2",
		.phy_addr = (U8500_PRCMU_BASE + 0x404),
		.size     = 0x1FC,
	},
	{
		.name     = "prcmu_icn_pmu",
		.phy_addr = (U8500_PRCMU_BASE + 0xE00),
		.size     = 0x90,
	},
};

static struct dbx500_dump_info db5500_dump[] = {
	{},
};

static struct dbx500_dump_info *dbx500_dump;
static int dbx500_dump_size;
static int dbx500_dump_done;

static int crash_notifier(struct notifier_block *nb, unsigned long val,
		void *data)
{
	int i;

	if (dbx500_dump_done)
		return 0;

	pr_info("dbx500_dump notified of crash\n");

	for (i = 0; i < dbx500_dump_size; i++) {
		memcpy_fromio(dbx500_dump[i].data, dbx500_dump[i].io_addr,
			dbx500_dump[i].size);
	}

	dbx500_dump_done = 1;

	return 0;
}

static void __init init_io_addresses(void)
{
	int i;

	for (i = 0; i < dbx500_dump_size; i++)
		dbx500_dump[i].io_addr = __io_address(dbx500_dump[i].phy_addr);
}

void dbx500_dump_in_panic(void)
{
       crash_notifier(0, 0, 0);
}
EXPORT_SYMBOL(dbx500_dump_in_panic);

static struct notifier_block die_notifier = {
	.notifier_call = crash_notifier,
	.priority = 0,
};

int __init dbx500_dump_init(void)
{
	int err, i;

	if (cpu_is_u5500()) {
		dbx500_dump = db5500_dump;
		dbx500_dump_size = ARRAY_SIZE(db5500_dump);
	} else if (cpu_is_u8500()) {
		dbx500_dump = db8500_dump;
		dbx500_dump_size = ARRAY_SIZE(db8500_dump);
	} else {
		ux500_unknown_soc();
	}

	for (i = 0; i < dbx500_dump_size; i++) {
		dbx500_dump[i].data = kmalloc(dbx500_dump[i].size, GFP_KERNEL);
		if (!dbx500_dump[i].data) {
			pr_err("dbx500_dump: Could not allocate memory for "
				"%s\n", dbx500_dump[i].name);
			err = -ENOMEM;
			goto free_mem;
		}
	}

	init_io_addresses();

	err = register_die_notifier(&die_notifier);
	if (err != 0) {
		pr_err("dbx500_dump: Unable to register a die notifier %d\n",
			err);
		goto free_mem;
	}
	pr_info("dbx500_dump: driver initialized\n");
	return err;

free_mem:
	for (i = i - 1; i >= 0; i--)
		kfree(dbx500_dump[i].data);

	return err;
}
arch_initcall(dbx500_dump_init);
