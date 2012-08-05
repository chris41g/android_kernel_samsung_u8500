/**
 * arch/arm/mach-xxxxxxx/sec_pmic.c
 *
 * Copyright (C) 2010-2011, Samsung Electronics, Co., Ltd. All Rights Reserved.
 *  Written by SERI R&D Team,
 *  Mobile Communication Division.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifdef CONFIG_AB8500_CORE
#include <linux/mfd/abx500.h>
#include <linux/mfd/ab8500.h>
#include <linux/mfd/ab8500/ab8500-gpio.h>
#include <mach/prcmu.h>
#endif


#include <mach/gpio.h>

#include <mach/sec_pmic.h>

static int pmic_panic_read(u8 bank, u8 addr, u8 *pData)
{
	int ret = -EPERM;
#if defined(CONFIG_AB8500_I2C_CORE) && defined(CONFIG_SAMSUNG_PANIC_DISPLAY_DEVICES)
	ret = prcmu_panic_abb_read(bank, addr, pData, 1);
#endif
	return ret;
}

static int pmic_panic_write(u8 bank, u8 addr, u8 *pData)
{
	int ret = -EPERM;
#ifdef CONFIG_AB8500_I2C_CORE
	ret = prcmu_panic_abb_write(bank, addr, pData, 1);
#endif
	return ret;
}

static int pmic_get_register(struct device *dev, u8 bank,
	u8 reg, u8 *pValue)
{
	return pmic_panic_read(bank, reg, pValue);
}

static int pmic_set_register(struct device *dev, u8 bank,
	u8 reg, u8 value)
{
	return pmic_panic_write(bank, reg, &value);
}

static int pmic_mask_and_set_register(struct device *dev, u8 bank,
	u8 reg, u8 bitmask, u8 bitvalues)
{
	int ret;
	u8 data;

	ret = pmic_panic_read(bank, reg, &data);
	if (ret < 0) {
		pr_emerg("failed to read reg %#x: %d\n",
			reg, ret);
		goto out;
	}

	data = (u8)ret;
	data = (~bitmask & data) | (bitmask & bitvalues);

	ret = pmic_panic_write(bank, reg, &data);
	if (ret < 0)
		pr_emerg("failed to write reg %#x: %d\n",
			reg, ret);

out:
	return ret;
}

#ifdef CONFIG_AB8500_CORE
extern void u8500_reroute_regulator_fp(void);
extern void ab8500_reroute_regulator_fp(void);
extern void ab8500_reroute_core_fp(struct abx500_ops *ops);

static void ab8500_reroute_fp( void )
{
	struct abx500_ops ops = {0,};

#ifdef CONFIG_AB8500_I2C_CORE
	ops.get_register = pmic_get_register;
	ops.set_register = pmic_set_register;
	ops.mask_and_set_register = pmic_mask_and_set_register;
#endif
	ab8500_reroute_core_fp(&ops);
	ab8500_reroute_regulator_fp();
	u8500_reroute_regulator_fp();
}
#endif

#ifdef CONFIG_AB8500_CORE
static void display_ab8500( void )
{
	struct ab8500_i2c_ranges *debug_ranges;
	u32 i;
	u32 bank;
	u32 size;

	debug_ranges = ab8500_register_debug(&size);

	printk(KERN_INFO "[Current State Of AB8500 registers]\n");

	for (bank = 0; bank < size; bank++){
		for (i = 0; i < debug_ranges[bank].num_ranges; i++) {
			u32 reg;

			for (reg = debug_ranges[bank].range[i].first;
				reg <= debug_ranges[bank].range[i].last;
				reg++) {
				u8 value;
				u16 err;

				err = pmic_panic_read((u8)bank, (u8)reg, &value);
				if (err < 0) {
					return ;
				}

				printk(KERN_INFO "  [0x%02x/0x%02X]: 0x%02X\n",
						bank, reg, value);
			}
		}
	}
}
#endif

int sec_pmic_panic_config(void)
{
	int err = -EPERM;
#ifdef CONFIG_AB8500_CORE
	ab8500_reroute_fp();
	err = 0;
#endif
	return err;
}

#ifdef CONFIG_SAMSUNG_PANIC_DISPLAY_PMIC
int sec_disp_pmic_info(void)
{
	int err = -EPERM;

#ifdef CONFIG_AB8500_CORE
	display_ab8500();
	err = 0;
#endif
	return err;
}
#endif


