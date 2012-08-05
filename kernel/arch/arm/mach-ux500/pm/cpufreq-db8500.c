/*
 * Copyright (C) ST-Ericsson SA 2011
 *
 * License Terms: GNU General Public License v2
 */

#include <linux/kernel.h>
#include <linux/cpufreq.h>

#include <mach/prcmu.h>

#include "cpufreq.h"

static struct cpufreq_frequency_table freq_table[] = {
	[0] = {
		.index = 0,
		.frequency = 200000,
	},
	[1] = {
		.index = 1,
		.frequency = 400000,
	},
	[2] = {
		.index = 2,
		.frequency = 800000,
	},
	[3] = {
		/* Used for MAX_OPP, if available */
		.index = 3,
		.frequency = CPUFREQ_TABLE_END,
	},
	[4] = {
		.index = 4,
		.frequency = CPUFREQ_TABLE_END,
	},
};

static enum arm_opp idx2opp[] = {
	ARM_EXTCLK,
	ARM_50_OPP,
	ARM_100_OPP,
	ARM_MAX_OPP
};

/*
 * Below is a temporary workaround for wlan performance issues
 */

#include <linux/kernel_stat.h>
#include <linux/workqueue.h>
#include <linux/cpu.h>

#include <mach/irqs.h>
#include <mach/prcmu-qos.h>

#define WLAN_PROBE_DELAY 3000 /* 3 seconds */
#define WLAN_LIMIT (3000/3) /* If we have more than 1000 irqs per second */
static struct delayed_work work_wlan_workaround;
bool wlan_mode_on;

#define USB_PROBE_DELAY 1000 /* 1 seconds */
#define USB_LIMIT (200) /* If we have more than 200 irqs per second */
static struct delayed_work work_usb_workaround;
bool usb_mode_on;

static void wlan_load(struct work_struct *work)
{
	int cpu;
	unsigned int num_irqs = 0;
	static unsigned int old_num_irqs = UINT_MAX;

	for_each_online_cpu(cpu)
		num_irqs += kstat_irqs_cpu(IRQ_DB8500_SDMMC1, cpu);

	if ((num_irqs > old_num_irqs) &&
	    (num_irqs - old_num_irqs) > WLAN_LIMIT)
		wlan_mode_on = true;
	else
		wlan_mode_on = false;

	old_num_irqs = num_irqs;

	schedule_delayed_work_on(0,
				 &work_wlan_workaround,
				 msecs_to_jiffies(WLAN_PROBE_DELAY));
}

static void usb_load(struct work_struct *work)
{
	int cpu;
	unsigned int num_irqs = 0;
	static unsigned int old_num_irqs = UINT_MAX;

	for_each_online_cpu(cpu)
		num_irqs += kstat_irqs_cpu(IRQ_DB8500_USBOTG, cpu);

	if ((num_irqs > old_num_irqs) &&
	    (num_irqs - old_num_irqs) > USB_LIMIT)
		usb_mode_on = true;
	else
		usb_mode_on = false;

	old_num_irqs = num_irqs;

	schedule_delayed_work_on(0,
				 &work_usb_workaround,
				 msecs_to_jiffies(USB_PROBE_DELAY));
}

void cpufreq_usb_connect_notify(bool connect)
{
	if (connect) {
		schedule_delayed_work_on(0,
				 &work_usb_workaround,
				 msecs_to_jiffies(USB_PROBE_DELAY));
	} else {
		cancel_delayed_work_sync(&work_usb_workaround);
		usb_mode_on = false;
	}
}

int dbx500_cpufreq_percent2freq(int percent)
{
	int op;
	int i;

	switch (percent) {
	case 0:
		/* Fall through */
	case 25:
		op = ARM_EXTCLK;
		break;
	case 50:
		op = ARM_50_OPP;
		break;
	case 100:
		op = ARM_100_OPP;
		break;
	case 125:
		if (prcmu_has_arm_maxopp())
			op = ARM_MAX_OPP;
		else
			op = ARM_100_OPP;
		break;
	default:
		pr_err("cpufreq-db8500: Incorrect arm target value (%d).\n",
		       percent);;
		return -EINVAL;
		break;
	}

	for (i = 0; idx2opp[i] != op && i < ARRAY_SIZE(freq_table); i++)
		;

	if (freq_table[i].frequency == CPUFREQ_TABLE_END) {
		pr_err("cpufreq-db8500: Matching frequency does not exist!\n");
		return -EINVAL;
	}

	return freq_table[i].frequency;
}

int u8500_cpufreq_limits(int cpu, int r, unsigned int *min, unsigned int *max)
{
	int freq;
	int ret;
	static int old_freq;
	struct cpufreq_policy p;

	freq = dbx500_cpufreq_percent2freq(r);

	if (freq < 0)
		return -EINVAL;

	if (freq != old_freq)
		pr_debug("cpufreq-db8500: set min arm freq to %d\n",
			 freq);

	(*min) = freq;

	ret = cpufreq_get_policy(&p, cpu);
	if (ret) {
		pr_err("cpufreq-db8500: Failed to get policy.\n");
		return -EINVAL;
	}

	(*max) = p.max;
	return 0;
}

static int __init u8500_cpufreq_register(void)
{
	int i = 0;

	BUILD_BUG_ON(ARRAY_SIZE(idx2opp) + 1 != ARRAY_SIZE(freq_table));

	if (prcmu_has_arm_maxopp())
		freq_table[3].frequency = 1000000;

	INIT_DELAYED_WORK_DEFERRABLE(&work_wlan_workaround,
				     wlan_load);

	INIT_DELAYED_WORK_DEFERRABLE(&work_usb_workaround,
				     usb_load);

	schedule_delayed_work_on(0,
				 &work_wlan_workaround,
				 msecs_to_jiffies(WLAN_PROBE_DELAY));

	pr_info("u8500-cpufreq : Available frequencies:\n");
	while (freq_table[i].frequency != CPUFREQ_TABLE_END)
		pr_info("  %d Mhz\n", freq_table[i++].frequency/1000);

	return ux500_cpufreq_register(freq_table, idx2opp);
}
device_initcall(u8500_cpufreq_register);
