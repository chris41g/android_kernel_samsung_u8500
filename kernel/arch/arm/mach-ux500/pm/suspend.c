/*
 * Copyright (C) STMicroelectronics 2009
 * Copyright (C) ST-Ericsson SA 2010-2011
 *
 * License Terms: GNU General Public License v2
 *
 * Authors: Rickard Andersson <rickard.andersson@stericsson.com>,
 *	    Jonas Aaberg <jonas.aberg@stericsson.com>,
 *          Sundar Iyer for ST-Ericsson.
 *
 */

#include <linux/suspend.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/regulator/ab8500-debug.h>
#include <linux/wakelock.h>

#include <mach/prcmu.h>
#include <mach/prcmu-regs.h>
#include <mach/suspend.h>

#include "timer.h"
#include "context.h"
#include "pm.h"
#include "suspend_dbg.h"

static void (*pins_suspend_force)(void);
static void (*pins_suspend_force_mux)(void);

void suspend_set_pins_force_fn(void (*force)(void), void (*force_mux)(void))
{
	pins_suspend_force = force;
	pins_suspend_force_mux = force_mux;
}

u32 alarm_sec;

void suspend_set_alarm(u32 s)
{
	alarm_sec = s;
}

static atomic_t block_sleep_modem = ATOMIC_INIT(0);
static atomic_t block_sleep_event = ATOMIC_INIT(0);


void suspend_block_sleep(int block_case)
{
	if (block_case == UX500_MODEM_BLOCK)
		atomic_inc(&block_sleep_modem);
	else if (block_case == UX500_EVENT_BLOCK)
		atomic_inc(&block_sleep_event);
}

void suspend_unblock_sleep(int block_case)
{
	if (block_case == UX500_MODEM_BLOCK)
		atomic_dec(&block_sleep_modem);
	else if (block_case == UX500_EVENT_BLOCK)
		atomic_dec(&block_sleep_event);
}

static bool sleep_is_blocked(void)
{
	return (atomic_read(&block_sleep_modem) != 0 ||
		atomic_read(&block_sleep_event) != 0);
}

static int suspend(bool do_deepsleep)
{
	bool pins_force = pins_suspend_force_mux && pins_suspend_force;
	int ret = 0;

	if (sleep_is_blocked()) {
		pr_info("suspend/resume: interrupted by modem(%d) or event(%d)\n",
			atomic_read(&block_sleep_modem),
			atomic_read(&block_sleep_event));
		return -EBUSY;
	}

	if (has_wake_lock(WAKE_LOCK_SUSPEND)) {
		pr_info("suspend/resume: wakelock has been locked!\n");
		return -EBUSY;
	}

	nmk_gpio_clocks_enable();

	ux500_suspend_dbg_add_wake_on_uart();
	nmk_gpio_wakeups_suspend();

	/* configure the prcm for a sleep wakeup */
	prcmu_enable_wakeups(PRCMU_WAKEUP(ABB) | PRCMU_WAKEUP(RTC));

	ux500_rtcrtt_next_seconds(alarm_sec);

	context_vape_save();

	if (pins_force) {
		/*
		 * Save GPIO settings before applying power save
		 * settings
		 */
		context_gpio_save();

		/* Apply GPIO power save mux settings */
		context_gpio_mux_safe_switch(true);
		pins_suspend_force_mux();
		context_gpio_mux_safe_switch(false);

		/* Apply GPIO power save settings */
		pins_suspend_force();
	}

	ux500_pm_gic_decouple();

	if (ux500_pm_gic_pending_interrupt()) {
		pr_info("suspend/resume: pending interrupt\n");

		/* Recouple GIC with the interrupt bus */
		ux500_pm_gic_recouple();
		ret = -EBUSY;

		goto exit;
	}
	ux500_pm_prcmu_set_ioforce(true);

	if (do_deepsleep) {
		context_varm_save_common();
		context_varm_save_core();
		context_gic_dist_disable_unneeded_irqs();
		context_save_cpu_registers();

		/*
		 * Due to we have only 100us between requesting a powerstate
		 * and wfi, we clean the cache before as well to assure the
		 * final cache clean before wfi has as little as possible to
		 * do.
		 */
		context_clean_l1_cache_all();

		(void) prcmu_set_power_state(PRCMU_AP_DEEP_SLEEP,
					     false, false);
		context_save_to_sram_and_wfi(true);

		context_restore_cpu_registers();
		context_varm_restore_core();
		context_varm_restore_common();

	} else {

		context_clean_l1_cache_all();
		(void) prcmu_set_power_state(APEXECUTE_TO_APSLEEP,
					     false, false);
		dsb();
		__asm__ __volatile__("wfi\n\t" : : : "memory");
	}

	context_vape_restore();

	/* If GPIO woke us up then save the pins that caused the wake up */
	ux500_pm_gpio_save_wake_up_status();

	ux500_suspend_dbg_sleep_status(do_deepsleep);

	/* APE was turned off, restore IO ring */
	ux500_pm_prcmu_set_ioforce(false);

exit:
	if (pins_force) {
		/* Restore gpio settings */
		context_gpio_mux_safe_switch(true);
		context_gpio_restore_mux();
		context_gpio_mux_safe_switch(false);
		context_gpio_restore();
	}

	ux500_rtcrtt_off();

	/* This is what cpuidle wants */
	prcmu_enable_wakeups(PRCMU_WAKEUP(ARM) | PRCMU_WAKEUP(RTC) |
			     PRCMU_WAKEUP(ABB));

	nmk_gpio_wakeups_resume();
	ux500_suspend_dbg_remove_wake_on_uart();

	nmk_gpio_clocks_disable();

	return ret;
}

static int ux500_suspend_enter(suspend_state_t state)
{

	if (ux500_suspend_enabled()) {
		if (ux500_suspend_deepsleep_enabled() &&
		    state == PM_SUSPEND_MEM)
			return suspend(true);
		if (ux500_suspend_sleep_enabled())
			return suspend(false);
		/* For debugging, if Sleep and DeepSleep disabled, do Idle */
		prcmu_set_power_state(PRCMU_AP_IDLE, true, true);
	}

	dsb();
	__asm__ __volatile__("wfi\n\t" : : : "memory");
	return 0;
}

static int ux500_suspend_valid(suspend_state_t state)
{
	return state == PM_SUSPEND_MEM || state == PM_SUSPEND_STANDBY;
}

static int ux500_suspend_prepare_late(void)
{
	/* ESRAM to retention instead of OFF until ROM is fixed */
	(void)prcmu_config_esram0_deep_sleep(ESRAM0_DEEP_SLEEP_STATE_RET);
	ab8500_regulator_debug_force();

	return 0;
}

static void ux500_suspend_wake(void)
{
	ab8500_regulator_debug_restore();
	(void)prcmu_config_esram0_deep_sleep(ESRAM0_DEEP_SLEEP_STATE_RET);
}

static struct platform_suspend_ops ux500_suspend_ops = {
	.enter	      = ux500_suspend_enter,
	.valid	      = ux500_suspend_valid,
	.prepare_late = ux500_suspend_prepare_late,
	.wake	      = ux500_suspend_wake,
	.begin	      = ux500_suspend_dbg_begin,
};

static __init int ux500_suspend_init(void)
{
	ux500_suspend_dbg_init();
	suspend_set_ops(&ux500_suspend_ops);
	return 0;
}

device_initcall(ux500_suspend_init);
