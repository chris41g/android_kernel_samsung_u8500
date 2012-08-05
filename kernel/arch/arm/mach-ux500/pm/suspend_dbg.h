/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * License Terms: GNU General Public License v2
 *
 * Author: Jonas Aaberg <jonas.aberg@stericsson.com> for ST-Ericsson
 *
 */

#ifndef UX500_SUSPEND_DBG_H
#define UX500_SUSPEND_DBG_H

#include <linux/kernel.h>
#include <linux/suspend.h>

#ifdef CONFIG_UX500_SUSPEND_DBG_WAKE_ON_UART
void ux500_suspend_dbg_add_wake_on_uart(void);
void ux500_suspend_dbg_remove_wake_on_uart(void);
#else
static inline void ux500_suspend_dbg_add_wake_on_uart(void) { }
static inline void ux500_suspend_dbg_remove_wake_on_uart(void) { }
#endif

#ifdef CONFIG_UX500_SUSPEND_DBG
bool ux500_suspend_enabled(void);
bool ux500_suspend_sleep_enabled(void);
bool ux500_suspend_deepsleep_enabled(void);
void ux500_suspend_dbg_sleep_status(bool is_deepsleep);
void ux500_suspend_dbg_init(void);
int ux500_suspend_dbg_begin(suspend_state_t state);

#else
static inline bool ux500_suspend_enabled(void)
{
	return true;
}
static inline bool ux500_suspend_sleep_enabled(void)
{
#ifdef CONFIG_UX500_SUSPEND_STANDBY
	return true;
#else
	return false;
#endif
}
static inline bool ux500_suspend_deepsleep_enabled(void)
{
#ifdef CONFIG_UX500_SUSPEND_MEM
	return true;
#else
	return false;
#endif
}
static inline void ux500_suspend_dbg_sleep_status(bool is_deepsleep) { }
static inline void ux500_suspend_dbg_init(void) { }

#define ux500_suspend_dbg_begin NULL

#endif

#endif
