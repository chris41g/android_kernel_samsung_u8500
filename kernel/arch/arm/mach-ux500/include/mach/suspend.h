/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * License terms: GNU General Public License (GPL) version 2
 */
#ifndef __MACH_SUSPEND_H
#define __MACH_SUSPEND_H

enum ux500_suspend_block_case {
	UX500_MODEM_BLOCK = 0,
	UX500_EVENT_BLOCK,
};

void suspend_block_sleep(int block_case);
void suspend_unblock_sleep(int block_case);

#ifdef CONFIG_UX500_SUSPEND
void suspend_set_pins_force_fn(void (*force)(void), void (*force_mux)(void));
#else
static inline void suspend_set_pins_force_fn(void (*force)(void),
					     void (*force_mux)(void)) { }
#endif

#endif /* __MACH_SUSPEND_H */
