/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * License Terms: GNU General Public License v2
 *
 * U5500 PRCMU API.
 */
#ifndef __MACH_PRCMU_U5500_H
#define __MACH_PRCMU_U5500_H

#ifdef CONFIG_U5500_PRCMU

void db5500_prcmu_early_init(void);

int db5500_prcmu_abb_read(u8 slave, u8 reg, u8 *value, u8 size);
int db5500_prcmu_abb_write(u8 slave, u8 reg, u8 *value, u8 size);

int db5500_prcmu_request_clock(u8 clock, bool enable);

int prcmu_resetout(u8 resoutn, u8 state);

static inline int prcmu_set_power_state(u8 state, bool keep_ulp_clk,
	bool keep_ap_pll)
{
	return 0;
}

static inline void prcmu_system_reset(u16 reset_code) {}

#else /* !CONFIG_U5500_PRCMU */

static inline void db5500_prcmu_early_init(void)
{
}

static inline int db5500_prcmu_abb_read(u8 slave, u8 reg, u8 *value, u8 size)
{
	return -ENOSYS;
}

static inline int db5500_prcmu_abb_write(u8 slave, u8 reg, u8 *value, u8 size)
{
	return -ENOSYS;
}

static inline int prcmu_resetout(u8 resoutn, u8 state)
{
	return 0;
}

#endif /* CONFIG_U5500_PRCMU */

static inline int db5500_prcmu_config_abb_event_readout(u32 abb_events)
{
#ifdef CONFIG_MACH_U5500_SIMULATOR
	return 0;
#else
	return -1;
#endif
}

#endif /* __MACH_PRCMU_U5500_H */
