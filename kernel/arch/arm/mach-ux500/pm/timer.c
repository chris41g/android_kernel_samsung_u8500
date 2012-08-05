/*
 * Copyright (C) ST-Ericsson SA 2010-2011
 *
 * Author: Jonas Aaberg <jonas.aberg@stericsson.com> for ST-Ericsson
 *
 * License Terms: GNU General Public License v2
 *
 * The RTC timer block is a ST Microelectronics variant of ARM PL031.
 * Clockwatch part is the same as PL031, while the timer part is only
 * present on the ST Microelectronics variant.
 * Here only the timer part is used.
 *
 * The timer part is quite troublesome to program correctly. Lots
 * of long delays must be there in order to secure that you actually get what
 * you wrote.
 *
 * In other words, this timer is and should only used from cpuidle during
 * special conditions when the surroundings are know in order to be able
 * to remove the number of delays.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/ktime.h>
#include <linux/delay.h>

#include <asm/errno.h>

#include <mach/hardware.h>

#include "cpuidle_dbg.h"

#define RTC_IMSC	0x10
#define RTC_MIS		0x18
#define RTC_ICR		0x1C
#define RTC_TDR		0x20
#define	RTC_TLR1	0x24
#define RTC_TCR		0x28

#define RTC_TLR2	0x2C
#define RTC_TPR1	0x3C

#define RTC_TCR_RTTOS	(1 << 0)
#define RTC_TCR_RTTEN	(1 << 1)
#define RTC_TCR_RTTSS	(1 << 2)

#define RTC_IMSC_TIMSC	(1 << 1)
#define RTC_ICR_TIC	(1 << 1)
#define RTC_MIS_RTCTMIS	(1 << 1)

#define RTC_TCR_RTTPS_2	(1 << 4)
#define RTC_TCR_RTTPS_3	(2 << 4)
#define RTC_TCR_RTTPS_4	(3 << 4)
#define RTC_TCR_RTTPS_5	(4 << 4)
#define RTC_TCR_RTTPS_6	(5 << 4)
#define RTC_TCR_RTTPS_7	(6 << 4)
#define RTC_TCR_RTTPS_8	(7 << 4)

#define WRITE_DELAY 130 /* 4 cycles plus margin */

/*
 * Count down measure point. It just have to be high to differ
 * from scheduled values.
 */
#define MEASURE_VAL 0xffffffff

/* Just a value bigger than any reason able scheduled timeout. */
#define MEASURE_VAL_LIMIT 0xf0000000


#define TICKS_TO_NS(x) ((s64)x * 30512)
#define US_TO_TICKS(x) ((u32)((1000 * x) / 30512))

static void __iomem *rtc_base;
static bool measure_latency;

#ifdef CONFIG_U8500_CPUIDLE_DEBUG

/*
 * The plan here is to be able to measure the ApSleep/ApDeepSleep exit latency
 * by having a know timer pattern.
 * The first entry in the pattern, LR1, is the value that the scheduler
 * wants us to sleep. The second pattern in a high value, too large to be
 * scheduled, so we can differ between a running scheduled value and a
 * time measure value.
 * When a RTT interrupt has occured, the block will automatically start
 * to execute the measure value in LR2 and when the ARM is awake, it reads
 * how far the RTT has decreased the value loaded from LR2 and from that
 * calculate how long time it took to wake up.
 */
ktime_t u8500_rtc_exit_latency_get(void)
{
	u32 ticks;

	if (measure_latency) {
		ticks = MEASURE_VAL - readl(rtc_base + RTC_TDR);

		/*
		 * Check if we are actually counting on a LR2 value.
		 * If not we have woken on another interrupt.
		 */
		if (ticks < MEASURE_VAL_LIMIT) {
			/* convert 32 kHz ticks to ns */
			return ktime_set(0, TICKS_TO_NS(ticks));
		}
	}
	return ktime_set(0, 0);
}

static void measure_latency_start(void)
{
	udelay(WRITE_DELAY);
	/*
	 * Disable RTT and clean self-start due to we want to restart,
	 * not continue from current pattern. (See below)
	 */
	writel(0, rtc_base + RTC_TCR);
	udelay(WRITE_DELAY);

	/*
	 * Program LR2 (load register two) to maximum value to ease
	 * identification of timer interrupt vs other.
	 */
	writel(MEASURE_VAL, rtc_base + RTC_TLR2);
	/*
	 * Set Load Register execution pattern, bit clear
	 * means pick LR1, bit set means LR2
	 * 0xfe, binary 11111110 means first do LR1 then do
	 * LR2 seven times
	 */
	writel(0xfe, rtc_base + RTC_TPR1);

	udelay(WRITE_DELAY);

	/*
	 * Enable self-start, plus a pattern of eight.
	 */
	writel(RTC_TCR_RTTSS | RTC_TCR_RTTPS_8,
	       rtc_base + RTC_TCR);
	udelay(WRITE_DELAY);
}

void ux500_rtcrtt_measure_latency(bool enable)
{
	if (enable) {
		measure_latency_start();
	} else {
		writel(RTC_TCR_RTTSS | RTC_TCR_RTTOS, rtc_base + RTC_TCR);
		writel(RTC_ICR_TIC, rtc_base + RTC_ICR);
		writel(RTC_IMSC_TIMSC, rtc_base + RTC_IMSC);
	}
	measure_latency = enable;
}
#else
static inline void measure_latency_start(void) { }
static inline void ux500_rtcrtt_measure_latency(bool enable) { }
#endif

void ux500_rtcrtt_off(void)
{
	if (measure_latency) {
		measure_latency_start();
	} else {
		/* Clear eventual interrupts */
		if (readl(rtc_base + RTC_MIS) & RTC_MIS_RTCTMIS)
			writel(RTC_ICR_TIC, rtc_base + RTC_ICR);

		/* Disable, self start and oneshot mode */
		writel(RTC_TCR_RTTSS | RTC_TCR_RTTOS, rtc_base + RTC_TCR);
	}
}

void ux500_rtcrtt_next(u32 time_us)
{
	writel(US_TO_TICKS(time_us), rtc_base + RTC_TLR1);
}

void ux500_rtcrtt_next_seconds(u32 sec)
{
	const unsigned int max_value = UINT_MAX/32768;
	if (sec > max_value) {
		pr_err("timer-rtt: %s : To large value %u s, "
			"timer set to max value %u s!\n"
			, __func__, sec, max_value);
			sec = max_value;
	}
	writel(sec * 32768, rtc_base + RTC_TLR1);
}

static int __init ux500_rtcrtt_init(void)
{
	if (cpu_is_u8500()) {
		rtc_base  = __io_address(U8500_RTC_BASE);
	} else if (cpu_is_u5500()) {
		rtc_base  = __io_address(U5500_RTC_BASE);
	} else {
		pr_err("timer-rtt: Unknown DB Asic!\n");
		return -EINVAL;
	}
	ux500_rtcrtt_measure_latency(false);
	return 0;
}
subsys_initcall(ux500_rtcrtt_init);

