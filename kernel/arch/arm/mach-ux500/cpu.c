/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * Author: Rabin Vincent <rabin.vincent@stericsson.com> for ST-Ericsson
 * License terms: GNU General Public License (GPL) version 2
 */

#include <linux/platform_device.h>
#include <linux/amba/bus.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/mm.h>

#include <asm/cacheflush.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/hardware/gic.h>
#include <asm/mach/map.h>

#include <plat/mtu.h>
#include <mach/hardware.h>
#include <mach/setup.h>
#include <mach/devices.h>
#include <mach/prcmu.h>
#include <mach/prcmu-db8500.h>
#include <mach/sec_common.h>
#include <mach/reboot_reasons.h>
#include <mach/system.h>

#include "clock.h"

void __iomem *gic_cpu_base_addr;
void __iomem *_PRCMU_BASE;

extern void setup_mm_for_reboot( char mode ) ;

#ifdef CONFIG_CACHE_L2X0
static void __iomem *l2x0_base;
#endif

/*
 * The reboot reason string can be 255 characters long and the memory
 * in which we save the sw reset reason is 2 bytes. Therefore we need to
 * convert the string into a 16 bit pattern.
 *
 * See file reboot_reasons.h for conversion.
 */
static unsigned short map_cmd_to_code(const char *cmd)
{
	int i;

	if (cmd == NULL)
		/* normal reboot w/o argument */
		return SW_RESET_NO_ARGUMENT;

	/* Search through reboot reason list */
	for (i = 0; i < reboot_reasons_size; i++) {
		if (!strcmp(reboot_reasons[i].reason, cmd))
			return reboot_reasons[i].code;
	}

	/* No valid Reboot Reason found */
	return SW_RESET_CRASH;
}

void __init ux500_init_devices(void)
{
#ifdef CONFIG_CACHE_L2X0
	BUG_ON(!l2x0_base);

	/*
	 * Unlock Data and Instruction Lock if locked.  This is done here
	 * instead of in l2x0_init since doing it there appears to cause the
	 * second core boot to occasionaly fail.
	 */
	if (readl_relaxed(l2x0_base + L2X0_LOCKDOWN_WAY_D) & 0xFF)
		writel_relaxed(0x0, l2x0_base + L2X0_LOCKDOWN_WAY_D);

	if (readl_relaxed(l2x0_base + L2X0_LOCKDOWN_WAY_I) & 0xFF)
		writel_relaxed(0x0, l2x0_base + L2X0_LOCKDOWN_WAY_I);

#endif
}

#ifdef CONFIG_SAMSUNG_KERNEL_DEBUG
extern unsigned int unhandled_reset_count ;
#endif /*CONFIG_SAMSUNG_KERNEL_DEBUG*/

void ux500_restart(char mode, const char *cmd)
{
	unsigned short reset_code;
	int i ;
	printk("ux500_restart: Call arch_reset(), mode: %c, cmd: %s\n", mode, cmd );

#ifdef CONFIG_SAMSUNG_KERNEL_DEBUG
	if( 'L' == mode || 'U' == mode)
	{
		for( i = 0 ; i < 100 ; i++ ) {
			arch_reset( mode, NULL ) ;
			unhandled_reset_count++ ;
		}
	}
#endif /*CONFIG_SAMSUNG_KERNEL_DEBUG*/

	/*
	 * Clean and disable cache, and turn off interrupts
	 */
	cpu_proc_fin();

	/*
	 * Tell the mm system that we are going to reboot -
	 * we may need it to insert some 1:1 mappings so that
	 * soft boot works.
	 */
	setup_mm_for_reboot(mode);

	/*
	 * Now call the architecture specific reboot code.
	 */
	arch_reset(mode, cmd);

	/*
	 * Whoops - the architecture was unable to reboot.
	 * Tell the user!
	 */
	mdelay(1000);
	printk("Reboot via PRCMU failed -- System halted\n");
	while (1);
}

void __init ux500_init_irq(void)
{
	void __iomem *dist_base;

	if (cpu_is_u5500()) {
		gic_cpu_base_addr = __io_address(U5500_GIC_CPU_BASE);
		dist_base = __io_address(U5500_GIC_DIST_BASE);
	} else if (cpu_is_u8500()) {
		gic_cpu_base_addr = __io_address(U8500_GIC_CPU_BASE);
		dist_base = __io_address(U8500_GIC_DIST_BASE);
	} else
		ux500_unknown_soc();

	gic_dist_init(0, dist_base, 29);
	gic_cpu_init(0, gic_cpu_base_addr);

	/*
	 * Init clocks here so that they are available for system timer
	 * initialization.
	 */
	if (cpu_is_u5500())
		db5500_prcmu_early_init();
	else {
		prcmu_early_init();
		arm_pm_restart = ux500_restart;
	        printk("arm_pm_restart: 0x%x, ux500_restart: 0x%x\n", arm_pm_restart, ux500_restart);
	}

	clk_init();
}

#ifdef CONFIG_CACHE_L2X0
static inline void ux500_cache_wait(void __iomem *reg, unsigned long mask)
{
	/* wait for the operation to complete */
	while (readl_relaxed(reg) & mask)
		;
}

static inline void ux500_cache_sync(void)
{
	void __iomem *base = l2x0_base;

#ifdef CONFIG_ARM_ERRATA_753970
	writel_relaxed(0, base + L2X0_DUMMY_REG);
#else
	writel_relaxed(0, base + L2X0_CACHE_SYNC);
#endif
}

/* The L2 cache cannot be turned off in the non-secure world.
   Dummy until a secure service is in place */
static void ux500_l2x0_disable(void) {}

/* This is only called when doing a kexec, just after turning off the L2
   and L1 cache, and it is surrounded by a spinlock in the generic version.
   However, we're not really turning off the L2 cache right now and the
   PL310 does not support exclusive accesses (used to implement the spinlock).
   So, the invalidation needs to be done without the spinlock. */
static void ux500_l2x0_inv_all(void)
{
	void __iomem *base = l2x0_base;
	uint32_t l2x0_way_mask = (1<<16) - 1;	/* Bitmask of active ways */

	/* invalidate all ways */
	writel_relaxed(l2x0_way_mask, base + L2X0_INV_WAY);
	ux500_cache_wait(base + L2X0_INV_WAY, l2x0_way_mask);
	ux500_cache_sync();
}

static int ux500_l2x0_init(void)
{
	if (cpu_is_u5500())
		l2x0_base = __io_address(U5500_L2CC_BASE);
	else if (cpu_is_u8500())
		l2x0_base = __io_address(U8500_L2CC_BASE);
	else
		ux500_unknown_soc();

	/* 64KB way size, 8 way associativity, force WA */
	l2x0_init(l2x0_base, 0x3e060000, 0xc0000fff);

	/* Override invalidate function */
	outer_cache.disable = ux500_l2x0_disable;
	outer_cache.inv_all = ux500_l2x0_inv_all;

	return 0;
}
early_initcall(ux500_l2x0_init);
#endif

