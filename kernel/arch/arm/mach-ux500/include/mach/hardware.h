/*
 * Copyright (C) 2009 ST-Ericsson.
 *
 * U8500 hardware definitions
 *
 * This file is licensed under  the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#ifndef __MACH_HARDWARE_H
#define __MACH_HARDWARE_H

/* macros to get at IO space when running virtually
 * We dont map all the peripherals, let ioremap do
 * this for us. We map only very basic peripherals here.
 */
#define U8500_IO_VIRTUAL	0xf0000000
#define U8500_IO_PHYSICAL	0xa0000000

/* this macro is used in assembly, so no cast */
#define IO_ADDRESS(x)           \
	(((x) & 0x0fffffff) + (((x) >> 4) & 0x0f000000) + U8500_IO_VIRTUAL)

/* typesafe io address */
#define __io_address(n)		__io(IO_ADDRESS(n))
/* used by some plat-nomadik code */
#define io_p2v(n)		__io_address(n)

#include <mach/db8500-regs.h>
#include <mach/db5500-regs.h>

/* ST-Ericsson modified pl022 id */
#define SSP_PER_ID                      0x01080022
#define SSP_PER_MASK                    0x0fffffff

/* SSP specific declaration */
#define SPI_PER_ID                      0x00080023
#define SPI_PER_MASK                    0x0fffffff

/* MSP specific declaration */
#define MSP_PER_ID			0x00280021
#define MSP_PER_MASK			0x00ffffff

/* DMA specific declaration */
#define DMA_PER_ID			0x8A280080
#define DMA_PER_MASK			0xffffffff

#define GPIO_PER_ID                     0x1f380060
#define GPIO_PER_MASK                   0xffffffff

/* RTC specific declaration */
#define RTC_PER_ID                      0x00280031
#define RTC_PER_MASK                    0x00ffffff

/*
 * FIFO offsets for IPs
 */
#define I2C_TX_REG_OFFSET	(0x10)
#define I2C_RX_REG_OFFSET	(0x18)
#define UART_TX_RX_REG_OFFSET	(0)
#define MSP_TX_RX_REG_OFFSET	(0)
#define SSP_TX_RX_REG_OFFSET	(0x8)
#define SPI_TX_RX_REG_OFFSET	(0x8)
#define SD_MMC_TX_RX_REG_OFFSET (0x80)
#define CRYP1_RX_REG_OFFSET	(0x10)
#define CRYP1_TX_REG_OFFSET	(0x8)

#define MSP_0_CONTROLLER 1
#define MSP_1_CONTROLLER 2
#define MSP_2_CONTROLLER 3
#define MSP_3_CONTROLLER 4

#define SSP_0_CONTROLLER 4
#define SSP_1_CONTROLLER 5

#define SPI023_0_CONTROLLER 6
#define SPI023_1_CONTROLLER 7
#define SPI023_2_CONTROLLER 8
#define SPI023_3_CONTROLLER 9

/* MSP related board specific declaration************************/

#define MSP_DATA_DELAY       MSP_DELAY_0
#define MSP_TX_CLOCK_EDGE    MSP_FALLING_EDGE
#define MSP_RX_CLOCK_EDGE    MSP_FALLING_EDGE
#define NUM_MSP_CONTROLLER 3

/* I2C configuration
 *  *  *
 *   *   */
#define I2C0_LP_OWNADDR 0x31
#define I2C1_LP_OWNADDR 0x60
#define I2C2_LP_OWNADDR 0x70
#define I2C3_LP_OWNADDR 0x80
#define I2C4_LP_OWNADDR 0x90

/* SDMMC specific declarations */
#define SDI_PER_ID		0x00480180
#define SDI_PER_MASK		0x00ffffff
/* B2R2 clock management register */
#define PRCM_B2R2CLK_MGT_REG	0x80157078 /** B2R2 clock selection */

#ifndef __ASSEMBLY__

extern void __iomem *_PRCMU_BASE;

#include <asm/cputype.h>
#include <asm/mach-types.h>

static inline bool ux500_is_svp(void)
{
	return machine_is_svp8500v1() ||
	       machine_is_svp8500v2() ||
	       machine_is_svp5500();
}
static inline bool cpu_is_u8500(void)
{
#ifdef CONFIG_UX500_SOC_DB8500
	return 1;
#else
	return 0;
#endif
}

#define CPUID_DB8500ED	0x410fc090
#define CPUID_DB8500V1	0x411fc091
#define CPUID_DB8500V2	0x412fc091

static inline bool cpu_is_u8500ed(void)
{
	/*
	 * SVP8500 unfortunately does not update the MIDR register on silicon
	 * revisions, but instead maintains the old ED value.
	 */
	if (ux500_is_svp())
		return false;

	return cpu_is_u8500() && (read_cpuid_id() == CPUID_DB8500ED);
}

static inline bool cpu_is_u8500v1(void)
{
	if (machine_is_svp8500v1())
		return true;
	else if (machine_is_svp8500v2())
		return false;

	return cpu_is_u8500() && (read_cpuid_id() == CPUID_DB8500V1);
}

static inline bool cpu_is_u8500v2(void)
{
	if (machine_is_svp8500v1())
		return false;
	else if (machine_is_svp8500v2())
		return true;

	return cpu_is_u8500() && (read_cpuid_id() == CPUID_DB8500V2);
}

#ifdef CONFIG_UX500_SOC_DB8500
bool cpu_is_u8500v10(void);
bool cpu_is_u8500v11(void);
bool cpu_is_u8500v20(void);
bool cpu_is_u8500v21(void);
bool cpu_is_u8500v22(void);
bool cpu_is_u8500v20_or_later(void);
#else
static inline bool cpu_is_u8500v10(void) { return false; }
static inline bool cpu_is_u8500v11(void) { return false; }
static inline bool cpu_is_u8500v20(void) { return false; }
static inline bool cpu_is_u8500v21(void) { return false; }
static inline bool cpu_is_u8500v22(void) { return false; }
static inline bool cpu_is_u8500v20_or_later(void) { return false; }
#endif

static inline bool cpu_is_u5500(void)
{
#ifdef CONFIG_UX500_SOC_DB5500
	return 1;
#else
	return 0;
#endif
}

#endif

/* Keep this greppable for SoC porters */
#define ux500_unknown_soc() BUG()

#endif				/* __MACH_HARDWARE_H */
