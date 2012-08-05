/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * Author: Rabin Vincent <rabin.vincent@stericsson.com> for ST-Ericsson
 * License terms: GNU General Public License (GPL), version 2.
 */

#ifndef __DEVICES_COMMON_H
#define __DEVICES_COMMON_H

#include <mach/uart.h>

extern struct amba_device *
dbx500_add_amba_device(const char *init_name, resource_size_t base,
		       int irq, void *pdata, unsigned int periphid,
		       const char *name);

extern struct platform_device *
dbx500_add_platform_device_4k1irq(const char *name, int id,
				  resource_size_t base,
				  int irq, void *pdata);

struct stm_msp_controller;

static inline struct amba_device *
dbx500_add_msp_spi(const char *name, resource_size_t base, int irq,
		   struct stm_msp_controller *pdata)
{
	return dbx500_add_amba_device(name, base, irq, pdata, MSP_PER_ID,
				      NULL);
}

struct pl022_ssp_controller;

static inline struct amba_device *
dbx500_add_spi(const char *init_name, resource_size_t base, int irq,
	       struct pl022_ssp_controller *pdata)
{
	const char *name = NULL;

	if (cpu_is_u5500())
		name = "db5500-spi";

	return dbx500_add_amba_device(init_name, base, irq, pdata, SPI_PER_ID,
				      name);
}

struct mmci_platform_data;

static inline struct amba_device *
dbx500_add_sdi(const char *name, resource_size_t base, int irq,
	       struct mmci_platform_data *pdata)
{
	return dbx500_add_amba_device(name, base, irq, pdata, SDI_PER_ID,
				      NULL);
}

static inline struct amba_device *
dbx500_add_uart(const char *name, resource_size_t base, int irq,
		struct uart_amba_plat_data *pdata)
{
	return dbx500_add_amba_device(name, base, irq, pdata, 0, NULL);
}

struct nmk_i2c_controller;

static inline struct platform_device *
dbx500_add_i2c(int id, resource_size_t base, int irq,
	       struct nmk_i2c_controller *pdata)
{
	return dbx500_add_platform_device_4k1irq("nmk-i2c", id, base, irq,
						 pdata);
}

struct msp_i2s_platform_data;

static inline struct platform_device *
dbx500_add_msp_i2s(int id, resource_size_t base, int irq,
		   struct msp_i2s_platform_data *pdata)
{
	return dbx500_add_platform_device_4k1irq("MSP_I2S", id, base, irq,
						 pdata);
}

static inline struct amba_device *
dbx500_add_rtc(resource_size_t base, int irq)
{
	return dbx500_add_amba_device("rtc-pl031", base, irq, NULL,
				      RTC_PER_ID, NULL);
}

#endif
