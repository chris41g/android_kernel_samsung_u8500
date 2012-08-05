/*
 * Copyright (C) 2010 ST-Ericsson
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/amba/bus.h>
#include <linux/amba/mmci.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/core.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <plat/ste_dma40.h>
#include <plat/pincfg.h>
#include <mach/devices.h>
#include <mach/gpio.h>
#include <mach/ste-dma40-db8500.h>

#include "devices-db8500.h"
#include "pins-db8500.h"
#include "board-mop500.h"


/*
 * SDI0 (SD/MMC card)
 */
#ifdef CONFIG_STE_DMA40
struct stedma40_chan_cfg sdi0_dma_cfg_rx = {
	.dir = STEDMA40_PERIPH_TO_MEM,
	.src_dev_type = DB8500_DMA_DEV1_SD_MMC0_RX,
	.dst_dev_type = STEDMA40_DEV_DST_MEMORY,
	.src_info.data_width = STEDMA40_WORD_WIDTH,
	.dst_info.data_width = STEDMA40_WORD_WIDTH,
	.use_fixed_channel = true,
	.phy_channel = 0,
};

static struct stedma40_chan_cfg sdi0_dma_cfg_tx = {
	.dir = STEDMA40_MEM_TO_PERIPH,
	.src_dev_type = STEDMA40_DEV_SRC_MEMORY,
	.dst_dev_type = DB8500_DMA_DEV1_SD_MMC0_TX,
	.src_info.data_width = STEDMA40_WORD_WIDTH,
	.dst_info.data_width = STEDMA40_WORD_WIDTH,
	.use_fixed_channel = true,
	.phy_channel = 0,
};
#endif

static void sdi0_configure(void)
{
	int ret;
	int gpio[2];
	if (machine_is_hrefv60())	{
		gpio[0] = HREFV60_SDMMC_EN_GPIO;
		gpio[1] = HREFV60_SDMMC_1V8_3V_GPIO;
	} else	{
		gpio[0] = EGPIO_PIN_17;
		gpio[1] = EGPIO_PIN_18;
	}
	ret = gpio_request(gpio[0], "level shifter enable");
	if (!ret)
		ret = gpio_request(gpio[1], "level shifter select");

	if (ret) {
		printk(KERN_WARNING "unable to config gpios for level shifter.\n");
		return;
	}

	/* Select the default 2.9V and eanble level shifter */
	gpio_direction_output(gpio[1], 0);
	gpio_direction_output(gpio[0], 1);
}

static void sdi0_vdd_handler(struct device *dev, unsigned int vdd,
		unsigned char power_mode)
{
	int gpio[2];
	if (machine_is_hrefv60())	{
		gpio[0] = HREFV60_SDMMC_EN_GPIO;
		gpio[1] = HREFV60_SDMMC_1V8_3V_GPIO;
	} else	{
		gpio[0] = EGPIO_PIN_17;
		gpio[1] = EGPIO_PIN_18;
	}
	switch (power_mode) {
	case MMC_POWER_UP:
	case MMC_POWER_ON:
		/*
		 * Level shifter voltage should depend on vdd to when deciding
		 * on either 1.8V or 2.9V. Once the decision has been made the
		 * level shifter must be disabled and re-enabled with a changed
		 * select signal in order to switch the voltage. Since there is
		 * no framework support yet for indicating 1.8V in vdd, use the
		 * default 2.9V.
		 */
		gpio_direction_output(gpio[1], 0);
		/* Enable level shifter */
		gpio_direction_output(gpio[0], 1);
		udelay(100);
		break;
	case MMC_POWER_OFF:
		gpio_direction_output(gpio[1], 0);
		/* Disable level shifter */
		gpio_direction_output(gpio[0], 0);
		break;
	}
}

static struct mmci_platform_data mop500_sdi0_data = {
	.vcc		= "v-mmc",
	.vcard		= "v-MMC-SD",
	.vdd_handler	= sdi0_vdd_handler,
	.disable	= 50,
	.f_max		= 50000000,
	.capabilities	= MMC_CAP_4_BIT_DATA |
				MMC_CAP_SD_HIGHSPEED |
				MMC_CAP_MMC_HIGHSPEED |
				MMC_CAP_DISABLE,
	.gpio_wp	= -1,
	.sigdir = MMCI_ST_DIRFBCLK |
				MMCI_ST_DIRCMD |
				MMCI_ST_DIRDAT0 |
				MMCI_ST_DIRDAT2,
#ifdef CONFIG_STE_DMA40
	.dma_filter	= stedma40_filter,
	.dma_rx_param	= &sdi0_dma_cfg_rx,
	.dma_tx_param	= &sdi0_dma_cfg_tx,
#endif
};

/*
 * SDI1 (SDIO WLAN)
 */
static irqreturn_t sdio_irq(int irq, void *dev_id)
{
	struct mmc_host *host = dev_id;

	/*
	 * Signal an sdio irq for the sdio client.
	 * If we are in suspend state the host has been claimed
	 * by the pm framework, which prevents any sdio request
	 * to be served until the host gets resumed and released.
	 */
	mmc_signal_sdio_irq(host);

	return IRQ_HANDLED;
}

static void sdio_config_irq(struct mmc_host *host,
				bool cfg_as_dat1,
				int irq_pin,
				pin_cfg_t irq_dat1,
				pin_cfg_t irq_gpio)
{
	/*
	 * If the pin mux switch or interrupt registration fails we're in deep
	 * trouble and there is no decent recovery.
	 */
	if (!gpio_is_valid(irq_pin)) {
		dev_err(mmc_dev(host),
			"invalid irq pin (%d) for sdio irq\n", irq_pin);
		return;
	}

	if (!cfg_as_dat1) {

		/*
		 * SDIO irq shall be handled as a gpio irq. We configure the
		 * dat[1] as gpio and register a gpio irq handler.
		 */

		if (nmk_config_pin(irq_gpio, 0))
			dev_err(mmc_dev(host),
				"err config irq gpio (%lu) for sdio irq\n",
				irq_gpio);

		/* We use threaded irq */
		if (request_threaded_irq(gpio_to_irq(irq_pin),
					NULL,
					sdio_irq,
					IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
					dev_driver_string(mmc_dev(host)),
					host))
			dev_err(mmc_dev(host),
				"err request_threaded_irq for sdio irq\n");

		/*
		 * Set the irq as a wakeup irq to be able to be woken up
		 * from the suspend state and thus trigger a resume of
		 * the system.
		 */
		if (enable_irq_wake(gpio_to_irq(irq_pin)))
			dev_err(mmc_dev(host),
				"err enabling wakeup irq for sdio irq\n");

		/*
		 * Workaround to fix PL180 hw-problem of missing sdio irqs.
		 * If the DAT1 line has been asserted low we signal an sdio
		 * irq.
		 */
		if (gpio_request(irq_pin, dev_driver_string(mmc_dev(host))))
			dev_err(mmc_dev(host),
				"err requesting irq for sdio irq\n");

		if (!gpio_get_value(irq_pin))
			mmc_signal_sdio_irq(host);

		gpio_free(irq_pin);

	} else {

		/*
		 * SDIO irq shall be handled as dat[1] irq by the mmci
		 * driver. Configure the gpio back into dat[1] and remove
		 * the gpio irq handler.
		 */
		if (disable_irq_wake(gpio_to_irq(irq_pin)))
			dev_err(mmc_dev(host),
				"err disabling wakeup irq for sdio irq\n");

		free_irq(gpio_to_irq(irq_pin), host);

		if (nmk_config_pin(irq_dat1, 0))
			dev_err(mmc_dev(host),
				"err config irq dat1 (%lu) for sdio irq\n",
				irq_dat1);

	}
}

static void wakeup_handler_sdi1(struct mmc_host *host, bool wakeup)
{
	if (host->card && mmc_card_sdio(host->card) && host->sdio_irqs)
		sdio_config_irq(host,
				wakeup,
				212,
				GPIO212_MC1_DAT1 | PIN_INPUT_PULLUP,
				GPIO212_GPIO | PIN_INPUT_PULLUP);
}

#ifdef CONFIG_STE_DMA40
static struct stedma40_chan_cfg sdi1_dma_cfg_rx = {
	.dir = STEDMA40_PERIPH_TO_MEM,
	.src_dev_type = DB8500_DMA_DEV32_SD_MM1_RX,
	.dst_dev_type = STEDMA40_DEV_DST_MEMORY,
	.src_info.data_width = STEDMA40_WORD_WIDTH,
	.dst_info.data_width = STEDMA40_WORD_WIDTH,
};

static struct stedma40_chan_cfg sdi1_dma_cfg_tx = {
	.dir = STEDMA40_MEM_TO_PERIPH,
	.src_dev_type = STEDMA40_DEV_SRC_MEMORY,
	.dst_dev_type = DB8500_DMA_DEV32_SD_MM1_TX,
	.src_info.data_width = STEDMA40_WORD_WIDTH,
	.dst_info.data_width = STEDMA40_WORD_WIDTH,
};
#endif

static struct mmci_platform_data mop500_sdi1_data = {
	.vcc		= "v-mmc",
	.disable	= 50,
	.ocr_mask	= MMC_VDD_29_30,
	.f_max		= 50000000,
	.capabilities	= MMC_CAP_4_BIT_DATA |
				MMC_CAP_DISABLE |
				MMC_CAP_SDIO_IRQ |
				MMC_CAP_BROKEN_SDIO_CMD53,
	.pm_flags	= MMC_PM_KEEP_POWER,
	.gpio_cd	= -1,
	.gpio_wp	= -1,
	.wakeup_handler	= wakeup_handler_sdi1,
#ifdef CONFIG_STE_DMA40
	.dma_filter	= stedma40_filter,
	.dma_rx_param	= &sdi1_dma_cfg_rx,
	.dma_tx_param	= &sdi1_dma_cfg_tx,
#endif
};

/*
 * SDI2 (POPed eMMC)
 */
#ifdef CONFIG_STE_DMA40
struct stedma40_chan_cfg sdi2_dma_cfg_rx = {
	.dir = STEDMA40_PERIPH_TO_MEM,
	.src_dev_type = DB8500_DMA_DEV28_SD_MM2_RX,
	.dst_dev_type = STEDMA40_DEV_DST_MEMORY,
	.src_info.data_width = STEDMA40_WORD_WIDTH,
	.dst_info.data_width = STEDMA40_WORD_WIDTH,
};
static struct stedma40_chan_cfg sdi2_dma_cfg_tx = {
	.dir = STEDMA40_MEM_TO_PERIPH,
	.src_dev_type = STEDMA40_DEV_SRC_MEMORY,
	.dst_dev_type = DB8500_DMA_DEV28_SD_MM2_TX,
	.src_info.data_width = STEDMA40_WORD_WIDTH,
	.dst_info.data_width = STEDMA40_WORD_WIDTH,
};
#endif

static struct mmci_platform_data mop500_sdi2_data = {
	.vcc		= "v-mmc",
	.disable	= 50,
	.ocr_mask	= MMC_VDD_165_195,
	.f_max		= 50000000,
	.capabilities	= MMC_CAP_4_BIT_DATA |
				MMC_CAP_8_BIT_DATA |
				MMC_CAP_DISABLE,
	.pm_flags	= MMC_PM_KEEP_POWER,
	.gpio_cd	= -1,
	.gpio_wp	= -1,
#ifdef CONFIG_STE_DMA40
	.dma_filter	= stedma40_filter,
	.dma_rx_param	= &sdi2_dma_cfg_rx,
	.dma_tx_param	= &sdi2_dma_cfg_tx,
#endif
};

/*
 * SDI4 (On-board eMMC)
 */
#ifdef CONFIG_STE_DMA40
struct stedma40_chan_cfg sdi4_dma_cfg_rx = {
	.dir = STEDMA40_PERIPH_TO_MEM,
	.src_dev_type = DB8500_DMA_DEV42_SD_MM4_RX,
	.dst_dev_type = STEDMA40_DEV_DST_MEMORY,
	.src_info.data_width = STEDMA40_WORD_WIDTH,
	.dst_info.data_width = STEDMA40_WORD_WIDTH,
};

static struct stedma40_chan_cfg sdi4_dma_cfg_tx = {
	.dir = STEDMA40_MEM_TO_PERIPH,
	.src_dev_type = STEDMA40_DEV_SRC_MEMORY,
	.dst_dev_type = DB8500_DMA_DEV42_SD_MM4_TX,
	.src_info.data_width = STEDMA40_WORD_WIDTH,
	.dst_info.data_width = STEDMA40_WORD_WIDTH,
};
#endif

static struct mmci_platform_data mop500_sdi4_data = {
	.vcc		= "v-mmc",
	.vcard		= "v-eMMC",
	.disable	= 50,
	.f_max		= 50000000,
	.capabilities	= MMC_CAP_4_BIT_DATA |
				MMC_CAP_8_BIT_DATA |
				MMC_CAP_MMC_HIGHSPEED |
				MMC_CAP_DISABLE,
	.gpio_cd	= -1,
	.gpio_wp	= -1,
#ifdef CONFIG_STE_DMA40
	.dma_filter	= stedma40_filter,
	.dma_rx_param	= &sdi4_dma_cfg_rx,
	.dma_tx_param	= &sdi4_dma_cfg_tx,
#endif
};

static int __init mop500_sdi_init(void)
{
	/* POP eMMC on v1.0 has problems with high speed */
	if (!cpu_is_u8500v10())
		mop500_sdi2_data.capabilities |= MMC_CAP_MMC_HIGHSPEED;
	db8500_add_sdi2(&mop500_sdi2_data);

	db8500_add_sdi4(&mop500_sdi4_data);

	sdi0_configure();
	if (machine_is_hrefv60())
		mop500_sdi0_data.gpio_cd	= HREFV60_SDMMC_CD_GPIO;
	else
		mop500_sdi0_data.gpio_cd	= EGPIO_PIN_3;
	db8500_add_sdi0(&mop500_sdi0_data);

	db8500_add_sdi1(&mop500_sdi1_data);

	return 0;
}

fs_initcall(mop500_sdi_init);
