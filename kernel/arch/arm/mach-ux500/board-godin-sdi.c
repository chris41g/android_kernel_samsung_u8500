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
#include <mach/board-sec-u8500.h>


/*
 * SDI0 (SD/MMC card)
 */
#ifdef CONFIG_STE_DMA40
struct stedma40_chan_cfg sdi0_dma_cfg_rx = {
	.dir = STEDMA40_PERIPH_TO_MEM,
	.src_dev_type = DB8500_DMA_DEV29_SD_MM0_RX,
	.dst_dev_type = STEDMA40_DEV_DST_MEMORY,
	.src_info.data_width = STEDMA40_WORD_WIDTH,
	.dst_info.data_width = STEDMA40_WORD_WIDTH,
};

static struct stedma40_chan_cfg sdi0_dma_cfg_tx = {
	.dir = STEDMA40_MEM_TO_PERIPH,
	.src_dev_type = STEDMA40_DEV_SRC_MEMORY,
	.dst_dev_type = DB8500_DMA_DEV29_SD_MM0_TX,
	.src_info.data_width = STEDMA40_WORD_WIDTH,
	.dst_info.data_width = STEDMA40_WORD_WIDTH,
};
#endif

static struct mmci_platform_data ssg_sdi0_data = {
	.vcc		= "v-mmc",
	.vcard		= "v-MMC-SD",
	.disable	= 50,
	.f_max		= 50000000,
	.capabilities	= MMC_CAP_4_BIT_DATA |
				MMC_CAP_SD_HIGHSPEED |
				MMC_CAP_MMC_HIGHSPEED |
				MMC_CAP_DISABLE,
	.gpio_cd	= T_FLASH_DETECT_GTI9060_R0_1,
	.gpio_wp	= -1,
	.cd_invert	= true,

#ifdef CONFIG_STE_DMA40
	.dma_filter	= stedma40_filter,
	.dma_rx_param	= &sdi0_dma_cfg_rx,
	.dma_tx_param	= &sdi0_dma_cfg_tx,
#endif
};

static void __init sdi0_configure(void)
{
	switch ( system_rev )
	{
		case GTI9060_R0_1:
		{
			ssg_sdi0_data.gpio_cd = T_FLASH_DETECT_GTI9060_R0_1;
			ssg_sdi0_data.sigdir = MMCI_ST_DIRFBCLK |
									MMCI_ST_DIRCMD |
									MMCI_ST_DIRDAT0 |
									MMCI_ST_DIRDAT2;
		}
		break;
		default:
			pr_err("%s: Error - Unknown platform id %d\n", __func__, system_rev);
	}
}

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


static bool sdi1_card_power_on = false;
 
static unsigned int sdi1_card_status(struct device *dev)
{
	if (sdi1_card_power_on) 
		return 1;
	else
		return 0;
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

static struct mmci_platform_data ssg_sdi1_data = {
	.vcc		= "v-mmc",
	.disable	= 500,
	.ocr_mask	= MMC_VDD_29_30,
	.f_max		= 50000000,
	.capabilities	= MMC_CAP_4_BIT_DATA |
				MMC_CAP_DISABLE |
				MMC_CAP_SDIO_IRQ |
				MMC_CAP_BROKEN_SDIO_CMD53,
	.pm_flags	= MMC_PM_KEEP_POWER,
	.gpio_cd	= -1,
	.gpio_wp	= -1,
	.status = sdi1_card_status,
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


static void suspend_resume_handler_sdi2(struct mmc_host *host, bool suspend)
{

   if(suspend)
   	{
	printk("[MMC] TURN OFF EXTERNAL LDO\n");
	gpio_set_value(MEM_LDO_EN_GTI9060_R0_1, 0);
   	}
   else
   	{
   	printk("[MMC] TURN ON EXTERNAL LDO\n");
	/* Enable external LDO */
	gpio_set_value(MEM_LDO_EN_GTI9060_R0_1, 1);
   	}
		
}

static struct mmci_platform_data ssg_sdi2_data = {
	.vcc		= "v-mmc",
	.disable	= 50,
	.ocr_mask	= MMC_VDD_165_195,
	.f_max		= 50000000,
	.capabilities	= MMC_CAP_4_BIT_DATA |
				MMC_CAP_8_BIT_DATA |
				MMC_CAP_DISABLE |
				MMC_CAP_MMC_HIGHSPEED,
	.pm_flags	= MMC_PM_KEEP_POWER,
	.gpio_cd	= -1,
	.gpio_wp	= -1,
	.suspend_resume_handler	= suspend_resume_handler_sdi2,
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

static struct mmci_platform_data ssg_sdi4_data = {
	.vcc		= "v-mmc",
	.disable	= 500,
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


/* BCM */
static int wifi_gpio_reset	= WLAN_RST_N_GTI9060_R0_1;
static int wifi_gpio_irq	= WL_HOST_WAKE_GTI9060_R0_1;

static void godin_wifi_init(void)
{
	int32_t status = 0;

	if (GTI9060_R0_1 > system_rev)
		return; 

	/* Enable the WLAN GPIO */
	status = gpio_request(wifi_gpio_reset, "wlan_power");
	if (status)
	{
		printk("INIT : Unable to request GPIO_WLAN_ENABLE \n");
		return;
	}

	gpio_direction_output(wifi_gpio_reset, 0);

	if(gpio_request(wifi_gpio_irq, "wlan_irq"))
	{
		printk("Unable to request WLAN_IRQ \n");
		return;
	}

	if(gpio_direction_input(wifi_gpio_irq))
	{
		printk("Unable to set directtion on WLAN_IRQ \n");
		return;
	} 
	return;
}

static void godin_sdi2_init(void)
{
	int32_t status = 0;

	/* Enable the eMMC_EN GPIO */
	status = gpio_request(MEM_LDO_EN_GTI9060_R0_1, "eMMC_EN");

	gpio_direction_output(MEM_LDO_EN_GTI9060_R0_1, 1);
	gpio_set_value(MEM_LDO_EN_GTI9060_R0_1, 1);

	return;
}
int u8500_wifi_power(int on)
{
//	if (GTI9060_R0_1 > system_rev)
//		return 0 ; 

	printk("%s: %d\n", __func__, on);

	gpio_set_value(wifi_gpio_reset, on);
	sdi1_card_power_on = (on==0) ? false : true;

	return 0;
}


static int __init ssg_sdi_init(void)
{
	ssg_sdi2_data.card_sleep_on_suspend = true;
	db8500_add_sdi2(&ssg_sdi2_data);
	godin_sdi2_init();

	if ((sec_debug_settings & SEC_DBG_STM_VIA_SD_OPTS) == 0) {
		/* not tracing via SDI0 pins, so can enable SDI0 */
		sdi0_configure();
		db8500_add_sdi0(&ssg_sdi0_data);
	}

	db8500_add_sdi1(&ssg_sdi1_data);

	if (GTI9060_R0_1 <= system_rev)
	{
		/* BCM */
		godin_wifi_init();
	}
	return 0;
}


fs_initcall(ssg_sdi_init);

/*BCM*/
EXPORT_SYMBOL (u8500_wifi_power);

