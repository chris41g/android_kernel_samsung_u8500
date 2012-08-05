/*
 * Copyright (C) 2007 STMicroelectronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/hsi.h>
#include <linux/err.h>
#include <mach/hsi-stm.h>
#include <mach/debug.h>
#include <linux/dmaengine.h>

char hsi_controller_string[] = "HSI-CONTROLLER";

static void __exit hsi_close_all_ch(struct hsi_controller *hsi_ctrlr)
{
	unsigned int i;

	if (hsi_ctrlr->dev_type == 0x0) {
		for (i = 0; i < hsi_ctrlr->max_ch; i++) {
			dma_release_channel(hsi_ctrlr->hsi_tx_channels[i].
					    dma_chan);
			hsi_ctrlr->algo->close(hsi_ctrlr->hsi_tx_channels[i].
					       dev);
		}
	} else {
		for (i = 0; i < hsi_ctrlr->max_ch; i++) {
			dma_release_channel(hsi_ctrlr->hsi_rx_channels[i]
					    .dma_chan);
			hsi_ctrlr->algo->close(hsi_ctrlr->hsi_rx_channels[i].
					       dev);
		}
	}
}

static void __init start_hsi_controller(struct platform_device *pdev)
{
	struct hsi_plat_data *pdata = NULL;
	struct hsi_controller *hsi_ctrlr = NULL;
	void __iomem *hsi_base;
	unsigned char n_ch;

	pdata = (struct hsi_plat_data *)pdev->dev.platform_data;
	hsi_ctrlr = pdata->controller;
	n_ch = hsi_ctrlr->max_ch;
	hsi_base = hsi_ctrlr->regbase;

	if (!(pdev->id)) {
		/** HSI transmit */
		/** flush all the channel buffers */
		writel(0x0, hsi_base + HSI_TX_BUFSTATE);
		/** change mode to FRAME*/
		writel((0x3 & 0x2), hsi_base + HSI_TX_MODE);
		/** enable watermark interrupt mask and destination 0
		writel(((1<<n_ch)-1),hsi_base+HSI_TX_WATERMARKIM);*/
		writel(0x0, hsi_base + HSI_TX_WATERMARKID);
	} else {
		/** flush the pipeline buffers */
		writel(0x0, hsi_base + HSI_RX_STATE);
		writel(0x0, hsi_base + HSI_RX_PIPEGAUGE);
		writel(0x0, hsi_base + HSI_RX_BUFSTATE);

		/** change mode to PIPELINED */
		/*writel((0x3 & 0x3),hsi_base+HSI_RX_MODE);*/

		/** MOP changes :change mode to FRAME */
		writel((0x3 & 0x2), hsi_base + HSI_RX_MODE);

		/** enable watermark interrupt mask and destination 0 */
		/** ..not done right now, done during data transfer /
		writel(((1<<n_ch)-1),hsi_base+HSI_RX_WATERMARKIM);*/
		writel(0x0, hsi_base + HSI_RX_WATERMARKID);
		/** for testing purpose
		writel(0xAAAAAAAA,hsi_base+HSI_RX_BUFFERX);
		writel(0xBBBBBBBB,hsi_base+HSI_RX_BUFFERX+0x4);
		writel(0xCCCCCCCC,
		hsi_base+HSI_RX_BUFFERX+0x8); */

		/** enable the exeception interrupts -
		 * parity,timeout,BREAK,overrun */
		writel(0xF, hsi_base + HSI_RX_EXCEPIM);
		/** enable channel overrun interrupt for 4 channels */
		writel(0xF, hsi_base + HSI_RX_OVERRUNIM);
	}
}

static void __init hsi_initialise_hw(struct platform_device *pdev)
{
	struct hsi_plat_data *pdata = NULL;
	struct hsi_controller *hsi_ctrlr = NULL;
	struct hsi_channel *txch, *rxch;
	void __iomem *hsi_base;
	u8 i;
	u8 n_ch;
	u32 partnum0;
	u32 des0partnum1;
	u32 revdes1;
	u32 config;

	pdata = (struct hsi_plat_data *)pdev->dev.platform_data;
	hsi_ctrlr = pdata->controller;
	n_ch = hsi_ctrlr->max_ch;
	hsi_base = hsi_ctrlr->regbase;

	if (!(pdev->id)) {
		/** HSI transmit */
		/** channel id and priority */
		writel((0xf & pdata->channels), hsi_base + HSI_TX_CHANNELS);
		writel((0xff & pdata->priority), hsi_base + HSI_TX_PRIORITY);
		/** channels framelen,base and span data */
		for (i = 0; i < n_ch; i++) {
			writel((0x1f & pdata->framelen),
			       hsi_base + HSI_TX_FRAMELENX + (0x4 * i));
			writel((0x1f & pdata->ch_base_span[i].base),
			       (hsi_base + HSI_TX_BASEX + (0x4 * i)));
			writel((0x1f & pdata->ch_base_span[i].span),
			       (hsi_base + HSI_TX_SPANX + (0x4 * i)));
		}

		/** serial link settings */
		/** sleep mode for now */
		writel((0x3 & 0x0), hsi_base + HSI_TX_MODE);
		writel((0xffffff & pdata->divisor), hsi_base + HSI_TX_DIVISOR);
		writel((0x3 & pdata->parity), hsi_base + HSI_TX_PARITY);
		writel((0x3f & pdata->flushbits), hsi_base + HSI_TX_FLUSHBITS);
		writel((0xffffff & pdata->burstlen),
		       hsi_base + HSI_TX_BURSTLEN);
		writel((0xffffff & pdata->preamble),
		       hsi_base + HSI_TX_PREAMBLE);

		/** watermark and interrupt settings */
		for (i = 0; i < n_ch; i++) {
			/* For MOP */
			txch = &hsi_ctrlr->hsi_tx_channels[i];
			writel((0x1f & (txch->watermark - 1)),
			       (hsi_base + HSI_TX_WATERMARKX + (0x4 * i)));
		}

		/** watermark interrupt mask - mask all interrupts now*/
		writel(0x0, hsi_base + HSI_TX_WATERMARKIM);

		/** Gather info from registers for the driver.(REVISION) */
		partnum0 = readl(hsi_base + HSI_TX_PERIPHID0);
		des0partnum1 = readl(hsi_base + HSI_TX_PERIPHID1);
		revdes1 = readl(hsi_base + HSI_TX_PERIPHID2);
		config = readl(hsi_base + HSI_TX_PERIPHID3);

		stm_dbg(DBG_ST.hsi,
			"HSIT Hardware VERSION (in hex) %x.%x.%x.%x.%x.%x\n",
			(partnum0 & 0xff), (des0partnum1 & 0xf0),
			(des0partnum1 & 0x0f), (revdes1 & 0xf0),
			(revdes1 & 0x0f), (config & 0xff));

	} else {
		/** HSI receive*/

		/** channel id and priority */
		writel((0xf & pdata->channels), hsi_base + HSI_RX_CHANNELS);
		for (i = 0; i < pdata->channels; i++) {
			/** channels framelen,base and span data */
			writel((0x1f & pdata->framelen),
			       (hsi_base + HSI_RX_FRAMELENX + (0x4 * i)));
			writel((0x1f & pdata->ch_base_span[i].base),
			       (hsi_base + HSI_RX_BASEX + (0x4 * i)));
			writel((0x1f & pdata->ch_base_span[i].span),
			       (hsi_base + HSI_RX_SPANX + (0x4 * i)));
		}

		/** serial link settings */
		  /** sleep mode for now */
		writel((0x3 & 0x0), hsi_base + HSI_RX_MODE);
		writel((0xffffff & pdata->detector),
		       hsi_base + HSI_RX_DETECTOR);
		writel((0x3 & pdata->parity), hsi_base + HSI_RX_PARITY);
		writel((0xffffff & pdata->preamble),
		       hsi_base + HSI_RX_PREAMBLE);

		/** watermark settings */
		for (i = 0; i < pdata->channels; i++) {
			/* For MOP */
			rxch = &hsi_ctrlr->hsi_rx_channels[i];
			writel((0x1f & (rxch->watermark - 1)),
			       (hsi_base + HSI_RX_WATERMARKX + (0x4 * i)));
		}
		/** watermark interrupt mask - mask all interrupts now*/
		writel(0x0, hsi_base + HSI_RX_WATERMARKIM);
		writel(0x0, hsi_base + HSI_RX_OVERRUNIM);
		writel(0x0, hsi_base + HSI_RX_EXCEPIM);

		/** threshold,realtime and timeout settings */
		writel((0x3f & pdata->threshold), hsi_base + HSI_RX_THRESHOLD);
		writel(pdata->realtime, hsi_base + HSI_RX_REALTIME);
		writel(pdata->timeout, hsi_base + HSI_RX_TIMEOUT);

		/** Gather info from registers for the driver.(REVISION) */
		partnum0 = readl(hsi_base + HSI_RX_PERIPHID0);
		des0partnum1 = readl(hsi_base + HSI_RX_PERIPHID1);
		revdes1 = readl(hsi_base + HSI_RX_PERIPHID2);
		config = readl(hsi_base + HSI_RX_PERIPHID3);

		stm_dbg(DBG_ST.hsi,
			"HSIR Hardware VERSION (in hex) %x.%x.%x.%x.%x.%x\n",
			(partnum0 & 0xff), (des0partnum1 & 0xf0),
			(des0partnum1 & 0x0f), (revdes1 & 0xf0),
			(revdes1 & 0x0f), (config & 0xff));

	}
}

static int __init hsi_initialise_irq(struct hsi_controller *hsi_ctrlr)
{
	int err = 0;
	int i = 0, j = 0;
	char irqname[20];

	if (!(hsi_ctrlr->dev_type)) {
		/** tx */
		tasklet_init(&hsi_ctrlr->hsi_tx_tasklet, do_hsi_tx_tasklet,
			     (unsigned long)hsi_ctrlr);
		err =
		    request_irq(hsi_ctrlr->irq1, hsi_tx_irq_handler,
				IRQF_DISABLED, "hsi-tx-d0", hsi_ctrlr);
		if (err < 0) {
			stm_dbg(DBG_ST.hsi,
			"Unable to allocate HSI tx interrupt line\n");
			err = -EBUSY;
			tasklet_disable(&hsi_ctrlr->hsi_tx_tasklet);
		}
	} else {
		/** rx */

		tasklet_init(&hsi_ctrlr->hsi_rx_tasklet, do_hsi_rx_tasklet,
			     (unsigned long)hsi_ctrlr);
		err =
		    request_irq(hsi_ctrlr->irq1, hsi_rx_irq_handler,
				IRQF_DISABLED, "hsi-rx-d0", hsi_ctrlr);
		if (err < 0) {
			stm_dbg(DBG_ST.hsi,
				"Unable to allocate HSI rx interrupt line\n");
			tasklet_disable(&hsi_ctrlr->hsi_rx_tasklet);
			err = -EBUSY;
			return err;
		}

		tasklet_init(&hsi_ctrlr->hsi_rxexcep_tasklet,
			     do_hsi_rxexcep_tasklet, (unsigned long)hsi_ctrlr);
		err =
		    request_irq(hsi_ctrlr->irqexcep, hsi_rxexcep_irq_handler,
				IRQF_DISABLED, "hsi-rx-excep", hsi_ctrlr);
		if (err < 0) {
			stm_dbg(DBG_ST.hsi, "Unable to allocate HSI rx\
				exception interrupt line\n");
			err = -EBUSY;
			tasklet_disable(&hsi_ctrlr->hsi_rx_tasklet);
			tasklet_disable(&hsi_ctrlr->hsi_rxexcep_tasklet);
			free_irq(hsi_ctrlr->irq1, hsi_ctrlr);
			return err;
		}
		for (i = 0; i < hsi_ctrlr->max_ch; i++) {
			snprintf(irqname, sizeof(irqname), "hsi-rx-ch%d-ovrn",
				 i);
			err =
			    request_irq(hsi_ctrlr->irq_choverrun[i],
					hsi_rxexcep_irq_handler, IRQF_DISABLED,
					"hsi-rx-overrun", hsi_ctrlr);
			if (err < 0) {
				stm_dbg(DBG_ST.hsi,
					"Unable to allocate HSI rx\
					overrun interrupt line\n");
				err = -EBUSY;
				tasklet_disable(&hsi_ctrlr->hsi_rx_tasklet);
				tasklet_disable(&hsi_ctrlr->
						hsi_rxexcep_tasklet);
				free_irq(hsi_ctrlr->irq1, hsi_ctrlr);
				free_irq(hsi_ctrlr->irqexcep, hsi_ctrlr);
				for (j = 0; j < i; j++)
					free_irq(hsi_ctrlr->irq_choverrun[j],
						 hsi_ctrlr);
			}

		}
	}

	return err;
}

static void free_hsi_irq(struct hsi_controller *hsi_ctrlr)
{
	int i = 0;

	if (!(hsi_ctrlr->dev_type)) {
		/** tx */
		tasklet_disable(&hsi_ctrlr->hsi_tx_tasklet);
		free_irq(hsi_ctrlr->irq1, hsi_ctrlr);
	} else {
		tasklet_disable(&hsi_ctrlr->hsi_rx_tasklet);
		tasklet_disable(&hsi_ctrlr->hsi_rxexcep_tasklet);
		free_irq(hsi_ctrlr->irq1, hsi_ctrlr);
		free_irq(hsi_ctrlr->irqexcep, hsi_ctrlr);
		for (i = 0; i < hsi_ctrlr->max_ch; i++)
			free_irq(hsi_ctrlr->irq_choverrun[i], hsi_ctrlr);
	}

}

int fill_hsi_dma_info(struct platform_device *pd, struct hsi_channel *ch)
{

	struct hsi_controller *hsi_ctrlr;
	struct hsi_plat_data *pdata;
	dma_cap_mask_t mask;

	pdata = (struct hsi_plat_data *)pd->dev.platform_data;
	hsi_ctrlr = pdata->controller;

	/*FIXME can it be made single register*/
	if (pd->id) {
		/* find and request free dma chanel */
		dma_cap_zero(mask);
		dma_cap_set(DMA_SLAVE, mask);
		ch->dma_chan = dma_request_channel(mask, stedma40_filter,
						   ch->hsi_dma_info);
		if (ch->dma_chan == NULL) {
			stm_error("RX pipe request failed\n");
			return -EBUSY;
		}
		tasklet_init(&ch->hsi_dma_tasklet, do_hsi_rx_dma_tasklet,
			     (unsigned long)ch);

	} else {
		/* find and request free dma chanel for tx */

		dma_cap_zero(mask);
		dma_cap_set(DMA_SLAVE, mask);
		ch->dma_chan = dma_request_channel(mask, stedma40_filter,
						   ch->hsi_dma_info);
		if (ch->dma_chan == NULL) {
			stm_error("RX pipe request failed\n");
			return -EBUSY;
		}

		tasklet_init(&ch->hsi_dma_tasklet, do_hsi_tx_dma_tasklet,
			     (unsigned long)ch);

	}
	return 0;
}

static void __init hsi_initialise_channels(struct platform_device *pdev)
{

	u8 ch = 0;
	u8 n_ch = 0;
	struct hsi_plat_data *pdata = NULL;
	struct hsi_controller *hsi_ctrlr = NULL;
	/** temp pointer to populate rxchannel array */
	struct hsi_channel *txch = NULL;
	struct hsi_channel *rxch = NULL;

	pdata = (struct hsi_plat_data *)pdev->dev.platform_data;
	hsi_ctrlr = pdata->controller;
	n_ch = hsi_ctrlr->max_ch;

	if (!(pdev->id)) {
		/** HSI transmit */
		for (ch = 0; ch < n_ch; ch++) {
			txch = &hsi_ctrlr->hsi_tx_channels[ch];
			txch->channel_number = ch;
			txch->span = (pdata->ch_base_span[ch].span) + 1;
			txch->watermark = pdata->watermark;
			txch->flags = 0;
			txch->ctrlr = hsi_ctrlr;
			txch->write_data.data = NULL;
			txch->write_data.count = 0;
			txch->hsi_dma_info = &pdata->hsi_dma_info[ch];
			txch->n_bytes = 0x1;
			fill_hsi_dma_info(pdev, txch);
			spin_lock_init(&txch->hsi_ch_lock);
		}
	} else {
		/** HSI receive */
		for (ch = 0; ch < n_ch; ch++) {
			rxch = &hsi_ctrlr->hsi_rx_channels[ch];
			rxch->channel_number = ch;
			rxch->span = (pdata->ch_base_span[ch].span) + 1;
			rxch->watermark = pdata->watermark;
			rxch->flags = 0;
			rxch->ctrlr = hsi_ctrlr;
			rxch->read_data.data = NULL;
			rxch->read_data.count = 0;
			rxch->hsi_dma_info = &pdata->hsi_dma_info[ch];
			rxch->n_bytes = 0x1;
			fill_hsi_dma_info(pdev, rxch);
			spin_lock_init(&rxch->hsi_ch_lock);
		}

	}
}

static void __init hsi_initialise_controller(struct platform_device *pdev)
{
	struct hsi_plat_data *pdata = NULL;
	struct hsi_controller *hsi_ctrlr = NULL;

	pdata = (struct hsi_plat_data *)pdev->dev.platform_data;

	hsi_ctrlr = pdata->controller;
	hsi_ctrlr->max_ch = pdata->channels;
	hsi_ctrlr->mode = pdata->mode;
	hsi_ctrlr->flags = 0;
	spin_lock_init(&hsi_ctrlr->lock);
}

static int __init hsi_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct hsi_controller *hsi_ctrlr = NULL;
	struct hsi_plat_data *pdata = NULL;
	unsigned char name[20];
	int err = 0;
	int i = 0;

	if ((pdev == NULL) || (pdev->dev.platform_data == NULL)) {
		stm_dbg(DBG_ST.hsi,
			"No device/platform_data found on HSI device\n");
		return -ENODEV;
	}

	hsi_ctrlr = kzalloc(sizeof(struct hsi_controller), GFP_KERNEL);
	if (hsi_ctrlr == NULL) {
		stm_dbg(DBG_ST.hsi,
			"Could not allocate memory for controller struct\n");
		return -ENOMEM;
	}

	hsi_ctrlr->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(hsi_ctrlr->clk)) {
		stm_dbg(DBG_ST.hsi, "Could not get HSI clock\n");
		err = PTR_ERR(hsi_ctrlr->clk);
		goto rollback_alloc;
	}

	/** initialise the HSI controller and channels (tx and rx) */
	hsi_ctrlr->dev_type = pdev->id;
	memcpy(hsi_ctrlr->name, hsi_controller_string,
	   strlen(hsi_controller_string)), pdata = pdev->dev.platform_data;
	pdata->controller = hsi_ctrlr;
	hsi_ctrlr->dev_type = pdata->dev_type;

	err = stm_gpio_altfuncenable(pdata->gpio_alt_func);
	if (err) {
		stm_dbg(DBG_ST.hsi, "Could not set HSI GPIO\
			alternate function correctly\n");
		err = -ENODEV;
		goto rollback_clock;
	}

	hsi_initialise_controller(pdev);
	hsi_initialise_channels(pdev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		stm_dbg(DBG_ST.hsi,
			"Could not get HSI IO memory information\n");
		err = -ENODEV;
		goto rollback_gpio;
	}

	hsi_ctrlr->regbase =
	    (void __iomem *)ioremap(res->start, res->end - res->start + 1);
	if (!(hsi_ctrlr->regbase)) {
		stm_dbg(DBG_ST.hsi, "Unable to map register base \n");
		err = -EBUSY;
		goto rollback_gpio;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		stm_dbg(DBG_ST.hsi, "Unable to map HSI D0/D1 IRQ base \n");
		err = -EBUSY;
		goto rollback_map;
	}

	/** storing D0 and D1 interrupt lines */
	if (res->end - res->start) {
		hsi_ctrlr->irq1 = res->start;
		hsi_ctrlr->irq2 = res->end;
	}

	/** store the HSIR exception interrupt and channel
	 * overrun interrupt line */
	if (hsi_ctrlr->dev_type == 0x1) {
		/** rx */
		res = platform_get_resource(pdev, IORESOURCE_IRQ, 1);
		if (!res) {
			stm_dbg(DBG_ST.hsi,
				"Unable to map HSIR EXCEP IRQ base \n");
			err = -EBUSY;
			goto rollback_map;
		}
		hsi_ctrlr->irqexcep = res->start;

		for (i = 0; i < hsi_ctrlr->max_ch; i++) {
			res =
			    platform_get_resource(pdev, IORESOURCE_IRQ, i + 2);
			if (!res) {
				stm_dbg(DBG_ST.hsi, "Unable to map HSIR\
					CHANNEL %d OVERRUN IRQ LINE \n",
					i);
				err = -EBUSY;
				goto rollback_map;
			}
			hsi_ctrlr->irq_choverrun[i] = res->start;
		}
	}

	/** install handlers and tasklets */
	if (hsi_initialise_irq(hsi_ctrlr)) {
		stm_dbg(DBG_ST.hsi, "HSI error in interrupt registration \n");
		goto rollback_map;
	}

	/** write to hardware but do not start yet*/
	hsi_initialise_hw(pdev);

	hsi_ctrlr->algo = &hsi_algo;
	/** register devices on hsi bus */
	err = hsi_add_controller(hsi_ctrlr);
	if (err < 0) {
		stm_dbg(DBG_ST.hsi, "HSI error in device registration \n");
		goto rollback_irq;
	}

	/** kick start the controller */
	start_hsi_controller(pdev);

	snprintf(name, sizeof(name), "%s:%d", pdev->name, pdev->id);
	if (!err)
		printk(KERN_NOTICE"%s probe COMPLETED \n", name);

	return err;

rollback_irq:
	free_hsi_irq(hsi_ctrlr);
rollback_map:
	iounmap(hsi_ctrlr->regbase);
rollback_gpio:
	stm_gpio_altfuncdisable(pdata->gpio_alt_func);
rollback_clock:
	clk_put(hsi_ctrlr->clk);
rollback_alloc:
	pdata->controller = NULL;
	kfree(hsi_ctrlr);
	return err;
}

static int __exit hsi_remove(struct platform_device *pdev)
{
	struct hsi_plat_data *pdata = NULL;
	struct hsi_controller *hsi_ctrlr = NULL;

	pdata = (struct hsi_plat_data *)pdev->dev.platform_data;
	hsi_ctrlr = pdata->controller;
	hsi_close_all_ch(hsi_ctrlr);
	hsi_remove_controller(hsi_ctrlr);
	stm_gpio_altfuncdisable(pdata->gpio_alt_func);
	free_hsi_irq(hsi_ctrlr);
	iounmap(hsi_ctrlr->regbase);
	clk_put(hsi_ctrlr->clk);
	pdata->controller = NULL;
	kfree(hsi_ctrlr);

	return 0;
}
#ifdef CONFIG_PM
/**
 * hsi_suspend - HSI suspend function registered with PM framework.
 * @pdev: Reference to platform device structure of the device
 * @state: power mgmt state.
 *
 * This function is invoked when the system is going into sleep, called
 * by the power management framework of the linux kernel.
 * Nothing is required as controller is configured with every transfer.
 * It is assumed that no active tranfer is in progress at this time.
 * Client driver should make sure of this.
 *
 */

int hsi_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct hsi_plat_data *pdata = NULL;
	struct hsi_controller *hsi_ctrlr = NULL;
	struct hsi_backup_regs *backup_regs;
	void __iomem *hsi_base;
	int i;

	pdata = (struct hsi_plat_data *)pdev->dev.platform_data;
	hsi_ctrlr = pdata->controller;
	hsi_base = hsi_ctrlr->regbase;
	backup_regs = &hsi_ctrlr->backup_regs;
	if(pdev->id) {
	for(i = 0;i < HSI_MAX_CHANNELS;i++) {
			backup_regs->framelen[i] =
				readl(hsi_base + HSI_RX_WATERMARKX + (i*4));
			backup_regs->watermark[i] =
				readl(hsi_base + HSI_RX_FRAMELENX + (i*4));
		}
		backup_regs->rx_threshold =
			readl(hsi_base + HSI_RX_THRESHOLD);
	} else {
		for(i = 0;i < HSI_MAX_CHANNELS;i++) {
			backup_regs->watermark[i] =
				readl(hsi_base + HSI_TX_WATERMARKX + (i*4));
			backup_regs->framelen[i] =
				readl(hsi_base + HSI_TX_FRAMELENX + (i*4));
		}
	}
	return 0;
}
/**
 * hsi_resume - HSI Resume function registered with PM framework.
 * @pdev: Reference to platform device structure of the device
 *
 * This function is invoked when the system is coming out of sleep, called
 * by the power management framework of the linux kernel.
 * Nothing is required.
 *
 */

int hsi_resume(struct platform_device *pdev)
{
	struct hsi_plat_data *pdata = NULL;
	struct hsi_controller *hsi_ctrlr = NULL;
	struct hsi_backup_regs *backup_regs;
	void __iomem *hsi_base;
	int i;

	pdata = (struct hsi_plat_data *)pdev->dev.platform_data;
	hsi_ctrlr = pdata->controller;
	backup_regs = &hsi_ctrlr->backup_regs;
	hsi_base = hsi_ctrlr->regbase;
	hsi_initialise_hw(pdev);
	start_hsi_controller(pdev);

	if(pdev->id) {
		for(i = 0;i < HSI_MAX_CHANNELS;i++) {
			writel(backup_regs->framelen[i],
					hsi_base + HSI_RX_WATERMARKX + (i*4));
			writel(backup_regs->watermark[i],
					hsi_base + HSI_RX_FRAMELENX + (i*4));
		}
		writel(backup_regs->rx_threshold,
				hsi_base + HSI_RX_THRESHOLD);
	} else {
		for(i = 0;i < HSI_MAX_CHANNELS;i++) {
			writel(backup_regs->watermark[i],
					hsi_base + HSI_TX_WATERMARKX + (i*4));
			writel(backup_regs->framelen[i],
					hsi_base + HSI_TX_FRAMELENX + (i*4));
		}
	}
	return 0;

}
#else
#define hsi_suspend NULL
#define hsi_resume NULL
#endif
static struct platform_driver hsi_driver = {
	.probe = hsi_probe,
	.remove = __exit_p(hsi_remove),
	.suspend = hsi_suspend,
	.resume = hsi_resume,
	.driver = {
		   .name = "stm-hsi",
		   .owner = THIS_MODULE,
		   }
};

static int __init hsi_driver_init(void)
{
	int err = 0;

	err = platform_driver_probe(&hsi_driver, hsi_probe);
	if (err < 0) {
		stm_dbg(DBG_ST.hsi,
			"HSI Platform DRIVER register FAILED: %d\n",
			err);
		return err;
	}

	return 0;
}

static void __exit hsi_driver_exit(void)
{
	platform_driver_unregister(&hsi_driver);

	printk(KERN_NOTICE"HSI Platform DRIVER removed\n");
}

module_init(hsi_driver_init);
module_exit(hsi_driver_exit);

MODULE_AUTHOR(HSI_DRIVER_AUTHOR);
MODULE_DESCRIPTION(HSI_DRIVER_DESC);
MODULE_LICENSE("GPL");
