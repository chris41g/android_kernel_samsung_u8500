/*----------------------------------------------------------------------------------*/
/*  copyright STMicroelectronics, 2007.                                            */
/*                                                                                  */
/* This program is free software; you can redistribute it and/or modify it under    */
/* the terms of the GNU General Public License as published by the Free             */
/* Software Foundation; either version 2.1 of the License, or (at your option)      */
/* any later version.                                                               */
/*                                                                                  */
/* This program is distributed in the hope that it will be useful, but WITHOUT      */
/* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS    */
/* FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.   */
/*                                                                                  */
/* You should have received a copy of the GNU General Public License                */
/* along with this program. If not, see <http://www.gnu.org/licenses/>.             */
/*----------------------------------------------------------------------------------*/


#include <linux/err.h>
#include "hsi_driver.h"
#include <linux/dmaengine.h>
#include <plat/ste_dma40.h>

static void hsi_dev_release(struct device *dev)
{
}

static void __exit hsi_close_all_ch(struct hsi_dev *hsi_ctrlr)
{
	unsigned int i;

	if (hsi_ctrlr->dev_type==0x0) {
		for (i=0;i<hsi_ctrlr->max_ch;i++)
			hsi_close(hsi_ctrlr->hsi_tx_channels[i].dev);
	}
	else {
		for (i=0;i<hsi_ctrlr->max_ch;i++)
			hsi_close(hsi_ctrlr->hsi_rx_channels[i].dev);
	}
}

static void __init start_hsi_controller(struct platform_device* pdev)
{
	struct hsi_plat_data *pdata = NULL;
	struct hsi_dev *hsi_ctrlr =NULL;
	void __iomem *hsi_base;
	unsigned char n_ch;

	pdata = (struct hsi_plat_data *) pdev->dev.platform_data;
	hsi_ctrlr = pdata->hsi_ctrlr;
	n_ch = hsi_ctrlr->max_ch;
	hsi_base=hsi_ctrlr->regbase;


	if (!(pdev->id)) {
		/** HSI transmit */
		/** flush all the channel buffers */
		writel(0x0,hsi_base+HSI_TX_BUFSTATE);
		/** change mode to FRAME*/
		writel((0x3 & 0x2),hsi_base+HSI_TX_MODE);
		/** enable watermark interrupt mask and destination 0*/
		//writel(((1<<n_ch)-1),hsi_base+HSI_TX_WATERMARKIM);
		writel(0x0,hsi_base+HSI_TX_WATERMARKID);
	}
	else {
		/** flush the pipeline buffers */
		writel(0x0,hsi_base+HSI_RX_STATE);
		writel(0x0,hsi_base+HSI_RX_PIPEGAUGE);
		writel(0x0,hsi_base+HSI_RX_BUFSTATE);

		/** change mode to FRAME*/
		writel((0x3 & 0x2),hsi_base+HSI_RX_MODE);

		/** enable watermark interrupt mask and destination 0 */
		/** ..not done right now, done during data transfer */
		//writel(((1<<n_ch)-1),hsi_base+HSI_RX_WATERMARKIM);
		writel(0x0,hsi_base+HSI_RX_WATERMARKID);
		/** for testing purpose */
		//writel(0xAAAAAAAA,hsi_base+HSI_RX_BUFFERX);
		//writel(0xBBBBBBBB,hsi_base+HSI_RX_BUFFERX+0x4);
		//writel(0xCCCCCCCC,hsi_base+HSI_RX_BUFFERX+0x8);

		/** enable the exeception interrupts - parity,timeout,BREAK,overrun */
		writel(0xF,hsi_base+HSI_RX_EXCEPIM);
		/** enable channel overrun interrupt for 4 channels */
		writel(0xF,hsi_base+HSI_RX_OVERRUNIM);
	}
}

static int __init register_hsi_devices(struct platform_device* pdev)
{
	struct hsi_plat_data *pdata = NULL;
	struct hsi_dev *hsi_ctrlr =NULL;
	unsigned int n_ch = 0;
	int ch = 0;
	int err = 0;
	struct hsi_device *dev;


	pdata = (struct hsi_plat_data *) pdev->dev.platform_data;
	hsi_ctrlr = pdata->hsi_ctrlr;
	n_ch = hsi_ctrlr->max_ch;


	if (!(pdev->id)) /** HSI transmit */{
		for (ch = 0; ((ch < n_ch) && (err >= 0)); ch++)
		{
			dev = kzalloc(sizeof(*dev), GFP_KERNEL);
			if (!dev) {
				err= -ENOMEM;
				break;
			}
			dev->ctrlrid = pdev->id;
			dev->chid = ch;
			dev->curr_mode = pdata->currmode;
			dev->ch = &pdata->hsi_ctrlr->hsi_tx_channels[ch];
			pdata->hsi_ctrlr->hsi_tx_channels[ch].dev = dev;
			dev->device.bus = &hsi_bus_type;
			dev->device.parent = &pdev->dev;
			dev->device.release = hsi_dev_release;
	                snprintf(dev->device.bus_id, sizeof(dev->device.bus_id),
				                        "hsitx-ch%u",ch);
			err = device_register(&dev->device);
		}
	}
	else {
		for (ch = 0; ((ch < n_ch) && (err >= 0)); ch++)
		{
			dev = kzalloc(sizeof(*dev), GFP_KERNEL);
			if (!dev) {
				err= -ENOMEM;
				break;
			}
			dev->ctrlrid = pdev->id;
			dev->chid = ch;
			dev->curr_mode = pdata->currmode;
			dev->ch = &pdata->hsi_ctrlr->hsi_rx_channels[ch];
			pdata->hsi_ctrlr->hsi_rx_channels[ch].dev = dev;
			dev->device.bus = &hsi_bus_type;
			dev->device.parent = &pdev->dev;
			dev->device.release = hsi_dev_release;
	                snprintf(dev->device.bus_id, sizeof(dev->device.bus_id),
				                        "hsirx-ch%u",ch);
			err = device_register(&dev->device);
		}
	}
	return err;
}

static void __init hsi_initialise_hw(struct platform_device* pdev)
{
	struct hsi_plat_data *pdata = NULL;
	struct hsi_dev *hsi_ctrlr =NULL;
	struct hsi_channel *txch,*rxch;
	void __iomem *hsi_base;
	u8 i;
	u8 n_ch;
	volatile u32 partnum0;
	volatile u32 des0partnum1;
	volatile u32 revdes1;
	volatile u32 config;


	pdata = (struct hsi_plat_data *) pdev->dev.platform_data;
	hsi_ctrlr = pdata->hsi_ctrlr;
	n_ch = hsi_ctrlr->max_ch;
	hsi_base=hsi_ctrlr->regbase;

	if (!(pdev->id)) {
		/** HSI transmit */
		/** channel id and priority */
		writel((0xf & pdata->channels),hsi_base+HSI_TX_CHANNELS);
		writel((0xff & pdata->priority),hsi_base+HSI_TX_PRIORITY);
		/** channels framelen,base and span data */
		for (i=0;i < n_ch;i++) {
			writel((0x1f & pdata->framelen),hsi_base+HSI_TX_FRAMELENX + (0x4*i));
			writel((0x1f & pdata->ch_base_span[i].base),(hsi_base+HSI_TX_BASEX + (0x4*i)));
			writel((0x1f & pdata->ch_base_span[i].span),(hsi_base+HSI_TX_SPANX + (0x4*i)));
		}

		/** serial link settings */
		/** sleep mode for now */
		writel((0x3 & 0x0),hsi_base+HSI_TX_MODE);
		writel((0xffffff & pdata->divisor),hsi_base+HSI_TX_DIVISOR);
		writel((0x3 & pdata->parity),hsi_base+HSI_TX_PARITY);
		writel((0x3f & pdata->flushbits),hsi_base+HSI_TX_FLUSHBITS);
		writel((0xffffff & pdata->burstlen),hsi_base+HSI_TX_BURSTLEN);
		writel((0xffffff & pdata->preamble),hsi_base+HSI_TX_PREAMBLE);

		/** watermark and interrupt settings */
		for (i=0;i<n_ch;i++) {
			/** watermark fixed to 4 free entries */
			txch = &hsi_ctrlr->hsi_tx_channels[i];
			writel((0x1f &  (txch->watermark-1)),(hsi_base+HSI_TX_WATERMARKX + (0x4*i)));
		}

		/** watermark interrupt mask - mask all interrupts now*/
		writel(0x0,hsi_base+HSI_TX_WATERMARKIM);


		/** Gather info from registers for the driver.(REVISION) */
		partnum0 = readl(hsi_base+HSI_TX_PERIPHID0);
		des0partnum1 = readl(hsi_base+HSI_TX_PERIPHID1);
		revdes1 = readl(hsi_base+HSI_TX_PERIPHID2);
		config = readl(hsi_base+HSI_TX_PERIPHID3);

		printk(KERN_INFO "HSIT Hardware VERSION (in hex) %x.%x.%x.%x.%x.%x\n",
				(partnum0 & 0xff) , (des0partnum1 & 0xf0),(des0partnum1 & 0x0f),
				(revdes1 & 0xf0),(revdes1 & 0x0f),(config & 0xff));


	}
	else  {
		/** HSI receive*/

		/** channel id and priority */
		writel((0xf & pdata->channels),hsi_base+HSI_RX_CHANNELS);
		for (i=0;i<pdata->channels;i++) {
			/** channels framelen,base and span data */
			writel((0x1f & pdata->framelen),(hsi_base+HSI_RX_FRAMELENX+ (0x4*i)));
			writel((0x1f & pdata->ch_base_span[i].base),(hsi_base+HSI_RX_BASEX + (0x4*i)));
			writel((0x1f & pdata->ch_base_span[i].span),(hsi_base+HSI_RX_SPANX + (0x4*i)));
		}

		/** serial link settings */
		  /** sleep mode for now */
		writel((0x3 & 0x0),hsi_base+HSI_RX_MODE);
		writel((0xffffff & pdata->detector),hsi_base+HSI_RX_DETECTOR);
		writel((0x3 & pdata->parity),hsi_base+HSI_RX_PARITY);
		writel((0xffffff & pdata->preamble),hsi_base+HSI_RX_PREAMBLE);

		/** watermark settings */
		for (i=0;i<pdata->channels;i++) {
			/** watermark fixed to 1 occupied entries */
			rxch = &hsi_ctrlr->hsi_rx_channels[i];
			writel((0x1f &  (rxch->watermark-1)),(hsi_base+HSI_RX_WATERMARKX + (0x4*i)));
		}

		/** watermark interrupt mask - mask all interrupts now*/
		writel(0x0,hsi_base+HSI_RX_WATERMARKIM);
		writel(0x0,hsi_base+HSI_RX_OVERRUNIM);
		writel(0x0,hsi_base+HSI_RX_EXCEPIM);

		/** threshold,realtime and timeout settings */
		writel((0x3f & pdata->threshold),hsi_base+HSI_RX_THRESHOLD);
		writel(pdata->realtime,hsi_base+HSI_RX_REALTIME);
		writel(pdata->timeout,hsi_base+HSI_RX_TIMEOUT);


		/** Gather info from registers for the driver.(REVISION) */
		partnum0 = readl(hsi_base+HSI_RX_PERIPHID0);
		des0partnum1 = readl(hsi_base+HSI_RX_PERIPHID1);
		revdes1 = readl(hsi_base+HSI_RX_PERIPHID2);
		config = readl(hsi_base+HSI_RX_PERIPHID3);

		printk(KERN_INFO "HSIR Hardware VERSION (in hex) %x.%x.%x.%x.%x.%x\n",
				(partnum0 & 0xff) , (des0partnum1 & 0xf0),(des0partnum1 & 0x0f),
				(revdes1 & 0xf0),(revdes1 & 0x0f),(config & 0xff));


	}
}


static int __init hsi_initialise_irq(struct hsi_dev* hsi_ctrlr)
{
	int err = 0;
	int i=0,j=0;
	char irqname[20];

	if (!(hsi_ctrlr->dev_type)) /** tx */ {
		tasklet_init(&hsi_ctrlr->hsi_tx_tasklet,do_hsi_tx_tasklet,(unsigned long)hsi_ctrlr);
		err = request_irq(hsi_ctrlr->irq1,hsi_tx_irq_handler,IRQF_DISABLED,"hsi-tx-d0",hsi_ctrlr);
		if (err < 0) {
			printk(KERN_INFO "Unable to allocate HSI tx interrupt line\n");
			err = -EBUSY;
			tasklet_disable(&hsi_ctrlr->hsi_tx_tasklet);
		}
	}
	else /** rx */ {
		tasklet_init(&hsi_ctrlr->hsi_rx_tasklet,do_hsi_rx_tasklet,(unsigned long)hsi_ctrlr);
		err = request_irq(hsi_ctrlr->irq1,hsi_rx_irq_handler,IRQF_DISABLED,"hsi-rx-d0",hsi_ctrlr);
		if (err < 0) {
			printk(KERN_INFO "Unable to allocate HSI rx interrupt line\n");
			tasklet_disable(&hsi_ctrlr->hsi_rx_tasklet);
			err = -EBUSY;
			return err;
		}

		tasklet_init(&hsi_ctrlr->hsi_rxexcep_tasklet,do_hsi_rxexcep_tasklet,(unsigned long)hsi_ctrlr);
		err = request_irq(hsi_ctrlr->irqexcep,hsi_rxexcep_irq_handler,IRQF_DISABLED,"hsi-rx-excep",hsi_ctrlr);
		if (err < 0) {
			printk(KERN_INFO "Unable to allocate HSI rx exception interrupt line\n");
			err = -EBUSY;
			tasklet_disable(&hsi_ctrlr->hsi_rx_tasklet);
			tasklet_disable(&hsi_ctrlr->hsi_rxexcep_tasklet);
			free_irq(hsi_ctrlr->irq1,hsi_ctrlr);
			return err;
		}
		for (i=0;i<hsi_ctrlr->max_ch;i++) {
			snprintf(irqname,sizeof(irqname),"hsi-rx-ch%d-ovrn",i);
			err = request_irq(hsi_ctrlr->irq_choverrun[i],hsi_rxexcep_irq_handler,
								IRQF_DISABLED,"hsi-rx-overrun",hsi_ctrlr);
			if (err < 0) {
				printk(KERN_INFO "Unable to allocate HSI rx overrun interrupt line\n");
				err = -EBUSY;
				tasklet_disable(&hsi_ctrlr->hsi_rx_tasklet);
				tasklet_disable(&hsi_ctrlr->hsi_rxexcep_tasklet);
				free_irq(hsi_ctrlr->irq1,hsi_ctrlr);
				free_irq(hsi_ctrlr->irqexcep,hsi_ctrlr);
				for (j=0;j<i;j++)
					free_irq(hsi_ctrlr->irq_choverrun[j],hsi_ctrlr);
			}

		}
	}

	return err;
}


static void free_hsi_irq(struct hsi_dev *hsi_ctrlr)
{
	int i=0;
	int j=0;

	if (!(hsi_ctrlr->dev_type)) /** tx */ {
		tasklet_disable(&hsi_ctrlr->hsi_tx_tasklet);
		free_irq(hsi_ctrlr->irq1,hsi_ctrlr);
	}
	else {
		tasklet_disable(&hsi_ctrlr->hsi_rx_tasklet);
		tasklet_disable(&hsi_ctrlr->hsi_rxexcep_tasklet);
		free_irq(hsi_ctrlr->irq1,hsi_ctrlr);
		free_irq(hsi_ctrlr->irqexcep,hsi_ctrlr);
		for (i=0;i<hsi_ctrlr->max_ch;i++)
			free_irq(hsi_ctrlr->irq_choverrun[i],hsi_ctrlr);
	}

}

int fill_hsi_dma_info(struct platform_device *pd, struct hsi_channel *ch) {

	struct hsi_dev *hsi_ctrlr;
	struct hsi_plat_data *pdata;
	dma_cap_mask_t mask;

	pdata = (struct hsi_plat_data *) pd->dev.platform_data;
	hsi_ctrlr=pdata->hsi_ctrlr;

	//FIXME can it be made single register
	if(pd->id)
	{

		/* find and request free dma chanel */
		#if 1
		dma_cap_zero(mask);
		dma_cap_set(DMA_SLAVE, mask);

		ch->dma_chan = dma_request_channel(mask, stedma40_filter,
						   ch->hsi_dma_info);
		if (!ch->dma_chan) {
			printk(KERN_INFO "hsi: dma_request_channel for RX failed for channel %d\n",
			       ch->channel_number);
			return -EBUSY;
		}
		#endif
		tasklet_init(&ch->hsi_dma_tasklet, do_hsi_rx_dma_tasklet,
				(unsigned long)ch);
	}
	else
	{
				/* find and request free dma chanel for tx */
		#if 1
		dma_cap_zero(mask);
		dma_cap_set(DMA_SLAVE, mask);

		ch->dma_chan = dma_request_channel(mask, stedma40_filter,
						   ch->hsi_dma_info);
		if (!ch->dma_chan) {
			printk(KERN_INFO "hsi: dma_request_channel for TX failed for channel %d\n",
			       ch->channel_number);
			return -EBUSY;
		}
		#endif
		tasklet_init(&ch->hsi_dma_tasklet, do_hsi_tx_dma_tasklet,
				(unsigned long)ch);
	}
	return 0;
}



static void __init hsi_initialise_channels(struct platform_device* pdev)
{

	u8 ch = 0;
	u8 n_ch = 0;
	struct hsi_plat_data *pdata = NULL;
	struct hsi_dev *hsi_ctrlr =NULL;
	struct hsi_channel* txch = NULL; /** temp pointer to populate txchannel array */
	struct hsi_channel* rxch = NULL; /** temp pointer to populate rxchannel array */

	pdata = (struct hsi_plat_data *) pdev->dev.platform_data;
	hsi_ctrlr = pdata->hsi_ctrlr;
	n_ch = hsi_ctrlr->max_ch;

	if (!(pdev->id)) {
		/** HSI transmit */
		for (ch=0;ch < n_ch; ch++) {
			txch = &hsi_ctrlr->hsi_tx_channels[ch];
			txch->channel_number = ch;
			txch->span = (pdata->ch_base_span[ch].span)+1;
			txch->watermark = pdata->watermark;
			txch->flags = 0;
			txch->ctrlr = hsi_ctrlr;
			txch->write_data.addr = NULL;
			txch->write_data.size = 0;
			txch->hsi_dma_info = &pdata->hsi_dma_info[ch];
			txch->n_bytes = 0x1;
			fill_hsi_dma_info(pdev,txch);
			spin_lock_init(&txch->hsi_ch_lock);
		}
	}
	else {
		/** HSI receive */
		for (ch=0;ch < n_ch; ch++) {
			rxch = &hsi_ctrlr->hsi_rx_channels[ch];
			rxch->channel_number = ch;
			rxch->span = (pdata->ch_base_span[ch].span)+1;
			rxch->watermark = pdata->watermark;
			rxch->flags = 0;
			rxch->ctrlr = hsi_ctrlr;
			rxch->read_data.addr = NULL;
			rxch->read_data.size = 0;
			rxch->hsi_dma_info = &pdata->hsi_dma_info[ch];
			rxch->n_bytes = 0x1;
			fill_hsi_dma_info(pdev,rxch);
			spin_lock_init(&rxch->hsi_ch_lock);
		}

	}
}

static void  __init hsi_initialise_controller(struct platform_device* pdev)
{
	struct hsi_plat_data *pdata = NULL;
	struct hsi_dev *hsi_ctrlr =NULL;

	pdata = (struct hsi_plat_data *) pdev->dev.platform_data;

	hsi_ctrlr=pdata->hsi_ctrlr;
	hsi_ctrlr->max_ch=pdata->channels;
	hsi_ctrlr->mode = pdata->mode;
	hsi_ctrlr->flags = 0;
	spin_lock_init(&hsi_ctrlr->lock);
}



static int __init hsi_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct hsi_dev *hsi_ctrlr = NULL;
	struct hsi_plat_data *pdata = NULL;
	unsigned char name[20];
	int err = 0;
	int i=0;

	/* FIXME : need to do tis in better way */
	*((unsigned int *)((void __iomem *)ioremap_nocache(0x8011f000, 32))) = 0x7fff;
	*((unsigned int *)((void __iomem *)ioremap_nocache(0x8011f008, 32))) = 0x7fff;

	if ((pdev == NULL) || (pdev->dev.platform_data == NULL)) {
		printk(KERN_INFO "No device/platform_data found on HSI device\n");
		return -ENODEV;
	}


	hsi_ctrlr = kzalloc(sizeof(struct hsi_dev),GFP_KERNEL);
	if (hsi_ctrlr == NULL) {
		printk(KERN_INFO "Could not allocate memory for struct hsi_dev\n");
		return -ENOMEM;
	}

	#if 1
	hsi_ctrlr->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(hsi_ctrlr->clk)) {
			printk(KERN_INFO " DBG_ST.hsi Could not get HSI clock\n");
			err = PTR_ERR(hsi_ctrlr->clk);
			goto rollback_alloc;
	}
	clk_enable(hsi_ctrlr->clk);
	#endif

	/** initialise the HSI controller and channels (tx and rx) */
	hsi_ctrlr->dev_type = pdev->id;
	pdata = pdev->dev.platform_data;
	pdata->hsi_ctrlr = hsi_ctrlr;

	err = stm_gpio_altfuncenable(pdata->gpio_alt_func);
	if (err) {
		printk(KERN_INFO "Could not set HSI GPIO alternate function correctly\n");
		err = -ENODEV;
		goto rollback_alloc;
	}

	hsi_initialise_controller(pdev);
	hsi_initialise_channels(pdev);

	res = platform_get_resource(pdev,IORESOURCE_MEM,0);
	if (!res) {
		printk(KERN_INFO "Could not get HSI IO memory information\n");
		err = -ENODEV;
		goto rollback_gpio;
	}

	hsi_ctrlr->regbase = (void __iomem *)ioremap(res->start,res->end - res->start + 1);
	if (!(hsi_ctrlr->regbase)) {
		printk(KERN_INFO "Unable to map register base \n");
		err = -EBUSY;
		goto rollback_gpio;
	}

	res = platform_get_resource(pdev,IORESOURCE_IRQ,0);
	if (!res) {
		printk(KERN_INFO "Unable to map HSI D0/D1 IRQ base \n");
		err = -EBUSY;
		goto rollback_map;
	}

	/** storing D0 and D1 interrupt lines */
	if (res->end - res->start) {
		hsi_ctrlr->irq1 = res->start;
		hsi_ctrlr->irq2 = res->end;
	}

	/** store the HSIR exception interrupt and channel overrun interrupt line */
	if (hsi_ctrlr->dev_type == 0x1) /** rx */ {
		res = platform_get_resource(pdev,IORESOURCE_IRQ,1);
		if (!res) {
			printk(KERN_INFO "Unable to map HSIR EXCEP IRQ base \n");
			err = -EBUSY;
			goto rollback_map;
		}
		hsi_ctrlr->irqexcep = res->start;

		for(i=0;i<hsi_ctrlr->max_ch;i++) {
			res = platform_get_resource(pdev,IORESOURCE_IRQ,i+2);
			if (!res) {
				printk(KERN_INFO "Unable to map HSIR CHANNEL %d OVERRUN IRQ LINE \n",i);
				err = -EBUSY;
				goto rollback_map;
			}
			hsi_ctrlr->irq_choverrun[i] = res->start;
		}
	}

	/** install handlers and tasklets */
	if (hsi_initialise_irq(hsi_ctrlr)) {
		printk(KERN_INFO "HSI error in interrupt registration \n");
		goto rollback_map;
	}

	/** write to hardware but do not start yet*/
	hsi_initialise_hw(pdev);

	/** register devices on hsi bus */
	err = register_hsi_devices(pdev);
	if (err < 0) {
		printk(KERN_INFO "HSI error in device registration \n");
		goto rollback_irq;
	}

	/** kick start the controller */
	start_hsi_controller(pdev);

	snprintf(name,sizeof(name),"%s:%d",pdev->name,pdev->id);
	if (!err) printk(KERN_INFO "%s probe COMPLETED \n",name);

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
	pdata->hsi_ctrlr = NULL;
	kfree(hsi_ctrlr);
	return err;
}

static int __exit hsi_remove(struct platform_device *pdev)
{
	struct hsi_plat_data *pdata = NULL;
	struct hsi_dev *hsi_ctrlr =NULL;

	pdata = (struct hsi_plat_data *) pdev->dev.platform_data;
	hsi_ctrlr=pdata->hsi_ctrlr;

	hsi_close_all_ch(hsi_ctrlr);
	free_hsi_irq(hsi_ctrlr);
	iounmap(hsi_ctrlr->regbase);
	clk_put(hsi_ctrlr->clk);
	pdata->hsi_ctrlr = NULL;
	kfree(hsi_ctrlr);

	return 0;
}

static struct platform_driver hsi_driver = {
	.probe = hsi_probe,
	.remove = __exit_p(hsi_remove),
	.driver = {
		.name = "stm-hsi",
		.owner = THIS_MODULE,
	}
};

static int __init hsi_driver_init(void)
{
	int err = 0;

	err = hsi_bus_init();
	if (err) {
		printk(KERN_INFO "HSI bus initialisation FAILED: %d\n", err);
		return err;
	}

	err = platform_driver_probe(&hsi_driver, hsi_probe);
	if (err < 0) {
		printk(KERN_INFO "HSI Platform DRIVER register FAILED: %d\n"
				, err);
		hsi_bus_exit();
		return err;
	}

	return 0;
}

static void __exit hsi_driver_exit(void)
{
	hsi_bus_exit();
	platform_driver_unregister(&hsi_driver);

	printk(KERN_INFO "HSI Platform DRIVER removed\n");
}

module_init(hsi_driver_init);
module_exit(hsi_driver_exit);

MODULE_AUTHOR(HSI_DRIVER_AUTHOR);
MODULE_DESCRIPTION(HSI_DRIVER_DESC);
MODULE_LICENSE("GPL");
