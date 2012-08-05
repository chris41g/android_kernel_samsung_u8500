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
#include "hsi_driver.h"
#include <linux/dmaengine.h>
#include <linux/pfn.h>
#include <plat/ste_dma40.h>

void hsi_dma_write_eot_handler(void * data)
{
	struct hsi_channel *ch = (struct hsi_channel *)data;
	struct hsi_dev* hsi_ctrlr;
	volatile unsigned int dma_mask;
	void __iomem* hsi_base;

	/*printk("HSI TX dma callback issued ...\n");*/
	hsi_ctrlr=ch->ctrlr;
	hsi_base= hsi_ctrlr->regbase;

	dma_mask =(0xff) & readl(hsi_base+HSI_TX_DMAEN);
	writel((dma_mask & (~(1<<ch->channel_number))), hsi_base+HSI_TX_DMAEN);

	/** restore the original watermark */
	writel((0x1f & (ch->watermark -1)),(hsi_base+HSI_TX_WATERMARKX + (0x4*ch->channel_number)));

	ch->write_data.addr = NULL;
	ch->write_data.size = 0;

	/** call the application callback */
	tasklet_schedule(&ch->hsi_dma_tasklet);
}

void hsi_dma_read_eot_handler(void * data)
{
	struct hsi_channel *ch = (struct hsi_channel *)data;
	struct hsi_dev* hsi_ctrlr;
	volatile unsigned int dma_mask;
	void __iomem* hsi_base;

	/*printk("HSI RX dma callback issued ...\n");*/
	hsi_ctrlr=ch->ctrlr;
	hsi_base= hsi_ctrlr->regbase;

	dma_mask =(0xff) & readl(hsi_base+HSI_RX_DMAEN);
	writel((dma_mask & (~(1<<ch->channel_number))), hsi_base+HSI_RX_DMAEN);

	/** restore the original watermark */
	writel((0x1f & (ch->watermark -1)),(hsi_base+HSI_RX_WATERMARKX + (0x4*ch->channel_number)));

	ch->read_data.addr = NULL;
	ch->read_data.size = 0;

	/** call the application callback */
	tasklet_schedule(&ch->hsi_dma_tasklet);
}

int hsi_write_dma_mode(struct hsi_channel *ch)
{
	/*We Need to do Mem to Periph DMA*/
	struct hsi_dev *hsi_ctrlr;
	void __iomem* hsi_base;
	void * dst_addr = NULL;
	void * src_addr = NULL;
	int count = 0;
	volatile unsigned int dma_mask;
	void* hsi_base_phy = (void *)0x8011C000;
	struct dma_async_tx_descriptor *dmach_desc;
	struct scatterlist sg;

	hsi_ctrlr=ch->ctrlr;
	hsi_base= hsi_ctrlr->regbase;

	src_addr = ch->write_data.addr;
	dst_addr = hsi_base_phy+HSI_TX_BUFFERX+(4*ch->channel_number);
	count = ch->write_data.size;

	if (ch->watermark <= 0x3) {
		(void) stedma40_set_psize(ch->dma_chan, STEDMA40_PSIZE_LOG_1,
					  STEDMA40_PSIZE_LOG_1);

		writel((0x1f & 0x0),(hsi_base+HSI_TX_WATERMARKX + (0x4*(ch->channel_number))));
	}
	else {
		(void) stedma40_set_psize(ch->dma_chan, STEDMA40_PSIZE_LOG_4,
					  STEDMA40_PSIZE_LOG_4);

		writel((0x1f & 0x3),(hsi_base+HSI_TX_WATERMARKX + (0x4*(ch->channel_number))));
	}

	sg_init_table(&sg, 1);
	sg_set_page(&sg, pfn_to_page(PFN_DOWN((dma_addr_t) src_addr)), count,
		    offset_in_page(src_addr));
	sg_dma_address(&sg) = (dma_addr_t) src_addr;
	sg_dma_len(&sg) = count;

	dmach_desc = ch->dma_chan->device->
		device_prep_slave_sg(ch->dma_chan, &sg, 1, DMA_TO_DEVICE,
				     DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!dmach_desc)
		return -ENOMEM;

	dmach_desc->callback = hsi_dma_write_eot_handler;
	dmach_desc->callback_param = ch;
	dmach_desc->tx_submit(dmach_desc);
	dma_async_issue_pending(ch->dma_chan);

	/*Enable DMA bits of HSI controller*/
	dma_mask =(0xff) & readl(hsi_base+HSI_TX_DMAEN);
	writel((dma_mask | (1<<ch->channel_number)), hsi_base+HSI_TX_DMAEN);

	return 0;
}

int hsi_read_dma_mode(struct hsi_channel *ch)
{
	/*We Need to do Periph to Mem DMA*/
	struct hsi_dev *hsi_ctrlr;
	void __iomem* hsi_base;
	void * dst_addr = NULL;
	void * src_addr = NULL;
	int count = 0;
	volatile unsigned int dma_mask;
	void *hsi_base_phy=(void *)0x8011B000;
	struct dma_async_tx_descriptor *dmach_desc;
	struct scatterlist sg;

	hsi_ctrlr=ch->ctrlr;
	hsi_base= hsi_ctrlr->regbase;

	src_addr = hsi_base_phy+HSI_RX_BUFFERX+(4*ch->channel_number);
	dst_addr = ch->read_data.addr;
	count = ch->read_data.size;

	/** sync up the watermark and burst size */
	if (ch->watermark <= 0x3) {
		(void) stedma40_set_psize(ch->dma_chan, STEDMA40_PSIZE_LOG_1,
					  STEDMA40_PSIZE_LOG_1);

		writel((0x1f & 0x0),(hsi_base+HSI_RX_WATERMARKX + (0x4*(ch->channel_number))));
	}
	else {
		(void) stedma40_set_psize(ch->dma_chan, STEDMA40_PSIZE_LOG_4,
					  STEDMA40_PSIZE_LOG_4);

		writel((0x1f & 0x3),(hsi_base+HSI_RX_WATERMARKX + (0x4*(ch->channel_number))));
	}

	sg_init_table(&sg, 1);
	sg_set_page(&sg, pfn_to_page(PFN_DOWN((dma_addr_t) dst_addr)), count,
		    offset_in_page(dst_addr));
	sg_dma_address(&sg) = (dma_addr_t) dst_addr;
	sg_dma_len(&sg) = count;

	dmach_desc = ch->dma_chan->device->
		device_prep_slave_sg(ch->dma_chan, &sg, 1, DMA_FROM_DEVICE,
				     DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!dmach_desc)
		return -ENOMEM;

	dmach_desc->callback = hsi_dma_read_eot_handler;
	dmach_desc->callback_param = ch;
	dmach_desc->tx_submit(dmach_desc);
	dma_async_issue_pending(ch->dma_chan);

	/*Enable DMA bits of HSI controller*/
	dma_mask =(0xff) & readl(hsi_base+HSI_RX_DMAEN);
	writel((dma_mask | (1<<ch->channel_number)), hsi_base+HSI_RX_DMAEN);

	return 0;
}

void hsi_cancel_write_dma_mode(struct hsi_channel *ch)
{
	struct hsi_dev *hsi_ctrlr;
	void __iomem* hsi_base;
	volatile unsigned int dma_mask;

	hsi_ctrlr=ch->ctrlr;
	hsi_base= hsi_ctrlr->regbase;

	ch->dma_chan->device->device_control(ch->dma_chan,
					     DMA_TERMINATE_ALL, 0);

	/*Disable DMA nebaling bits of HSI controller*/
	dma_mask =(0xff) & readl(hsi_base+HSI_TX_DMAEN);
	writel((dma_mask & (~(1<<ch->channel_number))), hsi_base+HSI_TX_DMAEN);
	/** flush the transmit channel buffers */
	writel((readl(hsi_base+HSI_TX_BUFSTATE) &
				~((unsigned long)(1 << ch->channel_number))),hsi_base+HSI_TX_BUFSTATE);
	/** restore the original watermark */
	writel((0x1f & (ch->watermark -1)),(hsi_base+HSI_TX_WATERMARKX + (0x4*ch->channel_number)));
	ch->write_data.addr = NULL;
	ch->write_data.size = 0;
}

void hsi_cancel_read_dma_mode(struct hsi_channel *ch)
{
	struct hsi_dev *hsi_ctrlr;
	void __iomem* hsi_base;
	volatile unsigned int dma_mask;

	hsi_ctrlr=ch->ctrlr;
	hsi_base= hsi_ctrlr->regbase;

	ch->dma_chan->device->device_control(ch->dma_chan,
					     DMA_TERMINATE_ALL, 0);

	/*Disable DMA nebaling bits of HSI controller*/
	dma_mask =(0xff) & readl(hsi_base+HSI_RX_DMAEN);
	writel((dma_mask & (~(1<<ch->channel_number))), hsi_base+HSI_RX_DMAEN);
	/** flush the receive channel buffers */
	writel((readl(hsi_base+HSI_RX_BUFSTATE) &
				~((unsigned long)(1 << ch->channel_number))),hsi_base+HSI_RX_BUFSTATE);
	/** restore the original watermark */
	writel((0x1f & (ch->watermark -1)),(hsi_base+HSI_RX_WATERMARKX + (0x4*ch->channel_number)));

	ch->read_data.addr=NULL;
	ch->read_data.size=0;
}

void do_hsi_tx_dma_tasklet(unsigned long param)
{
	struct hsi_channel *ch = (struct hsi_channel*)param;

	ch->ops.write_done(ch->dev);

}
EXPORT_SYMBOL(do_hsi_tx_dma_tasklet);

void do_hsi_rx_dma_tasklet(unsigned long param)
{
	struct hsi_channel *ch = (struct hsi_channel*)param;

	ch->ops.read_done(ch->dev);

}
EXPORT_SYMBOL(do_hsi_rx_dma_tasklet);

