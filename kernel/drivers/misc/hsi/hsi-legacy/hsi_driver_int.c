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

static void reset_ch_read(struct hsi_channel *ch)
{
	ch->read_data.addr = NULL;
	ch->read_data.size = 0;
}

static void reset_ch_write(struct hsi_channel *ch)
{
	ch->write_data.addr = NULL;
	ch->write_data.size = 0;
}

int hsi_write_interrupt_mode(struct hsi_channel *ch)
{
	struct hsi_dev *hsi_ctrlr;
	void __iomem* hsi_base;
	volatile unsigned int wmark_intrmask;

	hsi_ctrlr=ch->ctrlr;
	hsi_base= hsi_ctrlr->regbase;

        /** watermark fixed to count-1 entries */
        //writel((0x1f & ((ch->write_data.size)-1)),(hsi_base+ HSI_TX_WATERMARKX + (0x4*ch->channel_number)));
        /** enable the corresponding channel intr mask  */
	wmark_intrmask=(0xff) & readl(hsi_base+HSI_TX_WATERMARKIM);
	writel((wmark_intrmask | (1<<ch->channel_number)), hsi_base+HSI_TX_WATERMARKIM);

	return 0;
}

int hsi_read_interrupt_mode(struct hsi_channel *ch)
{
	struct hsi_dev *hsi_ctrlr;
	void __iomem* hsi_base;
	volatile unsigned int wmark_intrmask;

	hsi_ctrlr=ch->ctrlr;
	hsi_base= hsi_ctrlr->regbase;

        /** watermark fixed to count-1 entries - not required as watermark is fixed */
        // writel((0x1f & ((ch->read_data.size)-1)),(hsi_base+ HSI_RX_WATERMARKX + (0x4*ch->channel_number)));

	/** enable the corresponding channel intr mask  */
	wmark_intrmask=(0xff) & readl(hsi_base+HSI_RX_WATERMARKIM);
	writel((wmark_intrmask | (1<<ch->channel_number)), hsi_base+HSI_RX_WATERMARKIM);

	return 0;
}

void hsi_cancel_write_interrupt_mode(struct hsi_channel *ch)
{

	struct hsi_dev *hsi_ctrlr;
	void __iomem* hsi_base;
	volatile unsigned int wmark_intrmask;

	hsi_ctrlr=ch->ctrlr;
	hsi_base= hsi_ctrlr->regbase;
        /** reset watermark to high number,say 4 */
        //writel((0x1f & 0x3),(hsi_base+ HSI_TX_WATERMARKX + (0x4*ch->channel_number)));
        /** disable the corresponding channel intr mask  */
	wmark_intrmask=(0xff) & readl(hsi_base+HSI_TX_WATERMARKIM);
	writel((wmark_intrmask & ~(1<<ch->channel_number)), hsi_base+HSI_TX_WATERMARKIM);
	/** write the original watermark */
	writel((0x1f & (ch->watermark -1)),(hsi_base+HSI_TX_WATERMARKX + (0x4*ch->channel_number)));
	/** flush the transmit channel buffers */
	writel((readl(hsi_base+HSI_TX_BUFSTATE) &
				~((unsigned long)(1 << ch->channel_number))),hsi_base+HSI_TX_BUFSTATE);

	reset_ch_write(ch);

}

void hsi_cancel_read_interrupt_mode(struct hsi_channel *ch)
{
	struct hsi_dev *hsi_ctrlr;
	void __iomem* hsi_base;
	volatile unsigned int wmark_intrmask;

	hsi_ctrlr=ch->ctrlr;
	hsi_base= hsi_ctrlr->regbase;
        /** reset watermark to high number,say 4 */
        //writel((0x1f & 0x3),(hsi_base+ HSI_RX_WATERMARKX + (0x4*ch->channel_number)));

	/** disable the corresponding channel intr mask  */
	wmark_intrmask=(0xff) & readl(hsi_base+HSI_RX_WATERMARKIM);
	writel((wmark_intrmask & ~(1<<ch->channel_number)), hsi_base+HSI_RX_WATERMARKIM);
	/** write the original watermark */
	writel((0x1f & (ch->watermark -1)),(hsi_base+HSI_RX_WATERMARKX + (0x4*ch->channel_number)));
	/** flush the receive channel buffers */
	writel((readl(hsi_base+HSI_RX_BUFSTATE) &
				~((unsigned long)(1 << ch->channel_number))),hsi_base+HSI_RX_BUFSTATE);
	reset_ch_read(ch);
}

/**
 * hsi_u8_writer - Write FIFO data in Data register as a 8 Bit Data
 *
 * This function writes data in Tx FIFO till it is not full
 * which is indicated by the status register or our transfer is complete.
 * It also updates the temporary write ptr tx in drv_data which maintains
 * current write position in transfer buffer
 */
void hsi_u8_writer(struct hsi_channel *ch)
{
	struct hsi_dev *hsi_ctrlr = ch->ctrlr;
	void __iomem *hsi_base = hsi_ctrlr->regbase;
	int i=0;


	spin_lock(&ch->hsi_ch_lock);
	if (ch->write_data.addr) {
		//while ((i< (ch->num_xfer_perintr)) && (ch->write_data.size)) {
		while (!((readl(hsi_base+HSI_TX_BUFSTATE)) & (1<<ch->channel_number)) && (ch->write_data.size)) {
			writel(*(u8 *)(ch->write_data.addr), hsi_base+HSI_TX_BUFFERX+(4*ch->channel_number));
			ch->write_data.addr = ch->write_data.addr + ch->n_bytes;
			ch->write_data.size = ch->write_data.size - ch->n_bytes;
			i++;
		}
		if (ch->write_data.size == 0x0)
			reset_ch_write(ch);
	}
	spin_unlock(&ch->hsi_ch_lock);
}

/**
 * hsi_u8_reader - Read FIFO data in Data register as a 8 Bit Data
 *
 * This function reads data in Rx FIFO till it is not empty
 * which is indicated by the status register or our transfer is complete.
 * It also updates the temporary Read ptr rx in drv_data which maintains
 * current read position in transfer buffer
 */
void hsi_u8_reader(struct hsi_channel *ch)
{
	struct hsi_dev *hsi_ctrlr = ch->ctrlr;
	void __iomem *hsi_base = hsi_ctrlr->regbase;
	int i=0;


	spin_lock(&ch->hsi_ch_lock);
	if (ch->read_data.addr) {
		//while ((i< (ch->num_xfer_perintr)) && (ch->read_data.size)) {
		while (((readl(hsi_base+HSI_RX_BUFSTATE)) & (1<<ch->channel_number)) && (ch->read_data.size)) {
			*(u8 *)(ch->read_data.addr) = readl(hsi_base+HSI_RX_BUFFERX+(4*ch->channel_number));
			/** printk("HSI RX: data 0x%x arrived \n",*(u8 *)(ch->read_data.addr));*/
			ch->read_data.addr = ch->read_data.addr + ch->n_bytes;
			ch->read_data.size = ch->read_data.size - ch->n_bytes;
			i++;
		}
		if (ch->read_data.size == 0x0)
			reset_ch_read(ch);
	}
	spin_unlock(&ch->hsi_ch_lock);
}

/**
 * hsi_u16_writer - Write FIFO data in Data register as a 16 Bit Data
 *
 * This function writes data in Tx FIFO till it is not full
 * which is indicated by the status register or our transfer is complete.
 * It also updates the temporary write ptr tx in drv_data which maintains
 * current write position in transfer buffer
 */
void hsi_u16_writer(struct hsi_channel *ch)
{
	struct hsi_dev *hsi_ctrlr = ch->ctrlr;
	void __iomem *hsi_base = hsi_ctrlr->regbase;
	int i=0;


	spin_lock(&ch->hsi_ch_lock);
	if (ch->write_data.addr) {
		//while ((i< (ch->num_xfer_perintr)) && (ch->write_data.size)) {
		while (!((readl(hsi_base+HSI_TX_BUFSTATE)) & (1<<ch->channel_number)) && (ch->write_data.size)) {
			writel(*(u16 *)(ch->write_data.addr), hsi_base+HSI_TX_BUFFERX+(4*ch->channel_number));
			ch->write_data.addr = ch->write_data.addr + ch->n_bytes;
			ch->write_data.size = ch->write_data.size - ch->n_bytes;
			i++;
		}
		if (ch->write_data.size == 0x0)
			reset_ch_write(ch);
	}
	spin_unlock(&ch->hsi_ch_lock);
}


/**
 * hsi_u16_reader - Read FIFO data in Data register as a 16 Bit Data
 *
 * This function reads data in Rx FIFO till it is not empty
 * which is indicated by the status register or our transfer is complete.
 * It also updates the temporary Read ptr rx in drv_data which maintains
 * current read position in transfer buffer
 */
void hsi_u16_reader(struct hsi_channel *ch)
{
	struct hsi_dev *hsi_ctrlr = ch->ctrlr;
	void __iomem *hsi_base = hsi_ctrlr->regbase;
	int i=0;


	spin_lock(&ch->hsi_ch_lock);
	if (ch->read_data.addr) {
		//while ((i< (ch->num_xfer_perintr)) && (ch->read_data.size)) {
		while (((readl(hsi_base+HSI_RX_BUFSTATE)) & (1<<ch->channel_number)) && (ch->read_data.size)) {
			*(u16 *)(ch->read_data.addr) = readl(hsi_base+HSI_RX_BUFFERX+(4*ch->channel_number));
			/** printk("HSI RX: data 0x%x arrived \n",*(u16 *)(ch->read_data.addr)); */
			ch->read_data.addr = ch->read_data.addr + ch->n_bytes;
			ch->read_data.size = ch->read_data.size - ch->n_bytes;
			i++;
		}
		if (ch->read_data.size == 0x0)
			reset_ch_read(ch);
	}
	spin_unlock(&ch->hsi_ch_lock);
}

/**
 * hsi_u32_writer - Write FIFO data in Data register as a 32 Bit Data
 *
 * This function writes data in Tx FIFO till it is not full
 * which is indicated by the status register or our transfer is complete.
 * It also updates the temporary write ptr tx in drv_data which maintains
 * current write position in transfer buffer
 */
void hsi_u32_writer(struct hsi_channel *ch)
{
	struct hsi_dev *hsi_ctrlr = ch->ctrlr;
	void __iomem *hsi_base = hsi_ctrlr->regbase;
	int i=0;


	spin_lock(&ch->hsi_ch_lock);
	if (ch->write_data.addr) {
		while ((i< (ch->num_xfer_perintr)) && (ch->write_data.size)) {
		//while (!((readl(hsi_base+HSI_TX_BUFSTATE)) & (1<<ch->channel_number)) && (ch->write_data.size)) {
			writel(*(u32 *)(ch->write_data.addr), hsi_base+HSI_TX_BUFFERX+(4*ch->channel_number));
			ch->write_data.addr = ch->write_data.addr + ch->n_bytes;
			ch->write_data.size = ch->write_data.size - ch->n_bytes;
			i++;
		}
		if (ch->write_data.size == 0x0)
			reset_ch_write(ch);
	}
	spin_unlock(&ch->hsi_ch_lock);
}


/**
 * hsi_u32_reader - Read FIFO data in Data register as a 32 Bit Data
 *
 * This function reads data in Rx FIFO till it is not empty
 * which is indicated by the status register or our transfer is complete.
 * It also updates the temporary Read ptr rx in drv_data which maintains
 * current read position in transfer buffer
 */
void hsi_u32_reader(struct hsi_channel *ch)
{
	struct hsi_dev *hsi_ctrlr = ch->ctrlr;
	void __iomem *hsi_base = hsi_ctrlr->regbase;
	int i=0;


	spin_lock(&ch->hsi_ch_lock);
	if (ch->read_data.addr) {
		while ((i< (ch->num_xfer_perintr)) && (ch->read_data.size)) {
		//while (((readl(hsi_base+HSI_RX_BUFSTATE)) & (1<<ch->channel_number)) && (ch->read_data.size)) {
			*(u32 *)(ch->read_data.addr) = readl(hsi_base+HSI_RX_BUFFERX+(4*ch->channel_number));
			/** printk("HSI RX: data 0x%x arrived \n",*(u32 *)(ch->read_data.addr));*/
			ch->read_data.addr = ch->read_data.addr + ch->n_bytes;
			ch->read_data.size = ch->read_data.size - ch->n_bytes;
			i++;
		}
		if (ch->read_data.size == 0x0)
			reset_ch_read(ch);
	}
	spin_unlock(&ch->hsi_ch_lock);
}

void do_hsi_tx_tasklet(unsigned long ctrlr)
{
	struct hsi_dev *hsi_ctrlr=(struct hsi_dev *)ctrlr;
	void __iomem *hsi_base = hsi_ctrlr->regbase;
	volatile unsigned long wmark_intrstatus;
	volatile unsigned long wmark_intrmask;
	struct hsi_channel *chnl;
	unsigned char i;

	wmark_intrstatus = (0xFF) & readl(hsi_base + HSI_TX_WATERMARKIS);

	for (i=0;i<HSI_MAX_CHANNELS;i++) {
		if ((wmark_intrstatus) & (1<<i)) {
			/** data needs to be transferred for this channel*/
			chnl = &hsi_ctrlr->hsi_tx_channels[i];
			//do_channel_tx(chnl);
			(*chnl->ops.write)(chnl);
			/** FIXME: HSI bug - clearing interrupt disables delivery of further interrupts */
			//writel((1<<i), hsi_base+HSI_TX_WATERMARKIC);
			spin_lock(&chnl->hsi_ch_lock);
			if ((chnl->write_data.size) <= (chnl->watermark * chnl->n_bytes))
				/** TODO fix watermark on basis of bytes left */
				writel((0x1f & 0x0),(hsi_base+HSI_TX_WATERMARKX + (0x4*i)));
			/** we have transferred the required number of bytes user requested, callback*/
			if (chnl->write_data.size == 0x0) {
				/** disable the corresponding watermark as we are done */
				wmark_intrmask=(0xff) & readl(hsi_base+HSI_TX_WATERMARKIM);
				writel((wmark_intrmask & ~(1<<chnl->channel_number)), hsi_base+HSI_TX_WATERMARKIM);
				/** write the original watermark */
				writel((0x1f & (chnl->watermark -1)),(hsi_base+HSI_TX_WATERMARKX + (0x4*i)));
				spin_unlock(&chnl->hsi_ch_lock);
				/** callback */
				(*chnl->ops.write_done)(chnl->dev);
			}
			else
				spin_unlock(&chnl->hsi_ch_lock);
		}
	}
	enable_irq(hsi_ctrlr->irq1);
}

void do_hsi_rx_tasklet(unsigned long ctrlr)
{
	struct hsi_dev *hsi_ctrlr=(struct hsi_dev *)ctrlr;
	void __iomem *hsi_base = hsi_ctrlr->regbase;
	volatile unsigned long wmark_intrstatus;
	volatile unsigned long wmark_intrmask;
	struct hsi_channel *chnl;
	unsigned char i;
	volatile unsigned long gauge;

	wmark_intrstatus = (0xFF) & readl(hsi_base + HSI_RX_WATERMARKIS);

	for (i=0;i<HSI_MAX_CHANNELS;i++) {
		if ((wmark_intrstatus) & (1<<i)) {
			/** data has arrived on this channel */
			/** printk("Data arrived on channel %d\n",i);*/
			chnl = &hsi_ctrlr->hsi_rx_channels[i];
			/** gaurd against spurious interrupts */
			gauge = (0x1F) & readl(hsi_base + HSI_RX_GAUGEX+ (0x4 * chnl->channel_number));
			//do_channel_rx(chnl);
			if (!gauge) {
				/** clear and go away */
				writel((1<<i), hsi_base+HSI_RX_WATERMARKIC);
				continue;
			}
			(*chnl->ops.read)(chnl);
			spin_lock(&chnl->hsi_ch_lock);
			writel((1<<i), hsi_base+HSI_RX_WATERMARKIC);
			if ((chnl->read_data.size) <= (chnl->watermark * chnl->n_bytes))
				/** TODO fix watermark on basis of bytes left */
				writel((0x1f & 0x0),(hsi_base+HSI_RX_WATERMARKX + (0x4*i)));
			/** we have the required number of bytes user requested, callback*/
			if (chnl->read_data.size == 0x0) {
				/** disable the corresponding watermark as we are done */
				wmark_intrmask=(0xff) & readl(hsi_base+HSI_RX_WATERMARKIM);
				writel((wmark_intrmask & ~(1<<chnl->channel_number)), hsi_base+HSI_RX_WATERMARKIM);
				/** write the original watermark */
				writel((0x1f & (chnl->watermark -1)),(hsi_base+HSI_RX_WATERMARKX + (0x4*i)));
				spin_unlock(&chnl->hsi_ch_lock);
				(*chnl->ops.read_done)(chnl->dev);
			}
			else
				spin_unlock(&chnl->hsi_ch_lock);
		}
	}
	enable_irq(hsi_ctrlr->irq1);
}

void do_hsi_rxexcep_tasklet(unsigned long ctrlr)
{
	struct hsi_dev *hsi_ctrlr=(struct hsi_dev *)ctrlr;
	void __iomem *hsi_base=NULL;
	u32 exmis=0x0;
	u32 ovrmis=0x0;
	u8 i;
	u8 chid=0x0;

	hsi_base=hsi_ctrlr->regbase;
	exmis = (0xF) & readl(hsi_base+HSI_RX_EXCEPMIS);
	ovrmis = (0xFF) & readl(hsi_base+HSI_RX_OVERRUNMIS);


	if (exmis) {
		for (i=0;i<4;i++) {
			if ((1<<i) & exmis) {
				switch (i) {
					case 0x0:
						printk(KERN_INFO "HSIR:TIMEOUT !! \n");
						hsi_excep_handler(hsi_ctrlr,HSI_EXCEP_TIMEOUT,NULL);
						break;
					case 0x1:
						printk(KERN_INFO "HSIR:PIPEBUF OVERRUN !! \n");
						/** flush pipeline buffers and ack interrupt */
						writel(0x1,hsi_base+HSI_RX_PIPEGAUGE);
						hsi_excep_handler(hsi_ctrlr,HSI_EXCEP_PIPEBUF_OVERRUN,NULL);
						break;
					case 0x2:
						printk(KERN_INFO "HSIR:BREAK_DETECTED !! \n");
						hsi_excep_handler(hsi_ctrlr,HSI_EXCEP_BREAK_DETECTED,NULL);
						break;
					case 0x3:
						printk(KERN_INFO "HSIR:PARITY ERROR !! \n");
						hsi_excep_handler(hsi_ctrlr,HSI_EXCEP_PARITY_ERROR,NULL);
				}
				writel((1<<i),hsi_base+HSI_RX_ACK);
			}
		}
	}

	if (ovrmis) {
		while (ovrmis) {
			if (0x1 & ovrmis) {
				/** channel overrun ..flush the channel buffer */
				writel((readl(hsi_base+HSI_RX_BUFSTATE) &
				                        ~((unsigned long)(1 << chid))),hsi_base+HSI_RX_BUFSTATE);
				/** call the exception handler */
				hsi_excep_handler(hsi_ctrlr,HSI_RXCHANNELS_OVERRUN,&chid);
				/** set the ack bit  */
				writel((1<<chid),hsi_base+HSI_RX_OVERRUNACK);
			}
			ovrmis=ovrmis >>1;
			chid++;
		}
	}

	enable_irq(hsi_ctrlr->irqexcep);
}

/** interrupt context : tx handler */
irqreturn_t hsi_tx_irq_handler(int irq,void *ctrlr)
{
	struct hsi_dev *hsi_ctrlr=ctrlr;
	tasklet_hi_schedule(&hsi_ctrlr->hsi_tx_tasklet);
	disable_irq_nosync(hsi_ctrlr->irq1);

	return IRQ_HANDLED;
}

/** interrupt context : rx handler */
irqreturn_t hsi_rx_irq_handler(int irq,void *ctrlr)
{
	struct hsi_dev *hsi_ctrlr=ctrlr;

	tasklet_hi_schedule(&hsi_ctrlr->hsi_rx_tasklet);
	disable_irq_nosync(hsi_ctrlr->irq1);

	return IRQ_HANDLED;
}

/** interrupt context : rx exception handler */
irqreturn_t hsi_rxexcep_irq_handler(int irq,void *ctrlr)
{
	int i=0;
	struct hsi_dev *hsi_ctrlr=ctrlr;
	tasklet_hi_schedule(&hsi_ctrlr->hsi_rxexcep_tasklet);
	disable_irq_nosync(hsi_ctrlr->irqexcep);
	for (i=0;i<hsi_ctrlr->max_ch;i++)
		disable_irq_nosync(hsi_ctrlr->irq_choverrun[i]);
	return IRQ_HANDLED;
}


