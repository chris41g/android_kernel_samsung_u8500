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
#include "linux/delay.h"

//#define HSI_POLLING

/**
 * hsi_open - open a hsi device channel.
 * @dev: Reference to the hsi device channel to be openned.
 *
 * Returns 0 on success, -EINVAL on bad parameters, -EBUSY if is already opened.
 */
int hsi_open(struct hsi_device *dev)
{
	struct hsi_channel *ch;
	struct hsi_dev *hsi_ctrlr;
	void __iomem *hsi_base;

	if (!dev || !dev->ch) {
		printk(KERN_INFO "Wrong HSI device %p\n", dev);
		return -EINVAL;
	}

	ch = dev->ch;
	if (!ch->ops.read_done && !ch->ops.write_done) {
		printk(KERN_INFO "Trying to open with no callbacks registered\n");
		return -EINVAL;
	}
	hsi_ctrlr = ch->ctrlr;
	spin_lock_bh(&ch->hsi_ch_lock);
	if (ch->flags & HSI_CH_OPEN) {
		printk(KERN_INFO " Controller %d Channel %d already OPENED\n",
							dev->ctrlrid, dev->chid);
		spin_unlock_bh(&ch->hsi_ch_lock);
		return -EBUSY;
	}
	clk_enable(hsi_ctrlr->clk);
	ch->flags |= HSI_CH_OPEN;
	hsi_base=hsi_ctrlr->regbase;
	if (hsi_ctrlr->dev_type == 0x0) /** HSI transmit controller*/
		/** flush the transmit buffer */
		writel((readl(hsi_base+HSI_TX_BUFSTATE) &
			~((unsigned long)(1 << ch->channel_number))),hsi_base+HSI_TX_BUFSTATE);
	else
		writel((readl(hsi_base+HSI_RX_BUFSTATE) &
			~((unsigned long)(1 << ch->channel_number))),hsi_base+HSI_RX_BUFSTATE);

	spin_unlock_bh(&ch->hsi_ch_lock);

	return 0;
}
EXPORT_SYMBOL(hsi_open);

/**
 * hsi_write - write data into the hsi device channel
 * @dev: reference to the hsi device channel  to write into.
 * @data: pointer to a 32-bit word data to be written.
 * @datawidth: in bits (16bit or 32bit or ....)
 * @count: total number of bytes to be written.
 *
 * Return 0 on sucess, a negative value on failure.
 * A success values only indicates that the request has been accepted.
 * Transfer is only completed when the write_done callback is called.
 *
 */
int hsi_write(struct hsi_device *dev, void *data, u32 datawidth, unsigned int count)
{
	struct hsi_channel *ch;
	int err;

	if (dev->ctrlrid != 0x0) {
		printk(KERN_INFO "HSI Controller not enabled for write\n");
		return -EINVAL;
	}

	if (unlikely(!dev || !dev->ch || !data || (count <= 0))) {
		printk(KERN_INFO  "Wrong paramenters "
			"hsi_device %p data %p count %d", dev, data, count);
		return -EINVAL;
	}

	if (unlikely(!(dev->ch->flags & HSI_CH_OPEN))) {
		printk(KERN_INFO "HSI device NOT open\n");
		return -EINVAL;
	}

	ch = dev->ch;
	spin_lock_bh(&ch->hsi_ch_lock);
	ch->write_data.addr = data;
	ch->write_data.size = count;

	if (datawidth <=8) {
		/*printk("HSI: <= 8 bits per word \n");*/
		ch->n_bytes = 1;
		ch->ops.write = hsi_u8_writer;
	} else if (datawidth <= 16) {
		/*printk("HSI: <= 16 bits per word \n");*/
		ch->n_bytes = 2;
		ch->ops.write = hsi_u16_writer;
	} else {
		/*printk("HSI: <= 32 bits per word \n");*/
		ch->n_bytes = 4;
		ch->ops.write = hsi_u32_writer;
	}

	ch->num_xfer_perintr = ch->watermark ;

	if(dev->curr_mode == HSI_INTERRUPT_MODE)
		err = hsi_write_interrupt_mode(ch);
	else if(dev->curr_mode == HSI_DMA_MODE) {
		err = hsi_write_dma_mode(ch);
		}
	else {
		printk(KERN_INFO "HSI POLLING MODE NOT implemented \n");
		err = -EINVAL;
		/*FIXME: polling not yet there*/
	}

	if (unlikely(err < 0)) {
		ch->write_data.addr = NULL;
		ch->write_data.size = 0;
	}
	spin_unlock_bh(&ch->hsi_ch_lock);

	return err;

}
EXPORT_SYMBOL(hsi_write);

/**
 * hsi_read - read data from the hsi device channel
 * @dev: hsi device channel reference to read data from.
 * @data: pointer to a 32-bit word data to store the data.
 * @count: number of 32-bit word to be stored.
 *
 * Return 0 on sucess, a negative value on failure.
 * A success values only indicates that the request has been accepted.
 * Data is only available in the buffer when the read_done callback is called.
 *
 */
int hsi_read(struct hsi_device *dev, void* data, u32 datawidth, unsigned int count)
{
	struct hsi_channel *ch;
	int err=0;
	/** for POLLING MODE */
	unsigned int loop=0;
	struct hsi_dev *hsi_ctrlr=NULL ;
	volatile void __iomem *hsi_base=NULL ;

	if (dev->ctrlrid != 0x1) {
		printk(KERN_INFO "HSI Controller not enabled for read\n");
		return -EINVAL;
	}

	if (unlikely(!dev || !dev->ch || !data || (count <= 0))) {
		printk(KERN_INFO "Wrong paramenters "
				"hsi_device %p data %p count %d", dev, data, count);
		return -EINVAL;
	}
	if (unlikely(!(dev->ch->flags & HSI_CH_OPEN))) {
		printk(KERN_INFO "HSI device NOT open\n");
		return -EINVAL;
	}

	ch = dev->ch;
	spin_lock_bh(&ch->hsi_ch_lock);
	ch->read_data.addr = data;
	ch->read_data.size = count;

	if (datawidth <=8) {
		/*printk("HSI: <= 8 bits per word \n");*/
		ch->n_bytes = 1;
		ch->ops.read= hsi_u8_reader;
	} else if (datawidth <= 16) {
		/*printk("HSI: <= 16 bits per word \n");*/
		ch->n_bytes = 2;
		ch->ops.read= hsi_u16_reader;
	} else {
		/*printk("HSI: <= 32 bits per word \n");*/
		ch->n_bytes = 4;
		ch->ops.read= hsi_u32_reader;
	}

	ch->num_xfer_perintr = ch->watermark;

	if(dev->curr_mode == HSI_INTERRUPT_MODE)
		err = hsi_read_interrupt_mode(ch);
	else if(dev->curr_mode == HSI_DMA_MODE) {
		err = hsi_read_dma_mode(ch);
	}
	else {
		/** POLLING mode start*/
		hsi_ctrlr=ch->ctrlr;
		hsi_base = hsi_ctrlr->regbase;
		printk(KERN_INFO "hsi_read : channel %d regbase 0x%p\n"
				, ch->channel_number, hsi_base);
		while  (!((readl(hsi_base+HSI_RX_BUFSTATE)) & (1<<ch->channel_number))
							&& (loop < 50000)) {
			loop++;
			mdelay(1);
		}

		if (loop >=50000)
			printk(KERN_INFO "HSI_READ: NO DATA VIA POLLING\n");
		else
			printk(KERN_INFO "HSI_READ: RECEIVED DATA VIA POLLING\n");
		/** POLLING mode end*/

	}

	if (unlikely(err < 0)) {
		ch->read_data.addr = NULL;
		ch->read_data.size = 0;
	}
	spin_unlock_bh(&ch->hsi_ch_lock);

	return err;
}
EXPORT_SYMBOL(hsi_read);

void __hsi_write_cancel(struct hsi_channel *ch, int mode)
{
	if(mode == HSI_INTERRUPT_MODE)
		hsi_cancel_write_interrupt_mode(ch);
	else if(mode == HSI_DMA_MODE)
		hsi_cancel_write_dma_mode(ch);
	else
		printk(KERN_INFO "HSI read cancel Incorrect Mode \n");
	/*FIXME: No Polling Mode Yet*/
}

/**
 * hsi_write_cancel - Cancel pending write request.
 * @dev: hsi device channel where to cancel the pending write.
 *
 * write_done() callback will not be called after sucess of this function.
 * This call closes the channel also.
 */
void hsi_write_cancel(struct hsi_device *dev)
{
	struct hsi_channel *ch;

	if (dev->ctrlrid != 0x0) {
		printk(KERN_INFO "HSI Controller not enabled for write\n");
		return;
	}

	if (unlikely(!dev || !dev->ch)) {
		printk(KERN_INFO "Wrong HSI device %p\n", dev);
		return;
	}
	if (unlikely(!(dev->ch->flags & HSI_CH_OPEN))) {
		printk(KERN_INFO "HSI device NOT open\n");
		return;
	}

	ch=dev->ch;
	spin_lock_bh(&ch->hsi_ch_lock);
	__hsi_write_cancel(ch, dev->curr_mode);
	dev->ch->flags &= ~HSI_CH_OPEN;
	spin_unlock_bh(&ch->hsi_ch_lock);
}
EXPORT_SYMBOL(hsi_write_cancel);

void __hsi_read_cancel(struct hsi_channel *ch, int mode)
{
	if(mode == HSI_INTERRUPT_MODE)
		hsi_cancel_read_interrupt_mode(ch);
	else if(mode == HSI_DMA_MODE)
		hsi_cancel_read_dma_mode(ch);
	else
		printk(KERN_INFO "HSI read cancel Incorrect Mode \n");
	/*FIXME: No Polling Mode Yet*/
}


/**
 * hsi_read_cancel - Cancel pending read request.
 * @dev: hsi device channel where to cancel the pending read.
 *
 * read_done() callback will not be called after sucess of this function.
 */
void hsi_read_cancel(struct hsi_device *dev)
{
	struct hsi_channel *ch;

	if (dev->ctrlrid != 0x1) {
		printk(KERN_INFO "HSI Controller not enabled for read\n");
		return;
	}

	if (unlikely(!dev || !dev->ch)) {
		printk(KERN_INFO "Wrong HSI device %p\n", dev);
		return;
	}

	if (unlikely(!(dev->ch->flags & HSI_CH_OPEN))) {
		printk(KERN_INFO "HSI device NOT open\n");
		return;
	}

	ch=dev->ch;
	spin_lock_bh(&ch->hsi_ch_lock);
	__hsi_read_cancel(dev->ch, dev->curr_mode);
	dev->ch->flags &= ~HSI_CH_OPEN;
	spin_unlock_bh(&ch->hsi_ch_lock);
}
EXPORT_SYMBOL(hsi_read_cancel);


/**
 * hsi_close - close given hsi device channel
 * @dev: reference to hsi device channel.
 */
void hsi_close(struct hsi_device *dev)
{
	struct hsi_channel *ch;
	void __iomem *hsi_base;
	struct hsi_dev *hsi_ctrlr;

	if (!dev || !dev->ch) {
		printk(KERN_INFO "Trying to close wrong HSI device %p\n", dev);
		return;
	}

	ch=dev->ch;
	hsi_ctrlr=ch->ctrlr;
	hsi_base=hsi_ctrlr->regbase;

	spin_lock_bh(&ch->hsi_ch_lock);

	if (ch->flags & HSI_CH_OPEN) {
		dev->ch->flags &= ~HSI_CH_OPEN;
		if (hsi_ctrlr->dev_type == 0x0) /** HSIT */ {
			/** flush the transmit channel buffers */
			writel((readl(hsi_base+HSI_TX_BUFSTATE) &
				~((unsigned long)(1 << ch->channel_number))),hsi_base+HSI_TX_BUFSTATE);
		} else {
			/** flush the receive channel buffers */
			writel((readl(hsi_base+HSI_RX_BUFSTATE) &
				~((unsigned long)(1 << ch->channel_number))),hsi_base+HSI_RX_BUFSTATE);
		}
	  clk_disable(hsi_ctrlr->clk);
	}
	else
		printk(KERN_INFO "Trying to close an unopened HSI device : ctrlr %d chnl %d \n"
				, dev->ctrlrid, dev->chid);

	spin_unlock_bh(&ch->hsi_ch_lock);
}
EXPORT_SYMBOL(hsi_close);

/**
 * hsi_dev_set_cb - register read_done() and write_done() callbacks.
 * @dev: reference to hsi device channel where callbacks are associated.
 * @r_cb: callback to signal read transfer completed.
 * @w_cb: callback to signal write transfer completed.
 */
void hsi_dev_set_cb(struct hsi_device *dev, void (*r_cb)(struct hsi_device *dev)
					, void (*w_cb)(struct hsi_device *dev))
{
	struct hsi_channel *ch;

	if (unlikely(!dev || !dev->ch)) {
		printk(KERN_INFO "Wrong HSI device %p\n", dev);
		return;
	}

	if (unlikely(!r_cb && !w_cb)) {
		printk(KERN_INFO "HSI no valid callbacks supplied\n");
		return;
	}

	ch=dev->ch;
	spin_lock_bh(&ch->hsi_ch_lock);
	dev->ch->ops.read_done = r_cb;
	dev->ch->ops.write_done = w_cb;
	spin_unlock_bh(&ch->hsi_ch_lock);
}
EXPORT_SYMBOL(hsi_dev_set_cb);

/**
 * hsi_ioctl - HSI I/O control
 * @dev: hsi device channel reference to apply the I/O control (or port associated to it)
 * @command: HSI I/O control command
 * @arg: parameter associated to the control command. NULL, if no parameter.
 *
 * Return 0 on sucess, a negative value on failure.
 *
 */
int hsi_ioctl(struct hsi_device *dev, unsigned int command, void *arg)
{
	struct hsi_channel *ch;
	struct hsi_dev *hsi_ctrlr;
	void __iomem *hsi_base;
	int err = 0;

	if (!dev || !dev->ch) {
		printk(KERN_INFO "Wrong HSI device %p\n", dev);
		return -EINVAL;
	}

	ch = dev->ch;
	hsi_ctrlr = ch->ctrlr;

	if (unlikely(!(dev->ch->flags & HSI_CH_OPEN))) {
		printk(KERN_INFO "HSI device NOT open\n");
		return -EINVAL;
	}

	spin_lock_bh(&ch->hsi_ch_lock);

	hsi_base=hsi_ctrlr->regbase;

	switch (command) {
	case HSI_IOCTL_SET_WATERMARK:
		/** TODO: check span definition. Manual says (span-1) is actual span value
		 * and the watermark should be less than span (span -1)
		 */
		if (*(u32*)arg > (ch->span -2)) {
			printk(KERN_INFO "HSI IOCTL: illegal watermark value > span\n");
			spin_unlock_bh(&ch->hsi_ch_lock);
			return -EINVAL;
		}
		ch->watermark = *(u32*)arg;
		/*printk("HSI IOCTL: setting watermark value \n");*/
		if (hsi_ctrlr->dev_type == 0x0) /** HSI transmit controller*/
			writel((0x1f & ((*(u32*)arg)-1)),(hsi_base+
						HSI_TX_WATERMARKX + (0x4*ch->channel_number)));
		else
			writel((0x1f & ((*(u32*)arg)-1)),(hsi_base+
						HSI_RX_WATERMARKX + (0x4*ch->channel_number)));
		break;
	case HSI_IOCTL_GET_WATERMARK:
		printk(KERN_INFO "HSI IOCTL: getting watermark value \n");
		if (hsi_ctrlr->dev_type == 0x0) /** HSI transmit controller*/
			/** add 1 to the value read from register for actual watermark */
			*((u32*)arg) = (0x1F & (readl(hsi_base+ HSI_TX_WATERMARKX + (0x4*ch->channel_number))))+1;
		else
			*((u32*)arg) = (0x1F & (readl(hsi_base+ HSI_RX_WATERMARKX + (0x4*ch->channel_number))))+1;
		break;
	case HSI_IOCTL_GET_CURRMODE:
		printk(KERN_INFO "HSI IOCTL: Getting the current mode for transfer\n");
		*((u32*)arg) = dev->curr_mode;
		break;
	case HSI_IOCTL_SET_CURRMODE:
		/*printk("HSI IOCTL: Setting the current mode \
			for transfer\n");*/
		dev->curr_mode = *((u32*)arg);
		break;
	case HSI_IOCTL_SEND_BREAK:
		writel(0x1,hsi_base+HSI_TX_BREAK);
		break;
	case HSI_IOCTL_SET_FRAMELEN:
		if ((*(u32*)arg) > HSI_MAX_FRAMELEN) {
			printk(KERN_INFO "HSI IOCTL: Illegal framelen setting \n");
			spin_unlock_bh(&ch->hsi_ch_lock);
			return -EINVAL;
		}
		/*printk("HSI IOCTL: Setting the framelen for transfer\n");*/
		if (hsi_ctrlr->dev_type == 0x0) /** HSI transmit controller*/
			writel((0x1f & ((*(u32*)arg)-1)),(hsi_base+
						HSI_TX_FRAMELENX + (0x4*ch->channel_number)));
		else
			writel((0x1f & ((*(u32*)arg)-1)),(hsi_base+
						HSI_RX_FRAMELENX + (0x4*ch->channel_number)));
		break;
	case HSI_IOCTL_GET_FRAMELEN:
		printk(KERN_INFO "HSI IOCTL: getting framelen value \n");
		if (hsi_ctrlr->dev_type == 0x0) /** HSI transmit controller*/
			/** add 1 to the value read from register for actual framelen */
			*((u32*)arg) = (0x1F & (readl(hsi_base+ HSI_TX_FRAMELENX + (0x4*ch->channel_number))))+1;
		else
			*((u32*)arg) = (0x1F & (readl(hsi_base+ HSI_RX_FRAMELENX + (0x4*ch->channel_number))))+1;
		break;

	case HSI_IOCTL_SET_THRESHOLD:
		printk(KERN_INFO "HSI IOCTL: Setting threshold for detecting BREAK transmission\n");
		writel((0x3f & (*(u32*)arg)),(hsi_base+HSI_RX_THRESHOLD));
		break;

	default:
		err = -ENOIOCTLCMD;
		break;
	}

	spin_unlock_bh(&ch->hsi_ch_lock);
	return err;
}
EXPORT_SYMBOL(hsi_ioctl);


