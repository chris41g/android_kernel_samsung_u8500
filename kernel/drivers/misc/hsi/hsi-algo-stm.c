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


#include <linux/dmaengine.h>
#include <linux/hsi.h>
#include <mach/hsi-stm.h>
#include <mach/debug.h>

/**
 * hsi_open - open a hsi device channel.
 * @dev: Reference to the hsi device channel to be openned.
 *
 * Returns 0 on success, -EINVAL on bad parameters, -EBUSY if is already opened.
 */
static int stm_hsi_open(struct hsi_device *dev)
{
	struct hsi_channel *ch;
	struct hsi_controller *hsi_ctrlr;
	void __iomem *hsi_base;

	if (!dev || !dev->ch) {
		stm_dbg(DBG_ST.hsi, "Wrong HSI device %p\n", dev);
		return -EINVAL;
	}

	ch = dev->ch;
	/*if (!ch->ops.read_done && !ch->ops.write_done) {
	   stm_dbg(DBG_ST.hsi,"Trying to open with no callbacks registered\n");
	   return -EINVAL;
	   } */
	hsi_ctrlr = dev->ctrlr;
	spin_lock_bh(&ch->hsi_ch_lock);
	if (ch->flags & HSI_CH_OPEN) {
		stm_dbg(DBG_ST.hsi,
			" Controller %d Channel %d already OPENED\n", dev->cid,
			dev->chid);
		spin_unlock_bh(&ch->hsi_ch_lock);
		return -EBUSY;
	}
	clk_enable(hsi_ctrlr->clk);
	ch->flags |= HSI_CH_OPEN;
	hsi_base = hsi_ctrlr->regbase;
	if (hsi_ctrlr->dev_type == 0x0) {
		/** HSI transmit controller
		 * flush the transmit buffer */
		writel((readl(hsi_base + HSI_TX_BUFSTATE) &
			~((unsigned long)(1 << ch->channel_number))),
		       hsi_base + HSI_TX_BUFSTATE);
	} else {
		writel((readl(hsi_base + HSI_RX_BUFSTATE) &
			~((unsigned long)(1 << ch->channel_number))),
		       hsi_base + HSI_RX_BUFSTATE);
	}
	spin_unlock_bh(&ch->hsi_ch_lock);

	return 0;
}

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
static int stm_hsi_write(struct hsi_device *dev, struct hsi_data *xfer)
{
	struct hsi_channel *ch;
	int err = -EINVAL;

	if (dev->cid != 0x0) {
		stm_dbg(DBG_ST.hsi, "HSI Controller not enabled for write\n");
		return -EINVAL;
	}

	if (unlikely(!dev || !dev->ch || !xfer->data || (xfer->count <= 0))) {
		stm_dbg(DBG_ST.hsi, "Wrong paramenters "
			"hsi_device %p data %p count %d", dev, xfer->data,
			xfer->count);
		return -EINVAL;
	}

	if (unlikely(!(dev->ch->flags & HSI_CH_OPEN))) {
		stm_dbg(DBG_ST.hsi, "HSI device NOT open\n");
		return -EINVAL;
	}

	ch = dev->ch;
	spin_lock_bh(&ch->hsi_ch_lock);
	ch->write_data = *xfer;

	if (ch->datawidth <= 8) {
		ch->n_bytes = 1;
		ch->num_xfer_perintr = ch->watermark * 4;
		ch->ops.write = hsi_u8_writer;
	} else if (ch->datawidth <= 16) {
		ch->n_bytes = 2;
		ch->num_xfer_perintr = ch->watermark * 2;
		ch->ops.write = hsi_u16_writer;
	} else {
		ch->n_bytes = 4;
		ch->num_xfer_perintr = ch->watermark * 1;
		ch->ops.write = hsi_u32_writer;
	}

	ch->num_xfer_perintr = ch->watermark;

/**
 *	###Old method of setting modes###
 *	if(dev->curr_mode == HSI_INTERRUPT_MODE)
 *
 *		err = hsi_write_interrupt_mode(ch);
 *	else if(dev->curr_mode == HSI_DMA_MODE)
 *		err = hsi_write_dma_mode(ch);
 *	else {
 *		stm_dbg(DBG_ST.hsi,"HSI POLLING MODE NOT implemented \n");
 *		err = -EINVAL;
 *	}
**/

	/**
	 * We need to optimize on DMA setup and teardown overhead
	 * when the number of bytes to be transferred is less than
	 * a minimum value (currently 4). We configure for interrupt
	 * mode in this case.
	 **/

	if (dev->curr_mode == HSI_INTERRUPT_MODE)
		err = hsi_write_interrupt_mode(ch);
	else if (dev->curr_mode == HSI_DMA_MODE) {
		if (xfer->count <= 4)
			err = hsi_write_interrupt_mode(ch);
		else
			err = hsi_write_dma_mode(ch);
	}

	if (unlikely(err < 0)) {
		ch->write_data.data = NULL;
		ch->write_data.count = 0;
	}
	spin_unlock_bh(&ch->hsi_ch_lock);

	return err;

}

/**
 * stm_hsi_read - read data from the hsi device channel
 * @dev: hsi device channel reference to read data from.
 * @data: pointer to a 32-bit word data to store the data.
 * @count: number of 32-bit word to be stored.
 *
 * Return 0 on sucess, a negative value on failure.
 * A success values only indicates that the request has been accepted.
 * Data is only available in the buffer when the read_done callback is called.
 *
 */
static int stm_hsi_read(struct hsi_device *dev, struct hsi_data *xfer)
{
	struct hsi_channel *ch;
	int err = 0;
	/** for POLLING MODE **
	unsigned int loop=0;
	struct hsi_controller *hsi_ctrlr=NULL ;
	volatile void __iomem *hsi_base=NULL ;
	**/

	if (dev->cid != 0x1) {
		stm_dbg(DBG_ST.hsi, "HSI Controller not enabled for read\n");
		return -EINVAL;
	}

	if (unlikely(!dev || !dev->ch || !xfer->data || (xfer->count <= 0))) {
		stm_dbg(DBG_ST.hsi, "Wrong paramenters "
			"hsi_device %p data %p count %d", dev, xfer->data,
			xfer->count);
		return -EINVAL;
	}
	if (unlikely(!(dev->ch->flags & HSI_CH_OPEN))) {
		stm_dbg(DBG_ST.hsi, "HSI device NOT open\n");
		return -EINVAL;
	}

	ch = dev->ch;
	spin_lock_bh(&ch->hsi_ch_lock);
	ch->read_data = *xfer;

	if (ch->datawidth <= 8) {
		ch->n_bytes = 1;
		ch->num_xfer_perintr = ch->watermark * 4;
		ch->ops.read = hsi_u8_reader;
	} else if (ch->datawidth <= 16) {
		ch->n_bytes = 2;
		ch->num_xfer_perintr = ch->watermark * 2;
		ch->ops.read = hsi_u16_reader;
	} else {
		ch->n_bytes = 4;
		ch->num_xfer_perintr = ch->watermark * 1;
		ch->ops.read = hsi_u32_reader;
	}

	/**
	 * We need to optimize on DMA setup and teardown overhead
	 * when the number of bytes to be transferred is less than
	 * a minimum value (currently 4). We configure for interrupt
	 * mode in this case.
	 **/

	if (dev->curr_mode == HSI_INTERRUPT_MODE)
		err = hsi_read_interrupt_mode(ch);
	else if (dev->curr_mode == HSI_DMA_MODE) {
		if (xfer->count <= 4)
			err = hsi_read_interrupt_mode(ch);
		else
			err = hsi_read_dma_mode(ch);
	}
/**
*
*	if(dev->curr_mode == HSI_INTERRUPT_MODE)
*		err = hsi_read_interrupt_mode(ch);
*	else if(dev->curr_mode == HSI_DMA_MODE)
*		err = hsi_read_dma_mode(ch);
*	else {
*		** POLLING mode start**
*		hsi_ctrlr=ch->ctrlr;
*		hsi_base = hsi_ctrlr->regbase;
*		stm_dbg(DBG_ST.hsi,"hsi_read : channel %d regbase 0x%p\n",
*						ch->channel_number,hsi_base);
*		while  (!((readl(hsi_base+HSI_RX_BUFSTATE)) &
*			(1<<ch->channel_number) && (loop < 50000)) {
*			loop++;
*			mdelay(1);
*		}
*
*		if (loop >=50000)
*			stm_dbg(DBG_ST.hsi,"HSI_READ:NO DATA VIA POLL\n");
*		else
*			stm_dbg(DBG_ST.hsi,"HSI_READ:RECEIVED DATA POLL\n");
*		** POLLING mode end**
*
*	}
**/
	if (unlikely(err < 0)) {
		ch->read_data.data = NULL;
		ch->read_data.count = 0;
	}
	spin_unlock_bh(&ch->hsi_ch_lock);

	return err;
}

static void __hsi_write_cancel(struct hsi_channel *ch, int mode)
{
	if (mode == HSI_INTERRUPT_MODE)
		hsi_cancel_write_interrupt_mode(ch);
	else if (mode == HSI_DMA_MODE)
		hsi_cancel_write_dma_mode(ch);
	else
		stm_dbg(DBG_ST.hsi, "HSI write cancel Incorrect Mode \n");
		/*FIXME: No Polling Mode Yet */
}

/**
 * hsi_write_cancel - Cancel pending write request.
 * @dev: hsi device channel where to cancel the pending write.
 *
 * write_done() callback will not be called after sucess of this function.
 * This call closes the channel also.
 */
static void stm_hsi_write_cancel(struct hsi_device *dev)
{
	struct hsi_channel *ch;

	if (dev->cid != 0x0) {
		stm_dbg(DBG_ST.hsi, "HSI Controller not enabled for write\n");
		return;
	}

	if (unlikely(!dev || !dev->ch)) {
		stm_dbg(DBG_ST.hsi, "Wrong HSI device %p\n", dev);
		return;
	}
	if (unlikely(!(dev->ch->flags & HSI_CH_OPEN))) {
		stm_dbg(DBG_ST.hsi, "HSI device NOT open\n");
		return;
	}

	ch = dev->ch;
	spin_lock_bh(&ch->hsi_ch_lock);
	__hsi_write_cancel(ch, dev->curr_mode);
	dev->ch->flags &= ~HSI_CH_OPEN;
	spin_unlock_bh(&ch->hsi_ch_lock);
}

static void __hsi_read_cancel(struct hsi_channel *ch, int mode)
{
	if (mode == HSI_INTERRUPT_MODE)
		hsi_cancel_read_interrupt_mode(ch);
	else if (mode == HSI_DMA_MODE)
		hsi_cancel_read_dma_mode(ch);
	else
		stm_dbg(DBG_ST.hsi, "HSI read cancel Incorrect Mode \n");
		/*FIXME: No Polling Mode Yet */
}

/**
 * hsi_read_cancel - Cancel pending read request.
 * @dev: hsi device channel where to cancel the pending read.
 *
 * read_done() callback will not be called after sucess of this function.
 */
static void stm_hsi_read_cancel(struct hsi_device *dev)
{
	struct hsi_channel *ch;

	if (dev->cid != 0x1) {
		stm_dbg(DBG_ST.hsi, "HSI Controller not enabled for read\n");
		return;
	}

	if (unlikely(!dev || !dev->ch)) {
		stm_dbg(DBG_ST.hsi, "Wrong HSI device %p\n", dev);
		return;
	}

	if (unlikely(!(dev->ch->flags & HSI_CH_OPEN))) {
		stm_dbg(DBG_ST.hsi, "HSI device NOT open\n");
		return;
	}

	ch = dev->ch;
	spin_lock_bh(&ch->hsi_ch_lock);
	__hsi_read_cancel(dev->ch, dev->curr_mode);
	dev->ch->flags &= ~HSI_CH_OPEN;
	spin_unlock_bh(&ch->hsi_ch_lock);
}

/**
 * hsi_close - close given hsi device channel
 * @dev: reference to hsi device channel.
 */
static void stm_hsi_close(struct hsi_device *dev)
{
	struct hsi_channel *ch;
	void __iomem *hsi_base;
	struct hsi_controller *hsi_ctrlr;

	if (!dev || !dev->ch) {
		stm_dbg(DBG_ST.hsi, "Trying to close wrong HSI device %p\n",
			dev);
		return;
	}

	ch = dev->ch;
	hsi_ctrlr = dev->ctrlr;
	hsi_base = hsi_ctrlr->regbase;

	spin_lock_bh(&ch->hsi_ch_lock);

	if (ch->flags & HSI_CH_OPEN) {
		dev->ch->flags &= ~HSI_CH_OPEN;
		if (hsi_ctrlr->dev_type == 0x0) {
			/** HSIT */
			/** flush the transmit channel buffers */
			writel((readl(hsi_base + HSI_TX_BUFSTATE) &
				~((unsigned long)(1 << ch->channel_number))),
			       hsi_base + HSI_TX_BUFSTATE);
		} else {
			/** flush the receive channel buffers */
			writel((readl(hsi_base + HSI_RX_BUFSTATE) &
				~((unsigned long)(1 << ch->channel_number))),
			       hsi_base + HSI_RX_BUFSTATE);
		}

		clk_disable(hsi_ctrlr->clk);
	}

	spin_unlock_bh(&ch->hsi_ch_lock);
}

/**
 * hsi_dev_set_cb - register read_done() and write_done() callbacks.
 * @dev: reference to hsi device channel where callbacks are associated.
 * @r_cb: callback to signal read transfer completed.
 * @w_cb: callback to signal write transfer completed.
 */
static void stm_hsi_dev_set_cb(struct hsi_device *dev,
			       void (*r_cb) (struct hsi_device *dev)
			       , void (*w_cb) (struct hsi_device *dev))
{
	struct hsi_channel *ch;

	if (unlikely(!dev || !dev->ch)) {
		stm_dbg(DBG_ST.hsi, "Wrong HSI device %p\n", dev);
		return;
	}

	ch = dev->ch;
	spin_lock_bh(&ch->hsi_ch_lock);
	dev->ch->ops.read_done = r_cb;
	dev->ch->ops.write_done = w_cb;
	spin_unlock_bh(&ch->hsi_ch_lock);
}

/**
 * hsi_ioctl - HSI I/O control
 * @dev: hsi device channel reference to apply the I/O control
 * 	 or port associated to it)
 * @command: HSI I/O control command
 * @arg: parameter associated to the control command.NULL if no parameter.
 *
 * Return 0 on sucess, a negative value on failure.
**/
static int stm_hsi_ioctl(struct hsi_device *dev, unsigned int command,
			 void *arg)
{
	struct hsi_channel *ch;
	struct hsi_controller *hsi_ctrlr;
	void __iomem *hsi_base;
	int err = 0;

	if (!dev || !dev->ch) {
		stm_dbg(DBG_ST.hsi, "Wrong HSI device %p\n", dev);
		return -EINVAL;
	}

	ch = dev->ch;
	hsi_ctrlr = ch->ctrlr;

	if (unlikely(!(dev->ch->flags & HSI_CH_OPEN))) {
		stm_dbg(DBG_ST.hsi, "HSI device NOT open\n");
		return -EINVAL;
	}

	spin_lock_bh(&ch->hsi_ch_lock);

	hsi_base = hsi_ctrlr->regbase;

	switch (command) {
	case HSI_IOC_SET_WATERMARK:
	/** TODO: check span definition. Manual says (span-1)
	 * is actual span value and the watermark should be less
	 * than span (span -1)
	 **/
		if (*(u8 *) arg > (ch->span - 2)) {
			stm_dbg(DBG_ST.hsi,
				"HSI IOCTL: illegal watermark value > span\n");
			spin_unlock_bh(&ch->hsi_ch_lock);
			return -EINVAL;
		}
		ch->watermark = *(u8 *) arg;
		if (hsi_ctrlr->dev_type == 0x0)	{ /** HSI transmit controller*/
			writel((0x1f & ((*(u8 *) arg) - 1)), (hsi_base +
							    HSI_TX_WATERMARKX
							    +
							    (0x4 *
							     ch->
							     channel_number)));
		} else {
			writel((0x1f & ((*(u8 *) arg) - 1)), (hsi_base +
							    HSI_RX_WATERMARKX
							    +
							    (0x4 *
							    ch->
							    channel_number)));
		}
		break;
	case HSI_IOC_GET_WATERMARK:
		if (hsi_ctrlr->dev_type == 0x0) {
			/** HSI transmit controller*/
			/** add 1 to the value read from register
			 * for actual watermark
			 **/
			*((u8 *) arg) =
			    (0x1F &
			     (readl
			      (hsi_base + HSI_TX_WATERMARKX +
			       (0x4 * ch->channel_number)))) + 1;
		} else {
			*((u8 *) arg) =
			    (0x1F &
			     (readl
			      (hsi_base + HSI_RX_WATERMARKX +
			       (0x4 * ch->channel_number)))) + 1;
		}
		break;
	case HSI_IOC_GET_MODE:
		*((u8 *) arg) = dev->curr_mode;
		break;
	case HSI_IOC_SET_MODE:
		dev->curr_mode = *((u8 *) arg);
		break;
	case HSI_IOC_SEND_BREAK:
		writel(0x1, hsi_base + HSI_TX_BREAK);
		break;
	case HSI_IOC_SET_FRAMELEN:
		if ((*(u8 *) arg) > HSI_MAX_FRAMELEN) {
			stm_dbg(DBG_ST.hsi,
				"HSI IOCTL: Illegal framelen setting \n");
			spin_unlock_bh(&ch->hsi_ch_lock);
			return -EINVAL;
		}
		if (hsi_ctrlr->dev_type == 0x0) {
			/** HSI transmit controller*/
			writel((0x1f & ((*(u8 *) arg) - 1)), (hsi_base +
							  HSI_TX_FRAMELENX +
							  (0x4 *
							  ch->
							  channel_number)));
		} else {
			writel((0x1f & ((*(u8 *) arg) - 1)), (hsi_base +
							  HSI_RX_FRAMELENX +
							  (0x4 *
							  ch->
							  channel_number)));
		}
		dev->ch->datawidth = (*(u8 *) arg);
		break;
	case HSI_IOC_GET_FRAMELEN:
		if (hsi_ctrlr->dev_type == 0x0) {
			/** HSI transmit controller*/
			/** add 1 to the value read from register
			 * for actual framelen */
			*((u8 *) arg) =
			    (0x1F &
			     (readl
			      (hsi_base + HSI_TX_FRAMELENX +
			       (0x4 * ch->channel_number)))) + 1;
		} else {
			*((u8 *) arg) =
			    (0x1F &
			     (readl
			      (hsi_base + HSI_RX_FRAMELENX +
			       (0x4 * ch->channel_number)))) + 1;
		}
		break;

	case HSI_IOC_SET_THRESHOLD:
		writel((0x3f & (*(u32 *) arg)), (hsi_base + HSI_RX_THRESHOLD));
		break;

	default:
		err = -ENOIOCTLCMD;
		break;
	}

	spin_unlock_bh(&ch->hsi_ch_lock);
	return err;
}

/* NOTE: Function called in interrupt context */
static int stm_hsi_exception_handler(struct device_driver *drv, void *ex_event)
{
	struct hsi_ctrlr_excep *event = (struct hsi_ctrlr_excep *)ex_event;
	struct hsi_device_driver *hsi_drv = to_hsi_device_driver(drv);
	u8 id = 0;

	if ((hsi_drv->excep_event) &&
	    (test_bit
	     (event->event,
	      (const unsigned long *)&hsi_drv->excep_mask))
	    &&
	    (test_bit
	     (event->ctrlr->dev_type,
	      (const unsigned long *)&hsi_drv->ctrl_mask))
	    && (hsi_drv->ch_mask != 0)) {
		if (event->priv) {
			/** channel overrun - we pass the channel id
			 * as private data */
			id = *(u8 *) event->priv;
			if (test_bit
			    (id,
			     (const unsigned long *)&hsi_drv->ch_mask))
				/** callback for channel overrun */
				(*hsi_drv->excep_event) (event->ctrlr->dev_type,
							 event->event, &id);
		} else
			/** callback for everything except
			 * channel overrrun */
			(*hsi_drv->excep_event) (event->ctrlr->dev_type,
						 event->event, event->priv);
	}

	return 0;
}

struct hsi_algorithm hsi_algo = {
	.open = stm_hsi_open,
	.read = stm_hsi_read,
	.write = stm_hsi_write,
	.ioctl = stm_hsi_ioctl,
	.cancel_read = stm_hsi_read_cancel,
	.cancel_write = stm_hsi_write_cancel,
	.set_cb = stm_hsi_dev_set_cb,
	.close = stm_hsi_close,
	.exception_handler = stm_hsi_exception_handler,
};

static void reset_ch_read(struct hsi_channel *ch)
{
	ch->read_data.data = NULL;
	ch->read_data.dma_addr = (dma_addr_t) NULL;
	ch->read_data.count = 0;
}

static void reset_ch_write(struct hsi_channel *ch)
{
	ch->write_data.data = NULL;
	ch->write_data.dma_addr = (dma_addr_t) NULL;
	ch->write_data.count = 0;
}

int hsi_write_interrupt_mode(struct hsi_channel *ch)
{
	struct hsi_controller *hsi_ctrlr;
	void __iomem *hsi_base;
	unsigned int wmark_intrmask;

	stm_dbg(DBG_ST.hsi, "Write int mode\n");
	hsi_ctrlr = ch->ctrlr;
	hsi_base = hsi_ctrlr->regbase;

	/** watermark fixed to count-1 entries */
	/* writel((0x1f & ((ch->write_data.count)-1)),(hsi_base +
	 * HSI_TX_WATERMARKX + (0x4*ch->channel_number))); */
	/** enable the corresponding channel intr mask  */
	wmark_intrmask = (0xff) & readl(hsi_base + HSI_TX_WATERMARKIM);
	writel((wmark_intrmask | (1 << ch->channel_number)),
	       hsi_base + HSI_TX_WATERMARKIM);

	return 0;
}

int hsi_read_interrupt_mode(struct hsi_channel *ch)
{
	struct hsi_controller *hsi_ctrlr;
	void __iomem *hsi_base;
	unsigned int wmark_intrmask;

	stm_dbg(DBG_ST.hsi, "Read int mode\n");
	hsi_ctrlr = ch->ctrlr;
	hsi_base = hsi_ctrlr->regbase;

	/** watermark fixed to count-1 entries -
	 * not required as watermark is fixed
	 * writel((0x1f & ((ch->read_data.count)-1)),(hsi_base+
	 * HSI_RX_WATERMARKX + (0x4*ch->channel_number))); */

	/** enable the corresponding channel intr mask  */
	wmark_intrmask = (0xff) & readl(hsi_base + HSI_RX_WATERMARKIM);
	writel((wmark_intrmask | (1 << ch->channel_number)),
	       hsi_base + HSI_RX_WATERMARKIM);

	return 0;
}

void hsi_cancel_write_interrupt_mode(struct hsi_channel *ch)
{

	struct hsi_controller *hsi_ctrlr;
	void __iomem *hsi_base;
	unsigned int wmark_intrmask;

	hsi_ctrlr = ch->ctrlr;
	hsi_base = hsi_ctrlr->regbase;
	/** reset watermark to high number,say 4 */
	/*writel((0x1f & 0x3),(hsi_base+ HSI_TX_WATERMARKX +
	 * (0x4*ch->channel_number)));
	 */
	/** disable the corresponding channel intr mask  */
	wmark_intrmask = (0xff) & readl(hsi_base + HSI_TX_WATERMARKIM);
	writel((wmark_intrmask & ~(1 << ch->channel_number)),
	       hsi_base + HSI_TX_WATERMARKIM);
	/** flush the transmit channel buffers */
	writel((readl(hsi_base + HSI_TX_BUFSTATE) &
		~((unsigned long)(1 << ch->channel_number))),
	       hsi_base + HSI_TX_BUFSTATE);

	reset_ch_write(ch);

}

void hsi_cancel_read_interrupt_mode(struct hsi_channel *ch)
{
	struct hsi_controller *hsi_ctrlr;
	void __iomem *hsi_base;
	unsigned int wmark_intrmask;

	hsi_ctrlr = ch->ctrlr;
	hsi_base = hsi_ctrlr->regbase;
	/** reset watermark to high number,say 4 */
	/* writel((0x1f & 0x3),(hsi_base+ HSI_RX_WATERMARKX
	 * + (0x4*ch->channel_number)));
	 */

	/** disable the corresponding channel intr mask  */
	wmark_intrmask = (0xff) & readl(hsi_base + HSI_RX_WATERMARKIM);
	writel((wmark_intrmask & ~(1 << ch->channel_number)),
	       hsi_base + HSI_RX_WATERMARKIM);
	/** flush the receive channel buffers */
	writel((readl(hsi_base + HSI_RX_BUFSTATE) &
		~((unsigned long)(1 << ch->channel_number))),
	       hsi_base + HSI_RX_BUFSTATE);
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
	struct hsi_controller *hsi_ctrlr = ch->ctrlr;
	void __iomem *hsi_base = hsi_ctrlr->regbase;
	int i = 0;

	spin_lock(&ch->hsi_ch_lock);
	if (ch->write_data.data) {
		/*while ((i< (ch->num_xfer_perintr)) &&
		 * (ch->write_data.count)){ */
		while (!
		       ((readl(hsi_base + HSI_TX_BUFSTATE)) &
			(1 << ch->channel_number)) && (ch->write_data.count)) {
			writel(*(u8 *) (ch->write_data.data),
			       hsi_base + HSI_TX_BUFFERX +
			       (4 * ch->channel_number));
			ch->write_data.data = ch->write_data.data + ch->n_bytes;
			ch->write_data.count =
			    ch->write_data.count - ch->n_bytes;
			i++;
		}
		if (ch->write_data.count == 0x0)
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
	struct hsi_controller *hsi_ctrlr = ch->ctrlr;
	void __iomem *hsi_base = hsi_ctrlr->regbase;
	int i = 0;

	spin_lock(&ch->hsi_ch_lock);
	if (ch->read_data.data) {
		/*while ((i< (ch->num_xfer_perintr)) &&
		 * (ch->read_data.count)){ */
		while (((readl(hsi_base + HSI_RX_BUFSTATE)) &
			(1 << ch->channel_number)) && (ch->read_data.count)) {
			*(u8 *) (ch->read_data.data) =
			    readl(hsi_base + HSI_RX_BUFFERX +
				  (4 * ch->channel_number));
			/* stm_dbg(DBG_ST.hsi,"HSI RX: data 0x%x arrived \n",
			 * *(u8 *)(ch->read_data.data));
			 **/
			ch->read_data.data = ch->read_data.data + ch->n_bytes;
			ch->read_data.count = ch->read_data.count - ch->n_bytes;
			i++;
		}
		if (ch->read_data.count == 0x0)
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
	struct hsi_controller *hsi_ctrlr = ch->ctrlr;
	void __iomem *hsi_base = hsi_ctrlr->regbase;
	int i = 0;

	spin_lock(&ch->hsi_ch_lock);
	if (ch->write_data.data) {
		/*while ((i< (ch->num_xfer_perintr))
		 * && (ch->write_data.count)){ */
		while (!
		       ((readl(hsi_base + HSI_TX_BUFSTATE)) &
			(1 << ch->channel_number)) && (ch->write_data.count)) {
			writel(*(u16 *) (ch->write_data.data),
			       hsi_base + HSI_TX_BUFFERX +
			       (4 * ch->channel_number));
			ch->write_data.data = ch->write_data.data + ch->n_bytes;
			ch->write_data.count =
			    ch->write_data.count - ch->n_bytes;
			i++;
		}
		if (ch->write_data.count == 0x0)
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
	struct hsi_controller *hsi_ctrlr = ch->ctrlr;
	void __iomem *hsi_base = hsi_ctrlr->regbase;
	int i = 0;

	spin_lock(&ch->hsi_ch_lock);
	if (ch->read_data.data) {
		/*while ((i< (ch->num_xfer_perintr)) &&
		 * (ch->read_data.count)) { */
		while (((readl(hsi_base + HSI_RX_BUFSTATE)) &
			(1 << ch->channel_number)) && (ch->read_data.count)) {
			*(u16 *) (ch->read_data.data) =
			    readl(hsi_base + HSI_RX_BUFFERX +
				  (4 * ch->channel_number));
			/** stm_dbg(DBG_ST.hsi,"HSI RX: data 0x%x arrived \n",
			 * *(u16 *)(ch->read_data.data));
			 **/
			ch->read_data.data = ch->read_data.data + ch->n_bytes;
			ch->read_data.count = ch->read_data.count - ch->n_bytes;
			i++;
		}
		if (ch->read_data.count == 0x0)
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
	struct hsi_controller *hsi_ctrlr = ch->ctrlr;
	void __iomem *hsi_base = hsi_ctrlr->regbase;
	int i = 0;

	spin_lock(&ch->hsi_ch_lock);
	if (ch->write_data.data) {
		/*while (!((readl(hsi_base+HSI_TX_BUFSTATE)) &
		 * (1<<ch->channel_number))
		 * && (ch->write_data.count)) {
		 */
		while ((i < (ch->num_xfer_perintr)) && (ch->write_data.count)) {
			writel(*(u32 *) (ch->write_data.data),
			       hsi_base + HSI_TX_BUFFERX +
			       (4 * ch->channel_number));
			ch->write_data.data = ch->write_data.data + ch->n_bytes;
			ch->write_data.count =
			    ch->write_data.count - ch->n_bytes;
			i++;
		}
		if (ch->write_data.count == 0x0)
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
	struct hsi_controller *hsi_ctrlr = ch->ctrlr;
	void __iomem *hsi_base = hsi_ctrlr->regbase;
	int i = 0;

	spin_lock(&ch->hsi_ch_lock);
	if (ch->read_data.data) {
		/*
		 * while (((readl(hsi_base+HSI_RX_BUFSTATE))
		 * & (1<<ch->channel_number))
		 * && (ch->read_data.count)) {
		 */
		while ((i < (ch->num_xfer_perintr)) &&
				(ch->read_data.count)) {
			*(u32 *) (ch->read_data.data) =
			    readl(hsi_base + HSI_RX_BUFFERX +
				  (4 * ch->channel_number));
			/** stm_dbg(DBG_ST.hsi,"HSI RX: data 0x%x arrived \n",
			 * *(u32 *)(ch->read_data.data));
			 **/
			ch->read_data.data = ch->read_data.data + ch->n_bytes;
			ch->read_data.count = ch->read_data.count - ch->n_bytes;
			i++;
		}
		if (ch->read_data.count == 0x0)
			reset_ch_read(ch);
	}
	spin_unlock(&ch->hsi_ch_lock);
}

void do_hsi_tx_tasklet(unsigned long ctrlr)
{
	struct hsi_controller *hsi_ctrlr = (struct hsi_controller *)ctrlr;
	void __iomem *hsi_base = hsi_ctrlr->regbase;
	unsigned long wmark_intrstatus;
	unsigned long wmark_intrmask;
	struct hsi_channel *chnl;
	unsigned char i;

	stm_dbg(DBG_ST.hsi, "HSI TX tasklet\n");
	wmark_intrstatus = (0xFF) & readl(hsi_base + HSI_TX_WATERMARKIS);

	for (i = 0; i < HSI_MAX_CHANNELS; i++) {
		if ((wmark_intrstatus) & (1 << i)) {
			/** data needs to be transferred for this channel*/
			chnl = &hsi_ctrlr->hsi_tx_channels[i];
			/* do_channel_tx(chnl); */
			(*chnl->ops.write) (chnl);
			/** FIXME: HSI bug - clearing interrupt disables
			 * delivery of further interrupts
			 **/
			/* writel((1<<i), hsi_base+HSI_TX_WATERMARKIC);*/
			/** we have transferred the required number of
			 * bytes user requested, callback
			 **/
			spin_lock(&chnl->hsi_ch_lock);
			if ((chnl->write_data.count) <=
			    (chnl->watermark * chnl->n_bytes)) {
				/** TODO fix watermark on basis of bytes left */
				writel((0x1f & 0x0),
				       (hsi_base + HSI_TX_WATERMARKX +
					(0x4 * i)));
			}
			if (chnl->write_data.count == 0x0) {
				/** disable the corresponding watermark
				 * as we are done */
				wmark_intrmask =
				    (0xff) & readl(hsi_base +
						   HSI_TX_WATERMARKIM);
				writel((wmark_intrmask &
					~(1 << chnl->channel_number)),
				       hsi_base + HSI_TX_WATERMARKIM);
				/** write the original watermark */
				writel((0x1f & (chnl->watermark - 1)),
				       (hsi_base + HSI_TX_WATERMARKX +
					(0x4 * i)));
				spin_unlock(&chnl->hsi_ch_lock);

				/** callback */
				(*chnl->ops.write_done) (chnl->dev);
			} else
				spin_unlock(&chnl->hsi_ch_lock);
		}
	}
	enable_irq(hsi_ctrlr->irq1);
}

void do_hsi_rx_tasklet(unsigned long ctrlr)
{
	struct hsi_controller *hsi_ctrlr = (struct hsi_controller *)ctrlr;
	void __iomem *hsi_base = hsi_ctrlr->regbase;
	unsigned long wmark_intrstatus;
	unsigned long wmark_intrmask;
	struct hsi_channel *chnl;
	unsigned char i;
	int gauge;

	stm_dbg(DBG_ST.hsi, "HSI RX tasklet\n");
	/** enter sleep mode - to negate READY line so that
	* pipe buf is not overrun
	* writel((0x3 & 0x0),hsi_base+HSI_RX_MODE);
	**/
	wmark_intrstatus = (0xFF) & readl(hsi_base + HSI_RX_WATERMARKIS);

	for (i = 0; i < HSI_MAX_CHANNELS; i++) {
		if ((wmark_intrstatus) & (1 << i)) {
			/** data has arrived on this channel */
			/** stm_dbg(DBG_ST.hsi,"Data arrived
			 * on channel %d\n",i);*/
			chnl = &hsi_ctrlr->hsi_rx_channels[i];
			/** gaurd against spurious interrupts */
			gauge = (0x1F) & (readl(hsi_base + HSI_RX_GAUGEX
					+ (0x4 * chnl->channel_number)));
			if (!gauge) {
				/** clear and go away */
				writel((1<<i), hsi_base+HSI_RX_WATERMARKIC);
				continue;
			}

			/* do_channel_rx(chnl); */
			(*chnl->ops.read) (chnl);
			spin_lock(&chnl->hsi_ch_lock);
			writel((1 << i), hsi_base + HSI_RX_WATERMARKIC);
			if ((chnl->read_data.count) <=
			    (chnl->watermark * chnl->n_bytes))
				/**TODO fix wmark on basis of bytes left*/
				writel((0x1f & 0x0),
				       (hsi_base + HSI_RX_WATERMARKX +
					(0x4 * i)));

			/** we have the required number of bytes
			 * user requested, callback*/
			if (chnl->read_data.count == 0x0) {
				/** disable the corresponding watermark
				 * as we are done */
				wmark_intrmask =
				    (0xff) & readl(hsi_base +
						   HSI_RX_WATERMARKIM);
				writel((wmark_intrmask &
					~(1 << chnl->channel_number)),
				       hsi_base + HSI_RX_WATERMARKIM);
				/** write the original watermark */
				writel((0x1f & (chnl->watermark - 1)),
				       (hsi_base + HSI_RX_WATERMARKX +
					(0x4 * i)));
				spin_unlock(&chnl->hsi_ch_lock);
				(*chnl->ops.read_done) (chnl->dev);
			} else
				spin_unlock(&chnl->hsi_ch_lock);

		}
	}
	enable_irq(hsi_ctrlr->irq1);
	/** re-enter PIPELINE mode
	 *writel((0x3 & 0x3),hsi_base+HSI_RX_MODE); For MOP
	 **/
}

void do_hsi_rxexcep_tasklet(unsigned long ctrlr)
{
	struct hsi_controller *hsi_ctrlr = (struct hsi_controller *)ctrlr;
	void __iomem *hsi_base = NULL;
	u32 exmis = 0x0;
	u32 ovrmis = 0x0;
	u8 i;
	u8 chid = 0x0;

	hsi_base = hsi_ctrlr->regbase;
	exmis = (0xF) & readl(hsi_base + HSI_RX_EXCEPMIS);
	ovrmis = (0xFF) & readl(hsi_base + HSI_RX_OVERRUNMIS);

	if (exmis) {
		for (i = 0; i < 4; i++) {
			if ((1 << i) & exmis) {
				switch (i) {
				case 0x0:
					stm_dbg(DBG_ST.hsi,
						"HSIR:TIMEOUT !! \n");
					hsi_exception(hsi_ctrlr,
						      HSI_EXCEP_TIMEOUT, NULL);
					break;
				case 0x1:
					stm_dbg(DBG_ST.hsi,
						"HSIR:PIPEBUF OVERRUN !! \n");
						/** flush pipeline buffers
						 * and ack interrupt */
					writel(0x1,
					       hsi_base + HSI_RX_PIPEGAUGE);
					hsi_exception(hsi_ctrlr,
						   HSI_EXCEP_PIPEBUF_OVERRUN,
						   NULL);
					break;
				case 0x2:
					stm_dbg(DBG_ST.hsi,
						"HSIR:BRK_DETECTED!\n");
					hsi_exception(hsi_ctrlr,
						   HSI_EXCEP_BREAK_DETECTED,
						   NULL);
					break;
				case 0x3:
					stm_dbg(DBG_ST.hsi,
						"HSIR:PARITY ERROR !! \n");
					hsi_exception(hsi_ctrlr,
						      HSI_EXCEP_PARITY_ERROR,
						      NULL);
				}
				writel((1 << i), hsi_base + HSI_RX_ACK);
			}
		}
	}

	if (ovrmis) {
		while (ovrmis) {
			if (0x1 & ovrmis) {
				/** channel overrun ..
				 * flush the channel buffer */
				writel((readl(hsi_base + HSI_RX_BUFSTATE) &
					~((unsigned long)(1 << chid))),
				       hsi_base + HSI_RX_BUFSTATE);
				/** call the exception handler */
				hsi_exception(hsi_ctrlr,
					HSI_RXCHANNELS_OVERRUN,
					&chid);
				/** set the ack bit  */
				writel((1 << chid),
				       hsi_base + HSI_RX_OVERRUNACK);
			}
			ovrmis = ovrmis >> 1;
			chid++;
		}
	}

	enable_irq(hsi_ctrlr->irqexcep);
}

/** interrupt context : tx handler */
irqreturn_t hsi_tx_irq_handler(int irq, void *ctrlr)
{
	struct hsi_controller *hsi_ctrlr = ctrlr;
	tasklet_hi_schedule(&hsi_ctrlr->hsi_tx_tasklet);
	disable_irq_nosync(hsi_ctrlr->irq1);

	return IRQ_HANDLED;
}

/** interrupt context : rx handler */
irqreturn_t hsi_rx_irq_handler(int irq, void *ctrlr)
{
	struct hsi_controller *hsi_ctrlr = ctrlr;

	tasklet_hi_schedule(&hsi_ctrlr->hsi_rx_tasklet);
	disable_irq_nosync(hsi_ctrlr->irq1);

	return IRQ_HANDLED;
}

/** interrupt context : rx exception handler */
irqreturn_t hsi_rxexcep_irq_handler(int irq, void *ctrlr)
{
	int i = 0;
	struct hsi_controller *hsi_ctrlr = ctrlr;
	tasklet_hi_schedule(&hsi_ctrlr->hsi_rxexcep_tasklet);
	disable_irq_nosync(hsi_ctrlr->irqexcep);
	for (i = 0; i < hsi_ctrlr->max_ch; i++)
		disable_irq_nosync(hsi_ctrlr->irq_choverrun[i]);
	return IRQ_HANDLED;
}

void hsi_dma_write_eot_handler(void *data)
{
	struct hsi_channel *ch = (struct hsi_channel *)data;
	struct hsi_controller *hsi_ctrlr;
	unsigned int dma_mask;
	void __iomem *hsi_base;

	hsi_ctrlr = ch->ctrlr;
	hsi_base = hsi_ctrlr->regbase;

	stm_dbg(DBG_ST.hsi,
		"Callback:DMA Mem to Periph TC for HSI channel %d \n",
		ch->channel_number);

	/* For MOP */
	dma_mask = (0xff) & readl(hsi_base + HSI_TX_DMAEN);
	writel((dma_mask & (~(1 << ch->channel_number))),
	       hsi_base + HSI_TX_DMAEN);

	/** restore the original watermark */
	writel((0x1f & (ch->watermark - 1)),
	       (hsi_base + HSI_TX_WATERMARKX + (0x4 * ch->channel_number)));

	ch->write_data.data = NULL;
	ch->write_data.count = 0;
	/** disable the corresponding watermark as we are done
	* wmark_intrmask=(0xff) & readl(hsi_base+HSI_TX_WATERMARKIM);
	* writel((wmark_intrmask & ~(1<<ch->channel_number)),
	* hsi_base+HSI_TX_WATERMARKIM);
	*/
	/** call the application callback in tasklet
	* (*ch->ops.write_done)(ch->dev);
	*i*/
	tasklet_schedule(&ch->hsi_dma_tasklet);
}
EXPORT_SYMBOL(hsi_dma_write_eot_handler);

void hsi_dma_read_eot_handler(void *data)
{
	struct hsi_channel *ch = (struct hsi_channel *)data;
	struct hsi_controller *hsi_ctrlr;
	unsigned int dma_mask;
	void __iomem *hsi_base;

	hsi_ctrlr = ch->ctrlr;
	hsi_base = hsi_ctrlr->regbase;

	stm_dbg(DBG_ST.hsi,
		"DMA Periph to Mem Transfer Completed for HSI channel %d \n",
		ch->channel_number);

	/* For MOP */
	dma_mask = (0xff) & readl(hsi_base + HSI_RX_DMAEN);
	writel((dma_mask & (~(1 << ch->channel_number))),
	       hsi_base + HSI_RX_DMAEN);

	/** restore the original watermark */
	writel((0x1f & (ch->watermark - 1)),
	       (hsi_base + HSI_RX_WATERMARKX + (0x4 * ch->channel_number)));

	ch->read_data.data = NULL;
	ch->read_data.count = 0;
	/** disable the corresponding watermark as we are done
	* wmark_intrmask=(0xff) & readl(hsi_base+HSI_RX_WATERMARKIM);
	* writel((wmark_intrmask & ~(1<<ch->channel_number)),
	* hsi_base+HSI_RX_WATERMARKIM);
	**/

	/** call the application callback in tasklet
	*(*ch->ops.read_done)(ch->dev);
	**/
	tasklet_schedule(&ch->hsi_dma_tasklet);
}
EXPORT_SYMBOL(hsi_dma_read_eot_handler);

int hsi_write_dma_mode(struct hsi_channel *ch)
{
	/*We Need to do Mem to Periph DMA */
	struct hsi_controller *hsi_ctrlr;
	void __iomem *hsi_base;
	void *dst_addr = NULL;
	dma_addr_t src_addr;
	int count = 0;
	void *hsi_base_phy = (void *)0x8011C000;
	unsigned int dma_mask;
	struct dma_async_tx_descriptor *dmach_desc;
	struct scatterlist sg;

	stm_dbg(DBG_ST.hsi, "Write dma mode\n");
	hsi_ctrlr = ch->ctrlr;
	hsi_base = hsi_ctrlr->regbase;

	if (ch->write_data.is_dma) {
		src_addr = ch->write_data.dma_addr;
	} else {
		src_addr =
			dma_map_single(NULL, ch->write_data.data,
					   ch->write_data.count, DMA_TO_DEVICE);
		ch->write_data.dma_addr = src_addr;
	}
	dst_addr =
	    (void *)hsi_base_phy + HSI_TX_BUFFERX + (4 * ch->channel_number);
	count = ch->write_data.count;

	if (ch->watermark <= 0x3) {
		(void) stedma40_set_psize(ch->dma_chan,
					  STEDMA40_PSIZE_LOG_1,
					  STEDMA40_PSIZE_LOG_1);

		writel((0x1f & 0x0),
		       (hsi_base + HSI_TX_WATERMARKX +
			(0x4 * (ch->channel_number))));
	} else {
		(void) stedma40_set_psize(ch->dma_chan,
					  STEDMA40_PSIZE_LOG_4,
					  STEDMA40_PSIZE_LOG_4);

		writel((0x1f & 0x3),
		       (hsi_base + HSI_TX_WATERMARKX +
			(0x4 * (ch->channel_number))));
	}

	sg_init_table(&sg, 1);
	sg_set_page(&sg, pfn_to_page(PFN_DOWN(src_addr)), count,
		    offset_in_page(src_addr));
	sg_dma_address(&sg) = src_addr;
	sg_dma_len(&sg) = count;

	dmach_desc = ch->dma_chan->device->
		device_prep_slave_sg(ch->dma_chan,
				     &sg, 1,
				     DMA_TO_DEVICE,
				     DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!dmach_desc)
		return -ENOMEM;

	dmach_desc->callback = hsi_dma_write_eot_handler;
	dmach_desc->callback_param = ch;
	dmach_desc->tx_submit(dmach_desc);
	dma_async_issue_pending(ch->dma_chan);

	/*Enable DMA bits of HSI controller */
	dma_mask = (0xff) & readl(hsi_base + HSI_TX_DMAEN);
	writel((dma_mask | (1 << ch->channel_number)), hsi_base + HSI_TX_DMAEN);

	return 0;
}

int hsi_read_dma_mode(struct hsi_channel *ch)
{
	/*We Need to do Periph to Mem DMA */
	struct hsi_controller *hsi_ctrlr;
	void __iomem *hsi_base;
	dma_addr_t dst_addr;
	void *src_addr = NULL;
	int count = 0;
	unsigned int dma_mask;
	void *hsi_base_phy = (void *)0x8011B000;
	struct dma_async_tx_descriptor *dmach_desc;
	struct scatterlist sg;

	stm_dbg(DBG_ST.hsi, "Read dma mode\n");
	hsi_ctrlr = ch->ctrlr;
	hsi_base = hsi_ctrlr->regbase;

	src_addr = hsi_base_phy + HSI_RX_BUFFERX + (4 * ch->channel_number);
	if (ch->read_data.is_dma) {
		dst_addr = ch->read_data.dma_addr;
	} else {
		dst_addr =
			dma_map_single(NULL, ch->read_data.data,
					   ch->read_data.count,
					   DMA_FROM_DEVICE);
		ch->read_data.dma_addr = dst_addr;
	}
	count = ch->read_data.count;

	/** sync up the watermark and burst size */
	if (ch->watermark <= 0x3)
		(void) stedma40_set_psize(ch->dma_chan,
					  STEDMA40_PSIZE_LOG_1,
					  STEDMA40_PSIZE_LOG_1);
	else
		(void) stedma40_set_psize(ch->dma_chan,
					  STEDMA40_PSIZE_LOG_4,
					  STEDMA40_PSIZE_LOG_4);

	sg_init_table(&sg, 1);
	sg_set_page(&sg, pfn_to_page(PFN_DOWN(dst_addr)), count,
		    offset_in_page(dst_addr));
	sg_dma_address(&sg) = dst_addr;
	sg_dma_len(&sg) = count;

	dmach_desc = ch->dma_chan->device->
		device_prep_slave_sg(ch->dma_chan,
				     &sg, 1,
				     DMA_FROM_DEVICE,
				     DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!dmach_desc)
		return -ENOMEM;

	dmach_desc->callback = hsi_dma_read_eot_handler;
	dmach_desc->callback_param = ch;
	dmach_desc->tx_submit(dmach_desc);
	dma_async_issue_pending(ch->dma_chan);

	/*Enable DMA bits of HSI controller */
	dma_mask = (0xff) & readl(hsi_base + HSI_RX_DMAEN);
	writel((dma_mask | (1 << ch->channel_number)), hsi_base + HSI_RX_DMAEN);

	return 0;
}

void hsi_cancel_write_dma_mode(struct hsi_channel *ch)
{
	struct hsi_controller *hsi_ctrlr;
	void __iomem *hsi_base;
	unsigned int dma_mask;

	hsi_ctrlr = ch->ctrlr;
	hsi_base = hsi_ctrlr->regbase;

	ch->dma_chan->device->device_control(ch->dma_chan,
					     DMA_TERMINATE_ALL, 0);

	/*Disable DMA nebaling bits of HSI controller */
	dma_mask = (0xff) & readl(hsi_base + HSI_TX_DMAEN);
	writel((dma_mask & (~(1 << ch->channel_number))),
	       hsi_base + HSI_TX_DMAEN);
	/** flush the transmit channel buffers */
	writel((readl(hsi_base + HSI_TX_BUFSTATE) &
		~((unsigned long)(1 << ch->channel_number))),
	       hsi_base + HSI_TX_BUFSTATE);
	/** restore the original watermark */
	writel((0x1f & (ch->watermark - 1)),
	       (hsi_base + HSI_TX_WATERMARKX + (0x4 * ch->channel_number)));

	ch->write_data.data = NULL;
	ch->write_data.dma_addr = (dma_addr_t) NULL;
	ch->write_data.count = 0;
}

void hsi_cancel_read_dma_mode(struct hsi_channel *ch)
{
	struct hsi_controller *hsi_ctrlr;
	void __iomem *hsi_base;
	unsigned int dma_mask;

	hsi_ctrlr = ch->ctrlr;
	hsi_base = hsi_ctrlr->regbase;

	ch->dma_chan->device->device_control(ch->dma_chan,
					     DMA_TERMINATE_ALL, 0);

	/*Disable DMA nebaling bits of HSI controller */
	dma_mask = (0xff) & readl(hsi_base + HSI_RX_DMAEN);
	writel((dma_mask & (~(1 << ch->channel_number))),
	       hsi_base + HSI_RX_DMAEN);
	/** flush the receive channel buffers */
	writel((readl(hsi_base + HSI_RX_BUFSTATE) &
		~((unsigned long)(1 << ch->channel_number))),
	       hsi_base + HSI_RX_BUFSTATE);
	/** restore the original watermark */
	writel((0x1f & (ch->watermark - 1)),
	       (hsi_base + HSI_RX_WATERMARKX + (0x4 * ch->channel_number)));

	ch->read_data.data = NULL;
	ch->read_data.dma_addr = (dma_addr_t) NULL;
	ch->read_data.count = 0;
}

void do_hsi_tx_dma_tasklet(unsigned long param)
{
	struct hsi_channel *ch = (struct hsi_channel *)param;

	ch->ops.write_done(ch->dev);

}
EXPORT_SYMBOL(do_hsi_tx_dma_tasklet);

void do_hsi_rx_dma_tasklet(unsigned long param)
{
	struct hsi_channel *ch = (struct hsi_channel *)param;

	ch->ops.read_done(ch->dev);

}
EXPORT_SYMBOL(do_hsi_rx_dma_tasklet);

/** End of file **/
