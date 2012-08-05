/* Copyright (C) 2009 ST-Ericsson SA
 * Copyright (C) 2009 STMicroelectronics
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/pfn.h>
#include "musb_core.h"
#include "ste_config.h"
#include "stm_musb_dma.h"
#include <plat/ste_dma40.h>
#include <mach/ste-dma40-db8500.h>

/*
 * U8500 system DMA used for USB can't transfer less
 * than max packet size of Buldk EP which is 512
 */
#define U8500_USB_DMA_MIN_TRANSFER_SIZE	512

/**
 * dma_controller_start() - creates the logical channels pool and registers callbacks
 * @c:	pointer to DMA Controller
 *
 * This function requests the logical channels from the DMA driver and creates
 * logical channels based on event lines and also registers the callbacks which
 * are invoked after data transfer in the transmit or receive direction.
*/

static int dma_controller_start(struct dma_controller *c)
{
	struct musb_dma_controller *controller = container_of(c,
			struct musb_dma_controller, controller);
	struct musb_dma_channel *musb_channel = NULL;
	struct stedma40_chan_cfg *info;
	u8 bit;
	struct dma_channel *channel = NULL;
	/*bit 0 for receive and bit 1 for transmit*/
#ifndef CONFIG_USB_U8500_DMA
	for (bit = 0; bit < 2; bit++) {
#else
	for (bit = 0; bit < (U8500_DMA_END_POINTS*2); bit++) {
#endif
		dma_cap_mask_t mask;

		dma_cap_zero(mask);
		dma_cap_set(DMA_SLAVE, mask);

		musb_channel = &(controller->channel[bit]);
		info = kzalloc(sizeof(struct stedma40_chan_cfg), GFP_KERNEL);
		if (!info) {
			ERR("could not allocate dma info structure\n");
			return -1;
		}
		musb_channel->info = info;
		musb_channel->controller = controller;
#ifdef CONFIG_USB_U8500_DMA
		info->high_priority = true;
#else
		info->mode = STEDMA40_MODE_PHYSICAL;
		info->high_priority = true;
#endif

#ifndef CONFIG_USB_U8500_DMA
		if (bit) {
#else
		if ((bit <= TX_CHANNEL_7)) {
#endif
			int dst_dev_type;

			info->dir = STEDMA40_MEM_TO_PERIPH;
			info->src_dev_type = STEDMA40_DEV_SRC_MEMORY;

#ifdef CONFIG_USB_U8500_DMA
			switch (bit) {

			case TX_CHANNEL_1:
				dst_dev_type = DB8500_DMA_DEV38_USB_OTG_OEP_1_9;
				break;
			case TX_CHANNEL_2:
				dst_dev_type =
					 DB8500_DMA_DEV37_USB_OTG_OEP_2_10;
				break;
			case TX_CHANNEL_3:
				dst_dev_type =
					 DB8500_DMA_DEV36_USB_OTG_OEP_3_11;
				break;
			case TX_CHANNEL_4:
				dst_dev_type =
					 DB8500_DMA_DEV19_USB_OTG_OEP_4_12;
				break;
			case TX_CHANNEL_5:
				dst_dev_type =
					 DB8500_DMA_DEV18_USB_OTG_OEP_5_13;
				break;
			case TX_CHANNEL_6:
				dst_dev_type =
					 DB8500_DMA_DEV17_USB_OTG_OEP_6_14;
				break;
			case TX_CHANNEL_7:
				dst_dev_type =
					 DB8500_DMA_DEV16_USB_OTG_OEP_7_15;
				break;

			}

			info->dst_dev_type = dst_dev_type;
#endif

		} else {
			int src_dev_type;

			info->dir = STEDMA40_PERIPH_TO_MEM;

#ifdef CONFIG_USB_U8500_DMA
			switch (bit) {
			case RX_CHANNEL_1:
				src_dev_type = DB8500_DMA_DEV38_USB_OTG_IEP_1_9;
				break;
			case RX_CHANNEL_2:
				src_dev_type =
					DB8500_DMA_DEV37_USB_OTG_IEP_2_10;
				break;
			case RX_CHANNEL_3:
				src_dev_type =
					DB8500_DMA_DEV36_USB_OTG_IEP_3_11;
				break;
			case RX_CHANNEL_4:
				src_dev_type =
					DB8500_DMA_DEV19_USB_OTG_IEP_4_12;
				break;
			case RX_CHANNEL_5:
				src_dev_type =
					DB8500_DMA_DEV18_USB_OTG_IEP_5_13;
				break;
			case RX_CHANNEL_6:
				src_dev_type =
					DB8500_DMA_DEV17_USB_OTG_IEP_6_14;
				break;
			case RX_CHANNEL_7:
				src_dev_type =
					DB8500_DMA_DEV16_USB_OTG_IEP_7_15;
				break;
			}

			info->src_dev_type = src_dev_type;
#endif
			info->dst_dev_type = STEDMA40_DEV_DST_MEMORY;
		}
		info->src_info.data_width = STEDMA40_WORD_WIDTH;
		info->src_info.psize = STEDMA40_PSIZE_LOG_16;

		info->dst_info.data_width = STEDMA40_WORD_WIDTH ;
		info->dst_info.psize = STEDMA40_PSIZE_LOG_16;
		musb_channel->is_pipe_allocated = 1;
		channel = &(musb_channel->channel);
		channel->private_data = musb_channel;
		channel->status = MUSB_DMA_STATUS_FREE;
		channel->max_len = 0x10000;
		/* Tx => mode 1; Rx => mode 0 */
		channel->desired_mode = bit;
		channel->actual_len = 0;

		musb_channel->dma_chan = dma_request_channel(mask,
							     stedma40_filter,
							     info);
		if (!musb_channel->dma_chan) {
			ERR("dma pipe can't be allocated\n");
			return -1;
		}
#ifndef CONFIG_USB_U8500_DMA
		/* Tx => mode 1; Rx => mode 0 */
		if (bit) {
#else
		if ((bit <= TX_CHANNEL_7)) {
#endif
			INIT_WORK(&musb_channel->channel_data_tx,
				musb_channel_work_tx);
			DBG(2, "channel allocated for TX, %s\n",
			    dma_chan_name(musb_channel->dma_chan));
		} else {
			INIT_WORK(&musb_channel->channel_data_rx,
				musb_channel_work_rx);
			DBG(2, "channel allocated for RX, %s\n",
			    dma_chan_name(musb_channel->dma_chan));
		}

	}
	return 0;
}

static void dma_channel_release(struct dma_channel *channel);

/**
 * dma_controller_stop() - releases all the channels and frees the DMA pipes
 * @c: pointer to DMA controller
 *
 * This function frees all of the logical channels and frees the DMA pipes
*/

static int dma_controller_stop(struct dma_controller *c)
{
	struct musb_dma_controller *controller = container_of(c,
			struct musb_dma_controller, controller);
	struct musb_dma_channel *musb_channel;
	struct dma_channel *channel;
	u8 bit;
#ifndef CONFIG_USB_U8500_DMA
	for (bit = 0; bit < 2; bit++) {
#else
	for (bit = 0; bit < (U8500_DMA_END_POINTS*2); bit++) {
#endif
		channel = &controller->channel[bit].channel;
		musb_channel = channel->private_data;
		dma_channel_release(channel);
		if (musb_channel->info) {
			dma_release_channel(musb_channel->dma_chan);
			kfree(musb_channel->info);
			musb_channel->info = NULL;
		}
	}

	return 0;
}

/**
 * dma_controller_allocate() - allocates the DMA channels
 * @c: pointer to DMA controller
 * @hw_ep: pointer to endpoint
 * @transmit: transmit or receive direction
 *
 * This function allocates the DMA channel and initializes
 * the channel
*/

static struct dma_channel *dma_channel_allocate(struct dma_controller *c,
				struct musb_hw_ep *hw_ep, u8 transmit)
{
	struct musb_dma_controller *controller = container_of(c,
			struct musb_dma_controller, controller);
	struct musb_dma_channel *musb_channel = NULL;
	struct dma_channel *channel = NULL;
	u8 bit;

#ifndef CONFIG_USB_U8500_DMA

	 /*bit 0 for receive and bit 1 for transmit*/
	for (bit = 0; bit < 2; bit++) {
		if (!(controller->used_channels & (1 << bit))) {

			if ((transmit && !bit))
				continue;
			if ((!transmit && bit))
				break;
#else
			if (hw_ep->epnum > 0
				&& hw_ep->epnum <= U8500_DMA_END_POINTS) {
				if (transmit)
					bit = hw_ep->epnum - 1;
				else
					bit =
					hw_ep->epnum + RX_END_POINT_OFFSET;
			} else
				return NULL;
#endif
			controller->used_channels |= (1 << bit);
			musb_channel = &(controller->channel[bit]);
			musb_channel->idx = bit;
			musb_channel->epnum = hw_ep->epnum;
			musb_channel->hw_ep = hw_ep;
			musb_channel->transmit = transmit;
			musb_channel->is_pipe_allocated = 1;
			channel = &(musb_channel->channel);
#ifndef CONFIG_USB_U8500_DMA
			break;
		}
	}
#endif
	return channel;
}

/**
 * dma_channel_release() - releases the DMA channel
 * @channel:	channel to be released
 *
 * This function releases the DMA channel
 *
*/

static void dma_channel_release(struct dma_channel *channel)
{
	struct musb_dma_channel *musb_channel = channel->private_data;
	channel->actual_len = 0;
	musb_channel->start_addr = 0;
	musb_channel->len = 0;

	DBG(2, "enter\n");
	musb_channel->controller->used_channels &=
		~(1 << musb_channel->idx);

	channel->status = MUSB_DMA_STATUS_FREE;
	if (musb_channel->is_pipe_allocated)
		musb_channel->is_pipe_allocated = 0;
	DBG(2, "exit\n");
}

/**
 * configure_channel() - configures the source, destination addresses and
 *                       starts the transfer
 * @channel:    pointer to DMA channel
 * @packet_sz:  packet size
 * @mode: Dma mode
 * @dma_addr: DMA source address for transmit direction
 *            or DMA destination address for receive direction
 * @len: length
 * This function configures the source and destination addresses for DMA
 * operation and initiates the DMA transfer
*/

static bool configure_channel(struct dma_channel *channel,
				u16 packet_sz, u8 mode,
				dma_addr_t dma_addr, u32 len)
{
	struct musb_dma_channel *musb_channel = channel->private_data;
	struct musb_hw_ep       *hw_ep = musb_channel->hw_ep;
	struct musb *musb = hw_ep->musb;
	void __iomem *mbase = musb->mregs;
	u32 dma_count;
	struct dma_chan *dma_chan = musb_channel->dma_chan;
	struct dma_async_tx_descriptor *dma_desc;
	enum dma_data_direction direction;
	struct scatterlist sg;

#ifndef CONFIG_USB_U8500_DMA
	struct musb_qh          *qh;
	struct urb              *urb;
#endif
	unsigned int usb_fifo_addr =
		(unsigned int)(MUSB_FIFO_OFFSET(hw_ep->epnum) + mbase);

#ifndef CONFIG_USB_U8500_DMA
	if (musb_channel->transmit)
		qh = hw_ep->out_qh;
	else
		qh = hw_ep->in_qh;
	urb = next_urb(qh);
#endif

	dma_count = len - (len % packet_sz);
	musb_channel->cur_len = dma_count;
	usb_fifo_addr =
		U8500_USBOTG_BASE + ((unsigned int)usb_fifo_addr & 0xFFFF);

#ifndef CONFIG_USB_U8500_DMA
	if (!(urb->transfer_flags & URB_NO_TRANSFER_DMA_MAP)) {
		if (musb_channel->transmit)
			dma_addr = musb->tx_dma_phy;
		else
			dma_addr = musb->rx_dma_phy;
	}
#endif

	stedma40_set_dev_addr(dma_chan, usb_fifo_addr, usb_fifo_addr);

	sg_init_table(&sg, 1);
	sg_set_page(&sg, pfn_to_page(PFN_DOWN(dma_addr)), dma_count,
		    offset_in_page(dma_addr));
	sg_dma_address(&sg) = dma_addr;
	sg_dma_len(&sg) = dma_count;

	direction = musb_channel->transmit ? DMA_TO_DEVICE : DMA_FROM_DEVICE;
	dma_desc = dma_chan->device->
			device_prep_slave_sg(dma_chan, &sg, 1, direction,
					     DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!dma_desc)
		return false;

	dma_desc->callback = musb_channel->transmit ?
				musb_tx_dma_controller_handler :
				musb_rx_dma_controller_handler;
	dma_desc->callback_param = channel;
	dma_desc->tx_submit(dma_desc);
	dma_async_issue_pending(dma_chan);

	return true;
}

/**
 * dma_channel_program() - Configures the channel and initiates transfer
 * @channel:	pointer to DMA channel
 * @packet_sz:	packet size
 * @mode: mode
 * @dma_addr: physical address of memory
 * @len: length
 *
 * This function configures the channel and initiates the DMA transfer
*/

static int dma_channel_program(struct dma_channel *channel,
				u16 packet_sz, u8 mode,
				dma_addr_t dma_addr, u32 len)
{
	struct musb_dma_channel *musb_channel = channel->private_data;
	bool ret;

	BUG_ON(channel->status == MUSB_DMA_STATUS_UNKNOWN ||
		channel->status == MUSB_DMA_STATUS_BUSY);
	if (len < U8500_USB_DMA_MIN_TRANSFER_SIZE)
		return false;
	if (!musb_channel->transmit && len < packet_sz)
		return false;
	channel->actual_len = 0;
	musb_channel->start_addr = dma_addr;
	musb_channel->len = len;
	musb_channel->max_packet_sz = packet_sz;
	channel->status = MUSB_DMA_STATUS_BUSY;


	if ((mode == 1) && (len >= packet_sz))
		ret = configure_channel(channel, packet_sz, 1, dma_addr, len);
	else
		ret = configure_channel(channel, packet_sz, 0, dma_addr, len);
	return ret;
}

/**
 * dma_channel_abort() - aborts the DMA transfer
 * @channel:	pointer to DMA channel.
 *
 * This function aborts the DMA transfer.
*/

static int dma_channel_abort(struct dma_channel *channel)
{
	struct musb_dma_channel *musb_channel = channel->private_data;
	void __iomem *mbase = musb_channel->controller->base;
	u16 csr;
	if (channel->status == MUSB_DMA_STATUS_BUSY) {
		if (musb_channel->transmit) {

			csr = musb_readw(mbase,
				MUSB_EP_OFFSET(musb_channel->epnum,
						MUSB_TXCSR));
			csr &= ~(MUSB_TXCSR_AUTOSET |
				 MUSB_TXCSR_DMAENAB |
				 MUSB_TXCSR_DMAMODE);
			musb_writew(mbase,
				MUSB_EP_OFFSET(musb_channel->epnum, MUSB_TXCSR),
				csr);
		} else {
			csr = musb_readw(mbase,
				MUSB_EP_OFFSET(musb_channel->epnum,
						MUSB_RXCSR));
			csr &= ~(MUSB_RXCSR_AUTOCLEAR |
				 MUSB_RXCSR_DMAENAB |
				 MUSB_RXCSR_DMAMODE);
			musb_writew(mbase,
				MUSB_EP_OFFSET(musb_channel->epnum, MUSB_RXCSR),
				csr);
		}

		if (musb_channel->is_pipe_allocated) {
			musb_channel->dma_chan->device->
				device_control(musb_channel->dma_chan, DMA_TERMINATE_ALL, 0);
			channel->status = MUSB_DMA_STATUS_FREE;
		}
	}
	return 0;
}

/*undefined to avoid build warnings*/
#undef DBG
#undef WARNING
#undef INFO
#include <linux/usb/composite.h>

static int dma_is_compatible(struct dma_channel *channel,
		struct musb_request *req)
{

	struct musb_dma_channel *musb_channel = channel->private_data;
	struct musb_hw_ep       *hw_ep = musb_channel->hw_ep;
	struct musb *musb = hw_ep->musb;
	struct usb_descriptor_header **descriptors;
	struct usb_function		*f;

	struct usb_gadget		*gadget = &musb->g;

	struct usb_composite_dev	*cdev = get_gadget_data(gadget);

	list_for_each_entry(f, &cdev->config->functions, list) {

		if (!strcmp(f->name, "cdc_ethernet") ||
						!strcmp(f->name, "rndis") ||
						!strcmp(f->name, "phonet") ||
						!strcmp(f->name, "adb") ||
						!strcmp(f->name, "acm")) {

			if (gadget->speed == USB_SPEED_HIGH)
				descriptors = f->hs_descriptors;
			else
				descriptors = f->descriptors;

			for (; *descriptors; ++descriptors) {
				struct usb_endpoint_descriptor *ep;

				if ((*descriptors)->bDescriptorType !=
				USB_DT_ENDPOINT)
					continue;

				ep = (struct usb_endpoint_descriptor *)
					 *descriptors;

				if (ep->bEndpointAddress == req->epnum)
					return 0;
			}

		}
	}

	if (req->request.length < U8500_USB_DMA_MIN_TRANSFER_SIZE)
		return 0;

	return 1;
}

/**
 * musb_rx_dma_controller_handler() - callback invoked when the data is received in the receive direction
 * @private_data: DMA channel
 *
 * This callback is invoked when the DMA transfer is completed
 * in the receive direction.
*/
void musb_rx_dma_controller_handler(void *private_data)
{
	struct dma_channel      *channel = (struct dma_channel *)private_data;
	struct musb_dma_channel *musb_channel = channel->private_data;
#ifndef CONFIG_USB_U8500_DMA
	struct musb_hw_ep       *hw_ep = musb_channel->hw_ep;
	struct musb *musb = hw_ep->musb;
	void __iomem *mbase = musb->mregs;
	unsigned long flags, pio;
	unsigned int rxcsr;
	struct musb_qh          *qh = hw_ep->in_qh;
	struct urb              *urb;
	spin_lock_irqsave(&musb->lock, flags);
	urb = next_urb(qh);
	musb_ep_select(mbase, hw_ep->epnum);
	channel->actual_len = musb_channel->cur_len;
	pio = musb_channel->len - channel->actual_len;
	if (!(urb->transfer_flags & URB_NO_TRANSFER_DMA_MAP)) {
		memcpy(urb->transfer_buffer,
			(void *)musb->rx_dma_log, channel->actual_len);
		musb_memcpy(urb->transfer_buffer,
			(void *)musb->rx_dma_log, channel->actual_len);
	}
	if (!pio) {
		channel->status = MUSB_DMA_STATUS_FREE;
		musb_dma_completion(musb, musb_channel->epnum,
			musb_channel->transmit);
	}
	spin_unlock_irqrestore(&musb->lock, flags);
#else
	schedule_work(&musb_channel->channel_data_rx);
#endif
}

/**
 * musb_tx_dma_controller_handler() - callback invoked on the transmit direction DMA data transfer
 * @private_data:	pointer to DMA channel.
 *
 * This callback is invoked when the DMA tranfer is completed
 * in the transmit direction
*/

void musb_tx_dma_controller_handler(void *private_data)
{
	struct dma_channel      *channel = (struct dma_channel *)private_data;
	struct musb_dma_channel *musb_channel = channel->private_data;
#ifndef CONFIG_USB_U8500_DMA
	struct musb_hw_ep       *hw_ep = musb_channel->hw_ep;
	struct musb *musb = hw_ep->musb;
	void __iomem *mbase = musb->mregs;
	unsigned long flags, pio;
	unsigned int txcsr;
	struct musb_qh          *qh = hw_ep->out_qh;
	struct urb              *urb;
	spin_lock_irqsave(&musb->lock, flags);
	musb_ep_select(mbase, hw_ep->epnum);
	channel->actual_len = musb_channel->cur_len;
	pio = musb_channel->len - channel->actual_len;
	if (!pio) {
		channel->status = MUSB_DMA_STATUS_FREE;
		musb_dma_completion(musb, musb_channel->epnum,
			musb_channel->transmit);
	}
	if (pio) {
		channel->status = MUSB_DMA_STATUS_FREE;
		urb = next_urb(qh);
		qh->offset += channel->actual_len;
		buf = urb->transfer_buffer + qh->offset;
		musb_write_fifo(hw_ep, pio, buf);
		qh->segsize = pio;
		musb_writew(hw_ep->regs, MUSB_TXCSR, MUSB_TXCSR_TXPKTRDY);
	}
	spin_unlock_irqrestore(&musb->lock, flags);
#else
	schedule_work(&musb_channel->channel_data_tx);
#endif
}

/**
 * dma_controller_destroy() - deallocates the DMA controller
 * @c:	pointer to dma controller.
 *
 * This function deallocates the DMA controller.
*/

void dma_controller_destroy(struct dma_controller *c)
{
	struct musb_dma_controller *controller = container_of(c,
			struct musb_dma_controller, controller);
	struct musb_dma_channel *musb_channel = NULL;
	u8 bit;
	if (!controller)
		return;
	for (bit = 0; bit < MUSB_HSDMA_CHANNELS; bit++)
		musb_channel = &(controller->channel[bit]);
	if (controller->irq)
		free_irq(controller->irq, c);

	kfree(controller);
}

/**
 * musb_channel_work_tx() - Invoked by worker thread
 * @data: worker queue data
 *
 * This function is invoked by worker thread when the DMA transfer
 * is completed in the transmit direction.
*/

static void musb_channel_work_tx(struct work_struct *data)
{
	struct musb_dma_channel *musb_channel = container_of(data,
		struct musb_dma_channel, channel_data_tx);
	struct musb_hw_ep       *hw_ep = musb_channel->hw_ep;
	struct musb *musb = hw_ep->musb;
	unsigned long flags;
	spin_lock_irqsave(&musb->lock, flags);
	musb_channel->channel.actual_len = musb_channel->cur_len;
	musb_channel->channel.status = MUSB_DMA_STATUS_FREE;
	musb_ep_select(musb->mregs, hw_ep->epnum);
	musb_dma_completion(musb, musb_channel->epnum,
	musb_channel->transmit);
	spin_unlock_irqrestore(&musb->lock, flags);
}

/**
 * musb_channel_work_tx() - Invoked by worker thread
 * @data: worker queue data
 *
 * This function is invoked by worker thread when the
 * DMA transfer is completed in the receive direction.
*/

static void musb_channel_work_rx(struct work_struct *data)
{
	struct musb_dma_channel *musb_channel = container_of(data,
		struct musb_dma_channel, channel_data_rx);
	struct musb_hw_ep       *hw_ep = musb_channel->hw_ep;
	struct musb *musb = hw_ep->musb;
	void __iomem *mbase = musb->mregs;
	unsigned long flags;
	spin_lock_irqsave(&musb->lock, flags);
	musb_channel->channel.actual_len = musb_channel->cur_len;
	musb_channel->channel.status = MUSB_DMA_STATUS_FREE;
	musb_ep_select(mbase, hw_ep->epnum);
	musb_dma_completion(musb, musb_channel->epnum,
		musb_channel->transmit);
	spin_unlock_irqrestore(&musb->lock, flags);
}

/**
 * dma_controller_create() - creates the dma controller and initializes callbacks
 *
 * @musb:	pointer to mentor core driver data instance|
 * @base:	base address of musb registers.
 *
 * This function creates the DMA controller and initializes the callbacks
 * that are invoked from the Mentor IP core.
*/

struct dma_controller *__init
dma_controller_create(struct musb *musb, void __iomem *base)
{
	struct musb_dma_controller *controller;

	controller = kzalloc(sizeof(*controller), GFP_KERNEL);
	if (!controller)
		return NULL;

	controller->channel_count = MUSB_HSDMA_CHANNELS;
	controller->private_data = musb;
	controller->base = base;

	controller->controller.start = dma_controller_start;
	controller->controller.stop = dma_controller_stop;
	controller->controller.channel_alloc = dma_channel_allocate;
	controller->controller.channel_release = dma_channel_release;
	controller->controller.channel_program = dma_channel_program;
	controller->controller.channel_abort = dma_channel_abort;
	controller->controller.is_compatible = dma_is_compatible;

	return &controller->controller;
}
