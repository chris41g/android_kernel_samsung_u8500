/*
 *
 * Copyright (C) 2009 ST-Ericsson SA
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

#ifndef __STM_MUSB_DMA_H__
#define __STM_MUSB_DMA_H__

#define MUSB_HSDMA_CHANNELS     16
struct musb_dma_controller;
struct dma_chan;
struct stedma40_chan_cfg;
struct musb_dma_channel {
	struct dma_channel              channel;
	struct musb_dma_controller      *controller;
	struct stedma40_chan_cfg *info;
	struct musb_hw_ep       *hw_ep;
	struct work_struct channel_data_tx;
	struct work_struct channel_data_rx;
	u32                             start_addr;
	u32                             len;
	u32                             is_pipe_allocated;
	u16                             max_packet_sz;
	u8                              idx;
	struct dma_chan			*dma_chan;
	unsigned int cur_len;
	u8                              epnum;
	u8                              last_xfer;
	u8                              transmit;
};

struct musb_dma_controller {
	struct dma_controller           controller;
	struct musb_dma_channel         channel[MUSB_HSDMA_CHANNELS];
	void                            *private_data;
	void __iomem                    *base;
	u8                              channel_count;
	u8                              used_channels;
	u8                              irq;
};
void musb_rx_dma_controller_handler(void *private_data);
void musb_tx_dma_controller_handler(void *private_data);
static void musb_channel_work_tx(struct work_struct *data);
static void musb_channel_work_rx(struct work_struct *data);
#endif

