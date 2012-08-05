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

#ifndef __STE_CONFIG_H__
#define __STE_CONFIG_H__

#if (defined(CONFIG_ARCH_U8500) && !defined(CONFIG_MUSB_PIO_ONLY))
#define CONFIG_USB_U8500_DMA
#endif
#define U8500_DMA_END_POINTS 7
#define DMA_MODE_0 0
#define DMA_MODE_1 1
#define DMA_PACKET_THRESHOLD 512
#define RX_END_POINT_OFFSET 6
#define DELAY_IN_MICROSECONDS 10
#define MAX_COUNT 35000
void stm_prcmu_qos_handler(int);
#define SET_OPP 1

enum nmdk_dma_tx_rx_channel {
	TX_CHANNEL_1 = 0,
	TX_CHANNEL_2,
	TX_CHANNEL_3,
	TX_CHANNEL_4,
	TX_CHANNEL_5,
	TX_CHANNEL_6,
	TX_CHANNEL_7,
	RX_CHANNEL_1,
	RX_CHANNEL_2,
	RX_CHANNEL_3,
	RX_CHANNEL_4,
	RX_CHANNEL_5,
	RX_CHANNEL_6,
	RX_CHANNEL_7
};
#endif

