/*---------------------------------------------------------------------------*/
/* Copyright ST Ericsson, 2009.                                              */
/* This program is free software; you can redistribute it and/or modify it   */
/* under the terms of the GNU General Public License as published by the     */
/* Free Software Foundation; either version 2.1 of the License, or	     */
/* (at your option) any later version.                                       */
/*                                                                           */
/* This program is distributed in the hope that it will be useful, but 	     */
/* WITHOUT ANY WARRANTY; without even the implied warranty of 		     */
/* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 		     */
/* See the GNU General Public License for more details.   		     */
/*                                                                           */
/* You should have received a copy of the GNU General Public License         */
/* along with this program. If not, see <http://www.gnu.org/licenses/>.      */
/*---------------------------------------------------------------------------*/

#ifndef __MODEM_SSI_H_INCLUDED
#define __MODEM_SSI_H_INCLUDED

#define MAX_CHANNELS_MONITORED 4

#define CHANNEL_COMMON_CONTROL 0
#define CHANNEL_COMMON_DATA    3
#define CHANNEL_AUDIO_CONTROL  1
#define CHANNEL_AUDIO_DATA     2

struct t_data_buf {
	dma_addr_t phys_address;
	u32 *log_address;
};

enum t_l1_tx_state {
	L1_TX_FREE_STATE,
	L1_TX_ST_SENDING_STATE,
	L1_TX_ST_DONE_STATE,
	L1_TX_BUSY_STATE
};

enum t_l1_rx_state {
	L1_RX_FREE_STATE,
	L1_RX_WAITING_ST_STATE,
	L1_RX_ST_RECEIVED_STATE,
	L1_RX_BUSY_STATE
};

struct t_l1_message {
	u32 *st;
	struct t_data_buf rx_st;
	u8	mapid;
	u32 length;
	struct t_data_buf msgptr;
};

struct t_l1_desc {
	struct t_l1_message tx_msg;
	struct t_l1_message rx_msg;
	enum t_l1_tx_state tx_state;
	enum t_l1_rx_state rx_state;
	struct semaphore tx_mutex;
	struct semaphore rx_mutex;
	struct hsi_device *tx_dev;
	struct hsi_device *rx_dev;
	struct completion tx_complete;
};

void dlp_ssi_boot_state(u8 event);
void dlp_ssi_boot_state_audio(u8 event);
u8 dlp_ssi_get_boot_state(void);

#endif /*__MODEM_SSI_H_INCLUDED*/

