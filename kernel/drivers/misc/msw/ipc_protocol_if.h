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

#ifndef __MODEM_HSI_IF_H
#define __MODEM_HSI_IF_H

enum t_mapping_identifier {
	ISI_MESSAGING,
	RPC_MESSAGING,
	AUDIO_MESSAGING,
	SEC_MESSAGING,
	MAX_MAP_ID
};

int hsi_msg_send(void *p_data, u8 l2_msg_header, u32 length);
int dlp_protocol_init(void);
int dlp_protocol_deinit(void);
int dlp_protocol_start(u8 l2_header);
int dlp_protocol_stop(u8 l2_header);
int l3alloc(u32 n_bytes, u8 l2_msg_header, void *addr);
void l3receive(void *msg_ptr, u32 length, u8 l2_msg_header);

#endif /*__MODEM_HSI_IF_H*/
