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

#ifndef __HSI_STM_H__
#define __HSI_STM_H__

#include <plat/ste_dma40.h>

#define HSI_DRIVER_AUTHOR	"STMicroelectronics"
#define HSI_DRIVER_DESC		"High-Speed Synchronous Serial Interface Driver"

#define DRIVER_NAME            "DRIVER HSI"
/* enables/disables debug msgs */
#define DRIVER_DEBUG            0
/* msg header represents this module */
#define DRIVER_DEBUG_PFX        DRIVER_NAME
#define DRIVER_DBG              KERN_ERR

#define HSI_DRIVER_NAME		"hsi_driver"

#define HSI_DEVICE_NAME		"hsi_device"
#define HSI_TX_PREFIX "hsi_tx:"
#define HSI_RX_PREFIX "hsi_rx:"
#define HSI_PREFIX "hsi:"
/** HSIT register offsets */
#define HSI_TX_ID 0x0
#define HSI_TX_MODE 0x4
#define HSI_TX_STATE 0x8
#define HSI_TX_IOSTATE 0xC
#define HSI_TX_BUFSTATE 0x10
#define HSI_TX_DIVISOR 0x14
#define HSI_TX_PARITY 0x18
#define HSI_TX_BREAK 0x1C
#define HSI_TX_CHANNELS 0x20
#define HSI_TX_FLUSHBITS 0x24
#define HSI_TX_PRIORITY 0x28
#define HSI_TX_BURSTLEN 0x2C
#define HSI_TX_PREAMBLE 0x30
#define HSI_TX_DATASWAP 0x34
#define HSI_TX_FRAMELENX 0x80
#define HSI_TX_BUFFERX 0xC0
#define HSI_TX_BASEX 0x100
#define HSI_TX_SPANX 0x140
#define HSI_TX_GAUGEX 0x180
#define HSI_TX_WATERMARKX 0x1C0
#define HSI_TX_DMAEN 0x200
#define HSI_TX_WATERMARKIS 0x204
#define HSI_TX_WATERMARKIM 0x208
#define HSI_TX_WATERMARKIC 0x20C
#define HSI_TX_WATERMARKID 0x210
#define HSI_TX_PERIPHID0 0xFE0
#define HSI_TX_PERIPHID1 0xFE4
#define HSI_TX_PERIPHID2 0xFE8
#define HSI_TX_PERIPHID3 0xFEC


/** HSIT register offsets */
#define HSI_RX_ID 0x0
#define HSI_RX_MODE 0x4
#define HSI_RX_STATE 0x8
#define HSI_RX_BUFSTATE 0xC
#define HSI_RX_THRESHOLD 0x10
#define HSI_RX_PARITY 0x14
#define HSI_RX_DETECTOR 0x18
#define HSI_RX_EXCEP 0x1C
#define HSI_RX_ACK 0x20
#define HSI_RX_CHANNELS 0x24
#define HSI_RX_REALTIME 0x28
#define HSI_RX_OVERRUN 0x2C
#define HSI_RX_OVERRUNACK 0x30
#define HSI_RX_PREAMBLE 0x34
#define HSI_RX_PIPEGAUGE 0x38
#define HSI_RX_TIMEOUT 0x3C
#define HSI_RX_BUFFERX 0x80
#define HSI_RX_FRAMELENX 0xC0
#define HSI_RX_BASEX 0x100
#define HSI_RX_SPANX 0x140
#define HSI_RX_GAUGEX 0x180
#define HSI_RX_WATERMARKX 0x1C0
#define HSI_RX_DMAEN 0x200
#define HSI_RX_WATERMARKIS 0x204
#define HSI_RX_WATERMARKIM 0x208
#define HSI_RX_WATERMARKIC 0x20C
#define HSI_RX_WATERMARKID 0x210
#define HSI_RX_OVERRUNMIS 0x214
#define HSI_RX_OVERRUNIM 0x218
#define HSI_RX_EXCEPMIS 0x21C
#define HSI_RX_EXCEPIM 0x220
#define HSI_RX_PERIPHID0 0xFE0
#define HSI_RX_PERIPHID1 0xFE4
#define HSI_RX_PERIPHID2 0xFE8
#define HSI_RX_PERIPHID3 0xFEC

/*
 * Masks used to enable or disable the reception of certain hardware events
 * for the hsi_device_drivers
 */
#define HSI_EVENT_CLEAR			0x00
#define HSI_EVENT_MASK			0xFF
#define HSI_EVENT_BREAK_DETECTED_MASK	0x01
#define HSI_EVENT_ERROR_MASK		0x02

#define HSI_CH_OPEN 0x1

#define ANY_HSI_CONTROLLER	-1
#define ANY_CHANNEL		-1
#define CHANNEL(channel)	(1<<channel)


#define HSI_MAX_FRAMELEN 32
#define PLAT_HSI_MAX_CHANNELS 8

struct base_span {
	u8 base;
	u8 span;
};

struct hsi_plat_data {
	u8 dev_type;
	u8 mode;
	u8 parity;
	u8 priority;
	u8 channels;
	u8 threshold;
	u8 flushbits;
	u8 dataswap;
	u8 realtime;
	u8 detector;
	u8 framelen;
	u8 watermark;
	u8 currmode;
	gpio_alt_function gpio_alt_func;
	u32 divisor;
	u32 burstlen;
	u32 preamble;
	u32 timeout;
	struct base_span ch_base_span[PLAT_HSI_MAX_CHANNELS];
	struct stedma40_chan_cfg hsi_dma_info[PLAT_HSI_MAX_CHANNELS];
	struct hsi_controller *controller;
};

struct hsi_algorithm;
extern struct hsi_algorithm hsi_algo;
int hsi_read_interrupt_mode(struct hsi_channel *chnl);
int hsi_write_interrupt_mode(struct hsi_channel *chnl);
void hsi_cancel_write_interrupt_mode(struct hsi_channel *chnl);
void hsi_cancel_read_interrupt_mode(struct hsi_channel *chnl);

irqreturn_t hsi_tx_irq_handler(int irq, void *ctrlr);
irqreturn_t hsi_rx_irq_handler(int irq, void *ctrlr);
irqreturn_t hsi_rxexcep_irq_handler(int irq, void *ctrlr);

void do_hsi_tx_tasklet(unsigned long ctrlr);
void do_hsi_rx_tasklet(unsigned long ctrlr);
void do_hsi_tx_dma_tasklet(unsigned long ctrlr);
void do_hsi_rx_dma_tasklet(unsigned long ctrlr);
void do_hsi_rxexcep_tasklet(unsigned long ctrlr);
void hsi_u8_writer(struct hsi_channel *ch);
void hsi_u8_reader(struct hsi_channel *ch);
void hsi_u16_writer(struct hsi_channel *ch);
void hsi_u16_reader(struct hsi_channel *ch);
void hsi_u32_writer(struct hsi_channel *ch);
void hsi_u32_reader(struct hsi_channel *ch);
void hsi_dma_read_eot_handler(void *data);
void hsi_dma_write_eot_handler(void *data);

void hsi_cancel_write_dma_mode(struct hsi_channel *chnl);
void hsi_cancel_read_dma_mode(struct hsi_channel *chnl);
/*DMA Functions*/
int hsi_read_dma_mode(struct hsi_channel *chnl);
int hsi_write_dma_mode(struct hsi_channel *chnl);

#endif /* __HSI_STM_H__ */
