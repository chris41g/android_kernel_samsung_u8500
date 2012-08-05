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


#ifndef __HSI_DRIVER_H__
#define __HSI_DRIVER_H__

#include <linux/module.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>

#include <linux/hsi-legacy.h>
#include <mach/dma.h>

#define HSI_DRIVER_AUTHOR	"STMicroelectronics"
#define HSI_DRIVER_DESC		"High-Speed Synchronous Serial Interface Driver"

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
 * Callbacks use by the HSI upper layer (HSI protocol) to receive data
 * from the port channels.
 */
struct hsi_channel_ops {
	void (*write_done) (struct hsi_device *dev);
	void (*read_done) (struct hsi_device *dev);
	void (*read)(struct hsi_channel *ch);
	void (*write)(struct hsi_channel *ch);
};

struct hsi_data {
	/* Pointer to the data to be sent/received */
	void  *addr;
	/*
	 * Number of bytes to be sent or space reserved for data to be
	 * received.
	 */
	unsigned int size;
};

#if 0
struct hsi_tx_channel {
	struct hsi_channel_ops ops;
	struct hsi_data write_data;
	struct hsi_dev *ctrlr;
	u8 flags;
	u8 channel_number;
	spinlock_t hsi_ch_lock;
	struct hsi_tx_device *dev;
	void *priv;
};

struct hsi_rx_channel {
	struct hsi_channel_ops ops;
	struct hsi_data read_data;
	struct hsi_data write_data;
	struct hsi_dev *ctrlr;
	u8 flags;
	u8 channel_number;
	spinlock_t hsi_ch_lock;
	struct hsi_rx_device *dev;
	void *priv;
};
#endif

struct dma_chan;
struct stedma40_chan_cfg;

struct hsi_channel {
	int id;
	struct hsi_channel_ops ops;
	struct hsi_data read_data;
	struct hsi_data write_data;
	struct hsi_dev *ctrlr;
	u8 flags;
	u8 channel_number;
	u8 watermark;
	u8 num_xfer_perintr;
	struct dma_chan *dma_chan;
	struct stedma40_chan_cfg *hsi_dma_info;
	u8 span;
	u8 n_bytes;
	spinlock_t hsi_ch_lock;
	struct hsi_device *dev;
	struct tasklet_struct hsi_dma_tasklet;
	void *priv;
};


/*
 * Struct definition to hold information about hsi controller
 */
struct hsi_dev {
	u8 dev_type; /** tx or rx */
	u8 flags;
	u8 max_ch;
	u8 mode;
	u8 irq1;
	u8 irq2;
	u8 irqexcep;
	u8 irq_choverrun[HSI_MAX_CHANNELS];
	void __iomem *regbase;
	struct clk *clk;
	spinlock_t lock;
	u32 tx_wmark_intrstatus;
	struct tasklet_struct hsi_tx_tasklet;
	struct tasklet_struct hsi_rx_tasklet;
	struct tasklet_struct hsi_rxexcep_tasklet;
	struct hsi_channel hsi_tx_channels[HSI_MAX_CHANNELS];
	struct hsi_channel hsi_rx_channels[HSI_MAX_CHANNELS];
};

struct hsi_ctrlr_excep{
	struct hsi_dev *ctrlr;
	unsigned int event;
	void *priv;
};

int hsi_bus_init(void);
void hsi_bus_exit(void);
/* End HSI Bus */

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
void hsi_dma_read_eot_handler(void * data);
void hsi_dma_write_eot_handler(void * data);

void hsi_u32_reader(struct hsi_channel *ch);


/*DMA Functions*/
int hsi_read_dma_mode(struct hsi_channel *chnl);
int hsi_write_dma_mode(struct hsi_channel *chnl);
void hsi_cancel_write_dma_mode(struct hsi_channel *chnl);
void hsi_cancel_read_dma_mode(struct hsi_channel *chnl);



#endif
