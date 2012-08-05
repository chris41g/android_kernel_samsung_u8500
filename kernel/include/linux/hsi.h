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

#include <linux/ioctl.h>

#include <plat/ste_dma40.h>

#ifndef __HSI_DRIVER_IF_H__
#define __HSI_DRIVER_IF_H__

#define HSIT_CTRLR_ID 0x0
#define HSIR_CTRLR_ID 0x1

#define HSI_IOC_MAGIC			'h'

/** HSI ioctls */
#define HSI_IOC_SET_WATERMARK		_IOW(HSI_IOC_MAGIC, 1, __u8)
#define HSI_IOC_GET_WATERMARK		_IOR(HSI_IOC_MAGIC, 1, __u8)

/* Read / Write HSI watermark */
#define HSI_IOC_SET_MODE		_IOW(HSI_IOC_MAGIC, 2, __u8)
#define HSI_IOC_GET_MODE		_IOR(HSI_IOC_MAGIC, 2, __u8)

/* Read / Write HSI device frame length (1..N) */
#define HSI_IOC_SET_FRAMELEN		_IOW(HSI_IOC_MAGIC, 3, __u8)
#define HSI_IOC_GET_FRAMELEN		_IOR(HSI_IOC_MAGIC, 3, __u8)

/* HSI break and threshold settings */
#define HSI_IOC_SEND_BREAK		_IOW(HSI_IOC_MAGIC, 4, __u32)
#define HSI_IOC_SET_THRESHOLD		_IOW(HSI_IOC_MAGIC, 5, __u32)

/* HSI channel operating modes */
#define         HSI_POLLING_MODE        0x00
#define         HSI_INTERRUPT_MODE      0x01
#define         HSI_DMA_MODE            0x02

#define HSI_TYPE_SIZE 32

#ifdef __KERNEL__
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/clk.h>

/** Error events enumeration
 * Defines the type of errors that are encountered
 * by a channel. This enumeration is used by the
 * exception handler of each channel to take appropriate
 * action.
**/
enum {
	HSI_EXCEP_TIMEOUT,
	HSI_EXCEP_PIPEBUF_OVERRUN,
	HSI_EXCEP_BREAK_DETECTED,
	HSI_EXCEP_PARITY_ERROR,
	HSI_RXCHANNELS_OVERRUN,
};

/** struct hsi_device - HSI slave device
 * @cid - Controller of the device
 * @chid - Channel on which it transceives
 * @curr_mode - DMA, Interrupt or Polling
 * @ch - Channel information of the device
 * @ctrlr - Controller data of the device
 * dev - Device represention on sys interface
 * A @hsi_device is used to interchange data between an HSI slave
 * (usually a discrete chip) and CPU memory.
 *
**/
struct hsi_device {
	short cid;
	short chid;
	int curr_mode;
	struct hsi_channel *ch;
	struct hsi_controller *ctrlr;
	struct device dev;
};

#define to_hsi_device(hdev)	container_of(hdev, struct hsi_device, dev)

#define hsi_get_platform_data(dev)	(dev->platform_data)

/**
 * struct hsi_device_driver - Host side protocol driver
 * @ctrl_mask - Controller(s) with which the chip can communicate
 * @ch_mask - Channels on which the data can be recevied/trasnmitted
 * @execp_mask - Mask of exceptions that can be handled
 * @excep_event - Exception handler for the driver
 *
**/
struct hsi_device_driver {
	u32 ctrl_mask;
	u32 ch_mask;
	u32 excep_mask;
	void (*excep_event) (unsigned int c_id, unsigned int event, void *arg);
	int (*probe) (struct hsi_device *dev);
	int (*remove) (struct hsi_device *dev);
	int (*suspend) (struct hsi_device *dev, pm_message_t state);
	int (*resume) (struct hsi_device *dev);
	struct device_driver driver;
};

/**
 * struct hsi_data - Payload to be trnasmitted to the controller.
 * @data - Pointer to virtual memory containing the data
 * @dma_addr - DMA address of the data source
 * @datawidth - Framelength of the payload (8, 16, 32 bits)
 * @count - Size in (framelength) words of the payload
 * @is_dma - If the address is is dma-able/cacheable in which cache
 *           synchronisation is needed, before dma transfer
**/
struct hsi_data {
	void *data;
	dma_addr_t dma_addr;
	int datawidth;
	int count;
	int is_dma;
};

/**
 * struct hsi_board_info - Platform dependent client information
 * 			   structure.
 * @type - Name of the device on HSI bus
 * @flags - Flags to control operation
 * @controller_id - Controller on which it resides
 * @chan_num - Channel number on which it trasnceives
 * @platform_data - Any platform data to be passed to its driver
 * @mode - Reception/Transmssion device
 * @list - To link on devices list of the HSI bus
**/
struct hsi_board_info {
	char type[HSI_TYPE_SIZE];
	unsigned short flags;
	unsigned short controller_id;
	unsigned short chan_num;
	void *platform_data;
	int irq;
	int mode;
	struct list_head list;
};

#define HSI_BOARD_INFO(name, flags, cid, chid) \
	(.type = (name), .flags = (flags), \
	.controller_id = (cid), .chan_num = (chid))

/*
 * Callbacks use by the HSI upper layer (HSI protocol) to receive data
 * from the port channels.
 */
struct hsi_channel;

/** struct hsi_channel_ops - Callbacks for channel events
 * @write_done - Write completion notification
 * @read_done - Read completion notification
 * @read - Read routine for interrupt transfers
 * @write - Write routine for interrupt transfers
 *
 * For HSI protocol drivers the callbacks could be used
 * to wake up processes waiting for channel events. This
 * means that read/write operations can block
**/
struct hsi_channel_ops {
	void (*write) (struct hsi_channel *ch);
	void (*read) (struct hsi_channel *ch);
	void (*write_done) (struct hsi_device *dev);
	void (*read_done) (struct hsi_device *dev);
};

struct dma_chan;
struct stedma40_chan_cfg;

/** struct hsi_channel - Channel information
 * @id - Controller id this channel belongs to
 * @read_data - Payload structure contaning buffer
 * 		read pointer and length information
 * @write_data - Payload structure contaning buffer
 * 		write pointer and length information
 * @ctrlr - Pointer to repective controller. Can be
 * 	    used to access controller algorithms
 * @flags - Channel opened/busy
 * @channel_number - Channel id within the controller
 * @watermark - Programmed watermark level to trigger
 * 		transfer
 * @num_xfer_perintr - Number of transfers to be done
 * @datawidth - Framelength of the payload
 * @dma_pipe_id - DMA pipe id for performing DMA
 * 		  transfers on.
 * @hsi_dma_info - Structure containing source and
 * 		    destination information
 * @span - Span of the channel
 * @n_bytes - Number of bytes to transfer
 * @hsi_ch_lock - Channel lock
 * @dev - Device presenting the channel which attaches
 * 	 to the controller as child
 * @hsi_dma_tasklet - Tasklet to schedule on receving transfer
 * 		      completion
 * @priv - Private information for the channel
**/

struct hsi_channel {
	int id;
	struct hsi_channel_ops ops;
	struct hsi_data read_data;
	struct hsi_data write_data;
	struct hsi_controller *ctrlr;
	u8 flags;
	u8 channel_number;
	u8 watermark;
	u8 num_xfer_perintr;
	u8 datawidth;
	int dma_pipe_id;
	struct stedma40_chan_cfg *hsi_dma_info;
	u8 span;
	u8 n_bytes;
	spinlock_t hsi_ch_lock;
	struct dma_chan *dma_chan;
	struct hsi_device *dev;
	struct tasklet_struct hsi_dma_tasklet;
	void *priv;
};
#define HSI_MAX_CHANNELS 8
struct hsi_backup_regs {
	u32 framelen[HSI_MAX_CHANNELS];
	u32 watermark[HSI_MAX_CHANNELS];
	u32 rx_threshold;
};
/** struct hsi_controller -  Struct definition to hold information
 * 			     about hsi controller
 * @dev_type - Transmitter / receiver interface
 * @flags - Initialized or exiting
 * @max_ch - Maximum channels on this controller
 * @mode - Interrupt / DMA / Polling
 * @irq1/2 - IRQs for this controller
 * @irqexcep - Exception interrupt
 * @irq_choverrun - Array holding channel overrun information
 * @name - Id for this controller
 * @regbase - Virtual register address for this controller
 * @lock - Spinlock for the controller
 * @hsi_tx_tasklet - TX Tasklet to schedule on trasnfer completion
 * @hsi_rx_tasklet - RX Tasklet to schedule on trasnfer completion
 * @hsi_tx_channels - Transmitter channels information for transmitter
 * @hsi_rx_channels - Receiver channels information for receiver
 * @algo - Alorithm routines for the controller
 * @dev - Device information on which registers on the bus
**/
struct hsi_controller {
	u8 dev_type; /** tx or rx */
	u8 flags;
	u8 max_ch;
	u8 mode;
	u8 irq1;
	u8 irq2;
	u8 irqexcep;
	u8 irq_choverrun[HSI_MAX_CHANNELS];
	char name[20];
	void __iomem *regbase;
	struct clk *clk;
	spinlock_t lock;
	u32 tx_wmark_intrstatus;
	struct tasklet_struct hsi_tx_tasklet;
	struct tasklet_struct hsi_rx_tasklet;
	struct tasklet_struct hsi_rxexcep_tasklet;
	struct hsi_channel hsi_tx_channels[HSI_MAX_CHANNELS];
	struct hsi_channel hsi_rx_channels[HSI_MAX_CHANNELS];
	struct hsi_algorithm *algo;
	struct device *dev;
	struct hsi_backup_regs backup_regs;
};

/** struct hsi_ctrlr_excep - Exception information
 * @ctrlr - Controller on which the exception occured
 * @event - Type of event
 * @priv - Private data for execption handler
**/
struct hsi_ctrlr_excep {
	struct hsi_controller *ctrlr;
	unsigned int event;
	void *priv;
};

int hsi_bus_init(void);
void hsi_bus_exit(void);
/* End HSI Bus */

#define to_hsi_device_driver(drv) container_of(drv, \
						struct hsi_device_driver, \
						driver)

#ifdef CONFIG_HSI
extern int hsi_register_board_info(struct hsi_board_info const *info, int n);
#else
/* board init code may ignore whether HSI is configured or not */
static inline int
hsi_register_board_info(struct hsi_board_info const *info, int n)
{
	return 0;
}
#endif

struct hsi_algorithm {
	int (*open) (struct hsi_device *dev);
	int (*write) (struct hsi_device *dev, struct hsi_data *data);
	int (*read) (struct hsi_device *dev, struct hsi_data *data);
	int (*ioctl) (struct hsi_device *dev, unsigned int command, void *arg);
	void (*close) (struct hsi_device *dev);
	void (*set_cb) (struct hsi_device *dev,
			void (*r_cb) (struct hsi_device *dev),
			void (*w_cb) (struct hsi_device *dev));
	void (*cancel_read) (struct hsi_device *dev);
	void (*cancel_write) (struct hsi_device *dev);
	int (*exception_handler) (struct device_driver *driver, void *event);

};

int hsi_excep_handler(struct hsi_controller *hsi, unsigned int event,
		      void *arg);

extern int register_hsi_driver(struct hsi_device_driver *driver);
extern void unregister_hsi_driver(struct hsi_device_driver *driver);
extern int hsi_add_controller(struct hsi_controller *hsi_ctrlr);
extern void hsi_remove_controller(struct hsi_controller *hsi_ctrlr);
extern int hsi_open(struct hsi_device *dev);
extern void hsi_close(struct hsi_device *dev);
extern int hsi_write(struct hsi_device *dev, struct hsi_data *data);
extern int hsi_read(struct hsi_device *dev, struct hsi_data *data);

extern void hsi_write_cancel(struct hsi_device *dev);
extern void hsi_read_cancel(struct hsi_device *dev);
extern void hsi_set_callback(struct hsi_device *dev,
			     void (*r_cb) (struct hsi_device *dev)
			     , void (*w_cb) (struct hsi_device *dev));
extern int hsi_ioctl(struct hsi_device *dev, unsigned int command, void *arg);
extern int hsi_exception(struct hsi_controller *hsi_ctrlr, unsigned int event,
			 void *arg);

#endif /* __KERNEL__ */

#endif /* __HSI_H__ */
