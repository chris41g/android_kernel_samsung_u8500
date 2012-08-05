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

#ifndef __HSI_DRIVER_IF_H__
#define __HSI_DRIVER_IF_H__

#include <linux/device.h>
#include <mach/dma.h>
#include <asm/irq.h>
#include <linux/gpio.h>
#include <plat/ste_dma40.h>

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


#define HSI_MAX_CHANNELS 8
#define HSI_MAX_FRAMELEN 32

#define HSIT_CTRLR_ID 0x0
#define HSIR_CTRLR_ID 0x1


extern struct bus_type hsi_bus_type;

enum {
	HSI_EXCEP_TIMEOUT,
	HSI_EXCEP_PIPEBUF_OVERRUN,
	HSI_EXCEP_BREAK_DETECTED,
	HSI_EXCEP_PARITY_ERROR,
	HSI_RXCHANNELS_OVERRUN,
};


/** HSI ioctls */
enum {
	HSI_IOCTL_SET_WATERMARK,
	HSI_IOCTL_GET_WATERMARK,
	HSI_IOCTL_SET_CURRMODE,
	HSI_IOCTL_GET_CURRMODE,
	HSI_IOCTL_SEND_BREAK,
	HSI_IOCTL_SET_FRAMELEN,
	HSI_IOCTL_GET_FRAMELEN,
	HSI_IOCTL_SET_THRESHOLD,
};

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
	struct base_span ch_base_span[HSI_MAX_CHANNELS];
	struct stedma40_chan_cfg hsi_dma_info[HSI_MAX_CHANNELS];
	struct hsi_dev *hsi_ctrlr;
};

enum hsi_mode{
       HSI_POLLING_MODE=0x0,
       HSI_INTERRUPT_MODE,
       HSI_DMA_MODE,
};


struct hsi_device {
	int ctrlrid;
	enum hsi_mode curr_mode;
	unsigned int chid;
	char modalias[BUS_ID_SIZE];
	struct hsi_channel *ch;
	struct device device;
};


#define to_hsi_device(dev)	container_of(dev, struct hsi_device, device)

struct hsi_device_driver {
	u32		ctrl_mask;
	u32		ch_mask;
	u32		excep_mask;
	void 			(*excep_event) (unsigned int c_id,
						unsigned int event, void *arg);
	int			(*probe)(struct hsi_device *dev);
	int			(*remove)(struct hsi_device *dev);
	int			(*suspend)(struct hsi_device *dev,
						pm_message_t mesg);
	int			(*resume)(struct hsi_device *dev);
	struct device_driver 	driver;
};


#define to_hsi_device_driver(drv) container_of(drv, \
						struct hsi_device_driver, \
						driver)

int hsi_excep_handler(struct hsi_dev* hsi_ctrlr, unsigned int event, void *arg);

int register_hsi_driver(struct hsi_device_driver *driver);
void unregister_hsi_driver(struct hsi_device_driver *driver);
int hsi_open(struct hsi_device *dev);
int hsi_write(struct hsi_device *dev,void *data,  u32 datawidth,unsigned int count);
void hsi_write_cancel(struct hsi_device *dev);
int hsi_read(struct hsi_device *dev, void *data,  u32 datawidth,unsigned int count);
void hsi_read_cancel(struct hsi_device *dev);
int hsi_ioctl(struct hsi_device *dev, unsigned int command, void *arg);
void hsi_close(struct hsi_device *dev);
void hsi_dev_set_cb(struct hsi_device *dev, void (*r_cb)(struct hsi_device *dev)
					, void (*w_cb)(struct hsi_device *dev));
#endif
