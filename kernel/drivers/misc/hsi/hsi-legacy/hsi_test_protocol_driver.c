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

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sysdev.h>
#include <linux/input.h>
#include <linux/hsi-legacy.h>
#include <linux/hsi_test_prot.h>

#include <asm/fcntl.h>

#define HSI_TESTPROT_DRIVER_NAME "HSI_TESTPROT_DRV"
//#define MAX_CHANNELS_MONITORED 4


static struct hsi_device *lp_txdev[MAX_CHANNELS_MONITORED]={NULL,
						NULL,NULL,NULL};  /** store local pointers to tx channels monitored by this driver */
static struct hsi_device *lp_rxdev[MAX_CHANNELS_MONITORED]={NULL,
						NULL,NULL,NULL};  /** store local pointers to rx channels monitored by this driver */

/** shared between protocol driver and its apps */
struct callback_data cbdata[MAX_CHANNELS_MONITORED*2];

EXPORT_SYMBOL(cbdata);

int hsi_testprot_drv_open(unsigned char flags)
{
	int ch = -ENODEV;
	int i=0;

	if ((flags & O_ACCMODE) == O_WRONLY) {
		for (i=0;((i<MAX_CHANNELS_MONITORED) && (lp_txdev[i] != NULL));i++)
		{
			ch = hsi_open(lp_txdev[i]);
			if (!(ch)) {
				ch = i;
				break; /** found a good channel */
			}
		}
	}
	else if ((flags & O_ACCMODE) == O_RDONLY) {
		for (i=0;((i<MAX_CHANNELS_MONITORED) && (lp_rxdev[i] != NULL));i++)
		{
			ch = hsi_open(lp_rxdev[i]);
			if (!(ch)) {
				ch = i+MAX_CHANNELS_MONITORED;  /** to ensure tx and rx channel codes are unique */
				break;
			}
		}
	}

	printk(KERN_INFO "hsi_testprot_drv_open: Returned channel number %d \n"
			, ch);
	return ch;
}
EXPORT_SYMBOL(hsi_testprot_drv_open);

/** data - pointer to data buffer
 *  datawidth - in bits i.e 16 - 16bits, 32 - 32bits
 *  count - total number of data bytes to receive (should always be in multiple of words)
 *
 *  Please refer test applications on how to properly pack the buffer to receive data. Also DMA transfers should
 *  require dma coherent buffers (pls refer test app on how to do this)
 */
int hsi_testprot_drv_read(unsigned int ch,void* data,unsigned int datawidth,unsigned int count)
{
	int err=-ENODEV;
	int ch_code = ch - MAX_CHANNELS_MONITORED;

	if (ch_code >= MAX_CHANNELS_MONITORED) return err;

	err = hsi_read(lp_rxdev[ch_code],data,datawidth,count);

	return err;
}
EXPORT_SYMBOL(hsi_testprot_drv_read);

/** data - pointer to data
 *  datawidth - in bits i.e 16 - 16bits, 32 - 32bits
 *  count - total number of data bytes to send (should always be in multiple of words)
 *
 *  Please refer test applications on how to pack the data to be sent. Also DMA transfers should
 *  require dma coherent buffers (pls refer test app on how to do this)
 */

int hsi_testprot_drv_write(unsigned int ch,void* data,unsigned int datawidth,unsigned int count)
{
	int err=-ENODEV;

	if (ch >= MAX_CHANNELS_MONITORED) return err;

	err = hsi_write(lp_txdev[ch],data,datawidth,count);

	return err;
}
EXPORT_SYMBOL(hsi_testprot_drv_write);



int hsi_testprot_drv_write_cancel(unsigned int ch)
{
	int err=-ENODEV;

	if (ch >= MAX_CHANNELS_MONITORED) return err;

	hsi_write_cancel(lp_txdev[ch]);

	return 0;
}
EXPORT_SYMBOL(hsi_testprot_drv_write_cancel);

int hsi_testprot_drv_read_cancel(unsigned int ch)
{
	int err=-ENODEV;
	int ch_code = ch - MAX_CHANNELS_MONITORED;

	if (ch_code >= MAX_CHANNELS_MONITORED) return err;

	hsi_read_cancel(lp_rxdev[ch_code]);

	return 0;
}
EXPORT_SYMBOL(hsi_testprot_drv_read_cancel);

int  hsi_testprot_drv_close(unsigned int ch)
{
	int err=-ENODEV;
	int ch_code = ch - MAX_CHANNELS_MONITORED;

	if (ch >= MAX_CHANNELS_MONITORED) {
		if (!(lp_rxdev[ch_code])) return err;
		hsi_close(lp_rxdev[ch_code]);
	} else {
		if (!(lp_txdev[ch])) return err;
		hsi_close(lp_txdev[ch]);
	}

	return 0;
}
EXPORT_SYMBOL(hsi_testprot_drv_close);

/** NOTE: before sending/receiving data the user should set an appropriate
 * watermark through this ioctl. The watermark determines how often the low
 * -level HSI driver is interrupted to send/receive data on the link. The user
 *  will be notified through callback only when all the data has been sent or
 *  received
 */

int hsi_testprot_drv_ioctl(unsigned int ch,unsigned int command,void *arg)
{
	int err=-ENODEV;

	if (ch >= MAX_CHANNELS_MONITORED) {
		if (!(lp_rxdev[ch-MAX_CHANNELS_MONITORED])) return err;
		err = hsi_ioctl(lp_rxdev[ch-MAX_CHANNELS_MONITORED],command,arg);
	} else {
		if (!(lp_txdev[ch])) return err;
		err = hsi_ioctl(lp_txdev[ch],command,arg);
	}

	return err;
}
EXPORT_SYMBOL(hsi_testprot_drv_ioctl);

/** NOTE: the low-level HSI driver executes the read/write callbacks
 * only when it has received or sent all the data it had been requested
 * to receive/send through the read/write calls
 */

void read_callbk(struct hsi_device *dev)
{
	int ch=0,i=0;

	for (i=0;i<MAX_CHANNELS_MONITORED;i++) {
		if (lp_rxdev[i]==dev) {
			ch = i+MAX_CHANNELS_MONITORED;
			cbdata[ch].read_done = 0x1;
			printk(KERN_INFO "read callback executed for channel %d\n"
					, dev->chid);
			break;
		}
	}
}

void write_callbk(struct hsi_device *dev)
{
	int ch=0,i=0;

	for (i=0;i<MAX_CHANNELS_MONITORED;i++) {
		if (lp_txdev[i]==dev) {
			ch = i;
			cbdata[ch].write_done = 0x1;
			printk(KERN_INFO "write callback executed for channel %d\n"
					, dev->chid);
			break;
		}
	}
}

void hsi_testprot_excep_handler(u32 ctrlr_id,u32 event,void *arg)
{
	u8 chid;
	/** exceptions are received only on HSIR */
	if (ctrlr_id != HSIR_CTRLR_ID) {
		printk(KERN_INFO "HSI test protocol driver:exception on wrong controller\n");
		return;
	}

	/** NOTE: an actual protocol driver is expected to do proper
	 * exception handling. This could include flushing its buffers,
	 * cancelling read, informing the TX through a message to
	 * resend/start sending data
	 *
	 * Apart from informing the protocol driver the lowlevel HSI driver
	 * does not perform any operations on receiving an exception,
	 * except when it receives :
	 * 		HSI_EXCEP_PIPEBUF_OVERRUN, where it flushes HSIR pipeline buffer
	 * 		HSI_RXCHANNELS_OVERRUN , where it flushes the HSIR channel buffer
	 */
	switch (event) {
		case HSI_EXCEP_TIMEOUT:
			printk(KERN_INFO "HSI test protocol driver:TIMEOUT received\n");
			break;
		case HSI_EXCEP_PIPEBUF_OVERRUN:
			/** HSI has flushed pipeline buffer...protocol driver MUST take appropriate action */
			printk(KERN_INFO "HSI test protocol driver:PIPEBUF OVERRUN received\n");
			break;
		case HSI_EXCEP_BREAK_DETECTED:
			printk(KERN_INFO "HSI test protocol driver:BREAK DETECT received\n");
			break;
		case HSI_EXCEP_PARITY_ERROR:
			printk(KERN_INFO "HSI test protocol driver: PARITY ERROR received\n");
			break;
		case HSI_RXCHANNELS_OVERRUN:
			/** HSI has flushed corresponding channel buffer...protocol driver MUST take appropriate action */
			chid=*(u8 *)arg;
			printk(KERN_INFO "HSI test protocol driver: CHANNEL %d OVERRUN \n"
					, chid);
			break;
	}

}

static int __init hsi_testprot_drv_probe(struct hsi_device *dev)
{
	static int i=0;
	static int j=0;

	if (!dev) {
		printk(KERN_INFO "HSI device not populated \n");
		return -ENODEV;
	}

	/** set callbacks for channel read write completion notification */
	if (dev->ctrlrid == 0x0) {
		lp_txdev[i++] = dev;
		hsi_dev_set_cb(dev,NULL,&write_callbk);
	}
	else {
		lp_rxdev[j++] = dev;
		hsi_dev_set_cb(dev,&read_callbk,NULL);
	}

	printk(KERN_INFO "HSI test protocol driver : device %s probed \n"
			, dev->device.bus_id);

	return 0;
}

static int __exit hsi_testprot_drv_remove(struct hsi_device *dev)
{
	hsi_dev_set_cb(dev,NULL,NULL);
	return 0;
}

static struct hsi_device_driver hsi_testprot_driver = {
	/** HSI controller ids to monitor: 0 - transmit and 1 - receive  */
	.ctrl_mask = 0x3,
	/** HSI channels ids to monitor: 0xF - ch0,ch1,ch2,ch3 */
	.ch_mask = 0xF,
	/** HSI exception mask : set exceptions you want to receive
	 *  bit 0: link timeout, bit 1: pipebuf overrun, bit2: break detection
	 *  bit 3: link parity error, bit 4: rx channels overrun
	 */
	.excep_mask = 0x1F,
	.excep_event = hsi_testprot_excep_handler,
	.probe = hsi_testprot_drv_probe,
	.remove = __exit_p(hsi_testprot_drv_remove),
	.driver = {
		   .name = HSI_TESTPROT_DRIVER_NAME,
		  },
};

static int __init hsi_testprot_init(void)
{
	int res;

	res = register_hsi_driver(&hsi_testprot_driver);
	if (res < 0) {
		printk(KERN_INFO "HSI test protocol driver registration failed\n");
		return res;
	}

	memset(cbdata,0x0,sizeof(cbdata));

	return 0;
}

static void __exit hsi_testprot_exit(void)
{
	unregister_hsi_driver(&hsi_testprot_driver);
}

module_init(hsi_testprot_init);
module_exit(hsi_testprot_exit);

MODULE_AUTHOR("STMicroelectronics");
MODULE_DESCRIPTION("HSI test protocol driver");
MODULE_LICENSE("GPL");
