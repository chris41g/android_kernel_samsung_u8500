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
#include <linux/hsi.h>
#include <linux/hsidev.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/uaccess.h>

#include <linux/fcntl.h>

#define HSI_TESTPROT_DRIVER_NAME "HSI_LOOPBACK"

#define HSIDEV_MAJOR			153	/* assigned */
#define N_HSI_MINORS			4	/* ... up to 256 */
static unsigned long tx_minors;
static unsigned long rx_minors;
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

#define MAX_CHANNELS_MONITORED 4

/** NOTE: the low-level HSI driver executes the read/write callbacks
 * only when it has received or sent all the data it had been requested
 * to receive/send through the read/write calls
 */
static void transfer_callback(struct hsi_device *dev)
{
	struct hsidev_data *hsidev;

	list_for_each_entry(hsidev, &device_list, device_entry) {
		if (hsidev->hsi->cid == dev->cid &&
		    hsidev->hsi->chid == dev->chid) {
			hsidev->xfer_done = 1;
			wake_up_interruptible(&hsidev->wq);
			stm_dbg(DBG_ST.hsi,
				"%s callback executed for channel %d\n",
				dev->cid ? "Read" : "Write", dev->chid);
			break;
		}
	}
}

int hsidev_open(struct inode *inode, struct file *filp)
{
	struct hsidev_data *hsidev;
	int status = -ENXIO;

	lock_kernel();
	mutex_lock(&device_list_lock);

	list_for_each_entry(hsidev, &device_list, device_entry) {
		stm_dbg(DBG_ST.hsi, "%X:%X\n", hsidev->devt,
					inode->i_rdev);
		if (hsidev->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}
	if (status == 0) {
		if (hsidev->users > 0)
			return -EBUSY;
		hsidev->users++;
		filp->private_data = hsidev;
		nonseekable_open(inode, filp);
	} else {
		stm_dbg(DBG_ST.hsi, "hsidev: nothing for minor %d\n",
			iminor(inode));
		return status;
	}

	status = hsi_open(hsidev->hsi);
	if (!status) {
		if (iminor(inode) < MAX_CHANNELS_MONITORED)
			hsi_set_callback(hsidev->hsi, NULL, &transfer_callback);
		else
			hsi_set_callback(hsidev->hsi, &transfer_callback, NULL);
	}
	mutex_unlock(&device_list_lock);
	unlock_kernel();
	return status;

}

void print_data(char *caller, void *tx_buf, int num)
{
	int i = 0;
	for (i = 0; i < num / 4; i++) {
		stm_dbg(DBG_ST.hsi, "DRV %s:buf[%d]----->%x\n", caller, i,
			(*(u32 *) tx_buf));
		tx_buf += 4;
	}
}

/** buf - pointer to data buffer
 *  count - total number of data bytes to receive
 *  (should always be in multiple of words)
 *  Please refer test applications on how to properly pack
 *  the buffer to receive data. Also DMA transfers should require
 *  dma coherent buffers (pls refer test app on how to do this).
 **/
static ssize_t
hsidev_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct hsidev_data *hsidev;
	ssize_t status = 0;
	int missing = 0;
	struct hsi_data *xferdata;

	hsidev = (struct hsidev_data *)filp->private_data;
	mutex_lock(&hsidev->hsidev_lock);

	if (hsidev->hsi->cid != 1 ||
		hsidev->hsi->curr_mode == HSI_POLLING_MODE) {
		mutex_unlock(&hsidev->hsidev_lock);
		return -EINVAL;
	}

	if (hsidev->xfer && hsidev->xfer->data) {/* On going transfer */
		mutex_unlock(&hsidev->hsidev_lock);
		return -EBUSY;
	}

	if (!hsidev->xfer) {
		xferdata = kzalloc(sizeof(struct hsi_data), GFP_KERNEL);
		if (!xferdata) {
			mutex_unlock(&hsidev->hsidev_lock);
			return -ENOMEM;
		}
		hsidev->xfer = xferdata;
	}
	xferdata = hsidev->xfer;
	hsidev->xfer_done = 0;	/* Reset some fields */
	xferdata->count = count;

	if (hsidev->hsi->curr_mode == HSI_DMA_MODE) {
		xferdata->is_dma = 1;
		xferdata->data = dma_alloc_coherent(NULL, count,
						    &xferdata->dma_addr,
						    GFP_DMA);
	} else {
		xferdata->is_dma = 0;
		xferdata->data = kmalloc(count, GFP_KERNEL);
	}

	if (!xferdata->data) {
		mutex_unlock(&hsidev->hsidev_lock);
		return -ENOMEM;
	}

	status = hsi_read(hsidev->hsi, xferdata);
	if (status)
		goto out_err;

	status = wait_event_interruptible(hsidev->wq, hsidev->xfer_done);
	/* Check interrupted call, need to abort transfer */
	if (status)
		hsi_read_cancel(hsidev->hsi);
	else {
		missing = copy_to_user(buf, xferdata->data, count);
		if (missing)
			status = -EFAULT;
	}

out_err:
	if (xferdata->is_dma)
		dma_free_coherent(NULL, count,
				  xferdata->data, xferdata->dma_addr);
	else
		kfree(xferdata->data);

	xferdata->data = NULL;
	xferdata->dma_addr = (dma_addr_t) 0;
	mutex_unlock(&hsidev->hsidev_lock);
	return status;
}

/** data - pointer to data
 *  datawidth - in bits i.e 16 - 16bits, 32 - 32bits
 *  count - total number of data bytes to send (should always be in
 *  multiple of words)
 *
 *  Please refer test applications on how to pack the data to be sent.
 *  Also DMA transfers should  require dma coherent buffers (pls refer test
 *  app on how to do this)
 */
static ssize_t
hsidev_write(struct file *filp, const char __user *buf, size_t count,
	     loff_t *f_pos)
{
	struct hsidev_data *hsidev;
	ssize_t status = 0;
	int missing = 0;
	struct hsi_data *xferdata;

	hsidev = (struct hsidev_data *)filp->private_data;
	mutex_lock(&hsidev->hsidev_lock);

	if (hsidev->hsi->cid != 0 ||
		hsidev->hsi->curr_mode == HSI_POLLING_MODE){
		mutex_unlock(&hsidev->hsidev_lock);
		return -EINVAL;
	}

	if (hsidev->xfer && hsidev->xfer->data) {	/* On going transfer */
		mutex_unlock(&hsidev->hsidev_lock);
		return -EBUSY;
	}

	if (!hsidev->xfer) {
		xferdata = kzalloc(sizeof(struct hsi_data), GFP_KERNEL);
		if (!xferdata) {
			mutex_unlock(&hsidev->hsidev_lock);
			return -ENOMEM;
		}
		hsidev->xfer = xferdata;
	}
	xferdata = hsidev->xfer;
	hsidev->xfer_done = 0;	/* Reset some fields */

	if (hsidev->hsi->curr_mode == HSI_DMA_MODE) {
		xferdata->is_dma = 1;
		xferdata->data = dma_alloc_coherent(NULL, count,
						    &xferdata->dma_addr,
						    GFP_DMA);
	} else {
		xferdata->is_dma = 0;
		xferdata->data = kzalloc(count, GFP_KERNEL);
	}

	if (!xferdata->data) {
		mutex_unlock(&hsidev->hsidev_lock);
		return -ENOMEM;
	}

	missing = copy_from_user(xferdata->data, buf, count);
	if (missing) {
		status = -EFAULT;
		goto out_err;
	}

	xferdata->count = count;
	status = hsi_write(hsidev->hsi, xferdata);
	if (!status)
		status =
		    wait_event_interruptible(hsidev->wq, hsidev->xfer_done);
	else
		goto out_err;

	/* Check interrupted call, need to abort transfer */
	if (status)
		hsi_write_cancel(hsidev->hsi);

out_err:
	if (xferdata->is_dma)
		dma_free_coherent(NULL, count,
				  xferdata->data, xferdata->dma_addr);
	else
		kfree(xferdata->data);

	xferdata->data = NULL;
	xferdata->dma_addr = (dma_addr_t) 0;
	mutex_unlock(&hsidev->hsidev_lock);
	return status;
}

static int hsidev_release(struct inode *inode, struct file *filp)
{
	struct hsidev_data *hsidev;
	int status = 0;

	mutex_lock(&device_list_lock);
	hsidev = filp->private_data;
	hsi_close(hsidev->hsi);
	filp->private_data = NULL;

	/* last close? */
	mutex_lock(&hsidev->hsidev_lock);
	hsidev->users--;
	if (!hsidev->users) {
		kfree(hsidev->xfer);
		hsidev->xfer = NULL;
	}
	mutex_unlock(&hsidev->hsidev_lock);

	mutex_unlock(&device_list_lock);

	return status;
}

/** NOTE: before sending/receiving data the user should set an appropriate
 * watermark through this ioctl. The watermark determines how often the low
 * level HSI driver is interrupted to send/receive data on the link. The user
 * will be notified through callback only when all the data has been sent or
 * received
 */

static long hsidev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	u32 tmp;
	int size = 0;
	struct hsidev_data *hsidev;

	/* Check type and command number */
	if (_IOC_TYPE(cmd) != HSI_IOC_MAGIC)
		return -ENOTTY;

	/* Check access direction once here; don't repeat below.
	 * IOC_DIR is from the user perspective, while access_ok is
	 * from the kernel perspective; so they look reversed.
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
				 (void __user *)arg, _IOC_SIZE(cmd));
	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
				 (void __user *)arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;

	/* guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	hsidev = filp->private_data;
	get_device(&hsidev->hsi->dev);

	/* use the device lock here for triple duty:
	 *  - prevent concurrent HSI_IOC_WR_* from morphing
	 *    data fields while HSI_IOC_RD_* reads them;
	 *  - Prevent changing mode during an ongoing transfer
	 */
	mutex_lock(&hsidev->hsidev_lock);

	size = _IOC_SIZE(cmd);
	tmp = 0;
	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		err = copy_from_user((void *)&tmp, (void *)arg, size);
		if (err != 0)
			goto out_err;
	}
	err = hsi_ioctl(hsidev->hsi, cmd, &tmp);
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = copy_to_user((void *)arg, (void *)&tmp, size);
out_err:
	mutex_unlock(&hsidev->hsidev_lock);
	put_device(&hsidev->hsi->dev);
	return err;

}

void hsi_testprot_excep_handler(u32 ctrlr_id, u32 event, void *arg)
{
	u8 chid;
	/** exceptions are received only on HSIR */
	if (ctrlr_id != HSIR_CTRLR_ID) {
		stm_dbg(DBG_ST.hsi, "HSIDEV:excep on wrong ctrlr\n");
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
	 * 		HSI_EXCEP_PIPEBUF_OVERRUN,
	 * 		where it flushes HSIR pipeline buffer
	 * 		HSI_RXCHANNELS_OVERRUN ,
	 * 		where it flushes the HSIR channel buffer
	 */
	switch (event) {
	case HSI_EXCEP_TIMEOUT:
		stm_dbg(DBG_ST.hsi,
			"HSI test protocol driver:TIMEOUT received\n");
		break;
	case HSI_EXCEP_PIPEBUF_OVERRUN:
			/** HSI has flushed pipeline buffer...     *
			 *  protocol driver MUST take appropriate  *
			 *  action 				   *
			 **/
		stm_dbg(DBG_ST.hsi,
			"HSI test protocol driver:PIPEBUF OVERRUN received\n");
		break;
	case HSI_EXCEP_BREAK_DETECTED:
		stm_dbg(DBG_ST.hsi,
			"HSI test protocol driver:BREAK DETECT received\n");
		break;
	case HSI_EXCEP_PARITY_ERROR:
		stm_dbg(DBG_ST.hsi,
			"HSI test protocol driver: PARITY ERROR received\n");
		break;
	case HSI_RXCHANNELS_OVERRUN:
			/** HSI has flushed corresponding channel buffer  *
			 *...protocol driver MUST take appropriate action *
			 **/
		chid = *(u8 *) arg;
		stm_dbg(DBG_ST.hsi,
			"HSI test protocol driver: CHANNEL %d OVERRUN \n",
			chid);
		break;
	}

}

int hsi_testprot_drv_resume(struct hsi_device *dev)
{
	/*TODO: PM */
	return 0;
}

int hsi_testprot_drv_suspend(struct hsi_device *dev, pm_message_t state)
{
	 /*TODO*/ return 0;
}

static struct class *hsidev_class;

static int __init hsidev_probe(struct hsi_device *dev)
{
	int minor;
	int status = 0;
	unsigned long *minors;
	struct device *devc;
	struct hsidev_data *hsidev;

	stm_dbg(DBG_ST.hsi,
		"HSI test loopback protocol driver: device %s probed \n",
		dev->dev.bus_id);
	/* Allocate driver data */
	hsidev = kzalloc(sizeof(struct hsidev_data), GFP_KERNEL);
	if (!hsidev)
		return -ENOMEM;

	mutex_init(&hsidev->hsidev_lock);
	/* Initialize the driver data */
	hsidev->hsi = dev;
	minors = dev->cid ? &rx_minors : &tx_minors;

	minor = find_first_zero_bit(minors, N_HSI_MINORS);
	if (minor > 3) {
		stm_dbg(DBG_ST.hsi, "Minor %d invalid\n", minor);
		return -EBUSY;
	}

	mutex_lock(&device_list_lock);
	if (minor < N_HSI_MINORS) {
		int minor_t = minor;
		if (dev->cid == 0x1)
			minor_t += MAX_CHANNELS_MONITORED;

		hsidev->devt = MKDEV(HSIDEV_MAJOR, minor_t);
		devc = device_create(hsidev_class, &dev->dev,
					     hsidev->devt, hsidev,
					     "hsidev%d.%d", dev->cid, minor_t);
		status = IS_ERR(devc) ? PTR_ERR(devc) : 0;
	} else {
		dev_dbg(&dev->dev, "no minor number available!\n");
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&hsidev->device_entry, &device_list);
	}
	dev_set_drvdata(&dev->dev, (void *)hsidev);
	mutex_unlock(&device_list_lock);

	if (status != 0)
		kfree(hsidev);

	/* Initialize wait queue on which to block */
	init_waitqueue_head(&hsidev->wq);

	return status;
}

static int __exit hsidev_remove(struct hsi_device *dev)
{
	struct hsidev_data *hsidev = dev_get_drvdata(&dev->dev);
	unsigned long *minors;
	/* make sure ops on existing fds can abort cleanly */
	hsidev->hsi = NULL;
	dev_set_drvdata(&dev->dev, NULL);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&hsidev->device_entry);
	device_destroy(hsidev_class, hsidev->devt);
	minors = dev->cid ? &rx_minors : &tx_minors;
	clear_bit(MINOR(hsidev->devt), minors);
	if (hsidev->users == 0)
		kfree(hsidev);
	hsi_set_callback(dev, NULL, NULL);
	mutex_unlock(&device_list_lock);
	return 0;
}

static struct hsi_device_driver hsidev_driver = {
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
	.probe = hsidev_probe,
	.remove = __exit_p(hsidev_remove),
	.resume = hsi_testprot_drv_resume,
	.suspend = hsi_testprot_drv_suspend,
	.driver = {
		   .name = HSI_TESTPROT_DRIVER_NAME,
		   },
};

/* The main reason to have this class is to make mdev/udev create the
 * /dev/hsidevB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

static struct file_operations hsidev_fops = {
	.owner = THIS_MODULE,
	.write = hsidev_write,
	.read = hsidev_read,
	.unlocked_ioctl = hsidev_ioctl,
	.open = hsidev_open,
	.release = hsidev_release,
};

static int __init hsi_testprot_init(void)
{
	int status;

	status = register_chrdev(HSIDEV_MAJOR, "hsi", &hsidev_fops);
	if (status < 0)
		return status;

	hsidev_class = class_create(THIS_MODULE, "hsidev");
	if (IS_ERR(hsidev_class)) {
		unregister_chrdev(HSIDEV_MAJOR, hsidev_driver.driver.name);
		return PTR_ERR(hsidev_class);
	}

	status = register_hsi_driver(&hsidev_driver);
	if (status < 0) {
		class_destroy(hsidev_class);
		unregister_chrdev(HSIDEV_MAJOR, hsidev_driver.driver.name);
		stm_dbg(DBG_ST.hsi,
			"HSIDEV driver registration failed\n");
		return status;
	}
	printk(KERN_NOTICE"hsidev initialized\n");

	return 0;
}

static void __exit hsi_testprot_exit(void)
{
	unregister_hsi_driver(&hsidev_driver);
	class_destroy(hsidev_class);
	unregister_chrdev(HSIDEV_MAJOR, hsidev_driver.driver.name);
	printk(KERN_NOTICE"hsidev removed\n");
}

module_init(hsi_testprot_init);
module_exit(hsi_testprot_exit);

MODULE_AUTHOR("STMicroelectronics");
MODULE_DESCRIPTION("HSI test protocol driver");
MODULE_LICENSE("GPL");
