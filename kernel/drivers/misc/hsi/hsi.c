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

#include <linux/hsi.h>
#include <mach/hsi-stm.h>

/* LDM. defintions for the hsi bus, hsi device, and hsi_device driver */
struct bus_type hsi_bus_type;
static LIST_HEAD(__hsi_board_list);
static DEFINE_MUTEX(__hsi_board_lock);

static void hsi_controller_device_release(struct device *dev)
{
	struct hsi_device *hsi = to_hsi_device(dev);

	kfree(hsi);
	return;
}

/** hsi_add_device - instantiate an hsi device and add it to the bus
 * @dev: The device to instatiate.
 * This function is called by the kernel automatically on device release
 * HSI bustype and hsi controller class are registered after board init code
 * provides the HSI device tables, ensuring that both are present by the
 * time controller driver registration causes hsi_devices to "enumerate".
 */
static void hsidev_release(struct device *dev)
{
	struct hsi_device *hsi = to_hsi_device(dev);

	/* TODO: hsi masters may cleanup for released devices */
	kfree(hsi);
}

/**
 * hsi_add_device - instantiate an hsi device and add it to the bus
 * @info: describes one HSI device with channel and bus information
 * Context: can sleep
 *
 * Create a device to work with a hsi driver, where binding is
 * handled through driver model probe()/remove() methods.
 *
 * This returns the new hsi client, which may be saved for later use with
 * hsi_unregister_device(); or NULL to indicate an error.
 */
struct hsi_device *hsi_add_device(struct hsi_device *dev,
				  struct hsi_board_info const *info)
{
	struct hsi_device *client;
	int status;

	client = kzalloc(sizeof *client, GFP_KERNEL);
	if (!client) {
		printk(KERN_ERR"HSI: Device alloc failed\n");
		return NULL;
	}

	client->cid = info->controller_id;
	client->chid = info->chan_num;

	client->ctrlr = dev->ctrlr;
	client->curr_mode = info->mode;
	if (!client->cid)
		client->ch = &dev->ctrlr->hsi_tx_channels[client->chid];
	else
		client->ch = &dev->ctrlr->hsi_rx_channels[client->chid];

	client->dev.platform_data = info->platform_data;

	snprintf(client->dev.bus_id, sizeof client->dev.bus_id,
		 "%s.%u.%u", "hsi", info->controller_id, info->chan_num);

	strlcpy(client->name, info->type, sizeof(client->name));

	client->dev.parent = &dev->dev;
	client->dev.bus = &hsi_bus_type;
	client->dev.release = hsidev_release;
	client->ch->dev = client;

	status = device_register(&client->dev);
	if (status) {
		printk(KERN_ERR"HSI: Device register failed\n");
		return NULL;
	}

	return client;
}
EXPORT_SYMBOL_GPL(hsi_add_device);

/**
 * hsi_open - open a hsi device channel.
 * @dev: Reference to the hsi device channel to be openned.
 *
 * Returns 0 on success, -EINVAL on bad parameters, -EBUSY if is already opened.
**/

int hsi_open(struct hsi_device *dev)
{
	return dev->ctrlr->algo->open(dev);
}
EXPORT_SYMBOL_GPL(hsi_open);

/**
 * hsi_write - write data into the hsi device channel
 * @dev: reference to the hsi device channel  to write into.
 * @data: pointer to a 32-bit word data to be written.
 * @datawidth: in bits (16bit or 32bit or ....)
 * @count: total number of bytes to be written.
 *
 * Return 0 on sucess, a negative value on failure.
 * A success values only indicates that the request has been accepted.
 * Transfer is only completed when the write_done callback is called.
 *
**/
int hsi_write(struct hsi_device *dev, struct hsi_data *xfer)
{
	return dev->ctrlr->algo->write(dev, xfer);
}
EXPORT_SYMBOL_GPL(hsi_write);

/**
 * hsi_read - read data from the hsi device channel
 * @dev: hsi device channel reference to read data from.
 * @data: pointer to a 32/16-bit word data to store the data.
 * @count: number of 32/16-bit word to be stored.
 *
 * Return 0 on sucess, a negative value on failure.
 * A success values only indicates that the request has been accepted.
 * Data is only available in the buffer when the read_done callback is called.
 *
**/
int hsi_read(struct hsi_device *dev, struct hsi_data *xfer)
{
	return dev->ctrlr->algo->read(dev, xfer);
}
EXPORT_SYMBOL_GPL(hsi_read);

/**
 * hsi_write_cancel - Cancel pending write request.
 * @dev: hsi device channel where to cancel the pending write.
 *
 * write_done() callback will not be called after sucess of this function.
 * This call closes the channel also.
**/
void hsi_write_cancel(struct hsi_device *dev)
{
	dev->ctrlr->algo->cancel_write(dev);
	return;
}
EXPORT_SYMBOL_GPL(hsi_write_cancel);

/**
 * hsi_read_cancel - Cancel pending read request.
 * @dev: hsi device channel where to cancel the pending read.
 *
 * read_done() callback will not be called after sucess of this function.
**/
void hsi_read_cancel(struct hsi_device *dev)
{
	dev->ctrlr->algo->cancel_read(dev);
	return;
}
EXPORT_SYMBOL_GPL(hsi_read_cancel);

/**
 * hsi_dev_set_cb - register read_done() and write_done() callbacks.
 * @dev: reference to hsi device channel where callbacks are associated.
 * @r_cb: callback to signal read transfer completed.
 * @w_cb: callback to signal write transfer completed.
 * Context: Can sleep
**/
void hsi_set_callback(struct hsi_device *dev,
		      void (*r_cb) (struct hsi_device *dev)
		      , void (*w_cb) (struct hsi_device *dev))
{
	dev->ctrlr->algo->set_cb(dev, r_cb, w_cb);
	return;
}
EXPORT_SYMBOL_GPL(hsi_set_callback);

/**
 * hsi_ioctl - HSI I/O control
 * @dev: hsi device channel reference to apply the I/O control
 * 	(or port associated to it)
 * @command: HSI I/O control command
 * @arg: parameter associated to the control command. NULL, if no parameter.
 * Context: Can sleep
 *
 * Return 0 on sucess, a negative value on failure.
 *
**/
int hsi_ioctl(struct hsi_device *dev, unsigned int command, void *arg)
{
	return dev->ctrlr->algo->ioctl(dev, command, arg);
}
EXPORT_SYMBOL_GPL(hsi_ioctl);

/**
 * hsi_close - close given hsi device channel
 * @dev: reference to hsi device channel.
**/
void hsi_close(struct hsi_device *dev)
{
	dev->ctrlr->algo->close(dev);
	return;
}
EXPORT_SYMBOL_GPL(hsi_close);

static void hsi_scan_static_board_info(struct hsi_device *dev)
{
	struct hsi_board_info *boardinfo;

	mutex_lock(&__hsi_board_lock);
	list_for_each_entry(boardinfo, &__hsi_board_list, list) {
		if (boardinfo->controller_id == dev->cid
		    && !hsi_add_device(dev, boardinfo))
			printk(KERN_ERR "hsi-core: can't create hsi%d-%d\n",
			       boardinfo->controller_id, boardinfo->chan_num);
	}
	mutex_unlock(&__hsi_board_lock);
}

int hsi_add_controller(struct hsi_controller *hsi_ctrlr)
{
	struct hsi_device *hdev;
	unsigned int n_ch = 0;
	int ch = 0;
	int err = 0;

	n_ch = hsi_ctrlr->max_ch;

	hdev = kzalloc(sizeof(*hdev), GFP_KERNEL);
	if (!hdev) {
		err = -ENOMEM;
		return err;
	}
	hdev->cid = hsi_ctrlr->dev_type;
	hdev->ctrlr = hsi_ctrlr;
	hdev->dev.parent = &platform_bus;
	hdev->dev.release = hsi_controller_device_release;
	hsi_ctrlr->dev = &hdev->dev;
	snprintf(hdev->dev.bus_id, sizeof(hdev->dev.bus_id),
		 hdev->cid ? "hsirx-ch%u" : "hsitx-ch%u", ch);
	err = device_register(&hdev->dev);
	if (!err)
		hsi_scan_static_board_info(hdev);

	return err;
}
EXPORT_SYMBOL_GPL(hsi_add_controller);

static int __unregister(struct device *dev, void *master_dev)
{
	/* note: before about 2.6.14-rc1 this would corrupt memory: */
	device_unregister(dev);
	return 0;
}

/**
 * hsi_remove_controller - unregister HSI controller
 * @controller: The controller to be unregistered
 * Context: can sleep
 *
 * This call is used only by HSI controller drivers, which are the
 * only ones directly touching chip registers.
 *
 * This must be called from context that can sleep.
 */
void hsi_remove_controller(struct hsi_controller *controller)
{
	int dummy;

	dummy = device_for_each_child(controller->dev, controller->dev,
				      __unregister);
	device_unregister(controller->dev);
}
EXPORT_SYMBOL_GPL(hsi_remove_controller);

int hsi_register_board_info(struct hsi_board_info const *info, int len)
{
	int status;

	mutex_lock(&__hsi_board_lock);

	for (status = 0; len; len--, info++) {
		struct hsi_board_info *binfo;

		binfo = kzalloc(sizeof(*info), GFP_KERNEL);
		if (!binfo) {
			printk(KERN_NOTICE
			       "hsi-core: can't register boardinfo!\n");
			status = -ENOMEM;
			break;
		}
		memcpy(binfo, info, sizeof *info);
		list_add_tail(&binfo->list, &__hsi_board_list);
	}

	mutex_unlock(&__hsi_board_lock);

	return status;
}
EXPORT_SYMBOL_GPL(hsi_register_board_info);

static int modalias_show(struct device *dev, struct device_attribute *a,
			 char *buf)
{
	return snprintf(buf, BUS_ID_SIZE + 1, "%s%s\n", HSI_PREFIX,
			dev->bus_id);
}

static struct device_attribute hsi_dev_attrs[] = {
	__ATTR_RO(modalias),
	__ATTR_NULL,
};

static int hsi_bus_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	add_uevent_var(env, "MODALIAS=%s%s", HSI_PREFIX, dev->bus_id);
	return 0;
}

static ssize_t
show_controller_name(struct device *dev, struct device_attribute *attr,
		     char *buf)
{
	struct hsi_device *hdev = to_hsi_device(dev);
	return sprintf(buf, "%s\n", hdev->ctrlr->name);
}

static struct device_attribute hsi_controller_attrs[] = {
	__ATTR(name, S_IRUGO, show_controller_name, NULL),
	{},
};

static struct class hsi_controller_class = {
	.owner = THIS_MODULE,
	.name = "hsi-controller",
	.dev_attrs = hsi_controller_attrs,
};

static int hsi_bus_match(struct device *device, struct device_driver *driver)
{
	int error;
	struct hsi_device *dev = to_hsi_device(device);
	struct hsi_device_driver *drv = to_hsi_device_driver(driver);

	error = strcmp(dev->name, drv->driver.name);
	return !error;
}

int hsi_bus_unreg_dev(struct device *device, void *p)
{
	device->release(device);
	device_unregister(device);

	return 0;
}

int __init hsi_bus_init(void)
{
	int status;

	status = bus_register(&hsi_bus_type);
	if (status < 0)
		goto err0;

	status = class_register(&hsi_controller_class);
	if (status < 0)
		goto err1;

	printk(KERN_NOTICE"HSI bus initialised\n");
	return 0;
err1:
	bus_unregister(&hsi_bus_type);
err0:
	return status;

}

void hsi_bus_exit(void)
{
	bus_for_each_dev(&hsi_bus_type, NULL, NULL, hsi_bus_unreg_dev);
	class_unregister(&hsi_controller_class);
	bus_unregister(&hsi_bus_type);
	printk(KERN_NOTICE"HSI bus deinitialised\n");
}

static int hsi_driver_probe(struct device *dev)
{
	struct hsi_device_driver *drv = to_hsi_device_driver(dev->driver);

	return drv->probe(to_hsi_device(dev));
}

static int hsi_driver_remove(struct device *dev)
{
	struct hsi_device_driver *drv = to_hsi_device_driver(dev->driver);

	return drv->remove(to_hsi_device(dev));
}

static int hsi_driver_suspend(struct device *dev, pm_message_t mesg)
{
	struct hsi_device_driver *drv = to_hsi_device_driver(dev->driver);

	return drv->suspend(to_hsi_device(dev), mesg);
}

static int hsi_driver_resume(struct device *dev)
{
	struct hsi_device_driver *drv = to_hsi_device_driver(dev->driver);

	return drv->resume(to_hsi_device(dev));
}

/* NOTE: Function called in interrupt context */
int hsi_exception(struct hsi_controller *hsi_ctrlr, unsigned int event,
		  void *arg)
{
	int err = 0;
	struct hsi_ctrlr_excep c_ex = {
		.ctrlr = hsi_ctrlr,
		.event = event,
		.priv = arg
	};

	err =
	    bus_for_each_drv(&hsi_bus_type, NULL, &c_ex,
			     hsi_ctrlr->algo->exception_handler);

	return err;
}
EXPORT_SYMBOL_GPL(hsi_exception);

struct bus_type hsi_bus_type = {
	.name = "hsi",
	.dev_attrs = hsi_dev_attrs,
	.match = hsi_bus_match,
	.uevent = hsi_bus_uevent,
};

/**
 * register_hsi_driver - Register HSI device driver
 * @driver - reference to the HSI device driver.
 */
int register_hsi_driver(struct hsi_device_driver *driver)
{
	int ret = 0;

	if (driver == NULL)
		return -ENODEV;

	driver->driver.bus = &hsi_bus_type;

#define HSI_SETFN(fn)	{ if (!driver->driver.fn)  \
			driver->driver.fn = hsi_driver_##fn; }

	HSI_SETFN(probe);
	HSI_SETFN(remove);
	HSI_SETFN(suspend);
	HSI_SETFN(resume);

	ret = driver_register(&driver->driver);
	return ret;
}
EXPORT_SYMBOL(register_hsi_driver);

/**
 * unregister_hsi_driver - Unregister HSI device driver
 * @driver - reference to the HSI device driver.
 */
void unregister_hsi_driver(struct hsi_device_driver *driver)
{
	if (driver != NULL)
		driver_unregister(&driver->driver);
}
EXPORT_SYMBOL_GPL(unregister_hsi_driver);

/** board_info is normally registered in arch_initcall(),
  *  but even essential drivers wait till later
  *
  *  REVISIT only boardinfo really needs static linking. the rest (device and
  *  driver registration) _could_ be dynamically linked (modular) ... costs
  *  include needing to have boardinfo data structures be much more public.
**/
subsys_initcall(hsi_bus_init);

