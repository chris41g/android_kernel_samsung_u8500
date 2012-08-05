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

#include <linux/device.h>
#include "hsi_driver.h"

/* LDM. defintions for the hsi bus, hsi device, and hsi_device driver */
struct bus_type hsi_bus_type;

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

/* NOTE: Function called in interrupt context */
static int hsi_ex_handler(struct device_driver *drv, void *ex_event)
{
	struct hsi_ctrlr_excep *event = (struct hsi_ctrlr_excep *)ex_event;
	struct hsi_device_driver *hsi_drv =  to_hsi_device_driver(drv);
	u8 id=0;

	if ((hsi_drv->excep_event) &&
		(test_bit(event->event, (const volatile unsigned long *)&hsi_drv->excep_mask)) &&
		(test_bit(event->ctrlr->dev_type, (const volatile unsigned long *)&hsi_drv->ctrl_mask)) &&
		(hsi_drv->ch_mask != 0)) {
		if (event->priv) {
			/** channel overrun - we pass the channel id as private data */
			id = *(u8 *)event->priv;
			if (test_bit(id,(const volatile unsigned long *)&hsi_drv->ch_mask))
				/** callback for channel overrun */
				(*hsi_drv->excep_event)(event->ctrlr->dev_type,
						event->event, &id);
		}
		else
			/** callback for everything except channel overrrun */
			(*hsi_drv->excep_event)(event->ctrlr->dev_type,
							event->event, event->priv);
	}

	return 0;
}

/* NOTE: Function called in interrupt context */
int hsi_excep_handler(struct hsi_dev *hsi_ctrlr, unsigned int event, void *arg)
{
	int err = 0;
	struct hsi_ctrlr_excep c_ex = {
		.ctrlr = hsi_ctrlr,
		.event = event,
		.priv = arg
	};

	err = bus_for_each_drv(&hsi_bus_type, NULL, &c_ex, hsi_ex_handler);

	return err;
}

static int hsi_bus_match(struct device *device, struct device_driver *driver)
{
	struct hsi_device *dev = to_hsi_device(device);
	struct hsi_device_driver *drv = to_hsi_device_driver(driver);

	if (!test_bit(dev->ctrlrid, (const volatile unsigned long *)&drv->ctrl_mask))
		return 0;

	if (!test_bit(dev->chid, (const volatile unsigned long *)&drv->ch_mask))
		return 0;

	return 1;
}

int hsi_bus_unreg_dev(struct device *device, void *p)
{
	device->release(device);
	device_unregister(device);

	return 0;
}

int __init hsi_bus_init(void)
{
	return bus_register(&hsi_bus_type);
}

void hsi_bus_exit(void)
{
	bus_for_each_dev(&hsi_bus_type, NULL, NULL, hsi_bus_unreg_dev);
	bus_unregister(&hsi_bus_type);
}

static int hsi_driver_probe(struct device *dev)
{
	struct hsi_device_driver *drv = to_hsi_device_driver(dev->driver);

	return 	drv->probe(to_hsi_device(dev));
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

	if (driver == NULL) return -ENODEV;

	driver->driver.bus = &hsi_bus_type;
	if (driver->probe)
		driver->driver.probe = hsi_driver_probe;
	if (driver->remove)
		driver->driver.remove = hsi_driver_remove;
	if (driver->suspend)
		driver->driver.suspend = hsi_driver_suspend;
	if (driver->resume)
		driver->driver.resume = hsi_driver_resume;

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
	if (driver != NULL) driver_unregister(&driver->driver);
}
EXPORT_SYMBOL(unregister_hsi_driver);
