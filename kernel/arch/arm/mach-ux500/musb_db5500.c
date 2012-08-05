/*
 * Copyright (C) ST-Ericsson SA 2011
 *
 * Author: Avinash Kumar <avinash.kumar@stericsson.com> for ST-Ericsson
 * License terms: GNU General Public License (GPL) version 2
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/usb/musb.h>
#include <linux/interrupt.h>
#include <linux/mfd/abx500.h>
#include <linux/mfd/ab8500.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <mach/musb_db5500.h>
#include <mach/prcmu.h>
#ifdef CONFIG_USB_ANDROID
#include <linux/usb/android_composite.h>
#endif
#include <mach/devices.h>
#include <linux/mfd/abx500/ab5500.h>
#include <linux/mfd/abx500.h>

#define AB5500_USB_DEVICE_DISABLE 0x0
#define AB5500_MAIN_WDOG_CTRL_REG      0x01
#define AB5500_USB_LINE_STAT_REG       0x80
#define AB5500_USB_PHY_CTRL_REG        0x8A
#define AB5500_MAIN_WATCHDOG_ENABLE 0x1
#define AB5500_MAIN_WATCHDOG_KICK 0x2
#define AB5500_MAIN_WATCHDOG_DISABLE 0x0

#define DEVICE_NAME "musb_qos"
static struct completion usb_link_status_update;
static int irq_host_remove;
static int irq_device_remove;
static int irq_device_insert;
static int irq_link_status_update;
static struct device *device;
static struct clk *sysclock;
static int ab5500_rev;
/* Phy Status. Right now Used for Device only. */
#ifdef CONFIG_USB_OTG_20
static int irq_adp_plug;
static int irq_adp_unplug;
#endif

/*
 * work queue for USB cable insertion processing
 */
static struct work_struct usb_device_remove;
static struct work_struct usb_device_insert;

static struct work_struct usb_lnk_status_update;
static struct workqueue_struct *usb_cable_wq;

static enum musb_link_status stm_usb_link_curr_state = USB_LINK_NOT_CONFIGURED;

static void usb_link_status_update_work(struct work_struct *work);
static void usb_device_remove_work(struct work_struct *work);
static void usb_device_insert_work(struct work_struct *work);

#include <linux/wakelock.h>
static struct wake_lock ab5500_musb_wakelock;

/**
 * stm_musb_states - Different states of musb_chip
 *
 * Used for USB cable plug-in state machine
 */
enum stm_musb_states {
	USB_IDLE,
	USB_DEVICE,
	USB_HOST,
	USB_DEDICATED_CHG,
};
enum stm_musb_states stm_musb_curr_state = USB_IDLE;

void musb_platform_session_req()
{
   /* Discaharge the VBUS */
   /* TODO */
}

/**
 * usb_kick_watchdog() - Kick the watch dog timer
 *
 * This function used to Kick the watch dog timer
 */
static void usb_kick_watchdog(void)
{
	abx500_set_register_interruptible(device,
			AB8500_SYS_CTRL2_BLOCK,
			AB5500_MAIN_WDOG_CTRL_REG,
			AB5500_MAIN_WATCHDOG_ENABLE);

		udelay(WATCHDOG_DELAY_US);

	abx500_set_register_interruptible(device,
			AB8500_SYS_CTRL2_BLOCK,
			AB5500_MAIN_WDOG_CTRL_REG,
			(AB5500_MAIN_WATCHDOG_ENABLE
			 | AB5500_MAIN_WATCHDOG_KICK));

		udelay(WATCHDOG_DELAY_US);

	abx500_set_register_interruptible(device,
			AB8500_SYS_CTRL2_BLOCK,
			AB5500_MAIN_WDOG_CTRL_REG,
			AB5500_MAIN_WATCHDOG_DISABLE);

		udelay(WATCHDOG_DELAY_US);
}


/**
 * musb_platform_device_en(): Enable/Disable the device.
 * This function used to enable usb sysclk and supply
 */
void musb_platform_device_en(int enable)
{
	clk_enable(sysclock);
}

/**
 * This function is used to signal the completion of
 * USB Link status register update
 */
static irqreturn_t usb_link_status_update_handler(int irq, void *data)
{
	queue_work(usb_cable_wq, &usb_lnk_status_update);
	return IRQ_HANDLED;
}


/**
 * This function used to remove the voltage for USB device mode.
 */
static irqreturn_t usb_device_remove_handler(int irq, void *data)
{
	/* Change the current state */
	stm_musb_curr_state = USB_IDLE;
	queue_work(usb_cable_wq, &usb_device_remove);
	return IRQ_HANDLED;
}

/**
 * This function used to remove the voltage for USB device mode.
 */
static irqreturn_t usb_device_insert_handler(int irq, void *data)
{
	/* Change the current state */
	stm_musb_curr_state = USB_IDLE;
	queue_work(usb_cable_wq, &usb_device_insert);
	return IRQ_HANDLED;
}

/* Work created after an link status update handler*/
static void usb_link_status_update_work(struct work_struct *work)
{
	u8 val = 0;

	(void)abx500_get_register_interruptible(device,
			AB5500_BANK_USB, AB5500_USB_LINE_STAT_REG, &val);

	val = (val & AB5500_USB_LINK_STATUS) >> 3;

	switch (stm_usb_link_curr_state) {

	case USB_LINK_STD_HOST_NC:

	abx500_set_register_interruptible(device,
		AB5500_BANK_USB,
		AB5500_USB_PHY_CTRL_REG, 2);
	default:
		break;
	}
}

/**
 * usb_device_insert_work : work handler for device cable insert.
 * @work: work structure
 *
 * This function is used to handle the device cable insert work.
 */
static void usb_device_insert_work(struct work_struct *work)
{
	wake_lock(&ab5500_musb_wakelock);
}


/**
 * usb_device_remove_work : work handler for device cable insert.
 * @work: work structure
 *
 * This function is used to handle the device cable insert work.
 */
static void usb_device_remove_work(struct work_struct *work)
{
/* this delay needed for USB phy, confirmed by hardware team */
	abx500_set_register_interruptible(device,
				AB5500_BANK_USB,
				AB5500_USB_PHY_CTRL_REG,
				AB5500_USB_DEVICE_ENABLE);

	udelay(100);

	abx500_set_register_interruptible(device,
				AB5500_BANK_USB,
				AB5500_USB_PHY_CTRL_REG,
				AB5500_USB_DEVICE_DISABLE);

	wake_unlock(&ab5500_musb_wakelock);
}

/**
 * musb_phy_en : register USB callback handlers for ab5500
 * @mode: value for mode.
 *
 * This function is used to register USB callback handlers for ab5500.
 */
int musb_phy_en(u8 mode)
{
	int ret = 0;
	if (!device)
		return -EINVAL;
	ab5500_rev = abx500_get_chip_id(device);

	sysclock = clk_get(device, "sysclk");
	if (IS_ERR(sysclock)) {
		ret = PTR_ERR(sysclock);
		sysclock = NULL;
		return ret;
	}

	/* create a thread for work */
	usb_cable_wq = create_singlethread_workqueue(
				"usb_cable_wq");
		if (usb_cable_wq == NULL)
			return -ENOMEM;

	INIT_WORK(&usb_lnk_status_update,
				usb_link_status_update_work);

	/* Required for Host, Device and OTG mode */
	init_completion(&usb_link_status_update);

	ret = request_threaded_irq(irq_link_status_update,
		NULL, usb_link_status_update_handler,
		IRQF_NO_SUSPEND | IRQF_SHARED,
		"usb-link-status-update", device);
	if (ret < 0) {
		printk(KERN_ERR "failed to set the callback"
				" handler for usb charge"
				" detect done\n");
		return ret;
	}

	ret = request_threaded_irq(irq_device_remove, NULL,
		usb_device_remove_handler,
		IRQF_NO_SUSPEND | IRQF_SHARED,
		"usb-device-remove", device);
	if (ret < 0) {
		printk(KERN_ERR "failed to set the callback"
				" handler for usb device"
				" removal\n");
		return ret;
	}

	ret = request_threaded_irq(irq_device_insert, NULL,
		usb_device_insert_handler,
		IRQF_NO_SUSPEND | IRQF_SHARED,
		"usb-device-insert", device);
	if (ret < 0) {
		printk(KERN_ERR "failed to set the callback"
				" handler for usb device"
				" insertion\n");
		return ret;
	}

	INIT_WORK(&usb_device_remove, usb_device_remove_work);
	INIT_WORK(&usb_device_insert, usb_device_insert_work);

	usb_kick_watchdog();

	return 0;
}

/**
 * musb_force_detect : detect the USB cable during boot time.
 * @mode: value for mode.
 *
 * This function is used to detect the USB cable during boot time.
 */
int musb_force_detect(u8 mode)
{
	if (!device)
		return -EINVAL;

	abx500_set_register_interruptible(device,
			AB5500_BANK_USB,
			AB5500_USB_PHY_CTRL_REG,
			AB5500_USB_DEVICE_DISABLE);

	return 0;
}

/*
 * musb_get_abx500_rev : Get the ABx500 revision number
 *
 * This function returns the ABx500 revision number.
 */
int musb_get_abx500_rev()
{
	return abx500_get_chip_id(device);
}

static int __devinit ab5500_musb_probe(struct platform_device *pdev)
{
	device = &pdev->dev;

	irq_host_remove = platform_get_irq_byname(pdev, "usb_idgnd_f");
	if (irq_host_remove < 0) {
		dev_err(&pdev->dev, "Host remove irq not found, err %d\n",
			irq_host_remove);
		return irq_host_remove;
	}

	irq_device_remove = platform_get_irq_byname(pdev, "VBUS_F");
	if (irq_device_remove < 0) {
		dev_err(&pdev->dev, "Device remove irq not found, err %d\n",
			irq_device_remove);
		return irq_device_remove;
	}

	irq_device_insert = platform_get_irq_byname(pdev, "VBUS_R");
	if (irq_device_remove < 0) {
		dev_err(&pdev->dev, "Device remove irq not found, err %d\n",
			irq_device_remove);
		return irq_device_remove;
	}

	irq_link_status_update = platform_get_irq_byname(pdev,
		"Link_Update");
	if (irq_link_status_update < 0) {
		dev_err(&pdev->dev, "USB Link status irq not found, err %d\n",
			irq_link_status_update);
		return irq_link_status_update;
	}

	/*
	 * wake lock is acquired when usb cable is connected and released when
	 * cable is removed
	 */
	wake_lock_init(&ab5500_musb_wakelock, WAKE_LOCK_SUSPEND, "ab5500-usb");

	return 0;
}

static int __devexit ab5500_musb_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver ab5500_musb_driver = {
	.driver		= {
		.name	= "ab5500-usb",
		.owner	= THIS_MODULE,
	},
	.probe		= ab5500_musb_probe,
	.remove		= __devexit_p(ab5500_musb_remove),
};

static int __init ab5500_musb_init(void)
{
	return platform_driver_register(&ab5500_musb_driver);
}
module_init(ab5500_musb_init);

static void __exit ab5500_musb_exit(void)
{
	platform_driver_unregister(&ab5500_musb_driver);
}
module_exit(ab5500_musb_exit);

MODULE_LICENSE("GPL v2");
