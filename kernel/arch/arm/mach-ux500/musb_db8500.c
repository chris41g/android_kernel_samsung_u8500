/*
 * Copyright (C) 2009 ST Ericsson.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
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
#include <mach/musb_db8500.h>
#include <mach/prcmu.h>
#include <linux/usb_switcher.h>
#include <plat/pincfg.h>
#include "pins.h"
#include "pm/cpufreq.h"

/* Registers in Bank 0x02 */
#define AB8500_MAIN_WDOG_CTRL_REG      0x01

/* Registers in Bank 0x05 */
#define AB8500_REGU_VUSB_CTRL_REG      0x82
#define AB8500_USB_LINE_STAT_REG       0x80
#define AB8500_USB_LINE_CTRL1_REG      0x81
#define AB8500_USB_LINE_CTRL2_REG      0x82
#define AB8500_USB_LINE_CTRL3_REG      0x83
#define AB8500_USB_LINE_CTRL4_REG      0x84
#define AB8500_USB_LINE_CTRL5_REG      0x85
#define AB8500_USB_OTG_CTRL_REG        0x87
#define AB8500_USB_OTG_STAT_REG        0x88
#define AB8500_USB_OTG_STAT_REG        0x88
#define AB8500_USB_CTRL_SPARE_REG      0x89
#define AB8500_USB_PHY_CTRL_REG        0x8A
#define AB8500_USB_ADP_CTRL_REG        0x93

/* Registers in Bank 0x0E */
#define AB8500_IT_MASK2_REG            0x41
#define AB8500_IT_MASK12_REG           0x4B
#define AB8500_IT_MASK20_REG           0x53
#define AB8500_IT_MASK21_REG           0x54
#define AB8500_IT_SOURCE2_REG          0x01
#define AB8500_IT_SOURCE20_REG         0x13

/* Registers in bank 0x11 */
#define AB8500_BANK12_ACCESS	       0x00

/* Registers in bank 0x12 */
#define AB8500_USB_PHY_TUNE1           0x05
#define AB8500_USB_PHY_TUNE2           0x06
#define AB8500_USB_PHY_TUNE3           0x07

#define DEVICE_NAME "musb_qos"

static struct completion usb_link_status_update;
static struct device *device;
static struct regulator *musb_vape_supply;
static struct regulator *musb_vintcore_supply, *musb_smps2_supply;
static struct clk *sysclock;

static int boot_time_flag = USB_DISABLE;
static int irq_host_remove;
static int irq_device_remove;
static int irq_link_status_update;
static int ab8500_rev;
/* Phy Status. Right now Used for Device only. */
static int phy_enable_stat = USB_DISABLE;
#ifdef CONFIG_USB_OTG_20
static int irq_adp_plug;
static int irq_adp_unplug;
#endif
/*
 * work queue for USB cable insertion processing
 */
static struct work_struct usb_host_remove;
static struct work_struct usb_device_remove;
static struct work_struct usb_lnk_status_update;
static struct work_struct usb_dedicated_charger_remove;
static struct workqueue_struct *usb_cable_wq;

static void usb_host_remove_work(struct work_struct *work);
static void usb_device_remove_work(struct work_struct *work);
static void usb_link_status_update_work(struct work_struct *work);
static void usb_dedicated_charger_remove_work(struct work_struct *work);
static void usb_device_phy_en(int enable);
static void usb_kick_watchdog(void);
static void usb_host_phy_en(int enable);

static struct notifier_block notifier_block ;

#include <linux/wakelock.h>
static struct wake_lock ab8500_musb_wakelock;
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

enum muic_connect_states {
	MUIC_USB_DISCONNECT,
	MUIC_USB_CONNECT_CHECK,
	MUIC_DETECTION_SUCCESS,
};

/* ACA Modification */
static enum musb_link_status stm_usb_link_curr_state = USB_LINK_NOT_CONFIGURED;
static enum musb_link_status stm_usb_link_prev_state = USB_LINK_NOT_CONFIGURED;

void musb_set_session(void);
#ifdef	CONFIG_PM
void stm_musb_context(int);
#endif

#define MSG_DBG(fmt, args...) \
		printk(KERN_INFO "[MUSB] %s:%d "fmt, __func__, __LINE__, ##args)

#ifdef CONFIG_USB_OTG_20
int musb_adp(void);
static void enable_adp(void)
{
	if (ab8500_rev == AB8500_REV_20)
		abx500_set_register_interruptible(device,
			AB8500_USB,
			AB8500_USB_ADP_CTRL_REG,
			AB8500_USB_ADP_ENABLE);
}
#endif

void musb_platform_session_req(void)
{
   /* Discaharge the VBUS */
   /* TODO */
}

/*
 * musb_platform_device_en(): Enable/Disable the device.

 */

void musb_platform_device_en(int enable)
{
	int ret;

	if ((enable == 1) && (phy_enable_stat == USB_DISABLE)) {
		stm_musb_curr_state = USB_DEVICE;
		usb_device_phy_en(USB_ENABLE);
		return;
	}

	if ((enable == 1) && (phy_enable_stat == USB_ENABLE)) {
		/* Phy already enabled. no need to do anything. */
		return;
	}

	if ((enable == 0) && (phy_enable_stat == USB_DISABLE)) {
		/* Phy already disabled. no need to do anything. */
		return;
	}

	if ((enable == 0) && (phy_enable_stat == USB_ENABLE)) {
		/* Phy enabled. Disable it */
		abx500_set_register_interruptible(device,
			AB8500_USB,
			AB8500_USB_PHY_CTRL_REG,
			AB8500_USB_DEVICE_DISABLE);
		prcmu_qos_update_requirement(PRCMU_QOS_APE_OPP,
			DEVICE_NAME, 50);
		prcmu_release_usb_wakeup_state();
		regulator_disable(musb_vape_supply);
		regulator_disable(musb_vintcore_supply);
		regulator_set_optimum_mode(musb_vintcore_supply, 0);
		ret = regulator_set_voltage(musb_vintcore_supply,
					    0, 1350000);
		if (ret < 0)
			printk(KERN_ERR "Failed to set the Vintcore"
					" to 0 V .. 1.35 V, ret=%d\n",
					ret);
		regulator_disable(musb_smps2_supply);
		clk_disable(sysclock);
		phy_enable_stat = USB_DISABLE;

		return;
	}
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
		AB8500_MAIN_WDOG_CTRL_REG,
		AB8500_MAIN_WATCHDOG_ENABLE);
	udelay(WATCHDOG_DELAY_US);
	abx500_set_register_interruptible(device,
		AB8500_SYS_CTRL2_BLOCK,
		AB8500_MAIN_WDOG_CTRL_REG,
		(AB8500_MAIN_WATCHDOG_ENABLE
		 | AB8500_MAIN_WATCHDOG_KICK));
	udelay(WATCHDOG_DELAY_US);
	abx500_set_register_interruptible(device,
		AB8500_SYS_CTRL2_BLOCK,
		AB8500_MAIN_WDOG_CTRL_REG,
		AB8500_MAIN_WATCHDOG_DISABLE);
	udelay(WATCHDOG_DELAY_US);
}
/**
 * usb_host_phy_en() - for enabling the 5V to usb host
 * @enable: to enabling the Phy for host.
 *
 * This function used to set the voltage for USB host mode
 */
static void usb_host_phy_en(int enable)
{
	int volt = 0;
	int ret = -1;

	if (enable == USB_ENABLE) {
		wake_lock(&ab8500_musb_wakelock);

		clk_enable(sysclock);
		regulator_enable(musb_vape_supply);
		regulator_enable(musb_smps2_supply);

		/* Set Vintcore12 LDO to 1.3V */
		ret = regulator_set_voltage(musb_vintcore_supply,
						1300000, 1350000);
		if (ret < 0)
			printk(KERN_ERR "Failed to set the Vintcore"
					" to 1.3V, ret=%d\n", ret);
		ret = regulator_set_optimum_mode(musb_vintcore_supply,
						 28000);
		if (ret < 0)
			printk(KERN_ERR "Failed to set optimum mode"
					" (ret=%d)\n", ret);
		regulator_enable(musb_vintcore_supply);
		volt = regulator_get_voltage(musb_vintcore_supply);
		if ((volt != 1300000) && (volt != 1350000))
			printk(KERN_ERR "Vintcore is not"
					" set to 1.3V"
					" volt=%d\n", volt);
#ifdef	CONFIG_PM
		stm_musb_context(USB_ENABLE);
#endif
		usb_kick_watchdog();

		prcmu_qos_update_requirement(PRCMU_QOS_APE_OPP,
				DEVICE_NAME, 100);

		/* Enable the PHY */
		abx500_set_register_interruptible(device,
			AB8500_USB,
			AB8500_USB_PHY_CTRL_REG,
			AB8500_USB_HOST_ENABLE);
	} else { /* enable == USB_DISABLE */
		if (boot_time_flag)
			boot_time_flag = USB_DISABLE;

		/*
		 * Workaround for bug31952 in ABB cut2.0. Write 0x1
		 * before disabling the PHY.
		 */
		abx500_set_register_interruptible(device, AB8500_USB,
			     AB8500_USB_PHY_CTRL_REG,
			     AB8500_USB_HOST_ENABLE);

		udelay(100);

		abx500_set_register_interruptible(device, AB8500_USB,
			     AB8500_USB_PHY_CTRL_REG,
			     AB8500_USB_HOST_DISABLE);

		prcmu_qos_update_requirement(PRCMU_QOS_APE_OPP,
					DEVICE_NAME, 50);
		prcmu_release_usb_wakeup_state();
		regulator_disable(musb_vape_supply);
		regulator_disable(musb_smps2_supply);
		regulator_disable(musb_vintcore_supply);
		regulator_set_optimum_mode(musb_vintcore_supply, 0);
		/* Set Vintcore12 LDO to 0V to 1.35V */
		ret = regulator_set_voltage(musb_vintcore_supply,
						0000000, 1350000);
		if (ret < 0)
			printk(KERN_ERR "Failed to set the Vintcore"
					" to 0V to 1.35V,"
					" ret=%d\n", ret);
		clk_disable(sysclock);
#ifdef CONFIG_PM
		stm_musb_context(USB_DISABLE);
#endif
		wake_unlock(&ab8500_musb_wakelock);
	}
}

/**
 * usb_host_remove_handler() - Removed the USB host cable
 *
 * This function used to detect the USB host cable removed.
 */
static irqreturn_t usb_host_remove_handler(int irq, void *data)
{
	if (stm_musb_curr_state == USB_HOST) {
		/* Change the current state */
		stm_musb_curr_state = USB_IDLE;
		queue_work(usb_cable_wq, &usb_host_remove);
		return IRQ_HANDLED;
	}
	return IRQ_NONE;
}
/**
 * usb_device_phy_en() - for enabling the 5V to usb gadget
 * @enable: to enabling the Phy for device.
 *
 * This function used to set the voltage for USB gadget mode.
 */
static void usb_device_phy_en(int enable)
{
	int volt = 0;
	int ret = -1;

	if (phy_enable_stat == enable)
		return;

	if (enable == USB_ENABLE) {
		wake_lock(&ab8500_musb_wakelock);
		clk_enable(sysclock);
		phy_enable_stat = USB_ENABLE;
		regulator_enable(musb_vape_supply);
		regulator_enable(musb_smps2_supply);

		/* Set Vintcore12 LDO to 1.3V */
		ret = regulator_set_voltage(musb_vintcore_supply,
						1300000, 1350000);
		if (ret < 0)
			printk(KERN_ERR "Failed to set the Vintcore"
					" to 1.3V, ret=%d\n", ret);
		ret = regulator_set_optimum_mode(musb_vintcore_supply,
						 28000);
		if (ret < 0)
			printk(KERN_ERR "Failed to set optimum mode"
					" (ret=%d)\n", ret);
		regulator_enable(musb_vintcore_supply);
		volt = regulator_get_voltage(musb_vintcore_supply);
		if ((volt != 1300000) && (volt != 1350000))
			printk(KERN_ERR "Vintcore is not"
					" set to 1.3V"
					" volt=%d\n", volt);
#ifdef	CONFIG_PM
		stm_musb_context(USB_ENABLE);
#endif
		usb_kick_watchdog();

		/* Workaround for USB performance issue. */
		cpufreq_usb_connect_notify(true);

		prcmu_qos_update_requirement(PRCMU_QOS_APE_OPP,
				DEVICE_NAME, 100);

		abx500_set_register_interruptible(device,
				AB8500_USB,
				AB8500_USB_PHY_CTRL_REG,
				AB8500_USB_DEVICE_ENABLE);
	} else { /* enable == USB_DISABLE */
		if (boot_time_flag)
			boot_time_flag = USB_DISABLE;

		/*
		 * Workaround for bug31952 in ABB cut2.0. Write 0x1
		 * before disabling the PHY.
		 */
		abx500_set_register_interruptible(device, AB8500_USB,
			     AB8500_USB_PHY_CTRL_REG,
			     AB8500_USB_DEVICE_ENABLE);

		udelay(100);

		abx500_set_register_interruptible(device,
			AB8500_USB,
			AB8500_USB_PHY_CTRL_REG,
			AB8500_USB_DEVICE_DISABLE);
		prcmu_qos_update_requirement(PRCMU_QOS_APE_OPP,
				DEVICE_NAME, 50);

		/* Workaround for USB performance issue. */
		cpufreq_usb_connect_notify(false);

		prcmu_release_usb_wakeup_state();
		phy_enable_stat = USB_DISABLE;
		regulator_disable(musb_vape_supply);
		regulator_disable(musb_smps2_supply);
		regulator_disable(musb_vintcore_supply);
		regulator_set_optimum_mode(musb_vintcore_supply, 0);
		/* Set Vintcore12 LDO to 0V to 1.35V */
		ret = regulator_set_voltage(musb_vintcore_supply,
						0000000, 1350000);
		if (ret < 0)
			printk(KERN_ERR "Failed to set the Vintcore"
					" to 0V to 1.35V,"
					" ret=%d\n", ret);
		clk_disable(sysclock);
#ifdef CONFIG_PM
		stm_musb_context(USB_DISABLE);
#endif
		wake_unlock(&ab8500_musb_wakelock);
	}
}

/**
 * usb_device_remove_handler() - remove the 5V to usb device
 *
 * This function used to remove the voltage for USB device mode.
 */
static irqreturn_t usb_device_remove_handler(int irq, void *data)
{
	if (stm_musb_curr_state == USB_DEVICE) {
		/* Change the current state */
		stm_musb_curr_state = USB_IDLE;
		queue_work(usb_cable_wq, &usb_device_remove);
		return IRQ_HANDLED;
	} else if (stm_musb_curr_state == USB_DEDICATED_CHG) {
		stm_musb_curr_state = USB_IDLE;
		if (ab8500_rev == AB8500_REV_20)
			queue_work(usb_cable_wq, &usb_dedicated_charger_remove);
		return IRQ_HANDLED;
	}
	return IRQ_NONE;
}

/**
 * usb_link_status_update_handler() - USB Link status update complete
 *
 * This function is used to signal the completion of
 * USB Link status register update
 */
static irqreturn_t usb_link_status_update_handler(int irq, void *data)
{
	queue_work(usb_cable_wq, &usb_lnk_status_update);
	return IRQ_HANDLED;
}

/* 20111128 check for USB connection */
static int usb_switch_status;

static int usb_switcher_notify(struct notifier_block *self,
	unsigned long action, void *dev)
{
	if (action & (EXTERNAL_USB | EXTERNAL_JIG_USB_MASK)) {
		/* 20111128 check for USB connection */
		usb_switch_status = MUIC_USB_CONNECT_CHECK;
		queue_work(usb_cable_wq, &usb_lnk_status_update);
	} else {
		/* 20111128 check for USB connection */
		usb_switch_status = MUIC_USB_DISCONNECT;
	}
	return NOTIFY_OK ;
}


#ifdef CONFIG_USB_OTG_20
static irqreturn_t irq_adp_plug_handler(int irq, void *data)
{
	int ret;

	ret = musb_adp();
	if (ret) {

		if (stm_musb_curr_state == USB_HOST)
			musb_set_session();
		if (stm_musb_curr_state == USB_DEVICE) {
			/*TODO*/
			/* Generate SRP */
		}
		if (stm_musb_curr_state == USB_IDLE)
			printk(KERN_INFO "No device is connected\n");
	}

	return IRQ_HANDLED;
}

static irqreturn_t irq_adp_unplug_handler(int irq, void *data)
{
	if (stm_musb_curr_state == USB_HOST) {
		stm_musb_curr_state = USB_IDLE;
		queue_work(usb_cable_wq, &usb_host_remove);
	}
	if (stm_musb_curr_state == USB_DEVICE) {
		stm_musb_curr_state = USB_IDLE;
		queue_work(usb_cable_wq, &usb_device_remove);
	}

	return IRQ_HANDLED;
}
#endif

/**
 * musb_phy_en : register USB callback handlers for ab8500
 * @mode: value for mode.
 *
 * This function is used to register USB callback handlers for ab8500.
 */
int musb_phy_en(u8 mode)
{
	int ret = -1;

	if (!device)
		return -EINVAL;

	ab8500_rev = abx500_get_chip_id(device);
	if (ab8500_rev < 0) {
		dev_err(device, "get chip id failed\n");
		return ab8500_rev;
	}
	if (!((ab8500_rev == AB8500_REV_20)
	|| (ab8500_rev == AB8500_REV_30)
	|| (ab8500_rev == AB8500_REV_33))) {
		dev_err(device, "Unknown AB type!\n");
		return -ENODEV;
	}

	musb_vape_supply = regulator_get(device, "v-ape");
	if (IS_ERR(musb_vape_supply)) {
		dev_err(device, "Could not get %s:v-ape supply\n",
				dev_name(device));

		ret = PTR_ERR(musb_vape_supply);
		return ret;
	}
	musb_vintcore_supply = regulator_get(device, "v-intcore");
	if (IS_ERR(musb_vintcore_supply)) {
		dev_err(device, "Could not get %s:v-intcore12 supply\n",
				dev_name(device));

		ret = PTR_ERR(musb_vintcore_supply);
		return ret;
	}
	musb_smps2_supply = regulator_get(device, "musb_1v8");
	if (IS_ERR(musb_smps2_supply)) {
		dev_err(device, "Could not get %s:v-intcore12 supply\n",
				dev_name(device));

		ret = PTR_ERR(musb_smps2_supply);
		return ret;
	}
	sysclock = clk_get(device, "sysclk");
	if (IS_ERR(sysclock)) {
		ret = PTR_ERR(sysclock);
		sysclock = NULL;
		return ret;
	}

	/*
	 * When usb cable is not connected,set Qos for VAPE to 50.
	 * This is done to run APE at low OPP when usb is not used.
	 */
	prcmu_qos_add_requirement(PRCMU_QOS_APE_OPP, DEVICE_NAME, 50);

	if (mode == MUSB_HOST || mode == MUSB_OTG) {
		ret = request_threaded_irq(irq_host_remove, NULL,
			usb_host_remove_handler,
			IRQF_NO_SUSPEND | IRQF_SHARED,
			"usb-host-remove", device);
		if (ret < 0) {
			printk(KERN_ERR "failed to set the callback"
					" handler for usb host"
					" removal\n");
			return ret;
		}
	}
	if ((mode == MUSB_PERIPHERAL) || (mode == MUSB_OTG)) {
		ret = request_threaded_irq(irq_device_remove, NULL,
			usb_device_remove_handler,
			IRQF_NO_SUSPEND | IRQF_SHARED,
			"usb-device-remove", device);
		if (ret < 0) {
			printk(KERN_ERR "failed to set the callback"
					" handler for usb host"
					" removal\n");
			return ret;
		}
	}

	/* create a thread for work */
	usb_cable_wq = create_singlethread_workqueue(
			"usb_cable_wq");
	if (usb_cable_wq == NULL)
		return -ENOMEM;

	INIT_WORK(&usb_host_remove, usb_host_remove_work);
	INIT_WORK(&usb_device_remove, usb_device_remove_work);
	INIT_WORK(&usb_lnk_status_update,
			usb_link_status_update_work);

	if (ab8500_rev == AB8500_REV_20)
		INIT_WORK(&usb_dedicated_charger_remove,
			usb_dedicated_charger_remove_work);

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
#ifdef CONFIG_USB_OTG_20
	ret = request_threaded_irq(irq_adp_plug, NULL,
			irq_adp_plug_handler,
			IRQF_SHARED, "usb-adp-plug", device);
	if (ret < 0) {
		printk(KERN_ERR "failed to set the callback"
			" handler for usb adp"
			" plug\n");
		return ret;
	}
	ret = request_threaded_irq(irq_adp_unplug, NULL,
			irq_adp_unplug_handler,
			IRQF_SHARED, "usb-adp-unplug", device);
	if (ret < 0) {
		printk(KERN_ERR "failed to set the callback"
				" handler for usb adp"
				" unplug\n");
		return ret;
	}
#endif
	/* Write Phy tuning values */
	if ((ab8500_rev == AB8500_REV_30)
	|| (ab8500_rev == AB8500_REV_33)) {
		/* Enable the PBT/Bank 0x12 access */
		ret = abx500_set_register_interruptible(device,
						AB8500_DEVELOPMENT,
						AB8500_BANK12_ACCESS,
						0x01);
		if (ret < 0)
			printk(KERN_ERR "Failed to enable bank12"
					" access ret=%d\n", ret);

		ret = abx500_set_register_interruptible(device,
						AB8500_DEBUG,
						AB8500_USB_PHY_TUNE1,
						0xD8);
		if (ret < 0)
			printk(KERN_ERR "Failed to set PHY_TUNE1"
					" register ret=%d\n", ret);

		ret = abx500_set_register_interruptible(device,
						AB8500_DEBUG,
						AB8500_USB_PHY_TUNE2,
						0x00);
		if (ret < 0)
			printk(KERN_ERR "Failed to set PHY_TUNE2"
					" register ret=%d\n", ret);

		ret = abx500_set_register_interruptible(device,
						AB8500_DEBUG,
						AB8500_USB_PHY_TUNE3,
						0xFC);
		if (ret < 0)
			printk(KERN_ERR "Failed to set PHY_TUNE3"
					" regester ret=%d\n", ret);

		/* Switch to normal mode/disable Bank 0x12 access */
		ret = abx500_set_register_interruptible(device,
						AB8500_DEVELOPMENT,
						AB8500_BANK12_ACCESS,
						0x00);
		if (ret < 0)
			printk(KERN_ERR "Failed to switch bank12"
					" access ret=%d\n", ret);
	}
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
	int ret;
	u8 usb_status = 0;
	u8 val = 0;

	if (!device)
		return -EINVAL;

	/* Disabling PHY before selective enable or disable */
	abx500_set_register_interruptible(device,
			AB8500_USB,
			AB8500_USB_PHY_CTRL_REG,
			AB8500_USB_DEVICE_ENABLE);

	udelay(100);

	abx500_set_register_interruptible(device,
			AB8500_USB,
			AB8500_USB_PHY_CTRL_REG,
			AB8500_USB_DEVICE_DISABLE);

	abx500_set_register_interruptible(device,
			AB8500_USB,
			AB8500_USB_PHY_CTRL_REG,
			AB8500_USB_HOST_ENABLE);

	udelay(100);

	abx500_set_register_interruptible(device,
			AB8500_USB,
			AB8500_USB_PHY_CTRL_REG,
			AB8500_USB_HOST_DISABLE);

	if (mode == MUSB_HOST || mode == MUSB_OTG) {
		ret = abx500_get_register_interruptible(device,
			AB8500_INTERRUPT, AB8500_IT_SOURCE20_REG,
			&usb_status);
		if (ret < 0) {
			dev_err(device, "Read IT 20 failed\n");
			return ret;
		}

		if (usb_status & AB8500_SRC_INT_USB_HOST) {
			boot_time_flag = USB_ENABLE;
			/* Change the current state */
			stm_musb_curr_state = USB_HOST;
			/*+ 373056 +*/
			/* usb_host_phy_en(USB_ENABLE); */
			/*- 373056 -*/
		}
	}
	if (mode == MUSB_PERIPHERAL || mode == MUSB_OTG) {
		ret = abx500_get_register_interruptible(device,
			AB8500_INTERRUPT, AB8500_IT_SOURCE2_REG,
			&usb_status);
		if (ret < 0) {
			dev_err(device, "Read IT 2 failed\n");
			return ret;
		}

		if (usb_status & AB8500_SRC_INT_USB_DEVICE) {
			/* Check if it is a dedicated charger */
			(void)abx500_get_register_interruptible(device,
			AB8500_USB, AB8500_USB_LINE_STAT_REG, &val);

			val = (val & AB8500_USB_LINK_STATUS) >> 3;

			if (val == USB_LINK_DEDICATED_CHG) {
				/* Change the current state */
				stm_musb_curr_state = USB_DEDICATED_CHG;
			} else {
				boot_time_flag = USB_ENABLE;
				/* Change the current state */
				stm_musb_curr_state = USB_DEVICE;
				/*+ 373056 +*/
				/* usb_device_phy_en(USB_ENABLE); */
				/*- 373056 -*/
			}
		}
	}
	return 0;
}

/**
 * usb_host_remove_work : work handler for host cable insert.
 * @work: work structure
 *
 * This function is used to handle the host cable insert work.
 */
static void usb_host_remove_work(struct work_struct *work)
{
	usb_host_phy_en(USB_DISABLE);
}

/**
 * usb_device_remove_work : work handler for device cable insert.
 * @work: work structure
 *
 * This function is used to handle the device cable insert work.
 */
static void usb_device_remove_work(struct work_struct *work)
{
	usb_device_phy_en(USB_DISABLE);
}

/* Work created for PHY handling in case of dedicated charger disconnect */
static void usb_dedicated_charger_remove_work(struct work_struct *work)
{
	/*
	* Workaround for bug31952 in ABB cut2.0. Write 0x1
	* before disabling the PHY.
	*/
	abx500_set_register_interruptible(device, AB8500_USB,
				AB8500_USB_PHY_CTRL_REG,
				AB8500_USB_DEVICE_ENABLE);

	udelay(100);

	abx500_set_register_interruptible(device,
				AB8500_USB,
				AB8500_USB_PHY_CTRL_REG,
				AB8500_USB_DEVICE_DISABLE);
}

/* Work created after an link status update handler*/
static void usb_link_status_update_work(struct work_struct *work)
{
	u8 val = 0;

	/*+ 373056 +*/
	if (boot_time_flag == USB_ENABLE) {
		if (stm_musb_curr_state == USB_HOST)
			usb_host_phy_en(USB_ENABLE);
		else if (stm_musb_curr_state == USB_DEVICE)
			usb_device_phy_en(USB_ENABLE);
		boot_time_flag = USB_DISABLE;
		return;
	}
	/*- 373056 -*/

	(void)abx500_get_register_interruptible(device,
			AB8500_USB, AB8500_USB_LINE_STAT_REG, &val);

	val = (val & AB8500_USB_LINK_STATUS) >> 3;
	stm_usb_link_curr_state = (enum stm_musb_states) val;

/*+ 20120111 check for USB connection +*/
	MSG_DBG("stm_usb_link_curr_state=%d, usb_switch_status=%d\n",
			stm_usb_link_curr_state, usb_switch_status);

	if (usb_switch_status == MUIC_USB_CONNECT_CHECK) {
		if (stm_usb_link_curr_state == USB_LINK_NOT_CONFIGURED) {
			stm_usb_link_curr_state = USB_LINK_STD_HOST_NC;
			MSG_DBG("set the USB link state=%d\n",
				stm_usb_link_curr_state);
		} else {
			usb_switch_status = MUIC_DETECTION_SUCCESS;
		}
	}
/*- 20120111 -*/

	switch (stm_usb_link_curr_state) {
	case USB_LINK_DEDICATED_CHG:
		stm_musb_curr_state = USB_DEDICATED_CHG;
		break;
	case USB_LINK_NOT_CONFIGURED:
	case USB_LINK_NOT_VALID_LINK:
	case USB_LINK_ACA_RID_B:
		/* PHY is disabled */
		switch (stm_musb_curr_state) {
		case USB_DEVICE:
			/* Change the current state */
			stm_musb_curr_state = USB_IDLE;
			usb_device_phy_en(USB_DISABLE);
			break;
		case USB_HOST:
			/* Change the current state */
			stm_musb_curr_state = USB_IDLE;
			usb_host_phy_en(USB_DISABLE);
			break;
		case USB_IDLE:
		case USB_DEDICATED_CHG:
			break;
		}
		break;
	case USB_LINK_STD_HOST_NC:
	case USB_LINK_STD_HOST_C_NS:
	case USB_LINK_STD_HOST_C_S:
	case USB_LINK_HOST_CHG_NM:
	case USB_LINK_HOST_CHG_HS:
	case USB_LINK_HOST_CHG_HS_CHIRP:
	case USB_LINK_ACA_RID_C_NM:
	case USB_LINK_ACA_RID_C_HS:
	case USB_LINK_ACA_RID_C_HS_CHIRP:
		/* Device PHY is enabled */
		switch (stm_musb_curr_state) {
		case USB_HOST:
			/* Change the current state */
			stm_musb_curr_state = USB_DEVICE;
			usb_host_phy_en(USB_DISABLE);
			usb_device_phy_en(USB_ENABLE);
			break;
		case USB_IDLE:
			/* Change the current state */
			stm_musb_curr_state = USB_DEVICE;
			usb_device_phy_en(USB_ENABLE);
			break;
		case USB_DEVICE:
		case USB_DEDICATED_CHG:
			break;
		}
		break;
	case USB_LINK_HM_IDGND:
	case USB_LINK_ACA_RID_A:
		/* Host PHY is enabled */
		switch (stm_musb_curr_state) {
		case USB_DEVICE:
			/* Change the current state */
			stm_musb_curr_state = USB_HOST;
			usb_device_phy_en(USB_DISABLE);
			usb_host_phy_en(USB_ENABLE);
			musb_set_session();
			break;
		case USB_IDLE:
			/* Change the current state */
			stm_musb_curr_state = USB_HOST;
			usb_host_phy_en(USB_ENABLE);
			musb_set_session();
			break;
		case USB_HOST:
		case USB_DEDICATED_CHG:
			break;
		}
		break;
	case USB_LINK_OTG_HOST_NO_CURRENT:
	default:
		break;
	}
	stm_usb_link_prev_state = stm_usb_link_curr_state;
}

/*
 * musb_get_abx500_rev : Get the ABx500 revision number
 *
 * This function returns the ABx500 revision number.
 */
int musb_get_abx500_rev()
{
	return ab8500_rev;
}

static int __devinit ab8500_musb_probe(struct platform_device *pdev)
{
#ifdef CONFIG_USB_OTG_20
	u8 usb_status = 0;
	int ret;
#endif
	device = &pdev->dev;
	irq_host_remove = platform_get_irq_byname(pdev, "ID_WAKEUP_F");
	if (irq_host_remove < 0) {
		dev_err(&pdev->dev, "Host remove irq not found, err %d\n",
			irq_host_remove);
		return irq_host_remove;
	}

	irq_device_remove = platform_get_irq_byname(pdev, "VBUS_DET_F");
	if (irq_device_remove < 0) {
		dev_err(&pdev->dev, "Device remove irq not found, err %d\n",
			irq_device_remove);
		return irq_device_remove;
	}

	irq_link_status_update = platform_get_irq_byname(pdev,
		"USB_LINK_STATUS");
	if (irq_link_status_update < 0) {
		dev_err(&pdev->dev, "USB Link status irq not found, err %d\n",
			irq_link_status_update);
		return irq_link_status_update;
	}

	notifier_block.notifier_call = usb_switcher_notify ;
	usb_switch_register_notify( &notifier_block);
#ifdef CONFIG_USB_OTG_20
	enable_adp();
	irq_adp_plug = platform_get_irq_byname(pdev, "USB_ADP_PROBE_PLUG");
	if (irq_adp_plug < 0) {
		dev_err(&pdev->dev, "ADP Probe plug irq not found, err %d\n",
					irq_adp_plug);
		return irq_adp_plug;
	}
	irq_adp_unplug = platform_get_irq_byname(pdev, "USB_ADP_PROBE_UNPLUG");
	if (irq_adp_unplug < 0) {
			dev_err(&pdev->dev, "ADP Probe unplug irq not found,"
						" err %d\n",
						irq_adp_unplug);
		return irq_adp_unplug;
	}
	ret = abx500_get_register_interruptible(device,
				AB8500_INTERRUPT, AB8500_IT_SOURCE20_REG,
				&usb_status);
			if (ret < 0) {
				dev_err(device, "Read IT 2 failed\n");
				return ret;
			}

			if (usb_status & AB8500_USB_DEVICE_ENABLE) {
				boot_time_flag = USB_ENABLE;
				stm_musb_curr_state = USB_DEVICE;
				musb_phy_en(MUSB_HOST);
			}
#endif

	wake_lock_init(&ab8500_musb_wakelock, WAKE_LOCK_SUSPEND, "ab8500-usb");
	return 0;
}

static int __devexit ab8500_musb_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver ab8500_musb_driver = {
	.driver		= {
		.name	= "ab8500-usb",
		.owner	= THIS_MODULE,
	},
	.probe		= ab8500_musb_probe,
	.remove		= __devexit_p(ab8500_musb_remove),
};

static int __init ab8500_musb_init(void)
{
	return platform_driver_register(&ab8500_musb_driver);
}
module_init(ab8500_musb_init);

static void __exit ab8500_musb_exit(void)
{
	platform_driver_unregister(&ab8500_musb_driver);
}
module_exit(ab8500_musb_exit);

MODULE_LICENSE("GPL v2");
