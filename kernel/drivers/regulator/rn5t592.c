/*
 * Linux device driver for RN5T592 power management IC
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2011 Samsung Electronics Co. Ltd
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


/*
 * Notes:
 *  Currently, either both cameras are powered on or none. There is currently
 * no way to power on one camera but not the other.
 */


#include <linux/module.h>	/* Kernel module header */
#include <linux/kernel.h>	/* printk(), snprintf() */
#include <linux/init.h>		/* __init, __exit, __devinit, __devexit */
#include <linux/string.h>	/* memset() */
#include <linux/mutex.h>	/* DEFINE_MUTEX(), mutex_[un]lock() */
#include <linux/i2c.h>		/* struct i2c_client, i2c_*() */
#include <linux/gpio.h>		/* gpio_*() */
#include <linux/pm.h>		/* struct dev_pm_ops */
#include <linux/fs.h>		/* struct file_operations */
#include <linux/miscdevice.h>	/* struct miscdevice, misc_[de]register() */
#include <linux/mod_devicetable.h> /* MODULE_DEVICE_TABLE() */
#include <linux/sysfs.h>	/* sysfs stuff */
#include <mach/rn5t592.h>	/* RN5T592 platform-specific stuff */

#include "rn5t592_ioctl.h"	/* RN5T592 ioctl interface */



/* -------------------------------------------------------------------------
 * MACROS, TYPES AND VARIABLES
 * ------------------------------------------------------------------------- */


/* Uncomment the next line to enable debug prints */
//#define	RN5T592_DEBUG


#if !defined(RN5T592_DEBUG) && defined(DEBUG)
#define	RN5T592_DEBUG
#endif

#ifdef RN5T592_DEBUG
#define	rndbg(_fmt, ...)	\
	printk("RN5T592 DEBUG: " _fmt "\n", ## __VA_ARGS__)
#else
#define	rndbg(_fmt, ...)
#endif

#define	rninfo(_fmt, ...)	\
	printk("RN5T592 INFO: " _fmt "\n", ## __VA_ARGS__)

#define	rnwarn(_fmt, ...)	\
	printk("RN5T592 WARN: " _fmt "\n", ## __VA_ARGS__)

#define	rnerr(_fmt, ...)	\
	printk("RN5T592 ERROR: " _fmt "\n", ## __VA_ARGS__)


/* Register map */
#define	RN5T592_REG_REGEN	0x00
#define	RN5T592_REG_REGDIS	0x01
#define	RN5T592_REG_REGCTL	0x02
#define	RN5T592_REG_LDORDT	0x03
#define	RN5T592_REG_LDOFDT	0x04
#define	RN5T592_REG_ISPCTL	0x05


/* Useful bit values and masks */

#define	RN5T592_REG_REGEN_BIT_LDO1EN		0x01
#define	RN5T592_REG_REGEN_BIT_LDO2EN		0x02
#define	RN5T592_REG_REGEN_BIT_LDO3EN		0x04
#define	RN5T592_REG_REGEN_BIT_LDO4EN		0x08
#define	RN5T592_REG_REGEN_BIT_LDO5EN		0x10
#define	RN5T592_REG_REGEN_BIT_LDO6EN		0x20
#define	RN5T592_REG_REGEN_BIT_DC1EN		0x80

#define	RN5T592_REG_REGDIS_BIT_LDO1DIS		0x01
#define	RN5T592_REG_REGDIS_BIT_LDO2DIS		0x02
#define	RN5T592_REG_REGDIS_BIT_LDO3DIS		0x04
#define	RN5T592_REG_REGDIS_BIT_LDO4DIS		0x08
#define	RN5T592_REG_REGDIS_BIT_LDO5DIS		0x10
#define	RN5T592_REG_REGDIS_BIT_LDO6DIS		0x20
#define	RN5T592_REG_REGDIS_BIT_DC1DIS		0x80

#define	RN5T592_REG_REGCTL_MSK_DC1V		0x1f
/* NB: valid values for "_adjustment_over_1V_in_mV" are between 0 and 310 included */
#define	RN5T592_REG_REGCTL_VAL_DC1V(_adjustment_over_1V_in_mV)	\
	( (_adjustment_over_1V_in_mV) / 10 )

#define	RN5T592_REG_REGCTL_MSK_DC1FDT		0x60
/* NB: valid values for "_value_in_us" are: 0, 200, 400, 600 */
#define	RN5T592_REG_REGCTL_VAL_DC1FDT(_value_in_us)	\
	( ( (_value_in_us) / 200 ) << 5 )

#define	RN5T592_REG_REGCTL_MSK_LDO1V		0x80
#define	RN5T592_REG_REGCTL_VAL_LDO1V_2850mV	0x00
#define	RN5T592_REG_REGCTL_VAL_LDO1V_1800mV	0x80

/* NB: valid values for "_value_in_us" are: 200, 400, 600, 800 */
#define	RN5T592_REG_LDORDT_MSK_LDO1RDT		0x03
#define	RN5T592_REG_LDORDT_VAL_LDO1RDT(_value_in_us)	\
	( ( ((_value_in_us) - 200) / 200 ) << 0 )
#define	RN5T592_REG_LDORDT_MSK_LDO2RDT		0x0c
#define	RN5T592_REG_LDORDT_VAL_LDO2RDT(_value_in_us)	\
	( ( ((_value_in_us) - 200) / 200 ) << 2 )
#define	RN5T592_REG_LDORDT_MSK_LDO3RDT		0x30
#define	RN5T592_REG_LDORDT_VAL_LDO3RDT(_value_in_us)	\
	( ( ((_value_in_us) - 200) / 200 ) << 4 )
#define	RN5T592_REG_LDORDT_MSK_LDO4RDT		0xc0
#define	RN5T592_REG_LDORDT_VAL_LDO4RDT(_value_in_us)	\
	( ( ((_value_in_us) - 200) / 200 ) << 6 )

/* NB: valid values for "_value_in_us" are: 0, 200, 400, 600 */
#define	RN5T592_REG_LDOFDT_MSK_LDO1FDT		0x03
#define	RN5T592_REG_LDOFDT_VAL_LDO1FDT(_value_in_us)	\
	( ((_value_in_us) / 200) << 0 )
#define	RN5T592_REG_LDOFDT_MSK_LDO2FDT		0x0c
#define	RN5T592_REG_LDOFDT_VAL_LDO2FDT(_value_in_us)	\
	( ((_value_in_us) / 200) << 2 )
#define	RN5T592_REG_LDOFDT_MSK_LDO3FDT		0x30
#define	RN5T592_REG_LDOFDT_VAL_LDO3FDT(_value_in_us)	\
	( ((_value_in_us) / 200) << 4 )
#define	RN5T592_REG_LDOFDT_MSK_LDO4FDT		0xc0
#define	RN5T592_REG_LDOFDT_VAL_LDO4FDT(_value_in_us)	\
	( ((_value_in_us) / 200) << 6 )

#define	RN5T592_REG_ISPCTL_BIT_RESET_PIN	0x01
#define	RN5T592_REG_ISPCTL_BIT_STBY1_PIN	0x02
#define	RN5T592_REG_ISPCTL_BIT_STBY2_PIN	0x04
#define	RN5T592_REG_ISPCTL_MSK_MODESEL		0x08
#define	RN5T592_REG_ISPCTL_VAL_MODESEL_HW	0x00
#define	RN5T592_REG_ISPCTL_VAL_MODESEL_SW	0x08
#define	RN5T592_REG_ISPCTL_MSK_RESORDT		0x30
/* NB: valid values for "_value_in_us" are: 40, 60, 80, 100 */
#define	RN5T592_REG_ISPCTL_VAL_RESORDT(_value_in_us)	\
	( ( ((_value_in_us) - 40) / 20 ) << 4 )


/* Device structure */
struct rn5t592 {
	int			is_powered_on;
	unsigned int		gpio_power_on;
	struct i2c_client*	client;
	int			is_sysfs_status_created;
};



/* -------------------------------------------------------------------------
 * DRIVER DECLARATIONS
 * ------------------------------------------------------------------------- */

static DEFINE_MUTEX(rn5t592_mutex);
static struct rn5t592	rn5t592;
static int		rn5t592_is_created = 0;

/*
 * All the _rn5t592_dev_*_locked() function are not MT-safe and must be called
 * with the mutex locked.
 */
static int _rn5t592_dev_configure_locked(void);

/*
 * All the rn5t592_dev_*() function are MT-safe and must be called with the
 * mutex unlocked.
 */
static int rn5t592_dev_create(struct i2c_client* client);
static void rn5t592_dev_destroy(void);
static int rn5t592_dev_poweron(void);
static int rn5t592_dev_poweroff(void);



/* -------------------------------------------------------------------------
 * I2C DRIVER DECLARATIONS
 * ------------------------------------------------------------------------- */

static int rn5t592_i2c_write(struct i2c_client* client, u8 reg, u8 val);
static int rn5t592_i2c_probe(struct i2c_client* client,
		const struct i2c_device_id* id) __devinit;
static int rn5t592_i2c_remove(struct i2c_client* client) __devexit;

#ifdef CONFIG_PM
static int rn5t592_i2c_suspend(struct device* dev);
static int rn5t592_i2c_resume(struct device* dev);

static const struct dev_pm_ops rn5t592_pm_ops = {
	.suspend = rn5t592_i2c_suspend,
	.resume = rn5t592_i2c_resume,
};
#endif

static const struct i2c_device_id rn5t592_i2c_idtable[] = {
	{ RN5T592_I2C_DEVICE_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, rn5t592_i2c_idtable);

static struct i2c_driver rn5t592_i2c_driver = {
	.driver = {
		/* This should be the same as the module name */
		.name = "rn5t592",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &rn5t592_pm_ops,
#endif
	},
	.id_table = rn5t592_i2c_idtable,
	.probe = rn5t592_i2c_probe,
	.remove = rn5t592_i2c_remove,
};



/* -------------------------------------------------------------------------
 * MISC DRIVER DECLARATIONS
 * ------------------------------------------------------------------------- */

static int rn5t592_misc_open(struct inode* inode, struct file* filp);
static int rn5t592_misc_release(struct inode* inode, struct file* filp);
static int rn5t592_misc_ioctl(struct inode* inode, struct file* filp,
		unsigned int cmd, unsigned long arg);

static struct file_operations rn5t592_misc_ops = {
	.owner = THIS_MODULE,
	.open = rn5t592_misc_open,
	.release = rn5t592_misc_release,
	.ioctl = rn5t592_misc_ioctl,
};

static struct miscdevice rn5t592_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = RN5T592_DEVNAME,
	.fops = &rn5t592_misc_ops,
};



/* -------------------------------------------------------------------------
 * SYSFS ENTRIES DECLARATIONS
 * ------------------------------------------------------------------------- */

static ssize_t rn5t592_sysfs_show_status(struct device* dev,
		struct device_attribute* attr, char* buf);

#if defined(RN5T592_DEBUG)
static ssize_t rn5t592_sysfs_store_status(struct device* dev,
		struct device_attribute* attr, const char* buf, size_t len);

static DEVICE_ATTR(status, S_IRUGO | S_IWUSR,
		rn5t592_sysfs_show_status, rn5t592_sysfs_store_status);

#else
static DEVICE_ATTR(status, S_IRUGO, rn5t592_sysfs_show_status, NULL);
#endif



/* -------------------------------------------------------------------------
 * DRIVER IMPLEMENTATION
 * ------------------------------------------------------------------------- */


/*
 * NB: Configuration should really be done in user-land, the driver should
 * only offer access to the RN5T592.
 * But because we don't have time, configuration is hardcoded here. Ideally,
 * the driver should allow for the device to be configured via ioctl() calls.
 */
static int _rn5t592_dev_configure_locked(void)
{
	int	ret;
	u8	reg;
	u8	val;

	/* Enable all LDOs and the DCDC */
	reg = RN5T592_REG_REGEN;
	val =     RN5T592_REG_REGEN_BIT_LDO1EN
		| RN5T592_REG_REGEN_BIT_LDO2EN
		| RN5T592_REG_REGEN_BIT_LDO3EN
		| RN5T592_REG_REGEN_BIT_LDO4EN
		| RN5T592_REG_REGEN_BIT_LDO5EN
		| RN5T592_REG_REGEN_BIT_LDO6EN
		| RN5T592_REG_REGEN_BIT_DC1EN;
	ret = rn5t592_i2c_write(rn5t592.client, reg, val);
	if (ret < 0) {
		return ret;
	}

	/* Do not touch the REGDIS discharge things */
	reg = RN5T592_REG_REGDIS;
	val =     RN5T592_REG_REGDIS_BIT_LDO1DIS
		| RN5T592_REG_REGDIS_BIT_LDO2DIS
		| RN5T592_REG_REGDIS_BIT_LDO3DIS
		| RN5T592_REG_REGDIS_BIT_LDO4DIS
		| RN5T592_REG_REGDIS_BIT_LDO5DIS
		| RN5T592_REG_REGDIS_BIT_LDO6DIS
		| RN5T592_REG_REGDIS_BIT_DC1DIS;
	ret = rn5t592_i2c_write(rn5t592.client, reg, val);
	if (ret < 0) {
		return ret;
	}

	/*
	 * Set REGCTL for the main camera:
	 *  - DCDC (CAM_CORE_1.2V) voltage is 1.2V
	 *  - DCDC (CAM_CORE_1.2V) is turned off after all the others
	 *  - LDO1 (5M_VDDIO_1.8V) voltage is 1.8V
	 */
	reg = RN5T592_REG_REGCTL;
	val = RN5T592_REG_REGCTL_VAL_DC1FDT(600)
		| RN5T592_REG_REGCTL_VAL_LDO1V_1800mV;
	ret = rn5t592_i2c_write(rn5t592.client, reg, val);
	if (ret < 0) {
		return ret;
	}

	/*
	 * Set LDORDT for the main camera:
	 *  - LDO1 (5M_VDDIO_1.8V) rising delay timing is > 15us after LDO6
	 *    and LDO6 rising delay timing is fixed at 200us
	 *
	 * Set LDORDT for the secondary camera:
	 *  - LDO4 (VT_AVDD_2.8V) is turned on first
	 *  - LDO2 (VT_DVDD_1.5V) rising delay timing is > 20us after LDO4
	 *  - LDO3 (VT_VDDIO_1.8V) rising delay timing is > 15us after LDO2
	 */
	reg = RN5T592_REG_LDORDT;
	val =     RN5T592_REG_LDORDT_VAL_LDO1RDT(400)
		| RN5T592_REG_LDORDT_VAL_LDO4RDT(200)
		| RN5T592_REG_LDORDT_VAL_LDO2RDT(400)
		| RN5T592_REG_LDORDT_VAL_LDO3RDT(600);
	ret = rn5t592_i2c_write(rn5t592.client, reg, val);
	if (ret < 0) {
		return ret;
	}

	/*
	 * Set LDOFDT for the main camera:
	 *  - LDO1 (5M_VDDIO_1.8V) is turned off first
	 *
	 * Set LDOFDT for the secondary camera:
	 *  - LDO3 (VT_VDDIO_1.8V) is turned off first
	 *  - LDO2 (VT_DVDD_1.5V) is turned off > 0us after LDO3
	 *  - LDO4 (VT_AVDD_2.8V) is turned off > 0us after LDO2
	 */
	reg = RN5T592_REG_LDOFDT;
	val =     RN5T592_REG_LDOFDT_VAL_LDO1FDT(0)
		| RN5T592_REG_LDOFDT_VAL_LDO3FDT(0)
		| RN5T592_REG_LDOFDT_VAL_LDO2FDT(200)
		| RN5T592_REG_LDOFDT_VAL_LDO4FDT(400);
	ret = rn5t592_i2c_write(rn5t592.client, reg, val);
	if (ret < 0) {
		return ret;
	}

	/* ISPCTL is useless because the controlled pins are not wired */


	return ret;
}


static int rn5t592_dev_create(struct i2c_client* client)
{
	int				ret;
	struct rn5t592_platform_data*	platdata = client->dev.platform_data;

	rndbg("-> %s()", __func__);
	mutex_lock(&rn5t592_mutex);

	if (rn5t592_is_created) {
		rnwarn("RN5T592 already created");
		ret = 0;
		goto out;
	}

	if (!platdata || !gpio_is_valid(platdata->subpmu_pwron_gpio)) {
		rnerr("No GPIO configured for device \"%s\"", client->name);
		ret = -ENODEV;
		goto out;
	}

	memset(&rn5t592, 0, sizeof(rn5t592));
	rn5t592.client = client;
	rn5t592.gpio_power_on = platdata->subpmu_pwron_gpio;

	ret = gpio_request(rn5t592.gpio_power_on, "SUBPMU_PWRON");
	if (ret < 0) {
		rnerr("Failed to request SUBPMU_PWRON GPIO %d [errno=%d]",
				rn5t592.gpio_power_on, ret);
		goto out;
	}

	ret = _rn5t592_dev_configure_locked();
	if (ret < 0) {
		rnerr("Failed to configure RN5T592 [errno=%d]", ret);
		gpio_free(rn5t592.gpio_power_on);
		goto out;
	}

	ret = device_create_file(&rn5t592.client->dev, &dev_attr_status);
	if (ret < 0) {
		rnwarn("Failed to create sysfs \"%s\" attribute [errno=%d]",
				dev_attr_status.attr.name, ret);
		ret = 0;
	} else {
		rn5t592.is_sysfs_status_created = 1;
	}

	rninfo("RN5T592 successfully initialized (gpio_power_on=%d)",
			rn5t592.gpio_power_on);
	rn5t592_is_created = 1;

out :
	mutex_unlock(&rn5t592_mutex);
	rndbg("<- %s() = %d", __func__, ret);
	return ret;
}


static void rn5t592_dev_destroy(void)
{
	rndbg("-> %s()", __func__);

	rn5t592_dev_poweroff();

	mutex_lock(&rn5t592_mutex);

	if (!rn5t592_is_created) {
		rnwarn("RN5T592 already destroyed");
		goto out;
	}

	if (rn5t592.is_sysfs_status_created) {
		device_remove_file(&rn5t592.client->dev, &dev_attr_status);
	}

	gpio_free(rn5t592.gpio_power_on);

	rn5t592_is_created = 0;
	rninfo("RN5T592 successfully de-initialized");

out :
	mutex_unlock(&rn5t592_mutex);
	rndbg("<- %s()", __func__);
}


static int rn5t592_dev_poweron(void)
{
	int	ret;

	rndbg("-> %s()", __func__);
	mutex_lock(&rn5t592_mutex);

	if (!rn5t592_is_created) {
		rnerr("RN5T592 destroyed");
		ret = -ENODEV;
		goto out;
	}

	if (rn5t592.is_powered_on) {
		ret = 0;
		goto out; /* Device is already powered on */
	}

	gpio_set_value(rn5t592.gpio_power_on, GPIO_HIGH);
	rn5t592.is_powered_on = 1;
	ret = 0;
	rninfo("RN5T592 powered on");
out :
	mutex_unlock(&rn5t592_mutex);
	rndbg("<- %s() = %d", __func__, ret);
	return ret;
}


static int rn5t592_dev_poweroff(void)
{
	int	ret;

	rndbg("-> %s()", __func__);
	mutex_lock(&rn5t592_mutex);

	if (!rn5t592_is_created) {
		rnerr("RN5T592 destroyed");
		ret = -ENODEV;
		goto out;
	}

	if (!rn5t592.is_powered_on) {
		ret = 0;
		goto out; /* Device is already powered off */
	}

	gpio_set_value(rn5t592.gpio_power_on, GPIO_LOW);
	rn5t592.is_powered_on = 0;
	ret = 0;
	rninfo("RN5T592 powered off");

out :
	mutex_unlock(&rn5t592_mutex);
	rndbg("<- %s() = %d", __func__, ret);
	return ret;
}



/* -------------------------------------------------------------------------
 * I2C DRIVER IMPLEMENTATION
 * ------------------------------------------------------------------------- */


static int _rn5t592_i2c_send(struct i2c_client* client, const u8* data, int len)
{
	int	ret;

	if (len <= 0) {
		rnerr("%s(): invalid length %d", __func__, len);
		return -EINVAL;
	}

	ret = i2c_master_send(client, data, len);
	if (ret < 0) {
		rnerr("Failed to send %d bytes to RN5T592 [errno=%d]",
				len, ret);
	} else if (ret != len) {
		rnerr("Failed to send exactly %d bytes to RN5T592 (send %d)",
				len, ret);
		ret = -EIO;
	} else {
		ret = 0;
	}
	return ret;
}


static int rn5t592_i2c_write(struct i2c_client* client, u8 reg, u8 val)
{
	u8	buf[2];

	buf[0] = reg;
	buf[1] = val;

	return _rn5t592_i2c_send(client, buf, 2);
}


static int __devinit rn5t592_i2c_probe(struct i2c_client* client,
		const struct i2c_device_id* id)
{
	int	ret;

	rndbg("-> %s(client=%s, id=%s)", __func__, client->name, id->name);

	dev_set_name(&client->dev, client->name);

	ret = misc_register(&rn5t592_misc);
	if (ret < 0) {
		rnerr("Failed to register RN5T592 misc device [errno=%d]", ret);
	} else {
		ret = rn5t592_dev_create(client);
	}

	rndbg("<- %s(client=%s) = %d", __func__, client->name, ret);
	return ret;
}


static int __devexit rn5t592_i2c_remove(struct i2c_client* client)
{
	rndbg("-> %s(client=%s)", __func__, client->name);

	misc_deregister(&rn5t592_misc);
	rn5t592_dev_destroy();

	rndbg("<- %s(client=%s) = 0", __func__, client->name);
	return 0;
}


#ifdef CONFIG_PM

/*
 * NB: When going to sleep, Android would normally close all camera services,
 * so the next 2 functions are normally never called; they are added just for
 * compliance with kernel standards.
 */

static int rn5t592_i2c_suspend(struct device* dev)
{
	int	ret;

	rndbg("-> %s()", __func__);

	ret = rn5t592_dev_poweroff();

	rndbg("<- %s() = %d", __func__, ret);
	return ret;
}


static int rn5t592_i2c_resume(struct device* dev)
{
	int	ret;

	rndbg("-> %s()", __func__);

	ret = rn5t592_dev_poweron();

	rndbg("<- %s() = %d", __func__, ret);
	return ret;
}

#endif /* defined(CONFIG_PM) */



/* -------------------------------------------------------------------------
 * MISC DRIVER IMPLEMENTATION
 * ------------------------------------------------------------------------- */


static int rn5t592_misc_open(struct inode* inode, struct file* filp)
{
	int	ret;

	rndbg("-> %s()", __func__);

	ret = nonseekable_open(inode, filp);

	rndbg("<- %s() = %d", __func__, ret);
	return ret;
}


static int rn5t592_misc_release(struct inode* inode, struct file* filp)
{
	rndbg("-> %s()", __func__);

	rndbg("<- %s() = 0", __func__);
	return 0;
}


static int rn5t592_misc_ioctl(struct inode* inode, struct file* filp,
		unsigned int cmd, unsigned long arg)
{
	int	ret;

	rndbg("-> %s(cmd=0x%08x)", __func__, cmd);

	switch (cmd) {

	case RN5T592_IOC_TURN_ON :
		ret = rn5t592_dev_poweron();
		break;

	case RN5T592_IOC_TURN_OFF :
		ret = rn5t592_dev_poweroff();
		break;

	default :
		ret = -ENOTTY;
	}
	rndbg("<- %s() = %d", __func__, ret);
	return ret;
}



/* -------------------------------------------------------------------------
 * SYSFS ENTRIES IMPLEMENTATION
 * ------------------------------------------------------------------------- */


static ssize_t rn5t592_sysfs_show_status(struct device* dev,
		struct device_attribute* attr, char* buf)
{
	int	val = 0;

	mutex_lock(&rn5t592_mutex);
	if (rn5t592_is_created) {
		val = rn5t592.is_powered_on;
	}
	mutex_unlock(&rn5t592_mutex);

	return snprintf(buf, PAGE_SIZE, "%d\n", val);
}


#if defined(RN5T592_DEBUG)
static ssize_t rn5t592_sysfs_store_status(struct device* dev,
		struct device_attribute* attr, const char* buf, size_t len)
{
	if (buf[0] == '0') {
		rn5t592_dev_poweroff();
	} else {
		rn5t592_dev_poweron();
	}

	return len;
}
#endif



/* -------------------------------------------------------------------------
 * MODULE STUFF
 * ------------------------------------------------------------------------- */


static int __init rn5t592_module_init(void)
{
	int	ret;

	ret = i2c_add_driver(&rn5t592_i2c_driver);
	if (ret < 0) {
		rnerr("Failed to add i2c driver for RN5T592 [errno=%d]", ret);
	}

	return ret;
}


static void __exit rn5t592_module_exit(void)
{
	i2c_del_driver(&rn5t592_i2c_driver);
}


module_init(rn5t592_module_init);
module_exit(rn5t592_module_exit);

MODULE_AUTHOR("Fabrice Triboix <f.triboix@partner.samsung.com>");
MODULE_DESCRIPTION("Driver for the RN5T592 power management IC");
MODULE_LICENSE("GPL");
