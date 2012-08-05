
#include <linux/delay.h>
#include <linux/init.h>		/* Initiliasation support */
#include <linux/module.h>	/* Module support */
#include <linux/kernel.h>	/* Kernel support */
#include <linux/version.h>	/* Kernel version */
#include <linux/fs.h>		/* File operations (fops) defines */
#include <linux/errno.h>	/* Defines standard err codes */
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/mmio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>
#include <mach/prcmu.h>
//godin+
#include <linux/gpio.h>
#include <linux/i2c.h>		/* struct i2c_client, i2c_*() */
//godin-

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

#define	RN5T592_DEVNAME		"rn5t592"

static unsigned int gpio_power_on;
static struct i2c_client *pClient;

struct rn5t592_platform_data {
	unsigned int subpmu_pwron_gpio;
};

static const struct i2c_device_id rn5t592_i2c_idtable[] = {
	{RN5T592_DEVNAME, 0},
	{}
};

static int rn5t592_dev_poweron(void)
{
	int ret = 0;
	gpio_set_value(gpio_power_on, GPIO_HIGH);
	return ret;
}

static int rn5t592_dev_poweroff(void)
{
	int ret = 0;
	gpio_set_value(gpio_power_on, GPIO_LOW);
	return ret;
}

#ifdef CONFIG_PM
static int rn5t592_i2c_suspend(struct device *dev)
{
	int ret = 0;

	printk("-> %s()", __func__);

	ret = rn5t592_dev_poweroff();

	printk("<- %s() = %d", __func__, ret);
	return ret;
}

static int rn5t592_i2c_resume(struct device *dev)
{
	int ret = 0;

	printk("-> %s()", __func__);

	ret = rn5t592_dev_poweron();

	printk("<- %s() = %d", __func__, ret);
	return ret;
}

static const struct dev_pm_ops rn5t592_pm_ops = {
	.suspend = rn5t592_i2c_suspend,
	.resume = rn5t592_i2c_resume,
};

#endif

static int __devinit rn5t592_i2c_probe(struct i2c_client *client,
				       const struct i2c_device_id *id)
{
	int ret = 0;
	struct rn5t592_platform_data *platdata = client->dev.platform_data;

	printk("-> %s(client=%s, id=%s)", __func__, client->name, id->name);

	dev_set_name(&client->dev, client->name);
	pClient = client;

	gpio_power_on = platdata->subpmu_pwron_gpio;

	printk("<- %s(client=%s) = %d", __func__, client->name, ret);
	return ret;
}

static int __devexit rn5t592_i2c_remove(struct i2c_client *client)
{
	printk("-> %s(client=%s)", __func__, client->name);

	printk("<- %s(client=%s) = 0", __func__, client->name);
	return 0;
}

static struct i2c_driver subPMIC_i2c_driver = {
	.driver = {
		   /* This should be the same as the module name */
		   .name = RN5T592_DEVNAME,
		   .owner = THIS_MODULE,
#ifdef CONFIG_PM
		   .pm = &rn5t592_pm_ops,
#endif
		   },
	.id_table = rn5t592_i2c_idtable,
	.probe = rn5t592_i2c_probe,
	.remove = rn5t592_i2c_remove,
};

static int _rn5t592_i2c_send(struct i2c_client *client, const u8 * data,
			     int len)
{
	int ret = 0;

	if (len <= 0) {
		printk("%s(): invalid length %d", __func__, len);
		return -EINVAL;
	}

	ret = i2c_master_send(client, data, len);
	if (ret < 0) {
		printk("Failed to send %d bytes to RN5T592 [errno=%d]",
		       len, ret);
	} else if (ret != len) {
		printk("Failed to send exactly %d bytes to RN5T592 (send %d)",
		       len, ret);
		ret = -EIO;
	} else {
		ret = 0;
	}
	return ret;
}

static int rn5t592_i2c_write(struct i2c_client *client, u8 reg, u8 val)
{
	u8 buf[2];

	buf[0] = reg;
	buf[1] = val;

	return _rn5t592_i2c_send(client, buf, 2);
}

static int _rn5t592_dev_configure_locked(void)
{
	int ret = 0;
	u8 reg;
	u8 val;

	/* Enable all LDOs and the DCDC */
	reg = RN5T592_REG_REGEN;
	val = RN5T592_REG_REGEN_BIT_LDO1EN
	    | RN5T592_REG_REGEN_BIT_LDO2EN
	    | RN5T592_REG_REGEN_BIT_LDO3EN
	    | RN5T592_REG_REGEN_BIT_LDO4EN
	    | RN5T592_REG_REGEN_BIT_LDO5EN
	    | RN5T592_REG_REGEN_BIT_LDO6EN | RN5T592_REG_REGEN_BIT_DC1EN;
	ret = rn5t592_i2c_write(pClient, reg, val);
	if (ret < 0) {
		return ret;
	}

	/* Do not touch the REGDIS discharge things */
	reg = RN5T592_REG_REGDIS;
	val = RN5T592_REG_REGDIS_BIT_LDO1DIS
	    | RN5T592_REG_REGDIS_BIT_LDO2DIS
	    | RN5T592_REG_REGDIS_BIT_LDO3DIS
	    | RN5T592_REG_REGDIS_BIT_LDO4DIS
	    | RN5T592_REG_REGDIS_BIT_LDO5DIS
	    | RN5T592_REG_REGDIS_BIT_LDO6DIS | RN5T592_REG_REGDIS_BIT_DC1DIS;
	ret = rn5t592_i2c_write(pClient, reg, val);
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
	ret = rn5t592_i2c_write(pClient, reg, val);
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
	val = RN5T592_REG_LDORDT_VAL_LDO1RDT(400)
	    | RN5T592_REG_LDORDT_VAL_LDO4RDT(200)
	    | RN5T592_REG_LDORDT_VAL_LDO2RDT(400)
	    | RN5T592_REG_LDORDT_VAL_LDO3RDT(600);
	ret = rn5t592_i2c_write(pClient, reg, val);
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
	val = RN5T592_REG_LDOFDT_VAL_LDO1FDT(0)
	    | RN5T592_REG_LDOFDT_VAL_LDO3FDT(0)
	    | RN5T592_REG_LDOFDT_VAL_LDO2FDT(200)
	    | RN5T592_REG_LDOFDT_VAL_LDO4FDT(400);
	ret = rn5t592_i2c_write(pClient, reg, val);
	if (ret < 0) {
		return ret;
	}

	/* ISPCTL is useless because the controlled pins are not wired */

	return ret;
}

int subPMIC_module_init(void)
{
	int ret = 0;

	ret = i2c_add_driver(&subPMIC_i2c_driver);
	if (ret < 0) {
		printk("Failed to add i2c driver for subPMIC [errno=%d]", ret);
	}

	gpio_request(gpio_power_on, "SUBPMU_PWRON");

	_rn5t592_dev_configure_locked();

	return ret;
}

void subPMIC_module_exit(void)
{
	i2c_del_driver(&subPMIC_i2c_driver);
}

int subPMIC_PowerOn(int opt)
{
	int ret = 0;
	if (opt == 0)
		rn5t592_dev_poweron();

	return ret;
}

int subPMIC_PowerOff(int opt)
{
	int ret = 0;
	if (opt == 0xff)
		rn5t592_dev_poweroff();

	return ret;
}

int subPMIC_PinOnOff(int pin, int on_off)
{
	return 0;
}
