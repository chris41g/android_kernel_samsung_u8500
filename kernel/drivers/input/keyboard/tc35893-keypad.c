/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * Author: Jayeeta Banerjee <jayeeta.banerjee@stericsson.com> for ST-Ericsson
 * Author: Sundar Iyer <sundar.iyer@stericsson.com> for ST-Ericsson
 *
 * License Terms: GNU General Public License, version 2
 *
 * TC35893 MFD Keypad Controller driver
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <mach/tc35893-keypad.h>

/* Maximum supported keypad matrix row/columns size */
#define TC_MAX_KPROW               8
#define TC_MAX_KPCOL               12

/* keypad related Constants */
#define TC_MAX_DEBOUNCE_SETTLE     0xFF
#define DEDICATED_KEY_VAL	   0xFF

/* Keyboard Configuration Registers Index */
#define TC_KBDSETTLE_REG	0x01
#define TC_KBDBOUNCE		0x02
#define	TC_KBDSIZE		0x03
#define TC_KBCFG_LSB		0x04
#define TC_KBCFG_MSB		0x05
#define TC_KBDRIS		0x06
#define TC_KBDMIS		0x07
#define TC_KBDIC		0x08
#define TC_KBDMSK		0x09
#define TC_KBDCODE0		0x0B
#define TC_KBDCODE1		0x0C
#define TC_KBDCODE2		0x0D
#define TC_KBDCODE3		0x0E
#define TC_EVTCODE_FIFO		0x10

/* System registers Index */
#define TC_MANUFACTURE_CODE	0x80
#define TC_VERSION_ID		0x81
#define TC_I2CSA		0x82
#define TC_IOCFG		0xA7

/* clock control registers */
#define TC_CLKMODE		0x88
#define TC_CLKCFG		0x89
#define TC_CLKEN		0x8A

/* Reset Control registers */
#define TC_RSTCTRL		0x82
#define TC_RSTINTCLR		0x84

/* special function register and drive config registers */
#define TC_KBDMFS		0x8F
#define TC_IRQST		0x91
#define TC_DRIVE0_LSB		0xA0
#define TC_DRIVE0_MSB		0xA1
#define TC_DRIVE1_LSB		0xA2
#define TC_DRIVE1_MSB		0xA3
#define TC_DRIVE2_LSB		0xA4
#define TC_DRIVE2_MSB		0xA5
#define TC_DRIVE3		0xA6

/* Pull up/down  configuration registers */
#define TC_IOCFG		0xA7
#define TC_IOPULLCFG0_LSB	0xAA
#define TC_IOPULLCFG0_MSB	0xAB
#define TC_IOPULLCFG1_LSB	0xAC
#define TC_IOPULLCFG1_MSB	0xAD
#define TC_IOPULLCFG2_LSB	0xAE

/* Pull up/down masks */
#define TC_NO_PULL_MASK		0x0
#define TC_PULL_DOWN_MASK	0x1
#define TC_PULL_UP_MASK		0x2
#define TC_PULLUP_ALL_MASK	0xAA
#define TC_IO_PULL_VAL(index, mask)	((mask)<<((index)%4)*2))

/* Bit masks for IOCFG register */
#define IOCFG_BALLCFG		0x01
#define IOCFG_IG		0x08

#define KP_EVCODE_COL_MASK	0x0F
#define KP_EVCODE_ROW_MASK	0x70
#define KP_RELEASE_EVT_MASK	0x80

#define KP_ROW_SHIFT		4

#define KP_NO_VALID_KEY_MASK	0x7F


#define TC_MAN_CODE_VAL		0x03
#define TC_SW_VERSION		0x80

/* bit masks for RESTCTRL register */
#define TC_KBDRST		0x2
#define TC_IRQRST		0x10
#define TC_RESET_ALL		0x1B

/* KBDMFS register bit mask */
#define TC_KBDMFS_EN		0x1

/* CLKEN register bitmask */
#define KPD_CLK_EN		0x1

/* RSTINTCLR register bit mask */
#define IRQ_CLEAR		0x1

/* bit masks for keyboard interrupts*/
#define TC_EVT_LOSS_INT		0x8
#define TC_EVT_INT		0x4
#define TC_KBD_LOSS_INT		0x2
#define TC_KBD_INT		0x1

/* bit masks for keyboard interrupt clear*/
#define TC_EVT_INT_CLR		0x2
#define TC_KBD_INT_CLR		0x1

#define TC_KBD_KEYMAP_SIZE     64

/**
 * struct tc_keypad - data structure used by keypad driver
 * @input:      pointer to input device object
 * @board:      keypad platform device
 * @client:	pointer to i2c client
 * @lock:	lock
 * @krow:	number of rows
 * @kcol:	number of coloumns
 * @keymap:     matrix scan code table for keycodes
 * @enable:	bool to enable/disable keypad operation
 */
struct tc_keypad {
	struct input_dev *input;
	const struct tc35893_platform_data *board;
	struct i2c_client *client;
	struct mutex lock;
	unsigned int krow;
	unsigned int kcol;
	unsigned short keymap[TC_KBD_KEYMAP_SIZE];
	bool enable;
};


static int tc35893_write_byte(struct i2c_client *client,
				unsigned char reg, unsigned char data)
{
	int ret;
	struct tc_keypad *keypad = i2c_get_clientdata(client);

	mutex_lock(&keypad->lock);
	ret = i2c_smbus_write_byte_data(client, reg, data);
	if (ret < 0)
		dev_err(&client->dev, "Error in writing reg %x: error = %d\n",
			reg, ret);
	mutex_unlock(&keypad->lock);

	return ret;
}

static int tc35893_read_byte(struct i2c_client *client, unsigned char reg,
			unsigned char *val)
{
	int ret;
	struct tc_keypad *keypad = i2c_get_clientdata(client);

	mutex_lock(&keypad->lock);
	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0)
		dev_err(&client->dev, "Error in reading reg %x: error = %d\n",
			reg, ret);
	*val = ret;
	mutex_unlock(&keypad->lock);

	return ret;
}

static int tc35893_set_bits(struct i2c_client *client, u16 addr,
			u8 mask, u8 data)
{
	int err;
	u8 val;

	err = tc35893_read_byte(client, addr, &val);
	if (err < 0)
		return err;

	val &= ~mask;
	val |= data;

	err = tc35893_write_byte(client, addr, val);

	return err;
}

static int __devinit tc35893_keypad_init_key_hardware(struct tc_keypad *keypad)
{
	int ret;
	struct i2c_client *client = keypad->client;
	u8 settle_time = keypad->board->settle_time;
	u8 dbounce_period = keypad->board->debounce_period;
	u8 rows = keypad->board->krow & 0xf;	/* mask out the nibble */
	u8 column = keypad->board->kcol & 0xf;	/* mask out the nibble */

	/* validate platform configurations */
	if ((keypad->board->kcol > TC_MAX_KPCOL) ||
	    (keypad->board->krow > TC_MAX_KPROW) ||
	    (keypad->board->debounce_period > TC_MAX_DEBOUNCE_SETTLE) ||
	    (keypad->board->settle_time > TC_MAX_DEBOUNCE_SETTLE))
		return -EINVAL;

	/* configure KBDSIZE 4 LSbits for cols and 4 MSbits for rows */
	ret = tc35893_set_bits(client, TC_KBDSIZE, 0x0,
			(rows << KP_ROW_SHIFT) | column);
	if (ret < 0)
		return ret;

	/* configure dedicated key config, no dedicated key selected */
	ret = tc35893_set_bits(client, TC_KBCFG_LSB, 0x0, DEDICATED_KEY_VAL);
	if (ret < 0)
		return ret;

	ret = tc35893_set_bits(client, TC_KBCFG_MSB, 0x0, DEDICATED_KEY_VAL);
	if (ret < 0)
		return ret;

	/* Configure settle time */
	ret = tc35893_set_bits(client, TC_KBDSETTLE_REG, 0x0, settle_time);
	if (ret < 0)
		return ret;

	/* Configure debounce time */
	ret = tc35893_set_bits(client, TC_KBDBOUNCE, 0x0, dbounce_period);
	if (ret < 0)
		return ret;

	/* Start of initialise keypad GPIOs */
	ret = tc35893_set_bits(client, TC_IOCFG, 0x0, IOCFG_IG);
	if (ret < 0)
		return ret;

	/* Configure pull-up resistors for all row GPIOs */
	ret = tc35893_set_bits(client, TC_IOPULLCFG0_LSB, 0x0,
			TC_PULLUP_ALL_MASK);
	if (ret < 0)
		return ret;

	ret = tc35893_set_bits(client, TC_IOPULLCFG0_MSB, 0x0,
			TC_PULLUP_ALL_MASK);
	if (ret < 0)
		return ret;

	/* Configure pull-up resistors for all column GPIOs */
	ret = tc35893_set_bits(client, TC_IOPULLCFG1_LSB, 0x0,
			TC_PULLUP_ALL_MASK);
	if (ret < 0)
		return ret;

	ret = tc35893_set_bits(client, TC_IOPULLCFG1_MSB, 0x0,
			TC_PULLUP_ALL_MASK);
	if (ret < 0)
		return ret;

	ret = tc35893_set_bits(client, TC_IOPULLCFG2_LSB, 0x0,
			TC_PULLUP_ALL_MASK);

	return ret;
}

#define TC35893_DATA_REGS              4
#define TC35893_KEYCODE_FIFO_EMPTY     0x7f
#define TC35893_KEYCODE_FIFO_CLEAR     0xff

static irqreturn_t tc35893_keypad_irq(int irq, void *dev)
{
	struct tc_keypad *keypad = (struct tc_keypad *)dev;
	u8 i, row_index, col_index, kbd_code, up;
	u8 code;

	for (i = 0; i < (TC35893_DATA_REGS * 2); i++) {
		tc35893_read_byte(keypad->client, TC_EVTCODE_FIFO, &kbd_code);

		/* loop till fifo is empty and no more keys are pressed */
		if ((kbd_code == TC35893_KEYCODE_FIFO_EMPTY) ||
				(kbd_code == TC35893_KEYCODE_FIFO_CLEAR))
			continue;

		/* valid key is found */
		col_index = kbd_code & KP_EVCODE_COL_MASK;
		row_index = (kbd_code & KP_EVCODE_ROW_MASK) >> KP_ROW_SHIFT;
		code = MATRIX_SCAN_CODE(row_index, col_index, 0x3);
		up = kbd_code & KP_RELEASE_EVT_MASK;

		input_event(keypad->input, EV_MSC, MSC_SCAN, code);
		input_report_key(keypad->input, keypad->keymap[code], !up);
		input_sync(keypad->input);
	}

	/* clear IRQ */
	tc35893_set_bits(keypad->client, TC_KBDIC,
			0x0, TC_EVT_INT_CLR | TC_KBD_INT_CLR);
	/* enable IRQ */
	tc35893_set_bits(keypad->client, TC_KBDMSK,
			0x0, TC_EVT_LOSS_INT | TC_EVT_INT);

	return IRQ_HANDLED;
}

static int __devinit tc35893_keypad_enable(struct i2c_client *client)
{
	int ret;

	/* pull the keypad module out of reset */
	ret = tc35893_set_bits(client, TC_RSTCTRL, TC_KBDRST, 0x0);
	if (ret < 0)
		return ret;

	/* configure KBDMFS */
	ret = tc35893_set_bits(client, TC_KBDMFS, 0x0, TC_KBDMFS_EN);
	if (ret < 0)
		return ret;

	/* enable the keypad clock */
	ret = tc35893_set_bits(client, TC_CLKEN, 0x0, KPD_CLK_EN);
	if (ret < 0)
		return ret;

	/* clear pending IRQs */
	ret =  tc35893_set_bits(client, TC_RSTINTCLR, 0x0, 0x1);
	if (ret < 0)
		return ret;

	/* enable the IRQs */
	ret = tc35893_set_bits(client, TC_KBDMSK, 0x0,
					TC_EVT_LOSS_INT | TC_EVT_INT);

	return ret;
}

static int __devexit tc35893_keypad_disable(struct i2c_client *client)
{
	int ret;

	/* clear IRQ */
	ret = tc35893_set_bits(client, TC_KBDIC,
			0x0, TC_EVT_INT_CLR | TC_KBD_INT_CLR);
	if (ret < 0)
		return ret;

	/* disable all interrupts */
	ret = tc35893_set_bits(client, TC_KBDMSK,
			~(TC_EVT_LOSS_INT | TC_EVT_INT), 0x0);
	if (ret < 0)
		return ret;

	/* disable the keypad module */
	ret = tc35893_set_bits(client, TC_CLKEN, 0x1, 0x0);
	if (ret < 0)
		return ret;

	/* put the keypad module into reset */
	ret = tc35893_set_bits(client, TC_RSTCTRL, TC_KBDRST, 0x1);

	return ret;
}

static ssize_t keypad_show_attr_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct tc_keypad *keypad = i2c_get_clientdata(client);

	return sprintf(buf, "%u\n", keypad->enable);
}

static ssize_t keypad_store_attr_enable(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct tc_keypad *keypad = i2c_get_clientdata(client);
	unsigned long state;

	if (sscanf(buf, "%lu", &state) != 1)
		return -EINVAL;

	if ((state != 1) && (state != 0))
		return -EINVAL;

	if (state != keypad->enable) {
		if (state)
			tc35893_keypad_enable(keypad->client);
		else
			tc35893_keypad_disable(keypad->client);
		keypad->enable = state;
	}

	return strnlen(buf, count);
}

static DEVICE_ATTR(enable, S_IWUSR | S_IRUGO,
		keypad_show_attr_enable, keypad_store_attr_enable);

static struct attribute *tc35893_keypad_attrs[] = {
	&dev_attr_enable.attr,
	NULL,
};

static struct attribute_group tc35893_attr_group = {
	.attrs = tc35893_keypad_attrs,
};

static int __devinit tc35893_keypad_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	const struct tc35893_platform_data *plat = client->dev.platform_data;
	struct tc_keypad *keypad;
	struct input_dev *input;
	int error;

	if (!plat) {
		dev_err(&client->dev, "invalid keypad platform data\n");
		return -EINVAL;
	}

	if (plat->irq < 0) {
		dev_err(&client->dev, "failed to get keypad irq\n");
		return -EINVAL;
	}

	keypad = kzalloc(sizeof(struct tc_keypad), GFP_KERNEL);
	input = input_allocate_device();
	if (!keypad || !input) {
		dev_err(&client->dev, "failed to allocate keypad memory\n");
		error = -ENOMEM;
		goto err_free_mem;
	}

	keypad->client = client;
	keypad->board = plat;
	keypad->input = input;
	mutex_init(&keypad->lock);

	i2c_set_clientdata(client, keypad);

	/* issue a soft reset to all modules */
	error = tc35893_set_bits(client, TC_RSTCTRL, 0x0, TC_RESET_ALL);
	if (error < 0)
		return error;

	/* pull the irq module out of reset */
	error = tc35893_set_bits(client, TC_RSTCTRL, TC_IRQRST, 0x0);
	if (error < 0)
		return error;

	error = tc35893_keypad_enable(client);
	if (error < 0) {
		dev_err(&client->dev, "failed to enable keypad module\n");
		goto err_free_mem;
	}

	error = tc35893_keypad_init_key_hardware(keypad);
	if (error < 0) {
		dev_err(&client->dev, "failed to configure keypad module\n");
		goto err_free_mem;
	}

	input->id.bustype = BUS_HOST;
	input->name = client->name;
	input->dev.parent = &client->dev;

	input->keycode = keypad->keymap;
	input->keycodesize = sizeof(keypad->keymap[0]);
	input->keycodemax = ARRAY_SIZE(keypad->keymap);

	input_set_capability(input, EV_MSC, MSC_SCAN);

	__set_bit(EV_KEY, input->evbit);
	if (!plat->no_autorepeat)
		__set_bit(EV_REP, input->evbit);

	matrix_keypad_build_keymap(plat->keymap_data, 0x3,
			input->keycode, input->keybit);

	error = request_threaded_irq(plat->irq, NULL,
			tc35893_keypad_irq, plat->irqtype,
			input->name, keypad);
	if (error < 0) {
		dev_err(&client->dev,
				"Could not allocate irq %d,error %d\n",
				plat->irq, error);
		goto err_free_mem;
	}

	error = input_register_device(input);
	if (error) {
		dev_err(&client->dev, "Could not register input device\n");
		goto err_free_irq;
	}

	device_init_wakeup(&client->dev, plat->enable_wakeup);
	device_set_wakeup_capable(&client->dev, plat->enable_wakeup);

	/* sysfs implementation for dynamic enable/disable the input event */
	error = sysfs_create_group(&client->dev.kobj, &tc35893_attr_group);
	if (error) {
		dev_err(&client->dev, "Could not register sysfs entries\n");
		goto err_free_irq;
	}

	keypad->enable = true;

	return 0;

err_free_irq:
	free_irq(plat->irq, keypad);
err_free_mem:
	input_free_device(input);
	kfree(keypad);
	return error;
}

static int __devexit tc35893_keypad_remove(struct i2c_client *client)
{
	struct tc_keypad *keypad = i2c_get_clientdata(client);

	free_irq(keypad->board->irq, keypad);

	sysfs_remove_group(&client->dev.kobj, &tc35893_attr_group);

	input_unregister_device(keypad->input);

	tc35893_keypad_disable(keypad->client);

	kfree(keypad);

	return 0;
}

#ifdef CONFIG_PM
static int tc35893_keypad_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct tc_keypad *keypad = i2c_get_clientdata(client);
	int irq = keypad->board->irq;

	client = keypad->client;

	/* disable the IRQ */
	if (!device_may_wakeup(&client->dev))
		tc35893_keypad_disable(client);
	else
		enable_irq_wake(irq);

	return 0;
}

static int tc35893_keypad_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct tc_keypad *keypad = i2c_get_clientdata(client);
	int irq = keypad->board->irq;

	client = keypad->client;

	/* enable the IRQ */
	if (!device_may_wakeup(&client->dev))
		tc35893_keypad_enable(client);
	else
		disable_irq_wake(irq);

	return 0;
}

static const struct dev_pm_ops tc35893_keypad_dev_pm_ops = {
	.suspend = tc35893_keypad_suspend,
	.resume  = tc35893_keypad_resume,
};
#endif

static const struct i2c_device_id tc35893_id[] = {
	{ "tc35893-kp", 23 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tc35893_id);

static struct i2c_driver tc35893kpd_driver = {
	.driver = {
		.name = "tc35893-kp",
#ifdef CONFIG_PM
		.pm = &tc35893_keypad_dev_pm_ops,
#endif
	},
	.probe = tc35893_keypad_probe,
	.remove = __devexit_p(tc35893_keypad_remove),
	.id_table = tc35893_id,
};

static int __init tc35893_keypad_init(void)
{
	return i2c_add_driver(&tc35893kpd_driver);
}

static void __exit tc35893_keypad_exit(void)
{
	return i2c_del_driver(&tc35893kpd_driver);
}

module_init(tc35893_keypad_init);
module_exit(tc35893_keypad_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Jayeeta Banerjee/Sundar Iyer");
MODULE_DESCRIPTION("TC35893 Keypad Driver");

