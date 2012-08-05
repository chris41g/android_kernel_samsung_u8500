/* drivers/input/touchscreen/melfas_ts.c
 *
 * Copyright (C) 2010 Melfas, Inc.
 * 	Additions Samsung Electronics
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */


#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/device.h>
#include <linux/wakelock.h>
#ifdef CONFIG_LEDS_CLASS
#include <linux/leds.h>
#endif

#include <linux/input/melfas_ts.h>

#include <mcs8000_download.h>


#define MELFAS_MAX_TOUCH       5
#define FW_VERSION             0x01

#define TS_MAX_X_COORD		320
#define TS_MAX_Y_COORD		480
#define TS_MAX_Z_TOUCH		255
#define TS_MAX_W_TOUCH		30

#define TS_READ_START_ADDR	0x10
#define TS_READ_VERSION_ADDR	0x63
#define TS_READ_REGS_LEN	30

#define I2C_RETRY_CNT		3

#define PRESS_KEY		1
#define RELEASE_KEY		0

/* to be removed when touch screen works */
#define dev_dbg	dev_info


#ifdef CONFIG_LEDS_CLASS
#define TOUCHKEY_BACKLIGHT	"button-backlight"
#endif
 

struct muti_touch_info
{
    int state;
    int strength;
    int width;
    int posX;
    int posY;
};

struct melfas_ts_data
{
    uint16_t addr;
	uint8_t version[2];
    struct i2c_client *client;
    struct input_dev *input_dev;
	struct workqueue_struct *wq;
    struct work_struct  work;
    uint32_t flags;
	struct muti_touch_info mtouch_info[MELFAS_MAX_TOUCH];
	int key_status[MELFAS_MAX_KEYS];
    struct early_suspend early_suspend;
	struct melfas_tsi_platform_data pdata;
	struct regulator		*tsp_reg;
#ifdef CONFIG_LEDS_CLASS
	struct led_classdev 	leds;
	enum led_brightness 	brightness;
	struct work_struct		led_work;
#endif
	struct wake_lock		update_wakelock;
	struct work_struct		work_update;
};
static struct melfas_ts_data *melfas_ts_ptr;

/* firmware - update */
static int firmware_ret_val = 0;
extern int melfas_touchscreen_firmware_version;

/* sys fs */
struct device *touchscreen_dev;
EXPORT_SYMBOL(touchscreen_dev);

static ssize_t melfas_ts_show_tsp_info(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t melfas_ts_touchkey_keys(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t melfas_ts_show_touchkey_keys(struct device *dev, struct device_attribute *attr, char *buf);
#ifdef CONFIG_LEDS_CLASS
static ssize_t melfas_ts_show_touchkey_bright(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t melfas_ts_set_touchkey_bright( struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
#endif

static DEVICE_ATTR(tsp_info, 0444, melfas_ts_show_tsp_info, NULL);
static DEVICE_ATTR(touchkey_keys, 0444, melfas_ts_show_touchkey_keys, NULL);
#ifdef CONFIG_LEDS_CLASS
static DEVICE_ATTR(touchkey_brightness, 0666, melfas_ts_show_touchkey_bright, melfas_ts_set_touchkey_bright);
#endif

#if CONFIG_TSP_FACTORY_TEST
static ssize_t melfas_ts_show_firm_version_phone(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t melfas_ts_show_firm_version_panel(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t melfas_ts_show_firm_update_status(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t melfas_ts_firm_update( struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static ssize_t key_threshold_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t key_threshold_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size);
static DEVICE_ATTR(tsp_firm_version_phone, 0444, melfas_ts_show_firm_version_phone, NULL);
static DEVICE_ATTR(tsp_firm_version_panel, 0444, melfas_ts_show_firm_version_panel, NULL);
static DEVICE_ATTR(tsp_firm_update_status, 0444, melfas_ts_show_firm_update_status, NULL);
static DEVICE_ATTR(tsp_firm_update , 0222, NULL, melfas_ts_firm_update);
static DEVICE_ATTR(key_threshold, S_IRUGO | S_IWUSR, key_threshold_show, key_threshold_store);
#endif

static struct attribute *Melfas_Touch_attributes[] = {
	&dev_attr_tsp_info.attr,
#ifdef CONFIG_LEDS_CLASS
	&dev_attr_touchkey_brightness.attr,
#endif
	&dev_attr_touchkey_keys.attr,
#if CONFIG_TSP_FACTORY_TEST
	&dev_attr_tsp_firm_version_phone.attr,
	&dev_attr_tsp_firm_version_panel.attr,
	&dev_attr_tsp_firm_update_status.attr,
	&dev_attr_tsp_firm_update.attr,
	&dev_attr_key_threshold.attr,
#endif
	NULL,
};

static struct attribute_group Melfas_Touch_attr_group = {
	.attrs = Melfas_Touch_attributes,
};
/* sys fs */



#ifdef CONFIG_HAS_EARLYSUSPEND
static void melfas_ts_early_suspend(struct early_suspend *h);
static void melfas_ts_late_resume(struct early_suspend *h);
#endif

static int melfas_power_panel(struct melfas_ts_data *ts, int on)
{
    int buf = 0x00;
	int ret;

	if (on) {
		if (ts->tsp_reg) {
			regulator_enable(ts->tsp_reg);
		} else if (gpio_is_valid(ts->pdata.pwr_en_gpio)) {
			gpio_set_value(ts->pdata.pwr_en_gpio,1);
		}
		msleep(100);	/* wait for power to stablise */

	    ret = i2c_master_send(ts->client, &buf, 1);
	    ret = i2c_master_send(ts->client, &buf, 1);

	    if (ret < 0)
	    {
	        dev_err(&ts->client->dev, "Init failed\n [%d]", ret);
	        return -ENODEV;
	    }
	} else {
		if (ts->tsp_reg) {
			regulator_disable(ts->tsp_reg);
		} else if (gpio_is_valid(ts->pdata.pwr_en_gpio)) {
			gpio_set_value(ts->pdata.pwr_en_gpio, 0);
		}
	}
    return 0;
}

static void melfas_ts_work_func(struct work_struct *work)
{
    struct melfas_ts_data *ts = container_of(work, struct melfas_ts_data, work);
    int ret = 0, i;
    uint8_t buf[TS_READ_REGS_LEN];
    int touchNumber = 0, touchPosition = 0, posX = 0, posY = 0, width = 0, strength = 0;
    int keyEvent = 0, keyState = 0, keyID = 0, keystrength = 0;

    dev_dbg(&ts->client->dev, "work_func\n");
    if (ts == NULL)
        dev_err(&ts->client->dev, "work_func : TS NULL\n");


    /**
    Simple send transaction:
    	S Addr Wr [A]  Data [A] Data [A] ... [A] Data [A] P
    Simple recv transaction:
    	S Addr Rd [A]  [Data] A [Data] A ... A [Data] NA P
    */

    buf[0] = TS_READ_START_ADDR;
    for (i = 0; i < I2C_RETRY_CNT; i++) {
        ret = i2c_master_send(ts->client, buf, 1);
        dev_dbg(&ts->client->dev, "work_func : i2c_master_send [%d]\n", ret);
        if (ret >= 0) {
            ret = i2c_master_recv(ts->client, buf, TS_READ_REGS_LEN);
            dev_dbg(&ts->client->dev, "work_func : i2c_master_recv [%d]\n", ret);
            if (ret >= 0)
                break; // i2c success
        }
    }

    if (ret < 0) {
        dev_err(&ts->client->dev, "work_func: i2c failed\n");
        enable_irq(ts->client->irq);
        return ;
    } else  { // Five Multi Touch Interface
        touchNumber = buf[0] & 0x0F;
        touchPosition = buf[1] & 0x1F;

        for (i = 0; i < MELFAS_MAX_TOUCH; i++) {
            ts->mtouch_info[i].posX = ((buf[2 + 5*i] >> 4)   << 8) + buf[3 + 5*i];
            ts->mtouch_info[i].posY = ((buf[2 + 5*i] & 0x0F) << 8) + buf[4 + 5*i];
            ts->mtouch_info[i].width = buf[5 + 5*i];
            ts->mtouch_info[i].strength = buf[6 + 5*i];

            if (ts->mtouch_info[i].width != 0) {
                ts->mtouch_info[i].state = 1;
            } else {
                ts->mtouch_info[i].state = 0;
            }
        }

        keyID = buf[5*MELFAS_MAX_TOUCH + 2] & 0x07;
        keyState = (buf[5*MELFAS_MAX_TOUCH + 2] >> 3) & 0x01;
        keyEvent = (buf[5*MELFAS_MAX_TOUCH + 2] >> 4) & 0x01;
        keystrength = (buf[5*MELFAS_MAX_TOUCH + 3]);

        if (touchNumber > MELFAS_MAX_TOUCH) {
            dev_err(&ts->client->dev, "work_func: Touch Num: %d\n",  touchNumber);
            enable_irq(ts->client->irq);
            return;
        }

        for (i = 0; i < MELFAS_MAX_TOUCH; i++) {
            if ((ts->mtouch_info[i].posX == 0) || (ts->mtouch_info[i].posY == 0))
                continue;

            input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);
            input_report_abs(ts->input_dev, ABS_MT_POSITION_X, ts->mtouch_info[i].posX);
            input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, ts->mtouch_info[i].posY);
            input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, ts->mtouch_info[i].strength);
            input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, ts->mtouch_info[i].width);
            input_mt_sync(ts->input_dev);

            dev_dbg(&ts->client->dev, "work_func: Touch ID: %d, x: %d, y: %d, z: %d w: %d\n",
                   i,
                   ts->mtouch_info[i].posX, ts->mtouch_info[i].posY, ts->mtouch_info[i].strength,
                   ts->mtouch_info[i].width);
        }

        if (keyEvent) {
        	for (i = 0; i < ts->pdata.num_keys; i++) {
	            if (keyID == ts->pdata.touchkeys[i].key_id) {
	                input_report_key(ts->input_dev,
									ts->pdata.touchkeys[i].code,
									keyState ? PRESS_KEY : RELEASE_KEY);

					ts->key_status[i] = keyState;

	            	dev_dbg(&ts->client->dev, "work_func: keyID : %d, Name: %s State: %d\n",
							keyID, ts->pdata.touchkeys[i].name, keyState);
	            }
        	}
        }
        input_sync(ts->input_dev);
    }

    enable_irq(ts->client->irq);
}

static irqreturn_t melfas_ts_irq_handler(int irq, void *handle)
{
    struct melfas_ts_data *ts = (struct melfas_ts_data *)handle;

    dev_dbg(&ts->client->dev, "irq_handler\n");

    disable_irq_nosync(ts->client->irq);
	queue_work(ts->wq, &ts->work);

    return IRQ_HANDLED;
}

/* firmware - update */
void mcsdl_vdd_on(void)
{
	melfas_power_panel(melfas_ts_ptr, 1);
}

void mcsdl_vdd_off(void)
{
	melfas_power_panel(melfas_ts_ptr, 0);
}

void melfas_ts_cfg_pin(enum tsp_pin pin, int out, int val)
{
	int gpio;

	switch (pin)
	{
	case TSP_PIN_INTRPT:
		gpio = IRQ_TO_GPIO(melfas_ts_ptr->client->irq);
		break;
	case TSP_PIN_SCL:
		gpio = melfas_ts_ptr->pdata.scl_gpio;
		break;
	case TSP_PIN_SDA:
		gpio = melfas_ts_ptr->pdata.sda_gpio;
		break;
	default:
		return;
	}
	if (out)
		gpio_direction_output(gpio, val);
	else
		gpio_direction_input(gpio);
}

int melfas_ts_check_input(enum tsp_pin pin)
{
	switch (pin)
	{
		case TSP_PIN_INTRPT:
			return gpio_get_value(IRQ_TO_GPIO(melfas_ts_ptr->client->irq));
		case TSP_PIN_SCL:
			return gpio_get_value(melfas_ts_ptr->pdata.scl_gpio);
		case TSP_PIN_SDA:
			return gpio_get_value(melfas_ts_ptr->pdata.sda_gpio);
		default:
			return 0;
	}
}

int firm_update( struct melfas_ts_data *ts )
{
	int ret;

	dev_dbg(&ts->client->dev, "%s\n", __func__);

	gpio_request(ts->pdata.sda_gpio,"TSP SDA");
	gpio_request(ts->pdata.scl_gpio,"TSP SCL");

	dev_dbg(&ts->client->dev, "firm_update: disable_irq\n");
	if (ts->use_irq) {
		disable_irq(ts->client->irq);
		free_irq(ts->client->irq, ts);
	}
	ret = cancel_work_sync(&ts->work);

	local_irq_disable();
	
	firmware_ret_val = 1;
	ret = mcsdl_download_binary_data();

	dev_dbg(&ts->client->dev, "firm_update: enable_irq\n");
	local_irq_enable();

	if( ret ) {
		firmware_ret_val = 2;
		dev_dbg(&ts->client->dev, "%s success\n", __func__);
	} else {
		firmware_ret_val = -1;
		dev_dbg(&ts->client->dev, "%s fail\n", __func__);
	}

	if (ts->use_irq) {
        ret = request_irq(ts->client->irq, melfas_ts_irq_handler,
							IRQF_TRIGGER_FALLING, ts->client->name, ts);
	}

	gpio_free(ts->pdata.sda_gpio);
	gpio_free(ts->pdata.scl_gpio);

	wake_unlock(&ts->update_wakelock);
	return 0;
} 

static void melfas_ts_update_work_func(struct work_struct *work)
{
	struct melfas_ts_data *ts = container_of(work, struct melfas_ts_data, work_update);

	wake_lock(&ts->update_wakelock);
	firm_update(ts);
}
/* firmware - update */

/***********************************************************************************/
/*             SYS FS Interface Functions													     */
/***********************************************************************************/

#ifdef CONFIG_LEDS_CLASS
static void melfas_ts_led_work(struct work_struct *work)
{
	struct melfas_ts_data *ts = container_of(work, struct melfas_ts_data, led_work);

	if (ts->brightness == LED_OFF)
		gpio_set_value(ts->pdata.key_bl_en_gpio, 0);
	else
		gpio_set_value(ts->pdata.key_bl_en_gpio, 1);

}

static void melfas_ts_keybacklight_set (struct led_classdev *led_cdev, enum led_brightness brightness)
{
	struct melfas_ts_data *ts = container_of(led_cdev, struct melfas_ts_data, leds);

	ts->brightness = brightness;

	queue_work(ts->wq, &ts->led_work);
}
#endif

static ssize_t melfas_ts_show_tsp_info(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	ssize_t size;
	int i;

	/* output for each finger is [id],  x,y,width, [status] */
	/* status 0 or < 0 is released, 1 is touching */
	for (i= 0; i < MELFAS_MAX_TOUCH; i++) {
		size += sprintf(buf+size, "Touch id [%d]: Loc %d,%d,%d State %d\n",
						i,
						ts->mtouch_info[i].posX, ts->mtouch_info[i].posY, ts->mtouch_info[i].width,
						ts->mtouch_info[i].state);
	}
	return size;
}

static ssize_t melfas_ts_show_touchkey_keys(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	int chars, i;

	chars = 0;
	for (i = 0; i < ts->pdata.num_keys; i++) {
		chars += sprintf(buf+chars, "Key: %s, State: %d\n",
						ts->pdata.touchkeys[i].name,
						ts->key_status[i]);
	}
	return chars;
}

#ifdef CONFIG_LEDS_CLASS
static ssize_t melfas_ts_show_touchkey_bright(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", ts->brightness);
}
static ssize_t melfas_ts_set_touchkey_bright(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	char *after;
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	unsigned long value = simple_strtoul(buf, &after, 10);	

	ts->brightness = value;
	queue_work(ts->wq, &ts->led_work);	

	return size;
}
#endif

#if CONFIG_TSP_FACTORY_TEST
static ssize_t melfas_ts_show_firm_version_phone(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", melfas_touchscreen_firmware_version);
}

static ssize_t melfas_ts_show_firm_version_panel(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);

	/* 	The TSP Driver report like X-Y as decimal.
		The X is the HW version that TSP has
		The Y is the Firmware version what TSP has. */

	return sprintf(buf, "%d-%d\n", ts->version[0], ts->version[1]);
}

static ssize_t melfas_ts_show_firm_update_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count;
	
	printk(KERN_INFO "Enter firmware_status_show by Factory command \n");

	if (firmware_ret_val == 1) {
		count = sprintf(buf,"Downloading\n");
	} else if (firmware_ret_val == 2) {
		count = sprintf(buf,"PASS\n");
	} else if (firmware_ret_val == -1) {
		count = sprintf(buf,"FAIL\n");
	} else count = sprintf(buf,"NONE\n");
	
	return count;
}

static ssize_t melfas_ts_firm_update( struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	char *after;
	unsigned long value = simple_strtoul(buf, &after, 10);	
	struct melfas_ts_data *ts = dev_get_drvdata(dev);

	dev_dbg(&ts->client->dev, "%s\n", __func__);

	if ( value == 1 )
	{
		dev_dbg(&ts->client->dev, "Firmware update start!!\n" );

		firmware_ret_val = 1;
		queue_work(ts->wq, &ts->work_update);
		return size;
	}

	return size;
}

/* touch key threshold */
static ssize_t key_threshold_show(struct device *dev, struct device_attribute *attr, char *buf)
{	
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	dev_dbg(&ts->client->dev, "key threshold not support\n");
	return sprintf(buf, "%d\n", 0);
}

static ssize_t key_threshold_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	dev_dbg(&ts->client->dev, "key threshold not support\n");

	return size;
}
#endif

/***********************************************************************************/
/*             Basic Driver Functions													            */
/***********************************************************************************/

static int melfas_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct melfas_ts_data *ts;
	struct melfas_tsi_platform_data *pData = client->dev.platform_data;
    int ret = 0, i;
    uint8_t buf[2];

    dev_dbg(&ts->client->dev, "probe\n");

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        dev_err(&ts->client->dev, "probe: need I2C_FUNC_I2C\n");
        ret = -ENODEV;
        goto err_check_functionality_failed;
    }

	if (!pData) {
		dev_err(&ts->client->dev, "probe: No platform data");
		ret = -EINVAL;
		goto err_no_pdata;
	}

    ts = kmalloc(sizeof(struct melfas_ts_data), GFP_KERNEL);
    if (ts == NULL) {
        dev_err(&ts->client->dev, "probe: failed to create a state of melfas-ts\n");
        ret = -ENOMEM;
        goto err_alloc_data_failed;
    }
	melfas_ts_ptr = ts;
	ts->pdata = *pData;

	ts->wq = create_singlethread_workqueue("melfas tsp wq");
    INIT_WORK(&ts->work, melfas_ts_work_func);
	wake_lock_init(&ts->update_wakelock, WAKE_LOCK_IDLE, "tsp_update");
	INIT_WORK(&ts->work_update, melfas_ts_update_work_func);

    ts->client = client;
    i2c_set_clientdata(client, ts);

	if (ts->pdata.regulator) {
		ts->tsp_reg = regulator_get(&client->dev, ts->pdata.regulator);
	} else if (gpio_is_valid(ts->pdata.pwr_en_gpio)) {
		gpio_request(ts->pdata.pwr_en_gpio, "TSP_PWR_GPIO");
	}

	ret = melfas_power_panel(ts, 1);
	if (ret < 0) {
		dev_err(&ts->client->dev, "probe: power on failed\n");
		goto err_detect_failed;
	}

    buf[0] = TS_READ_VERSION_ADDR;
    for (i = 0; i < I2C_RETRY_CNT; i++) {
        ret = i2c_master_send(ts->client, buf, 1);
        if (ret >= 0) {
            ret = i2c_master_recv(ts->client, buf, 2);

            if (ret >= 0)
                break; // i2c success
        } else {
			dev_err(&ts->client->dev, "probe: err_detect failed\n");
            goto err_detect_failed;
        }
    }
	ts->version[0] = buf[0];
	ts->version[1] = buf[1];
	dev_err(&ts->client->dev, "HW Ver %x, FW Ver %x", buf[0], buf[1]);	


    ts->input_dev = input_allocate_device();
    if (!ts->input_dev)
    {
        dev_err(&ts->client->dev, "probe: Not enough memory\n");
        ret = -ENOMEM;
        goto err_input_dev_alloc_failed;
    }

    ts->input_dev->name = MELFAS_TS_NAME;

	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(EV_ABS, ts->input_dev->evbit);

	dev_dbg(&ts->client->dev, "probe: Max X,Y: %d,%d  Max Touch %d\n",
			ts->pdata.x_size, ts->pdata.y_size, MELFAS_MAX_TOUCH);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->pdata.x_size, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->pdata.y_size, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, TS_MAX_Z_TOUCH, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, MELFAS_MAX_TOUCH - 1, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, TS_MAX_W_TOUCH, 0, 0);

	for (i = 0; i < ts->pdata.num_keys; i++) {
		input_set_capability(ts->input_dev, EV_KEY, ts->pdata.touchkeys[i].code);
	}

    ret = input_register_device(ts->input_dev);
    if (ret)
    {
        dev_err(&ts->client->dev, "probe: Failed to register device\n");
        ret = -ENOMEM;
        goto err_input_register_device_failed;
    }

    if (ts->client->irq)
    {
        dev_dbg(&ts->client->dev, "probe: trying to request irq: %s-%d\n", ts->client->name, ts->client->irq);

        ret = request_irq(client->irq, melfas_ts_irq_handler, IRQF_TRIGGER_FALLING, ts->client->name, ts);
        if (ret > 0)
        {
            dev_err(&ts->client->dev, "probe: Can't allocate irq %d, ret %d\n", ts->client->irq, ret);
            ret = -EBUSY;
            goto err_request_irq;
        }
    }

	queue_work(ts->wq, &ts->work);

    dev_dbg(&ts->client->dev, "probe: succeed to register input device\n");

#if CONFIG_HAS_EARLYSUSPEND
    ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    ts->early_suspend.suspend = melfas_ts_early_suspend;
    ts->early_suspend.resume = melfas_ts_late_resume;
    register_early_suspend(&ts->early_suspend);
#endif

#ifdef CONFIG_LEDS_CLASS
	if (gpio_is_valid(ts->pdata.key_bl_en_gpio)) {
		gpio_request(ts->pdata.key_bl_en_gpio, "TSP Key BL");

		INIT_WORK(&ts->led_work, melfas_ts_led_work);

		ts->leds.name			= TOUCHKEY_BACKLIGHT;
		ts->leds.brightness 	= LED_FULL;
		ts->leds.max_brightness = LED_FULL;
		ts->leds.brightness_set = melfas_ts_keybacklight_set;
		
		ret = led_classdev_register(&client->dev, &ts->leds);
		if ( ret	) {
			goto err_backlight_failed;
		}
	}
#endif

	/* sys fs */
	touchscreen_dev = device_create(&input_class, NULL, 0, NULL, MELFAS_TS_NAME);
	if (IS_ERR(touchscreen_dev))
		dev_err(&ts->client->dev, "Failed to create device(touchscreen)!\n");

	dev_set_drvdata(touchscreen_dev, ts);
	ret = sysfs_create_group(&touchscreen_dev->kobj, &Melfas_Touch_attr_group);
	if (ret) {
		dev_err(&ts->client->dev, "Failed to create sysfs group!\n");
		goto err_remove_attr_group;
	}
	/* sys fs */
		
    dev_dbg(&ts->client->dev, "probe: Start touchscreen. name: %s, irq: %d\n", ts->client->name, ts->client->irq);
    return 0;

err_request_irq:
err_backlight_failed:
err_remove_attr_group:
	free_irq(client->irq, ts);
err_input_register_device_failed:
    input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
err_alloc_data_failed:
err_detect_failed:
	if (ts->pdata.regulator) {
		regulator_put(&ts->pdata.regulator);
	} else if (gpio_is_valid(ts->pdata.pwr_en_gpio)) {
		gpio_free(ts->pdata.pwr_en_gpio);
	}
	destroy_workqueue(ts->wq);
    kfree(ts);
err_check_functionality_failed:
err_no_pdata:
    return ret;
}

static int melfas_ts_remove(struct i2c_client *client)
{
    struct melfas_ts_data *ts = i2c_get_clientdata(client);

    unregister_early_suspend(&ts->early_suspend);
    free_irq(client->irq, ts);

	if (ts->tsp_reg) {
		regulator_put(ts->tsp_reg);
	} else if (gpio_is_valid(ts->pdata.pwr_en_gpio)) {
		gpio_free(ts->pdata.pwr_en_gpio);
	}

    input_unregister_device(ts->input_dev);

#ifdef CONFIG_LEDS_CLASS
	cancel_work_sync(&ts->led_work);
	led_classdev_unregister(&ts->leds);
	if (gpio_is_valid(ts->pdata.key_bl_en_gpio)) {
		gpio_free(ts->pdata.key_bl_en_gpio);
	}
#endif
	sysfs_remove_group(&touchscreen_dev->kobj, &Melfas_Touch_attr_group);

	cancel_work_sync(&ts->work);
	destroy_workqueue(ts->wq);

    kfree(ts);
    return 0;
}

static int melfas_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
    int ret;
    struct melfas_ts_data *ts = i2c_get_clientdata(client);

	dev_dbg(&ts->client->dev, "Suspend");

	if (wake_lock_active(&ts->update_wakelock))
		return -EBUSY;

    disable_irq(client->irq);

    ret = cancel_work_sync(&ts->work);
    if (ret) /* if work was pending disable-count is now 2 */
        enable_irq(client->irq);

	melfas_power_panel(ts, 0);

#ifdef CONFIG_LEDS_CLASS
	if (gpio_is_valid(ts->pdata.key_bl_en_gpio)) {
			gpio_set_value(ts->pdata.key_bl_en_gpio, 0);
	}
#endif

    return 0;
}

static int melfas_ts_resume(struct i2c_client *client)
{
    struct melfas_ts_data *ts = i2c_get_clientdata(client);

	dev_dbg(&ts->client->dev, "Resume");

	if (wake_lock_active(&ts->update_wakelock))
		return 0;

    melfas_power_panel(ts, 1);
    enable_irq(client->irq); // scl wave

#ifdef CONFIG_LEDS_CLASS
	if (gpio_is_valid(ts->pdata.key_bl_en_gpio)) {
		if (ts->brightness == LED_OFF)
			gpio_set_value(ts->pdata.key_bl_en_gpio, 0);
		else
			gpio_set_value(ts->pdata.key_bl_en_gpio, 1);
	}
#endif

    return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void melfas_ts_early_suspend(struct early_suspend *h)
{
    struct melfas_ts_data *ts;
    ts = container_of(h, struct melfas_ts_data, early_suspend);
    melfas_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void melfas_ts_late_resume(struct early_suspend *h)
{
    struct melfas_ts_data *ts;
    ts = container_of(h, struct melfas_ts_data, early_suspend);
    melfas_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id melfas_ts_id[] =
{
    { MELFAS_TS_NAME, 0 },
    { }
};

static struct i2c_driver melfas_ts_driver =
{
    .driver = {
    .name = MELFAS_TS_NAME,
    },
    .id_table = melfas_ts_id,
    .probe = melfas_ts_probe,
    .remove = __devexit_p(melfas_ts_remove),
#ifndef CONFIG_HAS_EARLYSUSPEND
    .suspend = melfas_ts_suspend,
    .resume = melfas_ts_resume,
#endif
};

static int __devinit melfas_ts_init(void)
{
    return i2c_add_driver(&melfas_ts_driver);
}

static void __exit melfas_ts_exit(void)
{
    i2c_del_driver(&melfas_ts_driver);
}

MODULE_DESCRIPTION("Driver for Melfas MTSI Touchscreen Controller");
MODULE_AUTHOR("MinSang, Kim <kimms@melfas.com>");
MODULE_VERSION("0.1");
MODULE_LICENSE("GPL");

module_init(melfas_ts_init);
module_exit(melfas_ts_exit);
