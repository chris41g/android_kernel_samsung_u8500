/* drivers/input/touchscreen/melfas_ts.c
 *
 * Copyright (C) 2010 Melfas, Inc.
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

#define SEC_TOUCHKEY_DEBUG

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
#include <linux/melfas_ts.h>


#include <mach/gpio.h>
#include <mach/board-sec-u8500.h>

#define MMS152_MAX_TOUCH 		10
#define MMS144_MAX_TOUCH		5
#define MMS136_MAX_TOUCH		5
#define MMS128_MAX_TOUCH		5

#define TS_MAX_Z_TOUCH			255
#define TS_MAX_W_TOUCH		30

#define TS_MAX_X_COORD			480	//model dependant
#define TS_MAX_Y_COORD			800	//model dependant

#define TS_READ_LEN_ADDR		0x0F	//0x10
#define TS_READ_START_ADDR		0x10
#define TS_READ_VERSION_ADDR		0xF0	//0x31

#define TS_READ_REGS_LEN		66	//5

#define MELFAS_MAX_TOUCH		3	//model dependant

#define FW_VERSION			0x01	//model dependant

#define I2C_RETRY_CNT			10
#define DOWNLOAD_RETRY_CNT		5

#define PRESS_KEY			1
#define RELEASE_KEY			0

#define DEBUG_PRINT			0

#define CONFIG_HAS_EARLYSUSPEND	1

#define SET_DOWNLOAD_BY_GPIO		1

#if SET_DOWNLOAD_BY_GPIO
#include <mms100_download.h>
#endif // SET_DOWNLOAD_BY_GPIO


/*sec_class sysfs*/
extern struct class *sec_class;

enum
{
	None = 0,
	TOUCH_SCREEN,
	TOUCH_KEY
};

struct muti_touch_info
{
	int strength;
	int width;	
	int posX;
	int posY;
};

struct melfas_ts_data
{
	uint16_t addr;
	struct i2c_client *client; 
	struct input_dev *input_dev;
	struct input_dev *input_dev_key;
	struct melfas_tsi_platform_data *pdata;
	struct work_struct work;
	uint32_t flags;
	int (*power)(int on);
	struct early_suspend early_suspend;
	char phys[32];
};

struct melfas_ts_data *ts = NULL;

#if CONFIG_HAS_EARLYSUSPEND
static void melfas_ts_early_suspend(struct early_suspend *h);
static void melfas_ts_late_resume(struct early_suspend *h);
#endif


static struct muti_touch_info g_Mtouch_info[MELFAS_MAX_TOUCH];

static int melfas_power_panel(int on)
{
	uint8_t buf = 0x00;
	int ret;

	if (on) {
		if (gpio_is_valid(TSP_LDO_ON_GAVINI_R0_0)) {
			gpio_set_value(TSP_LDO_ON_GAVINI_R0_0,1);
		}
		 
		msleep(100);	/* wait for power to stablise */

		ret = i2c_master_send(ts->client, &buf, 1);
		if (ret < 0)
		{
			printk(KERN_ERR "melfas_ts_probe: i2c_master_send() failed\n [%d]", ret);	
			return 0;
		}
	} else {
		if (gpio_is_valid(TSP_LDO_ON_GAVINI_R0_0)) {
			gpio_set_value(TSP_LDO_ON_GAVINI_R0_0, 0);
		}
	}
    return 0;
}

static bool melfas_ts_interrupt_check(void)
{
	return gpio_get_value(TSP_INT_GAVINI_R0_0);
}

static void melfas_ts_work_func(struct work_struct *work)
//static void melfas_ts_get_data()
{
	int ret = 0, i;
	uint8_t buf[TS_READ_REGS_LEN];
	//int touchType=0, touchState =0, touchID=0, posX=0, posY=0, width = 0, strength=0, keyID = 0, reportID = 0;
	int read_num = 0, fingerID = 0, keyID = 0, touchType = 0, touchState = 0;

	ret = melfas_ts_interrupt_check();
#if DEBUG_PRINT
	printk(KERN_INFO "melfas_ts_get_data : interrupt pin level : %d\n", ret);
#endif 
	if (ret){
		printk(KERN_ERR "melfas_ts_get_data : No Valid interrupt.\n");
		goto melfas_enable_irq;
	}

	//read length
	buf[0] = TS_READ_LEN_ADDR;

	for (i = 0; i < I2C_RETRY_CNT; i++)
	{
		ret = i2c_master_send(ts->client, buf, 1);

		if (ret >=0)
		{
			ret = i2c_master_recv(ts->client, buf, 1);

			if (ret >=0)
			{
#if DEBUG_PRINT
				printk(KERN_INFO "melfas_ts_get_data : TS_READ_LEN_ADDR [%d]\n", ret);			
#endif
				break; // i2c success
			}
		}
	}

	if (ret < 0)
	{
		printk(KERN_ERR "melfas_ts_get_data: i2c failed\n");
		goto melfas_enable_irq;
	}
	else
	{
		read_num = buf[0];
	}

	//read touch info
	if (read_num > 0)
	{
		buf[0] = TS_READ_START_ADDR;

		for (i = 0; i < I2C_RETRY_CNT; i++)
		{
			ret = i2c_master_send(ts->client, buf, 1);

			if (ret >=0)
			{
				ret = i2c_master_recv(ts->client, buf, read_num);

				if (ret >=0)
				{
#if DEBUG_PRINT
					printk(KERN_INFO "melfas_ts_get_data : TS_READ_START_ADDR [%d]\n", ret); 		
#endif
					break; // i2c success
				}
			}
		}

		if (ret < 0)
		{
			printk(KERN_ERR "melfas_ts_get_data: i2c failed\n");
			goto melfas_enable_irq;
		}
		else
		{
			for (i = 0; i < read_num; i = i + 6)
			{
				if (buf[0] == 0x0f) {
					melfas_power_panel(0);
					melfas_power_panel(1);
					goto melfas_enable_irq;
				}

				touchType = (buf[i] >> 5) & 0x03;

				if (touchType == TOUCH_SCREEN)
				{
					fingerID = (buf[i] & 0x0F) - 1;
					touchState = (buf[i] & 0x80);

					g_Mtouch_info[fingerID].posX = (uint16_t)(buf[i + 1] & 0x0F) << 8 | buf[i + 2];
					g_Mtouch_info[fingerID].posY = (uint16_t)(buf[i + 1] & 0xF0) << 4 | buf[i + 3];
					g_Mtouch_info[fingerID].width = buf[i + 4];

					if (touchState)
						g_Mtouch_info[fingerID].strength = buf[i + 5];
					else
						g_Mtouch_info[fingerID].strength = 0;
				}
				else if (touchType == TOUCH_KEY)
				{
					keyID = (buf[i] & 0x0F);
					touchState = (buf[i] & 0x80);

					if (keyID == 0x1)
						input_report_key(ts->input_dev_key, KEY_MENU, touchState ? PRESS_KEY : RELEASE_KEY);
					//if (keyID == 0x2)
					//	input_report_key(ts->input_dev, KEY_HOME, touchState ? PRESS_KEY : RELEASE_KEY);
					//if (keyID == 0x3)
					//	input_report_key(ts->input_dev, KEY_SEARCH, touchState ? PRESS_KEY : RELEASE_KEY);
					if (keyID == 0x2)
						input_report_key(ts->input_dev_key, KEY_BACK, touchState ? PRESS_KEY : RELEASE_KEY);
					input_sync(ts->input_dev_key);
					printk(KERN_ERR "melfas_ts_get_data: keyID : %d, touchState: %d\n", keyID, touchState);
				}
			}

			for (i = 0; i < MELFAS_MAX_TOUCH; i ++)
			{
				if (g_Mtouch_info[i].strength == -1)
					continue;

				input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, g_Mtouch_info[i].posX);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, g_Mtouch_info[i].posY);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, g_Mtouch_info[i].strength);
				input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, g_Mtouch_info[i].width);
				input_mt_sync(ts->input_dev);
				printk(KERN_ERR "melfas_ts_get_data: Touch ID: %d, State : %d, x: %d, y: %d, z: %d, w: %d\n",
					i, (g_Mtouch_info[i].strength > 0), g_Mtouch_info[i].posX, g_Mtouch_info[i].posY, g_Mtouch_info[i].strength, g_Mtouch_info[i].width);
				if (g_Mtouch_info[i].strength == 0)
					g_Mtouch_info[i].strength = -1;
			}
			input_sync(ts->input_dev);
		}
	}
	else {
		printk(KERN_ERR "melfas_ts_get_data: falling in collecting garbage data\n");

		/* read garbage data from melfas touchscreen */
		read_num = 1;
		
		buf[0] = TS_READ_START_ADDR;

		for (i = 0; i < I2C_RETRY_CNT; i++)
		{
			ret = i2c_master_send(ts->client, buf, 1);

			if (ret >=0)
			{
				ret = i2c_master_recv(ts->client, buf, read_num);

				if (ret >=0)
				{
					break; // i2c success
				}
			}
		}

		if (ret < 0)
		{
			printk(KERN_ERR "melfas_ts_get_data: i2c failed\n");
			goto melfas_enable_irq;
		}
	}

melfas_enable_irq:
	enable_irq(ts->client->irq);
}

static irqreturn_t melfas_ts_irq_handler(int irq, void *handle)
{
	struct melfas_ts_data *ts = (struct melfas_ts_data *)handle;
#if DEBUG_PRINT
	printk(KERN_INFO "melfas_ts_irq_handler\n");
#endif

	disable_irq_nosync(ts->client->irq);
	schedule_work(&ts->work);

	//melfas_ts_get_data(ts);

	return IRQ_HANDLED;
}

static int melfas_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0, i;

	uint8_t buf[4] = {0, };

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		printk(KERN_ERR "melfas_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(struct melfas_ts_data), GFP_KERNEL);
	if (ts == NULL)
	{
		printk(KERN_ERR "melfas_ts_probe: failed to create a state of melfas-ts\n");
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	INIT_WORK(&ts->work, melfas_ts_work_func);

	ts->client = client;
	i2c_set_clientdata(client, ts);

	ret = melfas_power_panel(1);
	if (ret < 0) {
		dev_err(&ts->client->dev, "probe: power on failed\n");
		goto err_detect_failed;
	}

#if DEBUG_PRINT
	printk(KERN_INFO "melfas_ts_probe: i2c_master_send() [%d], Add[%d]\n", ret, ts->client->addr);
#endif

#if SET_DOWNLOAD_BY_GPIO
	buf[0] = TS_READ_VERSION_ADDR;
	ret = i2c_master_send(ts->client, buf, 1);
	if (ret < 0)
	{
		printk(KERN_ERR "melfas_ts_work_func : i2c_master_send [%d]\n", ret);
	}

	ret = i2c_master_recv(ts->client, buf, 4);
	if (ret < 0)
	{
		printk(KERN_ERR "melfas_ts_work_func : i2c_master_recv [%d]\n", ret);
	}

	printk(KERN_ERR "MELFAS touchscreen driver version : TSP 0x%x, H/W 0x%x, F/W 0x%x \n", buf[0], buf[1], buf[3]);

	if (buf[3] < FW_VERSION)
	{
		/* enable gpio */

		for (i = 0; i < DOWNLOAD_RETRY_CNT; i++)
		{
			//ret = mcsdl_download_binary_file();
			ret = mcsdl_download_binary_data(3);
			if (ret < 0)
				printk(KERN_ERR "SET Download Fail - error code [%d]\n", ret);
			else
				break;
		}

		/* disable gpio */

	}
#endif // SET_DOWNLOAD_BY_GPIO

	ts->input_dev = input_allocate_device();
	if (!ts->input_dev)
	{
		printk(KERN_ERR "melfas_ts_probe: Not enough memory\n");
		ret = -ENOMEM;
		goto err_input_dev_alloc_failed;
	} 
	ts->input_dev_key = input_allocate_device();
	if (!ts->input_dev_key)
	{
		printk(KERN_ERR "melfas_ts_probe: Not enough memory\n");
		ret = -ENOMEM;
		goto err_input_dev_alloc_failed;
	} 

	ts->input_dev->name = "sec_touchscreen" ;

	/* begin of - touche key */
	snprintf(ts->phys, sizeof(ts->phys), "%s/input0", dev_name(&client->dev));
	ts->input_dev_key->name = "sec_touchkey" ;
	ts->input_dev_key->phys = ts->phys;
	ts->input_dev_key->id.bustype = BUS_I2C;
	ts->input_dev_key->dev.parent = &client->dev;
	set_bit(EV_SYN, ts->input_dev_key->evbit);
	set_bit(EV_KEY, ts->input_dev_key->evbit);
	set_bit(KEY_MENU, ts->input_dev_key->keybit);
	set_bit(KEY_BACK, ts->input_dev_key->keybit);
	input_set_drvdata(ts->input_dev_key, ts);
	/* end of - touche key */

	ts->input_dev->evbit[0] = BIT_MASK(EV_ABS) | BIT_MASK(EV_KEY);

	ts->input_dev->keybit[BIT_WORD(KEY_MENU)] |= BIT_MASK(KEY_MENU);
//	ts->input_dev->keybit[BIT_WORD(KEY_HOME)] |= BIT_MASK(KEY_HOME);
	ts->input_dev->keybit[BIT_WORD(KEY_BACK)] |= BIT_MASK(KEY_BACK);
//	ts->input_dev->keybit[BIT_WORD(KEY_SEARCH)] |= BIT_MASK(KEY_SEARCH);

	//__set_bit(BTN_TOUCH, ts->input_dev->keybit);
	//__set_bit(EV_ABS,  ts->input_dev->evbit);
	//ts->input_dev->evbit[0] =  BIT_MASK(EV_SYN) | BIT_MASK(EV_ABS) | BIT_MASK(EV_KEY);	

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, TS_MAX_X_COORD, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, TS_MAX_Y_COORD, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, TS_MAX_Z_TOUCH, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, (MELFAS_MAX_TOUCH - 1), 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, TS_MAX_W_TOUCH, 0, 0);
	//__set_bit(EV_SYN, ts->input_dev->evbit);
	//__set_bit(EV_KEY, ts->input_dev->evbit);

	ret = input_register_device(ts->input_dev);
	if (ret)
	{
		printk(KERN_ERR "melfas_ts_probe: Failed to register touchscreen device\n");
		ret = -ENOMEM;
		goto err_input_register_device_failed;
	}
	ret = input_register_device(ts->input_dev_key);
	if (ret)
	{
		printk(KERN_ERR "melfas_ts_probe: Failed to register touchkey device\n");
		ret = -ENOMEM;
		goto err_input_key_register_device_failed;
	}

	if (ts->client->irq)
	{
#if DEBUG_PRINT
		printk(KERN_INFO "melfas_ts_probe: trying to request irq: %s-%d\n", ts->client->name, ts->client->irq);
#endif
		//ret = request_irq(client->irq, melfas_ts_irq_handler, IRQF_TRIGGER_FALLING, ts->client->name, ts);
		ret = request_threaded_irq(client->irq, NULL, melfas_ts_irq_handler,IRQF_TRIGGER_FALLING | IRQF_ONESHOT, ts->client->name, ts);
		if (ret > 0)
		{
			printk(KERN_ERR "melfas_ts_probe: Can't allocate irq %d, ret %d\n", ts->client->irq, ret);
			ret = -EBUSY;
			goto err_request_irq;
		}
	}

	schedule_work(&ts->work);

	for (i = 0; i < MELFAS_MAX_TOUCH ; i++)  /* _SUPPORT_MULTITOUCH_ */
		g_Mtouch_info[i].strength = -1;	

#if DEBUG_PRINT	
	printk(KERN_INFO "melfas_ts_probe: succeed to register input device\n");
#endif

#if CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = melfas_ts_early_suspend;
	ts->early_suspend.resume = melfas_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif


#if DEBUG_PRINT
	printk(KERN_INFO "melfas_ts_probe: Start touchscreen. name: %s, irq: %d\n", ts->client->name, ts->client->irq);
#endif
	return 0;

err_request_irq:
	printk(KERN_ERR "melfas-ts: err_request_irq failed\n");
	free_irq(client->irq, ts);
err_input_key_register_device_failed:
	printk(KERN_ERR "melfas-ts: err_input_key_register_device failed\n");
	input_free_device(ts->input_dev_key);
err_input_register_device_failed:
	printk(KERN_ERR "melfas-ts: err_input_register_device failed\n");
	input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
	printk(KERN_ERR "melfas-ts: err_input_dev_alloc failed\n");
err_alloc_data_failed:
	printk(KERN_ERR "melfas-ts: err_alloc_data failed_\n");	
err_detect_failed:
	printk(KERN_ERR "melfas-ts: err_detect failed\n");
	kfree(ts);
err_check_functionality_failed:
	printk(KERN_ERR "melfas-ts: err_check_functionality failed_\n");

	return ret;
}

void melfas_ts_release_all_finger(struct i2c_client *client)
{
	int i;

	for (i = 0; i < MELFAS_MAX_TOUCH; i++)
	{
		if (g_Mtouch_info[i].strength == -1)
			continue ;

		if (g_Mtouch_info[i].strength > 0)
		{
			g_Mtouch_info[i].posX = 0;
			g_Mtouch_info[i].posY = 0;
			g_Mtouch_info[i].width = 0;
			g_Mtouch_info[i].strength = 0;
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, g_Mtouch_info[i].posX);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, g_Mtouch_info[i].posY);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, g_Mtouch_info[i].width);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, g_Mtouch_info[i].strength);
			input_mt_sync(ts->input_dev);
		}

		if (g_Mtouch_info[i].strength == 0)
			g_Mtouch_info[i].strength = -1;
	}

	input_report_key(ts->input_dev_key, KEY_MENU, RELEASE_KEY);
//	input_report_key(ts->input_dev, KEY_HOME, RELEASE_KEY);
//	input_report_key(ts->input_dev, KEY_SEARCH, RELEASE_KEY);
	input_report_key(ts->input_dev_key, KEY_BACK, RELEASE_KEY);
	input_sync(ts->input_dev_key);

	input_sync(ts->input_dev);
}

static int melfas_ts_remove(struct i2c_client *client)
{
#if DEBUG_PRINT
	printk(KERN_INFO "melfas_ts_remove\n");
#endif

	unregister_early_suspend(&ts->early_suspend);
	free_irq(client->irq, ts);
	ts->power(false);	// modified
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static int melfas_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;

	printk(KERN_INFO "melfas_ts_suspend\n");

	melfas_ts_release_all_finger(client);
	disable_irq(client->irq);

	ret = cancel_work_sync(&ts->work);
	if (ret) /* if work was pending disable-count is now 2 */
		enable_irq(client->irq);

	melfas_power_panel(0);

	return 0;
}

static int melfas_ts_resume(struct i2c_client *client)
{
	int ret;

	printk(KERN_INFO "melfas_ts_resume\n");

	melfas_power_panel(1);
	enable_irq(client->irq); // scl wave

	return 0;
}

#if CONFIG_HAS_EARLYSUSPEND
static void melfas_ts_early_suspend(struct early_suspend *h)
{
	printk(KERN_INFO "melfas_ts_early_suspend\n");
	melfas_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void melfas_ts_late_resume(struct early_suspend *h)
{
	melfas_ts_resume(ts->client);
	printk(KERN_INFO "melfas_ts_late_resume\n");
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
#if !CONFIG_HAS_EARLYSUSPEND
	.suspend = melfas_ts_suspend,
	.resume = melfas_ts_resume,
#endif
};


#if 0 // FIXME - firmware update
static ssize_t touch_version_read(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	u8 data;
	int count;

	data = i2c_smbus_read_byte_data(info->client, CYPRESS_FW_VER);
	count = sprintf(buf, "0x%02x\n", data);

        printk(KERN_DEBUG "[TouchKey] %s : FW Ver 0x%02x\n", __func__, data);

	return count;
}

static ssize_t touch_version_write(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	printk(KERN_DEBUG "[TouchKey] %s : input data --> %s\n", __func__, buf);

	return size;
}

static ssize_t touch_version_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int count;

	count = sprintf(buf, "0x%02x\n", JANICE_TOUCHKEY_FW_VER);
	return count;
}

static ssize_t touch_update_write(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	printk(KERN_DEBUG "[TouchKey] touchkey firmware update \n");

/*
	if (*buf == 'S') {
		disable_irq(IRQ_TOUCH_INT);
		INIT_WORK(&touch_update_work, touchkey_update_func);
		queue_work(touchkey_wq, &touch_update_work);
	}
*/
	return size;
}

extern int ISSP_main(void);
static int touchkey_update_status = 0;

static ssize_t touch_update_read(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int count = 0;
	int retry = NUM_OF_RETRY_UPDATE;
	u8 data;
	
	touchkey_update_status = 1;

        disable_irq(info->irq);
	while (retry--) {
		if (ISSP_main() == 0) {
			printk(KERN_ERR "[TOUCHKEY] Update success!\n");
			touchkey_update_status = 0;
			count = 1;
			enable_irq(info->irq);
			break;
		}
		printk(KERN_ERR "[TOUCHKEY] Touchkey_update failed... retry...\n");
	}
	
	if (retry <= 0) {
		cypress_touchkey_con_hw(info, false);
		msleep(300);
		count = 0;
		printk(KERN_ERR "[TOUCHKEY]Touchkey_update fail\n");
		touchkey_update_status = -1;
		return count;
	}

	msleep(500);

	data = i2c_smbus_read_byte_data(info->client, CYPRESS_FW_VER);
	count = sprintf(buf, "0x%02x\n", data);
	printk(KERN_DEBUG "[TouchKey] %s : FW Ver 0x%02x\n", __func__, data);

	return count;
}
#endif


#if 0 // FIXME - touchkey sensitivity 
static ssize_t touchkey_menu_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	u8 data[2] = {0,};
	int ret;

	ret = i2c_smbus_write_byte_data(info->client, CYPRESS_GEN, CYPRESS_DATA_UPDATE);
	if (ret < 0) {
		dev_err(&info->client->dev, "[Touchkey] fail to update menukey sensitivity data.\n");
		return ret;
	}
	msleep(150);
	ret = i2c_smbus_read_i2c_block_data(info->client, CYPRESS_DIFF_MENU,
		ARRAY_SIZE(data), data);
	if (ret != ARRAY_SIZE(data)) {
		dev_err(&info->client->dev, "[TouchKey] fail to read menu sensitivity.\n");
		return ret;		
	}
	menu_sensitivity = ((0x00FF & data[0])<<8) | data[1];

	printk("called %s , data : %d %d\n", __func__, data[0], data[1]);
	return sprintf(buf,"%d\n",menu_sensitivity);

}

static ssize_t touchkey_back_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u8 data[2] = {0,};
	int ret;

	ret = i2c_smbus_write_byte_data(info->client, CYPRESS_GEN, CYPRESS_DATA_UPDATE);
	if (ret < 0) {
		dev_err(&info->client->dev, "[Touchkey] fail to update backkey sensitivity data.\n");
		return ret;
	}
	msleep(150);
	ret = i2c_smbus_read_i2c_block_data(info->client, CYPRESS_DIFF_BACK,
		ARRAY_SIZE(data), data);
	if (ret != ARRAY_SIZE(data)) {
		dev_err(&info->client->dev, "[TouchKey] fail to read back sensitivity.\n");
		return ret;		
	}
	
	back_sensitivity = ((0x00FF & data[0])<<8) | data[1];

	printk("called %s , data : %d %d\n", __func__, data[0], data[1]);
	return sprintf(buf, "%d\n", back_sensitivity);

}

static ssize_t touchkey_raw_data0_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u8 data[2] = {0,};
	int ret;

	ret = i2c_smbus_write_byte_data(info->client, CYPRESS_GEN, CYPRESS_DATA_UPDATE);
	if (ret < 0) {
		dev_err(&info->client->dev, "[Touchkey] fail to update raw data0.\n");
		return ret;
	}
	msleep(150);
	ret = i2c_smbus_read_i2c_block_data(info->client, CYPRESS_RAW_DATA_MENU,
		ARRAY_SIZE(data), data);
	if (ret != ARRAY_SIZE(data)) {
		dev_err(&info->client->dev, "[TouchKey] fail to read MENU raw data.\n");
		return ret;		
	}
	
	raw_data0 = ((0x00FF & data[0])<<8) | data[1];

	printk("called %s , data : %d %d\n", __func__, data[0], data[1]);
	return sprintf(buf, "%d\n", raw_data0);

}

static ssize_t touchkey_raw_data1_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u8 data[2] = {0,};
	int ret;

	ret = i2c_smbus_write_byte_data(info->client, CYPRESS_GEN, CYPRESS_DATA_UPDATE);
	if (ret < 0) {
		dev_err(&info->client->dev, "[Touchkey] fail to update raw data1.\n");
		return ret;
	}
	msleep(150);
	ret = i2c_smbus_read_i2c_block_data(info->client, CYPRESS_RAW_DATA_BACK,
		ARRAY_SIZE(data), data);
	if (ret != ARRAY_SIZE(data)) {
		dev_err(&info->client->dev, "[TouchKey] fail to read BACK raw data.\n");
		return ret;		
	}
	
	raw_data1 = ((0x00FF & data[0])<<8) | data[1];

	printk("called %s , data : %d %d\n", __func__, data[0], data[1]);
	return sprintf(buf, "%d\n", raw_data1);

}

static ssize_t touchkey_idac0_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u8 data = 0;
	int ret;

	ret = i2c_smbus_write_byte_data(info->client, CYPRESS_GEN, CYPRESS_DATA_UPDATE);
	if (ret < 0) {
		dev_err(&info->client->dev, "[Touchkey] fail to update idac0.\n");
		return ret;
	}
	msleep(150);

	data = i2c_smbus_read_byte_data(info->client, CYPRESS_IDAC_MENU);

	printk("called %s , data : %d\n", __func__, data);
	idac0 = data;
	return sprintf(buf, "%d\n", idac0);

}

static ssize_t touchkey_idac1_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u8 data = 0;
	int ret;

	ret = i2c_smbus_write_byte_data(info->client, CYPRESS_GEN, CYPRESS_DATA_UPDATE);
	if (ret < 0) {
		dev_err(&info->client->dev, "[Touchkey] fail to update idac1.\n");
		return ret;
	}
	msleep(150);

	data = i2c_smbus_read_byte_data(info->client, CYPRESS_IDAC_BACK);

	printk("called %s , data : %d\n", __func__, data);
	idac1 = data;
	return sprintf(buf, "%d\n", idac1);

}

static ssize_t touchkey_threshold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u8 data = 0;

	data = i2c_smbus_read_byte_data(info->client, CYPRESS_THRESHOLD);

	printk("called %s , data : %d\n", __func__, data);
	touchkey_threshold = data;
	return sprintf(buf, "%d\n", touchkey_threshold);
}

static ssize_t touch_autocal_testmode(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t size)
{
	int count = 0;
	int on_off;	

	if (sscanf(buf, "%d\n", &on_off) == 1) {
		printk(KERN_ERR "[TouchKey] Test Mode : %d \n", on_off);

		if (on_off == 1) {
			count = i2c_smbus_write_byte_data(info->client,
				CYPRESS_GEN, CYPRESS_DATA_UPDATE);
		}
		else {
			cypress_touchkey_con_hw(info, false);
			msleep(50);
			cypress_touchkey_con_hw(info, true);
			msleep(50);
			cypress_touchkey_auto_cal(info);
		}
	}
	else
		printk(KERN_ERR "[TouchKey] touch_led_brightness Error\n");

	return count;
}

static ssize_t touch_sensitivity_control(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t size)
{
	u8 data[] = {CYPRESS_DATA_UPDATE};
	int ret;

	printk("called %s \n", __func__);
	
	ret = i2c_smbus_write_i2c_block_data(info->client, CYPRESS_GEN, ARRAY_SIZE(data), data);
	
	return ret;
}
#endif

#if 0 // FIXME - firmware update
static DEVICE_ATTR(touchkey_firm_version_panel, S_IRUGO | S_IWUSR | S_IWGRP, touch_version_read, touch_version_write);
static DEVICE_ATTR(touchkey_firm_version_phone, S_IRUGO | S_IWUSR | S_IWGRP, touch_version_show, NULL);
static DEVICE_ATTR(touchkey_firm_update, S_IRUGO | S_IWUSR | S_IWGRP, touch_update_read, touch_update_write);
#endif

#if 0 // FIXME - touchkey sensitivity 
static DEVICE_ATTR(touchkey_menu, S_IRUGO, touchkey_menu_show, NULL);
static DEVICE_ATTR(touchkey_back, S_IRUGO, touchkey_back_show, NULL);
static DEVICE_ATTR(touchkey_raw_data0, S_IRUGO, touchkey_raw_data0_show, NULL);
static DEVICE_ATTR(touchkey_raw_data1, S_IRUGO, touchkey_raw_data1_show, NULL);
static DEVICE_ATTR(touchkey_idac0, S_IRUGO, touchkey_idac0_show, NULL);
static DEVICE_ATTR(touchkey_idac1, S_IRUGO, touchkey_idac1_show, NULL);
static DEVICE_ATTR(touchkey_threshold, S_IRUGO, touchkey_threshold_show, NULL);
static DEVICE_ATTR(touch_sensitivity, S_IRUGO | S_IWUSR | S_IWGRP, NULL, touch_sensitivity_control);
static DEVICE_ATTR(touchkey_autocal_start, S_IRUGO | S_IWUSR | S_IWGRP, NULL, touch_autocal_testmode);
#endif

void create_sys_node_touchkey(void)
{
	struct device *sec_touchkey;

	sec_touchkey = device_create(sec_class, NULL, 0, NULL, "sec_touchkey");

	if (IS_ERR(sec_touchkey)) {
			printk(KERN_ERR "Failed to create device(sec_touchkey)!\n");
	}
	
#if 0 // FIXME - firmware update
	if (device_create_file(sec_touchkey, &dev_attr_touchkey_firm_update) < 0) {
		printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_touchkey_firm_update.attr.name);
	}
	
	if (device_create_file(sec_touchkey, &dev_attr_touchkey_firm_version_panel) < 0) {
		printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_touchkey_firm_version_panel.attr.name);
	}

	if (device_create_file(sec_touchkey, &dev_attr_touchkey_firm_version_phone) < 0) {
		printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_touchkey_firm_version_phone.attr.name);
	}
#endif
	

#if 0 // FIXME - touchkey sensitivity 
	if (device_create_file(sec_touchkey, &dev_attr_touchkey_menu) < 0) {
		printk(KERN_ERR "%s device_create_file fail dev_attr_touchkey_menu\n", __func__);
	}

	if (device_create_file(sec_touchkey, &dev_attr_touchkey_back) < 0) {
		printk(KERN_ERR "%s device_create_file fail dev_attr_touchkey_back\n", __func__);
	}

	if (device_create_file(sec_touchkey, &dev_attr_touchkey_raw_data0) < 0) {
		printk(KERN_ERR "%s device_create_file fail dev_attr_touchkey_raw_data0\n", __func__);
	}

	if (device_create_file(sec_touchkey, &dev_attr_touchkey_raw_data1) < 0) {
		printk(KERN_ERR "%s device_create_file fail dev_attr_touchkey_raw_data1\n", __func__);
	}

	if (device_create_file(sec_touchkey, &dev_attr_touchkey_idac0) < 0) {
		printk(KERN_ERR "%s device_create_file fail dev_attr_touchkey_idac0\n", __func__);
	}

	if (device_create_file(sec_touchkey, &dev_attr_touchkey_idac1) < 0) {
		printk(KERN_ERR "%s device_create_file fail dev_attr_touchkey_idac1\n", __func__);
	}

	if (device_create_file(sec_touchkey, &dev_attr_touchkey_threshold) < 0) {
		printk(KERN_ERR "%s device_create_file fail dev_attr_touchkey_threshold\n", __func__);
	}

	if (device_create_file(sec_touchkey, &dev_attr_touchkey_autocal_start) < 0) {
		printk(KERN_ERR "%s device_create_file fail dev_attr_touchkey_autocal_start\n", __func__);
	}
	
	
	if (device_create_file(sec_touchkey, &dev_attr_touch_sensitivity) < 0) {
		printk("%s device_create_file fail dev_attr_touch_sensitivity\n", __func__);
	}
#endif
}

void create_sys_node_touchscreen(void)
{
	struct device *sec_touchscreen;

#if 0
	int ret;

	ret = sysfs_create_group(&client->dev.kobj, &mxt224_attr_group);
	if (ret)
		printk(KERN_ERR"[TSP] sysfs_create_group()is falled\n");
#endif
	sec_touchscreen = device_create(sec_class, NULL, 0, NULL, "sec_touchscreen");
	dev_set_drvdata(sec_touchscreen, ts);
	if (IS_ERR(sec_touchscreen))
		printk(KERN_ERR "[TSP] Failed to create device(sec_touchscreen)!\n");
}

static int __devinit melfas_ts_init(void)
{
	int ret = 0;

	create_sys_node_touchkey();
	create_sys_node_touchscreen();

	ret = i2c_add_driver(&melfas_ts_driver);
	if (ret) {
		printk(KERN_ERR "[TouchKey] melfas touchscreen registration failed, module not inserted.ret= %d\n", ret);
	}
	return ret;
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


