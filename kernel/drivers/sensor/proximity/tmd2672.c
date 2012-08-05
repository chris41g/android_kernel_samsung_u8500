/*
 * Linux driver driver for proximity sensor TMD2672
 * ----------------------------------------------------------------------------
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/uaccess.h>
#include <linux/unistd.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/wakelock.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/earlysuspend.h>

#include <mach/tmd2672.h>
#include <linux/regulator/consumer.h>
#include "tmd2672_reg.h"

struct taos_data {
	struct input_dev *proximity_input_dev;
	struct i2c_client *client;
	struct work_struct work_prox;
	struct work_struct work_ptime;
	struct workqueue_struct *taos_wq;
	struct workqueue_struct *taos_test_wq;
	struct hrtimer timer;
	struct regulator *regulator_vcc;
	struct regulator *regulator_vio;
	struct wake_lock prx_wake_lock;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif

	ktime_t prox_polling_time;

	bool proximity_enable;
	bool factorytest_enable;
	short proximity_value;
	int avg[3];
	int	gpio;
	int	alsout;
	int irq;
};

static int opt_i2c_write(struct taos_data *taos, u8 reg, u8 * val)
{
	int err;
	unsigned char data[2];
	struct i2c_msg msg[1];

	if (taos->client == NULL)
		return -ENODEV;

	data[0] = reg;
	data[1] = *val;

	msg->addr = taos->client->addr;
	msg->flags = 0;
	msg->len = 2;
	msg->buf = data;

	err = i2c_transfer(taos->client->adapter, msg, 1);
	if (err >= 0)
		return 0;
	pr_err("%s: i2c transfer error : reg = [%X], err is %d\n",
			__func__, reg, err);
	return err;
}

static int taos_poweron(struct taos_data *taos)
{
	func_dbg();

	if (!taos)
		return -ENODEV;

	if (taos->regulator_vcc)
		regulator_enable(taos->regulator_vcc);
	if (taos->regulator_vio)
		regulator_enable(taos->regulator_vio);

	msleep(20);
	return 0;
}

static int taos_poweroff(struct taos_data *taos)
{
	func_dbg();

	if (!taos)
		return -ENODEV;

	if (taos->regulator_vcc)
		regulator_disable(taos->regulator_vcc);
	if (taos->regulator_vio)
		regulator_disable(taos->regulator_vio);

	return 0;
}

static void taos_work_func_prox(struct work_struct *work)
{
	struct taos_data *taos = container_of(work,
		struct taos_data, work_prox);
	u16 adc_data;
	u16 threshold_high;
	u16 threshold_low;
	u8 prox_int_thresh[4];
	int i;

	/* change Threshold */
	adc_data = i2c_smbus_read_word_data(taos->client, CMD_REG | 0x18);
	threshold_high = i2c_smbus_read_word_data(taos->client,
						(CMD_REG | PRX_MAXTHRESHLO));
	threshold_low = i2c_smbus_read_word_data(taos->client,
						(CMD_REG | PRX_MINTHRESHLO));
	printk(KERN_INFO "[PROXIMITY] %s: adc(0x%x), high(0x%x), low(0x%x)\n",
			__func__, adc_data, threshold_high, threshold_low);

	if ((threshold_high == (PRX_THRSH_HI_PARAM))
			 && (adc_data >= (PRX_THRSH_HI_PARAM))) {
		taos->proximity_value = ON;
		prox_int_thresh[0] = (PRX_THRSH_LO_PARAM) & 0xFF;
		prox_int_thresh[1] = (PRX_THRSH_LO_PARAM >> 8) & 0xFF;
		prox_int_thresh[2] = (0xFFFF) & 0xFF;
		prox_int_thresh[3] = (0xFFFF >> 8) & 0xFF;
		for (i = 0; i < 4; i++)
			opt_i2c_write(taos, (CMD_REG | (PRX_MINTHRESHLO + i)),
				       &prox_int_thresh[i]);
	} else if ((threshold_high == (0xFFFF))
					&& (adc_data <= (PRX_THRSH_LO_PARAM))) {
		taos->proximity_value = OFF;
		prox_int_thresh[0] = (0x0000) & 0xFF;
		prox_int_thresh[1] = (0x0000 >> 8) & 0xFF;
		prox_int_thresh[2] = (PRX_THRSH_HI_PARAM) & 0xFF;
		prox_int_thresh[3] = (PRX_THRSH_HI_PARAM >> 8) & 0xFF;
		for (i = 0; i < 4; i++)
			opt_i2c_write(taos, (CMD_REG | (PRX_MINTHRESHLO + i)),
				       &prox_int_thresh[i]);
	} else
		pr_err("%s: Not Common Case!", __func__);

	input_report_abs(taos->proximity_input_dev, ABS_DISTANCE,
			!taos->proximity_value);
	input_sync(taos->proximity_input_dev);
	mdelay(1);

    /* reset Interrupt pin */
    /* to active Interrupt, TMD2771x Interuupt pin shoud be reset. */
	i2c_smbus_write_byte(taos->client, (CMD_REG
			| CMD_SPL_FN | CMD_PROX_INTCLR));

    /* enable INT */
	enable_irq(taos->irq);
}

static irqreturn_t taos_irq_handler(int irq, void *dev_id)
{
	struct taos_data *taos = dev_id;

	printk(KERN_INFO "[PROXIMITY] taos->irq = %d\n", taos->irq);

	if (taos->irq != -1) {
		wake_lock_timeout(&taos->prx_wake_lock, 3 * HZ);
		disable_irq_nosync(taos->irq);
		queue_work(taos->taos_wq, &taos->work_prox);
	}
	return IRQ_HANDLED;
}

static void taos_work_func_ptime(struct work_struct *work)
{
	struct taos_data *taos =
			container_of(work, struct taos_data, work_ptime);
	u16 value = 0;
	int min = 0, max = 0, avg = 0;
	int i = 0;

	for (i = 0; i < PROX_READ_NUM; i++) {
		if (taos->proximity_enable == OFF)
			break;

		value = i2c_smbus_read_word_data(taos->client,
					CMD_REG | PRX_LO);
		if (value > TAOS_PROX_MIN) {
			if (value > TAOS_PROX_MAX)
				value = TAOS_PROX_MAX;
			avg += value;
			if (!i)
				min = value;
			else if (value < min)
				min = value;
			if (value > max)
				max = value;
		} else {
			value = TAOS_PROX_MIN;
			break;
		}
		msleep(40);
	}

	if (i != 0)
		avg /= i;
	taos->avg[0] = min;
	taos->avg[1] = avg;
	taos->avg[2] = max;
}

static enum hrtimer_restart taos_timer_func(struct hrtimer *timer)
{
	struct taos_data *taos =
	    container_of(timer, struct taos_data, timer);

	queue_work(taos->taos_test_wq, &taos->work_ptime);
	hrtimer_forward_now(&taos->timer, taos->prox_polling_time);
	return HRTIMER_RESTART;
}

void taos_chip_on(struct taos_data *taos)
{
	u8 value;
	u8 prox_int_thresh[4];
	int err = 0;
	int i;
	int fail_num = 0;

	value = CNTL_REG_CLEAR;
	err = opt_i2c_write(taos, (CMD_REG | CNTRL), &value);
	if (err < 0) {
		pr_err("%s: clr ctrl reg failed\n", __func__);
		fail_num++;
	}
	value = PRX_ADC_TIME_PARAM;
	err = opt_i2c_write(taos, (CMD_REG | PRX_TIME), &value);
	if (err < 0) {
		pr_err("%s: prox time reg failed\n", __func__);
		fail_num++;
	}
	value = PRX_WAIT_TIME_PARAM;
	err = opt_i2c_write(taos, (CMD_REG | WAIT_TIME), &value);
	if (err < 0) {
		pr_err("%s: wait time reg failed\n", __func__);
		fail_num++;
	}
	value = INTR_FILTER_PARAM;
	err = opt_i2c_write(taos, (CMD_REG | INTERRUPT), &value);
	if (err < 0) {
		pr_err("%s: interrupt reg failed\n", __func__);
		fail_num++;
	}
	value = PRX_CONFIG_PARAM;
	err = opt_i2c_write(taos, (CMD_REG | PRX_CFG), &value);
	if (err < 0) {
		pr_err("%s: prox cfg reg failed\n", __func__);
		fail_num++;
	}
	value = PRX_PULSE_CNT_PARAM;
	err = opt_i2c_write(taos, (CMD_REG | PRX_COUNT), &value);
	if (err < 0) {
		pr_err("%s: prox cnt reg failed\n", __func__);
		fail_num++;
	}

	value = PRX_GAIN_PARAM;	/* 100mA, ch1, pgain 4x, again 1x */
	err = opt_i2c_write(taos, (CMD_REG | GAIN), &value);
	if (err < 0) {
		pr_err("%s: prox gain reg failed\n", __func__);
		fail_num++;
	}

	value = PRX_GAIN_OFFSET;	/* 100 */
	err = opt_i2c_write(taos, (CMD_REG | PRX_OFFSET), &value);
	if (err < 0) {
		pr_err("%s: prox gain reg failed\n", __func__);
		fail_num++;
	}

	prox_int_thresh[0] = (0x0000) & 0xFF;
	prox_int_thresh[1] = (0x0000 >> 8) & 0xFF;
	prox_int_thresh[2] = (PRX_THRSH_HI_PARAM) & 0xFF;
	prox_int_thresh[3] = (PRX_THRSH_HI_PARAM >> 8) & 0xFF;

	for (i = 0; i < 4; i++) {
		err = opt_i2c_write(taos, (CMD_REG | (PRX_MINTHRESHLO + i)),
					&prox_int_thresh[i]);
		if (err < 0) {
			pr_err("%s: prox int thrsh reg failed\n", __func__);
			fail_num++;
		}
	}

	value = CNTL_INTPROXPON_ENBL;
	err = opt_i2c_write(taos, (CMD_REG | CNTRL), &value);
	if (err < 0) {
		pr_err("%s: ctrl reg failed\n", __func__);
		fail_num++;
	}

	mdelay(12);
	if (fail_num == 0) {
		err = set_irq_wake(taos->irq, 1);
		if (err)
			pr_err("%s: register wakeup source failed\n", __func__);
	} else
		pr_err("%s: I2C failed in taos_chip_on, fail I2C=[%d]\n",
			__func__, fail_num);
}

static int taos_chip_off(struct taos_data *taos)
{
	u8 reg_cntrl;
	int ret = 0;
	int err = 0;

	err = set_irq_wake(taos->irq, 0);
	if (err)
		pr_err("%s: register wakeup source failed\n", __func__);

	reg_cntrl = CNTL_REG_CLEAR;
	ret = opt_i2c_write(taos, (CMD_REG | CNTRL), &reg_cntrl);
	if (ret < 0) {
		pr_err("%s: opt_i2c_write to ctrl reg failed\n", __func__);
		return ret;
	}

	return ret;
}

void taos_on(struct taos_data *taos)
{
	func_dbg();

	taos_chip_on(taos);

	enable_irq(taos->irq);
	taos->proximity_enable = ON;
}

void taos_off(struct taos_data *taos)
{
	func_dbg();

	disable_irq(taos->irq);
	cancel_work_sync(&taos->work_prox);

	taos->factorytest_enable = OFF;
	taos->proximity_enable = OFF;
	taos->proximity_value = OFF;
	taos_chip_off(taos);
}

/************************************************************************/
/*  TAOS sysfs															*/
/************************************************************************/
static ssize_t proximity_enable_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct taos_data *taos = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", (taos->proximity_enable) ? 1 : 0);
}

static ssize_t proximity_enable_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct taos_data *taos = dev_get_drvdata(dev);
	int value;
	sscanf(buf, "%d", &value);

	if ((value == 1) && (taos->proximity_enable == OFF)) {
#if 0
		taos_poweron(taos);
#endif
	/* reset Interrupt pin */
	/* to active Interrupt, TMD2771x Interuupt pin shoud be reset. */
		i2c_smbus_write_byte(taos->client,
				(CMD_REG | CMD_SPL_FN |
						CMD_PROX_INTCLR));
		taos_on(taos);
		input_report_abs(taos->proximity_input_dev, ABS_DISTANCE,
				!taos->proximity_value);
		input_sync(taos->proximity_input_dev);
	} else if (value == 0 && taos->proximity_enable == ON) {
		taos_off(taos);
#if 0
		taos_poweroff(taos);
#endif
	}

	return size;
}

static ssize_t proximity_adc_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct taos_data *taos = dev_get_drvdata(dev);
	u16 value = 0;

	if (taos->proximity_enable == ON) {
		value = i2c_smbus_read_word_data(taos->client,
					CMD_REG | PRX_LO);
		if (value > TAOS_PROX_MAX)
			value = TAOS_PROX_MAX;
	}

	return sprintf(buf, "%d\n", value);
}

static ssize_t proximity_avg_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct taos_data *taos = dev_get_drvdata(dev);

	return sprintf(buf, "%d,%d,%d\n", taos->avg[0], taos->avg[1],
				taos->avg[2]);
}

static ssize_t proximity_avg_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct taos_data *taos = dev_get_drvdata(dev);
	int value;
	sscanf(buf, "%d", &value);

	if ((value == 1) && (taos->factorytest_enable == OFF)) {
		hrtimer_start(&taos->timer, taos->prox_polling_time,
				HRTIMER_MODE_REL);
		taos->factorytest_enable = ON;
	} else if ((value == 0) && (taos->factorytest_enable == ON)) {
		hrtimer_cancel(&taos->timer);
		cancel_work_sync(&taos->work_ptime);
		taos->factorytest_enable = OFF;
	}

	return size;
}

static DEVICE_ATTR(proximity_avg, 0644, proximity_avg_show,
			proximity_avg_store);
static DEVICE_ATTR(adc, 0644, proximity_adc_show, NULL);
static DEVICE_ATTR(state, 0644, proximity_adc_show, NULL);
/*
static DEVICE_ATTR(state, 0644, proximity_data_show, NULL);
*/

static struct device_attribute *proximity_sensor_attrs[] = {
	&dev_attr_state,
	&dev_attr_adc,
	&dev_attr_proximity_avg,
	NULL,
};

static struct device_attribute dev_attr_proximity_enable =
	__ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP, proximity_enable_show,
		proximity_enable_store);

static struct attribute *proximity_sysfs_attrs[] = {
	&dev_attr_proximity_enable.attr,
	NULL
};

static struct attribute_group proximity_attribute_group = {
	.attrs = proximity_sysfs_attrs,
};

static int taos_opt_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	int err = 0;

	struct taos_data *taos;
	struct input_dev *input_dev;
	struct tmd2672_platform_data *pdata = client->dev.platform_data;
	struct device *proximity_sensor_device = NULL;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: i2c_check_functionality error.\n", __func__);
		err = -ENODEV;
		goto exit;
	}

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		pr_err("%s: byte op is not permited.\n", __func__);
		goto exit;
	}

	taos = kzalloc(sizeof(struct taos_data), GFP_KERNEL);
	if (taos == NULL) {
		err = -ENOMEM;
		goto exit;
	}

	dev_set_name(&client->dev, client->name);
	taos->gpio = pdata->ps_vout_gpio;
	taos->alsout = pdata->alsout;
	taos->client = client;
	i2c_set_clientdata(client, taos);

	if (pdata->hw_setup) {
		err = pdata->hw_setup();
		if (err < 0) {
			pr_err("%s: Failed to setup HW [errno=%d]",
					__func__, err);
			goto exit_kfree;
		}
	}

	/* regulator output enable/disable control */
	taos->regulator_vcc = regulator_get(&client->dev, "v-prox-vcc");
	if (IS_ERR(taos->regulator_vcc)) {
		pr_err("%s: Failed to get v-prox-vcc regulator for tmd2672\n",
				__func__);
		err = PTR_ERR(taos->regulator_vcc);
		taos->regulator_vcc = NULL;
		goto exit_kfree;
	}

	taos->regulator_vio = regulator_get(&client->dev, "v-prox_vio");
	if (IS_ERR(taos->regulator_vio)) {
		pr_err("%s: Failed to get v-prox_vio regulator for tmd2672\n",
				__func__);
		err = PTR_ERR(taos->regulator_vio);
		taos->regulator_vio = NULL;
		goto exit_kfree;
	}

	taos_poweron(taos);

	err = i2c_smbus_read_byte_data(taos->client, CMD_REG | CHIPID);
	printk(KERN_INFO "[PROXIMITY] %s: chipID[%X]\n", __func__, err);
	if (err < 0)
		goto exit_kfree;

	/* INT Settings */
	taos->irq = gpio_to_irq(taos->gpio);
	if (taos->irq < 0) {
		err = taos->irq;
		pr_err("%s: Failed to convert GPIO %u to IRQ [errno=%d]",
				__func__, taos->gpio, err);
		goto exit_kfree;
	}
	err = request_threaded_irq(taos->irq, NULL, taos_irq_handler,
				 IRQF_DISABLED | IRQ_TYPE_EDGE_FALLING,
				 "taos_int", taos);
	if (err) {
		pr_err("%s: request_irq failed for taos\n", __func__);
		goto exit_kfree;
	}
	set_irq_wake(taos->irq, 1);

	/* wake lock init */
	wake_lock_init(&taos->prx_wake_lock, WAKE_LOCK_SUSPEND,
			"prx_wake_lock");

	 /* allocate proximity input_device */
	input_dev = input_allocate_device();
	if (input_dev == NULL) {
		pr_err("%s: Failed to allocate input device\n", __func__);
		err = -ENOMEM;
		goto err_input_allocate_device_proximity;
	}

	taos->proximity_input_dev = input_dev;
	input_set_drvdata(input_dev, taos);
	input_dev->name = "proximity_sensor";
	input_set_capability(input_dev, EV_ABS, ABS_DISTANCE);
	input_set_abs_params(input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	err = input_register_device(input_dev);
	if (err) {
		pr_err("%s: Unable to register %s input device\n",
				__func__, input_dev->name);
		goto err_input_register_device_proximity;
	}

	err = sysfs_create_group(&input_dev->dev.kobj,
				&proximity_attribute_group);
	if (err) {
		pr_err("%s: could not create sysfs group\n", __func__);
		goto err_sysfs_create_group_proximity;
	}

	/* WORK QUEUE Settings */
	taos->taos_wq = create_singlethread_workqueue("taos_wq");
	if (!taos->taos_wq) {
		err = -ENOMEM;
		pr_err("%s: could not create workqueue\n", __func__);
		goto err_create_workqueue;
	}
	INIT_WORK(&taos->work_prox, taos_work_func_prox);
	taos->taos_test_wq = create_singlethread_workqueue("taos_test_wq");
	if (!taos->taos_test_wq) {
		err = -ENOMEM;
		pr_err("%s: could not create workqueue\n", __func__);
		destroy_workqueue(taos->taos_wq);
		goto err_create_workqueue;
	}
	INIT_WORK(&taos->work_ptime, taos_work_func_ptime);

	    /* hrtimer settings.  we poll for factory test using a timer. */
	hrtimer_init(&taos->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	taos->prox_polling_time = ns_to_ktime(2000 * NSEC_PER_MSEC);
	taos->timer.function = taos_timer_func;

	err = sensors_register(proximity_sensor_device, taos,
				proximity_sensor_attrs, "proximity_sensor");
	if (err < 0) {
		pr_err("%s: could not register gyro sensor device(%d).\n",
					__func__, err);
		goto err_sensor_sysfs_create;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	taos->early_suspend.suspend = taos_early_suspend;
	taos->early_suspend.resume = taos_early_resume;
	register_early_suspend(&taos->early_suspend);
#endif

	taos_off(taos);
	func_dbg();
	return 0;

err_sensor_sysfs_create:
	destroy_workqueue(taos->taos_wq);
	destroy_workqueue(taos->taos_test_wq);
err_create_workqueue:
	sysfs_remove_group(&taos->proximity_input_dev->dev.kobj,
				&proximity_attribute_group);
err_sysfs_create_group_proximity:
	input_unregister_device(taos->proximity_input_dev);
err_input_register_device_proximity:
	input_free_device(taos->proximity_input_dev);
err_input_allocate_device_proximity:
	wake_lock_destroy(&taos->prx_wake_lock);
	free_irq(taos->irq, taos);
exit_kfree:
	kfree(taos);
exit:
	return err;
}

static int taos_opt_remove(struct i2c_client *client)
{
	struct taos_data *taos = i2c_get_clientdata(client);

	func_dbg();

	sysfs_remove_group(&taos->proximity_input_dev->dev.kobj,
			    &proximity_attribute_group);
	input_unregister_device(taos->proximity_input_dev);

	if (taos->taos_wq)
		destroy_workqueue(taos->taos_wq);
	if (taos->taos_test_wq)
		destroy_workqueue(taos->taos_test_wq);
	wake_lock_destroy(&taos->prx_wake_lock);
	kfree(taos);
	return 0;
}

static void taos_opt_shutdown(struct i2c_client *client)
{
	struct taos_data *taos = i2c_get_clientdata(client);

	func_dbg();

	sysfs_remove_group(&taos->proximity_input_dev->dev.kobj,
			    &proximity_attribute_group);
	input_unregister_device(taos->proximity_input_dev);

	if (taos->taos_wq)
		destroy_workqueue(taos->taos_wq);
	if (taos->taos_test_wq)
		destroy_workqueue(taos->taos_test_wq);
	wake_lock_destroy(&taos->prx_wake_lock);
	kfree(taos);
}

static int taos_opt_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct taos_data *taos = i2c_get_clientdata(client);

	func_dbg();

	if (taos->factorytest_enable == ON) {
		hrtimer_cancel(&taos->timer);
		cancel_work_sync(&taos->work_ptime);
	}

	return 0;
}
static int taos_opt_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct taos_data *taos = i2c_get_clientdata(client);

	func_dbg();

	if (taos->factorytest_enable == ON)
		hrtimer_start(&taos->timer, taos->prox_polling_time,
				HRTIMER_MODE_REL);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void taos_early_suspend(struct early_suspend *handler)
{
	struct taos_data *taos;
	taos = container_of(handler, struct taos_data, early_suspend);

	func_dbg();

	if (taos->factorytest_enable == ON) {
		hrtimer_cancel(&taos->timer);
		cancel_work_sync(&taos->work_ptime);
	}
}

static void taos_early_resume(struct early_suspend *handler)
{
	struct taos_data *taos;
	taos = container_of(handler, struct taos_data, early_suspend);

	func_dbg();

	if (taos->factorytest_enable == ON)
		hrtimer_start(&taos->timer, taos->prox_polling_time,
				HRTIMER_MODE_REL);
}
#endif

static const struct i2c_device_id taos_id[] = {
	{"tmd2672_prox", 0}, {}
};

MODULE_DEVICE_TABLE(i2c, taos_id);
static const struct dev_pm_ops taos_pm_ops = {
	.suspend = taos_opt_suspend,
	.resume = taos_opt_resume,
};

static struct i2c_driver taos_opt_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "tmd2672_prox",
#ifndef CONFIG_HAS_EARLYSUSPEND
		.pm = &taos_pm_ops
#endif
	},
	.probe = taos_opt_probe,
	.remove = taos_opt_remove,
	.shutdown = taos_opt_shutdown,
	.id_table = taos_id,
};

static int __init taos_opt_init(void)
{
	return i2c_add_driver(&taos_opt_driver);
}

static void __exit taos_opt_exit(void)
{
	i2c_del_driver(&taos_opt_driver);
}

module_init(taos_opt_init);
module_exit(taos_opt_exit);
MODULE_AUTHOR("SAMSUNG");
MODULE_DESCRIPTION("Optical Sensor driver for TMD2672");
MODULE_LICENSE("GPL");
