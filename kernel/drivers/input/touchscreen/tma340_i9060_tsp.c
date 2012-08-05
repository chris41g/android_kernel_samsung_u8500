/*
 * Samsung GT-I9060 Touchscreen panel driver.
 *
 * Author: Robert Teather  <robert.teather@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/device.h>
#include <linux/wakelock.h>
#ifdef CONFIG_LEDS_CLASS
#include <linux/leds.h>
#endif
#include <linux/gpio.h>
#include <plat/pincfg.h>

#include <linux/input/tma340_i9060_tsp.h>

#define TMA340_TSP_FACTORY_TEST 1

#define MAX_KEYS	4
#define MAX_USING_FINGER_NUM 2
 
#define MENU_KEY_IDX 0
#define BACK_KEY_IDX 1
static int touchkey_keycodes[MAX_KEYS] = {
			 KEY_MENU,
			 KEY_BACK,
};
 
#define TK_STATUS_PRESS		1
#define TK_STATUS_RELEASE	0

#ifdef CONFIG_LEDS_CLASS
#define TOUCHKEY_BACKLIGHT	"button-backlight"
#endif
 
typedef struct
{
	 int8_t id;  	/*!< (id>>8) + size */
	 int8_t status; /*!< dn>0, up=0, none=-1 */ 
	 int8_t z;	 	/*!width */
	 int16_t x; 	/*!< X */
	 int16_t y; 	/*!< Y */
} report_finger_info_t;
 
struct tma340_ts_data {
	int 					touchkey_status[MAX_KEYS];
	report_finger_info_t 	fingerInfo[MAX_USING_FINGER_NUM+1];
	struct i2c_client 		*client;
	struct input_dev 		*input_dev;
	int 					use_irq;
	struct regulator 		*tsp_1v8_reg;
	struct regulator		*tsp_3v_reg;
	struct hrtimer 			timer;
	struct work_struct  	work;
	int 					prev_wdog_val;
	int						check_ic_counter;
	struct work_struct  	work_timer;
	struct early_suspend 	early_suspend;
	struct workqueue_struct *tma340_wq;
#ifdef CONFIG_LEDS_CLASS
	struct led_classdev 	leds;
	enum led_brightness		brightness;
	struct work_struct 		led_work;
#endif
	struct wake_lock		update_wakelock;
	struct work_struct		work_update;
	struct tma340_tsp_platform_data pdata;
};
struct tma340_ts_data *ts_global;

int tsp_i2c_read(u8 reg, unsigned char *rbuf, int buf_size);
int tsp_i2c_write(u8 reg, unsigned char *rbuf, int buf_size);
 
/* firmware - update */
static int firmware_ret_val = 0;
static int HW_ver = -1;
 
int firm_update( struct tma340_ts_data *ts );
extern int cypress_update( int );

#define TMA340_PHONE_FIRMWARE_VERSION	1

#define TMA340_FIRMWARE_FORCED_UPDATE	1
#define TMA340_FIRMWARE_NORMAL_UPDATE	2
#define TMA340_FIRMWARE_NEVER_UPDATE	3
unsigned char  TMA340_FIRMWARE_UPDATE_MODE = TMA340_FIRMWARE_NORMAL_UPDATE;

// from board-sec-u8500.h
#define GODIN_VER0_2	5
#define GTI9060_R0_0	6
#define GTI9060_R0_1	7

extern unsigned int system_rev;
extern unsigned char Firmware_Data_Ver;
unsigned char Firmware_Ver;
/* firmware - update */

// For forced upload mode. From gpio_keys.c
extern void gpio_keys_setstate( int keycode, bool bState );
 
/* sys fs */
extern struct class *sec_class;
struct device *touchscreen_dev;
EXPORT_SYMBOL(touchscreen_dev);

static ssize_t tma340_show_tsp_info(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t tma340_show_touchkey_menu(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t tma340_show_touchkey_back(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t tma340_show_touchkey_bright(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t tma340_set_touchkey_bright( struct device *dev, struct device_attribute *attr, const char *buf, size_t size);

static DEVICE_ATTR(tsp_info, 0444, tma340_show_tsp_info, NULL);
static DEVICE_ATTR(touchkey_menu, 0444, tma340_show_touchkey_menu, NULL);
static DEVICE_ATTR(touchkey_back, 0444, tma340_show_touchkey_back, NULL);
static DEVICE_ATTR(touchkey_brightness, 0664, tma340_show_touchkey_bright, tma340_set_touchkey_bright);


#if TMA340_TSP_FACTORY_TEST
static ssize_t tma340_show_firm_version_phone(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t tma340_show_firm_version_panel(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t tma340_show_firm_update_status(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t tma340_firm_update( struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static ssize_t tma340_firm_update_r(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t key_threshold_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t key_threshold_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size);
static ssize_t tma340_threshold_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t tma340_threshold_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size);
static DEVICE_ATTR(tsp_firm_version_phone, 0444, tma340_show_firm_version_phone, NULL);
static DEVICE_ATTR(tsp_firm_version_panel, 0444, tma340_show_firm_version_panel, NULL);
static DEVICE_ATTR(tsp_firm_update_status, 0444, tma340_show_firm_update_status, NULL);
static DEVICE_ATTR(tsp_firm_update , 0664, tma340_firm_update_r, tma340_firm_update);
static DEVICE_ATTR(key_threshold, S_IRUGO | S_IWUSR, key_threshold_show, key_threshold_store);	/* TSK Firmware update */
static DEVICE_ATTR(tsp_threshold, S_IRUGO | S_IWUSR, tma340_threshold_show, tma340_threshold_store);	/* TSP Firmware update */
#endif

static struct attribute *Tma340Touch_attributes[] = {
	&dev_attr_tsp_info.attr,
	&dev_attr_touchkey_brightness.attr,
	&dev_attr_touchkey_menu.attr,
	&dev_attr_touchkey_back.attr,
#if TMA340_TSP_FACTORY_TEST
	&dev_attr_tsp_firm_version_phone.attr,
	&dev_attr_tsp_firm_version_panel.attr,
	&dev_attr_tsp_firm_update_status.attr,
	&dev_attr_tsp_firm_update.attr,
	&dev_attr_key_threshold.attr,
	&dev_attr_tsp_threshold.attr,
#endif
	NULL,
};

static struct attribute_group Tma340Touch_attr_group = {
	.attrs = Tma340Touch_attributes,
};
/* sys fs */

#ifdef CONFIG_HAS_EARLYSUSPEND
static void tma340_ts_early_suspend(struct early_suspend *h);
static void tma340_ts_late_resume(struct early_suspend *h);
#endif
 
static int tma340_ts_suspend(struct i2c_client *client, pm_message_t mesg);
static int tma340_ts_resume(struct i2c_client *client);

static void tma340_tsp_power_onoff(struct tma340_ts_data *ts, int turnOn, int wait)
{
	if (turnOn) {
		if (ts->tsp_1v8_reg) {
			regulator_enable(ts->tsp_1v8_reg);
			regulator_enable(ts->tsp_3v_reg);
		} else if (gpio_is_valid(ts->pdata.ldo_gpio)) {
			gpio_set_value(ts->pdata.ldo_gpio,1);
		}
	} else {
		if (ts->tsp_1v8_reg) {
			regulator_disable(ts->tsp_1v8_reg);
			regulator_disable(ts->tsp_3v_reg);
		} else if (gpio_is_valid(ts->pdata.ldo_gpio)) {
			gpio_set_value(ts->pdata.ldo_gpio,0);
		}
	}
	if (wait)
		msleep(ts->pdata.on_off_delay);	/* wait for power to stablise */
}

int tma340_reset(struct tma340_ts_data *ts)
{
	int key;

	printk("[TSP] %s+\n", __func__ );

	// for TSK
	for(key = 0; key < MAX_KEYS ; key++)
		ts->touchkey_status[key] = TK_STATUS_RELEASE;

	if (ts->use_irq)
	{
		disable_irq(ts->client->irq);
	}

	tma340_tsp_power_onoff(ts, 0, 1);

	msleep( 5 );

	tma340_tsp_power_onoff(ts, 1, 1);

	msleep(200);

	if (ts->use_irq)
	{
		enable_irq(ts->client->irq);
	}

	printk("[TSP] %s-\n", __func__ );

	return 0;
}

void TSP_forced_release_forkey(void)
{
	int i;
	int temp_value=0;
	
	for ( i= 0; i<MAX_USING_FINGER_NUM; i++ )
	{
		if(ts_global->fingerInfo[i].id >=1)
		{
			ts_global->fingerInfo[i].status = -2; // force release
		}

		if(ts_global->fingerInfo[i].status != -2) continue;
		
		input_report_abs(ts_global->input_dev, ABS_MT_POSITION_X, ts_global->fingerInfo[i].x);
		input_report_abs(ts_global->input_dev, ABS_MT_POSITION_Y, ts_global->fingerInfo[i].y);
		input_report_abs(ts_global->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts_global->input_dev, ABS_MT_WIDTH_MAJOR, ts_global->fingerInfo[i].z);
		input_mt_sync(ts_global->input_dev);

		printk("[TSP] force release\n");
		temp_value++;
	}
	if(temp_value>0)
		input_sync(ts_global->input_dev);

	//check each key status
	for(i = 0; i < MAX_KEYS; i++)
	{
		if (ts_global->touchkey_status[i]) {
			input_report_key(ts_global->input_dev, touchkey_keycodes[i], 0);
			ts_global->touchkey_status[i] = 0;
		}
	}
}
EXPORT_SYMBOL(TSP_forced_release_forkey);

static void tma340_process_key_event(struct tma340_ts_data *ts, uint8_t tsk_msg)
{
	int i;
	int keycode= 0;
	int st_old, st_new;
	const int max_keys = (system_rev >= GTI9060_R0_1) ? 4 : 2;

	if( system_rev <= GTI9060_R0_0 )
		tsk_msg = tsk_msg >> 6;
	else {
		// For Forced Upload mode
		if( (tsk_msg>>1) & 0x1) {
			// home key is pressed
			gpio_keys_setstate(KEY_HOME, true);
		}
		else
			gpio_keys_setstate(KEY_HOME, false);
	}

	//check each key status
	for(i = 0; i < max_keys; i++)
	{
		st_old = ts->touchkey_status[i];
		st_new = (tsk_msg>>(i)) & 0x1;
		keycode = touchkey_keycodes[i];

		ts->touchkey_status[i] = st_new;	// save status

		if(st_new > st_old)
		{
			// press event
			printk("[TSP] touchkey: %4d, press\n", keycode);
			input_report_key(ts->input_dev, keycode, 1);
		}
		else if(st_old > st_new)
		{
			// release event
			printk("[TSP] touchkey: %4d, release\n", keycode);
			input_report_key(ts->input_dev, keycode, 0);
		}
	}
}


#define ABS(a,b) (a>b?(a-b):(b-a))
static void tma340_ts_work_func(struct work_struct *work)
{
	int ret=0;
	uint8_t buf[32];// 00h ~ 1Fh
	uint8_t i2c_addr = 0x00;
	int i = 0;
	uint8_t finger = 0;
	uint16_t x,y,z;
	uint8_t id;

	struct tma340_ts_data *ts = container_of(work, struct tma340_ts_data, work);

	memset(buf, 0, sizeof(buf));
	ret = tsp_i2c_read( i2c_addr, buf, sizeof(buf));

	if (ret <= 0) {
		printk("[TSP] i2c failed : ret=%d, ln=%d\n",ret, __LINE__);
		goto work_func_out;
	}

	finger = buf[2] & 0x07;	

	x = (buf[3] << 8) |buf[4];
	y = (buf[5] << 8) |buf[6];
	z = buf[7]/2;
	id = (buf[8] >>4)& 0x0f;

	if (ts->pdata.flipxy) {
		ts->fingerInfo[0].x = y;
		ts->fingerInfo[0].y = x;
	} else {
		ts->fingerInfo[0].x = x;
		ts->fingerInfo[0].y = y;
	}
	ts->fingerInfo[0].z = z;
	ts->fingerInfo[0].id = id;

	if (ts->pdata.flipxy) {
		ts->fingerInfo[1].x = (buf[11] << 8) |buf[12];
		ts->fingerInfo[1].y = (buf[9] << 8) |buf[10];
	} else {
		ts->fingerInfo[1].x = (buf[9] << 8) |buf[10];
		ts->fingerInfo[1].y = (buf[11] << 8) |buf[12];
	}
	ts->fingerInfo[1].z = buf[13]/2;
	ts->fingerInfo[1].id = buf[8] & 0x0f;

	//	print message
//	for ( i= 0; i<MAX_USING_FINGER_NUM; i++ )
//		printk("[TSP] finger[%d].x = %d, finger[%d].y = %d, finger[%d].z = %x, finger[%d].id = %x\n", i, ts->fingerInfo[i].x, i, ts->fingerInfo[i].y, i, ts->fingerInfo[i].z, i, ts->fingerInfo[i].id);
/*
	if(ts->fingerInfo[0].status != 1 && ts->fingerInfo[1].status != 1)
		printk("[TSP] < press > [%d].x = %d, [%d].y = %d, [%d].z = %x, [%d].id = %x\n", i, ts->fingerInfo[i].x, i, ts->fingerInfo[i].y, i, ts->fingerInfo[i].z, i, ts->fingerInfo[i].id);
	if(ts->fingerInfo[0].id != 1 && ts->fingerInfo[0].id != 2)
		printk("[TSP] <release> [%d].x = %d, [%d].y = %d, [%d].z = %x, [%d].id = %x\n", i, ts->fingerInfo[i].x, i, ts->fingerInfo[i].y, i, ts->fingerInfo[i].z, i, ts->fingerInfo[i].id);
*/


	/* check key event*/
	if(ts->fingerInfo[0].status != 1 && ts->fingerInfo[1].status != 1)
	{
		if( system_rev <= GTI9060_R0_0)
			tma340_process_key_event(ts, buf[2]);
		else	// GTI9060_R0_1
			tma340_process_key_event(ts, buf[0x1b]);		// 0x1b == 27
	}

	/* check touch event */
	for ( i= 0; i<MAX_USING_FINGER_NUM; i++ )
	{
		if(ts->fingerInfo[i].id >=1) // press interrupt
		{
			if(i==0 && ts->fingerInfo[1].status != 1)
			{
				if((ts->fingerInfo[2].id != ts->fingerInfo[0].id)&&(ts->fingerInfo[2].id != 0))// no release with finger id change
				{
		//			if(ts->fingerInfo[1].id ==0)
					{
						input_report_abs(ts->input_dev, ABS_MT_POSITION_X, ts->fingerInfo[2].x);	
						input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, ts->fingerInfo[2].y);
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
						input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, ts->fingerInfo[2].z);
						input_mt_sync(ts->input_dev);
						input_sync(ts->input_dev);

						//printk("[TSP] [%d] 0 (%d,	%d,	%x)\n", i, ts->fingerInfo[2].x, ts->fingerInfo[2].y, ts->fingerInfo[2].z);
						ts->fingerInfo[1].status = -1;
					}
				}
				else if(ts->fingerInfo[2].id != 0) // check x or y jump with same finger id
				{
					
					if(ABS(ts->fingerInfo[2].x,ts->fingerInfo[0].x)>180)
					{
						input_report_abs(ts->input_dev, ABS_MT_POSITION_X, ts->fingerInfo[2].x);	
						input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, ts->fingerInfo[2].y);
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
						input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, ts->fingerInfo[2].z);
						input_mt_sync(ts->input_dev);
						input_sync(ts->input_dev);

						//printk("[TSP] [%d] 0 (%d,	%d,	%x)\n", i, ts->fingerInfo[2].x, ts->fingerInfo[2].y, ts->fingerInfo[2].z);
						ts->fingerInfo[1].status = -1;	
					}
					else if(ABS(ts->fingerInfo[2].y,ts->fingerInfo[0].y)>180)
					{
						input_report_abs(ts->input_dev, ABS_MT_POSITION_X, ts->fingerInfo[2].x);	
						input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, ts->fingerInfo[2].y);
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
						input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, ts->fingerInfo[2].z);
						input_mt_sync(ts->input_dev);
						input_sync(ts->input_dev);

						//printk("[TSP] [%d] 0 (%d,	%d,	%x)\n", i, ts->fingerInfo[2].x, ts->fingerInfo[2].y, ts->fingerInfo[2].z);
						ts->fingerInfo[1].status = -1;	
					}
					else // no jump
					{
						if(ts->fingerInfo[i].status != -2) // force release
							ts->fingerInfo[i].status = 1;
						else
							ts->fingerInfo[i].status = -2;
					}
				}
				else // single touch with normal condition
				{
					if(ts->fingerInfo[i].status != -2) // force release
						ts->fingerInfo[i].status = 1;
					else
						ts->fingerInfo[i].status = -2;
				}
			}
			else
			{
				if(ts->fingerInfo[i].status != -2) // force release
					ts->fingerInfo[i].status = 1;
				else
					ts->fingerInfo[i].status = -2;
			}
		}
		else if(ts->fingerInfo[i].id ==0) // release interrupt (only first finger)
		{
			if(ts->fingerInfo[i].status == 1) // prev status is press
				ts->fingerInfo[i].status = 0;
			else if(ts->fingerInfo[i].status == 0 || ts->fingerInfo[i].status == -2) // release already or force release
				ts->fingerInfo[i].status = -1;				
		}

		if(ts->fingerInfo[i].status < 0) continue;
		
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, ts->fingerInfo[i].x);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, ts->fingerInfo[i].y);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, ts->fingerInfo[i].status);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, ts->fingerInfo[i].z);
		input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, ts->fingerInfo[i].id);		
		input_mt_sync(ts->input_dev);

		//printk("[TSP] [%d] %d (%d,	%d,	%x)		%x\n", i, ts->fingerInfo[i].id, ts->fingerInfo[i].x, ts->fingerInfo[i].y, ts->fingerInfo[i].z, ts->fingerInfo[i].status);		
	}

//	printk("\n");
//	printk("%d	%d\n", ts->fingerInfo[0].status, ts->fingerInfo[1].status);
	input_sync(ts->input_dev);
	
	ts->fingerInfo[2].x = ts->fingerInfo[0].x;
	ts->fingerInfo[2].y = ts->fingerInfo[0].y;
	ts->fingerInfo[2].z = ts->fingerInfo[0].z;
	ts->fingerInfo[2].id = ts->fingerInfo[0].id;	
	
work_func_out:
	if (ts->use_irq)
	{
		enable_irq(ts->client->irq);
	}
}

int tsp_i2c_read(u8 reg, unsigned char *rbuf, int buf_size)
{
	int ret=-1;
	struct i2c_msg rmsg[2];
	uint8_t start_reg;

	rmsg[0].addr = ts_global->client->addr;
	rmsg[0].flags = 0;//I2C_M_WR;
	rmsg[0].len = 1;
	rmsg[0].buf = &start_reg;
	start_reg = reg;

	rmsg[1].addr = ts_global->client->addr;
	rmsg[1].flags = I2C_M_RD;
	rmsg[1].len = buf_size;
	rmsg[1].buf = rbuf;
	ret = i2c_transfer(ts_global->client->adapter, rmsg, 2 );

	if( ret < 0 ) {
		printk("[TSP] Error code : %d\n", __LINE__ );
//		printk("[TSP] reset ret=%d\n", tsp_reset( ) );
	}

	return ret;
}
int tsp_i2c_write(u8 reg, unsigned char *rbuf, int buf_size)
{
	int ret=-1;
	struct i2c_msg rmsg;
	unsigned char data[2];

	rmsg.addr = ts_global->client->addr;
	rmsg.flags = 0;
	rmsg.len = 2;
	rmsg.buf = data;
	data[0] = reg;
	data[1] = rbuf[0];
	ret = i2c_transfer(ts_global->client->adapter, &rmsg, 1);

	return ret;
}

static irqreturn_t tma340_ts_irq_handler(int irq, void *dev_id)
{
	struct tma340_ts_data *ts = dev_id;

	disable_irq_nosync(ts->client->irq);
	queue_work(ts->tma340_wq, &ts->work);
	return IRQ_HANDLED;
}

static void tma340_check_ic_work_func(struct work_struct *work)
{
	int ret=0;
	struct tma340_ts_data *ts = container_of(work, struct tma340_ts_data, work_timer);
	uint8_t i2c_addr = 0x1F;
	uint8_t wdog_val[1];

	wdog_val[0] = 1;

	if(ts->check_ic_counter == 0)
	{
		ret = tsp_i2c_read( i2c_addr, wdog_val, sizeof(wdog_val));
		if (ret <= 0) {
			tma340_reset(ts);
			printk("[TSP] i2c failed : ret=%d, ln=%d\n",ret, __LINE__);
		}
		else if(wdog_val[0] == (uint8_t)ts->prev_wdog_val || wdog_val[0] == 0x0 ||wdog_val[0] == 0xff)
		{
			printk("[TSP] %s tsp_reset counter = %x, prev = %x\n", __func__,
					wdog_val[0], (uint8_t)ts->prev_wdog_val);
			tma340_reset(ts);
			ts->prev_wdog_val = -1;
		}
		else
		{
//			printk("[TSP] %s counter = %x, prev = %x\n", __func__, wdog_val[0], (uint8_t)prev_wdog_val);
			ts->prev_wdog_val = wdog_val[0];
		}
		
		ts->check_ic_counter = 3;	
	}
	else
	{
		ts->check_ic_counter--;
	}
}

static enum hrtimer_restart tma340_watchdog_timer_func(struct hrtimer *timer)
{
	struct tma340_ts_data *ts;
	ts = container_of(timer, struct tma340_ts_data, timer);

	if (!wake_lock_active(&ts->update_wakelock)) {
		queue_work(ts->tma340_wq, &ts->work_timer);
		hrtimer_start(&ts->timer, ktime_set(0, 500000000), HRTIMER_MODE_REL);
	}

	return HRTIMER_NORESTART;
}


/***********************************************************************************/
/*             Firmware Update Functions													     		     */
/***********************************************************************************/
unsigned char fSDATACheck(void)
{
//	gpio_direction_input(ts_global->pdata.sda_gpio);
    if ( gpio_get_value(ts_global->pdata.sda_gpio) ) {
        return(1);
    } else {
        return(0);
    }
}

void SCLKHigh(void)
{
	gpio_direction_output(ts_global->pdata.scl_gpio, 1);
	udelay(3);
}

void SCLKLow(void)
{
	gpio_direction_output(ts_global->pdata.scl_gpio, 0);
	udelay(3);
}

void SetSCLKHiZ(void)
{
	gpio_direction_input(ts_global->pdata.scl_gpio);
}

void SetSCLKStrong(void)
{
}

void SetSDATAHigh(void)
{
	gpio_direction_output(ts_global->pdata.sda_gpio, 1);
}

void SetSDATALow(void)
{
	gpio_direction_output(ts_global->pdata.sda_gpio, 0);
}

void SetSDATAHiZ(void)
{
	gpio_direction_input(ts_global->pdata.sda_gpio);
}
void SetSDATAStrong(void)
{
	SCLKHigh();
	SetSDATAHigh();
}

void SetTargetVDDStrong(void)
{
// return nothing
}

void ApplyTargetVDD(void)
{
	//printk(KERN_INFO "[TSP] Update VDD on\n");
	tma340_tsp_power_onoff(ts_global, 1, 0);
}

void RemoveTargetVDD(void)
{
	tma340_tsp_power_onoff(ts_global, 0, 0);
	//printk(KERN_INFO "[TSP] Update VDD off\n");
}

int firm_update( struct tma340_ts_data *ts )
{
	int ret;
	printk(KERN_INFO "[TSP] %s, %d\n", __func__, __LINE__);

	if (ts->pdata.disable_i2c)
		ts->pdata.disable_i2c(1);

	gpio_request(ts->pdata.sda_gpio,"TSP SDA");
	gpio_request(ts->pdata.scl_gpio,"TSP SCL");

	printk("[TSP] disable_irq : %d\n", __LINE__ );
	if (ts->use_irq) {
		disable_irq(ts->client->irq);
		free_irq(ts->client->irq, ts);
	}
	if(ts->pdata.watchdog) {
		hrtimer_cancel(&ts->timer);
		ret = cancel_work_sync(&ts->work_timer);
	}
	ret = cancel_work_sync(&ts->work);

	local_irq_disable();

	// Change GPIO PIN's Configuration : GPIO
	nmk_config_pin(PIN_CFG(229,GPIO), false);

	firmware_ret_val = 1;
	ret = cypress_update( HW_ver );

	// Change GPIO PIN Configuration : I2C
	nmk_config_pin(PIN_CFG(229,ALT_C), false);

	printk("[TSP] enable_irq : %d\n", __LINE__ );
	local_irq_enable();

	if( ret ) {
		firmware_ret_val = 2;
		printk( "[TSP] %s success, %d\n", __func__, __LINE__);
	} else {
		firmware_ret_val = -1;
		printk( "[TSP] %s fail, %d\n", __func__, __LINE__);
	}

	tma340_tsp_power_onoff(ts, 1, 1);

	if (ts->use_irq) {
		ret = request_irq(ts->client->irq, tma340_ts_irq_handler,
						IRQF_TRIGGER_FALLING , ts->client->name, ts);
	}

	if (ts->pdata.disable_i2c)
		ts->pdata.disable_i2c(0);

	gpio_free(ts->pdata.sda_gpio);
	gpio_free(ts->pdata.scl_gpio);

	ts->prev_wdog_val = -1;

	if(ts->pdata.watchdog)
		hrtimer_start(&ts->timer, ktime_set(2, 0), HRTIMER_MODE_REL);

	wake_unlock(&ts->update_wakelock);
	return 0;
} 

static void tma340_update_work_func(struct work_struct *work)
{
	struct tma340_ts_data *ts = container_of(work, struct tma340_ts_data, work_update);

	wake_lock(&ts->update_wakelock);
	firm_update(ts);
}
/* firmware - update */

/***********************************************************************************/
/*             SYS FS Interface Functions													     */
/***********************************************************************************/

#ifdef CONFIG_LEDS_CLASS
static void tma340_led_work(struct work_struct *work)
{
	struct tma340_ts_data *ts = container_of(work, struct tma340_ts_data, led_work);

	if (ts->brightness == LED_OFF)
		gpio_set_value(ts->pdata.key_led_gpio, 0);
	else
		gpio_set_value(ts->pdata.key_led_gpio, 1);

}

static void tma340_keybacklight_set (struct led_classdev *led_cdev, enum led_brightness brightness)
{
	struct tma340_ts_data *ts = container_of(led_cdev, struct tma340_ts_data, leds);

	ts->brightness = brightness;

	queue_work(ts->tma340_wq, &ts->led_work);	
}
#endif

static ssize_t tma340_show_tsp_info(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tma340_ts_data *ts = dev_get_drvdata(dev);
	ssize_t size;

	/* output for each finger is [id], x,y,z,[status] */
	/* status 0 or < 0 is released, 1 is touching */
	size = sprintf(buf, "First [%d],%d,%d,%d,[%d] -- Second  [%d],%d,%d,%d,[%d]\n",
			ts->fingerInfo[0].id, ts->fingerInfo[0].x, ts->fingerInfo[0].y, ts->fingerInfo[0].z, ts->fingerInfo[0].status,
			ts->fingerInfo[1].id, ts->fingerInfo[1].x, ts->fingerInfo[1].y, ts->fingerInfo[1].z, ts->fingerInfo[1].status);

	return size;
}

static ssize_t tma340_show_touchkey_menu(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tma340_ts_data *ts = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", ts->touchkey_status[MENU_KEY_IDX]);
}
static ssize_t tma340_show_touchkey_back(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tma340_ts_data *ts = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", ts->touchkey_status[BACK_KEY_IDX]);
}
static ssize_t tma340_show_touchkey_bright(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tma340_ts_data *ts = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", ts->brightness);
}
static ssize_t tma340_set_touchkey_bright(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	char *after;
	struct tma340_ts_data *ts = dev_get_drvdata(dev);
	unsigned long value = simple_strtoul(buf, &after, 10);	

	ts->brightness = value;
	queue_work(ts->tma340_wq, &ts->led_work);	

	return size;
}

#if TMA340_TSP_FACTORY_TEST
static ssize_t tma340_show_firm_version_phone(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%x\n", Firmware_Data_Ver);
}

static ssize_t tma340_show_firm_version_panel(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint8_t i2c_addr = 0x1C;
	uint8_t buf_tmp[2] = {0};

	tsp_i2c_read( i2c_addr, buf_tmp, sizeof(buf_tmp));

	/* 	The TSP Driver report like X-Y as decimal.
		The X is the HW version that TSP has
		The Y is the Firmware version what TSP has. */

	return sprintf(buf, "%d-%d\n", buf_tmp[0], buf_tmp[1]);
}

static ssize_t tma340_show_firm_update_status(struct device *dev, struct device_attribute *attr, char *buf)
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

static ssize_t tma340_firm_update( struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct tma340_ts_data *ts = dev_get_drvdata(dev);

	printk(KERN_INFO "[TSP] %s, %d\n", __func__, __LINE__);

	if ( buf[0] == 'S' )
	{
		printk("[TSP] Firmware update start!!\n" );

		firmware_ret_val = 1;
		queue_work(ts->tma340_wq, &ts->work_update);
		return size;
	}

	return size;
}
static ssize_t tma340_firm_update_r(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tma340_ts_data *ts = dev_get_drvdata(dev);

	printk(KERN_INFO "[TSP] %s, %d\n", __func__, __LINE__);

	printk("[TSP] Firmware update start!!\n" );

	firmware_ret_val = 1;
	queue_work(ts->tma340_wq, &ts->work_update);

	return sprintf(buf,"STARTED\n");
}

/* touch key threshold */
static ssize_t key_threshold_show(struct device *dev, struct device_attribute *attr, char *buf)
{	
	printk(KERN_INFO "[TSP] threshold not support\n");
	return sprintf(buf, "%d\n", 0);
}

static ssize_t key_threshold_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	printk(KERN_INFO "[TSP] threshold not support\n");

	return size;
}

/* touch threshold */
static ssize_t tma340_threshold_show(struct device *dev, struct device_attribute *attr, char *buf)
{	
	printk(KERN_INFO "[TSP] threshold not support\n");
	return sprintf(buf, "%d\n", 0);
}

static ssize_t tma340_threshold_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	printk(KERN_INFO "[TSP] threshold not support\n");

	return size;
}
#endif

/***********************************************************************************/
/*             Basic Driver Functions													            */
/***********************************************************************************/
static int tma340_ts_probe(
		struct i2c_client *client, const struct i2c_device_id *id)
{
	struct tma340_ts_data *ts;
	struct tma340_tsp_platform_data *pdata;
	int ret = 0, key = 0;
	uint8_t i2c_addr = 0x1B;
	uint8_t buf[3]={0,};

	printk("[TSP] %s, %d\n", __func__, __LINE__ );

	if( system_rev == GTI9060_R0_1 )
	{
		printk("[TSP] rev 0.4 uses 4 Touch Keys\n");
		touchkey_keycodes[0] = KEY_MENU;
		touchkey_keycodes[1] = KEY_HOME;
		touchkey_keycodes[2] = KEY_BACK;
		touchkey_keycodes[3] = KEY_SEARCH;

		Firmware_Data_Ver = 0x00;
	}
	else
		Firmware_Data_Ver = 0x01;

	if (!client->dev.platform_data) {
		dev_err(&client->dev, "platform data is required\n");
		return -EINVAL;
	}
	pdata = client->dev.platform_data;

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	ts->pdata = *pdata;

	if (ts->pdata.reg_1v8) {
		ts->tsp_1v8_reg = regulator_get(&client->dev, pdata->reg_1v8);
		ts->tsp_3v_reg = regulator_get(&client->dev, pdata->reg_3v);
	} else if (gpio_is_valid(ts->pdata.ldo_gpio)) {
		gpio_request(ts->pdata.ldo_gpio, "TSP_LDO_GPIO");
	}
	tma340_tsp_power_onoff(ts, 1, 1);

	ts->tma340_wq = create_singlethread_workqueue("tma340_wq");
	INIT_WORK(&ts->work, tma340_ts_work_func);
	INIT_WORK(&ts->work_timer, tma340_check_ic_work_func );
	wake_lock_init(&ts->update_wakelock, WAKE_LOCK_IDLE, "tsp_update");
	INIT_WORK(&ts->work_update, tma340_update_work_func );

	ts->client = client;
	i2c_set_clientdata(client, ts);

	ts_global = ts;

	if (ts->pdata.watchdog) {
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = tma340_watchdog_timer_func;
	}

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		printk(KERN_ERR "tma340_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "sec_touchscreen";
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(EV_ABS, ts->input_dev->evbit);

	printk(KERN_INFO "tma340_ts_probe: max_x: %d, max_y: %d\n", ts->pdata.max_x, ts->pdata.max_y);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->pdata.max_x, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->pdata.max_y, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);

	for(key = 0; key < MAX_KEYS ; key++)
		input_set_capability(ts->input_dev, EV_KEY, touchkey_keycodes[key]);

	// for TSK
	for(key = 0; key < MAX_KEYS ; key++)
		ts->touchkey_status[key] = TK_STATUS_RELEASE;

	ts->fingerInfo[0].status = -1;
	ts->fingerInfo[1].status = -1;
	ts->fingerInfo[2].id = 0;

	ret = input_register_device(ts->input_dev);
	if (ret) {
		printk(KERN_ERR "tma340_ts_probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	printk("[TSP] %s, irq=%d\n", __func__, client->irq );
	if (client->irq) {
		gpio_request(irq_to_gpio(client->irq), "tsp_irq");
		ret = request_irq(client->irq, tma340_ts_irq_handler, IRQF_TRIGGER_FALLING , client->name, ts);
		if (ret == 0) 
			ts->use_irq = 1;
		else
			dev_err(&client->dev, "request_irq failed\n");
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = tma340_ts_early_suspend;
	ts->early_suspend.resume = tma340_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	printk(KERN_INFO "tma340_ts_probe: Start touchscreen %s in %s mode\n", ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");

#ifdef CONFIG_LEDS_CLASS
	if (gpio_is_valid(pdata->key_led_gpio)) {
		gpio_request(pdata->key_led_gpio, "TSP Key BL");

		INIT_WORK(&ts->led_work, tma340_led_work);

		ts->leds.name 			= TOUCHKEY_BACKLIGHT;
		ts->leds.brightness 	= LED_FULL;
		ts->leds.max_brightness = LED_FULL;
		ts->leds.brightness_set = tma340_keybacklight_set;
		
		ret = led_classdev_register(&client->dev, &ts->leds);
		if ( ret	) {
			goto err_input_register_device_failed;
		}
	}
#endif
	
	/* sys fs */
	touchscreen_dev = device_create(sec_class, NULL, 0, NULL, "sec_touchscreen");
	if (IS_ERR(touchscreen_dev))
		pr_err("Failed to create device(touchscreen)!\n");

	dev_set_drvdata(touchscreen_dev, ts);
	ret = sysfs_create_group(&touchscreen_dev->kobj, &Tma340Touch_attr_group);
	if (ret)
		goto err_remove_attr_group;
	/* sys fs */

	/* Check point - i2c check - start */
	ret = tsp_i2c_read( i2c_addr, buf, sizeof(buf));

	if (ret <= 0) {
		printk("[TSP] i2c failed : ret=%d, ln=%d\n",ret, __LINE__);
	}
	HW_ver = buf[1];
	Firmware_Ver = buf[2];
	
	printk("[TSP] %s: %d, buf[1]=%x, buf[2]=%x\n", __func__, __LINE__, buf[1], buf[2]);

	if( buf[1]==0 ){
		printk("[TSP] %s, ln:%d, Failed to register TSP!!!\n\tcheck the i2c line!!!, ret=%d\n", __func__,__LINE__, ret);
		if( TMA340_FIRMWARE_UPDATE_MODE == TMA340_FIRMWARE_NEVER_UPDATE )
			;
		else
			TMA340_FIRMWARE_UPDATE_MODE = TMA340_FIRMWARE_FORCED_UPDATE;
	} 

	/* Check point - i2c check - end */
	/* Check point - Firmware */
	if( system_rev == GTI9060_R0_0 )
		printk("[TSP] %s, ver CY=%x\n", __func__ , buf[0] );
	printk("[TSP] %s, ver HW=%x\n", __func__ , buf[1] );
	printk("[TSP] %s, ver SW=%x\n", __func__ , buf[2] );

	printk(KERN_INFO "tma340_ts_probe: Manufacturer ID: %x, HW ver=%d\n", buf[0], HW_ver);

#ifdef CONFIG_MACH_GODIN
	if( system_rev == GTI9060_R0_1)
	{
		switch( TMA340_FIRMWARE_UPDATE_MODE )
		{
		case TMA340_FIRMWARE_FORCED_UPDATE:
			printk("[TSP] Forced Firmware Update\n");
			queue_work(ts->tma340_wq, &ts->work_update);
			break;
		case TMA340_FIRMWARE_NORMAL_UPDATE:
			// check fw version
			if( buf[2] < Firmware_Data_Ver)
			{
				printk("[TSP] New firmware(ver. %x) is detected. Start Update.\n", Firmware_Data_Ver);
				queue_work(ts->tma340_wq, &ts->work_update);
			}
			break;
		case TMA340_FIRMWARE_NEVER_UPDATE:
			printk("[TSP] Never update firmware\n");
			// Nothing
			break;
		default:
			break;
		}
	}
#endif

	/* Check point - Firmware */

	if(ts->pdata.watchdog) {
		ts->prev_wdog_val = -1;
		ts->check_ic_counter = 3;
		hrtimer_start(&ts->timer, ktime_set(5, 0), HRTIMER_MODE_REL);
	}	
	
	return 0;

err_remove_attr_group:
err_failed_read_version:
	sysfs_remove_group(&touchscreen_dev->kobj, &Tma340Touch_attr_group);
err_input_register_device_failed:
	if (ts->use_irq)
		free_irq(ts->client->irq, ts);
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
	if(ts->pdata.watchdog)
		hrtimer_cancel(&ts->timer);

	destroy_workqueue(ts->tma340_wq);
	gpio_free(ts->pdata.ldo_gpio);
	kfree(ts);
err_alloc_data_failed:
	return ret;
}

static int tma340_ts_remove(struct i2c_client *client)
{
	struct tma340_ts_data *ts = i2c_get_clientdata(client);

	unregister_early_suspend(&ts->early_suspend);
	if (ts->use_irq)
		free_irq(client->irq, ts);

	if(ts->pdata.watchdog)
		hrtimer_cancel(&ts->timer);

	if (ts->tsp_1v8_reg) {
		regulator_put(ts->tsp_1v8_reg);
		regulator_put(ts->tsp_3v_reg);
	} else if (gpio_is_valid(ts->pdata.ldo_gpio)) {
		gpio_free(ts->pdata.ldo_gpio);
	}

	input_unregister_device(ts->input_dev);
	i2c_set_clientdata(client,NULL);

	sysfs_remove_group(&touchscreen_dev->kobj, &Tma340Touch_attr_group);

#ifdef CONFIG_LEDS_CLASS
	cancel_work_sync(&ts->led_work);
	led_classdev_unregister(&ts->leds);
	if (gpio_is_valid(ts->pdata.key_led_gpio)) {
		gpio_free(ts->pdata.key_led_gpio);
	}
#endif

	cancel_work_sync(&ts->work_timer);
	cancel_work_sync(&ts->work);
	destroy_workqueue(ts->tma340_wq);

	kfree(ts);
	return 0;
}

static int tma340_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct tma340_ts_data *ts = i2c_get_clientdata(client);

	printk("[TSP] %s+\n", __func__ );

	if (wake_lock_active(&ts->update_wakelock))
		return -EBUSY;

	if (ts->use_irq)
	{
		disable_irq(client->irq);
	}

	ret = cancel_work_sync(&ts->work_timer);
	ret = cancel_work_sync(&ts->work);
	if (ret && ts->use_irq) /* if work was pending disable-count is now 2 */
	{
		enable_irq(client->irq);
	}

	if(ts->pdata.watchdog)
		hrtimer_cancel(&ts->timer);

	if (gpio_is_valid(ts->pdata.key_led_gpio)) {
			gpio_set_value(ts->pdata.key_led_gpio, 0);
	}

	tma340_tsp_power_onoff(ts, 0, 1);

	TSP_forced_release_forkey();

	printk("[TSP] %s-\n", __func__ );
	return 0;
}

static int tma340_ts_resume(struct i2c_client *client)
{
	int ret, key;
	struct tma340_ts_data *ts = i2c_get_clientdata(client);
	uint8_t i2c_addr = 0x1D;
	uint8_t buf[1];

	printk("[TSP] %s+\n", __func__ );

	if (wake_lock_active(&ts->update_wakelock))
		return -EBUSY;

	tma340_tsp_power_onoff(ts, 1, 1);

	// for TSK
	for(key = 0; key < MAX_KEYS; key++)
		ts->touchkey_status[key] = TK_STATUS_RELEASE;

	ts->fingerInfo[0].status = -1;
	ts->fingerInfo[1].status = -1;
	ts->fingerInfo[2].id = 0;

	ret = tsp_i2c_read( i2c_addr, buf, sizeof(buf));
	if (ret <= 0) {
		printk("[TSP] %s : i2c_transfer failed\n", __func__);
	}
	else
	{
		printk("[TSP] %s, ver SW=%x\n", __func__, buf[0] );
		enable_irq(client->irq);
	}

	ts->prev_wdog_val = -1;

	if(ts->pdata.watchdog)
		hrtimer_start(&ts->timer, ktime_set(2, 0), HRTIMER_MODE_REL);

	if (gpio_is_valid(ts->pdata.key_led_gpio)) {
		if (ts->brightness == LED_OFF)
			gpio_set_value(ts->pdata.key_led_gpio, 0);
		else
			gpio_set_value(ts->pdata.key_led_gpio, 1);
	}

	printk("[TSP] %s-\n", __func__ );
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void tma340_ts_early_suspend(struct early_suspend *h)
{
	struct tma340_ts_data *ts;
	ts = container_of(h, struct tma340_ts_data, early_suspend);
	tma340_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void tma340_ts_late_resume(struct early_suspend *h)
{
	struct tma340_ts_data *ts;
	ts = container_of(h, struct tma340_ts_data, early_suspend);
	tma340_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id tma340_idtable[] = {
	{TMA340_I9060_TSP_NAME, 0,},
	{ }
};
MODULE_DEVICE_TABLE(i2c, tma340_idtable);

static struct i2c_driver tma340_i2c_driver = {
	.driver = {
		.name	= TMA340_I9060_TSP_NAME,
		.owner  = THIS_MODULE,
	},

	.id_table	= tma340_idtable,
	.probe		= tma340_ts_probe,
	.remove		= tma340_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= tma340_ts_suspend,
	.resume		= tma340_ts_resume,
#endif
};

static int __init tma340_init(void)
{
	int ret = 0;
	ret =  i2c_add_driver(&tma340_i2c_driver);
	return ret;
}

static void __exit tma340_exit(void)
{
	i2c_del_driver(&tma340_i2c_driver);
}

module_init(tma340_init);
module_exit(tma340_exit);

MODULE_AUTHOR("Robert Teather <robert.teather@samsung.com>");
MODULE_DESCRIPTION("GT-I9060 Touchscreen Driver");
MODULE_LICENSE("GPL");

