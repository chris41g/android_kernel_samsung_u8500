/*
 * cmc623.c
 *
 * driver supporting CMC623 ImageConverter functions for Samsung P3 device
 *
 * COPYRIGHT(C) Samsung Electronics Co., Ltd. 2006-2010 All Right Reserved.  
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/firmware.h>
#include <linux/wakelock.h>
#include <linux/blkdev.h>
#include <linux/i2c.h>
#include <mach/gpio.h>
#include <linux/backlight.h>

#include <linux/delay.h>
#include <linux/workqueue.h>

#include <linux/earlysuspend.h>

#include <video/cmc623.h>
#include "cmc623_cfg.h"

#define CABC_ONOFF_TEST 1
#define CMC_ONOFF_TEST 1
#define CMC623_TUNING 1
#define BYPASS_ONOFF_TEST 1

static int current_bypass_onoff = 0;
void bypass_onoff_ctrl(int);


#if CMC623_TUNING
#define CMC623_MAX_SETTINGS	 100
static Cmc623RegisterSet Cmc623_TuneSeq[CMC623_MAX_SETTINGS];
#define CMC623_PATH_TUNING_DATA   "/sdcard/cmc623_tunenull"
#define klogi(fmt, arg...)  printk(KERN_INFO "%s: " fmt "\n" , __func__, ## arg)
#define kloge(fmt, arg...)  printk(KERN_ERR "%s(%d): " fmt "\n" , __func__, __LINE__, ## arg)
#endif

#if 0
#define dprintk(x...) printk(x)
#else
#define dprintk(x...) (0)
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define GPIO_LEVEL_LOW      	0
#define GPIO_LEVEL_HIGH     	1

#define CMC623_PWM_MAX_INTENSITY	255
#define CMC623_PWM_DEFAULT_INTENSITY	102
#define MAX_LEVEL			1600

// brightness tuning
#define MAX_BRIGHTNESS_LEVEL 255
#define MID_BRIGHTNESS_LEVEL 102
#define LOW_BRIGHTNESS_LEVEL 30
#define DIM_BRIGHTNESS_LEVEL 20

#define MAX_BACKLIGHT_VALUE 1600 	/* 100% */
#define MID_BACKLIGHT_VALUE 720  	/* 40%  */
#define LOW_BACKLIGHT_VALUE 160 	/* 10%  */
#define DIM_BACKLIGHT_VALUE 80 		/* 5%   */


#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend	cmc623_early_suspend;
#endif

#define MAX_LEVEL 	1600

/* Each client has this additional data */
struct cmc623_data {
	struct i2c_client 		*client;
	struct backlight_device		*bd; 
	struct Cmc623PlatformData 	pdata;
};

static struct cmc623_data * p_cmc623_data = NULL;
struct workqueue_struct *ove_wq=NULL;
struct work_struct work_ove;

static struct i2c_client *g_client;
#define I2C_M_WR 0 /* for i2c */
#define I2c_M_RD 1 /* for i2c */

static int current_gamma_level = MAX_LEVEL;

static int lcdonoff = 0;

struct cmc623_state_type{
	unsigned int cabc_enabled;
	unsigned int brightness;
	unsigned int suspended;
	int white;
	int black;
	int saturation;
};

static struct cmc623_state_type cmc623_state = { 
	.cabc_enabled = TRUE,
	.brightness = 32,
	.suspended = FALSE,
	.white = 0,
	.black = 0,
	.saturation = 0,
};

typedef struct {
	unsigned short addr;
	unsigned short data;
} mDNIe_data_type;

typedef enum
{
	mDNIe_UI_MODE,
	mDNIe_VIDEO_MODE,
	mDNIe_VIDEO_WARM_MODE,
	mDNIe_VIDEO_COLD_MODE,
	mDNIe_CAMERA_MODE,
	mDNIe_NAVI,
	mDNIe_DMB_MODE,
	mDNIe_VT_MODE,
}Lcd_mDNIe_UI;

typedef enum
{
	mode_type_CABC_none,
	mode_type_CABC_on,
	mode_type_CABC_off,
}mDNIe_mode_CABC_type;


static Lcd_CMC623_UI_mode current_cmc623_UI = CMC623_UI_MODE; // mDNIe Set Status Checking Value.
static int current_cmc623_OutDoor_OnOff = FALSE;
static int current_cmc623_CABC_OnOff = FALSE;

static int setting_first = FALSE;
static int cmc623_bypass_mode = FALSE;
static int current_autobrightness_enable = FALSE;
static int cmc623_current_region_enable = FALSE;

mDNIe_mode_CABC_type cmc623_cabc_mode[]=
{
	mode_type_CABC_none,	// UI
	mode_type_CABC_on,	// Video
	mode_type_CABC_on,	// Video warm
	mode_type_CABC_on,	// Video cold
	mode_type_CABC_off, 	// Camera
	mode_type_CABC_none,	// Navi
};

#define NUM_ITEM_POWER_LUT	9
#define NUM_POWER_LUT	2

static int current_power_lut_num = 0;

unsigned char cmc623_Power_LUT[NUM_POWER_LUT][NUM_ITEM_POWER_LUT]={
	{ 0x48, 0x4d, 0x44, 0x52, 0x48, 0x45, 0x40, 0x3d, 0x45 },	//cabcon3
	{ 0x3e, 0x43, 0x3a, 0x48, 0x3e, 0x3b, 0x36, 0x33, 0x3b },
};

static bool cmc623_I2cWrite16(unsigned char Addr, unsigned long Data);
static void cmc623_cabc_pwm_brightness_reg(int value);
static void cmc623_manual_pwm_brightness_reg(int value);
static void cmc623_manual_pwm_brightness_reg_nosync(int value);

unsigned long last_cmc623_Bank = 0xffff;
unsigned long last_cmc623_Algorithm = 0xffff;

static struct platform_device *bl_pdev;
static int current_backlight_level = MID_BACKLIGHT_VALUE;
static int current_intensity;
static DEFINE_MUTEX(cmc623_pwm_mutex);
void cmc623_pwm_apply(int value);

/*
* Functions for PWM brightness control
*/
static void cmc623_pwm_apply_brightness(int level)
{
	cmc623_pwm_apply(level);
	current_backlight_level = level;

//	dev_dbg(&pdev->dev, "%s : apply_level=%d\n", __func__, level);
//	printk("%s : apply_level=%d\n", __func__, level);
}


static void cmc623_pwm_backlight_ctl(int intensity)
{
	int tune_level;

	// brightness tuning
	if (intensity >= MID_BRIGHTNESS_LEVEL)
		tune_level = (intensity - MID_BRIGHTNESS_LEVEL)
				* (MAX_BACKLIGHT_VALUE-MID_BACKLIGHT_VALUE)
				/ (MAX_BRIGHTNESS_LEVEL-MID_BRIGHTNESS_LEVEL)
				+ MID_BACKLIGHT_VALUE;
	else if (intensity >= LOW_BRIGHTNESS_LEVEL)
		tune_level = (intensity - LOW_BRIGHTNESS_LEVEL)
				* (MID_BACKLIGHT_VALUE-LOW_BACKLIGHT_VALUE)
				/ (MID_BRIGHTNESS_LEVEL-LOW_BRIGHTNESS_LEVEL)
				+ LOW_BACKLIGHT_VALUE;
	else if (intensity >= DIM_BRIGHTNESS_LEVEL)
		tune_level = (intensity - DIM_BRIGHTNESS_LEVEL)
				* (LOW_BACKLIGHT_VALUE-DIM_BACKLIGHT_VALUE)
				/ (LOW_BRIGHTNESS_LEVEL-DIM_BRIGHTNESS_LEVEL)
				+ DIM_BACKLIGHT_VALUE;
	else if (intensity > 0)
		tune_level = DIM_BACKLIGHT_VALUE;
	else
		tune_level = intensity;

	printk(KERN_INFO "--- [cmc backlight control]%d(%d)---\n", intensity, tune_level);

	cmc623_pwm_apply_brightness(tune_level);
}


static void cmc623_pwm_send_intensity(struct backlight_device *bd)
{
	int intensity = bd->props.brightness;

	mutex_lock(&cmc623_pwm_mutex);
	cmc623_pwm_backlight_ctl(intensity);
	mutex_unlock(&cmc623_pwm_mutex);

	current_intensity = intensity;
}

static int cmc623_pwm_set_intensity(struct backlight_device *bd)
{
	printk(KERN_INFO "BD->PROPS.BRIGHTNESS = %d\n", bd->props.brightness);

	cmc623_pwm_send_intensity(bd);

	return 0;
}

static int cmc623_pwm_get_intensity(struct backlight_device *bd)
{
	return current_intensity;
}


static struct backlight_ops cmc623_pwm_ops = {
	.get_brightness = cmc623_pwm_get_intensity,
	.update_status  = cmc623_pwm_set_intensity,
};

/* This structure defines all the properties of a backlight */
struct backlight_properties cmc623_backlight_props = {
	.brightness = CMC623_PWM_DEFAULT_INTENSITY,
	.max_brightness = CMC623_PWM_MAX_INTENSITY,
};


//for measuring luminance
void cmc623_pwm_set_brightness(int brightness)
{
	printk(KERN_INFO "%s: value=%d\n", __func__, brightness);

	mutex_lock(&cmc623_pwm_mutex);
	cmc623_pwm_apply_brightness(brightness);
	mutex_unlock(&cmc623_pwm_mutex);
}
EXPORT_SYMBOL(cmc623_pwm_set_brightness);


/**
 * functions for I2C transactions
 */
bool cmc623_I2cWrite16(unsigned char Addr, unsigned long Data)
{
	int err = -1000;
	struct i2c_msg msg[1];
	unsigned char data[3];

	dprintk("========cmc623_I2cWrite16=======(a:%x,d:%x)\n", Addr, Data);
	
	if(!p_cmc623_data) {
		printk(KERN_ERR "p_cmc623_data is NULL\n");
		return -ENODEV;
	}
	g_client = p_cmc623_data->client;		

	if( (g_client == NULL)) {
		printk(KERN_INFO "cmc623_I2cWrite16 g_client is NULL\n");
		return -ENODEV;
	}

	if (!g_client->adapter) {
		printk(KERN_INFO "cmc623_I2cWrite16 g_client->adapter is NULL\n");
		return -ENODEV;
	}

	if(TRUE == cmc623_state.suspended) {
		printk(KERN_INFO "cmc623 don't need writing while LCD off(a:%x,d:%lx)\n", Addr, Data);
		return 0;
	}


/*	
	if(Addr == 0x0000) {
		if(Data == last_cmc623_Bank) {
			printk(KERN_ERR "last_cmc623_Bank is returned\n");
			return 0;
		}
		last_cmc623_Bank = Data;
	} else if(Addr == 0x0001) {
		last_cmc623_Algorithm = Data;
	}
*/
	data[0] = Addr;
	data[1] = ((Data >>8)&0xFF);
	data[2] = (Data)&0xFF;
	msg->addr = g_client->addr;
	msg->flags = I2C_M_WR;
	msg->len = 3;
	msg->buf = data;
	
	err = i2c_transfer(g_client->adapter, msg, 1);

	if (err >= 0) {
		dprintk("%s %d i2c transfer OK\n", __func__, __LINE__);/* add by inter.park */
		return 1;
	}

	printk(KERN_INFO "%s i2c transfer error:%d(a:%d)\n", __func__, err, Addr);/* add by inter.park */
	return err;    
}


char cmc623_I2cRead(u8 reg, u8 *val, unsigned int len )
{
	int 	 err;
	struct	 i2c_msg msg[1];
	
	unsigned char data[1];
	if( (g_client == NULL) || (!g_client->adapter) ) {
		return -ENODEV;
	}
	
	msg->addr	= g_client->addr;
	msg->flags	= I2C_M_WR;
	msg->len	= 1;
	msg->buf	= data;
	*data		= reg;

	err = i2c_transfer(g_client->adapter, msg, 1);

	if (err >= 0) {
		msg->flags = I2C_M_RD;
		msg->len   = len;
		msg->buf   = val;
		err = i2c_transfer(g_client->adapter, msg, 1);
	}

	if (err >= 0) {
		return 0;
	}
	printk(KERN_INFO "%s %d i2c transfer error\n", __func__, __LINE__);/* add by inter.park */

	return err;

}


int cmc623_I2cRead16(u8 reg, u16 *val)
{
	int 	 err;
	struct	 i2c_msg msg[2];
	u8 regaddr = reg;
	u8 data[2];

	if(!p_cmc623_data) {
		printk(KERN_ERR "%s p_cmc623_data is NULL\n", __func__);
		return -ENODEV;
	}
	g_client = p_cmc623_data->client;		

	if( (g_client == NULL)) {
		printk(KERN_INFO "%s g_client is NULL\n", __func__);
		return -ENODEV;
	}

	if (!g_client->adapter) {
		printk(KERN_INFO "%s g_client->adapter is NULL\n", __func__);
		return -ENODEV;
	}
	
	if(regaddr == 0x0001) {
		*val = last_cmc623_Algorithm;
		return 0;
	}
	
	msg[0].addr   = g_client->addr;
	msg[0].flags  = I2C_M_WR;
	msg[0].len	  = 1;
	msg[0].buf	  = &regaddr;
	msg[1].addr   = g_client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len	 = 2;
	msg[1].buf	 = &data[0];
	err = i2c_transfer(g_client->adapter, &msg[0], 2);

	if (err >= 0) {
		*val = (data[0]<<8) | data[1];
		return 0;
	}
	printk(KERN_ERR "%s %d i2c transfer error: %d\n", __func__, __LINE__, err);/* add by inter.park */

	return err;
}



#define CMC623_INITSEQ cmc623_init1
/*#define CMC623_INITSEQ cmc623_init2*/

static bool CMC623_SetUp(void)
{
    int i=0;
    int num = ARRAY_SIZE(CMC623_INITSEQ);
	/*printk(KERN_ERR "%s num is %d\n", __func__, num);*/	
	
    for (i=0; i<num; i++) {
        if (!cmc623_I2cWrite16(CMC623_INITSEQ[i].RegAddr, CMC623_INITSEQ[i].Data)) {
		printk(KERN_INFO "why return false??!!!\n");
            	return FALSE;
        }
        if (CMC623_INITSEQ[i].RegAddr == CMC623_REG_SWRESET && 
            CMC623_INITSEQ[i].Data == 0xffff)
            msleep(3);  // 3ms
    }
    return TRUE;
}

void cmc623_reg_unmask(void)
{
	if(!p_cmc623_data) {
		printk(KERN_ERR "%s cmc623 is not initialized\n", __func__);
		return;
	}

	cmc623_I2cWrite16(0x28, 0x0000);
}


unsigned int ove_target_value=0;
void ove_workqueue_func(void* data)
{
	int i = 0;
	for(i=0; i<=8; ++i)
	{
		if(cmc623_state.suspended == TRUE)
			return;

		cmc623_I2cWrite16(0x0054, (((ove_target_value >> 8) * i / 8) << 8) | ((ove_target_value & 0x00ff) * i / 8));
		cmc623_reg_unmask();
		msleep(15);
	}
}


#define CMC623_TUNESEQ cmc623_cabcon3
#define CMC623_TUNESEQ2 cmc623_cabcoff3
/*#define CMC623_TUNESEQ cmc623_bypass*/

void cabc_onoff_ctrl(int value)
{
	if(value == 1) {
		int i = 0;
		int num = ARRAY_SIZE(CMC623_TUNESEQ);
		
		for (i=0; i<num; i++)
		{
			cmc623_I2cWrite16(CMC623_TUNESEQ[i].RegAddr, CMC623_TUNESEQ[i].Data);
		}
		dprintk(KERN_INFO "[cabc_on] <= value : %d \n",value);
	} else if(value == 0) {
		int i = 0;
		int num = ARRAY_SIZE(CMC623_TUNESEQ2);
		
		for (i=0; i<num; i++)
		{
			cmc623_I2cWrite16(CMC623_TUNESEQ2[i].RegAddr, CMC623_TUNESEQ2[i].Data);
		}
		dprintk(KERN_INFO "[cabc_off] <= value : %d \n",value);		
	}
		
}

static void CMC623_Set_Mode(void)
{
#if 1
	int i=0;
	int num = ARRAY_SIZE(CMC623_TUNESEQ);
	/*printk(KERN_ERR "%s num is %d\n", __func__, num);	*/
	cmc623_state.cabc_enabled = TRUE;

	for (i=0; i<num; i++) {
		if (!cmc623_I2cWrite16(CMC623_TUNESEQ[i].RegAddr, CMC623_TUNESEQ[i].Data)) 
			return;
	}
	return;
#else
	cabc_onoff_ctrl(cmc623_state.cabc_enabled);
#endif
}


// value: 0 ~ 1600
static void cmc623_cabc_pwm_brightness_reg(int value)
{
	int reg;
	unsigned char * p_plut;

	if(!p_cmc623_data) {
		printk(KERN_ERR "%s cmc623 is not initialized\n", __func__);
		return;
	}

	if(value==0)
		value = 4;
	
	p_plut = cmc623_Power_LUT[current_power_lut_num];

	cmc623_I2cWrite16(0x76,(p_plut[0] * value / 100) << 8 | (p_plut[1] * value / 100));	//PowerLUT
	cmc623_I2cWrite16(0x77,(p_plut[2] * value / 100) << 8 | (p_plut[3] * value / 100));	//PowerLUT
	cmc623_I2cWrite16(0x78,(p_plut[4] * value / 100) << 8 | (p_plut[5] * value / 100));	//PowerLUT
	cmc623_I2cWrite16(0x79,(p_plut[6] * value / 100) << 8 | (p_plut[7] * value / 100));	//PowerLUT
	cmc623_I2cWrite16(0x7a,(p_plut[8] * value / 100) << 8);	//PowerLUT

	reg = 0x5000 | (value<<4);

	cmc623_I2cWrite16(0xB4, reg);			//pwn duty
}

int Islcdonoff(void)
{
	return lcdonoff;
}
EXPORT_SYMBOL(Islcdonoff);

// value: 0 ~ 100
static void cmc623_cabc_pwm_brightness(int value)
{
	unsigned char * p_plut;

	if(!p_cmc623_data) {
		printk(KERN_ERR "%s cmc623 is not initialized\n", __func__);
		return;
	}

	p_plut = cmc623_Power_LUT[current_power_lut_num];

	cmc623_I2cWrite16(0x00,0x0000);	//BANK 0

	cmc623_cabc_pwm_brightness_reg(value);

	cmc623_I2cWrite16(0x28,0x0000);
}

// value: 0 ~ 100
static void cmc623_manual_pwm_brightness(int value)
{
	int reg;

	if(!p_cmc623_data) {
		printk(KERN_ERR "%s cmc623 is not initialized\n", __func__);
		return;
	}

	reg = 0x4000 | (value<<4);

	cmc623_I2cWrite16(0x00, 0x0000);		//bank0
	cmc623_I2cWrite16(0xB4, reg);			//pwn duty
	cmc623_I2cWrite16(0x28, 0x0000);

}

// value: 0 ~ 1600
void cmc623_pwm_brightness(int value)
{
	int data;
	dprintk(KERN_INFO "%s : BL brightness level = %d\n", __func__, value);

	if(value<0)
		data = 0;
	else if(value>1600)
		data = 1600;
	else
		data = value;
#if 0		
	if(data == 1280 && current_autobrightness_enable)
	{//outdoor mode on
		current_cmc623_OutDoor_OnOff = TRUE;
		cmc623_Set_Mode(current_cmc623_UI, current_cmc623_CABC_OnOff);
	}
	else if (current_cmc623_OutDoor_OnOff == TRUE && data < 1280)
	{//outdoor mode off
		current_cmc623_OutDoor_OnOff = FALSE;
		cmc623_Set_Mode(current_cmc623_UI, current_cmc623_CABC_OnOff);
	}
#else
	/*CMC623_Set_Mode();*/
#endif
	data >>= 4;

	cmc623_state.brightness = data;

	if((cmc623_state.cabc_enabled == TRUE)&&(current_bypass_onoff==FALSE)) {
		cmc623_cabc_pwm_brightness(data);
	} else {
		cmc623_manual_pwm_brightness(data);
	}
}

// value: 0 ~ 1600
void cmc623_pwm_brightness_bypass(int value)
{
	int data;
	dprintk(KERN_INFO "%s : BL brightness level = %d\n", __func__, value);

	if(value<0)
		data = 0;
	else if(value>1600)
		data = 1600;
	else
		data = value;

	cmc623_state.brightness = data;
#if 0	
	if(cmc623_state.cabc_enabled == TRUE) {
		cmc623_cabc_pwm_brightness(data);
	}
	else
#endif		
	{
		cmc623_manual_pwm_brightness(data);
	}
}


void cmc623_pwm_apply(int level)
{
/*	
	int i = 0;
	for(i=0; i<100; i++)
	{
		if(Islcdonoff())
			break;
		
		msleep(20);
	};
*/
	if(Islcdonoff()) {
		dprintk(KERN_INFO "%s : BL brightness level = %d\n", __func__, level);
		
		cmc623_pwm_brightness(level);
	}
	current_gamma_level = level;
}
EXPORT_SYMBOL(cmc623_pwm_apply);


int cmc623_gpio_init(void)
{
	int ret;
	printk(KERN_INFO "%s called\n", __func__);

	/* LVDS GPIO Initialize */
	ret = gpio_request(p_cmc623_data->pdata.failsafeb_gpio, "GPIO_IMA_PWREN");
	if (ret) {
		printk(KERN_ERR "failed to request CMC623 GPIO%d\n",
				p_cmc623_data->pdata.failsafeb_gpio);
		return ret;
	}

	ret = gpio_request(p_cmc623_data->pdata.reset_gpio, "GPIO_IMA_N_RST");
	if (ret) {
		printk(KERN_ERR "failed to request CMC623 GPIO%d\n",
				p_cmc623_data->pdata.reset_gpio);
		return ret;
	}

	ret = gpio_request(p_cmc623_data->pdata.bypass_gpio, "GPIO_IMA_BYPASS");
	if (ret) {
		printk(KERN_ERR "failed to request CMC623 GPIO%d\n",
				p_cmc623_data->pdata.bypass_gpio);
		return ret;
	}

	ret = gpio_request(p_cmc623_data->pdata.sleep_gpio, "GPIO_IMA_SLEEP");
	if (ret) {
		printk(KERN_ERR "failed to request CMC623 GPIO%d\n",
				p_cmc623_data->pdata.sleep_gpio);
		return ret;
	}

       	ret = gpio_request(p_cmc623_data->pdata.backlight_en_gpio, "GPIO_LCD_LDO_LED_EN");
	if (ret) {
		printk(KERN_ERR "failed to request CMC623 GPIO%d\n",
				p_cmc623_data->pdata.backlight_en_gpio);
		return ret;
	}

	if (p_cmc623_data->pdata.powerctrl)
		p_cmc623_data->pdata.powerctrl(0, 0);

	ret = gpio_direction_output(p_cmc623_data->pdata.failsafeb_gpio, 0);
	ret = gpio_direction_output(p_cmc623_data->pdata.bypass_gpio, 0);
	ret = gpio_direction_output(p_cmc623_data->pdata.sleep_gpio, 0);
	ret = gpio_direction_output(p_cmc623_data->pdata.reset_gpio, 1);

	msleep(1);
	
	if (p_cmc623_data->pdata.powerctrl)
		p_cmc623_data->pdata.powerctrl(1, 1);

	ret = gpio_direction_output(p_cmc623_data->pdata.backlight_en_gpio, 1);
	if (ret < 0)
		goto cleanup;
    
	ret = gpio_direction_output(p_cmc623_data->pdata.failsafeb_gpio, 1);
	if (ret < 0)
		goto cleanup;
       	msleep(1);
       
       	ret = gpio_direction_output(p_cmc623_data->pdata.bypass_gpio, 1);
	if (ret < 0)
		goto cleanup;
        msleep(1);
            
	ret = gpio_direction_output(p_cmc623_data->pdata.sleep_gpio, 1);
	if (ret < 0)
		goto cleanup;
        msleep(5);
#if 0        
	ret = gpio_direction_output(p_cmc623_data->pdata.reset_gpio, 0);
	if (ret < 0)
		goto cleanup;
       msleep(5);
#endif	
	ret = gpio_direction_output(p_cmc623_data->pdata.reset_gpio, 0);
	if (ret < 0)
		goto cleanup;
       	msleep(5);
       
	ret = gpio_direction_output(p_cmc623_data->pdata.reset_gpio, 1);
	if (ret < 0)
		goto cleanup;

       	msleep(16);

	return 0;

cleanup:
	gpio_free(p_cmc623_data->pdata.failsafeb_gpio);
	gpio_free(p_cmc623_data->pdata.reset_gpio);
	gpio_free(p_cmc623_data->pdata.bypass_gpio);
	gpio_free(p_cmc623_data->pdata.sleep_gpio);
	gpio_free(p_cmc623_data->pdata.backlight_en_gpio);
    
	return ret;	
}

#ifdef CONFIG_HAS_EARLYSUSPEND
void cmc623_suspend(struct early_suspend *h)
{
	printk(KERN_INFO "-0- %s called -0-\n", __func__);

	cmc623_state.suspended = TRUE;
	
	lcdonoff = FALSE;

	if(!p_cmc623_data)
	{
		printk(KERN_ERR "%s cmc623 is not initialized\n", __func__);
		return;
	}

	/* 1.2V/1.8V/3.3V may be on

	CMC623[0x07] := 0x0004
   	cmc623_I2cWrite16(0x07, 0x0004);*/

	if (p_cmc623_data->pdata.powerctrl)
		p_cmc623_data->pdata.powerctrl(0, 0);

	gpio_set_value(p_cmc623_data->pdata.backlight_en_gpio, GPIO_LEVEL_LOW);
	
	/* CMC623 SLEEPB <= LOW*/
	gpio_set_value(p_cmc623_data->pdata.sleep_gpio, GPIO_LEVEL_LOW);

	/*CMC623 BYPASSB <= LOW*/
	gpio_set_value(p_cmc623_data->pdata.bypass_gpio, GPIO_LEVEL_LOW);

	/* wait 1ms*/
	msleep(1);

	/* CMC623 FAILSAFEB <= LOW*/
	gpio_set_value(p_cmc623_data->pdata.failsafeb_gpio, GPIO_LEVEL_LOW);

	msleep(100);


	/*cmc623_state.suspended = TRUE;*/
	
	return;
}
EXPORT_SYMBOL(cmc623_suspend);


int cmc623_pre_resume(void)
{
	printk(KERN_INFO "-0- %s called -0-\n", __func__);

	if (p_cmc623_data->pdata.powerctrl)
		p_cmc623_data->pdata.powerctrl(1, 1);

	gpio_set_value(p_cmc623_data->pdata.backlight_en_gpio, GPIO_LEVEL_HIGH);

	gpio_set_value(p_cmc623_data->pdata.reset_gpio, GPIO_LEVEL_HIGH);
	gpio_set_value(p_cmc623_data->pdata.failsafeb_gpio, GPIO_LEVEL_LOW);
	gpio_set_value(p_cmc623_data->pdata.bypass_gpio, GPIO_LEVEL_LOW);
	gpio_set_value(p_cmc623_data->pdata.sleep_gpio, GPIO_LEVEL_LOW);

	msleep(1); 

	/*printk(KERN_INFO "-0- %s end -0-\n", __func__);*/
	
	return 0;
}
EXPORT_SYMBOL(cmc623_pre_resume);

/*CAUTION : pre_resume function must be called before using this function*/
void cmc623_resume(struct early_suspend *h)
{
	cmc623_pre_resume();
	printk(KERN_INFO "-0- %s called -0-\n", __func__);		

	msleep(1);
	/* FAILSAFEB <= HIGH */           
	gpio_set_value(p_cmc623_data->pdata.failsafeb_gpio, GPIO_LEVEL_HIGH);
	msleep(1);

	/* BYPASSB <= HIGH*/
	gpio_set_value(p_cmc623_data->pdata.bypass_gpio, GPIO_LEVEL_HIGH);
	msleep(1);

	/* SLEEPB <= HIGH*/
	gpio_set_value(p_cmc623_data->pdata.sleep_gpio, GPIO_LEVEL_HIGH);
	msleep(1);

	/*  RESETB(K6) <= HIGH*/
	gpio_set_value(p_cmc623_data->pdata.reset_gpio, GPIO_LEVEL_HIGH);
	msleep(2);// wait 1ms or above

	/*  RESETB(K6) <= LOW*/
	gpio_set_value(p_cmc623_data->pdata.reset_gpio, GPIO_LEVEL_LOW);
	msleep(5);// wait 4ms or above

	/* RESETB(K6) <= HIGH*/
	gpio_set_value(p_cmc623_data->pdata.reset_gpio, GPIO_LEVEL_HIGH);
        msleep(4);// wait 3ms or above

	cmc623_state.suspended = FALSE;
	if (!p_cmc623_data->pdata.init_in_bypass && !current_bypass_onoff) {

		if(!p_cmc623_data) {
			printk(KERN_ERR "%s cmc623 is not initialized\n", __func__);
			return;
		}
	
		CMC623_SetUp();
		CMC623_Set_Mode();

		if(current_bypass_onoff) {
			bypass_onoff_ctrl(current_bypass_onoff);
			cmc623_pwm_brightness_bypass(current_gamma_level);
		} else {
			cmc623_pwm_brightness(current_gamma_level);
		}
	} else {    
		cmc623_I2cWrite16(0x00, 0x0000);	/* Bank 0 */
		cmc623_I2cWrite16(0x07, 0x0004);	/* System PLL */
		cmc623_I2cWrite16(0x1c, 0x0002);	/* pull-up PWM */
		cmc623_I2cWrite16(0x0A, 0x0080);	/* PWM Clock Off */
		msleep(2);
		gpio_set_value(p_cmc623_data->pdata.sleep_gpio, 0);
		gpio_set_value(p_cmc623_data->pdata.bypass_gpio, 0);
		msleep(1);
		gpio_set_value(p_cmc623_data->pdata.failsafeb_gpio, 0);
		if (p_cmc623_data->pdata.powerctrl)
			p_cmc623_data->pdata.powerctrl(0, 1);
		msleep(1);
		gpio_set_value(p_cmc623_data->pdata.failsafeb_gpio, 1);
	}
	
	lcdonoff = TRUE;

#if 0
	// restore mode & cabc status
	setting_first = TRUE;
	cmc623_state.brightness = 0;
	cmc623_cabc_enable(cmc623_state.cabc_enabled);
	setting_first = FALSE;

	msleep(10);
#endif
	
	return;
}
EXPORT_SYMBOL(cmc623_resume);
#endif

void cmc623_shutdown(struct i2c_client *client)
{
	printk(KERN_INFO "-0- %s called -0-\n", __func__);

	cmc623_state.suspended = TRUE;
	
	lcdonoff = FALSE;

	if(!p_cmc623_data) {
		printk(KERN_ERR "%s cmc623 is not initialized\n", __func__);
		return;
	}

	/* 1.2V/1.8V/3.3V may be on

	CMC623[0x07] := 0x0004
	cmc623_I2cWrite16(0x07, 0x0004);*/

	/* CMC623 SLEEPB <= LOW*/
	gpio_set_value(p_cmc623_data->pdata.sleep_gpio, 0);

	/* CMC623 BYPASSB <= LOW*/
	gpio_set_value(p_cmc623_data->pdata.bypass_gpio, 0);

	/* wait 1ms*/
	msleep(1);

	/* CMC623 FAILSAFEB <= LOW*/
	gpio_set_value(p_cmc623_data->pdata.failsafeb_gpio, 0);

	msleep(100);

	return;
}

#ifdef CABC_ONOFF_TEST
static int current_cabc_onoff = 1;

static ssize_t cabc_onoff_file_cmd_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	printk(KERN_INFO "called %s \n", __func__);

	return sprintf(buf,"%u\n",current_cabc_onoff);
}

static ssize_t cabc_onoff_file_cmd_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
	int value;
	
	sscanf(buf, "%d", &value);

	printk(KERN_INFO "[cabc set] in cabc_onoff_file_cmd_store, input value = %d \n",value);

	if((cmc623_state.suspended == FALSE) && (lcdonoff == TRUE)&&(current_bypass_onoff == FALSE)) {
		if((current_cabc_onoff==0) && (value == 1)) {
			cabc_onoff_ctrl(value);
			current_cabc_onoff = 1;
			cmc623_state.cabc_enabled = TRUE;
			printk(KERN_INFO "[cabc_on] <= value : %d \n",value);
			cmc623_pwm_brightness(current_gamma_level);
		} else if((current_cabc_onoff==1) &&(value == 0)) {
			cabc_onoff_ctrl(value);
			current_cabc_onoff = 0;
			cmc623_state.cabc_enabled = FALSE;
			printk(KERN_INFO "[cabc_off] <= value : %d \n",value);
			cmc623_pwm_brightness(current_gamma_level);
		} else {
			printk(KERN_INFO "[cabc set] cabc is already = %d \n",current_cabc_onoff);
		}
	} else {
		printk(KERN_INFO "[cabc set] LCD is suspend = %d \n",cmc623_state.suspended);
	}	
	return size;
}

static DEVICE_ATTR(cabconoff, 0666, cabc_onoff_file_cmd_show, cabc_onoff_file_cmd_store);
#endif

static void cmc623_reinit(void)
{
	// FAILSAFEB <= HIGH            
	gpio_set_value(p_cmc623_data->pdata.failsafeb_gpio, GPIO_LEVEL_HIGH);
	msleep(1);

	// BYPASSB <= HIGH
	gpio_set_value(p_cmc623_data->pdata.bypass_gpio, GPIO_LEVEL_HIGH);
	//gpio_set_value(p_cmc623_data->pdata.bypass_gpio, GPIO_LEVEL_LOW);
	msleep(1);

	// SLEEPB <= HIGH
	gpio_set_value(p_cmc623_data->pdata.sleep_gpio, GPIO_LEVEL_HIGH);
	msleep(1);

	// RESETB <= LOW
	gpio_set_value(p_cmc623_data->pdata.reset_gpio, GPIO_LEVEL_LOW);

	// wait 4ms or above
	msleep(5);

	// RESETB(K6) <= HIGH
	gpio_set_value(p_cmc623_data->pdata.reset_gpio, GPIO_LEVEL_HIGH);

	// wait 0.3ms or above
	msleep(1);	//udelay(300);
	
	cmc623_state.suspended = FALSE;

	// set registers using I2C
	if(!p_cmc623_data)
	{
		printk(KERN_ERR "%s cmc623 is not initialized\n", __func__);
		return;
	}
	
	CMC623_SetUp();
		
	lcdonoff = TRUE;

}

void bypass_onoff_ctrl(int value)
{
	if(value == 1) {
		int i = 0;
		int num = ARRAY_SIZE(cmc623_bypass);

		//gpio_set_value(p_cmc623_data->pdata.bypass_gpio, 0); //GPIO bypass pin low <== bypass on

		cmc623_reinit();		
		
		for (i=0; i<num; i++)
		{
			cmc623_I2cWrite16(cmc623_bypass[i].RegAddr, cmc623_bypass[i].Data);
		}
		cmc623_pwm_brightness_bypass(current_gamma_level);
		printk(KERN_INFO "[bypass_onoff] = bypass_on : %d \n",value);
		
	} else if(value == 0) {
		cabc_onoff_ctrl(current_cabc_onoff);
		cmc623_state.cabc_enabled = current_cabc_onoff;
		current_bypass_onoff = 0;
		cmc623_pwm_brightness(current_gamma_level);
		printk(KERN_INFO "[bypass_onoff] = bypass_off : %d \n",value);
	}
}

static ssize_t bypass_onoff_file_cmd_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	printk(KERN_INFO "called %s \n", __func__);

	return sprintf(buf,"%u\n",current_bypass_onoff);
}

static ssize_t bypass_onoff_file_cmd_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
	int value;
	
	sscanf(buf, "%d", &value);

	printk(KERN_INFO "[bypass set] in bypass_onoff_file_cmd_store, input value = %d \n",value);

	if((cmc623_state.suspended == FALSE) && (lcdonoff == TRUE))
	{
		if((current_bypass_onoff==0) && (value == 1)) {
			bypass_onoff_ctrl(value);
			current_bypass_onoff = 1;
		} else if((current_bypass_onoff==1) &&(value == 0)) {
			bypass_onoff_ctrl(value);
			current_bypass_onoff = 0;
		} else {
			printk(KERN_INFO "[bypass set] bypass is already = %d \n",current_bypass_onoff);
		}
	}
	else
	{
		printk(KERN_INFO "[bypass set] LCD is suspend = %d \n",cmc623_state.suspended);
	}

	return size;
}

static DEVICE_ATTR(bypassonoff, 0666, bypass_onoff_file_cmd_show, bypass_onoff_file_cmd_store);

#ifdef CMC623_TUNING
bool cmc623_tune(unsigned long num)
{
	unsigned int i;

	printk(KERN_INFO "========== Start of tuning CMC623 Jun  ==========\n");

	for (i=0; i<num; i++) 
	{
		printk(KERN_INFO "[%2d] Writing => reg: 0x%2x, data: 0x%4x\n", i+1, Cmc623_TuneSeq[i].RegAddr, Cmc623_TuneSeq[i].Data);

		if (0 > cmc623_I2cWrite16(Cmc623_TuneSeq[i].RegAddr, Cmc623_TuneSeq[i].Data)) {
			printk(KERN_INFO "I2cWrite16 failed\n");
			return 0;
		} else {
			printk(KERN_INFO "I2cWrite16 succeed\n");
		}

		if ( Cmc623_TuneSeq[i].RegAddr == CMC623_REG_SWRESET && Cmc623_TuneSeq[i].Data == 0xffff ) {
			mdelay(3);
		}
	}
	printk(KERN_INFO "==========  End of tuning CMC623 Jun  ==========\n");
	return 1;
}

static int parse_text(char * src, int len)
{
	int i,count, ret;
	int index=0;
	char * str_line[CMC623_MAX_SETTINGS];
	char * sstart;
	char * c;
	unsigned int data1, data2;

	c = src;
	count = 0;
	sstart = c;
    
	for(i=0; i<len; i++,c++) {
		char a = *c;
		if(a=='\r' || a=='\n') {
			if(c > sstart) {
				str_line[count] = sstart;
				count++;
			}
			*c='\0';
			sstart = c+1;
		}
	}
    
	if(c > sstart) {
		str_line[count] = sstart;
		count++;
	}

	printk(KERN_INFO "----------------------------- Total number of lines:%d\n", count);

	for(i=0; i<count; i++) {
		printk(KERN_INFO "line:%d, [start]%s[end]\n", i, str_line[i]);
		ret = sscanf(str_line[i], "0x%x,0x%x\n", &data1, &data2);
		printk(KERN_INFO "Result => [0x%2x 0x%4x] %s\n", data1, data2, (ret == 2) ? "Ok" : "Not available");
		if(ret == 2) 
        	{   
			Cmc623_TuneSeq[index].RegAddr = (unsigned char)data1;
			Cmc623_TuneSeq[index++].Data  = (unsigned long)data2;
		}
	}
	return index;
}

static int cmc623_load_data(void)
{
	struct file *filp;
	char	*dp;
	long	l, i ;
	loff_t  pos;
	int     ret, num;
	mm_segment_t fs;

	klogi("cmc623_load_data start!");

	fs = get_fs();
	set_fs(get_ds());

	filp = filp_open(CMC623_PATH_TUNING_DATA, O_RDONLY, 0);
	if(IS_ERR(filp)) {
		kloge("file open error:%d", (s32)filp);

		return -1;
	}

	l = filp->f_path.dentry->d_inode->i_size;
	klogi("Size of the file : %ld(bytes)", l);

	//dp = kmalloc(l, GFP_KERNEL);
	dp = kmalloc(l+10, GFP_KERNEL);		// add cushion
	if(dp == NULL) {
		kloge("Out of Memory!");
		filp_close(filp, current->files);
		return -1;
	}
	pos = 0;
	memset(dp, 0, l);

    	kloge("== Before vfs_read ======");
	ret = vfs_read(filp, (char __user *)dp, l, &pos);   // P1_LSJ : DE08 : ¿©±â¼­ Á×À½ 
    	kloge("== After vfs_read ======");

	if(ret != l) {
		kloge("<CMC623> Failed to read file (ret = %d)", ret);
		kfree(dp);
		filp_close(filp, current->files);
		return -1;
	}

	filp_close(filp, current->files);

	set_fs(fs);

	for(i=0; i<l; i++)
    	{   
		printk(KERN_INFO "%x ", dp[i]);
    	}
	printk(KERN_INFO "\n");

	num = parse_text(dp, l);

	if(!num) {
		kloge("Nothing to parse!");
		kfree(dp);
		return -1;
	}
		
	printk(KERN_INFO "------ Jun Total number of parsed lines: %d\n", num);
	cmc623_tune(num);

	kfree(dp);
	return num;
}

static ssize_t tune_cmc623_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret=0;

	klogi("");
	ret = cmc623_load_data();

	if(ret<0)
		return sprintf(buf, "FAIL\n");
	else
		return sprintf(buf, "OK\n");
}

static ssize_t tune_cmc623_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t size)
{
	return size;
}

static DEVICE_ATTR(tune, 0666, tune_cmc623_show, tune_cmc623_store);
#endif	//CMC623_TUNING

#if CMC_ONOFF_TEST
static int current_cmc_onoff = 1;

static void cmc_on(void)
{
	current_cmc_onoff = 1;
}

static void cmc_off(void)
{
	int ret;
	
	ret = gpio_direction_output(p_cmc623_data->pdata.bypass_gpio, 0);

	current_cmc_onoff = 0;
}


static ssize_t cmc_onoff_file_cmd_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	printk(KERN_INFO "called %s \n", __func__);

	return sprintf(buf,"%u\n",current_cmc_onoff);
}


static ssize_t cmc_onoff_file_cmd_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
	int value;
	
	sscanf(buf, "%d", &value);

	printk(KERN_INFO "[CMC set] in cmc_onoff_file_cmd_store, input value = %d \n",value);

	if((cmc623_state.suspended == FALSE) && (lcdonoff == TRUE)) {
		if((current_cmc_onoff==0) && (value == 1)) {
			cmc_on();
			current_cmc_onoff = 1;
			printk(KERN_INFO "[CMC_on] <= value : %d \n",value);
			//cmc623_pwm_brightness(current_gamma_level);
		} else if((current_cmc_onoff==1) &&(value == 0)) {
			cmc_off();
			current_cmc_onoff = 0;
			printk(KERN_INFO "[CMC_off] <= value : %d \n",value);
			//cmc623_pwm_brightness(current_gamma_level);
		} else {
			printk(KERN_INFO "[CMC set] CMC is already = %d \n",current_cmc_onoff);
		}
		
	} else {
		printk(KERN_INFO "[CMC set] LCD is suspend = %d \n",cmc623_state.suspended);
	}

	return size;
}

static DEVICE_ATTR(cmconoff, 0666, cmc_onoff_file_cmd_show, cmc_onoff_file_cmd_store);
#endif

extern struct class *sec_class;
struct device *tune_cmc623_dev;

static int __devinit cmc623_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct cmc623_data *data;
	int ret=0;
	struct backlight_device *bd;
	struct Cmc623PlatformData *pdata = client->dev.platform_data;

	printk(KERN_INFO "==============================\n");
	printk(KERN_INFO "cmc623 attach START!!!        \n");
	printk(KERN_INFO "==============================\n");

	data = kzalloc(sizeof(struct cmc623_data), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
    	}

	data->client = client;
	i2c_set_clientdata(client, data);

	dev_info(&client->dev, "cmc623 i2c probe success!!!\n");

	p_cmc623_data = data;

	p_cmc623_data->pdata.reset_gpio 	= pdata->reset_gpio;
	p_cmc623_data->pdata.bypass_gpio 	= pdata->bypass_gpio;
	p_cmc623_data->pdata.sleep_gpio 	= pdata->sleep_gpio;
	p_cmc623_data->pdata.failsafeb_gpio 	= pdata->failsafeb_gpio;
	p_cmc623_data->pdata.backlight_en_gpio 	= pdata->backlight_en_gpio;
	p_cmc623_data->pdata.init_in_bypass 	= pdata->init_in_bypass;
	p_cmc623_data->pdata.powerctrl 		= pdata->powerctrl;

	cmc623_gpio_init();

	cmc623_state.suspended = FALSE;
	if (!p_cmc623_data->pdata.init_in_bypass) {
		if(!p_cmc623_data) {
			printk(KERN_ERR "%s cmc623 is not initialized\n", __func__);
			return 0;
		}	

		current_bypass_onoff = FALSE;

		// set registers using I2C
		ret = CMC623_SetUp();
		printk(KERN_ERR "%s CMC623_SetUp is %d\n", __func__, ret);
		//ret = CMC623_Set_Mode();
		CMC623_Set_Mode();
		printk(KERN_ERR "%s CMC623_Set_Mode is %d\n", __func__, ret);
	} else {
		cmc623_I2cWrite16(0x00, 0x0000);	/* Bank 0 */
		cmc623_I2cWrite16(0x07, 0x0004);	/* System PLL */
		cmc623_I2cWrite16(0x1c, 0x0002);	/* pull-up PWM */
		cmc623_I2cWrite16(0x0A, 0x0080);	/* PWM Clock Off */
		msleep(2);
		gpio_set_value(p_cmc623_data->pdata.sleep_gpio, 0);
		gpio_set_value(p_cmc623_data->pdata.bypass_gpio, 0);
		msleep(1);
		gpio_set_value(p_cmc623_data->pdata.failsafeb_gpio, 0);
		if (p_cmc623_data->pdata.powerctrl)
			p_cmc623_data->pdata.powerctrl(0, 1);
		msleep(1);
		gpio_set_value(p_cmc623_data->pdata.failsafeb_gpio, 1);
		current_bypass_onoff = TRUE;
	}

#if 0
	// restore mode & cabc status
	setting_first = TRUE;
	cmc623_state.brightness = 0;
	cmc623_cabc_enable(cmc623_state.cabc_enabled);
	setting_first = FALSE;

	msleep(10);
#endif

#ifdef CMC623_TUNING
//	cmc623_set_tuning();	//for test
#endif

#if 0
	setting_first = TRUE;
//	cmc623_cabc_enable(cmc623_state.cabc_enabled);
	setting_first = FALSE;
	
//	ret = cmc623_gamma_set();                             // P1_LSJ : DE19
    //printk("cmc623_gamma_set Return value  (%d)\n", ret);

//	cmc623_i2c_client = c;
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
	cmc623_early_suspend.level =  EARLY_SUSPEND_LEVEL_DISABLE_FB - 1;
	cmc623_early_suspend.suspend = cmc623_suspend;
	cmc623_early_suspend.resume = cmc623_resume;
	register_early_suspend(&cmc623_early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */


	tune_cmc623_dev = device_create(sec_class, NULL, 0, NULL, "sec_tune_cmc623");

	if (IS_ERR(tune_cmc623_dev)) 
    	{
		printk(KERN_INFO "Failed to create device!");
		ret = -1;
	}
#if CABC_ONOFF_TEST
	if (device_create_file(tune_cmc623_dev, &dev_attr_cabconoff) < 0) {
		printk(KERN_INFO "Failed to create device file!(%s)!\n", dev_attr_cabconoff.attr.name);
		ret = -1;
	}
#endif

#if BYPASS_ONOFF_TEST
	if (device_create_file(tune_cmc623_dev, &dev_attr_bypassonoff) < 0) {
		printk(KERN_INFO "Failed to create device file!(%s)!\n", dev_attr_bypassonoff.attr.name);
		ret = -1;
	}
#endif


#if CMC623_TUNING
	if (device_create_file(tune_cmc623_dev, &dev_attr_tune) < 0) {
		printk(KERN_INFO "Failed to create device file!(%s)!\n", dev_attr_tune.attr.name);
		ret = -1;
	}
#endif	

#if CMC_ONOFF_TEST
		if (device_create_file(tune_cmc623_dev, &dev_attr_cmconoff) < 0) {
			printk(KERN_INFO "Failed to create device file!(%s)!\n", dev_attr_cmconoff.attr.name);
			ret = -1;
		}
#endif

/*	
	ove_wq = create_singlethread_workqueue("ove_wq");
	INIT_WORK(&work_ove, ove_workqueue_func);
*/
	//cabc_onoff_ctrl(TRUE);
	cmc623_state.suspended = FALSE;
	lcdonoff = TRUE;

	printk(KERN_INFO "<cmc623_i2c_driver Add END>   \n");

	bd = backlight_device_register("cmc-pwm-backlight", &client->dev, p_cmc623_data,
					 &cmc623_pwm_ops, &cmc623_backlight_props);
	if (IS_ERR(bd))
		return PTR_ERR(bd);

	p_cmc623_data->bd = bd;

	dev_info(&client->dev, "cmc623_pwm backlight driver is enabled.\n");

	return 0;
}

static int __devexit cmc623_i2c_remove(struct i2c_client *client)
{
	struct cmc623_data *data = i2c_get_clientdata(client);

	if (ove_wq)
		destroy_workqueue(ove_wq);

	backlight_device_unregister(data->bd);

	p_cmc623_data = NULL;

	i2c_set_clientdata(client, NULL);

	kfree(data);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&cmc623_early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

	dev_info(&client->dev, "cmc623 i2c remove success!!!\n");

	return 0;
}

static const struct i2c_device_id cmc623[] = {
	{ CMC623_I2C_DEVICE_NAME, 0 },
};
MODULE_DEVICE_TABLE(i2c, cmc623);

struct i2c_driver cmc623_i2c_driver =  
{
	.driver	= {
		.name	= "image_convertor",
		.owner = THIS_MODULE,
	},
	.probe 		= cmc623_i2c_probe,
	.remove 	= __devexit_p(cmc623_i2c_remove),
	.id_table	= cmc623,
#if !(defined CONFIG_HAS_EARLYSUSPEND)
    	.suspend = cmc623_suspend,
	.resume  = cmc623_resume,	
#endif	
	.shutdown = cmc623_shutdown,
};

static int __init cmc623_init(void)
{
	printk(KERN_INFO "**** < cmc623_init  > *****\n");

	return i2c_add_driver(&cmc623_i2c_driver);    
}

static void __exit cmc623_exit(void)
{
	i2c_del_driver(&cmc623_i2c_driver);
}

module_init(cmc623_init);
module_exit(cmc623_exit);

/* Module information */
MODULE_AUTHOR("Samsung");
MODULE_DESCRIPTION("Tuning CMC623 image converter");
MODULE_LICENSE("GPL");
