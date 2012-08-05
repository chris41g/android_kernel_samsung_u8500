/*
 *	JACK device detection driver.
 *
 *	Copyright (C) 2009 Samsung Electronics, Inc.
 *	Copyright (C) 2010 Samsung Electronics, Inc.
 *
 *	Authors:
 *		Uk Kim <w0806.kim@samsung.com>
 *		20101019 PC Kennedy
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; version 2 of the License.
 */

#include <linux/module.h>
#include <linux/sysdev.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/switch.h>
#include <linux/input.h>
#include <linux/timer.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/mfd/ab8500.h>
#include <linux/mfd/abx500.h>
#include <linux/mfd/ab8500/ab8500-gpadc.h>
#include <linux/spinlock.h>
#include <asm/mach-types.h>

#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/sec_jack.h>
#include <mach/board-sec-u8500.h>

extern unsigned int system_rev;

#define CONFIG_DEBUG_SEC_JACK

#define SUBJECT "JACK_DRIVER"

#ifdef CONFIG_DEBUG_SEC_JACK
#define SEC_JACKDEV_DBG(format,...)\
	printk ("[ "SUBJECT " (%s,%d) ] " format "\n", __func__, __LINE__, ## __VA_ARGS__);

#else
#define SEC_JACKDEV_DBG(format,...)
#define DEBUG_LOG(format,...)
#endif

#define KEYCODE_SENDEND 248

static spinlock_t jacklock = SPIN_LOCK_UNLOCKED; //Initialise
static struct platform_driver sec_jack_driver;

struct class *jack_class;
EXPORT_SYMBOL(jack_class);
static struct device *jack_selector_fs;				// Sysfs device, this is used for communication with Cal App.
EXPORT_SYMBOL(jack_selector_fs);

typedef enum
{
	REMOVED = 0,
	REMOVING,
	INSERTING,
	INSERTED,
} jack_status_t;


struct sec_jack_info
{
	struct input_dev *input;
	struct device *dev;
	struct ab8500* pAb8500;
	struct work_struct jack_detect_work;
	struct work_struct jack_micref_work;
	struct work_struct jack_button_work;
	const char	*pRegIDMicRef;
	struct regulator *pRegMicRef;
	struct timer_list jack_detect_timer;
	struct timer_list jack_initstate_timer;
	unsigned int current_jack_type_status;
	struct wake_lock jack_sendend_wake_lock;
	struct mutex jack_mutex;
	jack_status_t jack_status;
	
	int send_end_status;
	int volume_up_status;
	int volume_down_status;
	int four_pole_25k_status;
	int mic_status;

	struct mutex regulator_lock;
};

static struct sec_jack_info *pJack;

struct switch_dev switch_jack_detection = {
		.name = "h2w",
};

struct switch_dev switch_sendend = {
		.name = "send_end",
};

struct switch_dev volume_up = {
		.name = "volume_up",
};
struct switch_dev volume_down = {
		.name = "volume_down",
};


/* AB8500 Accessory Detect interrupt resources */
#define ID_ACCDET1_R		"ACC_DETECT_1DB_R"
#define ID_ACCDET1_F		"ACC_DETECT_1DB_F"
#define ID_ACCDET21_R		"ACC_DETECT_21DB_R"
#define ID_ACCDET21_F		"ACC_DETECT_21DB_F"

#define SEC_ACCDETECT2_ADC_CHANNEL 5

#define ACCDET1TH_DB		0x35		/* ACCDET1 0.9V Threshold. Debounce 50ms*/
#define ACCDET2TH1_TH2		0xB3		/* ACCDET2 TH1 0.5V TH2 1.4V */

/* ACCDETECT CTRL register bits */
#define ACCDET1_ENA			(0x01 << 0)
#define ACCDET_VREF_ENA		(0x01 << 1)
#define ACCDET2_PULLUP_ENA	(0x01 << 3)
#define ACCDET21_ENA		(0x01 << 4)
#define ACCDET22_ENA		(0x01 << 5)

/*
 * Accessory detection register offsets
 * Bank : 0x08 - register missing from ab8500.h
 */
#define AB8500_ACC_DET_DB1_REG	0x0880
#define AB8500_ACC_DET_DB2_REG	0x0881
#define AB8500_ACC_DET_CTRL_REG	0x0882

#define INITIAL_CHECK_TIME	500		/* 500 ms */

#define DETECTION_CHECK_TIME	500		/* 500 ms */

/*
 * ADC levels for devices and system version
 */
#define ADC_SEC_HS_3_POLE_DEVICE	600  /* smaller than this means 3 pole headset jack */
#define ADC_SEC_HS_4_POLE_DEVICE	1600 /* smaller than this means 4 pole headset jack */
#define ADC_SEC_HS_4_POLE_25K_DEVICE 1800 /* 4 pole with 25kohm mic */

#define ADC_SEC_NO_DEVICE			1800 /* greater than this means no device */

#define ADC1_SEC_DETECT_TH		900 /* smaller than this means no plug detected */

/*
 * ADC2 levels to distinguish sendend/volume up/down keys
 */
#define JACK_BUTTON_SENDEND_ADC 90
#define JACK_BUTTON_VOLUMEUP_ADC 180
#define JACK_BUTTON_VOUMEDOWN_ADC 450

#define JACK_BUTTON_25K_SENDEND_ADC 30

#define JACK_BUTTON_25K_VOLUMEUP_ADC_LOW 90
#define JACK_BUTTON_25K_VOLUMEUP_ADC_HIGH 110

#define JACK_BUTTON_25K_VOUMEDOWN_ADC_LOW 210
#define JACK_BUTTON_25K_VOUMEDOWN_ADC_HIGH 235


static void jack_input_selector(int jack_type_status);
static void button_work(struct work_struct *pWork);

static void mic_pwr( bool on, struct sec_jack_info* pJack )
{
	static bool is_on = false;

	if (!pJack->pRegMicRef) {
		WARN_ON(1);
		return;	
	}

	SEC_JACKDEV_DBG("mic_pwr on[%d] mic_satus[%d]\n", on, pJack->mic_status);
		
	mutex_lock(&pJack->regulator_lock);

	if(on){
		if(!pJack->mic_status)
		{
			SEC_JACKDEV_DBG("mic_pwr: enable mic pwr\n");
			regulator_enable(pJack->pRegMicRef);
			pJack->mic_status = true;
		}
	} else {
		if(pJack->mic_status)
		{
			SEC_JACKDEV_DBG("mic_pwr: disable mic pwr\n");
			regulator_disable( pJack->pRegMicRef );
			pJack->mic_status = false;
		}
	}

	mutex_unlock(&pJack->regulator_lock);

}

static int jack_type_status( struct sec_jack_info* pData )
{
	int adc1, adc2;
	int status;

	adc1 = ab8500_gpadc_convert(ab8500_gpadc_get(), ACC_DETECT1);
	adc2 = ab8500_gpadc_convert(ab8500_gpadc_get(), ACC_DETECT2);

	pr_info("headset detect : ADC1 value = %d\n", adc1);
	pr_info("headset detect : ADC2 value = %d\n", adc2);	

	if ( adc1 < ADC1_SEC_DETECT_TH ) // plug detected
	{
		
		if ( adc2 < ADC_SEC_HS_3_POLE_DEVICE )
		{
			status = SEC_HEADSET_3_POLE_DEVICE;
			pData->four_pole_25k_status = 0;
		}
		else if ( adc2 < ADC_SEC_HS_4_POLE_DEVICE )
		{
			status = SEC_HEADSET_4_POLE_DEVICE;
			pData->four_pole_25k_status = 0;
		}
		else if ( adc2 < ADC_SEC_HS_4_POLE_25K_DEVICE )
		{
			SEC_JACKDEV_DBG("25kohm headset inserted.\n");
			status = SEC_HEADSET_4_POLE_DEVICE;
			pData->four_pole_25k_status = 1;
			
		}
		else if ( adc2 >= ADC_SEC_NO_DEVICE )
		{
			SEC_JACKDEV_DBG("Jack removed or not fully inserted\n");
			/* Default to 3 pole device, so we can still detect in case FM radio measurement is plugged in */
			status = SEC_HEADSET_3_POLE_DEVICE; 
			pData->four_pole_25k_status = 0;
		}
		else
		{
			pr_err("jack detected but unknown device : adc2 = %d\n", adc2);
			pData->four_pole_25k_status = 0;
			status = SEC_UNKNOWN_DEVICE;
		}
	}
	else // plug not detected
	{
		pr_err("jack not detected\n");
		status = SEC_JACK_NO_DEVICE;
	}
	return status;
}
static void button_work(struct work_struct *pWork)
{
	int adc2;
	int status;
	int sendend_adc, volumeup_adc, volumedown_adc;
	struct sec_jack_info* pJack = container_of(pWork, struct sec_jack_info, jack_button_work);

	SEC_JACKDEV_DBG("button work\n");
	
	if ( INSERTED == pJack->jack_status )
	{
		adc2 = ab8500_gpadc_convert(ab8500_gpadc_get(), ACC_DETECT2);
		
		SEC_JACKDEV_DBG("headset detect : ADC2 value = %d \n", adc2);	

		if (adc2 > 500)
		{

			if(pJack->send_end_status)
			{
				/* switch released*/
				SEC_JACKDEV_DBG("SEND/END released.\n");
				pJack->send_end_status = 0;
				switch_set_state(&switch_sendend, pJack->send_end_status);
				input_report_key(pJack->input, KEYCODE_SENDEND, pJack->send_end_status);
				input_sync(pJack->input);
			}
			else if (pJack->volume_up_status)
			{
				/* Volume up released*/
				SEC_JACKDEV_DBG("Volume Up released.\n");
				pJack->volume_up_status = 0;
				switch_set_state(&volume_up, pJack->volume_up_status);
				input_report_key(pJack->input, KEY_VOLUMEUP, pJack->volume_up_status);
				input_sync(pJack->input);

			}
			else if (pJack->volume_down_status)
			{
				/* voluem down released*/
				SEC_JACKDEV_DBG("Volume down released.\n");
				pJack->volume_down_status = 0;		
				switch_set_state(&volume_down, pJack->volume_down_status);
				input_report_key(pJack->input, KEY_VOLUMEDOWN, pJack->volume_down_status);
				input_sync(pJack->input);
			}
		}
		else
		{
			if ( pJack->four_pole_25k_status )
			{
				SEC_JACKDEV_DBG("25kohm headset button.\n");
				if(adc2 < JACK_BUTTON_25K_SENDEND_ADC)
				{
				
					/* switch pressed */
					SEC_JACKDEV_DBG("SEND/END pressed.\n");
					pJack->send_end_status = 1;
					
					switch_set_state(&switch_sendend, pJack->send_end_status);
					input_report_key(pJack->input, KEYCODE_SENDEND, pJack->send_end_status);
					input_sync(pJack->input);
				
				}
				else if ((adc2 > JACK_BUTTON_25K_VOLUMEUP_ADC_LOW) && 
						(adc2 < JACK_BUTTON_25K_VOLUMEUP_ADC_HIGH))
				{
				
					/* Volume Up pressed */
					SEC_JACKDEV_DBG("VOLUME UP pressed.\n");
					pJack->volume_up_status = 1;
					
					switch_set_state(&volume_up, pJack->volume_up_status);
					input_report_key(pJack->input, KEY_VOLUMEUP, pJack->volume_up_status);
					input_sync(pJack->input);
				}
				else if ((adc2 > JACK_BUTTON_25K_VOUMEDOWN_ADC_LOW) && 
					(adc2 < JACK_BUTTON_25K_VOUMEDOWN_ADC_HIGH))
				{
					/* Volume Up pressed */
					SEC_JACKDEV_DBG("VOLUME DOWN pressed.\n");
					pJack->volume_down_status = 1;
					
					switch_set_state(&volume_down, pJack->volume_down_status);
					input_report_key(pJack->input, KEY_VOLUMEDOWN, pJack->volume_down_status);
					input_sync(pJack->input);
				}
				else
				{
					SEC_JACKDEV_DBG("ERROR!!!: headset detect : out of range adc\n");	
				}

			}
			else
			{
				if(adc2 < JACK_BUTTON_SENDEND_ADC)
				{
				
					/* switch pressed */
					SEC_JACKDEV_DBG("SEND/END pressed.\n");
					pJack->send_end_status = 1;
					
					switch_set_state(&switch_sendend, pJack->send_end_status);
					input_report_key(pJack->input, KEYCODE_SENDEND, pJack->send_end_status);
					input_sync(pJack->input);
				
				}
				else if (adc2 < JACK_BUTTON_VOLUMEUP_ADC)
				{
				
					/* Volume Up pressed */
					SEC_JACKDEV_DBG("VOLUME UP pressed.\n");
					pJack->volume_up_status = 1;
					
					switch_set_state(&volume_up, pJack->volume_up_status);
					input_report_key(pJack->input, KEY_VOLUMEUP, pJack->volume_up_status);
					input_sync(pJack->input);
				}
				else if (adc2 < JACK_BUTTON_VOUMEDOWN_ADC)
				{
					/* Volume Up pressed */
					SEC_JACKDEV_DBG("VOLUME DOWN pressed.\n");
					pJack->volume_down_status = 1;
					
					switch_set_state(&volume_down, pJack->volume_down_status);
					input_report_key(pJack->input, KEY_VOLUMEDOWN, pJack->volume_down_status);
					input_sync(pJack->input);
				}
				else
				{
					SEC_JACKDEV_DBG("ERROR!!!: headset detect : out of range adc\n");	
				}
			}
		}
				

	}

}

static void jack_work(struct work_struct *pWork)
{
	
	struct sec_jack_info* pJack = container_of(pWork, struct sec_jack_info, jack_detect_work);
	pJack->current_jack_type_status = jack_type_status( pJack );

	long jack_lock_flag = 0;

	SEC_JACKDEV_DBG("jack_work::headset detection\n");

	if ( pJack->current_jack_type_status == SEC_HEADSET_3_POLE_DEVICE)
	{
		SEC_JACKDEV_DBG("3 pole headset attached and turn off mic\n");

		mic_pwr( false, pJack );
	}
	else if ( pJack->current_jack_type_status == SEC_HEADSET_4_POLE_DEVICE )
	{
		SEC_JACKDEV_DBG("4 pole  headset attached and turn on mic\n");
	}
	else if ( pJack->current_jack_type_status == SEC_TVOUT_DEVICE )
	{
		SEC_JACKDEV_DBG("TV_out jack attached\n");

		mic_pwr( false, pJack );
	}
	else if ( pJack->current_jack_type_status == SEC_JACK_NO_DEVICE )
	{
		SEC_JACKDEV_DBG("JACK removed\n");

		spin_lock_irqsave(&jacklock, jack_lock_flag);
		pJack->jack_status = REMOVED;
		spin_unlock_irqrestore(&jacklock, jack_lock_flag);

		/* Jack removed */
		mic_pwr( false, pJack );

		SEC_JACKDEV_DBG("switch_set_state(%d, %d)\n", switch_jack_detection.state, pJack->current_jack_type_status);

		switch_set_state(&switch_jack_detection, pJack->current_jack_type_status);
		return;
	}
	else
	{
		SEC_JACKDEV_DBG("jack detected but unknown device\n");
	}

	switch_set_state(&switch_jack_detection, pJack->current_jack_type_status);
	jack_input_selector(pJack->current_jack_type_status);

	spin_lock_irqsave(&jacklock, jack_lock_flag);
	pJack->jack_status = INSERTED;
	spin_unlock_irqrestore(&jacklock, jack_lock_flag);
}

/*
	Turn on Mic Reference/ Jack ID supply - needed when using the regulator API as can not be used in 
	interrupt context.
*/
static void micref_work(struct work_struct *pWork)
{
	struct sec_jack_info* pJack = container_of(pWork, struct sec_jack_info, jack_micref_work);
	SEC_JACKDEV_DBG("micref_work\n");

	mic_pwr(true, pJack);
}

static void jack_initstate_timer_handler( unsigned long param )
{
	struct sec_jack_info *pJack = (struct sec_jack_info *)param;
	SEC_JACKDEV_DBG("jack_initstate_timer_handler\n");
	
	del_timer(&pJack->jack_initstate_timer);
	schedule_work(&pJack->jack_detect_work);
}

static void jack_detect_timer_handler( unsigned long param )
{
	struct sec_jack_info *pJack = (struct sec_jack_info *)param;
	SEC_JACKDEV_DBG("jack_detect_timer_handler\n");

	del_timer(&pJack->jack_detect_timer);
	schedule_work(&pJack->jack_detect_work);
}

/* Jack insert/remove detection */
static irqreturn_t acc1_det_RE_handler ( int irq, void *pData )
{
	struct sec_jack_info *pJack = pData;

	SEC_JACKDEV_DBG("acc1_det_RE_handler\n");

	if ( INSERTING == pJack->jack_status || REMOVING == pJack->jack_status )
	{
		mod_timer(&pJack->jack_detect_timer, jiffies + msecs_to_jiffies(DETECTION_CHECK_TIME));
	}
	else if ( INSERTED == pJack->jack_status )
	{
		long jack_lock_flag = 0;	

		spin_lock_irqsave(&jacklock, jack_lock_flag);
		pJack->jack_status = REMOVING;
		spin_unlock_irqrestore(&jacklock, jack_lock_flag);

		del_timer( &pJack->jack_detect_timer );
		pJack->jack_detect_timer.expires = jiffies + msecs_to_jiffies(DETECTION_CHECK_TIME);
		add_timer( &pJack->jack_detect_timer );

		SEC_JACKDEV_DBG("headset removing\n");
	}

	return IRQ_HANDLED;
}

static irqreturn_t acc1_det_FE_handler ( int irq, void *pData )
{
	struct sec_jack_info *pJack = pData;

	SEC_JACKDEV_DBG("acc1_det_FE_handler\n");

	/* Jack inserted */
	if ( pJack->jack_status == REMOVED )
	{
		long jack_lock_flag = 0;	

		del_timer( &pJack->jack_detect_timer );
		pJack->jack_detect_timer.expires = jiffies + msecs_to_jiffies(DETECTION_CHECK_TIME);
		add_timer(&pJack->jack_detect_timer);

		spin_lock_irqsave(&jacklock, jack_lock_flag);
		pJack->jack_status = INSERTING;
		spin_unlock_irqrestore(&jacklock, jack_lock_flag);

		/* Turn on Mic reference/Jack ID supply */
		schedule_work(&pJack->jack_micref_work);

		SEC_JACKDEV_DBG("headset inserting\n");
	}
	else if ( INSERTING == pJack->jack_status || REMOVING == pJack->jack_status )
	{
		mod_timer(&pJack->jack_detect_timer, jiffies + msecs_to_jiffies(DETECTION_CHECK_TIME));
	}

	return IRQ_HANDLED;
}

/* Mic switch detection handler */
static irqreturn_t acc2th1_det_RE_handler ( int irq, void *pData )
{
	struct sec_jack_info *pJack = pData;
	SEC_JACKDEV_DBG("acc2th1_det_RE_handler\n");

	if(pJack->current_jack_type_status == SEC_HEADSET_4_POLE_DEVICE)
	{
		schedule_work(&pJack->jack_button_work);
	}
	return IRQ_HANDLED;
}

static irqreturn_t acc2th1_det_FE_handler ( int irq, void *pData )
{
	struct sec_jack_info *pJack = pData;

	SEC_JACKDEV_DBG("acc2th1_det_FE_handler\n");

	if(pJack->current_jack_type_status == SEC_HEADSET_4_POLE_DEVICE)
	{
		schedule_work(&pJack->jack_button_work);
	}
	return IRQ_HANDLED;
}

/* detection handler */
static irqreturn_t acc2th2_det_RE_handler ( int irq, void *pData )
{
	SEC_JACKDEV_DBG("acc2th2_det_RE_handler\n");

	return IRQ_HANDLED;
}


static irqreturn_t acc2th2_det_FE_handler ( int irq, void *pData )
{
	SEC_JACKDEV_DBG("acc2th2_det_FE_handler\n");

	return IRQ_HANDLED;
}

static int jack_init( struct platform_device *pdev )
{
	struct sec_jack_info* pData = platform_get_drvdata( pdev );
	int ret;
	int gpio;
	int irq;

	SEC_JACKDEV_DBG("jack_init\n");
	pr_info("JACK_DRIVER Start %s\n",__func__);

	/*
	* Turn on Headset MIC Bias LDO - used by ACCDETECT2 to determine type
	* of headset inserted.
	*/
	pData->mic_status = false;
	mic_pwr(true, pData);

	/*
	Initialise threshold for ACCDETECT1 comparator
	and the debounce for all ACCDETECT comparators
	*/
	ret = abx500_set_register_interruptible(pData->dev, AB8500_ECI_AV_ACC,
					(u8)AB8500_ACC_DET_DB1_REG, (u8)ACCDET1TH_DB);
	if (ret < 0)
	{
		pr_err("%s: ab8500 write failed\n",__func__);
		goto fail;
	}

	/* Initialise threshold for ACCDETECT2 comparator1 and comparator2 */
	ret = abx500_set_register_interruptible(pData->dev, AB8500_ECI_AV_ACC,
					(u8)AB8500_ACC_DET_DB2_REG, (u8)ACCDET2TH1_TH2);
	if (ret < 0)
	{
		pr_err("%s: ab8500 write failed\n",__func__);
		goto fail;
	}

	/* Register handlers and enable interrupts */

	irq = platform_get_irq_byname(pdev, ID_ACCDET1_R);
	if (irq < 0) {
		dev_err(pData->dev, "failed to get platform irq-%d\n", irq);
		ret = irq;
		goto fail;
	}

	ret = request_threaded_irq(irq, NULL,
		acc1_det_RE_handler, 0, DRIVER_HEADSET, pData);
	if (ret < 0) {
		dev_err(pData->dev, "Failed to register interrupt\n");
		goto fail;
	}

	irq = platform_get_irq_byname(pdev, ID_ACCDET1_F);
	if (irq < 0) {
		dev_err(pData->dev, "failed to get platform irq-%d\n", irq);
		ret = irq;
		goto fail;
	}

	ret = request_threaded_irq(irq, NULL,
		acc1_det_FE_handler, 0, DRIVER_HEADSET, pData);
	if (ret < 0) {
		dev_err(pData->dev, "Failed to register interrupt\n");
		goto fail;
	}

	irq = platform_get_irq_byname(pdev, ID_ACCDET21_R);
	if (irq < 0) {
		dev_err(pData->dev, "failed to get platform irq-%d\n", irq);
		ret = irq;
		goto fail;
	}

	ret = request_threaded_irq(irq, NULL,
		acc2th1_det_RE_handler, 0, DRIVER_HEADSET, pData);
	if (ret < 0) {
		dev_err(pData->dev, "Failed to register interrupt\n");
		goto fail;
	}

	irq = platform_get_irq_byname(pdev, ID_ACCDET21_F);
	if (irq < 0) {
		dev_err(pData->dev, "failed to get platform irq-%d\n", irq);
		ret = irq;
		goto fail;
	}

	ret = request_threaded_irq(irq, NULL,
		acc2th1_det_FE_handler, 0, DRIVER_HEADSET, pData);
	if (ret < 0) {
		dev_err(pData->dev, "Failed to register interrupt\n");
		goto fail;
	}

	/* Enable the Comparator Ref LDO, ACCDETECT 1 and 2 comparators */
	ret = abx500_set_register_interruptible(pData->dev, AB8500_ECI_AV_ACC,
					(u8)AB8500_ACC_DET_CTRL_REG, 
					(u8)(ACCDET1_ENA | ACCDET_VREF_ENA | ACCDET21_ENA | ACCDET22_ENA) );
	if (ret < 0)
	{
		pr_err("%s: ab8500 write failed\n",__func__);
		goto fail;
	}

	return 0;

fail:

	return -1;

}


static void jack_input_selector(int jack_type_status)
{
	SEC_JACKDEV_DBG("jack_type_status = 0X%x", jack_type_status);

	switch(jack_type_status)
	{
		case SEC_HEADSET_3_POLE_DEVICE:
		case SEC_HEADSET_4_POLE_DEVICE:
		{
			break;
		}
		case SEC_TVOUT_DEVICE :
		{
			break;
		}
		case SEC_UNKNOWN_DEVICE:
		{
			printk(KERN_INFO "unknown jack device attached. User must select jack device type\n");
			break;
		}
		default :
			printk(KERN_ERR "wrong selector value\n");
			break;
	}
}

//USER can select jack type if driver can't check the jack type
static int strtoi(const char *buf)
{
	int ret;
	ret = buf[0]-48;
	return ret;
}
static ssize_t select_jack_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk(KERN_INFO "[JACK] %s : operate nothing\n", __FUNCTION__);

	SEC_JACKDEV_DBG("select_jack_show\n");
	return 0;
}
static ssize_t select_jack_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int value = 0;

	int state;

	SEC_JACKDEV_DBG("buf = %s", buf);
	SEC_JACKDEV_DBG("buf size = %d", sizeof(buf));
	SEC_JACKDEV_DBG("buf size = %d", strlen(buf));

	if ( !pJack )
		return 0;

	state = pJack->jack_status;

	if(state)
	{
		if(pJack->current_jack_type_status != SEC_UNKNOWN_DEVICE)
		{
			printk(KERN_ERR "user can't select jack device if current_jack_status isn't unknown status");
			return -1;
		}

		if(sizeof(buf)!=1)
		{
			printk(KERN_INFO "input error\n");
			printk(KERN_INFO "Must be stored ( 1,2,4)\n");
			return -1;
		}

		value = strtoi(buf);
		SEC_JACKDEV_DBG("User  selection : 0X%x", value);

		switch(value)
		{
			case SEC_HEADSET_3_POLE_DEVICE:
			{
				pJack->current_jack_type_status = SEC_HEADSET_3_POLE_DEVICE;

				switch_set_state(&switch_jack_detection, pJack->current_jack_type_status);
				jack_input_selector(pJack->current_jack_type_status);
				break;
			}
			case SEC_HEADSET_4_POLE_DEVICE:
			{
				// Enable Send End interrup.

				pJack->current_jack_type_status = SEC_HEADSET_4_POLE_DEVICE;

				switch_set_state(&switch_jack_detection, pJack->current_jack_type_status);
				jack_input_selector(pJack->current_jack_type_status);
				break;
			}
			case SEC_TVOUT_DEVICE:
			{
				pJack->current_jack_type_status = SEC_TVOUT_DEVICE;

				mic_pwr( false, pJack );

				switch_set_state(&switch_jack_detection, pJack->current_jack_type_status);
				jack_input_selector(pJack->current_jack_type_status);
				break;
			}
		}
	}
	else
	{
		printk(KERN_ALERT "Error : mic bias enable complete but headset detached!!\n");
		pJack->current_jack_type_status = SEC_JACK_NO_DEVICE;
		mic_pwr( false, pJack );
	}

	return size;
}
static DEVICE_ATTR(select_jack, S_IRUGO | S_IWUSR | S_IWGRP | S_IXOTH, select_jack_show, select_jack_store);

static int sec_jack_probe(struct platform_device *pdev)
{
	int ret;
	struct input_dev	   *input;
	struct ab8500* pAb8500 = dev_get_drvdata(pdev->dev.parent);
	struct sec_jack_info* pData;

	SEC_JACKDEV_DBG("sec_jack_probe\n");

	printk(KERN_INFO "SEC JACK: Registering jack driver\n");

	if ( pAb8500 == NULL )
	{
		pr_err("SEC_JACK: Can not access AB8500\n");
		return -ENODEV;
	}

	pData = kzalloc(sizeof(struct sec_jack_info), GFP_KERNEL);
	if (!pData)
		return -ENOMEM;

	pJack = pData;

	pData->dev = &pdev->dev;
	pData->pAb8500 = pAb8500;
	pData->current_jack_type_status = SEC_JACK_NO_DEVICE;

	pData->pRegIDMicRef = "v-amic1";

	mutex_init(&pData->regulator_lock);
	
	SEC_JACKDEV_DBG("system_rev = %d\n", system_rev);

	SEC_JACKDEV_DBG("MicRef = %s\n", pData->pRegIDMicRef);

	pData->pRegMicRef = regulator_get(NULL, pData->pRegIDMicRef);

	if (IS_ERR(pData->pRegMicRef)) {
		ret = PTR_ERR( pData->pRegMicRef );
		pr_err("failed to get v-amic2 LDO\n");
		return ret;
	}

	input = pData->input = input_allocate_device();
	if (!input)
	{
		ret = -ENOMEM;
		printk(KERN_ERR "SEC JACK: Failed to allocate input device.\n");
		goto err_request_input_dev;
	}

	input->name = DRIVER_HEADSET;
	set_bit(EV_SYN, input->evbit);
	set_bit(EV_KEY, input->evbit);
	set_bit(KEYCODE_SENDEND, input->keybit);
	set_bit(KEY_VOLUMEUP, input->keybit);
	set_bit(KEY_VOLUMEDOWN, input->keybit);

	ret = input_register_device(input);
	if (ret < 0)
	{
		printk(KERN_ERR "SEC JACK: Failed to register driver\n");
		goto err_register_input_dev;
	}

	SEC_JACKDEV_DBG("registering switch_sendend switch_dev sysfs sec_jack");

	ret = switch_dev_register(&switch_jack_detection);
	if (ret < 0)
	{
		printk(KERN_ERR "SEC JACK: Failed to register switch device\n");
		goto err_switch_jack_detection;
	}

	ret = switch_dev_register(&switch_sendend);
	if (ret < 0)
	{
		printk(KERN_ERR "SEC JACK: Failed to register switch sendend device\n");
		goto err_switch_sendend;
	}

	ret = switch_dev_register(&volume_up);
	if (ret < 0)
	{
		printk(KERN_ERR "SEC JACK: Failed to register volume up device\n");
		goto err_volume_up;
	}

	ret = switch_dev_register(&volume_down);
	if (ret < 0)
	{
		printk(KERN_ERR "SEC JACK: Failed to register volume down device\n");
		goto err_volume_down;
	}

	//Create JACK Device file in Sysfs
	jack_class = class_create(THIS_MODULE, "jack");
	if(IS_ERR(jack_class))
	{
		printk(KERN_ERR "Failed to create class(sec_jack)\n");
	}

	jack_selector_fs = device_create(jack_class, NULL, 0, NULL, "jack_selector");
	if (IS_ERR(jack_selector_fs))
		printk(KERN_ERR "Failed to create device(sec_jack)!= %ld\n", IS_ERR(jack_selector_fs));

	if (device_create_file(jack_selector_fs, &dev_attr_select_jack) < 0)
		printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_select_jack.attr.name);

	init_timer(&pData->jack_detect_timer);
	pData->jack_detect_timer.function = jack_detect_timer_handler;
	pData->jack_detect_timer.data = (unsigned long) pData;

	init_timer(&pData->jack_initstate_timer);
	pData->jack_initstate_timer.function = jack_initstate_timer_handler;
	pData->jack_initstate_timer.data = (unsigned long) pData;

	mutex_init(&pData->jack_mutex);
	
	INIT_WORK(&pData->jack_detect_work, jack_work);

	INIT_WORK(&pData->jack_micref_work, micref_work);

	INIT_WORK(&pData->jack_button_work, button_work);

	platform_set_drvdata( pdev, pData);

	/* Initialise board/machine specific items */
	if ( jack_init( pdev ) )
	{
		printk(KERN_ERR "Failed to Initialise!\n");
		goto err_init;
	}

	wake_lock_init(&pData->jack_sendend_wake_lock, WAKE_LOCK_SUSPEND, DRIVER_HEADSET);

	del_timer( &pData->jack_detect_timer );

	/*  
	*	Detect headset that may already be plugged in.
	*	Allow the Headset MIC LDO to turn on and power up associated circuitry
	*/
	pData->jack_initstate_timer.expires = jiffies + msecs_to_jiffies(INITIAL_CHECK_TIME);
	add_timer( &pData->jack_initstate_timer );

	/* Set output polarity to Gnd when VAMIC1 is disabled */
	abx500_set_register_interruptible(pData->dev, AB8500_REGU_CTRL1, (u8)0x84, (u8)0x1);

	return 0;

err_init:
	switch_dev_unregister(&switch_sendend);

err_switch_sendend:
	switch_dev_unregister(&switch_jack_detection);

err_volume_up:
	switch_dev_unregister(&volume_up);

err_volume_down:
	switch_dev_unregister(&volume_down);

err_switch_jack_detection:
	input_unregister_device(input);

err_register_input_dev:
	input_free_device(input);

err_request_input_dev:
	regulator_put( pData->pRegMicRef );

	pJack = NULL;

	kfree (pData);

	return ret;
}

static int sec_jack_remove(struct platform_device *pdev)
{
	struct sec_jack_info* pData = platform_get_drvdata( pdev );
	
	SEC_JACKDEV_DBG("sec_jack_remove\n");

	wake_lock_destroy(&pData->jack_sendend_wake_lock);

	del_timer( &pData->jack_detect_timer );
	del_timer( &pData->jack_initstate_timer );

	switch_dev_unregister(&switch_sendend);
	switch_dev_unregister(&switch_jack_detection);
	switch_dev_unregister(&volume_up);
	switch_dev_unregister(&volume_down);

	input_unregister_device(pData->input);
	input_free_device(pData->input);

	mic_pwr(false, pData);
	regulator_put( pData->pRegMicRef );

	pJack = NULL;
	kfree (pData);

	return 0;
}

static int __init sec_jack_init(void)
{
	SEC_JACKDEV_DBG("sec_jack_init");
	spin_lock_init(&jacklock);
	return platform_driver_register(&sec_jack_driver);
}

static void __exit sec_jack_exit(void)
{
	SEC_JACKDEV_DBG("sec_jack_exit");

	platform_driver_unregister(&sec_jack_driver);
}

static struct platform_driver sec_jack_driver = {
	.probe		= sec_jack_probe,
	.remove		= sec_jack_remove,
	.driver		= {
		.name		= DRIVER_HEADSET,
		.owner		= THIS_MODULE,
	},
};

module_init(sec_jack_init);
module_exit(sec_jack_exit);

MODULE_AUTHOR("Uk Kim <w0806.kim@samsung.com>");
MODULE_DESCRIPTION("SEC JACK detection driver");
MODULE_LICENSE("GPL");
