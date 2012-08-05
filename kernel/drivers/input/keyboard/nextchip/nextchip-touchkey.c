#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <asm/gpio.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/earlysuspend.h>
#include <asm/io.h>

#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>

#include <mach/board-sec-u8500.h>

struct i2c_touchkey_driver {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct early_suspend early_suspend;
};
struct i2c_touchkey_driver *touchkey_driver = NULL;

extern unsigned char I2Cm_ReadBytes(unsigned char SlaveAdr, unsigned char *RxArray, 
									unsigned char SubAdr0, unsigned char SubAdr1, 
									unsigned char RxByteCount);

extern int touch_is_pressed; 

#if defined(CONFIG_MACH_JANICE)
static int touchkey_keycode[2] = { KEY_MENU, KEY_BACK };
#endif

struct work_struct touchkey_work;
struct workqueue_struct *touchkey_wq;

static void nextchip_early_suspend(struct early_suspend *h)
{
	disable_irq_nosync(TOUCHKEY_INT_JANICE_R0_0);
	gpio_direction_output(TSP_LDO_ON2_JANICE_R0_0, 0);
	gpio_direction_output(EN_LED_LDO_JANICE_R0_0, 0);
	gpio_direction_output(TSP_RST_JANICE_R0_0, 0);
}

static void nextchip_late_resume(struct early_suspend *h)
{
	gpio_direction_output(TSP_LDO_ON2_JANICE_R0_0, 1);
	gpio_direction_output(EN_LED_LDO_JANICE_R0_0, 1);
	mdelay(2);
	gpio_direction_output(TSP_RST_JANICE_R0_0, 1);
	mdelay(1000);
	enable_irq(TOUCHKEY_INT_JANICE_R0_0);
}

#ifdef CONFIG_SAMSUNG_PANIC_DISPLAY_I2C_PERIPHS
void nextchip_touchkey_panic_display(struct i2c_adapter *pAdap)
{
	unsigned char nKeyData;
	unsigned char firm_ver;
	int err = 0;
	int keycode= 0;
	int pressed=0;

	/*
	 * Check driver has been started.
	*/
	if ( !(touchkey_driver))
		return;

	/*
	 * If there is an associated LDO check to make sure it is powered, if
	 * not then we can exit as it wasn't powered when panic occurred.
	*/

	/*
	 * If pAdap is NULL then exit with message.
	*/
	if ( !pAdap ){
		pr_emerg("\n\n%s Passed NULL pointer!\n",__func__);
		
		return;
	}

	/*
	 * If pAdap->algo_data is not NULL then this driver is using HW I2C,
	 *  then change adapter to use GPIO I2C panic driver.
	 * NB!Will "probably" not work on systems with dedicated I2C pins.
	*/
	if ( pAdap->algo_data ){
//		touchkey_driver->client->adapter = pAdap;
	}
	else{
		/*
		 * Otherwise use panic safe SW I2C algo,
		*/
//		touchkey_driver->client->adapter->algo = pAdap->algo;
	}

	pr_emerg("\n\n[Display of Nextchip Touchkey registers]\n");

	err = I2Cm_ReadBytes(0xc0, &firm_ver, 0x81, 0xC1, 1);

	if(err){
		printk(KERN_CRIT "failed to read FW ver.\n");
		return;
	}

	printk(KERN_CRIT "\tFirmware version 0x%x\n", firm_ver);

	err = I2Cm_ReadBytes(0xC0, &nKeyData, 0x81, 0xC0, 1);

	if(err){
		printk(KERN_CRIT "failed to read Key data.\n");
		return;
	}

	printk(KERN_CRIT "\tKey data 0x%x\n", nKeyData);
}
#endif


void touchkey_work_func(struct work_struct *p)
{
	unsigned char nKeyData;
	int err = 0;
	int keycode= 0;
	int pressed=0;

	err = I2Cm_ReadBytes(0xC0, &nKeyData, 0x81, 0xC0, 1);

	if(err)
	{
		printk(KERN_INFO "[Touchkey] I2C Error. err=%d\n", err);
		return;
	}

	switch( nKeyData )
	{
	case 0x01:	//Left Button
		keycode = KEY_MENU;
		pressed = 1;
		printk(KERN_INFO "[Touchkey] KEY_MENU is pressed\n");
		break;
	case 0x09:	// Left Button release
		keycode = KEY_MENU;
		pressed = 0;
		printk(KERN_INFO "[Touchkey] KEY_MENU is releaseed\n");
		break;
	case 0x02:	//Right Button
		keycode = KEY_BACK;
		pressed = 1;
		printk(KERN_INFO "[Touchkey] KEY_BACK is pressed\n");
		break;
	case 0x0A:	// Right Button release
		keycode = KEY_BACK;
		pressed = 0;
		printk(KERN_INFO "[Touchkey] KEY_BACK is releaseed\n");
		break;
	}

	if(touch_is_pressed)
	{
		printk(KERN_DEBUG "[TouchKey]touchkey pressed but don't send event because touch is pressed. \n");
	}
	else
	{
		input_report_key(touchkey_driver->input_dev, keycode, pressed);
	}
	
	enable_irq(TOUCHKEY_INT_JANICE_R0_0);
}


static irqreturn_t touchkey_interrupt(int irq, void *dummy)
{
	disable_irq_nosync(TOUCHKEY_INT_JANICE_R0_0);
	queue_work(touchkey_wq, &touchkey_work);

	return IRQ_HANDLED;
}

static void init_hw(void)
{
	gpio_direction_input(TOUCHKEY_INT_JANICE_R0_0);
	gpio_direction_output(TSP_LDO_ON2_JANICE_R0_0, 1);
	gpio_direction_output(EN_LED_LDO_JANICE_R0_0, 1);
	gpio_direction_output(TSP_TEST_JANICE_R0_0, 0);
	mdelay(2);
	gpio_direction_output(TSP_RST_JANICE_R0_0, 1);

	mdelay(1000);
}


static int i2c_touchkey_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct input_dev *input_dev;
	int err = 0;
	int irq;
	unsigned char firm_ver = 0;

	printk(KERN_INFO "[Touchkey] %s\n", __func__);
	input_dev = input_allocate_device();

	if (!input_dev) {
		return -ENOMEM;
	}

	input_dev->name = "nextchip_touchkey";
	input_dev->phys = "nextchip_touchkey/input0";
	//input_dev->id.bustype = BUS_HOST;

	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(touchkey_keycode[0], input_dev->keybit);
	set_bit(touchkey_keycode[1], input_dev->keybit);

	err = input_register_device(input_dev);
	if (err) {
		printk(KERN_INFO "[Touchkey] err input_register_device. %d\n",
				err);
		input_free_device(input_dev);
		return err;
	}

	touchkey_driver = kzalloc(sizeof(struct i2c_touchkey_driver), GFP_KERNEL);
	if (touchkey_driver == NULL) {
		dev_err(dev, "failed to create our state\n");
		return -ENOMEM;
	}

	touchkey_driver->input_dev = input_dev;
	init_hw();	

	err = request_irq(client->irq, touchkey_interrupt, IRQF_TRIGGER_FALLING, "nextchip_touchkey",  NULL);

	if(err) {
		printk(KERN_ERR "[TouchKey] %s Can't allocate irq. err=%d ..\n", __func__, err);
		return -EBUSY;
	}

	touchkey_driver->early_suspend.suspend = nextchip_early_suspend;
	touchkey_driver->early_suspend.resume = nextchip_late_resume;
	register_early_suspend(&touchkey_driver->early_suspend);

	err = I2Cm_ReadBytes(0xc0, &firm_ver, 0x81, 0xC1, 1);

	if(err)
		printk(KERN_INFO "[Touchkey] I2C Error. err=%d\n", err);

	printk(KERN_INFO "[Touchkey] firmware version %x\n", firm_ver);

	return 0;
}

static const struct i2c_device_id nextchip_touchkey_id[] = {
	{"nextchip_touchkey", 0},
	{}
};


struct i2c_driver touchkey_i2c_driver = {
	.driver = {
		   .name = "nextchip_touchkey_driver",
		   },
	.id_table = nextchip_touchkey_id,
	.probe = i2c_touchkey_probe,
};


static int __init touchkey_init(void)
{
	int ret = 0;

	touchkey_wq = create_singlethread_workqueue("nextchip_touchkey_wq");
	if (!touchkey_wq) {
		return -ENOMEM;
	}

	INIT_WORK(&touchkey_work, touchkey_work_func);
	ret = i2c_add_driver(&touchkey_i2c_driver);

	if (ret) {
		printk(KERN_ERR"[TouchKey] Touch keypad registration failed, module not inserted.ret= %d\n", ret);
	}
	printk(KERN_INFO "[TouchKey] Nextchip Touch Key init\n");

	
	return ret;
}

static void __exit touchkey_exit(void)
{

	i2c_del_driver(&touchkey_i2c_driver);

	if (touchkey_wq) {
		destroy_workqueue(touchkey_wq);
	}
	
	printk(KERN_INFO "[TouchKey] Touch Key exit\n");
}

module_init(touchkey_init);
module_exit(touchkey_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("nextchip touch keypad");
