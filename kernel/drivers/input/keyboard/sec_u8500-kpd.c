/*
 * Overview:
 *	Keypad driver for u8500 platform
 *
 * Copyright (C) 2009 STMicroelectronics Pvt. Ltd.
 * Copyright (C) 2009 ST-Ericsson SA
 * Copyright (C) 2010 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

/* Keypad driver Version */
#define KEYPAD_VER_X		4
#define KEYPAD_VER_Y		0
#define KEYPAD_VER_Z		0

#include <asm/mach/irq.h>
#include <asm/irq.h>
#include <linux/ioport.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>

#include <linux/input.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/completion.h>
#include <linux/bitops.h>
#include <linux/list.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <mach/hardware.h>
/*#include <mach/debug.h>*/
#include <mach/kpd.h>
#include <linux/slab.h>

#ifdef CONFIG_KEYPAD_KESWICK_DEBUG
#define DEBUG_KP(x) pr_info x
#else
#define DEBUG_KP(x)
#endif

#define KEYPAD_NAME		"DRIVER KEYPAD"

extern struct driver_debug_st DBG_ST;
/*function declarations h/w independent*/
irqreturn_t keswick_kp_intrhandler(/*int irq, */void *dev_id);
static void keswick_kp_wq_kscan(struct work_struct *work);

/*
 * Module parameter defination to pass mode of operation
 * 0 = to initialize driver in Interrupt mode (default mode)
 * 1 = to Intialize driver in polling mode of operation
 */
int kpmode;
module_param(kpmode, int, 0);
MODULE_PARM_DESC(kpmode, "Keypad Operating mode (INT/POLL)=(0/1)");

/*static*/ DEFINE_MUTEX(keswick_keymap_mutex);
/*static*/ u8 keswick_keymap = 0; /* Default keymap is keymap 0 */

static ssize_t keswick_keymap_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", keswick_keymap);
}

static ssize_t keswick_keymap_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	int keymap;

	if (sscanf(buf, "%u", &keymap) != 1)
		return -EINVAL;

	if ((keymap != 0) && (keymap != 1))
		return -EINVAL;

	mutex_lock(&keswick_keymap_mutex);
	if (keymap != keswick_keymap)
		keswick_keymap = keymap;
	mutex_unlock(&keswick_keymap_mutex);

	return strnlen(buf, count);
}

static DEVICE_ATTR(keymap, S_IRUGO | S_IWUSR, keswick_keymap_show,  keswick_keymap_store);



/**
 * keswick_kp_wq_kdebounce - work queue to debounce keypad GPIO inputs
 * @work:	pointer to keypad data
 *
 * Executes on first keypress when keypad in idle state. When inputs are in steady state executes
 * the keypad scanning.
 */

static void keswick_kp_wq_kdebounce(struct work_struct *work)
{
	int input_cnt;
	int cnt;
	int key_stable;
	int key_cnt;
	struct gpio_list_t * pGpioList;
	int *pInput_state;
	struct keypad_t *kp = container_of((struct work_struct *)work,
					   struct keypad_t, kdebounce_work);

	pInput_state = kmalloc( sizeof(int) * kp->num_inputs, GFP_KERNEL);

	input_cnt = 0;

	/* Get Inital state */
	list_for_each_entry(pGpioList, &kp->gpio_in, list)
	{
		pInput_state[input_cnt++] = gpio_get_value(pGpioList->gpio & KEYPAD_GPIO_MASK);
	}

	/* Debounce count */
	cnt = 0;

	/* Scan keypad inputs until they reach a stable state for 3 debounce periods */
	do
	{
		msleep(KEYPAD_DEBOUNCE_PERIOD);

		key_stable = 1;
		input_cnt = 0;

		/* compare state */
		list_for_each_entry(pGpioList, &kp->gpio_in, list)
		{
			int value;

			value = gpio_get_value(pGpioList->gpio & KEYPAD_GPIO_MASK);

			if ( pInput_state[input_cnt] != value )
			{
				pInput_state[input_cnt] = value;

				key_stable = 0;
				cnt = 0;
			}

			input_cnt++;
		}
		cnt++;
	}while ( ! key_stable || cnt < 3 );

	// Process key press
	if (kp->board->autoscan_results)
	{
		key_cnt = kp->board->autoscan_results(kp);

		/* There should be a key pressed. */
		if ( 0 >= key_cnt )
		{
			clear_bit(KPINTR_LKBIT, &kp->lockbits);

			printk(KERN_ERR"Kdebounce: Keypad no key Error %d\n", cnt);
		}
		else
		{
			/* key is pressed, check for hold condition */
			DEBUG_KP(("Kdebounce:  Key pressed %d\n", cnt));

			/* Use single threaded work queue for processing work */
			queue_delayed_work( kp->wq ,&kp->kscan_work,
					      msecs_to_jiffies(KEYPAD_RELEASE_PERIOD) + 1);
		}
	}
	else
		printk(KERN_ERR"Kdebounce:Function not found\n");


	kfree(pInput_state);
}

/**
 * keswick_kp_wq_kscan - work queue for keypad scanning
 * @work:	pointer to keypad data
 *
 * Executes at each scan tick, execute the key press/release function,
 * Generates key press/release event message for input subsystem for valid key
 * events, enables keypad interrupts (for int mode)
 */

static void keswick_kp_wq_kscan(struct work_struct *work)
{
	int key_cnt = 0;
	struct keypad_t *kp = container_of((struct delayed_work *)work,
					   struct keypad_t, kscan_work);

	if (kp->board->autoscan_results)
	{
		key_cnt = kp->board->autoscan_results(kp);

		/* Enable interrupts when all keys have been released or error occured. Keypad enters idle state */
		if (0 >= key_cnt )
		{
			clear_bit(KPINTR_LKBIT, &kp->lockbits);

			DEBUG_KP(("Kscan: keypad idle\n"));
		}
		else
		{
			/*if key is pressed and hold condition */
			DEBUG_KP(("Kscan: pressed and hold\n"));

			/* Use single threaded work queue for processing work */
			queue_delayed_work( kp->wq ,&kp->kscan_work,
				msecs_to_jiffies(KEYPAD_RELEASE_PERIOD) + 1);
		}
	}
	else
		printk(KERN_ERR"Kscan: Function not found\n");
}


/**
 * keswick_kp_init_keypad - keypad parameter initialization
 * @kp:		pointer to keypad data
 *
 * Initializes Keybits to enable keyevents
 * Initializes Initial keypress status to default
 * Calls the keypad platform specific init function.
 */
int __init keswick_kp_init_keypad(struct keypad_t *kp)
{
	int row, column, err = 0;
	u8 *p_kcode = kp->board->kcode_tbl;

	for (row = 0; row < MAX_KPROW; row++) {
		for (column = 0; column < MAX_KPCOL; column++) {
			/*set keybits for the keycodes in use */
			set_bit(*p_kcode, kp->inp_dev->keybit);
			/*set key status to default value */
			kp->key_state[row][column] = KEYPAD_STATE_DEFAULT;
			p_kcode++;
		}
	}

	if (kp->board->init)
		err = kp->board->init(kp);

	return err;
}

#ifdef CONFIG_PM
/**
 * keswick_kp_suspend - suspend keypad
 * @pdev:       platform data
 * @state:	power down level
 */
int keswick_kp_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct keypad_t *kp = platform_get_drvdata(pdev);
	if (kpmode) {
		printk(KERN_INFO "Enabling interrupt \n");
		kp->board->irqen(kp);
	}
	if (!device_may_wakeup(&pdev->dev)) {
		printk(KERN_INFO "Disabling interrupt\n");
		if (kp->board->irqdis)
			kp->board->irqdis(kp);
		if (kp->board->irqdis_int)
			kp->board->irqdis_int(kp);
	}
	return 0;
}

/**
 * keswick_kp_resume - resumes keypad
 * @pdev:       platform data
 */

int keswick_kp_resume(struct platform_device *pdev)
{	struct keypad_t *kp = platform_get_drvdata(pdev);
	if (kpmode) {
		printk(KERN_INFO "Disabling interrupt\n");
		if (kp->board->irqdis)
			kp->board->irqdis(kp);
		if (kp->board->irqdis_int)
			kp->board->irqdis_int(kp);
	}
	if (!device_may_wakeup(&pdev->dev)) {
		printk(KERN_INFO "Enabling interrupt\n");
		kp->board->irqen(kp);
	}
	return 0;
}

#else
#define keswick_kp_suspend NULL
#define keswick_kp_resume NULL
#endif				/* CONFIG_PM */

/**
 * keswick_kp_res - keypad resource scan function
 * @pdev:	driver platform data
 * @kp:	keypad data
 * return error code.
 * Scans through GPIO resources and adds them to seperate list of inputs
 * and outputs. 
 */
static int keswick_kp_res( struct platform_device *pdev, struct keypad_t * kp )
{
	int i;
	int err = 0;
	struct resource * kp_res;
	int num_inputs = 0;
	
	int num_res;

	num_res = pdev->num_resources;

	for ( i= 0; i < num_res; i++)
	{
		int start;
		int end;

		kp_res = &pdev->resource[i];

		start =  kp_res->start & KEYPAD_GPIO_MASK;
		end  = kp_res->end & KEYPAD_GPIO_MASK;

		if (IORESOURCE_IO == resource_type(kp_res) )
		{
			if ( KEYPAD_IO_OUTPUT == (kp_res->start & KEYPAD_IO_OUTPUT ) )
			{
				// Single entry
				if ( start == end )
				{
					struct gpio_list_t * pList = kzalloc( sizeof(struct gpio_list_t), GFP_KERNEL);
					if (!pList)
					{
						err = -ENOMEM;
						goto err_kzalloc;
					}

					// All outputs are part of the key matrix
					pList->gpio =start | KEYPAD_IO_MATRIX;

					list_add_tail(&pList->list, &kp->gpio_out);
				}
				// Group entry
				else if ( start <  end )
				{
					for ( ; start < end ; start++)
					{
						struct gpio_list_t * pList = kzalloc( sizeof(struct gpio_list_t), GFP_KERNEL);
						if (!pList)
						{
							err = -ENOMEM;
							goto err_kzalloc;
						}
						
						// All outputs are part of the key matrix
						pList->gpio =start | KEYPAD_IO_MATRIX;
						
						list_add_tail(&pList->list, &kp->gpio_out);
					}
				}
			}
			else if ( KEYPAD_IO_INPUT == (kp_res->start & KEYPAD_IO_INPUT ) )
			{
				// Single entry
				if ( start == end )
				{
					struct gpio_list_t * pList = kzalloc( sizeof(struct gpio_list_t), GFP_KERNEL);
					if (!pList)
					{
						err = -ENOMEM;
						goto err_kzalloc;
					}

					pList->gpio = start;

					list_add_tail(&pList->list, &kp->gpio_in);

					num_inputs++;

					if ( KEYPAD_IO_SWITCH ==  (kp_res->start & KEYPAD_IO_SWITCH ) )
					{
						pList->gpio |= KEYPAD_IO_SWITCH;
					}
					else
					{
						// This  input is part of the key matrix
						pList->gpio |= KEYPAD_IO_MATRIX;
					}
				}
				// Group entry
				else if ( start <  end )
				{
					for ( ; start < end ; start++)
					{
						struct gpio_list_t * pList = kzalloc( sizeof(struct gpio_list_t), GFP_KERNEL);
						if (!pList)
						{
							err = -ENOMEM;
							goto err_kzalloc;
						}
						
						pList->gpio = start;
						
						list_add_tail(&pList->list, &kp->gpio_in);

						num_inputs++;

						if ( KEYPAD_IO_SWITCH ==  (kp_res->start & KEYPAD_IO_SWITCH ) )
						{
							pList->gpio |= KEYPAD_IO_SWITCH;
						}
						else
						{
							// This  input is part of the key matrix
							pList->gpio |= KEYPAD_IO_MATRIX;
						}
					}
				}
			}
		}
	}

	kp->num_inputs = num_inputs;

	if ( list_empty (&kp->gpio_out) || list_empty (&kp->gpio_in))
	{
		printk(KERN_ERR "keypad gpios incorrectly defined");
		err = -1;
	}

err_kzalloc:
	
	return err;

}

/**
 * keswick_kp_probe - keypad module probe function
 * @pdev:	driver platform data
 *
 * Allocates data memory, registers the module with input subsystem,
 * initializes keypad default condition, initializes keypad interrupt handler
 * for interrupt mode operation, initializes keypad work queues functions for
 * polling mode operation
 */
static int keswick_kp_probe(struct platform_device *pdev)
{
	struct keypad_t *kp;
	int err = 0;
	struct keypad_device *keypad_board;
	int num_res;

	printk(KERN_INFO "\nkeypad probe called");

	if (!pdev)
		return -EINVAL;

	keypad_board = pdev->dev.platform_data;

	kp = kzalloc(sizeof(struct keypad_t), GFP_KERNEL);
	if (!kp) {
		err = -ENOMEM;
		goto err_kzalloc;
	}

	platform_set_drvdata(pdev, kp);
	kp = platform_get_drvdata(pdev);

	num_res = pdev->num_resources;

	if ( 0 == num_res )
	{
		printk(KERN_ERR "keypad gpio not defined");
		err = -1;
		goto err_board;
	}

	INIT_LIST_HEAD(&kp->gpio_out);
	INIT_LIST_HEAD(&kp->gpio_in);

	/* Scan through list of keypad's GPIO resources and place into seperate lists of inputs and outputs */
	err = keswick_kp_res( pdev, kp );

	if ( err )
	{
		goto err_board;
	}

	printk(KERN_INFO "\nkp_probe 1");

	if (!keypad_board) {
		printk(KERN_ERR "keypad platform data not defined");
		err = -1;
		goto err_board;
	}

	printk(KERN_INFO "\nkp_probe 2");

	kp->board = keypad_board;
	kp->mode = kpmode;

	kp->inp_dev = input_allocate_device();
	if (!kp->inp_dev) {
		printk(KERN_ERR "Could not allocate memory for the device");
		err = -1;
		goto err_inp_devalloc;
	}

	printk(KERN_INFO "\nkp_probe 3");

	kp->inp_dev->evbit[0] = BIT(EV_KEY);
	kp->inp_dev->name = pdev->name;
	kp->inp_dev->phys = "stkpd/input0";

	kp->inp_dev->id.product = KEYPAD_VER_X;
	kp->inp_dev->id.version = KEYPAD_VER_Y * 0x100 + KEYPAD_VER_Z;
	/* kp->inp_dev->private = kp; */

	clear_bit(KPINTR_LKBIT, &kp->lockbits);

	printk(KERN_INFO "\nkp_probe 4");

	err = device_create_file(&pdev->dev, &dev_attr_keymap);
	if (err < 0) {
		printk(KERN_ERR "\nCould not create sysfs file for keypad");
		goto err_create_sysfs_entry;
	}

	if (input_register_device(kp->inp_dev) < 0) {
		printk(KERN_ERR "Could not register input device");
		err = -1;
		goto err_inp_reg;
	} else {
		printk(KERN_INFO "Registered keypad module with input subsystem");
	}
	printk(KERN_INFO "\nkp_probe 5");

	/* Initialize keypad workques  */
	kp->wq = create_singlethread_workqueue("keypad");
	INIT_DELAYED_WORK(&kp->kscan_work, keswick_kp_wq_kscan);
	INIT_WORK(&kp->kdebounce_work, keswick_kp_wq_kdebounce);

	/* Enable wakeup from keypad */
	device_init_wakeup(&pdev->dev, kp->board->enable_wakeup);
	device_set_wakeup_capable(&pdev->dev, kp->board->enable_wakeup);

	err = keswick_kp_init_keypad(kp);
	if (err)
		goto err_inp_reg;

	printk(KERN_INFO "Module initialized Ver(%d.%d.%d)",
		 KEYPAD_VER_X, KEYPAD_VER_Y, KEYPAD_VER_Z);
	return 0;

err_inp_reg:
	/* unregistering device */
	input_unregister_device(kp->inp_dev);

err_create_sysfs_entry:
	device_remove_file(&pdev->dev, &dev_attr_keymap);

err_inp_devalloc:
	input_free_device(kp->inp_dev);

err_board:
	kfree(kp);

err_kzalloc:
	return err;
}

/**
 * keswick_kp_remove - keypad module remove function
 *
 * @pdev:	driver platform data
 *
 * Disables Keypad interrupt if any, frees allocated keypad interrupt if any,
 * cancels keypad work queues if any, deallocate used GPIO pin, unregisters the
 * module, frees used memory
 */
static int keswick_kp_remove(struct platform_device *pdev)
{
	struct keypad_t *kp = platform_get_drvdata(pdev);

	struct gpio_list_t * pGpioList;
	struct gpio_list_t * pTemp;

	/*Frees allocated keypad interrupt if any */
	if (kp->board->exit)
		kp->board->exit(kp);
#if 0
	if (!kp->mode)
		free_irq(kp->irq, kp);
#endif
	/* cancel and flush keypad work queues if any  */
	cancel_delayed_work(&kp->kscan_work);
	cancel_delayed_work_sync(&kp->kscan_work);

	/* Clean linked lists */
	list_for_each_entry_safe( pGpioList, pTemp, &kp->gpio_in, list)
	{
		list_del( kp->gpio_in.next );
		kfree( pGpioList );
	}

	list_for_each_entry_safe( pGpioList, pTemp, &kp->gpio_out, list)
	{
		list_del( kp->gpio_out.next );
		kfree( pGpioList );
	}

	device_remove_file(&pdev->dev, &dev_attr_keymap);
	/* block until work struct's callback terminates */
	input_unregister_device(kp->inp_dev);
	input_free_device(kp->inp_dev);
	kfree(kp);
	printk(KERN_INFO "Module removed....");
	return 0;
}

struct platform_driver keswickkpd_driver = {
	.probe = keswick_kp_probe,
	.remove = keswick_kp_remove,
	.driver = {
		   .name = DRIVER_KP,
		   },
	.suspend = keswick_kp_suspend,
	.resume = keswick_kp_resume,
};

static int keswick_kp_init(void)
{
	return platform_driver_register(&keswickkpd_driver);
}

static void keswick_kp_exit(void)
{
	platform_driver_unregister(&keswickkpd_driver);
}

module_init(keswick_kp_init);
module_exit(keswick_kp_exit);

MODULE_AUTHOR("ST-Ericsson");
MODULE_DESCRIPTION("keswick keyboard driver");
MODULE_LICENSE("GPL");
