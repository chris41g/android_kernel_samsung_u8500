/*
 * Copyright STMicroelectronics, 2009.
 * Copyright (C) 2009 ST-Ericsson SA
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License
 */

#ifndef __U8500_KPD_H
#define __U8500_KPD_H

#ifndef CONFIG_AUTOSCAN_ENABLED
#define CONFIG_AUTOSCAN_ENABLED 1
#endif
#include <mach/hardware.h>
#include <linux/clk.h>
#ifdef CONFIG_MACH_KESWICK
#define MAX_KPROW               3
#define MAX_KPCOL               3
#else
#define MAX_KPROW               8
#define MAX_KPCOL               8
#define MAX_KEYS		(MAX_KPROW * MAX_KPCOL)
#endif

/*keypad related Constants*/
#define KEYPAD_RELEASE_PERIOD   11	/*11 msec, repeate key scan time */
#define KEYPAD_DEBOUNCE_PERIOD   10	/*10 msec, key debounce time */
#define KEYPAD_SCAN_PERIOD      4	/*4msec for new keypress */
#define KEYPAD_STATE_DEFAULT    0
#define KEYPAD_STATE_PRESSED    1
#define KEYPAD_STATE_PRESSACK   2
#define KEYPAD_STATE_RELEASED   3
#define KEYPAD_STATE_RELEASEACK 2

#define KPINTR_LKBIT		0	/*bit used for interrupt locking */

struct keypad_t;

/**
 * struct keypad_device - Device data structure for platform specific data
 * @init:		pointer to keypad init function
 * @exit:		pointer to keypad deinitialisation function
 * @autoscan_check:	pointer to read autoscan status function
 *			currently
 * @autoscan_disable:	pointer to autoscan feature disable function,
 *			not used currently
 * @autoscan_results:	pointer to read autoscan results function
 * @autoscan_en:	pointer to enable autoscan feature function
 *			currently
 * @irqen:		pointer to enable irq function
 * @irqdis:		pointer to disable irq function
 * @kcode_tbl:		lookup table for keycodes
 * @krow:		mask for available rows, value is 0xFF
 * @kcol:		mask for available columns, value is 0xFF
 * @irqdis_int:		pointer to disable irq function, to be called from ISR
 * @debounce_period:	platform specific debounce time, can be fine tuned later
 * @irqtype:		type of interrupt
 * @irq:		irq no,
 * @int_status:		interrupt status
 * @int_line_behaviour:	dynamis interrupt line behaviour
 * @enable_wakeup:	specifies if keypad event can wake up system from sleep
 */
struct keypad_device {
	int (*init)(struct keypad_t *kp);
	int (*exit)(struct keypad_t *kp);
	int (*autoscan_check)(void);
	void (*autoscan_disable)(void);
	int (*autoscan_results)(struct keypad_t *kp);
	void (*autoscan_en)(void);
	int (*irqen)(struct keypad_t *kp);
	int (*irqdis)(struct keypad_t *kp);
	u8 *kcode_tbl;
	u8 krow;
	u8 kcol;
	int (*irqdis_int)(struct keypad_t *kp);
	u8 debounce_period;
	unsigned long irqtype;
	u8 irq;
	u8 int_status;
	u8 int_line_behaviour;
	bool enable_wakeup;
};

#ifdef CONFIG_MACH_SAMSUNG_U8500
#include <linux/list.h>

#define DRIVER_KP "sec-u8500-kp"


#define KEYPAD_GPIO_MASK 0x00000FFF
#define KEYPAD_IO_OUTPUT (1<<12)
#define KEYPAD_IO_INPUT (1<<13)
#define KEYPAD_IO_MATRIX (1<<14)
#define KEYPAD_IO_SWITCH (1<<15)

struct switch_matrix_t
{
	int gpio;
	int row;
	int col;
};

struct gpio_list_t
{
	struct list_head list;
	int gpio;
};

struct keypad_t {
	struct list_head gpio_out;
	struct list_head gpio_in;
	int mode;
	int key_state[MAX_KPROW][MAX_KPCOL];
	int num_inputs;
	unsigned long lockbits;
	struct input_dev *inp_dev;
	struct workqueue_struct *wq;
	struct delayed_work kscan_work;
	struct work_struct kdebounce_work;
	struct keypad_device *board;
};
#else
/**
 * struct keypad_t - keypad data structure used internally by keypad driver
 * @irq:		irq no
 * @mode:		0 for interrupt mode, 1 for polling mode
 * @ske_regs:		ske regsiters base address
 * @cr_lock: 		spinlock variable
 * @key_state:		array for saving keystates
 * @lockbits: 		used for synchronisation in ISR
 * @inp_dev:		pointer to input device object
 * @kscan_work:		work queue
 * @board: 		keypad platform device
 * @key_cnt: 		count the keys pressed
 * @clk: 		clock structure pointer
 */
struct keypad_t {
	int irq;
	int mode;
	void __iomem *ske_regs;
	int key_state[MAX_KPROW][MAX_KPCOL];
	unsigned long lockbits;
	spinlock_t cr_lock;
	struct input_dev *inp_dev;
	struct delayed_work kscan_work;
	struct keypad_device *board;
	int key_cnt;
	struct clk *clk;
};
#endif
/**
 * enum kp_int_status - enum for INTR status
 * @KP_INT_DISABLED:	interrupt disabled
 * @KP_INT_ENABLED:	interrupt enabled
 */
enum kp_int_status {
	KP_INT_DISABLED = 0,
	KP_INT_ENABLED,
};
/**
 *enum kp_int_line_behaviour - enum for kp_int_line dynamic status
 * @INT_LINE_NOTSET: 	IRQ not asserted
 * @INT_LINE_SET: 	IRQ asserted
 */
enum kp_int_line_behaviour {
	INT_LINE_NOTSET = 0,
	INT_LINE_SET,
};
#endif	/*__U8500_KPD_H*/
