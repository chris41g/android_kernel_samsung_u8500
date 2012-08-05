/*
 * Copyright (C) 2009 ST-Ericsson SA
 * Copyright (C) 2010 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/amba/bus.h>
#include <linux/amba/pl022.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/i2s/i2s.h>
#include <linux/gpio.h>
#include <linux/input.h>

#include <linux/power_supply.h>
#include <linux/mfd/abx500.h>
#include <linux/mfd/ab8500.h>
#include <linux/ste_timed_vibra.h>
#include <sound/ux500_ab8500_ext.h>
#include <linux/mmio.h>
#include <linux/spi/stm_msp.h>
#include <linux/earlysuspend.h>
#include <linux/delay.h>
#include <linux/mfd/ab8500/ab8500-gpio.h>
#ifdef CONFIG_USB_ANDROID
#include <linux/usb/android_composite.h>
#endif
#include <linux/input/tma340_i9060_tsp.h>
#include <linux/gpio_keys.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci.h>

#include <asm/mach/arch.h>
#include <asm/mach/irq.h>
#include <mach/irqs-board-mop500.h>
#include <asm/mach/map.h>
#include <asm/mach-types.h>

#include <plat/pincfg.h>
#include <plat/ske.h>
#include <plat/i2c.h>
#include <plat/ste_dma40.h>

#include <mach/devices.h>
#include <mach/ab8500-accdet.h>
#include <mach/ste_audio_io.h>
#include <mach/kpd.h>
#include <mach/gp2a.h>
#include <linux/mpu.h>
#include <mach/mpu60x0.h>
#include <mach/rn5t592.h>
#include <mach/mmc.h>
#include <mach/setup.h>

#include <video/mcde_display.h>

#ifdef CONFIG_DB8500_MLOADER
#include <mach/mloader-dbx500.h>
#endif

#include <sound/ux500_ab8500_ext.h>

#include "devices-db8500.h"
#include "board-godin-regulators.h"
#include "regulator-u8500.h"
#include "pins-db8500.h"
#include "pins.h"

#include "board-mop500.h"	/* using some generic functions defined here */
#include "board-sec-bm.h"
#include <mach/board-sec-u8500.h>
#include <linux/mfd/ab8500/ab8500-gpadc.h>
#include <linux/usb_switcher.h>

#include <mach/sec_param.h>
#include <mach/sec_common.h>
#include <mach/sec_log_buf.h>

#ifdef CONFIG_USB_ANDROID
#define PUBLIC_ID_BACKUPRAM1 (U8500_BACKUPRAM1_BASE + 0x0FC0)
#define USB_SERIAL_NUMBER_LEN 31
#endif

#ifndef SSG_CAMERA_ENABLE
#define SSG_CAMERA_ENABLE
#endif

int jig_smd = 0;
EXPORT_SYMBOL(jig_smd);

unsigned int sec_debug_settings;

u32 config_uart_gpio_pin = 29;
u8 ux500_debug_uart_config = 2;

struct device *gps_dev = NULL;
EXPORT_SYMBOL(gps_dev);

#define GPIO_SET 1
#define GPIO_RESET 0

#ifdef CONFIG_ANDROID_RAM_CONSOLE

static struct resource ram_console_resource = {
	.name = "ram_console",
	.flags = IORESOURCE_MEM,
};

static struct platform_device ram_console_device = {
	.name = "ram_console",
	.id = -1,
	.num_resources = 0,
	.resource = &ram_console_resource,
};

static int __init ram_console_setup(char *p)
{
	resource_size_t ram_console_size = memparse(p, &p);

	if ((ram_console_size > 0) && (*p == '@')){
		ram_console_resource.start = memparse(p + 1, &p);
		ram_console_resource.end = ram_console_resource.start + ram_console_size - 1;
		ram_console_device.num_resources = 1;
	}

	return 1;
}	 

__setup("mem_ram_console=", ram_console_setup);

#endif // CONFIG_ANDROID_RAM_CONSOLE


#if defined(CONFIG_TOUCHSCREEN_TMA340)
static void godin_tma340_disable_i2c(int disable)
{
	struct ux500_pins *pins = ux500_pins_get("nmk-i2c.3");

	if (pins) {
		if (disable)
			ux500_pins_disable(pins);
		else
			ux500_pins_enable(pins);
	}
}

static struct tma340_tsp_platform_data gti9060_tma340_plat_data = {
	.ldo_gpio		= TSP_LDO_ON_GTI9060_R0_1,
	.on_off_delay	= 100,
	.watchdog		= 0,		/* 0 if no periodic checks on device */
	.flipxy 		= 0,		/* 0 if no flip, else invert X/Y inputs */
	.max_x			= 480,
	.max_y			= 800,
	.key_led_gpio		= EN_LED_LDO_GTI9060_R0_1,	/* GPIO to control touchkey backlight */
	.sda_gpio		= TSP_SDA_GTI9060_R0_1,
	.scl_gpio		= TSP_SCL_GTI9060_R0_1,
	//.disable_i2c		= godin_tma340_disable_i2c,
};

#endif

#if defined(CONFIG_MPU_SENSORS_MPU3050)

#define SENSOR_MPU_NAME "mpu3050"

static struct mpu3050_platform_data mpu_data = {
	.int_config  = 0x12,
	.orientation = { 
		-1, 0,  0, 
		0, -1,  0, 
		0,  0,  1
	},
	/* accel */
	.accel = {
		.get_slave_descr = get_accel_slave_descr,
		.adapt_num   = 2,
		.bus         = EXT_SLAVE_BUS_SECONDARY,
		.address     = 0x08,
		.orientation = {   
			0, -1,  0, 
			1,  0,  0, 
			0,  0,  1
		},
	},
	/* compass */
	.compass = {
		.get_slave_descr = get_compass_slave_descr,
		.adapt_num   = 4,
		.bus         = EXT_SLAVE_BUS_PRIMARY,
		.address     = 0x30,
		.orientation = {   
			1,  0,  0, 
			0,  1,  0, 
			0,  0,  1
		},
	},
};

static struct mpu3050_platform_data mpu_data_gti9060_r01 = {
	.int_config  = 0x12,
	.orientation = { 
		-1, 0,  0, 
		0, -1,  0, 
		0,  0,  1
	},
	/* accel */
	.accel = {
		.get_slave_descr = get_accel_slave_descr,
		.adapt_num   = 2,
		.bus         = EXT_SLAVE_BUS_SECONDARY,
		.address     = 0x08,
		.orientation = {   
			0, -1,  0, 
			-1,  0,  0, 
			0,  0,  -1
		},
	},
	/* compass */
	.compass = {
		.get_slave_descr = get_compass_slave_descr,
		.adapt_num   = 4,
		.bus         = EXT_SLAVE_BUS_PRIMARY,
		.address     = 0x30,
		.orientation = {   
			0,  1,  0, 
			-1,  0,  0, 
			0,  0,  1
		},
	},
};
#endif

#if defined(CONFIG_PROXIMITY_GP2A)

/* -------------------------------------------------------------------------
 * GP2A PROXIMITY SENSOR PLATFORM-SPECIFIC CODE AND DATA
 * ------------------------------------------------------------------------- */
static int __init gp2a_setup( void );

static struct gp2a_platform_data gp2a_plat_data __initdata = {
	.ps_vout_gpio = PS_VOUT_GTI9060_R0_1,
	.hw_setup = gp2a_setup,
	.alsout = ADC_AUX2,
};

static int __init gp2a_setup( void )
{
	int err;

	/* Configure the GPIO for the interrupt */
	err = gpio_request(gp2a_plat_data.ps_vout_gpio, "PS_VOUT");
	if (err < 0)
	{
		pr_err("PS_VOUT: failed to request GPIO %d,"
			" err %d\n", gp2a_plat_data.ps_vout_gpio, err);

		goto err1;
	}

	err = gpio_direction_input(gp2a_plat_data.ps_vout_gpio);
	if (err < 0)
	{
		pr_err("PS_VOUT: failed to configure input"
			" direction for GPIO %d, err %d\n",
			gp2a_plat_data.ps_vout_gpio, err);

		goto err2;
	}

	return 0;

err2:
	gpio_free(gp2a_plat_data.ps_vout_gpio);
err1:
	return err;
}

#endif

#if defined(CONFIG_REGULATOR_RN5T592)
struct rn5t592_platform_data rn5t592_plat_data = {
	.subpmu_pwron_gpio = SUBMPU_PWRON_GTI9060_R0_1,
};
#endif


#if defined(CONFIG_USB_SWITCHER)
static struct usb_switch fsa880_data =	{
		.name					=	"FSA880",
		.id	 				=	0x0	,
		.id_mask				=	0xff	,
		.control_register_default		=	0x05	,
		.control_register_inital_value  	=	0x1e	,
		.connection_changed_interrupt_gpio	=	95	,
		.charger_detect_gpio			=	0xffff 	, /*no charger detect gpio for this device*/
		.valid_device_register_1_bits		=	0x74	,
		.valid_device_register_2_bits		=	0x8F	,	
		.valid_registers			=	{0,1,1,1,1,0,0,1,0,0,1,1,0,0,0,0, 0, 0, 0, 1, 1  },
};
#endif

static struct i2c_board_info __initdata gti9060_r0_1_i2c0_devices[] = {
#if defined(CONFIG_PROXIMITY_GP2A)
	{
		/* GP2A proximity sensor */
		I2C_BOARD_INFO(GP2A_I2C_DEVICE_NAME, 0x44),
		.platform_data = &gp2a_plat_data,
	},
#endif
};

static struct i2c_board_info __initdata gti9060_r0_1_i2c1_devices[] = {
#if defined(CONFIG_USB_SWITCHER)
	{
		I2C_BOARD_INFO("musb", 0x25),
		.platform_data = &fsa880_data ,
		.irq = GPIO_TO_IRQ(JACK_NINT_GTI9060_R0_1),
	},
#endif
};

static struct i2c_board_info __initdata gti9060_r0_1_i2c2_devices[] = {
#if defined(CONFIG_MPU_SENSORS_MPU3050)
		{
			I2C_BOARD_INFO(MPU_NAME, DEFAULT_MPU_SLAVEADDR),
			.irq = GPIO_TO_IRQ(SENSOR_INT_GTI9060_R0_1),
			.platform_data = &mpu_data_gti9060_r01,
		},
#endif
};

static struct i2c_board_info __initdata gti9060_r0_1_i2c3_devices[] = {
#if defined(CONFIG_TOUCHSCREEN_TMA340)
	{
		I2C_BOARD_INFO(TMA340_I9060_TSP_NAME, 0x20),
		.irq		= GPIO_TO_IRQ(TSP_INT_GTI9060_R0_1),
		.platform_data = &gti9060_tma340_plat_data,
	},
#endif
};

static struct i2c_gpio_platform_data gti9060_gpio_i2c_data = {
	.sda_pin = SUBPMU_SDA_GTI9060_R0_1,
	.scl_pin = SUBPMU_SCL_GTI9060_R0_1,
	.udelay = 3,	/* closest to 400KHz */
};

static struct platform_device gti9060_gpio_i2c_pdata = {
	.name = "i2c-gpio",
	.id = 4,
	.dev = {
		.platform_data = &gti9060_gpio_i2c_data,
	},
};

static struct i2c_board_info __initdata gti9060_r0_1_gpio_i2c_devices[] = {
#if defined(CONFIG_REGULATOR_RN5T592)
	{
		/* RN5T592 power management IC for the cameras */
		I2C_BOARD_INFO("rn5t592", 0x32),
		.platform_data = &rn5t592_plat_data,
	},
#endif
#if defined(CONFIG_SENSORS_MMC328X)
	{
		/* Compass */
		I2C_BOARD_INFO("mmc328x", 0x30),
	},
#endif
};


#ifdef CONFIG_KEYBOARD_GPIO
struct gpio_keys_button gti9060_r0_1_gpio_keys[] = {
	{
	/* Configuration parameters */
	.code = KEY_VOLUMEUP,		/* input event code (KEY_*, SW_*) */
	.gpio = VOL_UP_GTI9060_R0_1,
	.active_low = 1,
	.desc = "volup_key",
	.type = EV_KEY,		/* input event type (EV_KEY, EV_SW) */
	.wakeup = 1,		/* configure the button as a wake-up source */
	.debounce_interval = 30,	/* debounce ticks interval in msecs */
	.can_disable = false,
	},
	{
	/* Configuration parameters */
	.code = KEY_VOLUMEDOWN,		/* input event code (KEY_*, SW_*) */
	.gpio = VOL_DOWN_GTI9060_R0_1,
	.active_low = 1,
	.desc = "voldown_key",
	.type = EV_KEY,		/* input event type (EV_KEY, EV_SW) */
	.wakeup = 1,		/* configure the button as a wake-up source */
	.debounce_interval = 30,	/* debounce ticks interval in msecs */
	.can_disable = false,
	},
};

struct gpio_keys_platform_data gti9060_r0_1_gpio_data = {
	.buttons = gti9060_r0_1_gpio_keys,
	.nbuttons = ARRAY_SIZE(gti9060_r0_1_gpio_keys),
};

struct platform_device gti9060_gpio_keys_device = {
	.name = "gpio-keys",
	.dev = {
		.platform_data = &gti9060_r0_1_gpio_data,
	},
};
#endif


#ifdef CONFIG_USB_ANDROID
/*
static char *usb_functions_adb[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
#ifdef CONFIG_USB_ANDROID_ACM
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	"usb_mass_storage",
#endif
#ifdef CONFIG_USB_ANDROID_ADB
	"adb",
#endif
#ifdef CONFIG_USB_ANDROID_ECM
	"cdc_ethernet",
#endif
};
*/

static char *usb_functions_ums[] = {
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
		"usb_mass_storage",
#endif
};

static char *usb_functions_rndis[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
};

static char *usb_functions_phonet[] = {
#ifdef CONFIG_USB_ANDROID_PHONET
	"phonet",
#endif
};

static char *usb_functions_ecm[] = {
#ifdef CONFIG_USB_ANDROID_ECM
	"cdc_ethernet",
#endif
};

#ifdef CONFIG_USB_ANDROID_SAMSUNG_COMPOSITE
/* soonyong.cho : Variables for samsung composite such as kies, mtp, ums, etc... */
/* kies mode */
static char *usb_functions_acm_mtp[] = {
	"acm",
	"mtp",
};

#ifdef CONFIG_USB_ANDROID_ECM // Temp !! will  be deleted 2011.04.12 
/*Temp debug mode */
static char *usb_functions_acm_ums_adb[] = {
#ifdef CONFIG_USB_ANDROID_ACM
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	"usb_mass_storage",
#endif
#ifdef CONFIG_USB_ANDROID_ADB
	"adb",
#endif
#ifdef CONFIG_USB_ANDROID_ECM
	"cdc_ethernet",
#endif
#if 0 //def CONFIG_USB_ANDROID_PHONET
        "phonet",
#endif
};
#else 
/* debug mode */
static char *usb_functions_acm_ums_adb[] = {
#ifdef CONFIG_USB_ANDROID_ACM
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	"usb_mass_storage",
#endif
#ifdef CONFIG_USB_ANDROID_ADB
	"adb",
#endif
#if 0//def CONFIG_USB_ANDROID_PHONET
        "phonet",
#endif
};
#endif 

/* mtp only mode */
static char *usb_functions_mtp[] = {
	"mtp",
};

#else /* android original composite*/
static char *usb_functions_ums_adb[] = {
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	"usb_mass_storage",
#endif
#ifdef CONFIG_USB_ANDROID_ADB
	"adb",
#endif
};
#endif

static char *usb_functions_all[] = {
#ifdef CONFIG_USB_ANDROID_SAMSUNG_COMPOSITE
/* soonyong.cho : Every function driver for samsung composite.
 *  		  Number of to enable function features have to be same as below.
 */
#ifdef CONFIG_USB_ANDROID_ACM
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	"usb_mass_storage",
#endif
#ifdef CONFIG_USB_ANDROID_ADB
	"adb",
#endif
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
#ifdef CONFIG_USB_ANDROID_ECM
	"cdc_ethernet",
#endif
#ifdef CONFIG_USB_ANDROID_SAMSUNG_MTP
	"mtp",
#endif
#ifdef CONFIG_USB_ANDROID_PHONET
	"phonet",
#endif
#else /* original */
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	"usb_mass_storage",
#endif
#ifdef CONFIG_USB_ANDROID_ADB
	"adb",
#endif
#ifdef CONFIG_USB_ANDROID_ACM
	"acm",
#endif
#endif /* CONFIG_USB_ANDROID_SAMSUNG_COMPOSITE */
};


static struct android_usb_product usb_products[] = {
#ifdef CONFIG_USB_ANDROID_SAMSUNG_COMPOSITE
	/* soonyong.cho : Please modify below value correctly if you customize composite */
#ifdef CONFIG_USB_ANDROID_SAMSUNG_ESCAPE /* USE DEVGURU HOST DRIVER */
	{
		.product_id	= SAMSUNG_DEBUG_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_acm_ums_adb),
		.functions	= usb_functions_acm_ums_adb,
		.bDeviceClass	= 0xEF,
		.bDeviceSubClass= 0x02,
		.bDeviceProtocol= 0x01,
		.s		= ANDROID_DEBUG_CONFIG_STRING,
		.mode		= USBSTATUS_ADB,
	},
	{
		.product_id	= SAMSUNG_KIES_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_acm_mtp),
		.functions	= usb_functions_acm_mtp,
		.bDeviceClass	= 0xEF,
		.bDeviceSubClass= 0x02,
		.bDeviceProtocol= 0x01,
		.s		= ANDROID_KIES_CONFIG_STRING,
		.mode		= USBSTATUS_SAMSUNG_KIES,
	},
	{
		.product_id	= SAMSUNG_UMS_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_ums),
		.functions	= usb_functions_ums,
		.bDeviceClass	= USB_CLASS_PER_INTERFACE,
		.bDeviceSubClass= 0,
		.bDeviceProtocol= 0,
		.s		= ANDROID_UMS_CONFIG_STRING,
		.mode		= USBSTATUS_UMS,
	},
	{
		.product_id = SAMSUNG_RNDIS_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis),
		.functions	= usb_functions_rndis,
#ifdef CONFIG_USB_ANDROID_SAMSUNG_RNDIS_WITH_MS_COMPOSITE
		.bDeviceClass	= 0xEF,
		.bDeviceSubClass= 0x02,
		.bDeviceProtocol= 0x01,
#else
#ifdef CONFIG_USB_ANDROID_RNDIS_WCEIS
		.bDeviceClass	= USB_CLASS_WIRELESS_CONTROLLER,
#else
		.bDeviceClass	= USB_CLASS_COMM,
#endif
		.bDeviceSubClass= 0,
		.bDeviceProtocol= 0,
#endif
		.s		= ANDROID_RNDIS_CONFIG_STRING,
		.mode		= USBSTATUS_VTP,
	},
#ifdef CONFIG_USB_ANDROID_PHONET
	{
		.product_id = SAMSUNG_PHONET_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_phonet),
		.functions	= usb_functions_phonet,
		.bDeviceClass	= USB_CLASS_COMM,
		.bDeviceSubClass= 0,
		.bDeviceProtocol= 0,
		.s		= ANDROID_PHONET_CONFIG_STRING,
		.mode		= USBSTATUS_PHONET,
	},
#endif
	{
		.product_id = SAMSUNG_MTP_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp),
		.functions	= usb_functions_mtp,
		.bDeviceClass	= USB_CLASS_PER_INTERFACE,
		.bDeviceSubClass= 0,
		.bDeviceProtocol= 0x01,
		.s		= ANDROID_MTP_CONFIG_STRING,
		.mode		= USBSTATUS_MTPONLY,
	},

	/*
	{
		.product_id	= 0x685d,
		.num_functions	= ARRAY_SIZE(usb_functions_adb),
		.functions	= usb_functions_adb,
	},

	*/
#else /* USE MCCI HOST DRIVER */
	{
		.product_id = SAMSUNG_DEBUG_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_acm_ums_adb),
		.functions	= usb_functions_acm_ums_adb,
		.bDeviceClass	= USB_CLASS_COMM,
		.bDeviceSubClass= 0,
		.bDeviceProtocol= 0,
		.s		= ANDROID_DEBUG_CONFIG_STRING,
		.mode		= USBSTATUS_ADB,
	},
	{
		.product_id	= SAMSUNG_KIES_PRODUCT_ID, /* change sequence */
		.num_functions	= ARRAY_SIZE(usb_functions_acm_mtp),
		.functions	= usb_functions_acm_mtp,
		.bDeviceClass	= USB_CLASS_COMM,
		.bDeviceSubClass= 0,
		.bDeviceProtocol= 0,
		.s		= ANDROID_KIES_CONFIG_STRING,
		.mode		= USBSTATUS_SAMSUNG_KIES,
	},
	{
		.product_id = SAMSUNG_UMS_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_ums),
		.functions	= usb_functions_ums,
		.bDeviceClass	= USB_CLASS_PER_INTERFACE,
		.bDeviceSubClass= 0,
		.bDeviceProtocol= 0,
		.s		= ANDROID_UMS_CONFIG_STRING,
		.mode		= USBSTATUS_UMS,
	},
	{
		.product_id = SAMSUNG_RNDIS_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis),
		.functions	= usb_functions_rndis,
#ifdef CONFIG_USB_ANDROID_RNDIS_WCEIS
		.bDeviceClass	= USB_CLASS_WIRELESS_CONTROLLER,
#else
		.bDeviceClass	= USB_CLASS_COMM,
#endif
		.bDeviceSubClass= 0,
		.bDeviceProtocol= 0,
		.s		= ANDROID_RNDIS_CONFIG_STRING,
		.mode		= USBSTATUS_VTP,
	},
	{
		.product_id = SAMSUNG_MTP_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp),
		.functions	= usb_functions_mtp,
		.bDeviceClass	= USB_CLASS_PER_INTERFACE,
		.bDeviceSubClass= 0,
		.bDeviceProtocol= 0x01,
		.s		= ANDROID_MTP_CONFIG_STRING,
		.mode		= USBSTATUS_MTPONLY,
	},
#endif 
#else  /* original android composite */
	{
		.product_id = ANDROID_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_ums),
		.functions	= usb_functions_ums,
	},
	{
		.product_id = ANDROID_ADB_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_ums_adb),
		.functions	= usb_functions_ums_adb,
	},
#endif 
};

static char android_usb_serial_num[USB_SERIAL_NUMBER_LEN] = "0123456789ABCDEF";

static struct android_usb_platform_data android_usb_pdata = {
#ifdef CONFIG_USB_ANDROID_SAMSUNG_COMPOSITE
	.vendor_id	= SAMSUNG_VENDOR_ID,
	.product_id	= SAMSUNG_DEBUG_PRODUCT_ID,
#else
	.vendor_id	= ANDROID_VENDOR_ID,
	.product_id = ANDROID_PRODUCT_ID,
#endif
	.version	= 0x0100,
#ifdef CONFIG_USB_ANDROID_SAMSUNG_COMPOSITE
	.product_name	= "SAMSUNG_Android",
	.manufacturer_name = "SAMSUNG",
#else
	.product_name	= "Android Phone",
	.manufacturer_name = "Android",
#endif
	.serial_number	= android_usb_serial_num,
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
	.platform_data = &android_usb_pdata,
	},
};

#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 2,
	.vendor		= "Samsung",
	.product	= "Android Phone",
	.release	= 0x0100,
};

static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &mass_storage_pdata,
	},
};
#endif /* CONFIG_USB_ANDROID_MASS_STORAGE */

#ifdef CONFIG_USB_ANDROID_ECM
static struct usb_ether_platform_data usb_ecm_pdata = {
	.ethaddr	= {0x02, 0x11, 0x22, 0x33, 0x44, 0x55},
	.vendorID	= 0x04e8,
	.vendorDescr = "Samsung",
};

struct platform_device usb_ecm_device = {
	.name	= "cdc_ethernet",
	.id	= -1,
	.dev	= {
		.platform_data = &usb_ecm_pdata,
	},
};
#endif /* CONFIG_USB_ANDROID_ECM */

#ifdef CONFIG_USB_ANDROID_RNDIS
static struct usb_ether_platform_data usb_rndis_pdata = {
	.ethaddr	= {0x01, 0x11, 0x22, 0x33, 0x44, 0x55},
	.vendorID	= SAMSUNG_VENDOR_ID,
	.vendorDescr = "Samsung",
};

struct platform_device usb_rndis_device = {
	.name = "rndis",
	.id = -1,
	.dev = {
		.platform_data = &usb_rndis_pdata,
	},
};
#endif /* CONFIG_USB_ANDROID_RNDIS */

#ifdef CONFIG_USB_ANDROID_PHONET
static struct usb_ether_platform_data usb_phonet_pdata = {
	.vendorID	= SAMSUNG_VENDOR_ID,
	.vendorDescr = "Samsung",
};

struct platform_device usb_phonet_device = {
	.name = "phonet",
	.id = -1,
	.dev = {
		.platform_data = &usb_phonet_pdata,
	},
};
#endif /* CONFIG_USB_ANDROID_PHONET */

#endif /* CONFIG_USB_ANDROID */

#define U8500_I2C_CONTROLLER(id, _slsu, _tft, _rft, clk, t_out, _sm) \
static struct nmk_i2c_controller godin_i2c##id##_data = { \
	/*				\
	 * slave data setup time, which is	\
	 * 250 ns,100ns,10ns which is 14,6,2	\
	 * respectively for a 48 Mhz	\
	 * i2c clock			\
	 */				\
	.slsu		= _slsu,	\
	/* Tx FIFO threshold */		\
	.tft		= _tft,		\
	/* Rx FIFO threshold */		\
	.rft		= _rft,		\
	/* std. mode operation */	\
	.clk_freq	= clk,		\
	/* Slave response timeout(ms) */\
	.timeout	= t_out,	\
	.sm		= _sm,		\
}

/*
 * The board uses 4 i2c controllers, initialize all of
 * them with slave data setup time of 250 ns,
 * Tx & Rx FIFO threshold values as 1 and standard
 * mode of operation
 */
U8500_I2C_CONTROLLER(0, 0xe, 1, 8, 400000, 200, I2C_FREQ_MODE_FAST);
U8500_I2C_CONTROLLER(1, 0xe, 1, 8, 400000, 200, I2C_FREQ_MODE_FAST);
U8500_I2C_CONTROLLER(2, 0xe, 1, 8, 400000, 200, I2C_FREQ_MODE_FAST);
U8500_I2C_CONTROLLER(3, 0xe, 1, 8, 400000, 200, I2C_FREQ_MODE_FAST);


/**
 * godin_gpio_wq_kdebounce - work queue to debounce GPIO inputs.
 * @work:	pointer to gpio data
 *
 * Waits until GPIOI input are in steady state.
 */
#define DEBOUNCE_PERIOD		10	/* scan period in ms */
#define DEBOUNCE_CYCLES		6	/* Number of periods to check for bounce */

void godin_gpio_wq_debounce(struct work_struct *work)
{
	int cnt;
	int input_state;
	struct gpio_debounce_t *pDb = container_of((struct work_struct *)work,
					   struct gpio_debounce_t, debounce_work);

	input_state = gpio_get_value( pDb->gpio );

	/* Debounce count */
	cnt = 0;

	/* Scan gpio input until it reachs a stable state for DEBOUNCE_CYCLES debounce periods */
	do
	{
		int value;

		msleep(DEBOUNCE_PERIOD);

		value = gpio_get_value( pDb->gpio );

		if ( input_state != value )
		{
			input_state = value;

			cnt = 0;
		}

		cnt++;
	}while ( cnt < DEBOUNCE_CYCLES );

	// Process action
	if ( pDb->pWork )
	{
		if ( pDb->wq )
		{
			queue_work(pDb->wq, pDb->pWork);
		}
		else
		{
			schedule_work( pDb->pWork );
		}
	}
	else if ( pDb->pFn )
	{
		pDb->pFn();

	}
	else
	{
		printk(KERN_ERR"Debounce: Action function not found\n");
	}

	if ( pDb->irq_disabled )
	{
		enable_irq( gpio_to_irq( pDb->gpio ) );
	}

}

/*
 * MSP-SPI
 */

#define NUM_MSP_CLIENTS 10
static struct stm_msp_controller godin_msp2_spi_data = {
	.id		= MSP_2_CONTROLLER,
	.num_chipselect	= NUM_MSP_CLIENTS,
	.base_addr	= U8500_MSP2_BASE,
	.device_name	= "msp2",
};

/*
 * SSP
 */
#define NUM_SPI_CLIENTS 1
static struct pl022_ssp_controller godin_spi0_data = {
	.bus_id		= SPI023_0_CONTROLLER,
	.num_chipselect	= NUM_SPI_CLIENTS,
};

#ifdef CONFIG_SPI_PL022

static void godin_cs_control(u32 control)
{
	gpio_set_value(LCD_CSX_GTI9060_R0_1,control);
}

static struct pl022_config_chip godin_spi0_controller = {
	.iface = SSP_INTERFACE_MOTOROLA_SPI,
	.hierarchy = SSP_MASTER,
	.slave_tx_disable = true,
	.clk_freq = {
		.cpsdvsr = 2,
		.scr = 100,
	},
	.com_mode = POLLING_TRANSFER, /* avoid interrupts */
	.cs_control = godin_cs_control,
};

static struct spi_board_info spi_board_info[] __initdata = { 
	{ 
		.modalias = "pri_lcd_spi", 
		.controller_data = &godin_spi0_controller,
		.max_speed_hz = 12000000,
		.bus_num = SPI023_0_CONTROLLER,
		.chip_select = 0,
		.mode = SPI_MODE_0,
		.irq = IRQ_DB8500_SPI0,
        }, 
}; 
#else

#define LCD_BUS_NUM     2

static struct spi_board_info spi_board_info[] __initdata = {
        {
		.modalias = "pri_lcd_spi", 
                .max_speed_hz   = 1200000,
                .bus_num        = SPI023_0_CONTROLLER,
                .chip_select    = 0,
                .mode           = SPI_MODE_3,
                .controller_data = (void *)LCD_CSX_GTI9060_R0_1, //LCD_CS
        },
};

static struct spi_gpio_platform_data godin_spi_gpio_data = {
        .sck    = LCD_CLK_GTI9060_R0_1,//LCD_CLK
        .mosi   = LCD_SDI_GTI9060_R0_1,//LCD_SDI
        .num_chipselect = 2,
};


static struct platform_device ux500_spi_gpio_device = {
        .name   = "spi_gpio",
        .id     = SPI023_0_CONTROLLER,
        .dev    = {
                .platform_data  = &godin_spi_gpio_data,
        },
};
#endif

#ifdef CONFIG_AB8500_GPIO
static struct ab8500_gpio_platform_data ab8500_gpio_pdata = {
	.gpio_base		= AB8500_PIN_GPIO1,
	.irq_base		= MOP500_AB8500_VIR_GPIO_IRQ_BASE,
	/* initial_pin_config is the initial configuration of ab8500 pins.
	 * The pins can be configured as GPIO or alt functions based
	 * on value present in GpioSel1 to GpioSel6 and AlternatFunction
	 * register. This is the array of 7 configuration settings.
	 * One has to compile time decide these settings. Below is the
	 * explaination of these setting
	 * GpioSel1 = 0xFF => 1-4 should be GPIO input, 5 Res,
	 *		      6-8 should be GPIO input
	 * GpioSel2 = 0xF1 => 9-13 GND, 14-16 NC. 9 is PD GND.
	 * GpioSel3 = 0x80 => Pin GPIO24 is configured as GPIO
	 * GpioSel4 = 0x75 => 25 is SYSCLKREQ8, but NC, should be GPIO
	 * GpioSel5 = 0x7A => Pins GPIO34, GPIO36 to GPIO39 are conf as GPIO
	 * GpioSel6 = 0x02 => 42 is NC
	 * AlternaFunction = 0x03 => If Pins GPIO10 to 13 are not configured
	 * as GPIO then this register selects the alternate fucntions
	 */
	.initial_pin_config	= {0xFF, 0xF1, 0x80, 0x75, 0x7A, 0x02, 0x03},

	/* initial_pin_direction allows for the initial GPIO direction to
	 * be set.
	 */
	.initial_pin_direction		= {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

	/*
	 * initial_pin_pullups allows for the intial configuration of the
	 * GPIO pullup/pulldown configuration.
	 */
	.initial_pin_pullups		= {0xE0, 0x1F, 0x00, 0x00, 0x80, 0x00},
};
#endif


static struct regulator_init_data *u8500_regulators[U8500_NUM_REGULATORS] = {
	[U8500_REGULATOR_VAPE]			= &godin_u8500_vape_regulator,
	[U8500_REGULATOR_VARM]			= &godin_u8500_varm_regulator,
	[U8500_REGULATOR_VMODEM]		= &godin_u8500_vmodem_regulator,
	[U8500_REGULATOR_VPLL]			= &godin_u8500_vpll_regulator,
	[U8500_REGULATOR_VSMPS1]		= &godin_u8500_vsmps1_regulator,
	[U8500_REGULATOR_VSMPS2]		= &godin_u8500_vsmps2_regulator,
	[U8500_REGULATOR_VSMPS3]		= &godin_u8500_vsmps3_regulator,
	[U8500_REGULATOR_VRF1]			= &godin_u8500_vrf1_regulator,
	[U8500_REGULATOR_SWITCH_SVAMMDSP]	= &godin_u8500_svammdsp_regulator,
	[U8500_REGULATOR_SWITCH_SVAMMDSPRET]	= &godin_u8500_svammdspret_regulator,
	[U8500_REGULATOR_SWITCH_SVAPIPE]	= &godin_u8500_svapipe_regulator,
	[U8500_REGULATOR_SWITCH_SIAMMDSP]	= &godin_u8500_siammdsp_regulator,
	[U8500_REGULATOR_SWITCH_SIAMMDSPRET]	= &godin_u8500_siammdspret_regulator,
	[U8500_REGULATOR_SWITCH_SIAPIPE]	= &godin_u8500_siapipe_regulator,
	[U8500_REGULATOR_SWITCH_SGA]		= &godin_u8500_sga_regulator,
	[U8500_REGULATOR_SWITCH_B2R2_MCDE]	= &godin_u8500_b2r2_mcde_regulator,
	[U8500_REGULATOR_SWITCH_ESRAM12]	= &godin_u8500_esram12_regulator,
	[U8500_REGULATOR_SWITCH_ESRAM12RET]	= &godin_u8500_esram12ret_regulator,
	[U8500_REGULATOR_SWITCH_ESRAM34]	= &godin_u8500_esram34_regulator,
	[U8500_REGULATOR_SWITCH_ESRAM34RET]	= &godin_u8500_esram34ret_regulator,
};

static struct platform_device u8500_regulator_dev = {
	.name = "u8500-regulators",
	.id   = 0,
	.dev  = {
		.platform_data = u8500_regulators,
	},
};

static struct ab8500_audio_platform_data ab8500_audio_plat_data = {
	.ste_gpio_altf_init = msp13_i2s_init,
	.ste_gpio_altf_exit = msp13_i2s_exit,
};


/*
 * NOTE! The regulator configuration below must be in exactly the same order as
 * the regulator description in the driver, see drivers/regulator/ab8500.c
 */
static struct ab8500_platform_data ab8500_platdata = {
	.irq_base = MOP500_AB8500_IRQ_BASE,
	.pm_power_off = true,
	.regulator_reg_init	= godin_ab8500_regulator_reg_init,
	.num_regulator_reg_init	= ARRAY_SIZE(godin_ab8500_regulator_reg_init),
	.regulator		= godin_regulators,
	.num_regulator		= ARRAY_SIZE(godin_regulators),
#ifdef CONFIG_AB8500_DENC
	.denc = &ab8500_denc_pdata,
#endif
	.audio = &ab8500_audio_plat_data,
	.battery = &ab8500_bm_data,
	.charger = &ab8500_charger_plat_data,
	.btemp = &ab8500_btemp_plat_data,
	.fg = &ab8500_fg_plat_data,
	.chargalg = &ab8500_chargalg_plat_data,
#ifdef CONFIG_AB8500_GPIO
	.gpio = &ab8500_gpio_pdata,
#endif
#ifdef CONFIG_INPUT_AB8500_ACCDET
       .accdet = &ab8500_accdet_pdata,
#endif
};

static struct resource ab8500_resources[] = {
	[0] = {
		.start = IRQ_DB8500_AB8500,
		.end = IRQ_DB8500_AB8500,
		.flags = IORESOURCE_IRQ
	}
};

static struct platform_device ux500_ab8500_device = {
	.name = "ab8500-i2c",
	.id = 0,
	.dev = {
		.platform_data = &ab8500_platdata,
	},
	.num_resources = 1,
	.resource = ab8500_resources,
};

static struct platform_device *godin_platform_devices[] __initdata = {
	/*TODO - add platform devices here */

};
/* Force feedback vibrator device */
static struct platform_device ste_ff_vibra_device = {
	.name = "ste_ff_vibra"
};

/* For details check ste_timed_vibra docbook */
static struct ste_timed_vibra_platform_data ste_timed_vibra_plat_data = {
	.is_linear_vibra = false,
	.boost_level    = 100,
	.boost_time     = 60,
	.on_level       = 50,
	.off_level      = 50,
	.off_time       = 60,
	.timed_vibra_control = ux500_ab8500_audio_pwm_vibra,
};

/* Timed output vibrator device */
static struct platform_device ste_timed_output_vibra_device = {
	.name = "ste_timed_output_vibra",
	.dev = {
		.platform_data = &ste_timed_vibra_plat_data,
	},
};

static struct platform_device sec_device_rfkill = {
	.name = "bt_rfkill",
	.id = -1,
};

#define PRCC_K_SOFTRST_SET	0x18
#define PRCC_K_SOFTRST_CLEAR	0x1C
static struct ux500_pins *uart0_pins;
static void ux500_pl011_reset(void)
{
	void __iomem *prcc_rst_set, *prcc_rst_clr;

	prcc_rst_set = (void __iomem *)IO_ADDRESS(U8500_CLKRST1_BASE +
			PRCC_K_SOFTRST_SET);
	prcc_rst_clr = (void __iomem *)IO_ADDRESS(U8500_CLKRST1_BASE +
			PRCC_K_SOFTRST_CLEAR);

	/* Activate soft reset PRCC_K_SOFTRST_CLEAR */
	writel((readl(prcc_rst_clr) | 0x1), prcc_rst_clr);
	udelay(1);

	/* Release soft reset PRCC_K_SOFTRST_SET */
	writel((readl(prcc_rst_set) | 0x1), prcc_rst_set);
	udelay(1);
}

static void ux500_pl011_init(void)
{
	int ret;

	if (!uart0_pins) {
		uart0_pins = ux500_pins_get("uart0");
		if (!uart0_pins) {
			pr_err("pl011: uart0 pins_get failed\n");
			return;
		}
	}

	ret = ux500_pins_enable(uart0_pins);
	if (ret)
		pr_err("pl011: uart0 pins_enable failed\n");
}

static void ux500_pl011_exit(void)
{
	int ret;

	if (uart0_pins) {
		ret = ux500_pins_disable(uart0_pins);
		if (ret)
			pr_err("pl011: uart0 pins_disable failed\n");
	}
}

static struct uart_amba_plat_data ux500_pl011_pdata = {
	.init = ux500_pl011_init,
	.exit = ux500_pl011_exit,
	.reset = ux500_pl011_reset,
};

static struct platform_device *platform_devs[] __initdata = {
	&u8500_shrm_device,
	&ste_ff_vibra_device,
	&ste_timed_output_vibra_device,
#ifdef SSG_CAMERA_ENABLE	
	&ux500_mmio_device,
#endif
	&ux500_musb_device,
	&ux500_hwmem_device,
#ifdef CONFIG_SPI_GPIO
	&ux500_spi_gpio_device,
#endif
	&ux500_mcde_device,
	&ux500_b2r2_device,
	&u8500_thsens_device,
#ifdef CONFIG_STE_TRACE_MODEM
	&u8500_trace_modem,
#endif
#ifdef CONFIG_ANDROID_PMEM
	&u8500_pmem_hwb_device,
#endif
#ifdef CONFIG_CRYPTO_DEV_UX500_HASH
	&ux500_hash1_device,
#endif
	&ux500_cryp1_device,
#ifdef CONFIG_USB_ANDROID
	&android_usb_device,
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	&usb_mass_storage_device,
#endif
#ifdef CONFIG_USB_ANDROID_ECM
	&usb_ecm_device,
#endif
#ifdef CONFIG_USB_ANDROID_RNDIS
	&usb_rndis_device,
#endif
#ifdef CONFIG_USB_ANDROID_PHONET
	&usb_phonet_device,
#endif
#endif
#ifdef CONFIG_DB8500_MLOADER
	&mloader_fw_device,
#endif
#ifdef CONFIG_RFKILL
	&sec_device_rfkill,
#endif
};

#if defined(CONFIG_MPU_SENSORS_MPU3050)
static void godin_mpl_init(void)
{
	int intrpt_gpio = SENSOR_INT_GTI9060_R0_1;

	gpio_request(intrpt_gpio,"MPUIRQ");
	gpio_direction_input(intrpt_gpio);
}
#endif

static void __init godin_i2c_init (void)
{
	db8500_add_i2c0(&godin_i2c0_data);
	db8500_add_i2c1(&godin_i2c1_data);
	db8500_add_i2c2(&godin_i2c2_data);
	db8500_add_i2c3(&godin_i2c3_data);

	if (system_rev == GTI9060_R0_1) {
		i2c_register_board_info(0, ARRAY_AND_SIZE(gti9060_r0_1_i2c0_devices));
		i2c_register_board_info(1, ARRAY_AND_SIZE(gti9060_r0_1_i2c1_devices));
		i2c_register_board_info(2, ARRAY_AND_SIZE(gti9060_r0_1_i2c2_devices));
		i2c_register_board_info(3, ARRAY_AND_SIZE(gti9060_r0_1_i2c3_devices));
	
		platform_device_register(&gti9060_gpio_i2c_pdata);
		i2c_register_board_info(4, ARRAY_AND_SIZE(gti9060_r0_1_gpio_i2c_devices));
	}
}

#ifdef CONFIG_USB_ANDROID
/*
 * Public Id is a Unique number for each board and is stored
 * in Backup RAM at address 0x80151FC0, ..FC4, FC8, FCC and FD0.
 *
 * This function reads the Public Ids from this address and returns
 * a single string, which can be used as serial number for USB.
 * Input parameter - serial_number should be of 'len' bytes long
*/
static void fetch_usb_serial_no(int len)
{
	u32 buf[5];
	void __iomem *backup_ram = NULL;

	backup_ram = ioremap(PUBLIC_ID_BACKUPRAM1, 0x14);

	if (backup_ram) {
		buf[0] = readl(backup_ram);
		buf[1] = readl(backup_ram + 4);
		buf[2] = readl(backup_ram + 8);
		buf[3] = readl(backup_ram + 0x0c);
		buf[4] = readl(backup_ram + 0x10);

		snprintf(android_usb_pdata.serial_number, len+1, "%X%X%X%X%X",
					buf[0], buf[1], buf[2], buf[3], buf[4]);
		iounmap(backup_ram);
	} else {
		printk(KERN_ERR "$$ ioremap failed\n");
	}
}
#endif


static void __init godin_spi_init(void)
{
	db8500_add_spi0(&godin_spi0_data);
	if (system_rev != GTI9060_R0_1)
		db8500_add_msp2_spi(&godin_msp2_spi_data);
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
}

static void __init godin_uart_init(void)
{
	db8500_add_uart0(&ux500_pl011_pdata);
	db8500_add_uart1();
	db8500_add_uart2();
}


void godin_cam_init(void);
    
static void __init ssg_init_machine(void)
{
	sec_common_init() ;
    
	sec_common_init_early() ;

#ifdef CONFIG_ANDROID_RAM_CONSOLE
	if (ram_console_device.num_resources == 1)
		platform_device_register(&ram_console_device);
#endif

#ifdef CONFIG_REGULATOR
	platform_device_register(&u8500_regulator_dev);
#endif
	u8500_init_devices();

#ifdef CONFIG_USB_ANDROID
	fetch_usb_serial_no(USB_SERIAL_NUMBER_LEN);
#endif

	platform_add_devices(platform_devs, ARRAY_SIZE(platform_devs));

	ssg_pins_init();

	platform_device_register(&ux500_ab8500_device);

/* TODO: This needs to be customised for Samsung Camera code 
	mmio_config.xenon_charge.gpio = EGPIO_PIN_5; */
	
	godin_i2c_init();
	godin_spi_init();
	mop500_msp_init();		/* generic for now */
	godin_uart_init();

#if defined(CONFIG_MPU_SENSORS_MPU3050)
	godin_mpl_init();
#endif


#ifdef CONFIG_KEYBOARD_GPIO
	if (system_rev == GTI9060_R0_1 ) {
		platform_device_register(&gti9060_gpio_keys_device);
	}
#endif

//godin +
        godin_cam_init();
//godin -

	platform_add_devices(godin_platform_devices,
			     ARRAY_SIZE(godin_platform_devices));

	sec_common_init_post() ;
}

static int __init sec_debug_setup(char *str)
{
	if (get_option(&str, &sec_debug_settings) != 1)
		sec_debug_settings = 0;
}
__setup("debug=",sec_debug_setup);

/* we have equally similar boards with very minimal
 * changes, so we detect the platform during boot
 */
static int __init board_id_setup(char *str)
{
	if (!str)
		return 1;

	switch (*str) {
	case '6':
		printk(KERN_INFO "GT-I9060 Board Rev 0.1\n");
		system_rev = GTI9060_R0_1;
		break;
	default:
		printk(KERN_INFO "Unknown board_id=%c\n", *str);
		break;
	};

	return 1;
}
__setup("board_id=", board_id_setup);

MACHINE_START(GODIN, "SAMSUNG GT-I9060")
	/* Maintainer: SAMSUNG based on ST Ericsson */
	.phys_io	= U8500_UART2_BASE,
	.io_pg_offst	= (IO_ADDRESS(U8500_UART2_BASE) >> 18) & 0xfffc,
	.boot_params	= 0x00000100,
	.map_io		= u8500_map_io,
	.init_irq	= ux500_init_irq,
	.timer		= &u8500_timer,
	.init_machine	= ssg_init_machine,
MACHINE_END

