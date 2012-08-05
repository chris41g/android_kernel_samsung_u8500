/*
 * Copyright (C) 2008-2009 ST-Ericsson
 *
 * Author: Srinidhi KASAGAR <srinidhi.kasagar@stericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 *
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/amba/bus.h>
#include <linux/amba/pl022.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/hsi/hsi.h>
#include <linux/i2s/i2s.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/mfd/stmpe.h>
#include <linux/mfd/tc35892.h>
#include <linux/i2c/lp5521.h>
#include <linux/power_supply.h>
#include <linux/mfd/abx500.h>
#include <linux/mfd/ab8500.h>
#include <linux/input/bu21013.h>
#include <linux/mmio.h>
#include <linux/cw1200-wlan.h>
#include <linux/spi/stm_msp.h>
#include <linux/leds_pwm.h>
#include <linux/pwm_backlight.h>
#include <linux/mfd/ab8500/denc.h>
#include <linux/mfd/ab8500/ab8500-gpio.h>
#include <linux/delay.h>

#ifdef CONFIG_USB_ANDROID
#include <linux/usb/android_composite.h>
#endif

#include <asm/mach/arch.h>
#include <asm/mach/irq.h>
#include <asm/mach/map.h>
#include <asm/mach-types.h>

#include <plat/pincfg.h>
#include <plat/ske.h>
#include <plat/i2c.h>
#include <plat/ste_dma40.h>

#include <mach/devices.h>
#include <mach/sensors1p.h>
#include <mach/ab8500-accdet.h>
#include <mach/ste_audio_io.h>
#include <mach/setup.h>
#include <mach/tc35893-keypad.h>
#include <video/av8100.h>
#include <mach/devices.h>
#include <mach/irqs.h>
#include <mach/ste-dma40-db8500.h>
#ifdef CONFIG_DB8500_MLOADER
#include <mach/mloader-dbx500.h>
#endif

#include <sound/ux500_ab8500_ext.h>

#include "devices-db8500.h"
#include "board-mop500-regulators.h"
#include "regulator-u8500.h"
#include "pins-db8500.h"
#include "board-mop500-bm.h"
#include "board-mop500.h"
#include "pins.h"

#define IRQ_KP 1 /*To DO*/

#ifdef CONFIG_USB_ANDROID
#define PUBLIC_ID_BACKUPRAM1 (U8500_BACKUPRAM1_BASE + 0x0FC0)
#define USB_SERIAL_NUMBER_LEN 31
#endif

u32 config_uart_gpio_pin = 29;
u8 ux500_debug_uart_config = 2;

/*
 * This variable holds the number of touchscreen (bu21013) enabled.
 */
static int cs_en_check;

/*
 * ux500 keymaps
 *
 * Organized row-wise as on the UIB, starting at the top-left
 *
 * we support two key layouts, specific to requirements. The first
 * keylayout includes controls for power/volume a few generic keys;
 * the second key layout contains the full numeric layout, enter/back/left
 * buttons along with a "."(dot), specifically for connectivity testing
 */
static const unsigned int ux500_keymap[] = {
#ifdef CONFIG_KEYLAYOUT_LAYOUT1
	KEY(2, 5, KEY_END),
	KEY(4, 1, KEY_HOME),
	KEY(3, 5, KEY_VOLUMEDOWN),
	KEY(1, 3, KEY_EMAIL),
	KEY(5, 2, KEY_RIGHT),
	KEY(5, 0, KEY_BACKSPACE),

	KEY(0, 5, KEY_MENU),
	KEY(7, 6, KEY_ENTER),
	KEY(4, 5, KEY_0),
	KEY(6, 7, KEY_DOT),
	KEY(3, 4, KEY_UP),
	KEY(3, 3, KEY_DOWN),

	KEY(6, 4, KEY_SEND),
	KEY(6, 2, KEY_BACK),
	KEY(4, 2, KEY_VOLUMEUP),
	KEY(5, 5, KEY_SPACE),
	KEY(4, 3, KEY_LEFT),
	KEY(3, 2, KEY_SEARCH),
#else
#ifdef CONFIG_KEYLAYOUT_LAYOUT2
	KEY(2, 5, KEY_RIGHT),
	KEY(4, 1, KEY_ENTER),
	KEY(3, 5, KEY_MENU),
	KEY(1, 3, KEY_3),
	KEY(5, 2, KEY_6),
	KEY(5, 0, KEY_9),

	KEY(0, 5, KEY_UP),
	KEY(7, 6, KEY_DOWN),
	KEY(4, 5, KEY_0),
	KEY(6, 7, KEY_2),
	KEY(3, 4, KEY_5),
	KEY(3, 3, KEY_8),

	KEY(6, 4, KEY_LEFT),
	KEY(6, 2, KEY_BACK),
	KEY(4, 2, KEY_KPDOT),
	KEY(5, 5, KEY_1),
	KEY(4, 3, KEY_4),
	KEY(3, 2, KEY_7),
#else
#warning "No keypad layout defined."
#endif
#endif
};

static struct matrix_keymap_data ux500_keymap_data = {
	.keymap         = ux500_keymap,
	.keymap_size    = ARRAY_SIZE(ux500_keymap),
};

/*
 * STMPE1601
 */
static struct stmpe_keypad_platform_data stmpe1601_keypad_data = {
	.debounce_ms    = 64,
	.scan_count     = 8,
	.no_autorepeat	= true,
	.keymap_data    = &ux500_keymap_data,
};

static struct stmpe_platform_data stmpe1601_data = {
	.id		= 1,
	.blocks		= STMPE_BLOCK_KEYPAD,
	.irq_trigger    = IRQF_TRIGGER_FALLING,
	.irq_base       = MOP500_STMPE1601_IRQ(0),
	.keypad		= &stmpe1601_keypad_data,
	.autosleep	= true,
	.autosleep_timeout = 1024,
};

/*
 * Nomadik SKE keypad
 */
#define ROW_PIN_I0      164
#define ROW_PIN_I1      163
#define ROW_PIN_I2      162
#define ROW_PIN_I3      161
#define ROW_PIN_I4      156
#define ROW_PIN_I5      155
#define ROW_PIN_I6      154
#define ROW_PIN_I7      153
#define COL_PIN_O0      168
#define COL_PIN_O1      167
#define COL_PIN_O2      166
#define COL_PIN_O3      165
#define COL_PIN_O4      160
#define COL_PIN_O5      159
#define COL_PIN_O6      158
#define COL_PIN_O7      157

static int ske_kp_rows[] = {
	ROW_PIN_I0, ROW_PIN_I1, ROW_PIN_I2, ROW_PIN_I3,
	ROW_PIN_I4, ROW_PIN_I5, ROW_PIN_I6, ROW_PIN_I7,
};
static int ske_kp_cols[] = {
	COL_PIN_O0, COL_PIN_O1, COL_PIN_O2, COL_PIN_O3,
	COL_PIN_O4, COL_PIN_O5, COL_PIN_O6, COL_PIN_O7,
};

static bool ske_config;
/*
 * ske_set_gpio_row: request and set gpio rows
 */
static int ske_set_gpio_row(int gpio)
{
	int ret;

	if (!ske_config) {
		ret = gpio_request(gpio, "ske-kp");
		if (ret < 0) {
			pr_err("ske_set_gpio_row: gpio request failed\n");
			return ret;
		}
	}

	ret = gpio_direction_output(gpio, 1);
	if (ret < 0) {
		pr_err("ske_set_gpio_row: gpio direction failed\n");
		gpio_free(gpio);
	}

	return ret;
}

/*
 * ske_kp_init - enable the gpio configuration
 */
static int ske_kp_init(void)
{
	struct ux500_pins *pins;
	int ret, i;

	pins = ux500_pins_get("ske");
	if (pins)
		ux500_pins_enable(pins);

	for (i = 0; i < SKE_KPD_MAX_ROWS; i++) {
		ret = ske_set_gpio_row(ske_kp_rows[i]);
		if (ret < 0) {
			pr_err("ske_kp_init: failed init\n");
			return ret;
		}
	}
	if (!ske_config)
		ske_config = true;

	return 0;
}

static int ske_kp_exit(void)
{
	struct ux500_pins *pins;

	pins = ux500_pins_get("ske");
	if (pins)
		ux500_pins_disable(pins);

	return 0;
}

static struct ske_keypad_platform_data mop500_ske_keypad_data = {
	.init           = ske_kp_init,
	.exit           = ske_kp_exit,
	.gpio_input_pins = ske_kp_rows,
	.gpio_output_pins = ske_kp_cols,
	.keymap_data    = &ux500_keymap_data,
	.no_autorepeat  = true,
	.krow           = SKE_KPD_MAX_ROWS,     /* 8x8 matrix */
	.kcol           = SKE_KPD_MAX_COLS,
	.debounce_ms    = 20,                   /* in timeout period */
	.switch_delay	= 200,			/* in jiffies */
};

static struct av8100_platform_data av8100_plat_data = {
	.irq = GPIO_TO_IRQ(192),
	.reset = 196,
	.inputclk_id = "sysclk2",
	.regulator_pwr_id = "hdmi_1v8",
	.alt_powerupseq = true,
	.mclk_freq = 3, /* MCLK_RNG_31_38 */
};

/*
 * TC35892 Expander
 */

static struct tc35892_gpio_platform_data tc35892_gpio_platform_data = {
	.gpio_base = U8500_NR_GPIO,
};

static struct tc35892_platform_data tc35892_data = {
	.gpio = &tc35892_gpio_platform_data,
	.irq_base = MOP500_EGPIO_IRQ_BASE,
};

static const unsigned int nuib_keymap[] = {
	KEY(3, 1, KEY_END),
	KEY(4, 1, KEY_HOME),
	KEY(6, 4, KEY_VOLUMEDOWN),
	KEY(4, 2, KEY_EMAIL),
	KEY(3, 3, KEY_RIGHT),
	KEY(2, 5, KEY_BACKSPACE),

	KEY(6, 7, KEY_MENU),
	KEY(5, 0, KEY_ENTER),
	KEY(4, 3, KEY_0),
	KEY(3, 4, KEY_DOT),
	KEY(5, 2, KEY_UP),
	KEY(3, 5, KEY_DOWN),

	KEY(4, 5, KEY_SEND),
	KEY(0, 5, KEY_BACK),
	KEY(6, 2, KEY_VOLUMEUP),
	KEY(1, 3, KEY_SPACE),
	KEY(7, 6, KEY_LEFT),
	KEY(5, 5, KEY_SEARCH),
};

static struct matrix_keymap_data nuib_keymap_data = {
	.keymap         = nuib_keymap,
	.keymap_size    = ARRAY_SIZE(nuib_keymap),
};

static struct tc35893_platform_data tc35893_data = {
	.krow = TC_KPD_ROWS,
	.kcol = TC_KPD_COLUMNS,
	.debounce_period = TC_KPD_DEBOUNCE_PERIOD,
	.settle_time = TC_KPD_SETTLE_TIME,
	.irqtype = (IRQF_TRIGGER_FALLING),
	.irq = GPIO_TO_IRQ(218),
	.enable_wakeup = true,
	.keymap_data    = &nuib_keymap_data,
	.no_autorepeat  = true,
};

static struct lp5521_platform_data lp5521_data = {
	.mode		= LP5521_MODE_DIRECT_CONTROL,
	.label		= "uib-led",
	.red_present    = true,
	.green_present  = true,
	.blue_present   = true,
};

/**
 * Touch panel related platform specific initialization
 */

/**
 * bu21013_gpio_board_init : configures the touch panel.
 * @reset_pin: reset pin number
 * This function can be used to configures
 * the voltage and reset the touch panel controller.
 */
static int bu21013_gpio_board_init(int reset_pin)
{
	int retval = 0;

	cs_en_check++;
	if (cs_en_check == 1) {
		retval = gpio_request(reset_pin, "touchp_reset");
		if (retval) {
			printk(KERN_ERR "Unable to request gpio reset_pin");
			return retval;
		}
		retval = gpio_direction_output(reset_pin, 1);
		if (retval < 0) {
			printk(KERN_ERR "%s: gpio direction failed\n",
							 __func__);
			return retval;
		}
		gpio_set_value(reset_pin, 1);
	}
	return retval;
}

/**
 * bu21013_gpio_board_exit : deconfigures the touch panel controller
 * @reset_pin: reset pin number
 * This function can be used to deconfigures the chip selection
 * for touch panel controller.
 */
static int bu21013_gpio_board_exit(int reset_pin)
{
	int retval = 0;

	if (cs_en_check == 1) {
		retval = gpio_direction_output(reset_pin, 0);
		if (retval < 0) {
			printk(KERN_ERR "%s: gpio direction failed\n",
							__func__);
			return retval;
		}
		gpio_set_value(reset_pin, 0);

		gpio_free(reset_pin);
	}
	cs_en_check--;
	return retval;
}

/**
 * bu21013_read_pin_val : get the interrupt pin value
 * This function can be used to get the interrupt pin value for touch panel
 * controller.
 */
static int bu21013_read_pin_val(void)
{
	return gpio_get_value(TOUCH_GPIO_PIN);
}

static struct bu21013_platform_device tsc_plat_device = {
	.cs_en = bu21013_gpio_board_init,
	.cs_dis = bu21013_gpio_board_exit,
	.irq_read_val = bu21013_read_pin_val,
	.irq = GPIO_TO_IRQ(TOUCH_GPIO_PIN),
	.x_max_res = 480,
	.y_max_res = 864,
	.touch_x_max = TOUCH_XMAX,
	.touch_y_max = TOUCH_YMAX,
	.portrait = true,
	.has_ext_clk = true,
	.enable_ext_clk = false,
#if defined(CONFIG_DISPLAY_GENERIC_DSI_PRIMARY_ROTATION_ANGLE) &&	\
		CONFIG_DISPLAY_GENERIC_DSI_PRIMARY_ROTATION_ANGLE == 270
	.x_flip		= true,
	.y_flip		= false,
#else
	.x_flip		= false,
	.y_flip		= true,
#endif
};

static struct bu21013_platform_device tsc_cntl2_plat_device = {
	.cs_en = bu21013_gpio_board_init,
	.cs_dis = bu21013_gpio_board_exit,
	.irq_read_val = bu21013_read_pin_val,
	.irq = GPIO_TO_IRQ(TOUCH_GPIO_PIN),
	.x_max_res = 480,
	.y_max_res = 864,
	.touch_x_max = TOUCH_XMAX,
	.touch_y_max = TOUCH_YMAX,
	.portrait = true,
	.has_ext_clk = true,
	.enable_ext_clk = false,
#if defined(CONFIG_DISPLAY_GENERIC_DSI_PRIMARY_ROTATION_ANGLE) &&	\
		CONFIG_DISPLAY_GENERIC_DSI_PRIMARY_ROTATION_ANGLE == 270
	.x_flip		= true,
	.y_flip		= false,
#else
	.x_flip		= false,
	.y_flip		= true,
#endif
};

static struct ab3550_platform_data ab3550_plf_data = {
	.irq = {
		.base = 0,
		.count = 0,
	},
	.dev_data = {
	},
	.dev_data_sz = {
	},
	.init_settings = NULL,
	.init_settings_sz = 0,
};

static struct i2c_board_info __initdata mop500_i2c0_devices[] = {
	{

		I2C_BOARD_INFO("tc35892", 0x42),
		.platform_data = &tc35892_data,
		.irq = GPIO_TO_IRQ(217),
	},
	{
		/* STMPE1601 on UIB */
		I2C_BOARD_INFO("stmpe1601", 0x40),
		.irq = NOMADIK_GPIO_TO_IRQ(218),
		.platform_data = &stmpe1601_data,
		.flags = I2C_CLIENT_WAKE, /* enable wakeup */
	},
	{
		/* TC35893 keypad */
		I2C_BOARD_INFO("tc35893-kp", 0x44),
		.platform_data = &tc35893_data,
		.flags = I2C_CLIENT_WAKE, /* enable wakeup */
	},
	{
		I2C_BOARD_INFO("av8100", 0x70),
		.platform_data = &av8100_plat_data,
	},
	{
	 /* FIXME - address TBD, TODO */
	 I2C_BOARD_INFO("uib-kpd", 0x45),
	},
	{
	 /* Audio step-up */
	 I2C_BOARD_INFO("tps61052", 0x33),
	}
};
static struct i2c_board_info __initdata mop500_i2c1_devices[] = {
	{
	 /* GPS - Address TBD, FIXME */
	 I2C_BOARD_INFO("gps", 0x68),
	},
	{
		/* AB3550 */
		I2C_BOARD_INFO("ab3550", 0x4A),
		.irq = -1,
		.platform_data = &ab3550_plf_data,
	}
};
static struct i2c_board_info __initdata mop500_i2c2_devices[] = {
	{
		/* ST 3D accelerometer @ 0x3A(w),0x3B(r) */
		I2C_BOARD_INFO("lis302dl", 0x1D),
	},
	{
		/* ASAHI KASEI AK8974 magnetometer, addr TBD FIXME */
		I2C_BOARD_INFO("ak8974", 0x1),
	},
#if defined(CONFIG_SENSORS_BH1780)
	{
		/* Rohm BH1780GLI ambient light sensor */
		I2C_BOARD_INFO("bh1780", 0x29),
	},
#endif
	{
		/* RGB LED driver, there are 1st and 2nd, TODO */
		I2C_BOARD_INFO("lp5521", 0x33),
		.platform_data = &lp5521_data,
	},
};
static struct i2c_board_info __initdata mop500_i2c3_devices[] = {
	{
	 /* NFC - Address TBD, FIXME */
	 I2C_BOARD_INFO("nfc", 0x68),
	},
	{
		/* Touschscreen */
		I2C_BOARD_INFO("bu21013_ts", 0x5C),
		.platform_data = &tsc_plat_device,
	},
	{
		/* Touschscreen */
		I2C_BOARD_INFO("bu21013_ts", 0x5D),
		.platform_data = &tsc_cntl2_plat_device,
	},
};

/*
 * MSP-SPI
 */

#define NUM_MSP_CLIENTS 10

static struct stm_msp_controller mop500_msp2_spi_data = {
	.id		= MSP_2_CONTROLLER,
	.num_chipselect	= NUM_MSP_CLIENTS,
	.base_addr	= U8500_MSP2_BASE,
	.device_name	= "msp2",
};

/*
 * SSP
 */

#define NUM_SSP_CLIENTS 10

#ifdef CONFIG_STE_DMA40
struct stedma40_chan_cfg ssp0_dma_cfg_rx = {
	.dir = STEDMA40_PERIPH_TO_MEM,
	.src_dev_type = DB8500_DMA_DEV8_SSP0_RX,
	.dst_dev_type = STEDMA40_DEV_DST_MEMORY,
	.src_info.data_width = STEDMA40_WORD_WIDTH,
	.dst_info.data_width = STEDMA40_WORD_WIDTH,
};

static struct stedma40_chan_cfg ssp0_dma_cfg_tx = {
	.dir = STEDMA40_MEM_TO_PERIPH,
	.src_dev_type = STEDMA40_DEV_SRC_MEMORY,
	.dst_dev_type = DB8500_DMA_DEV8_SSP0_TX,
	.src_info.data_width = STEDMA40_WORD_WIDTH,
	.dst_info.data_width = STEDMA40_WORD_WIDTH,
};
#endif

static struct pl022_ssp_controller mop500_ssp0_data = {
	.bus_id		= SSP_0_CONTROLLER,
	.num_chipselect	= NUM_SSP_CLIENTS,
#ifdef CONFIG_STE_DMA40
	.enable_dma	= 1,
	.dma_filter	= stedma40_filter,
	.dma_rx_param	= &ssp0_dma_cfg_rx,
	.dma_tx_param	= &ssp0_dma_cfg_tx,
#endif

};

#ifdef CONFIG_SENSORS1P_MOP
static struct sensors1p_config sensors1p_config = {
	/* SFH7741 */
	.proximity = {
		.startup_time = 120, /* ms */
		.regulator = "v-proximity",
	},
	/* HED54XXU11 */
	.hal = {
		.startup_time = 100, /* Actually, I have no clue. */
		.regulator = "v-hal",
	},
};

struct platform_device sensors1p_device = {
	.name = "sensors1p",
	.dev = {
		.platform_data = (void *)&sensors1p_config,
	},
};
#endif

#ifdef CONFIG_USB_ANDROID

static char *usb_functions_adb[] = {
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	"usb_mass_storage",
#endif
#ifdef CONFIG_USB_ANDROID_ECM
	"cdc_ethernet",
#endif
#ifdef CONFIG_USB_ANDROID_ACM
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_ADB
	"adb",
#endif

#ifdef CONFIG_USB_ANDROID_PHONET
        "phonet",
#endif
};

static struct android_usb_product usb_products[] = {
	{
		.product_id	= 0x2323,
		.num_functions	= ARRAY_SIZE(usb_functions_adb),
		.functions	= usb_functions_adb,
	},
};

struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x04cc,
	.product_id	= 0x2323,
	.version	= 0x0100,
	.product_name	= "Android Phone",
	.manufacturer_name = "ST-Ericsson",
	.serial_number	= "0123456789ABCDEF0123456789ABCDE",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_adb),
	.functions = usb_functions_adb,
};

struct platform_device ux500_android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
	.platform_data = &android_usb_pdata,
	},
};

#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 2,
	.vendor		= "ST-Ericsson",
	.product	= "Android Phone",
	.release	= 0x0100,
};

struct platform_device ux500_usb_mass_storage_device = {
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
	.vendorID	= 0x04cc,
	.vendorDescr = "ST Ericsson",
};

struct platform_device ux500_usb_ecm_device = {
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
	.vendorID	= 0x04cc,
	.vendorDescr = "ST Ericsson",
};

struct platform_device ux500_usb_rndis_device = {
	.name = "rndis",
	.id = -1,
	.dev = {
		.platform_data = &usb_rndis_pdata,
	},
};
#endif /* CONFIG_USB_ANDROID_RNDIS */

#endif /* CONFIG_USB_ANDROID */

#define U8500_I2C_CONTROLLER(id, _slsu, _tft, _rft, clk, t_out, _sm) \
static struct nmk_i2c_controller mop500_i2c##id##_data = { \
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

#ifdef CONFIG_HSI
static struct hsi_board_info __initdata u8500_hsi_devices[] = {
	{
		.name = "hsi_char",
		.hsi_id = 0,
		.port = 0,
		.tx_cfg = {
			.mode = HSI_MODE_STREAM,
			.channels = 2,
			.speed = 100000,
			{.arb_mode = HSI_ARB_RR},
		},
		.rx_cfg = {
			.mode = HSI_MODE_STREAM,
			.channels = 2,
			.speed = 200000,
			{.flow = HSI_FLOW_SYNC},
		},
	},
	{
		.name = "hsi_test",
		.hsi_id = 0,
		.port = 0,
		.tx_cfg = {
			.mode = HSI_MODE_FRAME,
			.channels = 2,
			.speed = 100000,
			{.arb_mode = HSI_ARB_RR},
		},
		.rx_cfg = {
			.mode = HSI_MODE_FRAME,
			.channels = 2,
			.speed = 200000,
			{.flow = HSI_FLOW_SYNC},
		},
	},
	{
		.name = "cfhsi_v3_driver",
		.hsi_id = 0,
		.port = 0,
		.tx_cfg = {
			.mode = HSI_MODE_STREAM,
			.channels = 2,
			.speed = 20000,
			{.arb_mode = HSI_ARB_RR},
		},
		.rx_cfg = {
			.mode = HSI_MODE_STREAM,
			.channels = 2,
			.speed = 200000,
			{.flow = HSI_FLOW_SYNC},
		},
	},
};
#endif

#ifdef CONFIG_INPUT_AB8500_ACCDET
static struct ab8500_accdet_platform_data ab8500_accdet_pdata = {
       .btn_keycode = KEY_MEDIA,
       .accdet1_dbth = ACCDET1_TH_1200mV | ACCDET1_DB_70ms,
       .accdet2122_th = ACCDET21_TH_1000mV | ACCDET22_TH_1000mV,
       .video_ctrl_gpio = AB8500_PIN_GPIO35,
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
	 * GpioSel1 = 0x0F => Pin GPIO1 (SysClkReq2)
	 *                    Pin GPIO2 (SysClkReq3)
	 *                    Pin GPIO3 (SysClkReq4)
	 *                    Pin GPIO4 (SysClkReq6)
	 * GpioSel2 = 0x1E => Pins GPIO10 to GPIO13 are configured as GPIO
	 * GpioSel3 = 0x80 => Pin GPIO24 (SysClkReq7) is configured as GPIO
	 * GpioSel4 = 0x01 => Pin GPIO25 (SysClkReq8) is configured as GPIO
	 * GpioSel5 = 0x78 => Pin GPIO36 (ApeSpiClk)
			      Pin GPIO37 (ApeSpiCSn)
			      Pin GPIO38 (ApeSpiDout)
			      Pin GPIO39 (ApeSpiDin) are configured as GPIO
	 * GpioSel6 = 0x02 => Pin GPIO42 (SysClkReq5) is configured as GPIO
	 * AlternaFunction = 0x00 => If Pins GPIO10 to 13 are not configured
	 * as GPIO then this register selectes the alternate fucntions
	 */
	.initial_pin_config     = {0x0F, 0x1E, 0x80, 0x01, 0x78, 0x02, 0x00},

	/* initial_pin_direction allows for the initial GPIO direction to
	 * be set.
	 */
	.initial_pin_direction  = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

	/*
	 * initial_pin_pullups allows for the intial configuration of the
	 * GPIO pullup/pulldown configuration.
	 */
	.initial_pin_pullups    = {0xE0, 0x01, 0x00, 0x00, 0x00, 0x00},
};
#endif

#ifdef CONFIG_AB8500_DENC
static struct ab8500_denc_platform_data ab8500_denc_pdata = {
	.ddr_enable = true,
	.ddr_little_endian = false,
};
#endif

static struct regulator_init_data *u8500_regulators[U8500_NUM_REGULATORS] = {
	[U8500_REGULATOR_VAPE]			= &u8500_vape_regulator,
	[U8500_REGULATOR_VARM]			= &u8500_varm_regulator,
	[U8500_REGULATOR_VMODEM]		= &u8500_vmodem_regulator,
	[U8500_REGULATOR_VPLL]			= &u8500_vpll_regulator,
	[U8500_REGULATOR_VSMPS1]		= &u8500_vsmps1_regulator,
	[U8500_REGULATOR_VSMPS2]		= &u8500_vsmps2_regulator,
	[U8500_REGULATOR_VSMPS3]		= &u8500_vsmps3_regulator,
	[U8500_REGULATOR_VRF1]			= &u8500_vrf1_regulator,
	[U8500_REGULATOR_SWITCH_SVAMMDSP]	= &u8500_svammdsp_regulator,
	[U8500_REGULATOR_SWITCH_SVAMMDSPRET]	= &u8500_svammdspret_regulator,
	[U8500_REGULATOR_SWITCH_SVAPIPE]	= &u8500_svapipe_regulator,
	[U8500_REGULATOR_SWITCH_SIAMMDSP]	= &u8500_siammdsp_regulator,
	[U8500_REGULATOR_SWITCH_SIAMMDSPRET]	= &u8500_siammdspret_regulator,
	[U8500_REGULATOR_SWITCH_SIAPIPE]	= &u8500_siapipe_regulator,
	[U8500_REGULATOR_SWITCH_SGA]		= &u8500_sga_regulator,
	[U8500_REGULATOR_SWITCH_B2R2_MCDE]	= &u8500_b2r2_mcde_regulator,
	[U8500_REGULATOR_SWITCH_ESRAM12]	= &u8500_esram12_regulator,
	[U8500_REGULATOR_SWITCH_ESRAM12RET]	= &u8500_esram12ret_regulator,
	[U8500_REGULATOR_SWITCH_ESRAM34]	= &u8500_esram34_regulator,
	[U8500_REGULATOR_SWITCH_ESRAM34RET]	= &u8500_esram34ret_regulator,
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
	.regulator_reg_init	= ab8500_regulator_reg_init,
	.num_regulator_reg_init	= ARRAY_SIZE(ab8500_regulator_reg_init),
	.regulator		= ab8500_regulators,
	.num_regulator		= ARRAY_SIZE(ab8500_regulators),
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

#ifdef CONFIG_LEDS_PWM
static struct led_pwm pwm_leds_data[] = {
	[0] = {
		.name = "lcd-backlight",
		.pwm_id = 1,
		.max_brightness = 255,
		.lth_brightness = 90,
		.pwm_period_ns = 1023,
	},
#ifdef CONFIG_DISPLAY_GENERIC_DSI_SECONDARY
	[1] = {
		.name = "sec-lcd-backlight",
		.pwm_id = 2,
		.max_brightness = 255,
		.lth_brightness = 90,
		.pwm_period_ns = 1023,
	},
#endif
};



static struct led_pwm_platform_data u8500_leds_data = {
#ifdef CONFIG_DISPLAY_GENERIC_DSI_SECONDARY
	.num_leds = 2,
#else
	.num_leds = 1,
#endif
	.leds = pwm_leds_data,
};

static struct platform_device ux500_leds_device = {
	.name = "leds_pwm",
	.dev = {
		.platform_data = &u8500_leds_data,
	},
};
#endif

#ifdef CONFIG_BACKLIGHT_PWM
static struct platform_pwm_backlight_data u8500_backlight_data[] = {
	[0] = {
	.pwm_id = 1,
	.max_brightness = 255,
	.dft_brightness = 200,
	.lth_brightness = 90,
	.pwm_period_ns = 1023,
	},
	[1] = {
	.pwm_id = 2,
	.max_brightness = 255,
	.dft_brightness = 200,
	.lth_brightness = 90,
	.pwm_period_ns = 1023,
	},
};

static struct platform_device ux500_backlight_device[] = {
	[0] = {
		.name = "pwm-backlight",
		.id = 0,
		.dev = {
			.platform_data = &u8500_backlight_data[0],
		},
	},
	[1] = {
		.name = "pwm-backlight",
		.id = 1,
		.dev = {
			.platform_data = &u8500_backlight_data[1],
		},
	},
};
#endif

/* Force feedback vibrator device */
static struct platform_device ste_ff_vibra_device = {
	.name = "ste_ff_vibra"
};

static struct wlan1200_platform_data hrefv60_plat_data = {
	.gpio_irq	=	4,
	.gpio_enable	=	85,
};

static struct wlan1200_platform_data hrefv50_plat_data = {
	.gpio_irq	=	216,
	.gpio_enable	=	215,
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

static struct platform_device *u8500_platform_devices[] __initdata = {
	/*TODO - add platform devices here */
#ifdef CONFIG_SENSORS1P_MOP
	&sensors1p_device,
#endif
#ifdef CONFIG_LEDS_PWM
	&ux500_leds_device,
#endif
#ifdef CONFIG_BACKLIGHT_PWM
	&ux500_backlight_device[0],
	&ux500_backlight_device[1],
#endif
#ifdef CONFIG_STM_TRACE
	&ux500_stm_device,
#endif
};
static struct platform_device *platform_devs[] __initdata = {
	&u8500_shrm_device,
	&ste_ff_vibra_device,
#ifdef CONFIG_U8500_MMIO
	&ux500_mmio_device,
#endif
	&ux500_musb_device,
	&ux500_hwmem_device,
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
	&ux500_android_usb_device,
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	&ux500_usb_mass_storage_device,
#endif
#ifdef CONFIG_USB_ANDROID_ECM
	&ux500_usb_ecm_device,
#endif
#ifdef CONFIG_USB_ANDROID_RNDIS
	&ux500_usb_rndis_device,
#endif
#endif
#ifdef CONFIG_DB8500_MLOADER
	&mloader_fw_device,
#endif
#ifdef CONFIG_HSI
	&u8500_hsi_device,
#endif
};

static void __init mop500_i2c_init(void)
{
	db8500_add_i2c0(&mop500_i2c0_data);
	db8500_add_i2c1(&mop500_i2c1_data);
	db8500_add_i2c2(&mop500_i2c2_data);
	db8500_add_i2c3(&mop500_i2c3_data);

	i2c_register_board_info(0, ARRAY_AND_SIZE(mop500_i2c0_devices));
	i2c_register_board_info(1, ARRAY_AND_SIZE(mop500_i2c1_devices));
	i2c_register_board_info(2, ARRAY_AND_SIZE(mop500_i2c2_devices));
	i2c_register_board_info(3, ARRAY_AND_SIZE(mop500_i2c3_devices));
}

static void __init mop500_spi_init(void)
{
	db8500_add_ssp0(&mop500_ssp0_data);
	db8500_add_msp2_spi(&mop500_msp2_spi_data);
}

static void __init mop500_uart_init(void)
{
	db8500_add_uart0(&ux500_pl011_pdata);
	db8500_add_uart1();
	db8500_add_uart2();
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

	if (len > strlen(android_usb_pdata.serial_number)) {
		printk(KERN_ERR "$$ serial number is too long! truncating\n");
		len = strlen(android_usb_pdata.serial_number);
	}

	backup_ram = ioremap(PUBLIC_ID_BACKUPRAM1, 0x14);

	if (backup_ram) {
		buf[0] = readl(backup_ram);
		buf[1] = readl(backup_ram + 4);
		buf[2] = readl(backup_ram + 8);
		buf[3] = readl(backup_ram + 0x0c);
		buf[4] = readl(backup_ram + 0x10);

		snprintf(android_usb_pdata.serial_number, len+1,
				"%.8X%.8X%.8X%.8X%.8X",
				buf[0], buf[1], buf[2], buf[3], buf[4]);
		iounmap(backup_ram);
	} else {
		printk(KERN_ERR "$$ ioremap failed\n");
	}
}
#endif

static void device_gpio_cfg(void)
{
	if (machine_is_hrefv60())	{
		tsc_plat_device.cs_pin = HREFV60_TOUCH_RST_GPIO;
		tsc_cntl2_plat_device.cs_pin = HREFV60_TOUCH_RST_GPIO;
#ifdef CONFIG_SENSORS1P_MOP
		sensors1p_config.proximity.pin = HREFV60_PROX_SENSE_GPIO;
		sensors1p_config.hal.pin = HREFV60_HAL_SW_GPIO;
#endif
	} else {
		tsc_plat_device.cs_pin = EGPIO_PIN_13;
		tsc_cntl2_plat_device.cs_pin = EGPIO_PIN_13;
#ifdef CONFIG_SENSORS1P_MOP
		sensors1p_config.proximity.pin = EGPIO_PIN_7;
		sensors1p_config.hal.pin = EGPIO_PIN_8;
#endif
	}
}

static void cw1200_gpio_cfg(void)
{
	if (machine_is_hrefv60())
		wlan1200_set_platdata(&hrefv60_plat_data);
	else
		wlan1200_set_platdata(&hrefv50_plat_data);
}

/*
 *  On boards hrefpv60 and later, the accessory insertion/removal,
 *  button press/release are inverted.
*/
static void accessory_detect_config(void)
{
	if (machine_is_hrefv60())
		ab8500_accdet_pdata.is_detection_inverted = true;
	else
		ab8500_accdet_pdata.is_detection_inverted = false;
}

static void __init mop500_init_machine(void)
{
	device_gpio_cfg();
	cw1200_gpio_cfg();
	accessory_detect_config();

#ifdef CONFIG_REGULATOR
	platform_device_register(&u8500_regulator_dev);
#endif

	u8500_init_devices();

#ifdef CONFIG_USB_ANDROID
	fetch_usb_serial_no(USB_SERIAL_NUMBER_LEN);
#endif

	platform_add_devices(platform_devs,
			ARRAY_SIZE(platform_devs));

	mop500_pins_init();

	platform_device_register(&ux500_ab8500_device);

	mop500_i2c_init();
	mop500_msp_init();
	mop500_spi_init();
	mop500_uart_init();

	db8500_add_ske_keypad(&mop500_ske_keypad_data);

#ifdef CONFIG_HSI
	hsi_register_board_info(u8500_hsi_devices,
				ARRAY_SIZE(u8500_hsi_devices));
#endif

#ifdef CONFIG_MOP500_NUIB
	mop500_nuib_init();
#endif

#ifdef CONFIG_ANDROID_STE_TIMED_VIBRA
	mop500_vibra_init();
#endif

	platform_add_devices(u8500_platform_devices,
			     ARRAY_SIZE(u8500_platform_devices));
}

MACHINE_START(U8500, "ST-Ericsson U8500 Platform")
	/* Maintainer: ST-Ericsson */
	.phys_io	= U8500_UART2_BASE,
	.io_pg_offst	= (IO_ADDRESS(U8500_UART2_BASE) >> 18) & 0xfffc,
	.boot_params	= 0x00000100,
	.map_io		= u8500_map_io,
	.init_irq	= ux500_init_irq,
	.timer		= &u8500_timer,
	.init_machine	= mop500_init_machine,
MACHINE_END

MACHINE_START(NOMADIK, "ST-Ericsson U8500 Platform")
	/* Maintainer: ST-Ericsson */
	.phys_io	= U8500_UART2_BASE,
	.io_pg_offst	= (IO_ADDRESS(U8500_UART2_BASE) >> 18) & 0xfffc,
	.boot_params	= 0x00000100,
	.map_io		= u8500_map_io,
	.init_irq	= ux500_init_irq,
	.timer		= &u8500_timer,
	.init_machine	= mop500_init_machine,
MACHINE_END

MACHINE_START(HREFV60, "ST-Ericsson U8500 Platform HREFv60+")
	/* Maintainer: ST-Ericsson */
	.phys_io	= U8500_UART2_BASE,
	.io_pg_offst	= (IO_ADDRESS(U8500_UART2_BASE) >> 18) & 0xfffc,
	.boot_params	= 0x00000100,
	.map_io		= u8500_map_io,
	.init_irq	= ux500_init_irq,
	.timer		= &u8500_timer,
	.init_machine	= mop500_init_machine,
MACHINE_END
