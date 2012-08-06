/*
 * Godin LCD panel driver.
 *
 * Author: Gareth Phillips  <gareth.phillips@samsung.com>
 *
 * Derived from kernel/drivers/video/display-ld9040.c
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

#include <linux/wait.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/lcd.h>
#include <linux/mutex.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <mach/prcmu.h>

#include <mach/board-sec-u8500.h>

#include <video/mcde_display.h>
#include <video/mcde_display_ssg_dpi.h>

#include <plat/pincfg.h>
#include <plat/gpio.h>
#include <mach/board-sec-u8500.h>
#include <mach/../../pins-db8500.h>
#include <mach/../../pins.h>

#define SLEEPMSEC	0x1000
#define ENDDEF		0x2000
#define	DEFMASK		0xFF00
#define COMMAND_ONLY	0xFE
#define DATA_ONLY	0xFF

#define LCD_POWER_UP	1
#define LCD_POWER_DOWN	0
#define LDI_STATE_ON	1
#define LDI_STATE_OFF	0

#define POWER_IS_ON(pwr)	((pwr) <= FB_BLANK_NORMAL)

#define ESD_PORT_NUM	93
#define ESD_OPERATION
/*
#define ESD_TEST
*/


/* Time between SCI probe or resume and panel initialisation sequence (to
     allow MCDE to complete initialisation and start DOTCLK).
*/
#define POWER_ON_WAIT_PERIOD_MS 2

/* to be removed when display works */
/* #define dev_dbg	dev_info */

extern unsigned int system_rev;

#ifdef CONFIG_HAS_EARLYSUSPEND
struct ux500_pins *dpi_pins;
#endif

struct gavini_lcd_driver {
	struct device				*dev;
	struct spi_device			*spi;
	struct mutex				lock;
	struct timer_list			timer;
	struct work_struct			work;
	unsigned int				beforepower;
	unsigned int				power;
	unsigned int				ldi_state;
	struct lcd_device			*ld;
	struct mcde_display_device		*mdd;
	struct ssg_dpi_display_platform_data	*pd;
	struct backlight_device			*bd;
	unsigned int				current_bl;
	unsigned int				goal_bl;

#ifdef ESD_OPERATION
	unsigned int				lcd_connected;
	unsigned int				esd_enable;
	unsigned int				esd_port;
	struct workqueue_struct			*esd_workqueue;
	struct work_struct			esd_work;
#ifdef ESD_TEST
	struct timer_list			esd_test_timer;
#endif
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend			earlysuspend;
#endif
};

struct gavini_spi_driver {
	struct spi_driver			base;
	struct gavini_lcd_driver		*lcd;
};

#ifdef ESD_OPERATION
#ifdef ESD_TEST
struct gavini_lcd_driver *pdpi;
#endif
#endif

#define LDI_ON_STATUS		1
#define LDI_OFF_STATUS		0

#define MIN_BRIGHTNESS		0
#define MAX_BRIGHTNESS		255
#define DEFAULT_BRIGHTNESS	120

#define MIN_SUPP_BRIGHTNESS	0
#define MAX_SUPP_BRIGHTNESS	10
#define MAX_REQ_BRIGHTNESS	255

#define DIM_BL			20
#define MIN_BL			30
#define MAX_BL			255
#define MAX_GAMMA_VALUE		25

static const unsigned short SET_DISPLAY_ON[] = {
	0x29,			COMMAND_ONLY,
	ENDDEF,		0x00
};

static const unsigned short SET_DISPLAY_OFF[] = {
	0x28,			COMMAND_ONLY,
	ENDDEF,		0x00
};

static const unsigned short ENTER_SLEEP_MODE[] = {
	0x10,			COMMAND_ONLY,
	ENDDEF,		0x00
};

static const unsigned short EXIT_SLEEP_MODE[] = {
	0x11,			COMMAND_ONLY,
	ENDDEF,		0x00
};

static const unsigned short SMD_FLIP_VERTICAL[] = {
	0x36,			COMMAND_ONLY,
	DATA_ONLY,			0x0A,
	ENDDEF,		0x00
};

static const unsigned short ACCESS_PROTECT_OFF[] = {
	0xB0,			COMMAND_ONLY,
	DATA_ONLY,			0x00,
	ENDDEF,		0x00
};

static const unsigned short SMD_PANEL_DRIVING[] = {
	0xC0,			COMMAND_ONLY,
	DATA_ONLY,			0x28,
	DATA_ONLY,			0x08,
	ENDDEF,		0x00
};

static const unsigned short SMD_SOURCE_CONTROL[] = {
	0xC1,			COMMAND_ONLY,
	DATA_ONLY,			0x01,
	DATA_ONLY,			0x30,
	DATA_ONLY,			0x15,
	DATA_ONLY,			0x05,
	DATA_ONLY,			0x22,
	ENDDEF,		0x00
};

static const unsigned short SMD_GATE_INTERFACE[] = {
	0xC4,			COMMAND_ONLY,
	DATA_ONLY,			0x10,
	DATA_ONLY,			0x01,
	DATA_ONLY,			0x00,
	ENDDEF,		0x00
};

static const unsigned short SMD_DISPLAY_H_TIMMING[] = {
	0xC5,			COMMAND_ONLY,
	DATA_ONLY,			0x06,
	DATA_ONLY,			0x55,
	DATA_ONLY,			0x03,
	DATA_ONLY,			0x07,
	DATA_ONLY,			0x0B,
	DATA_ONLY,			0x33,
	DATA_ONLY,			0x00,
	DATA_ONLY,			0x01,
	DATA_ONLY,			0x03,
	ENDDEF,		0x00
};

static const unsigned short SMD_RGB_SYNC_OPTION[] = {
	0xC6,			COMMAND_ONLY,
	DATA_ONLY,			0x01,
	ENDDEF,		0x00
};


static const unsigned short SMD_GAMMA_SET_RED[] = {
	0xC8,			COMMAND_ONLY,
	DATA_ONLY,			0x00,
	DATA_ONLY,			0x0A,
	DATA_ONLY,			0x31,
	DATA_ONLY,			0x3B,
	DATA_ONLY,			0x4E,
	DATA_ONLY,			0x58,
	DATA_ONLY,			0x59,
	DATA_ONLY,			0x5B,
	DATA_ONLY,			0x58,
	DATA_ONLY,			0x5E,
	DATA_ONLY,			0x62,
	DATA_ONLY,			0x60,
	DATA_ONLY,			0x61,
	DATA_ONLY,			0x5E,
	DATA_ONLY,			0x62,
	DATA_ONLY,			0x55,
	DATA_ONLY,			0x55,
	DATA_ONLY,			0x7F,
	DATA_ONLY,			0x08,
	DATA_ONLY,			0x00,
	DATA_ONLY,			0x0A,
	DATA_ONLY,			0x31,
	DATA_ONLY,			0x3B,
	DATA_ONLY,			0x4E,
	DATA_ONLY,			0x58,
	DATA_ONLY,			0x59,
	DATA_ONLY,			0x5B,
	DATA_ONLY,			0x58,
	DATA_ONLY,			0x5E,
	DATA_ONLY,			0x62,
	DATA_ONLY,			0x60,
	DATA_ONLY,			0x61,
	DATA_ONLY,			0x5E,
	DATA_ONLY,			0x62,
	DATA_ONLY,			0x55,
	DATA_ONLY,			0x55,
	DATA_ONLY,			0x7F,
	DATA_ONLY,			0x08,
	ENDDEF,		0x00
};

static const unsigned short SMD_GAMMA_SET_GREEN[] = {
	0xC9,			COMMAND_ONLY,
	DATA_ONLY,			0x00,
	DATA_ONLY,			0x25,
	DATA_ONLY,			0x15,
	DATA_ONLY,			0x28,
	DATA_ONLY,			0x3D,
	DATA_ONLY,			0x4A,
	DATA_ONLY,			0x48,
	DATA_ONLY,			0x4C,
	DATA_ONLY,			0x4A,
	DATA_ONLY,			0x52,
	DATA_ONLY,			0x59,
	DATA_ONLY,			0x59,
	DATA_ONLY,			0x5B,
	DATA_ONLY,			0x56,
	DATA_ONLY,			0x60,
	DATA_ONLY,			0x5D,
	DATA_ONLY,			0x55,
	DATA_ONLY,			0x7F,
	DATA_ONLY,			0x0A,
	DATA_ONLY,			0x00,
	DATA_ONLY,			0x25,
	DATA_ONLY,			0x15,
	DATA_ONLY,			0x28,
	DATA_ONLY,			0x3D,
	DATA_ONLY,			0x4A,
	DATA_ONLY,			0x48,
	DATA_ONLY,			0x4C,
	DATA_ONLY,			0x4A,
	DATA_ONLY,			0x52,
	DATA_ONLY,			0x59,
	DATA_ONLY,			0x59,
	DATA_ONLY,			0x5B,
	DATA_ONLY,			0x56,
	DATA_ONLY,			0x60,
	DATA_ONLY,			0x5D,
	DATA_ONLY,			0x55,
	DATA_ONLY,			0x7F,
	DATA_ONLY,			0x0A,
	ENDDEF,		0x00
};

static const unsigned short SMD_GAMMA_SET_BLUE[] = {
	0xCA,			COMMAND_ONLY,
	DATA_ONLY,			0x00,
	DATA_ONLY,			0x48,
	DATA_ONLY,			0x10,
	DATA_ONLY,			0x1F,
	DATA_ONLY,			0x2F,
	DATA_ONLY,			0x35,
	DATA_ONLY,			0x38,
	DATA_ONLY,			0x3D,
	DATA_ONLY,			0x3C,
	DATA_ONLY,			0x45,
	DATA_ONLY,			0x4D,
	DATA_ONLY,			0x4E,
	DATA_ONLY,			0x52,
	DATA_ONLY,			0x51,
	DATA_ONLY,			0x60,
	DATA_ONLY,			0x7F,
	DATA_ONLY,			0x7E,
	DATA_ONLY,			0x7F,
	DATA_ONLY,			0x0C,
	DATA_ONLY,			0x00,
	DATA_ONLY,			0x48,
	DATA_ONLY,			0x10,
	DATA_ONLY,			0x1F,
	DATA_ONLY,			0x2F,
	DATA_ONLY,			0x35,
	DATA_ONLY,			0x38,
	DATA_ONLY,			0x3D,
	DATA_ONLY,			0x3C,
	DATA_ONLY,			0x45,
	DATA_ONLY,			0x4D,
	DATA_ONLY,			0x4E,
	DATA_ONLY,			0x52,
	DATA_ONLY,			0x51,
	DATA_ONLY,			0x60,
	DATA_ONLY,			0x7F,
	DATA_ONLY,			0x7E,
	DATA_ONLY,			0x7F,
	DATA_ONLY,			0x0C,
	ENDDEF,		0x00
};

static const unsigned short SMD_BIAS_CURRENT_CTRL[] = {
	0xD1,			COMMAND_ONLY,
	DATA_ONLY,			0x33,
	DATA_ONLY,			0x13,
	ENDDEF,		0x00
};


static const unsigned short SMD_DDV_CTRL[] = {
	0xD2,			COMMAND_ONLY,
	DATA_ONLY,			0x11,
	DATA_ONLY,			0x00,
	DATA_ONLY,			0x00,
	ENDDEF,		0x00
};


static const unsigned short SMD_GAMMA_CTRL[] = {
	0xD3,			COMMAND_ONLY,
	DATA_ONLY,			0x50,
	DATA_ONLY,			0x50,
	ENDDEF,		0x00
};

static const unsigned short SMD_DCDC_CTRL[] = {
	0xD5,			COMMAND_ONLY,
	DATA_ONLY,			0x2F,
	DATA_ONLY,			0x11,
	DATA_ONLY,			0x1E,
	DATA_ONLY,			0x46,
	ENDDEF,		0x00
};

static const unsigned short SMD_VCL_CTRL[] = {
	0xD6,			COMMAND_ONLY,
	DATA_ONLY,			0x11,
	DATA_ONLY,			0x0A,
	ENDDEF,		0x00
};

static const unsigned short NVM_LOAD_SEQUENCE[] = {
	/*0xD4*/
	0xD4,			COMMAND_ONLY,
	DATA_ONLY,			0x52,
	DATA_ONLY,			0x5E,
	/*0xF8*/
	0xF8,			COMMAND_ONLY,
	DATA_ONLY,			0x01,
	DATA_ONLY,			0xF5,
	DATA_ONLY,			0xF2,
	DATA_ONLY,			0x71,
	DATA_ONLY,			0x44,
	/*0xFC*/
	0xFC,			COMMAND_ONLY,
	DATA_ONLY,			0x00,
	DATA_ONLY,			0x08,
	ENDDEF,		0x00
};

static const unsigned short CABC_TRUN_ON_SEQUENCE[] = {
	/*0XB4*/
	0xB4,			COMMAND_ONLY,
	DATA_ONLY,			0x0F,
	DATA_ONLY,			0x00,
	DATA_ONLY,			0x50,
	/*0xB5*/
	0xB5,			COMMAND_ONLY,
	DATA_ONLY,			0x80,
	/*0xB7*/
	0xB7,			COMMAND_ONLY,
	DATA_ONLY,			0x24,
	/*0xB8*/
	0xB8,			COMMAND_ONLY,
	DATA_ONLY,			0x01,
	ENDDEF,		0x00
};


static int gavini_power(struct gavini_lcd_driver *lcd, int power);
static int gavini_power_on(struct gavini_lcd_driver *lcd);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void gavini_mcde_panel_early_suspend(struct early_suspend \
								*earlysuspend);
static void gavini_mcde_panel_late_resume(struct early_suspend \
								*earlysuspend);
#endif

static void gavini_power_on_work_func(struct work_struct *work)
{
	struct gavini_lcd_driver *lcd = container_of(work, \
						struct gavini_lcd_driver, work);
	gavini_power_on(lcd);
}


static void gavini_power_on_timer_cb(unsigned long param)
{
	struct gavini_lcd_driver *lcd = (struct gavini_lcd_driver *)param;

	/* We are not allowed to sleep in this function, so we have to
	    delegate the true work to a work queue.
	 */
	schedule_work(&lcd->work);
}


static void gavini_start_power_on_timer(struct gavini_lcd_driver *lcd)
{
	INIT_WORK(&lcd->work, gavini_power_on_work_func);

	dev_dbg(lcd->dev, "Invoked %s\n", __func__);

	if (!timer_pending(&lcd->timer)) {

		init_timer(&lcd->timer);
		lcd->timer.expires = jiffies \
				+ msecs_to_jiffies(POWER_ON_WAIT_PERIOD_MS);

		lcd->timer.function = gavini_power_on_timer_cb;
		lcd->timer.data = (unsigned long)lcd;
		add_timer(&lcd->timer);
	}
}
static void gavini_print_vmode(struct mcde_video_mode *vmode)
{
	pr_debug("resolution: %dx%d\n", vmode->xres, vmode->yres);
	pr_debug("  pixclock: %d\n",    vmode->pixclock);
	pr_debug("       hbp: %d\n",    vmode->hbp);
	pr_debug("       hfp: %d\n",    vmode->hfp);
	pr_debug("       hsw: %d\n",    vmode->hsw);
	pr_debug("       vbp: %d\n",    vmode->vbp);
	pr_debug("       vfp: %d\n",    vmode->vfp);
	pr_debug("       vsw: %d\n",    vmode->vsw);
	pr_debug("interlaced: %s\n",    vmode->interlaced ? "true" : "false");
}

static int gavini_try_video_mode(
	struct mcde_display_device *ddev, struct mcde_video_mode *video_mode)
{
	struct ssg_dpi_display_platform_data *pd;
	int res = -EINVAL;

	if (ddev == NULL || video_mode == NULL) {
		dev_warn(&ddev->dev, "%s:ddev = NULL or video_mode = NULL\n",
			__func__);
		return res;
	}

	pd = ddev->dev.platform_data;

	#if 0
	if ((video_mode->xres == pd->video_mode.xres) &&
		(video_mode->yres == pd->video_mode.yres)) {

		video_mode->hsw		= pd->video_mode.hsw;
		video_mode->hbp		= pd->video_mode.hbp;
		video_mode->hfp		= pd->video_mode.hfp;
		video_mode->vsw		= pd->video_mode.vsw;
		video_mode->vbp		= pd->video_mode.vbp;
		video_mode->vfp		= pd->video_mode.vfp;
		video_mode->interlaced	= pd->video_mode.interlaced;
		video_mode->pixclock	= pd->video_mode.pixclock;
		res = 0;
	}
	#else
		video_mode->hsw		= pd->video_mode.hsw;
		video_mode->hbp		= pd->video_mode.hbp;
		video_mode->hfp		= pd->video_mode.hfp;
		video_mode->vsw		= pd->video_mode.vsw;
		video_mode->vbp		= pd->video_mode.vbp;
		video_mode->vfp		= pd->video_mode.vfp;
		video_mode->interlaced	= pd->video_mode.interlaced;
		video_mode->pixclock	= pd->video_mode.pixclock;
		res = 0;
	#endif

	if (res == 0)
		gavini_print_vmode(video_mode);
	else {
		dump_stack();
		dev_warn(&ddev->dev,\
			"%s:Failed to find video mode x=%d,\
			y=%d, pd->video_mode.xres:%d pd->video_mode.yres:%d\n",\
			__func__,
			video_mode->xres,
			video_mode->yres,
			pd->video_mode.xres,
			pd->video_mode.yres);
	}

	return res;

}

static int gavini_set_video_mode(
	struct mcde_display_device *ddev, struct mcde_video_mode *video_mode)
{
	struct ssg_dpi_display_platform_data *pd;
	struct mcde_video_mode channel_video_mode;
	static int video_mode_apply_during_boot = 1;
	struct gavini_lcd_driver *lcd = dev_get_drvdata(&ddev->dev);
	int res = -EINVAL;

	if (ddev == NULL || video_mode == NULL) {
		dev_warn(&ddev->dev, "%s:ddev = NULL or video_mode = NULL\n",
			__func__);
		goto out;
	}
	ddev->video_mode = *video_mode;
	gavini_print_vmode(video_mode);

	pd = ddev->dev.platform_data;

	if ((video_mode->xres == pd->video_mode.xres \
		&& video_mode->yres == pd->video_mode.yres) \
		||
		(video_mode->xres == pd->video_mode.yres \
		&& video_mode->yres == pd->video_mode.xres)) {

		/* TODO: set resolution dependent driver data here */
		/*driver_data->xxx = yyy;*/
		res = 0;
	}

	if (res < 0) {
		dev_warn(&ddev->dev, "%s:Failed to set video mode x=%d, y=%d\n",
			__func__, video_mode->xres, video_mode->yres);
		goto error;
	}

	/* TODO: set general driver data here */
	/*driver_data->xxx = yyy;*/

	channel_video_mode = ddev->video_mode;
	/* Dependant on if display should rotate or MCDE should rotate */
	if (ddev->rotation == MCDE_DISPLAY_ROT_90_CCW ||
		ddev->rotation == MCDE_DISPLAY_ROT_90_CW) {
		channel_video_mode.xres = ddev->native_x_res;
		channel_video_mode.yres = ddev->native_y_res;
	}
	res = mcde_chnl_set_video_mode(ddev->chnl_state, &channel_video_mode);

	if (res < 0) {
		dev_warn(&ddev->dev, "%s:Failed to set video mode on channel\n",
			__func__);

		goto error;
	}
	/* notify mcde display driver about updated video mode, excepted for
	 * the first update to preserve the splash screen and avoid a
	 * stop_flow() */
	if (video_mode_apply_during_boot && lcd->pd->platform_enabled) {
		ddev->update_flags |= UPDATE_FLAG_PIXEL_FORMAT;
		video_mode_apply_during_boot = 0;
	} else
		ddev->update_flags |= UPDATE_FLAG_VIDEO_MODE;
	return res;
out:
error:
	return res;
}


static int gavini_spi_write_words(struct gavini_lcd_driver *lcd,
	const u16 *buf, int len)
{
	struct spi_message msg;
	struct spi_transfer xfer = {
		.len		= 2 * len,
		.tx_buf		= buf,
	};

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	if (lcd->spi)
		return spi_sync(lcd->spi, &msg);
	else
		return -1;
}

static int gavini_panel_send_sequence(struct gavini_lcd_driver *lcd,
	const unsigned short *wbuf)
{
	int ret = 0, i = 0, j = 0;
	u16 temp[256] = {0,};

	mutex_lock(&lcd->lock);

	while ((wbuf[i] & DEFMASK) != ENDDEF) {
		if ((wbuf[i] & DEFMASK) != SLEEPMSEC) {
			if (wbuf[i] != DATA_ONLY)
				temp[j++] = wbuf[i];
			if (wbuf[i+1] != COMMAND_ONLY)
				temp[j++] = wbuf[i+1] | 0x100;
			i += 2;
		} else {
			if (j > 0)
				ret = gavini_spi_write_words(lcd, temp, j);

			msleep(wbuf[i+1]);
			j = 0;
		}
	}

	ret = gavini_spi_write_words(lcd, temp, j);

	mutex_unlock(&lcd->lock);

	return ret;
}

static const unsigned char pwm_duty_ratio[] = {
6, 6, 12, 18, 29, 41,
53, 64, 76, 88, 99,
111, 117, 123, 128, 134,
140, 146, 152, 158, 163,
169, 175, 181, 193, 204,
};
static int gavini_update_brightness(struct gavini_lcd_driver *lcd)
{
	static unsigned short CABC_SEQUENCE[] = {
		0xB5, COMMAND_ONLY,
		DATA_ONLY, 0x80,
		ENDDEF, 0x00
	};

	if (system_rev >= GAVINI_R0_0_C) {
		pr_info("%s system_rev : %d\n", __func__, system_rev);
		return 1;
	}
	CABC_SEQUENCE[3] = pwm_duty_ratio[lcd->goal_bl];
	gavini_panel_send_sequence(lcd, CABC_SEQUENCE);

	dev_info(lcd->dev, "current_bl=%d, goal_bl=%d\n",
				lcd->current_bl, lcd->goal_bl);

	lcd->current_bl = lcd->goal_bl;

	return 1;
}

static void gavini_bl_power(struct gavini_lcd_driver *lcd, int on)
{
	if (on)
		gpio_direction_output(lcd->pd->bl_en_gpio, 1);
	else
		gpio_direction_output(lcd->pd->bl_en_gpio, 0);

	dev_dbg(lcd->dev, "%s %s\n", __func__, on > 0 ? "on" : "off");
}

static int gavini_dpi_ldi_init(struct gavini_lcd_driver *lcd)
{
	int ret = 0;

	ret |= gavini_panel_send_sequence(lcd, SMD_FLIP_VERTICAL);
	ret |= gavini_panel_send_sequence(lcd, ACCESS_PROTECT_OFF);
	ret |= gavini_panel_send_sequence(lcd, SMD_PANEL_DRIVING);
	ret |= gavini_panel_send_sequence(lcd, SMD_SOURCE_CONTROL);
	ret |= gavini_panel_send_sequence(lcd, SMD_GATE_INTERFACE);
	ret |= gavini_panel_send_sequence(lcd, SMD_DISPLAY_H_TIMMING);
	ret |= gavini_panel_send_sequence(lcd, SMD_RGB_SYNC_OPTION);

	ret |= gavini_panel_send_sequence(lcd, SMD_GAMMA_SET_RED);
	ret |= gavini_panel_send_sequence(lcd, SMD_GAMMA_SET_GREEN);
	ret |= gavini_panel_send_sequence(lcd, SMD_GAMMA_SET_BLUE);
	ret |= gavini_panel_send_sequence(lcd, SMD_BIAS_CURRENT_CTRL);
	ret |= gavini_panel_send_sequence(lcd, SMD_DDV_CTRL);
	ret |= gavini_panel_send_sequence(lcd, SMD_GAMMA_CTRL);
	ret |= gavini_panel_send_sequence(lcd, SMD_DCDC_CTRL);
	ret |= gavini_panel_send_sequence(lcd, SMD_VCL_CTRL);

	ret |= gavini_panel_send_sequence(lcd, EXIT_SLEEP_MODE);
	if (lcd->pd->sleep_out_delay)
		msleep(lcd->pd->sleep_out_delay);

	/*NVM Load Sequence*/
	ret |= gavini_panel_send_sequence(lcd, NVM_LOAD_SEQUENCE);
	msleep(150);

	/*CABC TRUN ON SEQUENCE*/
	if (system_rev < GAVINI_R0_0_C)
		ret |= gavini_panel_send_sequence(lcd, CABC_TRUN_ON_SEQUENCE);

	return ret;
}

static int gavini_dpi_ldi_enable(struct gavini_lcd_driver *lcd)
{
	int ret = 0;

	ret |= gavini_panel_send_sequence(lcd, SET_DISPLAY_ON);

	if (!ret)
		lcd->ldi_state = LDI_ON_STATUS;

	dev_dbg(lcd->dev, "ldi power on successful\n");

	return ret;
}
static int gavini_power_on(struct gavini_lcd_driver *lcd)
{
	struct ssg_dpi_display_platform_data *pd = lcd->pd;
	int ret = 0;

	dev_dbg(lcd->dev, "Invoked %s\n", __func__);

	if (!pd) {
		dev_err(lcd->dev, "platform data is NULL.\n");
		goto err_exit;
	}

	if (!pd->power_on) {
		dev_err(lcd->dev, "power_on is NULL.\n");
		goto err_exit;
	} else {
		pd->power_on(pd, LCD_POWER_UP);
		if (pd->power_on_delay)
			msleep(pd->power_on_delay);
	}

	if (!pd->gpio_cfg_lateresume) {
		dev_err(lcd->dev, "gpio_cfg_lateresume is NULL.\n");
		goto err_exit;
	} else {
		pd->gpio_cfg_lateresume();
	}
	if (!pd->reset) {
		dev_err(lcd->dev, "reset is NULL.\n");
		goto err_exit;
	} else {
		pd->reset(pd);
		if (pd->reset_delay)
			msleep(pd->reset_delay);
	}

	/*dpi ldi init*/
	ret = gavini_dpi_ldi_init(lcd);

	if (ret) {
		dev_err(lcd->dev, "failed to initialize ldi.\n");
		goto err_exit;
	}
	dev_dbg(lcd->dev, "ldi init successful\n");

	/*dpi ldi enable*/
	ret = gavini_dpi_ldi_enable(lcd);

	if (ret) {
		dev_err(lcd->dev, "failed to enable ldi.\n");
		goto err_exit;
	}
	dev_dbg(lcd->dev, "ldi enable successful\n");

	if (system_rev < GAVINI_R0_0_C) {
		gavini_bl_power(lcd, 1);
		gavini_update_brightness(lcd);
	}

	return ret;

err_exit:
	return -EFAULT;
}


static int gavini_power_off(struct gavini_lcd_driver *lcd)
{
	int ret = 0;
	struct ssg_dpi_display_platform_data *pd = NULL;

	dev_dbg(lcd->dev, "Invoked %s\n", __func__);

	if (system_rev < GAVINI_R0_0_C)
		gavini_bl_power(lcd, 0);

	pd = lcd->pd;
	if (!pd) {
		dev_err(lcd->dev, "platform data is NULL.\n");
		return -EFAULT;
	}

	ret = gavini_panel_send_sequence(lcd, SET_DISPLAY_OFF);

	if (pd->display_off_delay)
		msleep(pd->display_off_delay);

	if (ret == 0)
		ret = gavini_panel_send_sequence(lcd, ENTER_SLEEP_MODE);

	if (ret) {
		dev_err(lcd->dev, "lcd setting failed.\n");
		return -EIO;
	}

	if (pd->sleep_in_delay)
		msleep(pd->sleep_in_delay);

	if (!pd->gpio_cfg_earlysuspend) {
		dev_err(lcd->dev, "gpio_cfg_earlysuspend is NULL.\n");
		return -EFAULT;
	} else
		pd->gpio_cfg_earlysuspend();

	if (!pd->power_on) {
		dev_err(lcd->dev, "power_on is NULL.\n");
		return -EFAULT;
	} else
		pd->power_on(pd, LCD_POWER_DOWN);

	lcd->ldi_state = LDI_OFF_STATUS;

	return 0;
}

#ifdef ESD_OPERATION
static void esd_work_func(struct work_struct *work)
{
	struct gavini_lcd_driver *lcd = container_of(work,
					struct gavini_lcd_driver, esd_work);

	msleep(500);
	pr_info("%s lcd->esd_enable:%d start\n", __func__, lcd->esd_enable);

	if ((lcd->esd_enable) && (!gpio_get_value(lcd->esd_port))) {
		pr_info("%s esd_port:%d\n", __func__,
				gpio_get_value(lcd->esd_port));
		gavini_power_off(lcd);
		gavini_power_on(lcd);
		msleep(500);

		/* HIGH is normal. On PBA esd_port coule be LOW */
		if (!gpio_get_value(lcd->esd_port)) {
			pr_info("%s esd_work_func re-armed\n", __func__);
			queue_work(lcd->esd_workqueue, &(lcd->esd_work));
		}
	}

	enable_irq(GPIO_TO_IRQ(lcd->esd_port));

	pr_info("%s end\n", __func__);
}

static irqreturn_t esd_interrupt_handler(int irq, void *data)
{
	struct gavini_lcd_driver *lcd = data;

	disable_irq_nosync(GPIO_TO_IRQ(lcd->esd_port));

	pr_info("%s lcd->esd_enable :%d\n", __func__, lcd->esd_enable);

	if (lcd->esd_enable) {
		if (list_empty(&(lcd->esd_work.entry))) {
			pr_info("%s esd_work_func queuing\n", __func__);
			queue_work(lcd->esd_workqueue, &(lcd->esd_work));
		} else
			pr_info("%s esd_work_func is not empty\n", __func__);
	} else
		enable_irq(GPIO_TO_IRQ(lcd->esd_port));

	return IRQ_HANDLED;
}
#endif

static int gavini_power(struct gavini_lcd_driver *lcd, int power)
{
	int ret = 0;

	dev_dbg(lcd->dev, "Invoked %s\n", __func__);
	pr_info("%s power:%d\n", __func__, power);

	dev_dbg(lcd->dev, "%s(): old=%d (%s), new=%d (%s)\n", __func__,
		lcd->power, POWER_IS_ON(lcd->power) ? "on" : "off",
		power, POWER_IS_ON(power) ? "on" : "off"
		);

	if (POWER_IS_ON(power) && !POWER_IS_ON(lcd->power))
		ret = gavini_power_on(lcd);
	else if (!POWER_IS_ON(power) && POWER_IS_ON(lcd->power))
		ret = gavini_power_off(lcd);

	if (!ret)
		lcd->power = power;
	return ret;
}


static int __init gavini_spi_probe(struct spi_device *spi);

static struct gavini_spi_driver gavini_spi_drv = {
	.base = {
		.driver = {
			.name	= "pri_lcd_spi",
			.bus	= &spi_bus_type,
			.owner	= THIS_MODULE,
		},
		.probe		= gavini_spi_probe,
	},
	.lcd = NULL,
};

#ifdef ESD_OPERATION
#ifdef ESD_TEST
static void est_test_timer_func(unsigned long data)
{
	pr_info("%s\n", __func__);

	if (list_empty(&(pdpi->esd_work.entry))) {
		disable_irq_nosync(GPIO_TO_IRQ(pdpi->esd_port));
		queue_work(pdpi->esd_workqueue, &(pdpi->esd_work));
		pr_info("%s invoked\n", __func__);
	}

	mod_timer(&pdpi->esd_test_timer,  jiffies + (3*HZ));
}
#endif
#endif

static int __init gavini_spi_probe(struct spi_device *spi)
{
	int ret = 0;
	struct gavini_lcd_driver *lcd = gavini_spi_drv.lcd;

	dev_dbg(&spi->dev, "panel gavini spi being probed\n");

	dev_set_drvdata(&spi->dev, lcd);

	/* gavini lcd panel uses MPI DBI Type C 9 bit (Option 1). */
	spi->bits_per_word = 9;

	ret = spi_setup(spi);
	if (ret < 0) {
		dev_err(&spi->dev, "spi setup failed.\n");
		goto out;
	}

	lcd->spi = spi;

	/*
	 * if lcd panel was on from bootloader like u-boot then
	 * do not lcd on.
	 */

	if (!lcd->pd->platform_enabled) {
		/*
		 * if lcd panel was off from bootloader then
		 * current lcd status is powerdown and then
		 * it enables lcd panel.
		 */
		lcd->power = FB_BLANK_POWERDOWN;

		gavini_power(lcd, FB_BLANK_UNBLANK);
	} else
		lcd->power = FB_BLANK_UNBLANK;

#ifdef ESD_OPERATION
	lcd->esd_workqueue = create_singlethread_workqueue("esd_workqueue");

	if (!lcd->esd_workqueue) {
		dev_info(lcd->dev, "esd_workqueue create fail\n");
		return -ENOMEM;
	}

	INIT_WORK(&(lcd->esd_work), esd_work_func);

	lcd->esd_port = ESD_PORT_NUM;

	if (request_threaded_irq(GPIO_TO_IRQ(lcd->esd_port), NULL,
	esd_interrupt_handler, IRQF_TRIGGER_FALLING, "esd_interrupt", lcd)) {
			dev_info(lcd->dev, "esd irq request fail\n");
			free_irq(GPIO_TO_IRQ(lcd->esd_port), NULL);
			lcd->lcd_connected = 0;
	} else {
		/* HIGH is normal. On PBA esd_port coule be LOW */
		if (gpio_get_value(lcd->esd_port)) {
			dev_info(lcd->dev, "esd irq enabled on booting\n");
			lcd->esd_enable = 1;
			lcd->lcd_connected = 1;
		} else {
			dev_info(lcd->dev, "esd irq disabled on booting\n");
			disable_irq(GPIO_TO_IRQ(lcd->esd_port));
			lcd->esd_enable = 0;
			lcd->lcd_connected = 0;
		}
	}

	dev_info(lcd->dev, "%s esd work success\n");

#ifdef ESD_TEST
		pdpi = lcd;
		setup_timer(&lcd->esd_test_timer, est_test_timer_func, 0);
		mod_timer(&lcd->esd_test_timer,  jiffies + (3*HZ));
#endif
#endif

	dev_dbg(&spi->dev, "gavini spi has been probed.\n");

out:
	return ret;
}

int gavini_get_brightness(struct backlight_device *bd)
{
	dev_dbg(&bd->dev, "lcd get brightness returns %d\n",
		bd->props.brightness);
	return bd->props.brightness;
}

static int get_gamma_value_from_bl(int bl)
{
	int gamma_value = 0;
	int gamma_val_x10 = 0;

	if (bl >= MIN_BL) {
		gamma_val_x10 = 10 * (MAX_GAMMA_VALUE - 1)*bl/(MAX_BL - MIN_BL)
		+ (10 - 10*(MAX_GAMMA_VALUE-1)*(MIN_BL)/(MAX_BL-MIN_BL));

		gamma_value = (gamma_val_x10  + 5)/10;
	} else
		gamma_value = 0;

	return gamma_value;
}

static int gavini_set_brightness(struct backlight_device *bd)
{
	int ret = 0, bl = bd->props.brightness;
	struct gavini_lcd_driver *lcd = bl_get_data(bd);

	if (bl < MIN_SUPP_BRIGHTNESS ||
		bl > bd->props.max_brightness) {
		dev_err(&bd->dev, "lcd brightness should be %d to %d.\n",
			MIN_SUPP_BRIGHTNESS, bd->props.max_brightness);
		return -EINVAL;
	}

	lcd->goal_bl = get_gamma_value_from_bl(bl);

	if ((lcd->ldi_state) && (lcd->current_bl != lcd->goal_bl)) {
		ret = gavini_update_brightness(lcd);
		if (ret < 0)
			dev_err(&bd->dev, "update brightness failed.\n");
	}

	return ret;
}


/* This structure defines all the properties of a backlight */
struct backlight_properties gavini_backlight_props = {
	.brightness = MAX_REQ_BRIGHTNESS,
	.max_brightness = MAX_REQ_BRIGHTNESS,
};


const struct backlight_ops gavini_backlight_ops = {
	.get_brightness = gavini_get_brightness,
	.update_status = gavini_set_brightness,
};


static ssize_t panel_type_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
		return sprintf(buf, "LMS397KF04\n");
}
static DEVICE_ATTR(panel_type, 0444, panel_type_show, NULL);

static int gavini_set_power(struct lcd_device *ld, int power)
{
	struct gavini_lcd_driver *lcd = lcd_get_data(ld);

	if (power != FB_BLANK_UNBLANK && power != FB_BLANK_POWERDOWN
		&&	power != FB_BLANK_NORMAL) {
		dev_err(lcd->dev, "power value should be 0, 1 or 4.\n");
		return -EINVAL;
	}

	return gavini_power(lcd, power);
}

static int gavini_get_power(struct lcd_device *ld)
{
	struct gavini_lcd_driver *lcd = lcd_get_data(ld);

	return lcd->power;
}

static struct lcd_ops gavini_lcd_ops = {
	.set_power = gavini_set_power,
	.get_power = gavini_get_power,
};


static int gavini_mcde_panel_probe(struct mcde_display_device *ddev)
{
	int ret = 0;
	struct gavini_lcd_driver *lcd = NULL;
	struct backlight_device *bd = NULL;
	struct ssg_dpi_display_platform_data *pdata = ddev->dev.platform_data;

	dev_dbg(&ddev->dev, "Invoked %s\n", __func__);

	if (pdata == NULL) {
		dev_err(&ddev->dev, "%s:Platform data missing\n", __func__);
		ret = -EINVAL;
		goto no_pdata;
	}

	if (ddev->port->type != MCDE_PORTTYPE_DPI) {
		dev_err(&ddev->dev,
			"%s:Invalid port type %d\n",
			__func__, ddev->port->type);
		ret = -EINVAL;
		goto invalid_port_type;
	}

	ddev->prepare_for_update = NULL;
	ddev->try_video_mode = gavini_try_video_mode;
	ddev->set_video_mode = gavini_set_video_mode;


	lcd = kzalloc(sizeof(struct gavini_lcd_driver), GFP_KERNEL);
	if (!lcd)
		return -ENOMEM;
	dev_set_drvdata(&ddev->dev, lcd);
	lcd->mdd = ddev;
	lcd->dev = &ddev->dev;
	lcd->pd = pdata;

	mutex_init(&lcd->lock);

	gavini_spi_drv.lcd = lcd;
	ret = spi_register_driver((struct spi_driver *) &gavini_spi_drv);
	if (ret < 0) {
		dev_err(&(ddev->dev), "Failed to register SPI driver");
		goto out;
	}

#ifdef CONFIG_LCD_CLASS_DEVICE
	lcd->ld = lcd_device_register("panel", &ddev->dev,
					lcd, &gavini_lcd_ops);
	if (IS_ERR(lcd->ld)) {
		ret = PTR_ERR(lcd->ld);
	} else {
		ret = device_create_file(&(lcd->ld->dev), &dev_attr_panel_type);
		if (ret < 0)
			dev_err(&(lcd->ld->dev),
			"failed to add panel_type sysfs entries\n");
	}
#endif

	if (system_rev < GAVINI_R0_0_C) {
		bd = backlight_device_register("pwm-backlight",
						&ddev->dev,
						lcd,
						&gavini_backlight_ops,
						&gavini_backlight_props);
		if (IS_ERR(bd)) {
			ret =  PTR_ERR(bd);
			goto out_backlight_unregister;
		}

		lcd->bd = bd;
		lcd->bd->props.max_brightness = MAX_BRIGHTNESS;
		lcd->bd->props.brightness = DEFAULT_BRIGHTNESS;
		lcd->current_bl = 0;
		lcd->goal_bl = DEFAULT_BRIGHTNESS;
		lcd->ldi_state = LDI_ON_STATUS;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	lcd->earlysuspend.level   = EARLY_SUSPEND_LEVEL_DISABLE_FB - 2;
	lcd->earlysuspend.suspend = gavini_mcde_panel_early_suspend;
	lcd->earlysuspend.resume  = gavini_mcde_panel_late_resume;
	register_early_suspend(&lcd->earlysuspend);
#endif
#if 0
	if (prcmu_qos_add_requirement(PRCMU_QOS_APE_OPP,
			"gavini_lcd_dpi", 100)) {
		pr_info("pcrm_qos_add APE failed\n");
	}
#endif
	dev_dbg(&ddev->dev, "DPI display probed\n");

	goto out;

out_backlight_unregister:
	if (system_rev < GAVINI_R0_0_C)
		backlight_device_unregister(bd);
invalid_port_type:
no_pdata:
out:
	return ret;
}

static int gavini_mcde_panel_resume(struct mcde_display_device *ddev)
{
	int ret = 0;
	struct gavini_lcd_driver *lcd = dev_get_drvdata(&ddev->dev);

	dev_dbg(&ddev->dev, "Invoked %s\n", __func__);

	/* set_power_mode will handle call platform_enable */
	ret = ddev->set_power_mode(ddev, MCDE_DISPLAY_PM_STANDBY);
	if (ret < 0)
		dev_warn(&ddev->dev, "%s:Failed to resume display\n"
			, __func__);

	/*
	 * after suspended, if lcd panel status is FB_BLANK_UNBLANK
	 * (at that time, power is FB_BLANK_UNBLANK) then
	 * it changes that status to FB_BLANK_POWERDOWN to get lcd on.
	 */
	if (lcd->beforepower == FB_BLANK_UNBLANK)
		lcd->power = FB_BLANK_POWERDOWN;

	dev_dbg(&ddev->dev, "power = %d\n", lcd->beforepower);

	ret = gavini_power(lcd, lcd->beforepower);

	return ret;
}

static int gavini_mcde_panel_suspend(struct mcde_display_device *ddev,
							pm_message_t state)
{
	int ret = 0;
	struct gavini_lcd_driver *lcd = dev_get_drvdata(&ddev->dev);

	pr_info("%s\n", __func__);
	dev_dbg(&ddev->dev, "Invoked %s\n", __func__);

	lcd->beforepower = lcd->power;
	/*
	 * when lcd panel is suspend, lcd panel becomes off
	 * regardless of status.
	 */
	ret = gavini_power(lcd, FB_BLANK_POWERDOWN);

	/* set_power_mode will handle call platform_disable */
	ret = ddev->set_power_mode(ddev, MCDE_DISPLAY_PM_OFF);
	if (ret < 0)
		dev_warn(&ddev->dev, "%s:Failed to suspend display\n"
			, __func__);

	return ret;
}


#ifdef CONFIG_HAS_EARLYSUSPEND

static pin_cfg_t gavini_sleep_pins[] = {
	GPIO150_GPIO | PIN_OUTPUT_LOW,
};

static pin_cfg_t gavini_resume_pins[] = {
	GPIO150_LCDA_CLK,
};

static int dpi_display_platform_enable(struct gavini_lcd_driver *lcd)
{
	int res = 0;
	dev_info(lcd->dev, "%s\n", __func__);
	nmk_config_pins(gavini_resume_pins, ARRAY_SIZE(gavini_resume_pins));
	res = ux500_pins_enable(dpi_pins);
	if (res)
		dev_warn(lcd->dev, "Failure during %s\n", __func__);
	return res;
}

static int dpi_display_platform_disable(struct gavini_lcd_driver *lcd)
{
	int res = 0;
	dev_info(lcd->dev, "%s\n", __func__);
	nmk_config_pins(gavini_sleep_pins, ARRAY_SIZE(gavini_sleep_pins));

	/* pins disabled to save power */
	res = ux500_pins_disable(dpi_pins);
	if (res)
		dev_warn(lcd->dev, "Failure during %s\n", __func__);
	return res;
}

static void gavini_mcde_panel_early_suspend(struct early_suspend \
								*earlysuspend)
{
	struct gavini_lcd_driver *lcd = container_of(earlysuspend, \
					struct gavini_lcd_driver, earlysuspend);
	pm_message_t dummy;
	pr_info("%s\n", __func__);

	#ifdef ESD_OPERATION
	if (lcd->esd_enable) {

		lcd->esd_enable = 0;
		set_irq_type(GPIO_TO_IRQ(lcd->esd_port), IRQF_TRIGGER_NONE);
		disable_irq_nosync(GPIO_TO_IRQ(lcd->esd_port));

		if (!list_empty(&(lcd->esd_work.entry))) {
			cancel_work_sync(&(lcd->esd_work));
			pr_info("%s cancel_work_sync\n", __func__);
		}

		pr_info("%s change lcd->esd_enable :%d\n", __func__,
				lcd->esd_enable);
	}
	#endif

	gavini_mcde_panel_suspend(lcd->mdd, dummy);
	dpi_display_platform_disable(lcd);
#if 0
	prcmu_qos_remove_requirement(PRCMU_QOS_APE_OPP,
				"gavini_lcd_dpi");
#endif
}

static void gavini_mcde_panel_late_resume(struct early_suspend \
								*earlysuspend)
{
	struct gavini_lcd_driver *lcd = container_of(earlysuspend, \
					struct gavini_lcd_driver, earlysuspend);

	pr_info("%s\n", __func__);

	#ifdef ESD_OPERATION
	if (lcd->lcd_connected)
		enable_irq(GPIO_TO_IRQ(lcd->esd_port));
	#endif
#if 0
	if (prcmu_qos_add_requirement(PRCMU_QOS_APE_OPP,
			"gavini_lcd_dpi", 100)) {
		pr_info("pcrm_qos_add APE failed\n");
	}
#endif
	dpi_display_platform_enable(lcd);
	gavini_mcde_panel_resume(lcd->mdd);

	#ifdef ESD_OPERATION
	if (lcd->lcd_connected) {
		set_irq_type(GPIO_TO_IRQ(lcd->esd_port), IRQF_TRIGGER_FALLING);
		lcd->esd_enable = 1;
		pr_info("%s change lcd->esd_enable :%d\n", __func__,
				lcd->esd_enable);
	} else
		pr_info("%s lcd_connected:%d\n", __func__, lcd->lcd_connected);
	#endif

}


#endif

static int gavini_mcde_panel_remove(struct mcde_display_device \
									*ddev)
{
	struct gavini_lcd_driver *lcd = dev_get_drvdata(&ddev->dev);

	dev_dbg(&ddev->dev, "Invoked %s\n", __func__);
	gavini_power(lcd, FB_BLANK_POWERDOWN);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&lcd->earlysuspend);
#endif

	spi_unregister_driver((struct spi_driver *)&gavini_spi_drv);
	kfree(lcd);

	return 0;
}

static void gavini_mcde_panel_shutdown(struct mcde_display_device \
									*ddev)
{
	struct gavini_lcd_driver *lcd = dev_get_drvdata(&ddev->dev);

	pr_info("%s shutdown", __func__);
	dev_dbg(&ddev->dev, "Invoked %s\n", __func__);

	#ifdef ESD_OPERATION
	if (lcd->esd_enable) {

		lcd->esd_enable = 0;
		disable_irq_nosync(GPIO_TO_IRQ(lcd->esd_port));

		if (!list_empty(&(lcd->esd_work.entry))) {
			cancel_work_sync(&(lcd->esd_work));
			pr_info("%s cancel_work_sync", __func__);
		}

		destroy_workqueue(lcd->esd_workqueue);
	}
	#endif

	gavini_power(lcd, FB_BLANK_POWERDOWN);


	spi_unregister_driver((struct spi_driver *)&gavini_spi_drv);

	#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&lcd->earlysuspend);
	#endif

	kfree(lcd);

	dev_dbg(&ddev->dev, "end %s\n", __func__);
}


static struct mcde_display_driver gavini_mcde = {
	.probe          = gavini_mcde_panel_probe,
	.remove         = gavini_mcde_panel_remove,
	.shutdown       = gavini_mcde_panel_shutdown,
#ifdef CONFIG_HAS_EARLYSUSPEND
	.suspend        = NULL,
	.resume         = NULL,
#else
	.suspend		= gavini_mcde_panel_suspend,
	.resume		= gavini_mcde_panel_resume,
#endif
	.driver		= {
		.name	= LCD_DRIVER_NAME_GAVINI,
		.owner	= THIS_MODULE,
	}
};


static int __init gavini_init(void)
{
	int ret = 0;
	ret =  mcde_display_driver_register(&gavini_mcde);

	#ifdef CONFIG_HAS_EARLYSUSPEND
	dpi_pins = ux500_pins_get("mcde-dpi");
	if (!dpi_pins)
		return -EINVAL;
	#endif

	return ret;
}

static void __exit gavini_exit(void)
{
	mcde_display_driver_unregister(&gavini_mcde);
}

module_init(gavini_init);
module_exit(gavini_exit);

MODULE_AUTHOR("Gareth Phillips <gareth.phillips@samsung.com>");
MODULE_DESCRIPTION("gavini LCD Driver");
MODULE_LICENSE("GPL");

