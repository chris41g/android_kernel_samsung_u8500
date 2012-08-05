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

#include <video/mcde_display.h>
#include <video/mcde_display_ssg_dpi.h>

#define SLEEPMSEC		0x1000
#define ENDDEF			0x2000
#define	DEFMASK			0xFF00
#define COMMAND_ONLY	0xFE
#define DATA_ONLY		0xFF

#define LCD_POWER_UP		1
#define LCD_POWER_DOWN		0
#define LDI_STATE_ON		1
#define LDI_STATE_OFF		0

#define POWER_IS_ON(pwr)	((pwr) <= FB_BLANK_NORMAL)


/* Time between SCI probe or resume and panel initialisation sequence (to 
     allow MCDE to complete initialisation and start DOTCLK).
*/
#define POWER_ON_WAIT_PERIOD_MS 2

/* to be removed when display works */
//#define dev_dbg	dev_info

extern unsigned int system_rev;

struct godin_lcd_driver {
	struct device			*dev;
	struct spi_device		*spi;
	struct mutex		    lock;
	struct timer_list	    timer;
	struct work_struct	    work;
	unsigned int 			beforepower;
	unsigned int			power;
	unsigned int 			ldi_state;
	struct mcde_display_device 	*mdd;
	struct ssg_dpi_display_platform_data		*pd;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend    earlysuspend;
#endif
};

struct godin_spi_driver {
		struct spi_driver	base;
		struct godin_lcd_driver		* lcd;
};


static const unsigned short SET_DISPLAY_ON[] =
{
    0x29,       COMMAND_ONLY,
    ENDDEF,     0x00
};

static const unsigned short SET_DISPLAY_OFF[] =
{
    0x28,       COMMAND_ONLY,
    ENDDEF,     0x00
};

static const unsigned short ENTER_SLEEP_MODE[] =
{
    0x10,       COMMAND_ONLY,
    ENDDEF,     0x00
};

static const unsigned short EXIT_SLEEP_MODE[] =
{
    0x11,       COMMAND_ONLY,
    ENDDEF,     0x00
};

static int godin_power(struct godin_lcd_driver *lcd, int power);
static void godin_power_on(struct godin_lcd_driver *lcd);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void godin_mcde_panel_early_suspend(struct early_suspend *earlysuspend);
static void godin_mcde_panel_late_resume(struct early_suspend *earlysuspend);
#endif

static void godin_power_on_work_func(struct work_struct* work)
{
	struct godin_lcd_driver *lcd = container_of(work, struct godin_lcd_driver, work);
	godin_power_on(lcd);
}


static void godin_power_on_timer_cb(unsigned long param)
{
	struct godin_lcd_driver *lcd = (struct godin_lcd_driver *)param;

	/* We are not allowed to sleep in this function, so we have to
	    delegate the true work to a work queue.
	 */
	schedule_work(&lcd->work);
}


static void godin_start_power_on_timer(struct godin_lcd_driver *lcd)
{
	INIT_WORK(&lcd->work,godin_power_on_work_func);

	dev_dbg(lcd->dev, "Invoked %s\n", __func__);

	if (!timer_pending(&lcd->timer)) 
	{
		init_timer(&lcd->timer);
		lcd->timer.expires = jiffies + msecs_to_jiffies(POWER_ON_WAIT_PERIOD_MS);
		lcd->timer.function = godin_power_on_timer_cb;
		lcd->timer.data=(unsigned long)lcd;
		add_timer(&lcd->timer);
	}
}
 

static void godin_print_vmode(struct mcde_video_mode *vmode)
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

static int godin_try_video_mode(
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

	if ((video_mode->xres == pd->video_mode.xres) &&
		(video_mode->yres == pd->video_mode.yres)) {
		
		video_mode->hsw  		= pd->video_mode.hsw;
		video_mode->hbp  		= pd->video_mode.hbp;
		video_mode->hfp  		= pd->video_mode.hfp;
		video_mode->vsw  		= pd->video_mode.vsw;
		video_mode->vbp 		= pd->video_mode.vbp;
		video_mode->vfp 		= pd->video_mode.vfp;
		video_mode->interlaced 	= pd->video_mode.interlaced;
		video_mode->pixclock 	= pd->video_mode.pixclock;
		res = 0;
	}

	if (res == 0)
		godin_print_vmode(video_mode);
	else
		dev_warn(&ddev->dev,
			"%s:Failed to find video mode x=%d, y=%d\n",
			__func__, video_mode->xres, video_mode->yres);

	return res;

}

static int godin_set_video_mode(
	struct mcde_display_device *ddev, struct mcde_video_mode *video_mode)
{
	struct ssg_dpi_display_platform_data *pd;
	struct mcde_video_mode channel_video_mode;
	static int video_mode_apply_during_boot = 1;
	struct godin_lcd_driver *lcd = dev_get_drvdata(&ddev->dev);
	int res = -EINVAL;

	if (ddev == NULL || video_mode == NULL) {
		dev_warn(&ddev->dev, "%s:ddev = NULL or video_mode = NULL\n",
			__func__);
		goto out;
	}
	ddev->video_mode = *video_mode;
	godin_print_vmode(video_mode);

	pd = ddev->dev.platform_data;

	if ((video_mode->xres == pd->video_mode.xres && video_mode->yres == pd->video_mode.yres) ||
		(video_mode->xres == pd->video_mode.yres && video_mode->yres == pd->video_mode.xres)) {

		/* TODO: set resolution dependent driver data here */
		//driver_data->xxx = yyy;
		res = 0;
	}

	if (res < 0) {
		dev_warn(&ddev->dev, "%s:Failed to set video mode x=%d, y=%d\n",
			__func__, video_mode->xres, video_mode->yres);
		goto error;
	}

	/* TODO: set general driver data here */
	//driver_data->xxx = yyy;

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


static int godin_spi_write_words(struct godin_lcd_driver *lcd,
	const u16 *buf, int len)
{
	struct spi_message msg;
	struct spi_transfer xfer = {
		.len		= 2 * len,
		.tx_buf		= buf,
	};

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	if(lcd->spi){
		return spi_sync(lcd->spi, &msg);
	}else{
		return -1;
	}
}

static int godin_panel_send_sequence(struct godin_lcd_driver *lcd,
	const unsigned short *wbuf)
{
	int ret = 0, i = 0, j = 0;
	u16 temp[256];

	while ((wbuf[i] & DEFMASK) != ENDDEF) {
		if ((wbuf[i] & DEFMASK) != SLEEPMSEC) {
			if (wbuf[i] != DATA_ONLY)
				temp[j++] = wbuf[i];
			if (wbuf[i+1] != COMMAND_ONLY)
				temp[j++] = wbuf[i+1] | 0x100;
			i += 2;
		} else {
			if (j > 0 )
				ret = godin_spi_write_words(lcd, temp, j);
			udelay(wbuf[i+1]*1000);
			j = 0;
		}
	}

	ret = godin_spi_write_words(lcd, temp, j);

	return ret;
}


static void godin_power_on(struct godin_lcd_driver *lcd)
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

	if (!pd->reset) {
		dev_err(lcd->dev, "reset is NULL.\n");
		goto err_exit;
	} else {
		pd->reset(pd);
		if (pd->reset_delay)
			msleep(pd->reset_delay);
	}

	if (!ret)
	{
		ret = godin_panel_send_sequence(lcd, EXIT_SLEEP_MODE);

		if (pd->sleep_out_delay)
			msleep(pd->sleep_out_delay);
	}

	if (!ret)
	{
		ret = godin_panel_send_sequence(lcd, SET_DISPLAY_ON);
	}

	if (ret) {
		dev_err(lcd->dev, "ldi power on failed\n");
		goto err_exit;
	}

	/* set_power_mode will handle call platform_disable */
	ret = lcd->mdd->set_power_mode(lcd->mdd, MCDE_DISPLAY_PM_STANDBY);
	if (ret < 0)
		dev_warn(&lcd->mdd->dev, "%s:Failed to resume display\n"
			, __func__);

	dev_dbg(lcd->dev, "ldi power on successful\n");

err_exit:
	mutex_unlock(&lcd->lock);
}


static int godin_power_off(struct godin_lcd_driver *lcd)
{
	int ret = 0;
	struct ssg_dpi_display_platform_data *pd = NULL;

	dev_dbg(lcd->dev, "Invoked %s\n", __func__);

	pd = lcd->pd;
	if (!pd) {
		dev_err(lcd->dev, "platform data is NULL.\n");
		return -EFAULT;
	}

	ret = godin_panel_send_sequence(lcd, SET_DISPLAY_OFF);

	if (pd->display_off_delay)
		msleep(pd->display_off_delay);
	
	if (ret == 0)
	    ret = godin_panel_send_sequence(lcd, ENTER_SLEEP_MODE);
	
	if (ret) {
		dev_err(lcd->dev, "lcd setting failed.\n");
		return -EIO;
	}

	if (pd->sleep_in_delay)
		msleep(pd->sleep_in_delay);

	if (!pd->power_on) {
		dev_err(lcd->dev, "power_on is NULL.\n");
		return -EFAULT;
	} else
		pd->power_on(pd, LCD_POWER_DOWN);

	return 0;
}


static int godin_power(struct godin_lcd_driver *lcd, int power)
{
	int ret = 0;

	dev_dbg(lcd->dev, "Invoked %s\n", __func__);

	mutex_lock(&lcd->lock);

	dev_dbg(lcd->dev, "%s(): old=%d (%s), new=%d (%s)\n", __func__,
		lcd->power, POWER_IS_ON(lcd->power)? "on": "off",
		power, POWER_IS_ON(power)? "on": "off"
		);

	if (POWER_IS_ON(power) && !POWER_IS_ON(lcd->power))
		godin_start_power_on_timer(lcd);
	else if (!POWER_IS_ON(power) && POWER_IS_ON(lcd->power))
	{
		ret = godin_power_off(lcd);
		mutex_unlock(&lcd->lock);
	}

	if (!ret)
		lcd->power = power;
	return ret;
}


static int __init godin_spi_probe(struct spi_device *spi);

static struct godin_spi_driver godin_spi_drv = {
	.base = {
		.driver = {
			.name	= "pri_lcd_spi",
			.bus	= &spi_bus_type,
			.owner	= THIS_MODULE,
		},
		.probe		= godin_spi_probe,
	},
	.lcd = NULL,
};

static int __init godin_spi_probe(struct spi_device *spi)
{
	int ret = 0;
	struct godin_lcd_driver *lcd = godin_spi_drv.lcd;
	
	dev_dbg(&spi->dev, "panel godin spi being probed\n");

	dev_set_drvdata(&spi->dev, lcd);

	/* godin lcd panel uses MPI DBI Type C 9 bit (Option 1). */
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

		godin_power(lcd, FB_BLANK_UNBLANK);
	} else
		lcd->power = FB_BLANK_UNBLANK;

	dev_dbg(&spi->dev, "godin spi has been probed.\n");

out:
	return ret;
}

static int godin_mcde_panel_probe(struct mcde_display_device *ddev)
{
	int ret = 0;
	struct godin_lcd_driver *lcd = NULL;
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
    ddev->try_video_mode = godin_try_video_mode;
	ddev->set_video_mode = godin_set_video_mode;


	lcd = kzalloc(sizeof(struct godin_lcd_driver), GFP_KERNEL);
	if (!lcd)
		return -ENOMEM;
	dev_set_drvdata(&ddev->dev, lcd);
	lcd->mdd = ddev;
	lcd->dev = &ddev->dev;
	lcd->pd = pdata;

	mutex_init(&lcd->lock);

	godin_spi_drv.lcd = lcd;
	ret = spi_register_driver((struct spi_driver *)& godin_spi_drv);
	if (ret < 0) {
		dev_err(&(ddev->dev), "Failed to register SPI driver");
        goto out;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	lcd->earlysuspend.level   = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	lcd->earlysuspend.suspend = godin_mcde_panel_early_suspend;
	lcd->earlysuspend.resume  = godin_mcde_panel_late_resume;
	register_early_suspend(&lcd->earlysuspend);
#endif

	dev_dbg(&ddev->dev, "DPI display probed\n");

	goto out;

invalid_port_type:
no_pdata:
out:
	return ret;
}

static int godin_mcde_panel_resume(struct mcde_display_device *ddev)
{
	int ret = 0;
	struct godin_lcd_driver *lcd = dev_get_drvdata(&ddev->dev);

	dev_dbg(&ddev->dev, "Invoked %s\n", __func__);

	/*
	 * after suspended, if lcd panel status is FB_BLANK_UNBLANK
	 * (at that time, power is FB_BLANK_UNBLANK) then
	 * it changes that status to FB_BLANK_POWERDOWN to get lcd on.
	 */
	if (lcd->beforepower == FB_BLANK_UNBLANK)
		lcd->power = FB_BLANK_POWERDOWN;

	dev_dbg(&ddev->dev, "power = %d\n", lcd->beforepower);

	ret = godin_power(lcd, lcd->beforepower);

	return ret;
}

static int godin_mcde_panel_suspend(struct mcde_display_device *ddev, pm_message_t state)
{
	int ret = 0;
	struct godin_lcd_driver *lcd = dev_get_drvdata(&ddev->dev);

	dev_dbg(&ddev->dev, "Invoked %s\n", __func__);

	lcd->beforepower = lcd->power;
	/*
	 * when lcd panel is suspend, lcd panel becomes off
	 * regardless of status.
	 */
	ret = godin_power(lcd, FB_BLANK_POWERDOWN);

	/* set_power_mode will handle call platform_disable */
	ret = ddev->set_power_mode(ddev, MCDE_DISPLAY_PM_OFF);
	if (ret < 0)
		dev_warn(&ddev->dev, "%s:Failed to suspend display\n"
			, __func__);

	return ret;
}


#ifdef CONFIG_HAS_EARLYSUSPEND

static void godin_mcde_panel_early_suspend(struct early_suspend *earlysuspend)
{
	struct godin_lcd_driver *lcd = container_of(earlysuspend, struct godin_lcd_driver, earlysuspend);
	pm_message_t dummy;
	
	godin_mcde_panel_suspend(lcd->mdd, dummy);
}

static void godin_mcde_panel_late_resume(struct early_suspend *earlysuspend)
{
	struct godin_lcd_driver *lcd = container_of(earlysuspend, struct godin_lcd_driver, earlysuspend);
	
	godin_mcde_panel_resume(lcd->mdd);
}


#endif

static int godin_mcde_panel_remove(struct mcde_display_device *ddev)
{
	struct godin_lcd_driver *lcd = dev_get_drvdata(&ddev->dev);

	dev_dbg(&ddev->dev, "Invoked %s\n", __func__);
	godin_power(lcd, FB_BLANK_POWERDOWN);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&lcd->earlysuspend);
#endif

	spi_unregister_driver((struct spi_driver *)&godin_spi_drv);
	kfree(lcd);

	return 0;
}


static struct mcde_display_driver godin_mcde = {
	.probe          = godin_mcde_panel_probe,
	.remove         = godin_mcde_panel_remove,
	.shutdown       = NULL,
#ifdef CONFIG_HAS_EARLYSUSPEND
	.suspend        = NULL,
	.resume         = NULL,
#else
	.suspend		= godin_mcde_panel_suspend,
	.resume 		= godin_mcde_panel_resume,
#endif
	.driver		= {
		.name	= LCD_DRIVER_NAME_GODIN,
		.owner 	= THIS_MODULE,
	}
};


static int __init godin_init(void)
{
	int ret = 0;
	ret =  mcde_display_driver_register(&godin_mcde);
	return ret;
}

static void __exit godin_exit(void)
{
	mcde_display_driver_unregister(&godin_mcde);
}

module_init(godin_init);
module_exit(godin_exit);

MODULE_AUTHOR("Gareth Phillips <gareth.phillips@samsung.com>");
MODULE_DESCRIPTION("godin LCD Driver");
MODULE_LICENSE("GPL");

