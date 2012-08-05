/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * Samsung MCDE Widechips WS2401 DCS display driver
 *
 * Author: Gareth Phillips <gareth.phillips@samsung.com>
 * for Samsung Electronics.
 *
 * License terms: GNU General Public License (GPL), version 2.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/lcd.h>
#include <linux/backlight.h>
#include <linux/mutex.h>
#include <linux/fb.h>
#include <linux/slab.h>
#include <mach/prcmu.h>

#include <video/mcde_display.h>
#include <video/mcde_display-ws2401_dsi.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define VMODE_XRES		480
#define VMODE_YRES		800
#define PIX_CLK_FREQ		26000000

#define POWER_IS_ON(pwr)	((pwr) <= FB_BLANK_NORMAL)

#define MIN_SUPP_BRIGHTNESS	0
#define MAX_SUPP_BRIGHTNESS	255
#define MAX_REQ_BRIGHTNESS	255
#define DEFAULT_BRIGHTNESS	255

/* timeout for OPP request */
#define OPP_TIMER 		200

#define DCS_CMD_COLMOD		0x3A	/* Set Pixel Format */

#define DCS_CMD_WS2401_RESCTL	0xB8	/* Resolution Select Control */
#define DCS_CMD_WS2401_PSMPS	0xBD	/* SMPS Positive Control */
#define DCS_CMD_WS2401_NSMPS	0xBE	/* SMPS Negative Control */
#define DCS_CMD_WS2401_SMPS	0xBF
#define DCS_CMD_WS2401_BCMODE	0xC1	/* BC Mode */
#define DCS_CMD_WS2401_WRBLCTL	0xC3	/* Backlight Control */
#define DCS_CMD_WS2401_WRDISBV	0xC4	/* Write Manual Brightness */
#define DCS_CMD_WS2401_WRCTRLD	0xC6	/* Write BL Control */
#define DCS_CMD_WS2401_WRMIE	0xC7	/* Write MIE mode */
#define DCS_CMD_WS2401_PASSWD1	0xF0	/* Password1 Command for Level2 */
#define DCS_CMD_WS2401_DISCTL	0xF2	/* Display Control */
#define DCS_CMD_WS2401_PWRCTL	0xF3	/* Power Control */
#define DCS_CMD_WS2401_VCOMCTL	0xF4	/* VCOM Control */
#define DCS_CMD_WS2401_SRCCTL	0xF5	/* Source Control */

#define DCS_CMD_SEQ_DELAY_MS	0xFE
#define DCS_CMD_SEQ_END		0xFF

static const u8 DCS_CMD_SEQ_WS2401_INIT[] = {
/*	Length	Command				Parameters */
	3,	DCS_CMD_WS2401_PASSWD1,		0x5A,	/* Unlock Level2 Commands */
						0x5A,
	2,	DCS_CMD_WS2401_RESCTL,		0x12,	/* 480RGB x 800 */
	2,	DCS_CMD_SET_ADDRESS_MODE,	0x09,	/* Flip V(d0), Flip H(d1), RGB/BGR(d3) */
	2,	DCS_CMD_COLMOD,			0x70,	/* 0x60 = 262K Colour(=18 bit/pixel), 0x70 = 16.7M Colour(=24 bit/pixel) */
	7,	DCS_CMD_WS2401_PSMPS,		0x06,
						0x01,	/* DDVDH = 4.6v */
						0x33,
						0x03,
						0x12,
						0x77,
	7,	DCS_CMD_WS2401_NSMPS,		0x06,
						0x01,	/* DDVDL = -4.6v */
						0x33,
						0x05,
						0x15,
						0x77,
	3,	DCS_CMD_WS2401_SMPS,		0x02,
						0x14,
	11,	DCS_CMD_WS2401_PWRCTL,		0x10,
						0xA9,
						0x00,
						0x01,
						0x44,
						0xF4,	/* VGH = 16.1v, VGL = -13.8v */
						0x50,	/* GREFP = 4.2v (default) */
						0x50,	/* GREFN = -4.2v (default) */
						0x00,
						0x3C,	/* VOUTL = -10v (default) */
	7,	DCS_CMD_WS2401_DISCTL,		0x01,
						0x00,
						0x00,
						0x00,
						0x14,
						0x16,
	4,	DCS_CMD_WS2401_VCOMCTL,		0x30,
						0x53,	/* VCOMDCT = -1.245v */
						0x53,	/* VCOMDCB = -1.245v */
	10,	DCS_CMD_WS2401_SRCCTL,		0x03,
						0x01,
						0x8A,
						0x0A,
						0x0A,
						0x01,	/* 2 dot inversion  */
						0x00,
						0x06,
						0x00,
#if 0
	2,	DCS_CMD_WS2401_BCMODE,		0x01,	/* Manual Brightness only  */
	4,	DCS_CMD_WS2401_WRBLCTL,		0x40,	/* Disable backlight dimming */
						0x00,
						0x00,
#endif
	2,	DCS_CMD_WS2401_WRCTRLD,		0x2C,	/* Enable Backlight Control */
	2,	DCS_CMD_WS2401_WRMIE,		0x01,

	DCS_CMD_SEQ_END
};


struct ws2401_lcd {
	struct device			*dev;
	struct mutex			lock;
	unsigned int			power;
	unsigned int			current_brightness;
	struct mcde_display_device	*mdd;
	struct lcd_device		*ld;
	struct backlight_device		*bd;
	struct ws2401_platform_data	*pd;
	bool				opp_is_requested;
	struct delayed_work		opp_timeout_work;
};

static int ws2401_power(struct ws2401_lcd *lcd, int power);

static int dsi_dcs_write(struct mcde_chnl_state *chnl, u8 cmd, u8* data, int len)
{
return 0;
	int ret = mcde_dsi_dcs_write(chnl, cmd, data, len);
/*	pr_info("%s: cmd = 0x%X, returned %d\n", __func__, cmd, ret); */
	return ret;
}

static int dsi_dcs_write_sequence(struct mcde_display_device *ddev, const u8 *p_seq)
{
	int ret = 0;

	while ((p_seq[0] != DCS_CMD_SEQ_END) && !ret) {
		if (p_seq[0] == DCS_CMD_SEQ_DELAY_MS) {
			msleep(p_seq[1]);
			p_seq += 2;
		} else {
			ret = dsi_dcs_write(ddev->chnl_state, p_seq[1], (u8 *)&p_seq[2], p_seq[0] - 1);
			p_seq += p_seq[0] + 1;
		}
	}

	/* NOTE:
	If the number of command parameters exceeds
	MCDE_MAX_DSI_DIRECT_CMD_WRITE, then dsi_dcs_write()
	will return an error. If we ever need to send a command which
	exceeds this limit, then this function will need to be modified
	to use a combination of dsi_dcs_write() followed by calls to
	mcde_dsi_generic_write() to send the additional parameters.
	*/
	if (ret != 0)
		dev_err(&ddev->dev, "failed to send DCS sequence.\n");

	return ret;
}


static void ws2401_work_opp_timeout_function(struct work_struct *work)
{
	struct ws2401_lcd *lcd = container_of(work,
		struct ws2401_lcd,
		opp_timeout_work.work);

	dev_vdbg(&lcd->mdd->dev, "%s: display update timeout\n",
		__func__);

	prcmu_qos_remove_requirement(PRCMU_QOS_APE_OPP, (char *)"ws2401");
	lcd->opp_is_requested = false;
}


static int ws2401_try_video_mode(struct mcde_display_device *ddev,
					struct mcde_video_mode *video_mode)
{
	int ret = -EINVAL;

	if (!ddev || !video_mode) {
		dev_warn(&ddev->dev, "%s:ddev or video_mode = NULL, aborting\n",
			__func__);
		return ret;
	}

	if ((video_mode->xres == ddev->native_x_res && video_mode->yres == ddev->native_y_res)
		|| (video_mode->xres == ddev->native_y_res && video_mode->yres == ddev->native_x_res)) {

		int hfp_pclk = 11, hbp_pclk = 11, hsw_pclk = 11;
		int bpp = 24;
		video_mode->hbp = hsw_pclk * bpp / 8 + hbp_pclk * bpp / 8 - 6 - 4 - 6;
		video_mode->hfp = hfp_pclk * bpp / 8 - 6;
		video_mode->hsw = 0;
		video_mode->vfp = 21;
		video_mode->vbp = 12;
		video_mode->vsw = 11;
		video_mode->interlaced = false;
		video_mode->pixclock = 30328;

		ret = 0;
	}

	if (!ret == 0)
		dev_warn(&ddev->dev, "%s:Failed to find video mode x=%d, y=%d\n",
			__func__, video_mode->xres, video_mode->yres);

	return ret;
}

static int ws2401_set_video_mode(struct mcde_display_device *ddev,
					struct mcde_video_mode *video_mode)
{
	int ret = 0;
	struct mcde_video_mode channel_video_mode;
	struct ws2401_lcd *lcd = dev_get_drvdata(&ddev->dev);

	/* TODO: Review this when splash screen is added
	- should depend on startup_graphics kernel parameter. */
	static int video_mode_apply_during_boot;

	if (!ddev || !video_mode) {
		dev_warn(&ddev->dev, "%s:ddev or video_mode = NULL, aborting\n", __func__);
		return -EINVAL;
	}

	ddev->video_mode = *video_mode;
	channel_video_mode = ddev->video_mode;
	/* Dependant on if display should rotate or MCDE should rotate */
	if (ddev->rotation == MCDE_DISPLAY_ROT_90_CCW ||
				ddev->rotation == MCDE_DISPLAY_ROT_90_CW) {
		channel_video_mode.xres = ddev->native_x_res;
		channel_video_mode.yres = ddev->native_y_res;
	}
	ret = mcde_chnl_set_video_mode(ddev->chnl_state, &channel_video_mode);
	if (ret < 0) {
		dev_warn(&ddev->dev, "%s:Failed to set video mode\n", __func__);
		return ret;
	}
	/* notify mcde display driver about updated video mode, excepted for
	 * the first update to preserve the splash screen and avoid a
	 * stop_flow() */
	if (video_mode_apply_during_boot && lcd->pd->platform_enabled) {
		ddev->update_flags |= UPDATE_FLAG_PIXEL_FORMAT;
		video_mode_apply_during_boot = 0;
	} else
		ddev->update_flags |= UPDATE_FLAG_VIDEO_MODE;

	return ret;
}

static int ws2401_update(struct mcde_display_device *ddev,
				bool tripple_buffer)
{
	struct ws2401_lcd *lcd = dev_get_drvdata(&ddev->dev);
	int ret = 0;

	dev_dbg(&ddev->dev, "%s\n", __func__);

return 0;
	/* Add 100% APE OPP request */
	if (!lcd->opp_is_requested) {
		if (prcmu_qos_add_requirement(PRCMU_QOS_APE_OPP, "ws2401", 100)) {
			dev_err(&ddev->dev, "APE OPP 100 failed\n");
			return -EFAULT;
		}
		lcd->opp_is_requested = true;
	}

	/* For a display running DSI Command Mode (i.e. with an internal framebuffer),
	the OPP request can be released once the frame has been output to the display.
	When running DSI Video Mode, the OPP request must remain
	in place contnuously whilst the panel is on. */
	if (ddev->port->mode == MCDE_PORTMODE_CMD) {
		cancel_delayed_work(&lcd->opp_timeout_work);
		schedule_delayed_work(&lcd->opp_timeout_work, msecs_to_jiffies(OPP_TIMER));
	}

	if (ddev->power_mode != MCDE_DISPLAY_PM_ON && ddev->set_power_mode) {
		ret = ddev->set_power_mode(ddev, MCDE_DISPLAY_PM_ON);
		if (ret < 0) {
			dev_warn(&ddev->dev,
				"%s:Failed to set power mode to on\n",
				__func__);
			return ret;
		}
	}

	/* TODO: Calculate & set update rect */
	ret = mcde_chnl_update(ddev->chnl_state, &ddev->update_area,
							tripple_buffer);
	if (ret < 0) {
		dev_warn(&ddev->dev, "%s:Failed to update channel\n", __func__);
		return ret;
	}

	if (ddev->first_update && ddev->on_first_update)
		ddev->on_first_update(ddev);

	dev_dbg(&ddev->dev, "Overlay updated, chnl=%d\n", ddev->chnl_id);

	return 0;
}


static int ws2401_update_brightness(struct mcde_display_device *ddev, u8 brightness)
{
	return dsi_dcs_write(ddev->chnl_state, DCS_CMD_WS2401_WRDISBV, &brightness, 1);
}

static int ws2401_get_brightness(struct backlight_device *bd)
{
	dev_dbg(&bd->dev, "lcd get brightness returns %d\n", bd->props.brightness);
	return bd->props.brightness;
}

static int ws2401_set_brightness(struct backlight_device *bd)
{
	int ret = 0, brightness = bd->props.brightness;
	struct ws2401_lcd *lcd = bl_get_data(bd);

	dev_dbg(&bd->dev, "lcd set brightness called with %d\n", brightness);

	if (brightness < MIN_SUPP_BRIGHTNESS ||
		brightness > bd->props.max_brightness) {
		dev_err(&bd->dev, "lcd brightness should be %d to %d.\n",
			MIN_SUPP_BRIGHTNESS, bd->props.max_brightness);
		return -EINVAL;
	}

	mutex_lock(&lcd->lock);

	if (brightness) {
		brightness = brightness / (MAX_REQ_BRIGHTNESS/MAX_SUPP_BRIGHTNESS);
		if (brightness >= MAX_SUPP_BRIGHTNESS)
			brightness = MAX_SUPP_BRIGHTNESS - 1;

		if (!lcd->pd->no_bl_ctrl) {
			if (POWER_IS_ON(lcd->power)) {
				ret = ws2401_update_brightness(lcd->mdd, brightness);
				if (ret) {
					dev_err(&bd->dev, "lcd brightness setting failed.\n");
					return -EIO;
				}
			}
		}
	}

	lcd->current_brightness = brightness;
	mutex_unlock(&lcd->lock);
	return ret;
}


static const struct backlight_ops ws2401_backlight_ops  = {
	.get_brightness = ws2401_get_brightness,
	.update_status = ws2401_set_brightness,
};

struct backlight_properties ws2401_backlight_props = {
	.brightness = DEFAULT_BRIGHTNESS,
	.max_brightness = MAX_REQ_BRIGHTNESS,
};

#ifdef CONFIG_LCD_CLASS_DEVICE

static int ws2401_power_on(struct ws2401_lcd *lcd)
{
	struct ws2401_platform_data	*pdata = lcd->pd;
	struct mcde_display_device	*ddev = lcd->mdd;

	dev_dbg(lcd->dev, "%s: Reset & power on WS2401 display\n", __func__);

	if (pdata->regulator) {
		if (regulator_enable(pdata->regulator) < 0) {
			dev_err(lcd->dev, "%s:Failed to enable regulator\n"
				, __func__);
			return -EINVAL;
		}
	}

	if (pdata->power_on_gpio)
		gpio_set_value(pdata->power_on_gpio, pdata->power_on_high);

	if (pdata->power_on_delay)
		msleep(pdata->power_on_delay); /* as per WS2401 lcd spec */

	if (pdata->reset_gpio)
		gpio_set_value(pdata->reset_gpio, pdata->reset_high);
	msleep(pdata->reset_delay);	/* as per WS2401 lcd spec */
	if (pdata->reset_gpio)
		gpio_set_value(pdata->reset_gpio, !pdata->reset_high);
	msleep(pdata->reset_low_delay);	/* as per WS2401 lcd spec */
	if (pdata->reset_gpio)
		gpio_set_value(pdata->reset_gpio, pdata->reset_high);
	msleep(pdata->reset_delay);	/* as per WS2401 lcd spec */

	dsi_dcs_write_sequence(ddev, DCS_CMD_SEQ_WS2401_INIT);

	dsi_dcs_write(ddev->chnl_state, DCS_CMD_EXIT_SLEEP_MODE, NULL, 0);
	msleep(pdata->sleep_out_delay);

	dsi_dcs_write(ddev->chnl_state, DCS_CMD_SET_DISPLAY_ON, NULL, 0);

	ws2401_update_brightness(ddev, lcd->bd->props.brightness);

	return 0;
}

static int ws2401_power_off(struct ws2401_lcd *lcd)
{
	struct ws2401_platform_data	*pdata = lcd->pd;
	struct mcde_display_device	*ddev = lcd->mdd;

	dev_dbg(lcd->dev, "%s:Reset & power off WS2401 display\n", __func__);

	dsi_dcs_write(ddev->chnl_state, DCS_CMD_ENTER_SLEEP_MODE, NULL, 0);
	msleep(pdata->sleep_in_delay);

	if (pdata->regulator) {
		if (regulator_disable(pdata->regulator) < 0) {
			dev_err(lcd->dev, "%s:Failed to disable regulator\n"
				, __func__);
			return -EINVAL;
		}
	}

	if (pdata->power_on_gpio)
		gpio_set_value(pdata->power_on_gpio, !pdata->power_on_high);

	return 0;
}

static int ws2401_power(struct ws2401_lcd *lcd, int power)
{
	int ret = 0;

	dev_dbg(lcd->dev, "%s(): old=%d (%s), new=%d (%s)\n", __func__,
		lcd->power, POWER_IS_ON(lcd->power) ? "on" : "off",
		power, POWER_IS_ON(power) ? "on" : "off"
		);

	if (POWER_IS_ON(power) && !POWER_IS_ON(lcd->power))
		ret = ws2401_power_on(lcd);
	else if (!POWER_IS_ON(power) && POWER_IS_ON(lcd->power))
		ret = ws2401_power_off(lcd);
	if (!ret)
		lcd->power = power;

	return ret;
}


static int ws2401_set_power(struct lcd_device *ld, int power)
{
	struct ws2401_lcd *lcd = lcd_get_data(ld);

	dev_dbg(lcd->dev, "%s function entered\n", __func__);

	if (power != FB_BLANK_UNBLANK && power != FB_BLANK_POWERDOWN &&
		power != FB_BLANK_NORMAL) {
		dev_err(lcd->dev, "power value should be 0, 1 or 4.\n");
		return -EINVAL;
	}

	return ws2401_power(lcd, power);
}

static int ws2401_get_power(struct lcd_device *ld)
{
	struct ws2401_lcd *lcd = lcd_get_data(ld);

	return lcd->power;
}

static struct lcd_ops ws2401_lcd_ops = {
	.set_power = ws2401_set_power,
	.get_power = ws2401_get_power,
};
#endif

static int ws2401_platform_enable(struct mcde_display_device *ddev)
{
	int ret = 0;
	struct ws2401_lcd *lcd = NULL;

	lcd = dev_get_drvdata(&ddev->dev);

	dev_dbg(lcd->dev, "%s: Reset & power on WS2401 display\n", __func__);

	if (!lcd->pd->no_bl_ctrl) {
		/* switch backlight on */
		gpio_set_value(lcd->pd->bl_en_gpio, 1);
	}
	if (lcd->pd->reset_gpio)
		gpio_set_value(lcd->pd->reset_gpio, lcd->pd->reset_high);

	if (lcd->pd->power_on_gpio)
		gpio_set_value(lcd->pd->power_on_gpio, lcd->pd->power_on_high);

	if (lcd->pd->power_on_delay)
		msleep(lcd->pd->power_on_delay); /* as per WS2401 lcd spec */

	if (lcd->pd->reset_gpio)
		gpio_set_value(lcd->pd->reset_gpio, !lcd->pd->reset_high);
	msleep(lcd->pd->reset_low_delay); /* as per WS2401 lcd spec */
	if (lcd->pd->reset_gpio)
		gpio_set_value(lcd->pd->reset_gpio, lcd->pd->reset_high);
	msleep(lcd->pd->reset_delay);	/* as per WS2401 lcd spec */

	return ret;
}

static int ws2401_platform_disable(struct mcde_display_device *ddev)
{
	int ret = 0;
	struct ws2401_lcd *lcd = NULL;

	lcd = dev_get_drvdata(&ddev->dev);

	if (!lcd->pd->no_bl_ctrl) {
		/* switch backlight off */
		gpio_set_value(lcd->pd->bl_en_gpio, 0);
	}
	/* Remove OPP request */
	if (ddev->port->mode == MCDE_PORTMODE_CMD)
		cancel_delayed_work(&lcd->opp_timeout_work);
	prcmu_qos_remove_requirement(PRCMU_QOS_APE_OPP, (char *)"ws2401");
	lcd->opp_is_requested = false;

	if (lcd->pd->power_on_gpio)
		gpio_set_value(lcd->pd->power_on_gpio, !lcd->pd->power_on_high);

	return ret;
}

static int ws2401_set_power_mode(struct mcde_display_device *ddev,
	enum mcde_display_power_mode power_mode)
{
	int ret = 0;
	struct ws2401_platform_data *pdata = ddev->dev.platform_data;
	struct ws2401_lcd *lcd = NULL;

	lcd = dev_get_drvdata(&ddev->dev);

	dev_dbg(&ddev->dev, "%s:Set Power mode %d->%d\n", __func__,
				ddev->power_mode, power_mode);

	/* OFF -> STANDBY */
	if (ddev->power_mode == MCDE_DISPLAY_PM_OFF &&
		power_mode != MCDE_DISPLAY_PM_OFF) {

		if (ddev->platform_enable) {
			ret = ddev->platform_enable(ddev);
			if (ret)
				return ret;
		}

		ddev->power_mode = MCDE_DISPLAY_PM_STANDBY;
	}

	/* STANDBY -> ON */
	if (ddev->power_mode == MCDE_DISPLAY_PM_STANDBY &&
		power_mode == MCDE_DISPLAY_PM_ON) {

		ret = dsi_dcs_write_sequence(ddev, DCS_CMD_SEQ_WS2401_INIT);
		if (ret)
			return ret;

		ret = dsi_dcs_write(ddev->chnl_state,
					DCS_CMD_EXIT_SLEEP_MODE, NULL, 0);
		if (ret)
			return ret;

		msleep(pdata->sleep_out_delay);

		ws2401_update_brightness(ddev, lcd->bd->props.brightness);

		ret = dsi_dcs_write(ddev->chnl_state,
					DCS_CMD_SET_DISPLAY_ON, NULL, 0);
		if (ret)
			return ret;

		ddev->power_mode = MCDE_DISPLAY_PM_ON;
		goto set_power_and_exit;
	}
	/* ON -> STANDBY */
	else if (ddev->power_mode == MCDE_DISPLAY_PM_ON &&
		power_mode <= MCDE_DISPLAY_PM_STANDBY) {

		ret = dsi_dcs_write(ddev->chnl_state,
					DCS_CMD_SET_DISPLAY_OFF, NULL, 0);
		if (ret)
			return ret;

		ret = dsi_dcs_write(ddev->chnl_state,
					DCS_CMD_ENTER_SLEEP_MODE, NULL, 0);
		if (ret)
			return ret;

		ddev->power_mode = MCDE_DISPLAY_PM_STANDBY;
	}

	/* SLEEP -> OFF */
	if (ddev->power_mode == MCDE_DISPLAY_PM_STANDBY &&
		power_mode == MCDE_DISPLAY_PM_OFF) {

		if (ddev->platform_disable) {
			ret = ddev->platform_disable(ddev);
			if (ret)
				return ret;
		}
		ddev->power_mode = MCDE_DISPLAY_PM_OFF;
	}

set_power_and_exit:
	mcde_chnl_set_power_mode(ddev->chnl_state, ddev->power_mode);

	return ret;
}

static int __devinit ws2401_probe(struct mcde_display_device *ddev)
{
	int ret = 0;
	struct ws2401_platform_data *pdata = ddev->dev.platform_data;
	struct backlight_device *bd = NULL;
	struct ws2401_lcd *lcd = NULL;

	dev_dbg(&ddev->dev, "%s function entered\n", __func__);

	if (pdata == NULL) {
		dev_err(&ddev->dev, "%s:Platform data missing\n", __func__);
		return -EINVAL;
	}

	if (ddev->port->type != MCDE_PORTTYPE_DSI) {
		dev_err(&ddev->dev, "%s:Invalid port type %d\n", __func__, ddev->port->type);
		return -EINVAL;
	}

	ddev->prepare_for_update = NULL;
	ddev->update = ws2401_update;
	ddev->try_video_mode = ws2401_try_video_mode;
	ddev->set_video_mode = ws2401_set_video_mode;

	if (pdata->reset_gpio) {
		ret = gpio_request(pdata->reset_gpio, "LCD Reset");
		if (ret) {
			dev_warn(&ddev->dev, "%s:Failed to request gpio %d\n", __func__, pdata->reset_gpio);
			goto request_reset_gpio_failed;
		}
		gpio_direction_output(pdata->reset_gpio,
			!pdata->reset_high);
	}

	if (pdata->power_on_gpio) {
		ret = gpio_request(pdata->power_on_gpio, "LCD LDO EN");
		if (ret) {
			dev_warn(&ddev->dev, "%s:Failed to request gpio %d\n", __func__, pdata->power_on_gpio);
			goto request_power_on_gpio_failed;
		}
		gpio_direction_output(pdata->power_on_gpio,
			pdata->power_on_high);
	}

	ddev->platform_enable = ws2401_platform_enable,
	ddev->platform_disable = ws2401_platform_disable,
	ddev->set_power_mode = ws2401_set_power_mode;
	ddev->native_x_res = VMODE_XRES;
	ddev->native_y_res = VMODE_YRES;

	lcd = kzalloc(sizeof(struct ws2401_lcd), GFP_KERNEL);
	if (!lcd)
		return -ENOMEM;

	dev_set_drvdata(&ddev->dev, lcd);
	lcd->mdd = ddev;
	lcd->dev = &ddev->dev;
	lcd->pd = pdata;
	lcd->opp_is_requested = false;

	if (ddev->port->mode == MCDE_PORTMODE_CMD) {
		INIT_DELAYED_WORK_DEFERRABLE(
			&lcd->opp_timeout_work,
			ws2401_work_opp_timeout_function);
	}

#ifdef CONFIG_LCD_CLASS_DEVICE
	lcd->ld = lcd_device_register("ws2401", &ddev->dev,
					lcd, &ws2401_lcd_ops);
	if (IS_ERR(lcd->ld)) {
		ret = PTR_ERR(lcd->ld);
		goto out_free_lcd;
	}
#endif

	mutex_init(&lcd->lock);

	if (pdata->no_bl_ctrl) {
		lcd->bd = NULL;
	} else {
		if (gpio_is_valid(pdata->bl_en_gpio)) {
			gpio_request(pdata->bl_en_gpio, "LCD BL Ctrl");
			gpio_direction_output(pdata->bl_en_gpio, 1);
		}

		bd = backlight_device_register("pwm-backlight",
						&ddev->dev,
						lcd,
						&ws2401_backlight_ops,
						&ws2401_backlight_props);
		if (IS_ERR(bd)) {
			ret =  PTR_ERR(bd);
			goto out_backlight_unregister;
		}
		lcd->bd = bd;
	}

	goto out;

out_backlight_unregister:
	backlight_device_unregister(bd);
#ifdef CONFIG_LCD_CLASS_DEVICE
out_free_lcd:
#endif
	kfree(lcd);
request_power_on_gpio_failed:
	if (pdata->reset_gpio)
		gpio_free(pdata->reset_gpio);
request_reset_gpio_failed:
out:
	return ret;
}

static int __devexit ws2401_remove(struct mcde_display_device *ddev)
{
	struct ws2401_platform_data *pdata = ddev->dev.platform_data;

	ddev->set_power_mode(ddev, MCDE_DISPLAY_PM_OFF);

	if (pdata->reset_gpio) {
		gpio_direction_input(pdata->reset_gpio);
		gpio_free(pdata->reset_gpio);
	}
	if (pdata->power_on_gpio) {
		gpio_direction_input(pdata->power_on_gpio);
		gpio_free(pdata->power_on_gpio);
	}

	return 0;
}

#if !defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_PM)

static int ws2401_resume(struct mcde_display_device *ddev)
{
	int ret;
	struct ws2401_lcd *lcd = dev_get_drvdata(&ddev->dev);

	dev_dbg(&ddev->dev, "%s function entered\n", __func__);

	/* set_power_mode will handle call platform_enable */
	ret = ddev->set_power_mode(ddev, MCDE_DISPLAY_PM_STANDBY);

	if (ret < 0)
		dev_warn(&ddev->dev, "%s:Failed to resume display\n"
			, __func__);
	else
		ret = ws2401_power(lcd, FB_BLANK_UNBLANK);

	return ret;
}

static int ws2401_suspend(struct mcde_display_device *ddev, \
							pm_message_t state)
{
	int ret;

	dev_dbg(&ddev->dev, "%s function entered\n", __func__);

	/* set_power_mode will handle call platform_disable */
	ret = ddev->set_power_mode(ddev, MCDE_DISPLAY_PM_OFF);
	if (ret < 0)
		dev_warn(&ddev->dev, "%s:Failed to suspend display\n"
			, __func__);
	return ret;
}

#endif /* !defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_PM) */


static struct mcde_display_driver ws2401_driver = {
	.probe	= ws2401_probe,
	.remove = ws2401_remove,
#if !defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_PM)
	.suspend = ws2401_suspend,
	.resume = ws2401_resume,
#else
	.suspend = NULL,
	.resume = NULL,
#endif
	.driver = {
		.name	= "mcde_disp_ws2401",
	},
};

/* Module init */
static int __init mcde_display_ws2401_init(void)
{
	pr_info("%s\n", __func__);

	return mcde_display_driver_register(&ws2401_driver);
}
module_init(mcde_display_ws2401_init);

static void __exit mcde_display_ws2401_exit(void)
{
	pr_info("%s\n", __func__);

	mcde_display_driver_unregister(&ws2401_driver);
}
module_exit(mcde_display_ws2401_exit);

MODULE_AUTHOR("Gareth Phillips <gareth.phillips@samsung.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Samsung MCDE Widechips WS2401 DCS display driver");

