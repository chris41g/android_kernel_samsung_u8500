/*
 * Copyright (C) ST-Ericsson AB 2010
 *
 * ST-Ericsson MCDE DPI display driver
 *
 * Author: Torbjorn Svensson <torbjorn.x.svensson@stericsson.com>
 * for ST-Ericsson.
 *
 * License terms: GNU General Public License (GPL), version 2.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include <video/mcde_display.h>
#include <video/mcde_display-dpi.h>

#define DPI_DISP_TRACE	dev_dbg(&ddev->dev, "%s\n", __func__)

static int try_video_mode(struct mcde_display_device *ddev,
				struct mcde_video_mode *video_mode);
static int set_video_mode(struct mcde_display_device *ddev,
				struct mcde_video_mode *video_mode);
static int set_power_mode(struct mcde_display_device *ddev,
				enum mcde_display_power_mode power_mode);

static int __devinit dpi_display_probe(struct mcde_display_device *ddev)
{
	int ret = 0;
	struct mcde_display_dpi_platform_data *pdata = ddev->dev.platform_data;
	DPI_DISP_TRACE;

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

	if (pdata->reset_gpio) {
		ret = gpio_request(pdata->reset_gpio, NULL);
		if (ret) {
			dev_warn(&ddev->dev, "%s:Failed to request gpio %d\n",
				__func__, pdata->reset_gpio);
			goto gpio_request_failed;
		}
		gpio_direction_output(pdata->reset_gpio, !pdata->reset_high);
	}
	if (pdata->regulator_id) {
		pdata->regulator = regulator_get(NULL, pdata->regulator_id);
		if (IS_ERR(pdata->regulator)) {
			ret = PTR_ERR(pdata->regulator);
			dev_warn(&ddev->dev,
				"%s:Failed to get regulator '%s'\n",
				__func__, pdata->regulator_id);
			pdata->regulator = NULL;
			goto regulator_get_failed;
		}
		regulator_set_voltage(pdata->regulator,
				pdata->min_supply_voltage,
				pdata->max_supply_voltage);
		/*
		* When u-boot displayed a startup screen.
		* U-boot has turned on display power however the
		* regulator framework does not know about that
		* This is the case here, the display driver has to
		* enable the regulator for the display.
		*/
		if (ddev->power_mode == MCDE_DISPLAY_PM_STANDBY) {
			ret = regulator_enable(pdata->regulator);
			if (ret < 0) {
				dev_err(&ddev->dev,
				"%s:Failed to enable regulator\n", __func__);
				goto regulator_enable_failed;
			}
		}
	}

	ddev->try_video_mode = try_video_mode;
	ddev->set_video_mode = set_video_mode;
	ddev->set_power_mode = set_power_mode;
	ddev->prepare_for_update = NULL;
	dev_info(&ddev->dev, "DPI display probed\n");

	goto out;
regulator_enable_failed:
regulator_get_failed:
	if (pdata->reset_gpio)
		gpio_free(pdata->reset_gpio);
gpio_request_failed:
invalid_port_type:
no_pdata:
out:
	return ret;
}

static int __devexit dpi_display_remove(struct mcde_display_device *ddev)
{
	struct mcde_display_dpi_platform_data *pdata = ddev->dev.platform_data;
	DPI_DISP_TRACE;

	ddev->set_power_mode(ddev, MCDE_DISPLAY_PM_OFF);
	if (pdata->regulator)
		regulator_put(pdata->regulator);
	if (pdata->reset_gpio) {
		gpio_direction_input(pdata->reset_gpio);
		gpio_free(pdata->reset_gpio);
	}

	return 0;
}

static int dpi_display_resume(struct mcde_display_device *ddev)
{
	int ret;
	DPI_DISP_TRACE;

	/* set_power_mode will handle call platform_enable */
	ret = ddev->set_power_mode(ddev, MCDE_DISPLAY_PM_STANDBY);
	if (ret < 0)
		dev_warn(&ddev->dev, "%s:Failed to resume display\n"
			, __func__);
	return ret;
}

static int dpi_display_suspend(struct mcde_display_device *ddev,
							pm_message_t state)
{
	int ret;
	DPI_DISP_TRACE;

	/* set_power_mode will handle call platform_disable */
	ret = ddev->set_power_mode(ddev, MCDE_DISPLAY_PM_OFF);
	if (ret < 0)
		dev_warn(&ddev->dev, "%s:Failed to suspend display\n"
			, __func__);
	return ret;
}

static void print_vmode(struct mcde_video_mode *vmode)
{
	pr_debug("resolution: %dx%d\n", vmode->xres, vmode->yres);
	pr_debug("  pixclock: %d\n",    vmode->pixclock);
	pr_debug("       hbp: %d\n",    vmode->hbp);
	pr_debug("       hfp: %d\n",    vmode->hfp);
	pr_debug("       hsw: %d\n",    vmode->hsw);
	pr_debug("       vbp: %d\n",    vmode->vbp);
	pr_debug("       vfp: %d\n",    vmode->vfp);
	pr_debug("       vsw: %d\n",    vmode->vsw);
	pr_debug("interlaced: %s\n", vmode->interlaced ? "true" : "false");
}

/* Taken from the programmed value of the LCD clock in PRCMU */
#define PIX_CLK_FREQ		25600000
#define VMODE_XRES		800
#define VMODE_YRES		480

static int try_video_mode(
	struct mcde_display_device *ddev, struct mcde_video_mode *video_mode)
{
	int res = -EINVAL;
	DPI_DISP_TRACE;

	if (ddev == NULL || video_mode == NULL) {
		dev_warn(&ddev->dev, "%s:ddev = NULL or video_mode = NULL\n",
			__func__);
		return res;
	}

	if (video_mode->xres == VMODE_XRES && video_mode->yres == VMODE_YRES) {
		video_mode->hbp = 8;
		video_mode->hfp = 8;
		video_mode->hsw = 2;
		video_mode->vbp = 3;
		video_mode->vfp = 28;
		video_mode->vsw = 2;
		video_mode->interlaced = false;
		/*
		 * The pixclock setting is not used within MCDE. The clock is
		 * setup elsewhere. But the pixclock value is visible in user
		 * space.
		 */
		video_mode->pixclock =	(int) (1e+12 * (1.0 / PIX_CLK_FREQ));
		res = 0;
	} /* TODO: add more supported resolutions here */

	if (res == 0)
		print_vmode(video_mode);
	else
		dev_warn(&ddev->dev,
			"%s:Failed to find video mode x=%d, y=%d\n",
			__func__, video_mode->xres, video_mode->yres);

	return res;

}

static int set_video_mode(
	struct mcde_display_device *ddev, struct mcde_video_mode *video_mode)
{
	int res = -EINVAL;
	DPI_DISP_TRACE;

	if (ddev == NULL || video_mode == NULL) {
		dev_warn(&ddev->dev, "%s:ddev = NULL or video_mode = NULL\n",
			__func__);
		goto out;
	}
	ddev->video_mode = *video_mode;
	print_vmode(video_mode);
	if (video_mode->xres == VMODE_XRES && video_mode->yres == VMODE_YRES) {
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

	res = mcde_chnl_set_video_mode(ddev->chnl_state, &ddev->video_mode);
	if (res < 0) {
		dev_warn(&ddev->dev, "%s:Failed to set video mode on channel\n",
			__func__);

		goto error;
	}
	/* notify mcde display driver about updated video mode */
	ddev->update_flags |= UPDATE_FLAG_VIDEO_MODE;
	return res;
out:
error:
	return res;
}

static int set_power_mode(struct mcde_display_device *ddev,
				enum mcde_display_power_mode power_mode)
{
	int ret = 0;
	/* uncomment if used in OFF -> STANDBY change
	struct mcde_display_dpi_platform_data *pdata = ddev->dev.platform_data;
	*/

	/* OFF -> STANDBY */
	if (ddev->power_mode == MCDE_DISPLAY_PM_OFF &&
		power_mode >= MCDE_DISPLAY_PM_OFF) {
		if (ddev->platform_enable)
			ret = ddev->platform_enable(ddev);
		/*
		 * Here you could call the display reset function, e.g:
		if (pdata->reset_gpio)
			my_display_reset(pdata->reset_gpio);
		 */

		if (!ret)
			ddev->power_mode = MCDE_DISPLAY_PM_STANDBY;
	}

	/* STANDBY -> ON */
	if (ddev->power_mode == MCDE_DISPLAY_PM_STANDBY &&
		power_mode == MCDE_DISPLAY_PM_ON) {

		/*
		 * Possibly you want to initialise your panel here.
		 * or perhaps you want to do this earlier.
		 * E.g. you could turn on background LEDS here.
		 * it that case turn them OFF below in ON-> STANDBY
		 * e.g. call some
		init_spi();
		spi_write(xxx, yyy);
		*/

		if (!ret)
			ddev->power_mode = MCDE_DISPLAY_PM_ON;
	}
	/* ON -> STANDBY */
	else if (ddev->power_mode == MCDE_DISPLAY_PM_ON &&
		power_mode <= MCDE_DISPLAY_PM_STANDBY) {


		/*
		 * Possibly you want to put your display in standby mode here.
		spi_write(xxx, yyy);
		deinit_spi();
		*/
		if (!ret)
			ddev->power_mode = MCDE_DISPLAY_PM_STANDBY;
	}

	else if (ddev->power_mode != power_mode)
		return -EINVAL;

	/* SLEEP -> OFF */
	if (!ret && ddev->power_mode == MCDE_DISPLAY_PM_STANDBY &&
		power_mode == MCDE_DISPLAY_PM_OFF) {
		if (ddev->platform_disable)
			ret = ddev->platform_disable(ddev);
		if (!ret)
			ddev->power_mode = MCDE_DISPLAY_PM_OFF;
	}

	mcde_chnl_set_power_mode(ddev->chnl_state, ddev->power_mode);

	return ret;
}

static struct mcde_display_driver dpi_display_driver = {
	.probe	= dpi_display_probe,
	.remove = dpi_display_remove,
	.suspend = dpi_display_suspend,
	.resume = dpi_display_resume,
	.driver = {
		.name	= "mcde_display_dpi",
	},
};

/* Module init */
static int __init mcde_dpi_display_init(void)
{
	pr_info("%s\n", __func__);

	return mcde_display_driver_register(&dpi_display_driver);
}
module_init(mcde_dpi_display_init);

static void __exit mcde_dpi_display_exit(void)
{
	pr_info("%s\n", __func__);

	mcde_display_driver_unregister(&dpi_display_driver);
}
module_exit(mcde_dpi_display_exit);

MODULE_AUTHOR("Torbjorn Svensson <torbjorn.x.svensson@stericsson.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ST-Ericsson MCDE DPI display driver");
