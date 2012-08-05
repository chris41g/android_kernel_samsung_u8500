/*
 * Copyright (C) ST-Ericsson AB 2010
 *
 * Author: Marcus Lorentzon <marcus.xm.lorentzon@stericsson.com>
 * for ST-Ericsson.
 *
 * License terms: GNU General Public License (GPL), version 2.
 */
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/mfd/ab8500/denc.h>
#include <video/av8100.h>
#include <video/mcde_display.h>
#include <video/mcde_display-sony_sy35560_dsi.h>
#include <video/mcde_display-av8100.h>
#include <video/mcde_display-ab8500.h>
#include <video/mcde_fb.h>
#include <video/mcde_dss.h>
#include "pins-db8500.h"
#include "pins.h"

#define DSI_UNIT_INTERVAL_0	0x9
#define DSI_UNIT_INTERVAL_1	0x9
#define DSI_UNIT_INTERVAL_2	0x6

#define PRIMARY_DISPLAY_ID	0
#define SECONDARY_DISPLAY_ID	1
#define TERTIARY_DISPLAY_ID	2

#define ROTATE_MAIN		0

static int display_initialized_during_boot;

static int __init startup_graphics_setup(char *str)
{

	if (get_option(&str, &display_initialized_during_boot) != 1)
		display_initialized_during_boot = 0;

	switch (display_initialized_during_boot) {
	case 1:
		pr_info("Startup graphics support\n");
		break;
	case 0:
	default:
		pr_info("No startup graphics supported\n");
		break;
	};

	return 1;
}
__setup("startup_graphics=", startup_graphics_setup);

#if defined(CONFIG_DISPLAY_AB8500_TERTIARY) ||\
					defined(CONFIG_DISPLAY_AV8100_TERTIARY)
static struct mcde_col_transform rgb_2_yCbCr_transform = {
	.matrix = {
		{0x0042, 0x0081, 0x0019},
		{0xffda, 0xffb6, 0x0070},
		{0x0070, 0xffa2, 0xffee},
	},
	.offset = {0x10, 0x80, 0x80},
};
#endif

#ifdef CONFIG_DISPLAY_SONY_SY35560_DSI_PRIMARY

static int sony_sy35560_update(struct mcde_display_device *dev)
{
	int ret;

	/* TODO: Dirty */
	if (dev->prepare_for_update) {
		/* TODO: Send dirty rectangle */
		ret = dev->prepare_for_update(dev, 0, 0,
			dev->native_x_res, dev->native_y_res);
		if (ret < 0) {
			dev_warn(&dev->dev,
				"%s:Failed to prepare for update\n", __func__);
			return ret;
		}
	}
	/* TODO: Calculate & set update rect */
	ret = mcde_chnl_update(dev->chnl_state, &dev->update_area);
	if (ret < 0) {
		dev_warn(&dev->dev, "%s:Failed to update channel\n", __func__);
		return ret;
	}
	if (dev->first_update && dev->on_first_update)
		dev->on_first_update(dev);

	if (dev->power_mode != MCDE_DISPLAY_PM_ON && dev->set_power_mode) {
		/* need to wait a while before turning on the display */
		mdelay(5);
		ret = dev->set_power_mode(dev, MCDE_DISPLAY_PM_ON);
		if (ret < 0) {
			dev_warn(&dev->dev,
				"%s:Failed to set power mode to on\n",
				__func__);
			return ret;
		}
	}

	dev_vdbg(&dev->dev, "Overlay updated, chnl=%d\n", dev->chnl_id);

	return 0;
}

static int sony_sy35560_platform_enable(struct mcde_display_device *dev)
{
	int ret = 0;
	struct sony_sy35560_platform_data *pdata =
		dev->dev.platform_data;

	dev_info(&dev->dev, "%s: Reset & power on sony display\n", __func__);

	if (pdata->regulator_id) {
		((struct sony_sy35560_device *)dev)->regulator =
			regulator_get(NULL, pdata->regulator_id);
		if (IS_ERR(((struct sony_sy35560_device *)dev)->regulator)) {
			ret = PTR_ERR(
				((struct sony_sy35560_device *)dev)->regulator);
			dev_err(&dev->dev,
				"%s:Failed to get regulator '%s'\n",
				__func__, pdata->regulator_id);
			((struct sony_sy35560_device *)dev)->regulator = NULL;
			goto out;
		}
		regulator_set_voltage(
			((struct sony_sy35560_device *)dev)->regulator,
			pdata->min_supply_voltage,
			pdata->max_supply_voltage);
		ret = regulator_enable(
			((struct sony_sy35560_device *)dev)->regulator);
		if (ret < 0) {
			dev_err(&dev->dev,
				"%s:Failed to enable regulator\n"
				, __func__);
			goto out;
		}
	}

	if (!pdata->skip_init) {
		dev_info(&dev->dev,
			"%s: Startup graphics disabled, doing full init\n",
			__func__);
		if (pdata->reset_gpio) {
			ret = gpio_request(pdata->reset_gpio, NULL);
			if (ret) {
				dev_warn(&dev->dev,
					"%s:Failed to request gpio %d\n",
					__func__, pdata->reset_gpio);
				goto out;
			}

			gpio_direction_output(pdata->reset_gpio, 1);
			gpio_set_value(pdata->reset_gpio, 0);
			mdelay(1);
			gpio_set_value(pdata->reset_gpio, 1);
		}
	} else {
		dev_info(&dev->dev,
			"%s: Display already initialized during boot\n",
			__func__);
		pdata->skip_init = false;
	}

	dev->update = sony_sy35560_update;

	/* TODO: Remove when DSI send command uses interrupts */
	dev->prepare_for_update = NULL;

#ifdef CONFIG_SONY_SY35560_ENABLE_ESD_CHECK
	/* add ESD status check to workqueue */
	queue_delayed_work(((struct sony_sy35560_device *)dev)->esd_wq,
		&(((struct sony_sy35560_device *)dev)->esd_work),
		SONY_SY35560_ESD_CHECK_PERIOD);
#endif

out:
	if (pdata->reset_gpio)
		gpio_free(pdata->reset_gpio);
	return ret;
}

static int sony_sy35560_platform_disable(struct mcde_display_device *dev)
{
	dev_info(&dev->dev, "%s: Reset & power off sony display\n", __func__);

	if (((struct sony_sy35560_device *)dev)->regulator) {
		if (regulator_disable(
			((struct sony_sy35560_device *)dev)->regulator) < 0)
			dev_err(&dev->dev,
				"%s:Failed to disable regulator\n",
				__func__);
	}
	return 0;
}

static struct mcde_port port0 = {
	.type = MCDE_PORTTYPE_DSI,
	.mode = MCDE_PORTMODE_CMD,
	.pixel_format = MCDE_PORTPIXFMT_DSI_24BPP,
	.ifc = 1,
	.link = 0,
	.sync_src = MCDE_SYNCSRC_BTA,
	.update_auto_trig = false,
	.phy = {
		.dsi = {
			.virt_id = 0,
			.num_data_lanes = 2,
			.ui = DSI_UNIT_INTERVAL_0,
			.clk_cont = false,
		},
	},
};

struct sony_sy35560_platform_data sony_sy35560_display0_pdata = {
	.reset_gpio = EGPIO_PIN_15,
#ifdef CONFIG_REGULATOR
	.regulator_id = "v-display",
	.min_supply_voltage = 2800000, /* 2.8V */
	.max_supply_voltage = 2800000 /* 2.8V */
#endif
};

struct sony_sy35560_device sony_sy35560_display0 = {
	.base = {
		.name = "mcde_disp_sony",
		.id = PRIMARY_DISPLAY_ID,
		.port = &port0,
		.chnl_id = MCDE_CHNL_A,
		.fifo = MCDE_FIFO_C0,
		.default_pixel_format = MCDE_OVLYPIXFMT_RGB565,
		.native_x_res = 480,
		.native_y_res = 854,
#ifdef CONFIG_DISPLAY_GENERIC_DSI_PRIMARY_VSYNC
		.synchronized_update = true,
#else
		.synchronized_update = false,
#endif
		.platform_enable = sony_sy35560_platform_enable,
		.platform_disable = sony_sy35560_platform_disable,
		/* TODO: Remove rotation buffers once ESRAM driver
		 * is completed */
		.rotbuf1 = U8500_ESRAM_BASE + 0x20000 * 4,
		.rotbuf2 = U8500_ESRAM_BASE + 0x20000 * 4 + 0x10000,
		.dev = {
			.platform_data = &sony_sy35560_display0_pdata,
		},
	}
};
#endif /* CONFIG_DISPLAY_SONY_SY35560_DSI_PRIMARY */

#ifdef CONFIG_DISPLAY_GENERIC_DSI_SECONDARY
static struct mcde_port subdisplay_port = {
	.type = MCDE_PORTTYPE_DSI,
	.mode = MCDE_PORTMODE_CMD,
	.pixel_format = MCDE_PORTPIXFMT_DSI_24BPP,
	.ifc = 1,
	.link = 1,
	.sync_src = MCDE_SYNCSRC_BTA,
	.update_auto_trig = false,
	.phy = {
		.dsi = {
			.virt_id = 0,
			.num_data_lanes = 2,
			.ui = DSI_UNIT_INTERVAL_1,
			.clk_cont = false,
		},
	},

};

static struct mcde_display_generic_platform_data generic_subdisplay_pdata = {
	.reset_gpio = EGPIO_PIN_14,
	.reset_delay = 1,
#ifdef CONFIG_REGULATOR
	.regulator_id = "v-display",
	.min_supply_voltage = 2500000, /* 2.5V */
	.max_supply_voltage = 2700000 /* 2.7V */
#endif
};

static struct mcde_display_device generic_subdisplay = {
	.name = "mcde_disp_generic_subdisplay",
	.id = SECONDARY_DISPLAY_ID,
	.port = &subdisplay_port,
	.chnl_id = MCDE_CHNL_C1,
	.fifo = MCDE_FIFO_C1,
	.default_pixel_format = MCDE_OVLYPIXFMT_RGB565,
	.native_x_res = 864,
	.native_y_res = 480,
#ifdef CONFIG_DISPLAY_GENERIC_DSI_SECONDARY_VSYNC
	.synchronized_update = true,
#else
	.synchronized_update = false,
#endif
	.dev = {
		.platform_data = &generic_subdisplay_pdata,
	},
};
#endif /* CONFIG_DISPLAY_GENERIC_DSI_SECONDARY */


#ifdef CONFIG_DISPLAY_AB8500_TERTIARY
static struct mcde_port port_tvout1 = {
	.type = MCDE_PORTTYPE_DPI,
	.pixel_format = MCDE_PORTPIXFMT_DPI_24BPP,
	.ifc = 0,
	.link = 1,			/* channel B */
	.sync_src = MCDE_SYNCSRC_OFF,
	.update_auto_trig = true,
	.phy = {
		.dpi = {
			.bus_width = 4, /* DDR mode */
			.tv_mode = true,
			.clock_div = MCDE_PORT_DPI_NO_CLOCK_DIV,
		},
	},
};

static struct ab8500_display_platform_data ab8500_display_pdata = {
	.denc_regulator_id = "v-tvout",
	.rgb_2_yCbCr_transform = &rgb_2_yCbCr_transform,
};

static struct ux500_pins *tvout_pins;

static int ab8500_platform_enable(struct mcde_display_device *ddev)
{
	int res = 0;

	dev_info(&ddev->dev, "%s\n", __func__);

	if (!tvout_pins) {
		tvout_pins = ux500_pins_get("mcde-tvout");
		if (!tvout_pins)
			return -EINVAL;
	}

	res = ux500_pins_enable(tvout_pins);
	if (res != 0)
		goto failed;

	return res;

failed:
	dev_warn(&ddev->dev, "Failure during %s\n", __func__);
	return res;
}

static int ab8500_platform_disable(struct mcde_display_device *ddev)
{
	int res;

	dev_info(&ddev->dev, "%s\n", __func__);

	res = ux500_pins_disable(tvout_pins);
	if (res != 0)
		goto failed;
	return res;

failed:
	dev_warn(&ddev->dev, "Failure during %s\n", __func__);
	return res;
}

struct mcde_display_device tvout_ab8500_display = {
	.name = "mcde_tv_ab8500",
	.id = TERTIARY_DISPLAY_ID,
	.port = &port_tvout1,
	.chnl_id = MCDE_CHNL_B,
	.fifo = MCDE_FIFO_B,
	.default_pixel_format = MCDE_OVLYPIXFMT_RGB565,
	.native_x_res = 720,
	.native_y_res = 576,
	.dev = {
		.platform_data = &ab8500_display_pdata,
	},

	/*
	* We might need to describe the std here:
	* - there are different PAL / NTSC formats (do they require MCDE
	*   settings?)
	*/
	.platform_enable = ab8500_platform_enable,
	.platform_disable = ab8500_platform_disable,
};
#endif /* CONFIG_DISPLAY_AB8500_TERTIARY */

#ifdef CONFIG_DISPLAY_AV8100_TERTIARY
static struct mcde_port port2 = {
	.type = MCDE_PORTTYPE_DSI,
	.mode = MCDE_PORTMODE_CMD,
	.pixel_format = MCDE_PORTPIXFMT_DSI_24BPP,
	.ifc = 1,
	.link = 2,
#ifdef CONFIG_AV8100_HWTRIG_INT
	.sync_src = MCDE_SYNCSRC_TE0,
#endif
#ifdef CONFIG_AV8100_HWTRIG_I2SDAT3
	.sync_src = MCDE_SYNCSRC_TE1,
#endif
#ifdef CONFIG_AV8100_HWTRIG_DSI_TE
	.sync_src = MCDE_SYNCSRC_TE_POLLING,
#endif
#ifdef CONFIG_AV8100_HWTRIG_NONE
	.sync_src = MCDE_SYNCSRC_OFF,
#endif
	.update_auto_trig = true,
	.phy = {
		.dsi = {
			.virt_id = 0,
			.num_data_lanes = 2,
			.ui = DSI_UNIT_INTERVAL_2,
			.clk_cont = false,
		},
	},
	.hdmi_sdtv_switch = HDMI_SWITCH,
};

struct mcde_display_hdmi_platform_data av8100_hdmi_pdata = {
	.reset_gpio = 0,
	.reset_delay = 1,
	.regulator_id = NULL, /* TODO: "display_main" */
	.ddb_id = 1,
	.rgb_2_yCbCr_transform = &rgb_2_yCbCr_transform,
};

static int av8100_platform_enable(struct mcde_display_device *dev)
{
	int ret;
	struct mcde_display_hdmi_platform_data *pdata =
		dev->dev.platform_data;

	if (pdata->reset_gpio)
		gpio_set_value(pdata->reset_gpio, pdata->reset_high);
	if (pdata->regulator)
		ret = regulator_enable(pdata->regulator);
alt_func_failed:
	return ret;
}

static int av8100_platform_disable(struct mcde_display_device *dev)
{
	int ret;
	struct mcde_display_hdmi_platform_data *pdata =
		dev->dev.platform_data;

	if (pdata->reset_gpio)
		gpio_set_value(pdata->reset_gpio, !pdata->reset_high);
	if (pdata->regulator)
		ret = regulator_disable(pdata->regulator);
alt_func_failed:
	return ret;
}

struct mcde_display_device av8100_hdmi = {
	.name = "av8100_hdmi",
	.id = TERTIARY_DISPLAY_ID,
	.port = &port2,
	.chnl_id = MCDE_CHNL_B,
	.fifo = MCDE_FIFO_B,
	.default_pixel_format = MCDE_OVLYPIXFMT_RGB565,
	.native_x_res = 1280,
	.native_y_res = 720,
	.synchronized_update = true,
	.dev = {
		.platform_data = &av8100_hdmi_pdata,
	},
	.platform_enable = av8100_platform_enable,
	.platform_disable = av8100_platform_disable,
};
#endif /* CONFIG_DISPLAY_AV8100_TERTIARY */

static struct fb_info *fbs[3] = { NULL, NULL, NULL };
static struct mcde_display_device *displays[3] = { NULL, NULL, NULL };

static int display_postregistered_callback(struct notifier_block *nb,
	unsigned long event, void *dev)
{
	struct mcde_display_device *ddev = dev;
	u16 width, height;
	u16 virtual_width, virtual_height;
	bool rotate;

	if (event != MCDE_DSS_EVENT_DISPLAY_REGISTERED)
		return 0;

	if (ddev->id < PRIMARY_DISPLAY_ID || ddev->id >= ARRAY_SIZE(fbs))
		return 0;

	mcde_dss_get_native_resolution(ddev, &width, &height);

	rotate = (ddev->id == 0 && ROTATE_MAIN);
	if (rotate) {
		u16 tmp = height;
		height = width;
		width = tmp;
	}

	virtual_width = width;
	virtual_height = height * 2;

	/* Create frame buffer */
	fbs[ddev->id] = mcde_fb_create(ddev,
		width, height,
		virtual_width, virtual_height,
		ddev->default_pixel_format,
		rotate ? FB_ROTATE_CW : FB_ROTATE_UR);
	if (IS_ERR(fbs[ddev->id]))
		pr_warning("Failed to create fb for display %s\n", ddev->name);
	else
		pr_info("Framebuffer created (%s)\n", ddev->name);

	return 0;
}

static struct notifier_block display_nb = {
	.notifier_call = display_postregistered_callback,
};

static int framebuffer_postregistered_callback(struct notifier_block *nb,
	unsigned long event, void *data)
{
	int ret = 0;
	struct fb_event *event_data = data;
	struct fb_info *info;
	struct fb_var_screeninfo var;
	struct fb_fix_screeninfo fix;
	struct mcde_fb *mfb;
	u8 *addr;

	if (event != FB_EVENT_FB_REGISTERED)
		return 0;

	if (!event_data)
		return 0;

	info = event_data->info;
	mfb = to_mcde_fb(info);
	if (mfb->id == 0 && display_initialized_during_boot)
		goto out;


	var = info->var;
	fix = info->fix;
	addr = ioremap(fix.smem_start,
			var.yres_virtual * fix.line_length);
	memset(addr, 0x00,
			var.yres_virtual * fix.line_length);
	var.yoffset = var.yoffset ? 0 : var.yres;
	if (info->fbops->fb_pan_display)
		ret = info->fbops->fb_pan_display(&var, info);
out:
	return ret;
}

static struct notifier_block framebuffer_nb = {
	.notifier_call = framebuffer_postregistered_callback,
};

int __init init_display_devices(void)
{
	int ret;

	ret = fb_register_client(&framebuffer_nb);
	if (ret)
		pr_warning("Failed to register framebuffer notifier\n");

	ret = mcde_dss_register_notifier(&display_nb);
	if (ret)
		pr_warning("Failed to register dss notifier\n");

#ifdef CONFIG_DISPLAY_SONY_SY35560_DSI_PRIMARY
	/* TODO: enable this code if uboot graphics should be used
	 if (display_initialized_during_boot)
		((struct sony_sy35560_platform_data *)sony_sy35560_display0.
		base.dev.platform_data)->skip_init = true;
	 */
	ret = mcde_display_device_register(
		(struct mcde_display_device *)&sony_sy35560_display0);
	if (ret)
		pr_warning("Failed to register Sony sy35560 display device 0\n");
	displays[0] = (struct mcde_display_device *)&sony_sy35560_display0;
#endif

#ifdef CONFIG_DISPLAY_GENERIC_DSI_SECONDARY
	ret = mcde_display_device_register(&generic_subdisplay);
	if (ret)
		pr_warning("Failed to register generic sub display device\n");
	displays[1] = &generic_subdisplay;
#endif

#ifdef CONFIG_DISPLAY_AV8100_TERTIARY
	ret = mcde_display_device_register(&av8100_hdmi);
	if (ret)
		pr_warning("Failed to register av8100_hdmi\n");
	displays[2] = &av8100_hdmi;
#endif
#ifdef CONFIG_DISPLAY_AB8500_TERTIARY
	ret = platform_device_register(&ab8500_denc);
	if (ret)
		pr_warning("Failed to register ab8500_denc device\n");
	else {
		ret = mcde_display_device_register(&tvout_ab8500_display);
		if (ret)
			pr_warning("Failed to register ab8500 tvout device\n");
		displays[2] = &tvout_ab8500_display;
	}
#endif

	return ret;
}

module_init(init_display_devices);
