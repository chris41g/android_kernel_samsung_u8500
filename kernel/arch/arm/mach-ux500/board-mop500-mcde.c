/*
 * Copyright (C) ST-Ericsson AB 2010
 *
 * Author: Marcus Lorentzon <marcus.xm.lorentzon@stericsson.com>
 * for ST-Ericsson.
 *
 * License terms: GNU General Public License (GPL), version 2.
 */
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/mfd/ab8500/denc.h>
#include <linux/workqueue.h>
#include <linux/dispdev.h>
#include <video/av8100.h>
#include <video/mcde_display.h>
#include <video/mcde_display-generic_dsi.h>
#include <video/mcde_display-vuib500-dpi.h>
#include <video/mcde_display-av8100.h>
#include <video/mcde_display-ab8500.h>
#include <video/mcde_fb.h>
#include <video/mcde_dss.h>
#include <plat/pincfg.h>
#include "pins-db8500.h"
#include "pins.h"
#include "board-mop500.h"

#define DSI_UNIT_INTERVAL_0	0x9
#define DSI_UNIT_INTERVAL_1	0x9
#define DSI_UNIT_INTERVAL_2	0x5

#ifdef CONFIG_FB_MCDE

/* The initialization of hdmi disp driver must be delayed in order to
 * ensure that inputclk will be available (needed by hdmi hw) */
#ifdef CONFIG_DISPLAY_AV8100_TERTIARY
static struct delayed_work work_dispreg_hdmi;
#define DISPREG_HDMI_DELAY 6000
#endif

enum {
	PRIMARY_DISPLAY_ID,
	SECONDARY_DISPLAY_ID,
	AV8100_DISPLAY_ID,
	AB8500_DISPLAY_ID,
	MCDE_NR_OF_DISPLAYS
};
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

#ifdef CONFIG_DISPLAY_GENERIC_DSI_PRIMARY
static struct mcde_port port0 = {
	.type = MCDE_PORTTYPE_DSI,
	.mode = MCDE_PORTMODE_CMD,
	.pixel_format = MCDE_PORTPIXFMT_DSI_24BPP,
	.ifc = 1,
	.link = 0,
#ifdef CONFIG_DISPLAY_GENERIC_DSI_PRIMARY_AUTO_SYNC
	.sync_src = MCDE_SYNCSRC_OFF,
	.update_auto_trig = true,
#else
	.sync_src = MCDE_SYNCSRC_BTA,
	.update_auto_trig = false,
#endif
	.phy = {
		.dsi = {
			.virt_id = 0,
			.num_data_lanes = 2,
			.ui = DSI_UNIT_INTERVAL_0,
			.clk_cont = false,
			.data_lanes_swap = false,
		},
	},
};

struct mcde_display_generic_platform_data generic_display0_pdata = {
	.reset_delay = 1,
#ifdef CONFIG_REGULATOR
	.regulator_id = "v-display",
	.min_supply_voltage = 2500000, /* 2.5V */
	.max_supply_voltage = 2700000 /* 2.7V */
#endif
};

struct mcde_display_device generic_display0 = {
	.name = "mcde_disp_generic",
	.id = PRIMARY_DISPLAY_ID,
	.port = &port0,
	.chnl_id = MCDE_CHNL_A,
	/*
	 * A large fifo is needed when ddr is clocked down to 25% to not get
	 * latency problems.
	 */
	.fifo = MCDE_FIFO_A,
#ifdef CONFIG_MCDE_DISPLAY_PRIMARY_16BPP
	.default_pixel_format = MCDE_OVLYPIXFMT_RGB565,
#else
	.default_pixel_format = MCDE_OVLYPIXFMT_RGBA8888,
#endif
	.native_x_res = 864,
	.native_y_res = 480,
	.synchronized_update = false,
	/* TODO: Remove rotation buffers once ESRAM driver is completed */
	.rotbuf1 = U8500_ESRAM_BASE + 0x20000 * 4 + 0x2000,
	.rotbuf2 = U8500_ESRAM_BASE + 0x20000 * 4 + 0x10000 + 0x1000,
	.dev = {
		.platform_data = &generic_display0_pdata,
	},
};
#endif /* CONFIG_DISPLAY_GENERIC_DSI_PRIMARY */

#ifdef CONFIG_DISPLAY_GENERIC_DSI_SECONDARY
static struct mcde_port subdisplay_port = {
	.type = MCDE_PORTTYPE_DSI,
	.mode = MCDE_PORTMODE_CMD,
	.pixel_format = MCDE_PORTPIXFMT_DSI_24BPP,
	.ifc = 1,
	.link = 1,
#ifdef CONFIG_DISPLAY_GENERIC_DSI_SECONDARY_AUTO_SYNC
	.sync_src = MCDE_SYNCSRC_OFF,
	.update_auto_trig = true,
#else
	.sync_src = MCDE_SYNCSRC_BTA,
	.update_auto_trig = false,
#endif
	.phy = {
		.dsi = {
			.virt_id = 0,
			.num_data_lanes = 2,
			.ui = DSI_UNIT_INTERVAL_1,
			.clk_cont = false,
			.data_lanes_swap = false,
		},
	},

};

static struct mcde_display_generic_platform_data generic_subdisplay_pdata = {
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

#ifdef CONFIG_MCDE_DISPLAY_DPI_PRIMARY
static struct mcde_port port0 = {
	.type = MCDE_PORTTYPE_DPI,
	.pixel_format = MCDE_PORTPIXFMT_DPI_24BPP,
	.ifc = 0,
	.link = 1,		/* DPI channel B can only be on link 1 */
	.sync_src = MCDE_SYNCSRC_OFF,   /* sync from output formatter  */
	.update_auto_trig = true,
	.phy = {
		.dpi = {
			.tv_mode = false,
			.clock_div = 2,
			.polarity = DPI_ACT_LOW_VSYNC | DPI_ACT_LOW_HSYNC,
		},
	},
};

struct mcde_display_dpi_platform_data generic_display0_pdata = {0};
static struct ux500_pins *dpi_pins;

static int dpi_display_platform_enable(struct mcde_display_device *ddev)
{
	int res;

	if (!dpi_pins) {
		dpi_pins = ux500_pins_get("mcde-dpi");
		if (!dpi_pins)
			return -EINVAL;
	}

	dev_info(&ddev->dev, "%s\n", __func__);
	res = ux500_pins_enable(dpi_pins);
	if (res)
		dev_warn(&ddev->dev, "Failure during %s\n", __func__);

	return res;
}

static int dpi_display_platform_disable(struct mcde_display_device *ddev)
{
	int res;

	dev_info(&ddev->dev, "%s\n", __func__);

	res = ux500_pins_disable(dpi_pins);
	if (res)
		dev_warn(&ddev->dev, "Failure during %s\n", __func__);

	return res;

}

struct mcde_display_device generic_display0 = {
	.name = "mcde_display_dpi",
	.id = 0,
	.port = &port0,
	.chnl_id = MCDE_CHNL_B,
	.fifo = MCDE_FIFO_B,
#ifdef CONFIG_MCDE_DISPLAY_PRIMARY_16BPP
	.default_pixel_format = MCDE_OVLYPIXFMT_RGB565,
#else
	.default_pixel_format = MCDE_OVLYPIXFMT_RGBA8888,
#endif
	.native_x_res = 640,
	.native_y_res = 480,
	/* .synchronized_update: Don't care: port is set to update_auto_trig */
	.dev = {
		.platform_data = &generic_display0_pdata,
	},
	.platform_enable = dpi_display_platform_enable,
	.platform_disable = dpi_display_platform_disable,
};
#endif /* CONFIG_MCDE_DISPLAY_DPI_PRIMARY */

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
	.nr_regulators = 2,
	.regulator_id  = {"v-tvout", "v-ab8500-AV-switch"},
	.rgb_2_yCbCr_transform = &rgb_2_yCbCr_transform,
};

static struct ux500_pins *tvout_pins;

static int ab8500_platform_enable(struct mcde_display_device *ddev)
{
	int res = 0;

	if (!tvout_pins) {
		tvout_pins = ux500_pins_get("mcde-tvout");
		if (!tvout_pins)
			return -EINVAL;
	}

	dev_info(&ddev->dev, "%s\n", __func__);
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
	.id = AB8500_DISPLAY_ID,
	.port = &port_tvout1,
	.chnl_id = MCDE_CHNL_B,
	.fifo = MCDE_FIFO_B,
	.default_pixel_format = MCDE_OVLYPIXFMT_RGB565,
	.native_x_res = 720,
	.native_y_res = 576,
	/* .synchronized_update: Don't care: port is set to update_auto_trig */
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
			.data_lanes_swap = false,
		},
	},
	.hdmi_sdtv_switch = HDMI_SWITCH,
};

struct mcde_display_hdmi_platform_data av8100_hdmi_pdata = {
	.reset_gpio = 0,
	.reset_delay = 1,
	.regulator_id = NULL, /* TODO: "display_main" */
	.cvbs_regulator_id = "v-av8100-AV-switch",
	.ddb_id = 1,
	.rgb_2_yCbCr_transform = &rgb_2_yCbCr_transform,
};

struct mcde_display_device av8100_hdmi = {
	.name = "av8100_hdmi",
	.id = AV8100_DISPLAY_ID,
	.port = &port2,
	.chnl_id = MCDE_CHNL_B,
	.fifo = MCDE_FIFO_B,
	.default_pixel_format = MCDE_OVLYPIXFMT_RGB888,
	.native_x_res = 1280,
	.native_y_res = 720,
	.synchronized_update = false,
	.dev = {
		.platform_data = &av8100_hdmi_pdata,
	},
	.platform_enable = NULL,
	.platform_disable = NULL,
};

static void delayed_work_dispreg_hdmi(struct work_struct *ptr)
{
	if (mcde_display_device_register(&av8100_hdmi))
		pr_warning("Failed to register av8100_hdmi\n");
}
#endif /* CONFIG_DISPLAY_AV8100_TERTIARY */

/*
* This function will create the framebuffer for the display that is registered.
*/
static int display_postregistered_callback(struct notifier_block *nb,
	unsigned long event, void *dev)
{
	struct mcde_display_device *ddev = dev;
	u16 width, height;
	u16 virtual_width, virtual_height;
	u32 rotate = FB_ROTATE_UR;
	struct fb_info *fbi;

	if (event != MCDE_DSS_EVENT_DISPLAY_REGISTERED)
		return 0;

	if (ddev->id < PRIMARY_DISPLAY_ID || ddev->id >= MCDE_NR_OF_DISPLAYS)
		return 0;

	mcde_dss_get_native_resolution(ddev, &width, &height);

#ifdef CONFIG_DISPLAY_GENERIC_DSI_PRIMARY
	if (ddev->id == PRIMARY_DISPLAY_ID) {
		switch (CONFIG_DISPLAY_GENERIC_DSI_PRIMARY_ROTATION_ANGLE) {
		case 0:
			rotate = FB_ROTATE_UR;
			break;
		case 90:
			rotate = FB_ROTATE_CW;
			swap(width, height);
			break;
		case 180:
			rotate = FB_ROTATE_UD;
			break;
		case 270:
			rotate = FB_ROTATE_CCW;
			swap(width, height);
			break;
		}
	}
#endif

	virtual_width = width;
	virtual_height = height * 2;

#if defined(CONFIG_DISPLAY_GENERIC_DSI_PRIMARY) &&	\
			defined(CONFIG_DISPLAY_GENERIC_DSI_PRIMARY_AUTO_SYNC)
	if (ddev->id == PRIMARY_DISPLAY_ID)
		virtual_height = height;
#endif

#if defined(CONFIG_DISPLAY_GENERIC_DSI_SECONDARY) &&	\
			defined(CONFIG_DISPLAY_GENERIC_DSI_SECONDARY_AUTO_SYNC)
	if (ddev->id == SECONDARY_DISPLAY_ID)
		virtual_height = height;
#endif

#ifdef CONFIG_DISPLAY_AV8100_TRIPPLE_BUFFER
	if (ddev->id == AV8100_DISPLAY_ID)
		virtual_height = height * 3;
#endif

	if (ddev->id == AV8100_DISPLAY_ID) {
#ifdef CONFIG_MCDE_DISPLAY_HDMI_FB_AUTO_CREATE
		hdmi_fb_onoff(ddev, 1, 0, 0);
#endif /* CONFIG_MCDE_DISPLAY_HDMI_FB_AUTO_CREATE */
	} else {
		struct mcde_fb *mfb;

		/* Create frame buffer */
		fbi = mcde_fb_create(ddev,
			width, height,
			virtual_width, virtual_height,
			ddev->default_pixel_format,
			rotate);

		if (IS_ERR(fbi)) {
			dev_warn(&ddev->dev,
				"Failed to create fb for display %s\n",
						ddev->name);
			goto display_postregistered_callback_err;
		} else {
			dev_info(&ddev->dev, "Framebuffer created (%s)\n",
						ddev->name);
		}

#ifdef CONFIG_DISPDEV
		mfb = to_mcde_fb(fbi);

		/* Create a dispdev overlay for this display */
		if (dispdev_create(ddev, true, mfb->ovlys[0]) < 0) {
			dev_warn(&ddev->dev,
				"Failed to create disp for display %s\n",
						ddev->name);
			goto display_postregistered_callback_err;
		} else {
			dev_info(&ddev->dev, "Disp dev created for (%s)\n",
						ddev->name);
		}
#endif
	}

	return 0;

display_postregistered_callback_err:
	return -1;
}

static struct notifier_block display_nb = {
	.notifier_call = display_postregistered_callback,
};

/*
* This function is used to refresh the display (lcd, hdmi, tvout) with black
* when the framebuffer is registered.
* The main display will not be updated if startup graphics is displayed
* from u-boot.
*/
#if defined(CONFIG_DISPLAY_GENERIC_DSI_PRIMARY_AUTO_SYNC) || \
			defined(CONFIG_DISPLAY_GENERIC_DSI_SECONDARY_AUTO_SYNC)
static int framebuffer_postregistered_callback(struct notifier_block *nb,
	unsigned long event, void *data)
{
	int ret = 0;
	struct fb_event *event_data = data;
	struct fb_info *info;
	struct fb_var_screeninfo var;
	struct fb_fix_screeninfo fix;
	struct mcde_fb *mfb;
	int i;

	if (event != FB_EVENT_FB_REGISTERED)
		return 0;

	if (!event_data)
		return 0;

	info = event_data->info;
	mfb = to_mcde_fb(info);
	var = info->var;
	fix = info->fix;

	/* Apply overlay info */
	for (i = 0; i < mfb->num_ovlys; i++) {
		struct mcde_overlay *ovly = mfb->ovlys[i];
		struct mcde_overlay_info ovly_info;
		struct mcde_fb *mfb = to_mcde_fb(info);
		memset(&ovly_info, 0, sizeof(ovly_info));
		ovly_info.paddr = fix.smem_start +
			fix.line_length * var.yoffset;
		if (ovly_info.paddr + fix.line_length * var.yres
			 > fix.smem_start + fix.smem_len)
			ovly_info.paddr = fix.smem_start;
		ovly_info.fmt = mfb->pix_fmt;
		ovly_info.stride = fix.line_length;
		ovly_info.w = var.xres;
		ovly_info.h = var.yres;
		ovly_info.dirty.w = var.xres;
		ovly_info.dirty.h = var.yres;
		(void) mcde_dss_apply_overlay(ovly, &ovly_info);
		ret = mcde_dss_update_overlay(ovly);
		if (ret)
			break;
	}

	return ret;
}
#else
static int framebuffer_postregistered_callback(struct notifier_block *nb,
	unsigned long event, void *data)
{
	int ret = 0;
	struct fb_event *event_data = data;
	struct fb_info *info;
	struct fb_var_screeninfo var;
	struct fb_fix_screeninfo fix;
	struct mcde_fb *mfb;

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
	var.yoffset = var.yoffset ? 0 : var.yres;
	if (info->fbops->fb_pan_display)
		ret = info->fbops->fb_pan_display(&var, info);
out:
	return ret;
}
#endif


static struct notifier_block framebuffer_nb = {
	.notifier_call = framebuffer_postregistered_callback,
};

int __init init_display_devices(void)
{
	int ret;

#ifdef CONFIG_DISPLAY_GENERIC_DSI_PRIMARY_VSYNC
	struct i2c_adapter *i2c0;
#endif

	ret = fb_register_client(&framebuffer_nb);
	if (ret)
		pr_warning("Failed to register framebuffer notifier\n");

	ret = mcde_dss_register_notifier(&display_nb);
	if (ret)
		pr_warning("Failed to register dss notifier\n");

#ifdef CONFIG_DISPLAY_GENERIC_PRIMARY
	if (machine_is_hrefv60())
		generic_display0_pdata.reset_gpio = HREFV60_DISP1_RST_GPIO;
	else
		generic_display0_pdata.reset_gpio = EGPIO_PIN_15;

#ifdef CONFIG_DISPLAY_GENERIC_DSI_PRIMARY_VSYNC
	i2c0 = i2c_get_adapter(0);
	if (i2c0) {
		/*
		* U8500-UIB has the TC35893 at 0x44 on I2C0, the
		* ST-UIB has not.
		*/
		ret = i2c_smbus_xfer(i2c0, 0x44, 0, I2C_SMBUS_WRITE, 0,
							I2C_SMBUS_QUICK, NULL);
		i2c_put_adapter(i2c0);

		/* ret == 0 => U8500 UIB connected */
		generic_display0.synchronized_update = (ret == 0);
	}
#endif

	if (display_initialized_during_boot)
		generic_display0.power_mode = MCDE_DISPLAY_PM_STANDBY;
	ret = mcde_display_device_register(&generic_display0);
	if (ret)
		pr_warning("Failed to register generic display device 0\n");
#endif

#ifdef CONFIG_DISPLAY_GENERIC_DSI_SECONDARY
	if (machine_is_hrefv60())
		generic_subdisplay_pdata.reset_gpio = HREFV60_DISP2_RST_GPIO;
	else
		generic_subdisplay_pdata.reset_gpio = EGPIO_PIN_14;
	ret = mcde_display_device_register(&generic_subdisplay);
	if (ret)
		pr_warning("Failed to register generic sub display device\n");
#endif

#ifdef CONFIG_DISPLAY_AV8100_TERTIARY
	INIT_DELAYED_WORK_DEFERRABLE(&work_dispreg_hdmi,
			delayed_work_dispreg_hdmi);

	schedule_delayed_work(&work_dispreg_hdmi,
			msecs_to_jiffies(DISPREG_HDMI_DELAY));
#endif
#ifdef CONFIG_DISPLAY_AB8500_TERTIARY
	ret = mcde_display_device_register(&tvout_ab8500_display);
	if (ret)
		pr_warning("Failed to register ab8500 tvout device\n");
#endif

	return ret;
}

struct mcde_display_device *mcde_get_main_display(void)
{
#if defined(CONFIG_DISPLAY_GENERIC_DSI_PRIMARY)
	return &generic_display0;
#elif defined(CONFIG_DISPLAY_GENERIC_DSI_SECONDARY)
	return &generic_subdisplay;
#elif defined(CONFIG_DISPLAY_AV8100_TERTIARY)
	return &av8100_hdmi;
#elif defined(CONFIG_DISPLAY_AB8500_TERTIARY)
	return &tvout_ab8500_display;
#else
	return NULL;
#endif
}
EXPORT_SYMBOL(mcde_get_main_display);

void hdmi_fb_onoff(struct mcde_display_device *ddev,
		bool enable, u8 cea, u8 vesa_cea_nr)
{
	u16 w, h;
	u16 vw, vh;
	u32 rotate = FB_ROTATE_UR;
	struct display_driver_data *driver_data = dev_get_drvdata(&ddev->dev);

	dev_dbg(&ddev->dev, "%s\n", __func__);
	dev_dbg(&ddev->dev, "en:%d cea:%d nr:%d\n", enable, cea, vesa_cea_nr);

	if (enable) {
		struct fb_info *fbi;
		if (ddev->enabled) {
			dev_dbg(&ddev->dev, "Display is already enabled.\n");
			return;
		}

		/* Create fb */
		if (ddev->fbi == NULL) {
#ifdef CONFIG_DISPLAY_AV8100_TRIPPLE_BUFFER
			int buffering = 3;
#else
			int buffering = 2;
#endif

			/* Get default values */
			mcde_dss_get_native_resolution(ddev, &w, &h);
			vw = w;
			vh = h * buffering;

			if (vesa_cea_nr != 0)
				ddev->ceanr_convert(ddev, cea, vesa_cea_nr,
						buffering, &w, &h, &vw, &vh);

			fbi = mcde_fb_create(ddev, w, h, vw, vh,
				ddev->default_pixel_format, rotate);

			if (IS_ERR(fbi)) {
				dev_warn(&ddev->dev,
					"Failed to create fb for display %s\n",
							ddev->name);
				goto hdmi_fb_onoff_end;
			} else {
				dev_info(&ddev->dev,
					"Framebuffer created (%s)\n",
							ddev->name);
			}
			driver_data->fbdevname = (char *)dev_name(fbi->dev);
		}
	} else {
		if (!ddev->enabled) {
			dev_dbg(&ddev->dev, "Display %s is already disabled.\n",
					ddev->name);
			return;
		}

		mcde_fb_destroy(ddev);
	}

hdmi_fb_onoff_end:
	return;
}
EXPORT_SYMBOL(hdmi_fb_onoff);


module_init(init_display_devices);

#endif
