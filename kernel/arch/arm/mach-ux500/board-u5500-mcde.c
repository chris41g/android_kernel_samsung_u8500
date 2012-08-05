/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * Author: Marcus Lorentzon <marcus.xm.lorentzon@stericsson.com>
 * for ST-Ericsson.
 *
 * License terms: GNU General Public License (GPL), version 2.
 */
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/dispdev.h>
#include <video/av8100.h>
#include <video/mcde_display.h>
#include <video/mcde_display-generic_dsi.h>
#include <video/mcde_display-av8100.h>
#include <video/mcde_fb.h>
#include <video/mcde_dss.h>

#define DSI_UNIT_INTERVAL_0	0xA
#define DSI_UNIT_INTERVAL_2	0x5

#define PRIMARY_DISPLAY_ID	0
#define AV8100_DISPLAY_ID	2

#ifdef CONFIG_FB_MCDE

/* The initialization of hdmi disp driver must be delayed in order to
 * ensure that inputclk will be available (needed by hdmi hw) */
#ifdef CONFIG_DISPLAY_AV8100_TERTIARY
static struct delayed_work work_dispreg_hdmi;
#define DISPREG_HDMI_DELAY 6000
#endif

#define MCDE_NR_OF_DISPLAYS 2
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

#ifdef CONFIG_DISPLAY_AV8100_TERTIARY
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
	.ifc = DSI_VIDEO_MODE,
	.link = 0,
#if defined(CONFIG_DISPLAY_GENERIC_DSI_PRIMARY) &&	\
			defined(CONFIG_DISPLAY_GENERIC_DSI_PRIMARY_AUTO_SYNC)
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
	.reset_gpio = 226,
	.reset_delay = 10,
	.sleep_out_delay = 140,
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
	.fifo = MCDE_FIFO_A,
#ifdef CONFIG_MCDE_DISPLAY_PRIMARY_16BPP
	.default_pixel_format = MCDE_OVLYPIXFMT_RGB565,
#else
	.default_pixel_format = MCDE_OVLYPIXFMT_RGBA8888,
#endif
	.native_x_res = 864,
	.native_y_res = 480,
#ifdef CONFIG_DISPLAY_GENERIC_DSI_PRIMARY_VSYNC
	.synchronized_update = true,
#else
	.synchronized_update = false,
#endif
	/* TODO: Remove rotation buffers once ESRAM driver is completed */
	.rotbuf1 = U5500_ESRAM_BASE + 0x20000 * 2,
	.rotbuf2 = U5500_ESRAM_BASE + 0x20000 * 2 + 0x10000,
	.dev = {
		.platform_data = &generic_display0_pdata,
	},
};
#endif /* CONFIG_DISPLAY_GENERIC_DSI_PRIMARY */

#ifdef CONFIG_DISPLAY_AV8100_TERTIARY
static struct mcde_port port2 = {
	.type = MCDE_PORTTYPE_DSI,
	.mode = MCDE_PORTMODE_CMD,
	.pixel_format = MCDE_PORTPIXFMT_DSI_24BPP,
	.ifc = DSI_VIDEO_MODE,
	.link = 1,
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
	.regulator_id = NULL,
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
#if defined(CONFIG_DISPLAY_GENERIC_DSI_PRIMARY) &&	\
			defined(CONFIG_DISPLAY_GENERIC_DSI_PRIMARY_AUTO_SYNC)
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

	ret = fb_register_client(&framebuffer_nb);
	if (ret)
		pr_warning("Failed to register framebuffer notifier\n");

	ret = mcde_dss_register_notifier(&display_nb);
	if (ret)
		pr_warning("Failed to register dss notifier\n");

#ifdef CONFIG_DISPLAY_GENERIC_PRIMARY
	if (display_initialized_during_boot)
		generic_display0.power_mode = MCDE_DISPLAY_PM_STANDBY;
	ret = mcde_display_device_register(&generic_display0);
	if (ret)
		pr_warning("Failed to register generic display device 0\n");
#endif

#ifdef CONFIG_DISPLAY_AV8100_TERTIARY
	INIT_DELAYED_WORK_DEFERRABLE(&work_dispreg_hdmi,
			delayed_work_dispreg_hdmi);

	schedule_delayed_work(&work_dispreg_hdmi,
			msecs_to_jiffies(DISPREG_HDMI_DELAY));
#endif

	return ret;
}

struct mcde_display_device *mcde_get_main_display(void)
{
#if defined(CONFIG_DISPLAY_GENERIC_DSI_PRIMARY)
	return &generic_display0;
#elif defined(CONFIG_DISPLAY_AV8100_TERTIARY)
	return &av8100_hdmi;
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
