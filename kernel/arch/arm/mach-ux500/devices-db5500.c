/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * Author: Rabin Vincent <rabin.vincent@stericsson.com> for ST-Ericsson
 *
 * Author: Pierre Peiffer <pierre.peiffer@stericsson.com> for ST-Ericsson.
 * for the System Trace Module part.
 *
 * License terms: GNU General Public License (GPL) version 2
 */

#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/usb/musb.h>
#include <linux/dma-mapping.h>

#include <mach/hardware.h>
#include <mach/devices.h>

#include <video/mcde.h>
#include <mach/db5500-regs.h>

#include <mach/prcmu.h>

#include <trace/stm.h>

#define GPIO_DATA(_name, first, num)					\
	{								\
		.name 		= _name,				\
		.first_gpio 	= first,				\
		.first_irq 	= NOMADIK_GPIO_TO_IRQ(first),		\
		.num_gpio	= num,					\
	}

#define GPIO_RESOURCE(block)						\
	{								\
		.start	= U5500_GPIOBANK##block##_BASE,			\
		.end	= U5500_GPIOBANK##block##_BASE + 127,		\
		.flags	= IORESOURCE_MEM,				\
	},								\
	{								\
		.start	= IRQ_DB5500_GPIO##block,			\
		.end	= IRQ_DB5500_GPIO##block,			\
		.flags	= IORESOURCE_IRQ,				\
	}

#define GPIO_DEVICE(block)						\
	{								\
		.name 		= "gpio",				\
		.id		= block,				\
		.num_resources 	= 2,					\
		.resource	= &u5500_gpio_resources[block * 2],	\
		.dev = {						\
			.platform_data = &u5500_gpio_data[block],	\
		},							\
	}

static struct nmk_gpio_platform_data u5500_gpio_data[] = {
	GPIO_DATA("GPIO-0-31", 0, 32),
	GPIO_DATA("GPIO-32-63", 32, 4), /* 36..63 not routed to pin */
	GPIO_DATA("GPIO-64-95", 64, 19), /* 83..95 not routed to pin */
	GPIO_DATA("GPIO-96-127", 96, 6), /* 102..127 not routed to pin */
	GPIO_DATA("GPIO-128-159", 128, 21), /* 149..159 not routed to pin */
	GPIO_DATA("GPIO-160-191", 160, 32),
	GPIO_DATA("GPIO-192-223", 192, 32),
	GPIO_DATA("GPIO-224-255", 224, 4), /* 228..255 not routed to pin */
};

static struct resource u5500_gpio_resources[] = {
	GPIO_RESOURCE(0),
	GPIO_RESOURCE(1),
	GPIO_RESOURCE(2),
	GPIO_RESOURCE(3),
	GPIO_RESOURCE(4),
	GPIO_RESOURCE(5),
	GPIO_RESOURCE(6),
	GPIO_RESOURCE(7),
};

struct platform_device u5500_gpio_devs[] = {
	GPIO_DEVICE(0),
	GPIO_DEVICE(1),
	GPIO_DEVICE(2),
	GPIO_DEVICE(3),
	GPIO_DEVICE(4),
	GPIO_DEVICE(5),
	GPIO_DEVICE(6),
	GPIO_DEVICE(7),
};

#define U5500_PWM_SIZE 0x20
static struct resource u5500_pwm0_resource[] = {
	{
		.name = "PWM_BASE",
		.start = U5500_PWM_BASE,
		.end = U5500_PWM_BASE + U5500_PWM_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct resource u5500_pwm1_resource[] = {
	{
		.name = "PWM_BASE",
		.start = U5500_PWM_BASE + U5500_PWM_SIZE,
		.end = U5500_PWM_BASE + U5500_PWM_SIZE * 2 - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct resource u5500_pwm2_resource[] = {
	{
		.name = "PWM_BASE",
		.start = U5500_PWM_BASE + U5500_PWM_SIZE * 2,
		.end = U5500_PWM_BASE + U5500_PWM_SIZE * 3 - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct resource u5500_pwm3_resource[] = {
	{
		.name = "PWM_BASE",
		.start = U5500_PWM_BASE + U5500_PWM_SIZE * 3,
		.end = U5500_PWM_BASE + U5500_PWM_SIZE * 4 - 1,
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device u5500_pwm0_device = {
	.id = 0,
	.name = "pwm",
	.resource = u5500_pwm0_resource,
	.num_resources = ARRAY_SIZE(u5500_pwm0_resource),
};

struct platform_device u5500_pwm1_device = {
	.id = 1,
	.name = "pwm",
	.resource = u5500_pwm1_resource,
	.num_resources = ARRAY_SIZE(u5500_pwm1_resource),
};

struct platform_device u5500_pwm2_device = {
	.id = 2,
	.name = "pwm",
	.resource = u5500_pwm2_resource,
	.num_resources = ARRAY_SIZE(u5500_pwm2_resource),
};

struct platform_device u5500_pwm3_device = {
	.id = 3,
	.name = "pwm",
	.resource = u5500_pwm3_resource,
	.num_resources = ARRAY_SIZE(u5500_pwm3_resource),
};

static struct resource mcde_resources[] = {
	[0] = {
		.name  = MCDE_IO_AREA,
		.start = U5500_MCDE_BASE,
		.end   = U5500_MCDE_BASE + U5500_MCDE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name  = MCDE_IO_AREA,
		.start = U5500_DSI_LINK1_BASE,
		.end   = U5500_DSI_LINK1_BASE + U5500_DSI_LINK_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[2] = {
		.name  = MCDE_IO_AREA,
		.start = U5500_DSI_LINK2_BASE,
		.end   = U5500_DSI_LINK2_BASE + U5500_DSI_LINK_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[3] = {
		.name  = MCDE_IRQ,
		.start = IRQ_DB5500_DISP,
		.end   = IRQ_DB5500_DISP,
		.flags = IORESOURCE_IRQ,
	},
};

static int mcde_platform_enable_dsipll(void)
{
	return prcmu_enable_dsipll();
}

static int mcde_platform_disable_dsipll(void)
{
	return prcmu_disable_dsipll();
}

static int mcde_platform_set_display_clocks(void)
{
	return prcmu_set_display_clocks();
}

static struct mcde_platform_data mcde_pdata = {
	.num_dsilinks = 2,
	.syncmux = 0x01,
	.num_channels = 2,
	.num_overlays = 3,
	.regulator_vana_id = "v-ana",
	.regulator_mcde_epod_id = "vsupply",
	.regulator_esram_epod_id = "v-esram12",
	.clock_dsi_id = "hdmi",
	.clock_dsi_lp_id = "tv",
	.clock_mcde_id = "mcde",
	.platform_set_clocks = mcde_platform_set_display_clocks,
	.platform_enable_dsipll = mcde_platform_enable_dsipll,
	.platform_disable_dsipll = mcde_platform_disable_dsipll,
};

struct platform_device ux500_mcde_device = {
	.name = "mcde",
	.id = -1,
	.dev = {
		.platform_data = &mcde_pdata,
	},
	.num_resources = ARRAY_SIZE(mcde_resources),
	.resource = mcde_resources,
};

static struct resource b2r2_resources[] = {
	[0] = {
		.start	= U5500_B2R2_BASE,
		.end	= U5500_B2R2_BASE + ((4*1024)-1),
		.name	= "b2r2_base",
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.name  = "B2R2_IRQ",
		.start = IRQ_DB5500_B2R2,
		.end   = IRQ_DB5500_B2R2,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ux500_b2r2_device = {
	.name	= "b2r2",
	.id	= 0,
	.dev	= {
		.init_name = "b2r2_bus",
		.coherent_dma_mask = ~0,
	},
	.num_resources	= ARRAY_SIZE(b2r2_resources),
	.resource	= b2r2_resources,
};

#if  defined(CONFIG_USB_MUSB_HOST)
#define MUSB_MODE	MUSB_HOST
#elif defined(CONFIG_USB_MUSB_PERIPHERAL)
#define MUSB_MODE	MUSB_PERIPHERAL
#elif defined(CONFIG_USB_MUSB_OTG)
#define MUSB_MODE	MUSB_OTG
#else
#define MUSB_MODE	MUSB_UNDEFINED
#endif
static struct musb_hdrc_config musb_hdrc_hs_otg_config = {
	.multipoint	= true,	/* multipoint device */
	.dyn_fifo	= true,	/* supports dynamic fifo sizing */
	.num_eps	= 16,	/* number of endpoints _with_ ep0 */
	.ram_bits	= 16,	/* ram address size */
};

static struct musb_hdrc_platform_data musb_hdrc_hs_otg_platform_data = {
	.mode	= MUSB_MODE,
	.clock	= "usb",	/* for clk_get() */
	.config = &musb_hdrc_hs_otg_config,
};

static struct resource usb_resources[] = {
	[0] = {
		.name	= "usb-mem",
		.start	=  U5500_USBOTG_BASE,
		.end	=  (U5500_USBOTG_BASE + SZ_64K - 1),
		.flags	=  IORESOURCE_MEM,
	},

	[1] = {
		.name   = "usb-irq",
		.start	= IRQ_DB5500_USBOTG,
		.end	= IRQ_DB5500_USBOTG,
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 musb_dmamask = DMA_BIT_MASK(32);

struct platform_device ux500_musb_device = {
	.name = "musb_hdrc",
	.id = 0,
	.dev = {
		.init_name	= "musb_hdrc.0",	/* for clk_get() */
		.platform_data = &musb_hdrc_hs_otg_platform_data,
		.dma_mask = &musb_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(32)
	},
	.num_resources = ARRAY_SIZE(usb_resources),
	.resource = usb_resources,
};

#ifdef CONFIG_STM_TRACE
/* TODO */
static void stm_disable_modem_on_mipi34(void)
{
}

/* TODO */
static void stm_enable_modem_on_mipi34(void)
{
}

/* TODO */
static int stm_enable_mipi34(void)
{
	return 0;
}

/* TODO */
static int stm_disable_mipi34(void)
{
	return 0;
}

/* TODO */
static int stm_enable_ape_modem_mipi60(void)
{
	return 0;
}

/* TODO */
static int stm_disable_ape_modem_mipi60(void)
{
	return 0;
}

static struct stm_platform_data stm_pdata = {
	.regs_phys_base      = U5500_STM_REG_BASE,
	.channels_phys_base  = U5500_STM_BASE,
	.periph_id           = 0xEC0D3800,
	.cell_id             = 0x0DF005B1,
	/*
	 * These are the channels used by NMF and some external softwares
	 * expect the NMF traces to be output on these channels
	 * For legacy reason, we need to reserve them.
	 * NMF channels reserved HOSTEE (151)
	 */
	.channels_reserved   = {151, -1},
	.ste_enable_modem_on_mipi34 = stm_enable_modem_on_mipi34,
	.ste_disable_modem_on_mipi34 = stm_disable_modem_on_mipi34,
	.ste_gpio_enable_mipi34 = stm_enable_mipi34,
	.ste_gpio_disable_mipi34 = stm_disable_mipi34,
	.ste_gpio_enable_ape_modem_mipi60 = stm_enable_ape_modem_mipi60,
	.ste_gpio_disable_ape_modem_mipi60 = stm_disable_ape_modem_mipi60,
};

struct platform_device ux500_stm_device = {
	.name = "stm",
	.id = -1,
	.dev = {
		.platform_data = &stm_pdata,
	},
};
#endif /* CONFIG_STM_TRACE */
