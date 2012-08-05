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

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/amba/bus.h>
#include <plat/pincfg.h>
#include <linux/usb/musb.h>
#include <linux/dma-mapping.h>

#include <mach/devices.h>
#include <mach/hardware.h>
#include <mach/setup.h>

#include <video/mcde.h>
#include <mach/prcmu.h>
#include <mach/prcmu-regs.h>
#include <mach/hsi.h>
#include <mach/ste-dma40-db8500.h>

#include <trace/stm.h>

#include "pins-db8500.h"
#include "pm/pm.h"

#include <plat/ste_dma40.h>

/*#include "board-janice-regulators.h"*/ /* TO BE ACTIVATED ON JANICE */

#define GPIO_DATA(_name, first, num)					\
	{								\
		.name		= _name,				\
		.first_gpio	= first,				\
		.first_irq	= NOMADIK_GPIO_TO_IRQ(first),		\
		.num_gpio	= num,					\
		.get_secondary_status = ux500_pm_gpio_read_wake_up_status, \
		.set_ioforce	= ux500_pm_prcmu_set_ioforce,		\
	}

#define GPIO_RESOURCE(block)						\
	{								\
		.start	= U8500_GPIOBANK##block##_BASE,			\
		.end	= U8500_GPIOBANK##block##_BASE + 127,		\
		.flags	= IORESOURCE_MEM,				\
	},								\
	{								\
		.start	= IRQ_DB8500_GPIO##block,			\
		.end	= IRQ_DB8500_GPIO##block,			\
		.flags	= IORESOURCE_IRQ,				\
	},								\
	{								\
		.start	= IRQ_PRCMU_GPIO##block,			\
		.end	= IRQ_PRCMU_GPIO##block,			\
		.flags	= IORESOURCE_IRQ,				\
	}

#define GPIO_DEVICE(block)						\
	{								\
		.name 		= "gpio",				\
		.id		= block,				\
		.num_resources 	= 3,					\
		.resource	= &u8500_gpio_resources[block * 3],	\
		.dev = {						\
			.platform_data = &u8500_gpio_data[block],	\
		},							\
	}

static struct nmk_gpio_platform_data u8500_gpio_data[] = {
	GPIO_DATA("GPIO-0-31", 0, 32),
	GPIO_DATA("GPIO-32-63", 32, 5), /* 37..63 not routed to pin */
	GPIO_DATA("GPIO-64-95", 64, 32),
	GPIO_DATA("GPIO-96-127", 96, 2), /* 98..127 not routed to pin */
	GPIO_DATA("GPIO-128-159", 128, 32),
	GPIO_DATA("GPIO-160-191", 160, 12), /* 172..191 not routed to pin */
	GPIO_DATA("GPIO-192-223", 192, 32),
	GPIO_DATA("GPIO-224-255", 224, 7), /* 231..255 not routed to pin */
	GPIO_DATA("GPIO-256-288", 256, 12), /* 268..288 not routed to pin */
};

static struct resource u8500_gpio_resources[] = {
	GPIO_RESOURCE(0),
	GPIO_RESOURCE(1),
	GPIO_RESOURCE(2),
	GPIO_RESOURCE(3),
	GPIO_RESOURCE(4),
	GPIO_RESOURCE(5),
	GPIO_RESOURCE(6),
	GPIO_RESOURCE(7),
	GPIO_RESOURCE(8),
};

struct platform_device u8500_gpio_devs[] = {
	GPIO_DEVICE(0),
	GPIO_DEVICE(1),
	GPIO_DEVICE(2),
	GPIO_DEVICE(3),
	GPIO_DEVICE(4),
	GPIO_DEVICE(5),
	GPIO_DEVICE(6),
	GPIO_DEVICE(7),
	GPIO_DEVICE(8),
};

static struct resource u8500_shrm_resources[] = {
	[0] = {
		.start = U8500_SHRM_GOP_INTERRUPT_BASE,
		.end = U8500_SHRM_GOP_INTERRUPT_BASE + ((4*4)-1),
		.name = "shrm_gop_register_base",
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_CA_WAKE_REQ_V1,
		.end = IRQ_CA_WAKE_REQ_V1,
		.name = "ca_irq_wake_req",
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.start = IRQ_AC_READ_NOTIFICATION_0_V1,
		.end = IRQ_AC_READ_NOTIFICATION_0_V1,
		.name = "ac_read_notification_0_irq",
		.flags = IORESOURCE_IRQ,
	},
	[3] = {
		.start = IRQ_AC_READ_NOTIFICATION_1_V1,
		.end = IRQ_AC_READ_NOTIFICATION_1_V1,
		.name = "ac_read_notification_1_irq",
		.flags = IORESOURCE_IRQ,
	},
	[4] = {
		.start = IRQ_CA_MSG_PEND_NOTIFICATION_0_V1,
		.end = IRQ_CA_MSG_PEND_NOTIFICATION_0_V1,
		.name = "ca_msg_pending_notification_0_irq",
		.flags = IORESOURCE_IRQ,
	},
	[5] = {
		.start = IRQ_CA_MSG_PEND_NOTIFICATION_1_V1,
		.end = IRQ_CA_MSG_PEND_NOTIFICATION_1_V1,
		.name = "ca_msg_pending_notification_1_irq",
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device u8500_shrm_device = {
	.name = "u8500_shrm",
	.id = 0,
	.dev = {
		.init_name = "shrm_bus",
		.coherent_dma_mask = ~0,
	},

	.num_resources = ARRAY_SIZE(u8500_shrm_resources),
	.resource = u8500_shrm_resources
};

static struct resource mcde_resources[] = {
	[0] = {
		.name  = MCDE_IO_AREA,
		.start = U8500_MCDE_BASE,
		.end   = U8500_MCDE_BASE + U8500_MCDE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name  = MCDE_IO_AREA,
		.start = U8500_DSI_LINK1_BASE,
		.end   = U8500_DSI_LINK1_BASE + U8500_DSI_LINK_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[2] = {
		.name  = MCDE_IO_AREA,
		.start = U8500_DSI_LINK2_BASE,
		.end   = U8500_DSI_LINK2_BASE + U8500_DSI_LINK_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[3] = {
		.name  = MCDE_IO_AREA,
		.start = U8500_DSI_LINK3_BASE,
		.end   = U8500_DSI_LINK3_BASE + U8500_DSI_LINK_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[4] = {
		.name  = MCDE_IRQ,
		.start = IRQ_DB8500_DISP,
		.end   = IRQ_DB8500_DISP,
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
	.num_dsilinks = 3,
	/*
	 * [0] = 3: 24 bits DPI: connect LSB Ch B to D[0:7]
	 * [3] = 4: 24 bits DPI: connect MID Ch B to D[24:31]
	 * [4] = 5: 24 bits DPI: connect MSB Ch B to D[32:39]
	 *
	 * [1] = 3: TV out     : connect LSB Ch B to D[8:15]
	 */
#define DONT_CARE 0
#if defined(CONFIG_MACH_SAMSUNG_U8500)
	/* Ch A LSB D[0:7], MID D[8:15], MSB D[32:39] */
	.outmux = { 0, 1, DONT_CARE, DONT_CARE, 2 },
#else
	.outmux = { 3, 3, DONT_CARE, 4, 5 },
#endif
#undef DONT_CARE
	.syncmux = 0x00,  /* DPI channel A and B on output pins A and B resp */
	.num_channels = 4,
	.num_overlays = 6,
	.regulator_vana_id = "v-ana",
	.regulator_mcde_epod_id = "vsupply",
	.regulator_esram_epod_id = "v-esram34",
	.clock_dsi_id = "hdmi",
	.clock_dsi_lp_id = "tv",
	.clock_dpi_id = "lcd",
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
		.start	= U8500_B2R2_BASE,
		.end	= U8500_B2R2_BASE + ((4*1024)-1),
		.name	= "b2r2_base",
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.name  = "B2R2_IRQ",
		.start = IRQ_DB8500_B2R2,
		.end   = IRQ_DB8500_B2R2,
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
		.start	=  U8500_USBOTG_BASE,
		.end	=  (U8500_USBOTG_BASE + SZ_64K - 1),
		.flags	=  IORESOURCE_MEM,
	},

	[1] = {
		.name   = "usb-irq",
		.start	= IRQ_DB8500_USBOTG,
		.end	= IRQ_DB8500_USBOTG,
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 musb_dmamask = DMA_BIT_MASK(32);

/*
 * WATCHDOG
 */

static struct resource ux500_wdt_resources[] = {
	[0] = {
		.start  = U8500_TWD_BASE,
		.end    = U8500_TWD_BASE+0x37,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = IRQ_LOCALWDOG,
		.end  = IRQ_LOCALWDOG,
		.flags  = IORESOURCE_IRQ,
	}
};

struct platform_device ux500_wdt_device = {
	.name           = "mpcore_wdt",
	.id             = -1,
	.resource       = ux500_wdt_resources,
	.num_resources  = ARRAY_SIZE(ux500_wdt_resources),
};

/*
 * HSI
 */
#define HSIR_OVERRUN(num) {			    \
	.start  = IRQ_DB8500_HSIR_CH##num##_OVRRUN, \
	.end    = IRQ_DB8500_HSIR_CH##num##_OVRRUN, \
	.flags  = IORESOURCE_IRQ,		    \
	.name   = "hsi_rx_overrun_ch"#num	    \
}

#define STE_HSI_PORT0_TX_CHANNEL_CFG(n) { \
       .dir = STEDMA40_MEM_TO_PERIPH, \
       .high_priority = false, \
       .mode = STEDMA40_MODE_LOGICAL, \
       .mode_opt = STEDMA40_LCHAN_SRC_LOG_DST_LOG, \
       .src_dev_type = STEDMA40_DEV_SRC_MEMORY, \
       .dst_dev_type = n,\
       .src_info.big_endian = false,\
       .src_info.data_width = STEDMA40_WORD_WIDTH,\
       .dst_info.big_endian = false,\
       .dst_info.data_width = STEDMA40_WORD_WIDTH,\
},

#define STE_HSI_PORT0_RX_CHANNEL_CFG(n) { \
       .dir = STEDMA40_PERIPH_TO_MEM, \
       .high_priority = false, \
       .mode = STEDMA40_MODE_LOGICAL, \
       .mode_opt = STEDMA40_LCHAN_SRC_LOG_DST_LOG, \
       .src_dev_type = n,\
       .dst_dev_type = STEDMA40_DEV_DST_MEMORY, \
       .src_info.big_endian = false,\
       .src_info.data_width = STEDMA40_WORD_WIDTH,\
       .dst_info.big_endian = false,\
       .dst_info.data_width = STEDMA40_WORD_WIDTH,\
},

static struct resource u8500_hsi_resources[] = {
       {
	       .start  = U8500_HSIR_BASE,
	       .end    = U8500_HSIR_BASE + SZ_4K - 1,
	       .flags  = IORESOURCE_MEM,
	       .name   = "hsi_rx_base"
       },
       {
	       .start  = U8500_HSIT_BASE,
	       .end    = U8500_HSIT_BASE + SZ_4K - 1,
	       .flags  = IORESOURCE_MEM,
	       .name   = "hsi_tx_base"
       },
       {
	       .start  = IRQ_DB8500_HSIRD0,
	       .end    = IRQ_DB8500_HSIRD0,
	       .flags  = IORESOURCE_IRQ,
	       .name   = "hsi_rx_irq0"
       },
       {
	       .start  = IRQ_DB8500_HSITD0,
	       .end    = IRQ_DB8500_HSITD0,
	       .flags  = IORESOURCE_IRQ,
	       .name   = "hsi_tx_irq0"
       },
       {
	       .start  = IRQ_DB8500_HSIR_EXCEP,
	       .end    = IRQ_DB8500_HSIR_EXCEP,
	       .flags  = IORESOURCE_IRQ,
	       .name   = "hsi_rx_excep0"
       },
       HSIR_OVERRUN(0),
       HSIR_OVERRUN(1),
       HSIR_OVERRUN(2),
       HSIR_OVERRUN(3),
       HSIR_OVERRUN(4),
       HSIR_OVERRUN(5),
       HSIR_OVERRUN(6),
       HSIR_OVERRUN(7),
};

#ifdef CONFIG_STE_DMA40
static struct stedma40_chan_cfg ste_hsi_port0_dma_tx_cfg[] = {
       STE_HSI_PORT0_TX_CHANNEL_CFG(DB8500_DMA_DEV20_SLIM0_CH0_TX_HSI_TX_CH0)
       STE_HSI_PORT0_TX_CHANNEL_CFG(DB8500_DMA_DEV21_SLIM0_CH1_TX_HSI_TX_CH1)
       STE_HSI_PORT0_TX_CHANNEL_CFG(DB8500_DMA_DEV22_SLIM0_CH2_TX_HSI_TX_CH2)
       STE_HSI_PORT0_TX_CHANNEL_CFG(DB8500_DMA_DEV23_SLIM0_CH3_TX_HSI_TX_CH3)
};

static struct stedma40_chan_cfg ste_hsi_port0_dma_rx_cfg[] = {
       STE_HSI_PORT0_RX_CHANNEL_CFG(DB8500_DMA_DEV20_SLIM0_CH0_RX_HSI_RX_CH0)
       STE_HSI_PORT0_RX_CHANNEL_CFG(DB8500_DMA_DEV21_SLIM0_CH1_RX_HSI_RX_CH1)
       STE_HSI_PORT0_RX_CHANNEL_CFG(DB8500_DMA_DEV22_SLIM0_CH2_RX_HSI_RX_CH2)
       STE_HSI_PORT0_RX_CHANNEL_CFG(DB8500_DMA_DEV23_SLIM0_CH3_RX_HSI_RX_CH3)
};
#endif

static struct ste_hsi_port_cfg ste_hsi_port0_cfg = {
#ifdef CONFIG_STE_DMA40
       .dma_filter = stedma40_filter,
       .dma_tx_cfg = ste_hsi_port0_dma_tx_cfg,
       .dma_rx_cfg = ste_hsi_port0_dma_rx_cfg
#endif
};

struct ste_hsi_platform_data u8500_hsi_platform_data = {
       .num_ports = 1,
       .use_dma = 1,
       .port_cfg = &ste_hsi_port0_cfg,
};

struct platform_device u8500_hsi_device = {
       .dev = {
		.platform_data = &u8500_hsi_platform_data,
       },
       .name = "ste_hsi",
       .id = 0,
       .resource = u8500_hsi_resources,
       .num_resources = ARRAY_SIZE(u8500_hsi_resources)
};

/*
 * Thermal Sensor
 */

static struct resource u8500_thsens_resources[] = {
	{
		.name = "IRQ_HOTMON_LOW",
		.start  = IRQ_PRCMU_HOTMON_LOW,
		.end    = IRQ_PRCMU_HOTMON_LOW,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name = "IRQ_HOTMON_HIGH",
		.start  = IRQ_PRCMU_HOTMON_HIGH,
		.end    = IRQ_PRCMU_HOTMON_HIGH,
		.flags  = IORESOURCE_IRQ,
	},
};

struct platform_device u8500_thsens_device = {
	.name           = "db8500_temp",
	.resource       = u8500_thsens_resources,
	.num_resources  = ARRAY_SIZE(u8500_thsens_resources),
};

#ifdef CONFIG_STM_TRACE
static pin_cfg_t mop500_stm_mipi34_pins[] = {
	GPIO70_STMAPE_CLK | PIN_SLPM_USE_MUX_SETTINGS_IN_SLEEP,
	GPIO71_STMAPE_DAT3 | PIN_SLPM_USE_MUX_SETTINGS_IN_SLEEP,
	GPIO72_STMAPE_DAT2 | PIN_SLPM_USE_MUX_SETTINGS_IN_SLEEP,
	GPIO73_STMAPE_DAT1 | PIN_SLPM_USE_MUX_SETTINGS_IN_SLEEP,
	GPIO74_STMAPE_DAT0 | PIN_SLPM_USE_MUX_SETTINGS_IN_SLEEP,
	GPIO75_U2_RXD | PIN_SLPM_USE_MUX_SETTINGS_IN_SLEEP,
	GPIO76_U2_TXD | PIN_SLPM_USE_MUX_SETTINGS_IN_SLEEP,
};

static pin_cfg_t mop500_stm_mipi60_pins[] = {
	GPIO153_U2_RXD,
	GPIO154_U2_TXD,
	GPIO155_STMAPE_CLK,
	GPIO156_STMAPE_DAT3,
	GPIO157_STMAPE_DAT2,
	GPIO158_STMAPE_DAT1,
	GPIO159_STMAPE_DAT0,
};

static pin_cfg_t mop500_stm_ape_microsd_pins[] = {
	GPIO18_GPIO		| PIN_OUTPUT_LOW,
	GPIO19_GPIO	| PIN_OUTPUT_HIGH,
	GPIO20_GPIO	| PIN_OUTPUT_HIGH,
	GPIO22_GPIO	| PIN_INPUT_NOPULL,
	GPIO23_MS_CLK	| PIN_SLPM_USE_MUX_SETTINGS_IN_SLEEP,
	GPIO24_MS_BS	| PIN_SLPM_USE_MUX_SETTINGS_IN_SLEEP,
	GPIO25_MS_DAT0	| PIN_SLPM_USE_MUX_SETTINGS_IN_SLEEP,
	GPIO26_MS_DAT1	| PIN_SLPM_USE_MUX_SETTINGS_IN_SLEEP,
	GPIO27_MS_DAT2	| PIN_SLPM_USE_MUX_SETTINGS_IN_SLEEP,
	GPIO28_MS_DAT3	| PIN_SLPM_USE_MUX_SETTINGS_IN_SLEEP,
};

static pin_cfg_t mop500_stm_modem_microsd_pins[] = {
	GPIO18_GPIO        | PIN_OUTPUT_LOW,
	GPIO19_GPIO        | PIN_OUTPUT_HIGH,
	GPIO20_GPIO        | PIN_OUTPUT_HIGH,
	GPIO22_GPIO        | PIN_INPUT_NOPULL,
	GPIO23_STMMOD_CLK  | PIN_SLPM_USE_MUX_SETTINGS_IN_SLEEP,
	GPIO24_UARTMOD_RXD | PIN_SLPM_USE_MUX_SETTINGS_IN_SLEEP,
	GPIO25_STMMOD_DAT0 | PIN_SLPM_USE_MUX_SETTINGS_IN_SLEEP,
	GPIO26_STMMOD_DAT1 | PIN_SLPM_USE_MUX_SETTINGS_IN_SLEEP,
	GPIO27_STMMOD_DAT2 | PIN_SLPM_USE_MUX_SETTINGS_IN_SLEEP,
	GPIO28_STMMOD_DAT3 | PIN_SLPM_USE_MUX_SETTINGS_IN_SLEEP,
};

/* sdi0 (removable MMC/SD/SDIO cards) */
static pin_cfg_t mop500_sdi0_pins[] = {
	GPIO18_MC0_CMDDIR	| PIN_OUTPUT_HIGH,
	GPIO19_MC0_DAT0DIR	| PIN_OUTPUT_HIGH,
	GPIO20_MC0_DAT2DIR	| PIN_OUTPUT_HIGH,

	GPIO22_MC0_FBCLK	| PIN_INPUT_NOPULL,
	GPIO23_MC0_CLK		| PIN_OUTPUT_LOW,
	GPIO24_MC0_CMD		| PIN_INPUT_PULLUP,
	GPIO25_MC0_DAT0		| PIN_INPUT_PULLUP,
	GPIO26_MC0_DAT1		| PIN_INPUT_PULLUP,
	GPIO27_MC0_DAT2		| PIN_INPUT_PULLUP,
	GPIO28_MC0_DAT3		| PIN_INPUT_PULLUP,
};

static pin_cfg_t mop500_ske_pins[] = {
	GPIO153_KP_I7,
	GPIO154_KP_I6,
	GPIO155_KP_I5,
	GPIO156_KP_I4,
	GPIO157_KP_O7,
	GPIO158_KP_O6,
	GPIO159_KP_O5,
};

/*
 * Disable Trace of modem on MIPI34 and MIPI60 probe 2
 * ie disable ALT-C3 on GPIO70-76
 */
static void stm_disable_modem_on_mipi34(void)
{
	u32 gpiocr;

	gpiocr = readl(PRCM_GPIOCR);
	/* Disable altC3 on GPIO70-74 (STMMOD) */
	gpiocr &= ~PRCM_GPIOCR_DBG_STM_MOD_CMD1;
	/* Disable altC3 on GPIO75-76 (UARTMOD) (?) */
	gpiocr &= ~PRCM_GPIOCR_DBG_UARTMOD_CMD0;
	writel(gpiocr, PRCM_GPIOCR);
}

/*
 * Enable Trace of modem on MIPI34 and MIPI60 probe 2
 * ie enable ALT-C3 on GPIO70-76
 */
static void stm_enable_modem_on_mipi34(void)
{
	u32 gpiocr;

	gpiocr = readl(PRCM_GPIOCR);
	/* Enable altC3 on GPIO70-74 (STMMOD) and GPIO75-76 (UARTMOD) */
	gpiocr |= (PRCM_GPIOCR_DBG_STM_MOD_CMD1 | PRCM_GPIOCR_DBG_UARTMOD_CMD0);
	writel(gpiocr, PRCM_GPIOCR);
}

static int stm_enable_mipi34(void)
{
	int retval;

	retval = nmk_config_pins(ARRAY_AND_SIZE(mop500_stm_mipi34_pins));
	if (retval)
		pr_err("STM: Failed to enable MIPI34");
	return retval;
}

static int stm_disable_mipi34(void)
{
	int retval;

	retval = nmk_config_pins_sleep(ARRAY_AND_SIZE(mop500_stm_mipi34_pins));
	if (retval)
		pr_err("STM: Failed to disable MIPI34");
	return retval;
}

static int stm_enable_ape_modem_mipi60(void)
{
	int retval;

	retval = nmk_config_pins_sleep(ARRAY_AND_SIZE(mop500_ske_pins));
	if (retval)
		pr_err("STM: Failed to disable SKE GPIO");
	else {
		retval =
			nmk_config_pins(ARRAY_AND_SIZE(mop500_stm_mipi60_pins));
		if (retval)
			pr_err("STM: Failed to enable MIPI60");
	}
	return retval;
}

static int stm_disable_ape_modem_mipi60(void)
{
	int retval;

	retval = nmk_config_pins_sleep(ARRAY_AND_SIZE(mop500_stm_mipi60_pins));
	if (retval)
		pr_err("STM: Failed to disable MIPI60");
	else {
		retval = nmk_config_pins(ARRAY_AND_SIZE(mop500_ske_pins));
		if (retval)
			pr_err("STM: Failed to enable SKE gpio");
	}
	return retval;
}

static int stm_enable_ape_microsd(void)
{
	int retval;
	u32 gpiocr;

	/*
	 * Configure STM APE on GPIO23,GPIO28,GPIO27,GPIO26,GPIO25
	 * On HREF board an external SD buffer exist (ST6G3244ME)
	 * to perform level conversion from 1.8v to 3.3V on SD card signals
	 * When STM is redirected on micro SD connector GPIO18,GP19,GPIO20
	 * are configured in standard GPIO mode and are used to configure
	 * direction on external SD buffer ST6G3244ME.
	 */

	retval = nmk_config_pins(ARRAY_AND_SIZE(mop500_stm_ape_microsd_pins));
	if (retval)
		pr_err("STM: Failed to enable STM APE on MICRO SD\n");


	gpiocr = readl(PRCM_GPIOCR);
	/* Enable altC1 on GPIO23-28 (STMAPE) */
	gpiocr |= PRCM_GPIOCR_DBG_STM_APE_CMD;
	writel(gpiocr, PRCM_GPIOCR);

	return retval;
}

static int stm_disable_ape_microsd(void)
{
	int retval;
	u32 gpiocr;

	/* Disable altC1 on GPIO23-28 (STMAPE) */
	gpiocr = readl(PRCM_GPIOCR);
	gpiocr &= ~PRCM_GPIOCR_DBG_STM_APE_CMD;
	writel(gpiocr, PRCM_GPIOCR);

	/* Reconfigure GPIO for SD */
	retval = nmk_config_pins_sleep(ARRAY_AND_SIZE(mop500_sdi0_pins));
	if (retval)
		pr_err("STM: Failed to disable STM APE on MICRO SD and to reconfigure GPIO for SD\n");

	return retval;
}

static int stm_enable_modem_microsd(void)
{
	int retval;

	/*
	 * Configure STM APE on GPIO23,GPIO28,GPIO27,GPIO26,GPIO25
	 * On HREF board an external SD buffer exist (ST6G3244ME)
	 * to perform level conversion from 1.8v to 3.3V on SD card
	 * signals. When STM is redirected on micro SD connector
	 * GPIO18,GP19,GPIO20 are configured in standard GPIO mode
	 * and are used to configure direction on external SD buffer
	 * ST6G3244ME.
	 */

	retval = nmk_config_pins(ARRAY_AND_SIZE(mop500_stm_modem_microsd_pins));
	if (retval)
		pr_err("STM: Failed to enable STM MODEM on MICRO SD\n");

	return retval;
}

static int stm_disable_modem_microsd(void)
{
	int retval;

	/* Reconfigure GPIO for SD */
	retval = nmk_config_pins_sleep(ARRAY_AND_SIZE(mop500_sdi0_pins));
	if (retval)
		pr_err("STM: Failed to disable STM MODEM on MICRO SD and to reconfigure GPIO for SD\n");

	return retval;
}

/* Enable regulator and level shifter for micro sd */
static int enable_power_for_microsd(void)
{
		/*janice_regulators[AB8500_LDO_AUX3].constraints.valid_ops_mask = 0;*/ /* TO BE ACTIVATE ON JANICE */
		/*janice_regulators[AB8500_LDO_AUX3].constraints.always_on = 1;*/ /* TO BE ACTIVATE ON JANICE */
		return 0;
}

/* Disable regulator and level shifter for micro sd */
static int disable_power_for_microsd(void)
{
		/*janice_regulators[AB8500_LDO_AUX3].constraints.valid_ops_mask = 0;*/ /* TO BE ACTIVATE ON JANICE */
		/*janice_regulators[AB8500_LDO_AUX3].constraints.always_on = 0;*/ /* TO BE ACTIVATE ON JANICE */
		return 0;
}

static struct stm_platform_data stm_pdata = {
	.regs_phys_base      = U8500_STM_REG_BASE,
	.channels_phys_base  = U8500_STM_BASE,
	.periph_id           = 0xEC0D3800,   /* or 0xEC0D2800 for 8500V1 */
	.cell_id             = 0x0DF005B1,
	/*
	 * These are the channels used by NMF and some external softwares
	 * expect the NMF traces to be output on these channels
	 * For legacy reason, we need to reserve them.
	 * NMF channels reserved resp MPCEE (100), CM (101) & HOSTEE (151)
	 */
	.channels_reserved   = {100, 101, 151, -1},
	.ste_enable_modem_on_mipi34 = stm_enable_modem_on_mipi34,
	.ste_disable_modem_on_mipi34 = stm_disable_modem_on_mipi34,
	.ste_gpio_enable_mipi34 = stm_enable_mipi34,
	.ste_gpio_disable_mipi34 = stm_disable_mipi34,
	.ste_gpio_enable_ape_modem_mipi60 = stm_enable_ape_modem_mipi60,
	.ste_gpio_disable_ape_modem_mipi60 = stm_disable_ape_modem_mipi60,
	.ste_gpio_enable_stm_ape_microsd = stm_enable_ape_microsd,
	.ste_gpio_disable_stm_ape_microsd = stm_disable_ape_microsd,
	.ste_gpio_enable_stm_modem_microsd = stm_enable_modem_microsd,
	.ste_gpio_disable_stm_modem_microsd = stm_disable_modem_microsd,
	.ste_enable_power_for_microsd = enable_power_for_microsd,
	.ste_disable_power_for_microsd = disable_power_for_microsd,
};

struct platform_device ux500_stm_device = {
	.name = "stm",
	.id = -1,
	.dev = {
		.platform_data = &stm_pdata,
	},
};
#endif /* CONFIG_STM_TRACE */

