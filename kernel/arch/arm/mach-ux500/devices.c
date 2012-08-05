/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * Author: Rabin Vincent <rabin.vincent@stericsson.com> for ST-Ericsson
 * License terms: GNU General Public License (GPL) version 2
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/sysdev.h>
#include <linux/amba/bus.h>
#include <linux/amba/serial.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/usb/musb.h>
#include <linux/amba/bus.h>
#include <linux/dma-mapping.h>
#ifdef CONFIG_ANDROID_PMEM
#include <linux/android_pmem.h>
#endif

#include <asm/irq.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/setup.h>

#include <mach/crypto-ux500.h>
#include <mach/irqs.h>
#include <mach/hardware.h>
#include <mach/devices.h>
#include <mach/setup.h>
#include <linux/hwmem.h>

#include <plat/ste_dma40.h>

#ifdef CONFIG_STE_TRACE_MODEM
#include <linux/db8500-modem-trace.h>
#endif

#ifdef CONFIG_STE_TRACE_MODEM
static struct resource trace_resource = {
	.start	= 0,
	.end	= 0,
	.name	= "db8500-trace-area",
	.flags	= IORESOURCE_MEM
};

static struct db8500_trace_platform_data trace_pdata = {
	.ape_base = U8500_APE_BASE,
	.modem_base = U8500_MODEM_BASE,
};

struct platform_device u8500_trace_modem = {
	.name	= "db8500-modem-trace",
	.id = 0,
	.dev = {
		.init_name = "db8500-modem-trace",
		.platform_data = &trace_pdata,
	},
	.num_resources = 1,
	.resource = &trace_resource,
};

static int __init early_trace_modem(char *p)
{
	struct resource *data = &trace_resource;
	u32 size = memparse(p, &p);
	if (*p == '@')
		data->start = memparse(p + 1, &p);
	data->end = data->start + size -1;
	return 0;
}

early_param("mem_mtrace", early_trace_modem);
#endif

#ifdef CONFIG_HWMEM


static struct hwmem_platform_data hwmem_pdata = {
	.start = 0,
	.size = 0,
};

static int __init early_hwmem(char *p)
{
	hwmem_pdata.size = memparse(p, &p);

	if (*p != '@')
		goto no_at;

	hwmem_pdata.start = memparse(p + 1, &p);

	return 0;

no_at:
	hwmem_pdata.size = 0;

	return -EINVAL;
}
early_param("hwmem", early_hwmem);

struct platform_device ux500_hwmem_device = {
	.name = "hwmem",
	.dev = {
		.platform_data = &hwmem_pdata,
	},
};
#endif

#ifdef CONFIG_ANDROID_PMEM
static int __init early_pmem_generic_parse(char *p,
				   struct android_pmem_platform_data * data)
{
	data->size = memparse(p, &p);
	if (*p == '@')
		data->start = memparse(p + 1, &p);

	return 0;
}

/*
 * Pmem device used by surface flinger
 */
static struct android_pmem_platform_data pmem_pdata = {
	.name = "pmem",
	.no_allocator = 1,	/* MemoryHeapBase is having an allocator */
	.cached = 1,
	.start = 0,
	.size = 0,
};

static int __init early_pmem(char *p)
{
	return early_pmem_generic_parse(p, &pmem_pdata);
}
early_param("pmem", early_pmem);

struct platform_device u8500_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = {
		.platform_data = &pmem_pdata,
	},
};

/*
 * Pmem device used by OMX components allocating buffers
 */
static struct android_pmem_platform_data pmem_hwb_pdata = {
	.name = "pmem_hwb",
	.no_allocator = 1,	/* We'll manage allocation */
	.cached = 1,
	.start = 0,
	.size = 0,
};

static int __init early_pmem_hwb(char *p)
{
	return early_pmem_generic_parse(p, &pmem_hwb_pdata);
}
early_param("pmem_hwb", early_pmem_hwb);

struct platform_device u8500_pmem_hwb_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = {
		.platform_data = &pmem_hwb_pdata,
	},
};
#endif

#ifdef CONFIG_CRYPTO_DEV_UX500_HASH
static struct resource ux500_hash1_resources[] = {
	[0] = {
		.start = U8500_HASH1_BASE,
		.end = U8500_HASH1_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	}
};

struct platform_device ux500_hash1_device = {
	.name = "hash1",
	.id = -1,
	.num_resources = 1,
	.resource = ux500_hash1_resources
};
#endif

static struct resource ux500_cryp1_resources[] = {
	[0] = {
		.start = U8500_CRYP1_BASE,
		.end = U8500_CRYP1_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_DB8500_CRYP1,
		.end = IRQ_DB8500_CRYP1,
		.flags = IORESOURCE_IRQ
	}
};

static struct cryp_platform_data cryp1_platform_data = {
	.mem_to_engine = {
		.dir = STEDMA40_MEM_TO_PERIPH,
		.src_dev_type = STEDMA40_DEV_SRC_MEMORY,
		.dst_dev_type = DB8500_DMA_DEV48_CAC1_TX,
		.src_info.data_width = STEDMA40_WORD_WIDTH,
		.dst_info.data_width = STEDMA40_WORD_WIDTH,
		.mode = STEDMA40_MODE_LOGICAL,
		.src_info.psize = STEDMA40_PSIZE_LOG_4,
		.dst_info.psize = STEDMA40_PSIZE_LOG_4,
	},
	.engine_to_mem = {
		.dir = STEDMA40_PERIPH_TO_MEM,
		.src_dev_type = DB8500_DMA_DEV48_CAC1_RX,
		.dst_dev_type = STEDMA40_DEV_DST_MEMORY,
		.src_info.data_width = STEDMA40_WORD_WIDTH,
		.dst_info.data_width = STEDMA40_WORD_WIDTH,
		.mode = STEDMA40_MODE_LOGICAL,
		.src_info.psize = STEDMA40_PSIZE_LOG_4,
		.dst_info.psize = STEDMA40_PSIZE_LOG_4,
	}
};

struct platform_device ux500_cryp1_device = {
	.name = "cryp1",
	.id = -1,
	.dev = {
		.platform_data = &cryp1_platform_data
	},
	.num_resources = ARRAY_SIZE(ux500_cryp1_resources),
	.resource = ux500_cryp1_resources
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

void __init amba_add_devices(struct amba_device *devs[], int num)
{
	int i;

	for (i = 0; i < num; i++) {
		struct amba_device *d = devs[i];
		amba_device_register(d, &iomem_resource);
	}
}
