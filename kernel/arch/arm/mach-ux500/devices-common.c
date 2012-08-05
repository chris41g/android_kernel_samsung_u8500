/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * Author: Rabin Vincent <rabin.vincent@stericsson.com> for ST-Ericsson
 * License terms: GNU General Public License (GPL), version 2.
 */

#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/amba/bus.h>
#include <linux/pm.h>

#include <mach/hardware.h>

#include "pm/pm.h"
#include "devices-common.h"

struct amba_device *
dbx500_add_amba_device(const char *init_name, resource_size_t base,
		       int irq, void *pdata, unsigned int periphid,
		       const char *name)
{
	struct amba_device *dev;
	int ret;

	dev = kzalloc(sizeof *dev, GFP_KERNEL);
	if (!dev)
		return ERR_PTR(-ENOMEM);

	dev->dev.init_name = init_name;

	dev->res.start = base;
	dev->res.end = base + SZ_4K - 1;
	dev->res.flags = IORESOURCE_MEM;

	dev->dma_mask = DMA_BIT_MASK(32);
	dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	dev->dev.pwr_domain = &ux500_dev_power_domain;

	dev->irq[0] = irq;
	dev->irq[1] = NO_IRQ;

	dev->periphid = periphid;
	dev->name = name;

	dev->dev.platform_data = pdata;

	ret = amba_device_register(dev, &iomem_resource);
	if (ret) {
		kfree(dev);
		return ERR_PTR(ret);
	}

	return dev;
}

static struct platform_device *
dbx500_add_platform_device(const char *name, int id, void *pdata,
			   struct resource *res, int resnum)
{
	struct platform_device *dev;
	int ret;

	dev = platform_device_alloc(name, id);
	if (!dev)
		return ERR_PTR(-ENOMEM);

	dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	dev->dev.dma_mask = &dev->dev.coherent_dma_mask;
	dev->dev.pwr_domain = &ux500_dev_power_domain;

	ret = platform_device_add_resources(dev, res, resnum);
	if (ret)
		goto out_free;

	dev->dev.platform_data = pdata;

	ret = platform_device_add(dev);
	if (ret)
		goto out_free;

	return dev;

out_free:
	platform_device_put(dev);
	return ERR_PTR(ret);
}

struct platform_device *
dbx500_add_platform_device_4k1irq(const char *name, int id,
				  resource_size_t base,
				  int irq, void *pdata)
{
	struct resource resources[] = {
		[0] = {
			.start	= base,
			.end	= base + SZ_4K - 1,
			.flags	= IORESOURCE_MEM,
		},
		[1] = {
			.start	= irq,
			.end	= irq,
			.flags	= IORESOURCE_IRQ,
		}
	};

	return dbx500_add_platform_device(name, id, pdata, resources,
					  ARRAY_SIZE(resources));
}
