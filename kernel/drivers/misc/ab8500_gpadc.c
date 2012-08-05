/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * License Terms: GNU General Public License v2
 * Author: Arun R Murthy <arun.murthy@stericsson.com>
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/completion.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/mfd/ab8500.h>
#include <linux/mfd/abx500.h>
#include <linux/mfd/ab8500/ab8500-gpadc.h>

/*
 * GPADC register offsets
 * Bank : 0x0A
 */
#define AB8500_GPADC_CTRL1_REG		0x00
#define AB8500_GPADC_CTRL2_REG		0x01
#define AB8500_GPADC_CTRL3_REG		0x02
#define AB8500_GPADC_AUTO_TIMER_REG	0x03
#define AB8500_GPADC_STAT_REG		0x04
#define AB8500_GPADC_MANDATAL_REG	0x05
#define AB8500_GPADC_MANDATAH_REG	0x06
#define AB8500_GPADC_AUTODATAL_REG	0x07
#define AB8500_GPADC_AUTODATAH_REG	0x08
#define AB8500_GPADC_MUX_CTRL_REG	0x09

/* gpadc constants */
#define EN_VINTCORE12			0x04
#define EN_VTVOUT			0x02
#define EN_GPADC			0x01
#define DIS_GPADC			0x00
#define SW_AVG_16			0x60
#define ADC_SW_CONV			0x04
#define EN_BUF				0x40
#define DIS_ZERO			0x00
#define GPADC_BUSY			0x01

/**
 * ab8500_gpadc_convert() - gpadc conversion
 * @input:	analog input to be converted to digital data
 *
 * This function converts the selected analog i/p to digital
 * data. Thereafter calibration has to be made to obtain the
 * data in the required quantity measurement.
 */
int ab8500_gpadc_convert(struct ab8500_gpadc *di, u8 input)
{
	int ret, data = 0, looplimit = 0;
	u8 val, low_data, high_data;

	if (!di)
		return -ENODEV;

	mutex_lock(&di->ab8500_gpadc_lock);
	/* Enable VTVout LDO this is required for GPADC */
	regulator_enable(di->regu);

	/* Check if ADC is not busy, lock and proceed */
	do {
		ret = abx500_get_register_interruptible(di->dev, AB8500_GPADC,
			AB8500_GPADC_STAT_REG, &val);
		if (ret < 0)
			goto out;
		msleep(10);
	} while ((val & GPADC_BUSY) && ++looplimit < 10);
	if (looplimit >= 10 && (val & GPADC_BUSY)) {
		dev_err(di->dev, "gpadc_conversion: GPADC busy");
		ret = -EINVAL;
		goto out;
	}

	/* Enable GPADC */
	ret = abx500_mask_and_set_register_interruptible(di->dev, AB8500_GPADC,
		AB8500_GPADC_CTRL1_REG, EN_GPADC, EN_GPADC);
	if (ret < 0) {
		dev_err(di->dev, "gpadc_conversion: enable gpadc failed\n");
		goto out;
	}
	/* Select the input source and set average samples to 16 */
	ret = abx500_set_register_interruptible(di->dev, AB8500_GPADC,
		AB8500_GPADC_CTRL2_REG, (input | SW_AVG_16));
	if (ret < 0) {
		dev_err(di->dev,
			"gpadc_conversion: set avg samples failed\n");
		goto out;
	}
	/* Enable ADC, Buffering and select rising edge, start Conversion */
	ret = abx500_mask_and_set_register_interruptible(di->dev, AB8500_GPADC,
		AB8500_GPADC_CTRL1_REG, EN_BUF, EN_BUF);
	if (ret < 0) {
		dev_err(di->dev,
			"gpadc_conversion: select falling edge failed\n");
		goto out;
	}
	ret = abx500_mask_and_set_register_interruptible(di->dev, AB8500_GPADC,
		AB8500_GPADC_CTRL1_REG, ADC_SW_CONV, ADC_SW_CONV);
	if (ret < 0) {
		dev_err(di->dev,
			"gpadc_conversion: start s/w conversion failed\n");
		goto out;
	}
	/* wait for completion of conversion */
	if (!wait_for_completion_timeout(&di->ab8500_gpadc_complete, 2*HZ)) {
		dev_err(di->dev,
			"timeout: didnt recieve GPADC conversion interrupt\n");
		ret = -EINVAL;
		goto out;
	}

	/* Read the converted RAW data */
	ret = abx500_get_register_interruptible(di->dev, AB8500_GPADC,
		AB8500_GPADC_MANDATAL_REG, &low_data);
	if (ret < 0) {
		dev_err(di->dev, "gpadc_conversion: read low data failed\n");
		goto out;
	}

	ret = abx500_get_register_interruptible(di->dev, AB8500_GPADC,
		AB8500_GPADC_MANDATAH_REG, &high_data);
	if (ret < 0) {
		dev_err(di->dev, "gpadc_conversion: read high data failed\n");
		goto out;
	}

	data = (high_data << 8) | low_data;
	/* Disable GPADC */
	ret = abx500_set_register_interruptible(di->dev, AB8500_GPADC,
		AB8500_GPADC_CTRL1_REG, DIS_GPADC);
	if (ret < 0) {
		dev_err(di->dev, "gpadc_conversion: disable gpadc failed\n");
		goto out;
	}
	/* Disable VTVout LDO this is required for GPADC */
	regulator_disable(di->regu);
	mutex_unlock(&di->ab8500_gpadc_lock);
	return data;

out:
	/*
	 * It has shown to be needed to turn off the GPADC if an error occurs,
	 * otherwise we might have problem when waiting for the busy bit in the
	 * GPADC status register to go low. In V1.1 there wait_for_completion
	 * seems to timeout when waiting for an interrupt.. Not seen in V2.0
	 */
	(void) abx500_set_register_interruptible(di->dev, AB8500_GPADC,
		AB8500_GPADC_CTRL1_REG, DIS_GPADC);
	regulator_disable(di->regu);
	mutex_unlock(&di->ab8500_gpadc_lock);
	dev_err(di->dev, "gpadc_conversion: Failed to AD convert channel %d\n",
		input & 0x1F);
	return ret;
}
EXPORT_SYMBOL(ab8500_gpadc_convert);

/**
 * ab8500_bm_gpswadcconvend_handler() - isr for s/w gpadc conversion completion
 * @irq:	irq number
 * @data:	pointer to the data passed during request irq
 *
 * This is a interrupt service routine for s/w gpadc conversion completion.
 * Notifies the gpadc completion is completed and the converted raw value
 * can be read from the registers.
 * Returns IRQ status(IRQ_HANDLED)
 */
static irqreturn_t ab8500_bm_gpswadcconvend_handler(int irq, void *_di)
{
	struct ab8500_gpadc *di = _di;

	complete(&di->ab8500_gpadc_complete);

	return IRQ_HANDLED;
}

static int __devinit ab8500_gpadc_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct ab8500_gpadc *di;

	di = kzalloc(sizeof(struct ab8500_gpadc), GFP_KERNEL);
	if (!di) {
		dev_err(&pdev->dev, "Error: No memory\n");
		return -ENOMEM;
	}

	di->parent = dev_get_drvdata(pdev->dev.parent);
	di->irq = platform_get_irq_byname(pdev, "SW_CONV_END");
	if (di->irq < 0) {
		dev_err(di->dev, "failed to get platform irq-%d\n", di->irq);
		ret = di->irq;
		goto fail;
	}

	di->dev = &pdev->dev;
	mutex_init(&di->ab8500_gpadc_lock);

	/* Initialize completion used to notify completion of conversion */
	init_completion(&di->ab8500_gpadc_complete);

	/* Register interrupt  - SwAdcComplete */
	ret = request_threaded_irq(di->irq, NULL,
		ab8500_bm_gpswadcconvend_handler,
		IRQF_NO_SUSPEND, "ab8500-gpadc", di);
	if (ret < 0) {
		dev_err(di->dev, "Failed to register interrupt, irq: %d\n",
			di->irq);
		goto fail;
	}

	/* VTVout LDO used to power up ab8500-GPADC */
	di->regu = regulator_get(&pdev->dev, "ab8500-gpadc");
	if (IS_ERR(di->regu)) {
		ret = PTR_ERR(di->regu);
		dev_err(di->dev, "failed to get vtvout LDO\n");
		goto fail;
	}
	di->parent->gpadc = di;
	return 0;
fail:
	kfree(di);
	di = NULL;
	return ret;
}

static int __devexit ab8500_gpadc_remove(struct platform_device *pdev)
{
	struct ab8500_gpadc *di = platform_get_drvdata(pdev);

	/* remove interrupt  - completion of Sw ADC conversion */
	free_irq(di->irq, di);
	/* disable VTVout LDO that is being used by GPADC */
	regulator_put(di->regu);
	kfree(di);
	di = NULL;
	return 0;
}

static struct platform_driver ab8500_gpadc_driver = {
	.probe = ab8500_gpadc_probe,
	.remove = __devexit_p(ab8500_gpadc_remove),
	.driver = {
		.name = "ab8500-gpadc",
		.owner = THIS_MODULE,
	},
};

static int __init ab8500_gpadc_init(void)
{
	return platform_driver_register(&ab8500_gpadc_driver);
}

static void __exit ab8500_gpadc_exit(void)
{
	platform_driver_unregister(&ab8500_gpadc_driver);
}

subsys_initcall_sync(ab8500_gpadc_init);
module_exit(ab8500_gpadc_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Arun R Murthy");
MODULE_ALIAS("platform:ab8500_gpadc");
MODULE_DESCRIPTION("AB8500 GPADC driver");
