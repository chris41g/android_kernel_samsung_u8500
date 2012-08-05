/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * License Terms: GNU General Public License v2
 * Author: Sundar Iyer <sundar.iyer@stericsson.com> for ST-Ericsson
 *
 * AB8500 Power-On Key handler
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/slab.h>

struct ab8500_ponkey_variant {
	const char *irq_falling;
	const char *irq_rising;
};

static const struct ab8500_ponkey_variant ab5500_onswa = {
	.irq_falling	= "ONSWAn_falling",
	.irq_rising	= "ONSWAn_rising",
};

static const struct ab8500_ponkey_variant ab8500_ponkey = {
	.irq_falling	= "ONKEY_DBF",
	.irq_rising	= "ONKEY_DBR",
};

/**
 * struct ab8500_ponkey_info - ab8500 ponkey information
 * @input_dev: pointer to input device
 * @irq_dbf: irq number for falling transition
 * @irq_dbr: irq number for rising transition
 */
struct ab8500_ponkey_info {
	struct input_dev	*idev;
	int			irq_dbf;
	int			irq_dbr;
};

/* For forced upload mode. From gpio_keys.c */
extern bool gpio_keys_getstate(int keycode);
extern void gpio_keys_start_upload_modtimer(void);
extern void gpio_keys_setstate(int keycode, bool bState);

/* For forced upload mode, change keys to enter mode */
extern int jack_is_detected;

/* sysfs */
static ssize_t key_pressed_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	if (gpio_keys_getstate(KEY_POWER))
		sprintf(buf, "PRESS");
	else
		sprintf(buf, "RELEASE");

	return strlen(buf);
}

static DEVICE_ATTR(key_pressed, 0664, key_pressed_show, NULL);

static struct attribute *ab8500_ponkey_attributes[] = {
	&dev_attr_key_pressed.attr,
	NULL,
};

static struct attribute_group ab8500_ponkey_attr_group = {
	.attrs = ab8500_ponkey_attributes,
};


/* AB8500 gives us an interrupt when ONKEY is held */
static irqreturn_t ab8500_ponkey_handler(int irq, void *data)
{
	struct ab8500_ponkey_info *info = data;

	if (irq == info->irq_dbf) {
#ifndef CONFIG_SAMSUNG_PRODUCT_SHIP
		if (gpio_keys_getstate(KEY_VOLUMEUP) && jack_is_detected)
			gpio_keys_start_upload_modtimer();
#endif

		gpio_keys_setstate(KEY_POWER, true);
		input_report_key(info->idev, KEY_POWER, true);
#if !defined(CONFIG_SAMSUNG_PRODUCT_SHIP)
		dev_info(info->idev->dev.parent, "Power KEY pressed %d\n", KEY_POWER);
#endif
	} else if (irq == info->irq_dbr) {
		/* For Forced Upload Mode */
		gpio_keys_setstate(KEY_POWER, false);
		input_report_key(info->idev, KEY_POWER, false);
#if !defined(CONFIG_SAMSUNG_PRODUCT_SHIP)
		dev_info(info->idev->dev.parent, "Power KEY released %d\n", KEY_POWER);
#endif
	}

	input_sync(info->idev);

	return IRQ_HANDLED;
}

static int __devinit ab8500_ponkey_probe(struct platform_device *pdev)
{
	const struct ab8500_ponkey_variant *variant;
	struct ab8500_ponkey_info *info;
	int irq_dbf, irq_dbr, ret;

	variant = (const struct ab8500_ponkey_variant *)
		  pdev->id_entry->driver_data;

	irq_dbf = platform_get_irq_byname(pdev, variant->irq_falling);
	if (irq_dbf < 0) {
		dev_err(&pdev->dev, "No IRQ for %s: %d\n",
			variant->irq_falling, irq_dbf);
		return irq_dbf;
	}

	irq_dbr = platform_get_irq_byname(pdev, variant->irq_rising);
	if (irq_dbr < 0) {
		dev_err(&pdev->dev, "No IRQ for %s: %d\n",
			variant->irq_rising, irq_dbr);
		return irq_dbr;
	}

	info = kzalloc(sizeof(struct ab8500_ponkey_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->irq_dbf = irq_dbf;
	info->irq_dbr = irq_dbr;

	info->idev = input_allocate_device();
	if (!info->idev) {
		dev_err(&pdev->dev, "Failed to allocate input dev\n");
		ret = -ENOMEM;
		goto out;
	}

	info->idev->name = "AB8500 POn(PowerOn) Key";
	info->idev->dev.parent = &pdev->dev;
	info->idev->evbit[0] = BIT_MASK(EV_KEY);
	info->idev->keybit[BIT_WORD(KEY_POWER)] = BIT_MASK(KEY_POWER);

	ret = input_register_device(info->idev);
	if (ret) {
		dev_err(&pdev->dev, "Can't register input device: %d\n", ret);
		goto out_unfreedevice;
	}

	ret = request_threaded_irq(info->irq_dbf, NULL, ab8500_ponkey_handler,
			IRQF_SHARED | IRQF_NO_SUSPEND, "ab8500-ponkey-dbf",
					info);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to request dbf IRQ#%d: %d\n",
				info->irq_dbf, ret);
		goto out_unregisterdevice;
	}

	ret = request_threaded_irq(info->irq_dbr, NULL, ab8500_ponkey_handler,
			IRQF_SHARED | IRQF_NO_SUSPEND, "ab8500-ponkey-dbr",
					info);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to request dbr IRQ#%d: %d\n",
				info->irq_dbr, ret);
		goto out_irq_dbf;
	}

	/* sysfs */
	ret = sysfs_create_group(&pdev->dev.kobj, &ab8500_ponkey_attr_group);
	if (ret) {
		dev_err(&pdev->dev, "Unable to export keys, error: %d\n", ret);
		goto out_remove_attr_group;
	}

	platform_set_drvdata(pdev, info);

	return 0;


out_remove_attr_group:
	sysfs_remove_group(&pdev->dev.kobj, &ab8500_ponkey_attr_group);
out_irq_dbf:
	free_irq(info->irq_dbf, info);
out_unregisterdevice:
	input_unregister_device(info->idev);
	info->idev = NULL;
out_unfreedevice:
	input_free_device(info->idev);
out:
	kfree(info);
	return ret;
}

static int __devexit ab8500_ponkey_remove(struct platform_device *pdev)
{
	struct ab8500_ponkey_info *info = platform_get_drvdata(pdev);

	free_irq(info->irq_dbf, info);
	free_irq(info->irq_dbr, info);
	input_unregister_device(info->idev);
	sysfs_remove_group(&pdev->dev.kobj, &ab8500_ponkey_attr_group);
	kfree(info);
	return 0;
}

static struct platform_device_id ab8500_ponkey_id_table[] = {
	{ "ab5500-onswa", (kernel_ulong_t)&ab5500_onswa, },
	{ "ab8500-poweron-key", (kernel_ulong_t)&ab8500_ponkey, },
	{ },
};
MODULE_DEVICE_TABLE(platform, ab8500_ponkey_id_table);

static struct platform_driver ab8500_ponkey_driver = {
	.driver		= {
		.name	= "ab8500-poweron-key",
		.owner	= THIS_MODULE,
	},
	.id_table	= ab8500_ponkey_id_table,
	.probe		= ab8500_ponkey_probe,
	.remove		= __devexit_p(ab8500_ponkey_remove),
};

static int __init ab8500_ponkey_init(void)
{
	return platform_driver_register(&ab8500_ponkey_driver);
}
module_init(ab8500_ponkey_init);

static void __exit ab8500_ponkey_exit(void)
{
	platform_driver_unregister(&ab8500_ponkey_driver);
}
module_exit(ab8500_ponkey_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Sundar Iyer <sundar.iyer@stericsson.com>");
MODULE_DESCRIPTION("ST-Ericsson AB8500 Power-ON(Pon) Key driver");
