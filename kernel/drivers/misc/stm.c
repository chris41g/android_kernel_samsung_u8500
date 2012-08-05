/*
 * Copyright (C) ST-Ericsson SA 2010
 * Author: Pierre Peiffer <pierre.peiffer@stericsson.com> for ST-Ericsson.
 * Author: Olivier Germain <olivier.germain@stericsson.com>
 * License terms: GNU General Public License (GPL), version 2.
 */

#include <linux/fs.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <mach/prcmu-regs.h>
#include <mach/gpio.h>
#include <trace/stm.h>

#define STM_CLOCK_SHIFT 6
#define STM_CLOCK_MASK  0x1C0
#define STM_ENABLE_MASK 0x23D
/* Software mode for all cores except PRCMU that doesn't support SW */
#define STM_MMC_DEFAULT 0x20

/* STM Registers */
#define STM_CR		(stm.virtbase)
#define STM_MMC		(stm.virtbase + 0x008)
#define STM_TER		(stm.virtbase + 0x010)
#define STMPERIPHID0	(stm.virtbase + 0xFC0)
#define STMPERIPHID1	(stm.virtbase + 0xFC8)
#define STMPERIPHID2	(stm.virtbase + 0xFD0)
#define STMPERIPHID3	(stm.virtbase + 0xFD8)
#define STMPCELLID0	(stm.virtbase + 0xFE0)
#define STMPCELLID1	(stm.virtbase + 0xFE8)
#define STMPCELLID2	(stm.virtbase + 0xFF0)
#define STMPCELLID3	(stm.virtbase + 0xFF8)

static struct stm_device {
	struct stm_platform_data *pdata;
    struct device dev;
	void __iomem *virtbase;
	volatile struct stm_channel __iomem *channels;
	/* Used to register the allocated channels */
	DECLARE_BITMAP(ch_bitmap, STM_NUMBER_OF_CHANNEL);
} stm;

static DEFINE_MUTEX(lock);

static char *mipi60= "none";
module_param(mipi60, charp, S_IRUGO);
MODULE_PARM_DESC(mipi60, "STM Trace to output on probe2 of mipi60 "
		 "('none' or 'ape' or 'modem')");

static char *mipi34 = "none";
module_param(mipi34, charp, S_IRUGO);
MODULE_PARM_DESC(mipi34, "STM Trace to output on mipi34 "
		 "('none' or 'ape' or 'modem')");

static char *microsd = "none";
module_param(microsd, charp, S_IRUGO);
MODULE_PARM_DESC(microsd, "STM Trace to output on SD card connector "
		 "('none' or 'ape' or 'modem')");

static unsigned int stm_ter;
module_param(stm_ter, uint, 0);
MODULE_PARM_DESC(stm_ter, "Value for STM_TER (trace control register). "
		 "Should be set by user as environment variable stm.stm_ter");

static bool microsd_cfg_done;

#define IS_APE_ON_MIPI34 (mipi34 && !strcmp(mipi34, "ape"))
#define IS_APE_ON_MIPI60 (mipi60 && !strcmp(mipi60, "ape"))
#define IS_APE_ON_MICROSD (microsd && !strcmp(microsd, "ape"))
#define IS_MODEM_ON_MICROSD (microsd && !strcmp(microsd, "modem"))

static int stm_configure_to_microsd(void)
{
	int err = 0;

	if ((IS_APE_ON_MICROSD) || (IS_MODEM_ON_MICROSD)) {

		/* Disable STM on MIPI34 */
		if (IS_APE_ON_MIPI34) {
			if (stm.pdata->ste_gpio_disable_mipi34)
				stm.pdata->ste_gpio_disable_mipi34();
		}

		/* Disable STM on MIPI60 */
		if (IS_APE_ON_MIPI60) {
			if (stm.pdata->ste_gpio_disable_ape_modem_mipi60)
				stm.pdata->ste_gpio_disable_ape_modem_mipi60();
		}

		if (IS_APE_ON_MICROSD) {
			/* Enable STM APE on SD Card connector */
			if (stm.pdata->ste_gpio_enable_stm_ape_microsd)
				err =
				stm.pdata->ste_gpio_enable_stm_ape_microsd();
		} else {
			if (IS_MODEM_ON_MICROSD)
				/* Enable STM APE on SD Card connector */
				if (stm.pdata->
				ste_gpio_enable_stm_modem_microsd)
					err =
					stm.pdata->
					ste_gpio_enable_stm_modem_microsd();
		}

		if (err)
			dev_err(&stm.dev,
			"failed to configure STM on MICRO SD\n");
		else {
			/* Enable SD card buffer and regulator on href */
			if (stm.pdata->
			ste_enable_power_for_microsd)
				stm.pdata->
				ste_enable_power_for_microsd();
		}

	}

	return err;

}

static int stm_open(struct inode *inode, struct file *file)
{
	file->private_data = kzalloc(sizeof(stm.ch_bitmap), GFP_KERNEL);
	if (file->private_data == NULL)
		return -ENOMEM;

	if (microsd_cfg_done)
		return 0;

	if (!stm_configure_to_microsd())
		microsd_cfg_done = true;

	return 0;
}

static int stm_release(struct inode *inode, struct file *filp)
{
	bitmap_andnot(stm.ch_bitmap, stm.ch_bitmap, filp->private_data,
		      STM_NUMBER_OF_CHANNEL);
	kfree(filp->private_data);
	return 0;
}

static int stm_mmap(struct file *filp, struct vm_area_struct *vma)
{
	/*
	 * Don't allow a mapping that covers more than the STM channels
	 * 4096 == sizeof(struct stm_channels)
	 */
	if ((vma->vm_end - vma->vm_start) > SZ_4K)
		return -EINVAL;

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	if (io_remap_pfn_range(vma, vma->vm_start,
			       stm.pdata->channels_phys_base>>PAGE_SHIFT,
			       SZ_4K, vma->vm_page_prot))
		return -EAGAIN ;

	return 0;
}

void stm_disable_src(void)
{
	mutex_lock(&lock);
	writel(0x0, STM_CR);      /* stop clock */
	writel(STM_MMC_DEFAULT, STM_MMC);
	writel(0x0, STM_TER);    /* Disable cores */
	mutex_unlock(&lock);
}
EXPORT_SYMBOL(stm_disable_src);

int stm_set_ckdiv(enum clock_div v)
{
	unsigned int val;

	mutex_lock(&lock);
	val = readl(STM_CR);
	val &= ~STM_CLOCK_MASK;
	writel(val | ((v << STM_CLOCK_SHIFT) & STM_CLOCK_MASK), STM_CR);
	mutex_unlock(&lock);

	return 0;
}
EXPORT_SYMBOL(stm_set_ckdiv);

unsigned int stm_get_cr(void)
{
	return readl(STM_CR);
}
EXPORT_SYMBOL(stm_get_cr);

int stm_enable_src(unsigned int v)
{
	unsigned int val;
	mutex_lock(&lock);
	val = readl(STM_CR);
	val &= ~STM_CLOCK_MASK;
	/* middle possible clock */
	writel(val | (STM_CLOCK_DIV8 << STM_CLOCK_SHIFT), STM_CR);
	writel(STM_MMC_DEFAULT, STM_MMC);
    val = val | (STM_CLOCK_DIV8 << STM_CLOCK_SHIFT);
	/*
	 * If the kernel argument stm_ter has been set by the boot loader
	 * all calls to stm_enable_src will be ignored
	 */
	v = stm_ter ? stm_ter : v;
	writel((v & STM_ENABLE_MASK), STM_TER);
	mutex_unlock(&lock);

	dev_info(&stm.dev,"stm_enable STM_CR=0x%x\n", val);
	dev_info(&stm.dev,"stm_enable STM_MMC=0x%x\n", STM_MMC_DEFAULT);
	dev_info(&stm.dev,"stm_enable STM_TER=0x%x\n", v & STM_ENABLE_MASK);
	return 0;
}
EXPORT_SYMBOL(stm_enable_src);

static int stm_get_channel(struct file *filp, int __user *arg)
{
	int c, err;

	/* Look for a free channel */
	do {
		c = find_first_zero_bit(stm.ch_bitmap, STM_NUMBER_OF_CHANNEL);
	} while ((c < STM_NUMBER_OF_CHANNEL)
		 && test_and_set_bit(c, stm.ch_bitmap));

	if (c < STM_NUMBER_OF_CHANNEL) {
		/* One free found ! */
		err = put_user(c, arg);
		if (err) {
			clear_bit(c, stm.ch_bitmap);
		} else {
			/* Register it in the context of the file */
			unsigned long *local_bitmap = filp->private_data;
			if (local_bitmap)
				set_bit(c, local_bitmap);
		}
	} else {
		err = -ENOMEM;
	}
	return err;
}

static int stm_free_channel(struct file *filp, int channel)
{
	if ((channel < 0) || (channel >= STM_NUMBER_OF_CHANNEL))
		return -EINVAL;
	clear_bit(channel, stm.ch_bitmap);
	if (filp->private_data)
		clear_bit(channel, filp->private_data);
	return 0;
}

static long stm_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;

	switch (cmd) {

	case STM_DISABLE:
		stm_disable_src();
		break;

	case STM_SET_CLOCK_DIV:
		err = stm_set_ckdiv((enum clock_div) arg);
		break;

	case STM_GET_CTRL_REG:
		err = put_user(stm_get_cr(), (unsigned int *)arg);
		break;

	case STM_ENABLE_SRC:
		err = stm_enable_src(arg);
		break;

	case STM_DISABLE_MIPI34_MODEM:
		if (stm.pdata->ste_disable_modem_on_mipi34)
		stm.pdata->ste_disable_modem_on_mipi34();
		break;

	case STM_ENABLE_MIPI34_MODEM:
		if (stm.pdata->ste_enable_modem_on_mipi34)
		stm.pdata->ste_enable_modem_on_mipi34();
		break;

	case STM_GET_FREE_CHANNEL:
		err = stm_get_channel(filp, (int *)arg);
		break;

	case STM_RELEASE_CHANNEL:
		err = stm_free_channel(filp, arg);
		break;

	default:
		err = -EINVAL;
		break;
  }

  return err;
}

#define DEFLLTFUN(size) \
	void  stm_trace_##size(unsigned char channel, uint##size##_t data) \
	{ \
		(__chk_io_ptr(&(stm.channels[channel].no_stamp##size))), \
			*(volatile uint##size##_t __force *)		  \
			(&(stm.channels[channel].no_stamp##size)) = data;\
	}								\
	EXPORT_SYMBOL(stm_trace_##size); \
	void stm_tracet_##size(unsigned char channel, uint##size##_t data) \
	{ \
		(__chk_io_ptr(&(stm.channels[channel].stamp##size))),    \
			*(volatile uint##size##_t __force *)		\
			(&(stm.channels[channel].stamp##size)) = data; } \
	EXPORT_SYMBOL(stm_tracet_##size)

DEFLLTFUN(8);
DEFLLTFUN(16);
DEFLLTFUN(32);
DEFLLTFUN(64);

static const struct file_operations stm_fops = {
	.owner =		THIS_MODULE,
	.unlocked_ioctl =	stm_ioctl,
	.open =			stm_open,
	.llseek =		no_llseek,
	.release =		stm_release,
	.mmap =			stm_mmap,
};

static struct miscdevice stm_misc = {
	.minor          = MISC_DYNAMIC_MINOR,
	.name           = STM_DEV_NAME,
	.fops           = &stm_fops
};

static int __devinit stm_probe(struct platform_device *pdev)
{
	int err;

	if (!pdev || !pdev->dev.platform_data)  {
		pr_alert("No device/platform_data found on STM driver\n");
		return -ENODEV;
	}

	stm.pdata = pdev->dev.platform_data;
	stm.dev = pdev->dev;

	/* Reserve channels if necessary */
	if (stm.pdata->channels_reserved) {
		int i = 0;
		while (stm.pdata->channels_reserved[i] != -1) {
			set_bit(stm.pdata->channels_reserved[i], stm.ch_bitmap);
			i++;
		}
	}

	err = misc_register(&stm_misc);
	if (err) {
		dev_alert(&pdev->dev, "Unable to register misc driver!\n");
		return err;
	}

	stm.virtbase = ioremap_nocache(stm.pdata->regs_phys_base, SZ_4K);
	if (stm.virtbase == NULL) {
		err = -EIO;
		dev_err(&pdev->dev, "could not remap STM Register\n");
		goto fail_init;
	}

	stm.channels =
		ioremap_nocache(stm.pdata->channels_phys_base,
				STM_NUMBER_OF_CHANNEL*sizeof(*stm.channels));
	if (stm.channels == NULL) {
		dev_err(&pdev->dev, "could not remap STM Msg register\n");
		goto fail_init;
	}

	/* Check chip IDs if necessary */
	if (stm.pdata->periph_id && stm.pdata->cell_id) {
		u32 periph_id, cell_id;

		periph_id = (readb(STMPERIPHID0)<<24) +
				 (readb(STMPERIPHID1)<<16) +
				 (readb(STMPERIPHID2)<<8) +
				 readb(STMPERIPHID3);
		cell_id = (readb(STMPCELLID0)<<24) +
				 (readb(STMPCELLID1)<<16) +
				 (readb(STMPCELLID2)<<8) +
				 readb(STMPCELLID3);
		/* Ignore periph id2 field verification */
		if ((stm.pdata->periph_id & 0xFFFF00FF)
				!= (periph_id & 0xFFFF00FF) ||
		    stm.pdata->cell_id != cell_id) {
			dev_err(&pdev->dev, "STM-Trace not supported\n");
			dev_err(&pdev->dev, "periph_id=%x\n", periph_id);
			dev_err(&pdev->dev, "pcell_id=%x\n", cell_id);
			err = -EINVAL;
			goto fail_init;
		}
	}

	if (IS_APE_ON_MIPI60) {
		if (IS_APE_ON_MIPI34) {
			dev_info(&pdev->dev, "Can't not enable APE trace on "
				 "mipi34 and mipi60-probe2: disabling APE on"
				 " mipi34\n");
			mipi34 = "modem";
		}
		if (stm.pdata->ste_gpio_enable_ape_modem_mipi60) {
			err = stm.pdata->ste_gpio_enable_ape_modem_mipi60();
			if (err)
				dev_err(&pdev->dev, "can't enable MIPI60\n");
		}
	}

	if (IS_APE_ON_MIPI34) {
		if (stm.pdata->ste_disable_modem_on_mipi34)
			stm.pdata->ste_disable_modem_on_mipi34();
	} else {
		if (stm.pdata->ste_enable_modem_on_mipi34)
			stm.pdata->ste_enable_modem_on_mipi34();
	}

	if (stm.pdata->ste_gpio_enable_mipi34) {
		err = stm.pdata->ste_gpio_enable_mipi34();
		if (err) {
			dev_err(&pdev->dev, "failed to set GPIO_ALT_TRACE\n");
			goto fail_init;
		}
	}

	dev_info(&stm.dev,"stm_probe mipi34=%s\n ", mipi34);
	dev_info(&stm.dev,"stm_probe mipi60=%s\n ", mipi60);
	dev_info(&stm.dev,"stm_probe microsd=%s\n ", microsd);
	dev_info(&stm.dev,"stm_probe stm_ter=0x%x\n ",stm_ter);

	if (stm_ter)
		stm_enable_src(stm_ter);
	dev_info(&pdev->dev, "STM-Trace driver probed successfully\n");
	return 0;
fail_init:
	if (stm.virtbase)
		iounmap(stm.virtbase);

	if (stm.channels)
		iounmap(stm.channels);
	misc_deregister(&stm_misc);

	return err;
}

static int __devexit stm_remove(struct platform_device *pdev)
{
	struct stm_platform_data *pdata;
	pdata = pdev->dev.platform_data;

	if (pdata->ste_gpio_disable_mipi34)
		pdata->ste_gpio_disable_mipi34();

	stm_disable_src();

	if ((IS_APE_ON_MICROSD) || (IS_MODEM_ON_MICROSD)) {

		if (microsd_cfg_done &&
			pdata->ste_disable_power_for_microsd)
			pdata->ste_disable_power_for_microsd();

		if (IS_APE_ON_MICROSD) {
			if (pdata->ste_gpio_disable_stm_ape_microsd)
				pdata->ste_gpio_disable_stm_ape_microsd();
		} else if (IS_MODEM_ON_MICROSD) {
			if (pdata->ste_gpio_disable_stm_modem_microsd)
				pdata->ste_gpio_disable_stm_modem_microsd();
		}
	}

	if (stm.virtbase)
		iounmap(stm.virtbase);

	if (stm.channels)
		iounmap(stm.channels);

	misc_deregister(&stm_misc);
	return 0;
}

static struct platform_driver stm_driver = {
	.probe = stm_probe,
	.remove = __devexit_p(stm_remove),
	.driver = {
		.name = "stm",
		.owner = THIS_MODULE,
	}
};

static int __init stm_init(void)
{
	return platform_driver_register(&stm_driver);
}

static void __exit stm_exit(void)
{
	platform_driver_unregister(&stm_driver);
}

module_init(stm_init);
module_exit(stm_exit);

MODULE_AUTHOR("Paul Ghaleb - ST Microelectronics");
MODULE_AUTHOR("Pierre Peiffer - ST-Ericsson");
MODULE_DESCRIPTION("Ux500 System Trace Module driver");
MODULE_ALIAS("stm");
MODULE_ALIAS("stm-trace");
MODULE_LICENSE("GPL v2");
