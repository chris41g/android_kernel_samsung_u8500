/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * License Terms: GNU General Public License v2
 * Author: Stefan Nilsson <stefan.xk.nilsson@stericsson.com> for
 * ST-Ericsson.
 * Author: Magnus Templing <magnus.templing@stericsson.com> for
 * ST-Ericsson.
 *
 * PWM (Pulse Width Modulator) driver for the DB5500 digital baseband
 * controller. Based on arch/arm/mach-pxa/pwm.c.
 */
#include <linux/err.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/slab.h>

/* Register offsets */
#define PWM_CONTROL_REG_OFFSET  0x00
#define PWM_DUTY_REG_OFFSET     0x04
#define PWM_PERIOD_REG_OFFSET   0x08
#define PWM_BURST_REG_OFFSET    0x10
#define PWM_SEQUENCE_REG_OFFSET 0x14
#define PWM_DELAY_REG_OFFSET    0x18

/* CONTROL_REG */
#define PWM_CONTROL_REG_ENABLE_POS              0
#define PWM_CONTROL_REG_CBM_POS                 1
#define PWM_CONTROL_REG_PRESCALER_POS           2
#define PWM_CONTROL_REG_DISABLE                 0
#define PWM_CONTROL_REG_ENABLE                  1
#define PWM_CONTROL_REG_CBM_ENABLE              1

#define PWM_BURST_REG_ONE_PULSE_PER_BURST       0
#define PWM_SEQUENCE_REG_ONE_BURST_PER_SEQUENCE 0
#define PWM_DELAY_REG_NO_DELAY                  0
#define PWM_PRESCALE                           25

struct pwm_device {
	struct list_head	node;
	struct platform_device *pdev;

	void __iomem	*mmio_base;

	unsigned int	pwm_id;
	unsigned int	use_count;
};

static DEFINE_MUTEX(pwm_lock);
static LIST_HEAD(pwm_list);
#if defined(CONFIG_DEBUG_FS)
static void init_debugfs(void);
#else
#define init_debugfs(x)
#endif

/*
 * PWM_CLK_RATE = 26000000 Hz
 *
 * period_ns = 10^9 * (PWM_PRESCALE + 1) * period / PWM_CLK_RATE
 * duty_ns   = 10^9 * (PWM_PRESCALE + 1) * dc / PWM_CLK_RATE
 *
 * period = period_ns * 26000000 / 10^9 / 26 => period = period_ns / 1000
 * dc = duty_ns * 26000000 / 10^9 / 26 => dc = duty_ns / 1000
 */
#define MAGIC_DIVISOR (1000)
int pwm_config(struct pwm_device *pwm, int duty_ns, int period_ns)
{
	u32 period;
	u32 dc;

	dev_dbg(&pwm->pdev->dev, "%s duty_ns: %d period_ns: %d\n",
		__func__, duty_ns, period_ns);

	if (pwm == NULL || period_ns == 0 || duty_ns >= period_ns) {
		dev_err(&pwm->pdev->dev, "%s INVALID ARGUMENTS!\n", __func__);
		return -EINVAL;
	}

	period = period_ns / MAGIC_DIVISOR;
	dc = duty_ns / MAGIC_DIVISOR;

	writel(period, pwm->mmio_base + PWM_PERIOD_REG_OFFSET);
	writel(dc, pwm->mmio_base + PWM_DUTY_REG_OFFSET);
	writel(PWM_BURST_REG_ONE_PULSE_PER_BURST,
		pwm->mmio_base + PWM_BURST_REG_OFFSET);
	writel(PWM_SEQUENCE_REG_ONE_BURST_PER_SEQUENCE,
		pwm->mmio_base + PWM_SEQUENCE_REG_OFFSET);
	writel(PWM_DELAY_REG_NO_DELAY, pwm->mmio_base + PWM_DELAY_REG_OFFSET);

	return 0;
}
EXPORT_SYMBOL(pwm_config);

int pwm_enable(struct pwm_device *pwm)
{
	u32 val;

	val = (PWM_CONTROL_REG_ENABLE << PWM_CONTROL_REG_ENABLE_POS);
	val |= (PWM_CONTROL_REG_CBM_ENABLE << PWM_CONTROL_REG_CBM_POS);
	val |= (PWM_PRESCALE << PWM_CONTROL_REG_PRESCALER_POS);

	writel(val, pwm->mmio_base + PWM_CONTROL_REG_OFFSET);

	return 0;
}
EXPORT_SYMBOL(pwm_enable);

void pwm_disable(struct pwm_device *pwm)
{
	writel(PWM_CONTROL_REG_DISABLE,
		pwm->mmio_base + PWM_CONTROL_REG_OFFSET);
}
EXPORT_SYMBOL(pwm_disable);

struct pwm_device *pwm_request(int pwm_id, const char *label)
{
	struct pwm_device *pwm;
	bool found = false;

	mutex_lock(&pwm_lock);

	list_for_each_entry(pwm, &pwm_list, node) {
		if (pwm->pwm_id == pwm_id) {
			found = true;
			break;
		}
	}

	if (found) {
		if (pwm->use_count == 0)
			pwm->use_count++;
		else
			pwm = ERR_PTR(-EBUSY);
	} else {
		pwm = ERR_PTR(-ENOENT);
	}

	mutex_unlock(&pwm_lock);
	return pwm;
}
EXPORT_SYMBOL(pwm_request);

void pwm_free(struct pwm_device *pwm)
{
	mutex_lock(&pwm_lock);

	if (pwm->use_count)
		pwm->use_count--;
	else
		dev_warn(&pwm->pdev->dev, "PWM device already freed\n");

	mutex_unlock(&pwm_lock);
}
EXPORT_SYMBOL(pwm_free);

int __init pwm_probe(struct platform_device *pdev)
{
	struct pwm_device *pwm;
	struct resource *r;
	int ret = 0;

	dev_dbg(&pdev->dev, "%s\n", __func__);

	pwm = kzalloc(sizeof(struct pwm_device), GFP_KERNEL);
	if (pwm == NULL) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	pwm->pwm_id = pdev->id;
	pwm->pdev = pdev;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		dev_err(&pdev->dev, "no memory resource defined\n");
		ret = -ENODEV;
		goto err_free;
	}

	pwm->mmio_base = ioremap(r->start, resource_size(r));
	if (pwm->mmio_base == NULL) {
		dev_err(&pdev->dev, "failed to ioremap() registers\n");
		ret = -ENODEV;
		goto err_free;
	}

	mutex_lock(&pwm_lock);
	list_add_tail(&pwm->node, &pwm_list);
	mutex_unlock(&pwm_lock);

	platform_set_drvdata(pdev, pwm);

	init_debugfs();

	return 0;

err_free:
	kfree(pwm);
	return ret;
}

static int __devexit pwm_remove(struct platform_device *pdev)
{
	struct pwm_device *pwm;

	pwm = platform_get_drvdata(pdev);
	if (pwm == NULL)
		return -ENODEV;

	mutex_lock(&pwm_lock);
	list_del(&pwm->node);
	mutex_unlock(&pwm_lock);

	iounmap(pwm->mmio_base);

	kfree(pwm);
	return 0;
}

static struct platform_driver pwm_driver = {
	.driver		= {
		.name	= "pwm",
		.owner = THIS_MODULE,
	},
	.remove         = __devexit_p(pwm_remove),
};

static int __init pwm_init(void)
{
	int ret = 0;

	ret = platform_driver_probe(&pwm_driver, pwm_probe);
	if (ret) {
		pr_err("PWM: probe failed ret=%d\n", ret);
		return ret;
	}

	return ret;
}

#if defined(CONFIG_DEBUG_FS)
#include <linux/debugfs.h>
#include <linux/seq_file.h>

static int pwm_dump(struct seq_file *s, void *data)
{
	struct list_head *pos;
	struct pwm_device *pwm;
	int pwm_index = 0;

	mutex_lock(&pwm_lock);

	list_for_each(pos, &pwm_list) {
		pwm = (struct pwm_device *)
			list_entry(pos, struct pwm_device, node);

		seq_printf(s, "===========================\n"
			" PWM DUMP %d\n"
			"   mmio_base: 0x%p\n"
			"   CONTROL  : 0x%X\n"
			"   DUTY     : 0x%X\n"
			"   PERIOD   : 0x%X\n"
			"   BURST    : 0x%X\n"
			"   SEQUENCE : 0x%X\n"
			"   DELAY    : 0x%X\n",
			pwm_index,
			pwm->mmio_base,
			readl(pwm->mmio_base + PWM_CONTROL_REG_OFFSET),
			readl(pwm->mmio_base + PWM_DUTY_REG_OFFSET),
			readl(pwm->mmio_base + PWM_PERIOD_REG_OFFSET),
			readl(pwm->mmio_base + PWM_BURST_REG_OFFSET),
			readl(pwm->mmio_base + PWM_SEQUENCE_REG_OFFSET),
			readl(pwm->mmio_base + PWM_DELAY_REG_OFFSET));
		pwm_index++;
	}

	mutex_unlock(&pwm_lock);

	return 0;
}

static int pwm_open(struct inode *inode, struct file *file)
{
	return single_open(file, pwm_dump, NULL);
}

static int get_value(char **c, char *s, int *v)
{
	char *val;
	long value;
	int ret = -1;

	if (c != NULL) {
		val = strsep(c, s);

		if (val) {
			ret = strict_strtol(val, 10, &value);

			if (!ret)
				*v = value;
		}
	}

	return ret;
}

#define MAX_BUFFER_SIZE 64

static int pwm_write_raw(struct file *file, const char __user *buffer,
	size_t size, loff_t *offset)
{
	char int_buf[MAX_BUFFER_SIZE];
	char *token;
	u32 period;
	u32 dc;
	u32 burst;
	u32 seq;
	u32 delay;
	u32 ctrl;
	int max_size = max(size, strlen(buffer));

	struct pwm_device *pwm = (struct pwm_device *)
		list_first_entry(&pwm_list, struct pwm_device, node);

	/* Copy the buffer since strsep alters it */
	strncpy((char *) &int_buf, (char*)buffer, max_size);

	token = (char *) &int_buf;

	if (get_value(&token, " ", &period)) {
		dev_err(&pwm->pdev->dev, "%s 1\n", __func__);
		return size;
	}

	if (get_value(&token, " ", &dc)) {
		dev_err(&pwm->pdev->dev, "%s 2\n", __func__);
		return size;
	}

	if (get_value(&token, " ", &burst)) {
		dev_err(&pwm->pdev->dev, "%s 3\n", __func__);
		return size;
	}

	if (get_value(&token, " ", &seq)) {
		dev_err(&pwm->pdev->dev, "%s 4\n", __func__);
		return size;
	}

	if (get_value(&token, " ", &delay)) {
		dev_err(&pwm->pdev->dev, "%s 5\n", __func__);
		return size;
	}

	if (get_value(&token, "\n", &ctrl)) {
		dev_err(&pwm->pdev->dev, "%s 6\n", __func__);
		return size;
	}

	dev_dbg(&pwm->pdev->dev,
		"%s p: 0x%X d: 0x%X b: 0x%X s: 0x%X d: 0x%X c: 0x%X\n"
		, __func__, period, dc, burst, seq, delay, ctrl);

	writel(period, pwm->mmio_base + PWM_PERIOD_REG_OFFSET);
	writel(dc    , pwm->mmio_base + PWM_DUTY_REG_OFFSET);
	writel(burst , pwm->mmio_base + PWM_BURST_REG_OFFSET);
	writel(seq   , pwm->mmio_base + PWM_SEQUENCE_REG_OFFSET);
	writel(delay , pwm->mmio_base + PWM_DELAY_REG_OFFSET);
	writel(ctrl  , pwm->mmio_base + PWM_CONTROL_REG_OFFSET);

	return size;
}

static int pwm_write(struct file *file,
			const char __user *buffer,
			size_t size, loff_t *offset)
{
	char int_buf[MAX_BUFFER_SIZE];
	char *token;
	u32 period = 0;
	u32 dc = 0;
	u32 ctrl = 0;
	int max_size = max(size, strlen(buffer));

	struct pwm_device *pwm = (struct pwm_device *)
		list_first_entry(&pwm_list, struct pwm_device, node);

	/* Copy the buffer since strsep alters it */
	strncpy((char *) &int_buf, (char*)buffer, max((int) size, max_size));

	token = (char *) &int_buf;

	if (get_value(&token, " ", &period)) {
		dev_err(&pwm->pdev->dev, "%s 1\n", __func__);
		return size;
	}

	if (get_value(&token, " ", &dc)) {
		dev_err(&pwm->pdev->dev, "%s 2\n", __func__);
		return size;
	}

	if (get_value(&token, "\n", &ctrl)) {
		dev_err(&pwm->pdev->dev, "%s 3\n", __func__);
		return size;
	}

	dev_dbg(&pwm->pdev->dev, "%s p: 0x%X d: 0x%X c: 0x%X\n",
		__func__, period, dc, ctrl);

	if (ctrl == 0)
		pwm_disable(pwm);
	else {
		pwm_config(pwm, dc, period);
		pwm_enable(pwm);
	}

	return size;
}

static const struct file_operations pwm_operations = {
	.owner = THIS_MODULE,
	.open = pwm_open,
	.read = seq_read,
	.write = pwm_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations pwm_operations_raw = {
	.owner = THIS_MODULE,
	.open = pwm_open,
	.read = seq_read,
	.write = pwm_write_raw,
	.llseek = seq_lseek,
	.release = single_release,
};

static void init_debugfs(void)
{
	(void) debugfs_create_file("pwm_if", S_IFREG | S_IRUGO | S_IWUGO,
		NULL, NULL, &pwm_operations);
	(void) debugfs_create_file("pwm_raw", S_IFREG | S_IRUGO | S_IWUGO,
		NULL, NULL, &pwm_operations_raw);
}
#endif

module_init(pwm_init);

static void __exit pwm_exit(void)
{
	platform_driver_unregister(&pwm_driver);
}
module_exit(pwm_exit);

MODULE_LICENSE("GPL v2");
