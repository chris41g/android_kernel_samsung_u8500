/*
 *  linux/drivers/mmc/host/mmci.c - ARM PrimeCell MMCI PL180/1 driver
 *
 *  Copyright (C) 2003 Deep Blue Solutions, Ltd, All Rights Reserved.
 *  Copyright (C) 2010 ST-Ericsson SA
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/highmem.h>
#include <linux/log2.h>
#include <linux/pm_runtime.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/pm.h>
#include <linux/amba/bus.h>
#include <linux/clk.h>
#include <linux/scatterlist.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/amba/mmci.h>

#include <mach/board-sec-u8500.h>
#include <asm/gpio.h>

#include <linux/io.h>
#include <asm/sizes.h>

#ifdef CONFIG_ARCH_U8500
/* Temporary solution to find out if HW is db8500 v1 or v2. */
#include <mach/hardware.h>
#endif

#include "mmci.h"

#define DRIVER_NAME "mmci-pl18x"
#define DATAEND_TIMEOUT_MS 1000

#define _MMC_SAFE_ACCESS_
#ifdef _MMC_SAFE_ACCESS_
mmc_is_available = 0;
EXPORT_SYMBOL(mmc_is_available);
#endif


static unsigned int fmax = 515633;
static unsigned int dataread_delay_clks = 7500000;

/**
 * struct variant_data - MMCI variant-specific quirks
 * @clkreg: default value for MCICLOCK register
 * @clkreg_enable: enable value for MMCICLOCK register
 * @dmareg_enable: enable value for MMCIDATACTRL register
 * @datalength_bits: number of bits in the MMCIDATALENGTH register
 * @fifosize: number of bytes that can be written when MMCI_TXFIFOEMPTY
 *	      is asserted (likewise for RX)
 * @fifohalfsize: number of bytes that can be written when MCI_TXFIFOHALFEMPTY
 *		  is asserted (likewise for RX)
 * @broken_blockend: the MCI_DATABLOCKEND is broken on the hardware
 *		and will not work at all.
 * @sdio: variant supports SDIO
 * @st_clkdiv: true if using a ST-specific clock divider algorithm
 * @pwrreg_powerup: power up value for MMCIPOWER register
 * @signal_direction: input/out direction of bus signals can be indicated,
 *		this is usually used by e.g. voltage level translators.
 * @non_power_of_2_blksize: variant supports block sizes that are not
 *		a power of two.
 */
struct variant_data {
	unsigned int		clkreg;
	unsigned int		clkreg_enable;
	unsigned int		dmareg_enable;
	unsigned int		datalength_bits;
	unsigned int		fifosize;
	unsigned int		fifohalfsize;
	bool			broken_blockend;
	bool			sdio;
	bool			st_clkdiv;
	unsigned int		pwrreg_powerup;
	bool			signal_direction;
	bool			non_power_of_2_blksize;
};

static struct variant_data variant_arm = {
	.fifosize		= 16 * 4,
	.fifohalfsize		= 8 * 4,
	.datalength_bits	= 16,
	.pwrreg_powerup		= MCI_PWR_UP,
};

static struct variant_data variant_u300 = {
	.fifosize		= 16 * 4,
	.fifohalfsize		= 8 * 4,
	.clkreg_enable		= 1 << 13, /* HWFCEN */
	.datalength_bits	= 16,
	.sdio			= true,
	.pwrreg_powerup		= MCI_PWR_ON,
	.signal_direction	= true,
};

static struct variant_data variant_ux500 = {
	.fifosize		= 30 * 4,
	.fifohalfsize		= 8 * 4,
	.clkreg			= MCI_CLK_ENABLE,
	.clkreg_enable		= 1 << 14, /* HWFCEN */
	.dmareg_enable		= 1 << 12, /* DMAREQCTRL */
	.datalength_bits	= 24,
	.broken_blockend	= true,
	.sdio			= true,
	.st_clkdiv		= true,
	.pwrreg_powerup		= MCI_PWR_ON,
	.signal_direction	= true,
	.non_power_of_2_blksize	= true,
};
/*
 * Debugfs
 */
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/seq_file.h>

static int mmci_regs_show(struct seq_file *seq, void *v)
{
	struct mmci_host *host = seq->private;
	unsigned long iflags;
	u32 pwr, clk, arg, cmd, rspcmd, r0, r1, r2, r3;
	u32 dtimer, dlength, dctrl, dcnt;
	u32 sta, clear, mask0, mask1, fifocnt;

	mmc_host_enable(host->mmc);
	spin_lock_irqsave(&host->lock, iflags);

	pwr = readl(host->base + MMCIPOWER);
	clk = readl(host->base + MMCICLOCK);
	arg = readl(host->base + MMCIARGUMENT);
	cmd = readl(host->base + MMCICOMMAND);
	rspcmd = readl(host->base + MMCIRESPCMD);
	r0 = readl(host->base + MMCIRESPONSE0);
	r1 = readl(host->base + MMCIRESPONSE1);
	r2 = readl(host->base + MMCIRESPONSE2);
	r3 = readl(host->base + MMCIRESPONSE3);
	dtimer = readl(host->base + MMCIDATATIMER);
	dlength = readl(host->base + MMCIDATALENGTH);
	dctrl = readl(host->base + MMCIDATACTRL);
	dcnt = readl(host->base + MMCIDATACNT);
	sta = readl(host->base + MMCISTATUS);
	clear = readl(host->base + MMCICLEAR);
	mask0 = readl(host->base + MMCIMASK0);
	mask1 = readl(host->base + MMCIMASK1);
	fifocnt = readl(host->base + MMCIFIFOCNT);

	spin_unlock_irqrestore(&host->lock, iflags);
	mmc_host_disable(host->mmc);

	seq_printf(seq, "\033[1;34mMMCI registers\033[0m\n");
	seq_printf(seq, "%-20s:0x%x\n", "mmci_power", pwr);
	seq_printf(seq, "%-20s:0x%x\n", "mmci_clock", clk);
	seq_printf(seq, "%-20s:0x%x\n", "mmci_arg", arg);
	seq_printf(seq, "%-20s:0x%x\n", "mmci_cmd", cmd);
	seq_printf(seq, "%-20s:0x%x\n", "mmci_respcmd", rspcmd);
	seq_printf(seq, "%-20s:0x%x\n", "mmci_resp0", r0);
	seq_printf(seq, "%-20s:0x%x\n", "mmci_resp1", r1);
	seq_printf(seq, "%-20s:0x%x\n", "mmci_resp2", r2);
	seq_printf(seq, "%-20s:0x%x\n", "mmci_resp3", r3);
	seq_printf(seq, "%-20s:0x%x\n", "mmci_datatimer", dtimer);
	seq_printf(seq, "%-20s:0x%x\n", "mmci_datalen", dlength);
	seq_printf(seq, "%-20s:0x%x\n", "mmci_datactrl", dctrl);
	seq_printf(seq, "%-20s:0x%x\n", "mmci_datacnt", dcnt);
	seq_printf(seq, "%-20s:0x%x\n", "mmci_status", sta);
	seq_printf(seq, "%-20s:0x%x\n", "mmci_iclear", clear);
	seq_printf(seq, "%-20s:0x%x\n", "mmci_imask0", mask0);
	seq_printf(seq, "%-20s:0x%x\n", "mmci_imask1", mask1);
	seq_printf(seq, "%-20s:0x%x\n", "mmci_fifocnt", fifocnt);

	return 0;
}

static void printk_mmci_regs(struct mmci_host *host)
{
	u32 pwr, clk, arg, cmd, rspcmd, r0, r1, r2, r3;
	u32 dtimer, dlength, dctrl, dcnt;
	u32 sta, clear, mask0, mask1, fifocnt;

	printk(KERN_INFO "[MMC]base value = %0x08u\n", host->base);
	if (host->base == NULL || host->phybase == NULL)
		return ;

	pwr = readl(host->base + MMCIPOWER);
	clk = readl(host->base + MMCICLOCK);
	arg = readl(host->base + MMCIARGUMENT);
	cmd = readl(host->base + MMCICOMMAND);
	rspcmd = readl(host->base + MMCIRESPCMD);
	r0 = readl(host->base + MMCIRESPONSE0);
	r1 = readl(host->base + MMCIRESPONSE1);
	r2 = readl(host->base + MMCIRESPONSE2);
	r3 = readl(host->base + MMCIRESPONSE3);
	dtimer = readl(host->base + MMCIDATATIMER);
	dlength = readl(host->base + MMCIDATALENGTH);
	dctrl = readl(host->base + MMCIDATACTRL);
	dcnt = readl(host->base + MMCIDATACNT);
	sta = readl(host->base + MMCISTATUS);
	clear = readl(host->base + MMCICLEAR);
	mask0 = readl(host->base + MMCIMASK0);
	mask1 = readl(host->base + MMCIMASK1);
	fifocnt = readl(host->base + MMCIFIFOCNT);

	printk(KERN_INFO "Registers of MMCI at 0x%08x\n", host->phybase);
	printk(KERN_INFO "%-20s:0x%08x\n", "mmci_power", pwr);
	printk(KERN_INFO " %-20s:0x%08x\n", "mmci_clock", clk);
	printk(KERN_INFO "%-20s:0x%08x\n", "mmci_arg", arg);
	printk(KERN_INFO "%-20s:0x%08x\n", "mmci_cmd", cmd);
	printk(KERN_INFO "%-20s:0x%08x\n", "mmci_respcmd", rspcmd);
	printk(KERN_INFO "%-20s:0x%08x\n", "mmci_resp0", r0);
	printk(KERN_INFO "%-20s:0x%08x\n", "mmci_resp1", r1);
	printk(KERN_INFO "%-20s:0x%08x\n", "mmci_resp2", r2);
	printk(KERN_INFO "%-20s:0x%08x\n", "mmci_resp3", r3);
	printk(KERN_INFO "%-20s:0x%08x\n", "mmci_datatimer", dtimer);
	printk(KERN_INFO "%-20s:0x%08x\n", "mmci_datalen", dlength);
	printk(KERN_INFO "%-20s:0x%08x\n", "mmci_datactrl", dctrl);
	printk(KERN_INFO "%-20s:0x%08x\n", "mmci_datacnt", dcnt);
	printk(KERN_INFO "%-20s:0x%08x\n", "mmci_status", sta);
	printk(KERN_INFO "%-20s:0x%08x\n", "mmci_iclear", clear);
	printk(KERN_INFO "%-20s:0x%08x\n", "mmci_imask0", mask0);
	printk(KERN_INFO "%-20s:0x%08x\n", "mmci_imask1", mask1);
	printk(KERN_INFO "%-20s:0x%08x\n", "mmci_fifocnt", fifocnt);
}

static int mmci_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, mmci_regs_show, inode->i_private);
}

static const struct file_operations mmci_fops_regs = {
	.owner		= THIS_MODULE,
	.open		= mmci_regs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void mmci_debugfs_create(struct mmci_host *host)
{
	host->debug_regs = debugfs_create_file("regs", S_IRUGO,
					       host->mmc->debugfs_root, host,
					       &mmci_fops_regs);

	if (IS_ERR(host->debug_regs))
		dev_err(mmc_dev(host->mmc),
				"failed to create debug regs file\n");
}

static void mmci_debugfs_remove(struct mmci_host *host)
{
	debugfs_remove(host->debug_regs);
}

#else
static inline void mmci_debugfs_create(struct mmci_host *host) { }
static inline void mmci_debugfs_remove(struct mmci_host *host) { }
#endif

static void dump_mmci_regs(struct mmc_host *mmc)
{
	u32 pwr, clk, arg, cmd, rspcmd, r0, r1, r2, r3;
	u32 dtimer, dlength, dctrl, dcnt;
	u32 sta, clear, mask0, mask1, fifocnt;

	struct mmci_host *host = mmc_priv(mmc);

	dev_info(mmc_dev(mmc), "[MMC]base value = %0x08u\n", host->base);
	if (host->base == NULL || host->phybase == NULL)
		return ;

	pwr = readl(host->base + MMCIPOWER);
	clk = readl(host->base + MMCICLOCK);
	arg = readl(host->base + MMCIARGUMENT);
	cmd = readl(host->base + MMCICOMMAND);
	rspcmd = readl(host->base + MMCIRESPCMD);
	r0 = readl(host->base + MMCIRESPONSE0);
	r1 = readl(host->base + MMCIRESPONSE1);
	r2 = readl(host->base + MMCIRESPONSE2);
	r3 = readl(host->base + MMCIRESPONSE3);
	dtimer = readl(host->base + MMCIDATATIMER);
	dlength = readl(host->base + MMCIDATALENGTH);
	dctrl = readl(host->base + MMCIDATACTRL);
	dcnt = readl(host->base + MMCIDATACNT);
	sta = readl(host->base + MMCISTATUS);
	clear = readl(host->base + MMCICLEAR);
	mask0 = readl(host->base + MMCIMASK0);
	mask1 = readl(host->base + MMCIMASK1);
	fifocnt = readl(host->base + MMCIFIFOCNT);

	dev_info(mmc_dev(mmc), "Registers of MMCI at 0x%08x\n", host->phybase);
	dev_info(mmc_dev(mmc), "%-20s:0x%08x\n", "mmci_power", pwr);
	dev_info(mmc_dev(mmc), "%-20s:0x%08x\n", "mmci_clock", clk);
	dev_info(mmc_dev(mmc), "%-20s:0x%08x\n", "mmci_arg", arg);
	dev_info(mmc_dev(mmc), "%-20s:0x%08x\n", "mmci_cmd", cmd);
	dev_info(mmc_dev(mmc), "%-20s:0x%08x\n", "mmci_respcmd", rspcmd);
	dev_info(mmc_dev(mmc), "%-20s:0x%08x\n", "mmci_resp0", r0);
	dev_info(mmc_dev(mmc), "%-20s:0x%08x\n", "mmci_resp1", r1);
	dev_info(mmc_dev(mmc), "%-20s:0x%08x\n", "mmci_resp2", r2);
	dev_info(mmc_dev(mmc), "%-20s:0x%08x\n", "mmci_resp3", r3);
	dev_info(mmc_dev(mmc), "%-20s:0x%08x\n", "mmci_datatimer", dtimer);
	dev_info(mmc_dev(mmc), "%-20s:0x%08x\n", "mmci_datalen", dlength);
	dev_info(mmc_dev(mmc), "%-20s:0x%08x\n", "mmci_datactrl", dctrl);
	dev_info(mmc_dev(mmc), "%-20s:0x%08x\n", "mmci_datacnt", dcnt);
	dev_info(mmc_dev(mmc), "%-20s:0x%08x\n", "mmci_status", sta);
	dev_info(mmc_dev(mmc), "%-20s:0x%08x\n", "mmci_iclear", clear);
	dev_info(mmc_dev(mmc), "%-20s:0x%08x\n", "mmci_imask0", mask0);
	dev_info(mmc_dev(mmc), "%-20s:0x%08x\n", "mmci_imask1", mask1);
	dev_info(mmc_dev(mmc), "%-20s:0x%08x\n", "mmci_fifocnt", fifocnt);
}

/*
 * Uggly hack! This must be removed soon!!
 *
 * u8500_sdio_detect_card() - Initiates card scan for sdio host.
 * This is required to initiate card rescan from sdio client device driver.
 *
 * sdio_host_ptr - Host pointer to save SDIO host data structure
 * (will only work when the SDIO device is probed as the last MMCI device).
 */
static struct mmci_host *sdio_host_ptr;
void u8500_sdio_detect_card(void)
{
	struct mmci_host *host = sdio_host_ptr;
	if (sdio_host_ptr && host->mmc)
		mmc_detect_change(host->mmc, msecs_to_jiffies(10));

	return;
}
EXPORT_SYMBOL(u8500_sdio_detect_card);

/*
 * This must be called with host->lock held
 */
static void mmci_set_clkreg(struct mmci_host *host, unsigned int desired)
{
	struct variant_data *variant = host->variant;
	u32 clk = variant->clkreg;

	if (desired) {
		if (desired >= host->mclk) {
			clk = MCI_CLK_BYPASS | MCI_NEG_EDGE;
			host->cclk = host->mclk;
		} else if (variant->st_clkdiv) {
			clk = ((host->mclk + desired - 1) / desired) - 2;
			if (clk >= 256)
				clk = 255;
			host->cclk = host->mclk / (clk + 2);
		} else {
			clk = host->mclk / (2 * desired) - 1;
			if (clk >= 256)
				clk = 255;
			host->cclk = host->mclk / (2 * (clk + 1));
		}

		clk |= variant->clkreg_enable;
		clk |= MCI_CLK_ENABLE;
		/* This hasn't proven to be worthwhile */
		/* clk |= MCI_CLK_PWRSAVE; */
	}

	if (host->mmc->ios.bus_width == MMC_BUS_WIDTH_4)
		clk |= MCI_4BIT_BUS;
	if (host->mmc->ios.bus_width == MMC_BUS_WIDTH_8)
		clk |= MCI_ST_8BIT_BUS;

	writel(clk, host->base + MMCICLOCK);
}

static void
mmci_request_end(struct mmci_host *host, struct mmc_request *mrq)
{
	writel(0, host->base + MMCICOMMAND);

	BUG_ON(host->data);

	host->mrq = NULL;
	host->cmd = NULL;

	if (mrq->data)
		mrq->data->bytes_xfered = host->data_xfered;

	/*
	 * Need to drop the host lock here; mmc_request_done may call
	 * back into the driver...
	 */
	spin_unlock(&host->lock);
	mmc_request_done(host->mmc, mrq);
	spin_lock(&host->lock);
}

static void mmci_set_mask1(struct mmci_host *host, unsigned int mask)
{
	void __iomem *base = host->base;

	if (host->singleirq) {
		unsigned int mask0 = readl(base + MMCIMASK0);

		mask0 &= ~MCI_IRQ1MASK;
		mask0 |= mask;

		writel(mask0, base + MMCIMASK0);
	}

	writel(mask, base + MMCIMASK1);
}

static void mmci_stop_data(struct mmci_host *host)
{
	u32 clk;
	unsigned int datactrl = 0;

	/*
	 * The ST Micro variants has a special bit
	 * to enable SDIO mode. This bit must remain set even when not
	 * doing data transfers, otherwise no SDIO interrupts can be
	 * received.
	 */
	if (host->variant->sdio &&
		host->mmc->card &&
		mmc_card_sdio(host->mmc->card))
		datactrl |= MCI_ST_DPSM_SDIOEN|MCI_ST_DPSM_BUSYMODE;

	writel(datactrl, host->base + MMCIDATACTRL);
	mmci_set_mask1(host, 0);

	/* Needed for DDR */
	if (host->mmc->card && mmc_card_ddr_mode(host->mmc->card)) {
		clk = readl(host->base + MMCICLOCK);
		clk &= ~(MCI_NEG_EDGE);

		writel(clk, (host->base + MMCICLOCK));
	}

	host->data = NULL;
}

static void mmci_init_sg(struct mmci_host *host, struct mmc_data *data)
{
	unsigned int flags = SG_MITER_ATOMIC;

	if (data->flags & MMC_DATA_READ)
		flags |= SG_MITER_TO_SG;
	else
		flags |= SG_MITER_FROM_SG;

	sg_miter_start(&host->sg_miter, data->sg, data->sg_len, flags);
}

static void
mmci_start_command(struct mmci_host *host, struct mmc_command *cmd, u32 c)
{
	void __iomem *base = host->base;

	dev_dbg(mmc_dev(host->mmc), "op %02x arg %08x flags %08x\n",
	    cmd->opcode, cmd->arg, cmd->flags);

	if (readl(base + MMCICOMMAND) & MCI_CPSM_ENABLE) {
		writel(0, base + MMCICOMMAND);
		udelay(1);
	}

	c |= cmd->opcode | MCI_CPSM_ENABLE;
	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136)
			c |= MCI_CPSM_LONGRSP;
		c |= MCI_CPSM_RESPONSE;
	}
	if (/*interrupt*/0)
		c |= MCI_CPSM_INTERRUPT;

	host->cmd = cmd;

	writel(cmd->arg, base + MMCIARGUMENT);
	writel(c, base + MMCICOMMAND);
}

static void
mmci_complete_data_xfer(struct mmci_host *host)
{
	struct mmc_data *data = host->data;

	if ((host->size == 0) || data->error) {

		/*
		 * The data transfer is done and then there is no need for the
		 * a delayed work any more, thus remove it.
		 */
		host->dataend_timeout_active = false;
		__cancel_delayed_work(&host->dataend_timeout);

		/*
		 * Variants with broken blockend flags and as well dma
		 * transfers handles the end of the entire transfer here.
		 */
		if (host->last_blockend && !data->error)
			host->data_xfered = data->blksz * data->blocks;

		mmci_stop_data(host);

		if (!data->stop)
			mmci_request_end(host, data->mrq);
		else
			mmci_start_command(host, data->stop, 0);
	} else {
		/*
		 * Schedule a delayed work to make sure we do not end up
		 * forever waiting for a data transfer to be finished.
		 */
		host->dataend_timeout_active = true;
		schedule_delayed_work(&host->dataend_timeout,
				msecs_to_jiffies(DATAEND_TIMEOUT_MS));
	}
}

/*
 * All the DMA operation mode stuff goes inside this ifdef.
 * This assumes that you have a generic DMA device interface,
 * no custom DMA interfaces are supported.
 */
#ifdef CONFIG_DMA_ENGINE
static void __devinit mmci_setup_dma(struct mmci_host *host)
{
	struct mmci_platform_data *plat = host->plat;
	dma_cap_mask_t mask;

	if (!plat || !plat->dma_filter) {
		dev_err(mmc_dev(host->mmc), "no DMA platform data!\n");
		return;
	}

	/* Try to acquire a generic DMA engine slave channel */
	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);
	/*
	 * If only an RX channel is specified, the driver will
	 * attempt to use it bidirectionally, however if it is
	 * is specified but cannot be located, DMA will be disabled.
	 */
	host->dma_rx_channel = dma_request_channel(mask,
						plat->dma_filter,
						plat->dma_rx_param);
	/* E.g if no DMA hardware is present */
	if (!host->dma_rx_channel) {
		dev_err(mmc_dev(host->mmc), "no RX DMA channel!\n");
		return;
	}
	if (plat->dma_tx_param) {
		host->dma_tx_channel = dma_request_channel(mask,
							   plat->dma_filter,
							   plat->dma_tx_param);
		if (!host->dma_tx_channel) {
			dma_release_channel(host->dma_rx_channel);
			host->dma_rx_channel = NULL;
			return;
		}
	} else {
		host->dma_tx_channel = host->dma_rx_channel;
	}
	host->dma_enable = true;
	dev_info(mmc_dev(host->mmc), "use DMA channels DMA RX %s, DMA TX %s\n",
		 dma_chan_name(host->dma_rx_channel),
		 dma_chan_name(host->dma_tx_channel));
}

/*
 * This is used in __devinit or __devexit so inline it
 * so it can be discarded.
 */
static inline void mmci_disable_dma(struct mmci_host *host)
{
	if (host->dma_rx_channel)
		dma_release_channel(host->dma_rx_channel);
	if (host->dma_tx_channel)
		dma_release_channel(host->dma_tx_channel);
	host->dma_enable = false;
}

static void mmci_dma_data_end(struct mmci_host *host)
{
	struct mmc_data *data = host->data;

	dma_unmap_sg(mmc_dev(host->mmc), data->sg, data->sg_len,
		     (data->flags & MMC_DATA_WRITE)
		     ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
	host->dma_on_current_xfer = false;
}

static void mmci_dma_terminate(struct mmci_host *host)
{
	struct mmc_data *data = host->data;
	struct dma_chan *chan;

	dev_err(mmc_dev(host->mmc), "error during DMA transfer!\n");
	if (data->flags & MMC_DATA_READ)
		chan = host->dma_rx_channel;
	else
		chan = host->dma_tx_channel;
	dma_unmap_sg(mmc_dev(host->mmc), data->sg, data->sg_len,
		     (data->flags & MMC_DATA_WRITE)
		     ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
	chan->device->device_control(chan, DMA_TERMINATE_ALL, 0);
	host->dma_on_current_xfer = false;
}

static void mmci_dma_callback(void *arg)
{
	unsigned long flags;
	struct mmci_host *host = arg;

	dev_vdbg(mmc_dev(host->mmc), "DMA transfer done!\n");

	spin_lock_irqsave(&host->lock, flags);

	/* Check that the DMA transfer is not already terminated */
	if (host->dma_on_current_xfer) {
		mmci_dma_data_end(host);

		/* Mark that the entire data is transferred
		   for this dma transfer. */
		host->size = 0;

		/*
		 * Make sure MMCI has received MCI_DATAEND before
		 * completing the data transfer.
		 */
		if (host->dataend) {
			mmci_complete_data_xfer(host);
		} else {
			/*
			 * Schedule a delayed work to make sure we do not end up
			 * forever waiting for a dataend irq.
			 */
			host->dataend_timeout_active = true;
			schedule_delayed_work(&host->dataend_timeout,
					msecs_to_jiffies(DATAEND_TIMEOUT_MS));
		}
	} else {
		dev_warn(mmc_dev(host->mmc),
			"Ignore dma callback from terminated DMA transfer\n");
	}

	spin_unlock_irqrestore(&host->lock, flags);
}

static int mmci_dma_start_data(struct mmci_host *host, unsigned int datactrl)
{
	struct variant_data *variant = host->variant;
	struct dma_slave_config conf = {
		.src_addr = host->phybase + MMCIFIFO,
		.dst_addr = host->phybase + MMCIFIFO,
		.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
		.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
		.src_maxburst = variant->fifohalfsize >> 2, /* # of words */
		.dst_maxburst = variant->fifohalfsize >> 2, /* # of words */
	};
	struct mmc_data *data = host->data;
	struct dma_chan *chan;
	struct dma_device *device;
	struct dma_async_tx_descriptor *desc;
	int memory_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	int maxburst_mult = 0;
	struct scatterlist *sg;
	int nr_sg, i;
	dma_cookie_t cookie;
	unsigned int irqmask0;

	if (data->flags & MMC_DATA_READ) {
		conf.direction = DMA_FROM_DEVICE;
		chan = host->dma_rx_channel;
	} else {
		conf.direction = DMA_TO_DEVICE;
		chan = host->dma_tx_channel;
	}

	/* If there's no DMA channel, fall back to PIO */
	if (!chan)
		return -EINVAL;

	/* If less than or equal to the fifo size, don't bother with DMA */
	if (host->size <= variant->fifosize)
		return -EINVAL;

	/*
	 * Verfiy alignment of all the sg lists elements. Calculate
	 * correct value for memory bus width value and adjust
	 * the maxburst for maximum throughput.
	 */
	for_each_sg(data->sg, sg, data->sg_len, i) {
		/* PL180 cannot handle non-word aligned sizes */
		if (sg->length & 3)
			return -EINVAL;

		/*
		 * Use highest possible memory bus width and adjust
		 * maxburst multiplier depending on alignment.
		 */
		switch (sg->offset & 3) {
		case 1:
		case 3:
			memory_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
			maxburst_mult = 2;
			break;
		case 2:
			memory_addr_width = min(memory_addr_width,
						DMA_SLAVE_BUSWIDTH_2_BYTES);
			maxburst_mult = max(maxburst_mult, 1);
			break;
		default:
			break;
		}
	}
	if (data->flags & MMC_DATA_READ) {
		conf.dst_addr_width = memory_addr_width;
		conf.dst_maxburst = conf.src_maxburst << maxburst_mult;
	} else {
		conf.src_addr_width = memory_addr_width;
		conf.src_maxburst = conf.dst_maxburst << maxburst_mult;
	}

	device = chan->device;
	nr_sg = dma_map_sg(device->dev, data->sg, data->sg_len, conf.direction);
	if (nr_sg == 0)
		return -EINVAL;

	device->device_control(chan, DMA_SLAVE_CONFIG, (unsigned long) &conf);
	desc = device->device_prep_slave_sg(chan, data->sg, nr_sg,
					    conf.direction,
					    DMA_CTRL_ACK | DMA_PREP_INTERRUPT);
	if (!desc)
		goto unmap_exit;

	/* Setup dma callback function. */
	desc->callback = mmci_dma_callback;
	desc->callback_param = host;

	/* Okay, go for it. */
	dev_vdbg(mmc_dev(host->mmc),
		 "Submit MMCI DMA job, sglen %d blksz %04x blks %04x flags %08x\n",
		 data->sg_len, data->blksz, data->blocks, data->flags);
	cookie = desc->tx_submit(desc);

	/* Here overloaded DMA controllers may fail */
	if (dma_submit_error(cookie))
		goto unmap_exit;

	host->dma_on_current_xfer = true;
	device->device_issue_pending(chan);

	datactrl |= variant->dmareg_enable | MCI_DPSM_DMAENABLE;

	/*
	 * MMCI monitors both MCI_DATAEND and the DMA callback.
	 * Both events must occur before the transfer is considered
	 * to be completed. MCI_DATABLOCKEND is not used in DMA mode.
	 */
	host->last_blockend = true;
	irqmask0 = readl(host->base + MMCIMASK0);
	irqmask0 &= ~MCI_DATABLOCKENDMASK;
	writel(irqmask0, host->base + MMCIMASK0);

	/* Trigger the DMA transfer */
	writel(datactrl, host->base + MMCIDATACTRL);
	return 0;

unmap_exit:
	device->device_control(chan, DMA_TERMINATE_ALL, 0);
	dma_unmap_sg(device->dev, data->sg, data->sg_len, conf.direction);
	return -ENOMEM;
}
#else
/* Blank functions if the DMA engine is not available */
static inline void mmci_setup_dma(struct mmci_host *host)
{
}

static inline void mmci_disable_dma(struct mmci_host *host)
{
}

static inline void mmci_dma_data_end(struct mmci_host *host)
{
}

static inline void mmci_dma_terminate(struct mmci_host *host)
{
}

static inline int mmci_dma_start_data(struct mmci_host *host,
		unsigned int datactrl)
{
	return -ENOSYS;
}
#endif

static void mmci_dataend_timeout(struct work_struct *work)
{
	unsigned long flags;
	struct mmci_host *host =
		container_of(work, struct mmci_host, dataend_timeout.work);

	spin_lock_irqsave(&host->lock, flags);

	if (host->dataend_timeout_active) {
		dev_err(mmc_dev(host->mmc), "datatransfer timeout!\n");

		if (host->dma_on_current_xfer)
			mmci_dma_terminate(host);

		host->data->error = -EAGAIN;
		mmci_complete_data_xfer(host);
	}

	spin_unlock_irqrestore(&host->lock, flags);
}

static void mmci_start_data(struct mmci_host *host, struct mmc_data *data)
{
	struct variant_data *variant = host->variant;
	unsigned int datactrl, timeout, irqmask0, irqmask1;
	unsigned int clkcycle_ns;
	void __iomem *base;
	int blksz_bits;
	u32 clk;
  
	dev_dbg(mmc_dev(host->mmc), "blksz %04x blks %04x flags %08x\n",
		data->blksz, data->blocks, data->flags);

	host->data = data;
	host->size = data->blksz * data->blocks;
	host->data_xfered = 0;
	host->last_blockend = false;
	host->dataend = false;
	host->cache_len = 0;
	host->cache = 0;

	clkcycle_ns = 1000000000 / host->cclk;
	timeout = data->timeout_ns / clkcycle_ns;
	timeout += data->timeout_clks;

	if (data->flags & MMC_DATA_READ
		&& host->mmc->card
		&& mmc_card_mmc(host->mmc->card)
		&& host->mmc->card->cid.manfid == 0x13 /* CID_MANFID_MICRON */
		&& host->mmc->card->cid.oemid == 0x200) {
		/*
		 * Since the read command is sent after we have setup
		 * the data transfer we must increase the data timeout.
		 * Unfortunately this is not enough since some cards
		 * does not seem to stick to what is stated in their
		 * CSD for TAAC and NSAC.
		 */
		timeout += dataread_delay_clks;
	}

	base = host->base;
	writel(timeout, base + MMCIDATATIMER);
	writel(host->size, base + MMCIDATALENGTH);

	blksz_bits = ffs(data->blksz) - 1;

#ifdef CONFIG_ARCH_U8500
	/* Temporary solution for db8500v2. */
	if (cpu_is_u8500v20_or_later())
		datactrl = MCI_DPSM_ENABLE | (data->blksz << 16);
	else
#endif
	datactrl = MCI_DPSM_ENABLE | blksz_bits << 4;

	if (data->flags & MMC_DATA_READ)
		datactrl |= MCI_DPSM_DIRECTION;

	if (host->mmc->card && mmc_card_ddr_mode(host->mmc->card)) {
		datactrl |= MCI_ST_DPSM_DDRMODE;

		/* Needed for DDR */
		clk = readl(base + MMCICLOCK);
		clk |= MCI_NEG_EDGE;

		writel(clk, (base + MMCICLOCK));
	}

	if (variant->sdio &&
		host->mmc->card &&
		mmc_card_sdio(host->mmc->card)) {
			/*
			 * The ST Micro variants has a special bit
			 * to enable SDIO mode. This bit is set the first time
			 * a SDIO data transfer is done and must remain set
			 * after the data transfer is completed. The reason is
			 * because of otherwise no SDIO interrupts can be
			 * received.
			 */
			datactrl |= MCI_ST_DPSM_SDIOEN|MCI_ST_DPSM_BUSYMODE;

			/*
			 * The ST Micro variant for SDIO transfer sizes
			 * less than or equal to 8 bytes needs to have clock
			 * H/W flow control disabled. Since flow control is
			 * not really needed for anything that fits in the
			 * FIFO, we can disable it for any write smaller
			 * than the FIFO size.
			 */
			if ((host->size < 8) &&
			    (data->flags & MMC_DATA_WRITE))
				writel(readl(host->base + MMCICLOCK) &
					~variant->clkreg_enable,
					host->base + MMCICLOCK);
			else
				writel(readl(host->base + MMCICLOCK) |
					variant->clkreg_enable,
					host->base + MMCICLOCK);
	}

	if (host->dma_enable) {
		int ret;

		/*
		 * Attempt to use DMA operation mode, if this
		 * should fail, fall back to PIO mode
		 */
		ret = mmci_dma_start_data(host, datactrl);
		if (!ret)
			return;
	}

	/* IRQ mode, map the SG list for CPU reading/writing */
	mmci_init_sg(host, data);

	if (data->flags & MMC_DATA_READ) {
		irqmask1 = MCI_RXFIFOHALFFULLMASK;

		/*
		 * If we have less than a FIFOSIZE of bytes to
		 * transfer, trigger a PIO interrupt as soon as any
		 * data is available.
		 */
		if (host->size < variant->fifosize)
		irqmask1 |= MCI_RXDATAAVLBLMASK;
	} else {
		/*
		 * We don't actually need to include "FIFO empty" here
		 * since its implicit in "FIFO half empty".
		 */
		irqmask1 = MCI_TXFIFOHALFEMPTYMASK;
	}

	if (host->variant->sdio && host->mmc->card &&
	   mmc_card_sdio(host->mmc->card)) {
	   		int count=0;
			while (readl(host->base + MMCISTATUS)&MCI_CARDBUSY) {
				udelay(10);
				if (++count > 50000) {
					dev_err(mmc_dev(host->mmc),"Time out 500msec.\n");
					break;
				}
			}
	}
	
	/* Setup IRQ */
	irqmask0 = readl(base + MMCIMASK0);
	if (variant->broken_blockend) {
		host->last_blockend = true;
		irqmask0 &= ~MCI_DATABLOCKENDMASK;
	} else {
		irqmask0 |= MCI_DATABLOCKENDMASK;
	}
	writel(irqmask0, base + MMCIMASK0);
	mmci_set_mask1(host, irqmask1);

	/* Start the data transfer */
	writel(datactrl, base + MMCIDATACTRL);
}

static void
mmci_data_irq(struct mmci_host *host, struct mmc_data *data,
		unsigned int status)
{
	struct variant_data *variant = host->variant;
	u32 data_opcode, cmd_print;

	cmd_print = readl(host->base + MMCICOMMAND);

	if (host->cmd != NULL)
		data_opcode = host->cmd->opcode;

	/* First check for errors */
	if (status & MCI_DATA_ERR) {
		dev_dbg(mmc_dev(host->mmc), "MCI ERROR IRQ (status %08x)\n",
				status);
		if (status & MCI_DATACRCFAIL) {
			data->error = -EILSEQ;
			printk(KERN_ERR"%s: [MMC] DATA CRC STATUS: %x, CMD: %d\n",
				mmc_hostname(host->mmc), status, cmd_print & 0x3F);
		} else if (status & MCI_DATATIMEOUT) {
			data->error = -ETIMEDOUT;
			printk(KERN_ERR"%s: [MMC] DATA TIMEOUT STATUS: %x, CMD: %d\n",
				mmc_hostname(host->mmc), status, cmd_print & 0x3F);
		} else if (status & MCI_STARTBITERR) {
			data->error = -ECOMM;
			printk(KERN_ERR"%s: [MMC] DATA STARTBIT STATUS: %x, CMD: %d\n",
				mmc_hostname(host->mmc), status, cmd_print & 0x3F);
		} else if (status & (MCI_TXUNDERRUN|MCI_RXOVERRUN)) {
			data->error = -EIO;
			printk(KERN_ERR"%s: [MMC] DATA TX, RX STATUS: %x, CMD: %d\n",
				mmc_hostname(host->mmc), status, cmd_print & 0x3F);
		}
	/* Terminate the DMA transfer */
		if (host->dma_on_current_xfer) {
			if (host->phybase == 0x80005000) {
				printk(KERN_ERR "%s remaining host->size %d,status 0x%08x\n",
					__func__, host->size, status);
				printk_mmci_regs(host);
			}
			mmci_dma_terminate(host);
		}
	}

	/*
	 * On ARM variants in PIO mode, MCI_DATABLOCKEND
	 * is always sent first, and we increase the
	 * transfered number of bytes for that IRQ. Then
	 * MCI_DATAEND follows and we conclude the transaction.
	 *
	 * On the Ux500 single-IRQ variant MCI_DATABLOCKEND
	 * doesn't seem to immediately clear from the status,
	 * so we can't use it keep count when only one irq is
	 * used because the irq will hit for other reasons, and
	 * then the flag is still up. So we use the MCI_DATAEND
	 * IRQ at the end of the entire transfer because
	 * MCI_DATABLOCKEND is broken.
	 *
	 * In the U300, the IRQs can arrive out-of-order,
	 * e.g. MCI_DATABLOCKEND sometimes arrives after MCI_DATAEND,
	 * so for this case we use the flags "last_blockend" and
	 * "dataend" to make sure both IRQs have arrived before
	 * concluding the transaction. (This does not apply
	 * to the Ux500 which doesn't fire MCI_DATABLOCKEND
	 * at all.) In DMA mode it suffers from the same problem
	 * as the Ux500.
	 */
	if (status & MCI_DATABLOCKEND) {
		/*
		 * Just being a little over-cautious, we do not
		 * use this progressive update if the hardware blockend
		 * flag is unreliable: since it can stay high between
		 * IRQs it will corrupt the transfer counter.
		 */
		if (!variant->broken_blockend && !host->dma_on_current_xfer) {
			host->data_xfered += data->blksz;

			if (host->data_xfered == data->blksz * data->blocks)
				host->last_blockend = true;
		}
	}

	if (status & MCI_DATAEND)
		host->dataend = true;

	/*
	 * On variants with broken blockend we shall only wait for dataend,
	 * on others we must sync with the blockend signal since they can
	 * appear out-of-order.
	 */
	if ((host->dataend && host->last_blockend) || data->error)
		mmci_complete_data_xfer(host);
}

	static void
mmci_cmd_irq(struct mmci_host *host, struct mmc_command *cmd,
		unsigned int status)
{
	void __iomem *base = host->base;
	u32 cmd_opcode, cmd_print;

	cmd_opcode = host->cmd->opcode;
	host->cmd = NULL;
	cmd_print = readl(host->base + MMCICOMMAND);

	cmd->resp[0] = readl(base + MMCIRESPONSE0);
	cmd->resp[1] = readl(base + MMCIRESPONSE1);
	cmd->resp[2] = readl(base + MMCIRESPONSE2);
	cmd->resp[3] = readl(base + MMCIRESPONSE3);

	if (status & MCI_CMDTIMEOUT) {
		cmd->error = -ETIMEDOUT;
		printk(KERN_ERR"%s: [MMC] CMD TIMEOUT STATUS: %x, CMD: %d\n",
			mmc_hostname(host->mmc), status, cmd_print & 0x3F);
	} else if (status & MCI_CMDCRCFAIL && cmd->flags & MMC_RSP_CRC) {
		cmd->error = -EILSEQ;
		printk(KERN_ERR"%s: [MMC] CMD Command CRC STATUS: %x, CMD: %d\n",
			mmc_hostname(host->mmc), status, cmd_print & 0x3F);
	}
	if (!cmd->data || cmd->error) {
		if (host->data) {
			if (host->dma_on_current_xfer)
				mmci_dma_terminate(host);
			mmci_stop_data(host);
		}
		mmci_request_end(host, cmd->mrq);
	} else if (!(cmd->data->flags & MMC_DATA_READ))
		mmci_start_data(host, cmd->data);
}

static int mmci_pio_read(struct mmci_host *host, char *buffer,
		unsigned int remain)
{
	void __iomem *base = host->base;
	char *ptr = buffer;
	u32 status;
	int host_remain = host->size;

	do {
		int count = host_remain - (readl(base + MMCIFIFOCNT) << 2);

		if (count > remain)
			count = remain;

		if (count <= 0)
			break;

		/*
		 * SDIO especially may want to receive something that is
		 * not divisible by 4 (as opposed to card sectors
		 * etc). Therefore make sure we always read the last bytes
		 * out of the FIFO.
		 */
		switch (count) {
		case 1:
		case 3:
			readsb(base + MMCIFIFO, ptr, count);
			break;
		case 2:
			readsw(base + MMCIFIFO, ptr, 1);
			break;
		default:
			readsl(base + MMCIFIFO, ptr, count >> 2);
			break;
		}

		ptr += count;
		remain -= count;
		host_remain -= count;

		if (remain == 0)
			break;

		status = readl(base + MMCISTATUS);
	} while (status & MCI_RXDATAAVLBL);

	return ptr - buffer;
}

static int mmci_pio_write(struct mmci_host *host, char *buffer,
		unsigned int remain, u32 status)
{
	struct variant_data *variant = host->variant;
	void __iomem *base = host->base;
	char *ptr = buffer;

	unsigned int data_left = host->size;
	unsigned int count, maxcnt;
	char *cache_ptr;
	int i;

	do {
		maxcnt = status & MCI_TXFIFOEMPTY ?
			 variant->fifosize : variant->fifohalfsize;

		/*
		 * A write to the FIFO must always be done of 4 bytes aligned
		 * data. If the buffer is not 4 bytes aligned we must pad the
		 * data, but this must only be done for the final write for the
		 * entire data transfer, otherwise we will corrupt the data.
		 * Thus a buffer cache of four bytes is needed to temporary
		 * store data.
		 */

		if (host->cache_len) {
			cache_ptr = (char *)&host->cache;
			cache_ptr = cache_ptr + host->cache_len;
			data_left += host->cache_len;

			while ((host->cache_len < 4) && (remain > 0)) {
				*cache_ptr = *ptr;
				cache_ptr++;
				ptr++;
				host->cache_len++;
				remain--;
			}

			if ((host->cache_len == 4) ||
				(data_left == host->cache_len)) {

				writesl(base + MMCIFIFO, &host->cache, 1);
				if (data_left == host->cache_len)
					break;

				host->cache = 0;
				host->cache_len = 0;
				maxcnt -= 4;
				data_left -= 4;
			}

			if (remain == 0)
				break;
		}

		count = min(remain, maxcnt);

		if (!(count % 4) || (data_left == count)) {
			/*
			 * The data is either 4-bytes aligned or it is the
			 * last data to write. It is thus fine to potentially
			 * pad the data if needed.
			 */
			writesl(base + MMCIFIFO, ptr, (count + 3) >> 2);
			ptr += count;
			remain -= count;
			data_left -= count;

		} else {

			host->cache_len = count % 4;
			count = (count >> 2) << 2;

			if (count)
				writesl(base + MMCIFIFO, ptr, count >> 2);

			ptr += count;
			remain -= count;
			data_left -= count;

			i = 0;
			cache_ptr = (char *)&host->cache;
			while (i < host->cache_len) {
				*cache_ptr = *ptr;
				cache_ptr++;
				ptr++;
				remain--;
				i++;
			}
		}

		if (remain == 0)
			break;

		status = readl(base + MMCISTATUS);
	} while (status & MCI_TXFIFOHALFEMPTY);

	return ptr - buffer;
}

/*
 * PIO data transfer IRQ handler.
 */
static irqreturn_t mmci_pio_irq(int irq, void *dev_id)
{
	struct mmci_host *host = dev_id;
	struct sg_mapping_iter *sg_miter = &host->sg_miter;
	struct variant_data *variant = host->variant;
	void __iomem *base = host->base;
	unsigned long flags;
	u32 status;

	status = readl(base + MMCISTATUS);

	dev_dbg(mmc_dev(host->mmc), "irq1 (pio) %08x\n", status);

	local_irq_save(flags);

	do {
		unsigned int remain, len;
		char *buffer;

		/*
		 * For write, we only need to test the half-empty flag
		 * here - if the FIFO is completely empty, then by
		 * definition it is more than half empty.
		 *
		 * For read, check for data available.
		 */
		if (!(status & (MCI_TXFIFOHALFEMPTY|MCI_RXDATAAVLBL)))
			break;

		if (!sg_miter_next(sg_miter))
			break;

		buffer = sg_miter->addr;
		remain = sg_miter->length;

		len = 0;
		if (status & MCI_RXACTIVE)
			len = mmci_pio_read(host, buffer, remain);
		if (status & MCI_TXACTIVE)
			len = mmci_pio_write(host, buffer, remain, status);

		sg_miter->consumed = len;

		host->size -= len;
		remain -= len;

		if (remain)
			break;

		status = readl(base + MMCISTATUS);
	} while (1);

	sg_miter_stop(sg_miter);

	local_irq_restore(flags);

	/*
	 * If we're nearing the end of the read, switch to
	 * "any data available" mode.
	 */
	if (status & MCI_RXACTIVE && host->size < variant->fifosize)
		mmci_set_mask1(host, MCI_RXDATAAVLBLMASK);

	/* If we run out of data, disable the data IRQs. */
	if (host->size == 0) {
		mmci_set_mask1(host, 0);

		/*
		 * If we already received MCI_DATAEND and the last
		 * MCI_DATABLOCKEND, the entire data transfer shall
		 * be completed.
		 */
		if (host->dataend && host->last_blockend) {
			mmci_complete_data_xfer(host);
		} else {
			/*
			 * Schedule a delayed work to make sure we do not end up
			 * forever waiting for a dataend irq.
			 */
			host->dataend_timeout_active = true;
			schedule_delayed_work(&host->dataend_timeout,
					msecs_to_jiffies(DATAEND_TIMEOUT_MS));
		}
	}

	return IRQ_HANDLED;
}

/*
 * Handle completion of command and data transfers.
 */
static irqreturn_t mmci_irq(int irq, void *dev_id)
{
	struct mmci_host *host = dev_id;
	u32 status;
	int sdio_irq = 0;
	int ret = 0;

	spin_lock(&host->lock);

	do {
		struct mmc_command *cmd;
		struct mmc_data *data;

		status = readl(host->base + MMCISTATUS);

		if (host->singleirq) {
			if (status & readl(host->base + MMCIMASK1))
				mmci_pio_irq(irq, dev_id);

			status &= ~MCI_IRQ1MASK;
		}

		status &= readl(host->base + MMCIMASK0);
		writel(status, host->base + MMCICLEAR);

		dev_dbg(mmc_dev(host->mmc), "irq0 (data+cmd) %08x\n", status);

		if (status & MCI_SDIOIT)
			sdio_irq = 1;

		data = host->data;
		if (status & MCI_DATA_IRQ && data)
			mmci_data_irq(host, data, status);

		cmd = host->cmd;
		if (status & MCI_CMD_IRQ && cmd)
			mmci_cmd_irq(host, cmd, status);

		ret = 1;
	} while (status);

	spin_unlock(&host->lock);

	if (sdio_irq)
		mmc_signal_sdio_irq(host->mmc);

	return IRQ_RETVAL(ret);
}

static void mmci_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct mmci_host *host = mmc_priv(mmc);
	struct variant_data *variant = host->variant;
	unsigned long flags;

	WARN_ON(host->mrq != NULL);

	if (mrq->data &&
	    (!variant->non_power_of_2_blksize ||
#ifdef CONFIG_ARCH_U8500
	     !cpu_is_u8500v20_or_later() ||
#endif
	    (mmc->card && mmc_card_ddr_mode(mmc->card))) &&
	    !is_power_of_2(mrq->data->blksz)) {
		dev_err(mmc_dev(mmc), "unsupported block size (%d bytes)\n",
			mrq->data->blksz);
		mrq->cmd->error = -EINVAL;
		mmc_request_done(mmc, mrq);
		return;
	}

	spin_lock_irqsave(&host->lock, flags);

	host->mrq = mrq;

	if (mrq->data && mrq->data->flags & MMC_DATA_READ)
		mmci_start_data(host, mrq->data);

	mmci_start_command(host, mrq->cmd, 0);

	spin_unlock_irqrestore(&host->lock, flags);
}

static void mmci_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct mmci_host *host = mmc_priv(mmc);
	struct variant_data *variant = host->variant;
	u32 pwr = 0;
	unsigned long flags;
	int ret;

	if (host->plat->vdd_handler)
		host->plat->vdd_handler(mmc_dev(mmc), ios->vdd,
				ios->power_mode);

	switch (ios->power_mode) {
	case MMC_POWER_OFF:
		if (host->vcard)
			ret = mmc_regulator_set_ocr(mmc, host->vcard, 0);
		break;
	case MMC_POWER_UP:
		if (host->vcard) {
			ret = mmc_regulator_set_ocr(mmc, host->vcard, ios->vdd);
			if (ret) {
				dev_err(mmc_dev(mmc), "unable to set OCR\n");
				/*
				 * The .set_ios() function in the mmc_host_ops
				 * struct return void, and failing to set the
				 * power should be rare so we print an error
				 * and return here.
				 */
				return;
			}
		}

		/*
		 * The ST Micro variant doesn't have the PL180s MCI_PWR_UP
		 * and instead uses MCI_PWR_ON so apply whatever value is
		 * configured in the variant data.
		 */
		pwr |= variant->pwrreg_powerup;

		break;
	case MMC_POWER_ON:
		pwr |= MCI_PWR_ON;
		break;
	}

	if (variant->signal_direction && ios->power_mode != MMC_POWER_OFF) {
		/*
		 * The ST Micro variant has some additional bits
		 * indicating signal direction for the signals in
		 * the SD/MMC bus and feedback-clock usage. The
		 * fall-throughs in the code below are intentional.
		 */

		if (host->plat->sigdir & MMCI_ST_DIRFBCLK)
			pwr |= MCI_ST_FBCLKEN;
		if (host->plat->sigdir & MMCI_ST_DIRCMD)
			pwr |= MCI_ST_CMDDIREN;

		switch (ios->bus_width) {
		case MMC_BUS_WIDTH_8:
			if (host->plat->sigdir & MMCI_ST_DIRDAT74)
				pwr |= MCI_ST_DATA74DIREN;
		case MMC_BUS_WIDTH_4:
			if (host->plat->sigdir & MMCI_ST_DIRDAT31)
				pwr |= MCI_ST_DATA31DIREN;
			if (host->plat->sigdir & MMCI_ST_DIRDAT2)
				pwr |= MCI_ST_DATA2DIREN;
		case MMC_BUS_WIDTH_1:
			if (host->plat->sigdir & MMCI_ST_DIRDAT0)
				pwr |= MCI_ST_DATA0DIREN;
			break;
		default:
			dev_err(mmc_dev(mmc), "unsupported MMC bus width %d\n",
				ios->bus_width);
			break;
		}
	}

	if (ios->bus_mode == MMC_BUSMODE_OPENDRAIN) {
		if (host->hw_designer != AMBA_VENDOR_ST)
			pwr |= MCI_ROD;
		else {
			/*
			 * The ST Micro variant use the ROD bit for something
			 * else and only has OD (Open Drain).
			 */
			if (mmc->card && (mmc->card->type == MMC_TYPE_MMC))
				pwr |= MCI_OD;
		}
	}

	spin_lock_irqsave(&host->lock, flags);

	mmci_set_clkreg(host, ios->clock);

	if (host->pwr != pwr) {
		host->pwr = pwr;
		writel(pwr, host->base + MMCIPOWER);
	}

	spin_unlock_irqrestore(&host->lock, flags);
}

static int mmci_get_ro(struct mmc_host *mmc)
{
	struct mmci_host *host = mmc_priv(mmc);

	if (host->gpio_wp == -ENOSYS)
		return -ENOSYS;

	return gpio_get_value_cansleep(host->gpio_wp);
}

static int mmci_get_cd(struct mmc_host *mmc)
{
	struct mmci_host *host = mmc_priv(mmc);
	struct mmci_platform_data *plat = host->plat;
	unsigned int status;

	if (host->gpio_cd == -ENOSYS) {
		if (!plat->status)
			return 1; /* Assume always present */

		status = plat->status(mmc_dev(host->mmc));
	} else
		status = !!gpio_get_value_cansleep(host->gpio_cd)
			^ plat->cd_invert;

	/*
	 * Use positive logic throughout - status is zero for no card,
	 * non-zero for card inserted.
	 */
	return status;
}

#ifdef CONFIG_PM
static int mmci_enable(struct mmc_host *mmc)
{
	unsigned long flags;
	struct mmci_host *host = mmc_priv(mmc);
	int ret = 0;

	clk_enable(host->clk);
	if (host->vcc) {
		ret = regulator_enable(host->vcc);
		if (ret) {
			dev_err(mmc_dev(host->mmc),
			"failed to enable regulator %s\n",
			host->plat->vcc);
			return ret;
		}
	}

	if (host->vcard) {
		if (regulator_set_mode(host->vcard, REGULATOR_MODE_NORMAL))
			dev_err(mmc_dev(host->mmc),
			"failed to set normal mode on regulator %s\n",
			host->plat->vcard);
	}

	if (pm_runtime_get_sync(mmc->parent) < 0)
		dev_err(mmc_dev(mmc), "failed pm_runtime_get_sync\n");

	spin_lock_irqsave(&host->lock, flags);

	/* Restore registers for POWER, CLOCK and IRQMASK0 */
	writel(host->clk_reg, host->base + MMCICLOCK);
	writel(host->pwr_reg, host->base + MMCIPOWER);
	writel(host->irqmask0_reg, host->base + MMCIMASK0);

	if (host->variant->sdio &&
		host->mmc->card &&
		mmc_card_sdio(host->mmc->card)) {
		/*
		 * The ST Micro variants has a special bit in the DATACTRL
		 * register to enable SDIO mode. This bit must be set otherwise
		 * no SDIO interrupts can be received.
		 */
		writel(MCI_ST_DPSM_SDIOEN|MCI_ST_DPSM_BUSYMODE, host->base + MMCIDATACTRL);
	}

	spin_unlock_irqrestore(&host->lock, flags);

	/* Restore settings done by the vdd_handler. */
	if (host->plat->vdd_handler)
		host->plat->vdd_handler(mmc_dev(mmc),
					mmc->ios.vdd,
					mmc->ios.power_mode);

	/*
	 * To be able to handle specific wake up scenarios for each host,
	 * the following function shall be implemented.
	 */
	if (host->plat->wakeup_handler)
		host->plat->wakeup_handler(host->mmc, true);

	return ret;
}

static int mmci_disable(struct mmc_host *mmc, int lazy)
{
	unsigned long flags;
	struct mmci_host *host = mmc_priv(mmc);

	/*
	 * To be able to handle specific shutdown scenarios for each host,
	 * the following function shall be implemented.
	 */
	if (host->plat->wakeup_handler)
		host->plat->wakeup_handler(host->mmc, false);

	/*
	 * Let the vdd_handler act on a POWER_OFF to potentially do some
	 * power save actions.
	 */
	if (host->plat->vdd_handler)
		host->plat->vdd_handler(mmc_dev(mmc), 0, MMC_POWER_OFF);

	spin_lock_irqsave(&host->lock, flags);

	/* Save registers for POWER, CLOCK and IRQMASK0 */
	host->irqmask0_reg = readl(host->base + MMCIMASK0);
	host->pwr_reg = readl(host->base + MMCIPOWER);
	host->clk_reg = readl(host->base + MMCICLOCK);

	/*
	 * Make sure we do not get any interrupts when we disabled the
	 * clock and the regulator and as well make sure to clear the
	 * registers for clock and power.
	 */
	writel(0, host->base + MMCIMASK0);
	writel(0, host->base + MMCIPOWER);
	writel(0, host->base + MMCICLOCK);

	spin_unlock_irqrestore(&host->lock, flags);

	if (pm_runtime_put_sync(mmc->parent) < 0)
		dev_err(mmc_dev(mmc), "failed pm_runtime_put_sync\n");

	clk_disable(host->clk);
	if (host->vcc)
		regulator_disable(host->vcc);

	if (host->vcard) {
		if (regulator_set_mode(host->vcard, REGULATOR_MODE_IDLE))
			dev_err(mmc_dev(host->mmc),
			"failed to set idle mode on regulator %s\n",
			host->plat->vcard);
	}

	return 0;
}
#else
#define mmci_enable	NULL
#define mmci_disable	NULL
#endif

static irqreturn_t mmci_cd_irq(int irq, void *dev_id)
{
	struct mmci_host *host = dev_id;

	if (host->plat->gpio_cd) {
		if (gpio_get_value(host->plat->gpio_cd) == 0) {
#ifdef _MMC_SAFE_ACCESS_
			mmc_is_available = 1;
			printk(KERN_ERR"[MMC]card inserted\n");
#endif
			mmc_detect_change(host->mmc, msecs_to_jiffies(200));
		} else {
#ifdef _MMC_SAFE_ACCESS_
			mmc_is_available = 0;
			printk(KERN_ERR"[MMC]card removed\n");
			mmc_detect_change(host->mmc, msecs_to_jiffies(50));
#endif
		}
      } else
			printk(KERN_ERR"gpio_cd is not defined\n");
	return IRQ_HANDLED;
}

static void mmci_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	unsigned long flags;
	unsigned int mask0;
	struct mmci_host *host = mmc_priv(mmc);

	spin_lock_irqsave(&host->lock, flags);

	mask0 = readl(host->base + MMCIMASK0);
	if (enable)
		mask0 |= MCI_SDIOIT;
	else
		mask0 &= ~MCI_SDIOIT;
	writel(mask0, host->base + MMCIMASK0);

	spin_unlock_irqrestore(&host->lock, flags);
}

#define PRCC_K_SOFTRST_SET	0x18
#define PRCC_K_SOFTRST_CLEAR	0x1C
static void mmci_abort_request(struct mmc_host *mmc)
{
	struct mmci_host *host = mmc_priv(mmc);
	unsigned long flags;
	unsigned int io_base;
	unsigned int value;

	spin_lock_irqsave(&host->lock, flags);

	dev_warn(mmc_dev(mmc), "Aborting request, %s%s\n",
		host->data ? "data" : "",
		host->cmd ? "cmd" : "");

	if (host->mrq) {
		if (host->data) {
			host->data->error = -ETIMEDOUT;

			if (host->dma_on_current_xfer)
				mmci_dma_terminate(host);

			mmci_stop_data(host);
		} else if (host->cmd) {
			host->cmd->error = -ETIMEDOUT;
		} else {
			host->mrq->cmd->error = -ETIMEDOUT;
		}

		host->cmd = NULL;
		host->mrq = NULL;

		host->dataend_timeout_active = false;
		__cancel_delayed_work(&host->dataend_timeout);

		writel(0, host->base + MMCICOMMAND);

		/* Reset the hosts mmci controller */
		if (host->phybase == U8500_SDI0_BASE) {
			value = 0x20;
			io_base = U8500_CLKRST1_BASE;
		} else if (host->phybase == U8500_SDI1_BASE) {
			value = 0x10;
			io_base = U8500_CLKRST2_BASE;
		} else if (host->phybase == U8500_SDI2_BASE) {
			value = 0x10;
			io_base = U8500_CLKRST3_BASE;
		} else if (host->phybase == U8500_SDI3_BASE) {
			value = 0x20;
			io_base = U8500_CLKRST2_BASE;
		} else if (host->phybase == U8500_SDI4_BASE) {
			value = 0x4;
			io_base = U8500_CLKRST2_BASE;
		} else if (host->phybase == U8500_SDI5_BASE) {
			value = 0x80;
			io_base = U8500_CLKRST3_BASE;
		} else {
			dev_warn(mmc_dev(mmc), "Unknown SDI controller\n");
			value = 0;
			io_base = 0;
		}

		if (io_base) {
			writel(value,
				__io_address(io_base + PRCC_K_SOFTRST_CLEAR));
			udelay(1);
			writel(value,
				__io_address(io_base + PRCC_K_SOFTRST_SET));
			udelay(1);
		}
	} else {
		dev_warn(mmc_dev(mmc), "Nothing to abort, "
			"request already completed?\n");
	}

	/* Disable DMA, use PIO */
	mmci_disable_dma(host);
	dev_warn(mmc_dev(mmc), "Disabling DMA for this host\n");
	host->dma_was_disabled = 1;

	spin_unlock_irqrestore(&host->lock, flags);
}

static const struct mmc_host_ops mmci_ops = {
	.request	= mmci_request,
	.set_ios	= mmci_set_ios,
	.get_ro		= mmci_get_ro,
	.get_cd		= mmci_get_cd,
	.enable		= mmci_enable,
	.disable	= mmci_disable,
	.enable_sdio_irq = mmci_enable_sdio_irq,
	.dump_regs	= dump_mmci_regs,
	.abort_request  = mmci_abort_request,
};

static int __devinit mmci_probe(struct amba_device *dev, struct amba_id *id)
{
	struct mmci_platform_data *plat = dev->dev.platform_data;
	struct variant_data *variant = id->data;
	struct mmci_host *host;
	struct mmc_host *mmc;
	int ret;

	/* must have platform data */
	if (!plat) {
		ret = -EINVAL;
		goto out;
	}

	ret = amba_request_regions(dev, DRIVER_NAME);
	if (ret)
		goto out;

	mmc = mmc_alloc_host(sizeof(struct mmci_host), &dev->dev);
	if (!mmc) {
		ret = -ENOMEM;
		goto rel_regions;
	}

	host = mmc_priv(mmc);
	host->mmc = mmc;
	host->plat = plat;
	host->variant = variant;

	host->gpio_wp = -ENOSYS;
	host->gpio_cd = -ENOSYS;
	host->gpio_cd_irq = -1;

	host->irqmask0_reg = 0;
	host->pwr_reg = 0;
	host->clk_reg = 0;

	host->dma_was_disabled = 0;

	host->hw_designer = amba_manf(dev);
	host->hw_revision = amba_rev(dev);
	dev_dbg(mmc_dev(mmc), "designer ID = 0x%02x\n", host->hw_designer);
	dev_dbg(mmc_dev(mmc), "revision = 0x%01x\n", host->hw_revision);

	host->clk = clk_get(&dev->dev, NULL);
	if (IS_ERR(host->clk)) {
		ret = PTR_ERR(host->clk);
		host->clk = NULL;
		goto host_free;
	}

	host->mclk = clk_get_rate(host->clk);
	/*
	 * According to the spec, mclk is max 100 MHz,
	 * so we try to adjust the clock down to this,
	 * (if possible).
	 */
	if (host->mclk > 100000000) {
		ret = clk_set_rate(host->clk, 100000000);
		if (ret < 0)
			goto clk_free;
		host->mclk = clk_get_rate(host->clk);
		dev_dbg(mmc_dev(mmc), "eventual mclk rate: %u Hz\n",
			host->mclk);
	}
	host->phybase = dev->res.start;
	host->base = ioremap(dev->res.start, resource_size(&dev->res));
	if (!host->base) {
		ret = -ENOMEM;
		goto clk_free;
	}

	mmc->ops = &mmci_ops;
	if (variant->st_clkdiv)
		mmc->f_min = host->mclk / 257;
	else
		mmc->f_min = (host->mclk + 511) / 512;
	/*
	 * If the platform data supplies a maximum operating
	 * frequency, this takes precedence. Else, we fall back
	 * to using the module parameter, which has a (low)
	 * default value in case it is not specified. Either
	 * value must not exceed the clock rate into the block,
	 * of course.
	 */
	if (plat->f_max)
		mmc->f_max = min(host->mclk, plat->f_max);
	else
		mmc->f_max = min(host->mclk, fmax);
	dev_dbg(mmc_dev(mmc), "clocking block at %u Hz\n", mmc->f_max);

	/* Host regulator */
	if (plat->vcc) {
		host->vcc = regulator_get(&dev->dev, plat->vcc);
		ret = IS_ERR(host->vcc);
		if (ret) {
			host->vcc = NULL;
			goto clk_free;
		}
		dev_dbg(mmc_dev(host->mmc), "fetched regulator %s\n",
			plat->vcc);
	}

	/* Card regulator (could be a level shifter) */
	if (plat->vcard) {
		int mask;
		host->vcard = regulator_get(&dev->dev, plat->vcard);
		ret = IS_ERR(host->vcard);
		if (ret) {
			host->vcard = NULL;
			goto vcc_free;
		}
		dev_dbg(mmc_dev(host->mmc), "fetched regulator %s\n",
			plat->vcard);

		mask = mmc_regulator_get_ocrmask(host->vcard);
		if (mask < 0)
			dev_err(&dev->dev, "error getting OCR mask (%d)\n",
				mask);
		else {
			host->mmc->ocr_avail = (u32) mask;
			if (plat->ocr_mask)
				dev_warn(&dev->dev,
				 "Provided ocr_mask/setpower will not be used "
				 "(using regulator instead)\n");
		}

	} else {
		/* Use platform data if no regulator for vcard is used */
		mmc->ocr_avail = plat->ocr_mask;
	}

	/* Use platform capabilities */
	mmc->caps = plat->capabilities;

#ifdef _MMC_SAFE_ACCESS_
	mmc_is_available = 1;
#endif

	/* Set disable timeout if supported */
	if (mmc->caps & MMC_CAP_DISABLE)
		mmc_set_disable_delay(mmc, plat->disable);

	/* We support these PM capabilities. */
	mmc->pm_caps = MMC_PM_KEEP_POWER;

	/*
	 * We can do SGIO
	 */
	mmc->max_segs = NR_SG;

	/*
	 * Since only a certain number of bits are valid in the data length
	 * register, we must ensure that we don't exceed 2^num-1 bytes in a
	 * single request.
	 */
	mmc->max_req_size = (1 << variant->datalength_bits) - 1;

	/*
	 * Set the maximum segment size. Right now DMA sets the
	 * limit and not the data length register. Thus until the DMA
	 * driver not handles this, the segment size is limited by DMA.
	 * DMA limit: src_addr_width x (64 KB -1). src_addr_width
	 * can be 1.
	 */
	mmc->max_seg_size = 65535;

	/*
	 * Block size can be up to 2048 bytes, but must be a power of two.
	 */
	mmc->max_blk_size = 2048;

	/*
	 * No limit on the number of blocks transferred.
	 */
	mmc->max_blk_count = mmc->max_req_size;

	spin_lock_init(&host->lock);

	if (gpio_is_valid(plat->gpio_cd)) {
		ret = gpio_request(plat->gpio_cd, DRIVER_NAME " (cd)");
		if (ret == 0)
			ret = gpio_direction_input(plat->gpio_cd);
		if (ret == 0)
			host->gpio_cd = plat->gpio_cd;
		else if (ret != -ENOSYS)
			goto err_gpio_cd;

		ret = request_any_context_irq(gpio_to_irq(plat->gpio_cd),
					      mmci_cd_irq,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
					      DRIVER_NAME " (cd)", host);
		if (ret >= 0)
			host->gpio_cd_irq = gpio_to_irq(plat->gpio_cd);
	}
	if (gpio_is_valid(plat->gpio_wp)) {
		ret = gpio_request(plat->gpio_wp, DRIVER_NAME " (wp)");
		if (ret == 0)
			ret = gpio_direction_input(plat->gpio_wp);
		if (ret == 0)
			host->gpio_wp = plat->gpio_wp;
		else if (ret != -ENOSYS)
			goto err_gpio_wp;
	}

	if ((host->plat->status || host->gpio_cd != -ENOSYS)
	    && host->gpio_cd_irq < 0
	    && !(mmc->caps & MMC_CAP_NONREMOVABLE))
		mmc->caps |= MMC_CAP_NEEDS_POLL;

	mmci_setup_dma(host);
	INIT_DELAYED_WORK(&host->dataend_timeout, mmci_dataend_timeout);

	ret = request_irq(dev->irq[0], mmci_irq, IRQF_SHARED,
			DRIVER_NAME " (cmd)", host);
	if (ret)
		goto unmap;

	if (dev->irq[1] == NO_IRQ)
		host->singleirq = true;
	else {
		ret = request_irq(dev->irq[1], mmci_pio_irq, IRQF_SHARED,
				  DRIVER_NAME " (pio)", host);
		if (ret)
			goto irq0_free;
	}

	/* Prepare IRQMASK0 */
	host->irqmask0_reg = MCI_IRQENABLE;
	if (host->variant->broken_blockend)
		host->irqmask0_reg &= ~MCI_DATABLOCKEND;

	amba_set_drvdata(dev, mmc);

	pm_runtime_enable(mmc->parent);
	if (pm_runtime_get_sync(mmc->parent) < 0)
		dev_err(mmc_dev(mmc), "failed pm_runtime_get_sync\n");
	if (pm_runtime_put_sync(mmc->parent) < 0)
		dev_err(mmc_dev(mmc), "failed pm_runtime_put_sync\n");

	mmc_add_host(mmc);

	if (host->plat->suspend_resume_handler)
		host->plat->suspend_resume_handler(host->mmc, 0);

	dev_info(&dev->dev,
		"%s: MMCI/PL180 manf %x rev %x cfg %02x at 0x%016llx\n",
		 mmc_hostname(mmc), amba_manf(dev), amba_rev(dev),
		 amba_config(dev), (unsigned long long)dev->res.start);
	dev_info(&dev->dev, "IRQ %d, %d (pio)\n", dev->irq[0], dev->irq[1]);

	/* Ugly hack for u8500_sdio_detect_card, to be removed soon. */
	sdio_host_ptr = host;

	mmci_debugfs_create(host);

	return 0;

 irq0_free:
	free_irq(dev->irq[0], host);
 unmap:
	mmci_disable_dma(host);
	if (host->gpio_wp != -ENOSYS)
		gpio_free(host->gpio_wp);
 err_gpio_wp:
	if (host->gpio_cd_irq >= 0)
		free_irq(host->gpio_cd_irq, host);
	if (host->gpio_cd != -ENOSYS)
		gpio_free(host->gpio_cd);
 err_gpio_cd:
	iounmap(host->base);
 vcc_free:
	if (host->vcc)
		regulator_put(host->vcc);
 clk_free:
	clk_put(host->clk);
 host_free:
	mmc_free_host(mmc);
 rel_regions:
	amba_release_regions(dev);
 out:
	return ret;
}

static int __devexit mmci_remove(struct amba_device *dev)
{
	struct mmc_host *mmc = amba_get_drvdata(dev);

	amba_set_drvdata(dev, NULL);

	if (mmc) {
		struct mmci_host *host = mmc_priv(mmc);

		pm_runtime_disable(mmc->parent);

		if (host->vcc)
			regulator_enable(host->vcc);

		mmci_debugfs_remove(host);
		mmc_remove_host(mmc);

		writel(0, host->base + MMCIMASK0);
		writel(0, host->base + MMCIMASK1);

		writel(0, host->base + MMCICOMMAND);
		writel(0, host->base + MMCIDATACTRL);

		mmci_disable_dma(host);
		cancel_delayed_work(&host->dataend_timeout);

		free_irq(dev->irq[0], host);
		if (!host->singleirq)
			free_irq(dev->irq[1], host);

		if (host->gpio_wp != -ENOSYS)
			gpio_free(host->gpio_wp);
		if (host->gpio_cd_irq >= 0)
			free_irq(host->gpio_cd_irq, host);
		if (host->gpio_cd != -ENOSYS)
			gpio_free(host->gpio_cd);

		iounmap(host->base);
		clk_disable(host->clk);
		clk_put(host->clk);

		if (host->vcc) {
			regulator_disable(host->vcc);
			regulator_put(host->vcc);
		}

		if (host->vcard) {
			mmc_regulator_set_ocr(mmc, host->vcard, 0);
			regulator_put(host->vcard);
		}

		mmc_free_host(mmc);

		amba_release_regions(dev);
	}

	return 0;
}

#ifdef CONFIG_PM
static int mmci_suspend(struct amba_device *dev, pm_message_t state)
{
	struct mmc_host *mmc = amba_get_drvdata(dev);
	int ret = 0;

	if (mmc) {
		/*
		 * The host must be claimed to prevent request handling etc.
		 * Also make sure to not sleep, since we must return with a
		 * response quite quickly.
		 */
		if (mmc_try_claim_host(mmc)) {
			struct mmci_host *host = mmc_priv(mmc);

			if (mmc->enabled) {
				/*
				 * Do not suspend if the host has not been
				 * disabled.
				 */
				mmc_do_release_host(mmc);
				ret = -EBUSY;
			} else if ((!(host->plat->pm_flags & MMC_PM_KEEP_POWER))
				&& (!(mmc->pm_flags & MMC_PM_KEEP_POWER))) {
				/* Cut the power to the card to save current. */
				mmc_host_enable(mmc);
				mmc_power_save_host(mmc);
				mmc_host_disable(mmc);
				host->pwr_reg = 0;
				host->clk_reg = 0;
			}
		} else {
			/*
			 * We did not manage to claim the host, thus someone
			 * is still using it. Do not suspend.
			 */
			ret = -EBUSY;
		}
	}

	return ret;
}

static int mmci_resume(struct amba_device *dev)
{
	struct mmc_host *mmc = amba_get_drvdata(dev);

	if (mmc) {
		struct mmci_host *host = mmc_priv(mmc);

		/* We expect the host to be claimed. */
		WARN_ON(!mmc->claimed);

		/* Restore power and re-initialize the card. */
		if ((!(host->plat->pm_flags & MMC_PM_KEEP_POWER)) &&
			(!(mmc->pm_flags & MMC_PM_KEEP_POWER))) {
			mmc_host_enable(mmc);
			mmc_power_restore_host(mmc);
			mmc_host_disable(mmc);
		}

		if (!host->dma_enable && host->dma_was_disabled)
			mmci_setup_dma(host);
		
		/* Release the host to provide access to it again. */
		mmc_do_release_host(mmc);
	}

	return 0;
}
#else
#define mmci_suspend	NULL
#define mmci_resume	NULL
#endif

static struct amba_id mmci_ids[] = {
	{
		.id	= 0x00041180,
		.mask	= 0x000fffff,
		.data	= &variant_arm,
	},
	{
		.id	= 0x00041181,
		.mask	= 0x000fffff,
		.data	= &variant_arm,
	},
	/* ST Micro variants */
	{
		.id     = 0x00180180,
		.mask   = 0x00ffffff,
		.data	= &variant_u300,
	},
	{
		.id     = 0x00280180,
		.mask   = 0x00ffffff,
		.data	= &variant_u300,
	},
	{
		.id     = 0x00480180,
		.mask   = 0x00ffffff,
		.data	= &variant_ux500,
	},
	{ 0, 0 },
};

static struct amba_driver mmci_driver = {
	.drv		= {
		.name	= DRIVER_NAME,
	},
	.probe		= mmci_probe,
	.remove		= __devexit_p(mmci_remove),
	.suspend	= mmci_suspend,
	.resume		= mmci_resume,
	.id_table	= mmci_ids,
};

static int __init mmci_init(void)
{
	return amba_driver_register(&mmci_driver);
}

static void __exit mmci_exit(void)
{
	amba_driver_unregister(&mmci_driver);
}

module_init(mmci_init);
module_exit(mmci_exit);
module_param(fmax, uint, 0444);

MODULE_DESCRIPTION("ARM PrimeCell PL180/181 Multimedia Card Interface driver");
MODULE_LICENSE("GPL");
