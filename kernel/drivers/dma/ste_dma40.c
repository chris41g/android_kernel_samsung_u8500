/*
 * Copyright (C) ST-Ericsson SA 2007-2010
 * Author: Per Forlin <per.forlin@stericsson.com> for ST-Ericsson
 * Author: Jonas Aaberg <jonas.aberg@stericsson.com> for ST-Ericsson
 * License terms: GNU General Public License (GPL) version 2
 */

#if defined(CONFIG_MACH_CODINA) || defined(CONFIG_MACH_GAVINI)
#define MMC_HOST_DEBUGGING
#endif

#ifdef MMC_HOST_DEBUGGING
#define CHANNEL_OF_CHOICE 0
#endif

#include <linux/kernel.h>
#include <linux/dmaengine.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>

#include <plat/ste_dma40.h>

#include "ste_dma40_ll.h"

#ifdef CONFIG_STE_DMA40_DEBUG
#include "ste_dma40_debug.h"
#define MARK sted40_history_text((char *)__func__)
#else
#define MARK
#endif

#define D40_NAME "dma40"

#define D40_PHY_CHAN -1

/* For masking out/in 2 bit channel positions */
#define D40_CHAN_POS(chan)  (2 * (chan / 2))
#define D40_CHAN_POS_MASK(chan) (0x3 << D40_CHAN_POS(chan))

/* Maximum iterations taken before giving up suspending a channel */
#define D40_SUSPEND_MAX_IT 500

/* Milliseconds */
#define DMA40_AUTOSUSPEND_DELAY	100

/* Hardware requirement on LCLA alignment */
#define LCLA_ALIGNMENT 0x40000

/* Max number of links per event group */
#define D40_LCLA_LINK_PER_EVENT_GRP 128
#define D40_LCLA_END D40_LCLA_LINK_PER_EVENT_GRP

/* Attempts before giving up to trying to get pages that are aligned */
#define MAX_LCLA_ALLOC_ATTEMPTS 256

/* Bit markings for allocation map */
#define D40_ALLOC_FREE		(1 << 31)
#define D40_ALLOC_PHY		(1 << 30)
#define D40_ALLOC_LOG_FREE	0

/* Hardware designer of the block */
#define D40_PERIPHID2_DESIGNER 0x8

/**
 * enum 40_command - The different commands and/or statuses.
 *
 * @D40_DMA_STOP: DMA channel command STOP or status STOPPED,
 * @D40_DMA_RUN: The DMA channel is RUNNING of the command RUN.
 * @D40_DMA_SUSPEND_REQ: Request the DMA to SUSPEND as soon as possible.
 * @D40_DMA_SUSPENDED: The DMA channel is SUSPENDED.
 */
enum d40_command {
	D40_DMA_STOP		= 0,
	D40_DMA_RUN		= 1,
	D40_DMA_SUSPEND_REQ	= 2,
	D40_DMA_SUSPENDED	= 3
};

/*
 * These are the registers that has to be saved and later restored
 * when the DMA hw is powered off.
 * TODO: Add save/restore of D40_DREG_GCC on dma40 v3 or later, if that works.
 */
static u32 d40_backup_regs[] = {
	D40_DREG_LCPA,
	D40_DREG_LCLA,
	D40_DREG_PRMSE,
	D40_DREG_PRMSO,
	D40_DREG_PRMOE,
	D40_DREG_PRMOO,
};

/* TODO: Check if all these registers have to be saved/restored on dma40 v3 */
static u32 d40_backup_regs_v3[] = {
	D40_DREG_PSEG1,
	D40_DREG_PSEG2,
	D40_DREG_PSEG3,
	D40_DREG_PSEG4,
	D40_DREG_PCEG1,
	D40_DREG_PCEG2,
	D40_DREG_PCEG3,
	D40_DREG_PCEG4,
	D40_DREG_RSEG1,
	D40_DREG_RSEG2,
	D40_DREG_RSEG3,
	D40_DREG_RSEG4,
	D40_DREG_RCEG1,
	D40_DREG_RCEG2,
	D40_DREG_RCEG3,
	D40_DREG_RCEG4,
};

static u32 d40_backup_regs_chan[] = {
	D40_CHAN_REG_SSCFG,
	D40_CHAN_REG_SSELT,
	D40_CHAN_REG_SSPTR,
	D40_CHAN_REG_SSLNK,
	D40_CHAN_REG_SDCFG,
	D40_CHAN_REG_SDELT,
	D40_CHAN_REG_SDPTR,
	D40_CHAN_REG_SDLNK,
};

/**
 * struct d40_lli_pool - Structure for keeping LLIs in memory
 *
 * @base: Pointer to memory area when the pre_alloc_lli's are not large
 * enough, IE bigger than the most common case, 1 dst and 1 src. NULL if
 * pre_alloc_lli is used.
 * @size: The size in bytes of the memory at base or the size of pre_alloc_lli.
 * @pre_alloc_lli: Pre allocated area for the most common case of transfers,
 * one buffer to one buffer.
 */
struct d40_lli_pool {
	void	*base;
	int	 size;
	/* Space for dst and src, plus an extra for padding */
	u8	 pre_alloc_lli[3 * sizeof(struct d40_phy_lli)];
};

/**
 * struct d40_desc - A descriptor is one DMA job.
 *
 * @lli_phy: LLI settings for physical channel. Both src and dst=
 * points into the lli_pool, to base if lli_len > 1 or to pre_alloc_lli if
 * lli_len equals one.
 * @lli_log: Same as above but for logical channels.
 * @last_lcla: lcla used for last link (logical channels)
 * @lli_pool: The pool with two entries pre-allocated.
 * @lli_len: Number of llis of current descriptor.
 * @lli_current: Number of transfered llis.
 * @lcla_alloc: Number of LCLA entries allocated.
 * @txd: DMA engine struct. Used for among other things for communication
 * during a transfer.
 * @node: List entry.
 * @is_in_client_list: true if the client owns this descriptor.
 * @cyclic: true if this is a cyclic job
 *
 * This descriptor is used for both logical and physical transfers.
 */
struct d40_desc {
	/* LLI physical */
	struct d40_phy_lli_bidir	 lli_phy;
	/* LLI logical */
	struct d40_log_lli_bidir	 lli_log;
	struct d40_log_lli		*last_lcla;

	struct d40_lli_pool		 lli_pool;
	int				 lli_len;
	int				 lli_current;
	int				 lcla_alloc;

	struct dma_async_tx_descriptor	 txd;
	struct list_head		 node;

	bool				 is_in_client_list;
	bool				 cyclic;
};

/**
 * struct d40_lcla_pool - LCLA pool settings and data.
 *
 * @base: The virtual address of LCLA. 18 bit aligned.
 * @base_unaligned: The orignal kmalloc pointer, if kmalloc is used.
 * This pointer is only there for clean-up on error.
 * @pages: The number of pages needed for all physical channels.
 * Only used later for clean-up on error
 * @lock: Lock to protect the content in this struct.
 * @alloc_map: big map over which LCLA entry is own by which job.
 */
struct d40_lcla_pool {
	void		*base;
	void		*base_unaligned;
	int		 pages;
	spinlock_t	 lock;
	struct d40_desc	**alloc_map;
};

/**
 * struct d40_phy_res - struct for handling eventlines mapped to physical
 * channels.
 *
 * @lock: A lock protection this entity.
 * @reserved: True if used by secure world or otherwise.
 * @num: The physical channel number of this entity.
 * @allocated_src: Bit mapped to show which src event line's are mapped to
 * this physical channel. Can also be free or physically allocated.
 * @allocated_dst: Same as for src but is dst.
 * allocated_dst and allocated_src uses the D40_ALLOC* defines as well as
 * event line number.
 */
struct d40_phy_res {
	spinlock_t lock;
	bool	   reserved;
	int	   num;
	u32	   allocated_src;
	u32	   allocated_dst;
};

struct d40_base;

/**
 * struct d40_chan - Struct that describes a channel.
 *
 * @lock: A spinlock to protect this struct.
 * @log_num: The logical number, if any of this channel.
 * @completed: Starts with 1, after first interrupt it is set to dma engine's
 * current cookie.
 * @pending_tx: The number of pending transfers. Used between interrupt handler
 * and tasklet.
 * @busy: Set to true when transfer is ongoing on this channel.
 * @phy_chan: Pointer to physical channel which this instance runs on. If this
 * point is NULL, then the channel is not allocated.
 * @chan: DMA engine handle.
 * @tasklet: Tasklet that gets scheduled from interrupt context to complete a
 * transfer and call client callback.
 * @client: Cliented owned descriptor list.
 * @active: Active descriptor.
 * @done: Completed jobs
 * @queue: Queued jobs.
 * @dma_cfg: The client configuration of this dma channel.
 * @configured: whether the dma_cfg configuration is valid
 * @base: Pointer to the device instance struct.
 * @cdesc: Cyclic descriptor
 * @src_def_cfg: Default cfg register setting for src.
 * @dst_def_cfg: Default cfg register setting for dst.
 * @log_def: Default logical channel settings.
 * @lcpa: Pointer to dst and src lcpa settings.
 * @runtime_addr: runtime configured address.
 * @runtime_direction: runtime configured direction.
 * @src_dev_addr: device source address for the channel transfer.
 * @dst_dev_addr: device destination address for the channel transfer.
 *
 * This struct can either "be" a logical or a physical channel.
 */
struct d40_chan {
	spinlock_t			 lock;
	int				 log_num;
	/* ID of the most recent completed transfer */
	int				 completed;
	int				 pending_tx;
	bool				 busy;
	struct d40_phy_res		*phy_chan;
	struct dma_chan			 chan;
	struct tasklet_struct		 tasklet;
	struct list_head		 client;
	struct list_head		 active;
	struct list_head		 done;
	struct list_head		 queue;
	struct stedma40_chan_cfg	 dma_cfg;
	bool				 configured;
	struct d40_base			*base;
	struct stedma40_cyclic_desc	*cdesc;
	/* Default register configurations */
	u32				 src_def_cfg;
	u32				 dst_def_cfg;
	struct d40_def_lcsp		 log_def;
	struct d40_log_lli_full		*lcpa;
	/* Runtime reconfiguration */
	dma_addr_t			runtime_addr;
	enum dma_data_direction		runtime_direction;
	dma_addr_t			 src_dev_addr;
	dma_addr_t			 dst_dev_addr;
#ifdef MMC_HOST_DEBUGGING
	struct list_head		list;
#endif
};

/**
 * struct d40_base - The big global struct, one for each probe'd instance.
 *
 * @interrupt_lock: Lock used to make sure one interrupt is handle a time.
 * @execmd_lock: Lock for execute command usage since several channels share
 * the same physical register.
 * @dev: The device structure.
 * @virtbase: The virtual base address of the DMA's register.
 * @rev: silicon revision detected.
 * @clk: Pointer to the DMA clock structure.
 * @phy_start: Physical memory start of the DMA registers.
 * @phy_size: Size of the DMA register map.
 * @irq: The IRQ number.
 * @num_phy_chans: The number of physical channels. Read from HW. This
 * is the number of available channels for this driver, not counting "Secure
 * mode" allocated physical channels.
 * @num_log_chans: The number of logical channels. Calculated from
 * num_phy_chans.
 * @dma_both: dma_device channels that can do both memcpy and slave transfers.
 * @dma_slave: dma_device channels that can do only do slave transfers.
 * @dma_memcpy: dma_device channels that can do only do memcpy transfers.
 * @phy_chans: Room for all possible physical channels in system.
 * @log_chans: Room for all possible logical channels in system.
 * @lookup_log_chans: Used to map interrupt number to logical channel. Points
 * to log_chans entries.
 * @lookup_phy_chans: Used to map interrupt number to physical channel. Points
 * to phy_chans entries.
 * @plat_data: Pointer to provided platform_data which is the driver
 * configuration.
 * @lcpa_regulator: Pointer to hold the regulator for the esram bank for lcla.
 * @phy_res: Vector containing all physical channels.
 * @lcla_pool: lcla pool settings and data.
 * @lcpa_base: The virtual mapped address of LCPA.
 * @phy_lcpa: The physical address of the LCPA.
 * @lcpa_size: The size of the LCPA area.
 * @desc_slab: cache for descriptors.
 * @usage: The number of dma executions. Used by suspend to determite if
 * the dma can suspend or not.
 * @usage_lock: lock for usage count.
 * @reg_val_backup: Here the values of some hardware registers are stored
 * before the DMA is powered off. They are restored when the power is back on.
 * @reg_val_backup_v3: Backup of registers that only exits on dma40 v3 and
 * later.
 * @reg_val_backup_chan: Backup data for standard channel parameter registers.
 * @initialized: true if the dma has been initialized
 */
struct d40_base {
	spinlock_t			 interrupt_lock;
	spinlock_t			 execmd_lock;
	struct device			 *dev;
	void __iomem			 *virtbase;
	u8				  rev:4;
	struct clk			 *clk;
	phys_addr_t			  phy_start;
	resource_size_t			  phy_size;
	int				  irq;
	int				  num_phy_chans;
	int				  num_log_chans;
	struct dma_device		  dma_both;
	struct dma_device		  dma_slave;
	struct dma_device		  dma_memcpy;
	struct d40_chan			 *phy_chans;
	struct d40_chan			 *log_chans;
	struct d40_chan			**lookup_log_chans;
	struct d40_chan			**lookup_phy_chans;
	struct stedma40_platform_data	 *plat_data;
	struct regulator		 *lcpa_regulator;
	/* Physical half channels */
	struct d40_phy_res		 *phy_res;
	struct d40_lcla_pool		  lcla_pool;
	void				 *lcpa_base;
	dma_addr_t			  phy_lcpa;
	resource_size_t			  lcpa_size;
	struct kmem_cache		 *desc_slab;
	int				  usage;
	spinlock_t			  usage_lock;
	u32				  reg_val_backup
					  [ARRAY_SIZE(d40_backup_regs)];
	u32				  reg_val_backup_v3
					  [ARRAY_SIZE(d40_backup_regs_v3)];
	u32				 *reg_val_backup_chan;
	bool				  initialized;
};

/**
 * struct d40_interrupt_lookup - lookup table for interrupt handler
 *
 * @src: Interrupt mask register.
 * @clr: Interrupt clear register.
 * @is_error: true if this is an error interrupt.
 * @offset: start delta in the lookup_log_chans in d40_base. If equals to
 * D40_PHY_CHAN, the lookup_phy_chans shall be used instead.
 */
struct d40_interrupt_lookup {
	u32 src;
	u32 clr;
	bool is_error;
	int offset;
};

/**
 * struct d40_reg_val - simple lookup struct
 *
 * @reg: The register.
 * @val: The value that belongs to the register in reg.
 */
struct d40_reg_val {
	unsigned int reg;
	unsigned int val;
};

static struct device *chan2dev(struct d40_chan *d40c)
{
	return &d40c->chan.dev->device;
}

static bool chan_is_physical(struct d40_chan *chan)
{
	return chan->log_num == D40_PHY_CHAN;
}

static bool chan_is_logical(struct d40_chan *chan)
{
	return !chan_is_physical(chan);
}

static int d40_pool_lli_alloc(struct d40_desc *d40d,
			      int lli_len, bool is_log)
{
	u32 align;
	void *base;

	if (is_log)
		align = sizeof(struct d40_log_lli);
	else
		align = sizeof(struct d40_phy_lli);

	if (lli_len == 1) {
		base = d40d->lli_pool.pre_alloc_lli;
		d40d->lli_pool.size = sizeof(d40d->lli_pool.pre_alloc_lli);
		d40d->lli_pool.base = NULL;
	} else {
		d40d->lli_pool.size = ALIGN(lli_len * 2 * align, align);

		base = kmalloc(d40d->lli_pool.size + align, GFP_NOWAIT);
		d40d->lli_pool.base = base;

		if (d40d->lli_pool.base == NULL)
			return -ENOMEM;
	}

	if (is_log) {
		d40d->lli_log.src = PTR_ALIGN((struct d40_log_lli *) base,
					      align);
		d40d->lli_log.dst = PTR_ALIGN(d40d->lli_log.src + lli_len,
					      align);
	} else {
		d40d->lli_phy.src = PTR_ALIGN((struct d40_phy_lli *)base,
					      align);
		d40d->lli_phy.dst = PTR_ALIGN(d40d->lli_phy.src + lli_len,
					      align);
	}

	return 0;
}

static void d40_pool_lli_free(struct d40_desc *d40d)
{
	kfree(d40d->lli_pool.base);
	d40d->lli_pool.base = NULL;
	d40d->lli_pool.size = 0;
	d40d->lli_log.src = NULL;
	d40d->lli_log.dst = NULL;
	d40d->lli_phy.src = NULL;
	d40d->lli_phy.dst = NULL;
	d40d->last_lcla = NULL;
}

static int d40_lcla_alloc_one(struct d40_chan *d40c,
			      struct d40_desc *d40d)
{
	unsigned long flags;
	int i;
	int ret = -EINVAL;

	spin_lock_irqsave(&d40c->base->lcla_pool.lock, flags);

	/*
	 * Allocate both src and dst at the same time, therefore the half
	 * start on 1 since 0 can't be used since zero is used as end marker.
	 */
	for (i = 1 ; i < D40_LCLA_LINK_PER_EVENT_GRP / 2; i++) {
		int idx = d40c->phy_chan->num * D40_LCLA_LINK_PER_EVENT_GRP + i;

		if (!d40c->base->lcla_pool.alloc_map[idx]) {
			d40c->base->lcla_pool.alloc_map[idx] = d40d;
			d40d->lcla_alloc++;
			ret = i;
			break;
		}
	}

	spin_unlock_irqrestore(&d40c->base->lcla_pool.lock, flags);

	return ret;
}

static int d40_lcla_free_all(struct d40_chan *d40c,
			     struct d40_desc *d40d)
{
	unsigned long flags;
	int i;
	int ret = -EINVAL;

	if (chan_is_physical(d40c))
		return 0;

	spin_lock_irqsave(&d40c->base->lcla_pool.lock, flags);

	for (i = 1 ; i < D40_LCLA_LINK_PER_EVENT_GRP / 2; i++) {
		int idx = d40c->phy_chan->num * D40_LCLA_LINK_PER_EVENT_GRP + i;

		if (d40c->base->lcla_pool.alloc_map[idx] == d40d) {
			d40c->base->lcla_pool.alloc_map[idx] = NULL;
			d40d->lcla_alloc--;
			if (d40d->lcla_alloc == 0) {
				ret = 0;
				break;
			}
		}
	}

	spin_unlock_irqrestore(&d40c->base->lcla_pool.lock, flags);

	return ret;

}

static void d40_desc_remove(struct d40_desc *d40d)
{
	list_del(&d40d->node);
}

static struct d40_desc *d40_desc_get(struct d40_chan *d40c)
{
	struct d40_desc *desc = NULL;

	if (!list_empty(&d40c->client)) {
		struct d40_desc *d;
		struct d40_desc *_d;

		list_for_each_entry_safe(d, _d, &d40c->client, node) {
			if (async_tx_test_ack(&d->txd)) {
				d40_pool_lli_free(d);
				d40_desc_remove(d);
				desc = d;
				memset(desc, 0, sizeof(struct d40_desc));
				break;
			}
		}
	}

	if (!desc)
		desc = kmem_cache_zalloc(d40c->base->desc_slab, GFP_NOWAIT);

	if (desc)
		INIT_LIST_HEAD(&desc->node);

	return desc;
}

static void d40_desc_free(struct d40_chan *d40c, struct d40_desc *d40d)
{

	d40_lcla_free_all(d40c, d40d);
	kmem_cache_free(d40c->base->desc_slab, d40d);
}

static void d40_desc_submit(struct d40_chan *d40c, struct d40_desc *desc)
{
	list_add_tail(&desc->node, &d40c->active);
}

static void d40_desc_done(struct d40_chan *d40c, struct d40_desc *desc)
{
	list_add_tail(&desc->node, &d40c->done);
}

static int d40_desc_log_lli_to_lcxa(struct d40_chan *d40c,
				    struct d40_desc *d40d,
				    bool use_lcpa)
{
	struct d40_log_lli_bidir *lli = &d40d->lli_log;
	int curr_lcla = -EINVAL;
	int first_lcla = 0;
	bool use_esram_lcla = d40c->base->plat_data->use_esram_lcla;

	if ((d40d->lli_len - d40d->lli_current) > 1 ||
	     d40d->cyclic || !use_lcpa) {

		curr_lcla = d40_lcla_alloc_one(d40c, d40d);
		first_lcla = curr_lcla;
	}

	if (!d40d->cyclic && use_lcpa) {
		d40_log_lli_lcpa_write(d40c->lcpa,
				       &lli->dst[d40d->lli_current],
				       &lli->src[d40d->lli_current],
				       curr_lcla,
				       curr_lcla == -EINVAL);

		d40d->lli_current++;
	}

	/*
	 * Run only in LCPA space for non-cyclic.  For cyclic, caller
	 * will handle the error.
	 */
	if (first_lcla < 0)
		return first_lcla;

	for (; d40d->lli_current < d40d->lli_len; d40d->lli_current++) {
		int lli_current = d40d->lli_current;
		struct d40_log_lli *lcla;
		int next_lcla;
		bool interrupt;

		if (d40d->lli_current + 1 < d40d->lli_len)
			next_lcla = d40_lcla_alloc_one(d40c, d40d);
		else
			next_lcla = d40d->cyclic ? first_lcla : -EINVAL;

		interrupt = d40d->cyclic
			    ? d40d->txd.flags & DMA_PREP_INTERRUPT
			    : next_lcla == -EINVAL;

		if (d40d->cyclic && curr_lcla == first_lcla) {
			/*
			 * For cyclic transactions, the first link is
			 * present in both LCPA and LCLA space because
			 * we can't link back to the one in LCPA space.
			 */
			d40_log_lli_lcpa_write(d40c->lcpa,
					       &lli->dst[lli_current],
					       &lli->src[lli_current],
					       next_lcla,
					       interrupt);
		}

		lcla = d40c->base->lcla_pool.base +
			d40c->phy_chan->num * 1024 +
			8 * curr_lcla * 2;

		d40_log_lli_lcla_write(lcla,
				       &lli->dst[lli_current],
				       &lli->src[lli_current],
				       next_lcla,
				       interrupt);

		if (d40d->lli_current == d40d->lli_len - 1)
			d40d->last_lcla = lcla;
		/*
		 * Cache maintenance is not needed if lcla is
		 * mapped in esram
		 */
		if (!use_esram_lcla) {
			(void) dma_map_single(d40c->base->dev, lcla,
					      2 * sizeof(struct d40_log_lli),
					      DMA_TO_DEVICE);
		}
		curr_lcla = next_lcla;

		if (curr_lcla == -EINVAL || curr_lcla == first_lcla) {
			d40d->lli_current++;
			break;
		}

	}

	return first_lcla;
}

static void d40_desc_load(struct d40_chan *d40c, struct d40_desc *d40d)
{
	if (chan_is_physical(d40c)) {
		d40_phy_lli_write(d40c->base->virtbase,
				  d40c->phy_chan->num,
				  d40d->lli_phy.dst,
				  d40d->lli_phy.src);
		d40d->lli_current = d40d->lli_len;
	} else
		(void) d40_desc_log_lli_to_lcxa(d40c, d40d, true);
}

static struct d40_desc *d40_first_active_get(struct d40_chan *d40c)
{
	struct d40_desc *d;

	if (list_empty(&d40c->active))
		return NULL;

	d = list_first_entry(&d40c->active,
			     struct d40_desc,
			     node);
	return d;
}

static void d40_desc_queue(struct d40_chan *d40c, struct d40_desc *desc)
{
	list_add_tail(&desc->node, &d40c->queue);
}

static struct d40_desc *d40_first_queued(struct d40_chan *d40c)
{
	struct d40_desc *d;

	if (list_empty(&d40c->queue))
		return NULL;

	d = list_first_entry(&d40c->queue,
			     struct d40_desc,
			     node);
	return d;
}

static struct d40_desc *d40_first_done(struct d40_chan *d40c)
{
	if (list_empty(&d40c->done))
		return NULL;

	return list_first_entry(&d40c->done, struct d40_desc, node);
}


#ifdef CONFIG_PM
static void d40_save_restore_registers(struct d40_base *base, bool save)
{
	int i;
	int j;
	int idx;
	void __iomem *addr;

	/* Enable all clocks -- revisit after HW bug is fixed */
	if (!save)
		writel(D40_DREG_GCC_ENABLE_ALL, base->virtbase + D40_DREG_GCC);

	/* Save/Restore channel specific registers */
	for (i = 0; i < base->num_phy_chans; i++) {
		if (base->phy_res[i].reserved)
			continue;

		addr = base->virtbase + D40_DREG_PCBASE + i * D40_DREG_PCDELTA;
		idx = i * ARRAY_SIZE(d40_backup_regs_chan);

		for (j = 0; j < ARRAY_SIZE(d40_backup_regs_chan); j++) {
			if (save)
				base->reg_val_backup_chan[idx + j] =
					readl(addr + d40_backup_regs_chan[j]);
			else
				writel(base->reg_val_backup_chan[idx + j],
				       addr + d40_backup_regs_chan[j]);
		}
	}

	/* Save/Restore global registers */
	for (i = 0 ; i < ARRAY_SIZE(d40_backup_regs); i++) {
		addr = base->virtbase + d40_backup_regs[i];

		if (save)
			base->reg_val_backup[i] = readl(addr);
		else
			writel(base->reg_val_backup[i], addr);
	}

	/* Save/Restore registers only existing on dma40 v3 and later */
	if (base->rev >= 3) {
		for (i = 0 ; i < ARRAY_SIZE(d40_backup_regs_v3); i++) {
			addr = base->virtbase + d40_backup_regs_v3[i];

			if (save)
				base->reg_val_backup_v3[i] = readl(addr);
			else
				writel(base->reg_val_backup_v3[i], addr);
		}
	}
}
#else
static void d40_save_restore_registers(struct d40_base *base, bool save)
{
}
#endif

static void d40_power_off(struct d40_base *base, int phy_num)
{
	u32 gcc;
	int i;
	int j;
	int p;

	/*
	 * Disable the rest of the code because of GCC register HW bugs on v1
	 * which are not worth working around.  Revisit later.
	 */
	return;

	/*
	 * Power off event group related to physical channel, if
	 * the other physical channels that belong to the same
	 * event group are not in use
	 */

	for (j = 0; j < base->num_phy_chans; j += D40_GROUP_SIZE) {

		for (i = 0; i < 2; i++) {
			p = (((phy_num & (base->num_phy_chans - 1)) + i)
			     & (D40_GROUP_SIZE - 1)) + j;
			if (p == phy_num)
				continue;
			/*
			 * If another physical channel in the same group is
			 * allocated, just return.
			 */
			if (base->phy_res[p].allocated_dst == D40_ALLOC_PHY ||
			    base->phy_res[p].allocated_src == D40_ALLOC_PHY) {
				return;
			}
		}
	}

	/* The GCC register is protected via the usage_lock */
	gcc = readl(base->virtbase + D40_DREG_GCC);

	gcc &= ~D40_DREG_GCC_EVTGRP_ENA(D40_PHYS_TO_GROUP(phy_num),
					D40_DREG_GCC_SRC);
	gcc &= ~D40_DREG_GCC_EVTGRP_ENA(D40_PHYS_TO_GROUP(phy_num),
					D40_DREG_GCC_DST);

	writel(gcc, base->virtbase + D40_DREG_GCC);
}

static void d40_power_on(struct d40_base *base, int phy_num)
{
	u32 gcc;

	/*
	 * Disable the rest of the code because of GCC register HW bugs on v1
	 * which are not worth working around.  Revisit later.
	 */
	return;

	/* The GCC register is protected via the usage_lock */
	gcc = readl(base->virtbase + D40_DREG_GCC);

	gcc |= D40_DREG_GCC_EVTGRP_ENA(D40_PHYS_TO_GROUP(phy_num),
				       D40_DREG_GCC_SRC);
	gcc |= D40_DREG_GCC_EVTGRP_ENA(D40_PHYS_TO_GROUP(phy_num),
				       D40_DREG_GCC_DST);

	writel(gcc, base->virtbase + D40_DREG_GCC);
}

static void d40_usage_inc(struct d40_chan *d40c)
{
	unsigned long flags;

	spin_lock_irqsave(&d40c->base->usage_lock, flags);

	d40c->base->usage++;

	if (d40c->base->usage == 1)
		pm_runtime_get_sync(d40c->base->dev);

	d40_power_on(d40c->base, d40c->phy_chan->num);

	spin_unlock_irqrestore(&d40c->base->usage_lock, flags);
}

static void d40_usage_dec(struct d40_chan *d40c)
{
	unsigned long flags;

	spin_lock_irqsave(&d40c->base->usage_lock, flags);

	d40_power_off(d40c->base, d40c->phy_chan->num);

	d40c->base->usage--;

	if (d40c->base->usage == 0) {
		pm_runtime_mark_last_busy(d40c->base->dev);
		pm_runtime_put_autosuspend(d40c->base->dev);
	}

	spin_unlock_irqrestore(&d40c->base->usage_lock, flags);
}

static int __d40_execute_command_phy(struct d40_chan *d40c,
				     enum d40_command command)
{
	u32 status;
	int i;
	void __iomem *active_reg;
	int ret = 0;
	unsigned long flags;
	u32 wmask;

	if (command == D40_DMA_STOP) {
		ret = __d40_execute_command_phy(d40c, D40_DMA_SUSPEND_REQ);
		if (ret)
			return ret;
	}

	spin_lock_irqsave(&d40c->base->execmd_lock, flags);

	if (d40c->phy_chan->num % 2 == 0)
		active_reg = d40c->base->virtbase + D40_DREG_ACTIVE;
	else
		active_reg = d40c->base->virtbase + D40_DREG_ACTIVO;

	if (command == D40_DMA_SUSPEND_REQ) {
		status = (readl(active_reg) &
			  D40_CHAN_POS_MASK(d40c->phy_chan->num)) >>
			D40_CHAN_POS(d40c->phy_chan->num);

		if (status == D40_DMA_SUSPENDED || status == D40_DMA_STOP)
			goto done;
	}

#ifdef CONFIG_STE_DMA40_DEBUG
	if (command == D40_DMA_RUN)
		sted40_history_snapshot();
#endif

	wmask = 0xffffffff & ~(D40_CHAN_POS_MASK(d40c->phy_chan->num));
	writel(wmask | (command << D40_CHAN_POS(d40c->phy_chan->num)),
	       active_reg);

	if (command == D40_DMA_SUSPEND_REQ) {

		for (i = 0 ; i < D40_SUSPEND_MAX_IT; i++) {
			status = (readl(active_reg) &
				  D40_CHAN_POS_MASK(d40c->phy_chan->num)) >>
				D40_CHAN_POS(d40c->phy_chan->num);

			cpu_relax();
			/*
			 * Reduce the number of bus accesses while
			 * waiting for the DMA to suspend.
			 */
			udelay(3);

			if (status == D40_DMA_STOP ||
			    status == D40_DMA_SUSPENDED)
				break;
		}

		if (i == D40_SUSPEND_MAX_IT) {
			dev_err(&d40c->chan.dev->device,
				"[%s]: unable to suspend the chl %d (log: %d) status %x\n",
				__func__, d40c->phy_chan->num, d40c->log_num,
				status);
#ifdef CONFIG_STE_DMA40_DEBUG
			sted40_history_dump();
#endif
			dump_stack();
			ret = -EBUSY;
		}

	}
done:
	spin_unlock_irqrestore(&d40c->base->execmd_lock, flags);
	return ret;
}

static void d40_term_all(struct d40_chan *d40c)
{
	struct d40_desc *d40d;

	/* Release completed descriptors */
	while ((d40d = d40_first_done(d40c))) {
		d40_desc_remove(d40d);
		d40_desc_free(d40c, d40d);
	}

	/* Release active descriptors */
	while ((d40d = d40_first_active_get(d40c))) {
		d40_desc_remove(d40d);
		d40_desc_free(d40c, d40d);
	}

	/* Release queued descriptors waiting for transfer */
	while ((d40d = d40_first_queued(d40c))) {
		d40_desc_remove(d40d);
		d40_desc_free(d40c, d40d);
	}

	d40c->pending_tx = 0;
}

static void __d40_config_set_event(struct d40_chan *d40c, bool enable,
				   u32 event, int reg)
{
	void __iomem *addr = d40c->base->virtbase + D40_DREG_PCBASE
			     + d40c->phy_chan->num * D40_DREG_PCDELTA + reg;
	int tries;

	if (!enable) {
		writel((D40_DEACTIVATE_EVENTLINE << D40_EVENTLINE_POS(event))
		       | ~D40_EVENTLINE_MASK(event), addr);
		return;
	}

	/*
	 * The hardware sometimes doesn't register the enable when src and dst
	 * event lines are active on the same logical channel.  Retry to ensure
	 * it does.  Usually only one retry is sufficient.
	 */
	tries = 100;
	while (--tries) {
		writel((D40_ACTIVATE_EVENTLINE << D40_EVENTLINE_POS(event))
		       | ~D40_EVENTLINE_MASK(event), addr);

		if (readl(addr) & D40_EVENTLINE_MASK(event))
			break;
	}

	if (tries != 99)
		dev_dbg(chan2dev(d40c),
			"[%s] workaround enable S%cLNK (%d tries)\n",
			__func__, reg == D40_CHAN_REG_SSLNK ? 'S' : 'D',
			100 - tries);

	WARN_ON(!tries);
}

static void d40_config_set_event(struct d40_chan *d40c, bool do_enable)
{
	/* Enable event line connected to device (or memcpy) */
	if ((d40c->dma_cfg.dir ==  STEDMA40_PERIPH_TO_MEM) ||
	    (d40c->dma_cfg.dir == STEDMA40_PERIPH_TO_PERIPH)) {
		u32 event = D40_TYPE_TO_EVENT(d40c->dma_cfg.src_dev_type);

		__d40_config_set_event(d40c, do_enable, event,
				       D40_CHAN_REG_SSLNK);
	}

	if (d40c->dma_cfg.dir !=  STEDMA40_PERIPH_TO_MEM) {
		u32 event = D40_TYPE_TO_EVENT(d40c->dma_cfg.dst_dev_type);

		__d40_config_set_event(d40c, do_enable, event,
				       D40_CHAN_REG_SDLNK);
	}
}

static u32 d40_chan_has_events(struct d40_chan *d40c)
{
	u32 val;

	val = readl(d40c->base->virtbase + D40_DREG_PCBASE +
		    d40c->phy_chan->num * D40_DREG_PCDELTA +
		    D40_CHAN_REG_SSLNK);

	val |= readl(d40c->base->virtbase + D40_DREG_PCBASE +
		     d40c->phy_chan->num * D40_DREG_PCDELTA +
		     D40_CHAN_REG_SDLNK);
	return val;
}

static int
__d40_execute_command_log(struct d40_chan *d40c, enum d40_command command)
{
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&d40c->phy_chan->lock, flags);

	switch (command) {
	case D40_DMA_STOP:
	case D40_DMA_SUSPEND_REQ:
		ret = __d40_execute_command_phy(d40c, D40_DMA_SUSPEND_REQ);
		if (ret)
			goto out;

		d40_config_set_event(d40c, false);

		if (d40_chan_has_events(d40c))
			ret = __d40_execute_command_phy(d40c, D40_DMA_RUN);
		else if (command == D40_DMA_STOP)
			ret = __d40_execute_command_phy(d40c, command);
		break;

	case D40_DMA_RUN:
		if (d40c->base->rev == 0) {
			ret = __d40_execute_command_phy(d40c,
							D40_DMA_SUSPEND_REQ);
			if (ret)
				goto out;
		}

		d40_config_set_event(d40c, true);

		ret = __d40_execute_command_phy(d40c, command);
		break;

	case D40_DMA_SUSPENDED:
		BUG();
		break;
	}

out:
	spin_unlock_irqrestore(&d40c->phy_chan->lock, flags);
	return ret;
}

static int d40_channel_execute_command(struct d40_chan *d40c,
				       enum d40_command command)
{
	if (chan_is_logical(d40c))
		return __d40_execute_command_log(d40c, command);
	else
		return __d40_execute_command_phy(d40c, command);
}

static u32 d40_get_prmo(struct d40_chan *d40c)
{
	static const unsigned int phy_map[] = {
		[STEDMA40_PCHAN_BASIC_MODE]
			= D40_DREG_PRMO_PCHAN_BASIC,
		[STEDMA40_PCHAN_MODULO_MODE]
			= D40_DREG_PRMO_PCHAN_MODULO,
		[STEDMA40_PCHAN_DOUBLE_DST_MODE]
			= D40_DREG_PRMO_PCHAN_DOUBLE_DST,
	};
	static const unsigned int log_map[] = {
		[STEDMA40_LCHAN_SRC_PHY_DST_LOG]
			= D40_DREG_PRMO_LCHAN_SRC_PHY_DST_LOG,
		[STEDMA40_LCHAN_SRC_LOG_DST_PHY]
			= D40_DREG_PRMO_LCHAN_SRC_LOG_DST_PHY,
		[STEDMA40_LCHAN_SRC_LOG_DST_LOG]
			= D40_DREG_PRMO_LCHAN_SRC_LOG_DST_LOG,
	};

	if (chan_is_physical(d40c))
		return phy_map[d40c->dma_cfg.mode_opt];
	else
		return log_map[d40c->dma_cfg.mode_opt];
}

static void d40_config_write(struct d40_chan *d40c)
{
	u32 addr_base;
	u32 var;

	/* Odd addresses are even addresses + 4 */
	addr_base = (d40c->phy_chan->num % 2) * 4;
	/* Setup channel mode to logical or physical */
	var = ((u32)(chan_is_logical(d40c)) + 1) <<
		D40_CHAN_POS(d40c->phy_chan->num);
	writel(var, d40c->base->virtbase + D40_DREG_PRMSE + addr_base);

	/* Setup operational mode option register */
	var = d40_get_prmo(d40c) << D40_CHAN_POS(d40c->phy_chan->num);

	writel(var, d40c->base->virtbase + D40_DREG_PRMOE + addr_base);

	if (chan_is_logical(d40c)) {
		/* Set default config for CFG reg */
		writel(d40c->src_def_cfg,
		       d40c->base->virtbase + D40_DREG_PCBASE +
		       d40c->phy_chan->num * D40_DREG_PCDELTA +
		       D40_CHAN_REG_SSCFG);
		writel(d40c->dst_def_cfg,
		       d40c->base->virtbase + D40_DREG_PCBASE +
		       d40c->phy_chan->num * D40_DREG_PCDELTA +
		       D40_CHAN_REG_SDCFG);

		/* Set LIDX for lcla */
		writel((d40c->phy_chan->num << D40_SREG_ELEM_LOG_LIDX_POS) &
		       D40_SREG_ELEM_LOG_LIDX_MASK,
		       d40c->base->virtbase + D40_DREG_PCBASE +
		       d40c->phy_chan->num * D40_DREG_PCDELTA +
		       D40_CHAN_REG_SDELT);

		writel((d40c->phy_chan->num << D40_SREG_ELEM_LOG_LIDX_POS) &
		       D40_SREG_ELEM_LOG_LIDX_MASK,
		       d40c->base->virtbase + D40_DREG_PCBASE +
		       d40c->phy_chan->num * D40_DREG_PCDELTA +
		       D40_CHAN_REG_SSELT);

		/* Clear LNK which will be used by d40_chan_has_events() */
		writel(0,
		       d40c->base->virtbase + D40_DREG_PCBASE +
		       d40c->phy_chan->num * D40_DREG_PCDELTA +
		       D40_CHAN_REG_SSLNK);
		writel(0,
		       d40c->base->virtbase + D40_DREG_PCBASE +
		       d40c->phy_chan->num * D40_DREG_PCDELTA +
		       D40_CHAN_REG_SDLNK);
	}
}

static u32 d40_residue(struct d40_chan *d40c)
{
	u32 num_elt;

	if (chan_is_logical(d40c))
		num_elt = (readl(&d40c->lcpa->lcsp2) &
			   D40_MEM_LCSP2_ECNT_MASK) >> D40_MEM_LCSP2_ECNT_POS;
	else
		num_elt = (readl(d40c->base->virtbase + D40_DREG_PCBASE +
				 d40c->phy_chan->num * D40_DREG_PCDELTA +
				 D40_CHAN_REG_SDELT) &
			   D40_SREG_ELEM_PHY_ECNT_MASK) >>
			D40_SREG_ELEM_PHY_ECNT_POS;
	return num_elt * (1 << d40c->dma_cfg.dst_info.data_width);
}

static bool d40_tx_is_linked(struct d40_chan *d40c)
{
	bool is_link;

	if (chan_is_logical(d40c))
		is_link = readl(&d40c->lcpa->lcsp3) &  D40_MEM_LCSP3_DLOS_MASK;
	else
		is_link = readl(d40c->base->virtbase + D40_DREG_PCBASE +
				d40c->phy_chan->num * D40_DREG_PCDELTA +
				D40_CHAN_REG_SDLNK) &
			D40_SREG_LNK_PHYS_LNK_MASK;
	return is_link;
}

static int d40_pause(struct dma_chan *chan)
{
	struct d40_chan *d40c =
		container_of(chan, struct d40_chan, chan);
	int res = 0;
	unsigned long flags;

	if (!d40c->busy)
		return 0;

	d40_usage_inc(d40c);
	spin_lock_irqsave(&d40c->lock, flags);

	res = d40_channel_execute_command(d40c, D40_DMA_SUSPEND_REQ);

	d40_usage_dec(d40c);

	spin_unlock_irqrestore(&d40c->lock, flags);
	return res;
}

static int d40_resume(struct dma_chan *chan)
{
	struct d40_chan *d40c =
		container_of(chan, struct d40_chan, chan);
	int res = 0;
	unsigned long flags;

	if (!d40c->busy)
		return 0;

	spin_lock_irqsave(&d40c->lock, flags);

	d40_usage_inc(d40c);

	/* If bytes left to transfer or linked tx resume job */
	if (d40_residue(d40c) || d40_tx_is_linked(d40c))
		res = d40_channel_execute_command(d40c, D40_DMA_RUN);

	d40_usage_dec(d40c);
	spin_unlock_irqrestore(&d40c->lock, flags);
	return res;
}

static dma_cookie_t d40_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct d40_chan *d40c = container_of(tx->chan,
					     struct d40_chan,
					     chan);
	struct d40_desc *d40d = container_of(tx, struct d40_desc, txd);
	unsigned long flags;

	spin_lock_irqsave(&d40c->lock, flags);

	d40c->chan.cookie++;

	if (d40c->chan.cookie < 0)
		d40c->chan.cookie = 1;

	d40d->txd.cookie = d40c->chan.cookie;

	d40_desc_queue(d40c, d40d);

	spin_unlock_irqrestore(&d40c->lock, flags);

	return tx->cookie;
}

static int d40_start(struct d40_chan *d40c)
{
	return d40_channel_execute_command(d40c, D40_DMA_RUN);
}

static struct d40_desc *d40_queue_start(struct d40_chan *d40c)
{
	struct d40_desc *d40d;
	int err;

	/* Start queued jobs, if any */
	d40d = d40_first_queued(d40c);

	if (d40d != NULL) {
		if (!d40c->busy) {
			d40_usage_inc(d40c);
			d40c->busy = true;
		}

		/* Remove from queue */
		d40_desc_remove(d40d);

		/* Add to active queue */
		d40_desc_submit(d40c, d40d);


		/* Initiate DMA job */
		d40_desc_load(d40c, d40d);

		/* Start dma job */
		err = d40_start(d40c);

		if (err)
			return NULL;
	}
	return d40d;
}

/* called from interrupt context */
static void dma_tc_handle(struct d40_chan *d40c)
{
	struct d40_desc *d40d;
	bool islastactive;

	if (d40c->cdesc) {
		d40c->pending_tx++;
		tasklet_schedule(&d40c->tasklet);
		return;
	}

	/* Get first active entry from list */
redo:
	d40d = d40_first_active_get(d40c);

	if (d40d == NULL)
		return;

	d40_lcla_free_all(d40c, d40d);

	if (d40d->lli_current < d40d->lli_len) {
		d40_desc_load(d40c, d40d);
		/* Start dma job */
		(void) d40_start(d40c);
		return;
	}

	/*
	 * More than one active happens when we have
	 * hw linked transfers.
	 */
	islastactive = list_is_last(&d40d->node, &d40c->active);
	if (islastactive && d40_queue_start(d40c) == NULL) {
		d40c->busy = false;
		d40_usage_dec(d40c);
	}

	d40_desc_remove(d40d);
	d40_desc_done(d40c, d40d);

	d40c->pending_tx++;
	tasklet_schedule(&d40c->tasklet);

	/*
	 * When we have multiple active transfers, there is a chance that we
	 * might miss some link interrupts if the time to perform each link is
	 * very small (mostly with mem-to-mem transfers).  So, if the hardware
	 * is not transmitting any more links, assume that all the active
	 * transfers are complete.
	 */
	if (!islastactive && !d40_tx_is_linked(d40c))
		goto redo;
}

static void dma_tasklet(unsigned long data)
{
	struct d40_chan *d40c = (struct d40_chan *) data;
	struct d40_desc *d40d;
	unsigned long flags;
	dma_async_tx_callback callback;
	void *callback_param;

	spin_lock_irqsave(&d40c->lock, flags);

	if (d40c->cdesc)
		d40d = d40c->cdesc->d40d;
	else {
		/* Get first active entry from list */
		d40d = d40_first_done(d40c);
		if (d40d == NULL)
			goto err;

		d40c->completed = d40d->txd.cookie;
	}

	/*
	 * If terminating a channel pending_tx is set to zero.
	 * This prevents any finished active jobs to return to the client.
	 */
	if (d40c->pending_tx == 0) {
		spin_unlock_irqrestore(&d40c->lock, flags);
		return;
	}

	/* Callback to client */

	if (d40c->cdesc) {
		callback = d40c->cdesc->period_callback;
		callback_param = d40c->cdesc->period_callback_param;
	} else {
		callback = d40d->txd.callback;
		callback_param = d40d->txd.callback_param;

		if (async_tx_test_ack(&d40d->txd)) {
			d40_pool_lli_free(d40d);
			d40_desc_remove(d40d);
			d40_desc_free(d40c, d40d);
		} else if (!d40d->is_in_client_list) {
			d40_desc_remove(d40d);
			d40_lcla_free_all(d40c, d40d);
			list_add_tail(&d40d->node, &d40c->client);
			d40d->is_in_client_list = true;
		}
	}

	d40c->pending_tx--;

	if (d40c->pending_tx)
		tasklet_schedule(&d40c->tasklet);

	spin_unlock_irqrestore(&d40c->lock, flags);

	if (callback && (d40d->txd.flags & DMA_PREP_INTERRUPT))
		callback(callback_param);

	return;

err:
	/* Rescue manouver if receiving double interrupts */
	if (d40c->pending_tx > 0)
		d40c->pending_tx--;
	spin_unlock_irqrestore(&d40c->lock, flags);
}

static irqreturn_t d40_handle_interrupt(int irq, void *data)
{
	static struct d40_interrupt_lookup il[] = {
		{D40_DREG_LCTIS0, D40_DREG_LCICR0, false,  0},
		{D40_DREG_LCTIS1, D40_DREG_LCICR1, false, 32},
		{D40_DREG_LCTIS2, D40_DREG_LCICR2, false, 64},
		{D40_DREG_LCTIS3, D40_DREG_LCICR3, false, 96},
		{D40_DREG_LCEIS0, D40_DREG_LCICR0, true,   0},
		{D40_DREG_LCEIS1, D40_DREG_LCICR1, true,  32},
		{D40_DREG_LCEIS2, D40_DREG_LCICR2, true,  64},
		{D40_DREG_LCEIS3, D40_DREG_LCICR3, true,  96},
		{D40_DREG_PCTIS,  D40_DREG_PCICR,  false, D40_PHY_CHAN},
		{D40_DREG_PCEIS,  D40_DREG_PCICR,  true,  D40_PHY_CHAN},
	};

	int i;
	u32 regs[ARRAY_SIZE(il)];
	u32 idx;
	u32 row;
	long chan = -1;
	struct d40_chan *d40c;
	unsigned long flags;
	struct d40_base *base = data;

	spin_lock_irqsave(&base->interrupt_lock, flags);
#ifdef CONFIG_STE_DMA40_DEBUG
	sted40_history_text("IRQ enter");
#endif
	/* Read interrupt status of both logical and physical channels */
	for (i = 0; i < ARRAY_SIZE(il); i++)
		regs[i] = readl(base->virtbase + il[i].src);

	for (;;) {

		chan = find_next_bit((unsigned long *)regs,
				     BITS_PER_LONG * ARRAY_SIZE(il), chan + 1);

		/* No more set bits found? */
		if (chan == BITS_PER_LONG * ARRAY_SIZE(il))
			break;

		row = chan / BITS_PER_LONG;
		idx = chan & (BITS_PER_LONG - 1);

		if (il[row].offset == D40_PHY_CHAN)
			d40c = base->lookup_phy_chans[idx];
		else
			d40c = base->lookup_log_chans[il[row].offset + idx];

		if (!d40c) {
			/*
			 * No error because this can happen if something else
			 * in the system is using the channel.
			 */
			continue;
		}

		/* ACK interrupt */
		writel(1 << idx, base->virtbase + il[row].clr);

		spin_lock(&d40c->lock);

		if (!il[row].is_error) {

			dma_tc_handle(d40c);
		} else {
			dev_err(base->dev, "[%s] Error IRQ chan: %ld offset %d idx %d\n",
				__func__, chan, il[row].offset, idx);
#ifdef CONFIG_STE_DMA40_DEBUG
			sted40_history_dump();
#endif
		}

		spin_unlock(&d40c->lock);
	}
#ifdef CONFIG_STE_DMA40_DEBUG
	sted40_history_text("IRQ leave");
#endif
	spin_unlock_irqrestore(&base->interrupt_lock, flags);

	return IRQ_HANDLED;
}

static int d40_validate_conf(struct d40_chan *d40c,
			     struct stedma40_chan_cfg *conf)
{
	int res = 0;
	u32 dst_event_group = D40_TYPE_TO_GROUP(conf->dst_dev_type);
	u32 src_event_group = D40_TYPE_TO_GROUP(conf->src_dev_type);
	bool is_log = conf->mode == STEDMA40_MODE_LOGICAL;

	if (!conf->dir) {
		dev_err(&d40c->chan.dev->device, "[%s] Invalid direction.\n",
			__func__);
		res = -EINVAL;
	}

	if (conf->dst_dev_type != STEDMA40_DEV_DST_MEMORY &&
	   d40c->base->plat_data->dev_tx[conf->dst_dev_type] == 0) {
		dev_err(&d40c->chan.dev->device,
			"[%s] Invalid TX channel address (%d)\n",
			__func__, conf->dst_dev_type);
		res = -EINVAL;
	}

	if (conf->src_dev_type != STEDMA40_DEV_SRC_MEMORY &&
	   d40c->base->plat_data->dev_rx[conf->src_dev_type] == 0) {
		dev_err(&d40c->chan.dev->device,
			"[%s] Invalid RX channel address (%d)\n",
			__func__, conf->src_dev_type);
		res = -EINVAL;
	}

	if (conf->dir == STEDMA40_MEM_TO_PERIPH &&
	    dst_event_group == STEDMA40_DEV_DST_MEMORY) {
		dev_err(&d40c->chan.dev->device, "[%s] Invalid dst\n",
			__func__);
		res = -EINVAL;
	}

	if (conf->dir == STEDMA40_PERIPH_TO_MEM &&
	    src_event_group == STEDMA40_DEV_SRC_MEMORY) {
		dev_err(&d40c->chan.dev->device, "[%s] Invalid src\n",
			__func__);
		res = -EINVAL;
	}

	if (src_event_group == STEDMA40_DEV_SRC_MEMORY &&
	    dst_event_group == STEDMA40_DEV_DST_MEMORY && is_log) {
		dev_err(&d40c->chan.dev->device,
			"[%s] No event line\n", __func__);
		res = -EINVAL;
	}

	if (conf->dir == STEDMA40_PERIPH_TO_PERIPH &&
	    (src_event_group != dst_event_group)) {
		dev_err(&d40c->chan.dev->device,
			"[%s] Invalid event group\n", __func__);
		res = -EINVAL;
	}

	if (conf->dir == STEDMA40_PERIPH_TO_PERIPH) {
		/*
		 * DMAC HW supports it. Will be added to this driver,
		 * in case any dma client requires it.
		 */
		dev_err(&d40c->chan.dev->device,
			"[%s] periph to periph not supported\n",
			__func__);
		res = -EINVAL;
	}

	return res;
}

static bool d40_alloc_mask_set(struct d40_phy_res *phy,
			       bool is_src, int log_event_line, bool is_log,
			       bool *first_user)
{
	unsigned long flags;
	spin_lock_irqsave(&phy->lock, flags);

	*first_user = ((phy->allocated_src | phy->allocated_dst)
			== D40_ALLOC_FREE);

	if (!is_log) {
		/* Physical interrupts are masked per physical full channel */
		if (phy->allocated_src == D40_ALLOC_FREE &&
		    phy->allocated_dst == D40_ALLOC_FREE) {
			phy->allocated_dst = D40_ALLOC_PHY;
			phy->allocated_src = D40_ALLOC_PHY;
			goto found;
		} else
			goto not_found;
	}

	/* Logical channel */
	if (is_src) {
		if (phy->allocated_src == D40_ALLOC_PHY)
			goto not_found;

		if (phy->allocated_src == D40_ALLOC_FREE)
			phy->allocated_src = D40_ALLOC_LOG_FREE;

		if (!(phy->allocated_src & (1 << log_event_line))) {
			phy->allocated_src |= 1 << log_event_line;
			goto found;
		} else
			goto not_found;
	} else {
		if (phy->allocated_dst == D40_ALLOC_PHY)
			goto not_found;

		if (phy->allocated_dst == D40_ALLOC_FREE)
			phy->allocated_dst = D40_ALLOC_LOG_FREE;

		if (!(phy->allocated_dst & (1 << log_event_line))) {
			phy->allocated_dst |= 1 << log_event_line;
			goto found;
		} else
			goto not_found;
	}

not_found:
	spin_unlock_irqrestore(&phy->lock, flags);
	return false;
found:
	spin_unlock_irqrestore(&phy->lock, flags);
	return true;
}

static bool d40_alloc_mask_free(struct d40_phy_res *phy, bool is_src,
			       int log_event_line)
{
	unsigned long flags;
	bool is_free = false;

	spin_lock_irqsave(&phy->lock, flags);
	if (!log_event_line) {
		phy->allocated_dst = D40_ALLOC_FREE;
		phy->allocated_src = D40_ALLOC_FREE;
		is_free = true;
		goto out;
	}

	/* Logical channel */
	if (is_src) {
		phy->allocated_src &= ~(1 << log_event_line);
		if (phy->allocated_src == D40_ALLOC_LOG_FREE)
			phy->allocated_src = D40_ALLOC_FREE;
	} else {
		phy->allocated_dst &= ~(1 << log_event_line);
		if (phy->allocated_dst == D40_ALLOC_LOG_FREE)
			phy->allocated_dst = D40_ALLOC_FREE;
	}

	is_free = ((phy->allocated_src | phy->allocated_dst) ==
		   D40_ALLOC_FREE);

out:
	spin_unlock_irqrestore(&phy->lock, flags);

	return is_free;
}

static int d40_allocate_channel(struct d40_chan *d40c, bool *first_phy_user)
{
	int dev_type;
	int event_group;
	int event_line;
	struct d40_phy_res *phys;
	int i;
	int j;
	int log_num;
	bool is_src;
	bool is_log = d40c->dma_cfg.mode == STEDMA40_MODE_LOGICAL;

	phys = d40c->base->phy_res;

	if (d40c->dma_cfg.dir == STEDMA40_PERIPH_TO_MEM) {
		dev_type = d40c->dma_cfg.src_dev_type;
		log_num = 2 * dev_type;
		is_src = true;
	} else if (d40c->dma_cfg.dir == STEDMA40_MEM_TO_PERIPH ||
		   d40c->dma_cfg.dir == STEDMA40_MEM_TO_MEM) {
		/* dst event lines are used for logical memcpy */
		dev_type = d40c->dma_cfg.dst_dev_type;
		log_num = 2 * dev_type + 1;
		is_src = false;
	} else
		return -EINVAL;

	event_group = D40_TYPE_TO_GROUP(dev_type);
	event_line = D40_TYPE_TO_EVENT(dev_type);

	if (!is_log) {
		if (d40c->dma_cfg.dir == STEDMA40_MEM_TO_MEM) {
			/* Find physical half channel */
			for (i = 0; i < d40c->base->num_phy_chans; i++) {

				if (d40_alloc_mask_set(&phys[i], is_src,
						       0, is_log,
						       first_phy_user))
					goto found_phy;
			}
		} else
			for (j = 0; j < d40c->base->num_phy_chans; j += 8) {
				int phy_num = j  + event_group * 2;
				for (i = phy_num; i < phy_num + 2; i++) {
					if (d40_alloc_mask_set(&phys[i],
							       is_src, 0,
							       is_log,
							       first_phy_user))
						goto found_phy;
				}
			}
		return -EINVAL;
found_phy:
		d40c->phy_chan = &phys[i];
		d40c->log_num = D40_PHY_CHAN;
		goto out;
	}
	if (dev_type == -1)
		return -EINVAL;

	/* Find logical channel */
	for (j = 0; j < d40c->base->num_phy_chans; j += 8) {
		int phy_num = j + event_group * 2;

		if (d40c->dma_cfg.use_fixed_channel) {
			i = d40c->dma_cfg.phy_channel;

			if ((i != phy_num) && (i != phy_num + 1)) {
				dev_err(chan2dev(d40c),
					"invalid fixed phy channel %d\n", i);
				return -EINVAL;
			}

			if (d40_alloc_mask_set(&phys[i], is_src, event_line, is_log,
					       first_phy_user))
				goto found_log;

			dev_err(chan2dev(d40c),
				"could not allocated fixed phy channel %d\n", i);
			return -EINVAL;
		}

		/*
		 * Spread logical channels across all available physical rather
		 * than pack every logical channel at the first available phy
		 * channels.
		 */
		if (is_src) {
			for (i = phy_num; i < phy_num + 2; i++) {
				if (d40_alloc_mask_set(&phys[i], is_src,
						       event_line, is_log,
						       first_phy_user))
					goto found_log;
			}
		} else {
			for (i = phy_num + 1; i >= phy_num; i--) {
				if (d40_alloc_mask_set(&phys[i], is_src,
						       event_line, is_log,
						       first_phy_user))
					goto found_log;
			}
		}
	}
	return -EINVAL;

found_log:
	d40c->phy_chan = &phys[i];
	d40c->log_num = log_num;
out:

	if (is_log)
		d40c->base->lookup_log_chans[d40c->log_num] = d40c;
	else
		d40c->base->lookup_phy_chans[d40c->phy_chan->num] = d40c;

	return 0;

}

static int d40_config_memcpy(struct d40_chan *d40c)
{
	dma_cap_mask_t cap = d40c->chan.device->cap_mask;

	if (dma_has_cap(DMA_MEMCPY, cap) && !dma_has_cap(DMA_SLAVE, cap)) {
		d40c->dma_cfg = *d40c->base->plat_data->memcpy_conf_log;
		d40c->dma_cfg.src_dev_type = STEDMA40_DEV_SRC_MEMORY;
		d40c->dma_cfg.dst_dev_type = d40c->base->plat_data->
			memcpy[d40c->chan.chan_id];

	} else if (dma_has_cap(DMA_MEMCPY, cap) &&
		   dma_has_cap(DMA_SLAVE, cap)) {
		d40c->dma_cfg = *d40c->base->plat_data->memcpy_conf_phy;
	} else {
		dev_err(&d40c->chan.dev->device, "[%s] No memcpy\n",
			__func__);
		return -EINVAL;
	}

	return 0;
}

static int d40_free_dma(struct d40_chan *d40c)
{

	int res = 0;
	u32 event;
	struct d40_phy_res *phy = d40c->phy_chan;
	bool is_src;
	struct d40_desc *d;
	struct d40_desc *_d;

	/* Terminate all queued and active transfers */
	d40_term_all(d40c);

	/* Release client owned descriptors */
	if (!list_empty(&d40c->client))
		list_for_each_entry_safe(d, _d, &d40c->client, node) {
			d40_pool_lli_free(d);
			d40_desc_remove(d);
			d40_desc_free(d40c, d);
		}

	if (phy == NULL) {
		dev_err(&d40c->chan.dev->device, "[%s] phy == null\n",
			__func__);
		return -EINVAL;
	}

	if (phy->allocated_src == D40_ALLOC_FREE &&
	    phy->allocated_dst == D40_ALLOC_FREE) {
		dev_err(&d40c->chan.dev->device, "[%s] channel already free\n",
			__func__);
		return -EINVAL;
	}

	if (d40c->dma_cfg.dir == STEDMA40_MEM_TO_PERIPH ||
	    d40c->dma_cfg.dir == STEDMA40_MEM_TO_MEM) {
		event = D40_TYPE_TO_EVENT(d40c->dma_cfg.dst_dev_type);
		is_src = false;
	} else if (d40c->dma_cfg.dir == STEDMA40_PERIPH_TO_MEM) {
		event = D40_TYPE_TO_EVENT(d40c->dma_cfg.src_dev_type);
		is_src = true;
	} else {
		dev_err(&d40c->chan.dev->device,
			"[%s] Unknown direction\n", __func__);
		return -EINVAL;
	}

	d40_usage_inc(d40c);

	res = d40_channel_execute_command(d40c, D40_DMA_STOP);
	if (res) {
		dev_err(&d40c->chan.dev->device, "[%s] stop failed\n",
			__func__);
		d40_usage_dec(d40c);
		return res;
	}

	d40_alloc_mask_free(phy, is_src, chan_is_logical(d40c) ? event : 0);

	if (chan_is_logical(d40c))
		d40c->base->lookup_log_chans[d40c->log_num] = NULL;
	else
		d40c->base->lookup_phy_chans[phy->num] = NULL;

	d40_usage_dec(d40c);
	if (d40c->busy)
		d40_usage_dec(d40c);
	d40c->busy = false;

	d40c->phy_chan = NULL;
	d40c->configured = false;

	return 0;
}

#ifdef MMC_HOST_DEBUGGING
struct d40_base *_base;
static void dump_channels(void);
static bool _kickrx, _kicktx;

void kickrx(void)
{
	_kickrx = true;
}

void kicktx(void)
{
	_kicktx = true;
}

void dump(void)
{
	static struct d40_interrupt_lookup il[] = {
		{D40_DREG_LCTIS0, D40_DREG_LCICR0, false,  0},
		{D40_DREG_LCTIS1, D40_DREG_LCICR1, false, 32},
		{D40_DREG_LCTIS2, D40_DREG_LCICR2, false, 64},
		{D40_DREG_LCTIS3, D40_DREG_LCICR3, false, 96},
		{D40_DREG_LCEIS0, D40_DREG_LCICR0, true,   0},
		{D40_DREG_LCEIS1, D40_DREG_LCICR1, true,  32},
		{D40_DREG_LCEIS2, D40_DREG_LCICR2, true,  64},
		{D40_DREG_LCEIS3, D40_DREG_LCICR3, true,  96},
		{D40_DREG_PCTIS,  D40_DREG_PCICR,  false, D40_PHY_CHAN},
		{D40_DREG_PCEIS,  D40_DREG_PCICR,  true,  D40_PHY_CHAN},
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(il); i++)
		printk("i %d: %#x\n", i, readl(_base->virtbase + il[i].src));

	printk("ACTIVE: %#x\n", readl(_base->virtbase + D40_DREG_ACTIVO));
	printk("ACTIVO: %#x\n", readl(_base->virtbase + D40_DREG_ACTIVO));

	i = CHANNEL_OF_CHOICE;
	printk("Channel %d\n", i);
	printk("SSCFG: %#x\n", readl(_base->virtbase + D40_DREG_PCBASE + i * D40_DREG_PCDELTA + D40_CHAN_REG_SSCFG));
	printk("SSELT: %#x\n", readl(_base->virtbase + D40_DREG_PCBASE + i * D40_DREG_PCDELTA + D40_CHAN_REG_SSELT));
	printk("SSPTR: %#x\n", readl(_base->virtbase + D40_DREG_PCBASE + i * D40_DREG_PCDELTA + D40_CHAN_REG_SSPTR));
	printk("SSLNK: %#x\n", readl(_base->virtbase + D40_DREG_PCBASE + i * D40_DREG_PCDELTA + D40_CHAN_REG_SSLNK));

	printk("SDCFG: %#x\n", readl(_base->virtbase + D40_DREG_PCBASE + i * D40_DREG_PCDELTA + D40_CHAN_REG_SDCFG));
	printk("SDELT: %#x\n", readl(_base->virtbase + D40_DREG_PCBASE + i * D40_DREG_PCDELTA + D40_CHAN_REG_SDELT));
	printk("SDPTR: %#x\n", readl(_base->virtbase + D40_DREG_PCBASE + i * D40_DREG_PCDELTA + D40_CHAN_REG_SDPTR));
	printk("SDLNK: %#x\n", readl(_base->virtbase + D40_DREG_PCBASE + i * D40_DREG_PCDELTA + D40_CHAN_REG_SDLNK));

	dump_channels();

	print_hex_dump(KERN_INFO, "lcla channel " __stringify(CHANNEL_OF_CHOICE)  ": ", DUMP_PREFIX_OFFSET, 16, 4, _base->lcla_pool.base + SZ_1K * CHANNEL_OF_CHOICE, SZ_1K, true);
}
#endif

static bool d40_is_paused(struct d40_chan *d40c)
{
	bool is_paused = false;
	unsigned long flags;
	void __iomem *active_reg;
	u32 status;
	u32 event;

	spin_lock_irqsave(&d40c->lock, flags);

	if (chan_is_physical(d40c)) {
		if (d40c->phy_chan->num % 2 == 0)
			active_reg = d40c->base->virtbase + D40_DREG_ACTIVE;
		else
			active_reg = d40c->base->virtbase + D40_DREG_ACTIVO;

		status = (readl(active_reg) &
			  D40_CHAN_POS_MASK(d40c->phy_chan->num)) >>
			D40_CHAN_POS(d40c->phy_chan->num);
		if (status == D40_DMA_SUSPENDED || status == D40_DMA_STOP)
			is_paused = true;

		goto _exit;
	}

	if (d40c->dma_cfg.dir == STEDMA40_MEM_TO_PERIPH ||
	    d40c->dma_cfg.dir == STEDMA40_MEM_TO_MEM) {
		event = D40_TYPE_TO_EVENT(d40c->dma_cfg.dst_dev_type);
		status = readl(d40c->base->virtbase + D40_DREG_PCBASE +
			       d40c->phy_chan->num * D40_DREG_PCDELTA +
			       D40_CHAN_REG_SDLNK);
	} else if (d40c->dma_cfg.dir == STEDMA40_PERIPH_TO_MEM) {
		event = D40_TYPE_TO_EVENT(d40c->dma_cfg.src_dev_type);
		status = readl(d40c->base->virtbase + D40_DREG_PCBASE +
			       d40c->phy_chan->num * D40_DREG_PCDELTA +
			       D40_CHAN_REG_SSLNK);
	} else {
		dev_err(&d40c->chan.dev->device,
			"[%s] Unknown direction\n", __func__);
		goto _exit;
	}

	status = (status & D40_EVENTLINE_MASK(event)) >>
		D40_EVENTLINE_POS(event);

	if (status != D40_DMA_RUN)
		is_paused = true;
_exit:
	spin_unlock_irqrestore(&d40c->lock, flags);
	return is_paused;

}

struct dma_async_tx_descriptor *stedma40_memcpy_sg(struct dma_chan *chan,
						   struct scatterlist *sgl_dst,
						   struct scatterlist *sgl_src,
						   unsigned int sgl_len,
						   unsigned long dma_flags)
{
	int res;
	struct d40_desc *d40d;
	struct d40_chan *d40c = container_of(chan, struct d40_chan,
					     chan);
	unsigned long flags;

	if (d40c->phy_chan == NULL) {
		dev_err(&d40c->chan.dev->device,
			"[%s] Unallocated channel.\n", __func__);
		return ERR_PTR(-EINVAL);
	}

	spin_lock_irqsave(&d40c->lock, flags);
	d40d = d40_desc_get(d40c);

	if (d40d == NULL)
		goto err;

	d40d->lli_len = sgl_len;
	d40d->lli_current = 0;
	d40d->txd.flags = dma_flags;

	if (chan_is_logical(d40c)) {

		if (d40_pool_lli_alloc(d40d, sgl_len, true) < 0) {
			dev_err(&d40c->chan.dev->device,
				"[%s] Out of memory\n", __func__);
			goto err;
		}

		(void) d40_log_sg_to_lli(sgl_src,
					 sgl_len,
					 d40d->lli_log.src,
					 d40c->log_def.lcsp1,
					 d40c->dma_cfg.src_info.data_width);

		(void) d40_log_sg_to_lli(sgl_dst,
					 sgl_len,
					 d40d->lli_log.dst,
					 d40c->log_def.lcsp3,
					 d40c->dma_cfg.dst_info.data_width);
	} else {
		if (d40_pool_lli_alloc(d40d, sgl_len, false) < 0) {
			dev_err(&d40c->chan.dev->device,
				"[%s] Out of memory\n", __func__);
			goto err;
		}

		res = d40_phy_sg_to_lli(sgl_src,
					sgl_len,
					0,
					d40d->lli_phy.src,
					virt_to_phys(d40d->lli_phy.src),
					d40c->src_def_cfg,
					d40c->dma_cfg.src_info.data_width,
					d40c->dma_cfg.src_info.psize,
					false,
					false);

		if (res < 0)
			goto err;

		res = d40_phy_sg_to_lli(sgl_dst,
					sgl_len,
					0,
					d40d->lli_phy.dst,
					virt_to_phys(d40d->lli_phy.dst),
					d40c->dst_def_cfg,
					d40c->dma_cfg.dst_info.data_width,
					d40c->dma_cfg.dst_info.psize,
					false,
					false);

		if (res < 0)
			goto err;

		(void) dma_map_single(d40c->base->dev, d40d->lli_phy.src,
				      d40d->lli_pool.size, DMA_TO_DEVICE);

	}

	dma_async_tx_descriptor_init(&d40d->txd, chan);

	d40d->txd.tx_submit = d40_tx_submit;

	spin_unlock_irqrestore(&d40c->lock, flags);

	return &d40d->txd;
err:
	if (d40d)
		d40_desc_free(d40c, d40d);
	spin_unlock_irqrestore(&d40c->lock, flags);
	return NULL;
}
EXPORT_SYMBOL(stedma40_memcpy_sg);

bool stedma40_filter(struct dma_chan *chan, void *data)
{
	struct stedma40_chan_cfg *info = data;
	struct d40_chan *d40c =
		container_of(chan, struct d40_chan, chan);
	int err;

	if (data) {
		err = d40_validate_conf(d40c, info);
		if (!err)
			d40c->dma_cfg = *info;
	} else
		err = d40_config_memcpy(d40c);

	if (!err)
		d40c->configured = true;

	return err == 0;
}
EXPORT_SYMBOL(stedma40_filter);

static void __d40_set_prio_rt(struct d40_chan *d40c, int dev_type, bool src)
{
	bool realtime = d40c->dma_cfg.realtime;
	bool highprio = d40c->dma_cfg.high_priority;
	u32 prioreg = highprio ? D40_DREG_PSEG1 : D40_DREG_PCEG1;
	u32 rtreg = realtime ? D40_DREG_RSEG1 : D40_DREG_RCEG1;
	u32 event = D40_TYPE_TO_EVENT(dev_type);
	u32 group = D40_TYPE_TO_GROUP(dev_type);
	u32 bit = 1 << event;

	/* Destination event lines are stored in the upper halfword */
	if (!src)
		bit <<= 16;

	writel(bit, d40c->base->virtbase + prioreg + group * 4);
	writel(bit, d40c->base->virtbase + rtreg + group * 4);
}

static void d40_set_prio_realtime(struct d40_chan *d40c)
{
	if (d40c->base->rev < 3)
		return;

	if ((d40c->dma_cfg.dir ==  STEDMA40_PERIPH_TO_MEM) ||
	    (d40c->dma_cfg.dir == STEDMA40_PERIPH_TO_PERIPH))
		__d40_set_prio_rt(d40c, d40c->dma_cfg.src_dev_type, true);

	if ((d40c->dma_cfg.dir ==  STEDMA40_MEM_TO_PERIPH) ||
	    (d40c->dma_cfg.dir == STEDMA40_PERIPH_TO_PERIPH))
		__d40_set_prio_rt(d40c, d40c->dma_cfg.dst_dev_type, false);
}

#ifdef MMC_HOST_DEBUGGING
static DEFINE_SPINLOCK(list_lock);
static LIST_HEAD(list);
#endif

/* DMA ENGINE functions */
static int d40_alloc_chan_resources(struct dma_chan *chan)
{
	int err;
	unsigned long flags;
	struct d40_chan *d40c =
		container_of(chan, struct d40_chan, chan);
	bool is_free_phy;

	d40_usage_inc(d40c);

	spin_lock_irqsave(&d40c->lock, flags);

	d40c->completed = chan->cookie = 1;

	if (!d40c->configured) {
		err = d40_config_memcpy(d40c);
		if (err) {
			dev_err(&d40c->chan.dev->device,
				"[%s] Failed to configure memcpy channel\n",
				__func__);
			goto fail;
		}
	}

	err = d40_allocate_channel(d40c, &is_free_phy);
	if (err) {
		dev_err(&d40c->chan.dev->device,
			"[%s] Failed to allocate channel\n", __func__);
		goto fail;
	}

	/* Fill in basic CFG register values */
	d40_phy_cfg(&d40c->dma_cfg, &d40c->src_def_cfg,
		    &d40c->dst_def_cfg, chan_is_logical(d40c));

	d40_set_prio_realtime(d40c);

	if (chan_is_logical(d40c)) {
		d40_log_cfg(&d40c->dma_cfg,
			    &d40c->log_def.lcsp1, &d40c->log_def.lcsp3);

		if (d40c->dma_cfg.dir == STEDMA40_PERIPH_TO_MEM)
			d40c->lcpa = d40c->base->lcpa_base +
				d40c->dma_cfg.src_dev_type *
				D40_LCPA_CHAN_SIZE;
		else
			d40c->lcpa = d40c->base->lcpa_base +
				d40c->dma_cfg.dst_dev_type *
				D40_LCPA_CHAN_SIZE + D40_LCPA_CHAN_DST_DELTA;
	}

	dev_info(chan2dev(d40c), "allocated %s channel (phy %d%s)\n",
		 chan_is_logical(d40c) ? "logical" : "physical",
		 d40c->phy_chan->num,
		 d40c->dma_cfg.use_fixed_channel ? ", fixed" : "");


	/*
	 * Only write channel configuration to the DMA if the physical
	 * resource is free. In case of multiple logical channels
	 * on the same physical resource, only the first write is necessary.
	 */

	if (is_free_phy)
		d40_config_write(d40c);
fail:
	d40_usage_dec(d40c);
	spin_unlock_irqrestore(&d40c->lock, flags);
#ifdef MMC_HOST_DEBUGGING
	spin_lock(&list_lock);
	list_add_tail(&d40c->list, &list);
	spin_unlock(&list_lock);
#endif
	return err;
}

static void d40_free_chan_resources(struct dma_chan *chan)
{
	struct d40_chan *d40c =
		container_of(chan, struct d40_chan, chan);
	int err;
	unsigned long flags;

	if (d40c->phy_chan == NULL) {
		dev_err(&d40c->chan.dev->device,
			"[%s] Cannot free unallocated channel\n", __func__);
		return;
	}

#ifdef MMC_HOST_DEBUGGING
	spin_lock(&list_lock);
	list_del(&d40c->list);
	spin_unlock(&list_lock);
#endif

	spin_lock_irqsave(&d40c->lock, flags);

	err = d40_free_dma(d40c);

	if (err)
		dev_err(&d40c->chan.dev->device,
			"[%s] Failed to free channel\n", __func__);
	spin_unlock_irqrestore(&d40c->lock, flags);
}

#ifdef MMC_HOST_DEBUGGING
static void __dump_descs(const char *name, struct list_head *list)
{
	struct d40_desc *desc;
	int i;

	list_for_each_entry(desc, list, node) {
		struct d40_log_lli_bidir *lli = &desc->lli_log;

		for (i = 0; i < desc->lli_len; i++) {
			printk("%s: src lcsp02 %#x\n", name, lli->src[i].lcsp02);
			printk("%s: src lcsp13 %#x\n", name, lli->src[i].lcsp13);
			printk("%s: dst lcsp02 %#x\n", name, lli->dst[i].lcsp02);
			printk("%s: dst lcsp13 %#x\n", name, lli->dst[i].lcsp13);
		}
	}
}

static dma_addr_t d40_dev_rx_addr(struct d40_chan *d40c);
static dma_addr_t d40_dev_tx_addr(struct d40_chan *d40c);

static void __dump_channel(struct d40_chan *chan)
{
	struct device *dev = chan2dev(chan);

	if (chan->phy_chan->num != CHANNEL_OF_CHOICE)
	       return;

	dev_info(dev, "busy: %d\n", chan->busy);
	dev_info(dev, "log_num: %d\n", chan->log_num);
	dev_info(dev, "phy_chan->num: %d\n", chan->phy_chan->num);
	dev_info(dev, "pending_tx: %d\n", chan->pending_tx);
	dev_info(dev, "completed: %d\n", chan->completed);

	if (chan->dma_cfg.src_dev_type != -1)
		dev_info(dev, "rx_addr: %#x\n", d40_dev_rx_addr(chan));

	if (chan->dma_cfg.dst_dev_type != -1)
		dev_info(dev, "tx_addr: %#x\n", d40_dev_tx_addr(chan));

	dev_info(dev, "lcpa lcsp0: %#x\n", chan->lcpa->lcsp0);
	dev_info(dev, "lcpa lcsp1: %#x\n", chan->lcpa->lcsp1);
	dev_info(dev, "lcpa lcsp2: %#x\n", chan->lcpa->lcsp2);
	dev_info(dev, "lcpa lcsp3: %#x\n", chan->lcpa->lcsp3);

	__dump_descs("queue", &chan->queue);
	__dump_descs("active", &chan->active);
	__dump_descs("done", &chan->done);
	__dump_descs("client", &chan->client);

	dev_info(dev, "phy: SSLNK: %#x\n",
	readl(chan->base->virtbase + D40_DREG_PCBASE + chan->phy_chan->num * D40_DREG_PCDELTA + D40_CHAN_REG_SSLNK));
	dev_info(dev, "phy: SDLNK: %#x\n",
	readl(chan->base->virtbase + D40_DREG_PCBASE + chan->phy_chan->num * D40_DREG_PCDELTA + D40_CHAN_REG_SDLNK));
}

static void dump_channels(void)
{
	struct d40_chan *chan, *temp;

	list_for_each_entry_safe(chan, temp, &list, list) {
		unsigned long flags;

		spin_lock_irqsave(&chan->lock, flags);
		__dump_channel(chan);
		spin_unlock_irqrestore(&chan->lock, flags);
	}

}
#endif

static struct dma_async_tx_descriptor *d40_prep_memcpy(struct dma_chan *chan,
						       dma_addr_t dst,
						       dma_addr_t src,
						       size_t size,
						       unsigned long dma_flags)
{
	struct d40_desc *d40d;
	struct d40_chan *d40c = container_of(chan, struct d40_chan,
					     chan);
	unsigned long flags;
	int err = 0;

	if (d40c->phy_chan == NULL) {
		dev_err(&d40c->chan.dev->device,
			"[%s] Channel is not allocated.\n", __func__);
		return ERR_PTR(-EINVAL);
	}

	spin_lock_irqsave(&d40c->lock, flags);
	d40d = d40_desc_get(d40c);

	if (d40d == NULL) {
		dev_err(&d40c->chan.dev->device,
			"[%s] Descriptor is NULL\n", __func__);
		goto err;
	}

	d40_usage_inc(d40c);

	d40d->txd.flags = dma_flags;

	dma_async_tx_descriptor_init(&d40d->txd, chan);

	d40d->txd.tx_submit = d40_tx_submit;

	if (chan_is_logical(d40c)) {

		if (d40_pool_lli_alloc(d40d, 1, true) < 0) {
			dev_err(&d40c->chan.dev->device,
				"[%s] Out of memory\n", __func__);
			goto err2;
		}
		d40d->lli_len = 1;
		d40d->lli_current = 0;

		d40_log_fill_lli(d40d->lli_log.src,
				 src,
				 size,
				 d40c->log_def.lcsp1,
				 d40c->dma_cfg.src_info.data_width,
				 true);

		d40_log_fill_lli(d40d->lli_log.dst,
				 dst,
				 size,
				 d40c->log_def.lcsp3,
				 d40c->dma_cfg.dst_info.data_width,
				 true);

	} else {

		if (d40_pool_lli_alloc(d40d, 1, false) < 0) {
			dev_err(&d40c->chan.dev->device,
				"[%s] Out of memory\n", __func__);
			goto err2;
		}

		err = d40_phy_fill_lli(d40d->lli_phy.src,
				       src,
				       size,
				       d40c->dma_cfg.src_info.psize,
				       0,
				       d40c->src_def_cfg,
				       true,
				       d40c->dma_cfg.src_info.data_width,
				       false);
		if (err)
			goto err_fill_lli;

		err = d40_phy_fill_lli(d40d->lli_phy.dst,
				       dst,
				       size,
				       d40c->dma_cfg.dst_info.psize,
				       0,
				       d40c->dst_def_cfg,
				       true,
				       d40c->dma_cfg.dst_info.data_width,
				       false);

		if (err)
			goto err_fill_lli;

		(void) dma_map_single(d40c->base->dev, d40d->lli_phy.src,
				      d40d->lli_pool.size, DMA_TO_DEVICE);
	}

	d40_usage_dec(d40c);
	spin_unlock_irqrestore(&d40c->lock, flags);
	return &d40d->txd;

err_fill_lli:
	dev_err(&d40c->chan.dev->device,
		"[%s] Failed filling in PHY LLI\n", __func__);
err2:
	d40_usage_dec(d40c);
err:
	if (d40d)
		d40_desc_free(d40c, d40d);
	spin_unlock_irqrestore(&d40c->lock, flags);
	return NULL;
}

static dma_addr_t d40_dev_rx_addr(struct d40_chan *d40c)
{
	dma_addr_t dev_addr = 0;

	if (d40c->runtime_addr)
		dev_addr = d40c->runtime_addr;
	else if (d40c->src_dev_addr)
		dev_addr = d40c->src_dev_addr;
	else
		dev_addr = d40c->base->plat_data->
			   dev_rx[d40c->dma_cfg.src_dev_type];

	return dev_addr;
}

static dma_addr_t d40_dev_tx_addr(struct d40_chan *d40c)
{
	dma_addr_t dev_addr = 0;

	if (d40c->runtime_addr)
		dev_addr = d40c->runtime_addr;
	else if (d40c->dst_dev_addr)
		dev_addr = d40c->dst_dev_addr;
	else
		dev_addr = d40c->base->plat_data->
			   dev_tx[d40c->dma_cfg.dst_dev_type];

	return dev_addr;
}

int stedma40_set_dev_addr(struct dma_chan *chan,
			  dma_addr_t src_dev_addr,
			  dma_addr_t dst_dev_addr)
{
	struct d40_chan *d40c = container_of(chan, struct d40_chan, chan);
	unsigned long flags;

	spin_lock_irqsave(&d40c->lock, flags);

	d40c->src_dev_addr = src_dev_addr;
	d40c->dst_dev_addr = dst_dev_addr;

	spin_unlock_irqrestore(&d40c->lock, flags);

	return 0;
}
EXPORT_SYMBOL(stedma40_set_dev_addr);

static int d40_prep_slave_sg_log(struct d40_desc *d40d,
				 struct d40_chan *d40c,
				 struct scatterlist *sgl,
				 unsigned int sg_len,
				 enum dma_data_direction direction,
				 unsigned long dma_flags)
{
	dma_addr_t dev_addr = 0;
	int total_size;

	if (d40_pool_lli_alloc(d40d, sg_len, true) < 0) {
		dev_err(&d40c->chan.dev->device,
			"[%s] Out of memory\n", __func__);
		return -ENOMEM;
	}

	d40d->lli_len = sg_len;
	d40d->lli_current = 0;

	if (direction == DMA_FROM_DEVICE)
		dev_addr = d40_dev_rx_addr(d40c);
	else if (direction == DMA_TO_DEVICE)
		dev_addr = d40_dev_tx_addr(d40c);
	else
		return -EINVAL;

	total_size = d40_log_sg_to_dev(sgl, sg_len,
				       &d40d->lli_log,
				       &d40c->log_def,
				       d40c->dma_cfg.src_info.data_width,
				       d40c->dma_cfg.dst_info.data_width,
				       direction,
				       dev_addr);
	if (total_size < 0)
		return -EINVAL;

	return 0;
}

static int d40_prep_slave_sg_phy(struct d40_desc *d40d,
				 struct d40_chan *d40c,
				 struct scatterlist *sgl,
				 unsigned int sgl_len,
				 enum dma_data_direction direction,
				 unsigned long dma_flags)
{
	dma_addr_t src_dev_addr;
	dma_addr_t dst_dev_addr;
	int res;

	if (d40_pool_lli_alloc(d40d, sgl_len, false) < 0) {
		dev_err(&d40c->chan.dev->device,
			"[%s] Out of memory\n", __func__);
		return -ENOMEM;
	}

	d40d->lli_len = sgl_len;
	d40d->lli_current = 0;

	if (direction == DMA_FROM_DEVICE) {
		dst_dev_addr = 0;
		src_dev_addr = d40_dev_rx_addr(d40c);
	} else if (direction == DMA_TO_DEVICE) {
		dst_dev_addr = d40_dev_tx_addr(d40c);
		src_dev_addr = 0;
	} else
		return -EINVAL;

	res = d40_phy_sg_to_lli(sgl,
				sgl_len,
				src_dev_addr,
				d40d->lli_phy.src,
				virt_to_phys(d40d->lli_phy.src),
				d40c->src_def_cfg,
				d40c->dma_cfg.src_info.data_width,
				d40c->dma_cfg.src_info.psize,
				d40d->cyclic,
				d40d->txd.flags & DMA_PREP_INTERRUPT);
	if (res < 0)
		return res;

	res = d40_phy_sg_to_lli(sgl,
				sgl_len,
				dst_dev_addr,
				d40d->lli_phy.dst,
				virt_to_phys(d40d->lli_phy.dst),
				d40c->dst_def_cfg,
				d40c->dma_cfg.dst_info.data_width,
				d40c->dma_cfg.dst_info.psize,
				d40d->cyclic,
				d40d->txd.flags & DMA_PREP_INTERRUPT);
	if (res < 0)
		return res;

	(void) dma_map_single(d40c->base->dev, d40d->lli_phy.src,
			      d40d->lli_pool.size, DMA_TO_DEVICE);
	return 0;
}

static struct dma_async_tx_descriptor *
d40_prep_slave_sg(struct dma_chan *chan, struct scatterlist *sgl,
		  unsigned int sg_len, enum dma_data_direction direction,
		  unsigned long dma_flags)
{
	struct d40_desc *d40d;
	struct d40_chan *d40c = container_of(chan, struct d40_chan,
					     chan);
	unsigned long flags;
	int err;


	if (d40c->phy_chan == NULL) {
		dev_err(&d40c->chan.dev->device,
			"[%s] Cannot prepare unallocated channel\n", __func__);
		return ERR_PTR(-EINVAL);
	}

	spin_lock_irqsave(&d40c->lock, flags);
	d40d = d40_desc_get(d40c);

	if (d40d == NULL)
		goto err;
	d40_usage_inc(d40c);

	if (chan_is_logical(d40c))
		err = d40_prep_slave_sg_log(d40d, d40c, sgl, sg_len,
					    direction, dma_flags);
	else
		err = d40_prep_slave_sg_phy(d40d, d40c, sgl, sg_len,
					    direction, dma_flags);
	d40_usage_dec(d40c);

	if (err) {
		dev_err(&d40c->chan.dev->device,
			"[%s] Failed to prepare %s slave sg job: %d\n",
			__func__,
			chan_is_logical(d40c) ? "log" : "phy", err);
		goto err;
	}

	d40d->txd.flags = dma_flags;

	dma_async_tx_descriptor_init(&d40d->txd, chan);

	d40d->txd.tx_submit = d40_tx_submit;

	spin_unlock_irqrestore(&d40c->lock, flags);
	return &d40d->txd;

err:
	if (d40d)
		d40_desc_free(d40c, d40d);
	spin_unlock_irqrestore(&d40c->lock, flags);
	return NULL;
}

static enum dma_status d40_tx_status(struct dma_chan *chan,
				     dma_cookie_t cookie,
				     struct dma_tx_state *txstate)
{
	struct d40_chan *d40c = container_of(chan, struct d40_chan, chan);
	unsigned long flags;
	dma_cookie_t last_used;
	dma_cookie_t last_complete;
	int ret;

	if (d40c->phy_chan == NULL) {
		dev_err(&d40c->chan.dev->device,
			"[%s] Cannot read status of unallocated channel\n",
			__func__);
		return -EINVAL;
	}

	last_complete = d40c->completed;
	last_used = chan->cookie;

	if (d40_is_paused(d40c))
		ret = DMA_PAUSED;
	else
		ret = dma_async_is_complete(cookie, last_complete, last_used);

	if (txstate) {
		txstate->last = last_complete;
		txstate->used = last_used;

		spin_lock_irqsave(&d40c->lock, flags);
		txstate->residue = d40_residue(d40c);
		spin_unlock_irqrestore(&d40c->lock, flags);
	}

	return ret;
}

static void d40_issue_pending(struct dma_chan *chan)
{
	struct d40_chan *d40c = container_of(chan, struct d40_chan, chan);
	unsigned long flags;

	if (d40c->phy_chan == NULL) {
		dev_err(&d40c->chan.dev->device,
			"[%s] Channel is not allocated!\n", __func__);
		return;
	}

	spin_lock_irqsave(&d40c->lock, flags);

	/* Busy means that pending jobs are already being processed */
	if (!d40c->busy)
		(void) d40_queue_start(d40c);

	spin_unlock_irqrestore(&d40c->lock, flags);
}

static void d40_terminate_all(struct dma_chan *chan)
{
	unsigned long flags;
	struct d40_chan *d40c = container_of(chan, struct d40_chan, chan);
	int ret;

	spin_lock_irqsave(&d40c->lock, flags);

	d40_usage_inc(d40c);

	ret = d40_channel_execute_command(d40c, D40_DMA_STOP);
	if (ret)
		dev_err(&d40c->chan.dev->device,
			"[%s] Failed to stop channel\n", __func__);

	d40_term_all(d40c);
	d40_usage_dec(d40c);
	if (d40c->busy)
		d40_usage_dec(d40c);
	d40c->busy = false;

	spin_unlock_irqrestore(&d40c->lock, flags);
}

static int
dma40_config_to_halfchannel(struct d40_chan *d40c,
			    struct stedma40_half_channel_info *info,
			    enum dma_slave_buswidth width,
			    u32 maxburst)
{
	enum stedma40_periph_data_width addr_width;
	int psize;

	switch (width) {
	case DMA_SLAVE_BUSWIDTH_1_BYTE:
		addr_width = STEDMA40_BYTE_WIDTH;
		break;
	case DMA_SLAVE_BUSWIDTH_2_BYTES:
		addr_width = STEDMA40_HALFWORD_WIDTH;
		break;
	case DMA_SLAVE_BUSWIDTH_4_BYTES:
		addr_width = STEDMA40_WORD_WIDTH;
		break;
	case DMA_SLAVE_BUSWIDTH_8_BYTES:
		addr_width = STEDMA40_DOUBLEWORD_WIDTH;
		break;
	default:
		dev_err(d40c->base->dev,
			"illegal peripheral address width "
			"requested (%d)\n",
			width);
		return -EINVAL;
	}

	if (chan_is_logical(d40c)) {
		if (maxburst >= 16)
			psize = STEDMA40_PSIZE_LOG_16;
		else if (maxburst >= 8)
			psize = STEDMA40_PSIZE_LOG_8;
		else if (maxburst >= 4)
			psize = STEDMA40_PSIZE_LOG_4;
		else
			psize = STEDMA40_PSIZE_LOG_1;
	} else {
		if (maxburst >= 16)
			psize = STEDMA40_PSIZE_PHY_16;
		else if (maxburst >= 8)
			psize = STEDMA40_PSIZE_PHY_8;
		else if (maxburst >= 4)
			psize = STEDMA40_PSIZE_PHY_4;
		else
			psize = STEDMA40_PSIZE_PHY_1;
	}

	info->data_width = addr_width;
	info->psize = psize;
	info->flow_ctrl = STEDMA40_NO_FLOW_CTRL;

	return 0;
}

/* Runtime reconfiguration extension */
static int d40_set_runtime_config(struct dma_chan *chan,
				  struct dma_slave_config *config)
{
	struct d40_chan *d40c = container_of(chan, struct d40_chan, chan);
	struct stedma40_chan_cfg *cfg = &d40c->dma_cfg;
	enum dma_slave_buswidth src_addr_width, dst_addr_width;
	dma_addr_t config_addr;
	u32 src_maxburst, dst_maxburst;
	int ret;

	src_addr_width = config->src_addr_width;
	src_maxburst = config->src_maxburst;
	dst_addr_width = config->dst_addr_width;
	dst_maxburst = config->dst_maxburst;

	if (config->direction == DMA_FROM_DEVICE) {
		dma_addr_t dev_addr_rx =
			d40c->base->plat_data->dev_rx[cfg->src_dev_type];

		config_addr = config->src_addr;
		if (dev_addr_rx)
			dev_dbg(d40c->base->dev,
				"channel has a pre-wired RX address %08x "
				"overriding with %08x\n",
				dev_addr_rx, config_addr);
		if (cfg->dir != STEDMA40_PERIPH_TO_MEM)
			dev_dbg(d40c->base->dev,
				"channel was not configured for peripheral "
				"to memory transfer (%d) overriding\n",
				cfg->dir);
		cfg->dir = STEDMA40_PERIPH_TO_MEM;

		/* Configure the memory side */
		if (dst_addr_width == DMA_SLAVE_BUSWIDTH_UNDEFINED)
			dst_addr_width = src_addr_width;
		if (dst_maxburst == 0)
			dst_maxburst = src_maxburst;

	} else if (config->direction == DMA_TO_DEVICE) {
		dma_addr_t dev_addr_tx =
			d40c->base->plat_data->dev_tx[cfg->dst_dev_type];

		config_addr = config->dst_addr;
		if (dev_addr_tx)
			dev_dbg(d40c->base->dev,
				"channel has a pre-wired TX address %08x "
				"overriding with %08x\n",
				dev_addr_tx, config_addr);
		if (cfg->dir != STEDMA40_MEM_TO_PERIPH)
			dev_dbg(d40c->base->dev,
				"channel was not configured for memory "
				"to peripheral transfer (%d) overriding\n",
				cfg->dir);
		cfg->dir = STEDMA40_MEM_TO_PERIPH;

		/* Configure the memory side */
		if (src_addr_width == DMA_SLAVE_BUSWIDTH_UNDEFINED)
			src_addr_width = dst_addr_width;
		if (src_maxburst == 0)
			src_maxburst = dst_maxburst;

	} else {
		dev_err(d40c->base->dev,
			"unrecognized channel direction %d\n",
			config->direction);
		return -EINVAL;
	}

	if (src_maxburst * src_addr_width != dst_maxburst * dst_addr_width) {
		dev_err(d40c->base->dev,
			"src/dst width/maxburst mismatch: %d*%d != %d*%d\n",
			src_maxburst,
			src_addr_width,
			dst_maxburst,
			dst_addr_width);
		return -EINVAL;
	}

	ret = dma40_config_to_halfchannel(d40c, &cfg->src_info,
					  src_addr_width,
					  src_maxburst);
	if (ret)
		return ret;

	ret = dma40_config_to_halfchannel(d40c, &cfg->dst_info,
					  dst_addr_width,
					  dst_maxburst);
	if (ret)
		return ret;

	/* Fill in register values */
	if (chan_is_logical(d40c))
		d40_log_cfg(cfg, &d40c->log_def.lcsp1, &d40c->log_def.lcsp3);
	else
		d40_phy_cfg(cfg, &d40c->src_def_cfg,
			    &d40c->dst_def_cfg, false);

	/* These settings will take precedence later */
	d40c->runtime_addr = config_addr;
	d40c->runtime_direction = config->direction;
	dev_dbg(d40c->base->dev,
		"configured channel %s for %s, data width %d/%d, "
		"maxburst %d/%d elements, LE, no flow control\n",
		dma_chan_name(chan),
		(config->direction == DMA_FROM_DEVICE) ? "RX" : "TX",
		src_addr_width, dst_addr_width,
		src_maxburst, dst_maxburst);

	return 0;
}

static int d40_control(struct dma_chan *chan, enum dma_ctrl_cmd cmd,
		       unsigned long arg)
{
	struct d40_chan *d40c = container_of(chan, struct d40_chan, chan);

	if (d40c->phy_chan == NULL) {
		dev_err(&d40c->chan.dev->device,
			"[%s] Channel is not allocated!\n", __func__);
		return -EINVAL;
	}

	switch (cmd) {
	case DMA_TERMINATE_ALL:
		d40_terminate_all(chan);
		return 0;
	case DMA_PAUSE:
		return d40_pause(chan);
	case DMA_RESUME:
		return d40_resume(chan);
	case DMA_SLAVE_CONFIG:
		return d40_set_runtime_config(chan,
			(struct dma_slave_config *) arg);
	default:
		break;
	}

	/* Other commands are unimplemented */
	return -ENXIO;
}

dma_addr_t stedma40_get_src_addr(struct dma_chan *chan)
{
	struct d40_chan *d40c = container_of(chan, struct d40_chan, chan);
	dma_addr_t addr;

	if (chan_is_physical(d40c))
		addr = readl(d40c->base->virtbase + D40_DREG_PCBASE +
			     d40c->phy_chan->num * D40_DREG_PCDELTA +
			     D40_CHAN_REG_SSPTR);
	else {
		unsigned long lower;
		unsigned long upper;

		/*
		 * There is a potential for overflow between the time the two
		 * halves of the pointer are read.
		 */
		lower = d40c->lcpa->lcsp0 & D40_MEM_LCSP0_SPTR_MASK;
		upper = d40c->lcpa->lcsp1 & D40_MEM_LCSP1_SPTR_MASK;

		addr = upper | lower;
	}

	return addr;
}
EXPORT_SYMBOL(stedma40_get_src_addr);

dma_addr_t stedma40_get_dst_addr(struct dma_chan *chan)
{
	struct d40_chan *d40c = container_of(chan, struct d40_chan, chan);
	dma_addr_t addr;

	if (chan_is_physical(d40c))
		addr = readl(d40c->base->virtbase + D40_DREG_PCBASE +
			     d40c->phy_chan->num * D40_DREG_PCDELTA +
			     D40_CHAN_REG_SDPTR);
	else {
		unsigned long lower;
		unsigned long upper;

		lower = d40c->lcpa->lcsp2 & D40_MEM_LCSP2_DPTR_MASK;
		upper = d40c->lcpa->lcsp3 & D40_MEM_LCSP3_DPTR_MASK;

		addr = upper | lower;
	}

	return addr;
}
EXPORT_SYMBOL(stedma40_get_dst_addr);

int stedma40_cyclic_start(struct dma_chan *chan)
{
	struct d40_chan *d40c = container_of(chan, struct d40_chan, chan);
	unsigned long flags;
	int ret = -EINVAL;

	spin_lock_irqsave(&d40c->lock, flags);

	if (!d40c->cdesc)
		goto out;

	d40_usage_inc(d40c);

	ret = d40_start(d40c);
	if (!ret)
		d40c->busy = true;
	else
		d40_usage_dec(d40c);

out:
	spin_unlock_irqrestore(&d40c->lock, flags);
	return ret;
}
EXPORT_SYMBOL(stedma40_cyclic_start);

void stedma40_cyclic_stop(struct dma_chan *chan)
{
	d40_terminate_all(chan);
}
EXPORT_SYMBOL(stedma40_cyclic_stop);

void stedma40_cyclic_free(struct dma_chan *chan)
{
	struct d40_chan *d40c = container_of(chan, struct d40_chan, chan);
	struct stedma40_cyclic_desc *cdesc;
	unsigned long flags;

	spin_lock_irqsave(&d40c->lock, flags);

	cdesc = d40c->cdesc;
	if (!cdesc) {
		spin_unlock_irqrestore(&d40c->lock, flags);
		return;
	}

	d40c->cdesc = NULL;
	d40_lcla_free_all(d40c, cdesc->d40d);

	spin_unlock_irqrestore(&d40c->lock, flags);

	kfree(cdesc);
}
EXPORT_SYMBOL(stedma40_cyclic_free);

struct stedma40_cyclic_desc *
stedma40_cyclic_prep_sg(struct dma_chan *chan,
			struct scatterlist *sgl,
			unsigned int sg_len,
			enum dma_data_direction direction,
			unsigned long dma_flags)
{
	struct d40_chan *d40c = container_of(chan, struct d40_chan, chan);
	struct stedma40_cyclic_desc *cdesc;
	struct d40_desc *d40d;
	unsigned long flags;
	void *mem;
	int err;

	mem = kzalloc(sizeof(struct stedma40_cyclic_desc)
		      + sizeof(struct d40_desc), GFP_ATOMIC);
	if (!mem)
		return ERR_PTR(-ENOMEM);

	cdesc = mem;
	d40d = cdesc->d40d = mem + sizeof(struct stedma40_cyclic_desc);

	spin_lock_irqsave(&d40c->lock, flags);

	if (d40c->phy_chan == NULL) {
		dev_err(&chan->dev->device,
			"[%s] Cannot prepare unallocated channel\n", __func__);
		err = -EINVAL;
		goto out;
	}

	if (d40c->cdesc || d40c->busy) {
		dev_err(&d40c->chan.dev->device,
			"[%s] Cannot prepare cyclic job for busy channel\n",
			__func__);
		err = -EBUSY;
		goto out;
	}

	d40d->cyclic = true;
	d40d->txd.flags = dma_flags;
	INIT_LIST_HEAD(&d40d->node);

	d40_usage_inc(d40c);

	if (chan_is_logical(d40c))
		err = d40_prep_slave_sg_log(d40d, d40c, sgl, sg_len,
					    direction, dma_flags);
	else
		err = d40_prep_slave_sg_phy(d40d, d40c, sgl, sg_len,
					    direction, dma_flags);

	if (err) {
		dev_err(&d40c->chan.dev->device,
			"[%s] Failed to prepare %s slave sg job: %d\n",
			__func__,
			chan_is_logical(d40c) ? "log" : "phy", err);
		goto out2;
	}

	d40_desc_load(d40c, d40d);

	/*
	 * Couldn't get enough LCLA.  We don't support splitting of cyclic
	 * jobs.
	 */
	if (d40d->lli_current != d40d->lli_len) {
		dev_err(&chan->dev->device,
			"[%s] Couldn't prepare cyclic job: not enough LCLA",
			__func__);
		err = -EBUSY;
		goto out2;
	}

	d40c->cdesc = cdesc;
	d40_usage_dec(d40c);
	spin_unlock_irqrestore(&d40c->lock, flags);
	return cdesc;
out2:
	d40_usage_dec(d40c);
out:
	if (d40c->phy_chan)
		d40_lcla_free_all(d40c, cdesc->d40d);
	kfree(cdesc);
	spin_unlock_irqrestore(&d40c->lock, flags);
	return ERR_PTR(err);
}
EXPORT_SYMBOL(stedma40_cyclic_prep_sg);

/* Initialization functions */

static void __init d40_chan_init(struct d40_base *base, struct dma_device *dma,
				 struct d40_chan *chans, int offset,
				 int num_chans)
{
	int i = 0;
	struct d40_chan *d40c;

	INIT_LIST_HEAD(&dma->channels);

	for (i = offset; i < offset + num_chans; i++) {
		d40c = &chans[i];
		d40c->base = base;
		d40c->chan.device = dma;

		spin_lock_init(&d40c->lock);

		d40c->log_num = D40_PHY_CHAN;

		INIT_LIST_HEAD(&d40c->done);
		INIT_LIST_HEAD(&d40c->active);
		INIT_LIST_HEAD(&d40c->queue);
		INIT_LIST_HEAD(&d40c->client);

		tasklet_init(&d40c->tasklet, dma_tasklet,
			     (unsigned long) d40c);

		list_add_tail(&d40c->chan.device_node,
			      &dma->channels);
	}
}

static int __init d40_dmaengine_init(struct d40_base *base,
				     int num_reserved_chans)
{
	int err ;

	d40_chan_init(base, &base->dma_slave, base->log_chans,
		      0, base->num_log_chans);

	dma_cap_zero(base->dma_slave.cap_mask);
	dma_cap_set(DMA_SLAVE, base->dma_slave.cap_mask);

	base->dma_slave.device_alloc_chan_resources = d40_alloc_chan_resources;
	base->dma_slave.device_free_chan_resources = d40_free_chan_resources;
	base->dma_slave.device_prep_dma_memcpy = d40_prep_memcpy;
	base->dma_slave.device_prep_slave_sg = d40_prep_slave_sg;
	base->dma_slave.device_tx_status = d40_tx_status;
	base->dma_slave.device_control = d40_control;
	base->dma_slave.device_issue_pending = d40_issue_pending;
	base->dma_slave.dev = base->dev;

	err = dma_async_device_register(&base->dma_slave);

	if (err) {
		dev_err(base->dev,
			"[%s] Failed to register slave channels\n",
			__func__);
		goto failure1;
	}

	d40_chan_init(base, &base->dma_memcpy, base->log_chans,
		      base->num_log_chans, base->plat_data->memcpy_len);

	dma_cap_zero(base->dma_memcpy.cap_mask);
	dma_cap_set(DMA_MEMCPY, base->dma_memcpy.cap_mask);

	base->dma_memcpy.device_alloc_chan_resources = d40_alloc_chan_resources;
	base->dma_memcpy.device_free_chan_resources = d40_free_chan_resources;
	base->dma_memcpy.device_prep_dma_memcpy = d40_prep_memcpy;
	base->dma_memcpy.device_prep_slave_sg = d40_prep_slave_sg;
	base->dma_memcpy.device_tx_status = d40_tx_status;
	base->dma_memcpy.device_control = d40_control;
	base->dma_memcpy.device_issue_pending = d40_issue_pending;
	base->dma_memcpy.dev = base->dev;
	/*
	 * This controller can only access address at even
	 * 32bit boundaries, i.e. 2^2
	 */
	base->dma_memcpy.copy_align = 2;

	err = dma_async_device_register(&base->dma_memcpy);

	if (err) {
		dev_err(base->dev,
			"[%s] Failed to regsiter memcpy only channels\n",
			__func__);
		goto failure2;
	}

	d40_chan_init(base, &base->dma_both, base->phy_chans,
		      0, num_reserved_chans);

	dma_cap_zero(base->dma_both.cap_mask);
	dma_cap_set(DMA_SLAVE, base->dma_both.cap_mask);
	dma_cap_set(DMA_MEMCPY, base->dma_both.cap_mask);

	base->dma_both.device_alloc_chan_resources = d40_alloc_chan_resources;
	base->dma_both.device_free_chan_resources = d40_free_chan_resources;
	base->dma_both.device_prep_dma_memcpy = d40_prep_memcpy;
	base->dma_both.device_prep_slave_sg = d40_prep_slave_sg;
	base->dma_both.device_tx_status = d40_tx_status;
	base->dma_both.device_control = d40_control;
	base->dma_both.device_issue_pending = d40_issue_pending;

	base->dma_both.dev = base->dev;
	base->dma_both.copy_align = 2;
	err = dma_async_device_register(&base->dma_both);

	if (err) {
		dev_err(base->dev,
			"[%s] Failed to register logical and physical capable channels\n",
			__func__);
		goto failure3;
	}
	return 0;
failure3:
	dma_async_device_unregister(&base->dma_memcpy);
failure2:
	dma_async_device_unregister(&base->dma_slave);
failure1:
	return err;
}

/* Suspend resume functionality */
#ifdef CONFIG_PM
static int dma40_pm_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct d40_base *base = platform_get_drvdata(pdev);
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&base->usage_lock, flags);

	if (base->usage)
		ret = -EBUSY;

	spin_unlock_irqrestore(&base->usage_lock, flags);

	if (base->lcpa_regulator)
		ret = regulator_disable(base->lcpa_regulator);

	return ret;
}

static int dma40_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct d40_base *base = platform_get_drvdata(pdev);

	d40_save_restore_registers(base, true);

	return 0;
}

static int dma40_runtime_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct d40_base *base = platform_get_drvdata(pdev);

	if (base->initialized)
		d40_save_restore_registers(base, false);

	return 0;
}

static int dma40_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct d40_base *base = platform_get_drvdata(pdev);
	int ret = 0;

	if (base->lcpa_regulator)
		ret = regulator_enable(base->lcpa_regulator);

	return ret;
}

static const struct dev_pm_ops dma40_pm_ops = {
	.suspend		= dma40_pm_suspend,
	.runtime_suspend	= dma40_runtime_suspend,
	.runtime_resume		= dma40_runtime_resume,
	.resume			= dma40_resume,
};
#define DMA40_PM_OPS	(&dma40_pm_ops)
#else
#define DMA40_PM_OPS	NULL
#endif

/* Initialization functions. */

static int __init d40_phy_res_init(struct d40_base *base)
{
	int i;
	int num_phy_chans_avail = 0;
	u32 val[2];
	u32 gcc = D40_DREG_GCC_ENA;
	int odd_even_bit = -2;

	val[0] = readl(base->virtbase + D40_DREG_PRSME);
	val[1] = readl(base->virtbase + D40_DREG_PRSMO);

	for (i = 0; i < base->num_phy_chans; i++) {
		base->phy_res[i].num = i;
		odd_even_bit += 2 * ((i % 2) == 0);
		if (((val[i % 2] >> odd_even_bit) & D40_DREG_PRSM_MODE_MASK)
		      == D40_DREG_PRSM_MODE_SECURE) {
			/* Mark security only channels as occupied */
			base->phy_res[i].allocated_src = D40_ALLOC_PHY;
			base->phy_res[i].allocated_dst = D40_ALLOC_PHY;
			base->phy_res[i].reserved = true;

			gcc |= D40_DREG_GCC_EVTGRP_ENA(D40_PHYS_TO_GROUP(i),
						       D40_DREG_GCC_DST);
			gcc |= D40_DREG_GCC_EVTGRP_ENA(D40_PHYS_TO_GROUP(i),
						       D40_DREG_GCC_SRC);

		} else {
			base->phy_res[i].allocated_src = D40_ALLOC_FREE;
			base->phy_res[i].allocated_dst = D40_ALLOC_FREE;
			base->phy_res[i].reserved = false;
			num_phy_chans_avail++;
		}
		spin_lock_init(&base->phy_res[i].lock);
	}

	/* Mark disabled channels as occupied */
	for (i = 0; base->plat_data->disabled_channels[i] != -1; i++) {
		int chan = base->plat_data->disabled_channels[i];

		base->phy_res[chan].allocated_src = D40_ALLOC_PHY;
		base->phy_res[chan].allocated_dst = D40_ALLOC_PHY;
		base->phy_res[chan].reserved = true;
		num_phy_chans_avail--;
	}

	dev_info(base->dev, "%d of %d physical DMA channels available\n",
		 num_phy_chans_avail, base->num_phy_chans);

	/* Verify settings extended vs standard */
	val[0] = readl(base->virtbase + D40_DREG_PRTYP);

	for (i = 0; i < base->num_phy_chans; i++) {

		if (base->phy_res[i].allocated_src == D40_ALLOC_FREE &&
		    (val[0] & 0x3) != 1)
			dev_info(base->dev,
				 "[%s] INFO: channel %d is misconfigured (%d)\n",
				 __func__, i, val[0] & 0x3);

		val[0] = val[0] >> 2;
	}

	/* Enable all clocks -- revisit after HW bug is fixed */
	writel(D40_DREG_GCC_ENABLE_ALL, base->virtbase + D40_DREG_GCC);

	return num_phy_chans_avail;
}

static struct d40_base * __init d40_hw_detect_init(struct platform_device *pdev)
{
	static const struct d40_reg_val dma_id_regs[] = {
		/* Peripheral Id */
		{ .reg = D40_DREG_PERIPHID0, .val = 0x0040},
		{ .reg = D40_DREG_PERIPHID1, .val = 0x0000},
		/*
		 * D40_DREG_PERIPHID2 Depends on HW revision:
		 *  DB8500ed has 0x0008,
		 *  ? has 0x0018,
		 *  DB8500v1 has 0x0028
		 *  DB8500v2 has 0x0038
		 */
		{ .reg = D40_DREG_PERIPHID3, .val = 0x0000},

		/* PCell Id */
		{ .reg = D40_DREG_CELLID0, .val = 0x000d},
		{ .reg = D40_DREG_CELLID1, .val = 0x00f0},
		{ .reg = D40_DREG_CELLID2, .val = 0x0005},
		{ .reg = D40_DREG_CELLID3, .val = 0x00b1}
	};
	struct stedma40_platform_data *plat_data;
	struct clk *clk = NULL;
	void __iomem *virtbase = NULL;
	struct resource *res = NULL;
	struct d40_base *base = NULL;
	int num_log_chans = 0;
	int num_phy_chans;
	int i;
	u32 val;

	clk = clk_get(&pdev->dev, NULL);

	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "[%s] No matching clock found\n",
			__func__);
		goto failure;
	}

	/*
	 * Since the secure world does not handle clock, we have to
	 * let it run all the time
	 */
	clk_enable(clk);

	/* Get IO for DMAC base address */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "base");
	if (!res)
		goto failure;

	if (request_mem_region(res->start, resource_size(res),
			       D40_NAME " I/O base") == NULL)
		goto failure;

	virtbase = ioremap(res->start, resource_size(res));
	if (!virtbase)
		goto failure;

	/* HW version check */
	for (i = 0; i < ARRAY_SIZE(dma_id_regs); i++) {
		if (dma_id_regs[i].val !=
		    readl(virtbase + dma_id_regs[i].reg)) {
			dev_err(&pdev->dev,
				"[%s] Unknown hardware! Expected 0x%x at 0x%x but got 0x%x\n",
				__func__,
				dma_id_regs[i].val,
				dma_id_regs[i].reg,
				readl(virtbase + dma_id_regs[i].reg));
			goto failure;
		}
	}

	/* Get silicon revision */
	val = readl(virtbase + D40_DREG_PERIPHID2);

	if ((val & 0xf) != D40_PERIPHID2_DESIGNER) {
		dev_err(&pdev->dev,
			"[%s] Unknown designer! Got %x wanted %x\n",
			__func__, val & 0xf, D40_PERIPHID2_DESIGNER);
		goto failure;
	}

	/* The number of physical channels on this HW */
	num_phy_chans = 4 * (readl(virtbase + D40_DREG_ICFG) & 0x7) + 4;

	dev_info(&pdev->dev, "hardware revision: %d @ 0x%x\n",
		 (val >> 4) & 0xf, res->start);

	plat_data = pdev->dev.platform_data;

	/* Count the number of logical channels in use */
	for (i = 0; i < plat_data->dev_len; i++)
		if (plat_data->dev_rx[i] != 0)
			num_log_chans++;

	for (i = 0; i < plat_data->dev_len; i++)
		if (plat_data->dev_tx[i] != 0)
			num_log_chans++;

	base = kzalloc(ALIGN(sizeof(struct d40_base), 4) +
		       (num_phy_chans + num_log_chans + plat_data->memcpy_len) *
		       sizeof(struct d40_chan), GFP_KERNEL);

	if (base == NULL) {
		dev_err(&pdev->dev, "[%s] Out of memory\n", __func__);
		goto failure;
	}

	base->rev = (val >> 4) & 0xf;
	base->clk = clk;
	base->num_phy_chans = num_phy_chans;
	base->num_log_chans = num_log_chans;
	base->phy_start = res->start;
	base->phy_size = resource_size(res);
	base->virtbase = virtbase;
	base->plat_data = plat_data;
	base->dev = &pdev->dev;
	base->phy_chans = ((void *)base) + ALIGN(sizeof(struct d40_base), 4);
	base->log_chans = &base->phy_chans[num_phy_chans];
	base->usage = 1;

	base->phy_res = kzalloc(num_phy_chans * sizeof(struct d40_phy_res),
				GFP_KERNEL);
	if (!base->phy_res)
		goto failure;

	base->lookup_phy_chans = kzalloc(num_phy_chans *
					 sizeof(struct d40_chan *),
					 GFP_KERNEL);
	if (!base->lookup_phy_chans)
		goto failure;

	if (num_log_chans + plat_data->memcpy_len) {
		/*
		 * The max number of logical channels are event lines for all
		 * src devices and dst devices
		 */
		base->lookup_log_chans = kzalloc(plat_data->dev_len * 2 *
						 sizeof(struct d40_chan *),
						 GFP_KERNEL);
		if (!base->lookup_log_chans)
			goto failure;
	}

	base->reg_val_backup_chan = kmalloc(base->num_phy_chans *
					    sizeof(d40_backup_regs_chan),
					    GFP_KERNEL);
	if (!base->reg_val_backup_chan)
		goto failure;

	base->lcla_pool.alloc_map =
		kzalloc(num_phy_chans * sizeof(struct d40_desc *)
			* D40_LCLA_LINK_PER_EVENT_GRP, GFP_KERNEL);
	if (!base->lcla_pool.alloc_map)
		goto failure;

	base->desc_slab = kmem_cache_create(D40_NAME, sizeof(struct d40_desc),
					    0, SLAB_HWCACHE_ALIGN,
					    NULL);
	if (base->desc_slab == NULL)
		goto failure;

	return base;

failure:
	if (!IS_ERR(clk)) {
		clk_disable(clk);
		clk_put(clk);
	}
	if (virtbase)
		iounmap(virtbase);
	if (res)
		release_mem_region(res->start,
				   resource_size(res));
	if (virtbase)
		iounmap(virtbase);

	if (base) {
		kfree(base->lcla_pool.alloc_map);
		kfree(base->reg_val_backup_chan);
		kfree(base->lookup_log_chans);
		kfree(base->lookup_phy_chans);
		kfree(base->phy_res);
		kfree(base);
	}

	return NULL;
}

static void __init d40_hw_init(struct d40_base *base)
{

	static struct d40_reg_val dma_init_reg[] = {
		/* Clock every part of the DMA block from start */
		{ .reg = D40_DREG_GCC,    .val = D40_DREG_GCC_ENABLE_ALL},

		/* Interrupts on all logical channels */
		{ .reg = D40_DREG_LCMIS0, .val = 0xFFFFFFFF},
		{ .reg = D40_DREG_LCMIS1, .val = 0xFFFFFFFF},
		{ .reg = D40_DREG_LCMIS2, .val = 0xFFFFFFFF},
		{ .reg = D40_DREG_LCMIS3, .val = 0xFFFFFFFF},
		{ .reg = D40_DREG_LCICR0, .val = 0xFFFFFFFF},
		{ .reg = D40_DREG_LCICR1, .val = 0xFFFFFFFF},
		{ .reg = D40_DREG_LCICR2, .val = 0xFFFFFFFF},
		{ .reg = D40_DREG_LCICR3, .val = 0xFFFFFFFF},
		{ .reg = D40_DREG_LCTIS0, .val = 0xFFFFFFFF},
		{ .reg = D40_DREG_LCTIS1, .val = 0xFFFFFFFF},
		{ .reg = D40_DREG_LCTIS2, .val = 0xFFFFFFFF},
		{ .reg = D40_DREG_LCTIS3, .val = 0xFFFFFFFF}
	};
	int i;
	u32 prmseo[2] = {0, 0};
	u32 activeo[2] = {0xFFFFFFFF, 0xFFFFFFFF};
	u32 pcmis = 0;
	u32 pcicr = 0;

	for (i = 0; i < ARRAY_SIZE(dma_init_reg); i++)
		writel(dma_init_reg[i].val,
		       base->virtbase + dma_init_reg[i].reg);

	/* Configure all our dma channels to default settings */
	for (i = 0; i < base->num_phy_chans; i++) {

		activeo[i % 2] = activeo[i % 2] << 2;

		if (base->phy_res[base->num_phy_chans - i - 1].allocated_src
		    == D40_ALLOC_PHY) {
			activeo[i % 2] |= 3;
			continue;
		}

		/* Enable interrupt # */
		pcmis = (pcmis << 1) | 1;

		/* Clear interrupt # */
		pcicr = (pcicr << 1) | 1;

		/* Set channel to physical mode */
		prmseo[i % 2] = prmseo[i % 2] << 2;
		prmseo[i % 2] |= 1;

	}

	writel(prmseo[1], base->virtbase + D40_DREG_PRMSE);
	writel(prmseo[0], base->virtbase + D40_DREG_PRMSO);
	writel(activeo[1], base->virtbase + D40_DREG_ACTIVE);
	writel(activeo[0], base->virtbase + D40_DREG_ACTIVO);

	/* Write which interrupt to enable */
	writel(pcmis, base->virtbase + D40_DREG_PCMIS);

	/* Write which interrupt to clear */
	writel(pcicr, base->virtbase + D40_DREG_PCICR);

}

static int __init d40_lcla_allocate(struct d40_base *base)
{
	unsigned long *page_list;
	int i;
	int j;
	int ret = 0;

	/*
	 * This is somewhat ugly. We need 8192 bytes that are 18 bit aligned,
	 * To full fill this hardware requirement without wasting 256 kb
	 * we allocate pages until we get an aligned one.
	 */
	page_list = kmalloc(sizeof(unsigned long) * MAX_LCLA_ALLOC_ATTEMPTS,
			    GFP_KERNEL);

	if (!page_list) {
		ret = -ENOMEM;
		goto failure;
	}

	/* Calculating how many pages that are required */
	base->lcla_pool.pages = SZ_1K * base->num_phy_chans / PAGE_SIZE;

	for (i = 0; i < MAX_LCLA_ALLOC_ATTEMPTS; i++) {
		page_list[i] = __get_free_pages(GFP_KERNEL,
						base->lcla_pool.pages);
		if (!page_list[i]) {

			dev_err(base->dev,
				"[%s] Failed to allocate %d pages.\n",
				__func__, base->lcla_pool.pages);

			for (j = 0; j < i; j++)
				free_pages(page_list[j], base->lcla_pool.pages);
			goto failure;
		}

		if ((virt_to_phys((void *)page_list[i]) &
		     (LCLA_ALIGNMENT - 1)) == 0)
			break;
	}

	for (j = 0; j < i; j++)
		free_pages(page_list[j], base->lcla_pool.pages);

	if (i < MAX_LCLA_ALLOC_ATTEMPTS) {
		base->lcla_pool.base = (void *)page_list[i];
	} else {
		/*
		 * After many attempts, no succees with finding the correct
		 * alignment try with allocating a big buffer.
		 */
		dev_warn(base->dev,
			 "[%s] Failed to get %d pages @ 18 bit align.\n",
			 __func__, base->lcla_pool.pages);
		base->lcla_pool.base_unaligned = kmalloc(SZ_1K *
							 base->num_phy_chans +
							 LCLA_ALIGNMENT,
							 GFP_KERNEL);
		if (!base->lcla_pool.base_unaligned) {
			ret = -ENOMEM;
			goto failure;
		}

		base->lcla_pool.base = PTR_ALIGN(base->lcla_pool.base_unaligned,
						 LCLA_ALIGNMENT);
	}

	writel(virt_to_phys(base->lcla_pool.base),
	       base->virtbase + D40_DREG_LCLA);
failure:
	kfree(page_list);
	return ret;
}

static int __init d40_probe(struct platform_device *pdev)
{
	int err;
	int ret = -ENOENT;
	struct d40_base *base;
	struct resource *res = NULL;
	int num_reserved_chans;
	u32 val;
	unsigned long flags;

	base = d40_hw_detect_init(pdev);

	if (!base)
		goto failure;

	num_reserved_chans = d40_phy_res_init(base);

	platform_set_drvdata(pdev, base);

	spin_lock_init(&base->interrupt_lock);
	spin_lock_init(&base->execmd_lock);
	spin_lock_init(&base->usage_lock);

	/* Get IO for logical channel parameter address */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "lcpa");
	if (!res) {
		ret = -ENOENT;
		dev_err(&pdev->dev,
			"[%s] No \"lcpa\" memory resource\n",
			__func__);
		goto failure;
	}
	base->lcpa_size = resource_size(res);
	base->phy_lcpa = res->start;

	if (request_mem_region(res->start, resource_size(res),
			       D40_NAME " I/O lcpa") == NULL) {
		ret = -EBUSY;
		dev_err(&pdev->dev,
			"[%s] Failed to request LCPA region 0x%x-0x%x\n",
			__func__, res->start, res->end);
		goto failure;
	}

	/* We make use of ESRAM memory for this. */
	val = readl(base->virtbase + D40_DREG_LCPA);
	if (res->start != val && val != 0) {
		dev_warn(&pdev->dev,
			 "[%s] Mismatch LCPA dma 0x%x, def 0x%x\n",
			 __func__, val, res->start);
	} else
		writel(res->start, base->virtbase + D40_DREG_LCPA);

	base->lcpa_base = ioremap(res->start, resource_size(res));
	if (!base->lcpa_base) {
		ret = -ENOMEM;
		dev_err(&pdev->dev,
			"[%s] Failed to ioremap LCPA region\n",
			__func__);
		goto failure;
	}
	/* If lcla has to be located in ESRAM we don't need to allocate */
	if (base->plat_data->use_esram_lcla) {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
							"lcla_esram");
		if (!res) {
			ret = -ENOENT;
			dev_err(&pdev->dev,
				"[%s] No \"lcla_esram\" memory resource\n",
				__func__);
			goto failure;
		}
		base->lcla_pool.base = ioremap(res->start,
						resource_size(res));
		if (!base->lcla_pool.base) {
			ret = -ENOMEM;
			dev_err(&pdev->dev,
				"[%s] Failed to ioremap LCLA region\n",
				__func__);
			goto failure;
		}
		writel(res->start, base->virtbase + D40_DREG_LCLA);

	} else {
		ret = d40_lcla_allocate(base);
		if (ret) {
			dev_err(&pdev->dev,
				"[%s] Failed to allocate LCLA area\n",
				__func__);
			goto failure;
		}
	}

#ifdef CONFIG_STE_DMA40_DEBUG
	sted40_history_set_virtbase(base->virtbase,
				    base->lcpa_base,
				    base->lcpa_size,
				    base->lcla_pool.base,
				    SZ_1K * base->num_phy_chans);
#endif
	spin_lock_init(&base->lcla_pool.lock);

	base->irq = platform_get_irq(pdev, 0);

	ret = request_irq(base->irq, d40_handle_interrupt, 0, D40_NAME, base);

	if (ret) {
		dev_err(&pdev->dev, "[%s] No IRQ defined\n", __func__);
		goto failure;
	}

	pm_runtime_irq_safe(base->dev);
	pm_runtime_set_autosuspend_delay(base->dev, DMA40_AUTOSUSPEND_DELAY);
	pm_runtime_use_autosuspend(base->dev);
	pm_runtime_enable(base->dev);
	pm_runtime_resume(base->dev);

	if (base->plat_data->use_esram_lcla) {

		base->lcpa_regulator = regulator_get(base->dev, "lcla_esram");
		if (IS_ERR(base->lcpa_regulator)) {
			dev_err(&pdev->dev, "Failed to get lcpa_regulator\n");
			base->lcpa_regulator = NULL;
			goto failure;
		}

		ret = regulator_enable(base->lcpa_regulator);
		if (ret) {
			dev_err(&pdev->dev,
				"Failed to enable lcpa_regulator\n");
			regulator_put(base->lcpa_regulator);
			base->lcpa_regulator = NULL;
			goto failure;
		}
	}

	base->initialized = true;

	err = d40_dmaengine_init(base, num_reserved_chans);
	if (err)
		goto failure;

	d40_hw_init(base);

	spin_lock_irqsave(&base->usage_lock, flags);
	base->usage--;
	spin_unlock_irqrestore(&base->usage_lock, flags);
#ifdef MMC_HOST_DEBUGGING
	_base = base;
#endif
	dev_info(base->dev, "initialized\n");
	return 0;

failure:
	if (base) {
		if (base->desc_slab)
			kmem_cache_destroy(base->desc_slab);
		if (base->virtbase)
			iounmap(base->virtbase);

		if (base->lcla_pool.base && base->plat_data->use_esram_lcla) {
			iounmap(base->lcla_pool.base);
			base->lcla_pool.base = NULL;
		}

		if (!base->lcla_pool.base_unaligned && base->lcla_pool.base)
			free_pages((unsigned long)base->lcla_pool.base,
				   base->lcla_pool.pages);

		kfree(base->lcla_pool.base_unaligned);
		if (base->phy_lcpa)
			release_mem_region(base->phy_lcpa,
					   base->lcpa_size);
		if (base->phy_start)
			release_mem_region(base->phy_start,
					   base->phy_size);
		if (base->clk) {
			clk_disable(base->clk);
			clk_put(base->clk);
		}

		if (base->lcpa_regulator) {
			regulator_disable(base->lcpa_regulator);
			regulator_put(base->lcpa_regulator);
		}

		kfree(base->lcla_pool.alloc_map);
		kfree(base->lookup_log_chans);
		kfree(base->lookup_phy_chans);
		kfree(base->phy_res);
		kfree(base);
	}

	dev_err(&pdev->dev, "[%s] probe failed\n", __func__);
	return ret;
}

static struct platform_driver d40_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = D40_NAME,
		.pm = DMA40_PM_OPS,
	},
};

int __init stedma40_init(void)
{
	return platform_driver_probe(&d40_driver, d40_probe);
}
subsys_initcall(stedma40_init);
